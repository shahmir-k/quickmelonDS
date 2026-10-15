/*
    liteDS-headless - Unit 0 headless benchmark runner for liteDS-v2.

    Runs a DS ROM headlessly with the software renderer, null audio and no
    input, for a fixed number of frames, and reports timing + framebuffer
    hashes. Uses FreeBIOS + generated firmware with direct boot, so no external
    BIOS/firmware files are required.

    See docs/liteDS-v2-plan.md B.2 "Unit 0" for the spec.
*/

#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <utility>
#include <memory>
#include <optional>
#include <random>
#include <chrono>
#include <time.h>
#include <algorithm>
#ifdef __linux__
#include <sys/prctl.h>
#endif

#include "types.h"
#include "Args.h"
#include "NDS.h"
#include "SPU.h"
#include "NDSCart.h"
#include "GPU.h"
#include "GPU_Soft.h"
#ifdef LITEV_HEADLESS_GL
#include "GPU_OpenGL.h"
#include "GPU_Hybrid.h"
void* MacGLProc(const char* name);
bool MacGLCreate();
void MacGLShims();
#endif
#include "Platform.h"
#include "Savestate.h"
#include "SPI_Firmware.h"
#include "FreeBIOS.h"
#ifdef __APPLE__
extern "C" int thread_selfcounts(int type, void* buf, size_t nbytes);   // libsystem_kernel
#endif

#include "xxhash/xxhash.h"

#include "PlatformHeadless.h"
#ifdef __APPLE__
// Instructions retired / cycles of the calling thread (Apple Silicon PMU, libsystem_kernel).
extern "C" int thread_selfcounts(int type, void* buf, size_t nbytes);
#endif
// emu-thread counters (macOS: instructions/cycles; Linux: instructions stay 0, "cycles" = thread CPU ns)
static uint64_t EmuThreadInstr = 0, EmuThreadCycles = 0;
#include "LiteProfile.h"
#include "VerifyTrace.h"
#include "ARM9HLE.h"
#include "InputScript.h"
namespace melonDS { extern u64 JitReviveCount; extern u64 JitRestoreCount, JitRecompileSame[8]; extern u64 JitCompileCount, JitProtectCalls, JitProtectFaults, JitStoreRepromotions; }
#ifdef LITEV_JIT_COMPILE_STATS
#include "ARMJIT_Internal.h"
#endif
#ifdef LITEV_ACCESS_STATS
namespace melonDS { extern u64 LitevAccess[6][0x10000]; extern u64 LitevRomctrlPC[0x100000]; }
static void DumpAccessStats()
{
    static const char* kinds[6] = {"R16", "R32", "W16", "W32", "R8", "W8"};
    std::vector<std::pair<unsigned long long, std::pair<int, int>>> v;
    for (int k = 0; k < 6; k++)
        for (int a = 0; a < 0x10000; a++)
            if (melonDS::LitevAccess[k][a]) v.push_back({melonDS::LitevAccess[k][a], {k, a}});
    std::sort(v.rbegin(), v.rend());
    for (int a = 0; a < 0x100000; a++)
        if (melonDS::LitevRomctrlPC[a] > 1000) fprintf(stderr, "ROMCTRL-PC %05X %llu\n", a << 1, (unsigned long long)melonDS::LitevRomctrlPC[a]);
    for (size_t i = 0; i < v.size() && i < 400; i++)
    {
        int a = v[i].second.second;
        if (a >= 0xC000 && a < 0xD000) { fprintf(stderr, "GXCMD %02X entries %llu\n", a & 0xFF, (unsigned long long)v[i].first); continue; }
        if (a >= 0xD000 && a < 0xE000) { int step = (a & 0x3FF) << 6; fprintf(stderr, "SPUCH type %d rate ~%d Hz (timer step %d) channel-samples %llu\n", (a >> 10) & 3, step ? (int)(33513982.0 / 2 / step) : 0, step, (unsigned long long)v[i].first); continue; }
        if (a >= 0xE000 && a < 0xF000) { fprintf(stderr, "IRQ %s source %d %llu\n", v[i].second.first == 4 ? "ARM9" : "ARM7", a & 0xFF, (unsigned long long)v[i].first); continue; }
        if (a >= 0xF000) fprintf(stderr, "ACCESS %s region %02X:%X %llu\n", kinds[v[i].second.first], (a >> 4) & 0xFF, a & 0xF, (unsigned long long)v[i].first);
        else fprintf(stderr, "ACCESS %s io %08X %llu\n", kinds[v[i].second.first], a >= 0x2000 ? 0x04100000 | (a & 0xFF) : 0x04000000 | a, (unsigned long long)v[i].first);
    }
}
#endif
#ifdef LITEV_HEADLESS_LAN
#include <thread>
#include "MPInterface.h"
#include "LAN.h"
#ifdef LITEV_AMBIENT_LAN
#include "AmbientLAN.h"
#include "NetplayInput.h"
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <atomic>
#endif
#endif

using namespace melonDS;

namespace {

enum class RunMode { Benchmark, RecordTrace, VerifyTrace, VerifyInterpConverge, MPTest, Replay };

// Native DS screen dimensions; the software renderer writes 256x192 u32 per screen.
constexpr int kScreenW = 256;
constexpr int kScreenH = 192;
constexpr size_t kScreenBytes = (size_t)kScreenW * kScreenH * sizeof(u32);

struct Options
{
    std::string rom;
    std::string savestate;
    std::string dataDir = "./headless-data";
    std::string profileJson;
    int frames = 300;
    bool jit = true;
    bool fastmem = true;                // JIT fast-memory path (ignored if unsupported)
    int fbHashEvery = 0;                // 0 => only final hash
    // frame:path pairs to dump as PPM. Multiple frames in ONE run (comma-separated in a single
    // --fb-dump-ppm, or the flag repeated) so a visual A/B needs one replay, not one-per-frame.
    std::vector<std::pair<int, std::string>> fbDumps;
    std::string scriptShotDir;          // capture both screens on each reactive @log milestone
    std::string videoMp4;               // --video-mp4: every frame, through ffmpeg
    std::string checkRecording;         // --check-recording: an app recording's frames.csv
    int reloadAt = -1;                  // --reload-at: load --savestate again after this many frames

    // Unit 1 trace/verify modes.
    RunMode mode = RunMode::Benchmark;
    std::string tracePath;
    std::vector<std::string> roms;      // --mp-test: --romK = console K's ROM (default --rom)
    std::vector<std::string> mpScripts; // --mp-test: --mp-scriptK = instance K's input script (0 = host)
    int replayConsole = -1;             // --replay-console K (Hosted Netplay replica)
    std::string replayLog;              // --replay-log <dir>: the records LITEV_MP_RECORD wrote
    long long fixedRtc = liteds::kDefaultRtcEpoch;

    AudioInterpolation interp = AudioInterpolation::None;
    int frameskip = 0;                  // LITEV_AGGRESSIVE_SKIP target (0 = off)
    bool accurate3d = false;            // --soft3d accurate: SoftRenderer3D instead of the tile renderer
    std::string renderer = "soft";      // --renderer soft|hybrid|glhires (GL ones: macOS, ENABLE_OGLRENDERER)
    int glScale = 1;                    // --scale N: GL internal resolution

    std::string inputScript;            // --input-script: scripted button input
    std::string lanJoin;                // --lan-join <host>: join a LAN session (LITEV_HEADLESS_LAN)
    int lanHost = 0;                    // --lan-host <maxplayers>: host a LAN session instead
    int macXor = 0;                     // --mac-xor <n>: distinct firmware MAC per headless instance
    std::string lanName = "Headless";
    std::string ambient;                // --ambient <name>: Just Play local wireless (LITEV_AMBIENT_LAN)
    bool paced = false;                 // --paced: real time (60 fps) as with a link, without one (A/B baselines)   // --lan-name: player name shown in the host's lobby

    // --bench-window <start>:<end>: measure avg FPS ONLY over frames [start,end]
    // (inclusive), while the total run still executes all --frames frames. Lets a
    // benchmark isolate a steady-state gameplay window from the boot/menu ramp.
    int benchWindowStart = -1;          // -1 => no window (overall FPS only)
    int benchWindowEnd   = -1;

    // --dump-savestate <frame>:<path>: after running frame <frame>, write a full
    // core savestate to <path>. Lets a scripted menu run bake an in-race start
    // state once, so benchmark runs can load it and measure gameplay immediately
    // (script-once / savestate-many). Savestates embed copyrighted RAM contents,
    // so they are LOCAL-ONLY (gitignored) and regenerated from the input script.
    int dumpSavestateFrame = -1;
    std::string dumpSavestatePath;
};

[[noreturn]] void Usage(const char* argv0, int code)
{
    fprintf(stderr,
        "liteDS-headless - headless melonDS benchmark runner (liteDS-v2 Unit 0)\n"
        "Usage: %s --rom <path> [options]\n"
        "  --rom <path>              DS ROM to run (required)\n"
        "  --romK <path>             --mp-test / --replay-console: console K's ROM (default --rom).\n"
        "                            Netplay/Hosted: the ROMs this process has; each console boots\n"
        "                            its player's ROM, found among them by hash\n"
        "  --savestate <path>        load a savestate after boot (optional)\n"
        "  --dump-savestate <f>:<p>  write a core savestate to <p> after running frame f\n"
        "  --frames N                number of frames to run (default 300)\n"
        "  --mode jit|interp         execution mode (default jit)\n"
        "  --fastmem on|off          JIT fast-memory path (default on; no-op where unsupported)\n"
        "  --fb-hash-every N         print xxhash of both framebuffers every N frames\n"
        "  --fb-dump-ppm <f>:<path>[,<f>:<path>...]  dump both framebuffers at each frame as PPM\n"
        "  --script-shot-dir <dir>   dump both screens at each @log milestone (directory must exist)\n"
        "                            (multiple frames in ONE run; flag may also repeat)\n"
        "  --audio-interp <mode>     SPU interpolation: none|linear|cosine|cubic|gaussian (default none)\n"
        "  --frameskip N             skip N of every N+1 frames' rasterization (LITEV_AGGRESSIVE_SKIP build)\n"
        "  --soft3d fast|accurate    software 3D backend: tile renderer (default when built in) or SoftRenderer3D\n"
        "  --renderer soft|hybrid|glhires  renderer (GL ones: macOS build with ENABLE_OGLRENDERER; --fb-dump-ppm\n"
        "                            then writes the GL output at 256N x 384N). Env debug.litev.* = GL A/B props\n"
        "  --scale N                 GL internal resolution (default 1)\n"
        "  --bench-window <s>:<e>    report avg FPS over frames [s,e] inclusive only (still runs all frames)\n"
        "  --profile-json <path>     write per-run totals as JSON\n"
        "  --data-dir <path>         local firmware/save directory (default ./headless-data)\n"
        "  --video-mp4 <path>        write every frame (top over bottom) to an MP4 (needs ffmpeg)\n"
        "  --check-recording <csv>   compare the state hashes with an app recording's frames.csv\n"
        "  --reload-at <n>           after n frames, load --savestate again and restart the frame count\n"
        "                            (finds emulator state a savestate does not hold)\n"
        "  --fixed-rtc <unix-ts>     fixed RTC epoch for determinism (default 946684800)\n"
        "  --lan-join <host>         join the LAN multiplayer session at <host> (e.g. a SereneDS\n"
        "                            device); runs paced at 60 fps (LITEV_HEADLESS_LAN build)\n"
        "  --lan-host <maxplayers>   host a LAN session instead (others --lan-join this machine)\n"
        "  --lan-name <name>         player name for --lan-join/--lan-host (default Headless)\n"
        "  --ambient <name>          Just Play local wireless as <name> (LITEV_AMBIENT_LAN build): no lobby;\n"
        "                            env LITEV_AMBIENT_ID=<hex> device id (MAC from it), LITEV_AMBIENT_PEERS=\n"
        "                            ip:port,... unicast presence targets (as NSD), LITEV_AMBIENT_UPORT=<port>\n"
        "                            this one's presence port, LITEV_AMBIENT_PPORT=<port> the broadcast port\n"
        "                            (default 7065; tests), LITEV_AMBIENT_VIS=0|1|2 (everyone / while its\n"
        "                            wireless is on / hidden); prints the nearby table + counters every 120 frames\n"
        "  --paced                   run in real time (60 fps) as --lan-* / --ambient do, with no link (A/B baselines)\n"
        "  --mac-xor <n>             XOR the generated firmware's last MAC byte with n, so several\n"
        "                            headless instances are distinct wireless players\n"
        "  --input-script <path>     scripted button input: lines '<frame> <keys>'\n"
        "                            keys = comma list (A,B,SELECT,START,RIGHT,LEFT,UP,\n"
        "                            DOWN,R,L,X,Y), NONE, or a 0x hex pressed-mask; held\n"
        "                            level from that frame on (applies to all run modes)\n"
        "\n"
        "  Unit 1 oracle modes (mutually exclusive; run --frames frames):\n"
        "  --record-trace <path>     record a per-frame binary state trace to <path>\n"
        "  --verify-trace <path>     replay and compare against a golden trace <path>\n"
        "  --verify-interp-converge  run JIT + interp side by side, report convergence\n"
        "  --replay-console K        Hosted Netplay replica (LITEV_HOSTED_NETPLAY build): run only\n"
        "                            --mp-test's console K, its link fed by the server's records:\n"
        "  --replay-log <dir>        ... from the files LITEV_MP_RECORD=<dir> wrote (else from a\n"
        "                            server over the network: LITEV_HOSTED=host=IP:PORT,port=N)\n",
        argv0);
    exit(code);
}

bool ParseArgs(int argc, char** argv, Options& o)
{
    for (int i = 1; i < argc; i++)
    {
        std::string a = argv[i];
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) { fprintf(stderr, "error: %s requires an argument\n", name); Usage(argv[0], 2); }
            return argv[++i];
        };

        if (a == "--rom") o.rom = next("--rom");
        else if (a == "--savestate") o.savestate = next("--savestate");
        else if (a == "--dump-savestate")
        {
            std::string spec = next("--dump-savestate");
            auto colon = spec.find(':');
            if (colon == std::string::npos) { fprintf(stderr, "error: --dump-savestate needs <frame>:<path>\n"); return false; }
            o.dumpSavestateFrame = std::atoi(spec.substr(0, colon).c_str());
            o.dumpSavestatePath = spec.substr(colon + 1);
        }
        else if (a == "--frames") o.frames = std::atoi(next("--frames").c_str());
        else if (a == "--mode")
        {
            std::string m = next("--mode");
            if (m == "jit") o.jit = true;
            else if (m == "interp") o.jit = false;
            else { fprintf(stderr, "error: --mode must be jit or interp\n"); return false; }
        }
        else if (a == "--fastmem")
        {
            std::string m = next("--fastmem");
            if (m == "on" || m == "1" || m == "true") o.fastmem = true;
            else if (m == "off" || m == "0" || m == "false") o.fastmem = false;
            else { fprintf(stderr, "error: --fastmem must be on or off\n"); return false; }
        }
        else if (a == "--fb-hash-every") o.fbHashEvery = std::atoi(next("--fb-hash-every").c_str());
        else if (a == "--video-mp4") o.videoMp4 = next("--video-mp4");
        else if (a == "--check-recording") o.checkRecording = next("--check-recording");
        else if (a == "--reload-at") o.reloadAt = std::atoi(next("--reload-at").c_str());
        else if (a == "--script-shot-dir") o.scriptShotDir = next("--script-shot-dir");
        else if (a == "--fb-dump-ppm")
        {
            // Accept one or more <frame>:<path> pairs, comma-separated, in a single flag (the flag
            // may also be repeated — pairs accumulate). Lets one replay dump every A/B sample frame.
            std::string spec = next("--fb-dump-ppm");
            size_t pos = 0;
            while (pos < spec.size())
            {
                size_t comma = spec.find(',', pos);
                std::string one = spec.substr(pos, comma == std::string::npos ? std::string::npos : comma - pos);
                size_t colon = one.find(':');
                if (colon == std::string::npos) { fprintf(stderr, "error: --fb-dump-ppm needs <frame>:<path>[,<frame>:<path>...]\n"); return false; }
                o.fbDumps.emplace_back(std::atoi(one.substr(0, colon).c_str()), one.substr(colon + 1));
                if (comma == std::string::npos) break;
                pos = comma + 1;
            }
        }
        else if (a == "--audio-interp")
        {
            std::string m = next("--audio-interp");
            if (m == "none") o.interp = AudioInterpolation::None;
            else if (m == "linear") o.interp = AudioInterpolation::Linear;
            else if (m == "cosine") o.interp = AudioInterpolation::Cosine;
            else if (m == "cubic") o.interp = AudioInterpolation::Cubic;
            else if (m == "gaussian") o.interp = AudioInterpolation::SNESGaussian;
            else { fprintf(stderr, "error: --audio-interp must be none|linear|cosine|cubic|gaussian\n"); return false; }
        }
        else if (a == "--frameskip") o.frameskip = std::atoi(next("--frameskip").c_str());
        else if (a == "--renderer") o.renderer = next("--renderer");
        else if (a == "--scale") o.glScale = std::max(1, std::atoi(next("--scale").c_str()));
        else if (a == "--soft3d")
        {
            std::string m = next("--soft3d");
            if (m == "accurate") o.accurate3d = true;
            else if (m == "fast") o.accurate3d = false;
            else { fprintf(stderr, "error: --soft3d must be fast or accurate\n"); Usage(argv[0], 2); }
        }
        else if (a == "--profile-json") o.profileJson = next("--profile-json");
        else if (a == "--data-dir") o.dataDir = next("--data-dir");
        else if (a == "--fixed-rtc") o.fixedRtc = std::atoll(next("--fixed-rtc").c_str());
        else if (a == "--input-script") o.inputScript = next("--input-script");
        else if (a == "--lan-join") o.lanJoin = next("--lan-join");
        else if (a == "--lan-host") o.lanHost = std::atoi(next("--lan-host").c_str());
        else if (a == "--mac-xor") o.macXor = std::atoi(next("--mac-xor").c_str());
        else if (a == "--lan-name") o.lanName = next("--lan-name");
        else if (a == "--ambient") o.ambient = next("--ambient");
        else if (a == "--paced") o.paced = true;
        else if (a == "--bench-window")
        {
            std::string spec = next("--bench-window");
            auto colon = spec.find(':');
            if (colon == std::string::npos) { fprintf(stderr, "error: --bench-window needs <start>:<end>\n"); return false; }
            o.benchWindowStart = std::atoi(spec.substr(0, colon).c_str());
            o.benchWindowEnd   = std::atoi(spec.substr(colon + 1).c_str());
        }
        else if (a == "--record-trace") { o.mode = RunMode::RecordTrace; o.tracePath = next("--record-trace"); }
        else if (a == "--verify-trace") { o.mode = RunMode::VerifyTrace; o.tracePath = next("--verify-trace"); }
        else if (a == "--verify-interp-converge") o.mode = RunMode::VerifyInterpConverge;
        else if (a == "--mp-test") o.mode = RunMode::MPTest;
        else if (a == "--replay-console") { o.mode = RunMode::Replay; o.replayConsole = std::atoi(next("--replay-console").c_str()); }
        else if (a == "--replay-log") o.replayLog = next("--replay-log");
        else if (a.rfind("--rom", 0) == 0 && a.size() > 5 && isdigit((unsigned char)a[5]))
        {
            size_t k = std::stoul(a.substr(5));
            if (k >= 16) { fprintf(stderr, "error: %s: at most 16 instances\n", a.c_str()); return false; }
            if (o.roms.size() <= k) o.roms.resize(k + 1);
            o.roms[k] = next(a.c_str());
        }
        else if (a.rfind("--mp-script", 0) == 0 && a.size() > 11 && isdigit((unsigned char)a[11]))
        {
            size_t k = std::stoul(a.substr(11));
            if (k >= 16) { fprintf(stderr, "error: %s: at most 16 instances\n", a.c_str()); return false; }
            if (o.mpScripts.size() <= k) o.mpScripts.resize(k + 1);
            o.mpScripts[k] = next(a.c_str());
        }
        else if (a == "--help" || a == "-h") Usage(argv[0], 0);
        else { fprintf(stderr, "error: unknown argument '%s'\n", a.c_str()); return false; }
    }

    if (o.rom.empty()) { fprintf(stderr, "error: --rom is required\n"); return false; }
    if (o.frames <= 0) { fprintf(stderr, "error: --frames must be positive\n"); return false; }
    if (o.benchWindowStart >= 0 || o.benchWindowEnd >= 0)
    {
        if (o.benchWindowStart < 0 || o.benchWindowEnd < 0
            || o.benchWindowStart > o.benchWindowEnd
            || o.benchWindowEnd >= o.frames)
        {
            fprintf(stderr, "error: --bench-window needs 0 <= start <= end < frames (got %d:%d, frames=%d)\n",
                    o.benchWindowStart, o.benchWindowEnd, o.frames);
            return false;
        }
    }
    return true;
}

std::unique_ptr<u8[]> ReadFile(const std::string& path, u32& lenOut)
{
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) return nullptr;
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (len <= 0) { fclose(f); return nullptr; }
    auto buf = std::make_unique<u8[]>((size_t)len);
    size_t rd = fread(buf.get(), 1, (size_t)len, f);
    fclose(f);
    if (rd != (size_t)len) return nullptr;
    lenOut = (u32)len;
    return buf;
}

// Dump both framebuffers as a single side-by-side binary PPM (P6, 24-bit).
// The software renderer stores pixels as BGRA/ABGR u32; we extract RGB.
#ifdef LITEV_HEADLESS_GL
// The GL renderers' output: a 2-layer RGBA array texture (0 = top, 1 = bottom), 256N x 192N.
// Dumped at full scale, top over bottom (a hybrid GetFramebuffers merges into it first).
bool DumpGLArray(const std::string& path, GLuint tex, int n)
{
    const int W = 256 * n, H = 192 * n;
    std::vector<u8> px((size_t)W * H * 2 * 4);
    glFinish();
    glBindTexture(GL_TEXTURE_2D_ARRAY, tex);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glGetTexImage(GL_TEXTURE_2D_ARRAY, 0, GL_RGBA, GL_UNSIGNED_BYTE, px.data());
    FILE* f = fopen(path.c_str(), "wb");
    if (!f) return false;
    fprintf(f, "P6\n%d %d\n255\n", W, H * 2);
    for (size_t i = 0; i < px.size(); i += 4) fwrite(&px[i], 1, 3, f);
    fclose(f);
    return true;
}
#endif

bool DumpPPM(const std::string& path, const u32* top, const u32* bottom)
{
    FILE* f = fopen(path.c_str(), "wb");
    if (!f) return false;
    const int W = kScreenW;
    const int H = kScreenH * 2; // stacked vertically
    fprintf(f, "P6\n%d %d\n255\n", W, H);
    std::vector<u8> row(W * 3);
    auto writeScreen = [&](const u32* src) {
        for (int y = 0; y < kScreenH; y++)
        {
            for (int x = 0; x < W; x++)
            {
                u32 px = src[y * W + x];
                // the software framebuffer is 0xFFRRGGBB (BGRA in memory; GPU_Soft.cpp ConvertToBGRA)
                row[x*3+0] = (u8)((px >> 16) & 0xFF); // R
                row[x*3+1] = (u8)((px >> 8) & 0xFF);  // G
                row[x*3+2] = (u8)(px & 0xFF);         // B
            }
            fwrite(row.data(), 1, row.size(), f);
        }
    };
    writeScreen(top);
    writeScreen(bottom);
    fclose(f);
    return true;
}

// Write a full core savestate to `path` using the same in-memory Savestate API
// as the Qt frontend (default 32 MB save buffer -> DoSavestate -> flush bytes).
bool SaveSavestate(NDS& nds, const std::string& path, std::vector<u8>* captured = nullptr, bool coldCaches = false)
{
    Savestate state; // default ctor: save mode, DEFAULT_SIZE buffer
    if (state.Error) { fprintf(stderr, "error: savestate alloc failed\n"); return false; }
    state.ColdCaches = coldCaches;
    if (!nds.DoSavestate(&state) || state.Error)
    {
        fprintf(stderr, "error: DoSavestate failed while dumping\n");
        return false;
    }
    if (captured)
    {
        const u8* bytes = static_cast<const u8*>(state.Buffer());
        captured->assign(bytes, bytes + state.Length());
    }
    if (path.empty())
    {
        fprintf(stderr, "captured savestate (%u bytes) in memory\n", state.Length());
        return true;
    }
    FILE* f = fopen(path.c_str(), "wb");
    if (!f) { fprintf(stderr, "error: cannot open savestate '%s' for write\n", path.c_str()); return false; }
    size_t wr = fwrite(state.Buffer(), 1, state.Length(), f);
    const int writeError = wr == state.Length() ? 0 : errno;
    const int closeResult = fclose(f);
    const int closeError = closeResult ? errno : 0;
    if (wr != state.Length() || closeResult != 0)
    {
        const std::string writeText = writeError ? strerror(writeError) : "ok";
        const std::string closeText = closeError ? strerror(closeError) : "ok";
        fprintf(stderr, "error: savestate write failed '%s' (%zu/%u bytes, write=%s, close=%s)\n",
                path.c_str(), wr, state.Length(), writeText.c_str(), closeText.c_str());
        return false;
    }
    fprintf(stderr, "dumped savestate (%u bytes) to %s\n", state.Length(), path.c_str());
    return true;
}

bool LoadSavestate(NDS& nds, const std::string& path)
{
    u32 len = 0;
    auto buf = ReadFile(path, len);
    if (!buf) { fprintf(stderr, "error: cannot read savestate '%s'\n", path.c_str()); return false; }
    Savestate state(buf.get(), len, false);
    if (state.Error) { fprintf(stderr, "error: savestate parse error\n"); return false; }
    if (!nds.DoSavestate(&state) || state.Error)
    {
        fprintf(stderr, "error: failed to load savestate into emulator\n");
        return false;
    }
    // the hardware divider/sqrt registers as loaded (state 14.1 carries them; for re-creating
    // them in emulators whose savestates don't, e.g. stock melonDS)
    if (getenv("LITEV_PRINT_DIV"))
        printf("DIVREGS %04X %08X %08X %08X %08X %04X %08X %08X\n", nds.ARM9IORead16(0x04000280),
               nds.ARM9IORead32(0x04000290), nds.ARM9IORead32(0x04000294),
               nds.ARM9IORead32(0x04000298), nds.ARM9IORead32(0x0400029C),
               nds.ARM9IORead16(0x040002B0), nds.ARM9IORead32(0x040002B8), nds.ARM9IORead32(0x040002BC));
    return true;
}

} // namespace

// A live handoff checkpoint loaded into a fresh scratch console (its own userdata and save files)
static bool PrepareScratch(const NetplayLiveCheckpoint& candidate, const std::string& romPath, bool jit,
                           std::unique_ptr<HeadlessHost::InstanceUserData>& user,
                           std::unique_ptr<NDS>& instance, std::string& reason)
{
    auto scratch = std::make_unique<HeadlessHost::InstanceUserData>();
    scratch->savePrefix = candidate.Player == 0 ? "handoff-scratch-host" : candidate.Player == 1 ? "handoff-scratch-client" :
                          "handoff-scratch-p" + std::to_string(candidate.Player);
    u32 romLength = 0;
    auto romData = ReadFile(romPath, romLength);
    if (!romData) { reason = "scratch replica ROM unavailable"; return false; }
    auto cart = NDSCart::ParseROM(std::move(romData), romLength, scratch.get(), std::nullopt);
    if (!cart) { reason = "scratch replica ROM parse failed"; return false; }
    NDSArgs replicaArgs;
    replicaArgs.Firmware = Firmware(candidate.Firmware.data(), (u32)candidate.Firmware.size());
#ifdef JIT_ENABLED
    replicaArgs.JIT = jit ? std::optional<JITArgs>(JITArgs{}) : std::nullopt;
#endif
    auto replica = std::make_unique<NDS>(std::move(replicaArgs), scratch.get());
    replica->SetRenderer(std::make_unique<SoftRenderer>(*replica));
    replica->SetNDSCart(std::move(cart));
    replica->Reset();
    replica->SetupDirectBoot("headless.nds");
    replica->Start();
    Savestate parsed((void*)candidate.State.data(), (u32)candidate.State.size(), false);
    if (parsed.Error || !replica->DoSavestate(&parsed) || parsed.Error)
    { reason = "scratch replica savestate load failed"; return false; }
    const Firmware& loaded = replica->GetFirmware();
    if (loaded.Length() != candidate.Firmware.size() ||
        memcmp(loaded.Buffer(), candidate.Firmware.data(), loaded.Length()) != 0)
    { reason = "scratch replica firmware mismatch"; return false; }
    user = std::move(scratch);
    instance = std::move(replica);
    return true;
}

#ifdef LITEV_AMBIENT_LAN
// ---- N-player live handoff (LITEV_AMBIENT_CUT_MULTI=N), test harness --------------------------------
// One process per member of an N-player Ambient LAN session. The member that offers (role host: the
// game's MP parent, linked to every other member) proposes once it is linked to N-1 members and the
// frame reaches LITEV_AMBIENT_CUT_MIN_FRAME; the others (role client) accept. Every member publishes its
// ROM; the offerer numbers the members (roster: itself 0, then by device id) and sends the roster to
// each. Every pair of members opens a live link (TCP, the lower player listens on
// LITEV_AMBIENT_CUT_XFER_PORT + 16 * lower + higher), each sends its state as the baseline. Then the
// cut: the parent freezes first when idle, every child runs on to its own idle cut, every member's
// fence on every link of its group completes, each member's checkpoint goes to every other (a delta
// against its baseline), each member loads the N-1 others into scratch replicas (vote per link), runs
// the N-console 2-frame proof, and every pair compares start manifests (all N checkpoints + proof).
// Commit = the frame-zero proof of every member's first input (UDP LITEV_AMBIENT_CUT_NP_PORT + 50 + p).
// Netplay then runs every console here (this one live, the others from their checkpoints) on UDP
// LITEV_AMBIENT_CUT_NP_PORT + player, delay 3, plus LITEV_AMBIENT_CUT_NP_EXTRA (e.g. loss=3,latency=10)
// for LITEV_AMBIENT_CUT_SWAP_TRIAL_FRAMES frames. Any failure: every member aborts its fence (an abort
// on any link reaches the whole group through the parent) and rolls back to its frozen Ambient source.
// Partial exits (LITEV_NP_QUIT_AT, LITEV_NP_LIVE_EXIT; VerifyTrace MPTest): the device whose own console
// leaves (or is left alone) plays on in Ambient with that very console: its inputs already scheduled,
// then LITEV_AMBIENT_CUT_TRIAL_SCRIPT live, for LITEV_AMBIENT_CUT_EXIT_FRAMES frames.
// LITEV_AMBIENT_CUT_PEER_ROMS: the ROM library (|-separated) where the other members' ROMs are found.
struct MultiCut
{
    int N = 0;
    bool Host = false;
    int MinFrame = 0, Trial = 0;
    u16 Xfer = 0, NpPort = 0;
    std::string Dir;
    std::vector<std::string> Library;
    NetplayRom OwnRom;
    u64 Epoch = 0;
    std::array<u8, 16> Token{};
    bool Offered = false, Accepted = false, Done = false, Failed = false;
    int Me = -1;
    std::vector<AmbientLAN::HandoffMember> Roster;
    std::vector<std::string> RomPaths;      // by player
    std::vector<std::unique_ptr<NetplayLiveLink>> Links;   // by player (none for Me)
    std::chrono::steady_clock::time_point BaselineAt, FreezeAt;
    std::vector<u64> CutClocks;             // every console's raw clock at the cut (the link clock origins)
    long Ms() const { return (long)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - FreezeAt).count(); }
};

static std::string Ipv4Text(u32 ip)
{
    char b[INET_ADDRSTRLEN] = {};
    in_addr a{}; a.s_addr = ip;
    inet_ntop(AF_INET, &a, b, sizeof(b));
    return b;
}

// one frame of the N-player cut harness: -1 = carry on (in Ambient), else the process's exit code
static int MultiCutFrame(MultiCut& mc, std::unique_ptr<NDS>& nds, AmbientLAN* ambient, const Options& opt, int frame)
{
    const char* role = mc.Host ? "host" : "client";
    if (mc.Done || mc.Failed) return -1;
    auto fail = [&](const std::string& why) {
        printf("CUTMULTI %s abort f%d t=%ldms: %s\n", role, frame, mc.FreezeAt.time_since_epoch().count() ? mc.Ms() : 0L, why.c_str());
        fflush(stdout);
        ambient->AbortHandoffFence();
        ambient->CancelHandoffOffer();
        mc.Links.clear();
        mc.Failed = true;
        nds->JIT.JitEnableExecute();
        return -1;
    };
    // the offer
    if (!mc.Offered && !mc.Accepted)
    {
        if (mc.Host)
        {
            if (frame < mc.MinFrame || ambient->HandoffGroupSize() != mc.N - 1) return -1;
            static const u64 epochBase = 0x4d554c0000000000ull | ((u64)getpid() << 16) | ((u64)time(nullptr) & 0xFF00);
            mc.Epoch = epochBase + 1;
            std::random_device random;
            for (u8& b : mc.Token) b = (u8)random();
            if (!ambient->ProposeHandoff(mc.Epoch, mc.Token, AmbientLAN::HandoffPlainNetplay)) return -1;
            mc.Offered = true;
            printf("CUTMULTI host offered epoch=%llx to %d members f%d\n", (unsigned long long)mc.Epoch, mc.N - 1, frame);
        }
        else
        {
            u64 epoch = 0; AmbientLAN::HandoffMode mode;
            if (!ambient->IncomingHandoffOffer(epoch, mode) || mode != AmbientLAN::HandoffPlainNetplay ||
                !ambient->RespondToHandoffOffer(epoch, true)) return -1;
            mc.Accepted = true;
            mc.Epoch = epoch;
            printf("CUTMULTI client accepted epoch=%llx f%d\n", (unsigned long long)epoch, frame);
        }
        fflush(stdout);
        return -1;
    }
    AmbientLAN::HandoffOfferStatus status{};
    if (!ambient->GetHandoffOfferStatus(status))
    {
        // the offer went away (declined, cancelled, a member left) before the cut
        return fail("offer closed before the cut");
    }
    if (!status.Accepted) return -1;
    // ROMs, roster, links
    if (mc.Roster.empty())
    {
        if (!ambient->PublishHandoffRom(mc.OwnRom)) return fail("own ROM manifest not sent");
        std::vector<AmbientLAN::HandoffMember> roster;
        int me = -1;
        if (!ambient->GetHandoffRoster(roster, me)) return -1;
        if ((int)roster.size() != mc.N) return fail("roster of " + std::to_string(roster.size()) + " members, expected " + std::to_string(mc.N));
        mc.Roster = roster;
        mc.Me = me;
        mc.RomPaths.assign(mc.N, "");
        mc.Links.clear();
        mc.Links.resize(mc.N);
        printf("CUTMULTI %s roster: me=%d", role, me);
        for (auto& m : roster) printf(" [p%d %016llx %s %.4s]", m.Player, (unsigned long long)m.ID, m.IPv4 ? Ipv4Text(m.IPv4).c_str() : "self", m.Rom.GameCode);
        printf("\n");
        for (auto& m : roster)
        {
            if (m.Player == me) { if (m.Rom != mc.OwnRom) return fail("own roster ROM differs"); mc.RomPaths[me] = opt.rom; continue; }
            mc.RomPaths[m.Player] = NetplayFindRom(m.Rom, mc.Library);
            if (mc.RomPaths[m.Player].empty()) return fail("no local copy of player " + std::to_string(m.Player) + "'s ROM " + m.Rom.Name());
            const int lo = std::min(me, m.Player), hi = std::max(me, m.Player);
            mc.Links[m.Player] = std::make_unique<NetplayLiveLink>(me, m.Player, (u16)(mc.Xfer + 16 * lo + hi), Ipv4Text(m.IPv4));
            std::vector<u8> base;
            if (SaveSavestate(*nds, "", &base)) mc.Links[m.Player]->SendBaseline(std::move(base));
        }
        mc.BaselineAt = std::chrono::steady_clock::now();
        printf("CUTMULTI %s exact ROMs of all %d members found; %d live links opening\n", role, mc.N, mc.N - 1);
        fflush(stdout);
        return -1;
    }
    // a member aborted (its fence, a vote, a lost link) before this one froze: roll back too
    if (ambient->HandoffFenceAborted() && ambient->GetHandoffFenceStats().Epoch) return fail("a member aborted the cut before this console froze");
    if (std::chrono::steady_clock::now() - mc.BaselineAt > std::chrono::seconds(40)) return fail("no cut within 40 s of the roster");
    // the cut: the parent (not an MP client) freezes first when idle; a child once a fence reached it
    bool baselines = std::chrono::steady_clock::now() - mc.BaselineAt > std::chrono::seconds(20);
    if (!baselines) { baselines = true; for (auto& l : mc.Links) if (l && !l->BaselineAcked()) baselines = false; }
    const bool mayStart = ambient->HasPeerHandoffFence() || !nds->Wifi.IsMPClientMode();
    if ((frame % 120) == 0)
    {
        printf("CUTMULTI %s waiting f%d busy=%02x queued=%zu peerFence=%d baselines=%d\n", role, frame, nds->Wifi.HandoffBusyMask(),
               ambient->HandoffQueuedFrames(), ambient->HasPeerHandoffFence(), baselines);
        fflush(stdout);
    }
    if (!mayStart || !(baselines || ambient->HasPeerHandoffFence()) || nds->Wifi.HandoffBusyMask() || ambient->HandoffQueuedFrames()) return -1;
    mc.FreezeAt = std::chrono::steady_clock::now();
    const u64 epoch = ambient->HasPeerHandoffFence() ? ambient->PeerHandoffEpoch() : mc.Epoch;
    printf("CUTMULTI %s holding f%d clock=%llu epoch=%llx\n", role, frame, (unsigned long long)nds->GetSysTimestamp(), (unsigned long long)epoch);
    fflush(stdout);
    if (!ambient->StartHandoffFence(epoch, mc.Host ? mc.Token : std::array<u8, 16>{}))
        return fail(std::string("cannot start the fence: ") + ambient->FenceStartFailure);
    // this console is final from its freeze: the checkpoint goes to every member now
    NetplayLiveCheckpoint local;
    local.Epoch = epoch; local.Build = NetplayHandoffCompat(); local.RawClock = nds->GetSysTimestamp();
    local.Frame = frame; local.Player = (u8)mc.Me; local.Rom = mc.OwnRom;
    if (!ambient->GetHandoffFenceToken(local.FenceToken)) return fail("no fence token");
    { const Firmware& fw = nds->GetFirmware(); local.Firmware.assign(fw.Buffer(), fw.Buffer() + fw.Length()); }
    if (!SaveSavestate(*nds, "", &local.State, true)) return fail("checkpoint capture");
    for (auto& l : mc.Links)
        if (l) { std::string e; if (!l->SendCheckpoint(local, e)) printf("CUTMULTI %s early checkpoint not sent: %s\n", role, e.c_str()); }
    // the fence on every link of the group
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!ambient->HandoffFenceComplete() && !ambient->HandoffFenceAborted() && std::chrono::steady_clock::now() < deadline)
    {
        ambient->Process();
        if (nds->Wifi.HandoffBusyMask()) return fail("MP exchange busy at the freeze");
        if (ambient->CanAcknowledgeHandoffFence())
        {
            if (const char* f = getenv("LITEV_AMBIENT_CUT_INJECT_FAIL"); f && !strcmp(f, role)) return fail("injected failure before READY");
            ambient->AcknowledgeHandoffFence();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (!ambient->HandoffFenceComplete() || ambient->HandoffFenceAborted() || ambient->HandoffQueuedFrames())
        return fail(ambient->HandoffFenceAborted() ? "fence aborted by a member or a link lost" : "fence incomplete after 10 s");
    printf("CUTMULTI %s t=%ldms fence complete on %d link(s), checkpoint %zu bytes\n", role, mc.Ms(), ambient->HandoffGroupSize(), local.State.size());
    fflush(stdout);
    // checkpoints both ways on every pair, each peer's into a scratch replica
    std::vector<NetplayLiveCheckpoint> got(mc.N);
    std::vector<std::unique_ptr<HeadlessHost::InstanceUserData>> users(mc.N);
    std::vector<std::unique_ptr<NDS>> copies(mc.N);
    std::vector<std::string> errors(mc.N);
    std::vector<char> ok(mc.N, 1);
    std::mutex prepLock;
    std::atomic<bool> cancel{false};
    std::vector<std::thread> xfer;
    std::atomic<int> remaining{mc.N - 1};
    for (int p = 0; p < mc.N; p++)
        if (mc.Links[p])
            xfer.emplace_back([&, p] {
                auto prepare = [&](const NetplayLiveCheckpoint& c, std::string& reason) {
                    if (const char* f = getenv("LITEV_HANDOFF_XFER_PREPARE_FAIL"); f && !strcmp(f, role))
                    { reason = "injected post-transfer prepare failure"; return false; }
                    std::lock_guard<std::mutex> lk(prepLock);
                    return PrepareScratch(c, mc.RomPaths[p], opt.jit, users[p], copies[p], reason);
                };
                ok[p] = mc.Links[p]->ExchangeCheckpoints(local, mc.Roster[p].Rom, prepare, cancel, got[p], errors[p]);
                remaining--;
            });
    // meanwhile this console's own proof copy
    std::string ownError;
    { std::lock_guard<std::mutex> lk(prepLock); if (!PrepareScratch(local, opt.rom, opt.jit, users[mc.Me], copies[mc.Me], ownError)) copies[mc.Me].reset(); }
    while (remaining.load() > 0)
    {
        ambient->Process();
        if (ambient->HandoffFenceAborted()) cancel = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    for (auto& t : xfer) t.join();
    nds->JIT.JitEnableExecute();
    for (int p = 0; p < mc.N; p++)
        if (mc.Links[p] && !ok[p]) return fail("checkpoint exchange with player " + std::to_string(p) + ": " + errors[p]);
    if (!copies[mc.Me]) return fail("own proof copy: " + ownError);
    for (int p = 0; p < mc.N; p++) if (p != mc.Me && !copies[p]) return fail("no replica of player " + std::to_string(p));
    got[mc.Me] = local;
    printf("CUTMULTI %s t=%ldms %d checkpoints in, replicas loaded\n", role, mc.Ms(), mc.N - 1);
    // the reversible proof: all N copies, two Lockstep frames
    std::vector<NDS*> proofConsoles(mc.N);
    std::vector<u64> clocks(mc.N), proof;
    for (int p = 0; p < mc.N; p++) { proofConsoles[p] = copies[p].get(); clocks[p] = got[p].RawClock; users[p]->instanceID = p; }
    mc.CutClocks = clocks;
    std::string proofError;
    bool proven = NetplayRunLiveProofN(proofConsoles, clocks, [&](int p) { copies[p]->RunFrame(); }, proof, proofError);
    for (int p = 0; p < mc.N && proven; p++)
    {
        Savestate st((void*)got[p].State.data(), (u32)got[p].State.size(), false);
        if (st.Error || !copies[p]->DoSavestate(&st) || st.Error || copies[p]->GetSysTimestamp() != got[p].RawClock)
        { proven = false; proofError = "a proof copy did not load its checkpoint again"; }
    }
    nds->JIT.JitEnableExecute();
    if (!proven) return fail("proof: " + proofError);
    printf("CUTMULTI %s t=%ldms proof of %d consoles:", role, mc.Ms(), mc.N);
    for (u64 h : proof) printf(" %016llx", (unsigned long long)h);
    printf("\n");
    // every pair compares its start manifests
    std::vector<const NetplayLiveCheckpoint*> all(mc.N);
    for (int p = 0; p < mc.N; p++) all[p] = &got[p];
    for (int p = 0; p < mc.N; p++)
        if (mc.Links[p]) { std::string e; if (!mc.Links[p]->SendStart(all, proof, e)) return fail("start manifest to player " + std::to_string(p) + ": " + e); }
    for (int p = 0; p < mc.N; p++)
        if (mc.Links[p])
        {
            std::string e;
            std::atomic<bool> done{false}, stop{false};
            bool r = false;
            std::thread w([&] { r = mc.Links[p]->WaitStart(all, proof, stop, e); done = true; });
            while (!done) { ambient->Process(); if (ambient->HandoffFenceAborted()) stop = true; std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
            w.join();
            if (!r || ambient->HandoffFenceAborted()) return fail("start rendezvous with player " + std::to_string(p) + ": " + (e.empty() ? "fence aborted" : e));
        }
    printf("CUTMULTI %s t=%ldms live-start agreed with all %d members\n", role, mc.Ms(), mc.N - 1);
    fflush(stdout);
    mc.Links.clear();
    // Netplay: every console, this one the live one
    std::string peers, startPeers;
    for (auto& m : mc.Roster)
        if (m.Player != mc.Me)
        {
            const std::string ip = Ipv4Text(m.IPv4);
            peers += (peers.empty() ? "" : "|") + std::to_string(m.Player) + "@" + ip + ":" + std::to_string(mc.NpPort + m.Player);
            startPeers += (startPeers.empty() ? "" : "|") + std::to_string(m.Player) + "@" + ip + ":" + std::to_string(mc.NpPort + 50 + m.Player);
        }
    const char* extra = getenv("LITEV_AMBIENT_CUT_NP_EXTRA");
    const std::string np = "player=" + std::to_string(mc.Me) + ",delay=3,port=" + std::to_string(mc.NpPort + mc.Me) + ",peers=" + peers +
                           (extra && *extra ? std::string(",") + extra : "");
    setenv("LITEV_NETPLAY", np.c_str(), 1);
    setenv("LITEV_MP_HANDOFF_PROBE", "1", 1);
    setenv("LITEV_MP_HANDOFF_STARTUP_GATE", "1", 1);
    char epochText[32], tokenText[33];
    snprintf(epochText, sizeof(epochText), "%llu", (unsigned long long)local.Epoch);
    for (int i = 0; i < 16; i++) snprintf(tokenText + i * 2, 3, "%02x", local.FenceToken[i]);
    setenv("LITEV_MP_HANDOFF_EPOCH", epochText, 1);
    setenv("LITEV_MP_HANDOFF_TOKEN", tokenText, 1);
    setenv("LITEV_MP_HANDOFF_START_PORT", std::to_string(mc.NpPort + 50 + mc.Me).c_str(), 1);
    setenv("LITEV_MP_HANDOFF_START_PEER", startPeers.c_str(), 1);
    { const auto t0 = std::chrono::duration_cast<std::chrono::microseconds>(mc.FreezeAt.time_since_epoch()).count();
      setenv("LITEV_CUT_T0_US", std::to_string(t0).c_str(), 1); }
    liteds::TraceRunConfig cfg;
    cfg.rom = opt.rom;
    cfg.roms = mc.RomPaths;
    cfg.dataDir = opt.dataDir;
    cfg.jit = opt.jit;
    cfg.fixedRtcEpoch = opt.fixedRtc;
    auto savedAmbient = MPInterface::Take();
    std::atomic<bool> keepAmbient{true};
    std::thread ambientService([&] {
        while (keepAmbient.load(std::memory_order_acquire)) { ambient->Process(); std::this_thread::sleep_for(std::chrono::milliseconds(2)); }
    });
    std::vector<liteds::PreparedMPConsole> replicas(mc.N);
    copies[mc.Me].reset(); users[mc.Me].reset();
    nds->JIT.JitEnableExecute();
    for (int p = 0; p < mc.N; p++)
    {
        auto& r = replicas[p];
        if (p == mc.Me) { r.udata = std::make_unique<HeadlessHost::InstanceUserData>(); nds->UserData = r.udata.get(); r.identity = nds.get(); r.nds = std::move(nds); }
        else { r.identity = copies[p].get(); r.udata = std::move(users[p]); r.nds = std::move(copies[p]); }
    }
    std::vector<std::string> scripts(mc.N);
    if (const char* ts = getenv("LITEV_AMBIENT_CUT_TRIAL_SCRIPT")) scripts[mc.Me] = ts;
    printf("CUTMULTI %s t=%ldms Netplay of %d players begins: %s\n", role, mc.Ms(), mc.N, np.c_str());
    fflush(stdout);
    liteds::LiveExitResult ended;
    const int trial = liteds::MPTest(cfg, mc.Trial, scripts, std::move(replicas), &ended);
    keepAmbient = false;
    ambientService.join();
    mc.Done = true;
    if (ended.EndsAfter < 0)
    {
        printf("CUTMULTI %s Netplay trial %s frames=%d (still in the session)\n", role, trial == 0 ? "pass" : "failed", mc.Trial);
        fflush(stdout);
        return trial == 0 ? 0 : 2;
    }
    // this device left the session (its console left, it was left alone, or the session ended): its own
    // console plays on in Ambient, never reloaded
    const auto returned = std::chrono::steady_clock::now();
    const int F = ended.EndsAfter;
    auto& mine = ended.Consoles[mc.Me];
    // the whole session ended: every console's state now, for the uninterrupted-session reference below
    std::vector<NetplayLiveCheckpoint> atEnd;
    if (!ended.Partial)
        for (int k = 0; k < mc.N; k++)
        {
            Savestate st;
            ended.Consoles[k].nds->DoSavestate(&st);
            NetplayLiveCheckpoint c;
            c.Player = (u8)k;
            c.State.assign((const u8*)st.Buffer(), (const u8*)st.Buffer() + st.Length());
            const Firmware& fw = ended.Consoles[k].nds->GetFirmware();
            c.Firmware.assign(fw.Buffer(), fw.Buffer() + fw.Length());
            atEnd.push_back(std::move(c));
        }
    const auto snapped = std::chrono::steady_clock::now();
    std::vector<u64> realHash;
    mine.nds->DetachMPLink();
    std::vector<liteds::PreparedMPConsole> dropped;
    for (int k = 0; k < (int)ended.Consoles.size(); k++) if (k != mc.Me) dropped.push_back(std::move(ended.Consoles[k]));
    std::thread dropper([d = std::move(dropped), l = MPInterface::Take()]() mutable { d.clear(); l.reset(); });
    mine.nds->JIT.JitEnableExecute();
    mine.udata->instanceID = 0;
    MPInterface::Set(std::move(savedAmbient), MPInterface_Ambient);
    ambient->EndCommittedHandoff();
    if (mine.nds->Wifi.IsPoweredOn()) MPInterface::Get().Begin(0);
    liteds::InputScript& live = *ended.Scripts[mc.Me];
    const auto& sched = ended.Scheduled[mc.Me];
    const int exitFrames = getenv("LITEV_AMBIENT_CUT_EXIT_FRAMES") ? atoi(getenv("LITEV_AMBIENT_CUT_EXIT_FRAMES")) : 600;
    double pauseMs = 0, worstMs = 0;
    const u64 exchBefore = ambient->GetStats().Exchanges;
    auto pace = std::chrono::steady_clock::now();
    auto last = returned;
    for (int f = F + 1; f <= F + exitFrames; f++)
    {
        const auto start = std::chrono::steady_clock::now();
        const double gap = std::chrono::duration<double, std::milli>(start - last).count();
        if (f == F + 1) pauseMs = gap - std::chrono::duration<double, std::milli>(snapped - returned).count(); else worstMs = std::max(worstMs, gap);
        const int i = f - F - 1;
        NetplayFrameInput in;
        if (i < (int)sched.size()) in = sched[i];
        else
        {
            in.Keys = live.Loaded() ? live.KeyMaskForFrame(f) : 0xFFF;
            int tx, ty;
            if (live.HasTouch() && live.TouchForFrame(f, tx, ty)) { in.TouchX = tx; in.TouchY = ty; }
        }
        mine.nds->SetKeyMask(in.Keys & 0xFFF);
        if (in.TouchX >= 0) mine.nds->TouchScreen(in.TouchX, in.TouchY); else mine.nds->ReleaseScreen();
        mine.nds->RunFrame();
        liteds::AudioTapDrain(*mine.nds, 2, f);
        if (i < (int)sched.size()) realHash.push_back(NetplayLiveProofHash(*mine.nds));
        if (live.HasProgram())
        {
            void* top = nullptr; void* bot = nullptr;
            if (mine.nds->GPU.GetFramebuffers(&top, &bot) && top && bot) live.OnFrame(f, (const u32*)top, (const u32*)bot);
            else live.OnFrame(f, nullptr, nullptr);
        }
        if (static const int every = getenv("LITEV_AMBIENT_CUT_EXIT_DUMP") ? atoi(getenv("LITEV_AMBIENT_CUT_EXIT_DUMP")) : 0; every > 0 && (f - F) % every == 0)
        {
            void* top = nullptr; void* bot = nullptr;
            if (mine.nds->GPU.GetFramebuffers(&top, &bot) && top && bot)
                DumpPPM(mc.Dir + "/exit-p" + std::to_string(mc.Me) + "-f" + std::to_string(f) + ".ppm", (const u32*)top, (const u32*)bot);
        }
        ambient->Process();
        if ((f - F) % 120 == 0) { printf("AMBIENT after exit f%d: %s\n", f, ambient->DebugLine().c_str()); fflush(stdout); }
        pace += std::chrono::microseconds(16715);
        std::this_thread::sleep_until(pace);
        last = std::chrono::steady_clock::now();
    }
    dropper.join();
    if (ended.Linger.joinable()) ended.Linger.join();
    // the whole session ended: the frames after it equal an uninterrupted session's (the consoles still in it, from their
    // end states, in-process, with the inputs already scheduled)
    int same = -1;
    size_t n = SIZE_MAX;
    std::vector<int> in;    // the consoles in the session at its end (one that left before has nothing scheduled)
    for (int k = 0; k < (int)ended.Scheduled.size(); k++) if (!ended.Scheduled[k].empty()) { in.push_back(k); n = std::min(n, ended.Scheduled[k].size()); }
    const int meIn = (int)(std::find(in.begin(), in.end(), mc.Me) - in.begin());
    if (!ended.Partial && in.size() >= 2 && meIn < (int)in.size())
    {
        const int m = (int)in.size();
        std::vector<std::unique_ptr<HeadlessHost::InstanceUserData>> ru(m);
        std::vector<std::unique_ptr<NDS>> rn(m);
        std::vector<NDS*> rp(m);
        std::vector<u64> origins(m);
        std::string err;
        bool ok = true;
        for (int i = 0; i < m && ok; i++)
        {
            const int k = in[i];
            ok = PrepareScratch(atEnd[k], mc.RomPaths[k], opt.jit, ru[i], rn[i], err);
            if (ok) { ru[i]->instanceID = i; rp[i] = rn[i].get(); origins[i] = mc.CutClocks[k]; }
        }
        std::vector<u64> refHash;
        if (ok)
        {
            auto saved = MPInterface::Take();
            NetplayRunLockstepFramesN(rp, origins, [&](int i, int f) {
                const NetplayFrameInput fin = f < (int)n ? ended.Scheduled[in[i]][f] : NetplayFrameInput{};
                rn[i]->SetKeyMask(fin.Keys & 0xFFF);
                if (fin.TouchX >= 0) rn[i]->TouchScreen(fin.TouchX, fin.TouchY); else rn[i]->ReleaseScreen();
                rn[i]->RunFrame();
            }, (int)n, refHash, err);
            MPInterface::Take().reset();
            MPInterface::Set(std::move(saved), MPInterface_Ambient);
            same = 0;
            for (size_t f = 0; f < n && f < realHash.size(); f++) same += refHash[f * m + meIn] == realHash[f];
        }
        for (int i = 0; i < m; i++) { rn[i].reset(); ru[i].reset(); }
        if (mine.nds) mine.nds->JIT.JitEnableExecute();
    }
    printf("CUTMULTI %s frames after the exit equal to an uninterrupted session: %d of %zu\n", role, same, n == SIZE_MAX ? (size_t)0 : n);
    printf("CUTMULTI %s live exit after f%d (%s): back in Ambient, pause %.1f ms (worst later frame gap %.1f ms), "
           "%zu scheduled inputs applied (frames %d..%d), then live; links=%u; Ambient exchanges after the exit: %llu; speed %d (0 = 1x)\n",
           role, F, ended.Partial ? "this device alone" : "the whole session ended", pauseMs, worstMs, sched.size(), F + 1, F + (int)sched.size(),
           ambient->GetStats().Links, (unsigned long long)(ambient->GetStats().Exchanges - exchBefore), ambient->HasLinks() ? ambient->Speed() : 0);
    fflush(stdout);
    return trial == 0 || ended.Partial ? 0 : 2;
}
#endif

int main(int argc, char** argv)
{
#ifdef LITEV_ACCESS_STATS
    atexit(DumpAccessStats);   // (every mode, also --mp-test)
#endif
    Options opt;
    if (!ParseArgs(argc, argv, opt))
        Usage(argv[0], 2);

    HeadlessHost::SetDataDir(opt.dataDir);

#ifdef LITEV_AMBIENT_LAN
    // LITEV_TEST_PROOF_STATES=state0|fw0|rom0|state1|fw1|rom1: the reversible proof on two
    // checkpoint files (a cut's host.mln/host.fw and client.mln/client.fw); prints its hashes.
    if (const char* spec = getenv("LITEV_TEST_PROOF_STATES"))
    {
        std::vector<std::string> f(1);
        for (const char* q = spec; *q; q++) if (*q == '|') f.emplace_back(); else f.back() += *q;
        if (f.size() != 6) { fprintf(stderr, "LITEV_TEST_PROOF_STATES needs 6 paths\n"); return 2; }
        std::unique_ptr<HeadlessHost::InstanceUserData> user[2];
        std::unique_ptr<NDS> console[2];
        std::array<u64, 2> clocks{};
        std::string error;
        for (int p = 0; p < 2; p++)
        {
            NetplayLiveCheckpoint c;
            c.Player = (u8)p;
            u32 len = 0;
            auto st = ReadFile(f[p * 3], len);
            if (!st) { fprintf(stderr, "cannot read %s\n", f[p * 3].c_str()); return 2; }
            c.State.assign(st.get(), st.get() + len);
            auto fw = ReadFile(f[p * 3 + 1], len);
            if (!fw) { fprintf(stderr, "cannot read %s\n", f[p * 3 + 1].c_str()); return 2; }
            c.Firmware.assign(fw.get(), fw.get() + len);
            if (!PrepareScratch(c, f[p * 3 + 2], opt.jit, user[p], console[p], error)) { fprintf(stderr, "prepare %d: %s\n", p, error.c_str()); return 2; }
            user[p]->instanceID = p;
            clocks[p] = console[p]->GetSysTimestamp();
        }
        std::array<u64, 4> hashes{};
        const bool ok = NetplayRunLiveProof({console[0].get(), console[1].get()}, clocks,
                                            [&](int p) { console[p]->RunFrame(); }, hashes, error);
        printf("PROOF %s p0=%016llx/%016llx p1=%016llx/%016llx %s\n", ok ? "ok" : "FAIL",
               (unsigned long long)hashes[0], (unsigned long long)hashes[2],
               (unsigned long long)hashes[1], (unsigned long long)hashes[3], error.c_str());
        return ok ? 0 : 2;
    }
    // LITEV_TEST_LIVE_LINK: the live handoff link on loopback: baselines, checkpoints both ways
    // as deltas, the prepare votes, the start manifests (equal, and a proof mismatch)
    if (getenv("LITEV_TEST_LIVE_LINK"))
    {
        auto fakeState = [](u8 seed, u8 change) {   // savestate-shaped: a header and two sections
            std::vector<u8> v(16 + 2 * (16 + 4096), 0);
            for (int k = 0; k < 2; k++)
            {
                u8* sec = &v[16 + k * (16 + 4096)];
                memcpy(sec, k ? "BBBB" : "AAAA", 4);
                const u32 len = 16 + 4096;
                memcpy(sec + 4, &len, 4);
                for (int i = 0; i < 4096; i++) sec[16 + i] = (u8)(seed + i * 7);
            }
            v[16 + 16 + 100] ^= change; v.back() ^= change;
            return v;
        };
        NetplayLiveCheckpoint c[2];
        for (int p = 0; p < 2; p++)
        {
            c[p].Player = (u8)p; c[p].Epoch = 42; c[p].Build = 0x123456789abcdef0ull; c[p].FenceToken[0] = 1;
            c[p].RawClock = 1000 + p; c[p].Frame = 7;
            c[p].Rom.Size = 4096 + p; c[p].Rom.Sha[0] = (u8)(p + 1); memcpy(c[p].Rom.GameCode, "TEST", 4);
            c[p].Firmware = {7, 8, (u8)p};
            c[p].State = fakeState((u8)(p * 50), 0x5A);
        }
        bool ok = NetplayLiveDelta(NetplayLiveDelta(c[0].State, fakeState(0, 0)), fakeState(0, 0)) == c[0].State;
        const u16 port = static_cast<u16>(32000 + getpid() % 10000);
        NetplayLiveLink l0(0, port, "127.0.0.1"), l1(1, port, "127.0.0.1");
        l0.SendBaseline(fakeState(0, 0));
        l1.SendBaseline(fakeState(50, 0));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::atomic<bool> stop{false};
        NetplayLiveCheckpoint got[2];
        std::string err[2];
        bool ex[2];
        std::thread t([&] { ex[1] = l1.ExchangeCheckpoints(c[1], c[0].Rom, nullptr, stop, got[1], err[1]); });
        ex[0] = l0.ExchangeCheckpoints(c[0], c[1].Rom, nullptr, stop, got[0], err[0]);
        t.join();
        ok = ok && ex[0] && ex[1] && got[0].State == c[1].State && got[1].State == c[0].State;
        if (!ok) printf("LIVE_LINK exchange: %d %d '%s' '%s'\n", ex[0], ex[1], err[0].c_str(), err[1].c_str());
        auto start = [&](bool mismatch) {
            const std::array<u64, 4> good {11, 22, 33, 44};
            std::array<u64, 4> bad = good;
            if (mismatch) bad[2]++;
            bool r[2];
            std::thread u([&] { r[1] = l1.ExchangeStart(c[1], got[1], bad, stop, err[1]); });
            r[0] = l0.ExchangeStart(c[0], got[0], good, stop, err[0]);
            u.join();
            return mismatch ? !r[0] && !r[1] && err[0].find("pre-commit frame proof") != std::string::npos
                            : r[0] && r[1];
        };
        ok = ok && start(false) && start(true);
        printf("LIVE_LINK %s\n", ok ? "PASS" : "FAIL");
        return ok ? 0 : 2;
    }
    if (getenv("LITEV_TEST_PREPARE_BRIDGE"))
    {
        NetplayLiveCheckpoint checkpoint;
        checkpoint.Epoch = 42;
        std::atomic<bool> cancel{false};
        const auto owner = std::this_thread::get_id();
        auto waitPending = [](NetplayLivePrepareBridge& bridge) {
            const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(1);
            while (!bridge.HasPending() && std::chrono::steady_clock::now() < until)
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            return bridge.HasPending();
        };
        NetplayLivePrepareBridge success;
        bool successResult = false;
        std::string successError;
        std::thread worker([&] {
            successResult = success.Request(checkpoint, successError, cancel, 1000);
        });
        const bool queued = waitPending(success);
        const bool serviced = success.Service([&](const NetplayLiveCheckpoint& request,
                                                   std::string&) {
            return &request == &checkpoint && request.Epoch == 42 &&
                   std::this_thread::get_id() == owner;
        });
        worker.join();

        NetplayLivePrepareBridge rejected;
        bool rejectResult = true;
        std::string rejectError;
        std::thread rejectedWorker([&] {
            rejectResult = rejected.Request(checkpoint, rejectError, cancel, 1000);
        });
        const bool rejectQueued = waitPending(rejected);
        const bool rejectServiced = rejected.Service([](const NetplayLiveCheckpoint&,
                                                        std::string& reason) {
            reason = "replica mismatch";
            return false;
        });
        rejectedWorker.join();

        NetplayLivePrepareBridge cancelled;
        bool cancelResult = true;
        std::string cancelError;
        std::thread cancelledWorker([&] {
            cancelResult = cancelled.Request(checkpoint, cancelError, cancel, 1000);
        });
        const bool cancelQueued = waitPending(cancelled);
        cancelled.Cancel();
        cancelledWorker.join();

        NetplayLivePrepareBridge timedOut;
        std::string timeoutError;
        const bool timeoutResult = timedOut.Request(checkpoint, timeoutError, cancel, 20);

        NetplayLivePrepareBridge cancelledDuringLoad;
        bool loadResult = true;
        std::string loadError;
        std::atomic<bool> loadCancel{false};
        std::thread loadWorker([&] {
            loadResult = cancelledDuringLoad.Request(checkpoint, loadError, loadCancel, 1000);
        });
        const bool loadQueued = waitPending(cancelledDuringLoad);
        const bool loadServiced = cancelledDuringLoad.Service([&](const NetplayLiveCheckpoint&,
                                                                   std::string&) {
            loadCancel.store(true, std::memory_order_release);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            return true;
        });
        loadWorker.join();
        const bool ok = queued && serviced && successResult && successError.empty() &&
                        rejectQueued && rejectServiced && !rejectResult &&
                        rejectError == "replica mismatch" &&
                        cancelQueued && !cancelResult && !cancelError.empty() &&
                        !timeoutResult && timeoutError == "live replica prepare timed out" &&
                        loadQueued && loadServiced && !loadResult && !loadError.empty();
        printf("PREPARE_BRIDGE %s\n", ok ? "PASS" : "FAIL");
        return ok ? 0 : 2;
    }
    if (getenv("LITEV_TEST_HANDOFF_WIRE_AUTH"))
    {
        // An old Netplay sender can reuse the same player index and target IP.
        // Its untagged input and hash must not enter a newly armed live cut.
        // Also run with host id 15: Hosted Netplay's record server uses that
        // input seat rather than its visible console seat 0.
        const int hostPlayer = getenv("LITEV_TEST_HANDOFF_WIRE_AUTH_HOSTED") ? 15 : 0;
        const int base = 22000 + (getpid() % 4000) * 5;
        const auto address = [](int port) { return "127.0.0.1:" + std::to_string(port); };
        NetplayInput host(hostPlayer, 3, base, {{1, address(base + 1)}});
        NetplayInput peer(1, 3, base + 1, {{hostPlayer, address(base)}});
        NetplayInput stale(1, 3, base + 2, {{hostPlayer, address(base)}});
        NetplayInput wrongToken(1, 3, base + 5, {{hostPlayer, address(base)}});
        std::array<u8, 16> token{};
        for (int i = 0; i < 16; i++) token[i] = (u8)(i + 1);
        std::array<u8, 16> wrong = token;
        wrong[0] ^= 1;
        if (!host.Ok() || !peer.Ok() || !stale.Ok() || !wrongToken.Ok() ||
            !host.ArmHandoffSession(42, token, base + 3, {{1, address(base + 4)}}) ||
            !peer.ArmHandoffSession(42, token, base + 4, {{hostPlayer, address(base + 3)}}) ||
            !wrongToken.ArmHandoffSession(42, wrong, base + 6, {{hostPlayer, address(base + 3)}}))
        { printf("HANDOFF_WIRE_AUTH FAIL: sockets\n"); return 2; }
        // A correctly tagged packet is still premature until the separate
        // frame-zero proof completes on this receiver.
        peer.SendHash(59, 0xBEEF);
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        u64 prematureHash = 0;
        const bool prematureRejected = !host.PeerHash(1, 59, prematureHash);
        host.SubmitLocal(0, NetplayFrameInput{0xFFE, -1, -1});
        peer.SubmitLocal(0, NetplayFrameInput{0xFFD, -1, -1});
        bool hostProof = false, peerProof = false;
        std::thread hostStart([&] { hostProof = host.WaitForHandoffPeerInput(3000); });
        std::thread peerStart([&] { peerProof = peer.WaitForHandoffPeerInput(3000); });
        hostStart.join();
        peerStart.join();
        if (!hostProof || !peerProof)
        { printf("HANDOFF_WIRE_AUTH FAIL: startup proof\n"); return 2; }

        stale.SendHash(60, 0xBAD);
        stale.SubmitLocal(1, NetplayFrameInput{0xFF7, -1, -1});
        wrongToken.SendHash(60, 0xBAD);
        wrongToken.SubmitLocal(1, NetplayFrameInput{0xFF7, -1, -1});
        std::atomic<bool> gotInput{false};
        NetplayFrameInput received;
        std::thread getter([&] {
            received = host.Get(1, 4);
            gotInput.store(true, std::memory_order_release);
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        u64 hash = 0;
        const bool staleRejected = !gotInput.load(std::memory_order_acquire) &&
                                   !host.PeerHash(1, 60, hash);
        peer.SendHash(60, 0xA55A);
        peer.SubmitLocal(1, NetplayFrameInput{0xFFB, -1, -1});
        const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!gotInput.load(std::memory_order_acquire) &&
               std::chrono::steady_clock::now() < until)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (!gotInput.load(std::memory_order_acquire)) host.Abort();
        getter.join();
        bool validHash = false;
        const auto hashUntil = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        while (!(validHash = host.PeerHash(1, 60, hash)) &&
               std::chrono::steady_clock::now() < hashUntil)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        const bool ok = prematureRejected && staleRejected && gotInput && received.Keys == 0xFFB &&
                        validHash && hash == 0xA55A;
        printf("HANDOFF_WIRE_AUTH %s\n", ok ? "PASS" : "FAIL");
        return ok ? 0 : 2;
    }
    if (getenv("LITEV_TEST_ROM_REHASH"))
    {
        // Same-size replacement: ordinary setup may reuse a hash, but a live
        // handoff must rehash the current file before accepting a peer ROM.
        char path[] = "/tmp/litev-romhash-XXXXXX";
        int fd = mkstemp(path);
        if (fd < 0) return 2;
        u8 bytes[32]{};
        memcpy(bytes, "TEST ROM    ABCD", 16);
        bytes[31] = 1;
        const bool firstWrite = write(fd, bytes, sizeof(bytes)) == sizeof(bytes);
        close(fd);
        const NetplayRom before = firstWrite ? NetplayDescribeRom(path) : NetplayRom{};
        fd = open(path, O_WRONLY | O_TRUNC);
        bytes[31] = 2;
        const bool secondWrite = fd >= 0 && write(fd, bytes, sizeof(bytes)) == sizeof(bytes);
        if (fd >= 0) close(fd);
        const NetplayRom cached = NetplayDescribeRom(path);
        const NetplayRom fresh = NetplayDescribeRom(path, true);
        const NetplayRom after = NetplayDescribeRom(path);
        unlink(path);
        const bool ok = firstWrite && secondWrite && before.Size == sizeof(bytes) &&
                        cached == before && fresh != before && after == fresh &&
                        fresh == NetplayDescribeRomBytes(bytes, sizeof(bytes));
        printf("ROM_REHASH %s\n", ok ? "PASS" : "FAIL");
        return ok ? 0 : 2;
    }
#endif

    // --- Unit 1 oracle modes -------------------------------------------------
    if (opt.mode != RunMode::Benchmark)
    {
        liteds::TraceRunConfig cfg;
        cfg.rom = opt.rom;
        cfg.roms = opt.roms;
        cfg.dataDir = opt.dataDir;
        cfg.jit = opt.jit;
        cfg.fixedRtcEpoch = opt.fixedRtc;
        cfg.inputScript = opt.inputScript;
        cfg.savestate = opt.savestate;

        switch (opt.mode)
        {
        case RunMode::RecordTrace:
            return liteds::RecordTrace(cfg, opt.frames, opt.tracePath);
        case RunMode::VerifyTrace:
            return liteds::VerifyTrace(cfg, opt.tracePath);
        case RunMode::VerifyInterpConverge:
            return liteds::VerifyInterpConverge(cfg, opt.frames);
        case RunMode::MPTest:
            return liteds::MPTest(cfg, opt.frames, opt.mpScripts);
        case RunMode::Replay:
            return liteds::ReplayConsole(cfg, opt.frames, opt.mpScripts, opt.replayConsole, opt.replayLog);
        default:
            break;
        }
    }

    // --- Load ROM ---
    u32 romlen = 0;
    auto romdata = ReadFile(opt.rom, romlen);
    if (!romdata)
    {
        fprintf(stderr, "error: cannot read ROM '%s'\n", opt.rom.c_str());
        return 1;
    }
#ifdef LITEV_AMBIENT_LAN
    NetplayRom acquiredCutRom;
    if (getenv("LITEV_AMBIENT_CUT_XFER_PORT"))
        acquiredCutRom = NetplayDescribeRomBytes(romdata.get(), romlen);
#endif

    auto cart = NDSCart::ParseROM(std::move(romdata), romlen, nullptr, std::nullopt);
    if (!cart)
    {
        fprintf(stderr, "error: failed to parse DS ROM '%s'\n", opt.rom.c_str());
        return 1;
    }

    // --- Build NDS (FreeBIOS + generated firmware, software renderer) ---
    NDSArgs args; // defaults: FreeBIOS ARM9/ARM7, generated NDS firmware
    args.Interpolation = opt.interp;
#ifdef JIT_ENABLED
    if (opt.jit)
    {
        JITArgs ja{};               // default JIT settings
        ja.FastMemory = opt.fastmem;
        args.JIT = ja;
    }
    else
        args.JIT = std::nullopt;    // interpreter
#else
    if (opt.jit)
        fprintf(stderr, "warning: build has no JIT; running interpreter\n");
    args.JIT = std::nullopt;
#endif

    // NDS is a large object; heap-allocate it (a stack instance overflows the
    // main thread stack).
    auto nds = std::make_unique<NDS>(std::move(args), nullptr);
    nds->SetRenderer(std::make_unique<SoftRenderer>(*nds));
    // The qt_sdl frontend enables 3D render threading via SetRenderSettings;
    // the headless harness never did, so the (banded) threaded soft-3D path was
    // dead code here. Enable it (default on; LITEV_3DTHREAD=0 forces serial) so
    // the banded raster + render-thread overlap are actually exercised.
    {
        const char* e = getenv("LITEV_3DTHREAD");
        bool threaded = !(e && e[0] == '0');
        melonDS::RendererSettings rs{};
        rs.ScaleFactor = 1;
        rs.Threaded = threaded;
        rs.HiresCoordinates = false;
        rs.BetterPolygons = false;
        rs.Accurate3D = opt.accurate3d;
#ifdef LITEV_HEADLESS_GL
        if (opt.renderer != "soft")
        {
            if (!MacGLCreate() || !gladLoadGLLoader((GLADloadproc)MacGLProc))
            { fprintf(stderr, "error: no OpenGL 4.1 core context\n"); return 1; }
            MacGLShims();
            if (opt.renderer == "hybrid") nds->GPU.SetRenderer(std::make_unique<HybridRenderer>(*nds));
            else if (opt.renderer == "glhires") nds->GPU.SetRenderer(std::make_unique<GLRenderer>(*nds, false));
            else { fprintf(stderr, "error: --renderer soft|hybrid|glhires\n"); return 1; }
            auto& r = nds->GPU.GetRenderer();
            while (r.NeedsShaderCompile()) { int cur = 0, cnt = 0; r.ShaderCompileStep(cur, cnt); }
            rs.ScaleFactor = opt.glScale;
        }
#else
        if (opt.renderer != "soft") { fprintf(stderr, "error: build has no GL renderers (macOS + ENABLE_OGLRENDERER)\n"); return 1; }
#endif
        nds->GPU.GetRenderer().SetRenderSettings(rs);
    }
    nds->SetNDSCart(std::move(cart));
    if (const char* sav = getenv("LITEV_HEADLESS_SAV"))   // game save (.sav) to boot with
    {
        u32 savlen = 0;
        if (auto s = ReadFile(sav, savlen)) nds->SetNDSSave(s.get(), savlen);
        else fprintf(stderr, "warning: cannot read save '%s'\n", sav);
    }
    nds->Reset();

    // Direct boot so no external firmware boot menu / BIOS files are needed.
    nds->SetupDirectBoot("headless.nds");
    nds->Start();

    if (!opt.savestate.empty())
    {
        if (!LoadSavestate(*nds, opt.savestate))
            return 1;
    }

    nds->SetKeyMask(0xFFFF); // no buttons pressed (active-low)

    // Scripted input (optional). Loaded once; the run loop applies the level
    // key mask for each frame before RunFrame so menu-driven test ROMs can be
    // navigated. Script content is part of the deterministic input, so a trace
    // recorded with a script only replays under the same script.
    liteds::InputScript inputScript;
    if (!opt.inputScript.empty())
    {
        std::string err;
        if (!inputScript.LoadFile(opt.inputScript, err))
        {
            fprintf(stderr, "error: %s\n", err.c_str());
            return 1;
        }
        fprintf(stderr, "input-script: %s (%zu directives, hash=%016llx)\n",
                opt.inputScript.c_str(), inputScript.DirectiveCount(),
                (unsigned long long)inputScript.Hash());
    }

    if (opt.frameskip > 0)
    {
#ifdef LITEV_AGGRESSIVE_SKIP
        nds->GPU.SetFrameskipTarget(opt.frameskip);
        fprintf(stderr, "frameskip: rendering 1 of every %d frames\n", opt.frameskip + 1);
#else
        fprintf(stderr, "warning: --frameskip ignored (build lacks LITEV_AGGRESSIVE_SKIP)\n");
#endif
    }

    // Report the engine that ACTUALLY engaged (not just what was requested), so a
    // silent JIT->interp or fastmem->slowmem fallback is detectable on device.
#ifdef JIT_ENABLED
    {
        bool jitOn = nds->IsJITEnabled();
        bool fmOn  = jitOn && nds->JIT.FastMemoryEnabled();
        fprintf(stderr, "engine: %s  fastmem_requested=%s  fastmem_active=%s\n",
                jitOn ? "JIT" : "interpreter",
                opt.fastmem ? "on" : "off",
                fmOn ? "yes" : "no");
        if (opt.jit && !jitOn)
            fprintf(stderr, "warning: JIT requested but interpreter engaged (silent fallback)\n");
        if (opt.jit && opt.fastmem && !fmOn)
            fprintf(stderr, "note: fastmem requested but inactive (unsupported on this platform)\n");
    }
#endif

    fprintf(stderr, "liteDS-headless: rom=%s mode=%s frames=%d jit=%s\n",
            opt.rom.c_str(), opt.jit ? "jit" : "interp", opt.frames,
#ifdef JIT_ENABLED
            opt.jit ? "yes" : "no");
#else
            "unavailable");
#endif

    // --- Run loop ---
    u64 lastTopHash = 0, lastBotHash = 0;
    bool anyChange = false;
    u64 firstTopHash = 0;
    bool haveFirst = false;

    // Rolling xxhash over all SPU output samples. This lets acceptance checks
    // detect audio-only changes (e.g. LITEV_SPU_FAST_INTERP) that leave the
    // framebuffer identical. Deterministic given a fixed ROM + frame count.
    XXH3_state_t* audioHashState = XXH3_createState();
    XXH3_64bits_reset(audioHashState);
    u64 audioSampleCount = 0;                 // stereo frames drained
    std::vector<s16> audioDrain(2048 * 2);    // interleaved L/R scratch buffer
    // LITEV_WAV=<path>: also write the output as a 48 kHz 16-bit stereo WAV (listening tests)
    FILE* wav = getenv("LITEV_WAV") ? fopen(getenv("LITEV_WAV"), "wb") : nullptr;
    if (wav) fseek(wav, 44, SEEK_SET);

    auto wallStart = std::chrono::steady_clock::now();
    timespec emuCpu0{};   // emu (this) thread CPU time: the device A/B metric (blocked time excluded)
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &emuCpu0);
#ifdef __APPLE__
    // emu-thread instructions/cycles (this thread only; render workers excluded): deterministic
    // A/B metric on a loaded Mac, where wall time is noise.
    uint64_t thsc0[2] = {0, 0};
    thread_selfcounts(1, thsc0, sizeof(thsc0));
#endif

    // --bench-window: measured span within the full run. windowStart is stamped
    // just before frame benchWindowStart's RunFrame; windowEnd just after frame
    // benchWindowEnd's RunFrame. The window covers [start,end] inclusive.
    const bool haveWindow = opt.benchWindowStart >= 0;
    std::chrono::steady_clock::time_point windowStart{}, windowEnd{};
    timespec windowCpuStart{}, windowCpuEnd{};

#if LITEV_PROFILE
    // Run totals: g_Frame is reset every frame, so accumulate each frame's counters
    // into totals here to observe whole-run behaviour (esp. the Unit 4 link counters).
    struct { uint64_t linksPatched=0, linksUnlinked=0, pendingPeak=0,
                       cppReentries=0, dispatcherMisses=0, dispatcherHits=0, icacheHits=0, rasHits=0,
                       linkSitesEmitted=0, dispatchOnlyExits=0,
                       schedIterations=0, schedEventsFired=0,
                       arm9ExecNs=0, arm7ExecNs=0, gpu3dNs=0, runSystemNs=0, spuMixNs=0,
                       runFrameNs=0, dma9Ns=0, dma7Ns=0, gxCommands=0, lightingCalls=0,
                       arm9IdleHits=0, arm7IdleHits=0,
                       idleCandidates=0, idleAccepted=0, idleRejWriteMem=0,
                       idleRejCoproc=0, idleRejBranch=0, idleRejRegDep=0,
                       memBlock9HelperCalls=0, memRead9U32HelperCalls=0;
#ifdef LITEV_JIT_DIRECTPATCH
             uint64_t directGuardHits=0, directGuardMisses=0, directPromotions=0,
                       directDemotions=0, directReverts=0;
#endif
             } profTotals;
    uint64_t schedByType[32] = {0};
    // Frames actually folded into profTotals. When --bench-window is set we
    // only accumulate frames inside [start,end] so the M6.11 decomposition is
    // window-scoped (per-frame averages divide by this). Without a window it
    // equals opt.frames (whole-run behaviour, unchanged).
    uint64_t profFrames = 0;
#endif

    if (opt.macXor)
    {
        melonDS::Firmware& fw = nds->GetFirmware();
        fw.GetHeader().MacAddr[5] ^= (melonDS::u8) opt.macXor;
        fw.UpdateChecksums();
    }

#ifdef LITEV_HEADLESS_LAN
    // LAN client: join a session hosted elsewhere (the SereneDS app's Multiplayer lobby, or
    // desktop melonDS on the same core version) before the game boots, then run in real time:
    // the other end is a live 60 fps instance, and DS wireless timing assumes both advance at
    // the same rate.
    bool lan = !opt.lanJoin.empty() || opt.lanHost > 0 || opt.paced;
    auto lanNext = std::chrono::steady_clock::now();
#ifdef LITEV_AMBIENT_LAN
    melonDS::AmbientLAN* ambient = nullptr;
    // Test-only two-process cut probe. Ambient's reliable control channel and
    // sequenced MP retirement fence coordinate the pause; this directory holds
    // only the resulting diagnostic checkpoints.
    const char* cutDir = getenv("LITEV_AMBIENT_CUT_DIR");
    const char* cutRole = getenv("LITEV_AMBIENT_CUT_ROLE");
    const int cutMinFrame = getenv("LITEV_AMBIENT_CUT_MIN_FRAME") ? atoi(getenv("LITEV_AMBIENT_CUT_MIN_FRAME")) : 0;
    const u16 cutXferPort = getenv("LITEV_AMBIENT_CUT_XFER_PORT") ? (u16)atoi(getenv("LITEV_AMBIENT_CUT_XFER_PORT")) : 0;
    const int cutSwapTrialFrames = getenv("LITEV_AMBIENT_CUT_SWAP_TRIAL_FRAMES") ?
                                   atoi(getenv("LITEV_AMBIENT_CUT_SWAP_TRIAL_FRAMES")) : 0;
    // Only the headless cut harness may impersonate an Android handoff build.
    // Production builds still require their compile-time native fingerprint.
    // LITEV_AMBIENT_CUT_TEST_BUILD overrides it (negative tests: a peer with other emulation options).
    const char* cutBuildText = getenv("LITEV_AMBIENT_CUT_TEST_BUILD");
    u64 cutTestBuild = NetplayHandoffCompat();
    if (cutBuildText)
    {
        char* end = nullptr;
        errno = 0;
        const unsigned long long parsed = strtoull(cutBuildText, &end, 0);
        if (!cutXferPort || errno || end == cutBuildText || *end || !parsed)
        { fprintf(stderr, "CUTFENCE invalid test build fingerprint\n"); return 2; }
        cutTestBuild = (u64)parsed;
    }
    const char* cutPeerIPText = getenv("LITEV_AMBIENT_CUT_PEER_IP");
    const std::string cutPeerIP = cutPeerIPText ? cutPeerIPText : "192.168.18.3";
    const bool cutExternalPeer = getenv("LITEV_AMBIENT_CUT_EXTERNAL_PEER") != nullptr;
    if (cutExternalPeer)
    {
        in_addr peerAddress{};
        if (!cutXferPort || !cutPeerIPText || !cutRole ||
            strcmp(cutRole, "host") != 0 || cutSwapTrialFrames <= 0 ||
            inet_pton(AF_INET, cutPeerIP.c_str(), &peerAddress) != 1 ||
            !getenv("LITEV_NETPLAY"))
        { fprintf(stderr, "CUTFENCE external peer needs host role, trial frames, port, IPv4 and Netplay configuration\n"); return 2; }
    }
    // LITEV_AMBIENT_CUT_FENCE_LOOP=N: N cut fences in one run (fast stress of the fence, no Netplay): each
    // completed fence is aborted on purpose (by the host, after its peer saw it complete too), both
    // reset and the game runs on LOOP_GAP frames (default 60) before the next offer. A summary line
    // "CUTLOOP done ok=K fail=F" ends the run.
    const int cutFenceLoop = getenv("LITEV_AMBIENT_CUT_FENCE_LOOP") ? atoi(getenv("LITEV_AMBIENT_CUT_FENCE_LOOP")) : 0;
    const int cutLoopGap = getenv("LITEV_AMBIENT_CUT_LOOP_GAP") ? atoi(getenv("LITEV_AMBIENT_CUT_LOOP_GAP")) : 60;
    int cutLoopOk = 0, cutLoopFail = 0, cutNextFrame = 0;
    const bool cutRetryAfterAbort = cutFenceLoop || (getenv("LITEV_AMBIENT_CUT_RETRY_AFTER_ABORT") &&
                                    *getenv("LITEV_AMBIENT_CUT_RETRY_AFTER_ABORT"));
    const int cutRetries = cutFenceLoop ? cutFenceLoop - 1 : 1;   // attempts after the first
    int cutAttempt = 0;
    // milliseconds since this console froze for the cut (the CUTFENCE lines' t= field)
    auto cutFreezeStart = std::chrono::steady_clock::now();
    auto frozenMs = [&] { return (long)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - cutFreezeStart).count(); };
    bool cutRetryReady = false;
    bool cutResetDelayed = false;
    bool cutOfferSent = false;
    bool cutRomVerified = false;
    std::unique_ptr<NetplayLiveLink> cutLink;   // the live handoff's TCP link, from the verified offer on
    std::chrono::steady_clock::time_point cutBaselineAt;
    std::array<u8, 16> cutOfferToken{};
    NetplayRom cutOwnRom, cutPeerRom;
    MultiCut multiCut;
    if (const char* m = getenv("LITEV_AMBIENT_CUT_MULTI"); m && atoi(m) >= 2)
    {
        multiCut.N = atoi(m);
        multiCut.Host = cutRole && !strcmp(cutRole, "host");
        multiCut.MinFrame = cutMinFrame;
        multiCut.Trial = cutSwapTrialFrames;
        multiCut.Xfer = cutXferPort;
        multiCut.NpPort = getenv("LITEV_AMBIENT_CUT_NP_PORT") ? (u16)atoi(getenv("LITEV_AMBIENT_CUT_NP_PORT")) : (u16)(cutXferPort + 300);
        multiCut.Dir = cutDir ? cutDir : ".";
        multiCut.Library = {opt.rom};
        if (const char* lib = getenv("LITEV_AMBIENT_CUT_PEER_ROMS"))
            for (std::string v = lib; !v.empty();)
            {
                const size_t e = v.find('|');
                multiCut.Library.push_back(v.substr(0, e));
                v = e == std::string::npos ? "" : v.substr(e + 1);
            }
        multiCut.OwnRom = acquiredCutRom;
        if (!cutXferPort || !cutRole || !multiCut.OwnRom.Size || !multiCut.Trial)
        { fprintf(stderr, "CUTMULTI needs LITEV_AMBIENT_CUT_XFER_PORT, _ROLE, _SWAP_TRIAL_FRAMES\n"); return 2; }
        cutDir = nullptr;   // (the two-player cut harness below stays off)
        printf("CUTMULTI %s: %d-player live handoff harness, offer from frame %d, %d Netplay frames\n", cutRole, multiCut.N, cutMinFrame, multiCut.Trial);
    }
    if (cutXferPort && !multiCut.N)
    {
        const char* peerRomPath = getenv("LITEV_AMBIENT_CUT_PEER_ROM");
        if (!cutDir || !cutRole || !peerRomPath || !*peerRomPath)
        { fprintf(stderr, "CUTFENCE transfer requires cut dir, role, and peer ROM path\n"); return 2; }
        cutOwnRom = NetplayDescribeRom(opt.rom);
        cutPeerRom = NetplayDescribeRom(peerRomPath);
        if (!cutOwnRom.Size || !cutPeerRom.Size || cutOwnRom != acquiredCutRom)
        { fprintf(stderr, "CUTFENCE transfer ROM description failed\n"); return 2; }
    }
    if (!opt.ambient.empty())
    {
        melonDS::AmbientLAN::Config c;
        c.DeviceID = getenv("LITEV_AMBIENT_ID") ? strtoull(getenv("LITEV_AMBIENT_ID"), nullptr, 16)
                                                : std::hash<std::string>()(opt.ambient) ^ ((u64)getpid() << 32);
        c.Name = opt.ambient;
        c.RequireHandoffOffer = cutXferPort != 0;
        c.BuildID = (u32)(cutTestBuild ^ (cutTestBuild >> 32));
        memcpy(c.GameCode, nds->NDSCartSlot.GetCart()->GetHeader().GameCode, 4);
        if (getenv("LITEV_AMBIENT_UPORT")) c.UnicastPort = (u16)atoi(getenv("LITEV_AMBIENT_UPORT"));
        if (getenv("LITEV_AMBIENT_PPORT")) c.PresencePort = (u16)atoi(getenv("LITEV_AMBIENT_PPORT"));   // (tests: a router that drops broadcast)
        // (tests: off the default ports, so Mac test runs never meet a device's discovery or a device test's ports)
        if (getenv("LITEV_AMBIENT_LPORT")) c.LinkPort = (u16)atoi(getenv("LITEV_AMBIENT_LPORT"));
        if (getenv("LITEV_AMBIENT_VIS")) c.Vis = atoi(getenv("LITEV_AMBIENT_VIS"));
        auto a = std::make_unique<melonDS::AmbientLAN>(c);
        if (!a->Ok()) { fprintf(stderr, "Ambient: could not start\n"); return 1; }
        if (const char* peers = getenv("LITEV_AMBIENT_PEERS"))
            for (std::string t : [&] { std::vector<std::string> v; std::string x; for (const char* q = peers;; q++) { if (!*q || *q == ',') { if (!x.empty()) v.push_back(x); x.clear(); if (!*q) break; } else x += *q; } return v; }())
            {
                auto colon = t.find(':');
                if (colon == std::string::npos) continue;
                in_addr ia;
                if (inet_pton(AF_INET, t.substr(0, colon).c_str(), &ia) == 1) a->AddPeerAddress(ia.s_addr, (u16)atoi(t.c_str() + colon + 1));
            }
        // the console's MAC from the device id (two consoles with one MAC cannot see each other)
        melonDS::Firmware& fw = nds->GetFirmware();
        u8 mac[6];
        melonDS::AmbientLAN::MacFromID(c.DeviceID, mac);
        memcpy(fw.GetHeader().MacAddr.data(), mac, 6);
        fw.UpdateChecksums();
        printf("Ambient: '%s' id %016llx game %.4s presence port %u link port %u\n", c.Name.c_str(), (unsigned long long)c.DeviceID,
               c.GameCode, a->GetUnicastPort(), a->GetLinkPort());
        ambient = a.get();
        // the app's wait for a peer's MP frame (kLanRecvTimeoutMs; LITEV_MPTIMEOUT overrides)
        a->SetRecvTimeout(getenv("LITEV_MPTIMEOUT") ? atoi(getenv("LITEV_MPTIMEOUT")) : 50);
        melonDS::MPInterface::Set(std::move(a), melonDS::MPInterface_Ambient);
        lan = true;
    }
    else
#endif
    if (lan && !opt.paced)
    {
        melonDS::MPInterface::Set(melonDS::MPInterface_LAN);
        auto& l = (melonDS::LAN&) melonDS::MPInterface::Get();
        if (opt.lanHost > 0)
        {
            if (!l.StartHost(opt.lanName.c_str(), opt.lanHost))
            {
                fprintf(stderr, "LAN: could not host\n");
                return 1;
            }
            printf("LAN: hosting as '%s' (max %d players)\n", opt.lanName.c_str(), opt.lanHost);
        }
        else if (!l.StartClient(opt.lanName.c_str(), opt.lanJoin.c_str()))
        {
            fprintf(stderr, "LAN: could not join %s\n", opt.lanJoin.c_str());
            return 1;
        }
        else
            printf("LAN: joined %s as '%s'\n", opt.lanJoin.c_str(), opt.lanName.c_str());
    }
#else
    if (!opt.lanJoin.empty())
        fprintf(stderr, "warning: --lan-join ignored (build lacks LITEV_HEADLESS_LAN)\n");
#endif

    // LITEV_CLONE_MAC=<12 hex>:<k>, LITEV_CLONE_RAMPATCH=<hex>:<hex> (tests): this console started from another's
    // savestate becomes another player (as --mp-test's LITEV_MP_MACSWAP / LITEV_MP_RAMPATCH do for its consoles k > 0)
    if (const char* cm = getenv("LITEV_CLONE_MAC"); cm && strchr(cm, ':'))
        printf("clone MAC: %d copies\n", liteds::CloneMac(*nds, cm, atoi(strchr(cm, ':') + 1)));
    if (const char* rp = getenv("LITEV_CLONE_RAMPATCH"))
        printf("clone RAM patch: %d copies\n", liteds::PatchRam(*nds, rp));
    // App record mode (MelonDS.cpp recordFrame): every 60 frames the app logs a hash of the input
    // applied, the clock and the RAM. Same formula here, so a replay proves it matches the device.
    std::map<int, unsigned long long> recordedHashes;
    if (!opt.checkRecording.empty())
        if (FILE* f = fopen(opt.checkRecording.c_str(), "r"))
        {
            // the "hash" column by name: newer recordings append columns after it (jit, mprotect, rewrites);
            // taking the last field read those as the hash -> a false "REPLAY DIFFERS at frame 59"
            char line[512];
            int fr, col = 7;
            unsigned long long h;
            if (fgets(line, sizeof(line), f))
            {
                int i = 0;
                for (char* t = strtok(line, ",\r\n"); t; t = strtok(nullptr, ",\r\n"), i++)
                    if (!strcmp(t, "hash")) col = i;
            }
            while (fgets(line, sizeof(line), f))
            {
                const char* c = line;
                for (int i = 0; i < col && c; i++) { c = strchr(c, ','); if (c) c++; }
                if (sscanf(line, "%d", &fr) == 1 && c && sscanf(c, "%llx", &h) == 1)
                    recordedHashes[fr] = h;
            }
            fclose(f);
        }
    int firstDiff = -1, hashesChecked = 0;
    FILE* video = nullptr;
    if (!opt.videoMp4.empty())
    {
        std::string cmd = "ffmpeg -loglevel error -y -f rawvideo -pix_fmt rgb24 -s 256x384 -r 60 -i - "
                          "-c:v libx264 -crf 20 -pix_fmt yuv420p '" + opt.videoMp4 + "'";
        video = popen(cmd.c_str(), "w");
    }

#ifdef LITEV_REMOTE_GX_SINK
    // (measurement) LITEV_SP_GXSINK=1: this console as a Netplay remote's geometry (fixed GX timing + sink: no 3D output)
    if (getenv("LITEV_SP_GXSINK")) { nds->GPU.GPU3D.TimingFixed = true; nds->GPU.GPU3D.Sink = atoi(getenv("LITEV_SP_GXSINK")) != 0; }
    if (getenv("LITEV_SP_GXFIXED")) nds->GPU.GPU3D.TimingFixed = true;   // (measurement) fixed GX timing only
#endif
#ifdef LITEV_FF_HEADLESS3D
    if (getenv("LITEV_FFHEADLESS")) nds->GPU.FFHeadless3D = atoi(getenv("LITEV_FFHEADLESS")) != 0;   // the app's fast-forward mode
#ifdef LITEV_LCD_EVT
    if (getenv("LITEV_LCDEVT")) nds->GPU.LcdEvt = atoi(getenv("LITEV_LCDEVT")) != 0;   // A/B (default on)
#endif
#endif
#ifdef LITEV_SKIP_REPEAT_FRAMES
    if (getenv("LITEV_SKIPREPEAT")) nds->GPU.SkipRepeatEnabled = atoi(getenv("LITEV_SKIPREPEAT")) != 0;   // the app's "Skip repeated frames"
#endif
    // LITEV_LOOP=<n>: run the --frames window n more times from --savestate (profile a short burst)
    int loopsLeft = getenv("LITEV_LOOP") ? atoi(getenv("LITEV_LOOP")) : 0;
    for (int frame = 0; frame < opt.frames; frame++)
    {
        if (frame == opt.frames - 1 && loopsLeft > 0 && !opt.savestate.empty())
        {
            loopsLeft--;
            if (!LoadSavestate(*nds, opt.savestate)) { fprintf(stderr, "error: loop reload failed\n"); return 1; }
            frame = 0;
        }
        // test hook: LITEV_STATE_ROUNDTRIP=<frame>: save the state to a file and load it back before that frame (a state the
        // savestate misses shows as a hash change from there on, against a run without it)
        if (static const int rt = getenv("LITEV_STATE_ROUNDTRIP") ? atoi(getenv("LITEV_STATE_ROUNDTRIP")) : -1; rt == frame && opt.reloadAt < 0)
        {
            const std::string tmp = std::string(getenv("TMPDIR") ? getenv("TMPDIR") : "/tmp") + "/litev-roundtrip.mln";
            if (!SaveSavestate(*nds, tmp) || !LoadSavestate(*nds, tmp)) { fprintf(stderr, "error: roundtrip failed\n"); return 1; }
            fprintf(stderr, "state roundtrip before frame %d\n", frame);
        }
        if (frame == opt.reloadAt && !opt.savestate.empty())
        {
            if (!LoadSavestate(*nds, opt.savestate)) { fprintf(stderr, "error: reload failed\n"); return 1; }
            fprintf(stderr, "reloaded %s after %d frames\n", opt.savestate.c_str(), frame);
            opt.reloadAt = -1;
            frame = 0;
        }
        if (inputScript.Loaded())
        {
            nds->SetKeyMask(inputScript.KeyMaskForFrame(frame));
            int tx, ty;
            if (inputScript.TouchForFrame(frame, tx, ty)) nds->TouchScreen(tx, ty);
            else if (inputScript.HasTouch()) nds->ReleaseScreen();
        }

        // test hook: LITEV_POKE32="frame:addr:value,..." writes a guest word (ARM9 bus) before
        // that frame runs, e.g. to check that code rewrites invalidate JIT/HLE assumptions
        if (static const char* pk = getenv("LITEV_POKE32"); pk)
            for (const char* q = pk; *q; )
            {
                unsigned long pf = strtoul(q, (char**)&q, 0); q += (*q == ':');
                unsigned long pa = strtoul(q, (char**)&q, 0); q += (*q == ':');
                unsigned long pv = strtoul(q, (char**)&q, 0); q += (*q == ',');
                if ((int)pf == frame) { nds->ARM9Write32((u32)pa, (u32)pv); fprintf(stderr, "poke32 frame %d %08lx = %08lx\n", frame, pa, pv); }
            }
        // same through the ARM7 bus (ARM7 WRAM code)
        if (static const char* pk = getenv("LITEV_POKE32_ARM7"); pk)
            for (const char* q = pk; *q; )
            {
                unsigned long pf = strtoul(q, (char**)&q, 0); q += (*q == ':');
                unsigned long pa = strtoul(q, (char**)&q, 0); q += (*q == ':');
                unsigned long pv = strtoul(q, (char**)&q, 0); q += (*q == ',');
                if ((int)pf == frame) { nds->ARM7Write32((u32)pa, (u32)pv); fprintf(stderr, "poke32 arm7 frame %d %08lx = %08lx\n", frame, pa, pv); }
            }
        if (haveWindow && frame == opt.benchWindowStart)
        {
            windowStart = std::chrono::steady_clock::now();
            clock_gettime(CLOCK_THREAD_CPUTIME_ID, &windowCpuStart);
        }

        LITE_PROFILE_RESET_FRAME();
        static const bool frameMs = getenv("LITEV_FRAME_MS") != nullptr;   // per-frame time + JIT compiles
        const auto fms0 = std::chrono::steady_clock::now();
        const u64 jit0 = melonDS::JitCompileCount, prot0 = melonDS::JitProtectCalls, flt0 = melonDS::JitProtectFaults;
#ifdef __APPLE__
        uint64_t tc0[2] = {};
        thread_selfcounts(1, tc0, sizeof(tc0));
#else
        timespec tcs0; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &tcs0);
#endif
#ifdef __linux__
        {   // LITEV_MARKFRAMES=A-B: name the thread "markframes" during frames A..B (perf report --comm)
            static int ma = -1, mb = -2;
            static bool once = [] { if (const char* m = getenv("LITEV_MARKFRAMES")) sscanf(m, "%d-%d", &ma, &mb); return true; }();
            (void)once;
            if (frame == ma) prctl(PR_SET_NAME, "markframes");
            if (frame == mb + 1) prctl(PR_SET_NAME, "headless");
        }
#endif
#ifdef LITEV_AMBIENT_LAN
        const auto runStart = std::chrono::steady_clock::now();
#endif
        nds->RunFrame();
        // LITEV_FRAME_LOAD_MS=<ms> (tests): busy-wait that long a frame, as a slower device's emulation (the RG DS
        // needs ~10 ms a frame where the Mac needs ~2): link round trips then cost frames as they do there
        if (static const double loadMs = getenv("LITEV_FRAME_LOAD_MS") ? atof(getenv("LITEV_FRAME_LOAD_MS")) : 0; loadMs > 0)
        {
            const auto until = std::chrono::steady_clock::now() + std::chrono::microseconds((long long)(loadMs * 1000));
            while (std::chrono::steady_clock::now() < until) {}
        }
#ifdef LITEV_AMBIENT_LAN
        if (ambient) ambient->NoteFrameUs((u32)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - runStart).count());
#endif
        if (inputScript.HasProgram())   // closed-loop script steps (@until / @repeat ... rgb) see this frame's screens
        {
            void* top = nullptr; void* bot = nullptr;
            if (nds->GPU.GetFramebuffers(&top, &bot) && top && bot) inputScript.OnFrame(frame, (const u32*)top, (const u32*)bot);
            else inputScript.OnFrame(frame, nullptr, nullptr);
        }
        const auto scriptMilestones = inputScript.TakeMilestones();
        if (static const bool p2d = getenv("LITEV_PRINT_2D") != nullptr; p2d)   // display registers after the frame
            fprintf(stderr, "2D f%d A: dispcnt=%08x bldcnt=%04x eva=%d evb=%d evy=%d mb=%04x  B: dispcnt=%08x bldcnt=%04x evy=%d mb=%04x cap=%08x\n", frame,
                    nds->GPU.GPU2D_A.DispCnt, nds->GPU.GPU2D_A.BlendCnt, nds->GPU.GPU2D_A.EVA, nds->GPU.GPU2D_A.EVB, nds->GPU.GPU2D_A.EVY, nds->GPU.MasterBrightnessA,
                    nds->GPU.GPU2D_B.DispCnt, nds->GPU.GPU2D_B.BlendCnt, nds->GPU.GPU2D_B.EVY, nds->GPU.MasterBrightnessB, nds->GPU.CaptureCnt);
#ifdef __APPLE__
        {
            uint64_t tc1[2] = {};
            thread_selfcounts(1, tc1, sizeof(tc1));
            EmuThreadInstr += tc1[0] - tc0[0];
            EmuThreadCycles += tc1[1] - tc0[1];
        }
#endif
#ifndef __APPLE__
        {
            timespec tcs1; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &tcs1);
            EmuThreadCycles += (tcs1.tv_sec - tcs0.tv_sec) * 1000000000ull + tcs1.tv_nsec - tcs0.tv_nsec;
        }
#endif
        if (frameMs)
{
            static u64 li, lc;   // emu-thread instructions / cycles this frame (macOS thread counters)
            printf("FRAME %d %.3f ms jit %llu mprotect %llu rewrites %llu instr %llu cyc %llu\n", frame,
                   std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - fms0).count(),
                   (unsigned long long)(melonDS::JitCompileCount - jit0),
                   (unsigned long long)(melonDS::JitProtectCalls - prot0), (unsigned long long)(melonDS::JitProtectFaults - flt0),
                   (unsigned long long)(EmuThreadInstr - li), (unsigned long long)(EmuThreadCycles - lc));
            li = EmuThreadInstr; lc = EmuThreadCycles;
            }
#ifdef LITEV_JIT_COMPILE_STATS
        if (frameMs)
        {   // compile time per phase this frame, microseconds (src/ARMJIT_Internal.h JitPhase)
            static u64 last[melonDS::JitPhase_Count];
            static const double us = 1e6 / melonDS::JitTicksPerSec();
            printf("JITPHASE %d", frame);
            for (int p = 0; p < melonDS::JitPhase_Count; p++)
            {
                printf(" %.0f", (melonDS::JitCompileTicks[p] - last[p]) * us);
                last[p] = melonDS::JitCompileTicks[p];
            }
            {   // LITEV_JIT_BGEMIT: emission on the worker / stolen by the emulator thread
                static u64 lw, ls;
                u64 w = melonDS::JitWorkerTicks.load(), st = melonDS::JitStealTicks.load();
                printf(" %.0f %.0f", (w - lw) * us, (st - ls) * us);
                lw = w; ls = st;
            }
            printf("\n");
        }
#endif
#ifdef LITEV_JIT_STORE_REPROMOTE
        if (frameMs) printf("REPROMOTED %d %llu\n", frame, (unsigned long long)melonDS::JitStoreRepromotions);
#endif
        {   // LITEV_PRINT_REGS=<frame>: display registers after that frame (diagnosis)
            static const int regsAt = getenv("LITEV_PRINT_REGS") ? atoi(getenv("LITEV_PRINT_REGS")) : -1;
            if (frame == regsAt)
                printf("REGS frame %d DISPCNT_A %08X DISPCNT_B %08X DISPCAPCNT %08X POWCNT %04X VRAMCNT %08X %08X BG0CNT %04X DISP3DCNT %04X MASTERBRIGHT %04X\n",
                       frame, nds->ARM9IORead32(0x04000000), nds->ARM9IORead32(0x04001000), nds->ARM9IORead32(0x04000064),
                       nds->ARM9IORead16(0x04000304), nds->ARM9IORead32(0x04000240), nds->ARM9IORead32(0x04000244),
                       nds->ARM9IORead16(0x04000008), nds->ARM9IORead16(0x04000060), nds->ARM9IORead16(0x0400006C));
        }

        static const bool emitHashes = getenv("LITEV_RECORD_EMIT") != nullptr;   // print them (tests)
        static const int emitEvery = getenv("LITEV_RECORD_EVERY") ? atoi(getenv("LITEV_RECORD_EVERY")) : 60;   // tests: every n frames
        if ((!recordedHashes.empty() || emitHashes) && ((frame + 1) % emitEvery) == 0)
        {
            auto it = recordedHashes.find(frame);
            if (it != recordedHashes.end() || emitHashes)
            {
                // the app's NetplayFrameInput: active-low keys, touch -1 when released
                u64 keys = inputScript.Loaded() ? (inputScript.KeyMaskForFrame(frame) & 0xFFF) : 0xFFF;
                int tx = -1, ty = -1;
                if (inputScript.Loaded() && !inputScript.TouchForFrame(frame, tx, ty)) tx = ty = -1;
                u64 v[4] = { keys | ((u64)(u16)tx << 32) | ((u64)(u16)ty << 48), nds->GetSysTimestamp(),
                             XXH3_64bits(nds->MainRAM, nds->MainRAMMask + 1), (u64)frame };
                u64 hash = XXH3_64bits(v, sizeof(v));
                if (emitHashes) printf("%d,0,0,0,0,1,0,%016llx,%llu,%016llx\n", frame, (unsigned long long) hash, (unsigned long long)v[1], (unsigned long long)v[2]);
                if (it != recordedHashes.end() && ++hashesChecked && hash != it->second && firstDiff < 0)
                    firstDiff = frame;
            }
        }

#ifdef LITEV_HEADLESS_LAN
        if (lan)
        {
            melonDS::MPInterface::Get().Process();
#ifdef LITEV_AMBIENT_LAN
            static const auto ambT0 = std::chrono::steady_clock::now();
            const double ambT = std::chrono::duration<double>(std::chrono::steady_clock::now() - ambT0).count();
            if (ambient)
            {
                // the session's state changes, when they happen (links, the slowdown offer)
                static u32 lastLinks = 0; static bool lastOffer = false; static int lastVerdict = 0;
                const auto st = ambient->GetStats();
                if (st.Verdict != lastVerdict)
                {
                    lastVerdict = st.Verdict;
                    static const char* const kVerdict[] = {"UNKNOWN", "LAN_OK", "LAN_BAD"};
                    printf("AMBIENT verdict t=%.2fs f%d: %s predicted %.1f fps, emulation %.1f ms, link_rtt %.1f ms\n", ambT, frame,
                           kVerdict[st.Verdict], st.PredFpsX100 / 100.0, st.EmuUs / 1000.0, st.LinkRttUs / 1000.0);
                    fflush(stdout);
                }
                if (st.Links != lastLinks || st.SlowdownPending != lastOffer)
                {
                    lastLinks = st.Links; lastOffer = st.SlowdownPending;
                    printf("AMBIENT event t=%.2fs f%d: links=%u offer=%d link_rtt=%.1fms\n", ambT, frame, st.Links, st.SlowdownPending ? 1 : 0, st.LinkRttUs / 1000.0);
                    fflush(stdout);
                }
            }
            if (ambient && cutRetryAfterAbort && cutAttempt == 1 && !cutFenceLoop && cutRole &&
                ambient->HandoffFenceAborted())
            {
                const char* slow = getenv("LITEV_AMBIENT_CUT_DELAY_RESET_ROLE");
                if (slow && strcmp(slow, cutRole) == 0 && !cutResetDelayed)
                {
                    cutResetDelayed = true;
                    printf("CUTFENCE %s delaying reset 250ms\n", cutRole);
                    std::this_thread::sleep_for(std::chrono::milliseconds(250));
                }
            }
            if (ambient && ambient->ResetHandoffFenceAfterAbort())
            {
                printf("CUTFENCE %s reset after mutual abort\n", cutRole ? cutRole : "peer");
                if (cutRetryAfterAbort && cutAttempt >= 1 && cutAttempt <= cutRetries && cutDir && cutRole)
                {
                    cutRetryReady = true;
                    cutNextFrame = frame + cutLoopGap;
                    cutOfferSent = false;
                    cutRomVerified = false;
                    cutLink.reset();
                    cutOfferToken = {};
                    if (strcmp(cutRole, "host") == 0 &&
                        getenv("LITEV_AMBIENT_CUT_STALE_REPLAY") &&
                        *getenv("LITEV_AMBIENT_CUT_STALE_REPLAY"))
                    {
                        const bool injected = ambient->InjectClosedHandoffFenceForTest();
                        printf("CUTFENCE host enqueued stale closed-epoch FENCE ok=%d\n", injected);
                        if (!injected) return 2;
                    }
                }
            }
            if (ambient && getenv("LITEV_AMBIENT_CUT_SCAN"))
            {
                const int start = atoi(getenv("LITEV_AMBIENT_CUT_SCAN"));
                if (frame >= start)
                    printf("CUTSCAN f%d busy=%02x queued=%zu\n", frame,
                           nds->Wifi.HandoffBusyMask(), ambient->HandoffQueuedFrames());
            }
            // LITEV_AMBIENT_TIMELINE=1: the LAN session's traffic every 0.25 s (exchanges = CMDs sent / received, MP frames
            // and their bytes each way) for burst-vs-steady studies
            if (static const bool tl = getenv("LITEV_AMBIENT_TIMELINE") != nullptr; tl && ambient)
            {
                static double t0 = -1; static int f0 = 0; static melonDS::AmbientLAN::Stats s0 = {};
                if (t0 < 0) { t0 = ambT; f0 = frame; s0 = ambient->GetStats(); }
                if (ambT - t0 >= 0.25)
                {
                    const auto s1 = ambient->GetStats();
                    const int n = std::max(1, frame - f0);
                    if (s1.Links)
                        printf("AMBTL t=%.2f f%d frames=%d exch/f=%.2f tx/f=%.2f rx/f=%.2f txB/f=%.0f rxB/f=%.0f fps=%.1f links=%u\n", ambT, frame, n,
                               (double)(s1.Exchanges - s0.Exchanges) / n, (double)(s1.LinkTx - s0.LinkTx) / n,
                               (double)(s1.LinkRx - s0.LinkRx) / n, (double)(s1.LinkTxBytes - s0.LinkTxBytes) / n,
                               (double)(s1.LinkRxBytes - s0.LinkRxBytes) / n, n / (ambT - t0), s1.Links);
                    t0 = ambT; f0 = frame; s0 = s1;
                }
            }
            if (ambient && frame % 120 == 0)
            {
                const auto st = ambient->GetStats();
                printf("AMBIENT f%d: nearby=%u links=%u opened=%llu presence tx=%llu rx=%llu link tx=%llu rx=%llu beacons=%llu blocked=%.1fms exch=%llu rtt=%ums slow_fps=%.2f slow_wait=%.2fms slow_exch=%.2f offer=%d host_wait=%.1fms/%llu miss=%llu timeout=%llu client_wait=%.1fms/%llu miss=%llu stale=%llu/%llu/%llu/%llu link_rtt=%.1fms t=%.1fs\n",
                       frame, st.Nearby, st.Links, (unsigned long long)st.LinksOpened, (unsigned long long)st.PresenceTx, (unsigned long long)st.PresenceRx,
                       (unsigned long long)st.LinkTx, (unsigned long long)st.LinkRx, (unsigned long long)st.BeaconsInjected, st.BlockedUs / 1000.0,
                       (unsigned long long)st.Exchanges, st.MaxRttMs, st.SlowFpsX100 / 100.0, st.SlowWaitUsPerFrame / 1000.0,
                       st.SlowExchangesPerFrameX100 / 100.0, st.SlowdownPending ? 1 : 0,
                       st.HostWaitUs / 1000.0, (unsigned long long)st.HostWaitCalls, (unsigned long long)st.HostWaitMisses,
                       (unsigned long long)st.HostWaitTimeouts,
                       st.ClientWaitUs / 1000.0, (unsigned long long)st.ClientWaitCalls, (unsigned long long)st.ClientWaitMisses,
                       (unsigned long long)st.Stale[0], (unsigned long long)st.Stale[1], (unsigned long long)st.Stale[2],
                       (unsigned long long)st.Stale[3], st.LinkRttUs / 1000.0, ambT);
                for (const auto& r : ambient->NearbyRows()) printf("AMBIENT  %s\n", r.c_str());
                fflush(stdout);
            }
            else
#endif
            if (frame % 120 == 0 && melonDS::MPInterface::GetType() == melonDS::MPInterface_LAN)
            {
                auto players = ((melonDS::LAN&) melonDS::MPInterface::Get()).GetPlayerList();
                printf("LAN f%d:", frame);
                for (const auto& p : players)
                    printf(" [%d %s status=%d ping=%u%s]", p.ID, p.Name, (int) p.Status, p.Ping, p.IsLocalPlayer ? " local" : "");
                printf("\n");
                fflush(stdout);
            }
            int period = 16667;
#ifdef LITEV_AMBIENT_LAN
            if (ambient)
            {
                // the LAN session's fast-forward (AmbientLAN::ToggleSpeed): LITEV_AMBIENT_FF=[NAME@]F:t/F:u...: at frame F
                // this player presses the key (t: 2x, u: uncapped); NAME@: only the player named NAME
                static std::map<int, int> ffPlan = [&] {
                    std::map<int, int> m;
                    std::string v = getenv("LITEV_AMBIENT_FF") ? getenv("LITEV_AMBIENT_FF") : "";
                    const size_t at = v.find('@');
                    if (at != std::string::npos) { if (v.substr(0, at) != opt.ambient) v.clear(); else v = v.substr(at + 1); }
                    for (size_t i = 0; i < v.size(); )
                    {
                        size_t e = v.find('/', i); if (e == std::string::npos) e = v.size();
                        const std::string t = v.substr(i, e - i);
                        const size_t c = t.find(':');
                        if (c != std::string::npos) m[atoi(t.c_str())] = t[c + 1] == 'u' ? 63 : 8;
                        i = e + 1;
                    }
                    return m;
                }();
                static int lastSpeed = 0;
                if (ffPlan.count(frame))
                    printf("AMBIENT ff press f%d: links=%u shows %d\n", frame, (unsigned)ambient->HasLinks(), ambient->HasLinks() ? ambient->ToggleSpeed(ffPlan[frame]) : -1);
                const int sp = ambient->HasLinks() ? ambient->Speed() : 0;
                if (sp != lastSpeed)
                {
                    lastSpeed = sp;
                    printf("AMBIENT speed f%d: %d by '%s'\n", frame, sp, ambient->SpeedWho().c_str());
                    fflush(stdout);
                }
                period = sp == 63 ? 0 : sp ? 16667 * 4 / sp : 16667;
                // LITEV_AMBIENT_SOLO_FF=<multiplier> (0: uncapped): this player's own fast-forward during discovery; a LAN
                // session ends it (1x from the first link; leaving the session stays 1x)
                static float soloFf = getenv("LITEV_AMBIENT_SOLO_FF") ? (float)atof(getenv("LITEV_AMBIENT_SOLO_FF")) : 1.0f;
                if (ambient->HasLinks() && soloFf != 1.0f) { soloFf = 1.0f; printf("AMBIENT solo ff off f%d: a link opened\n", frame); }
                if (soloFf != 1.0f) period = soloFf <= 0 ? 0 : (int)(16667 / soloFf);
            }
#endif
            if (!getenv("LITEV_LAN_NOPACE"))
            {
                lanNext += std::chrono::microseconds(period);
                // time lost (a link wait, a slow start, uncapped) is not made up in a burst: at most 100 ms behind
                const auto now = std::chrono::steady_clock::now();
                if (!period || now - lanNext > std::chrono::milliseconds(100)) lanNext = now;
                std::this_thread::sleep_until(lanNext);
            }
            if (ambient && multiCut.N)
            {
                const int rc = MultiCutFrame(multiCut, nds, ambient, opt, frame);
                if (rc >= 0) return rc;
            }
            else if (ambient && cutDir && cutRole && (frame >= cutMinFrame || ambient->HasPeerHandoffFence()) && ambient->GetStats().Links)
            {
                const bool host = strcmp(cutRole, "host") == 0;
                const std::string abort = std::string(cutDir) + "/abort";
                auto exists = [](const std::string& p) { FILE* f = fopen(p.c_str(), "rb"); if (!f) return false; fclose(f); return true; };
                auto mark = [](const std::string& p) { FILE* f = fopen(p.c_str(), "wb"); if (!f) return false; fclose(f); return true; };
                const bool retryOpen = cutRetryAfterAbort && cutAttempt >= 1 && cutAttempt <= cutRetries && cutRetryReady &&
                                       frame >= cutNextFrame;
                const bool eligible = (cutAttempt == 0 && !exists(abort)) || retryOpen;
                // the live link with the offer (as the app opens it), this console's state as the
                // baseline of the cut's delta
                auto openCutLink = [&] {
                    if (cutLink || !cutXferPort) return;
                    cutLink = std::make_unique<NetplayLiveLink>(host ? 0 : 1, cutXferPort, cutPeerIP);
                    std::vector<u8> base;
                    if (SaveSavestate(*nds, "", &base)) cutLink->SendBaseline(std::move(base));
                    cutBaselineAt = std::chrono::steady_clock::now();
                };
                // per process: a peer that already answered one run's offer must not see the next
                // run's as the same one (the app keeps its answer per epoch)
                static const u64 epochBase = 0x48414e0000000000ull | ((u64)getpid() << 16) | ((u64)time(nullptr) & 0xFF00);
                const u64 nextEpoch = epochBase + (u64)cutAttempt + 1;
                if (cutXferPort && eligible)
                {
                    if (host && !cutOfferSent)
                    {
                        std::random_device random;
                        for (u8& byte : cutOfferToken) byte = (u8)random();
                        if (ambient->ProposeHandoff(nextEpoch, cutOfferToken,
                                                    melonDS::AmbientLAN::HandoffPlainNetplay))
                        {
                            cutOfferSent = true;
                            printf("CUTFENCE host offered epoch=%llx\n", (unsigned long long)nextEpoch);
                            fflush(stdout);
                            openCutLink();
                        }
                    }
                    else if (!host)
                    {
                        u64 offeredEpoch = 0;
                        melonDS::AmbientLAN::HandoffMode offeredMode;
                        if (ambient->IncomingHandoffOffer(offeredEpoch, offeredMode) &&
                            offeredMode == melonDS::AmbientLAN::HandoffPlainNetplay &&
                            ambient->RespondToHandoffOffer(offeredEpoch, true))
                        {
                            printf("CUTFENCE client accepted epoch=%llx\n", (unsigned long long)offeredEpoch);
                            fflush(stdout);
                            openCutLink();
                        }
                    }
                }
                const bool offerReady = !cutXferPort || !host ||
                    ambient->HandoffOfferReady(nextEpoch, cutOfferToken,
                                               melonDS::AmbientLAN::HandoffPlainNetplay);
                bool romReady = !cutXferPort;
                if (cutXferPort && eligible)
                {
                    melonDS::AmbientLAN::HandoffOfferStatus offer{};
                    if (ambient->GetHandoffOfferStatus(offer) && offer.Accepted && (!host || offer.Epoch == nextEpoch))
                    {
                        if (!ambient->PublishHandoffRom(cutOwnRom))
                        { fprintf(stderr, "CUTFENCE local ROM manifest failed\n"); return 2; }
                        NetplayRom described;
                        if (ambient->GetPeerHandoffRom(described))
                        {
                            if (described != cutPeerRom)
                            { fprintf(stderr, "CUTFENCE peer ROM manifest differs\n"); return 2; }
                            romReady = true;
                            if (!cutRomVerified)
                            {
                                cutRomVerified = true;
                                printf("CUTFENCE %s exact peer ROM verified before cut\n", cutRole);
                                fflush(stdout);
                            }
                        }
                    }
                }
                if (eligible && offerReady && romReady && (frame % 120) == 0)
                {
                    const auto ws = ambient->GetHandoffFenceStats();
                    printf("CUTFENCE %s waiting f%d busy=%02x queued=%zu peerFence=%d (fence epoch=%llx aborted=%d compatible=%d)\n",
                           cutRole, frame, nds->Wifi.HandoffBusyMask(), ambient->HandoffQueuedFrames(), ambient->HasPeerHandoffFence(),
                           (unsigned long long)ws.Epoch, ws.Aborted, ambient->HandoffBuildCompatible());
                    fflush(stdout);
                }
                // Who freezes first is the game's MP role, not the handoff role: an MP client is
                // inside an exchange at nearly every frame boundary while its host keeps issuing
                // commands, so it waits for the peer's fence and then runs on to its own idle cut
                // (the host's last ACK retires). Anyone else may start the fence when idle.
                const bool mayStartFence = ambient->HasPeerHandoffFence() ||
                    (cutXferPort ? !nds->Wifi.IsMPClientMode() : host);
                // (models the player's answer: the app sends its baseline at the offer, seconds before;
                // a peer app acks it only once it has read this side's ROM, up to ~10 s on an SD card)
                const bool baselineReady = !cutLink || cutLink->BaselineAcked() || ambient->HasPeerHandoffFence() ||
                    (cutBaselineAt != std::chrono::steady_clock::time_point{} &&
                     std::chrono::steady_clock::now() - cutBaselineAt > std::chrono::seconds(20));
                if (eligible && offerReady && romReady && mayStartFence && baselineReady &&
                    nds->Wifi.HandoffBusyMask() == 0 && ambient->HandoffQueuedFrames() == 0)
                {
                    ++cutAttempt;
                    cutRetryReady = false;
                    melonDS::AmbientLAN::HandoffOfferStatus accepted{};
                    const u64 epoch = ambient->HasPeerHandoffFence() ? ambient->PeerHandoffEpoch() :
                        host ? epochBase + (u64)cutAttempt :
                        ambient->GetHandoffOfferStatus(accepted) ? accepted.Epoch : 0;
                    cutFreezeStart = std::chrono::steady_clock::now();   // (the game is frozen from here)
                    printf("CUTFENCE %s attempt=%d holding f%d clock=%llu epoch=%llx\n", cutRole, cutAttempt,
                           frame, (unsigned long long)nds->GetSysTimestamp(), (unsigned long long)epoch);
                    fflush(stdout);
                    std::array<u8, 16> token{};
                    if (host && cutXferPort) token = cutOfferToken;
                    else if (host) { std::random_device random; for (u8& byte : token) byte = (u8)random(); }
                    if (!ambient->StartHandoffFence(epoch, token)) { mark(abort); fprintf(stderr, "CUTFENCE start failed\n"); return 2; }
                    if (const char* inject = getenv("LITEV_AMBIENT_CUT_POSTFENCE_SEND"); inject && strcmp(inject, cutRole) == 0)
                    {
                        u8 packet[36] = {};
                        ambient->SendCmd(0, packet, sizeof(packet), nds->GetSysTimestamp());
                        printf("CUTFENCE %s injected post-fence MP send; aborted=%d\n", cutRole,
                               ambient->HandoffFenceAborted());
                        fflush(stdout);
                    }
                    // This console's state is final from its freeze: capture and send the checkpoint
                    // now, while the fence completes (the checkpoint does not depend on it).
                    std::vector<u8> earlyState;
                    if (cutXferPort && cutLink && !getenv("LITEV_HANDOFF_WARM_CHECKPOINT") &&
                        SaveSavestate(*nds, "", &earlyState, true))
                    {
                        NetplayLiveCheckpoint early;
                        early.Epoch = epoch; early.Build = cutTestBuild; early.RawClock = nds->GetSysTimestamp();
                        early.Frame = frame; early.Player = host ? 0 : 1; early.Rom = cutOwnRom;
                        ambient->GetHandoffFenceToken(early.FenceToken);
                        const Firmware& fw = nds->GetFirmware();
                        early.Firmware.assign(fw.Buffer(), fw.Buffer() + fw.Length());
                        early.State = earlyState;
                        std::string sendError;
                        if (!cutLink->SendCheckpoint(early, sendError))
                            printf("CUTFENCE %s early checkpoint not sent: %s\n", cutRole, sendError.c_str());
                    }
                    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
                    const char* why = "8 s deadline";
                    while (!ambient->HandoffFenceComplete() && !ambient->HandoffFenceAborted() &&
                           std::chrono::steady_clock::now() < deadline)
                    {
                        ambient->Process();
                        if (ambient->HandoffFenceAborted()) why = ambient->GetStats().Links ? "peer aborted or link lost" : "no Ambient link";
                        // A peer can have an MP packet in flight when our fence is sent.
                        // Process() retires stale packets under Ambient's normal rules;
                        // the READY and completion checks still require an empty queue
                        // and full sequence-watermark retirement. Aborting on the first
                        // transient queued packet races the peer's fence response.
                        if (nds->Wifi.HandoffBusyMask() || !ambient->GetStats().Links)
                        { why = nds->Wifi.HandoffBusyMask() ? "MP exchange busy" : "no Ambient link"; ambient->AbortHandoffFence(); break; }
                        if (getenv("LITEV_AMBIENT_FENCE_TRACE"))
                        {
                            static auto lastTrace = std::chrono::steady_clock::now();
                            if (std::chrono::steady_clock::now() - lastTrace > std::chrono::milliseconds(200))
                            {
                                lastTrace = std::chrono::steady_clock::now();
                                const auto fs = ambient->GetHandoffFenceStats();
                                printf("CUTFENCE %s t=%ldms fencing: local=%u retired=%u peer=%u peerRetired=%u ready=%d/%d queued=%zu peerFence=%d\n",
                                       cutRole, frozenMs(), fs.LastSent, fs.RetiredThrough, fs.PeerLastSent, fs.PeerRetiredThrough,
                                       fs.LocalReady, fs.PeerReady, ambient->HandoffQueuedFrames(), ambient->HasPeerHandoffFence());
                            }
                        }
                        if (ambient->CanAcknowledgeHandoffFence())
                        {
                            if (const char* fail = getenv("LITEV_AMBIENT_CUT_INJECT_FAIL"); fail && strcmp(fail, cutRole) == 0)
                            {
                                printf("CUTFENCE %s injected failure before READY\n", cutRole);
                                ambient->AbortHandoffFence();
                                break;
                            }
                            ambient->AcknowledgeHandoffFence();
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    }
                    if (!ambient->HandoffFenceComplete() || ambient->HandoffFenceAborted() ||
                        nds->Wifi.HandoffBusyMask() || ambient->HandoffQueuedFrames())
                    {
                        const auto fs = ambient->GetHandoffFenceStats();
                        printf("CUTFENCE %s abort f%d t=%ldms (%s) busy=%02x queued=%zu peerFence=%d ready=%d\n", cutRole, frame,
                               frozenMs(), why, nds->Wifi.HandoffBusyMask(), ambient->HandoffQueuedFrames(),
                               ambient->HasPeerHandoffFence(), ambient->CanAcknowledgeHandoffFence());
                        printf("CUTFENCE watermark local=%u retired=%u peer=%u peerRetired=%u ready=%d/%d aborted=%d\n",
                               fs.LastSent, fs.RetiredThrough, fs.PeerLastSent, fs.PeerRetiredThrough,
                               fs.LocalReady, fs.PeerReady, fs.Aborted);
                        ambient->AbortHandoffFence();
                        mark(abort);
                        if (cutFenceLoop)
                        {
                            cutLoopFail++;
                            printf("CUTLOOP fail attempt=%d (%s) local=%u retired=%u peer=%u\n", cutAttempt, why,
                                   fs.LastSent, fs.RetiredThrough, fs.PeerLastSent);
                            if (cutAttempt >= cutFenceLoop) printf("CUTLOOP done ok=%d fail=%d\n", cutLoopOk, cutLoopFail);
                            fflush(stdout);
                        }
                    }
                    else if (cutFenceLoop)
                    {
                        // the peer completes within a round trip of us: the host lets it see READY, then aborts
                        // (the deliberate end of this loop step); the client waits for that abort
                        cutLoopOk++;
                        printf("CUTLOOP ok attempt=%d t=%ldms f%d\n", cutAttempt, frozenMs(), frame);
                        const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(host ? 300 : 5000);
                        while (!ambient->HandoffFenceAborted() && std::chrono::steady_clock::now() < until)
                        { ambient->Process(); std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
                        ambient->AbortHandoffFence();   // (both send one: the mutual reset needs both)
                        cutLink.reset();
                        if (cutAttempt >= cutFenceLoop)
                        {
                            printf("CUTLOOP done ok=%d fail=%d\n", cutLoopOk, cutLoopFail);
                            fflush(stdout);
                        }
                    }
                    else
                    {
                        // A real checkpoint exchange can keep both source
                        // games frozen for longer than ordinary Ambient's
                        // 10-second idle timeout. Pump only control traffic
                        // and prove the fenced link survives that interval.
                        if (const char* hold = getenv("LITEV_AMBIENT_CUT_FROZEN_MS"))
                        {
                            const int ms = std::clamp(atoi(hold), 0, 60000);
                            const auto until = std::chrono::steady_clock::now() +
                                               std::chrono::milliseconds(ms);
                            while (std::chrono::steady_clock::now() < until)
                            {
                                ambient->Process();
                                if (!ambient->GetStats().Links || ambient->HandoffFenceAborted() ||
                                    !ambient->HandoffFenceComplete())
                                { fprintf(stderr, "CUTFENCE fenced link expired during frozen hold\n"); return 2; }
                                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                            }
                            printf("CUTFENCE %s frozen link survived %d ms\n", cutRole, ms);
                        }
                        const auto fs = ambient->GetHandoffFenceStats();
                        const std::string path = getenv("LITEV_AMBIENT_CUT_NO_DUMP") ? "" :
                            std::string(cutDir) + (host ? "/host.mln" : "/client.mln");
                        std::vector<u8> liveState;
                        // The handoff checkpoint carries an empty JIT cache and empties this
                        // console's too: every copy on every build starts from the same cache.
                        // (LITEV_HANDOFF_WARM_CHECKPOINT=1: the old warm-JIT checkpoint, negative control)
                        const bool coldCheckpoint = cutXferPort && !getenv("LITEV_HANDOFF_WARM_CHECKPOINT");
                        if (!earlyState.empty())
                        {
                            liveState = std::move(earlyState);   // (captured at the freeze, already sent)
                            if (!path.empty()) { FILE* f = fopen(path.c_str(), "wb"); if (f) { fwrite(liveState.data(), 1, liveState.size(), f); fclose(f); } }
                        }
                        else if (!SaveSavestate(*nds, path, cutXferPort ? &liveState : nullptr, coldCheckpoint)) { mark(abort); return 2; }
                        printf("CUTFENCE %s t=%ldms saved f%d busy=%02x queued=%zu\n", cutRole, frozenMs(), frame,
                               nds->Wifi.HandoffBusyMask(), ambient->HandoffQueuedFrames());
                        printf("CUTFENCE watermark local=%u retired=%u peer=%u peerRetired=%u ready=%d/%d aborted=%d\n",
                               fs.LastSent, fs.RetiredThrough, fs.PeerLastSent, fs.PeerRetiredThrough,
                               fs.LocalReady, fs.PeerReady, fs.Aborted);
                        fflush(stdout);
                        bool transferFailed = false;
                        // A successful PREPARED vote must retain the actually loaded
                        // replica, not merely prove that a temporary load once worked.
                        // Keep userdata before NDS so cart/save teardown cannot use a
                        // destroyed save-path context.
                        std::unique_ptr<HeadlessHost::InstanceUserData> preparedUser;
                        std::unique_ptr<NDS> preparedReplica;
                        std::unique_ptr<HeadlessHost::InstanceUserData> preparedLocalUser;
                        std::unique_ptr<NDS> preparedLocalReplica;
                        NetplayLiveCheckpoint local, remote;
                        auto prepareScratch = [&](const NetplayLiveCheckpoint& candidate,
                                                  const std::string& romPath,
                                                  std::unique_ptr<HeadlessHost::InstanceUserData>& user,
                                                  std::unique_ptr<NDS>& instance,
                                                  std::string& reason) {
                            return PrepareScratch(candidate, romPath, opt.jit, user, instance, reason);
                        };
                        if (cutXferPort)
                        {
                            local.Epoch = epoch;
                            local.Build = cutTestBuild;
                            local.RawClock = nds->GetSysTimestamp();
                            local.Frame = frame;
                            local.Player = host ? 0 : 1;
                            if (!ambient->GetHandoffFenceToken(local.FenceToken))
                            { ambient->AbortHandoffFence(); mark(abort); return 2; }
                            local.Rom = cutOwnRom;
                            {
                                const Firmware& fw = nds->GetFirmware();
                                local.Firmware.assign(fw.Buffer(), fw.Buffer() + fw.Length());
                            }
                            local.State = std::move(liveState);
                            std::string transferError;
                            std::atomic<bool> transferDone{false};
                            std::atomic<bool> transferCancel{false};
                            bool transferOk = false;
                            std::thread transfer([&] {
                                const std::string address = cutPeerIP + ":" + std::to_string(cutXferPort);
                                auto prepare = [&](const NetplayLiveCheckpoint& candidate, std::string& reason) {
                                    const char* fail = getenv("LITEV_HANDOFF_XFER_PREPARE_FAIL");
                                    if (fail && strcmp(fail, cutRole) == 0)
                                    { reason = "injected post-transfer prepare failure"; return false; }
                                    const char* peerRomPath = getenv("LITEV_AMBIENT_CUT_PEER_ROM");
                                    return peerRomPath && prepareScratch(candidate, peerRomPath,
                                                                         preparedUser, preparedReplica, reason);
                                };
                                (void)address;
                                transferOk = cutLink && cutLink->ExchangeCheckpoints(local, cutPeerRom, prepare, transferCancel,
                                                                                     remote, transferError);
                                transferDone.store(true, std::memory_order_release);
                            });
                            // meanwhile, this console's own scratch copy (the proof's and the trial's)
                            std::string localPrepError;
                            bool localPrepared = false;
                            while (!transferDone.load(std::memory_order_acquire))
                            {
                                ambient->Process();
                                if (!localPrepared)
                                {
                                    localPrepared = true;
                                    if (!prepareScratch(local, opt.rom, preparedLocalUser, preparedLocalReplica, localPrepError))
                                    { preparedLocalReplica.reset(); preparedLocalUser.reset(); }
                                    nds->JIT.JitEnableExecute();
                                }
                                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                            }
                            transfer.join();
                            if (!transferOk || ambient->HandoffFenceAborted() || ambient->HandoffQueuedFrames())
                            {
                                preparedReplica.reset();
                                preparedUser.reset();
                                ambient->AbortHandoffFence();
                                mark(abort);
                                printf("CUTFENCE %s transfer abort: %s\n", cutRole, transferError.c_str());
                                transferFailed = true;
                            }
                            else
                            {
                                const char* inject = getenv("LITEV_HANDOFF_PROBE_FAIL_PHASE");
                                std::string localError;
                                if ((inject && std::string(inject) == std::string(cutRole) + ":localprepare") ||
                                    (!preparedLocalReplica &&
                                     !prepareScratch(local, opt.rom, preparedLocalUser, preparedLocalReplica, localError)))
                                {
                                    if (localError.empty())
                                    { localError = "injected localprepare failure"; printf("CUTFENCE %s %s\n", cutRole, localError.c_str()); }
                                    ambient->AbortHandoffFence();
                                    mark(abort);
                                    transferFailed = true;
                                    printf("CUTFENCE %s local scratch abort: %s\n", cutRole, localError.c_str());
                                }
                            }
                            if (!transferFailed)
                            {
                                if (!getenv("LITEV_AMBIENT_CUT_NO_DUMP"))
                                {
                                    const std::string remotePath = std::string(cutDir) + (host ? "/remote-client.mln" : "/remote-host.mln");
                                    FILE* f = fopen(remotePath.c_str(), "wb");
                                    if (!f || fwrite(remote.State.data(), 1, remote.State.size(), f) != remote.State.size())
                                    { if (f) fclose(f); ambient->AbortHandoffFence(); mark(abort); return 2; }
                                    fclose(f);
                                    auto writeFirmware = [&](const std::string& fwPath, const std::vector<u8>& bytes) {
                                        FILE* out = fopen(fwPath.c_str(), "wb");
                                        if (!out) return false;
                                        const bool wrote = fwrite(bytes.data(), 1, bytes.size(), out) == bytes.size();
                                        fclose(out);
                                        return wrote;
                                    };
                                    const std::string ownFwPath = std::string(cutDir) + (host ? "/host.fw" : "/client.fw");
                                    const std::string peerFwPath = std::string(cutDir) + (host ? "/remote-client.fw" : "/remote-host.fw");
                                    if (!writeFirmware(ownFwPath, local.Firmware) || !writeFirmware(peerFwPath, remote.Firmware))
                                    { ambient->AbortHandoffFence(); mark(abort); return 2; }
                                }
                                Firmware peerFirmware(remote.Firmware.data(), (u32)remote.Firmware.size());
                                const auto& peerMac = peerFirmware.GetHeader().MacAddr;
                                printf("CUTFENCE %s t=%ldms transfer verified peer=%u frame=%u clock=%llu bytes=%zu\n",
                                       cutRole, frozenMs(), remote.Player, remote.Frame,
                                       (unsigned long long)remote.RawClock, remote.State.size());
                                printf("CUTFENCE %s peer firmware=%zu MAC=%02x:%02x:%02x:%02x:%02x:%02x\n",
                                       cutRole, remote.Firmware.size(), peerMac[0], peerMac[1], peerMac[2],
                                       peerMac[3], peerMac[4], peerMac[5]);
                            }
                            fflush(stdout);
                        }
                        // The reversible pre-commit proof, as the app runs it: fresh copies of
                        // both checkpoints run two Lockstep frames here and on the peer; the
                        // live-start rendezvous below compares the four state hashes.
                        std::array<u64, 4> proof{};
                        if (!transferFailed && cutXferPort)
                        {
                            // the proof runs the two prepared copies (as the app does), which then load
                            // their checkpoints again for the session
                            std::string proofError;
                            const int me = host ? 0 : 1;
                            NDS* copies[2];
                            HeadlessHost::InstanceUserData* users[2];
                            copies[me] = preparedLocalReplica.get(); users[me] = preparedLocalUser.get();
                            copies[1 - me] = preparedReplica.get(); users[1 - me] = preparedUser.get();
                            bool proven = copies[0] && copies[1];
                            if (proven)
                            {
                                // (headless MP callbacks route by the userdata's instance id)
                                users[0]->instanceID = 0;
                                users[1]->instanceID = 1;
                                proven = NetplayRunLiveProof({copies[0], copies[1]},
                                    {me == 0 ? local.RawClock : remote.RawClock, me == 1 ? local.RawClock : remote.RawClock},
                                    [&](int p) { copies[p]->RunFrame(); }, proof, proofError);
                                for (int p = 0; p < 2 && proven; p++)
                                {
                                    const NetplayLiveCheckpoint& c = p == me ? local : remote;
                                    Savestate st((void*)c.State.data(), (u32)c.State.size(), false);
                                    if (st.Error || !copies[p]->DoSavestate(&st) || st.Error || copies[p]->GetSysTimestamp() != c.RawClock)
                                    { proven = false; proofError = "a proof copy did not load its checkpoint again"; }
                                }
                            }
                            else proofError = "no prepared copies for the proof";
                            nds->JIT.JitEnableExecute();   // (macOS: scratch JIT teardown left this thread writing)
                            if (!proven)
                            {
                                ambient->AbortHandoffFence();
                                mark(abort);
                                transferFailed = true;
                                printf("CUTFENCE %s proof abort: %s\n", cutRole, proofError.c_str());
                            }
                            else
                                printf("CUTFENCE %s t=%ldms proof p0=%016llx/%016llx p1=%016llx/%016llx\n", cutRole, frozenMs(),
                                       (unsigned long long)proof[0], (unsigned long long)proof[2],
                                       (unsigned long long)proof[1], (unsigned long long)proof[3]);
                            fflush(stdout);
                        }
                        // (the prepare votes travel with the checkpoints on the live link; the start
                        // manifest below is the only step left: no Ambient PREPARED/ARMED/GO round trips)
                        if (!transferFailed && cutXferPort)
                        {
                            const char* inject = getenv("LITEV_HANDOFF_PROBE_FAIL_PHASE");
                            if (inject && std::string(inject) == std::string(cutRole) + ":prepared" &&
                                (!cutRetryAfterAbort || cutAttempt == 1))
                            {
                                printf("CUTFENCE %s injected prepared failure\n", cutRole);
                                ambient->AbortHandoffFence();
                                mark(abort);
                                transferFailed = true;
                                cutLink.reset();   // (the peer waits on the link: closing it ends that wait)
                            }
                        }
                        if (!transferFailed && cutXferPort)
                        {
                            // GO is only a vote, not authority to run a frame: one peer
                            // can receive GO while the sender fails immediately after.
                            // Rendezvous over a second, checkpoint-bound connection while
                            // both original Ambient consoles remain frozen and recoverable.
                            std::atomic<bool> startDone{false};
                            std::atomic<bool> startCancel{false};
                            bool startOk = false;
                            std::string startError;
                            std::thread start([&] {
                                startOk = cutLink && cutLink->ExchangeStart(local, remote, proof, startCancel, startError);
                                startDone.store(true, std::memory_order_release);
                            });
                            while (!startDone.load(std::memory_order_acquire))
                            {
                                ambient->Process();
                                if (ambient->HandoffFenceAborted() || !ambient->GetStats().Links ||
                                    ambient->HandoffQueuedFrames()) startCancel.store(true, std::memory_order_release);
                                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                            }
                            start.join();
                            if (!startOk || ambient->HandoffFenceAborted() || !ambient->GetStats().Links)
                            {
                                ambient->AbortHandoffFence();
                                mark(abort);
                                transferFailed = true;
                                printf("CUTFENCE %s live-start abort: %s\n", cutRole, startError.c_str());
                            }
                            else printf("CUTFENCE %s t=%ldms live-start rendezvous agreed\n", cutRole, frozenMs());
                            fflush(stdout);
                        }
                        if (!transferFailed && cutSwapTrialFrames > 0)
                        {
                            // Test-only reversible backend trial: keep the original Ambient
                            // console and ENet host alive while independent scratch replicas
                            // run a short Lockstep session, then restore Ambient without a
                            // ROM restart or writing the owner's save.
                            const u64 sourceClock = nds->GetSysTimestamp();
                            const u64 sourceRAM = XXH3_64bits(nds->MainRAM, nds->MainRAMMask + 1);
                            setenv("LITEV_MP_HANDOFF_PROBE", "1", 1);
                            setenv("LITEV_MP_HANDOFF_STARTUP_GATE", "1", 1);
                            char epochText[32], tokenText[33];
                            snprintf(epochText, sizeof(epochText), "%llu", (unsigned long long)local.Epoch);
                            for (int i = 0; i < 16; i++) snprintf(tokenText + i * 2, 3, "%02x", local.FenceToken[i]);
                            setenv("LITEV_MP_HANDOFF_EPOCH", epochText, 1);
                            setenv("LITEV_MP_HANDOFF_TOKEN", tokenText, 1);
                            const int startPort = (int)cutXferPort + 50 + (cutExternalPeer ? 0 : (host ? 0 : 1));
                            const int peerStartPort = (int)cutXferPort + 50 + (cutExternalPeer ? 0 : (host ? 1 : 0));
                            const std::string startPortText = std::to_string(startPort);
                            const std::string startPeerText =
                                (cutExternalPeer ? cutPeerIP : "127.0.0.1") + ":" + std::to_string(peerStartPort);
                            setenv("LITEV_MP_HANDOFF_START_PORT", startPortText.c_str(), 1);
                            setenv("LITEV_MP_HANDOFF_START_PEER", startPeerText.c_str(), 1);
                            if (getenv("LITEV_AMBIENT_CUT_STALE_FIRST_INPUT") &&
                                *getenv("LITEV_AMBIENT_CUT_STALE_FIRST_INPUT"))
                                setenv("LITEV_MP_INJECT_STALE_INPUT", "1", 1);
                            liteds::TraceRunConfig trialCfg;
                            const std::string peerRomPath = getenv("LITEV_AMBIENT_CUT_PEER_ROM");
                            trialCfg.rom = host ? opt.rom : peerRomPath;
                            trialCfg.roms = {host ? opt.rom : peerRomPath,
                                             host ? peerRomPath : opt.rom};
                            trialCfg.dataDir = opt.dataDir;
                            trialCfg.jit = opt.jit;
                            trialCfg.fixedRtcEpoch = opt.fixedRtc;
                            auto savedAmbient = melonDS::MPInterface::Take();
                            printf("CUTFENCE %s t=%ldms swap trial begin %d frames\n", cutRole, frozenMs(), cutSwapTrialFrames);
                            fflush(stdout);
                            std::atomic<bool> keepAmbient{true};
                            std::thread ambientService([&] {
                                while (keepAmbient.load(std::memory_order_acquire))
                                { ambient->Process(); std::this_thread::sleep_for(std::chrono::milliseconds(2)); }
                            });
                            std::vector<liteds::PreparedMPConsole> replicas(2);
                            auto& own = replicas[host ? 0 : 1];
                            // LITEV_AMBIENT_CUT_LIVE_LOCAL=1: the console that kept running plays
                            // on, as in the app (only the peer's console is a checkpoint copy).
                            // There is no source left to roll back to: the process exits after.
                            const bool liveLocal = getenv("LITEV_AMBIENT_CUT_LIVE_LOCAL") && *getenv("LITEV_AMBIENT_CUT_LIVE_LOCAL");
                            if (liveLocal)
                            {
                                preparedLocalReplica.reset();
                                preparedLocalUser.reset();
                                nds->JIT.JitEnableExecute();
                                own.udata = std::make_unique<HeadlessHost::InstanceUserData>();
                                nds->UserData = own.udata.get();
                                own.identity = nds.get();
                                own.nds = std::move(nds);
                            }
                            else
                            {
                                own.identity = preparedLocalReplica.get();
                                own.udata = std::move(preparedLocalUser);
                                own.nds = std::move(preparedLocalReplica);
                            }
                            auto& other = replicas[host ? 1 : 0];
                            other.identity = preparedReplica.get();
                            other.udata = std::move(preparedUser);
                            other.nds = std::move(preparedReplica);
                            printf("CUTFENCE %s trial exact local=%p peer=%p\n", cutRole,
                                   (void*)own.identity, (void*)other.identity);
                            fflush(stdout);
                            // LITEV_AMBIENT_CUT_TRIAL_SCRIPT: this player's input script for the trial
                            std::vector<std::string> trialScripts(2);
                            if (const char* ts = getenv("LITEV_AMBIENT_CUT_TRIAL_SCRIPT")) trialScripts[host ? 0 : 1] = ts;
                            {   // (MPTest stamps the first Netplay frame against the freeze)
                                const auto t0 = std::chrono::duration_cast<std::chrono::microseconds>(cutFreezeStart.time_since_epoch()).count();
                                setenv("LITEV_CUT_T0_US", std::to_string(t0).c_str(), 1);
                            }
                            liteds::LiveExitResult ended;
                            const int trialResult = liteds::MPTest(trialCfg, cutSwapTrialFrames, trialScripts, std::move(replicas),
                                                                   liveLocal ? &ended : nullptr);
                            keepAmbient.store(false, std::memory_order_release);
                            ambientService.join();
                            if (liveLocal && ended.EndsAfter >= 0)
                            {
                                // The return path (LITEV_NP_LIVE_EXIT): the session ended by itself. This
                                // device's console - the very object that played on - plays on alone in
                                // Ambient; the peer's copy is dropped (never saved). Its inputs: the ones
                                // Netplay had already scheduled (the input delay), then live (the trial
                                // script, same frame numbers). LITEV_AMBIENT_CUT_EXIT_FRAMES of them.
                                const auto returned = std::chrono::steady_clock::now();
                                const int me = host ? 0 : 1, F = ended.EndsAfter;
                                auto& mine = ended.Consoles[me];
                                // the reference: both consoles as they were after frame F, for the frames
                                // an uninterrupted session would have run with the inputs already sent
                                std::array<NetplayLiveCheckpoint, 2> atEnd;
                                for (int k = 0; k < 2; k++)
                                {
                                    Savestate st;
                                    ended.Consoles[k].nds->DoSavestate(&st);
                                    atEnd[k].Player = (u8)k;
                                    atEnd[k].State.assign((const u8*)st.Buffer(), (const u8*)st.Buffer() + st.Length());
                                    const Firmware& fw = ended.Consoles[k].nds->GetFirmware();
                                    atEnd[k].Firmware.assign(fw.Buffer(), fw.Buffer() + fw.Length());
                                }
                                const auto snapped = std::chrono::steady_clock::now();   // (test-only work ends)
                                // the session's link lets go of every console first (its hooks on ours would call into
                                // the freed link the next time the game does MP)
                                mine.nds->DetachMPLink();
                                // the other consoles and the Netplay link are released on a worker, as the app does (their
                                // destructors are tens of ms under load; nothing there touches this console)
                                std::vector<std::decay_t<decltype(ended.Consoles[0])>> dropped;
                                for (int k = 0; k < (int)ended.Consoles.size(); k++)
                                    if (k != me) dropped.push_back(std::move(ended.Consoles[k]));
                                std::thread dropper([d = std::move(dropped), l = melonDS::MPInterface::Take()]() mutable { d.clear(); l.reset(); });
                                mine.nds->JIT.JitEnableExecute();
                                mine.udata->instanceID = 0;
                                melonDS::MPInterface::Set(std::move(savedAmbient), melonDS::MPInterface_Ambient);
                                ambient->EndCommittedHandoff();
                                if (mine.nds->Wifi.IsPoweredOn()) melonDS::MPInterface::Get().Begin(0);
                                liteds::InputScript& live = *ended.Scripts[me];   // (the trial's, as far as it got)
                                const auto& sched = ended.Scheduled[me];
                                const int exitFrames = getenv("LITEV_AMBIENT_CUT_EXIT_FRAMES") ? atoi(getenv("LITEV_AMBIENT_CUT_EXIT_FRAMES")) : 600;
                                double pauseMs = 0, worstMs = 0;
                                std::vector<u64> realHash;
                                const u64 exchBefore = ambient->GetStats().Exchanges;
                                auto pace = std::chrono::steady_clock::now();   // solo: 60 fps from the first frame (no catch-up)
                                // (from the end of the session's own teardown: the waits before it - the peer's copy
                                // reaching the last frame, the peer's scheduled inputs - exist only for the
                                // reference below; a device drops its copies of the others at once)
                                const double testWaitMs = std::chrono::duration<double, std::milli>(returned - ended.StoppedAt[me]).count();
                                auto last = returned;
                                for (int f = F + 1; f <= F + exitFrames; f++)
                                {
                                    const auto start = std::chrono::steady_clock::now();
                                    const double gap = std::chrono::duration<double, std::milli>(start - last).count();
                                    if (f == F + 1)   // the pause without the test-only snapshot of both consoles
                                        pauseMs = gap - std::chrono::duration<double, std::milli>(snapped - returned).count();
                                    else worstMs = std::max(worstMs, gap);
                                    const int i = f - F - 1;
                                    NetplayFrameInput in;
                                    if (i < (int)sched.size()) in = sched[i];
                                    else
                                    {
                                        in.Keys = live.Loaded() ? live.KeyMaskForFrame(f) : 0xFFF;
                                        int tx, ty;
                                        if (live.HasTouch() && live.TouchForFrame(f, tx, ty)) { in.TouchX = tx; in.TouchY = ty; }
                                    }
                                    mine.nds->SetKeyMask(in.Keys);
                                    if (in.TouchX >= 0) mine.nds->TouchScreen(in.TouchX, in.TouchY);
                                    else mine.nds->ReleaseScreen();
                                    mine.nds->RunFrame();
                                    if ((f - F) % 120 == 0)
                                    { printf("AMBIENT after exit f%d: %s\n", f, ambient->DebugLine().c_str()); fflush(stdout); }
                                    liteds::AudioTapDrain(*mine.nds, 2, f);
                                    if (i < (int)sched.size()) realHash.push_back(NetplayLiveProofHash(*mine.nds));
                                    if (live.HasProgram())   // closed-loop script steps see this frame's screens
                                    {
                                        void* top = nullptr; void* bot = nullptr;
                                        if (mine.nds->GPU.GetFramebuffers(&top, &bot) && top && bot) live.OnFrame(f, (const u32*)top, (const u32*)bot);
                                        else live.OnFrame(f, nullptr, nullptr);
                                    }
                                    // LITEV_AMBIENT_CUT_EXIT_DUMP=<every>: screens of the continuation (cut dir)
                                    if (static const int every = getenv("LITEV_AMBIENT_CUT_EXIT_DUMP") ? atoi(getenv("LITEV_AMBIENT_CUT_EXIT_DUMP")) : 0;
                                        every > 0 && (f - F) % every == 0)
                                    {
                                        void* top = nullptr; void* bot = nullptr;
                                        if (mine.nds->GPU.GetFramebuffers(&top, &bot) && top && bot)
                                            DumpPPM(std::string(cutDir) + "/exit-" + cutRole + "-f" + std::to_string(f) + ".ppm", (const u32*)top, (const u32*)bot);
                                    }
                                    ambient->Process();
                                    pace += std::chrono::microseconds(16715);
                                    std::this_thread::sleep_until(pace);
                                    if (f <= F + 64)
                                        printf("inst%d state f%d: sys=%llu ram=%016llx r9=%016llx\n", me, f,
                                               (unsigned long long)mine.nds->GetSysTimestamp(),
                                               (unsigned long long)XXH3_64bits(mine.nds->MainRAM, mine.nds->MainRAMMask + 1),
                                               (unsigned long long)XXH3_64bits(mine.nds->ARM9.R, sizeof(mine.nds->ARM9.R)));
                                    last = std::chrono::steady_clock::now();
                                }
                                dropper.join();
                                // the uninterrupted run, in-process: both copies, the same link clocks
                                int same = -1;
                                const size_t n = std::min(ended.Scheduled[0].size(), ended.Scheduled[1].size());
                                if (n)
                                {
                                    std::unique_ptr<HeadlessHost::InstanceUserData> ru[2];
                                    std::unique_ptr<NDS> rn[2];
                                    std::string err;
                                    const char* peerRomPath = getenv("LITEV_AMBIENT_CUT_PEER_ROM");
                                    const bool ok = PrepareScratch(atEnd[me], opt.rom, opt.jit, ru[me], rn[me], err) &&
                                                    PrepareScratch(atEnd[1 - me], peerRomPath, opt.jit, ru[1 - me], rn[1 - me], err);
                                    std::vector<u64> refHash;
                                    if (ok)
                                    {
                                        ru[0]->instanceID = 0; ru[1]->instanceID = 1;
                                        auto saved = melonDS::MPInterface::Take();
                                        const std::array<u64, 2> origins {me == 0 ? local.RawClock : remote.RawClock,
                                                                          me == 1 ? local.RawClock : remote.RawClock};
                                        NetplayRunLockstepFrames({rn[0].get(), rn[1].get()}, origins, [&](int p, int f) {
                                            const NetplayFrameInput in = f < (int)n ? ended.Scheduled[p][f] : NetplayFrameInput{};
                                            rn[p]->SetKeyMask(in.Keys);
                                            if (in.TouchX >= 0) rn[p]->TouchScreen(in.TouchX, in.TouchY); else rn[p]->ReleaseScreen();
                                            rn[p]->RunFrame();
                                        }, (int)n, refHash, err);
                                        melonDS::MPInterface::Take().reset();
                                        melonDS::MPInterface::Set(std::move(saved), melonDS::MPInterface_Ambient);
                                        same = 0;
                                        for (size_t f = 0; f < n && f < realHash.size(); f++) same += refHash[f * 2 + me] == realHash[f];
                                    }
                                    for (int k = 0; k < 2; k++) { rn[k].reset(); ru[k].reset(); }
                                    mine.nds->JIT.JitEnableExecute();
                                }
                                printf("CUTFENCE %s live exit after f%d: back in Ambient, pause %.1f ms (worst later frame gap %.1f ms), "
                                       "%zu scheduled inputs applied, then live; link=%u; frames after the exit equal to an "
                                       "uninterrupted session: %d of %zu; Ambient exchanges after the exit: %llu (test-only waits before: %.1f ms)\n", cutRole, F, pauseMs, worstMs, sched.size(),
                                       ambient->GetStats().Links, same, n, (unsigned long long)(ambient->GetStats().Exchanges - exchBefore), testWaitMs);
                                fflush(stdout);
                                return trialResult == 0 ? 0 : 2;
                            }
                            if (cutExternalPeer || liveLocal)
                            {
                                // The Android peer has committed. This bounded headless
                                // participant must not roll its source back into Ambient.
                                // Exit after the requested real cross-device Netplay frames.
                                printf("CUTFENCE external peer trial %s frames=%d\n",
                                       trialResult == 0 ? "pass" : "failed", cutSwapTrialFrames);
                                fflush(stdout);
                                return trialResult == 0 ? 0 : 2;
                            }
                            auto trialLink = melonDS::MPInterface::Take();
                            trialLink.reset();
                            melonDS::MPInterface::Set(std::move(savedAmbient), melonDS::MPInterface_Ambient);
                            preparedReplica.reset();
                            preparedUser.reset();
                            // macOS MAP_JIT write protection is per-thread, not per-console.
                            // Destroying the scratch JIT leaves this thread in write mode;
                            // the paused source console will execute its own JIT on resume.
                            nds->JIT.JitEnableExecute();
                            const bool sourceIntact = nds->GetSysTimestamp() == sourceClock &&
                                XXH3_64bits(nds->MainRAM, nds->MainRAMMask + 1) == sourceRAM;
                            printf("CUTFENCE %s swap trial %s sourceIntact=%d link=%u\n", cutRole,
                                   trialResult == 0 ? "pass" : "failed", sourceIntact, ambient->GetStats().Links);
                            fflush(stdout);
                            ambient->AbortHandoffFence();
                            mark(abort);
                            transferFailed = true; // trial deliberately rolls back to original Ambient
                            const char* injected = getenv("LITEV_HANDOFF_PROBE_FAIL_PHASE");
                            const bool expectedAfterLiveAbort = injected && strcmp(injected, "host:afterlive") == 0;
                            if ((!expectedAfterLiveAbort && trialResult != 0) ||
                                !sourceIntact || !ambient->GetStats().Links) return 2;
                        }
                        preparedReplica.reset();
                        preparedUser.reset();
                        preparedLocalReplica.reset();
                        preparedLocalUser.reset();
                        nds->JIT.JitEnableExecute();
                        if (!transferFailed) break;
                    }
                }
            }
        }
#endif

        if (haveWindow && frame == opt.benchWindowEnd)
        {
            windowEnd = std::chrono::steady_clock::now();
            clock_gettime(CLOCK_THREAD_CPUTIME_ID, &windowCpuEnd);
        }

        // diagnostics LITEV_WIFI_TRACE=<every>: the Wi-Fi chip's MP state (live handoff exit rule)
        if (static const int wt = getenv("LITEV_WIFI_TRACE") ? atoi(getenv("LITEV_WIFI_TRACE")) : 0; wt > 0 && frame % wt == 0)
            printf("WIFISTATE f%d on=%d mp=%d client=%d exch=%u busy=%02x\n", frame, nds->Wifi.IsPoweredOn(), nds->Wifi.IsMultiplayer(),
                   nds->Wifi.IsMPClientMode(), nds->Wifi.MPExchanges, nds->Wifi.HandoffBusyMask());
        if (opt.dumpSavestateFrame == frame && !opt.dumpSavestatePath.empty())
        {
            if (!SaveSavestate(*nds, opt.dumpSavestatePath))
                fprintf(stderr, "warning: savestate dump failed at frame %d\n", frame);
        }

#if LITEV_PROFILE
        // Window-scoped when --bench-window is set: only fold frames inside the
        // window into the decomposition totals. Otherwise fold every frame.
        if (!haveWindow ||
            (frame >= opt.benchWindowStart && frame <= opt.benchWindowEnd))
        {
            using namespace melonDS::LiteProfile;
            profFrames++;
            profTotals.runFrameNs  += g_Frame.RunFrameNs.load(std::memory_order_relaxed);
            profTotals.dma9Ns      += g_Frame.DMA9Ns.load(std::memory_order_relaxed);
            profTotals.dma7Ns      += g_Frame.DMA7Ns.load(std::memory_order_relaxed);
            profTotals.gxCommands  += g_Frame.GXCommands.load(std::memory_order_relaxed);
            profTotals.lightingCalls += g_Frame.LightingCalls.load(std::memory_order_relaxed);
            profTotals.linksPatched    += g_Frame.LinksPatched.load(std::memory_order_relaxed);
            profTotals.linksUnlinked   += g_Frame.LinksUnlinked.load(std::memory_order_relaxed);
            profTotals.cppReentries    += g_Frame.CppReentries.load(std::memory_order_relaxed);
            profTotals.dispatcherMisses+= g_Frame.DispatcherMisses.load(std::memory_order_relaxed);
            profTotals.dispatcherHits  += g_Frame.DispatcherHits.load(std::memory_order_relaxed);
            profTotals.icacheHits      += g_Frame.ICacheHits.load(std::memory_order_relaxed);
            profTotals.rasHits         += g_Frame.RasHits.load(std::memory_order_relaxed);
#ifdef LITEV_JIT_DIRECTPATCH
            profTotals.directGuardHits   += g_Frame.DirectGuardHits.load(std::memory_order_relaxed);
            profTotals.directGuardMisses += g_Frame.DirectGuardMisses.load(std::memory_order_relaxed);
            profTotals.directPromotions  += g_Frame.DirectPromotions.load(std::memory_order_relaxed);
            profTotals.directDemotions   += g_Frame.DirectDemotions.load(std::memory_order_relaxed);
            profTotals.directReverts     += g_Frame.DirectReverts.load(std::memory_order_relaxed);
#endif
            profTotals.linkSitesEmitted += g_Frame.LinkSitesEmitted.load(std::memory_order_relaxed);
            profTotals.dispatchOnlyExits+= g_Frame.DispatchOnlyExits.load(std::memory_order_relaxed);
            profTotals.schedIterations  += g_Frame.SchedulerIterations.load(std::memory_order_relaxed);
            profTotals.schedEventsFired += g_Frame.SchedulerEventsFired.load(std::memory_order_relaxed);
            for (int e = 0; e < 32; e++)
                schedByType[e] += g_Frame.SchedEventByType[e].load(std::memory_order_relaxed);
            profTotals.arm9ExecNs   += g_Frame.ARM9ExecNs.load(std::memory_order_relaxed);
            profTotals.arm7ExecNs   += g_Frame.ARM7ExecNs.load(std::memory_order_relaxed);
            profTotals.gpu3dNs      += g_Frame.GPU3DNs.load(std::memory_order_relaxed);
            profTotals.runSystemNs  += g_Frame.RunSystemNs.load(std::memory_order_relaxed);
            profTotals.spuMixNs     += g_Frame.SPUMixNs.load(std::memory_order_relaxed);
            profTotals.arm9IdleHits += g_Frame.ARM9IdleHits.load(std::memory_order_relaxed);
            profTotals.arm7IdleHits += g_Frame.ARM7IdleHits.load(std::memory_order_relaxed);
            profTotals.idleCandidates  += g_Frame.IdleCandidates.load(std::memory_order_relaxed);
            profTotals.idleAccepted    += g_Frame.IdleAccepted.load(std::memory_order_relaxed);
            profTotals.idleRejWriteMem += g_Frame.IdleRejWriteMem.load(std::memory_order_relaxed);
            profTotals.idleRejCoproc   += g_Frame.IdleRejCoproc.load(std::memory_order_relaxed);
            profTotals.idleRejBranch   += g_Frame.IdleRejBranch.load(std::memory_order_relaxed);
            profTotals.idleRejRegDep   += g_Frame.IdleRejRegDep.load(std::memory_order_relaxed);
            profTotals.memBlock9HelperCalls   += g_Frame.MemBlock9HelperCalls.load(std::memory_order_relaxed);
            profTotals.memRead9U32HelperCalls += g_Frame.MemRead9U32HelperCalls.load(std::memory_order_relaxed);
            uint64_t pk = g_Frame.PendingPeak.load(std::memory_order_relaxed);
            if (pk > profTotals.pendingPeak) profTotals.pendingPeak = pk;
        }
#endif

        // Drain the SPU output buffer produced this frame into the rolling hash.
        for (;;)
        {
            int got = nds->SPU.ReadOutput(audioDrain.data(), 2048);
            if (got <= 0) break;
            XXH3_64bits_update(audioHashState, audioDrain.data(),
                               (size_t)got * 2 * sizeof(s16));
            if (wav) fwrite(audioDrain.data(), 4, (size_t)got, wav);
            liteds::AudioTapWrite(0, frame, audioDrain.data(), got);
            audioSampleCount += (u64)got;
            if (got < 2048) break;
        }

        void* top = nullptr;
        void* bot = nullptr;
#ifdef LITEV_HEADLESS_GL
        if (opt.renderer != "soft")
        {
            for (const auto& d : opt.fbDumps)
                if (d.first == frame && !d.second.empty())
                {
                    nds->GPU.GetFramebuffers(&top, &bot);
                    if (DumpGLArray(d.second, *(GLuint*)top, opt.glScale))
                        fprintf(stderr, "dumped frame %d to %s\n", frame, d.second.c_str());
                }
            continue;
        }
#endif
        bool haveFb = nds->GPU.GetFramebuffers(&top, &bot);

        if (haveFb && top && bot)
        {
            u64 topHash = XXH3_64bits(top, kScreenBytes);
            u64 botHash = XXH3_64bits(bot, kScreenBytes);

            if (!haveFirst) { firstTopHash = topHash; haveFirst = true; }
            else if (topHash != firstTopHash) anyChange = true;

            lastTopHash = topHash;
            lastBotHash = botHash;

            if (!opt.scriptShotDir.empty())
            {
                for (const auto& label : scriptMilestones)
                {
                    std::string safe;
                    for (unsigned char c : label)
                        safe += std::isalnum(c) ? (char)c : '_';
                    if (safe.size() > 48) safe.resize(48);
                    const std::string path = opt.scriptShotDir + "/f" + std::to_string(frame + 1) + "_" + safe + ".ppm";
                    if (!DumpPPM(path, (const u32*)top, (const u32*)bot))
                        fprintf(stderr, "warning: milestone screenshot failed: %s\n", path.c_str());
                    printf("MILESTONE frame=%d name=%s top=%016llx bottom=%016llx file=%s\n", frame + 1,
                           safe.c_str(), (unsigned long long)topHash, (unsigned long long)botHash, path.c_str());
                    fflush(stdout);
                }
            }

            if (opt.fbHashEvery > 0 && ((frame + 1) % opt.fbHashEvery) == 0)
            {
                static const bool fbRetire = getenv("LITEV_FB_RETIRE") != nullptr;   // + frames finished, drawn / skip-repeat
                auto* sr = fbRetire ? dynamic_cast<melonDS::SoftRenderer*>(&nds->GPU.GetRenderer()) : nullptr;
                if (sr)
                    printf("frame %d  top=%016llx  bottom=%016llx r=%u d=%d sr=%d\n",
                           frame + 1, (unsigned long long)topHash, (unsigned long long)botHash, sr->RetiredFrames(),
#ifdef LITEV_AGGRESSIVE_SKIP
                           !nds->GPU.SkipThisFrame,
#else
                           1,
#endif
#ifdef LITEV_SKIP_REPEAT_FRAMES
                           (int)nds->GPU.SkipRepeat);
#else
                           0);
#endif
                else
                printf("frame %d  top=%016llx  bottom=%016llx\n",
                       frame + 1, (unsigned long long)topHash, (unsigned long long)botHash);
                fflush(stdout);
            }

            if (video && opt.reloadAt < 0)   // not the warm-up frames before a --reload-at
            {
                std::vector<u8> rgb(256 * 384 * 3);
                for (int i = 0; i < 256 * 384; i++)
                {
                    u32 px = i < 256 * 192 ? ((const u32*)top)[i] : ((const u32*)bot)[i - 256 * 192];
                    rgb[i*3+0] = (px >> 16) & 0xFF; rgb[i*3+1] = (px >> 8) & 0xFF; rgb[i*3+2] = px & 0xFF;
                }
                fwrite(rgb.data(), 1, rgb.size(), video);
            }

            for (const auto& d : opt.fbDumps)
            {
                if (d.first == frame && !d.second.empty())
                {
                    if (DumpPPM(d.second, (const u32*)top, (const u32*)bot))
                        fprintf(stderr, "dumped frame %d to %s\n", frame, d.second.c_str());
                    else
                        fprintf(stderr, "warning: failed to dump PPM to %s\n", d.second.c_str());
                }
            }
        }
    }

    if (video) pclose(video);
    if (!recordedHashes.empty())
        printf("%s (%d of %zu recorded hashes checked)\n", firstDiff >= 0 ? ("REPLAY DIFFERS at frame " + std::to_string(firstDiff)).c_str() : "REPLAY OK",
               hashesChecked, recordedHashes.size());

    auto wallEnd = std::chrono::steady_clock::now();
    double wallSec = std::chrono::duration<double>(wallEnd - wallStart).count();
    double avgFps = wallSec > 0 ? opt.frames / wallSec : 0.0;

    // --bench-window: FPS over frames [start,end] inclusive only.
    int    windowFrames = 0;
    double windowSec = 0.0, windowFps = 0.0;
    if (haveWindow)
    {
        windowFrames = opt.benchWindowEnd - opt.benchWindowStart + 1;
        windowSec = std::chrono::duration<double>(windowEnd - windowStart).count();
        windowFps = windowSec > 0 ? windowFrames / windowSec : 0.0;
    }

    u64 audioHash = XXH3_64bits_digest(audioHashState);
    XXH3_freeState(audioHashState);
#ifdef LITEV_AMBIENT_LAN
    // leave the LAN session at the last frame (as the app does on stop): the others see the links close now, not
    // after the exit's summary work
    if (ambient) { melonDS::MPInterface::Set(melonDS::MPInterface_Dummy); ambient = nullptr; }
#endif

#if defined(LITEV_A9HLE) && defined(LITEV_HLE_DIAG)
    if (getenv("LITEV_HLESIG_SCAN")) melonDS::A9HLE::SigScan(*nds);
#endif
    // LITEV_HEADLESS_SAVOUT=<path> (test only): the game save memory at the end (e.g. after a LAN trade)
    if (const char* so = getenv("LITEV_HEADLESS_SAVOUT"))
        if (const u8* sv = nds->GetNDSSave())
            if (FILE* fp = fopen(so, "wb")) { fwrite(sv, 1, nds->GetNDSSaveLength(), fp); fclose(fp); }
    printf("=== liteDS-headless summary ===\n");
    printf("mode:        %s\n", opt.jit ? "jit" : "interp");
    printf("frames:      %d\n", opt.frames);
    printf("wall_time_s: %.4f\n", wallSec);
    {
        timespec t1{};
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t1);
        printf("emu_thread_cpu_s: %.4f\n", (t1.tv_sec - emuCpu0.tv_sec) + (t1.tv_nsec - emuCpu0.tv_nsec) * 1e-9);
    }
#ifdef __APPLE__
    {
        uint64_t thsc1[2] = {0, 0};
        if (thread_selfcounts(1, thsc1, sizeof(thsc1)) == 0)
            printf("emu_thread_instructions: %llu\nemu_thread_cycles: %llu\n",
                   (unsigned long long)(thsc1[0] - thsc0[0]), (unsigned long long)(thsc1[1] - thsc0[1]));
    }
#endif
    printf("avg_fps:     %.2f\n", avgFps);
    if (haveWindow)
    {
        printf("window:      %d:%d\n", opt.benchWindowStart, opt.benchWindowEnd);
        printf("window_frames: %d\n", windowFrames);
        printf("window_wall_s: %.4f\n", windowSec);
        printf("window_fps:  %.2f\n", windowFps);
        printf("window_emu_thread_cpu_s: %.6f\n",
               (windowCpuEnd.tv_sec - windowCpuStart.tv_sec) +
               (windowCpuEnd.tv_nsec - windowCpuStart.tv_nsec) * 1e-9);
    }
    printf("final_top:   %016llx\n", (unsigned long long)lastTopHash);
    printf("final_bot:   %016llx\n", (unsigned long long)lastBotHash);
    printf("fb_changing: %s\n", anyChange ? "yes" : "no");
    printf("audio_hash:  %016llx\n", (unsigned long long)audioHash);
    printf("audio_samples: %llu\n", (unsigned long long)audioSampleCount);
#ifdef __APPLE__
    // emulation thread only (RunFrame), so render/worker threads don't dilute A/B deltas
    printf("jit_compiles: %llu restored: %llu same-bytes recompiles: restored %llu not %llu (nocand %llu addr %llu lit %llu ranges %llu) resets %llu revert-locks %llu\n", (unsigned long long)melonDS::JitCompileCount, (unsigned long long)melonDS::JitRestoreCount, (unsigned long long)melonDS::JitRecompileSame[0], (unsigned long long)melonDS::JitRecompileSame[1], (unsigned long long)melonDS::JitRecompileSame[2], (unsigned long long)melonDS::JitRecompileSame[3], (unsigned long long)melonDS::JitRecompileSame[4], (unsigned long long)melonDS::JitRecompileSame[5], (unsigned long long)melonDS::JitRecompileSame[6], (unsigned long long)melonDS::JitRecompileSame[7]);
#ifdef LITEV_JIT_REVIVE
    printf("jit_revived: %llu\n", (unsigned long long)melonDS::JitReviveCount);
#endif
#ifdef LITEV_JIT_COMPILE_STATS
    printf("jit_phase_us_total:");   // Lookup Decode Hash Alloc Emit Protect Insert Link Flush
    for (int p = 0; p < melonDS::JitPhase_Count; p++) printf(" %.0f", melonDS::JitCompileTicks[p] * 1e6 / melonDS::JitTicksPerSec());
    printf("\n");
#endif
    printf("emu_thread_instructions: %llu\nemu_thread_cycles: %llu\n",
           (unsigned long long)EmuThreadInstr, (unsigned long long)EmuThreadCycles);
#endif
#ifndef __APPLE__
    printf("emu_thread_cpu_ns: %llu\n", (unsigned long long)EmuThreadCycles);
#endif
    if (wav)
    {
        const u32 n = (u32)audioSampleCount * 4, r = n + 36, rate = 48000, bps = rate * 4;
        const u8 h[44] = {'R','I','F','F', u8(r), u8(r>>8), u8(r>>16), u8(r>>24), 'W','A','V','E','f','m','t',' ',
                          16,0,0,0, 1,0, 2,0, u8(rate), u8(rate>>8), u8(rate>>16), 0, u8(bps), u8(bps>>8), u8(bps>>16), 0,
                          4,0, 16,0, 'd','a','t','a', u8(n), u8(n>>8), u8(n>>16), u8(n>>24)};
        fseek(wav, 0, SEEK_SET); fwrite(h, 1, 44, wav); fclose(wav);
    }
#if LITEV_PROFILE
    printf("links_patched:   %llu\n", (unsigned long long)profTotals.linksPatched);
    printf("links_unlinked:  %llu\n", (unsigned long long)profTotals.linksUnlinked);
    printf("pending_peak:    %llu\n", (unsigned long long)profTotals.pendingPeak);
    printf("cpp_reentries:   %llu\n", (unsigned long long)profTotals.cppReentries);
    printf("dispatcher_miss: %llu\n", (unsigned long long)profTotals.dispatcherMisses);
    printf("dispatcher_hits: %llu\n", (unsigned long long)profTotals.dispatcherHits);
    printf("icache_hits:     %llu\n", (unsigned long long)profTotals.icacheHits);
    printf("ras_hits:        %llu\n", (unsigned long long)profTotals.rasHits);
    {
        unsigned long long ic = profTotals.icacheHits, dh = profTotals.dispatcherHits;
        unsigned long long tot = ic + dh;
        if (tot) printf("icache_hit_rate: %.2f%% (of %llu region-resolved hops)\n",
                        100.0 * (double)ic / (double)tot, tot);
    }
#ifdef LITEV_JIT_DIRECTPATCH
    printf("direct_promotions:  %llu\n", (unsigned long long)profTotals.directPromotions);
    printf("direct_guard_hits:  %llu\n", (unsigned long long)profTotals.directGuardHits);
    printf("direct_guard_misses:%llu\n", (unsigned long long)profTotals.directGuardMisses);
    printf("direct_demotions:   %llu\n", (unsigned long long)profTotals.directDemotions);
    printf("direct_reverts:     %llu\n", (unsigned long long)profTotals.directReverts);
    {
        unsigned long long gh = profTotals.directGuardHits, gm = profTotals.directGuardMisses;
        unsigned long long gt = gh + gm;
        if (gt) printf("direct_guard_hit_rate: %.2f%% (of %llu guard-stub hops)\n",
                       100.0 * (double)gh / (double)gt, gt);
    }
#endif
    printf("mem_block9_helper_calls:    %llu\n", (unsigned long long)profTotals.memBlock9HelperCalls);
    printf("mem_read9_u32_helper_calls: %llu\n", (unsigned long long)profTotals.memRead9U32HelperCalls);
    printf("link_sites_emitted:  %llu\n", (unsigned long long)profTotals.linkSitesEmitted);
    printf("dispatch_only_exits: %llu\n", (unsigned long long)profTotals.dispatchOnlyExits);
    printf("sched_iterations:    %llu\n", (unsigned long long)profTotals.schedIterations);
    printf("sched_events_fired:  %llu\n", (unsigned long long)profTotals.schedEventsFired);
    // Per-frame averages divide by the number of frames folded into profTotals
    // (window-scoped when --bench-window is set, else all frames).
    const double pf = profFrames ? (double)profFrames : 1.0;
    printf("prof_frames:     %llu\n", (unsigned long long)profFrames);
    printf("sched_iters_per_frame: %.2f\n", (double)profTotals.schedIterations / pf);
    printf("cpp_reentries_per_frame: %.2f\n", (double)profTotals.cppReentries / pf);
    {
        // M6.11 RunFrame decomposition. RunFrameNs is the measured parent;
        // ARM9/GPU3D/ARM7/DMA/RunSystem are children carved out of it; the
        // residual (parent minus children) is scheduler/event-dispatch + slice
        // loop overhead. Shares are % of the true parent (RunFrameNs), not of
        // the child sum, so geometry's slice of the whole frame is honest.
        uint64_t dmaNs   = profTotals.dma9Ns + profTotals.dma7Ns;
        uint64_t childNs = profTotals.arm9ExecNs + profTotals.gpu3dNs
                         + profTotals.arm7ExecNs + dmaNs + profTotals.runSystemNs;
        uint64_t parentNs = profTotals.runFrameNs;
        // Guard: if instrumentation ever over-counts children vs parent, clamp.
        uint64_t residualNs = parentNs > childNs ? parentNs - childNs : 0;
        double denom = parentNs ? (double)parentNs : 1.0;

        printf("run_frame_ns:    %llu\n", (unsigned long long)parentNs);
        printf("arm9_exec_ns:    %llu\n", (unsigned long long)profTotals.arm9ExecNs);
        printf("gpu3d_ns:        %llu\n", (unsigned long long)profTotals.gpu3dNs);
        printf("arm7_exec_ns:    %llu\n", (unsigned long long)profTotals.arm7ExecNs);
        printf("dma9_ns:         %llu\n", (unsigned long long)profTotals.dma9Ns);
        printf("dma7_ns:         %llu\n", (unsigned long long)profTotals.dma7Ns);
        printf("dma_ns:          %llu\n", (unsigned long long)dmaNs);
        printf("run_system_ns:   %llu\n", (unsigned long long)profTotals.runSystemNs);
        printf("spu_mix_ns:      %llu\n", (unsigned long long)profTotals.spuMixNs);
        printf("residual_ns:     %llu\n", (unsigned long long)residualNs);
        printf("gx_commands:     %llu\n", (unsigned long long)profTotals.gxCommands);

        // per-frame ns (over the accumulated frames)
        printf("run_frame_ns_per_frame: %.0f\n", (double)parentNs / pf);
        printf("arm9_exec_ns_per_frame: %.0f\n", (double)profTotals.arm9ExecNs / pf);
        printf("gpu3d_ns_per_frame:     %.0f\n", (double)profTotals.gpu3dNs / pf);
        printf("arm7_exec_ns_per_frame: %.0f\n", (double)profTotals.arm7ExecNs / pf);
        printf("dma_ns_per_frame:       %.0f\n", (double)dmaNs / pf);
        printf("run_system_ns_per_frame:%.0f\n", (double)profTotals.runSystemNs / pf);
        printf("spu_mix_ns_per_frame:   %.0f\n", (double)profTotals.spuMixNs / pf);
        printf("residual_ns_per_frame:  %.0f\n", (double)residualNs / pf);
        printf("gx_commands_per_frame:  %.0f\n", (double)profTotals.gxCommands / pf);
        printf("lighting_calls:  %llu\n", (unsigned long long)profTotals.lightingCalls);
        printf("lighting_calls_per_frame: %.1f\n", (double)profTotals.lightingCalls / pf);
        printf("gpu3d_ns_per_gx_command: %.1f\n",
               profTotals.gxCommands ? (double)profTotals.gpu3dNs / profTotals.gxCommands : 0.0);

        // shares as % of the RunFrame parent
        printf("arm9_share_pct:  %.2f\n", 100.0 * profTotals.arm9ExecNs / denom);
        printf("gpu3d_share_pct: %.2f\n", 100.0 * profTotals.gpu3dNs / denom);
        printf("arm7_share_pct:  %.2f\n", 100.0 * profTotals.arm7ExecNs / denom);
        printf("dma_share_pct:   %.2f\n", 100.0 * dmaNs / denom);
        printf("system_share_pct:%.2f\n", 100.0 * profTotals.runSystemNs / denom);
        printf("residual_share_pct: %.2f\n", 100.0 * residualNs / denom);

        // Per-event-type scheduler dispatch breakdown: which event floods
        // RunSystem. Names track the NDS Event_* enum order.
        {
            static const char* kEvtNames[] = {
                "LCD","SPU","Wifi","RTC","DisplayFIFO",
                "CartROMTransfer9","CartSPITransfer9","CartROMTransfer7","CartSPITransfer7",
                "SPITransfer","Div","Sqrt","Timer9","Timer7",
                "DSi_SDMMC","DSi_SDIO","DSi_NWifi","DSi_CamIRQ","DSi_CamTransfer",
                "DSi_DSP","DSi_DSPHLE","DSi_Cart2ROM9","DSi_Cart2SPI9","DSi_Cart2ROM7",
                "DSi_Cart2SPI7","DSi_Cart1Power","DSi_Cart2Power"
            };
            const int nNames = (int)(sizeof(kEvtNames)/sizeof(kEvtNames[0]));
            for (int e = 0; e < 32; e++)
            {
                if (!schedByType[e]) continue;
                printf("evt[%02d]_%-16s per_frame=%8.1f  total=%llu\n",
                       e, (e < nNames ? kEvtNames[e] : "?"),
                       (double)schedByType[e] / pf,
                       (unsigned long long)schedByType[e]);
            }
        }

        printf("arm9_idle_hits:  %llu\n", (unsigned long long)profTotals.arm9IdleHits);
        printf("arm7_idle_hits:  %llu\n", (unsigned long long)profTotals.arm7IdleHits);
        printf("arm9_idle_hits_per_frame: %.2f\n", (double)profTotals.arm9IdleHits / pf);
        printf("arm7_idle_hits_per_frame: %.2f\n", (double)profTotals.arm7IdleHits / pf);
        // IsIdleLoop reason histogram (target A). Static compile-time totals over
        // the run (loops are compiled once), NOT per-frame. accepted+rej* == cand.
        printf("idle_candidates:   %llu\n", (unsigned long long)profTotals.idleCandidates);
        printf("idle_accepted:     %llu\n", (unsigned long long)profTotals.idleAccepted);
        printf("idle_rej_writemem: %llu\n", (unsigned long long)profTotals.idleRejWriteMem);
        printf("idle_rej_coproc:   %llu\n", (unsigned long long)profTotals.idleRejCoproc);
        printf("idle_rej_branch:   %llu\n", (unsigned long long)profTotals.idleRejBranch);
        printf("idle_rej_regdep:   %llu\n", (unsigned long long)profTotals.idleRejRegDep);
    }
#endif
    fflush(stdout);

    if (!opt.profileJson.empty())
    {
        FILE* jf = fopen(opt.profileJson.c_str(), "wb");
        if (jf)
        {
            fprintf(jf,
                "{\n"
                "  \"rom\": \"%s\",\n"
                "  \"mode\": \"%s\",\n"
                "  \"frames\": %d,\n"
                "  \"wall_time_s\": %.6f,\n"
                "  \"avg_fps\": %.4f,\n"
                "  \"window_start\": %d,\n"
                "  \"window_end\": %d,\n"
                "  \"window_frames\": %d,\n"
                "  \"window_wall_s\": %.6f,\n"
                "  \"window_fps\": %.4f,\n"
                "  \"final_top_hash\": \"%016llx\",\n"
                "  \"final_bottom_hash\": \"%016llx\",\n"
                "  \"framebuffer_changing\": %s,\n"
                "  \"audio_hash\": \"%016llx\",\n"
                "  \"audio_samples\": %llu\n"
#if LITEV_PROFILE
                "  ,\"links_patched\": %llu\n"
                "  ,\"links_unlinked\": %llu\n"
                "  ,\"pending_peak\": %llu\n"
                "  ,\"cpp_reentries\": %llu\n"
                "  ,\"dispatcher_misses\": %llu\n"
                "  ,\"sched_iterations\": %llu\n"
                "  ,\"sched_events_fired\": %llu\n"
                "  ,\"prof_frames\": %llu\n"
                "  ,\"run_frame_ns\": %llu\n"
                "  ,\"arm9_exec_ns\": %llu\n"
                "  ,\"gpu3d_ns\": %llu\n"
                "  ,\"arm7_exec_ns\": %llu\n"
                "  ,\"dma9_ns\": %llu\n"
                "  ,\"dma7_ns\": %llu\n"
                "  ,\"run_system_ns\": %llu\n"
                "  ,\"gx_commands\": %llu\n"
                "  ,\"arm9_idle_hits\": %llu\n"
                "  ,\"arm7_idle_hits\": %llu\n"
#endif
                "}\n",
                opt.rom.c_str(),
                opt.jit ? "jit" : "interp",
                opt.frames,
                wallSec,
                avgFps,
                opt.benchWindowStart,
                opt.benchWindowEnd,
                windowFrames,
                windowSec,
                windowFps,
                (unsigned long long)lastTopHash,
                (unsigned long long)lastBotHash,
                anyChange ? "true" : "false",
                (unsigned long long)audioHash,
                (unsigned long long)audioSampleCount
#if LITEV_PROFILE
                , (unsigned long long)profTotals.linksPatched
                , (unsigned long long)profTotals.linksUnlinked
                , (unsigned long long)profTotals.pendingPeak
                , (unsigned long long)profTotals.cppReentries
                , (unsigned long long)profTotals.dispatcherMisses
                , (unsigned long long)profTotals.schedIterations
                , (unsigned long long)profTotals.schedEventsFired
                , (unsigned long long)profFrames
                , (unsigned long long)profTotals.runFrameNs
                , (unsigned long long)profTotals.arm9ExecNs
                , (unsigned long long)profTotals.gpu3dNs
                , (unsigned long long)profTotals.arm7ExecNs
                , (unsigned long long)profTotals.dma9Ns
                , (unsigned long long)profTotals.dma7Ns
                , (unsigned long long)profTotals.runSystemNs
                , (unsigned long long)profTotals.gxCommands
                , (unsigned long long)profTotals.arm9IdleHits
                , (unsigned long long)profTotals.arm7IdleHits
#endif
                );
            fclose(jf);
            fprintf(stderr, "wrote profile json: %s\n", opt.profileJson.c_str());
        }
        else
        {
            fprintf(stderr, "warning: could not write profile json '%s'\n", opt.profileJson.c_str());
        }
    }

#ifdef LITEV_ACCESS_STATS
    if (getenv("LITEV_DUMPMEM"))
    {
        u32 a = strtoul(getenv("LITEV_DUMPMEM"), nullptr, 16);
        FILE* f = fopen("/tmp/claude-501/memdump.bin", "wb");
        for (u32 i = 0; i < 0x100; i += 4) { u32 w = nds->ARM9Read32(a + i); fwrite(&w, 4, 1, f); }
        fclose(f);
        fprintf(stderr, "CPSR %08X PC %08X\n", nds->ARM9.CPSR, nds->ARM9.R[15]);
    }
#endif
    return 0;
}
