/*
    liteDS-headless - Unit 0 headless benchmark runner for liteDS-v2.

    Runs a DS ROM headlessly with the software renderer, null audio and no
    input, for a fixed number of frames, and reports timing + framebuffer
    hashes. Uses FreeBIOS + generated firmware with direct boot, so no external
    BIOS/firmware files are required.

    See docs/liteDS-v2-plan.md B.2 "Unit 0" for the spec.
*/

#include <cctype>
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
#include <chrono>
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
static uint64_t EmuThreadInstr = 0, EmuThreadCycles = 0;
#endif
#include "LiteProfile.h"
#include "VerifyTrace.h"
#include "InputScript.h"
namespace melonDS { extern u64 JitCompileCount, JitProtectCalls, JitProtectFaults, JitStoreRepromotions; }
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
    for (size_t i = 0; i < v.size() && i < 60; i++)
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

    std::string inputScript;            // --input-script: scripted button input
    std::string lanJoin;                // --lan-join <host>: join a LAN session (LITEV_HEADLESS_LAN)
    int lanHost = 0;                    // --lan-host <maxplayers>: host a LAN session instead
    int macXor = 0;                     // --mac-xor <n>: distinct firmware MAC per headless instance
    std::string lanName = "Headless";   // --lan-name: player name shown in the host's lobby

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
        "                            (multiple frames in ONE run; flag may also repeat)\n"
        "  --audio-interp <mode>     SPU interpolation: none|linear|cosine|cubic|gaussian (default none)\n"
        "  --frameskip N             skip N of every N+1 frames' rasterization (LITEV_AGGRESSIVE_SKIP build)\n"
        "  --soft3d fast|accurate    software 3D backend: tile renderer (default when built in) or SoftRenderer3D\n"
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
bool SaveSavestate(NDS& nds, const std::string& path)
{
    Savestate state; // default ctor: save mode, DEFAULT_SIZE buffer
    if (state.Error) { fprintf(stderr, "error: savestate alloc failed\n"); return false; }
    if (!nds.DoSavestate(&state) || state.Error)
    {
        fprintf(stderr, "error: DoSavestate failed while dumping\n");
        return false;
    }
    FILE* f = fopen(path.c_str(), "wb");
    if (!f) { fprintf(stderr, "error: cannot open savestate '%s' for write\n", path.c_str()); return false; }
    size_t wr = fwrite(state.Buffer(), 1, state.Length(), f);
    fclose(f);
    if (wr != state.Length())
    {
        fprintf(stderr, "error: short write to savestate '%s'\n", path.c_str());
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

int main(int argc, char** argv)
{
    Options opt;
    if (!ParseArgs(argc, argv, opt))
        Usage(argv[0], 2);

    HeadlessHost::SetDataDir(opt.dataDir);

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
    const bool lan = !opt.lanJoin.empty() || opt.lanHost > 0;
    auto lanNext = std::chrono::steady_clock::now();
    if (lan)
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

    // App record mode (MelonDS.cpp recordFrame): every 60 frames the app logs a hash of the input
    // applied, the clock and the RAM. Same formula here, so a replay proves it matches the device.
    std::map<int, unsigned long long> recordedHashes;
    if (!opt.checkRecording.empty())
        if (FILE* f = fopen(opt.checkRecording.c_str(), "r"))
        {
            char line[256];
            int fr;
            unsigned long long h;
            while (fgets(line, sizeof(line), f))
                if (const char* c = strrchr(line, ','); sscanf(line, "%d", &fr) == 1 && c && sscanf(c + 1, "%llx", &h) == 1)
                    recordedHashes[fr] = h;
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

#ifdef LITEV_FF_HEADLESS3D
    if (getenv("LITEV_FFHEADLESS")) nds->GPU.FFHeadless3D = atoi(getenv("LITEV_FFHEADLESS")) != 0;   // the app's fast-forward mode
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
        if (haveWindow && frame == opt.benchWindowStart)
            windowStart = std::chrono::steady_clock::now();

        LITE_PROFILE_RESET_FRAME();
        static const bool frameMs = getenv("LITEV_FRAME_MS") != nullptr;   // per-frame time + JIT compiles
        const auto fms0 = std::chrono::steady_clock::now();
        const u64 jit0 = melonDS::JitCompileCount, prot0 = melonDS::JitProtectCalls, flt0 = melonDS::JitProtectFaults;
#ifdef __APPLE__
        uint64_t tc0[2] = {};
        thread_selfcounts(1, tc0, sizeof(tc0));
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
        nds->RunFrame();
#ifdef __APPLE__
        {
            uint64_t tc1[2] = {};
            thread_selfcounts(1, tc1, sizeof(tc1));
            EmuThreadInstr += tc1[0] - tc0[0];
            EmuThreadCycles += tc1[1] - tc0[1];
        }
#endif
        if (frameMs)
            printf("FRAME %d %.3f ms jit %llu mprotect %llu rewrites %llu\n", frame,
                   std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - fms0).count(),
                   (unsigned long long)(melonDS::JitCompileCount - jit0),
                   (unsigned long long)(melonDS::JitProtectCalls - prot0), (unsigned long long)(melonDS::JitProtectFaults - flt0));
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
        if ((!recordedHashes.empty() || emitHashes) && ((frame + 1) % 60) == 0)
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
                if (emitHashes) printf("%d,0,0,0,0,1,0,%016llx\n", frame, (unsigned long long) hash);
                if (it != recordedHashes.end() && ++hashesChecked && hash != it->second && firstDiff < 0)
                    firstDiff = frame;
            }
        }

#ifdef LITEV_HEADLESS_LAN
        if (lan)
        {
            melonDS::MPInterface::Get().Process();
            if (frame % 120 == 0)
            {
                auto players = ((melonDS::LAN&) melonDS::MPInterface::Get()).GetPlayerList();
                printf("LAN f%d:", frame);
                for (const auto& p : players)
                    printf(" [%d %s status=%d ping=%u%s]", p.ID, p.Name, (int) p.Status, p.Ping, p.IsLocalPlayer ? " local" : "");
                printf("\n");
                fflush(stdout);
            }
            if (!getenv("LITEV_LAN_NOPACE"))
            {
                lanNext += std::chrono::microseconds(16667);
                std::this_thread::sleep_until(lanNext);
            }
        }
#endif

        if (haveWindow && frame == opt.benchWindowEnd)
            windowEnd = std::chrono::steady_clock::now();

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
            audioSampleCount += (u64)got;
            if (got < 2048) break;
        }

        void* top = nullptr;
        void* bot = nullptr;
        bool haveFb = nds->GPU.GetFramebuffers(&top, &bot);

        if (haveFb && top && bot)
        {
            u64 topHash = XXH3_64bits(top, kScreenBytes);
            u64 botHash = XXH3_64bits(bot, kScreenBytes);

            if (!haveFirst) { firstTopHash = topHash; haveFirst = true; }
            else if (topHash != firstTopHash) anyChange = true;

            lastTopHash = topHash;
            lastBotHash = botHash;

            if (opt.fbHashEvery > 0 && ((frame + 1) % opt.fbHashEvery) == 0)
            {
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

    printf("=== liteDS-headless summary ===\n");
    printf("mode:        %s\n", opt.jit ? "jit" : "interp");
    printf("frames:      %d\n", opt.frames);
    printf("wall_time_s: %.4f\n", wallSec);
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
    }
    printf("final_top:   %016llx\n", (unsigned long long)lastTopHash);
    printf("final_bot:   %016llx\n", (unsigned long long)lastBotHash);
    printf("fb_changing: %s\n", anyChange ? "yes" : "no");
    printf("audio_hash:  %016llx\n", (unsigned long long)audioHash);
    printf("audio_samples: %llu\n", (unsigned long long)audioSampleCount);
#ifdef __APPLE__
    // emulation thread only (RunFrame), so render/worker threads don't dilute A/B deltas
    printf("emu_thread_instructions: %llu\nemu_thread_cycles: %llu\n",
           (unsigned long long)EmuThreadInstr, (unsigned long long)EmuThreadCycles);
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
    DumpAccessStats();
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
