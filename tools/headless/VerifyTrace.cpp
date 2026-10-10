/*
    liteDS-v2 headless harness - trace recording / verification / convergence.
    See VerifyTrace.h for the oracle rationale.
*/

#include <cstdlib>
#include "VerifyTrace.h"
#include "LockstepMP.h"
#include "NetplayInput.h"
#ifdef LITEV_HOSTED_NETPLAY
#include "HostedMP.h"
#endif
#include <map>
#include <sys/resource.h>

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cinttypes>
#include <ctime>
#include <string>
#include <vector>
#include <memory>
#include <optional>
#include <thread>
#include <atomic>
#include <chrono>
#include <algorithm>

#include "Args.h"
#include "NDS.h"
#include "NDSCart.h"
#include "GPU.h"
#include "GPU_Soft.h"
#include "Platform.h"
#include "xxhash/xxhash.h"

#include "PlatformHeadless.h"
#include "LiteProfile.h"
#include "InputScript.h"
#include "MPInterface.h"
#include "SPI.h"
#include "SPI_Firmware.h"

using namespace melonDS;

namespace liteds
{

namespace
{

constexpr int    kScreenW = 256;
constexpr int    kScreenH = 192;
constexpr size_t kScreenBytes = (size_t)kScreenW * kScreenH * sizeof(u32);

constexpr char   kMagic[8]   = { 'L','I','T','E','T','R','A','C' };
constexpr u32    kTraceVersion = 1;

// ---------------------------------------------------------------------------
// Fixed-size on-disk trace layout. Hand-packed so natural alignment leaves no
// padding; static_asserts guard the sizes so a struct change can't silently
// invalidate committed golden traces.
// ---------------------------------------------------------------------------

#pragma pack(push, 1)
struct TraceHeader
{
    char magic[8];        // "LITETRAC"
    u32  version;         // kTraceVersion
    u32  recordSize;      // sizeof(TraceRecord)
    u32  frames;          // number of records that follow
    u32  jit;             // 1 if recorded under JIT, else 0
    u64  romSize;         // ROM length in bytes
    u64  romHash;         // XXH3_64bits of the ROM image
    s64  rtcEpoch;        // fixed RTC unix timestamp used
    char romName[64];     // ROM basename, NUL-padded/truncated
    u64  scriptHash;      // xxhash of the input script (0 == no script)
    u8   reserved[8];
};

struct TraceRecord
{
    u32 frame;
    u32 pad;
    u32 arm9R[16];
    u32 arm9CPSR;
    u32 arm7R[16];
    u32 arm7CPSR;
    u64 sysTimestamp;
    u64 arm9Timestamp;
    u64 arm7Timestamp;
    u64 mainRamHash;
    u64 fbTopHash;
    u64 fbBotHash;
};
#pragma pack(pop)

static_assert(sizeof(TraceHeader) == 128, "TraceHeader layout drift");
static_assert(sizeof(TraceRecord) == 192, "TraceRecord layout drift");

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// LITEV_TRACE_ROUNDTRIP=<frame>: save the state to memory after that frame and load it back
// (a run with the round trip must trace identically to one without).
void RoundTripAt(NDS& nds, int frame)
{
    static const char* e = getenv("LITEV_TRACE_ROUNDTRIP");
    if (!e || atoi(e) != frame) return;
    Savestate save;
    nds.DoSavestate(&save);
    Savestate load(save.Buffer(), save.Length(), false);
    if (save.Error || !nds.DoSavestate(&load) || load.Error) fprintf(stderr, "error: round trip failed\n");
    else fprintf(stderr, "round trip at frame %d (%u bytes)\n", frame, save.Length());
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

std::string Basename(const std::string& path)
{
    auto slash = path.find_last_of("/\\");
    return slash == std::string::npos ? path : path.substr(slash + 1);
}

// A booted NDS instance plus the per-instance platform userdata it points at.
// The userdata must outlive the NDS, so they are bundled together.
struct BuiltNDS
{
    std::unique_ptr<NDS> nds;
    std::unique_ptr<HeadlessHost::InstanceUserData> udata;
    u64 romHash = 0;
    u32 romSize = 0;
    InputScript inputScript;   // optional scripted input (empty => none)

    // Apply the scripted key mask for `frame` (no-op when no script loaded).
    void ApplyInput(int frame)
    {
        if (inputScript.Loaded())
            nds->SetKeyMask(inputScript.KeyMaskForFrame(frame));
        int tx, ty;
        if (inputScript.HasTouch())
        {
            if (inputScript.TouchForFrame(frame, tx, ty)) nds->TouchScreen(tx, ty);
            else                                          nds->ReleaseScreen();
        }
    }
};

// "k=v,k=v" (LITEV_NETPLAY-style specs)
std::map<std::string, std::string> ParseSpec(const char* spec)
{
    std::map<std::string, std::string> kv;
    std::string s = spec ? spec : "";
    for (size_t pos = 0; pos < s.size();)
    {
        size_t end = s.find(',', pos);
        std::string one = s.substr(pos, end == std::string::npos ? std::string::npos : end - pos);
        size_t eq = one.find('=');
        kv[one.substr(0, eq)] = eq == std::string::npos ? "1" : one.substr(eq + 1);
        if (end == std::string::npos) break;
        pos = end + 1;
    }
    return kv;
}

int SpecInt(std::map<std::string, std::string>& kv, const char* k, int def)
{
    return kv.count(k) ? atoi(kv[k].c_str()) : def;
}

// process CPU time so far (user + system), ms
double CpuMs()
{
    rusage ru {};
    getrusage(RUSAGE_SELF, &ru);
    return (ru.ru_utime.tv_sec + ru.ru_stime.tv_sec) * 1e3 + (ru.ru_utime.tv_usec + ru.ru_stime.tv_usec) / 1e3;
}

// Build, reset, direct-boot and start an NDS from `cfg`. Returns false + err on
// failure. `jitOverride`, when set, wins over cfg.jit (used by the converge
// path to build one JIT and one interp instance from the same config).
bool BuildAndBoot(const TraceRunConfig& cfg, std::optional<bool> jitOverride,
                  BuiltNDS& out, std::string& err)
{
    u32 romlen = 0;
    // one in-memory ROM for every console of this process that boots the same file
    auto romdata = NDSCart::AcquireSharedROM(cfg.rom, romlen);
    if (!romdata) { err = "cannot read ROM '" + cfg.rom + "'"; return false; }

    // Hash the file as loaded: a console booted earlier may since have re-encrypted the secure
    // area of the shared buffer in place. The first load of a path is always fresh from the file.
    static std::map<std::string, u64> romHashes;
    auto h = romHashes.find(cfg.rom);
    if (h == romHashes.end()) h = romHashes.emplace(cfg.rom, XXH3_64bits(romdata.get(), romlen)).first;
    out.romHash = h->second;
    out.romSize = romlen;

    auto cart = NDSCart::ParseROM(std::move(romdata), romlen, nullptr, std::nullopt);
    if (!cart) { err = "failed to parse DS ROM '" + cfg.rom + "'"; return false; }

    bool jit = jitOverride.value_or(cfg.jit);

    NDSArgs args; // FreeBIOS ARM9/ARM7 + generated firmware, software renderer
#ifdef JIT_ENABLED
    if (jit) args.JIT = JITArgs{};
    else     args.JIT = std::nullopt;
#else
    (void)jit;
    args.JIT = std::nullopt;
#endif

    // LITEV_APP_FW=1: console mpK built as the Android app builds Netplay player K's console
    // (netplayFixConfiguration + EmulatorArgsBuilder customizeFirmware: nickname "SereneDS", English,
    // 1 Jan, colour 0, the generated firmware's MAC + the per-instance offsets; battery okay, RTC
    // 2026-01-01, Reset before the cart is inserted), so a Mac server matches an app guest.
    // Without it the harness's own firmware/MAC/RTC (Mac-only sessions).
    const bool appMac = getenv("LITEV_APP_FW") != nullptr;
    if (appMac)
    {
        int k = cfg.instanceTag.rfind("mp", 0) == 0 ? atoi(cfg.instanceTag.c_str() + 2) : 0;
        auto& u = args.Firmware.GetEffectiveUserData();
        const std::u16string name = u"SereneDS";
        u.NameLength = (u16)name.size(); memcpy(u.Nickname, name.data(), name.size() * 2);
        u.Settings &= ~Firmware::Language::Reserved; u.Settings |= Firmware::Language::English;
        u.FavoriteColor = 0; u.BirthdayMonth = 1; u.BirthdayDay = 1;
        auto& h = args.Firmware.GetHeader();
        if (k > 0)
        {
            h.MacAddr[3] += k; h.MacAddr[4] += k * 0x44; h.MacAddr[5] += k * 0x10;
            h.MacAddr[0] &= 0xFC;
            h.UpdateChecksum();
        }
        args.Firmware.UpdateChecksums();
    }

    out.udata = std::make_unique<HeadlessHost::InstanceUserData>();
    out.udata->savePrefix = cfg.instanceTag;

    out.nds = std::make_unique<NDS>(std::move(args), out.udata.get());
    out.nds->SetRenderer(std::make_unique<SoftRenderer>(*out.nds));
#ifdef LITEV_AGGRESSIVE_SKIP
    if (getenv("LITEV_TRACE_HEADLESS")) // what Netplay does to another player's console
    {
        out.nds->GPU.Headless = true;
        out.nds->GPU.GPU3D.Headless = true;
    }
#endif
    if (appMac)
    {   // the app's order: Reset without a cart, battery + RTC, then insert the cart (no second Reset)
        out.nds->Reset();
        out.nds->SPI.GetPowerMan()->SetBatteryLevelOkay(true);
        out.nds->RTC.SetDateTime(2026, 1, 1, 0, 0, 0);   // what the app's Netplay pins
        out.nds->SetNDSCart(std::move(cart));
    }
    else
    {
    out.nds->SetNDSCart(std::move(cart));
    out.nds->Reset();

    // Pin the RTC to a fixed epoch for determinism (see kDefaultRtcEpoch).
    {
        time_t t = (time_t)cfg.fixedRtcEpoch;
        struct tm g;
#if defined(_WIN32)
        gmtime_s(&g, &t);
#else
        gmtime_r(&t, &g);
#endif
        out.nds->RTC.SetDateTime(g.tm_year + 1900, g.tm_mon + 1, g.tm_mday,
                                 g.tm_hour, g.tm_min, g.tm_sec);
    }
    }

    out.nds->SetupDirectBoot("headless.nds");
    out.nds->Start();
    out.nds->SetKeyMask(0xFFFF); // no buttons pressed (active-low)

    if (!cfg.savestate.empty())
    {
        u32 len = 0;
        auto buf = ReadFile(cfg.savestate, len);
        if (!buf) { err = "cannot read savestate '" + cfg.savestate + "'"; return false; }
        Savestate st(buf.get(), len, false);
        if (st.Error || !out.nds->DoSavestate(&st) || st.Error) { err = "failed to load savestate"; return false; }
    }

    if (!cfg.inputScript.empty())
    {
        std::string serr;
        if (!out.inputScript.LoadFile(cfg.inputScript, serr))
        {
            err = serr;
            return false;
        }
    }
    return true;
}

// What BuiltNDS::ApplyInput applies at `frame`, as one input (no script = nothing pressed).
NetplayFrameInput ScriptInput(BuiltNDS& b, int frame)
{
    NetplayFrameInput in;
    in.Keys = b.inputScript.Loaded() ? b.inputScript.KeyMaskForFrame(frame) : 0xFFFF;
    int tx, ty;
    if (b.inputScript.HasTouch() && b.inputScript.TouchForFrame(frame, tx, ty)) { in.TouchX = tx; in.TouchY = ty; }
    return in;
}

#ifdef LITEV_NP_SPEED
std::string SpeedName(int code)
{
    char b[16];
    if (code == NetplaySpeed::kUncapped) return "uncapped";
    snprintf(b, sizeof(b), "%.2fx", NetplaySpeed::Multiplier(code));
    return b;
}
#endif

// Netplay's way to apply an input
void ApplyNetInput(NDS& nds, const NetplayFrameInput& in)
{
    nds.SetKeyMask(in.Keys);
    if (in.TouchX >= 0) nds.TouchScreen(in.TouchX, in.TouchY);
    else                nds.ReleaseScreen();
}

// Hosted Netplay frame marker state: the input applied this frame, the clock, and every 60
// frames the RAM (the replica must compute the same from its own console)
u64 HostedState(NDS& nds, const NetplayFrameInput& in, int frame)
{
    u64 v[4] = { in.Keys | ((u64)(u16)in.TouchX << 32) | ((u64)(u16)in.TouchY << 48), nds.GetSysTimestamp(),
                 ((frame + 1) % 60) == 0 ? XXH3_64bits(nds.MainRAM, nds.MainRAMMask + 1) : 0, (u64)frame };
    return XXH3_64bits(v, sizeof(v));
}

#ifdef LITEV_HOSTED_NETPLAY
// Hosted Netplay: the server's NetplayInput player index (the consoles are 0..14)
constexpr int kHostedServerId = NetplayInput::kMaxPlayers - 1;

NetFaults FaultsFrom(std::map<std::string, std::string>& kv)
{
    NetFaults f;
    f.LatencyMs = SpecInt(kv, "latency", 0);
    f.JitterMs = SpecInt(kv, "jitter", 0);
    f.LossPct = SpecInt(kv, "loss", 0);
    return f;
}
#endif

// Fill a trace record from the current emulator state after a frame.
void CaptureRecord(NDS& nds, int frame, TraceRecord& rec)
{
    memset(&rec, 0, sizeof(rec));
    rec.frame = (u32)frame;

    for (int i = 0; i < 16; i++)
    {
        rec.arm9R[i] = nds.ARM9.R[i];
        rec.arm7R[i] = nds.ARM7.R[i];
    }
    rec.arm9CPSR = nds.ARM9.CPSR;
    rec.arm7CPSR = nds.ARM7.CPSR;

    rec.sysTimestamp  = nds.GetSysTimestamp();
    rec.arm9Timestamp = nds.ARM9Timestamp;
    rec.arm7Timestamp = nds.ARM7Timestamp;

    rec.mainRamHash = XXH3_64bits(nds.MainRAM, (size_t)nds.MainRAMMask + 1);

    void* top = nullptr; void* bot = nullptr;
    if (nds.GPU.GetFramebuffers(&top, &bot) && top && bot)
    {
        rec.fbTopHash = XXH3_64bits(top, kScreenBytes);
        rec.fbBotHash = XXH3_64bits(bot, kScreenBytes);
    }
}

u64 FramebufferPairHash(NDS& nds)
{
    void* top = nullptr; void* bot = nullptr;
    if (nds.GPU.GetFramebuffers(&top, &bot) && top && bot)
    {
        u64 h[2] = { XXH3_64bits(top, kScreenBytes), XXH3_64bits(bot, kScreenBytes) };
        return XXH3_64bits(h, sizeof(h));
    }
    return 0;
}

// Report the fields that differ between an expected and actual record.
// Returns the number of differing fields (0 == identical).
int DiffRecords(const TraceRecord& e, const TraceRecord& a)
{
    int n = 0;
    auto u32field = [&](const char* name, u32 ev, u32 av) {
        if (ev != av) { printf("  %-16s expected=0x%08x  actual=0x%08x\n", name, ev, av); n++; }
    };
    auto u64field = [&](const char* name, u64 ev, u64 av) {
        if (ev != av)
        {
            printf("  %-16s expected=0x%016" PRIx64 "  actual=0x%016" PRIx64 "\n",
                   name, ev, av);
            n++;
        }
    };

    char buf[24];
    for (int i = 0; i < 16; i++)
    {
        snprintf(buf, sizeof(buf), "arm9.R%d", i);  u32field(buf, e.arm9R[i], a.arm9R[i]);
    }
    u32field("arm9.CPSR", e.arm9CPSR, a.arm9CPSR);
    for (int i = 0; i < 16; i++)
    {
        snprintf(buf, sizeof(buf), "arm7.R%d", i);  u32field(buf, e.arm7R[i], a.arm7R[i]);
    }
    u32field("arm7.CPSR", e.arm7CPSR, a.arm7CPSR);

    u64field("sysTimestamp",  e.sysTimestamp,  a.sysTimestamp);
    u64field("arm9Timestamp", e.arm9Timestamp, a.arm9Timestamp);
    u64field("arm7Timestamp", e.arm7Timestamp, a.arm7Timestamp);
    u64field("mainRamHash",   e.mainRamHash,   a.mainRamHash);
    u64field("fbTopHash",     e.fbTopHash,     a.fbTopHash);
    u64field("fbBotHash",     e.fbBotHash,     a.fbBotHash);
    return n;
}

} // namespace

// ---------------------------------------------------------------------------
// --record-trace
// ---------------------------------------------------------------------------

int RecordTrace(const TraceRunConfig& cfg, int frames, const std::string& outPath)
{
    BuiltNDS b;
    std::string err;
    if (!BuildAndBoot(cfg, std::nullopt, b, err))
    {
        fprintf(stderr, "error: %s\n", err.c_str());
        return 1;
    }

    FILE* f = fopen(outPath.c_str(), "wb");
    if (!f) { fprintf(stderr, "error: cannot open trace '%s' for writing\n", outPath.c_str()); return 1; }

    TraceHeader hdr;
    memset(&hdr, 0, sizeof(hdr));
    memcpy(hdr.magic, kMagic, sizeof(hdr.magic));
    hdr.version    = kTraceVersion;
    hdr.recordSize = sizeof(TraceRecord);
    hdr.frames     = (u32)frames;
    hdr.jit        = cfg.jit ? 1u : 0u;
    hdr.romSize    = b.romSize;
    hdr.romHash    = b.romHash;
    hdr.rtcEpoch   = cfg.fixedRtcEpoch;
    hdr.scriptHash = b.inputScript.Hash();  // 0 when no script
    {
        std::string base = Basename(cfg.rom);
        strncpy(hdr.romName, base.c_str(), sizeof(hdr.romName) - 1);
    }
    if (fwrite(&hdr, sizeof(hdr), 1, f) != 1)
    {
        fprintf(stderr, "error: failed to write trace header\n");
        fclose(f);
        return 1;
    }

    for (int frame = 0; frame < frames; frame++)
    {
        b.ApplyInput(frame);
        LITE_PROFILE_RESET_FRAME();
        b.nds->RunFrame();
        RoundTripAt(*b.nds, (int)frame);

        TraceRecord rec;
        CaptureRecord(*b.nds, frame, rec);
        if (fwrite(&rec, sizeof(rec), 1, f) != 1)
        {
            fprintf(stderr, "error: failed to write trace record at frame %d\n", frame);
            fclose(f);
            return 1;
        }
    }

    fflush(f);
    fclose(f);

    printf("=== liteDS-headless record-trace ===\n");
    printf("rom:      %s\n", cfg.rom.c_str());
    printf("mode:     %s\n", cfg.jit ? "jit" : "interp");
    printf("frames:   %d\n", frames);
    printf("rtc:      %lld\n", cfg.fixedRtcEpoch);
    printf("rom_hash: 0x%016" PRIx64 "\n", b.romHash);
    printf("trace:    %s (%zu bytes)\n", outPath.c_str(),
           sizeof(TraceHeader) + (size_t)frames * sizeof(TraceRecord));
    fflush(stdout);
    return 0;
}

// ---------------------------------------------------------------------------
// --verify-trace
// ---------------------------------------------------------------------------

int VerifyTrace(const TraceRunConfig& cfg, const std::string& tracePath)
{
    FILE* f = fopen(tracePath.c_str(), "rb");
    if (!f) { fprintf(stderr, "error: cannot open trace '%s'\n", tracePath.c_str()); return 1; }

    TraceHeader hdr;
    if (fread(&hdr, sizeof(hdr), 1, f) != 1)
    {
        fprintf(stderr, "error: trace '%s' is too short for a header\n", tracePath.c_str());
        fclose(f);
        return 1;
    }
    if (memcmp(hdr.magic, kMagic, sizeof(kMagic)) != 0)
    {
        fprintf(stderr, "error: '%s' is not a liteDS trace (bad magic)\n", tracePath.c_str());
        fclose(f);
        return 1;
    }
    if (hdr.version != kTraceVersion || hdr.recordSize != sizeof(TraceRecord))
    {
        fprintf(stderr, "error: trace version/record-size mismatch (v%u rec=%u, expected v%u rec=%zu)\n",
                hdr.version, hdr.recordSize, kTraceVersion, sizeof(TraceRecord));
        fclose(f);
        return 1;
    }

    // Match the recording conditions: same JIT mode + RTC epoch.
    TraceRunConfig rc = cfg;
    rc.jit           = (hdr.jit != 0);
    rc.fixedRtcEpoch = hdr.rtcEpoch;

    BuiltNDS b;
    std::string err;
    if (!BuildAndBoot(rc, std::nullopt, b, err))
    {
        fprintf(stderr, "error: %s\n", err.c_str());
        fclose(f);
        return 1;
    }

    if (b.romHash != hdr.romHash)
        fprintf(stderr, "warning: ROM hash differs from trace (trace=0x%016" PRIx64
                        " current=0x%016" PRIx64 ") - verifying anyway\n",
                (u64)hdr.romHash, b.romHash);

    // Sanity: scripted input is part of the deterministic input. Replaying a
    // scripted golden WITHOUT (or with a different) --input-script feeds the
    // core different button input, so the very first post-divergence frame
    // mismatches on register/mainRAM state. That is NOT a core regression and
    // NOT a "born-bad" oracle — it is a harness invocation error. Fail FAST
    // here with an actionable message BEFORE running any frames, so the cause
    // is unambiguous instead of surfacing as a cryptic frame-N state diff that
    // gets mis-attributed to the emulation core. This strengthens (never
    // weakens) the oracle: a genuine same-script replay still runs the full
    // bit-exact comparison below.
    if (hdr.scriptHash != b.inputScript.Hash())
    {
        fprintf(stderr,
                "error: input-script hash mismatch — this trace was recorded "
                "with scripted input (trace=0x%016" PRIx64 ", current=0x%016" PRIx64 ").\n"
                "       Re-run --verify-trace with the SAME --input-script that recorded "
                "the golden\n"
                "       (see the trace's .json sidecar 'input_script' field). Refusing "
                "to verify\n"
                "       under different input — the result would be a false mismatch, not "
                "a core check.\n",
                (u64)hdr.scriptHash, b.inputScript.Hash());
        fclose(f);
        return 4;
    }

    for (u32 frame = 0; frame < hdr.frames; frame++)
    {
        TraceRecord expected;
        if (fread(&expected, sizeof(expected), 1, f) != 1)
        {
            fprintf(stderr, "error: trace truncated: expected %u records, ran out at %u\n",
                    hdr.frames, frame);
            fclose(f);
            return 1;
        }

        b.ApplyInput((int)frame);
        LITE_PROFILE_RESET_FRAME();
        b.nds->RunFrame();
        RoundTripAt(*b.nds, (int)frame);

        TraceRecord actual;
        CaptureRecord(*b.nds, (int)frame, actual);
        // LITEV_TRACE_GUEST_ONLY=1: the framebuffers are produced asynchronously (TILE_COORD,
        // async 2D), so their hashes vary run to run; compare guest state only.
        static const bool guestOnly = getenv("LITEV_TRACE_GUEST_ONLY") != nullptr;
        if (guestOnly)
        {
            actual.fbTopHash = expected.fbTopHash;
            actual.fbBotHash = expected.fbBotHash;
        }

        if (memcmp(&expected, &actual, sizeof(TraceRecord)) != 0)
        {
            printf("MISMATCH at frame %u\n", frame);
            int nd = DiffRecords(expected, actual);
            printf("(%d differing field%s)\n", nd, nd == 1 ? "" : "s");
            fflush(stdout);
            fclose(f);
            return 2;
        }
    }

    fclose(f);
    printf("=== liteDS-headless verify-trace: OK ===\n");
    printf("trace:  %s\n", tracePath.c_str());
    printf("frames: %u (all identical)\n", hdr.frames);
#ifdef LITEV_AGGRESSIVE_SKIP
    printf("captures: %u  capture_seen: %d\n", b.nds->GPU.CaptureCount, (int)b.nds->GPU.CaptureSeen);
#else
    printf("captures: %u\n", b.nds->GPU.CaptureCount);
#endif
    printf("mode:   %s\n", rc.jit ? "jit" : "interp");
    fflush(stdout);
    return 0;
}

// ---------------------------------------------------------------------------
// --verify-interp-converge
// ---------------------------------------------------------------------------

int VerifyInterpConverge(const TraceRunConfig& cfg, int frames)
{
    TraceRunConfig jc = cfg; jc.jit = true;  jc.instanceTag = "headless-jit";
    TraceRunConfig ic = cfg; ic.jit = false; ic.instanceTag = "headless-interp";

    BuiltNDS jb, ib;
    std::string err;
    if (!BuildAndBoot(jc, true, jb, err))  { fprintf(stderr, "error (jit): %s\n", err.c_str()); return 1; }
    if (!BuildAndBoot(ic, false, ib, err)) { fprintf(stderr, "error (interp): %s\n", err.c_str()); return 1; }

    int  diffCount        = 0;   // frames whose fb hashes differ
    int  longestRun       = 0;   // longest run of consecutive differing frames
    int  curRun           = 0;
    int  firstDiff        = -1;
    int  lastDiff         = -1;
    int  tailWindow       = 60;
    int  tailDiffs        = 0;   // diffs within the final `tailWindow` frames

    for (int frame = 0; frame < frames; frame++)
    {
        jb.ApplyInput(frame);
        ib.ApplyInput(frame);
        LITE_PROFILE_RESET_FRAME();
        jb.nds->RunFrame();
        ib.nds->RunFrame();

        u64 hj = FramebufferPairHash(*jb.nds);
        u64 hi = FramebufferPairHash(*ib.nds);

        bool differ = (hj != hi);
        if (differ)
        {
            diffCount++;
            if (firstDiff < 0) firstDiff = frame;
            lastDiff = frame;
            curRun++;
            if (curRun > longestRun) longestRun = curRun;
        }
        else
        {
            curRun = 0;
        }
        if (frame >= frames - tailWindow && differ)
            tailDiffs++;
    }

    bool tailClean = (tailDiffs == 0);

    printf("=== liteDS-headless verify-interp-converge ===\n");
    printf("rom:            %s\n", cfg.rom.c_str());
    printf("frames:         %d\n", frames);
    printf("differing:      %d\n", diffCount);
    printf("first_diff:     %d\n", firstDiff);
    printf("last_diff:      %d\n", lastDiff);
    printf("longest_run:    %d\n", longestRun);
    printf("tail_window:    %d\n", tailWindow);
    printf("tail_diffs:     %d\n", tailDiffs);
    printf("converged:      %s\n", tailClean ? "yes" : "no");
    printf("(interpretation: JIT and interp legitimately diverge mid-boot; the\n");
    printf(" oracle requires the final %d frames to be identical.)\n", tailWindow);
    fflush(stdout);

    return tailClean ? 0 : 3;
}

// ---------------------------------------------------------------------------
// --mp-test : two-instance local-multiplayer harness.
//
// Phase 0 proved two NDS instances run CONCURRENTLY on two threads under the full
// LITEV stack with no global/static conflict (NDS::Current is thread_local,
// NDS.cpp:78; each NDS owns its JIT/memory) -- the same model the Qt frontend uses
// for local wireless. Phase 1 (this version) WIRES local MP: both instances share
// one in-process LocalMP (MPInterface::Set(Local)); each carries a distinct
// instance id + MAC so a wireless test ROM can associate them as two players. The
// game drives MP_Begin/SendCmd/RecvReplies through Wifi.cpp automatically. With a
// non-MP ROM (e.g. shrek) nothing associates and the run behaves like Phase 0 --
// proving the MP wiring is inert until a game actually uses wireless.
// ---------------------------------------------------------------------------
// Draws nothing: stands in for a console whose screens nobody looks at (LITEV_MP_NORENDER1).
// Display captures are skipped too, so that console's VRAM differs from a rendered one.
class NullRenderer3D : public Renderer3D
{
public:
    explicit NullRenderer3D(melonDS::GPU3D& gpu3D) : Renderer3D(gpu3D) {}
    void Reset() override {}
    void RenderFrame() override {}
    u32* GetLine(int) override { return Line; }
private:
    u32 Line[256] = {};
};

class NullRenderer : public Renderer
{
public:
    explicit NullRenderer(melonDS::GPU& gpu) : Renderer(gpu) { Rend3D = std::make_unique<NullRenderer3D>(gpu.GPU3D); }
    bool Init() override { return true; }
    void Reset() override {}
    void Stop() override {}
    void SetRenderSettings(RendererSettings&) override {}
    void DrawScanline(u32) override {}
    void DrawSprites(u32) override {}
    void VBlank() override {}
    void VBlankEnd() override {}
    void AllocCapture(u32, u32, u32) override {}
    void SyncVRAMCapture(u32, u32, u32, bool) override {}
    bool GetFramebuffers(void**, void**) override { return false; }
};

// The real 2D renderer with the 3D renderer replaced by NullRenderer3D (LITEV_MP_NO3D1): bisects
// which half of rendering feeds back into emulated state.
class SoftNo3DRenderer : public SoftRenderer
{
public:
    explicit SoftNo3DRenderer(NDS& nds) : SoftRenderer(nds) { Rend3D = std::make_unique<NullRenderer3D>(nds.GPU.GPU3D); }
};

#ifdef LITEV_EVENT_TRACE
// LITEV_MP_EVTRACE=<file>: every scheduler event of instance 1 (time, event id) from frame
// LITEV_MP_EVTRACE_FROM on, to diff two runs and find the first event that differs.
}
namespace melonDS { extern void (*LitevEventTrace)(NDS*, int, u64, u64); }
namespace liteds {
static FILE* gEvTrace;
static melonDS::NDS* gEvTraceNDS;
static std::atomic<bool> gEvTraceOn {false};
#endif

// ROM transfer in session setup. LITEV_NP_CACHE=<dir>: where ROMs received from the others go
// (unset = this process receives none); LITEV_NP_CACHE_MB: its size cap (2048);
// LITEV_NP_CONSENT=yes|no: the answer to "send / receive this game?" (unset = no).
// Test switches (NetplayInput.cpp): LITEV_NP_XFER_CUT=<bytes>, LITEV_NP_XFER_CORRUPT=1.
static void NetplayHarnessXfer(NetplaySetup& setup)
{
    if (const char* d = getenv("LITEV_NP_CACHE")) setup.CacheDir = d;
    if (const char* mb = getenv("LITEV_NP_CACHE_MB")) setup.CacheMaxBytes = strtoull(mb, nullptr, 10) << 20;
    const char* yes = getenv("LITEV_NP_CONSENT");
    bool answer = yes && !strcmp(yes, "yes");
    setup.Consent = [answer](const std::string& q) {
        printf("netplay consent (%s): %s", answer ? "yes" : "no", q.c_str());
        return answer;
    };
}

int MPTest(const TraceRunConfig& cfg, int frames, const std::vector<std::string>& scripts)
{
    // LITEV_NETPLAY="player=P,delay=D,port=N,peer=IP:PORT[,latency=MS]": two-player Netplay (fixed
    // peer, no session setup). This process is player P's device: it emulates every console, but
    // only player P's input comes from its script; the others' arrive over UDP from their
    // processes, applied D frames late.
    // LITEV_NETPLAY="player=P,players=N,port=X[,host=IP:PORT][,delay=D][,latency=MS]": N-player
    // Netplay with the app's session setup (NetplayHandshake): player 0 hosts (waits for N - 1
    // guests on port X + 1), the others join host=IP:PORT (the host's port). delay omitted = picked
    // by the host from the measured round trips.
    // jitter=MS, loss=PCT: more simulated network faults on receive; pace=FPS: this player's own
    // console runs at most FPS frames per second (the app: 60), the others as their inputs allow.
    // adaptive=0|1: input delay follows the measured round trip (NetplayInput::Adaptive).
    // ff=F:M/F:M...: (LITEV_NP_SPEED) from this player's sampled frame F on, it requests session speed
    // M (multiplier of pace; 1 = none, 0 or -1 = uncapped), as the app's fast-forward key does.
    std::map<int, float> ffPlan;
    int player = 0, delay = -1, port = 7100, latency = 0, players = 0, jitter = 0, loss = 0, pace = 0, adaptive = -1;
    std::string peer, host = "127.0.0.1:7100";
    const char* np = getenv("LITEV_NETPLAY");
    if (np)
    {
        std::string spec = np;
        for (size_t pos = 0; pos < spec.size();)
        {
            size_t end = spec.find(',', pos);
            std::string kv = spec.substr(pos, end == std::string::npos ? std::string::npos : end - pos);
            size_t eq = kv.find('=');
            std::string k = kv.substr(0, eq), v = kv.substr(eq + 1);
            if (k == "player") player = atoi(v.c_str());
            else if (k == "delay") delay = atoi(v.c_str());
            else if (k == "port") port = atoi(v.c_str());
            else if (k == "peer") peer = v;
            else if (k == "host") host = v;
            else if (k == "players") players = atoi(v.c_str());
            else if (k == "latency") latency = atoi(v.c_str());
            else if (k == "jitter") jitter = atoi(v.c_str());
            else if (k == "loss") loss = atoi(v.c_str());
            else if (k == "pace") pace = atoi(v.c_str());
            else if (k == "adaptive") adaptive = atoi(v.c_str());
            else if (k == "ff")
                for (size_t q = 0; q < v.size();)
                {
                    size_t sl = v.find('/', q), c = v.find(':', q);
                    if (c < sl) ffPlan[atoi(v.substr(q, c - q).c_str())] = (float)atof(v.substr(c + 1, sl - c - 1).c_str());
                    if (sl == std::string::npos) break;
                    q = sl + 1;
                }
            if (end == std::string::npos) break;
            pos = end + 1;
        }
        if (players == 0 && peer.empty()) peer = "127.0.0.1:7101";
    }

#ifdef LITEV_HOSTED_NETPLAY
    // LITEV_HOSTED="players=N,port=X[,play][,delay=D][,dropms=MS][,latency=MS][,jitter=MS][,loss=PCT]": Hosted
    // Netplay server. Emulates every console (as Netplay does) and sends each console's records to
    // the replica client that runs it (--replay-console, LITEV_HOSTED=host=...), whose input comes
    // back here, applied D frames late. play: this device also plays console 0 (input from
    // --mp-script0, applied at once); else a dedicated server. Session setup on TCP X + 1, inputs
    // on UDP X, records on UDP X + 2. A guest whose input is dropms (3000) late is dropped: its
    // console plays on with nothing pressed. latency/jitter/loss: simulated network faults on receive.
    // LITEV_MP_RECORD=<dir>: write each console's records to <dir>/console<K>.rec.
    const char* hostedSpec = getenv("LITEV_HOSTED");
    const char* recordDir = getenv("LITEV_MP_RECORD");
    auto hosted = ParseSpec(hostedSpec);
    const bool hostedPlay = hosted.count("play") > 0;
    if (hostedSpec) players = SpecInt(hosted, "players", 2);
    if (hostedSpec && np) { fprintf(stderr, "error: LITEV_HOSTED and LITEV_NETPLAY\n"); return 1; }
    if (hostedSpec && players > kHostedServerId) { fprintf(stderr, "error: Hosted Netplay: at most %d consoles\n", kHostedServerId); return 1; }
#else
    const char* hostedSpec = nullptr;
    auto hosted = ParseSpec(nullptr);
    const char* recordDir = nullptr;
    const bool hostedPlay = false;
#endif

    // LITEV_MP_PLAYERS=N: N consoles (default 2; Netplay: its player count)
    int n = players ? players : getenv("LITEV_MP_PLAYERS") ? atoi(getenv("LITEV_MP_PLAYERS")) : 2;
    if (n < 2 || n > LockstepMP::kMaxInst) { fprintf(stderr, "error: %d instances (2..%d)\n", n, LockstepMP::kMaxInst); return 1; }
    const u16 allMask = (u16)((1u << n) - 1);

    // Each console's ROM: --romK (default --rom). With session setup (Netplay with players=, the
    // Hosted server) this process's own console keeps its ROM, and every other console boots the
    // ROM its player sent, found by hash among the ROMs this process was given (--rom, --romK);
    // when one is missing, the setup fails on every process.
    std::vector<std::string> romPaths(n);
    for (int k = 0; k < n; k++) romPaths[k] = cfg.RomFor(k);
    NetplaySetup setup;
    if (hostedSpec || (np && players))
    {
        std::vector<std::string> library {cfg.rom};
        for (const std::string& r : cfg.roms) if (!r.empty()) library.push_back(r);
        setup.RomPath = romPaths[hostedSpec ? 0 : player];
        setup.Rom = NetplayDescribeRom(setup.RomPath);
        setup.FindRom = [library](const NetplayRom& r) { return NetplayFindRom(r, library); };
        NetplayHarnessXfer(setup);
        if (hostedSpec)
        {
            setup.Player = 0;
            setup.NumPlayers = n;
            setup.Port = SpecInt(hosted, "port", 7100);
            setup.Delay = SpecInt(hosted, "delay", 0);
            setup.Hosted = true;
            setup.HostPlays = hostedPlay;
        }
        else
        {
            setup.Player = player;
            setup.NumPlayers = players;
            setup.Port = port;
            setup.Host = host;
            setup.Save.assign(1000 + player, (u8)(0xA0 + player)); // a recognisable stand-in save
            setup.Delay = delay > 0 ? delay : 0;
        }
        bool ok = NetplayHandshake(setup);
        printf("%s", setup.Log.c_str());
        if (!ok) { fprintf(stderr, "%s: session setup failed\n", hostedSpec ? "hosted" : "netplay"); return 1; }
        for (auto& [p, path] : setup.RomPaths)
            if (p < n) romPaths[p] = path;
    }

    std::vector<BuiltNDS> b(n);
    std::vector<std::string> instScripts(n);
    for (int k = 0; k < n; k++)
    {
        TraceRunConfig c = cfg;
        c.rom = romPaths[k];
        c.instanceTag = "mp" + std::to_string(k);
        // Per-instance scripts (host vs client differ). Fall back to cfg.inputScript.
        c.inputScript = (k < (int)scripts.size() && !scripts[k].empty()) ? scripts[k] : cfg.inputScript;
        instScripts[k] = c.inputScript;
        std::string err;
        if (!BuildAndBoot(c, true, b[k], err)) { fprintf(stderr, "error (mp%d): %s\n", k, err.c_str()); return 1; }
        printf("console %d rom: %s (xxh3 %016llx)\n", k, c.rom.c_str(), (unsigned long long)b[k].romHash);
    }
    // Instance 0 knobs act on instance 0; the "1" knobs on every other instance (with two
    // instances: instance 1, as before).
    // LITEV_MP_NORENDER1: instance 1 draws nothing; LITEV_MP_NORENDER0: instance 0 too
    if (getenv("LITEV_MP_NORENDER0"))
    {
        b[0].nds->GPU.SetRenderer(std::make_unique<NullRenderer>(b[0].nds->GPU));
        printf("instance 0: renderer off\n");
    }
#ifdef LITEV_EVENT_TRACE
    if (const char* ev = getenv("LITEV_MP_EVTRACE"))
    {
        gEvTrace = fopen(ev, "w");
        gEvTraceNDS = b[1].nds.get();
        melonDS::LitevEventTrace = [](NDS* nd, int id, u64 et, u64 st) {
            if (nd == gEvTraceNDS && gEvTraceOn.load(std::memory_order_relaxed))
                fprintf(gEvTrace, "%llu %d %llu pc9=%08x pc7=%08x\n", (unsigned long long)et, id, (unsigned long long)st, nd->ARM9.R[15], nd->ARM7.R[15]);
        };
    }
#endif
    for (int k = 1; k < n; k++)
    {
        NDS& nds = *b[k].nds;
        if (getenv("LITEV_MP_ACC3D1")) // the reference SoftRenderer3D instead of the tile renderer
        {
            RendererSettings rs{};
            rs.ScaleFactor = 1;
            rs.Accurate3D = true;
            nds.GPU.GetRenderer().SetRenderSettings(rs);
            printf("instance %d: accurate 3D renderer\n", k);
        }
        if (getenv("LITEV_MP_NO3D1"))
        {
            nds.GPU.SetRenderer(std::make_unique<SoftNo3DRenderer>(nds));
            printf("instance %d: 3D renderer off\n", k);
        }
        if (getenv("LITEV_MP_NORENDER1"))
        {
            nds.GPU.SetRenderer(std::make_unique<NullRenderer>(nds.GPU));
            printf("instance %d: renderer off\n", k);
        }
        if (getenv("LITEV_MP_SILENT1"))
        {
            nds.SPU.Silent = true;  // what Netplay does to the other players' consoles
            printf("instance %d: silent (no audio mix)\n", k);
        }
#ifdef LITEV_REMOTE_GX_SINK
        if (getenv("LITEV_MP_GXSINK1"))   // =<frame>: what Netplay does to the other players' consoles
            printf("instance %d: geometry sink from frame %d unless the game uses geometry results\n", k, atoi(getenv("LITEV_MP_GXSINK1")));
#endif
#ifdef LITEV_AGGRESSIVE_SKIP
        if (getenv("LITEV_MP_HEADLESS1"))
        {
            nds.GPU.Headless = true; // what Netplay does to the other players' consoles
            nds.GPU.GPU3D.Headless = true;
            printf("instance %d: headless (draws only once the game uses display capture)\n", k);
        }
#endif
    }
#ifdef LITEV_SKIP_REPEAT_FRAMES
    if (getenv("LITEV_MP_SKIPREPEAT0"))
    {
        b[0].nds->GPU.SkipRepeatEnabled = true;   // display-only: must not change any hash
        printf("instance 0: skip repeated-3D frames\n");
    }
#endif

#ifdef LITEV_REMOTE_GX_SINK
    if (getenv("LITEV_MP_GXSINK1") || getenv("LITEV_MP_GXTIMING"))   // Netplay: every console, local too
        for (int k = 0; k < n; k++) b[k].nds->GPU.GPU3D.TimingFixed = true;
#endif
    // Install one shared in-process link and give each instance a distinct id.
    // LITEV_MP_LOCKSTEP=1: the deterministic LockstepMP (Netplay) instead of LocalMP. Netplay
    // always uses it, as the app does: LocalMP's replies depend on thread timing, so two devices
    // running the same consoles diverge.
    bool lockstep = np || getenv("LITEV_MP_LOCKSTEP") != nullptr || hostedSpec || recordDir;
    MPInterface::Set(lockstep ? MPInterface_Netplay : MPInterface_Local);
    LockstepMP* lockstepMP = lockstep ? dynamic_cast<LockstepMP*>(&MPInterface::Get()) : nullptr;
    printf("link: %s\n", lockstep ? "LockstepMP (deterministic)" : "LocalMP");
#ifdef LITEV_HOSTED_NETPLAY
    // Hosted Netplay: the same link, with what it returns to each console recorded
    RecordMP* record = nullptr;
    std::vector<FILE*> recordFiles(n, nullptr);
    std::unique_ptr<HostedServer> hostedServer;
    if (hostedSpec || recordDir)
    {
        auto r = std::make_unique<RecordMP>(std::make_unique<LockstepMP>());
        record = r.get();
        lockstepMP = &r->Link();
        MPInterface::Set(std::move(r), MPInterface_Netplay);
        printf("link: recording each console's link results\n");
    }
    for (int k = 0; recordDir && k < n; k++)
    {
        std::string path = std::string(recordDir) + "/console" + std::to_string(k) + ".rec";
        recordFiles[k] = fopen(path.c_str(), "wb");
        if (!recordFiles[k]) { fprintf(stderr, "error: cannot write %s\n", path.c_str()); return 1; }
    }
#endif

    std::unique_ptr<NetplayInput> net;
#ifdef LITEV_HOSTED_NETPLAY
    if (hostedSpec)
    {
        int hport = setup.Port;
        for (auto& [p, addr] : setup.Peers)
            if (p >= n) { fprintf(stderr, "hosted: player %d outside 0..%d\n", p, n - 1); return 1; }
        for (auto& [p, save] : setup.Saves)   // each guest's save in its console, as the app host does
            if (p < n && !save.empty()) { b[p].nds->SetNDSSave(save.data(), (u32)save.size()); printf("hosted: console %d: guest's save, %zu bytes\n", p, save.size()); }
        NetFaults faults = FaultsFrom(hosted);
        net = std::make_unique<NetplayInput>(kHostedServerId, setup.Delay, hport, setup.Peers, faults.LatencyMs, faults);
        net->DropAfterMs = SpecInt(hosted, "dropms", 3000);
        hostedServer = std::make_unique<HostedServer>(hport + 2, faults);
        if (!net->Ok() || !hostedServer->Ok()) { fprintf(stderr, "hosted: socket setup failed\n"); return 1; }
        printf("hosted: server for %d consoles (%s), delay %d frames, ports %d (inputs) %d (records), faults latency %d jitter %d loss %d%%\n",
               n, hostedPlay ? "plays console 0" : "dedicated", setup.Delay, hport, hport + 2, faults.LatencyMs, faults.JitterMs, faults.LossPct);
    }
    else
#endif
    if (np && players == 0)
    {
        if (delay < 0) delay = 3;
        NetFaults faults;
        faults.JitterMs = jitter;
        faults.LossPct = loss;
        net = std::make_unique<NetplayInput>(player, delay, port, std::vector<std::pair<int, std::string>> {{1 - player, peer}}, latency, faults);
        if (!net->Ok()) { fprintf(stderr, "netplay: socket setup failed\n"); return 1; }
        printf("netplay: player %d, delay %d frames, port %d, peer %s, faults latency %d jitter %d loss %d%%\n", player, delay, port, peer.c_str(), latency, jitter, loss);
    }
    else if (np)
    {
        for (auto& [p, save] : setup.Saves)
        {
            bool expected = save.size() == 1000u + p && std::all_of(save.begin(), save.end(), [p = p](u8 v) { return v == 0xA0 + p; });
            printf("netplay: player %d's save: %zu bytes, %s\n", p, save.size(), expected ? "as sent" : "WRONG");
        }
        if ((int)setup.Peers.size() != n - 1) { fprintf(stderr, "netplay: %zu peers for %d players\n", setup.Peers.size(), n); return 1; }
        for (auto& [p, addr] : setup.Peers)
            if (p >= n) { fprintf(stderr, "netplay: player %d outside 0..%d\n", p, n - 1); return 1; }
        NetFaults faults;
        faults.JitterMs = jitter;
        faults.LossPct = loss;
        net = std::make_unique<NetplayInput>(player, setup.Delay, port, setup.Peers, latency, faults);
        if (!net->Ok()) { fprintf(stderr, "netplay: socket setup failed\n"); return 1; }
        printf("netplay: player %d of %d, delay %d frames, port %d, peers", player, n, setup.Delay, port);
        for (auto& [p, addr] : setup.Peers) printf(" %d@%s", p, addr.c_str());
        printf(", artificial latency %d ms\n", latency);
    }
#ifdef LITEV_NP_ADAPTIVE_DELAY
    if (net && np) net->Adaptive = adaptive != 0;   // plain Netplay, on unless adaptive=0
#endif
    for (int k = 0; k < n; k++)
    {
        if (lockstepMP)
        {
            NDS* nd = b[k].nds.get();
#ifdef LITEV_HOSTED_NETPLAY
            if (record) record->SetClock(k, [nd] { return nd->GetSysTimestamp(); });
            else
#endif
            lockstepMP->SetClock(k, [nd] { return nd->GetSysTimestamp(); });
            lockstepMP->SetWake(k, *nd);
        }
        b[k].udata->instanceID = k;
        // Distinct MAC per instance so they associate as different wireless players.
        if (k && !getenv("LITEV_APP_FW"))
        {
            Firmware& fw = b[k].nds->GetFirmware();
            fw.GetHeader().MacAddr[5] ^= (u8)k;
            fw.UpdateChecksums();
        }
    }

    printf("=== liteDS-headless mp-test (%d instances) ===\n", n);
    printf("rom:      %s\n", cfg.rom.c_str());
    printf("frames:   %d\n", frames);
    for (int k = 0; k < n; k++) printf("script%d:  %s\n", k, instScripts[k].c_str());
    fflush(stdout);

    std::unique_ptr<std::atomic<int>[]> done(new std::atomic<int>[n]);
    for (int k = 0; k < n; k++) done[k] = 0;
    auto minDone = [&] { int m = INT32_MAX; for (int k = 0; k < n; k++) m = std::min(m, done[k].load()); return m; };
    std::atomic<bool> crashed{false};
    std::atomic<u32>  peakConnected{0};  // highest ConnectedBitmask seen

    // LITEV_MP_DUMP_DIR + LITEV_MP_DUMP_EVERY=N: write both screens of each instance every N frames
    // (instN_fFRAME.ppm, top over bottom) to find where the menus are. Every 60 frames each
    // instance also prints how many 3D frames the game finished (SwapBuffers), i.e. the game's own
    // frame rate, which the emulator frame count does not show.
    const char* dumpDir = getenv("LITEV_MP_DUMP_DIR");
    int dumpEvery = getenv("LITEV_MP_DUMP_EVERY") ? atoi(getenv("LITEV_MP_DUMP_EVERY")) : 0;
    auto runInstance = [&](int inst)
    {
        BuiltNDS& bi = b[inst];
        // LITEV_MP_NICE0 / LITEV_MP_NICE1: nice of console 0's thread / every other console's (the
        // app: EmulatorThread -10, NetplayRemote -16)
        if (const char* nv = getenv(inst ? "LITEV_MP_NICE1" : "LITEV_MP_NICE0"))
            if (setpriority(PRIO_PROCESS, 0, atoi(nv)) != 0) fprintf(stderr, "inst%d: setpriority %s failed\n", inst, nv);
        u32 lastSwaps = 0;
        auto lastT = std::chrono::steady_clock::now();
        try
        {
            // Keep emulating past `frames` until the other instances are done too: they may still
            // need this one's MP frames, and without a partner they stall a full receive timeout per tick.
            for (int f = 0; f < frames || minDone() < frames; f++)
            {
                NetplayFrameInput applied;  // for the Hosted Netplay frame marker
                if (hostedSpec)
                {
                    if (f < frames)
                    {
                        applied = hostedPlay && inst == 0 ? ScriptInput(bi, f) : net->Get(inst, f);
                        ApplyNetInput(*bi.nds, applied);
                    }
                }
                else if (!net)
                {
                    // LITEV_MP_GUEST_DELAY=D: consoles 1.. take their script D frames late, as a
                    // Hosted server with input delay D does (the same route as thor.sh)
                    static const int guestDelay = getenv("LITEV_MP_GUEST_DELAY") ? atoi(getenv("LITEV_MP_GUEST_DELAY")) : 0;
                    if (guestDelay > 0 && inst > 0)
                    {
                        applied = f >= guestDelay ? ScriptInput(bi, f - guestDelay) : NetplayFrameInput {};
                        ApplyNetInput(*bi.nds, applied);
                    }
                    else
                    {
                        bi.ApplyInput(f);
                        applied = ScriptInput(bi, f);
                    }
                }
                else if (f < frames)
                {
                    if (inst == net->LocalPlayer())
                    {
                        NetplayFrameInput local;
                        local.Keys = bi.inputScript.Loaded() ? bi.inputScript.KeyMaskForFrame(f) : 0xFFF;
                        int tx, ty;
                        if (bi.inputScript.HasTouch() && bi.inputScript.TouchForFrame(f, tx, ty)) { local.TouchX = tx; local.TouchY = ty; }
#ifdef LITEV_NP_SPEED
                        auto plan = ffPlan.upper_bound(f);
                        if (plan != ffPlan.begin()) NetplaySpeed::Set(local, NetplaySpeed::Encode(std::prev(plan)->second));
#endif
                        net->SubmitLocal(f, local);
                    }
                    NetplayFrameInput in = net->Get(inst, f);
                    bi.nds->SetKeyMask(in.Keys);
                    if (in.TouchX >= 0) bi.nds->TouchScreen(in.TouchX, in.TouchY);
                    else                bi.nds->ReleaseScreen();
                    applied = in;
                }
#ifdef LITEV_EVENT_TRACE
                if (inst == 1 && getenv("LITEV_MP_EVTRACE_FROM")) gEvTraceOn = f + 1 >= atoi(getenv("LITEV_MP_EVTRACE_FROM"));
#endif
#ifdef LITEV_REMOTE_GX_SINK
                static const int sinkFrom = getenv("LITEV_MP_GXSINK1") ? atoi(getenv("LITEV_MP_GXSINK1")) : -1;
                if (inst > 0 && f == sinkFrom && !GPU3D::SinkVeto.load())
                    bi.nds->GPU.GPU3D.Sink = true;
#endif
#ifdef LITEV_NP_SPEED
                auto runT0 = std::chrono::steady_clock::now();
#endif
                bi.nds->RunFrame();
#ifdef LITEV_NP_SPEED
                // per console: emulation wall time per frame (incl. link waits), and for the local
                // console the session speed, waits for the other players' input and pacing sleeps
                static thread_local double runMs = 0, waitMs0 = 0, sleepMs = 0, workEma = 16.667;
                static thread_local auto next = std::chrono::steady_clock::now();
                static thread_local int lastCode = 0;
                double runNow = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - runT0).count();
                runMs += runNow;
                if (net && np && inst == net->LocalPlayer() && f < frames)
                {
                    u32 who = 0;
                    int code = net->SpeedAt(f, &who);
                    if (code != lastCode)
                    {
                        printf("inst%d npspeed switch frame %d: %s requested by 0x%x\n", inst, f, SpeedName(code).c_str(), who);
                        lastCode = code;
                    }
                    float m = NetplaySpeed::Multiplier(code);
#ifdef LITEV_AGGRESSIVE_SKIP
                    // LITEV_NP_FFSKIP=N: while the session runs faster than 1x the local console draws
                    // 1 of N+1 frames, never skipping once the game captured (the app's Netplay fast-forward)
                    static const int ffSkip = getenv("LITEV_NP_FFSKIP") ? atoi(getenv("LITEV_NP_FFSKIP")) : 0;
                    if (ffSkip) { bi.nds->GPU.KeepCaptures = true; bi.nds->GPU.SetFrameskipTarget(code ? ffSkip : 0); }
#endif
                    workEma = workEma * 0.9 + runNow * 0.1;
                    auto now = std::chrono::steady_clock::now();
                    if (pace > 0 && m > 0)
                    {   // no catch-up after a stall (a burst would hide it in the average)
                        next = std::max(next + std::chrono::microseconds((int)(1e6 / (pace * m))), now);
                        std::this_thread::sleep_until(next);
                        sleepMs += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - now).count();
                    }
                    else next = now;
                    net->SetFramePeriodUs(pace > 0 && m > 0 ? (int)(1e6 / (pace * m)) : (int)(workEma * 1000));
                }
                else
#endif
                if (pace > 0 && net && inst == net->LocalPlayer())
                {   // no catch-up after a stall (a burst would hide it in the average)
                    static thread_local auto next = std::chrono::steady_clock::now();
                    next = std::max(next + std::chrono::microseconds(1000000 / pace), std::chrono::steady_clock::now());
                    std::this_thread::sleep_until(next);
                }
#ifdef LITEV_HOSTED_NETPLAY
                if (record && f < frames)
                {
                    record->EndFrame(inst, f, HostedState(*bi.nds, applied, f), applied);
                    std::vector<u8> out;
                    record->TakeRecords(inst, out);
                    if (recordFiles[inst]) fwrite(out.data(), 1, out.size(), recordFiles[inst]);
                    if (hostedServer && !(hostedPlay && inst == 0)) hostedServer->Push(inst, out.data(), out.size());
                }
#endif
                if (f >= frames) continue;
                done[inst].store(f + 1, std::memory_order_relaxed);
                static const int every = getenv("LITEV_MP_EVERY") ? atoi(getenv("LITEV_MP_EVERY")) : 60;
                if (((f + 1) % every) == 0)
                {
                    u32 sw = bi.nds->GPU.GPU3D.SwapCount;
                    auto now = std::chrono::steady_clock::now();
                    printf("inst%d frame %d: sys=%llu pc9=%08x pc7=%08x game 3D frames=%u/60 captures=%u ms=%.0f ram=%016llx\n", inst, f + 1,
                           (unsigned long long)bi.nds->GetSysTimestamp(), bi.nds->ARM9.R[15], bi.nds->ARM7.R[15],
                           sw - lastSwaps, bi.nds->GPU.CaptureCount,
                           std::chrono::duration<double, std::milli>(now - lastT).count(),
                           (unsigned long long)XXH3_64bits(bi.nds->MainRAM, bi.nds->MainRAMMask + 1));
                    if (lockstepMP)   // what the other consoles see of this one: its sent frames
                        printf("inst%d tx %d: %016llx n=%u data=%016llx\n", inst, f + 1, (unsigned long long)lockstepMP->TxHash(inst), lockstepMP->TxCount(inst), (unsigned long long)lockstepMP->TxDataHash(inst));
                    if (getenv("LITEV_MP_STATE"))
                    {
                        // diagnostics: more of the console's state, to find what diverges first
                        GPU& g = bi.nds->GPU;
                        u64 vram = 0;
                        for (int k = 0; k < 9; k++)
                            vram ^= XXH3_64bits(g.VRAM[k], g.VRAMMask[k] + 1) * (k + 1);
                        printf("inst%d state %d: vram=%016llx gxstat=%08x fifo=%u polys=%u verts=%u arm9r=%016llx arm7r=%016llx\n", inst, f + 1,
                               (unsigned long long)vram, g.GPU3D.GXStat, g.GPU3D.FifoLevel(), g.GPU3D.NumPolygons, g.GPU3D.NumVertices,
                               (unsigned long long)XXH3_64bits(bi.nds->ARM9.R, sizeof(bi.nds->ARM9.R)),
                               (unsigned long long)XXH3_64bits(bi.nds->ARM7.R, sizeof(bi.nds->ARM7.R)));
                    }
                    if (lockstepMP && getenv("LITEV_MP_STATS"))
                    {   // per console: CPU, link calls / blocked waits / blocked ms per frame (Packet, Host, Replies)
                        timespec ts; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
                        double cpu = ts.tv_sec * 1e3 + ts.tv_nsec / 1e6;
                        static thread_local double lastCpu = 0;
                        LockstepMP::WaitStats w = lockstepMP->TakeWaitStats(inst);
                        double fr = every;
                        printf("inst%d wait %d: cpu %.2f ms/f | pkt %.0f calls %.1f blocked %.2f ms | host %.0f calls %.1f blocked %.2f ms | replies %.1f calls %.1f blocked %.2f ms | wakeups %.1f /f | host lock contended %.1f /f %.3f ms/f | missed wakes %.2f /f\n",
                               inst, f + 1, (cpu - lastCpu) / fr, w.Calls[0] / fr, w.Blocked[0] / fr, w.Ns[0] / 1e6 / fr,
                               w.Calls[1] / fr, w.Blocked[1] / fr, w.Ns[1] / 1e6 / fr, w.Calls[2] / fr, w.Blocked[2] / fr, w.Ns[2] / 1e6 / fr, w.Wakeups / fr, w.Contended / fr, w.LockNs / 1e6 / fr, w.Missed / fr);
                        lastCpu = cpu;
                    }
#ifdef LITEV_NP_SPEED
                    if (net && np)
                    {
                        timespec ts; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
                        double cpu = ts.tv_sec * 1e3 + ts.tv_nsec / 1e6;
                        static thread_local double lastCpu = 0, lastWait = 0;
                        if (inst == net->LocalPlayer())
                        {
                            double w = net->PeerWaitMs();
                            printf("inst%d npspeed %d: speed %s fps %.1f | run %.2f cpu %.2f ms/f | wait for peers %.2f ms/f | sleep %.2f ms/f | delay %d\n", inst, f + 1,
                                   SpeedName(lastCode).c_str(), every * 1000.0 / std::chrono::duration<double, std::milli>(now - lastT).count(),
                                   runMs / every, (cpu - lastCpu) / every, (w - lastWait) / every, sleepMs / every, net->CurrentDelay());
                            lastWait = w;
                        }
                        else printf("inst%d npcost %d: run %.2f cpu %.2f ms/f\n", inst, f + 1, runMs / every, (cpu - lastCpu) / every);
                        lastCpu = cpu; runMs = sleepMs = 0;
                    }
#endif
                    if (net && np && inst == net->LocalPlayer())
                        printf("inst%d net %d: delay %d frames, peer round trip %.1f ms, stalled %.1f s\n", inst, f + 1, net->CurrentDelay(), net->PeerRttMs(), net->StallMs() / 1000);
                    lastT = now;
                    fflush(stdout);
                    lastSwaps = sw;
                }
                if (getenv("LITEV_MP_DUMP7") && inst == 1 && f + 1 == atoi(getenv("LITEV_MP_DUMP7")))
                {   // diagnostics: ARM7 WRAM (0x037F8000, 64 KB) to disassemble the hot ARM7 code
                    if (FILE* fp = fopen("arm7wram.bin", "wb")) { fwrite(bi.nds->ARM7WRAM, 1, 0x10000, fp); fclose(fp); }
                }
                if (dumpDir && dumpEvery > 0 && ((f + 1) % dumpEvery) == 0)
                {
                    void* top = nullptr; void* bot = nullptr;
                    if (bi.nds->GPU.GetFramebuffers(&top, &bot) && top && bot)
                    {
                        char path[512];
                        snprintf(path, sizeof(path), "%s/inst%d_f%05d.ppm", dumpDir, inst, f + 1);
                        if (FILE* fp = fopen(path, "wb"))
                        {
                            fprintf(fp, "P6\n256 384\n255\n");
                            for (const void* scr : { top, bot })
                                for (int i = 0; i < 256 * 192; i++)
                                {
                                    u32 px = ((const u32*)scr)[i];
                                    u8 rgb[3] = { (u8)(px >> 16), (u8)(px >> 8), (u8)px };   // 0xFFRRGGBB
                                    fwrite(rgb, 1, 3, fp);
                                }
                            fclose(fp);
                        }
                    }
                }
            }
        }
        catch (...) { crashed.store(true); }
        // leave the link, so a peer still finishing a frame does not wait on this clock forever
        if (lockstepMP) { lockstepMP->End(inst); lockstepMP->Stop(); }
    };

    std::vector<std::thread> threads;
    auto wall0 = std::chrono::steady_clock::now();
    double cpu0 = CpuMs();
    for (int k = 0; k < n; k++) threads.emplace_back(runInstance, k);

    // Poll MP health from the main thread while the instances run, and log the
    // first frame all of them associate (ConnectedBitmask == every instance).
    int firstAssocFrame = -1;
    while (minDone() < frames)
    {
        u16 mask = MPInterface::Get().ObserveConnectedBitmask();
        if (mask > peakConnected.load()) peakConnected.store(mask);
        if (mask == allMask && firstAssocFrame < 0)
        {
            firstAssocFrame = minDone();
            printf("  [assoc] ConnectedBitmask=0x%x at ~frame %d (cmd=%llu reply=%llu pkt=%llu)\n",
                   mask, firstAssocFrame,
                   (unsigned long long)MPInterface::Get().ObserveCmdCount(),
                   (unsigned long long)MPInterface::Get().ObserveReplyCount(),
                   (unsigned long long)MPInterface::Get().ObservePacketCount());
            fflush(stdout);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    for (std::thread& t : threads) t.join();
    {
        double wall = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wall0).count();
        printf("perf: %d frames, wall %.2f ms/frame, process CPU %.2f ms/frame (%d consoles)\n", frames, wall / frames, (CpuMs() - cpu0) / frames, n);
#ifdef LITEV_HOSTED_NETPLAY
        for (int k = 0; hostedServer && k < n; k++)
        {
            u64 sent, resent;
            hostedServer->Traffic(k, sent, resent);
            if (sent) printf("hosted: console %d stream: %.0f KB sent (%.0f KB re-sent), %.0f B/frame, %.1f KB/s\n", k, sent / 1024.0, resent / 1024.0,
                             (double)sent / frames, sent / 1024.0 / (wall / 1000));
        }
#endif
    }
#ifdef LITEV_HOSTED_NETPLAY
    for (FILE* f : recordFiles) if (f) fclose(f);
    hostedServer.reset();   // every client has its whole stream
#endif

    u64 cmd = MPInterface::Get().ObserveCmdCount();
    u64 reply = MPInterface::Get().ObserveReplyCount();
    u64 pkt = MPInterface::Get().ObservePacketCount();
    if (net) printf("netplay stall: %.1f ms waiting for the remote players' input\n", net->StallMs());
    bool ranClean = !crashed.load() && minDone() == frames;
    bool associated = (peakConnected.load() == allMask);
    bool exchanged = (cmd > 0 || reply > 0 || pkt > 0);

    for (int k = 0; k < n; k++) printf("mp%d_frames:      %d\n", k, done[k].load());
    printf("peak_connected:  0x%x\n", peakConnected.load());
    printf("first_assoc_frame: %d\n", firstAssocFrame);
    printf("cmd_frames:      %llu\n", (unsigned long long)cmd);
#if LITEV_PROFILE
    {   // JIT transition counters over the whole run, all consoles (LITEV_PROFILE builds)
        using namespace melonDS::LiteProfile;
        auto v = [](std::atomic<uint64_t>& a) { return (unsigned long long)a.load(); };
        printf("jit: cpp_reentries=%llu dispatcher_hits=%llu icache_hits=%llu commit_stub=%llu"
#ifdef LITEV_JIT_DIRECTPATCH
               " direct_guard_hits=%llu"
#endif
               "\n", v(g_Frame.CppReentries), v(g_Frame.DispatcherHits), v(g_Frame.ICacheHits), v(g_Frame.CommitStubEntries)
#ifdef LITEV_JIT_DIRECTPATCH
               , v(g_Frame.DirectGuardHits)
#endif
               );
    }
#endif
    printf("reply_frames:    %llu\n", (unsigned long long)reply);
    printf("packets:         %llu\n", (unsigned long long)pkt);
    printf("ran_clean:       %s\n", ranClean ? "yes" : "no");
    printf("associated:      %s\n", associated ? "yes" : "no");
    printf("exchanged:       %s\n", exchanged ? "yes" : "no");
    fflush(stdout);
    // Success (for now, the Phase-2 milestone) = ran clean AND every instance associated.
    return (ranClean && associated) ? 0 : 1;
}

// ---------------------------------------------------------------------------
// --replay-console K : Hosted Netplay replica of --mp-test's console K.
// ---------------------------------------------------------------------------
#ifdef LITEV_HOSTED_NETPLAY
int ReplayConsole(const TraceRunConfig& cfg, int frames, const std::vector<std::string>& scripts, int k, const std::string& logDir)
{
    if (k < 0 || k >= kHostedServerId) { fprintf(stderr, "error: --replay-console %d (0..%d)\n", k, kHostedServerId - 1); return 1; }
    // LITEV_HOSTED="host=IP:PORT,port=N[,latency=MS][,jitter=MS][,loss=PCT]" (no --replay-log):
    // join the server at IP:PORT (its LITEV_HOSTED port); inputs from UDP port N.
    auto spec = ParseSpec(getenv("LITEV_HOSTED"));
    if (logDir.empty() && !spec.count("host")) { fprintf(stderr, "error: --replay-console needs --replay-log or LITEV_HOSTED=host=...\n"); return 1; }

    // console K exactly as --mp-test builds it
    TraceRunConfig c = cfg;
    c.rom = cfg.RomFor(k);
    c.instanceTag = "mp" + std::to_string(k);
    c.inputScript = (k < (int)scripts.size() && !scripts[k].empty()) ? scripts[k] : cfg.inputScript;
    BuiltNDS b;
    std::string err;
    if (!BuildAndBoot(c, true, b, err)) { fprintf(stderr, "error: %s\n", err.c_str()); return 1; }
    b.udata->instanceID = k;
    if (k && !getenv("LITEV_APP_FW"))
    {
        Firmware& fw = b.nds->GetFirmware();
        fw.GetHeader().MacAddr[5] ^= (u8)k;
        fw.UpdateChecksums();
    }
    if (getenv("LITEV_MP_NORENDER0")) b.nds->GPU.SetRenderer(std::make_unique<NullRenderer>(b.nds->GPU));

    auto replayOwned = std::make_unique<ReplayMP>(k);
    ReplayMP* replay = replayOwned.get();
    NDS* nd = b.nds.get();
    replay->SetClock([nd] { return nd->GetSysTimestamp(); });
    MPInterface::Set(std::move(replayOwned), MPInterface_Netplay);

    std::unique_ptr<NetplayInput> net;
    std::unique_ptr<HostedClient> client;
    if (!logDir.empty())
    {
        u32 len = 0;
        std::string path = logDir + "/console" + std::to_string(k) + ".rec";
        auto data = ReadFile(path, len);
        if (!data) { fprintf(stderr, "error: cannot read %s\n", path.c_str()); return 1; }
        replay->Feed(data.get(), len);
        printf("replay: console %d from %s (%u bytes)\n", k, path.c_str(), len);
    }
    else
    {
        NetplaySetup setup;
        setup.Player = k;
        setup.Port = SpecInt(spec, "port", 7110 + k);
        setup.Host = spec["host"];
        setup.Join = true;
        setup.RomPath = c.rom;                   // a replica needs only its own ROM
        setup.Rom = NetplayDescribeRom(c.rom);
        NetplayHarnessXfer(setup);
        bool ok = NetplayHandshake(setup);
        printf("%s", setup.Log.c_str());
        if (!ok) { fprintf(stderr, "hosted: session setup failed\n"); return 1; }
        NetFaults faults = FaultsFrom(spec);
        net = std::make_unique<NetplayInput>(k, setup.Delay, setup.Port, std::vector<std::pair<int, std::string>>{{kHostedServerId, setup.Host}}, faults.LatencyMs, faults);
        std::string host = setup.Host.substr(0, setup.Host.rfind(':'));
        int hport = atoi(setup.Host.substr(setup.Host.rfind(':') + 1).c_str());
        client = std::make_unique<HostedClient>(k, host + ":" + std::to_string(hport + 2), [replay](const u8* d, size_t l) { replay->Feed(d, l); }, faults);
        if (!net->Ok() || !client->Ok()) { fprintf(stderr, "hosted: socket setup failed\n"); return 1; }
        NetplayInput* n = net.get();
        replay->SetServerSilence([n] { return n->MsSincePeer(); });
        printf("hosted: replica of console %d, server %s, delay %d frames, faults latency %d jitter %d loss %d%%\n",
               k, setup.Host.c_str(), setup.Delay, faults.LatencyMs, faults.JitterMs, faults.LossPct);
    }

    // LITEV_REPLAY_INJECT=<frame>: negative test, toggle the A button at that frame on this replica
    // only. LITEV_REPLAY_INJECT_HIDDEN=1: the frame marker gets the uninjected input (a divergence
    // the input check cannot see, so the call hash / clock / RAM must catch it).
    int inject = getenv("LITEV_REPLAY_INJECT") ? atoi(getenv("LITEV_REPLAY_INJECT")) : -1;
    bool hidden = getenv("LITEV_REPLAY_INJECT_HIDDEN") != nullptr;
    int every = getenv("LITEV_MP_EVERY") ? atoi(getenv("LITEV_MP_EVERY")) : 60;
    // LITEV_REPLAY_SERVER_INPUT=1 (--replay-log): the input the server applied (from its frame
    // markers) instead of the script, as a second replica of a console must
    bool follow = !net && getenv("LITEV_REPLAY_SERVER_INPUT");

    int firstDesync = -1, lostAt = -1;
    bool dropped = false;
    auto wall0 = std::chrono::steady_clock::now();
    double cpu0 = CpuMs();
    double maxFrameMs = 0;
    for (int f = 0; f < frames; f++)
    {
        auto t0 = std::chrono::steady_clock::now();
        NetplayFrameInput in;
        if (net)
        {
            NetplayFrameInput local = ScriptInput(b, f);
            if (!b.inputScript.Loaded()) local.Keys = 0xFFF;
            net->SubmitLocal(f, local);
            in = net->Get(k, f);
        }
        else
            in = ScriptInput(b, f);
        // live: the server has not acknowledged our input for this frame yet, so it may have
        // dropped this player: apply what it applied (from its frame marker)
        NetplayFrameInput srv;
        if ((follow || (net && f >= net->Delay() && f > net->AckedBy(kHostedServerId))) && replay->ServerInput(f, srv, true))
        {
            if (net && !dropped && (srv.Keys != in.Keys || srv.TouchX != in.TouchX || srv.TouchY != in.TouchY))
            {
                printf("hosted: the server dropped this player (frame %d): its console plays on with the server's input\n", f);
                dropped = true;
            }
            in = srv;
        }
        NetplayFrameInput marker = in;
        if (f == inject) { in.Keys ^= 1; if (!hidden) marker = in; printf("replay: injected A toggle at frame %d%s\n", f, hidden ? " (hidden from the marker)" : ""); }
        if (net || follow) ApplyNetInput(*b.nds, in);
        else
        {
            b.ApplyInput(f);
            if (f == inject) b.nds->SetKeyMask(in.Keys);
        }
        b.nds->RunFrame();
        bool ok = replay->EndFrame(f, HostedState(*b.nds, marker, f));
        if (const char* sh = getenv("LITEV_REPLAY_STATEHASH")) // diagnostics: full-state hash after these frames ("f1,f2,...")
            if (("," + std::string(sh) + ",").find("," + std::to_string(f) + ",") != std::string::npos)
            {
                Savestate st;
                b.nds->DoSavestate(&st);
                printf("statehash frame %d: %016llx (%u bytes)\n", f, (unsigned long long)XXH3_64bits(st.Buffer(), st.Length()), st.Length());
            }
        maxFrameMs = std::max(maxFrameMs, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
        if (((f + 1) % every) == 0)
        {
            printf("inst%d frame %d: sys=%llu ram=%016llx\n", k, f + 1, (unsigned long long)b.nds->GetSysTimestamp(),
                   (unsigned long long)XXH3_64bits(b.nds->MainRAM, b.nds->MainRAMMask + 1));
            fflush(stdout);
        }
        if (!ok && replay->Lost())
        {
            lostAt = f;
            printf("hosted: session ended at frame %d: %s\n", f, replay->Error().c_str());
            break;
        }
        if (!ok)
        {
            firstDesync = f;
            printf("DESYNC at frame %d: %s\n", f, replay->Error().c_str());
            break;
        }
    }
    double wall = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wall0).count();
    printf("perf: wall %.2f ms/frame (max %.1f), process CPU %.2f ms/frame, waited for records %.0f ms, for input %.0f ms\n",
           wall / frames, maxFrameMs, (CpuMs() - cpu0) / frames, replay->StallMs(), net ? net->StallMs() : 0.0);
    printf("replay: %s\n", firstDesync >= 0 ? ("DESYNC first at frame " + std::to_string(firstDesync)).c_str()
                          : lostAt >= 0 ? ("MATCH until the server was lost at frame " + std::to_string(lostAt)).c_str() : "MATCH (every frame)");
    fflush(stdout);
    replay->Stop();
    if (net) net->Abort();
    return firstDesync >= 0 ? 2 : lostAt >= 0 ? 3 : 0;
}
#else
int ReplayConsole(const TraceRunConfig&, int, const std::vector<std::string>&, int, const std::string&)
{
    fprintf(stderr, "error: --replay-console needs a LITEV_HOSTED_NETPLAY build\n");
    return 1;
}
#endif

} // namespace liteds
