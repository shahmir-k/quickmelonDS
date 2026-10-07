/*
    liteDS-v2 headless harness - trace recording / verification / convergence.
    See VerifyTrace.h for the oracle rationale.
*/

#include <cstdlib>
#include "VerifyTrace.h"
#include "LockstepMP.h"
#include "NetplayInput.h"

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

// Build, reset, direct-boot and start an NDS from `cfg`. Returns false + err on
// failure. `jitOverride`, when set, wins over cfg.jit (used by the converge
// path to build one JIT and one interp instance from the same config).
bool BuildAndBoot(const TraceRunConfig& cfg, std::optional<bool> jitOverride,
                  BuiltNDS& out, std::string& err)
{
    u32 romlen = 0;
    auto romdata = ReadFile(cfg.rom, romlen);
    if (!romdata) { err = "cannot read ROM '" + cfg.rom + "'"; return false; }

    out.romHash = XXH3_64bits(romdata.get(), romlen);
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

    out.udata = std::make_unique<HeadlessHost::InstanceUserData>();
    out.udata->savePrefix = cfg.instanceTag;

    out.nds = std::make_unique<NDS>(std::move(args), out.udata.get());
    out.nds->SetRenderer(std::make_unique<SoftRenderer>(*out.nds));
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

int MPTest(const TraceRunConfig& cfg, int frames,
           const std::string& script0, const std::string& script1)
{
    TraceRunConfig c0 = cfg; c0.instanceTag = "mp0";
    TraceRunConfig c1 = cfg; c1.instanceTag = "mp1";
    // Per-instance scripts (host vs client differ). Fall back to cfg.inputScript.
    c0.inputScript = script0.empty() ? cfg.inputScript : script0;
    c1.inputScript = script1.empty() ? cfg.inputScript : script1;

    BuiltNDS b0, b1;
    std::string err;
    if (!BuildAndBoot(c0, true, b0, err)) { fprintf(stderr, "error (mp0): %s\n", err.c_str()); return 1; }
    if (!BuildAndBoot(c1, true, b1, err)) { fprintf(stderr, "error (mp1): %s\n", err.c_str()); return 1; }
    // LITEV_MP_NORENDER1: instance 1 draws nothing; LITEV_MP_NORENDER0: instance 0 too
    if (getenv("LITEV_MP_NORENDER0"))
    {
        b0.nds->GPU.SetRenderer(std::make_unique<NullRenderer>(b0.nds->GPU));
        printf("instance 0: renderer off\n");
    }
#ifdef LITEV_EVENT_TRACE
    if (const char* ev = getenv("LITEV_MP_EVTRACE"))
    {
        gEvTrace = fopen(ev, "w");
        gEvTraceNDS = b1.nds.get();
        melonDS::LitevEventTrace = [](NDS* n, int id, u64 et, u64 st) {
            if (n == gEvTraceNDS && gEvTraceOn.load(std::memory_order_relaxed))
                fprintf(gEvTrace, "%llu %d %llu pc9=%08x pc7=%08x\n", (unsigned long long)et, id, (unsigned long long)st, n->ARM9.R[15], n->ARM7.R[15]);
        };
    }
#endif
    if (getenv("LITEV_MP_ACC3D1")) // instance 1 on the reference SoftRenderer3D instead of the tile renderer
    {
        RendererSettings rs{};
        rs.ScaleFactor = 1;
        rs.Accurate3D = true;
        b1.nds->GPU.GetRenderer().SetRenderSettings(rs);
        printf("instance 1: accurate 3D renderer\n");
    }
    if (getenv("LITEV_MP_NO3D1"))
    {
        b1.nds->GPU.SetRenderer(std::make_unique<SoftNo3DRenderer>(*b1.nds));
        printf("instance 1: 3D renderer off\n");
    }
    if (getenv("LITEV_MP_NORENDER1"))
    {
        b1.nds->GPU.SetRenderer(std::make_unique<NullRenderer>(b1.nds->GPU));
        printf("instance 1: renderer off\n");
    }

    // Install one shared in-process link and give each instance a distinct id.
    // LITEV_MP_LOCKSTEP=1: the deterministic LockstepMP (Netplay) instead of LocalMP.
    bool lockstep = getenv("LITEV_MP_LOCKSTEP") != nullptr;
    MPInterface::Set(lockstep ? MPInterface_Netplay : MPInterface_Local);
    LockstepMP* lockstepMP = lockstep ? dynamic_cast<LockstepMP*>(&MPInterface::Get()) : nullptr;
    printf("link: %s\n", lockstep ? "LockstepMP (deterministic)" : "LocalMP");

    // LITEV_NETPLAY="player=P,delay=D,port=N,peer=IP:PORT[,latency=MS]": Netplay. This process is
    // player P's device: it emulates both consoles, but only player P's input comes from its
    // script; the other player's arrives over UDP from the peer process, applied D frames late.
    std::unique_ptr<NetplayInput> net;
    if (const char* np = getenv("LITEV_NETPLAY"))
    {
        int player = 0, delay = 3, port = 7100, latency = 0;
        std::string peer = "127.0.0.1:7101", spec = np;
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
            else if (k == "latency") latency = atoi(v.c_str());
            if (end == std::string::npos) break;
            pos = end + 1;
        }
        net = std::make_unique<NetplayInput>(player, delay, port, peer, latency);
        if (!net->Ok()) { fprintf(stderr, "netplay: socket setup failed\n"); return 1; }
        printf("netplay: player %d, delay %d frames, port %d, peer %s, artificial latency %d ms\n", player, delay, port, peer.c_str(), latency);
    }
    if (lockstepMP)
    {
        NDS* n0 = b0.nds.get(); NDS* n1 = b1.nds.get();
        lockstepMP->SetClock(0, [n0] { return n0->GetSysTimestamp(); });
        lockstepMP->SetClock(1, [n1] { return n1->GetSysTimestamp(); });
    }
    b0.udata->instanceID = 0;
    b1.udata->instanceID = 1;

    // Distinct MAC per instance so they associate as different wireless players.
    {
        Firmware& fw = b1.nds->GetFirmware();
        fw.GetHeader().MacAddr[5] ^= 0x01;
        fw.UpdateChecksums();
    }

    printf("=== liteDS-headless mp-test (Phase 2: Shrek 2-player association) ===\n");
    printf("rom:      %s\n", cfg.rom.c_str());
    printf("frames:   %d\n", frames);
    printf("script0:  %s\n", c0.inputScript.c_str());
    printf("script1:  %s\n", c1.inputScript.c_str());
    fflush(stdout);

    std::atomic<int>  done0{0}, done1{0};
    std::atomic<bool> crashed{false};
    std::atomic<u32>  peakConnected{0};  // highest ConnectedBitmask seen

    // LITEV_MP_DUMP_DIR + LITEV_MP_DUMP_EVERY=N: write both screens of each instance every N frames
    // (instN_fFRAME.ppm, top over bottom) to find where the menus are. Every 60 frames each
    // instance also prints how many 3D frames the game finished (SwapBuffers), i.e. the game's own
    // frame rate, which the emulator frame count does not show.
    const char* dumpDir = getenv("LITEV_MP_DUMP_DIR");
    int dumpEvery = getenv("LITEV_MP_DUMP_EVERY") ? atoi(getenv("LITEV_MP_DUMP_EVERY")) : 0;
    auto runInstance = [&](BuiltNDS& b, std::atomic<int>& doneCounter)
    {
        int inst = (&b == &b0) ? 0 : 1;
        std::atomic<int>& otherDone = (inst == 0) ? done1 : done0;
        u32 lastSwaps = 0;
        auto lastT = std::chrono::steady_clock::now();
        try
        {
            // Keep emulating past `frames` until the other instance is done too: it may still need
            // this one's MP frames, and without a partner it stalls a full receive timeout per tick.
            for (int f = 0; f < frames || otherDone.load() < frames; f++)
            {
                if (!net)
                    b.ApplyInput(f);
                else if (f < frames)
                {
                    if (inst == net->LocalPlayer())
                    {
                        NetplayFrameInput local;
                        local.Keys = b.inputScript.Loaded() ? b.inputScript.KeyMaskForFrame(f) : 0xFFF;
                        int tx, ty;
                        if (b.inputScript.HasTouch() && b.inputScript.TouchForFrame(f, tx, ty)) { local.TouchX = tx; local.TouchY = ty; }
                        net->SubmitLocal(f, local);
                    }
                    NetplayFrameInput in = net->Get(inst, f);
                    b.nds->SetKeyMask(in.Keys);
                    if (in.TouchX >= 0) b.nds->TouchScreen(in.TouchX, in.TouchY);
                    else                b.nds->ReleaseScreen();
                }
#ifdef LITEV_EVENT_TRACE
                if (inst == 1 && getenv("LITEV_MP_EVTRACE_FROM")) gEvTraceOn = f + 1 >= atoi(getenv("LITEV_MP_EVTRACE_FROM"));
#endif
                b.nds->RunFrame();
                if (f >= frames) continue;
                doneCounter.store(f + 1, std::memory_order_relaxed);
                static const int every = getenv("LITEV_MP_EVERY") ? atoi(getenv("LITEV_MP_EVERY")) : 60;
                if (((f + 1) % every) == 0)
                {
                    u32 sw = b.nds->GPU.GPU3D.SwapCount;
                    auto now = std::chrono::steady_clock::now();
                    printf("inst%d frame %d: sys=%llu pc9=%08x pc7=%08x game 3D frames=%u/60 captures=%u ms=%.0f ram=%016llx\n", inst, f + 1,
                           (unsigned long long)b.nds->GetSysTimestamp(), b.nds->ARM9.R[15], b.nds->ARM7.R[15],
                           sw - lastSwaps, b.nds->GPU.CaptureCount,
                           std::chrono::duration<double, std::milli>(now - lastT).count(),
                           (unsigned long long)XXH3_64bits(b.nds->MainRAM, b.nds->MainRAMMask + 1));
                    if (getenv("LITEV_MP_STATE"))
                    {
                        // diagnostics: more of the console's state, to find what diverges first
                        GPU& g = b.nds->GPU;
                        u64 vram = 0;
                        for (int k = 0; k < 9; k++)
                            vram ^= XXH3_64bits(g.VRAM[k], g.VRAMMask[k] + 1) * (k + 1);
                        printf("inst%d state %d: vram=%016llx gxstat=%08x fifo=%u polys=%u verts=%u arm9r=%016llx arm7r=%016llx\n", inst, f + 1,
                               (unsigned long long)vram, g.GPU3D.GXStat, g.GPU3D.FifoLevel(), g.GPU3D.NumPolygons, g.GPU3D.NumVertices,
                               (unsigned long long)XXH3_64bits(b.nds->ARM9.R, sizeof(b.nds->ARM9.R)),
                               (unsigned long long)XXH3_64bits(b.nds->ARM7.R, sizeof(b.nds->ARM7.R)));
                    }
                    lastT = now;
                    fflush(stdout);
                    lastSwaps = sw;
                }
                if (dumpDir && dumpEvery > 0 && ((f + 1) % dumpEvery) == 0)
                {
                    void* top = nullptr; void* bot = nullptr;
                    if (b.nds->GPU.GetFramebuffers(&top, &bot) && top && bot)
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
                                    u8 rgb[3] = { (u8)px, (u8)(px >> 8), (u8)(px >> 16) };
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

    std::thread t0(runInstance, std::ref(b0), std::ref(done0));
    std::thread t1(runInstance, std::ref(b1), std::ref(done1));

    // Poll MP health from the main thread while the instances run, and log the
    // first frame the two associate (ConnectedBitmask == 0x3).
    int firstAssocFrame = -1;
    while (done0.load() < frames || done1.load() < frames)
    {
        u16 mask = MPInterface::Get().ObserveConnectedBitmask();
        if (mask > peakConnected.load()) peakConnected.store(mask);
        if (mask == 0x3 && firstAssocFrame < 0)
        {
            firstAssocFrame = std::min(done0.load(), done1.load());
            printf("  [assoc] ConnectedBitmask=0x3 at ~frame %d (cmd=%llu reply=%llu pkt=%llu)\n",
                   firstAssocFrame,
                   (unsigned long long)MPInterface::Get().ObserveCmdCount(),
                   (unsigned long long)MPInterface::Get().ObserveReplyCount(),
                   (unsigned long long)MPInterface::Get().ObservePacketCount());
            fflush(stdout);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    t0.join();
    t1.join();

    u64 cmd = MPInterface::Get().ObserveCmdCount();
    u64 reply = MPInterface::Get().ObserveReplyCount();
    u64 pkt = MPInterface::Get().ObservePacketCount();
    if (net) printf("netplay stall: %.1f ms waiting for the remote player's input\n", net->StallMs());
    bool ranClean = !crashed.load() && done0.load() == frames && done1.load() == frames;
    bool associated = (peakConnected.load() == 0x3);
    bool exchanged = (cmd > 0 || reply > 0 || pkt > 0);

    printf("mp0_frames:      %d\n", done0.load());
    printf("mp1_frames:      %d\n", done1.load());
    printf("peak_connected:  0x%x\n", peakConnected.load());
    printf("first_assoc_frame: %d\n", firstAssocFrame);
    printf("cmd_frames:      %llu\n", (unsigned long long)cmd);
    printf("reply_frames:    %llu\n", (unsigned long long)reply);
    printf("packets:         %llu\n", (unsigned long long)pkt);
    printf("ran_clean:       %s\n", ranClean ? "yes" : "no");
    printf("associated:      %s\n", associated ? "yes" : "no");
    printf("exchanged:       %s\n", exchanged ? "yes" : "no");
    fflush(stdout);
    // Success (for now, the Phase-2 milestone) = ran clean AND the two associated.
    return (ranClean && associated) ? 0 : 1;
}

} // namespace liteds
