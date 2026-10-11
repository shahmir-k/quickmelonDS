// LITEV_A7HLE: see ARM7HLE.h.
#ifdef LITEV_A7HLE
#include "ARM7HLE.h"
#include "ARM.h"
#include "NDS.h"
#include "ARMInterpreter.h"
#include "FreeBIOS.h"
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <vector>
#include <unordered_map>
#include <algorithm>
#if defined(__ANDROID__)
#include <sys/system_properties.h>
#endif

namespace melonDS::A7HLE
{
#ifdef LITEV_HLE_DIAG
thread_local bool CheckPending = false;
#endif
namespace
{
// ---- Pokemon Black/White (NitroSDK ARM7 SND driver in ARM7 WRAM) ---------------------------
// SND_ExChannelMain(BOOL step) at 0x03800A5C (ARM). Signature = FNV-1a 64 over the bytes of
// [0x0380021C,0x03800F48) (ExChannelMain + IsChannelActive, CalcTimer, CalcChannelVolume,
// SinIdx, envelope + literal pools), [0x03801418,0x038014D4) (UpdateLfo, GetLfoValue) and the
// shared-WRAM Thumb SVC stubs [0x037FB574,0x037FB598) (GetPitchTable / GetVolumeTable).
constexpr u32 kChannels = 0x03809FBC, kChanSize = 0x54;     // (PW: the SND hardware commit's)
constexpr u32 kTwlFlag = 0x03FFFFC8;

// SND_ExChannelMain per game build. Mario Kart DS (2005 NitroSDK, older compiler: envelope / sweep / LFO in their own
// functions, the driver in shared WRAM 0x037Fxxxx): the same SNDExChannel layout and the same arithmetic except no
// DSi-BIOS check and no "volume > -0x8000" gate on a volume LFO. Ranges: every byte the native version replaces
// (functions, literal pools, BIOS SVC stubs; MKDS: ExChannelMain + helpers [0x037FBBD8,0x037FD128), the Thumb SVC
// stubs, the 64/32-bit divides of the sweep).
struct Range3 { u32 a, b; };
struct ExVar
{
    const char* name;
    u32 entry, instr;
    Range3 code[3];
    u64 sig;
    u32 chans, dbSq, sin;
    bool twl, lfoVolGate;
    s32 cyc0, cycCh;        // ponytail: fixed estimate per call: base + per active channel (check-mode guest averages)
};
constexpr ExVar kEx[] = {
    {"PW/PB/W2", 0x03800A5C, 0xE92D4FF8 /* push {r3-r11, lr} */,
     {{0x0380021C, 0x03800F48}, {0x03801418, 0x038014D4}, {0x037FB574, 0x037FB598}}, 0xd732d69f87473043ull,
     0x03809FBC, 0x03808E48, 0x03808E24, true, true, 200, 480},
    {"MKDS", 0x037FCE24, 0xE92D4FF0 /* push {r4-r11, lr} */,
     {{0x037FBBD8, 0x037FD128}, {0x03801210, 0x0380121C}, {0x0380697C, 0x03806D38}}, 0x74d99b93607adcfdull,
     0x03807ECC, 0x03806F40, 0x03806F1C, false, false, 394, 507},
};

// FreeBIOS ARM7 tables used by SVC 0x1B GetPitchTable (u16 x 768) and 0x1C GetVolumeTable
// (u8 x 724). A loaded BIOS must contain the same bytes (the real tables) or the HLE stays off.
constexpr u32 kFreePitch = 0x169C, kFreeVol = 0x1CB0;

struct HwVar;
struct State
{
    int status = 0;                 // 0 unprobed, 1 active, -1 off
    bool on = true;
    int copyOn = -1;                // MI_CpuCopy32 hook, latched separately (no signature probe)
    const ExVar* v = nullptr;       // the matching build
    std::vector<u8> code;           // snapshot of its ranges (per-call memcmp)
    u16 pitch[768] = {};
    u8 vol[724] = {};
    u64 copies = 0, calls = 0, native = 0, fallbacks = 0, checks = 0, checkDiffs = 0;
    int hw = 0;                     // SND hardware commit: 0 unprobed, 1 on, -1 off
    std::vector<u8> hwCode;
    const HwVar* hv = nullptr;      // the matching build
    u64 hwN[6] = {};                // calls, native, fallback, checks, check diffs, charged cycles (diag)
};
std::unordered_map<const melonDS::NDS*, State> g_State;
#ifdef LITEV_HLE_DIAG
bool g_Check = getenv("LITEV_A7HLE_CHECK") && atoi(getenv("LITEV_A7HLE_CHECK"));
bool g_CommitCheck = getenv("LITEV_A7HLE_COMMITCHECK") && atoi(getenv("LITEV_A7HLE_COMMITCHECK"));
// LITEV_A7HLE_DEFER=1 (with LITEV_A7HLE_CHECK=1, JIT mode): compare at the next A7HLE hook entry instead of the guest's
// return (the JIT has no per-instruction check): SND commit -> SeqMain -> ExChannelMain are called back to back, so
// the commit and the sequencer are checked in scenes the interpreter never reaches (MKDS race). Not compared:
// registers, stack frames, SOUNDxCNT bit 31.
bool g_Defer = getenv("LITEV_A7HLE_DEFER") && atoi(getenv("LITEV_A7HLE_DEFER"));
// LITEV_A7HLE_CYCADJ=n: n more cycles per native call (timing-sensitivity tests of the MKDS 8P session)
s32 g_CycAdj = getenv("LITEV_A7HLE_CYCADJ") ? atoi(getenv("LITEV_A7HLE_CYCADJ")) : 0;
// LITEV_A7HLE_FROM=<frame>: the A7 hooks run the guest code before that frame (an A/B in a scene reached by a
// timing-sensitive input script, e.g. the MKDS 8-console race)
u32 g_From = getenv("LITEV_A7HLE_FROM") ? (u32)atoi(getenv("LITEV_A7HLE_FROM")) : 0;
#else
constexpr bool g_Check = false, g_CommitCheck = false;   // compare mode compiled out
#endif
bool g_Stats = getenv("LITEV_A7HLE_STATS") && atoi(getenv("LITEV_A7HLE_STATS"));


inline u8* W7(melonDS::NDS& nds, u32 a) { return &nds.ARM7WRAM[a & (ARM7WRAMSize - 1)]; }
// host pointer of an ARM7 WRAM / shared WRAM (0x03000000-0x037FFFFF as the ARM7 sees it) byte
inline const u8* P7(melonDS::NDS& nds, u32 a)
{
    if (a >= 0x03800000 || !nds.SWRAM_ARM7.Mem) return W7(nds, a);
    return &nds.SWRAM_ARM7.Mem[a & nds.SWRAM_ARM7.Mask];
}
inline u16 R16(const u8* p) { u16 v; memcpy(&v, p, 2); return v; }
inline u32 R32(const u8* p) { u32 v; memcpy(&v, p, 4); return v; }
inline void W16(u8* p, u16 v) { memcpy(p, &v, 2); }
inline void W32(u8* p, u32 v) { memcpy(p, &v, 4); }
// a changed word written as the bus would (JIT invalidation check, store) without the bus dispatch;
// p = host pointer of the guest word a in that region
template <int region>
inline void Store32(melonDS::NDS& nds, u32 a, u8* p, u32 v)
{
    nds.JIT.CheckAndInvalidate<1, region>(a);
    W32(p, v);
}

// debug.litev.a7hle: 0 off, 1 (or unset) all but 16, else a mask: 1 ExChannelMain, 2 SeqMain, 4 MI_CpuCopy32, 8 SND hardware commit,
// 16 SeqMain / SND commit on Mario Kart DS's driver too (default off: with them the Mac 8-console LockstepMP race (fixed harness,
// liteDS-main 5b2f7c6a) ends in "Communication error" on 5 of 7 remote consoles; the hooks' results are compare-mode exact)
bool ReadOn(u32 bit)
{
#if defined(__ANDROID__)
    char b[16] = {0}; int n = __system_property_get("debug.litev.a7hle", b);
    const char* e = n > 0 ? b : nullptr;
#else
    const char* e = getenv("debug.litev.a7hle");
#endif
    const u32 m = e ? (u32)strtoul(e, nullptr, 0) : 1;
    return (m == 1 ? 15u : m) & bit;
}

void Probe(melonDS::NDS& nds, State& s)
{
    s.status = -1;
    s.on = ReadOn(1);
    if (!s.on) return;
    std::vector<u8> code;
    for (const ExVar& v : kEx)
    {
        code.clear();
        for (const Range3& r : v.code)
            for (u32 a = r.a; a < r.b; a++) code.push_back(*P7(nds, a));
        u64 h = 0xcbf29ce484222325ull;
        for (u8 b : code) h = (h ^ b) * 0x100000001b3ull;
        if (g_Stats) fprintf(stderr, "A7HLE: ExChannelMain %s signature %016llx\n", v.name, (unsigned long long)h);
        if (h == v.sig) { s.v = &v; break; }
    }
    if (!s.v) return;

    // BIOS tables: take FreeBIOS's, require the loaded ARM7 BIOS to hold the same bytes.
    auto fb = FreeBIOSGetNtrArm7();
    const auto& bios = nds.GetARM7BIOS();
    auto has = [&](const u8* p, size_t n) { return std::search(bios.begin(), bios.end(), p, p + n) != bios.end(); };
    if (!has(&fb[kFreePitch], 768 * 2) || !has(&fb[kFreeVol], 724)) return;
    memcpy(s.pitch, &fb[kFreePitch], sizeof(s.pitch));
    memcpy(s.vol, &fb[kFreeVol], sizeof(s.vol));
    s.code = std::move(code);
    s.status = 1;
}

State& Get(melonDS::NDS& nds)
{
    static const melonDS::NDS* last = nullptr;     // per-call map lookup avoided (one console)
    static State* lastS = nullptr;
    if (&nds == last) return *lastS;
    State& s = g_State[&nds];                      // map nodes are stable
    last = &nds; lastS = &s;
    return s;
}

// ---- native SND_ExChannelMain -----------------------------------------------------------
// Offsets in SNDExChannel (0x54 bytes), from the PW ARM7 code:
//  +1 type  +2 envStatus  +3 flags (b0 active, b1 start, b2 autoSweep, b3-7 syncFlag)
//  +4 panRange  +5 rootKey  +6 userDecay2(s16)  +8 midiKey  +9 velocity  +A initPan(s8)
//  +B userPan(s8)  +C userDecay(s16)  +E userPitch(s16)  +10 envAttenuation(s32)
//  +14 sweepCounter(s32)  +18 sweepLength(s32)  +1C envAttack  +1D envSustain  +1E envDecay(u16)
//  +20 envRelease(u16)  +22 priority  +23 pan  +24 volume(u16)  +26 timer(u16)
//  +28 lfo {target, speed, depth, range, delay(u16), delayCounter(u16), counter(u16)}
//  +32 sweepPitch(s16)  +3C waveTimer(u16)  +48 callback  +4C callbackData

u16 CalcTimer(const State& s, s32 timer, s32 pitch)
{
    s32 octave = 0;
    s32 p = -pitch;
    while (p < 0) { octave--; p += 768; }
    while (p >= 768) { octave++; p -= 768; }
    u64 result = (u64)(s.pitch[p] + 0x10000u) * (u64)(s64)timer;
    s32 shift = octave - 16;
    if (shift <= 0)
    {
        shift = -shift;
        result = shift >= 64 ? 0 : result >> shift;
    }
    else if (shift < 32)
    {
        if (result & (~0ull << (32 - shift))) return 0xFFFF;
        result <<= shift;
    }
    else
        return 0xFFFF;
    if (result < 0x10) result = 0x10;
    else if (result > 0xFFFF) result = 0xFFFF;
    return (u16)result;
}

u16 CalcChannelVolume(const State& s, s32 v)
{
    if (v < -723) v = -723;
    else if (v > 0) v = 0;
    u32 r = s.vol[v + 723];
    u32 div = v < -240 ? 3 : v < -120 ? 2 : v < -60 ? 1 : 0;
    return (u16)(r | (div << 8));
}

s32 SinIdx(melonDS::NDS& nds, const ExVar& v, s32 x)
{
    auto t = [&](s32 i) { return (s32)(s8)*W7(nds, v.sin + i); };
    if (x < 0x20) return t(x);
    if (x < 0x40) return t(0x40 - x);
    if (x < 0x60) return (s32)(s8)(-t(x - 0x40));
    return (s32)(s8)(-t(0x20 - (x - 0x60)));
}

// Runs the update into chans (a copy of the 16 structs). Returns false if the guest code has
// to run instead (a channel callback would be called, or an unsupported path).
bool ExChannelMain(melonDS::NDS& nds, const State& s, u8* chans, bool step)
{
    const ExVar& v = *s.v;
    if (v.twl && (*W7(nds, kTwlFlag) & 0x04) && !(*W7(nds, kTwlFlag) & 0x20)) return false;   // DSi-BIOS workaround path
    for (int i = 0; i < 16; i++)
    {
        u8* c = chans + i * kChanSize;
        u8 flags = c[3];
        if (!(flags & 1)) continue;

        if (flags & 2)
            c[3] = (u8)((flags | 0x08) & ~2);          // syncFlag |= START, start = 0
        else if (!(nds.ARM7Read8(0x04000403 + i * 16) & 0x80))
        {
            if (R32(c + 0x48)) return false;           // callback
            c[0x22] = 0;
            W16(c + 0x24, 0);
            c[3] &= ~1;
            continue;
        }

        s32 vol = (s16)R16(W7(nds, v.dbSq + c[9] * 2));
        s32 pitch = ((s32)c[8] - (s32)c[5]) << 6;

        // envelope
        s32 att = (s32)R32(c + 0x10);
        if (step)
        {
            switch (c[2])
            {
            case 0:
                att = (s32)(0u - (u32)((s32)((0u - (u32)att) * (u32)c[0x1C]) >> 8));
                W32(c + 0x10, (u32)att);
                if (att == 0) c[2] = 1;
                break;
            case 1:
            {
                s32 sus = (s32)(s16)R16(W7(nds, v.dbSq + c[0x1D] * 2)) << 7;
                att -= R16(c + 0x1E);
                W32(c + 0x10, (u32)att);
                if (att <= sus) { W32(c + 0x10, (u32)sus); att = sus; c[2] = 2; }
                break;
            }
            case 3:
                att -= R16(c + 0x20);
                W32(c + 0x10, (u32)att);
                break;
            default: break;
            }
        }
        vol += att >> 7;

        // sweep
        s32 sweep = 0;
        s16 sweepPitch = (s16)R16(c + 0x32);
        if (sweepPitch)
        {
            s32 cnt = (s32)R32(c + 0x14), len = (s32)R32(c + 0x18);
            if (cnt < len)
            {
                if (len == 0) return false;
                sweep = (s32)(((s64)sweepPitch * (s64)(s32)((u32)len - (u32)cnt)) / (s64)len);
                if (step && (c[3] & 4)) W32(c + 0x14, (u32)(cnt + 1));
            }
        }

        vol += (s16)R16(c + 0x0C) + (s16)R16(c + 0x06);
        pitch += sweep + (s16)R16(c + 0x0E);

        // LFO (value before the step update)
        u8* lfo = c + 0x28;
        s32 lfoRaw = 0;
        if (lfo[2] && R16(lfo + 6) >= R16(lfo + 4))
            lfoRaw = SinIdx(nds, v, R16(lfo + 8) >> 8) * (s32)lfo[2] * (s32)lfo[3];
        s32 lfoVal = lfoRaw;
        if (lfoRaw)
        {
            s64 v = (s64)lfoRaw;
            if (lfo[0] == 0 || lfo[0] == 2) v = (s64)((u64)v << 6);
            else if (lfo[0] == 1) v = (s64)((u64)v * 60);
            lfoVal = (s32)(u32)((u64)v >> 14);
        }
        if (step)
        {
            u16 dc = R16(lfo + 6), delay = R16(lfo + 4);
            if (dc < delay) W16(lfo + 6, dc + 1);
            else
            {
                u32 sp = (u32)lfo[1] << 6;
                u32 ctr = R16(lfo + 8);
                u32 hi = (ctr + sp) >> 8;
                while (hi >= 0x80) hi -= 0x80;
                ctr = (ctr + (sp & 0xFFFF)) & 0xFFFF;
                ctr &= 0xFF;
                ctr |= (hi & 0xFF) << 8;
                W16(lfo + 8, (u16)ctr);
            }
        }
        s32 pan = 0;
        switch (lfo[0])
        {
        case 0: pitch += lfoVal; break;
        case 1: if (!v.lfoVolGate || vol > -0x8000) vol += lfoVal; break;
        case 2: pan += lfoVal; break;
        default: break;
        }

        pan += (s8)c[0x0A];
        if (c[4] != 0x7F) pan = (pan * (s32)c[4] + 0x40) >> 7;
        pan += (s8)c[0x0B];

        if (c[2] == 3 && vol <= -723)
        {
            c[3] = (u8)((c[3] & 7) | 0x10);            // syncFlag = STOP
            if (R32(c + 0x48)) return false;           // callback
            c[0x22] = 0;
            W16(c + 0x24, 0);
            c[3] &= ~1;
            continue;
        }

        u16 v16 = CalcChannelVolume(s, vol);
        u16 timer = CalcTimer(s, R16(c + 0x3C), pitch);
        if (c[1] == 1) timer &= 0xFFFC;
        pan += 0x40;
        if (pan < 0) pan = 0;
        else if (pan > 0x7F) pan = 0x7F;

        if (v16 != R16(c + 0x24)) { W16(c + 0x24, v16); c[3] |= 0x40; }
        if (timer != R16(c + 0x26)) { W16(c + 0x26, timer); c[3] |= 0x20; }
        if ((u8)pan != c[0x23]) { c[0x23] = (u8)pan; c[3] |= 0x80; }
    }
    return true;
}

bool CodeIntact(melonDS::NDS& nds, const State& s)
{
    const u8* c = s.code.data();
    for (const Range3& r : s.v->code)
    {
        for (u32 a = r.a; a < r.b; a++, c++)
            if (*P7(nds, a) != *c) return false;
    }
    return true;
}


// ---- native SND_SeqMain (sequencer: players, tracks, SSEQ bytecode) -----------------------
// PW layout (from its ARM7 code):
//  work  0x03809F9C: printEnabled, seq cache {begin, end, buf[16]} (begin +4, end +8, buf +C),
//        0x03809FB8 SNDi_SharedWork*, channels 0x03809FBC (16 x 0x54), players 0x0380A4FC
//        (16 x 0x24), tracks 0x0380A73C (32 x 0x40). Random state 0x03809360.
//  player: +0 flags (b0 active, b1 prepared, b2 paused) +1 id +4 prio +5 volume +6 extFader(s16)
//        +8 tracks[16] +18 tempo +1A tempoRatio +1C tempoCounter +20 bank
//  track: +0 flags (b0 active, b1 noteWait, b2 muted, b3 tie, b4 noteFinishWait, b5 portamento,
//        b6 cmp, b7 channelMask) +1 panRange +2 program +4 volume +5 expression +6 pitchBend(s8)
//        +7 bendRange +8 pan(s8) +9 extPan(s8) +A extFader(s16) +C extPitch(s16) +E..+11 ADSR
//        +12 priority +13 transpose(s8) +14 portaKey +15 portaTime +16 sweepPitch(s16)
//        +18 mod {target, speed, depth, range, delay(u16)} +1E channelMask +20 wait(s32)
//        +24 base +28 cur +2C callStack[3] +38 loopCount[3] +3B depth +3C channel list head
//  channel list link: ExChannel +50.
// Runs on a copy of the work area + shared work + random state ("overlay"); reads elsewhere
// (sequence data, tables) go to memory. Anything unhandled (a note that needs a new channel:
// bank lookup + channel allocation) or a write outside the overlay abandons the copy and the
// guest code runs instead.
// Per driver build (SeqVar): the work area has the same layout in both (printEnabled, seq cache, shared work pointer
// +0x1C, channels +0x20, players +0x560, tracks +0x7A0; 0xFA0 bytes). Mario Kart DS (2005 NitroSDK, driver in shared
// WRAM): the same sequencer code, except TrackUpdateChannel truncates volume / fader to s16 instead of clamping them
// to -0x8000, and there is no 0xD7 (mute) command. Ranges: every byte the native version replaces (MKDS: CalcRandom ..
// SeqMain [0x037FBF70,0x037FEF54), the 32-bit divide, the decibel-square and attack tables).
constexpr u32 kWorkSize = 0xFA0, kSharedSize = 0x280;
constexpr u32 kWShared = 0x1C, kWPlayers = 0x560, kWTracks = 0x7A0;   // offsets in the work area
struct SeqVar
{
    const char* name;
    u32 entry, instr;
    Range3 code[4];
    u64 sig;
    u32 work, rnd, dbSq, attack;    // sound work area, random state, SNDi_DecibelSquareTable, attack table
    bool clamp, mute;               // TrackUpdateChannel clamps volume / fader to -0x8000; command 0xD7 exists
    s32 cyc0, cycTrack, cycStep, cycCmd, cycChan, cycPlayer;   // ponytail: fixed estimate (MKDS: check-mode fit): base + per
                                    // updated track, stepped track, command, channel update, active player
};
constexpr SeqVar kSeqV[] = {
    {"PW/PB/W2", 0x038015C4, 0xE92D47F0 /* push {r4-r10, lr} */,
     {{0x03801190, 0x03802E74}, {0x03800574, 0x038005A0}, {0, 0}, {0, 0}}, 0x5b3b713e31e0c679ull,
     0x03809F9C, 0x03809360, 0x03808E48, 0x03808F5C, true, true, 150, 400, 0, 0, 0, 0},
    {"MKDS", 0x037FEEB4, 0xE92D47F0 /* push {r4-r10, lr} */,
     {{0x037FBF70, 0x037FEF54}, {0x0380697C, 0x03806D38}, {0x03806F40, 0x03807040}, {0x03807054, 0x03807067}}, 0x7e34b2b9c5282c13ull,
     0x03807EAC, 0x038075A0, 0x03806F40, 0x03807054, false, false, 249, 70, 154, 313, 42, 752},   // 87k calls, p95 15%
};

struct SeqState
{
    int status = 0;
    const SeqVar* v = nullptr;
    std::vector<u8> code;
    u64 calls = 0, native = 0, fallbacks = 0, checks = 0, checkDiffs = 0, cyc = 0;
};
std::unordered_map<const melonDS::NDS*, SeqState> g_Seq;

SeqState& GetSeq(melonDS::NDS& nds)
{
    static const melonDS::NDS* last = nullptr;
    static SeqState* lastS = nullptr;
    if (&nds == last) return *lastS;
    SeqState& s = g_Seq[&nds];
    last = &nds; lastS = &s;
    if (s.status != 0) return s;
    s.status = -1;
    if (!ReadOn(2)) return s;
    std::vector<u8> code;
    for (const SeqVar& v : kSeqV)
    {
        code.clear();
        for (const Range3& r : v.code)
            for (u32 a = r.a; a < r.b; a++) code.push_back(*P7(nds, a));
        u64 h = 0xcbf29ce484222325ull;
        for (u8 b : code) h = (h ^ b) * 0x100000001b3ull;
        if (g_Stats) fprintf(stderr, "A7HLE: SeqMain %s signature %016llx\n", v.name, (unsigned long long)h);
        if (h == v.sig) { s.v = &v; break; }
    }
    if (!s.v || (s.v != &kSeqV[0] && !ReadOn(16))) { s.v = nullptr; return s; }
    s.code = std::move(code);
    s.status = 1;
    return s;
}

bool SeqCodeIntact(melonDS::NDS& nds, const SeqState& s)
{
    const u8* c = s.code.data();
    for (const Range3& r : s.v->code)
        for (u32 a = r.a; a < r.b; a++, c++)
            if (*P7(nds, a) != *c) return false;
    return true;
}

struct Seq
{
    melonDS::NDS& nds;
    const SeqVar& V;                // driver build
    const u32 wA;                   // its work area
    u8 work[kWorkSize];
    u8 shared[kSharedSize];
    u8 rnd[4];
    u32 sw = 0;                     // shared work address (0 = none)
    bool bail = false;
    u32 steps = 0, cmds = 0, chans = 0, players = 0; // tracks stepped, commands, channel updates, active players (cycle estimate)

    explicit Seq(melonDS::NDS& n, const SeqVar& sv) : nds(n), V(sv), wA(sv.work)
    {
        memcpy(work, W7(nds, wA), sizeof(work));
        memcpy(rnd, W7(nds, V.rnd), 4);
        sw = R32(work + kWShared);
        if (sw)
        {
            // shared work in main RAM only, word aligned, not wrapping the RAM mask
            if ((sw >> 24) != 0x02 || (sw & 3) || ((sw & nds.MainRAMMask) + kSharedSize) > nds.MainRAMMask + 1)
            { bail = true; sw = 0; return; }
            memcpy(shared, &nds.MainRAM[sw & nds.MainRAMMask], kSharedSize);
        }
    }

    u64 dirtyW[(kWorkSize + 255) / 256] = {};      // one bit per word
    u64 dirtyS[(kSharedSize + 255) / 256] = {};
    static void Mark(u64* bits, u32 o, u32 n)
    {
        for (u32 w = o >> 2; w <= (o + n - 1) >> 2; w++) bits[w >> 6] |= 1ull << (w & 63);
    }
    // writable overlay pointer for [a, a+n): marks those words for Commit
    u8* AtW(u32 a, u32 n)
    {
        if (a - wA <= kWorkSize - n) { Mark(dirtyW, a - wA, n); return work + (a - wA); }
        if (sw && a - sw <= kSharedSize - n) { Mark(dirtyS, a - sw, n); return shared + (a - sw); }
        if (a - V.rnd <= 4 - n) return rnd + (a - V.rnd);
        return nullptr;
    }
    // overlay pointer for [a, a+n)
    u8* At(u32 a, u32 n)
    {
        if (a - wA <= kWorkSize - n) return work + (a - wA);
        if (sw && a - sw <= kSharedSize - n) return shared + (a - sw);
        if (a - V.rnd <= 4 - n) return rnd + (a - V.rnd);
        return nullptr;
    }
    // reads outside the overlay: ARM7 WRAM / main RAM directly, anything else through the bus
    const u8* Mem(u32 a, u32 n)
    {
        if ((a >> 23) == (0x03800000 >> 23)) return W7(nds, a);
        if ((a >> 24) == 0x02 && ((a & nds.MainRAMMask) + n) <= nds.MainRAMMask + 1) return &nds.MainRAM[a & nds.MainRAMMask];
        return nullptr;
    }
    u8 r8(u32 a) { if (const u8* p = At(a, 1)) return *p; if (const u8* m = Mem(a, 1)) return *m; return nds.ARM7Read8(a); }
    u16 r16(u32 a) { if (const u8* p = At(a, 2)) return R16(p); if (const u8* m = Mem(a, 2)) return R16(m); return nds.ARM7Read16(a); }
    u32 r32(u32 a) { if (const u8* p = At(a, 4)) return R32(p); if (const u8* m = Mem(a, 4)) return R32(m); return nds.ARM7Read32(a); }
    void w8(u32 a, u8 v) { if (u8* p = AtW(a, 1)) *p = v; else bail = true; }
    void w16(u32 a, u16 v) { if (a & 1) bail = true; else if (u8* p = AtW(a, 2)) W16(p, v); else bail = true; }
    void w32(u32 a, u32 v) { if (a & 3) bail = true; else if (u8* p = AtW(a, 4)) W32(p, v); else bail = true; }

    // write back changed words (JIT code invalidation, same as guest stores)
    template <int region>
    void CommitRegion(u32 addr, u8* mem, const u8* buf, const u64* bits, u32 nbits)
    {
        for (u32 i = 0; i < nbits; i++)
            for (u64 b = bits[i]; b; b &= b - 1)
            {
                u32 w = (i * 64 + __builtin_ctzll(b)) * 4;
                if (R32(mem + w) != R32(buf + w)) Store32<region>(nds, addr + w, mem + w, R32(buf + w));
            }
    }
    void Commit()
    {
        using M = melonDS::ARMJIT_Memory;
        CommitRegion<M::memregion_WRAM7>(wA, W7(nds, wA), work, dirtyW, sizeof(dirtyW) / 8);
        if (R32(W7(nds, V.rnd)) != R32(rnd)) Store32<M::memregion_WRAM7>(nds, V.rnd, W7(nds, V.rnd), R32(rnd));
        if (sw) CommitRegion<M::memregion_MainRAM>(sw, &nds.MainRAM[sw & nds.MainRAMMask], shared, dirtyS, sizeof(dirtyS) / 8);
    }

    // ---- helpers (addresses of the PW functions they mirror) ----
    u16 CalcRandom()                                    // 03800574
    {
        u32 st = r32(V.rnd) * 0x19660Du + 0x3C6EF35Fu;
        w32(V.rnd, st);
        return (u16)(st >> 16);
    }
    void CacheFetch(u32 addr)                           // 03801DCC
    {
        u32 a = addr & ~3u;
        w32(wA + 4, a);
        w32(wA + 8, a + 16);
        for (u32 i = 0; i < 4; i++) w32(wA + 0xC + i * 4, nds.ARM7Read32(a + i * 4));
    }
    u8 ReadU8(u32 t)                                    // 038018DC
    {
        u32 cur = r32(t + 0x28);
        if (cur < r32(wA + 4) || cur >= r32(wA + 8)) CacheFetch(cur);
        u8 b = r8(wA + 0xC + (cur - r32(wA + 4)));
        w32(t + 0x28, cur + 1);
        return b;
    }
    u32 ReadU16(u32 t) { u32 lo = ReadU8(t); u32 hi = ReadU8(t); return lo | (hi << 8); }
    u32 ReadU24(u32 t) { u32 a = ReadU8(t); u32 b = ReadU8(t); u32 c = ReadU8(t); return a | (b << 8) | (c << 16); }
    u32 VarPtr(u32 p, u32 var)                          // 03802D78
    {
        u32 s = r32(wA + kWShared);
        if (!s) return 0;
        if (var < 16) return s + 0x20 + r8(p + 1) * 0x24 + var * 2;
        return s + 0x260 + (var - 16) * 2;
    }
    s32 Parse(u32 t, u32 p, int type)                   // 03801E60
    {
        switch (type)
        {
        case 0: return ReadU8(t);
        case 1: return (s32)ReadU16(t);
        case 2: { s32 v = 0; u32 b; do { b = ReadU8(t); v = (s32)(((u32)v << 7) | (b & 0x7F)); } while (b & 0x80); return v; }
        case 3:
        {
            s32 lo = (s32)(ReadU16(t) << 16);
            s32 hi = (s16)ReadU16(t);
            s32 ran = CalcRandom();
            s32 r1 = (s32)((u32)ran * (u32)(hi - (lo >> 16) + 1));
            return (r1 >> 16) + (lo >> 16);
        }
        case 4:
        {
            u32 a = VarPtr(p, ReadU8(t));
            if (!a) { bail = true; return 0; }          // guest returns a stale register here
            return (s16)r16(a);
        }
        }
        bail = true;
        return 0;
    }
    u32 GetTrack(u32 p, u32 i)                          // 03802090
    {
        if (i > 15) return 0;
        u8 idx = r8(p + 8 + i);
        return idx == 0xFF ? 0 : wA + kWTracks + idx * 0x40;
    }
    static u16 DecayCoeff(u32 v)                        // 038014D4
    {
        if (v == 0x7F) return 0xFFFF;
        if (v == 0x7E) return 0x3C00;
        if (v < 0x32) return (u16)(v * 2 + 1);
        return (u16)((s32)0x1E00 / (s32)(0x7E - v));
    }
    void SetAttack(u32 c, u32 v) { w8(c + 0x1C, v >= 0x6D ? r8(V.attack + (0x7F - v)) : (u8)(0xFF - v)); }
    void SetRelease(u32 c, u32 v) { w16(c + 0x20, DecayCoeff(v)); }

    void UpdateChannels(u32 t, u32 p, bool release)     // 038021AC TrackUpdateChannel
    {
        const u8* T = At(t, 0x40);
        const u8* P = At(p, 0x24);
        if (!T || !P) { bail = true; return; }
        const u8* db = W7(nds, V.dbSq);
        s32 vol = (s16)R16(db + T[4] * 2) + (s16)R16(db + T[5] * 2) + (s16)R16(db + P[5] * 2);
        s32 pitch = (s16)R16(T + 0xC) + (((s32)(s8)T[6] * (s32)(T[7] << 6)) >> 7);
        s32 pan = (s8)T[8];
        u8 panRange = T[1];
        if (panRange != 0x7F) pan = (pan * panRange + 0x40) >> 7;
        pan += (s8)T[9];
        s32 fader = (s16)R16(T + 0xA) + (s16)R16(P + 6);
        if (V.clamp)
        {
            if (vol < -0x8000) vol = -0x8000;
            if (fader < -0x8000) fader = -0x8000;
        }
        if (pan < -128) pan = -128; else if (pan > 127) pan = 127;
        for (u32 c = R32(T + 0x3C); c; )
        {
            u8* C = At(c, 0x54);
            if (!C || c < wA) { bail = true; return; }
            const u32 o = c - wA;
            chans++;
            Mark(dirtyW, o + 4, 4);                     // userDecay2 (+6)
            W16(C + 6, (u16)fader);
            if (C[2] != 3)
            {
                Mark(dirtyW, o + 8, 8);                 // userPan (+B), userDecay/userPitch (+C, +E)
                Mark(dirtyW, o + 0x28, 8);              // lfo param (+28..+2D); panRange (+4) is in word +4
                Mark(dirtyW, o, 1);                     // envStatus (+2)
                Mark(dirtyW, o + 0x20, 4);              // priority (+22)
                W16(C + 0xC, (u16)vol);
                W16(C + 0xE, (u16)pitch);
                C[0xB] = (u8)pan;
                C[4] = T[1];
                memcpy(C + 0x28, T + 0x18, 6);
                if (R32(C + 0x34) == 0 && release) { C[0x22] = 1; C[2] = 3; }
            }
            c = R32(C + 0x50);
        }
    }
    void ReleaseChannels(u32 t, u32 p, s32 rel)         // 03801FF4
    {
        UpdateChannels(t, p, false);
        for (u32 c = r32(t + 0x3C); c && !bail; c = r32(c + 0x50))
            if (r8(c + 3) & 1)
            {
                if (rel >= 0) SetRelease(c, rel & 0xFF);
                w8(c + 0x22, 1);
                w8(c + 2, 3);
            }
    }
    void FreeChannels(u32 t)                            // 0380205C
    {
        for (u32 c = r32(t + 0x3C); c && !bail; c = r32(c + 0x50)) { w32(c + 0x48, 0); w32(c + 0x4C, 0); }
        w32(t + 0x3C, 0);
    }
    void StopTrack(u32 p, u32 i)                        // 038020BC
    {
        u32 t = GetTrack(p, i);
        if (!t) return;
        ReleaseChannels(t, p, -1);
        FreeChannels(t);
        u32 ta = wA + kWTracks + r8(p + 8 + i) * 0x40;
        w8(ta, r8(ta) & ~1);
        w8(p + 8 + i, 0xFF);
    }
    void StopPlayer(u32 p)                              // 03802114
    {
        for (u32 i = 0; i < 16; i++) StopTrack(p, i);
        w8(p, r8(p) & ~1);
    }
    void Mute(u32 t, u32 p, u32 mode)                   // 03802DF8
    {
        switch (mode)
        {
        case 0: w8(t, r8(t) & ~4); break;
        case 1: w8(t, r8(t) | 4); break;
        case 2: w8(t, r8(t) | 4); ReleaseChannels(t, p, -1); break;
        case 3: w8(t, r8(t) | 4); ReleaseChannels(t, p, 127); FreeChannels(t); break;
        }
    }
    s32 StepTrack(u32 t, u32 p, bool playNotes)         // TrackStepTicks, inlined in 03802304
    {
        for (u32 c = r32(t + 0x3C); c; c = r32(c + 0x50))
        {
            s32 len = (s32)r32(c + 0x34);
            if (len > 0) w32(c + 0x34, len - 1);
            if (!(r8(c + 3) & 4))
            {
                s32 cnt = (s32)r32(c + 0x14);
                if (cnt < (s32)r32(c + 0x18)) w32(c + 0x14, cnt + 1);
            }
        }
        if (r8(t) & 0x10)
        {
            if (r32(t + 0x3C)) return 0;
            w8(t, r8(t) & ~0x10);
        }
        s32 wait = (s32)r32(t + 0x20);
        if (wait > 0)
        {
            wait--;
            w32(t + 0x20, wait);
            if (wait > 0) return 0;
        }
        CacheFetch(r32(t + 0x28));
        while (r32(t + 0x20) == 0 && !(r8(t) & 0x10))
        {
            if (bail) return 0;
            bool special = false, run = true;
            int vt = 0;
            u32 cmd = ReadU8(t);
            cmds++;
            if (cmd == 0xA2) { cmd = ReadU8(t); run = (r8(t) >> 6) & 1; }
            if (cmd == 0xA0) { cmd = ReadU8(t); vt = 3; special = true; }
            if (cmd == 0xA1) { cmd = ReadU8(t); vt = 4; special = true; }
            if (!(cmd & 0x80))
            {
                u32 vel = ReadU8(t);
                s32 len = Parse(t, p, special ? vt : 2);
                s32 key = (s32)cmd + (s8)r8(t + 0x13);
                if (!run) continue;
                if (key < 0) key = 0; else if (key > 127) key = 127;
                u8 f = r8(t);
                if (!(f & 4) && playNotes)
                {
                    u32 c = 0;
                    if (f & 8)
                    {
                        c = r32(t + 0x3C);
                        if (c) { w8(c + 8, (u8)key); w8(c + 9, (u8)vel); }
                    }
                    if (!c) { bail = true; return 0; }  // needs bank lookup + channel allocation
                    if (r8(t + 0xE) != 0xFF) SetAttack(c, r8(t + 0xE));
                    if (r8(t + 0xF) != 0xFF) w16(c + 0x1E, DecayCoeff(r8(t + 0xF)));
                    if (r8(t + 0x10) != 0xFF) w8(c + 0x1D, r8(t + 0x10));
                    if (r8(t + 0x11) != 0xFF) SetRelease(c, r8(t + 0x11));
                    w16(c + 0x32, r16(t + 0x16));
                    if (r8(t) & 0x20)
                        w16(c + 0x32, (u16)((s16)r16(c + 0x32) + ((s32)((u32)(r8(t + 0x14) - key) << 22) >> 16)));
                    u32 pt = r8(t + 0x15);
                    if (pt == 0)
                    {
                        w32(c + 0x18, (u32)(len > 0 ? len : -1));
                        w8(c + 3, r8(c + 3) & ~4);
                    }
                    else
                    {
                        s32 sp = (s16)r16(c + 0x32);
                        if (sp < 0) sp = -sp;
                        w32(c + 0x18, (u32)((s32)(pt * pt * (u32)sp) >> 11));
                    }
                    w32(c + 0x14, 0);
                }
                w8(t + 0x14, (u8)key);
                if (r8(t) & 2)
                {
                    w32(t + 0x20, (u32)len);
                    if (len == 0) w8(t, r8(t) | 0x10);
                }
                continue;
            }
            switch (cmd & 0xF0)
            {
            case 0x80:
            {
                s32 par = Parse(t, p, special ? vt : 2);
                if (!run) break;
                if (cmd == 0x80) w32(t + 0x20, (u32)par);
                else if (cmd == 0x81 && par < 0x10000) w16(t + 2, (u16)par);
                break;
            }
            case 0x90:
                if (cmd == 0x93)
                {
                    u32 idx = ReadU8(t);
                    u32 off = ReadU24(t);
                    if (!run) break;
                    u32 nt = GetTrack(p, idx);
                    if (nt && nt != t)
                    {
                        ReleaseChannels(nt, p, -1);
                        FreeChannels(nt);
                        u32 base = r32(t + 0x24);
                        w32(nt + 0x24, base);
                        w32(nt + 0x28, base + off);
                    }
                }
                else if (cmd == 0x94)
                {
                    u32 off = ReadU24(t);
                    if (run) w32(t + 0x28, r32(t + 0x24) + off);
                }
                else if (cmd == 0x95)
                {
                    u32 off = ReadU24(t);
                    if (!run) break;
                    u32 d = r8(t + 0x3B);
                    if (d < 3)
                    {
                        w32(t + 0x2C + d * 4, r32(t + 0x28));
                        w8(t + 0x3B, (u8)(d + 1));
                        w32(t + 0x28, r32(t + 0x24) + off);
                    }
                }
                break;
            case 0xC0:
            case 0xD0:
            {
                u8 par = (u8)Parse(t, p, special ? vt : 0);
                if (!run) break;
                switch (cmd)
                {
                case 0xC0: w8(t + 8, (u8)(par - 0x40)); break;
                case 0xC1: w8(t + 4, par); break;
                case 0xC2: w8(p + 5, par); break;
                case 0xC3: w8(t + 0x13, par); break;
                case 0xC4: w8(t + 6, par); break;
                case 0xC5: w8(t + 7, par); break;
                case 0xC6: w8(t + 0x12, par); break;
                case 0xC7: w8(t, (u8)((r8(t) & ~2) | ((par & 1) << 1))); break;
                case 0xC8:
                    w8(t, (u8)((r8(t) & ~8) | ((par & 1) << 3)));
                    ReleaseChannels(t, p, -1);
                    FreeChannels(t);
                    break;
                case 0xC9: w8(t + 0x14, (u8)(par + (s8)r8(t + 0x13))); w8(t, r8(t) | 0x20); break;
                case 0xCA: w8(t + 0x1A, par); break;
                case 0xCB: w8(t + 0x19, par); break;
                case 0xCC: w8(t + 0x18, par); break;
                case 0xCD: w8(t + 0x1B, par); break;
                case 0xCE: w8(t, (u8)((r8(t) & ~0x20) | ((par & 1) << 5))); break;
                case 0xCF: w8(t + 0x15, par); break;
                case 0xD0: w8(t + 0xE, par); break;
                case 0xD1: w8(t + 0xF, par); break;
                case 0xD2: w8(t + 0x10, par); break;
                case 0xD3: w8(t + 0x11, par); break;
                case 0xD4:
                {
                    u32 d = r8(t + 0x3B);
                    if (d < 3)
                    {
                        w32(t + 0x2C + d * 4, r32(t + 0x28));
                        w8(t + 0x38 + d, par);
                        w8(t + 0x3B, (u8)(d + 1));
                    }
                    break;
                }
                case 0xD5: w8(t + 5, par); break;
                case 0xD6: break;                       // print variable (no effect)
                case 0xD7: if (V.mute) Mute(t, p, par); break;
                default: break;
                }
                break;
            }
            case 0xE0:
            {
                u16 par = (u16)Parse(t, p, special ? vt : 1);
                if (!run) break;
                if (cmd == 0xE0) w16(t + 0x1C, par);
                else if (cmd == 0xE1) w16(p + 0x18, par);
                else if (cmd == 0xE3) w16(t + 0x16, par);
                break;
            }
            case 0xB0:
            {
                u32 var = ReadU8(t);
                s32 par = (s16)Parse(t, p, special ? vt : 1);
                u32 a = VarPtr(p, var);
                if (!run || !a) break;
                s32 v = (s16)r16(a);
                switch (cmd)
                {
                case 0xB0: w16(a, (u16)par); break;
                case 0xB1: w16(a, (u16)(v + par)); break;
                case 0xB2: w16(a, (u16)(v - par)); break;
                case 0xB3: w16(a, (u16)((u32)v * (u32)par)); break;
                case 0xB4: if (par != 0) w16(a, (u16)(v / par)); break;
                case 0xB5:
                    if (par >= 0) { u32 sh = (u32)par & 0xFF; w16(a, (u16)(sh >= 32 ? 0 : (u32)v << sh)); }
                    else { u32 sh = (u32)(-par) & 0xFF; w16(a, (u16)(sh >= 32 ? (v >> 31) : (v >> sh))); }
                    break;
                case 0xB6:
                {
                    bool neg = par < 0;
                    if (neg) par = (s16)(-par);
                    s32 r = CalcRandom();
                    s32 x = (s32)((u32)(par + 1) * (u32)r) >> 16;
                    if (neg) x = -x;
                    w16(a, (u16)x);
                    break;
                }
                case 0xB8: case 0xB9: case 0xBA: case 0xBB: case 0xBC: case 0xBD:
                {
                    bool c = cmd == 0xB8 ? v == par : cmd == 0xB9 ? v >= par : cmd == 0xBA ? v > par
                           : cmd == 0xBB ? v <= par : cmd == 0xBC ? v < par : v != par;
                    w8(t, (u8)((r8(t) & ~0x40) | (c ? 0x40 : 0)));
                    break;
                }
                default: break;
                }
                break;
            }
            case 0xF0:
            {
                if (!run) break;
                u32 d = r8(t + 0x3B);
                if (cmd == 0xFD)
                {
                    if (d) { d--; w8(t + 0x3B, (u8)d); w32(t + 0x28, r32(t + 0x2C + d * 4)); }
                }
                else if (cmd == 0xFC)
                {
                    if (!d) break;
                    u32 lc = r8(t + 0x37 + d);
                    if (lc)
                    {
                        lc = (lc - 1) & 0xFF;
                        if (!lc) { w8(t + 0x3B, (u8)(d - 1)); break; }
                    }
                    w8(t + 0x37 + d, (u8)lc);
                    w32(t + 0x28, r32(t + 0x28 + d * 4));
                }
                else if (cmd == 0xFF)
                    return -1;
                break;
            }
            default: break;
            }
        }
        return 0;
    }
    bool StepPlayer(u32 p, bool playNotes)              // 03802304 PlayerStepTicks
    {
        bool playing = false;
        for (u32 i = 0; i < 16 && !bail; i++)
        {
            u32 t = GetTrack(p, i);
            if (!t || !r32(t + 0x28)) continue;
            steps++;
            if (StepTrack(t, p, playNotes) == 0) playing = true;
            else StopTrack(p, i);
        }
        return !playing;
    }
    int Main(bool step)                                 // 038015C4 SND_SeqMain; returns active tracks
    {
        u32 status = 0;
        int tracks = 0;
        for (u32 i = 0; i < 16 && !bail; i++)
        {
            u32 p = wA + kWPlayers + i * 0x24;
            u8 f = r8(p);
            if (!(f & 1)) continue;
            players++;
            if (f & 2)
            {
                if (step && !(f & 4))
                {
                    u32 ticks = 0;
                    while (r16(p + 0x1C) >= 0xF0) { w16(p + 0x1C, (u16)(r16(p + 0x1C) - 0xF0)); ticks++; }
                    u32 j = 0;
                    for (; j < ticks && !bail; j++)
                        if (StepPlayer(p, true)) { StopPlayer(p); break; }
                    u32 s = r32(wA + kWShared);
                    if (s) { u32 a = s + r8(p + 1) * 0x24 + 0x40; w32(a, r32(a) + j); }
                    u32 inc = ((u32)r16(p + 0x18) * (u32)r16(p + 0x1A)) << 8;
                    w16(p + 0x1C, (u16)(r16(p + 0x1C) + (inc >> 16)));
                }
                for (u32 k = 0; k < 16 && !bail; k++)
                    if (u32 t = GetTrack(p, k)) { UpdateChannels(t, p, true); tracks++; }
            }
            if (r8(p) & 1) status |= 1u << i;
        }
        u32 s = r32(wA + kWShared);
        if (s) w32(s + 4, status);
        return tracks;
    }
};

#ifdef LITEV_HLE_DIAG
// check mode: one pending comparison (the hooked functions never nest)
struct Pending
{
    u32 ret = 0;
    const melonDS::ARM* cpu = nullptr;      // the console being compared (headless --mp-test runs several)
    int busy = -1;              // exchannel: SOUNDxCNT busy bits at entry (the native call samples them there)
    const char* name = "";
    u64* diffs = nullptr;
    std::vector<std::pair<u32, std::vector<u8>>> regions;   // address, expected bytes
    bool irq = false;           // an IRQ handler ran inside the guest call
    bool regs = false;          // also compare r0-r3, r12 and NZCV at the return (expected: R, cpsr)
    bool deferred = false;      // compared at the next hook entry (LITEV_A7HLE_DEFER): memory + SPU only
    u32 R[16] = {}, cpsr = 0;
    u64 t0 = 0; u32 fit[6] = {};            // guest cycles of the call (fit of the cycle estimates)
    std::vector<std::pair<int, u32>> spu;   // SPU state: (channel * 8 + what (0 Cnt & 0xFFFF, 1 pan byte, 2 Cnt bit 31, 3 timer, 4 Cnt byte 0), value)
};
thread_local Pending g_Pending;
u64 g_IrqDiffs = 0;
u64 g_BusyDiffs = 0;            // exchannel differences where a channel's busy bit changed inside the guest call             // differences with an IRQ in the guest window (the native call takes it after)

void ArmCheck(melonDS::ARM* cpu, const char* name, u64* diffs)
{
    g_Pending.ret = cpu->R[14] & ~1u;
    g_Pending.cpu = cpu;
    g_Pending.name = name;
    g_Pending.diffs = diffs;
    g_Pending.irq = false;
    g_Pending.regs = false;
    g_Pending.deferred = false;
    g_Pending.spu.clear();
    CheckPending = true;
}
#endif
struct StatsDump
{
    ~StatsDump()
    {
        if (!g_Stats) return;
        for (auto& [k, s] : g_State)
            fprintf(stderr, "A7HLE: status=%d copies=%llu calls=%llu native=%llu fallback=%llu checks=%llu check_diffs=%llu\n",
                    s.status, (unsigned long long)s.copies, (unsigned long long)s.calls, (unsigned long long)s.native,
                    (unsigned long long)s.fallbacks, (unsigned long long)s.checks, (unsigned long long)s.checkDiffs);
        for (auto& [k, s] : g_Seq)
            fprintf(stderr, "A7HLE seq: status=%d calls=%llu native=%llu fallback=%llu checks=%llu check_diffs=%llu cyc=%llu\n",
                    s.status, (unsigned long long)s.calls, (unsigned long long)s.native,
                    (unsigned long long)s.fallbacks, (unsigned long long)s.checks, (unsigned long long)s.checkDiffs, (unsigned long long)s.cyc);
        for (auto& [k, s] : g_State)
            fprintf(stderr, "A7HLE hwcommit: status=%d calls=%llu native=%llu fallback=%llu checks=%llu check_diffs=%llu cyc=%llu\n", s.hw,
                    (unsigned long long)s.hwN[0], (unsigned long long)s.hwN[1], (unsigned long long)s.hwN[2], (unsigned long long)s.hwN[3], (unsigned long long)s.hwN[4], (unsigned long long)s.hwN[5]);
#ifdef LITEV_HLE_DIAG
        if (g_BusyDiffs) fprintf(stderr, "A7HLE check: %llu exchannel differences with a channel stopping inside the guest call (busy bits sampled at entry; not counted)\n", (unsigned long long)g_BusyDiffs);
        if (g_IrqDiffs) fprintf(stderr, "A7HLE check: %llu differences with an IRQ inside the guest call (not counted above)\n", (unsigned long long)g_IrqDiffs);
#endif
    }
} g_StatsDump;
}


// MI_CpuCopy32(src, dst, size) (NitroSDK, ARM, position independent): word copy loop
//   add ip,r1,r2 / cmp r1,ip / ldmlt r0!,{r2} / stmlt r1!,{r2} / blt -> cmp / bx lr
// PW's sound driver copies its 4.4 KB driver info to main RAM with it every frame.
namespace
{
constexpr u32 kCopy32[6] = {0xE081C002, 0xE151000C, 0xB8B00004, 0xB8A10004, 0xBAFFFFFB, 0xE12FFF1E};

bool IsCopy32(melonDS::NDS& nds, u32 addr)
{
    for (u32 i = 1; i < 6; i++)
        if (nds.ARM7Read32(addr + i * 4) != kCopy32[i]) return false;
    return true;
}

// Same loads/stores in the same order through the normal bus, same final registers.
void Copy32(melonDS::ARM* cpu)
{
    melonDS::NDS& nds = cpu->NDS;
    u32 src = cpu->R[0], dst = cpu->R[1];
    const u32 end = dst + cpu->R[2];
    u32 n = 0;
    // ARM7 WRAM -> main RAM, both contiguous in host memory: host copy, the JIT invalidation
    // check of ARM7Write32 once per 16-byte granule (what the per-word checks amount to)
    const u32 len = (s32)dst < (s32)end ? ((end - dst + 3) & ~3u) : 0;
    const u32 s0 = src & ~3u, d0 = dst & ~3u;
    if (len && (s0 >> 23) == (0x03800000 >> 23) && (d0 >> 24) == 0x02
        && (s0 & (ARM7WRAMSize - 1)) + len <= ARM7WRAMSize && (d0 & nds.MainRAMMask) + len <= nds.MainRAMMask + 1)
    {
        const u8* sp = &nds.ARM7WRAM[s0 & (ARM7WRAMSize - 1)];
        u8* dp = &nds.MainRAM[d0 & nds.MainRAMMask];
        for (u32 o = 0; o < len; o += 4)
        {
            if (o == 0 || ((d0 + o) & 15) == 0)
                nds.JIT.CheckAndInvalidate<1, melonDS::ARMJIT_Memory::memregion_MainRAM>(d0 + o);
            memcpy(dp + o, sp + o, 4);
        }
        n = len / 4;
        cpu->R[2] = R32(sp + len - 4);
        src += len; dst += len;
    }
    while ((s32)dst < (s32)end)
    {
        u32 v = nds.ARM7Read32(src & ~3u);
        nds.ARM7Write32(dst & ~3u, v);
        cpu->R[2] = v;
        src += 4; dst += 4; n++;
    }
    cpu->R[0] = src; cpu->R[1] = dst; cpu->R[12] = end;
    // ponytail: fixed cycle estimate (guest WRAM->main RAM: ~15 cycles/word on PW)
    cpu->Cycles += 8 + 15 * n;
    cpu->JumpTo(cpu->R[14]);
}

bool CopyOn(melonDS::NDS& nds)
{
    State& s = g_State[&nds];
    if (s.copyOn < 0) s.copyOn = ReadOn(4);
    return s.copyOn != 0;
}
}

namespace
{
void GuestFallback(melonDS::ARM* cpu)
{
    // execute the real first instruction (a push); the guest code follows
    u32 icode = ((cpu->CurInstr >> 4) & 0xF) | ((cpu->CurInstr >> 16) & 0xFF0);
    ARMInterpreter::ARMInstrTable[icode](cpu);
}

__attribute__((noinline)) bool RunSeq(melonDS::ARM* cpu, bool jit)
{
    melonDS::NDS& nds = cpu->NDS;
    SeqState& s = GetSeq(nds);
    if (s.status != 1 || cpu->R[15] - 8 != s.v->entry) return false;
    s.calls++;
    Seq q(nds, *s.v);
    int tracks = 0;
    bool ok = !q.bail && (jit || SeqCodeIntact(nds, s));
#ifdef LITEV_HLE_DIAG
    ok = ok && nds.NumFrames >= g_From;
#endif
    if (ok) { tracks = q.Main(cpu->R[0] != 0); ok = !q.bail; }
#ifdef LITEV_HLE_DIAG
    if (ok && g_Check)
    {
        g_Pending.regions.clear();
        g_Pending.regions.push_back({q.wA, std::vector<u8>(q.work, q.work + sizeof(q.work))});
        g_Pending.regions.push_back({q.V.rnd, std::vector<u8>(q.rnd, q.rnd + 4)});
        if (q.sw) g_Pending.regions.push_back({q.sw, std::vector<u8>(q.shared, q.shared + kSharedSize)});
        ArmCheck(cpu, "seq", &s.checkDiffs);
        g_Pending.t0 = nds.ARM7Timestamp + cpu->Cycles; g_Pending.fit[0] = tracks; g_Pending.fit[1] = q.steps; g_Pending.fit[2] = ~0u; g_Pending.fit[3] = q.cmds; g_Pending.fit[4] = q.chans; g_Pending.fit[5] = q.players;
        s.checks++;
        ok = false;
    }
#endif
    if (!ok)
    {
        s.fallbacks += !g_Check;
        GuestFallback(cpu);
        return true;
    }
    s.native++;
    q.Commit();
#ifdef LITEV_HLE_DIAG
    if (g_CommitCheck)                                  // every overlay write reached memory
    {
        bool bad = memcmp(W7(nds, q.wA), q.work, sizeof(q.work)) != 0 || memcmp(W7(nds, q.V.rnd), q.rnd, 4) != 0
                || (q.sw && memcmp(&nds.MainRAM[q.sw & nds.MainRAMMask], q.shared, kSharedSize) != 0);
        if (bad && s.checkDiffs++ < 5) fprintf(stderr, "A7HLE COMMITCHECK seq: memory != overlay after commit\n");
    }
#endif
    // ponytail: fixed cycle estimate (PW guest: ~4.2k cycles/tick for ~10 tracks; MKDS: check-mode fit)
    const SeqVar& v = *s.v;
    const s32 cyc = v.cyc0 + v.cycTrack * tracks + v.cycStep * (s32)q.steps + v.cycCmd * (s32)q.cmds + v.cycChan * (s32)q.chans
                    + v.cycPlayer * (s32)q.players;
    cpu->Cycles += cyc;
#ifdef LITEV_HLE_DIAG
    s.cyc += cyc;
    cpu->Cycles += g_CycAdj;
#endif
    cpu->JumpTo(cpu->R[14]);
    return true;
}
}

// ---- SND hardware commit (PW/PB/W2 0x03800870, MKDS 0x037FD130) ------------------------------------------------
// After a sound tick the driver commits each channel's pending hardware updates (the SNDExChannel flag bits 3-7
// of byte 3: start, stop, timer, volume, pan) with one helper call per update (SOUNDxCNT / SOUNDxTMR stores), then
// sets the start bits and clears the flags: ~550 guest instructions a call (3.2 calls a frame, 12-15% of the ARM7's
// guest instructions on PW). Natively when no channel starts (the three start paths stay guest code): the same SOUND
// register stores in the same order, the volume / pan shadow bytes, the flag bytes, the stack frames below sp and
// the guest's final registers / flags. PW: only with the master-effect adjustment off ([fx] <= 0). MKDS (2005 driver,
// shared WRAM; its effect level is > 0 in the race): the effect (0x037FBA94: volume scaled by pan for channels other
// than 1 and 3 when the pan is < 0x18 or > 0x68) natively, its helpers' registers and frames.
// Category B: a fixed cycle estimate (check-mode fit).
namespace
{
struct HwVar
{
    const char* name;
    u32 entry, instr;
    Range3 code[2];                 // helpers, the function (literal pools included)
    u64 sig;
    u32 chans, fx, vol, pan, panOvr;    // channels, master effect level, volume / pan shadow bytes, pan override
    bool mk;                        // 2005 driver: effect natively, its helpers' registers / frames
    u32 retVol, retPan;             // return addresses the volume / pan helpers push
    s32 cyc0, cycCh, cycVol, cycOther, cycFx;   // ponytail: fixed estimate (check-mode fit): base + per channel with updates
                                    // + per volume / other store + per effect scaling (MKDS)
};
constexpr HwVar kHwV[] = {
    {"PW/PB/W2", 0x03800870, 0xE92D47F0 /* push {r4-r10, lr} */, {{0x038000EC, 0x0380021C}, {0x03800870, 0x03800A5C}},
     0x24e4d4770c4a81dcull, 0x03809FBC, 0x03809A5C, 0x03809A70, 0x03809A60, 0x0380935C, false, 0x038009D0, 0x038009F0,
     482, 42, 40, 13, 0},   // 6.9k calls, within 1.5%
    {"MKDS", 0x037FD130, 0xE92D47F0 /* push {r4-r10, lr} */, {{0x037FBA94, 0x037FBD38}, {0x037FD130, 0x037FD31C}},
     0x52806648b3d4f885ull, 0x03807ECC, 0x0380796C, 0x03807980, 0x03807970, 0x0380759C, true, 0x037FD290, 0x037FD2B0,
     482, 39, 66, 15, 18},  // 88k calls (8-console menus), p95 0.4%
};
}
int HwProbe(melonDS::NDS& nds)
{
    State& s = Get(nds);
    if (s.hw) return s.hw;
    s.hw = -1;
    if (!ReadOn(8)) return s.hw;
    std::vector<u8> code;
    for (const HwVar& v : kHwV)
    {
        code.clear();
        for (const Range3& r : v.code)
            for (u32 a = r.a; a < r.b; a++) code.push_back(*P7(nds, a));
        u64 h = 0xcbf29ce484222325ull;
        for (u8 b : code) h = (h ^ b) * 0x100000001b3ull;
        if (g_Stats) fprintf(stderr, "A7HLE: SND hw commit %s signature %016llx\n", v.name, (unsigned long long)h);
        if (h == v.sig) { s.hv = &v; break; }
    }
    if (!s.hv || (s.hv != &kHwV[0] && !ReadOn(16))) { s.hv = nullptr; return s.hw; }
    s.hwCode = std::move(code);
    return s.hw = 1;
}
bool HwIntact(melonDS::NDS& nds)
{
    const State& s = Get(nds);
    const u8* c = s.hwCode.data();
    for (const Range3& r : s.hv->code)
        for (u32 a = r.a; a < r.b; a++, c++)
            if (*P7(nds, a) != *c) return false;
    return true;
}

__attribute__((noinline)) bool RunHw(melonDS::ARM* cpu, bool jit)
{
    melonDS::NDS& nds = cpu->NDS;
    if (HwProbe(nds) != 1) return false;
    State& S = Get(nds);
    const HwVar& V = *S.hv;
    if (cpu->R[15] - 8 != V.entry) return false;
    u64* g_Hw = S.hwN;
    g_Hw[0]++;
    const u32 sp = cpu->R[13];
    u8* ch = W7(nds, V.chans);
    const s32 fx = (s32)R32(W7(nds, V.fx));
    bool ok = !CheckPending && (jit || HwIntact(nds)) && (V.mk || fx <= 0)
              && (sp >> 16) == 0x0380 && ((sp - 72) >> 16) == 0x0380 && !(sp & 3);
#ifdef LITEV_HLE_DIAG
    ok = ok && nds.NumFrames >= g_From;
#endif
    u32 any = 0;
    for (int i = 0; ok && i < 16; i++)
    {
        const u32 f = ch[i * kChanSize + 3] >> 3;
        if (f & 1) ok = false;          // a start: guest
        any |= f;
    }
    if (!ok)
    {
        g_Hw[2] += !g_Check;
        GuestFallback(cpu);
        return true;
    }
    // registers as the guest leaves them (r0-r3, r12, NZCV); stack words in write order
    u32 r1 = cpu->R[1], r2 = cpu->R[2], r3 = cpu->R[3], ip = cpu->R[12];
    struct Wr { u32 a, v; } st[8 + 6 * 16];
    u32 nst = 0;
    for (int i = 0; i < 7; i++) st[nst++] = {sp - 32 + i * 4, cpu->R[4 + i]};
    st[nst++] = {sp - 4, cpu->R[14]};
    struct Io { u32 a, v; u8 sz; } io[5 * 16];
    u32 nio = 0, nvol = 0, nother = 0, nch = 0, nfx = 0;
    u8 flags[16], vol[16], pan[16];
    bool vset[16] = {}, pset[16] = {};
    const s32 ovr = (s32)R32(W7(nds, V.panOvr));
    // MKDS 0x037FBA94: the effect's volume for pan p (r2 / r12 as it leaves them)
    auto fxVol = [&](u32 v, s32 p) -> u32
    {
        if (p < 0x18) { nfx++; r2 = 0x7FFF - (u32)fx; return (u32)((s32)(v * ((u32)(p + 0x28) * (u32)fx + (r2 << 6))) >> 21); }
        if (p <= 0x68) return v;
        nfx++;
        ip = (u32)fx; r2 = 0u - (u32)fx;
        return (u32)((s32)(v * ((u32)(p - 0x28) * r2 + (((u32)fx + 0x7FFF) << 6))) >> 21);
    };
    for (int i = 0; i < 16; i++)
    {
        u8* c = ch + i * kChanSize;
        const u32 f = c[3] >> 3, hw = 0x04000400 + i * 16;
        flags[i] = c[3];
        if (!f) continue;
        nch++;
        if (f & 2)
        {
            // stop: SOUNDxCNT &= ~0x80000000 (32-bit read and store)
            const u32 cnt = nds.ARM7Read32(hw);
            io[nio++] = {hw, cnt & ~0x80000000u, 4};
            if (V.mk) { r2 = cnt & ~0x80000000u; ip = hw; }
            else { r1 = cnt & ~0x80000000u; r2 = cnt; r3 = i << 4; }
            nother++;
        }
        if (f & 4)
        {
            r1 = 0x10000 - R16(c + 0x26);
            io[nio++] = {hw + 8, r1 & 0xFFFF, 2};
            nother++;
        }
        if (f & 8)
        {
            // volume: shadow byte, SOUNDxCNT low halfword (PW: push {r3, r4, r5, lr} at sp - 72; MKDS: push {r4, r5, lr}
            // at sp - 68, the effect for pan = SOUNDxCNT byte 2)
            const u32 v = R16(c + 0x24);
            vol[i] = v & 0xFF; vset[i] = true;
            if (V.mk)
            {
                u32 lo = v & 0xFF;
                r2 = v >> 8;
                if (fx > 0 && ((r2 = 1u << i) & 0xFFF5)) { r2 = nds.ARM7Read8(hw + 2); lo = fxVol(lo, (s32)r2); }
                io[nio++] = {hw, (lo | ((v >> 8) << 8)) & 0xFFFF, 2};
            }
            else
            {
                r1 = (v & 0xFF) | ((v >> 8) << 8); r2 = v >> 8; ip = V.vol;
                io[nio++] = {hw, r1 & 0xFFFF, 2};
                st[nst++] = {sp - 72, r3};
            }
            st[nst++] = {sp - 68, 2}; st[nst++] = {sp - 64, 1}; st[nst++] = {sp - 60, V.retVol};
            nvol++;
        }
        if (f & 0x10)
        {
            // pan: shadow byte, the override if >= 0, SOUNDxCNT byte 2 (push {r4, lr} at sp - 64); MKDS: the effect's
            // volume for the new pan into SOUNDxCNT byte 0
            pan[i] = c[0x23]; pset[i] = true;
            r1 = ovr >= 0 ? (u32)ovr : c[0x23];
            io[nio++] = {hw + 2, r1 & 0xFF, 1};
            if (V.mk)
            {
                r2 = (u32)fx;
                if (fx > 0 && (r2 = (1u << i) & 0xFFF5))
                {
                    r2 = V.vol;
                    io[nio++] = {hw, fxVol(vset[i] ? vol[i] : *W7(nds, V.vol + i), (s32)r1) & 0xFF, 1};
                }
            }
            else { r2 = (u32)fx; r3 = V.fx; }
            st[nst++] = {sp - 64, 2}; st[nst++] = {sp - 60, V.retPan};
            nother++;
        }
    }
    const u8 last = ch[15 * kChanSize + 3];
    const u32 r0 = (last >> 3) ? last & 7 : 0;
#ifdef LITEV_HLE_DIAG
    if (g_Check)
    {
        g_Pending.regions.clear();
        for (int i = 0; i < 16; i++)
        {
            if (flags[i] >> 3) g_Pending.regions.push_back({V.chans + i * kChanSize + 3, {(u8)(flags[i] & 7)}});
            if (vset[i]) g_Pending.regions.push_back({V.vol + i, {vol[i]}});
            if (pset[i]) g_Pending.regions.push_back({V.pan + i, {pan[i]}});
        }
        for (u32 k = 0; k < nst && !g_Defer; k++)
        {
            bool later = false;     // a later push to the same slot: compare the last one only
            for (u32 j = k + 1; j < nst; j++) later |= st[j].a == st[k].a;
            if (later) continue;
            std::vector<u8> b(4); memcpy(b.data(), &st[k].v, 4);
            g_Pending.regions.push_back({st[k].a, b});
        }
        ArmCheck(cpu, "hwcommit", &g_Hw[4]);
        g_Pending.regs = true;
        g_Pending.t0 = nds.ARM7Timestamp + cpu->Cycles; g_Pending.fit[0] = nch; g_Pending.fit[1] = nvol; g_Pending.fit[2] = nother; g_Pending.fit[3] = nfx;
        for (int i = 0; i < 16; i++) g_Pending.R[i] = cpu->R[i];
        g_Pending.R[0] = r0; g_Pending.R[1] = 0x54; g_Pending.R[2] = r2; g_Pending.R[3] = V.chans; g_Pending.R[12] = ip;
        g_Pending.cpsr = 0x60000000;    // cmp r4, #16 (16)
        for (u32 k = 0; k < nio; k++)
        {
            const int c = (io[k].a >> 4) & 15, o = io[k].a & 15;
            if (o == 0 && io[k].sz == 1)    // byte 0 after a halfword store to the same channel: one expectation
            {
                bool merged = false;
                for (auto& [w, v] : g_Pending.spu)
                    if (w == c * 8) { v = (v & 0xFF00) | io[k].v; merged = true; }
                if (!merged) g_Pending.spu.push_back({c * 8 + 4, io[k].v});
                continue;
            }
            g_Pending.spu.push_back({c * 8 + (o == 8 ? 3 : o == 2 ? 1 : io[k].sz == 4 ? 2 : 0), io[k].v});
        }
        g_Hw[3]++;
        GuestFallback(cpu);
        return true;
    }
#endif
    for (u32 k = 0; k < nio; k++)
    {
        if (io[k].sz == 4) nds.ARM7Write32(io[k].a, io[k].v);
        else if (io[k].sz == 2) nds.ARM7Write16(io[k].a, (u16)io[k].v);
        else nds.ARM7Write8(io[k].a, (u8)io[k].v);
    }
    auto st8 = [&](u32 a, u8 v) { u8* p = W7(nds, a); if (*p != v) { nds.JIT.CheckAndInvalidate<1, melonDS::ARMJIT_Memory::memregion_WRAM7>(a & ~3u); *p = v; } };
    for (int i = 0; i < 16; i++)
    {
        if (flags[i] >> 3) st8(V.chans + i * kChanSize + 3, flags[i] & 7);
        if (vset[i]) st8(V.vol + i, vol[i]);
        if (pset[i]) st8(V.pan + i, pan[i]);
    }
    for (u32 k = 0; k < nst; k++)
    {
        u8* p = W7(nds, st[k].a);
        if (R32(p) != st[k].v) Store32<melonDS::ARMJIT_Memory::memregion_WRAM7>(nds, st[k].a, p, st[k].v);
    }
    cpu->R[0] = r0; cpu->R[1] = 0x54; cpu->R[2] = r2; cpu->R[3] = V.chans; cpu->R[12] = ip;
    cpu->CPSR = (cpu->CPSR & 0x0FFFFFFF) | 0x60000000;
    const s32 cyc = V.cyc0 + V.cycCh * (s32)nch + V.cycVol * (s32)nvol + V.cycOther * (s32)nother + V.cycFx * (s32)nfx;
    cpu->Cycles += cyc;
#ifdef LITEV_HLE_DIAG
    g_Hw[5] += cyc;
    cpu->Cycles += g_CycAdj;
#endif
    g_Hw[1]++;
    (void)any;
    cpu->JumpTo(cpu->R[14]);
    return true;
}

int IsHook(melonDS::NDS& nds, u32 addr, u32 instr)
{
    if ((addr == kHwV[0].entry && instr == kHwV[0].instr) || (addr == kHwV[1].entry && instr == kHwV[1].instr))
        return HwProbe(nds) != 1 || addr != Get(nds).hv->entry ? 0 : HwIntact(nds) ? 1 : 2;
    if (instr == kCopy32[0]) return CopyOn(nds) && IsCopy32(nds, addr) ? 1 : 0;
    if ((addr == kSeqV[0].entry && instr == kSeqV[0].instr) || (addr == kSeqV[1].entry && instr == kSeqV[1].instr))
    {
        SeqState& s = GetSeq(nds);
        return s.status != 1 || addr != s.v->entry ? 0 : SeqCodeIntact(nds, s) ? 1 : 2;
    }
    if (!((addr == kEx[0].entry && instr == kEx[0].instr) || (addr == kEx[1].entry && instr == kEx[1].instr))) return 0;
    State& s = Get(nds);
    if (s.status == 0) Probe(nds, s);              // (at its entry: the driver is loaded)
    return !(s.status == 1 && s.on && addr == s.v->entry) ? 0 : CodeIntact(nds, s) ? 1 : 2;
}
int Deps(u32 addr, u32 instr, Range* out)
{
    if (instr == kCopy32[0]) { out[0] = {addr, addr + 24}; return 1; }
    for (const HwVar& v : kHwV)
        if (addr == v.entry) { out[0] = {v.code[0].a, v.code[0].b}; out[1] = {v.code[1].a, v.code[1].b}; return 2; }
    for (const SeqVar& v : kSeqV)
        if (addr == v.entry)
        {
            int n = 0;
            for (const Range3& r : v.code) if (r.b > r.a) out[n++] = {r.a, r.b};
            return n;
        }
    const ExVar& v = addr == kEx[1].entry ? kEx[1] : kEx[0];
    for (int i = 0; i < 3; i++) out[i] = {v.code[i].a, v.code[i].b};
    return 3;
}

bool Run(melonDS::ARM* cpu, bool jit)
{
    if (cpu->Num != 1 || (cpu->CPSR & 0x20)) return false;
    const u32 pc = cpu->R[15] - 8;
    melonDS::NDS& nds = cpu->NDS;
#ifdef LITEV_HLE_DIAG
    if (g_Defer && CheckPending && cpu == g_Pending.cpu) { g_Pending.ret = pc; g_Pending.deferred = true; CheckAt(cpu, pc); }
#endif
    if (cpu->CurInstr == kCopy32[0])
    {
        if (!CopyOn(nds) || (!jit && !IsCopy32(nds, pc))) return false;
        g_State[&nds].copies++;
        Copy32(cpu);
        return true;
    }
    if ((pc == kSeqV[0].entry && cpu->CurInstr == kSeqV[0].instr) || (pc == kSeqV[1].entry && cpu->CurInstr == kSeqV[1].instr))
        return RunSeq(cpu, jit);
    if ((pc == kHwV[0].entry && cpu->CurInstr == kHwV[0].instr) || (pc == kHwV[1].entry && cpu->CurInstr == kHwV[1].instr))
        return RunHw(cpu, jit);
    if (!((pc == kEx[0].entry && cpu->CurInstr == kEx[0].instr) || (pc == kEx[1].entry && cpu->CurInstr == kEx[1].instr))) return false;
    State& s = Get(nds);
    if (s.status == 0) Probe(nds, s);
    if (s.status != 1 || !s.on || pc != s.v->entry) return false;
    s.calls++;

    u8 chans[16 * kChanSize];
    const u32 chA = s.v->chans;
    memcpy(chans, W7(nds, chA), sizeof(chans));
    bool ok = (jit || CodeIntact(nds, s)) && ExChannelMain(nds, s, chans, cpu->R[0] != 0);
#ifdef LITEV_HLE_DIAG
    ok = ok && nds.NumFrames >= g_From;
#endif
#ifdef LITEV_HLE_DIAG
    if (ok && g_Check)
    {
        // run the guest too; compare at its return
        g_Pending.regions.assign(1, {chA, std::vector<u8>(chans, chans + sizeof(chans))});
        ArmCheck(cpu, "exchannel", &s.checkDiffs);
        g_Pending.t0 = nds.ARM7Timestamp + cpu->Cycles; g_Pending.fit[0] = 0;
        for (int i = 0; i < 16; i++) g_Pending.fit[0] += chans[i * kChanSize + 3] & 1;
        g_Pending.fit[1] = ~0u;     // (exchannel fit line)
        g_Pending.busy = 0;
        for (int i = 0; i < 16; i++) g_Pending.busy |= (nds.ARM7Read8(0x04000403 + i * 16) >> 7) << i;
        s.checks++;
        ok = false;
    }
#endif
    if (!ok)
    {
        s.fallbacks += !g_Check;
        // execute the real first instruction (push {r3-r11, lr}); the guest code follows
        u32 icode = ((cpu->CurInstr >> 4) & 0xF) | ((cpu->CurInstr >> 16) & 0xFF0);
        ARMInterpreter::ARMInstrTable[icode](cpu);
        return true;
    }
    s.native++;
    // changed words only, as the guest's stores leave them (kChannels and the size are word aligned)
    u8* dst = W7(nds, chA);
    for (u32 i = 0; i < sizeof(chans); i += 4)
        if (R32(dst + i) != R32(chans + i)) Store32<melonDS::ARMJIT_Memory::memregion_WRAM7>(nds, chA + i, dst + i, R32(chans + i));
    // ponytail: fixed cycle estimate (guest: ~7.3k cycles/tick for ~14 active channels on PW)
    int active = 0;
    for (int i = 0; i < 16; i++) active += chans[i * kChanSize + 3] & 1;
    cpu->Cycles += s.v->cyc0 + s.v->cycCh * active;
#ifdef LITEV_HLE_DIAG
    cpu->Cycles += g_CycAdj;
#endif
    cpu->JumpTo(cpu->R[14]);
    return true;
}

#ifdef LITEV_HLE_DIAG
void CheckAt(melonDS::ARM* cpu, u32 pc)
{
    if (cpu != g_Pending.cpu) return;
    if ((cpu->CPSR & 0x1F) == 0x12) { g_Pending.irq = true; return; }
    if (pc != g_Pending.ret) return;
    CheckPending = false;
    melonDS::NDS& nds = cpu->NDS;
    bool diff = false;
    for (auto& [addr, exp] : g_Pending.regions)
        for (u32 i = 0; i < exp.size(); i++)
        {
            u8 got = nds.ARM7Read8(addr + i);
            if (got == exp[i]) continue;
            if (!diff && *g_Pending.diffs < 10)
                fprintf(stderr, "A7HLE CHECK %s (cpu %p) diffs:", g_Pending.name, (const void*)cpu);
            if (*g_Pending.diffs < 10)
                fprintf(stderr, " %08x guest %02x native %02x;", addr + i, got, exp[i]);
            diff = true;
        }
    if (g_Pending.fit[1] == ~0u && !g_Pending.irq && getenv("LITEV_A7HLE_HWFIT"))
        fprintf(stderr, "EXFIT %u %llu\n", g_Pending.fit[0], (unsigned long long)(nds.ARM7Timestamp + cpu->Cycles - g_Pending.t0));
    if (g_Pending.fit[2] == ~0u && !g_Pending.irq && getenv("LITEV_A7HLE_HWFIT"))
        fprintf(stderr, "SEQFIT %u %u %u %u %u %llu\n", g_Pending.fit[0], g_Pending.fit[1], g_Pending.fit[3], g_Pending.fit[4], g_Pending.fit[5], (unsigned long long)(nds.ARM7Timestamp + cpu->Cycles - g_Pending.t0));
    if (g_Pending.regs && !g_Pending.irq && getenv("LITEV_A7HLE_HWFIT"))
        fprintf(stderr, "HWFIT %u %u %u %u %llu\n", g_Pending.fit[0], g_Pending.fit[1], g_Pending.fit[2], g_Pending.fit[3], (unsigned long long)(nds.ARM7Timestamp + cpu->Cycles - g_Pending.t0));
    g_Pending.fit[1] = g_Pending.fit[2] = g_Pending.fit[3] = 0;
    if (g_Pending.regs)
    {
        const int ri[5] = {0, 1, 2, 3, 12};
        for (int r : ri)
            if (!g_Pending.deferred && cpu->R[r] != g_Pending.R[r])
            {
                if (!diff && *g_Pending.diffs < 10) fprintf(stderr, "A7HLE CHECK %s diffs:", g_Pending.name);
                if (*g_Pending.diffs < 10) fprintf(stderr, " r%d guest %08x native %08x;", r, cpu->R[r], g_Pending.R[r]);
                diff = true;
            }
        if (!g_Pending.deferred && ((cpu->CPSR ^ g_Pending.cpsr) & 0xF0000000))
        {
            if (!diff && *g_Pending.diffs < 10) fprintf(stderr, "A7HLE CHECK %s diffs:", g_Pending.name);
            if (*g_Pending.diffs < 10) fprintf(stderr, " nzcv guest %x native %x;", cpu->CPSR >> 28, g_Pending.cpsr >> 28);
            diff = true;
        }
        for (auto& [w, v] : g_Pending.spu)
        {
            const int k = w & 7;            // (channel * 8 + what: 0 Cnt & 0xFFFF, 1 pan byte, 2 Cnt bit 31, 3 timer, 4 Cnt byte 0)
            if (k == 3 || (k == 2 && g_Pending.deferred)) continue;           // SOUNDxTMR is write-only (the timer path ran 0 times in the PW profiles)
            const u32 cnt = nds.ARM7Read32(0x04000400 + (w >> 3) * 16);
            const u32 got = k == 0 ? cnt & 0xFFFF : k == 1 ? (cnt >> 16) & 0x7F : k == 4 ? cnt & 0x7F : cnt & 0x80000000;
            const u32 want = k == 2 ? 0 : k == 1 || k == 4 ? v & 0x7F : v;
            if (got != want)
            {
                if (!diff && *g_Pending.diffs < 10) fprintf(stderr, "A7HLE CHECK %s diffs:", g_Pending.name);
                if (*g_Pending.diffs < 10) fprintf(stderr, " spu ch%d/%d guest %x native %x;", w >> 3, k, got, want);
                diff = true;
            }
        }
    }
    int busyNow = -1;
    if (g_Pending.busy >= 0)
    {
        busyNow = 0;
        for (int i = 0; i < 16; i++) busyNow |= (nds.ARM7Read8(0x04000403 + i * 16) >> 7) << i;
        // a channel that stopped inside the guest call: the native call sampled it at entry (documented category B)
        if (diff && busyNow != g_Pending.busy) { g_BusyDiffs++; g_Pending.busy = -1; return; }
        g_Pending.busy = -1;
    }
    if (diff)
    {
        if (*g_Pending.diffs < 10) fprintf(stderr, "%s\n", g_Pending.irq ? " (IRQ in the guest window: not counted)" : "");
        if (g_Pending.irq) g_IrqDiffs++;
        else (*g_Pending.diffs)++;
    }
}
#endif
}
#endif
