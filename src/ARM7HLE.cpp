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
bool CheckPending = false;
#endif
namespace
{
// ---- Pokemon Black/White (NitroSDK ARM7 SND driver in ARM7 WRAM) ---------------------------
// SND_ExChannelMain(BOOL step) at 0x03800A5C (ARM). Signature = FNV-1a 64 over the bytes of
// [0x0380021C,0x03800F48) (ExChannelMain + IsChannelActive, CalcTimer, CalcChannelVolume,
// SinIdx, envelope + literal pools), [0x03801418,0x038014D4) (UpdateLfo, GetLfoValue) and the
// shared-WRAM Thumb SVC stubs [0x037FB574,0x037FB598) (GetPitchTable / GetVolumeTable).
constexpr u32 kEntry = 0x03800A5C;
constexpr u32 kEntryInstr = 0xE92D4FF8;            // push {r3-r11, lr}
constexpr u32 kR1a = 0x0380021C, kR1b = 0x03800F48;
constexpr u32 kR2a = 0x03801418, kR2b = 0x038014D4;
constexpr u32 kStubA = 0x037FB574, kStubB = 0x037FB598;
constexpr u64 kSig = 0xd732d69f87473043ull;
constexpr u32 kChannels = 0x03809FBC, kChanSize = 0x54;
constexpr u32 kDbSqTable = 0x03808E48;              // SNDi_DecibelSquareTable (s16)
constexpr u32 kSinTable = 0x03808E24;               // LFO sine table (s8)
constexpr u32 kTwlFlag = 0x03FFFFC8;

// FreeBIOS ARM7 tables used by SVC 0x1B GetPitchTable (u16 x 768) and 0x1C GetVolumeTable
// (u8 x 724). A loaded BIOS must contain the same bytes (the real tables) or the HLE stays off.
constexpr u32 kFreePitch = 0x169C, kFreeVol = 0x1CB0;

struct State
{
    int status = 0;                 // 0 unprobed, 1 active, -1 off
    bool on = true;
    int copyOn = -1;                // MI_CpuCopy32 hook, latched separately (no signature probe)
    std::vector<u8> code;           // snapshot of R1+R2 (per-call memcmp)
    u32 stub[9] = {};
    u16 pitch[768] = {};
    u8 vol[724] = {};
    u64 copies = 0, calls = 0, native = 0, fallbacks = 0, checks = 0, checkDiffs = 0;
};
std::unordered_map<const melonDS::NDS*, State> g_State;
#ifdef LITEV_HLE_DIAG
bool g_Check = getenv("LITEV_A7HLE_CHECK") && atoi(getenv("LITEV_A7HLE_CHECK"));
bool g_CommitCheck = getenv("LITEV_A7HLE_COMMITCHECK") && atoi(getenv("LITEV_A7HLE_COMMITCHECK"));
#else
constexpr bool g_Check = false, g_CommitCheck = false;   // compare mode compiled out
#endif
bool g_Stats = getenv("LITEV_A7HLE_STATS") && atoi(getenv("LITEV_A7HLE_STATS"));


inline u8* W7(melonDS::NDS& nds, u32 a) { return &nds.ARM7WRAM[a & (ARM7WRAMSize - 1)]; }
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

bool ReadOn()
{
#if defined(__ANDROID__)
    char b[8] = {0}; int n = __system_property_get("debug.litev.a7hle", b);
    return (n > 0) ? (atoi(b) != 0) : true;
#else
    const char* e = getenv("debug.litev.a7hle"); return e ? (atoi(e) != 0) : true;
#endif
}

bool StubsMatch(melonDS::NDS& nds, const State& s)
{
    for (u32 i = 0; i < 9; i++)
        if (nds.ARM7Read32(kStubA + i * 4) != s.stub[i]) return false;
    return true;
}

void Probe(melonDS::NDS& nds, State& s)
{
    s.status = -1;
    s.on = ReadOn();
    if (!s.on) return;
    std::vector<u8> code;
    for (u32 a = kR1a; a < kR1b; a++) code.push_back(*W7(nds, a));
    for (u32 a = kR2a; a < kR2b; a++) code.push_back(*W7(nds, a));
    u8 stubBytes[kStubB - kStubA];
    for (u32 i = 0; i < 9; i++) { s.stub[i] = nds.ARM7Read32(kStubA + i * 4); memcpy(stubBytes + i * 4, &s.stub[i], 4); }
    u64 h = 0xcbf29ce484222325ull;
    auto mix = [&](const u8* p, size_t n) { for (size_t i = 0; i < n; i++) h = (h ^ p[i]) * 0x100000001b3ull; };
    mix(code.data(), code.size());
    mix(stubBytes, sizeof(stubBytes));
    if (h != kSig) return;

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
    if (s.status == 0) Probe(nds, s);
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

s32 SinIdx(melonDS::NDS& nds, s32 x)
{
    auto t = [&](s32 i) { return (s32)(s8)*W7(nds, kSinTable + i); };
    if (x < 0x20) return t(x);
    if (x < 0x40) return t(0x40 - x);
    if (x < 0x60) return (s32)(s8)(-t(x - 0x40));
    return (s32)(s8)(-t(0x20 - (x - 0x60)));
}

// Runs the update into chans (a copy of the 16 structs). Returns false if the guest code has
// to run instead (a channel callback would be called, or an unsupported path).
bool ExChannelMain(melonDS::NDS& nds, const State& s, u8* chans, bool step)
{
    if ((*W7(nds, kTwlFlag) & 0x04) && !(*W7(nds, kTwlFlag) & 0x20)) return false;   // DSi-BIOS workaround path
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

        s32 vol = (s16)R16(W7(nds, kDbSqTable + c[9] * 2));
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
                s32 sus = (s32)(s16)R16(W7(nds, kDbSqTable + c[0x1D] * 2)) << 7;
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
            lfoRaw = SinIdx(nds, R16(lfo + 8) >> 8) * (s32)lfo[2] * (s32)lfo[3];
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
        case 1: if (vol > -0x8000) vol += lfoVal; break;
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
    return memcmp(W7(nds, kR1a), s.code.data(), kR1b - kR1a) == 0
        && memcmp(W7(nds, kR2a), s.code.data() + (kR1b - kR1a), kR2b - kR2a) == 0
        && StubsMatch(nds, s);
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
constexpr u32 kSeqEntry = 0x038015C4;
constexpr u32 kSeqEntryInstr = 0xE92D47F0;         // push {r4-r10, lr}
constexpr u32 kSeqRa = 0x03801190, kSeqRb = 0x03802E74;
constexpr u32 kRndA = 0x03800574, kRndB = 0x038005A0;
constexpr u64 kSeqSig = 0x5b3b713e31e0c679ull;
constexpr u32 kWorkA = 0x03809F9C, kWorkB = 0x0380AF3C;
constexpr u32 kSharedPtr = 0x03809FB8, kSharedSize = 0x280;
constexpr u32 kRndState = 0x03809360;
constexpr u32 kPlayers = 0x0380A4FC, kTracks = 0x0380A73C;
constexpr u32 kAttackTable = 0x03808F5C;

struct SeqState
{
    int status = 0;
    std::vector<u8> code;
    u64 calls = 0, native = 0, fallbacks = 0, checks = 0, checkDiffs = 0;
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
    if (!ReadOn()) return s;
    std::vector<u8> code;
    for (u32 a = kSeqRa; a < kSeqRb; a++) code.push_back(*W7(nds, a));
    for (u32 a = kRndA; a < kRndB; a++) code.push_back(*W7(nds, a));
    u64 h = 0xcbf29ce484222325ull;
    for (u8 b : code) h = (h ^ b) * 0x100000001b3ull;
    if (h != kSeqSig) return s;
    s.code = std::move(code);
    s.status = 1;
    return s;
}

bool SeqCodeIntact(melonDS::NDS& nds, const SeqState& s)
{
    return memcmp(W7(nds, kSeqRa), s.code.data(), kSeqRb - kSeqRa) == 0
        && memcmp(W7(nds, kRndA), s.code.data() + (kSeqRb - kSeqRa), kRndB - kRndA) == 0;
}

struct Seq
{
    melonDS::NDS& nds;
    u8 work[kWorkB - kWorkA];
    u8 shared[kSharedSize];
    u8 rnd[4];
    u32 sw = 0;                     // shared work address (0 = none)
    bool bail = false;

    explicit Seq(melonDS::NDS& n) : nds(n)
    {
        memcpy(work, W7(nds, kWorkA), sizeof(work));
        memcpy(rnd, W7(nds, kRndState), 4);
        sw = R32(work + (kSharedPtr - kWorkA));
        if (sw)
        {
            // shared work in main RAM only, word aligned, not wrapping the RAM mask
            if ((sw >> 24) != 0x02 || (sw & 3) || ((sw & nds.MainRAMMask) + kSharedSize) > nds.MainRAMMask + 1)
            { bail = true; sw = 0; return; }
            memcpy(shared, &nds.MainRAM[sw & nds.MainRAMMask], kSharedSize);
        }
    }

    u64 dirtyW[(kWorkB - kWorkA + 255) / 256] = {};      // one bit per word
    u64 dirtyS[(kSharedSize + 255) / 256] = {};
    static void Mark(u64* bits, u32 o, u32 n)
    {
        for (u32 w = o >> 2; w <= (o + n - 1) >> 2; w++) bits[w >> 6] |= 1ull << (w & 63);
    }
    // writable overlay pointer for [a, a+n): marks those words for Commit
    u8* AtW(u32 a, u32 n)
    {
        if (a - kWorkA <= (kWorkB - kWorkA) - n) { Mark(dirtyW, a - kWorkA, n); return work + (a - kWorkA); }
        if (sw && a - sw <= kSharedSize - n) { Mark(dirtyS, a - sw, n); return shared + (a - sw); }
        if (a - kRndState <= 4 - n) return rnd + (a - kRndState);
        return nullptr;
    }
    // overlay pointer for [a, a+n)
    u8* At(u32 a, u32 n)
    {
        if (a - kWorkA <= (kWorkB - kWorkA) - n) return work + (a - kWorkA);
        if (sw && a - sw <= kSharedSize - n) return shared + (a - sw);
        if (a - kRndState <= 4 - n) return rnd + (a - kRndState);
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
        CommitRegion<M::memregion_WRAM7>(kWorkA, W7(nds, kWorkA), work, dirtyW, sizeof(dirtyW) / 8);
        if (R32(W7(nds, kRndState)) != R32(rnd)) Store32<M::memregion_WRAM7>(nds, kRndState, W7(nds, kRndState), R32(rnd));
        if (sw) CommitRegion<M::memregion_MainRAM>(sw, &nds.MainRAM[sw & nds.MainRAMMask], shared, dirtyS, sizeof(dirtyS) / 8);
    }

    // ---- helpers (addresses of the PW functions they mirror) ----
    u16 CalcRandom()                                    // 03800574
    {
        u32 st = r32(kRndState) * 0x19660Du + 0x3C6EF35Fu;
        w32(kRndState, st);
        return (u16)(st >> 16);
    }
    void CacheFetch(u32 addr)                           // 03801DCC
    {
        u32 a = addr & ~3u;
        w32(kWorkA + 4, a);
        w32(kWorkA + 8, a + 16);
        for (u32 i = 0; i < 4; i++) w32(kWorkA + 0xC + i * 4, nds.ARM7Read32(a + i * 4));
    }
    u8 ReadU8(u32 t)                                    // 038018DC
    {
        u32 cur = r32(t + 0x28);
        if (cur < r32(kWorkA + 4) || cur >= r32(kWorkA + 8)) CacheFetch(cur);
        u8 b = r8(kWorkA + 0xC + (cur - r32(kWorkA + 4)));
        w32(t + 0x28, cur + 1);
        return b;
    }
    u32 ReadU16(u32 t) { u32 lo = ReadU8(t); u32 hi = ReadU8(t); return lo | (hi << 8); }
    u32 ReadU24(u32 t) { u32 a = ReadU8(t); u32 b = ReadU8(t); u32 c = ReadU8(t); return a | (b << 8) | (c << 16); }
    u32 VarPtr(u32 p, u32 var)                          // 03802D78
    {
        u32 s = r32(kSharedPtr);
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
        return idx == 0xFF ? 0 : kTracks + idx * 0x40;
    }
    static u16 DecayCoeff(u32 v)                        // 038014D4
    {
        if (v == 0x7F) return 0xFFFF;
        if (v == 0x7E) return 0x3C00;
        if (v < 0x32) return (u16)(v * 2 + 1);
        return (u16)((s32)0x1E00 / (s32)(0x7E - v));
    }
    void SetAttack(u32 c, u32 v) { w8(c + 0x1C, v >= 0x6D ? r8(kAttackTable + (0x7F - v)) : (u8)(0xFF - v)); }
    void SetRelease(u32 c, u32 v) { w16(c + 0x20, DecayCoeff(v)); }

    void UpdateChannels(u32 t, u32 p, bool release)     // 038021AC TrackUpdateChannel
    {
        const u8* T = At(t, 0x40);
        const u8* P = At(p, 0x24);
        if (!T || !P) { bail = true; return; }
        const u8* db = W7(nds, kDbSqTable);
        s32 vol = (s16)R16(db + T[4] * 2) + (s16)R16(db + T[5] * 2) + (s16)R16(db + P[5] * 2);
        s32 pitch = (s16)R16(T + 0xC) + (((s32)(s8)T[6] * (s32)(T[7] << 6)) >> 7);
        s32 pan = (s8)T[8];
        u8 panRange = T[1];
        if (panRange != 0x7F) pan = (pan * panRange + 0x40) >> 7;
        pan += (s8)T[9];
        s32 fader = (s16)R16(T + 0xA) + (s16)R16(P + 6);
        if (vol < -0x8000) vol = -0x8000;
        if (fader < -0x8000) fader = -0x8000;
        if (pan < -128) pan = -128; else if (pan > 127) pan = 127;
        for (u32 c = R32(T + 0x3C); c; )
        {
            u8* C = At(c, 0x54);
            if (!C || c < kWorkA) { bail = true; return; }
            const u32 o = c - kWorkA;
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
        u32 ta = kTracks + r8(p + 8 + i) * 0x40;
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
                case 0xD7: Mute(t, p, par); break;
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
            u32 p = kPlayers + i * 0x24;
            u8 f = r8(p);
            if (!(f & 1)) continue;
            if (f & 2)
            {
                if (step && !(f & 4))
                {
                    u32 ticks = 0;
                    while (r16(p + 0x1C) >= 0xF0) { w16(p + 0x1C, (u16)(r16(p + 0x1C) - 0xF0)); ticks++; }
                    u32 j = 0;
                    for (; j < ticks && !bail; j++)
                        if (StepPlayer(p, true)) { StopPlayer(p); break; }
                    u32 s = r32(kSharedPtr);
                    if (s) { u32 a = s + r8(p + 1) * 0x24 + 0x40; w32(a, r32(a) + j); }
                    u32 inc = ((u32)r16(p + 0x18) * (u32)r16(p + 0x1A)) << 8;
                    w16(p + 0x1C, (u16)(r16(p + 0x1C) + (inc >> 16)));
                }
                for (u32 k = 0; k < 16 && !bail; k++)
                    if (u32 t = GetTrack(p, k)) { UpdateChannels(t, p, true); tracks++; }
            }
            if (r8(p) & 1) status |= 1u << i;
        }
        u32 s = r32(kSharedPtr);
        if (s) w32(s + 4, status);
        return tracks;
    }
};

#ifdef LITEV_HLE_DIAG
// check mode: one pending comparison (the hooked functions never nest)
struct Pending
{
    u32 ret = 0;
    const char* name = "";
    u64* diffs = nullptr;
    std::vector<std::pair<u32, std::vector<u8>>> regions;   // address, expected bytes
} g_Pending;

void ArmCheck(melonDS::ARM* cpu, const char* name, u64* diffs)
{
    g_Pending.ret = cpu->R[14] & ~1u;
    g_Pending.name = name;
    g_Pending.diffs = diffs;
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
            fprintf(stderr, "A7HLE seq: status=%d calls=%llu native=%llu fallback=%llu checks=%llu check_diffs=%llu\n",
                    s.status, (unsigned long long)s.calls, (unsigned long long)s.native,
                    (unsigned long long)s.fallbacks, (unsigned long long)s.checks, (unsigned long long)s.checkDiffs);
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
    if (s.copyOn < 0) s.copyOn = ReadOn();
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

bool RunSeq(melonDS::ARM* cpu, bool jit)
{
    melonDS::NDS& nds = cpu->NDS;
    SeqState& s = GetSeq(nds);
    if (s.status != 1) return false;
    s.calls++;
    Seq q(nds);
    int tracks = 0;
    bool ok = !q.bail && (jit || SeqCodeIntact(nds, s));
    if (ok) { tracks = q.Main(cpu->R[0] != 0); ok = !q.bail; }
#ifdef LITEV_HLE_DIAG
    if (ok && g_Check)
    {
        g_Pending.regions.clear();
        g_Pending.regions.push_back({kWorkA, std::vector<u8>(q.work, q.work + sizeof(q.work))});
        g_Pending.regions.push_back({kRndState, std::vector<u8>(q.rnd, q.rnd + 4)});
        if (q.sw) g_Pending.regions.push_back({q.sw, std::vector<u8>(q.shared, q.shared + kSharedSize)});
        ArmCheck(cpu, "seq", &s.checkDiffs);
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
        bool bad = memcmp(W7(nds, kWorkA), q.work, sizeof(q.work)) != 0 || memcmp(W7(nds, kRndState), q.rnd, 4) != 0
                || (q.sw && memcmp(&nds.MainRAM[q.sw & nds.MainRAMMask], q.shared, kSharedSize) != 0);
        if (bad && s.checkDiffs++ < 5) fprintf(stderr, "A7HLE COMMITCHECK seq: memory != overlay after commit\n");
    }
#endif
    // ponytail: fixed cycle estimate (guest: ~4.2k cycles/tick for ~10 tracks on PW)
    cpu->Cycles += 150 + 400 * tracks;
    cpu->JumpTo(cpu->R[14]);
    return true;
}
}

int IsHook(melonDS::NDS& nds, u32 addr, u32 instr)
{
    if (instr == kCopy32[0]) return CopyOn(nds) && IsCopy32(nds, addr) ? 1 : 0;
    if (addr == kSeqEntry && instr == kSeqEntryInstr)
    {
        SeqState& s = GetSeq(nds);
        return s.status != 1 ? 0 : SeqCodeIntact(nds, s) ? 1 : 2;
    }
    if (addr != kEntry || instr != kEntryInstr) return 0;
    State& s = Get(nds);
    return !(s.status == 1 && s.on) ? 0 : CodeIntact(nds, s) ? 1 : 2;
}
int Deps(u32 addr, u32 instr, Range* out)
{
    if (instr == kCopy32[0]) { out[0] = {addr, addr + 24}; return 1; }
    if (addr == kSeqEntry) { out[0] = {kSeqRa, kSeqRb}; out[1] = {kRndA, kRndB}; return 2; }
    out[0] = {kR1a, kR1b}; out[1] = {kR2a, kR2b}; out[2] = {kStubA, kStubB};
    return 3;
}

bool Run(melonDS::ARM* cpu, bool jit)
{
    if (cpu->Num != 1 || (cpu->CPSR & 0x20)) return false;
    const u32 pc = cpu->R[15] - 8;
    melonDS::NDS& nds = cpu->NDS;
    if (cpu->CurInstr == kCopy32[0])
    {
        if (!CopyOn(nds) || (!jit && !IsCopy32(nds, pc))) return false;
        g_State[&nds].copies++;
        Copy32(cpu);
        return true;
    }
    if (pc == kSeqEntry && cpu->CurInstr == kSeqEntryInstr) return RunSeq(cpu, jit);
    if (pc != kEntry || cpu->CurInstr != kEntryInstr) return false;
    State& s = Get(nds);
    if (s.status != 1 || !s.on) return false;
    s.calls++;

    u8 chans[16 * kChanSize];
    memcpy(chans, W7(nds, kChannels), sizeof(chans));
    bool ok = (jit || CodeIntact(nds, s)) && ExChannelMain(nds, s, chans, cpu->R[0] != 0);
#ifdef LITEV_HLE_DIAG
    if (ok && g_Check)
    {
        // run the guest too; compare at its return
        g_Pending.regions.assign(1, {kChannels, std::vector<u8>(chans, chans + sizeof(chans))});
        ArmCheck(cpu, "exchannel", &s.checkDiffs);
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
    u8* dst = W7(nds, kChannels);
    for (u32 i = 0; i < sizeof(chans); i += 4)
        if (R32(dst + i) != R32(chans + i)) Store32<melonDS::ARMJIT_Memory::memregion_WRAM7>(nds, kChannels + i, dst + i, R32(chans + i));
    // ponytail: fixed cycle estimate (guest: ~7.3k cycles/tick for ~14 active channels on PW)
    int active = 0;
    for (int i = 0; i < 16; i++) active += chans[i * kChanSize + 3] & 1;
    cpu->Cycles += 200 + 480 * active;
    cpu->JumpTo(cpu->R[14]);
    return true;
}

#ifdef LITEV_HLE_DIAG
void CheckAt(melonDS::ARM* cpu, u32 pc)
{
    if (pc != g_Pending.ret || (cpu->CPSR & 0x1F) == 0x12) return;
    CheckPending = false;
    melonDS::NDS& nds = cpu->NDS;
    bool diff = false;
    for (auto& [addr, exp] : g_Pending.regions)
        for (u32 i = 0; i < exp.size(); i++)
        {
            u8 got = nds.ARM7Read8(addr + i);
            if (got == exp[i]) continue;
            if (!diff && *g_Pending.diffs < 10)
                fprintf(stderr, "A7HLE CHECK %s diffs:", g_Pending.name);
            if (*g_Pending.diffs < 10)
                fprintf(stderr, " %08x guest %02x native %02x;", addr + i, got, exp[i]);
            diff = true;
        }
    if (diff)
    {
        if (*g_Pending.diffs < 10) fprintf(stderr, "\n");
        (*g_Pending.diffs)++;
    }
}
#endif
}
#endif
