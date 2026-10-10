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
bool g_Check = getenv("LITEV_A7HLE_CHECK") && atoi(getenv("LITEV_A7HLE_CHECK"));
bool g_Stats = getenv("LITEV_A7HLE_STATS") && atoi(getenv("LITEV_A7HLE_STATS"));

struct StatsDump
{
    ~StatsDump()
    {
        if (!g_Stats) return;
        for (auto& [k, s] : g_State)
            fprintf(stderr, "A7HLE: status=%d copies=%llu calls=%llu native=%llu fallback=%llu checks=%llu check_diffs=%llu\n",
                    s.status, (unsigned long long)s.copies, (unsigned long long)s.calls, (unsigned long long)s.native,
                    (unsigned long long)s.fallbacks, (unsigned long long)s.checks, (unsigned long long)s.checkDiffs);
    }
} g_StatsDump;

inline u8* W7(melonDS::NDS& nds, u32 a) { return &nds.ARM7WRAM[a & (ARM7WRAMSize - 1)]; }
inline u16 R16(const u8* p) { u16 v; memcpy(&v, p, 2); return v; }
inline u32 R32(const u8* p) { u32 v; memcpy(&v, p, 4); return v; }
inline void W16(u8* p, u16 v) { memcpy(p, &v, 2); }
inline void W32(u8* p, u32 v) { memcpy(p, &v, 4); }

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
    State& s = g_State[&nds];
    if (s.status == 0) Probe(nds, s);
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

// check mode state
u8 g_Expected[16 * kChanSize];
u32 g_CheckRet = 0;
}

bool CheckPending = false;

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

bool IsHook(melonDS::NDS& nds, u32 addr, u32 instr)
{
    if (instr == kCopy32[0]) return CopyOn(nds) && IsCopy32(nds, addr);
    if (addr != kEntry || instr != kEntryInstr) return false;
    State& s = Get(nds);
    return s.status == 1 && s.on;
}

bool Run(melonDS::ARM* cpu)
{
    if (cpu->Num != 1 || (cpu->CPSR & 0x20)) return false;
    const u32 pc = cpu->R[15] - 8;
    melonDS::NDS& nds = cpu->NDS;
    if (cpu->CurInstr == kCopy32[0])
    {
        if (!CopyOn(nds) || !IsCopy32(nds, pc)) return false;
        g_State[&nds].copies++;
        Copy32(cpu);
        return true;
    }
    if (pc != kEntry || cpu->CurInstr != kEntryInstr) return false;
    State& s = Get(nds);
    if (s.status != 1 || !s.on) return false;
    s.calls++;

    u8 chans[16 * kChanSize];
    memcpy(chans, W7(nds, kChannels), sizeof(chans));
    bool ok = CodeIntact(nds, s) && ExChannelMain(nds, s, chans, cpu->R[0] != 0);
    if (ok && g_Check)
    {
        // run the guest too; compare at its return
        memcpy(g_Expected, chans, sizeof(chans));
        g_CheckRet = cpu->R[14] & ~1u;
        CheckPending = true;
        s.checks++;
        ok = false;
    }
    if (!ok)
    {
        s.fallbacks += !g_Check;
        // execute the real first instruction (push {r3-r11, lr}); the guest code follows
        u32 icode = ((cpu->CurInstr >> 4) & 0xF) | ((cpu->CurInstr >> 16) & 0xFF0);
        ARMInterpreter::ARMInstrTable[icode](cpu);
        return true;
    }
    s.native++;
    u8* dst = W7(nds, kChannels);
    for (u32 i = 0; i < sizeof(chans); i++)
        if (dst[i] != chans[i]) nds.ARM7Write8(kChannels + i, chans[i]);
    // ponytail: fixed cycle estimate (guest: ~7.3k cycles/tick for ~14 active channels on PW)
    int active = 0;
    for (int i = 0; i < 16; i++) active += chans[i * kChanSize + 3] & 1;
    cpu->Cycles += 200 + 480 * active;
    cpu->JumpTo(cpu->R[14]);
    return true;
}

void CheckAt(melonDS::ARM* cpu, u32 pc)
{
    if (pc != g_CheckRet || (cpu->CPSR & 0x1F) == 0x12) return;
    CheckPending = false;
    melonDS::NDS& nds = cpu->NDS;
    State& s = Get(nds);
    const u8* got = W7(nds, kChannels);
    if (memcmp(got, g_Expected, sizeof(g_Expected)) == 0) return;
    s.checkDiffs++;
    if (s.checkDiffs <= 10)
        for (u32 i = 0; i < sizeof(g_Expected); i++)
            if (got[i] != g_Expected[i])
                fprintf(stderr, "A7HLE CHECK diff: chan %u +0x%02x guest %02x native %02x\n", i / kChanSize, i % kChanSize, got[i], g_Expected[i]);
}
}
#endif
