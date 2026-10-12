// LITEV_A9HLE: see HLELeaf.h.
#ifdef LITEV_A9HLE
#include "HLELeaf.h"
#include "ARM9HLE.h"
#include "ARM.h"
#include "NDS.h"
#include "ARMInterpreter.h"
#include "ARMJIT.h"
#include "ARMJIT_Memory.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>
#if defined(__ANDROID__)
#include <sys/system_properties.h>
#endif

namespace melonDS::HLELeaf
{
namespace
{
inline u32 R32(const u8* p) { u32 v; memcpy(&v, p, 4); return v; }

// code word as the ARM9 sees it (ITCM or main RAM), 0 elsewhere (no bus access, no side effects)
u32 Code(melonDS::ARMv5* c, u32 a)
{
    if (a < c->ITCMSize) return R32(&c->ITCM[a & (ITCMPhysicalSize - 1)]);
    if ((a >> 24) == 0x02) return R32(&c->NDS.MainRAM[a & c->NDS.MainRAMMask & ~3u]);
    return 0;
}

enum Kind { kNone, kDcFlush, kDcStore, kDcInv, kMtx, kHook, kKinds };
constexpr u32 kDcFlushCode[9] = {0xE3A0C000, 0xE0811000, 0xE3C0001F, 0xEE07CF9A, 0xEE070F3E, 0xE2800020, 0xE1500001, 0xBAFFFFFA, 0xE12FFF1E};
// DC_StoreRange (c7, c10, 1) / DC_InvalidateRange (c7, c6, 1): the same loop without the drain
constexpr u32 kDcRangeCode[7] = {0xE0811000, 0xE3C0001F, 0, 0xE2800020, 0xE1500001, 0xBAFFFFFB, 0xE12FFF1E};
constexpr u32 kMcrStore = 0xEE070F3A, kMcrInv = 0xEE070F36;
// ponytail: cycles per call (BL to return) = base + per 32-byte line, fitted from check mode
constexpr s32 kCyc0[kKinds] = {0, 4, 7, 6, 0, 0}, kCycLine[kKinds] = {0, 14, 11, 10, 0, 0};   // (7 games, 45k calls: total within 0.1%, p95 14%)
const char* const kName[kKinds] = {"", "DC_FlushRange", "DC_StoreRange", "DC_InvalidateRange", "MTX 4x3*4x4", "A9HLE hook"};
// a straight-line fx32 matrix product (Shrek Smash n' Crash: 97 calls a frame, ~7.6% of its ARM9 guest instructions):
// o(r0) = a(r1) * b(r2) for a 4x3 (rows of 4 words, 3 used) and an affine 4x4 b (row 3 added), 64-bit sums >> 12.
// push {r4-r11, lr} .. pop {r4-r11, pc}, 164 words, no literals / branches: exact code (FNV), anywhere.
constexpr u32 kMtxW0 = 0xE92D4FF0, kMtxW1 = 0xE5917000, kMtxWords = 164;
constexpr u64 kMtxSig = 0x5ec634cd221a3caeull;
constexpr s32 kMtxCyc = 342;     // (check-mode guest average incl. the BL and the return: 57.9k calls, p95 12%)

bool Eq(melonDS::ARMv5* c, u32 a, const u32* w, u32 n, u32 hole = ~0u, u32 hv = 0)
{
    for (u32 i = 0; i < n; i++)
        if (Code(c, a + 4 * i) != (i == hole ? hv : w[i])) return false;
    return true;
}
// verify: compare the whole code (compile time); else only enough to tell the kinds apart (the block depends on the code)
int KindAt(melonDS::ARMv5* c, u32 t, u32& len, bool verify = true)
{
    if (t & 1) return kNone;                            // (ARM functions only)
    const u32 w0 = Code(c, t);
    if (w0 == kMtxW0 && Code(c, t + 4) == kMtxW1)
    {
        len = kMtxWords * 4;
        if (!verify) return kMtx;
        u64 h = 0xcbf29ce484222325ull;
        for (u32 i = 0; i < kMtxWords; i++)
        {
            const u32 w = Code(c, t + 4 * i);
            for (int k = 0; k < 4; k++) h = (h ^ ((w >> (8 * k)) & 0xFF)) * 0x100000001b3ull;
        }
        return h == kMtxSig ? kMtx : kNone;
    }
    if (w0 == kDcFlushCode[0]) { len = 9 * 4; return !verify || Eq(c, t, kDcFlushCode, 9) ? kDcFlush : kNone; }
    if (w0 == kDcRangeCode[0])
    {
        len = 7 * 4;
        const u32 m = Code(c, t + 8);
        if (m != kMcrStore && m != kMcrInv) return kNone;
        return !verify || Eq(c, t, kDcRangeCode, 7, 2, m) ? (m == kMcrStore ? kDcStore : kDcInv) : kNone;
    }
    return kNone;
}

struct Stats { u64 calls[kKinds] = {}, native[kKinds] = {}, checks[kKinds] = {}, diffs[kKinds] = {}; ~Stats(); };
Stats g_St;
const bool g_Stats = getenv("LITEV_HLELEAF_STATS") != nullptr;
#ifdef LITEV_HLE_DIAG
const bool g_Check = getenv("LITEV_HLELEAF_CHECK") != nullptr;
#else
constexpr bool g_Check = false;
#endif
Stats::~Stats()
{
    if (!g_Stats) return;
    for (int k = 1; k < kKinds; k++)
        fprintf(stderr, "HLELeaf %s: calls=%llu native=%llu checks=%llu check_diffs=%llu\n", kName[k], (unsigned long long)calls[k],
                (unsigned long long)native[k], (unsigned long long)checks[k], (unsigned long long)diffs[k]);
}

u32 Flags(u32 a, u32 b)   // NZCV of cmp a, b
{
    u32 r = a - b;
    return (r & 0x80000000) | ((r == 0) << 30) | ((a >= b) << 29) | ((((a ^ b) & (a ^ r)) >> 31) << 28);
}

// the native result: registers r0-r3, r12, NZCV and cycles (the caller's lr / pc untouched); false: the guest runs
// guest memory writes of a native (applied only when its result is taken); wp: host pointer, wa: guest address | 1 if DTCM
struct Out { u32 r[4], ip, nzcv; s32 cyc; u32 n; u32 nw = 0; u8* wp[32]; u32 wa[32], wv[32]; };   // n: the cycle model's count
// host pointer of [a, a + len) in main RAM or DTCM (one block, word aligned), else nullptr; dtcm set for DTCM
u8* Host(melonDS::ARMv5* c, u32 a, u32 len, bool& dtcm)
{
    if (a & 3) return nullptr;
    dtcm = (a & c->DTCMMask) == c->DTCMBase && ((a + len - 1) & c->DTCMMask) == c->DTCMBase && a >= c->ITCMSize;
    if (dtcm) return (a & (DTCMPhysicalSize - 1)) + len <= DTCMPhysicalSize ? &c->DTCM[a & (DTCMPhysicalSize - 1)] : nullptr;
    if ((a >> 24) != 0x02 || ((a + len - 1) >> 24) != 0x02 || a < c->ITCMSize) return nullptr;
    const u32 m = c->NDS.MainRAMMask;
    if ((a & m) + len > m + 1 || ((a + len - 1) & c->DTCMMask) == c->DTCMBase) return nullptr;
    return &c->NDS.MainRAM[a & m];
}
void Commit(melonDS::ARMv5* c, const Out& o)
{
    for (u32 i = 0; i < o.nw; i++)
    {
        if (R32(o.wp[i]) == o.wv[i]) continue;
        if (!(o.wa[i] & 1)) c->NDS.JIT.CheckAndInvalidate<0, melonDS::ARMJIT_Memory::memregion_MainRAM>(o.wa[i]);
        memcpy(o.wp[i], &o.wv[i], 4);
    }
}
bool Native(melonDS::ARMv5* c, int k, Out& o, u32 ret)
{
    for (int i = 0; i < 4; i++) o.r[i] = c->R[i];
    o.ip = c->R[12];
    switch (k)
    {
    case kMtx:
    {
        u32 r0 = c->R[0], r1 = c->R[1], r2 = c->R[2], r3 = c->R[3], r4 = c->R[4], r5 = c->R[5], r6 = c->R[6], r7 = c->R[7],
            r8 = c->R[8], r9 = c->R[9], r10 = c->R[10], r11 = c->R[11];
        const u32 sp = c->R[13];
        bool d0, d1, d2, ds;
        u8 *Pr0 = Host(c, r0, 0x40, d0), *Pr1 = Host(c, r1, 0x3C, d1), *Pr2 = Host(c, r2, 0x40, d2), *Ps = Host(c, sp - 36, 36, ds);
        // the output must not overlap what is read (the guest interleaves loads and stores): guest
        if (!Pr0 || !Pr1 || !Pr2 || !Ps || (r0 < r1 + 0x3C && r1 < r0 + 0x40) || (r0 < r2 + 0x40 && r2 < r0 + 0x40)
            || (sp - 36 < r0 + 0x40 && r0 < sp)) return false;
        const u32 push[9] = {r4, r5, r6, r7, r8, r9, r10, r11, ret};
        for (int i = 0; i < 9; i++) { o.wp[o.nw] = Ps + 4 * i; o.wa[o.nw] = (sp - 36 + 4 * i) | ds; o.wv[o.nw++] = push[i]; }
#define RD(b, off) R32(P##b + (off))
#define WR(b, off, v) do { o.wp[o.nw] = P##b + (off); o.wa[o.nw] = (b + (off)) | d0; o.wv[o.nw++] = (v); } while (0)
#include "HLELeafMtx.inc"
#undef RD
#undef WR
        (void)r4; (void)r5; (void)r6; (void)r7; (void)r8; (void)r9; (void)r10; (void)r11;
        o.r[3] = r3;                                // (r4-r11 popped: the caller's; r0-r2, ip, flags untouched)
        o.nzcv = c->CPSR & 0xF0000000;
        o.cyc = kMtxCyc;
        o.n = 1;
        return true;
    }
    case kDcFlush: case kDcStore: case kDcInv:
    {
        // add r1, r1, r0; bic r0, r0, #0x1f; do { mcr..; add r0, r0, #0x20; } while ((s32)r0 < (s32)r1)
        const u32 end = c->R[1] + c->R[0];
        u32 a = c->R[0] & ~0x1Fu;
        // a range that wraps around the signed boundary: the guest (exact, rare)
        if ((s32)a < 0 || (s32)end < 0 || (s32)(a + 0x20) < 0) return false;
        u32 lines = (s32)end > (s32)(a + 0x20) ? (end - a + 0x1F) >> 5 : 1;
        if ((s32)(a + 0x20 * lines) < 0) return false;
        a += 0x20 * lines;
        o.r[0] = a; o.r[1] = end;
        if (k == kDcFlush) o.ip = 0;
        o.nzcv = Flags(a, end);
        o.cyc = kCyc0[k] + kCycLine[k] * (s32)lines;
        o.n = lines;
        return true;
    }
    }
    return false;
}

// the guest function from the call to its return, in the interpreter (target: bit 0 = Thumb; ret: the caller's next
// instruction, bit 0 = Thumb caller). The caller's lr is set as the call would.
void Guest(melonDS::ARMv5* c, u32 target, u32 ret, bool started = false)
{
    if (started)
    {
        // an A9HLE hook ran the function's first instruction (its fallback): go on from there (a pipeline refill)
        const u32 pc = (c->CPSR & 0x20) ? (c->R[15] - 2) | 1 : c->R[15] - 4;
        c->SetupCodeMem(pc & ~1u);
        c->JumpTo(pc);
    }
    else
    {
        c->R[14] = ret;
        c->SetupCodeMem(target & ~1u);
        c->JumpTo(target);
    }
    const bool rt = ret & 1; const u32 ra = ret & ~1u;
    for (u64 n = 0; n < 4000000; n++)
    {
        if (c->CPSR & 0x20)
        {
            c->R[15] += 2;
            c->CurInstr = c->NextInstr[0];
            c->NextInstr[0] = c->NextInstr[1];
            if (c->R[15] & 2) { c->NextInstr[1] >>= 16; c->CodeCycles = 0; }
            else c->NextInstr[1] = c->CodeRead32(c->R[15], false);
            if (melonDS::A9HLE::CheckPending) melonDS::A9HLE::CheckAt(c, c->R[15] - 4);   // (A9HLE compare mode)
            if (rt && c->R[15] - 4 == ra) return;
            ARMInterpreter::THUMBInstrTable[(c->CurInstr >> 6) & 0x3FF](c);
        }
        else
        {
            c->R[15] += 4;
            c->CurInstr = c->NextInstr[0];
            c->NextInstr[0] = c->NextInstr[1];
            c->NextInstr[1] = c->CodeRead32(c->R[15], false);
            if (melonDS::A9HLE::CheckPending) melonDS::A9HLE::CheckAt(c, c->R[15] - 8);
            if (!rt && c->R[15] - 8 == ra) return;
            if (c->CheckCondition(c->CurInstr >> 28))
                ARMInterpreter::ARMInstrTable[((c->CurInstr >> 4) & 0xF) | ((c->CurInstr >> 16) & 0xFF0)](c);
            else if ((c->CurInstr & 0xFE000000) == 0xFA000000) ARMInterpreter::A_BLX_IMM(c);
            else c->AddCycles_C();
        }
    }
    fprintf(stderr, "HLELeaf: guest call to %08x never returned\n", target);
}

// the call at the current instruction: target (bit 0 Thumb), ret (bit 0 Thumb caller), the caller's R15 convention
void Call(melonDS::ARMv5* c, u32 target, u32 ret, u32 r15)
{
    u32 len;
    const int k = KindAt(c, target, len, false);    // (verified at the block's compile: the block depends on it; this is a
                                                // dispatch on 1-2 words, not a check)
    const MemRegion codeMem = c->CodeMem;
    const u32 codeRegion = c->CodeRegion, cpsrT = c->CPSR & 0x20;
    const s32 codeCycles = c->CodeCycles, regionCycles = c->RegionCodeCycles;
    const u32 next0 = c->NextInstr[0], next1 = c->NextInstr[1];
    g_St.calls[k ? k : kHook]++;
    Out o;
    if (!k)
    {
        // an A9HLE hook entry (CallableHook at compile time): its native runs as at its JIT hook block, the BL done first
        c->R[14] = ret;
        c->CPSR &= ~0x20u;
        c->R[15] = target + 8;
        c->CurInstr = Code(c, target);
        if (!melonDS::A9HLE::Run(c, true)) Guest(c, target, ret);
        else if (c->R[15] != (ret & 1 ? (ret & ~1u) + 2 : ret + 4)) Guest(c, target, ret, true);
        else g_St.native[kHook]++;
    }
    else if (melonDS::A9HLE::MaskOn(c->NDS, 16777216) && Native(c, k, o, ret))
    {
        if (g_Check)
        {
#ifdef LITEV_HLE_DIAG
            const s32 cyc0 = c->Cycles;
            Guest(c, target, ret);
            const s32 cyc = c->Cycles - cyc0;
            g_St.checks[k]++;
            bool d = (c->CPSR >> 28) != (o.nzcv >> 28) || c->R[12] != o.ip;
            for (int i = 0; i < 4; i++) d |= c->R[i] != o.r[i];
            for (u32 i = 0; i < o.nw; i++)
                if (R32(o.wp[i]) != o.wv[i])
                {
                    if (g_St.diffs[k] < 10) fprintf(stderr, "HLELeaf CHECK %s memory %08x guest %08x native %08x\n", kName[k], o.wa[i] & ~1u, R32(o.wp[i]), o.wv[i]);
                    d = true;
                }
            if (d && g_St.diffs[k]++ < 10)
                fprintf(stderr, "HLELeaf CHECK %s diff: guest r0-3 %08x %08x %08x %08x ip %08x nzcv %x, native %08x %08x %08x %08x ip %08x nzcv %x\n",
                        kName[k], c->R[0], c->R[1], c->R[2], c->R[3], c->R[12], c->CPSR >> 28, o.r[0], o.r[1], o.r[2], o.r[3], o.ip, o.nzcv >> 28);
            if (getenv("LITEV_HLELEAF_FIT")) fprintf(stderr, "LEAFFIT%d %u %d\n", k, o.n, cyc);
#endif
        }
        else
        {
            g_St.native[k]++;
            Commit(c, o);
            for (int i = 0; i < 4; i++) c->R[i] = o.r[i];
            c->R[12] = o.ip;
            c->R[14] = ret;
            c->CPSR = (c->CPSR & 0x0FFFFFFF) | o.nzcv;
            c->Cycles += o.cyc;
        }
    }
    else
        Guest(c, target, ret);
    // back at the caller's next instruction, as a non-branching instruction leaves the CPU
    c->CPSR = (c->CPSR & ~0x20u) | cpsrT;
    c->R[15] = r15;
    c->CodeMem = codeMem; c->CodeRegion = codeRegion; c->CodeCycles = codeCycles; c->RegionCodeCycles = regionCycles;
    c->NextInstr[0] = next0; c->NextInstr[1] = next1;
}
}

int At(melonDS::NDS& nds, u32 target, u32* a, u32* b, int max)
{
    if (!melonDS::A9HLE::MaskOn(nds, 16777216)) return 0;
    u32 len = 0;
    const int k = KindAt(&nds.ARM9, target, len);
    if (k) { a[0] = target & ~1u; b[0] = a[0] + len; return 1; }
    // A/B: hooks through their block exits instead (Android: debug.litev.hlehooks=0, else env LITEV_HLELEAF_NOHOOKS=1)
    static const bool noHooks = []
    {
#if defined(__ANDROID__)
        char b[8] = {0};
        return __system_property_get("debug.litev.hlehooks", b) > 0 && atoi(b) == 0;
#else
        return getenv("LITEV_HLELEAF_NOHOOKS") != nullptr;
#endif
    }();
    if (noHooks) return 0;
    const melonDS::A9HLE::Range* r;
    const int n = melonDS::A9HLE::CallableHook(nds, target, r);
    if (n < 0 || n > max) return 0;
    for (int i = 0; i < n; i++) { a[i] = r[i].a; b[i] = r[i].b; }
    return n ? n : 0;
}

void CallA(melonDS::ARM* cpu)
{
    melonDS::ARMv5* c = (melonDS::ARMv5*)cpu;
    const u32 r15 = c->R[15], w = c->CurInstr;
    s32 off = (s32)(w << 8) >> 6;
    u32 t = r15 + off;
    if ((w >> 28) == 0xF) t = (t + ((w >> 23) & 2)) | 1;     // BLX imm: Thumb target
    Call(c, t, r15 - 4, r15);
}

void CallT(melonDS::ARM* cpu)
{
    // the second half of a BL / BLX pair: lr = first half's partial target, R15 = this halfword + 4
    melonDS::ARMv5* c = (melonDS::ARMv5*)cpu;
    const u32 r15 = c->R[15], w = c->CurInstr;
    u32 t = c->R[14] + ((w & 0x7FF) << 1);
    t = (w & (1 << 12)) ? t | 1 : t & ~3u;
    Call(c, t, (r15 - 2) | 1, r15);
}
}
#endif
