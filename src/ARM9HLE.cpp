// LITEV_A9HLE: see ARM9HLE.h.
#ifdef LITEV_A9HLE
#include "ARM9HLE.h"
#include "ARM.h"
#include "NDS.h"
#include "ARMInterpreter.h"
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <chrono>
#if defined(__ANDROID__)
#include <sys/system_properties.h>
#endif

namespace melonDS::A9HLE
{
bool CheckPending = false;
namespace
{
// ---- Pokemon Black/White (USA/EU), NitroSDK ARM9 OS --------------------------------------
constexpr u32 kWake = 0x01FF8160, kWakeInstr = 0xE58C2064;    // OS_IrqHandler_ThreadSwitch wake loop (queue not empty): str r2, [ip, #0x64]
constexpr u32 kSet = 0x0208478C, kSetInstr = 0xE92D47F0;      // OS_SetIrqFunction: push {r4-r10, lr}
constexpr u32 kGet = 0x02084830, kGetInstr = 0xE59F207C;      // OS_GetIrqFunction: ldr r2, =OS_IRQTable

constexpr u32 kIrqQueue = 0x02FE00A0;   // OS_IrqThreadQueue {head, tail} (DTCM)
constexpr u32 kIrqCheck = 0x02FE3FF8;   // OS_IRQ check flags (DTCM)
constexpr u32 kIrqTable = 0x02FE0020;   // OS_IRQTable[32] (DTCM)
constexpr u32 kIrqTable2 = 0x02150F58;  // DMA / timer / ... entries {func, enable, arg} x 12
constexpr u32 kInfo = 0x0215100C;       // OSThreadInfo: +0 u16 isNeedRescheduling, +4 current, +8 list, +C switchCallback
constexpr u32 kOsi = 0x02150FF0;        // +0 switchCallback, +4 (nonzero: no reschedule), +8 &current, +1E u16 reschedule lock
// OSThread: +0 context {cpsr, r0-r14, pc, sp_svc, CP context (+0x48, 0x1C bytes)}, +0x64 state,
// +0x68 list link, +0x70 priority, +0x78 queue, +0x7C/+0x80 queue link prev/next
// return addresses of a thread asleep in OS_WaitIrq: OS_SaveContext's resume point and the
// call chain OS_RescheduleThread <- OS_SleepThread <- OS_WaitIrq loop
constexpr u32 kCtxPc = 0x02085C68, kCtxLr = 0x02085224, kRetSleep = 0x02085808, kRetWait = 0x02084610, kRetLoad = 0x02085270;
constexpr u32 kBiosLdm = 0xE8BD500F, kBiosSubs = 0xE25EF004;  // BIOS IRQ epilogue: ldmia sp!, {r0-r3,r12,lr}; subs pc, lr, #4

// every function the round trip / hooks execute, with literal pools
constexpr Range kCode[kNumCode] = {
    {0x01FF80F0, 0x01FF82A8},   // OS_IrqHandler + OS_IrqHandler_ThreadSwitch (ITCM)
    {0x020775D0, 0x0207764C},   // CP_SaveContext, CP_RestoreContext
    {0x020845B4, 0x02084628},   // OS_WaitIrq
    {0x0208478C, 0x020848BC},   // OS_SetIrqFunction, OS_GetIrqFunction
    {0x02084FE4, 0x0208505C},   // thread queue insert
    {0x020851B8, 0x0208527C},   // OS_RescheduleThread
    {0x020857C8, 0x02085818},   // OS_SleepThread
    {0x020858A8, 0x020858D4},   // select runnable thread
    {0x02085C20, 0x02085CD4},   // OS_SaveContext, OS_LoadContext
    {0x02085D54, 0x02085D9C},   // (TWL divider check)
    {0x020879A0, 0x020879CC},   // OS_DisableInterrupts, OS_RestoreInterrupts
    {0x02087A04, 0x02087A10},   // OS_GetProcMode
};
constexpr u64 kSig = 0xc03b33186e018603ull;

// ponytail: fixed cycle estimates (guest averages measured in check mode on PW)
constexpr s32 kWakeCycles = 900, kSetCycles = 500, kGetCyclesBase = 27, kGetCyclesPerBit = 17;

struct State
{
    int status = 0;                 // 0 unprobed, 1 active, -1 off
    u32 mask = 7;                   // 1 wake, 2 set, 4 get
    std::vector<u8> code;
    u64 calls[3] = {}, native[3] = {}, fallback[3] = {}, checks[3] = {}, diffs[3] = {}, irqDuring[3] = {};
    u64 guestCyc[3] = {}, guestN[3] = {}, getBits = 0;
    u32 biosOk = 0;                 // BIOS IRQ epilogue address verified once
    u64 ns = 0;                     // stats: host time inside Run (native calls)
};
std::unordered_map<const melonDS::NDS*, State> g_State;
bool g_Check = getenv("LITEV_A9HLE_CHECK") && atoi(getenv("LITEV_A9HLE_CHECK"));
bool g_Stats = getenv("LITEV_A9HLE_STATS") && atoi(getenv("LITEV_A9HLE_STATS"));
// cost measurement: compute the native result (nothing written) and run the guest code anyway
bool g_Dry = getenv("LITEV_A9HLE_DRY") && atoi(getenv("LITEV_A9HLE_DRY"));
const char* kName[3] = {"irqwake", "setirqfn", "getirqfn"};

// stats only
u64 Now() { return (u64)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
double TimerNs()   // cost of one Now() pair, subtracted from the per-call figure
{
    u64 t = Now(), z = 0;
    for (int i = 0; i < 100000; i++) { u64 a = Now(); z += Now() - a; }
    (void)t;
    return (double)z / 100000;
}
inline u32 R32(const u8* p) { u32 v; memcpy(&v, p, 4); return v; }
inline u16 R16(const u8* p) { u16 v; memcpy(&v, p, 2); return v; }

bool ReadOn()
{
#if defined(__ANDROID__)
    char b[8] = {0}; int n = __system_property_get("debug.litev.a9hle", b);
    return (n > 0) ? (atoi(b) != 0) : true;
#else
    const char* e = getenv("debug.litev.a9hle"); return e ? (atoi(e) != 0) : true;
#endif
}

// code bytes as the ARM9 sees them (ITCM or main RAM), no timing / side effects
const u8* CodePtr(melonDS::ARMv5* c, u32 a)
{
    if (a < c->ITCMSize) return &c->ITCM[a & (ITCMPhysicalSize - 1)];
    if ((a >> 24) == 0x02) return &c->NDS.MainRAM[a & c->NDS.MainRAMMask];
    return nullptr;
}

void CodeBytes(melonDS::ARMv5* c, std::vector<u8>& out)
{
    out.clear();
    for (auto& r : kCode)
        for (u32 a = r.a; a < r.b; a++) { const u8* p = CodePtr(c, a); out.push_back(p ? *p : 0); }
}

bool CodeIntact(melonDS::ARMv5* c, const State& s, int k)
{
    size_t o = 0;
    for (auto& r : kCode)
    {
        if (k != 0 && r.a != 0x0208478C) { o += r.b - r.a; continue; }
        u32 n = r.b - r.a;
        const u8* p = CodePtr(c, r.a);
        // ranges never straddle a mirror boundary
        if (!p || memcmp(p, s.code.data() + o, n) != 0) return false;
        o += n;
    }
    return true;
}

State& Get(melonDS::ARMv5* c)
{
    if (c->A9HLEState) return *(State*)c->A9HLEState;
    State& s = g_State[&c->NDS];         // map nodes are stable
    c->A9HLEState = &s;
    if (s.status != 0) return s;
    s.status = -1;
    if (!ReadOn()) return s;
    if (const char* m = getenv("LITEV_A9HLE_ONLY")) s.mask = (u32)strtoul(m, nullptr, 0);
    std::vector<u8> code;
    CodeBytes(c, code);
    u64 h = 0xcbf29ce484222325ull;
    for (u8 b : code) h = (h ^ b) * 0x100000001b3ull;
    if (h != kSig)
    {
        if (g_Stats) fprintf(stderr, "A9HLE: signature %016llx != PW, off\n", (unsigned long long)h);
        return s;
    }
    s.code = std::move(code);
    s.status = 1;
    return s;
}

// ---- guest memory view with a write log --------------------------------------------------
// Reads are host loads from main RAM / DTCM. Writes (only after every address was validated)
// go straight to memory and only when the word changes; in check mode they are logged instead.
struct Wr { u32 a, v; u8 sz; };
// a validated, contiguous guest object in host memory
struct Obj
{
    u8* p = nullptr; u32 a = 0; bool dtcm = false;
    explicit operator bool() const { return p != nullptr; }
    u32 r(u32 o) const { return R32(p + o); }
    u16 r16(u32 o) const { return R16(p + o); }
};
struct Mem
{
    melonDS::ARMv5* c;
    bool logOnly;
    u32 n = 0;
    bool bad = false;
    Wr log[64];
    Mem(melonDS::ARMv5* cpu, bool check) : c(cpu), logOnly(check) {}
    // DTCM or main RAM (the only places the hooked code writes), else nullptr
    __attribute__((always_inline)) u8* P(u32 a)
    {
        if (a < c->ITCMSize) return nullptr;
        if ((a & c->DTCMMask) == c->DTCMBase) return &c->DTCM[a & (DTCMPhysicalSize - 1)];
        if ((a >> 24) == 0x02) return &c->NDS.MainRAM[a & c->NDS.MainRAMMask];
        return nullptr;
    }
    u32 r32(u32 a) { a &= ~3u; if (u8* p = P(a)) return R32(p); bad = true; return 0; }
    u16 r16(u32 a) { a &= ~1u; if (u8* p = P(a)) return R16(p); bad = true; return 0; }
    bool ok(u32 a) { return P(a) != nullptr; }
    // [a, a+len) in one host block (main RAM or DTCM, no mirror wrap), else empty
    __attribute__((always_inline)) Obj O(u32 a, u32 len)
    {
        Obj x;
        u8* p = P(a);
        if (!p || (a & 3) || P(a + len - 1) != p + len - 1) return x;
        x.p = p; x.a = a; x.dtcm = (a & c->DTCMMask) == c->DTCMBase;
        return x;
    }
    // queue a word write (applied by Flush, after every check passed)
    struct Pw { u8* p; u32 a, v; bool d; } pw[48];
    u32 np = 0;
    __attribute__((always_inline)) void W(const Obj& x, u32 o, u32 v) { pw[np++] = {x.p + o, x.a + o, v, x.dtcm}; }
    // only changed words are written: DTCM directly (never holds JIT code), main RAM through the
    // bus (JIT invalidation); in check mode they are logged instead
    void Flush()
    {
        for (u32 i = 0; i < np; i++)
        {
            const Pw& w = pw[i];
            if (__builtin_expect(logOnly, 0)) { if (n < 64) log[n++] = {w.a, w.v, 4}; else bad = true; continue; }
            if (R32(w.p) == w.v) continue;
            if (w.d) memcpy(w.p, &w.v, 4);
            else c->NDS.ARM9Write32(w.a, w.v);
        }
        np = 0;
    }
    // a must be valid (checked with ok() before the first write)
    void w32(u32 a, u32 v)
    {
        if (__builtin_expect(logOnly, 0)) { if (n < 64) log[n++] = {a, v, 4}; else bad = true; return; }
        u8* p = P(a);
        if (R32(p) == v) return;
        if ((a & c->DTCMMask) == c->DTCMBase) memcpy(p, &v, 4);   // DTCM never holds JIT code
        else c->NDS.ARM9Write32(a, v);                            // main RAM: JIT invalidation
    }
};

// ---- check mode -----------------------------------------------------------------------------
struct Expect
{
    u32 R[16]; u32 CPSR; u32 IRQ[3]; u32 SVC[3]; bool banks = false;
    u32 retPc = 0; u32 cur = 0;     // cur: wake check also requires this current thread
};
struct Pending
{
    int kind = 0;
    Expect e;
    std::vector<Wr> log;
    std::vector<u8> ram, dtcm;
    std::vector<melonDS::ARMv5::Idle2Access> acc;
    u64 t0 = 0, steps = 0;
    bool irq = false;
} g_P;

__attribute__((noinline, cold)) void ArmCheck(melonDS::ARMv5* c, int kind, const Mem& m, const Expect& e)
{
    melonDS::NDS& nds = c->NDS;
    g_P.kind = kind; g_P.e = e; g_P.log.assign(m.log, m.log + m.n); g_P.irq = false; g_P.steps = 0;
    g_P.ram.assign(nds.MainRAM, nds.MainRAM + nds.MainRAMMask + 1);
    g_P.dtcm.assign(c->DTCM, c->DTCM + DTCMPhysicalSize);
    g_P.acc.clear();
    c->Idle2Log = &g_P.acc;
    g_P.t0 = nds.ARM9Timestamp + c->Cycles;
    CheckPending = true;
}

void GuestFallback(melonDS::ARM* cpu)
{
    u32 icode = ((cpu->CurInstr >> 4) & 0xF) | ((cpu->CurInstr >> 16) & 0xFF0);
    ARMInterpreter::ARMInstrTable[icode](cpu);
}

u32 Flags(u32 a, u32 b)   // NZCV of cmp a, b
{
    u32 r = a - b;
    return (r & 0x80000000) | ((r == 0) << 30) | ((a >= b) << 29) | ((((a ^ b) & (a ^ r)) >> 31) << 28);
}

// ---- 1. spurious OS_WaitIrq wake-up ---------------------------------------------------------
// At the first iteration of OS_IrqHandler_ThreadSwitch's wake loop (IRQ mode, ip = queue head;
// the hook sits inside the loop so IRQs with an empty queue never leave the JIT). Returns false (guest runs) unless the only thread
// in OS_IrqThreadQueue is asleep in OS_WaitIrq with its IRQ flags still clear, it is the thread
// the switch would pick, and the interrupted thread is the one picked once it sleeps again.
// An IRQ that is already pending (or arrives during the guest's round trip) would be taken in
// the woken thread's short IRQs-on window; here it is taken right after the return instead.
bool Wake(melonDS::ARMv5* c, State& s, Mem& m, Expect& e, u32& tpc, u32& tcpsr)
{
    melonDS::NDS& nds = c->NDS;
    if ((c->CPSR & 0x3F) != 0x12) return false;
    const u32 spsr = c->R_IRQ[2];
    if ((spsr & 0x1F) != 0x1F && (spsr & 0x1F) != 0x10) return false;
    const u32 F = c->R[13], svcsp = c->R_SVC[0];
    // host views of every guest object touched (validated once; contiguous)
    Obj q = m.O(kIrqQueue, 8), chk = m.O(kIrqCheck, 4), osi = m.O(kOsi, kInfo + 0x10 - kOsi);
    Obj fr = m.O(F - 12, 40), sv = m.O(svcsp - 24, 24);
    if (!q || !chk || !osi || !fr || !sv) return false;
    // in-order cores: start the independent cold loads together
    __builtin_prefetch(osi.p); __builtin_prefetch(osi.p + 32); __builtin_prefetch(fr.p); __builtin_prefetch(sv.p);
    const u32 t = q.r(0);
    if (!t || c->R[12] != t || q.r(4) != t) return false;
    const u32 info = kInfo - kOsi;
    if (osi.r16(info) || osi.r(info + 0xC) || osi.r(0) || osi.r(4) || osi.r16(0x1E) || osi.r(8) != kInfo + 4) return false;
    const u32 cur = osi.r(info + 4);
    if (!cur || cur == t) return false;
    Obj tp = m.O(t, 0x84), cp = m.O(cur, 0x64);
    if (!tp || !cp) return false;
    __builtin_prefetch(tp.p); __builtin_prefetch(tp.p + 64); __builtin_prefetch(cp.p); __builtin_prefetch(cp.p + 64);
    // t asleep in OS_WaitIrq on this queue, flags not satisfied
    if (tp.r(0x80) || tp.r(0x7C) || tp.r(0x64) != 0 || tp.r(0x78) != kIrqQueue || tp.r(0x40) != kCtxPc || tp.r(0x3C) != kCtxLr)
        return false;
    const u32 tsp = tp.r(0x38);
    Obj ts = m.O(tsp - 12, 44);
    if (!ts) return false;
    if (ts.r(24) != kRetSleep || ts.r(40) != kRetWait || ts.r(32) != kIrqQueue || ts.r(36) != kIrqCheck) return false;
    if (ts.r(28) & chk.r(0)) return false;                        // real wake-up
    tcpsr = tp.r(0);
    tpc = kCtxPc;
    if ((tcpsr & 0xFF) != 0x9F) return false;                     // SYS mode, ARM, IRQs off
    // the switch picks t (first thread that is ready or t), and with t asleep again the first
    // ready thread is cur
    u32 first = 0, firstReady = 0;
    u32 x = osi.r(info + 8);
    for (int i = 0; x && i < 64; i++)
    {
        Obj xp = m.O(x + 0x64, 8);
        if (!xp) return false;
        u32 st = xp.r(0);
        if (st > 0xFFFF) return false;
        if (!first && (st == 1 || x == t)) first = x;
        if (!firstReady && st == 1 && x != t) firstReady = x;
        if (first && firstReady) break;
        x = xp.r(4);
    }
    if (first != t || firstReady != cur) return false;
    // BIOS IRQ frame: [F] = return into the BIOS, [F+4..F+28) = r0-r3, r12, lr  (fr: F-12..F+28)
    const u32 lrb = fr.r(12);
    if (lrb != s.biosOk)
    {
        if ((lrb >> 12) != 0xFFFF0 || (lrb & 0xFFF) > ARM9BIOSSize - 8) return false;
        const u8* bios = nds.GetARM9BIOS().data() + (lrb & 0xFFF);
        if (R32(bios) != kBiosLdm || R32(bios + 4) != kBiosSubs) return false;
        s.biosOk = lrb;
    }
    u32 f[6];
    for (int i = 0; i < 6; i++) f[i] = fr.r(16 + i * 4);
    const u32 tf[6] = {tp.r(4), tp.r(8), tp.r(0xC), tp.r(0x10), tp.r(0x34), kCtxPc};

    // interrupted thread's context, as the IRQ path saves it
    m.W(cp, 0x00, spsr);
    for (int i = 0; i < 4; i++) m.W(cp, 4 + i * 4, f[i]);
    for (int i = 4; i < 12; i++) m.W(cp, 4 + i * 4, c->R[i]);
    m.W(cp, 0x34, f[4]);
    m.W(cp, 0x38, c->R_IRQ[0]);                                  // user/sys r13, r14 (banked out in IRQ mode)
    m.W(cp, 0x3C, c->R_IRQ[1]);
    m.W(cp, 0x40, f[5]);
    m.W(cp, 0x44, svcsp);
    // CP_SaveContext: divider numerator/denominator, sqrt param, DIVCNT&3, SQRTCNT&1
    // (the register values themselves: no lazy-divider materialisation needed for these bits)
    u32 cpx[7];
    nds.A9HLECpContext(cpx);
    for (int i = 0; i < 7; i++) m.W(cp, 0x48 + i * 4, cpx[i]);
    // IRQ stack below the frame: CP_SaveContext's push {r4}, push {r0 = C, r1 = t}
    m.W(fr, 0, c->R[4]);
    m.W(fr, 4, cur);
    m.W(fr, 8, t);
    // the IRQ frame rewritten with t's registers (the BIOS returned into t with them)
    for (int i = 0; i < 6; i++) m.W(fr, 16 + i * 4, tf[i]);
    // t slept again: its context holds r5 = the thread it switched to; its stack holds
    // OS_LoadContext's push {r0 = cur, lr} and CP_RestoreContext's push {r4 = OSThreadInfo}
    m.W(tp, 0x18, cur);
    m.W(ts, 0, kInfo);
    m.W(ts, 4, cur);
    m.W(ts, 8, kRetLoad);
    // OS_LoadContext(cur) pushed {r0-r3, r12, lr = pc} on cur's SVC stack
    for (int i = 0; i < 6; i++) m.W(sv, i * 4, f[i]);
    m.Flush();

    // registers after the guest's return into cur
    for (int i = 0; i < 16; i++) e.R[i] = c->R[i];
    for (int i = 0; i < 4; i++) e.R[i] = f[i];
    e.R[12] = f[4];
    e.R[13] = c->R_IRQ[0];
    e.R[14] = c->R_IRQ[1];
    e.CPSR = spsr;
    e.IRQ[0] = F + 28; e.IRQ[1] = tpc; e.IRQ[2] = tcpsr;
    e.SVC[0] = svcsp; e.SVC[1] = f[5]; e.SVC[2] = spsr;
    e.banks = true;
    e.retPc = (f[5] - 4) & ((spsr & 0x20) ? ~1u : ~3u);
    e.cur = cur;
    return true;
}

void WakeCommit(melonDS::ARMv5* c, const Expect& e)
{
    c->R_SVC[1] = e.SVC[1];
    c->R_SVC[2] = e.SVC[2];
    for (int i = 0; i < 4; i++) c->R[i] = e.R[i];
    c->R[12] = e.R[12];
    c->R[13] = e.IRQ[0];
    c->R[14] = e.SVC[1];
    c->Cycles += kWakeCycles;
    c->JumpTo(e.SVC[1] - 4, true);      // subs pc, lr, #4: CPSR = SPSR_irq, IRQ bank out
    c->R_IRQ[1] = e.IRQ[1];
    c->R_IRQ[2] = e.IRQ[2];
}

// ---- 2. OS_SetIrqFunction(mask, func) ----------------------------------------------------
bool SetIrq(melonDS::ARMv5* c, Mem& m, Expect& e)
{
    u32 mask = c->R[0], fn = c->R[1], sp = c->R[13];
    u32 lr = c->R[14];
    if (!m.ok(sp - 32) || !m.ok(sp - 4) || !m.ok(kIrqTable) || !m.ok(kIrqTable + 127) || !m.ok(kIrqTable2)) return false;
    for (u32 i = 0; i < 32; i++)
    {
        if (!(mask >> i & 1)) continue;
        u32 ent = 0;
        if (i >= 8 && i <= 11) ent = kIrqTable2 + (i - 8) * 12;
        else if (i >= 0x1C) ent = kIrqTable2 + (i - 0x18) * 12;
        else if (i >= 3 && i <= 6) ent = kIrqTable2 + (i + 5) * 12;
        lr = ent;
        if (ent) { m.w32(ent, fn); m.w32(ent + 4, 1); m.w32(ent + 8, 0); }
        else m.w32(kIrqTable + i * 4, fn);
    }
    // push {r4-r10, lr}
    for (int i = 0; i < 7; i++) m.w32(sp - 32 + i * 4, c->R[4 + i]);
    m.w32(sp - 4, c->R[14]);
    if (m.bad) return false;
    for (int i = 0; i < 16; i++) e.R[i] = c->R[i];
    e.R[0] = 0; e.R[1] = fn; e.R[2] = kIrqTable2; e.R[3] = 0; e.R[12] = 0x20; e.R[14] = lr;
    e.CPSR = (c->CPSR & 0x0FFFFFFF) | Flags(0x20, 0x20);
    e.retPc = c->R[14];
    if (e.retPc & 1) e.CPSR |= 0x20;
    return true;
}

// ---- 3. OS_GetIrqFunction(mask) ----------------------------------------------------------
bool GetIrq(melonDS::ARMv5* c, Mem& m, Expect& e, u32& bits)
{
    u32 mask = c->R[0];
    for (int i = 0; i < 16; i++) e.R[i] = c->R[i];
    u32 i = 0;
    while (i < 32 && !(mask >> i & 1)) i++;
    bits = i;
    u32 fl;
    if (i == 32) { e.R[0] = 0; e.R[1] = 32; e.R[2] = kIrqTable + 128; fl = Flags(32, 32); }
    else if (i >= 8 && i <= 11) { e.R[1] = i - 8; e.R[2] = e.R[1] * 12; e.R[0] = m.r32(kIrqTable2 + e.R[2]); fl = Flags(i, 11); }
    else if (i >= 3 && i <= 6) { e.R[1] = i + 5; e.R[2] = e.R[1] * 12; e.R[0] = m.r32(kIrqTable2 + e.R[2]); fl = Flags(i, 6); }
    else { e.R[1] = i; e.R[2] = kIrqTable + 4 * i; e.R[0] = m.r32(e.R[2]); fl = i < 3 ? Flags(i, 3) : Flags(i, 6); }
    if (m.bad) return false;
    e.CPSR = (c->CPSR & 0x0FFFFFFF) | fl;
    e.retPc = c->R[14];
    if (e.retPc & 1) e.CPSR |= 0x20;
    return true;
}

void Return(melonDS::ARMv5* c, const Expect& e, s32 cycles)
{
    for (int i = 0; i < 15; i++) c->R[i] = e.R[i];
    c->CPSR = e.CPSR;
    c->Cycles += cycles;
    c->JumpTo(e.retPc);
}

struct StatsDump
{
    ~StatsDump()
    {
        if (!g_Stats) return;
        for (auto& [k, s] : g_State)
            for (int i = 0; i < 3; i++)
                fprintf(stderr, "A9HLE %s: status=%d calls=%llu native=%llu fallback=%llu checks=%llu check_diffs=%llu irq_during=%llu guest_cyc_avg=%.0f\n",
                        kName[i], s.status, (unsigned long long)s.calls[i], (unsigned long long)s.native[i],
                        (unsigned long long)s.fallback[i], (unsigned long long)s.checks[i], (unsigned long long)s.diffs[i],
                        (unsigned long long)s.irqDuring[i], s.guestN[i] ? (double)s.guestCyc[i] / s.guestN[i] : 0.0);
        for (auto& [k, s] : g_State)
        {
            u64 n = s.native[0] + s.native[1] + s.native[2];
            if (n) fprintf(stderr, "A9HLE: %.1f ns per native call (Run, timer pair %.1f ns subtracted)\n", (double)s.ns / n - TimerNs(), TimerNs());
        }
        for (auto& [k, s] : g_State)
            if (s.checks[2]) fprintf(stderr, "A9HLE getirqfn: avg lowest set bit %.2f\n", (double)s.getBits / s.checks[2]);
    }
} g_StatsDump;
}

int Deps(u32 addr, const Range*& r)
{
    if (addr == kWake) { r = kCode; return kNumCode; }
    r = &kCode[3];              // OS_SetIrqFunction / OS_GetIrqFunction
    return 1;
}

int IsHook(melonDS::NDS& nds, u32 addr, u32 instr)
{
    int k = addr == kWake && instr == kWakeInstr ? 0 : addr == kSet && instr == kSetInstr ? 1 : addr == kGet && instr == kGetInstr ? 2 : -1;
    if (k < 0) return 0;
    State& s = Get(&nds.ARM9);
    if (s.status != 1 || !(s.mask >> k & 1)) return 0;
    return CodeIntact(&nds.ARM9, s, k) ? 1 : 2;
}

bool Run(melonDS::ARM* cpu, bool jit)
{
    if (cpu->Num != 0 || (cpu->CPSR & 0x20)) return false;
    auto* c = (melonDS::ARMv5*)cpu;
    const u32 pc = cpu->R[15] - 8;
    int k = pc == kWake && cpu->CurInstr == kWakeInstr ? 0 : pc == kSet && cpu->CurInstr == kSetInstr ? 1
          : pc == kGet && cpu->CurInstr == kGetInstr ? 2 : -1;
    if (k < 0) return false;
    State& s = Get(c);
    if (s.status != 1 || !(s.mask >> k & 1)) return false;
    s.calls[k]++;
    const u64 t0 = __builtin_expect(g_Stats, 0) ? Now() : 0;
    Mem m(c, g_Check || g_Dry);
    Expect e;
    u32 tpc = 0, tcpsr = 0, bits = 0;
    // JIT: the hook block was compiled after IsHook verified the code, and it depends on every
    // checked byte (ARMJIT adds the Deps() ranges to the block), so a write there invalidates it.
    // Interpreter: compare per call.
    bool ok = !CheckPending && (jit || CodeIntact(c, s, k));
    if (ok)
        ok = k == 0 ? Wake(c, s, m, e, tpc, tcpsr) : k == 1 ? SetIrq(c, m, e) : GetIrq(c, m, e, bits);
    if (ok && g_Check)
    {
        if (k == 2) s.getBits += bits;
        ArmCheck(c, k, m, e);
        s.checks[k]++;
        ok = false;
    }
    if (__builtin_expect(g_Dry, 0) && ok) { s.checks[k]++; ok = false; }
    if (!ok)
    {
        s.fallback[k] += !g_Check;
        GuestFallback(cpu);
        return true;
    }
    s.native[k]++;
    if (k == 0) WakeCommit(c, e);
    else Return(c, e, k == 1 ? kSetCycles : kGetCyclesBase + kGetCyclesPerBit * (s32)bits);
    if (__builtin_expect(g_Stats, 0)) s.ns += Now() - t0;
    return true;
}

void CheckAt(melonDS::ARM* cpu, u32 pc)
{
    auto* c = (melonDS::ARMv5*)cpu;
    if (pc == c->ExceptionBase + 0x18) g_P.irq = true;
    if (++g_P.steps > 2000000)
    {
        fprintf(stderr, "A9HLE CHECK %s: guest never returned to %08x\n", kName[g_P.kind], g_P.e.retPc);
        CheckPending = false;
        c->Idle2Log = nullptr;
        g_State[&c->NDS].diffs[g_P.kind]++;
        return;
    }
    const Expect& e = g_P.e;
    if (pc != (e.retPc & ~1u) || cpu->CPSR != e.CPSR) return;
    melonDS::NDS& nds = c->NDS;
    if (g_P.kind == 0 && R32(&nds.MainRAM[(kInfo + 4) & nds.MainRAMMask]) != e.cur) return;
    CheckPending = false;
    c->Idle2Log = nullptr;
    State& s = g_State[&nds];
    const int k = g_P.kind;
    if (g_P.irq && k != 0)
    {
        s.irqDuring[k]++;   // an IRQ handler ran inside the function: its writes would show as diffs
        return;
    }
    s.guestCyc[k] += nds.ARM9Timestamp + c->Cycles - g_P.t0;
    s.guestN[k]++;
    // expected bytes: native log over the snapshot; compared at every byte either side wrote
    std::unordered_map<u32, u8> exp;
    Mem m(c, false);
    auto old = [&](u32 a) -> u8 {
        if ((a & c->DTCMMask) == c->DTCMBase) return g_P.dtcm[a & (DTCMPhysicalSize - 1)];
        return g_P.ram[a & nds.MainRAMMask];
    };
    for (auto& w : g_P.log)
        for (u32 i = 0; i < w.sz; i++) exp[w.a + i] = (u8)(w.v >> (8 * i));
    std::unordered_set<u32> addrs;
    for (auto& [a, v] : exp) addrs.insert(a);
    for (auto& a : g_P.acc)
        if (a.Write)
            for (u32 i = 0; i < a.Size; i++)
                if (m.P(a.Addr + i)) addrs.insert(a.Addr + i);
    int nd = 0;
    char buf[1024]; int bl = 0;
    for (u32 a : addrs)
    {
        u8* p = m.P(a);
        if (!p) continue;
        auto it = exp.find(a);
        u8 want = it != exp.end() ? it->second : old(a);
        if (*p != want)
        {
            if (nd < 12) bl += snprintf(buf + bl, sizeof(buf) - bl, " %08x guest %02x native %02x;", a, *p, want);
            nd++;
        }
    }
    for (int i = 0; i < 15; i++)
        if (cpu->R[i] != e.R[i])
        {
            if (nd < 12) bl += snprintf(buf + bl, sizeof(buf) - bl, " r%d guest %08x native %08x;", i, cpu->R[i], e.R[i]);
            nd++;
        }
    if (e.banks)
    {
        const u32 gi[3] = {c->R_IRQ[0], c->R_IRQ[1], c->R_IRQ[2]}, gs[3] = {c->R_SVC[0], c->R_SVC[1], c->R_SVC[2]};
        for (int i = 0; i < 3; i++)
        {
            if (gi[i] != e.IRQ[i]) { if (nd < 12) bl += snprintf(buf + bl, sizeof(buf) - bl, " irq[%d] guest %08x native %08x;", i, gi[i], e.IRQ[i]); nd++; }
            if (gs[i] != e.SVC[i]) { if (nd < 12) bl += snprintf(buf + bl, sizeof(buf) - bl, " svc[%d] guest %08x native %08x;", i, gs[i], e.SVC[i]); nd++; }
        }
    }
    if (nd && g_P.irq)
        nd = 0;   // an IRQ arrived inside the guest round trip: the native path takes it after; counted in irq_during
    if (nd)
    {
        if (s.diffs[k] < 10) fprintf(stderr, "A9HLE CHECK %s (irq_in_window=%d) %d diffs:%s\n", kName[k], (int)g_P.irq, nd, buf);
        s.diffs[k]++;
    }
    if (g_P.irq) s.irqDuring[k]++;
}
}
#endif
