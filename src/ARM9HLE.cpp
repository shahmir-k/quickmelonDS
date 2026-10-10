// LITEV_A9HLE: see ARM9HLE.h.
#ifdef LITEV_A9HLE
#include "ARM9HLE.h"
#include "ARM.h"
#include "NDS.h"
#include "ARMInterpreter.h"
#include "NDSCart.h"
#include "Savestate.h"
#ifdef LITEV_GX_BULK
#include "GPU.h"
#include "DMA_Timings.h"
#endif
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
#ifdef LITEV_A9HLE_GXCHECK
std::vector<u32>* GxTap = nullptr;
bool GxOtherSeen = false;
#endif
namespace
{
// ---- Pokemon Black/White (USA/EU), NitroSDK ARM9 OS --------------------------------------
constexpr u32 kWake = 0x01FF8160, kWakeInstr = 0xE58C2064;    // OS_IrqHandler_ThreadSwitch wake loop (queue not empty): str r2, [ip, #0x64]
constexpr u32 kSet = 0x0208478C, kSetInstr = 0xE92D47F0;      // OS_SetIrqFunction: push {r4-r10, lr}
constexpr u32 kGet = 0x02084830, kGetInstr = 0xE59F207C;      // OS_GetIrqFunction: ldr r2, =OS_IRQTable
constexpr u32 kGx = 0x02082864, kGxInstr = 0xE92D40F8;        // MIi_FIFOCallback: push {r3-r7, lr}

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
    // native HBlank IRQ (3.): the rest of the IRQ path (OS_IrqHandler is the first range)
    {0x02005204, 0x0200520C},   // OS_IRQTable[HBlank] stub (Thumb, literal)
    {0x02005610, 0x02005624},   // -> HBlank callback with its object (Thumb, literals)
    {0x02030D40, 0x02030DC0},   // HBlank callback (callback list walk)
    {0x02085B38, 0x02085B48},   // OS idle thread loop
    {0x020882E8, 0x020882F4},   // OS_Halt
};
constexpr int kNumWake = 12;    // ranges of 1. and 2. (signature kSig); the rest: 3. (kIrqSig)
constexpr u64 kSig = 0xc03b33186e018603ull;
constexpr u64 kIrqSig = 0xba271beda6c559d8ull;
// 5.: MIi_FIFOCallback with its literal pool (MIi_GXDmaParams, MIi_DMACallback, GXFIFO, DMA control)
constexpr Range kGxCode = {0x02082864, 0x02082918};
constexpr u64 kGxSig = 0x907c24457e1359d8ull;
constexpr u32 kGxParams = 0x02150E8C;       // MIi_GXDmaParams {busy, dmaNo, src, length, callback, arg, ...}
constexpr u32 kGxChunk = 0x1D8;             // bytes per DMA (118 words)
constexpr u32 kIrqHandler = 0x01FF80F0;     // OS_IrqHandler (BIOS jumps to [DTCM+0x3FFC])
constexpr u32 kHbFn = 0x02005205;           // OS_IRQTable[1] (HBlank)
constexpr u32 kHbObjPtr = 0x020AA1B4;       // callback object = [[kHbObjPtr + 0x10] + 0x18]
constexpr u32 kHbRet = 0x01FF8148;          // OS_IrqHandler after the table call
constexpr u32 kIdlePc = 0x020882F0, kIdleLr = 0x02085B44;   // halted in OS_Halt, called from the idle loop

// 6.: CARDi_ReadRom CPU-read loop (one 512-byte page per ROM command): poll ROMCTRL, read the
// data word, store it to the page buffer while fewer than 128 words
constexpr u32 kCard = 0x02076EE8, kCardInstr = 0xE5942000, kCardExit = 0x02076F0C;
constexpr u32 kCardLoop[9] = {0xE5942000, 0xE3120502, 0x0A000003, 0xE59B1000, 0xE3530080, 0x37851103, 0x32833001, 0xE3120102, 0x1AFFFFF6};
constexpr Range kCardCode = {kCard, kCardExit};

// ponytail: fixed cycle estimates (guest averages measured in check mode on PW)
constexpr s32 kWakeCycles = 900, kSetCycles = 500, kGetCyclesBase = 27, kGetCyclesPerBit = 17;
// 3.: guest averages in check mode (PW f17000 / f6500): IRQ entry to return, empty queue / with the wake round trip
constexpr s32 kIrqCycles = 158, kIrqWakeCycles = 1032;

constexpr int kKinds = 7;
struct State
{
    int status = 0;                 // 0 unprobed, 1 active, -1 off
    u32 mask = 63;                  // 1 wake, 2 set, 4 get, 8 HBlank IRQ, 16 GX send, 32 card read
    std::vector<u8> code, irqCode, gxCode;
    bool gxOk = false;              // MIi_FIFOCallback matches PW (5.)
    bool cardOk = false;            // CARD read loop matches PW (6.)
    bool irqOk = false;             // IRQ path code matches PW (3.)
    u32 biosRet = 0;                // BIOS IRQ entry verified (once): its return address, else 0
    u64 calls[kKinds] = {}, native[kKinds] = {}, fallback[kKinds] = {}, checks[kKinds] = {}, diffs[kKinds] = {}, irqDuring[kKinds] = {};
    u64 guestCyc[kKinds] = {}, guestN[kKinds] = {}, getBits = 0, gxWords = 0, cardWords = 0;
    u32 biosOk = 0;                 // BIOS IRQ epilogue address verified once
    u64 ns = 0;                     // stats: host time inside Run (native calls)
    // host pointers of the fixed-address OS objects, valid for fkey (DTCM base/mask, ITCM size)
    const u8 *fq = nullptr, *fchk = nullptr, *fosi = nullptr, *fhp = nullptr, *ftb = nullptr, *fop = nullptr;
    u32 fkey[3] = {~0u, ~0u, ~0u};
};
std::unordered_map<const melonDS::NDS*, State> g_State;
bool g_Check = getenv("LITEV_A9HLE_CHECK") && atoi(getenv("LITEV_A9HLE_CHECK"));
bool g_Stats = getenv("LITEV_A9HLE_STATS") && atoi(getenv("LITEV_A9HLE_STATS"));
// cost measurement: compute the native result (nothing written) and run the guest code anyway
bool g_Dry = getenv("LITEV_A9HLE_DRY") && atoi(getenv("LITEV_A9HLE_DRY"));
// same, IRQ path (3.) only
bool g_DryIrq = g_Dry || (getenv("LITEV_A9HLE_DRYIRQ") && atoi(getenv("LITEV_A9HLE_DRYIRQ")));
const char* kName[kKinds] = {"irqwake", "setirqfn", "getirqfn", "hblank", "hblank+wake", "gxsend", "cardread"};
// kind -> LITEV_A9HLE_ONLY / debug.litev.a9hle mask bit
inline u32 Bit(int k) { return k == 5 ? 16 : k >= 6 ? 1u << (k - 1) : 1u << k; }

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

// 0 off, 1 (or unset) all hooks, else the mask of hooks (as LITEV_A9HLE_ONLY)
u32 ReadOn()
{
#if defined(__ANDROID__)
    char b[16] = {0}; int n = __system_property_get("debug.litev.a9hle", b);
    const char* e = n > 0 ? b : nullptr;
#else
    const char* e = getenv("debug.litev.a9hle");
#endif
    return e ? (u32)strtoul(e, nullptr, 0) : 1;
}

// code bytes as the ARM9 sees them (ITCM or main RAM), no timing / side effects
const u8* CodePtr(melonDS::ARMv5* c, u32 a)
{
    if (a < c->ITCMSize) return &c->ITCM[a & (ITCMPhysicalSize - 1)];
    if ((a >> 24) == 0x02) return &c->NDS.MainRAM[a & c->NDS.MainRAMMask];
    return nullptr;
}

void CodeBytes(melonDS::ARMv5* c, std::vector<u8>& out, int from, int to)
{
    out.clear();
    for (int i = from; i < to; i++)
        for (u32 a = kCode[i].a; a < kCode[i].b; a++) { const u8* p = CodePtr(c, a); out.push_back(p ? *p : 0); }
}

u64 Fnv(const std::vector<u8>& v)
{
    u64 h = 0xcbf29ce484222325ull;
    for (u8 b : v) h = (h ^ b) * 0x100000001b3ull;
    return h;
}

bool IrqCodeIntact(melonDS::ARMv5* c, const State& s)
{
    size_t o = 0;
    for (int i = kNumWake; i < kNumCode; i++)
    {
        u32 n = kCode[i].b - kCode[i].a;
        const u8* p = CodePtr(c, kCode[i].a);
        if (!p || memcmp(p, s.irqCode.data() + o, n) != 0) return false;
        o += n;
    }
    return true;
}

// BIOS IRQ entry (FreeBIOS or the original): push {r0-r3, r12, lr}; DTCM base; lr = return;
// ldr pc, [DTCM + 0x3FFC]; then the epilogue. Returns the return address (0: unknown BIOS).
u32 BiosIrqRet(melonDS::NDS& nds)
{
    const u8* b = nds.GetARM9BIOS().data();
    const u32 v = R32(b + 0x18);
    if ((v >> 24) != 0xEA) return 0;
    u32 a = 0x18 + 8 + (u32)(((s32)(v << 8)) >> 6);
    auto w = [&](u32 x) { return x + 4 <= ARM9BIOSSize ? R32(b + x) : 0u; };
    if (w(a) != 0xE92D500F || w(a + 4) != 0xEE190F11) return 0;   // push; mrc p15, 0, r0, c9, c1, 0
    a += 8;
    if (w(a) == 0xE3C000FF) a += 4;                                   // bic r0, r0, #0xFF
    else if (w(a) == 0xE1A00620 && w(a + 4) == 0xE1A00600) a += 8;    // lsr/lsl #12
    else return 0;
    if (w(a) != 0xE2800901) return 0;                                 // add r0, r0, #0x4000
    if (w(a + 4) != 0xE1A0E00F && w(a + 4) != 0xE28FE000) return 0;  // mov lr, pc / add lr, pc, #0
    if (w(a + 8) != 0xE510F004) return 0;                             // ldr pc, [r0, #-4]
    a += 12;
    if (w(a) != kBiosLdm || w(a + 4) != kBiosSubs) return 0;
    return 0xFFFF0000 | a;
}

bool CodeIntact(melonDS::ARMv5* c, const State& s, int k)
{
    size_t o = 0;
    for (int i = 0; i < kNumWake; i++)
    {
        const Range& r = kCode[i];
        if (k != 0 && r.a != 0x0208478C) { o += r.b - r.a; continue; }
        u32 n = r.b - r.a;
        const u8* p = CodePtr(c, r.a);
        // ranges never straddle a mirror boundary
        if (!p || memcmp(p, s.code.data() + o, n) != 0) return false;
        o += n;
    }
    return true;
}

bool GxIntact(melonDS::ARMv5* c, const State& s)
{
    const u8* p = CodePtr(c, kGxCode.a);
    return p && memcmp(p, s.gxCode.data(), kGxCode.b - kGxCode.a) == 0;
}

bool CardIntact(melonDS::ARMv5* c)
{
    const u8* p = CodePtr(c, kCard);
    return p && memcmp(p, kCardLoop, sizeof(kCardLoop)) == 0;
}

State& Get(melonDS::ARMv5* c)
{
    if (c->A9HLEState) return *(State*)c->A9HLEState;
    State& s = g_State[&c->NDS];         // map nodes are stable
    c->A9HLEState = &s;
    if (s.status != 0) return s;
    s.status = -1;
    const u32 on = ReadOn();
    if (!on) return s;
    if (on != 1) s.mask = on;
    if (const char* m = getenv("LITEV_A9HLE_ONLY")) s.mask = (u32)strtoul(m, nullptr, 0);
    std::vector<u8> code;
    CodeBytes(c, code, 0, kNumWake);
    u64 h = Fnv(code);
    if (h != kSig)
    {
        if (g_Stats) fprintf(stderr, "A9HLE: signature %016llx != PW, off\n", (unsigned long long)h);
        return s;
    }
    s.code = std::move(code);
    s.status = 1;
    CodeBytes(c, s.irqCode, kNumWake, kNumCode);
    s.biosRet = BiosIrqRet(c->NDS);
    s.irqOk = Fnv(s.irqCode) == kIrqSig && s.biosRet;
    if (g_Stats) fprintf(stderr, "A9HLE: IRQ path signature %016llx, BIOS IRQ return %08x -> %s\n", (unsigned long long)Fnv(s.irqCode), s.biosRet, s.irqOk ? "on" : "off");
    for (u32 a = kGxCode.a; a < kGxCode.b; a++) { const u8* p = CodePtr(c, a); s.gxCode.push_back(p ? *p : 0); }
    s.gxOk = Fnv(s.gxCode) == kGxSig;
    if (g_Stats) fprintf(stderr, "A9HLE: GX send signature %016llx -> %s\n", (unsigned long long)Fnv(s.gxCode), s.gxOk ? "on" : "off");
    s.cardOk = CardIntact(c);
    if (g_Stats) fprintf(stderr, "A9HLE: card read loop -> %s\n", s.cardOk ? "on" : "off");
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
    Wr log[80];
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
    struct Pw { u8* p; u32 a, v; bool d; } pw[64];
    u32 np = 0;
    __attribute__((always_inline)) void W(const Obj& x, u32 o, u32 v) { pw[np++] = {x.p + o, x.a + o, v, x.dtcm}; }
    // only changed words are written: DTCM directly (never holds JIT code), main RAM through the
    // bus (JIT invalidation); in check mode they are logged instead
    void Flush()
    {
        for (u32 i = 0; i < np; i++)
        {
            const Pw& w = pw[i];
            if (__builtin_expect(logOnly, 0)) { if (n < 80) log[n++] = {w.a, w.v, 4}; else bad = true; continue; }
            if (R32(w.p) == w.v) continue;
            if (w.d) memcpy(w.p, &w.v, 4);
            else c->NDS.ARM9Write32(w.a, w.v);
        }
        np = 0;
    }
    // a must be valid (checked with ok() before the first write)
    void w32(u32 a, u32 v)
    {
        if (__builtin_expect(logOnly, 0)) { if (n < 80) log[n++] = {a, v, 4}; else bad = true; return; }
        u8* p = P(a);
        if (R32(p) == v) return;
        if ((a & c->DTCMMask) == c->DTCMBase) memcpy(p, &v, 4);   // DTCM never holds JIT code
        else c->NDS.ARM9Write32(a, v);                            // main RAM: JIT invalidation
    }
};

// fixed-address objects: validated once per memory map (DTCM/ITCM setting) instead of per call
__attribute__((noinline)) bool Refix(melonDS::ARMv5* c, State& s, Mem& m)
{
    s.fkey[0] = c->DTCMBase; s.fkey[1] = c->DTCMMask; s.fkey[2] = c->ITCMSize;
    Obj q = m.O(kIrqQueue, 8), chk = m.O(kIrqCheck, 4), osi = m.O(kOsi, kInfo + 0x10 - kOsi);
    Obj hp = m.O(c->DTCMBase + 0x3FFC, 4), tb = m.O(kIrqTable + 4, 4), op = m.O(kHbObjPtr + 0x10, 4);
    s.fq = q.p; s.fchk = chk.p; s.fosi = osi.p; s.fhp = hp.p; s.ftb = tb.p; s.fop = op.p;
    return q && chk && osi;
}
inline bool Fixed(melonDS::ARMv5* c, State& s, Mem& m)
{
    if (__builtin_expect(s.fkey[0] == c->DTCMBase && s.fkey[1] == c->DTCMMask && s.fkey[2] == c->ITCMSize, 1))
        return s.fq && s.fchk && s.fosi;
    return Refix(c, s, m);
}

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
    bool vecSkip = false;   // native IRQ check: the guest's own entry through the IRQ vector
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
// The IRQ context at the wake loop: the interrupted thread's CPSR, the IRQ stack pointer there (F),
// SVC sp, the interrupted thread's sp/lr, the queue head ip, and (native IRQ, 3.) the IRQ frame
// values that are still only queued writes.
struct IrqIn { u32 spsr, F, svcsp, sysSp, sysLr, ip; bool queued; u32 lrb, f[6]; };

bool Wake(melonDS::ARMv5* c, State& s, Mem& m, Expect& e, const IrqIn& in, u32& tpc, u32& tcpsr)
{
    melonDS::NDS& nds = c->NDS;
    const u32 spsr = in.spsr;
    if ((spsr & 0x1F) != 0x1F && (spsr & 0x1F) != 0x10) return false;
    const u32 F = in.F, svcsp = in.svcsp;
    // host views of every guest object touched (validated; contiguous)
    if (!Fixed(c, s, m)) return false;
    Obj fr = m.O(F - 12, 40), sv = m.O(svcsp - 24, 24);
    if (!fr || !sv) return false;
    const u8* osi = s.fosi;
    // in-order cores: start the independent cold loads together
    __builtin_prefetch(osi); __builtin_prefetch(osi + 32); __builtin_prefetch(fr.p); __builtin_prefetch(sv.p);
    const u32 t = R32(s.fq);
    if (!t || in.ip != t || R32(s.fq + 4) != t) return false;
    const u32 info = kInfo - kOsi;
    if (R16(osi + info) || R32(osi + info + 0xC) || R32(osi) || R32(osi + 4) || R16(osi + 0x1E) || R32(osi + 8) != kInfo + 4) return false;
    const u32 cur = R32(osi + info + 4);
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
    if (ts.r(28) & R32(s.fchk)) return false;                     // real wake-up
    tcpsr = tp.r(0);
    tpc = kCtxPc;
    if ((tcpsr & 0xFF) != 0x9F) return false;                     // SYS mode, ARM, IRQs off
    // the switch picks t (first thread that is ready or t), and with t asleep again the first
    // ready thread is cur
    u32 first = 0, firstReady = 0;
    u32 x = R32(osi + info + 8);
    const u8* ram = nds.MainRAM;
    const u32 rmask = nds.MainRAMMask;
    for (int i = 0; x && i < 64; i++)
    {
        // thread structs: main RAM, not under DTCM, no wrap (+0x64 state, +0x68 next)
        if ((x >> 24) != 0x02 || ((x + 0x64) & c->DTCMMask) == c->DTCMBase || ((x + 0x6B) & c->DTCMMask) == c->DTCMBase
            || ((x + 0x64) & rmask) + 8 > rmask + 1 || (x & 3)) return false;
        const u8* xp = ram + ((x + 0x64) & rmask);
        u32 st = R32(xp);
        if (st > 0xFFFF) return false;
        if (!first && (st == 1 || x == t)) first = x;
        if (!firstReady && st == 1 && x != t) firstReady = x;
        if (first && firstReady) break;
        x = R32(xp + 4);
    }
    if (first != t || firstReady != cur) return false;
    // BIOS IRQ frame: [F] = return into the BIOS, [F+4..F+28) = r0-r3, r12, lr  (fr: F-12..F+28)
    const u32 lrb = in.queued ? in.lrb : fr.r(12);
    if (lrb != s.biosOk && !in.queued)
    {
        if ((lrb >> 12) != 0xFFFF0 || (lrb & 0xFFF) > ARM9BIOSSize - 8) return false;
        const u8* bios = nds.GetARM9BIOS().data() + (lrb & 0xFFF);
        if (R32(bios) != kBiosLdm || R32(bios + 4) != kBiosSubs) return false;
        s.biosOk = lrb;
    }
    u32 f[6];
    for (int i = 0; i < 6; i++) f[i] = in.queued ? in.f[i] : fr.r(16 + i * 4);
    const u32 tf[6] = {tp.r(4), tp.r(8), tp.r(0xC), tp.r(0x10), tp.r(0x34), kCtxPc};

    // interrupted thread's context, as the IRQ path saves it
    m.W(cp, 0x00, spsr);
    for (int i = 0; i < 4; i++) m.W(cp, 4 + i * 4, f[i]);
    for (int i = 4; i < 12; i++) m.W(cp, 4 + i * 4, c->R[i]);
    m.W(cp, 0x34, f[4]);
    m.W(cp, 0x38, in.sysSp);                                     // user/sys r13, r14 (banked out in IRQ mode)
    m.W(cp, 0x3C, in.sysLr);
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
    e.R[13] = in.sysSp;
    e.R[14] = in.sysLr;
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
__attribute__((noinline)) bool SetIrq(melonDS::ARMv5* c, Mem& m, Expect& e)
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
__attribute__((noinline)) bool GetIrq(melonDS::ARMv5* c, Mem& m, Expect& e, u32& bits)
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

// ---- 3. HBlank IRQ ----------------------------------------------------------------------------
// At IRQ delivery (IRQs on, not in IRQ mode). The guest path: BIOS push {r0-r3, r12, lr} on the
// IRQ stack, OS_IrqHandler push {lr}, IF acknowledge of the lowest pending bit (HBlank), table
// call -> Thumb stub -> HBlank callback push {r4-r6, lr}; with an empty callback list it stores
// list head and 0 into its object; back in OS_IrqHandler either the queue is empty (no
// reschedule pending -> return) or the wake loop of 1. runs; BIOS pops and returns.
bool IrqNative(melonDS::ARMv5* c, State& s, Mem& m, Expect& e, bool halted, int& kind, u32& tpc, u32& tcpsr)
{
    melonDS::NDS& nds = c->NDS;
    if (((nds.IE[0] & nds.IF[0]) & 3) != 2) return false;     // OS_IrqHandler takes the lowest bit
    const u32 cpsr = c->CPSR, mode = cpsr & 0x1F;
    if ((cpsr & 0x80) || (mode != 0x1F && mode != 0x10 && mode != 0x13)) return false;
    const bool thumb = cpsr & 0x20;
    // halted in OS_Halt (mcr wait-for-interrupt), called from the idle loop: the guest returns
    // there and halts again with the same registers
    if (halted && (thumb || c->R[15] - 4 != kIdlePc || c->R[0] != 0 || c->R[14] != kIdleLr)) return false;
    const u32 sp = c->R_IRQ[0];
    if (!Fixed(c, s, m) || !s.fhp || !s.ftb || !s.fop) return false;
    Obj fs = m.O(sp - 44, 44);
    if (!fs) return false;
    __builtin_prefetch(fs.p); __builtin_prefetch(fs.p + 40);
    if (R32(s.fhp) != kIrqHandler || R32(s.ftb) != kHbFn) return false;
    Obj o2 = m.O(R32(s.fop) + 0x18, 4);
    if (!o2) return false;
    const u32 obj = o2.r(0);
    Obj ob = m.O(obj, 0x34);
    if (!ob) return false;
    const u32 lrIrq = c->R[15] + (thumb ? 2 : 0);
    const u32 f[6] = {c->R[0], c->R[1], c->R[2], c->R[3], c->R[12], lrIrq};
    const u32 head = R32(s.fq);
    // stack: [sp-44] callback push {r4, r5, r6, lr}, [sp-28] OS_IrqHandler push {lr = BIOS return},
    // [sp-24] BIOS push {r0-r3, r12, lr}; with a wake (head != 0) Wake writes [sp-40, sp-28) and
    // [sp-24, sp) again
    m.W(fs, 0, c->R[4]);
    m.W(fs, 16, s.biosRet);
    if (!head)
    {
        m.W(fs, 4, c->R[5]);
        m.W(fs, 8, c->R[6]);
        m.W(fs, 12, kHbRet);
        for (int i = 0; i < 6; i++) m.W(fs, 20 + i * 4, f[i]);
    }
    if (ob.r(0x2C) == 0)
    {
        const u32 l = ob.r(0x10);
        if (l != obj + 8) return false;                 // HBlank callbacks queued: guest
        m.W(ob, 0x30, l);
        m.W(ob, 0x1C, 0);
    }
    for (int i = 0; i < 16; i++) e.R[i] = c->R[i];
    e.CPSR = cpsr;
    e.retPc = (lrIrq - 4) | (thumb ? 1 : 0);
    e.IRQ[0] = sp; e.IRQ[1] = lrIrq; e.IRQ[2] = cpsr;
    e.SVC[0] = c->R_SVC[0]; e.SVC[1] = c->R_SVC[1]; e.SVC[2] = c->R_SVC[2];
    e.banks = true;
    e.cur = 0;
    if (!head)
    {
        if (R16(s.fosi + (kInfo - kOsi))) return false; // reschedule pending: thread switch
        kind = 3;
        return true;
    }
    IrqIn in{cpsr, sp - 28, c->R_SVC[0], c->R[13], c->R[14], head, true, s.biosRet, {f[0], f[1], f[2], f[3], f[4], f[5]}};
    if (!Wake(c, s, m, e, in, tpc, tcpsr)) return false;
    kind = 4;
    return true;
}

#ifdef LITEV_GX_BULK
// ---- 5. MI_SendGXCommandAsync: the display list at once ----------------------------------------
// MIi_FIFOCallback sends the list in 118-word immediate DMAs to GXFIFO, the next one from the
// GXFIFO "less than half full" IRQ. Under LITEV_GX_BULK each DMA finds the FIFO empty and runs
// in bulk, so the FIFO is empty again at once and the IRQ that follows only starts the next DMA:
// ~2 IRQs per list, 100-200 a frame in PW's 3D scenes. At MIi_FIFOCallback's entry, with the
// FIFO empty: every chunk but the last goes through GPU3D::BulkWords now (what those DMAs do)
// and MIi_GXDmaParams.src/length move past them; the guest code then sends the last chunk
// (DMA with IRQ -> MIi_DMACallback) as before. Category B: the elided IRQs take a fixed estimate
// (the DMA cycles themselves are the DMA's own burst timing).
// ponytail: fixed estimate per elided GXFIFO IRQ (BIOS, OS_IrqHandler, MIi_FIFOCallback, DMA
// setup): check mode measures ~850 guest cycles per chunk on 7 PW segments, ~488 of them the DMA
constexpr s32 kGxIrqCycles = 360;

// the words the guest would send before its last chunk (n), from src; false: guest path
bool GxPlan(melonDS::ARMv5* c, u32& src, u32& len, u32& n)
{
    melonDS::NDS& nds = c->NDS;
    if ((kGxParams & c->DTCMMask) == c->DTCMBase || ((kGxParams + 0xF) & c->DTCMMask) == c->DTCMBase) return false;
    const u8* p = &nds.MainRAM[kGxParams & nds.MainRAMMask];
    len = R32(p + 0xC); src = R32(p + 8);
    const u32 dma = R32(p + 4);
    // DMA reads the bus (main RAM, never TCM), like the guest's
    if (len <= kGxChunk || len > 0x100000 || (len & 3) || (src & 3) || dma > 3 || (src >> 24) != 0x02 || ((src + len - 1) >> 24) != 0x02)
        return false;
    if (nds.A9HLEDmaCnt(dma) & 0x80000000) return false;
    melonDS::GPU3D& gx = nds.GPU.GPU3D;
    if (!gx.GeometryEnabled || !gx.BulkReady()) return false;
    n = ((len - 1) / kGxChunk) * (kGxChunk / 4);
    return true;
}

// sends them (stops early if the FIFO stops being empty: a SWAP_BUFFERS); returns the count
u32 GxSend(melonDS::ARMv5* c, u32 src, u32 len, u32 n)
{
    melonDS::NDS& nds = c->NDS;
    melonDS::GPU3D& gx = nds.GPU.GPU3D;
    const u8* ram = nds.MainRAM;
    const u32 mask = nds.MainRAMMask, per = kGxChunk / 4;
    // the bulk DMA loop's timing (DMA::Run9): burst table restarted at each DMA
    const u8* bt = (nds.ARM9MemTimings[0x1000][6] == 2) ? DMATiming::MRAMRead32Bursts[0].data() : DMATiming::MRAMRead32Bursts[1].data();
    u32 done = 0, cyc = 0, bc = 0;
    for (u32 o = 0; o < 512; o += 64) __builtin_prefetch(ram + ((src + o) & mask));
#ifdef LITEV_GX_SEND_DIRECT
    // The list read in place (no copy) when it doesn't wrap the RAM mirror, and the DMA cycles
    // from a per-chunk prefix sum of the same burst table (the count restarts at every chunk).
    if ((src & mask) + n * 4 <= mask + 1)
    {
        const u32* w = (const u32*)(ram + (src & mask));
        static u32 cum[2][kGxChunk / 4 + 1];
        static bool cumOk[2];
        const int t = bt == DMATiming::MRAMRead32Bursts[0].data() ? 0 : 1;
        if (!cumOk[t])
        {
            for (u32 i = 0, b = 0, c2 = 0; i < per; i++) { if (bt[b] == 0) b = 0; c2 += bt[b++]; cum[t][i + 1] = c2; }
            cumOk[t] = true;
        }
        while (done < n && gx.BulkReady())
        {
            const u32 m = n - done < 64 ? n - done : 64;
            const char* pf = (const char*)(w + done) + 512;
            if (pf + 256 <= (const char*)(ram + mask + 1))
                for (u32 o = 0; o < 256; o += 64) __builtin_prefetch(pf + o);
            gx.BulkWords(w + done, m);
            done += m;
        }
        cyc = (done / per) * cum[t][per] + cum[t][done % per];
    }
    else
#endif
    while (done < n && gx.BulkReady())
    {
        u32 w[64];
        const u32 m = n - done < 64 ? n - done : 64;
        for (u32 i = 0; i < m; i++)
        {
            if ((done + i) % per == 0 || bt[bc] == 0) bc = 0;
            cyc += bt[bc++];
            w[i] = R32(ram + ((src + (done + i) * 4) & mask));
        }
        __builtin_prefetch(ram + ((src + (done + m) * 4 + 512) & mask));
        gx.BulkWords(w, m);
        done += m;
    }
    nds.ARM9Write32(kGxParams + 8, src + done * 4);
    nds.ARM9Write32(kGxParams + 0xC, len - done * 4);
    c->Cycles += (s32)(cyc << nds.ARM9ClockShift) + kGxIrqCycles * (s32)(done / per);
    return done;
}
#endif

#ifdef LITEV_A9HLE_GXCHECK
// check mode for 5. (interpreter, env LITEV_A9HLE_CHECK): the guest sends the list; every word
// reaching GXFIFO (and any other geometry command write) is recorded; when the guest's
// MIi_FIFOCallback reaches the point the native path would leave, the words, src and length
// must be what the native path computed.
std::vector<u32> g_GxTapBuf;
struct GxPending { bool on = false; u32 src = 0, len = 0, n = 0; u64 t0 = 0; std::vector<u32> w; } g_Gx;
u64 g_GxDmaCyc = 0;     // the DMA part of the guest cycles (same burst timing as GxSend)
void GxCheckAt(State& s, melonDS::ARMv5* c)
{
    melonDS::NDS& nds = c->NDS;
    if (!g_Gx.on) return;
    const u8* p = &nds.MainRAM[kGxParams & nds.MainRAMMask];
    const u32 src = R32(p + 8), len = R32(p + 0xC), want = g_Gx.len - g_Gx.n * 4;
    if (len > want) return;     // guest not there yet
    g_Gx.on = false;
    GxTap = nullptr;
    bool bad = len != want || src != g_Gx.src + g_Gx.n * 4 || g_GxTapBuf != g_Gx.w;
    if (GxOtherSeen) bad = true;
    if (bad)
    {
        if (s.diffs[5] < 10)
            fprintf(stderr, "A9HLE CHECK gxsend: src %08x len %x (native %08x %x), words guest %zu native %zu%s\n", src, len,
                    g_Gx.src + g_Gx.n * 4, want, g_GxTapBuf.size(), g_Gx.w.size(), GxOtherSeen ? ", other GX writes" : "");
        s.diffs[5]++;
    }
    GxOtherSeen = false;
    // resolved checks; guest cycles per elided chunk (for kGxIrqCycles: minus the DMA's own)
    s.guestN[5] += g_Gx.n / (kGxChunk / 4);
    s.guestCyc[5] += nds.ARM9Timestamp + c->Cycles - g_Gx.t0;
    const u8* bt = (nds.ARM9MemTimings[0x1000][6] == 2) ? DMATiming::MRAMRead32Bursts[0].data() : DMATiming::MRAMRead32Bursts[1].data();
    for (u32 i = 0, bc = 0; i < g_Gx.n; i++) { if (i % (kGxChunk / 4) == 0 || bt[bc] == 0) bc = 0; g_GxDmaCyc += (u64)bt[bc++] << nds.ARM9ClockShift; }
    s.irqDuring[5]++;
}
#endif


// ---- 6. CARD ROM CPU read: the page loop at once ------------------------------------------------
// CARDi_ReadRom reads with the CPU: per 512-byte page it starts a ROM command, then loops
//   loop: ldr r2, [ROMCTRL]; tst r2, #DATA_READY; beq 1f; ldr r1, [CARD_DATA]; cmp r3, #128;
//         strlo r1, [r5, r3, lsl #2]; addlo r3, r3, #1; 1: tst r2, #BUSY; bne loop
// (~8 guest instructions and two IO reads a word, 10-25% of PW's ARM9 guest instructions:
// it streams map/graphics data every frame). Native: the same ROMCTRL / ROMData reads in the
// same order (the same cart functions the IO reads call; under LITEV_CART_SYNC every word but
// the command's first is ready on the next poll, nothing is scheduled in between), the same
// stores, leaving the registers and flags the guest has at the loop exit, or at the loop head
// when the next word isn't ready yet (the guest then polls as before). Category B: a fixed
// cycle estimate per word; an IRQ that would arrive inside the loop is taken after it.
// ponytail: fixed estimate (guest average per word, check mode on PW)
constexpr s32 kCardCycWord = 31;

struct CardIo { u32 r1, r2, r3, flags; bool exit; u32 words; };
// the registers / buffer the native loop handles
bool CardArgsOk(melonDS::ARMv5* c, u32 r3)
{
    melonDS::NDS& nds = c->NDS;
    if (c->R[4] != 0x040001A4 || c->R[11] != 0x04100010) return false;
    // IO reads pass the protection unit (else the guest's data abort)
    if (!(c->PU_Map[0x040001A4 >> 12] & 0x01) || !(c->PU_Map[0x04100010 >> 12] & 0x01)) return false;
    if (r3 >= 128) return true;
    // stores [r5 + r3*4, r5 + 512): main RAM, not under a TCM, writable, one host block
    // (a DTCM of >= 16 KB overlapping a range of <= 512 bytes holds one of its ends)
    const u32 r5 = c->R[5], a = r5 + r3 * 4, b = r5 + 511;
    return !((r5 & 3) || (a >> 24) != 0x02 || (b >> 24) != 0x02 || a < c->ITCMSize || ((a & c->DTCMMask) == c->DTCMBase)
             || ((b & c->DTCMMask) == c->DTCMBase) || ((a & nds.MainRAMMask) > (b & nds.MainRAMMask))
             || !(c->PU_Map[a >> 12] & 0x02) || !(c->PU_Map[b >> 12] & 0x02));
}

// io in: r1, r2, r3 and NZCV at the loop head; log: check mode (stores recorded, memory
// untouched). false: nothing done (the guest runs the iteration).
bool CardCore(melonDS::ARMv5* c, CardIo& io, std::vector<Wr>* log)
{
    melonDS::NDS& nds = c->NDS;
    if (!CardArgsOk(c, io.r3)) return false;
    const u32 r5 = c->R[5];
    u32 r1 = io.r1, r2 = io.r2, r3 = io.r3;
    u8* buf = nds.MainRAM;
    melonDS::NDSCart::NDSCartSlot& slot = *nds.NDSCartSlots[0];
    u32 n = 0;
    bool cmpd = false, exit = false;
    for (;;)
    {
        const u32 cnt = slot.ReadROMCnt(0);
        if (!(cnt & 0x800000))
        {
            if (cnt & 0x80000000) break;        // not ready: the guest polls from the loop head
            r2 = cnt; exit = true; break;       // done
        }
        r2 = cnt;
        r1 = slot.ReadROMData(0);
        cmpd = true; n++;
        if (r3 < 128)
        {
            const u32 a = r5 + r3 * 4;
            u8* p = buf + (a & nds.MainRAMMask);
            if (__builtin_expect(log != nullptr, 0)) log->push_back({a, r1, 4});
            else if (R32(p) != r1)
            {
                nds.JIT.CheckAndInvalidate<0, melonDS::ARMJIT_Memory::memregion_MainRAM>(a);
                memcpy(p, &r1, 4);
            }
            r3++;
        }
        if (!(cnt & 0x80000000)) { exit = true; break; }
    }
    if (!n && !exit) return false;
    // flags of the last tst r2, #BUSY (C from the rotated immediate); V: cmp r3, #128 clears it
    io.flags = (r2 & 0x80000000) | ((r2 & 0x80000000) ? 0 : 0x40000000) | 0x20000000 | (cmpd ? 0 : (io.flags & 0x10000000));
    io.r1 = r1; io.r2 = r2; io.r3 = r3; io.exit = exit; io.words = n;
    return true;
}

// check mode (interpreter): the guest runs the loop; at its exit the native loop reruns from the
// entry state (the cart's ROM-read state restored) and must produce the same registers, flags,
// stores, cart state and transfer IRQ
struct CardPending
{
    bool on = false;
    u32 r[16] = {}, cpsr = 0, ifr = 0;
    std::vector<u8> cart, buf;
    u64 t0 = 0, steps = 0;
} g_Card;

__attribute__((noinline, cold)) std::vector<u8> CartState(melonDS::NDS& nds)
{
    melonDS::Savestate st(1 << 16);
    nds.NDSCartSlots[0]->HleRomState(&st);
    st.Finish();
    const u8* b = (const u8*)st.Buffer();
    return std::vector<u8>(b, b + st.Length());
}
__attribute__((noinline, cold)) void CartLoad(melonDS::NDS& nds, std::vector<u8>& v)
{
    melonDS::Savestate st(v.data(), (u32)v.size(), false);
    nds.NDSCartSlots[0]->HleRomState(&st);
}

__attribute__((noinline, cold)) void CardCheckStart(melonDS::ARMv5* c, State& s)
{
    melonDS::NDS& nds = c->NDS;
    g_Card.on = true;
    for (int i = 0; i < 16; i++) g_Card.r[i] = c->R[i];
    g_Card.cpsr = c->CPSR; g_Card.ifr = nds.IF[0];
    g_Card.cart = CartState(nds);
    g_Card.buf.assign(512, 0);
    for (u32 i = 0; i < 512; i++) g_Card.buf[i] = nds.ARM9Read8(c->R[5] + i);
    g_Card.t0 = nds.ARM9Timestamp + c->Cycles; g_Card.steps = 0;
    CheckPending = true;
    s.checks[6]++;
}

__attribute__((noinline, cold)) void CardCheckAt(melonDS::ARMv5* c, u32 pc)
{
    melonDS::NDS& nds = c->NDS;
    State& s = g_State[&nds];
    if (++g_Card.steps > 2000000) { fprintf(stderr, "A9HLE CHECK cardread: guest never left the loop\n"); g_Card.on = CheckPending = false; s.diffs[6]++; return; }
    if (pc != kCardExit || (c->CPSR & 0x3F) != (g_Card.cpsr & 0x3F)) return;
    g_Card.on = CheckPending = false;
    const u64 gcyc = nds.ARM9Timestamp + c->Cycles - g_Card.t0;
    const u32 g1 = c->R[1], g2 = c->R[2], g3 = c->R[3], gfl = c->CPSR & 0xF0000000, gif = nds.IF[0];
    std::vector<u8> gbuf(512), gcart = CartState(nds);
    for (u32 i = 0; i < 512; i++) gbuf[i] = nds.ARM9Read8(g_Card.r[5] + i);
    // native from the entry state
    CartLoad(nds, g_Card.cart);
    const u32 saveR[16] = {c->R[0], c->R[1], c->R[2], c->R[3], c->R[4], c->R[5], c->R[6], c->R[7], c->R[8], c->R[9], c->R[10], c->R[11], c->R[12], c->R[13], c->R[14], c->R[15]};
    for (int i = 0; i < 16; i++) c->R[i] = g_Card.r[i];
    nds.IF[0] = g_Card.ifr;
    std::vector<Wr> log;
    CardIo io{g_Card.r[1], g_Card.r[2], g_Card.r[3], g_Card.cpsr & 0xF0000000, false, 0};
    bool ran = CardCore(c, io, &log);
    const u32 nif = nds.IF[0];
    std::vector<u8> ncart = CartState(nds);
    for (int i = 0; i < 16; i++) c->R[i] = saveR[i];
    nds.IF[0] = gif;
    nds.UpdateIRQ(0);
    CartLoad(nds, gcart);
    std::vector<u8> nbuf = g_Card.buf;
    for (auto& w : log) for (u32 i = 0; i < 4; i++) nbuf[w.a - g_Card.r[5] + i] = (u8)(w.v >> (8 * i));
    char b[512]; int bl = 0, nd = 0;
    auto d = [&](const char* what, u32 g, u32 n) { if (g != n) { if (nd < 8) bl += snprintf(b + bl, sizeof(b) - bl, " %s guest %08x native %08x;", what, g, n); nd++; } };
    if (!ran || !io.exit) { bl += snprintf(b + bl, sizeof(b) - bl, " native %s;", ran ? "stopped early" : "did nothing"); nd++; }
    else
    {
        d("r1", g1, io.r1); d("r2", g2, io.r2); d("r3", g3, io.r3); d("nzcv", gfl, io.flags);
        d("xfer irq", (gif & ~g_Card.ifr) & (1u << 19), (nif & ~g_Card.ifr) & (1u << 19));
        for (u32 i = 0; i < 512; i++) if (gbuf[i] != nbuf[i]) { d("buf", gbuf[i], nbuf[i]); break; }
        if (gcart != ncart) { bl += snprintf(b + bl, sizeof(b) - bl, " cart state;"); nd++; }
    }
    if (nd)
    {
        if (s.diffs[6] < 10) fprintf(stderr, "A9HLE CHECK cardread %d diffs:%s\n", nd, b);
        s.diffs[6]++;
    }
    if (ran && io.exit && io.words) { s.guestCyc[6] += gcyc; s.guestN[6] += io.words; }
}

struct StatsDump
{
    ~StatsDump()
    {
        if (!g_Stats) return;
        for (auto& [k, s] : g_State)
            for (int i = 0; i < kKinds; i++)
                fprintf(stderr, "A9HLE %s: status=%d calls=%llu native=%llu fallback=%llu checks=%llu check_diffs=%llu irq_during=%llu guest_cyc_avg=%.0f\n",
                        kName[i], s.status, (unsigned long long)s.calls[i], (unsigned long long)s.native[i],
                        (unsigned long long)s.fallback[i], (unsigned long long)s.checks[i], (unsigned long long)s.diffs[i],
                        (unsigned long long)s.irqDuring[i], s.guestN[i] ? (double)s.guestCyc[i] / s.guestN[i] : 0.0);
        for (auto& [k, s] : g_State)
        {
            u64 n = 0;
            for (int i = 0; i < kKinds; i++) n += s.native[i];
            if (s.native[5]) fprintf(stderr, "A9HLE gxsend: %.1f words per native call\n", (double)s.gxWords / s.native[5]);
            if (s.native[6]) fprintf(stderr, "A9HLE cardread: %.1f words per native call\n", (double)s.cardWords / s.native[6]);
            if (n) fprintf(stderr, "A9HLE: %.1f ns per native call (Run, timer pair %.1f ns subtracted)\n", (double)s.ns / n - TimerNs(), TimerNs());
        }
        for (auto& [k, s] : g_State)
            if (s.checks[2]) fprintf(stderr, "A9HLE getirqfn: avg lowest set bit %.2f\n", (double)s.getBits / s.checks[2]);
#ifdef LITEV_A9HLE_GXCHECK
        for (auto& [k, s] : g_State)
            if (s.checks[5]) fprintf(stderr, "A9HLE gxsend check: %llu resolved (irq_during above), %.0f guest cycles per elided chunk, of them DMA %.0f\n",
                                     (unsigned long long)s.irqDuring[5], s.guestN[5] ? (double)s.guestCyc[5] / s.guestN[5] : 0.0,
                                     s.guestN[5] ? (double)g_GxDmaCyc / s.guestN[5] : 0.0);
#endif
    }
} g_StatsDump;
}

void HookCompiled(melonDS::NDS& nds, u32 addr, const void* block)
{
    if (addr != kWake) return;
    melonDS::ARMv5* c = &nds.ARM9;
    State& s = Get(c);
    // the wake hook block depends on all of kCode (Deps): while it lives, the IRQ path is intact
    c->A9HLEGuard = s.status == 1 && s.irqOk && (s.mask & 8) && IrqCodeIntact(c, s) ? block : nullptr;
}

void BlockGone(melonDS::NDS& nds, const void* block)
{
    if (nds.ARM9.A9HLEGuard == block) nds.ARM9.A9HLEGuard = nullptr;
}

bool Irq(melonDS::ARMv5* c, bool halted)
{
    if (CheckPending) return false;
    static bool dryHalted = false;      // dry: the halted call already computed this delivery
    if (__builtin_expect(g_DryIrq, 0) && dryHalted) { dryHalted = false; return false; }
    const bool jit = c->A9HLEGuard != nullptr;
    if (!jit && c->NDS.IsJITEnabled()) return false;
    State& s = Get(c);
    if (!jit && (s.status != 1 || !s.irqOk || !(s.mask & 8) || !IrqCodeIntact(c, s))) return false;
    const u64 t0 = __builtin_expect(g_Stats, 0) ? Now() : 0;
    Mem m(c, g_Check || g_DryIrq);
    Expect e;
    int kind = 3;
    u32 tpc = 0, tcpsr = 0;
    if (!IrqNative(c, s, m, e, halted, kind, tpc, tcpsr)) return false;
    s.calls[kind]++;
    m.Flush();                                          // check / dry: logs only
    if (__builtin_expect(g_Check, 0))
    {
        ArmCheck(c, kind, m, e);
        g_P.vecSkip = true;
        s.checks[kind]++;
        return false;
    }
    if (__builtin_expect(g_DryIrq, 0)) { s.checks[kind]++; dryHalted = halted; return false; }
    s.native[kind]++;
    melonDS::NDS& nds = c->NDS;
    nds.IF[0] &= ~2u;                                   // OS_IrqHandler: str r1, [IF]
    nds.GPU.GPU3D.CheckFIFOIRQ();
    nds.UpdateIRQ(0);
    if (kind == 4) { c->R_SVC[1] = e.SVC[1]; c->R_SVC[2] = e.SVC[2]; }
    c->R_IRQ[1] = e.IRQ[1];
    c->R_IRQ[2] = e.IRQ[2];
    c->Cycles += kind == 4 ? kIrqWakeCycles : kIrqCycles;
    if (__builtin_expect(g_Stats, 0)) s.ns += Now() - t0;
    return true;
}

int Deps(u32 addr, const Range*& r)
{
    if (addr == kCard) { r = &kCardCode; return 1; }
    if (addr == kWake) { r = kCode; return kNumCode; }
    if (addr == kGx) { r = &kGxCode; return 1; }
    r = &kCode[3];              // OS_SetIrqFunction / OS_GetIrqFunction
    return 1;
}

int IsHook(melonDS::NDS& nds, u32 addr, u32 instr)
{
    int k = addr == kWake && instr == kWakeInstr ? 0 : addr == kSet && instr == kSetInstr ? 1 : addr == kGet && instr == kGetInstr ? 2
          : addr == kGx && instr == kGxInstr ? 5 : addr == kCard && instr == kCardInstr ? 6 : -1;
    if (k < 0) return 0;
    State& s = Get(&nds.ARM9);
    if (s.status != 1 || !(s.mask & Bit(k))) return 0;
    if (k == 6) return !s.cardOk ? 0 : CardIntact(&nds.ARM9) ? 1 : 2;
    if (k == 5) return !s.gxOk ? 0 : GxIntact(&nds.ARM9, s) ? 1 : 2;
    return CodeIntact(&nds.ARM9, s, k) ? 1 : 2;
}

namespace
{
// each kind its own function: a call touches only its own code lines (the in-order A55's L1I)
__attribute__((noinline)) bool RunCard(melonDS::ARMv5* c, State& s, bool jit)
{
    if (!s.cardOk) return false;
    bool ok = !CheckPending && (jit || CardIntact(c));
    if (ok && g_Check)
    {
        // only where the native loop makes progress (a word ready, or the transfer done)
        const u32 cnt = c->NDS.NDSCartSlots[0]->ReadROMCnt(0);
        if (((cnt & 0x800000) || !(cnt & 0x80000000)) && CardArgsOk(c, c->R[3])) CardCheckStart(c, s);
        ok = false;
    }
    CardIo io{c->R[1], c->R[2], c->R[3], c->CPSR & 0xF0000000, false, 0};
    const u64 t0 = __builtin_expect(g_Stats, 0) ? Now() : 0;
    if (!ok || g_Dry || !CardCore(c, io, nullptr))
    {
        s.fallback[6] += !g_Check;
        GuestFallback(c);
        return true;
    }
    s.native[6]++;
    s.cardWords += io.words;
    c->R[1] = io.r1; c->R[2] = io.r2; c->R[3] = io.r3;
    c->CPSR = (c->CPSR & 0x0FFFFFFF) | io.flags;
    c->Cycles += kCardCycWord * (s32)io.words;
    c->JumpTo(io.exit ? kCardExit : kCard);
    if (__builtin_expect(g_Stats, 0)) s.ns += Now() - t0;
    return true;
}

#ifdef LITEV_GX_BULK
__attribute__((noinline)) bool RunGx(melonDS::ARMv5* c, State& s, bool jit)
{
    // a prefix: the native part, then the guest function from its first instruction
    if (!s.gxOk) return false;
#ifdef LITEV_A9HLE_GXCHECK
    if (g_Check) GxCheckAt(s, c);
#endif
    u32 src, len, n;
    if ((jit || GxIntact(c, s)) && GxPlan(c, src, len, n))
    {
        if (g_Check)
        {
#ifdef LITEV_A9HLE_GXCHECK
            if (!g_Gx.on)
            {
                g_Gx.on = true; g_Gx.src = src; g_Gx.len = len; g_Gx.n = n; g_Gx.w.resize(n);
                g_Gx.t0 = c->NDS.ARM9Timestamp + c->Cycles;
                for (u32 i = 0; i < n; i++) g_Gx.w[i] = R32(&c->NDS.MainRAM[(src + i * 4) & c->NDS.MainRAMMask]);
                g_GxTapBuf.clear(); GxOtherSeen = false; GxTap = &g_GxTapBuf;
                s.checks[5]++;
            }
#endif
        }
        else if (g_Dry) s.checks[5]++;
        else
        {
            const u64 t0 = __builtin_expect(g_Stats, 0) ? Now() : 0;
            s.gxWords += GxSend(c, src, len, n);
            s.native[5]++;
            if (__builtin_expect(g_Stats, 0)) s.ns += Now() - t0;
        }
    }
    else s.fallback[5]++;
    GuestFallback(c);
    return true;
}
#endif

__attribute__((noinline)) bool RunOs(melonDS::ARMv5* c, State& s, int k, bool jit)
{
    const u64 t0 = __builtin_expect(g_Stats, 0) ? Now() : 0;
    Mem m(c, g_Check || g_Dry);
    Expect e;
    u32 tpc = 0, tcpsr = 0, bits = 0;
    // JIT: the hook block was compiled after IsHook verified the code, and it depends on every
    // checked byte (ARMJIT adds the Deps() ranges to the block), so a write there invalidates it.
    // Interpreter: compare per call.
    bool ok = !CheckPending && (jit || CodeIntact(c, s, k));
    if (ok)
    {
        IrqIn in{c->R_IRQ[2], c->R[13], c->R_SVC[0], c->R_IRQ[0], c->R_IRQ[1], c->R[12], false, 0, {}};
        ok = k == 0 ? (c->CPSR & 0x3F) == 0x12 && Wake(c, s, m, e, in, tpc, tcpsr) : k == 1 ? SetIrq(c, m, e) : GetIrq(c, m, e, bits);
    }
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
        GuestFallback(c);
        return true;
    }
    s.native[k]++;
    if (k == 0) WakeCommit(c, e);
    else Return(c, e, k == 1 ? kSetCycles : kGetCyclesBase + kGetCyclesPerBit * (s32)bits);
    if (__builtin_expect(g_Stats, 0)) s.ns += Now() - t0;
    return true;
}
}

bool Run(melonDS::ARM* cpu, bool jit)
{
    if (cpu->Num != 0 || (cpu->CPSR & 0x20)) return false;
    auto* c = (melonDS::ARMv5*)cpu;
    const u32 pc = cpu->R[15] - 8, in = cpu->CurInstr;
    int k;
    switch (pc)
    {
    case kWake: k = in == kWakeInstr ? 0 : -1; break;
    case kSet: k = in == kSetInstr ? 1 : -1; break;
    case kGet: k = in == kGetInstr ? 2 : -1; break;
    case kGx: k = in == kGxInstr ? 5 : -1; break;
    case kCard: k = in == kCardInstr ? 6 : -1; break;
    default: return false;
    }
    if (k < 0) return false;
    State& s = Get(c);
    if (s.status != 1 || !(s.mask & Bit(k))) return false;
    s.calls[k]++;
    if (k == 6) return RunCard(c, s, jit);
#ifdef LITEV_GX_BULK
    if (k == 5) return RunGx(c, s, jit);
#else
    if (k == 5) return false;
#endif
    return RunOs(c, s, k, jit);
}

void CheckAt(melonDS::ARM* cpu, u32 pc)
{
    auto* c = (melonDS::ARMv5*)cpu;
    if (g_Card.on) { CardCheckAt(c, pc); return; }
    const Expect& e = g_P.e;
    // guest registers as seen in the interrupted mode after the return
    u32 gR[15], gI[3] = {c->R_IRQ[0], c->R_IRQ[1], c->R_IRQ[2]}, gS[3] = {c->R_SVC[0], c->R_SVC[1], c->R_SVC[2]};
    for (int i = 0; i < 15; i++) gR[i] = cpu->R[i];
    bool atVec = false;     // native IRQ check completed at the entry of the next IRQ
    if (pc == c->ExceptionBase + 0x18)
    {
        if (g_P.vecSkip) g_P.vecSkip = false;
        else if (g_P.kind >= 3 && !g_P.irq && (c->CPSR & 0x1F) == 0x12 && c->R_IRQ[2] == e.CPSR && cpu->R[14] - 4 == (e.retPc & ~1u))
        {
            // the guest returned and took the next IRQ at once: undo that entry's banking
            atVec = true;
            std::swap(gR[13], gI[0]); std::swap(gR[14], gI[1]);
            if ((e.CPSR & 0x1F) == 0x13) { std::swap(gR[13], gS[0]); std::swap(gR[14], gS[1]); }
            if (g_P.kind == 4) { gI[1] = e.IRQ[1]; gI[2] = e.IRQ[2]; }   // overwritten by the new entry
        }
        else g_P.irq = true;
    }
    if (++g_P.steps > 2000000)
    {
        fprintf(stderr, "A9HLE CHECK %s: guest never returned to %08x\n", kName[g_P.kind], g_P.e.retPc);
        CheckPending = false;
        c->Idle2Log = nullptr;
        g_State[&c->NDS].diffs[g_P.kind]++;
        return;
    }
    if (!atVec && (pc != (e.retPc & ~1u) || cpu->CPSR != e.CPSR)) return;
    melonDS::NDS& nds = c->NDS;
    if ((g_P.kind == 0 || g_P.kind == 4) && R32(&nds.MainRAM[(kInfo + 4) & nds.MainRAMMask]) != e.cur) return;
    CheckPending = false;
    c->Idle2Log = nullptr;
    State& s = g_State[&nds];
    const int k = g_P.kind;
    if (g_P.irq && (k == 1 || k == 2))
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
        if (gR[i] != e.R[i])
        {
            if (nd < 12) bl += snprintf(buf + bl, sizeof(buf) - bl, " r%d guest %08x native %08x;", i, gR[i], e.R[i]);
            nd++;
        }
    if (e.banks)
    {
        const u32* gi = gI; const u32* gs = gS;
        for (int i = 0; i < 3; i++)
        {
            if (gi[i] != e.IRQ[i]) { if (nd < 12) bl += snprintf(buf + bl, sizeof(buf) - bl, " irq[%d] guest %08x native %08x;", i, gi[i], e.IRQ[i]); nd++; }
            if (gs[i] != e.SVC[i]) { if (nd < 12) bl += snprintf(buf + bl, sizeof(buf) - bl, " svc[%d] guest %08x native %08x;", i, gs[i], e.SVC[i]); nd++; }
        }
    }
    if (k == 3 || k == 4)
    {
        // IF: the guest acknowledged exactly HBlank (the native path does IF &= ~2)
        int acks = 0, other = 0;
        for (auto& a : g_P.acc)
            if (a.Write && (a.Addr & ~3u) == 0x04000214) { if (a.Addr == 0x04000214 && a.Size == 4 && a.Val == 2) acks++; else other++; }
        if (acks != 1 || other)
        {
            if (nd < 12) bl += snprintf(buf + bl, sizeof(buf) - bl, " IF writes: %d HBlank acks, %d other;", acks, other);
            nd++;
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
