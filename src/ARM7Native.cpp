// LITEV_A7HLE: see ARM7Native.h.
#ifdef LITEV_A7HLE
#include "ARM7Native.h"
#include "ARM.h"
#include "NDS.h"
#include "ARMInterpreter.h"
#include "ARMJIT.h"
#include "ARMJIT_Memory.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <unordered_map>
#if defined(__ANDROID__)
#include <sys/system_properties.h>
#endif

namespace melonDS::A7Native
{
#ifdef LITEV_HLE_DIAG
thread_local bool CheckPending = false;
#endif
namespace
{
struct A7NMask { s16 off; u8 type; u8 arg; };      // type 0 call (arg: callee kind), 1 field (arg: A7NField), 2 call never made
struct A7NSig { const char* name; u8 kind; u8 fam; u32 w0; u64 h; u8 nr; const s16 (*r)[2]; u8 nm; const A7NMask* m; };
#include "A7NSigs.inc"

using Mem = melonDS::ARMJIT_Memory;
inline u32 R32(const u8* p) { u32 v; memcpy(&v, p, 4); return v; }

// host pointer of the 4 bytes at the ARM7 address a (main RAM, shared WRAM as the ARM7 sees it, ARM7 WRAM), else nullptr
u8* Host(melonDS::NDS& nds, u32 a, u8& reg)
{
    if (a & 3) return nullptr;
    if ((a >> 24) == 0x02) { reg = Mem::memregion_MainRAM; return &nds.MainRAM[a & nds.MainRAMMask]; }
    if ((a >> 24) != 0x03) return nullptr;
    if (a < 0x03800000 && nds.SWRAM_ARM7.Mem) { reg = Mem::memregion_SharedWRAM; return &nds.SWRAM_ARM7.Mem[a & nds.SWRAM_ARM7.Mask]; }
    reg = Mem::memregion_WRAM7;
    return &nds.ARM7WRAM[a & (nds.ARM7WRAMSize - 1)];
}
bool Word(melonDS::NDS& nds, u32 a, u32& v) { u8 r; const u8* p = Host(nds, a, r); if (!p) return false; v = R32(p); return true; }

// ---- matching (compile time) -------------------------------------------------------------------------------------
struct Env { u8 fam = 0; u32 f[kFields] = {}; u32 has = 0; };
struct Rng { u32 a, b; };
constexpr int kMaxDeps = 48;

bool Verify(melonDS::NDS& nds, u32 a, u8 kind, Env& e, Rng* deps, int& nd, int depth)
{
    u32 w0;
    if (depth > 6 || !Word(nds, a, w0)) return false;
    for (const A7NSig& s : kSigs)
    {
        if (s.kind != kind || s.w0 != w0) continue;
        u64 h = 0xcbf29ce484222325ull;
        int mi = 0;
        bool ok = true;
        for (int r = 0; r < s.nr && ok; r++)
            for (int o = s.r[r][0]; o < s.r[r][1]; o++)
            {
                u32 w;
                if (!Word(nds, a + 4 * o, w)) { ok = false; break; }
                while (mi < s.nm && s.m[mi].off < o) mi++;
                if (mi < s.nm && s.m[mi].off == o) w = s.m[mi].type == 1 ? 0 : (w & 0xFF000000);
                for (int k = 0; k < 4; k++) h = (h ^ ((w >> (8 * k)) & 0xFF)) * 0x100000001b3ull;
            }
        if (!ok || h != s.h) continue;
        Env e2 = e;
        int nd2 = nd;
        for (int i = 0; i < s.nm && ok; i++)
        {
            const A7NMask& m = s.m[i];
            u32 w;
            Word(nds, a + 4 * m.off, w);
            if (m.type == 0)
                ok = Verify(nds, a + 4 * m.off + 8 + (u32)((s32)(w << 8) >> 6), m.arg, e2, deps, nd2, depth + 1);
            else if (m.type == 1)
            {
                if ((e2.has >> m.arg & 1) && e2.f[m.arg] != w) ok = false;
                e2.f[m.arg] = w; e2.has |= 1u << m.arg;
            }
        }
        for (int r = 0; r < s.nr && ok; r++)
        {
            const Rng q = {a + 4 * (u32)(s32)s.r[r][0], a + 4 * (u32)(s32)s.r[r][1]};
            bool dup = false;
            for (int i = 0; i < nd2 && !dup; i++) dup = deps[i].a == q.a && deps[i].b == q.b;
            if (!dup) { if (nd2 >= kMaxDeps) ok = false; else deps[nd2++] = q; }
        }
        if (!ok) continue;
        if (depth == 0) e2.fam = s.fam;
        e = e2; nd = nd2;
        return true;
    }
    return false;
}

// ---- per console ---------------------------------------------------------------------------------------------------
struct Entry { u32 addr; u8 kind; Env e; };
constexpr u8 kTop[] = {kGetTick, kSetTimer, kInsertAlarm};     // what runs natively at a call
const char* const kName[kKinds] = {"", "", "", "", "OS_GetTick", "", "", "", "", "", "", "OSi_SetTimer", "OSi_InsertAlarm"};
struct State
{
    int on = -1;
    std::vector<Entry> ents;
    u64 calls[kKinds] = {}, native[kKinds] = {}, guest[kKinds] = {}, checks[kKinds] = {}, diffs[kKinds] = {}, timing[kKinds] = {};
};
std::unordered_map<const melonDS::NDS*, State> g_State;
const bool g_Stats = getenv("LITEV_A7HLE_STATS") && atoi(getenv("LITEV_A7HLE_STATS"));
#ifdef LITEV_HLE_DIAG
const bool g_Check = getenv("LITEV_A7N_CHECK") && atoi(getenv("LITEV_A7N_CHECK"));
const bool g_Fit = getenv("LITEV_A7N_FIT") && atoi(getenv("LITEV_A7N_FIT"));
#else
constexpr bool g_Check = false;
#endif

State& Get(melonDS::NDS& nds)
{
    static thread_local const melonDS::NDS* last = nullptr;
    static thread_local State* lastS = nullptr;
    if (&nds == last) return *lastS;
    State& s = g_State[&nds];       // (map nodes are stable)
    last = &nds; lastS = &s;
    return s;
}
// debug.litev.a7hle bit 32 (unset or 1: all on)
bool On(State& s)
{
    if (s.on < 0)
    {
#if defined(__ANDROID__)
        char b[16] = {0};
        const char* v = __system_property_get("debug.litev.a7hle", b) > 0 ? b : nullptr;
#else
        const char* v = getenv("debug.litev.a7hle");
#endif
        const u32 m = v ? (u32)strtoul(v, nullptr, 0) : 1;
        s.on = m == 1 || (m & 32) ? 1 : 0;
    }
    return s.on == 1;
}

struct StatsDump
{
    ~StatsDump()
    {
        if (!g_Stats) return;
        for (auto& [k, s] : g_State)
            for (u8 t : kTop)
                if (s.calls[t] || s.checks[t])
                    fprintf(stderr, "A7N %s: calls=%llu native=%llu guest=%llu checks=%llu check_diffs=%llu timing=%llu\n", kName[t],
                            (unsigned long long)s.calls[t], (unsigned long long)s.native[t], (unsigned long long)s.guest[t],
                            (unsigned long long)s.checks[t], (unsigned long long)s.diffs[t], (unsigned long long)s.timing[t]);
    }
} g_StatsDump;

// the entry of a native function at t: verified now if not seen (a block compiled before, e.g. restored)
const Entry* Find(melonDS::NDS& nds, State& s, u32 t, bool verify)
{
    for (const Entry& e : s.ents) if (e.addr == t) return &e;
    if (!verify) return nullptr;
    Rng d[kMaxDeps];
    for (u8 k : kTop)
    {
        Env env;
        int nd = 0;
        if (Verify(nds, t, k, env, d, nd, 0)) { s.ents.push_back({t, k, env}); return &s.ents.back(); }
    }
    return nullptr;
}

// ---- the natives -------------------------------------------------------------------------------------------------
// A transaction: everything a native call does, applied after it decided to run natively (or compared in check mode).
struct Txn
{
    struct W { u8* p; u32 a, v; u8 reg; };
    struct Io { u32 a, v; s32 t; u8 op; };      // op 0: 16-bit store, 1: IE |= v
    W w[40]; u32 nw = 0;
    Io io[12]; u32 nio = 0;
    u32 r0 = 0, r1 = 0; bool ret = false;
    s32 t = 0;                  // the virtual clock: cycles since the call
    u16 ticks[4]; u32 nticks = 0;
    s32 walk = 0, ins = -1, steps = -1, set = 0, inc = 0, dpath = -1;   // the path taken (cycle fits)
};
struct Ctx
{
    melonDS::NDS& nds;
    const Env& e;
    Txn& x;
    u64 t0;                     // time of the call
    bool interp;                // IO at the fitted offsets (the interpreter's time advances per instruction)
};
bool Rd(Ctx& c, u32 a, u32& v)
{
    for (u32 i = c.x.nw; i-- > 0;) if (c.x.w[i].a == a) { v = c.x.w[i].v; return true; }
    return Word(c.nds, a, v);
}
bool Wr(Ctx& c, u32 a, u32 v)
{
    u8 reg;
    u8* p = Host(c.nds, a, reg);
    if (!p || c.x.nw >= 40) return false;
    c.x.w[c.x.nw++] = {p, a, v, reg};
    return true;
}
bool Io(Ctx& c, u32 a, u32 v, u8 op)
{
    if (c.x.nio >= 12) return false;
    c.x.io[c.x.nio++] = {a, v, c.x.t, op};
    return true;
}

// Cycle model per SDK build: the guest's cycles from the call, fitted in check mode (LITEV_A7N_FIT: the TM0 read times
// from the ticks the guest got, TM1's start time from its counter) on PW, PB, The Sims 3 (TWL) and Shrek, Castlevania,
// Mario Kart DS (NTR); with it the interpreter's native calls read and start the timers at the guest's cycle
struct Cyc
{
    s32 gtRead, gtTot, gtInc;               // OS_GetTick: TM0 read, return, + the IF-pending increment
    s32 stGt, stStart, stEnd, stNeg;        // OSi_SetTimer: its GetTick call; after that returns: TM1 start, return; delta < 0
    s32 iaGt, iaNp, iaApp, iaIns, iaWalk, iaHead, iaHeadEnd;   // OSi_InsertAlarm (after the period arithmetic): see below
};
constexpr Cyc kCyc[2] = {
    {21, 124, 16,  8, 113, 164, -2,  27, 40, 127, 123, 18, 130, 14},   // TWL
    {16,  77, 17,  8, 100, 139, -3,  22, 40, 133, 126, 18, 138, 12},   // NTR
};
// ARM7 multiply cycles by the multiplier's significant bytes (as the interpreter charges UMULL / MLA)
inline s32 MulCyc(u32 x) { return x < 0x100 ? 2 : x < 0x10000 ? 3 : x < 0x1000000 ? 4 : 5; }

bool GetTick(Ctx& c, u64& tick)
{
    const Env& e = c.e;
    const Cyc& k = kCyc[e.fam];
    u32 extra = 0;
    if (e.fam == famTWL)
    {
        // TwlSDK: on a DSi console the IRQ flags are read elsewhere (guest); the software IF of the TWL IRQ wrapper
        const u32 fl = e.f[f_flags];
        u32 init, twl;
        if (!Rd(c, fl + 0x1C, init)) return false;
        if (!init)
        {
            u32 w;
            if (!Word(c.nds, 0x0380FFC8, w)) return false;
            twl = ((w & 0xFF) >> 2 & 3) == 1;
            if (!Wr(c, fl + 4, twl) || !Wr(c, fl + 0x1C, 1)) return false;
            c.x.t += 5;         // (the first call's probe)
        }
        else if (!Rd(c, fl + 4, twl)) return false;
        if (twl == 1) return false;
        u32 h;
        if (!Word(c.nds, 0x0380FFFC, h)) return false;
        if (h == e.f[f_handler] && !Rd(c, e.f[f_soft] + 8, extra)) return false;
    }
    u16 lo;
    if (!c.nds.PeekTimer7(4, c.interp ? c.t0 + c.x.t + k.gtRead : c.t0, lo)) return false;
    if (c.x.nticks < 4) c.x.ticks[c.x.nticks++] = lo;
    const u32 tc = e.f[f_tc] + (e.fam == famTWL ? 8 : 0);
    u32 cl, ch;
    if (!Rd(c, tc, cl) || !Rd(c, tc + 4, ch)) return false;
    u64 cnt = (u64)(ch & 0xFFFF) << 32 | cl;
    c.x.t += k.gtTot;
    if (((c.nds.IF[1] | extra) & 8) && !(lo & 0x8000)) { cnt++; c.x.t += k.gtInc; c.x.inc++; }
    tick = cnt << 16 | lo;
    return true;
}

bool SetTimer(Ctx& c, u32 alarm)
{
    const Env& e = c.e;
    const Cyc& k = kCyc[e.fam];
    u64 tick;
    c.x.t += k.stGt;
    if (!GetTick(c, tick)) return false;
    const s32 tr = c.x.t;                       // (GetTick returned)
    if (!Io(c, 0x04000106, 0, 0)) return false;
    u32 flo, fhi;
    if (!Rd(c, alarm + 0xC, flo) || !Rd(c, alarm + 0x10, fhi)) return false;
    const s64 delta = (s64)(((u64)fhi << 32 | flo) - tick);
    // OSi_EnterTimerCallback(OS_TIMER_1, OSi_AlarmHandler, NULL): callback table entry 1 (12-byte entries), IE |= TIMER1
    if (!Wr(c, e.f[f_cbf] + 12, e.f[f_ah]) || !Wr(c, e.f[f_cba] + 12, 0) || !Io(c, 0, 0x10, 1) || !Wr(c, e.f[f_cbe] + 12, 1))
        return false;
    const u16 count = delta < 0 ? 0xFFFE : delta < 0x10000 ? (u16)~(u32)delta : 0;
    c.x.dpath = delta < 0 ? 1 : delta < 0x10000 ? 0 : 2;
    c.x.set++;
    const s32 adj = delta < 0 ? k.stNeg : 0;
    if (!Io(c, 0x04000104, count, 0)) return false;
    c.x.t = tr + k.stStart + adj;
    if (!Io(c, 0x04000106, 0xC1, 0) || !Io(c, 0, 0x10, 1)) return false;
    c.x.t = tr + k.stEnd + adj;
    return true;
}

bool InsertAlarm(Ctx& c, u32 alarm, u64 fire)
{
    const Env& e = c.e;
    const Cyc& k = kCyc[e.fam];
    u32 plo, phi;
    if (!Rd(c, alarm + 0x1C, plo) || !Rd(c, alarm + 0x20, phi)) return false;
    const u64 period = (u64)phi << 32 | plo;
    if (period)
    {
        u64 tick;
        c.x.t += k.iaGt;
        if (!GetTick(c, tick)) return false;
        u32 slo, shi;
        if (!Rd(c, alarm + 0x24, slo) || !Rd(c, alarm + 0x28, shi)) return false;
        const u64 start = (u64)shi << 32 | slo;
        fire = start;
        s32 p = -40;            // ponytail: start >= tick (no division): estimate (not seen in the fits)
        if (start < tick)
        {
            const u64 n = tick - start, q1 = n / period + 1;
            fire += period * q1;
            // _ll_udiv's 32-bit divider (3 cycles a step, 32 minus its shift) + the period * (q + 1) multiplies
            if ((n >> 32) || (period >> 32)) p = 300;   // ponytail: the 64-bit divide loop (after ~2.3 h of ticks): estimate
            else if ((u32)n < (u32)period) p = 2 * MulCyc((u32)q1) - 10;
            else
            {
                u32 r2 = 0x1C, r3 = (u32)n >> 4;
                const u32 d = (u32)period;
                if (d <= r3 >> 12) { r2 -= 16; r3 >>= 16; }
                if (d <= r3 >> 4) { r2 -= 8; r3 >>= 8; }
                if (d <= r3) r2 -= 4;
                p = 3 * (s32)(32 - r2) + 2 * MulCyc((u32)q1);
                c.x.steps = 32 - r2;
            }
        }
        c.x.t += p;
    }
    else c.x.t += k.iaNp;       // ponytail: one-shot alarm (no GetTick / division): estimate (not seen in the fits)
    const s32 tp = c.x.t;
    if (!Wr(c, alarm + 0xC, (u32)fire) || !Wr(c, alarm + 0x10, (u32)(fire >> 32))) return false;
    const u32 head = e.f[f_q] + (e.fam == famTWL ? 4 : 0), tail = head + 4;
    u32 next;
    if (!Rd(c, head, next)) return false;
    s32 w = 0;
    for (; next; )
    {
        if (++w > 64 || next == alarm) return false;
        u32 nlo, nhi;
        if (!Rd(c, next + 0xC, nlo) || !Rd(c, next + 0x10, nhi)) return false;
        if ((s64)(fire - ((u64)nhi << 32 | nlo)) < 0)
        {
            u32 prev;
            if (!Rd(c, next + 0x14, prev) || !Wr(c, alarm + 0x14, prev) || !Wr(c, next + 0x14, alarm) || !Wr(c, alarm + 0x18, next))
                return false;
            c.x.walk = w; c.x.ins = prev ? 1 : 2;
            if (prev)
            {
                if (!Wr(c, prev + 0x18, alarm)) return false;
                c.x.t = tp + k.iaIns + k.iaWalk * w;
                return true;
            }
            c.x.t = tp + k.iaHead + k.iaWalk * (w - 1);
            if (!Wr(c, head, alarm) || !SetTimer(c, alarm)) return false;
            c.x.t += k.iaHeadEnd;
            return true;
        }
        if (!Rd(c, next + 0x18, next)) return false;
    }
    u32 prev;
    c.x.walk = w; c.x.ins = 0;
    if (!Wr(c, alarm + 0x18, 0) || !Rd(c, tail, prev) || !Wr(c, tail, alarm) || !Wr(c, alarm + 0x14, prev)) return false;
    if (prev)
    {
        if (!Wr(c, prev + 0x18, alarm)) return false;
        c.x.t = tp + k.iaApp + k.iaWalk * w;
        return true;
    }
    c.x.t = tp + k.iaHead;      // ponytail: empty queue (w = 0): estimate as a head insert
    if (!Wr(c, tail, alarm) || !Wr(c, head, alarm) || !SetTimer(c, alarm)) return false;
    c.x.t += k.iaHeadEnd;
    return true;
}

bool Native(melonDS::ARM* cpu, const Entry& en, Txn& x, bool interp)
{
    Ctx c{cpu->NDS, en.e, x, cpu->NDS.ARM7Timestamp, interp};
    switch (en.kind)
    {
    case kGetTick:
    {
        u64 t;
        if (!GetTick(c, t)) return false;
        x.r0 = (u32)t; x.r1 = (u32)(t >> 32); x.ret = true;
        return true;
    }
    case kSetTimer: return SetTimer(c, cpu->R[0]);
    case kInsertAlarm: return InsertAlarm(c, cpu->R[0], (u64)cpu->R[2] << 32 | cpu->R[1]);
    }
    return false;
}

void Commit(melonDS::ARM* cpu, const Txn& x, bool interp)
{
    melonDS::NDS& nds = cpu->NDS;
    for (u32 i = 0; i < x.nw; i++)
    {
        const Txn::W& w = x.w[i];
        if (R32(w.p) == w.v) continue;
        switch (w.reg)
        {
        case Mem::memregion_MainRAM: nds.JIT.CheckAndInvalidate<1, Mem::memregion_MainRAM>(w.a); break;
        case Mem::memregion_SharedWRAM: nds.JIT.CheckAndInvalidate<1, Mem::memregion_SharedWRAM>(w.a); break;
        default: nds.JIT.CheckAndInvalidate<1, Mem::memregion_WRAM7>(w.a); break;
        }
        memcpy(w.p, &w.v, 4);
    }
    for (u32 i = 0; i < x.nio; i++)
    {
        const Txn::Io& o = x.io[i];
        // interpreter: at the guest's time of the store (the instruction's cycles are charged after it, >= t)
        const u64 dt = interp ? (u64)o.t : 0;
        nds.ARM7Timestamp += dt;
        if (o.op == 0) nds.ARM7Write16(o.a, (u16)o.v);
        else nds.ARM7Write32(0x04000210, nds.IE[1] | o.v);
        nds.ARM7Timestamp -= dt;
    }
}

// the guest function from the call to its return, in the interpreter (no IRQs inside; the native's rare fallback)
void Guest(melonDS::ARMv4* c, u32 target, u32 ret)
{
    c->R[14] = ret;
    c->JumpTo(target);
    for (u64 n = 0; n < 4000000; n++)
    {
        if (c->CPSR & 0x20)
        {
            c->R[15] += 2;
            c->CurInstr = c->NextInstr[0];
            c->NextInstr[0] = c->NextInstr[1];
            c->NextInstr[1] = c->CodeRead16(c->R[15]);
            if (c->R[15] - 4 == (ret & ~1u) && (ret & 1)) return;
            ARMInterpreter::THUMBInstrTable[(c->CurInstr >> 6) & 0x3FF](c);
        }
        else
        {
            c->R[15] += 4;
            c->CurInstr = c->NextInstr[0];
            c->NextInstr[0] = c->NextInstr[1];
            c->NextInstr[1] = c->CodeRead32(c->R[15]);
            if (c->R[15] - 8 == ret && !(ret & 1)) return;
            if (c->CheckCondition(c->CurInstr >> 28))
                ARMInterpreter::ARMInstrTable[((c->CurInstr >> 4) & 0xF) | ((c->CurInstr >> 16) & 0xFF0)](c);
            else c->AddCycles_C();
        }
    }
    fprintf(stderr, "A7Native: guest call to %08x never returned\n", target);
}

// ---- compare mode (interpreter) --------------------------------------------------------------------------------------
#ifdef LITEV_HLE_DIAG
struct Pending
{
    const melonDS::ARM* cpu = nullptr;
    State* s = nullptr;
    u32 ret = 0, sp = 0, ie = 0, entry = 0;
    u8 kind = 0;
    Txn x;
    u64 t0 = 0;
    bool irq = false;
    u32 mode = 0;
    std::vector<u8> w7, sw;
    u32 innerRet[4] = {}; u32 nInner = 0;       // inner OS_GetTick calls of the guest: their return addresses
    u16 gticks[4] = {}; u32 ngt = 0;            // ... and the tick (low half) they returned
};
thread_local Pending g_P;

void Arm(melonDS::ARM* cpu, State& s, const Entry& en, const Txn& x, u32 ret)
{
    melonDS::NDS& nds = cpu->NDS;
    g_P.cpu = cpu; g_P.s = &s; g_P.ret = ret; g_P.sp = cpu->R[13]; g_P.ie = nds.IE[1]; g_P.kind = en.kind; g_P.x = x;
    g_P.entry = en.addr; g_P.t0 = nds.ARM7Timestamp; g_P.irq = false; g_P.nInner = 0; g_P.ngt = 0;
    g_P.mode = cpu->CPSR & 0x1F;
    g_P.w7.assign(nds.ARM7WRAM, nds.ARM7WRAM + nds.ARM7WRAMSize);
    if (nds.SWRAM_ARM7.Mem) g_P.sw.assign(nds.SWRAM_ARM7.Mem, nds.SWRAM_ARM7.Mem + nds.SWRAM_ARM7.Mask + 1);
    else g_P.sw.clear();
    CheckPending = true;
}

void Finish(melonDS::ARM* cpu)
{
    CheckPending = false;
    melonDS::NDS& nds = cpu->NDS;
    State& s = *g_P.s;
    const u8 k = g_P.kind;
    const Txn& x = g_P.x;
    const s64 cyc = (s64)(nds.ARM7Timestamp + cpu->Cycles - g_P.t0);
    if (g_P.irq) { CheckPending = false; return; }
    s.checks[k]++;
    // the guest's tick reads (an OS_GetTick call: its return value; inner calls: recorded at their returns)
    u16 gt[4]; u32 ngt = 0;
    if (k == kGetTick) gt[ngt++] = (u16)cpu->R[0];
    else { ngt = g_P.ngt; memcpy(gt, g_P.gticks, sizeof(gt)); }
    if (g_Fit)
    {
        // TM0 read offsets consistent with each guest tick (the timer is linear back from its last update), TM1's start
        fprintf(stderr, "A7NFIT %s cyc=%lld native=%d walk=%d ins=%d steps=%d set=%d inc=%d dpath=%d", kName[k], (long long)cyc, x.t,
                x.walk, x.ins, x.steps, x.set, x.inc, x.dpath);
        const Timer& t1 = nds.Timers[5];
        if (k != kGetTick && (t1.Cnt & 0x80))
        {
            const s64 run = ((s64)t1.Counter - ((s64)t1.Reload << 10)) >> t1.CycleShift;
            fprintf(stderr, " tm1start=%lld", (long long)((s64)nds.TimerTimestamp7() - run - (s64)g_P.t0));
        }
        for (u32 i = 0; i < ngt; i++)
        {
            int lo = -1, hi = -1;
            for (int off = 0; off < 600; off++)
            {
                u16 v;
                if (nds.PeekTimer7(4, g_P.t0 + off, v) && v == gt[i]) { if (lo < 0) lo = off; hi = off; }
            }
            fprintf(stderr, " tick%u=[%d,%d]", i, lo, hi);
        }
        fprintf(stderr, "\n");
    }
    bool same = ngt == x.nticks;
    for (u32 i = 0; same && i < ngt; i++) same = gt[i] == x.ticks[i];
    if (!same) { s.timing[k]++; return; }      // the TM0 reads came out at another time: category B timing, not compared
    int nd = 0;
    auto diff = [&](const char* what, u32 a, u32 g, u32 n)
    {
        if (s.diffs[k] < 10 && nd < 8) fprintf(stderr, "A7N CHECK %s: %s %08x guest %08x native %08x\n", kName[k], what, a, g, n);
        nd++;
    };
    if (x.ret && (cpu->R[0] != x.r0 || cpu->R[1] != x.r1)) diff("r0/r1", 0, cpu->R[0], x.r0);
    // memory: every native write has the guest's value; the guest wrote nothing else (ARM7 / shared WRAM; not the stack)
    std::vector<u8> w7 = g_P.w7, sw = g_P.sw;
    for (u32 i = 0; i < x.nw; i++)
    {
        u32 g;
        Word(nds, x.w[i].a, g);
        if (g != x.w[i].v) diff("mem", x.w[i].a, g, x.w[i].v);
        u8 reg; u8* p = Host(nds, x.w[i].a, reg);
        if (p >= nds.ARM7WRAM && p < nds.ARM7WRAM + nds.ARM7WRAMSize) memcpy(&w7[p - nds.ARM7WRAM], &x.w[i].v, 4);
        else if (!sw.empty() && p >= nds.SWRAM_ARM7.Mem && p < nds.SWRAM_ARM7.Mem + sw.size()) memcpy(&sw[p - nds.SWRAM_ARM7.Mem], &x.w[i].v, 4);
    }
    const u32 spo = g_P.sp & (nds.ARM7WRAMSize - 1);
    for (u32 o = 0; o < nds.ARM7WRAMSize; o += 4)
        if (R32(&w7[o]) != R32(&nds.ARM7WRAM[o]) && !(g_P.sp >= 0x03800000 && o < spo && o + 0x100 >= spo))
            diff("guest-only mem", 0x03800000 + o, R32(&nds.ARM7WRAM[o]), R32(&w7[o]));
    for (u32 o = 0; o < sw.size(); o += 4)
        if (R32(&sw[o]) != R32(&nds.SWRAM_ARM7.Mem[o]) && !(g_P.sp < 0x03800000 && (g_P.sp & nds.SWRAM_ARM7.Mask) > o && (g_P.sp & nds.SWRAM_ARM7.Mask) <= o + 0x100))
            diff("guest-only swram", 0x03000000 + o, R32(&nds.SWRAM_ARM7.Mem[o]), R32(&sw[o]));
    // IO: TM1 reload / control and IE as the native's stores leave them
    u32 ie = g_P.ie; int rel = -1, cnt = -1;
    for (u32 i = 0; i < x.nio; i++)
    {
        const Txn::Io& o = x.io[i];
        if (o.op == 1) ie |= o.v;
        else if (o.a == 0x04000104) rel = o.v;
        else if (o.a == 0x04000106) cnt = o.v;
    }
    if (nds.IE[1] != ie) diff("IE", 0x04000210, nds.IE[1], ie);
    if (rel >= 0 && nds.Timers[5].Reload != rel) diff("TM1 reload", 0x04000104, nds.Timers[5].Reload, rel);
    if (cnt >= 0 && nds.Timers[5].Cnt != (cnt & 0xC7)) diff("TM1CNT", 0x04000106, nds.Timers[5].Cnt, cnt);
    if (nd) s.diffs[k]++;
}
#endif

void Call(melonDS::ARM* cpu, u32 t, u32 ret, bool interp)
{
    melonDS::NDS& nds = cpu->NDS;
    State& s = Get(nds);
    const Entry* en = Find(nds, s, t, true);
    if (en)
    {
        s.calls[en->kind]++;
        Txn x;
        if (Native(cpu, *en, x, interp))
        {
            s.native[en->kind]++;
            Commit(cpu, x, interp);
            if (x.ret) { cpu->R[0] = x.r0; cpu->R[1] = x.r1; }
            cpu->R[14] = ret;
            cpu->Cycles += x.t;
            return;
        }
        s.guest[en->kind]++;
    }
    // the guest code, then back after the call as a non-branching instruction leaves the CPU
    melonDS::ARMv4* c = (melonDS::ARMv4*)cpu;
    const u32 r15 = c->R[15], cpsrT = c->CPSR & 0x20, n0 = c->NextInstr[0], n1 = c->NextInstr[1], cr = c->CodeRegion;
    const s32 cc = c->CodeCycles;
    Guest(c, t, ret);
    c->CPSR = (c->CPSR & ~0x20u) | cpsrT;
    c->R[15] = r15; c->NextInstr[0] = n0; c->NextInstr[1] = n1; c->CodeRegion = cr; c->CodeCycles = cc;
}
}

int At(melonDS::NDS& nds, u32 target, u32* a, u32* b, int max)
{
    State& s = Get(nds);
    if (!On(s) || (target & 3)) return 0;
    u32 w0;
    if (!Word(nds, target, w0)) return 0;
    bool cand = false;
    for (const A7NSig& g : kSigs) cand |= g.w0 == w0 && (g.kind == kGetTick || g.kind == kSetTimer || g.kind == kInsertAlarm);
    if (!cand) return 0;
    for (size_t i = 0; i < s.ents.size(); i++)
        if (s.ents[i].addr == target) { s.ents.erase(s.ents.begin() + i); break; }   // (re-verified: the code may have changed)
    Rng d[kMaxDeps];
    for (u8 k : kTop)
    {
        Env env;
        int nd = 0;
        if (!Verify(nds, target, k, env, d, nd, 0)) continue;
        if (nd > max) return 0;
        s.ents.push_back({target, k, env});
        for (int i = 0; i < nd; i++) { a[i] = d[i].a; b[i] = d[i].b; }
        return nd;
    }
    return 0;
}

void CallA(melonDS::ARM* cpu)
{
    const u32 r15 = cpu->R[15], w = cpu->CurInstr;
    Call(cpu, r15 + (u32)((s32)(w << 8) >> 6), r15 - 4, false);
}
void CallT(melonDS::ARM* cpu)
{
    // (ARMv4: a Thumb BL calls Thumb code; the natives are ARM functions, At() never marks one)
    const u32 r15 = cpu->R[15];
    Call(cpu, cpu->R[14] + ((cpu->CurInstr & 0x7FF) << 1), (r15 - 2) | 1, false);
}

bool Interp(melonDS::ARM* cpu, bool thumb)
{
    if (thumb) return false;
    melonDS::NDS& nds = cpu->NDS;
    State& s = Get(nds);
    if (!On(s)) return false;
    const u32 r15 = cpu->R[15], w = cpu->CurInstr;
    const u32 t = r15 + (u32)((s32)(w << 8) >> 6);
#ifdef LITEV_HLE_DIAG
    if (CheckPending)
    {
        // the guest is being compared: record its inner OS_GetTick calls
        if (cpu == g_P.cpu && g_P.nInner < 4)
            for (const Entry& e : s.ents)
                if (e.addr == t && e.kind == kGetTick) { g_P.innerRet[g_P.nInner++] = r15 - 4; break; }
        return false;
    }
#endif
    // the interpreter checks the code per call (no block to hang the dependency on)
    u32 w0;
    if (!Word(nds, t, w0)) return false;
    bool cand = false;
    for (const A7NSig& g : kSigs) cand |= g.w0 == w0;
    if (!cand) return false;
    for (size_t i = 0; i < s.ents.size(); i++)
        if (s.ents[i].addr == t) { s.ents.erase(s.ents.begin() + i); break; }
    const Entry* en = Find(nds, s, t, true);
    if (!en) return false;
#ifdef LITEV_HLE_DIAG
    if (g_Check)
    {
        Txn x;
        if (Native(cpu, *en, x, true)) Arm(cpu, s, *en, x, r15 - 4);
        return false;           // the guest runs
    }
#endif
    Call(cpu, t, r15 - 4, true);
    return true;
}

#ifdef LITEV_HLE_DIAG
void CheckAt(melonDS::ARM* cpu, u32 pc)
{
    if (cpu != g_P.cpu) return;
    if ((cpu->CPSR & 0x1F) == 0x12 && g_P.mode != 0x12) { g_P.irq = true; return; }   // an IRQ inside the guest window
    if (g_P.nInner && pc == g_P.innerRet[g_P.nInner - 1])
    {
        if (g_P.ngt < 4) g_P.gticks[g_P.ngt++] = (u16)cpu->R[0];
        g_P.nInner--;
    }
    if (pc == g_P.ret && cpu->R[13] == g_P.sp) Finish(cpu);
}
#endif
}
#endif
