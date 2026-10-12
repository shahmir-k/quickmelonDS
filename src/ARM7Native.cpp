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
#include <map>
#include <algorithm>
#include <string>
#include <unordered_map>
#if defined(__ANDROID__)
#include <sys/system_properties.h>
#endif
#if defined(__APPLE__) && defined(LITEV_HLE_DIAG)
extern "C" int thread_selfcounts(int type, void* buf, size_t nbytes);   // libsystem_kernel: this thread's PMU counts
#endif

namespace melonDS::A7Native
{
#ifdef LITEV_HLE_DIAG
thread_local bool CheckPending = false;
#endif
namespace
{
// type 0 call (arg: callee kind), 1 field (arg: A7NField), 2 call never made, 3 code-pointer field (kPtrKind[arg] there)
struct A7NMask { s16 off; u8 type; u8 arg; };
// post: offset of the instruction after the function's first call through a register (mov lr, pc; bx rX), pw: that word
struct A7NSig { const char* name; u8 kind; u8 fam; u32 w0; u64 h; u8 nr; const s16 (*r)[2]; u8 nm; const A7NMask* m; s16 post; u32 pw; };
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

// ---- matching ------------------------------------------------------------------------------------------------------
struct Env
{
    u8 fam = 0;                     // the top function's SDK build
    u8 kf[kKinds] = {};             // the build each kind matched (its callees, code pointers)
    u32 ka[kKinds] = {};            // ... and where
    u32 f[kFields] = {};
    u32 has = 0;
};
struct Rng { u32 a, b; };
constexpr int kMaxDeps = 48;
constexpr int kMaxGuard = 96;       // (the thread switch hook: every function of the IRQ path)
struct Prog { u32 a; u8 k; };       // functions being verified (a cycle through one counts as a match)

thread_local std::vector<Rng>* g_Read = nullptr;
#ifdef LITEV_HLE_DIAG
thread_local u64 g_InvalN = 0;      // (stats: native stores that hit JIT-compiled code)
#define A7N_INVALN() g_InvalN++
#else
constexpr u64 g_InvalN = 0;
#define A7N_INVALN()
#endif    // (Chk: the code a failed verification read)
bool Verify(melonDS::NDS& nds, u32 a, u8 kind, Env& e, Rng* deps, int& nd, int depth, Prog* prog = nullptr, int np = 0)
{
    if (g_Read) g_Read->push_back({a, a + 4});
    Prog local[16];
    if (!prog) prog = local;
    for (int i = 0; i < np; i++) if (prog[i].a == a && prog[i].k == kind) return true;
    u32 w0;
    if (np >= 16 || !Word(nds, a, w0)) return false;
    prog[np] = {a, kind};
    for (const A7NSig& s : kSigs)
    {
        if (s.kind != kind || s.w0 != w0) continue;
        if (g_Read) for (int r = 0; r < s.nr; r++) g_Read->push_back({a + 4 * (u32)(s32)s.r[r][0], a + 4 * (u32)(s32)s.r[r][1]});
        u64 h = 0xcbf29ce484222325ull;
        int mi = 0;
        bool ok = true;
        for (int r = 0; r < s.nr && ok; r++)
            for (int o = s.r[r][0]; o < s.r[r][1]; o++)
            {
                u32 w;
                if (!Word(nds, a + 4 * o, w)) { ok = false; break; }
                while (mi < s.nm && s.m[mi].off < o) mi++;
                if (mi < s.nm && s.m[mi].off == o) w = s.m[mi].type & 1 ? 0 : (w & 0xFF000000);
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
                ok = Verify(nds, a + 4 * m.off + 8 + (u32)((s32)(w << 8) >> 6), m.arg, e2, deps, nd2, depth + 1, prog, np + 1);
            else if (m.type == 1 || m.type == 3)
            {
                if ((e2.has >> m.arg & 1) && e2.f[m.arg] != w) ok = false;
                e2.f[m.arg] = w; e2.has |= 1u << m.arg;
                if (ok && m.type == 3) ok = Verify(nds, w, kPtrKind[m.arg], e2, deps, nd2, depth + 1, prog, np + 1);
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
        e2.kf[kind] = s.fam; e2.ka[kind] = a;
        e = e2; nd = nd2;
        return true;
    }
    return false;
}
const A7NSig* SigOf(u8 kind, u8 fam) { for (const A7NSig& s : kSigs) if (s.kind == kind && s.fam == fam) return &s; return nullptr; }
// the signatures of each kind (kSigs indices; a few per kind)
constexpr int kNSigs = sizeof(kSigs) / sizeof(kSigs[0]);
struct ByKind
{
    u8 n[kKinds] = {}; u8 i[kKinds][8] = {};
    constexpr ByKind() { for (int j = 0; j < kNSigs; j++) { const u8 k = kSigs[j].kind; if (n[k] < 8) i[k][n[k]++] = (u8)j; } }
};
constexpr ByKind kByKind{};
static_assert(kNSigs < 256);

// ---- per console ---------------------------------------------------------------------------------------------------
struct Entry { u32 addr; u8 kind; Env e; };
// a function verified for the IRQ path, with a snapshot of all its code (checked at each use: a pure function of memory)
struct Fn { u32 addr; u8 kind; Env e; std::vector<Rng> rs; std::vector<u8> snap; u64 run = 0; const void* guard = nullptr; };
// what runs natively at a call (OS_DisableInterrupts / OS_RestoreInterrupts too measured slower: the JIT inlines them after
// a followed BL)
constexpr u8 kTop[] = {kGetTick, kSetTimer, kInsertAlarm};
enum Seg : u8 { sBiosIn, sBiosRet, sIrqH, sStub, sIcbPre, sIcbPost, sAwPre, sAwPost, sArrPre, sArrPost, sTs, sHand, kSegs };
const char* const kSegName[kSegs] = {"bios-in", "bios-ret", "irqh", "stub", "icb-pre", "icb-post", "aw-pre", "aw-post", "arr-pre", "arr-post", "ts", "hand"};
const char* KindName(u8 k) { return k == kGetTick ? "OS_GetTick" : k == kSetTimer ? "OSi_SetTimer" : k == kInsertAlarm ? "OSi_InsertAlarm" : "?"; }
struct State
{
    int on = -1, irqOn = -1, handOn = -1, wholeOn = -1, fastOn = -1, leanOn = -1, preOn = -1;
    int preR = 0; bool preSet = false;   // (IrqPre tried this IRQ: its result, for Irq)
    std::vector<Entry> ents;
    std::vector<Fn> fns;
    std::vector<Fn> neg;                // addresses that are not that function: the code that decided it (unchanged: still not)
    const Fn* last[kKinds] = {};        // the last function found per kind (a guarded one needs no further check)
    // A JIT hook block depends on the code of the functions its native run steps through: while it lives, they are
    // unchanged (no snapshot compare). A pure optimization: Chk's answer is the same with or without a guard.
    struct Cover { u32 hook; std::vector<std::pair<u32, u8>> fn; };
    std::vector<Cover> cover;           // what the hooks being compiled guard
    // the thread switch hook guards every function of the IRQ path verified when it was compiled (the alarm IRQs no longer
    // return through the other hooks); a function verified later: that block is recompiled (invalidated at the next IRQ)
    u32 tsHook = 0; const void* tsBlock = nullptr; bool reguard = false;
    u64 run = 0;
    u32 gen = 0;                        // bumped when a function or a guard goes or comes (Fast's plan: the Fns it used)
    struct Plan
    {
        u32 gen = ~0u, h = 0, stub = 0, sw[4] = {}, aw = 0;
        const Fn *fh = nullptr, *fi = nullptr, *fa = nullptr, *ft = nullptr, *fs = nullptr;
        u32 hand[2] = {}; const Fn* handF[2] = {}; bool handPad[2] = {};     // the alarm handlers seen (PAD / sound)
        u32 guestH[2] = {};             // ... and ones that are neither (the guest runs those IRQs: always exact)
    } plan;
    int bios = 0; u32 biosRet = 0;  // ARM7 BIOS IRQ vector: 0 unchecked, 1 the standard one (return stub at biosRet), -1 other
    u64 calls[kKinds] = {}, native[kKinds] = {}, guest[kKinds] = {}, checks[kKinds] = {}, diffs[kKinds] = {}, timing[kKinds] = {};
    u64 irqs = 0, irqNative = 0, exits = 0, segs[kSegs] = {}, chainChecks = 0, chainDiffs = 0, chainTiming = 0, chainLost = 0;
    u64 stops[kSegs + 1] = {};
    u64 verN = 0, shW = 0, shMat = 0, fast = 0;
    std::map<u32, u64> stopAt;          // (stats) the guest functions the chain stopped at
    void Dump() const;
    State() { fns.reserve(256); }      // (no reallocation: a run holds Fn pointers across Chk calls)
    ~State() { Dump(); }
};
#if defined(__APPLE__) && defined(LITEV_HLE_DIAG)
// LITEV_A7N_PMU=1: host instructions per native run (minus an empty measurement's)
// (=1: top-level runs only (irq, exit, call); =2: also the nested chain / commit / apply / per-segment counts, each
// inflating its parent by a counter read)
const int g_PmuLvl = getenv("LITEV_A7N_PMU") ? atoi(getenv("LITEV_A7N_PMU")) : 0;
const bool g_Pmu = g_PmuLvl > 0, g_Pmu2 = g_PmuLvl == 2, g_Pmu3 = g_PmuLvl == 3;   // (=3: each IO store)
u64 g_PmuN[9 + kSegs] = {}, g_PmuI[9 + kSegs] = {}, g_PmuE = 0, g_PmuEN = 0;
struct PmuDump { ~PmuDump() { if (g_Pmu) for (int i = 0; i < 9 + kSegs; i++) if (g_PmuN[i]) fprintf(stderr, "A7N pmu %s: %llu runs, %.0f host instr each (empty %.0f)\n", i >= 9 ? kSegName[i - 9] : (const char*[]){"irq", "exit", "call", "chain", "commit", "apply", "io", "wr", "rd"}[i], (unsigned long long)g_PmuN[i], (double)g_PmuI[i] / g_PmuN[i] - (double)g_PmuE / (g_PmuEN ? g_PmuEN : 1), (double)g_PmuE / (g_PmuEN ? g_PmuEN : 1)); } } g_PmuDump;
struct PmuScope
{
    int k; u64 c0[2];
    bool on;
    PmuScope(int kind) : k(kind), on(kind < 3 ? g_Pmu : kind == 6 ? g_Pmu3 : kind == 7 ? g_PmuLvl == 5 : kind == 8 ? g_PmuLvl == 6 : g_Pmu2) { if (on) { u64 e0[2], e1[2]; thread_selfcounts(1, e0, sizeof(e0)); thread_selfcounts(1, e1, sizeof(e1)); g_PmuE += e1[0] - e0[0]; g_PmuEN++; thread_selfcounts(1, c0, sizeof(c0)); } }
    ~PmuScope() { if (on) { u64 c1[2]; thread_selfcounts(1, c1, sizeof(c1)); g_PmuI[k] += c1[0] - c0[0]; g_PmuN[k]++; } }
};
#define A7N_PMU(k) PmuScope pmu_(k)
// LITEV_A7N_PMU=4: host instructions from the end of a native run that stopped at an alarm handler to the exit hook after it
// (the JIT's way back into guest code, the handler, the block exit): median
u64 g_Gap0 = 0; u32 g_GapPc = 0;
std::map<u32, std::vector<u64>> g_Gaps;
struct GapDump { ~GapDump() { for (auto& [pc, v] : g_Gaps) { std::sort(v.begin(), v.end()); fprintf(stderr, "A7N gap %08x: %zu, median %llu p25 %llu p75 %llu host instr\n", pc, v.size(), (unsigned long long)v[v.size() / 2], (unsigned long long)v[v.size() / 4], (unsigned long long)v[v.size() * 3 / 4]); } } } g_GapDump;
u32 g_LastStop = 0;
// LITEV_A7N_PMU=7: Fast's checkpoints (host instr between consecutive ones, minus a counter read)
u64 g_CpI[16] = {}, g_CpN[16] = {}, g_CpL = 0; int g_CpK = -1;
inline void Cp(int k) { if (g_PmuLvl != 7) return; u64 c[2]; thread_selfcounts(1, c, sizeof(c)); if (g_CpK >= 0 && k > 0) { g_CpI[k] += c[0] - g_CpL; g_CpN[k]++; } g_CpK = k; thread_selfcounts(1, c, sizeof(c)); g_CpL = c[0]; }
struct CpDump { ~CpDump() { if (g_PmuLvl == 7) for (int i = 1; i < 16; i++) if (g_CpN[i]) fprintf(stderr, "A7N cp %d: %llu, %.0f host instr each (incl. one counter read)\n", i, (unsigned long long)g_CpN[i], (double)g_CpI[i] / g_CpN[i]); } } g_CpDump;
#define A7N_CP(k) Cp(k)
#else
#define A7N_CP(k)
#define A7N_PMU(k)
#endif
const bool g_Stats = getenv("LITEV_A7HLE_STATS") && atoi(getenv("LITEV_A7HLE_STATS"));
#ifdef LITEV_HLE_DIAG
const bool g_Check = getenv("LITEV_A7N_CHECK") && atoi(getenv("LITEV_A7N_CHECK"));
const bool g_Fit = getenv("LITEV_A7N_FIT") && atoi(getenv("LITEV_A7N_FIT"));
#else
constexpr bool g_Check = false;
#endif

State& Get(melonDS::NDS& nds)
{
    auto& p = nds.ARM7.A7NState;        // (per console: --mp-test / Netplay run a console per thread)
    if (!p) p = {new State, [](void* q) { delete (State*)q; }};
    return *(State*)p.get();
}
// debug.litev.a7hle (unset or 1: the default set, all on): bit 32 the OS call natives, bit 64 the IRQ path, bit 128 the SDK alarm
// handlers in it, bit 256 whole runs only (the TM1 alarm IRQ vector -> BIOS return, else the guest; no exit hooks), bit 512 that
// run as one function (Fast). 64+128 stepped (exits included) was slower on the RG DS than bit 32 alone (PGO, emu-thread CPU: PW
// +1.3%, Shrek +0.8%, MKDS +1.4%): the A55 pays per native run for its cold code footprint (~160 L1I refills, ~15-25k cycles a
// run at IPC ~0.2), not per instruction. With 256+512: PW -1.4%, Shrek -2.3%, MKDS -0.7% (PGO; non-PGO -1.1 / 0.0 / -0.6%).
u32 Mask()
{
#if defined(__ANDROID__)
    char b[16] = {0};
    const char* v = __system_property_get("debug.litev.a7hle", b) > 0 ? b : nullptr;
#else
    const char* v = getenv("debug.litev.a7hle");
#endif
    const u32 m = v ? (u32)strtoul(v, nullptr, 0) : 1;
    return m == 1 ? ~0u : m;
}
bool On(State& s) { if (s.on < 0) s.on = Mask() & 32 ? 1 : 0; return s.on == 1; }
bool IrqOn(State& s) { if (s.irqOn < 0) s.irqOn = Mask() & 64 ? 1 : 0; return s.irqOn == 1; }
// bit 128: the SDK alarm handlers natively inside the IRQ path (needs 64)
bool HandOn(State& s) { if (s.handOn < 0) s.handOn = Mask() & 128 ? 1 : 0; return s.handOn == 1; }
// bit 512 (needs 256): the TM1 alarm IRQ as one function (Fast), not stepped
bool FastOn(State& s) { if (s.fastOn < 0) s.fastOn = (Mask() & 960) == 960 ? 1 : 0; return s.fastOn == 1; }
// bit 256: whole runs only (needs 64 + 128): a native run starts only for a TM1 alarm IRQ and is kept only when it reaches the
// BIOS return (else undone: the guest runs the IRQ from the vector); no exit hooks run (their blocks only guard the code)
// bit 1024 (needs 512): Fast's lean stores: the IO in one loop at the end (TM1 / RCNT directly, one UpdateIRQ), the TWL
// register save and the context switch's thread save written last without the undo log (nothing can fail after them)
bool LeanOn(State& s) { if (s.leanOn < 0) s.leanOn = (Mask() & 1984) == 1984 ? 1 : 0; return s.leanOn == 1; }
// bit 2048 (needs 1024): the TM1 alarm IRQ from ARM::TriggerIRQ before the vector entry (IrqPre)
bool PreOn(State& s) { if (s.preOn < 0) s.preOn = (Mask() & 4032) == 4032 ? 1 : 0; return s.preOn == 1; }
bool WholeOn(State& s) { if (s.wholeOn < 0) s.wholeOn = (Mask() & 448) == 448 ? 1 : 0; return s.wholeOn == 1; }


void State::Dump() const
{
    if (!g_Stats) return;
    {
        const State& s = *this;
        {
            for (u8 t : kTop)
                if (s.calls[t] || s.checks[t])
                    fprintf(stderr, "A7N %s: calls=%llu native=%llu guest=%llu checks=%llu check_diffs=%llu timing=%llu\n", KindName(t),
                            (unsigned long long)s.calls[t], (unsigned long long)s.native[t], (unsigned long long)s.guest[t],
                            (unsigned long long)s.checks[t], (unsigned long long)s.diffs[t], (unsigned long long)s.timing[t]);
            if (s.irqs || s.exits || s.chainChecks)
            {
                fprintf(stderr, "A7N irq: irqs=%llu native=%llu exits=%llu checks=%llu check_diffs=%llu timing=%llu lost=%llu segs:",
                        (unsigned long long)s.irqs, (unsigned long long)s.irqNative, (unsigned long long)s.exits,
                        (unsigned long long)s.chainChecks, (unsigned long long)s.chainDiffs, (unsigned long long)s.chainTiming,
                        (unsigned long long)s.chainLost);
                for (int i = 0; i < kSegs; i++) fprintf(stderr, " %s=%llu", kSegName[i], (unsigned long long)s.segs[i]);
                fprintf(stderr, " | fast=%llu", (unsigned long long)s.fast);
                fprintf(stderr, " | verify=%llu shadow=%llu/%llu inval=%llu", (unsigned long long)s.verN, (unsigned long long)s.shW, (unsigned long long)s.shMat, (unsigned long long)g_InvalN);
                fprintf(stderr, " | stops:");
                for (int i = 0; i <= kSegs; i++) if (s.stops[i]) fprintf(stderr, " %s=%llu", i < kSegs ? kSegName[i] : "guest", (unsigned long long)s.stops[i]);
                fprintf(stderr, "\n");
                for (const auto& [a, n] : s.stopAt) fprintf(stderr, "A7N stop-at %08x: %llu\n", a, (unsigned long long)n);
            }
        }
    }
}

// the entry of a native function at t: verified now if not seen (a block compiled before, e.g. restored)
const Entry* Find(melonDS::NDS& nds, State& s, u32 t, bool verify)
{
    for (size_t i = 0; i < s.ents.size(); i++)
        if (s.ents[i].addr == t)
        {
            if (i) std::swap(s.ents[i], s.ents[i - 1]);       // (hot targets move to the front)
            return &s.ents[i ? i - 1 : 0];
        }
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

// the IRQ path's function of this kind at a, its code unchanged since it was verified (memcmp of the snapshot, once per
// native run), else verified now; nullptr: not that function
bool Same(melonDS::NDS& nds, const Fn& f)
{
    size_t o = 0;
    for (const Rng& r : f.rs)
    {
        u8 reg;
        const u8* p = Host(nds, r.a, reg);
        if (!p || memcmp(p, &f.snap[o], r.b - r.a) != 0) return false;
        o += r.b - r.a;
    }
    return true;
}
const Fn* Chk(melonDS::NDS& nds, State& s, u32 a, u8 kind)
{
    if (const Fn* f = s.last[kind]; f && f->addr == a && f->guard) return f;
    for (size_t i = 0; i < s.fns.size(); i++)
    {
        Fn& f = s.fns[i];
        if (f.addr != a || f.kind != kind) continue;
        if (f.guard || f.run == s.run || Same(nds, f)) { f.run = s.run; s.last[kind] = &f; return &f; }
        s.fns.erase(s.fns.begin() + i);
        s.gen++;
        for (const Fn*& l : s.last) l = nullptr;
        break;
    }
    u32 w0;
    if (!Word(nds, a, w0)) return nullptr;
    bool cand = false;
    for (int j = 0; j < kByKind.n[kind]; j++) cand |= kSigs[kByKind.i[kind][j]].w0 == w0;
    if (!cand) return nullptr;
    for (size_t i = 0; i < s.neg.size(); i++)
        if (s.neg[i].addr == a && s.neg[i].kind == kind)
        {
            if (Same(nds, s.neg[i])) return nullptr;
            s.neg.erase(s.neg.begin() + i);
            break;
        }
    Fn f{a, kind, {}, {}, {}, s.run};
    s.verN++;
    Rng d[kMaxDeps];
    int nd = 0;
    std::vector<Rng> read;
    g_Read = &read;
    const bool ok = Verify(nds, a, kind, f.e, d, nd, 0);
    g_Read = nullptr;
    if (!ok)
    {
        Fn n{a, kind, {}, {}, {}, 0};
        for (const Rng& r : read)
        {
            u8 reg;
            const u8* p = Host(nds, r.a, reg);
            const u8* q = Host(nds, r.b - 4, reg);
            if (!p || !q || q - p != (ptrdiff_t)(r.b - r.a - 4)) return nullptr;
            n.rs.push_back(r);
            n.snap.insert(n.snap.end(), p, p + (r.b - r.a));
        }
        if (s.neg.size() < 64) s.neg.push_back(std::move(n));
        return nullptr;
    }
    for (int i = 0; i < nd; i++)
    {
        u8 reg;
        const u8* p = Host(nds, d[i].a, reg);
        const u8* q = Host(nds, d[i].b - 4, reg);
        if (!p || !q || q - p != (ptrdiff_t)(d[i].b - d[i].a - 4)) return nullptr;     // (one host block)
        f.rs.push_back(d[i]);
        f.snap.insert(f.snap.end(), p, p + (d[i].b - d[i].a));
    }
    s.fns.push_back(std::move(f));
    s.gen++;
    if (s.tsBlock) s.reguard = true;
    for (const Fn*& l : s.last) l = nullptr;    // (the vector may have moved)
    s.last[kind] = &s.fns.back();
    return &s.fns.back();
}

// ---- the natives -------------------------------------------------------------------------------------------------
constexpr u32 kShWin = 0x200;
// A transaction: everything a native run does, applied after it decided to run natively (or compared in check mode).
struct Txn
{
    struct W { u8* p; u32 a, v; u8 reg; };
    // IO, in program order: op 0 16-bit store, 1 IE |= v, 2 IE &= ~v, 3 IF acknowledge (32-bit store of v to IF)
    struct Io { u32 a, v; s32 t; u8 op; };
    // compare mode (dry): the stores and IO are recorded here, reads see them; else they happen at once and the undo log
    // takes back a segment's stores when it cannot finish (its IO comes after everything that can fail)
    bool dry = false;
    W w[96]; u32 nw = 0;
    u64 bloom = 0;              // words written (bit (a >> 2) & 63): most reads skip the scan
    struct U { u8* p; u32 a, old; u8 reg; };
    U undo[256]; u32 nu = 0;
    Io io[32]; u32 nio = 0;
    u32 ioDone = 0;             // IO stores issued (direct: at the end of each segment / native call; a failed one issues none)
    u32 r0 = 0, r1 = 0; bool ret = false;
    s32 t = 0;                  // the virtual clock: cycles since the start
    u16 ticks[4]; u32 nticks = 0;
    s32 tickT[4];               // (cycle fits: when the native read them)
    s32 walk = 0, ins = -1, steps = -1, set = 0, inc = 0, dpath = -1;   // the path taken (cycle fits)
    s32 tsWake = 0, tsRes = 0, tsWalk = 0, tsSw = 0;
    struct SegAt { u32 pc; s32 t; u8 seg; };
    SegAt seg[24]; u32 nseg = 0;
    // the IRQ stack below the run's starting sp (the frames the IRQ path pushes and pops): kept here, written to memory at
    // the end only where still live (at or above the final IRQ sp; below it: dead stack, category B)
    u32 gtTc = 0, gtCl = 0, gtCh = 0, gtExtra = 0; u16 gtLo = 0;    // the first OS_GetTick's inputs (JIT runs)
    u32 shLo = 0, shTop = 0;    // [shLo, shTop): pushed by this run (shLo: the lowest sp a push reached)
    u32 shMin = 0;              // the lowest sp the shadow may take (the window checked at the start)
    u32 shv[kShWin / 4];        // the words, by (a - shMin) / 4
    u64 shm[kShWin / 256] = {}; // ... which hold one
    struct SU { u32 i, old; };
    SU shu[32]; u32 nshu = 0;   // valid words overwritten (a failed segment restores them)
};
struct Ctx
{
    melonDS::NDS& nds;
    const Env& e;
    Txn& x;
    u64 t0;                     // time of the call
    bool interp;                // IO at the fitted offsets (the interpreter's time advances per instruction)
};
// the JIT invalidation check of a store (ARMJIT::CheckAndInvalidate with the address localised inline for the ARM7's WRAM
// and main RAM: most stores here hit pages without code)
inline void Inval(melonDS::NDS& nds, u32 a, u8 reg)
{
    if (reg == Mem::memregion_WRAM7)
    {
        const u32 o = a & (melonDS::ARM7WRAMSize - 1);
        if (nds.JIT.CodeIndexARM7WRAM[o / 512].Code & (1u << ((o & 0x1FF) / 16))) { A7N_INVALN(); nds.JIT.InvalidateByAddr(o | (Mem::memregion_WRAM7 << 27)); }
    }
    else if (reg == Mem::memregion_MainRAM)
    {
        const u32 o = a & nds.MainRAMMask;
        if (nds.JIT.CodeIndexMainRAM[o / 512].Code & (1u << ((o & 0x1FF) / 16))) { A7N_INVALN(); nds.JIT.InvalidateByAddr(o | (Mem::memregion_MainRAM << 27)); }
    }
    else nds.JIT.CheckAndInvalidate<1, Mem::memregion_SharedWRAM>(a);
}
bool Rd(Ctx& c, u32 a, u32& v)
{
    if (a - c.x.shLo < c.x.shTop - c.x.shLo)
    {
        const u32 i = (a - c.x.shMin) >> 2;
        if (c.x.shm[i >> 6] >> (i & 63) & 1) { v = c.x.shv[i]; return true; }
    }
    if (!c.x.dry) return Word(c.nds, a, v);
    if (c.x.bloom >> (a >> 2 & 63) & 1)
    {
        for (u32 i = c.x.nw; i-- > 0;) if (c.x.w[i].a == a) { v = c.x.w[i].v; return true; }
    }
    return Word(c.nds, a, v);
}
bool Wr(Ctx& c, u32 a, u32 v)
{
    if (a - c.x.shLo < c.x.shTop - c.x.shLo && !(a & 3))
    {
        const u32 i = (a - c.x.shMin) >> 2;
        u64& m = c.x.shm[i >> 6];
        if (m >> (i & 63) & 1) { if (c.x.nshu >= 32) return false; c.x.shu[c.x.nshu++] = {i, c.x.shv[i]}; }
        m |= 1ull << (i & 63);
        c.x.shv[i] = v;
        return true;
    }
    u8 reg;
    u8* p = Host(c.nds, a, reg);
    if (!p) return false;
    if (!c.x.dry)
    {
        const u32 old = R32(p);
        if (old == v) return true;
        if (c.x.nu >= 256) return false;
        c.x.undo[c.x.nu++] = {p, a, old, reg};
        Inval(c.nds, a, reg);
        memcpy(p, &v, 4);
        return true;
    }
    if (c.x.nw >= 96) return false;
    c.x.w[c.x.nw++] = {p, a, v, reg};
    c.x.bloom |= 1ull << (a >> 2 & 63);
    return true;
}
void Undo(melonDS::NDS& nds, Txn& x, u32 to)
{
    while (x.nu > to)
    {
        const Txn::U& u = x.undo[--x.nu];
        Inval(nds, u.a, u.reg);
        memcpy(u.p, &u.old, 4);
    }
}
// an IO store now (or recorded in compare mode); interpreter: at the guest's time of the store (t0 + t: the run's cycles
// are charged after it, >= t)
void IoNow(melonDS::NDS& nds, u32 a, u32 v, u8 op, u64 at, bool interp)
{
    A7N_PMU(6);
    const u64 now = nds.ARM7Timestamp;
    const u64 dt = interp && at > now ? at - now : 0;
    nds.ARM7Timestamp += dt;
    // (the register effects directly, as ARM7IOWrite16 / 32 do them; the IRQ line: IoFlush / Commit after the last store)
    switch (op)
    {
    case 0:
        if (a == 0x04000104) nds.Timers[5].Reload = (u16)v;
        else if (a == 0x04000106) nds.A7NTimerStart(5, (u16)v);
        else nds.ARM7Write16(a, (u16)v);
        break;
    case 1: nds.IE[1] |= v; break;
    case 2: nds.IE[1] &= ~v; break;
    case 3: nds.IF[1] &= ~v; break;
    }
    nds.ARM7Timestamp -= dt;
}
bool Rd16(Ctx& c, u32 a, u32& v) { u32 w; if (!Rd(c, a & ~3u, w)) return false; v = (w >> (8 * (a & 2))) & 0xFFFF; return true; }
bool Wr16(Ctx& c, u32 a, u32 v)
{
    u32 w;
    if (!Rd(c, a & ~3u, w)) return false;
    const u32 sh = 8 * (a & 2);
    return Wr(c, a & ~3u, (w & ~(0xFFFFu << sh)) | (v & 0xFFFF) << sh);
}
bool Io(Ctx& c, u32 a, u32 v, u8 op)
{
    if (c.x.nio >= 32) return false;
    c.x.io[c.x.nio++] = {a, v, c.x.t, op};
    return true;
}
// direct mode: the recorded IO stores not issued yet, in order
void IoFlush(melonDS::NDS& nds, Txn& x, u64 t0, bool interp)
{
    if (x.ioDone == x.nio) return;
    for (; x.ioDone < x.nio; x.ioDone++) { const Txn::Io& o = x.io[x.ioDone]; IoNow(nds, o.a, o.v, o.op, t0 + (u64)o.t, interp); }
    nds.UpdateIRQ(1);
}
// IE / IF as the pending IO leaves them
u32 IE(Ctx& c)
{
    u32 v = c.nds.IE[1];
    for (u32 i = c.x.dry ? 0 : c.x.ioDone; i < c.x.nio; i++) { if (c.x.io[i].op == 1) v |= c.x.io[i].v; else if (c.x.io[i].op == 2) v &= ~c.x.io[i].v; }
    return v;
}
u32 IF(Ctx& c)
{
    u32 v = c.nds.IF[1];
    for (u32 i = c.x.dry ? 0 : c.x.ioDone; i < c.x.nio; i++) if (c.x.io[i].op == 3) v &= ~c.x.io[i].v;
    return v;
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
constexpr Cyc kCyc[3] = {
    {21, 124, 16,  8, 113, 164, -2,  27, 40, 127, 123, 18, 130, 14},   // TWL
    {16,  77, 17,  8, 100, 139, -3,  22, 40, 133, 126, 18, 138, 12},   // NTR
    {19,  74, 16,  8, 110, 161, -3,  20, -68, 134, 127, 18, 139, 12},  // V4 (Kirby, Chrono, CV OoE, M&L BIS)
};
inline int CycIdx(u8 fam) { return fam == famTWL || fam == famT5 ? 0 : fam == famV4 ? 2 : 1; }
inline const Cyc& CycOf(u8 fam) { return kCyc[CycIdx(fam)]; }
// the OS data layouts by build: the tick counter 8 bytes into its literal, the alarm queue's head 4 bytes into its own
inline bool Plus(u8 fam) { return fam == famTWL || fam == famV4; }
// ARM7 multiply cycles by the multiplier's significant bytes (as the interpreter charges UMULL / MLA)
inline s32 MulCyc(u32 x) { return x < 0x100 ? 2 : x < 0x10000 ? 3 : x < 0x1000000 ? 4 : 5; }

bool GetTick(Ctx& c, u64& tick)
{
    const Env& e = c.e;
    const u8 fam = e.kf[kGetTick];
    const Cyc& k = CycOf(fam);
    u32 extra = 0;
    const u32 tc = e.f[f_tc] + (Plus(fam) ? 8 : 0);
    u16 lo;
    u32 cl, ch;
    if (c.x.gtTc == tc && !c.interp)
    {
        // (JIT: the IO happens at the call, so every OS_GetTick of a run reads TM0 at the same time: the same inputs)
        lo = c.x.gtLo; cl = c.x.gtCl; ch = c.x.gtCh; extra = c.x.gtExtra;
        if (c.x.nticks < 4) c.x.ticks[c.x.nticks++] = lo;
    }
    else
    {
    bool probe = false;         // (the first call's probe: not cached, a failed segment undoes its stores)
    if (fam == famTWL)
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
            probe = true;
        }
        else if (!Rd(c, fl + 4, twl)) return false;
        if (twl == 1) return false;
        u32 h;
        if (!Word(c.nds, 0x0380FFFC, h)) return false;
        if (h == e.f[f_handler] && !Rd(c, e.f[f_soft] + 8, extra)) return false;
    }
    if (!c.nds.PeekTimer7(4, c.interp ? c.t0 + c.x.t + k.gtRead : c.t0, lo)) return false;
    if (c.x.nticks < 4) { c.x.tickT[c.x.nticks] = c.x.t + k.gtRead; c.x.ticks[c.x.nticks++] = lo; }
    if (!Rd(c, tc, cl) || !Rd(c, tc + 4, ch)) return false;
    if (!probe) { c.x.gtTc = tc; c.x.gtLo = lo; c.x.gtCl = cl; c.x.gtCh = ch; c.x.gtExtra = extra; }
    }
    u64 cnt = (u64)(ch & 0xFFFF) << 32 | cl;
    c.x.t += k.gtTot;
    if (((IF(c) | extra) & 8) && !(lo & 0x8000)) { cnt++; c.x.t += k.gtInc; c.x.inc++; }
    tick = cnt << 16 | lo;
    return true;
}

bool SetTimer(Ctx& c, u32 alarm)
{
    const Env& e = c.e;
    const Cyc& k = CycOf(e.kf[kSetTimer]);
    u64 tick;
    c.x.t += k.stGt;
    if (!GetTick(c, tick)) return false;
    const s32 tr = c.x.t;                       // (GetTick returned)
    u32 flo, fhi;
    if (!Rd(c, alarm + 0xC, flo) || !Rd(c, alarm + 0x10, fhi)) return false;
    if (!Io(c, 0x04000106, 0, 0)) return false;
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
    const u8 fam = e.kf[kInsertAlarm];
    const Cyc& k = CycOf(fam);
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
    const u32 head = e.f[f_q] + (Plus(fam) ? 4 : 0), tail = head + 4;
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

// t0: the time the native run started (the IO of an interpreter run happens at t0 + its offset)
void Commit(melonDS::ARM* cpu, Txn& x, bool interp, u64 t0)
{
    melonDS::NDS& nds = cpu->NDS;
    if (!x.dry) { IoFlush(nds, x, t0, interp); return; }     // (direct: the memory is written; the IO now)
    for (u32 i = 0; i < x.nw; i++)
    {
        const Txn::W& w = x.w[i];
        if (R32(w.p) == w.v) continue;
        Inval(nds, w.a, w.reg);
        memcpy(w.p, &w.v, 4);
    }
    for (u32 i = 0; i < x.nio; i++)
    {
        const Txn::Io& o = x.io[i];
        // interpreter: at the guest's time of the store (the run's cycles are charged after it, >= t)
        const u64 at = t0 + (u64)o.t, now = nds.ARM7Timestamp;
        (void)now;
        IoNow(nds, o.a, o.v, o.op, at, interp);
    }
    if (x.nio) nds.UpdateIRQ(1);
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

// ---- the IRQ path ----------------------------------------------------------------------------------------------------
// The guest IRQ path as a chain of segments (each the exact transcription of a stretch of guest code between calls),
// stepped natively from the BIOS IRQ vector (TriggerIRQ) or from the instruction a guest function returns to (a JIT hook:
// the thread switch, the IRQ callback dispatcher and OSi_ArrangeTimer after their calls). A segment runs only when every
// function on it is verified and the state is the handled case; the chain stops at a guest function the OS calls (an IRQ
// table function, a timer callback, an alarm handler) or when it returns to the interrupted code. The state at a stop
// is the guest's: registers (incl. banked), the stack frames, memory and IO (scratch registers r1-r3 / r12 and the flags
// at a call into guest code are the guest's too where the code sets them; category B: the cycles).
struct St
{
    u32 r[16];                      // IRQ mode view; r[15] = the next instruction's address
    u32 usr13, usr14, svc13, spsr;  // the user/system bank (R_IRQ[0..1] in IRQ mode), SVC sp, SPSR_irq
    u32 cpsr;
    bool ret = false;               // returned to the interrupted code (r[15] = its pc, CPSR = spsr)
};
struct Cyc2
{
    s32 biosIn, biosRet;
    s32 irqh, irqhBit, irqhSave;    // OS_IrqHandler: to bx r0 (+ per lowest-bit loop step, + the TWL register save)
    s32 irqhNone;                   // ... IME off or nothing pending: to its return
    s32 stub, icbPre, icbNoCall, icbPost, icbDis;
    s32 awPre, awPost;              // (the NTR 2005 build's OSi_AlarmHandler: one cycle less each)
    s32 arrPre, arrEmpty, arrSet, arrSetEnd, arrDue, arrNoHandler, arrPost, arrIns, arrHead, arrEnd;
    s32 tsNone, tsWake, tsWakeEach, tsResched, tsWalk, tsSwitch;
    s32 arrNoIns = 0;               // OSi_ArrangeTimer after a one-shot alarm (no re-insert)
};
// fitted in check mode (LITEV_A7N_FIT: the guest's time at each segment, its TM0 reads and TM1's start; PW, PB, The Sims 3 /
// Shrek, Castlevania, Mario Kart DS); ponytail: irqhNone, icbNoCall, icbDis, arrEmpty, arrNoHandler: instruction-count
// estimates (paths not seen in the fits)
constexpr Cyc2 kCyc2[5] = {
    {16, 8,  40, 5, 9, 12,  6, 38, 32, 26, 30,  5, 6,  63, 12, 20, 8, 47, 40, 8, 6, 8, 8,  23, 12, 8, 28, 10, 78},   // TWL
    {16, 8,  34, 5, 0, 12,  6, 38, 32, 26, 30,  5, 6,  50, 12, 18, 10, 50, 40, 8, 6, 8, 8,  23, 12, 8, 28, 10, 78},  // NTR
    {16, 8,  40, 7, 0, 12,  6, 38, 32, 26, 30,  5, 6,  63, 12, 18, 10, 49, 40, 9, 6, 6, 9,  25, 12, 8, 34, 16, 78, 3},  // V4 (Kirby: arr-post end, one-shot, resched, walk refitted)
    {16, 8,  40, 5, 9, 12,  6, 38, 32, 26, 30,  5, 6,  63, 12, 20, 8, 47, 40, 8, 6, 8, 8,  23, 12, 8, 28, 10, 79},   // T5 (its thread switch: Spider-Man SD, Golden Sun DD)
    {16, 8,  34, 5, 0, 12,  6, 38, 32, 26, 30,  5, 6,  50, 12, 18, 10, 50, 40, 8, 6, 8, 8,  22, 12, 8, 28, 10, 78},  // S2 (its thread switch: Brain Age, Nintendogs, SM64DS; ponytail: its flag wake path estimated as NTR's queue wake)
};
inline const Cyc2& Cyc2Of(u8 fam) { return kCyc2[fam == famT5 ? 3 : fam == famS2 ? 4 : CycIdx(fam)]; }
constexpr u32 kMovLrPc = 0xE1A0E00F;

enum Want : u8 { wBiosIn, wIrqH, wStub, wIcb, wIcbPost, wAw, wAwPost, wArr, wArrPost, wTs, wBiosRet, wNone };
struct Run
{
    melonDS::NDS& nds;
    State& s;
    Txn& x;
    u64 t0;
    bool interp;
    St& st;
    u8 want;                        // the segment expected next (the path the guest code takes; anything else: stop)
    const Env* env = nullptr;       // the IRQ handler's / ArrangeTimer's / ... (for Ctx)
};
bool Push(Run& R, const u32* v, int n)
{
    // stmdb sp!, {...}: lowest register at the lowest address
    u32 sp = R.st.r[13] - 4 * n;
    // (the shadowed IRQ stack grows with the pushes below the run's starting sp, inside the checked window)
    if (sp < R.x.shLo && sp >= R.x.shMin) R.x.shLo = sp;
    Ctx c{R.nds, *R.env, R.x, R.t0, R.interp};
    for (int i = 0; i < n; i++) if (!Wr(c, sp + 4 * i, v[i])) return false;
    R.st.r[13] = sp;
    return true;
}
bool Pop(Run& R, u32* v, int n)
{
    Ctx c{R.nds, *R.env, R.x, R.t0, R.interp};
    for (int i = 0; i < n; i++) if (!Rd(c, R.st.r[13] + 4 * i, v[i])) return false;
    R.st.r[13] += 4 * n;
    return true;
}

// BIOS: stmdb sp!, {r0-r3, r12, lr}; mov r0, #0x04000000; add/mov lr, pc...; ldr pc, [r0, #-4]
bool BiosOk(melonDS::NDS& nds, State& s)
{
    if (s.bios) return s.bios > 0;
    s.bios = -1;
    const auto& b = nds.GetARM7BIOS();
    auto w = [&](u32 a) { u32 v; memcpy(&v, &b[a], 4); return v; };
    const u32 v18 = w(0x18);
    if ((v18 >> 24) != 0xEA) return false;
    const u32 h = 0x18 + 8 + (u32)((s32)(v18 << 8) >> 6);
    if (h + 24 > b.size()) return false;
    if (w(h) != 0xE92D500F || w(h + 4) != 0xE3A00301 || (w(h + 8) != 0xE28FE000 && w(h + 8) != kMovLrPc) || w(h + 12) != 0xE510F004
        || w(h + 16) != 0xE8BD500F || w(h + 20) != 0xE25EF004) return false;
    s.bios = 1; s.biosRet = h + 16;
    return true;
}

// The SDK's alarm handlers natively (inside OSi_ArrangeTimer's segment: an alarm IRQ is one native run from the vector to the
// BIOS return): the PAD alarm (RCNT, then EXTKEYIN's X / Y / hinge bits into the system work area) and the sound driver's
// alarm (OS_SendMessage(queue, 1, NOBLOCK) to its thread: the message, OS_WakeupThread, OS_RescheduleThread in IRQ mode =
// the reschedule flag). The scratch registers are left as the call set them (dead: the rest of the path sets or restores
// them before any stop). Returns the guest's cycles; -1 not one of these (nothing written), -2 failed (the caller fails the
// segment: undone).
struct HandCyc { s32 pad, sndFull, snd, sndDiv, sndWake, sndWakeEach, sndRes; };
// fitted in check mode (LITEV_A7N_FIT: the hand segment's guest time, 1 woken thread); ponytail: sndFull, sndDiv (a slot index
// past the end), and the split of wake / each / resched: estimates (paths not seen in the fits)
constexpr HandCyc kHandCyc[6] = {
    {60, 40, 140, 3, 43, 12, 30},   // TWL (PW)
    {56, 40, 159, 3, 43, 12, 30},   // NTR (Shrek; Mario Kart DS's PAD alarm)
    {55, 40, 139, 3, 43, 12, 30},   // V4 (Kirby)
    {56, 40, 168, 3, 43, 12, 30},   // NTR05 (Mario Kart DS's sound alarm)
    {60, 40, 140, 3, 43, 12, 30},   // T5 (no handler of its own: as TWL)
    {56, 40, 153, 3, 43, 12, 30},   // S2 (Brain Age, Nintendogs: the 2005 sound alarm with the 2.x OS_SendMessage)
};
s32 HandPad(Run& R, const Fn* f, bool lean = false)
{
    melonDS::NDS& nds = R.nds;
    Ctx c{nds, f->e, R.x, R.t0, R.interp};
    const HandCyc& k = kHandCyc[f->e.kf[kPadAlarm]];
    // (lean: the registers' values as ARM7IORead16 returns them, without the bus dispatch)
    const u32 rc = lean ? nds.RCnt : nds.ARM7Read16(0x04000134), key = lean ? (u16)(nds.KeyInput >> 16) : nds.ARM7Read16(0x04000136);
    if (!Io(c, 0x04000134, (rc & ~0xC000u) | 0x8000, 0) || !Wr16(c, f->e.f[f_pad], (key & 0x80 ? 0x8000 : 0) | (key & 0xB) << 10))
        return -2;
    return k.pad;
}
s32 HandSnd(Run& R, const Fn* f)
{
    melonDS::NDS& nds = R.nds;
    Ctx c{nds, f->e, R.x, R.t0, R.interp};
    const Env& e = f->e;
    const u32 q = e.f[f_sndq];
    // the queue: TWL / NTR / V4 send / receive thread queues (head, tail) at +0 / +8, array +0x10, count +0x14, first +0x18,
    // used +0x1C; SDK 2.x thread queues are u16 masks at +0 / +2, array +4, count +8, first +0xC, used +0x10
    const bool s2 = e.kf[kSendMsg] == famS2;
    const HandCyc& k = kHandCyc[s2 ? famS2 : e.kf[kSndAlarm]];
    const u32 oArr = s2 ? 4 : 0x10;
    u32 cnt, used, first, arr;
    if (!Rd(c, q + oArr + 4, cnt) || !Rd(c, q + oArr + 0xC, used)) return -1;
    if ((s32)cnt <= (s32)used) return e.kf[kSndAlarm] == famNTR05 ? -1 : k.sndFull;   // (full: not sent; 2005: *sndfail, the guest)
    if (!Rd(c, q + oArr + 8, first) || !Rd(c, q + oArr, arr) || (s32)used < 0) return -1;
    // _s32_div_f's remainder (sign of the dividend, as C); its cycles: 3 a step after the shift search
    const s32 num = (s32)(first + used), den = (s32)cnt;
    const u32 slot = (u32)(num % den);
    s32 cyc = k.snd;
    if ((u32)std::abs(num) >= (u32)den)
    {
        u32 r2 = 0x1C, r3 = (u32)std::abs(num) >> 4;
        const u32 d = (u32)den;
        if (d <= r3 >> 12) { r2 -= 16; r3 >>= 16; }
        if (d <= r3 >> 4) { r2 -= 8; r3 >>= 8; }
        if (d <= r3) r2 -= 4;
        cyc += k.sndDiv * (s32)(32 - r2);
    }
    if (!Wr(c, arr + 4 * slot, 1) || !Wr(c, q + oArr + 0xC, used + 1)) return -2;
    // OS_WakeupThread(&queue->queueReceive)
    u32 t;
    if (s2)
    {
        // SDK 2.x: every thread (the list from the thread info + 0xC) whose number (+0x50) is in the mask: READY
        const u32 tq = q + 2;
        u32 m;
        if (!Rd16(c, tq, m)) return -2;
        if (!m) return cyc;
        if (!Rd(c, e.f[f_tinfo] + 0xC, t)) return -2;
        for (int n = 0; t; n++)
        {
            u32 b;
            if (n > 64 || !Rd(c, t + 0x50, b)) return -2;
            if ((b & 0xFF) < 32 && (m >> (b & 0xFF) & 1) && !Wr(c, t + 0x48, 1)) return -2;
            if (!Rd(c, t + 0x4C, t)) return -2;
            cyc += k.sndWakeEach;
        }
        if (!Wr16(c, tq, 0)) return -2;
    }
    else
    {
        const u32 tq = q + 8;
        if (!Rd(c, tq, t)) return -2;
        if (!t) return cyc;
        for (int n = 0; t; n++)
        {
            u32 nx;
            if (n > 64 || !Wr(c, t + 0x48, 1) || !Wr(c, t + 0x5C, 0) || !Wr(c, t + 0x60, 0) || !Rd(c, t + 0x64, nx) || !Wr(c, t + 0x64, 0))
                return -2;
            t = nx; cyc += k.sndWakeEach;
        }
        if (!Wr(c, tq + 4, 0) || !Wr(c, tq, 0)) return -2;
    }
    cyc += k.sndWake;
    // OS_RescheduleThread: unless rescheduling is off (its count), in IRQ mode only the flag
    const u8 rf = e.kf[kResched];
    u32 rcnt;
    if (!Rd(c, e.f[f_rsc] + (rf == famTWL || rf == famV4 ? 4 : 0), rcnt)) return -2;
    if (rcnt) return cyc;
    if (!Wr16(c, e.f[f_tinfo], 1)) return -2;
    return cyc + k.sndRes;
}
s32 Hand(Run& R, u32 h)
{
    if (const Fn* f = Chk(R.nds, R.s, h, kPadAlarm)) return HandPad(R, f);
    const Fn* f = Chk(R.nds, R.s, h, kSndAlarm);
    return f ? HandSnd(R, f) : -1;
}

// one segment at st.r[15]: 1 ran, 0 not ours (the guest runs from here)
int Step(Run& R)
{
    St& st = R.st;
    melonDS::NDS& nds = R.nds;
    State& s = R.s;
    const u32 pc = st.r[15];
    if ((st.cpsr & 0x1F) != 0x12 || (pc & 3)) return 0;
    auto at = [&](u8 seg, s32 cyc) { if (R.x.nseg < 24) R.x.seg[R.x.nseg++] = {pc, R.x.t, seg}; s.segs[seg]++; R.x.t += cyc; };
    // the BIOS vector and its return stub
    if (R.want == wBiosIn && pc == 0x18 && BiosOk(nds, s))
    {
        u32 h;
        if (!Word(nds, 0x03FFFFFC, h)) return 0;
        const Fn* fh = Chk(nds, s, h, kIrqHandler);
        if (!fh) return 0;
        R.env = &fh->e;
        const u32 fr[6] = {st.r[0], st.r[1], st.r[2], st.r[3], st.r[12], st.r[14]};
        if (!Push(R, fr, 6)) return 0;
        at(sBiosIn, Cyc2Of(fh->e.fam).biosIn);
        st.r[0] = 0x04000000; st.r[14] = s.biosRet; st.r[15] = h;
        R.want = wIrqH;
        return 1;
    }
    if (R.want == wBiosRet && s.bios > 0 && pc == s.biosRet)
    {
        u32 fr[6];
        Env none; const Env* keep = R.env; if (!R.env) R.env = &none;
        const bool ok = Pop(R, fr, 6);
        R.env = keep;
        if (!ok) return 0;
        at(sBiosRet, kCyc2[0].biosRet);
        st.r[0] = fr[0]; st.r[1] = fr[1]; st.r[2] = fr[2]; st.r[3] = fr[3]; st.r[12] = fr[4]; st.r[14] = fr[5];
        st.ret = true;
        st.r[15] = fr[5] - 4;           // (subs pc, lr, #4: CPSR = SPSR at apply)
        R.want = wNone;
        return 1;
    }
    u32 w0;
    if (R.want >= wNone || !Word(nds, pc, w0)) return 0;
    // an IRQ table stub: TWL ldr ip, [pc, #4]; mov r0, #n; bx ip / NTR mov r0, #n; ldr ip, [pc]; bx ip; then the dispatcher
    {
        u32 w1, w2, w3;
        if (R.want == wStub && (w0 == 0xE59FC004 || (w0 & 0xFFFFFF00) == 0xE3A00000) && Word(nds, pc + 4, w1) && Word(nds, pc + 8, w2) && w2 == 0xE12FFF1C
            && Word(nds, pc + 12, w3))
        {
            const bool twl = w0 == 0xE59FC004;
            const u32 mv = twl ? w1 : w0, ld = twl ? w0 : w1;
            if ((mv & 0xFFFFFF00) == 0xE3A00000 && ld == (twl ? 0xE59FC004u : 0xE59FC000u) && Chk(nds, s, w3, kIrqCallback))
            {
                at(sStub, kCyc2[0].stub);
                st.r[0] = mv & 0xFF; st.r[12] = w3; st.r[15] = w3;
                R.want = wIcb;
                return 1;
            }
        }
    }
    static const u8 kWantKind[] = {kKinds, kIrqHandler, kKinds, kIrqCallback, kIrqCallback, kAlarmWrap, kAlarmWrap, kArrangeTimer,
                                   kArrangeTimer, kThreadSwitch, kKinds, kKinds};
    const u8 wk = kWantKind[R.want];
    if (wk >= kKinds) return 0;
    for (int gi = 0; gi < kByKind.n[wk]; gi++)
    {
        const A7NSig& g = kSigs[kByKind.i[wk][gi]];
        const bool entry = g.w0 == w0, post = g.post && g.pw == w0;
        if (!entry && !post) continue;
        switch (g.kind)
        {
        case kIrqHandler:
        {
            if (!entry || R.want != wIrqH) break;
            const Fn* f = Chk(nds, s, pc, kIrqHandler);
            if (!f) break;
            R.env = &f->e;
            Ctx c{nds, f->e, R.x, R.t0, R.interp};
            const u8 fam = f->e.kf[kIrqHandler];
            const Cyc2& k = Cyc2Of(fam);
            const u32 lr = st.r[14], sp0 = st.r[13];
            u32 v;
            // (stmdb sp!, {lr}: the frame is written below)
            const u32 ime = nds.IME[1];
            u32 pend = IE(c) & IF(c);
            if (fam == famTWL && ime && !pend)
            {
                // TwlSDK: the DSi IRQ flags (IE2 / IF2); on a DS they read 0
                if (nds.ARM7Read32(0x04000218) & nds.ARM7Read32(0x0400021C)) return 0;
            }
            if (!Push(R, &lr, 1)) { st.r[13] = sp0; return 0; }
            if (!ime || !pend)
            {
                // ldmeq sp!, {pc}
                at(sIrqH, k.irqhNone);
                st.r[1] = ime ? 0 : 0; st.r[12] = 0x04000210 + (fam == famTWL && ime ? 8 : 0);
                if (ime) st.r[2] = fam == famTWL ? nds.ARM7Read32(0x0400021C) : IF(c);
                if (!Rd(c, st.r[13], v)) return 0;
                st.r[13] += 4; st.r[15] = v;
                R.want = wBiosRet;
                return 1;
            }
            u32 idx = 0;
            while (!(pend >> idx & 1)) idx++;
            u32 fn;
            if (!Rd(c, f->e.f[f_table] + 4 * idx, fn)) return 0;
            if (fam == famTWL)
            {
                const u32 sv[9] = {st.r[4], st.r[5], st.r[6], st.r[7], st.r[8], st.r[9], st.r[10], st.r[11], st.r[13]};
                for (int i = 0; i < 9; i++) if (!Wr(c, f->e.f[f_save] + 4 * i, sv[i])) return 0;
            }
            if (!Io(c, 0x04000214, 1u << idx, 3)) return 0;
            at(sIrqH, k.irqh + k.irqhBit * (s32)idx + (fam == famTWL ? k.irqhSave : 0));
            st.r[0] = fn; st.r[1] = f->e.f[f_table]; st.r[2] = fam == famTWL ? f->e.f[f_save] : 1u << idx; st.r[3] = 1;
            st.r[12] = 0x04000210; st.r[14] = f->e.f[f_ts]; st.r[15] = fn;
            if (g_Stats) s.stopAt[fn]++;
            // (bx r0 to Thumb code: the guest runs it)
            R.want = wStub;
            return 1;
        }
        case kIrqCallback:
        {
            if (R.want != (entry ? wIcb : wIcbPost)) break;
            const Fn* f = Chk(nds, s, entry ? pc : pc - g.post, kIrqCallback);
            if (!f || f->e.kf[kIrqCallback] != g.fam) break;
            R.env = &f->e;
            Ctx c{nds, f->e, R.x, R.t0, R.interp};
            const bool twl = g.fam == famTWL;
            const Cyc2& k = Cyc2Of(g.fam);
            const u32 icbf = f->e.f[f_icbf], icba = f->e.f[f_icba], icbe = f->e.f[f_icbe], ibits = f->e.f[f_ibits];
            if (entry)
            {
                const u32 n = st.r[0];
                if (n > 31) return 0;
                u32 fn, bit, arg = 0;
                if (!Rd(c, icbf + 12 * n, fn) || !Rd16(c, ibits + 2 * n, bit)) return 0;
                const u32 mask = 1u << (bit & 31);
                if (fn && !Rd(c, icba + 12 * n, arg)) return 0;
                if (twl) { const u32 fr[4] = {st.r[3], st.r[4], st.r[5], st.r[14]}; if (!Push(R, fr, 4)) return 0; }
                else { const u32 fr[3] = {st.r[4], st.r[5], st.r[14]}; if (!Push(R, fr, 3)) return 0; st.r[13] -= 4; }
                if (!Wr(c, icbf + 12 * n, 0)) return 0;
                if (twl) { st.r[1] = fn; st.r[2] = icbf; st.r[3] = bit; st.r[4] = mask; st.r[5] = 12 * n; }
                else { st.r[1] = fn; st.r[2] = icbf; st.r[3] = 1; st.r[4] = 12 * n; st.r[5] = mask; }
                if (fn)
                {
                    at(sIcbPre, k.icbPre);
                    st.r[0] = arg; st.r[14] = f->addr + g.post; st.r[15] = fn;
                    R.want = wAw;
                    return 1;
                }
                at(sIcbPre, k.icbNoCall);
                st.r[0] = 0; st.r[15] = f->addr + g.post;
                R.want = wIcbPost;
                return 1;
            }
            // after the callback: the IRQ check flag, the IRQ off again unless the callback is kept
            const u32 mask = twl ? st.r[4] : st.r[5], off = twl ? st.r[5] : st.r[4];
            u32 flag, en, fr[4];
            if (!Rd(c, 0x0380FFF8, flag) || !Rd(c, icbe + off, en)) return 0;
            const u32 sp = st.r[13] + (twl ? 0 : 4);
            for (int i = 0; i < (twl ? 4 : 3); i++) if (!Rd(c, sp + 4 * i, fr[i])) return 0;
            if (!Wr(c, 0x0380FFF8, flag | mask)) return 0;
            s32 cyc = k.icbPost;
            if (!en)
            {
                const u32 old = IE(c);
                if (!Io(c, 0, mask, 2)) return 0;
                st.r[0] = old; st.r[1] = 0; st.r[2] = 0x04000208; st.r[3] = old; st.r[12] = 0x04000210;
                cyc += k.icbDis;
            }
            else { st.r[0] = en; st.r[1] = flag | mask; st.r[2] = twl ? 0x0380FFF8 : st.r[2]; if (!twl) st.r[1] = 0x0380FFF8; }
            st.r[13] = sp + 4 * (twl ? 4 : 3);
            if (twl) { st.r[3] = fr[0]; st.r[4] = fr[1]; st.r[5] = fr[2]; st.r[14] = fr[3]; }
            else { st.r[4] = fr[0]; st.r[5] = fr[1]; st.r[14] = fr[2]; }
            at(sIcbPost, cyc);
            st.r[15] = st.r[14];
            R.want = wTs;
            return 1;
        }
        case kAlarmWrap:
        {
            if (R.want != (entry ? wAw : wAwPost)) break;
            const Fn* f = Chk(nds, s, entry ? pc : pc - g.post, kAlarmWrap);
            if (!f || f->e.kf[kAlarmWrap] != g.fam) break;
            R.env = &f->e;
            const bool r0 = g.fam != famNTR05;
            if (entry)
            {
                if (r0) { const u32 fr[2] = {st.r[0], st.r[14]}; if (!Push(R, fr, 2)) return 0; }
                else if (!Push(R, &st.r[14], 1)) return 0;
                at(sAwPre, Cyc2Of(g.fam).awPre - (g.fam == famNTR05));
                st.r[14] = pc + 8; st.r[15] = f->e.ka[kArrangeTimer];
                R.want = wArr;
                return 1;
            }
            if (r0) { u32 fr[2]; if (!Pop(R, fr, 2)) return 0; st.r[0] = fr[0]; st.r[14] = fr[1]; }
            else if (!Pop(R, &st.r[14], 1)) return 0;
            at(sAwPost, Cyc2Of(g.fam).awPost - (g.fam == famNTR05));
            st.r[15] = st.r[14];
            R.want = wIcbPost;
            return 1;
        }
        case kArrangeTimer:
        {
            if (R.want != (entry ? wArr : wArrPost)) break;
            const Fn* f = Chk(nds, s, entry ? pc : pc - g.post, kArrangeTimer);
            if (!f || f->e.kf[kArrangeTimer] != g.fam) break;
            R.env = &f->e;
            Ctx c{nds, f->e, R.x, R.t0, R.interp};
            // frames / registers by build: TWL push {r4-r6, lr}, r4 = 0, r5 = alarm, r6 = handler; NTR push {r4, r5, lr} + 4,
            // r4 = alarm, r5 = handler; V4 push {r3-r5, lr}, r4 = alarm, r5 = handler (queue head as TWL's)
            const bool twl = g.fam == famTWL, v4 = g.fam == famV4;
            const Cyc2& k = Cyc2Of(g.fam);
            const u32 q = f->e.f[f_q], head = q + (Plus(g.fam) ? 4 : 0), tail = head + 4;
            u32& ra = twl ? st.r[5] : st.r[4];
            u32& rh = twl ? st.r[6] : st.r[5];
            auto end = [&](s32 cyc) -> bool
            {
                if (twl) { u32 fr[4]; if (!Pop(R, fr, 4)) return false; st.r[4] = fr[0]; st.r[5] = fr[1]; st.r[6] = fr[2]; st.r[14] = fr[3]; }
                else if (v4) { u32 fr[4]; if (!Pop(R, fr, 4)) return false; st.r[3] = fr[0]; st.r[4] = fr[1]; st.r[5] = fr[2]; st.r[14] = fr[3]; }
                else { st.r[13] += 4; u32 fr[3]; if (!Pop(R, fr, 3)) return false; st.r[4] = fr[0]; st.r[5] = fr[1]; st.r[14] = fr[2]; }
                R.x.t += cyc;
                st.r[15] = st.r[14];
                R.want = wAwPost;
                return true;
            };
            // the tail after the alarm (from the post-call instruction): re-insert a periodic alarm, re-arm the timer
            auto tail2 = [&](u32 alarm, u32 handler) -> bool
            {
                u32 plo, phi;
                if (!Rd(c, alarm + 0x1C, plo) || !Rd(c, alarm + 0x20, phi)) return false;
                R.x.t += k.arrPost;
                if (plo | phi)
                {
                    if (!Wr(c, alarm, handler)) return false;
                    R.x.t += k.arrIns;
                    if (!InsertAlarm(c, alarm, 0)) return false;
                }
                else R.x.t += k.arrNoIns;
                u32 h;
                if (!Rd(c, head, h)) return false;
                if (h) { R.x.t += k.arrHead; if (!SetTimer(c, h)) return false; }
                return end(k.arrEnd);
            };
            if (!entry)
            {
                at(sArrPost, 0);
                return tail2(ra, rh) ? 1 : 0;
            }
            if (twl) { const u32 fr[4] = {st.r[4], st.r[5], st.r[6], st.r[14]}; if (!Push(R, fr, 4)) return 0; }
            else if (v4) { const u32 fr[4] = {st.r[3], st.r[4], st.r[5], st.r[14]}; if (!Push(R, fr, 4)) return 0; }
            else { const u32 fr[3] = {st.r[4], st.r[5], st.r[14]}; if (!Push(R, fr, 3)) return 0; st.r[13] -= 4; }
            at(sArrPre, 0);
            const s32 tio = R.x.t;
            R.x.t += k.arrPre;
            // everything that can fail before the IO (a segment that stops leaves no trace)
            u32 flag, alarm, flo = 0, fhi = 0, next = 0, handler = 0, plo = 0, phi = 0, arg = 0;
            u64 tick;
            if (!Rd(c, 0x0380FFF8, flag) || !GetTick(c, tick) || !Rd(c, head, alarm)) return 0;
            if (alarm && (!Rd(c, alarm + 0xC, flo) || !Rd(c, alarm + 0x10, fhi))) return 0;
            const bool due = alarm && tick >= ((u64)fhi << 32 | flo);
            if (due)
            {
                u8 reg;
                if (!Rd(c, alarm + 0x18, next) || (next && !Host(nds, next + 0x14, reg)) || !Rd(c, alarm, handler)
                    || !Rd(c, alarm + 0x1C, plo) || !Rd(c, alarm + 0x20, phi) || !Rd(c, alarm + 4, arg)) return 0;
                if (!handler) return 0;         // ponytail: a due alarm without a handler: the guest (not seen)
            }
            {
                const s32 t = R.x.t;
                R.x.t = tio;
                if (!Io(c, 0x04000106, 0, 0) || !Io(c, 0, 0x10, 2)) return 0;
                R.x.t = t;
            }
            if (!Wr(c, 0x0380FFF8, flag | 0x10)) return 0;
            if (twl) st.r[4] = 0;
            ra = alarm;
            if (!alarm) return end(k.arrEmpty) ? 1 : 0;
            if (!due)
            {
                R.x.t += k.arrSet;
                if (!SetTimer(c, alarm)) return 0;
                return end(k.arrSetEnd) ? 1 : 0;
            }
            if (!Wr(c, head, next) || !(next ? Wr(c, next + 0x14, 0) : Wr(c, tail, 0))) return 0;
            if (!(plo | phi) && !Wr(c, alarm, 0)) return 0;
            rh = handler;
            R.x.t += k.arrDue;
            // the call: r0 = arg (TWL r1 = period lo, r2 = q, r3 = fire hi, ip = fire lo; NTR r1 = period hi, r2 = period lo)
            st.r[0] = arg; st.r[3] = fhi; st.r[12] = flo;
            if (twl || v4) { st.r[1] = plo; st.r[2] = q; } else { st.r[1] = phi; st.r[2] = plo; }
            st.r[14] = f->addr + g.post; st.r[15] = handler;
            if (HandOn(s))
            {
                const s32 th = R.x.t, hc = Hand(R, handler);
                if (hc == -2) return 0;
                if (hc >= 0)
                {
                    // the handler returned: the rest of OSi_ArrangeTimer in this segment (its IO is issued at the end)
                    const u32 post = f->addr + g.post;
                    if (R.x.nseg < 23) { R.x.seg[R.x.nseg++] = {handler, th, sHand}; R.x.seg[R.x.nseg++] = {post, th + hc, sArrPost}; }
                    s.segs[sHand]++; s.segs[sArrPost]++;
                    R.x.t = th + hc;
                    st.r[15] = post;
                    return tail2(alarm, handler) ? 1 : 0;
                }
            }
            if (g_Stats) s.stopAt[handler]++;
            R.want = wNone;             // (the alarm handler: guest code)
            return 1;
        }
        case kThreadSwitch:
        {
            if (!entry || R.want != wTs) break;
            const Fn* f = Chk(nds, s, pc, kThreadSwitch);
            if (!f) break;
            R.env = &f->e;
            Ctx c{nds, f->e, R.x, R.t0, R.interp};
            const Cyc2& k = Cyc2Of(f->e.kf[kThreadSwitch]);
            const u32 irqq = f->e.f[f_irqq], ti = f->e.f[f_tinfo];
            s32 cyc = k.tsNone;
            // SDK 2.x: the thread info one word further (current, list, callback at +8 / +0xC / +0x10)
            const bool s2 = f->e.kf[kThreadSwitch] == famS2;
            const u32 o = s2 ? 4 : 0;
            // wake the threads waiting for an IRQ (OS_WaitIrq)
            u32 t;
            if (s2)
            {
                // SDK 2.x: a pending-IRQ flags halfword (irqq); every thread whose wait-IRQ number (+0x50) is in it: READY
                u32 fl;
                if (!Rd16(c, irqq, fl)) return 0;
                if (fl)
                {
                    if (!Wr16(c, irqq, 0) || !Wr16(c, ti, 1) || !Rd(c, ti + 0xC, t)) return 0;
                    for (int n = 0; t; n++)
                    {
                        u32 b;
                        if (n > 64 || !Rd(c, t + 0x50, b)) return 0;
                        if ((b & 0xFF) < 32 && (fl >> (b & 0xFF) & 1)) { if (!Wr(c, t + 0x48, 1)) return 0; R.x.tsWake++; }
                        if (!Rd(c, t + 0x4C, t)) return 0;
                        cyc += k.tsWakeEach;
                    }
                    cyc += k.tsWake;
                }
            }
            else if (!Rd(c, irqq, t)) return 0;
            else if (t)
            {
                for (int n = 0; t; n++)
                {
                    u32 nx;
                    if (n > 64 || !Wr(c, t + 0x48, 1) || !Wr(c, t + 0x5C, 0) || !Wr(c, t + 0x60, 0) || !Rd(c, t + 0x64, nx) || !Wr(c, t + 0x64, 0))
                        return 0;
                    t = nx; cyc += k.tsWakeEach; R.x.tsWake++;
                }
                if (!Wr(c, irqq, 0) || !Wr(c, irqq + 4, 0) || !Wr16(c, ti, 1)) return 0;
                cyc += k.tsWake;
            }
            u32 resched;
            if (!Rd16(c, ti, resched)) return 0;
            u32 to = 0, cur = 0;
            if (resched)
            {
                cyc += k.tsResched; R.x.tsRes = 1;
                if (!Rd(c, ti + 8 + o, to)) return 0;
                for (int n = 0; to; n++)
                {
                    u32 stt;
                    if (n > 64 || !Rd16(c, to + 0x48, stt)) return 0;
                    if (stt == 1) break;
                    cyc += k.tsWalk + ((to >> 24) == 0x02 ? 11 : 0); R.x.tsWalk++;     // (main RAM: slower loads)
                    if (!Rd(c, to + 0x4C, to)) return 0;
                }
                if (!Rd(c, ti + 4 + o, cur)) return 0;
                u32 cb;
                if (to && to != cur && (!Rd(c, ti + 0xC + o, cb) || cb)) return 0;      // a thread switch callback: the guest
                if (!Wr16(c, ti, 0)) return 0;
            }
            if (!resched || !to || to == cur)
            {
                // popeq {pc} / pop {pc}
                at(sTs, cyc);
                u32 v;
                if (!Rd(c, st.r[13], v)) return 0;
                st.r[13] += 4; st.r[15] = v;
                if (resched) st.cpsr = (st.cpsr & ~0xFFu) | 0x92;
                R.want = wBiosRet;
                return 1;
            }
            // the context switch: the interrupted thread's registers (from the BIOS frame) into its OSThread, the next
            // thread's into the frame
            const u32 S0 = st.r[13];
            u32 fr[7];                      // [S0] = the BIOS return, then r0-r3, r12, lr
            for (int i = 0; i < 7; i++) if (!Rd(c, S0 + 4 * i, fr[i])) return 0;
            const u32 old[18] = {st.spsr, fr[1], fr[2], fr[3], fr[4], st.r[4], st.r[5], st.r[6], st.r[7], st.r[8], st.r[9], st.r[10],
                                 st.r[11], fr[5], st.usr13, st.usr14, fr[6], st.svc13};
            if (!Wr(c, ti + 4 + o, to)) return 0;
            for (int i = 0; i < 18; i++) if (!Wr(c, cur + 4 * i, old[i])) return 0;
            u32 nw[18];
            for (int i = 0; i < 18; i++) if (!Rd(c, to + 4 * i, nw[i])) return 0;
            // msr spsr_fc, r2; lr = pc + 4; ldmib r1, {r0-r14}^; stmda sp!, {r0-r3, ip, lr}; ldmia sp!, {pc}
            st.spsr = (st.spsr & 0x00FFFF00) | (nw[0] & 0xFF0000FF);
            st.svc13 = nw[17];
            for (int i = 0; i < 13; i++) st.r[i] = nw[1 + i];
            st.usr13 = nw[14]; st.usr14 = nw[15]; st.r[14] = nw[16];
            const u32 nf[6] = {nw[1], nw[2], nw[3], nw[4], nw[13], nw[16]};
            for (int i = 0; i < 6; i++) if (!Wr(c, S0 + 4 + 4 * i, nf[i])) return 0;
            st.cpsr = (st.cpsr & ~0xFFu) | 0xD2;
            R.x.tsSw = 1;
            at(sTs, cyc + k.tsSwitch + (((cur | to) >> 24) == 0x02 ? 48 : 0));   // ponytail: main-RAM contexts: one fit
            st.r[13] = S0 + 4; st.r[15] = fr[0];
            R.want = wBiosRet;
            return 1;
        }
        default: break;
        }
    }
    return 0;
}

void Load(melonDS::ARM* cpu, St& st, u32 pc)
{
    for (int i = 0; i < 15; i++) st.r[i] = cpu->R[i];
    st.r[15] = pc;
    st.usr13 = cpu->R_IRQ[0]; st.usr14 = cpu->R_IRQ[1]; st.svc13 = cpu->R_SVC[0]; st.spsr = cpu->R_IRQ[2]; st.cpsr = cpu->CPSR;
    st.ret = false;
}
void Apply(melonDS::ARM* cpu, const St& st)
{
    for (int i = 0; i < 15; i++) cpu->R[i] = st.r[i];
    cpu->R_IRQ[0] = st.usr13; cpu->R_IRQ[1] = st.usr14; cpu->R_SVC[0] = st.svc13; cpu->R_IRQ[2] = st.spsr;
    const s32 cyc = cpu->Cycles;
    if (st.ret) cpu->JumpTo(st.r[15], true);       // (subs pc, lr, #4)
    else
    {
        cpu->CPSR = (cpu->CPSR & ~0xFFu) | (st.cpsr & 0xDF);
        cpu->JumpTo(st.r[15]);
    }
    cpu->Cycles = cyc;                              // (the fetch is in the fitted cycles)
}

// IrqPre's end: the state Apply + the BIOS return leave, from the interrupted (USR / SYS) mode without the entry; the vector
// fetch's cycles as JumpTo(0x18) charges them
void ApplyPre(melonDS::ARM* cpu, const St& st)
{
    for (int i = 0; i < 13; i++) cpu->R[i] = st.r[i];
    cpu->R[13] = st.usr13; cpu->R[14] = st.usr14;
    cpu->R_IRQ[0] = st.r[13]; cpu->R_IRQ[1] = st.r[14]; cpu->R_IRQ[2] = st.spsr; cpu->R_SVC[0] = st.svc13;
    cpu->CPSR = st.spsr | 0x10;        // (RestoreCPSR)
    const s32 cyc = cpu->Cycles + ((melonDS::ARMv4*)cpu)->VectorFetchCycles();
    cpu->JumpTo(cpu->CPSR & 0x20 ? st.r[15] | 1 : st.r[15] & ~1u);
    cpu->Cycles = cyc;
}

// step from the CPU's state at pc; n segments stepped (0: nothing ran)
int Chain(melonDS::ARM* cpu, State& s, u32 pc, u8 want, St& st, Txn& x, bool interp, u64 t0)
{
    Load(cpu, st, pc);
    if ((st.cpsr & 0x1F) == 0x12)
    {
        // (one host block: the stack words at the end need no checks)
        u8 r0, r1;
        const u8* p = Host(cpu->NDS, st.r[13] - kShWin, r0);
        const u8* q = Host(cpu->NDS, st.r[13] - 4, r1);
        if (p && q && r0 == r1 && q - p == (ptrdiff_t)kShWin - 4) { x.shLo = x.shTop = st.r[13]; x.shMin = st.r[13] - kShWin; }
    }
    s.run++;
    Run R{cpu->NDS, s, x, t0, interp, st, want};
    int n = 0;
    while (n < 24 && !st.ret)
    {
        const St s0 = st;
        const u8 w0 = R.want;
        const u32 nw = x.nw, nio = x.nio, nt = x.nticks, ns = x.nseg, nu = x.nu, nshu = x.nshu, shLo = x.shLo;
        const u64 shm0 = x.shm[0], shm1 = x.shm[1];
        const s32 t = x.t;
        int ok;
#if defined(__APPLE__) && defined(LITEV_HLE_DIAG)
        u64 c0[2] = {}, c1[2] = {};
        if (g_Pmu2) thread_selfcounts(1, c0, sizeof(c0));
        ok = Step(R);
        if (g_Pmu2) { thread_selfcounts(1, c1, sizeof(c1)); if (ok > 0 && x.nseg) { const int sg = 9 + x.seg[x.nseg - 1].seg; g_PmuI[sg] += c1[0] - c0[0]; g_PmuN[sg]++; } }
#else
        ok = Step(R);
#endif
        if (ok > 0 && !x.dry && s.wholeOn != 1) IoFlush(cpu->NDS, x, t0, interp);   // (whole runs: all IO at the end)
        if (ok <= 0)
        {
            Undo(cpu->NDS, x, nu);
            st = s0; x.nw = nw; x.nio = nio; x.nticks = nt; x.nseg = ns; x.t = t; R.want = w0; x.shLo = shLo;
            while (x.nshu > nshu) { const Txn::SU& u = x.shu[--x.nshu]; x.shv[u.i] = u.old; }
            x.shm[0] = shm0; x.shm[1] = shm1;
            x.bloom = 0;
            for (u32 i = 0; i < nw; i++) x.bloom |= 1ull << (x.w[i].a >> 2 & 63);
            break;
        }
        n++;
    }
    return n;
}

// ---- the whole alarm IRQ in one function (bit 512, with 256) ---------------------------------------------------------
// The TM1 alarm IRQ from the BIOS vector to the BIOS return, as the chain steps it (Step's segments, in order) but without
// the stepping machinery: no per-step dispatch / state copies / signature scans, no stack frames (every frame the path
// pushes is popped before the BIOS return: dead stack, and the registers end as they were, or as the context switch loads
// them). Same helpers (GetTick, SetTimer, InsertAlarm, Hand) and the same cycles. JIT only (the IO at the call).
// 1 applied, -1 the guest runs the IRQ (as the chain would decide), 0 not this path (the chain decides; nothing written).
// (On the A55 the cost of a native run is its code footprint, cold at every IRQ: RG DS 1 PMU, the chain ~160 L1I refills
// and ~15-25k cycles a run at IPC ~0.2; this path runs a few KB of straight code.)
const A7NSig* SigAt(u8 kind, u8 fam, u32 w0)
{
    for (int j = 0; j < kByKind.n[kind]; j++) { const A7NSig& g = kSigs[kByKind.i[kind][j]]; if (g.w0 == w0 && g.fam == fam) return &g; }
    return nullptr;
}
// lean: n words at a written now, no undo (the caller writes them after everything that can fail); Span checks they are one
// host block first
u8* Span(melonDS::NDS& nds, u32 a, int n, u8& reg)
{
    u8 r1;
    u8* p = Host(nds, a, reg);
    const u8* q = Host(nds, a + 4 * (n - 1), r1);
    return p && q && q - p == 4 * (n - 1) ? p : nullptr;
}
void Put(melonDS::NDS& nds, u8* p, u8 reg, u32 a, const u32* v, int n)
{
    for (int i = 0; i < n; i++)
        if (R32(p + 4 * i) != v[i]) { Inval(nds, a + 4 * i, reg); memcpy(p + 4 * i, &v[i], 4); }
}
// lean: the run's IO at once (JIT: all at the run's time, in order: a timer start runs the timers, whose overflow IRQs
// may set IF again after the acknowledge), TM1 reload / control and RCNT directly (as ARM7IOWrite16), one UpdateIRQ
void FlushLean(melonDS::NDS& nds, const Txn& x)
{
    u32& ie = nds.IE[1];
    u32& fl = nds.IF[1];
    for (u32 i = 0; i < x.nio; i++)
    {
        const Txn::Io& o = x.io[i];
        switch (o.op)
        {
        case 0:
            if (o.a == 0x04000104) nds.Timers[5].Reload = (u16)o.v;
            else if (o.a == 0x04000106) nds.A7NTimerStart(5, (u16)o.v);
            else if (o.a == 0x04000134) nds.RCnt = (u16)o.v;
            else nds.ARM7Write16(o.a, (u16)o.v);
            break;
        case 1: ie |= o.v; break;
        case 2: ie &= ~o.v; break;
        case 3: fl &= ~o.v; break;
        }
    }
    nds.UpdateIRQ(1);
}
// L (bit 1024): the lean run, plan only (every lookup done by an earlier generic run): 2 = not planned (the caller undoes it
// and runs the generic Fast<false>, which plans); a compact function of its own (the generic one is cold)
template <bool L>
#if defined(__GNUC__)
__attribute__((noinline))
#endif
int Fast(melonDS::ARM* cpu, State& s, Txn& x, u64 t0, const St& in, bool pre)
{
    melonDS::NDS& nds = cpu->NDS;
    A7N_CP(1);
    State::Plan& P = s.plan;
    const u32 g0 = s.gen;
    u32 h, w0;
    if (s.bios <= 0 || !Word(nds, 0x03FFFFFC, h)) return 0;
    // (the plan: the functions of the last fast run, all guarded by JIT blocks, none gone since: no lookups)
    bool pl = P.gen == g0 && P.h == h;
    if constexpr (L) if (!pl) return 2;
    // the BIOS vector: its frame (r0-r3, r12, lr) is the interrupted code's registers at the return
    const Fn* fh = P.fh;
    if (!pl && (!Word(nds, h, w0) || !(fh = Chk(nds, s, h, kIrqHandler)) || !SigAt(kIrqHandler, fh->e.kf[kIrqHandler], w0))) return 0;
    const Env& eh = fh->e;
    const u8 fam = eh.kf[kIrqHandler];
    const Cyc2& kh = Cyc2Of(eh.fam);
    x.t = kh.biosIn;
    // OS_IrqHandler: IME on, the lowest pending bit TM1 (Irq checked it)
    const u32 sp0 = in.r[13];
    Ctx ch{nds, eh, x, t0, false};
    u32 stub;
    if (!Rd(ch, eh.f[f_table] + 4 * melonDS::IRQ_Timer1, stub)) return 0;
    u8* svP = nullptr; u8 svReg = 0;
    if (fam == famTWL)
    {
        if (L) { if (!(svP = Span(nds, eh.f[f_save], 9, svReg))) return 0; }     // (written at the end)
        else
        {
        const u32 sv[9] = {in.r[4], in.r[5], in.r[6], in.r[7], in.r[8], in.r[9], in.r[10], in.r[11], sp0 - 28};
        for (int i = 0; i < 9; i++) if (!Wr(ch, eh.f[f_save] + 4 * i, sv[i])) return 0;
        }
    }
    if (!Io(ch, 0x04000214, 1u << melonDS::IRQ_Timer1, 3)) return 0;
    x.t += Cyc2Of(fam).irqh + Cyc2Of(fam).irqhBit * melonDS::IRQ_Timer1 + (fam == famTWL ? Cyc2Of(fam).irqhSave : 0);
    A7N_CP(2);
    // the IRQ table stub -> the IRQ callback dispatcher
    u32 sw[4];
    if ((stub & 3) || !Word(nds, stub, sw[0]) || !Word(nds, stub + 4, sw[1]) || !Word(nds, stub + 8, sw[2]) || !Word(nds, stub + 12, sw[3])) return 0;
    pl = pl && stub == P.stub && !memcmp(sw, P.sw, sizeof(sw));
    if constexpr (L) if (!pl) return 2;
    const bool stw = sw[0] == 0xE59FC004;
    const u32 mv = stw ? sw[1] : sw[0], icbA = sw[3];
    const Fn* fi = P.fi;
    if (!pl)
    {
        const u32 ld = stw ? sw[0] : sw[1];
        u32 wi;
        if (sw[2] != 0xE12FFF1C || (mv & 0xFFFFFF00) != 0xE3A00000 || ld != (stw ? 0xE59FC004u : 0xE59FC000u)) return 0;
        if (!(fi = Chk(nds, s, icbA, kIrqCallback)) || !Word(nds, icbA, wi) || !SigAt(kIrqCallback, fi->e.kf[kIrqCallback], wi)) return 0;
    }
    const u8 famI = fi->e.kf[kIrqCallback];
    x.t += kCyc2[0].stub;
    const Env& ei = fi->e;
    Ctx ci{nds, ei, x, t0, false};
    const Cyc2& ki = Cyc2Of(famI);
    const u32 n = mv & 0xFF;
    if (n > 31) return 0;
    const u32 icbf = ei.f[f_icbf], icbe = ei.f[f_icbe];
    u32 aw, bit, arg = 0;
    if (!Rd(ci, icbf + 12 * n, aw) || !Rd16(ci, ei.f[f_ibits] + 2 * n, bit) || !aw || !Rd(ci, ei.f[f_icba] + 12 * n, arg)) return 0;
    const u32 mask = 1u << (bit & 31);
    if (!Wr(ci, icbf + 12 * n, 0)) return 0;
    x.t += ki.icbPre;
    A7N_CP(3);
    // OSi_AlarmHandler -> OSi_ArrangeTimer
    pl = pl && aw == P.aw;
    if constexpr (L) if (!pl) return 2;
    const Fn* fa = P.fa;
    u32 wa;
    if (!pl && (!(fa = Chk(nds, s, aw, kAlarmWrap)) || !Word(nds, aw, wa) || !SigAt(kAlarmWrap, fa->e.kf[kAlarmWrap], wa))) return 0;
    const u8 famA = fa->e.kf[kAlarmWrap];
    x.t += Cyc2Of(famA).awPre - (famA == famNTR05);
    const u32 arr = fa->e.ka[kArrangeTimer];
    const Fn* ft = P.ft;
    u32 wt;
    if (!pl && (!(ft = Chk(nds, s, arr, kArrangeTimer)) || !Word(nds, arr, wt) || !SigAt(kArrangeTimer, ft->e.kf[kArrangeTimer], wt))) return 0;
    const u8 famT = ft->e.kf[kArrangeTimer];
    const Env& et = ft->e;
    Ctx ct{nds, et, x, t0, false};
    const Cyc2& kt = Cyc2Of(famT);
    const u32 q = et.f[f_q], head = q + (Plus(famT) ? 4 : 0), tail = head + 4;
    const s32 tio = x.t;
    x.t += kt.arrPre;
    u32 flag, alarm, flo = 0, fhi = 0, next = 0, handler = 0, plo = 0, phi = 0;
    u64 tick;
    if (!Rd(ct, 0x0380FFF8, flag) || !GetTick(ct, tick) || !Rd(ct, head, alarm) || !alarm) return 0;
    if (!Rd(ct, alarm + 0xC, flo) || !Rd(ct, alarm + 0x10, fhi)) return 0;
    const bool due = tick >= ((u64)fhi << 32 | flo);
    if (due)
    {
        u8 reg;
        if (!Rd(ct, alarm + 0x18, next) || (next && !Host(nds, next + 0x14, reg)) || !Rd(ct, alarm, handler)
            || !Rd(ct, alarm + 0x1C, plo) || !Rd(ct, alarm + 0x20, phi) || !handler) return 0;
    }
    {
        const s32 t = x.t;
        x.t = tio;
        if (!Io(ct, 0x04000106, 0, 0) || !Io(ct, 0, 0x10, 2)) return 0;
        x.t = t;
    }
    if (!Wr(ct, 0x0380FFF8, flag | 0x10)) return 0;
    A7N_CP(4);
    if (!due)
    {
        x.t += kt.arrSet;
        if (!SetTimer(ct, alarm)) return 0;
        x.t += kt.arrSetEnd;
    }
    else
    {
        if (!Wr(ct, head, next) || !(next ? Wr(ct, next + 0x14, 0) : Wr(ct, tail, 0))) return 0;
        if (!(plo | phi) && !Wr(ct, alarm, 0)) return 0;
        x.t += kt.arrDue;
        // the SDK alarm handler natively (PAD / sound), else the guest runs the IRQ
        St dummy;
        Run R{nds, s, x, t0, false, dummy, wNone};
        const int hi = !pl ? -1 : P.hand[0] == handler ? 0 : P.hand[1] == handler ? 1 : -1;
        s32 hc;
        if (hi >= 0) hc = P.handPad[hi] ? HandPad(R, P.handF[hi], L) : HandSnd(R, P.handF[hi]);
        else if constexpr (L) return P.guestH[0] == handler || P.guestH[1] == handler ? -1 : 2;
        else
        {
            const Fn* f = Chk(nds, s, handler, kPadAlarm);
            const bool pad = f;
            if (!pad) f = Chk(nds, s, handler, kSndAlarm);
            if (!f)
            {
                if (pl && s.gen == g0) P.guestH[P.guestH[0] ? 1 : 0] = handler;
                return s.gen == g0 ? -1 : 0;
            }
            if (s.gen != g0) return 0;
            hc = pad ? HandPad(R, f, L) : HandSnd(R, f);
            if (pl && f->guard)
            {
                const int k = P.hand[0] ? 1 : 0;
                P.hand[k] = handler; P.handF[k] = f; P.handPad[k] = pad;
            }
        }
        if (hc < 0) return hc == -1 ? -1 : 0;
        x.t += hc;
        A7N_CP(5);
        // the rest of OSi_ArrangeTimer: re-insert a periodic alarm, re-arm the timer
        if (!Rd(ct, alarm + 0x1C, plo) || !Rd(ct, alarm + 0x20, phi)) return 0;
        x.t += kt.arrPost;
        if (plo | phi)
        {
            if (!Wr(ct, alarm, handler)) return 0;
            x.t += kt.arrIns;
            if (!InsertAlarm(ct, alarm, 0)) return 0;
        }
        else x.t += kt.arrNoIns;
        A7N_CP(6);
        u32 hd;
        if (!Rd(ct, head, hd)) return 0;
        if (hd) { x.t += kt.arrHead; if (!SetTimer(ct, hd)) return 0; }
        x.t += kt.arrEnd;
    }
    A7N_CP(7);
    x.t += Cyc2Of(famA).awPost - (famA == famNTR05);
    // the IRQ callback dispatcher after the callback: the IRQ check flag, the IRQ off unless the callback is kept
    {
        u32 fl, en;
        if (!Rd(ci, 0x0380FFF8, fl) || !Rd(ci, icbe + 12 * n, en) || !Wr(ci, 0x0380FFF8, fl | mask)) return 0;
        s32 cyc = ki.icbPost;
        if (!en) { if (!Io(ci, 0, mask, 2)) return 0; cyc += ki.icbDis; }
        x.t += cyc;
    }
    A7N_CP(8);
    // the thread switch (OS_IrqHandler's return goes through it)
    const u32 tsA = eh.f[f_ts];
    const Fn* fs = P.fs;
    u32 ws;
    if (!pl && (!(fs = Chk(nds, s, tsA, kThreadSwitch)) || !Word(nds, tsA, ws) || !SigAt(kThreadSwitch, fs->e.kf[kThreadSwitch], ws))) return 0;
    if (s.gen != g0) return 0;      // (a function verified or dropped during the run: the chain decides)
    const Env& es = fs->e;
    Ctx cs{nds, es, x, t0, false};
    const Cyc2& ks = Cyc2Of(es.kf[kThreadSwitch]);
    const u32 irqq = es.f[f_irqq], ti = es.f[f_tinfo];
    s32 cyc = ks.tsNone;
    const bool s2 = es.kf[kThreadSwitch] == famS2;
    const u32 o = s2 ? 4 : 0;
    u32 t;
    if (s2)
    {
        u32 fl;
        if (!Rd16(cs, irqq, fl)) return 0;
        if (fl)
        {
            if (!Wr16(cs, irqq, 0) || !Wr16(cs, ti, 1) || !Rd(cs, ti + 0xC, t)) return 0;
            for (int k = 0; t; k++)
            {
                u32 b;
                if (k > 64 || !Rd(cs, t + 0x50, b)) return 0;
                if ((b & 0xFF) < 32 && (fl >> (b & 0xFF) & 1) && !Wr(cs, t + 0x48, 1)) return 0;
                if (!Rd(cs, t + 0x4C, t)) return 0;
                cyc += ks.tsWakeEach;
            }
            cyc += ks.tsWake;
        }
    }
    else if (!Rd(cs, irqq, t)) return 0;
    else if (t)
    {
        for (int k = 0; t; k++)
        {
            u32 nx;
            if (k > 64 || !Wr(cs, t + 0x48, 1) || !Wr(cs, t + 0x5C, 0) || !Wr(cs, t + 0x60, 0) || !Rd(cs, t + 0x64, nx) || !Wr(cs, t + 0x64, 0))
                return 0;
            t = nx; cyc += ks.tsWakeEach;
        }
        if (!Wr(cs, irqq, 0) || !Wr(cs, irqq + 4, 0) || !Wr16(cs, ti, 1)) return 0;
        cyc += ks.tsWake;
    }
    u32 resched, to = 0, cur = 0;
    if (!Rd16(cs, ti, resched)) return 0;
    if (resched)
    {
        cyc += ks.tsResched;
        if (!Rd(cs, ti + 8 + o, to)) return 0;
        for (int k = 0; to; k++)
        {
            u32 stt;
            if (k > 64 || !Rd16(cs, to + 0x48, stt)) return 0;
            if (stt == 1) break;
            cyc += ks.tsWalk + ((to >> 24) == 0x02 ? 11 : 0);
            if (!Rd(cs, to + 0x4C, to)) return 0;
        }
        if (!Rd(cs, ti + 4 + o, cur)) return 0;
        u32 cb;
        if (to && to != cur && (!Rd(cs, ti + 0xC + o, cb) || cb)) return 0;
        if (!Wr16(cs, ti, 0)) return 0;
    }
    St st = in;
    st.r[15] = in.r[14] - 4;            // (the BIOS return: subs pc, lr, #4)
    st.ret = true;
    if (!resched || !to || to == cur) x.t += cyc;
    else
    {
        // the context switch: the interrupted thread's registers into its OSThread, the next thread's loaded
        const u32 old[18] = {st.spsr, st.r[0], st.r[1], st.r[2], st.r[3], st.r[4], st.r[5], st.r[6], st.r[7], st.r[8], st.r[9], st.r[10],
                             st.r[11], st.r[12], st.usr13, st.usr14, st.r[14], st.svc13};
        u32 nw[18];
        if (L)
        {
            // (the next thread's registers first: then nothing can fail, the old thread's are written without the undo log)
            u8 rc, rt, rn;
            const u8* pt = Span(nds, to, 18, rt);
            u8* pc = Span(nds, cur, 18, rc);
            u8* pn = Span(nds, ti + 4 + o, 1, rn);
            if (!pt || !pc || !pn || !(to + 72 <= cur || cur + 72 <= to) || ti + 4 + o - to < 72) return 0;
            for (int i = 0; i < 18; i++) nw[i] = R32(pt + 4 * i);
            Put(nds, pn, rn, ti + 4 + o, &to, 1);
            Put(nds, pc, rc, cur, old, 18);
        }
        else
        {
        if (!Wr(cs, ti + 4 + o, to)) return 0;
        for (int i = 0; i < 18; i++) if (!Wr(cs, cur + 4 * i, old[i])) return 0;
        for (int i = 0; i < 18; i++) if (!Rd(cs, to + 4 * i, nw[i])) return 0;
        }
        st.spsr = (st.spsr & 0x00FFFF00) | (nw[0] & 0xFF0000FF);
        st.svc13 = nw[17];
        for (int i = 0; i < 13; i++) st.r[i] = nw[1 + i];
        st.usr13 = nw[14]; st.usr14 = nw[15]; st.r[14] = nw[16];
        st.r[15] = nw[16] - 4;
        x.t += cyc + ks.tsSwitch + (((cur | to) >> 24) == 0x02 ? 48 : 0);
    }
    x.t += kCyc2[0].biosRet;
    s.fast++;
    if (!L && !pl && fh->guard && fi->guard && fa->guard && ft->guard && fs->guard)
    {
        // every function of this run is guarded by a JIT block (unchanged while it lives; s.gen moves when one goes)
        P = State::Plan{};
        P.gen = g0; P.h = h; P.stub = stub; memcpy(P.sw, sw, sizeof(sw)); P.aw = aw;
        P.fh = fh; P.fi = fi; P.fa = fa; P.ft = ft; P.fs = fs;
    }
    // (pre: the run ends in the interrupted mode's bank, USR / SYS; another (a context switch into one): the vector entry first)
    if (pre && (st.spsr & 0x1F) != 0x10 && (st.spsr & 0x1F) != 0x1F) return 0;
    A7N_CP(9);
    if (L)
    {
        if (svP)
        {
            const u32 sv[9] = {in.r[4], in.r[5], in.r[6], in.r[7], in.r[8], in.r[9], in.r[10], in.r[11], sp0 - 28};
            Put(nds, svP, svReg, eh.f[f_save], sv, 9);
        }
        FlushLean(nds, x);
    }
    else IoFlush(nds, x, t0, false);
    A7N_CP(10);
    if (pre) ApplyPre(cpu, st);
    else Apply(cpu, st);
    A7N_CP(11);
    cpu->Cycles += x.t;
    return 1;
}
}

// the generic run (plans the lean one): its own transaction, undone unless applied
#if defined(__GNUC__)
__attribute__((noinline, cold))
#endif
static int Slow(melonDS::ARM* cpu, State& s, u64 t0, const St& in, bool pre)
{
    Txn x;
    const int r = Fast<false>(cpu, s, x, t0, in, pre);
    if (r != 1) Undo(cpu->NDS, x, 0);
    return r;
}

// the TM1 alarm IRQ as one run: the lean one, else the generic one (plans it); 1 applied, -1 the guest runs it, 0 the chain decides
int TryFast(melonDS::ARM* cpu, State& s, const St& in, bool pre)
{
    melonDS::NDS& nds = cpu->NDS;
    int r = 2;
    if (LeanOn(s))
    {
        Txn x;
        r = Fast<true>(cpu, s, x, nds.ARM7Timestamp, in, pre);
        if (r != 1) Undo(nds, x, 0);    // (nothing issued: the IO comes at the end)
    }
    if (r == 2) r = Slow(cpu, s, nds.ARM7Timestamp, in, pre);
    return r;
}

// ---- entry points ----------------------------------------------------------------------------------------------------
int At(melonDS::NDS& nds, u32 target, u32* a, u32* b, int max)
{
    State& s = Get(nds);
    if (!On(s) || (target & 3)) return 0;
    u32 w0;
    if (!Word(nds, target, w0)) return 0;
    bool cand = false;
    for (const A7NSig& g : kSigs)
        for (u8 k : kTop) cand |= g.w0 == w0 && g.kind == k;
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

namespace
{
void Call(melonDS::ARM* cpu, u32 t, u32 ret, bool interp)
{
    A7N_PMU(2);
    melonDS::NDS& nds = cpu->NDS;
    State& s = Get(nds);
    const Entry* en = Find(nds, s, t, true);
    u16 tm0;
    if (en && nds.PeekTimer7(4, interp ? nds.ARM7Timestamp + 4000 : nds.ARM7Timestamp, tm0))
    {
        s.calls[en->kind]++;
        Txn x;
        if (Native(cpu, *en, x, interp))
        {
            s.native[en->kind]++;
            Commit(cpu, x, interp, nds.ARM7Timestamp);
            if (x.ret) { cpu->R[0] = x.r0; cpu->R[1] = x.r1; }
            cpu->R[14] = ret;
            cpu->Cycles += x.t;
            return;
        }
        Undo(nds, x, 0);
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

#ifdef LITEV_HLE_DIAG
// ---- compare mode (interpreter) ----------------------------------------------------------------------------------------
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
    // an IRQ-path chain
    bool chain = false;
    St end;
    u32 ime = 0, ifl = 0, n = 0, segi = 0;
    s64 segT[24] = {};
    u16 tmR = 0, tmC = 0;
};
thread_local Pending g_P;

void Snap(melonDS::NDS& nds)
{
    g_P.w7.assign(nds.ARM7WRAM, nds.ARM7WRAM + nds.ARM7WRAMSize);
    if (nds.SWRAM_ARM7.Mem) g_P.sw.assign(nds.SWRAM_ARM7.Mem, nds.SWRAM_ARM7.Mem + nds.SWRAM_ARM7.Mask + 1);
    else g_P.sw.clear();
}
void Arm(melonDS::ARM* cpu, State& s, const Entry& en, const Txn& x, u32 ret)
{
    melonDS::NDS& nds = cpu->NDS;
    g_P.cpu = cpu; g_P.s = &s; g_P.ret = ret; g_P.sp = cpu->R[13]; g_P.ie = nds.IE[1]; g_P.kind = en.kind; g_P.x = x;
    g_P.entry = en.addr; g_P.t0 = nds.ARM7Timestamp; g_P.irq = false; g_P.nInner = 0; g_P.ngt = 0;
    g_P.mode = cpu->CPSR & 0x1F; g_P.chain = false;
    Snap(nds);
    CheckPending = true;
}
void ArmChain(melonDS::ARM* cpu, State& s, const St& end, const Txn& x, u64 t0)
{
    melonDS::NDS& nds = cpu->NDS;
    g_P.cpu = cpu; g_P.s = &s; g_P.chain = true; g_P.end = end; g_P.x = x; g_P.t0 = t0; g_P.ie = nds.IE[1]; g_P.ime = nds.IME[1];
    g_P.ifl = nds.IF[1]; g_P.n = 0; g_P.segi = 0; g_P.irq = false;
    g_P.tmR = nds.Timers[5].Reload; g_P.tmC = nds.Timers[5].Cnt; g_P.nInner = 0; g_P.ngt = 0;
    // (an exit hook arms at its own instruction, which the guest is about to run: that segment starts now)
    if (x.nseg && x.seg[0].pc == cpu->R[15] - 8 && x.seg[0].pc != 0x18) { g_P.segT[0] = 0; g_P.segi = 1; }
    Snap(nds);
    CheckPending = true;
}

int g_Nd = 0;
void Diff(State& s, u64 d, const char* tag, const char* what, u32 a, u32 g, u32 n)
{
    if (d < 12 && g_Nd < 10) fprintf(stderr, "A7N CHECK %s: %s %08x guest %08x native %08x\n", tag, what, a, g, n);
    g_Nd++;
}
// memory: every native write has the guest's value; the guest wrote nothing else in ARM7 / shared WRAM (but the stack below sp)
void MemDiff(melonDS::NDS& nds, State& s, u64 d, const char* tag, const Txn& x, u32 spLo)
{
    std::vector<u8> w7 = g_P.w7, sw = g_P.sw;
    for (u32 i = 0; i < x.nw; i++)
    {
        bool later = false;
        for (u32 j = i + 1; j < x.nw && !later; j++) later = x.w[j].a == x.w[i].a;
        if (later) continue;            // (only the last store to a word stays)
        u32 g;
        Word(nds, x.w[i].a, g);
        if (g != x.w[i].v) Diff(s, d, tag, "mem", x.w[i].a, g, x.w[i].v);
        u8 reg; u8* p = Host(nds, x.w[i].a, reg);
        if (p >= nds.ARM7WRAM && p < nds.ARM7WRAM + nds.ARM7WRAMSize) memcpy(&w7[p - nds.ARM7WRAM], &x.w[i].v, 4);
        else if (!sw.empty() && p >= nds.SWRAM_ARM7.Mem && p < nds.SWRAM_ARM7.Mem + sw.size()) memcpy(&sw[p - nds.SWRAM_ARM7.Mem], &x.w[i].v, 4);
    }
    const u32 spo = spLo & (nds.ARM7WRAMSize - 1);
    for (u32 o = 0; o < nds.ARM7WRAMSize; o += 4)
        if (R32(&w7[o]) != R32(&nds.ARM7WRAM[o]) && !(spLo >= 0x03800000 && o < spo && o + 0x200 >= spo))
            Diff(s, d, tag, "guest-only mem", 0x03800000 + o, R32(&nds.ARM7WRAM[o]), R32(&w7[o]));
    for (u32 o = 0; o < sw.size(); o += 4)
        if (R32(&sw[o]) != R32(&nds.SWRAM_ARM7.Mem[o]) && !(spLo < 0x03800000 && (spLo & nds.SWRAM_ARM7.Mask) > o && (spLo & nds.SWRAM_ARM7.Mask) <= o + 0x100))
            Diff(s, d, tag, "guest-only swram", 0x03000000 + o, R32(&nds.SWRAM_ARM7.Mem[o]), R32(&sw[o]));
}
void IoDiff(melonDS::NDS& nds, State& s, u64 d, const char* tag, const Txn& x, u32 ie0, u32 if0)
{
    u32 ie = ie0, ifl = if0; int rel = -1, cnt = -1;
    for (u32 i = 0; i < x.nio; i++)
    {
        const Txn::Io& o = x.io[i];
        if (o.op == 1) ie |= o.v; else if (o.op == 2) ie &= ~o.v; else if (o.op == 3) ifl &= ~o.v;
        else if (o.a == 0x04000104) rel = o.v; else if (o.a == 0x04000106) cnt = o.v;
        else if (o.a == 0x04000134 && nds.ARM7Read16(0x04000134) != (o.v & 0xFFFF)) Diff(s, d, tag, "RCNT", o.a, nds.ARM7Read16(0x04000134), o.v);
    }
    if (nds.IE[1] != ie) Diff(s, d, tag, "IE", 0x04000210, nds.IE[1], ie);
    // (a timer may overflow again after the acknowledge: a timer bit the guest has and the native acknowledged is hardware)
    const u32 ifg = nds.IF[1] & if0, ifn = ifl & if0;
    if ((ifg ^ ifn) & ~(ifg & ~ifn & 0x78)) Diff(s, d, tag, "IF", 0x04000214, nds.IF[1], ifl);
    if (rel >= 0 && nds.Timers[5].Reload != rel) Diff(s, d, tag, "TM1 reload", 0x04000104, nds.Timers[5].Reload, rel);
    if (cnt >= 0 && nds.Timers[5].Cnt != (cnt & 0xC7)) Diff(s, d, tag, "TM1CNT", 0x04000106, nds.Timers[5].Cnt, cnt);
}

void Finish(melonDS::ARM* cpu)
{
    CheckPending = false;
    melonDS::NDS& nds = cpu->NDS;
    State& s = *g_P.s;
    const u8 k = g_P.kind;
    const Txn& x = g_P.x;
    const s64 cyc = (s64)(nds.ARM7Timestamp + cpu->Cycles - g_P.t0);
    if (g_P.irq) return;
    s.checks[k]++;
    // the guest's tick reads (an OS_GetTick call: its return value; inner calls: recorded at their returns)
    u16 gt[4]; u32 ngt = 0;
    if (k == kGetTick) gt[ngt++] = (u16)cpu->R[0];
    else { ngt = g_P.ngt; memcpy(gt, g_P.gticks, sizeof(gt)); }
    if (g_Fit)
    {
        // TM0 read offsets consistent with each guest tick (the timer is linear back from its last update), TM1's start
        fprintf(stderr, "A7NFIT %s cyc=%lld native=%d walk=%d ins=%d steps=%d set=%d inc=%d dpath=%d", KindName(k), (long long)cyc, x.t,
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
    g_Nd = 0;
    const char* tag = KindName(k);
    if (x.ret && (cpu->R[0] != x.r0 || cpu->R[1] != x.r1)) Diff(s, s.diffs[k], tag, "r0/r1", 0, cpu->R[0], x.r0);
    MemDiff(nds, s, s.diffs[k], tag, x, g_P.sp);
    IoDiff(nds, s, s.diffs[k], tag, x, g_P.ie, nds.IF[1]);
    if (g_Nd) s.diffs[k]++;
}

// the chain's stop: compare the whole state
void FinishChain(melonDS::ARM* cpu)
{
    CheckPending = false;
    melonDS::NDS& nds = cpu->NDS;
    State& s = *g_P.s;
    const St& e = g_P.end;
    const Txn& x = g_P.x;
    const s64 cyc = (s64)(nds.ARM7Timestamp + cpu->Cycles - g_P.t0);
    s.chainChecks++;
    if (g_Fit)
    {
        fprintf(stderr, "A7NFIT2 cyc=%lld native=%d stop=%08x%s", (long long)cyc, x.t, e.r[15], e.ret ? "(ret)" : "");
        for (u32 i = 0; i < x.nseg; i++)
            fprintf(stderr, " %s:%lld/%d", kSegName[x.seg[i].seg], i < g_P.segi ? (long long)g_P.segT[i] : -1LL, x.seg[i].t);
        fprintf(stderr, " walk=%d ins=%d steps=%d set=%d inc=%d tsw=%d tsr=%d tsl=%d tss=%d", x.walk, x.ins, x.steps, x.set, x.inc,
                x.tsWake, x.tsRes, x.tsWalk, x.tsSw);
        for (u32 i = 0; i < g_P.ngt; i++)
        {
            int lo = -1, hi = -1;
            for (int off = 0; off < 1500; off++)
            {
                u16 v;
                if (nds.PeekTimer7(4, g_P.t0 + off, v) && v == g_P.gticks[i]) { if (lo < 0) lo = off; hi = off; }
            }
            fprintf(stderr, " tick%u=[%d,%d]@%d", i, lo, hi, i < x.nticks ? x.tickT[i] : -1);
        }
        const Timer& t1 = nds.Timers[5];
        if (x.set && (t1.Cnt & 0x80))
        {
            const s64 run = ((s64)t1.Counter - ((s64)t1.Reload << 10)) >> t1.CycleShift;
            fprintf(stderr, " tm1start=%lld", (long long)((s64)nds.TimerTimestamp7() - run - (s64)g_P.t0));
        }
        fprintf(stderr, "\n");
    }
    // the guest's TM0 reads at other times than the native's: category B timing, not compared
    bool same = g_P.ngt == x.nticks;
    for (u32 i = 0; same && i < g_P.ngt; i++) same = g_P.gticks[i] == x.ticks[i];
    if (!same) { s.chainTiming++; return; }
    // the guest's tick reads vs the native's: another time = timing (category B), not compared
    g_Nd = 0;
    const char* tag = "irq-path";
    const u64 d = s.chainDiffs;
    St g;
    Load(cpu, g, e.r[15]);
    // (the guest reached the stop from another interrupted context: it took another path, e.g. another IRQ bit came up
    // between the native's decision and the guest's: not comparable)
    if (!e.ret && g.spsr != e.spsr && g.usr13 != e.usr13) { s.chainChecks--; s.chainLost++; return; }
    // in IRQ mode at the stop (a guest function's entry, or the BIOS's return instruction): all registers and the banks
    for (int i = 0; i < 15; i++) if (cpu->R[i] != e.r[i]) Diff(s, d, tag, "reg", i, cpu->R[i], e.r[i]);
    if (g.usr13 != e.usr13 || g.usr14 != e.usr14) Diff(s, d, tag, "usr13/14", 0, g.usr13, e.usr13);
    if (g.svc13 != e.svc13) Diff(s, d, tag, "svc13", 0, g.svc13, e.svc13);
    if (g.spsr != e.spsr) Diff(s, d, tag, "spsr", 0, g.spsr, e.spsr);
    if (!e.ret && (g.cpsr & 0xDF) != (e.cpsr & 0xDF)) Diff(s, d, tag, "cpsr", 0, g.cpsr, e.cpsr);
    MemDiff(nds, s, d, tag, x, e.r[13]);       // (the IRQ stack below the stop's sp: the guest's callees' dead frames)
    IoDiff(nds, s, d, tag, x, g_P.ie, g_P.ifl);
    if (g_Nd)
    {
        // (a TM0 read at another time changes what follows: count that apart)
        s.chainDiffs++;
        if (s.chainDiffs <= 12)
        {
            fprintf(stderr, "A7N CHECK irq-path: stop %08x%s segs", e.r[15], e.ret ? " (ret)" : "");
            for (u32 i = 0; i < x.nseg; i++) fprintf(stderr, " %s@%08x", kSegName[x.seg[i].seg], x.seg[i].pc);
            fprintf(stderr, "\n");
        }
    }
}
#endif

// a native run from pc (the vector or an exit hook): stepped, applied; false if nothing ran
bool RunChain(melonDS::ARM* cpu, u32 pc, u8 want, bool interp)
{
    melonDS::NDS& nds = cpu->NDS;
    State& s = Get(nds);
    St st;
    Txn x;
    const u64 t0 = nds.ARM7Timestamp + (interp ? cpu->Cycles : 0);
    // (TM0 must not overflow within the run: every OS_GetTick on it then reads it natively, so nothing fails after IO)
    u16 tm0;
    if (!nds.PeekTimer7(4, interp ? t0 + 4000 : t0, tm0)) { s.stops[kSegs]++; return false; }
#ifdef LITEV_HLE_DIAG
    x.dry = g_Check && interp;
#endif
    int n;
    { A7N_PMU(3); n = Chain(cpu, s, pc, want, st, x, interp, t0); }
    if (!n || (s.wholeOn == 1 && !st.ret))
    {
        if (n && !x.dry) Undo(nds, x, 0);      // (whole runs: a run that stops in guest code leaves no trace; its IO not issued)
        s.stops[kSegs]++;
        return false;
    }
    {
        // the shadowed IRQ stack: the words at or above the final IRQ sp are live (frames of a stop inside the path)
        Env none;
        Ctx c{nds, none, x, t0, interp};
        const u32 sp = st.r[13];
        x.shLo = x.shTop = 0;
        for (u32 k = 0; k < kShWin / 256; k++)
            for (u64 m = x.shm[k]; m; m &= m - 1)
        {
            const u32 i = 64 * k + (u32)__builtin_ctzll(m), a = x.shMin + 4 * i, v = x.shv[i];
            s.shW++;
            if (a < sp) continue;
            s.shMat++;
            if (x.dry) { Wr(c, a, v); continue; }
            u8 reg;
            u8* p = Host(nds, a, reg);          // (valid: Chain checked the window)
            if (R32(p) != v) { Inval(nds, a, reg); memcpy(p, &v, 4); }
        }
    }
#ifdef LITEV_HLE_DIAG
    if (g_Check && interp)
    {
        ArmChain(cpu, s, st, x, t0);
        return false;                   // the guest runs; compared at the stop
    }
#endif
    s.stops[x.nseg ? x.seg[x.nseg - 1].seg : kSegs]++;
#if defined(__APPLE__) && defined(LITEV_HLE_DIAG)
    if (!st.ret) g_LastStop = st.r[15];
#endif
    { A7N_PMU(4); Commit(cpu, x, interp, t0); }
    { A7N_PMU(5); Apply(cpu, st); }
    cpu->Cycles += x.t;
    return true;
}
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
        {
            bool gt = false;
            for (const Entry& e : s.ents) gt |= e.addr == t && e.kind == kGetTick;
            for (const Fn& f : s.fns) gt |= f.e.ka[kGetTick] == t && (f.e.has >> f_tc & 1);
            if (gt) g_P.innerRet[g_P.nInner++] = r15 - 4;
        }
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
        x.dry = true;
        if (Native(cpu, *en, x, true)) Arm(cpu, s, *en, x, r15 - 4);
        return false;           // the guest runs
    }
#endif
    Call(cpu, t, r15 - 4, true);
    return true;
}

bool VecCand(melonDS::NDS& nds);
void Irq(melonDS::ARM* cpu)
{
    if (cpu->Num != 1 || CheckPending) return;
    State& s = Get(cpu->NDS);
    if (!IrqOn(s)) return;
    if (s.preSet)
    {
        // (IrqPre ran this IRQ: counted, guards checked; 0 = it could not apply: the usual path)
        s.preSet = false;
        if (s.preR < 0) { s.stops[kSegs]++; return; }
        s.irqs--;
    }
    s.irqs++;
    if (s.reguard)
    {
        // a function of the IRQ path verified since the thread switch hook was compiled: recompile it (between blocks)
        s.reguard = false;
        u8 reg;
        if (s.tsBlock && cpu->NDS.IsJITEnabled() && Host(cpu->NDS, s.tsHook, reg)) Inval(cpu->NDS, s.tsHook, reg);
    }
#ifdef LITEV_HLE_DIAG
    // LITEV_A7N_MEMDUMP=<prefix>: main RAM + the ARM7's shared / own WRAM at the 2000th IRQ (LITEV_A7N_MEMDUMP_AT) as <prefix>.mem (the A7PROF
    // layout: tools/hle/a7sig.py finds a new SDK build's functions in it)
    static const u64 dumpAt = getenv("LITEV_A7N_MEMDUMP_AT") ? strtoull(getenv("LITEV_A7N_MEMDUMP_AT"), nullptr, 0) : 2000;
    if (s.irqs == dumpAt)
        if (const char* d = getenv("LITEV_A7N_MEMDUMP"))
            if (FILE* f = fopen((std::string(d) + ".mem").c_str(), "wb"))
            {
                melonDS::NDS& nds = cpu->NDS;
                fwrite(nds.MainRAM, 1, 0x400000, f);
                for (u32 a = 0x037F8000; a < 0x03810000; a += 4) { u32 v = 0; Word(nds, a, v); fwrite(&v, 4, 1, f); }
                fclose(f);
            }
#endif
    {
        if (WholeOn(s))
        {
            // (whole runs: only the TM1 alarm IRQ, the lowest pending bit, runs through to the BIOS return natively)
            melonDS::NDS& nds = cpu->NDS;
            const u32 pend = nds.IME[1] ? nds.IE[1] & nds.IF[1] : 0;
            if ((pend & (0u - pend)) != (1u << melonDS::IRQ_Timer1)) { s.stops[kSegs]++; return; }
            u16 tm0;
            if (FastOn(s) && nds.IsJITEnabled() && nds.PeekTimer7(4, nds.ARM7Timestamp, tm0))
            {
                A7N_PMU(0);
                A7N_CP(0);
                St in;
                Load(cpu, in, cpu->R[14] - 4);
                const int r = TryFast(cpu, s, in, false);
                if (r > 0) { A7N_CP(12); s.irqNative++; return; }
                if (r < 0) { s.stops[kSegs]++; return; }
            }
        }
        // (a vector target that is no SDK IRQ handler, e.g. a library's nested IRQ handler: nothing to step, skip the run)
        if (!VecCand(cpu->NDS)) { s.stops[kSegs]++; return; }
        A7N_PMU(0);
        if (RunChain(cpu, 0x18, wBiosIn, !cpu->NDS.IsJITEnabled())) s.irqNative++;
    }
#if defined(__APPLE__) && defined(LITEV_HLE_DIAG)
    if (g_PmuLvl == 4 && g_LastStop) { g_GapPc = g_LastStop; g_LastStop = 0; u64 c[2]; thread_selfcounts(1, c, sizeof(c)); g_Gap0 = c[0]; }
#endif
}

bool IrqPre(melonDS::ARM* cpu)
{
    if (CheckPending) return false;
    State& s = Get(cpu->NDS);
    melonDS::NDS& nds = cpu->NDS;
    if (!IrqOn(s) || !PreOn(s) || !nds.IsJITEnabled() || ((cpu->CPSR & 0x1F) != 0x10 && (cpu->CPSR & 0x1F) != 0x1F)) return false;
    const u32 pend = nds.IME[1] ? nds.IE[1] & nds.IF[1] : 0;
    u16 tm0;
    if ((pend & (0u - pend)) != (1u << melonDS::IRQ_Timer1) || !nds.PeekTimer7(4, nds.ARM7Timestamp, tm0)) return false;
    s.irqs++;
    if (s.reguard)
    {
        s.reguard = false;
        u8 reg;
        if (s.tsBlock && Host(nds, s.tsHook, reg)) Inval(nds, s.tsHook, reg);
    }
    A7N_PMU(0);
    A7N_CP(0);
    // the state at the vector entry (IRQ mode view), without entering it
    St in;
    for (int i = 0; i < 13; i++) in.r[i] = cpu->R[i];
    in.r[13] = cpu->R_IRQ[0]; in.r[14] = cpu->R[15] + (cpu->CPSR & 0x20 ? 2 : 0); in.r[15] = in.r[14] - 4;
    in.usr13 = cpu->R[13]; in.usr14 = cpu->R[14]; in.svc13 = cpu->R_SVC[0]; in.spsr = cpu->CPSR;
    in.cpsr = (cpu->CPSR & ~0xFFu) | 0xD2; in.ret = false;
    const int r = TryFast(cpu, s, in, true);
    if (r > 0) { A7N_CP(12); s.irqNative++; return true; }
    s.preSet = true; s.preR = r;
    return false;
}

// the IRQ vector's target may be an SDK OS_IrqHandler (its first word)
bool VecCand(melonDS::NDS& nds)
{
    u32 h, w0;
    if (!Word(nds, 0x03FFFFFC, h) || !Word(nds, h, w0)) return false;
    for (int j = 0; j < kByKind.n[kIrqHandler]; j++) if (kSigs[kByKind.i[kIrqHandler][j]].w0 == w0) return true;
    return false;
}

bool MaybeExit(u32 instr)
{
    for (int j = 0; j < kByKind.n[kThreadSwitch]; j++) if (kSigs[kByKind.i[kThreadSwitch][j]].w0 == instr) return true;
    for (int j = 0; j < kByKind.n[kIrqCallback]; j++) if (kSigs[kByKind.i[kIrqCallback][j]].pw == instr) return true;
    for (int j = 0; j < kByKind.n[kArrangeTimer]; j++) if (kSigs[kByKind.i[kArrangeTimer][j]].pw == instr) return true;
    return false;
}

bool HookGuardOnly(melonDS::NDS& nds) { return WholeOn(Get(nds)); }
int IsHook(melonDS::NDS& nds, u32 addr, u32 instr, u32* a, u32* b, int max)
{
    State& s = Get(nds);
    if (!IrqOn(s) || !MaybeExit(instr)) return 0;
    Rng d[kMaxGuard];
    for (const A7NSig& g : kSigs)
    {
        u32 at;
        // (the thread switch only while the IRQ vector is an SDK OS_IrqHandler: a library's nested IRQ handler (Brain Age,
        // Zelda PH) enters it on another stack layout, where the native run stops right after it: slower than the guest)
        if (g.kind == kThreadSwitch && g.w0 == instr) { if (!VecCand(nds)) continue; at = addr; }
        else if ((g.kind == kIrqCallback || g.kind == kArrangeTimer) && g.pw == instr)
        {
            u32 w;
            if (!Word(nds, addr - 8, w) || w != kMovLrPc) continue;
            at = addr - g.post;
        }
        else continue;
        Env e;
        int nd = 0;
        if (!Verify(nds, at, g.kind, e, d, nd, 0) || e.kf[g.kind] != g.fam || nd > max) continue;
        if (g.kind == kThreadSwitch) s.tsHook = addr;
        if (g.kind != kThreadSwitch)
        {
            // (after a callback the native run goes on through the thread switch to the BIOS return: only worth a hook
            // when that IRQ handler is one of ours too; else it would stop right after a dozen instructions)
            u32 h;
            Env eh;
            Rng dh[kMaxDeps];
            int nh = 0;
            if (!Word(nds, 0x03FFFFFC, h) || !Verify(nds, h, kIrqHandler, eh, dh, nh, 0)) continue;
        }
        State::Cover cv{addr, {{at, g.kind}}};
        if (g.kind == kArrangeTimer) cv.fn.push_back({e.f[f_ah], kAlarmWrap});
        if (g.kind == kThreadSwitch)
        {
            // the IRQ handler whose thread switch this is (the IRQ vector's target now): guarded by this block too
            u32 h;
            Env eh;
            int n2 = nd;
            if (Word(nds, 0x03FFFFFC, h) && Verify(nds, h, kIrqHandler, eh, d, n2, 0) && eh.f[f_ts] == addr && n2 <= max)
            { nd = n2; cv.fn.push_back({h, kIrqHandler}); }
            // ... and every other function of the IRQ path verified so far (unchanged since): the alarm IRQs run natively
            // from the vector to the BIOS return without passing another hook
            for (const Fn& f : s.fns)
            {
                if (f.kind == kThreadSwitch || f.kind == kIrqHandler || !Same(nds, f)) continue;
                int n3 = nd;
                for (const Rng& r : f.rs)
                {
                    bool dup = false;
                    for (int j = 0; j < n3 && !dup; j++) dup = d[j].a == r.a && d[j].b == r.b;
                    if (!dup && n3 < max && n3 < kMaxGuard) d[n3++] = r;
                    else if (!dup) { n3 = -1; break; }
                }
                if (n3 < 0) continue;
                nd = n3;
                cv.fn.push_back({f.addr, f.kind});
            }
        }
        for (size_t i = 0; i < s.cover.size(); i++) if (s.cover[i].hook == addr) { s.cover.erase(s.cover.begin() + i); break; }
        s.cover.push_back(cv);
        for (int i = 0; i < nd; i++) { a[i] = d[i].a; b[i] = d[i].b; }
        return nd;
    }
    return 0;
}

void HookCompiled(melonDS::NDS& nds, u32 addr, const void* block)
{
    State& s = Get(nds);
    for (size_t i = 0; i < s.cover.size(); i++)
    {
        if (s.cover[i].hook != addr) continue;
        const State::Cover cv = s.cover[i];
        s.cover.erase(s.cover.begin() + i);
        s.run++;
        if (addr == s.tsHook) s.tsBlock = nullptr;     // (Chk below: a function verified now needs no recompile)
        for (const auto& [a, k] : cv.fn)
            if (const Fn* f = Chk(nds, s, a, k)) const_cast<Fn*>(f)->guard = block;
        if (addr == s.tsHook) { s.tsBlock = block; s.reguard = false; }
        return;
    }
}
void BlockGone(melonDS::NDS& nds, const void* block)
{
    State& s = Get(nds);
    for (Fn& f : s.fns) if (f.guard == block) { f.guard = nullptr; s.gen++; }
    if (s.tsBlock == block) s.tsBlock = nullptr;
}
void GuardsReset(melonDS::NDS& nds)
{
    State& s = Get(nds);
    for (Fn& f : s.fns) f.guard = nullptr;
    s.gen++;
    s.tsBlock = nullptr;
}

bool Hook(melonDS::ARM* cpu, bool jit)
{
    if (cpu->Num != 1 || (cpu->CPSR & 0x20) || CheckPending || !MaybeExit(cpu->CurInstr)) return false;
    State& s = Get(cpu->NDS);
    if (!IrqOn(s) || (WholeOn(s) && !jit)) return false;
    const u32 pc = cpu->R[15] - 8, w = cpu->CurInstr;
    u8 want = wNone;
    for (const A7NSig& g : kSigs)
        if (g.kind == kThreadSwitch && g.w0 == w) want = wTs;
        else if (g.kind == kIrqCallback && g.pw == w) want = wIcbPost;
        else if (g.kind == kArrangeTimer && g.pw == w) want = wArrPost;
#if defined(__APPLE__) && defined(LITEV_HLE_DIAG)
    if (g_PmuLvl == 4 && g_Gap0) { u64 c[2]; thread_selfcounts(1, c, sizeof(c)); g_Gaps[g_GapPc].push_back(c[0] - g_Gap0); g_Gap0 = 0; }
#endif
    if (want == wTs && jit && !VecCand(cpu->NDS)) s.reguard = true;     // (recompile the block without the hook: see IsHook)
    A7N_PMU(1);
    if (want != wNone && (want != wTs || !s.reguard) && RunChain(cpu, pc, want, !jit))
    {
        s.exits++;
        return true;
    }
    if (!jit) return false;
    // (the JIT compiled the hook as an undefined instruction: run the real one)
    if (cpu->CheckCondition(cpu->CurInstr >> 28))
        ARMInterpreter::ARMInstrTable[((cpu->CurInstr >> 4) & 0xF) | ((cpu->CurInstr >> 16) & 0xFF0)](cpu);
    else cpu->AddCycles_C();
    return true;
}

#ifdef LITEV_HLE_DIAG
void CheckAt(melonDS::ARM* cpu, u32 pc)
{
    if (cpu != g_P.cpu) return;
    melonDS::NDS& nds = cpu->NDS;
    if (g_P.chain)
    {
        // the guest reaches the native's segments in order: their times (cycle fits)
        if (g_P.segi < g_P.x.nseg && pc == g_P.x.seg[g_P.segi].pc)
            g_P.segT[g_P.segi++] = (s64)(nds.ARM7Timestamp + cpu->Cycles - g_P.t0);
        if (g_P.nInner && pc == g_P.innerRet[g_P.nInner - 1])
        {
            if (g_P.ngt < 4) g_P.gticks[g_P.ngt++] = (u16)cpu->R[0];
            g_P.nInner--;
        }
        const St& e = g_P.end;
        const bool irq = (cpu->CPSR & 0x1F) == 0x12;
        // (a return to the interrupted code is compared at the BIOS's subs pc, lr, #4: right after it, another pending IRQ
        // may enter the handler again before the interrupted code runs an instruction)
        if (irq && cpu->R[13] == e.r[13] && (e.ret ? pc == g_P.s->biosRet + 4 : pc == (e.r[15] & ~1u))) FinishChain(cpu);   // (a Thumb stop: bit 0)
        else if (++g_P.n > 50000) { CheckPending = false; g_P.s->chainLost++; }
        return;
    }
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
