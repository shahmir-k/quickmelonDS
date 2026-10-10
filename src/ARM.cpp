/*
    Copyright 2016-2026 melonDS team

    This file is part of melonDS.

    melonDS is free software: you can redistribute it and/or modify it under
    the terms of the GNU General Public License as published by the Free
    Software Foundation, either version 3 of the License, or (at your option)
    any later version.

    melonDS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with melonDS. If not, see http://www.gnu.org/licenses/.
*/

#include <stdio.h>
#include <assert.h>
#include <algorithm>
#include "NDS.h"
#include "DSi.h"
#include "ARM.h"
#include "ARMInterpreter.h"
#include "AREngine.h"
#include "ARMJIT.h"
#include "Platform.h"
#include "GPU.h"
#include "ARMJIT_Memory.h"
#include "LiteProfile.h"
#ifdef LITEV_JIT_IDLE2
#include <unordered_map>
#include "ARM_InstrInfo.h"
#if defined(__ANDROID__)
#include <sys/system_properties.h>
#endif
#endif

namespace melonDS
{
using Platform::Log;
using Platform::LogLevel;

#if defined(LITEV_SHADOW_ASSERT)
// liteDS-v2 Unit 2 shadow-mode equivalence check.
//
// Called right after each ARM_Dispatch/CompileBlock returns, while <timestamp> is
// still the PRE-increment value (the loop does `Timestamp += Cycles` immediately
// after). It proves that the not-yet-active budget mechanism (Unit 3) would make
// the exact same "is this slice over?" decision as the current timestamp/target
// compare.
//
// Anchoring / units: the loop's identity is `Timestamp += Cycles`, so <cycles> is,
// by construction, in Timestamp units (for ARM9 that is bus-clock << ARM9ClockShift;
// for ARM7 it is bus-clock, no shift). The budget is captured before dispatch as
// `min(Target - Timestamp, INT32_MAX)`, i.e. in those same units. Hence
// algebraically  (Cycles >= budget)  ==  (Timestamp + Cycles >= Target).
//
// Invariant handling of the two edge conditions:
//  * budget == 0  => ForceExecutionExit() fired mid-block. The dispatch loop only
//    ever enters with Timestamp < Target, so the budget captured before dispatch
//    was strictly > 0; a 0 here therefore MUST be a forced exit. Unit 3's dispatcher
//    treats budget<=cycles as "exit", which is trivially true (Cycles >= 0), so the
//    forced exit is always honoured. There is no timestamp-side obligation in this
//    case (the forcing event -- reschedule/IRQ/halt/DMA/GXstall -- may or may not
//    have advanced Timestamp past Target), so we do not compare. This is the ONLY
//    case where the two sides may legitimately differ.
//  * budget > 0  => no ForceExecutionExit fired. Mid-slice, Target only ever moves
//    EARLIER (NDS::Reschedule), and that path zeroes the budget; so a positive
//    budget also proves Target is unchanged since capture. The equivalence is then
//    exact and asserted. (If the pre-dispatch difference exceeded INT32_MAX the
//    budget is clamped, but a single block never consumes ~2^31 cycles, so both
//    sides read false and still agree -- benign, and it does not occur in the suite.)
static void LiteV_ShadowAssertBudget(const char* who, s32 cycles, s32 budget,
                                     s64 timestamp, s64 target)
{
    if (budget <= 0)
        return; // forced exit (budget zeroed) or slice already consumed; no timestamp
                // obligation. Non-dispatch builds never make the budget negative, so this
                // is identical to `== 0` there; the Unit 3 dispatcher maintains the budget
                // as `budget -= Cycles` per chained block, which can drive a
                // ForceExecutionExit()-zeroed budget below zero.

    bool budgetExit = (cycles >= budget);
    bool tsExit = (timestamp + (s64)cycles >= target);
    if (budgetExit != tsExit)
    {
        fprintf(stderr,
            "[LITEV_SHADOW_ASSERT] %s budget<->timestamp divergence: "
            "cycles=%d budget=%d timestamp=%lld target=%lld "
            "(budgetExit=%d tsExit=%d)\n",
            who, cycles, budget, (long long)timestamp, (long long)target,
            (int)budgetExit, (int)tsExit);
        fflush(stderr);
        abort();
    }
}
#endif

#ifdef GDBSTUB_ENABLED
void ARM::GdbCheckA()
{
    if (!IsSingleStep && !BreakReq)
    { // check if eg. break signal is incoming etc.
        Gdb::StubState st = GdbStub.Enter(false, Gdb::TgtStatus::NoEvent, ~(u32)0u, BreakOnStartup);
        BreakOnStartup = false;
        IsSingleStep = st == Gdb::StubState::Step;
        BreakReq = st == Gdb::StubState::Attach || st == Gdb::StubState::Break;
    }
}
void ARM::GdbCheckB()
{
    if (IsSingleStep || BreakReq)
    { // use else here or we single-step the same insn twice in gdb
        u32 pc_real = R[15] - ((CPSR & 0x20) ? 2 : 4);
        Gdb::StubState st = GdbStub.Enter(true, Gdb::TgtStatus::SingleStep, pc_real);
        IsSingleStep = st == Gdb::StubState::Step;
        BreakReq = st == Gdb::StubState::Attach || st == Gdb::StubState::Break;
    }
}
void ARM::GdbCheckC()
{
    u32 pc_real = R[15] - ((CPSR & 0x20) ? 2 : 4);
    Gdb::StubState st = GdbStub.CheckBkpt(pc_real, true, true);
    if (st != Gdb::StubState::CheckNoHit)
    {
        IsSingleStep = st == Gdb::StubState::Step;
        BreakReq = st == Gdb::StubState::Attach || st == Gdb::StubState::Break;
    }
    else GdbCheckB();
}
#else
void ARM::GdbCheckA() {}
void ARM::GdbCheckB() {}
void ARM::GdbCheckC() {}
#endif


// instruction timing notes
//
// * simple instruction: 1S (code)
// * LDR: 1N+1N+1I (code/data/internal)
// * STR: 1N+1N (code/data)
// * LDM: 1N+1N+(n-1)S+1I
// * STM: 1N+1N+(n-1)S
// * MUL/etc: 1N+xI (code/internal)
// * branch: 1N+1S (code/code) (pipeline refill)
//
// MUL/MLA seems to take 1I on ARM9



const u32 ARM::ConditionTable[16] =
{
    0xF0F0, // EQ
    0x0F0F, // NE
    0xCCCC, // CS
    0x3333, // CC
    0xFF00, // MI
    0x00FF, // PL
    0xAAAA, // VS
    0x5555, // VC
    0x0C0C, // HI
    0xF3F3, // LS
    0xAA55, // GE
    0x55AA, // LT
    0x0A05, // GT
    0xF5FA, // LE
    0xFFFF, // AL
    0x0000  // NE
};

ARM::ARM(u32 num, bool jit, std::optional<GDBArgs> gdb, melonDS::NDS& nds) :
#ifdef GDBSTUB_ENABLED
    GdbStub(this),
    BreakOnStartup(false),
#endif
    Num(num), // well uh
    NDS(nds)
{
    SetGdbArgs(jit ? std::nullopt : gdb);
}

ARM::~ARM()
{
    // dorp
}

ARMv5::ARMv5(melonDS::NDS& nds, std::optional<GDBArgs> gdb, bool jit) : ARM(0, jit, gdb, nds)
{
    DTCM = NDS.JIT.Memory.GetARM9DTCM();

    PU_Map = PU_PrivMap;
}

#ifdef LITEV_JIT_IDLE2
// ---------------------------------------------------------------------------------------
// LITEV_JIT_IDLE2: generalized ARM9 wait-loop skipping.
//
// Host time scales with guest instructions, and a frame has a fixed cycle budget, so a
// guest spinning in a wait loop the JIT does not recognise burns host time for nothing.
// The JIT marks the conditional back-edges of loops it cannot prove idle statically (they
// store to memory, or they leave the block through calls) as branch_Idle2Cand. A taken
// candidate back-edge exits to C++ (IdleLoop = 2) while the site's enable byte is set.
// Idle2Handle then interprets two iterations, recording every data access, and proves
// with a taint pass that the loop waits on state only an event can change:
//   * both iterations take the same path (same instructions, same condition outcomes);
//   * values read from timers / DIV-SQRT results (TIME) or from memory the previous
//     iteration stored (CARRY) never reach a condition, a PC write, an MSR or an address:
//     they may only be stored (last writer wins);
//   * no CARRY-derived value is stored to an address the loop reads before writing (an
//     accumulator such as `n++` in memory is rejected; `last = now` is accepted);
//   * the registers live into the loop head are unchanged by an iteration (no counters);
//   * the loop writes no IO except the DIV/SQRT engine and reads only IO without read
//     side effects (others are rejected).
// What the iteration reads from outside itself (live-in registers, CPSR, memory the loop
// does not write) becomes the site's memo. Within an ARM9 slice nothing but the ARM9
// itself changes that state (IRQs, DMA, the ARM7 and timer overflows all happen at
// events, i.e. at slice boundaries), so the loop cannot exit before ARM9Target and the
// CPU skips to it -- exactly like the classic melonDS idle loop. Later arrivals re-check
// the memo (registers + inputs re-read) instead of tracing again.
//
// Approximate, deterministic: like the classic idle loop, the skipped iterations do not
// run, so loop-stored time-derived values (e.g. a stopwatch's `last` tick, the DIV
// registers) keep the value of the last executed iteration until the next one refreshes
// them. The per-site state (proofs, memos, rejections) lives exactly as long as the JIT's
// compiled blocks: it is cleared by ARMJIT::ResetBlockCache (savestate load, reset), and
// rejected sites are retried after a back-off counted in frames from there. So it depends on
// the same history as the JIT itself (whose compile-time cycle guesses already do).
// ---------------------------------------------------------------------------------------

bool LiteIdle2On()
{
    static const bool on = [] {
#if defined(__ANDROID__)
        char b[PROP_VALUE_MAX] = {0}; int n = __system_property_get("debug.litev.idle2", b);
        return n > 0 ? atoi(b) != 0 : true;
#else
        const char* e = getenv("debug.litev.idle2"); return e ? atoi(e) != 0 : true;
#endif
    }();
    return on;
}

struct Idle2Input { u32 Addr, Val; u8 Size; };
struct Idle2SiteInfo
{
    bool Memo = false;
    u8 Tries = 0;
    u8 Backoff = 0;          // consecutive give-ups; disabled for 16 << Backoff frames
    u8 Hits = 0;             // arrivals before the first trace (rarely taken back-edges are never traced)
    u32 DisabledUntil = 0;
    u16 Live = 0;
    u32 Head = 0, CPSR = 0;
    u32 Regs[16] = {};
    std::vector<Idle2Input> Inputs;
};
// Byte address -> (taint, iteration) of the last store in a trace; open addressing,
// cleared per trace by a generation stamp.
struct Idle2ByteMap
{
    static constexpr u32 N = 1024;
    u32 Key[N], Gen[N] = {}, Cur = 0, Count = 0;
    u64 Taint[N];
    u8 Iter[N];
    void Clear() { Cur++; Count = 0; }
    int Find(u32 k) const
    {
        for (u32 h = (k * 0x9E3779B1u) >> 22;; h = (h + 1) & (N - 1))
        {
            if (Gen[h] != Cur) return -1;
            if (Key[h] == k) return (int)h;
        }
    }
    int Insert(u32 k)
    {
        for (u32 h = (k * 0x9E3779B1u) >> 22;; h = (h + 1) & (N - 1))
        {
            if (Gen[h] != Cur) { Gen[h] = Cur; Key[h] = k; Count++; return (int)h; }
            if (Key[h] == k) return (int)h;
        }
    }
};

struct Idle2State
{
    // ponytail: fixed slot table (one byte per back-edge site, never recycled); a game with
    // more candidate sites just stops getting new ones.
    static constexpr u32 MaxSites = 8192;
    u8 Enable[MaxSites];
    std::unordered_map<u32, u32> Index;   // (branch addr | thumb) -> slot
    std::vector<Idle2SiteInfo> Sites;
    std::vector<ARMv5::Idle2Access> Log;
    Idle2ByteMap Stores;
    u64 Traces = 0, Accepts = 0, Rejects = 0, Skips = 0, Frames = 0, Steps = 0;
};

u8* ARMv5::Idle2EnableByte(u32 site)
{
    if (!Idle2)
    {
        Idle2 = new Idle2State();
        memset(Idle2->Enable, 1, sizeof(Idle2->Enable));
    }
    auto it = Idle2->Index.find(site);
    if (it == Idle2->Index.end())
    {
        if (Idle2->Sites.size() >= Idle2State::MaxSites)
            return nullptr;
        it = Idle2->Index.emplace(site, (u32)Idle2->Sites.size()).first;
        Idle2->Sites.emplace_back();
    }
    return &Idle2->Enable[it->second];
}

void ARMv5::Idle2Reset()
{
    if (!Idle2)
        return;
    memset(Idle2->Enable, 1, sizeof(Idle2->Enable));
    Idle2->Index.clear();
    Idle2->Sites.clear();
    Idle2->Frames = 0;
}

void ARMv5::Idle2Frame()
{
    if (!Idle2)
        return;
    ++Idle2->Frames;
    for (size_t i = 0; i < Idle2->Sites.size(); i++)
    {
        // give a given-up site another chance once its back-off ran out (the loop may be
        // waiting now)
        Idle2SiteInfo& s = Idle2->Sites[i];
        if (!Idle2->Enable[i] && Idle2->Frames >= s.DisabledUntil)
        {
            Idle2->Enable[i] = 1;
            s.Tries = 0;
        }
    }
    static const bool stats = getenv("LITEV_IDLE2_STATS") != nullptr;
    if (stats && Idle2->Frames % 300 == 0)
    {
        printf("idle2 300f: sites=%zu traces=%llu steps=%llu accepts=%llu rejects=%llu skips=%llu\n", Idle2->Sites.size(),
            (unsigned long long)Idle2->Traces, (unsigned long long)Idle2->Steps, (unsigned long long)Idle2->Accepts,
            (unsigned long long)Idle2->Rejects, (unsigned long long)Idle2->Skips);
        Idle2->Traces = Idle2->Accepts = Idle2->Rejects = Idle2->Skips = Idle2->Steps = 0;
    }
}

u32 ARMv5::Idle2Peek(u32 addr, int size)
{
    u8* p = nullptr;
    if (addr < ITCMSize) p = &ITCM[addr & (ITCMPhysicalSize - 1)];
    else if ((addr & DTCMMask) == DTCMBase) p = &DTCM[addr & (DTCMPhysicalSize - 1)];
    if (p)
        return size == 1 ? *p : size == 2 ? *(u16*)p : *(u32*)p;
    return size == 1 ? BusRead8(addr) : size == 2 ? BusRead16(addr) : BusRead32(addr);
}

// Address operand registers of a load/store (base, register offset); 0 for others.
static u16 Idle2AddrRegs(bool thumb, u32 i)
{
    if (thumb)
    {
        if ((i & 0xF800) == 0x4800) return 1 << 15;                                  // LDR PC-rel
        if ((i & 0xF000) == 0x5000) return (1 << ((i >> 3) & 7)) | (1 << ((i >> 6) & 7)); // reg offset
        if ((i & 0xE000) == 0x6000 || (i & 0xF000) == 0x8000) return 1 << ((i >> 3) & 7); // imm offset
        if ((i & 0xF000) == 0x9000 || (i & 0xF600) == 0xB400) return 1 << 13;      // SP-rel, PUSH/POP
        if ((i & 0xF000) == 0xC000) return 1 << ((i >> 8) & 7);                     // LDMIA/STMIA
        return 0;
    }
    const u16 rn = 1 << ((i >> 16) & 0xF);
    if ((i & 0x0C000000) == 0x04000000) return rn | ((i & (1 << 25)) ? 1 << (i & 0xF) : 0); // LDR/STR
    if ((i & 0x0E000000) == 0x08000000) return rn;                                          // LDM/STM
    if ((i & 0x0E000090) == 0x00000090 && (i & 0x60)) return rn | ((i & (1 << 22)) ? 0 : 1 << (i & 0xF)); // H/SB/D
    if ((i & 0x0FB000F0) == 0x01000090) return rn;                                          // SWP
    return 0;
}

// ARM9 IO read class: 0 = no read side effects and only changed at events (an input),
// 1 = time-derived (timer counters), 2 = DIV/SQRT engine, -1 = not allowed in a wait loop.
static int Idle2IOClass(u32 o)
{
    if (o < 0x8) return 0;                          // DISPCNT, DISPSTAT, VCOUNT
    if (o >= 0x100 && o < 0x110) return (o & 2) ? 0 : 1; // TMxCNT_H / TMxCNT_L
    if (o >= 0x130 && o < 0x134) return 0;          // KEYINPUT, KEYCNT
    if (o >= 0x180 && o < 0x188) return 0;          // IPCSYNC, IPCFIFOCNT
    if (o >= 0x204 && o < 0x218) return 0;          // EXMEMCNT, IME, IE, IF
    if (o >= 0x240 && o < 0x24A) return 0;          // VRAMCNT, WRAMCNT
    if (o >= 0x280 && o < 0x2C0) return 2;          // DIV/SQRT
    if (o >= 0x300 && o < 0x308) return 0;          // POSTFLG, POWCNT1
    return -1;
}

// One interpreter step, as ARMv5::Execute<Interpreter> does it; false = condition failed.
bool ARMv5::Idle2Step()
{
    bool pass = true;
    if (CPSR & 0x20)
    {
        if ((NextInstr[0] & 0xF000) == 0xD000 && (NextInstr[0] & 0x0E00) != 0x0E00)
            pass = CheckCondition((NextInstr[0] >> 8) & 0xF); // Thumb B<cond>
        R[15] += 2;
        CurInstr = NextInstr[0];
        NextInstr[0] = NextInstr[1];
        if (R[15] & 0x2) { NextInstr[1] >>= 16; CodeCycles = 0; }
        else NextInstr[1] = CodeRead32(R[15], false);
        ARMInterpreter::THUMBInstrTable[(CurInstr >> 6) & 0x3FF](this);
    }
    else
    {
        R[15] += 4;
        CurInstr = NextInstr[0];
        NextInstr[0] = NextInstr[1];
        NextInstr[1] = CodeRead32(R[15], false);
        if (CheckCondition(CurInstr >> 28))
            ARMInterpreter::ARMInstrTable[((CurInstr >> 4) & 0xF) | ((CurInstr >> 16) & 0xFF0)](this);
        else if ((CurInstr & 0xFE000000) == 0xFA000000)
            ARMInterpreter::A_BLX_IMM(this);
        else
        {
            pass = false;
            AddCycles_C();
        }
    }
    NDS.ARM9Timestamp += Cycles;
    Cycles = 0;
    return pass;
}

// Is there a compiled block at the next instruction? (Resuming the JIT anywhere else
// would compile a new block entry.)
bool ARMv5::Idle2AtBlock()
{
    const u32 a = R[15] - ((CPSR & 0x20) ? 2 : 4);
    if (a < FastBlockLookupStart || a >= FastBlockLookupStart + FastBlockLookupSize)
    {
#ifdef LITEV_JIT_REGION_CACHE
        if (!JitSetupRegion(a))
#else
        if (!NDS.JIT.SetupExecutableRegion(0, a, FastBlockLookup, FastBlockLookupStart, FastBlockLookupSize))
#endif
            return true;
    }
    return NDS.JIT.LookUpBlock(0, FastBlockLookup, a - FastBlockLookupStart, a) != nullptr;
}

// Interpret two iterations of the loop starting at `head` (the CPU is there now) and
// decide. 1 = wait loop (memo stored, the CPU is at `head` again), 0 = not one,
// -1 = undecided (left the loop, IRQ, end of slice, iterations differ).
int ARMv5::Idle2Trace(u32 head, Idle2SiteInfo& s)
{
    // Taint: bit 0 = TIME, bit 1+i = depends on loop-carried cell i (a word iteration 2
    // read from iteration 1's stores before writing it itself).
    enum : u64 { T_TIME = 1 };
    Idle2ByteMap& stores = Idle2->Stores;    // byte -> taint and iteration of its last store
    stores.Clear();
    u32 cellAddr[63];                        // carried words; index = cell
    int cells = 0;
    auto cellOf = [&](u32 w) { for (int c = 0; c < cells; c++) if (cellAddr[c] == w) return c; return -1; };
    u64 cellDeps[63] = {};                   // carried cells each carried cell's new value depends on
    std::vector<Idle2Input> inputs;
    u64 regT[16] = {}, headT[16] = {}, flagT[4] = {}, headFlagT[4] = {}; // flagT: per flag, bit = flag_V..flag_N
    u16 live = 0, written = 0, changed = 0; // changed: registers the first iteration modified or tainted
    u8 flagsLive = 0, flagsWritten = 0;
    auto flagsT = [&](u8 mask) { u64 t = 0; for (int f = 0; f < 4; f++) if (mask >> f & 1) t |= flagT[f]; return t; };
    static const u8 condFlags[7] = {0x4, 0x2, 0x8, 0x1, 0x6, 0x9, 0xD}; // Z, C, N, V, CZ, NV, NZV
    u32 regs[3][16], cpsr[3], hash[2] = {}, steps[2] = {};
    int iter = 0;
    std::vector<Idle2Access>& log = Idle2->Log;

    memcpy(regs[0], R, sizeof(R)); cpsr[0] = CPSR;
    FillPipeline();
    for (u32 n = 0;; n++)
    {
        const bool thumb = CPSR & 0x20;
        const u32 addr = R[15] - (thumb ? 2 : 4);
        if (n > 0 && addr == head)
        {
            if (iter == 0)
            {
                memcpy(regs[1], R, sizeof(R)); cpsr[1] = CPSR;
                memcpy(headT, regT, sizeof(regT)); memcpy(headFlagT, flagT, sizeof(flagT));
                if (cpsr[0] != cpsr[1])
                    return 0;
                for (int r = 0; r < 15; r++)
                    if (regs[0][r] != regs[1][r] || headT[r])
                        changed |= 1 << r;
                iter = 1;
            }
            else
            {
                if (hash[0] != hash[1] || steps[0] != steps[1])
                    return 0; // the iterations differ: not waiting (now)
                if (cpsr[0] != cpsr[1] || cpsr[1] != CPSR)
                    return 0;
                for (int r = 0; r < 15; r++)
                    if ((live >> r & 1) && (regs[0][r] != regs[1][r] || regs[1][r] != R[r] || headT[r]))
                        return 0;
                for (int f = 0; f < 4; f++)
                    if ((flagsLive >> f & 1) && headFlagT[f])
                        return 0;
                // Loop-carried cells must not feed each other in a cycle (`n = n + 1`,
                // swaps): then a skip only shifts their values in time and the next
                // executed iterations refresh them.
                u64 reach[63];
                for (int c = 0; c < cells; c++)
                    reach[c] = cellDeps[c];
                for (int round = 0; round < cells; round++)
                    for (int c = 0; c < cells; c++)
                        for (int d = 0; d < cells; d++)
                            if (reach[c] >> d & 1)
                                reach[c] |= reach[d];
                for (int c = 0; c < cells; c++)
                    if (reach[c] >> c & 1)
                        return 0;
                std::sort(inputs.begin(), inputs.end(), [](const Idle2Input& a, const Idle2Input& b)
                    { return a.Addr != b.Addr ? a.Addr < b.Addr : a.Size < b.Size; });
                inputs.erase(std::unique(inputs.begin(), inputs.end(), [](const Idle2Input& a, const Idle2Input& b)
                    { return a.Addr == b.Addr && a.Size == b.Size; }), inputs.end());
                for (Idle2Input& in : inputs)
                    in.Val = Idle2Peek(in.Addr, in.Size);
                s.Memo = true;
                s.Head = head;
                s.CPSR = CPSR;
                s.Live = live;
                memcpy(s.Regs, R, sizeof(R));
                s.Inputs = std::move(inputs);
                return 1;
            }
        }
        if (n >= 320)
            return 0; // ponytail: iterations this long are not treated as waits

        const u32 instr = thumb ? (NextInstr[0] & 0xFFFF) : NextInstr[0];
        const ARMInstrInfo::Info info = ARMInstrInfo::Decode(thumb, 0, instr, false);
        const u16 k = info.Kind;
        if (thumb ? (k == ARMInstrInfo::tk_SVC || k == ARMInstrInfo::tk_UNK)
                  : (k == ARMInstrInfo::ak_MCR || k == ARMInstrInfo::ak_MRC || k == ARMInstrInfo::ak_SVC || k == ARMInstrInfo::ak_UNK))
            return 0;
        const u8 condMask = thumb ? (k == ARMInstrInfo::tk_BCOND ? condFlags[(instr >> 9) & 7] : 0)
                                  : ((instr >> 28) < 0xE ? condFlags[instr >> 29] : 0);
        log.clear();
        Idle2Log = &log;
        const bool pass = Idle2Step();
        Idle2Log = nullptr;

        Idle2->Steps++;
        hash[iter] = (hash[iter] ^ (addr | pass)) * 0x01000193;
        steps[iter]++;

        // ---- taint ----
        const u16 src = info.SrcRegs & 0x7FFF, dst = info.DstRegs;
        const u16 am = Idle2AddrRegs(thumb, instr);
        const bool isMRS = !thumb && k == ARMInstrInfo::ak_MRS;
        const bool isMSR = !thumb && (k == ARMInstrInfo::ak_MSR_IMM || k == ARMInstrInfo::ak_MSR_REG);
        const bool isLoad = info.SpecialKind == ARMInstrInfo::special_LoadMem || info.SpecialKind == ARMInstrInfo::special_LoadLiteral;
        const bool isMem = isLoad || info.SpecialKind == ARMInstrInfo::special_WriteMem;
        const u8 readMask = info.ReadFlags | condMask | (isMRS ? 0xF : 0);
        const u64 condT = flagsT(condMask);
        u64 srcT = 0, addrT = 0, dataT = 0, loadT = 0;
        for (int r = 0; r < 15; r++)
        {
            if (src >> r & 1) srcT |= regT[r];
            if (am >> r & 1) addrT |= regT[r];
            else if (src >> r & 1) dataT |= regT[r];
        }
        srcT |= flagsT(info.ReadFlags | (isMRS ? 0xF : 0));
        if (iter == 1)
        {
            if (src & ~written & changed)
                return 0; // a register the loop changes feeds the next iteration (counter, pointer)
            live |= src & ~written;
            flagsLive |= readMask & ~flagsWritten;
            if (pass)
            {
                written |= dst & 0x7FFF;
                flagsWritten |= (isMSR && (instr & (1 << 19))) ? 0xF : (info.WriteFlags & 0xF);
            }
        }
        if ((condT && (isMem || (dst & 0x8000))) || (addrT && isMem) || (isMSR && (srcT | condT)))
            return 0;

        for (const Idle2Access& a : log)
        {
            const bool io = (a.Addr >> 24) == 0x04;
            const u32 o = a.Addr & 0xFFFFFF;
            const int cls = io ? (o < 0x1000 ? Idle2IOClass(o) : -1) : 0;
            if (a.Write)
            {
                if (io && cls != 2)
                    return 0; // the only IO a wait loop may write is the DIV/SQRT engine
                for (u32 b = a.Addr; b < a.Addr + a.Size; b++)
                {
                    const int c = iter == 1 ? cellOf(b & ~3u) : -1;
                    if (c >= 0)
                        cellDeps[c] |= dataT >> 1;
                    if (stores.Count >= Idle2ByteMap::N / 2)
                        return 0;
                    const int h = stores.Insert(b);
                    stores.Taint[h] = dataT;
                    stores.Iter[h] = (u8)iter;
                }
                continue;
            }
            if (cls < 0)
                return 0;
            if (cls == 1)
            {
                loadT |= T_TIME;
                continue;
            }
            u32 lo = a.Addr, hi = a.Addr + a.Size;
            bool result = false;
            if (cls == 2)
            {
                // DIV/SQRT control and parameter registers read back what was written;
                // the results depend on the parameters (and are treated as TIME).
                result = (o >= 0x2A0 && o < 0x2B0) || (o >= 0x2B4 && o < 0x2B8);
#ifndef LITEV_INSTANT_DIVSQRT
                if (o < 0x284 || (o >= 0x2B0 && o < 0x2B4))
                    loadT |= T_TIME; // busy bit
#endif
                if (result)
                {
                    loadT |= T_TIME;
                    lo = o >= 0x2B0 ? 0x040002B8 : 0x04000290;
                    hi = lo + 8 + (o >= 0x2B0 ? 0 : 8);
                }
            }
            bool fromStore = false, allStored = true;
            for (u32 b = lo; b < hi; b++)
            {
                const int h = stores.Find(b);
                if (h < 0) { allStored = false; continue; }
                fromStore = true;
                loadT |= stores.Taint[h];
                if (stores.Iter[h] < iter)
                {
                    int c = cellOf(b & ~3u);
                    if (c < 0)
                    {
                        if (cells >= 63)
                            return 0;
                        c = cells;
                        cellAddr[cells++] = b & ~3u;
                    }
                    loadT |= 2ull << c;
                }
            }
            if (result)
                continue;
            if (fromStore && !allStored)
                return 0; // partly loop-written, partly external: not tracked
            if (!fromStore && iter == 1)
                inputs.push_back({a.Addr, 0, a.Size});
        }

        u16 loaded = 0;
        if (isLoad)
        {
            if (thumb)
                loaded = (k == ARMInstrInfo::tk_POP || k == ARMInstrInfo::tk_LDMIA)
                    ? (u16)((instr & 0xFF) | ((k == ARMInstrInfo::tk_POP && (instr & 0x100)) ? 0x8000 : 0)) : dst;
            else if (k == ARMInstrInfo::ak_LDM)
                loaded = instr & 0xFFFF;
            else
                loaded = (dst & ~am) | (1 << ((instr >> 12) & 0xF));
        }
        const u64 aluT = isMem ? (addrT | condT) : (srcT | condT); // non-loaded dst: ALU result / writeback base
        const u64 ldT = loadT | condT | addrT;
        if ((dst & 0x8000) && ((loaded & 0x8000) ? ldT : aluT))
            return 0; // tainted PC
        for (int r = 0; r < 15; r++)
            if (dst >> r & 1)
            {
                const u64 t = (loaded >> r & 1) ? ldT : aluT;
                regT[r] = pass ? t : (regT[r] | t);
            }
        if (isMSR && (instr & (1 << 19)))
            memset(flagT, 0, sizeof(flagT));
        else
            for (int f = 0; f < 4; f++)
            {
                if (pass && (info.WriteFlags >> f & 1))
                    flagT[f] = srcT | condT;
                else if ((info.WriteFlags >> f & 0x11) != 0)
                    flagT[f] |= srcT | condT;
            }

        if (Halted)
            return 0; // the loop halts: it already waits for free
        if ((IRQ && !(CPSR & 0x80)) || NDS.ARM9Timestamp >= NDS.ARM9Target)
            return -1;
    }
}

bool ARMv5::Idle2Handle()
{
    if (!Idle2)
        return false;
    auto it = Idle2->Index.find(Idle2Site);
    if (it == Idle2->Index.end())
        return false;
    const u32 slot = it->second;
    Idle2SiteInfo& s = Idle2->Sites[slot];
    const u32 head = R[15] - ((CPSR & 0x20) ? 2 : 4);
    if (s.Memo && s.Head == head && s.CPSR == CPSR)
    {
        bool same = true;
        for (int r = 0; r < 15 && same; r++)
            if ((s.Live >> r & 1) && R[r] != s.Regs[r])
                same = false;
        for (size_t i = 0; i < s.Inputs.size() && same; i++)
            if (Idle2Peek(s.Inputs[i].Addr, s.Inputs[i].Size) != s.Inputs[i].Val)
                same = false;
        if (same)
        {
            Idle2->Skips++;
            return true;
        }
    }
    s.Memo = false;
    if (s.Hits < 16)
    {
        s.Hits++;
        return false;
    }
    auto giveUp = [&] {
        Idle2->Enable[slot] = 0;
        s.DisabledUntil = (u32)Idle2->Frames + (16u << s.Backoff);
        if (s.Backoff < 6)
            s.Backoff++;
    };
    if (++s.Tries > 4)
    {
        giveUp(); // inputs keep changing / never decided
        return false;
    }
    Idle2->Traces++;
    const int res = Idle2Trace(head, s);
    if (res > 0)
    {
        s.Tries = 0;
        s.Backoff = 0;
        Idle2->Accepts++;
        Idle2->Skips++;
        return true;
    }
    if (res == 0)
    {
        Idle2->Rejects++;
        giveUp();
    }
    // Hand back to the JIT at an existing block, not mid-block (that would compile a new
    // block entry): interpret on to the next block start.
    for (int i = 0; i < 64 && !Halted && !(IRQ && !(CPSR & 0x80)) && NDS.ARM9Timestamp < NDS.ARM9Target && !Idle2AtBlock(); i++)
        Idle2Step();
    return false;
}
#endif

ARMv4::ARMv4(melonDS::NDS& nds, std::optional<GDBArgs> gdb, bool jit) : ARM(1, jit, gdb, nds)
{
    //
}

ARMv5::~ARMv5()
{
    // DTCM is owned by Memory, not going to delete it
#ifdef LITEV_JIT_IDLE2
    delete Idle2;
    Idle2 = nullptr; // the JIT may still reset its block cache (Idle2Reset) while tearing down
#endif
}

void ARM::SetGdbArgs(std::optional<GDBArgs> gdb)
{
#ifdef GDBSTUB_ENABLED
    GdbStub.Close();
    if (gdb)
    {
        int port = Num ? gdb->PortARM7 : gdb->PortARM9;
        GdbStub.Init(port);
        BreakOnStartup = Num ? gdb->ARM7BreakOnStartup : gdb->ARM9BreakOnStartup;
    }
    IsSingleStep = false;
#endif
}

void ARM::Reset()
{
    Cycles = 0;
    Halted = 0;

    IRQ = 0;

    for (int i = 0; i < 16; i++)
        R[i] = 0;

    CPSR = 0x000000D3;

    for (int i = 0; i < 7; i++)
        R_FIQ[i] = 0;
    for (int i = 0; i < 2; i++)
    {
        R_SVC[i] = 0;
        R_ABT[i] = 0;
        R_IRQ[i] = 0;
        R_UND[i] = 0;
    }

    R_FIQ[7] = 0x00000010;
    R_SVC[2] = 0x00000010;
    R_ABT[2] = 0x00000010;
    R_IRQ[2] = 0x00000010;
    R_UND[2] = 0x00000010;

    ExceptionBase = Num ? 0x00000000 : 0xFFFF0000;

    CodeMem.Mem = NULL;

#ifdef JIT_ENABLED
    FastBlockLookup = NULL;
    FastBlockLookupStart = 0;
    FastBlockLookupSize = 0;
#ifdef LITEV_JIT_REGION_CACHE
    JitRegionCacheClear();
#endif
#endif

#ifdef GDBSTUB_ENABLED
    IsSingleStep = false;
    BreakReq = false;
#endif

    // zorp
    JumpTo(ExceptionBase);
}

void ARMv5::Reset()
{
    PU_Map = PU_PrivMap;

    ARM::Reset();
}


void ARM::DoSavestate(Savestate* file)
{
    file->Section((char*)(Num ? "ARM7" : "ARM9"));

    file->Var32((u32*)&Cycles);
    //file->Var32((u32*)&CyclesToRun);

    // hack to make save states compatible
    u32 halted = Halted;
    file->Var32(&halted);
    Halted = halted;

    file->VarArray(R, 16*sizeof(u32));
    file->Var32(&CPSR);
    file->VarArray(R_FIQ, 8*sizeof(u32));
    file->VarArray(R_SVC, 3*sizeof(u32));
    file->VarArray(R_ABT, 3*sizeof(u32));
    file->VarArray(R_IRQ, 3*sizeof(u32));
    file->VarArray(R_UND, 3*sizeof(u32));
    file->Var32(&CurInstr);
#ifdef JIT_ENABLED
    if (file->Saving && NDS.IsJITEnabled())
    {
        // hack, the JIT doesn't really pipeline
        // but we still want JIT save states to be
        // loaded while running the interpreter
        FillPipeline();
    }
#endif
    file->VarArray(NextInstr, 2*sizeof(u32));

    file->Var32(&ExceptionBase);

    if (!file->Saving)
    {
        CPSR |= 0x00000010;
        R_FIQ[7] |= 0x00000010;
        R_SVC[2] |= 0x00000010;
        R_ABT[2] |= 0x00000010;
        R_IRQ[2] |= 0x00000010;
        R_UND[2] |= 0x00000010;

        if (!Num)
        {
            SetupCodeMem(R[15]); // should fix it
            ((ARMv5*)this)->RegionCodeCycles = ((ARMv5*)this)->MemTimings[R[15] >> 12][0];

            if ((CPSR & 0x1F) == 0x10)
                ((ARMv5*)this)->PU_Map = ((ARMv5*)this)->PU_UserMap;
            else
                ((ARMv5*)this)->PU_Map = ((ARMv5*)this)->PU_PrivMap;
        }
        else
        {
            CodeRegion = R[15] >> 24;
            CodeCycles = R[15] >> 15; // cheato
        }
    }
}

void ARMv5::DoSavestate(Savestate* file)
{
    ARM::DoSavestate(file);
    CP15DoSavestate(file);
}


void ARM::SetupCodeMem(u32 addr)
{
    if (!Num)
    {
        ((ARMv5*)this)->GetCodeMemRegion(addr, &CodeMem);
    }
    else
    {
        // not sure it's worth it for the ARM7
        // esp. as everything there generally runs on WRAM
        // and due to how it's mapped, we can't use this optimization
        //NDS::ARM7GetMemRegion(addr, false, &CodeMem);
    }
}

void ARMv5::JumpTo(u32 addr, bool restorecpsr)
{
    if (restorecpsr)
    {
        RestoreCPSR();

        if (CPSR & 0x20)    addr |= 0x1;
        else                addr &= ~0x1;
    }

    // aging cart debug crap
    //if (addr == 0x0201764C) printf("capture test %d: R1=%08X\n", R[6], R[1]);
    //if (addr == 0x020175D8) printf("capture test %d: res=%08X\n", R[6], R[0]);

    u32 oldregion = R[15] >> 24;
    u32 newregion = addr >> 24;

    RegionCodeCycles = MemTimings[addr >> 12][0];

    if (addr & 0x1)
    {
        addr &= ~0x1;
        R[15] = addr+2;

        if (newregion != oldregion) SetupCodeMem(addr);

        // two-opcodes-at-once fetch
        // doesn't matter if we put garbage in the MSbs there
        if (addr & 0x2)
        {
            NextInstr[0] = CodeRead32(addr-2, true) >> 16;
            Cycles += CodeCycles;
            NextInstr[1] = CodeRead32(addr+2, false);
            Cycles += CodeCycles;
        }
        else
        {
            NextInstr[0] = CodeRead32(addr, true);
            NextInstr[1] = NextInstr[0] >> 16;
            Cycles += CodeCycles;
        }

        CPSR |= 0x20;
    }
    else
    {
        addr &= ~0x3;
        R[15] = addr+4;

        if (newregion != oldregion) SetupCodeMem(addr);

        NextInstr[0] = CodeRead32(addr, true);
        Cycles += CodeCycles;
        NextInstr[1] = CodeRead32(addr+4, false);
        Cycles += CodeCycles;

        CPSR &= ~0x20;
    }

    if (!(PU_Map[addr>>12] & 0x04))
    {
        PrefetchAbort();
        return;
    }

    NDS.MonitorARM9Jump(addr);
}

void ARMv4::JumpTo(u32 addr, bool restorecpsr)
{
    if (restorecpsr)
    {
        RestoreCPSR();

        if (CPSR & 0x20)    addr |= 0x1;
        else                addr &= ~0x1;
    }

    u32 oldregion = R[15] >> 23;
    u32 newregion = addr >> 23;

    CodeRegion = addr >> 24;
    CodeCycles = addr >> 15; // cheato

    if (addr & 0x1)
    {
        addr &= ~0x1;
        R[15] = addr+2;

        //if (newregion != oldregion) SetupCodeMem(addr);

        NextInstr[0] = CodeRead16(addr);
        NextInstr[1] = CodeRead16(addr+2);
        Cycles += NDS.ARM7MemTimings[CodeCycles][0] + NDS.ARM7MemTimings[CodeCycles][1];

        CPSR |= 0x20;
    }
    else
    {
        addr &= ~0x3;
        R[15] = addr+4;

        //if (newregion != oldregion) SetupCodeMem(addr);

        NextInstr[0] = CodeRead32(addr);
        NextInstr[1] = CodeRead32(addr+4);
        Cycles += NDS.ARM7MemTimings[CodeCycles][2] + NDS.ARM7MemTimings[CodeCycles][3];

        CPSR &= ~0x20;
    }
}

void ARM::RestoreCPSR()
{
    u32 oldcpsr = CPSR;

    switch (CPSR & 0x1F)
    {
    case 0x11:
        CPSR = R_FIQ[7];
        break;

    case 0x12:
        CPSR = R_IRQ[2];
        break;

    case 0x13:
        CPSR = R_SVC[2];
        break;

    case 0x14:
    case 0x15:
    case 0x16:
    case 0x17:
        CPSR = R_ABT[2];
        break;

    case 0x18:
    case 0x19:
    case 0x1A:
    case 0x1B:
        CPSR = R_UND[2];
        break;

    default:
        Log(LogLevel::Warn, "!! attempt to restore CPSR under bad mode %02X, %08X\n", CPSR&0x1F, R[15]);
        break;
    }

    CPSR |= 0x00000010;

    UpdateMode(oldcpsr, CPSR);
}

void ARM::UpdateMode(u32 oldmode, u32 newmode, bool phony)
{
#ifdef LITEV_EXIT_PROTO_STOP
    // Every C++ write of the CPSR control byte (MSR, RestoreCPSR, exception entry) ends
    // here, so this is where clearing CPSR.I can unmask a pending IRQ mid-slice. Before the
    // early return: an I-only change keeps the mode.
    JitStopToBudget();
#endif
    if ((oldmode & 0x1F) == (newmode & 0x1F)) return;

    switch (oldmode & 0x1F)
    {
    case 0x11:
        std::swap(R[8], R_FIQ[0]);
        std::swap(R[9], R_FIQ[1]);
        std::swap(R[10], R_FIQ[2]);
        std::swap(R[11], R_FIQ[3]);
        std::swap(R[12], R_FIQ[4]);
        std::swap(R[13], R_FIQ[5]);
        std::swap(R[14], R_FIQ[6]);
        break;

    case 0x12:
        std::swap(R[13], R_IRQ[0]);
        std::swap(R[14], R_IRQ[1]);
        break;

    case 0x13:
        std::swap(R[13], R_SVC[0]);
        std::swap(R[14], R_SVC[1]);
        break;

    case 0x17:
        std::swap(R[13], R_ABT[0]);
        std::swap(R[14], R_ABT[1]);
        break;

    case 0x1B:
        std::swap(R[13], R_UND[0]);
        std::swap(R[14], R_UND[1]);
        break;
    }

    switch (newmode & 0x1F)
    {
    case 0x11:
        std::swap(R[8], R_FIQ[0]);
        std::swap(R[9], R_FIQ[1]);
        std::swap(R[10], R_FIQ[2]);
        std::swap(R[11], R_FIQ[3]);
        std::swap(R[12], R_FIQ[4]);
        std::swap(R[13], R_FIQ[5]);
        std::swap(R[14], R_FIQ[6]);
        break;

    case 0x12:
        std::swap(R[13], R_IRQ[0]);
        std::swap(R[14], R_IRQ[1]);
        break;

    case 0x13:
        std::swap(R[13], R_SVC[0]);
        std::swap(R[14], R_SVC[1]);
        break;

    case 0x17:
        std::swap(R[13], R_ABT[0]);
        std::swap(R[14], R_ABT[1]);
        break;

    case 0x1B:
        std::swap(R[13], R_UND[0]);
        std::swap(R[14], R_UND[1]);
        break;
    }

    if ((!phony) && (Num == 0))
    {
        if ((newmode & 0x1F) == 0x10)
            ((ARMv5*)this)->PU_Map = ((ARMv5*)this)->PU_UserMap;
        else
            ((ARMv5*)this)->PU_Map = ((ARMv5*)this)->PU_PrivMap;
    }
}

void ARM::TriggerIRQ()
{
    if (CPSR & 0x80)
        return;

    // liteDS-v2 Unit 2 (shadow, inert): an IRQ being delivered forces the slice to
    // end so the dispatcher re-enters C++. Currently redundant with StopExecution.
    ForceExecutionExit();

    u32 oldcpsr = CPSR;
    CPSR &= ~0xFF;
    CPSR |= 0xD2;
    UpdateMode(oldcpsr, CPSR);

    R_IRQ[2] = oldcpsr;
    R[14] = R[15] + (oldcpsr & 0x20 ? 2 : 0);
    JumpTo(ExceptionBase + 0x18);

    // ARDS cheat support
    // normally, those work by hijacking the ARM7 VBlank handler
    if (Num == 1)
    {
        if ((NDS.IF[1] & NDS.IE[1]) & (1<<IRQ_VBlank))
            NDS.AREngine.RunCheats();
    }
}

void ARMv5::PrefetchAbort()
{
    Log(LogLevel::Warn, "ARM9: prefetch abort (%08X)\n", R[15]);

    u32 oldcpsr = CPSR;
    CPSR &= ~0xBF;
    CPSR |= 0x97;
    UpdateMode(oldcpsr, CPSR);

    // this shouldn't happen, but if it does, we're stuck in some nasty endless loop
    // so better take care of it
    if (!(PU_Map[ExceptionBase>>12] & 0x04))
    {
        Log(LogLevel::Error, "!!!!! EXCEPTION REGION NOT EXECUTABLE. THIS IS VERY BAD!!\n");
        NDS.Stop(Platform::StopReason::BadExceptionRegion);
        return;
    }

    R_ABT[2] = oldcpsr;
    R[14] = R[15] + (oldcpsr & 0x20 ? 2 : 0);
    JumpTo(ExceptionBase + 0x0C);
}

void ARMv5::DataAbort()
{
    Log(LogLevel::Warn, "ARM9: data abort (%08X)\n", R[15]);

    u32 oldcpsr = CPSR;
    CPSR &= ~0xBF;
    CPSR |= 0x97;
    UpdateMode(oldcpsr, CPSR);

    R_ABT[2] = oldcpsr;
    R[14] = R[15] + (oldcpsr & 0x20 ? 4 : 0);
    JumpTo(ExceptionBase + 0x10);
}

void ARM::CheckGdbIncoming()
{
    GdbCheckA();
}

#ifdef LITEV_JIT_REGION_CACHE
bool ARM::JitSetupRegion(u32 instrAddr)
{
    for (const JitRegionEntry& e : JitRegions)
    {
        if (instrAddr - e.Start < e.Size)
        {
            FastBlockLookup = e.Lookup;
            FastBlockLookupStart = e.Start;
            FastBlockLookupSize = e.Size;
            return true;
        }
    }
    int region;
    if (!NDS.JIT.SetupExecutableRegion(Num, instrAddr, FastBlockLookup, FastBlockLookupStart, FastBlockLookupSize, &region))
        return false;
    if (NDS.JIT.RegionCacheable(Num, region, FastBlockLookupStart, FastBlockLookupSize))
    {
        JitRegions[JitRegionNext] = {FastBlockLookupStart, FastBlockLookupSize, FastBlockLookup};
        JitRegionNext = (JitRegionNext + 1) % JitRegionCount;
    }
    return true;
}
#endif

template <CPUExecuteMode mode>
void ARMv5::Execute()
{
    if constexpr (mode == CPUExecuteMode::InterpreterGDB)
        GdbCheckB();

    if (Halted)
    {
        if (Halted == 2)
        {
            Halted = 0;
        }
        else if (NDS.HaltInterrupted(0))
        {
            Halted = 0;
            if (NDS.IME[0] & 0x1)
                TriggerIRQ();
        }
        else
        {
            NDS.ARM9Timestamp = NDS.ARM9Target;
            return;
        }
    }

    while (NDS.ARM9Timestamp < NDS.ARM9Target)
    {
#ifdef JIT_ENABLED
        if constexpr (mode == CPUExecuteMode::JIT)
        {
            u32 instrAddr = R[15] - ((CPSR&0x20)?2:4);

            if ((instrAddr < FastBlockLookupStart || instrAddr >= (FastBlockLookupStart + FastBlockLookupSize))
#ifdef LITEV_JIT_REGION_CACHE
                && !JitSetupRegion(instrAddr))
#else
                && !NDS.JIT.SetupExecutableRegion(0, instrAddr, FastBlockLookup, FastBlockLookupStart, FastBlockLookupSize))
#endif
            {
                NDS.ARM9Timestamp = NDS.ARM9Target;
                Log(LogLevel::Error, "ARMv5 PC in non executable region %08X\n", R[15]);
                return;
            }

            JitBlockEntry block = NDS.JIT.LookUpBlock(0, FastBlockLookup,
                instrAddr - FastBlockLookupStart, instrAddr);

            // liteDS-v2 Unit 2 (shadow): maintain the slice budget in the exact same
            // unit as Cycles. The loop guarantees ARM9Timestamp < ARM9Target here, so
            // this is strictly positive before dispatch (a 0 later => forced exit).
            CyclesBudget = (s32)std::min<s64>((s64)(NDS.ARM9Target - NDS.ARM9Timestamp), INT32_MAX);
#ifdef LITEV_JIT_BUDGET_REG
            JitTsPtr = &NDS.ARM9Timestamp;
            JitTsBase = NDS.ARM9Timestamp + CyclesBudget;
#endif
#ifdef LITEV_EXIT_PROTO_STOP
            JitStopToBudget();   // a stop already pending: exit at the first hop
#endif

            if (block)
            {
#ifdef LITEV_JIT_LAZYFLAGS
                // V2 lazy-flags: bracket the JIT slice ONLY (not the CompileBlock path,
                // which INTERPRETS the block and updates ARM::CPSR NZCV directly — see
                // ARMJIT.cpp:996). Seed the per-slice NZCV mirror slot from the
                // authoritative ARM::CPSR before entering JIT; inside the slice the JIT
                // homes guest NZCV in host PSTATE / this slot and ARM::CPSR[31:28] goes
                // stale. This keeps ARM::CPSR the single authoritative NZCV home for every
                // C++ reader/writer (TriggerIRQ SPSR=CPSR, RestoreCPSR, savestate,
                // interpreter-compile) outside the JIT slice.
                JitNZCV = CPSR & 0xF0000000;
#endif
                ARM_Dispatch(this, block);
#ifdef LITEV_JIT_LAZYFLAGS
                // Merge the slice's final NZCV nibble (canonical in JitNZCV at every block
                // exit) back into ARM::CPSR before any C++ observes it (the IRQ dispatch
                // below does SPSR = CPSR).
                CPSR = (CPSR & 0x0FFFFFFF) | (JitNZCV & 0xF0000000);
#endif
            }
            else
                NDS.JIT.CompileBlock(this);

#if defined(LITEV_SHADOW_ASSERT)
            LiteV_ShadowAssertBudget("ARM9", Cycles, CyclesBudget,
                                     (s64)NDS.ARM9Timestamp, (s64)NDS.ARM9Target);
#endif

#if defined(LITEV_JIT_DISPATCH) && LITEV_PROFILE
            // Unit 3: count C++ re-entries from the dispatcher. Dispatcher *hits*
            // (chained transitions) happen entirely in asm and are not counted here;
            // this tallies only the boundaries where the dispatcher handed control back.
            if (block)
            {
                using namespace melonDS::LiteProfile;
                AddAtomic(g_Frame.CppReentries);
                // budget left and no pending stop => the dispatcher's inline lookup missed
                // (uncompiled target or region change) rather than the slice ending.
                if (!StopExecution && CyclesBudget > 0)
                    AddAtomic(g_Frame.DispatcherMisses);
            }
#endif

            if (StopExecution)
            {
#ifdef LITEV_JIT_IDLE2
                if (IdleLoop == 2)
                {
                    // candidate wait-loop back-edge (LITEV_JIT_IDLE2)
                    IdleLoop = 0;
                    if (!Halted && !(IRQ && !(CPSR & 0x80)) && Idle2Handle())
                    {
                        Cycles = 0;
                        if (NDS.ARM9Timestamp < NDS.ARM9Target)
                            NDS.ARM9Timestamp = NDS.ARM9Target;
                        break;
                    }
                }
#endif
                // this order is crucial otherwise idle loops waiting for an IRQ won't function
                if (IRQ)
                    TriggerIRQ();

                if (Halted || IdleLoop)
                {
                    if ((Halted == 1 || IdleLoop) && NDS.ARM9Timestamp < NDS.ARM9Target)
                    {
#if LITEV_PROFILE
                        if (IdleLoop)
                            melonDS::LiteProfile::AddAtomic(melonDS::LiteProfile::g_Frame.ARM9IdleHits);
#endif
                        Cycles = 0;
                        NDS.ARM9Timestamp = NDS.ARM9Target;
                    }
                    IdleLoop = 0;
                    break;
                }
            }
        }
        else
#endif
        {
            if (CPSR & 0x20) // THUMB
            {
                if constexpr (mode == CPUExecuteMode::InterpreterGDB)
                    GdbCheckC();

                // prefetch
                R[15] += 2;
                CurInstr = NextInstr[0];
                NextInstr[0] = NextInstr[1];
                if (R[15] & 0x2) { NextInstr[1] >>= 16; CodeCycles = 0; }
                else             NextInstr[1] = CodeRead32(R[15], false);

                // actually execute
                u32 icode = (CurInstr >> 6) & 0x3FF;
                ARMInterpreter::THUMBInstrTable[icode](this);
            }
            else
            {
                if constexpr (mode == CPUExecuteMode::InterpreterGDB)
                    GdbCheckC();

                // prefetch
                R[15] += 4;
                CurInstr = NextInstr[0];
                NextInstr[0] = NextInstr[1];
                NextInstr[1] = CodeRead32(R[15], false);

                // actually execute
                if (CheckCondition(CurInstr >> 28))
                {
                    u32 icode = ((CurInstr >> 4) & 0xF) | ((CurInstr >> 16) & 0xFF0);
                    ARMInterpreter::ARMInstrTable[icode](this);
                }
                else if ((CurInstr & 0xFE000000) == 0xFA000000)
                {
                    ARMInterpreter::A_BLX_IMM(this);
                }
                else
                    AddCycles_C();
            }

            // TODO optimize this shit!!!
            if (Halted)
            {
                if (Halted == 1 && NDS.ARM9Timestamp < NDS.ARM9Target)
                {
                    NDS.ARM9Timestamp = NDS.ARM9Target;
                }
                break;
            }
            /*if (NDS::IF[0] & NDS::IE[0])
            {
                if (NDS::IME[0] & 0x1)
                    TriggerIRQ();
            }*/
            if (IRQ) TriggerIRQ();

        }

        NDS.ARM9Timestamp += Cycles;
        Cycles = 0;
    }

    if (Halted == 2)
        Halted = 0;
}
template void ARMv5::Execute<CPUExecuteMode::Interpreter>();
template void ARMv5::Execute<CPUExecuteMode::InterpreterGDB>();
#ifdef JIT_ENABLED
template void ARMv5::Execute<CPUExecuteMode::JIT>();
#endif

template <CPUExecuteMode mode>
void ARMv4::Execute()
{
    if constexpr (mode == CPUExecuteMode::InterpreterGDB)
        GdbCheckB();

    if (Halted)
    {
        if (Halted == 2)
        {
            Halted = 0;
        }
        else if (NDS.HaltInterrupted(1))
        {
            Halted = 0;
            if (NDS.IME[1] & 0x1)
                TriggerIRQ();
        }
        else
        {
            NDS.ARM7Timestamp = NDS.ARM7Target;
            return;
        }
    }

    while (NDS.ARM7Timestamp < NDS.ARM7Target)
    {
#ifdef JIT_ENABLED
        if constexpr (mode == CPUExecuteMode::JIT)
        {
            u32 instrAddr = R[15] - ((CPSR&0x20)?2:4);

            if ((instrAddr < FastBlockLookupStart || instrAddr >= (FastBlockLookupStart + FastBlockLookupSize))
#ifdef LITEV_JIT_REGION_CACHE
                && !JitSetupRegion(instrAddr))
#else
                && !NDS.JIT.SetupExecutableRegion(1, instrAddr, FastBlockLookup, FastBlockLookupStart, FastBlockLookupSize))
#endif
            {
                NDS.ARM7Timestamp = NDS.ARM7Target;
                Log(LogLevel::Error, "ARMv4 PC in non executable region %08X\n", R[15]);
                return;
            }

            JitBlockEntry block = NDS.JIT.LookUpBlock(1, FastBlockLookup,
                instrAddr - FastBlockLookupStart, instrAddr);

            // liteDS-v2 Unit 2 (shadow): ARM7 analog. ARM7 timestamps carry no clock
            // shift, so budget = ARM7Target - ARM7Timestamp is already in Cycles units.
            CyclesBudget = (s32)std::min<s64>((s64)(NDS.ARM7Target - NDS.ARM7Timestamp), INT32_MAX);
#ifdef LITEV_JIT_BUDGET_REG
            JitTsPtr = &NDS.ARM7Timestamp;
            JitTsBase = NDS.ARM7Timestamp + CyclesBudget;
#endif
#ifdef LITEV_EXIT_PROTO_STOP
            JitStopToBudget();   // a stop already pending: exit at the first hop
#endif

            if (block)
            {
#ifdef LITEV_JIT_LAZYFLAGS
                // V2 lazy-flags (ARM7 analog): bracket the JIT slice ONLY, not the
                // CompileBlock (interpreter) path. See the ARM9 comment above.
                JitNZCV = CPSR & 0xF0000000;
#endif
                ARM_Dispatch(this, block);
#ifdef LITEV_JIT_LAZYFLAGS
                CPSR = (CPSR & 0x0FFFFFFF) | (JitNZCV & 0xF0000000);
#endif
            }
            else
                NDS.JIT.CompileBlock(this);

#if defined(LITEV_SHADOW_ASSERT)
            LiteV_ShadowAssertBudget("ARM7", Cycles, CyclesBudget,
                                     (s64)NDS.ARM7Timestamp, (s64)NDS.ARM7Target);
#endif

#if defined(LITEV_JIT_DISPATCH) && LITEV_PROFILE
            if (block)
            {
                using namespace melonDS::LiteProfile;
                AddAtomic(g_Frame.CppReentries);
                if (!StopExecution && CyclesBudget > 0)
                    AddAtomic(g_Frame.DispatcherMisses);
            }
#endif

            if (StopExecution)
            {
                if (IRQ)
                    TriggerIRQ();

                if (Halted || IdleLoop)
                {
                    if ((Halted == 1 || IdleLoop) && NDS.ARM7Timestamp < NDS.ARM7Target)
                    {
#if LITEV_PROFILE
                        if (IdleLoop)
                            melonDS::LiteProfile::AddAtomic(melonDS::LiteProfile::g_Frame.ARM7IdleHits);
#endif
                        Cycles = 0;
                        NDS.ARM7Timestamp = NDS.ARM7Target;
                    }
                    IdleLoop = 0;
                    break;
                }
            }
        }
        else
#endif
        {
            if (CPSR & 0x20) // THUMB
            {
                if constexpr (mode == CPUExecuteMode::InterpreterGDB)
                    GdbCheckC();

                // prefetch
                R[15] += 2;
                CurInstr = NextInstr[0];
                NextInstr[0] = NextInstr[1];
                NextInstr[1] = CodeRead16(R[15]);

                // actually execute
                u32 icode = (CurInstr >> 6);
                ARMInterpreter::THUMBInstrTable[icode](this);
            }
            else
            {
                if constexpr (mode == CPUExecuteMode::InterpreterGDB)
                    GdbCheckC();

                // prefetch
                R[15] += 4;
                CurInstr = NextInstr[0];
                NextInstr[0] = NextInstr[1];
                NextInstr[1] = CodeRead32(R[15]);

                // actually execute
                if (CheckCondition(CurInstr >> 28))
                {
                    u32 icode = ((CurInstr >> 4) & 0xF) | ((CurInstr >> 16) & 0xFF0);
                    ARMInterpreter::ARMInstrTable[icode](this);
                }
                else
                    AddCycles_C();
            }

            // TODO optimize this shit!!!
            if (Halted)
            {
                if (Halted == 1 && NDS.ARM7Timestamp < NDS.ARM7Target)
                {
                    NDS.ARM7Timestamp = NDS.ARM7Target;
                }
                break;
            }
            /*if (NDS::IF[1] & NDS::IE[1])
            {
                if (NDS::IME[1] & 0x1)
                    TriggerIRQ();
            }*/
            if (IRQ) TriggerIRQ();
        }

        NDS.ARM7Timestamp += Cycles;
        Cycles = 0;
    }

    if (Halted == 2)
        Halted = 0;

    if (Halted == 4)
    {
        assert(NDS.ConsoleType == 1);
        auto& dsi = dynamic_cast<melonDS::DSi&>(NDS);
        dsi.SoftReset();
        Halted = 2;
    }
}

template void ARMv4::Execute<CPUExecuteMode::Interpreter>();
template void ARMv4::Execute<CPUExecuteMode::InterpreterGDB>();
#ifdef JIT_ENABLED
template void ARMv4::Execute<CPUExecuteMode::JIT>();
#endif

void ARMv5::FillPipeline()
{
    SetupCodeMem(R[15]);

    if (CPSR & 0x20)
    {
        if ((R[15] - 2) & 0x2)
        {
            NextInstr[0] = CodeRead32(R[15] - 4, false) >> 16;
            NextInstr[1] = CodeRead32(R[15], false);
        }
        else
        {
            NextInstr[0] = CodeRead32(R[15] - 2, false);
            NextInstr[1] = NextInstr[0] >> 16;
        }
    }
    else
    {
        NextInstr[0] = CodeRead32(R[15] - 4, false);
        NextInstr[1] = CodeRead32(R[15], false);
    }
}

void ARMv4::FillPipeline()
{
    SetupCodeMem(R[15]);

    if (CPSR & 0x20)
    {
        NextInstr[0] = CodeRead16(R[15] - 2);
        NextInstr[1] = CodeRead16(R[15]);
    }
    else
    {
        NextInstr[0] = CodeRead32(R[15] - 4);
        NextInstr[1] = CodeRead32(R[15]);
    }
}

#ifdef GDBSTUB_ENABLED
u32 ARM::ReadReg(Gdb::Register reg)
{
    using Gdb::Register;
    int r = static_cast<int>(reg);

    if (reg < Register::pc) return R[r];
    else if (reg == Register::pc)
    {
        return R[r] - ((CPSR & 0x20) ? 2 : 4);
    }
    else if (reg == Register::cpsr) return CPSR;
    else if (reg == Register::sp_usr || reg == Register::lr_usr)
    {
        r -= static_cast<int>(Register::sp_usr);
        if (ModeIs(0x10) || ModeIs(0x1f))
        {
            return R[13 + r];
        }
        else switch (CPSR & 0x1f)
        {
        case 0x11: return R_FIQ[5 + r];
        case 0x12: return R_IRQ[0 + r];
        case 0x13: return R_SVC[0 + r];
        case 0x17: return R_ABT[0 + r];
        case 0x1b: return R_UND[0 + r];
        }
    }
    else if (reg >= Register::r8_fiq && reg <= Register::lr_fiq)
    {
        r -= static_cast<int>(Register::r8_fiq);
        return ModeIs(0x11) ? R[ 8 + r] : R_FIQ[r];
    }
    else if (reg == Register::sp_irq || reg == Register::lr_irq)
    {
        r -= static_cast<int>(Register::sp_irq);
        return ModeIs(0x12) ? R[13 + r] : R_IRQ[r];
    }
    else if (reg == Register::sp_svc || reg == Register::lr_svc)
    {
        r -= static_cast<int>(Register::sp_svc);
        return ModeIs(0x13) ? R[13 + r] : R_SVC[r];
    }
    else if (reg == Register::sp_abt || reg == Register::lr_abt)
    {
        r -= static_cast<int>(Register::sp_abt);
        return ModeIs(0x17) ? R[13 + r] : R_ABT[r];
    }
    else if (reg == Register::sp_und || reg == Register::lr_und)
    {
        r -= static_cast<int>(Register::sp_und);
        return ModeIs(0x1b) ? R[13 + r] : R_UND[r];
    }
    else if (reg == Register::spsr_fiq) return ModeIs(0x11) ? CPSR : R_FIQ[7];
    else if (reg == Register::spsr_irq) return ModeIs(0x12) ? CPSR : R_IRQ[2];
    else if (reg == Register::spsr_svc) return ModeIs(0x13) ? CPSR : R_SVC[2];
    else if (reg == Register::spsr_abt) return ModeIs(0x17) ? CPSR : R_ABT[2];
    else if (reg == Register::spsr_und) return ModeIs(0x1b) ? CPSR : R_UND[2];

    Log(LogLevel::Warn, "GDB reg read: unknown reg no %d\n", r);
    return 0xdeadbeef;
}
void ARM::WriteReg(Gdb::Register reg, u32 v)
{
    using Gdb::Register;
    int r = static_cast<int>(reg);

    if (reg < Register::pc) R[r] = v;
    else if (reg == Register::pc) JumpTo(v);
    else if (reg == Register::cpsr) CPSR = v;
    else if (reg == Register::sp_usr || reg == Register::lr_usr)
    {
        r -= static_cast<int>(Register::sp_usr);
        if (ModeIs(0x10) || ModeIs(0x1f))
        {
            R[13 + r] = v;
        }
        else switch (CPSR & 0x1f)
        {
        case 0x11: R_FIQ[5 + r] = v; break;
        case 0x12: R_IRQ[0 + r] = v; break;
        case 0x13: R_SVC[0 + r] = v; break;
        case 0x17: R_ABT[0 + r] = v; break;
        case 0x1b: R_UND[0 + r] = v; break;
        }
    }
    else if (reg >= Register::r8_fiq && reg <= Register::lr_fiq)
    {
        r -= static_cast<int>(Register::r8_fiq);
        *(ModeIs(0x11) ? &R[ 8 + r] : &R_FIQ[r]) = v;
    }
    else if (reg == Register::sp_irq || reg == Register::lr_irq)
    {
        r -= static_cast<int>(Register::sp_irq);
        *(ModeIs(0x12) ? &R[13 + r] : &R_IRQ[r]) = v;
    }
    else if (reg == Register::sp_svc || reg == Register::lr_svc)
    {
        r -= static_cast<int>(Register::sp_svc);
        *(ModeIs(0x13) ? &R[13 + r] : &R_SVC[r]) = v;
    }
    else if (reg == Register::sp_abt || reg == Register::lr_abt)
    {
        r -= static_cast<int>(Register::sp_abt);
        *(ModeIs(0x17) ? &R[13 + r] : &R_ABT[r]) = v;
    }
    else if (reg == Register::sp_und || reg == Register::lr_und)
    {
        r -= static_cast<int>(Register::sp_und);
        *(ModeIs(0x1b) ? &R[13 + r] : &R_UND[r]) = v;
    }
    else if (reg == Register::spsr_fiq)
    {
        *(ModeIs(0x11) ? &CPSR : &R_FIQ[7]) = v;
    }
    else if (reg == Register::spsr_irq)
    {
        *(ModeIs(0x12) ? &CPSR : &R_IRQ[2]) = v;
    }
    else if (reg == Register::spsr_svc)
    {
        *(ModeIs(0x13) ? &CPSR : &R_SVC[2]) = v;
    }
    else if (reg == Register::spsr_abt)
    {
        *(ModeIs(0x17) ? &CPSR : &R_ABT[2]) = v;
    }
    else if (reg == Register::spsr_und)
    {
        *(ModeIs(0x1b) ? &CPSR : &R_UND[2]) = v;
    }
    else Log(LogLevel::Warn, "GDB reg write: unknown reg no %d (write 0x%08x)\n", r, v);
}
u32 ARM::ReadMem(u32 addr, int size)
{
    if (size == 8) return BusRead8(addr);
    else if (size == 16) return BusRead16(addr);
    else if (size == 32) return BusRead32(addr);
    else return 0xfeedface;
}
void ARM::WriteMem(u32 addr, int size, u32 v)
{
    if (size == 8) BusWrite8(addr, (u8)v);
    else if (size == 16) BusWrite16(addr, (u16)v);
    else if (size == 32) BusWrite32(addr, v);
}

void ARM::ResetGdb()
{
    NDS.Reset();
    NDS.GPU.StartFrame(); // need this to properly kick off the scheduler & frame output
}
int ARM::RemoteCmd(const u8* cmd, size_t len)
{
    (void)len;

    Log(LogLevel::Info, "[ARMGDB] Rcmd: \"%s\"\n", cmd);
    if (!strcmp((const char*)cmd, "reset") || !strcmp((const char*)cmd, "r"))
    {
        Reset();
        return 0;
    }

    return 1; // not implemented (yet)
}

void ARMv5::WriteMem(u32 addr, int size, u32 v)
{
    if (addr < ITCMSize)
    {
        if (size == 8) *(u8*)&ITCM[addr & (ITCMPhysicalSize - 1)] = (u8)v;
        else if (size == 16) *(u16*)&ITCM[addr & (ITCMPhysicalSize - 1)] = (u16)v;
        else if (size == 32) *(u32*)&ITCM[addr & (ITCMPhysicalSize - 1)] = (u32)v;
        else {}
        return;
    }
    else if ((addr & DTCMMask) == DTCMBase)
    {
        if (size == 8) *(u8*)&DTCM[addr & (DTCMPhysicalSize - 1)] = (u8)v;
        else if (size == 16) *(u16*)&DTCM[addr & (DTCMPhysicalSize - 1)] = (u16)v;
        else if (size == 32) *(u32*)&DTCM[addr & (DTCMPhysicalSize - 1)] = (u32)v;
        else {}
        return;
    }

    ARM::WriteMem(addr, size, v);
}
u32 ARMv5::ReadMem(u32 addr, int size)
{
    if (addr < ITCMSize)
    {
        if (size == 8) return *(u8*)&ITCM[addr & (ITCMPhysicalSize - 1)];
        else if (size == 16) return *(u16*)&ITCM[addr & (ITCMPhysicalSize - 1)];
        else if (size == 32) return *(u32*)&ITCM[addr & (ITCMPhysicalSize - 1)];
        else return 0xfeedface;
    }
    else if ((addr & DTCMMask) == DTCMBase)
    {
        if (size == 8) return *(u8*)&DTCM[addr & (DTCMPhysicalSize - 1)];
        else if (size == 16) return *(u16*)&DTCM[addr & (DTCMPhysicalSize - 1)];
        else if (size == 32) return *(u32*)&DTCM[addr & (DTCMPhysicalSize - 1)];
        else return 0xfeedface;
    }

    return ARM::ReadMem(addr, size);
}
#endif

void ARMv4::DataRead8(u32 addr, u32* val)
{
    *val = BusRead8(addr);
    DataRegion = addr;
    DataCycles = NDS.ARM7MemTimings[addr >> 15][0];
}

void ARMv4::DataRead16(u32 addr, u32* val)
{
    addr &= ~1;

    *val = BusRead16(addr);
    DataRegion = addr;
    DataCycles = NDS.ARM7MemTimings[addr >> 15][0];
}

void ARMv4::DataRead32(u32 addr, u32* val)
{
    addr &= ~3;

    *val = BusRead32(addr);
    DataRegion = addr;
    DataCycles = NDS.ARM7MemTimings[addr >> 15][2];
}

void ARMv4::DataRead32S(u32 addr, u32* val)
{
    addr &= ~3;

    *val = BusRead32(addr);
    DataCycles += NDS.ARM7MemTimings[addr >> 15][3];
}

void ARMv4::DataWrite8(u32 addr, u8 val)
{
    BusWrite8(addr, val);
    DataRegion = addr;
    DataCycles = NDS.ARM7MemTimings[addr >> 15][0];
}

void ARMv4::DataWrite16(u32 addr, u16 val)
{
    addr &= ~1;

    BusWrite16(addr, val);
    DataRegion = addr;
    DataCycles = NDS.ARM7MemTimings[addr >> 15][0];
}

void ARMv4::DataWrite32(u32 addr, u32 val)
{
    addr &= ~3;

    BusWrite32(addr, val);
    DataRegion = addr;
    DataCycles = NDS.ARM7MemTimings[addr >> 15][2];
}

void ARMv4::DataWrite32S(u32 addr, u32 val)
{
    addr &= ~3;

    BusWrite32(addr, val);
    DataCycles += NDS.ARM7MemTimings[addr >> 15][3];
}


void ARMv4::AddCycles_C()
{
    // code only. this code fetch is sequential.
    Cycles += NDS.ARM7MemTimings[CodeCycles][(CPSR&0x20)?1:3];
}

void ARMv4::AddCycles_CI(s32 num)
{
    // code+internal. results in a nonseq code fetch.
    Cycles += NDS.ARM7MemTimings[CodeCycles][(CPSR&0x20)?0:2] + num;
}

void ARMv4::AddCycles_CDI()
{
    // LDR/LDM cycles.
    s32 numC = NDS.ARM7MemTimings[CodeCycles][(CPSR&0x20)?0:2];
    s32 numD = DataCycles;

    if ((DataRegion >> 24) == 0x02) // mainRAM
    {
        if (CodeRegion == 0x02)
            Cycles += numC + numD;
        else
        {
            numC++;
            Cycles += std::max(numC + numD - 3, std::max(numC, numD));
        }
    }
    else if (CodeRegion == 0x02)
    {
        numD++;
        Cycles += std::max(numC + numD - 3, std::max(numC, numD));
    }
    else
    {
        Cycles += numC + numD + 1;
    }
}

void ARMv4::AddCycles_CD()
{
    // TODO: max gain should be 5c when writing to mainRAM
    s32 numC = NDS.ARM7MemTimings[CodeCycles][(CPSR&0x20)?0:2];
    s32 numD = DataCycles;

    if ((DataRegion >> 24) == 0x02)
    {
        if (CodeRegion == 0x02)
            Cycles += numC + numD;
        else
            Cycles += std::max(numC + numD - 3, std::max(numC, numD));
    }
    else if (CodeRegion == 0x02)
    {
        Cycles += std::max(numC + numD - 3, std::max(numC, numD));
    }
    else
    {
        Cycles += numC + numD;
    }
}

u8 ARMv5::BusRead8(u32 addr)
{
    return NDS.ARM9Read8(addr);
}

u16 ARMv5::BusRead16(u32 addr)
{
    return NDS.ARM9Read16(addr);
}

u32 ARMv5::BusRead32(u32 addr)
{
    return NDS.ARM9Read32(addr);
}

void ARMv5::BusWrite8(u32 addr, u8 val)
{
    NDS.ARM9Write8(addr, val);
}

void ARMv5::BusWrite16(u32 addr, u16 val)
{
    NDS.ARM9Write16(addr, val);
}

void ARMv5::BusWrite32(u32 addr, u32 val)
{
    NDS.ARM9Write32(addr, val);
}

u8 ARMv4::BusRead8(u32 addr)
{
    return NDS.ARM7Read8(addr);
}

u16 ARMv4::BusRead16(u32 addr)
{
    return NDS.ARM7Read16(addr);
}

u32 ARMv4::BusRead32(u32 addr)
{
    return NDS.ARM7Read32(addr);
}

void ARMv4::BusWrite8(u32 addr, u8 val)
{
    NDS.ARM7Write8(addr, val);
}

void ARMv4::BusWrite16(u32 addr, u16 val)
{
    NDS.ARM7Write16(addr, val);
}

void ARMv4::BusWrite32(u32 addr, u32 val)
{
    NDS.ARM7Write32(addr, val);
}
}

