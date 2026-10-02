/*
    Copyright 2016-2026 melonDS team, RSDuck

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

#include "ARMJIT_Compiler.h"

#include "../ARMJIT_Internal.h"
#include "../ARMInterpreter.h"
#include "../ARMJIT.h"
#include "../NDS.h"
#include "../ARMJIT_Global.h"
#include "../ARMJIT_x64/ARMJIT_Offsets.h"
#include "../LiteProfile.h"

#include <stdlib.h>
#include <cstddef>

using namespace Arm64Gen;

extern "C" void ARM_Ret();

namespace melonDS
{

#ifdef LITEV_JIT_PERFMAP
// ============================================================================
// LITEV_JIT_PERFMAP — emit a simpleperf / linux-perf "perf-<pid>.map" so the
// opaque anonymous-rwx JIT blob can be split per emitted region.
//
// PURELY OBSERVATIONAL: every function here only reads the emitter's already-
// computed RX pointers (GetRXPtr()) and writes a text file. It emits NO host
// instructions and never advances m_code, so flag-ON JIT codegen is byte-for-
// byte identical to flag-OFF -> guest emulation stays bit-exact / MP-timing-
// exact. RUNTIME-GATED: nothing is written unless an output directory is named
// at run time, so the flag can ship ON in a .dev build at zero file cost.
//
// Map line format (hex, absolute runtime addresses):  <addr> <size> <name>
// Output path resolution (first match wins; else the feature stays dormant):
//   1. env  LITEV_PERFMAP_DIR=<dir>              -> <dir>/perf-<pid>.map   (headless)
//   2. Android prop debug.litev.perfmap=/abs/dir -> /abs/dir/perf-<pid>.map
//   3. Android prop debug.litev.perfmap=1|true   -> /data/data/<pkg>/perf-<pid>.map
//      (the app's own writable data dir, read from /proc/self/cmdline; matches
//       simpleperf's app convention -> found automatically, NO root needed)
//
// ResetBlockCache wholesale-frees every block, so the file is TRUNCATED and the
// process-lifetime "static" stub regions (recorded once at Compiler construction)
// are rewritten at each epoch, after which the regenerated dispatchers/commit
// stubs and freshly compiled blocks append to it.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>
#include <unistd.h>
#ifdef __ANDROID__
#include <sys/system_properties.h>
#endif

namespace litev_perfmap
{
    struct Region { unsigned long long addr; unsigned long long size; std::string name; };

    static std::string        g_path;
    static FILE*              g_file  = nullptr;
    static int                g_state = 0;   // 0=unresolved, 1=armed, -1=disabled
    static std::vector<Region> g_static;     // process-lifetime regions, re-emitted each epoch
    static std::vector<Region> g_dynamic;    // per-block regions, ACCUMULATED across epochs
                                             // so a cache reset (BeginEpoch) does not wipe the
                                             // live blocks a concurrent profile is sampling.
                                             // Reused host addrs may collide (minor mis-attrib
                                             // noise) but the JIT blob becomes resolvable.

    // Resolve g_path once. Returns true iff an output location was named (armed).
    static bool ResolvePath()
    {
        const char* dir = getenv("LITEV_PERFMAP_DIR");
        std::string dirbuf;
#ifdef __ANDROID__
        char prop[PROP_VALUE_MAX] = {0};
        if ((!dir || !dir[0]) && __system_property_get("debug.litev.perfmap", prop) > 0 && prop[0])
        {
            if (prop[0] == '/') { dirbuf = prop; dir = dirbuf.c_str(); }
            else
            {
                // truthy-but-not-a-path -> derive the app's own writable data dir
                // from the process name (/proc/self/cmdline == package name).
                FILE* cf = fopen("/proc/self/cmdline", "rb");
                if (cf)
                {
                    char nm[256] = {0};
                    size_t n = fread(nm, 1, sizeof(nm) - 1, cf);
                    fclose(cf);
                    for (size_t i = 0; i < n; i++) { if (nm[i] == 0 || nm[i] == ':') { nm[i] = 0; break; } }
                    if (nm[0]) { dirbuf = std::string("/data/data/") + nm; dir = dirbuf.c_str(); }
                }
            }
        }
#endif
        if (!dir || !dir[0]) return false;   // not armed -> stay dormant

        char path[600];
        snprintf(path, sizeof(path), "%s/perf-%d.map", dir, (int)getpid());
        g_path = path;
        return true;
    }

    // Record a process-lifetime region (the Compiler-construction stubs). Buffered
    // only; flushed to the file by BeginEpoch (their RX addresses never change).
    static void AddStatic(const char* name, void* rxstart, void* rxend)
    {
        if ((u8*)rxend <= (u8*)rxstart) return;
        g_static.push_back({ (unsigned long long)(uintptr_t)rxstart,
                             (unsigned long long)((u8*)rxend - (u8*)rxstart),
                             std::string(name) });
    }

    // Start a fresh cache epoch: (re)open the file truncated, rewrite the static
    // regions. Called at the top of Compiler::Reset() (== every ResetBlockCache).
    static void BeginEpoch()
    {
        if (g_state == 0) g_state = ResolvePath() ? 1 : -1;
        if (g_state < 0) return;
        if (g_file) fclose(g_file);
        g_file = fopen(g_path.c_str(), "w");
        if (!g_file)
        {
            fprintf(stderr, "[litev-perfmap] failed to open %s\n", g_path.c_str());
            g_state = -1;
            return;
        }
        fprintf(stderr, "[litev-perfmap] writing %s (%zu static + %zu dynamic regions)\n",
                g_path.c_str(), g_static.size(), g_dynamic.size());
        for (const Region& r : g_static)
            fprintf(g_file, "%llx %llx %s\n", r.addr, r.size, r.name.c_str());
        // Re-emit every per-block region compiled so far so a mid-race cache reset
        // does not blank the map out from under a running profile.
        for (const Region& r : g_dynamic)
            fprintf(g_file, "%llx %llx %s\n", r.addr, r.size, r.name.c_str());
        fflush(g_file);
    }

    // Append one epoch region (dispatchers, commit stubs, compiled blocks).
    static void Add(const char* name, void* rxstart, void* rxend)
    {
        // Only accumulate/emit when ARMED (prop set) — otherwise a normal play
        // session would grow g_dynamic unbounded per block-compile. Unarmed
        // (g_state != 1) this is a cheap early-out.
        if (g_state != 1 || !g_file) return;
        if ((u8*)rxend <= (u8*)rxstart) return;
        g_dynamic.push_back({ (unsigned long long)(uintptr_t)rxstart,
                              (unsigned long long)((u8*)rxend - (u8*)rxstart),
                              std::string(name) });
        fprintf(g_file, "%llx %llx %s\n",
                (unsigned long long)(uintptr_t)rxstart,
                (unsigned long long)((u8*)rxend - (u8*)rxstart), name);
        fflush(g_file);
    }
}
#endif  // LITEV_JIT_PERFMAP

// liteDS-v2 Unit 2: prove the hand-maintained ARMJIT_Offsets.h values against the
// real ARM struct layout. These offsets are baked as immediates in the A64
// linkage/dispatch code, so a silent layout shift (e.g. a field inserted before
// CPSR) must fail the build, not the emulation. The layout is host-arch-
// independent, but the assert is kept under the A64 guard to mirror where the
// offsets are consumed.
#ifdef __aarch64__
static_assert(offsetof(ARM, Cycles) == ARM_Cycles_offset,
    "ARM_Cycles_offset out of sync with ARM::Cycles");
static_assert(offsetof(ARM, StopExecution) == ARM_StopExecution_offset,
    "ARM_StopExecution_offset out of sync with ARM::StopExecution");
static_assert(offsetof(ARM, CPSR) == ARM_CPSR_offset,
    "ARM_CPSR_offset out of sync with ARM::CPSR");
static_assert(offsetof(ARM, CyclesBudget) == ARM_CyclesBudget_offset,
    "ARM_CyclesBudget_offset out of sync with ARM::CyclesBudget");
#ifdef LITEV_JIT_LAZYFLAGS
static_assert(offsetof(ARM, JitNZCV) == ARM_JitNZCV_offset,
    "ARM_JitNZCV_offset out of sync with ARM::JitNZCV (V2 NZCV mirror slot)");
#endif
// Unit 3 dispatcher lookup fields (see ARMJIT_Offsets.h). Proven here so a layout
// shift fails the build rather than corrupting the emitted inline block lookup.
static_assert(offsetof(ARM, FastBlockLookupStart) == ARM_FastBlockLookupStart_offset,
    "ARM_FastBlockLookupStart_offset out of sync with ARM::FastBlockLookupStart");
static_assert(offsetof(ARM, FastBlockLookupSize) == ARM_FastBlockLookupSize_offset,
    "ARM_FastBlockLookupSize_offset out of sync with ARM::FastBlockLookupSize");
static_assert(offsetof(ARM, FastBlockLookup) == ARM_FastBlockLookup_offset,
    "ARM_FastBlockLookup_offset out of sync with ARM::FastBlockLookup");
#ifdef LITEV_JIT_GLOBALREG
// GLOBALREG: the ARM_Dispatch/ARM_Ret linkage loads/spills pinned guest regs at
// ARM_R_offset + reg*4; prove that base against the real ARM::R[] layout.
static_assert(offsetof(ARM, R) == ARM_R_offset,
    "ARM_R_offset out of sync with ARM::R");
#endif
#endif

#ifdef LITEV_JIT_GLOBALREG
// GLOBALREG global fixed register map. STEP 1 pins the single hottest never-banked
// guest reg (r0) to a callee-saved host reg (W19). Only r0..r7 are eligible: they
// are never mode-banked, so a mode/exception switch (which reorders R_FIQ/R_SVC/...
// via the register file) never relocates them and needs no sync. The host regs are
// the leading entries of NativeRegAllocOrder (callee-saved => preserved for free
// across the emitted dispatcher, linked chains, and any C helper BL). Widening =
// append pairs here AND the matching `ldr/str wNN` in ARMJIT_Linkage.S.
static const struct { int GuestReg; Arm64Gen::ARM64Reg HostReg; } GlobalRegPins[] =
{
    // STEP 2: the full free callee-saved pool. AArch64 reserves x26/x27/x28/x29
    // (RMemBase/RCPSR/RCycles/RCPU), so W19..W25 (7 regs) are the architectural
    // maximum for a global pin here — a full DraStic r0..r14 (15-reg) pin is not
    // representable on this host without evicting the fastmem/CPSR/cycle regs.
    // Guest r0..r6 (never mode-banked) map to the leading NativeRegAllocOrder
    // entries the dynamic cache used to hand out.
    { 0, Arm64Gen::W19 },
    { 1, Arm64Gen::W20 },
    { 2, Arm64Gen::W21 },
    { 3, Arm64Gen::W22 },
    { 4, Arm64Gen::W23 },
    { 5, Arm64Gen::W24 },
    { 6, Arm64Gen::W25 },
#if defined(LITEV_JIT_LAZYFLAGS)
    // FULL LAZY-FLAGS frees W27 (formerly RCPSR, the whole-CPSR carrier) as the 8th
    // global pin: guest r7 -> W27. Only when SWTABLE is OFF (else r7->W26 above and W27
    // is left as scratch-pool relief). ARM_Dispatch/Ret load/spill w27 in lockstep.
    { 7, Arm64Gen::W27 },
#endif
};
static constexpr int NumGlobalRegPins = sizeof(GlobalRegPins) / sizeof(GlobalRegPins[0]);
// r0..r5 are always pinned; r6 unless it is repurposed as the down-counter's Ran
// accumulator; r7 when the sw-table (W26) or full lazy-flags (W27) frees a host reg.
// Byte-identical to the previous #if-ladder for every pre-LAZYFLAGS combo.
static constexpr u16 GlobalRegPinnedMask =
    0x003F
    | 0x0040   // r6
#if defined(LITEV_JIT_LAZYFLAGS)
    | 0x0080   // r7 (W26 via sw-table, or W27 freed by lazy-flags)
#endif
    ;
#endif

/*

    Recompiling classic ARM to ARMv8 code is at the same time
    easier and trickier than compiling to a less related architecture
    like x64. At one hand you can translate a lot of instructions directly.
    But at the same time, there are a ton of exceptions, like for
    example ADD and SUB can't have a RORed second operand on ARMv8.
 
    While writing a JIT when an instruction is recompiled into multiple ones
    not to write back until you've read all the other operands!
*/

template <>
const ARM64Reg RegisterCache<Compiler, ARM64Reg>::NativeRegAllocOrder[] =
{
    W19, W20, W21, W22, W23, W24, W25,
#ifdef LITEV_JIT_BUDGET_REG
    W8, W9, W10, W11, W12, W13, W14     // W15 = slice budget (RBudget)
};
template <>
const int RegisterCache<Compiler, ARM64Reg>::NativeRegsAvailable = 14;

const BitSet32 CallerSavedPushRegs({W8, W9, W10, W11, W12, W13, W14});
#else
    W8, W9, W10, W11, W12, W13, W14, W15
};
template <>
const int RegisterCache<Compiler, ARM64Reg>::NativeRegsAvailable = 15;

const BitSet32 CallerSavedPushRegs({W8, W9, W10, W11, W12, W13, W14, W15});
#endif

#ifdef LITEV_JIT_BUDGET_REG
// Every C++ helper call: spill the budget, materialize Timestamp = JitTsBase - budget
// (the exact block-start time the per-hop commit used to leave in memory), call, then
// reload the budget (a helper may have forced an exit = zeroed it). X16/X17 are never
// allocated; X15 is free once spilled. Args in X0-X7 and `scratchreg` are untouched.
void Compiler::QuickCallFunction(ARM64Reg scratchreg, const void* func)
{
    EmitBudgetSpill(X16, X17, X15);
    ARM64XEmitter::QuickCallFunction(scratchreg, func);
    LDR(INDEX_UNSIGNED, RBudget, RCPU, offsetof(ARM, CyclesBudget));
}

// STR budget; Timestamp = JitTsBase - (s64)budget. Clobbers t0, t1, t2 (64-bit regs).
void Compiler::EmitBudgetSpill(ARM64Reg t0, ARM64Reg t1, ARM64Reg t2)
{
    STR(INDEX_UNSIGNED, RBudget, RCPU, offsetof(ARM, CyclesBudget));
    SXTW(t1, RBudget);
    LDR(INDEX_UNSIGNED, t0, RCPU, offsetof(ARM, JitTsBase));
    SUB(t0, t0, t1);
    LDR(INDEX_UNSIGNED, t2, RCPU, offsetof(ARM, JitTsPtr));
    STR(INDEX_UNSIGNED, t0, t2, 0);
}
#endif

void Compiler::MovePC()
{
    ADD(MapReg(15), MapReg(15), Thumb ? 2 : 4);
}

void Compiler::A_Comp_MRS()
{
    Comp_AddCycles_C();

    ARM64Reg rd = MapReg(CurInstr.A_Reg(12));

    if (CurInstr.Instr & (1 << 22))
    {
#ifdef LITEV_JIT_LAZYFLAGS
        LDR(INDEX_UNSIGNED, W5, RCPU, offsetof(ARM, CPSR));   // control-only (mode) read
        ANDI2R(W5, W5, 0x1F);
#else
        ANDI2R(W5, RCPSR, 0x1F);
#endif
        MOVI2R(W3, 0);
        MOVI2R(W1, 15 - 8);
        BL(ReadBanked);
        MOV(rd, W3);
    }
    else
    {
#ifdef LITEV_JIT_LAZYFLAGS
        // V2 full CPSR read: the guest CPSR word = ARM::CPSR control bits [27:0] (always
        // canonical) merged with the JitNZCV slot's NZCV nibble [31:28]. Flush any host-
        // resident guest flags to the slot first (Comp_MaterializeFlags -> slot; for an
        // unconditional MRS ReconcileFlags already did this via ReadFlags, so it is a no-op
        // then). Then rd = (CPSR & 0x0FFFFFFF) | (slot & 0xF0000000). rd is a mapped guest
        // reg (never W0/W1), so W0/W1 are safe scratch.
        Comp_MaterializeFlags();
        LDR(INDEX_UNSIGNED, rd, RCPU, offsetof(ARM, CPSR));
        LDR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, JitNZCV));
        ANDI2R(rd, rd, 0x0FFFFFFF, W1);
        ORR(rd, rd, W0);
#else
        MOV(rd, RCPSR);
#endif
    }
}

void UpdateModeTrampoline(ARM* arm, u32 oldmode, u32 newmode)
{
    arm->UpdateMode(oldmode, newmode);
}

void Compiler::A_Comp_MSR()
{
    Comp_AddCycles_C();

    ARM64Reg val;
    if (CurInstr.Instr & (1 << 25))
    {
        val = W0;
        MOVI2R(val, melonDS::ROR((CurInstr.Instr & 0xFF), ((CurInstr.Instr >> 7) & 0x1E)));
    }
    else
    {
        val = MapReg(CurInstr.A_Reg(0));
    }

    u32 mask = 0;
    if (CurInstr.Instr & (1<<16)) mask |= 0x000000FF;
    if (CurInstr.Instr & (1<<17)) mask |= 0x0000FF00;
    if (CurInstr.Instr & (1<<18)) mask |= 0x00FF0000;
    if (CurInstr.Instr & (1<<19)) mask |= 0xFF000000;

    if (CurInstr.Instr & (1 << 22))
    {
#ifdef LITEV_JIT_LAZYFLAGS
        LDR(INDEX_UNSIGNED, W5, RCPU, offsetof(ARM, CPSR));   // mode (control) read
        ANDI2R(W5, W5, 0x1F);
#else
        ANDI2R(W5, RCPSR, 0x1F);
#endif
        MOVI2R(W3, 0);
        MOVI2R(W1, 15 - 8);
        BL(ReadBanked);

        MOVI2R(W1, mask);
        MOVI2R(W2, mask & 0xFFFFFF00);
#ifdef LITEV_JIT_LAZYFLAGS
        LDR(INDEX_UNSIGNED, W5, RCPU, offsetof(ARM, CPSR));   // mode (control) read
        ANDI2R(W5, W5, 0x1F);
#else
        ANDI2R(W5, RCPSR, 0x1F);
#endif
        CMP(W5, 0x10);
        CSEL(W1, W2, W1, CC_EQ);

        BIC(W3, W3, W1);
        AND(W0, val, W1);
        ORR(W3, W3, W0);

        MOVI2R(W1, 15 - 8);

        BL(WriteBanked);
    }
    else
    {
        mask &= 0xFFFFFFDF;
        CPSRDirty = true;

#ifdef LITEV_JIT_LAZYFLAGS
        // MSR can write the flag byte; its InstrInfo WriteFlags==0xF makes ReconcileFlags's
        // full-producer fast path SKIP the flush, yet MSR does NOT set host NZCV. Flush any
        // host-resident guest flags to memory NOW so the following memory RMW is not later
        // clobbered by a stale materialize (and NZCVDeferred is cleared).
        Comp_MaterializeFlags();
        if ((mask & 0xFF) == 0)
        {
            LDR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, CPSR));
            ANDI2R(W1, W1, ~mask);
            ANDI2R(W0, val, mask);
            ORR(W1, W1, W0);
            STR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, CPSR));
        }
        else
        {
            LDR(INDEX_UNSIGNED, W4, RCPU, offsetof(ARM, CPSR));   // W4 = CPSR word
            MOVI2R(W2, mask);
            MOVI2R(W3, mask & 0xFFFFFF00);
            ANDI2R(W1, W4, 0x1F);
            // W1 = first argument (old mode)
            CMP(W1, 0x10);
            CSEL(W2, W3, W2, CC_EQ);

            BIC(W4, W4, W2);
            AND(W0, val, W2);
            ORR(W4, W4, W0);
            STR(INDEX_UNSIGNED, W4, RCPU, offsetof(ARM, CPSR));

            MOV(W2, W4);   // newmode arg
            MOV(X0, RCPU);

            PushRegs(true, true);
            QuickCallFunction(X3, UpdateModeTrampoline);
            PopRegs(true, true);
        }

        // V2: if this MSR wrote the guest flag byte, ARM::CPSR[31:28] now holds the NEW
        // guest NZCV; refresh the JitNZCV slot (the in-slice NZCV home) from it so the
        // resuming JIT observes the new flags. If the flag byte was NOT written, the slot
        // still holds the canonical NZCV (untouched here) and CPSR[31:28] is stale-but-
        // harmless (re-synced at the next escape / the slice merge), so the slot is left
        // alone. NZCVDeferred is 0 (Comp_MaterializeFlags above); W0/W1 are free.
        if (mask & 0xF0000000)
        {
            LDR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, CPSR));
            ANDI2R(W0, W0, 0xF0000000, W1);
            STR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, JitNZCV));
        }
    }
#else
        if ((mask & 0xFF) == 0)
        {
            ANDI2R(RCPSR, RCPSR, ~mask);
            ANDI2R(W0, val, mask);
            ORR(RCPSR, RCPSR, W0);
        }
        else
        {
            MOVI2R(W2, mask);
            MOVI2R(W3, mask & 0xFFFFFF00);
            ANDI2R(W1, RCPSR, 0x1F);
            // W1 = first argument
            CMP(W1, 0x10);
            CSEL(W2, W3, W2, CC_EQ);

            BIC(RCPSR, RCPSR, W2);
            AND(W0, val, W2);
            ORR(RCPSR, RCPSR, W0);

            MOV(W2, RCPSR);
            MOV(X0, RCPU);

            PushRegs(true, true);
            QuickCallFunction(X3, UpdateModeTrampoline);
            PopRegs(true, true);
        }
    }
#endif
}


void Compiler::PushRegs(bool saveHiRegs, bool saveRegsToBeChanged, bool allowUnload)
{
    BitSet32 loadedRegs(RegCache.LoadedRegs);

    if (saveHiRegs)
    {
        BitSet32 hiRegsLoaded(RegCache.LoadedRegs & 0x7F00);
        for (int reg : hiRegsLoaded)
        {
            if (Thumb || CurInstr.Cond() == 0xE)
                RegCache.UnloadRegister(reg);
            else
                SaveReg(reg, RegCache.Mapping[reg]);
            // prevent saving the register twice
            loadedRegs[reg] = false;
        }
    }

    for (int reg : loadedRegs)
    {
        if (CallerSavedPushRegs[RegCache.Mapping[reg]]
            && (saveRegsToBeChanged || !((1<<reg) & CurInstr.Info.DstRegs && !((1<<reg) & CurInstr.Info.SrcRegs))))
        {
            if ((Thumb || CurInstr.Cond() == 0xE) && !((1 << reg) & (CurInstr.Info.DstRegs|CurInstr.Info.SrcRegs)) && allowUnload)
                RegCache.UnloadRegister(reg);
            else
                SaveReg(reg, RegCache.Mapping[reg]);
        }
    }
}

void Compiler::PopRegs(bool saveHiRegs, bool saveRegsToBeChanged)
{
    BitSet32 loadedRegs(RegCache.LoadedRegs);
    for (int reg : loadedRegs)
    {
        if ((saveHiRegs && reg >= 8 && reg < 15)
            || (CallerSavedPushRegs[RegCache.Mapping[reg]]
                && (saveRegsToBeChanged || !((1<<reg) & CurInstr.Info.DstRegs && !((1<<reg) & CurInstr.Info.SrcRegs)))))
        {
            LoadReg(reg, RegCache.Mapping[reg]);
        }
    }
}

Compiler::Compiler(melonDS::NDS& nds) : Arm64Gen::ARM64XEmitter(), NDS(nds)
{
#ifdef __SWITCH__
    JitRWBase = aligned_alloc(0x1000, JitMemSize);

    JitRXStart = (u8*)&__start__ - JitMemSize - 0x1000;
    virtmemLock();
    JitRWStart = virtmemFindAslr(JitMemSize, 0x1000);
    MemoryInfo info = {0};
    u32 pageInfo = {0};
    int i = 0;
    while (JitRXStart != NULL)
    {
        svcQueryMemory(&info, &pageInfo, (u64)JitRXStart);
        if (info.type != MemType_Unmapped)
            JitRXStart = (void*)((u8*)info.addr - JitMemSize - 0x1000);
        else
            break;
        if (i++ > 8)
        {
            Log(LogLevel::Error, "couldn't find unmapped place for jit memory\n");
            JitRXStart = NULL;
        }
    }

    assert(JitRXStart != NULL);

    bool succeded = R_SUCCEEDED(svcMapProcessCodeMemory(envGetOwnProcessHandle(), (u64)JitRXStart, (u64)JitRWBase, JitMemSize));
    assert(succeded);
    succeded = R_SUCCEEDED(svcSetProcessMemoryPermission(envGetOwnProcessHandle(), (u64)JitRXStart, JitMemSize, Perm_Rx));
    assert(succeded);
    succeded = R_SUCCEEDED(svcMapProcessMemory(JitRWStart, envGetOwnProcessHandle(), (u64)JitRXStart, JitMemSize));
    assert(succeded);

    virtmemUnlock();

    SetCodeBase((u8*)JitRWStart, (u8*)JitRXStart);
    JitMemMainSize = JitMemSize;
#else
    ARMJIT_Global::Init();

    CodeMemBase = ARMJIT_Global::AllocateCodeMem();
    nds.JIT.JitEnableWrite();

    SetCodeBase(reinterpret_cast<u8*>(CodeMemBase), reinterpret_cast<u8*>(CodeMemBase));
    JitMemMainSize = ARMJIT_Global::CodeMemorySliceSize;
#endif
    SetCodePtr(0);

    for (int i = 0; i < 3; i++)
    {
        JumpToFuncs9[i] = Gen_JumpTo9(i);
#ifdef LITEV_JIT_PERFMAP
        { char nm[24]; snprintf(nm, sizeof(nm), "jit_jumpto9_%d", i);
          litev_perfmap::AddStatic(nm, JumpToFuncs9[i], GetRXPtr()); }
#endif
        JumpToFuncs7[i] = Gen_JumpTo7(i);
#ifdef LITEV_JIT_PERFMAP
        { char nm[24]; snprintf(nm, sizeof(nm), "jit_jumpto7_%d", i);
          litev_perfmap::AddStatic(nm, JumpToFuncs7[i], GetRXPtr()); }
#endif
    }

    /*
        W4 - whether the register was written to
        W5 - mode
        W1 - reg num
        W3 - in/out value of reg
    */
    {
        ReadBanked = GetRXPtr();

        ADD(X2, RCPU, X1, ArithOption(X2, ST_LSL, 2));
        CMP(W5, 0x11);
        FixupBranch fiq = B(CC_EQ);
        SUBS(W1, W1, 13 - 8);
        ADD(X2, RCPU, X1, ArithOption(X2, ST_LSL, 2));
        FixupBranch notEverything = B(CC_LT);
        CMP(W5, 0x12);
        FixupBranch irq = B(CC_EQ);
        CMP(W5, 0x13);
        FixupBranch svc = B(CC_EQ);
        CMP(W5, 0x17);
        FixupBranch abt = B(CC_EQ);
        CMP(W5, 0x1B);
        FixupBranch und = B(CC_EQ);
        SetJumpTarget(notEverything);
        RET();

        SetJumpTarget(fiq);
        LDR(INDEX_UNSIGNED, W3, X2, offsetof(ARM, R_FIQ));
        RET();
        SetJumpTarget(irq);
        LDR(INDEX_UNSIGNED, W3, X2, offsetof(ARM, R_IRQ));
        RET();
        SetJumpTarget(svc);
        LDR(INDEX_UNSIGNED, W3, X2, offsetof(ARM, R_SVC));
        RET();
        SetJumpTarget(abt);
        LDR(INDEX_UNSIGNED, W3, X2, offsetof(ARM, R_ABT));
        RET();
        SetJumpTarget(und);
        LDR(INDEX_UNSIGNED, W3, X2, offsetof(ARM, R_UND));
        RET();
    }
#ifdef LITEV_JIT_PERFMAP
    litev_perfmap::AddStatic("jit_readbanked", ReadBanked, GetRXPtr());
#endif
    {
        WriteBanked = GetRXPtr();

        ADD(X2, RCPU, X1, ArithOption(X2, ST_LSL, 2));
        CMP(W5, 0x11);
        FixupBranch fiq = B(CC_EQ);
        SUBS(W1, W1, 13 - 8);
        ADD(X2, RCPU, X1, ArithOption(X2, ST_LSL, 2));
        FixupBranch notEverything = B(CC_LT);
        CMP(W5, 0x12);
        FixupBranch irq = B(CC_EQ);
        CMP(W5, 0x13);
        FixupBranch svc = B(CC_EQ);
        CMP(W5, 0x17);
        FixupBranch abt = B(CC_EQ);
        CMP(W5, 0x1B);
        FixupBranch und = B(CC_EQ);
        SetJumpTarget(notEverything);
        MOVI2R(W4, 0);
        RET();

        SetJumpTarget(fiq);
        STR(INDEX_UNSIGNED, W3, X2, offsetof(ARM, R_FIQ));
        MOVI2R(W4, 1);
        RET();
        SetJumpTarget(irq);
        STR(INDEX_UNSIGNED, W3, X2, offsetof(ARM, R_IRQ));
        MOVI2R(W4, 1);
        RET();
        SetJumpTarget(svc);
        STR(INDEX_UNSIGNED, W3, X2, offsetof(ARM, R_SVC));
        MOVI2R(W4, 1);
        RET();
        SetJumpTarget(abt);
        STR(INDEX_UNSIGNED, W3, X2, offsetof(ARM, R_ABT));
        MOVI2R(W4, 1);
        RET();
        SetJumpTarget(und);
        STR(INDEX_UNSIGNED, W3, X2, offsetof(ARM, R_UND));
        MOVI2R(W4, 1);
        RET();
    }
#ifdef LITEV_JIT_PERFMAP
    litev_perfmap::AddStatic("jit_writebanked", WriteBanked, GetRXPtr());
#endif

    for (int consoleType = 0; consoleType < 2; consoleType++)
    {
        for (int num = 0; num < 2; num++)
        {
            for (int size = 0; size < 3; size++)
            {
                for (int reg = 0; reg < 32; reg++)
                {
                    // FULL LAZY-FLAGS pins guest r7 -> W27 (the freed RCPSR reg). A guest
                    // LDR/STR r7 through the fault-based fastmem path indexes these tables by
                    // the mapped host reg, so W27 needs its own patched load/store funcs (the
                    // shipping LAZYFLAGS config has SWTABLE OFF, so fault-fastmem is the memory
                    // mechanism). Missing -> a NULL PatchFunc -> wild BL -> SIGILL at boot.
#ifdef LITEV_JIT_LAZYFLAGS
                    if (!(reg == W4 || (reg >= W8 && reg <= W15) || (reg >= W19 && reg <= W25) || reg == W27))
                        continue;
#else
                    if (!(reg == W4 || (reg >= W8 && reg <= W15) || (reg >= W19 && reg <= W25)))
                        continue;
#endif
                    ARM64Reg rdMapped = (ARM64Reg)reg;
                    PatchedStoreFuncs[consoleType][num][size][reg] = GetRXPtr();
                    if (num == 0)
                    {
                        MOV(X1, RCPU);
                        MOV(W2, rdMapped);
                    }
                    else
                    {
                        MOV(W1, rdMapped);
                    }
                    ABI_PushRegisters(BitSet32({30}) | CallerSavedPushRegs);
                    if (consoleType == 0)
                    {
                        switch ((8 << size) |  num)
                        {
                        case 32: QuickCallFunction(X3, SlowWrite9<u32, 0>); break;
                        case 33: QuickCallFunction(X3, SlowWrite7<u32, 0>); break;
                        case 16: QuickCallFunction(X3, SlowWrite9<u16, 0>); break;
                        case 17: QuickCallFunction(X3, SlowWrite7<u16, 0>); break;
                        case 8: QuickCallFunction(X3, SlowWrite9<u8, 0>); break;
                        case 9: QuickCallFunction(X3, SlowWrite7<u8, 0>); break;
                        }
                    }
                    else
                    {
                        switch ((8 << size) |  num)
                        {
                        case 32: QuickCallFunction(X3, SlowWrite9<u32, 1>); break;
                        case 33: QuickCallFunction(X3, SlowWrite7<u32, 1>); break;
                        case 16: QuickCallFunction(X3, SlowWrite9<u16, 1>); break;
                        case 17: QuickCallFunction(X3, SlowWrite7<u16, 1>); break;
                        case 8: QuickCallFunction(X3, SlowWrite9<u8, 1>); break;
                        case 9: QuickCallFunction(X3, SlowWrite7<u8, 1>); break;
                        }
                    }
                    
                    ABI_PopRegisters(BitSet32({30}) | CallerSavedPushRegs);
                    RET();
#ifdef LITEV_JIT_PERFMAP
                    { char nm[40]; snprintf(nm, sizeof(nm), "jit_pstore_c%dn%ds%dr%d",
                        consoleType, num, size, reg);
                      litev_perfmap::AddStatic(nm, PatchedStoreFuncs[consoleType][num][size][reg], GetRXPtr()); }
#endif

                    for (int signextend = 0; signextend < 2; signextend++)
                    {
                        PatchedLoadFuncs[consoleType][num][size][signextend][reg] = GetRXPtr();
                        if (num == 0)
                            MOV(X1, RCPU);
                        ABI_PushRegisters(BitSet32({30}) | CallerSavedPushRegs);
                        if (consoleType == 0)
                        {
                            switch ((8 << size) |  num)
                            {
                            case 32: QuickCallFunction(X3, SlowRead9<u32, 0>); break;
                            case 33: QuickCallFunction(X3, SlowRead7<u32, 0>); break;
                            case 16: QuickCallFunction(X3, SlowRead9<u16, 0>); break;
                            case 17: QuickCallFunction(X3, SlowRead7<u16, 0>); break;
                            case 8: QuickCallFunction(X3, SlowRead9<u8, 0>); break;
                            case 9: QuickCallFunction(X3, SlowRead7<u8, 0>); break;
                            }
                        }
                        else
                        {
                            switch ((8 << size) |  num)
                            {
                            case 32: QuickCallFunction(X3, SlowRead9<u32, 1>); break;
                            case 33: QuickCallFunction(X3, SlowRead7<u32, 1>); break;
                            case 16: QuickCallFunction(X3, SlowRead9<u16, 1>); break;
                            case 17: QuickCallFunction(X3, SlowRead7<u16, 1>); break;
                            case 8: QuickCallFunction(X3, SlowRead9<u8, 1>); break;
                            case 9: QuickCallFunction(X3, SlowRead7<u8, 1>); break;
                            }
                        }
                        ABI_PopRegisters(BitSet32({30}) | CallerSavedPushRegs);
                        if (size == 32)
                            MOV(rdMapped, W0);
                        else if (signextend)
                            SBFX(rdMapped, W0, 0, 8 << size);
                        else
                            UBFX(rdMapped, W0, 0, 8 << size);
                        RET();
#ifdef LITEV_JIT_PERFMAP
                        { char nm[44]; snprintf(nm, sizeof(nm), "jit_pload_c%dn%ds%dse%dr%d",
                            consoleType, num, size, signextend, reg);
                          litev_perfmap::AddStatic(nm, PatchedLoadFuncs[consoleType][num][size][signextend][reg], GetRXPtr()); }
#endif
                    }
                }
            }
        }
    }

    FlushIcache();

    JitMemSecondarySize = 1024*1024*4;

    JitMemMainSize -= GetCodeOffset();
    JitMemMainSize -= JitMemSecondarySize;

    SetCodeBase((u8*)GetRWPtr(), (u8*)GetRXPtr());
}

Compiler::~Compiler()
{
#ifdef __SWITCH__
    if (JitRWStart != NULL)
    {
        bool succeded = R_SUCCEEDED(svcUnmapProcessMemory(JitRWStart, envGetOwnProcessHandle(), (u64)JitRXStart, JitMemSize));
        assert(succeded);
        succeded = R_SUCCEEDED(svcUnmapProcessCodeMemory(envGetOwnProcessHandle(), (u64)JitRXStart, (u64)JitRWBase, JitMemSize));
        assert(succeded);
        free(JitRWBase);
    }
#endif

    ARMJIT_Global::FreeCodeMem(CodeMemBase);
    ARMJIT_Global::DeInit();
}

void Compiler::LoadCycles()
{
    LDR(INDEX_UNSIGNED, RCycles, RCPU, offsetof(ARM, Cycles));
}

void Compiler::SaveCycles()
{
#ifdef LITEV_JIT_CYCLE_BATCH
    FlushPendingCycles();
#endif
    STR(INDEX_UNSIGNED, RCycles, RCPU, offsetof(ARM, Cycles));
}

void Compiler::LoadReg(int reg, ARM64Reg nativeReg)
{
    if (reg == 15)
        MOVI2R(nativeReg, R15);
    else
        LDR(INDEX_UNSIGNED, nativeReg, RCPU, offsetof(ARM, R) + reg*4);
}

void Compiler::SaveReg(int reg, ARM64Reg nativeReg)
{
    STR(INDEX_UNSIGNED, nativeReg, RCPU, offsetof(ARM, R) + reg*4);
}

void Compiler::LoadCPSR()
{
    assert(!CPSRDirty);
#ifdef LITEV_JIT_LAZYFLAGS
    // V2: no RCPSR register to reload — but the within-slice C++ escape we are returning
    // from (interpreter fallback / restoreCPSR trampoline) may have written the guest NZCV
    // straight into ARM::CPSR[31:28]. Refresh the JitNZCV slot (the in-slice NZCV home)
    // from ARM::CPSR so the resuming JIT reads the up-to-date flags. NZCVDeferred is 0 at
    // every LoadCPSR site (the matching SaveCPSR cleared it). W0/W1 are free.
    LDR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, CPSR));
    ANDI2R(W0, W0, 0xF0000000, W1);
    STR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, JitNZCV));
#else
    LDR(INDEX_UNSIGNED, RCPSR, RCPU, offsetof(ARM, CPSR));
#endif
}

void Compiler::SaveCPSR(bool markClean)
{
#ifdef LITEV_JIT_FIXEDREG
    // Reconcile any host-NZCV-resident guest flags into the canonical CPSR store before a
    // C escape / block boundary (mode switch, exception, interpreter fallback). Under
    // LAZYFLAGS this flushes host NZCV into the JitNZCV slot.
    Comp_MaterializeFlags();
#endif
#ifdef LITEV_JIT_LAZYFLAGS
    // V2: the materialize above put the guest NZCV in the JitNZCV slot and the control bits
    // are always canonical in ARM::CPSR — but ARM::CPSR[31:28] itself is now STALE (V2 does
    // NOT keep the full word current mid-slice). Merge the slot's NZCV nibble into the
    // ARM::CPSR word so the imminent within-slice C++/interpreter/trampoline reader observes
    // a fully-canonical CPSR (LoadCPSR refreshes the slot back afterwards). W0/W1/W2 free.
    LDR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, JitNZCV));   // W0 = NZCV<<28 (low bits 0)
    LDR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, CPSR));      // control (+ stale NZCV)
    ANDI2R(W1, W1, 0x0FFFFFFF, W2);                          // clear the stale NZCV nibble
    ORR(W1, W1, W0);
    STR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, CPSR));
    if (markClean)
        CPSRDirty = false;
#else
    if (CPSRDirty)
    {
        STR(INDEX_UNSIGNED, RCPSR, RCPU, offsetof(ARM, CPSR));
        CPSRDirty = CPSRDirty && !markClean;
    }
#endif
}

FixupBranch Compiler::CheckCondition(u32 cond)
{
#ifdef LITEV_JIT_FIXEDREG
    if (NZCVDeferred)
    {
        // Guest flags are resident in host NZCV. Reconcile them into RCPSR (so the
        // conditionally-skipped body still observes a canonical CPSR word).
        // Comp_MaterializeFlags leaves PSTATE untouched.
        bool condValid = NZCVCondValid;
        Comp_MaterializeFlags();
        if (condValid)
        {
            // Full guest NZCV resident (arithmetic producer): evaluate the guest
            // condition NATIVELY on the still-live host NZCV. CCFlags == the ARM
            // cond field and (cond ^ 1) is the AArch64 inversion, so we branch
            // (skip the body) exactly when the guest condition is false. Valid for
            // every cond 0..13 (CheckCondition is only called for cond < 0xE).
            return B((CCFlags)(cond ^ 1));
        }
        // Only N,Z were host-resident (logical producer); host C,V are invalid.
        // The materialize above wrote N,Z into the canonical CPSR store (C,V were
        // already there), so fall through to the standard word path — which under
        // LAZYFLAGS reloads the full guest CPSR from ARM::CPSR memory.
    }
#endif
#ifdef LITEV_JIT_LAZYFLAGS
    // V2: RCPSR is gone and the guest NZCV is homed in the JitNZCV slot (NZCV in [31:28],
    // low bits 0). Read the condition from the slot: if the FIXEDREG block above hit the
    // FR_HOST_NZ path it just materialised N,Z into the slot (C,V were already there); under
    // FR_MEMORY the slot is canonical. Load it into a scratch (W1) and evaluate exactly as
    // the RCPSR path did — MSR NZCV consumes [31:28] and TBNZ/TBZ test a single bit in
    // [28..31], so the zero low bits are irrelevant. LAZYFLAGS requires CONDFOLD.
    {
        LDR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, JitNZCV));
        if (cond >= 0x8)
        {
            _MSR(FIELD_NZCV, EncodeRegTo64(W1));
            return B((CCFlags)(cond ^ 1));
        }
        else
        {
            u8 bit = (28 + ((~(cond >> 1) & 1) << 1 | (cond >> 2 & 1) ^ (cond >> 1 & 1)));
            if (cond & 1)
                return TBNZ(W1, bit);
            else
                return TBZ(W1, bit);
        }
    }
#else
    if (cond >= 0x8)
    {
#ifdef LITEV_JIT_CONDFOLD
        // Compound condition (HI/LS/GE/LT/GT/LE). Guest CPSR lives in RCPSR with
        // NZCV in bits [31:28] — exactly the layout MSR NZCV consumes. Push the
        // guest flags into the host PSTATE and let the hardware evaluate the
        // condition natively. CCFlags is encoded identically to the ARM cond
        // field (CC_EQ..CC_LE == 0..13), and inverting an AArch64 condition is a
        // low-bit flip (cond ^ 1). We want to SKIP when the condition is FALSE,
        // so branch on the inverted condition. Bit-for-bit the same decision as
        // the flag-table path below, in 2 host instructions instead of 5.
        _MSR(FIELD_NZCV, EncodeRegTo64(RCPSR));
        return B((CCFlags)(cond ^ 1));
#else
        LSR(W1, RCPSR, 28);
        MOVI2R(W2, 1);
        LSLV(W2, W2, W1);
        ANDI2R(W2, W2, ARM::ConditionTable[cond], W3);

        return CBZ(W2);
#endif
    }
    else
    {
        u8 bit = (28 + ((~(cond >> 1) & 1) << 1 | (cond >> 2 & 1) ^ (cond >> 1 & 1)));

        if (cond & 1)
            return TBNZ(RCPSR, bit);
        else
            return TBZ(RCPSR, bit);
    }
#endif
}

#define F(x) &Compiler::A_Comp_##x
const Compiler::CompileFunc A_Comp[ARMInstrInfo::ak_Count] =
{
    // AND
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    // EOR
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    // SUB
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    // RSB
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    // ADD
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    // ADC
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    // SBC
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    // RSC
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    // ORR
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    // MOV
    F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp),
    F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp),
    // BIC
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    // MVN
    F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp),
    F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp),
    // TST
    F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp),
    // TEQ
    F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp),
    // CMP
    F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp),
    // CMN
    F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp),
    // Mul
    F(Mul), F(Mul), F(Mul_Long), F(Mul_Long), F(Mul_Long), F(Mul_Long), F(Mul_Short), F(Mul_Short), F(Mul_Short), F(Mul_Short), F(Mul_Short),
    // ARMv5 exclusives
    F(Clz), NULL, NULL, NULL, NULL, 
    
    // STR
    F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB),
    // STRB
    F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB),
    // LDR
    F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB),
    // LDRB
    F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB),
    // STRH
    F(MemHD), F(MemHD), F(MemHD), F(MemHD),
    // LDRD
    NULL, NULL, NULL, NULL,
    // STRD
    NULL, NULL, NULL, NULL,
    // LDRH
    F(MemHD), F(MemHD), F(MemHD), F(MemHD),
    // LDRSB
    F(MemHD), F(MemHD), F(MemHD), F(MemHD),
    // LDRSH
    F(MemHD), F(MemHD), F(MemHD), F(MemHD),
    // Swap
    NULL, NULL,
    // LDM, STM
    F(LDM_STM), F(LDM_STM),
    // Branch
    F(BranchImm), F(BranchImm), F(BranchImm), F(BranchXchangeReg), F(BranchXchangeReg),
    // Special
    NULL, F(MSR), F(MSR), F(MRS), NULL, NULL, NULL,
    &Compiler::Nop
};
#undef F
#define F(x) &Compiler::T_Comp_##x
const Compiler::CompileFunc T_Comp[ARMInstrInfo::tk_Count] =
{
    // Shift imm
    F(ShiftImm), F(ShiftImm), F(ShiftImm),
    // Add/sub tri operand
    F(AddSub_), F(AddSub_), F(AddSub_), F(AddSub_),
    // 8 bit imm
    F(ALUImm8), F(ALUImm8), F(ALUImm8), F(ALUImm8),
    // ALU
    F(ALU), F(ALU), F(ALU), F(ALU), F(ALU), F(ALU), F(ALU), F(ALU),
    F(ALU), F(ALU), F(ALU), F(ALU), F(ALU), F(ALU), F(ALU), F(ALU),
    // ALU hi reg
    F(ALU_HiReg), F(ALU_HiReg), F(ALU_HiReg),
    // PC/SP relative ops
    F(RelAddr), F(RelAddr), F(AddSP),
    // LDR PC rel
    F(LoadPCRel),
    // LDR/STR reg offset
    F(MemReg), F(MemReg), F(MemReg), F(MemReg),
    // LDR/STR sign extended, half
    F(MemRegHalf), F(MemRegHalf), F(MemRegHalf), F(MemRegHalf),
    // LDR/STR imm offset
    F(MemImm), F(MemImm), F(MemImm), F(MemImm),
    // LDR/STR half imm offset
    F(MemImmHalf), F(MemImmHalf),
    // LDR/STR sp rel
    F(MemSPRel), F(MemSPRel),
    // PUSH/POP
    F(PUSH_POP), F(PUSH_POP),
    // LDMIA, STMIA
    F(LDMIA_STMIA), F(LDMIA_STMIA),
    // Branch
    F(BCOND), F(BranchXchangeReg), F(BranchXchangeReg), F(B), F(BL_LONG_1), F(BL_LONG_2),
    // Unk, SVC
    NULL, NULL,
    F(BL_Merged)
};

bool Compiler::CanCompile(bool thumb, u16 kind)
{
    return (thumb ? T_Comp[kind] : A_Comp[kind]) != NULL;
}

#ifdef LITEV_JIT_DISPATCH
// liteDS-v2 Unit 3 — emitted per-CPU block dispatcher.
//
// Entered by an unconditional `B` from a block's exit tail. The exiting block has
// flushed all guest registers to memory and committed R[15] (the invariant upstream's
// C++ loop already relies on to recompute instrAddr after every ARM_Ret). RCPU(x29)/
// RCPSR(w27)/RCycles(w28) are live; RMemBase(x26) is reloaded by each block's own
// prologue; scratch w0-w7/x2-x7 are freely clobbered (the next block reloads its guest
// state from memory).
//
// It reproduces, bit-for-bit, the inter-block work of ARM::Execute<JIT>. Upstream runs:
//     run block K
//     if (StopExecution) { if (IRQ) TriggerIRQ(); if (Halted||IdleLoop) {..} }
//     Timestamp += Cycles; Cycles = 0;
// The order is load-bearing: the StopExecution handling (IRQ delivery, idle/halt skip)
// must observe Timestamp *without* block K's cycles, and only same-slice work that
// continues (chains) may advance Timestamp. Crucially `NDS::ScheduleEvent` schedules
// relative to ARMxTimestamp (not Timestamp+Cycles), so a stale mid-slice Timestamp
// would move every event a chained block schedules — hence the per-hop Timestamp update.
//
//   (a) StopExecution set  -> exit to C++ WITHOUT touching Timestamp/Cycles/budget, so
//       C++ runs its StopExecution handling (old Timestamp) then does Timestamp += Cycles.
//   (b) otherwise commit block K: Timestamp += Cycles; budget -= Cycles; Cycles = 0.
//   (c) budget <= 0 (natural slice end, or ForceExecutionExit which zeroes it) -> exit.
//   (d) region miss / lookup miss -> exit (Timestamp already advanced, Cycles 0).
//   (e) hit -> commit CPSR, br straight into the next block.
// All exit edges go to ARM_Ret (commits Cycles/CPSR, pops the callee frame).
#ifdef LITEV_JIT_DIRECTPATCH
// C-ABI trampolines the dispatcher BLs into (baked Compiler* as arg0). Defined after
// the Compiler methods below; forward-declared so Gen_Dispatcher can take their address.
static void LiteV_DirectPatchPromote(Compiler* c, u32 num, u32 site, u32 guestTarget, u64 hostEntry);
static void LiteV_DirectPatchDemote(Compiler* c, u32 num, u32 site);
#endif
void* Compiler::Gen_Dispatcher(u32 num)
{
    AlignCode16();
    void* res = GetRXPtr();

    // (Compiler has a member named NDS, so the bare name resolves to it; force the type.)
    // Bake the exact address of this CPU's Timestamp field. offsetof(NDS, ARMxTimestamp)
    // is multiple MB (NDS embeds the two ARM cores' huge PU/MemTimings arrays), which
    // overflows the scaled 12-bit immediate of a 64-bit LDR/STR — so address it at offset 0.
    void* tsPtr = (num == 0) ? (void*)&NDS.ARM9Timestamp : (void*)&NDS.ARM7Timestamp;

    // (a) StopExecution first — bounce out leaving RCycles = block K's cycles and Timestamp
    //     untouched (C++ handles IRQ/halt/idle with the pre-block Timestamp, then adds Cycles).
    LDR(INDEX_UNSIGNED, W2, RCPU, offsetof(ARM, StopExecution));
    FixupBranch exitStop = CBNZ(W2);

#ifdef LITEV_JIT_BUDGET_REG
    // (b) budget register: consume block K (Timestamp stays JitTsBase - budget; it is
    //     written only at the C++ exit tail below and before helper calls).
    (void)tsPtr;
    SUBS(RBudget, RBudget, RCycles);
    MOVI2R(RCycles, 0);
    FixupBranch exitBudget = B(CC_LE);
#else
    // (b) commit block K to the timeline: Timestamp += Cycles (64-bit, unshifted, exactly as
    //     `NDS.ARMxTimestamp += Cycles`), keep budget == Target-Timestamp, restart the
    //     accumulator at 0 for the next block. Cycles is always >= 0 so uxtw == the s32 value.
    MOVP2R(X4, tsPtr);
    LDR(INDEX_UNSIGNED, X5, X4, 0);
    ADD(X5, X5, RCycles, ArithOption(RCycles));      // x5 += (u32)RCycles
    STR(INDEX_UNSIGNED, X5, X4, 0);
    LDR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, CyclesBudget));
    SUB(W1, W1, RCycles);
    STR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, CyclesBudget));
    MOVI2R(RCycles, 0);

    // (c) slice over or forced exit? budget <= 0
    CMP(W1, 0);
    FixupBranch exitBudget = B(CC_LE);
#endif

    // (d) instrAddr = R[15] - ((CPSR&0x20)?2:4) == (R[15] - 4) + (thumb << 1), as in the loop
    LDR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, R[15]));
#ifdef LITEV_JIT_LAZYFLAGS
    // RCPSR(W27) is now guest r7; read the Thumb bit from the canonical ARM::CPSR memory
    // (the exiting block materialized NZCV + control is always current, so it is canonical).
    LDR(INDEX_UNSIGNED, W3, RCPU, offsetof(ARM, CPSR));
    UBFX(W3, W3, 5, 1);                               // W3 = Thumb bit
#else
    UBFX(W3, RCPSR, 5, 1);                            // W3 = Thumb bit
#endif
    SUB(W0, W0, 4);
    ADD(W0, W0, W3, ArithOption(W3, ST_LSL, 1));      // W0 = clean instrAddr

#ifdef LITEV_JIT_ICACHE
    // (d.5-ICACHE) per-site 2-way inline cache. W9 = site index (set by the exiting
    // block; 0 == "no cache", e.g. an unlinked EmitLinkExit hop). W0 = instrAddr is
    // PRESERVED (steps e/f/writeback below also rely on it). A hit requires the entry
    // epoch to match the live ARM::ICacheEpoch (O(1) invalidation) AND key0/key1 ==
    // instrAddr, then branches straight to the cached host block — skipping the region
    // bounds + the cache-cold FastBlockLookup gather. The commit (a)(b)(c) is already
    // done and the CPSR store is byte-identical to step (g), so a hit is timing-exact.
    FixupBranch icNoCache = CBZ(W9);
    MOVP2R(X2, (void*)ICacheTable[num]);
    ADD(X2, X2, X9, ArithOption(X9, ST_LSL, ICacheEntryShift)); // &ICacheTable[num][W9]
    LDR(INDEX_UNSIGNED, W3, X2, offsetof(ICacheEntry, epoch));
    LDR(INDEX_UNSIGNED, W4, RCPU, offsetof(ARM, ICacheEpoch));
    CMP(W3, W4);
    FixupBranch icEpochMiss = B(CC_NEQ);
    LDR(INDEX_UNSIGNED, W3, X2, offsetof(ICacheEntry, key0));
    CMP(W3, W0);
    FixupBranch icHit0 = B(CC_EQ);
    LDR(INDEX_UNSIGNED, W3, X2, offsetof(ICacheEntry, key1));
    CMP(W3, W0);
    FixupBranch icHit1 = B(CC_EQ);
    SetJumpTarget(icEpochMiss);
    SetJumpTarget(icNoCache);
    // miss -> fall through to the region-bounds + tag lookup, which WRITES BACK below.
#endif

    // (e) region bounds: offset = instrAddr - FastBlockLookupStart, exit if >= Size (unsigned;
    //     a single compare also catches instrAddr < Start via wraparound)
    LDR(INDEX_UNSIGNED, W3, RCPU, offsetof(ARM, FastBlockLookupStart));
    SUB(W4, W0, W3);
    LDR(INDEX_UNSIGNED, W5, RCPU, offsetof(ARM, FastBlockLookupSize));
    CMP(W4, W5);
#ifdef LITEV_JIT_REGION_CACHE
    // (e') window miss: try the cached regions (ARM::JitRegions, exact by construction)
    //      before giving up to C++. A hit makes it the current window, exactly as the
    //      C++ re-entry's JitSetupRegion would, then continues with the tag lookup.
    FixupBranch inRegion = B(CC_LO);
    FixupBranch rcHit[ARM::JitRegionCount];
    const u32 rcBase = offsetof(ARM, JitRegions);
    const u32 rcSize = sizeof(ARM::JitRegionEntry);
    for (int k = 0; k < ARM::JitRegionCount; k++)
    {
        LDR(INDEX_UNSIGNED, W3, RCPU, rcBase + k * rcSize + offsetof(ARM::JitRegionEntry, Start));
        SUB(W4, W0, W3);
        LDR(INDEX_UNSIGNED, W5, RCPU, rcBase + k * rcSize + offsetof(ARM::JitRegionEntry, Size));
        CMP(W4, W5);
        rcHit[k] = B(CC_LO);
    }
    FixupBranch exitRegion = B();
    FixupBranch rcToLookup[ARM::JitRegionCount];
    for (int k = 0; k < ARM::JitRegionCount; k++)
    {
        SetJumpTarget(rcHit[k]);
        LDR(INDEX_UNSIGNED, X6, RCPU, rcBase + k * rcSize + offsetof(ARM::JitRegionEntry, Lookup));
        STR(INDEX_UNSIGNED, W3, RCPU, offsetof(ARM, FastBlockLookupStart));
        STR(INDEX_UNSIGNED, W5, RCPU, offsetof(ARM, FastBlockLookupSize));
        STR(INDEX_UNSIGNED, X6, RCPU, offsetof(ARM, FastBlockLookup));
        rcToLookup[k] = B();
    }
    SetJumpTarget(inRegion);
#else
    FixupBranch exitRegion = B(CC_HS);
#endif

    // (f) inline tag lookup: entry = FastBlockLookup[offset/2]; tag = entry>>32 == (instrAddr|num)
    LDR(INDEX_UNSIGNED, X6, RCPU, offsetof(ARM, FastBlockLookup));
#ifdef LITEV_JIT_REGION_CACHE
    for (int k = 0; k < ARM::JitRegionCount; k++)
        SetJumpTarget(rcToLookup[k]);
#endif
    LSR(W4, W4, 1);
    LDR(X7, X6, ArithOption(W4, true));               // ldr x7, [x6, w4, uxtw #3]
    LSR(X2, X7, 32);                                  // W2 = tag (high word)
    FixupBranch miss;
    if (num == 0)
    {
        CMP(W2, W0);
        miss = B(CC_NEQ);
    }
    else
    {
        ORRI2R(W3, W0, 1);                            // ARM7 tag carries num bit
        CMP(W2, W3);
        miss = B(CC_NEQ);
    }

    // (g) hit: commit live CPSR to memory before entering the next block. Blocks are compiled
    //     assuming CPSR-in-memory is current at their entry (per-block CPSRDirty starts false,
    //     so an interpreter-fallback first instruction skips its SaveCPSR); upstream upholds
    //     this via ARM_Ret's CPSR store, which chaining bypasses.
#ifndef LITEV_JIT_LAZYFLAGS
    STR(INDEX_UNSIGNED, RCPSR, RCPU, offsetof(ARM, CPSR));
#endif
    // LAZYFLAGS: ARM::CPSR is already canonical in memory at every block exit (the exiting
    // block materialized host NZCV and control bits are never register-cached), so the
    // per-hop commit is unnecessary; W27 now holds guest r7 and must not be stored here.
    // low 32 bits of the entry (W7) are the sub-entry offset into the RX region; baked RXBase
    // == GetRXBase() (stable rebased base, the same one SubEntryOffset subtracts).
    MOVI2R(X3, (u64)GetRXBase());
#ifdef LITEV_JIT_ICACHE
    // Compute host addr into X6 (keep W0 = instrAddr for the cache key), fill this
    // site's 2-way entry (LRU: demote slot0->slot1, new -> slot0) stamped with the
    // live epoch, then branch. W9 == 0 means the exiting site opted out of caching.
    ADD(X6, X3, W7, ArithOption(W7));                 // X6 = absolute host block entry
    {
        FixupBranch wbSkip = CBZ(W9);
        MOVP2R(X2, (void*)ICacheTable[num]);
        ADD(X2, X2, X9, ArithOption(X9, ST_LSL, ICacheEntryShift));
        LDR(INDEX_UNSIGNED, W3, X2, offsetof(ICacheEntry, key0));
        STR(INDEX_UNSIGNED, W3, X2, offsetof(ICacheEntry, key1));   // key1 = old key0
        LDR(INDEX_UNSIGNED, X4, X2, offsetof(ICacheEntry, ptr0));
        STR(INDEX_UNSIGNED, X4, X2, offsetof(ICacheEntry, ptr1));   // ptr1 = old ptr0
        STR(INDEX_UNSIGNED, W0, X2, offsetof(ICacheEntry, key0));   // key0 = instrAddr
        STR(INDEX_UNSIGNED, X6, X2, offsetof(ICacheEntry, ptr0));   // ptr0 = host entry
        LDR(INDEX_UNSIGNED, W3, RCPU, offsetof(ARM, ICacheEpoch));
        STR(INDEX_UNSIGNED, W3, X2, offsetof(ICacheEntry, epoch));
        SetJumpTarget(wbSkip);
    }
  #if LITEV_PROFILE
    MOVP2R(X4, (void*)&melonDS::LiteProfile::g_Frame.DispatcherHits);
    LDR(INDEX_UNSIGNED, X5, X4, 0);
    ADD(X5, X5, 1);
    STR(INDEX_UNSIGNED, X5, X4, 0);
  #endif
#ifdef LITEV_JIT_DIRECTPATCH
    // A writeback == this site did NOT hit slot0 -> break its consecutive-slot0 streak.
    // If it is nonetheless PROMOTED, its guard permanently missed (primary target
    // changed) and we are re-resolving in the dispatcher -> DEMOTE (revert the exit-B).
    // W9 = site (0 = no cache); X6 = host entry (preserved across the demote call).
    {
        FixupBranch dpNoCache = CBZ(W9);
        MOVP2R(X2, (void*)ICacheTable[num]);
        ADD(X2, X2, X9, ArithOption(X9, ST_LSL, ICacheEntryShift));
        STR(INDEX_UNSIGNED, WZR, X2, offsetof(ICacheEntry, hitCount));   // streak reset
        LDR(INDEX_UNSIGNED, W3, X2, offsetof(ICacheEntry, promoted));
        FixupBranch dpNotPromoted = CBZ(W3);
        ABI_PushRegisters({6});                       // preserve X6 (host entry)
        MOV(W2, W9);                                  // arg2 = site
        MOVI2R(W1, num);                              // arg1 = num
        MOVP2R(X0, (void*)this);                      // arg0 = Compiler*
        QuickCallFunction(X16, (void*)&LiteV_DirectPatchDemote);
        ABI_PopRegisters({6});
        SetJumpTarget(dpNotPromoted);
        SetJumpTarget(dpNoCache);
    }
#endif
    BR(X6);
#else
    ADD(X0, X3, W7, ArithOption(W7));                 // add x0, x3, w7, uxtw
#if LITEV_PROFILE
    // Runtime dispatcher-resolved chain hops (compare to CommitStubEntries = linked hops -> the
    // ACTUAL hot-loop link coverage). X4/X5 are dead here; X0 (the branch target) is preserved.
    MOVP2R(X4, (void*)&melonDS::LiteProfile::g_Frame.DispatcherHits);
    LDR(INDEX_UNSIGNED, X5, X4, 0);
    ADD(X5, X5, 1);
    STR(INDEX_UNSIGNED, X5, X4, 0);
#endif
    BR(X0);
#endif

#ifdef LITEV_JIT_ICACHE
    // ICACHE hit tails (reached from (d.5-ICACHE); X2 still holds the entry pointer).
    // Under LAZYFLAGS ARM::CPSR is already canonical (as at step (g)); otherwise commit
    // the live RCPSR exactly as the plain hit path does.
    SetJumpTarget(icHit0);
  #ifndef LITEV_JIT_LAZYFLAGS
    STR(INDEX_UNSIGNED, RCPSR, RCPU, offsetof(ARM, CPSR));
  #endif
    LDR(INDEX_UNSIGNED, X5, X2, offsetof(ICacheEntry, ptr0));
  #if LITEV_PROFILE
    MOVP2R(X4, (void*)&melonDS::LiteProfile::g_Frame.ICacheHits);
    LDR(INDEX_UNSIGNED, X3, X4, 0);
    ADD(X3, X3, 1);
    STR(INDEX_UNSIGNED, X3, X4, 0);
  #endif
#ifdef LITEV_JIT_DIRECTPATCH
    // A slot0 hit is a consecutive-same-target (monomorphic) resolve. Bump this site's
    // streak; at DirectPatchThreshold, PROMOTE (emit a per-site guard stub + patch the
    // exit-B to it) via a C++ helper. A promoted site never re-enters here (its guard
    // takes the target directly), so this counter only runs for not-yet-promoted sites.
    // Live: X2=&entry, W0=instrAddr(guest target), W9=site, X5=host entry (ptr0).
    {
        // An ALREADY-promoted site can land here on a guard-miss fallback whose
        // target matches key0 (bimodal site). Skip the streak/promote work entirely:
        // the C++ promote call would be a useless round-trip (idempotent early-out).
        LDR(INDEX_UNSIGNED, W3, X2, offsetof(ICacheEntry, promoted));
        FixupBranch dpProm = CBNZ(W3);
        LDR(INDEX_UNSIGNED, W3, X2, offsetof(ICacheEntry, hitCount));
        ADD(W3, W3, 1);
        STR(INDEX_UNSIGNED, W3, X2, offsetof(ICacheEntry, hitCount));
        CMP(W3, DirectPatchThreshold);
        FixupBranch dpBelow = B(CC_LT);
        ABI_PushRegisters({5});                       // preserve X5 (host entry) across the call
        MOV(W3, W0);                                  // arg3 = guestTarget (from W0=instrAddr)
        MOV(X4, X5);                                  // arg4 = hostEntry
        MOV(W2, W9);                                  // arg2 = site
        MOVI2R(W1, num);                              // arg1 = num
        MOVP2R(X0, (void*)this);                      // arg0 = Compiler*
        QuickCallFunction(X16, (void*)&LiteV_DirectPatchPromote);
        ABI_PopRegisters({5});
        SetJumpTarget(dpBelow);
        SetJumpTarget(dpProm);
    }
#endif
    BR(X5);
    SetJumpTarget(icHit1);
  #ifndef LITEV_JIT_LAZYFLAGS
    STR(INDEX_UNSIGNED, RCPSR, RCPU, offsetof(ARM, CPSR));
  #endif
    LDR(INDEX_UNSIGNED, X5, X2, offsetof(ICacheEntry, ptr1));
  #if LITEV_PROFILE
    MOVP2R(X4, (void*)&melonDS::LiteProfile::g_Frame.ICacheHits);
    LDR(INDEX_UNSIGNED, X3, X4, 0);
    ADD(X3, X3, 1);
    STR(INDEX_UNSIGNED, X3, X4, 0);
  #endif
#ifdef LITEV_JIT_DIRECTPATCH
    // A slot1 hit breaks the consecutive-slot0 streak (site is bimodal, not monomorphic).
    STR(INDEX_UNSIGNED, WZR, X2, offsetof(ICacheEntry, hitCount));
#endif
    BR(X5);
#endif

    SetJumpTarget(exitStop);
    SetJumpTarget(exitBudget);
    SetJumpTarget(exitRegion);
    SetJumpTarget(miss);
#ifdef LITEV_JIT_BUDGET_REG
    // Back to C++: leave CyclesBudget / Timestamp exactly as the per-hop commit did
    // (on the StopExecution path block K's cycles are still in RCycles, uncommitted).
    EmitBudgetSpill(X1, X2, X3);
#endif
#if LITEV_PROFILE
    // Dispatcher exits to C++ (block-lookup miss / recompile churn + the rare slice-end).
    MOVP2R(X4, (void*)&melonDS::LiteProfile::g_Frame.DispatcherMisses);
    LDR(INDEX_UNSIGNED, X5, X4, 0);
    ADD(X5, X5, 1);
    STR(INDEX_UNSIGNED, X5, X4, 0);
#endif
    QuickTailCall(X0, ARM_Ret);

    return res;
}

void Compiler::EmitBlockExit()
{
#ifdef LITEV_JIT_LAZYFLAGS
    // The dispatcher no longer stores RCPSR, so ARM::CPSR memory MUST be canonical here.
    // In every path this exit is reached the flags were already flushed (block-end 1419 /
    // Comp_JumpTo / CheckCondition), so this is a compile-time no-op (NZCVDeferred==0 ->
    // emits nothing); it is the "flags-dirty-guarded" flush the design specifies, and it
    // makes the invariant explicit / robust rather than implicit.
    Comp_MaterializeFlags();
#endif
    LITE_PROFILE_ADD(melonDS::LiteProfile::g_Frame.DispatchOnlyExits);
#ifdef LITEV_JIT_ICACHE
    // Hand this dynamic exit its own per-site cache slot; the dispatcher (d.5-ICACHE)
    // reads W9 to index / fill the 2-way entry. Index 0 (assigned on table overflow)
    // disables caching for this site. Compile-time constant -> baked immediate.
#ifdef LITEV_JIT_DIRECTPATCH
    u32 dpSite = ICacheAssignSite();
    MOVI2R(W9, dpSite);
    // Record the RX offset of the patchable exit `B` so DIRECTPATCH can later rewrite
    // it to a guard stub. Compile-time; survives until the next ICacheReset (which
    // zeroes the whole table, incl. patchOff/hitCount/promoted). Site 0 = no cache.
    if (dpSite != 0 && ICacheTable[Num])
        ICacheTable[Num][dpSite].patchOff = (u32)((u8*)GetRXPtr() - GetRXBase());
#else
    MOVI2R(W9, ICacheAssignSite());
#endif
#endif
    B(DispatcherEntry[Num]);
}
#endif

#ifdef LITEV_JIT_LINK
// liteDS-v2 Unit 4 — a linkable static exit.
//
// A linked branch bypasses the dispatcher, so it must reproduce, at the SOURCE
// site, exactly the per-hop work the dispatcher does before entering the next
// block — otherwise a chained hop diverges from the pure-dispatch build. The
// order here mirrors Gen_Dispatcher bit-for-bit:
//   (a) StopExecution set  -> dispatcher (which, still holding this block's Cycles
//       and an un-advanced Timestamp, bounces to C++ identically to a plain exit);
//   (b) commit this block: Timestamp += Cycles; budget -= Cycles; Cycles = 0;
//   (c) budget <= 0        -> dispatcher (re-commits as a no-op, then exits);
//   (d) hit: commit CPSR, then the patchable `B`. UNLINKED it targets the
//       dispatcher (whose inline lookup finds the — possibly not-yet-compiled —
//       target); LINKED it is rewritten to branch straight into the target block.
// Because the slot only ever holds an unconditional `B`, it can be re-patched
// (link / unlink) concurrently with execution on ARMv8 without synchronisation.
void Compiler::EmitLinkExit(u32 targetAddr)
{
#ifdef LITEV_JIT_LAZYFLAGS
    // A linked hop bypasses the dispatcher AND (under LAZYFLAGS) neither the stub nor the
    // inline (d) stores RCPSR, so ARM::CPSR memory MUST be canonical before the exit. It
    // always is (the mid-block conditional edges reach here after Comp_JumpTo / CheckCondition
    // flushed, and the block-end after 1419), so this guarded flush is a compile-time no-op.
    Comp_MaterializeFlags();
#endif
    void* tsPtr = (Num == 0) ? (void*)&NDS.ARM9Timestamp : (void*)&NDS.ARM7Timestamp;

    // (a)
    LDR(INDEX_UNSIGNED, W2, RCPU, offsetof(ARM, StopExecution));
    FixupBranch toDispatchStop = CBNZ(W2);

#ifdef LITEV_JIT_BUDGET_REG
    // (b)+(c) in the budget register. On the slice end, undo the subtraction and let
    // the dispatcher do the exit (it re-runs (a)-(c) with the canonical state).
    (void)tsPtr;
    SUBS(RBudget, RBudget, RCycles);
    FixupBranch toBudgetEnd = B(CC_LE);
    MOVI2R(RCycles, 0);
#else
    // (b)
    MOVP2R(X4, tsPtr);
    LDR(INDEX_UNSIGNED, X5, X4, 0);
    ADD(X5, X5, RCycles, ArithOption(RCycles));      // x5 += (u32)RCycles
    STR(INDEX_UNSIGNED, X5, X4, 0);
    LDR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, CyclesBudget));
    SUB(W1, W1, RCycles);
    STR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, CyclesBudget));
    MOVI2R(RCycles, 0);

    // (c)
    CMP(W1, 0);
    FixupBranch toDispatchBudget = B(CC_LE);
#endif

    // (d)
#ifndef LITEV_JIT_LAZYFLAGS
    STR(INDEX_UNSIGNED, RCPSR, RCPU, offsetof(ARM, CPSR));
#endif
    // LAZYFLAGS: ARM::CPSR already canonical in memory at the linkable exit; W27 = guest r7.
#ifdef LITEV_JIT_ICACHE
    MOVI2R(W9, 0);   // a linkable exit opts out of the per-site cache (0 = no cache)
#endif
    u32 patchOffset = (u32)((u8*)GetRXPtr() - GetRXBase());
    B(DispatcherEntry[Num]);   // the patch slot (unlinked state)

#ifdef LITEV_JIT_BUDGET_REG
    SetJumpTarget(toBudgetEnd);
    ADD(RBudget, RBudget, RCycles);
    SetJumpTarget(toDispatchStop);
#else
    SetJumpTarget(toDispatchStop);
    SetJumpTarget(toDispatchBudget);
#endif
    B(DispatcherEntry[Num]);

    if (NumLinkExits < 2)
    {
        LinkExits[NumLinkExits].PatchOffset = patchOffset;
        LinkExits[NumLinkExits].TargetAddr = targetAddr;
        NumLinkExits++;
    }
    LITE_PROFILE_ADD(melonDS::LiteProfile::g_Frame.LinkSitesEmitted);
}

void Compiler::PatchLinkSite(u32 rxOffset, u32 targetRxOffset)
{
    ptrdiff_t delta = (ptrdiff_t)targetRxOffset - (ptrdiff_t)rxOffset;
    // A64 unconditional B reaches +-128MB; the whole block cache is a few MB.
    assert(delta >= -(1 << 27) && delta < (1 << 27));
    u32 instr = 0x14000000u | ((u32)((delta >> 2)) & 0x03FFFFFFu);

    u8* rwbase = GetWriteableRWPtr() - GetCodeOffset();   // m_rwbase
    *(u32*)(rwbase + rxOffset) = instr;

    u8* rxptr = GetRXBase() + rxOffset;
    __builtin___clear_cache((char*)rxptr, (char*)rxptr + 4);
}
#endif

#ifdef LITEV_JIT_DIRECTPATCH
void Compiler::DirectPatchWriteBranch(u32 rxOffset, u32 targetRxOffset)
{
    ptrdiff_t delta = (ptrdiff_t)targetRxOffset - (ptrdiff_t)rxOffset;
    // A64 unconditional B reaches +-128MB; a code slice is 32MB, so exit-site->stub,
    // stub->target and stub->dispatcher are all trivially in range.
    assert(delta >= -(1 << 27) && delta < (1 << 27));
    u32 instr = 0x14000000u | ((u32)((delta >> 2)) & 0x03FFFFFFu);

    u8* rwbase = GetWriteableRWPtr() - GetCodeOffset();   // m_rwbase
    *(u32*)(rwbase + rxOffset) = instr;

    u8* rxptr = GetRXBase() + rxOffset;
    __builtin___clear_cache((char*)rxptr, (char*)rxptr + 4);
}

// Promote a monomorphic exit site. Emits a per-site GUARD STUB and rewrites the exit
// `B dispatcher` to it. The stub reproduces the dispatcher's per-hop commit (a-d) EXACTLY
// (same ops/order -> identical Timestamp/budget/event schedule -> trace bit-exact), then
// a single guard `instrAddr == guestTarget`:
//   match  -> a DIRECT (statically-predicted) B into hostEntry -- skips the 2-way
//             epoch/key load chain AND the shared, mispredicted indirect BR;
//   miss / StopExecution / slice-end -> fall to the dispatcher, which re-commits as a
//             no-op (RCycles was zeroed by the stub) and resolves via ICACHE exactly as
//             before. W9 = site is still live at stub entry (the exit site set it and the
//             stub's (a-d) never touch W9), so the dispatcher gets it for its bookkeeping.
// Runs on the emu thread only (single JIT thread); the JitEnableWrite/Execute bracket
// keeps the whole write window in C++ text (Apple W^X-safe), a no-op on Android RWX.
void Compiler::DirectPatchPromote(u32 num, u32 site, u32 guestTarget, u64 hostEntry)
{
    if (site == 0 || site >= ICacheSites || !ICacheTable[num])
        return;
    ICacheEntry& e = ICacheTable[num][site];
    if (e.promoted) return;                              // idempotent
    if (e.patchOff == 0) { e.hitCount = 0; return; }     // no exit-site recorded
    // Never emit past the main region (CompileBlock resets at <16KB; keep a stub's
    // headroom). Back off (reset streak, retry later) rather than promote when tight.
    if ((ptrdiff_t)JitMemMainSize - GetCodeOffset() < 4096) { e.hitCount = 0; return; }

    NDS.JIT.JitEnableWrite();

    // --- emit the guard stub at the main high-water (append; advances m_code) ---
    AlignCode16();
    u32 stubOff = (u32)((u8*)GetRXPtr() - GetRXBase());

    void* dispatcher = DispatcherEntry[num];
    void* tsPtr = (num == 0) ? (void*)&NDS.ARM9Timestamp : (void*)&NDS.ARM7Timestamp;

    // (a) StopExecution -> dispatcher (bounce to C++, block K's Cycles intact).
    LDR(INDEX_UNSIGNED, W2, RCPU, offsetof(ARM, StopExecution));
    FixupBranch toDispStop = CBNZ(W2);

#ifdef LITEV_JIT_BUDGET_REG
    // (b)+(c) in the budget register; on the slice end the dispatcher (a)-(c) re-runs as a
    // no-op subtraction (RCycles = 0) and exits through its spill tail.
    (void)tsPtr;
    SUBS(RBudget, RBudget, RCycles);
    MOVI2R(RCycles, 0);
    FixupBranch toDispBudget = B(CC_LE);
#else
    // (b) commit block K: Timestamp += Cycles; budget -= Cycles; Cycles = 0.
    MOVP2R(X4, tsPtr);
    LDR(INDEX_UNSIGNED, X5, X4, 0);
    ADD(X5, X5, RCycles, ArithOption(RCycles));
    STR(INDEX_UNSIGNED, X5, X4, 0);
    LDR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, CyclesBudget));
    SUB(W1, W1, RCycles);
    STR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, CyclesBudget));
    MOVI2R(RCycles, 0);

    // (c) slice over? budget <= 0 -> dispatcher (re-commit no-op, then exits).
    CMP(W1, 0);
    FixupBranch toDispBudget = B(CC_LE);
#endif

    // (d) instrAddr = R[15] - ((CPSR&0x20)?2:4)  (== dispatcher step (d), byte for byte).
    LDR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, R[15]));
#ifdef LITEV_JIT_LAZYFLAGS
    LDR(INDEX_UNSIGNED, W3, RCPU, offsetof(ARM, CPSR));
    UBFX(W3, W3, 5, 1);
#else
    UBFX(W3, RCPSR, 5, 1);
#endif
    SUB(W0, W0, 4);
    ADD(W0, W0, W3, ArithOption(W3, ST_LSL, 1));         // W0 = instrAddr

    // guard: does the ACTUAL guest target still equal the promoted target?
    MOVI2R(W3, guestTarget);
    CMP(W0, W3);
    FixupBranch guardMiss = B(CC_NEQ);

    // hit: under LAZYFLAGS ARM::CPSR is already canonical (dispatcher step (g) no-op);
    // otherwise commit the live RCPSR exactly as the plain hit path does.
#ifndef LITEV_JIT_LAZYFLAGS
    STR(INDEX_UNSIGNED, RCPSR, RCPU, offsetof(ARM, CPSR));
#endif
#if LITEV_PROFILE
    MOVP2R(X4, (void*)&melonDS::LiteProfile::g_Frame.DirectGuardHits);
    LDR(INDEX_UNSIGNED, X5, X4, 0);
    ADD(X5, X5, 1);
    STR(INDEX_UNSIGNED, X5, X4, 0);
#endif
    B((const void*)hostEntry);            // DIRECT, statically-predicted hop into the target

    // guard miss: the guest target changed -> count (PROFILE only), fall to the
    // dispatcher, whose writeback path will DEMOTE this site. X4/X5 are dispatcher
    // scratch; W9 = site stays live (the counter bump must not touch it). The stop /
    // budget exits land PAST the counter (they are not guard misses).
    SetJumpTarget(guardMiss);
#if LITEV_PROFILE
    MOVP2R(X4, (void*)&melonDS::LiteProfile::g_Frame.DirectGuardMisses);
    LDR(INDEX_UNSIGNED, X5, X4, 0);
    ADD(X5, X5, 1);
    STR(INDEX_UNSIGNED, X5, X4, 0);
#endif
    SetJumpTarget(toDispStop);
    SetJumpTarget(toDispBudget);
    B(dispatcher);

    FlushIcache();                        // covers the align padding + the whole stub

    // --- patch the exit-site B -> the stub, then flip back to executable ---
    DirectPatchWriteBranch(e.patchOff, stubOff);

    NDS.JIT.JitEnableExecute();

    e.stubOff = stubOff;
    e.promoted = 1;
#if LITEV_PROFILE
    melonDS::LiteProfile::g_Frame.DirectPromotions.fetch_add(1, std::memory_order_relaxed);
#endif
}

// Demote a promoted site: revert its exit `B stub` back to `B dispatcher`. Called from
// the dispatcher writeback when a promoted site re-resolves (its guard permanently
// missed). The leaked stub is reclaimed at the next ResetBlockCache.
void Compiler::DirectPatchDemote(u32 num, u32 site)
{
    if (site == 0 || site >= ICacheSites || !ICacheTable[num]) return;
    ICacheEntry& e = ICacheTable[num][site];
    if (!e.promoted) return;

    NDS.JIT.JitEnableWrite();
    u32 dispOff = (u32)((u8*)DispatcherEntry[num] - GetRXBase());
    DirectPatchWriteBranch(e.patchOff, dispOff);
    NDS.JIT.JitEnableExecute();

    e.promoted = 0;
    e.hitCount = 0;
#if LITEV_PROFILE
    melonDS::LiteProfile::g_Frame.DirectDemotions.fetch_add(1, std::memory_order_relaxed);
#endif
}

// Revert EVERY live promotion back to the dispatcher. The sound backstop for block
// invalidation: RX is append-only within a cache epoch, so a promoted site's direct B
// could otherwise outlive its (retired-but-RX-preserved) target block. Cheap and rare
// (single-block invalidations ~4/3400f); over-reverting is harmless (sites re-promote).
void Compiler::DirectPatchRevertAll()
{
    bool any = false;
    NDS.JIT.JitEnableWrite();
    for (u32 num = 0; num < 2; num++)
    {
        if (!ICacheTable[num]) continue;
        u32 dispOff = (u32)((u8*)DispatcherEntry[num] - GetRXBase());
        u32 n = ICacheNextSite;   // only [1, ICacheNextSite) were ever handed out
        if (n > ICacheSites) n = ICacheSites;
        for (u32 s = 1; s < n; s++)
        {
            ICacheEntry& e = ICacheTable[num][s];
            if (e.promoted)
            {
                DirectPatchWriteBranch(e.patchOff, dispOff);
                e.promoted = 0;
                e.hitCount = 0;
                any = true;
            }
        }
    }
    NDS.JIT.JitEnableExecute();
#if LITEV_PROFILE
    if (any) melonDS::LiteProfile::g_Frame.DirectReverts.fetch_add(1, std::memory_order_relaxed);
#else
    (void)any;
#endif
}

static void LiteV_DirectPatchPromote(Compiler* c, u32 num, u32 site, u32 guestTarget, u64 hostEntry)
{
    c->DirectPatchPromote(num, site, guestTarget, hostEntry);
}

static void LiteV_DirectPatchDemote(Compiler* c, u32 num, u32 site)
{
    c->DirectPatchDemote(num, site);
}
#endif

void Compiler::Comp_BranchSpecialBehaviour(bool taken)
{
    if (taken && CurInstr.BranchFlags & branch_IdleBranch)
    {
        MOVI2R(W0, 1);
        STRB(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, IdleLoop));
#ifdef LITEV_JIT_DISPATCH
        // The dispatcher checks the budget, not IdleLoop; zero it so the exit bounces
        // to C++ (which performs the idle-skip). Keeps the ForceExecutionExit invariant:
        // every setter of StopExecution/Halted/IdleLoop also zeroes CyclesBudget.
#ifdef LITEV_JIT_BUDGET_REG
        // Register form of ForceExecutionExit: re-base JitTsBase so Timestamp stays the
        // block-start time with a zero budget.
        SXTW(X17, RBudget);
        LDR(INDEX_UNSIGNED, X16, RCPU, offsetof(ARM, JitTsBase));
        SUB(X16, X16, X17);
        STR(INDEX_UNSIGNED, X16, RCPU, offsetof(ARM, JitTsBase));
        MOVI2R(RBudget, 0);
#endif
        STR(INDEX_UNSIGNED, WZR, RCPU, offsetof(ARM, CyclesBudget));
#endif
    }

    if ((CurInstr.BranchFlags & branch_FollowCondNotTaken && taken)
        || (CurInstr.BranchFlags & branch_FollowCondTaken && !taken))
    {
        RegCache.PrepareExit();

#ifdef LITEV_JIT_CYCLE_BATCH
        // Mid-block exit: add the pending cycles on this path only; the path that
        // continues in the block keeps them deferred.
        u32 exitCycles = ConstantCycles + PendingCycles;
        for (; exitCycles > 0xFFF; exitCycles -= 0xFFF)
            ADD(RCycles, RCycles, 0xFFF);
        if (exitCycles)
            ADD(RCycles, RCycles, exitCycles);
#else
        if (ConstantCycles)
            ADD(RCycles, RCycles, ConstantCycles);
#endif
#ifdef LITEV_JIT_DISPATCH
  #if defined(LITEV_JIT_LINK) && defined(LITEV_LINK_COND)
        // Conditional-branch edges each have a single compile-time-constant next PC:
        //   taken edge     -> the static branch target (Comp_JumpTo(u32) just ran);
        //   not-taken edge -> the fall-through address after this instruction.
        // A dynamic conditional branch (e.g. conditional BX) leaves HasStaticExit
        // false on the taken edge -> fall back to the dispatcher for that edge.
        if (taken && HasStaticExit)
        {
            EmitLinkExit(StaticExitTarget);
        }
        else if (!taken)
        {
            EmitLinkExit(CurInstr.Addr + (Thumb ? 2 : 4));
        }
        else
        {
            EmitBlockExit();
        }
  #else
        EmitBlockExit();
  #endif
#else
        QuickTailCall(X0, ARM_Ret);
#endif
    }
}

JitBlockEntry Compiler::CompileBlock(ARM* cpu, bool thumb, FetchedInstr instrs[], int instrsCount, bool hasMemInstr)
{
    if (JitMemMainSize - GetCodeOffset() < 1024 * 16)
    {
        Log(LogLevel::Debug, "JIT near memory full, resetting...\n");
        NDS.JIT.ResetBlockCache();
    }
    if ((JitMemMainSize +  JitMemSecondarySize) - OtherCodeRegion < 1024 * 8)
    {
        Log(LogLevel::Debug, "JIT far memory full, resetting...\n");
        NDS.JIT.ResetBlockCache();
    }

    JitBlockEntry res = (JitBlockEntry)GetRXPtr();

    Thumb = thumb;
    Num = cpu->Num;
    CurCPU = cpu;
    ConstantCycles = 0;
#ifdef LITEV_JIT_CYCLE_BATCH
    PendingCycles = 0;
    DeferCycles = false;
#endif
    RegCache = RegisterCache<Compiler, ARM64Reg>(this, instrs, instrsCount, true);
#ifdef LITEV_JIT_GLOBALREG
    // GLOBALREG: install the fixed guest->host map. These regs are already resident
    // in their host regs (loaded by ARM_Dispatch at slice entry, preserved across
    // block boundaries / the dispatcher / linked chains), so the cache marks them
    // loaded WITHOUT emitting any per-block reload, and never spills them at a
    // block boundary. That removes the block-entry first-load + block-exit
    // writeback traffic for the pinned subset.
    for (int i = 0; i < NumGlobalRegPins; i++)
        RegCache.PinRegister(GlobalRegPins[i].GuestReg, GlobalRegPins[i].HostReg);
#endif
    CPSRDirty = false;
#ifdef LITEV_JIT_FIXEDREG
    NZCVDeferred = 0;
    NZCVCondValid = false;
#endif
#ifdef LITEV_JIT_LAZYFLAGS
    // V3 item 1: precompute the block's per-instruction flag LiveIn (barrier-conservative,
    // block exit all-live). Used by Comp_MaterializeFlags' item-1b upgrade to prove the
    // C,V it would preserve are dead. CompileBlock INTERPRETS the block on first compile
    // elsewhere, but this pass is pure analysis over CurInstr[] and emits nothing.
    Comp_ComputeFlagLiveness(instrs, instrsCount);
    FlagsLiveInCur = 0xF;
    CarryInHostResident = false;
#endif

#ifdef LITEV_JIT_LINK
    NumLinkExits = 0;
    HasStaticExit = false;
    LastInstrCompiledNonBranch = false;
#endif

    if (hasMemInstr)
        MOVP2R(RMemBase, Num == 0 ? NDS.JIT.Memory.FastMem9Start : NDS.JIT.Memory.FastMem7Start);

    for (int i = 0; i < instrsCount; i++)
    {
        CurInstr = instrs[i];
        R15 = CurInstr.Addr + (Thumb ? 4 : 8);
        CodeRegion = R15 >> 24;
#ifdef LITEV_JIT_LAZYFLAGS
        // V3: carry this instruction's flag LiveIn into the flush helpers, and clear the
        // per-instruction native-carry signal (set only by Comp_ReconcileFlags for this
        // instruction's ADC/SBC body).
        FlagsLiveInCur = FlagsLiveIn[i];
        CarryInHostResident = false;
#endif

#ifdef LITEV_JIT_LINK
        // Reset per instruction so a mid-block followed branch's static target does
        // not leak into the block-end exit decision.
        HasStaticExit = false;
#endif

        CompileFunc comp = Thumb
            ? T_Comp[CurInstr.Info.Kind]
            : A_Comp[CurInstr.Info.Kind];

        Exit = i == (instrsCount - 1) || (CurInstr.BranchFlags & branch_FollowCondNotTaken);

        //printf("%x instr %x regs: r%x w%x n%x flags: %x %x %x\n", R15, CurInstr.Instr, CurInstr.Info.SrcRegs, CurInstr.Info.DstRegs, CurInstr.Info.ReadFlags, CurInstr.Info.NotStrictlyNeeded, CurInstr.Info.WriteFlags, CurInstr.SetFlags);

        bool isConditional = Thumb ? CurInstr.Info.Kind == ARMInstrInfo::tk_BCOND : CurInstr.Cond() < 0xE;
#ifdef LITEV_JIT_CYCLE_BATCH
        // RCycles is read by SaveCycles (which flushes first) and at exits (which add the
        // pending cycles on the exit path). Flush up front for interpreter fallbacks and
        // conditional branches that may SaveCycles inside their skippable body (PC-writing ALU/LDM/LDR,
        // which can restore the CPSR). Plain conditional B/BL/BX/BLX and every other
        // conditional instruction leave the pending adds alone (nothing in the body reads
        // RCycles) but keep their own adds inline, since those run on one path only.
        // Unconditional instructions defer; the block-end add then carries everything.
        bool plainBranch = Thumb || CurInstr.Info.Kind == ARMInstrInfo::ak_B
            || CurInstr.Info.Kind == ARMInstrInfo::ak_BL || CurInstr.Info.Kind == ARMInstrInfo::ak_BX
            || CurInstr.Info.Kind == ARMInstrInfo::ak_BLX_REG;
        bool readsCycles = comp == NULL || (isConditional && CurInstr.Info.Branches() && !plainBranch);
        if (readsCycles)
            FlushPendingCycles();
        DeferCycles = !readsCycles && !isConditional;
#endif
        if (comp == NULL || (CurInstr.BranchFlags & branch_FollowCondTaken) || (i == instrsCount - 1 && (!CurInstr.Info.Branches() || isConditional)))
        {
            MOVI2R(W0, R15);
            STR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, R[15]));
            if (comp == NULL)
            {
                MOVI2R(W0, CurInstr.Instr);
                STR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, CurInstr));
            }
            if (Num == 0)
            {
                MOVI2R(W0, (s32)CurInstr.CodeCycles);
                STR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, CodeCycles));
            }
        }

        if (comp == NULL)
        {
            SaveCycles();
            SaveCPSR();
            RegCache.Flush();
#ifdef LITEV_JIT_GLOBALREG
            // GLOBALREG: the interpreter reads/writes the guest register FILE
            // directly, but pinned regs live in host regs and Flush() deliberately
            // did NOT spill them. Spill them to the file now so the interpreter
            // sees the authoritative value; the matching reload runs after the
            // interpreter call (the sole file-coherence point for pinned r0..r7).
            for (int p = 0; p < NumGlobalRegPins; p++)
                SaveReg(GlobalRegPins[p].GuestReg, GlobalRegPins[p].HostReg);
#endif
        }
        else
            RegCache.Prepare(Thumb, i);

        if (Thumb)
        {
#ifdef LITEV_JIT_FIXEDREG
            // Thumb instructions carry no per-instruction main-loop CheckCondition, so
            // reconcile any deferred host flags before the body. Stage 2b: spill ONLY
            // when this body forces it (reads a deferred flag, or clobbers host NZCV
            // without being a full producer); a transparent Thumb body keeps the flags
            // resident in host PSTATE across it.
            Comp_ReconcileFlags();
#endif
            if (comp == NULL)
            {
                MOV(X0, RCPU);
                QuickCallFunction(X1, InterpretTHUMB[CurInstr.Info.Kind]);
            }
            else
            {
                (this->*comp)();
            }
        }
        else
        {
            u32 cond = CurInstr.Cond();
            if (CurInstr.Info.Kind == ARMInstrInfo::ak_BLX_IMM)
            {
#ifdef LITEV_JIT_FIXEDREG
                Comp_MaterializeFlags();
#endif
                if (comp)
                    (this->*comp)();
                else
                {
                    MOV(X0, RCPU);
                    QuickCallFunction(X1, ARMInterpreter::A_BLX_IMM);
                }
            }
            else if (cond == 0xF)
            {
#ifdef LITEV_JIT_FIXEDREG
                Comp_MaterializeFlags();
#endif
                Comp_AddCycles_C();
            }
            else
            {
                IrregularCycles = comp == NULL;

                FixupBranch skipExecute;
                if (cond < 0xE)
                    skipExecute = CheckCondition(cond);
#ifdef LITEV_JIT_FIXEDREG
                else
                    // Unconditional (AL) body: no CheckCondition ran. Stage 2b: keep
                    // the resident flags alive across the body unless it reads a
                    // deferred flag or clobbers host NZCV without being a full producer.
                    Comp_ReconcileFlags();
#endif

                if (comp == NULL)
                {
                    MOV(X0, RCPU);
                    QuickCallFunction(X1, InterpretARM[CurInstr.Info.Kind]);
                }
                else
                {
                    (this->*comp)();
                }

                Comp_BranchSpecialBehaviour(true);

                if (cond < 0xE)
                {
                    if (IrregularCycles || (CurInstr.BranchFlags & branch_FollowCondTaken))
                    {
                        FixupBranch skipNop = B();
                        SetJumpTarget(skipExecute);

                        if (IrregularCycles)
                            Comp_AddCycles_C(true);

                        Comp_BranchSpecialBehaviour(false);

                        SetJumpTarget(skipNop);
                    }
                    else
                    {
                        SetJumpTarget(skipExecute);
                    }
                }

            }
        }

        if (comp == NULL)
        {
            LoadCycles();
            LoadCPSR();
#ifdef LITEV_JIT_GLOBALREG
            // GLOBALREG: the interpreter may have written the pinned guest regs in
            // the file (e.g. an ALU/LDM/SWI fallback targeting r0). Reload them into
            // their fixed host regs so the pinned invariant holds for the rest of
            // the block / the chained successor.
            for (int p = 0; p < NumGlobalRegPins; p++)
                LoadReg(GlobalRegPins[p].GuestReg, GlobalRegPins[p].HostReg);
#endif
        }

#ifdef LITEV_JIT_LINK
        // Only a JIT-compiled non-branch has a statically-known single next PC (an
        // interpreter fallback might mutate R15). Captured for the block-end tail.
        LastInstrCompiledNonBranch = (comp != NULL) && !CurInstr.Info.Branches();
        LastInstrFallthroughAddr = CurInstr.Addr + (Thumb ? 2 : 4);
#endif
#ifdef LITEV_JIT_LAZYFLAGS
        // CORRECTNESS FIX (carry/overflow deferral is unsafe): never carry the guest C/V
        // flags deferred-resident in the host PSTATE across an instruction boundary.
        // The deferral tracking failed to materialize host C,V before a later host op
        // clobbered them, so a subsequent guest conditional read stale C/V and took the
        // WRONG branch — e.g. Pokémon White's intro mis-branched in its 3D geometry
        // submission and dropped the professor's polygons (RenderNumPolygons 23->0 =>
        // solid-white professor). Shrek never hit the pattern, and the CPU/RAM state
        // re-converged, so both the Shrek gate and the state-trace oracle missed it;
        // only a framebuffer diff vs the interpreter catches it.
        //
        // Fix: materialize eagerly whenever C or V is deferred (flushes the arithmetic
        // producer's NZCV to the canonical JitNZCV slot now). The common logical-producer
        // N/Z-only deferral is retained, so most of the lazy-flags win is kept. Validated
        // byte-identical to the interpreter on Pokémon White (professor renders) and Shrek
        // (no regression). The slot structure and the r7->W27 global pin are untouched.
        if (NZCVDeferred & 0x3)
            Comp_MaterializeFlags();
#endif
    }

    RegCache.Flush();

#ifdef LITEV_JIT_LAZYFLAGS
    // V3: block boundary == all-live (slot must be fully canonical for ARM_Ret /
    // dispatcher). Force FlagsLiveInCur=0xF so the block-end flush NEVER takes the
    // item-1b C,V-clobber upgrade.
    FlagsLiveInCur = 0xF;
#endif
#ifdef LITEV_JIT_FIXEDREG
    // Block ends with the last instruction's flags possibly still resident in host
    // NZCV (e.g. a trailing unconditional CMP). The dispatcher / ARM_Ret stores the
    // live RCPSR at runtime, so reconcile now.
    Comp_MaterializeFlags();
#endif

#ifdef LITEV_JIT_CYCLE_BATCH
    DeferCycles = false;
    if (ConstantCycles + PendingCycles <= 0xFFF)
    {
        ConstantCycles += PendingCycles;   // both are added at this one point
        PendingCycles = 0;
    }
    else
        FlushPendingCycles();
#endif
    if (ConstantCycles)
        ADD(RCycles, RCycles, ConstantCycles);
#ifdef LITEV_JIT_DISPATCH
  #ifdef LITEV_JIT_LINK
    // Block-end exit eligibility:
    //   - an UNCONDITIONAL static branch (Comp_JumpTo(u32), not a BCOND) has one
    //     target and no converging not-taken path here -> LINK_UNCOND;
    //   - a JIT-compiled non-branch falls through to a single known PC -> LINK_FALLTHROUGH.
    // A conditional branch at block end converges taken+not-taken onto this one
    // exit (two possible PCs) and a dynamic branch has no compile-time target;
    // both keep the plain dispatcher exit.
    bool linked = false;
    if (HasStaticExit && !StaticExitCond)
    {
    #ifdef LITEV_LINK_UNCOND
        EmitLinkExit(StaticExitTarget);
        linked = true;
    #endif
    }
    else if (!HasStaticExit && LastInstrCompiledNonBranch)
    {
    #ifdef LITEV_LINK_FALLTHROUGH
        EmitLinkExit(LastInstrFallthroughAddr);
        linked = true;
    #endif
    }
    if (!linked)
    {
        EmitBlockExit();
    }
  #else
    EmitBlockExit();
  #endif
#else
    QuickTailCall(X0, ARM_Ret);
#endif

    FlushIcache();


#ifdef LITEV_JIT_PERFMAP
    {
        char nm[24];
        snprintf(nm, sizeof(nm), "jit_a%d_%x", Num == 0 ? 9 : 7, (unsigned)instrs[0].Addr);
        litev_perfmap::Add(nm, (void*)res, GetRXPtr());
    }
#endif

    return res;
}

#ifdef LITEV_JIT_ICACHE
void Compiler::ICacheAllocOnce()
{
    for (int c = 0; c < 2; c++)
        if (!ICacheTable[c])
            ICacheTable[c] = (ICacheEntry*)calloc(ICacheSites, sizeof(ICacheEntry));
}

void Compiler::ICacheReset()
{
    // Wholesale block-cache reset: every host block is gone, so every cached
    // (guest-PC -> host-ptr) pair is stale. Zeroing the tables makes key0/key1 == 0
    // (an impossible even instrAddr only if 0 is never a real block start; the epoch
    // guard is the real safety net regardless). Reset the site counter and bump the
    // per-CPU epoch so any in-flight entry (there are none post-reset, but be robust)
    // is rejected. NextSite starts at 1 (0 reserved for "no cache").
    for (int c = 0; c < 2; c++)
    {
        if (ICacheTable[c]) memset(ICacheTable[c], 0, (size_t)ICacheSites * sizeof(ICacheEntry));
    }
    ICacheNextSite = 1;
    NDS.ARM9.ICacheEpoch++;
    NDS.ARM7.ICacheEpoch++;
}
#endif

void Compiler::Reset()
{
    LoadStorePatches.clear();

#ifdef LITEV_JIT_ICACHE
    // Allocate the per-CPU inline-cache tables BEFORE Gen_Dispatcher bakes their base
    // address, then clear them + bump the epoch for this new cache epoch.
    ICacheAllocOnce();
    ICacheReset();
#endif

#ifdef LITEV_JIT_PERFMAP
    // Wholesale block-cache reset: truncate the perf map and rewrite the
    // process-lifetime stub regions before the dispatchers/blocks re-populate it.
    litev_perfmap::BeginEpoch();
#endif

    SetCodePtr(0);
    OtherCodeRegion = JitMemMainSize;

    const u32 brk_0 = 0xD4200000;

    for (int i = 0; i < (JitMemMainSize + JitMemSecondarySize) / 4; i++)
        *(((u32*)GetRWPtr()) + i) = brk_0;

#ifdef LITEV_JIT_DISPATCH
    // Generate the per-CPU dispatchers at the very start of the (now-brk-filled)
    // block-cache region. Doing it here — not in the constructor — means GetRXBase()
    // is already the stable, rebased block-cache base (== the base SubEntryOffset uses),
    // so the RXBase baked into the hit path is always correct. Blocks compiled after
    // this Reset() start just past the dispatcher code. Regenerated on every
    // ResetBlockCache(), which is cheap and keeps the base bit-for-bit consistent.
    SetCodePtr(0);
    DispatcherEntry[0] = Gen_Dispatcher(0);
#ifdef LITEV_JIT_PERFMAP
    litev_perfmap::Add("jit_dispatcher9", DispatcherEntry[0], GetRXPtr());
#endif
    DispatcherEntry[1] = Gen_Dispatcher(1);
#ifdef LITEV_JIT_PERFMAP
    litev_perfmap::Add("jit_dispatcher7", DispatcherEntry[1], GetRXPtr());
#endif
    FlushIcache();
#endif
}

void Compiler::Comp_AddCycles_C(bool forceNonConstant)
{
    s32 cycles = Num ?
        NDS.ARM7MemTimings[CurInstr.CodeCycles][Thumb ? 1 : 3]
        : ((R15 & 0x2) ? 0 : CurInstr.CodeCycles);

    if (forceNonConstant)
        ConstantCycles += cycles;
#ifdef LITEV_JIT_CYCLE_BATCH
    else if (DeferCycles)
        PendingCycles += cycles;
#endif
    else
        ADD(RCycles, RCycles, cycles);
}

void Compiler::Comp_AddCycles_CI(u32 numI)
{
    IrregularCycles = true;

    s32 cycles = (Num ?
        NDS.ARM7MemTimings[CurInstr.CodeCycles][Thumb ? 0 : 2]
        : ((R15 & 0x2) ? 0 : CurInstr.CodeCycles)) + numI;

    if (Thumb || CurInstr.Cond() == 0xE)
        ConstantCycles += cycles;
    else
        ADD(RCycles, RCycles, cycles);
}

void Compiler::Comp_AddCycles_CI(u32 c, ARM64Reg numI, ArithOption shift)
{
    IrregularCycles = true;

    s32 cycles = (Num ?
        NDS.ARM7MemTimings[CurInstr.CodeCycles][Thumb ? 0 : 2]
        : ((R15 & 0x2) ? 0 : CurInstr.CodeCycles)) + c;

#ifdef LITEV_JIT_CYCLE_BATCH
    if (DeferCycles)
        PendingCycles += cycles;
    else
#endif
    ADD(RCycles, RCycles, cycles);
    if (Thumb || CurInstr.Cond() >= 0xE)
        ConstantCycles += cycles;
    else
        ADD(RCycles, RCycles, cycles);
}

void Compiler::Comp_AddCycles_CDI()
{
    if (Num == 0)
        Comp_AddCycles_CD();
    else
    {
        IrregularCycles = true;

        s32 cycles;

        s32 numC = NDS.ARM7MemTimings[CurInstr.CodeCycles][Thumb ? 0 : 2];
        s32 numD = CurInstr.DataCycles;

        if ((CurInstr.DataRegion >> 24) == 0x02) // mainRAM
        {
            if (CodeRegion == 0x02)
                cycles = numC + numD;
            else
            {
                numC++;
                cycles = std::max(numC + numD - 3, std::max(numC, numD));
            }
        }
        else if (CodeRegion == 0x02)
        {
            numD++;
            cycles = std::max(numC + numD - 3, std::max(numC, numD));
        }
        else
        {
            cycles = numC + numD + 1;
        }
        
        if (!Thumb && CurInstr.Cond() < 0xE)
            ADD(RCycles, RCycles, cycles);
        else
            ConstantCycles += cycles;
    }
}

void Compiler::Comp_AddCycles_CD()
{
    u32 cycles = 0;
    if (Num == 0)
    {
        s32 numC = (R15 & 0x2) ? 0 : CurInstr.CodeCycles;
        s32 numD = CurInstr.DataCycles;

        //if (DataRegion != CodeRegion)
            cycles = std::max(numC + numD - 6, std::max(numC, numD));

        IrregularCycles = cycles != numC;
    }
    else
    {
        s32 numC = NDS.ARM7MemTimings[CurInstr.CodeCycles][Thumb ? 0 : 2];
        s32 numD = CurInstr.DataCycles;

        if ((CurInstr.DataRegion >> 24) == 0x02)
        {
            if (CodeRegion == 0x02)
                cycles += numC + numD;
            else
                cycles += std::max(numC + numD - 3, std::max(numC, numD));
        }
        else if (CodeRegion == 0x02)
        {
            cycles += std::max(numC + numD - 3, std::max(numC, numD));
        }
        else
        {
            cycles += numC + numD;
        }

        IrregularCycles = true;
    }

    if ((!Thumb && CurInstr.Cond() < 0xE) && IrregularCycles)
        ADD(RCycles, RCycles, cycles);
    else
        ConstantCycles += cycles;
}

}
