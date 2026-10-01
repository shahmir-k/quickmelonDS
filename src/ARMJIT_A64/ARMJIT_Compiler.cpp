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
// Unit 3 dispatcher lookup fields (see ARMJIT_Offsets.h). Proven here so a layout
// shift fails the build rather than corrupting the emitted inline block lookup.
static_assert(offsetof(ARM, FastBlockLookupStart) == ARM_FastBlockLookupStart_offset,
    "ARM_FastBlockLookupStart_offset out of sync with ARM::FastBlockLookupStart");
static_assert(offsetof(ARM, FastBlockLookupSize) == ARM_FastBlockLookupSize_offset,
    "ARM_FastBlockLookupSize_offset out of sync with ARM::FastBlockLookupSize");
static_assert(offsetof(ARM, FastBlockLookup) == ARM_FastBlockLookup_offset,
    "ARM_FastBlockLookup_offset out of sync with ARM::FastBlockLookup");
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
    W8, W9, W10, W11, W12, W13, W14, W15
};
template <>
const int RegisterCache<Compiler, ARM64Reg>::NativeRegsAvailable = 15;

const BitSet32 CallerSavedPushRegs({W8, W9, W10, W11, W12, W13, W14, W15});

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
        ANDI2R(W5, RCPSR, 0x1F);
        MOVI2R(W3, 0);
        MOVI2R(W1, 15 - 8);
        BL(ReadBanked);
        MOV(rd, W3);
    }
    else
        MOV(rd, RCPSR);
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
        ANDI2R(W5, RCPSR, 0x1F);
        MOVI2R(W3, 0);
        MOVI2R(W1, 15 - 8);
        BL(ReadBanked);

        MOVI2R(W1, mask);
        MOVI2R(W2, mask & 0xFFFFFF00);
        ANDI2R(W5, RCPSR, 0x1F);
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
        JumpToFuncs7[i] = Gen_JumpTo7(i);
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

    for (int consoleType = 0; consoleType < 2; consoleType++)
    {
        for (int num = 0; num < 2; num++)
        {
            for (int size = 0; size < 3; size++)
            {
                for (int reg = 0; reg < 32; reg++)
                {
                    if (!(reg == W4 || (reg >= W8 && reg <= W15) || (reg >= W19 && reg <= W25)))
                        continue;
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
    LDR(INDEX_UNSIGNED, RCPSR, RCPU, offsetof(ARM, CPSR));
}

void Compiler::SaveCPSR(bool markClean)
{
#ifdef LITEV_JIT_FIXEDREG
    // Reconcile any host-NZCV-resident guest flags into the canonical CPSR store before a
    // C escape / block boundary (mode switch, exception, interpreter fallback). Under
    // LAZYFLAGS this flushes host NZCV into the JitNZCV slot.
    Comp_MaterializeFlags();
#endif
    if (CPSRDirty)
    {
        STR(INDEX_UNSIGNED, RCPSR, RCPU, offsetof(ARM, CPSR));
        CPSRDirty = CPSRDirty && !markClean;
    }
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

    // (d) instrAddr = R[15] - ((CPSR&0x20)?2:4) == (R[15] - 4) + (thumb << 1), as in the loop
    LDR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, R[15]));
    UBFX(W3, RCPSR, 5, 1);                            // W3 = Thumb bit
    SUB(W0, W0, 4);
    ADD(W0, W0, W3, ArithOption(W3, ST_LSL, 1));      // W0 = clean instrAddr


    // (e) region bounds: offset = instrAddr - FastBlockLookupStart, exit if >= Size (unsigned;
    //     a single compare also catches instrAddr < Start via wraparound)
    LDR(INDEX_UNSIGNED, W3, RCPU, offsetof(ARM, FastBlockLookupStart));
    SUB(W4, W0, W3);
    LDR(INDEX_UNSIGNED, W5, RCPU, offsetof(ARM, FastBlockLookupSize));
    CMP(W4, W5);
    FixupBranch exitRegion = B(CC_HS);

    // (f) inline tag lookup: entry = FastBlockLookup[offset/2]; tag = entry>>32 == (instrAddr|num)
    LDR(INDEX_UNSIGNED, X6, RCPU, offsetof(ARM, FastBlockLookup));
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
    STR(INDEX_UNSIGNED, RCPSR, RCPU, offsetof(ARM, CPSR));
    // LAZYFLAGS: ARM::CPSR is already canonical in memory at every block exit (the exiting
    // block materialized host NZCV and control bits are never register-cached), so the
    // per-hop commit is unnecessary; W27 now holds guest r7 and must not be stored here.
    // low 32 bits of the entry (W7) are the sub-entry offset into the RX region; baked RXBase
    // == GetRXBase() (stable rebased base, the same one SubEntryOffset subtracts).
    MOVI2R(X3, (u64)GetRXBase());
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


    SetJumpTarget(exitStop);
    SetJumpTarget(exitBudget);
    SetJumpTarget(exitRegion);
    SetJumpTarget(miss);
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
    LITE_PROFILE_ADD(melonDS::LiteProfile::g_Frame.DispatchOnlyExits);
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
    void* tsPtr = (Num == 0) ? (void*)&NDS.ARM9Timestamp : (void*)&NDS.ARM7Timestamp;

    // (a)
    LDR(INDEX_UNSIGNED, W2, RCPU, offsetof(ARM, StopExecution));
    FixupBranch toDispatchStop = CBNZ(W2);

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

    // (d)
    STR(INDEX_UNSIGNED, RCPSR, RCPU, offsetof(ARM, CPSR));
    // LAZYFLAGS: ARM::CPSR already canonical in memory at the linkable exit; W27 = guest r7.
    u32 patchOffset = (u32)((u8*)GetRXPtr() - GetRXBase());
    B(DispatcherEntry[Num]);   // the patch slot (unlinked state)

    SetJumpTarget(toDispatchStop);
    SetJumpTarget(toDispatchBudget);
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
        STR(INDEX_UNSIGNED, WZR, RCPU, offsetof(ARM, CyclesBudget));
#endif
    }

    if ((CurInstr.BranchFlags & branch_FollowCondNotTaken && taken)
        || (CurInstr.BranchFlags & branch_FollowCondTaken && !taken))
    {
        RegCache.PrepareExit();

        if (ConstantCycles)
            ADD(RCycles, RCycles, ConstantCycles);
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
    RegCache = RegisterCache<Compiler, ARM64Reg>(this, instrs, instrsCount, true);
    CPSRDirty = false;
#ifdef LITEV_JIT_FIXEDREG
    NZCVDeferred = 0;
    NZCVCondValid = false;
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
        }

#ifdef LITEV_JIT_LINK
        // Only a JIT-compiled non-branch has a statically-known single next PC (an
        // interpreter fallback might mutate R15). Captured for the block-end tail.
        LastInstrCompiledNonBranch = (comp != NULL) && !CurInstr.Info.Branches();
        LastInstrFallthroughAddr = CurInstr.Addr + (Thumb ? 2 : 4);
#endif
    }

    RegCache.Flush();
#ifdef LITEV_JIT_FIXEDREG
    // Block ends with the last instruction's flags possibly still resident in host
    // NZCV (e.g. a trailing unconditional CMP). The dispatcher / ARM_Ret stores the
    // live RCPSR at runtime, so reconcile now.
    Comp_MaterializeFlags();
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

    return res;
}

void Compiler::Reset()
{
    LoadStorePatches.clear();

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
    DispatcherEntry[1] = Gen_Dispatcher(1);
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
