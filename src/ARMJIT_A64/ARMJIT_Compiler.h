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

#ifndef ARMJIT_A64_COMPILER_H
#define ARMJIT_A64_COMPILER_H

#if defined(JIT_ENABLED) && defined(__aarch64__)

#include "../ARM.h"

#include "../dolphin/Arm64Emitter.h"

#include "../ARMJIT_Internal.h"
#include "../ARMJIT_RegisterCache.h"

#include <unordered_map>

namespace melonDS
{
class ARMJIT;
const Arm64Gen::ARM64Reg RMemBase = Arm64Gen::X26;
const Arm64Gen::ARM64Reg RCPSR = Arm64Gen::W27;
const Arm64Gen::ARM64Reg RCycles = Arm64Gen::W28;
const Arm64Gen::ARM64Reg RCPU = Arm64Gen::X29;

struct Op2
{
    Op2()
    {}

    Op2(Arm64Gen::ARM64Reg rm) : IsImm(false)
    {
        Reg.Rm = rm;
        Reg.ShiftType = Arm64Gen::ST_LSL;
        Reg.ShiftAmount = 0;
    }

    Op2(u32 imm) : IsImm(true), Imm(imm)
    {}

    Op2(Arm64Gen::ARM64Reg rm, Arm64Gen::ShiftType st, int amount) : IsImm(false)
    {
        Reg.Rm = rm;
        Reg.ShiftType = st;
        Reg.ShiftAmount = amount;
    }

    Arm64Gen::ArithOption ToArithOption()
    {
        assert(!IsImm);
        return Arm64Gen::ArithOption(Reg.Rm, Reg.ShiftType, Reg.ShiftAmount);
    }

    bool IsSimpleReg()
    { return !IsImm && !Reg.ShiftAmount && Reg.ShiftType == Arm64Gen::ST_LSL; }
    bool ImmFits12Bit()
    { return IsImm && ((Imm & 0xFFF) == Imm); }
    bool IsZero()
    { return IsImm && !Imm; }

    bool IsImm;
    union
    {
        struct
        {
            Arm64Gen::ARM64Reg Rm;
            Arm64Gen::ShiftType ShiftType;
            int ShiftAmount;
        } Reg;
        u32 Imm;
    };
};

struct LoadStorePatch
{
    void* PatchFunc;
    s32 PatchOffset;
    u32 PatchSize;
};

class Compiler : public Arm64Gen::ARM64XEmitter
{
public:
    typedef void (Compiler::*CompileFunc)();

    explicit Compiler(melonDS::NDS& nds);
    ~Compiler() override;

    void PushRegs(bool saveHiRegs, bool saveRegsToBeChanged, bool allowUnload = true);
    void PopRegs(bool saveHiRegs, bool saveRegsToBeChanged);

    Arm64Gen::ARM64Reg MapReg(int reg)
    {
        assert(RegCache.Mapping[reg] != Arm64Gen::INVALID_REG);
        return RegCache.Mapping[reg];
    }

    JitBlockEntry CompileBlock(ARM* cpu, bool thumb, FetchedInstr instrs[], int instrsCount, bool hasMemInstr);

    bool CanCompile(bool thumb, u16 kind);

    bool FlagsNZNeeded() const
    {
        return CurInstr.SetFlags & 0xC;
    }

    void Reset();

    void Comp_AddCycles_C(bool forceNonConstant = false);
    void Comp_AddCycles_CI(u32 numI);
    void Comp_AddCycles_CI(u32 c, Arm64Gen::ARM64Reg numI, Arm64Gen::ArithOption shift);
    void Comp_AddCycles_CD();
    void Comp_AddCycles_CDI();

    void MovePC();

    void LoadReg(int reg, Arm64Gen::ARM64Reg nativeReg);
    void SaveReg(int reg, Arm64Gen::ARM64Reg nativeReg);

    void LoadCPSR();
    void SaveCPSR(bool markClean = true);

    void LoadCycles();
    void SaveCycles();

    void Nop() {}

    void A_Comp_ALUTriOp();
    void A_Comp_ALUMovOp();
    void A_Comp_ALUCmpOp();

    void A_Comp_Mul();
    void A_Comp_Mul_Long();
    void A_Comp_Mul_Short();

    void A_Comp_Clz();

    void A_Comp_MemWB();
    void A_Comp_MemHD();

    void A_Comp_LDM_STM();
    
    void A_Comp_BranchImm();
    void A_Comp_BranchXchangeReg();

    void A_Comp_MRS();
    void A_Comp_MSR();

    void T_Comp_ShiftImm();
    void T_Comp_AddSub_();
    void T_Comp_ALUImm8();
    void T_Comp_ALU();
    void T_Comp_ALU_HiReg();
    void T_Comp_AddSP();
    void T_Comp_RelAddr();

    void T_Comp_MemReg();
    void T_Comp_MemImm();
    void T_Comp_MemRegHalf();
    void T_Comp_MemImmHalf();
    void T_Comp_LoadPCRel();
    void T_Comp_MemSPRel();

    void T_Comp_LDMIA_STMIA();
    void T_Comp_PUSH_POP();

    void T_Comp_BCOND();
    void T_Comp_B();
    void T_Comp_BranchXchangeReg();
    void T_Comp_BL_LONG_1();
    void T_Comp_BL_LONG_2();
    void T_Comp_BL_Merged();

    s32 Comp_MemAccessBlock(int rn, BitSet16 regs, bool store, bool preinc, bool decrement, bool usermode, bool skipLoadingRn);

    void Comp_Mul_Mla(bool S, bool mla, Arm64Gen::ARM64Reg rd, Arm64Gen::ARM64Reg rm, Arm64Gen::ARM64Reg rs, Arm64Gen::ARM64Reg rn);

    void Comp_Compare(int op, Arm64Gen::ARM64Reg rn, Op2 op2);
    void Comp_Logical(int op, bool S, Arm64Gen::ARM64Reg rd, Arm64Gen::ARM64Reg rn, Op2 op2);
    void Comp_Arithmetic(int op, bool S, Arm64Gen::ARM64Reg rd, Arm64Gen::ARM64Reg rn, Op2 op2);

    void Comp_RetriveFlags(bool retriveCV);

#ifdef LITEV_JIT_FIXEDREG
    // liteDS-v2 Stage 1 (DraStic teardown 01 §6.1: guest CPSR flags = host NZCV).
    // An UNCONDITIONAL flag-setting ADD/SUB/RSB/CMP/CMN leaves its guest NZCV in
    // the host PSTATE (the SUBS/ADDS already computed them there) instead of
    // extracting them into the RCPSR word. NZCVDeferred records which flag bits
    // (encoded like CurInstr.SetFlags: N=8,Z=4,C=2,V=1) are currently resident in
    // host NZCV and NOT yet written back to RCPSR. Comp_MaterializeFlags re-runs
    // the exact deferred extraction (byte-identical CSET/BFI sequence to
    // Comp_RetriveFlags) at every consumer / host-NZCV-clobber / block boundary;
    // it does NOT touch host PSTATE, so a CheckCondition can still branch on the
    // resident flags immediately afterwards.
    u8 NZCVDeferred = 0;
    // liteDS-v2 Stage 2a: the deferred set widened beyond full-NZCV arithmetic to
    // LOGICAL producers (AND/EOR/ORR/BIC/TST/TEQ), whose host op leaves ONLY guest
    // N,Z resident in PSTATE (the AArch64 logical op zeroes host C,V; guest C is in
    // RCPSR from the barrel shifter, guest V is preserved in RCPSR). For those the
    // host NZCV is NOT a valid full guest-condition source. NZCVCondValid records
    // whether the CURRENTLY-deferred flags represent the COMPLETE guest NZCV in host
    // PSTATE (true for arithmetic ADD/SUB/RSB/ADC/SBC/CMP/CMN, false for logical):
    // only then may a consumer evaluate the guest condition natively via B.<cc>.
    // Otherwise the consumer materializes N,Z into RCPSR and uses the RCPSR path.
    bool NZCVCondValid = false;
    void Comp_MaterializeFlags();

    // liteDS-v2 Stage 2b: host NZCV as the SOLE canonical block-wide flag store.
    // Instead of spilling the resident flags into RCPSR before EVERY next instruction
    // body (the conservative Stage-1/2a reconcile), Comp_ReconcileFlags classifies the
    // upcoming body and spills ONLY when it must: when the body READS a currently-
    // deferred guest flag, or CLOBBERS host NZCV without fully re-establishing the
    // guest condition there (partial producers, register-specified-shift scratch CMPs,
    // memory stub BLs, helpers). A full-NZCV arithmetic producer (SUBS/ADDS/CMP/CMN)
    // and a flag-transparent body keep the flags resident with no RCPSR round trip.
    // Comp_BodyIsNZCVTransparent is the hand classifier the Stage-2a handoff required
    // (Info.WriteFlags/ReadFlags cannot express the shift-helper's scratch clobber).
    void Comp_ReconcileFlags();
    bool Comp_BodyIsNZCVTransparent(u16 kind, u8 writeFlags, u8 readFlags);

#endif

    Arm64Gen::FixupBranch CheckCondition(u32 cond);

    void Comp_JumpTo(Arm64Gen::ARM64Reg addr, bool switchThumb, bool restoreCPSR = false);
    void Comp_JumpTo(u32 addr, bool forceNonConstantCycles = false);

    void A_Comp_GetOp2(bool S, Op2& op2);

    void Comp_RegShiftImm(int op, int amount, bool S, Op2& op2, Arm64Gen::ARM64Reg tmp = Arm64Gen::W0);
    void Comp_RegShiftReg(int op, bool S, Op2& op2, Arm64Gen::ARM64Reg rs);

    bool Comp_MemLoadLiteral(int size, bool signExtend, int rd, u32 addr);

    enum
    {
        memop_Writeback = 1 << 0,
        memop_Post = 1 << 1,
        memop_SignExtend = 1 << 2,
        memop_Store = 1 << 3,
        memop_SubtractOffset = 1 << 4
    };
    void Comp_MemAccess(int rd, int rn, Op2 offset, int size, int flags);

    // 0 = switch mode, 1 = stay arm, 2 = stay thumb
    void* Gen_JumpTo9(int kind);
    void* Gen_JumpTo7(int kind);

#ifdef LITEV_JIT_DISPATCH
    // liteDS-v2 Unit 3: emitted per-CPU dispatcher stub (num=0 ARM9, num=1 ARM7).
    // Generated in Reset() so GetRXBase() is the stable rebased block-cache base.
    void* Gen_Dispatcher(u32 num);
    // Common block-exit tail: jump to this CPU's dispatcher instead of ARM_Ret so
    // execution stays in JIT context across block boundaries.
    void EmitBlockExit();
    void* DispatcherEntry[2] = { nullptr, nullptr };


#endif

#ifdef LITEV_JIT_LINK
    // liteDS-v2 Unit 4: emit a per-hop-commit guard followed by a patchable `B`
    // (initially -> dispatcher) for a static, same-mode exit whose next PC is the
    // compile-time constant targetAddr. Records the patch site + target into
    // LinkExits[] for the C++ registry to resolve after the block is registered.
    void EmitLinkExit(u32 targetAddr);
    // Rewrite the 4-byte `B` at RX offset rxOffset to branch to RX offset
    // targetRxOffset. Caller owns the W^X (JitEnableWrite/Execute) bracket.
    void PatchLinkSite(u32 rxOffset, u32 targetRxOffset);
    u32 DispatcherRXOffset(u32 num) { return (u32)((u8*)DispatcherEntry[num] - GetRXBase()); }

    // Populated during CompileBlock; copied into the JitBlock by ARMJIT::CompileBlock.
    u8 NumLinkExits = 0;
    OutgoingLink LinkExits[2];

    // Threaded from Comp_JumpTo(u32) to the exit tail: does the last-compiled branch
    // have a compile-time-constant same-mode target, and was it conditional?
    bool HasStaticExit = false;
    u32 StaticExitTarget = 0;
    bool StaticExitCond = false;
    // Tracked per instruction so the block-end fall-through exit is only linked when
    // the last instruction was a JIT-compiled non-branch (single known next PC).
    bool LastInstrCompiledNonBranch = false;
    u32 LastInstrFallthroughAddr = 0;
#endif

    void Comp_BranchSpecialBehaviour(bool taken);

    JitBlockEntry AddEntryOffset(u32 offset)
    {
        return (JitBlockEntry)(GetRXBase() + offset);
    }

    u32 SubEntryOffset(JitBlockEntry entry)
    {
        return (u8*)entry - GetRXBase();
    }

    bool IsJITFault(const u8* pc);
    u8* RewriteMemAccess(u8* pc);

    void SwapCodeRegion()
    {
        ptrdiff_t offset = GetCodeOffset();
        SetCodePtrUnsafe(OtherCodeRegion);
        OtherCodeRegion = offset;
    }

    melonDS::NDS& NDS;
    ptrdiff_t OtherCodeRegion;

    bool Exit;

    FetchedInstr CurInstr;
    bool Thumb;
    u32 R15;
    u32 Num;
    ARM* CurCPU;
    u32 ConstantCycles;
    u32 CodeRegion;

    BitSet32 SavedRegs;

    u32 JitMemSecondarySize;
    u32 JitMemMainSize;

    std::unordered_map<ptrdiff_t, LoadStorePatch> LoadStorePatches; 

    RegisterCache<Compiler, Arm64Gen::ARM64Reg> RegCache;

    bool CPSRDirty = false;

    bool IrregularCycles = false;

#ifdef __SWITCH__
    void* JitRWBase;
    void* JitRWStart;
    void* JitRXStart;
#endif
    void* CodeMemBase;

    void* ReadBanked, *WriteBanked;

    void* JumpToFuncs9[3];
    void* JumpToFuncs7[3];

    // [Console Type][Num][Size][Sign Extend][Output register]
    void* PatchedLoadFuncs[2][2][3][2][32];
    void* PatchedStoreFuncs[2][2][3][32];
};

}

#endif

#endif
