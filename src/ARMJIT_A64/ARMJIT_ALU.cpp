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

using namespace Arm64Gen;

namespace melonDS
{

void Compiler::Comp_RegShiftReg(int op, bool S, Op2& op2, ARM64Reg rs)
{
    if (!(CurInstr.SetFlags & 0x2))
        S = false;

    CPSRDirty |= S;

    UBFX(W1, rs, 0, 8);

    if (!S)
    {
        if (op == 3)
            RORV(W0, op2.Reg.Rm, W1);
        else
        {
            CMP(W1, 32);
            if (op == 2)
            {
                MOVI2R(W2, 31);
                CSEL(W1, W2, W1, CC_GE);
                ASRV(W0, op2.Reg.Rm, W1);
            }
            else
            {
                if (op == 0)
                    LSLV(W0, op2.Reg.Rm, W1);
                else if (op == 1)
                    LSRV(W0, op2.Reg.Rm, W1);
                CSEL(W0, WZR, W0, CC_GE);
            }
        }
    }
    else
    {
        MOV(W0, op2.Reg.Rm);
        FixupBranch zero = CBZ(W1);

        SUB(W1, W1, 1);
        if (op == 3)
        {
            RORV(W0, op2.Reg.Rm, W1);
            BFI(RCPSR, W0, 29, 1);
        }
        else
        {
            CMP(W1, 31);
            if (op == 2)
            {
                MOVI2R(W2, 31);
                CSEL(W1, W2, W1, CC_GT);
                ASRV(W0, op2.Reg.Rm, W1);
                BFI(RCPSR, W0, 29, 1);
            }
            else
            {
                if (op == 0)
                {
                    LSLV(W0, op2.Reg.Rm, W1);
                    UBFX(W1, W0, 31, 1);
                }
                else if (op == 1)
                    LSRV(W0, op2.Reg.Rm, W1);
                CSEL(W1, WZR, op ? W0 : W1, CC_GT);
                BFI(RCPSR, W1, 29, 1);
                CSEL(W0, WZR, W0, CC_GE);
            }
        }

        MOV(W0, W0, ArithOption(W0, (ShiftType)op, 1));
        SetJumpTarget(zero);
    }
    op2 = Op2(W0, ST_LSL, 0);
}

void Compiler::Comp_RegShiftImm(int op, int amount, bool S, Op2& op2, ARM64Reg tmp)
{
    if (!(CurInstr.SetFlags & 0x2))
        S = false;

    CPSRDirty |= S;
    
    switch (op)
    {
    case 0: // LSL
        if (S && amount)
        {
            UBFX(tmp, op2.Reg.Rm, 32 - amount, 1);
            BFI(RCPSR, tmp, 29, 1);
        }
        op2 = Op2(op2.Reg.Rm, ST_LSL, amount);
        return;
    case 1: // LSR
        if (S)
        {
            UBFX(tmp, op2.Reg.Rm, (amount ? amount : 32) - 1, 1);
            BFI(RCPSR, tmp, 29, 1);
        }
        if (amount == 0)
        {
            op2 = Op2(0);
            return;
        }
        op2 = Op2(op2.Reg.Rm, ST_LSR, amount);
        return;
    case 2: // ASR
        if (S)
        {
            UBFX(tmp, op2.Reg.Rm, (amount ? amount : 32) - 1, 1);
            BFI(RCPSR, tmp, 29, 1);
        }
        op2 = Op2(op2.Reg.Rm, ST_ASR, amount ? amount : 31);
        return;
    case 3: // ROR
        if (amount == 0)
        {
            UBFX(tmp, RCPSR, 29, 1);
            LSL(tmp, tmp, 31);
            if (S)
                BFI(RCPSR, op2.Reg.Rm, 29, 1);
            ORR(tmp, tmp, op2.Reg.Rm, ArithOption(tmp, ST_LSR, 1));

            op2 = Op2(tmp, ST_LSL, 0);
        }
        else
        {
            if (S)
            {
                UBFX(tmp, op2.Reg.Rm, amount - 1, 1);
                BFI(RCPSR, tmp, 29, 1);
            }
            op2 = Op2(op2.Reg.Rm, ST_ROR, amount);
        }
        return;
    }
}

void Compiler::Comp_RetriveFlags(bool retriveCV)
{
    if (CurInstr.SetFlags)
        CPSRDirty = true;

    if (CurInstr.SetFlags & 0x4)
    {
        CSET(W0, CC_EQ);
        BFI(RCPSR, W0, 30, 1);
    }
    if (CurInstr.SetFlags & 0x8)
    {
        CSET(W0, CC_MI);
        BFI(RCPSR, W0, 31, 1);
    }
    if (retriveCV)
    {
        if (CurInstr.SetFlags & 0x2)
        {
            CSET(W0, CC_CS);
            BFI(RCPSR, W0, 29, 1);
        }
        if (CurInstr.SetFlags & 0x1)
        {
            CSET(W0, CC_VS);
            BFI(RCPSR, W0, 28, 1);
        }
    }
}

#ifdef LITEV_JIT_FIXEDREG
void Compiler::Comp_MaterializeFlags()
{
    // Flush guest flags resident in host NZCV back into the RCPSR word. This is
    // the deferred half of Comp_RetriveFlags(true): the SAME CSET/BFI per live
    // flag bit reading the SAME host NZCV, so the resulting RCPSR is bit-exact to
    // what the producer would have extracted in place. CSET/BFI do not modify host
    // PSTATE, so callers (e.g. CheckCondition) may still branch natively on the
    // resident flags after materializing.
    if (!NZCVDeferred)
        return;

    u8 m = NZCVDeferred;
    NZCVDeferred = 0;
    CPSRDirty = true;

    if (m & 0x4) { CSET(W0, CC_EQ); BFI(RCPSR, W0, 30, 1); } // Z
    if (m & 0x8) { CSET(W0, CC_MI); BFI(RCPSR, W0, 31, 1); } // N
    if (m & 0x2) { CSET(W0, CC_CS); BFI(RCPSR, W0, 29, 1); } // C
    if (m & 0x1) { CSET(W0, CC_VS); BFI(RCPSR, W0, 28, 1); } // V
}


// liteDS-v2 Stage 2b: is the CURRENT instruction body flag-transparent, i.e. does it
// leave host PSTATE NZCV completely untouched AND read no guest CPSR flag? Only such
// bodies may run with the resident flags kept alive in host NZCV. This is the hand
// classifier the Stage-2a handoff flagged as required: Info.WriteFlags/ReadFlags
// alone cannot express the shift-helper's scratch CMP (a non-S `LSL rd,rn,rs` writes
// ZERO guest flags yet Comp_RegShiftReg emits a `CMP` that clobbers host NZCV) --
// exactly the hazard armwrestler/rockwrestler torture.
//
// The check is conservatively FALSE (spill) for anything not proven transparent.
bool Compiler::Comp_BodyIsNZCVTransparent(u16 kind, u8 writeFlags, u8 readFlags)
{
    using namespace ARMInstrInfo;

    // A transparent body neither produces guest flags nor reads them. (A body that
    // sets flags necessarily writes host NZCV; a body that reads flags needs a
    // canonical RCPSR.) This is necessary but not sufficient -- WriteFlags==0 bodies
    // can still scratch-clobber host NZCV, filtered per-kind below.
    if (writeFlags != 0 || readFlags != 0)
        return false;

    if (Thumb)
    {
        // The only WriteFlags==0/ReadFlags==0 Thumb bodies that touch NEITHER host
        // NZCV nor memory (no scratch CMP, no stub BL) are the non-flag hi-reg / SP /
        // PC-relative address forms. Everything else with no flags is a load/store or
        // a branch -> spill.
        switch (kind)
        {
        case tk_ADD_HIREG: case tk_MOV_HIREG:
        case tk_ADD_PCREL: case tk_ADD_SPREL: case tk_ADD_SP:
            return true;
        default:
            return false;
        }
    }

    // ARM. Data-processing ALU ops occupy [ak_AND_REG_LSL_IMM .. ak_MVN_IMM_S], 18
    // kinds per op in the fixed ak_ALU() order: op-relative 0..3 = immediate-shifted
    // register, 4..7 = REGISTER-specified shift, 8 = immediate, 9..17 = the S forms.
    // A register-specified shift amount routes op2 through Comp_RegShiftReg, whose
    // `CMP W1,32/31` clobbers host NZCV even when the instruction is non-S -- so only
    // the immediate / immediate-shift forms (rel 0,1,2,3,8) are transparent. The RRX
    // form (rel 3, ROR #0) reads guest C and is already excluded by readFlags!=0.
    if (kind <= ak_MVN_IMM_S)
    {
        u32 rel = kind % 18;
        return rel == 0 || rel == 1 || rel == 2 || rel == 3 || rel == 8;
    }

    // CLZ writes only a GPR. The non-S long/short multiplies (MUL..SMLAL; WriteFlags
    // is 0 here, so the S-form with its host TST is already excluded) write only GPRs
    // on ARM9, or a GPR via CLS/CLZ on ARM7 -- host NZCV is untouched. The SMLAxy /
    // SMLAWy family (> ak_SMLAL) emits `ADDS` and is deliberately NOT included.
    if (kind == ak_CLZ)
        return true;
    if (kind >= ak_MUL && kind <= ak_SMLAL)
        return true;

    // Memory, SMLAxy family, QADD/QSUB, MSR/MRS, branches, coprocessor, unknown: any
    // of these may clobber host PSTATE (stub BL, scratch ADDS, CPSR write) -> spill.
    return false;
}

// liteDS-v2 Stage 2b reconcile: called before each unconditional / Thumb instruction
// body in place of the Stage-1 unconditional Comp_MaterializeFlags. Spills the flags
// resident in host NZCV to RCPSR ONLY when the upcoming body forces it; otherwise the
// flags stay canonical in host PSTATE across the body (the block-wide-NZCV win).
void Compiler::Comp_ReconcileFlags()
{
    if (!NZCVDeferred)
        return;

    const u8 read = CurInstr.Info.ReadFlags;
    const u8 wf   = CurInstr.Info.WriteFlags;
    const u16 kind = CurInstr.Info.Kind;

    // (A) body reads a guest flag that is currently deferred -> RCPSR must be canonical
    // for that read (e.g. ADC/SBC/RSC read C, RRX reads C, MRS reads all).
    if (read & NZCVDeferred)
    {
        Comp_MaterializeFlags();
        return;
    }

    // (B) full-NZCV arithmetic producer: the host SUBS/ADDS/CMP/CMN sets N,Z,C,V ALWAYS
    // (low nibble of WriteFlags == 0xF, only arithmetic compares/add/sub reach this),
    // wholesale overwriting host NZCV and re-establishing the guest condition. The
    // resident flags -- which (A) proved this body does not read -- are thereby dead
    // (a real consumer would have read them via (A) first). Let the producer run; it
    // re-defers / re-extracts a complete, self-consistent flag state. No spill.
    if ((wf & 0x0F) == 0x0F)
        return;

    // (C)/(D): keep resident iff the body is flag-transparent, else spill before it
    // clobbers host NZCV (partial producers, register-shift scratch CMP, stubs, ...).
    if (Comp_BodyIsNZCVTransparent(kind, wf, read))
        return;

    // V3 item 1(a) NOTE — "skip a materialize flush whose deferred bits are all DEAD":
    // this is already achieved UPSTREAM, not here. Every producer defers only LIVE bits
    // (Comp_Logical/Arithmetic/Compare set `NZCVDeferred = CurInstr.SetFlags & mask`, and
    // SetFlags is FloodFillSetFlags' exact backward liveness). So within a block
    // NZCVDeferred can never contain a dead flag, and a "discard the flush if all deferred
    // bits are dead" test here is provably unreachable (verified: a gated discard counted
    // 0 on shrek-race/armwrestler/rockwrestler). The dead-flag elimination the prompt asks
    // for is therefore complete in V2; nothing to add at the flush site.
    Comp_MaterializeFlags();
}
#endif

void Compiler::Comp_Logical(int op, bool S, ARM64Reg rd, ARM64Reg rn, Op2 op2)
{
    if (S && !CurInstr.SetFlags)
        S = false;

    switch (op)
    {
    case 0x0: // AND
        if (S)
        {
            if (op2.IsImm)
                ANDSI2R(rd, rn, op2.Imm, W0);
            else
                ANDS(rd, rn, op2.Reg.Rm, op2.ToArithOption());
        }
        else
        {
            if (op2.IsImm)
                ANDI2R(rd, rn, op2.Imm, W0);
            else
                AND(rd, rn, op2.Reg.Rm, op2.ToArithOption());
        }
        break;
    case 0x1: // EOR
        if (op2.IsImm)
            EORI2R(rd, rn, op2.Imm, W0);
        else
            EOR(rd, rn, op2.Reg.Rm, op2.ToArithOption());
        if (S && FlagsNZNeeded())
            TST(rd, rd);
        break;
    case 0xC: // ORR
        if (op2.IsImm)
            ORRI2R(rd, rn, op2.Imm, W0);
        else
            ORR(rd, rn, op2.Reg.Rm, op2.ToArithOption());
        if (S && FlagsNZNeeded())
            TST(rd, rd);
        break;
    case 0xE: // BIC
        if (S)
        {
            if (op2.IsImm)
                ANDSI2R(rd, rn, ~op2.Imm, W0);
            else
                BICS(rd, rn, op2.Reg.Rm, op2.ToArithOption());
        }
        else
        {
            if (op2.IsImm)
                ANDI2R(rd, rn, ~op2.Imm, W0);
            else
                BIC(rd, rn, op2.Reg.Rm, op2.ToArithOption());
        }
        break;
    }

    if (S)
    {
#ifdef LITEV_JIT_FIXEDREG
        // liteDS-v2 Stage 2a: logical S-op. The AArch64 logical op (ANDS/BICS, or
        // TST after EOR/ORR) leaves guest N,Z LIVE in host PSTATE but ZEROES host
        // C,V. Guest C is already in RCPSR (barrel-shifter carry, written by
        // A_Comp_GetOp2 / Comp_RegShift*) and guest V is preserved in RCPSR, so
        // only N,Z are host-resident. For an UNCONDITIONAL op (single path) defer
        // exactly those bits (SetFlags & 0xC == what Comp_RetriveFlags(false) would
        // extract). Host C,V are NOT valid guest flags -> NZCVCondValid = false, so
        // a consumer materializes N,Z and evaluates its condition from RCPSR. Only
        // take this when N,Z are actually live (mask != 0); otherwise the baseline
        // Comp_RetriveFlags(false) still runs (it sets CPSRDirty for a C-only op).
        if ((Thumb || CurInstr.Cond() == 0xE) && (CurInstr.SetFlags & 0xC))
        {
            NZCVDeferred = CurInstr.SetFlags & 0xC;
            NZCVCondValid = false;
        }
        else
#endif
            Comp_RetriveFlags(false);
    }
}

void Compiler::Comp_Arithmetic(int op, bool S, ARM64Reg rd, ARM64Reg rn, Op2 op2)
{
    if (!op2.IsImm && op2.Reg.ShiftType == ST_ROR)
    {
        MOV(W0, op2.Reg.Rm, op2.ToArithOption());
        op2 = Op2(W0, ST_LSL, 0);
    }

    if (S && !CurInstr.SetFlags)
        S = false;

    bool CVInGPR = false;
    switch (op)
    {
    case 0x2: // SUB
        if (S)
        {
            if (op2.IsImm)
                SUBSI2R(rd, rn, op2.Imm, W0);
            else
                SUBS(rd, rn, op2.Reg.Rm, op2.ToArithOption());
        }
        else
        {
            if (op2.IsImm)
            {
                MOVI2R(W2, op2.Imm);
                SUBI2R(rd, rn, op2.Imm, W0);
            }
            else
                SUB(rd, rn, op2.Reg.Rm, op2.ToArithOption());
        }
        break;
    case 0x3: // RSB
        if (op2.IsZero())
        {
            op2 = Op2(WZR);
        }
        else if (op2.IsImm)
        {
            MOVI2R(W1, op2.Imm);
            op2 = Op2(W1);
        }
        else if (op2.Reg.ShiftAmount != 0)
        {
            MOV(W1, op2.Reg.Rm, op2.ToArithOption());
            op2 = Op2(W1);
        }

        if (S)
            SUBS(rd, op2.Reg.Rm, rn);
        else
            SUB(rd, op2.Reg.Rm, rn);
        break;
    case 0x4: // ADD
        if (S)
        {
            if (op2.IsImm)
                ADDSI2R(rd, rn, op2.Imm, W0);
            else
                ADDS(rd, rn, op2.Reg.Rm, op2.ToArithOption());
        }
        else
        {
            if (op2.IsImm)
                ADDI2R(rd, rn, op2.Imm, W0);
            else
                ADD(rd, rn, op2.Reg.Rm, op2.ToArithOption());
        }
        break;
    case 0x5: // ADC
        UBFX(W2, RCPSR, 29, 1);
        if (S)
        {
            if (op2.IsImm)
            {
                CVInGPR = true;
                ADDS(W1, rn, W2);
                CSET(W2, CC_CS);
                CSET(W3, CC_VS);
                if (op2.IsImm)
                    ADDSI2R(rd, W1, op2.Imm, W0);
                else
                    ADDS(rd, W1, op2.Reg.Rm, op2.ToArithOption());
                CSINC(W2, W2, WZR, CC_CC);
                CSINC(W3, W3, WZR, CC_VC);
            }
            else
            {
                if (op2.Reg.ShiftAmount > 0)
                {
                    MOV(W0, op2.Reg.Rm, op2.ToArithOption());
                    op2 = Op2(W0, ST_LSL, 0);
                }
                CMP(W2, 1);
                ADCS(rd, rn, op2.Reg.Rm);
            }
        }
        else
        {
            ADD(W1, rn, W2);
            if (op2.IsImm)
                ADDI2R(rd, W1, op2.Imm, W0);
            else
                ADD(rd, W1, op2.Reg.Rm, op2.ToArithOption());
        }
        break;
    case 0x6: // SBC
        UBFX(W2, RCPSR, 29, 1);
        if (S && !op2.IsImm)
        {
            if (op2.Reg.ShiftAmount > 0)
            {
                MOV(W0, op2.Reg.Rm, op2.ToArithOption());
                op2 = Op2(W0, ST_LSL, 0);
            }
            CMP(W2, 1);
            SBCS(rd, rn, op2.Reg.Rm);
        }
        else
        {
            // W1 = -op2 - 1
            if (op2.IsImm)
                MOVI2R(W1, ~op2.Imm);
            else
                ORN(W1, WZR, op2.Reg.Rm, op2.ToArithOption());
            if (S)
            {
                CVInGPR = true;
                ADDS(W1, W2, W1);
                CSET(W2, CC_CS);
                CSET(W3, CC_VS);
                ADDS(rd, rn, W1);
                CSINC(W2, W2, WZR, CC_CC);
                CSINC(W3, W3, WZR, CC_VC);
            }
            else
            {
                ADD(W1, W2, W1);
                ADD(rd, rn, W1);
            }
        }
        break;
    case 0x7: // RSC
        UBFX(W2, RCPSR, 29, 1);
        // W1 = -rn - 1
        MVN(W1, rn);
        if (S)
        {
            CVInGPR = true;
            ADDS(W1, W2, W1);
            CSET(W2, CC_CS);
            CSET(W3, CC_VS);
            if (op2.IsImm)
                ADDSI2R(rd, W1, op2.Imm);
            else
                ADDS(rd, W1, op2.Reg.Rm, op2.ToArithOption());
            CSINC(W2, W2, WZR, CC_CC);
            CSINC(W3, W3, WZR, CC_VC);
        }
        else
        {
            ADD(W1, W2, W1);
            if (op2.IsImm)
                ADDI2R(rd, W1, op2.Imm);
            else
                ADD(rd, W1, op2.Reg.Rm, op2.ToArithOption());
        }
        break;
    }

    if (S)
    {
        if (CVInGPR)
        {
            BFI(RCPSR, W2, 29, 1);
            BFI(RCPSR, W3, 28, 1);
        }
        else
        {
#ifdef LITEV_JIT_FIXEDREG
            // Reaching this (non-CVInGPR) branch MEANS the host op left the full
            // guest NZCV in host PSTATE: SUB/RSB/ADD via SUBS/ADDS, and the
            // register-operand ADC/SBC via `CMP Wc,1; ADCS/SBCS` (which seed host
            // carry-in from guest C, so host N,Z,C,V == full guest NZCV). Stage 2a
            // widens the deferral from {SUB,RSB,ADD} to that whole class
            // (op 0x2..0x6; RSC/0x7 always takes the CVInGPR path above, never here).
            // For an UNCONDITIONAL instruction (always executes, so host NZCV is
            // unambiguously current on the single path) keep the flags resident and
            // skip the RCPSR extraction. Conditional producers must stay on the
            // extract path: the host op runs only on the taken side, so a compile-
            // time "resident" claim would be wrong on the skipped side.
            if (op >= 0x2 && op <= 0x6 && (Thumb || CurInstr.Cond() == 0xE))
            {
                NZCVDeferred = CurInstr.SetFlags & 0xF;
                NZCVCondValid = true; // full guest NZCV lives in host PSTATE
            }
            else
#endif
                Comp_RetriveFlags(true);
        }
    }
}

void Compiler::Comp_Compare(int op, ARM64Reg rn, Op2 op2)
{
    if (!op2.IsImm && op2.Reg.ShiftType == ST_ROR)
    {
        MOV(W0, op2.Reg.Rm, op2.ToArithOption());
        op2 = Op2(W0, ST_LSL, 0);
    }

    switch (op)
    {
    case 0x8: // TST
        if (op2.IsImm)
            TSTI2R(rn, op2.Imm, W0);
        else
            ANDS(WZR, rn, op2.Reg.Rm, op2.ToArithOption());
        break;
    case 0x9: // TEQ
        if (op2.IsImm)
            EORI2R(W0, rn, op2.Imm, W0);
        else
            EOR(W0, rn, op2.Reg.Rm, op2.ToArithOption());
        TST(W0, W0);
        break;
    case 0xA: // CMP
        if (op2.IsImm)
            CMPI2R(rn, op2.Imm, W0);
        else
            CMP(rn, op2.Reg.Rm, op2.ToArithOption());
        break;
    case 0xB: // CMN
        if (op2.IsImm)
            ADDSI2R(WZR, rn, op2.Imm, W0);
        else
            CMN(rn, op2.Reg.Rm, op2.ToArithOption());
        break;
    }

#ifdef LITEV_JIT_FIXEDREG
    // CMP/CMN (arithmetic compares) set host NZCV == full guest NZCV. Defer when
    // unconditional; TST/TEQ (logical, op 8/9) fall through to Comp_Compare's tail
    // below which keeps the logical (N,Z-only) extraction.
    if ((op == 0xA || op == 0xB) && (Thumb || CurInstr.Cond() == 0xE))
    {
        NZCVDeferred = CurInstr.SetFlags & 0xF;
        NZCVCondValid = true; // full guest NZCV lives in host PSTATE
        return;
    }
    // TST/TEQ (op 8/9): logical compares — host N,Z live, host C,V zeroed, guest C
    // in RCPSR (shifter), guest V preserved. Defer N,Z only (same as Comp_Logical).
    if ((Thumb || CurInstr.Cond() == 0xE) && (CurInstr.SetFlags & 0xC))
    {
        NZCVDeferred = CurInstr.SetFlags & 0xC;
        NZCVCondValid = false;
        return;
    }
#endif
    Comp_RetriveFlags(op >= 0xA);
}

// also counts cycles!
void Compiler::A_Comp_GetOp2(bool S, Op2& op2)
{
    if (CurInstr.Instr & (1 << 25))
    {
        Comp_AddCycles_C();

        u32 shift = (CurInstr.Instr >> 7) & 0x1E;
        u32 imm = melonDS::ROR(CurInstr.Instr & 0xFF, shift);

        if (S && shift && (CurInstr.SetFlags & 0x2))
        {
            CPSRDirty = true;
            if (imm & 0x80000000)
                ORRI2R(RCPSR, RCPSR, 1 << 29);
            else
                ANDI2R(RCPSR, RCPSR, ~(1 << 29));
        }

        op2 = Op2(imm);    
    }
    else
    {
        int op = (CurInstr.Instr >> 5) & 0x3;
        op2.Reg.Rm = MapReg(CurInstr.A_Reg(0));
        if (CurInstr.Instr & (1 << 4))
        {
            Comp_AddCycles_CI(1);

            ARM64Reg rs = MapReg(CurInstr.A_Reg(8));
            if (CurInstr.A_Reg(0) == 15)
            {
                ADD(W0, op2.Reg.Rm, 4);
                op2.Reg.Rm = W0;
            }
            Comp_RegShiftReg(op, S, op2, rs);
        }
        else
        {
            Comp_AddCycles_C();

            int amount = (CurInstr.Instr >> 7) & 0x1F;
            Comp_RegShiftImm(op, amount, S, op2);
        }
    }
}

void Compiler::A_Comp_ALUCmpOp()
{
    u32 op = (CurInstr.Instr >> 21) & 0xF;
    ARM64Reg rn = MapReg(CurInstr.A_Reg(16));
    Op2 op2;
    A_Comp_GetOp2(op <= 0x9, op2);
    
    Comp_Compare(op, rn, op2);
}

void Compiler::A_Comp_ALUMovOp()
{
    bool S = CurInstr.Instr & (1 << 20);
    u32 op = (CurInstr.Instr >> 21) & 0xF;

    ARM64Reg rd = MapReg(CurInstr.A_Reg(12));
    Op2 op2;
    A_Comp_GetOp2(S, op2);

    if (op == 0xF) // MVN
    {
        if (op2.IsImm)
        {
            if (CurInstr.Cond() == 0xE)
                RegCache.PutLiteral(CurInstr.A_Reg(12), ~op2.Imm);
            MOVI2R(rd, ~op2.Imm);
        }
        else
            ORN(rd, WZR, op2.Reg.Rm, op2.ToArithOption());
    }
    else // MOV
    {
        if (op2.IsImm)
        {
            if (CurInstr.Cond() == 0xE)
                RegCache.PutLiteral(CurInstr.A_Reg(12), op2.Imm);
            MOVI2R(rd, op2.Imm);
        }
        else
        {
            MOV(rd, op2.Reg.Rm, op2.ToArithOption());
        }
    }

    if (S)
    {
        if (FlagsNZNeeded())
            TST(rd, rd);
        Comp_RetriveFlags(false);
    }

    if (CurInstr.Info.Branches())
        Comp_JumpTo(rd, true, S);
}

void Compiler::A_Comp_ALUTriOp()
{
    bool S = CurInstr.Instr & (1 << 20);
    u32 op = (CurInstr.Instr >> 21) & 0xF;
    bool logical = (1 << op) & 0xF303;

    ARM64Reg rd = MapReg(CurInstr.A_Reg(12));
    ARM64Reg rn = MapReg(CurInstr.A_Reg(16));
    Op2 op2;
    A_Comp_GetOp2(S && logical, op2);

    if (op2.IsImm && op2.Imm == 0)
        op2 = Op2(WZR, ST_LSL, 0);
    
    if (logical)
        Comp_Logical(op, S, rd, rn, op2);
    else
        Comp_Arithmetic(op, S, rd, rn, op2);

    if (CurInstr.Info.Branches())
        Comp_JumpTo(rd, true, S);
}

void Compiler::A_Comp_Clz()
{
    Comp_AddCycles_C();

    ARM64Reg rd = MapReg(CurInstr.A_Reg(12));
    ARM64Reg rm = MapReg(CurInstr.A_Reg(0));

    CLZ(rd, rm);

    assert(Num == 0);
}

void Compiler::Comp_Mul_Mla(bool S, bool mla, ARM64Reg rd, ARM64Reg rm, ARM64Reg rs, ARM64Reg rn)
{
    if (Num == 0)
    {
        Comp_AddCycles_CI(S ? 3 : 1);
    }
    else
    {
        CLS(W0, rs);
        Comp_AddCycles_CI(mla ? 1 : 0, W0, ArithOption(W0, ST_LSR, 3));
    }

    if (mla)
        MADD(rd, rm, rs, rn);
    else
        MUL(rd, rm, rs);

    if (S && FlagsNZNeeded())
    {
        TST(rd, rd);
        Comp_RetriveFlags(false);
    }
}

void Compiler::A_Comp_Mul_Long()
{
    ARM64Reg rd = MapReg(CurInstr.A_Reg(16));
    ARM64Reg rm = MapReg(CurInstr.A_Reg(0));
    ARM64Reg rs = MapReg(CurInstr.A_Reg(8));
    ARM64Reg rn = MapReg(CurInstr.A_Reg(12));

    bool S = CurInstr.Instr & (1 << 20);
    bool add = CurInstr.Instr & (1 << 21);
    bool sign = CurInstr.Instr & (1 << 22);

    if (Num == 0)
    {
        Comp_AddCycles_CI(S ? 3 : 1);
    }
    else
    {
        if (sign)
            CLS(W0, rs);
        else
            CLZ(W0, rs);
        Comp_AddCycles_CI(0, W0, ArithOption(W0, ST_LSR, 3));
    }

    if (add)
    {
        MOV(W0, rn);
        BFI(X0, EncodeRegTo64(rd), 32, 32);
        if (sign)
            SMADDL(EncodeRegTo64(rn), rm, rs, X0);
        else
            UMADDL(EncodeRegTo64(rn), rm, rs, X0);
        if (S && FlagsNZNeeded())
            TST(EncodeRegTo64(rn), EncodeRegTo64(rn));
        UBFX(EncodeRegTo64(rd), EncodeRegTo64(rn), 32, 32);
    }
    else
    {
        if (sign)
            SMULL(EncodeRegTo64(rn), rm, rs);
        else
            UMULL(EncodeRegTo64(rn), rm, rs);
        if (S && FlagsNZNeeded())
            TST(EncodeRegTo64(rn), EncodeRegTo64(rn));
        UBFX(EncodeRegTo64(rd), EncodeRegTo64(rn), 32, 32);
    }
    
    if (S)
        Comp_RetriveFlags(false);
}

void Compiler::A_Comp_Mul_Short()
{
    ARM64Reg rd = MapReg(CurInstr.A_Reg(16));
    ARM64Reg rm = MapReg(CurInstr.A_Reg(0));
    ARM64Reg rs = MapReg(CurInstr.A_Reg(8));
    u32 op = (CurInstr.Instr >> 21) & 0xF;

    bool x = CurInstr.Instr & (1 << 5);
    bool y = CurInstr.Instr & (1 << 6);

    SBFX(W1, rs, y ? 16 : 0, 16);

    if (op == 0b1000)
    {
        // SMLAxy

        SBFX(W0, rm, x ? 16 : 0, 16);

        MUL(W0, W0, W1);

        ORRI2R(W1, RCPSR, 0x08000000);

        ARM64Reg rn = MapReg(CurInstr.A_Reg(12));
        ADDS(rd, W0, rn);

        CSEL(RCPSR, W1, RCPSR, CC_VS);

        CPSRDirty = true;

        Comp_AddCycles_C();
    }
    else if (op == 0b1011)
    {
        // SMULxy

        SBFX(W0, rm, x ? 16 : 0, 16);

        MUL(rd, W0, W1);

        Comp_AddCycles_C();
    }
    else if (op == 0b1010)
    {
        // SMLALxy

        ARM64Reg rn = MapReg(CurInstr.A_Reg(12));

        MOV(W2, rn);
        BFI(X2, rd, 32, 32);

        SBFX(W0, rm, x ? 16 : 0, 16);

        SMADDL(EncodeRegTo64(rn), W0, W1, X2);

        UBFX(EncodeRegTo64(rd), EncodeRegTo64(rn), 32, 32);

        Comp_AddCycles_CI(1);
    }
    else if (op == 0b1001)
    {
        // SMLAWy/SMULWy
        SMULL(X0, rm, W1);
        ASR(x ? EncodeRegTo64(rd) : X0, X0, 16);

        if (!x)
        {
            ORRI2R(W1, RCPSR, 0x08000000);

            ARM64Reg rn = MapReg(CurInstr.A_Reg(12));
            ADDS(rd, W0, rn);

            CSEL(RCPSR, W1, RCPSR, CC_VS);

            CPSRDirty = true;
        }

        Comp_AddCycles_C();
    }
}

void Compiler::A_Comp_Mul()
{
    ARM64Reg rd = MapReg(CurInstr.A_Reg(16));
    ARM64Reg rm = MapReg(CurInstr.A_Reg(0));
    ARM64Reg rs = MapReg(CurInstr.A_Reg(8));

    bool S = CurInstr.Instr & (1 << 20);
    bool mla = CurInstr.Instr & (1 << 21);
    ARM64Reg rn = INVALID_REG;
    if (mla)
        rn = MapReg(CurInstr.A_Reg(12));

    Comp_Mul_Mla(S, mla, rd, rm, rs, rn);
}

void Compiler::T_Comp_ShiftImm()
{
    Comp_AddCycles_C();

    u32 op = (CurInstr.Instr >> 11) & 0x3;
    int amount = (CurInstr.Instr >> 6) & 0x1F;

    ARM64Reg rd = MapReg(CurInstr.T_Reg(0));
    Op2 op2;
    op2.Reg.Rm = MapReg(CurInstr.T_Reg(3));
    Comp_RegShiftImm(op, amount, true, op2);
    if (op2.IsImm)
        MOVI2R(rd, op2.Imm);
    else
        MOV(rd, op2.Reg.Rm, op2.ToArithOption());
    if (FlagsNZNeeded())
        TST(rd, rd);

    Comp_RetriveFlags(false);
}

void Compiler::T_Comp_AddSub_()
{
    Comp_AddCycles_C();

    Op2 op2;
    if (CurInstr.Instr & (1 << 10))
        op2 = Op2((CurInstr.Instr >> 6) & 0x7);
    else
        op2 = Op2(MapReg(CurInstr.T_Reg(6)));
    
    Comp_Arithmetic(
        CurInstr.Instr & (1 << 9) ? 0x2 : 0x4,
        true,
        MapReg(CurInstr.T_Reg(0)),
        MapReg(CurInstr.T_Reg(3)),
        op2);
}

void Compiler::T_Comp_ALUImm8()
{
    Comp_AddCycles_C();

    u32 imm = CurInstr.Instr & 0xFF;
    int op = (CurInstr.Instr >> 11) & 0x3;

    ARM64Reg rd = MapReg(CurInstr.T_Reg(8));

    switch (op)
    {
    case 0:
        MOVI2R(rd, imm);
        if (FlagsNZNeeded())
            TST(rd, rd);
        Comp_RetriveFlags(false);
        break;
    case 1:
        Comp_Compare(0xA, rd, Op2(imm));
        break;
    case 2:
    case 3:
        Comp_Arithmetic(op == 2 ? 0x4 : 0x2, true, rd, rd, Op2(imm));
        break;
    }
}

void Compiler::T_Comp_ALU()
{
    int op = (CurInstr.Instr >> 6) & 0xF;
    ARM64Reg rd = MapReg(CurInstr.T_Reg(0));
    ARM64Reg rs = MapReg(CurInstr.T_Reg(3));
    
    if ((op >= 0x2 && op <= 0x4) || op == 0x7)
        Comp_AddCycles_CI(1);
    else
        Comp_AddCycles_C();

    switch (op)
    {
    case 0x0:
        Comp_Logical(0x0, true, rd, rd, Op2(rs));
        break;
    case 0x1:
        Comp_Logical(0x1, true, rd, rd, Op2(rs));
        break;
    case 0x2:
    case 0x3:
    case 0x4:
    case 0x7:
        {   
            Op2 op2;
            op2.Reg.Rm = rd;
            Comp_RegShiftReg(op == 0x7 ? 3 : (op - 0x2), true, op2, rs);
            MOV(rd, op2.Reg.Rm, op2.ToArithOption());
            if (FlagsNZNeeded())
                TST(rd, rd);
            Comp_RetriveFlags(false);
        }
        break;
    case 0x5:
        Comp_Arithmetic(0x5, true, rd, rd, Op2(rs));
        break;
    case 0x6:
        Comp_Arithmetic(0x6, true, rd, rd, Op2(rs));
        break;
    case 0x8:
        Comp_Compare(0x8, rd, Op2(rs));
        break;
    case 0x9:
        Comp_Arithmetic(0x3, true, rd, rs, Op2(0));
        break;
    case 0xA:
        Comp_Compare(0xA, rd, Op2(rs));
        break;
    case 0xB:
        Comp_Compare(0xB, rd, Op2(rs));
        break;
    case 0xC:
        Comp_Logical(0xC, true, rd, rd, Op2(rs));
        break;
    case 0xD:
        Comp_Mul_Mla(true, false, rd, rd, rs, INVALID_REG);
        break;
    case 0xE:
        Comp_Logical(0xE, true, rd, rd, Op2(rs));
        break;
    case 0xF:
        MVN(rd, rs);
        if (FlagsNZNeeded())
            TST(rd, rd);
        Comp_RetriveFlags(false);
        break;
    }
}

void Compiler::T_Comp_ALU_HiReg()
{
    u32 rd = ((CurInstr.Instr & 0x7) | ((CurInstr.Instr >> 4) & 0x8));
    ARM64Reg rdMapped = MapReg(rd);
    ARM64Reg rs = MapReg((CurInstr.Instr >> 3) & 0xF);

    u32 op = (CurInstr.Instr >> 8) & 0x3;

    Comp_AddCycles_C();

    switch (op)
    {
    case 0:
        Comp_Arithmetic(0x4, false, rdMapped, rdMapped, Op2(rs));
        break;
    case 1:
        Comp_Compare(0xA, rdMapped, rs);
        return;
    case 2:
        MOV(rdMapped, rs);
        break;
    }

    if (rd == 15)
    {
        Comp_JumpTo(rdMapped, false, false);
    }
}

void Compiler::T_Comp_AddSP()
{
    Comp_AddCycles_C();

    ARM64Reg sp = MapReg(13);
    u32 offset = (CurInstr.Instr & 0x7F) << 2;
    if (CurInstr.Instr & (1 << 7))
        SUB(sp, sp, offset);
    else
        ADD(sp, sp, offset);
}

void Compiler::T_Comp_RelAddr()
{
    Comp_AddCycles_C();

    ARM64Reg rd = MapReg(CurInstr.T_Reg(8));
    u32 offset = (CurInstr.Instr & 0xFF) << 2;
    if (CurInstr.Instr & (1 << 11))
    {
        ARM64Reg sp = MapReg(13);
        ADD(rd, sp, offset);
    }
    else
        MOVI2R(rd, (R15 & ~2) + offset);
}

}