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

#ifdef LITEV_JIT_LAZYFLAGS
#include <cstdio>
#include <cstdlib>
// ---- LAZY-FLAGS V3 eliminated-write counters (diagnostics only) ----
// Compile-time (emit-time) tallies of the flag traffic V3 removed vs V2. Printed once
// at process exit via a static object's destructor (runs from __cxa_atexit when main
// returns normally). The core library gets no LITEV_HEADLESS macro (that CMake option
// only add_subdirectory's the harness), so this lives under LITEV_JIT_LAZYFLAGS and is
// SILENT unless the LITEV_LF_STATS env var is set -> zero output on the shipping .dev
// APK, a printed line under the headless gate. The three counter increments run only at
// JIT COMPILE sites (never in emitted guest code), so they cost nothing at run time and
// are architecturally invisible. Single-writer (blocks compile on the emu thread).
namespace LazyFlagsV3Stat
{
    unsigned long long CVSlotWritesElided = 0;   // item 1a: CVInGPR C/V slot RMWs skipped (dead)
    unsigned long long PartialFlushUpgraded = 0; // item 1b: FR_HOST_NZ partial flush -> MRS+STR
    unsigned long long NativeCarryIn = 0;        // item 3: ADC/SBC consumed host C directly

    struct Reporter {
        ~Reporter() {
            if (getenv("LITEV_LF_STATS"))
                fprintf(stderr,
                    "[lazyflags-v3] eliminated-writes: CV-slot-elided=%llu "
                    "partial-flush-upgraded=%llu native-carry-in=%llu\n",
                    CVSlotWritesElided, PartialFlushUpgraded, NativeCarryIn);
        }
    };
    __attribute__((used)) Reporter g_reporter;
}
#define LFV3_STAT(name) (++melonDS::LazyFlagsV3Stat::name)
#else
#define LFV3_STAT(name) ((void)0)
#endif

#ifdef LITEV_JIT_LAZYFLAGS
// V3 item 1: per-block backward dead-flag liveness. melonDS already runs this pass
// (FloodFillSetFlags -> FetchedInstr::SetFlags gives the LIVE PRODUCED bits, gated by
// every producer flush); this companion retains the running live set as LiveIn[i]
// (flags live at the START of instr i, i.e. observed by i's body OR anything after it)
// so a flush emitted just before i's body can answer "are the C,V I would preserve
// dead?" — which SetFlags (a produced-bit mask) cannot express.
//
// Semantics mirror FloodFillSetFlags exactly: block exit = ALL-LIVE (0xF); a DEFINITE
// write (WriteFlags low nibble — the InstrInfo conditional transform already zeroes it
// for cond<0xE ARM ops and moves the maybe-write to the high nibble) KILLS; ReadFlags
// (which already folds in the condition's own reads) is a USE. A "barrier" — any point
// that can reach C++ / raise an exception / restore SPSR mid-block, where ARM::CPSR
// (fed from the slot) must be fully canonical — forces the live-in to 0xF. Barriers are
// over-approximated (SOUND-first): only a compilable, non-branching ARM ALU / multiply
// / CLZ body is a non-barrier (its every flag observation is already in Info.ReadFlags);
// EVERYTHING else — memory ops, MRS/MSR, SWI/undef/coproc, interpreter fallbacks,
// branches, and (conservatively) ALL Thumb bodies — is an all-live barrier. This
// matches the prompt's barrier list and Comp_ReconcileFlags' own conservatism.
void Compiler::Comp_ComputeFlagLiveness(FetchedInstr* instrs, int instrsCount)
{
    using namespace ARMInstrInfo;
    u8 live = 0xF; // live-out of the block (slot-canonical boundary contract)
    for (int i = instrsCount - 1; i >= 0; i--)
    {
        FetchedInstr& in = instrs[i];
        // `live` here == LiveOut[i]. Fold in instr i's own effect to get LiveIn[i].
        bool nonBarrier = false;
        const u16 kind = in.Info.Kind;
        // ARM data-processing / multiply / CLZ that stays in JIT and cannot escape.
        // Thumb is left entirely conservative (barrier) — item 1b simply never fires
        // inside Thumb blocks, which is sound (LiveIn stays 0xF there).
        if (!Thumb && !in.Info.Branches() && CanCompile(false, kind))
        {
            if (kind <= ak_MVN_IMM_S
                || (kind >= ak_MUL && kind <= ak_SMLAL)
                || kind == ak_CLZ)
                nonBarrier = true;
        }

        if (!nonBarrier)
            live = 0xF; // all-live barrier: everything before it is live
        else
        {
            u8 kill = in.Info.WriteFlags & 0x0F; // DEFINITE writes only
            u8 use  = in.Info.ReadFlags  & 0x0F; // reads (incl. condition reads)
            live = (u8)((live & ~kill) | use);
        }
        FlagsLiveIn[i] = live; // flags live at the START of instr i
    }
}
#endif

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
#ifdef LITEV_JIT_LAZYFLAGS
            Comp_CPSRInsertBitToMem(W2, W0, 29);   // guest C -> ARM::CPSR[29] (W2 scratch)
#else
            BFI(RCPSR, W0, 29, 1);
#endif
        }
        else
        {
            CMP(W1, 31);
            if (op == 2)
            {
                MOVI2R(W2, 31);
                CSEL(W1, W2, W1, CC_GT);
                ASRV(W0, op2.Reg.Rm, W1);
#ifdef LITEV_JIT_LAZYFLAGS
                Comp_CPSRInsertBitToMem(W2, W0, 29);   // W2 dead (was #31) -> reuse as word
#else
                BFI(RCPSR, W0, 29, 1);
#endif
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
#ifdef LITEV_JIT_LAZYFLAGS
                // W1 = carry (src), W2 free here (LSL/LSR sub-branch never wrote it); the
                // LDR/BFI/STR leave host NZCV (the CMP W1,31 flags) intact for the CSEL below.
                Comp_CPSRInsertBitToMem(W2, W1, 29);
#else
                BFI(RCPSR, W1, 29, 1);
#endif
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
#ifdef LITEV_JIT_LAZYFLAGS
            Comp_CPSRInsertBitToMem(W1, tmp, 29);  // shifter C -> ARM::CPSR[29]
#else
            BFI(RCPSR, tmp, 29, 1);
#endif
        }
        op2 = Op2(op2.Reg.Rm, ST_LSL, amount);
        return;
    case 1: // LSR
        if (S)
        {
            UBFX(tmp, op2.Reg.Rm, (amount ? amount : 32) - 1, 1);
#ifdef LITEV_JIT_LAZYFLAGS
            Comp_CPSRInsertBitToMem(W1, tmp, 29);  // shifter C -> ARM::CPSR[29]
#else
            BFI(RCPSR, tmp, 29, 1);
#endif
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
#ifdef LITEV_JIT_LAZYFLAGS
            Comp_CPSRInsertBitToMem(W1, tmp, 29);  // shifter C -> ARM::CPSR[29]
#else
            BFI(RCPSR, tmp, 29, 1);
#endif
        }
        op2 = Op2(op2.Reg.Rm, ST_ASR, amount ? amount : 31);
        return;
    case 3: // ROR
        if (amount == 0)
        {
#ifdef LITEV_JIT_LAZYFLAGS
            // RRX: read old guest C from the ARM::JitNZCV slot, and (if S) write new
            // C = Rm[0] back to it. Single load (W1) serves both; read must precede the
            // write. tmp(result)/Rm/W1 are distinct, and the RMW leaves host PSTATE
            // untouched. RRX reads C so Comp_ReconcileFlags materialised any residency to
            // the slot first -> slot[29] is the canonical old guest C.
            LDR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, JitNZCV));
            UBFX(tmp, W1, 29, 1);
            LSL(tmp, tmp, 31);
            if (S)
            {
                BFI(W1, op2.Reg.Rm, 29, 1);
                STR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, JitNZCV));
            }
            ORR(tmp, tmp, op2.Reg.Rm, ArithOption(tmp, ST_LSR, 1));
            op2 = Op2(tmp, ST_LSL, 0);
#else
            UBFX(tmp, RCPSR, 29, 1);
            LSL(tmp, tmp, 31);
            if (S)
                BFI(RCPSR, op2.Reg.Rm, 29, 1);
            ORR(tmp, tmp, op2.Reg.Rm, ArithOption(tmp, ST_LSR, 1));

            op2 = Op2(tmp, ST_LSL, 0);
#endif
        }
        else
        {
            if (S)
            {
                UBFX(tmp, op2.Reg.Rm, amount - 1, 1);
#ifdef LITEV_JIT_LAZYFLAGS
                Comp_CPSRInsertBitToMem(W1, tmp, 29);  // shifter C -> ARM::CPSR[29]
#else
                BFI(RCPSR, tmp, 29, 1);
#endif
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

#ifdef LITEV_JIT_LAZYFLAGS
    // V2: the guest NZCV nibble is homed in the dedicated ARM::JitNZCV slot (DraStic's
    // cpu+0x2354 analog), NOT the full ARM::CPSR word (V1's full-word LDR-merge-STR is the
    // measured A55 regression). Two shapes:
    //   * FULL nibble (retriveCV && all four flags set — the hot arithmetic S / CMP / CMN):
    //     host PSTATE already holds the COMPLETE guest NZCV, so `MRS w0,NZCV; STR w0,[slot]`
    //     writes it in 2 instrs with NO load — MRS zeroes the low bits and the slot is
    //     NZCV-only, so the raw store is clean. This is the big new win (V1 paid 8-10 here)
    //     and it is independent of FLAGMERGE, so conditional producers win it too.
    //   * PARTIAL (logical N,Z; or a sub-nibble arithmetic form): the un-written slot bits
    //     (e.g. barrel-shifter C, preserved V) must survive, so RMW the slot per set flag.
    // Bit-exact to the CSET/BFI extraction: MRS[31:28] == the CSET(CC_MI/EQ/CS/VS)+BFI
    // result for the same host PSTATE.
    {
        if (retriveCV && (CurInstr.SetFlags & 0xF) == 0xF)
        {
            MRS(X0, FIELD_NZCV);
            STR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, JitNZCV));
            return;
        }
        u32 wm = 0;
        if (CurInstr.SetFlags & 0x4) wm |= 1u << 30; // Z
        if (CurInstr.SetFlags & 0x8) wm |= 1u << 31; // N
        if (retriveCV)
        {
            if (CurInstr.SetFlags & 0x2) wm |= 1u << 29; // C
            if (CurInstr.SetFlags & 0x1) wm |= 1u << 28; // V
        }
        if (!wm)
            return;
        LDR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, JitNZCV));
        if (CurInstr.SetFlags & 0x4) { CSET(W0, CC_EQ); BFI(W1, W0, 30, 1); }
        if (CurInstr.SetFlags & 0x8) { CSET(W0, CC_MI); BFI(W1, W0, 31, 1); }
        if (retriveCV)
        {
            if (CurInstr.SetFlags & 0x2) { CSET(W0, CC_CS); BFI(W1, W0, 29, 1); }
            if (CurInstr.SetFlags & 0x1) { CSET(W0, CC_VS); BFI(W1, W0, 28, 1); }
        }
        STR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, JitNZCV));
    }
    return;
#endif
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

#ifdef LITEV_JIT_LAZYFLAGS
    // V2 MaterializeFlagsToSlot: flush the host-PSTATE-resident guest flags into the
    // dedicated ARM::JitNZCV slot (NOT the full ARM::CPSR word — V1's LDR-merge-STR was
    // the regression). Two shapes:
    //   * FULL nibble (m == 0xF <=> FR_HOST_FULL with all four deferred; only an
    //     arithmetic producer ever defers C,V, so m==0xF implies NZCVCondValid): host
    //     PSTATE holds the complete guest NZCV, so `MRS w0,NZCV; STR w0,[slot]` = 2
    //     instrs, NO load. MRS zeroes the low bits -> the slot (NZCV-only) stays clean.
    //     Replaces V1's LDR + 4*(CSET/BFI) + STR (10 instrs) at every block boundary /
    //     pre-non-transparent-body flush after a full arithmetic S-op.
    //   * PARTIAL (m == 0xC under FR_HOST_NZ: N,Z host-resident, C,V canonical in the
    //     slot; or a sub-nibble form): the un-deferred slot bits (C,V) must survive, so
    //     RMW the slot. LDR/CSET/BFI/STR do NOT touch host PSTATE, so a caller (e.g.
    //     CheckCondition) may still branch natively on the resident flags afterwards.
    // V3 item 1(b): upgrade the partial FR_HOST_NZ flush (N,Z host-resident, C,V
    // canonical in the slot) to the 2-instr full-nibble MRS+STR when the C,V it would
    // preserve are DEAD to their next definition. FR_HOST_NZ (!NZCVCondValid) with
    // m==0xC means the host op was a LOGICAL producer: host N,Z == guest N,Z, host
    // C,V == 0 (AArch64 zeroes them). MRS therefore yields N,Z,0,0; storing it clobbers
    // the slot's guest C,V with 0 — SOUND only if C,V are dead (never observed before
    // their next definition). FlagsLiveInCur (barrier-conservative; folds in this and
    // every later instr's flag reads, and every C++/exception escape) is the exact
    // dead test; it DEFAULTS to 0xF at block-end / trampoline call sites so those never
    // upgrade. This replaces LDR + 2*(CSET/BFI) + STR (6 instrs) with MRS + STR (2).
    if (m != 0xF && !NZCVCondValid && m == 0xC && (FlagsLiveInCur & 0x3) == 0)
    {
        MRS(X0, FIELD_NZCV);
        STR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, JitNZCV));
        LFV3_STAT(PartialFlushUpgraded);
    }
    else if (m == 0xF)
    {
        MRS(X0, FIELD_NZCV);
        STR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, JitNZCV));
    }
    else
    {
        LDR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, JitNZCV));
        if (m & 0x4) { CSET(W0, CC_EQ); BFI(W1, W0, 30, 1); } // Z
        if (m & 0x8) { CSET(W0, CC_MI); BFI(W1, W0, 31, 1); } // N
        if (m & 0x2) { CSET(W0, CC_CS); BFI(W1, W0, 29, 1); } // C
        if (m & 0x1) { CSET(W0, CC_VS); BFI(W1, W0, 28, 1); } // V
        STR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, JitNZCV));
    }
#else
    if (m & 0x4) { CSET(W0, CC_EQ); BFI(RCPSR, W0, 30, 1); } // Z
    if (m & 0x8) { CSET(W0, CC_MI); BFI(RCPSR, W0, 31, 1); } // N
    if (m & 0x2) { CSET(W0, CC_CS); BFI(RCPSR, W0, 29, 1); } // C
    if (m & 0x1) { CSET(W0, CC_VS); BFI(RCPSR, W0, 28, 1); } // V
#endif
}

#ifdef LITEV_JIT_LAZYFLAGS
void Compiler::Comp_CPSRInsertBitToMem(ARM64Reg word, ARM64Reg src, int pos)
{
    // V2: read-modify-write the dedicated ARM::JitNZCV slot (the guest NZCV home while a
    // JIT slice runs): word = *slot; word[pos] = src[0]; *slot = word. Used ONLY for the
    // barrel-shifter carry (guest C, bit 29) of a shifted logical-S op. The op's
    // Comp_ReconcileFlags materialised any prior residency to the slot first (a flag-
    // writing body is never transparent), so the slot is canonical here and the RMW
    // preserves the other three flag bits. LDR/BFI/STR do not touch host PSTATE, so the
    // following logical host op (which then defers N,Z) is unaffected. `pos` is always 29.
    LDR(INDEX_UNSIGNED, word, RCPU, offsetof(ARM, JitNZCV));
    BFI(word, src, pos, 1);
    STR(INDEX_UNSIGNED, word, RCPU, offsetof(ARM, JitNZCV));
}
#endif

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
#ifdef LITEV_JIT_LAZYFLAGS
        // V3 item 3 (native carry-in): if the ONLY deferred flag this body reads is C,
        // and the FULL guest NZCV is genuinely resident in host PSTATE (FR_HOST_FULL),
        // an ADC/SBC with a PSTATE-safe register op2 can consume host C DIRECTLY via the
        // AArch64 ADCS/SBCS form (carry-in == guest C, 0 setup instrs) instead of
        // materialising C to the slot and re-reading it (LDR+UBFX+CMP). We KEEP host
        // PSTATE live (skip the materialize): nothing between here and the ADCS/SBCS
        // clobbers it — for a register op2 with a non-register-specified shift,
        // A_Comp_GetOp2 emits only cycle ADDs and, if shifted, a no-flag MOV; Thumb
        // ADC/SBC op2 is a plain register. We exclude register-specified-shift (its CMP
        // clobbers host NZCV) and ROR/RRX (reads C for the shift, from the slot). The old
        // resident N,Z,V are DEAD across the ADCS/SBCS (a full producer this body does not
        // read) — identical soundness to reconcile case (B). Comp_Arithmetic re-defers the
        // fresh NZCV, so residency stays self-consistent.
        if (NZCVCondValid && (NZCVDeferred & 0x2) && read == 0x2)
        {
            bool nativeCarry;
            if (!Thumb)
            {
                u32 instr = CurInstr.Instr;
                u32 aop = (instr >> 21) & 0xF;
                nativeCarry = (aop == 0x5 || aop == 0x6)       // ADC / SBC
                    && (instr & (1 << 20))                     // S-form
                    && !(instr & (1 << 25))                    // register op2
                    && !(instr & (1 << 4))                     // not register-specified shift
                    && ((instr >> 5) & 3) != 3;                // not ROR / RRX
            }
            else
                nativeCarry = (CurInstr.Info.Kind == ARMInstrInfo::tk_ADC_REG
                            || CurInstr.Info.Kind == ARMInstrInfo::tk_SBC_REG);
            if (nativeCarry)
            {
                CarryInHostResident = true; // Comp_Arithmetic emits ADCS/SBCS directly
                return;                     // keep host PSTATE; do NOT materialize
            }
        }
#endif
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
#ifdef LITEV_JIT_LAZYFLAGS
        // V2 carry-in from the guest C flag, read from the ARM::JitNZCV slot. The
        // invariant guarantees C is canonical in the slot here: an ADC/SBC/RSC reads C,
        // so Comp_ReconcileFlags/CheckCondition already materialised any host-resident C
        // to the slot (case A), and under FR_HOST_NZ / FR_MEMORY guest C already lives in
        // slot[29].
        // V3 item 3: when Comp_ReconcileFlags proved the guest C is live in host PSTATE-C
        // (FR_HOST_FULL) for this S-form register-op2 ADC, skip the slot read entirely —
        // ADCS consumes host C directly below.
        if (!CarryInHostResident)
        {
            LDR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, JitNZCV));
            UBFX(W2, W1, 29, 1);
        }
#else
        UBFX(W2, RCPSR, 29, 1);
#endif
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
#ifdef LITEV_JIT_LAZYFLAGS
                // V3 item 3: host PSTATE-C already holds guest C (ADCS reads it as
                // carry-in) — no `CMP W2,1` seed needed. Bit-exact: ADCS' carry-in and
                // resulting N,Z,C,V are identical whether host C came from CMP W2,1 or
                // was left resident by the producer.
                if (CarryInHostResident)
                    LFV3_STAT(NativeCarryIn);
                else
#endif
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
#ifdef LITEV_JIT_LAZYFLAGS
        // V2 carry-in from the guest C flag, read from the ARM::JitNZCV slot. The
        // invariant guarantees C is canonical in the slot here: an ADC/SBC/RSC reads C,
        // so Comp_ReconcileFlags/CheckCondition already materialised any host-resident C
        // to the slot (case A), and under FR_HOST_NZ / FR_MEMORY guest C already lives in
        // slot[29].
        // V3 item 3: skip the slot read when guest C is live in host PSTATE-C — SBCS
        // consumes host C directly. AArch64 SBC and ARM SBC both use NOT-borrow, so a
        // resident host C == guest C makes this exact.
        if (!CarryInHostResident)
        {
            LDR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, JitNZCV));
            UBFX(W2, W1, 29, 1);
        }
#else
        UBFX(W2, RCPSR, 29, 1);
#endif
        if (S && !op2.IsImm)
        {
            if (op2.Reg.ShiftAmount > 0)
            {
                MOV(W0, op2.Reg.Rm, op2.ToArithOption());
                op2 = Op2(W0, ST_LSL, 0);
            }
#ifdef LITEV_JIT_LAZYFLAGS
            if (CarryInHostResident)
                LFV3_STAT(NativeCarryIn);
            else
#endif
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
#ifdef LITEV_JIT_LAZYFLAGS
        // V2 carry-in from the guest C flag, read from the ARM::JitNZCV slot. The
        // invariant guarantees C is canonical in the slot here: an ADC/SBC/RSC reads C,
        // so Comp_ReconcileFlags/CheckCondition already materialised any host-resident C
        // to the slot (case A), and under FR_HOST_NZ / FR_MEMORY guest C already lives in
        // slot[29].
        LDR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, JitNZCV));
        UBFX(W2, W1, 29, 1);
#else
        UBFX(W2, RCPSR, 29, 1);
#endif
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
#ifdef LITEV_JIT_LAZYFLAGS
            // C in W2, V in W3 (GPR-computed for ADC/SBC/RSC-imm & RSC) -> ARM::JitNZCV
            // slot RMW; N,Z then via Comp_RetriveFlags(false) (its own slot RMW). The slot
            // is canonical here (Comp_ReconcileFlags materialised the prior residency
            // before this arithmetic producer). W1 is dead.
            //
            // V3 item 1(a): this C,V slot RMW is the last eager PRODUCER write V2 emitted
            // ungated by liveness (Comp_RetriveFlags and the deferral masks are all already
            // SetFlags-gated). Gate it the same way: SetFlags (FloodFillSetFlags, all-live
            // at block exit) tells us which of C,V are live; skip the dead ones, and when
            // BOTH are dead the whole load/store disappears. Sound by the same argument as
            // every other SetFlags-gated flush: a dead bit is never observed before its
            // next definition, and anything reaching the block boundary is live -> written.
            {
                u8 cvsf = CurInstr.SetFlags & 0x3;
                // V2 unconditionally wrote BOTH C and V; count each dead bit we now skip
                // (both the full-elision cvsf==0 and the partial case, e.g. a GE/LT/GT/LE
                // consumer reads N,V so C is dead -> skip the C BFI).
                if (!(cvsf & 0x2)) LFV3_STAT(CVSlotWritesElided); // C dead
                if (!(cvsf & 0x1)) LFV3_STAT(CVSlotWritesElided); // V dead
                if (cvsf)
                {
                    LDR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, JitNZCV));
                    if (cvsf & 0x2) BFI(W1, W2, 29, 1); // C
                    if (cvsf & 0x1) BFI(W1, W3, 28, 1); // V
                    STR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, JitNZCV));
                }
            }
            Comp_RetriveFlags(false);
#else
            BFI(RCPSR, W2, 29, 1);
            BFI(RCPSR, W3, 28, 1);
            Comp_RetriveFlags(false);
#endif
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
#ifdef LITEV_JIT_LAZYFLAGS
            // Compile-time-known shifter C (imm-rotate carry) -> ARM::JitNZCV slot[29] RMW.
            // The slot is canonical (this logical-S op's Comp_ReconcileFlags materialised
            // any prior residency to it first); the RMW preserves the other flag bits.
            LDR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, JitNZCV));
            if (imm & 0x80000000)
                ORRI2R(W1, W1, 1 << 29);
            else
                ANDI2R(W1, W1, ~(1 << 29));
            STR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, JitNZCV));
#else
            if (imm & 0x80000000)
                ORRI2R(RCPSR, RCPSR, 1 << 29);
            else
                ANDI2R(RCPSR, RCPSR, ~(1 << 29));
#endif
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

#ifdef LITEV_JIT_LAZYFLAGS
        // Sticky Q (bit 27) is a control bit -> ARM::CPSR memory RMW conditioned on the
        // ADDS overflow (host V). Flags are already canonical in memory (SMLAxy is not a
        // transparent body, so ReconcileFlags materialized the prior residency); the ADDS
        // clobbers host NZCV but SMLAxy does not set guest NZCV, so nothing is deferred and
        // memory NZCV (preserved by the RMW) stays correct. W2 = word scratch.
        LDR(INDEX_UNSIGNED, W2, RCPU, offsetof(ARM, CPSR));
        ORRI2R(W1, W2, 0x08000000);

        ARM64Reg rn = MapReg(CurInstr.A_Reg(12));
        ADDS(rd, W0, rn);

        CSEL(W2, W1, W2, CC_VS);
        STR(INDEX_UNSIGNED, W2, RCPU, offsetof(ARM, CPSR));

        CPSRDirty = true;
#else
        ORRI2R(W1, RCPSR, 0x08000000);

        ARM64Reg rn = MapReg(CurInstr.A_Reg(12));
        ADDS(rd, W0, rn);

        CSEL(RCPSR, W1, RCPSR, CC_VS);

        CPSRDirty = true;
#endif

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
#ifdef LITEV_JIT_LAZYFLAGS
            // Sticky Q (bit 27) -> ARM::CPSR memory RMW on ADDS overflow (as SMLAxy above).
            LDR(INDEX_UNSIGNED, W2, RCPU, offsetof(ARM, CPSR));
            ORRI2R(W1, W2, 0x08000000);

            ARM64Reg rn = MapReg(CurInstr.A_Reg(12));
            ADDS(rd, W0, rn);

            CSEL(W2, W1, W2, CC_VS);
            STR(INDEX_UNSIGNED, W2, RCPU, offsetof(ARM, CPSR));

            CPSRDirty = true;
#else
            ORRI2R(W1, RCPSR, 0x08000000);

            ARM64Reg rn = MapReg(CurInstr.A_Reg(12));
            ADDS(rd, W0, rn);

            CSEL(RCPSR, W1, RCPSR, CC_VS);

            CPSRDirty = true;
#endif
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