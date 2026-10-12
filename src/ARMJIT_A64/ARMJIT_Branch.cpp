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
#include "../NDS.h"

using namespace Arm64Gen;

// hack
const int kCodeCacheTiming = 3;

namespace melonDS
{

template <typename T>
void JumpToTrampoline(T* cpu, u32 addr, bool changeCPSR)
{
    cpu->JumpTo(addr, changeCPSR);
}

#ifdef LITEV_JIT_BGEMIT
// What Comp_JumpTo(u32) does to the CPU at compile time (in program order), and the timing it
// bakes in. Same calls as the base build's inline code.
void Compiler::JumpToCpuEffects(ARM* cpu, u32 addr, u32 r15, u32& regionCodeCycles, u32& cycles)
{
    cycles = 0;
    if (cpu->Num == 0)
    {
        ARMv5* cpu9 = (ARMv5*)cpu;

        u32 oldregion = r15 >> 24;
        u32 newregion = addr >> 24;

        regionCodeCycles = cpu9->MemTimings[addr >> 12][0];
        u32 compileTimeCodeCycles = cpu9->RegionCodeCycles;
        cpu9->RegionCodeCycles = regionCodeCycles;

        bool setupRegion = newregion != oldregion;
        if (setupRegion)
            cpu9->SetupCodeMem(addr);

        if (addr & 0x1)
        {
            addr &= ~0x1;
            if (addr & 0x2)
            {
                cpu9->CodeRead32(addr-2, true);
                cycles += cpu9->CodeCycles;
                cpu9->CodeRead32(addr+2, false);
                cycles += cpu9->CodeCycles;
            }
            else
            {
                cpu9->CodeRead32(addr, true);
                cycles += cpu9->CodeCycles;
            }
        }
        else
        {
            addr &= ~0x3;
            cpu9->CodeRead32(addr, true);
            cycles += cpu9->CodeCycles;
            cpu9->CodeRead32(addr+4, false);
            cycles += cpu9->CodeCycles;
        }

        cpu9->RegionCodeCycles = compileTimeCodeCycles;
        if (setupRegion)
            cpu9->SetupCodeMem(r15);
    }
    else
    {
        ARMv4* cpu7 = (ARMv4*)cpu;
        regionCodeCycles = 0;

        u32 codeRegion = addr >> 24;
        u32 codeCycles = addr >> 15; // cheato

        cpu7->CodeRegion = codeRegion;
        cpu7->CodeCycles = codeCycles;

        if (addr & 0x1)
        {
            addr &= ~0x1;
            u32 compileTimePC = cpu->R[15];
            cpu->R[15] = addr+2;
            cycles += NDS.ARM7MemTimings[codeCycles][0] + NDS.ARM7MemTimings[codeCycles][1];
            cpu->R[15] = compileTimePC;
        }
        else
        {
            addr &= ~0x3;
            u32 compileTimePC = cpu->R[15];
            cpu->R[15] = addr+4;
            cycles += NDS.ARM7MemTimings[codeCycles][2] + NDS.ARM7MemTimings[codeCycles][3];
            cpu->R[15] = compileTimePC;
        }

        cpu7->CodeRegion = r15 >> 24;
        cpu7->CodeCycles = addr >> 15;
    }
}
#endif

void Compiler::Comp_JumpTo(u32 addr, bool forceNonConstantCycles)
{
    // we can simplify constant branches by a lot
    // it's not completely safe to assume stuff like, which instructions to preload
    // we'll see how it works out

#ifdef LITEV_JIT_FIXEDREG
    Comp_MaterializeFlags();
#endif
#ifdef LITEV_JIT_RAS
    Comp_RasCallRet(false, false);
#endif
    IrregularCycles = true;

    u32 newPC;
    u32 cycles = 0;
    bool setupRegion = false;

    if (addr & 0x1 && !Thumb)
    {
        CPSRDirty = true;
#ifdef LITEV_JIT_LAZYFLAGS
        // Set guest T-bit in ARM::CPSR memory (control RMW). Comp_MaterializeFlags above
        // already flushed host NZCV, so the word is canonical; W1 is free here.
        LDR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, CPSR));
        ORRI2R(W1, W1, 0x20);
        STR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, CPSR));
#else
        ORRI2R(RCPSR, RCPSR, 0x20);
#endif
    }
    else if (!(addr & 0x1) && Thumb)
    {
        CPSRDirty = true;
#ifdef LITEV_JIT_LAZYFLAGS
        LDR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, CPSR));
        ANDI2R(W1, W1, ~0x20);
        STR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, CPSR));
#else
        ANDI2R(RCPSR, RCPSR, ~0x20);
#endif
    }

#ifdef LITEV_JIT_BGEMIT
    // The CPU-field part (timing lookups through the CPU, fields left changed) is
    // JumpToCpuEffects; a background-emitted block reads its captured result instead.
    {
        u32 regionCodeCycles = 0;
        if (CurJob)
        {
            JitEmitJob::Snap* sn = EmitSnap(JitEmitJob::SnapJump, CurIdx);
            if (!sn || sn->A2 != addr) EmitSnapMissing("jump", addr);
            regionCodeCycles = sn->A;
            cycles = sn->B;
        }
        else
        {
            JumpToCpuEffects(CurCPU, addr, R15, regionCodeCycles, cycles);
            if (VerifyJob)
            {
                JitEmitJob::Snap* sn = VerifyJob->Find(JitEmitJob::SnapJump, CurIdx);
                if (!sn || sn->A2 != addr || sn->A != regionCodeCycles || sn->B != cycles)
                    VerifyFail("jump", addr, sn ? sn->B : ~0u, cycles);
                sn->Used = 1;
            }
        }
        if (Num == 0)
        {
            MOVI2R(W0, regionCodeCycles);
            STR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARMv5, RegionCodeCycles));
        }
        else
        {
            MOVI2R(W0, addr >> 24);
            STR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, CodeRegion));
            MOVI2R(W0, addr >> 15);
            STR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, CodeCycles));
        }
        if (addr & 0x1)
        {
            addr &= ~0x1;
            newPC = addr+2;
        }
        else
        {
            addr &= ~0x3;
            newPC = addr+4;
        }
    }
#else
    if (Num == 0)
    {
        ARMv5* cpu9 = (ARMv5*)CurCPU;

        u32 oldregion = R15 >> 24;
        u32 newregion = addr >> 24;

        u32 regionCodeCycles = cpu9->MemTimings[addr >> 12][0];
        u32 compileTimeCodeCycles = cpu9->RegionCodeCycles;
        cpu9->RegionCodeCycles = regionCodeCycles;

        MOVI2R(W0, regionCodeCycles);
        STR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARMv5, RegionCodeCycles));

        setupRegion = newregion != oldregion;
        if (setupRegion)
            cpu9->SetupCodeMem(addr);

        if (addr & 0x1)
        {
            addr &= ~0x1;
            newPC = addr+2;

            // two-opcodes-at-once fetch
            // doesn't matter if we put garbage in the MSbs there
            if (addr & 0x2)
            {
                cpu9->CodeRead32(addr-2, true);
                cycles += cpu9->CodeCycles;
                cpu9->CodeRead32(addr+2, false);
                cycles += CurCPU->CodeCycles;
            }
            else
            {
                cpu9->CodeRead32(addr, true);
                cycles += cpu9->CodeCycles;
            }
        }
        else
        {
            addr &= ~0x3;
            newPC = addr+4;

            cpu9->CodeRead32(addr, true);
            cycles += cpu9->CodeCycles;
            cpu9->CodeRead32(addr+4, false);
            cycles += cpu9->CodeCycles;
        }

        cpu9->RegionCodeCycles = compileTimeCodeCycles;
        if (setupRegion)
            cpu9->SetupCodeMem(R15);
#ifdef LITEV_FAST_TIMING
        if (LITEV_FT) cycles = FastTiming.R9;   // fast timing: a constant refill
#endif
    }
    else
    {
        ARMv4* cpu7 = (ARMv4*)CurCPU;

        u32 codeRegion = addr >> 24;
        u32 codeCycles = addr >> 15; // cheato

        cpu7->CodeRegion = codeRegion;
        cpu7->CodeCycles = codeCycles;

        MOVI2R(W0, codeRegion);
        STR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, CodeRegion));
        MOVI2R(W0, codeCycles);
        STR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, CodeCycles));

        if (addr & 0x1)
        {
            addr &= ~0x1;
            newPC = addr+2;

            // this is necessary because ARM7 bios protection
            u32 compileTimePC = CurCPU->R[15];
            CurCPU->R[15] = newPC;

            cycles += NDS.ARM7MemTimings[codeCycles][0] + NDS.ARM7MemTimings[codeCycles][1];

            CurCPU->R[15] = compileTimePC;
        }
        else
        {
            addr &= ~0x3;
            newPC = addr+4;

            u32 compileTimePC = CurCPU->R[15];
            CurCPU->R[15] = newPC;

            cycles += NDS.ARM7MemTimings[codeCycles][2] + NDS.ARM7MemTimings[codeCycles][3];

            CurCPU->R[15] = compileTimePC;
        }

        cpu7->CodeRegion = R15 >> 24;
        cpu7->CodeCycles = addr >> 15;
#ifdef LITEV_FAST_TIMING
        if (LITEV_FT) cycles = FastTiming.R7;
#endif
    }

#endif

#ifdef LITEV_JIT_LINK
    StaticExitNewPC = newPC;
#endif
#ifdef LITEV_EXIT_PROTO_PC
    // Skip the store when this exit is certainly a link exit: the taken edge of a
    // conditional branch followed not-taken (Comp_BranchSpecialBehaviour), or an
    // unconditional static branch ending the block (block-end tail). EmitLinkExit's
    // cold path stores newPC instead; EmitBlockExit aborts if this guess is ever wrong.
    bool linkExit = (CurInstr.BranchFlags & branch_FollowCondNotTaken)
        || !(Thumb ? (CurInstr.Info.Kind == ARMInstrInfo::tk_BCOND) : (CurInstr.Cond() < 0xE));
    if (Exit && linkExit && ExitPCElidable(addr))
        PCElided = true;
    else
#endif
    if (Exit)
    {
        MOVI2R(W0, newPC);
        STR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, R[15]));
    }
    if ((Thumb || CurInstr.Cond() >= 0xE) && !forceNonConstantCycles)
        ConstantCycles += cycles;
    else
        ADD(RCycles, RCycles, cycles);

#ifdef LITEV_JIT_LINK
    // liteDS-v2 Unit 4: this exit has a compile-time-constant, same-mode target.
    // `addr` is now masked (thumb bit / alignment stripped) == the target block's
    // key (blockAddr) that JitBlocks / the dispatcher tag lookup use. The T-bit is
    // committed into CPSR above, so linking straight into the target block (compiled
    // for that instruction-set state) is correct across ARM<->Thumb transitions.
    HasStaticExit = true;
    StaticExitTarget = addr;
    StaticExitCond = Thumb ? (CurInstr.Info.Kind == ARMInstrInfo::tk_BCOND)
                           : (CurInstr.Cond() < 0xE);
#endif
}


#ifdef LITEV_REFILL_CHECK
// Diagnostic (-DLITEV_REFILL_CHECK=1): every indirect-jump stub run is compared with the refill the
// interpreter's ARMv5/ARMv4::JumpTo charges for the same target (cycles, and ARM9 RegionCodeCycles).
struct { u32 Addr; s32 C0; u32 Mode; } RefillDiag;   // Mode: 0 ARM, 1 Thumb, 2 from bit 0
static u64 RefillN[2], RefillBad[2];
static struct RefillPrint { ~RefillPrint() { fprintf(stderr, "refill_check arm9 %llu jumps %llu mismatches, arm7 %llu jumps %llu mismatches\n",
    (unsigned long long)RefillN[0], (unsigned long long)RefillBad[0], (unsigned long long)RefillN[1], (unsigned long long)RefillBad[1]); } } RefillPrintAtExit;

static void RefillCheck9(ARMv5* cpu, s32 cycles)
{
    u32 a = RefillDiag.Addr;
    bool thumb = RefillDiag.Mode == 2 ? (a & 1) : RefillDiag.Mode;
    const s32 got = cycles - RefillDiag.C0, stubRcc = cpu->RegionCodeCycles, cc = cpu->CodeCycles;
    cpu->RegionCodeCycles = cpu->MemTimings[a >> 12][0];
    s32 exp = 0;
    if (thumb)
    {
        a &= ~1u;
        if (a & 2) { cpu->CodeRead32(a-2, true); exp += cpu->CodeCycles; cpu->CodeRead32(a+2, false); exp += cpu->CodeCycles; }
        else       { cpu->CodeRead32(a, true); exp += cpu->CodeCycles; }
    }
    else
    {
        a &= ~3u;
        cpu->CodeRead32(a, true); exp += cpu->CodeCycles; cpu->CodeRead32(a+4, false); exp += cpu->CodeCycles;
    }
    RefillN[0]++;
    if (exp != got || stubRcc != cpu->RegionCodeCycles) RefillBad[0]++;
    cpu->RegionCodeCycles = stubRcc; cpu->CodeCycles = cc;
}

static void RefillCheck7(ARMv4* cpu, s32 cycles)
{
    const u32 a = RefillDiag.Addr;
    const bool thumb = RefillDiag.Mode == 2 ? (a & 1) : RefillDiag.Mode;
    const u8* t = cpu->NDS.ARM7MemTimings[a >> 15];
    const s32 exp = thumb ? t[0] + t[1] : t[2] + t[3];
    RefillN[1]++;
    if (exp != cycles - RefillDiag.C0) RefillBad[1]++;
}
#define REFILL_ENTRY(k) { MOVP2R(X16, &RefillDiag); STR(INDEX_UNSIGNED, W0, X16, 0); STR(INDEX_UNSIGNED, RCycles, X16, 4); \
    MOVI2R(W17, (k) == 0 ? 2 : (k) == 1 ? 0 : 1); STR(INDEX_UNSIGNED, W17, X16, 8); }
#define REFILL_CALL(fn) { SUB(SP, SP, 160); \
    STP(INDEX_SIGNED, X0, X1, SP, 0); STP(INDEX_SIGNED, X2, X3, SP, 16); STP(INDEX_SIGNED, X4, X5, SP, 32); \
    STP(INDEX_SIGNED, X6, X7, SP, 48); STP(INDEX_SIGNED, X8, X9, SP, 64); STP(INDEX_SIGNED, X10, X11, SP, 80); \
    STP(INDEX_SIGNED, X12, X13, SP, 96); STP(INDEX_SIGNED, X14, X15, SP, 112); STP(INDEX_SIGNED, X16, X17, SP, 128); \
    STP(INDEX_SIGNED, X18, X30, SP, 144); \
    MOV(W1, RCycles); MOV(X0, RCPU); MOVP2R(X16, (void*)fn); BLR(X16); \
    LDP(INDEX_SIGNED, X0, X1, SP, 0); LDP(INDEX_SIGNED, X2, X3, SP, 16); LDP(INDEX_SIGNED, X4, X5, SP, 32); \
    LDP(INDEX_SIGNED, X6, X7, SP, 48); LDP(INDEX_SIGNED, X8, X9, SP, 64); LDP(INDEX_SIGNED, X10, X11, SP, 80); \
    LDP(INDEX_SIGNED, X12, X13, SP, 96); LDP(INDEX_SIGNED, X14, X15, SP, 112); LDP(INDEX_SIGNED, X16, X17, SP, 128); \
    LDP(INDEX_SIGNED, X18, X30, SP, 144); ADD(SP, SP, 160); }
#else
#define REFILL_ENTRY(k)
#define REFILL_CALL(fn)
#endif

// Fetch cost of ARMv5::CodeRead32(a, false) for the second refill word, added to RCycles:
//   a < ITCMSize ? 1 : raw == 0xFF ? (a & 0x1F ? 1 : kCodeCacheTiming) : raw
// in: W0 = a, W1 = raw (MemTimings[target >> 12][0]), W2 = ITCMSize; clobbers W3, flags.
void Compiler::Gen_JumpTo9SeqFetch()
{
    MOVI2R(W3, kCodeCacheTiming);
    TSTI2R(W0, 0x1F);
    CSINC(W3, W3, WZR, CC_EQ);          // line start ? 3 : 1
    CMP(W1, 0xFF);
    CSEL(W3, W3, W1, CC_EQ);            // cached ? that : raw
    CMP(W0, W2);
    CSINC(W3, W3, WZR, CC_HS);          // ITCM ? 1
    ADD(RCycles, RCycles, W3);
}

// Indirect jump refill, charged exactly as ARMv5::JumpTo (two CodeRead32 for ARM and for an
// unaligned Thumb target, one for an aligned Thumb target; ARMv5::CodeRead32's fetch cost each).
// The upstream stub read MemTimings without the x4 row scaling (a different page's byte), charged
// the branch-fetch cost twice for ARM (6 instead of 3 + 1 in cached memory) and the Thumb x2 on
// the aligned instead of the unaligned target.
void* Compiler::Gen_JumpTo9(int kind)
{
    AlignCode16();
    void* res = GetRXPtr();
    REFILL_ENTRY(kind);

    // W1 = raw = MemTimings[addr >> 12][0]
#ifdef LITEV_JIT_REFILL_FAST
    // X1 = RCPU + (addr >> 12) * 4 in 2 instructions (bits 63:32 of X1 are zero after the W write).
    {
        constexpr u32 mt = offsetof(ARMv5, MemTimings);
        static_assert(mt < 0x1000000, "MemTimings offset must fit ADD imm12 lsl 12 + LDRB imm12");
        ANDI2R(W1, W0, 0xFFFFF000);
        ADD(X1, RCPU, X1, ArithOption(X1, ST_LSR, 10));
        ADD(X1, X1, mt >> 12, true);
        LDRB(INDEX_UNSIGNED, W1, X1, mt & 0xFFF);
    }
#else
    LSR(W1, W0, 12);
    LSL(W1, W1, 2);
    ADDI2R(W1, W1, offsetof(ARMv5, MemTimings), W2);
    LDRB(W1, RCPU, W1);
#endif

    LDR(INDEX_UNSIGNED, W2, RCPU, offsetof(ARMv5, ITCMSize));

    STR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARMv5, RegionCodeCycles));

    // first word (CodeRead32(addr & ~3, true)): addr < ITCMSize ? 1 : raw == 0xFF ? 3 : raw
    // (ITCMSize is a multiple of 512, so the low address bits do not change the compare)
#ifdef LITEV_FAST_TIMING
    if (LITEV_FT)   // fast timing: the refill is a constant, no fetch-cost lookup
        ADD(RCycles, RCycles, FastTiming.R9);
    else
#endif
    {
    CMP(W1, 0xFF);
    MOVI2R(W3, kCodeCacheTiming);
    CSEL(W3, W3, W1, CC_EQ);
    CMP(W0, W2);
    CSINC(W3, W3, WZR, CC_HS);
    ADD(RCycles, RCycles, W3);
    }

    FixupBranch switchToThumb;
    if (kind == 0)
        switchToThumb = TBNZ(W0, 0);

    if (kind == 0 || kind == 1)
    {
        // ARM
        if (kind == 0)
#ifdef LITEV_JIT_LAZYFLAGS
        {
            // Clear guest T-bit in ARM::CPSR memory. W3 is dead here (the first-word cost is
            // added); the caller (Comp_JumpTo reg) already materialized host NZCV so the word is
            // canonical and only the T-bit changes.
            LDR(INDEX_UNSIGNED, W3, RCPU, offsetof(ARM, CPSR));
            ANDI2R(W3, W3, ~0x20);
            STR(INDEX_UNSIGNED, W3, RCPU, offsetof(ARM, CPSR));
        }
#else
            ANDI2R(RCPSR, RCPSR, ~0x20);
#endif

        ANDI2R(W0, W0, ~3);
        ADD(W0, W0, 4);
        STR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARMv5, R[15]));

        // second word: CodeRead32(addr + 4, false), W0 == addr + 4
#ifdef LITEV_FAST_TIMING
        if (!LITEV_FT)
#endif
        Gen_JumpTo9SeqFetch();
        REFILL_CALL(RefillCheck9); RET();
    }

    if (kind == 0 || kind == 2)
    {
        // Thumb
        if (kind == 0)
        {
            SetJumpTarget(switchToThumb);
#ifdef LITEV_JIT_LAZYFLAGS
            LDR(INDEX_UNSIGNED, W3, RCPU, offsetof(ARM, CPSR));
            ORRI2R(W3, W3, 0x20);
            STR(INDEX_UNSIGNED, W3, RCPU, offsetof(ARM, CPSR));
#else
            ORRI2R(RCPSR, RCPSR, 0x20);
#endif
        }

        ANDI2R(W0, W0, ~1);
        ADD(W0, W0, 2);
        STR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARMv5, R[15]));

        // an unaligned target (addr & 2) also fetches the next word: CodeRead32(addr + 2, false).
        // W0 == addr + 2 here, so it is unaligned exactly when W0 & 2 is clear, and then W0 is
        // that word's address.
#ifdef LITEV_FAST_TIMING
        if (!LITEV_FT)
#endif
        {
        FixupBranch aligned = TBNZ(W0, 1);
        Gen_JumpTo9SeqFetch();
        SetJumpTarget(aligned);
        }
        REFILL_CALL(RefillCheck9); RET();
    }

    return res;
}
void* Compiler::Gen_JumpTo7(int kind)
{
    void* res = GetRXPtr();
    REFILL_ENTRY(kind);

    LSR(W1, W0, 24);
    STR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, CodeRegion));
    LSR(W1, W0, 15);
    STR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, CodeCycles));

#ifdef LITEV_JIT_REFILL_FAST
    // ARM7 blocks run with RCPU == &NDS.ARM7, and the table is a member of the same NDS object:
    // address it from RCPU (ADD+ADD) instead of a 64-bit pointer constant (MOVZ+MOVK+MOVK).
    const s64 a7mt = (s64)((u8*)NDS.ARM7MemTimings - (u8*)&NDS.ARM7);
    if (a7mt > -0x1000000 && a7mt < 0x1000000)
        ADDI2R(X2, RCPU, (u64)a7mt);
    else
#endif
    MOVP2R(X2, NDS.ARM7MemTimings);
    LDR(W3, X2, ArithOption(W1, true));

    FixupBranch switchToThumb;
    if (kind == 0)
        switchToThumb = TBNZ(W0, 0);
    
    if (kind == 0 || kind == 1)
    {
#ifdef LITEV_FAST_TIMING
        if (LITEV_FT) ADD(RCycles, RCycles, FastTiming.R7);
        else
#endif
        {
        // ARMv4::JumpTo: ARM refill = N32 + S32 (bytes 2 and 3; upstream added the 16-bit pair)
        UBFX(W2, W3, 16, 8);
#ifdef LITEV_JIT_REFILL_FAST
        ADD(W2, W2, W3, ArithOption(W3, ST_LSR, 24));   // byte 2 + byte 3
#else
        UBFX(W3, W3, 24, 8);
        ADD(W2, W3, W2);
#endif
        ADD(RCycles, RCycles, W2);
        }

        ANDI2R(W0, W0, ~3);

        if (kind == 0)
#ifdef LITEV_JIT_LAZYFLAGS
        {
            // Clear guest T-bit in ARM::CPSR memory (W2 is dead here).
            LDR(INDEX_UNSIGNED, W2, RCPU, offsetof(ARM, CPSR));
            ANDI2R(W2, W2, ~0x20);
            STR(INDEX_UNSIGNED, W2, RCPU, offsetof(ARM, CPSR));
        }
#else
            ANDI2R(RCPSR, RCPSR, ~0x20);
#endif

        ADD(W3, W0, 4);
        STR(INDEX_UNSIGNED, W3, RCPU, offsetof(ARM, R[15]));

        REFILL_CALL(RefillCheck7); RET();
    }
    if (kind == 0 || kind == 2)
    {
        if (kind == 0)
        {
            SetJumpTarget(switchToThumb);

#ifdef LITEV_JIT_LAZYFLAGS
            // Set guest T-bit in ARM::CPSR memory (W2 free; W3 holds the live memtimings word).
            LDR(INDEX_UNSIGNED, W2, RCPU, offsetof(ARM, CPSR));
            ORRI2R(W2, W2, 0x20);
            STR(INDEX_UNSIGNED, W2, RCPU, offsetof(ARM, CPSR));
#else
            ORRI2R(RCPSR, RCPSR, 0x20);
#endif
        }

#ifdef LITEV_FAST_TIMING
        if (LITEV_FT) ADD(RCycles, RCycles, FastTiming.R7);
        else
#endif
        {
        // ARMv4::JumpTo: Thumb refill = N16 + S16 (bytes 0 and 1; upstream added the 32-bit pair)
        UBFX(W2, W3, 0, 8);
        UBFX(W3, W3, 8, 8);
        ADD(W2, W3, W2);
        ADD(RCycles, RCycles, W2);
        }

        ANDI2R(W0, W0, ~1);

        ADD(W3, W0, 2);
        STR(INDEX_UNSIGNED, W3, RCPU, offsetof(ARM, R[15]));

        REFILL_CALL(RefillCheck7); RET();
    }

    return res;
}

void Compiler::Comp_JumpTo(Arm64Gen::ARM64Reg addr, bool switchThumb, bool restoreCPSR)
{
#ifdef LITEV_JIT_FIXEDREG
    // A same-instruction branch (Rd=PC ALU op) can follow an unconditional deferring
    // producer; reconcile before the branch stub BL clobbers host NZCV / before the
    // CPSR is saved-restored.
    Comp_MaterializeFlags();
#endif
#ifdef LITEV_JIT_RAS
    Comp_RasCallRet(true, restoreCPSR);
#endif
    IrregularCycles = true;

    if (!restoreCPSR)
    {
        if (switchThumb)
            CPSRDirty = true;
#ifdef LITEV_JIT_MOV_ELIDE
        if (addr != W0 || !JitQOn(jitq_MovElide))   // `mov w0, w0` when the caller computed it there
#endif
        MOV(W0, addr);
        BL((Num ? JumpToFuncs7 : JumpToFuncs9)[switchThumb ? 0 : (Thumb + 1)]);
    }
    else
    {
        
        bool cpsrDirty = CPSRDirty;
        SaveCPSR();
        SaveCycles();
        PushRegs(restoreCPSR, true);

        if (switchThumb)
            MOV(W1, addr);
        else
        {
            if (Thumb)
                ORRI2R(W1, addr, 1);
            else
                ANDI2R(W1, addr, ~1);
        }
        MOV(X0, RCPU);
        MOVI2R(W2, restoreCPSR);
        if (Num == 0)
            QuickCallFunction(X3, JumpToTrampoline<ARMv5>);
        else
            QuickCallFunction(X3, JumpToTrampoline<ARMv4>);

        PopRegs(restoreCPSR, true);
        LoadCycles();
        LoadCPSR();
        if (CurInstr.Cond() < 0xE)
            CPSRDirty = cpsrDirty;
    }
}

#ifdef LITEV_JIT_RAS
// Guest call: push this call site's own ICACHE site (its return lands on one address,
// so the entry is monomorphic). Guest return (a non-linking branch through LR or a
// load via SP, not an exception return): pop into ARM::RasSite, which the block exit
// hands to the dispatcher as W9. Emitted on the taken path only; W16/W17 are scratch
// (never allocated, and the branch target register is never one of them).
void Compiler::Comp_RasCallRet(bool regTarget, bool restoreCPSR)
{
    if (!RasOn || restoreCPSR)
        return;
    u16 src = CurInstr.Info.SrcRegs, dst = CurInstr.Info.DstRegs;
    bool call = (dst & (1 << 14)) && !(src & (1 << 13));
    bool ret = regTarget && !call && (src & ((1 << 13) | (1 << 14)));
    if (call)
    {
        u32 site = ICacheAssignSite();
        if (!site)
            return;
        LDR(INDEX_UNSIGNED, W16, RCPU, offsetof(ARM, RasTop));
        ADD(W16, W16, 1);
        ANDI2R(W16, W16, ARM::RasMask);
        STR(INDEX_UNSIGNED, W16, RCPU, offsetof(ARM, RasTop));
        ADD(X16, RCPU, X16, ArithOption(X16, ST_LSL, 2));
        MOVI2R(W17, site);
        STR(INDEX_UNSIGNED, W17, X16, offsetof(ARM, RasRing));
    }
    else if (ret)
    {
        LDR(INDEX_UNSIGNED, W16, RCPU, offsetof(ARM, RasTop));
        ADD(X17, RCPU, X16, ArithOption(X16, ST_LSL, 2));
        LDR(INDEX_UNSIGNED, W17, X17, offsetof(ARM, RasRing));
        STR(INDEX_UNSIGNED, W17, RCPU, offsetof(ARM, RasSite));
        SUB(W16, W16, 1);
        ANDI2R(W16, W16, ARM::RasMask);
        STR(INDEX_UNSIGNED, W16, RCPU, offsetof(ARM, RasTop));
        RasRetBlock = true;
    }
}
#endif

void Compiler::A_Comp_BranchImm()
{
    int op = (CurInstr.Instr >> 24) & 1;
    s32 offset = (s32)(CurInstr.Instr << 8) >> 6;
    u32 target = R15 + offset;
    bool link = op;

    if (CurInstr.Cond() == 0xF) // BLX_imm
    {
        target += (op << 1) + 1;
        link = true;
    }

    if (link)
        MOVI2R(MapReg(14), R15 - 4);

    Comp_JumpTo(target);
}

void Compiler::A_Comp_BranchXchangeReg()
{
    ARM64Reg rn = MapReg(CurInstr.A_Reg(0));
    MOV(W0, rn);
    if ((CurInstr.Instr & 0xF0) == 0x30) // BLX_reg
        MOVI2R(MapReg(14), R15 - 4);
    Comp_JumpTo(W0, true);
}

void Compiler::T_Comp_BCOND()
{
    u32 cond = (CurInstr.Instr >> 8) & 0xF;
#ifdef LITEV_JIT_COLD_EXITS
    if (ColdExitOK())
    {
        // see Compiler::ColdExit; mirrors the in-place sequence below edge by edge
        if (CurInstr.BranchFlags & branch_FollowCondNotTaken)
        {
            DeferColdExit(CheckCondition(cond ^ 1), 2);
            IrregularCycles = true;   // what the deferred Comp_JumpTo sets
            Comp_AddCycles_C(true);
        }
        else
        {
            FixupBranch notTaken = CheckCondition(cond);
            s32 offset = (s32)(CurInstr.Instr << 24) >> 23;
            Comp_JumpTo(R15 + offset + 1, true);
            Comp_BranchSpecialBehaviour(true);
            Comp_AddCycles_C(true);
            DeferColdExit(notTaken, 1);
  #ifdef LITEV_EXIT_PROTO_PC
            PCElided = false;
  #endif
        }
        return;
    }
#endif
    FixupBranch skipExecute = CheckCondition(cond);

    s32 offset = (s32)(CurInstr.Instr << 24) >> 23;
    Comp_JumpTo(R15 + offset + 1, true);

    Comp_BranchSpecialBehaviour(true);

    FixupBranch skipFailed = B();
    SetJumpTarget(skipExecute);
    Comp_AddCycles_C(true);

    Comp_BranchSpecialBehaviour(false);

    SetJumpTarget(skipFailed);
}

void Compiler::T_Comp_B()
{
    s32 offset = (s32)((CurInstr.Instr & 0x7FF) << 21) >> 20;
    Comp_JumpTo(R15 + offset + 1);
}

void Compiler::T_Comp_BranchXchangeReg()
{
    bool link = CurInstr.Instr & (1 << 7);

    if (link)
    {
        if (Num == 1)
        {
            Log(LogLevel::Warn, "BLX unsupported on ARM7!!!\n");
            return;
        }
        MOV(W0, MapReg(CurInstr.A_Reg(3)));
        MOVI2R(MapReg(14), R15 - 1);
        Comp_JumpTo(W0, true);
    }
    else
    {
        ARM64Reg rn = MapReg(CurInstr.A_Reg(3));
        Comp_JumpTo(rn, true);
    }
}

void Compiler::T_Comp_BL_LONG_1()
{
    s32 offset = (s32)((CurInstr.Instr & 0x7FF) << 21) >> 9;
    MOVI2R(MapReg(14), R15 + offset);
    Comp_AddCycles_C();
}

void Compiler::T_Comp_BL_LONG_2()
{
    ARM64Reg lr = MapReg(14);
    s32 offset = (CurInstr.Instr & 0x7FF) << 1;
    ADD(W0, lr, offset);
    MOVI2R(lr, (R15 - 2) | 1);
    Comp_JumpTo(W0, Num == 0 && !(CurInstr.Instr & (1 << 12)));
}

void Compiler::T_Comp_BL_Merged()
{
    Comp_AddCycles_C();

    R15 += 2;

    u32 upperPart = CurInstr.Instr >> 16;
    u32 target = (R15 - 2) + ((s32)((CurInstr.Instr & 0x7FF) << 21) >> 9);
    target += (upperPart & 0x7FF) << 1;

    if (Num == 1 || upperPart & (1 << 12))
        target |= 1;

    MOVI2R(MapReg(14), (R15 - 2) | 1);
    
    Comp_JumpTo(target);
}

}
