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

#include "../ARMJIT.h"

#include "../ARMJIT_Memory.h"
#include "../NDS.h"


#if defined(LITEV_JIT_STORE_REPROMOTE) || defined(LITEV_JIT_COND_MEMGUESS) || defined(LITEV_JIT_USERSTM_FASTMEM)
#if defined(__ANDROID__)
#include <sys/system_properties.h>
#endif
#include <stdlib.h>
#include <algorithm>
#endif

using namespace Arm64Gen;

#ifdef LITEV_SLOWMEM_HIST
namespace melonDS { void LitevSlowSiteNote(const void* ret, u32 info); }
#define NOTE_SLOW_SITE(info) LitevSlowSiteNote((const u8*)GetRXPtr() - 4, (info))
#else
#define NOTE_SLOW_SITE(info) ((void)0)
#endif

namespace melonDS
{

#ifdef LITEV_JIT_STORE_REPROMOTE
u64 JitStoreRepromotions = 0;   // diagnosis counter (headless LITEV_FRAME_MS)

static bool StoreRepromoteOn()   // debug.litev.storerepromote (default on), env LITEV_STOREREPROMOTE off the device
{
    static const bool on = [] {
#if defined(__ANDROID__)
        char b[8] = {0};
        return !(__system_property_get("debug.litev.storerepromote", b) > 0 && atoi(b) == 0);
#else
        const char* e = getenv("LITEV_STOREREPROMOTE");
        return !(e && atoi(e) == 0);
#endif
    }();
    return on;
}

// Called by the fault handler for a fault on a code-protected page, before RewriteMemAccess.
void Compiler::NoteProtectFault(u8* pc)
{
    if (!StoreRepromoteOn())
        return;
    auto it = LoadStorePatches.find(pc - GetRXBase());
    if (it == LoadStorePatches.end())
        return;   // RewriteMemAccess reports the JIT bug
    ptrdiff_t start = (pc - GetRXBase()) + it->second.PatchOffset;
    u32 frame = NDS.NumFrames;
    SlowStoreSite& site = SlowStoreSites[start];
    if (site.Orig.empty())
    {
        const u32* code = (const u32*)(GetRXBase() + start);
        site.Orig.assign(code, code + it->second.PatchSize / 4);
        site.Faults = 0;
    }
    else if (frame - site.RestoredFrame > 600)
        site.Faults = 0;   // it ran fast for 10 s since the last retry: start the backoff over
    else if (site.Faults < 10)
        site.Faults++;
    site.Slow = true;
    site.RetryFrame = frame + (4u << site.Faults);
    SlowStoreNextRetry = std::min(SlowStoreNextRetry, site.RetryFrame);
}

// Called at the end of NDS::RunFrame, where no JIT code is executing. Puts the original
// fast-path bytes back, i.e. the exact code the compiler emitted, a state every other
// fastmem site is in: a store to a page that holds code faults and is rewritten again.
void Compiler::RepromoteStores()
{
    u32 frame = NDS.NumFrames;
    if (frame < SlowStoreNextRetry)
        return;
    u32 next = ~0u;
    ptrdiff_t cur = GetCodeOffset();
    NDS.JIT.JitEnableWrite();
    for (auto& [start, site] : SlowStoreSites)
    {
        if (!site.Slow)
            continue;
        if (frame < site.RetryFrame)
        {
            next = std::min(next, site.RetryFrame);
            continue;
        }
        SetCodePtrUnsafe(start);
        for (u32 w : site.Orig)
            Write32(w);
        FlushIcacheSection(GetRXBase() + start, (u8*)GetRXPtr());
        site.Slow = false;
        site.RestoredFrame = frame;
        JitStoreRepromotions++;
    }
    SetCodePtrUnsafe(cur);
    NDS.JIT.JitEnableExecute();
    SlowStoreNextRetry = next;
}
#endif

bool Compiler::IsJITFault(const u8* pc)
{
    return (u64)pc >= (u64)GetRXBase() && (u64)pc - (u64)GetRXBase() < (JitMemMainSize + JitMemSecondarySize);
}

u8* Compiler::RewriteMemAccess(u8* pc)
{
    ptrdiff_t pcOffset = pc - GetRXBase();

    auto it = LoadStorePatches.find(pcOffset);

    if (it != LoadStorePatches.end())
    {
        LoadStorePatch patch = it->second;
#ifdef LITEV_JIT_STORE_REPROMOTE
        // kept: the site may be restored to this fast path later (the rewritten code has no
        // memory access at this offset, so the stale entry can't be looked up meanwhile)
        if (!StoreRepromoteOn())
#endif
        LoadStorePatches.erase(it);

        ptrdiff_t curCodeOffset = GetCodeOffset();

        SetCodePtrUnsafe(pcOffset + patch.PatchOffset);

        BL(patch.PatchFunc);
        for (int i = 0; i < patch.PatchSize / 4 - 1; i++)
            HINT(HINT_NOP);
        FlushIcacheSection((u8*)pc + patch.PatchOffset, (u8*)GetRXPtr());

        SetCodePtrUnsafe(curCodeOffset);

        return pc + (ptrdiff_t)patch.PatchOffset;
    }
    Log(LogLevel::Error, "this is a JIT bug! %08x\n", __builtin_bswap32(*(u32*)pc));
    abort();
}

// A conditional ARM access normally takes the fastmem path whatever its region looked like,
// since a condition that failed while compiling left DataRegion stale. When it did run, its
// region is known: an IO/VRAM access then goes straight to the slow path instead of faulting
// once (a signal + rewrite, ~50-80 us on the A55). Fast or slow path is guest-invisible.
bool Compiler::CondMemGuess(bool addrIsStatic)
{
#ifdef LITEV_JIT_COND_MEMGUESS
    static const bool on = [] {
#if defined(__ANDROID__)
        char b[8] = {0};
        return !(__system_property_get("debug.litev.condmemguess", b) > 0 && atoi(b) == 0);
#else
        const char* e = getenv("LITEV_CONDMEMGUESS");
        return !(e && atoi(e) == 0);
#endif
    }();
    return on && (addrIsStatic || CurInstr.DataExecuted);
#else
    return false;
#endif
}

#ifdef LITEV_JIT_USERSTM_FASTMEM
// User-bank STM (STM ... ^, e.g. NitroSDK's thread context save STMIB r0, {r2-r14}^, ~200 per
// frame in the PW overworld) always took the slow helper. It now takes the fastmem path like any
// other STM: same words to the same addresses, banked registers read through ReadBanked exactly
// as the slow path does. Guest-invisible.
static bool UserStmFastOn()   // debug.litev.userstmfast (default on), env LITEV_USERSTMFAST off the device
{
    static const bool on = [] {
#if defined(__ANDROID__)
        char b[8] = {0};
        return !(__system_property_get("debug.litev.userstmfast", b) > 0 && atoi(b) == 0);
#else
        const char* e = getenv("LITEV_USERSTMFAST");
        return !(e && atoi(e) == 0);
#endif
    }();
    return on;
}
#endif

bool Compiler::Comp_MemLoadLiteral(int size, bool signExtend, int rd, u32 addr)
{
    u32 localAddr = NDS.JIT.LocaliseCodeAddress(Num, addr);

    int invalidLiteralIdx = NDS.JIT.InvalidLiterals.Find(localAddr);
    if (invalidLiteralIdx != -1)
    {
        return false;
    }

    Comp_AddCycles_CDI();

    u32 val;
    // make sure arm7 bios is accessible
    u32 tmpR15 = CurCPU->R[15];
    CurCPU->R[15] = R15;
    if (size == 32)
    {
        CurCPU->DataRead32(addr & ~0x3, &val);
        val = melonDS::ROR(val, (addr & 0x3) << 3);
    }
    else if (size == 16)
    {
        CurCPU->DataRead16(addr & ~0x1, &val);
        if (signExtend)
            val = ((s32)val << 16) >> 16;
    }
    else
    {
        CurCPU->DataRead8(addr, &val);
        if (signExtend)
            val = ((s32)val << 24) >> 24;
    }
    CurCPU->R[15] = tmpR15;

    MOVI2R(MapReg(rd), val);

    if (Thumb || CurInstr.Cond() == 0xE)
        RegCache.PutLiteral(rd, val);
    
    return true;
}

void Compiler::Comp_MemAccess(int rd, int rn, Op2 offset, int size, int flags)
{
    u32 addressMask = ~0;
    if (size == 32)
        addressMask = ~3;
    if (size == 16)
        addressMask = ~1;

    if (NDS.JIT.LiteralOptimizationsEnabled() && rn == 15 && rd != 15 && offset.IsImm && !(flags & (memop_Post|memop_Store|memop_Writeback)))
    {
        u32 addr = R15 + offset.Imm * ((flags & memop_SubtractOffset) ? -1 : 1);
        
        if (Comp_MemLoadLiteral(size, flags & memop_SignExtend, rd, addr))
            return;
    }
    
    if (flags & memop_Store)
        Comp_AddCycles_CD();
    else
        Comp_AddCycles_CDI();

    ARM64Reg rdMapped = MapReg(rd);
    ARM64Reg rnMapped = MapReg(rn);

    if (Thumb && rn == 15)
    {
        ANDI2R(W3, rnMapped, ~2);
        rnMapped = W3;
    }

    if (flags & memop_Store && flags & (memop_Post|memop_Writeback) && rd == rn)
    {
        MOV(W4, rdMapped);
        rdMapped = W4;
    }

    ARM64Reg finalAddr = W0;
    if (flags & memop_Post)
    {
        finalAddr = rnMapped;
        MOV(W0, rnMapped);
    }

    bool addrIsStatic = NDS.JIT.LiteralOptimizationsEnabled()
        && RegCache.IsLiteral(rn) && offset.IsImm && !(flags & (memop_Writeback|memop_Post));
    u32 staticAddress;
    if (addrIsStatic)
        staticAddress = RegCache.LiteralValues[rn] + offset.Imm * ((flags & memop_SubtractOffset) ? -1 : 1);

    if (!offset.IsImm)
        Comp_RegShiftImm(offset.Reg.ShiftType, offset.Reg.ShiftAmount, false, offset, W2);
    // offset might has become an immediate
    if (offset.IsImm)
    {
        if (offset.Imm)
        {
            if (flags & memop_SubtractOffset)
                SUB(finalAddr, rnMapped, offset.Imm);
            else
                ADD(finalAddr, rnMapped, offset.Imm);
        }
        else if (finalAddr != rnMapped)
            MOV(finalAddr, rnMapped);
    }
    else
    {
        if (offset.Reg.ShiftType == ST_ROR)
        {
            ROR(W0, offset.Reg.Rm, offset.Reg.ShiftAmount);
            offset = Op2(W0);
        }

        if (flags & memop_SubtractOffset)
            SUB(finalAddr, rnMapped, offset.Reg.Rm, offset.ToArithOption());
        else
            ADD(finalAddr, rnMapped, offset.Reg.Rm, offset.ToArithOption());
    }

    if (!(flags & memop_Post) && (flags & memop_Writeback))
        MOV(rnMapped, W0);

    u32 expectedTarget = Num == 0
        ? NDS.JIT.Memory.ClassifyAddress9(addrIsStatic ? staticAddress : CurInstr.DataRegion)
        : NDS.JIT.Memory.ClassifyAddress7(addrIsStatic ? staticAddress : CurInstr.DataRegion);

    if (NDS.JIT.FastMemoryEnabled() && ((!Thumb && CurInstr.Cond() != 0xE && !CondMemGuess(addrIsStatic)) || NDS.JIT.Memory.IsFastmemCompatible(expectedTarget)))
    {
        ptrdiff_t memopStart = GetCodeOffset();
        LoadStorePatch patch;

        assert((rdMapped >= W8 && rdMapped <= W15) || (rdMapped >= W19 && rdMapped <= W25) || rdMapped == W4
#ifdef LITEV_JIT_LAZYFLAGS
            || rdMapped == W27   // guest r7 pin under full lazy-flags (patched funcs generated for W27)
#endif
        );
        patch.PatchFunc = flags & memop_Store
            ? PatchedStoreFuncs[NDS.ConsoleType][Num][__builtin_ctz(size) - 3][rdMapped]
            : PatchedLoadFuncs[NDS.ConsoleType][Num][__builtin_ctz(size) - 3][!!(flags & memop_SignExtend)][rdMapped];

#ifdef LITEV_JIT_LDR_ALIGNCHK
        // Word / halfword LOAD: test the alignment and load from the address itself, instead of
        // masking it (and rotating a word afterwards). An aligned access, the normal case, then
        // has no ALU op between the address and the load nor between the load and its user
        // (2 cycles less on the A55's load chain); a misaligned one runs the slow-path thunk
        // from the block tail, which rotates / masks exactly as the fastmem sequence did.
        if (!(flags & memop_Store) && size > 8 && !(size == 32 && addrIsStatic) && JitQOn(jitq_LdrAlignChk))
        {
            FixupBranch misaligned;
            if (size == 32)
            {
                ANDI2R(W1, W0, 3);
                misaligned = CBNZ(W1);
            }
            else
                misaligned = TBNZ(W0, 0);
            ptrdiff_t loadPosition = GetCodeOffset();
            LDRGeneric(size, flags & memop_SignExtend, rdMapped, X0, RMemBase);
            patch.PatchOffset = memopStart - loadPosition;
            patch.PatchSize = GetCodeOffset() - memopStart;
            LoadStorePatches[loadPosition] = patch;
            TailStub t{};
            t.Kind = 1;
            t.A = misaligned;
            t.Func = patch.PatchFunc;
            t.Back = (const u8*)GetRXPtr();
            TailStubs.push_back(t);
        }
        else
#endif
        {
        // take a chance at fastmem
        if (size > 8)
            ANDI2R(W1, W0, addressMask);

        ptrdiff_t loadStorePosition = GetCodeOffset();
        if (flags & memop_Store)
        {
            STRGeneric(size, rdMapped, size > 8 ? X1 : X0, RMemBase);
        }
        else
        {
            LDRGeneric(size, flags & memop_SignExtend, rdMapped, size > 8 ? X1 : X0, RMemBase);
            if (size == 32 && !addrIsStatic)
            {
                UBFIZ(W0, W0, 3, 2);
                RORV(rdMapped, rdMapped, W0);
            }
        }

        patch.PatchOffset = memopStart - loadStorePosition;
        patch.PatchSize = GetCodeOffset() - memopStart;
        LoadStorePatches[loadStorePosition] = patch;
        }
    }
    else
    {
        void* func = NULL;
        if (addrIsStatic)
            func = NDS.JIT.Memory.GetFuncForAddr(CurCPU, staticAddress, flags & memop_Store, size);

        // ---- M3 Tier A: MainRAM-hit inline single u32 load ------------------
        // Only for dynamic-address u32 ARM9 loads whose decoded data region is
        // MainRAM (the SlowRead9<u32> dominant case). DS only, region 0x02000000.
        // Runtime guards exclude DTCM (which can be based inside the 0x02 range)
        // and any non-MainRAM target, falling back to the exact helper. Stores
        // are deliberately NOT accelerated (no MainRAM raw-store thunk: preserves
        // JIT invalidation semantics, per plan constraint). MainRAM base/mask are
        // baked at compile time (stable for the block cache's lifetime; a reset
        // reallocates and recompiles). Cycles already added by Comp_AddCycles_CDI.
#ifdef LITEV_MEM_MAINRAM_LOAD
        const bool mainramFast = (Num == 0) && (NDS.ConsoleType == 0) && !func
            && (size == 32) && !(flags & memop_Store)
            && (expectedTarget == ARMJIT_Memory::memregion_MainRAM);
#else
        const bool mainramFast = false;
#endif
#ifdef LITEV_MEM_MAINRAM_LOAD
        FixupBranch mainramDone;
        if (mainramFast)
        {
            FixupBranch mrMiss[2];
            // W0 = access address (low 2 bits carry the u32 rotate). W4 = addr&~3.
            ANDI2R(W4, W0, ~3);
            // exclude DTCM (SlowRead9 checks DTCM before the region switch).
            LDR(INDEX_UNSIGNED, W5, RCPU, offsetof(ARMv5, DTCMBase));
            LDR(INDEX_UNSIGNED, W6, RCPU, offsetof(ARMv5, DTCMMask));
            AND(W6, W4, W6);
            CMP(W6, W5);
            mrMiss[0] = B(CC_EQ);
            // require MainRAM region (DS: (addr & 0xFF000000) == 0x02000000).
            ANDI2R(W5, W4, 0xFF000000);
            MOVI2R(W6, 0x02000000);
            CMP(W5, W6);
            mrMiss[1] = B(CC_NEQ);
            // hit: rd = MainRAM[(addr&~3) & MainRAMMask], then ROR by (addr&3)*8.
            ANDI2R(W4, W4, NDS.MainRAMMask);
            MOVP2R(X7, NDS.MainRAM);
            LDRGeneric(32, false, rdMapped, X4, X7);
            UBFIZ(W0, W0, 3, 2);
            RORV(rdMapped, rdMapped, W0);
            mainramDone = B();
            SetJumpTarget(mrMiss[0]);
            SetJumpTarget(mrMiss[1]);
        }
#endif

#ifdef LITEV_JIT_VWRAM_LOAD
        // ARM7 load from VRAM mapped as ARM7 memory (0x06000000-0x06FFFFFF): read the slot's one
        // bank directly (GPU::VRAMPtr_ARM7) instead of SlowRead7 -> ARM7Read -> ReadVRAM_ARM7.
        // Same value: an unmapped slot or both banks (ORed reads) take the helper. Stores keep the
        // helper (JIT invalidation of code in VRAM).
        const bool vwramFast = (Num == 1) && (NDS.ConsoleType == 0) && !func && !(flags & memop_Store)
            && (expectedTarget == ARMJIT_Memory::memregion_VWRAM);
        FixupBranch vwramDone;
        if (vwramFast)
        {
            FixupBranch vwMiss[2];
            // (scratch: W1-W3 only; W4-W7 can hold guest registers on the ARM7)
            ANDI2R(W1, W0, 0xFF000000);
            MOVI2R(W2, 0x06000000);
            CMP(W1, W2);
            vwMiss[0] = B(CC_NEQ);
            UBFX(W1, W0, 17, 1);
            MOVP2R(X2, &NDS.GPU.VRAMPtr_ARM7[0]);
            LDR(X2, X2, ArithOption(X1, true));
            vwMiss[1] = CBZ(X2);
            ANDI2R(W3, W0, 0x1FFFF & ~(u32)(size / 8 - 1));
            LDRGeneric(size, flags & memop_SignExtend, rdMapped, X3, X2);
            if (size == 32)
            {
                UBFIZ(W0, W0, 3, 2);
                RORV(rdMapped, rdMapped, W0);
            }
            vwramDone = B();
            SetJumpTarget(vwMiss[0]);
            SetJumpTarget(vwMiss[1]);
        }
#endif

#ifdef LITEV_JIT_VWRAM_LOAD
        // (no register unloading: the fast path joins after PopRegs with the registers still loaded)
        PushRegs(false, false, !mainramFast && !vwramFast);
#else
        PushRegs(false, false, !mainramFast);
#endif

        if (func)
        {
            // LITEV dTLB: region helpers now take the live CPU pointer (2nd arg) and use
            // cpu->NDS instead of resolving thread_local NDS::Current via tlsdesc on every
            // call. Mirror the SlowWrite9 arg layout: W0=addr (already set), X1=cpu, W2=val.
            MOV(X1, RCPU);
            if (flags & memop_Store)
                MOV(W2, rdMapped);
            QuickCallFunction(X3, (void (*)())func);

            PopRegs(false, false);

            if (!(flags & memop_Store))
            {
                if (size == 32)
                {
                    if (staticAddress & 0x3)
                        ROR(rdMapped, W0, (staticAddress & 0x3) << 3);
                    else
                        MOV(rdMapped, W0);
                }
                else
                {
                    if (flags & memop_SignExtend)
                        SBFX(rdMapped, W0, 0, size);
                    else
                        UBFX(rdMapped, W0, 0, size);
                }
            }
        }
        else
        {
            if (Num == 0)
            {
                MOV(X1, RCPU);
                if (flags & memop_Store)
                {
                    MOV(W2, rdMapped);
                    switch (size | NDS.ConsoleType)
                    {
                    case 32: QuickCallFunction(X3, SlowWrite9<u32, 0>); break;
                    case 33: QuickCallFunction(X3, SlowWrite9<u32, 1>); break;
                    case 16: QuickCallFunction(X3, SlowWrite9<u16, 0>); break;
                    case 17: QuickCallFunction(X3, SlowWrite9<u16, 1>); break;
                    case 8: QuickCallFunction(X3, SlowWrite9<u8, 0>); break;
                    case 9: QuickCallFunction(X3, SlowWrite9<u8, 1>); break;
                    }
                    NOTE_SLOW_SITE(expectedTarget | (CurInstr.Cond() << 8) | (CurInstr.DataExecuted << 12) | ((u32)Thumb << 13) | ((u32)(size >> 3) << 16));
                }
                else
                {
                    switch (size | NDS.ConsoleType)
                    {
                    case 32: QuickCallFunction(X3, SlowRead9<u32, 0>); break;
                    case 33: QuickCallFunction(X3, SlowRead9<u32, 1>); break;
                    case 16: QuickCallFunction(X3, SlowRead9<u16, 0>); break;
                    case 17: QuickCallFunction(X3, SlowRead9<u16, 1>); break;
                    case 8: QuickCallFunction(X3, SlowRead9<u8, 0>); break;
                    case 9: QuickCallFunction(X3, SlowRead9<u8, 1>); break;
                    }
                }
            }
            else
            {
#ifdef LITEV_JIT_ARM7_CPUARG
                MOV(X1, RCPU);
#endif
                if (flags & memop_Store)
                {
#ifdef LITEV_JIT_ARM7_CPUARG
                    MOV(W2, rdMapped);
#else
                    MOV(W1, rdMapped);
#endif
                    switch (size | NDS.ConsoleType)
                    {
                    case 32: QuickCallFunction(X3, SlowWrite7<u32, 0>); break;
                    case 33: QuickCallFunction(X3, SlowWrite7<u32, 1>); break;
                    case 16: QuickCallFunction(X3, SlowWrite7<u16, 0>); break;
                    case 17: QuickCallFunction(X3, SlowWrite7<u16, 1>); break;
                    case 8: QuickCallFunction(X3, SlowWrite7<u8, 0>); break;
                    case 9: QuickCallFunction(X3, SlowWrite7<u8, 1>); break;
                    }
                }
                else
                {
                    switch (size | NDS.ConsoleType)
                    {
                    case 32: QuickCallFunction(X3, SlowRead7<u32, 0>); break;
                    case 33: QuickCallFunction(X3, SlowRead7<u32, 1>); break;
                    case 16: QuickCallFunction(X3, SlowRead7<u16, 0>); break;
                    case 17: QuickCallFunction(X3, SlowRead7<u16, 1>); break;
                    case 8: QuickCallFunction(X3, SlowRead7<u8, 0>); break;
                    case 9: QuickCallFunction(X3, SlowRead7<u8, 1>); break;
                    }
                }
            }

            PopRegs(false, false);

            if (!(flags & memop_Store))
            {
                if (size == 32)
                    MOV(rdMapped, W0);
                else if (flags & memop_SignExtend)
                    SBFX(rdMapped, W0, 0, size);
                else
                    UBFX(rdMapped, W0, 0, size);
            }
        }
#ifdef LITEV_MEM_MAINRAM_LOAD
        if (mainramFast)
            SetJumpTarget(mainramDone);
#endif
#ifdef LITEV_JIT_VWRAM_LOAD
        if (vwramFast)
            SetJumpTarget(vwramDone);
#endif
    }

    if (CurInstr.Info.Branches())
    {
        if (size < 32)
            Log(LogLevel::Debug, "LDR size < 32 branching?\n");
        Comp_JumpTo(rdMapped, Num == 0, false);
    }
}

void Compiler::A_Comp_MemWB()
{
    Op2 offset;
    if (CurInstr.Instr & (1 << 25))
        offset = Op2(MapReg(CurInstr.A_Reg(0)), (ShiftType)((CurInstr.Instr >> 5) & 0x3), (CurInstr.Instr >> 7) & 0x1F);
    else
        offset = Op2(CurInstr.Instr & 0xFFF);

    bool load = CurInstr.Instr & (1 << 20);
    bool byte = CurInstr.Instr & (1 << 22);

    int flags = 0;
    if (!load)
        flags |= memop_Store;
    if (!(CurInstr.Instr & (1 << 24)))
        flags |= memop_Post;
    if (CurInstr.Instr & (1 << 21))
        flags |= memop_Writeback;
    if (!(CurInstr.Instr & (1 << 23)))
        flags |= memop_SubtractOffset;

    Comp_MemAccess(CurInstr.A_Reg(12), CurInstr.A_Reg(16), offset, byte ? 8 : 32, flags);
}

void Compiler::A_Comp_MemHD()
{
    bool load = CurInstr.Instr & (1 << 20);
    bool signExtend;
    int op = (CurInstr.Instr >> 5) & 0x3;
    int size;
    
    if (load)
    {
        signExtend = op >= 2;
        size = op == 2 ? 8 : 16;
    }
    else
    {
        size = 16;
        signExtend = false;
    }

    Op2 offset;
    if (CurInstr.Instr & (1 << 22))
        offset = Op2((CurInstr.Instr & 0xF) | ((CurInstr.Instr >> 4) & 0xF0));
    else
        offset = Op2(MapReg(CurInstr.A_Reg(0)));
    
    int flags = 0;
    if (signExtend)
        flags |= memop_SignExtend;
    if (!load)
        flags |= memop_Store;
    if (!(CurInstr.Instr & (1 << 24)))
        flags |= memop_Post;
    if (!(CurInstr.Instr & (1 << 23)))
        flags |= memop_SubtractOffset;
    if (CurInstr.Instr & (1 << 21))
        flags |= memop_Writeback;

    Comp_MemAccess(CurInstr.A_Reg(12), CurInstr.A_Reg(16), offset, size, flags);
}

void Compiler::T_Comp_MemReg()
{
    int op = (CurInstr.Instr >> 10) & 0x3;
    bool load = op & 0x2;
    bool byte = op & 0x1;

    Comp_MemAccess(CurInstr.T_Reg(0), CurInstr.T_Reg(3), 
        Op2(MapReg(CurInstr.T_Reg(6))), byte ? 8 : 32, load ? 0 : memop_Store);
}

void Compiler::T_Comp_MemImm()
{
    int op = (CurInstr.Instr >> 11) & 0x3;
    bool load = op & 0x1;
    bool byte = op & 0x2;
    u32 offset = ((CurInstr.Instr >> 6) & 0x1F) * (byte ? 1 : 4);

    Comp_MemAccess(CurInstr.T_Reg(0), CurInstr.T_Reg(3), Op2(offset), 
        byte ? 8 : 32, load ? 0 : memop_Store);
}

void Compiler::T_Comp_MemRegHalf()
{
    int op = (CurInstr.Instr >> 10) & 0x3;
    bool load = op != 0;
    int size = op != 1 ? 16 : 8;
    bool signExtend = op & 1;

    int flags = 0;
    if (signExtend)
        flags |= memop_SignExtend;
    if (!load)
        flags |= memop_Store;

    Comp_MemAccess(CurInstr.T_Reg(0), CurInstr.T_Reg(3), Op2(MapReg(CurInstr.T_Reg(6))),
        size, flags);
}

void Compiler::T_Comp_MemImmHalf()
{
    u32 offset = (CurInstr.Instr >> 5) & 0x3E;
    bool load = CurInstr.Instr & (1 << 11);

    Comp_MemAccess(CurInstr.T_Reg(0), CurInstr.T_Reg(3), Op2(offset), 16,
        load ? 0 : memop_Store);
}

void Compiler::T_Comp_LoadPCRel()
{
    u32 offset = ((CurInstr.Instr & 0xFF) << 2);
    u32 addr = (R15 & ~0x2) + offset;

    if (!NDS.JIT.LiteralOptimizationsEnabled() || !Comp_MemLoadLiteral(32, false, CurInstr.T_Reg(8), addr))
        Comp_MemAccess(CurInstr.T_Reg(8), 15, Op2(offset), 32, 0);
}

void Compiler::T_Comp_MemSPRel()
{
    u32 offset = (CurInstr.Instr & 0xFF) * 4;
    bool load = CurInstr.Instr & (1 << 11);

    Comp_MemAccess(CurInstr.T_Reg(8), 13, Op2(offset), 32, load ? 0 : memop_Store);
}

s32 Compiler::Comp_MemAccessBlock(int rn, BitSet16 regs, bool store, bool preinc, bool decrement, bool usermode, bool skipLoadingRn)
{
    IrregularCycles = true;

    int regsCount = regs.Count();

    if (regsCount == 0)
        return 0; // actually not the right behaviour TODO: fix me

    int firstReg = *regs.begin();
    if (regsCount == 1 && !usermode && RegCache.LoadedRegs & (1 << firstReg) && !(firstReg == rn && skipLoadingRn))
    {
        int flags = 0;
        if (store)
            flags |= memop_Store;
        if (decrement)
            flags |= memop_SubtractOffset;
        Op2 offset = preinc ? Op2(4) : Op2(0);

        Comp_MemAccess(firstReg, rn, offset, 32, flags);

        return decrement ? -4 : 4;
    }

    if (store)
        Comp_AddCycles_CD();
    else
        Comp_AddCycles_CDI();

    int expectedTarget = Num == 0
        ? NDS.JIT.Memory.ClassifyAddress9(CurInstr.DataRegion)
        : NDS.JIT.Memory.ClassifyAddress7(CurInstr.DataRegion);

#ifdef LITEV_JIT_USERSTM_FASTMEM
    const bool userFast = usermode && store && UserStmFastOn();
#else
    const bool userFast = false;
#endif
#ifdef LITEV_JIT_LDM_FASTMEM
    // Loads take the fault-backed fastmem path too (LDP straight into the guest registers,
    // the helper call out of line in the far region), not only stores. A fault mid-transfer
    // re-runs the whole transfer through the slow path from W0 (the start address, never a
    // loaded register), which rewrites every destination, so a partially loaded block leaves
    // no trace. Cycles were added above, identically for both paths.
    bool compileFastPath = NDS.JIT.FastMemoryEnabled()
        && (!usermode || userFast) && ((CurInstr.Cond() < 0xE && !CondMemGuess(false)) || NDS.JIT.Memory.IsFastmemCompatible(expectedTarget));
#else
    bool compileFastPath = NDS.JIT.FastMemoryEnabled()
        && store && (!usermode || userFast) && ((CurInstr.Cond() < 0xE && !CondMemGuess(false)) || NDS.JIT.Memory.IsFastmemCompatible(expectedTarget));
#endif

    {
        s32 offset = decrement
            ? -regsCount * 4 + (preinc ? 0 : 4)
            : (preinc ? 4 : 0);

        if (offset)
            ADDI2R(W0, MapReg(rn), offset);
        else if (compileFastPath)
            ANDI2R(W0, MapReg(rn), ~3);
        else
            MOV(W0, MapReg(rn));
    }

    u8* patchFunc;
    if (compileFastPath)
    {
        ptrdiff_t fastPathStart = GetCodeOffset();
        ptrdiff_t loadStoreOffsets[16];

        u32 offset = 0;
        BitSet16::Iterator it = regs.begin();
        u32 i = 0;

        if (usermode)
        {
            // user-bank store (userFast): one STR per word; r8-r14 go through ReadBanked
            // (W5 = mode, W1 = index, W3 = value in/out; clobbers X1, X2, flags, LR), so the
            // host address lives in X6. On a fault the whole region becomes a call to the
            // stub below, which redoes the transfer from W0 (untouched here).
            if (regs & BitSet16(0x7f00))
            {
#ifdef LITEV_JIT_LAZYFLAGS
                LDR(INDEX_UNSIGNED, W5, RCPU, offsetof(ARM, CPSR));
                UBFX(W5, W5, 0, 5);
#else
                UBFX(W5, RCPSR, 0, 5);
#endif
            }
            ADD(X6, RMemBase, X0);
            for (int reg : regs)
            {
                ARM64Reg val = W3;
                if (RegCache.LoadedRegs & (1 << reg))
                {
                    if (reg >= 8 && reg < 15)
                        MOV(W3, MapReg(reg));
                    else
                        val = MapReg(reg);
                }
                else
                    LoadReg(reg, W3);
                if (reg >= 8 && reg < 15)
                {
                    MOVI2R(W1, reg - 8);
                    BL(ReadBanked);
                }
                loadStoreOffsets[i++] = GetCodeOffset();
                STR(INDEX_UNSIGNED, val, X6, offset);
                offset += 4;
            }
            it = regs.end();
        }
        else
            ADD(X1, RMemBase, X0);

        if (!usermode && (regsCount & 1))
        {
            int reg = *it;
            it++;

            ARM64Reg first = W3;
            if (RegCache.LoadedRegs & (1 << reg))
                first = MapReg(reg);
            else if (store)
                LoadReg(reg, first);

            loadStoreOffsets[i++] = GetCodeOffset();

            if (store)
            {
                STR(INDEX_UNSIGNED, first, X1, offset);
            }
            else if (!(reg == rn && skipLoadingRn))
            {
                LDR(INDEX_UNSIGNED, first, X1, offset);

                if (!(RegCache.LoadedRegs & (1 << reg)))
                    SaveReg(reg, first);
            }

            offset += 4;
        }

        while (it != regs.end())
        {
            int reg = *it;
            it++;
            int nextReg = *it;
            it++;

            ARM64Reg first = W3, second = W4;
            if (RegCache.LoadedRegs & (1 << reg))
            {
                if (!(reg == rn && skipLoadingRn))
                    first = MapReg(reg);
            }
            else if (store)
            {
                LoadReg(reg, first);
            }
            if (RegCache.LoadedRegs & (1 << nextReg))
            {
                if (!(nextReg == rn && skipLoadingRn))
                    second = MapReg(nextReg);
            }
            else if (store)
            {
                LoadReg(nextReg, second);
            }

            loadStoreOffsets[i++] = GetCodeOffset();
            if (store)
            {
                STP(INDEX_SIGNED, first, second, X1, offset);
            }
            else
            {
                LDP(INDEX_SIGNED, first, second, X1, offset);
            
                if (!(RegCache.LoadedRegs & (1 << reg)))
                    SaveReg(reg, first);
                if (!(RegCache.LoadedRegs & (1 << nextReg)))
                    SaveReg(nextReg, second);
            }

            offset += 8;
        }

        LoadStorePatch patch;
        patch.PatchSize = GetCodeOffset() - fastPathStart;
        SwapCodeRegion();
        patchFunc = (u8*)GetRXPtr();
        patch.PatchFunc = patchFunc;
        u32 numLoadStores = i;
        for (i = 0; i < numLoadStores; i++)
        {
            patch.PatchOffset = fastPathStart - loadStoreOffsets[i];
            LoadStorePatches[loadStoreOffsets[i]] = patch;
        }

        ABI_PushRegisters({30});
    }

    int i = 0;

    SUB(SP, SP, ((regsCount + 1) & ~1) * 8);
    if (store)
    {
        if (usermode && (regs & BitSet16(0x7f00)))
#ifdef LITEV_JIT_LAZYFLAGS
            // Usermode LDM/STM bank select reads guest mode (control) from ARM::CPSR memory
            // (RCPSR is now guest r7). Control is always canonical in memory.
            LDR(INDEX_UNSIGNED, W5, RCPU, offsetof(ARM, CPSR));
            UBFX(W5, W5, 0, 5);
#else
            UBFX(W5, RCPSR, 0, 5);
#endif

        BitSet16::Iterator it = regs.begin();
        while (it != regs.end())
        {
            BitSet16::Iterator nextReg = it;
            nextReg++;

            int reg = *it;

            if (usermode && reg >= 8 && reg < 15)
            {
                if (RegCache.LoadedRegs & (1 << reg))
                    MOV(W3, MapReg(reg));
                else
                    LoadReg(reg, W3);
                MOVI2R(W1, reg - 8);
                BL(ReadBanked);
                STR(INDEX_UNSIGNED, W3, SP, i * 8);
            }
            else if (!usermode && nextReg != regs.end())
            {
                ARM64Reg first = W3, second = W4;

                if (RegCache.LoadedRegs & (1 << reg))
                    first = MapReg(reg);
                else
                    LoadReg(reg, W3);

                if (RegCache.LoadedRegs & (1 << *nextReg))
                    second = MapReg(*nextReg);
                else
                    LoadReg(*nextReg, W4);

                STP(INDEX_SIGNED, EncodeRegTo64(first), EncodeRegTo64(second), SP, i * 8);

                i++;
                it++;
            }
            else if (RegCache.LoadedRegs & (1 << reg))
            {
                STR(INDEX_UNSIGNED, MapReg(reg), SP, i * 8);
            }
            else
            {
                LoadReg(reg, W3);
                STR(INDEX_UNSIGNED, W3, SP, i * 8);
            }
            i++;
            it++;
        }
    }

    // ---- M3 Tier A: DTCM-hit inline block transfer -------------------------
    // Emitted ONLY where the block helper would otherwise be called (i.e. the
    // fastmem path was not taken: fastmem disabled on this host, or a shape
    // fastmem never covers such as ANY LDM / a non-fastmem-region STM). ARM9
    // only (DTCM is an ARMv5 feature). One compound runtime guard, one fallback
    // to the exact existing helper. The guest-register<->stack marshalling above
    // (store) and below (load) is SHARED by both paths and is the only part that
    // touches the register cache; the guard merely chooses whether the stack
    // buffer is filled/drained by an inline DTCM copy or by SlowBlockTransfer9.
    // Cycles were already added by Comp_AddCycles_CD/CDI above, identically on
    // both paths, so this is bit-exact.
#ifdef LITEV_MEM_DTCM_BLOCK
    const bool dtcmFast = (Num == 0) && !compileFastPath && (regsCount >= 1);
#else
    const bool dtcmFast = false;
#endif
#ifdef LITEV_MEM_DTCM_BLOCK
    FixupBranch dtcmDone;
    if (dtcmFast)
    {
        FixupBranch dtcmMiss[3];
        int nMiss = 0;

        // W0 = transfer start address (lowest addr, not yet & ~3 masked on this
        // path; the helper masks internally). DTCMMask has zero low bits so the
        // region test is unaffected by the low 2 bits.
        LDR(INDEX_UNSIGNED, W6, RCPU, offsetof(ARMv5, DTCMBase));
        LDR(INDEX_UNSIGNED, W7, RCPU, offsetof(ARMv5, DTCMMask));

        // guard 1: first word in the DTCM region.
        AND(W5, W0, W7);
        CMP(W5, W6);
        dtcmMiss[nMiss++] = B(CC_NEQ);

        // guard 2: last word in the SAME region (no region exit mid-transfer;
        // SlowBlockTransfer re-classifies every word, so a partial-region block
        // must fall back).
        if (regsCount > 1)
        {
            ADDI2R(W5, W0, (u32)((regsCount - 1) * 4));
            AND(W5, W5, W7);
            CMP(W5, W6);
            dtcmMiss[nMiss++] = B(CC_NEQ);
        }

        // guard 3: no 16KB physical wrap (matters only for DTCM regions > 16KB,
        // which mirror; keeps the inline copy contiguous == SlowRead's per-word
        // & (DTCMPhysicalSize-1) indexing). idx = (addr & ~3) & (0x4000-1).
        ANDI2R(W5, W0, (u32)(DTCMPhysicalSize - 4));
        CMPI2R(W5, (u32)(DTCMPhysicalSize - regsCount * 4), W3);
        dtcmMiss[nMiss++] = B(CC_HI);

        // hit: X4 = DTCM base pointer + idx (W5, zero-extended by the 32-bit AND).
        ADDI2R(X4, RCPU, offsetof(ARMv5, DTCM), X3);
        LDR(INDEX_UNSIGNED, X4, X4, 0);
        ADD(X4, X4, EncodeRegTo64(W5));

        // Copy between the stack marshalling buffer (8 bytes/word, low 4 = data,
        // matching u64* data[]) and DTCM (contiguous 4 bytes/word).
        //
        // LITEV_JIT_LDMSTM: pair the contiguous DTCM side with LDP/STP, mirroring
        // DraStic's arm64_load/store_blockN stubs which move two guest words per
        // instruction (ldp/stp) instead of a per-word ldr/str loop. Only the DTCM
        // side is contiguous 4-byte and pairs cleanly; the stack buffer has an
        // 8-byte stride (u64 slots) so its side stays scalar. Byte-identical to the
        // per-word copy: same words, same values, same relative order (the two
        // words STP writes are contiguous, exactly the two STRs they replace).
        // X4 (== W4) is the live base pointer, so the second scratch is W5 (free
        // after the region guards), never W4.
        int w = 0;
#ifdef LITEV_JIT_LDMSTM
        for (; w + 1 < regsCount; w += 2)
        {
            if (store)
            {
                LDR(INDEX_UNSIGNED, W3, SP, w * 8);
                LDR(INDEX_UNSIGNED, W5, SP, (w + 1) * 8);
                STP(INDEX_SIGNED, W3, W5, X4, w * 4);
            }
            else
            {
                LDP(INDEX_SIGNED, W3, W5, X4, w * 4);
                STR(INDEX_UNSIGNED, W3, SP, w * 8);
                STR(INDEX_UNSIGNED, W5, SP, (w + 1) * 8);
            }
        }
#endif
        for (; w < regsCount; w++)
        {
            if (store)
            {
                LDR(INDEX_UNSIGNED, W3, SP, w * 8);
                STR(INDEX_UNSIGNED, W3, X4, w * 4);
            }
            else
            {
                LDR(INDEX_UNSIGNED, W3, X4, w * 4);
                STR(INDEX_UNSIGNED, W3, SP, w * 8);
            }
        }

        dtcmDone = B();
        for (int m = 0; m < nMiss; m++)
            SetJumpTarget(dtcmMiss[m]);
    }
#endif

#if defined(LITEV_JIT_LDMSTM)
    // ---- MainRAM-hit inline block LOAD (LITEV_JIT_LDMSTM) -------------------
    // Composes with the DTCM tier above: a block whose whole address range lies
    // inside MainRAM (the SlowBlockTransfer9 -> per-word SlowRead9<u32> dominant
    // case, ~86% of in-race slow reads) takes a guarded direct-pointer copy in
    // place of the per-word helper. LOADS ONLY: stores keep the helper so JIT
    // block invalidation is completely untouched by this change. ARM9 + DS only
    // (SlowBlockTransfer9; MainRAMMask/base are DS values, baked at compile time
    // for the block cache's lifetime). Runtime guards fall back to the exact
    // helper on any DTCM overlay, mid-block region exit, or MainRAM mirror wrap;
    // the guest-register<->stack marshalling (drain, below) is shared with the
    // helper path, identical for both. Cycles were already added by
    // Comp_AddCycles_CDI above, so this is bit-exact.
    //
    // Gated on the static CurInstr.DataRegion classification (expectedTarget ==
    // MainRAM): measured in-race this hint resolves for ~98% of the MainRAM LDMs,
    // so the runtime guards below rarely miss. Emitting mainram guards for EVERY
    // block (as the DTCM tier does) captured only ~1.6% more transfers while
    // running an 8-branch dtcm+mainram guard prologue on every non-MainRAM block
    // -- net-negative overhead. The runtime guards still fully protect exactness;
    // the gate is a pure emission-cost optimisation.
    const bool mainramBlockFast = (Num == 0) && (NDS.ConsoleType == 0)
        && !compileFastPath && !store && (regsCount >= 1)
        && (expectedTarget == ARMJIT_Memory::memregion_MainRAM);
    FixupBranch mainramBlockDone;
    if (mainramBlockFast)
    {
        FixupBranch mrbMiss[5];
        int nMiss = 0;

        // W0 = transfer start address (lowest addr; low 2 bits ignored, the
        // helper masks internally). DTCMMask has zero low bits, so masking off
        // the low 2 bits does not disturb the DTCM region test.
        LDR(INDEX_UNSIGNED, W6, RCPU, offsetof(ARMv5, DTCMBase));
        LDR(INDEX_UNSIGNED, W7, RCPU, offsetof(ARMv5, DTCMMask));
        ANDI2R(W4, W0, ~3);                     // word-aligned start address

        // guard 1: first word NOT in the DTCM overlay (SlowRead9 checks DTCM
        // before the region switch, so a DTCM-shadowed word must fall back).
        AND(W5, W4, W7);
        CMP(W5, W6);
        mrbMiss[nMiss++] = B(CC_EQ);

        // guard 2: first word in the MainRAM region (DS: (addr&0xFF000000)==0x02000000).
        ANDI2R(W5, W4, 0xFF000000);
        MOVI2R(W3, 0x02000000);
        CMP(W5, W3);
        mrbMiss[nMiss++] = B(CC_NEQ);

        // guard for multi-word blocks: last word in the SAME region and NOT in
        // DTCM (no region/overlay exit mid-block; SlowBlockTransfer re-classifies
        // every word). The block span (<=64B) is far smaller than the DTCM min
        // region and the 16MB MainRAM region, so endpoint checks cover every
        // interior word of the contiguous transfer.
        if (regsCount > 1)
        {
            ADDI2R(W3, W4, (u32)((regsCount - 1) * 4));   // last word address
            AND(W5, W3, W7);
            CMP(W5, W6);
            mrbMiss[nMiss++] = B(CC_EQ);
            ANDI2R(W5, W3, 0xFF000000);
            MOVI2R(W3, 0x02000000);
            CMP(W5, W3);
            mrbMiss[nMiss++] = B(CC_NEQ);
        }

        // guard 3: no MainRAM physical wrap. The region is mirrored, so a block
        // that crosses the physical end wraps to offset 0 per SlowRead's per-word
        // & MainRAMMask; keep the inline contiguous copy equivalent by falling
        // back in that case. idx = (addr & ~3) & MainRAMMask.
        ANDI2R(W5, W4, (u32)(NDS.MainRAMMask & ~3));
        CMPI2R(W5, (u32)((NDS.MainRAMMask + 1) - regsCount * 4), W3);
        mrbMiss[nMiss++] = B(CC_HI);

        // hit: X4 = MainRAM base + physical index (W5, zero-extended by the AND).
        MOVP2R(X7, NDS.MainRAM);
        ADD(X4, X7, EncodeRegTo64(W5));

        // Drain MainRAM -> stack marshalling buffer (8 bytes/word, low 4 = data,
        // matching u64* data[]); the shared load code below writes it to guest regs.
        //
        // LITEV_JIT_LDMSTM: pair the contiguous MainRAM side with LDP (DraStic's
        // arm64_load_blockN moves two guest words per instruction). The stack
        // buffer stride is 8 bytes (u64 slots) so its side stays scalar. X4 (== W4)
        // is the live base; second scratch is W5 (free after the region guards).
        // Byte-identical to the per-word drain.
        int w = 0;
#ifdef LITEV_JIT_LDMSTM
        for (; w + 1 < regsCount; w += 2)
        {
            LDP(INDEX_SIGNED, W3, W5, X4, w * 4);
            STR(INDEX_UNSIGNED, W3, SP, w * 8);
            STR(INDEX_UNSIGNED, W5, SP, (w + 1) * 8);
        }
#endif
        for (; w < regsCount; w++)
        {
            LDR(INDEX_UNSIGNED, W3, X4, w * 4);
            STR(INDEX_UNSIGNED, W3, SP, w * 8);
        }

        mainramBlockDone = B();
        for (int m = 0; m < nMiss; m++)
            SetJumpTarget(mrbMiss[m]);
    }
#endif

    // ---- Tier B / C fallback: the exact upstream helper --------------------
#if defined(LITEV_JIT_LDMSTM)
    PushRegs(false, false, !compileFastPath && !dtcmFast && !mainramBlockFast);
#else
    PushRegs(false, false, !compileFastPath && !dtcmFast);
#endif

    ADD(X1, SP, 0);
    MOVI2R(W2, regsCount);

    if (Num == 0)
    {
        MOV(X3, RCPU);
        switch ((u32)store * 2 | NDS.ConsoleType)
        {
        case 0: QuickCallFunction(X4, SlowBlockTransfer9<false, 0>); break;
        case 1: QuickCallFunction(X4, SlowBlockTransfer9<false, 1>); break;
        case 2: QuickCallFunction(X4, SlowBlockTransfer9<true, 0>); break;
        case 3: QuickCallFunction(X4, SlowBlockTransfer9<true, 1>); break;
        }
        if (store)
            NOTE_SLOW_SITE((u32)expectedTarget | (CurInstr.Cond() << 8) | (CurInstr.DataExecuted << 12) | ((u32)Thumb << 13)
                | ((u32)usermode << 14) | ((u32)compileFastPath << 15) | ((u32)regsCount << 16) | (1u << 31));
    }
    else
    {
        switch ((u32)store * 2 | NDS.ConsoleType)
        {
        case 0: QuickCallFunction(X4, SlowBlockTransfer7<false, 0>); break;
        case 1: QuickCallFunction(X4, SlowBlockTransfer7<false, 1>); break;
        case 2: QuickCallFunction(X4, SlowBlockTransfer7<true, 0>); break;
        case 3: QuickCallFunction(X4, SlowBlockTransfer7<true, 1>); break;
        }
    }

    PopRegs(false, false);

#ifdef LITEV_MEM_DTCM_BLOCK
    if (dtcmFast)
        SetJumpTarget(dtcmDone);
#endif
#if defined(LITEV_JIT_LDMSTM)
    if (mainramBlockFast)
        SetJumpTarget(mainramBlockDone);
#endif

    if (!store)
    {
        if (usermode && !regs[15] && (regs & BitSet16(0x7f00)))
#ifdef LITEV_JIT_LAZYFLAGS
            // Usermode LDM/STM bank select reads guest mode (control) from ARM::CPSR memory
            // (RCPSR is now guest r7). Control is always canonical in memory.
            LDR(INDEX_UNSIGNED, W5, RCPU, offsetof(ARM, CPSR));
            UBFX(W5, W5, 0, 5);
#else
            UBFX(W5, RCPSR, 0, 5);
#endif

        BitSet16::Iterator it = regs.begin();
        while (it != regs.end())
        {
            BitSet16::Iterator nextReg = it;
            nextReg++;

            int reg = *it;

            if (usermode && !regs[15] && reg >= 8 && reg < 15)
            {
                LDR(INDEX_UNSIGNED, W3, SP, i * 8);
                MOVI2R(W1, reg - 8);
                BL(WriteBanked);
                if (!(reg == rn && skipLoadingRn))
                {
                    FixupBranch alreadyWritten = CBNZ(W4);
                    if (RegCache.LoadedRegs & (1 << reg))
                        MOV(MapReg(reg), W3);
                    else
                        SaveReg(reg, W3);
                    SetJumpTarget(alreadyWritten);
                }
            }
            else if (!usermode && nextReg != regs.end())
            {
                ARM64Reg first = W3, second = W4;
                
                if (RegCache.LoadedRegs & (1 << reg) && !(reg == rn && skipLoadingRn))
                    first = MapReg(reg);
                if (RegCache.LoadedRegs & (1 << *nextReg) && !(*nextReg == rn && skipLoadingRn))
                    second = MapReg(*nextReg);

                LDP(INDEX_SIGNED, EncodeRegTo64(first), EncodeRegTo64(second), SP, i * 8);

                if (first == W3)
                    SaveReg(reg, W3);
                if (second == W4)
                    SaveReg(*nextReg, W4);

                it++;
                i++;
            }
            else if (RegCache.LoadedRegs & (1 << reg))
            {
                if (!(reg == rn && skipLoadingRn))
                {
                    ARM64Reg mapped = MapReg(reg);
                    LDR(INDEX_UNSIGNED, mapped, SP, i * 8);
                }
            }
            else
            {
                LDR(INDEX_UNSIGNED, W3, SP, i * 8);
                SaveReg(reg, W3);
            }

            it++;
            i++;
        }
    }
    ADD(SP, SP, ((regsCount + 1) & ~1) * 8);

    if (compileFastPath)
    {
        ABI_PopRegisters({30});
        RET();

        FlushIcacheSection(patchFunc, (u8*)GetRXPtr());
        SwapCodeRegion();
    }

    if (!store && regs[15])
    {
        ARM64Reg mapped = MapReg(15);
        Comp_JumpTo(mapped, Num == 0, usermode);
    }

    return regsCount * 4 * (decrement ? -1 : 1);
}

void Compiler::A_Comp_LDM_STM()
{
    BitSet16 regs(CurInstr.Instr & 0xFFFF);

    bool load = CurInstr.Instr & (1 << 20);
    bool pre = CurInstr.Instr & (1 << 24);
    bool add = CurInstr.Instr & (1 << 23);
    bool writeback = CurInstr.Instr & (1 << 21);
    bool usermode = CurInstr.Instr & (1 << 22);

    ARM64Reg rn = MapReg(CurInstr.A_Reg(16));

    if (load && writeback && regs[CurInstr.A_Reg(16)])
        writeback = Num == 0
            && (!(regs & ~BitSet16(1 << CurInstr.A_Reg(16)))) || (regs & ~BitSet16((2 << CurInstr.A_Reg(16)) - 1));

    s32 offset = Comp_MemAccessBlock(CurInstr.A_Reg(16), regs, !load, pre, !add, usermode, load && writeback);

    if (writeback && offset)
    {
        if (offset > 0)
            ADD(rn, rn, offset);
        else
            SUB(rn, rn, -offset);
    }
}

void Compiler::T_Comp_PUSH_POP()
{
    bool load = CurInstr.Instr & (1 << 11);
    BitSet16 regs(CurInstr.Instr & 0xFF);
    if (CurInstr.Instr & (1 << 8))
    {
        if (load)
            regs[15] = true;
        else
            regs[14] = true;
    }

    ARM64Reg sp = MapReg(13);
    s32 offset = Comp_MemAccessBlock(13, regs, !load, !load, !load, false, false);

    if (offset)
    {
        if (offset > 0)
            ADD(sp, sp, offset);
        else
            SUB(sp, sp, -offset);
    }
}

void Compiler::T_Comp_LDMIA_STMIA()
{
    BitSet16 regs(CurInstr.Instr & 0xFF);
    ARM64Reg rb = MapReg(CurInstr.T_Reg(8));
    bool load = CurInstr.Instr & (1 << 11);
    u32 regsCount = regs.Count();

    bool writeback = !load || !regs[CurInstr.T_Reg(8)];

    s32 offset = Comp_MemAccessBlock(CurInstr.T_Reg(8), regs, !load, false, false, false, load && writeback);

    if (writeback && offset)
    {
        if (offset > 0)
            ADD(rb, rb, offset);
        else
            SUB(rb, rb, -offset);
    }
}

}
