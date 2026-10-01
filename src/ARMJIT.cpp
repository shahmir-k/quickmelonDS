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

#include "ARMJIT.h"
#include "ARMJIT_Memory.h"
#include <string.h>
#include <assert.h>
#include <unordered_map>
#include <unordered_set>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <algorithm>

#define XXH_STATIC_LINKING_ONLY
#include "xxhash/xxhash.h"

#include "Platform.h"

#include "ARMJIT_Internal.h"
#include "ARMJIT_Memory.h"
#include "ARMJIT_Compiler.h"
#include "ARMJIT_Global.h"
#include "LiteProfile.h"

#include "ARMInterpreter_ALU.h"
#include "ARMInterpreter_LoadStore.h"
#include "ARMInterpreter_Branch.h"
#include "ARMInterpreter.h"

#include "DSi.h"
#include "GPU.h"
#include "GPU3D.h"
#include "SPU.h"
#include "Wifi.h"
#include "NDSCart.h"
#include "Platform.h"
#include "ARMJIT_x64/ARMJIT_Offsets.h"

namespace melonDS
{
using Platform::Log;
using Platform::LogLevel;

static_assert(offsetof(ARM, CPSR) == ARM_CPSR_offset, "");
static_assert(offsetof(ARM, Cycles) == ARM_Cycles_offset, "");
static_assert(offsetof(ARM, StopExecution) == ARM_StopExecution_offset, "");


#define JIT_DEBUGPRINT(msg, ...)
//#define JIT_DEBUGPRINT(msg, ...) Platform::Log(Platform::LogLevel::Debug, msg, ## __VA_ARGS__)

const u32 CodeRegionSizes[ARMJIT_Memory::memregions_Count] =
{
    0,
    ITCMPhysicalSize,
    0,
    ARM9BIOSSize,
    MainRAMMaxSize,
    SharedWRAMSize,
    0,
    0x100000,
    ARM7BIOSSize,
    ARM7WRAMSize,
    0,
    0,
    0x40000,
    0x10000,
    0x10000,
    NWRAMSize,
    NWRAMSize,
    NWRAMSize,
};

u32 ARMJIT::LocaliseCodeAddress(u32 num, u32 addr) const noexcept
{
    int region = num == 0
        ? Memory.ClassifyAddress9(addr)
        : Memory.ClassifyAddress7(addr);

    if (CodeMemRegions[region])
        return Memory.LocaliseAddress(region, num, addr);
    return 0;
}

template <typename T, int ConsoleType>
T SlowRead9(u32 addr, ARMv5* cpu)
{
    if (std::is_same<T, u32>::value)
        LITE_PROFILE_ADD(melonDS::LiteProfile::g_Frame.MemRead9U32HelperCalls);
    u32 offset = addr & 0x3;
    addr &= ~(sizeof(T) - 1);

    T val;
    if (addr < cpu->ITCMSize)
        val = *(T*)&cpu->ITCM[addr & 0x7FFF];
    else if ((addr & cpu->DTCMMask) == cpu->DTCMBase)
        val = *(T*)&cpu->DTCM[addr & 0x3FFF];
    // Use the ARM back-pointer (cpu->NDS) instead of the thread_local NDS::Current:
    // this shared-library build otherwise routes every NDS::Current read through a
    // TLS access (emulated-TLS PLT call by default) on the hot ARM9 JIT slow path.
    // cpu is already in a register here and cpu->NDS == NDS::Current for the running
    // instance, so this is bit-exact and removes TLS entirely from these helpers.
    else if (std::is_same<T, u32>::value)
        val = cpu->NDS.ARM9Read32(addr);
    else if (std::is_same<T, u16>::value)
        val = cpu->NDS.ARM9Read16(addr);
    else
        val = cpu->NDS.ARM9Read8(addr);

    if (std::is_same<T, u32>::value)
        return ROR(val, offset << 3);
    else
        return val;
}

template <typename T, int ConsoleType>
T SlowRead7(u32 addr)
{
    u32 offset = addr & 0x3;
    addr &= ~(sizeof(T) - 1);

    T val;
    if (std::is_same<T, u32>::value)
        val = NDS::Current->ARM7Read32(addr);
    else if (std::is_same<T, u16>::value)
        val = NDS::Current->ARM7Read16(addr);
    else
        val = NDS::Current->ARM7Read8(addr);

    if (std::is_same<T, u32>::value)
        return ROR(val, offset << 3);
    else
        return val;
}

template <typename T, int ConsoleType>
void SlowWrite9(u32 addr, ARMv5* cpu, u32 val)
{
    addr &= ~(sizeof(T) - 1);

    if (addr < cpu->ITCMSize)
    {
        cpu->NDS.JIT.CheckAndInvalidate<0, ARMJIT_Memory::memregion_ITCM>(addr);
        *(T*)&cpu->ITCM[addr & 0x7FFF] = val;
    }
    else if ((addr & cpu->DTCMMask) == cpu->DTCMBase)
    {
        *(T*)&cpu->DTCM[addr & 0x3FFF] = val;
    }
    // cpu->NDS back-pointer instead of thread_local NDS::Current (see SlowRead9):
    // removes the TLS access from the hot ARM9 store slow path. Bit-exact.
    else if (std::is_same<T, u32>::value)
    {
        cpu->NDS.ARM9Write32(addr, val);
    }
    else if (std::is_same<T, u16>::value)
    {
        cpu->NDS.ARM9Write16(addr, val);
    }
    else
    {
        cpu->NDS.ARM9Write8(addr, val);
    }
}

template <typename T, int ConsoleType>
void SlowWrite7(u32 addr, u32 val)
{
    addr &= ~(sizeof(T) - 1);

    if (std::is_same<T, u32>::value)
        NDS::Current->ARM7Write32(addr, val);
    else if (std::is_same<T, u16>::value)
        NDS::Current->ARM7Write16(addr, val);
    else
        NDS::Current->ARM7Write8(addr, val);
}

template <bool Write, int ConsoleType>
void SlowBlockTransfer9(u32 addr, u64* data, u32 num, ARMv5* cpu)
{
    LITE_PROFILE_ADD(melonDS::LiteProfile::g_Frame.MemBlock9HelperCalls);
    addr &= ~0x3;
#ifdef LITEV_JIT_BLOCKXFER_FAST
    // Hoist the per-element region dispatch out of the loop for the two homogeneous,
    // no-side-effect regions (ITCM/DTCM). The per-element SlowRead9/SlowWrite9 re-run the
    // ITCM/DTCM/region classification for EVERY word; for a block that lies wholly inside
    // one TCM we classify ONCE (the whole range fits in a single <=64B LDM/STM, far smaller
    // than the TCM span, so both ends inside => all words inside) and loop directly.
    // BIT-EXACT: identical value at each identical address; ITCM writes still
    // CheckAndInvalidate the JIT block cache per word exactly as the scalar path does. Any
    // block that straddles a region or hits a side-effectful region falls through to the
    // exact per-element path below (MMIO/VRAM/palette/OAM semantics unchanged). MP-safe:
    // no AddCycles / event / timing change — only the host classification is elided.
    if (num)
    {
        const u32 bytes = num << 2;
        const u32 last  = addr + bytes - 4;
        if (addr + bytes <= cpu->ITCMSize)               // whole block in ITCM
        {
            for (u32 i = 0; i < num; i++)
            {
                const u32 a = addr + (i << 2);
                if (Write)
                {
                    cpu->NDS.JIT.CheckAndInvalidate<0, ARMJIT_Memory::memregion_ITCM>(a);
                    *(u32*)&cpu->ITCM[a & 0x7FFF] = (u32)data[i];
                }
                else
                    data[i] = *(u32*)&cpu->ITCM[a & 0x7FFF];
            }
            return;
        }
        if ((addr & cpu->DTCMMask) == cpu->DTCMBase
         && (last & cpu->DTCMMask) == cpu->DTCMBase)      // whole block in DTCM
        {
            for (u32 i = 0; i < num; i++)
            {
                const u32 a = addr + (i << 2);
                if (Write) *(u32*)&cpu->DTCM[a & 0x3FFF] = (u32)data[i];
                else       data[i] = *(u32*)&cpu->DTCM[a & 0x3FFF];
            }
            return;
        }
    }
#endif

    for (u32 i = 0; i < num; i++)
    {
        if (Write)
            SlowWrite9<u32, ConsoleType>(addr, cpu, data[i]);
        else
            data[i] = SlowRead9<u32, ConsoleType>(addr, cpu);
        addr += 4;
    }
}

template <bool Write, int ConsoleType>
void SlowBlockTransfer7(u32 addr, u64* data, u32 num)
{
    addr &= ~0x3;
    for (u32 i = 0; i < num; i++)
    {
        if (Write)
            SlowWrite7<u32, ConsoleType>(addr, data[i]);
        else
            data[i] = SlowRead7<u32, ConsoleType>(addr);
        addr += 4;
    }
}

#define INSTANTIATE_SLOWMEM(consoleType) \
    template void SlowWrite9<u32, consoleType>(u32, ARMv5*, u32); \
    template void SlowWrite9<u16, consoleType>(u32, ARMv5*, u32); \
    template void SlowWrite9<u8, consoleType>(u32, ARMv5*, u32); \
    \
    template u32 SlowRead9<u32, consoleType>(u32, ARMv5*); \
    template u16 SlowRead9<u16, consoleType>(u32, ARMv5*); \
    template u8 SlowRead9<u8, consoleType>(u32, ARMv5*); \
    \
    template void SlowWrite7<u32, consoleType>(u32, u32); \
    template void SlowWrite7<u16, consoleType>(u32, u32); \
    template void SlowWrite7<u8, consoleType>(u32, u32); \
    \
    template u32 SlowRead7<u32, consoleType>(u32); \
    template u16 SlowRead7<u16, consoleType>(u32); \
    template u8 SlowRead7<u8, consoleType>(u32); \
    \
    template void SlowBlockTransfer9<false, consoleType>(u32, u64*, u32, ARMv5*); \
    template void SlowBlockTransfer9<true, consoleType>(u32, u64*, u32, ARMv5*); \
    template void SlowBlockTransfer7<false, consoleType>(u32 addr, u64* data, u32 num); \
    template void SlowBlockTransfer7<true, consoleType>(u32 addr, u64* data, u32 num); \

INSTANTIATE_SLOWMEM(0)
INSTANTIATE_SLOWMEM(1)

ARMJIT::~ARMJIT() noexcept
{
    JitEnableWrite();
    ResetBlockCache();
}

void ARMJIT::Reset() noexcept
{
    JitEnableWrite();
    ResetBlockCache();

    Memory.Reset();
}

void FloodFillSetFlags(FetchedInstr instrs[], int start, u8 flags)
{
    for (int j = start; j >= 0; j--)
    {
        u8 match = instrs[j].Info.WriteFlags & flags;
        u8 matchMaybe = (instrs[j].Info.WriteFlags >> 4) & flags;
        if (matchMaybe) // writes flags maybe
            instrs[j].SetFlags |= matchMaybe;
        if (match)
        {
            instrs[j].SetFlags |= match;
            flags &= ~match;
            if (!flags)
                return;
        }
    }
}

bool DecodeLiteral(bool thumb, const FetchedInstr& instr, u32& addr)
{
    if (!thumb)
    {
        switch (instr.Info.Kind)
        {
        case ARMInstrInfo::ak_LDR_IMM:
        case ARMInstrInfo::ak_LDRB_IMM:
            addr = (instr.Addr + 8) + ((instr.Instr & 0xFFF) * (instr.Instr & (1 << 23) ? 1 : -1));
            return true;
        case ARMInstrInfo::ak_LDRH_IMM:
            addr = (instr.Addr + 8) + (((instr.Instr & 0xF00) >> 4 | (instr.Instr & 0xF)) * (instr.Instr & (1 << 23) ? 1 : -1));
            return true;
        default:
            break;
        }
    }
    else if (instr.Info.Kind == ARMInstrInfo::tk_LDR_PCREL)
    {
        addr = ((instr.Addr + 4) & ~0x2) + ((instr.Instr & 0xFF) << 2);
        return true;
    }

    JIT_DEBUGPRINT("Literal %08x %x not recognised %d\n", instr.Instr, instr.Addr, instr.Info.Kind);
    return false;
}

bool DecodeBranch(bool thumb, const FetchedInstr& instr, u32& cond, bool hasLink, u32 lr, bool& link,
    u32& linkAddr, u32& targetAddr)
{
    if (thumb)
    {
        u32 r15 = instr.Addr + 4;
        cond = 0xE;

        link = instr.Info.Kind == ARMInstrInfo::tk_BL_LONG;
        linkAddr = instr.Addr + 4;

        if (instr.Info.Kind == ARMInstrInfo::tk_BL_LONG && !(instr.Instr & (1 << 12)))
        {
            targetAddr = r15 + ((s32)((instr.Instr & 0x7FF) << 21) >> 9);
            targetAddr += ((instr.Instr >> 16) & 0x7FF) << 1;
            return true;
        }
        else if (instr.Info.Kind == ARMInstrInfo::tk_B)
        {
            s32 offset = (s32)((instr.Instr & 0x7FF) << 21) >> 20;
            targetAddr = r15 + offset;
            return true;
        }
        else if (instr.Info.Kind == ARMInstrInfo::tk_BCOND)
        {
            cond = (instr.Instr >> 8) & 0xF;
            s32 offset = (s32)(instr.Instr << 24) >> 23;
            targetAddr = r15 + offset;
            return true;
        }
        else if (hasLink && instr.Info.Kind == ARMInstrInfo::tk_BX && instr.A_Reg(3) == 14)
        {
            JIT_DEBUGPRINT("returning!\n");
            targetAddr = lr;
            return true;
        }
    }
    else
    {
        link = instr.Info.Kind == ARMInstrInfo::ak_BL;
        linkAddr = instr.Addr + 4;

        cond = instr.Cond();
        if (instr.Info.Kind == ARMInstrInfo::ak_BL
            || instr.Info.Kind == ARMInstrInfo::ak_B)
        {
            s32 offset = (s32)(instr.Instr << 8) >> 6;
            u32 r15 = instr.Addr + 8;
            targetAddr = r15 + offset;
            return true;
        }
        else if (hasLink && instr.Info.Kind == ARMInstrInfo::ak_BX && instr.A_Reg(0) == 14)
        {
            JIT_DEBUGPRINT("returning!\n");
            targetAddr = lr;
            return true;
        }
    }
    return false;
}

bool IsIdleLoop(bool thumb, FetchedInstr* instrs, int instrsCount)
{
    // see https://github.com/dolphin-emu/dolphin/blob/master/Source/Core/Core/PowerPC/PPCAnalyst.cpp#L678
    // it basically checks if one iteration of a loop depends on another
    // the rules are quite simple

    JIT_DEBUGPRINT("checking potential idle loop\n");
    u16 regsWrittenTo = 0;
    u16 regsDisallowedToWrite = 0;
    for (int i = 0; i < instrsCount; i++)
    {
        JIT_DEBUGPRINT("instr %d %08x regs(%x %x) %x %x\n", i, instrs[i].Instr, instrs[i].Info.DstRegs, instrs[i].Info.SrcRegs, regsWrittenTo, regsDisallowedToWrite);
        if (instrs[i].Info.SpecialKind == ARMInstrInfo::special_WriteMem)
            return false;
        if (!thumb && instrs[i].Info.Kind >= ARMInstrInfo::ak_MSR_IMM && instrs[i].Info.Kind <= ARMInstrInfo::ak_MRC)
            return false;
        if (i < instrsCount - 1 && instrs[i].Info.Branches())
            return false;

        u16 srcRegs = instrs[i].Info.SrcRegs & ~(1 << 15);
        u16 dstRegs = instrs[i].Info.DstRegs & ~(1 << 15);

        regsDisallowedToWrite |= srcRegs & ~regsWrittenTo;

        if (dstRegs & regsDisallowedToWrite)
            return false;
        regsWrittenTo |= dstRegs;
    }
    return true;
}

typedef void (*InterpreterFunc)(ARM* cpu);

void NOP(ARM* cpu) {}

#define F(x) &ARMInterpreter::A_##x
#define F_ALU(name, s) \
    F(name##_REG_LSL_IMM##s), F(name##_REG_LSR_IMM##s), F(name##_REG_ASR_IMM##s), F(name##_REG_ROR_IMM##s), \
    F(name##_REG_LSL_REG##s), F(name##_REG_LSR_REG##s), F(name##_REG_ASR_REG##s), F(name##_REG_ROR_REG##s), F(name##_IMM##s)
#define F_MEM_WB(name) \
    F(name##_REG_LSL), F(name##_REG_LSR), F(name##_REG_ASR), F(name##_REG_ROR), F(name##_IMM), \
    F(name##_POST_REG_LSL), F(name##_POST_REG_LSR), F(name##_POST_REG_ASR), F(name##_POST_REG_ROR), F(name##_POST_IMM)
#define F_MEM_HD(name) \
    F(name##_REG), F(name##_IMM), F(name##_POST_REG), F(name##_POST_IMM)
InterpreterFunc InterpretARM[ARMInstrInfo::ak_Count] =
{
    F_ALU(AND,), F_ALU(AND,_S),
    F_ALU(EOR,), F_ALU(EOR,_S),
    F_ALU(SUB,), F_ALU(SUB,_S),
    F_ALU(RSB,), F_ALU(RSB,_S),
    F_ALU(ADD,), F_ALU(ADD,_S),
    F_ALU(ADC,), F_ALU(ADC,_S),
    F_ALU(SBC,), F_ALU(SBC,_S),
    F_ALU(RSC,), F_ALU(RSC,_S),
    F_ALU(ORR,), F_ALU(ORR,_S),
    F_ALU(MOV,), F_ALU(MOV,_S),
    F_ALU(BIC,), F_ALU(BIC,_S),
    F_ALU(MVN,), F_ALU(MVN,_S),
    F_ALU(TST,),
    F_ALU(TEQ,),
    F_ALU(CMP,),
    F_ALU(CMN,),

    F(MUL), F(MLA), F(UMULL), F(UMLAL), F(SMULL), F(SMLAL), F(SMLAxy), F(SMLAWy), F(SMULWy), F(SMLALxy), F(SMULxy),
    F(CLZ), F(QADD), F(QSUB), F(QDADD), F(QDSUB),

    F_MEM_WB(STR),
    F_MEM_WB(STRB),
    F_MEM_WB(LDR),
    F_MEM_WB(LDRB),

    F_MEM_HD(STRH),
    F_MEM_HD(LDRD),
    F_MEM_HD(STRD),
    F_MEM_HD(LDRH),
    F_MEM_HD(LDRSB),
    F_MEM_HD(LDRSH),

    F(SWP), F(SWPB),
    F(LDM), F(STM),

    F(B), F(BL), F(BLX_IMM), F(BX), F(BLX_REG),
    F(UNK), F(MSR_IMM), F(MSR_REG), F(MRS), F(MCR), F(MRC), F(SVC),
    NOP
};
#undef F_ALU
#undef F_MEM_WB
#undef F_MEM_HD
#undef F

void T_BL_LONG(ARM* cpu)
{
    ARMInterpreter::T_BL_LONG_1(cpu);
    cpu->R[15] += 2;
    ARMInterpreter::T_BL_LONG_2(cpu);
}

#define F(x) ARMInterpreter::T_##x
InterpreterFunc InterpretTHUMB[ARMInstrInfo::tk_Count] =
{
    F(LSL_IMM), F(LSR_IMM), F(ASR_IMM),
    F(ADD_REG_), F(SUB_REG_), F(ADD_IMM_), F(SUB_IMM_),
    F(MOV_IMM), F(CMP_IMM), F(ADD_IMM), F(SUB_IMM),
    F(AND_REG), F(EOR_REG), F(LSL_REG), F(LSR_REG), F(ASR_REG),
    F(ADC_REG), F(SBC_REG), F(ROR_REG), F(TST_REG), F(NEG_REG),
    F(CMP_REG), F(CMN_REG), F(ORR_REG), F(MUL_REG), F(BIC_REG), F(MVN_REG),
    F(ADD_HIREG), F(CMP_HIREG), F(MOV_HIREG),
    F(ADD_PCREL), F(ADD_SPREL), F(ADD_SP),
    F(LDR_PCREL), F(STR_REG), F(STRB_REG), F(LDR_REG), F(LDRB_REG), F(STRH_REG),
    F(LDRSB_REG), F(LDRH_REG), F(LDRSH_REG), F(STR_IMM), F(LDR_IMM), F(STRB_IMM),
    F(LDRB_IMM), F(STRH_IMM), F(LDRH_IMM), F(STR_SPREL), F(LDR_SPREL),
    F(PUSH), F(POP), F(LDMIA), F(STMIA),
    F(BCOND), F(BX), F(BLX_REG), F(B), F(BL_LONG_1), F(BL_LONG_2),
    F(UNK), F(SVC),
    T_BL_LONG // BL_LONG psudo opcode
};
#undef F

ARMJIT::ARMJIT(melonDS::NDS& nds, std::optional<JITArgs> jit) noexcept : 
        NDS(nds),
        Memory(nds, (jit.has_value() ? jit->FastMemory : false) && ARMJIT_Memory::IsFastMemSupported()),
        JITCompiler(nds),
        MaxBlockSize(jit.has_value() ? std::clamp(jit->MaxBlockSize, 1u, 32u) : 32),
        LiteralOptimizations(jit.has_value() ? jit->LiteralOptimizations : false),
        BranchOptimizations(jit.has_value() ? jit->BranchOptimizations : false),
        FastMemory((jit.has_value() ? jit->FastMemory : false) && ARMJIT_Memory::IsFastMemSupported())
{}

void ARMJIT::RetireJitBlock(JitBlock* block) noexcept
{
    auto it = RestoreCandidates.find(block->InstrHash);
    if (it != RestoreCandidates.end())
    {
        delete it->second;
        it->second = block;
    }
    else
    {
        RestoreCandidates[block->InstrHash] = block;
    }
}

#ifdef LITEV_JIT_LINK
static inline void LiteV_UpdatePendingPeak(size_t p9, size_t p7) noexcept
{
#if LITEV_PROFILE
    using namespace melonDS::LiteProfile;
    uint64_t p = (uint64_t)(p9 + p7);
    if (p > g_Frame.PendingPeak.load(std::memory_order_relaxed))
        g_Frame.PendingPeak.store(p, std::memory_order_relaxed);
#else
    (void)p9; (void)p7;
#endif
}

void ARMJIT::LinkBlock(JitBlock* block) noexcept
{
    auto& blocks  = block->Num == 0 ? JitBlocks9    : JitBlocks7;
    auto& pending = block->Num == 0 ? PendingLinks9 : PendingLinks7;

    JitEnableWrite();

    // (a) resolve this block's own outgoing links: target compiled -> patch now and
    //     register in the target's incoming list; else park in pending.
    for (int i = 0; i < block->NumOutgoing; i++)
    {
        const OutgoingLink& link = block->Outgoing[i];
        auto it = blocks.find(link.TargetAddr);
        if (it != blocks.end())
        {
            JitBlock* target = it->second;
            u32 targetOff = JITCompiler.SubEntryOffset(target->EntryPoint);
            JITCompiler.PatchLinkSite(link.PatchOffset, targetOff);
            target->Incoming.Add(LinkSite{block->StartAddr, link.PatchOffset});
            LITE_PROFILE_ADD(melonDS::LiteProfile::g_Frame.LinksPatched);
        }
        else
        {
            pending.insert({link.TargetAddr, LinkSite{block->StartAddr, link.PatchOffset}});
        }
    }

    // (b) drain pending links waiting on THIS block's start address.
    u32 entryOff = JITCompiler.SubEntryOffset(block->EntryPoint);
    auto range = pending.equal_range(block->StartAddr);
    for (auto it = range.first; it != range.second; ++it)
    {
        const LinkSite& site = it->second;
        JITCompiler.PatchLinkSite(site.PatchOffset, entryOff);
        block->Incoming.Add(site);
        LITE_PROFILE_ADD(melonDS::LiteProfile::g_Frame.LinksPatched);
    }
    pending.erase(range.first, range.second);

    JitEnableExecute();

    LiteV_UpdatePendingPeak(PendingLinks9.size(), PendingLinks7.size());
}

void ARMJIT::UnlinkBlock(JitBlock* block) noexcept
{
    auto& blocks  = block->Num == 0 ? JitBlocks9    : JitBlocks7;
    auto& pending = block->Num == 0 ? PendingLinks9 : PendingLinks7;

    JitEnableWrite();

    u32 dispOff = JITCompiler.DispatcherRXOffset(block->Num);

    // (a) rewrite every incoming site back to the dispatcher and re-pend it so a
    //     recompile at this StartAddr re-arms the link.
    for (int i = 0; i < block->Incoming.Length; i++)
    {
        LinkSite site = block->Incoming[i];
        JITCompiler.PatchLinkSite(site.PatchOffset, dispOff);
        pending.insert({block->StartAddr, site});
        LITE_PROFILE_ADD(melonDS::LiteProfile::g_Frame.LinksUnlinked);
    }
    block->Incoming.Clear();

    // (b) purge our own outgoing sites. A site is in exactly one place: pending, or
    //     the target's Incoming. Search pending first (this also cleanly removes the
    //     self-link re-pended by (a) when a block links to itself), else the target.
    for (int i = 0; i < block->NumOutgoing; i++)
    {
        const OutgoingLink& link = block->Outgoing[i];

        bool found = false;
        auto range = pending.equal_range(link.TargetAddr);
        for (auto pit = range.first; pit != range.second; ++pit)
        {
            if (pit->second.PatchOffset == link.PatchOffset
                && pit->second.SourceBlockAddr == block->StartAddr)
            {
                pending.erase(pit);
                found = true;
                break;
            }
        }
        if (!found)
        {
            auto it = blocks.find(link.TargetAddr);
            if (it != blocks.end())
            {
                JitBlock* target = it->second;
                for (int k = 0; k < target->Incoming.Length; k++)
                {
                    if (target->Incoming[k].PatchOffset == link.PatchOffset
                        && target->Incoming[k].SourceBlockAddr == block->StartAddr)
                    {
                        target->Incoming.Remove(k);
                        break;
                    }
                }
            }
        }

        // Reset our own slot: our RX is preserved until ResetBlockCache, but a stale
        // B->dead-target must never survive a restore (LinkBlock re-drives it).
        JITCompiler.PatchLinkSite(link.PatchOffset, dispOff);
    }

    JitEnableExecute();

    LiteV_UpdatePendingPeak(PendingLinks9.size(), PendingLinks7.size());
}

#if defined(LITEV_SHADOW_ASSERT)
void ARMJIT::ValidateLinkSites() noexcept
{
    // Build the set of valid branch targets: the two dispatcher entries + every
    // live block's entry offset. Every incoming/outgoing site must currently hold
    // an unconditional B to one of these.
    std::unordered_set<u32> valid;
    valid.insert(JITCompiler.DispatcherRXOffset(0));
    valid.insert(JITCompiler.DispatcherRXOffset(1));
    for (auto& kv : JitBlocks9) valid.insert(JITCompiler.SubEntryOffset(kv.second->EntryPoint));
    for (auto& kv : JitBlocks7) valid.insert(JITCompiler.SubEntryOffset(kv.second->EntryPoint));

    u8* rxbase = JITCompiler.GetRXBase();
    auto checkSite = [&](u32 srcBlock, u32 patchOffset)
    {
        u32 instr = *(const u32*)(rxbase + patchOffset);
        // unconditional B: top 6 bits == 0b000101
        s32 imm = (s32)(instr << 6) >> 4;   // sign-extend imm26, *4
        u32 targetOff = patchOffset + (u32)imm;
        // Use abort() not assert(): the shadow build is Release (NDEBUG), matching
        // the LiteV_ShadowAssertBudget convention.
        if ((instr & 0xFC000000u) != 0x14000000u || valid.count(targetOff) != 1)
        {
            fprintf(stderr,
                "[LITEV_SHADOW_ASSERT] stale link site: srcBlock=%08x off=%x "
                "instr=%08x targetOff=%x valid=%d\n",
                srcBlock, patchOffset, instr, targetOff, (int)valid.count(targetOff));
            fflush(stderr);
            abort();
        }
    };

    auto walk = [&](std::unordered_map<u32, JitBlock*>& map)
    {
        for (auto& kv : map)
        {
            JitBlock* b = kv.second;
            for (int i = 0; i < b->NumOutgoing; i++) checkSite(b->StartAddr, b->Outgoing[i].PatchOffset);
            for (int i = 0; i < b->Incoming.Length; i++) checkSite(b->Incoming[i].SourceBlockAddr, b->Incoming[i].PatchOffset);
        }
    };
    walk(JitBlocks9);
    walk(JitBlocks7);
}
#endif
#endif  // LITEV_JIT_LINK

void ARMJIT::SetJITArgs(JITArgs args) noexcept
{
    args.FastMemory = args.FastMemory && ARMJIT_Memory::IsFastMemSupported();
    args.MaxBlockSize = std::clamp(args.MaxBlockSize, 1u, 32u);

    if (MaxBlockSize != args.MaxBlockSize
        || LiteralOptimizations != args.LiteralOptimizations
        || BranchOptimizations != args.BranchOptimizations
        || FastMemory != args.FastMemory)
        ResetBlockCache();

    // Keep the fastmem fault handler installation in sync with the effective
    // fastmem state (it may be toggled at runtime).
    if (FastMemory != args.FastMemory)
        Memory.SetFastMemHandler(args.FastMemory);

    MaxBlockSize = args.MaxBlockSize;
    LiteralOptimizations = args.LiteralOptimizations;
    BranchOptimizations = args.BranchOptimizations;
    FastMemory = args.FastMemory;
}

void ARMJIT::SetMaxBlockSize(int size) noexcept
{
    SetJITArgs(JITArgs{static_cast<unsigned>(size), LiteralOptimizations, LiteralOptimizations, FastMemory});
}

void ARMJIT::SetLiteralOptimizations(bool enabled) noexcept
{
    SetJITArgs(JITArgs{static_cast<unsigned>(MaxBlockSize), enabled, BranchOptimizations, FastMemory});
}

void ARMJIT::SetBranchOptimizations(bool enabled) noexcept
{
    SetJITArgs(JITArgs{static_cast<unsigned>(MaxBlockSize), LiteralOptimizations, enabled, FastMemory});
}

void ARMJIT::SetFastMemory(bool enabled) noexcept
{
    SetJITArgs(JITArgs{static_cast<unsigned>(MaxBlockSize), LiteralOptimizations, BranchOptimizations, enabled});
}

void ARMJIT::CompileBlock(ARM* cpu) noexcept
{
    bool thumb = cpu->CPSR & 0x20;

    u32 blockAddr = cpu->R[15] - (thumb ? 2 : 4);

    u32 localAddr = LocaliseCodeAddress(cpu->Num, blockAddr);
    if (!localAddr)
    {
        Log(LogLevel::Warn, "trying to compile non executable code? %x\n", blockAddr);
    }

    auto& map = cpu->Num == 0 ? JitBlocks9 : JitBlocks7;
    auto existingBlockIt = map.find(blockAddr);
    if (existingBlockIt != map.end())
    {
        // there's already a block, though it's not inside the fast map
        // could be that there are two blocks at the same physical addr
        // but different mirrors
        u32 otherLocalAddr = existingBlockIt->second->StartAddrLocal;

        if (localAddr == otherLocalAddr)
        {
            JIT_DEBUGPRINT("switching out block %x %x %x\n", localAddr, blockAddr, existingBlockIt->second->StartAddr);

            u64* entry = &FastBlockLookupRegions[localAddr >> 27][(localAddr & 0x7FFFFFF) / 2];
            *entry = ((u64)blockAddr | cpu->Num) << 32;
            *entry |= JITCompiler.SubEntryOffset(existingBlockIt->second->EntryPoint);
            return;
        }

        // some memory has been remapped
#ifdef LITEV_JIT_LINK
        // This block dies here without passing through InvalidateByAddr; unlink it
        // before it leaves JitBlocks so no stale link points into its recycled code.
        UnlinkBlock(existingBlockIt->second);
#endif
        RetireJitBlock(existingBlockIt->second);
        map.erase(existingBlockIt);
    }

    FetchedInstr instrs[MaxBlockSize];
    int i = 0;
    u32 r15 = cpu->R[15];

    u32 addressRanges[MaxBlockSize];
    u32 addressMasks[MaxBlockSize];
    memset(addressMasks, 0, MaxBlockSize * sizeof(u32));
    u32 numAddressRanges = 0;

    u32 numLiterals = 0;
    u32 literalLoadAddrs[MaxBlockSize];
    // they are going to be hashed
    u32 literalValues[MaxBlockSize];
    u32 instrValues[MaxBlockSize];
    // due to instruction merging i might not reflect the amount of actual instructions
    u32 numInstrs = 0;

    u32 writeAddrs[MaxBlockSize];
    u32 numWriteAddrs = 0, writeAddrsTranslated = 0;

    cpu->FillPipeline();
    u32 nextInstr[2] = {cpu->NextInstr[0], cpu->NextInstr[1]};
    u32 nextInstrAddr[2] = {blockAddr, r15};

    JIT_DEBUGPRINT("start block %x %08x (%x)\n", blockAddr, cpu->CPSR, localAddr);

    u32 lastSegmentStart = blockAddr;
    u32 lr;
    bool hasLink = false;

    bool hasMemoryInstr = false;

    do
    {
        r15 += thumb ? 2 : 4;

        instrs[i].BranchFlags = 0;
        instrs[i].SetFlags = 0;
        instrs[i].Instr = nextInstr[0];
        nextInstr[0] = nextInstr[1];

        instrs[i].Addr = nextInstrAddr[0];
        nextInstrAddr[0] = nextInstrAddr[1];
        nextInstrAddr[1] = r15;
        JIT_DEBUGPRINT("instr %08x %x\n", instrs[i].Instr & (thumb ? 0xFFFF : ~0), instrs[i].Addr);

        instrValues[numInstrs++] = instrs[i].Instr;

        u32 translatedAddr = LocaliseCodeAddress(cpu->Num, instrs[i].Addr);
        assert(translatedAddr >> 27);
        u32 translatedAddrRounded = translatedAddr & ~0x1FF;
        if (i == 0 || translatedAddrRounded != addressRanges[numAddressRanges - 1])
        {
            bool returning = false;
            for (u32 j = 0; j < numAddressRanges; j++)
            {
                if (addressRanges[j] == translatedAddrRounded)
                {
                    std::swap(addressRanges[j], addressRanges[numAddressRanges - 1]);
                    std::swap(addressMasks[j], addressMasks[numAddressRanges - 1]);
                    returning = true;
                    break;
                }
            }
            if (!returning)
                addressRanges[numAddressRanges++] = translatedAddrRounded;
        }
        addressMasks[numAddressRanges - 1] |= 1 << ((translatedAddr & 0x1FF) / 16);

        if (cpu->Num == 0)
        {
            ARMv5* cpuv5 = (ARMv5*)cpu;
            if (thumb && r15 & 0x2)
            {
                nextInstr[1] >>= 16;
                instrs[i].CodeCycles = 0;
            }
            else
            {
                nextInstr[1] = cpuv5->CodeRead32(r15, false);
                instrs[i].CodeCycles = cpu->CodeCycles;
            }
        }
        else
        {
            ARMv4* cpuv4 = (ARMv4*)cpu;
            if (thumb)
                nextInstr[1] = cpuv4->CodeRead16(r15);
            else
                nextInstr[1] = cpuv4->CodeRead32(r15);
            instrs[i].CodeCycles = cpu->CodeCycles;
        }
        instrs[i].Info = ARMInstrInfo::Decode(thumb, cpu->Num, instrs[i].Instr, LiteralOptimizations);

        hasMemoryInstr |= thumb
            ? (instrs[i].Info.Kind >= ARMInstrInfo::tk_LDR_PCREL && instrs[i].Info.Kind <= ARMInstrInfo::tk_STMIA)
            : (instrs[i].Info.Kind >= ARMInstrInfo::ak_STR_REG_LSL && instrs[i].Info.Kind <= ARMInstrInfo::ak_STM);

        cpu->R[15] = r15;
        cpu->CurInstr = instrs[i].Instr;
        cpu->CodeCycles = instrs[i].CodeCycles;

        if (instrs[i].Info.DstRegs & (1 << 14)
            || (!thumb
                && (instrs[i].Info.Kind == ARMInstrInfo::ak_MSR_IMM || instrs[i].Info.Kind == ARMInstrInfo::ak_MSR_REG)
                && instrs[i].Instr & (1 << 16)))
            hasLink = false;

        if (thumb)
        {
            InterpretTHUMB[instrs[i].Info.Kind](cpu);
        }
        else
        {
            if (cpu->Num == 0 && instrs[i].Info.Kind == ARMInstrInfo::ak_BLX_IMM)
            {
                ARMInterpreter::A_BLX_IMM(cpu);
            }
            else
            {
                u32 icode = ((instrs[i].Instr >> 4) & 0xF) | ((instrs[i].Instr >> 16) & 0xFF0);
                assert(InterpretARM[instrs[i].Info.Kind] == ARMInterpreter::ARMInstrTable[icode]
                    || instrs[i].Info.Kind == ARMInstrInfo::ak_MOV_REG_LSL_IMM
                    || instrs[i].Info.Kind == ARMInstrInfo::ak_Nop
                    || instrs[i].Info.Kind == ARMInstrInfo::ak_UNK);
                if (cpu->CheckCondition(instrs[i].Cond()))
                    InterpretARM[instrs[i].Info.Kind](cpu);
                else
                    cpu->AddCycles_C();
            }
        }

        instrs[i].DataCycles = cpu->DataCycles;
        instrs[i].DataRegion = cpu->DataRegion;

        u32 literalAddr;
        if (LiteralOptimizations
            && instrs[i].Info.SpecialKind == ARMInstrInfo::special_LoadLiteral
            && DecodeLiteral(thumb, instrs[i], literalAddr))
        {
            u32 translatedAddr = LocaliseCodeAddress(cpu->Num, literalAddr);
            if (!translatedAddr)
            {
                Log(LogLevel::Warn,"literal in non executable memory?\n");
            }
            if (InvalidLiterals.Find(translatedAddr) == -1)
            {
                u32 translatedAddrRounded = translatedAddr & ~0x1FF;

                u32 j = 0;
                for (; j < numAddressRanges; j++)
                    if (addressRanges[j] == translatedAddrRounded)
                        break;
                if (j == numAddressRanges)
                    addressRanges[numAddressRanges++] = translatedAddrRounded;
                addressMasks[j] |= 1 << ((translatedAddr & 0x1FF) / 16);
                JIT_DEBUGPRINT("literal loading %08x %08x %08x %08x\n", literalAddr, translatedAddr, addressMasks[j], addressRanges[j]);
                cpu->DataRead32(literalAddr, &literalValues[numLiterals]);
                literalLoadAddrs[numLiterals++] = translatedAddr;
            }
        }
        else if (instrs[i].Info.SpecialKind == ARMInstrInfo::special_WriteMem)
            writeAddrs[numWriteAddrs++] = instrs[i].DataRegion;
        else if (thumb && instrs[i].Info.Kind == ARMInstrInfo::tk_BL_LONG_2 && i > 0
            && instrs[i - 1].Info.Kind == ARMInstrInfo::tk_BL_LONG_1)
        {
            i--;
            instrs[i].Info.Kind = ARMInstrInfo::tk_BL_LONG;
            instrs[i].Instr = (instrs[i].Instr & 0xFFFF) | (instrs[i + 1].Instr << 16);
            instrs[i].Info.DstRegs = 0xC000;
            instrs[i].Info.SrcRegs = 0;
            instrs[i].Info.EndBlock = true;
            JIT_DEBUGPRINT("merged BL\n");
        }

        if (instrs[i].Info.Branches() && BranchOptimizations
            && instrs[i].Info.Kind != (thumb ? ARMInstrInfo::tk_SVC : ARMInstrInfo::ak_SVC))
        {
            bool hasBranched = cpu->R[15] != r15;

            bool link;
            u32 cond, target, linkAddr;
            bool staticBranch = DecodeBranch(thumb, instrs[i], cond, hasLink, lr, link, linkAddr, target);
            JIT_DEBUGPRINT("branch cond %x target %x (%d)\n", cond, target, hasBranched);

            if (staticBranch)
            {
                instrs[i].BranchFlags |= branch_StaticTarget;

                bool isBackJump = false;
                if (hasBranched)
                {
                    for (int j = 0; j < i; j++)
                    {
                        if (instrs[j].Addr == target)
                        {
                            isBackJump = true;
                            break;
                        }
                    }
                }

                if (cond < 0xE && target < instrs[i].Addr && target >= lastSegmentStart)
                {
                    // we might have an idle loop
                    u32 backwardsOffset = (instrs[i].Addr - target) / (thumb ? 2 : 4);
                    if (IsIdleLoop(thumb, &instrs[i - backwardsOffset], backwardsOffset + 1))
                    {
                        instrs[i].BranchFlags |= branch_IdleBranch;
                        JIT_DEBUGPRINT("found %s idle loop %d in block %08x\n", thumb ? "thumb" : "arm", cpu->Num, blockAddr);
                    }
                }
                else if (hasBranched && !isBackJump && i + 1 < MaxBlockSize)
                {
                    if (link)
                    {
                        lr = linkAddr;
                        hasLink = true;
                    }

                    r15 = target + (thumb ? 2 : 4);
                    assert(r15 == cpu->R[15]);

                    JIT_DEBUGPRINT("block lengthened by static branch (target %x)\n", target);

                    nextInstr[0] = cpu->NextInstr[0];
                    nextInstr[1] = cpu->NextInstr[1];

                    nextInstrAddr[0] = target;
                    nextInstrAddr[1] = r15;

                    lastSegmentStart = target;

                    instrs[i].Info.EndBlock = false;

                    if (cond < 0xE)
                        instrs[i].BranchFlags |= branch_FollowCondTaken;
                }
            }

            if (!hasBranched && cond < 0xE && i + 1 < MaxBlockSize)
            {
                JIT_DEBUGPRINT("block lengthened by untaken branch\n");
                instrs[i].Info.EndBlock = false;
                instrs[i].BranchFlags |= branch_FollowCondNotTaken;
            }
        }

        i++;

        bool canCompile = JITCompiler.CanCompile(thumb, instrs[i - 1].Info.Kind);
        bool secondaryFlagReadCond = !canCompile || (instrs[i - 1].BranchFlags & (branch_FollowCondTaken | branch_FollowCondNotTaken));
        if (instrs[i - 1].Info.ReadFlags != 0 || secondaryFlagReadCond)
            FloodFillSetFlags(instrs, i - 2, !secondaryFlagReadCond ? instrs[i - 1].Info.ReadFlags : 0xF);
    } while(!instrs[i - 1].Info.EndBlock && i < MaxBlockSize && !cpu->Halted && (!cpu->IRQ || (cpu->CPSR & 0x80)));

    if (numLiterals)
    {
        for (u32 j = 0; j < numWriteAddrs; j++)
        {
            u32 translatedAddr = LocaliseCodeAddress(cpu->Num, writeAddrs[j]);
            if (translatedAddr)
            {
                for (u32 k = 0; k < numLiterals; k++)
                {
                    if (literalLoadAddrs[k] == translatedAddr)
                    {
                        if (InvalidLiterals.Find(translatedAddr) == -1)
                            InvalidLiterals.Add(translatedAddr);
                        break;
                    }
                }
            }
        }
    }

    u32 literalHash = (u32)XXH3_64bits(literalValues, numLiterals * 4);
    u32 instrHash = (u32)XXH3_64bits(instrValues, numInstrs * 4);

    auto prevBlockIt = RestoreCandidates.find(instrHash);
    JitBlock* prevBlock = NULL;
    bool mayRestore = true;
    if (prevBlockIt != RestoreCandidates.end())
    {
        prevBlock = prevBlockIt->second;
        RestoreCandidates.erase(prevBlockIt);

        mayRestore = prevBlock->StartAddr == blockAddr && prevBlock->LiteralHash == literalHash;

        if (mayRestore && prevBlock->NumAddresses == numAddressRanges)
        {
            for (u32 j = 0; j < numAddressRanges; j++)
            {
                if (prevBlock->AddressRanges()[j] != addressRanges[j]
                    || prevBlock->AddressMasks()[j] != addressMasks[j])
                {
                    mayRestore = false;
                    break;
                }
            }
        }
        else
            mayRestore = false;
    }
    else
    {
        mayRestore = false;
    }

    JitBlock* block;
    if (!mayRestore)
    {
        if (prevBlock)
            delete prevBlock;

        block = new JitBlock(cpu->Num, i, numAddressRanges, numLiterals);
        block->LiteralHash = literalHash;
        block->InstrHash = instrHash;
        for (u32 j = 0; j < numAddressRanges; j++)
            block->AddressRanges()[j] = addressRanges[j];
        for (u32 j = 0; j < numAddressRanges; j++)
            block->AddressMasks()[j] = addressMasks[j];
        for (int j = 0; j < numLiterals; j++)
            block->Literals()[j] = literalLoadAddrs[j];

        block->StartAddr = blockAddr;
        block->StartAddrLocal = localAddr;

        FloodFillSetFlags(instrs, i - 1, 0xF);

        JitEnableWrite();
        block->EntryPoint = JITCompiler.CompileBlock(cpu, thumb, instrs, i, hasMemoryInstr);
        JitEnableExecute();

#ifdef LITEV_JIT_LINK
        // Carry the compiler's recorded outgoing link sites onto the block (a
        // restored block already has these from its original compile).
        block->NumOutgoing = JITCompiler.NumLinkExits;
        for (int j = 0; j < JITCompiler.NumLinkExits; j++)
            block->Outgoing[j] = JITCompiler.LinkExits[j];
        block->Incoming.Clear();
#endif

        JIT_DEBUGPRINT("block start %p\n", block->EntryPoint);
    }
    else
    {
        JIT_DEBUGPRINT("restored! %p\n", prevBlock);
        block = prevBlock;
    }

    assert((localAddr & 1) == 0);
    for (u32 j = 0; j < numAddressRanges; j++)
    {
        assert(addressRanges[j] == block->AddressRanges()[j]);
        assert(addressMasks[j] == block->AddressMasks()[j]);
        assert(addressMasks[j] != 0);

        AddressRange* region = CodeMemRegions[addressRanges[j] >> 27];

        if (!PageContainsCode(&region[(addressRanges[j] & 0x7FFF000 & ~(Memory.PageSize - 1)) / 512], Memory.PageSize))
            Memory.SetCodeProtection(addressRanges[j] >> 27, addressRanges[j] & 0x7FFFFFF, true);

        AddressRange* range = &region[(addressRanges[j] & 0x7FFFFFF) / 512];
        range->Code |= addressMasks[j];
        range->Blocks.Add(block);
    }

    if (cpu->Num == 0)
        JitBlocks9[blockAddr] = block;
    else
        JitBlocks7[blockAddr] = block;

    u64* entry = &FastBlockLookupRegions[(localAddr >> 27)][(localAddr & 0x7FFFFFF) / 2];
    *entry = ((u64)blockAddr | cpu->Num) << 32;
    *entry |= JITCompiler.SubEntryOffset(block->EntryPoint);

#ifdef LITEV_JIT_LINK
    // Block is now in JitBlocks + FastBlockLookup: resolve its outgoing links and
    // drain any pending links that were waiting for a block at this StartAddr.
    LinkBlock(block);
#endif
}

void ARMJIT::InvalidateByAddr(u32 localAddr) noexcept
{
    JIT_DEBUGPRINT("invalidating by addr %x\n", localAddr);

    AddressRange* region = CodeMemRegions[localAddr >> 27];
    AddressRange* range = &region[(localAddr & 0x7FFFFFF) / 512];
    u32 mask = 1 << ((localAddr & 0x1FF) / 16);

    range->Code = 0;
#ifdef LITEV_JIT_DIRECTPATCH
    bool dpAnyInval = false;
#endif
    for (int i = 0; i < range->Blocks.Length;)
    {
        JitBlock* block = range->Blocks[i];

        bool invalidated = false;
        u32 mask = 0;
        for (int j = 0; j < block->NumAddresses; j++)
        {
            if (block->AddressRanges()[j] == (localAddr & ~0x1FF))
            {
                mask = block->AddressMasks()[j];
                invalidated = block->AddressMasks()[j] & mask;
                assert(mask);
                break;
            }
        }
        assert(mask);
        if (!invalidated)
        {
            range->Code |= mask;
            i++;
            continue;
        }
        range->Blocks.Remove(i);

        if (range->Blocks.Length == 0
            && !PageContainsCode(&region[(localAddr & 0x7FFF000 & ~(Memory.PageSize - 1)) / 512], Memory.PageSize))
        {
            Memory.SetCodeProtection(localAddr >> 27, localAddr & 0x7FFFFFF, false);
        }

        bool literalInvalidation = false;
        for (int j = 0; j < block->NumLiterals; j++)
        {
            u32 addr = block->Literals()[j];
            if (addr == localAddr)
            {
                if (InvalidLiterals.Find(localAddr) == -1)
                {
                    InvalidLiterals.Add(localAddr);
                    JIT_DEBUGPRINT("found invalid literal %d\n", InvalidLiterals.Length);
                }
                literalInvalidation = true;
                break;
            }
        }
        for (int j = 0; j < block->NumAddresses; j++)
        {
            u32 addr = block->AddressRanges()[j];
            if ((addr / 512) != (localAddr / 512))
            {
                AddressRange* otherRegion = CodeMemRegions[addr >> 27];
                AddressRange* otherRange = &otherRegion[(addr & 0x7FFFFFF) / 512];
                assert(otherRange != range);

                bool removed = otherRange->Blocks.RemoveByValue(block);
                assert(removed);

                if (otherRange->Blocks.Length == 0)
                {
                    if (!PageContainsCode(&otherRegion[(addr & 0x7FFF000 & ~(Memory.PageSize - 1)) / 512], Memory.PageSize))
                        Memory.SetCodeProtection(addr >> 27, addr & 0x7FFFFFF, false);

                    otherRange->Code = 0;
                }
            }
        }

#ifdef LITEV_JIT_LINK
        // Unlink BEFORE the block leaves JitBlocks / FastBlockLookup / is retired:
        // rewrite incoming sites back to the dispatcher (+ re-pend) and purge our
        // outgoing sites from targets / pending.
        UnlinkBlock(block);
#endif

        FastBlockLookupRegions[block->StartAddrLocal >> 27][(block->StartAddrLocal & 0x7FFFFFF) / 2] = (u64)UINT32_MAX << 32;
#ifdef LITEV_JIT_ICACHE
        // Same hazard for the per-site inline cache: a block just left FastBlockLookup, so any
        // per-site entry caching a host pointer into it is now stale. Bump the per-CPU epoch ->
        // every existing entry fails its epoch compare in the dispatcher and re-resolves. O(1)
        // whole-cache invalidation (single-block invalidations are ~4/3400f in steady state).
        NDS.ARM9.ICacheEpoch++;
        NDS.ARM7.ICacheEpoch++;
#endif
#ifdef LITEV_JIT_DIRECTPATCH
        // A block just left FastBlockLookup. Its RX is preserved until ResetBlockCache,
        // but any promoted direct edge into it (or out of a co-invalidated source) must
        // be neutralised BEFORE it can execute. Revert-all once after the loop.
        dpAnyInval = true;
#endif
        if (block->Num == 0)
            JitBlocks9.erase(block->StartAddr);
        else
            JitBlocks7.erase(block->StartAddr);

        if (!literalInvalidation)
        {
            RetireJitBlock(block);
        }
        else
        {
            delete block;
        }
    }

#ifdef LITEV_JIT_DIRECTPATCH
    // Neutralise every live promotion once, after all block removals for this address.
    if (dpAnyInval)
        JITCompiler.DirectPatchRevertAll();
#endif

#if defined(LITEV_JIT_LINK) && defined(LITEV_SHADOW_ASSERT)
    // Prove every surviving link site still holds a B to the dispatcher or a live
    // block entry after this invalidation churn.
    ValidateLinkSites();
#endif
}

void ARMJIT::CheckAndInvalidateITCM() noexcept
{
    for (u32 i = 0; i < ITCMPhysicalSize; i+=512)
    {
        if (CodeIndexITCM[i / 512].Code)
        {
            // maybe using bitscan would be better here?
            // The thing is that in densely populated sets
            // The old fashioned way can actually be faster
            for (u32 j = 0; j < 512; j += 16)
            {
                if (CodeIndexITCM[i / 512].Code & (1 << ((j & 0x1FF) / 16)))
                    InvalidateByAddr((i+j) | (ARMJIT_Memory::memregion_ITCM << 27));
            }
        }
    }
}

void ARMJIT::CheckAndInvalidateWVRAM(int bank) noexcept
{
    u32 start = bank == 1 ? 0x20000 : 0;
    for (u32 i = start; i < start+0x20000; i+=512)
    {
        if (CodeIndexARM7WVRAM[i / 512].Code)
        {
            for (u32 j = 0; j < 512; j += 16)
            {
                if (CodeIndexARM7WVRAM[i / 512].Code & (1 << ((j & 0x1FF) / 16)))
                    InvalidateByAddr((i+j) | (ARMJIT_Memory::memregion_VWRAM << 27));
            }
        }
    }
}

JitBlockEntry ARMJIT::LookUpBlock(u32 num, u64* entries, u32 offset, u32 addr) noexcept
{
    u64* entry = &entries[offset / 2];
    if (*entry >> 32 == (addr | num))
        return JITCompiler.AddEntryOffset((u32)*entry);
    return NULL;
}

void ARMJIT::blockSanityCheck(u32 num, u32 blockAddr, JitBlockEntry entry) noexcept
{
    u32 localAddr = LocaliseCodeAddress(num, blockAddr);
    assert(JITCompiler.AddEntryOffset((u32)FastBlockLookupRegions[localAddr >> 27][(localAddr & 0x7FFFFFF) / 2]) == entry);
}

bool ARMJIT::SetupExecutableRegion(u32 num, u32 blockAddr, u64*& entry, u32& start, u32& size) noexcept
{
    // amazingly ignoring the DTCM is the proper behaviour for code fetches
    int region = num == 0
        ? Memory.ClassifyAddress9(blockAddr)
        : Memory.ClassifyAddress7(blockAddr);

    u32 memoryOffset;
    if (FastBlockLookupRegions[region]
        && Memory.GetMirrorLocation(region, num, blockAddr, memoryOffset, start, size))
    {
        //printf("setup exec region %d %d %08x %08x %x %x\n", num, region, blockAddr, start, size, memoryOffset);
        entry = FastBlockLookupRegions[region] + memoryOffset / 2;
        return true;
    }
    return false;
}

template void ARMJIT::CheckAndInvalidate<0, ARMJIT_Memory::memregion_MainRAM>(u32) noexcept;
template void ARMJIT::CheckAndInvalidate<1, ARMJIT_Memory::memregion_MainRAM>(u32) noexcept;
template void ARMJIT::CheckAndInvalidate<0, ARMJIT_Memory::memregion_SharedWRAM>(u32) noexcept;
template void ARMJIT::CheckAndInvalidate<1, ARMJIT_Memory::memregion_SharedWRAM>(u32) noexcept;
template void ARMJIT::CheckAndInvalidate<1, ARMJIT_Memory::memregion_WRAM7>(u32) noexcept;
template void ARMJIT::CheckAndInvalidate<1, ARMJIT_Memory::memregion_VWRAM>(u32) noexcept;
template void ARMJIT::CheckAndInvalidate<0, ARMJIT_Memory::memregion_VRAM>(u32) noexcept;
template void ARMJIT::CheckAndInvalidate<0, ARMJIT_Memory::memregion_ITCM>(u32) noexcept;
template void ARMJIT::CheckAndInvalidate<0, ARMJIT_Memory::memregion_NewSharedWRAM_A>(u32) noexcept;
template void ARMJIT::CheckAndInvalidate<1, ARMJIT_Memory::memregion_NewSharedWRAM_A>(u32) noexcept;
template void ARMJIT::CheckAndInvalidate<0, ARMJIT_Memory::memregion_NewSharedWRAM_B>(u32) noexcept;
template void ARMJIT::CheckAndInvalidate<1, ARMJIT_Memory::memregion_NewSharedWRAM_B>(u32) noexcept;
template void ARMJIT::CheckAndInvalidate<0, ARMJIT_Memory::memregion_NewSharedWRAM_C>(u32) noexcept;
template void ARMJIT::CheckAndInvalidate<1, ARMJIT_Memory::memregion_NewSharedWRAM_C>(u32) noexcept;

void ARMJIT::ResetBlockCache() noexcept
{
    Log(LogLevel::Debug, "Resetting JIT block cache...\n");

    // could be replace through a function which only resets
    // the permissions but we're too lazy
    Memory.Reset();

    InvalidLiterals.Clear();
    for (int i = 0; i < ARMJIT_Memory::memregions_Count; i++)
    {
        if (FastBlockLookupRegions[i])
            memset(FastBlockLookupRegions[i], 0xFF, CodeRegionSizes[i] * sizeof(u64) / 2);
    }
    for (auto it = RestoreCandidates.begin(); it != RestoreCandidates.end(); it++)
        delete it->second;
    RestoreCandidates.clear();
    for (auto it : JitBlocks9)
    {
        JitBlock* block = it.second;
        for (int j = 0; j < block->NumAddresses; j++)
        {
            u32 addr = block->AddressRanges()[j];
            AddressRange* range = &CodeMemRegions[addr >> 27][(addr & 0x7FFFFFF) / 512];
            range->Blocks.Clear();
            range->Code = 0;
        }
        delete block;
    }
    for (auto it : JitBlocks7)
    {
        JitBlock* block = it.second;
        for (int j = 0; j < block->NumAddresses; j++)
        {
            u32 addr = block->AddressRanges()[j];
            AddressRange* range = &CodeMemRegions[addr >> 27][(addr & 0x7FFFFFF) / 512];
            range->Blocks.Clear();
            range->Code = 0;
        }
        delete block;
    }
    JitBlocks9.clear();
    JitBlocks7.clear();

#ifdef LITEV_JIT_LINK
    // Code memory is wiped and the dispatcher regenerated by JITCompiler.Reset();
    // every block (and its Incoming vector) is deleted above, so simply drop all
    // pending links. Both structures must be empty after this.
    PendingLinks9.clear();
    PendingLinks7.clear();
#endif

    JITCompiler.Reset();

#if defined(LITEV_JIT_LINK) && defined(LITEV_SHADOW_ASSERT)
    if (!PendingLinks9.empty() || !PendingLinks7.empty()
        || !JitBlocks9.empty() || !JitBlocks7.empty())
    {
        fprintf(stderr, "[LITEV_SHADOW_ASSERT] link structures not empty after "
                        "ResetBlockCache: pending9=%zu pending7=%zu blocks9=%zu blocks7=%zu\n",
                PendingLinks9.size(), PendingLinks7.size(),
                JitBlocks9.size(), JitBlocks7.size());
        fflush(stderr);
        abort();
    }
#endif
}

void ARMJIT::JitEnableWrite() noexcept
{
    #if defined(__APPLE__) && defined(__aarch64__)
        if (__builtin_available(macOS 11.0, *))
            pthread_jit_write_protect_np(false);
    #elif defined(__NetBSD__)
        mprotect(JITCompiler.CodeMemBase, ARMJIT_Global::CodeMemorySliceSize, PROT_READ | PROT_WRITE);
    #endif
}

void ARMJIT::JitEnableExecute() noexcept
{
    #if defined(__APPLE__) && defined(__aarch64__)
        if (__builtin_available(macOS 11.0, *))
            pthread_jit_write_protect_np(true);
    #elif defined(__NetBSD__)
        mprotect(JITCompiler.CodeMemBase, ARMJIT_Global::CodeMemorySliceSize, PROT_READ | PROT_EXEC);
    #endif
}

}
