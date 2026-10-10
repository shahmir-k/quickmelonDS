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

#ifndef ARMJIT_INTERNAL_H
#define ARMJIT_INTERNAL_H

#include "types.h"
#include <stdint.h>
#include <string.h>
#include <assert.h>

#include "ARM_InstrInfo.h"
#include "JitBlock.h"
#include "TinyVector.h"

namespace melonDS
{
class ARM;
class ARMv5;

// here lands everything which doesn't fit into ARMJIT.h
// where it would be included by pretty much everything

enum
{
    branch_IdleBranch = 1 << 0,
    branch_FollowCondTaken = 1 << 1,
    branch_FollowCondNotTaken = 1 << 2,
    branch_StaticTarget = 1 << 3,
    branch_Idle2Cand = 1 << 4, // LITEV_JIT_IDLE2: taken back-edge exits to ARMv5::Idle2Handle
};

struct FetchedInstr
{
    u32 A_Reg(int pos) const
    {
        return (Instr >> pos) & 0xF;
    }

    u32 T_Reg(int pos) const
    {
        return (Instr >> pos) & 0x7;
    }

    u32 Cond() const
    {
        return Instr >> 28;
    }

    u8 BranchFlags;
    u8 SetFlags;
    u32 Instr;
    u32 Addr;

    u8 DataCycles;
#ifdef LITEV_JIT_COND_MEMGUESS
    u8 DataExecuted;   // ran during the compile-time interpretation: DataRegion is its own access
#endif
    u16 CodeCycles;
    u32 DataRegion;

    ARMInstrInfo::Info Info;
};

// size should be 16 bytes because I'm to lazy to use mul and whatnot
struct __attribute__((packed)) AddressRange
{
    TinyVector<JitBlock*> Blocks;
    u32 Code;
};


typedef void (*InterpreterFunc)(ARM* cpu);
extern InterpreterFunc InterpretARM[];
extern InterpreterFunc InterpretTHUMB[];

inline bool PageContainsCode(const AddressRange* range, u32 pageSize)
{
    for (int i = 0; i < pageSize / 512; i++)
    {
        if (range[i].Blocks.Length > 0)
            return true;
    }
    return false;
}

template <typename T, int ConsoleType> T SlowRead9(u32 addr, ARMv5* cpu);
template <typename T, int ConsoleType> void SlowWrite9(u32 addr, ARMv5* cpu, u32 val);
// LITEV_JIT_ARM7_CPUARG: the ARM7 slow helpers take the CPU like the ARM9 ones (W0=addr, X1=cpu,
// W2=val) instead of reading thread_local NDS::Current. Off: (addr) / (addr, val).
#ifdef LITEV_JIT_ARM7_CPUARG
#define ARM7_SLOW_CPU_PARAM , ARM* cpu
#define ARM7_SLOW_NDS (cpu->NDS)
#else
#define ARM7_SLOW_CPU_PARAM
#define ARM7_SLOW_NDS (*NDS::Current)
#endif
template <typename T, int ConsoleType> T SlowRead7(u32 addr ARM7_SLOW_CPU_PARAM);
template <typename T, int ConsoleType> void SlowWrite7(u32 addr ARM7_SLOW_CPU_PARAM, u32 val);

template <bool Write, int ConsoleType> void SlowBlockTransfer9(u32 addr, u64* data, u32 num, ARMv5* cpu);
template <bool Write, int ConsoleType> void SlowBlockTransfer7(u32 addr, u64* data, u32 num);

}

#endif