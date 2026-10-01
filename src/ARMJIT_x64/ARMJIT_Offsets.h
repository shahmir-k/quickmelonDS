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

#define ARM_CPSR_offset 0x64
#define ARM_Cycles_offset 0xc
#define ARM_StopExecution_offset 0x10
// liteDS-v2 Unit 2: slice-budget slot. Hand-maintained like the offsets above;
// proven equal to offsetof(ARM, CyclesBudget) by static_assert in the JIT
// compiler TUs (ARMJIT_A64/ARMJIT_Compiler.cpp, ARMJIT_x64/ARMJIT_Compiler.cpp).
#define ARM_CyclesBudget_offset 0xe8
// liteDS-v2 Unit 3 (LITEV_JIT_DISPATCH): the ARM-base FastBlockLookup* fields the
// emitted A64 dispatcher reads to do the block lookup inline (region bounds + the
// flat u64 tag array + its base pointer). They sit immediately before CyclesBudget
// (0xe0 + 8 == 0xe8), and are proven by static_assert in ARMJIT_A64/ARMJIT_Compiler.cpp.
#define ARM_FastBlockLookupStart_offset 0xd8
#define ARM_FastBlockLookupSize_offset  0xdc
#define ARM_FastBlockLookup_offset      0xe0

