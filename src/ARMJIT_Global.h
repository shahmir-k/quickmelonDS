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

#ifndef ARMJIT_GLOBAL_H
#define ARMJIT_GLOBAL_H

#include "types.h"

#include <stdlib.h>

namespace melonDS
{

namespace ARMJIT_Global
{

static constexpr size_t CodeMemorySliceSize = 1024*1024*32;

void Init();
void DeInit();

// Fastmem fault-handler management. The SIGSEGV/SIGBUS (or Windows vectored)
// handler that services fastmem faults must ONLY be installed while at least
// one JIT instance actually has fastmem enabled. Installing it unconditionally
// (as long as the JIT is on) means it intercepts every process-wide fault even
// when fastmem is off/unsupported, mis-handling unrelated faults (e.g. ART's on
// Android, or a JIT code fault) into a hard crash. These are ref-counted so the
// handler is registered on the first fastmem user and removed with the last.
void AcquireFaultHandler();
void ReleaseFaultHandler();

void* AllocateCodeMem();
void FreeCodeMem(void* codeMem);

}

}

#endif