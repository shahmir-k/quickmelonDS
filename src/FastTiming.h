// SPDX-License-Identifier: GPL-3.0-or-later
// Ported from DSperate (https://github.com/beebono/DSperate, commit f5fa35e),
// src/core/cpu/timing_mode.h and src/core/cpu/cpu_cycles.h.
// Copyright (C) 2026 DSperate contributors.
//
// LITEV_FAST_TIMING: DSperate's "fast" CPU timing model (category B: deterministic,
// version-locked, not the hardware's timing).
//   exact (melonDS)  per-page data costs, the CD/CDI fetch/data overlap, the ARM7 main-RAM
//                    rules, per-target pipeline refills.
//   fast             every CPU data access costs a constant, loads and stores pay
//                    numC + data with no overlap and no region rules, every jump's refill is
//                    a constant. numC (the code fetch) stays exact. DMA and the cartridge keep
//                    their exact timing.
// The ARM9 constant depends on the base register (a translation-time fact): SP-based accesses
// (the stack, in DTCM in nearly every game) cost d9s, all others d9 (DSperate's tuning: one
// constant cannot fit both, see their src/core/cpu/jit/README.md "Fast timing").
//
// Fixed at process start, before the JIT emits its stubs. Runtime switch (same binary):
// prop / env debug.litev.fasttiming = 0 (exact) | 1 (fast, default); constants override
// (tuning knob) debug.litev.fastk = "d9s,d9,d7,r9,r7".
#pragma once
#include "types.h"

namespace melonDS
{
#ifdef LITEV_FAST_TIMING
struct FastTimingCfg { bool On; s32 D9S, D9, D7, R9, R7; };
extern const FastTimingCfg FastTiming;
#define LITEV_FT (melonDS::FastTiming.On)

// DSperate timing_mode.h fast_sp_based(): whether a load/store addresses through SP. ARM: the
// base is bits 19-16 of every load/store encoding. Thumb: only SP-relative LDR/STR and PUSH/POP.
inline bool FastSPBased(u32 instr, bool thumb)
{
    return thumb ? ((instr & 0xF000) == 0x9000 || (instr & 0xF600) == 0xB400) : ((instr >> 16) & 15) == 13;
}
#else
#define LITEV_FT false
#endif
}
