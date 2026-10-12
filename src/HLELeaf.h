// LITEV_A9HLE: NitroSDK leaf functions called with BL / BLX imm, run natively AT THE CALL SITE (the "BL intrinsic").
//
// An A9HLE hook at a function entry ends the JIT block (the entry decodes as an undefined instruction, A_UNK runs the
// native code, JumpTo(lr), the dispatcher finds the caller's next block): the exit costs more than a short function. For
// a static call the return address is known when the caller's block is compiled, so the JIT decodes the BL itself as a
// non-branching interpreter fallback (ak_HLECALL / tk_HLECALL) to Call(): the native function runs as a plain C++ call
// and the caller's block goes on with the next instruction. The callee's code is verified when the block is compiled and
// the block depends on it (as hooks do). A native that falls back runs the guest function in the interpreter, from the
// BL to its return, inside the call (leaf functions only: no IRQ is taken inside, as on the native path).
//
// The functions (exact code, position independent: found in every game that links them; the census in
// docs/PATH-TO-HLE.md section 2l):
//  - DC_FlushRange / DC_StoreRange / DC_InvalidateRange (NitroSDK data-cache maintenance loops; the cache operations are
//    no-ops under emulation): the loop's final registers and flags, cycles = the guest's (fit per 32-byte line).
//
// Category B (a fitted cycle estimate per call). Runtime: A9HLE mask bit 16777216 (debug.litev.a9hle / LITEV_A9HLE_ONLY).
// Diagnostics (LITEV_HLE_DIAG): LITEV_HLELEAF_CHECK=1 runs the guest instead and compares registers, flags and cycles
// (LEAFFIT lines for the cycle fit); LITEV_HLELEAF_STATS=1 counts.
#pragma once
#ifdef LITEV_A9HLE
#include "types.h"

namespace melonDS { class ARM; class NDS; }

namespace melonDS::HLELeaf
{
// JIT decode of an ARM9 BL / BLX imm (ARM) or the second half of a Thumb BL pair at addr: the leaf function it calls, if
// verified now (0: none). target: the call's destination (bit 0 = Thumb). a/b: the code the block must depend on.
int At(melonDS::NDS& nds, u32 target, u32& a, u32& b);
// ak_HLECALL (ARM BL / BLX imm at R15-8) / tk_HLECALL (Thumb BL pair, this half at R15-4, the first half already set lr)
void CallA(melonDS::ARM* cpu);
void CallT(melonDS::ARM* cpu);
}
#endif
