// LITEV_A7HLE: high-level emulation of hot NitroSDK ARM7 sound-driver functions.
//
// SND_ExChannelMain (the per-tick envelope / LFO / sweep / volume / pitch / pan update of the 16
// sound channels, ~34% of the ARM7's guest instructions in the Pokemon White overworld) runs as
// native C++ that writes the same bytes into the same SNDExChannel structs. Detected by a code
// signature (exact bytes of the function, its callees and the BIOS-call stubs); anything the
// native version doesn't cover (a channel callback, a DSi-BIOS workaround) makes it fall back
// to the guest code for that call.
//
// Category B: guest memory results are identical, but the call takes a fixed estimated number
// of ARM7 cycles instead of the real count, scratch registers/stack bytes are not written and
// the channels' hardware-busy bits are sampled at entry. Deterministic and version-locked.
// Runtime: debug.litev.a7hle (prop on Android, env elsewhere; default on), latched per NDS.
//
// SND_SeqMain (the sequencer: players, tracks, SSEQ bytecode, track->channel parameter update,
// ~20% of the ARM7's guest instructions) runs natively on a copy of the sound work area, the
// shared work and the random state; it commits the changed words only when the whole call
// stayed within what it handles. A note that needs a new channel (bank lookup + allocation)
// falls back to the guest for that call (~4-14% of calls on PW).
//
// LITEV_A7HLE_CHECK=1 (env, interpreter mode): run native AND guest, compare everything the
// native path writes at the guest's return, report diffs. LITEV_A7HLE_COMMITCHECK=1: after a
// native sequencer commit, verify memory equals the native result. LITEV_A7HLE_STATS=1: counts.
#pragma once
#ifdef LITEV_A7HLE
#include "types.h"

namespace melonDS { class ARM; class NDS; }

namespace melonDS::A7HLE
{
// JIT decode / interpreter: is the ARM-mode instruction at addr the entry of a hooked function?
bool IsHook(melonDS::NDS& nds, u32 addr, u32 instr);
// Execute the hooked function at R15-8 (native, or the guest instruction on fallback).
// Returns false if cpu is not at a hook (caller does its normal thing).
bool Run(melonDS::ARM* cpu);
// interpreter check mode: called before every ARM7 instruction while a check is pending
extern bool CheckPending;
void CheckAt(melonDS::ARM* cpu, u32 pc);
}
#endif
