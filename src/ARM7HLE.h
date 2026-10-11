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
// Runtime: debug.litev.a7hle (prop on Android, env elsewhere; default on), latched per NDS: 0 off, 1 all, else a
// mask (1 ExChannelMain, 2 SeqMain, 4 MI_CpuCopy32, 8 SND hardware commit).
// ExChannelMain has a variant per driver build: Pokemon B/W/W2 and Mario Kart DS (2005 SDK; the 8-player Netplay
// benchmark: ~22% of its ARM7 guest instructions in the race).
//
// SND_SeqMain (the sequencer: players, tracks, SSEQ bytecode, track->channel parameter update,
// ~20% of the ARM7's guest instructions) runs natively on a copy of the sound work area, the
// shared work and the random state; it commits the changed words only when the whole call
// stayed within what it handles. A note that needs a new channel (bank lookup + allocation)
// falls back to the guest for that call (~4-14% of calls on PW).
//
// The SND hardware commit after a tick (0x03800870: per channel the stop / timer / volume / pan stores to the
// SOUND registers, the shadow bytes, the flags cleared; ~550 guest instructions a call) runs natively when no
// channel starts, with the guest's stack frames and final registers (category B: a fitted cycle estimate).
// SeqMain also has a Mario Kart DS variant (2005 driver in shared WRAM: SeqVar).
//
// Under the JIT the code is verified when the hook block is compiled and the block depends on
// it (no per-call compare); MI_CpuCopy32 from ARM7 WRAM to main RAM copies through host
// pointers with the JIT invalidation check per 16-byte granule instead of two bus calls a word.
//
// Diagnostics (build with LITEV_HLE_DIAG): LITEV_A7HLE_CHECK=1 (env, interpreter mode): run native AND guest, compare everything the
// native path writes at the guest's return, report diffs. LITEV_A7HLE_COMMITCHECK=1: after a
// native sequencer commit, verify memory equals the native result. LITEV_A7HLE_STATS=1: counts.
#pragma once
#ifdef LITEV_A7HLE
#include "types.h"

namespace melonDS { class ARM; class NDS; }

namespace melonDS::A7HLE
{
// JIT decode: is the ARM-mode instruction at addr the entry of a hooked function?
// 0 no, 1 yes (its code verified now: under the JIT this compile-time check is the code check),
// 2 hook site whose code differs now (compile the guest code, still depend on the ranges)
int IsHook(melonDS::NDS& nds, u32 addr, u32 instr);
// guest code the hook at addr depends on: ARMJIT adds these to the hook block's code ranges, so a
// write there invalidates the block and the recompile re-verifies (as for A9HLE)
struct Range { u32 a, b; };
int Deps(u32 addr, u32 instr, Range* out);   // up to 4
// Execute the hooked function at R15-8 (native, or the guest instruction on fallback).
// jit: reached from a JIT-compiled hook (code verified at compile); else compared per call.
// Returns false if cpu is not at a hook (caller does its normal thing).
bool Run(melonDS::ARM* cpu, bool jit);
// interpreter check mode: called before every ARM7 instruction while a check is pending
#ifdef LITEV_HLE_DIAG
extern thread_local bool CheckPending;   // (per thread: headless --mp-test runs a console per thread)
void CheckAt(melonDS::ARM* cpu, u32 pc);
#else
constexpr bool CheckPending = false;   // compare mode compiled out (LITEV_HLE_DIAG)
inline void CheckAt(melonDS::ARM*, u32) {}
#endif
}
#endif
