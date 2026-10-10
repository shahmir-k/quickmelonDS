// LITEV_A9HLE: high-level emulation of hot NitroSDK ARM9 OS functions in Pokemon Black/White.
//
// 1. Spurious OS_WaitIrq wake-ups. NitroSDK's IRQ handler wakes every thread waiting in
//    OS_WaitIrq on ANY interrupt. PW's main thread waits for VBlank with HBlank IRQs on, so
//    ~220 times a frame the handler switches to it (save idle-thread context, CP context,
//    load main-thread context), the thread finds its flag clear, goes back to sleep and
//    switches back to the idle thread: 25-50% of all ARM9 guest instructions on PW. When the
//    woken thread provably goes straight back to sleep, the hook in OS_IrqHandler_ThreadSwitch's
//    wake loop writes everything that round trip would leave in memory and
//    the banked registers (both thread contexts, the IRQ/SVC/thread stack bytes, banked
//    LR/SPSR) and returns from the IRQ to the interrupted thread.
// 2. OS_SetIrqFunction / OS_GetIrqFunction (32-iteration bit loops, up to 17% of ARM9 guest
//    instructions in PW towns): native, same memory writes, registers and flags. Small host win
//    (-0.5% emu-thread instructions in town): the JIT already runs these ALU loops cheaply.
//
// Detected by a code signature (exact bytes of every function involved). Under the JIT the code
// is checked when the hook block is compiled and the block depends on all those bytes (a write
// there invalidates it, the recompile re-checks); the interpreter compares per call. Anything
// outside the handled case runs the guest code. The native path reads/writes guest memory
// through host pointers and writes only words that change (the in-order A55 pays for every
// extra cache line and call).
// Category B: guest memory and registers are what the guest code would leave; the call takes
// a fixed estimated number of ARM9 cycles instead of the real count (and an IRQ that would
// have arrived inside the elided round trip is taken right after it). Deterministic.
// Runtime: debug.litev.a9hle (prop on Android, env elsewhere; default on), latched per NDS.
// Env LITEV_A9HLE_ONLY=<mask> (1 wake, 2 set, 4 get) for A/B of single hooks.
//
// LITEV_A9HLE_CHECK=1 (env, interpreter mode): compute the native result, run the guest code
// instead, and at the guest's return compare all of main RAM, ITCM, DTCM and the registers
// with the native prediction. LITEV_A9HLE_STATS=1: counts, host ns per native call (and measured
// guest cycles in check). LITEV_A9HLE_DRY=1: compute the native result but run the guest code
// (cost measurement: dry - off = native path cost, dry - on = guest round trip cost).
#pragma once
#ifdef LITEV_A9HLE
#include "types.h"

namespace melonDS { class ARM; class NDS; }

namespace melonDS::A9HLE
{
// first instruction words of the hooked entries (cheap pre-filter for the interpreter)
inline bool MaybeHook(u32 instr) { return instr == 0xE58C2064 || instr == 0xE92D47F0 || instr == 0xE59F207C; }
// JIT decode: is the ARM-mode instruction at addr a hooked entry?
// 0 no, 1 yes (code signature verified: under the JIT this compile-time check is the code
// check), 2 hook site whose code differs now (compile the guest code, but still depend on the
// ranges so restoring the code re-enables the hook)
int IsHook(melonDS::NDS& nds, u32 addr, u32 instr);
// guest code the hook at addr depends on: ARMJIT adds these to the hook block's code ranges,
// so any write there invalidates the block (and the next compile re-verifies)
struct Range { u32 a, b; };
constexpr int kNumCode = 12;
int Deps(u32 addr, const Range*& r);
// Execute the hook at R15-8 (native, or the guest instruction on fallback). jit: reached from a
// JIT-compiled hook (code already verified); else the code is compared per call.
// Returns false if cpu is not at a hook (caller does its normal thing).
bool Run(melonDS::ARM* cpu, bool jit);
// interpreter check mode: called before every ARM9 instruction while a check is pending
extern bool CheckPending;
void CheckAt(melonDS::ARM* cpu, u32 pc);
}
#endif
