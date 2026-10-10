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
// Detected by a code signature (exact bytes of every function involved); the code is
// re-checked per call; anything outside the handled case runs the guest code.
// Category B: guest memory and registers are what the guest code would leave; the call takes
// a fixed estimated number of ARM9 cycles instead of the real count (and an IRQ that would
// have arrived inside the elided round trip is taken right after it). Deterministic.
// Runtime: debug.litev.a9hle (prop on Android, env elsewhere; default on), latched per NDS.
// Env LITEV_A9HLE_ONLY=<mask> (1 wake, 2 set, 4 get) for A/B of single hooks.
//
// LITEV_A9HLE_CHECK=1 (env, interpreter mode): compute the native result, run the guest code
// instead, and at the guest's return compare all of main RAM, ITCM, DTCM and the registers
// with the native prediction. LITEV_A9HLE_STATS=1: counts (and measured guest cycles in check).
#pragma once
#ifdef LITEV_A9HLE
#include "types.h"

namespace melonDS { class ARM; class NDS; }

namespace melonDS::A9HLE
{
// first instruction words of the hooked entries (cheap pre-filter for the interpreter)
inline bool MaybeHook(u32 instr) { return instr == 0xE58C2064 || instr == 0xE92D47F0 || instr == 0xE59F207C; }
// JIT decode: is the ARM-mode instruction at addr a hooked entry?
bool IsHook(melonDS::NDS& nds, u32 addr, u32 instr);
// Execute the hook at R15-8 (native, or the guest instruction on fallback).
// Returns false if cpu is not at a hook (caller does its normal thing).
bool Run(melonDS::ARM* cpu);
// interpreter check mode: called before every ARM9 instruction while a check is pending
extern bool CheckPending;
void CheckAt(melonDS::ARM* cpu, u32 pc);
}
#endif
