// LITEV_A7HLE: the native ARM7 OS layer (the standard NitroSDK / TwlSDK ARM7 component, per SDK build).
//
// The ARM7's OS functions run as C++ at their call sites (the BL intrinsic, as HLELeaf on the ARM9): OS_GetTick,
// OSi_SetTimer, OSi_InsertAlarm (the alarm list driven by TM1, with the 64-bit period arithmetic). Every function is found
// by its signature (tools/hle/a7sig.py: the function's whole code with call offsets and the game's own data pointers
// masked, its callees checked the same way), so every game with that SDK build gets it, at any address; the data
// addresses (tick counter, alarm queue, IRQ callback table, ...) come from the game's own literal pools.
// All state stays in the guest's own structures: savestates keep working and the native and guest paths can switch at
// any instruction. The Wi-Fi (WM) driver is not touched (it stays guest code).
//
// Category B: guest memory and the hardware register writes are the guest's; the call takes a fitted cycle estimate,
// the IO accesses happen at the call (JIT) or at fitted offsets from it (interpreter), scratch registers r2, r3, r12 and
// the flags are not written (dead after a call), the stack bytes below sp are not written.
// Runtime: debug.litev.a7hle (unset or 1: all on) bit 32 the OS call natives, bit 64 the IRQ path (Irq / IsHook / Hook), 128 the
// SDK alarm handlers in it, 256 whole runs only (the TM1 alarm IRQ to the BIOS return or the guest), 512 that run as one function
// (see Mask()).
// Diagnostics (LITEV_HLE_DIAG): LITEV_A7N_CHECK=1 in --mode interp: native result computed, the guest runs, compared at its
// return (memory, return registers, timer / IE state); LITEV_A7N_FIT=1 prints the guest cycles per call (cycle fits).
// LITEV_A7HLE_STATS=1: per-kind call counts.
#pragma once
#ifdef LITEV_A7HLE
#include "types.h"

namespace melonDS { class ARM; class NDS; }

namespace melonDS::A7Native
{
// JIT: a call (ARM BL / Thumb BL pair) to target is compiled as a native call when target is a native OS function of a
// known SDK build: its code ranges go to a / b (the block depends on them); returns their count, 0: an ordinary call
int At(melonDS::NDS& nds, u32 target, u32* a, u32* b, int max);
// ak_HLECALL (ARM BL at R15-8) / tk_HLECALL (Thumb BL pair, this half at R15-4) on the ARM7
void CallA(melonDS::ARM* cpu);
void CallT(melonDS::ARM* cpu);
// interpreter: the current instruction is an ARM BL (thumb: the second half of a BL pair) whose condition passed;
// true if it ran natively (PC is past the call as after a non-branching instruction)
bool Interp(melonDS::ARM* cpu, bool thumb);
// The IRQ path: ARM::TriggerIRQ (the ARM7 at its BIOS IRQ vector) steps the OS IRQ path natively (Irq); the instructions guest
// functions return to on that path (the thread switch, the IRQ callback dispatcher's and OSi_ArrangeTimer's post-call
// instruction) are JIT hooks (IsHook at compile: their code ranges into a / b; Hook at run: false if not one)
void Irq(melonDS::ARM* cpu);
// bit 2048: ARM::TriggerIRQ before the vector entry (interrupted mode USR / SYS): the TM1 alarm IRQ natively from the
// interrupted state to the state after the BIOS return (no entry / return mode switches, no vector fetch); true if applied
bool IrqPre(melonDS::ARM* cpu);
bool MaybeExit(u32 instr);
int IsHook(melonDS::NDS& nds, u32 addr, u32 instr, u32* a, u32* b, int max);
bool Hook(melonDS::ARM* cpu, bool jit);
// bit 256 (whole runs): the hook blocks compile the real instruction (they only guard the functions the native run uses)
bool HookGuardOnly(melonDS::NDS& nds);
// ARMJIT: an exit hook block was compiled (it guards the functions it depends on) / a block is gone / the cache reset
void HookCompiled(melonDS::NDS& nds, u32 addr, const void* block);
void BlockGone(melonDS::NDS& nds, const void* block);
void GuardsReset(melonDS::NDS& nds);
#ifdef LITEV_HLE_DIAG
extern thread_local bool CheckPending;
void CheckAt(melonDS::ARM* cpu, u32 pc);
#else
constexpr bool CheckPending = false;
inline void CheckAt(melonDS::ARM*, u32) {}
#endif
}
#endif
