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
// Runtime: debug.litev.a9hle (prop on Android, env elsewhere; default on), latched per NDS:
// 0 off, 1 all, other values = the mask below.
// Env LITEV_A9HLE_ONLY=<mask> (1 wake, 2 set, 4 get, 8 HBlank IRQ, 16 GX send, 32 card read, 64 LZ, 128 G3D material,
// 256 _ll_sdiv, 512 GX async start, 1024 GX DMA-end IRQ,
// 2048 G3D shape (needs 512); 8 needs 1)
// for A/B of single hooks.
// Hooks 1-5 are keyed to a per-game Variant (ARM9HLE.cpp: Pokemon White, Pokemon Black, Pokemon White 2),
// probed when a hook entry of that variant is first reached; hook 6 is position independent (any game).
// W2 (TWL SDK build) has the same IRQ handler / context switch code but its thread functions and
// OS_Set/GetIrqFunction / MIi_FIFOCallback are Thumb: hooks 1-5 (2, 4, 5 Thumb entries).
//
// 3. Whole HBlank IRQs (~265 a frame on PW, ~3.8k Mac host instructions each through the JIT
//    even with 1. native; ~1.7k native): when the only pending enabled IRQ is HBlank and the game's HBlank callback
//    list is empty, the IRQ is taken natively at delivery: BIOS frame, OS_IrqHandler, the IF
//    acknowledge, the (empty) callback, then either the empty-queue return or the wake-skip of
//    1., then the BIOS return. Nothing changes in the interrupted registers; guest memory and the
//    banked IRQ/SVC registers get what the guest path leaves. When the ARM9 was halted in the
//    OS idle loop it stays halted. Code verified at JIT compile of the wake hook block (which
//    depends on all of it: ARMv5::A9HLEGuard), BIOS once; anything else runs the guest path.
//
// 5. MI_SendGXCommandAsync display lists (with LITEV_GX_BULK): MIi_FIFOCallback sends a list in
//    118-word immediate DMAs, the next one from the GXFIFO "less than half" IRQ; with the bulk
//    geometry path the FIFO is empty again at once, so every chunk costs an IRQ (100-200 a frame
//    in PW's 3D scenes). At MIi_FIFOCallback's entry with the FIFO empty, all chunks but the last
//    go through GPU3D::BulkWords at once and MIi_GXDmaParams.src/length move past them; the guest
//    code then sends the last chunk as before. Category B: DMA cycles as the DMA's, plus a fixed
//    estimate per elided IRQ. Check mode (build with LITEV_A9HLE_GXCHECK) compares the words
//    the guest sends to GXFIFO, and src/length where the native path would leave them.
//
// 6. MIi_UncompressBackward (backward LZ, overlay/data unpacking in loading): the loop natively in
//    256-byte chunks with the guest's exact registers at each stop (see ARM9HLE.cpp).
// 7. NitroSDK CARD CPU read loop: one ROM page natively (see ARM9HLE.cpp).
// 8. NNS G3D material (NNSi_G3dFuncSbc_MAT + MAT_InternalDefault + NNS_G3dGeBufferOP_N + MI_CpuSend32):
//    the material result, the stack/state writes and the 7 GXFIFO words natively, guest fallback on any
//    callback / cache hit / buffered geometry (position independent, literal-pool globals read at the call).
//    W2 (TWL SDK build, Thumb NNS): the same at SBC MAT's Thumb entry (variant code, exact bytes).
// 9. _ll_sdiv (64-bit signed divide of the compiler runtime): native quotient with the guest's exact
//    registers / flags / stack bytes (position independent).
// 10. MI_SendGXCommandAsync's synchronous part (PW, PB; W2: Thumb, TWL SDK build): the same IO writes in the same order, every memory
//    byte and register up to its return; the display list's DMA-end IRQ stays guest code.
// 11. That DMA-end IRQ (PW, PB, W2: Thumb) natively at delivery like 3.: OS_IrqHandler -> OSi_IrqCallback -> MIi_DMACallback
//    (OS_DisableIrqMask, GXSTAT, OS_SetIrqFunction, busy = 0, NNS G3D's "[arg] = 0" callback) with the same IO writes
//    in order, the IRQ stack bytes and the banked IRQ registers.
// 13. NNS G3D shape (PW, PB): SBC SHP -> SHP_InternalDefault -> NNS_G3dGeSendDL natively at SHP's entry: a list of
//    >= 0x100 bytes with 10.'s work (from the register file the guest has at MI_SendGXCommandAsync), a small one as
//    NNS_G3dGeBufferOP_N's direct GXFIFO send (BulkWords); the frames, flags and registers up to SHP's return.
//
// Diagnostics (build with LITEV_HLE_DIAG; compiled out of shipping builds):
//
// LITEV_A9HLE_CHECK=1 (env, interpreter mode): compute the native result, run the guest code
// instead, and at the guest's return compare all of main RAM, ITCM, DTCM and the registers
// with the native prediction. LITEV_A9HLE_STATS=1: counts, host ns per native call (and measured
// guest cycles in check). LITEV_A9HLE_DRY=1: compute the native result but run the guest code
// (cost measurement: dry - off = native path cost, dry - on = guest round trip cost);
// LITEV_A9HLE_DRYIRQ=1 the same for 3. only.
#pragma once
#ifdef LITEV_A9HLE
#include "types.h"
#ifdef LITEV_A9HLE_GXCHECK
#include <vector>
#endif

namespace melonDS { class ARM; class ARMv5; class NDS; }

namespace melonDS::A9HLE
{
// first instruction words of the hooked entries (cheap pre-filter for the interpreter)
inline bool MaybeHook(u32 instr) { return instr == 0xE58C2064 || instr == 0xE92D47F0 || instr == 0xE59F207C || instr == 0xE92D40F8 || instr == 0xE1530001 || instr == 0xE5942000
                                        || instr == 0xE92D4010 || instr == 0xE92D58F0; }
// Thumb entries (W2 OS_SetIrqFunction push {r4-r7} / OS_GetIrqFunction push {r3, r4} / MIi_FIFOCallback
// push {r3-r7, lr} / SBC MAT push {r4-r6, lr}); instr: the halfword
inline bool MaybeHookT(u32 instr) { instr &= 0xFFFF; return instr == 0xB4F0 || instr == 0xB418 || instr == 0xB5F8 || instr == 0xB570; }
// JIT decode: is the instruction at addr (ARM, or Thumb with thumb set) a hooked entry?
// 0 no, 1 yes (code signature verified: under the JIT this compile-time check is the code
// check), 2 hook site whose code differs now (compile the guest code, but still depend on the
// ranges so restoring the code re-enables the hook)
int IsHook(melonDS::NDS& nds, u32 addr, u32 instr, bool thumb = false);
// guest code the hook at addr depends on (the wake hook: also all of 3.): ARMJIT adds these to the hook block's code ranges,
// so any write there invalidates the block (and the next compile re-verifies)
struct Range { u32 a, b; };
constexpr int kNumCode = 17;
int Deps(melonDS::NDS& nds, u32 addr, u32 instr, const Range*& r);
// Execute the hook at R15-8 (R15-4 in Thumb) (native, or the guest instruction on fallback). jit: reached from a
// JIT-compiled hook (code already verified); else the code is compared per call.
// Returns false if cpu is not at a hook (caller does its normal thing).
bool Run(melonDS::ARM* cpu, bool jit);
// IRQ delivery to the ARM9 (TriggerIRQ, IRQs enabled). halted: the ARM9 is waking from a halt.
// true: taken natively (registers unchanged, Cycles added); the caller then delivers any IRQ that
// is still pending as usual (and a halted ARM9 in the OS idle loop stays halted if none is).
bool Irq(melonDS::ARMv5* c, bool halted);
// IRQ delivery (TriggerIRQ) while an ARM9 DMA is about to stop the CPU (Halted == 2): true = deliver it after the
// DMA instead (A9HLEDefer; the guest enters the vector now but runs no instruction before the DMA, so its handler
// sees the IF bits the DMA raised). Used for the GXFIFO IRQ of 11.: after the list's DMA its end IRQ is the lowest
// pending one and is taken natively.
bool Defer(melonDS::ARMv5* c);
// ARMJIT: a block compiled with the wake hook (code verified, IsHook == 1) at addr
void HookCompiled(melonDS::NDS& nds, u32 addr, const void* block);
// ARMJIT: a block is leaving the JIT (invalidated, replaced or deleted)
void BlockGone(melonDS::NDS& nds, const void* block);
// interpreter check mode: called before every ARM9 instruction while a check is pending
// (compiled out without LITEV_HLE_DIAG)
#ifdef LITEV_HLE_DIAG
extern bool CheckPending;
void CheckAt(melonDS::ARM* cpu, u32 pc);
#else
constexpr bool CheckPending = false;
inline void CheckAt(melonDS::ARM*, u32) {}
#endif
#ifdef LITEV_A9HLE_GXCHECK
// check mode of 5. (diagnostic build): words written to GXFIFO / other geometry command writes
extern std::vector<u32>* GxTap;
extern bool GxOtherSeen;
#endif
}
#endif
