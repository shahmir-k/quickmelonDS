# LITEV optimizations

This fork adds a set of compile-time `LITEV_*` options to melonDS, aimed at the Anbernic RG DS (Rockchip RK3566, four in-order Cortex-A55 cores, Android 14). Every option defaults to OFF, so a build with no `-DLITEV_*` flags is upstream melonDS plus inert scaffolding. The Android app turns the shipping set on in its `build.gradle.kts`. Each option was added by exactly one commit; that commit message carries the same text as the section below.

See also: [NEGATIVE-RESULTS.md](NEGATIVE-RESULTS.md) (levers that were tried and dropped), [BUILD-METHODOLOGY.md](BUILD-METHODOLOGY.md), [TESTING-METHODOLOGY.md](TESTING-METHODOLOGY.md) and [ANALYSIS-METHODOLOGY.md](ANALYSIS-METHODOLOGY.md).

## Conventions

- **Exactness.** **A**: guest state and cycle timing are byte-identical to the build
  without the option. **B**: a deterministic timing relaxation (not byte-identical, but
  reproducible, MP-safe and version-locked). **R**: emulation is identical; only how the
  local framebuffer or audio output is produced changes (threading, approximation).
- **Measured (historical)** numbers come from the development log and are tied to the stack and
  date they were taken on. Numbers from different eras (GL renderer, software renderer at a fixed
  1800 MHz, tile renderer) are not comparable with each other. "Headless" means the
  `liteDS-headless` harness (`tools/headless`), "app" means the Android `.dev` APK on the RG DS in
  the Shrek 8-kart race. UNMEASURED means no credible isolated number exists.
- The "Measured (cumulative, this history)" line in each commit message is filled in by a later
  on-device pass over this history.

## Summary

| Commit | Option(s) | Exactness |
|---|---|---|
| build: add LITEV option scaffold and re-export LITEV_* defines on core | (unflagged) | A |
| android: port the OpenGL renderer to GLES3 and fix Mali attribute signedness | (unflagged) | A |
| jit: install the fastmem fault handler only when fastmem is in use | (unflagged) | A |
| diag: add LiteProfile frame-breakdown counters (LITEV_PROFILE) | `LITEV_PROFILE` | A |
| diag: add LitevSoftProf software-render phase profiler (LITEV_SOFTPROF) | `LITEV_SOFTPROF` | A |
| tools: add the liteDS-headless benchmark and byte-exact oracle harness | `LITEV_HEADLESS` | A |
| jit: add the CyclesBudget slice slot and ForceExecutionExit (LITEV_SHADOW_ASSERT) | `LITEV_SHADOW_ASSERT` | A |
| jit: emit an A64 block dispatcher that stays in generated code (LITEV_JIT_DISPATCH) | `LITEV_JIT_DISPATCH` | A |
| jit: link static block exits directly to their successors (LITEV_LINK_*) | `LITEV_LINK_UNCOND`, `LITEV_LINK_COND`, `LITEV_LINK_FALLTHROUGH` | A |
| sched: run each CPU slice to the true next event (LITEV_EVENT_SLICES) | `LITEV_EVENT_SLICES` | B |
| jit: inline DTCM block transfers and MainRAM u32 loads (LITEV_MEM_DTCM_BLOCK, LITEV_MEM_MAINRAM_LOAD) | `LITEV_MEM_DTCM_BLOCK`, `LITEV_MEM_MAINRAM_LOAD` | A |
| jit: evaluate compound guest conditions with native NZCV (LITEV_JIT_CONDFOLD) | `LITEV_JIT_CONDFOLD` | A |
| jit: keep guest NZCV resident in host flags (LITEV_JIT_FIXEDREG) | `LITEV_JIT_FIXEDREG` | A |
| jit: pin never-banked guest registers to host registers across blocks (LITEV_JIT_GLOBALREG) | `LITEV_JIT_GLOBALREG` | A |
| jit: delete the reserved CPSR register and keep NZCV lazy (LITEV_JIT_LAZYFLAGS) | `LITEV_JIT_LAZYFLAGS` | A |
| jit: add a per-site 2-way inline cache to the dispatcher (LITEV_JIT_ICACHE) | `LITEV_JIT_ICACHE` | A |
| jit: patch monomorphic dynamic exits into guarded direct branches (LITEV_JIT_DIRECTPATCH) | `LITEV_JIT_DIRECTPATCH` | A |
| jit: emit a simpleperf perf map for generated code (LITEV_JIT_PERFMAP) | `LITEV_JIT_PERFMAP` | A |
| jit: pair LDM/STM inline copies with LDP/STP and inline MainRAM block loads (LITEV_JIT_LDMSTM) | `LITEV_JIT_LDMSTM` | A |
| jit: merge host NZCV into guest CPSR with one MRS (LITEV_JIT_FLAGMERGE) | `LITEV_JIT_FLAGMERGE` | A |
| jit: classify TCM block transfers once in SlowBlockTransfer9 (LITEV_JIT_BLOCKXFER_FAST) | `LITEV_JIT_BLOCKXFER_FAST` | A |
| mem: map DTCM into the fastmem window (LITEV_MEM_DTCM_FASTMEM) | `LITEV_MEM_DTCM_FASTMEM` | A |
| io: dispatch ARM9 32-bit I/O through a dense jump table (LITEV_IO_DISPATCH_TABLE) | `LITEV_IO_DISPATCH_TABLE` | A |
| build: compile with -fno-plt (LITEV_LINKOPT) | `LITEV_LINKOPT` | A |
| jit: pass the ARM* to slow-memory and region helpers | (unflagged) | A |
| diag: histogram slow-helper calls by memory region (LITEV_SLOWMEM_HIST) | `LITEV_SLOWMEM_HIST` | A |
| sched: complete ARM9 DIV/SQRT on write (LITEV_INSTANT_DIVSQRT) | `LITEV_INSTANT_DIVSQRT` | B |
| spu: mix N samples per scheduled SPU event (LITEV_SPU_BATCH) | `LITEV_SPU_BATCH` | B |
| rtc: skip inert 32768 Hz ticks when no RTC interrupt is armed (LITEV_COARSE_RTC) | `LITEV_COARSE_RTC` | B |
| sched: cache the next timer-overflow deadline (LITEV_TIMER_FAST) | `LITEV_TIMER_FAST` | A |
| jit: fast-forward register-recurrent poll loops (LITEV_IDLE_AGGRESSIVE) | `LITEV_IDLE_AGGRESSIVE` | B |
| dma: load ARM9 32-bit DMA unit timings lazily (LITEV_DMA_TIMING_LAZY) | `LITEV_DMA_TIMING_LAZY` | A |
| spu: replace channel interpolation with linear interpolation (LITEV_SPU_FAST_INTERP) | `LITEV_SPU_FAST_INTERP` | R |
| spu: decode then mix, skip silent channels, NEON pan-accumulate (LITEV_SPU_MIX_NEON) | `LITEV_SPU_MIX_NEON` | A |
| fifo: wrap power-of-two FIFO indices with a mask | (unflagged) | A |
| gx: drain GX command batches in a threaded interpreter loop (LITEV_GXFIFO_THREADED) | `LITEV_GXFIFO_THREADED` | A |
| dma: feed geometry DMA straight from MainRAM into the GXFIFO (LITEV_DMA_GXFIFO_FAST) | `LITEV_DMA_GXFIFO_FAST` | A |
| gx: inline the GXFIFO producer into the geometry-DMA loop (LITEV_GXFIFO_DMA_INLINE) | `LITEV_GXFIFO_DMA_INLINE` | A |
| gx: touch the GXFIFO IRQ line only when its state changes | (unflagged) | A |
| geom: skip the clipper for polygons fully inside the frustum | (unflagged) | A |
| geom: integer-NEON vertex transform and lighting math (LITEV_NEON_GEOMETRY) | `LITEV_NEON_GEOMETRY` | A |
| geom: NEON backface cull and 4x3/3x3 matrix multiply (LITEV_GEOM_NEON2) | `LITEV_GEOM_NEON2` | A |
| geom: NEON box/pos/vec tests and matrix scale/translate (LITEV_GEOM_NEON3) | `LITEV_GEOM_NEON3` | A |
| gx: sort polygons with an allocation-free radix sort (LITEV_POLY_RADIX) | `LITEV_POLY_RADIX` | A |
| 2d: NEON master-brightness and colour expansion (LITEV_NEON_RENDERER) | `LITEV_NEON_RENDERER` | A |
| gpu: add a per-frame rasterization skip gate (LITEV_AGGRESSIVE_SKIP) | `LITEV_AGGRESSIVE_SKIP` | A |
| 2d: rasterize software 2D off the emulator thread (LITEV_SOFT2D_THREADED) | `LITEV_SOFT2D_THREADED` | R |
| 2d: double-buffer 2D snapshots so the emulator runs a frame ahead (LITEV_SOFT2D_DEPTH2) | `LITEV_SOFT2D_DEPTH2` | R |
| 2d: NEON BG/OBJ colour-effect compositor (LITEV_SOFT2D_NEON) | `LITEV_SOFT2D_NEON` | A |
| 2d: NEON reject scan for the sprite interleaver (LITEV_SOFT2D_OBJNEON) | `LITEV_SOFT2D_OBJNEON` | A |
| 2d: NEON 3D-layer compositor (LITEV_SOFT2D_BG3DNEON) | `LITEV_SOFT2D_BG3DNEON` | A |
| 3d: add the DraStic-style tile software 3D renderer (LITEV_SOFT3D_DRASTIC) | `LITEV_SOFT3D_DRASTIC` | R |
| 3d: let the software 3D raster span into the next frame (LITEV_SOFT3D_ASYNC) | `LITEV_SOFT3D_ASYNC` | R |
| 3d: decouple the tile raster from the emulator thread (LITEV_TILE_COORD) | `LITEV_TILE_COORD` | R |
| render: pin software render threads off the emulator core (LITEV_PIN_RENDER) | `LITEV_PIN_RENDER` | R |
| gpu: copy only dirty chunks into the 2D VRAM shadows (LITEV_SNAP_DIRTY) | `LITEV_SNAP_DIRTY` | R |
| geom: staged deep prefetch in BuildFrameGeom (LITEV_GEOM_PREFETCH2) | `LITEV_GEOM_PREFETCH2` | A |
| sched: drain DMA9/GXFIFO-stall steps in one iteration while ARM7 is halted (LITEV_SCHED_DRAIN) | `LITEV_SCHED_DRAIN` | A |
| jit: region-aware dispatcher, keep the last 4 code regions per CPU (LITEV_JIT_REGION_CACHE) | `LITEV_JIT_REGION_CACHE` | A |
| jit: keep the slice budget in W15 across block hops (LITEV_JIT_BUDGET_REG) | `LITEV_JIT_BUDGET_REG` | A |
| jit: dispatcher continues past IRQs the guest has masked (LITEV_JIT_IRQMASK_CONT) | `LITEV_JIT_IRQMASK_CONT` | A |
| sched: compute the sqrt result on read, from the FPU (LITEV_LAZY_SQRT) | `LITEV_LAZY_SQRT` | A (vs INSTANT_DIVSQRT) |
| geom: skip the copy passes of clip planes no vertex crosses (LITEV_GEOM_CLIP_PLANESKIP) | `LITEV_GEOM_CLIP_PLANESKIP` | A |
| build: add LITEV_AUTO_FRAMESKIP option (adaptive real-time frameskip) | `LITEV_AUTO_FRAMESKIP` | A (UX feature; see section) |

## Performance options

### jit: emit an A64 block dispatcher that stays in generated code (LITEV_JIT_DISPATCH)

**What.** ARMJIT_A64: block exits jump to an emitted dispatcher stub that does the region check and FastBlockLookup inline and branches straight into the next block. It only returns to C++ (ARM_Ret) on budget expiry, StopExecution or a lookup miss. CMake refuses the flag on non-AArch64 targets; the x64 JIT keeps the upstream C++ loop.

**Why it works.** Upstream returns to ARM::Execute<JIT> after every block to look up the next one: a C++ call/return plus re-lookup per hop. On its own the saving moves into the dispatcher's own per-hop bookkeeping (Timestamp += Cycles commit, ~34 instructions, PLAN:974-977), so the value is mostly as the enabler for block linking, the per-site inline cache, GLOBALREG and LAZYFLAGS.

**Exactness.** A.

**Measured (historical).** Flat on its own. Headless host menu +0.2% fps (L:b7a40053, 2026-07-04); headless A55 menu +1.4%/+0.1% fastmem off/on, median of 3 (results-rgds-2026-07-04); app GL 3x in-race with LINK_*: 33.0 fps median vs a 31-33 baseline (A:cebcf951, 2026-07-05).

**Depends on.** jit: CyclesBudget slice slot

**Option.** LITEV_JIT_DISPATCH (default OFF; enabled in the app's shipping build)

### jit: link static block exits directly to their successors (LITEV_LINK_*)

**What.** Three exit classes - unconditional static branch, both edges of a conditional branch, block-end fall-through - emit a per-hop commit followed by a patchable `B` (initially to the dispatcher). Once the target block exists the site is patched to branch straight into it. Link registry, JitBlock link fields and unpatching on invalidation live in ARMJIT; the umbrella LITEV_JIT_LINK is derived in CMake when any class is on. With LITEV_SHADOW_ASSERT the registry is cross-checked on every invalidation.

**Why it works.** A linked hop is one perfectly predicted direct branch instead of the dispatcher's shared indirect BR plus lookup; the A55's ~6.4% branch-miss rate makes indirect hops expensive. 63% of exit sites are link-eligible (L:7cc67bb4). The per-hop cycle commit is kept at the link site, which caps the saving.

**Exactness.** A.

**Measured (historical).** Flat to small, no per-class numbers. Headless A55 menu -1.4%/+2.8% (fastmem off/on) at frameskip 0, +3.3% at frameskip 9; headless host in-race -0.4% (results-rgds/results-race-2026-07-04). Bundled with DISPATCH in the app GL measurement above.

**Depends on.** jit: emitted A64 block dispatcher (LITEV_JIT_DISPATCH)

**Option.** LITEV_LINK_UNCOND, LITEV_LINK_COND, LITEV_LINK_FALLTHROUGH (default ON, effective only with LITEV_JIT_DISPATCH; enabled in the shipping build)

### sched: run each CPU slice to the true next event (LITEV_EVENT_SLICES)

**What.** NDS::RunFrame/NextTarget: instead of the fixed kMaxIterationCycles cap, each scheduler iteration runs the CPUs to the real next deadline - next scheduled event, frame end, or next timer overflow.

**Why it works.** Upstream re-enters the run loop every 64 cycles, ~9.4k scheduler iterations per frame. Longer slices keep both CPUs in generated code for whole event intervals (iterations 9424 -> 3168/frame on host) and are what make dispatcher-side levers such as GLOBALREG pay off.

**Exactness.** B. Deterministic timing relaxation: the coarser ARM9/ARM7 interleave is observable (cross-CPU shared-memory/IPC visibility shifts), so traces must be recorded with the flag on. MP-safe and version-locked like the other shipping B levers.

**Measured (historical).** Headless host: +13.5% fps, +23.2% CPU-bound (PLAN:955, L:833bf80e, 2026-07-04). Headless A55 menu: +9.5%/+7.2% (fastmem off/on) at frameskip 0, +12.6%/+16.0% at frameskip 9; headless host in-race +5.1% fs0 (results-rgds/results-race-2026-07-04). No app number.

**Option.** LITEV_EVENT_SLICES (default OFF; enabled in the app's shipping build)

### jit: inline DTCM block transfers and MainRAM u32 loads (LITEV_MEM_DTCM_BLOCK, LITEV_MEM_MAINRAM_LOAD)

**What.** ARMJIT_A64/ARMJIT_LoadStore.cpp: on the slow-helper fallback (shapes fastmem does not cover statically, e.g. every LDM), emit guarded inline paths - an LDM/STM whose range lies in DTCM copies directly through the DTCM pointer, and a dynamic u32 ARM9 load whose decoded region is MainRAM reads MainRAM directly. Any guard miss falls back to the exact helper. LITEV_MEM_FAST is an option-only umbrella that defaults both sub-flags.

**Why it works.** A guard hit replaces a SlowBlockTransfer9/SlowRead9 call (region-decode switch, call/return, per-word re-classification) with a direct pointer copy or load. Cycles are baked at compile time by Comp_AddCycles_*, so guest timing is identical on both paths.

**Exactness.** A.

**Measured (historical).** Headless 600 frames: block-transfer helper calls 4,558,718 -> 73,184 (-98.4%), u32-read helper calls -57.7% (L:6d256c17). Headless host +4.4% fps menu, +6.1% in-race; headless A55 menu +3.8%/+2.1% fastmem off/on (2026-07-04). No per-flag split and no app number. UNCERTAIN whether the DTCM path still fires once LITEV_MEM_DTCM_FASTMEM maps DTCM into fastmem.

**Option.** LITEV_MEM_DTCM_BLOCK, LITEV_MEM_MAINRAM_LOAD (default OFF; enabled in the app's shipping build)

### jit: evaluate compound guest conditions with native NZCV (LITEV_JIT_CONDFOLD)

**What.** For guest instructions with a compound condition (HI/LS/GE/LT/GT/LE) the A64 CheckCondition transiently does `MSR NZCV, <cpsr>` and skips with a native `B.<inverted cond>` instead of the 5-instruction flag-table lookup. Single-flag conditions (already one TBZ/TBNZ) are unchanged.

**Why it works.** The hardware evaluates the same condition in 2 instructions; the branch decision is identical. Smaller blocks, less i-cache pressure.

**Exactness.** A.

**Measured (historical).** Instruction-count only: 5 -> 2 host instructions per compound condition, host ARM9 time "within noise" (L:95c7a7ae); app "emu opts are neutral here" (A:a5a26f69, 2026-07-08).

**Option.** LITEV_JIT_CONDFOLD (default OFF; enabled in the app's shipping build)

### jit: keep guest NZCV resident in host flags (LITEV_JIT_FIXEDREG)

**What.** Flag-setting ALU ops (ADD/SUB/RSB/CMP/CMN and the logical S-ops) leave the guest N/Z/C/V in host PSTATE instead of extracting them into the emulated CPSR register with CSET+BFI. The flags are materialized lazily, bit-exactly, only at consumers, host-flag clobbers and block boundaries, and CheckCondition reads the resident host flags directly.

**Why it works.** Fewer emitted instructions per flag-setting op and smaller blocks; the foundation that FLAGMERGE and LAZYFLAGS build on. Statically 70% of flag-body boundaries keep flags resident and 3637 CSET+BFI pairs disappear (L:17e0bfed).

**Exactness.** A. Codegen only; cycle counts, timing and the register cache are untouched.

**Measured (historical).** Stage 1, headless host: ARM9ExecNs -2.3% (511k -> 500k ns/frame) (L:8a6d7098, 2026-07-05). Later stages measured as a wash on host. No isolated device number (UNMEASURED on device).

**Option.** LITEV_JIT_FIXEDREG (default OFF; enabled in the app's shipping build)

### jit: pin never-banked guest registers to host registers across blocks (LITEV_JIT_GLOBALREG)

**What.** Guest r0..r6 are pinned to callee-saved W19..W25 globally: loaded once at ARM_Dispatch (slice entry), spilled once at ARM_Ret (slice exit) and kept live across block boundaries, the dispatcher and linked chains. The per-block RegisterCache no longer reloads or writes back the pinned registers. The interpreter fallback is bracketed with spill/reload. ARMJIT_Offsets.h gains ARM_R_offset (static_asserted).

**Why it works.** Removes the per-block first-load and exit-writeback traffic (~49k loads + ~117k writebacks per frame). Only never-banked registers are pinned, so mode/exception switches need no sync. It only pays with long slices (EVENT_SLICES): with short slices the ~3600 dispatch entries/exits per frame each load and spill the pins.

**Exactness.** A. Codegen only.

**Measured (historical).** Device A55, throttled ~83 C, two matched pairs: ARM9 exec 10521 -> 10009 us (-4.9%) and 10884 -> 10604 us (-2.6%), "~0.6-0.9% of total frame time; wall/FPS flat"; headless host with event slices ~516 -> ~484 us ARM9 (-6%) (L:e241422e, PLAN:1924-1993, 2026-07).

**Depends on.** jit: emitted A64 block dispatcher; jit: keep guest NZCV resident in host flags

**Option.** LITEV_JIT_GLOBALREG (default OFF; enabled in the app's shipping build)

### jit: delete the reserved CPSR register and keep NZCV lazy (LITEV_JIT_LAZYFLAGS)

**What.** The guest control bits become canonical in ARM::CPSR memory; guest NZCV lives either in host PSTATE (deferred) or in a dedicated ARM::JitNZCV slot (flushed with MRS+STR, no load-merge). C/V are materialized eagerly at every instruction boundary; only N/Z deferral is kept across instructions, because deferring C/V across instructions broke Pokemon White (a host op clobbered a still-deferred carry, a later guest conditional read it stale and the intro's professor rendered solid white; source fix L:abeca2ef). RCPSR (W27) is deleted as a CPSR carrier and becomes the 8th GLOBALREG pin (r7). Touches the ALU/branch/load-store emitters, the dispatcher, ARM_Dispatch/ARM_Ret in ARMJIT_Linkage.S and ARM.cpp's Execute<JIT> bracket. CMake requires DISPATCH, FIXEDREG, GLOBALREG and CONDFOLD.

**Why it works.** The win is a smaller static code footprint (less i-cache pressure on the A55's 32 KB L1I) and one more pinned guest register, not fewer dynamic instructions. The first version, which kept a register-resident CPSR and merged on flush, regressed; this V2/V3 form is the one that measured positive.

**Exactness.** A. Codegen only. This is the highest-risk JIT commit (it moves the W27 pin and the linkage save/restore). Gated with a JIT-vs-interpreter framebuffer comparison on Pokemon White's intro as well as Shrek: the CPU/RAM trace oracle alone missed the C/V-deferral bug, because the divergence was GPU-only.

**Measured (historical).** Device, app, software renderer, two concurring pairs: frontend stall -3.18%/frame, L1I refills -3.17%, cycles -0.98%, IPC +0.92%, fps +0.43 in a wall/blit-limited ~40 fps session; static JIT footprint -5.67% (FETCH-STALL-SESSION-2026-07-16). V3 footprint -5.78% (P60 §4). The first version regressed: +1.58% instructions/frame, 39.38 -> 38.70 fps. All of these predate the eager C/V materialization; the effect after that fix is UNMEASURED.

**Depends on.** jit: emitted A64 block dispatcher (LITEV_JIT_DISPATCH); jit: evaluate compound guest conditions with native NZCV (LITEV_JIT_CONDFOLD); jit: keep guest NZCV resident in host flags (LITEV_JIT_FIXEDREG); jit: pin never-banked guest registers (LITEV_JIT_GLOBALREG)

**Option.** LITEV_JIT_LAZYFLAGS (default OFF; enabled in the app's shipping build)

### jit: add a per-site 2-way inline cache to the dispatcher (LITEV_JIT_ICACHE)

**What.** Each dynamic exit site gets a private 2-way (guest PC -> host block) cache, checked by the dispatcher before the region bounds check and the FastBlockLookup gather. Entries are filled from dispatcher hits and guarded by ARM::ICacheEpoch, which every block invalidation or cache reset bumps so all entries go stale in O(1).

**Why it works.** The dispatcher's target stream is strongly per-site mono- or bimodal: per-site 2-way covers 93.6-95.4% of lookups vs 15% for a global single entry (shrek-race replay). A hit skips a load from the multi-MB lookup table, which on the in-order A55 is a full memory stall when cold.

**Exactness.** A. Pure predictor; the same block runs with the same cycle accounting.

**Measured (historical).** Live hit rate 96.59% (49.35M/51.1M); ~14.5k cache-cold FastBlockLookup gathers removed per frame; +1.8% JIT code (FETCH-STALL-SESSION-2026-07-16). No isolated device fps A/B (UNMEASURED); it shipped in a bundle measured at 49.4-50.1 fps.

**Depends on.** jit: emitted A64 block dispatcher (LITEV_JIT_DISPATCH)

**Option.** LITEV_JIT_ICACHE (default OFF; enabled in the app's shipping build)

### jit: patch monomorphic dynamic exits into guarded direct branches (LITEV_JIT_DIRECTPATCH)

**What.** When a dynamic exit site hits the same target 8 times in a row in the inline cache, its `B dispatcher` is rewritten to a per-site guard stub (the dispatcher's per-hop commit plus one target-PC compare) and a direct `B` into the target. A guard miss falls back to the dispatcher; every block invalidation reverts all live patches. The headless profile reports promotion/demotion counts.

**Why it works.** Replaces the shared, poorly predicted indirect BR with a perfectly predicted direct branch. ICACHE already removed the expensive part (the cold gather), so only ~10-20 cycles per hop are left to save.

**Exactness.** A.

**Measured (historical).** Device: flat, 48.70 median vs 49.40 without it (in noise), ceiling ~0.15 ms; 27,480 promotions vs 24,082 demotions, guard hit rate 96.18% (PATH-TO-60FPS-2026-07-17 §4/§6). Kept because it is harmless.

**Depends on.** jit: add a per-site 2-way inline cache to the dispatcher (LITEV_JIT_ICACHE)

**Option.** LITEV_JIT_DIRECTPATCH (default OFF; enabled in the app's shipping build)

### jit: pair LDM/STM inline copies with LDP/STP and inline MainRAM block loads (LITEV_JIT_LDMSTM)

**What.** Pairs the contiguous memory side of the inline DTCM block copy with LDP/STP, and adds a guarded inline MainRAM block-LOAD tier: an LDM whose whole range lies in MainRAM (statically classified, then checked at run time for DTCM overlay, region exit and mirror wrap) copies directly instead of calling SlowBlockTransfer9 per word. Stores keep the helper, so JIT invalidation is untouched. (The MainRAM tier was staged earlier as LITEV_MEM_MAINRAM_BLOCK; that option is folded in here.)

**Why it works.** Two guest words per instruction, in the style of DraStic's arm64_load_blockN stubs, and ~8.2k SlowBlockTransfer9 calls/frame (86% of in-race slow reads) replaced by a direct copy. Cycles are baked at compile time, so timing is identical.

**Exactness.** A.

**Measured (historical).** "Removes ~8k per-word helper calls/frame ... neutral on the render-bound headless" (L:35366fb6, 2026-07-08). The MainRAM tier alone on host: SlowBlockTransfer9 calls -63.3%, ARM9 -1.2..-2.5% (L:921546c1). No device number (UNMEASURED).

**Depends on.** jit: inline DTCM block transfers and MainRAM u32 loads (LITEV_MEM_DTCM_BLOCK)

**Option.** LITEV_JIT_LDMSTM (default OFF; enabled in the app's shipping build)

### jit: merge host NZCV into guest CPSR with one MRS (LITEV_JIT_FLAGMERGE)

**What.** When three or more flags must move from host NZCV into the guest CPSR, Comp_RetriveFlags uses one `MRS X0, NZCV` plus mask/ORR instead of a CSET+BFI pair per flag.

**Why it works.** The top nibble of MRS NZCV is laid out bit-for-bit like the guest N/Z/C/V, so 8 host instructions become 3.

**Exactness.** A.

**Measured (historical).** Instruction-count only: full-flag extraction 8 -> 3 instructions (L:27c19c09); "bit-exact, neutral on Shrek" (A:c993cc44, 2026-07-08).

**Depends on.** jit: keep guest NZCV resident in host flags (LITEV_JIT_FIXEDREG)

**Option.** LITEV_JIT_FLAGMERGE (default OFF; enabled in the app's shipping build)

### jit: classify TCM block transfers once in SlowBlockTransfer9 (LITEV_JIT_BLOCKXFER_FAST)

**What.** When an LDM/STM that reached SlowBlockTransfer9 lies wholly inside ITCM or DTCM, classify it once and loop directly over ITCM[]/DTCM[] (ITCM writes still invalidate per word). Straddling or side-effect regions take the original per-word path.

**Why it works.** The helper re-ran the full ITCM/DTCM/region classification for every word; stack LDM/STM hit non-fastmem TCM constantly.

**Exactness.** A.

**Measured (historical).** PMU on device, 3 reps, all separated (L:445f16b3, 2026-09-12): total instructions -0.52%, cycles ~-0.6..-1%; SlowBlockTransfer9 self-time 0.59 -> 0.47, SlowRead9 0.55 -> 0.51 (raster-diluted headless; ~4x larger share in the decoupled app).

**Option.** LITEV_JIT_BLOCKXFER_FAST (default OFF; enabled in the app's shipping build)

### mem: map DTCM into the fastmem window (LITEV_MEM_DTCM_FASTMEM)

**What.** ARMJIT_Memory::MapAtAddress/Unmap gain two symmetric guards so the DTCM region itself gets a normal mapping instead of being hole-punched to nothing. Runtime prop debug.litev.dtcmfastmem (default on when unset) for a single-binary A/B.

**Why it works.** The "map around the DTCM hole" logic was never guarded against the DTCM region itself, so DTCM - the guest stack, 84-99% of all slow-helper calls - was never mapped: every DTCM access faulted once and was patched to the slow C helper. With the mapping, stack loads/stores are single host instructions. The backing store and baked cycles are the ones the helper used.

**Exactness.** A. Same bytes, same cycles; DTCM is non-executable so no JIT invalidation is involved.

**Measured (historical).** App, software renderer, real .dev build, 2026-09-16: 55.9 -> 57.8 fps (+1.9), runFrame 16.69 -> 15.58 ms (measurements ledger 06:14:57; HANDOVER -2026-09-16). Headless production renderer: -3.30% instructions, -3.75% cycles; DTCM slow calls 8.11M -> 0 (L:0e337437).

**Depends on.** jit: install the fastmem fault handler only when fastmem is in use

**Option.** LITEV_MEM_DTCM_FASTMEM (default OFF; enabled in the app's shipping build)

### io: dispatch ARM9 32-bit I/O through a dense jump table (LITEV_IO_DISPATCH_TABLE)

**What.** NDS::ARM9IORead32/ARM9IOWrite32: word-aligned accesses in the primary 8 KB I/O window index a switch on (addr & 0x1FFF) >> 2, which compiles to a jump table. Each fast case keeps its original body; out-of-window and misaligned accesses fall through to the verbatim original switch.

**Why it works.** The original switch over the full sparse address lowered to a ~6-7-branch data-dependent binary search. On the in-order A55 the mispredicted branch chain, not the instruction count, was the cost.

**Exactness.** A.

**Measured (historical).** PMU on device (L:5fd1a861, 2026-09-13): ARM9IOWrite32 self-time 0.42 -> 0.27, branch-misses -0.95% (1.18M fewer), branch instructions -7.9M. A later whitepaper check could not re-trace these deltas; the commit message is the only record.

**Option.** LITEV_IO_DISPATCH_TABLE (default OFF; enabled in the app's shipping build)

### build: compile with -fno-plt (LITEV_LINKOPT)

**What.** Adds -fno-plt at directory scope so every core TU calls external functions through the GOT. The matching -Wl,-Bsymbolic-functions belongs on the final shared library and lives in the app's CMakeLists.

**Why it works.** Removes the lazy `bl func@plt` trampolines from the instruction stream; @plt + linker64 + tlsdesc were ~6% of L1I refills. An initial-exec TLS model for NDS::Current was tried alongside and dropped: bionic rejects IE TLS for a dlopen'd library's own symbol.

**Exactness.** A. Link/codegen only.

**Measured (historical).** PLT relocations 2932 -> 225 (-92.3%); device "~flat solo; kept (footprint)" (FETCH-STALL-SESSION-2026-07-16; P60 §4).

**Option.** LITEV_LINKOPT (default OFF; enabled in the app's shipping build)

### sched: complete ARM9 DIV/SQRT on write (LITEV_INSTANT_DIVSQRT)

**What.** Writing the divider/sqrt operands computes the result immediately and skips scheduling Event_Div/Event_Sqrt.

**Why it works.** Div and Sqrt completions were the largest scheduler flood, ~1016-1233 Div + ~80 Sqrt events per frame (~42% of all events). Results are identical; only the busy-bit transient disappears, and games poll the ready bit.

**Exactness.** B. Deterministic ARM9-only timing relaxation; MP-safe.

**Measured (historical).** App, software renderer, fixed 1800 MHz, 2026-07-08: 24.4 -> 26.1 fps (+7%), runFrame 39 -> 35.9 ms (A:3b463725; single sequential window, treat as +/-1 fps). Headless "+5% cooled emu-only" (build.gradle comment).

**Option.** LITEV_INSTANT_DIVSQRT (default OFF; enabled in the app's shipping build)

### spu: mix N samples per scheduled SPU event (LITEV_SPU_BATCH)

**What.** SPU::Mix generates LITEV_SPU_BATCH_N samples (default 8) per Event_SPU instead of one.

**Why it works.** Cuts the SPU scheduler flood ~8x (547 -> 68 events/frame). Sample values are identical; channel-finish, SPU IRQ and capture side effects land up to N-1 samples late.

**Exactness.** B. Deterministic timing relaxation; MP-safe.

**Measured (historical).** Measured together with LITEV_COARSE_RTC, no per-flag split (UNMEASURED alone): app, 1800 MHz, 2026-07-08: 26.1 -> 27.0 fps (A:4c8ae2a0, noisy samples 25.8/26.7/27.0/26.7); headless +3.7% for both; total scheduler events 1622 -> 596/frame, spu_mix_ns -20% (L:d95c628f).

**Option.** LITEV_SPU_BATCH (default OFF; enabled in the app's shipping build); LITEV_SPU_BATCH_N cache value

### rtc: skip inert 32768 Hz ticks when no RTC interrupt is armed (LITEV_COARSE_RTC)

**What.** When neither the periodic INT1 frequency nor the INT2 alarm is armed, RTC::ClockTimer hops to the next 1024-tick boundary in one event instead of ~1024 per-tick events; it reverts to the exact per-tick path as soon as an RTC IRQ is armed.

**Why it works.** Without an armed interrupt each tick is a no-op counter bump; the 1-second boundary still lands exactly. RTC events 548 -> ~0.5 per frame.

**Exactness.** B. Deterministic timing relaxation; MP-safe. Known: the headless harness hangs at --frameskip 99 (not a gameplay configuration).

**Measured (historical).** Measured together with LITEV_SPU_BATCH (see that commit): app 26.1 -> 27.0 fps (A:4c8ae2a0, 2026-07-08). No per-flag split (UNMEASURED alone).

**Option.** LITEV_COARSE_RTC (default OFF; enabled in the app's shipping build)

### sched: cache the next timer-overflow deadline (LITEV_TIMER_FAST)

**What.** NextTimerDeadline() is cached and recomputed only when a timer overflows or reloads (HandleTimerOverflow) or its control/prescaler/start state is written (TimerStart); savestate load and reset mark it dirty. The savestate format is unchanged.

**Why it works.** Under EVENT_SLICES, NextTarget() rescanned all 8 timers on every one of ~1957 scheduler iterations per frame, although the soonest overflow only changes on an overflow or a control write. Inert without EVENT_SLICES.

**Exactness.** A (relative to the EVENT_SLICES build).

**Measured (historical).** Headless "exact, +0.7%" (session log, 2026-07-08). App, 1800 MHz, together with IDLE_AGGRESSIVE: flat at ~28 fps. Effectively UNMEASURED on the app.

**Depends on.** sched: run each CPU slice to the true next event (LITEV_EVENT_SLICES)

**Option.** LITEV_TIMER_FAST (default OFF; enabled in the app's shipping build)

### jit: fast-forward register-recurrent poll loops (LITEV_IDLE_AGGRESSIVE)

**What.** IsIdleLoop tracks which registers recur across iterations and which registers feed load addresses. With the flag, a backward loop with a register recurrence is still accepted as idle when it reads memory/I/O, has no stores or coprocessor access, and no recurrent register is used as a load address (so scans and copies are excluded).

**Why it works.** Stock melonDS rejects any loop with a cross-iteration recurrence such as a timeout counter. Such a loop that only polls memory is a real wait; it is skipped to the next event instead of being executed thousands of times. The dead counter ends with a different value.

**Exactness.** B. Accuracy trade; verified safe on Shrek only.

**Measured (historical).** Headless, cooled: 51.1 -> 54.9 fps (+7.5%), idle hits 2.31 -> 17.68 per frame, ARM9 exec -8%, framebuffer bit-identical (L:3ea9f4af). App at 1800 MHz before PIN_RENDER: flat (27.8 -> ~28), when the app was bound by render-thread preemption. No post-PIN_RENDER app A/B exists.

**Option.** LITEV_IDLE_AGGRESSIVE (default OFF; enabled in the app's shipping build)

### dma: load ARM9 32-bit DMA unit timings lazily (LITEV_DMA_TIMING_LAZY)

**What.** DMA::UnitTimings9_32 loads the four ARM9MemTimings entries only inside the branch that reads them.

**Why it works.** The function loaded four scattered entries from the 2 MB timing table on every call; the hot geometry-DMA burst path uses none of them. Those were cache-cold loads on a hot path of the in-order A55.

**Exactness.** A. Same timing values.

**Measured (historical).** PMU on device, 3 interleaved reps (L:63dc877c, 2026-09-13): instructions -0.088%; UnitTimings9_32 self-time 0.47 -> 0.40.

**Option.** LITEV_DMA_TIMING_LAZY (default OFF; enabled in the app's shipping build)

### spu: replace channel interpolation with linear interpolation (LITEV_SPU_FAST_INTERP)

**What.** When a channel's InterpType is not None, cosine/cubic/Gaussian sample interpolation collapses to one linear mul/add.

**Why it works.** Cheaper per-sample mixing for games/frontends that select a higher-quality interpolation mode.

**Exactness.** R (approximate audio; guest-visible only through sound capture).

**Measured (historical).** UNMEASURED by construction: dormant on the benchmark, which uses InterpType None (L:87faed0a: SPU::Mix is 66 us/frame at interp None).

**Option.** LITEV_SPU_FAST_INTERP (default OFF; enabled in the app's shipping build)

### spu: decode then mix, skip silent channels, NEON pan-accumulate (LITEV_SPU_MIX_NEON)

**What.** SPU::Mix first decodes all 16 channels (cv[]/mv[]) and then mixes, and SPUChannel::Run returns early for a zero-volume channel (both unflagged). With the flag, the 16-channel pan-accumulate runs as vmull_s32 -> vshrq -> vaddq into int64x2 accumulators.

**Why it works.** The scalar pan-accumulate is 16 serial smull->asr->add chains into one register; that dependency chain stalls the in-order A55. Bit-exact by bound: |in| <= 2^26, per-term <= 2^23, 16-term sum <= 2^27, so 64-bit accumulate-then-narrow equals the scalar 32-bit accumulation for every game. Decode order (0..15) is unchanged, so finish/FIFO/capture side effects stay on the timeline.

**Exactness.** A (20M-case fuzz and a 2000-frame trace incl. framebuffers identical, L:cf2eddbc).

**Measured (historical).** Device microbench, core 3: ~175 -> ~75 ns per call (2.30x); in-game SPU::Mix self-time 0.97% -> 0.96% (noise), window fps 39.25 -> 39.30, ~0.055 ms/frame expected (L:cf2eddbc, 2026-09-12). Sub-fps.

**Depends on.** spu: mix N samples per scheduled SPU event (LITEV_SPU_BATCH)

**Option.** LITEV_SPU_MIX_NEON (default OFF; enabled in the app's shipping build)

### gx: drain GX command batches in a threaded interpreter loop (LITEV_GXFIFO_THREADED)

**What.** GPU3D::Run's per-command drain loop moves into ExecuteCommand as a threaded loop: each command re-dispatches through the computed-goto table and falls through to the next one. CmdFIFORead and its CheckFIFODMA/CheckFIFOIRQ side effects still run once per command in the same order.

**Why it works.** One call drains a whole batch with no per-command bl/ret, prologue or bounds check.

**Exactness.** A.

**Measured (historical).** Headless: gpu3d dispatch -6% (281 -> 264 ns per command) (L:add8e4cb). No fps number.

**Option.** LITEV_GXFIFO_THREADED (default OFF; enabled in the app's shipping build)

### dma: feed geometry DMA straight from MainRAM into the GXFIFO (LITEV_DMA_GXFIFO_FAST)

**What.** Once IsGXFIFODMA has proven src = MainRAM and dst = 0x04000400 (fixed), DMA::Run9 reads the source word directly from MainRAM and calls WriteToGXFIFO under the GeometryEnabled guard, with the same per-word timing and stall handling. UnitTimings9_32_GXFIFO specialises the burst timing for the fixed destination (props debug.litev.gxfifotiming and debug.litev.gxtiminginline, default on).

**Why it works.** Each display-list word went through ARM9Read32's region decode and ARM9Write32 -> ARM9IOWrite32 -> GPU3D::Write32 before reaching WriteToGXFIFO. Geometry DMA is the dominant in-race DMA (~60% of DMA::Run9, itself ~10.5% of the emulator thread).

**Exactness.** A. Pure dispatch elision.

**Measured (historical).** App, 1800 MHz, 2026-07-08: 27.0 -> 27.8 fps (A:5dab4b6e). Headless: DMA bucket 2.79 -> 1.49 ms/frame (-47%), RunFrame 25.65 -> 24.46 ms (L:524a721e).

**Option.** LITEV_DMA_GXFIFO_FAST (default OFF; enabled in the app's shipping build)

### gx: inline the GXFIFO producer into the geometry-DMA loop (LITEV_GXFIFO_DMA_INLINE)

**What.** WriteToGXFIFO and CmdFIFOWrite move into GPU3D_GXFIFO_inl.h as one source of always-inline bodies; GPU3D.cpp's public functions delegate to them (unflagged, byte-exact refactor). With the flag, DMA::Run9's per-word GXFIFO loop calls the inline versions directly (prop debug.litev.gxinline, default on).

**Why it works.** The .dev build has LTO off, so both were real cross-TU calls on every DMA word; inlining removes the call/return and prologue per word.

**Exactness.** A.

**Measured (historical).** Headless production renderer, 250 frames: -0.526% instructions, -0.473% cycles (spread 0.106%) (L:59097aef, 2026-09-16).

**Depends on.** dma: feed geometry DMA straight from MainRAM into the GXFIFO (LITEV_DMA_GXFIFO_FAST)

**Option.** LITEV_GXFIFO_DMA_INLINE (default OFF; enabled in the app's shipping build)

### geom: integer-NEON vertex transform and lighting math (LITEV_NEON_GEOMETRY)

**What.** The per-vertex 4x4 clip transform, texcoord generation, MatrixMult4x4 and the lighting normal transform use widening NEON multiply-accumulate (vmull/vmlal/vshr) kernels. Guarded by __ARM_NEON with the scalar code as fallback.

**Why it works.** Replaces 16 scalar smull dependency chains per transform with 2-lane 64-bit accumulators. Exact: products are 32x32->64 with modular sums. The command dispatch dominates geometry cost, so the gain is small.

**Exactness.** A.

**Measured (historical).** Device A55 in-race, medians of 3: geometry 2.3205 -> 2.2971 ms/frame (-1.0%), window fps 24.97 -> 25.10 (+0.5%) (L:cbaaf32e, 2026-07-05). Later on-device A/B with GEOM_RECIP: null (<0.3 ms, L:292aafe2). Sub-fps.

**Option.** LITEV_NEON_GEOMETRY (default OFF; enabled in the app's shipping build)

### geom: NEON backface cull and 4x3/3x3 matrix multiply (LITEV_GEOM_NEON2)

**What.** Per-polygon backface cull cross/dot (9 scalar s64 multiplies -> ~4 vmull) and MatrixMult4x3/3x3 reuse the NEON kernels; MatrixMult3x3 copies into a 16-entry temp so the kernel's 4-lane loads stay in bounds (scalar path unchanged). The shared NEON helper block is widened to this flag. (The BuildFrameGeom vertex-pack part arrives with the tile renderer.)

**Why it works.** Exact integer equivalents of the scalar math; fewer serial multiply chains on the in-order core.

**Exactness.** A (4M-case fuzz, 2000-frame trace).

**Measured (historical).** UNMEASURED in isolation. Stacked with SPU_MIX_NEON: -1.0% cycles / +0.7% fps (session log 2026-09-12); app commit "sub-1% each" (A:8ca127cd).

**Depends on.** geom: integer-NEON vertex transform and lighting math (LITEV_NEON_GEOMETRY)

**Option.** LITEV_GEOM_NEON2 (default OFF; enabled in the app's shipping build)

### geom: NEON box/pos/vec tests and matrix scale/translate (LITEV_GEOM_NEON3)

**What.** BoxTest (8 clip transforms per command), PosTest, VecTest, MatrixScale and MatrixTranslate reuse the exact NEON kernels.

**Why it works.** Games that use hardware box culling or picking issue these commands heavily; same exactness argument as the other geometry kernels.

**Exactness.** A (24M-case fuzz).

**Measured (historical).** Null for Shrek - these commands are not in its hot path (L:b16513e6). A forward investment for other titles.

**Depends on.** geom: integer-NEON vertex transform and lighting math (LITEV_NEON_GEOMETRY)

**Option.** LITEV_GEOM_NEON3 (default OFF; enabled in the app's shipping build)

### gx: sort polygons with an allocation-free radix sort (LITEV_POLY_RADIX)

**What.** The VBlank polygon Y-sort becomes a 4-pass LSD radix sort over preallocated scratch.

**Why it works.** Produces the identical stable permutation without std::stable_sort's allocation and merge copies; it had been the #2 backend-stall source (~13%).

**Exactness.** A (stable, identical order).

**Measured (historical).** Mechanism only: no separate fps. Stacked with SNAP_DIRTY on device the matched cooled A/B was fps-neutral on that render-gated stack: stack 50.10 vs +snap+radix(+geometry offload) 48.85 (PATH-TO-60FPS-2026-07-17, COOLED A/B).

**Option.** LITEV_POLY_RADIX (default OFF; enabled in the app's shipping build)

### 2d: NEON master-brightness and colour expansion (LITEV_NEON_RENDERER)

**What.** GPU2D_NEON.{h,cpp} (compiled on AArch64 only, via src/CMakeLists.txt) provide 4-8-pixel NEON versions of SoftRenderer::ApplyMasterBrightness and the 6->8-bit BGRA expansion; ConvertToBGRA uses paired integer expansion (prop debug.litev.convertscalar).

**Why it works.** Master-brightness and output expansion process 4-8 pixels per NEON op instead of one.

**Exactness.** A (300-frame framebuffer hashes identical, L:6f4a6bf3).

**Measured (historical).** UNMEASURED in this stack. convertscalar: -0.18..-0.22% instructions (measurements ledger 2026-09-16).

**Option.** LITEV_NEON_RENDERER (default OFF; enabled in the app's shipping build)

### gpu: add a per-frame rasterization skip gate (LITEV_AGGRESSIVE_SKIP)

**What.** GPU gains FrameskipTarget/Counter/SkipThisFrame and SetFrameskipTarget(); StartHBlank is re-nested so DrawScanline/DrawSprites and Start3DRendering sit inside the skip gate. CPU, DMA, timers and Wi-Fi run unconditionally. The harness exposes --frameskip N.

**Why it works.** Skipped frames do no 2D/3D rasterization.

**Exactness.** A at frameskip 0 (identical to the flag-off build); skipped frames do not render.

**Measured (historical).** Headless menu --frameskip 1: 1050 -> 1356 fps (+29%) (L:3f6fe05a). Shipping effect is 0: real gameplay runs at frameskip 0.

**Option.** LITEV_AGGRESSIVE_SKIP (default OFF; enabled in the app's shipping build)

### build: add LITEV_AUTO_FRAMESKIP option (adaptive real-time frameskip)

**What.** A CMake option that compiles in the frontend's adaptive frameskip controller. The lib side is only the option: it sets the LITEV_AUTO_FRAMESKIP define, which the LITEV_* re-export loop carries PUBLIC to the out-of-tree Android frontend, and refuses to configure without LITEV_AGGRESSIVE_SKIP, whose GPU::SetFrameskipTarget() the controller drives. The controller itself lives in the app (MelonDSAndroidJNI.cpp emu loop) behind a user setting, default off.

**Why it works.** This is a UX feature, not a throughput optimisation. When a scene cannot sustain 60 fps the frame limiter has no time left to sleep, so the game runs in slow motion. Skipping rasterisation on some frames (CPU, DMA, timers and audio keep running) lowers the per-frame cost enough to hold real-time pace, at the price of fewer displayed frames. The controller compares a fast EMA (alpha 0.3) of per-frame work time with the 60 fps budget: above 1.15x it raises the skip level at once (by 2 when at or above 1.9x) and then waits 5 frames (fast attack); below 0.70x for 120 consecutive frames it lowers the level by one (slow release); the 0.70x-1.15x dead zone holds the level so it does not oscillate. The skip level is capped at 3 (render 1 frame in 4). It is inactive while fast-forward is on and resets when fast-forward is toggled or the setting is turned off.

**Exactness.** A for the option itself: no core code changes, and with the setting off the frontend never touches the frameskip target. With the setting on, emulation is unchanged; only displayed frames are skipped (as with LITEV_AGGRESSIVE_SKIP at a non-zero target). Host-side pacing only, so it cannot change guest semantics or MP behaviour.

**Measured.** Does not change the measured fps by design. It holds the emulated frame rate at the target by dropping rendered frames, which hides the per-frame cost that an fps measurement exists to see; the testing hard rule ([TESTING-METHODOLOGY.md](TESTING-METHODOLOGY.md) §3.0, frameskip OFF) requires the setting off, no fast-forward and debug.litev.frameskip unset for every performance measurement.

**Depends on.** gpu: add a per-frame rasterization skip gate (LITEV_AGGRESSIVE_SKIP)

**Option.** LITEV_AUTO_FRAMESKIP (default OFF; enabled in the app's build, runtime-gated by the "Auto frameskip" setting, default off)

### 2d: rasterize software 2D off the emulator thread (LITEV_SOFT2D_THREADED)

**What.** At each scanline the emulator thread only snapshots the 2D register, palette and sprite state; the BG/OBJ raster and composite run on async band threads. Byte-exact prep refactors, unflagged: SoftRenderer2D reads the palette through CurPalette, and DrawScanlineA/B/DoCapture take an explicit source, DISPCNT and master brightness instead of reading live GPU state.

**Why it works.** Removes the 2D raster from the emulator's per-scanline critical path; the base that SOFT2D_DEPTH2 builds on.

**Exactness.** R. Emulation is unchanged; the 2D framebuffer is the same, produced on another thread.

**Measured (historical).** Headless: +5% (24.6 -> 25.85), +8% (24.6 -> 26.63), depth-1 async 16.3 -> 17.2 (+5%) (L:9f901889, 2f3d9cba, a3bbb6fc, 2026-07-07/08). No isolated app number; a cool-peak "~41 -> ~57" with S2D_NBANDS=2 is UNCERTAIN (sustained ~36-44 the same day).

**Depends on.** diag: add LitevSoftProf software-render phase profiler (LITEV_SOFTPROF)

**Option.** LITEV_SOFT2D_THREADED (default OFF; enabled in the app's shipping build)

### 2d: double-buffer 2D snapshots so the emulator runs a frame ahead (LITEV_SOFT2D_DEPTH2)

**What.** The 2D snapshots (and the flat BG/OBJ/ext-palette VRAM shadows in GPU.h) are double-buffered by frame parity, so the emulator builds frame N+1 while the 2D compositor finishes frame N. Pipe depth is tunable via debug.litev.pipedepth / LITEV_PIPEDEPTH (default 2); debug.litev.pipetrace logs the handoff. (Previously reached through an internal LITEV_2D_SNAP2 alias, now named after the flag.)

**Why it works.** Removes the VBlank bar2D wait where the emulator slept until the previous frame's 2D composite finished.

**Exactness.** R. Emulation state byte-identical.

**Measured (historical).** App, software renderer, scene-matched, interleaved, 66-73 C, 2026-07-18: 52.05/52.00 -> 55.35/54.20 fps (+2.8), runFrame 18.5 -> 17.7 ms, bar2D 3.18 -> 0.78 ms (PATH-TO-60FPS-2026-07-17 lines 525-527; L:d45e1cb8).

**Depends on.** 2d: rasterize software 2D off the emulator thread (LITEV_SOFT2D_THREADED)

**Option.** LITEV_SOFT2D_DEPTH2 (default OFF; enabled in the app's shipping build)

### 2d: NEON BG/OBJ colour-effect compositor (LITEV_SOFT2D_NEON)

**What.** SoftRenderer2D::ColorComposite runs as a branchless 4-pixel NEON kernel (masks/selects for blend, window and priority) instead of a ~133-instruction out-of-line call per pixel; the brightness-up path is prop-gated (debug.litev.compositebrightup, default on). Widens the GPU2D_NEON.cpp build condition.

**Why it works.** ~133 -> ~44 dynamic instructions per pixel. The work runs on the async 2D thread, so it helps fps only when that thread is on the critical path.

**Exactness.** A (2000-frame framebuffer/audio hashes identical).

**Measured (historical).** M3 host threaded +59-81%; on device pinned 1416 MHz, 5x200 frames: "every one is neutral on the A55" (session log). Later: ColorCompositeLine is 38% of the s2d worker; compositebrightup -1.085% instructions, s2d wall 13.20 -> 10.73 ms (INVESTIGATION-LOG; ledger).

**Depends on.** 2d: NEON master-brightness and colour expansion (LITEV_NEON_RENDERER)

**Option.** LITEV_SOFT2D_NEON (default OFF; enabled in the app's shipping build)

### 2d: NEON reject scan for the sprite interleaver (LITEV_SOFT2D_OBJNEON)

**What.** InterleaveSprites tests the priority match 16 pixels at a time and skips whole chunks with no match; matching chunks take the original scalar body.

**Why it works.** At a given BG priority most of the 256 pixels carry no matching sprite, so the reject path dominates; it is ~3.5x denser in NEON.

**Exactness.** A (framebuffer hashes identical over 2000 frames).

**Measured (historical).** M3 host single-thread +3.8%, neutral threaded; device neutral (session log, 2026-07-10).

**Option.** LITEV_SOFT2D_OBJNEON (default OFF; enabled in the app's shipping build)

### 2d: NEON 3D-layer compositor (LITEV_SOFT2D_BG3DNEON)

**What.** DrawBG_3D (the BG0 3D layer in 3D games) becomes a branchless 4-pixel NEON mask/select kernel. Widens the GPU2D_NEON.cpp build condition.

**Why it works.** Removes two per-pixel branches; 3584 -> 1280 dynamic instructions per scanline.

**Exactness.** A (framebuffer hashes identical over 600 frames).

**Measured (historical).** Device neutral, as for the other 2D NEON kernels (session log, 2026-07-10).

**Depends on.** 2d: NEON master-brightness and colour expansion (LITEV_NEON_RENDERER)

**Option.** LITEV_SOFT2D_BG3DNEON (default OFF; enabled in the app's shipping build)

### 3d: add the DraStic-style tile software 3D renderer (LITEV_SOFT3D_DRASTIC)

**What.** GPU3D_TileSoft.{h,cpp}: TileRenderer3D, selected in SoftRenderer's constructor instead of SoftRenderer3D. Two planes per pixel (colour and packed depth|id) in 32 KB tiles, polygons bucketed per 16-line block, band workers per tile row, AA/edge/fog recomputed from neighbours in a post-pass. BuildFrameGeom packs the frame's geometry for the workers (with the LITEV_GEOM_NEON2 vertex-pack branch).

**Why it works.** A tile fits one core's L1D, so raster memory traffic stays in cache instead of streaming a ~1.2 MB full-frame buffer through DRAM. That frees core time and cuts contention with the emulator thread.

**Exactness.** R. Render only; emulation state is byte-identical.

**Measured (historical).** App, matched protocol, 2026-07-17: 49.40 fps vs 44.70 for the previous depth-2 renderer stack (+4.7); vs the band reference renderer parity (~48.5 vs ~47.7) with -56% L2 refill, -43% L1D, -12% raster core-ms (L:78fea3a5, 2026-07-16). Also the enabler for TILE_COORD.

**Depends on.** geom: NEON backface cull ... (LITEV_GEOM_NEON2); 2d: double-buffer 2D snapshots (LITEV_SOFT2D_DEPTH2)

**Option.** LITEV_SOFT3D_DRASTIC (default OFF; enabled in the app's shipping build)

### 3d: let the software 3D raster span into the next frame (LITEV_SOFT3D_ASYNC)

**What.** At VBlank, GPU only waits for the 3D renderer (Finish3DRendering) when GPU3D::NeedsRenderBarrier() says VBlank would mutate state the in-flight raster reads (geometry re-sort, Render* registers); GPU3D::VBlank skips the Render* rewrite when nothing changed; SoftRenderer3D flattens texture VRAM behind the same barrier and only when it is dirty. Adds the LSP_* barrier timing hooks.

**Why it works.** The raster needed ~26 ms but was given the ~15 ms between VCount 215 and the VBlank barrier, so the emulator stalled ~9 ms every frame. Shrek flushes geometry every other frame, which leaves two frames of headroom.

**Exactness.** R. Output unchanged; the 2D compositor still paces against the raster per scanline. Known issue (also in the source tree, abeca2ef): with the reference SoftRenderer3D, i.e. built without LITEV_SOFT3D_DRASTIC, the emulator can deadlock in FinishRendering waiting for a render that was never started (Pokemon White intro). That is why this commit lands after the tile renderer, which the shipping build uses.

**Measured (historical).** App, software renderer, 1992 MHz pinned, 2026-07-12: ~38 -> ~44.5 fps (+17%), 26 -> 22.5 ms, emulator bar3D ~9 ms -> 0 (L:5b35a7a1/A:13e0a840); re-verified 43.3-43.9.

**Depends on.** diag: add LitevSoftProf software-render phase profiler (LITEV_SOFTPROF)

**Option.** LITEV_SOFT3D_ASYNC (default OFF; enabled in the app's shipping build)

### 3d: decouple the tile raster from the emulator thread (LITEV_TILE_COORD)

**What.** A coordinator thread owns raster dispatch and the wait. Geometry is buffered NGEOM = NCOL = 3 deep and colour is triple-buffered with per-row readiness, so the emulator hands off frame N and builds N+1 while workers raster N. Includes the geometry-lap fix (NGEOM must equal NCOL) and a texture-input barrier before tile reuse (debug.litev.texbarrier, default on).

**Why it works.** Previously the emulator slept on the 3D barrier; now the ~6 ms raster hides inside the emulator's ~15 ms frame. With NGEOM < NCOL the emulator could lap the geometry arena the workers were still reading, which showed as black tile bars.

**Exactness.** R. Emulation state byte-identical.

**Measured (historical).** App, scene-matched, clock held, interleaved A-B-A-B, 68-72 C, 2026-07-18: 49.3 -> 53.1 fps (+3.8), runFrame 19.5 -> 18.0 ms, bar3D -> 0.00 (PATH-TO-60FPS-2026-07-17 lines 504-507). NGEOM=NCOL: black-bar rate 1.63% -> 0.00% over 1336 frames (L:2b9c88c1, 2026-09-11).

**Depends on.** 3d: add the DraStic-style tile software 3D renderer (LITEV_SOFT3D_DRASTIC)

**Option.** LITEV_TILE_COORD (default OFF; enabled in the app's shipping build)

### render: pin software render threads off the emulator core (LITEV_PIN_RENDER)

**What.** Android only. The async 2D band threads, the SoftRenderer3D render thread and the tile workers set their affinity off core 3 (where the app pins the emulator thread): 2D and 3D threads to {0,1,2} (2D overridable via debug.litev.s2dcpu), tile worker b to core b % 3.

**Why it works.** The library's render threads were created unpinned and the scheduler put them on core 3, where they preempted the latency-critical emulator thread for ~13 ms/frame. The RK3566 has four identical cores, so isolation is the whole game. Pinning each tile worker to its own core stopped two workers doubling up on one core.

**Exactness.** R. Thread placement only.

**Measured (historical).** App, software renderer, fixed 1800 MHz, 2026-07-08: 28 -> 40.6 fps (+45%), runFrame 33.5 -> 22 ms (A:ff89b923) - the largest single lever of the campaign. Mask {1,2} -> {0,1,2}: "+1 fps" (L:5e3bf395). Per-worker cores: runFrame ~17.30 -> ~16.93 ms (L:f161b8ef, 2026-09-11).

**Option.** LITEV_PIN_RENDER (default OFF; enabled in the app's shipping build)

### gpu: copy only dirty chunks into the 2D VRAM shadows (LITEV_SNAP_DIRTY)

**What.** The VBlank flat BG/OBJ/ext-palette VRAM shadow snapshots used by SOFT2D_DEPTH2 copy only the 512 B chunks marked dirty since that shadow bank was last written, instead of the whole ~640 KB-1.1 MB every frame. (The original also shadowed texture VRAM for the GL render-thread seam, which is not part of this history.)

**Why it works.** Removes a large blind memcpy that was the #1 backend-stall source (~21%).

**Exactness.** R (A for emulation): shadows are byte-identical to a full copy.

**Measured (historical).** EmuSnap 0.78 -> 0.05 ms/frame on device; with POLY_RADIX the cooled matched A/B was fps-neutral on that render-gated stack (50.10 vs 48.85 with geometry offload, PATH-TO-60FPS-2026-07-17 §6).

**Depends on.** 2d: double-buffer 2D snapshots so the emulator runs a frame ahead (LITEV_SOFT2D_DEPTH2)

**Option.** LITEV_SNAP_DIRTY (default OFF; enabled in the app's shipping build)

### geom: staged deep prefetch in BuildFrameGeom (LITEV_GEOM_PREFETCH2)

**What.** BuildFrameGeom prefetches Polygon structs 8 polygons ahead and their Vertex lines 4 ahead (staged so the polygon is warm before Vertices[] is chased). Distances via debug.litev.geopf_poly/vtx/vl, on/off via debug.litev.geoprefetch.

**Why it works.** BuildFrameGeom chases pointers to scattered 64 B Vertex structs; on the in-order A55 each miss costs ~150-190 cycles and freezes the pipeline. Prefetching far enough ahead overlaps the miss with useful work; only a few lines are in flight so the fill buffers are not flooded. A shallow one-polygon prefetch measured null because it arrived too late.

**Exactness.** A. Pure prefetch.

**Measured (historical).** App emulator thread, capture.sh, prop off vs on, 2026-09-16: BuildFrameGeom stall_backend share 19.33% -> 8.86%, l2d_cache_refill share 15.19% -> 7.43%, ~3% of total emulator cycles, no cost to the raster workers (L:25993610). No app fps A/B (UNMEASURED).

**Depends on.** 3d: add the DraStic-style tile software 3D renderer (LITEV_SOFT3D_DRASTIC)

**Option.** LITEV_GEOM_PREFETCH2 (default OFF; enabled in the app's shipping build)

### jit: batch per-instruction cycle adds until the next cycle read (LITEV_JIT_CYCLE_BATCH)

**What.** The inline `add w28` of every unconditional ALU/Thumb instruction is summed at compile time and added where RCycles is read: SaveCycles, mid-block exits (on the exit path only) and the block-end add.

**Why it works.** ~8 % of hot JIT code was cycle adds; 18.8k static adds become 357.

**Exactness.** A.

**Measured.** Device headless, emu thread, n = 3: instructions -0.79 %, cycles -0.89 %, L1I -1.75 %, stall_frontend -2.26 % (lib b189d618, 2026-10-02).

**Option.** LITEV_JIT_CYCLE_BATCH (default OFF; enabled in the app's shipping build)

### gx: PIPE and FIFO in one ring (LITEV_GXFIFO_UNIFIED)

**What.** The 4-entry command PIPE and 256-entry FIFO share one ring; the FIFO-to-PIPE refill is a counter update. Savestates keep the two-FIFO layout.

**Why it works.** Every one of ~11.5k entries per frame was copied FIFO -> PIPE before being read.

**Exactness.** A (levels, IRQ, DMA trigger and stall points unchanged).

**Measured.** Device headless, emu thread, n = 3: instructions -1.55 %, cycles -0.52 % (lib 4e558dad, 2026-10-02).

**Option.** LITEV_GXFIFO_UNIFIED (default OFF; enabled in the app's shipping build)

### sched: drain DMA9/GXFIFO-stall steps in one iteration while ARM7 is halted (LITEV_SCHED_DRAIN)

**What.** While ARM9 is stopped on a DMA or a GXFIFO stall, the scheduler loops the ARM9-side step (DMA burst or stall advance, `RunTimers(0)`, `GPU3D.Run`) inside one iteration instead of also running `NextTarget`, the ARM7 catch-up, `RunTimers(1)` and `RunSystem` each time. It only does so while those tails are no-ops: the slice target is not reached (no event can fire), ARM7 is halted with no pending IRQ and no DMA7, and EVENT_SLICES bounds the slice by the next timer overflow (`NDS::SchedDrainContinue`).

**Why it works.** Shrek's geometry DMA ping-pongs with the FIFO: ~1,760 scheduler steps per frame, ~1,000 of them drainable (host count). PW has almost none.

**Exactness.** A. The study's probe also deferred a running ARM7; that is not exact (an event ARM7 schedules inside the deferred window fires late) and the trace gate catches it on PW.

**Measured.** Device headless, emu thread, 500 f, n = 3: Shrek slot 2 cycles -0.80 %, instructions -1.51 %; PW overworld ~-1 % (2026-10-02).

**Option.** LITEV_SCHED_DRAIN (default OFF; needs LITEV_EVENT_SLICES; enabled in the app's shipping build)

### jit: region-aware dispatcher, keep the last 4 code regions per CPU (LITEV_JIT_REGION_CACHE)

**What.** Each CPU keeps its last 4 executable code windows `{start, size, lookup}` (`ARM::JitRegions`). On a FastBlockLookup window miss the emitted dispatcher checks them and, on a hit, makes that window current and continues with the inline lookup instead of returning to C++. The C++ re-entry uses the same cache before `SetupExecutableRegion`.

**Why it works.** Pokemon White bounces between main RAM, ITCM and BIOS through dynamic branches: ~2,000 C++ re-entries per frame were region switches to already-compiled blocks (host: ARM9 re-entries 2,949 -> 931 per frame on pw.ml1).

**Exactness.** A. Only windows for which `SetupExecutableRegion` gives the same answer at every address are cached (`ARMJIT::RegionCacheable`), and the caches are cleared on every ITCM/DTCM, WRAMCNT, savestate or reset change; NDS only.

**Measured.** Device headless, emu thread, 500 f, n = 3: PW overworld cycles -3.4 % (headless fps 59.0 -> 61.3), Shrek slot 2 -0.75 %. With SCHED_DRAIN: PW overworld -3.6 %, PW pw.ml1 -4.8 %, Shrek -1.9 % (2026-10-02).

**Option.** LITEV_JIT_REGION_CACHE (default OFF; needs LITEV_JIT_DISPATCH; enabled in the app's shipping build)

### jit: keep the slice budget in W15 across block hops (LITEV_JIT_BUDGET_REG)

**What.** The remaining slice budget lives in W15 (removed from the register allocator) for the whole JIT slice. A hop (linked exit, DIRECTPATCH guard stub, dispatcher) is `SUBS W15, W15, RCycles` + `B.LE` instead of a 64-bit Timestamp read-modify-write through a MOVZ/MOVK address plus a CyclesBudget read-modify-write. Timestamp is `JitTsBase - budget`; it and CyclesBudget are written to memory before every C++ helper call (`Compiler::QuickCallFunction` wrapper, which reloads W15 afterwards) and in the dispatcher's exit tail. `ForceExecutionExit` and the idle-branch exit re-base `JitTsBase` before zeroing the budget.

**Why it works.** ~15k (PW) to ~22k (Shrek) hops per frame each paid ~11 instructions, 2 loads and 2 stores; the exit commit was 14 % of hot JIT bytes.

**Exactness.** A: every C++ reader sees the same block-start Timestamp as before. The per-hop StopExecution check stays (an IRQ can be pending while the budget is positive). The trace gate catches both a missing Timestamp write at helper calls and a missing re-base.

**Measured.** Device headless, emu thread, 500 f, vs SCHED_DRAIN + REGION_CACHE: PW overworld cycles -1.66 % (n = 6), Shrek slot 2 -2.70 % (n = 3), PW pw.ml1 -1.07 % (n = 2). App PW overworld software uncapped ABBA: 63.3/63.0 -> 63.8/63.8 fps, runFrame CPU 11.87 -> 11.59 ms (2026-10-02).

**Option.** LITEV_JIT_BUDGET_REG (default OFF; needs LITEV_JIT_DISPATCH; enabled in the app's shipping build)

### jit: dispatcher continues past IRQs the guest has masked (LITEV_JIT_IRQMASK_CONT)

**What.** The dispatcher's StopExecution check continues the hop when the only stop reason is an IRQ (StopExecution == 0x100) and ARM::CPSR has the I bit set, instead of returning to C++.

**Why it works.** StopExecution includes the IRQ line, which follows IME/IE/IF regardless of CPSR.I. While an IRQ is pending but masked (IRQ handlers, OS critical sections) every block hop bounced to C++, where TriggerIRQ returns at once and the loop re-dispatches. Pokemon White overworld: 2,642 of 3,505 ARM9 C++ re-entries per frame (pw.ml1 343 of 931, Shrek 2 of 598).

**Exactness.** A. The C++ round trip did nothing but the same Timestamp/budget arithmetic.

**Measured.** Device headless, emu thread, 500 f, n = 3: PW overworld cycles -0.87 %, instructions -1.45 %; Shrek flat (2026-10-02).

**Option.** LITEV_JIT_IRQMASK_CONT (default OFF; needs LITEV_JIT_DISPATCH; enabled in the app's shipping build)

### sched: compute the sqrt result on read, from the FPU (LITEV_LAZY_SQRT)

**What.** With INSTANT_DIVSQRT, writes to SQRTCNT / SQRT_PARAM only mark the result stale; SqrtDone runs on the next ARM9 read of 0x040002B0-0x040002B7 or at a savestate. The root comes from the FPU with an exact integer correction below 2^62; the original bit loop stays above it (its u32 `prod` overflows there and differs from the true root).

**Why it works.** A 64-bit sqrt writes three registers and each write ran the 32-step loop. PW overworld: ~300 sqrts per frame.

**Exactness.** A relative to INSTANT_DIVSQRT: every value the ARM9 reads from 0x280-0x2BF hashes identically (probe, 4 scenes, JIT + interp).

**Measured.** Device headless, emu thread, 500 f, n = 3: PW overworld instructions -1.31 %, cycles -0.48 %; Shrek flat (2026-10-02).

**Option.** LITEV_LAZY_SQRT (default OFF; needs LITEV_INSTANT_DIVSQRT; enabled in the app's shipping build)

### geom: skip the copy passes of clip planes no vertex crosses (LITEV_GEOM_CLIP_PLANESKIP)

**What.** ClipAgainstPlane returns early (after the colour fix-up) for a plane no vertex crosses, instead of copying every Vertex through a temporary and back in two passes.

**Why it works.** Only polygons that cross some plane reach the clipper, but they paid all three planes. ClipAgainstPlane was 3.3-3.8 % of the emu thread in the PW overworld.

**Exactness.** A. Output after every plane hashes identically to the parent (probe, 4 scenes); a differential build found no difference.

**Measured.** Device headless, emu thread, 500 f, n = 3: PW overworld instructions -0.97 %, cycles -0.66 %; Shrek instructions -0.22 % (2026-10-02).

**Option.** LITEV_GEOM_CLIP_PLANESKIP (default OFF; enabled in the app's shipping build)

**All three together (app, PW overworld slot 2, uncapped, frameskip prefs off, ABBA).** Software: 63.5/63.9 -> 64.6/64.2 fps, runFrame CPU 11.66/11.61 -> 11.34/11.38 ms. Hybrid 3x: 71.7/71.0 -> 71.0/71.1 fps (flat; runFrame CPU 11.56/11.57 -> 11.38/11.38 ms, but emu-thread run-queue wait 71 -> 81-85 ms/s in those legs). Device headless with them plus MSR_SAMEMODE (later dropped): PW overworld cycles -1.70 %, pw.ml1 -0.80 %, Shrek flat.

## Unflagged optimizations

The renderer changes in the first table are approximate (R); the rest are byte-exact and always on; a few have a `debug.litev.*` Android property to turn them off for an A/B.

### Renderer threading and draw-count changes (runtime props, 2026-10-02)

All exactness **R** (local framebuffer only, MP-safe). Each has a `debug.litev.*` prop to turn it off.

| Change | Prop (=0 disables) | Measured (RG DS, Shrek slot 2, uncapped) |
|---|---|---|
| GL 3D draws line polygons as thin quads (scale-factor px wide) so they batch with triangles | `linequads` | hybrid 3x draws/job 327 -> 187, GL job CPU 9.05 -> 6.64 ms, 65.5/66.0 -> 72.3/72.0 fps |
| Hybrid descriptors uploaded straight from RAM (not via a 2D-thread-filled PBO) | `hybdirect` | 72.5/73.2 -> 75.0/75.5 fps |
| Hybrid upload + merge + frame hand-off on a present thread (shared EGL context) | `hybasync` | 75.3/75.4 -> 80.3/79.5 fps |
| GL hi-res: 3D on its own GL thread (GLThread3D) | `glhithread` | with `glhilate`: 1x 46.5 -> 58.5, 3x 31.8 -> 31.2 (GPU-bound) |
| GL hi-res: show the 3D rendered one frame earlier, so the emu thread never waits for the newest render. **Trade-off: one extra frame of 3D display latency relative to the 2D layers** | `glhilate` | without it the emu waits 3.6 ms (1x) / 14.7 ms (3x) per frame: 1x 48.5, 3x 26.2 |
| Diagnostics (not optimizations): `glskip` bitmask skips GPU passes for a per-pass cost breakdown (1 3D polygons, 2 3D edge/fog, 4 2D compositor, 8 sprites, 16 final pass, 32 hi-res present copy, 64 hybrid merge, 128 compositor without the 3D texture, 256 trivial compositor shader; `glcomp=2..5` = compositor variants: constant line, no OBJ, no BG fetches, BG0 only); with `prof=1` a `LITEV_GLSTAT` line counts passes per frame | `glskip` | GH3 Shrek slot 2 (31.5 fps): compositor ~16-21 ms/frame, 3D polygons ~4, final pass ~2.3, fog ~0.7, present copy ~0.7; after `glcomp`: sprites ~1.3, per-line UBO indexing ~1.5. Hybrid 3x: merge skipped -> GPU 91 % -> 47 % |
| GL hi-res: frame copy-out on a present thread | `glhiasync` | 1x 58.4/58.4 -> 60.1/61.3 fps |
| GL hi-res: 2D compositor shader without the 4x5 priority loop (top-two selection over draw-order keys), no fetches of disabled BGs, mediump blend maths. Same output (in-process check `compcheck=1`: 0 px differ over 8.8M px at 3x) | `glcomp` | hi-res 3x 31.5/31.5 -> 43.3/43.3 fps (GPU-bound) |
| GL 3D (hybrid and hi-res): attachments invalidated before the clear, depth/stencil + attribute buffer invalidated after the frame (never loaded or written back); fog reads depth/attributes by framebuffer fetch (`GL_EXT_shader_framebuffer_fetch` + `GL_ARM_shader_framebuffer_fetch_depth_stencil`) instead of sampling the attachments (texture path kept when edge marking is on or the extensions are missing) | `gl3dtile` | hi-res 3x 43.3/43.2 -> 43.8/43.8 fps (GPU-bound; Shrek's 3D renders at 30 Hz) |
| Hybrid merge shader compiled with `precision mediump float/int` (colours <= 63, products <= 63*32, coordinates < 2^15, 8-bit unorm fetches round exactly in fp16). The merge was ~44 % of the Mali's busy time at hybrid 3x (`glskip=64`: GPU 91 % -> 47 %) | `hybmp` | hybrid 4x (GPU-bound) ABBA 55.9/55.9 -> 61.6/61.6 fps |
| GL hi-res compositor drawn per run of lines whose per-line config differs only by a linear BG-offset step (text BG Y +1/line, rotscale reference +(B,D)/line), with the run's config in a single-struct UBO block: constant offsets instead of a per-pixel indexed load of the 192-line array. Falls back to the array shader above 16 runs. `compcheck=1`: 0 px differ | `glcomprun` | hi-res 3x 43.9/43.9 -> 45.0/45.0 fps (1 run per engine per frame on Shrek) |
| GL 3D render shader: colour maths in mediump float (texcoords, depth and the 32-bit polygon attribute ints stay highp) | `gl3dmp` | hybrid 4x (GPU-bound) ABBA 61.6/61.6 -> 63.1/63.0 fps; hi-res 3x neutral (45.0 -> 45.3 with it off, 3D at 30 Hz is a small share there) |
| Hybrid merge: the native (1x) position comes from an interpolated varying instead of a per-pixel `P / uScale` integer divide (no integer divide unit on Mali; emulated in ALU) | `hybdiv` | hybrid 4x (GPU-bound) ABBA 63.1/63.0 -> 67.0/67.0 fps; hybrid 3x GPU load 91 % -> 79 % at 85 fps |
| GL 3D plain clear (no clear bitmap) by `glClearBuffer` with the same colour/attribute/depth/stencil values instead of a full-screen shaded draw (Mali applies it at tile load) | `gl3dclr` | hybrid 4x (GPU-bound) ABBA 67.0/67.1 -> 68.5/68.6 fps |


### jit: pass the ARM* to slow-memory and region helpers

**What.** The JIT's slow and region helpers (VRAM, GPU3D, I/O, cart ROM) take the live ARM* and use cpu->NDS instead of the thread_local NDS::Current. The A64 emitter passes RCPU in X1 (value in W2). The x86-64 emitter is updated to the same (addr, cpu[, val]) ABI: the original change updated only the A64 call site, which left x64 builds calling the helpers with a garbage cpu argument.

**Why it works.** Each NDS::Current access in a dlopen'd library is a general-dynamic TLS lookup through __tls_get_addr/tlsdesc. Passing the pointer removes that call and its dTLB-missing loads from every slow-memory access.

**Exactness.** A. Unflagged; same helpers, same values.

**Measured (historical).** App: 34.7 -> 35.3 fps (L:fe69c06a, 2026-07-10, perf-timeline); the tlsdesc dTLB share went 6.5% -> 0.34% of l1d_tlb samples (L:fb262986).

**Depends on.** jit: install the fastmem fault handler only when fastmem is in use

### fifo: wrap power-of-two FIFO indices with a mask

**What.** FIFO<T,N>::Write/Read use `Pos = (Pos+1) & (N-1)` when N is a power of two (if constexpr), the original compare-and-reset otherwise.

**Why it works.** One fewer branch per read/write on the GX FIFOs (CmdFIFO 256, CmdPIPE 4, CmdStallQueue 64), which are hit thousands of times per geometry-heavy frame.

**Exactness.** A. Unflagged; byte-identical index sequence.

**Measured (historical).** Headless production renderer: deterministic instructions -0.202% (spread < 0.03%); guest trace identical over 500 race frames (L:7309eb0a, 2026-09-16).

### gx: touch the GXFIFO IRQ line only when its state changes

**What.** GPU3D::CheckFIFOIRQ calls SetIRQ/ClearIRQ only when the computed GXFIFO IRQ differs from IF[0]. Unflagged; on Android debug.litev.irqguard=0 restores the unconditional path for A/B.

**Why it works.** CmdFIFORead runs this every ~2 commands, and with no GXFIFO IRQ configured that was a redundant ClearIRQ -> UpdateIRQ each time. This is the ARM9 path and UpdateIRQ is idempotent when IF is unchanged, so the skipped calls are no-ops.

**Exactness.** A (900 in-race frames identical).

**Measured (historical).** On-device emulator-thread profile: NDS::ClearIRQ 0.50% self-time -> ~0; fps within noise (L:48ad5450, 2026-09-11).

### geom: skip the clipper for polygons fully inside the frustum

**What.** SubmitPolygon tests every vertex against all six planes with the clipper's own comparisons; when all are inside with W > 0 it applies only the idempotent 5-bit colour clamp and skips ClipPolygon. W <= 0 and straddling polygons take the full clipper. Android-only for now, behind debug.litev.trivclip (default on), so the host headless gate does not exercise it.

**Why it works.** ClipPolygon ran all three axes x two half-plane passes for every visible polygon, copying fully-inside vertices back unchanged. Vertex values, order and count are unchanged, so the guest-visible RAM_COUNT is identical.

**Exactness.** A (900 in-race frames identical, emulator state and framebuffers).

**Measured (historical).** On-device emulator-thread profile: the three ClipAgainstPlane passes 2.25% -> ~0 self-time, net ~1.7-1.9% of emulator-thread time (~0.35 ms/frame); fps A/B under the noise floor (L:f89508fa, 2026-09-11).

**Depends on.** gx: touch the GXFIFO IRQ line only when its state changes (shared prop helper)

## Infrastructure and diagnostics

### build: add LITEV option scaffold and re-export LITEV_* defines on core

**What.** Opens the "liteDS-v2 additions" section of the top-level CMakeLists.txt (later commits each append one option() block there) and adds a loop that re-exports every LITEV_* directory compile definition as a PUBLIC compile definition of the `core` target.

**Why it works.** add_compile_definitions() is directory-scoped. An out-of-tree target that links `core` (the Android JNI frontend doing `new NDS(...)`) would otherwise compile the core headers without the flags. Several flags change the layout of ARMJIT/Compiler, which is NDS's first member, so the frontend would allocate a smaller NDS than the constructor writes and crash during construction (L:1ca8b152, 2026-07-05: "dispatcher+link ON app boots and races (prev: 100% crash at construction)").

**Exactness.** A. No flag exists yet; in-tree builds are unchanged.

### android: port the OpenGL renderer to GLES3 and fix Mali attribute signedness

**What.** GLES_Compat.h shims for desktop-only GL entry points/enums, included from PlatformOGL.h on Android; OpenGLSupport.cpp rewrites the embedded shaders' `#version 140` to GLSL ES 3.00 and injects default precision; the 19 OpenGL_shaders/*.glsl get the GLSL ES strictness edits (FRAGLOC outputs, explicit conversions); GPU3D_OpenGL.cpp feeds the signed vPolygonAttr with GL_INT; net-utils links enet on ANDROID; MakeEmbed.cmake is referenced relative to the source dir so the lib builds as a subdirectory.

**Why it works.** The shaders were desktop GLSL 1.40 and do not compile on GLES3 drivers. The texcache encodes the "normal texture" sentinel as 0xFFFF0000 in a signed ivec3; the Mali-G52 driver mangles unsigned values >= 2^31 fed into a signed attribute, so every polygon rendered untextured until the attribute type matched the shader declaration.

**Exactness.** A for emulation (OpenGL renderer and build glue only).

### jit: install the fastmem fault handler only when fastmem is in use

**What.** ARMJIT_Global gains a ref-counted Acquire/ReleaseFaultHandler; ARMJIT_Memory takes the effective fastmem flag and installs or removes the handler via SetFastMemHandler(); ARMJIT keeps it in sync when fastmem is toggled at runtime (SetJITArgs). JIT init no longer installs it unconditionally.

**Why it works.** The SIGSEGV/SIGBUS handler assumes a fastmem context and interprets every fault relative to the fastmem arena. Installed with fastmem off or unsupported, it intercepts unrelated process-wide faults: on Android ART itself uses SIGSEGV, and on macOS/OpenBSD fastmem is unsupported yet the handler was installed anyway (L:9eb337fe).

**Exactness.** A. No behaviour change when fastmem is on and supported.

### diag: add LiteProfile frame-breakdown counters (LITEV_PROFILE)

**What.** LiteProfile.h: per-frame atomic counters and ScopeTimer phases that compile to nothing unless LITEV_PROFILE=1. Call sites in NDS::RunFrame (re-nested so each phase - ARM9, ARM7, DMA, GPU3D, RunSystem - has its own scope, plus per-event-type scheduler counts), ARM Execute, the ARMJIT slow-memory helpers, IsIdleLoop, SPU::Mix and the GX command/lighting paths.

**Why it works.** Per-phase nanosecond attribution for the headless harness. Most of the headless "bucket" numbers quoted by later commits (RunFrame, DMA, gpu3d, sched iterations) come from these counters. Counters for later levers (link sites, icache hits, idle hits) live in the header from the start; they stay zero until the code that bumps them exists.

**Exactness.** A. Everything compiles out at LITEV_PROFILE=0 (RunFrame is textually re-nested but equivalent).

**Measured (historical).** Not a lever. Kept OFF in the shipping app because the per-event ScopeTimer itself costs ~1.8 ms/frame on the A55 (simpleperf, app build.gradle.kts comment).

**Option.** LITEV_PROFILE (default OFF; diagnostic, not enabled in the shipping build)

### diag: add LitevSoftProf software-render phase profiler (LITEV_SOFTPROF)

**What.** LitevSoftProf.{h,cpp}: names every render thread and logs a per-60-frame decomposition (emu 3D/2D barrier and snapshot time, 3D clear/raster/final, 2D wall, render critical path). The LSP_* macros are no-ops when the flag is off; the call sites arrive with the renderer commits that own them.

**Why it works.** Separates "the emulator thread waited on the renderer" from "the renderer was slow". The bar3D/bar2D/EmuSnap figures quoted by the async-render commits (SOFT3D_ASYNC, TILE_COORD, SOFT2D_DEPTH2, SNAP_DIRTY) come from it.

**Exactness.** A. Observational only.

**Measured (historical).** Diagnostic. An earlier intrusive per-pixel effect census inside it cost the 2D worker ~5.9M atomics per 60 frames; that census is separately opt-in (perf-evidence §7, INVESTIGATION-LOG "Profiler perturbation removed").

**Option.** LITEV_SOFTPROF (default OFF; enabled in the app's shipping build)

### tools: add the liteDS-headless benchmark and byte-exact oracle harness

**What.** tools/headless: a frontend-free CLI runner (FreeBIOS direct boot, scripted input, savestate load/dump, fps windows, PPM framebuffer dumps) and the oracle modes --record-trace / --verify-trace (per-frame ARM9/ARM7 registers, timestamps, MainRAM hash and framebuffer hashes) and --verify-interp-converge. Adds NDS::GetSysTimestamp, LocalMP/MPInterface packet counters for the two-instance MP harness, the input scripts and the Shrek race README under baselines/, the redistributable Nitro Engine ne-multiplemodels test ROM, and tools/android-bench (NDK cross-build; `build.sh app` parses the app's build.gradle.kts so headless runs the shipping flag set).

**Why it works.** Every later commit is gated on it: a category-A lever must reproduce the per-frame trace of the build without it. Golden traces are generated outputs and are not committed; record them locally with --record-trace.

**Exactness.** A. Harness only; the core gains a read-only accessor and counters.

**Option.** LITEV_HEADLESS (default OFF; builds the harness)

### jit: add the CyclesBudget slice slot and ForceExecutionExit (LITEV_SHADOW_ASSERT)

**What.** ARM::CyclesBudget at a fixed offset (0xe8, next to the upstream FastBlockLookup fields) seeded in Execute<JIT>; ARM::ForceExecutionExit() zeroes it at every point the C++ loop would end a slice (halt, IRQ, GXFIFO stall, HALTCNT, CP15 writes, ...). ARMJIT_Offsets.h gains the CyclesBudget and FastBlockLookup* offsets, static_asserted against the real ARM layout in both JIT backends. LITEV_SHADOW_ASSERT aborts if the budget and the timestamp/target mechanisms ever disagree.

**Why it works.** The emitted dispatcher (next commit) stays in generated code across block hops and needs a single, register-cheap "slice over" test. This lands the slot in shadow mode - nothing reads it for control flow yet - so its equivalence with Timestamp >= Target can be asserted on its own.

**Exactness.** A. Inert without LITEV_JIT_DISPATCH.

**Option.** LITEV_SHADOW_ASSERT (default OFF; diagnostic, not enabled in the shipping build)

### jit: emit a simpleperf perf map for generated code (LITEV_JIT_PERFMAP)

**What.** The A64 backend appends one `<addr> <size> <name>` line per emitted region (blocks, dispatcher, commit/jumpto stubs, patched load/store helpers) to a perf-<pid>.map. Runtime-gated: nothing is written unless LITEV_PERFMAP_DIR or (Android) debug.litev.perfmap names an output directory.

**Why it works.** The JIT blob is otherwise one opaque anonymous mapping. A profile with a large unattributed share cannot be trusted; this resolved the ~45% of L1I and L2 samples that sat in the unsymbolized blob (L:60eb6571).

**Exactness.** A. Observational: emits no host instructions and never advances the code pointer.

**Measured (historical).** Diagnostic; zero cost unless armed at run time.

**Option.** LITEV_JIT_PERFMAP (default OFF; enabled in the app's shipping build)

### diag: histogram slow-helper calls by memory region (LITEV_SLOWMEM_HIST)

**What.** Tallies every ARM9 slow-helper call by ClassifyAddress9 region and dumps SLOWHIST lines to stderr at exit; SLOWHIST() is a no-op otherwise.

**Why it works.** Tells whether residual slow-memory cost is fastmem-recoverable (MainRAM, DTCM) or genuinely unmapped (I/O, VRAM, palette, OAM). This is how the DTCM fastmem bug was found.

**Exactness.** A.

**Measured (historical).** Diagnostic.

**Option.** LITEV_SLOWMEM_HIST (default OFF; diagnostic, not enabled in the shipping build)
