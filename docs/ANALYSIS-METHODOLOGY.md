# Analysis Methodology — diagnosing "an optimisation broke a game" by differential comparison

The method for diagnosing a per-game bug (wrong pixels, hang, desync) in this history: the lib
branch `litev-clean` and the app `melonDS-android-app-clean` branch `app-clean`. Building is in
`BUILD-METHODOLOGY.md` and the correctness gates are tiered in `TESTING-METHODOLOGY.md` (T0). Follow
this document in order. It is a **funnel**: each phase halves the search space by comparing our
build with a **known-good oracle**, never by guessing. If you find yourself toggling things at
random, switching repos, or arguing from memory, stop and come back here.

---

## Contents

1. [The oracles](#1-the-oracles)
2. [Step 0: localise before theorising](#2-step-0-localise-before-theorising)
3. [The funnel](#3-the-funnel)
4. [Gates and their blind spots](#4-gates-and-their-blind-spots)
5. [Reading the renderer path (only once the renderer is implicated)](#5-reading-the-renderer-path)
6. [Hard rules](#6-hard-rules)
7. [Anti-patterns](#7-anti-patterns)
8. [Worked example: Pokémon White "white professor"](#8-worked-example)
9. [Command reference](#9-command-reference)

---

## 1. The oracles

This fork is a stack of `LITEV_*` options on top of upstream melonDS, which is already correct. That
gives three exact oracles, each of which tells you the right answer for any input:

| Oracle | How to get it | Catches |
|---|---|---|
| **Upstream behaviour** | build with **all `LITEV_*` OFF** (`-DLITEV_HEADLESS=ON` only), or `litev-clean`'s base commit `10a173b5` | any bug introduced by our options |
| **The interpreter** | the same build, `--mode interp` | **JIT codegen** bugs (the interpreter ignores the JIT) |
| **An earlier commit** | a `litev-clean` prefix: every option is one commit, so the history itself is a bisect axis | *which* option introduced the regression |

The interpreter stands in for hardware: JIT authors debug by running the JIT beside the interpreter
and finding the first point where they disagree. **Every per-game bug we create is a divergence
between our build and one of these oracles.** The job is to localise that divergence from "a wrong
screen" down to "this instruction / option / line" by diffing at ever finer grain.

## 2. Step 0: localise before theorising

A wrong-pixels symptom does **not** mean the renderer is wrong; the data feeding it may be. Two cheap
tests split renderer from emulation in minutes, with no code reading. Do both before forming a
theory.

1. **Uncommitted changes.** Run `git status` and `git diff` in the lib tree you built from. An
   uncommitted experiment (a register-pin probe, a JIT lever, anything marked PROBE or WIP) is the
   most likely cause of a bug that just appeared, and it is invisible to `git log`, `git log -S` and
   `git bisect`. Build only committed trees (`git archive`, BUILD §3) so that the question cannot
   arise for a measured or shipped build.
2. **JIT vs interpreter A/B on the exact repro.** Render the same savestate both ways headless:
   ```sh
   liteDS-headless --rom R.nds --savestate S.ml1 --frames 60 --fixed-rtc 1600000000 --mode interp --fb-dump-ppm 59:int.ppm
   liteDS-headless --rom R.nds --savestate S.ml1 --frames 60 --fixed-rtc 1600000000 --mode jit    --fb-dump-ppm 59:jit.ppm
   ```
   Interpreter right and JIT wrong ⇒ a **JIT** bug (codegen, register allocation, flags, dispatch):
   stop reading GPU code. Both wrong ⇒ a shared emulation-core option (memory, DMA, timing) or a
   renderer bug; continue with the funnel. With the shipping set, JIT ≠ interp at the byte level is
   expected (the category-B timing options do not apply to the interpreter, TESTING §3.11), so use
   this A/B to answer "does the interpreter render it right?", not as a byte-equality gate.

Read the right screen while characterising the symptom: on the RG DS the game is on **Display 1**
(`su 0 screencap -p -d 1`; BUILD §9). A Display 0 capture looks like nothing renders.

## 3. The funnel

### Phase 0: reproduce deterministically, off-device
- Pull the exact ROM and savestate from the device as root (BUILD §11).
- Run headless with `--fixed-rtc` and a fixed `--frames`. **Confirm determinism:** the same command
  twice gives the same guest hash. If not, find the nondeterminism first (RTC, a time-driven
  animation such as a blinking dialog cursor: mask that region or compare a settled frame). Under
  `TILE_COORD` the framebuffer hash itself is async (TESTING §3.8), so compare settled PPMs with a
  same-binary control.
- State the symptom precisely: which screen, which region, which frames, and what renders correctly
  beside it. "Professor illustration solid white; dialog box fine; 60 fps", not "Pokémon looks
  broken". The healthy signals exclude whole classes of causes.

### Phase 1: is it our code at all?
Build all-`LITEV_*`-OFF and run the repro.
- Bug **gone** → one of our options (the common case). Go to Phase 2.
- Bug **still there** → upstream or an unflagged change. In this history the unflagged changes are
  individual commits too (LITEV-OPTIMIZATIONS "Unflagged optimizations"), so bisect `litev-clean`.

### Phase 2: which option?
Two equivalent O(log n) searches; pick whichever is cheaper to build:
- **Bisect the flag set:** turn on half of the `-DLITEV_*` list from `app/build.gradle.kts`, test,
  keep the failing half, repeat. Respect dependencies (several options refuse to configure without
  their prerequisite; include the minimal required set).
- **Bisect the history:** `git bisect` over `litev-clean` from `10a173b5` to the tip, building the
  app's flag list at each step (options that do not exist yet are ignored by CMake).

### Phase 3: which layer, and a gate that can see the bug
With the culprit on, compare `--mode jit` with `--mode interp` (as in step 0). Then choose the
equivalence check that can actually observe the failure (§4). If the symptom is visual, the gate
must include pixels and the relevant GPU counter, never CPU state alone.

### Phase 4: which operation? Narrow in time, then in space
- **In time:** find the first frame where our build and the oracle diverge in the chosen signal. A
  3D layer that goes blank because polygons are **dropped** points at a wrong control-flow decision;
  one with **mis-transformed** polygons points at arithmetic.
- **In space:** instrument the subsystem that produces the divergent signal and diff against the
  oracle. Geometry: log GXFIFO commands and polygon submissions and find the first that differs,
  then the guest instruction that produced it. JIT: dump the compiled block for that guest PC and
  compare its flag and branch handling with the interpreter's.

### Phase 5: fix minimally, re-validate against the oracle
- Read the exact codegen or handler and compare it with the reference (interpreter handler or
  upstream JIT). Fix the smallest thing.
- Re-validate with the Phase 3 gate on **at least two games**: the repro and a regression check.
  A byte-exact pass on Shrek alone is never "correct": Shrek exercises one narrow path.

## 4. Gates and their blind spots

| Gate | Sees | Blind to |
|---|---|---|
| `--record-trace` / `tracediff.py` (or `--verify-trace`) | CPU registers and main RAM per frame | **GPU-only divergence** (geometry, VRAM), async render, audio |
| Framebuffer: `--fb-dump-ppm` + `ppmdiff.py` (and `final_top`) | the visible pixels | *why* (which layer); async capture jitter and time-driven animation add noise |
| `RenderNumPolygons` and the geometry counters | 3D geometry dropped vs present | 2D, textures |
| Register reads (`DispCnt`, `VRAMMap_*`, OBJ/3D line counts) | which hardware mode is actually active | anything upstream of the register write |

**A green state trace does not mean correct.** A JIT bug can mis-branch inside the geometry
submission, emitting wrong GPU commands as a write-only side effect while CPU and RAM re-converge;
the trace stays green for hundreds of frames. Match the gate to the signal that is actually wrong.

## 5. Reading the renderer path

Only once step 0 and Phase 3 put the bug in the renderer. Reading forms a **hypothesis**; reproduce
and instrument before you write down a root cause.

1. **Map the symptom to a hardware mechanism.** Turn it into a short list of testable claims rather
   than falling for the first plausible one:

   | Symptom | Mechanisms to test |
   |---|---|
   | Whole screen or region solid white/blank, engine running | DISPCNT display mode (bits 16–17: 0 off, 1 BG/OBJ, 2 VRAM bitmap (engine A only), 3 FIFO); the 3D layer blank (geometry missing or mis-transformed, render list empty); **or the CPU produced the wrong state** (JIT miscompile, step 0) |
   | Missing or garbled sprite | OBJ/OAM attributes, OBJ VRAM mapping |
   | Wrong colours, washed out, too dark | palette RAM, master brightness, blend EVA/EVB, colour-effect select |
   | Garbage tiles, scrambled BG | VRAMCNT bank mapping, BG base/screen block, tile vs bitmap mode |
   | Gaps in 3D geometry | clip/cull, near-plane clip, W-buffer, polygon RAM overflow |
   | Correct still frame, wrong motion | mid-frame scanline effects (scroll, window, blend), capture, DMA timing |

2. **Read the actual values before choosing.** Print the registers and state the hypothesis is
   about from the exact repro: `(DispCnt >> 16) & 3`, `RenderNumPolygons`, `VRAMMap_LCDC`, OBJ/3D
   line counts, and per-frame PPM dumps. One headless run refutes a wrong theory faster than any
   amount of reading.
3. **Read the whole output path, not one file.** The fork moved code when it split the 2D pipeline:
   `SoftRenderer2D::DrawScanline` (`GPU2D_Soft.cpp`) handles only BG/OBJ, and the display-mode
   `switch` lives downstream in `SoftRenderer::DrawScanlineA/B` (`GPU_Soft.cpp`). A missing branch in
   the first file you open is a hypothesis; grep the whole tree for the feature before concluding it
   is unhandled.
4. **Diff against a known-good reference.** `git show <ref>:<path>` shows what the base did;
   `git log -S"<code string>" -- <path>` finds the commit that added or removed it. Caveats: `-S` on
   one path cannot see code that was **moved** to another file, and it never sees **uncommitted**
   code (step 0). In this history the natural reference is the commit just before the suspected
   option, or the base `10a173b5`.
5. **Confirm any timeline from primary sources:** committed code and git history first, then the
   development logs and ledger. The later event and the committed code win over a stale document or
   a recollection. Mark gaps UNCERTAIN rather than filling them with a plausible story.

## 6. Hard rules

1. **Know which tree you are debugging.** The measured and shipped trees are `litev-clean` at the
   commit the app pins and `app-clean`. Confirm the repo, branch, commit and a clean `git status`
   before editing anything, and fix on that tree.
2. **Localise renderer vs emulation first** (step 0): uncommitted-diff check plus the JIT-vs-interp
   framebuffer A/B.
3. **Diff against an oracle; never reason in a vacuum.** If you are about to assert a root cause
   without a diff or a measured value that shows it, you are guessing.
4. **Know your gate's blind spot** (§4). The state trace is blind to GPU-only, async and audio
   divergence.
5. **Bisect options, not games.** O(log n) over the flag list or the `litev-clean` history.
6. **Validate on more than one game.** Pokémon White (`.ml1` on the device SD card) is the standard
   second game: its intro uses display mode 1 with a 3D-billboard professor and is sensitive to JIT
   codegen.

## 7. Anti-patterns

- **Concluding from one file.** "The display-mode switch is missing" was true of one file and false
  of the build.
- **Asserting a root cause from code reading alone.** Two confident, written-down root causes were
  wrong in the worked example; one instrumented run refuted each.
- **Inventing a timeline.** "It has been broken for months since commit X" was fabricated; the cause
  was the newest, uncommitted change. Check the working tree, then establish history from git.
- **"Don't build, just read."** The targeted A/B (JIT vs interp, probe in vs probe out) is what found
  the bug. Reason first, then run the one A/B that splits the search space. Random flag toggling is
  still noise.
- **Per-game `git bisect` of a game with no reliable repro.** It answers "which commit changed a
  hash", not "why", and needs a repro you may not have. Bisect options against a deterministic repro.
- **Reading the wrong screen** (Display 0) and concluding nothing renders.
- **Shrek-only validation.** A change that byte-matches Shrek can still break another game's path.

## 8. Worked example

Pokémon White, 2026-10-01, on the pre-history tree (`melonDS-android-lib` @ `liteDS-v2-android`):
the professor illustration in the intro is solid white, the dialog box below it renders, 60 fps.

- **Two wrong renderer theories came first.** (1) "Display mode 2 is dropped": the switch had only
  moved to `GPU_Soft.cpp`, and the real `DispCnt = 0x00011B18` is mode 1. (2) "The 3D geometry is
  missing": the savestate holds the professor's polygons. Both were refuted by reading the actual
  values, which should have been done before writing either down.
- **Step 0 settled it.** Rendered headless from the pulled ROM and `.ml1`, the interpreter drew the
  professor and the JIT drew white (different framebuffer hashes): a JIT bug, renderer exonerated.
- **The JIT causes on record.** The day's records describe two JIT defects with the same symptom:
  - a committed `LITEV_JIT_LAZYFLAGS` defect: guest C/V flags deferred in host PSTATE across an
    instruction boundary were clobbered before being materialised, so a later guest conditional read
    stale carry/overflow and mis-branched in the geometry submission. The fix materialises eagerly
    whenever C or V is deferred and keeps the N/Z-only deferral. **This history's
    `LITEV_JIT_LAZYFLAGS` commit already contains that fix.**
  - an **uncommitted** `src/ARMJIT_A64/` probe that moved the JIT's CPSR register from W27 to W16.
    W16 is AArch64 IP0, which linker veneers and the emitter's far-call paths clobber, so guest flag
    state was corrupted across calls. Removing it (`git stash push src/ARMJIT_A64/*`, rebuild) made
    the JIT render the professor again. It was never committed and is not in this history.

  **Status: fixed.** With the lazy-flags fix, Pokémon White's JIT and interpreter framebuffers match
  (final top-screen hash `d6774491583a8cf7`, the professor rendered correctly); the pre-fix build
  (`e9b8222f`) gives JIT != interpreter. This gate runs on every commit from the `LITEV_JIT_LAZYFLAGS`
  commit onward.
- **Why the gates missed it.** The 300-frame state trace stayed green (GPU-only divergence, §4), and
  every change had been validated on Shrek only, which never exercises the pattern. Only a
  framebuffer diff against the interpreter on a second game exposes it.

## 9. Command reference

```sh
# host headless from a committed tree with the app's flag list (BUILD §11)
cd <lib tree>
flags=( $(grep -vE '^[[:space:]]*//' ../app/build.gradle.kts | grep -oE '"-DLITEV_[A-Z0-9_]+=(ON|OFF)"' | tr -d '"' | grep -v LITEV_PROFILE=) )
cmake -S . -B bh -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_QT_SDL=OFF -DENABLE_OGLRENDERER=OFF \
  -DENABLE_GDBSTUB=OFF -DENABLE_JIT=ON -DLITEV_HEADLESS=ON -DLITEV_PROFILE=OFF "${flags[@]}"
cmake --build bh --target liteDS-headless -j 8
# upstream oracle: the same command with no "${flags[@]}"

# JIT vs interpreter, deterministic, settled frame
bh/liteDS-headless --rom R.nds --savestate S.ml1 --frames 120 --fixed-rtc 1600000000 --mode interp --fb-dump-ppm 119:ref.ppm
bh/liteDS-headless --rom R.nds --savestate S.ml1 --frames 120 --fixed-rtc 1600000000 --mode jit    --fb-dump-ppm 119:our.ppm
# then diff the PPMs and RenderNumPolygons; repeat once to rule out async capture jitter

# guest-state trace (blind to GPU-only divergence)
bh/liteDS-headless ... --mode interp --record-trace ref.trace
bh/liteDS-headless ... --mode jit    --record-trace our.trace   # compare with tracediff.py
```
