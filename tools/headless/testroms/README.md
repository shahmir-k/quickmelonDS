# Test ROMs

This directory holds test ROMs used by the liteDS-v2 headless oracle.

Two classes:

1. **`redistributable/` — committed.** Open-source homebrew whose license
   permits shipping the ROM binary itself. These `.nds` files **are committed**
   (see `.gitignore`, which re-includes `redistributable/*.nds`) so the
   benchmark is reproducible by anyone with just `git clone`. See
   *Redistributable benchmark ROMs* below.
2. **Everything else — git-ignored, never committed.** Third-party ROMs with no
   redistribution license (Shrek commercial dump, the instruction testers). The
   user authorized downloading/using these locally; only their *scripts, traces,
   and metadata* (this file) are committed. Repopulate a fresh checkout with the
   fetch commands in the inventory below (and copy the Shrek ROM from its local
   path).

## Scripted input (`--input-script`)

Menu-driven test ROMs need input to reach their tests. The headless runner
accepts `--input-script <file>`: newline directives `<frame> <keys>` meaning
"hold exactly these keys from this frame onward" (level, not edge). `<keys>` is
a comma list from `{A,B,SELECT,START,RIGHT,LEFT,UP,DOWN,R,L,X,Y}`, the word
`NONE`, or a `0x` hex pressed-mask (bit layout `0:A 1:B 2:SELECT 3:START
4:RIGHT 5:LEFT 6:UP 7:DOWN 8:R 9:L 10:X 11:Y`). The DS keymask is active-low;
the runner inverts for you. Scripts apply to every run mode (benchmark, record,
verify, converge). Because scripted input is part of the deterministic input, a
golden trace recorded with a script embeds the script's xxhash in its header and
`--verify-trace` warns + mismatches if replayed under a different/absent script.

Committed input scripts live in `../baselines/*.script`.

## Inventory

| ROM | Source URL | sha256 | License | Script (`../baselines/`) | Validates | Oracle result |
|---|---|---|---|---|---|---|
| `shrek.nds` | local: `/Users/shahmir/Documents/GitHub/quickmelonDS/.tmp-shrek.nds` | `7428867c…ec9e8` | commercial (Activision); user's own dump, local only | `shrek-menu-advance.script` (demo) | input-script feature proof (A on menu → "SELECT GAME TYPE") | A-press diverges FB hash vs no-script (proves scripted input reaches emulation); see also `shrek-600*.trace` |
| `armwrestler.nds` | https://raw.githubusercontent.com/Atem2069/armwrestler-fixed/main/armwrestler.nds | `1512de79…12531` | no LICENSE (all-rights-reserved); mic- ARMWrestler DS + Atem2069 hardware fixes | `armwrestler-arm-600.script` | ARM9 instructions: ARM ALU pt1, ALU pt2/MISC, ARM LDR/STR, ARM LDM/STM | interp: **all OK**. jit-full-stack: **byte-identical** final screen |
| `rockwrestler.nds` | https://raw.githubusercontent.com/RockPolish/rockwrestler/master/rockwrestler.nds | `f905e165…6069a` | no LICENSE (all-rights-reserved); RockPolish DS tester | `rockwrestler-600.script` | ARMv4 condition-code / instruction sub-tests (driven subset) | interp: **OK**. jit-full-stack: **byte-identical** final screen |

Full sha256:
- shrek:        `7428867c7447e73eaf2dc10653582a009b99cee019fc903c2084217e5ffec9e8`
- armwrestler:  `1512de7953f9e4eb8376f54e3476b2e01f0639465efb8fbf9b709a394e212531`
- rockwrestler: `f905e16510b00500d432b62dbfafc33e9a7179187475981fe74d9e691a96069a`

Fetch commands (run from this directory):

```
cp /Users/shahmir/Documents/GitHub/quickmelonDS/.tmp-shrek.nds shrek.nds
curl -sL -o armwrestler.nds  https://raw.githubusercontent.com/Atem2069/armwrestler-fixed/main/armwrestler.nds
curl -sL -o rockwrestler.nds https://raw.githubusercontent.com/RockPolish/rockwrestler/master/rockwrestler.nds
```

## CPU-correctness oracle

For each test ROM, run to test-completion twice and compare the final screen:
reference = `--mode interp` (exact-timing build, e.g. `build-host`); candidate =
`--mode jit` with the **full flag stack** (`build-mem-es`:
`LITEV_JIT_DISPATCH + LINK_* + EVENT_SLICES + MEM_DTCM_BLOCK + MEM_MAINRAM_LOAD`).

```
# reference (interpreter)
build-host/liteDS-headless   --rom testroms/armwrestler.nds --frames 600 --mode interp \
    --input-script baselines/armwrestler-arm-600.script --fb-dump-ppm 599:/tmp/aw-interp.ppm
# candidate (JIT, full stack)
build-mem-es/liteDS-headless --rom testroms/armwrestler.nds --frames 600 --mode jit \
    --input-script baselines/armwrestler-arm-600.script --fb-dump-ppm 599:/tmp/aw-jit.ppm
cmp /tmp/aw-interp.ppm /tmp/aw-jit.ppm   # byte-identical
```

Results (frame 600, this branch):

| ROM | interp = reference passes? | jit-full-stack final screen | final_top / final_bot hash |
|---|---|---|---|
| armwrestler | yes — ARM ALU/LDR-STR/LDM-STM all **OK** | **byte-identical** to interp | `aff3a5a037969248` / `bcf2cf1d8d3b4974` |
| rockwrestler | yes — driven ARMv4 sub-tests **OK** | **byte-identical** to interp | `6086c6af0e759b1d` / `bcf2cf1d8d3b4974` |

No test flagged a failure under either mode for the driven subsets. (Upstream
melonDS is known to fail some ARMWrestler edge cases even on interp; none were
observed in the ARM ALU/LDR-STR/LDM-STM pages driven here.)

## Golden traces

Per-frame state traces (exact-timing config, `--record-trace`, double-recorded
byte-identical) are committed at `../baselines/<rom>.trace` + `.trace.json`.
Verify with the same script:

```
build-host/liteDS-headless --rom testroms/armwrestler.nds \
    --input-script baselines/armwrestler-arm-600.script \
    --verify-trace baselines/armwrestler-arm-600.trace
```

## Coverage gaps / TODO

- **ARMWrestler THUMB + ARM vsTE** suites (menu items 4–7) are not yet scripted;
  only the ARM instruction suite is driven.
- **Rockwrestler** ARMv5, IPC, DS MATH, MEMORY, INITIAL STATE, and the **ARM7**
  tab are reachable via its hierarchical menu but not yet scripted.
- **Arisotura/arm7wrestler** (melonDS author's dedicated ARM7 tester) is
  source-only — https://github.com/Arisotura/arm7wrestler — no prebuilt `.nds`;
  build with devkitARM if ARM7-specific coverage is needed.
- **B.4 commercial ROMs** still needed for the full compat oracle: Pokémon,
  NSMB, Mario Kart (the U5 GXFIFO-interleave watch item), WarioWare, and an SMC
  stressor. Only Shrek is locally available.

---

# Redistributable benchmark ROMs

The existing suite has a gap for a *permanent, anyone-can-reproduce* perf
benchmark: Shrek is a commercial dump (local-only, not committable) and the two
instruction testers (`armwrestler`/`rockwrestler`) sit in tight ALU loops with
no meaningful renderer/CPU load. This section adds an open-source homebrew ROM
with a real, sustained 3D+2D workload whose **binary is committed** to the repo.

Constraint that drove the choice: **the headless harness has no FAT/nitroFS
backend** (no `argv`, no DLDI/SD). Homebrew that loads assets at runtime from
nitroFS fails at boot (e.g. DS-Craft only ever renders an "Error initializing
FAT / Function not implemented" screen). The ROM must therefore embed all
assets in the ARM9 binary (`bin2s`/`grit`), which Nitro Engine's examples do.

## Inventory (committed — `redistributable/`)

| ROM | Source | sha256 | License | Input | Workload |
|---|---|---|---|---|---|
| `redistributable/ne-multiplemodels.nds` | Nitro Engine `MultipleModels` example, prebuilt by the engine author (Antonio Niño Díaz / AntonioND). From `ne_examples.rar` at http://www.skylyrac.net/old-site/downloads/ne_examples.rar (extract with `bsdtar -xf`). | `d47d3f8856438a291c0df0de599f5fc1ff98a71a141f6207716deb3733c87605` | **Fully redistributable.** Nitro Engine engine code = **MIT**; its examples = **CC0-1.0** (https://github.com/AntonioND/nitro-engine `readme.rst` + `licenses/`); linked **libnds = Zlib**. Every component permits shipping the compiled binary. | none (auto-runs its render loop) | 16 flat-shaded 3D models, continuously rotating |

- ROM sha1: `912acc15b42b7e4cb5daed34c3b65d92a329ac56`, size 162,880 bytes.
- Provenance: source archive `ne_examples.rar` sha256
  `5aba7c6a7703b8eacd74fde8ee3680a87316540b51bc56f656d375e6e093b19c`.

### What it loads

Boots straight into an animated 3D scene — no menu, no input, deterministic
(double-run framebuffer/audio hashes are byte-identical; animation is
frame-driven, not RTC-driven). The on-screen readout confirms the geometry:
`Polygon RAM: 1280`, `Vertex RAM: 3840`.

- **ARM9** — submits 16 models' geometry every frame; per-model matrix-stack math
  and a rotation update per frame.
- **GPU3D** — full geometry pipeline (vertex transform + shading) plus **software
  rasterization of ~1280 polys/frame**. This is the dominant cost: frameskip 9
  more than *doubles* FPS (see below), i.e. the workload is rasterization-bound.
- **GPU2D** — top-screen BG text layer compositing (the RAM readout).
- **SPU** runs (silent); minimal ARM7; no WiFi; no FAT/nitroFS.

So it exercises the 3D geometry engine, the 3D software rasterizer, and a 2D
text layer — the subsystems the instruction testers never touch.

### Golden traces

Both recorded with `--record-trace`, **double-recorded byte-identical**, exactly
like the Shrek goldens (which needed two variants for the same reason):

| Trace | Config family | Verified OK on |
|---|---|---|
| `../baselines/ne-multiplemodels-600.trace` | exact-timing (`LITEV_EVENT_SLICES=OFF`) | baseline (`build-host`), dispatch (`build-linkoff`), link (`build-dispatch`) |
| `../baselines/ne-multiplemodels-600-eventslices.trace` | event-slices (`LITEV_EVENT_SLICES=ON`) | es (`build-es`), full (`build-mem-es`) |

The two are intentionally *not* bit-comparable (event-slices coarsens the
ARM9/ARM7 interleave — same caveat as `shrek-600-eventslices.trace`). Verify:

```
build-host/liteDS-headless --rom testroms/redistributable/ne-multiplemodels.nds \
    --frames 600 --mode jit --verify-trace baselines/ne-multiplemodels-600.trace
```

## Host benchmark results

Apple Silicon host, JIT, software renderer (OGL off), 2400-frame run, steady-state
window `--bench-window 200:2399`, **median of 3**. Config→build-dir map matches
`tools/android-bench/build.sh`'s cumulative flag stack. (Steady state is
immediate — `window_fps` tracks overall `avg_fps` within 0.2% — so plain
`--frames` full-run FPS reproduces these numbers on any harness build, with or
without `--bench-window`.)

### Frameskip 0 (median window_fps)

| config | build dir | window_fps | Δ vs baseline |
|---|---|---|---|
| baseline | `build-host`    | 353.98 | — |
| dispatch | `build-linkoff` | 356.01 | +0.6% |
| link     | `build-dispatch`| 354.00 | +0.0% |
| es       | `build-es`      | 375.54 | +6.1% |
| full     | `build-mem-es`  | 377.36 | +6.6% |

The whole win is at **event-slices** (`es`); dispatch/link are flat because on
the host software renderer this ROM is renderer-bound — precisely the pattern
Appendix C records for Shrek on host ("U3/U4 flat on host, renderer-bound; U5
event-slices is the host win"). This ROM reproduces that behaviour with a fully
committable binary.

### Frameskip 9

Only meaningful on a build compiled with `LITEV_AGGRESSIVE_SKIP=ON`. The reused
`baseline/dispatch/link/es` host build dirs were configured with the flag OFF
(so their `--frameskip` is inert — `fs9 == fs0`); `build.sh` compiles the flag
into **all** configs, so a full fs9 sweep is available on device. Measured on the
`full` config rebuilt with `-DLITEV_AGGRESSIVE_SKIP=ON`:

| full config, skip build | window_fps |
|---|---|
| frameskip 0 | 374.78 |
| frameskip 9 | **824.09** (+120%) |

More-than-doubling under 9:1 frameskip confirms the workload is rasterization-
bound (skipping 9 of every 10 frames' rasterization is what pays).

### Does it actually stress the emulator? (anchors, same window, baseline, fs0)

| ROM | baseline window_fps | relative load |
|---|---|---|
| **ne-multiplemodels** | ~355 | 1.0× (this benchmark) |
| Shrek — **menu screen** | ~833 | 0.43× (2.3× lighter) |
| armwrestler (ALU tester) | ~1079 | 0.33× (3.0× lighter) |

MultipleModels runs at roughly **one-third** the FPS of the trivial ALU tester
and **under half** the FPS of the Shrek *menu* — i.e. it is a genuinely heavier,
sustained workload, which is the whole point (the Shrek menu is a near-static
screen; gameplay isn't reachable unattended, and the dump isn't committable).
(The Shrek-*menu* `full`-config number is omitted: it was too noisy to trust in
this run — `[635, 239, 207]` — the menu's load varies frame-to-frame and the run
overlapped other host activity. The baseline anchor above is stable.)

## Reproduce from a clean clone

The ROM is committed, so no download is needed:

```
# 1. build the headless harness (baseline config shown; see android-bench/build.sh
#    or docs/liteDS-v2-plan.md for the other four LITEV configs)
cmake -B build-host -DLITEV_HEADLESS=ON -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_QT_SDL=OFF -DENABLE_OGLRENDERER=OFF && cmake --build build-host -j

# 2. run the benchmark (one command)
build-host/liteDS-headless --rom tools/headless/testroms/redistributable/ne-multiplemodels.nds \
      --frames 2400 --mode jit
#    -> avg_fps line is the benchmark number; add --bench-window 200:2399 if your
#       harness build has it (optional — steady state is immediate).
```

## Other examples in the same (MIT/CC0/Zlib) archive

`ne_examples.rar` also contains `BoxTower.nds` (~1747 fps, lighter 3D),
`LoadSimpleModel.nds` (~935 fps, one animated textured model),
`ScreenEffects.nds` (~763 fps, 2D screen effects). `MultipleModels` was chosen
as the heaviest auto-running scene. `StylusTextureDrawing.nds` needs touch input
and was skipped. Any of these could be added the same way if a second, lighter
or 2D-focused benchmark is wanted.
