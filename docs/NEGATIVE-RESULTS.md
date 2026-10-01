# Negative results and dropped levers

These levers were built and measured during development but are **not** in this history. They
are recorded here so nobody repeats them without a new idea. Numbers come from the development
log (lib/app commit messages, `PATH-TO-60FPS-2026-07-17`, the measurements ledger); the same
caveat as in [LITEV-OPTIMIZATIONS.md](LITEV-OPTIMIZATIONS.md) applies: numbers from different
eras are not comparable. Methodology: [BUILD-METHODOLOGY.md](BUILD-METHODOLOGY.md),
[TESTING-METHODOLOGY.md](TESTING-METHODOLOGY.md).

Two lessons recur. On the RK3566 the four cores are balanced and fully used, so moving work from
the emulator thread to another thread usually just makes the emulator wait for it (the "4-core
wall"). And on the in-order A55, levers that cut instructions but not cache misses or
mispredicts tend to vanish into stall shadows.

## Regressions or no gain

| Lever | What it did | Result | Source |
|---|---|---|---|
| `LITEV_RELAXED_ARM9_TIMING` | Flat cycles-per-instruction for ARM9 memory accesses (DraStic-style) | **-4.8% fps on the A55** (ARM9 7.76 -> 9.11 ms), -6.0% on host, and not MP-safe. (A "-27%" figure in the log is a bundle with two other flags.) | L:8e761d33, 2026-07-05 |
| `LITEV_GEOM_RECIP` | `frecpe`+Newton reciprocal for the SubmitPolygon viewport/Z divides | On-device iso-thermal A/B **null** (<0.3 ms, within noise), and approximate: 4.9% of pixels differ | L:292aafe2 / A:050b5eb6, 2026-07-11 |
| `LITEV_SPU_OFFLOAD` | Pan/mix/blip of the SPU on a worker thread (decode stayed on the emulator thread) | Correct and MP-safe, but a regression on 4 cores: -1.3 to -2 fps; 2 bands + offload 50.70 vs the 3-band baseline 55.83 fps | L:cf2eddbc, session log 2026-09-13 |
| `LITEV_GEOM_OFFLOAD` | Record the GX command stream, replay the vertex transform on a helper thread | ~0 fps and a garbled device render (same-thread stage); coordinator variant flat or worse | gradle notes; d45e1cb8 "work can't move on 4 cores" |
| `LITEV_REGION_TIMING` | Per-region instead of per-address memory-timing tables (dTLB relief) | Byte-exact but null for Shrek | L:e093d4a3 |
| `LITEV_MEM_SWTABLE` (+ store variant) | Software page table for JIT loads/stores instead of fault-based fastmem | Load path is 7-9 host instructions vs 1 for native fastmem. Replaced by native fastmem: app 38.3 -> 39.2 fps | L:5cd17c5e, L:4ac8b0f5, A:8e3c9223 |
| Lazy-flags V1 | Kept a register-resident CPSR and merged on flush | +1.58% instructions/frame, 39.38 -> 38.70 fps. V2/V3 (shipped as `LITEV_JIT_LAZYFLAGS`) fixed it | FETCH-STALL-SESSION-2026-07-16 |
| `LITEV_JIT_HOTLAYOUT` | Hot/cold block placement in the code cache | -9% L1I refills (MPKI 6.48 -> 5.90) but flat fps | L:d45e1cb8 |
| `LITEV_GEOM_NOSORTOPAQUE` | Skip the opaque-polygon sort | -6.6% emulator-thread L2 refills, but more overdraw on the raster workers: app fps flat, headless single-core -3.2% | L:b0d9d713, reverted 03163bb0 |
| Shallow `GEOM_PREFETCH` | One-polygon-ahead prefetch in BuildFrameGeom | Self-time -0.9 to -1.05 pp but fps null (54.72 vs 54.29); the prefetch arrived after the miss. Replaced by the staged `LITEV_GEOM_PREFETCH2` | L:85ff8d08 |
| `RENDER_4CORE`, `SOFT3D_STREAM` | More render threads / streamed 3D | 44.5 -> 43.2 fps and 44.5 -> 41 fps | L:687188a4, 2026-07-12 |
| `LITEV_GXFIFO_BATCH` | Batched GX dispatch, predecessor of `LITEV_GXFIFO_THREADED` | -1.1% GPU3D time on host; superseded | L:629b35b6 |
| Initial-exec TLS for `NDS::Current` | Part of the original `LITEV_LINKOPT` | `dlopen` fails: bionic rejects the IE model for a dlopen'd library's own TLS. The TLS cost was later removed by passing the `ARM*` to the JIT helpers instead | NDS.h notes, L:fb262986 |
| Huge pages | dTLB relief | Not available: no THP on the device kernel | profiler notes, 2026-09-16 |
| DMA::Run9 prefetch, NDS struct co-location | dTLB/L2 drills | Washed: the Run9 stall is dTLB, not L2; co-location moved nothing after the tlsdesc fix | profiler docs 2026-09-16/17 |

## Real gains on a renderer that no longer ships

`LITEV_SOFT3D_DRASTIC` replaces `SoftRenderer3D` with the tile renderer, so levers that live only
inside `SoftRenderer3D` do nothing in the shipping configuration. Their measured gains were real
on the old renderer:

| Lever | Result on the old renderer | Source |
|---|---|---|
| `LITEV_SOFT3D_BANDED` | Band-parallel 3D raster: "fast bundle" with FAST +14% headless; persistent band pool +5.8%; 3 bands 49 -> 59.5 cooled headless, app warm ~40 -> ~44 | L:90496ca7, L:44779c4f, L:a720b003 |
| `LITEV_SOFT3D_FAST` | Subaffine span interpolation +19% headless; active-edge table +17% (14.6 -> 17.1); approximate | L:902f101f |
| `LITEV_SOFT3D_HANDNEON` | Inline depth test instead of a per-pixel function pointer: +1.8% sustained | L:012f1355 |
| `LITEV_SOFT3D_INTERPNEON` | NEON span-interpolation ramp: +1.14% sustained headless (69.5 -> 70.3) | L:8826c5ca |
| COMPACTVTX / UNDERCOLO | Compact vertex/attribute layouts in SoftRenderer3D: +1.6 fps on the race at the time | L:5aaf9aef |

## GL renderer only

The shipping app uses the software renderer, so these were dropped with the GL render-thread work:

| Lever | Result | Source |
|---|---|---|
| `LITEV_RENDER_THREAD` | Capture/submit seam that moves GL submission off the emulator thread. GL at 3x: 32 -> 40 fps once actually compiled into the app. An earlier "34 -> 55 fps" headline was a lighter scene and was retracted (heavy race: ~31 fps on or off). In the software-renderer app the flag also gated app-side glue (the emulator-thread core pin, a headless no-render switch); that glue has to stand on its own in the app | A:5a4f4cb7, HANDOFF-2026-07-06 |
| `LITEV_GL_STATE_CACHE` | Shadow cache for redundant GL binds: binds -18%, texParameter calls -70%, fps 30.75 -> 30.90 (noise). Its CMake option had also been deleted by accident, so it had not been compiled in for a long time | liteDS-v2 plan D.7 (PLAN:1388-1395) |

## Measurement bisect switches

`debug.litev.*` properties added only to bisect a measurement (`dmabulk`, `gxdrain`, `no3d`,
`noedge`, `nofog`, `noattr`, `hybrid`, `defercapture`) and diagnostic code guarded by macros
that no build ever defined (`LITEV_JIT_DISPATCHLOG`, `LITEV_LF_BYTES`) are not part of this
history. `gxdrain` and `dmabulk` changed emulation when set.
