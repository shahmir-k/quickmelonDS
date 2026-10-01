# Shrek in-race gameplay benchmark workload

The `shrek-600.*` baselines measure Shrek's **boot/intro/menu** sequence (600
frames from direct boot, no input) — which is *not* gameplay. This `shrek-race`
workload drives the emulator into an **actual Quick Race** and measures live
3D gameplay (rasterization + GXFIFO + DMA every frame).

## Committed artifacts

| file | what |
|---|---|
| `shrek-race.script`         | input script: direct boot → menus → live Quick Race, then A held (accelerate) |
| `shrek-race-hold-a.script`  | input script: hold A from frame 0 (used with the savestate) |

## NOT committed (local only)

- Golden traces (`*.trace`): generated outputs that depend on the build configuration. Record one
  with the command below before a change and verify against it after.

- `savestates/shrek-race.mln` — a race-start savestate embeds copyrighted RAM
  contents, so it is **never** committed (`.gitignore`: `savestates/`, `*.mln`).
  Regenerate it locally with one command (see below).

## Benchmark workflow: script-once / savestate-many

Replaying the whole menu path (~1540 frames of boot+menus) on every benchmark
run is wasteful and dominated by non-gameplay work. Instead:

1. **Once**, bake a race-start savestate at frame F=1600 (kart on the start
   line, race live) using the script:

   ```
   liteDS-headless --rom tools/headless/testroms/shrek.nds \
     --frames 1601 --input-script tools/headless/baselines/shrek-race.script \
     --dump-savestate 1600:savestates/shrek-race.mln
   ```

2. **Many times**, load that savestate and measure a steady in-race window,
   holding A so the kart drives during the measured frames:

   ```
   liteDS-headless --rom tools/headless/testroms/shrek.nds \
     --savestate savestates/shrek-race.mln \
     --input-script tools/headless/baselines/shrek-race-hold-a.script \
     --frames 961 --bench-window 60:960 --frameskip 0
   ```

   `--bench-window s:e` reports avg FPS over frames [s,e] only (skipping a small
   settle window after the savestate load) while still running the full range.

The savestate shortcut is **bit-exact** with script replay: savestate-loaded
frame N reproduces script frame 1601+N to the frame-hash. So the shortcut does
not distort the measurement — it only skips the menu ramp.

## Regenerate + verify the golden trace

```
liteDS-headless --rom tools/headless/testroms/shrek.nds --frames 3400 \
  --input-script tools/headless/baselines/shrek-race.script \
  --record-trace tools/headless/baselines/shrek-race-3400.trace

# verify on any exact-timing config (baseline/dispatch/link):
liteDS-headless --rom tools/headless/testroms/shrek.nds \
  --input-script tools/headless/baselines/shrek-race.script \
  --verify-trace tools/headless/baselines/shrek-race-3400.trace
```

EVENT_SLICES / full configs deliberately re-time and must be verified against a
separate ES golden (as with `shrek-600-eventslices.trace`).

## PPM-verified race milestones (software renderer, FreeBIOS direct boot)

| frame | screen |
|------:|--------|
| ~900  | main menu, "Single Player" |
| ~960  | SELECT GAME TYPE → DOWN,DOWN → Quick Race |
| ~1080 | QUICK RACE track select ("Swamp") |
| ~1150 | SELECT CHARACTER ("Shrek") |
| ~1230 | SELECT PROFILE ("DEFAULT") |
| ~1320 | "Loading..." |
| ~1560 | **RACE LIVE**: HUD "1st"/"Lap 1/3", P1–P4, minimap, live 3D |
| 1600  | savestate dump point F (kart at start line) |

The entire menu path is **button-only** — no touchscreen input is used or
required to reach the race.
