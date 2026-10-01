# Testing Methodology — measuring liteDS performance on the RG DS without fooling ourselves

Status: written 2026-10-01 from the tooling, the code, the memory notes, the chat history **and an
empirical A/A study run the same day** (Appendix A). The procedures apply to this history: the lib
branch `litev-clean` with the app `melonDS-android-app-clean` branch `app-clean` (see
`BUILD-METHODOLOGY.md`, which also covers building and installing). The A/A numbers were measured on the
pre-history build (app `0b66608d` + lib `e9b8222f`). They are kept as the noise-floor estimate until the
study is repeated on the clean build. **[V]** marks
something verified by execution today. **[UNVERIFIED]** marks something carried over from the
history and not re-checked.

Goal (user): *"ensure that our measurement system is actually representative of the performance and
not falling into issues of measuring wrong, inaccurately, or having problems due to the randomness
of each race."*

---

## Contents

0. [Headline numbers (A/A noise floor)](#0-headline-numbers)
1. [The canonical metric](#1-the-canonical-metric)
2. [Measurement tiers and when to use each](#2-measurement-tiers)
3. [Sources of error and their controls](#3-sources-of-error-and-controls)
4. [Statistics: reps, noise band, MDE, reporting](#4-statistics)
5. [Standard procedures (copy-paste)](#5-standard-procedures)
6. [Tooling audit: what is solid, what is broken, fixes](#6-tooling-audit)
7. [Appendix A: A/A study raw data](#appendix-a)
8. [Appendix B: sources](#appendix-b)

---

## 0. Headline numbers

All numbers come from one build (history note: the pre-history app `0b66608d` + lib `e9b8222f`, clean `git archive`), measured
repeatedly on the RG DS on 2026-10-01. Governor `performance` at 1.992 GHz throughout; no
`debug.litev.*` props set except `prof=1`. Before every run the device was cooled (CPU < 45 °C for
the app, SoC < 50 °C for headless).

| Measurement | n | Run-to-run spread | MDE at 80 % power, α = 0.05 | Recommended reps/arm |
|---|---|---|---|---|
| **App, Shrek slot-2, emulated fps (median of ~58 one-second windows over a 60 s leg, cold start)** | 6 legs | mean 58.84, **sd 0.43 fps (CV 0.73 %)**, range 58.45–59.65 | n=3: **0.98 fps (1.7 %)**; n=6: **0.69 fps (1.2 %)**; n=10: 0.54 fps | **6** (≥ 0.7 fps effects); 10 for 0.5 fps |
| App, mean fps per leg | 6 | sd 0.34 (CV 0.58 %) | n=6: 0.55 fps (0.93 %) | |
| App, runFrame median (ms) | 6 | 14.74 ms, sd 0.23 (CV 1.6 %) | n=6: 0.38 ms (2.6 %) | runFrame is noisier than fps in relative terms |
| **Headless `liteDS-app` instructions** (`race-fresh.mln`, 500 f, `taskset 08`) | 8 | **CV 0.096 %**, range 0.26 % | single pair: 0.38 %; n=3: **0.22 %**; n=5: 0.17 % | **5** for ≥ 0.2 %; there is no "1 rep suffices" |
| Headless cpu_cycles (same) | 8 | CV 0.28 %, range 0.89 % | n=3: 0.65 %; n=5: 0.50 % | 5 |
| **Headless avg_fps** (same) | 8 | **CV 1.47 %**, range 4.6 % | n=3: 3.4 %; n=5: 2.6 %; n=8: 2.1 % | 8+, and still only resolves > 2 % |
| Headless `run-race.sh` scene (`shrek-race.mln` + hold-A, unpinned, no fixed RTC), window_fps | 8 | CV 6.0 %, one outlier at 82.9 vs ~99 | > 8 % | do not use for A/B |

**What the noise floor means:**
- On the real app, **a lever under ~0.7 fps (≈ 1.2 %) cannot be resolved with 6 cold-start legs per
  arm.** Such levers must be judged with deterministic instruction or PMU counts (Tier 2).
- Headless fps is about 2× noisier than app fps per run, and it measures a different scene.
  **Never A/B on headless fps.**
- Headless instruction count is the sharpest instrument (~0.2 % at n=3). It measures *work*, not
  *time*.
- In an A/A test, the tools' current verdict rules call a "WIN/REGRESS" **14–79 % of the time**
  (§6.2).

---

## 1. The canonical metric

**Canonical app performance metric = emulated frames per second of the emulation thread, frameskip 0,
on the heavy segment of the Shrek slot-2 race, from a cold start, at a fixed time-since-load
window, reported with the cap-saturation fraction.**

What the numbers actually are (read from code, **[V]**):
- `LITEV_PROF … wall/frame=X ms (Y fps)` (`MelonInstance.cpp:749`) is 60 000 / (wall-clock span of
  the last **60 emulated frames** on the emu thread). It includes the frame limiter's sleep. Next to
  it, `cpu_loop` is the emu thread's per-frame *work*, and `runFrame` is the `NDS::RunFrame` portion
  of that work.
- The on-screen `FPS: NN` overlay (`MelonDSAndroidJNI.cpp:746`) counts the same emulated frames,
  averaged over 30 frames. Neither number counts presented or vsync'd frames.
- Frameskip only skips *rasterisation*; emulated frames are still counted, so a skipping run reports
  the fps of a cheaper workload. Three things can set the frameskip target: the **Auto frameskip**
  setting (`enable_auto_frameskip`, `LITEV_AUTO_FRAMESKIP`), **fast-forward** (it drives frameskip in
  the pre-history app line), and the `debug.litev.frameskip` dev prop (`LITEV_AGGRESSIVE_SKIP`).
  All three must be off for every measurement: see the hard rule in §3.0.
- **Cap:** the limiter (`limitFps`, target 60 × 263-line frames = 16.67 ms) caps the metric at ~60.
  In the A/A legs, **25–59 % of one-second windows were at ≥ 59.5 fps**. Those samples are
  *censored*: a lever that makes such a frame cheaper cannot show up in them.
- **Cap-saturation rule:** report `capped%` (the share of windows ≥ 59.5) with every result. If
  capped% exceeds about 20 %, fps under-reports wins. Then either:
  - (a) measure **uncapped: the limiter off (`limitFps=false`) with frameskip 0**, asserted per §3.0.
    There is no supported switch for this yet; a `debug.litev.nolimit` prop would provide it
    (§6.3). **Fast-forward is not an uncapped mode.** It is a speed-up feature that the §3.0 rule
    excludes (in the pre-history app it also raises the frameskip target). The fast-forward numbers
    in Appendix A.3 are a feasibility note only and must not be reused as a method.
  - (b) or report **frame time and cap share** instead of fps: `cpu_loop`/`runFrame` (or emu-thread
    busy %) per arm, plus capped%. Caveat: runFrame was **higher** uncapped (14–15.7 ms) than capped
    (12.5–13.5 ms) in the same scene, so capped runFrame *understates* the uncapped cost; compare
    it only between arms measured the same way.

Historical context: sustained race numbers of ~55–58 fps, with momentary 60s, are consistent with
this metric. Today's A/A legs gave per-leg medians of 58.45–59.65.

---

## 2. Measurement tiers

| Tier | Instrument | Measures | Deterministic? | Resolution (measured) | Use for |
|---|---|---|---|---|---|
| **T0 Correctness gates** | headless `--record-trace`/`tracediff.py` (guest state), `--fb-dump-ppm` + `ppmdiff.py` (settled framebuffer), JIT-vs-interp A/B, 2nd game (Pokémon White `.ml1`), MP `--mp-test` | is the output still right; category A/B | trace: yes; PPM: **mostly** (§3.8) | exact | **Every** lever, before any timing |
| **T1 Headless deterministic work** | `quickprof.sh`/`abtest.sh` → `simpleperf stat` instructions/cycles over `race-fresh.mln` | host work done by the emu core + workers | instructions ≈ yes (CV 0.1 %) | ~0.2 % (n=3) | Emu-core levers (JIT, scheduler, memory) that do not move work between threads |
| **T2 App PMU self-time** | `capture.sh -e <event>` on the APP EmulatorThread (or `--thread s3d-tile0/s2d-async`) | where the misses and stalls are, per function, including the JIT blob (perf-map) | no (live scene) | function-local; interleave same session | Sub-fps / function-local levers; diagnosis (sample-on-miss); completeness gate ≥ 92 % attributed |
| **T3 App whole-frame fps** | cold-start legs of the `.dev` app, slot 2, `LITEV_PROF` | the shipping user-visible throughput | no | **~0.7 fps / 1.2 % at n=6** | Final verdict for whole-frame levers (> ~1 fps) and anything that moves work between threads |

### Decision table

| Lever / question | Gate first | Then measure with | Why not the others |
|---|---|---|---|
| JIT/emitter, scheduler, memory path (stays on the emu thread, byte-exact) | T0 trace + PPM | T1 instructions (n ≥ 5, interleaved) **and** T2 self-time on the target fns | app fps cannot resolve < 0.7 fps |
| Moves work between threads (render prep, sort, offload, affinity) | T0 | **T3 whole-frame** (plus T2 on every involved thread) | single-thread counters lie: NOSORTOPAQUE showed −6.6 % emu L2 but was flat or negative on the frame |
| Renderer/raster (tile renderer, 2D compositor) | T0 PPM (+ control run) | T3; T2 `--thread s3d-tile*` / `s2d-async` | headless renders with the same tile renderer but without GL/present/Android contention |
| Timing-relaxation (category B) | T0 trace (expect a guest diff) + MP test + 2nd game | T3 | — |
| "Is it a bug in the renderer or the JIT?" | T0 JIT-vs-interp on the exact repro + `git diff` for uncommitted probes | — | see `ANALYSIS-METHODOLOGY.md` |
| Expected effect < 0.5 % instructions and you need fps | — | Don't. Report it as a PMU brick (T1/T2) | below every fps noise floor |

**Headless is not the app scene, and headless fps is not app fps** **[V]**:

| Same build | fps |
|---|---|
| App, slot-2 race, capped (A/A mean) | 58.6 |
| App, fast-forward, late race, 71 °C (feasibility note only; not a valid measurement, §3.0) | ~62–63 |
| Headless `race-fresh.mln` (abtest/quickprof scene), emu pinned | 74.2 |
| Headless `shrek-race.mln` + hold-A (run-race scene) | 96.7 (window) |

These are different scenes. `race-fresh.mln` is the race start (timer 00:02 at frame 300, kart
stationary). `shrek-race.mln` + hold-A has the kart driving (3rd, 00:06). App slot 2 opens at about
00:28 into a race whose minimap layout looks different. The `abtest.sh` header comment ("Same race
the app loads as save slot 2") is **false**. Headless is also missing GL composite/present, ADPF and
system contention, and it uses a different NDK. **Use headless for deterministic deltas, never for
absolute or app-comparable fps.**

---

## 3. Sources of error and controls

Each entry gives the mechanism, the evidence, and the **control** to apply.

### 3.0 HARD RULE: frameskip OFF for every performance measurement
Any frameskip drops rendered frames to hold the pace, so it **hides the per-frame cost** that an
optimisation changes. A lever that takes 2 ms off a 31 ms frame shows up as an fps delta only if
every frame is rendered; with frameskip holding the target, before and after both read the target
and the win (or the regression) is invisible. Three sources, all of which must be off:

| Source | What it does | Off means |
|---|---|---|
| **Auto frameskip** setting (`enable_auto_frameskip`, compiled in by `LITEV_AUTO_FRAMESKIP`) | adaptive controller raises the skip level (up to 3) whenever a frame runs over budget | setting off (the default) |
| **Fast-forward** (▶▶ overlay button) | lifts the limiter; in the pre-history app it also raises the frameskip target | never pressed during a leg |
| `debug.litev.frameskip` dev prop (`LITEV_AGGRESSIVE_SKIP`) | fixed skip target | unset or 0 |

Frameskip is a user-experience feature, not a measurement mode. (Headless `--frameskip N` is a
separate, deliberate tool for isolating emulation from raster cost; it is never the normal gain
measurement, and every headless A/B passes `--frameskip 0`.)

**Pre-measurement assertions (every session, and recorded with the result):**
```sh
adb shell getprop debug.litev.frameskip                       # must print nothing or 0
adb shell "su 0 cat /data/data/me.magnum.melonds.dev/shared_prefs/me.magnum.melonds.dev_preferences.xml" \
  | grep enable_auto_frameskip                                 # must be absent or value="false"
adb logcat -c                                                  # before launching the leg
# ... run the leg (no fast-forward) ...
adb logcat -d -s LITEV_AUTOFS | grep -c LITEV_AUTOFS           # after the leg: must print 0
```
`LITEV_AUTOFS` logs every adaptive level change, so zero lines prove the controller never
skipped during the leg (`app-run.sh sample` clears the log when it starts, so run the check right
after `sample`; it then covers exactly the measured window). If any assertion fails, the leg is invalid; fix the state, force-stop,
relaunch and repeat it. An fps that will not move under a real optimisation is a reason to re-check
these assertions before calling the lever null.

### 3.1 Scene variance: the race is not the same race twice
- **No fixed RTC in the app.** Each slot-2 load replays the race slightly differently. Headless runs
  pass `--fixed-rtc 1600000000` and are deterministic (8/8 identical `audio_hash`).
- **The scene drifts as the race plays out.** With no input, the player kart idles while the AI
  karts drive. In the 5-minute sustained leg, fps rose **58.5 → 59.6** and runFrame fell **14.3 →
  13.3 ms** minute by minute, even as the device heated to 70 °C (Appendix A.3). A 60 s window
  starting at a different time-since-load is a different workload. The historical "54 fps" had to
  be retracted as "a lighter race moment" (P60, 2026-07-18). So had the "34 → 55 fps" R4 claim
  (lighter scene; heavy race ~31 either way).
- **One leg in six was an outlier** (leg 5: 59.65 median, 59 % capped vs ~25 % for the others).
  That is consistent with a lighter replay.
- **Controls:**
  - Reload slot 2 fresh for **every** window (never continue a window from a previous
    measurement).
  - Start sampling at a fixed offset (immediately after load, i.e. the first 60 s) and use the
    same duration for both arms.
  - Use ≥ 6 legs per arm and medians.
  - Record the race timer (display 0) or the time since load.
  - Headless: always `--fixed-rtc`, `--savestate`, `--frameskip 0`, and a fresh copy of the data
    dir (the game rewrites `headless.sav`, BUILD P7).

### 3.2 Thermal and DVFS: cool peak vs sustained
- **Evidence:**
  - From a 44 °C cold start, the CPU reached 56 °C by the time the race was loaded, 63 °C after
    60 s, 66 °C at 2 min, and **70 °C with `mStatus=3` (hot-throttle status) at ~4 min**. Clock
    stayed 1.992 GHz with `cooling_device0=0` during these 5 min **[V]**.
  - `scaling_cur_freq` is **not** a comparability check: 42–53 fps "regressions" at 70–73 °C
    showed 1.992 GHz (IL 2026-09-16).
  - Past the stock 85 °C trip the A55s are capped at 1.61 GHz (−19 %) **[UNVERIFIED today]**.
  - Cooling 63 → 44 °C took 182–204 s.
- **Controls:**
  - `app-race.sh cool 45` (or `app-run.sh cool 45`) before every leg. Reject any leg whose CPU
    (`dumpsys thermalservice`, the last `Temperature{…mType=0…}` line, not the cached one or the
    threshold line) is ≥ 65 °C or has `mStatus` > 0 at start **or** end. This limits a cold-start
    leg to roughly the first 2 minutes after load.
  - Report start/end °C for every leg.
  - **Cool-peak numbers are not the shipping state.** The user's target is sustained performance.
    For a sustained claim, run a separate ≥ 5-minute leg per arm, interleaved, and report it
    *separately* from cold-start legs. Never mix the two.
  - `race-tune.sh` (raises trip_point_1 to 100 °C, moves IRQs and system threads off core 3) is
    bench-only and boot-volatile. Results under it are not shipping numbers. If you use it, apply it
    to both arms and label the result.
  - The governor was found at `performance` on all 4 cores. Verify it per session:
    `cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor`.

### 3.3 Run-order bias
- Device temperature, background activity and absolute PMU counts (`l2d_refill` 42 M vs 76 M for
  the same run across sessions) drift within and across sessions.
- **Control:** interleave arms ABAB…, or better ABBA blocks, within one session, with the same
  cooldown before each leg. Compare only within a session.
- `run.sh`/`run-race.sh`/`abtest.sh` interleave. **`app-race.sh` has no A/B mode at all**, so
  app legs must be interleaved by hand or by a wrapper (§5.3).

### 3.4 Warm-up, JIT compile and first window
- The first `LITEV_PROF` window straddles the load transient (e.g. leg 1 window 1: 54.9 fps vs a
  58.5 median).
- **Controls:**
  - Drop window 1 (the analysis script does).
  - Headless: use `--bench-window` or a constant frame count. Savestate load and JIT warm-up are
    identical in both arms, so for whole-run instructions they cancel.
  - Never compare a 300-frame and a 500-frame run.

### 3.5 Background contention (core 3, SurfaceFlinger, system_server, co-running emulator)
- The app pins the emu thread to core 3 and the workers to cores 0–2. In headless, `taskset 08`
  pins the emu thread to core 3, and the tile workers self-pin to 0, 1, 2 (`s2d-async` floats over
  0–2) **[V via /proc]**. Pinning is worth **~3 %** headless fps (74.2 pinned vs 72.0 unpinned).
- A **running app** steals the cluster from a headless run. A 39 → 14 fps "drop" was exactly this.
- **Controls:**
  - Always `am force-stop me.magnum.melonds.dev` before headless (the tools do this).
  - Never run two benchmarks at once. One headless S2 outlier (82.9 vs ~99 fps with only +2.7 %
    cycles) shows off-CPU waiting from transient contention. Medians absorb such outliers; means
    don't.
  - Check `top -H` for foreign load before a session.

### 3.6 Wrong package, build, flags or props
- Measure `me.magnum.melonds.dev` only. Fingerprint the installed APK by md5 (BUILD §6). Verify the
  flags in the CMake cache (BUILD §4.2).
- (Fixed in `litev-clean`; history note.) **Headless `liteDS-app` had been silently built with `LITEV_GEOM_OFFLOAD=ON`** (a commented-out
  gradle line, BUILD §10.1). The as-is binary does +1.8 % instructions with different hashes, and
  runs +2.9 % *faster* in fps, because offload parallelises. Every `liteDS-app` result since
  2026-09-16 has this confound.
- Props are boot-volatile, and some are latched in a `static` at first read (DMA props, etc.).
  Toggling a prop mid-run does nothing, or only half-engages (`debug.litev.software` mid-game).
  **Controls:**
  - Set props before launch, then force-stop and relaunch per config.
  - Snapshot `getprop | grep litev` into the run record.
  - Restore defaults afterwards: `rendercpu=2`, `s2dcpu=-1`, `softprof=0`, `frameskip` unset, and
    the Auto frameskip setting off (§3.0).
- Diagnostic props perturb the hot loop: `debug.litev.softprof=1` costs ~5.9 M atomics per 60
  frames. Keep it 0 for timing.

### 3.7 Frame pacing and the 60 fps cap
See §1. Report capped%; when it exceeds 20 %, measure uncapped (limiter off, frameskip 0) or
report frame time and cap share. Never use fast-forward or any frameskip to get above the cap
(§3.0), and never record a steady 60 as a measurement.

### 3.8 Async capture jitter in correctness gates
- Under TILE_COORD the framebuffer *hash* is async: `final_top` differs run to run (host 3/3 JIT
  runs gave 2 values; headless S2 on the device gave 2 values over 8 runs). Guest state (`audio_hash`,
  trace fields 0–176) stays deterministic.
- Settled PPM dumps are **usually** deterministic: on the device 4/4 identical at frames 100/250/400
  **[V]**. They are **not always**: on the host, frames 100 and 299 had two variants each over 6 runs,
  8.8 % px apart (an off-by-one-frame capture).
- **Controls:**
  - Gate guest state with `tracediff.py` (record mode only, not `--verify-trace`, which
    early-exits on the async fb mismatch).
  - Gate visuals with PPM **plus a same-binary control** (`abtest.sh --control` / `--control-on`)
    showing 0 px. Re-run any 1-frame PPM diff before believing it.
  - Do not use record/verify runs for performance (trace and hashing overhead).

### 3.9 Headless sync-render dilution and single-thread vs whole-frame
- The headless `app` config renders with the same tile workers, but the absolute scene and frame
  composition differ from the app (§2), and `full` uses the reference renderer (3–10× off).
- An emu-thread win can be a worker-thread loss (NOSORTOPAQUE).
- **Control:** judge thread-shifting levers only on T3. With T2, profile every thread that does
  frame work (`capture.sh --thread s3d-tile0` …) and sum.

### 3.10 Observer effect
- `simpleperf record -g` costs ~−33 %; flat `record` ~−10 %; `stat` ~0 (memory
  `tool-selection…`) **[UNVERIFIED today]**.
- **Control:** never report fps from a profiled run. Compare PMU only against PMU taken the same way.

### 3.11 Correctness: multi-game and JIT-vs-interp
- Shrek byte-matched while Pokémon White's JIT path was broken. **Gate renderer/JIT changes on a
  second game** (Pokémon White `.ml1` is on the device SD card).
- With the shipping flags, JIT ≠ interp is *expected*: the category-B timing levers (EVENT_SLICES
  etc.) do not apply to the interpreter. Host: audio differs and the framebuffer is 8.8 % apart
  **[V]**. Use JIT-vs-interp to *localise* ("does interp render it right?") or under an exact-timing
  config, not as a byte gate for the shipping set.
- A green guest-state trace does not prove a visual change correct: a JIT bug can corrupt only GPU
  input (geometry) while CPU and RAM state re-converge. For anything visual, gate on pixels too.
  How to diagnose a failing gate is in `ANALYSIS-METHODOLOGY.md`.

### 3.12 UI automation failures that masquerade as performance
- Wrong slot (`SLOT_XY 213 266` = empty slot 3) leaves the emulator idling on the title screen at
  60 fps.
- Pause dialog left open, screen sleep, the user touching the device.
- **Scene verification (mandatory per leg):**
  1. `ResumedActivity … EmulatorActivity`.
  2. EmulatorThread busy ≥ 50 % (66–76 % measured right after load).
  3. `su 0 screencap -p -d 1` > 20 KB and showing the race view. The game is on **Display 1**; a
     plain `screencap` captures Display 0 and makes a running game look blank (BUILD §9).
  4. 3 × `s3d-tile` threads.
  5. Median fps < 60 and capped% reported.
- The user may be holding the device. Keep automated UI sequences short and do not loop
  unattended while they are using it.

---

## 4. Statistics

### 4.1 Unit of replication
- **App:** one *cold-start leg* (cool → launch → slot 2 → 60 s of `LITEV_PROF` windows, window 1
  dropped). Windows within a leg are autocorrelated and share the scene draw, so **n = legs, not
  windows**. Summarise each leg by its **median** window fps, then compare the legs.
- **Headless:** one process run (fresh data dir, interleaved).

### 4.2 Median vs mean
- Use the **median** per leg (robust to the load transient and stalls). Across legs, report the
  median and the mean ± sd. The headless S2 outlier (82.9 among ~99) moves the mean by 2.4 fps but
  the median by 0.3.

### 4.3 Noise band and minimum detectable effect
- MDE ≈ 2.8 × sd × √(2/n) for a two-arm comparison (80 % power, α = 0.05).
- Using the measured sds from §0:
  - **App fps:** n=6 → 0.69 fps.
  - **Headless instructions:** n=3 → 0.22 %; n=5 → 0.17 %.
  - **Headless fps:** n=5 → 2.6 %.
- A result smaller than the MDE is "not resolved". It is **not** "null" and **not** "ceiling".
- Do not use "range of the OFF arm" as the noise band (what `abperf.py`/`quickprof.sh` do). The
  range of 1–3 samples is a poor estimate, and it ignores the ON arm.
- Use a Welch t-test or Mann–Whitney on the per-leg (or per-run) values, plus a **bootstrap 95 %
  CI of the median difference**. Call WIN/REGRESS only if the CI excludes 0 **and** |Δ| ≥ MDE.

### 4.4 Recommended reps

| Instrument | Reps per arm | Interleave | Cooldown |
|---|---|---|---|
| T3 app fps | **6** (10 for 0.5 fps) | ABBA… legs | CPU < 45 °C, gate < 65 °C / mStatus 0 at start and end |
| T1 headless instructions | **5** | ABAB… | SoC < 50 °C (cheap) |
| T1 headless cycles | 5 | ABAB | as above |
| T2 app PMU self-time | ≥ 3 per arm, same session | ABAB, fresh slot-2 load per capture | as T3 |

### 4.5 How to report a result (minimum)
Include:
- build pair (app rev, lib rev, APK md5 / binary md5);
- the flag diff;
- props;
- scene (savestate file and md5, or app slot 2 plus time window);
- metric definition;
- n per arm;
- median, mean ± sd, CI of Δ, MDE;
- capped%;
- start/end °C per leg;
- clock;
- interleave order;
- verdict;
- the T0 gate result (category A/B, plus the control run);
- the §3.0 frameskip assertions (prop, setting, empty `LITEV_AUTOFS` log, no fast-forward).

### 4.6 Ledger (`results/measurements.jsonl`)
Today's records carry `ts, tool, scene, binary, event/prop, verdict`, attribution `top`, and for
A/Bs `fps_off/on, delta%, noise%, reps`.

Gaps:
- `scene` defaults to `shrek-race-slot2` even for headless `race-fresh.mln` runs (mislabelled).
- No build fingerprint, n/sd/CI, temperatures, capped%, or prop snapshot.

Proposed additional fields:
`app_rev, lib_rev, bin_md5, flags_sha, scene_file, scene_md5, n_off, n_on, median_off/on, sd_off/on, ci95_delta, mde, capped_pct, temp_start/end_c, clock_khz, props, gate, control_gate`.

The A/A raw data of this study was deliberately **not** written to the ledger; Appendix A summarises it.

---

## 5. Standard procedures

`$SP` = `tools/measure/` in the lib repo (`app-run.sh`, `ui.py`, `aa-app-analyze.py`, `aa-*.sh`).
Always `export ANDROID_SERIAL=192.168.0.165:44675`.

### 5.1 Session preflight (every session)
```sh
adb shell getprop ro.product.model                                   # RG DS
adb shell 'getprop | grep litev'                                     # expect only prof (and nothing stale)
adb shell getprop debug.litev.frameskip                              # empty or 0 (§3.0)
adb shell "su 0 cat /data/data/me.magnum.melonds.dev/shared_prefs/me.magnum.melonds.dev_preferences.xml" | grep enable_auto_frameskip   # absent or false
adb shell 'cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; cat /sys/class/thermal/thermal_zone0/trip_point_1_temp'
adb shell dumpsys package me.magnum.melonds.dev | grep lastUpdateTime   # + APK md5 check (BUILD §6)
adb shell 'top -H -b -n1 | head -15'                                 # no foreign load
```

### 5.2 One app leg (cold start, verified, 60 s) **[V]**
```sh
bash $SP/app-run.sh cool 45            # force-stop + wait CPU<45 °C  (~3 min after a leg)
bash $SP/app-run.sh start              # launch, Shrek, slot 2 by text, verify busy%/display-1/s3d-tile
bash $SP/app-run.sh sample 60          # one line per 60-frame window: fps runFrame wall cpuC_start/end
```

### 5.3 App A/B (whole-frame lever)
1. Build both APKs (BUILD §5) from clean trees. Record their md5s.
2. For leg in A B B A A B B A A B B A: `adb install -r <arm>.apk` → §5.2 → append the output with
   an arm label. Optionally run a runtime-prop arm with prop set → force-stop → relaunch.
3. Analyse per leg (median window fps, drop window 1, capped%), then compare arms with a bootstrap
   CI. Reject legs that fail the thermal gate (≥ 65 °C / mStatus > 0).
4. If capped% > 20 % in either arm, repeat uncapped (limiter off, frameskip 0, §1) or report frame
   time and cap share. Never use fast-forward for this (§3.0).

### 5.4 Headless deterministic A/B (T1) **[V for the A/A form]**
```sh
# isolated dir + fresh data dir per run; interleaved; n=5
for r in 1 2 3 4 5; do for arm in A B; do
  adb shell "su 0 sh -c 'cd /data/local/tmp/liteds-aa && rm -rf dd && cp -r dd0 dd && \
    taskset 08 simpleperf stat -e cpu-cycles,instructions ./liteDS-$arm --rom shrek.nds \
    --savestate race-fresh.mln --frames 500 --fixed-rtc 1600000000 --mode jit --fastmem on \
    --frameskip 0 --data-dir dd'" | grep -E 'avg_fps|instructions|cpu-cycles|audio_hash'
done; done
```
Report instructions Δ with a CI. Δ < 0.2 % is "unresolved". `abtest.sh`/`quickprof.sh` do the
same, but with the verdict defects listed in §6.2.

### 5.5 Correctness gate (T0)
`abtest.sh --bin B --prop P --gate-only`, **plus `--control`** (OFF vs OFF must be 0 guest diffs
and 0 px). For JIT/renderer changes, also run a second game and the JIT-vs-interp localisation on
the host (BUILD §11).

---

## 6. Tooling audit

### 6.1 Solid (keep)
- **Deterministic headless scene with fixed RTC:** 8/8 identical guest hashes; instruction CV
  0.1 %. This is the best instrument we have.
- **`capture.sh` + `attribute.py`:** perf-map JIT resolution and a completeness gate (FAIL above
  8 % unattributed). This is what ended the "45 % invisible" era.
- **`abtest.sh` gate structure:** trace guest diff, settled PPM, category A/B, `--control` /
  `--control-on`, persistent run artifacts.
- **`app-race.sh` thermal gate:** live HAL CPU reading, < 65 °C and mStatus 0 at start and end;
  `cool` subcommand; `softprof` forced 0; no upper fps "sanity" cap.
- **`run.sh`/`run-race.sh`:** interleaved, cooldown, temp/clock logged per run.

### 6.2 Broken or misleading (with evidence)

| # | Defect | Evidence (A/A, 2026-10-01) | Fix |
|---|---|---|---|
| D1 | (fixed in `litev-clean`) `build.sh app` included **commented-out** gradle flags, so the headless "shipping" binary has `GEOM_OFFLOAD=ON` | hashes differ; +1.8 % instr; +2.9 % fps vs the fixed build | strip `//` lines before the grep (BUILD §10.1); assert the CMakeCache ON-set equals the APK's |
| D2 | `quickprof.sh` default `--reps 1`: spread = 0, so the threshold is 0.05 %, below the 0.1 % instruction sd | resampling the A/A: **false WIN/REGRESS in 79 % (pinned) / 61 % of single-pair runs**; reps 3 → 10–21 % | default reps ≥ 5; threshold from a pooled sd/CI (§4.3), never "0.05 % floor" |
| D3 | `abperf.py` noise = range of the OFF arm only, n = 3, threshold max(range, 0.3 %) | **false fps verdict in 14–16 %** of A/A splits | Welch/bootstrap CI on both arms; MDE gate; n ≥ 5 |
| D4 | Headless scene ≠ app scene; the `abtest.sh` comment says otherwise; ledger `scene` mislabelled | race-fresh is the race start (00:02, kart idle); app slot 2 starts at 00:28 with a different-looking minimap; fps 74 vs 58.6 | label scenes by file + md5; bake a headless savestate *from the app's slot 2* (`.ml2` is a melonDS savestate; pull it with `su 0 cat`) **[UNVERIFIED that headless loads `.ml2` directly]** |
| D5 | Two headless race savestates in use (`race-fresh.mln` vs `shrek-race.mln`+hold-A); `run-race.sh` has no `--fixed-rtc` or pinning | S2 CV 6 %, final_top unstable | one canonical scene; always `--fixed-rtc` and `taskset 08` |
| D6 | `app-race.sh` hard-coded coordinates are wrong (`SLOT_XY 213 266` = empty slot 3; `ROM_ROW_XY 250 168` = Pokémon W2); stale `ROM_DIR`; `preflight` picks the first adb device | verified UI dump | resolve by text (`ui.py`), honour `ANDROID_SERIAL` |
| D7 | `app-race.sh fps` samples whatever is running: no time-since-load anchor, no window-1 drop, no capped%, no per-leg verdict, no A/B or repeat mode | sustained run: fps drifts +1.1 over 5 min from scene alone | add `ab` mode (ABBA legs, cold start each), anchor at load, emit capped% |
| D8 | Headless data dir is mutable (the game rewrites `headless.sav`); the tools run in the user's `hdata/` | three different `headless.sav` md5s on the device | copy a pristine data dir per run |
| D9 | `abtest`/`quickprof` do no cooldown between runs and never log temperature | — | log SoC °C/clock per run; cool to a threshold |
| D10 | Ledger lacks a build fingerprint, n/sd/CI, temps, props, capped% | `results/measurements.jsonl` | schema in §4.6 |
| D11 | No uncapped app mode | 25–59 % of windows capped | `debug.litev.nolimit` prop (limiter off, frameskip untouched at 0). Not fast-forward (§3.0) |
| D12 | `capture.sh` records 12 s of whatever scene is live (no slot-2 reload, no thermal check) | scene drift §3.1 | reload slot 2 per capture; log time since load and °C |
| D13 | Headless uses NDK 27, the APK NDK 28 | — | `ANDROID_NDK=…/28.0.13004108` in `build.sh` |

### 6.3 Missing (highest value first)
1. **An app A/B runner** (`app-race.sh ab A.apk B.apk --legs 12`): ABBA cold-start legs, the
   §3.12 scene verification, the thermal gate, per-leg medians, bootstrap CI, MDE, capped%, and a
   ledger write with a full fingerprint.
2. **An uncapped mode** (`debug.litev.nolimit=1` → `limitFps=false` at launch, frameskip left at
   0), so whole-frame levers are not censored at 60.
3. **A shared stats module** (`tools/profiler/stats.py`) used by abperf/quickprof/app runner:
   pooled sd, Welch, bootstrap, MDE, and an "unresolved" verdict.
4. **A canonical scene manifest** (`scenes.json`): savestate md5, frames, RTC, input script and
   expected guest hash. The tools refuse to run on a mismatch (this catches a wrong or mutated
   savestate).
5. **A sustained-mode protocol**: a 5–10 minute interleaved leg per arm, reported separately from
   cold-start numbers.

---

<a id="appendix-a"></a>
## Appendix A: A/A study raw data (2026-10-01)

Scripts in `tools/measure/` of the lib repo: `aa-headless.sh` writes `aa-headless.tsv`; `aa-app.sh` +
`app-run.sh` write `aa-app.txt` (every window); `aa-app-analyze.py` turns that into the per-leg summary.
The raw logs of the 2026-10-01 study are not in the repo; the tables below summarise them. Device dir used: `/data/local/tmp/liteds-aa/` (isolated copies; the
user's `/data/local/tmp/liteds` was not modified).

### A.1 Headless (`liteDS-app-fixed` unless noted; cooldown to SoC < 50 °C before each run; clock 1 992 000 kHz on every run)

Variants:
- `S1p-asis` = the as-is `build.sh app` binary (GEOM_OFFLOAD=ON).
- `S1p` = race-fresh, 500 f, fixed RTC, `taskset 08` (= the abtest/quickprof recipe).
- `S1u` = S1p without taskset.
- `S2` = shrek-race.mln + hold-A, 961 f, window 60:960, no RTC/taskset (= the run-race recipe).

| rep | variant | avg_fps | window_fps | instructions | cpu_cycles | final_top | SoC °C pre→post |
|---|---|---|---|---|---|---|---|
| 1 | S1p-asis | 76.37 | | 36067853191 | 41518317694 | b1f77c42… | 50.6→53.1 |
| 1 | S1p | 75.20 | | 35387261127 | 40327121864 | 97b2d128… | 48.9→52.5 |
| 1 | S1u | 70.99 | | 35455444089 | 40375901888 | 97b2d128… | 48.9→51.9 |
| 1 | S2 | 93.35 | 95.51 | 50530136080 | 58282321888 | 5f993f57… | 48.9→53.1 |
| 2 | S1p-asis | 76.60 | | 36017169947 | 41385028252 | b1f77c42… | 49.4→53.8 |
| 2 | S1p | 73.46 | | 35398442245 | 40408581970 | 97b2d128… | 50.0→52.5 |
| 2 | S1u | 73.00 | | 35445469457 | 40413703037 | 97b2d128… | 48.9→53.8 |
| 2 | S2 | 97.68 | 100.30 | 50568518057 | 57615497073 | 5f993f57… | 49.4→53.1 |
| 3 | S1p-asis | 76.75 | | 36041048813 | 41469724226 | b1f77c42… | 49.4→52.5 |
| 3 | S1p | 74.05 | | 35461771650 | 40517053452 | 97b2d128… | 49.4→53.1 |
| 3 | S1u | 74.02 | | 35444219055 | 40360918623 | 97b2d128… | 50.0→53.8 |
| 3 | S2 | 96.94 | 99.83 | 50411915826 | 57554574071 | 5f993f57… | 48.9→53.1 |
| 4–8 | (see `logs/aa-headless.tsv`) | | | | | | |

Per-variant summary (n=8):

| variant | avg_fps mean ± sd (CV) | range | instructions CV / range | cycles CV / range | guest hash (audio) |
|---|---|---|---|---|---|
| S1p-asis | 76.35 ± 0.34 (0.44 %) | 75.73–76.75 | 0.060 % / 0.20 % | 0.155 % / 0.51 % | dd9af824 ×8 |
| S1p | 74.22 ± 1.09 (1.47 %) | 72.20–75.65 | 0.096 % / 0.26 % | 0.283 % / 0.89 % | 5072399c ×8 |
| S1u | 72.01 ± 1.14 (1.59 %) | 70.55–74.02 | 0.063 % / 0.20 % | 0.110 % / 0.36 % | 5072399c ×8 |
| S2 | 94.35 ± 5.23 (5.5 %); window 96.71 ± 5.83 | 81.99–97.68 | 0.244 % / 0.68 % | 0.973 % / 2.9 % | 9e8362d3 ×8; final_top 2 values |

- S1p fps by rep: 75.20, 73.46, 74.05, 72.20, 74.75, 74.67, 73.78, 75.65.
- S2 window_fps: 95.51, 100.30, 99.83, 100.16, 100.04, 97.76, **82.95**, 97.11.
- Instructions as-is vs fixed: 36.05 G vs 35.42 G = **+1.78 %** (GEOM_OFFLOAD).

Tool-rule false-verdict rates on this A/A data (all ordered pairs/splits of the 8 runs):
- quickprof reps=1 (threshold 0.05 %): **79 % (S1p) / 61 % (S1u)**.
- quickprof reps=3: 21 % / 10 %.
- abtest fps reps=3 (threshold max(OFF range, 0.3 %)): **14 % / 16 %**.

### A.2 App (6 cold-start legs, 60 s each, window 1 dropped, CPU cooled to < 44.4 °C before each; all legs verified: EmulatorThread 66–76 % busy, display-1 PNG ~80 KB, 3 × s3d-tile)

| leg | windows | fps median | fps mean | fps p10 | runFrame median ms | wall median ms | capped % (≥ 59.5) | first-20 / last-20 mean fps | CPU °C start→end |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 58 | 58.45 | 58.25 | 56.50 | 15.02 | 17.11 | 25.9 | 57.80 / 58.04 | 56.1→62.8 |
| 2 | 58 | 58.90 | 58.63 | 56.90 | 14.77 | 16.98 | 32.8 | 58.67 / 58.23 | 56.7→62.8 |
| 3 | 57 | 58.80 | 58.55 | 56.60 | 14.89 | 17.01 | 24.6 | 58.13 / 58.95 | 56.7→63.3 |
| 4 | 57 | 58.70 | 58.54 | 56.80 | 14.72 | 17.04 | 28.1 | 58.49 / 57.94 | 56.1→62.8 |
| 5 | 58 | **59.65** | 59.27 | 57.40 | 14.33 | 16.76 | **58.6** | 59.55 / 58.88 | 56.1→62.8 |
| 6 | 58 | 58.55 | 58.56 | 56.90 | 14.68 | 17.08 | 25.9 | 58.38 / 58.45 | 56.7→62.8 |

Across legs:
- fps median: mean **58.84**, sd **0.43** (CV 0.73 %).
- fps mean: 58.63 ± 0.34.
- runFrame median: 14.74 ± 0.23 ms.
- wall/frame: 17.07 ± 0.10 ms.

Cooldown between legs took 83–204 s. Each leg (cool + launch + verify + 60 s) took ~5 min.

### A.3 Sustained leg (one cold start, 5 consecutive 60 s samples)

| minute | fps median | fps mean | capped % | runFrame median | CPU °C | clock (kHz) / cooling_device0 |
|---|---|---|---|---|---|---|
| 1 | 59.15 | 58.49 | 41 | 14.34 | 56.1→62.8 | 1992000 / 0 |
| 2 | 59.70 | 59.18 | 57 | 14.43 | 63.3→66.2 | 1992000 / 0 |
| 3 | 59.70 | 59.11 | 58 | 14.18 | 66.2→68.1 | 1992000 / 0 |
| 4 | 59.80 | 59.67 | 75 | 13.36 | 68.8→69.4 | 1992000 / 0 |
| 5 | 60.00 | 59.61 | 85 | 13.28 | 69.4→70.0 (`mStatus=3` at the end) | 1992000 / 0 |

Interpretation: over the first 5 minutes, scene lightening dominates heating. Fps *rises* while the
temperature crosses the 65 °C gate (~2 min) and reaches the 70 °C hot status. The minute-1 numbers
of this leg (59.15 median) also show leg-to-leg scene variance relative to A.2.

Fast-forward check, immediately after this leg (71 °C, late race, feasibility only). Kept as history:
under the §3.0 rule fast-forward is not a measurement mode, and these numbers are not results.

| mode | windows (fps / runFrame ms) |
|---|---|
| capped | 59.8/12.9, 59.7/13.5, 60.6/12.5 |
| **FF (unlimited)** | 65.5/14.2, 60.8/15.4, 64.1/14.7, 63.0/15.0, 60.8/15.7, 62.3/14.8 |
| capped again | 59.9/12.3, 60.0/13.4 |

### A.4 Correctness-gate determinism
- Device, `liteDS-app-fixed`, race-fresh 500 f, `--fb-dump-ppm` 100/250/400 × 4 runs: **4/4
  identical** at each frame.
- Host (Mac), same flags, 300 f, 6 runs: f250 6/6 identical; **f100 and f299 split 3/3 into two
  variants** (8.8 % px). `audio_hash` identical in all JIT runs. Interp runs fully identical.
  JIT ≠ interp by design under the shipping flags.

<a id="appendix-b"></a>
## Appendix B: sources
- Code:
  - `app/src/main/cpp/MelonInstance.cpp`: `updateRenderer`, the `LITEV_PROF` window, the frameskip prop.
  - `app/src/main/cpp/MelonDSAndroidJNI.cpp`: limiter, overlay fps, ADPF, the Auto frameskip
    controller (`LITEV_AUTOFS`).
  - `res/values/arrays.xml`: fast-forward `-1` = unlimited.
  - `melonDS-android-lib/tools/android-bench/*.sh`, `tools/headless/main.cpp`.
  - `melonDS-profiler/tools/profiler/*`.
- Docs:
  - `INVESTIGATION-LOG.md`: thermal validity gate, softprof perturbation, affinity sweeps.
  - `PATH-TO-60FPS-2026-07-17.md`: scene-matched rebaseline, iso-thermal ABAB, retractions.
  - `PROFILER-MASTER-PLAN.md`, `DIAGNOSIS-COMPLETENESS-GATE.md`, `DIAGNOSTIC-PLAYBOOK.md` (the
    diagnosis parts now live in this repo's `ANALYSIS-METHODOLOGY.md`).
  - Dossier `perf-evidence.md`, `perf-timeline.md`.
- Memory notes:
  - `measure-whole-frame-not-one-thread`, `tool-selection-…`, `headless-must-use-shipping-renderer`
  - `reliable-bottleneck-diagnosis-sample-on-miss` (async fb hash, l2 drift, app contention)
  - `app-savestate-load-and-emu-profile` (contention ≠ thermal; target = the throttled state)
  - `renderer-prop-regression-and-core3`, `verify-build-flags-actually-set`, `wrong-package-…`
  - `rgds-device-quirks`, `diagnose-by-reading-state-and-code`
