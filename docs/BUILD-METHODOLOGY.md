# Build Methodology — liteDS (melonDS performance fork) for the Anbernic RG DS

Status: **verified by execution on 2026-10-01** (Mac arm64 host, RG DS on Android 14, adb serial
`192.168.0.165:44675`). Each command marked **[V]** was run exactly as written (apart from path
variables), and its measured duration is shown. **[UNVERIFIED]** marks steps that were not run
this time.

Companion documents: `TESTING-METHODOLOGY.md` covers how to measure once the build is installed;
`ANALYSIS-METHODOLOGY.md` covers how to diagnose a correctness failure.

**Which trees this describes.** The lib is this repository, branch `litev-clean` (one commit per
optimisation on upstream `10a173b5`). The app is `melonDS-android-app-clean`, branch `app-clean`
(10 commits on upstream `ae8790bd`), whose `melonDS-android-lib` submodule points at a `litev-clean`
commit. In this history `LITEV_RENDER_THREAD` and `LITEV_GL_STATE_CACHE` no longer exist, the app pins
the emulator thread to core 3 unconditionally, and `build.gradle.kts` passes exactly the 49 shipping
optimisation flags plus `LITEV_AUTO_FRAMESKIP` (the Auto frameskip UX setting, added by the 10th app
commit; it is inert unless the setting is on, and the setting must be off when measuring, see
TESTING §3.0). The commands below were verified on 2026-10-01 against the pre-history trees (app `0b66608d`,
lib `e9b8222f`); they apply unchanged to the clean trees unless a note says otherwise. Mentions of the
old trees are kept only as history notes.

---

## Contents

1. [What gets built](#1-what-gets-built)
2. [Prerequisites](#2-prerequisites)
3. [Get a clean source tree (never build the dirty working tree)](#3-get-a-clean-source-tree)
4. [The LITEV flag set: where it is defined, and proving it reached the compiler](#4-the-litev-flag-set)
5. [Build the APK](#5-build-the-apk)
6. [Install and fingerprint on the device](#6-install-and-fingerprint)
7. [Device and app setup (ROM, savestate, grants, renderer)](#7-device-and-app-setup)
8. [Launch and load the Shrek race (slot 2) robustly](#8-launch-and-load-the-shrek-race)
9. [Confirm the right package, process, thread and scene](#9-confirm-the-right-thing-is-running)
10. [Build the headless benchmark binary for the device](#10-headless-device-build)
11. [Host (Mac arm64) headless build for correctness gates](#11-host-headless-build)
12. [Pitfall register](#12-pitfall-register)
13. [Appendix: the verification run of 2026-10-01](#13-appendix-verification-run)

---

## 1. What gets built

| Artifact | Built from | Used for |
|---|---|---|
| `.dev` APK (`me.magnum.melonds.dev`) | `melonDS-android-app-clean` (branch `app-clean`) + submodule `melonDS-android-lib` (branch `litev-clean` of this repo) | Whole-frame truth: real app fps on the RG DS |
| `liteDS-headless` (Android arm64, static) | `melonDS-android-lib/tools/android-bench/build.sh <config>` | Deterministic on-device instruction/PMU counts, correctness gates (trace/PPM) |
| `liteDS-headless` (macOS arm64) | plain CMake on the host | Fast byte-exact and JIT-vs-interpreter gates. The A64 JIT runs on Apple Silicon. |

The **shipping configuration** is whatever `app/build.gradle.kts` passes to CMake. Every other
build has to be derived from that file and checked against it (§4).

## 2. Prerequisites

Verified on the build Mac:

| Item | Value | How to check |
|---|---|---|
| JDK | **21** — `/opt/homebrew/opt/openjdk@21/libexec/openjdk.jdk/Contents/Home` (21.0.10) | `$JAVA_HOME/bin/java -version` |
| Android SDK | `~/android-sdk` (`sdk.dir` in `local.properties`, which is git-ignored) | `cat local.properties` |
| NDK for the APK | **28.0.13004108** (`AppConfig.ndkVersion`, `buildSrc/src/main/.../AppConfig.kt`) | `ls ~/android-sdk/ndk` |
| NDK for headless `build.sh` | **27.0.12077973** (script default; override with `ANDROID_NDK=`) | see pitfall P9 |
| CMake (APK) | 3.22.1 from the SDK (`externalNativeBuild.cmake.version`) | `ls ~/android-sdk/cmake` |
| CMake (host) | Homebrew 4.2.3 + ninja | `cmake --version` |
| Gradle | wrapper 9.5.0 (`gradle/wrapper/gradle-wrapper.properties`) | |
| compile/target SDK | 36; minSdk 24; ABI **arm64-v8a only** | `AppConfig.kt`, `abiFilters` |
| Signing | `local.properties` points `MELONDS_KEYSTORE` at `~/.android/debug.keystore`. All builds therefore share one key and `adb install -r` upgrades in place. | |
| adb | wireless, two transports per device (IP and mDNS) | `adb devices -l` |

**Always set `ANDROID_SERIAL` explicitly.** The RG DS appears twice (`192.168.0.165:44675` and an
`adb-…-tls-connect` mDNS alias), and an AYN Thor is also attached. Never run an unqualified `adb`.
`app-race.sh`'s `preflight` picks "the first device", which is wrong in this setup.

```sh
export ANDROID_SERIAL=192.168.0.165:44675        # RG DS (model RG_DS, product rk3568_u)
adb shell getprop ro.product.model                # must print: RG DS
adb shell pm list packages | grep melon           # me.magnum.melonds.dev present
```

The wireless-adb IP and port change between sessions. To (re)connect:

```sh
adb mdns services                                 # find <ip>:<port> for _adb-tls-pairing / _adb-tls-connect
adb pair <ip>:<pairPort> <6-digit code>           # code from "Pair device with pairing code" on the device
adb connect <ip>:<connectPort>
export ANDROID_SERIAL=<ip>:<connectPort>          # then pin it as above
```

## 3. Get a clean source tree

A dirty working tree must never become the measured "shipping" build. (History note: on 2026-10-01
the pre-history lib working tree carried an uncommitted GEOMLOG diagnostic and whitespace edits, and an
**uncommitted** JIT probe had already shipped a miscompile once, the Pokémon White "white professor".)

Do **not** checkout, stash or reset in the user's repos. Instead, export the committed trees with
`git archive`. This changes no git state, not even worktree metadata. **[V] 1 s**

```sh
SP=<scratch dir>; T=$SP/bt/app; A=<path to melonDS-android-app-clean>      # branch app-clean
LIB_REV=$(git -C $A ls-tree HEAD melonDS-android-lib | awk '{print $3}')   # the litev-clean commit the app pins
mkdir -p $T
git -C $A archive HEAD | tar -x -C $T                                      # app @ HEAD
git -C $A/melonDS-android-lib archive $LIB_REV | tar -x -C $T/melonDS-android-lib
for s in enet faad2 oboe; do                                               # the app's other submodules
  c=$(git -C $A ls-tree HEAD app/src/main/cpp/$s | awk '{print $3}')
  git -C $A/app/src/main/cpp/$s archive $c | tar -x -C $T/app/src/main/cpp/$s
done
cp $A/local.properties $T/                                                 # sdk.dir + signing (git-ignored)
```

- To build a *different* lib commit, replace `$LIB_REV`. Record the pair `(app HEAD, lib rev)` with
  every measurement.
- Alternative: `git -C <lib> worktree add --detach <dir> <rev>` (+ an app worktree). It works, but it
  writes worktree metadata into the user's `.git` and requires the submodules to be populated by
  hand. `git archive` is simpler. **[UNVERIFIED in this run]**
- Building **in place** (`cd $A && ./gradlew …`) is only acceptable when
  `git -C $A/melonDS-android-lib status --porcelain --untracked-files=no` is empty **and**
  `git -C $A/melonDS-android-lib rev-parse HEAD` equals `$LIB_REV`. Otherwise the APK contains
  uncommitted code.

## 4. The LITEV flag set

### 4.1 Where it is defined

- **Shipping set:** `app/build.gradle.kts`, `defaultConfig.externalNativeBuild.cmake.arguments(…)`,
  one `"-DLITEV_<NAME>=ON|OFF"` string per flag. In `app-clean` these are exactly the 49 shipping
  flags, all `=ON`, plus `-DLITEV_AUTO_FRAMESKIP=ON` from the 10th app commit (the pre-history app
  had 57 live entries, 55 ON, including options that this history dropped). Against a lib commit
  older than the `LITEV_AUTO_FRAMESKIP` option the extra `-D` is an unused CMake variable (it lands
  in the cache as `LITEV_AUTO_FRAMESKIP:UNINITIALIZED=ON`, which the §4.2 check reads as `ON`) and
  the app compiles the feature out, so any `litev-clean` prefix still builds with the app tip.
- **Option declarations and defaults:** `melonDS-android-lib/CMakeLists.txt` `option(LITEV_… OFF)`.
  Almost all of them default OFF, so the app turns them on.
- **Per-option documentation:** `docs/LITEV-OPTIMIZATIONS.md` in the lib (one section per option, the
  same text as the commit that added it); dropped options are in `docs/NEGATIVE-RESULTS.md`.
- **Runtime props:** many flags also have a `debug.litev.*` property (see TESTING §3.6). Props are
  separate from compile flags and are **boot-volatile**.
- `-O3`: the `.dev` variant is a Gradle *Debug* build, so CMake would default to `-O0`.
  `build.gradle.kts` overrides `CMAKE_{C,CXX}_FLAGS_DEBUG="-O3 -DNDEBUG"`. Native code in `.dev` is
  therefore release-grade. The Kotlin side is still debuggable (ART interprets it), and the
  emulator does not care.

### 4.2 Proving the flags reached the compiler (mandatory after every build)

A build that "succeeds" with a flag silently OFF gives a vacuous result (see memory
`verify-build-flags-actually-set`). Check the CMake cache and the actual compile line. **[V]**

```sh
C=$(ls $T/app/.cxx/Debug/*/arm64-v8a/CMakeCache.txt | head -1)
# 1) every live gradle flag has the same value in the cache  (strip // comment lines FIRST — see P1)
grep -vE '^[[:space:]]*//' $T/app/build.gradle.kts | grep -oE '"-DLITEV_[A-Z0-9_]+=(ON|OFF)"' | tr -d '"' | sed 's/^-D//' |
  while IFS== read k v; do got=$(grep "^$k:" $C | cut -d= -f2); [ "$got" = "$v" ] || echo "MISMATCH $k want=$v got=${got:-absent}"; done
# expected output: nothing
# 2) optimisation level really is -O3
grep -m1 -o 'CMAKE_CXX_FLAGS_DEBUG:STRING=.*' $C                      # -O3 -DNDEBUG
grep -m1 -oE 'FLAGS = .*' $(dirname $C)/build.ninja | grep -o -- '-O3'  # on the real compile line
# 3) a flag's -D define is on the compile lines
grep -c -- '-DLITEV_TILE_COORD' $(dirname $C)/build.ninja            # > 0
```

For host CMake builds made from zsh, never use `cmake … $FLAGS`. zsh does not word-split, so the
whole string becomes the value of the first `-D`. Use an array (`"${flags[@]}"`) or `${=FLAGS}`,
then `grep LITEV_X:BOOL $builddir/CMakeCache.txt`.

### 4.3 App-level build options (in the app's CMakeLists, not this repo)

The SereneDS app (`app-clean`) adds compile/link options around this core that are not LITEV
core options: `LITEV_THINLTO` (ThinLTO; this repo's own LTO only applies to
`CMAKE_BUILD_TYPE=Release`, which the app's RelWithDebInfo never was), `LITEV_MTUNE_A55`,
and two **game-trained** ones, `LITEV_HOT_ORDER` (`hot-symbols.order`) and `LITEV_PGO_USE`
(`pgo/core-cs.profdata`). How to retrain those on new games, gate and measure them:
`melonDS-profiler/docs/PROFILE-GUIDED-BUILD.md` (tools: `hot-order.py`, `pgo-train.sh`).

## 5. Build the APK

**[V] 6 min 38 s** cold (fresh tree, warm Gradle caches, 71 tasks):

```sh
cd $T
JAVA_HOME=/opt/homebrew/opt/openjdk@21/libexec/openjdk.jdk/Contents/Home \
  ./gradlew :app:assembleGitHubProdDebug --console=plain 2>&1 | tee $SP/logs/gradle.log
# -> $T/app/build/outputs/apk/gitHubProd/debug/app-gitHub-prod-debug.apk  (~38 MB)
# unstripped lib for symbolisation (capture.sh -s):
#    $T/app/build/intermediates/cxx/Debug/<hash>/obj/arm64-v8a/libmelonDS-android-frontend.so (~77 MB)
```

- **Variant:** `gitHub` (version) × `prod` (build) × `debug` gives `applicationIdSuffix ".dev"`, so the
  package is `me.magnum.melonds.dev`. Never measure `me.magnum.melonds` (the release package). That
  mistake cost a whole session (memory `wrong-package-and-r4-compiled-out`).
- **JDK:** in this session the default `java` was already 21. Older shells had Homebrew 17 on PATH,
  which fails with `invalid source release: 21`. Pass `JAVA_HOME` inline every time. The Bash tool
  does not re-read `~/.zprofile` mid-session.
- **Incremental builds:** after editing a `CMakeLists.txt`, a struct layout, or the gradle CMake
  arguments, delete `app/.cxx` (and `app/build/intermediates/cxx`) before rebuilding. AGP reuses
  stale objects and stale `.so` files. Stale struct layouts produced a repeatable "frame-9
  corruption" signature three times.
- `capture.sh` auto-locates the unstripped `.so` only under
  `../melonDS-android-app/app/build/intermediates/...`. When you build elsewhere, pass
  `-s <unstripped .so>`.

## 6. Install and fingerprint

**[V] 8 s**

```sh
APK=$T/app/build/outputs/apk/gitHubProd/debug/app-gitHub-prod-debug.apk
adb install -r "$APK"                      # -r: keep data (prefs, SAF grants, savestates)
# fingerprint: the installed APK must be byte-identical to the one you built
adb shell md5sum $(adb shell pm path me.magnum.melonds.dev | sed 's/package://' | tr -d '\r')
md5 -q "$APK"                              # must match
adb shell dumpsys package me.magnum.melonds.dev | grep lastUpdateTime
```

- **Never `adb uninstall`.** Uninstalling wipes prefs **and the SAF URI grant** for the ROM folder.
  The symptoms are an empty ROM list or empty savestate slots. Recovery needs root: inject the grant
  into `/data/system/urigrants.xml` (ABX: `abx2xml`/`xml2abx`), then `su 0 sh -c 'stop && start'`
  (memory `rgds-device-quirks`). **[UNVERIFIED this run; not needed]**
- After any install, **force-stop and relaunch**. Native libraries, and props latched with
  `static` caching, are read only at process start.

## 7. Device and app setup

State found and used on 2026-10-01:

| Thing | Location / value |
|---|---|
| ROM (app) | `/storage/0123-4567/DS/Shrek - Smash n' Crash Racing (USA).nds` (SD card; the app reaches it via its SAF grant). `app-race.sh`'s `ROM_DIR=/sdcard/Documents/DS` is **stale**. |
| Race savestate (app) | `/storage/0123-4567/DS/Shrek - Smash n' Crash Racing (USA).ml2` = **slot 2**, labelled "Wed, 04 Feb 2026 06:50:24" |
| Other games on the device | Pokémon White (`.ml1` savestate), Pokémon White 2. Use them as the second validation game. |
| Renderer | **Software is the default** in code (`MelonInstance.cpp::updateRenderer`). It is used unless `debug.litev.software=0`. The app pref `video_renderer=opengl` is **overridden** by this. Do not change the pref. |
| Profiler log stream | `adb shell setprop debug.litev.prof 1` (set **before** launch) gives one `LITEV_PROF` line per 60 frames. Keep `debug.litev.softprof=0`: the colour-effect census performs ~5.9 M atomics per 60 frames and slows the 2D worker. |
| Other `debug.litev.*` props | **None were set** on 2026-10-01 (6-day uptime). Check before every session: `adb shell getprop | grep litev`. A stale `debug.litev.frameskip`, `norender`, `software=0`, `geoprefetch=0`, … silently changes what you measure. |
| CPU governor | `performance`, all 4 cores at 1 992 000 kHz (found that way, not changed) |
| Thermal trip | `soc-thermal` trip_point_1 = 85 °C (stock). `race-tune.sh` raises it to 100 °C. That is a bench-only, boot-volatile, **non-shipping** change (see TESTING §3.2). |
| Screen-on | `adb shell svc power stayon true` (one window was once ruined by the screen sleeping) |

Headless data on the device (read-only inputs; copy them, never write into them):

```
/data/local/tmp/liteds/shrek.nds                 md5 9737b519bb0181d35aa631be7863b55a (keep a host copy with the same md5)
/data/local/tmp/liteds/hdata/race-fresh.mln      md5 1ac25312…  (scene used by abtest.sh / quickprof.sh; no input)
/data/local/tmp/liteds/shrek-race.mln            md5 7526dc6e…  (scene used by run-race.sh; with hold-A script)
/data/local/tmp/liteds/shrek-race-hold-a.script  "0 A"
/data/local/tmp/liteds/headless-data/headless.sav   game save; the GAME REWRITES it during a run (see P7)
```

These are **two different race savestates**. Results from `abtest.sh`/`quickprof.sh` and from
`run-race.sh` are not comparable with each other (TESTING §3.1).

## 8. Launch and load the Shrek race

**Resolve every tap by text with `uiautomator`. Never use hard-coded coordinates.** On 2026-10-01
the coordinates in the tooling were wrong:

| Source | Coordinate it uses | Where that lands on 2026-10-01 |
|---|---|---|
| `app-race.sh` `ROM_ROW_XY="250 168"` (fallback) | ROM list row | **Pokémon White 2** (Shrek is at y≈234) |
| `app-race.sh` `SLOT_XY="213 266"` (default) | slot dialog | **slot 3 `<Empty>`**: nothing loads and the emulator idles on the title |
| historical `SLOT_XY` values in chat logs: `320 176`, `280 200`, `320 190`, `320 287` | — | they drift with app version and dialog layout |

The verified sequence. `tools/measure/app-run.sh` and `tools/measure/ui.py` implement it. **[V] 42 s
end-to-end**

```sh
adb shell svc power stayon true
adb shell setprop debug.litev.prof 1; adb shell setprop debug.litev.softprof 0
adb shell am force-stop me.magnum.melonds.dev
adb shell monkey -p me.magnum.melonds.dev -c android.intent.category.LAUNCHER 1; sleep 4
# 1. ROM list -> tap the node whose text matches 'Shrek.*\.nds$'   (center was 328,246)
# 2. wait until dumpsys shows ResumedActivity …EmulatorActivity, then ~8 s for the game to boot
# 3. adb shell input keyevent 4        (BACK -> "Pause" dialog: Settings / Save state / Load state / …)
# 4. tap text '^Load state$'           (center 320,239)
# 5. slot dialog lists "Quick Slot. <Empty>", "1. <Empty>", "2." + "Wed, 04 Feb 2026" + "06:50:24", "3. <Empty>"…
#    tap the date node that FOLLOWS the "2." node   (center 335,199)
```

`ui.py` reads a uiautomator dump on stdin. With no argument it lists `x y text` for every text
node; with a regex argument it prints the centre of the first match:

```sh
adb shell uiautomator dump /sdcard/ui.xml >/dev/null; adb exec-out cat /sdcard/ui.xml | python3 ui.py 'Load state'
```

Failure modes and recoveries:
- **Emulator paused** (EmulatorThread at 0 ticks after some navigation or focus loss):
  `adb shell am start -n me.magnum.melonds.dev/me.magnum.melonds.ui.emulator.EmulatorActivity`.
- **Taps go to the wrong place:** re-dump and re-resolve the coordinates. One scripted run once
  opened the clock app.
- **The user is physically using the device.** Taps fight with the user. Do not run long unattended
  UI loops while the user may be holding the device.
- adb game input (`input keyevent`, `sendevent`) does **not** reach the emulated game. Only the
  Android UI (dialogs) responds. The race is therefore entered only through Load state.

## 9. Confirm the right thing is running

Run all of these after every launch (the `app-run.sh start` checks). **[V]**

```sh
PID=$(adb shell pidof me.magnum.melonds.dev | tr -d '\r')                       # 1. right package is alive
adb shell dumpsys activity activities | grep ResumedActivity                    # 2. …EmulatorActivity is resumed
adb shell "ps -T -p $PID -o TID,CMD" | grep -E 'EmulatorThread|s3d-tile|s2d-async'   # 3. 3x s3d-tile => software TILE renderer
TID=$(adb shell "ps -T -p $PID -o TID,CMD" | awk '/EmulatorThread/{print $1; exit}')
a=$(adb shell cat /proc/$PID/task/$TID/stat | awk '{print $14+$15}'); sleep 2
b=$(adb shell cat /proc/$PID/task/$TID/stat | awk '{print $14+$15}'); echo "emu busy $(( (b-a)/2 ))%"   # 4. race: ~65–97 %
adb shell "su 0 screencap -p -d 1 /sdcard/d1.png"; adb pull /sdcard/d1.png .   # 5. TOP screen = 3D race view ("4th", "Lap 1/3")
adb shell "su 0 screencap -p -d 0 /sdcard/d0.png"; adb pull /sdcard/d0.png .   #    BOTTOM = minimap + timer + "FPS: NN" overlay
```

- **Screenshot rule (recurring mistake): the game is on Display 1.** The RG DS is a dual-screen
  clamshell with two physical displays. Always capture with root and `-d 1`:
  `adb shell "su 0 screencap -p -d 1 /sdcard/d1.png" && adb pull /sdcard/d1.png`. A plain
  `screencap` (no `-d`) captures Display 0, which carries the control overlay and the `FPS:`
  counter; concluding "the game is not rendering" from it is wrong. Sanity check by size: a game
  capture is tens of KB (40–80 KB), a blank control display is ~7 KB. List the displays with
  `adb shell dumpsys SurfaceFlinger --display-id` (Display 0 = Primary, Display 1 = Secondary).
- **Display mapping:** `-d 1` is the TOP screen with the behind-kart 3D view (80 KB PNG). `-d 0` is
  the BOTTOM/touch screen with the map, the race timer, the FPS overlay and the touch overlay (63 KB).
  A ~7 KB capture means you grabbed a blank control display. **With `su 0` the capture works.** The
  older memory claim "screencap is BLACK (FLAG_SECURE)" is out of date for root captures (verified
  2026-10-01).
- The race timer on display 0 tells you **how far into the race** the scene is. Record it, because
  the scene changes as the race plays out (TESTING §3.1).
- The ps listing shows two threads named `EmulatorThread`. The busy one, listed first by `ps -T`
  and busiest in `top -H`, is the emu thread. `capture.sh` picks the first one.

## 10. Headless device build

`melonDS-android-lib/tools/android-bench/build.sh <config>` cross-compiles a static `liteDS-headless`
into `build-android${BUILD_TAG}/<config>/`.

| config | Use |
|---|---|
| **`app`** | **The only representative config.** It parses the LITEV flags out of `../app/build.gradle.kts`, so it runs the shipping core **and** the shipping DRASTIC tile renderer with TILE_COORD and the 2D/3D workers. |
| `full`, `full-*` | Legacy. `full` uses the reference `SoftRenderer3D`, which never ships, so its frame cost is 3–10× wrong. Use `full` only to isolate an emu-core delta, never for anything render-adjacent. |
| `baseline/dispatch/link/es` | Historical stack bisection |

### 10.1 Fixed: `build.sh app` used to enable a flag that the APK did not

**Fixed in this history** (`litev-clean`, headless-harness commit): the `app` config now skips
commented-out gradle lines. The rest of this section is the history note that motivated the fix.

`config_flags app` greps every `"-DLITEV_…=ON|OFF"` string in `build.gradle.kts` **including
commented-out lines**. At app HEAD, line 335 is

```
                    // , "-DLITEV_GEOM_OFFLOAD=ON"
```

so `liteDS-app` is built with **`LITEV_GEOM_OFFLOAD=ON`**, while the APK has it **OFF** (verified:
`build-android-asis/app/CMakeCache.txt` has `LITEV_GEOM_OFFLOAD:BOOL=ON`; the APK cache has `OFF`).
GEOM_OFFLOAD is not runtime-gated. It replaces exact geometry timing with an approximate model
(`GPU3D.cpp` "G2 OFFLOAD … APPROXIMATE cycle model"). As a result the headless "shipping" binary
emulates differently: on the same scene the `final_top` and `audio_hash` differ
(`b1f77c42…/dd9af824…` as-is vs `97b2d128…/5072399c…` fixed), and it executes **+1.8 %
instructions** (TESTING Appendix A.1). A telltale sign is that the binary prints
`geom_event_peak:` lines.

The bug has existed since the `app` config was added (lib `e0c7781e`, 2026-09-16). Every
`liteDS-app` headless number since then carries it.

**Fix** (one line; verified in a scratch copy as `build-fixed.sh`):

```sh
# tools/android-bench/build.sh, config `app`:
-      grep -oE '"-DLITEV_[A-Z0-9_]+=(ON|OFF)"' "$g" | tr -d '"' | grep -v 'LITEV_PROFILE='
+      grep -vE '^[[:space:]]*//' "$g" | grep -oE '"-DLITEV_[A-Z0-9_]+=(ON|OFF)"' | tr -d '"' | grep -v 'LITEV_PROFILE='
```

After the fix, the headless cache's ON set equals the APK cache's ON set exactly, apart from
`LITEV_HEADLESS`. Check it like this:

```sh
H=<lib>/build-android-fixed/app/CMakeCache.txt; Ap=$C   # C from §4.2
for k in $(grep -oE '^LITEV_[A-Z0-9_]+:BOOL=ON' $H | cut -d: -f1); do grep -q "^$k:BOOL=ON" $Ap || echo "headless-only: $k"; done
for k in $(grep -oE '^LITEV_[A-Z0-9_]+:BOOL=ON' $Ap | cut -d: -f1); do grep -q "^$k:BOOL=ON" $H || echo "apk-only: $k"; done
# expected: only "headless-only: LITEV_HEADLESS"
```

### 10.2 Build, push, run

**[V] 83 s (as-is) / 67 s (fixed)**

```sh
cd $T/melonDS-android-lib
BUILD_TAG=-fixed ./tools/android-bench/build.sh app        # (with the fix applied)
grep -E '^LITEV_(TILE_COORD|SOFT3D_DRASTIC|PROFILE):' build-android-fixed/app/CMakeCache.txt
# push into YOUR OWN directory; copy the inputs instead of writing into /data/local/tmp/liteds
adb shell "su 0 sh -c 'mkdir -p /data/local/tmp/liteds-aa/dd0 && cd /data/local/tmp/liteds-aa && \
  cp ../liteds/shrek.nds ../liteds/shrek-race.mln ../liteds/shrek-race-hold-a.script ../liteds/hdata/race-fresh.mln . && \
  cp ../liteds/headless-data/headless.sav dd0/ && chmod 777 . dd0'"
adb push build-android-fixed/app/liteDS-headless /data/local/tmp/liteds-aa/liteDS-app-fixed
adb shell chmod 755 /data/local/tmp/liteds-aa/liteDS-app-fixed
# smoke run (fresh data dir per run, see P7)
adb shell "su 0 sh -c 'cd /data/local/tmp/liteds-aa && rm -rf dd && cp -r dd0 dd && \
  ./liteDS-app-fixed --rom shrek.nds --savestate race-fresh.mln --frames 500 --fixed-rtc 1600000000 \
  --mode jit --fastmem on --frameskip 0 --data-dir dd'"
# -> avg_fps ~73, final_top 97b2d128cd7c0f72, final_bot dc20a08293661d34, audio_hash 5072399c79dc0e32
```

Differences that remain between headless `app` and the APK (beyond the Android layer): NDK 27 vs 28
compiler, `-std=gnu++17` vs `c++17`, no `-fno-emulated-tls`, static vs shared linking, and no
GL/present/ADPF. Headless **fps is not app fps** (TESTING §2).

`--profile-json`, `--record-trace/--verify-trace`, `--fb-dump-ppm f:path[,f:path]`, `--fb-hash-every`,
`--bench-window s:e`, `--input-script`, `--fixed-rtc` and `--mp-test` are documented in
`tools/headless/main.cpp` `Usage()`. Set `LITEV_PROFILE=ON` for a per-part ns breakdown. That build
is instrumented, so do not use it for timing.

## 11. Host headless build

**[V] 28 s** (Ninja, 101 steps). It uses the same live flag list as the APK:

```sh
cd $T/melonDS-android-lib
flags=( $(grep -vE '^[[:space:]]*//' ../app/build.gradle.kts | grep -oE '"-DLITEV_[A-Z0-9_]+=(ON|OFF)"' | tr -d '"' | grep -v LITEV_PROFILE=) )
cmake -S . -B build-host-app -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_QT_SDL=OFF -DENABLE_OGLRENDERER=OFF \
  -DENABLE_GDBSTUB=OFF -DENABLE_JIT=ON -DENABLE_LTO=OFF -DENABLE_LTO_RELEASE=OFF -DLITEV_HEADLESS=ON -DLITEV_PROFILE=OFF "${flags[@]}"
cmake --build build-host-app --target liteDS-headless -j 8
grep -c '^LITEV_.*:BOOL=ON' build-host-app/CMakeCache.txt     # 51 (49 shipping + AUTO_FRAMESKIP + HEADLESS); 50 before the option existed; pre-history: 56
```

The ROM on the host is a local copy of the device's `shrek.nds` (check the md5 in §7). Pull the
savestate with `adb pull /data/local/tmp/liteds/hdata/race-fresh.mln`. Any app ROM or savestate on
the SD card (for example the Pokémon White `.ml1` used as the second game) can be pulled as root,
which is how a device-only repro becomes a deterministic host repro (`ANALYSIS-METHODOLOGY.md`):

```sh
adb exec-out "su 0 cat '/storage/0123-4567/DS/<rom>.nds'" > rom.nds
adb exec-out "su 0 cat '/storage/0123-4567/DS/<rom>.ml1'" > rom.ml1
```

What the host build is good for, measured 2026-10-01 (300 frames of `race-fresh.mln`):

| Check | Result | Interpretation |
|---|---|---|
| JIT run 3× | `audio_hash` identical (b1cfb685…); `final_top` **two different values** | Guest state is deterministic. The top framebuffer is **async** under TILE_COORD. |
| `--fb-dump-ppm` at frames 100/250/299, 6 runs | f250: 6/6 identical. **f100 and f299: two variants each, 3/3, 8.8 % px apart** | The settled-PPM gate is **not** deterministic at every frame (TESTING §3.8) |
| interp run 3× | fully identical (top, audio, PPM) | |
| JIT vs interp | differ (audio and 8.8 % px) | With the shipping flags, JIT ≠ interp **by design**: EVENT_SLICES and the other category-B timing levers do not apply to the interpreter path. Use JIT-vs-interp to *localise* (does interp look right?), not as a byte-equality gate, unless you build with the exact-timing configs. |

The host is ~13× faster than the device (JIT 973 fps headless). Use it for gates, never for
performance numbers.

## 12. Pitfall register

| # | Pitfall | Symptom | Control |
|---|---|---|---|
| P1 | (fixed in litev-clean) `build.sh app` picked up **commented-out** gradle flags (`GEOM_OFFLOAD`) | headless ≠ shipping (different hashes, +1.8 % instructions) | Apply the §10.1 fix; diff the CMakeCache ON-sets |
| P2 | Building the dirty lib working tree | uncommitted probes ship (white-professor JIT miscompile) | `git archive` the committed rev (§3) |
| P3 | Measuring `me.magnum.melonds` instead of `.dev` | "no-op" changes | Always use `.dev`; fingerprint by APK md5 (§6) |
| P4 | zsh does not word-split `$FLAGS` | flag silently OFF, vacuous "byte-exact" result | arrays/`${=FLAGS}`; grep the CMakeCache |
| P5 | Stale `.cxx`/`.so` after CMake or struct edits | old code runs; frame-9 corruption | `rm -rf app/.cxx app/build/intermediates/cxx` |
| P6 | JDK 17 on PATH | `invalid source release: 21` | `JAVA_HOME=<jdk21>` inline |
| P7 | Headless data dir is **mutable**: the game rewrites `headless.sav` (three different `headless.sav` md5s on the device today) | uncontrolled input drift between runs | Copy a pristine data dir per run (`cp -r dd0 dd`) |
| P8 | Hard-coded UI coordinates (`SLOT_XY 213 266`, `ROM_ROW_XY 250 168`) | wrong slot (empty), wrong game | Resolve by text (§8) |
| P9 | Headless uses NDK 27, the APK uses NDK 28 | small codegen differences | `ANDROID_NDK=~/android-sdk/ndk/28.0.13004108 ./build.sh app` **[UNVERIFIED]** |
| P10 | Unqualified `adb` with two RG DS transports plus a Thor | commands hit the wrong device or fail | `export ANDROID_SERIAL=192.168.0.165:44675` |
| P11 | `adb uninstall` | SAF grant and prefs lost; empty ROM list | Only `install -r` |
| P12 | Props are boot-volatile and some are latched (`static`) at first use | toggling a prop mid-run is a no-op or only half-engages | Set props **before** launch; force-stop and relaunch per config |
| P13 | `capture.sh` looks for the unstripped `.so` only in the in-repo build dir | sparse C++ symbols | `-s <unstripped .so>` |
| P14 | Screenshot of Display 0 (plain `screencap`) | game looks blank, "not rendering" | `su 0 screencap -p -d 1`; a ~7 KB PNG means the wrong display (§9) |
| P15 | Any frameskip active while measuring (Auto frameskip setting, fast-forward, `debug.litev.frameskip`) | a real lever reads as null; fps pinned at the target | TESTING §3.0 assertions before and after every leg |

## 13. Appendix: verification run (history note)

2026-10-01, on the pre-history trees app `0b66608d` + lib `e9b8222f` (clean `git archive`):

| Step | Command | Result | Time |
|---|---|---|---|
| export | §3 | 73 MB tree | 1 s |
| APK | `./gradlew :app:assembleGitHubProdDebug` | BUILD SUCCESSFUL, 71 tasks | 6 m 38 s |
| flag check | §4.2 | 0 mismatches (the only grep hit was the commented-out GEOM_OFFLOAD, which is correctly OFF); `-O3 -DNDEBUG` | — |
| install | `adb install -r` | Success; device APK md5 `e3990654…` == local | 8 s |
| launch + slot 2 | §8 | EmulatorActivity resumed; 3× s3d-tile; display-1 PNG 80 KB showing the race ("4th", "Lap 1/3"); display 0 showing the map, timer 00:28 and FPS 53 | 42 s |
| live fps | `LITEV_PROF` | 53–60 fps windows, runFrame 14.6–17.8 ms (see TESTING for the controlled study) | — |
| headless `app` (as-is / fixed) | §10 | built; `GEOM_OFFLOAD` ON / OFF | 83 s / 67 s |
| host headless | §11 | built; JIT/interp gate behaviour as tabulated | 28 s |
