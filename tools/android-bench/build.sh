#!/usr/bin/env bash
#
# Cross-compile the liteDS-headless CLI for Android arm64-v8a (Cortex-A55)
# using the Android NDK's CMake toolchain. Produces one static-libc++ binary
# per liteDS perf config, so the A55 benchmark matrix (Unit 7) is runnable on
# a device over adb WITHOUT porting the Android app.
#
# Usage:
#   tools/android-bench/build.sh            # build all configs
#   tools/android-bench/build.sh baseline   # build one config
#
# Output: build-android/<config>/liteDS-headless  (stripped)
#
# Configs (cumulative liteDS stack):
#   baseline  - no LITEV perf flags (upstream-equivalent core)
#   dispatch  - + LITEV_JIT_DISPATCH, links OFF
#   link      - + all LITEV_LINK_* (dispatcher + block chaining)
#   es        - + LITEV_EVENT_SLICES (event-true scheduler slices)
#   full      - + LITEV_MEM_DTCM_BLOCK + LITEV_MEM_MAINRAM_LOAD (full stack)
#
# LITEV_AGGRESSIVE_SKIP is compiled into every config so the runtime --frameskip
# knob is exercisable across the whole matrix; it is inert at frameskip 0, so the
# frameskip-0 numbers remain representative of each config.
set -euo pipefail

# ---- toolchain / device target ---------------------------------------------
NDK="${ANDROID_NDK:-/Users/shahmir/android-sdk/ndk/27.0.12077973}"
CMAKE_BIN="${CMAKE_BIN:-/Users/shahmir/android-sdk/cmake/3.22.1/bin/cmake}"
ABI="arm64-v8a"
API="${ANDROID_API:-34}"          # RG DS is Android 14 (API 34)
JOBS="${JOBS:-$(sysctl -n hw.ncpu 2>/dev/null || echo 4)}"

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TOOLCHAIN="$NDK/build/cmake/android.toolchain.cmake"

[ -f "$TOOLCHAIN" ] || { echo "error: NDK toolchain not found at $TOOLCHAIN"; exit 1; }
[ -x "$CMAKE_BIN" ] || { echo "error: cmake not found at $CMAKE_BIN"; exit 1; }

# NDK r27's llvm-strip.
STRIP="$(echo "$NDK"/toolchains/llvm/prebuilt/*/bin/llvm-strip)"

# Common flags: no Qt/SDL, no GL, no GDB; headless harness on; static libc++ so
# the binary runs standalone in /data/local/tmp; LTO off for fast, reproducible
# cross builds.
COMMON=(
  -G "Unix Makefiles"
  -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN"
  -DANDROID_ABI="$ABI"
  -DANDROID_PLATFORM="android-$API"
  -DANDROID_STL=c++_static
  -DCMAKE_BUILD_TYPE=Release
  -DBUILD_QT_SDL=OFF
  -DENABLE_OGLRENDERER=OFF
  -DENABLE_GDBSTUB=OFF
  -DENABLE_JIT=ON
  -DENABLE_LTO=OFF
  -DENABLE_LTO_RELEASE=OFF
  -DLITEV_HEADLESS=ON
  -DLITEV_PROFILE="${LITEV_PROFILE:-OFF}"
  -DLITEV_AGGRESSIVE_SKIP=ON
)

# Optional separate output tree so a profiled build (LITEV_PROFILE=ON) does not
# clobber the default non-profiled binaries. Defaults empty => unchanged paths.
BUILD_TAG="${BUILD_TAG:-}"

# Per-config LITEV flag deltas.
config_flags() {
  case "$1" in
    baseline)
      echo "-DLITEV_JIT_DISPATCH=OFF" ;;
    dispatch)
      echo "-DLITEV_JIT_DISPATCH=ON -DLITEV_LINK_UNCOND=OFF -DLITEV_LINK_COND=OFF -DLITEV_LINK_FALLTHROUGH=OFF" ;;
    link)
      echo "-DLITEV_JIT_DISPATCH=ON -DLITEV_LINK_UNCOND=ON -DLITEV_LINK_COND=ON -DLITEV_LINK_FALLTHROUGH=ON" ;;
    es)
      echo "-DLITEV_JIT_DISPATCH=ON -DLITEV_LINK_UNCOND=ON -DLITEV_LINK_COND=ON -DLITEV_LINK_FALLTHROUGH=ON -DLITEV_EVENT_SLICES=ON" ;;
    full)
      # The emulator-core stack with the reference SoftRenderer3D (use `app` for the shipping
      # renderer). Env overrides allow per-flag A/B, e.g. LITEV_LDMSTM=OFF ./build.sh full
      echo "-DLITEV_JIT_DISPATCH=ON -DLITEV_LINK_UNCOND=ON -DLITEV_LINK_COND=ON -DLITEV_LINK_FALLTHROUGH=ON -DLITEV_EVENT_SLICES=ON -DLITEV_MEM_DTCM_BLOCK=ON -DLITEV_MEM_MAINRAM_LOAD=ON -DLITEV_JIT_LDMSTM=${LITEV_LDMSTM:-ON} -DLITEV_JIT_BLOCKXFER_FAST=${LITEV_BLOCKXFER:-ON} -DLITEV_IO_DISPATCH_TABLE=${LITEV_IODISP:-OFF} -DLITEV_INSTANT_DIVSQRT=ON -DLITEV_SPU_BATCH=${LITEV_SPU_BATCH:-ON} -DLITEV_SPU_FAST_INTERP=${LITEV_SPU_FAST_INTERP:-ON} -DLITEV_SPU_MIX_NEON=${LITEV_SPU_MIX_NEON:-ON} -DLITEV_GEOM_NEON2=${LITEV_GEOM_NEON2:-OFF} -DLITEV_GEOM_NEON3=${LITEV_GEOM_NEON3:-OFF} -DLITEV_COARSE_RTC=${LITEV_COARSE_RTC:-ON} -DLITEV_DMA_GXFIFO_FAST=${LITEV_DMA_GXFIFO_FAST:-ON} -DLITEV_GXFIFO_DMA_INLINE=${LITEV_GXFIFO_DMA_INLINE:-ON} -DLITEV_DMA_TIMING_LAZY=${LITEV_DMATIMING:-OFF} -DLITEV_IDLE_AGGRESSIVE=${LITEV_IDLE_AGGRESSIVE:-OFF} -DLITEV_TIMER_FAST=${LITEV_TIMER_FAST:-OFF} -DLITEV_JIT_FLAGMERGE=${LITEV_FLAGMERGE:-ON}" ;;
    full-neon)
      # `full` core stack + integer-NEON GPU3D geometry (isolates LITEV_NEON_GEOMETRY).
      echo "-DLITEV_JIT_DISPATCH=ON -DLITEV_LINK_UNCOND=ON -DLITEV_LINK_COND=ON -DLITEV_LINK_FALLTHROUGH=ON -DLITEV_EVENT_SLICES=ON -DLITEV_MEM_DTCM_BLOCK=ON -DLITEV_MEM_MAINRAM_LOAD=ON -DLITEV_NEON_GEOMETRY=ON" ;;
    full-instantdiv)
      # `full` core stack + instant ARM9 divider/sqrt (isolates LITEV_INSTANT_DIVSQRT).
      echo "-DLITEV_JIT_DISPATCH=ON -DLITEV_LINK_UNCOND=ON -DLITEV_LINK_COND=ON -DLITEV_LINK_FALLTHROUGH=ON -DLITEV_EVENT_SLICES=ON -DLITEV_MEM_DTCM_BLOCK=ON -DLITEV_MEM_MAINRAM_LOAD=ON -DLITEV_INSTANT_DIVSQRT=ON" ;;
    full-gxthreaded)
      # `full` core stack + FIXEDREG/GLOBALREG + the threaded GXFIFO interpreter.
      echo "-DLITEV_JIT_DISPATCH=ON -DLITEV_LINK_UNCOND=ON -DLITEV_LINK_COND=ON -DLITEV_LINK_FALLTHROUGH=ON -DLITEV_EVENT_SLICES=ON -DLITEV_MEM_DTCM_BLOCK=ON -DLITEV_MEM_MAINRAM_LOAD=ON -DLITEV_JIT_FIXEDREG=ON -DLITEV_JIT_GLOBALREG=ON -DLITEV_GXFIFO_THREADED=ON" ;;
    app)
      # PRODUCTION-MATCHED: mirror the app's SHIPPING LITEV flags verbatim — including the DRASTIC
      # tile renderer (LITEV_SOFT3D_DRASTIC) + TILE_COORD + the threaded 2D/3D software raster + core
      # pinning — so headless runs the SAME emulation core AND the SAME renderer that ships. The flags
      # are PARSED from app/build.gradle.kts at configure time, so this config can never drift from
      # production. The only things headless still can't replicate are the Android layer (ADPF hints,
      # GL composite/display, system contention) — the core + software renderer are identical. This is
      # the correct default for representative profiling; `full` uses the reference SoftRenderer3D and
      # is only for isolating emu-core deltas against the non-shipping renderer.
      # LITEV_PROFILE forced OFF for clean perf runs (override with LITEV_PROFILE=ON for a per-part
      # ns breakdown). App-frontend/GL-only flags with no CMake option are silently ignored by cmake.
      local g="$REPO/../app/build.gradle.kts"
      [ -f "$g" ] || { echo "error: app build.gradle.kts not found at $g" >&2; return 1; }
      # skip commented-out gradle lines (a "// , -DLITEV_X=ON" note must not enable X here)
      grep -vE '^[[:space:]]*//' "$g" | grep -oE '"-DLITEV_[A-Z0-9_]+=(ON|OFF)"' | tr -d '"' | grep -v 'LITEV_PROFILE='
      echo "-DLITEV_PROFILE=${LITEV_PROFILE:-OFF}"
      # env-gated extra flags for validating a not-yet-shipped lever/diagnostic in liteDS-app WITHOUT
      # editing build.gradle.kts (keeps the APK clean). e.g. LITEV_SLOWMEM_HIST=ON.
      [ -n "${LITEV_SLOWMEM_HIST:-}" ] && echo "-DLITEV_SLOWMEM_HIST=${LITEV_SLOWMEM_HIST}"
      [ -n "${LITEV_MEM_DTCM_FASTMEM:-}" ] && echo "-DLITEV_MEM_DTCM_FASTMEM=${LITEV_MEM_DTCM_FASTMEM}"
      # Optional diagnostic gates above intentionally return false when unset;
      # don't let that become config_flags()'s status under `set -e`.
      true
      ;;
    *)
      echo "error: unknown config '$1'" >&2; return 1 ;;
  esac
}

build_one() {
  local cfg="$1"
  local bdir="$REPO/build-android${BUILD_TAG}/$cfg"
  local flags; flags="$(config_flags "$cfg")" || exit 1
  echo "=============================================================="
  echo ">>> configuring config=$cfg  (ABI=$ABI API=$API)"
  echo "    flags: $flags"
  echo "=============================================================="
  # shellcheck disable=SC2086
  "$CMAKE_BIN" -S "$REPO" -B "$bdir" "${COMMON[@]}" $flags
  "$CMAKE_BIN" --build "$bdir" --target liteDS-headless -j "$JOBS"

  local bin="$bdir/liteDS-headless"
  [ -f "$bin" ] || { echo "error: build produced no binary for $cfg"; exit 1; }
  local before; before=$(stat -f%z "$bin")
  "$STRIP" --strip-all "$bin"
  local after; after=$(stat -f%z "$bin")
  echo ">>> $cfg: $bin  ($before -> $after bytes stripped)"
}

CONFIGS=(baseline dispatch link es full)
if [ $# -ge 1 ]; then CONFIGS=("$@"); fi

for c in "${CONFIGS[@]}"; do
  build_one "$c"
done

echo
echo "=== binary sizes (stripped) ==="
for c in "${CONFIGS[@]}"; do
  b="$REPO/build-android${BUILD_TAG}/$c/liteDS-headless"
  [ -f "$b" ] && printf "  %-10s %8d bytes  (%s)\n" "$c" "$(stat -f%z "$b")" "$(file -b "$b" | cut -c1-40)"
done
