#!/usr/bin/env bash
#
# Drive the liteDS-headless Android benchmark matrix on a device over adb.
#
# Matrix: {baseline,dispatch,link,es,full} x {fastmem on,off} x {frameskip 0,9},
# REPS runs of FRAMES frames each; plus interp for scale. Runs are interleaved
# across configs (one full pass over all cells per repetition) and separated by
# a SLEEP-second cooldown, so no single config is measured back-to-back and the
# SoC has time to shed heat. SoC temperature and A55 clock are sampled before
# every run so thermal drift is visible in the raw log.
#
# Output: a TSV (one row per run) to $OUT. Post-process for medians.
#
# Usage: tools/android-bench/run.sh [quick]
#   quick -> FRAMES=300 REPS=1 SLEEP=5 (smoke; not for the deliverable)
set -euo pipefail

DEV_DIR=/data/local/tmp/liteds
CONFIGS=(baseline dispatch link es full)
FASTMEM=(on off)
FRAMESKIP=(0 9)
FRAMES=600
REPS=3
SLEEP=30

if [ "${1:-}" = "quick" ]; then FRAMES=300; REPS=1; SLEEP=5; fi

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="${OUT:-$REPO/tools/android-bench/raw-rgds.tsv}"

soc_temp()  { adb shell 'cat /sys/class/thermal/thermal_zone0/temp' 2>/dev/null | tr -d '\r'; }
cpu_freq()  { adb shell 'cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq' 2>/dev/null | tr -d '\r'; }

# Run one invocation, echo the parsed avg_fps.
run_fps() {
  local bin="$1"; shift
  adb shell "cd $DEV_DIR && ./$bin $* --data-dir hdata 2>/dev/null" \
    | tr -d '\r' | awk '/^avg_fps:/{print $2}'
}

echo -e "ts\tconfig\tfastmem\tframeskip\trep\tavg_fps\tsoc_mC_pre\tcpu_khz_pre" > "$OUT"
echo "writing raw results to $OUT"
echo "matrix: ${#CONFIGS[@]} configs x ${#FASTMEM[@]} fastmem x ${#FRAMESKIP[@]} frameskip x $REPS reps, FRAMES=$FRAMES"

emit() { # config fastmem frameskip rep fps tpre fpre
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$(date +%H:%M:%S)" "$1" "$2" "$3" "$4" "$5" "$6" "$7" | tee -a "$OUT"
}

first=1
for rep in $(seq 1 "$REPS"); do
  for cfg in "${CONFIGS[@]}"; do
    bin="liteDS-$cfg"
    for fm in "${FASTMEM[@]}"; do
      for fs in "${FRAMESKIP[@]}"; do
        [ "$first" = 1 ] || sleep "$SLEEP"
        first=0
        tpre=$(soc_temp); fpre=$(cpu_freq)
        fps=$(run_fps "$bin" --rom shrek.nds --frames "$FRAMES" --mode jit --fastmem "$fm" --frameskip "$fs")
        emit "$cfg" "$fm" "$fs" "$rep" "${fps:-ERR}" "${tpre:-?}" "${fpre:-?}"
      done
    done
  done
done

# Interpreter scale runs (no JIT / no fastmem), frameskip 0, REPS times.
for rep in $(seq 1 "$REPS"); do
  sleep "$SLEEP"
  tpre=$(soc_temp); fpre=$(cpu_freq)
  fps=$(run_fps "liteDS-baseline" --rom shrek.nds --frames "$FRAMES" --mode interp --frameskip 0)
  emit "interp" "na" "0" "$rep" "${fps:-ERR}" "${tpre:-?}" "${fpre:-?}"
done

echo "=== done: $OUT ==="
