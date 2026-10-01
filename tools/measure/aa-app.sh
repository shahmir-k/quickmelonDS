#!/usr/bin/env bash
# A/A on the real app: same APK, REPS cold-start legs. Each leg: cool -> launch -> slot -> sample SECS.
# Summarise with: python3 aa-app-analyze.py "$OUT".  Env: see app-run.sh, plus REPS, SECS, COOL, OUT.
set -uo pipefail
: "${ANDROID_SERIAL:?}"; HERE="$(cd "$(dirname "$0")" && pwd)"
REPS="${REPS:-6}"; SECS="${SECS:-60}"; COOL="${COOL:-45}"; PKG="${PKG:-me.magnum.melonds.dev}"
LOGDIR="${LOGDIR:-$PWD/measure-logs}"; mkdir -p "$LOGDIR"; OUT="${OUT:-$LOGDIR/aa-app.txt}"
for r in $(seq 1 "$REPS"); do
  echo "=== rep $r $(date +%H:%M:%S)" | tee -a "$OUT"
  bash "$HERE/app-run.sh" cool "$COOL" 2>&1 | tee -a "$OUT"
  if ! bash "$HERE/app-run.sh" start 2>&1 | tee -a "$OUT" | grep -q 's3d-tile threads=3' || grep -q "FAIL" <(tail -3 "$OUT"); then
    echo "rep $r START FAILED" | tee -a "$OUT"; continue; fi
  bash "$HERE/app-run.sh" sample "$SECS" 2>/dev/null | sed "s/^/rep $r /" | tee -a "$OUT"
done
adb shell am force-stop "$PKG"
echo DONE | tee -a "$OUT"
