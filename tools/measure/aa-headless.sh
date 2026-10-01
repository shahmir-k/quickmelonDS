#!/usr/bin/env bash
# A/A noise-floor study for the headless race benchmark (same binary, repeated runs).
# Variants (interleaved per rep, cooldown before every run):
#   S1p = abtest/quickprof scene: race-fresh.mln, 500f, fixed RTC, taskset 08 (exactly as the tools run it)
#   S1u = same scene, NO taskset (all 4 cores available to the renderer workers)
#   S2  = run-race.sh scene: shrek-race.mln + hold-A script, 961f, --bench-window 60:960, no taskset
# Output TSV: one row per run.
# Env: ANDROID_SERIAL (required); BIN = headless binary name inside DEVDIR (default liteDS-app);
#      DEVDIR = device work dir holding BIN, shrek.nds, race-fresh.mln, shrek-race.mln,
#      shrek-race-hold-a.script and a pristine data dir dd0/ (default /data/local/tmp/liteds-aa);
#      REPS (8), COOL_C (SoC cooldown threshold, 50), OUT (default ./measure-logs/aa-headless.tsv).
set -uo pipefail
: "${ANDROID_SERIAL:?set ANDROID_SERIAL}"
REPS="${REPS:-8}"; COOL_C="${COOL_C:-50}"; BIN="${BIN:-liteDS-app}"
OUT="${OUT:-$PWD/measure-logs/aa-headless.tsv}"
D="${DEVDIR:-/data/local/tmp/liteds-aa}"; mkdir -p "$(dirname "$OUT")"
S='su 0 sh -c'
temp(){ adb shell cat /sys/class/thermal/thermal_zone0/temp | tr -d '\r'; }
freq(){ adb shell cat /sys/devices/system/cpu/cpu3/cpufreq/scaling_cur_freq | tr -d '\r'; }
cool(){ local t; for i in $(seq 1 60); do t=$(temp); [ "$t" -lt $((COOL_C*1000)) ] && return; sleep 10; done; }
adb shell am force-stop me.magnum.melonds.dev
echo -e "rep\tvariant\tavg_fps\twindow_fps\twall_s\tinstructions\tcpu_cycles\tfinal_top\tfinal_bot\taudio_hash\ttemp_pre_mC\ttemp_post_mC\tfreq_pre\tsav_md5_post" > "$OUT"
COMMON="--rom shrek.nds --mode jit --fastmem on --frameskip 0"
for rep in $(seq 1 "$REPS"); do
  for v in S1p S1u S2; do
    case $v in
      S1p) B=$BIN; PRE="taskset 08"; ARGS="$COMMON --savestate race-fresh.mln --frames 500 --fixed-rtc 1600000000";;
      S1u) B=$BIN; PRE="";           ARGS="$COMMON --savestate race-fresh.mln --frames 500 --fixed-rtc 1600000000";;
      S2)  B=$BIN; PRE="";           ARGS="$COMMON --savestate shrek-race.mln --input-script shrek-race-hold-a.script --frames 961 --bench-window 60:960";;
    esac
    cool; tp=$(temp); fp=$(freq)
    # fresh data dir per run (headless.sav is written by the game)
    out=$(adb shell "$S 'cd $D && rm -rf dd && cp -r dd0 dd && $PRE simpleperf stat -e cpu-cycles,instructions ./$B $ARGS --data-dir dd 2>/dev/null; md5sum dd/headless.sav'" | tr -d '\r')
    tq=$(temp)
    g(){ echo "$out" | awk -v k="$1" '$1==k{print $2; exit}'; }
    ins=$(echo "$out" | awk '/instructions/{gsub(/,/,"",$1); print $1; exit}')
    cyc=$(echo "$out" | awk '/cpu-cycles/{gsub(/,/,"",$1); print $1; exit}')
    sav=$(echo "$out" | awk '/headless.sav/{print $1}')
    row="$rep\t$v\t$(g avg_fps:)\t$(g window_fps:)\t$(g wall_time_s:)\t$ins\t$cyc\t$(g final_top:)\t$(g final_bot:)\t$(g audio_hash:)\t$tp\t$tq\t$fp\t$sav"
    echo -e "$row" | tee -a "$OUT"
  done
done
echo DONE >> "$OUT"
