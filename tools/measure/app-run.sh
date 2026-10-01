#!/usr/bin/env bash
# app-run.sh — robust cold launch of the .dev app into the Shrek slot-2 race, then an fps sample.
#   app-run.sh start            force-stop, launch, open Shrek, load slot 2, verify (exit!=0 on failure)
#   app-run.sh sample <secs>    capture LITEV_PROF for <secs>, print one line per 60-frame window
#   app-run.sh cool <C>         force-stop and wait until CPU (thermalservice) < C
# Every UI target is resolved by TEXT via uiautomator (ui.py); no hard-coded coordinates.
# Env: ANDROID_SERIAL (required), PKG (default me.magnum.melonds.dev), ROM_RE (ROM list entry regex,
#      default Shrek), SLOT (savestate slot number, default 2), LOGDIR (default ./measure-logs).
set -uo pipefail
: "${ANDROID_SERIAL:?set ANDROID_SERIAL}"
HERE="$(cd "$(dirname "$0")" && pwd)"
PKG="${PKG:-me.magnum.melonds.dev}"; ROM_RE="${ROM_RE:-Shrek.*\.nds$}"; SLOT="${SLOT:-2}"
LOGDIR="${LOGDIR:-$PWD/measure-logs}"; mkdir -p "$LOGDIR"
ui(){ adb shell uiautomator dump /sdcard/ui.xml >/dev/null 2>&1; adb exec-out cat /sdcard/ui.xml; }
tap(){ local xy; for i in 1 2 3 4 5; do xy=$(ui | python3 "$HERE/ui.py" "$1") && { adb shell input tap $xy; return 0; }; sleep 1; done; echo "FAIL: no UI node /$1/" >&2; return 1; }
cpu_c(){ adb shell dumpsys thermalservice | tr -d '\r' | grep 'Temperature{.*mType=0' | tail -1 | sed -nE 's/.*mValue=([0-9.]+).*mStatus=([0-9]+).*/\1 \2/p'; }
emu_tid(){ adb shell "ps -T -p $(adb shell pidof $PKG | tr -d '\r') -o TID,CMD" | tr -d '\r' | awk '/EmulatorThread/{print $1; exit}'; }
emu_busy_pct(){ # % of one core used by the main EmulatorThread over 2 s
  local pid tid a b; pid=$(adb shell pidof $PKG | tr -d '\r'); tid=$(emu_tid)
  a=$(adb shell cat /proc/$pid/task/$tid/stat | awk '{print $14+$15}'); sleep 2
  b=$(adb shell cat /proc/$pid/task/$tid/stat | awk '{print $14+$15}'); echo $(( (b-a)/2 ))  # USER_HZ=100 ticks over 2 s -> % of one core
}
case "${1:-}" in
cool)
  adb shell am force-stop $PKG; t0=$(date +%s)
  while :; do read -r c s <<< "$(cpu_c)"; awk -v c="$c" -v t="$2" 'BEGIN{exit !(c<t)}' && break; sleep 10; done
  echo "cooled CPU=${c}C status=$s in $(( $(date +%s)-t0 ))s";;
start)
  adb shell svc power stayon true
  adb shell setprop debug.litev.prof 1; adb shell setprop debug.litev.softprof 0
  adb shell am force-stop $PKG; sleep 1
  adb shell monkey -p $PKG -c android.intent.category.LAUNCHER 1 >/dev/null 2>&1; sleep 4
  tap "$ROM_RE" || exit 1
  for i in $(seq 1 20); do adb shell dumpsys activity activities | grep -q "ResumedActivity.*EmulatorActivity" && break; sleep 1; done
  sleep 8                                  # game boot to title
  adb shell input keyevent 4; sleep 1.5    # BACK -> Pause dialog
  tap '^Load state$' || exit 1; sleep 1.5
  # slot row: the populated slot shows "N." plus a date; tap the date text of the node after "N."
  xy=$(ui | python3 "$HERE/ui.py" | awk -v n="  $SLOT." '$0 ~ n"$"{f=1; next} f{print $1, $2; exit}')
  [ -n "$xy" ] || { echo "FAIL: slot $SLOT not found / empty" >&2; exit 1; }
  echo "slot$SLOT tap at $xy"; adb shell input tap $xy; sleep 4
  adb shell dumpsys activity activities | grep -q "ResumedActivity.*EmulatorActivity" || { echo "FAIL: emulator not resumed" >&2; exit 1; }
  b=$(emu_busy_pct); echo "EmulatorThread busy=${b}% (heavy race ~65-97, menu ~45)"
  adb shell "su 0 screencap -p -d 1 /sdcard/d1.png"; sz=$(adb shell stat -c %s /sdcard/d1.png | tr -d '\r'); echo "display1 png=${sz}B (blank ~7KB)"
  adb shell "ps -T -p $(adb shell pidof $PKG | tr -d '\r') -o CMD" | tr -d '\r' | grep -c s3d-tile | sed 's/^/s3d-tile threads=/'
  [ "$b" -ge 50 ] && [ "$sz" -gt 20000 ] || { echo "FAIL: race not verified" >&2; exit 1; };;
sample)
  read -r c0 s0 <<< "$(cpu_c)"
  adb logcat -c; ( exec adb logcat -s LITEV_PROF:I > $LOGDIR/.litevprof.$$ 2>/dev/null & echo $! > $LOGDIR/.lcpid.$$; wait ) & sleep "$2"; kill $(cat $LOGDIR/.lcpid.$$) 2>/dev/null; sleep 1
  read -r c1 s1 <<< "$(cpu_c)"
  grep -oE 'runFrame=[0-9.]+.*\([0-9.]+ fps\)' $LOGDIR/.litevprof.$$ | sed -E 's/runFrame=([0-9.]+).*wall\/frame=([0-9.]+)ms \(([0-9.]+) fps\)/\3 \1 \2/' \
    | awk -v c0="$c0" -v c1="$c1" '{print "win", NR, "fps", $1, "runFrame", $2, "wall", $3, "cpuC_start", c0, "cpuC_end", c1}'
  rm -f $LOGDIR/.litevprof.$$;;
*) sed -n 2,6p "$0"; exit 2;;
esac
