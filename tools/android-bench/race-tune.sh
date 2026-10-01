#!/bin/bash
# race-tune.sh — restore the RG DS "fast state" after a reboot (2026-07-17).
# Everything here is boot-volatile on the device; run once per boot, BEFORE
# launching the app (props are read at app/emu init).
#
# What it sets and why (measured on the slot-2 Shrek race, hot device):
#   1. debug.litev.software=1  — REQUIRED. Without it the app silently falls
#      back to the OpenGL renderer, which is GPU-bound at ~40fps. The whole
#      soft-renderer perf campaign lives behind this prop. (The 07-16..17
#      "40fps regression" was exactly this prop dying on a reboot.)
#   2. debug.litev.pipedepth=2 — opt-in depth-2 soft pipeline (needs
#      LITEV_SOFT2D_DEPTH2 compiled in; harmless otherwise).
#   3. performance governor @ max on all cores.
#   4. Core-3 isolation for the emulator: pin the app's aux threads (main,
#      binder, mali workers), surfaceflinger, system_server, the Rockchip
#      power HAL, and all movable IRQs to cores 0-2. Measured: core 3 was
#      only 73.8% emulator before; this recovered ~1.5-3ms/frame
#      (46.7 -> ~50fps hot, gate 1.3 -> 0.4ms).
#
# Usage: ./race-tune.sh [serial]   (run app + load race AFTER this; the
#        app-thread pinning step needs the app running, so this script
#        applies system-side first, then re-applies app-side if the app is up)
set -u
SER="${1:-325004385b24c5c8}"
A() { adb -s "$SER" shell su 0 "$@"; }

echo "== props =="
A setprop debug.litev.software 1
A setprop debug.litev.pipedepth 2
A setprop debug.litev.prof 1
A setprop debug.litev.pipetrace 0

echo "== governor =="
A "sh -c 'for c in 0 1 2 3; do echo performance > /sys/devices/system/cpu/cpu\$c/cpufreq/scaling_governor 2>/dev/null; done; cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor'"

echo "== hold clock (raise SoC thermal trip 85C->100C) =="
# MEASURED 2026-07-17: the "performance" governor requests 1.99GHz but the SoC-thermal
# cooling device (cpufreq-cpu0) caps the A55s to 1.61GHz once soc-thermal passes trip_1
# (85C) -- a silent 19% clock loss = the single biggest fps factor (46->54fps hot when
# released). Raise trip_1 to 100C so 1.99GHz holds through a benched race; trip_2 (115C
# critical/shutdown) stays as the safety net. Active cooling keeps it ~90C.
A "sh -c 'echo 100000 > /sys/class/thermal/thermal_zone0/trip_point_1_temp 2>/dev/null; echo 0 > /sys/class/thermal/cooling_device0/cur_state 2>/dev/null; echo trip1=\$(cat /sys/class/thermal/thermal_zone0/trip_point_1_temp) clk=\$(cat /sys/devices/system/cpu/cpu3/cpufreq/scaling_cur_freq) temp=\$(cat /sys/class/thermal/thermal_zone0/temp)'"

echo "== system threads off core 3 =="
A "sh -c 'taskset -a -p 7 \$(pidof surfaceflinger) >/dev/null 2>&1; taskset -a -p 7 \$(pidof system_server) >/dev/null 2>&1; P=\$(pidof android.hardware.power-service.rockchip); [ -n \"\$P\" ] && taskset -a -p 7 \$P >/dev/null 2>&1; echo ok'"

echo "== IRQs to cores 0-2 =="
A "sh -c 'n=0; for i in /proc/irq/*/smp_affinity; do echo 7 > \$i 2>/dev/null && n=\$((n+1)); done; echo moved=\$n'"

echo "== app aux threads off core 3 (no-op if app not running) =="
PID=$(adb -s "$SER" shell pidof me.magnum.melonds.dev | tr -d '\r')
if [ -n "$PID" ]; then
  A "sh -c 'for t in /proc/$PID/task/*; do a=\$(grep Cpus_allowed_list \$t/status 2>/dev/null | cut -f2); [ \"\$a\" = \"0-3\" ] && taskset -p 7 \$(basename \$t) >/dev/null 2>&1; done; echo pinned'"
  # NOTE (measured 2026-07-17): renicing the s3d-tile* workers to -19 to cut their
  # wake-latency spike REGRESSED fps (54 -> 47) — the boosted workers preempt the 2D
  # compositor + GL present thread, so runFrame stays fast (min 15.3ms) but wall-clock
  # jitters up (+3ms) and stutters. The 4-thread/3-core render side is priority-balanced;
  # do NOT boost one stage. Left disabled. The wake-latency barrier needs a structural fix
  # (fewer render threads, or the emu absorbing render-prep), not a scheduling knob.
else
  echo "app not running — rerun this script after launching the app+race"
fi
echo "done. Launch the app AFTER this script (props are read at init)."
