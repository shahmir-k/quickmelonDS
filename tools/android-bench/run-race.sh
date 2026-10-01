#!/usr/bin/env bash
# In-race device benchmark over adb (RG DS / Cortex-A55), savestate-based.
# Matrix (in-race savestate window 60:960, medians of 3):
#   baseline,full  x  fastmem{on,off} x frameskip{0,9}
#   dispatch,link,es x fastmem off    x frameskip{0,9}
# Interleaved by rep, 30s thermal gaps, SoC temp+A55 clock sampled per run.
# Coordination: waits while the co-tenant app emulator (me.magnum.melonds*) is in
# R (actively computing) state or a foreign liteDS run is live, so we never
# contend for CPU. Only touches /data/local/tmp/liteds/.
D=/data/local/tmp/liteds
ROM=shrek.nds; SS=shrek-race.mln; HA=shrek-race-hold-a.script
FRAMES=961; WIN=60:960
OUT=/tmp/dev-race960.tsv
echo -e "config\tfastmem\tframeskip\trep\twindow_fps\tsoc_mC_pre\tcpu_khz_pre\tsoc_mC_post" > "$OUT"

soc(){ adb shell 'cat /sys/class/thermal/thermal_zone0/temp' 2>/dev/null | tr -d '\r'; }
freq(){ adb shell 'cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq' 2>/dev/null | tr -d '\r'; }
app_busy(){ adb shell "ps -A" 2>/dev/null | tr -d '\r' | grep -iE 'melonds|emu' | grep -v systemui | awk '{print $8}' | grep -q '^R' && echo 1 || echo 0; }
wait_clear(){ while [ "$(app_busy)" = "1" ]; do echo "  [wait] co-tenant app active, sleeping 30s"; sleep 30; done; }

# cell list: config:fastmem:frameskip
CELLS=( \
  baseline:on:0 baseline:off:0 baseline:on:9 baseline:off:9 \
  dispatch:off:0 dispatch:off:9 \
  link:off:0 link:off:9 \
  es:off:0 es:off:9 \
  full:on:0 full:off:0 full:on:9 full:off:9 )

first=1
for rep in 1 2 3; do
  for cell in "${CELLS[@]}"; do
    IFS=: read -r cfg fm fs <<< "$cell"
    [ "$first" = 1 ] || sleep 30
    first=0
    wait_clear
    tpre=$(soc); fpre=$(freq)
    fps=$(adb shell "cd $D && ./liteDS-$cfg --rom $ROM --savestate $SS --input-script $HA --frames $FRAMES --bench-window $WIN --mode jit --fastmem $fm --frameskip $fs --data-dir hdata 2>/dev/null" | tr -d '\r' | awk '/^window_fps:/{print $2}')
    tpost=$(soc)
    echo -e "$cfg\t$fm\t$fs\t$rep\t${fps:-ERR}\t${tpre:-?}\t${fpre:-?}\t${tpost:-?}" >> "$OUT"
  done
done
echo "DONE" >> "$OUT"
