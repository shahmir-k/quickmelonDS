#!/usr/bin/env bash
# Discovery at fast-forward, then a LAN session at 1x. P0 hosts MKDS at 1x; once it beacons, P1 starts with its own
# fast-forward (LITEV_AMBIENT_SOLO_FF, default 10x), scans, lists P0 (replayed beacons at the 1x rate per emulated
# second) and joins: the first link ends P1's fast-forward, both run the session at 1x. Same scripts as
# test-ambient-mkds-four.sh (p0, p1). No output path is ever deleted.
set -euo pipefail

if [[ $# -ne 3 && $# -ne 4 ]]; then
    echo "usage: $0 HEADLESS_BINARY MKDS_ROM INPUT_SCRIPT_DIR [FRESH_OUTPUT_DIR]" >&2
    exit 2
fi
binary="$(cd -- "$(dirname -- "$1")" && pwd -P)/$(basename -- "$1")"
rom="$(cd -- "$(dirname -- "$2")" && pwd -P)/$(basename -- "$2")"
scripts="$(cd -- "$3" && pwd -P)"
ff=${LITEV_AMBIENT_SOLO_FF:-10}
frames=2400
if [[ $# -eq 4 ]]; then
    output=$4
    if [[ -e $output ]]; then echo "output path must not already exist: $output" >&2; exit 2; fi
    mkdir -m 700 -- "$output"
else
    output=$(mktemp -d "${TMPDIR:-/tmp}/ambient-ff-scan.XXXXXX")
fi
echo "ambient ff-scan output: $output (P1 solo ff $ff)"

run() {   # seat, extra env
    mkdir -m 700 -- "$output/p$1"
    (cd -- "$output/p$1" && env LITEV_AMBIENT_ID="$(printf '00000000e1e1%04x' "$1")" $2 "$binary" --rom "$rom" \
        --input-script "$scripts/p$1.txt" --frames "$frames" --data-dir hd --ambient "P$1" > log.txt 2>&1)
}
run 0 "LITEV_AMBIENT_SOLO_FF=1" &
p0=$!
# P0 beacons from about frame 1400 (its wireless menu); P1 then reaches its scan + join in a few seconds
until grep -q '^AMBIENT f1440:' "$output/p0/log.txt" 2>/dev/null; do
    kill -0 $p0 2>/dev/null || { echo "P0 exited early" >&2; exit 1; }
    sleep 1
done
run 1 "LITEV_AMBIENT_SOLO_FF=$ff" &
p1=$!
fail=0
wait $p0 || { echo "P0 failed" >&2; fail=1; }
wait $p1 || { echo "P1 failed" >&2; fail=1; }
[[ $fail -eq 0 ]] || exit 1

for seat in 0 1; do
    log="$output/p$seat/log.txt"
    grep -Eq '^frames:[[:space:]]+'$frames'$' "$log" || { echo "seat $seat did not complete $frames frames" >&2; exit 1; }
    last=$(grep '^AMBIENT f' "$log" | tail -n 1)
    [[ $last == *"links=1 "* ]] || { echo "seat $seat: no LAN session at the end: $last" >&2; exit 1; }
    echo "seat $seat: $(grep '^avg_fps:' "$log") | $last"
done
log1="$output/p1/log.txt"
grep -q '^AMBIENT solo ff off' "$log1" || { echo "P1's fast-forward did not end at the first link" >&2; exit 1; }
echo "P1: $(grep '^AMBIENT solo ff off' "$log1")"
# replayed beacons per emulated second while P1 scanned (f1200..f1560: P0 beaconing, no link yet)
b() { grep "^AMBIENT f$1:" "$log1" | grep -o 'beacons=[0-9]*' | cut -d= -f2; }
rate=$(( ($(b 1560) - $(b 1200)) * 60 / 360 ))
echo "P1 beacons injected per emulated second while scanning at ${ff}x: $rate"
[[ $rate -ge 2 ]] || { echo "too few beacons for a scan at fast-forward" >&2; exit 1; }
echo "ambient-ff-scan: PASS"
