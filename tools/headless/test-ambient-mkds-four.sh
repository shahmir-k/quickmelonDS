#!/usr/bin/env bash
# Four-process Just Play regression. Requires an MKDS ROM and the p0..p3 input
# scripts that navigate into the same race. No output path is ever deleted.
set -euo pipefail

if [[ $# -ne 3 && $# -ne 4 ]]; then
    echo "usage: $0 HEADLESS_BINARY MKDS_ROM INPUT_SCRIPT_DIR [FRESH_OUTPUT_DIR]" >&2
    exit 2
fi

binary=$1
rom=$2
scripts=$3
frames=3600
if [[ ! -x $binary || ! -f $rom ]]; then
    echo "headless binary must be executable and ROM must exist" >&2
    exit 2
fi
binary="$(cd -- "$(dirname -- "$binary")" && pwd -P)/$(basename -- "$binary")"
rom="$(cd -- "$(dirname -- "$rom")" && pwd -P)/$(basename -- "$rom")"
scripts="$(cd -- "$scripts" && pwd -P)"
for seat in 0 1 2 3; do
    if [[ ! -f $scripts/p$seat.txt ]]; then
        echo "missing input script: $scripts/p$seat.txt" >&2
        exit 2
    fi
done

if [[ $# -eq 4 ]]; then
    output=$4
    if [[ -e $output ]]; then
        echo "output path must not already exist: $output" >&2
        exit 2
    fi
    mkdir -m 700 -- "$output"
else
    output=$(mktemp -d "${TMPDIR:-/tmp}/ambient-mkds-four.XXXXXX")
fi
echo "ambient regression output: $output"

pids=()
for seat in 0 1 2 3; do
    mkdir -m 700 -- "$output/p$seat"
    identity=$(printf '00000000e1e0%04x' "$seat")
    (
        cd -- "$output/p$seat"
        env LITEV_AMBIENT_ID="$identity" "$binary" \
            --rom "$rom" --input-script "$scripts/p$seat.txt" \
            --frames "$frames" --data-dir hd --ambient "P$seat" > log.txt 2>&1
    ) &
    pids+=("$!")
    if [[ $seat -eq 0 ]]; then sleep 1; fi
done

failed=0
for seat in 0 1 2 3; do
    if ! wait "${pids[$seat]}"; then
        echo "seat $seat failed; see $output/p$seat/log.txt" >&2
        failed=1
    fi
done
if [[ $failed -ne 0 ]]; then exit 1; fi

for seat in 0 1 2 3; do
    log="$output/p$seat/log.txt"
    if ! grep -Eq '^frames:[[:space:]]+3600$' "$log"; then
        echo "seat $seat did not complete 3600 frames: $log" >&2
        exit 1
    fi
    last=$(grep '^AMBIENT f' "$log" | tail -n 1)
    expected_links=1
    if [[ $seat -eq 0 ]]; then expected_links=3; fi
    if [[ $last != *"links=$expected_links "* ]]; then
        echo "seat $seat has wrong final link count: $last" >&2
        exit 1
    fi
    echo "seat $seat: $(grep '^avg_fps:' "$log") | $last"
done
for seat in 1 2 3; do
    if ! grep -Fq -- "00000000e1e0000$seat (P$seat)" "$output/p0/log.txt"; then
        echo "host did not link seat $seat" >&2
        exit 1
    fi
    if ! grep -Fq -- '00000000e1e00000 (P0)' "$output/p$seat/log.txt"; then
        echo "seat $seat did not link host" >&2
        exit 1
    fi
done
echo 'ambient-mkds-four: PASS'
