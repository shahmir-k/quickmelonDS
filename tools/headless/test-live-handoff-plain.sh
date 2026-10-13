#!/usr/bin/env bash
# Mac-only plain Netplay continuation gate from two already-fenced live checkpoints.
# Source save paths are read-only inputs; each process gets separate scratch copies.
set -euo pipefail

if [[ $# -ne 10 ]]; then
    echo "usage: $0 HEADLESS ROM_HOST ROM_GUEST STATE_HOST STATE_GUEST FW_HOST FW_GUEST SAVE_HOST SAVE_GUEST OUT_DIR" >&2
    exit 2
fi
exe=$1; rom_host=$2; rom_guest=$3; state_host=$4; state_guest=$5
fw_host=$6; fw_guest=$7; save_host=$8; save_guest=$9; out_dir=${10}
port=${LITEV_HANDOFF_PLAIN_TEST_PORT:-19080}
frames=${LITEV_HANDOFF_PLAIN_TEST_FRAMES:-600}
timeout_s=${LITEV_HANDOFF_PLAIN_TEST_TIMEOUT_S:-90}
for input in "$exe" "$rom_host" "$rom_guest" "$state_host" "$state_guest" "$fw_host" "$fw_guest" "$save_host" "$save_guest"; do
    if [[ ! -f "$input" ]]; then echo "missing input: $input" >&2; exit 2; fi
done
if [[ -e "$out_dir" ]]; then echo "output already exists: $out_dir" >&2; exit 2; fi
mkdir -p "$out_dir/p0/hd" "$out_dir/p1/hd" "$out_dir/tx0" "$out_dir/tx1"
for process in p0 p1; do
    cp "$save_host" "$out_dir/$process/hd/mp0.sav"
    cp "$save_guest" "$out_dir/$process/hd/mp1.sav"
done
hash_files() {
    if command -v shasum >/dev/null 2>&1; then shasum -a 256 "$@"; else sha256sum "$@"; fi
}
save_paths=("$save_host" "$save_guest" "$out_dir/p0/hd/mp0.sav" "$out_dir/p0/hd/mp1.sav" "$out_dir/p1/hd/mp0.sav" "$out_dir/p1/hd/mp1.sav")
hash_files "${save_paths[@]}" > "$out_dir/owner-before.sha256"
run_bounded() { perl -e 'alarm shift; exec @ARGV' "$timeout_s" "$@"; }

for player in 0 1; do
    (
        run_bounded env LITEV_NETPLAY="player=$player,delay=3,port=$((port + player)),peer=127.0.0.1:$((port + 1 - player))" \
            LITEV_MP_SS0="$state_host" LITEV_MP_SS1="$state_guest" \
            LITEV_MP_FW0="$fw_host" LITEV_MP_FW1="$fw_guest" \
            LITEV_MP_HANDOFF_PROBE=1 LITEV_MP_TXLOG="$out_dir/tx$player" \
            "$exe" --rom "$rom_host" --rom1 "$rom_guest" --mp-test \
            --frames "$frames" --data-dir "$out_dir/p$player/hd" > "$out_dir/p$player/log.txt" 2>&1
    ) &
    if [[ $player -eq 0 ]]; then p0_pid=$!; else p1_pid=$!; fi
done
set +e
wait "$p0_pid"; p0_status=$?
wait "$p1_pid"; p1_status=$?
set -e
printf 'P0_EXIT=%s\nP1_EXIT=%s\n' "$p0_status" "$p1_status" > "$out_dir/result.txt"
hash_files "${save_paths[@]}" > "$out_dir/owner-after.sha256"

if [[ $p0_status -ne 0 || $p1_status -ne 0 ]] ||
   ! cmp -s "$out_dir/owner-before.sha256" "$out_dir/owner-after.sha256" ||
   ! cmp -s "$out_dir/tx0/tx0.txt" "$out_dir/tx1/tx0.txt" ||
   ! cmp -s "$out_dir/tx0/tx1.txt" "$out_dir/tx1/tx1.txt"; then
    echo "FAIL: plain live-checkpoint continuation (logs in $out_dir)" >&2
    exit 1
fi
for player in 0 1; do
    log="$out_dir/p$player/log.txt"
    if ! grep -q 'ran_clean:       yes' "$log" ||
       ! grep -q "reply_frames:    $((frames * 2))" "$log" ||
       [[ $(grep -c "frame${frames}: wifiOn=1 mp=1" "$log") -ne 2 ]]; then
        echo "FAIL: player $player lost active Wi-Fi or replies (logs in $out_dir)" >&2
        exit 1
    fi
done
echo "PASS: plain $frames frames, both Wi-Fi links active, TX byte-identical, saves unchanged ($out_dir)"
