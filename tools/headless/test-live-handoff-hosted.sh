#!/usr/bin/env bash
# Mac-only HostedNetplay continuation gate from two already-fenced live DS checkpoints.
# This does not perform the Ambient cut or commit; those have separate protocol gates.
set -euo pipefail

if [[ $# -ne 10 ]]; then
    echo "usage: $0 HEADLESS ROM_HOST ROM_GUEST STATE_HOST STATE_GUEST FW_HOST FW_GUEST SAVE_HOST SAVE_GUEST OUT_DIR" >&2
    exit 2
fi

exe=$1
rom_host=$2
rom_guest=$3
state_host=$4
state_guest=$5
fw_host=$6
fw_guest=$7
save_host=$8
save_guest=$9
out_dir=${10}
port=${LITEV_HANDOFF_HOSTED_TEST_PORT:-19072}
frames=${LITEV_HANDOFF_HOSTED_TEST_FRAMES:-600}
timeout_s=${LITEV_HANDOFF_HOSTED_TEST_TIMEOUT_S:-90}

for input in "$exe" "$rom_host" "$rom_guest" "$state_host" "$state_guest" "$fw_host" "$fw_guest" "$save_host" "$save_guest"; do
    if [[ ! -f "$input" ]]; then echo "missing input: $input" >&2; exit 2; fi
done
if [[ -e "$out_dir" ]]; then echo "output already exists: $out_dir" >&2; exit 2; fi
mkdir -p "$out_dir/server/hd" "$out_dir/guest/hd"
cp "$save_host" "$out_dir/server/hd/mp0.sav"
cp "$save_guest" "$out_dir/server/hd/mp1.sav"
cp "$save_guest" "$out_dir/guest/hd/mp1.sav"

hash_files() {
    if command -v shasum >/dev/null 2>&1; then shasum -a 256 "$@"; else sha256sum "$@"; fi
}
save_paths=("$save_host" "$save_guest" "$out_dir/server/hd/mp0.sav" "$out_dir/server/hd/mp1.sav" "$out_dir/guest/hd/mp1.sav")
hash_files "${save_paths[@]}" > "$out_dir/owner-before.sha256"
run_bounded() { perl -e 'alarm shift; exec @ARGV' "$timeout_s" "$@"; }

(
    run_bounded env LITEV_HOSTED="host=127.0.0.1:$port,port=$((port + 1))" \
        LITEV_MP_SS1="$state_guest" LITEV_MP_FW1="$fw_guest" \
        "$exe" --rom "$rom_guest" --rom1 "$rom_guest" --replay-console 1 \
        --frames "$frames" --data-dir "$out_dir/guest/hd" > "$out_dir/guest/log.txt" 2>&1
) &
guest_pid=$!
(
    run_bounded env LITEV_HOSTED="players=2,port=$port,play,delay=3" \
        LITEV_MP_SS0="$state_host" LITEV_MP_SS1="$state_guest" \
        LITEV_MP_FW0="$fw_host" LITEV_MP_FW1="$fw_guest" LITEV_MP_HANDOFF_PROBE=1 \
        "$exe" --rom "$rom_host" --rom1 "$rom_guest" --mp-test \
        --frames "$frames" --data-dir "$out_dir/server/hd" > "$out_dir/server/log.txt" 2>&1
) &
server_pid=$!
set +e
wait "$guest_pid"; guest_status=$?
wait "$server_pid"; server_status=$?
set -e
printf 'GUEST_EXIT=%s\nSERVER_EXIT=%s\n' "$guest_status" "$server_status" > "$out_dir/result.txt"
hash_files "${save_paths[@]}" > "$out_dir/owner-after.sha256"

if [[ $guest_status -ne 0 || $server_status -ne 0 ]] ||
   ! cmp -s "$out_dir/owner-before.sha256" "$out_dir/owner-after.sha256" ||
   ! grep -q 'replay: MATCH (every frame)' "$out_dir/guest/log.txt" ||
   ! grep -q 'ran_clean:       yes' "$out_dir/server/log.txt" ||
   ! grep -q "reply_frames:    $((frames * 2))" "$out_dir/server/log.txt" ||
   [[ $(grep -c "frame${frames}: wifiOn=1 mp=1" "$out_dir/server/log.txt") -ne 2 ]]; then
    echo "FAIL: Hosted live-checkpoint continuation (logs in $out_dir)" >&2
    exit 1
fi
echo "PASS: Hosted $frames frames, both Wi-Fi links active, replay exact, saves unchanged ($out_dir)"
