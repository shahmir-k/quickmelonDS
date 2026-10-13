#!/usr/bin/env bash
# Mac-only two-process Ambient -> provisional Netplay -> Ambient rollback gate.
# The trial deliberately rolls back; it is not a production commit test.
set -euo pipefail

if [[ $# -ne 10 ]]; then
    echo "usage: $0 HEADLESS ROM_HOST ROM_GUEST STATE_HOST STATE_GUEST SAVE_HOST SAVE_GUEST INPUT_HOST INPUT_GUEST OUT_DIR" >&2
    exit 2
fi
exe=$1; rom_host=$2; rom_guest=$3; state_host=$4; state_guest=$5
save_host=$6; save_guest=$7; input_host=$8; input_guest=$9; out_dir=${10}
: "${LITEV_HANDOFF_TEST_IP:?set the Mac LAN IPv4 address, not loopback}"
presence=${LITEV_HANDOFF_TEST_PRESENCE_PORT:-7620}
transfer=${LITEV_HANDOFF_TEST_TRANSFER_PORT:-19090}
netplay=${LITEV_HANDOFF_TEST_NETPLAY_PORT:-19092}
cut_frame=${LITEV_HANDOFF_TEST_CUT_FRAME:-3100}
frames=${LITEV_HANDOFF_TEST_FRAMES:-3250}
trial_frames=${LITEV_HANDOFF_TEST_TRIAL_FRAMES:-60}
timeout_s=${LITEV_HANDOFF_TEST_TIMEOUT_S:-120}
expect=${LITEV_HANDOFF_EXPECT:-pass}
retry=; [[ $expect == retry ]] && retry=1
stale=${LITEV_HANDOFF_TEST_STALE_REPLAY:-}
slow_reset=${LITEV_HANDOFF_TEST_SLOW_RESET_ROLE:-}
fault=${LITEV_HANDOFF_TEST_FAULT:-}
postfence=${LITEV_HANDOFF_TEST_POSTFENCE_SEND:-}

for input in "$exe" "$rom_host" "$rom_guest" "$state_host" "$state_guest" "$save_host" "$save_guest" "$input_host" "$input_guest"; do
    if [[ ! -f "$input" ]]; then echo "missing input: $input" >&2; exit 2; fi
done
if [[ -e "$out_dir" ]]; then echo "output already exists: $out_dir" >&2; exit 2; fi
mkdir -p "$out_dir/p0/hd" "$out_dir/p1/hd" "$out_dir/tx0" "$out_dir/tx1"
cp "$save_host" "$out_dir/p0/hd/headless.sav"
cp "$save_guest" "$out_dir/p1/hd/headless.sav"
save_paths=("$save_host" "$save_guest" "$out_dir/p0/hd/headless.sav" "$out_dir/p1/hd/headless.sav")
hash_files() {
    if command -v shasum >/dev/null 2>&1; then shasum -a 256 "$@"; else sha256sum "$@"; fi
}
run_bounded() { perl -e 'alarm shift; exec @ARGV' "$timeout_s" "$@"; }
hash_files "${save_paths[@]}" > "$out_dir/owner-before.sha256"
printf 'IP=%s\nPRESENCE=%s\nTRANSFER=%s\nNETPLAY=%s\nCUT_FRAME=%s\nTRIAL_FRAMES=%s\nEXPECT=%s\nFAULT=%s\nPOSTFENCE=%s\n' \
    "$LITEV_HANDOFF_TEST_IP" "$presence" "$transfer" "$netplay" "$cut_frame" "$trial_frames" "$expect" "$fault" "$postfence" > "$out_dir/config.txt"

(
    run_bounded env LITEV_AMBIENT_ID=a17e0002 LITEV_AMBIENT_PPORT="$presence" \
        LITEV_AMBIENT_UPORT="$((presence + 2))" LITEV_AMBIENT_PEERS="$LITEV_HANDOFF_TEST_IP:$((presence + 1))" \
        LITEV_AMBIENT_CUT_DIR="$out_dir" LITEV_AMBIENT_CUT_ROLE=client \
        LITEV_AMBIENT_CUT_MIN_FRAME="$cut_frame" LITEV_AMBIENT_CUT_XFER_PORT="$transfer" \
        LITEV_AMBIENT_CUT_PEER_ROM="$rom_host" LITEV_AMBIENT_CUT_SWAP_TRIAL_FRAMES="$trial_frames" \
        LITEV_HANDOFF_PROBE_FAIL_PHASE="$fault" LITEV_AMBIENT_CUT_POSTFENCE_SEND="$postfence" \
        LITEV_AMBIENT_CUT_RETRY_AFTER_ABORT="$retry" \
        LITEV_AMBIENT_CUT_STALE_REPLAY="$stale" \
        LITEV_AMBIENT_CUT_DELAY_RESET_ROLE="$slow_reset" \
        LITEV_NETPLAY="player=1,delay=3,port=$((netplay + 1)),peer=127.0.0.1:$netplay" \
        LITEV_MP_TXLOG="$out_dir/tx1" "$exe" --rom "$rom_guest" --rom1 "$rom_host" \
        --savestate "$state_guest" --input-script "$input_guest" --frames "$frames" \
        --ambient White --data-dir "$out_dir/p1/hd" > "$out_dir/p1/log.txt" 2>&1
) &
client_pid=$!
(
    run_bounded env LITEV_AMBIENT_ID=b1ac0001 LITEV_AMBIENT_PPORT="$presence" \
        LITEV_AMBIENT_UPORT="$((presence + 1))" LITEV_AMBIENT_PEERS="$LITEV_HANDOFF_TEST_IP:$((presence + 2))" \
        LITEV_AMBIENT_CUT_DIR="$out_dir" LITEV_AMBIENT_CUT_ROLE=host \
        LITEV_AMBIENT_CUT_MIN_FRAME="$cut_frame" LITEV_AMBIENT_CUT_XFER_PORT="$transfer" \
        LITEV_AMBIENT_CUT_PEER_ROM="$rom_guest" LITEV_AMBIENT_CUT_SWAP_TRIAL_FRAMES="$trial_frames" \
        LITEV_HANDOFF_PROBE_FAIL_PHASE="$fault" LITEV_AMBIENT_CUT_POSTFENCE_SEND="$postfence" \
        LITEV_AMBIENT_CUT_RETRY_AFTER_ABORT="$retry" \
        LITEV_AMBIENT_CUT_STALE_REPLAY="$stale" \
        LITEV_AMBIENT_CUT_DELAY_RESET_ROLE="$slow_reset" \
        LITEV_NETPLAY="player=0,delay=3,port=$netplay,peer=127.0.0.1:$((netplay + 1))" \
        LITEV_MP_TXLOG="$out_dir/tx0" "$exe" --rom "$rom_host" --rom1 "$rom_guest" \
        --savestate "$state_host" --input-script "$input_host" --frames "$frames" \
        --ambient Black --data-dir "$out_dir/p0/hd" > "$out_dir/p0/log.txt" 2>&1
) &
host_pid=$!
set +e
wait "$client_pid"; client_status=$?
wait "$host_pid"; host_status=$?
set -e
printf 'CLIENT_EXIT=%s\nHOST_EXIT=%s\n' "$client_status" "$host_status" > "$out_dir/result.txt"
hash_files "${save_paths[@]}" > "$out_dir/owner-after.sha256"

if [[ $client_status -ne 0 || $host_status -ne 0 ]] ||
   ! cmp -s "$out_dir/owner-before.sha256" "$out_dir/owner-after.sha256" ||
   ! grep -Eq 'AMBIENT f3240:.*links=1.*exch=[1-9]' "$out_dir/p0/log.txt" ||
   ! grep -Eq 'AMBIENT f3240:.*links=1.*exch=[1-9]' "$out_dir/p1/log.txt"; then
    echo "FAIL: source rollback/save gate ($out_dir)" >&2; exit 1
fi
if [[ $expect == pass || $expect == retry ]]; then
    for process in p0 p1; do
        log="$out_dir/$process/log.txt"
        for phase in prepared armed go; do
            if ! grep -q " $phase agreed" "$log"; then echo "FAIL: $process missing $phase ($out_dir)" >&2; exit 1; fi
        done
        if ! grep -q 'live-start rendezvous agreed' "$log" ||
           ! grep -q 'swap trial pass sourceIntact=1 link=1' "$log" ||
           ! grep -q 'ran_clean:       yes' "$log"; then
            echo "FAIL: $process provisional Netplay/rollback ($out_dir)" >&2; exit 1
        fi
    done
    if ! cmp -s "$out_dir/tx0/tx0.txt" "$out_dir/tx1/tx0.txt" ||
       ! cmp -s "$out_dir/tx0/tx1.txt" "$out_dir/tx1/tx1.txt"; then
        echo "FAIL: cross-process TX differs ($out_dir)" >&2; exit 1
    fi
    if [[ $expect == retry ]]; then
        role_dir=p0; [[ ${fault%%:*} == client ]] && role_dir=p1
        if ! grep -q "injected ${fault#*:} failure" "$out_dir/$role_dir/log.txt"; then
            echo "FAIL: requested first-attempt fault did not execute ($out_dir)" >&2; exit 1
        fi
        for process in p0 p1; do
            log="$out_dir/$process/log.txt"
            if [[ $(grep -c 'CUTFENCE .*attempt=' "$log") -ne 2 ]] ||
               ! grep -q 'reset after mutual abort' "$log"; then
                echo "FAIL: fresh retry was not reached after a mutual abort ($out_dir)" >&2; exit 1
            fi
        done
        if [[ -n $stale ]] &&
           { ! grep -q 'enqueued stale closed-epoch FENCE ok=1' "$out_dir/p0/log.txt" ||
             ! grep -q 'ignored stale closed epoch' "$out_dir/p1/log.txt"; }; then
            echo "FAIL: stale old-epoch control was not enqueued and ignored ($out_dir)" >&2; exit 1
        fi
        if [[ -n $slow_reset ]]; then
            slow_dir=p0; [[ $slow_reset == client ]] && slow_dir=p1
            if ! grep -q "CUTFENCE $slow_reset delaying reset 250ms" "$out_dir/$slow_dir/log.txt"; then
                echo "FAIL: asymmetric reset delay was not exercised ($out_dir)" >&2; exit 1
            fi
        fi
    fi
elif [[ $expect == abort ]]; then
    if [[ -n $fault ]]; then
        role_dir=p0; [[ ${fault%%:*} == client ]] && role_dir=p1
        if ! grep -q "injected ${fault#*:} failure" "$out_dir/$role_dir/log.txt"; then
            echo "FAIL: requested fault did not execute ($out_dir)" >&2; exit 1
        fi
        for process in p0 p1; do
            if ! grep -q 'reset after mutual abort' "$out_dir/$process/log.txt"; then
                echo "FAIL: $process did not clear the fence after abort ($out_dir)" >&2; exit 1
            fi
        done
    fi
    if [[ -n $postfence ]] && ! grep -q 'injected post-fence MP send; aborted=1' "$out_dir/p0/log.txt" "$out_dir/p1/log.txt"; then
        echo "FAIL: post-fence send did not execute ($out_dir)" >&2; exit 1
    fi
    if [[ $fault == host:afterlive ]]; then
        if ! grep -q 'zero Netplay frames run' "$out_dir/p1/log.txt" ||
           grep -q 'handoff startup peer input confirmed' "$out_dir/p0/log.txt" "$out_dir/p1/log.txt" ||
           grep -q 'swap trial pass' "$out_dir/p0/log.txt" "$out_dir/p1/log.txt"; then
            echo "FAIL: a replica advanced after asymmetric live-start response ($out_dir)" >&2; exit 1
        fi
    elif grep -q 'swap trial begin' "$out_dir/p0/log.txt" "$out_dir/p1/log.txt"; then
        echo "FAIL: trial ran despite expected abort ($out_dir)" >&2; exit 1
    fi
else
    echo "bad LITEV_HANDOFF_EXPECT: $expect" >&2; exit 2
fi
echo "PASS: Ambient cut $expect, source rollback and saves intact ($out_dir)"
