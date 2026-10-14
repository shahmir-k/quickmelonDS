#!/usr/bin/env python3
"""Mac-only, evidence-gated Pokémon Black/White Just Play latency sweep.

A *test* of a slower network for AmbientLAN (NetSim, AmbientLAN.cpp): both processes hold
their received MP frames and presence pings/pongs for half the round trip (+ jitter), so the
MP exchange and the measured round trip see the whole RTT. ENet's setup, acks and control
channel are not held. LITEV_FRAME_LOAD_MS makes the Mac's frame cost a device's.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time


SCENARIOS = {
    "trade": {
        "black": ["shimmer is in the room", "menu open", "trade requested", "trade screen", "offered", "returned to trade selection"],
        "white": ["in the room", "offer", "trade screen", "offered", "returned to trade selection"],
        "frames": 18000,
    },
    "battle": {
        "black": ["shimmer is in the room", "menu open", "battle requested", "in the battle room", "on the marker", "selection screen", "lead entered", "back in the battle room"],
        "white": ["in the room", "offer", "in the battle room", "on the marker", "selection screen", "lead entered", "back in the battle room"],
        "frames": 11000,
    },
}
SCRIPT_DIR = Path(__file__).resolve().parent / "scenarios" / "pokemon-bw"
MILESTONE = re.compile(r"^MILESTONE frame=(\d+) name=([^ ]+) top=([0-9a-f]+) bottom=([0-9a-f]+) file=(.+)$", re.M)
DIAG = re.compile(r"^AMBIENT DIAG f(\d+): (.+)$", re.M)
EVENT = re.compile(r"^AMBIENT event t=([0-9.]+)s f(\d+): links=(\d+) offer=(\d) link_rtt=([0-9.]+)ms", re.M)
VERDICT = re.compile(r"^AMBIENT verdict t=([0-9.]+)s f(\d+): (\w+)", re.M)
STAT = re.compile(r"^AMBIENT f(\d+): .*?links=(\d+) .*?exch=(\d+) .*?host_wait=([0-9.]+)ms/(\d+) miss=\d+ timeout=(\d+) .*?stale=([0-9/]+)", re.M)
FPS = re.compile(r"^avg_fps:\s+([0-9.]+)", re.M)


def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def ppm_evidence(path):
    with path.open("rb") as f:
        if f.readline() != b"P6\n" or f.readline() != b"256 384\n" or f.readline() != b"255\n":
            raise ValueError(f"bad framebuffer dimensions/format: {path}")
        pixels = f.read()
    if len(pixels) != 256 * 384 * 3 or len(set(pixels[::97])) < 4:
        raise ValueError(f"empty/truncated framebuffer: {path}")
    return {"sha256": sha(path), "bytes": path.stat().st_size}


def trade_icon_scores(path):
    """Mean R-B in the two trainer-board icon boxes of these fixed BW savestates.

    The Black trainer initially owns a blue icon and White an orange one. A
    completed trade swaps them. This is a *visual oracle for these fixtures*,
    not a general Pokémon save parser. A changed UI or sprite invalidates the
    case (unknown), never silently turns it into a pass.
    """
    with path.open("rb") as f:
        if f.readline() != b"P6\n" or f.readline() != b"256 384\n" or f.readline() != b"255\n":
            raise ValueError(f"bad PPM: {path}")
        pixels = f.read()
    if len(pixels) != 256 * 384 * 3:
        raise ValueError(f"truncated PPM: {path}")
    boxes = ((23, 92, 38, 104), (153, 92, 170, 104))
    scores = []
    for x0, y0, x1, y1 in boxes:
        delta = 0
        for y in range(y0, y1):
            for x in range(x0, x1):
                off = (y * 256 + x) * 3
                delta += pixels[off] - pixels[off + 2]
        scores.append(delta / ((x1 - x0) * (y1 - y0)))
    return scores


def trade_swapped(side, milestones):
    """Require original → opposite icon colors at the two exact phase captures."""
    try:
        before = trade_icon_scores(Path(milestones["offered"]["shot"]))
        after = trade_icon_scores(Path(milestones["returned_to_trade_selection"]["shot"]))
    except (KeyError, OSError, ValueError):
        return {"verified": False, "reason": "missing or invalid before/after frame"}
    # Thresholds are intentionally well outside grayscale (0): a sprite that
    # moves/changes cannot be misreported as a trade. Verify both consoles.
    blue, orange = -7, 25
    if side == "black":
        verified = before[0] < blue and before[1] > orange and after[0] > orange and after[1] < blue
    else:
        verified = before[0] > orange and before[1] < blue and after[0] < blue and after[1] > orange
    return {"verified": verified, "before_rb": before, "after_rb": after,
            "reason": "icon colors did not swap" if not verified else "both trainer icons swapped"}


def free_udp_ports(n):
    sockets = []
    try:
        for _ in range(n):
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind(("127.0.0.1", 0))
            sockets.append(s)
        return [s.getsockname()[1] for s in sockets]
    finally:
        for s in sockets:
            s.close()


def prepare_inputs(args, root):
    supplied = {
        "black_rom": Path(args.black_rom).resolve(strict=True),
        "white_rom": Path(args.white_rom).resolve(strict=True),
        "black_state": Path(args.black_state).resolve(strict=True),
        "white_state": Path(args.white_state).resolve(strict=True),
        "black_save": Path(args.black_save).resolve(strict=True),
        "white_save": Path(args.white_save).resolve(strict=True),
    }
    inputs = root / "inputs"
    inputs.mkdir()
    manifest = {}
    for key, src in supplied.items():
        if not src.is_file():
            raise ValueError(f"not a regular file: {src}")
        # copyfile follows a source symlink but never writes through it. No hardlinks.
        dst = inputs / (key + src.suffix)
        before = sha(src)
        shutil.copyfile(src, dst)
        if sha(dst) != before:
            raise RuntimeError(f"copy verification failed: {src}")
        dst.chmod(0o444)
        manifest[key] = {"source": str(src), "copy": str(dst), "sha256": before}
    for side in ("black", "white"):
        src = SCRIPT_DIR / f"{args.scenario}-{side}.script"
        if not src.is_file():
            raise FileNotFoundError(src)
        dst = inputs / src.name
        shutil.copyfile(src, dst)
        dst.chmod(0o444)
        manifest[f"{side}_script"] = {"source": str(src), "copy": str(dst), "sha256": sha(src)}
    return manifest


def verify_inputs(inputs):
    for key, item in inputs.items():
        if sha(Path(item["source"])) != item["sha256"] or sha(Path(item["copy"])) != item["sha256"]:
            raise RuntimeError(f"input changed during sweep: {key}")


def run_one(args, root, inputs, delay, repeat):
    name = f"{args.scenario}-rtt{delay:g}-r{repeat}"
    case = root / name
    case.mkdir()
    ports = free_udp_ports(4)
    procs = []
    logs = []
    spec = SCENARIOS[args.scenario]
    started = time.monotonic()
    try:
        for i, side in enumerate(("black", "white")):
            d = case / side
            d.mkdir()
            (d / "shots").mkdir()
            (d / "data").mkdir()
            save = d / "input.sav"
            shutil.copyfile(inputs[f"{side}_save"]["copy"], save)
            env = os.environ.copy()
            env.update({
                "LITEV_HEADLESS_SAV": str(save),
                "LITEV_HEADLESS_SAVOUT": str(d / "output.sav"),
                "LITEV_AMBIENT_ID": f"{(0xB1AC0001 if i == 0 else 0xA17E0002) + repeat * 256:016x}",
                "LITEV_AMBIENT_UPORT": str(ports[i]),
                "LITEV_AMBIENT_PPORT": str(ports[i + 2]),
                "LITEV_AMBIENT_PEERS": f"127.0.0.1:{ports[1 - i]}",
                "LITEV_AMBIENT_RTT_MS": str(delay),
                "LITEV_AMBIENT_JITTER_MS": str(delay * args.jitter_pct / 100),
                "LITEV_FRAME_LOAD_MS": str(args.frame_load_ms),
                "LITEV_MPTIMEOUT": str(args.mp_timeout_ms),
                "LITEV_RECORD_EMIT": "1",
                "LITEV_RECORD_EVERY": "600",
            })
            command = [str(Path(args.headless).resolve()), "--rom", inputs[f"{side}_rom"]["copy"],
                       "--savestate", inputs[f"{side}_state"]["copy"],
                       "--frames", str(args.frames or spec["frames"]), "--ambient", side.title(),
                       "--input-script", inputs[f"{side}_script"]["copy"],
                       "--script-shot-dir", str(d / "shots"), "--data-dir", str(d / "data"),
                       "--fixed-rtc", "946684800", "--renderer", "soft"]
            log = (d / "log.txt").open("wb")
            logs.append(log)
            procs.append(subprocess.Popen(command, cwd=d, env=env, stdout=log, stderr=subprocess.STDOUT,
                                          start_new_session=True))
            if i == 0:
                time.sleep(1)
        deadline = started + args.timeout_sec
        while any(p.poll() is None for p in procs) and time.monotonic() < deadline:
            time.sleep(0.25)
        timed_out = any(p.poll() is None for p in procs)
        if timed_out:
            for p in procs:
                if p.poll() is None:
                    os.killpg(p.pid, signal.SIGTERM)
            time.sleep(1)
            for p in procs:
                if p.poll() is None:
                    os.killpg(p.pid, signal.SIGKILL)
        for p in procs:
            p.wait()
    finally:
        for log in logs:
            log.close()
        for p in procs:
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGTERM)
                p.wait()
    result = {"case": name, "rtt_ms": delay, "jitter_ms": delay * args.jitter_pct / 100,
              "frame_load_ms": args.frame_load_ms,
              "fault_model": "NetSim: MP frames + presence pings held RTT/2 +- jitter/2 on each side; ENet setup/acks not held",
              "duration_s": round(time.monotonic() - started, 3), "timeout": timed_out,
              "ports": ports, "sides": {}}
    for i, side in enumerate(("black", "white")):
        d = case / side
        log = (d / "log.txt").read_text(errors="replace")
        got = {}
        for m in MILESTONE.finditer(log):
            label = m.group(2)
            shot = Path(m.group(5))
            if label not in got and shot.is_file() and shot.parent == d / "shots":
                got[label] = {"frame": int(m.group(1)), "top_xxh3": m.group(3),
                              "bottom_xxh3": m.group(4), "shot": str(shot), **ppm_evidence(shot)}
        required = [re.sub(r"[^A-Za-z0-9]", "_", s) for s in spec[side]]
        ordered = [got[s]["frame"] for s in required if s in got]
        missing = [s for s in required if s not in got]
        distinct = len({got[s]["sha256"] for s in required if s in got})
        failed = ("TIMEOUT at step" in log or "Fatal signal" in log or "data abort" in log or
                  "prefetch abort" in log or procs[i].returncode != 0)
        valid = not failed and not missing and ordered == sorted(ordered) and distinct >= 3
        exchange = trade_swapped(side, got) if args.scenario == "trade" else None
        if exchange is not None:
            valid = valid and exchange["verified"]
        fps = FPS.search(log)
        events = [(float(t), int(f), int(l), int(o)) for t, f, l, o, _ in EVENT.findall(log)]
        linked = [e for e in events if e[2] > 0]
        t_link = linked[0][0] if linked else None
        dropped = [e for e in events if t_link is not None and e[0] > t_link and e[2] == 0]
        offer = [e for e in events if e[3] == 1]
        verdicts = VERDICT.findall(log)
        bad = [v for v in verdicts if v[2] == "LAN_BAD"]
        stats = STAT.findall(log)
        last = stats[-1] if stats else None
        frames_linked = None
        session = {"joined": t_link is not None,
                   "link_s": t_link,
                   "disconnect_after_s": round(dropped[0][0] - t_link, 2) if dropped else None,
                   "prompt_after_s": round(offer[0][0] - t_link, 2) if offer and t_link is not None else None,
                   "lan_bad_after_s": round(float(bad[0][0]) - t_link, 2) if bad and t_link is not None else None,
                   "verdicts": verdicts,
                   "exchanges": int(last[2]) if last else 0,
                   "host_waits": int(last[4]) if last else 0, "host_timeouts": int(last[5]) if last else 0,
                   "stale": last[6] if last else None}
        result["sides"][side] = {"session": session,"returncode": procs[i].returncode, "script_failed": failed,
                                  "valid": valid, "missing": missing, "phase_frame_order_ok": ordered == sorted(ordered),
                                  "distinct_phase_images": distinct, "avg_fps": float(fps.group(1)) if fps else None,
                                  "exchange": exchange,
                                  "diagnostics": DIAG.findall(log), "milestones": got,
                                  "log": str(d / "log.txt"), "save_copy_unchanged": sha(d / "input.sav") == inputs[f"{side}_save"]["sha256"]}
    result["valid"] = all(x["valid"] and x["save_copy_unchanged"] for x in result["sides"].values()) and not timed_out
    (case / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--headless", required=True, type=Path, help="LITEV_AMBIENT_LAN-enabled Mac headless binary")
    for side in ("black", "white"):
        for kind in ("rom", "state", "save"):
            ap.add_argument(f"--{side}-{kind}", required=True, type=Path)
    ap.add_argument("--scenario", choices=SCENARIOS, required=True)
    ap.add_argument("--output", required=True, type=Path, help="directory for a new, retained evidence run")
    ap.add_argument("--delays", default="0,5,10,20,35,50,100,200",
                    help="round trips in ms (NetSim, both processes)")
    ap.add_argument("--jitter-pct", type=float, default=10, help="jitter as a percentage of the round trip")
    ap.add_argument("--frame-load-ms", type=float, default=8, help="LITEV_FRAME_LOAD_MS: a device's frame cost")
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--frames", type=int, default=0)
    ap.add_argument("--timeout-sec", type=int, default=600)
    ap.add_argument("--mp-timeout-ms", type=int, default=50)
    ap.add_argument("--keep-going", action="store_true", help="go on after a failed zero-RTT baseline")
    ap.add_argument("--prepare-only", action="store_true", help="copy/hash inputs without launching emulators")
    args = ap.parse_args()
    if sys.platform != "darwin":
        ap.error("this controlled sweep is Mac-only")
    if not args.headless.is_file() or not os.access(args.headless, os.X_OK):
        ap.error("--headless must name an executable binary")
    if args.repeats < 1 or args.timeout_sec < 1 or args.mp_timeout_ms < 1:
        ap.error("repeats, timeout, and MP timeout must be positive")
    delays = [float(s) for s in args.delays.split(",")]
    if not delays or any(not 0 <= d <= 1000 for d in delays):
        ap.error("delays must be 0..1000 ms")
    args.output.mkdir(parents=True, exist_ok=True)
    root = Path(tempfile.mkdtemp(prefix="pokemon-ambient-", dir=args.output.resolve()))
    inputs = prepare_inputs(args, root)
    manifest = {"scenario": args.scenario, "inputs": inputs, "delay_ms": delays,
                "repeats": args.repeats, "headless": str(args.headless.resolve()),
                "headless_sha256": sha(args.headless), "cases": []}
    (root / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    if args.prepare_only:
        print(root)
        return
    for delay in delays:
        for repeat in range(args.repeats):
            result = run_one(args, root, inputs, delay, repeat)
            manifest["cases"].append({"case": result["case"], "valid": result["valid"]})
            (root / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
            print(f"{result['case']}: {'PASS' if result['valid'] else 'INCOMPLETE/FAIL'}", flush=True)
            for side, x in result["sides"].items():
                ss = x["session"]
                print(f"  {side}: fps {x['avg_fps']} joined {ss['joined']} missing {x['missing']} "
                      f"disconnect {ss['disconnect_after_s']} prompt {ss['prompt_after_s']} lan_bad {ss['lan_bad_after_s']} "
                      f"exch {ss['exchanges']} host_waits {ss['host_waits']} timeouts {ss['host_timeouts']} stale {ss['stale']}", flush=True)
            # A failed zero-delay baseline makes every latency result uninterpretable.
            if delay == 0 and not result["valid"] and not args.keep_going:
                verify_inputs(inputs)
                print("Zero-delay baseline failed; stopping before latency claims.", file=sys.stderr)
                raise SystemExit(2)
    verify_inputs(inputs)
    print(root)


if __name__ == "__main__":
    main()
