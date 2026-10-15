#!/usr/bin/env python3
"""Audio continuity across a live handoff (enter: Ambient -> Netplay; leave: Netplay -> Ambient).

Reads a LITEV_AUDIO_TAP file (records: u32 phase, u32 frame, u32 stereo samples, s16 pairs) of one
process's own console and reports, for each phase change: whether every emulated frame produced its
samples (none missing, none repeated), the step at the junction against the steps of the second
before it, and the longest run of silence around it.

  audio_continuity.py <tap file>      exit status 1 when a junction looks like a gap or a pop
"""
import struct
import sys


def read(path):
    recs = []
    with open(path, "rb") as f:
        while True:
            h = f.read(12)
            if len(h) < 12:
                break
            phase, frame, n = struct.unpack("<III", h)
            data = f.read(4 * n)
            recs.append((phase, frame, list(struct.unpack("<%dh" % (2 * n), data)) if n else []))
    return recs


def longest_zero_run(samples):
    best = run = 0
    for i in range(0, len(samples) - 1, 2):
        run = run + 1 if samples[i] == 0 and samples[i + 1] == 0 else 0
        best = max(best, run)
    return best


def main():
    recs = read(sys.argv[1])
    bad = False
    phases = sorted({r[0] for r in recs})
    for p in phases:
        rs = [r for r in recs if r[0] == p]
        counts = [len(r[2]) // 2 for r in rs]
        frames = [r[1] for r in rs]
        gaps = sum(1 for a, b in zip(frames, frames[1:]) if b != a + 1)
        print("phase %d: frames %d..%d (%d records, %d frame-number gaps), samples/frame %d..%d"
              % (p, frames[0], frames[-1], len(rs), gaps, min(counts), max(counts)))
    for a, b in zip(phases, phases[1:]):
        before = [s for r in recs if r[0] == a for s in r[2]]
        after = [s for r in recs if r[0] == b for s in r[2]]
        if not before or not after:
            continue
        window = before[-2 * 32768:]   # ~1 s before the junction
        steps = sorted(abs(window[i + 2] - window[i]) for i in range(len(window) - 2))
        p999 = steps[int(len(steps) * 0.999)] if steps else 0
        jl = abs(after[0] - before[-2])
        jr = abs(after[1] - before[-1])
        junction = max(jl, jr)
        zb = longest_zero_run(window)
        za = longest_zero_run(before[-2 * 1600:] + after[:2 * 1600])   # ~50 ms each side
        last_count = len([r for r in recs if r[0] == a][-1][2]) // 2
        first_count = len([r for r in recs if r[0] == b][0][2]) // 2
        pop = junction > max(p999, 1) * 1.5 and junction > 2000
        gap = za > max(zb, 32) or first_count == 0 or last_count == 0
        bad |= pop or gap
        print("junction phase %d -> %d: step %d (99.9%% of the second before: %d, max %d), silence around it %d "
              "samples (longest in the second before: %d), last/first frame samples %d/%d -> %s"
              % (a, b, junction, p999, steps[-1] if steps else 0, za, zb, last_count, first_count,
                 "POP" if pop else "GAP" if gap else "continuous"))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
