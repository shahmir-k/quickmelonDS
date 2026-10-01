#!/usr/bin/env python3
"""Summarise aa-app.txt: per-leg window stats, then across-leg spread and MDE."""
import sys, re, statistics as st
from collections import defaultdict
legs = defaultdict(list)
for line in open(sys.argv[1]):
    m = re.match(r'rep (\d+) win (\d+) fps ([\d.]+) runFrame ([\d.]+) wall ([\d.]+) cpuC_start ([\d.]+) cpuC_end ([\d.]+)', line)
    if m:
        r, w, fps, rf, wall, c0, c1 = m.groups()
        legs[int(r)].append((int(w), float(fps), float(rf), float(wall), float(c0), float(c1)))
summ = []
print("leg  n  fps_med fps_mean  fps_p10  rf_med  wall_med  capped%  first20_fps last20_fps  cpuC start->end")
for r, ws in sorted(legs.items()):
    ws = ws[1:]  # drop window 1: it straddles logcat -c / the load transient
    f = [w[1] for w in ws]; rf = [w[2] for w in ws]; wall = [w[3] for w in ws]
    cap = sum(x >= 59.5 for x in f) / len(f) * 100
    p10 = sorted(f)[len(f) // 10]
    summ.append((st.median(f), st.mean(f), st.median(rf), st.mean(wall)))
    print(f"{r:3d} {len(f):3d} {st.median(f):7.2f} {st.mean(f):7.2f} {p10:7.2f} {st.median(rf):7.2f} {st.median(wall):8.2f} {cap:7.1f}  "
          f"{st.mean(f[:20]):8.2f} {st.mean(f[-20:]):9.2f}   {ws[0][4]:.1f}->{ws[0][5]:.1f}")
if len(summ) >= 2:
    for i, name in enumerate(('fps median', 'fps mean', 'runFrame median', 'wall/frame mean')):
        xs = [s[i] for s in summ]; m = st.mean(xs); sd = st.stdev(xs)
        print(f"across legs  {name:16s} mean={m:.3f} sd={sd:.3f} cv={sd/m*100:.2f}% range={min(xs):.2f}..{max(xs):.2f}")
        for n in (3, 6, 10):
            print(f"     MDE(80% power, a=.05, n={n:2d}/arm) ~ {2.8*sd*(2/n)**.5:.2f}  ({2.8*sd*(2/n)**.5/m*100:.2f}%)")
