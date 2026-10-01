#!/usr/bin/env python3
"""ui.py [regex] — read a uiautomator XML dump on stdin.
No arg: print every text node with its bounds center.  With a regex: print 'x y' of the
first node whose text matches (exit 1 if none)."""
import sys, re, xml.etree.ElementTree as ET
root = ET.fromstring(sys.stdin.read())
pat = re.compile(sys.argv[1]) if len(sys.argv) > 1 else None
for n in root.iter('node'):
    t = n.get('text', '')
    if not t: continue
    x1, y1, x2, y2 = map(int, re.findall(r'\d+', n.get('bounds')))
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    if pat is None: print(f'{cx:4d} {cy:4d}  {t}')
    elif pat.search(t): print(cx, cy); sys.exit(0)
if pat: sys.exit(1)
