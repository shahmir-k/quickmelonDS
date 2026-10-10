#!/usr/bin/env python3
# jitdup.py <early.txt> <late.txt> <frames> [cpu]: JIT duplication of the blocks entered between two
# LITEV_BLOCKPROF dumps of the same deterministic run (LITEV_BLOCKPROF_ALL=1, --frames N1 and N2):
# entered blocks, compiled vs distinct guest instructions, host bytes, block-start classes, and
# how the duplicate copies are reached (seg0 = straight from a block start, seg1+ = after followed branches).
import sys, collections
def load(p):
    d = {}
    for l in open(p):
        head, host = l.rstrip('\n').split(' |')
        f = head.split()
        num, thumb, addr, last, n, idle, cnt, seq, endb = int(f[0]), int(f[1]), int(f[2],16), int(f[3],16), int(f[4]), int(f[5]), int(f[6]), int(f[7]), int(f[8])
        ins = [tuple(int(x,16) for x in t.split(':')) for t in f[9:]]
        d[seq] = dict(seq=seq, num=num, thumb=thumb, addr=addr, n=n, cnt=cnt, endb=endb, ins=ins, hb=len(host)//2 - 28)
    return d
a, b = load(sys.argv[1]), load(sys.argv[2]); F = float(sys.argv[3]); cpu = int(sys.argv[4]) if len(sys.argv) > 4 else 0
blks = []
for s, e in b.items():
    c = e['cnt'] - (a[s]['cnt'] if s in a and a[s]['addr'] == e['addr'] else 0)
    if c > 0 and e['num'] == cpu: e['d'] = c; blks.append(e)
# static branch targets and return points over ALL compiled blocks (any cpu-matching)
tgt, ret = set(), set()
for e in b.values():
    if e['num'] != cpu: continue
    for (ad, ins, bf) in e['ins']:
        if e['thumb']:
            op = ins & 0xffff
            if op >> 12 == 0xD and (op >> 8) & 0xF < 0xE: o = op & 0xff; o -= 256 if o & 0x80 else 0; tgt.add(ad + 4 + 2*o)
            elif op >> 11 == 0x1C: o = op & 0x7ff; o -= 2048 if o & 0x400 else 0; tgt.add(ad + 4 + 2*o)
            elif (op >> 11) == 0x1E: ret.add(ad + 4)   # BL pair (merged); ins high half = 2nd
            elif op >> 7 == 0x8F: ret.add(ad + 2)      # BLX reg
        else:
            if (ins >> 25) & 7 == 5:
                o = ins & 0xffffff; o -= 1 << 24 if o & 0x800000 else 0
                t = ad + 8 + 4*o
                if ins >> 28 == 0xF: t |= 1; tgt.add(t); ret.add(ad + 4)  # BLX imm
                else:
                    tgt.add(t)
                    if ins & (1 << 24): ret.add(ad + 4)
            elif ins & 0x0ffffff0 == 0x012fff30: ret.add(ad + 4)
ends = {}   # addr after a block that ended without a branch (maxsize / irq / halt cut)
for e in b.values():
    if e['num'] != cpu or not e['ins']: continue
    ad, ins, bf = e['ins'][-1]
    nxt = ad + (2 if e['thumb'] else 4)
    if e['n'] >= 32: ends[nxt] = 'maxsize'
    elif not e['endb']: ends.setdefault(nxt, 'cut')
cov = collections.Counter(); covn = collections.Counter()
for e in blks:
    for (ad, ins, bf) in e['ins']: cov[ad] += e['d']; covn[ad] += 1
tot_i = sum(len(e['ins']) for e in blks); tot_hb = sum(e['hb'] for e in blks)
print(f"cpu{cpu}: {len(blks)} blocks entered over {F:.0f} frames; entries/frame {sum(e['d'] for e in blks)/F:.0f}")
print(f"  guest instrs compiled (sum over blocks) {tot_i}, distinct guest addrs {len(covn)}, dup factor {tot_i/len(covn):.2f}")
print(f"  host bytes {tot_hb} ({tot_hb/1024:.0f} KB), per distinct guest instr {tot_hb/len(covn):.1f} B")
# start classification
def cls(e):
    s = e['addr'] | (1 if e['thumb'] else 0)
    k = []
    if s in tgt or e['addr'] in tgt: k.append('btarget')
    if e['addr'] in ret: k.append('retpoint')
    if e['addr'] in ends: k.append(ends[e['addr']])
    return '+'.join(k) or 'other'
mid = [e for e in blks if covn[e['addr']] > 1]   # start is inside some other entered block too
C = collections.Counter(); CB = collections.Counter(); CM = collections.Counter()
for e in blks:
    c = cls(e); C[c] += 1; CB[c] += e['hb']
    inside = any(e['addr'] in [x[0] for x in o['ins'][1:]] for o in blks if o is not e)
    if inside: CM[c] += 1
print("  start class: blocks / host bytes / of which start lies inside another entered block")
for c, n in C.most_common(): print(f"    {c:22s} {n:5d} {CB[c]:7d} {CM[c]:5d}")
# exact-suffix: block whose instruction list (addr,instr,bf) is a suffix of another entered block's list
sfx = 0; sfxb = 0
lists = [tuple((x[0], x[1]) for x in o['ins']) for o in blks]
for i, e in enumerate(blks):
    L = tuple((x[0], x[1]) for x in e['ins'])
    if any(len(o) > len(L) and o[-len(L):] == L for o in lists): sfx += 1; sfxb += e['hb']
print(f"  blocks that are an exact (addr,instr) suffix of another entered block: {sfx} ({sfxb} host B)")
# duplicated host bytes estimate: for each block, bytes * fraction of its instrs also covered by an earlier-ranked (larger count) block

# duplicate copies by how they are reached in their block
copies = collections.defaultdict(list)
for e in blks:
    seg = 0; per = e['hb'] / max(1, len(e['ins'])); prev = None
    for j, (ad, ins, bf) in enumerate(e['ins']):
        if prev is not None and ad != prev + (2 if e['thumb'] else 4): seg += 1
        copies[ad].append((seg, per, e['n'] >= 32)); prev = ad
T = collections.Counter(); TB = collections.Counter()
for ad, c in copies.items():
    if len(c) < 2: continue
    c.sort(key=lambda x: x[0])   # keep the copy closest to its block start as the primary
    for seg, per, mx in c[1:]:
        k = ('seg0' if seg == 0 else 'seg1' if seg == 1 else 'seg2+')
        T[k] += 1; TB[k] += per
tot = sum(e['hb'] for e in blks)
print(f"cpu{cpu} entered {tot} B; excess copies by how the copy is reached in its block (seg = followed branches/jumps before it):")
for k in ('seg0', 'seg1', 'seg2+'): print(f"  {k:6s} {T[k]:5d} instrs ~{TB[k]:.0f} B ({100*TB[k]/tot:.1f}%)")
