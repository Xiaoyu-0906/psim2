#!/usr/bin/env python3
"""fcdcmp.py <testdir> <time> [lane-regex] [maxpos]: SUMO (S/fcd.xml.gz) and psim2 (P/fcd.csv.gz) states side by side"""
import gzip, re, sys

d, t = sys.argv[1], float(sys.argv[2])
lre = re.compile(sys.argv[3]) if len(sys.argv) > 3 else None
maxpos = float(sys.argv[4]) if len(sys.argv) > 4 else 1e30
S, P = {}, {}
on = False
with gzip.open(d + '/S/fcd.xml.gz', 'rt') as f:
    for line in f:
        if '<timestep' in line:
            tt = float(re.search(r'time="([^"]*)"', line).group(1))
            if tt > t:
                break
            on = abs(tt - t) < 1e-6
        elif on and '<vehicle' in line:
            a = dict(re.findall(r'(\w+)="([^"]*)"', line))
            S[a['id']] = (a.get('lane'), float(a['pos']), float(a['speed']), float(a.get('posLat', 0)))
with gzip.open(d + '/P/fcd.csv.gz', 'rt') as f:
    next(f)
    for line in f:
        p = line.rstrip().split(',')
        tt = float(p[0])
        if tt > t:
            break
        if abs(tt - t) < 1e-6:
            P[p[1]] = (p[2], float(p[3]), float(p[4]), float(p[5]) if len(p) > 5 else 0.)
for vid in sorted(set(S) | set(P)):
    s, p = S.get(vid), P.get(vid)
    ok = [x for x in (s, p) if x is not None and (lre is None or lre.search(x[0])) and x[1] <= maxpos]
    if not ok:
        continue
    fmt = lambda x: '-' if x is None else '%-10s %8.2f %6.2f %6.2f' % x
    print('%-10s %s | %s %s' % (vid, fmt(s), fmt(p), '' if s == p else '  <>'))
