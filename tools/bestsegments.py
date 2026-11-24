"""Bus travel times between consecutive stops, SUMO (work/best/stops/S<seed>_stops.xml)
vs psim2 (work/best/P<seed>/stopinfo.csv).

Usage: bestsegments.py [--sumo 1,2] [--psim 1,2,3] [--top N] [--hours A-B]

For every pair of consecutive stops of a bus route the mean travel time (end of the
first stop to the arrival at the next) and the mean dwell time are compared; segments
are ranked by their total excess over all buses (seconds per bus and run).
"""
import argparse
import collections
import csv
import os
import re
import statistics

HERE = os.path.dirname(os.path.abspath(__file__))
WORK = os.path.join(os.path.dirname(HERE), "work", "best")


def visits_sumo(seed):
    # line by line (not as XML) so that the output of a running simulation can be read
    out = collections.defaultdict(list)
    pat = re.compile(r'<stopinfo id="([^"]*)".* started="([^"]*)" ended="([^"]*)".* busStop="([^"]*)"')
    with open(os.path.join(WORK, "stops", "S%d_stops.xml" % seed)) as f:
        for line in f:
            m = pat.search(line)
            if m:
                out[m.group(1)].append((float(m.group(2)), float(m.group(3)), m.group(4)))
    return out


def visits_psim(seed):
    out = collections.defaultdict(list)
    with open(os.path.join(WORK, "P%d" % seed, "stopinfo.csv")) as f:
        for r in csv.DictReader(f):
            out[r["id"]].append((float(r["started"]), float(r["ended"]), r["busStop"]))
    return out


def segments(runs, hours):
    """(line, stop A, stop B) -> list of travel times; (line, stop) -> dwell times"""
    travel = collections.defaultdict(list)
    dwell = collections.defaultdict(list)
    for run in runs:
        for vid, vs in run.items():
            vs.sort()
            line = vid.rsplit(".", 1)[0]
            for i, (st, en, bs) in enumerate(vs):
                if hours and not hours[0] <= st / 3600 < hours[1]:
                    continue
                dwell[(line, bs)].append(en - st)
                if i + 1 < len(vs):
                    travel[(line, bs, vs[i + 1][2])].append(vs[i + 1][0] - en)
    return travel, dwell


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sumo", default="1,2")
    ap.add_argument("--psim", default="1,2,3")
    ap.add_argument("--top", type=int, default=15)
    ap.add_argument("--hours")
    a = ap.parse_args()
    hours = tuple(float(x) for x in a.hours.split("-")) if a.hours else None
    S = [visits_sumo(int(s)) for s in a.sumo.split(",")]
    P = [visits_psim(int(s)) for s in a.psim.split(",")]
    st, sd = segments(S, hours)
    pt, pd = segments(P, hours)
    nbus = statistics.fmean(len(r) for r in S)
    for name, s, p in (("travel between stops", st, pt), ("dwell at stops", sd, pd)):
        rows = []
        for k in s:
            if k in p and len(s[k]) >= 4 and len(p[k]) >= 4:
                ms, mp = statistics.fmean(s[k]), statistics.fmean(p[k])
                n = len(s[k]) / len(S)  # visits per run
                se = (statistics.pvariance(s[k]) / len(s[k]) + statistics.pvariance(p[k]) / len(p[k])) ** 0.5
                rows.append(((mp - ms) * n, k, ms, mp, n, (mp - ms) / se if se > 0 else 0.0))
        total = sum(r[0] for r in rows)
        print("%s: total excess %+.0f s per run (%+.2f s per bus)" % (name, total, total / nbus))
        rows.sort(key=lambda r: -abs(r[0]))
        for ex, k, ms, mp, n, z in rows[:a.top]:
            print("  %+8.0f  %-60s SUMO %7.1f psim2 %7.1f  n %5.0f  z %+5.1f" % (ex, " ".join(k), ms, mp, n, z))


if __name__ == "__main__":
    main()
