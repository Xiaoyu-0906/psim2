"""Compare per-step vehicle and person states of SUMO (fcd.xml) and psim (fcd.csv, personfcd.csv).

Usage: fcddiff.py <dir with S/fcd.xml[.gz] and P/fcd.csv[.gz]> [max reported steps]

Vehicles: lane, pos, speed (and the lateral position when both outputs have it). Persons: edge, pos, x, y, speed, angle.
The files are read as streams (one time step in memory), gzip-compressed files are accepted.
"""
import argparse
import csv
import gzip
import io
import math
import os
import sys
import xml.etree.ElementTree as ET


def open_any(path):
    """the file or its .gz version (None if neither exists)"""
    if os.path.exists(path):
        return open(path, "rb")
    if os.path.exists(path + ".gz"):
        return gzip.open(path + ".gz", "rb")
    return None


def sumo_steps(f):
    """(time, vehicles, persons) per timestep of a SUMO fcd-output"""
    for _, el in ET.iterparse(f):
        if el.tag == "timestep":
            veh = {v.get("id"): (v.get("lane"), float(v.get("pos")), float(v.get("speed")))
                   + ((float(v.get("posLat")),) if v.get("posLat") is not None else ()) for v in el.findall("vehicle")}
            per = {p.get("id"): (p.get("edge"), float(p.get("pos")), float(p.get("x")), float(p.get("y")),
                                 float(p.get("speed")), float(p.get("angle"))) for p in el.findall("person")}
            time = float(el.get("time"))
            if not math.isfinite(time):
                raise ValueError("non-finite SUMO timestep")
            yield time, (veh, per)
            el.clear()


def csv_steps(f, convert):
    """(time, rows by id) per time of a psim2 csv output (rows ordered by time)"""
    if f is None:
        return
    cur, rows = None, {}
    for r in csv.DictReader(io.TextIOWrapper(f)):
        t = float(r["time"])
        if not math.isfinite(t) or (cur is not None and t < cur):
            raise ValueError("invalid or out-of-order FCD time")
        if cur is not None and t != cur:
            yield cur, rows
            rows = {}
        cur = t
        if r["id"] in rows:
            raise ValueError(f"duplicate FCD id {r['id']} at {t}")
        rows[r["id"]] = convert(r)
    if cur is not None:
        yield cur, rows


def merge(*streams):
    """time-ordered union of the streams: (time, [state dict of each stream])"""
    heads = [next(s, None) for s in streams]
    while any(h is not None for h in heads):
        t = min(h[0] for h in heads if h is not None)
        out = []
        for i, h in enumerate(heads):
            if h is not None and abs(h[0] - t) < 1e-6:
                out.append(h[1])
                heads[i] = next(streams[i], None)
            else:
                out.append({})
        yield t, out


def angle_diff(a, b):
    x = abs(a - b) % 360
    return min(x, 360 - x)


def person_differs(x, y):
    if x is None or y is None or x[0] != y[0]:
        return True
    return not (all(abs(x[i] - y[i]) <= 0.011 for i in range(1, 5))
                and angle_diff(x[5], y[5]) <= 0.11)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory")
    parser.add_argument("limit", type=int, nargs="?", default=2)
    parser.add_argument("--check", action="store_true", help="require matching non-empty city trajectories")
    parser.add_argument("--all-differences", action="store_true", help="report every differing step")
    args = parser.parse_args()
    d, limit = args.directory, args.limit
    sf = open_any(os.path.join(d, "S", "fcd.xml"))
    pf = open_any(os.path.join(d, "P", "fcd.csv"))
    ppf = open_any(os.path.join(d, "P", "personfcd.csv"))
    if sf is None or pf is None:
        raise FileNotFoundError(f"missing reference or candidate FCD in {d}")

    p_veh = csv_steps(pf, lambda r: (r["lane"], float(r["pos"]), float(r["speed"]))
                      + ((float(r["posLat"]),) if r.get("posLat") is not None else ()))
    p_per = csv_steps(ppf, lambda r: (r["edge"], float(r["pos"]), float(r["x"]), float(r["y"]), float(r["speed"]),
                                      float(r["angle"])))
    shown = states = pstates = steps = 0
    for t, (reference, b, pp) in merge(sumo_steps(sf), p_veh, p_per):
        a, sp = reference or ({}, {})
        steps += 1
        bad = []
        for vid in set(a) | set(b):
            x, y = a.get(vid), b.get(vid)
            states += 1
            if (x is None or y is None or x[0] != y[0] or not abs(x[1] - y[1]) <= 0.011 or not abs(x[2] - y[2]) <= 0.011
                    or (len(x) > 3 and len(y) > 3 and not abs(x[3] - y[3]) <= 0.011)):
                bad.append((vid, x, y))
        for pid in set(sp) | set(pp):
            x, y = sp.get(pid), pp.get(pid)
            pstates += 1
            if person_differs(x, y):
                bad.append(("person " + pid, x, y))
        if bad:
            print("time %.2f differences %d" % (t, len(bad)))
            for z in sorted(bad, key=str)[:8]:
                print("   ", z)
            shown += 1
            if not args.all_differences and shown >= limit:
                break
    extra = ", %d person states" % pstates if pstates else ""
    print("compared %d vehicle states%s over %d steps, differing steps reported: %d" % (states, extra, steps, shown))
    return int(args.check and (shown > 0 or states == 0))


if __name__ == "__main__":
    sys.exit(main())
