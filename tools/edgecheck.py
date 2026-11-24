"""Exact comparison of edge data (SUMO edgeData vs psim2 edgedata.csv) on a citycheck/subcheck case.

Usage: edgecheck.py NAME [--period P]   (work/city/NAME must contain net.net.xml, the routes and scenario.psim2)

Runs SUMO with an edgeData definition and psim2 (--sumo-order) with --edgedata-period and reports the
largest differences of entered, sampledSeconds, speed (2 decimals) and waitingTime.
"""
import argparse
import csv
import os
import subprocess
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CITY = os.environ.get("PSIM2_CITY_DIR", os.path.join(ROOT, "work", "city"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("name")
    ap.add_argument("--period", type=float, default=300)
    ap.add_argument("--end", type=float, default=1800)
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    d = os.path.join(CITY, a.name)
    os.makedirs(os.path.join(d, "E"), exist_ok=True)
    add = os.path.join(d, "E", "edgedata.add.xml")
    with open(add, "w") as f:
        f.write('<additional>\n    <edgeData id="ed" file="edgedata.xml" period="%g"/>\n</additional>\n' % a.period)
    # bus cases (buscheck.py) keep the car routes in cars.rou.xml
    names = ("cars.rou.xml", "bus.rou.xml") if os.path.exists(os.path.join(d, "cars.rou.xml")) else ("routes.rou.xml", "bus.rou.xml")
    routes = [os.path.join(d, r) for r in names if os.path.exists(os.path.join(d, r))]
    ttt = "300" if os.path.exists(os.path.join(d, "cars.rou.xml")) else "120"  # buscheck.py / subcheck.py settings
    adds = [add] + [os.path.join(d, x) for x in ("stops.add.xml",) if os.path.exists(os.path.join(d, x))]
    subprocess.run([os.path.join(os.environ.get("PSIM2_SUMO_BIN", ""), "sumo"), "-n", os.path.join(d, "net.net.xml"), "-r", ",".join(routes),
                    "-a", ",".join(adds), "--step-length", "1", "--seed", "1", "--time-to-teleport", ttt,
                    "--ignore-junction-blocker", "20", "--no-step-log", "true", "--end", str(a.end)], check=True,
                   capture_output=True)
    subprocess.run([os.path.join(ROOT, "psim2", "psim2"), os.path.join(d, "scenario.psim2"), "--out", os.path.join(d, "E", ""),
                    "--end", str(a.end), "--time-to-teleport", ttt, "--ignore-junction-blocker", "20", "--sumo-order",
                    "--edgedata-period", str(a.period), "--no-lc-output", "--seed", "1"], check=True, capture_output=True)
    S = {}
    begin = None
    for ev, el in ET.iterparse(os.path.join(d, "E", "edgedata.xml"), events=("start", "end")):
        if ev == "start" and el.tag == "interval":
            begin = float(el.get("begin"))
        elif ev == "end" and el.tag == "edge":
            ss = float(el.get("sampledSeconds", "0"))
            S[(begin, el.get("id"))] = (int(el.get("entered", "0")), ss, float(el.get("speed", "0")) if ss > 0 else 0.,
                                        float(el.get("waitingTime", "0")))
    P = {}
    with open(os.path.join(d, "E", "edgedata.csv")) as f:
        for r in csv.DictReader(f):
            ss = float(r["sampledSeconds"])
            P[(float(r["begin"]), r["edge"])] = (int(r["entered"]), ss, round(float(r["travelledDistance"]) / ss, 2) if ss > 0 else 0.,
                                                float(r["waitingTime"]))
    keys = sorted(set(S) | set(P))
    names = ("entered", "sampledSeconds", "speed", "waitingTime")
    tol = (0, 0.011, 0.0101, 0.011)
    print("%d edge-intervals (SUMO %d, psim2 %d)" % (len(keys), len(S), len(P)))
    failed = not keys
    for i, n in enumerate(names):
        # speeds of edge-intervals with less than 1 s of samples are ratios of rounded numbers
        ks = [k for k in keys if i != 2 or S.get(k, (0, 0))[1] >= 1]
        diffs = sorted(((abs(P.get(k, (0, 0, 0, 0))[i] - S.get(k, (0, 0, 0, 0))[i]), k) for k in ks), reverse=True)
        bad = [x for x in diffs if not x[0] <= tol[i]]
        failed |= bool(bad)
        print("  %-18s differing %4d  max %.3f %s" % (n, len(bad), diffs[0][0], diffs[0][1] if bad else ""))
    return int(a.check and failed)


if __name__ == "__main__":
    raise SystemExit(main())
