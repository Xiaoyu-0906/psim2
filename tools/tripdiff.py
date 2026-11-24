"""Per-vehicle comparison of SUMO tripinfo.xml (DIR/S) with psim2 tripinfo.csv (DIR/P).

Usage: tripdiff.py DIR [N]   (N: number of differing vehicles to list per attribute)
"""
import argparse
import csv
import os
import sys
import xml.etree.ElementTree as ET

KEYS = ["depart", "departDelay", "duration", "routeLength", "timeLoss", "waitingTime", "waitingCount", "stopTime"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory")
    parser.add_argument("limit", nargs="?", type=int, default=5)
    parser.add_argument("--check", action="store_true", help="require the same non-empty set of trips and matching attributes")
    args = parser.parse_args()
    d, n = args.directory, args.limit
    S = {}
    for _, el in ET.iterparse(os.path.join(d, "S", "tripinfo.xml")):
        if el.tag == "tripinfo":
            S[el.get("id")] = {k: float(el.get(k, "0")) for k in KEYS}
            el.clear()
    P = {}
    with open(os.path.join(d, "P", "tripinfo.csv")) as f:
        for r in csv.DictReader(f):
            if r["id"] in P:
                raise ValueError("duplicate trip " + r["id"])
            P[r["id"]] = {k: float(r[k]) for k in KEYS}
    common = sorted(set(S) & set(P))
    print("%d vehicles in SUMO, %d in psim2, %d common" % (len(S), len(P), len(common)))
    failed = not common or S.keys() != P.keys()
    for k in KEYS:
        diff = [(abs(P[i][k] - S[i][k]), i) for i in common if not abs(P[i][k] - S[i][k]) <= 0.011]
        failed |= bool(diff)
        tot = sum(P[i][k] - S[i][k] for i in common)
        print("%-12s differing %5d  sum(psim2-SUMO) %+9.2f" % (k, len(diff), tot))
        for _, i in sorted(diff, reverse=True)[:n]:
            print("    %-14s SUMO %9.2f psim2 %9.2f" % (i, S[i][k], P[i][k]))
    return int(args.check and failed)


if __name__ == "__main__":
    sys.exit(main())
