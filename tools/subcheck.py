"""Step-by-step comparison on a piece of a real network (psim2 vs SUMO, sigma 0).

Usage: subcheck.py NAME NET JUNCTION[,JUNCTION...] [--hops K] [--period P] [--end S] [--sumo-order]
                   [--bus ROUTEFILE:ROUTE:FIRST:LAST ...] [--stops-file ADD] [--bus-period P]

Cuts the edges within K junction hops around the given junctions out of NET
(netconvert --keep-edges.explicit), drives random trips through it and compares
all vehicle states of SUMO and psim2.

--bus takes the edges FIRST..LAST of a bus route (with its stops) from ROUTEFILE; the
cut then also contains these edges and the junctions along them (JUNCTION may be "-"),
the bus stops come from --stops-file, and a bus flow (period --bus-period) drives the
piece of route, parking at its stops.
"""
import argparse
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CITY = os.environ.get("PSIM2_CITY_DIR", os.path.join(ROOT, "work", "city"))
BIN = os.environ.get("PSIM2_SUMO_BIN", "")
TOOLS = os.path.join(os.environ["SUMO_HOME"], "tools")


def run(cmd, cwd=None):
    r = subprocess.run(cmd, capture_output=True, text=True, cwd=cwd)
    if r.returncode != 0:
        print(" ".join(cmd))
        print(r.stdout[-3000:], r.stderr[-3000:])
        raise SystemExit(1)
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("name")
    ap.add_argument("net")
    ap.add_argument("junctions")
    ap.add_argument("--hops", type=int, default=2)
    ap.add_argument("--period", type=float, default=2)
    ap.add_argument("--end", type=float, default=1800)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--sumo-order", action="store_true")
    ap.add_argument("--bus", action="append", default=[])
    ap.add_argument("--stops-file")
    ap.add_argument("--bus-period", type=float, default=120)
    ap.add_argument("--check", action="store_true", help="fail on empty or differing trajectories")
    a = ap.parse_args()
    d = os.path.join(CITY, a.name)
    for sub in ("S", "P"):
        os.makedirs(os.path.join(d, sub), exist_ok=True)
    # normal edges around the junctions
    frm, to = {}, {}
    for _, el in ET.iterparse(a.net):
        if el.tag == "edge" and el.get("function") != "internal":
            frm[el.get("id")] = el.get("from")
            to[el.get("id")] = el.get("to")
            el.clear()
        elif el.tag in ("junction", "connection", "tlLogic"):
            el.clear()
    junctions = set(a.junctions.split(",")) - {"-"}
    keep = set()
    buses = []  # (edges, stops)
    for spec in a.bus:
        rfile, rid, first, last = spec.split(":")
        for el in ET.parse(rfile).getroot():
            if el.tag == "route" and el.get("id") == rid:
                edges = el.get("edges").split()
                edges = edges[edges.index(first):edges.index(last) + 1]
                stops = [st.attrib for st in el.findall("stop")]
                buses.append((edges, stops))
                keep |= set(edges)
                junctions |= {frm[e] for e in edges} | {to[e] for e in edges}
    for _ in range(a.hops):
        new = {e for e in frm if frm[e] in junctions or to[e] in junctions}
        keep |= new
        junctions |= {frm[e] for e in new} | {to[e] for e in new}
    with open(os.path.join(d, "edges.txt"), "w") as f:
        f.write("\n".join(sorted(keep)) + "\n")
    net = os.path.join(d, "net.net.xml")
    run([os.path.join(BIN, "netconvert"), "-s", a.net, "--keep-edges.input-file", os.path.join(d, "edges.txt"),
         "-o", net, "--no-warnings"])
    trips = os.path.join(d, "trips.xml")
    run([sys.executable, os.path.join(TOOLS, "randomTrips.py"), "-n", net, "-o", trips, "-e", str(a.end * 0.8),
         "-p", str(a.period), "--seed", str(a.seed), "--fringe-factor", "100", "--min-distance", "100",
         "--trip-attributes", 'departLane="best" departSpeed="max"'], cwd=d)  # randomTrips writes temporary files to the cwd
    vtype = os.path.join(d, "vtype.add.xml")
    with open(vtype, "w") as f:
        f.write('<additional><vType id="DEFAULT_VEHTYPE" sigma="0" speedDev="0"/></additional>\n')
    rou = os.path.join(d, "routes.rou.xml")
    run([os.path.join(BIN, "duarouter"), "-n", net, "-r", trips, "-a", vtype, "-o", rou, "--no-step-log",
         "--seed", str(a.seed), "--ignore-errors"])
    add = []
    if buses:
        lanes = set()
        for _, el in ET.iterparse(net):
            if el.tag == "lane":
                lanes.add(el.get("id"))
        used = {}
        for _, el in ET.iterparse(a.stops_file):
            if el.tag == "busStop" and el.get("lane") in lanes:
                used[el.get("id")] = dict(el.attrib)
        with open(os.path.join(d, "stops.add.xml"), "w") as f:
            f.write("<additional>\n")
            for sid, b in sorted(used.items()):
                f.write('    <busStop id="%s" lane="%s" startPos="%s" endPos="%s" parkingLength="%s"/>\n'
                        % (sid, b["lane"], b.get("startPos", "0"), b.get("endPos", "-1"), b.get("parkingLength", "0")))
            f.write("</additional>\n")
        with open(os.path.join(d, "bus.rou.xml"), "w") as f:
            f.write('<routes>\n    <vType id="bus" vClass="bus" sigma="0" speedDev="0"/>\n')
            for i, (edges, stops) in enumerate(buses):
                f.write('    <route id="bus%d" edges="%s">\n' % (i, " ".join(edges)))
                for st in stops:
                    if st.get("busStop") in used and used[st["busStop"]]["lane"].rsplit("_", 1)[0] in edges:
                        f.write('        <stop busStop="%s" duration="%s" parking="true"/>\n' % (st["busStop"], st.get("duration", "20")))
                f.write("    </route>\n")
                f.write('    <flow id="line%d" type="bus" route="bus%d" begin="%d" end="%d" period="%g"/>\n'
                        % (i, i, 10 + 7 * i, a.end * 0.8, a.bus_period))
            f.write("</routes>\n")
        add = [os.path.join(d, "stops.add.xml")]
        rou = rou + "," + os.path.join(d, "bus.rou.xml")
    run([os.path.join(BIN, "sumo"), "-n", net, "-r", rou, "--step-length", "1", "--seed", str(a.seed)] + (["-a"] + add if add else []) + [
         "--time-to-teleport", "120", "--ignore-junction-blocker", "20", "--fcd-output", os.path.join(d, "S", "fcd.xml"),
         "--tripinfo-output", os.path.join(d, "S", "tripinfo.xml"), "--no-step-log", "true", "--end", str(a.end)])
    scen = os.path.join(d, "scenario.psim2")
    run([sys.executable, os.path.join(HERE, "psim2scen.py"), net, rou, scen, "--full-net"] + (["--additional"] + add if add else []))
    run([os.environ.get("PSIM2_BIN", os.path.join(ROOT, "psim2", "psim2")), scen, "--fcd", "--out", os.path.join(d, "P", ""), "--end", str(a.end),
         "--time-to-teleport", "120", "--ignore-junction-blocker", "20", "--seed", str(a.seed)]
        + (["--sumo-order"] if a.sumo_order else []))
    print(run([sys.executable, os.path.join(HERE, "fcddiff.py"), d, "3"] + (["--check"] if a.check else [])).stdout)


if __name__ == "__main__":
    main()
