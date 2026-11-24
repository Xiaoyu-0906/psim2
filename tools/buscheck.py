"""Step-by-step comparison of bus stops (psim2 vs SUMO, sigma 0).

Usage: buscheck.py NAME BASE [--parking] [--end S] [--sumo-order]

Takes the network and the car routes of an earlier citycheck run (work/city/BASE),
adds bus stops on two long car routes and bus flows stopping there (duration,
until, parking or stopping on the lane), and compares all vehicle states.
"""
import argparse
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CITY = os.environ.get("PSIM2_CITY_DIR", os.path.join(ROOT, "work", "city"))
BIN = os.environ.get("PSIM2_SUMO_BIN", "")


def run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(" ".join(cmd))
        print(r.stdout[-3000:], r.stderr[-3000:])
        raise SystemExit(1)
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("name")
    ap.add_argument("base")
    ap.add_argument("--end", type=float, default=1200)
    ap.add_argument("--sumo-order", action="store_true")
    ap.add_argument("--check", action="store_true", help="fail on empty or differing trajectories")
    a = ap.parse_args()
    base = os.path.join(CITY, a.base)
    d = os.path.join(CITY, a.name)
    for sub in ("S", "P"):
        os.makedirs(os.path.join(d, sub), exist_ok=True)
    net = os.path.join(d, "net.net.xml")
    shutil.copy(os.path.join(base, "net.net.xml"), net)
    lane_len = {}
    for _, el in ET.iterparse(net):
        if el.tag == "lane":
            lane_len[el.get("id")] = float(el.get("length"))
    routes = []
    for _, el in ET.iterparse(os.path.join(base, "routes.rou.xml")):
        if el.tag == "route" and el.get("edges"):
            e = el.get("edges").split()
            if len(e) >= 5 and not any(r[1] == e[1] for r in routes):
                routes.append(e)
        if len(routes) == 2:
            break
    stops = []
    with open(os.path.join(d, "stops.add.xml"), "w") as f:
        f.write("<additional>\n")
        for i, e in enumerate(routes):
            for j in (1, 3):
                lane = e[j] + "_0"
                L = lane_len[lane]
                sid = "bs_%d_%d" % (i, j)
                stops.append(sid)
                f.write('    <busStop id="%s" lane="%s" startPos="%.2f" endPos="%.2f" parkingLength="14"/>\n'
                        % (sid, lane, L * 0.35, L * 0.35 + 30))
        f.write("</additional>\n")
    cars = os.path.join(d, "cars.rou.xml")
    shutil.copy(os.path.join(base, "routes.rou.xml"), cars)
    buses = os.path.join(d, "bus.rou.xml")
    with open(buses, "w") as f:
        f.write('<routes>\n    <vType id="bus" vClass="bus" sigma="0" speedDev="0"/>\n')
        for i, e in enumerate(routes):
            f.write('    <route id="bus%d" edges="%s">\n' % (i, " ".join(e)))
            # first stop: parking with duration, second: on the lane with until
            f.write('        <stop busStop="bs_%d_1" duration="20" parking="true"/>\n' % i)
            # until counts from each bus's departure (route stop) and binds: buses arrive earlier
            f.write('        <stop busStop="bs_%d_3" duration="10" until="%d" parking="false"/>\n' % (i, 150 + 10 * i))
            f.write("    </route>\n")
        for i in range(len(routes)):
            f.write('    <flow id="line%d" type="bus" route="bus%d" begin="%d" end="%d" period="90"/>\n'
                    % (i, i, 5 + i, a.end * 0.7))
        f.write("</routes>\n")
    rou = cars + "," + buses
    run([os.path.join(BIN, "sumo"), "-n", net, "-r", rou, "-a", os.path.join(d, "stops.add.xml"), "--step-length", "1",
         "--seed", "1", "--time-to-teleport", "300", "--fcd-output", os.path.join(d, "S", "fcd.xml"),
         "--tripinfo-output", os.path.join(d, "S", "tripinfo.xml"), "--no-step-log", "true", "--end", str(a.end)])
    scen = os.path.join(d, "scenario.psim2")
    run([sys.executable, os.path.join(HERE, "psim2scen.py"), net, rou, scen, "--full-net",
         "--additional", os.path.join(d, "stops.add.xml")])
    run([os.path.join(ROOT, "psim2", "psim2"), scen, "--fcd", "--out", os.path.join(d, "P", ""), "--end", str(a.end),
         "--time-to-teleport", "300"] + (["--sumo-order"] if a.sumo_order else []))
    print(run([sys.executable, os.path.join(HERE, "fcddiff.py"), d, "3"] + (["--check"] if a.check else [])).stdout)


if __name__ == "__main__":
    main()
