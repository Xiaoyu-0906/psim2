"""Step-by-step comparison of runtime network edits (psim2 --edits vs SUMO TraCI).

Usage: editcheck.py NAME BASE [--end S] [--sumo-order]

Takes the network and routes of an earlier citycheck run (work/city/BASE),
picks the two most used multi-lane edges and, while the simulation runs,
lowers a speed limit, closes a lane to cars (bus only) and later reverts both.
SUMO receives the edits through TraCI between two steps, psim2 from an edits
file; all vehicle states are compared (sigma 0).
"""
import argparse
import collections
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CITY = os.environ.get("PSIM2_CITY_DIR", os.path.join(ROOT, "work", "city"))
BIN = os.environ.get("PSIM2_SUMO_BIN", "")
sys.path.insert(0, os.path.join(os.environ["SUMO_HOME"], "tools"))
import traci  # noqa: E402

ALL = "passenger,bus,truck,delivery,coach"


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
    rou = os.path.join(d, "routes.rou.xml")
    shutil.copy(os.path.join(base, "net.net.xml"), net)
    shutil.copy(os.path.join(base, "routes.rou.xml"), rou)
    use = collections.Counter()
    for _, el in ET.iterparse(rou):
        if el.tag == "route" and el.get("edges"):
            use.update(el.get("edges").split()[1:-1])
    e1, e2 = [e for e, _ in use.most_common(2)]
    # time, kind, lane, value (psim2 syntax)
    edits = [(200, "LANE_SPEED", e1 + "_0", "5.0"),
             (300, "LANE_PERMISSIONS", e2 + "_1", "bus"),
             (500, "LANE_SPEED", e1 + "_0", "13.89"),
             (650, "LANE_PERMISSIONS", e2 + "_1", ALL)]
    with open(os.path.join(d, "edits.txt"), "w") as f:
        for e in edits:
            f.write("%d %s %s %s\n" % e)
    traci.start([os.path.join(BIN, "sumo"), "-n", net, "-r", rou, "--step-length", "1", "--seed", "1",
                 "--time-to-teleport", "300", "--fcd-output", os.path.join(d, "S", "fcd.xml"), "--no-step-log", "true"])
    pending = list(edits)
    while traci.simulation.getTime() < a.end:
        # getTime() is the time of the step that runs next: edits apply before it, as in psim2
        now = traci.simulation.getTime()
        while pending and pending[0][0] <= now:
            _, kind, lane, value = pending.pop(0)
            if kind == "LANE_SPEED":
                traci.lane.setMaxSpeed(lane, float(value))
            else:
                traci.lane.setAllowed(lane, value.split(","))
        traci.simulationStep()
    traci.close()
    scen = os.path.join(d, "scenario.psim2")
    run([sys.executable, os.path.join(HERE, "psim2scen.py"), net, rou, scen, "--full-net"])
    run([os.path.join(ROOT, "psim2", "psim2"), scen, "--fcd", "--out", os.path.join(d, "P", ""), "--end", str(a.end),
         "--time-to-teleport", "300", "--edits", os.path.join(d, "edits.txt")] + (["--sumo-order"] if a.sumo_order else []))
    print("edits:", edits)
    print(run([sys.executable, os.path.join(HERE, "fcddiff.py"), d, "3"] + (["--check"] if a.check else [])).stdout)


if __name__ == "__main__":
    main()
