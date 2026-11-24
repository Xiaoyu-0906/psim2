"""Step-by-step comparison of psim2 with SUMO on small generated city networks.

Usage: citycheck.py NAME [--lanes N] [--junction priority|right_before_left|traffic_light]
                         [--tls static|actuated] [--grid N] [--length M] [--period P] [--end S]
                         [--sigma S] [--speeddev D] [--seed N] [--threads N]

With sigma 0 and speeddev 0 neither simulator draws random numbers (except
for right_before_left deadlock resolution), so the vehicle states of every
step can be compared directly (tools/fcddiff.py).
"""
import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CITY = os.environ.get("PSIM2_CITY_DIR", os.path.join(ROOT, "work", "city"))
BIN = os.environ.get("PSIM2_SUMO_BIN", "")
TOOLS = os.path.join(os.environ["SUMO_HOME"], "tools")


def run(cmd, **kw):
    r = subprocess.run(cmd, capture_output=True, text=True, **kw)
    if r.returncode != 0:
        print(" ".join(cmd))
        print(r.stdout[-3000:], r.stderr[-3000:])
        raise SystemExit(1)
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("name")
    ap.add_argument("--lanes", type=int, default=1)
    ap.add_argument("--junction", default="priority")
    ap.add_argument("--tls", default="actuated")
    ap.add_argument("--grid", type=int, default=3)
    ap.add_argument("--length", type=float, default=200)
    ap.add_argument("--period", type=float, default=3)
    ap.add_argument("--end", type=float, default=1200)
    ap.add_argument("--sigma", type=float, default=0)
    ap.add_argument("--speeddev", type=float, default=0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--threads", type=int, default=1)
    ap.add_argument("--ttt", type=float, default=300)
    ap.add_argument("--sumo-order", action="store_true")
    ap.add_argument("--check", action="store_true", help="fail on empty or differing trajectories")
    a = ap.parse_args()
    d = os.path.join(CITY, a.name)
    os.makedirs(os.path.join(d, "S"), exist_ok=True)
    os.makedirs(os.path.join(d, "P"), exist_ok=True)
    net = os.path.join(d, "net.net.xml")
    gen = [os.path.join(BIN, "netgenerate"), "--grid", "--grid.number", str(a.grid), "--grid.length", str(a.length),
           "--default.lanenumber", str(a.lanes), "--no-turnarounds", "true", "-o", net]
    if a.junction == "traffic_light":
        gen += ["--default-junction-type", "traffic_light", "--tls.default-type", a.tls]
    else:
        gen += ["--default-junction-type", a.junction]
    run(gen)
    trips = os.path.join(d, "trips.xml")
    run([sys.executable, os.path.join(TOOLS, "randomTrips.py"), "-n", net, "-o", trips, "-e", str(a.end * 0.75),
         "-p", str(a.period), "--seed", str(a.seed), "--fringe-factor", "10", "--min-distance", "300",
         "--trip-attributes", 'departLane="best" departSpeed="max"' if a.lanes > 1 else ""], cwd=d)
    vtype = os.path.join(d, "vtype.add.xml")
    with open(vtype, "w") as f:
        f.write('<additional><vType id="DEFAULT_VEHTYPE" sigma="%g" speedDev="%g"/></additional>\n' % (a.sigma, a.speeddev))
    rou = os.path.join(d, "routes.rou.xml")
    run([os.path.join(BIN, "duarouter"), "-n", net, "-r", trips, "-a", vtype, "-o", rou, "--no-step-log", "--seed", str(a.seed)])
    # SUMO
    run([os.path.join(BIN, "sumo"), "-n", net, "-r", rou, "--step-length", "1", "--seed", str(a.seed),
         "--time-to-teleport", str(a.ttt), "--fcd-output", os.path.join(d, "S", "fcd.xml"),
         "--tripinfo-output", os.path.join(d, "S", "tripinfo.xml"), "--no-step-log", "true",
         "--statistic-output", os.path.join(d, "S", "stats.xml"), "--end", str(a.end)])
    # psim2
    scen = os.path.join(d, "scenario.psim2")
    run([sys.executable, os.path.join(HERE, "psim2scen.py"), net, rou, scen, "--full-net"])
    run([os.environ.get("PSIM2_BIN", os.path.join(ROOT, "psim2", "psim2")), scen, "--fcd", "--out", os.path.join(d, "P", ""), "--end", str(a.end),
         "--time-to-teleport", str(a.ttt), "--threads", str(a.threads), "--seed", str(a.seed)]
        + (["--sumo-order"] if a.sumo_order else []))
    r = run([sys.executable, os.path.join(HERE, "fcddiff.py"), d, "3"] + (["--check"] if a.check else []))
    print(r.stdout)


if __name__ == "__main__":
    main()
