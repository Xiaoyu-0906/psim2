"""Run BeST Mitte (berlin-mitte.sumocfg settings) with SUMO or psim2.

Usage: bestrun.py sumo SEED [--end E]
       bestrun.py psim SEED [--end E] [--threads N]

Outputs go to work/best/S<seed>/ and work/best/P<seed>/. SUMO reads the
extracted scenario from PSIM2_BEST_DIR. psim reads work/best/mitte.psim2.
Both simulators write trip info, a summary over time,
statistics and hourly edge data (SUMO edgeData / psim2 edgedata.csv), which
tools/bestcompare.py compares.
"""
import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
BEST = os.environ.get("PSIM2_BEST_DIR")
WORK = os.path.join(ROOT, "work", "best")
# berlin-mitte.sumocfg <processing>
PROCESSING = ["--route-steps", "200", "--ignore-junction-blocker", "20", "--time-to-teleport", "120",
              "--time-to-teleport.highways", "0", "--eager-insert", "false"]


def run_sumo(seed, end):
    d = os.path.join(WORK, "S%d" % seed)
    os.makedirs(d, exist_ok=True)
    add = os.path.join(d, "edgedata.add.xml")
    with open(add, "w") as f:
        f.write('<additional>\n    <edgeData id="hourly" file="edgedata.xml" period="3600"/>\n</additional>\n')
    cmd = [os.path.join(os.environ.get("PSIM2_SUMO_BIN", ""), "sumo"),
           "-n", os.path.join(BEST, "berlin.net.xml"),
           "-r", os.path.join(BEST, "cutouts", "mitte.rou.gz") + "," + os.path.join(BEST, "cutouts", "mitte_bus.rou.xml"),
           "-a", os.path.join(BEST, "berlin_bus_stops.add.xml") + "," + add,
           "--begin", "0", "--end", str(end), "--seed", str(seed)] + PROCESSING + [
           "--tripinfo-output", os.path.join(d, "tripinfo.xml"),
           "--summary-output", os.path.join(d, "summary.xml"),
           "--statistic-output", os.path.join(d, "stats.xml"),
           "--no-step-log", "true", "--duration-log.statistics", "true"]
    t0 = time.time()
    with open(os.path.join(d, "log.txt"), "w") as log:
        subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, check=True)
    with open(os.path.join(d, "wall.txt"), "w") as f:
        f.write("%.1f\n" % (time.time() - t0))


def run_psim(seed, end, threads):
    d = os.path.join(WORK, "P%d" % seed)
    os.makedirs(d, exist_ok=True)
    scen = os.path.join(WORK, "mitte.psim2")
    cmd = [os.path.join(ROOT, "psim2", "psim2"), scen, "--out", d + "/", "--end", str(end), "--seed", str(seed),
           "--threads", str(threads), "--time-to-teleport", "120", "--ignore-junction-blocker", "20",
           "--no-lc-output", "--edgedata-period", "3600", "--stop-output"]
    t0 = time.time()
    with open(os.path.join(d, "log.txt"), "w") as log:
        subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, check=True)
    with open(os.path.join(d, "wall.txt"), "w") as f:
        f.write("%.1f\n" % (time.time() - t0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sim", choices=["sumo", "psim"])
    ap.add_argument("seed", type=int)
    ap.add_argument("--end", type=float, default=86400)
    ap.add_argument("--threads", type=int, default=8)
    a = ap.parse_args()
    if a.sim == "sumo":
        if not BEST:
            print("Set PSIM2_BEST_DIR to the extracted BeST v2 directory.", file=sys.stderr)
            return 1
        if not os.path.isdir(BEST):
            print(f"PSIM2_BEST_DIR is not a directory: {BEST}", file=sys.stderr)
            return 1
        run_sumo(a.seed, a.end)
    else:
        run_psim(a.seed, a.end, a.threads)


if __name__ == "__main__":
    sys.exit(main())
