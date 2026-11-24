#!/usr/bin/env bash
# Rail speed benchmark: a double-track rail grid (rail signals at every junction, trains of 120 m, RB425,
# routes from duarouter) run by SUMO and by psim2 with 1, 2, 4, 8, 10 threads. The runs are interleaved and
# repeated (REPS rounds of SUMO, psim2 1 thread, psim2 2 threads, ...) so that load from other processes hits
# all configurations alike; the report gives the fastest and the median wall time of each configuration and
# the load average seen during the rounds. The report also compares each thread count's tripinfo with SUMO's.
# Usage: railbench.sh [GRID] [LEN] [PERIOD] [END] [REPS]   (default 20 1000 2 5400 3: 20x20 junctions 1 km
# apart, a train every 2 s until 3600 s, simulated until 5400 s)
# PYTHON selects the interpreter (default python3). SUMO_HOME locates tools/randomTrips.py.
# PSIM2_SUMO_BIN can select a binary directory independently of SUMO_HOME.
set -euo pipefail
export LC_ALL=C
cd "$(dirname "$0")/.."
GRID=${1:-20}
LEN=${2:-1000}
PERIOD=${3:-2}
END=${4:-5400}
REPS=${5:-3}
THREADS=(1 2 4 8 10)
if [[ ! $GRID =~ ^[0-9]+$ || ! $LEN =~ ^[0-9]+$ || ! $REPS =~ ^[1-9][0-9]*$ || $PERIOD == */* ]]; then
  echo 'Usage: railbench.sh [grid size] [length in metres] [departure period] [end time] [positive rounds]' >&2
  exit 1
fi
python=${PYTHON:-python3}
if ! command -v "$python" >/dev/null 2>&1; then
  echo "Cannot find Python interpreter '$python'. Set PYTHON to a Python 3 executable." >&2
  exit 1
fi
if [[ -z ${SUMO_HOME:-} ]]; then
  if ! SUMO_HOME=$("$python" -c 'from sumo import SUMO_HOME; print(SUMO_HOME)' 2>/dev/null); then
    echo "Set SUMO_HOME to the SUMO installation or install tools/requirements.txt with '$python'." >&2
    exit 1
  fi
fi
export SUMO_HOME

sumo_binary() {
  local name=$1 binary
  if [[ -n ${PSIM2_SUMO_BIN:-} ]]; then
    binary="$PSIM2_SUMO_BIN/$name"
  else
    binary=$(PATH="$SUMO_HOME/bin:$PATH" command -v "$name") || binary=
  fi
  if [[ ! -f $binary || ! -x $binary ]]; then
    echo "Cannot find executable '$name'. Set PSIM2_SUMO_BIN to the SUMO binary directory." >&2
    return 1
  fi
  printf '%s\n' "$binary"
}

sumo=$(sumo_binary sumo)
O=work/railbench/g${GRID}_${LEN}_${PERIOD}
mkdir -p "$O"
if [[ ! -f "$O/scenario.psim2" ]]; then
  random_trips="$SUMO_HOME/tools/randomTrips.py"
  if [[ ! -f $random_trips ]]; then
    echo "Cannot find '$random_trips'. Set SUMO_HOME to the SUMO installation containing tools/randomTrips.py." >&2
    exit 1
  fi
  netgenerate=$(sumo_binary netgenerate)
  DUAROUTER_BINARY=$(sumo_binary duarouter)
  export DUAROUTER_BINARY  # randomTrips.py uses sumolib to find its router
  "$netgenerate" --grid --grid.number "$GRID" --grid.length "$LEN" --default.lanenumber 1 --default.speed 33.33 \
    --default.allow rail --default.junctions.type rail_signal --no-turnarounds -o "$O/bidi.net.xml" >/dev/null
  # separate tracks for both directions (netgenerate makes them one bidirectional track)
  sed -E 's/ bidi="[^"]*"//' "$O/bidi.net.xml" > "$O/net.net.xml"
  rm "$O/bidi.net.xml"
  cat > "$O/types.add.xml" <<'EOF'
<additional>
    <vType id="train" vClass="rail" length="120" maxSpeed="33.33" carFollowModel="Rail" trainType="RB425"/>
</additional>
EOF
  "$python" "$random_trips" -n "$O/net.net.xml" --edge-permission rail --seed 7 --period "$PERIOD" --end 3600 \
    --min-distance "$((10#$LEN * 6))" --trip-attributes 'type="train"' --additional-file "$O/types.add.xml" \
    -o "$O/trips.rou.xml" -r "$O/routes.rou.xml" >/dev/null
  rm -f "$O/trips.rou.xml" "$O/routes.rou.alt.xml"
  "$python" tools/psim2scen.py "$O/net.net.xml" "$O/routes.rou.xml" "$O/scenario.psim2.tmp" --full-net >/dev/null
  mv "$O/scenario.psim2.tmp" "$O/scenario.psim2"
fi

time_run() {
  local label=$1 directory=$2
  shift 2
  mkdir -p "$directory"
  if ! { time -p "$@" >/dev/null; } 2>"$directory/time.txt"; then
    cat "$directory/time.txt" >&2
    return 1
  fi
  awk -v label="$label" '
    $1 == "real" { real = $2 }
    $1 == "user" { user = $2 }
    END { if (real == "" || user == "") exit 1; print label, real, user }
  ' "$directory/time.txt" >> "$O/times.txt"
}

: > "$O/times.txt"
: > "$O/load.txt"
for ((round = 0; round < REPS; round++)); do
  uptime >> "$O/load.txt"
  time_run sumo "$O/s" "$sumo" -n "$O/net.net.xml" -r "$O/routes.rou.xml" --end "$END" --no-step-log --no-warnings true \
    --tripinfo-output "$O/s/tripinfo.xml"
  for t in "${THREADS[@]}"; do
    time_run "psim2-$t" "$O/p$t" psim2/psim2 "$O/scenario.psim2" --out "$O/p$t/" --end "$END" --threads "$t"
  done
done
uptime >> "$O/load.txt"
"$python" - "$O" "$GRID" "$LEN" "$PERIOD" "$END" <<'EOF'
import csv
import re
import statistics
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

o, grid, length, period, end = sys.argv[1:]
root = Path(o)
runs = {}
with (root / "times.txt").open() as source:
    for line in source:
        name, real, user = line.split()
        runs.setdefault(name, []).append((float(real), float(user)))
loads = []
with (root / "load.txt").open() as source:
    for line in source:
        match = re.search(r"\bload averages?:\s*([0-9]+(?:\.[0-9]+)?)(?=[,\s]|$)", line)
        if not match:
            sys.exit(f"Cannot parse one-minute load from uptime: {line.strip()}")
        loads.append(float(match.group(1)))
sumo_trips = {trip.attrib["id"]: trip.attrib for trip in ET.parse(root / "s" / "tripinfo.xml").iter("tripinfo")}
out = ["rail grid %sx%s, %s m, a train every %s s until 3600 s, end %s s; %d trains" % (grid, grid, length, period, end, len(sumo_trips)),
       "load average (1 min) at the rounds: " + " ".join("%.1f" % l for l in loads),
       "%-10s %8s %8s %8s %9s" % ("", "min [s]", "median", "user [s]", "vs SUMO")]
sumo = min(r for r, u in runs["sumo"])
for name, rs in runs.items():
    best = min(r for r, u in rs)
    speedup = "%.2fx" % (sumo / best) if best > 0 and sumo > 0 else "n/a"
    out.append("%-10s %8.2f %8.2f %8.2f %9s" % (name, best, statistics.median(r for r, u in rs), min(u for r, u in rs), speedup))
fields = ("depart", "arrival", "duration", "routeLength", "timeLoss", "waitingTime")
for name in runs:
    if not name.startswith("psim2"):
        continue
    with (root / ("p" + name.split("-")[1]) / "tripinfo.csv").open(newline="") as source:
        psim_trips = {row["id"]: row for row in csv.DictReader(source)}
    diff = len(sumo_trips.keys() ^ psim_trips.keys())
    for vehicle in sumo_trips.keys() & psim_trips.keys():
        reference, candidate = sumo_trips[vehicle], psim_trips[vehicle]
        if not all(abs(float(reference[f]) - float(candidate[f])) <= 0.011 for f in fields):
            diff += 1
    total = len(sumo_trips.keys() | psim_trips.keys())
    out.append(f"{name}: {diff} of {total} trips differ from SUMO's tripinfo")
report = "\n".join(out)
(root / "report.txt").write_text(report + "\n")
print(report)
EOF
