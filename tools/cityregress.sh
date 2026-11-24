#!/usr/bin/env bash
# Acceptance run against SUMO 1.28. Optional arguments after --no-sub
# are forwarded to the synthetic city generator (e.g. --threads 4).
set -euo pipefail
cd "$(dirname "$0")/.."
nosub=0
if [[ ${1:-} == --no-sub ]]; then nosub=1; shift; fi
B=${PSIM2_BEST_DIR:-}
export PSIM2_CITY_DIR=${PSIM2_CITY_DIR:-"$PWD/work/city"}
mkdir -p "$PSIM2_CITY_DIR"
city_root=$(cd "$PSIM2_CITY_DIR" && pwd)
# Keep concurrent runs and artifacts from earlier binaries independent.
PSIM2_CITY_DIR=$(mktemp -d "$city_root/run.XXXXXX")
echo "City outputs: $PSIM2_CITY_DIR"
T1=cluster_1862786410_1862786411_1862786436_1862786453_1862786464_1862786476_1862786491_1862786497_21432798_29123438_29221602_29221603_3168036672_3168036673_3168036674_3168036675
T2=cluster_12614600_1860618754_1860618762_1860618778_1860618795_1860618797_1860618805_1860618824_1860618825_29270276_29270294_29270295_3649758678_3651880147_3657149480_3657636381
cases=(
  "prio1 --junction priority --period 4 --end 600"
  "prio2 --junction priority --period 1 --end 1200 --grid 4"
  "tlsA1 --junction traffic_light --tls actuated --period 1.5 --end 1200 --grid 4"
  "tlsS1 --junction traffic_light --tls static --period 1.5 --end 1200 --grid 4"
  "lc_prio1 --junction priority --lanes 2 --period 2 --end 1200 --grid 4 --sumo-order"
  "lc_tls1 --junction traffic_light --tls actuated --lanes 2 --period 1.5 --end 1200 --grid 4 --sumo-order"
  "lc_prio2 --junction priority --lanes 3 --period 1 --end 1500 --grid 5 --sumo-order"
  "lc_tls2 --junction traffic_light --tls actuated --lanes 3 --period 0.8 --end 1500 --grid 5 --sumo-order"
)
trips() {
  python3 tools/tripdiff.py "$PSIM2_CITY_DIR/$1" 0 --check
}

for c in "${cases[@]}"; do
  read -r -a args <<< "$c"
  echo "${args[0]}"
  python3 tools/citycheck.py "${args[@]}" "$@" --check
  trips "${args[0]}"
done
for c in "bus1 tlsA1 --end 1200" "bus2 lc_tls2 --end 1500 --sumo-order" "bus3 lc_prio2 --end 1500 --sumo-order"; do
  read -r -a args <<< "$c"
  echo "${args[0]}"
  python3 tools/buscheck.py "${args[@]}" --check
  trips "${args[0]}"
done
for c in "tlsA1 1200" "lc_tls2 1500" "bus2 1500"; do
  read -r name end <<< "$c"
  python3 tools/edgecheck.py "$name" --end "$end" --check
done

# The reference has two known bicycle mismatches. Check the full difference
# reports against the pre-release baseline, including all differing timesteps.
official=$(mktemp -d "$PSIM2_CITY_DIR/official.XXXXXX")
rc=0
PSIM2_WORK="$official" python3 tools/sumotest.py --expect-count 25 --derandomize \
  junction_model/junction_type/zipper extended/bikes > "$official/results" || rc=$?
cat "$official/results"
[[ $rc == 1 ]] && grep -Fxq '23/25 passed (0 matched errors)' "$official/results" || exit 1
for c in bikeDepartDelay bikeUnnecessaryEmergencyBraking; do
  python3 tools/fcddiff.py "$official/extended/bikes/$c" --all-differences > "$official/$c.diff"
done
if command -v sha256sum >/dev/null 2>&1; then
  checksum=(sha256sum -c)
elif command -v shasum >/dev/null 2>&1; then
  checksum=(shasum -a 256 -c)
else
  echo 'Checking the bicycle baselines requires sha256sum or shasum.' >&2
  exit 1
fi
(cd "$official" && "${checksum[@]}" - <<'HASHES'
17bae56a4e5fa95c89a87749e57edfe73ffe432d331035f30d7bb2fb737fb143  bikeDepartDelay.diff
472b2421e8b6ab0fa571e2f7cf954a8bd9883bd682176bf55e0e938a06b2eb3a  bikeUnnecessaryEmergencyBraking.diff
HASHES
)

if [[ -z $B ]]; then
  echo 'SKIPPED (PSIM2_BEST_DIR not set)'
elif (( nosub )); then
  echo 'SKIPPED (BeST cut-outs disabled by --no-sub)'
else
  for c in "sub46 310743775,29270503,21432805 1.5" "subT1 $T1 1.5" "subT2 $T2 1.5" "subR1 3243718354,1668717174 2"; do
    read -r name junctions period <<< "$c"
    echo "$name"
    python3 tools/subcheck.py "$name" "$B/berlin.net.xml" "$junctions" --period "$period" --hops 2 --end 1800 --sumo-order --check
    trips "$name"
  done
  R="$B/cutouts/mitte_bus.rou.xml"
  echo subB1
  python3 tools/subcheck.py subB1 "$B/berlin.net.xml" - --hops 1 --period 1.5 --end 1300 --sumo-order \
    --stops-file "$B/berlin_bus_stops.add.xml" --bus-period 90 --bus "$R:165_10305:-176031303#0:-625647965#0" \
    --bus "$R:265_10882:-176031303#0:328426251" --check
  trips subB1
  python3 tools/edgecheck.py sub46 --end 1800 --check
  python3 tools/edgecheck.py subB1 --end 1300 --check
fi
echo edit1
python3 tools/editcheck.py edit1 lc_prio2 --end 1500 --sumo-order --check
