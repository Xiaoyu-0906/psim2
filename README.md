# psim2 v0.1-experimental

psim2 is a parallel microscopic traffic simulator for road, pedestrian and rail networks. It implements selected Eclipse SUMO 1.28 models in approximately 35,000 lines of C++17. A preconverted 3 × 3 grid with 113 vehicles is included.

## Quick start

With Make, a C++17 compiler, POSIX threads and zlib development headers installed, run these commands from the repository root. The example runs directly on Linux and macOS without installing SUMO.

```sh
make -C psim2 -j4
out=$(mktemp -d)
./psim2/psim2 examples/grid3x3/scene.psim2 --threads 4 --fcd --out "$out/"
wc -l "$out/fcd.csv" "$out/tripinfo.csv"
```

The grid produces 7,243 vehicle states and 113 completed trips (7,244 and 114 lines including headers). Use a fresh output directory for each run. The final slash makes the output prefix a directory. The grid produced identical FCD and trip CSV bytes with one and four threads on the validation machine.

## Engine and input

The engine in `psim2/` loads the road network and demand, then advances the simulation in phases. Lane and edge tasks run on a worker pool. Movement and lane changes publish updates at phase boundaries. The `--sumo-order` option follows SUMO's processing order for stepwise comparisons. The runtime needs pthreads and zlib. Make selects Clang on macOS and GCC on Linux and accepts a `CXX` override.

The converters in `tools/` read SUMO XML. To build a new scene, run `python3 tools/psim2scen.py net.net.xml routes.rou.xml scene.psim2 --full-net`. The simulator models car following, lane changing, right of way, signals, public transport, rail operations and pedestrian movement. Model details and historical coverage are in the pedestrian and sublane notes. [Pedestrians](docs/PEDESTRIANS.md) (in Chinese) · [Sublanes](docs/SUBLANE.md) (in Chinese)

## Validation

| Dataset | Result | Run conditions |
| --- | --- | --- |
| SUMO 1.28 railway TextTests | **321/323** on the patched candidate. 314 cases agreed in the stepwise comparison and 7 matched SUMO's expected error. | Derandomized inputs and SUMO processing order. Four of the 314 had zero vehicle states. The harness also compares requested rail XML outputs. |
| Two release regressions | Both fail before the patches and pass after them. | Positive-speed waypoint insertion on the departure and following lanes. A 65 m train reversing across an internal lane on a 42 m branch. |
| Ten other regular TextTest suites | Historical run with **1,421/1,881** zero-difference markers, including 14 matched errors. | Prior engine and harness, with rail, sublane and project-specific synthetic cases counted separately. |
| SL2015 sublane suite | Historical run with **207/302** matching cases. | Experimental coverage, with lateral maneuver reservations and sublane lane-change output still incomplete. |

In the candidate railway run, `rail/tramwayLoop/basic` first diverged at 1,863 s and `rail/tramwayLoop/track_closed` at 2,286 s. In both cases SUMO recorded tram8 at 52.47 m and 6.71 m/s on the depot lane. The candidate recorded 51.94 m and 6.19 m/s. An isolated build reproduced both differences with the reversal patch alone. Its route-length check counts only ordinary route edges under a reversing train. SUMO 1.28 also counts internal lanes in this check when internal lanes are enabled. The exact railway gate currently exits 1.

The trajectory comparator checks IDs, lanes, positions and speeds step by step, with a tolerance of 0.011 for the numeric vehicle fields. The threaded grid check compares SHA-256 hashes of complete CSV files.

### Berlin Mitte

A previous full-day BeST v2 run (0–86,400 s, 361,122 vehicles including 3,223 buses) compared SUMO 1.28 and psim2 over three random seeds per engine. Seeds ran concurrently with other machine load and three threads per psim2 run. Reported wall times were approximately 5,100 s for SUMO and 570 s for psim2 (about 9× for that workload). Mean car trip time was 414.90 s versus 415.08 s (+0.04%). Mean bus trip time was 1,149.09 s versus 1,149.04 s (−0.004%, rounded to −0.00%). A separate psim2 memory-layout change reduced peak memory after loading Mitte from 1.28 GB to 0.45 GB. These are historical measurements. [BeST v2 dataset](https://www.dcaiti.tu-berlin.de/research/simulation/downloads/get/best-scenario-v2.zip)

## Running the checks

```sh
python3 tests/rail_regress.py psim2/psim2
make -C psim2 psim2-tsan
out=$(mktemp -d)
TSAN_OPTIONS=halt_on_error=1 ./psim2/psim2-tsan examples/grid3x3/scene.psim2 --threads 4 --fcd --out "$out/"
```

For SUMO comparisons, install the pinned packages in an activated Python 3.12 virtual environment and supply the `tests/sumo` directory from the SUMO v1_28_0 source tree. Set `PSIM2_SUMO_TESTS` to its absolute path or pass `--sumo-tests` to the TextTest runner. [SUMO 1.28 test tree](https://github.com/eclipse-sumo/sumo/tree/v1_28_0/tests/sumo)

```sh
python3 -m pip install -r tools/requirements.txt
export SUMO_HOME="$(python3 -c 'from sumo import SUMO_HOME; print(SUMO_HOME)')"
export PSIM2_SUMO_BIN="$SUMO_HOME/bin"
python3 tools/sumotest.py --expect-count 323 --derandomize rail  # currently 321/323 (exits 1); see Validation above
bash tools/cityregress.sh
```

The city runner checks generated networks, buses, edge statistics, the official zipper and bicycle cases, and a scheduled edit against a SUMO TraCI reference. The latter requires local loopback access. Two known bicycle discrepancies are checked against their full baseline difference reports. Set `PSIM2_BEST_DIR` to the extracted BeST directory to include five Berlin cut-outs. `PSIM2_CITY_DIR` and `PSIM2_WORK` can point to dedicated absolute scratch directories for city outputs and TextTest outputs respectively. Each city run prints its fresh output directory. The TextTest runner regenerates selected cases in its work directory.

The rail benchmark (`bash tools/railbench.sh`) uses `PYTHON` (default `python3`) to find the installed `sumo` package when `SUMO_HOME` is unset. To use a separate SUMO binary directory, set `PSIM2_SUMO_BIN`.

The CI workflow checks shell and Python syntax, BeST input diagnostics and zlib, builds with GCC and Clang on Ubuntu and Apple Clang on macOS, and checks the grid and both railway fixtures. The macOS job also runs a four-thread ThreadSanitizer smoke test. Run `make -C psim2 clean` to remove compiled products.

## v0.1 boundaries

SL2015 coverage is partial. Opposite-direction overtaking, taxi service, a TraCI server for psim2 and live topology changes are outside v0.1. The public `--edits` file accepts lane speed limits and lane permissions. The internal edit handler can also apply signal phase programs.

Distributed under the Eclipse Public License 2.0. [License](LICENSE) · [Attributions](NOTICE)
