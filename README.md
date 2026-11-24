# psim2

[![Build and test](https://github.com/Xiaoyu-0906/psim2/actions/workflows/ci.yml/badge.svg)](https://github.com/Xiaoyu-0906/psim2/actions/workflows/ci.yml)
[![License: EPL-2.0](https://img.shields.io/badge/license-EPL--2.0-blue.svg)](LICENSE)

psim2 is a multithreaded microscopic traffic simulator for road traffic, public
transport, pedestrians, and rail. It reimplements selected behavioral models
from [Eclipse SUMO 1.28.0](https://github.com/eclipse-sumo/sumo/tree/v1_28_0)
in C++17 on top of contiguous arrays and phase-parallel time steps, with only
POSIX threads and `zlib` as dependencies. Output does not depend on the thread
count: 1-thread and 4-thread runs produce byte-identical results.

This is a personal experiment (`v0.1-experimental`) and is not affiliated with
the Eclipse SUMO project or DLR. The `.psim2` scene format and command-line
options may still change, and parts of SUMO are not implemented yet (see
[Limitations](#limitations)).

## Building

Supported on Linux and macOS. Requirements: `make`, a C++17 compiler (`clang++`
is selected by default on macOS, `g++` on Linux; override with `CXX`), POSIX
threads, and `zlib` development headers.

```sh
make -C psim2 -j4
```

This builds `psim2/psim2`. Other Makefile targets are `psim2-tsan`
(ThreadSanitizer), `psim2-debug` (ASan + UBSan), and `clean`.

## Quick start

`examples/grid3x3/` contains a preconverted 3×3 urban grid (113 vehicles), so
you can run psim2 without installing SUMO:

```sh
mkdir -p /tmp/grid
./psim2/psim2 examples/grid3x3/scene.psim2 --threads 4 --fcd --out /tmp/grid/
wc -l /tmp/grid/fcd.csv /tmp/grid/tripinfo.csv
```

Expected line counts: 7,244 in `fcd.csv` (7,243 vehicle states + header) and
114 in `tripinfo.csv` (113 completed trips + header).

`--out` is a filename prefix rather than a directory path, so make sure the
directory exists and keep the trailing `/`.

## Usage

Converting SUMO XML scenarios requires Python 3.12+:

```sh
python3 tools/psim2scen.py net.net.xml routes.rou.xml scene.psim2 --full-net
```

Without `--full-net`, the network is trimmed to the junctions used by the
routes. Run `python3 tools/psim2scen.py --help` for additional files, time
bounds, and overrides such as `--sigma` and `--speeddev`.

Run a converted scene:

```sh
mkdir -p results
./psim2/psim2 scene.psim2 --threads 8 --seed 1 --out results/
```

Common options:

| Option | Description |
|---|---|
| `--threads N` | Number of worker threads |
| `--seed N` | Random seed |
| `--begin S`, `--end S` | Simulation start and end time in seconds |
| `--out PREFIX` | Output file prefix |
| `--fcd` | Write per-step vehicle states (`fcd.csv`, plus `personfcd.csv` if persons are present) |
| `--gzip` | Gzip-compress large per-step outputs |
| `--edits FILE` | Apply scheduled lane speed or permission edits between steps |
| `--sumo-order` | Run in SUMO's sequential processing order for step-by-step comparison |

Many SUMO flags are accepted under their original names (`--pedestrian.*`,
`--device.rerouting.*`, `--weights.*`, `--time-to-teleport`, etc.); see
[psim2/main.cpp](psim2/main.cpp) for the full list.

Default outputs are `tripinfo.csv`, `summary.csv`, `summary.txt`,
`lanechange.csv` (disable with `--no-lc-output`), and `collisions.csv`. Other
outputs (`fcd.csv`, `edgedata.csv`, `stopinfo.csv`, `personinfo.csv`, and rail
signal XML files) are written when their options or scene features are active.

An `--edits` file has one edit per line (`TIME KIND LANE VALUE`):

```text
120 LANE_SPEED A0A1_0 8.33
300 LANE_PERMISSIONS A1A0_0 passenger,bicycle
```

## Implemented features

- **Roads and junctions**: internal lanes and junctions, conflict geometry
  (`foe links` / `foe lanes`), priority, right-before-left, unregulated,
  dead-end, and zipper junctions, static and actuated traffic lights (with
  automatic loop detectors), jam/yield/wrong-lane teleports, and junction
  collision checks.
- **Driving models**: 17 SUMO car-following models (`Krauss`, `KraussOrig1`,
  `KraussPS`, `KraussX`, `SmartSK`, `Daniel1`, `PWagner2009`, `IDM`, `IDMM`,
  `EIDM`, `BKerner`, `Wiedemann`, `W99`, `ACC`, `CACC`, `Rail`, `NaSch`),
  ballistic updates, action step lengths, `LC2013` lane-changing, and indirect
  bicycle turns.
- **Public transport and parking**: bus stops (`duration`, `until`, `parking`,
  `parkingLength`), on-road and off-road `parkingArea` with reservations and
  badges, and `parkingAreaReroute` with memory and frustration.
- **Routing and scheduled edits**: pre-insertion and periodic Dijkstra
  rerouting, closing and variable-speed rerouters, and between-step lane speed
  / permission edits via `--edits` (signal program updates supported
  internally).
- **Pedestrians and intermodal**: `MSPModel_Striping` sidewalks, walking areas,
  signalized and unsignalized crossings, vehicle-pedestrian yielding,
  `personTrip` routing (walk, bus, car, bike), stop access stages, and
  container transport/tranship. Implementation notes (in Chinese):
  [docs/PEDESTRIANS.md](docs/PEDESTRIANS.md).
- **Rail**: `rail_signal` driveways (flank protection, bidirectional tracks,
  sidings, moving block), rail crossings, bidirectional reversal, `Rail`
  traction/resistance curves, `RailwayRouter`, split/join, overtaking and
  station rerouting, deadlock detection, and rail block/vehicle/deadlock XML
  outputs.

Each step runs as barrier-separated phases (`planMove` → lock-free bucketed
junction approach registration → `executeMove` → merge → graph-colored parallel
lane-changing → insertion), with 3-wave parallel pedestrian stepping and
speculative parallel rail signal updates. Random draws use a stateless
counter-based hash instead of a shared stream so results stay identical across
thread counts.

## Validation

### Berlin Mitte full day (BeST v2)

Compared on 2026-11-24 against official Eclipse SUMO 1.28.0 using the
[BeST v2](https://www.dcaiti.tu-berlin.de/research/simulation/downloads/get/best-scenario-v2.zip)
Berlin Mitte scenario (0–86,400 s, 361,122 vehicles including 3,223 buses,
1,809 junctions, 281 traffic lights). Both engines ran 3 seeds (`1, 2, 3`) with
all stochasticity enabled (SUMO single-threaded, psim2 commit `61cd8885` on 4
threads) via `tools/bestrun.py` and `tools/bestcompare.py`.

| Metric | SUMO 1.28.0 (mean ± sd) | psim2 (mean ± sd) | Diff | z |
|---|---:|---:|---:|---:|
| Inserted vehicles | 361,122 ± 0 | 361,122 ± 0 | 0.00% | – |
| Teleports | 92.0 ± 6.1 | 86.0 ± 20.4 | -6.52% | -0.5 |
| Car trip duration (357.7k cars) | 414.90 ± 0.07 s | 414.97 ± 0.06 s | +0.02% | +1.2 |
| Car route length | 3,476.62 ± 0.02 m | 3,476.62 ± 0.02 m | +0.00% | +0.4 |
| Car time loss | 126.65 ± 0.08 s | 126.59 ± 0.08 s | -0.04% | -0.8 |
| Car waiting time | 56.20 ± 0.03 s | 56.17 ± 0.08 s | -0.06% | -0.6 |
| Bus trip duration (3,223 buses) | 1,149.09 ± 0.21 s | 1,149.32 ± 0.21 s | +0.02% | +1.4 |
| Bus time loss | 197.08 ± 0.86 s | 197.74 ± 0.30 s | +0.33% | +1.2 |
| Bus stop time | 492.57 ± 0.98 s | 492.20 ± 0.15 s | -0.08% | -0.6 |

Across 49,074 hourly edge cells (≥ 20 veh/h), inter-run RMS between psim2 and
SUMO matches SUMO's own seed-to-seed RMS:

| Edge-hour metric | SUMO–SUMO RMS | psim2–psim2 RMS | psim2–SUMO RMS | Bias |
|---|---:|---:|---:|---:|
| Flow, `entered` (mean 178.32) | 2.358 | 2.404 | 2.400 | -0.001 |
| Mean speed (mean 8.28 m/s) | 0.367 | 0.363 | 0.363 | +0.001 |
| Waiting time (mean 0.11 s) | 0.050 | 0.052 | 0.050 | -0.000 |

For individual cars (357,670 matched by ID), trip duration RMS is 57.7 s across
SUMO–SUMO, psim2–psim2, and psim2–SUMO (mean bias +0.07 s).

Runtime and memory on a 10-core Apple Silicon machine:

| Engine | Threads | Wall time (seeds 1 / 2 / 3) | Median wall | User CPU (mean) | Peak RSS |
|---|---:|---|---:|---:|---:|
| SUMO 1.28.0 | 1 | 1,490.2 / 1,480.6 / 1,479.9 s | 1,480.6 s | 1,356.7 s | 1.34–1.40 GB |
| psim2 | 4 | 127.4 / 129.9 / 363.3 s | 129.9 s (~11.4×) | 431.8 s (seeds 1–2) | 0.87–1.91 GB |

Notes on this measurement:
- The host was running background compilation during the test (1-min load
  average 9.4 at start, peaking at 33.4 during psim2 seed 3, 9.7 after SUMO),
  which caused the 363 s outlier on seed 3. Treat these timings as a rough
  loaded-machine comparison rather than an isolated benchmark.
- Full-day peak RSS includes `--stop-output` and buffered trip records for all
  361k vehicles. Right after loading Mitte, psim2 uses about 0.45 GB.

### SUMO 1.28 test suites

| Suite | Result | Notes |
|---|---|---|
| Rail TextTests | 321 / 323 | 314 exact matches + 7 matched errors (`--derandomize`, `--sumo-order`; 4 of 314 have no vehicle states; rail XML outputs also compared). |
| SL2015 sublane | 207 / 302 | Older run; lateral maneuver reservations and sublane lane-change output remain incomplete. Notes (Chinese): [docs/SUBLANE.md](docs/SUBLANE.md). |
| Ten other suites | 1,421 / 1,881 | Older run on a previous engine/harness version (includes 14 matched errors); mismatches are mainly unsupported features or stochastic demand. |

The 2 rail differences (`rail/tramwayLoop/basic` at 1,863 s and `track_closed`
at 2,286 s, where `tram8` is at 51.94 m / 6.19 m/s vs SUMO's 52.47 m / 6.71
m/s) are intentional: psim2's reversal logic counts only ordinary route edges,
whereas SUMO 1.28 also counts internal lanes. Because of these two cases, the
rail run of `tools/sumotest.py` currently exits with status 1.

## Testing

Standalone checks that do not require SUMO:

```sh
python3 tests/rail_regress.py psim2/psim2

make -C psim2 psim2-tsan
mkdir -p /tmp/grid-tsan
TSAN_OPTIONS=halt_on_error=1 ./psim2/psim2-tsan examples/grid3x3/scene.psim2 --threads 4 --fcd --out /tmp/grid-tsan/
```

[CI](.github/workflows/ci.yml) runs on Ubuntu (`g++`, `clang++`) and macOS
(`clang++`): warning-free `-Wall -Wextra` build, 1- vs 4-thread SHA-256 output
check on `examples/grid3x3`, `tests/rail_regress.py`, and 4-thread TSan on
macOS.

### Running SUMO comparison scripts

To run the SUMO 1.28 comparison harnesses, install the pinned packages in a
Python 3.12+ virtual environment and point to `tests/sumo` from the
[SUMO v1_28_0 source tree](https://github.com/eclipse-sumo/sumo/tree/v1_28_0/tests/sumo):

```sh
python3 -m pip install -r tools/requirements.txt
export SUMO_HOME="$(python3 -c 'from sumo import SUMO_HOME; print(SUMO_HOME)')"
export PSIM2_SUMO_BIN="$SUMO_HOME/bin"
export PSIM2_SUMO_TESTS=/path/to/sumo/tests/sumo

python3 tools/sumotest.py --expect-count 323 --derandomize rail
bash tools/cityregress.sh
bash tools/railbench.sh
```

- `cityregress.sh` checks generated networks, buses, edge statistics, SUMO's
  zipper and bicycle cases (two known bicycle diffs are checked against stored
  baseline reports), and a scheduled `--edits` case against TraCI (requires
  local loopback access). Set `PSIM2_BEST_DIR` to include five Berlin cut-outs,
  and `PSIM2_CITY_DIR` / `PSIM2_WORK` to override scratch paths.
- `railbench.sh` benchmarks a multithreaded rail grid against SUMO. It locates
  SUMO via `SUMO_HOME` or `$PYTHON`'s `sumo` package, with `PSIM2_SUMO_BIN` as
  an override.
- To rerun BeST Berlin Mitte, set `PSIM2_BEST_DIR` to the extracted BeST v2
  directory, convert the scene to `work/best/mitte.psim2`, run
  `python3 tools/bestrun.py {sumo|psim} <seed>`, and compare with
  `python3 tools/bestcompare.py --sumo 1,2,3 --psim 1,2,3`.
- `tools/fcddiff.py` diffs SUMO's `fcd.xml` against psim2's `fcd.csv` step by
  step (IDs, lanes, positions, speeds; tolerance `0.011`).

## Limitations

- Command-line batch simulator only: no GUI and no TraCI server.
- No live topology editing yet: `--edits` only supports lane speed limits and
  lane permissions (plus internal signal program updates), not adding/removing
  lanes or rebuilding junctions mid-run.
- Sublane (`SL2015`) support is partial (207/302 official cases).
- Not yet implemented: opposite-direction overtaking and two-way left-turn
  lanes (TWLT), taxis (`MSDevice_Taxi`), `delay_based` and `NEMA` traffic light
  types, parking maneuvers (`--parking.maneuver`), A* and CH routers, and PLEXE
  cooperative cruise control (`CC`).

## Plans

No fixed timeline, roughly in these directions:

- Implement the remaining SUMO features (including completing SL2015 sublane
  support and the missing components listed above).
- Add live hot-editing support for modifying road network topology during a
  running simulation.
- Further optimize multi-core scaling and simulation throughput.

## Repository layout

```text
psim2/             C++17 simulation engine and Makefile
examples/grid3x3/  Preconverted 3×3 grid (113 vehicles), runs without SUMO
tests/             Standalone railway regression scenes and rail_regress.py
tools/             SUMO XML converters (psim2net.py, psim2scen.py) and comparison scripts
docs/              Pedestrian and sublane notes (in Chinese)
```

## Issues & feedback

Since this is a personal experiment, bug reports, questions, and feedback via
GitHub Issues are welcome. If you find a case where psim2 behaves differently
from SUMO, attaching the input files (or converted `.psim2` scene) and the
command line makes it much easier to reproduce.

## License

Distributed under the [Eclipse Public License 2.0](LICENSE).

Most simulation models are adapted from Eclipse SUMO 1.28.0 (Copyright German
Aerospace Center (DLR) and contributors); see [NOTICE](NOTICE) for detailed
upstream attributions. The BeST v2 Berlin scenario (DCAITI / TU Berlin,
[CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)) was used for
validation only and is not included in this repository.
