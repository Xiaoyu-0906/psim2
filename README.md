# psim2 (`v0.1-experimental`)

**psim2** is a parallel, deterministic microscopic traffic simulator for **road, public transport, pedestrian, and rail networks**. It re-implements selected [Eclipse SUMO 1.28.0](https://github.com/eclipse-sumo/sumo/tree/v1_28_0) behavioral models in ~35,000 lines of dependency-free C++17 (requiring only POSIX threads and `zlib`) on a contiguous, phase-parallel memory layout.

- **Deterministic across thread counts**: 1-thread and 4-thread runs produce byte-identical `fcd.csv` and `tripinfo.csv` outputs (checked via SHA-256 in [CI](.github/workflows/ci.yml)) and pass 4-thread ThreadSanitizer cleanly.
- **Validated against SUMO 1.28.0**: reproduces full-day Berlin Mitte (BeST v2, 361,122 vehicles, 3 seeds vs 3 seeds) within SUMO's own seed-to-seed variance at **~11.4× median wall-clock speedup** on 4 threads (see [Berlin Mitte validation](#berlin-mitte-best-v2-validation)).
- **Self-contained quick start**: ships with a preconverted 3 × 3 urban grid (`examples/grid3x3/scene.psim2`) that compiles and runs on Linux and macOS without installing SUMO.

> **Experimental status (`v0.1-experimental`)**: `psim2` is an offline batch simulation engine and SUMO XML converter. It does not include a GUI, TraCI server, live topology editing, taxis, opposite-direction overtaking, or complete SL2015 sublane coverage (see [Known Limitations](#known-limitations)).

---

## Quick Start

**Requirements**: `make`, a C++17 compiler (`g++` or `clang++`; `Makefile` selects Clang on macOS and GCC on Linux and accepts a `CXX` override), POSIX threads, and `zlib` development headers. No SUMO installation is needed for the bundled grid example.

```sh
make -C psim2 -j4
out=$(mktemp -d)
./psim2/psim2 examples/grid3x3/scene.psim2 --threads 4 --fcd --out "$out/"
wc -l "$out/fcd.csv" "$out/tripinfo.csv"
```

Expected output: **`7,244`** lines in `fcd.csv` (7,243 vehicle states + header) and **`114`** lines in `tripinfo.csv` (113 completed trips + header). Use a fresh output directory with a trailing `/` so `--out` writes files inside that directory. Run `make -C psim2 clean` to remove build artifacts.

Run the built-in railway regression fixtures and 4-thread ThreadSanitizer smoke test:

```sh
python3 tests/rail_regress.py psim2/psim2
make -C psim2 psim2-tsan
out=$(mktemp -d)
TSAN_OPTIONS=halt_on_error=1 ./psim2/psim2-tsan examples/grid3x3/scene.psim2 --threads 4 --fcd --out "$out/"
```

---

## Implemented Capabilities

| Domain | Capabilities in `v0.1` |
|---|---|
| **Roads & Junctions** | Internal lanes and junctions, conflict geometry (`foe links` / `foe lanes`), priority / right-before-left / unregulated / dead-end / zipper junctions, static and actuated traffic lights (with automatic loop detectors), jam/yield/wrong-lane teleports, junction collision checks. |
| **Car-Following & Lane-Changing** | 17 SUMO car-following models (`Krauss`, `KraussOrig1`, `KraussPS`, `KraussX`, `SmartSK`, `Daniel1`, `PWagner2009`, `IDM`, `IDMM`, `EIDM`, `BKerner`, `Wiedemann`, `W99`, `ACC`, `CACC`, `Rail`, `NaSch`), ballistic updates, action step lengths, full `LC2013` lane-changing, indirect bicycle turns. |
| **Public Transport & Parking** | Scheduled bus stops (`duration`, `until`, `parking=true/false`, `parkingLength`), off-road and on-road `parkingArea`, reservations, badges, and full `parkingAreaReroute` search with memory and frustration. |
| **Routing & Scheduled Edits** | Pre-insertion and periodic Dijkstra rerouting, closing/variable-speed rerouters, and between-step `--edits` files for lane speed limits and lane permissions (signal program updates supported internally). |
| **Pedestrians & Intermodal** | `MSPModel_Striping` sidewalks, walking areas, signalized and unsignalized crossings, vehicle–pedestrian yielding, `personTrip` intermodal routing (walk / bus / car / bike), stop access stages, and container transport/tranship. [Pedestrian notes (Chinese)](docs/PEDESTRIANS.md) |
| **Rail Operations** | `rail_signal` driveways (flank protection, bidirectional tracks, sidings, moving block), rail crossings, bidirectional train reversal, `Rail` traction/resistance curves, `RailwayRouter`, split/join, overtaking and station rerouting, deadlock detection, and rail block/vehicle/deadlock XML outputs. |
| **Parallel Architecture** | Phase-barrier execution (`planMove` → lock-free bucketed junction approaches → `executeMove` → merge → graph-colored parallel lane-changing → insertion), 3-wave parallel pedestrian stepping, speculative parallel rail signal updates, counter-based RNG for thread-count determinism, and `--sumo-order` for stepwise SUMO comparison. |

---

## Validation & Benchmarks

### Summary Matrix

| Suite / Check | Result | Conditions & Notes |
|---|---|---|
| **GitHub Actions CI** | Pass on Ubuntu (`g++`, `clang++`) & macOS (`clang++`) | Shell/Python syntax checks, BeST input diagnostics, `zlib` check, zero-warning `-Wall -Wextra` build, 1 vs 4 thread SHA-256 byte identity on `examples/grid3x3`, `tests/rail_regress.py`, and 4-thread TSan on macOS. |
| **Berlin Mitte full-day (BeST v2)** | **Statistically indistinguishable from SUMO 1.28.0** | **2026-11-24 rerun** on release commit `61cd8885`: 361,122 vehicles, 3 seeds vs 3 seeds, **+0.02%** car & bus mean trip duration, **11.4× median wall-clock speedup** (details below). |
| **Release railway regressions** | **2 / 2 PASS** | Positive-speed waypoint insertion (departure & following lanes) and 65 m train reversal across an internal lane on a 42 m branch (`tests/rail_regress.py`). |
| **SUMO 1.28 railway TextTests** | **321 / 323** (`314` exact + `7` matched errors) | Derandomized `--sumo-order` comparison (4 of the 314 had zero vehicle states; rail XML outputs also compared). The 2 remaining diffs (`rail/tramwayLoop/basic` at 1,863 s and `track_closed` at 2,286 s; `tram8` at 51.94 m / 6.19 m/s vs SUMO's 52.47 m / 6.71 m/s) stem intentionally from the reversal patch counting only ordinary route edges while SUMO 1.28 also counts internal lanes. The exact gate currently exits 1. |
| **Ten other regular TextTest suites** | **1,421 / 1,881** zero-difference markers | Historical run on prior engine/harness (includes 14 matched errors; rail, sublane, and synthetic cases counted separately); non-matches are mainly unsupported features or stochastic demand. |
| **SL2015 sublane suite** | **207 / 302** matching cases | Historical partial coverage; lateral maneuver reservations and sublane lane-change output remain incomplete. [Sublane notes (Chinese)](docs/SUBLANE.md) |

---

### Berlin Mitte (BeST v2) Validation

**2026-11-24 full-day comparison** (`0–86,400 s`, **361,122 vehicles** including **3,223 buses**, 1,809 junctions, 281 traffic lights) between official **Eclipse SUMO 1.28.0** and release commit **`61cd8885`** (`psim2` at 4 threads, sequential; SUMO 1.28.0 single-threaded, 3 seeds in parallel; all stochasticity enabled, seeds `1, 2, 3`, via `tools/bestrun.py` + `tools/bestcompare.py`).

#### Statistical Agreement (3 seeds vs 3 seeds)

| Metric | SUMO 1.28.0 (mean ± sd) | psim2 `61cd8885` (mean ± sd) | Diff | z-score |
|---|---:|---:|---:|---:|
| Inserted vehicles | `361,122 ± 0` | `361,122 ± 0` | `0.00%` | — |
| Teleports (total) | `92.0 ± 6.1` | `86.0 ± 20.4` | `-6.52%` | `-0.5` |
| Car trip duration (357.7k cars) | `414.90 ± 0.07 s` | `414.97 ± 0.06 s` | **`+0.02%`** | `+1.2` |
| Car route length | `3,476.62 ± 0.02 m` | `3,476.62 ± 0.02 m` | **`+0.00%`** | `+0.4` |
| Car time loss | `126.65 ± 0.08 s` | `126.59 ± 0.08 s` | `-0.04%` | `-0.8` |
| Car waiting time | `56.20 ± 0.03 s` | `56.17 ± 0.08 s` | `-0.06%` | `-0.6` |
| Bus trip duration (3,223 buses) | `1,149.09 ± 0.21 s` | `1,149.32 ± 0.21 s` | **`+0.02%`** | `+1.4` |
| Bus time loss | `197.08 ± 0.86 s` | `197.74 ± 0.30 s` | `+0.33%` | `+1.2` |
| Bus stop time | `492.57 ± 0.98 s` | `492.20 ± 0.15 s` | `-0.08%` | `-0.6` |

- **49,074 hourly edge cells** (≥20 veh/h): inter-run RMS between `psim2` and SUMO matches SUMO's own seed-to-seed RMS:
  - Flow (`entered`, mean 178.32): SUMO–SUMO `2.358`, psim2–psim2 `2.404`, **psim2–SUMO `2.400`** (bias `-0.001`)
  - Mean speed (mean 8.28 m/s): SUMO–SUMO `0.367`, psim2–psim2 `0.363`, **psim2–SUMO `0.363`** (bias `+0.001`)
  - Waiting time (mean 0.11 s): SUMO–SUMO `0.050`, psim2–psim2 `0.052`, **psim2–SUMO `0.050`** (bias `-0.000`)
- **Per-vehicle car trip durations** (357,670 matched cars): RMS difference is **`57.7 s`** across SUMO–SUMO, psim2–psim2, and **psim2–SUMO** (mean bias `+0.07 s`).

#### Wall-Clock Speed & Memory (2026-11-24, Apple Silicon 10-core)

| Engine | Threads | Wall Time per Seed (`1 / 2 / 3`) | Median Wall | User CPU (mean) | Full-Run Peak RSS |
|---|---:|---|---:|---:|---:|
| **SUMO 1.28.0** | 1 | `1,490.2 s` / `1,480.6 s` / `1,479.9 s` | `1,480.6 s` | `1,356.7 s` | `1.34 – 1.40 GB` |
| **psim2 `61cd8885`** | 4 | `127.4 s` / `129.9 s` / `363.3 s`* | **`129.9 s`** (**11.4×**) | `431.8 s` (seeds 1–2) | `0.87 – 1.91 GB`† |

- \*Measured under concurrent background compilation load (1-min load average `9.4` at start, peaking at `33.4` during `psim2` seed 3, `9.7` after SUMO); treat these timings as a loaded-machine measurement rather than an isolated idle microbenchmark.
- †Full-day peak RSS includes `--stop-output` and buffered per-vehicle trip records across all 361k vehicles; historical post-load memory immediately after loading Mitte is `0.45 GB` (down from `1.28 GB` before cold-field separation).
- BeST v2 dataset (CC BY 4.0, DCAITI / TU Berlin): [best-scenario-v2.zip](https://www.dcaiti.tu-berlin.de/research/simulation/downloads/get/best-scenario-v2.zip).

---

## Converting SUMO Scenarios & Running Full Verification

Convert any standard SUMO network and route file into a `.psim2` scene:

```sh
python3 tools/psim2scen.py net.net.xml routes.rou.xml scene.psim2 --full-net
```

To run the SUMO 1.28 comparison harnesses, install the pinned packages in an activated Python 3.12+ virtual environment and provide the `tests/sumo` directory from the [SUMO v1_28_0 source tree](https://github.com/eclipse-sumo/sumo/tree/v1_28_0/tests/sumo):

```sh
python3 -m pip install -r tools/requirements.txt
export SUMO_HOME="$(python3 -c 'from sumo import SUMO_HOME; print(SUMO_HOME)')"
export PSIM2_SUMO_BIN="$SUMO_HOME/bin"
export PSIM2_SUMO_TESTS="/path/to/sumo/tests/sumo"

python3 tools/sumotest.py --expect-count 323 --derandomize rail  # currently 321/323 (exits 1); see Summary Matrix
bash tools/cityregress.sh                                        # set PSIM2_BEST_DIR to include 5 Berlin cut-outs
bash tools/railbench.sh                                          # multi-thread rail grid benchmark vs SUMO
```

- `tools/cityregress.sh` checks generated networks, buses, edge statistics, official zipper and bicycle cases (with two known bicycle discrepancies verified against their full baseline diff reports), and a scheduled `--edits` case against a SUMO TraCI reference (requires local loopback access). Set `PSIM2_CITY_DIR` and `PSIM2_WORK` to override scratch output directories.
- `tools/railbench.sh` uses `PYTHON` (default `python3`) to locate the installed `sumo` package when `SUMO_HOME` is unset; set `PSIM2_SUMO_BIN` to point to a custom SUMO binary directory.
- For full-day BeST Berlin Mitte runs, set `PSIM2_BEST_DIR` to the extracted BeST directory, convert `work/best/mitte.psim2`, then run `python3 tools/bestrun.py {sumo|psim} <seed>` and `python3 tools/bestcompare.py --sumo 1,2,3 --psim 1,2,3`.

---

## Repository Layout

```text
psim2/               C++17 parallel engine (~35k LOC, 39 source/header files) + Makefile
examples/grid3x3/    Preconverted 3×3 urban grid (113 vehicles) — runs without SUMO
tests/               Standalone railway regression fixtures and runner (rail_regress.py)
tools/               SUMO XML converters (psim2net.py, psim2scen.py) and comparison suites
docs/                Pedestrian and sublane implementation notes (in Chinese)
.github/workflows/   Cross-platform CI (Ubuntu g++/clang++, macOS clang++ + TSan)
```

---

## Known Limitations

- **No GUI / No TraCI server**: offline batch CLI simulator only.
- **No live topology editing**: `--edits` supports lane speed limits and lane permissions (plus internal signal phase updates), not adding/removing lanes or rebuilding junctions mid-run.
- **Partial sublane (`SL2015`) support**: 207/302 official cases match; lateral maneuver reservations and sublane lane-change output remain incomplete.
- **Unimplemented SUMO features**: opposite-direction overtaking and two-way left-turn lanes (TWLT), taxi dispatch (`MSDevice_Taxi`), `delay_based` / `NEMA` TLS algorithms, parking maneuvers (`--parking.maneuver`), A* / CH routers, and PLEXE cooperative cruise control (`CC`).
- **Trajectory comparator**: `tools/fcddiff.py` checks IDs, lanes, positions, and speeds step-by-step with a numeric tolerance of `0.011`.

---

## License & Attributions

Distributed under the **Eclipse Public License 2.0**. See [LICENSE](LICENSE) and [NOTICE](NOTICE) for full upstream attributions to **Eclipse SUMO 1.28.0** (Copyright DLR and contributors) and the **BeST v2** Berlin scenario (CC BY 4.0).
