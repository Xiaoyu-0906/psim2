"""Exercise insertion on both waypoint lanes and reversal across an internal lane."""
import argparse
import csv
from pathlib import Path
import subprocess
import sys
import tempfile


def rows(directory, name):
    with (directory / name).open(newline="") as source:
        return list(csv.DictReader(source))


def check(case, directory):
    trips = rows(directory, "tripinfo.csv")
    states = rows(directory, "fcd.csv")
    if case == "waypoint":
        # Current-lane stop at 81 m, then a second train 2 m before the next lane.
        starts = {r["id"]: (r["time"], r["speed"]) for r in reversed(states)}
        return (starts == {"train": ("0.00", "12.00"), "next": ("20.00", "12.00")}
                and len(trips) == 2 and all(r["teleports"] == "0" for r in trips))
    # A 65 m train stops on the 42 m branch, with its tail across the junction.
    # On reversal the former tail becomes the front on the returning main track.
    return (len(trips) == 1 and trips[0]["id"] == "train" and trips[0]["teleports"] == "0"
            and trips[0]["arrival"] == "36.00"
            and any(r["time"] == "18.00" and r["lane"] == "-ab_0" for r in states))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path)
    args = parser.parse_args()
    failed = False
    for case in ("waypoint", "reversal"):
        with tempfile.TemporaryDirectory(prefix="psim2-rail-") as tmp:
            directory = Path(tmp)
            scene = Path(__file__).parent / "rail" / (case + ".psim2")
            subprocess.run([args.binary.resolve(), scene, "--end", "180", "--fcd", "--out", tmp + "/"], check=True)
            ok = check(case, directory)
            print(f"{case} {'PASS' if ok else 'FAIL'}")
            failed |= not ok
    return int(failed)


if __name__ == "__main__":
    sys.exit(main())
