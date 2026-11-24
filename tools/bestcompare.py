"""Compare BeST Mitte runs of psim2 with SUMO runs (tools/bestrun.py).

Usage: bestcompare.py [--sumo 1,2,3] [--psim 1,2,3] [--json out.json]

Every psim2-SUMO difference is shown next to the spread between runs with
different seeds: psim2 is statistically consistent with SUMO when the distance
between psim2 and SUMO runs is not larger than the distance between two SUMO
(or two psim2) runs.
"""
import argparse
import csv
import json
import math
import os
import statistics
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
WORK = os.path.join(os.path.dirname(HERE), "work", "best")
TRIP_KEYS = ["duration", "routeLength", "timeLoss", "waitingTime", "departDelay", "stopTime"]


def read_sumo(seed):
    d = os.path.join(WORK, "S%d" % seed)
    trips = {}
    for _, el in ET.iterparse(os.path.join(d, "tripinfo.xml")):
        if el.tag == "tripinfo":
            trips[el.get("id")] = dict(bus=el.get("vType") != "DefaultVehicle",
                                       depart=float(el.get("depart")),
                                       **{k: float(el.get(k, "0")) for k in TRIP_KEYS})
            el.clear()
    stats = ET.parse(os.path.join(d, "stats.xml")).getroot()
    tele = stats.find("teleports")
    veh = stats.find("vehicles")
    info = dict(teleports=int(tele.get("total")), teleJam=int(tele.get("jam")), teleYield=int(tele.get("yield")),
                teleWrongLane=int(tele.get("wrongLane")), inserted=int(veh.get("inserted")))
    summary = []
    for _, el in ET.iterparse(os.path.join(d, "summary.xml")):
        if el.tag == "step":
            t = float(el.get("time"))
            if t % 60 == 0:
                summary.append(dict(time=t, running=int(el.get("running")), waiting=int(el.get("waiting")),
                                    halting=int(el.get("halting")), meanSpeed=float(el.get("meanSpeed")),
                                    arrived=int(el.get("arrived"))))
            el.clear()
    edges = {}
    begin = None
    for ev, el in ET.iterparse(os.path.join(d, "edgedata.xml"), events=("start", "end")):
        if ev == "start" and el.tag == "interval":
            begin = float(el.get("begin"))
        elif ev == "end" and el.tag == "edge":
            edges[(begin, el.get("id"))] = (int(el.get("entered", "0")), float(el.get("sampledSeconds", "0")),
                                            float(el.get("speed", "0")) * float(el.get("sampledSeconds", "0")),
                                            float(el.get("waitingTime", "0")))
            el.clear()
    wall = float(open(os.path.join(d, "wall.txt")).read())
    return dict(trips=trips, info=info, summary=summary, edges=edges, wall=wall)


def read_psim(seed):
    d = os.path.join(WORK, "P%d" % seed)
    trips = {}
    with open(os.path.join(d, "tripinfo.csv")) as f:
        for r in csv.DictReader(f):
            trips[r["id"]] = dict(bus=not r["id"].replace(".", "").isdigit() and "." in r["id"],
                                  depart=float(r["depart"]), **{k: float(r[k]) for k in TRIP_KEYS})
    s = dict(l.split() for l in open(os.path.join(d, "summary.txt")) if len(l.split()) == 2)
    info = dict(teleports=int(s["teleported"]), teleJam=int(s["teleJam"]), teleYield=int(s["teleYield"]),
                teleWrongLane=int(s["teleWrongLane"]), inserted=int(s["inserted"]))
    summary = []
    with open(os.path.join(d, "summary.csv")) as f:
        for r in csv.DictReader(f):
            summary.append(dict(time=float(r["time"]), running=int(r["running"]), waiting=int(r["waiting"]),
                                halting=int(r["halting"]), meanSpeed=float(r["meanSpeed"]), arrived=int(r["arrived"])))
    edges = {}
    with open(os.path.join(d, "edgedata.csv")) as f:
        for r in csv.DictReader(f):
            edges[(float(r["begin"]), r["edge"])] = (int(r["entered"]), float(r["sampledSeconds"]),
                                                    float(r["travelledDistance"]), float(r["waitingTime"]))
    return dict(trips=trips, info=info, summary=summary, edges=edges, wall=float(s["wall"]))


def trip_stats(trips, bus):
    rows = [t for t in trips.values() if t["bus"] == bus]
    out = dict(n=len(rows))
    for k in TRIP_KEYS:
        vals = sorted(t[k] for t in rows)
        if not vals:
            continue
        out[k] = statistics.fmean(vals)
        if k in ("duration", "timeLoss", "waitingTime"):
            out[k + "_p50"] = vals[len(vals) // 2]
            out[k + "_p90"] = vals[int(len(vals) * 0.9)]
    return out


def hourly(summary, key):
    by = {}
    for r in summary:
        if r[key] >= 0:
            by.setdefault(int(r["time"] // 3600), []).append(r[key])
    return {h: statistics.fmean(v) for h, v in by.items()}


def edge_table(edges, min_entered):
    """(hour, edge) -> (entered, mean speed) for edge-hours with traffic."""
    out = {}
    for key, (entered, sampled, dist, wait) in edges.items():
        if sampled > 0 or entered > 0:
            out[key] = (entered, dist / sampled if sampled > 0 else 0., wait / 3600.0)
    return out


def rms(xs):
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else float("nan")


def edge_rms(tabs_a, tabs_b, keys, idx, same):
    """mean RMS difference over run pairs (a_i, b_j); same=True: pairs within one list"""
    out = []
    for i in range(len(tabs_a)):
        for j in range(len(tabs_b)):
            if same and j <= i:
                continue
            out.append(rms([tabs_a[i].get(k, (0, 0, 0))[idx] - tabs_b[j].get(k, (0, 0, 0))[idx] for k in keys]))
    return statistics.fmean(out) if out else float("nan")


def compare_edges(S, P):
    """RMS differences of hourly edge values within SUMO, within psim2 and across."""
    st = [edge_table(s["edges"], 0) for s in S]
    pt = [edge_table(p["edges"], 0) for p in P]
    keys = set(st[0])
    for t in st[1:]:
        keys &= set(t)
    keys = [k for k in keys if min(t[k][0] for t in st) >= 20]  # edge-hours with >= 20 vehicles in every SUMO run
    res = {"edge_hours": len(keys)}
    for idx, name in ((0, "entered"), (1, "speed"), (2, "waiting")):
        smean = {k: statistics.fmean(t[k][idx] for t in st) for k in keys}
        pmean = {k: statistics.fmean(t.get(k, (0, 0, 0))[idx] for t in pt) for k in keys}
        res[name] = dict(mean=statistics.fmean(smean.values()),
                         sumo_vs_sumo=edge_rms(st, st, keys, idx, True),
                         psim_vs_psim=edge_rms(pt, pt, keys, idx, True),
                         psim_vs_sumo=edge_rms(pt, st, keys, idx, False),
                         psim_bias=statistics.fmean(pmean[k] - smean[k] for k in keys))
    return res


def paired_trips(S, P):
    """Mean and RMS of per-vehicle car trip duration differences over run pairs."""
    common = set(S[0]["trips"])
    for r in S[1:] + P:
        common &= set(r["trips"])
    common = [k for k in common if not S[0]["trips"][k]["bus"]]

    def pairs(A, B, same):
        out = []
        for i in range(len(A)):
            for j in range(len(B)):
                if same and j <= i:
                    continue
                d = [A[i]["trips"][k]["duration"] - B[j]["trips"][k]["duration"] for k in common]
                out.append((statistics.fmean(d), rms(d)))
        return out
    ss, pp, ps = pairs(S, S, True), pairs(P, P, True), pairs(P, S, False)
    f = lambda xs, i: statistics.fmean(x[i] for x in xs) if xs else float("nan")
    return dict(n=len(common), sumo_vs_sumo_rms=f(ss, 1), psim_vs_psim_rms=f(pp, 1), psim_vs_sumo_rms=f(ps, 1),
                psim_vs_sumo_mean=f(ps, 0))


def fmt_row(name, svals, pvals):
    ms, mp = statistics.fmean(svals), statistics.fmean(pvals)
    ss = statistics.stdev(svals) if len(svals) > 1 else 0.0
    sp = statistics.stdev(pvals) if len(pvals) > 1 else 0.0
    rel = (mp - ms) / ms * 100 if ms else float("nan")
    se = math.sqrt(ss * ss / len(svals) + sp * sp / len(pvals))
    z = (mp - ms) / se if se > 0 else float("nan")
    return "  %-18s SUMO %10.2f ± %-7.2f psim2 %10.2f ± %-7.2f diff %+6.2f%%  z %+6.1f" % (name, ms, ss, mp, sp, rel, z)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sumo", default="1,2,3")
    ap.add_argument("--psim", default="1")
    ap.add_argument("--json")
    a = ap.parse_args()
    S = [read_sumo(int(x)) for x in a.sumo.split(",")]
    P = [read_psim(int(x)) for x in a.psim.split(",")]
    report = {}
    print("wall time: SUMO %s s, psim2 %s s" % (", ".join("%.0f" % s["wall"] for s in S), ", ".join("%.0f" % p["wall"] for p in P)))
    print("vehicles and teleports (mean ± sd over seeds; z = difference / its standard error)")
    for k in ("inserted", "teleports", "teleJam", "teleYield", "teleWrongLane"):
        print(fmt_row(k, [s["info"][k] for s in S], [p["info"][k] for p in P]))
    report["info"] = dict(sumo=[s["info"] for s in S], psim=[p["info"] for p in P])
    for bus in (False, True):
        print("trips (%s)" % ("buses" if bus else "cars"))
        st = [trip_stats(s["trips"], bus) for s in S]
        pt = [trip_stats(p["trips"], bus) for p in P]
        for k in pt[0]:
            print(fmt_row(k, [x[k] for x in st], [x[k] for x in pt]))
        report["trips_bus" if bus else "trips_car"] = dict(sumo=st, psim=pt)
    print("hourly means of the summary (SUMO mean / psim2 mean over seeds)")
    keys = ("running", "waiting", "halting", "meanSpeed")
    hs = {k: [hourly(s["summary"], k) for s in S] for k in keys}
    hp = {k: [hourly(p["summary"], k) for p in P] for k in keys}
    print("  hour " + " ".join("%17s" % k for k in keys))
    for h in sorted(hp["running"][0]):
        cells = []
        for k in keys:
            sv = statistics.fmean(x.get(h, 0) for x in hs[k])
            pv = statistics.fmean(x.get(h, 0) for x in hp[k])
            cells.append("%8.1f/%8.1f" % (sv, pv))
        print("  %4d " % h + " ".join(cells))
    report["hourly"] = dict(sumo=hs, psim=hp)
    print("hourly edge data (edge-hours with >= 20 vehicles in every SUMO run): RMS of differences between runs")
    ec = compare_edges(S, P)
    print("  %d edge-hours" % ec["edge_hours"])
    for k in ("entered", "speed", "waiting"):
        e = ec[k]
        print("  %-8s mean %8.2f  SUMO-SUMO %7.3f  psim2-psim2 %7.3f  psim2-SUMO %7.3f  psim2 bias %+7.3f" % (
            k, e["mean"], e["sumo_vs_sumo"], e["psim_vs_psim"], e["psim_vs_sumo"], e["psim_bias"]))
    report["edges"] = ec
    pt = paired_trips(S, P)
    print("per-vehicle car trip durations (%d cars arrived in all runs), RMS of differences between runs" % pt["n"])
    print("  SUMO-SUMO %.1f s  psim2-psim2 %.1f s  psim2-SUMO %.1f s  (mean psim2-SUMO %+.2f s)" % (
        pt["sumo_vs_sumo_rms"], pt["psim_vs_psim_rms"], pt["psim_vs_sumo_rms"], pt["psim_vs_sumo_mean"]))
    report["paired"] = pt
    if a.json:
        with open(a.json, "w") as f:
            json.dump(report, f, indent=1, default=str)


if __name__ == "__main__":
    main()
