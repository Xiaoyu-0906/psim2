"""Convert a SUMO .net.xml into the psim2 network text format.

The whole network can be converted, or only the part a set of routes uses:
every junction touched by a used edge is kept completely (all incoming and
outgoing edges, internal lanes, right-of-way logic and the controlling traffic
light), because the link indices of the junction logic refer to all of them.

Line types (tokens separated by single spaces, shapes as x,y;x,y;...):
  EDGE id function from to priority bidi  (function: normal, internal, crossing, walkingarea; bidi edge or -)
  ETYPE edge routingType                (MSEdge::getRoutingType: routingType, otherwise type; only typed edges)
  LBIDI lane bidiLane                   (MSLane::getBidiLane)
  LANEZ lane z,z,...                    (the elevation of the lane's shape points, only when not flat)
  LANE id edge index speed length width allow changeLeft changeRight shape
  JUNC id type incLanes intLanes            (comma separated, '-' if empty)
  REQ junction index response foes cont
  CONN fromLane toLane via dir state tl linkIndex visibility keepClear indirect
  TLS id type programID offset          (type rail_signal / rail_crossing: no phases)
  PHASE tls duration minDur maxDur state
  TLPARAM tls key value
  ROUNDABOUT edge
  NETVERSION major minor
"""
import gzip
import math
import sys
import xml.etree.ElementTree as ET

# SUMO's vehicle classes in bit order (SUMOVehicleClass.h), psim2 uses the same masks
CLASS_ORDER = ["private", "emergency", "authority", "army", "vip", "pedestrian", "passenger", "hov", "taxi", "bus", "coach",
               "delivery", "truck", "trailer", "motorcycle", "moped", "bicycle", "evehicle", "tram", "rail_urban", "rail",
               "rail_electric", "rail_fast", "ship", "container", "cable_car", "subway", "aircraft", "wheelchair", "scooter",
               "drone", "custom1", "custom2"]
VCLASS_BITS = {c: 1 << i for i, c in enumerate(CLASS_ORDER)}
ALL_SUMO_CLASSES = set(CLASS_ORDER)
ALL_MASK = (1 << len(CLASS_ORDER)) - 1


def perm_mask(allow, disallow):
    if allow is not None:
        classes = set(allow.split()) if allow != "all" else ALL_SUMO_CLASSES
    elif disallow is not None:
        classes = ALL_SUMO_CLASSES - (set(disallow.split()) if disallow != "all" else ALL_SUMO_CLASSES)
    else:
        classes = ALL_SUMO_CLASSES
    m = 0
    for c, b in VCLASS_BITS.items():
        if c in classes:
            m |= b
    return m


def change_mask(value):
    # SUMO lane attributes changeLeft/changeRight list the classes that may change (default: all)
    if value is None:
        return ALL_MASK
    return perm_mask(value, None) if value not in ("", "ignoring") else ALL_MASK


def _shape(lane):
    return [tuple(float(v) for v in p.split(",")[:2]) for p in lane["shape"].split()]


def _almost_same(a, b, max_div):
    """PositionVector::almostSame"""
    if len(a) != len(b):
        return False
    return all(math.hypot(p[0] - q[0], p[1] - q[1]) <= max_div for p, q in zip(a, b))


def bidi_lanes(edges, lanes_of, version):
    """MSEdge::checkAndRegisterBiDirEdge / setBidiLanes: (edge -> bidi edge, lane -> bidi lane)"""
    bidi_edge = {eid: e["bidi"] for eid, e in edges.items() if e.get("bidi")}
    if not bidi_edge and version <= (1, 0):
        # legacy networks: superposable edges in opposite direction
        by_ends = {}
        for eid, e in edges.items():
            if e.get("function", "normal") == "normal":
                by_ends.setdefault((e["from"], e["to"]), []).append(eid)
        for eid, e in edges.items():
            if e.get("function", "normal") != "normal":
                continue
            for other in by_ends.get((e["to"], e["from"]), []):
                l1, l2 = lanes_of[eid], lanes_of[other]
                if len(l1) == len(l2) and all(_shape(a)[::-1] == _shape(b) for a, b in zip(l1, reversed(l2))):
                    bidi_edge[eid] = other
    lane_pairs = {}
    for eid, other in bidi_edge.items():
        l1, l2 = lanes_of[eid], lanes_of.get(other, [])
        if len(l1) == 1 and len(l2) == 1:
            lane_pairs[l1[0]["id"]] = l2[0]["id"]
        else:
            for a in l1:
                for b in l2:
                    if _almost_same(_shape(a)[::-1], _shape(b), 0.2):
                        lane_pairs[a["id"]] = b["id"]
    return bidi_edge, lane_pairs


def open_xml(path):
    return gzip.open(path, "rb") if path.endswith(".gz") else open(path, "rb")


def junction_edges(net_path):
    """junction id -> (incoming normal edges, outgoing normal edges) (MSJunction::getIncoming / getOutgoing)"""
    res = {}
    opener = gzip.open if net_path.endswith(".gz") else open
    with opener(net_path, "rb") as f:
        for ev, el in ET.iterparse(f):
            if el.tag == "edge" and el.get("function", "normal") == "normal" and el.get("from") is not None:
                res.setdefault(el.get("to"), ([], []))[0].append(el.get("id"))
                res.setdefault(el.get("from"), ([], []))[1].append(el.get("id"))
                el.clear()
            elif el.tag == "junction" and not el.get("id", "").startswith(":"):
                res.setdefault(el.get("id"), ([], []))
                el.clear()
    return res


def convert(net_path, out_path, used_edges=None, tls_override=None, no_internal=False):
    """used_edges: set of normal edge ids routes use (None = keep everything).
    tls_override: traffic light programs loaded after the net (additional files), id -> (attrs, phases, params);
    the last loaded program of a traffic light is the active one (MSTLLogicControl::TLSLogicVariants::addLogic).
    no_internal: SUMO's --no-internal-links (NLHandler: the connections run from normal lane to normal lane without
    via lanes, junctions have no internal lanes and internal junctions are not built; the unconnected internal
    edges are left out)."""
    edges, lanes_of, junctions, conns, tls = {}, {}, {}, [], {}
    roundabout_edges = set()
    cur = None
    version = (0, 0)
    lefthand = False
    with open_xml(net_path) as f:
        for ev, el in ET.iterparse(f, events=("start", "end")):
            if ev == "start":
                if el.tag == "net" and el.get("version"):
                    parts = (el.get("version").split(".") + ["0"])[:2]
                    version = (int(parts[0]), int(parts[1]))
                if el.tag == "net" and el.get("lefthand") in ("true", "1"):
                    lefthand = True
                if el.tag == "edge":
                    cur = el.get("id")
                    edges[cur] = dict(el.attrib)
                    lanes_of[cur] = []
                continue
            if el.tag == "lane" and cur is not None:
                lanes_of[cur].append(dict(el.attrib))
            elif el.tag == "edge":
                cur = None
                el.clear()
            elif el.tag == "junction":
                d = dict(el.attrib)
                d["requests"] = [dict(r.attrib) for r in el.findall("request")]
                junctions[d["id"]] = d
                el.clear()
            elif el.tag == "tlLogic":
                tls[el.get("id")] = (dict(el.attrib), [dict(p.attrib) for p in el.findall("phase")],
                                     [(p.get("key"), p.get("value")) for p in el.findall("param")])
            elif el.tag == "connection":
                conns.append(dict(el.attrib))
                el.clear()
            elif el.tag == "roundabout":
                roundabout_edges.update(el.get("edges", "").split())

    def is_internal(eid):
        """edges inside a junction: internal lanes, pedestrian crossings and walking areas (':<junction>_...')"""
        return edges[eid].get("function") in ("internal", "crossing", "walkingarea")

    if used_edges is None:
        keep_junctions = set(junctions)
    else:
        missing = [e for e in used_edges if e not in edges]
        if missing:
            raise ValueError("routes use unknown edges: %s" % missing[:5])
        keep_junctions = set()
        for e in used_edges:
            keep_junctions.add(edges[e]["from"])
            keep_junctions.add(edges[e]["to"])
        # junctions sharing a traffic light are kept together
        tl_of = {}
        for c in conns:
            if c.get("tl"):
                j = edges[c["from"]]["to"] if not is_internal(c["from"]) else None
                if j is not None:
                    tl_of.setdefault(c["tl"], set()).add(j)
        changed = True
        while changed:
            changed = False
            for js in tl_of.values():
                if js & keep_junctions and not js <= keep_junctions:
                    keep_junctions |= js
                    changed = True
    keep_edges = set()
    for eid, e in edges.items():
        if is_internal(eid) and no_internal:
            continue
        if is_internal(eid):
            # internal edge ids are ':<junction>_<index>'
            if eid[1:].rsplit("_", 1)[0] in keep_junctions:
                keep_edges.add(eid)
        elif e["from"] in keep_junctions or e["to"] in keep_junctions:
            keep_edges.add(eid)
    # internal junctions (':<junction>_<n>_0') of kept junctions
    keep_internal_junctions = set()
    for jid, j in junctions.items():
        if j.get("type") == "internal" and not no_internal:
            inc = j.get("incLanes", "").split()
            if inc and inc[0].rsplit("_", 1)[0] in keep_edges:
                keep_internal_junctions.add(jid)
    lane_ids = set()
    for eid in keep_edges:
        for ln in lanes_of[eid]:
            lane_ids.add(ln["id"])

    bidi_edge, bidi_lane = bidi_lanes(edges, lanes_of, version)
    out = open(out_path, "w")
    out.write("NETVERSION %d %d\n" % version)
    if lefthand:
        out.write("LEFTHAND\n")  # MSGlobals::gLefthand
    if no_internal:
        out.write("NOINTERNAL\n")  # MSGlobals::gUsingInternalLanes = false
    n_edges = n_lanes = n_conns = 0
    # edges in net-file order: SUMO numbers lanes in this order (MSLane numerical ids)
    for eid in edges:
        if eid not in keep_edges:
            continue
        e = edges[eid]
        func = e.get("function", "normal")
        if func not in ("normal", "internal", "crossing", "walkingarea"):
            raise ValueError("edge %s: unsupported function %s" % (eid, func))
        frm = e.get("from", "-") if func == "normal" else eid[1:].rsplit("_", 1)[0]
        to = e.get("to", "-") if func == "normal" else frm
        out.write("EDGE %s %s %s %s %s %s\n" % (eid, func, frm, to, e.get("priority", "-1"),
                                               bidi_edge[eid] if bidi_edge.get(eid) in keep_edges else "-"))
        if e.get("routingType") or e.get("type"):
            out.write("ETYPE %s %s\n" % (eid, e.get("routingType") or e.get("type")))
        n_edges += 1
        for ln in sorted(lanes_of[eid], key=lambda l: int(l["index"])):
            shape = ";".join(",".join(p.split(",")[:2]) for p in ln["shape"].split())
            out.write("LANE %s %s %s %s %s %s %d %d %d %s\n" % (
                ln["id"], eid, ln["index"], ln["speed"], ln["length"], ln.get("width", "3.2"),
                perm_mask(ln.get("allow"), ln.get("disallow")),
                change_mask(ln.get("changeLeft")), change_mask(ln.get("changeRight")), shape))
            zs = [p.split(",")[2] if len(p.split(",")) > 2 else "0" for p in ln["shape"].split()]
            if any(float(z) != 0 for z in zs):
                out.write("LANEZ %s %s\n" % (ln["id"], ",".join(zs)))  # the elevation of the shape points
            n_lanes += 1
    for lid in sorted(bidi_lane):
        if lid in lane_ids and bidi_lane[lid] in lane_ids:
            out.write("LBIDI %s %s\n" % (lid, bidi_lane[lid]))
    for jid in sorted(junctions):
        j = junctions[jid]
        if jid not in keep_junctions and jid not in keep_internal_junctions:
            continue
        inc = [l for l in j.get("incLanes", "").split() if not (no_internal and l.startswith(":"))]
        intl = [l for l in j.get("intLanes", "").split()] if not no_internal else []
        for l in inc + intl:
            if l not in lane_ids:
                raise ValueError("junction %s refers to lane %s outside the cut" % (jid, l))
        out.write("JUNC %s %s %s %s\n" % (jid, j["type"], ",".join(inc) or "-", ",".join(intl) or "-"))
        for r in j["requests"]:
            out.write("REQ %s %s %s %s %s\n" % (jid, r["index"], r["response"], r["foes"], r.get("cont", "0")))
    used_tls = set()
    for c in conns:
        fl = "%s_%s" % (c["from"], c["fromLane"])
        tl = "%s_%s" % (c["to"], c["toLane"])
        if fl not in lane_ids or tl not in lane_ids:
            continue
        # edges at the border of the cut keep no connections into junctions outside it
        cj = c["from"][1:].rsplit("_", 1)[0] if is_internal(c["from"]) else edges[c["from"]]["to"]
        if cj not in keep_junctions:
            continue
        via = c.get("via", "-") if not no_internal else "-"
        if via != "-" and via not in lane_ids:
            raise ValueError("via lane %s outside the cut" % via)
        if c.get("tl"):
            used_tls.add(c["tl"])
        out.write("CONN %s %s %s %s %s %s %s %s %s %s\n" % (
            fl, tl, via, c["dir"], c["state"], c.get("tl", "-"), c.get("linkIndex", "-1"),
            c.get("visibility", "-"), c.get("keepClear", "1").replace("true", "1").replace("false", "0"),
            c.get("indirect", "0").replace("true", "1").replace("false", "0")))
        n_conns += 1
    extra_params = {}
    for tid, (attrs, phases, params) in (tls_override or {}).items():
        if not phases and attrs.get("programID") == "off":
            # NLJunctionControlBuilder: the program "off" is an MSOffTrafficLightLogic (psim2 builds its phase)
            tls[tid] = ({"type": "off", "programID": "off", "offset": "0"}, [], params)
        elif not phases:
            # a program without phases only sets parameters of the loaded one (e.g. moving-block of a rail signal)
            if tid in tls:
                tls[tid] = (tls[tid][0], tls[tid][1], tls[tid][2] + list(params))
            else:
                extra_params.setdefault(tid, []).extend(params)
        else:
            tls[tid] = (attrs, phases, params)
    # creation order of the logics (numerical ids of rail signals): the programs in file order, then the rail
    # signals and crossings of junctions without a program (NLJunctionControlBuilder::closeJunction)
    tls_order = [tid for tid in tls if tid in used_tls]
    # (a rail signal or crossing junction gets its logic even when no connection refers to it)
    tls_order += [jid for jid in junctions if jid not in tls and (jid in used_tls or (
        jid in keep_junctions and junctions[jid].get("type") in ("rail_signal", "rail_crossing")))]
    tls_order += sorted(used_tls - set(tls_order))
    for tid in tls_order:
        jtype = junctions.get(tid, {}).get("type", "")
        if jtype in ("rail_signal", "rail_crossing") and tid not in tls:
            # NLJunctionControlBuilder: MSRailSignal / MSRailCrossing without phases
            out.write("TLS %s %s 0 0\n" % (tid, jtype))
            for k, v in extra_params.get(tid, []):
                out.write("TLPARAM %s %s %s\n" % (tid, k, v))
            continue
        if tid not in tls:
            raise ValueError("junction %s (%s) controls links without a tlLogic" % (tid, jtype))
        attrs, phases, params = tls[tid]
        out.write("TLS %s %s %s %s\n" % (tid, attrs.get("type", "static"), attrs.get("programID", "0"), attrs.get("offset", "0")))
        for p in phases:
            d = p["duration"]
            if p.get("next") or p.get("earliestEnd") or p.get("latestEnd"):
                raise ValueError("phase attributes next/earliestEnd/latestEnd are not supported (tls %s)" % tid)
            # NLHandler::addPhase: maxDur defaults to "unlimited" when minDur is given
            max_default = "2147483.647" if p.get("minDur") is not None else d
            out.write("PHASE %s %s %s %s %s\n" % (tid, d, p.get("minDur", d), p.get("maxDur", max_default), p["state"]))
        for k, v in params:
            out.write("TLPARAM %s %s %s\n" % (tid, k, v))
    for eid in sorted(roundabout_edges & keep_edges):
        out.write("ROUNDABOUT %s\n" % eid)
    out.close()
    return dict(edges=n_edges, lanes=n_lanes, connections=n_conns, junctions=len(keep_junctions), tls=len(used_tls))


if __name__ == "__main__":
    print(convert(sys.argv[1], sys.argv[2]))
