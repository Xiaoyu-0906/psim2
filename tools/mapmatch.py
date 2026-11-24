"""Matching positions to the network like SUMO's MapMatcher (MSRouteHandler::parseGeoEdges): fromXY / toXY /
viaXY, fromLonLat / toLonLat / viaLonLat and stops given by x, y or lon, lat.

Lon/lat positions are projected with the network's projection (only UTM, as +proj=utm, is supported) and moved
by its netOffset (GeoConvHelper::x2cartesian_const).
"""
import gzip
import math
import xml.etree.ElementTree as ET

import psim2net


def utm_forward(lon, lat, zone, south=False):
    """UTM easting / northing on WGS84 (Krueger series, sub-millimetre like PROJ's etmerc)"""
    a = 6378137.0
    f = 1 / 298.257223563
    k0 = 0.9996
    n = f / (2 - f)
    big_a = a / (1 + n) * (1 + n ** 2 / 4 + n ** 4 / 64)
    alpha = (n / 2 - 2 * n ** 2 / 3 + 5 * n ** 3 / 16 + 41 * n ** 4 / 180,
             13 * n ** 2 / 48 - 3 * n ** 3 / 5 + 557 * n ** 4 / 1440,
             61 * n ** 3 / 240 - 103 * n ** 4 / 140,
             49561 * n ** 4 / 161280)
    e = math.sqrt(f * (2 - f))
    phi = math.radians(lat)
    lam = math.radians(lon) - math.radians(-183 + 6 * zone)
    t = math.sinh(math.atanh(math.sin(phi)) - e * math.atanh(e * math.sin(phi)))
    xi_p = math.atan2(t, math.cos(lam))
    eta_p = math.atanh(math.sin(lam) / math.sqrt(1 + t * t))
    xi = xi_p + sum(alpha[j - 1] * math.sin(2 * j * xi_p) * math.cosh(2 * j * eta_p) for j in range(1, 5))
    eta = eta_p + sum(alpha[j - 1] * math.cos(2 * j * xi_p) * math.sinh(2 * j * eta_p) for j in range(1, 5))
    return 500000 + k0 * big_a * eta, k0 * big_a * xi + (10000000 if south else 0)


def _segment_distance(p, a, b):
    """distance of p to the segment a-b (GeomHelper::nearest_offset_on_line_to_point2D, not perpendicular)"""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    if length2 == 0:
        return math.hypot(p[0] - a[0], p[1] - a[1])
    u = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length2))
    return math.hypot(p[0] - a[0] - u * dx, p[1] - a[1] - u * dy)


class MapMatcher:
    def __init__(self, net_path, match_junctions=False, match_taz=False, distance=100.0):
        self.match_junctions = match_junctions
        self.match_taz = match_taz
        self.distance = distance
        self.lanes = []          # (id, edge, permissions, shape, bounding box), network file order
        self.edge_from_to = {}   # normal edge -> (from junction, to junction)
        self.internal_succ = {}  # internal edge -> first successor edge
        self.junction_pos = {}
        self.offset = (0.0, 0.0)
        self.proj = None
        opener = gzip.open if net_path.endswith(".gz") else open
        with opener(net_path, "rb") as f:
            for ev, el in ET.iterparse(f):
                if el.tag == "location":
                    self.offset = tuple(float(v) for v in el.get("netOffset", "0,0").split(","))
                    self.proj = el.get("projParameter", "!")
                elif el.tag == "edge":
                    eid = el.get("id")
                    if el.get("function", "normal") == "normal":
                        self.edge_from_to[eid] = (el.get("from"), el.get("to"))
                    for ln in el.findall("lane"):
                        shape = [tuple(float(v) for v in p.split(",")[:2]) for p in ln.get("shape").split()]
                        xs, ys = [p[0] for p in shape], [p[1] for p in shape]
                        self.lanes.append((ln.get("id"), eid, psim2net.perm_mask(ln.get("allow"), ln.get("disallow")),
                                           shape, (min(xs), min(ys), max(xs), max(ys))))
                    el.clear()
                elif el.tag == "junction":
                    self.junction_pos[el.get("id")] = (float(el.get("x")), float(el.get("y")))
                    el.clear()
                elif el.tag == "connection":
                    frm = el.get("from")
                    if frm.startswith(":") and frm not in self.internal_succ:
                        self.internal_succ[frm] = el.get("to")
                    el.clear()

    def to_cartesian(self, lon, lat):
        params = dict(p.lstrip("+").split("=", 1) if "=" in p else (p.lstrip("+"), "") for p in (self.proj or "").split())
        if params.get("proj") != "utm":
            raise ValueError("Cannot convert geo-positions: only UTM projections are supported (%s)" % self.proj)
        x, y = utm_forward(lon, lat, int(params["zone"]), "south" in params)
        return x + self.offset[0], y + self.offset[1]

    def closest_lane(self, pos, vclass_bit, dist):
        """MapMatcher::getClosestLane: the nearest lane allowing the class among those whose bounding box meets
        the box around pos (ties: the first lane of the network file)"""
        best, best_dist = None, float("inf")
        for lane in self.lanes:
            box = lane[4]
            if box[0] > pos[0] + dist or box[2] < pos[0] - dist or box[1] > pos[1] + dist or box[3] < pos[1] - dist:
                continue
            if (lane[2] & vclass_bit) != vclass_bit:
                continue
            shape = lane[3]
            d = min(_segment_distance(pos, shape[i], shape[i + 1]) for i in range(len(shape) - 1)) \
                if len(shape) > 1 else math.hypot(pos[0] - shape[0][0], pos[1] - shape[0][1])
            if d < best_dist:
                best, best_dist = lane, d
        return best

    def nearest_offset(self, edge, pos):
        """PositionVector::nearest_offset_to_point2D(pos, false) on the first lane of the edge (a geometric offset)"""
        shape = next(ln[3] for ln in self.lanes if ln[1] == edge)
        best, best_dist, seen = 0.0, float("inf"), 0.0
        for a, b in zip(shape, shape[1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]
            length = math.hypot(dx, dy)
            u = 0.0 if length == 0 else max(0.0, min(length, ((pos[0] - a[0]) * dx + (pos[1] - a[1]) * dy) / length))
            d = math.hypot(pos[0] - a[0] - (u / length * dx if length else 0), pos[1] - a[1] - (u / length * dy if length else 0))
            if d < best_dist:
                best, best_dist = seen + u, d
            seen += length
        return best

    def match(self, positions, geo, vclass_bit, is_from, rid, zones, force_edge=False):
        """MapMatcher::parseGeoEdges: edge ids, or zone connector ids ("<zone>-source" / "<zone>-sink") when
        positions are matched to junctions or taz; zones: zone id -> (sources, sinks)"""
        into = []
        for orig in positions:
            pos = self.to_cartesian(*orig) if geo else orig
            dist = min(10.0, self.distance)
            best = self.closest_lane(pos, vclass_bit, dist)
            while best is None and dist < self.distance:
                dist = min(dist * 2, self.distance)
                best = self.closest_lane(pos, vclass_bit, dist)
            if best is None:
                raise ValueError("No edge found near position %.2f,%.2f within the route %s." % (orig[0], orig[1], rid))
            edge = best[1]
            while edge.startswith(":"):
                edge = self.internal_succ[edge]
            if (self.match_junctions or self.match_taz) and not force_edge:
                if self.match_taz:
                    into.append(self._taz(pos, edge, is_from, zones))
                else:
                    into.append(self._junction_taz(pos, edge, vclass_bit, is_from, zones))
            elif len(positions) == 1 or not into or into[-1] != edge:
                into.append(edge)
        return into

    def _junction_taz(self, pos, edge, vclass_bit, is_from, zones):
        """MapMatcher::getJunctionTaz"""
        fj, tj = self.edge_from_to[edge]
        for j in (fj, tj):
            if j not in zones:
                raise ValueError("Junction-taz '%s' not found when mapping position %.2f,%.2f." % (j, pos[0], pos[1]))
        fp, tp = self.junction_pos[fj], self.junction_pos[tj]
        from_closer = (fp[0] - pos[0]) ** 2 + (fp[1] - pos[1]) ** 2 < (tp[0] - pos[0]) ** 2 + (tp[1] - pos[1]) ** 2

        def possible(j):
            if is_from:
                return any(self._edge_allows(e, vclass_bit) for e in zones[j][0])
            return bool(zones[j][1])
        suffix = "-source" if is_from else "-sink"
        if from_closer and possible(fj):
            return fj + suffix
        if not from_closer and possible(tj):
            return tj + suffix
        return (fj if possible(fj) else tj) + suffix

    def _edge_allows(self, edge, vclass_bit):
        """MSEdge::getSuccessors(vClass) of a connector: edges with a lane allowing the class"""
        return any(ln[1] == edge and (ln[2] & vclass_bit) == vclass_bit for ln in self.lanes)

    def _taz(self, pos, edge, is_from, zones):
        """MapMatcher::getTaz (several zones: SUMO draws one at random, here the first)"""
        side = 0 if is_from else 1
        cands = [z for z, d in zones.items() if edge in d[side]]
        if not cands:
            raise ValueError("Taz for edge '%s' not found when mapping position %.2f,%.2f." % (edge, pos[0], pos[1]))
        return cands[0] + ("-source" if is_from else "-sink")
