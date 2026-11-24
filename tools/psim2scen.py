"""Build a psim2 scenario file from SUMO inputs (net, routes, additional files).

Usage: psim2scen.py NET ROUTES[,ROUTES2...] OUT [--additional ADD] [--step 1] [--begin B] [--end E]
                    [--full-net] [--sigma S] [--speeddev D]

Vehicles, routes, flows (expanded into vehicles), vehicle types and stops at
bus stops are converted. The network is cut to the junctions the routes use
(see psim2net.py) unless --full-net is given. --sigma / --speeddev override
the driver imperfection and the speed deviation of all types (sigma 0 and
speeddev 0 give deterministic runs for step-by-step comparison with SUMO).
"""
import argparse
import math
import gzip
import os
import re
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import psim2net  # noqa: E402
import mapmatch  # noqa: E402


def sumo_bool(v):
    """StringUtils::toBool"""
    s = v.lower()
    if s in ("1", "yes", "true", "on", "x", "t"):
        return True
    if s in ("0", "no", "false", "off", "-", "f"):
        return False
    raise ValueError("invalid boolean %s" % v)


def parse_time(v):
    """SUMO time values: seconds or human readable [[d:]h:]m:s (SUMOTime string2time)"""
    if ":" not in v:
        return float(v)
    parts = [float(p) for p in v.split(":")]
    secs = 0.
    for p, mul in zip(reversed(parts), (1, 60, 3600, 86400)):
        secs += p * mul
    return secs


def time2steps(t):
    """SUMO's TIME2STEPS: seconds to integer milliseconds"""
    return int(t * 1000. + (0.5 if t >= 0 else -0.5))


# SUMO 1.28 defaults per vehicle class (SUMOVTypeParameter::VClassDefaultValues, getDefaultAccel/Decel/
# EmergencyDecel/Imperfection, getDefaultVehicleLength)
def _class_defaults():
    length = dict(pedestrian=0.215, wheelchair=1.2, bicycle=1.6, scooter=1.2, moped=2.1, motorcycle=2.2, truck=7.1,
                  trailer=16.5, bus=12., coach=14., tram=22., rail_urban=36.5 * 3, subway=36.5 * 3, rail=67.5 * 2,
                  rail_electric=25. * 8, rail_fast=25. * 8, delivery=6.5, emergency=6.5, ship=17, container=6.096,
                  drone=0.5, aircraft=72.7)
    accel = dict(pedestrian=1.5, wheelchair=1.5, bicycle=1.2, scooter=1.2, motorcycle=6., moped=1.1, truck=1.3,
                 trailer=1.1, bus=1.2, coach=2., tram=1., rail_urban=1., rail=0.25, rail_electric=0.5, rail_fast=0.5,
                 ship=0.1)
    decel = dict(pedestrian=2., wheelchair=2., bicycle=3., scooter=3., moped=7., motorcycle=10., truck=4., trailer=4.,
                 bus=4., coach=4., tram=3., rail_urban=3., rail=1.3, rail_electric=1.3, rail_fast=1.3, ship=0.15)
    emergency = dict(pedestrian=5., wheelchair=5., bicycle=7., scooter=7., moped=10., motorcycle=10., truck=7.,
                     trailer=7., bus=7., coach=7., tram=7., rail_urban=7., rail=5., rail_electric=5., rail_fast=5.,
                     ship=1.)
    max_speed = dict(pedestrian=37.58 / 3.6, wheelchair=30. / 3.6, bicycle=50. / 3.6, scooter=25. / 3.6,
                     moped=60. / 3.6, truck=130. / 3.6, trailer=130. / 3.6, bus=100. / 3.6, coach=100. / 3.6,
                     tram=80. / 3.6, rail_urban=100. / 3.6, subway=100. / 3.6, rail=160. / 3.6,
                     rail_electric=220. / 3.6, rail_fast=330. / 3.6, ship=8 / 1.94)
    desired = dict(pedestrian=5. / 3.6, wheelchair=5. / 3.6, bicycle=20. / 3.6, scooter=20. / 3.6)
    min_gap = dict(pedestrian=0.25, wheelchair=0.5, bicycle=0.5, scooter=0.5, rail_urban=5., subway=5., rail=5.,
                   rail_electric=5., rail_fast=5.)
    speed_dev = dict(pedestrian=0.1, wheelchair=0.1, bicycle=0.1, scooter=0.1, moped=0.1, motorcycle=0.1,
                     truck=0.05, trailer=0.05, coach=0.05, delivery=0.05, private=0.1, vip=0.1, passenger=0.1,
                     hov=0.1, custom1=0.1, custom2=0.1, taxi=0.05, evehicle=0.1, ship=0.1)
    width = dict(pedestrian=0.478, wheelchair=0.72, bicycle=0.65, scooter=0.5, moped=0.78, motorcycle=0.9, truck=2.4,
                 trailer=2.55, bus=2.5, coach=2.6, tram=2.4, rail_urban=3.0, subway=3.0, rail=2.84, rail_electric=2.95,
                 rail_fast=2.95, delivery=2.16, emergency=2.16, container=2.438, drone=0.5, aircraft=79.8, ship=4.)
    person_cap = dict(bicycle=1, scooter=1, moped=1, motorcycle=1, truck=2, trailer=2, bus=85, coach=70, tram=120,
                      rail_urban=300, subway=300, rail=434, rail_electric=425, rail_fast=425, delivery=2, emergency=2)
    container_cap = dict(truck=1, trailer=2)
    no_sigma = {"tram", "rail_urban", "rail", "rail_electric", "rail_fast", "ship"}
    out = {}
    for c in psim2net.CLASS_ORDER:
        d = decel.get(c, 4.5)
        out[c] = dict(accel=accel.get(c, 2.6), decel=d, emergencyDecel=max(d, emergency.get(c, 9.)),
                      sigma=0.0 if c in no_sigma else 0.5, tau=1.0, length=length.get(c, 5.),
                      minGap=min_gap.get(c, 2.5), maxSpeed=max_speed.get(c, 200. / 3.6),
                      desiredMaxSpeed=desired.get(c, 10000. / 3.6), speedDev=speed_dev.get(c, 0.0),
                      width=width.get(c, 1.8), personCapacity=person_cap.get(c, 4), containerCapacity=container_cap.get(c, 0))
    return out


CLASS_DEFAULTS = _class_defaults()
# SVC_IGNORING: the generic defaults (SUMOVTypeParameter::VClassDefaultValues without a class case)
CLASS_DEFAULTS["ignoring"] = dict(accel=2.6, decel=4.5, emergencyDecel=9., sigma=0.5, tau=1.0, length=5., minGap=2.5,
                                  maxSpeed=200. / 3.6, desiredMaxSpeed=10000. / 3.6, speedDev=0.0, width=1.8)


def open_xml(path):
    return gzip.open(path, "rb") if path.endswith(".gz") else open(path, "rb")


def parse_speed_factor(t, d):
    sf = t.get("speedFactor")
    dev = float(t["speedDev"]) if t.get("speedDev") is not None else d["speedDev"]
    mean, mn, mx = 1.0, 0.2, 2.0
    if sf is not None:
        m = re.match(r"norm[c]?\(([^)]*)\)", sf)
        if m:
            parts = [float(x) for x in m.group(1).split(",")]
            mean, dev = parts[0], parts[1]
            if len(parts) > 2:
                mn, mx = parts[2], parts[3]
        else:
            mean = float(sf)
            if t.get("speedDev") is None:
                dev = 0.0 if d["speedDev"] == 0 else d["speedDev"]
        if t.get("speedDev") is not None:
            dev = float(t["speedDev"])  # (SUMOVehicleParserHelper::beginVTypeParsing: after the distribution)
    return mean, dev, mn, mx


DEFAULT_SPEEDDEV = None  # --default-speeddev


# the car-following model's own attributes (SUMOVehicleParserHelper::getAllowedCFModelAttrs), passed to psim2 as given
CF_ATTRS = {"sigmaStep", "startupDelay", "tmp1", "tmp2", "tmp3", "tmp4", "tmp5", "vehdynamics", "tpreview", "tPersDrive",
            "treaction", "tPersEstimate", "ccoolness", "sigmaleader", "sigmagap", "sigmaerror", "jerkmax", "epsilonacc",
            "taccmax", "Mflatness", "Mbegin", "maxvehpreview", "speedControlGain", "gapClosingControlGainSpeed",
            "gapClosingControlGainSpace", "gapControlGainSpeed", "gapControlGainSpace", "collisionAvoidanceGainSpeed",
            "collisionAvoidanceGainSpace", "collisionAvoidanceOverride", "speedControlGainCACC", "gapClosingControlGainGap",
            "gapClosingControlGainGapDot", "gapControlGainGap", "gapControlGainGapDot", "collisionAvoidanceGainGap",
            "collisionAvoidanceGainGapDot", "tauCACCToACC", "speedControlMinGap", "applyDriverState", "k", "phi",
            "tauLast", "apProb", "delta", "stepping", "adaptFactor", "adaptTime", "security", "estimation",
            "cc1", "cc2", "cc3", "cc4", "cc5", "cc6", "cc7", "cc8", "cc9", "collisionMinGapFactor", "maxAccelProfile",
            "desAccelProfile", "jmSigmaMinor", "speedTable"}
CF_MODELS = ("Krauss", "KraussOrig1", "KraussPS", "KraussX", "SmartSK", "Daniel1", "PWagner2009", "IDM", "IDMM", "EIDM",
             "BKerner", "Wiedemann", "W99", "Rail", "ACC", "CACC", "CC", "NaSch")
DEFAULT_CF_MODEL = "Krauss"  # --carfollow.model
LC_ATTRS = ("lcStrategic", "lcCooperative", "lcSpeedGain", "lcKeepRight", "lcAssertive", "lcLookaheadLeft", "lcSpeedGainRight",
            "lcSpeedGainLookahead", "lcSpeedGainRemainTime", "lcSpeedGainUrgency", "lcCooperativeSpeed",
            "lcKeepRightAcceptanceTime", "lcOvertakeDeltaSpeedFactor", "lcCooperativeHelpTime", "lcCooperativeHelpThreshold",
            "lcCooperativeMinSpeed", "lcStrategicLookahead", "lcExperimental1", "lcCooperativeRoundabout", "lcOvertakeRight",
            # SL2015 (sublane model)
            "lcSublane", "lcPushy", "lcPushyGap", "lcImpatience", "lcTimeToImpatience", "lcAccelLat", "lcTurnAlignmentDistance",
            "lcLaneDiscipline", "lcSigma", "lcMaxDistLatStanding", "lcOpposite", "minGapLat")
# SUMOVTypeParameter::parseLatAlignment (psim2 codes)
LAT_ALIGNMENTS = {"right": 0, "center": 1, "arbitrary": 2, "nice": 3, "compact": 4, "left": 5}


def stop_dict(el):
    """the attributes of a stop element (and its flexArrival parameter)"""
    d = dict(el.attrib)
    for prm in el.findall("param"):
        if prm.get("key") == "flexArrival":
            d["_flexArrival"] = prm.get("value")
    return d


def type_line(t, sigma_override, speeddev_override):
    vclass = t.get("vClass", "passenger")
    d = CLASS_DEFAULTS[vclass]
    vals = {k: d[k] for k in ("accel", "decel", "sigma", "tau", "length", "minGap", "maxSpeed", "desiredMaxSpeed", "width")}
    for k in vals:
        if t.get(k) is not None:
            vals[k] = float(t[k])
    # SUMOVehicleParserHelper::beginVTypeParsing: maxSpeed without desiredMaxSpeed (backward compatibility)
    if t.get("maxSpeed") is not None and t.get("desiredMaxSpeed") is None:
        if vclass == "pedestrian":
            vals["desiredMaxSpeed"] = vals["maxSpeed"]
            vals["maxSpeed"] = max(vals["maxSpeed"], d["maxSpeed"])
        elif vclass == "bicycle":
            vals["desiredMaxSpeed"] = max(vals["maxSpeed"], vals["desiredMaxSpeed"])
    emerg = float(t["emergencyDecel"]) if t.get("emergencyDecel") else max(vals["decel"], d["emergencyDecel"])
    apparent = float(t["apparentDecel"]) if t.get("apparentDecel") else vals["decel"]
    if DEFAULT_SPEEDDEV is not None and t.get("speedDev") is None and t.get("speedFactor") is None:
        d = dict(d, speedDev=DEFAULT_SPEEDDEV)
    mean, dev, mn, mx = parse_speed_factor(t, d)
    if sigma_override is not None:
        vals["sigma"] = sigma_override
    if speeddev_override is not None:
        dev = speeddev_override
    cfm = t.get("carFollowModel", DEFAULT_CF_MODEL)
    if cfm not in CF_MODELS:
        raise ValueError("car-following model %s not supported (type %s)" % (cfm, t["id"]))
    lcm = t.get("laneChangeModel", "default")
    if lcm not in ("LC2013", "SL2015", "default"):
        raise ValueError("lane-change model %s not supported (type %s)" % (lcm, t["id"]))
    params = dict(vals, emergencyDecel=emerg, apparentDecel=apparent, speedFactorMean=mean, speedFactorDev=dev,
                  speedFactorMin=mn, speedFactorMax=mx)
    # junction model and pedestrian parameters (only when given)
    for attr, key in (("impatience", "impatience"), ("jmDriveAfterRedTime", "jmDriveAfterRedTime"),
                      ("jmDriveAfterYellowTime", "jmDriveAfterYellowTime"), ("jmIgnoreFoeProb", "jmIgnoreFoeProb"),
                      ("jmIgnoreFoeSpeed", "jmIgnoreFoeSpeed"), ("jmIgnoreJunctionFoeProb", "jmIgnoreJunctionFoeProb"),
                      ("jmCrossingGap", "jmCrossingGap"), ("jmStoplineGap", "jmStoplineGap"),
                      ("jmStoplineGapMinor", "jmStoplineGapMinor"), ("jmStoplineCrossingGap", "jmStoplineCrossingGap"),
                      ("jmTimegapMinor", "jmTimegapMinor"), ("jmAllwayStopWait", "jmAllwayStopWait"),
                      ("jmStopSignWait", "jmStopSignWait")):
        if t.get(attr) is not None:
            params[key] = float(t[attr])
    # SUMOVTypeParameter::initRailVisualizationParameters: the locomotive length (minimum length after a split)
    shape = t.get("guiShape") or {"rail": "rail", "rail_electric": "rail", "rail_fast": "rail", "rail_urban": "rail/railcar",
                                  "tram": "rail/railcar", "subway": "rail/railcar", "trailer": "truck/trailer",
                                  "private": "passenger", "vip": "passenger", "passenger": "passenger", "hov": "passenger",
                                  "custom1": "passenger", "custom2": "passenger"}.get(vclass, "")
    # the outline (MSVehicle::getBoundingBox / getBoundingPoly): 1 passenger cars, 2 along the occupied lanes
    if shape in ("passenger", "passenger/sedan", "passenger/hatchback", "passenger/wagon", "passenger/van"):
        params["shapeKind"] = 1
    elif shape in ("rail", "rail/railcar", "rail/cargo", "bus/flexible", "truck/semitrailer", "truck/trailer"):
        params["shapeKind"] = 2
    loco = t.get("_params", {}).get("locomotiveLength")
    if loco is None:
        carriage = t.get("_params", {}).get("carriageLength")
        if shape == "rail":
            loco = {"rail_electric": 19.1, "rail_fast": 25.835}.get(vclass, 16.4)
        elif shape == "rail/railcar":
            loco = {"tram": 5.71, "rail_urban": 18.4}.get(vclass, carriage if carriage is not None else 16.85)
        elif shape == "truck/semitrailer":
            loco = 2.5
        elif shape == "truck/trailer":
            loco = 9.25
        elif carriage is not None:
            loco = carriage
        else:
            loco = {"rail/cargo": 13.86, "bus/flexible": 8.25}.get(shape, -1)
    params["locomotiveLength"] = float(loco)
    # LC2013 parameters (the sublane, continuous lane change and opposite driving parameters have no effect here)
    for attr in LC_ATTRS:
        if t.get(attr) is not None:
            params[attr] = float(t[attr])
    for attr in ("maxSpeedLat", "lcMaxSpeedLatStanding", "lcMaxSpeedLatFactor"):
        if t.get(attr) is not None:
            params[attr] = float(t[attr])
    if lcm != "default":
        params["lcModel"] = 1.0 if lcm == "LC2013" else 2.0
    if t.get("latAlignment") is not None:
        al = t["latAlignment"]
        if al in LAT_ALIGNMENTS:
            params["latAlignment"] = float(LAT_ALIGNMENTS[al])
        else:
            params["latAlignment"] = 6.0  # GIVEN
            params["latAlignmentOffset"] = float(al)
    if t.get("speedFactorPremature") is not None:
        params["speedFactorPremature"] = float(t["speedFactorPremature"])
    if t.get("actionStepLength") is not None:
        params["actionStepLength"] = float(t["actionStepLength"])
    if t.get("_params", {}).get("pedestrian.timegap-crossing") is not None:
        params["timegapCrossing"] = float(t["_params"]["pedestrian.timegap-crossing"])
    # transportables (capacities by class, boarding times only when given)
    d = CLASS_DEFAULTS[vclass]
    params["personCapacity"] = float(t.get("personCapacity", d.get("personCapacity", 4)))
    params["containerCapacity"] = float(t.get("containerCapacity", d.get("containerCapacity", 0)))
    for attr in ("boardingDuration", "loadingDuration", "boardingFactor"):
        if t.get(attr) is not None:
            params[attr] = parse_time(t[attr]) if attr != "boardingFactor" else float(t[attr])
    line = "TYPE %s %s %s\n" % (t["id"], vclass, " ".join("%s=%r" % (k, v) for k, v in params.items()))
    cf_attrs = {k: v for k, v in t.items() if k in CF_ATTRS}
    if cfm not in ("Krauss", "Rail") or cf_attrs:
        # (profiles "speed accel,speed accel": blanks become ';')
        line += "CFTYPE %s %s %s\n" % (t["id"], cfm, " ".join("%s=%s" % (k, ";".join(v.split())) for k, v in sorted(cf_attrs.items())))
    if cfm == "Rail":
        # MSCFModel_Rail: the train type replaces the defaults of length, maxSpeed, decel and mass
        rail = ["lengthSet=%d" % (t.get("length") is not None), "maxSpeedSet=%d" % (t.get("maxSpeed") is not None),
                "decelSet=%d" % (t.get("decel") is not None), "emergencyDecelSet=%d" % (t.get("emergencyDecel") is not None),
                "massSet=%d" % (t.get("mass") is not None)]
        for k in ("mass", "massFactor", "maxPower", "maxTraction", "resCoef_constant", "resCoef_linear", "resCoef_quadratic",
                  "curveResistance", "curveIntegration", "roeckl_sharp_radius", "roeckl_numerator",
                  "roeckl_numerator_sharp", "roeckl_offset", "roeckl_offset_sharp"):
            if t.get(k) is not None:
                rail.append("%s=%r" % (k, float(t[k])))
        for k in ("speedTable", "tractionTable", "resistanceTable"):
            if t.get(k) is not None:
                rail.append("%s=%s" % (k, ",".join(t[k].replace(",", " ").split())))
        line += "RAILTYPE %s %s %s\n" % (t["id"], t.get("trainType", "NGT400"), " ".join(rail))
    return line


def parse_rerouter(el):
    """MSTriggeredRerouter definition: (attributes, [(begin, end, [psim2 lines of the interval])])"""
    if el.get("radius") is not None or el.get("optional", "false") in ("true", "1"):
        raise ValueError("rerouter %s: radius and optional are not supported" % el.get("id"))
    intervals = []
    for iv in el.findall("interval"):
        items = []
        for c in iv:
            if c.tag == "closingReroute":
                perm = -1 if c.get("allow") is None and c.get("disallow") is None else \
                    psim2net.perm_mask(c.get("allow"), c.get("disallow"))
                # until: the expected end of the closing (routing only)
                items.append(("CLOSE", c.get("id"), "%d %s" % (perm, c.get("until", "-1"))))
            elif c.tag == "closingLaneReroute":
                if c.get("allow") is None and c.get("disallow") is None:
                    perm = psim2net.perm_mask("authority", None)  # SVC_AUTHORITY
                else:
                    perm = psim2net.perm_mask(c.get("allow"), c.get("disallow"))
                items.append(("CLOSELANE", c.get("id"), perm))
            elif c.tag == "destProbReroute":
                items.append(("DEST", c.get("id"), float(c.get("probability", "1"))))
            elif c.tag == "routeProbReroute":
                items.append(("ROUTE", c.get("id"), float(c.get("probability", "1"))))
            elif c.tag == "viaProbReroute":
                items.append(("VIA", c.get("id"), float(c.get("probability", "1"))))
            elif c.tag == "parkingAreaReroute":
                items.append(("PARK", c.get("id"), "%r %d" % (float(c.get("probability", "1")),
                                                            c.get("visible", "false") in ("true", "1"))))
            elif c.tag == "overtakingReroute":
                # MSTriggeredRerouter: a siding where a slow train lets a faster one overtake (minSaving default
                # 300 s, defer default: true unless it is the first location of the interval)
                items.append(("OVERTAKE", ",".join(c.get("main").split()), "%s %s %s" % (
                    ",".join(c.get("siding").split()), c.get("minSaving", "300"),
                    {"true": "1", "1": "1", "false": "0", "0": "0"}.get(c.get("defer"), "-"))))
            elif c.tag == "stationReroute":
                items.append(("STATION", c.get("id"), "1"))
            elif c.tag == "param":
                pass
            else:
                raise ValueError("rerouter %s: %s not supported" % (el.get("id"), c.tag))
        intervals.append((iv.get("begin", "-1"), iv.get("end", "1e18"), items))
    return (dict(el.attrib), intervals)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("net")
    ap.add_argument("routes")
    ap.add_argument("out")
    ap.add_argument("--additional")
    ap.add_argument("--step", type=float, default=1.0)
    ap.add_argument("--begin", type=float, default=0)
    ap.add_argument("--end", type=float, default=1e18)
    ap.add_argument("--full-net", action="store_true")
    ap.add_argument("--no-internal-links", action="store_true", help="SUMO --no-internal-links")
    ap.add_argument("--routing", action="store_true", help="vehicles are rerouted (full network)")
    ap.add_argument("--default-departlane", default="best_prob", help="SUMO --default.departlane")
    ap.add_argument("--default-departspeed", default="avg", help="SUMO --default.departspeed")
    ap.add_argument("--default-speeddev", type=float, help="SUMO --default.speeddev")
    ap.add_argument("--sigma", type=float)
    ap.add_argument("--use-stop-ended", action="store_true", help="SUMO --use-stop-ended")
    ap.add_argument("--speeddev", type=float)
    ap.add_argument("--scale", type=float, default=1.0, help="SUMO --scale (demand scaling)")
    ap.add_argument("--junction-taz", action="store_true", help="SUMO --junction-taz")
    ap.add_argument("--carfollow-model", default="Krauss", help="SUMO --carfollow.model (the default model)")
    ap.add_argument("--with-taz", action="store_true", help="SUMO --device.rerouting.with-taz (--with-taz)")
    ap.add_argument("--mapmatch-distance", type=float, default=100, help="SUMO --mapmatch.distance")
    ap.add_argument("--mapmatch-junctions", action="store_true", help="SUMO --mapmatch.junctions")
    ap.add_argument("--mapmatch-taz", action="store_true", help="SUMO --mapmatch.taz")
    ap.add_argument("--weight-files", help="SUMO --weight-files (edge travel times per interval)")
    a = ap.parse_args()
    if a.mapmatch_junctions:
        a.junction_taz = True  # MSFrame::checkOptions
    matcher_cache = []

    def matcher():
        """positions to edges (only built when a trip or stop is given by positions)"""
        if not matcher_cache:
            matcher_cache.append(mapmatch.MapMatcher(a.net, a.mapmatch_junctions, a.mapmatch_taz, a.mapmatch_distance))
        return matcher_cache[0]

    def geo_positions(veh_el, positions, geo, is_from, force_edge=False):
        """MapMatcher::parseGeoEdges for the class of the vehicle's type (passenger for distributions)"""
        t = types.get(veh_el.get("type", "DEFAULT_VEHTYPE"), {})
        vbit = psim2net.VCLASS_BITS[t.get("vClass", "passenger")]
        return matcher().match(positions, geo, vbit, is_from, "for %s '%s'" % (veh_el.tag, veh_el.get("id")), tazs_def,
                               force_edge)

    def geo_match(el, attr_xy, attr_lonlat, is_from):
        """the edges (or zone connectors) of the positions in attr_xy / attr_lonlat, None without them"""
        value, geo = el.get(attr_xy), False
        if value is None:
            value, geo = el.get(attr_lonlat), True
        if value is None:
            return None
        return geo_positions(el, [tuple(float(c) for c in p.split(",")[:2]) for p in value.split()], geo, is_from)
    global DEFAULT_SPEEDDEV, DEFAULT_CF_MODEL
    DEFAULT_SPEEDDEV = a.default_speeddev
    DEFAULT_CF_MODEL = a.carfollow_model

    type_dists = {}      # vTypeDistribution id -> [(type, probability)]
    types = {"DEFAULT_VEHTYPE": {"id": "DEFAULT_VEHTYPE"}, "DEFAULT_BUSTYPE": {"id": "DEFAULT_BUSTYPE", "vClass": "bus"},
             "DEFAULT_PEDTYPE": {"id": "DEFAULT_PEDTYPE", "vClass": "pedestrian"},
             "DEFAULT_BIKETYPE": {"id": "DEFAULT_BIKETYPE", "vClass": "bicycle"},
             "DEFAULT_TAXITYPE": {"id": "DEFAULT_TAXITYPE", "vClass": "taxi"},
             "DEFAULT_RAILTYPE": {"id": "DEFAULT_RAILTYPE", "vClass": "rail"},
             "DEFAULT_CONTAINERTYPE": {"id": "DEFAULT_CONTAINERTYPE", "vClass": "container"}}
    routes = {}          # id -> edges
    trips = set()        # trip ids: implicit route from (via) to, routed by psim2 (MSRouteHandler::parseFromViaTo)
    vias = {}            # vehicle / flow id -> via edges
    veh_speed_factor = {}  # vehicle / flow id -> the given speedFactor
    route_stops = {}     # id -> [stop attrs]
    vehicles = []        # (depart, order, id, type, route, departLane, departSpeed, departPos, arrivalPos, stops, untilOffset,
    #                       arrivalLane, arrivalSpeed)
    flows = []           # random flows: (begin, order, id, type, route, departLane, departSpeed, departPos, arrivalPos,
    #                       arrivalLane, arrivalSpeed, stops, kind, value, end, number)
    persons = []         # (depart, order, id, type, departPos, speedFactor, stages)
    flow_of = {}         # vehicle id -> flow id (vehicles of deterministic flows)
    veh_params = {}      # vehicle / flow id -> generic parameters
    veh_badges = {}      # vehicle / flow id -> parkingBadges
    init_replaced = set()  # trips whose initial routing replaces the loaded route in any case
    rerouters = []       # (attributes, intervals)
    speed_signs = []     # (id, lanes, ["time:speed"])
    anon = 0
    order = 0
    loaded = [0]  # MSVehicleControl::myLoadedVehNo

    def scaling_quota(frac, loaded_before):
        """getScalingQuota (StdDefs.cpp)"""
        if frac < 0 or frac == 1:
            return 1
        base = int(frac)
        intFrac = int(math.floor((frac - base) * 1000 + 0.5))
        return base + 1 if ((loaded_before % 1000) * intFrac) % 1000 < intFrac else base

    implicit_routes = set()  # routes defined by their stops only
    # public transport schedules for the intermodal router (MSInsertionControl / MSVehicleControl::adaptIntermodalRouter):
    # (flow?, order, id, line, begin_ms, repetitionNumber, offset_ms, lastDepart_ms, [(busStop, until_ms)], vClass)
    pt_scheds = []

    def trip_attrs(c, person):
        """MSRouteHandler::addPersonTrip: the modes and vTypes default to those of the person"""
        at = dict(c.attrib)
        at["_modes"] = c.get("modes", person.get("modes", ""))
        at["_vtypes"] = c.get("vTypes", person.get("vTypes", ""))
        # MSRouteHandler::parseFromViaTo: positions matched with the person's class
        for side, is_from in (("from", True), ("to", False)):
            value, geo = c.get(side + "XY"), False
            if value is None:
                value, geo = c.get(side + "LonLat"), True
            if value is not None and c.get(side) is None:
                matched = person_geo(person, [tuple(float(v) for v in p.split(",")[:2]) for p in value.split()], geo, is_from)
                at[side] = matched[0] if is_from else matched[-1]
        # MSRouteHandler::parseFromViaTo: zones (loaded taz or junctions with --junction-taz) by their connectors
        for side, suffix in (("from", "-source"), ("to", "-sink")):
            zone = c.get(side + "Taz") or c.get(side + "Junction")
            if zone is not None and (a.with_taz or c.get(side) is None):
                if zone not in tazs_def:
                    raise ValueError("%s zone '%s' not known for person '%s'" % (("Source", "Sink")[side == "to"], zone, person.get("id")))
                at[side] = zone + suffix
        return at

    def sched_stops(route_st, own_st, depart_ms):
        """IntermodalNetwork::addSchedule: route stops count from the departure, the vehicle's own stops are absolute
        (until -1 when not given); only bus and train stops"""
        res = []
        for st, offset in [(s, depart_ms) for s in route_st] + [(s, 0) for s in own_st]:
            place = st.get("busStop", st.get("trainStop"))
            until = time2steps(parse_time(st["until"])) if st.get("until") is not None else -1
            res.append((place or "", until + offset))
        return res

    def person_geo(person, positions, geo, is_from, force_edge=False):
        """MapMatcher::parseGeoEdges for the class of the person's type"""
        t = types.get(person.get("type", "DEFAULT_PEDTYPE"), {})
        vbit = psim2net.VCLASS_BITS[t.get("vClass", "pedestrian")]
        return matcher().match(positions, geo, vbit, is_from, "for person '%s'" % person.get("id"), tazs_def, force_edge)

    def stop_edge(st):
        if "lane" in st:
            return st["lane"].rsplit("_", 1)[0]
        if "edge" in st:
            return st["edge"]
        for key in ("busStop", "trainStop", "parkingArea", "containerStop", "chargingStation"):
            if st.get(key) in stop_lanes:
                return stop_lanes[st[key]].rsplit("_", 1)[0]
        raise ValueError("stop without a known place: %s" % st)

    def repeat_route(rid, el):
        """MSRouteHandler::closeRoute: a route with repeat is driven repeat + 1 times, the 'until' times of its
        stops shift by cycleTime per repetition"""
        repeat = int(el.get("repeat", "0"))
        if repeat <= 0:
            return
        edges, stops = list(routes[rid]), list(route_stops[rid])
        period = float(el.get("cycleTime", "0"))
        for i in range(repeat):
            routes[rid] = routes[rid] + edges
            for st in stops:
                st = dict(st)
                for key in ("until", "arrival"):
                    if st.get(key) is not None and float(st[key]) > 0:
                        if period <= 0:
                            raise ValueError("Cannot repeat stops with '%s' in route %s because no cycleTime is defined." % (key, rid))
                        st[key] = repr(float(st[key]) + period * (i + 1))
                route_stops[rid].append(st)

    sources = (a.additional.split(",") if a.additional else []) + [r for r in a.routes.split(",") if r]
    # stopping places first (the implicit routes of trips with stops need their lanes)
    stop_lanes = {}
    # zones: id -> (sources, sinks); junction zones first (NLBuilder, by junction id), then the loaded ones
    tazs_def = {}
    if a.junction_taz:
        for jid, (inc, out_) in sorted(psim2net.junction_edges(a.net).items()):
            tazs_def[jid] = (out_, inc)
    veh_zones = {}  # vehicle / flow id -> {"fromtaz": id, "totaz": id}
    triggered_persons = set()  # persons with depart="triggered"
    container_ids = set()  # transportables that are containers
    triggered_vehicles = set()  # vehicles with depart="triggered" (or "split")
    split_vehicles = set()  # vehicles with depart="split" (parts of a train that is split at a stop)
    veh_lines = {}  # vehicle / flow id -> line (public transport line, matched by rides)
    veh_depart_edge = {}  # vehicle / flow id -> departEdge (route index)
    veh_ichecks = {}  # vehicle / flow id -> insertionChecks
    veh_poslat = {}  # vehicle / flow id -> [VOPT keys of departPosLat / arrivalPosLat]
    full_route_of = {}  # vehicle id -> the whole route of a vehicle with arrivalEdge
    preferences = []  # PREFERENCE lines (routing preferences of routing types)
    for path in sources:
        with open_xml(path) as f:
            for ev, el in ET.iterparse(f):
                if el.tag in ("busStop", "trainStop", "parkingArea", "containerStop", "chargingStation"):
                    stop_lanes[el.get("id")] = el.get("lane")
                    el.clear()
                elif el.tag == "taz":
                    # NLHandler::addDistrict / addDistrictEdge: 'edges' are sources and sinks
                    srcs = el.get("edges", "").split()
                    sinks = list(srcs)
                    for c in el:
                        if c.tag == "tazSource":
                            srcs.append(c.get("id"))
                        elif c.tag == "tazSink":
                            sinks.append(c.get("id"))
                    tazs_def[el.get("id")] = (srcs, sinks)
                    el.clear()
    for path in sources:
        with open_xml(path) as f:
            stack = []
            for ev, el in ET.iterparse(f, events=("start", "end")):
                if ev == "start":
                    stack.append(el)
                    continue
                stack.pop()
                parent = stack[-1] if stack else None
                if el.tag == "preference":
                    # NLHandler: routing preference of a routing type (for classes, types or everybody)
                    preferences.append("PREFERENCE %s %s %s %s\n" % (
                        el.get("routingType"), ",".join(el.get("vClasses", "").split()) or "-",
                        ",".join(el.get("vTypes", "").split()) or "-", el.get("priority")))
                if el.tag == "rerouter":
                    rerouters.append(parse_rerouter(el))
                    el.clear()
                    continue
                if el.tag == "variableSpeedSign":
                    # MSLaneSpeedTrigger: lane speeds by time (a step without speed restores the default)
                    if el.get("file"):
                        raise ValueError("variableSpeedSign %s: steps in a file are not supported" % el.get("id"))
                    steps = []
                    for st in el.findall("step"):
                        if st.get("friction") is not None and st.get("speed") is None:
                            continue
                        steps.append("%s:%s" % (st.get("time"), st.get("speed", "-")))
                    speed_signs.append((el.get("id"), el.get("lanes").split(), steps))
                    el.clear()
                    continue
                if el.tag == "vTypeDistribution":
                    # MSRouteHandler::openVehicleTypeDistribution / closeVehicleType: nested types with their
                    # probability, or the ids of types (or distributions) with the given probabilities
                    members = [(c.get("id"), float(c.get("probability", "1"))) for c in el.findall("vType")]
                    if el.get("vTypes"):
                        probs = [float(x) for x in el.get("probs", "").split()]
                        for i, tid in enumerate(el.get("vTypes").split()):
                            if tid in type_dists:
                                sub = type_dists[tid]
                                total = sum(p for _, p in sub)
                                f = (probs[i] if i < len(probs) else 1.) / total
                                members += [(t, f * p) for t, p in sub]
                            else:
                                members.append((tid, probs[i] if i < len(probs) else float(types[tid].get("probability", "1"))))
                    type_dists[el.get("id")] = members
                    el.clear()
                    continue
                if el.tag == "vType":
                    types[el.get("id")] = dict(el.attrib, _params={p.get("key"): p.get("value") for p in el.findall("param")})
                    for c in el:
                        if c.tag.startswith("carFollowing-"):
                            # SUMOVehicleParserHelper::parseCFMParams: the model element and its attributes
                            types[el.get("id")].update(c.attrib, carFollowModel=c.tag[len("carFollowing-"):])
                elif el.tag in ("person", "personFlow", "container", "containerFlow"):
                    # (containers are transported like riding persons; psim2 does not output them)
                    is_container = el.tag.startswith("container")
                    stages = []
                    for c in el:
                        if c.tag == "walk":
                            if c.get("edges") is not None:
                                edges_ = c.get("edges").split()
                            elif c.get("route") is not None:
                                edges_ = routes[c.get("route")]
                            else:
                                # MSRouteHandler::addPersonTrip: routed by psim2 when the walk starts
                                stages.append(("trip", None, trip_attrs(c, el)))
                                continue
                            stages.append(("walk", edges_, dict(c.attrib)))
                        elif c.tag == "personTrip":
                            stages.append(("trip", None, trip_attrs(c, el)))
                        elif c.tag == "stop":
                            st = dict(c.attrib)
                            if "edge" not in st and "lane" not in st and ((st.get("x") and st.get("y"))
                                                                          or (st.get("lon") and st.get("lat"))):
                                # MSRouteHandler::addStop: the matched edge, the end at the nearest offset of its first lane
                                geo = st.get("x") is None
                                pos = (float(st["lon"]), float(st["lat"])) if geo else (float(st["x"]), float(st["y"]))
                                # (the person waits at the arrival position of the previous stage)
                                st["edge"] = person_geo(el, [pos], geo, True, True)[0]
                            stages.append(("stop", None, st))
                        elif c.tag in ("ride", "transport"):
                            stages.append(("ride", None, dict(c.attrib)))
                        elif c.tag == "tranship":
                            stages.append(("tranship", c.get("edges").split() if c.get("edges") else None, dict(c.attrib)))
                    common_p = (el.get("type", "DEFAULT_CONTAINERTYPE" if is_container else "DEFAULT_PEDTYPE"), el.get("departPos", "-"),
                                el.get("speedFactor", "-"), stages)
                    first_new = len(persons)
                    if el.tag in ("person", "container"):
                        # depart="triggered": the person starts in the vehicle of its first ride (written as such)
                        dep = el.get("depart")
                        persons.append((-1.0 if dep == "triggered" else parse_time(dep), order, el.get("id")) + common_p + (None,))
                        if dep == "triggered":
                            triggered_persons.add(el.get("id"))
                        order += 1
                    elif el.get("probability") is not None or (el.get("period") or "").startswith("exp("):
                        # random flow: generated by psim2 with its own random numbers (MSRouteHandler::closeTransportableFlow)
                        if el.get("probability") is not None:
                            spec = ("prob", float(el.get("probability")))
                        else:
                            spec = ("exp", float(el.get("period")[4:-1]))
                        end = parse_time(el.get("end")) if el.get("end") is not None else 1e18
                        number = int(el.get("number")) if el.get("number") is not None else -1
                        persons.append((parse_time(el.get("begin", "0")), order, el.get("id")) + common_p + (spec + (end, number),))
                        order += 1
                    else:
                        # SUMOVehicleParserHelper::parseFlowAttributes, MSRouteHandler::closeTransportableFlow (milliseconds)
                        begin_ms = time2steps(parse_time(el.get("begin", "0")))
                        number = int(el.get("number")) if el.get("number") is not None else None
                        end = el.get("end")
                        end_ms = time2steps(parse_time(end)) if end is not None else (begin_ms + time2steps(86400.) if number is None else None)
                        period = el.get("period")
                        # (perHour: the generic name, SUMOVehicleParserHelper::parseFlowAttributes)
                        for key in ("personsPerHour", "containersPerHour", "perHour"):
                            if el.get(key) is not None:
                                period = str(3600.0 / float(el.get(key)))
                        if period is not None:
                            offset_ms = time2steps(parse_time(period))
                        elif number is not None and end_ms is not None:
                            offset_ms = (end_ms - begin_ms) // number
                        else:
                            raise ValueError("personFlow %s: no period, personsPerHour or number" % el.get("id"))
                        if number is None:
                            number = int(math.ceil((end_ms - begin_ms) / offset_ms))
                        k = 0
                        while k < number:
                            persons.append(((begin_ms + k * offset_ms) / 1000.0, order, "%s.%d" % (el.get("id"), k)) + common_p + (None,))
                            order += 1
                            k += 1
                    if is_container:
                        container_ids.update(q[2] for q in persons[first_new:])
                    el.clear()
                elif el.tag == "route" and parent is not None and parent.tag in ("vehicle", "flow"):
                    pass  # handled with the parent
                elif el.tag == "route":
                    route_stops[el.get("id")] = [stop_dict(s) for s in el.findall("stop")]
                    if el.get("edges") is None:
                        # MSRouteHandler::closeRoute: a route without edges consists of its stop edges, the vehicles
                        # are routed before the insertion (VEHPARS_FORCE_REROUTE)
                        if not route_stops[el.get("id")]:
                            raise ValueError("Route '%s' has no edges." % el.get("id"))
                        routes[el.get("id")] = [stop_edge(st) for st in route_stops[el.get("id")]]
                        implicit_routes.add(el.get("id"))
                    else:
                        routes[el.get("id")] = el.get("edges").split()
                    repeat_route(el.get("id"), el)
                    el.clear()
                elif el.tag in ("vehicle", "flow", "trip"):
                    veh_params[el.get("id")] = {q.get("key"): q.get("value") for q in el.findall("param")}
                    if el.get("insertionChecks") is not None:
                        veh_ichecks[el.get("id")] = ",".join(el.get("insertionChecks").replace(",", " ").split()) or "none"
                    for attr, key in (("departPosLat", "dposlat"), ("arrivalPosLat", "aposlat")):
                        if el.get(attr) is not None:
                            veh_poslat.setdefault(el.get("id"), []).append("%s=%s" % (key, el.get(attr)))
                    if el.get("departEdge") is not None:
                        if not el.get("departEdge").isdigit():
                            raise ValueError("%s %s: departEdge %s not supported" % (el.tag, el.get("id"), el.get("departEdge")))
                        veh_depart_edge[el.get("id")] = el.get("departEdge")
                    if el.get("line") is not None:
                        veh_lines[el.get("id")] = el.get("line")
                    if el.get("parkingBadges") is not None:
                        veh_badges[el.get("id")] = el.get("parkingBadges")
                    if el.tag in ("vehicle", "trip", "flow"):
                        # stops and via edges given by positions (MSRouteHandler::addStop, parseFromViaTo); the end
                        # position computed for a stop is replaced by the default (the edge end) in SUMO 1.28
                        for st in el.findall("stop"):
                            if st.get("x") is not None and st.get("y") is not None:
                                st.set("edge", geo_positions(el, [(float(st.get("x")), float(st.get("y")))], False, True, True)[0])
                            elif st.get("lon") is not None and st.get("lat") is not None:
                                st.set("edge", geo_positions(el, [(float(st.get("lon")), float(st.get("lat")))], True, True, True)[0])
                        via_geo = geo_match(el, "viaXY", "viaLonLat", False)
                        if via_geo is not None:
                            el.set("via", " ".join(via_geo))
                    if el.get("viaJunctions") and not el.get("via"):
                        # MSRouteHandler::parseFromViaTo: via the sink connectors of junction zones
                        el.set("via", " ".join(j + "-sink" for j in el.get("viaJunctions").split()))
                    if el.get("via") and (el.tag == "flow" or (el.get("route") is None and el.find("route") is None)):
                        # via edges of trips and flows (SUMO 1.28 ignores via of vehicles with a route)
                        vias[el.get("id")] = el.get("via").split()
                    rid = el.get("route")
                    stops = [stop_dict(s) for s in el.findall("stop")]
                    nested = el.find("route")
                    if rid is None and nested is None:
                        # a trip: implicit route from (via) to, routed before the insertion
                        from_, to_ = el.get("from"), el.get("to")
                        # MSRouteHandler::parseFromViaTo: positions (matched to edges, junction or taz connectors)
                        # instead of from / to; zones take precedence only with --with-taz or without any of them
                        for attrs_, is_from in ((("fromXY", "fromLonLat", "fromTaz", "fromJunction"), True),
                                                (("toXY", "toLonLat", "toTaz", "toJunction"), False)):
                            if a.with_taz and (el.get(attrs_[2]) or el.get(attrs_[3])):
                                continue
                            matched = geo_match(el, attrs_[0], attrs_[1], is_from)
                            if matched is not None:
                                if is_from:
                                    from_ = matched[0]
                                else:
                                    to_ = matched[-1]
                        no_ends = from_ is None and to_ is None and stops and el.get("via") is None and not any(
                            k in el.attrib for k in ("fromTaz", "toTaz", "fromJunction", "toJunction", "fromXY", "toXY",
                                                     "fromLonLat", "toLonLat"))
                        geo = any(k in el.attrib for k in ("fromTaz", "toTaz", "fromJunction", "toJunction", "fromXY", "toXY",
                                                            "fromLonLat", "toLonLat"))
                        # MSRouteHandler::parseFromViaTo: the stop edges follow 'from' (a missing 'to': the last stop)
                        half_open = stops and el.get("via") is None and not geo and (from_ is None) != (to_ is None)
                        # MSRouteHandler::parseFromViaTo: zones (taz, or junctions with --junction-taz) instead of edges
                        # (a junction is a zone with --junction-taz or a loaded taz with the junction's id)
                        zone_from = el.get("fromTaz") or el.get("fromJunction")
                        zone_to = el.get("toTaz") or el.get("toJunction")
                        # MSRouteHandler::parseFromViaTo errors (SUMO quits)
                        for zone, attr_edge, kind, side in ((zone_from, from_, "Source", 0), (zone_to, to_, "Sink", 1)):
                            if zone is not None and (a.with_taz or attr_edge is None):
                                if zone not in tazs_def:
                                    raise ValueError("%s zone '%s' not known for %s '%s'" % (kind, zone, el.tag, el.get("id")))
                                if not tazs_def[zone][side]:
                                    raise ValueError("%s zone '%s' has no %s edges for %s '%s'" % (
                                        kind, zone, ("outgoing", "incoming")[side], el.tag, el.get("id")))
                        # the connector edges of a zone (NLHandler::addDistrict: "<taz>-source", "<taz>-sink") given
                        # as from / to edges (or matched positions) act like fromTaz / toTaz
                        if from_ is not None and from_.endswith("-source") and from_[:-7] in tazs_def:
                            zone_from, from_ = from_[:-7], None
                        if to_ is not None and to_.endswith("-sink") and to_[:-5] in tazs_def:
                            zone_to, to_ = to_[:-5], None
                        if zone_from is not None and (a.with_taz or from_ is None) and tazs_def.get(zone_from, ([], []))[0]:
                            veh_zones.setdefault(el.get("id"), {})["fromtaz"] = zone_from
                            from_ = tazs_def[zone_from][0][0]
                        if zone_to is not None and (a.with_taz or to_ is None) and tazs_def.get(zone_to, ([], []))[1]:
                            veh_zones.setdefault(el.get("id"), {})["totaz"] = zone_to
                            to_ = tazs_def[zone_to][1][0]
                        if from_ is None and to_ is None and el.get("via"):
                            # a trip given by its via edges only: the route consists of them (parseFromViaTo)
                            from_, to_ = el.get("via").split()[0], el.get("via").split()[-1]
                        # (only 'from': the route starts there and ends after the via edges, if any)
                        from_only = from_ is not None and to_ is None and not geo and not stops
                        if not no_ends and not half_open and not from_only and (from_ is None or to_ is None):
                            raise ValueError("%s %s: only trips with from and to edges, zones or positions are supported"
                                             % (el.tag, el.get("id")))
                        via = el.get("via", "").split()
                        rid = "!%s" % el.get("id")
                        if stops and not via:
                            # MSRouteHandler::addStop (myInsertStopEdgesAt): the stop edges between from and to (a trip
                            # without from and to consists of its stop edges)
                            stop_edges = []
                            for s in stops:
                                if "lane" in s:
                                    stop_edges.append(s["lane"].rsplit("_", 1)[0])
                                elif "edge" in s:
                                    stop_edges.append(s["edge"])
                                elif s.get("busStop", s.get("trainStop", s.get("parkingArea"))) in stop_lanes:
                                    stop_edges.append(stop_lanes[s.get("busStop", s.get("trainStop", s.get("parkingArea")))]
                                                      .rsplit("_", 1)[0])
                                else:
                                    raise ValueError("trip %s: only stops on lanes, edges or bus stops are supported"
                                                     % el.get("id"))
                            if no_ends:
                                routes[rid] = stop_edges
                            else:
                                mid = [e for i, e in enumerate(stop_edges) if (i > 0 or e != from_)
                                       and (i + 1 < len(stop_edges) or e != to_)]
                                routes[rid] = ([from_] if from_ else []) + mid + ([to_] if to_ else [])
                                # SUMO's initial route keeps repeated edges, so its first routing always counts as a
                                # route replacement (rerouteNo)
                                raw = ([from_] if from_ else []) + stop_edges + ([to_] if to_ else [])
                                if any(raw[i] == raw[i + 1] for i in range(len(raw) - 1)):
                                    init_replaced.add(el.get("id"))
                        else:
                            # (taz connectors among the via edges are no edges of the route)
                            routes[rid] = [from_] + [e for e in via if not (e.endswith(("-source", "-sink"))
                                                                         and e.rsplit("-", 1)[0] in tazs_def)] + ([to_] if to_ else [])
                        route_stops[rid] = []
                        trips.add(el.get("id"))
                    elif nested is not None:
                        anon += 1
                        rid = "!%s" % el.get("id")
                        routes[rid] = nested.get("edges").split()
                        route_stops[rid] = [stop_dict(s) for s in nested.findall("stop")]
                        repeat_route(rid, nested)
                    if rid in implicit_routes:
                        # MSRouteHandler::addVehicleStopsToImplicitRoute: the vehicle's stops extend the route
                        edges = list(routes[rid])
                        for st in stops:
                            e = stop_edge(st)
                            if st.get("index") == "0":
                                if edges[0] != e:
                                    edges.insert(0, e)
                            elif edges[-1] != e:
                                edges.append(e)
                        own = "!%s" % el.get("id")
                        routes[own] = edges
                        route_stops[own] = route_stops[rid]
                        rid = own
                        trips.add(el.get("id"))
                        if any(edges[i] == edges[i + 1] for i in range(len(edges) - 1)):
                            init_replaced.add(el.get("id"))
                    # MSBaseVehicle::addStops: 'until' of route stops counts from the departure,
                    # 'until' of vehicle stops from the flow repetition offset
                    allstops = [dict(s, _route=True) for s in route_stops.get(rid, [])]
                    for st in stops:
                        # MSBaseVehicle::addStop: index 0 / a number / "end" (default) / "fit" (appended here)
                        idx = st.get("index", "end")
                        if idx not in ("end", "fit"):
                            if int(idx) > len(allstops):
                                # appended, skipping occurrences of the stop edge in a looped route
                                st = dict(st, _skip=int(idx) - len(allstops))
                            allstops.insert(min(int(idx), len(allstops)), st)
                        else:
                            allstops.append(st)
                    if el.get("arrivalEdge") is not None:
                        # arrivalEdge: the vehicle arrives on this route index (MSVehicle::hasArrivedInternal; the
                        # rest of the route is not driven)
                        if not el.get("arrivalEdge").isdigit():
                            raise ValueError("%s %s: arrivalEdge %s not supported" % (el.tag, el.get("id"), el.get("arrivalEdge")))
                        own = "!%s" % el.get("id")
                        cut = int(el.get("arrivalEdge")) + 1
                        # (driveways are built along the whole route)
                        full_route_of[el.get("id")] = "!%s.full" % el.get("id")
                        routes[full_route_of[el.get("id")]] = list(routes[rid])
                        routes[own] = routes[rid][:cut]
                        route_stops[own] = route_stops.get(rid, [])
                        rid = own
                    if el.get("speedFactor") is not None:
                        veh_speed_factor[el.get("id")] = repr(float(el.get("speedFactor")))
                    common = (el.get("type", "DEFAULT_VEHTYPE"), rid, el.get("departLane", a.default_departlane),
                              el.get("departSpeed", a.default_departspeed), el.get("departPos", "base"), el.get("arrivalPos", "max"))
                    arrival = (el.get("arrivalLane", "current"), el.get("arrivalSpeed", "current"))
                    frac = a.scale * float(types.get(common[0], {}).get("scale", "1"))
                    if el.tag in ("vehicle", "trip"):
                        if el.get("depart") in ("triggered", "split"):
                            # inserted when a person boards or when the train it splits from reaches the stop
                            # (psim2 keeps it waiting on its first edge)
                            depart = 1e18
                            triggered_vehicles.add(el.get("id"))
                            if el.get("depart") == "split":
                                split_vehicles.add(el.get("id"))
                        else:
                            depart = parse_time(el.get("depart"))
                            if el.get("line") and types.get(common[0], {}).get("vClass") != "taxi":
                                depart_ms = time2steps(depart)
                                pt_scheds.append((False, order, el.get("id"), el.get("line"), depart_ms, -1, -1, depart_ms,
                                                  sched_stops(route_stops.get(rid, []), stops, depart_ms)))
                        # MSRouteHandler::closeVehicle with MSVehicleControl::getQuota: copies "<id>.<i>"
                        loaded[0] += 1
                        quota = scaling_quota(frac, int(loaded[0] / frac) if frac > 1 else loaded[0] - 1)
                        for i in range(quota):
                            vid = el.get("id") if i == 0 else "%s.%d" % (el.get("id"), i)
                            if i > 0:
                                loaded[0] += 1
                                for src in (trips, init_replaced):
                                    if el.get("id") in src:
                                        src.add(vid)
                                for src in (vias, veh_params, veh_badges, veh_speed_factor):
                                    if el.get("id") in src:
                                        src[vid] = src[el.get("id")]
                            vehicles.append((depart, order, vid) + common + (allstops, 0.0) + arrival)
                            order += 1
                    else:
                        begin = parse_time(el.get("begin", "0"))
                        number = int(el.get("number")) if el.get("number") is not None else None
                        end = parse_time(el.get("end")) if el.get("end") is not None else (86400.0 if number is None else None)
                        period = el.get("period")
                        for key in ("vehsPerHour", "perHour"):
                            if el.get(key) is not None:
                                period = str(3600.0 / float(el.get(key)))
                        if period is not None and period.startswith("exp("):
                            # Poisson arrivals: generated by psim2 with its own random numbers
                            flows.append((begin, order, el.get("id")) + common + arrival
                                         + (allstops, "exp", float(period[4:-1]), end if end is not None else 1e18, number or -1))
                            order += 1
                        elif el.get("probability") is not None:
                            flows.append((begin, order, el.get("id")) + common + arrival
                                         + (allstops, "prob", float(el.get("probability")), end if end is not None else 1e18, number or -1))
                            order += 1
                        else:
                            # SUMOVehicleParserHelper::parseFlowAttributes, MSInsertionControl: all in milliseconds
                            begin_ms = time2steps(begin)
                            if period is not None:
                                offset_ms = time2steps(parse_time(period))
                                end_ms = time2steps(end) if end is not None else None
                            elif number is not None and end is None:
                                if number != 1:
                                    raise ValueError("flow %s: number without end or period" % el.get("id"))
                                end_ms, offset_ms = None, 0
                            elif number is not None:
                                end_ms = time2steps(end)
                                offset_ms = (end_ms - begin_ms) // number
                            else:
                                raise ValueError("flow %s: no period, vehsPerHour, probability or number" % el.get("id"))
                            if el.get("line") and el.get("line") != "taxi":
                                rep = number if number is not None else (
                                    int(math.ceil((end_ms - begin_ms) / offset_ms)) if end_ms is not None and offset_ms > 0 else 1)
                                pt_scheds.append((True, order, el.get("id"), el.get("line"), begin_ms, rep, offset_ms,
                                                  begin_ms + (rep - 1) * offset_ms,
                                                  sched_stops(route_stops.get(rid, []), stops, begin_ms)))
                            k = 0
                            limit = number
                            if frac != 1:
                                # MSInsertionControl::determineCandidates with scale: (number * scale + 0.5) vehicles,
                                # the offset divided by the scale (SUMOVehicleParameter::incrementFlow)
                                if number is None and end_ms is not None and offset_ms > 0:
                                    number = int(math.ceil((end_ms - begin_ms) / offset_ms))
                                if number is not None:
                                    limit = int(number * frac + 0.5)
                                    end_ms = None
                                offset_ms = int(offset_ms / frac)
                            while (end_ms is None or begin_ms + k * offset_ms < end_ms) and (limit is None or k < limit):
                                vehicles.append(((begin_ms + k * offset_ms) / 1000.0, order, "%s.%d" % (el.get("id"), k)) + common
                                                + (allstops, k * offset_ms / 1000.0) + arrival)
                                flow_of[vehicles[-1][2]] = el.get("id")
                                order += 1
                                k += 1
                    el.clear()
    vehicles = [v for v in vehicles if a.begin <= v[0] < a.end or v[2] in triggered_vehicles]
    # MSVehicleContainer: the departures of a step (rounded up to the step) in the order they were added: the loaded
    # vehicles in load order, then the vehicles of the flows (MSInsertionControl::determineCandidates, flow by flow)
    step_ms = time2steps(a.step)
    vehicles.sort(key=lambda v: (-(-time2steps(v[0]) // step_ms), v[2] in flow_of, v[1]))
    persons = [q for q in persons if ((a.begin <= q[0] or q[7] is not None) and q[0] < a.end) or q[2] in triggered_persons]
    # MSTransportableControl::add: departures wait for the next step, in load order (a personFlow adds all its
    # persons when it is loaded)
    persons.sort(key=lambda q: (-(-time2steps(q[0]) // step_ms), q[1]))
    flows = [f for f in flows if f[0] < a.end]
    used_types = {v[3] for v in vehicles} | {f[3] for f in flows} | {q[3] for q in persons}
    for q in persons:
        # MSStageTrip::getVehicles: the vehicle types of the trip, or the default car and bicycle types of its modes
        for kind, _, at in q[6]:
            if kind == "trip":
                if at.get("_vtypes"):
                    used_types.update(at["_vtypes"].split())
                else:
                    modes = at.get("_modes", "").split()
                    if "car" in modes:
                        used_types.add("DEFAULT_VEHTYPE")
                    if "bicycle" in modes:
                        used_types.add("DEFAULT_BIKETYPE")
    used_dists = sorted(t for t in used_types if t in type_dists)
    used_types = sorted({t for t in used_types if t not in type_dists} | {m for d in used_dists for m, _ in type_dists[d]})
    used_routes = sorted({v[4] for v in vehicles} | {f[4] for f in flows} | set(full_route_of.values())
                         | {r for _, ivs in rerouters for _, _, items in ivs for k, r, _ in items if k == "ROUTE"})
    used_edges = set()
    for r in used_routes:
        used_edges.update(routes[r])
    busstops = {}
    parkareas = {}  # id -> (attributes, [space attributes])
    tls_override = {}
    deadlocks = []  # rail signal deadlock hints (signal ids)
    constraints = []  # rail signal constraints (signal, kind, tripId, foe signal, limit, active, foe trips)
    if a.additional:
        for path in a.additional.split(","):
            with open_xml(path) as f:
                for ev, el in ET.iterparse(f):
                    if el.tag in ("busStop", "trainStop", "containerStop", "chargingStation"):
                        # (vehicles stop at container stops and charging stations like at bus stops)
                        busstops[el.get("id")] = dict(el.attrib, _tag=el.tag,
                                                      _access=[dict(c.attrib) for c in el.findall("access")])
                        el.clear()
                    elif el.tag == "parkingArea":
                        parkareas[el.get("id")] = (dict(el.attrib), [dict(c.attrib) for c in el.findall("space")])
                        el.clear()
                    elif el.tag == "deadlock":
                        deadlocks.append(el.get("signals").split())
                    elif el.tag == "railSignalConstraints":
                        # NLHandler::addPredecessorConstraint (the children are complete when the parent ends)
                        for c in el:
                            if c.tag in ("predecessor", "insertionPredecessor", "foeInsertion", "insertionOrder",
                                         "bidiPredecessor"):
                                foes = c.get("foes").split()
                                constraints.append((el.get("id"), c.tag, c.get("tripId"), c.get("tl"),
                                                    int(c.get("limit", str(len(foes)))),
                                                    c.get("active", "true") in ("true", "1"), foes))
                        el.clear()
                    elif el.tag == "tlLogic":
                        tls_override[el.get("id")] = (dict(el.attrib), [dict(p.attrib) for p in el.findall("phase")],
                                                      [(p.get("key"), p.get("value")) for p in el.findall("param")])
    used_stops = set()
    used_pas = {r for _, ivs in rerouters for _, _, items in ivs for k, r, _ in items if k == "PARK"}
    used_stops.update(r for _, ivs in rerouters for _, _, items in ivs for k, r, _ in items if k == "STATION")
    for v in vehicles + [f[:9] + (f[11],) for f in flows]:
        for s in v[9]:
            for key in ("trainStop", "containerStop", "chargingStation"):
                if key in s and "busStop" not in s:
                    s["busStop"] = s[key]
            if "busStop" in s:
                used_stops.add(s["busStop"])
            elif "parkingArea" in s:
                used_pas.add(s["parkingArea"])
            elif "lane" in s:
                used_edges.add(s["lane"].rsplit("_", 1)[0])
            elif "edge" in s:
                used_edges.add(s["edge"])
            else:
                raise ValueError("only stops at bus stops, lanes or edges are supported (vehicle %s)" % v[2])
    for q in persons:
        for kind, edges_, attrs in q[6]:
            if kind == "walk":
                used_edges.update(edges_)
            if kind == "trip":
                used_edges.update(e for e in (attrs.get("from"), attrs.get("to"))
                                  if e and not (e.endswith(("-source", "-sink")) and e.rsplit("-", 1)[0] in tazs_def))
            for key in ("busStop", "trainStop", "containerStop"):
                if attrs.get(key):
                    used_stops.add(attrs[key])
            if kind == "ride" and attrs.get("parkingArea"):
                used_pas.add(attrs["parkingArea"])
            if kind == "ride":
                used_edges.update(e for e in (attrs.get("from"), attrs.get("to")) if e)
            if kind == "stop" and attrs.get("edge"):
                used_edges.add(attrs["edge"])
            if kind == "stop" and attrs.get("lane"):
                used_edges.add(attrs["lane"].rsplit("_", 1)[0])
    if any("priority" in s for stops_ in [v[9] for v in vehicles] + [f[11] for f in flows] for s in stops_):
        # stops with priority may be replaced by a stopping place of the same name (MSNet::getStoppingPlaceAlternatives)
        names = {(busstops[sid].get("name"), busstops[sid]["_tag"] == "containerStop") for sid in used_stops}
        used_stops.update(sid for sid, b in busstops.items()
                          if b.get("name") and (b["name"], b["_tag"] == "containerStop") in names)
    have_trips = any(kind == "trip" for q in persons for kind, _, _ in q[6])
    if have_trips:
        # the intermodal router has access to all stopping places (MSNet::adaptIntermodalRouter)
        used_stops.update(busstops)
        used_pas.update(parkareas)
    for sid in used_stops:
        used_edges.add(busstops[sid]["lane"].rsplit("_", 1)[0])
    for pid in used_pas:
        used_edges.add(parkareas[pid][0]["lane"].rsplit("_", 1)[0])

    def stop_triggers(s):
        """SUMOVehicleParameter::parseStopTriggers: 'p' (person), 'c' (container); a stop without duration, until
        and speed is triggered by default"""
        trig = set()
        values = s.get("triggered", "").split()
        if not values and s.get("duration") is None and s.get("until") is None and s.get("speed") is None:
            trig.add("p")
        for v in values:
            if v == "person":
                trig.add("p")
            elif v == "container":
                trig.add("c")
            elif v == "join":
                trig.add("j")  # waits until another train joins (MSStop::joinTriggered)
            elif v == "split":
                pass
            elif sumo_bool(v):
                trig.add("p")
        if sumo_bool(s.get("containerTriggered", "false")):
            trig.add("c")
        return "".join(sorted(trig))

    def stop_parking(s):
        """the parking flag: given, or off the road for triggered stops, expected transportables, parking areas"""
        if s.get("parking") is not None:
            return "1" if (s["parking"] != "opportunistic" and sumo_bool(s["parking"])) or "parkingArea" in s else "0"
        return "1" if stop_triggers(s).replace("j", "") or s.get("expected") or s.get("expectedContainers") or "parkingArea" in s else "0"

    def stop_extras(s, until_offset):
        """optional STOP attributes: jump (seconds), jumpUntil (absolute like until), triggered, expected, permitted"""
        res = ""
        if stop_triggers(s):
            res += " triggered=" + stop_triggers(s)
        if s.get("expected"):
            res += " expected=" + ",".join(s["expected"].split())
        if s.get("expectedContainers"):
            res += " expectedContainers=" + ",".join(s["expectedContainers"].split())
        if s.get("permitted"):
            res += " permitted=" + ",".join(s["permitted"].split())
        if s.get("onDemand") is not None and sumo_bool(s["onDemand"]):
            res += " onDemand=1"
        if s.get("extension") is not None:
            res += " extension=%r" % parse_time(s["extension"])
        if s.get("jump") is not None:
            res += " jump=%r" % parse_time(s["jump"])
        if s.get("arrival") is not None:
            res += " arrival=%r" % (parse_time(s["arrival"]) + until_offset)
        if s.get("priority") is not None:
            res += " priority=%r" % float(s["priority"])  # may be skipped after rerouting (MSStopOptimizer)
        if s.get("split"):
            res += " split=" + s["split"]  # the train part that leaves (depart="split")
        if s.get("join"):
            res += " join=" + s["join"]    # the train this one joins
        if s.get("jumpUntil") is not None:
            res += " jumpUntil=%r" % (parse_time(s["jumpUntil"]) + until_offset)
        if s.get("_skip"):
            res += " skip=%d" % s["_skip"]
        if s.get("posLat") is not None:
            res += " posLat=%r" % float(s["posLat"])  # the lateral position while stopped
        # MSVehicle::slowDownForSchedule: the given start of the stop (--use-stop-started), flexArrival
        if s.get("started") is not None:
            res += " started=%r" % (parse_time(s["started"]) + until_offset)
        if s.get("_flexArrival") is not None:
            res += " flexArrival=%r" % (parse_time(s["_flexArrival"]) + until_offset)
        return res

    def stop_place(s):
        """bus stop id, or @lane,startPos,endPos (positions as given, psim2 resolves defaults)"""
        if s.get("parking") == "opportunistic":
            raise ValueError("opportunistic parking is not supported")
        if "busStop" in s:
            return s["busStop"]
        if "parkingArea" in s:
            return "%" + s["parkingArea"]
        # an edge stop uses the first lane allowing the vehicle class (resolved by psim2)
        lane = s["lane"] if "lane" in s else "=" + s["edge"]
        return "@%s,%s,%s" % (lane, s.get("startPos", "-"), s.get("endPos", "-"))

    netfile = a.out + ".net.tmp"
    # routing may use any edge (and the router breaks ties by the edge order of the full network)
    full = a.full_net or a.routing or bool(trips) or bool(rerouters) or bool(speed_signs)
    info = psim2net.convert(a.net, netfile, None if full else used_edges, tls_override, a.no_internal_links)
    with open(a.out, "w") as out:
        out.write("STEP %g\n" % a.step)
        for line in preferences:
            out.write(line)
        # NLBuilder::EdgeFloatTimeLineRetriever_EdgeTravelTime: the traveltime attribute of the edges per interval
        # (written after the network)
        weight_lines = []
        for wf in (a.weight_files.split(",") if a.weight_files else []):
            with open_xml(wf) as f:
                begin = end = None
                for ev, el in ET.iterparse(f, events=("start", "end")):
                    if el.tag == "interval" and ev == "start":
                        begin, end = el.get("begin"), el.get("end")
                    elif el.tag == "edge" and ev == "end" and el.get("traveltime") is not None:
                        weight_lines.append("WEIGHT %s %s %s %s\n" % (el.get("id"), begin, end, el.get("traveltime")))
        for t in used_types:
            out.write(type_line(types[t], a.sigma, a.speeddev))
        for d in used_dists:
            out.write("TYPEDIST %s %s\n" % (d, " ".join("%s %r" % (m, p) for m, p in type_dists[d])))
        with open(netfile) as nf:
            for line in nf:
                out.write(line)
        os.remove(netfile)
        for line in weight_lines:
            out.write(line)
        used_zones = {z for zs in veh_zones.values() for z in zs.values()}
        # zones of person trips
        for q in persons:
            for kind, _, at in q[6]:
                for e in (at.get("from"), at.get("to")) if kind == "trip" else ():
                    if e and e.endswith(("-source", "-sink")) and e.rsplit("-", 1)[0] in tazs_def:
                        used_zones.add(e.rsplit("-", 1)[0])
        # zones whose connector edges are via edges
        for vs in vias.values():
            for e in vs:
                for suffix in ("-source", "-sink"):
                    if e.endswith(suffix) and e[:-len(suffix)] in tazs_def:
                        used_zones.add(e[:-len(suffix)])
        for zid, (srcs, sinks) in tazs_def.items():
            if zid in used_zones:
                out.write("TAZ %s %s %s\n" % (zid, ",".join(srcs) or "-", ",".join(sinks) or "-"))
        for d in deadlocks:
            out.write("DEADLOCK %s\n" % " ".join(d))
        for sig, kind, trip, tl, limit, active, foes in constraints:
            out.write("RSC %s %s %s %s %d %d %s\n" % (sig, kind, trip, tl, limit, int(active), " ".join(foes)))
        load_order = {sid: i for i, sid in enumerate(busstops)}
        for sid in sorted(used_stops):
            b = busstops[sid]
            # (then the load order and the name, the rest of the line)
            out.write("BUSSTOP %s %s %s %s %s %s %s %s %d %s\n" % (sid, b["lane"], b.get("startPos", "0"), b.get("endPos", "-"),
                                                                  b.get("parkingLength", "0"),
                                                                  # (default: by the length, see NLTriggerBuilder)
                                                                  b.get("containerCapacity" if b["_tag"] == "containerStop"
                                                                        else "personCapacity", "-"),
                                                                  b.get("angle", "90"),
                                                                  {"containerStop": "C", "chargingStation": "E"}.get(b["_tag"], "B"),
                                                                  load_order[sid], b.get("name", "")))
            for acc in b["_access"]:
                # NLTriggerBuilder::addAccess: position (or random, doors, carriage), length (-1: the distance)
                out.write("BSACCESS %s %s %s %s %d\n" % (sid, acc["lane"], acc.get("pos", "0"), acc.get("length", "-1"),
                                                         sumo_bool(acc.get("friendlyPos", "false"))))
        if have_trips:
            out.write("VDEFAULTS %s %s\n" % (a.default_departlane, a.default_departspeed))
            for flow, _, sid, line, begin_ms, rep, offset_ms, last_ms, sst in sorted(pt_scheds, key=lambda s: (not s[0], s[1])):
                out.write("PTSCHED %s %s %s %d %d %d %d %s\n" % ("F" if flow else "V", sid, line, begin_ms, rep, offset_ms, last_ms,
                                                              " ".join("%s:%d" % (p or "-", u) for p, u in sst) or "-"))
        for pid in sorted(used_pas):
            p, spaces = parkareas[pid]
            badges = ",".join(p.get("acceptedBadges", "").replace(",", " ").split()) or "-"
            out.write("PARKINGAREA %s %s %s %s %d %s %d %s %s %s %s %d %s\n" % (
                pid, p["lane"], p.get("startPos", "0"), p.get("endPos", "-"), p.get("friendlyPos", "false") in ("true", "1"),
                # NLTriggerBuilder::updateParkingAreaDefaultCapacity: one roadside lot without capacity and spaces
                p.get("roadsideCapacity", "0" if spaces else "1"), p.get("onRoad", "false") in ("true", "1"), p.get("width", "0"),
                p.get("length", "0"), p.get("angle", "0"), p.get("departPos", "-") or "-",
                p.get("reservable", "false") in ("true", "1"), badges))
            for sp in spaces:
                out.write("PASPACE %s %s %s %s\n" % (pid, sp["x"], sp["y"], sp.get("angle", "-")))
        for r in used_routes:
            out.write("ROUTE %s %s\n" % (r, " ".join(routes[r])))
        for sid, sign_lanes, steps in speed_signs:
            out.write("VSS %s %s %s\n" % (sid, ",".join(sign_lanes), " ".join(steps)))
        for attrs, intervals in rerouters:
            rid = attrs["id"]
            vtypes = ",".join(attrs.get("vTypes", "").split()) or "-"
            out.write("REROUTER %s %s %d %s %s %s\n" % (rid, attrs.get("probability", "1"),
                                                      attrs.get("off", "false") in ("true", "1"),
                                                      attrs.get("timeThreshold", "0"), vtypes, ",".join(attrs["edges"].split())))
            for begin, end, items in intervals:
                out.write("RRINTERVAL %s %s %s\n" % (rid, begin, end))
                for kind, target, value in items:
                    out.write("RR%s %s %s %s\n" % (kind, rid, target, value))

        def routing_opts(vid, vtype):
            """VOPT keys of the rerouting device assignment (MSDevice::equippedByDefaultAssignmentOptions)"""
            vp = veh_params.get(vid) or veh_params.get(flow_of.get(vid), {})
            tp = types[vtype].get("_params", {}) if vtype in types else {}
            res = []
            key = "has.rerouting.device"
            if key in vp or key in tp:
                res.append("device=%d" % ((vp.get(key) if key in vp else tp.get(key)) in ("true", "1")))
            elif "device.rerouting.probability" in tp:
                res.append("deviceprob=%s" % tp["device.rerouting.probability"])
            # the tripinfo device (has.tripinfo.device of the vehicle or its type)
            key = "has.tripinfo.device"
            if key in vp or key in tp:
                res.append("tripinfo=%d" % ((vp.get(key) if key in vp else tp.get(key)) in ("true", "1")))
            period = vp.get("device.rerouting.period", tp.get("device.rerouting.period"))
            if period is not None:
                res.append("period=%s" % period)
            # MSBaseVehicle::initDevices: getStringParam("device.rerouting.mode") of the vehicle or its type
            mode = vp.get("device.rerouting.mode", tp.get("device.rerouting.mode"))
            if mode is not None:
                res.append("rmode=%d" % int(mode))
            if vp.get("tripId") is not None:
                res.append("tripId=%s" % vp["tripId"])
            # overtakingReroute.prio / .maxDelay of the vehicle or its type (MSBaseVehicle::getFloatParam)
            for key, opt in (("overtakingReroute.prio", "otprio"), ("overtakingReroute.maxDelay", "otmaxdelay"),
                             ("closingReroute.maxDelay", "crmaxdelay")):
                value = vp.get(key, tp.get(key))
                if value is not None:
                    res.append("%s=%s" % (opt, value))
            # MSVehicle::ignoreFoe, MSLink::ignoreFoe: lists of the vehicle's parameters
            for key, opt in (("carFollowModel.ignoreIDs", "cfignoreids"), ("carFollowModel.ignoreTypes", "cfignoretypes"),
                             ("junctionModel.ignoreIDs", "jmignoreids"), ("junctionModel.ignoreTypes", "jmignoretypes")):
                if vp.get(key):
                    res.append("%s=%s" % (opt, ",".join(vp[key].replace(",", " ").split())))
            # departEdge: the route index of the departure (SUMOVehicleParameter::departEdge)
            ichecks = veh_ichecks.get(vid, veh_ichecks.get(flow_of.get(vid)))
            if ichecks is not None:
                res.append("ichecks=%s" % ichecks)
            res.extend(veh_poslat.get(vid, veh_poslat.get(flow_of.get(vid), [])))
            dedge = veh_depart_edge.get(vid, veh_depart_edge.get(flow_of.get(vid)))
            if dedge is not None:
                res.append("departEdge=%s" % dedge)
            line = veh_lines.get(vid, veh_lines.get(flow_of.get(vid)))
            if line is not None:
                res.append("line=%s" % line)
            for key, zone in sorted(veh_zones.get(vid, veh_zones.get(flow_of.get(vid), {})).items()):
                res.append("%s=%s" % (key, zone))
            # MSBaseVehicle::getParkingBadges: the vehicle's badges, otherwise those of its type
            badges = veh_badges.get(vid, veh_badges.get(flow_of.get(vid)))
            if badges is None:
                badges = types[vtype].get("parkingBadges") if vtype in types else None
            if badges:
                res.append("badges=" + ",".join(badges.replace(",", " ").split()))
            # MSStoppingPlaceRerouter::getWeight: parking search parameters of the vehicle, otherwise of its type
            pp = {k: v for k, v in tp.items() if k.startswith("parking.")}
            pp.update({k: v for k, v in vp.items() if k.startswith("parking.")})
            for k in sorted(pp):
                res.append("%s=%s" % (k, pp[k]))
            return res

        for f in flows:
            begin, _, fid, vtype, rid, dl, ds, dp, arr, alane, aspeed, stops, kind, value, end, number = f
            out.write("FLOW %s %s %s %r %r %s %r %d %s %s %s %s %s %s %s\n" % (fid, vtype, rid, begin, end, kind, value, number,
                                                                             dl, ds, dp, arr, alane, aspeed,
                                                                             veh_speed_factor.get(fid, "-")))
            vopts = routing_opts(fid, vtype)
            if fid in trips:
                vopts.append("reroute=1")
            if fid in init_replaced:
                vopts.append("initreplaced=1")
            if fid in vias:
                vopts.append("via=" + ",".join(vias[fid]))
            if vopts:
                out.write("VOPT %s %s\n" % (fid, " ".join(vopts)))
            for s in stops:
                until = s.get("until")
                # route stops count from each vehicle's departure (D), vehicle stops are absolute
                until = ("D%r" if s.get("_route") else "%r") % parse_time(until) if until is not None else "-"
                duration = "%r" % parse_time(s.get("duration", "0"))
                if a.use_stop_ended and s.get("ended") is not None:
                    until, duration = "%r" % float(s["ended"]), "-1"  # MSStop::getMinDuration: ended - time
                parking = stop_parking(s)
                out.write("STOP %s %s %s %s %s %s %s%s\n" % (fid, stop_place(s), duration, until, parking, s.get("tripId", "-"),
                                                            s.get("speed", "0"), stop_extras(s, 0)))
        for v in vehicles:
            # the desired depart time: psim2 inserts at the first step >= depart and reports the delay from it
            depart, _, vid, vtype, rid, dl, ds, dp, arr, stops, until_offset, alane, aspeed = v
            src = vid if vid not in flow_of else flow_of[vid]
            out.write("VEH %s %s %s %s %s %s %s %s %s %s %s\n" % (vid, vtype, rid,
                                                              ("split" if vid in split_vehicles else "triggered")
                                                              if vid in triggered_vehicles else repr(depart),
                                                              dl, ds, dp, arr, veh_speed_factor.get(src, "-"), alane, aspeed))
            vopts = routing_opts(vid, vtype)
            if src in trips:
                vopts.append("reroute=1")
            if src in init_replaced:
                vopts.append("initreplaced=1")
            if src in vias:
                vopts.append("via=" + ",".join(vias[src]))
            if vid in flow_of:
                vopts.append("flow=1")
            if vid in full_route_of:
                vopts.append("fullroute=" + full_route_of[vid])
            if vopts:
                out.write("VOPT %s %s\n" % (vid, " ".join(vopts)))
            for s in stops:
                until = s.get("until")
                offset = depart if s.get("_route") else until_offset
                until = "%r" % (parse_time(until) + offset) if until is not None else "-"
                duration = "%r" % parse_time(s.get("duration", "0"))
                if a.use_stop_ended and s.get("ended") is not None:
                    until, duration = "%r" % float(s["ended"]), "-1"  # MSStop::getMinDuration: ended - time
                parking = stop_parking(s)
                out.write("STOP %s %s %s %s %s %s %s%s\n" % (vid, stop_place(s), duration, until, parking, s.get("tripId", "-"),
                                                            s.get("speed", "0"), stop_extras(s, offset)))
        for q in persons:
            depart, _, pid, ptype, dpos, sf, stages, flow = q
            kind = "CONTAINER" if pid in container_ids else "PERSON"
            if flow is None:
                out.write("%s %s %s %s %s %s\n" % (kind, pid, ptype, "triggered" if pid in triggered_persons else repr(depart),
                                                    dpos, sf))
            else:
                out.write("%s %s %s %r %r %s %r %d %s %s\n" % ("CFLOW" if kind == "CONTAINER" else "PFLOW", pid, ptype, depart,
                                                              flow[2], flow[0], flow[1], flow[3], dpos, sf))
            last_edge = None
            for kind, edges_, at in stages:
                if kind == "tranship":
                    # MSRouteHandler::addTranship: the edges, or from (the previous stage's end) and to (or the stop's edge)
                    stop = at.get("containerStop", at.get("busStop", at.get("trainStop", at.get("chargingStation", "-"))))
                    if edges_:
                        es = edges_
                    else:
                        to = at.get("to") or (busstops[stop]["lane"].rsplit("_", 1)[0] if stop != "-" else None)
                        es = [at.get("from") or last_edge, to]
                        if None in es:
                            raise ValueError("tranship of container %s without start or destination" % pid)
                    speed = at.get("speed") or types.get(ptype, {}).get("maxSpeed") or repr(5. / 3.6)
                    out.write("PTRANSHIP %s %s %s %s %s %s\n" % (pid, ",".join(es), speed, at.get("departPos", "-"),
                                                                at.get("arrivalPos", "-"), stop))
                    last_edge = es[-1]
                    continue
                if kind == "ride":
                    stop = at.get("busStop", at.get("trainStop", at.get("containerStop", "-")))
                    if at.get("parkingArea"):
                        stop = "%" + at["parkingArea"]
                    out.write("PRIDE %s %s %s %s %s %s %s\n" % (
                        pid, at.get("from", "-"), at.get("to", "-"), stop, at.get("arrivalPos", "-"),
                        ",".join(at.get("lines", "ANY").split()), at.get("intended", "-")))
                    last_edge = at.get("to") or (busstops[stop]["lane"].rsplit("_", 1)[0] if stop not in ("-", "") and stop[0] != "%"
                                                 else (parkareas[stop[1:]][0]["lane"].rsplit("_", 1)[0] if stop != "-" else None))
                    continue
                if kind == "trip":
                    # the destination: a stopping place of any kind (parking areas marked by '%')
                    stop = at.get("busStop", at.get("trainStop", at.get("containerStop", at.get("chargingStation", "-"))))
                    if at.get("parkingArea"):
                        stop = "%" + at["parkingArea"]
                    out.write("PTRIP %s %s %s %s %s %s %s %s %s %s %s\n" % (
                        pid, at.get("from", "-"), at.get("to", "-"), stop,
                        at.get("arrivalPos", "-"), at.get("speed", "-"), at.get("duration", "-"), at.get("departPosLat", "-"),
                        at.get("walkFactor", "-"), ",".join(at.get("_modes", "").split()) or "-",
                        ",".join(at.get("_vtypes", "").split()) or "-"))
                    last_edge = at.get("to") or (busstops[stop]["lane"].rsplit("_", 1)[0] if stop in busstops
                                                 else (parkareas[stop[1:]][0]["lane"].rsplit("_", 1)[0] if stop != "-" else None))
                elif kind == "walk":
                    out.write("PWALK %s %s %s %s %s %s %s %s\n" % (
                        pid, ",".join(edges_), at.get("speed", "-"), at.get("duration", "-"), at.get("arrivalPos", "-"),
                        at.get("departPosLat", "-"), at.get("departLane", "-"), at.get("busStop", at.get("trainStop", "-"))))
                    last_edge = edges_[-1]
                else:
                    if at.get("busStop") or at.get("trainStop"):
                        place = "@" + (at.get("busStop") or at.get("trainStop"))
                    elif at.get("edge"):
                        place = at["edge"]
                    elif at.get("lane"):
                        place = at["lane"].rsplit("_", 1)[0]
                    elif last_edge is not None:
                        place = last_edge
                    else:
                        raise ValueError("person stop without position (person %s)" % pid)
                    out.write("PSTOP %s %s %s %s %s\n" % (pid, place, at.get("duration", "-"), at.get("until", "-"),
                                                          at.get("actType", "-")))
    info.update(vehicles=len(vehicles), flows=len(flows), routes=len(used_routes), busStops=len(used_stops),
                persons=len(persons))
    print(info)


if __name__ == "__main__":
    main()
