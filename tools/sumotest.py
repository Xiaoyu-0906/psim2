"""Run SUMO's official texttest cases (sumo-src/tests/sumo/...) with SUMO and psim2.

Usage: sumotest.py TESTDIR [TESTDIR ...] [--keep-sigma] [--parallel] [--derandomize] [--stats N]

TESTDIR is relative to sumo-src/tests/sumo (e.g. junction_model/junction_type/zipper/3links)
or a directory containing tests (all leaf tests below it are run). Like texttest, the
input files of config.sumo's copy_test_path list are taken from the nearest ancestor
directory, and the options of tests/sumo/options.sumo come before the test's own.

Default mode (debugging): deterministic step-by-step comparison. sigma and speedDev of
all vehicle types are set to 0 (unless --keep-sigma), psim2 runs with --sumo-order
(unless --parallel), --derandomize replaces random demand (Poisson flows, random
departure positions, lanes and speeds, speed factor distributions) by fixed values.

--stats N (final validation): all randomness on, N SUMO runs and N psim2 runs with
different seeds, trip statistics compared (z = difference / standard error).

Each result line names the SUMO options that psim2 does not handle.

Disk usage: SUMO and psim2 (--gzip) write their fcd gzip-compressed, the work directories of exact tests are
removed (PSIM2_KEEP=1 keeps them), the other large outputs of differing tests compressed (tools/fcddiff.py reads them).
"""
import argparse
import csv
import gzip
import math
import os
import re
import shlex
import shutil
import statistics
import subprocess
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
TESTS = os.path.join(ROOT, "sumo-src", "tests", "sumo")
BIN = os.environ.get("PSIM2_SUMO_BIN", "")
WORK = os.path.join(ROOT, "work", os.environ.get("PSIM2_WORK", "sumotests"))
PSIM2_BIN = os.environ.get("PSIM2_BIN", os.path.join(ROOT, "psim2", "psim2"))
COPY_NAMES = ["net.net.xml", "net.net.xml.gz", "net2.net.xml", "net3.net.xml", "net4.net.xml", "net5.net.xml",
              "three_split.net.xml", "motorway.net.xml", "lefthand.net.xml", "input_routes.rou.xml", "input_routes.rou.xml.gz",
              "input_routes2.rou.xml", "input_routes3.rou.xml", "input_additional.add.xml", "input_additional2.add.xml",
              "input_additional3.add.xml", "input_additional4.add.xml", "input_trips.trips.xml", "input_districts.taz.xml",
              "busnet.net.xml", "sumo.sumocfg", "input_state.xml"]
OUTPUT_OPTIONS = {"--tripinfo-output", "--vehroutes", "--fcd-output", "--summary-output", "--vehroute-output", "--netstate-dump",
                  "--lanechange-output", "--stop-output", "--statistic-output", "--emission-output", "--queue-output",
                  "--edgedata-output", "--lanedata-output", "--link-output", "--person-summary-output", "--collision-output",
                  "--railsignal-block-output", "--railsignal-vehicle-output", "--deadlock-output", "--amitran-output", "--full-output", "--battery-output",
                  "--elechybrid-output", "--chargingstations-output", "--vtk-output", "--personinfo-output",
                  "--log", "-l", "--message-log", "--error-log", "--save-state.times", "--save-state.files",
                  "--save-configuration", "-C", "--write-license", "--output-prefix", "--precision", "--human-readable-time",
                  "--fcd-output.attributes", "--fcd-output.geo", "--fcd-output.signals", "--fcd-output.distance",
                  "--device.fcd.explicit", "--device.fcd.begin", "--device.fcd.end", "--device.fcd.period",
                  "--device.fcd.probability", "--person-device.fcd.probability", "--person-device.fcd.explicit",
                  "--device.tripinfo.probability", "--tripinfo-output.write-unfinished",
                  "--vehroute-output.cost", "--vehroute-output.route-length", "--vehroute-output.write-unfinished",
                  "--vehroute-output.exit-times", "--vehroute-output.last-route", "--vehroute-output.sorted",
                  "--vehroute-output.dua", "--vehroute-output.intended-depart", "--vehroute-output.incomplete",
                  "--vehroute-output.stop-edges", "--vehroute-output.skip-ptlines", "--vehroute-output.speedfactor", "--vehroute-output.internal",
                  "--stop-output.write-unfinished",
                  "-g", "--gui-settings-file", "--delay", "-d", "--start", "-S", "--quit-on-end", "-Q"}  # (sumo-gui only)
FLAGS = {"--no-step-log", "--no-duration-log", "-t", "--duration-log.statistics", "--no-warnings", "-W", "--verbose", "-v",
         "--duration-log.disable", "--tripinfo-output.write-unfinished", "--write-license"}
# boolean SUMO options psim2 understands
PSIM_FLAGS = {"--device.rerouting.railsignal": "--device.rerouting.railsignal", "--collision.check-junctions": "--collision.check-junctions", "--emergency-insert": "--emergency-insert",
              "--lanechange.overtake-right": "--lanechange.overtake-right",
              "--tls.all-off": "--tls.all-off", "--use-stop-started": "--use-stop-started",
              "--time-to-teleport.remove": "--time-to-teleport.remove",
              "--step-method.ballistic": "--step-method.ballistic", "--device.routing.railsignal": "--device.rerouting.railsignal",
              "--extrapolate-departpos": "--extrapolate-departpos", "--ignore-route-errors": "--ignore-route-errors",
              "--railsignal-moving-block": "--railsignal-moving-block", "--railsignal.moving-block": "--railsignal-moving-block",
              "--time-to-teleport.remove-constraint": "--time-to-teleport.remove-constraint",
              "--eager-insert": "--eager-insert", "--weights.random-factor.dynamic": "--weights.random-factor.dynamic",
              "--device.rerouting.init-with-loaded-weights": "--device.rerouting.init-with-loaded-weights",
              "--device.rerouting.bike-speeds": "--device.rerouting.bike-speeds"}
# SUMO option -> psim2 option (with value)
PSIM_OPTIONS = {"--collision.check-junctions.mingap": "--collision.check-junctions.mingap", "--time-to-teleport": "--time-to-teleport", "--ignore-junction-blocker": "--ignore-junction-blocker",
                "--time-to-teleport.railsignal-deadlock": "--time-to-teleport.railsignal-deadlock",
                "--collision.action": "--collision.action", "--insertion-checks": "--insertion-checks",
                "--collision.mingap-factor": "--collision.mingap-factor", "--startup-wait-threshold": "--startup-wait-threshold",
                "--default.action-step-length": "--default.action-step-length",
                "--max-depart-delay": "--max-depart-delay", "--max-num-vehicles": "--max-num-vehicles",
                "--lanechange.duration": "--lanechange.duration",
                "--lateral-resolution": "--lateral-resolution",
                "--railsignal.max-block-length": "--railsignal.max-block-length",
                "--railsignal.moving-block.max-dist": "--railsignal.moving-block.max-dist",
                "--railsignal.default-classes": "--railsignal.default-classes",
                "--railsignal.moving-block.default-classes": "--railsignal.moving-block.default-classes",
                "--end": "--end", "-e": "--end", "--begin": "--begin", "-b": "--begin", "--seed": "--seed",
                "--weights.random-factor": "--weights.random-factor", "--weights.priority-factor": "--weights.priority-factor",
                "--weights.separate-turns": "--weights.separate-turns",
                "--persontrip.transfer.car-walk": "--persontrip.transfer.car-walk",
                "--persontrip.transfer.taxi-walk": "--persontrip.transfer.taxi-walk",
                "--persontrip.transfer.walk-taxi": "--persontrip.transfer.walk-taxi",
                "--persontrip.taxi.waiting-time": "--persontrip.taxi.waiting-time",
                "--persontrip.ride-public-line": "--persontrip.ride-public-line",
                "--persontrip.walkfactor": "--persontrip.walkfactor",
                "--persontrip.walk-opposite-factor": "--persontrip.walk-opposite-factor"}
# SUMO option -> psim2scen option (with value)
# options only SUMO needs (texttest runs SUMO without XML schema validation: the schemas of the binary may lag)
SUMO_ONLY = {"--seed", "--xml-validation", "--xml-validation.net", "--xml-validation.routes"}
CONV_FLAGS = {"--use-stop-ended": "--use-stop-ended", "--mapmatch.junctions": "--mapmatch-junctions", "--mapmatch.taz": "--mapmatch-taz"}
CONV_OPTIONS = {"--carfollow.model": "--carfollow-model", "--default.carfollowmodel": "--carfollow-model", "--scale": "--scale", "--step-length": "--step", "--mapmatch.distance": "--mapmatch-distance",
                "--weight-files": "--weight-files",
                "--default.departlane": "--default-departlane",
                "--default.departspeed": "--default-departspeed", "--default.speeddev": "--default-speeddev"}
FILE_OPTIONS = {"-n": "net", "--net-file": "net", "--net": "net", "-r": "routes", "--route-files": "routes", "--routes": "routes",
                "-a": "add", "--additional-files": "add", "--additional": "add"}
SKIP = {
        "--mesosim": lambda v: "mesoscopic simulation not supported",
        # (astar, CH and CHWrapper find the same shortest routes as dijkstra, psim2 uses dijkstra)
        "--routing-algorithm": lambda v: None if v in ("dijkstra", "astar", "CH", "CHWrapper") else "routing algorithm %s not supported" % v,
        "--device.rerouting.mode": lambda v: None if v in ("0", "1", "4", "8", "9", "12") else "routing mode %s not supported" % v,
        "--parking.maneuver": lambda v: None if v in ("false", "0") else "parking manoeuvres not supported"}
# vehicle routing options psim2 understands (--device.routing.* is a synonym)
ROUTING_OPTIONS = {"probability", "explicit", "period", "pre-period", "adaptation-interval", "adaptation-steps", "adaptation-weight",
                   "threshold.factor", "threshold.constant", "synchronize", "mode"}
ROUTING_IGNORED = {"threads", "output"}  # no effect on the results


def nearest(test, name):
    d = test
    while True:
        p = os.path.join(d, name)
        if os.path.exists(p):
            return p
        if os.path.normpath(d) == os.path.normpath(TESTS):
            return None
        d = os.path.dirname(d)


def leaf_tests(path):
    """test directories below path (texttest: directories with an options.sumo or output files)"""
    out = []
    for d, subdirs, files in os.walk(path):
        if "options.sumo" in files or "output.sumo" in files:
            if not any(os.path.exists(os.path.join(d, s, "options.sumo")) or os.path.exists(os.path.join(d, s, "output.sumo"))
                       for s in subdirs):
                out.append(d)
    return sorted(out)


def sumocfg_args(path):
    """options of a .sumocfg as command line arguments"""
    out = []
    for group in ET.parse(path).getroot():
        for opt in group:
            out += ["--" + opt.tag, opt.get("value", "")]
    return out


def zero_sigma(path, defined, eidm_default=False):
    """set sigma and speedDev to 0 in all vTypes of a route/additional file, define the default types
    (once, in the first routes file; defined collects the default types already written or found)"""
    s = open(path).read()
    # the root element (not a match inside a comment such as "<additional-files ...>" of a header)
    comments = [(m.start(), m.end()) for m in re.finditer(r"<!--.*?-->", s, re.S)]
    root = None
    for m in re.finditer(r"<(routes|additional|add)(\s[^>]*)?>", s):
        if not any(a <= m.start() < b for a, b in comments):
            root = m
            break
    if root is None:
        return

    def fix(m):
        tag = m.group(0)
        # (the EIDM's estimation and driving errors are its imperfection)
        eidm = "EIDM" in tag or (eidm_default and "carFollowModel" not in tag)
        for attr in ("sigma", "speedDev") + (("sigmaleader", "sigmagap", "sigmaerror") if eidm else ()):
            if re.search(r'\s%s="[^"]*"' % attr, tag):
                tag = re.sub(r'(\s%s=")[^"]*(")' % attr, r'\g<1>0\g<2>', tag)
            else:
                tag = tag.replace("<vType", '<vType %s="0"' % attr, 1)
        # (the lateral imperfection of the sublane model, only where it is given)
        tag = re.sub(r'(\slcSigma=")[^"]*(")', r'\g<1>0\g<2>', tag)
        return tag
    s = re.sub(r"<vType\b[^>]*>", fix, s)
    defaults = ""
    for vid, vclass in (("DEFAULT_VEHTYPE", None), ("DEFAULT_BIKETYPE", "bicycle"), ("DEFAULT_TAXITYPE", "taxi"),
                        ("DEFAULT_PEDTYPE", "pedestrian")):
        if vid not in defined and root.group(1) == "routes":
            defaults += '\n    <vType id="%s"%s sigma="0" speedDev="0"%s/>' % (vid, ' vClass="%s"' % vclass if vclass else "",
                                                                       ' sigmaleader="0" sigmagap="0" sigmaerror="0"' if eidm_default else "")
            defined.add(vid)
    s = s[:root.end()] + defaults + s[root.end():]
    open(path, "w").write(s)


def derandomize(path):
    """replace random demand attributes by deterministic ones (applied for both simulators)"""
    s = open(path).read()
    # only flows and person flows (rerouters and distributions also have probabilities)
    s = re.sub(r'(<(?:flow|personFlow)\b[^>]*?)period="exp\(([^)]*)\)"',
               lambda m: m.group(1) + 'period="%g"' % (1 / float(m.group(2))), s)
    s = re.sub(r'(<(?:flow|personFlow)\b[^>]*?)probability="([^"]*)"',
               lambda m: m.group(1) + ('period="%g"' % (1 / float(m.group(2))) if float(m.group(2)) > 0 else 'number="0"'), s)
    s = re.sub(r'departPos="(random|random_free)"', 'departPos="base"', s)
    s = s.replace('departLane="random"', 'departLane="best"').replace('departSpeed="random"', 'departSpeed="max"')
    s = re.sub(r'speedFactor="normc?\(([^,]*),[^"]*"', lambda m: 'speedFactor="%s"' % m.group(1), s)
    s = re.sub(r'(<person[^>]*\s)departPos="random"', r'\1departPos="0"', s)
    s = re.sub(r'(<walk[^>]*\s)arrivalPos="random"', r'\1arrivalPos="-1"', s)
    s = re.sub(r'departPosLat="random"', 'departPosLat="0"', s)
    s = s.replace('arrivalLane="random"', 'arrivalLane="current"')

    # vehicle type distributions: always the first type
    def first_type(m):
        n = [0]

        def prob(tm):
            n[0] += 1
            tag = re.sub(r'\sprobability="[^"]*"', '', tm.group(0))
            return tag.replace('<vType', '<vType probability="%d"' % (1 if n[0] == 1 else 0), 1)
        return re.sub(r'<vType\b[^>]*>', prob, m.group(0))
    s = re.sub(r'<vTypeDistribution\b[^>]*?>.*?</vTypeDistribution>', first_type, s, flags=re.S)
    s = re.sub(r'(<vTypeDistribution\b[^>]*?\s)probabilities="([^"]*)"',
               lambda m: m.group(1) + 'probabilities="%s"' % " ".join(["1"] + ["0"] * (len(m.group(2).split()) - 1)), s)
    s = re.sub(r'(<(?:vehicle|flow|trip)\b[^>]*\s)arrivalPos="random"', r'\1arrivalPos="max"', s)

    # rerouter intervals: the first route / destination alternative with a positive probability only
    def first_alternative(m):
        for tag in ("routeProbReroute", "destProbReroute"):
            seen = [False]

            def prob(tm):
                pm = re.search(r'\sprobability="([^"]*)"', tm.group(0))
                positive = pm is None or float(pm.group(1)) > 0
                keep = positive and not seen[0]
                seen[0] |= keep
                t = re.sub(r'\sprobability="[^"]*"', '', tm.group(0))
                return t.replace('<' + tag, '<%s probability="%d"' % (tag, 1 if keep else 0), 1)
            m_s = re.sub(r'<%s\b[^>]*>' % tag, prob, m if isinstance(m, str) else m.group(0))
            m = m_s
        return m
    s = re.sub(r'<interval\b[^>]*?>.*?</interval>', first_alternative, s, flags=re.S)
    open(path, "w").write(s)


class Prepared:
    pass


EXTRA_OPTIONS = []


def prepare(test, work, keep_sigma=True, derand=False):
    """copy the inputs into work and translate the options; returns Prepared or an error string"""
    shutil.rmtree(work, ignore_errors=True)
    os.makedirs(work)
    if nearest(test, "options.sumo") is None:
        return "no options.sumo"
    # texttest combines the options files of all directories from the root down to the test
    opt_files = []
    d = test
    while True:
        if os.path.exists(os.path.join(d, "options.sumo")):
            opt_files.insert(0, os.path.join(d, "options.sumo"))
        if os.path.normpath(d) == os.path.normpath(TESTS):
            break
        d = os.path.dirname(d)
    args = []
    for f in opt_files:
        text = open(f).read()
        if "{CLEAR}" in text:  # texttest: forget the inherited options
            args = []
            text = text.replace("{CLEAR}", " ")
        for m in re.finditer(r"\{CLEAR ([^}]*)\}", text):  # texttest: forget the given inherited options
            drop = shlex.split(m.group(1))
            for i in range(len(args) - len(drop), -1, -1):
                if args[i:i + len(drop)] == drop:
                    del args[i:i + len(drop)]
        text = re.sub(r"\{CLEAR [^}]*\}", " ", text)
        args += shlex.split(text.replace("\n", " "))
    args += EXTRA_OPTIONS  # (--add-options: e.g. the ballistic variant of the test suites)
    split = []
    for a in args:  # --option=value
        if a.startswith("-") and "=" in a:
            split += a.split("=", 1)
        else:
            split.append(a)
    # an option given again further down replaces the earlier value (also flags and short options)
    items = []  # (option, [option and value tokens])
    i = 0
    while i < len(split):
        a = split[i]
        if a.startswith("-") and not re.match(r"^-[0-9.]", a):
            if i + 1 < len(split) and (not split[i + 1].startswith("-") or re.match(r"^-[0-9.]", split[i + 1])):
                items.append((a, [a, split[i + 1]]))
                i += 2
                continue
            items.append((a, [a]))
        else:
            items.append((None, [a]))
        i += 1
    # file lists given again are merged (SUMO's expected outputs list each file once, e.g. additional-files
    # "a.add.xml,b.rou.xml,c.add.xml" for -a in the parent and in the test options)
    synonyms = {"-a": "-a", "--additional-files": "-a", "--additional": "-a", "-r": "-r", "--route-files": "-r", "--routes": "-r"}
    merged = {}
    for name, toks in items:
        if name in synonyms and len(toks) == 2:
            merged.setdefault(synonyms[name], []).append(toks[1])
    args = []
    done = set()
    for k, (name, toks) in enumerate(items):
        if name in synonyms and len(toks) == 2:
            key = synonyms[name]
            if key not in done:
                done.add(key)
                files = []
                for value in merged[key]:
                    files += [f for f in value.split(",") if f and f not in files]
                args += [toks[0], ",".join(files)]
            continue
        if name is not None and any(n == name for n, _ in items[k + 1:]):
            continue
        args += toks
    for name in COPY_NAMES:
        src = nearest(test, name)
        if src:
            shutil.copy(src, os.path.join(work, name))
    # other input files named by an option value (e.g. --astar.landmark-distances lookuptable, weight files)
    for t in args:
        for name in t.split(","):
            if name and not name.startswith("-") and "/" not in name and not os.path.exists(os.path.join(work, name)):
                src = nearest(test, name)
                if src and os.path.isfile(src):
                    shutil.copy(src, os.path.join(work, name))
    # a configuration file is expanded into options (later options override it)
    expanded = []
    i = 0
    while i < len(args):
        if args[i] in ("-c", "--configuration-file"):
            expanded += sumocfg_args(os.path.join(work, args[i + 1]))
            i += 2
        else:
            expanded.append(args[i])
            i += 1
    p = Prepared()
    p.files, p.sumo, p.psim, p.conv, p.unhandled = {}, [], [], [], []
    p.blocks = False
    p.deadlocks = False
    p.rsvehicles = False
    i = 0
    while i < len(expanded):
        a = expanded[i]
        nxt = expanded[i + 1] if i + 1 < len(expanded) else None
        # a value may be a negative number
        v = nxt if nxt is not None and (not nxt.startswith("-") or re.match(r"^-[0-9.]", nxt)) else None
        if a in SKIP and SKIP[a](v if v is not None else "true"):
            return "SKIP " + SKIP[a](v if v is not None else "true")
        if a in OUTPUT_OPTIONS:
            if a == "--railsignal-block-output":
                p.blocks = True  # (compared as well)
            if a == "--deadlock-output":
                p.deadlocks = True
            if a == "--railsignal-vehicle-output":
                p.rsvehicles = True
            i += 2 if v is not None else 1
            continue
        if a in ("-H", "--human-readable-time"):
            # (output formatting only; the fcd comparison reads times in seconds)
            i += 2 if v in ("true", "false", "1", "0") else 1
            continue
        if a in FLAGS:
            p.sumo.append(a)
            i += 1
            continue
        if a in PSIM_FLAGS and (v is None or v in ("true", "1")):
            p.sumo.append(a)
            p.psim.append(PSIM_FLAGS[a])
            i += 1 if v is None else 2
            continue
        if a in ("--with-taz", "--device.rerouting.with-taz", "--device.routing.with-taz", "--junction-taz"):
            # zones: the converter builds the trips, psim2 keeps the zone for later reroutes
            if v is None or v in ("true", "1"):
                p.conv.append("--junction-taz" if a == "--junction-taz" else "--with-taz")
                if a != "--junction-taz":
                    p.psim.append("--with-taz")
            p.sumo += [a] if v is None else [a, v]
            i += 1 if v is None else 2
            continue
        if a == "--no-internal-links":
            # the converter leaves out the internal lanes, psim2 switches off what depends on them
            if v is None or v in ("true", "1"):
                p.conv.append(a)
                p.psim.append(a)
            p.sumo += [a] if v is None else [a, v]
            i += 1 if v is None else 2
            continue
        if a in CONV_FLAGS and (v is None or v in ("true", "1")):
            p.sumo.append(a)
            p.conv.append(CONV_FLAGS[a])
            i += 1 if v is None else 2
            continue
        if v is None:  # boolean option given without value
            p.sumo.append(a)
            if a.startswith("--pedestrian."):
                p.psim += [a, "true"]
            else:
                p.unhandled.append(a)
            i += 1
            continue
        p.sumo += [a, v]
        routing = None
        for prefix in ("--device.rerouting.", "--device.routing."):
            if a.startswith(prefix):
                routing = a[len(prefix):]
        if routing in ROUTING_OPTIONS:
            p.psim += ["--device.rerouting." + routing, v]
        elif routing in ROUTING_IGNORED or (routing == "railsignal" and v in ("false", "0")):
            pass
        elif a in ("--weights.minor-penalty", "--weights.turnaround-penalty", "--weights.reversal-penalty",
                   "--railway.max-train-length"):
            p.psim += [a, v]
        elif a.startswith("--pedestrian."):
            if derand and a == "--pedestrian.striping.dawdling":
                v = "0"
                p.sumo[-1] = v
            p.psim += [a, v]
        elif a in FILE_OPTIONS:
            p.files[FILE_OPTIONS[a]] = v
        elif a in PSIM_OPTIONS:
            p.psim += [PSIM_OPTIONS[a], v]
            if a == "--seed":
                p.psim += ["--sumo-seed", v]  # SUMO's hash randomness (vehicle random seeds)
            if PSIM_OPTIONS[a] == "--begin":
                p.conv += ["--begin", v]  # vehicles departing before the begin are dropped
        elif a in CONV_OPTIONS:
            p.conv += [CONV_OPTIONS[a], v]
        elif a not in SUMO_ONLY:
            p.unhandled.append("%s=%s" % (a, v))
        i += 2
    if derand and "--pedestrian.striping.dawdling" not in p.sumo:
        # no random slow-downs of pedestrians in the deterministic comparison
        p.sumo += ["--pedestrian.striping.dawdling", "0"]
        p.psim += ["--pedestrian.striping.dawdling", "0"]
    if "net" not in p.files or "routes" not in p.files:
        if "net" in p.files and ("add" in p.files or p.blocks):
            p.files["routes"] = ""  # persons are often given as additional files (or only the rail signals are written)
        else:
            return "no net or routes"
    demand = [os.path.join(work, f) for f in p.files["routes"].split(",") + p.files.get("add", "").split(",")
              if f and os.path.exists(os.path.join(work, f))]
    defined = set()
    for f in demand:
        text = open(f).read()
        defined.update(vid for vid in ("DEFAULT_VEHTYPE", "DEFAULT_BIKETYPE", "DEFAULT_TAXITYPE", "DEFAULT_PEDTYPE")
                       if 'id="%s"' % vid in text)
    # SUMO loads additional files before route files: the default types go into the first file loaded
    for f in p.files.get("add", "").split(",") + p.files["routes"].split(","):
        if f and os.path.exists(os.path.join(work, f)):
            # every vehicle and person is compared in the fcd output
            text = open(os.path.join(work, f)).read()
            text2 = re.sub(r'<param\s+key="has\.fcd\.device"\s+value="false"\s*/>', "", text)
            if text2 != text:
                open(os.path.join(work, f), "w").write(text2)
            if not keep_sigma:
                zero_sigma(os.path.join(work, f), defined, "--carfollow-model" in p.conv
                           and p.conv[p.conv.index("--carfollow-model") + 1] == "EIDM")
            if derand:
                derandomize(os.path.join(work, f))
    conv = [sys.executable, os.path.join(HERE, "psim2scen.py"), p.files["net"], p.files["routes"], "scenario.psim2",
            "--full-net"] + p.conv
    if "add" in p.files:
        conv += ["--additional", p.files["add"]]
    r = subprocess.run(conv, cwd=work, capture_output=True, text=True)
    if r.returncode != 0:
        return "conversion: " + (r.stderr or r.stdout).strip().splitlines()[-1][:150]
    return p


def first_error(r):
    """the first error message of a run (normalized: SUMO's "Error: " and psim2's "error: " prefixes removed)"""
    for line in (r.stderr + r.stdout).splitlines():
        if line.lower().startswith("error:"):
            return line[6:].strip()
    return None


def last_line(r):
    lines = (r.stderr or r.stdout).strip().splitlines()
    return lines[-1][:150] if lines else "exit %d" % r.returncode


# load tests whose outputs are far too large for a step-by-step comparison (extended/stops/stress: 70 GB of fcd)
SKIP_TESTS = {"extended/stops/stress": "load test (output too large)"}


def run_test(test, keep_sigma, parallel, derand):
    rel = os.path.relpath(test, TESTS)
    if rel in SKIP_TESTS:
        return rel, "SKIP " + SKIP_TESTS[rel]
    work = os.path.join(WORK, rel)
    p = prepare(test, work, keep_sigma, derand)
    if isinstance(p, str):
        return rel, p
    note = (" [unhandled: %s]" % " ".join(p.unhandled)) if p.unhandled else ""
    os.makedirs(os.path.join(work, "S"))
    os.makedirs(os.path.join(work, "P"))
    fcd_opts = ["--fcd-output", "S/fcd.xml.gz", "--no-warnings", "true"]
    if "--lateral-resolution" in p.psim or "--lanechange.duration" in p.psim:
        fcd_opts += ["--fcd-output.attributes", "location,posLat"]  # (the lateral positions are compared as well)
    if p.blocks:
        fcd_opts += ["--railsignal-block-output", "S/railsignal_blocks.xml"]
    if p.deadlocks:
        fcd_opts += ["--deadlock-output", "S/deadlocks.xml"]
    if p.rsvehicles:
        fcd_opts += ["--railsignal-vehicle-output", "S/railsignal_vehicles.xml"]
    try:
        r = subprocess.run([os.path.join(BIN, "sumo")] + p.sumo + fcd_opts,
                           cwd=work, capture_output=True, text=True, timeout=900)
    except subprocess.TimeoutExpired:
        return rel, "SUMO timeout (900 s)" + note
    if r.returncode < 0:
        return rel, f"SUMO terminated by signal {-r.returncode}"
    sumo_failed = r.returncode != 0
    sumo_error = first_error(r) if sumo_failed else None
    ps = [PSIM2_BIN, "scenario.psim2", "--fcd", "--gzip", "--out", "P/"] + p.psim
    if not parallel:
        ps.append("--sumo-order")
    if p.blocks:
        ps.append("--railsignal-block-output")
    if p.deadlocks:
        ps.append("--deadlock-output")
    if p.rsvehicles:
        ps.append("--railsignal-vehicle-output")
    with open(os.path.join(work, "cmd.txt"), "w") as f:
        # (for debugging: the commands of both runs)
        f.write(" ".join([os.path.join(BIN, "sumo")] + p.sumo + fcd_opts) + "\n")
        f.write(" ".join(ps) + "\n")
    if sumo_failed:
        # tests of error handling: psim2 must quit with the same error
        try:
            r = subprocess.run(ps, cwd=work, capture_output=True, text=True, timeout=600)
        except subprocess.TimeoutExpired:
            return rel, "SUMO failed: %s, psim2 timeout" % sumo_error + note
        if r.returncode > 0 and sumo_error and first_error(r) == sumo_error:
            shutil.rmtree(work, ignore_errors=True)
            return rel, "expected error matched: %s%s, differing steps reported: 0" % (sumo_error, note)
        return rel, "SUMO failed: %s | psim2: %s" % (sumo_error, first_error(r) if r.returncode != 0 else "ran") + note
    try:
        r = subprocess.run(ps, cwd=work, capture_output=True, text=True, timeout=600)
    except subprocess.TimeoutExpired:
        return rel, "psim2 timeout (600 s)" + note
    if r.returncode != 0:
        return rel, "psim2 failed: " + last_line(r) + note
    r = subprocess.run([sys.executable, os.path.join(HERE, "fcddiff.py"), work, "1"], capture_output=True, text=True)
    if r.returncode:
        return rel, "FCD comparison failed: " + last_line(r)
    result = " | ".join(r.stdout.strip().splitlines()[-3:]) + note
    for flag, name, root, what in ((p.blocks, "railsignal_blocks.xml", "<railsignal-block-output", "railsignal blocks"),
                                   (p.deadlocks, "deadlocks.xml", "<additional", "deadlocks"),
                                   (p.rsvehicles, "railsignal_vehicles.xml", "<railsignal-vehicle-output", "railsignal vehicles")):
        diff = compare_xml(work, name, root) if flag else ""
        if diff:
            result = result.replace("differing steps reported: 0", "fcd steps differing: 0") + " | %s differ: %s" % (what, diff)
    tidy_outputs(work, "differing steps reported: 0" in result)
    return rel, result


def compare_xml(work, name, root):
    """an XML output of both runs (the elements below the root, without SUMO's header); empty if equal, else the
    first difference"""
    def body(path):
        if not os.path.exists(path):
            return None
        lines = open(path).read().splitlines()
        for i, line in enumerate(lines):
            if line.startswith(root):
                return [x.rstrip() for x in lines[i + 1:] if x.strip()]
        raise ValueError(f"missing {root} in {path}")
    s, p = body(os.path.join(work, "S", name)), body(os.path.join(work, "P", name))
    if s is None or p is None:
        return "missing output (SUMO %s, psim2 %s)" % (s is not None, p is not None)
    for i in range(max(len(s), len(p))):
        a = s[i] if i < len(s) else "<end>"
        b = p[i] if i < len(p) else "<end>"
        if a != b:
            return "line %d: %s ## %s" % (i + 1, a.strip()[:120], b.strip()[:120])
    return ""


def tidy_outputs(work, exact):
    """disk usage: the work directories of exact tests are removed (they are regenerated by a new run), the large
    outputs of differing tests compressed"""
    if exact and not os.environ.get("PSIM2_KEEP"):
        shutil.rmtree(work, ignore_errors=True)
        return
    for sub in ("S", "P"):
        d = os.path.join(work, sub)
        if not os.path.isdir(d):
            continue
        if exact:
            shutil.rmtree(d, ignore_errors=True)
            continue
        for name in os.listdir(d):
            path = os.path.join(d, name)
            if name.endswith(".gz") or os.path.getsize(path) < (1 << 20):
                continue
            with open(path, "rb") as fin, gzip.open(path + ".gz", "wb", compresslevel=3) as fout:
                shutil.copyfileobj(fin, fout, 1 << 20)
            os.remove(path)


def trip_means(path, csv_format):
    """per-run means of the trip statistics"""
    keys = ("duration", "timeLoss", "waitingTime", "departDelay", "routeLength", "rerouteNo", "stopTime")
    if csv_format:
        with open(path) as f:
            rows = [{k: float(r.get(k, 0)) for k in keys + ("arrival",)} for r in csv.DictReader(f)]
    else:
        rows = [{k: float(t.get(k, "0")) for k in keys + ("arrival",)} for t in ET.parse(path).getroot().iter("tripinfo")]
    # the means include the vehicles still on the way at the end (write-unfinished: arrival -1)
    out = {"arrived": float(sum(1 for r in rows if r["arrival"] >= 0))}
    for k in keys:
        out[k] = statistics.fmean(r[k] for r in rows) if rows else float("nan")
    return out


def walk_means(path, csv_format):
    """per-run means of the walk statistics (personinfo), None without persons"""
    keys = ("duration", "routeLength", "timeLoss", "waitingTime")
    if csv_format:
        if not os.path.exists(path):
            return None
        with open(path) as f:
            rows = [{k: float(r[k]) for k in keys} for r in csv.DictReader(f) if float(r["arrival"]) >= 0]
    else:
        rows = []
        for p in ET.parse(path).getroot().iter("personinfo"):
            for w in p.iter("walk"):
                if float(w.get("arrival", "-1")) >= 0:
                    rows.append({k: float(w.get(k, "0")) for k in keys})
        if not rows:
            return None
    out = {"walks": float(len(rows))}
    for k in keys:
        out["walk." + k] = statistics.fmean(r[k] for r in rows) if rows else float("nan")
    return out


def run_stats(test, n):
    """randomness on: n SUMO seeds against n psim2 seeds, trip statistics compared"""
    rel = os.path.relpath(test, TESTS)
    if rel in SKIP_TESTS:
        return rel, "SKIP " + SKIP_TESTS[rel]
    work = os.path.join(WORK, "stats", rel)
    p = prepare(test, work)
    if isinstance(p, str):
        return rel, p
    note = (" [unhandled: %s]" % " ".join(p.unhandled)) if p.unhandled else ""
    sumo_args = [a for i, a in enumerate(p.sumo) if a != "--seed" and (i == 0 or p.sumo[i - 1] != "--seed")]
    psim_args = [a for i, a in enumerate(p.psim) if a not in ("--seed", "--sumo-seed")
                 and (i == 0 or p.psim[i - 1] not in ("--seed", "--sumo-seed"))]
    S, P = [], []
    for seed in range(1, n + 1):
        r = subprocess.run([os.path.join(BIN, "sumo")] + sumo_args + ["--seed", str(seed), "--tripinfo-output",
                                                                       "S%d.xml" % seed, "--tripinfo-output.write-unfinished",
                                                                       "--no-warnings", "true"],
                           cwd=work, capture_output=True, text=True)
        if r.returncode != 0:
            return rel, "SUMO failed: " + last_line(r) + note
        S.append(trip_means(os.path.join(work, "S%d.xml" % seed), False))
        S[-1].update(walk_means(os.path.join(work, "S%d.xml" % seed), False) or {})
        os.makedirs(os.path.join(work, "P%d" % seed), exist_ok=True)
        r = subprocess.run([PSIM2_BIN, "scenario.psim2", "--out", "P%d/" % seed,
                            "--seed", str(seed), "--sumo-seed", str(seed), "--no-lc-output", "--tripinfo-unfinished"] + psim_args, cwd=work,
                           capture_output=True, text=True)
        if r.returncode != 0:
            return rel, "psim2 failed: " + last_line(r) + note
        P.append(trip_means(os.path.join(work, "P%d" % seed, "tripinfo.csv"), True))
        P[-1].update(walk_means(os.path.join(work, "P%d" % seed, "personinfo.csv"), True) or {})
    cells, worst = [], 0.0
    keys = ["arrived", "duration", "routeLength", "rerouteNo", "timeLoss", "waitingTime", "departDelay", "stopTime"]
    if "walks" in S[0]:
        keys += ["walks", "walk.duration", "walk.routeLength", "walk.timeLoss", "walk.waitingTime"]
    for k in keys:
        if any(k not in x or math.isnan(x[k]) for x in S + P):
            continue
        s = [x[k] for x in S]
        q = [x[k] for x in P]
        ms, mp = statistics.fmean(s), statistics.fmean(q)
        se = math.sqrt(statistics.pvariance(s) / n + statistics.pvariance(q) / n) if n > 1 else 0
        z = (mp - ms) / se if se > 1e-9 else (0.0 if abs(mp - ms) < 1e-6 else float("inf"))
        worst = max(worst, abs(z))
        cells.append("%s %.1f/%.1f (z %+.1f)" % (k, ms, mp, z))
    if not os.environ.get("PSIM2_KEEP"):
        shutil.rmtree(work, ignore_errors=True)  # (the trip files of all seeds)
    return rel, ("OK " if worst < 3 else "CHECK ") + "; ".join(cells) + note


def main():
    global TESTS
    ap = argparse.ArgumentParser()
    ap.add_argument("tests", nargs="+")
    ap.add_argument("--keep-sigma", action="store_true")
    ap.add_argument("--parallel", action="store_true")
    ap.add_argument("--derandomize", action="store_true")
    ap.add_argument("--stats", type=int, default=0, help="randomness on: compare N SUMO seeds with N psim2 seeds")
    ap.add_argument("--add-options", default="", help="SUMO options appended to every test (e.g. '--step-method.ballistic')")
    ap.add_argument("--sumo-tests", default=os.environ.get("PSIM2_SUMO_TESTS", TESTS),
                    help="root of the SUMO v1_28_0 tests/sumo tree")
    ap.add_argument("--expect-count", type=int, help="require exactly this many unique cases")
    a = ap.parse_args()
    TESTS = os.path.realpath(a.sumo_tests)
    EXTRA_OPTIONS.extend(shlex.split(a.add_options))
    tests = []
    for t in a.tests:
        path = os.path.realpath(os.path.join(TESTS, t))
        if os.path.commonpath([TESTS, path]) != TESTS or not os.path.isdir(path):
            ap.error("missing or out-of-tree test directory: " + path)
        found = leaf_tests(path)
        if not found:
            ap.error("no test cases in " + path)
        if path == os.path.join(TESTS, "rail") and len(found) != 323:
            ap.error("expected 323 railway cases, discovered %d" % len(found))
        tests.extend(found)
    tests = sorted(set(tests))
    workroot = os.path.realpath(WORK)
    if os.path.commonpath([workroot, TESTS]) in (workroot, TESTS) or TESTS in tests:
        ap.error("use disjoint work and input directories and select named cases or suites")
    if a.expect_count is not None and len(tests) != a.expect_count:
        ap.error("expected %d cases, discovered %d" % (a.expect_count, len(tests)))
    passed = errors = 0
    for t in tests:
        if a.stats:
            rel, result = run_stats(t, a.stats)
        else:
            rel, result = run_test(t, a.keep_sigma, a.parallel, a.derandomize)
        print("%-70s %s" % (rel, result), flush=True)
        matched = result.startswith("expected error matched:")
        exact = result.startswith("compared ") and "differing steps reported: 0" in result
        passed += result.startswith("OK ") if a.stats else (matched or exact)
        errors += matched
    print(f"{passed}/{len(tests)} passed ({errors} matched errors)")
    return int(passed != len(tests))


if __name__ == "__main__":
    sys.exit(main())
