// Loading, the per-step phase sequence and outputs.
//
// Step order follows MSNet::simulationStep: traffic-light switches, plan
// movements, register junction approaches, execute movements, lane
// changing, insertion, teleport/parking transfers.
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <zlib.h>

#include "model.hpp"

namespace ps {

Time DELTA_T = 1000;
double TS = 1.0;
bool BALLISTIC = false;

static double nowSec() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

static void setTypeParam(VType& t, const std::string& key, double value) {
    if (key == "accel") t.accel = value;
    else if (key == "decel") t.decel = value;
    else if (key == "emergencyDecel") t.emergencyDecel = value;
    else if (key == "apparentDecel") t.apparentDecel = value;
    else if (key == "tau") t.tau = value;
    else if (key == "sigma") t.sigma = value;
    else if (key == "length") t.length = value;
    else if (key == "minGap") t.minGap = value;
    else if (key == "maxSpeed") t.maxSpeed = value;
    else if (key == "desiredMaxSpeed") t.desiredMaxSpeed = value;
    else if (key == "speedFactorMean") t.speedFactorMean = value;
    else if (key == "speedFactorDev") t.speedFactorDev = value;
    else if (key == "speedFactorMin") t.speedFactorMin = value;
    else if (key == "speedFactorMax") t.speedFactorMax = value;
    else if (key == "lcStrategic") t.lcStrategic = value;
    else if (key == "lcCooperative") t.lcCooperative = value;
    else if (key == "lcSpeedGain") t.lcSpeedGain = value;
    else if (key == "lcKeepRight") t.lcKeepRight = value;
    else if (key == "lcAssertive") t.lcAssertive = value;
    else if (key == "maxSpeedLat") { t.maxSpeedLat = value; t.maxSpeedLatSet = true; }
    else if (key == "lcMaxSpeedLatStanding") t.lcMaxSpeedLatStanding = value;
    else if (key == "lcMaxSpeedLatFactor") t.lcMaxSpeedLatFactor = value;
    else if (key == "lcLookaheadLeft") t.lcLookaheadLeft = value;
    else if (key == "lcSpeedGainRight") t.lcSpeedGainRight = value;
    else if (key == "lcSpeedGainLookahead") { t.lcSpeedGainLookahead = value; t.lcSpeedGainLookaheadSet = true; }
    else if (key == "lcSpeedGainRemainTime") t.lcSpeedGainRemainTime = value;
    else if (key == "lcSpeedGainUrgency") t.lcSpeedGainUrgency = value;
    else if (key == "lcCooperativeSpeed") t.lcCooperativeSpeed = value;
    else if (key == "lcKeepRightAcceptanceTime") t.lcKeepRightAcceptanceTime = value;
    else if (key == "lcOvertakeDeltaSpeedFactor") t.lcOvertakeDeltaSpeedFactor = value;
    else if (key == "lcCooperativeHelpTime") t.lcCooperativeHelpTime = value;
    else if (key == "lcCooperativeHelpThreshold") t.lcCooperativeHelpThreshold = value;
    else if (key == "lcCooperativeMinSpeed") t.lcCooperativeMinSpeed = value;
    else if (key == "lcStrategicLookahead") t.lcStrategicLookahead = value;
    else if (key == "lcExperimental1") t.lcExperimental1 = value;
    else if (key == "lcCooperativeRoundabout") t.lcCooperativeRoundabout = value;
    else if (key == "lcOvertakeRight") t.lcOvertakeRight = value;
    else if (key == "lcModel") t.lcModel = (int)value;
    else if (key == "lcSublane") t.lcSublane = value;
    else if (key == "lcPushy") t.lcPushy = value;
    else if (key == "lcPushyGap") t.lcPushyGap = value;
    else if (key == "lcImpatience") t.lcImpatience = value;
    else if (key == "lcTimeToImpatience") t.lcTimeToImpatience = value;
    else if (key == "lcAccelLat") t.lcAccelLat = value;
    else if (key == "lcTurnAlignmentDistance") t.lcTurnAlignmentDistance = value;
    else if (key == "lcLaneDiscipline") t.lcLaneDiscipline = value;
    else if (key == "lcSigma") t.lcSigma = value;
    else if (key == "lcMaxDistLatStanding") t.lcMaxDistLatStanding = value;
    else if (key == "lcOpposite") t.lcOpposite = value;
    else if (key == "minGapLat") t.minGapLat = value;
    else if (key == "latAlignment") t.latAlignment = (int)value;
    else if (key == "latAlignmentOffset") t.latAlignmentOffset = value;
    else if (key == "width") t.width = value;
    else if (key == "personCapacity") t.personCapacity = (int)value;
    else if (key == "containerCapacity") t.containerCapacity = (int)value;
    else if (key == "boardingDuration") t.boardingDuration = TIME2STEPS(value);
    else if (key == "loadingDuration") t.loadingDuration = TIME2STEPS(value);
    else if (key == "boardingFactor") t.boardingFactor = value;
    else if (key == "impatience") t.impatience = value;
    else if (key == "jmDriveAfterRedTime") t.jmDriveAfterRedTime = value;
    else if (key == "jmDriveAfterYellowTime") t.jmDriveAfterYellowTime = value;
    else if (key == "timegapCrossing") t.timegapCrossing = value;
    else if (key == "locomotiveLength") t.locomotiveLength = value;
    else if (key == "shapeKind") t.shapeKind = (int)value;
    else if (key == "jmIgnoreFoeProb") t.jmIgnoreFoeProb = value;
    else if (key == "jmIgnoreFoeSpeed") t.jmIgnoreFoeSpeed = value;
    else if (key == "jmIgnoreJunctionFoeProb") t.jmIgnoreJunctionFoeProb = value;
    else if (key == "jmCrossingGap") t.jmCrossingGap = value;
    else if (key == "jmStoplineGap") t.jmStoplineGap = value;
    else if (key == "jmStoplineGapMinor") t.jmStoplineGapMinor = value;
    else if (key == "jmStoplineCrossingGap") t.jmStoplineCrossingGap = value;
    else if (key == "jmTimegapMinor") t.jmTimegapMinor = value;
    else if (key == "jmAllwayStopWait") t.jmAllwayStopWait = value;
    else if (key == "jmStopSignWait") t.jmStopSignWait = value;
    else if (key == "actionStepLength") t.actionStepLength = value;
    else if (key == "speedFactorPremature") t.speedFactorPremature = value;
    else throw std::runtime_error("unknown vType parameter " + key);
}


namespace {
// the lines of a text file, gzip-compressed or not (zlib reads uncompressed files unchanged)
class LineReader {
public:
    explicit LineReader(const std::string& path) : f(gzopen(path.c_str(), "rb")) {
        if (f == nullptr) {
            throw std::runtime_error("cannot open " + path);
        }
        gzbuffer(f, 1 << 18);
    }
    ~LineReader() { gzclose(f); }
    LineReader(const LineReader&) = delete;
    LineReader& operator=(const LineReader&) = delete;
    void rewind() {
        gzrewind(f);
        pos = len = 0;
        eof = false;
    }
    // like std::getline: false at the end of the file
    bool getline(std::string& line) {
        line.clear();
        while (true) {
            if (pos == len) {
                if (eof) {
                    return !line.empty();
                }
                const int n = gzread(f, buf, sizeof(buf));
                if (n < 0) {
                    throw std::runtime_error("read error");
                }
                if (n == 0) {
                    eof = true;
                    return !line.empty();
                }
                pos = 0;
                len = (size_t)n;
            }
            const char* start = buf + pos;
            const char* nl = static_cast<const char*>(std::memchr(start, '\n', len - pos));
            if (nl != nullptr) {
                line.append(start, (size_t)(nl - start));
                pos = (size_t)(nl - buf) + 1;
                return true;
            }
            line.append(start, len - pos);
            pos = len;
        }
    }

private:
    gzFile f;
    char buf[1 << 16];
    size_t pos = 0, len = 0;
    bool eof = false;
};
}  // namespace

void Sim::load(const std::string& file) {
    LineReader in(file);
    std::string line;
    // the vehicles are counted first: one allocation instead of the doubling growth (peak memory)
    size_t nVeh = 0;
    while (in.getline(line)) {
        if (line.compare(0, 4, "VEH ") == 0) {
            ++nVeh;
        }
    }
    veh.reserve(nVeh);
    kin.reserve(nVeh);
    in.rewind();
    std::vector<std::string> tok;
    while (in.getline(line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        tok.clear();
        std::istringstream ss(line);
        std::string w;
        while (ss >> w) {
            tok.push_back(w);
        }
        if (tok.empty() || tok[0][0] == '#') {
            continue;
        }
        const std::string& k = tok[0];
        if (k == "STEP") {
            TS = std::stod(tok[1]);
            DELTA_T = TIME2STEPS(TS);
        } else if (k == "TYPE") {
            VType t;
            t.id = tok[1];
            t.vclass = vclassFromName(tok[2]);
            net.usedClasses |= t.vclass;
            net.usedIgnoring |= t.vclass == 0;
            for (size_t i = 3; i < tok.size(); ++i) {
                const size_t eq = tok[i].find('=');
                setTypeParam(t, tok[i].substr(0, eq), std::stod(tok[i].substr(eq + 1)));
            }
            t.initDerived();
            initCarFollowModel(t);
            typeIdx[t.id] = (int)types.size();
            types.push_back(t);
        } else if (k == "CFTYPE") {
            setCarFollowModel(types[typeIdx.at(tok[1])], tok);
        } else if (k == "TAZ") {
            // TAZ id source,source...|- sink,sink...|-
            Taz z;
            z.id = tok[1];
            auto edgesOf = [&](const std::string& list, std::vector<int>& into) {
                if (list == "-") {
                    return;
                }
                std::stringstream ss(list);
                std::string e;
                while (std::getline(ss, e, ',')) {
                    into.push_back(net.edgeIdx.at(e));
                }
            };
            edgesOf(tok[2], z.sources);
            edgesOf(tok[3], z.sinks);
            tazIdx[z.id] = (int)tazs.size();
            tazs.push_back(z);
        } else if (k == "WEIGHT") {
            // WEIGHT edge begin end traveltime (MSNet::getWeightsStorage)
            if (loadedWeights.empty()) {
                loadedWeights.resize(net.edges.size());
            }
            loadedWeights[net.edgeIdx.at(tok[1])].push_back({std::stod(tok[2]), std::stod(tok[3]), std::stod(tok[4])});
        } else if (k == "PREFERENCE") {
            preferenceLines.push_back(tok);  // resolved with the vehicle types (buildRoutingGraph)
        } else if (k == "TYPEDIST") {
            // TYPEDIST id type prob type prob ...
            std::vector<std::pair<int, double>>& d = typeDists[tok[1]];
            for (size_t i = 2; i + 1 < tok.size(); i += 2) {
                d.push_back({typeIdx.at(tok[i]), std::stod(tok[i + 1])});
            }
        } else if (k == "RSC") {
            railConstraintLines.push_back(tok);
        } else if (k == "DEADLOCK") {
            railDeadlockLines.push_back(std::vector<std::string>(tok.begin() + 1, tok.end()));
        } else if (k == "RAILTYPE") {
            setRailType(types[typeIdx.at(tok[1])], tok);
        } else if (k == "BUSSTOP") {
            BusStop b;
            b.id = tok[1];
            b.lane = net.laneIdx.at(tok[2]);
            // NLTriggerBuilder::parseAndBuildStoppingPlace, SUMORouteHandler::checkStopPos: the end defaults to
            // the lane end, negative positions count from the lane end
            const double laneLength = net.lanes[b.lane].length;
            b.startPos = std::stod(tok[3]);
            b.endPos = tok[4] == "-" ? laneLength : std::stod(tok[4]);
            if (b.startPos < 0) {
                b.startPos += laneLength;
            }
            if (b.endPos < 0) {
                b.endPos += laneLength;
            }
            // friendlyPos (without it SUMO refuses invalid positions)
            b.endPos = std::max(POSITION_EPS, std::min(b.endPos, laneLength));
            b.startPos = std::max(0.0, std::min(b.startPos, b.endPos - POSITION_EPS));
            b.parkingLength = std::stod(tok[5]);
            b.parkingFactor = b.parkingLength <= 0 ? 1 : (b.endPos - b.startPos) / b.parkingLength;
            b.lastFreePos = b.endPos;
            if (tok.size() > 7) {
                b.angle = std::stod(tok[7]) * M_PI / 180.;  // NLTriggerBuilder: default 90 degrees
            }
            if (tok.size() > 8) {
                b.category = tok[8][0];
            }
            if (tok.size() > 6 && tok[6] != "-") {
                b.waitingCapacity = std::stoi(tok[6]);
            } else {
                // NLTriggerBuilder::parseAndBuildStoppingPlace: three rows of persons (at least 6), one of containers
                const int abreast = std::max(1, (int)std::floor((b.endPos - b.startPos) / (b.category == 'C' ? 2.5 : 0.8)));
                b.waitingCapacity = b.category == 'C' ? abreast : std::max(abreast * 3, 6);
            }
            if (tok.size() > 9) {
                b.loadOrder = std::stoi(tok[9]);
                for (size_t i = 10; i < tok.size(); ++i) {
                    b.name += (i > 10 ? " " : "") + tok[i];
                }
            }
            busStopIdx[b.id] = (int)busStops.size();
            busStops.push_back(b);
        } else if (k == "BSACCESS") {
            // NLTriggerBuilder::addAccess, MSStoppingPlace::addAccess (BSACCESS stop lane pos|random|doors|carriage length
            // friendlyPos)
            BusStop& b = busStops[busStopIdx.at(tok[1])];
            BusStop::Access acc;
            acc.lane = net.laneIdx.at(tok[2]);
            const double laneLength = net.lanes[acc.lane].length;
            if (tok[3] == "random" || tok[3] == "doors" || tok[3] == "carriage") {
                acc.startPos = 0;
                acc.endPos = laneLength;
                acc.exit = tok[3] == "doors" ? 1 : (tok[3] == "carriage" ? 2 : 0);
            } else {
                acc.startPos = std::stod(tok[3]);
                if (acc.startPos < 0) {
                    acc.startPos += laneLength;
                }
                if (tok[5] == "1") {
                    acc.startPos = std::max(0.0, std::min(acc.startPos, laneLength));
                }
                acc.endPos = acc.startPos;
            }
            acc.length = std::stod(tok[4]);
            if (acc.length < 0) {
                const Lane& al = net.lanes[acc.lane];
                const Lane& sl = net.lanes[b.lane];
                const Pos accPos = geom::positionAtOffset(al.shape, al.interpolateLanePosToGeometryPos((acc.startPos + acc.endPos) / 2.), 0);
                const Pos stopPos = geom::positionAtOffset(sl.shape, sl.interpolateLanePosToGeometryPos((b.startPos + b.endPos) / 2.), 0);
                acc.length = accPos.distanceTo2D(stopPos);
            }
            bool dup = false;
            for (const BusStop::Access& o : b.accesses) {
                dup = dup || o.lane == acc.lane;
            }
            if (!dup && (net.lanes[acc.lane].permissions & VC_PEDESTRIAN) != 0) {
                b.accesses.push_back(acc);
            }
        } else if (k == "VDEFAULTS") {
            vDefDepartLane = tok[1];
            vDefDepartSpeed = tok[2];
        } else if (k == "PTSCHED") {
            // PTSCHED F|V id line begin repetitionNumber offset lastDepart stop:until...
            PTSched ps;
            ps.flow = tok[1] == "F";
            ps.id = tok[2];
            ps.line = tok[3];
            ps.begin = std::stoll(tok[4]);
            ps.repetitionNumber = std::stoll(tok[5]);
            ps.offset = std::stoll(tok[6]);
            ps.lastDepart = std::stoll(tok[7]);
            for (size_t i = 8; i < tok.size() && tok[i] != "-"; ++i) {
                const size_t c = tok[i].rfind(':');
                const std::string place = tok[i].substr(0, c);
                ps.stops.push_back({place == "-" ? "" : place, std::stoll(tok[i].substr(c + 1))});
            }
            ptScheds.push_back(std::move(ps));
        } else if (k == "PARKINGAREA" || k == "PASPACE") {
            parseParkingAreaLine(tok);
        } else if (k == "ROUTE") {
            std::vector<int> r;
            for (size_t i = 2; i < tok.size(); ++i) {
                r.push_back(net.edgeIdx.at(tok[i]));
            }
            routeIdx[tok[1]] = (int)routes.size();
            routes.push_back(r);
        } else if (k == "VEH") {
            parseVehicleLine(tok);
        } else if (k == "FLOW") {
            parseFlowLine(tok);
        } else if (k == "STOP") {
            parseStopLine(tok);
        } else if (k == "VOPT") {
            parseVehicleOptions(tok);
        } else if (k == "REROUTER" || k.rfind("RR", 0) == 0) {
            parseRerouterLine(tok);
        } else if (k == "VSS") {
            speedSignLines.push_back(tok);  // after the network
        } else if (k == "PERSON" || k == "PFLOW" || k == "PWALK" || k == "PSTOP" || k == "PTRIP" || k == "PRIDE" || k == "CONTAINER"
                   || k == "CFLOW" || k == "PTRANSHIP") {
            personLines.push_back(tok);  // resolved after the network (edge lengths)
        } else {
            net.loadLine(tok);
        }
    }
    // MSEdge::closeBuilding: every edge has a lane changer with continuous or sublane changes
    net.allEdgesChangeLanes = opt.laneChangeDuration > 0 || opt.lateralResolution > 0;
    net.lateralResolution = opt.lateralResolution;
    net.finish();
    loadPersonLines();
    generateFlows();
    // vehicles are sorted by their departure step (MSVehicleContainer rounds the depart time up to the step; stable:
    // file order within a step); MSInsertionControl: flow vehicles are built when they are due, after the loaded
    // vehicles of the step
    std::vector<int> order(veh.size());
    for (size_t i = 0; i < order.size(); ++i) {
        order[i] = (int)i;
    }
    auto departStep = [](Time d) { return d <= 0 ? d / DELTA_T : (d + DELTA_T - 1) / DELTA_T; };
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
        const Time sa = departStep(veh[a].depart), sb = departStep(veh[b].depart);
        return sa != sb ? sa < sb : (!veh[a].fromFlow && veh[b].fromFlow);
    });
    // the permutation is applied in place, cycle by cycle (no second copy of all vehicles: peak memory)
    for (size_t i = 0; i < order.size(); ++i) {
        if (order[i] < 0 || order[i] == (int)i) {
            continue;
        }
        Veh tv = std::move(veh[i]);
        const Kin tk = kin[i];
        size_t j = i;
        while (true) {
            const size_t k = (size_t)order[j];
            order[j] = -1;
            if (k == i) {
                veh[j] = std::move(tv);
                kin[j] = tk;
                break;
            }
            veh[j] = std::move(veh[k]);
            kin[j] = kin[k];
            j = k;
        }
    }
    // route ids are only needed while loading
    std::unordered_map<std::string, int>().swap(routeIdx);
    // MSNet::addStoppingPlace: named stopping places (train stops count as bus stops)
    for (int b = 0; b < (int)busStops.size(); ++b) {
        if (!busStops[b].name.empty()) {
            namedStops[{busStops[b].category, busStops[b].name}].push_back(b);
        }
    }
    for (auto& item : namedStops) {
        std::sort(item.second.begin(), item.second.end(),
                  [&](int a, int b) { return busStops[a].loadOrder < busStops[b].loadOrder; });
    }
    // trains splitting or joining at stops: the vehicle indices of the other parts, and an own type for the
    // trains whose length changes (MSBaseVehicle::getSingularType)
    {
        std::unordered_map<std::string, int> ids;
        std::set<std::string> joinTargets;
        for (const Veh& x : veh) {
            for (const Stop& s : x.stops) {
                if (!s.split.empty() || !s.join.empty()) {
                    haveJoinSplit = true;
                    joinTargets.insert(s.join);
                }
            }
        }
        if (haveJoinSplit) {
            for (int v = 0; v < (int)veh.size(); ++v) {
                ids[veh[v].id] = v;
            }
            for (int v = 0; v < (int)veh.size(); ++v) {
                bool own = joinTargets.count(veh[v].id) > 0;
                for (Stop& s : veh[v].stops) {
                    if (!s.split.empty()) {
                        auto it = ids.find(s.split);
                        s.splitVeh = it == ids.end() ? -1 : it->second;
                        own = true;
                    }
                    if (!s.join.empty()) {
                        auto it = ids.find(s.join);
                        s.joinVeh = it == ids.end() ? -1 : it->second;
                        own = true;
                    }
                }
                if (own) {
                    types.push_back(types[kin[v].type]);
                    kin[v].type = (int)types.size() - 1;
                }
            }
        }
    }
    // person trips may build a vehicle each (MSStageTrip::reroute): reserved, so that the vehicles never move
    size_t tripVehicles = 0;
    for (const Person& p : persons) {
        for (const PStage& s : p.plan) {
            tripVehicles += s.kind == PStage::TRIP && (!s.tripVTypes.empty() || (s.tripModes & (VC_PASSENGER | VC_BICYCLE)) != 0);
        }
    }
    if (tripVehicles > 0) {
        veh.reserve(veh.size() + tripVehicles);
        kin.reserve(kin.size() + tripVehicles);
    }
    for (size_t v = 0; v < veh.size(); ++v) {
        if (veh[v].routingMode < 0) {
            veh[v].routingMode = (int8_t)opt.routingMode;  // MSBaseVehicle::initDevices: getStringParam
        }
        veh[v].randomSeed = sumoMurmur3(veh[v].id, opt.sumoSeed);
        if (veh[v].departTriggered) {
            // MSRouteHandler::closeVehicle: waits on its first edge (MSEdge::addWaiting), not active
            triggeredWaiting[routes[veh[v].route][veh[v].routeIdx]].push_back((int)v);
            nTriggeredDepart++;
        }
        resolveArrival((int)v);
        if (veh[v].id == opt.trace) {
            traceVeh = (int)v;
        }
    }
    if (!opt.editsFile.empty()) {
        loadEdits(opt.editsFile);
    }
    initSpeedSigns();
    initRerouters();
    initRouting();
    updateMaxBrakeDist();
    lanes.assign(net.lanes.size(), LaneRT());
    edgesRT.assign(net.edges.size(), EdgeRT());
    edgeDelayed.assign(net.edges.size(), 0);
    approaching.assign(net.links.size(), {});
    bruttoSnap.assign(net.lanes.size(), 0);
    bruttoSnapMark.assign(net.lanes.size(), 0);
    kinSnap.assign(veh.size(), Kin());
    // car-following model state (MSCFModel::createVehicleVariables)
    for (const VType& t : types) {
        cfHaveVars |= t.startupDelay > 0 || cfNeedsVars(t);
    }
    if (cfHaveVars) {
        cfVars.reserve(veh.capacity());
        cfVars.resize(veh.size());
        for (int v = 0; v < (int)veh.size(); ++v) {
            initCarFollowVars(v);
        }
    }
    // MSGlobals::gSublane (MSFrame::setMSGlobals)
    sublane = opt.laneChangeDuration > 0 || opt.lateralResolution > 0;
    latRes = opt.lateralResolution;
    if (sublane) {
        lat.reserve(veh.capacity());
        lat.resize(veh.size());
        latSnap.resize(veh.size());
        if (latRes > 0) {
            sl.reserve(veh.capacity());
            sl.resize(veh.size());
            for (int v = 0; v < (int)veh.size(); ++v) {
                sl[v].impatience = vt(v).lcImpatience;
            }
        }
        lcManeuverSnap.assign(veh.size(), 0);
        lcStratBlockedSnap.assign(veh.size(), 0);
        lcSpeedLatSnap.assign(veh.size(), 0);
    }
    lcSnapStage.assign(veh.size(), -1);
    lcStateSnap.assign(veh.size(), 0);
    lcLeftSpaceSnap.assign(veh.size(), 0);
    lcMessages.assign(net.edges.size(), {});
    lcEventsPerEdge.assign(net.edges.size(), {});
    lcShadowPerEdge.assign(net.edges.size(), {});
    // MSFrame::setMSGlobals, MSVehicleType::check: action steps longer than the step length switch to the ballistic
    // update unless the update method was given
    if (!opt.ballisticSet) {
        bool longActionSteps = opt.defaultActionStepLengthSet;
        for (int v = 0; v < (int)veh.size() && !longActionSteps; ++v) {
            longActionSteps = actionStepLength(v) != DELTA_T;
        }
        for (const FlowDef& f : flows) {
            longActionSteps = longActionSteps || (f.type >= 0 && typeActionStepLength(types[f.type]) != DELTA_T);
        }
        if (longActionSteps) {
            BALLISTIC = true;
        }
    }
    now = TIME2STEPS(opt.begin);
    initTLS();
    initRail();
    initOvertakeLocations();
    initParkingAreas();
    // SUMO's active-lane order (MSEdgeControl::myActiveLanes): lane changing with --sumo-order, the parking
    // area states seen by parking searches
    trackActiveList = opt.sumoOrder || (hasParkingAreas() && hasRerouters());
    computeEdgeLevels();
    initPersons();
    initPersonPreferences();
    loadRoutesStep();  // NLBuilder::init preloads the routes (MSNet::loadRoutes) before the first step
}

// ---------------------------------------------------------------------------
// phases
// ---------------------------------------------------------------------------
void Sim::planMovements() {
    // vehicles that may board persons are planned afterwards, serially in lane order (boarding changes
    // persons and the waiting lists)
    const bool boarding = !waiting4Vehicle.empty();
    if (boarding && planDeferred.size() < activeLanes.size()) {
        planDeferred.resize(activeLanes.size());
    }
    pool.setTag(ThreadPool::T_PLAN);
    pool.parallel_for((int)activeLanes.size(), [&](int i) {
        const int lane = activeLanes[i];
        const LaneRT& L = lanes[lane];
        double cumulatedVehLength = 0;
        int leader = -1;
        int p = (int)L.partials.size() - 1;
        if (boarding) {
            planDeferred[i].clear();
        }
        if (latRes > 0) {
            // leaders per sublane (MSLane::planMovements, updateLeaderInfo)
            LeaderInfo leaders(*this, net.lanes[lane].width);
            for (int j = (int)L.vehs.size() - 1; j >= 0; --j) {
                const int v = L.vehs[j];
                while (p >= 0 && positionOnLane(L.partials[p], lane) > kin[v].pos) {
                    leaders.addLeader(L.partials[p], false, latOffset(L.partials[p], lane));
                    --p;
                }
                if (boarding && mayBoard(v)) {
                    planDeferred[i].push_back({v, -1, cumulatedVehLength, std::make_shared<LeaderInfo>(leaders)});
                } else {
                    planMove(v, -1, cumulatedVehLength, &leaders);
                }
                cumulatedVehLength += vt(v).lengthWithGap();
                leaders.addLeader(v, false, 0);
            }
            return;
        }
        for (int j = (int)L.vehs.size() - 1; j >= 0; --j) {
            const int v = L.vehs[j];
            // partial occupants ahead of v (MSLane::updateLeaderInfo)
            while (p >= 0 && positionOnLane(L.partials[p], lane) > kin[v].pos) {
                leader = L.partials[p];
                --p;
            }
            if (boarding && mayBoard(v)) {
                planDeferred[i].push_back({v, leader, cumulatedVehLength, nullptr});
            } else {
                planMove(v, leader, cumulatedVehLength);
            }
            cumulatedVehLength += vt(v).lengthWithGap();
            leader = v;
        }
    }, 4);
    if (boarding) {
        for (size_t i = 0; i < activeLanes.size(); ++i) {
            for (const DeferredPlan& d : planDeferred[i]) {
                planMove(d.veh, d.leader, d.lengthsInFront, d.ahead.get());
            }
        }
    }
}

// MSEdgeControl::setJunctionApproaches. Every per-link list is sorted by vehicle; a vehicle with a
// looped route keeps its first registration (emplace). The links are split into ranges of ids
// (partitions): chunks of the running vehicles write their registrations into one bucket per
// partition, then each partition builds the lists of its links from the buckets (chunk order) and
// sorts them. No locks, and each link list is written by one thread.
void Sim::setJunctionApproaches() {
    for (int l : approachMoved) {
        approaching[l].clear();
    }
    approachMoved.clear();
    const int nLinks = (int)approaching.size();
    const int nChunks = pool.size() * 8 + 1;
    const int nParts = std::max(1, std::min(nLinks, pool.size() * 8));
    if (approachParts != nParts || approachNLinks != nLinks) {
        // partition layout changed (thread count or network edit): start from empty lists
        for (std::vector<Approach>& a : approaching) {
            a.clear();
        }
        approachTouchedOf.assign(nParts, {});
        approachParts = nParts;
        approachNLinks = nLinks;
    }
    if ((int)approachBuckets.size() != nChunks * nParts) {
        approachBuckets.assign((size_t)nChunks * nParts, {});
    }
    auto partOf = [&](int link) { return (int)((long long)link * nParts / nLinks); };
    pool.setTag(ThreadPool::T_APPROACH);
    pool.parallel_for(nChunks, [&](int c) {
        std::vector<ApproachRec>* buckets = &approachBuckets[(size_t)c * nParts];
        for (int p = 0; p < nParts; ++p) {
            buckets[p].clear();
        }
        const size_t from = running.size() * c / nChunks;
        const size_t to = running.size() * (c + 1) / nChunks;
        for (size_t i = from; i < to; ++i) {
            const int v = running[i];
            const Veh& x = veh[v];
            if (x.status != VehStatus::RUNNING) {
                continue;
            }
            const double len = vt(v).length;
            for (const DriveItem& d : x.lf) {
                if (d.link < 0) {
                    continue;
                }
                ApproachRec& r = buckets[partOf(d.link)].emplace_back();
                r.link = d.link;
                Approach& ap = r.ap;
                ap.veh = v;
                ap.arrivalTime = d.arrivalTime;
                ap.leavingTime = leaveTime(d.link, d.arrivalTime, d.arrivalSpeed, d.leaveSpeed(), len);
                ap.arrivalSpeed = d.arrivalSpeed;
                ap.leaveSpeed = d.leaveSpeed();
                ap.willPass = d.setRequest;
                ap.arrivalSpeedBraking = d.arrivalSpeedBraking;
                // (registered at the last action: MSVehicle::setApproachingForAllLinks)
                ap.waitingTime = d.link == x.actionHaveStoppedFor ? TIME_MAX : x.actionWaitingTime;
                ap.dist = d.distance;
                ap.speed = x.actionSpeed;
                ap.lane = kin[v].lane;
                ap.latOffset = x.actionPosLat;
            }
            if (x.actionShadowDir != 0) {
                // MSVehicle::setApproachingForAllLinks: the parallel links of the shadow lane
                for (const DriveItem& d : x.lf) {
                    if (d.link < 0) {
                        continue;
                    }
                    const int parallel = x.actionShadowDir < 0 ? net.links[d.link].parallelRight : net.links[d.link].parallelLeft;
                    if (parallel < 0) {
                        continue;
                    }
                    ApproachRec& r = buckets[partOf(parallel)].emplace_back();
                    r.link = parallel;
                    Approach& ap = r.ap;
                    ap.veh = v;
                    ap.arrivalTime = d.arrivalTime;
                    ap.leavingTime = leaveTime(parallel, d.arrivalTime, d.arrivalSpeed, d.leaveSpeed(), len);
                    ap.arrivalSpeed = d.arrivalSpeed;
                    ap.leaveSpeed = d.leaveSpeed();
                    ap.willPass = d.setRequest;
                    ap.arrivalSpeedBraking = d.arrivalSpeedBraking;
                    ap.waitingTime = d.link == x.actionHaveStoppedFor ? TIME_MAX : x.actionWaitingTime;
                    ap.dist = d.distance;
                    ap.speed = x.actionSpeed;
                    ap.lane = kin[v].lane;
                    ap.latOffset = x.actionShadowLatOffset;
                }
            }
        }
    }, 1);
    pool.parallel_for(nParts, [&](int p) {
        std::vector<int>& touched = approachTouchedOf[p];
        for (int l : touched) {
            approaching[l].clear();
        }
        touched.clear();
        for (int c = 0; c < nChunks; ++c) {
            for (const ApproachRec& r : approachBuckets[(size_t)c * nParts + p]) {
                std::vector<Approach>& a = approaching[r.link];
                if (a.empty()) {
                    touched.push_back(r.link);
                }
                a.push_back(r.ap);
            }
        }
        for (int l : touched) {
            std::vector<Approach>& a = approaching[l];
            if (a.size() > 1) {
                // a vehicle's own registrations keep their drive-item order (appended by one chunk)
                std::stable_sort(a.begin(), a.end(), [](const Approach& x, const Approach& y) { return x.veh < y.veh; });
                a.erase(std::unique(a.begin(), a.end(), [](const Approach& x, const Approach& y) { return x.veh == y.veh; }), a.end());
            }
        }
    }, 1);
}

void Sim::executeMovements() {
    snapshotBrutto(activeLanes);
    const int n = (int)activeLanes.size();
    laneDet.resize(n);
    laneStop.resize(n);
    laneEd.resize(n);
    laneAlight.resize(n);
    if (railActive) {
        dwEvents.resize(n);
        railResolvedDeadlocks.resize(n);
    }
    pool.setTag(ThreadPool::T_EXEC);
    deferReroutes = true;
    if (hasParkingAreas()) {
        if (laneExecOrder.size() != net.lanes.size()) {
            laneExecOrder.assign(net.lanes.size(), -1);
            laneExecStep.assign(net.lanes.size(), -1);
        }
        // SUMO's lane order of executeMovements (MSEdgeControl::myActiveLanes) with --sumo-order
        const std::vector<int>& order = trackActiveList ? activeList : activeLanes;
        for (int i = 0; i < (int)order.size(); ++i) {
            laneExecOrder[order[i]] = i;
            laneExecStep[order[i]] = (int)step;
        }
    }
    auto execLane = [&](int i) {
        const int lane = activeLanes[i];
        LaneRT& L = lanes[lane];
        execTaskLane = lane;
        execTaskOrder = hasParkingAreas() && laneExecStep[lane] == (int)step ? laneExecOrder[lane] : i;
        execTaskSeq = 0;
        std::vector<DetEvent>& det = laneDet[i];
        std::vector<StopEvent>& stopEv = laneStop[i];
        det.clear();
        stopEv.clear();
        laneEd[i].clear();
        L.outbox.clear();
        L.arrived.clear();
        L.arrivedLaneVehicles.clear();
        L.parked.clear();
        L.stay.clear();
        L.teleportVeh = -1;
        int firstNotStopped = -1;
        for (int j = (int)L.vehs.size() - 1; j >= 0; --j) {
            const int v = L.vehs[j];
            veh[v].emergencyStop = false;
            executeMove(v, det, stopEv, edFile != nullptr ? &laneEd[i] : nullptr, railActive ? &dwEvents[i] : nullptr);
            if (veh[v].extra.p != nullptr && (!veh[v].extra.p->riders.empty() || veh[v].extra.p->ridersStopped
                                              || !veh[v].extra.p->cargo.empty() || veh[v].extra.p->cargoStopped)) {
                // MSDevice_Transportable::notifyMove (a move reminder); the lane holds the vehicles that did not leave yet
                const int laneVehicles = (int)(L.vehs.size() + L.partials.size() - L.outbox.size() - L.arrived.size() - L.parked.size());
                ridersNotifyMove(v, laneVehicles, laneAlight[i]);
            }
            // MSBaseVehicle::isJumping: a stop with jump ended in this step and the vehicle is still on its edge
            const bool jumps = veh[v].jumpPending && curEdge(v) == veh[v].exr().jumpEdge;
            veh[v].jumpPending = false;
            if (hasArrivedInternal(v)) {
                L.arrivedLaneVehicles.push_back((int)(L.vehs.size() + L.partials.size() - L.outbox.size() - L.arrived.size()
                                                      - L.parked.size()));
                L.arrived.push_back(v);
            } else if (kin[v].lane != lane) {
                L.outbox.push_back(v);
            } else if (isParking(v)) {
                L.parked.push_back(v);
            } else if (jumps) {
                veh[v].jumping = true;  // MSLane::executeMovements: to the vehicle transfer
                L.parked.push_back(v);
            } else {
                if (firstNotStopped < 0 && !isStopped(v)) {
                    firstNotStopped = v;
                }
                L.stay.push_back(v);
            }
        }
        std::reverse(L.stay.begin(), L.stay.end());
        // teleport check (MSLane::executeMovements)
        const Time ttt = TIME2STEPS(opt.timeToTeleport);
        const bool r1 = firstNotStopped >= 0 && opt.timeToTeleport > 0 && veh[firstNotStopped].waitingTime > ttt;
        // trains in a rail signal deadlock
        bool r5 = false;
        if (!r1 && firstNotStopped >= 0 && opt.timeToTeleportRSDeadlock > 0 && !railSignals.empty()
                && veh[firstNotStopped].waitingTime > TIME2STEPS(opt.timeToTeleportRSDeadlock)) {
            int constraint;
            std::set<int> seen;
            std::vector<std::pair<int, RailWaitRelation>> circle;
            if (railHaveDeadlock(firstNotStopped, constraint, seen, &circle)) {
                // MSRailSignalControl::haveDeadlock: a constraint of the circle is deactivated instead (after the phase)
                railResolvedDeadlocks[i].push_back({constraint >= 0 && opt.removeConstraint ? constraint : -1, seen, circle});
                r5 = constraint < 0 || !opt.removeConstraint;
            }
        }
        if (r1 || r5) {
            const int v = firstNotStopped;
            const bool wrongLane = !appropriate(lane, v);
            const int link = succLinkSec(v, 1, lane, bestContinuation(v));
            const bool minorLink = !wrongLane && link >= 0 && !net.links[link].havePriority();
            L.teleportVeh = v;
            L.teleportReason = wrongLane ? 'w' : (minorLink ? 'y' : 'j');
        }
    };
    if (haveJoinSplit) {
        // trains splitting or joining change the other train part: serially, in SUMO's lane order
        // (MSEdgeControl::myActiveLanes) when it is tracked
        std::vector<int> order;
        if (trackActiveList) {
            std::unordered_map<int, int> idx;
            for (int i = 0; i < n; ++i) {
                idx[activeLanes[i]] = i;
            }
            for (int lane : activeList) {
                auto it = idx.find(lane);
                if (it != idx.end()) {
                    order.push_back(it->second);
                    idx.erase(it);
                }
            }
            for (int i = 0; i < n; ++i) {
                if (idx.count(activeLanes[i]) != 0) {
                    order.push_back(i);
                }
            }
        } else {
            for (int i = 0; i < n; ++i) {
                order.push_back(i);
            }
        }
        for (int i : order) {
            execLane(i);
        }
    } else {
        pool.parallel_for(n, execLane, 2);
    }
    deferReroutes = false;
    execTaskOrder = -1;  // (the main thread also runs tasks)
    execTaskLane = -1;
}

// partial occupants of every lane, ordered by position on the lane, then by vehicle index. The vehicles
// append themselves in parallel (one spin lock per lane), then each touched lane sorts its list.
void Sim::rebuildPartials() {
    for (int l : partialLanes) {
        lanes[l].partials.clear();
    }
    partialLanes.clear();
    if (laneLock.size() != lanes.size()) {
        laneLock = std::vector<std::atomic_flag>(lanes.size());
        laneTouched.assign(lanes.size(), 0);
    }
    touchedOfChunk.resize(pool.size() * 8 + 1);
    const int nChunks = (int)touchedOfChunk.size();
    pool.setTag(ThreadPool::T_PARTIALS);
    pool.parallel_for(nChunks, [&](int c) {
        std::vector<int>& mine = touchedOfChunk[c];
        mine.clear();
        const size_t from = running.size() * c / nChunks;
        const size_t to = running.size() * (c + 1) / nChunks;
        for (size_t i = from; i < to; ++i) {
            const int v = running[i];
            const Kin& k = kin[v];
            auto add = [&](int l) {
                while (laneLock[l].test_and_set(std::memory_order_acquire)) {
                }
                lanes[l].partials.push_back(v);
                if (!laneTouched[l]) {
                    laneTouched[l] = 1;
                    mine.push_back(l);
                }
                laneLock[l].clear(std::memory_order_release);
            };
            for (int j = 0; j < k.nFurther; ++j) {
                add(k.further[j]);
            }
            if (k.shadowLane >= 0) {
                // MSAbstractLaneChangeModel::updateShadowLane: the shadow and its further lanes are partially occupied
                add(k.shadowLane);
                for (int j = 0; j < k.nShadowFurther; ++j) {
                    add(k.shadowFurther[j]);
                }
            }
        }
    }, 1);
    for (const std::vector<int>& mine : touchedOfChunk) {
        partialLanes.insert(partialLanes.end(), mine.begin(), mine.end());
    }
    pool.parallel_for((int)partialLanes.size(), [&](int i) {
        const int l = partialLanes[i];
        laneTouched[l] = 0;
        std::vector<int>& p = lanes[l].partials;
        if (p.size() > 1) {
            if (latRes > 0) {
                // MSLane::vehicle_natural_position_sorter
                std::sort(p.begin(), p.end(), [&](int a, int b) {
                    const double pa = backPositionOnLane(a, l);
                    const double pb = backPositionOnLane(b, l);
                    return pa != pb ? pa < pb : kin[a].posLat != kin[b].posLat ? kin[a].posLat < kin[b].posLat : a < b;
                });
            } else {
                std::sort(p.begin(), p.end(), [&](int a, int b) {
                    const double pa = positionOnLane(a, l);
                    const double pb = positionOnLane(b, l);
                    return pa != pb ? pa < pb : a < b;
                });
            }
        }
    }, 64);
}

void Sim::integrate() {
    // stop and detector events in lane order
    std::vector<DetEvent> det;
    for (size_t i = 0; i < activeLanes.size(); ++i) {
        det.insert(det.end(), laneDet[i].begin(), laneDet[i].end());
    }
    applyDetEvents(det);
    if (edFile != nullptr) {
        for (size_t i = 0; i < activeLanes.size(); ++i) {
            for (const EdSample& e : laneEd[i]) {
                edSampled[e.lane] += e.sampled;
                edDist[e.lane] += e.dist;
                edWait[e.lane] += e.wait;
            }
        }
    }
    std::vector<StopEvent> stopEv;
    for (size_t i = 0; i < activeLanes.size(); ++i) {
        stopEv.insert(stopEv.end(), laneStop[i].begin(), laneStop[i].end());
    }
    std::stable_sort(stopEv.begin(), stopEv.end(), [](const StopEvent& a, const StopEvent& b) { return a.veh < b.veh; });
    for (const StopEvent& ev : stopEv) {
        if (ev.enter) {
            busStopEnter(ev.busStop, ev.veh, ev.pos, veh[ev.veh].stops.empty() ? false : veh[ev.veh].stops.front().parking, ev.length);
        } else {
            busStopLeave(ev.busStop, ev.veh);
        }
    }
    // removals (MSLane::updateLengthSum): lane-local sums and lists in parallel, the vehicles leaving
    // the network (arrivals, parking, teleports) and the moves to other lanes in lane order
    pool.setTag(ThreadPool::T_INTEGRATE);
    pool.parallel_for((int)activeLanes.size(), [&](int i) {
        LaneRT& L = lanes[activeLanes[i]];
        double removed = 0, removedNetto = 0;
        auto remove = [&](int v) {
            removed += vt(v).lengthWithGap();
            removedNetto += vt(v).length;
        };
        for (int v : L.outbox) {
            remove(v);
        }
        for (int v : L.arrived) {
            remove(v);
        }
        for (int v : L.parked) {
            remove(v);
        }
        if (L.teleportVeh >= 0) {
            remove(L.teleportVeh);
            L.stay.erase(std::find(L.stay.begin(), L.stay.end(), L.teleportVeh));
        }
        L.vehs.swap(L.stay);
        L.brutto -= removed;
        L.netto -= removedNetto;
        if (L.vehs.empty()) {
            L.brutto = 0;
            L.netto = 0;
        }
        int emergency = 0;
        for (int v : L.vehs) {
            emergency += veh[v].emergencyStop ? 1 : 0;
        }
        L.nEmergency = emergency;
    }, 16);
    std::vector<int> incomingLanes;
    for (int li = 0; li < (int)activeLanes.size(); ++li) {
        const int lane = activeLanes[li];
        LaneRT& L = lanes[lane];
        if (li < (int)laneAlight.size() && !laneAlight[li].empty()) {
            applyAlightings(laneAlight[li]);  // the persons that left a vehicle of this lane continue
            laneAlight[li].clear();
        }
        nEmergencyStops += L.nEmergency;
        for (int v : L.outbox) {
            if (edFile != nullptr) {
                // every normal edge entered in this move, also those crossed within the step
                for (int i = 0; i < veh[v].nEnteredNormal; ++i) {
                    edEntered[veh[v].enteredNormal[i]]++;
                }
            }
            std::vector<int>& in = lanes[kin[v].lane].incoming;
            if (in.empty()) {
                incomingLanes.push_back(kin[v].lane);
            }
            in.push_back(v);
            markEdgeDelayed(kin[v].lane);
        }
        for (size_t ai = 0; ai < L.arrived.size(); ++ai) {
            const int v = L.arrived[ai];
            if (edFile != nullptr) {
                // lanes entered in the arrival step count as entered as well
                for (int i = 0; i < veh[v].nEnteredNormal; ++i) {
                    edEntered[veh[v].enteredNormal[i]]++;
                }
            }
            Veh& x = veh[v];
            x.status = VehStatus::ARRIVED;
            x.arrivalTime = STEPS2TIME(now);
            const int arrivalLane = kin[v].lane;
            removeFromNet(v);
            writeTripinfo(v);
            // (the riders leave while the lane still holds the vehicle: MSVehicle::onRemovalFromNet)
            alightLane = arrivalLane;
            alightLaneVehicles = ai < L.arrivedLaneVehicles.size() ? L.arrivedLaneVehicles[ai] : 0;
            ridersVehicleArrived(v);
            alightLane = -1;
            releaseVehicle(v);
            nArrived++;
        }
        for (int v : L.parked) {
            transferAdd(v, !veh[v].jumping);
        }
        if (L.teleportVeh >= 0) {
            const int v = L.teleportVeh;
            if (L.teleportReason == 'w') {
                nTeleWrongLane++;
            } else if (L.teleportReason == 'y') {
                nTeleYield++;
            } else {
                nTeleJam++;
            }
            if (opt.removeGridlocked) {
                // --time-to-teleport.remove: onRemovalFromNet(NOTIFICATION_TELEPORT_ARRIVED)
                removeFromNet(v, DWR_TELEPORT_ARRIVED);
                veh[v].lf.clear();
                veh[v].status = VehStatus::ARRIVED;
                veh[v].arrivalTime = STEPS2TIME(now);
                writeTripinfo(v);
                ridersVehicleArrived(v);
                releaseVehicle(v);
                nArrived++;
            } else {
                transferAdd(v, false);
            }
        }
    }
    // MSLane::integrateNewVehicles, lanes in id order (only SUMO's active-lane list depends on it)
    if (trackActiveList) {
        std::sort(incomingLanes.begin(), incomingLanes.end(), [&](int a, int b) { return net.lanes[a].id < net.lanes[b].id; });
        activeRemoveEmpty();
    }
    if (trackActiveList) {
        for (int lane : incomingLanes) {
            if (lanes[lane].vehs.empty()) {
                activeAdd(lane);
            }
        }
    }
    // each lane takes its new vehicles behind its own (sorted by position, then vehicle index: the order
    // of SUMO's vehicle buffer does not matter)
    pool.parallel_for((int)incomingLanes.size(), [&](int i) {
        LaneRT& L = lanes[incomingLanes[i]];
        std::vector<int>& in = L.incoming;
        std::sort(in.begin(), in.end(), [&](int a, int b) {
            return kin[a].pos != kin[b].pos ? kin[a].pos < kin[b].pos : a > b;
        });
        int emergency = 0;
        for (auto it = in.rbegin(); it != in.rend(); ++it) {  // the summation order of SUMO's buffer
            L.brutto += vt(*it).lengthWithGap();
            L.netto += vt(*it).length;
            emergency += veh[*it].emergencyStop ? 1 : 0;
        }
        in.insert(in.end(), L.vehs.begin(), L.vehs.end());
        L.vehs.swap(in);
        in.clear();
        L.nEmergency = emergency;
    }, 16);
    if (latRes > 0) {
        // MSLane::integrateNewVehicles of all lanes that moved (MSLane::executeMovements in sublane mode):
        // vehicle_natural_position_sorter (vehicles may pass each other within a lane)
        std::vector<int> toSort(activeLanes.begin(), activeLanes.end());
        toSort.insert(toSort.end(), incomingLanes.begin(), incomingLanes.end());
        std::sort(toSort.begin(), toSort.end());
        toSort.erase(std::unique(toSort.begin(), toSort.end()), toSort.end());
        pool.parallel_for((int)toSort.size(), [&](int i) {
            const int lane = toSort[i];
            std::vector<int>& vs = lanes[lane].vehs;
            std::stable_sort(vs.begin(), vs.end(), [&](int a, int b) {
                const double pa = backPositionOnLane(a, lane);
                const double pb = backPositionOnLane(b, lane);
                return pa != pb ? pa < pb : kin[a].posLat < kin[b].posLat;
            });
        }, 16);
    }
    for (int lane : incomingLanes) {
        nEmergencyStops += lanes[lane].nEmergency;
    }
    compactRunning();
    rebuildPartials();
}

void Sim::activePatch() {
    std::sort(changedState.begin(), changedState.end());
    changedState.erase(std::unique(changedState.begin(), changedState.end()), changedState.end());
    for (int l : changedState) {
        if (!amActive[l] && !lanes[l].vehs.empty()) {
            activeAdd(l);
        }
    }
    changedState.clear();
}

void Sim::activeRemoveEmpty() {
    std::vector<int> keep;
    for (int l : activeList) {
        if (lanes[l].vehs.empty()) {
            amActive[l] = 0;
        } else {
            keep.push_back(l);
        }
    }
    activeList.swap(keep);
}

void Sim::activeAdd(int l) {
    if (amActive[l]) {
        return;
    }
    amActive[l] = 1;
    if (net.edges[net.lanes[l].edge].hasLaneChanger) {
        activeList.insert(activeList.begin(), l);
    } else {
        activeList.push_back(l);
    }
}

void Sim::updateActiveLanes() {
    collectOccupiedLanes(activeLanes);
}

// the lanes with vehicles (the lanes of the running vehicles) in id order. Many vehicles per lane: a
// parallel scan of the lane lists; few: the lanes of the running vehicles, sorted (same result)
void Sim::collectOccupiedLanes(std::vector<int>& out) {
    const int nLanes = (int)lanes.size();
    if ((long long)nLanes > 4LL * pool.size() * (long long)running.size()) {
        out.clear();
        for (int v : running) {
            const int l = kin[v].lane;
            if (l >= 0 && !laneActive[l] && !lanes[l].vehs.empty()) {
                laneActive[l] = 1;
                out.push_back(l);
            }
        }
        for (int l : out) {
            laneActive[l] = 0;
        }
        std::sort(out.begin(), out.end());
        return;
    }
    const int nChunks = std::max(1, std::min(nLanes / 512, pool.size() * 8));
    scanChunks.resize(nChunks);
    pool.setTag(ThreadPool::T_OTHER);
    pool.parallel_for(nChunks, [&](int c) {
        std::vector<int>& mine = scanChunks[c];
        mine.clear();
        const int to = (int)((long long)nLanes * (c + 1) / nChunks);
        for (int l = (int)((long long)nLanes * c / nChunks); l < to; ++l) {
            if (!lanes[l].vehs.empty()) {
                mine.push_back(l);
            }
        }
    }, 1);
    out.clear();
    for (const std::vector<int>& mine : scanChunks) {
        out.insert(out.end(), mine.begin(), mine.end());
    }
}

// drops the vehicles that left the network from the running list (order kept)
void Sim::compactRunning() {
    const int n = (int)running.size();
    const int nChunks = std::max(1, std::min(n / 1024, pool.size() * 8));
    scanChunks.resize(nChunks);
    pool.setTag(ThreadPool::T_OTHER);
    pool.parallel_for(nChunks, [&](int c) {
        std::vector<int>& mine = scanChunks[c];
        mine.clear();
        const int to = (int)((long long)n * (c + 1) / nChunks);
        for (int i = (int)((long long)n * c / nChunks); i < to; ++i) {
            if (veh[running[i]].status == VehStatus::RUNNING) {
                mine.push_back(running[i]);
            }
        }
    }, 1);
    running.clear();
    for (const std::vector<int>& mine : scanChunks) {
        running.insert(running.end(), mine.begin(), mine.end());
    }
}

void Sim::stepOnce() {
    double t0 = nowSec();
    while (nextScheduledEdit < scheduledEdits.size() && scheduledEdits[nextScheduledEdit].first <= now) {
        pendingEdits.push_back(scheduledEdits[nextScheduledEdit++].second);
    }
    applyEdits();
    tlsStep();
    if (!accessEvents.empty()) {
        processAccessEnds(now);  // MSPersonStage_Access::ProceedCmd: overdue begin-of-step events come first
    }
    if (pedNonInteracting) {
        niStep();  // MSPModel_NonInteracting::MoveToNextEdge, begin-of-step events
    } else {
        movePedestrians();  // MSPModel_Striping::MovePedestrians, a begin-of-step event after the traffic lights
    }
    if (!accessEvents.empty()) {
        processAccessEnds(now + DELTA_T);  // (events within the step: after the pedestrians' move at the step time)
    }
    periodicReroutes();  // MSDevice_Routing, begin-of-step events
    if (railActive) {
        railBeginStep();  // MSRailSignalControl::updateSignals
    }
    if (trackActiveList) {
        activePatch();
        activeRemoveEmpty();
    }
    double t1 = nowSec();
    updateActiveLanes();
    if (hasParkingAreas()) {
        paSnapshot();
    }
    planMovements();
    double t2 = nowSec();
    setJunctionApproaches();
    if (railActive) {
        railApproaches();
    }
    double t3 = nowSec();
    executeMovements();
    applyDeferredReroutes();
    if (railActive) {
        applyDriveWayEvents(dwEvents);
        processRailNewRoutes();  // rerouters
        // (in SUMO's lane order of executeMovements: the first lane's train starts the reported circle)
        std::vector<int> dlOrder;
        bool anyDeadlock = false;
        for (const std::vector<RailDeadlockFound>& r : railResolvedDeadlocks) {
            anyDeadlock |= !r.empty();
        }
        if (anyDeadlock && trackActiveList) {
            std::unordered_map<int, int> idx;
            for (int i = 0; i < (int)activeLanes.size() && i < (int)railResolvedDeadlocks.size(); ++i) {
                idx[activeLanes[i]] = i;
            }
            for (int lane : activeList) {
                auto it = idx.find(lane);
                if (it != idx.end()) {
                    dlOrder.push_back(it->second);
                    idx.erase(it);
                }
            }
            for (int i = 0; i < (int)railResolvedDeadlocks.size(); ++i) {
                if (std::find(dlOrder.begin(), dlOrder.end(), i) == dlOrder.end()) {
                    dlOrder.push_back(i);
                }
            }
        } else {
            for (int i = 0; i < (int)railResolvedDeadlocks.size(); ++i) {
                dlOrder.push_back(i);
            }
        }
        for (int task : dlOrder) {
            std::vector<RailDeadlockFound>& r = railResolvedDeadlocks[task];
            for (const RailDeadlockFound& d : r) {
                if (railWrittenDeadlocks.insert(d.seen).second) {
                    if (d.resolved >= 0) {
                        railConstraints[d.resolved].active = false;
                    }
                    if (opt.deadlockOutput) {
                        writeDeadlock(d);
                    }
                }
            }
            r.clear();
        }
    }
    double t4 = nowSec();
    integrate();
    detectCollisions("move");  // MSNet::STAGE_MOVEMENTS
    double t5 = nowSec();
    changeLanes();  // keeps the partial occupants up to date (updatePartials)
    detectCollisions("laneChange");  // MSNet::STAGE_LANECHANGE
    personCheckWaiting();
    double t6 = nowSec();
    railWaitRelations.clear();  // (those of the insertion are kept for the next step)
    insertVehicles();
    detectCollisions("insertion");  // MSNet::STAGE_INSERTIONS
    checkTransfers();
    if (hasParkingAreas()) {
        paEndStep();  // MSParkingArea::updateOccupancy (end-of-step events)
    }
    rebuildPartials();
    adaptEdgeWeights();  // MSRoutingEngine::adaptEdgeEfforts, an end-of-step event
    double t7 = nowSec();
    if (fcdFile != nullptr) {
        writeFcd();
    }
    if (pfcdFile != nullptr) {
        writePersonFcd();
    }
    if (lcFile != nullptr) {
        flushLaneChanges();
    }
    const Time next = now + DELTA_T;
    if (edFile != nullptr && next % TIME2STEPS(opt.edgedataPeriod) == 0) {
        writeEdgeData(STEPS2TIME(next) - opt.edgedataPeriod, STEPS2TIME(next));
        edLastEnd = next;
    }
    if (sumFile != nullptr && now % TIME2STEPS(opt.summaryPeriod) == 0) {
        writeSummaryRow();
    }
    tTLS += t1 - t0;
    tPlan += t2 - t1;
    tApproach += t3 - t2;
    tExec += t4 - t3;
    tIntegrate += t5 - t4;
    tLC += t6 - t5;
    tInsert += t7 - t6;
    tOther += nowSec() - t7;
}

// output files: a missing output directory is an error, not a crash
static FILE* openOutputFile(const std::string& path) {
    FILE* f = std::fopen(path.c_str(), "w");
    if (f == nullptr) {
        throw std::runtime_error("cannot write " + path);
    }
    return f;
}

// a stdio stream writing through zlib (fast compression: the outputs are written by the main thread)
#ifdef __APPLE__
static int gzStreamWrite(void* cookie, const char* data, int n) {
    if (n <= 0) {
        return 0;
    }
    const int w = gzwrite(static_cast<gzFile>(cookie), data, (unsigned)n);
    return w > 0 ? w : -1;
}
static int gzStreamClose(void* cookie) {
    return gzclose(static_cast<gzFile>(cookie)) == Z_OK ? 0 : -1;
}
#else
static ssize_t gzStreamWrite(void* cookie, const char* data, size_t n) {
    if (n == 0) {
        return 0;
    }
    const int w = gzwrite(static_cast<gzFile>(cookie), data, (unsigned)n);
    return w > 0 ? w : -1;
}
static int gzStreamClose(void* cookie) {
    return gzclose(static_cast<gzFile>(cookie)) == Z_OK ? 0 : -1;
}
#endif

static FILE* openGzipOutputFile(const std::string& path) {
    gzFile gz = gzopen(path.c_str(), "wb2");
    if (gz == nullptr) {
        throw std::runtime_error("cannot write " + path);
    }
    gzbuffer(gz, 1 << 18);
#ifdef __APPLE__
    FILE* f = funopen(gz, nullptr, gzStreamWrite, nullptr, gzStreamClose);
#else
    cookie_io_functions_t io = {nullptr, gzStreamWrite, nullptr, gzStreamClose};
    FILE* f = fopencookie(gz, "w", io);
#endif
    if (f == nullptr) {
        gzclose(gz);
        throw std::runtime_error("cannot write " + path);
    }
    std::setvbuf(f, nullptr, _IOFBF, 1 << 16);
    return f;
}

FILE* Sim::openOutput(const std::string& name, bool compressible) const {
    if (compressible && opt.gzip) {
        return openGzipOutputFile(opt.outPrefix + name + ".gz");
    }
    return openOutputFile(opt.outPrefix + name);
}

// the lane changes of the step (they are not collected until the end: memory)
void Sim::flushLaneChanges() {
    for (const LaneChangeEvent& e : lcEvents) {
        std::fprintf(lcFile, "%.2f,%s,%s,%s,%.2f,%.2f,%d\n", e.time, veh[e.veh].id.c_str(), net.lanes[e.from].id.c_str(),
                     net.lanes[e.to].id.c_str(), e.pos, e.speed, e.reason);
    }
    nLaneChangesTotal += lcEvents.size();
    lcEvents.clear();
}

// MSMeanData_Net, simplified: a vehicle counts for the lane its front is on
// after the move (whole steps), entering counts every normal lane entered
void Sim::writeEdgeData(double begin, double end) {
    for (int e = 0; e < (int)net.edges.size(); ++e) {
        const Edge& edge = net.edges[e];
        if (edge.internal) {
            continue;
        }
        double sampled = 0, dist = 0, wait = 0;
        int entered = 0;
        for (int l : edge.lanes) {
            sampled += edSampled[l];
            dist += edDist[l];
            wait += edWait[l];
            entered += edEntered[l];
            edSampled[l] = edDist[l] = edWait[l] = 0;
            edEntered[l] = 0;
        }
        if (sampled > 0 || entered > 0) {
            std::fprintf(edFile, "%.0f,%.0f,%s,%d,%.4f,%.4f,%.4f\n", begin, end, edge.id.c_str(), entered, sampled, dist, wait);
        }
    }
}

// MSNet::writeSummaryOutput
void Sim::writeSummaryRow() {
    int halting = 0, moving = 0;
    double speedSum = 0;
    for (int v : running) {
        if (veh[v].status != VehStatus::RUNNING) {
            continue;
        }
        const double speed = kin[v].speed;
        halting += speed < HALTING_SPEED;
        if (!isStopped(v)) {
            moving++;
            speedSum += speed;
        }
    }
    std::fprintf(sumFile, "%.0f,%d,%zu,%zu,%d,%d,%d,%.4f\n", STEPS2TIME(now), nInserted, running.size() + transfer.size(),
                 pending.size(), nArrived, nTeleported, halting, moving > 0 ? speedSum / moving : -1.0);
}

// MSLane::getMaximumBrakeDist for all lanes
void Sim::updateMaxBrakeDist() {
    for (Lane& l : net.lanes) {
        const double maxSpeed = l.speed * maxSpeedFactor;
        l.maxBrakeDist = std::min(maxSpeed * maxSpeed * 0.5 / minDecel + maxMinGap, 1000.0);
    }
}

// MSVehicleControl::vehicleDeparted (first insertion only)
void Sim::vehicleDeparted(int v) {
    const VType& t = vt(v);
    const double sf = std::max(maxSpeedFactor, veh[v].speedFactor);
    const double gap = std::max(maxMinGap, t.minGap);
    const double dec = std::min(minDecel, t.decel);
    if (sf != maxSpeedFactor || gap != maxMinGap || dec != minDecel) {
        maxSpeedFactor = sf;
        maxMinGap = gap;
        minDecel = dec;
        updateMaxBrakeDist();
    }
}

void Sim::writeFcd() {
    // like SUMO's fcd-output, parking vehicles are written at their stop (speed 0 after the first parking step)
    std::vector<int> vs(running.begin(), running.end());
    for (int v : transfer) {
        if (veh[v].status == VehStatus::PARKING) {
            vs.push_back(v);
        }
    }
    std::sort(vs.begin(), vs.end(), [&](int a, int b) { return veh[a].id < veh[b].id; });
    static const int precision = getenv("PSIM2_FCD_PRECISION") != nullptr ? std::atoi(getenv("PSIM2_FCD_PRECISION")) : 2;
    for (int v : vs) {
        if (sublane) {
            std::fprintf(fcdFile, "%.2f,%s,%s,%.*f,%.*f,%.*f\n", STEPS2TIME(now), veh[v].id.c_str(), net.lanes[kin[v].lane].id.c_str(),
                         precision, kin[v].pos, precision, kin[v].speed, precision, kin[v].posLat);
        } else {
            std::fprintf(fcdFile, "%.2f,%s,%s,%.*f,%.*f\n", STEPS2TIME(now), veh[v].id.c_str(), net.lanes[kin[v].lane].id.c_str(),
                         precision, kin[v].pos, precision, kin[v].speed);
        }
    }
}

// time2string of SUMO (output precision 2): a time in seconds as milliseconds (TIME2STEPS), rounded half up
static std::string time2string(double seconds) {
    long long ms = (long long)(seconds * 1000. + (seconds >= 0 ? 0.5 : -0.5));
    const bool neg = ms < 0;
    ms = (std::llabs(ms) + 5) / 10;
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%s%lld.%02lld", neg ? "-" : "", ms / 100, ms % 100);
    return buf;
}

// unfinished: a vehicle still on the way at the end (--tripinfo-output.write-unfinished, arrival -1)
// an arrived (or removed) vehicle keeps only what the outputs at the end need (memory of long simulations)
void Sim::releaseVehicle(int v) {
    Veh& x = veh[v];
    std::vector<DriveItem>().swap(x.lf);
    x.nextDriveItem = 0;
    std::vector<std::vector<LaneQ>>().swap(x.bestLanes);
    x.lastBestLanesEdge = x.lastBestLanesInternalLane = -1;
    std::vector<LCAdvice>().swap(x.lc.advices);
    std::vector<Stop>().swap(x.stops);
    if (x.extra.p != nullptr) {
        if (x.extra.p->stopLog.empty()) {
            x.extra.p.reset();
        } else {
            std::vector<StopRecord> log = std::move(x.extra.p->stopLog);
            *x.extra.p = VehExtra();
            x.extra.p->stopLog = std::move(log);
        }
    }
}

void Sim::writeTripinfo(int v, bool unfinished) {
    const Veh& x = veh[v];
    if (!x.tripinfoDevice) {
        return;
    }
    char buf[512];
    // MSDevice_Tripinfo::generateOutput: the duration up to the simulation end, the route length up to the position
    const double end = STEPS2TIME(now + DELTA_T);
    const double duration = unfinished ? end - x.departTime : x.arrivalTime - x.departTime;
    const double routeLength = unfinished ? x.routeLength + std::max(0.0, kin[v].pos) : x.routeLength;
    // a stop going on at the end: SUMO also counts the step in which it was reached (notifyMove) and, while
    // parking, the time up to the simulation end (updateParkingStopTime)
    const double stopTime = x.stopTime + (!unfinished || !isStopped(v) ? 0. : (x.status == VehStatus::PARKING ? 2 * TS : TS));
    std::snprintf(buf, sizeof(buf), "%s,%s,%s,%d,%.2f,%s,%s,%.2f,%s,%s,%d,%d,%d,%s,%d\n", x.id.c_str(), time2string(x.departTime).c_str(),
                  time2string(STEPS2TIME(x.depart) == x.departTime ? 0.0 : x.departTime - STEPS2TIME(x.depart)).c_str(),
                  x.departLaneActual, x.departSpeedActual, unfinished ? "-1.00" : time2string(x.arrivalTime).c_str(),
                  time2string(duration).c_str(), routeLength, time2string(x.timeLoss).c_str(),
                  time2string(x.tripWaiting).c_str(), x.waitingCount, x.nLaneChanges, x.nTeleports, time2string(stopTime).c_str(),
                  x.nReroutes);
    tripRows.push_back(buf);
}

void Sim::run() {
    if (opt.fcd) {
        fcdFile = openOutput("fcd.csv", true);
        std::fprintf(fcdFile, sublane ? "time,id,lane,pos,speed,posLat\n" : "time,id,lane,pos,speed\n");
        if (!persons.empty()) {
            pfcdFile = openOutput("personfcd.csv", true);
            std::fprintf(pfcdFile, "time,id,edge,pos,x,y,angle,speed\n");
        }
    }
    if (opt.lcOutput) {
        lcFile = openOutput("lanechange.csv", true);
        std::fprintf(lcFile, "time,id,from,to,pos,speed,reason\n");
    }
    laneActive.assign(net.lanes.size(), 0);
    amActive.assign(net.lanes.size(), 0);
    if (opt.edgedataPeriod > 0) {
        edFile = openOutput("edgedata.csv");
        std::fprintf(edFile, "begin,end,edge,entered,sampledSeconds,travelledDistance,waitingTime\n");
        edSampled.assign(net.lanes.size(), 0);
        edDist.assign(net.lanes.size(), 0);
        edWait.assign(net.lanes.size(), 0);
        edEntered.assign(net.lanes.size(), 0);
    }
    if (opt.summaryPeriod > 0) {
        sumFile = openOutput("summary.csv");
        std::fprintf(sumFile, "time,inserted,running,waiting,arrived,teleports,halting,meanSpeed\n");
    }
    const double wall0 = nowSec();
    size_t maxRunning = 0;
    double vehSteps = 0;
    while (true) {
        stepOnce();
        maxRunning = std::max(maxRunning, running.size());
        vehSteps += running.size();
        // MSNet::simulationState: no active vehicles (those waiting for persons do not count), no persons
        // other than those waiting for a vehicle
        const bool noVehicles = nextToLoad >= veh.size() - (size_t)nTriggeredDepart
                                && pending.size() + running.size() + transfer.size() == (size_t)nVehWaiting.load();
        // MSTransportableControl::hasNonWaiting: riding persons do not count, walking ones do
        const bool noPersons = waiting4Departure.empty() && nWalking == 0 && waitingUntil.empty() && !haveNewWaiting
                               && accessEvents.empty();
        const bool done = noVehicles && (personsDone() || noPersons);
        if (done && !personsDone()) {
            abortAnyWaitingForVehicle();
        }
        // MSNet::simulationState: the step at time 'end' is not executed any more
        if (done || (opt.end >= 0 && now + DELTA_T >= TIME2STEPS(opt.end))) {
            break;
        }
        now += DELTA_T;
        ++step;
    }
    const double wall = nowSec() - wall0;
    if (fcdFile != nullptr) {
        std::fclose(fcdFile);
    }
    if (pfcdFile != nullptr) {
        std::fclose(pfcdFile);
    }
    if (edFile != nullptr) {
        // MSMeanData at the end of the simulation: the last interval ends with the period or --end
        if (edLastEnd <= now) {
            Time end = edLastEnd + TIME2STEPS(opt.edgedataPeriod);
            if (opt.end >= 0) {
                end = std::min(end, TIME2STEPS(opt.end));
            }
            writeEdgeData(STEPS2TIME(edLastEnd), STEPS2TIME(end));
        }
        std::fclose(edFile);
    }
    if (sumFile != nullptr) {
        std::fclose(sumFile);
    }
    writeOutputs();
    FILE* f = openOutput("summary.txt");
    std::fprintf(f, "steps %d\nsimEnd %.3f\nwall %.6f\nthreads %d\nvehicles %zu\ninserted %d\narrived %d\nrunning %zu\n"
                 "waiting %zu\nteleported %d\nteleJam %d\nteleYield %d\nteleWrongLane %d\ncollisions %d\nemergencyStops %d\n"
                 "maxRunning %zu\nvehSteps %.0f\nlaneChanges %zu\nlcLevels %zu\nlcColors %zu\nlcEdges %zu\nlcStages %lld\n"
                 "tTLS %.6f\ntPlan %.6f\ntApproach %.6f\ntExec %.6f\ntIntegrate %.6f\ntLC %.6f\ntInsert %.6f\ntOther %.6f\n",
                 step + 1, STEPS2TIME(now), wall, pool.size(), veh.size(), nInserted, nArrived, running.size() + transfer.size(),
                 pending.size() + (veh.size() - nextToLoad), nTeleported, nTeleJam, nTeleYield, nTeleWrongLane, nCollisions, nEmergencyStops,
                 maxRunning, vehSteps, (size_t)nLaneChangesTotal, edgesByLevel.size(), edgesByColor.size(), lcEdges.size(), nLCStages, tTLS, tPlan, tApproach, tExec, tIntegrate, tLC, tInsert, tOther);
    std::fprintf(f, "tLcSched %.6f\ntLcSerial %.6f\ntLcPartials %.6f\nlcPartialRebuilds %lld\nlcStageEdges %lld\n", tLcSched, tLcSerial,
                 tLcPartials, nLcPartialRebuilds, lcStageEdges);
    std::fprintf(f, "lcScheduleConflicts %lld\nmaxSpanEdges %d\nfurtherOverflow %lld\n", nLcScheduleConflicts.load(), lcMaxSpanEdges.load(),
                 nFurtherOverflow);
    if (!railSignals.empty()) {
        std::fprintf(f, "driveWays %zu\nrailSignalUpdates %lld\nrailSignalSpeculative %lld\nrailSignalReevaluated %lld\n", driveWays.size(),
                     nRsUpdates, nRsSpeculative, nRsReevaluated);
    }
    // parallel regions: wall time, working time of all threads, waiting = wall * threads - busy
    for (int t = 0; t < ThreadPool::T_NUM; ++t) {
        const ThreadPool::Stat& st = pool.stat(t);
        if (st.calls + st.serialCalls > 0) {
            std::fprintf(f, "pool.%s calls=%lld serial=%lld wall=%.3f busy=%.3f wait=%.3f\n", ThreadPool::tagName(t), st.calls,
                         st.serialCalls, st.wall, st.busy, st.wall * pool.size() - st.busy);
        }
    }
    std::fclose(f);
}

void Sim::writeOutputs() {
    if (opt.railsignalBlockOutput) {
        writeRailSignalBlocks(false);
    }
    if (opt.railsignalVehicleOutput) {
        writeRailSignalBlocks(true);
    }
    if (opt.deadlockOutput) {
        FILE* dl = openOutput("deadlocks.xml");
        std::fprintf(dl, "<additional>\n");
        for (const std::string& r : deadlockRows) {
            std::fputs(r.c_str(), dl);
        }
        std::fprintf(dl, "</additional>\n");
        std::fclose(dl);
    }
    if (opt.stopOutput) {
        FILE* so = openOutput("stopinfo.csv");
        std::fprintf(so, "id,busStop,lane,pos,parking,started,ended\n");
        for (const Veh& x : veh) {
            for (const StopRecord& r : x.exr().stopLog) {
                std::fprintf(so, "%s,%s,%s,%.2f,%d,%.2f,%.2f\n", x.id.c_str(), r.busStop >= 0 ? busStops[r.busStop].id.c_str() : "",
                             net.lanes[r.lane].id.c_str(), r.pos, (int)r.parking, STEPS2TIME(r.started), STEPS2TIME(r.ended));
            }
        }
        std::fclose(so);
    }
    if (!persons.empty()) {
        // persons still walking at the end (MSTransportableControl::eraseAll with tripinfo-output.write-unfinished)
        FILE* pf = openOutput("personinfo.csv");
        std::fprintf(pf, "id,stage,depart,departPos,arrival,arrivalPos,duration,routeLength,timeLoss,maxSpeed,waitingTime\n");
        std::vector<std::string> prow = personRows;
        std::sort(prow.begin(), prow.end());
        for (const std::string& r : prow) {
            std::fputs(r.c_str(), pf);
        }
        std::fclose(pf);
    }
    if (opt.tripinfoUnfinished) {
        for (int v = 0; v < (int)veh.size(); ++v) {
            const VehStatus st = veh[v].status;
            if (st == VehStatus::RUNNING || st == VehStatus::PARKING || st == VehStatus::TELEPORTING) {
                writeTripinfo(v, true);
            }
        }
    }
    FILE* f = openOutput("tripinfo.csv");
    std::fprintf(f, "id,depart,departDelay,departLane,departSpeed,arrival,duration,routeLength,timeLoss,waitingTime,waitingCount,laneChanges,teleports,stopTime,rerouteNo\n");
    std::sort(tripRows.begin(), tripRows.end());  // in place (no copy of all rows)
    for (const std::string& r : tripRows) {
        std::fputs(r.c_str(), f);
    }
    std::fclose(f);
    {
        FILE* c = openOutput("collisions.csv");
        std::fprintf(c, "time,stage,lane,collider,victim,gap\n");
        for (const std::string& r : collisionRows) {
            std::fputs(r.c_str(), c);
        }
        std::fclose(c);
    }
    if (lcFile != nullptr) {
        flushLaneChanges();
        std::fclose(lcFile);
        lcFile = nullptr;
    }
}

}  // namespace ps
