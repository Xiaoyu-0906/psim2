// Demand: departure / arrival definitions of vehicles and flows, random flows
// generated with psim2's own counter-based random numbers.
// Semantics follow SUMO 1.28 (SUMOVehicleParameter, MSInsertionControl,
// MSBaseVehicle::calculateArrivalParams, MSLane::insertVehicle); the random
// draws are statistically equivalent, not identical to SUMO's.
//
// Scenario lines:
//   VEH id type route depart departLane departSpeed departPos arrivalPos [speedFactor|- [arrivalLane arrivalSpeed]]
//   FLOW id type route begin end exp|prob value number departLane departSpeed departPos arrivalPos arrivalLane arrivalSpeed [speedFactor]
//   STOP ...   (after a VEH: its stop; after a FLOW: a stop of every generated vehicle, until "D<x>" counts from
//               the vehicle's departure)
#include <cmath>
#include <sstream>
#include <stdexcept>

#include "model.hpp"

namespace ps {

// normal distribution capped by resampling (Distribution_Parameterized::sample)
double sampleSpeedFactor(const VType& t, uint64_t seed, int v, uint64_t purpose) {
    if (t.speedFactorDev <= 0) {
        return t.speedFactorMean;
    }
    for (uint64_t i = 0; i < 1000; ++i) {
        const double u1 = std::max(rand01(seed, (uint64_t)v, 2 * i, purpose), 1e-300);
        const double u2 = rand01(seed, (uint64_t)v, 2 * i + 1, purpose);
        const double z = std::sqrt(-2.0 * std::log(u1)) * std::cos(2 * M_PI * u2);
        const double val = t.speedFactorMean + t.speedFactorDev * z;
        if (val >= t.speedFactorMin && val <= t.speedFactorMax) {
            return std::round(val * 10000.) / 10000.;  // gPrecisionRandom
        }
    }
    return t.speedFactorMean;
}

static int parseDepartLane(const std::string& s) {
    if (s == "free") return DL_FREE;
    if (s == "best") return DL_BEST;
    if (s == "first") return DL_FIRST;
    if (s == "best_prob") return DL_BEST_PROB;
    if (s == "random") return DL_RANDOM;
    if (s == "allowed") return DL_ALLOWED;
    return std::stoi(s);
}

static void parseDepartSpeed(const std::string& s, int& mode, double& value) {
    value = 0;
    if (s == "max") mode = DS_MAX;
    else if (s == "desired") mode = DS_DESIRED;
    else if (s == "avg") mode = DS_AVG;
    else if (s == "random") mode = DS_RANDOM;
    else if (s == "speedLimit") mode = DS_LIMIT;
    else if (s == "last") mode = DS_LAST;
    else {
        mode = DS_GIVEN;
        value = std::stod(s);
    }
}

static void parseDepartPos(const std::string& s, int& mode, double& value) {
    value = 0;
    if (s == "base") mode = DP_BASE;
    else if (s == "random") mode = DP_RANDOM;
    else if (s == "random_free") mode = DP_RANDOM_FREE;
    else if (s == "free") mode = DP_FREE;
    else if (s == "last") mode = DP_LAST;
    else if (s == "stop") mode = DP_STOP;
    else if (s == "splitFront") mode = DP_SPLIT_FRONT;
    else {
        mode = DP_GIVEN;
        value = std::stod(s);
    }
}

static void parseArrivalPos(const std::string& s, int& mode, double& value) {
    value = 0;
    if (s == "max") mode = AP_MAX;
    else if (s == "random") mode = AP_RANDOM;
    else if (s == "center") mode = AP_CENTER;
    else {
        mode = AP_GIVEN;
        value = std::stod(s);
    }
}

static void parseArrivalLane(const std::string& s, int& mode, int& lane) {
    lane = -1;
    if (s == "current" || s == "-") mode = AL_CURRENT;
    else if (s == "random") mode = AL_RANDOM;
    else if (s == "first") mode = AL_FIRST;
    else {
        mode = AL_GIVEN;
        lane = std::stoi(s);
    }
}

// fields shared by VEH and FLOW lines: departLane departSpeed departPos arrivalPos arrivalLane arrivalSpeed
// SUMOVehicleParameter::parseInsertionChecks (names separated by spaces or commas)
int parseInsertionChecks(const std::string& s) {
    static const std::pair<const char*, int> names[] = {
        {"none", IC_NONE}, {"collision", IC_COLLISION}, {"leaderGap", IC_LEADER_GAP}, {"followerGap", IC_FOLLOWER_GAP},
        {"junction", IC_JUNCTION}, {"stop", IC_STOP}, {"arrivalSpeed", IC_ARRIVAL_SPEED},
        {"oncomingTrain", IC_ONCOMING_TRAIN}, {"speedLimit", IC_SPEED_LIMIT}, {"pedestrian", IC_PEDESTRIAN},
        {"bidi", IC_BIDI}, {"laneChange", IC_LANECHANGE}, {"all", IC_ALL}};
    int r = 0;
    std::string cur;
    for (char c : s + " ") {
        if (c == ' ' || c == ',') {
            if (!cur.empty()) {
                bool found = false;
                for (const auto& n : names) {
                    if (cur == n.first) {
                        r |= n.second;
                        found = true;
                    }
                }
                if (!found) {
                    throw std::runtime_error("unknown insertion check " + cur);
                }
            }
            cur.clear();
        } else {
            cur += c;
        }
    }
    return r;
}

void parseVehicleDefinitions(Veh& x, const std::string& dl, const std::string& ds, const std::string& dp,
                             const std::string& ap, const std::string& al, const std::string& as) {
    x.departLane = parseDepartLane(dl);
    parseDepartSpeed(ds, x.departSpeedMode, x.departSpeed);
    parseDepartPos(dp, x.departPosMode, x.departPos);
    parseArrivalPos(ap, x.arrivalPosMode, x.arrivalPosGiven);
    parseArrivalLane(al, x.arrivalLaneMode, x.arrivalLane);
    x.arrivalSpeed = (as == "current" || as == "-") ? -1 : std::stod(as);
}

void Sim::addVehicle(Veh x, int type, const std::string& speedFactor) {
    x.vclass = types[type].vclass;
    const int v = (int)veh.size();
    if (speedFactor != "-") {
        x.speedFactor = std::stod(speedFactor);
    } else if (opt.deterministicSpeedFactor) {
        x.speedFactor = types[type].speedFactorMean;
    } else {
        x.speedFactor = sampleSpeedFactor(types[type], opt.seed, v, RP_SPEEDFACTOR);
    }
    veh.push_back(std::move(x));
    Kin kk;
    kk.type = type;
    kin.push_back(kk);
}

// a vehicle type, or one drawn from a vTypeDistribution (MSVehicleControl::getVType with RandomDistributor::get)
int Sim::resolveType(const std::string& id, uint64_t a, uint64_t b) const {
    auto it = typeIdx.find(id);
    if (it != typeIdx.end()) {
        return it->second;
    }
    const std::vector<std::pair<int, double>>& dist = typeDists.at(id);
    double sum = 0;
    for (const auto& d : dist) {
        sum += d.second;
    }
    double prob = rand01(opt.seed, a, b, RP_TYPEDIST) * sum;
    for (const auto& d : dist) {
        if (prob < d.second) {
            return d.first;
        }
        prob -= d.second;
    }
    return dist.back().first;
}

void Sim::parseVehicleLine(const std::vector<std::string>& tok) {
    Veh x;
    x.id = tok[1];
    const int type = resolveType(tok[2], veh.size(), 0);
    x.route = routeIdx.at(tok[3]);
    if (tok[4] == "triggered" || tok[4] == "split") {
        x.departSplit = tok[4] == "split";
        // waits on its first edge until a person boards (sorted behind all other vehicles)
        x.departTriggered = true;
        x.depart = TIME_MAX;
    } else {
        x.depart = TIME2STEPS(std::stod(tok[4]));
    }
    const std::string al = tok.size() > 10 ? tok[10] : "current";
    const std::string as = tok.size() > 11 ? tok[11] : "current";
    parseVehicleDefinitions(x, tok[5], tok[6], tok[7], tok[8], al, as);
    addVehicle(std::move(x), type, tok.size() > 9 ? tok[9] : "-");
    lastFlow = -1;
}

void Sim::parseFlowLine(const std::vector<std::string>& tok) {
    FlowDef f;
    f.id = tok[1];
    f.typeName = tok[2];
    f.type = resolveType(tok[2], flows.size() + (1ULL << 40), 0);  // (the type of the proto: definitions only)
    f.route = routeIdx.at(tok[3]);
    f.begin = TIME2STEPS(std::stod(tok[4]));
    const double end = std::stod(tok[5]);
    f.end = end >= 1e15 ? TIME_MAX : TIME2STEPS(end);
    f.poisson = tok[6] == "exp";
    if (!f.poisson && tok[6] != "prob") {
        throw std::runtime_error("unknown flow kind " + tok[6]);
    }
    f.value = std::stod(tok[7]);
    f.number = std::stoll(tok[8]);
    parseVehicleDefinitions(f.proto, tok[9], tok[10], tok[11], tok[12], tok[13], tok[14]);
    f.proto.route = f.route;
    if (tok.size() > 15) {
        f.speedFactor = tok[15];
    }
    flows.push_back(f);
    lastFlow = (int)flows.size() - 1;
}

// STOP veh busStop|%parkingArea|@lane,startPos,endPos duration until parking
void Sim::addStop(Veh& x, int type, const std::vector<std::string>& tok, Time departDesired) {
    Stop s;
    if (tok[2][0] == '%') {
        s.parkingArea = parkingAreaIdx.at(tok[2].substr(1));
        const ParkingArea& pa = parkingAreas[s.parkingArea];
        s.lane = pa.lane;
        s.startPos = pa.begPos;
        s.endPos = pa.endPos;
        // MSBaseVehicle::addStop: the area forbids access without a valid badge
        bool accepted = pa.badges.empty();
        for (const std::string& b : x.exr().parkingBadges) {
            accepted = accepted || std::find(pa.badges.begin(), pa.badges.end(), b) != pa.badges.end();
        }
        if (!accepted) {
            const std::string msg = "parkingArea '" + pa.id + "'on lane '" + net.lanes[pa.lane].id + "' forbids access because vehicle '"
                                    + x.id + "' does not provide any valid badge.";
            if (!opt.ignoreRouteErrors) {
                throw std::runtime_error(msg);
            }
            std::fprintf(stderr, "Warning: %s\n", msg.c_str());
            return;
        }
    } else if (tok[2][0] == '@') {
        // stop on a lane (SUMOVehicleParserHelper::parseStop: endPos defaults to the lane end,
        // startPos to endPos - 2 * POSITION_EPS, negative positions count from the lane end)
        std::vector<std::string> f;
        std::stringstream ss(tok[2].substr(1));
        std::string w;
        while (std::getline(ss, w, ',')) {
            f.push_back(w);
        }
        s.busStop = -1;
        if (f[0][0] == '=') {
            // stop on an edge: the first lane allowing the vehicle class (MSEdge::getFirstAllowed)
            s.lane = -1;
            for (int l : net.edges[net.edgeIdx.at(f[0].substr(1))].lanes) {
                if ((net.lanes[l].permissions & x.vclass) == x.vclass) {
                    s.lane = l;
                    break;
                }
            }
            if (s.lane < 0) {
                throw std::runtime_error("vehicle " + x.id + " is not allowed to stop on any lane of edge " + f[0].substr(1));
            }
        } else {
            s.lane = net.laneIdx.at(f[0]);
        }
        const double len = net.lanes[s.lane].length;
        s.endPos = f[2] == "-" ? len : std::stod(f[2]);
        if (s.endPos < 0) {
            s.endPos += len;
        }
        s.startPos = f[1] == "-" ? std::max(0.0, s.endPos - 2 * POSITION_EPS) : std::stod(f[1]);
        if (s.startPos < 0) {
            s.startPos += len;
        }
    } else {
        s.busStop = busStopIdx.at(tok[2]);
        const BusStop& b = busStops[s.busStop];
        s.lane = b.lane;
        s.startPos = b.startPos;
        s.endPos = b.endPos;
    }
    s.duration = TIME2STEPS(std::stod(tok[3]));
    if (tok[4] == "-") {
        s.until = -1;
    } else if (tok[4][0] == 'D') {
        s.until = departDesired + TIME2STEPS(std::stod(tok[4].substr(1)));
    } else {
        s.until = TIME2STEPS(std::stod(tok[4]));
    }
    s.parking = tok[5] == "1";
    if (tok.size() > 7) {
        s.speed = std::stod(tok[7]);  // a waypoint
    }
    for (size_t i = 8; i < tok.size(); ++i) {
        // optional key=value attributes
        const size_t eq = tok[i].find('=');
        const std::string key = tok[i].substr(0, eq);
        const std::string value = tok[i].substr(eq + 1);
        if (key == "jump") {
            s.jump = TIME2STEPS(std::stod(value));
        } else if (key == "jumpUntil") {
            s.jumpUntil = TIME2STEPS(std::stod(value));
        } else if (key == "arrival") {
            s.arrival = TIME2STEPS(std::stod(value));
        } else if (key == "priority") {
            s.priority = std::stod(value);
        } else if (key == "skip") {
            s.skipLooped = std::stoi(value);
        } else if (key == "started") {
            s.startedGiven = TIME2STEPS(std::stod(value));
        } else if (key == "flexArrival") {
            s.flexArrival = TIME2STEPS(std::stod(value));
        } else if (key == "posLat") {
            s.posLat = std::stod(value);
        } else if (key == "triggered") {
            s.triggered = value.find('p') != std::string::npos;
            s.containerTriggered = value.find('c') != std::string::npos;
            s.joinTriggered = value.find('j') != std::string::npos;
        } else if (key == "split") {
            s.split = value;
        } else if (key == "join") {
            s.join = value;
            s.joinTriggered = true;  // MSStop::initPars: joinTriggered || join != ""
        } else if (key == "expected") {
            std::stringstream ss(value);
            std::string id;
            while (std::getline(ss, id, ',')) {
                s.awaitedPersons.push_back(id);
            }
            s.triggered = true;  // (expected persons make the stop triggered)
            s.numExpectedPerson = (int)s.awaitedPersons.size();
        } else if (key == "onDemand") {
            s.onDemand = value == "1";
        } else if (key == "extension") {
            s.extension = TIME2STEPS(std::stod(value));
        } else if (key == "expectedContainers") {
            std::stringstream ss(value);
            std::string id;
            while (std::getline(ss, id, ',')) {
                s.awaitedContainers.push_back(id);
            }
            s.containerTriggered = true;
            s.numExpectedContainer = (int)s.awaitedContainers.size();
        } else if (key == "permitted") {
            std::stringstream ss(value);
            std::string id;
            while (std::getline(ss, id, ',')) {
                s.permitted.push_back(id);
            }
        } else {
            throw std::runtime_error("unknown stop attribute " + key);
        }
    }
    if (tok.size() > 6 && tok[6] != "-") {
        s.tripId = tok[6];
        railTripIdLookup[s.tripId] = x.id;  // MSRailSignalConstraint::storeTripId
    }
    s.routeIdx = stopRouteIndex(x, type, routes[x.route], s, x.stops.empty() ? nullptr : &x.stops.back());
    if (s.routeIdx < 0) {
        throw std::runtime_error("stop of vehicle " + x.id + " is not on its route");
    }
    x.stops.push_back(s);
}

// MSBaseVehicle::addStop (vehicle not departed): route index of the stop edge, the next occurrence when
// the stop lies behind the previous stop or behind the departure position on the same edge
int Sim::stopRouteIndex(const Veh& x, int type, const std::vector<int>& r, const Stop& s, const Stop* prevStop) const {
    const int stopEdge = net.lanes[s.lane].edge;
    auto findFrom = [&](int from) {
        for (int i = std::max(0, from); i < (int)r.size(); ++i) {
            if (r[i] == stopEdge) {
                return i;
            }
        }
        return -1;
    };
    int searchStart = x.routeIdx;  // (the departEdge)
    while (searchStart < (int)r.size()) {
        int idx = findFrom(searchStart);
        int prevIdx = x.routeIdx;  // myCurrEdge
        int prevEdge = r[x.routeIdx];  // the vehicle's edge
        double prevPos = 0;       // getPositionOnLane() before the departure
        if (prevStop != nullptr && prevStop->routeIdx >= searchStart) {
            prevIdx = prevStop->routeIdx;
            prevEdge = net.lanes[prevStop->lane].edge;
            prevPos = prevStop->endPos;
            idx = findFrom(prevIdx);
            if (prevIdx == idx && prevEdge == stopEdge && prevPos > s.endPos) {
                idx = findFrom(prevIdx + 1);
            }
        }
        // MSBaseVehicle::addStop: skip occurrences of the stop edge in a looped route
        for (int j = 0; j < s.skipLooped && idx >= 0; ++j) {
            const int next = findFrom(idx + 1);
            if (next < 0) {
                break;
            }
            idx = next;
        }
        if (idx < 0) {
            break;
        }
        const bool tooClose = prevIdx == idx && prevEdge == stopEdge && prevPos > s.endPos + POSITION_EPS;
        if (prevIdx > idx || tooClose) {
            searchStart = idx + 1;
            continue;
        }
        if (idx == 0) {
            double pos = -1;
            if (x.departPosMode == DP_GIVEN) {
                pos = x.departPos < 0 ? x.departPos + net.edges[r[0]].length : x.departPos;
            } else if (x.departPosMode == DP_BASE) {
                pos = std::min(s.endPos, std::min(types[type].length + POSITION_EPS, net.edges[r[0]].length));
            }
            if (pos > s.endPos) {
                searchStart = idx + 1;
                continue;
            }
        }
        return idx;
    }
    return -1;
}

// VOPT veh key=value...: reroute=1 (a trip: routed before the insertion), via=e1,e2, flow=1 (generated
// at its departure)
void Sim::parseVehicleOptions(const std::vector<std::string>& tok) {
    Veh& x = lastFlow >= 0 ? flows[lastFlow].proto : veh.back();
    for (size_t i = 2; i < tok.size(); ++i) {
        const size_t eq = tok[i].find('=');
        const std::string key = tok[i].substr(0, eq);
        const std::string value = tok[i].substr(eq + 1);
        if (key == "reroute") {
            x.forceReroute = value == "1";
        } else if (key == "fullroute") {
            x.ex().fullRoute = routeIdx.at(value);
        } else if (key == "via") {
            std::stringstream ss(value);
            std::string e;
            while (std::getline(ss, e, ',')) {
                // an edge or the connector of a zone ("<taz>-source", "<taz>-sink")
                auto it = net.edgeIdx.find(e);
                if (it != net.edgeIdx.end()) {
                    x.ex().via.push_back(it->second);
                } else {
                    const bool source = e.size() > 7 && e.compare(e.size() - 7, 7, "-source") == 0;
                    const size_t cut = source ? 7 : 5;
                    if (!source && (e.size() <= 5 || e.compare(e.size() - 5, 5, "-sink") != 0)) {
                        throw std::runtime_error("unknown via edge " + e);
                    }
                    x.ex().via.push_back(tazNode(tazIdx.at(e.substr(0, e.size() - cut)), source));
                }
            }
        } else if (key == "flow") {
            x.fromFlow = value == "1";
        } else if (key == "device") {
            x.deviceParam = value == "1" ? 1 : 0;
        } else if (key == "line") {
            x.ex().line = value;
        } else if (key == "rmode") {
            x.routingMode = (int8_t)std::stoi(value);
        } else if (key == "deviceprob") {
            x.deviceProb = std::stod(value);
        } else if (key == "period") {
            x.periodParam = std::stod(value);
        } else if (key == "tripId") {
            x.tripId = value;
        } else if (key == "otprio") {
            x.ex().otPrio = std::stod(value);
        } else if (key == "otmaxdelay") {
            x.ex().otMaxDelay = std::stod(value);
        } else if (key == "crmaxdelay") {
            x.ex().crMaxDelay = std::stod(value);
        } else if (key == "cfignoreids" || key == "cfignoretypes" || key == "jmignoreids" || key == "jmignoretypes") {
            VehExtra& e = x.ex();
            std::vector<std::string>& list = key == "cfignoreids" ? e.cfIgnoreIDs : key == "cfignoretypes" ? e.cfIgnoreTypes
                                             : key == "jmignoreids" ? e.jmIgnoreIDs : e.jmIgnoreTypes;
            std::stringstream ss(value);
            std::string id;
            while (std::getline(ss, id, ',')) {
                list.push_back(id);
            }
        } else if (key == "dposlat") {
            VehExtra& e = x.ex();
            if (value == "random") {
                e.departPosLatProc = DPL_RANDOM;
            } else if (value == "right") {
                e.departPosLatProc = DPL_RIGHT;
            } else if (value == "center") {
                e.departPosLatProc = DPL_CENTER;
            } else if (value == "left") {
                e.departPosLatProc = DPL_LEFT;
            } else if (value == "free") {
                e.departPosLatProc = DPL_FREE;
            } else if (value == "random_free") {
                e.departPosLatProc = DPL_RANDOM_FREE;
            } else {
                e.departPosLatProc = DPL_GIVEN;
                e.departPosLat = std::stod(value);
            }
        } else if (key == "aposlat") {
            VehExtra& e = x.ex();
            if (value == "right") {
                e.arrivalPosLatProc = APL_RIGHT;
            } else if (value == "center") {
                e.arrivalPosLatProc = APL_CENTER;
            } else if (value == "left") {
                e.arrivalPosLatProc = APL_LEFT;
            } else {
                e.arrivalPosLatProc = APL_GIVEN;
                e.arrivalPosLat = std::stod(value);
            }
        } else if (key == "ichecks") {
            x.ex().insertionChecks = parseInsertionChecks(value);  // the vehicle's insertionChecks
        } else if (key == "departEdge") {
            x.routeIdx = std::stoi(value);  // MSBaseVehicle: myCurrEdge = the route begin + departEdge
            x.departEdge = x.routeIdx;
        } else if (key == "fromtaz") {
            x.fromTaz = tazIdx.at(value);
        } else if (key == "totaz") {
            x.toTaz = tazIdx.at(value);
        } else if (key == "tripinfo") {
            x.tripinfoDevice = value == "1";
        } else if (key == "initreplaced") {
            x.initReplaced = value == "1";
        } else if (key == "badges") {
            std::stringstream ss(value);
            std::string b;
            while (std::getline(ss, b, ',')) {
                x.ex().parkingBadges.push_back(b);
            }
        } else if (key.rfind("parking.", 0) == 0) {
            // MSStoppingPlaceRerouter::getWeight: invalid numbers are ignored
            char* end = nullptr;
            const double d = std::strtod(value.c_str(), &end);
            if (end != value.c_str() && *end == '\0') {
                x.ex().parkingParams.push_back({key.substr(8), d});
            } else {
                std::fprintf(stderr, "Warning: Invalid value '%s' for vehicle parameter '%s'\n", value.c_str(), key.c_str());
            }
        } else {
            throw std::runtime_error("unknown vehicle option " + key);
        }
    }
}

void Sim::parseStopLine(const std::vector<std::string>& tok) {
    if (lastFlow >= 0) {
        flows[lastFlow].stops.push_back(tok);
    } else {
        addStop(veh.back(), kin.back().type, tok, veh.back().depart);
    }
}

// MSInsertionControl::determineCandidates / SUMOVehicleParameter::incrementFlow for
// Poisson (period="exp(rate)") and probability flows
void Sim::generateFlows() {
    // the departures of all flows first: the vehicles are allocated at once (peak memory)
    std::vector<std::vector<Time>> flowDeparts(flows.size());
    size_t total = 0;
    for (int fi = 0; fi < (int)flows.size(); ++fi) {
        const FlowDef& f = flows[fi];
        std::vector<Time>& departs = flowDeparts[fi];
        if (f.poisson) {
            // the first vehicle already departs after an exponential offset
            Time offset = TIME2STEPS(randExp(f.value, rand01(opt.seed, (uint64_t)fi, 0, RP_FLOW)));
            for (long long k = 0; (f.number < 0 || k < f.number) && f.begin + offset < f.end; ++k) {
                departs.push_back(f.begin + offset);
                offset += TIME2STEPS(randExp(f.value, rand01(opt.seed, (uint64_t)fi, (uint64_t)k + 1, RP_FLOW)));
            }
        } else {
            // one vehicle per step with probability value * TS
            for (Time t = f.begin; t < f.end && (f.number < 0 || (long long)departs.size() < f.number); t += DELTA_T) {
                if (rand01(opt.seed, (uint64_t)fi, (uint64_t)(t / DELTA_T), RP_FLOW) < f.value * TS) {
                    departs.push_back(t);
                }
                if (f.end == TIME_MAX && t > TIME2STEPS(opt.end >= 0 ? opt.end : 86400)) {
                    break;
                }
            }
        }
        total += departs.size();
    }
    veh.reserve(veh.size() + total);
    kin.reserve(kin.size() + total);
    for (int fi = 0; fi < (int)flows.size(); ++fi) {
        const FlowDef& f = flows[fi];
        const std::vector<Time>& departs = flowDeparts[fi];
        for (size_t k = 0; k < departs.size(); ++k) {
            Veh x = f.proto;
            x.id = f.id + "." + std::to_string(k);
            x.fromFlow = true;
            x.depart = departs[k];
            const int type = resolveType(f.typeName, (uint64_t)fi + (1ULL << 40), (uint64_t)k + 1);
            x.vclass = types[type].vclass;
            for (const std::vector<std::string>& st : f.stops) {
                addStop(x, type, st, x.depart);
            }
            addVehicle(std::move(x), type, f.speedFactor);
        }
    }
}

// MSBaseVehicle::calculateArrivalParams
void Sim::resolveArrival(int v) {
    Veh& x = veh[v];
    const int lastEdge = routes[x.route].back();
    const Edge& e = net.edges[lastEdge];
    const double lastLaneLength = net.lanes[e.lanes[0]].length;
    switch (x.arrivalPosMode) {
        case AP_GIVEN:
            x.arrivalPos = x.arrivalPosGiven < 0 ? x.arrivalPosGiven + lastLaneLength : x.arrivalPosGiven;
            x.arrivalPos = std::max(0.0, std::min(x.arrivalPos, lastLaneLength));
            break;
        case AP_RANDOM:
            x.arrivalPos = rand01(opt.seed, (uint64_t)v, 0, RP_ARRIVAL) * lastLaneLength;
            break;
        case AP_CENTER:
            x.arrivalPos = lastLaneLength / 2.;
            break;
        default:
            x.arrivalPos = lastLaneLength;
            break;
    }
    if (x.arrivalLaneMode == AL_GIVEN) {
        x.arrivalLane = std::min(x.arrivalLane, (int)e.lanes.size() - 1);
    } else if (x.arrivalLaneMode == AL_FIRST || x.arrivalLaneMode == AL_RANDOM) {
        std::vector<int> usable;
        for (int l : e.lanes) {
            if (net.lanes[l].allows(x.vclass)) {
                usable.push_back(net.lanes[l].index);
            }
        }
        if (usable.empty()) {
            x.arrivalLane = 0;
        } else if (x.arrivalLaneMode == AL_FIRST) {
            x.arrivalLane = usable.front();
        } else {
            x.arrivalLane = usable[(size_t)(rand01(opt.seed, (uint64_t)v, 1, RP_ARRIVAL) * usable.size())];
        }
    }
}

// MSLane::lastInsertion: insert behind the last vehicle on the lane (or on the
// following lanes within braking distance)
bool Sim::lastInsertion(int v, int lane, double speed, bool patchSpeed) {
    const VType& t = vt(v);
    double pos = net.lanes[lane].length - POSITION_EPS;
    int leader = lastAnyVehicle(lane);
    double leaderBack = 0;
    if (leader < 0) {
        // leaders on the following lanes (vehicles only, no junction foes)
        kin[v].lane = lane;
        kin[v].pos = pos;
        updateBestLanes(v, false, lane);
        const std::vector<int>& conts = bestContinuation(v);
        const double dist = cfBrakeGap(v, speed);
        double seen = net.lanes[lane].length - pos;
        int cur = lane;
        int view = 1;
        while (leader < 0 && seen <= dist) {
            const int link = succLinkSec(v, view, cur, conts);
            if (link < 0) {
                break;
            }
            const bool internal = net.links[link].via >= 0;
            cur = net.links[link].viaOrLane();
            leader = lastAnyVehicle(cur);
            if (leader >= 0) {
                leaderBack = pos + seen + backPositionOnLane(leader, cur);
                break;
            }
            seen += net.lanes[cur].length;
            if (!internal) {
                view++;
            }
        }
    } else {
        leaderBack = backPositionOnLane(leader, lane);
    }
    if (leader < 0) {
        return isInsertionSuccess(v, lane, speed, pos, patchSpeed, false);
    }
    const double frontGapNeeded = cfSecureGap(v, leader, speed, kin[leader].speed, vt(leader).decel) + t.minGap + POSITION_EPS;
    if (leaderBack >= frontGapNeeded) {
        return isInsertionSuccess(v, lane, speed, std::min(pos, leaderBack - frontGapNeeded), patchSpeed, false);
    }
    return false;
}

}  // namespace ps
