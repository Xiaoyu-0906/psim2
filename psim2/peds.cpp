// Persons (MSTransportable, MSStageWalking, MSStageWaiting, MSTransportableControl) and the
// striping pedestrian model (MSPModel_Striping, MSPModel_Interacting) of SUMO 1.28.
//
// The model moves pedestrians at the start of a step (after the traffic lights, before the
// vehicles). It processes the lanes with pedestrians in lane order, first for pedestrians
// walking forward, then backward; on each lane the pedestrians are handled from the front.
// This version follows SUMO's sequential order (see PEDESTRIANS.md for the parallel plan).
#include <algorithm>
#include <cmath>
#include <map>
#include <set>

#include "model.hpp"

namespace ps {

namespace {
constexpr double DIST_FAR_AWAY = 10000;
constexpr double DIST_BEHIND = 1000;
constexpr double DIST_OVERLAP = -1;
constexpr double LOOKAHEAD_SAMEDIR = 4.0;   // seconds
constexpr double LOOKAHEAD_ONCOMING = 10.0; // seconds
constexpr double LOOKAHEAD_ONCOMING_DIST = 10.0;  // m
constexpr double LATERAL_PENALTY = -1.;
constexpr double OBSTRUCTED_PENALTY = -300000.;
constexpr double INAPPROPRIATE_PENALTY = -20000.;
constexpr double ONCOMING_CONFLICT_PENALTY = -1000.;
constexpr double OBSTRUCTION_THRESHOLD = OBSTRUCTED_PENALTY * 0.5;
constexpr double SQUEEZE = 0.7;
constexpr double MAX_WAIT_TOLERANCE = 120.;
constexpr double LATERAL_SPEED_FACTOR = 0.4;
constexpr double MIN_STARTUP_DIST = 0.4;

PedObstacle noObstacle(int dir, double dist = DIST_FAR_AWAY) {
    PedObstacle o;
    o.xFwd = dir * dist;
    o.xBack = dir * dist;
    return o;
}

PedObstacle explicitObstacle(double x, double speed, int type, double width = 0., int vehicle = -1) {
    PedObstacle o;
    o.xFwd = x + width / 2.;
    o.xBack = x - width / 2.;
    o.speed = speed;
    o.type = type;
    o.vehicle = vehicle;
    return o;
}

// the event buffer of the current parallel lane task (nullptr: apply effects right away)
thread_local std::vector<Sim::PedEvent>* tlPedEvents = nullptr;

bool closer(const PedObstacle& a, const PedObstacle& o, int dir) {
    return dir == PED_FORWARD ? a.xBack <= o.xBack : a.xFwd >= o.xFwd;
}

int toI(const std::string& s) {
    return s == "-" ? -1 : (int)std::strtol(s.c_str(), nullptr, 10);
}

std::vector<std::string> splitBy(const std::string& s, char c) {
    std::vector<std::string> out;
    size_t start = 0;
    while (true) {
        const size_t e = s.find(c, start);
        out.push_back(s.substr(start, e == std::string::npos ? std::string::npos : e - start));
        if (e == std::string::npos) {
            break;
        }
        start = e + 1;
    }
    return out;
}
}  // namespace

// ---------------------------------------------------------------------------
// person loading (MSRouteHandler)
// ---------------------------------------------------------------------------

// PERSON id type depart departPos speedFactor
void Sim::parsePersonLine(const std::vector<std::string>& tok) {
    Person p;
    p.id = tok[1];
    p.type = resolveType(tok[2], persons.size() + (2ULL << 40), 0);
    if (tok[3] == "triggered") {
        // MSRouteHandler::addRideOrTransport: the depart of the vehicle of the first ride (set with that stage)
        p.triggered = true;
        p.depart = 0;
    } else {
        p.depart = TIME2STEPS(std::stod(tok[3]));
    }
    // a numeric departPos or "random"; other definitions (base, ...) mean 0 (SUMOVehicleParameter::departPos)
    std::string dp = tok[4];
    if (dp != "random" && dp != "-") {
        char* end = nullptr;
        std::strtod(dp.c_str(), &end);
        if (end == dp.c_str() || *end != 0) {
            dp = dp == "random_free" ? "random" : "-";
        }
    }
    personDepartPos.push_back(dp);
    const int idx = (int)persons.size();
    if (tok[5] == "-") {
        p.speedFactor = sampleSpeedFactor(types[p.type], opt.seed, idx, RP_PERSON_SPEEDFACTOR);
    } else {
        p.speedFactor = std::stod(tok[5]);
    }
    persons.push_back(p);
}

// persons and person flows. PFLOW id type begin end prob|exp value number departPos speedFactor is followed by
// the stage lines of the flow; probabilistic and Poisson flows are expanded with psim2's random numbers
// (MSRouteHandler::closeTransportableFlow: probability per second, Poisson with a random first departure).
void Sim::loadPersonLines() {
    int nFlows = 0;
    for (size_t i = 0; i < personLines.size(); ++i) {
        const std::vector<std::string>& t = personLines[i];
        // containers (CONTAINER, CFLOW) are transportables like persons that never walk
        auto isHead = [](const std::string& k) { return k == "PERSON" || k == "PFLOW" || k == "CONTAINER" || k == "CFLOW"; };
        if (t[0] == "PERSON" || t[0] == "CONTAINER") {
            parsePersonLine(t);
            persons.back().container = t[0] == "CONTAINER";
        } else if (t[0] == "PFLOW" || t[0] == "CFLOW") {
            std::vector<std::vector<std::string>> stages;
            while (i + 1 < personLines.size() && !isHead(personLines[i + 1][0])) {
                stages.push_back(personLines[++i]);
            }
            const uint64_t flowKey = (uint64_t)nFlows++;
            const Time begin = TIME2STEPS(std::stod(t[3]));
            const double endS = std::stod(t[4]);
            const Time end = endS >= 1e17 ? TIME_MAX : TIME2STEPS(endS);
            const double value = std::stod(t[6]);
            const long long number = std::stoll(t[7]);
            std::vector<Time> departs;
            uint64_t draw = 0;
            if (t[5] == "prob") {
                if (end == TIME_MAX) {
                    throw std::runtime_error("probabilistic personFlow " + t[1] + " must specify end time");
                }
                for (Time d = begin; d < end; d += 1000) {
                    if (rand01(opt.seed, flowKey, draw++, RP_PERSON_FLOW) < value) {
                        departs.push_back(d);
                    }
                }
            } else {
                Time offset = TIME2STEPS(randExp(value, rand01(opt.seed, flowKey, draw++, RP_PERSON_FLOW)));
                while ((number < 0 || (long long)departs.size() < number) && (number >= 0 || begin + offset <= end)) {
                    departs.push_back(begin + offset);
                    offset += TIME2STEPS(randExp(value, rand01(opt.seed, flowKey, draw++, RP_PERSON_FLOW)));
                }
            }
            for (size_t k = 0; k < departs.size(); ++k) {
                const std::string id = t[1] + "." + std::to_string(k);
                parsePersonLine({"PERSON", id, t[2], std::to_string(STEPS2TIME(departs[k])), t[8], t[9]});
                persons.back().container = t[0] == "CFLOW";
                for (std::vector<std::string> st : stages) {
                    st[1] = id;
                    parsePersonStageLine(st);
                }
            }
        } else {
            parsePersonStageLine(t);
        }
    }
    personLines.clear();
}

// PRIDE person from|- to|- busStop|- arrivalPos|- lines intended|-   (a ride, lines comma separated)
// PWALK person edges speed duration arrivalPos departPosLat departLane busStop
// PSTOP person edge|@busStop duration until actType   (waiting stage)
// PTRIP person from|- to|- busStop arrivalPos speed duration departPosLat walkFactor   (walk without edges, routed
//       when it starts; '-' = not given)
void Sim::parsePersonStageLine(const std::vector<std::string>& tok) {
    const int pi = (int)persons.size() - 1;
    if (pi < 0 || persons[pi].id != tok[1]) {
        throw std::runtime_error("stage of unknown person " + tok[1]);
    }
    Person& p = persons[pi];
    if (tok[0] == "PRIDE") {
        // MSRouteHandler::addRideOrTransport
        PStage s;
        s.kind = PStage::DRIVING;
        s.lines = splitBy(tok[6], ',');
        s.intendedVeh = tok.size() > 7 && tok[7] != "-" ? tok[7] : "";
        const int stop = tok[4] != "-" && tok[4][0] != '%' ? busStopIdx.at(tok[4]) : -1;
        const int parking = tok[4][0] == '%' ? parkingAreaIdx.at(tok[4].substr(1)) : -1;
        int to = stop >= 0 ? net.lanes[busStops[stop].lane].edge : (parking >= 0 ? net.lanes[parkingAreas[parking].lane].edge : -1);
        if (tok[3] != "-") {
            to = net.edgeIdx.at(tok[3]);
        }
        int from = tok[2] != "-" ? net.edgeIdx.at(tok[2]) : -1;
        if (p.plan.empty() && p.triggered) {
            // the person starts in the vehicle: its depart and first edge
            for (const Veh& x : veh) {
                if (x.id == s.lines.front()) {
                    p.depart = x.depart;
                    if (from < 0) {
                        from = routes[x.route].front();
                    }
                    break;
                }
            }
        }
        if (p.plan.empty()) {
            if (from < 0) {
                throw std::runtime_error("start edge not defined for person " + p.id);
            }
            PStage w;
            w.kind = PStage::WAITING_FOR_DEPART;
            w.destination = from;
            const std::string& v = personDepartPos[pi];
            const double len = net.edges[from].length;
            double dp = 0;
            if (v == "random") {
                dp = rand01(opt.seed, (uint64_t)pi, 0, RP_PERSON_DEPARTPOS) * len;
            } else if (v != "-") {
                dp = std::stod(v);
            }
            w.arrivalPos = interpretEdgePos(dp, len);
            w.until = p.depart;
            p.plan.push_back(w);
        }
        if (to < 0) {
            throw std::runtime_error("no destination edge for person " + p.id);
        }
        s.origin = from;
        s.destination = to;
        s.destinationStop = stop;
        s.destinationParking = parking;
        if (tok[5] != "-") {
            s.arrivalPos = interpretEdgePos(std::stod(tok[5]), net.edges[to].length);
        } else if (stop >= 0) {
            s.arrivalPos = busStops[stop].endPos;
        } else if (parking >= 0) {
            s.arrivalPos = parkingAreas[parking].endPos;
        } else {
            s.arrivalPosUnspecified = true;  // MSStage::ARRIVALPOS_UNSPECIFIED (the edge end)
            s.arrivalPos = net.edges[to].length;
        }
        p.plan.push_back(s);
    } else if (tok[0] == "PTRIP") {
        // MSRouteHandler::addPersonTrip
        PStage s;
        s.kind = PStage::TRIP;
        // edges or the connectors of zones ("<taz>-source", "<taz>-sink")
        auto node = [&](const std::string& id) {
            auto e = net.edgeIdx.find(id);
            if (e != net.edgeIdx.end()) {
                return e->second;
            }
            for (const char* suffix : {"-source", "-sink"}) {
                const size_t n = std::strlen(suffix);
                if (id.size() > n && id.compare(id.size() - n, n, suffix) == 0) {
                    auto z = tazIdx.find(id.substr(0, id.size() - n));
                    if (z != tazIdx.end()) {
                        return tazNode(z->second, suffix[1] == 's' && suffix[2] == 'o');
                    }
                }
            }
            throw std::runtime_error("unknown edge " + id + " for person " + p.id);
        };
        auto nodeLength = [&](int n) { return n < (int)net.edges.size() ? net.edges[n].length : 0.; };
        int from;
        if (tok[2] != "-") {
            from = node(tok[2]);
        } else if (!p.plan.empty()) {
            from = p.plan.back().kind == PStage::TRIP ? p.plan.back().destination : p.plan.back().destination;
        } else {
            throw std::runtime_error("start edge not defined for person " + p.id);
        }
        int to = tok[3] != "-" ? node(tok[3]) : -1;
        const int stop = tok[4] != "-" && tok[4][0] != '%' ? busStopIdx.at(tok[4]) : -1;
        const int parking = tok[4][0] == '%' ? parkingAreaIdx.at(tok[4].substr(1)) : -1;
        if (to < 0) {
            if (stop < 0 && parking < 0) {
                throw std::runtime_error("no destination edge for person " + p.id);
            }
            to = net.lanes[stop >= 0 ? busStops[stop].lane : parkingAreas[parking].lane].edge;
        }
        if (p.plan.empty()) {
            PStage w;
            w.kind = PStage::WAITING_FOR_DEPART;
            w.destination = from;
            const std::string& v = personDepartPos[pi];
            const double len = nodeLength(from);
            double dp = 0;
            if (v == "random") {
                dp = rand01(opt.seed, (uint64_t)pi, 0, RP_PERSON_DEPARTPOS) * len;
            } else if (v != "-") {
                dp = std::stod(v);
            }
            w.arrivalPos = interpretEdgePos(dp, len);
            w.until = p.depart;
            p.plan.push_back(w);
        }
        // MSRouteHandler::parseWalkPositions (arrival)
        const double len = nodeLength(to);
        auto walkPos = [&](const std::string& v) {
            double r;
            if (v == "random") {
                r = rand01(opt.seed, (uint64_t)pi, p.plan.size(), RP_PERSON_ARRIVALPOS) * len;
            } else if (v == "center") {
                r = len / 2.;
            } else if (v == "max") {
                r = len;
            } else {
                r = std::stod(v);
            }
            return interpretEdgePos(r, len);
        };
        double arrivalPos;
        if (stop >= 0 || parking >= 0) {
            const double startPos = stop >= 0 ? busStops[stop].startPos : parkingAreas[parking].begPos;
            const double endPos = stop >= 0 ? busStops[stop].endPos : parkingAreas[parking].endPos;
            arrivalPos = (startPos + endPos) / 2.;
            if (tok[5] != "-") {
                const double arrPos = walkPos(tok[5]);
                if (arrPos >= startPos && arrPos < endPos) {
                    arrivalPos = arrPos;
                }
            }
        } else {
            arrivalPos = tok[5] != "-" ? walkPos(tok[5]) : len / 2.;
        }
        s.tripFrom = from;
        s.destination = to;
        s.destinationStop = stop;
        s.arrivalPos = arrivalPos;
        s.hasArrivalPos = tok[5] != "-";
        s.speed = tok[6] == "-" ? -1 : std::stod(tok[6]);
        s.walkingTime = tok[7] == "-" ? -1 : TIME2STEPS(std::stod(tok[7]));
        s.departPosLat = parsePersonPosLat(tok[8], from, -1);
        s.walkFactor = tok[9] == "-" ? opt.walkFactor : std::stod(tok[9]);
        s.destinationParking = parking;
        // SUMOVehicleParameter::parsePersonModes; the vehicle types add their mode (bicycle or car)
        if (tok.size() > 10 && tok[10] != "-") {
            for (const std::string& m : splitBy(tok[10], ',')) {
                if (m == "car") {
                    s.tripModes |= VC_PASSENGER;
                } else if (m == "taxi") {
                    s.tripModes |= VC_TAXI;
                } else if (m == "bicycle") {
                    s.tripModes |= VC_BICYCLE;
                } else if (m == "public") {
                    s.tripModes |= VC_BUS;
                } else if (m != "walk") {
                    throw std::runtime_error("Unknown person mode '" + m + "' for person " + p.id);
                }
            }
        }
        if (tok.size() > 11 && tok[11] != "-") {
            for (const std::string& t : splitBy(tok[11], ',')) {
                const int type = typeIdx.at(t);
                s.tripVTypes.push_back(type);
                s.tripModes |= types[type].vclass == VC_BICYCLE ? VC_BICYCLE : VC_PASSENGER;
            }
        }
        p.plan.push_back(s);
    } else if (tok[0] == "PTRANSHIP") {
        // PTRANSHIP container edges speed departPos|- arrivalPos|- stop|- (MSRouteHandler::addTranship)
        PStage s;
        s.kind = PStage::TRANSHIP;
        for (const std::string& e : splitBy(tok[2], ',')) {
            s.route.push_back(net.edgeIdx.at(e));
        }
        s.speed = std::stod(tok[3]);
        const double prePos = p.plan.empty() ? 0. : p.plan.back().arrivalPos;
        s.departPos = tok[4] == "-" ? prePos : std::stod(tok[4]);
        s.destinationStop = tok[6] != "-" ? busStopIdx.at(tok[6]) : -1;
        s.destination = s.route.back();
        s.arrivalPos = tok[5] != "-" ? std::stod(tok[5])
                       : (s.destinationStop >= 0 ? busStops[s.destinationStop].endPos : net.edges[s.route.back()].length);
        if (p.plan.empty()) {
            PStage w;
            w.kind = PStage::WAITING_FOR_DEPART;
            w.destination = s.route.front();
            w.arrivalPos = s.departPos;
            w.until = p.depart;
            p.plan.push_back(w);
        }
        p.plan.push_back(s);
    } else if (tok[0] == "PWALK") {
        PStage s;
        s.kind = PStage::WALKING;
        for (const std::string& e : splitBy(tok[2], ',')) {
            s.route.push_back(net.edgeIdx.at(e));
        }
        s.speed = tok[3] == "-" ? -1 : std::stod(tok[3]);
        s.walkingTime = tok[4] == "-" ? -1 : TIME2STEPS(std::stod(tok[4]));
        const int fromEdge = s.route.front();
        const int toEdge = s.route.back();
        if (p.plan.empty()) {
            // initial waiting stage at the person's departPos (MSStageWaiting, interpretEdgePos)
            PStage w;
            w.kind = PStage::WAITING_FOR_DEPART;
            w.destination = fromEdge;
            double dp = 0;
            const std::string& v = personDepartPos[pi];
            if (v == "random") {
                dp = rand01(opt.seed, (uint64_t)pi, 0, RP_PERSON_DEPARTPOS) * net.edges[fromEdge].length;
            } else if (v != "-") {
                dp = std::stod(v);
            }
            w.arrivalPos = interpretEdgePos(dp, net.edges[fromEdge].length);
            w.until = p.depart;
            p.plan.push_back(w);
        }
        // MSRouteHandler::parseWalkPositions
        const PStage& last = p.plan.back();
        double departPos = 0.;
        if (last.destinationStop >= 0) {
            departPos = stopAccessPos(last.destinationStop, fromEdge);
        } else if (last.destination == fromEdge) {
            departPos = last.arrivalPos;
        } else if (net.edges[last.destination].toJ == net.edges[fromEdge].toJ) {
            departPos = net.edges[fromEdge].length;
        }
        double arrivalPos;
        if (tok[8] != "-") {
            s.destinationStop = busStopIdx.at(tok[8]);
            arrivalPos = stopAccessPos(s.destinationStop, toEdge);
        } else if (tok[5] != "-") {
            const double len = net.edges[toEdge].length;
            if (tok[5] == "random") {
                arrivalPos = rand01(opt.seed, (uint64_t)pi, p.plan.size(), RP_PERSON_ARRIVALPOS) * len;
            } else if (tok[5] == "center") {
                arrivalPos = len / 2.;
            } else if (tok[5] == "max") {
                arrivalPos = len;
            } else {
                arrivalPos = std::stod(tok[5]);
            }
            arrivalPos = interpretEdgePos(arrivalPos, len);
        } else {
            arrivalPos = net.edges[toEdge].length / 2.;
        }
        s.departPosLat = parsePersonPosLat(tok[6], fromEdge, toI(tok[7]));
        s.departLane = toI(tok[7]);
        // MSStageWalking
        s.departPos = interpretEdgePos(departPos, net.edges[fromEdge].length);
        s.arrivalPos = interpretEdgePos(arrivalPos, net.edges[toEdge].length);
        s.destination = toEdge;
        if (s.walkingTime > 0) {
            s.speed = walkDistance(p, s, false) / STEPS2TIME(s.walkingTime + 1);
        }
        p.plan.push_back(s);
    } else {
        // PSTOP person edge|@busStop duration until actType (MSRouteHandler::addStop for persons)
        PStage s;
        s.kind = PStage::WAITING;
        if (tok[2][0] == '@') {
            s.destinationStop = busStopIdx.at(tok[2].substr(1));
            s.destination = net.lanes[busStops[s.destinationStop].lane].edge;
        } else {
            s.destination = net.edgeIdx.at(tok[2]);
        }
        s.duration = tok[3] == "-" ? -1 : TIME2STEPS(std::stod(tok[3]));
        s.until = tok[4] == "-" ? -1 : TIME2STEPS(std::stod(tok[4]));
        s.actType = tok.size() > 5 ? tok[5] : "";
        if (p.plan.empty()) {
            PStage w;
            w.kind = PStage::WAITING_FOR_DEPART;
            w.destination = s.destination;
            w.destinationStop = s.destinationStop;
            const std::string& v = personDepartPos[pi];
            const double len = net.edges[s.destination].length;
            double dp = 0;
            if (v == "random") {
                dp = rand01(opt.seed, (uint64_t)pi, 0, RP_PERSON_DEPARTPOS) * len;
            } else if (v != "-") {
                dp = std::stod(v);
            } else if (s.destinationStop >= 0) {
                dp = (busStops[s.destinationStop].startPos + busStops[s.destinationStop].endPos) / 2;
            }
            w.arrivalPos = interpretEdgePos(dp, len);
            w.until = p.depart;
            p.plan.push_back(w);
        } else if (p.plan.back().destination >= (int)net.edges.size()) {
            // MSStage::setDestination: the previous stage ends at the stop instead of a zone
            p.plan.back().destination = s.destination;
            p.plan.back().destinationStop = s.destinationStop;
        } else if (p.plan.back().destination != s.destination) {
            throw std::runtime_error("disconnected plan for person " + p.id);
        }
        // the stop is at the arrival position of the previous stage
        s.arrivalPosUnspecified = p.plan.back().arrivalPosUnspecified;
        s.arrivalPos = p.plan.back().arrivalPos;
        p.plan.push_back(s);
    }
}

// SUMOVehicleParameter::interpretEdgePos
double Sim::interpretEdgePos(double pos, double maximumValue) const {
    if (pos < 0) {
        pos = maximumValue + pos;
    }
    if (pos > maximumValue && pos != std::numeric_limits<double>::infinity()) {
        pos = maximumValue;
    }
    return pos;
}

// MSStoppingPlace::getAccessPos (stops without access elements)
double Sim::stopAccessPos(int stop, int edge) const {
    return placeAccessPos(stop, -1, edge, 0, 0, false);
}

// MSRouteHandler::interpretDepartPosLat
double Sim::parsePersonPosLat(const std::string& value, int edge, int departLane) const {
    if (value == "-") {
        return UNSPECIFIED_POS_LAT;
    }
    if (value == "random" || value == "free" || value == "random_free") {
        return RANDOM_POS_LAT;
    }
    if (value == "right" || value == "left" || value == "center") {
        const int lane = departLane > 0 ? net.edges[edge].lanes[departLane] : net.edges[edge].lanes[0];
        const double usableWidth = net.lanes[lane].width - 0.5;
        return value == "right" ? -usableWidth / 2 : (value == "left" ? usableWidth / 2 : 0.);
    }
    return std::stod(value);
}

// SUMO options of the pedestrian model
void Sim::applyPedOptions() {
    for (const auto& kv : opt.pedOptions) {
        const std::string& k = kv.first;
        const std::string& v = kv.second;
        if (k == "pedestrian.model") {
            if (v == "nonInteracting") {
                pedNonInteracting = true;
            } else if (v != "striping") {
                throw std::runtime_error("pedestrian model " + v + " not supported");
            }
        } else if (k == "pedestrian.timegap-crossing") timegapCrossing = std::stod(v);
        else if (k == "pedestrian.striping.stripe-width") striping.stripeWidth = std::stod(v);
        else if (k == "pedestrian.striping.dawdling") striping.dawdling = std::stod(v);
        else if (k == "pedestrian.striping.mingap-to-vehicle") striping.minGapToVehicle = std::stod(v);
        else if (k == "pedestrian.striping.jamtime") striping.jamTime = TIME2STEPS(std::stod(v));
        else if (k == "pedestrian.striping.jamtime.crossing") striping.jamTimeCrossing = TIME2STEPS(std::stod(v));
        else if (k == "pedestrian.striping.jamtime.narrow") striping.jamTimeNarrow = TIME2STEPS(std::stod(v));
        else if (k == "pedestrian.striping.jamfactor") striping.jamFactor = std::stod(v);
        else if (k == "pedestrian.striping.reserve-oncoming") striping.reserveOncoming = std::stod(v);
        else if (k == "pedestrian.striping.reserve-oncoming.junctions") striping.reserveOncomingJunctions = std::stod(v);
        else if (k == "pedestrian.striping.reserve-oncoming.max") striping.reserveOncomingMax = std::stod(v);
        else if (k == "pedestrian.striping.legacy-departposlat") striping.legacyPosLat = v == "true" || v == "1";
        else if (k == "pedestrian.striping.walkingarea-detail") striping.walkingAreaDetail = std::stoi(v);
        else throw std::runtime_error("unknown pedestrian option " + k);
    }
    // beginning with 1.20.0, sensible default speeds were set for crossings and walkingareas
    striping.useNetSpeeds = net.netVersion >= 1020;
    // non-positive jam times disable squeezing (MSPModel_Striping constructor)
    if (striping.jamTime <= 0) {
        striping.jamTime = TIME_MAX;
    }
    if (striping.jamTimeCrossing <= 0) {
        striping.jamTimeCrossing = TIME_MAX;
    }
    if (striping.jamTimeNarrow <= 0) {
        striping.jamTimeNarrow = TIME_MAX;
    }
}

void Sim::initPersons() {
    applyPedOptions();
    // MSPModel_Interacting::usingInternalLanesStatic: internal links and a pedestrian network (walking areas)
    bool internal = false, walkingAreas = false;
    for (const Edge& e : net.edges) {
        internal = internal || e.internal;
        walkingAreas = walkingAreas || e.isWalkingArea();
    }
    pedUsingInternalLanes = !pedNonInteracting && internal && walkingAreas;
    for (int p = 0; p < (int)persons.size(); ++p) {
        if (persons[p].id == opt.trace) {
            tracePerson = p;
        }
        // persons departing before the begin time are not loaded
        if (persons[p].depart < TIME2STEPS(opt.begin)) {
            nPersonsEnded++;
            continue;
        }
        // MSTransportableControl::add: departures at the next step boundary
        const Time d = persons[p].depart;
        const Time step = d % DELTA_T == 0 ? d : (d / DELTA_T + 1) * DELTA_T;
        waiting4Departure[step].push_back(p);
    }
    edgePersons.assign(net.edges.size(), 0);
    if (!persons.empty()) {
        initWalkingAreaPaths();
        pedRouter.walkOppositeFactor = opt.walkOppositeFactor;
        pedRouter.build(net);
        pedsOnLane.assign(net.lanes.size(), {});
        pedSnapOnLane.assign(net.lanes.size(), {});
        pedInWave.assign(net.lanes.size(), 0);
        approachingPersons.assign(net.links.size(), {});
        pedChanged.assign(persons.size(), 0);
    }
}

// ---------------------------------------------------------------------------
// MSTransportableControl / MSTransportable
// ---------------------------------------------------------------------------

// MSTransportableControl::checkWaiting: departures, then the ends of waiting stages
void Sim::personCheckWaiting() {
    haveNewWaiting = false;
    auto it = waiting4Departure.find(now);
    if (it != waiting4Departure.end()) {
        std::vector<int> ps = std::move(it->second);
        waiting4Departure.erase(it);
        for (int p : ps) {
            persons[p].departed = true;
            persons[p].plan[0].departed = now;
            nPersonsRunning++;
            if (!personProceed(p)) {
                personErase(p);
            }
        }
    }
    auto w = waitingUntil.find(now);
    while (w != waitingUntil.end()) {
        std::vector<int> ps = std::move(w->second);
        waitingUntil.erase(w);
        for (int p : ps) {
            if (!personProceed(p)) {
                personErase(p);
            }
        }
        w = waitingUntil.find(now);
    }
}

// MSTransportableControl::erase
void Sim::personErase(int pi) {
    writePersonInfo(pi);
    if (persons[pi].departed) {
        nPersonsRunning--;
    }
    nPersonsEnded++;
}

// MSStoppingPlace::addTransportable / removeTransportable (waiting places are not modelled, only the count)
void Sim::stopAddPerson(int stop, int pi) {
    BusStop& b = busStops[stop];
    Person& p = persons[pi];
    if (p.atStop != stop && b.numWaitingPersons < b.waitingCapacity) {
        // the first free waiting spot (MSStoppingPlace::addTransportable)
        if ((int)b.spotUsed.size() != b.waitingCapacity) {
            b.spotUsed.assign(b.waitingCapacity, 0);
        }
        int spot = 0;
        while (b.spotUsed[spot]) {
            ++spot;
        }
        b.spotUsed[spot] = 1;
        b.numWaitingPersons++;
        p.atStop = stop;
        p.stopSpot = spot;
    }
}

void Sim::edgeAddPerson(int edge, int pi) {
    Person& p = persons[pi];
    if (p.container) {
        return;  // (MSEdge::myContainers: not seen by vehicles and pedestrians)
    }
    if (p.onEdge != edge) {
        edgeRemovePerson(pi);
        __atomic_add_fetch(&edgePersons[edge], 1, __ATOMIC_RELAXED);
        p.onEdge = edge;
    }
}

void Sim::edgeRemovePerson(int pi) {
    Person& p = persons[pi];
    if (p.onEdge >= 0) {
        __atomic_sub_fetch(&edgePersons[p.onEdge], 1, __ATOMIC_RELAXED);
        p.onEdge = -1;
    }
}

void Sim::stopRemovePerson(int stop, int pi) {
    Person& p = persons[pi];
    if (p.atStop == stop) {
        busStops[stop].numWaitingPersons--;
        if (p.stopSpot >= 0 && p.stopSpot < (int)busStops[stop].spotUsed.size()) {
            busStops[stop].spotUsed[p.stopSpot] = 0;
        }
        p.atStop = -1;
        p.stopSpot = -1;
    }
}

// MSTransportable::proceed: end the current stage and start the next one
bool Sim::personProceed(int pi) {
    Person& p = persons[pi];
    p.plan[p.step].arrived = now;
    if (p.plan[p.step].kind == PStage::DRIVING) {
        rideSetArrived(pi);
    } else if (p.plan[p.step].walkCounted) {
        nWalking--;
    }
    edgeRemovePerson(pi);
    const int prior = p.step;
    p.step++;
    // an access stage to the stop where a walk or ride ended at the stop's access
    const PStage::Kind priorKind = p.plan[prior].kind;
    bool accessToStop = false;
    if (priorKind == PStage::WALKING || priorKind == PStage::DRIVING) {
        accessToStop = checkAccess(pi, prior, true);
    }
    // persons leave the stop of the prior stage unless a ride (or a trip) starts there
    const PStage::Kind next = p.step < (int)p.plan.size() ? p.plan[p.step].kind : PStage::WAITING;
    if (!accessToStop && (p.step == (int)p.plan.size() || (next != PStage::DRIVING && next != PStage::TRIP))) {
        const int priorStop = priorKind == PStage::TRIP ? p.plan[prior].originStop : p.plan[prior].destinationStop;
        if (priorStop >= 0) {
            stopRemovePerson(priorStop, pi);
        }
    }
    if (p.step < (int)p.plan.size()) {
        // an access stage from the stop to the first edge of a walk
        if (next == PStage::WALKING && (priorKind != PStage::ACCESS || p.plan[prior].destination != p.plan[p.step].route[0])) {
            checkAccess(pi, prior, false);
        } else if (next == PStage::WAITING && priorKind == PStage::WAITING && p.plan[prior].destination != p.plan[p.step].destination) {
            checkAccess(pi, prior, false);
        }
        stageProceed(pi, prior);
        return true;
    }
    p.arrived = true;
    p.arrivalTime = STEPS2TIME(now);
    nPersonsArrived++;
    return false;
}

// MSStage*::proceed
void Sim::stageProceed(int pi, int priorIdx) {
    Person& p = persons[pi];
    PStage& s = p.plan[p.step];
    const PStage& prior = p.plan[priorIdx];
    if (s.kind == PStage::DRIVING) {
        rideProceed(pi, priorIdx);  // (departed is the time the ride starts moving)
        return;
    }
    s.departed = now;
    if (s.kind == PStage::TRIP) {
        tripProceed(pi, priorIdx);
        return;
    }
    if (s.kind == PStage::TRANSHIP) {
        // MSStageTranship::proceed with MSPModel_NonInteracting::CState: straight from the start to the end
        s.departPos = stageEdgePos(pi, priorIdx);
        const Lane& from = net.lanes[net.edges[s.route.front()].lanes[0]];
        const Lane& to = net.lanes[net.edges[s.route.back()].lanes[0]];
        const Pos a = geom::positionAtOffset(from.shape, from.interpolateLanePosToGeometryPos(s.departPos), 0);
        const Pos b = geom::positionAtOffset(to.shape, to.interpolateLanePosToGeometryPos(s.arrivalPos), 0);
        s.accessArrival = now + std::max((Time)1, TIME2STEPS(a.distanceTo2D(b) / s.speed));
        accessEvents.insert({s.accessArrival, pi});  // (MoveToNextEdge: a begin-of-step event)
        return;
    }
    if (s.kind == PStage::ACCESS) {
        // MSPersonStage_Access::proceed: the end is a begin-of-step event
        s.accessArrival = now + TIME2STEPS(s.accessDist / personMaxSpeed(pi));
        accessEvents.insert({s.accessArrival, pi});
        edgeAddPerson(s.destinationStop >= 0 ? net.lanes[busStops[s.destinationStop].lane].edge : s.destination, pi);
        return;
    }
    if (s.kind == PStage::WALKING) {
        s.routeStep = 0;
        s.lastEdgeEntryTime = now;
        if (s.walkingTime == 0) {
            if (!personProceed(pi)) {
                personErase(pi);
            }
            return;
        }
        // previous->getEdgePos(now) >= 0 && previous->getEdge() == *myRouteStep
        const double prevPos = stageEdgePos(pi, priorIdx);
        if (prevPos >= 0 && stageEdge(prior) == s.route[0]) {
            s.departPos = prevPos;
            if (s.walkingTime > 0) {
                s.speed = walkDistance(p, s, false) / STEPS2TIME(s.walkingTime + 1);
            }
        }
        if (pedNonInteracting) {
            niAdd(pi);
        } else {
            pedAdd(pi);
        }
        nWalking++;
        s.walkCounted = true;
        edgeAddPerson(s.route[0], pi);
    } else {
        // MSStageWaiting::proceed
        s.stopEndTime = std::max(std::max(now, now + s.duration), s.until);
        if (s.arrivalPosUnspecified) {
            s.arrivalPos = prior.arrivalPos;
        }
        if (s.destinationStop >= 0) {
            stopAddPerson(s.destinationStop, pi);
        }
        edgeAddPerson(s.destination, pi);
        // MSTransportableControl::setWaitEnd
        const Time t = s.stopEndTime;
        const Time step = t % DELTA_T == 0 ? t : (t / DELTA_T + 1) * DELTA_T;
        waitingUntil[step].push_back(pi);
    }
}

int Sim::stageEdge(const PStage& s) const {
    if (s.kind == PStage::TRIP) {
        return s.tripFrom;
    }
    if (s.kind == PStage::DRIVING) {
        // MSStageDriving::getEdge
        if (s.vehicle >= 0) {
            return kin[s.vehicle].lane >= 0 ? net.lanes[kin[s.vehicle].lane].edge : curEdge(s.vehicle);
        }
        return s.arrived >= 0 ? s.destination : s.waitingEdge;
    }
    if (s.kind == PStage::WALKING) {
        if (s.currentInternalEdge >= 0) {
            return s.currentInternalEdge;
        }
        return s.routeStep < (int)s.route.size() ? s.route[s.routeStep] : -1;
    }
    return s.destination;
}

double Sim::stageEdgePos(int pi, int si) const {
    const PStage& s = persons[pi].plan[si];
    if (s.kind == PStage::WALKING) {
        return s.pstate < 0 ? s.departPos : pstates[s.pstate].edgePos;
    }
    if (s.kind == PStage::TRIP) {
        return s.departPos;
    }
    if (s.kind == PStage::DRIVING) {
        // MSStageDriving::getEdgePos
        if (s.vehicle < 0 && s.arrived < 0) {
            return s.waitingPos;
        }
        if (s.arrived >= 0) {
            return s.arrivalPos;
        }
        return std::min(kin[s.vehicle].pos, net.edges[stageEdge(s)].length);
    }
    return s.arrivalPos;
}

double Sim::personMaxSpeed(int pi) const {
    const VType& t = types[persons[pi].type];
    return std::min(t.maxSpeed, t.desiredMaxSpeed * persons[pi].speedFactor);
}

// MSStageWalking::getMaxSpeed
double Sim::walkMaxSpeed(int pi, const PStage& s) const {
    return s.speed >= 0 ? s.speed : personMaxSpeed(pi);
}

// MSLane::getVehicleMaxSpeed for a person
double Sim::laneMaxSpeedFor(int lane, int pi) const {
    return std::min(personMaxSpeed(pi), net.lanes[lane].speed * persons[pi].speedFactor);
}

// MSPModel::canTraverse
int Sim::canTraverse(int dir, const std::vector<int>& route, int& passedEdges) const {
    int junction = -1;
    for (int e : route) {
        if (junction >= 0) {
            if (junction == net.edges[e].fromJ) {
                dir = PED_FORWARD;
            } else if (junction == net.edges[e].toJ) {
                dir = PED_BACKWARD;
            } else {
                return PED_UNDEFINED;
            }
        }
        junction = dir == PED_FORWARD ? net.edges[e].toJ : net.edges[e].fromJ;
        passedEdges++;
    }
    return dir;
}

// MSStageWalking::walkDistance
double Sim::walkDistance(const Person& p, const PStage& s, bool partial) const {
    double length = 0;
    const int endIt = partial && s.arrived < 0 ? s.routeStep + 1 : (int)s.route.size();
    for (int i = 0; i < endIt; ++i) {
        length += net.edges[s.route[i]].length;
    }
    if (s.route.size() > 1 && pedUsingInternalLanes) {  // MSPModel::usingInternalLanes
        if (s.internalDistance > 0) {
            length += s.internalDistance;
        } else {
            for (int i = 0; i < endIt - 1; ++i) {
                const int fromEdge = s.route[i];
                const int toEdge = s.route[i + 1];
                const int from = getSidewalk(net, fromEdge);
                const int to = getSidewalk(net, toEdge);
                if (from >= 0 && to >= 0) {
                    const Edge& F = net.edges[fromEdge];
                    const Edge& T = net.edges[toEdge];
                    Pos fromPos, toPos;
                    if (F.toJ == T.fromJ) {
                        fromPos = net.lanes[from].shape.back();
                        toPos = net.lanes[to].shape.front();
                    } else if (F.toJ == T.toJ) {
                        fromPos = net.lanes[from].shape.back();
                        toPos = net.lanes[to].shape.back();
                    } else if (F.fromJ == T.fromJ) {
                        fromPos = net.lanes[from].shape.front();
                        toPos = net.lanes[to].shape.front();
                    } else if (F.fromJ == T.toJ) {
                        fromPos = net.lanes[from].shape.front();
                        toPos = net.lanes[to].shape.back();
                    }
                    length += fromPos.distanceTo2D(toPos);
                }
            }
        }
    }
    int dummy = 0;
    const int departFwdArrivalDir = canTraverse(PED_FORWARD, s.route, dummy);
    const int departBwdArrivalDir = canTraverse(PED_BACKWARD, s.route, dummy);
    const bool mayStartForward = departFwdArrivalDir != PED_UNDEFINED;
    const bool mayStartBackward = departBwdArrivalDir != PED_UNDEFINED;
    const double arrivalPos = partial && s.arrived < 0 ? (s.pstate >= 0 ? pstates[s.pstate].edgePos : s.departPos) : s.arrivalPos;
    const double lastLen = net.edges[s.route.back()].length;
    const double lengthFwd = length - s.departPos - (departFwdArrivalDir == PED_BACKWARD ? arrivalPos : lastLen - arrivalPos);
    const double lengthBwd = length - (net.edges[s.route.front()].length - s.departPos)
                             - (departBwdArrivalDir == PED_BACKWARD ? arrivalPos : lastLen - arrivalPos);
    if (s.route.size() == 1) {
        length = s.departPos > s.arrivalPos ? lengthBwd : lengthFwd;
    } else if (mayStartForward && mayStartBackward) {
        length = lengthFwd < lengthBwd ? lengthFwd : lengthBwd;
    } else if (mayStartForward) {
        length = lengthFwd;
    } else if (mayStartBackward) {
        length = lengthBwd;
    } else {
        length = lengthFwd;
    }
    (void)p;
    return std::max(POSITION_EPS, length);
}

// ---------------------------------------------------------------------------
// walking area paths (MSPModel_Striping::initWalkingAreaPaths)
// ---------------------------------------------------------------------------
void Sim::initWalkingAreaPaths() {
    waPaths.clear();
    waPathIdx.clear();
    minNextLength.assign(net.lanes.size(), 0);
    waFoes.assign(net.edges.size(), {});
    // vehicle lanes crossing walking areas (shared space)
    for (const Edge& E : net.edges) {
        if (E.isWalkingArea() || E.isCrossing()) {
            continue;
        }
        for (int lane : E.lanes) {
            for (int l : net.lanes[lane].links) {
                const Link& k = net.links[l];
                if (k.walkingAreaFoe >= 0) {
                    waFoes[net.lanes[k.walkingAreaFoe].edge].push_back(k.laneBefore);
                }
                if (k.walkingAreaFoeExit >= 0) {
                    waFoes[net.lanes[k.walkingAreaFoeExit].edge].push_back(k.laneBefore);
                }
            }
        }
    }
    for (int e = 0; e < (int)net.edges.size(); ++e) {
        const Edge& E = net.edges[e];
        if (!E.isWalkingArea()) {
            continue;
        }
        const int walkingArea = getSidewalk(net, e);
        minNextLength[walkingArea] = net.lanes[walkingArea].length;
        std::vector<int> lanes;
        for (int in : E.predecessors) {
            lanes.push_back(getSidewalk(net, in));
            if (lanes.back() < 0) {
                throw std::runtime_error("invalid connection from edge " + net.edges[in].id + " to walkingarea " + E.id);
            }
        }
        for (int out : E.successors) {
            lanes.push_back(getSidewalk(net, out));
            if (lanes.back() < 0) {
                throw std::runtime_error("invalid connection from walkingarea " + E.id + " to edge " + net.edges[out].id);
            }
        }
        const double waWidth = net.lanes[walkingArea].width;
        for (int j = 0; j < (int)lanes.size(); ++j) {
            for (int k = 0; k < (int)lanes.size(); ++k) {
                if (j == k) {
                    continue;
                }
                const int from = lanes[j];
                const int to = lanes[k];
                const int fromDir = net.linkTo(from, walkingArea) != nullptr ? PED_FORWARD : PED_BACKWARD;
                const int toDir = net.linkTo(walkingArea, to) != nullptr ? PED_FORWARD : PED_BACKWARD;
                const Shape& fs = net.lanes[from].shape;
                const Shape& ts = net.lanes[to].shape;
                Shape shape;
                const Pos fromPos = fromDir == PED_FORWARD ? fs.back() : fs.front();
                const Pos toPos = toDir == PED_FORWARD ? ts.front() : ts.back();
                const double maxExtent = fromPos.distanceTo2D(toPos) / 4;
                const double extrapolateBy = std::min(maxExtent, waWidth / 2);
                shape.push_back(fromPos);
                if (extrapolateBy > POSITION_EPS) {
                    Shape fromShp = fs;
                    geom::extrapolate(fromShp, extrapolateBy);
                    geom::pushBackNoDoublePos(shape, fromDir == PED_FORWARD ? fromShp.back() : fromShp.front());
                    Shape nextShp = ts;
                    geom::extrapolate(nextShp, extrapolateBy);
                    geom::pushBackNoDoublePos(shape, toDir == PED_FORWARD ? nextShp.front() : nextShp.back());
                }
                geom::pushBackNoDoublePos(shape, toPos);
                if (shape.size() < 2) {
                    Shape fromShp = fs;
                    geom::extrapolate(fromShp, 1.5 * POSITION_EPS);
                    geom::pushBackNoDoublePos(shape, fromDir == PED_FORWARD ? fromShp.back() : fromShp.front());
                } else if (striping.walkingAreaDetail > 4) {
                    shape = geom::bezier(shape, striping.walkingAreaDetail);
                }
                double angleOverride = INVALID_DOUBLE;
                if (shape.size() >= 4 && geom::length2D(shape) < waWidth) {
                    const double aStart = geom::angleAt2D(shape, 0);
                    const double aEnd = geom::angleAt2D(shape, (int)shape.size() - 2);
                    if (std::fabs(aStart - aEnd) < geom::rad(10)) {
                        angleOverride = (aStart + aEnd) / 2;
                    }
                }
                if (fromDir == PED_BACKWARD) {
                    std::reverse(shape.begin(), shape.end());
                }
                WalkingAreaPath w;
                w.from = from;
                w.to = to;
                w.lane = walkingArea;
                w.shape = shape;
                w.dir = fromDir;
                w.angleOverride = angleOverride;
                w.length = geom::length2D(shape);
                // std::map::insert keeps the first path of a (from, to) pair
                if (waPathIdx.emplace(std::make_pair(from, to), (int)waPaths.size()).second) {
                    waPaths.push_back(w);
                }
                minNextLength[walkingArea] = std::min(minNextLength[walkingArea], w.length);
            }
        }
    }
}

int Sim::getArbitraryPath(int walkingAreaEdge) const {
    const Edge& E = net.edges[walkingAreaEdge];
    std::vector<int> lanes;
    for (int pred : E.predecessors) {
        lanes.push_back(getSidewalk(net, pred));
    }
    for (int succ : E.successors) {
        lanes.push_back(getSidewalk(net, succ));
    }
    if (lanes.empty()) {
        throw std::runtime_error("invalid walkingarea " + E.id + " does not allow continuation");
    }
    return waPathIdx.at(std::make_pair(lanes.front(), lanes.back()));
}

int Sim::getWalkingAreaPath(int walkingAreaEdge, int before, int after) const {
    auto it = waPathIdx.find(std::make_pair(before, after));
    if (it != waPathIdx.end()) {
        return it->second;
    }
    for (int pred : net.edges[walkingAreaEdge].predecessors) {
        auto it2 = waPathIdx.find(std::make_pair(getSidewalk(net, pred), after));
        if (it2 != waPathIdx.end()) {
            return it2->second;
        }
    }
    return getArbitraryPath(walkingAreaEdge);
}

// ---------------------------------------------------------------------------
// PState helpers
// ---------------------------------------------------------------------------
int Sim::numStripes(int lane) const {
    return std::max(1, (int)std::floor(net.lanes[lane].width / striping.stripeWidth));
}

int Sim::stripeOf(double relY) const {
    return (int)std::floor(relY / striping.stripeWidth + 0.5);
}

double Sim::pedWidth(const PState& p) const {
    return p.vehicle >= 0 ? p.yWidth : types[persons[p.person].type].width;
}

double Sim::pedLength(const PState& p) const {
    return types[persons[p.person].type].length;
}

double Sim::pedMinGap(const PState& p) const {
    return types[persons[p.person].type].minGap;
}

int Sim::otherStripeOf(const PState& p, double relY) const {
    const int s = stripeOf(relY);
    const double offset = relY - s * striping.stripeWidth;
    const double threshold = std::max(NUMERICAL_EPS, striping.stripeWidth - SQUEEZE * pedWidth(p));
    if (offset > threshold) {
        return s + 1;
    } else if (offset < -threshold) {
        return s - 1;
    }
    return s;
}

int Sim::pedStripe(const PState& p) const {
    return std::min(std::max(0, stripeOf(p.posLat)), numStripes(p.lane) - 1);
}

int Sim::pedOtherStripe(const PState& p) const {
    return std::min(std::max(0, otherStripeOf(p, p.posLat)), numStripes(p.lane) - 1);
}

double Sim::pedMinX(const PState& p, bool includeMinGap) const {
    if (p.vehicle >= 0) {
        return p.xWidth > 0 ? p.edgePos - p.xWidth : p.edgePos;
    }
    if (p.dir == PED_FORWARD) {
        return p.edgePos - pedLength(p);
    }
    return p.edgePos - (includeMinGap ? pedMinGap(p) : 0.);
}

double Sim::pedMaxX(const PState& p, bool includeMinGap) const {
    if (p.vehicle >= 0) {
        return p.xWidth > 0 ? p.edgePos : p.edgePos - p.xWidth;
    }
    if (p.dir == PED_FORWARD) {
        return p.edgePos + (includeMinGap ? pedMinGap(p) : 0.);
    }
    return p.edgePos + pedLength(p);
}

PedObstacle Sim::obstacleOf(const PState& p) const {
    PedObstacle o;
    o.xFwd = pedMaxX(p, true);
    o.xBack = pedMinX(p, true);
    o.speed = p.dir * p.speed;
    o.type = p.vehicle >= 0 ? OBSTACLE_VEHICLE : OBSTACLE_PED;
    o.person = p.person;
    o.vehicle = p.vehicle;
    return o;
}

double Sim::distanceTo(const PState& p, const PedObstacle& obs, bool includeMinGap) const {
    const double maxX = pedMaxX(p, includeMinGap);
    const double minX = pedMinX(p, includeMinGap);
    if ((obs.xFwd >= maxX && obs.xBack <= maxX) || (obs.xFwd <= maxX && obs.xFwd >= minX)) {
        return (obs.type == OBSTACLE_PED && obs.person == p.person) ? DIST_FAR_AWAY : DIST_OVERLAP;
    }
    if (p.dir == PED_FORWARD) {
        return obs.xFwd < minX ? DIST_BEHIND : obs.xBack - maxX;
    }
    return obs.xBack > maxX ? DIST_BEHIND : minX - obs.xFwd;
}

void Sim::mergeObstacles(const PState& p, std::vector<PedObstacle>& into, const std::vector<PedObstacle>& obs2) const {
    for (int i = 0; i < (int)into.size(); ++i) {
        const double dO = distanceTo(p, obs2[i], obs2[i].type == OBSTACLE_PED);
        const double dI = distanceTo(p, into[i], into[i].type == OBSTACLE_PED);
        if (dO < dI) {
            into[i] = obs2[i];
        } else if (dO == dI && into[i].type != OBSTACLE_PED && into[i].type != OBSTACLE_VEHICLE
                   && (obs2[i].type == OBSTACLE_PED || obs2[i].type == OBSTACLE_VEHICLE)) {
            into[i] = obs2[i];
        }
    }
}

void Sim::mergeObstaclesDir(std::vector<PedObstacle>& into, const std::vector<PedObstacle>& obs2, int dir, int offset) {
    for (int i = 0; i < (int)into.size(); ++i) {
        const int i2 = i + offset;
        if (i2 >= 0 && i2 < (int)obs2.size()) {
            if (dir == PED_FORWARD) {
                if (obs2[i2].xBack < into[i].xBack) {
                    into[i] = obs2[i2];
                }
            } else if (obs2[i2].xFwd > into[i].xFwd) {
                into[i] = obs2[i2];
            }
        }
    }
}

const PStage& Sim::walkStage(const PState& p) const {
    const Person& q = persons[p.person];
    return q.plan[q.step];
}

PStage& Sim::walkStageMut(const PState& p) {
    Person& q = persons[p.person];
    return q.plan[q.step];
}

int Sim::nextRouteEdge(const PStage& s) const {
    return s.routeStep + 1 >= (int)s.route.size() ? -1 : s.route[s.routeStep + 1];
}

double Sim::distToLaneEnd(const PState& p) const {
    const PStage& s = walkStage(p);
    if (nextRouteEdge(s) < 0) {
        bool capacity = true;
        if (s.destinationStop >= 0) {
            capacity = busStops[s.destinationStop].waitingCapacity > busStops[s.destinationStop].numWaitingPersons;
        }
        return p.dir * (s.arrivalPos - p.edgePos) - POSITION_EPS - ((p.waitingTime > DELTA_T && capacity) ? pedMinGap(p) : 0);
    }
    const double length = p.wap < 0 ? net.lanes[p.lane].length : waPaths[p.wap].length;
    return p.dir == PED_FORWARD ? length - p.edgePos : p.edgePos;
}

double Sim::pedImpatience(const PState& p) const {
    return std::max(0., std::min(1., types[persons[p.person].type].impatience + STEPS2TIME(p.waitingTime) / MAX_WAIT_TOLERANCE));
}

bool Sim::pedIgnoreRed(const PState& p, int link) const {
    const Link& L = net.links[link];
    if (L.haveRed()) {
        const double ignoreRedTime = types[persons[p.person].type].jmDriveAfterRedTime;
        if (ignoreRedTime >= 0) {
            return ignoreRedTime > STEPS2TIME(now - L.lastStateChange);
        }
    }
    return false;
}

bool Sim::pedStopForYellow(const PState& p, int link) const {
    const Link& L = net.links[link];
    if (L.haveYellow()) {
        const double ignoreYellowTime = types[persons[p.person].type].jmDriveAfterYellowTime;
        if (ignoreYellowTime >= 0) {
            return ignoreYellowTime < STEPS2TIME(now - L.lastStateChange);
        }
        return true;
    }
    return false;
}

// PState::getPosition
Pos Sim::pedPosition(const PState& p) const {
    if (p.lane < 0) {
        return INVALID_POS;
    }
    const Lane& L = net.lanes[p.lane];
    const double lateralOffset = -(.5 * (L.width - striping.stripeWidth) - p.posLat);
    if (p.wap < 0) {
        return geom::positionAtOffset(L.shape, L.interpolateLanePosToGeometryPos(p.edgePos), lateralOffset);
    }
    const WalkingAreaPath& w = waPaths[p.wap];
    if (w.angleOverride == INVALID_DOUBLE) {
        return geom::positionAtOffset(w.shape, p.edgePos, lateralOffset);
    }
    const double rotationOffset = p.dir == PED_FORWARD ? 0 : geom::rad(180);
    return geom::sidePositionAtAngle(w.shape, p.edgePos, lateralOffset, w.angleOverride + rotationOffset);
}

// PState::getAngle (radians; SUMO caches the value until the next move)
double Sim::pedAngle(const PState& p) const {
    if (p.lane < 0) {
        return 0;
    }
    if (p.wap >= 0 && waPaths[p.wap].angleOverride != INVALID_DOUBLE) {
        return waPaths[p.wap].angleOverride;
    }
    const Shape& shp = p.wap < 0 ? net.lanes[p.lane].shape : waPaths[p.wap].shape;
    const double geomX = p.wap < 0 ? net.lanes[p.lane].interpolateLanePosToGeometryPos(p.edgePos) : p.edgePos;
    double angle = geom::rotationAtOffset(shp, geomX) + (p.dir == PED_BACKWARD ? M_PI : 0);
    if (p.dir == PED_BACKWARD) {
        angle += std::atan2(p.speedLat, std::max(p.speed, NUMERICAL_EPS));
    } else {
        angle -= std::atan2(p.speedLat, std::max(p.speed, NUMERICAL_EPS));
    }
    if (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    return angle;
}

// ---------------------------------------------------------------------------
// next lane (MSPModel_Striping::getNextLane)
// ---------------------------------------------------------------------------
int Sim::getNextWalkingArea(int currentLane, int dir, int& link) const {
    if (dir == PED_FORWARD) {
        for (int l : net.lanes[currentLane].links) {
            if (net.lanes[net.links[l].lane].walkingArea) {
                link = l;
                return net.links[l].lane;
            }
        }
    } else {
        for (const IncomingLane& in : net.lanes[currentLane].incoming) {
            if (net.lanes[in.lane].walkingArea) {
                link = in.link;
                return in.lane;
            }
        }
    }
    return -1;
}

int Sim::connectedDirection(int from, int to) const {
    if (from < 0 || to < 0) {
        return PED_UNDEFINED;
    } else if (net.linkTo(from, to) != nullptr) {
        return PED_FORWARD;
    } else if (net.linkTo(to, from) != nullptr) {
        return PED_BACKWARD;
    }
    return PED_UNDEFINED;
}

PedNextLane Sim::getNextLane(const PState& p, int currentLane, int prevLane) {
    const int currentEdge = net.lanes[currentLane].edge;
    const Edge& CE = net.edges[currentEdge];
    const int junction = p.dir == PED_FORWARD ? CE.toJ : CE.fromJ;
    const PStage& stage = walkStage(p);
    const int nextRouteE = nextRouteEdge(stage);
    const SVCPermissions svc = types[persons[p.person].type].vclass;
    int nextRouteLane = getSidewalk(net, nextRouteE, svc);
    int nextLane = nextRouteLane;
    int link = -1;
    int nextDir = PED_UNDEFINED;
    if (nextRouteLane < 0 && nextRouteE >= 0) {
        throw std::runtime_error("person " + persons[p.person].id + " could not find sidewalk on edge " + net.edges[nextRouteE].id);
    }
    if (nextRouteLane >= 0) {
        const Edge& NRE = net.edges[nextRouteE];
        if (CE.internal) {
            nextDir = junction == NRE.fromJ ? PED_FORWARD : PED_BACKWARD;
            if (nextDir == PED_FORWARD) {
                nextLane = net.links[net.lanes[currentLane].links[0]].viaOrLane();
            } else {
                nextLane = net.lanes[currentLane].logicalPred;
            }
        } else if (CE.isCrossing()) {
            nextDir = p.dir;
            if (nextDir == PED_FORWARD) {
                nextLane = net.links[net.lanes[currentLane].links[0]].lane;
            } else {
                nextLane = net.lanes[currentLane].logicalPred;
            }
            if (isPriorityCrossing(currentLane)) {
                unregisterCrossingApproach(p.person, currentLane);
            }
        } else if (CE.isWalkingArea()) {
            const int nextRouteEdgeDir = NRE.fromJ == junction ? PED_FORWARD : PED_BACKWARD;
            const double arrivalPos = (nextRouteE == stage.route.back()
                                       ? stage.arrivalPos
                                       : (nextRouteEdgeDir == PED_FORWARD ? 0 : NRE.length));
            PedRouter::Trip trip;
            trip.from = currentEdge;
            trip.to = nextRouteE;
            trip.departPos = 0;
            trip.arrivalPos = arrivalPos;
            trip.speed = walkMaxSpeed(p.person, stage);
            trip.onlyNode = junction;
            trip.prohibitedEdge = prevLane >= 0 ? net.lanes[prevLane].edge : -1;
            trip.edgeFactor = prefFactor(persons[p.person].type);
            trip.randomFactor = opt.weightsRandomFactor;
            trip.randomSeed = opt.seed;
            trip.randomKey = ((uint64_t)p.person << 24) ^ (uint64_t)now ^ ((uint64_t)currentEdge << 40);
            const std::vector<int> crossingRoute = pedRouter.compute(net, trip);
            if (p.person == tracePerson) {
                std::fprintf(stderr, "%.2f route %s:", STEPS2TIME(now), persons[p.person].id.c_str());
                for (int e : crossingRoute) {
                    std::fprintf(stderr, " %s", net.edges[e].id.c_str());
                }
                std::fprintf(stderr, " (prohibited %s)\n", trip.prohibitedEdge >= 0 ? net.edges[trip.prohibitedEdge].id.c_str() : "-");
            }
            if (crossingRoute.size() > 1) {
                const int nextEdge = crossingRoute[1];
                nextLane = getSidewalk(net, nextEdge, svc);
                nextDir = connectedDirection(currentLane, nextLane);
                if (nextDir == PED_FORWARD) {
                    link = net.linkIdxTo(currentLane, nextLane);
                } else {
                    link = net.linkIdxTo(nextLane, currentLane);
                    if (net.edges[nextEdge].isCrossing() && net.links[link].tls < 0) {
                        const int oppositeWalkingArea = net.lanes[nextLane].logicalPred;
                        link = net.linkIdxTo(oppositeWalkingArea, nextLane);
                    }
                }
                if (isPriorityCrossing(nextLane)) {
                    registerCrossingApproach(p, nextLane, prevLane);
                }
            } else {
                // a direct connection (moving onto the walking area was the wrong choice)
                if (p.dir == PED_FORWARD) {
                    link = net.linkIdxTo(prevLane, nextRouteLane);
                } else {
                    link = net.linkIdxTo(nextRouteLane, prevLane);
                }
                if (link >= 0) {
                    nextLane = net.links[link].viaOrLane();
                } else {
                    nextLane = nextRouteLane;
                }
            }
        } else if (currentEdge == nextRouteE) {
            nextDir = -p.dir;
        } else {
            nextDir = p.dir;
            nextLane = getNextWalkingArea(currentLane, p.dir, link);
            if (nextLane < 0) {
                if (junction == NRE.toJ) {
                    nextDir = PED_BACKWARD;
                } else if (junction == NRE.fromJ) {
                    nextDir = PED_FORWARD;
                } else {
                    std::vector<int> futureRoute(stage.route.begin() + stage.routeStep + 1, stage.route.end());
                    int passedFwd = 0;
                    int passedBwd = 0;
                    canTraverse(PED_FORWARD, futureRoute, passedFwd);
                    canTraverse(PED_BACKWARD, futureRoute, passedBwd);
                    nextDir = (passedFwd >= passedBwd) ? PED_FORWARD : PED_BACKWARD;
                }
                // direct links (tl-controlled links without crossings)
                if (p.dir == PED_FORWARD) {
                    link = net.linkIdxTo(currentLane, nextRouteLane);
                    if (link >= 0) {
                        nextLane = internalFollowingLane(currentLane, nextRouteLane);
                    }
                } else {
                    link = net.linkIdxTo(nextRouteLane, currentLane);
                    if (link >= 0) {
                        nextLane = internalFollowingLane(nextRouteLane, currentLane);
                        if (nextLane >= 0) {
                            while (net.lanes[net.links[net.lanes[nextLane].links[0]].viaOrLane()].internal) {
                                nextLane = net.links[net.lanes[nextLane].links[0]].viaOrLane();
                            }
                        }
                    }
                }
            }
            if (nextLane < 0) {
                nextLane = nextRouteLane;
            } else if (net.lanes[nextLane].length <= POSITION_EPS) {
                // internal lane too short (zero-size junction) unless a crossing must be passed
                const int succ = net.lanes[nextLane].canonicalSucc;
                const int pred = net.lanes[nextLane].logicalPred;
                if ((succ < 0 || !net.lanes[succ].crossing) && (pred < 0 || !net.lanes[pred].crossing)) {
                    nextLane = nextRouteLane;
                    nextDir = NRE.fromJ == junction ? PED_FORWARD : PED_BACKWARD;
                }
            }
        }
    }
    PedNextLane r;
    r.lane = nextLane;
    r.link = link;
    r.dir = nextDir;
    return r;
}

// MSLane::getInternalFollowingLane
int Sim::internalFollowingLane(int from, int target) const {
    for (int l : net.lanes[from].links) {
        if (net.links[l].lane == target) {
            return net.links[l].via;
        }
    }
    return -1;
}

bool Sim::isPriorityCrossing(int lane) const {
    if (lane < 0 || !net.lanes[lane].crossing) {
        return false;
    }
    return net.links[net.lanes[lane].incoming[0].link].offState == LS_MAJOR;
}

// MSPModel_Striping::registerCrossingApproach
void Sim::registerCrossingApproach(const PState& p, int crossing, int beforeWA) {
    Time arrivalTime = now;
    const int wap = getWalkingAreaPath(net.lanes[p.lane].edge, beforeWA, crossing);
    const double speed = walkMaxSpeed(p.person, walkStage(p)) * (1 - striping.dawdling / 2);
    arrivalTime += TIME2STEPS(waPaths[wap].length / speed);
    const Time leavingTime = arrivalTime + TIME2STEPS(net.lanes[crossing].length / speed);
    // std::map::emplace keeps an existing registration
    PedEvent ev;
    ev.kind = PedEvent::REGISTER;
    ev.person = p.person;
    ev.a = net.lanes[crossing].incoming[0].link;
    ev.arrivalTime = arrivalTime;
    ev.leavingTime = leavingTime;
    if (tlPedEvents != nullptr) {
        tlPedEvents->push_back(ev);
    } else {
        applyPedEvent(ev);
    }
}

void Sim::unregisterCrossingApproach(int person, int crossing) {
    PedEvent ev;
    ev.kind = PedEvent::UNREGISTER;
    ev.person = person;
    ev.a = net.lanes[crossing].incoming[0].link;
    if (tlPedEvents != nullptr) {
        tlPedEvents->push_back(ev);
    } else {
        applyPedEvent(ev);
    }
}

// ---------------------------------------------------------------------------
// adding a pedestrian (MSPModel_Striping::add, PState constructor)
// ---------------------------------------------------------------------------
void Sim::pedAdd(int pi) {
    Person& person = persons[pi];
    PStage& stage = person.plan[person.step];
    const int edge = stage.route[0];
    const SVCPermissions svc = types[person.type].vclass;
    // MSStageMoving::checkDepartLane
    int lane = getSidewalk(net, edge, svc);
    if (stage.departLane > 0) {
        const std::vector<int>& lanes = net.edges[edge].lanes;
        if ((int)lanes.size() <= stage.departLane || !net.lanes[lanes[stage.departLane]].allows(svc)) {
            throw std::runtime_error("invalid departLane for person " + person.id);
        }
        lane = lanes[stage.departLane];
    }
    if (lane < 0) {
        throw std::runtime_error("person " + person.id + " could not find sidewalk on edge " + net.edges[edge].id);
    }
    pstates.emplace_back();
    const int psi = (int)pstates.size() - 1;
    PState& p = pstates[psi];
    p.person = pi;
    p.lane = lane;
    p.edgePos = stage.departPos;
    p.posLat = stage.departPosLat;
    p.waitingToEnter = true;
    stage.pstate = psi;
    const std::vector<int>& route = stage.route;
    p.dir = PED_FORWARD;
    if (route.size() == 1) {
        p.dir = (p.edgePos <= stage.arrivalPos) ? PED_FORWARD : PED_BACKWARD;
    } else if (!net.edges[route.front()].isNormal()) {
        if (net.edges[route.front()].isWalkingArea()) {
            p.wap = getArbitraryPath(route.front());
        }
    } else {
        int passedFwd = 0;
        int passedBwd = 0;
        const bool mayStartForward = canTraverse(PED_FORWARD, route, passedFwd) != PED_UNDEFINED;
        const bool mayStartBackward = canTraverse(PED_BACKWARD, route, passedBwd) != PED_UNDEFINED;
        if (mayStartForward && mayStartBackward) {
            PedRouter::Trip trip;
            trip.from = edge;
            trip.to = route.back();
            trip.departPos = p.edgePos;
            trip.arrivalPos = stage.arrivalPos;
            trip.speed = walkMaxSpeed(pi, stage);
            trip.edgeFactor = prefFactor(persons[pi].type);
            trip.randomFactor = opt.weightsRandomFactor;
            trip.randomSeed = opt.seed;
            trip.randomKey = ((uint64_t)pi << 24) ^ (uint64_t)now ^ ((uint64_t)edge << 40);
            const std::vector<int> crossingRoute = pedRouter.compute(net, trip);
            if (crossingRoute.size() > 1) {
                const int nextEdge = crossingRoute[1];
                if (net.edges[nextEdge].fromJ == net.edges[edge].fromJ || net.edges[nextEdge].toJ == net.edges[edge].fromJ) {
                    p.dir = PED_BACKWARD;
                }
            }
        } else if (!mayStartForward && !mayStartBackward) {
            p.dir = passedFwd >= passedBwd ? PED_FORWARD : PED_BACKWARD;
        } else {
            p.dir = !mayStartBackward ? PED_FORWARD : PED_BACKWARD;
        }
    }
    const double sw = striping.stripeWidth;
    if (p.posLat == UNSPECIFIED_POS_LAT || striping.legacyPosLat) {
        if (p.posLat == UNSPECIFIED_POS_LAT) {
            p.posLat = 0;
        }
        if (laneVehicleNumberWithPartials(lane) > 0 && p.posLat == 0) {
            p.posLat -= sw;
        }
        if (p.dir == PED_FORWARD || (net.lanes[lane].permissions & VC_ALL) != VC_PEDESTRIAN) {
            p.posLat = sw * (numStripes(lane) - 1) - p.posLat;
        }
    } else if (p.posLat == RANDOM_POS_LAT) {
        p.posLat = rand01(opt.seed, (uint64_t)pi, (uint64_t)person.step, RP_PERSON_POSLAT) * sw * (numStripes(lane) - 1);
    } else {
        p.posLat = .5 * (net.lanes[lane].width - sw) - p.posLat;  // posLatConversion
    }
    p.nli = getNextLane(p, lane, -1);
    pedLaneAdd(lane, psi);
    nPedsActive++;
}

void Sim::pedLaneAdd(int lane, int psi) {
    pedsOnLane[lane].push_back(psi);
    pedActiveLanes.insert(lane);
}

int Sim::laneVehicleNumberWithPartials(int lane) const {
    if (lane == alightLane) {
        return alightLaneVehicles;
    }
    return (int)lanes[lane].vehs.size() + (int)lanes[lane].partials.size();
}

// ---------------------------------------------------------------------------
// obstacles
// ---------------------------------------------------------------------------
std::vector<PedObstacle> Sim::getNeighboringObstacles(const std::vector<PState*>& peds, int egoIndex, int stripes) const {
    const PState& ego = *peds[egoIndex];
    const int egoStripe = pedStripe(ego);
    std::vector<PedObstacle> obs(stripes, noObstacle(ego.dir));
    std::vector<bool> haveBlocker(stripes, false);
    for (int index = egoIndex + 1; index < (int)peds.size(); index++) {
        const PState& p = *peds[index];
        if (!p.waitingToEnter && !p.jammed) {
            const PedObstacle o = obstacleOf(p);
            const double d = distanceTo(ego, o, true);
            if (d == DIST_BEHIND) {
                break;
            }
            const int ps = pedStripe(p);
            const int pos = pedOtherStripe(p);
            if (d == DIST_OVERLAP) {
                if (ps != egoStripe || p.dir != ego.dir) {
                    obs[ps] = o;
                    haveBlocker[ps] = true;
                }
                if (pos != egoStripe || p.dir != ego.dir) {
                    obs[pos] = o;
                    haveBlocker[pos] = true;
                }
            } else {
                if (!haveBlocker[ps]) {
                    obs[ps] = o;
                }
                if (!haveBlocker[pos]) {
                    obs[pos] = o;
                }
            }
        }
    }
    return obs;
}

int Sim::getStripeOffset(int origStripes, int destStripes, bool addRemainder) {
    int offset = (destStripes - origStripes) / 2;
    if (addRemainder) {
        offset += (destStripes - origStripes) % 2;
    }
    return offset;
}

void Sim::transformToCurrentLanePositions(std::vector<PedObstacle>& obs, int currentDir, int nextDir, double currentLength, double nextLength) {
    for (PedObstacle& o : obs) {
        if (currentDir == PED_FORWARD) {
            if (nextDir == PED_FORWARD) {
                o.xFwd += currentLength;
                o.xBack += currentLength;
            } else {
                const double tmp = o.xFwd;
                o.xFwd = currentLength + nextLength - o.xBack;
                o.xBack = currentLength + nextLength - tmp;
            }
        } else {
            if (nextDir == PED_FORWARD) {
                const double tmp = o.xFwd;
                o.xFwd = -o.xBack;
                o.xBack = -tmp;
            } else {
                o.xFwd -= nextLength;
                o.xBack -= nextLength;
            }
        }
    }
}

void Sim::addCloserObstacle(std::vector<PedObstacle>& obs, double x, int stripe, int numStripes, int person, double width, int dir, int type) {
    if (stripe >= 0 && stripe < numStripes) {
        if ((dir == PED_FORWARD && x - width / 2. < obs[stripe].xBack) || (dir == PED_BACKWARD && x + width / 2. > obs[stripe].xFwd)) {
            PedObstacle o = explicitObstacle(x, 0, type, width);
            o.person = person;
            obs[stripe] = o;
        }
    }
}

// the pedestrians of a lane sorted by position in direction dir (by_xpos_sorter)
void Sim::sortPeds(std::vector<int>& v, int dir) const {
    std::sort(v.begin(), v.end(), [&](int a, int b) {
        const PState& p1 = pstates[a];
        const PState& p2 = pstates[b];
        if (p1.edgePos != p2.edgePos) {
            return dir * p1.edgePos > dir * p2.edgePos;
        }
        return persons[p1.person].id < persons[p2.person].id;
    });
}

void Sim::sortPedPtrs(std::vector<PState*>& v, int dir) const {
    std::sort(v.begin(), v.end(), [&](const PState* p1, const PState* p2) {
        if (p1->edgePos != p2->edgePos) {
            return dir * p1->edgePos > dir * p2->edgePos;
        }
        return pedID(*p1) < pedID(*p2);
    });
}

const std::string& Sim::pedID(const PState& p) const {
    return p.vehicle >= 0 ? veh[p.vehicle].id : persons[p.person].id;
}

const std::vector<PedObstacle>& Sim::getNextLaneObstacles(std::map<int, std::vector<PedObstacle>>& nextLanesObs, int lane, int nextLane,
                                                          int stripes, int nextDir, double currentLength, int currentDir) {
    auto found = nextLanesObs.find(nextLane);
    if (found != nextLanesObs.end()) {
        return found->second;
    }
    const double sw = striping.stripeWidth;
    const double nextLength = net.lanes[nextLane].walkingArea ? minNextLength[nextLane] : net.lanes[nextLane].length;
    const int nextStripes = numStripes(nextLane);
    std::vector<PedObstacle> obs(stripes, explicitObstacle(nextDir == PED_FORWARD ? nextLength : 0, 0, OBSTACLE_NEXTEND));
    const int offset = getStripeOffset(nextStripes, stripes, currentDir != nextDir && nextStripes > stripes);
    if (nextStripes < stripes) {
        for (int ii = 0; ii < stripes; ++ii) {
            if (ii < offset || ii >= nextStripes + offset) {
                obs[ii] = explicitObstacle(nextDir == PED_FORWARD ? 0 : nextLength, 0, OBSTACLE_END);
            }
        }
    }
    // the pedestrians of the next lane: live, or in a parallel wave for a lane of the same wave the state at the
    // start of the wave (a copy of the list, since tasks may sort it concurrently)
    const bool fromSnap = pedParallel && pedInWave[nextLane] != 0;
    thread_local std::vector<int> nextPeds;
    std::vector<int>* pedsPtr = &pedsOnLane[nextLane];
    if (pedParallel) {
        nextPeds = fromSnap ? pedSnapOnLane[nextLane] : pedsOnLane[nextLane];
        pedsPtr = &nextPeds;
    }
    std::vector<int>& peds = *pedsPtr;
    auto stateOf = [&](int psi) -> const PState& { return fromSnap ? pedSnap[psi] : pstates[psi]; };
    if (net.lanes[nextLane].walkingArea) {
        transformToCurrentLanePositions(obs, currentDir, nextDir, currentLength, nextLength);
        double lateralOffset = (net.lanes[lane].width - sw) * 0.5;
        if ((stripes - nextStripes) % 2 != 0) {
            lateralOffset += 0.5 * sw;
        }
        for (int psi : peds) {
            const PState& p = stateOf(psi);
            if (p.waitingToEnter || p.jammed) {
                continue;
            }
            const Pos pPos = pedPosition(p);
            const Pos relPos = geom::transformToVectorCoordinates(net.lanes[lane].shape, pPos, true);
            const double newY = relPos.y + lateralOffset;
            if ((currentDir == PED_FORWARD && relPos.x >= net.lanes[lane].length) || (currentDir == PED_BACKWARD && relPos.x < 0)) {
                const double w = types[persons[p.person].type].width;
                addCloserObstacle(obs, relPos.x, stripeOf(newY), stripes, p.person, w, currentDir, OBSTACLE_PED);
                addCloserObstacle(obs, relPos.x, otherStripeOf(p, newY), stripes, p.person, w, currentDir, OBSTACLE_PED);
            }
        }
    } else {
        // by_xpos_sorter
        std::sort(peds.begin(), peds.end(), [&](int a, int b) {
            const PState& p1 = stateOf(a);
            const PState& p2 = stateOf(b);
            if (p1.edgePos != p2.edgePos) {
                return nextDir * p1.edgePos > nextDir * p2.edgePos;
            }
            return persons[p1.person].id < persons[p2.person].id;
        });
        for (int psi : peds) {
            const PState& p = stateOf(psi);
            if (p.waitingToEnter || p.jammed) {
                continue;
            }
            double newY = p.posLat;
            PedObstacle pObs = obstacleOf(p);
            if (nextDir != currentDir) {
                newY = (nextStripes - 1) * sw - newY;
                pObs.speed *= -1;
            }
            newY += offset * sw;
            const int stripe = stripeOf(newY);
            if (stripe >= 0 && stripe < stripes) {
                obs[stripe] = pObs;
            }
            const int otherStripe = otherStripeOf(p, newY);
            if (otherStripe >= 0 && otherStripe < stripes) {
                obs[otherStripe] = pObs;
            }
        }
        if (net.lanes[nextLane].crossing) {
            const int crossingEntryLink = net.lanes[nextLane].incoming.front().link;
            const bool prio = net.links[crossingEntryLink].havePriority() || net.links[crossingEntryLink].tls >= 0;
            addCrossingVehs(nextLane, stripes, offset, nextDir, obs, prio, currentDir != nextDir);
        }
        if (laneVehicleNumberWithPartials(nextLane) > 0) {
            std::vector<PedObstacle> vehObs = getVehicleObstacles(nextLane, nextDir, nullptr);
            mergeObstaclesDir(obs, vehObs, nextDir, offset);
        }
        transformToCurrentLanePositions(obs, currentDir, nextDir, currentLength, nextLength);
    }
    return nextLanesObs[nextLane] = obs;
}

// ---------------------------------------------------------------------------
// movement
// ---------------------------------------------------------------------------
// MSPModel_Striping::MovePedestrians: all pedestrians walking forward, then all walking backward.
//
// --sumo-order: SUMO's sequential processing (lanes by numerical id, persons that changed the lane in this
// step are only obstacles). Otherwise each direction runs in three waves, crossings, walking areas and the
// other lanes, which is the order of their numerical ids in a SUMO network. The lanes of a wave move in
// parallel: a lane task changes only the pedestrians on its lane, reads the pedestrians of other lanes of the
// wave from the state at the start of the wave (lanes of other waves are not changed meanwhile) and records
// its effects on other lanes and on shared data (lane changes, finished walks, crossing approaches) as events,
// which are applied in lane order after the wave. Pedestrians thus see the lanes ahead as in SUMO (already
// moved if processed earlier); the result does not depend on the thread count.
void Sim::movePedestrians() {
    if (nPedsActive == 0 && pedActiveLanes.empty()) {
        return;
    }
    std::fill(pedChanged.begin(), pedChanged.end(), 0);
    moveInDirection(PED_FORWARD);
    moveInDirection(PED_BACKWARD);
}

void Sim::moveInDirection(int dir) {
    if (opt.sumoOrder) {
        // myActiveLanes is a map ordered by the lanes' numerical ids; lanes added during the
        // iteration are visited as well when their id is larger
        for (auto it = pedActiveLanes.begin(); it != pedActiveLanes.end(); ++it) {
            moveLaneInDirection(*it, dir);
        }
        return;
    }
    for (int wave = 0; wave < 3; ++wave) {
        std::vector<int> active;
        for (int lane : pedActiveLanes) {
            const Lane& L = net.lanes[lane];
            const int w = L.crossing ? 0 : (L.walkingArea ? 1 : 2);
            if (w == wave && !pedsOnLane[lane].empty()) {
                active.push_back(lane);
            }
        }
        if (active.empty()) {
            continue;
        }
        // the state at the start of the wave, read by the tasks of other lanes of the wave
        if (pedSnap.size() < pstates.size()) {
            pedSnap.resize(pstates.size());
        }
        for (int lane : active) {
            pedSnapOnLane[lane] = pedsOnLane[lane];
            pedInWave[lane] = 1;
            for (int psi : pedsOnLane[lane]) {
                pedSnap[psi] = pstates[psi];
            }
        }
        pedEventsOfTask.resize(active.size());
        pedParallel = true;
        pool.setTag(ThreadPool::T_PED);
        pool.parallel_for((int)active.size(), [&](int i) {
            tlPedEvents = &pedEventsOfTask[i];
            tlPedEvents->clear();
            moveLaneInDirection(active[i], dir);
            tlPedEvents = nullptr;
        }, 1);
        pedParallel = false;
        for (size_t i = 0; i < active.size(); ++i) {
            pedSnapOnLane[active[i]].clear();
            pedInWave[active[i]] = 0;
        }
        for (size_t i = 0; i < active.size(); ++i) {
            for (const PedEvent& ev : pedEventsOfTask[i]) {
                applyPedEvent(ev);
            }
        }
    }
}

void Sim::applyPedEvent(const PedEvent& ev) {
    switch (ev.kind) {
        case PedEvent::REGISTER: {
            auto& v = approachingPersons[ev.a];
            bool found = false;
            for (const PersonApproach& a : v) {
                found = found || a.person == ev.person;
            }
            if (!found) {
                v.push_back({ev.person, ev.arrivalTime, ev.leavingTime});
            }
            break;
        }
        case PedEvent::UNREGISTER: {
            auto& v = approachingPersons[ev.a];
            v.erase(std::remove_if(v.begin(), v.end(), [&](const PersonApproach& a) { return a.person == ev.person; }), v.end());
            break;
        }
        case PedEvent::ADD:
            pedChanged[ev.person] = 1;
            pedLaneAdd(ev.a, ev.psi);
            break;
        case PedEvent::FINISH:
            nPedsActive--;
            stageMoveToNextEdge(ev.person, ev.a, -1);
            break;
    }
}

void Sim::moveLaneInDirection(int lane, int dir) {
    {
        std::vector<int>& peds = pedsOnLane[lane];
        if (peds.empty()) {
            return;
        }
        if (net.lanes[lane].walkingArea) {
            const double sw = striping.stripeWidth;
            const double lateralOffset = (net.lanes[lane].width - sw) * 0.5;
            const double minY = sw * -0.5 + NUMERICAL_EPS;
            const double maxY = sw * (numStripes(lane) - 0.5) - NUMERICAL_EPS;
            // walking area paths in the order of (from, to) lane ids (walkingarea_path_sorter)
            std::set<std::pair<int, int>> pathKeys;
            for (int psi : peds) {
                const PState& p = pstates[psi];
                if (p.dir == dir) {
                    pathKeys.insert({waPaths[p.wap].from, waPaths[p.wap].to});
                }
            }
            const double usableWidth = (numStripes(lane) - 1) * sw;
            for (const auto& key : pathKeys) {
                const int path = waPathIdx.at(key);
                const WalkingAreaPath& P = waPaths[path];
                std::deque<PState> tmp;  // transformed copies
                std::vector<PState*> transformed;
                transformed.reserve(peds.size());
                for (int psi : peds) {
                    PState& p = pstates[psi];
                    const WalkingAreaPath& PP = waPaths[p.wap];
                    if (p.wap == path) {
                        transformed.push_back(&p);
                    } else if (PP.from == P.to && PP.to == P.from) {
                        if (PP.dir != P.dir) {
                            transformed.push_back(&p);
                        } else {
                            tmp.push_back(p);
                            PState& tp = tmp.back();
                            // PState::reverse
                            tp.edgePos = P.length - tp.edgePos;
                            tp.posLat = usableWidth - tp.posLat;
                            tp.dir = -PP.dir;
                            tp.speedLat = -tp.speedLat;
                            transformed.push_back(&tp);
                        }
                    } else {
                        const Pos relPos = geom::transformToVectorCoordinates(P.shape, pedPosition(p), false);
                        const double newY = relPos.y + lateralOffset;
                        if (relPos.valid() && newY >= minY && newY <= maxY) {
                            tmp.push_back(p);
                            PState& tp = tmp.back();
                            // PState::reset
                            tp.edgePos = relPos.x;
                            tp.posLat = newY;
                            tp.dir = PED_UNDEFINED;
                            tp.speed = 0.;
                            tp.speedLat = 0.;
                            transformed.push_back(&tp);
                        }
                    }
                }
                addWalkingAreaVehicleFoes(lane, P, dir, lateralOffset, minY, maxY, tmp, transformed);
                moveInDirectionOnLane(transformed, lane, dir);
                arriveAndAdvance(lane, dir);
            }
        } else {
            std::vector<PState*> ptrs;
            ptrs.reserve(peds.size());
            for (int psi : peds) {
                ptrs.push_back(&pstates[psi]);
            }
            moveInDirectionOnLane(ptrs, lane, dir);
            arriveAndAdvance(lane, dir);
        }
    }
}

void Sim::arriveAndAdvance(int lane, int dir) {
    std::vector<int>& peds = pedsOnLane[lane];
    sortPeds(peds, dir);
    for (int i = 0; i < (int)peds.size(); i++) {
        const int psi = peds[i];
        PState& p = pstates[psi];
        if (p.dir == dir && distToLaneEnd(p) < 0) {
            peds.erase(peds.begin() + i);
            i--;
            const int person = p.person;
            pedMoveToNextLane(psi);
            PState& q = pstates[psi];
            if (q.lane < 0) {
                q.dead = true;  // end walking stage
            }
            PedEvent ev;
            ev.kind = q.lane >= 0 ? PedEvent::ADD : PedEvent::FINISH;
            ev.person = person;
            ev.psi = psi;
            ev.a = q.lane >= 0 ? q.lane : dir;
            if (tlPedEvents != nullptr) {
                tlPedEvents->push_back(ev);
            } else {
                applyPedEvent(ev);
            }
        }
    }
}

void Sim::moveInDirectionOnLane(std::vector<PState*>& peds, int lane, int dir) {
    const int stripes = numStripes(lane);
    std::vector<PedObstacle> obs(stripes, noObstacle(dir));
    std::map<int, std::vector<PedObstacle>> nextLanesObs;
    sortPedPtrs(peds, dir);
    std::vector<PedObstacle> crossingVehs(stripes, noObstacle(dir));
    bool hasCrossingVehObs = false;
    if (net.lanes[lane].crossing) {
        hasCrossingVehObs = addCrossingVehs(lane, stripes, 0, dir, crossingVehs, true, false);
    }
    for (int ii = 0; ii < (int)peds.size(); ++ii) {
        PState& p = *peds[ii];
        std::vector<PedObstacle> currentObs = obs;
        if (p.dir != dir || p.vehicle >= 0 || pedChanged[p.person] != 0) {
            if (!p.waitingToEnter && !p.jammed) {
                PedObstacle o = obstacleOf(p);
                if (p.dir != dir && p.speed == 0.) {
                    o.speed = (p.dir == PED_FORWARD ? 0.1 : -0.1);
                }
                const int s = pedStripe(p);
                const int os = pedOtherStripe(p);
                if (closer(o, obs[s], dir)) {
                    obs[s] = o;
                }
                if (closer(o, obs[os], dir)) {
                    obs[os] = o;
                }
            }
            continue;
        }
        const PStage& stage = walkStage(p);
        const int nextLane = p.nli.lane;
        const int link = p.nli.link;
        const double dist = distToLaneEnd(p);
        const double speed = stage.speed >= 0 ? stage.speed
                             : ((nextLane >= 0 && (striping.useNetSpeeds || net.lanes[nextLane].isNormal() || net.lanes[nextLane].internal))
                                ? laneMaxSpeedFor(nextLane, p.person)
                                : walkMaxSpeed(p.person, stage));
        if (nextLane >= 0 && dist <= LOOKAHEAD_ONCOMING_DIST) {
            const double currentLength = p.wap < 0 ? net.lanes[lane].length : waPaths[p.wap].length;
            const std::vector<PedObstacle>& nextObs = getNextLaneObstacles(nextLanesObs, lane, nextLane, stripes, p.nli.dir, currentLength, dir);
            mergeObstacles(p, currentObs, nextObs);
        }
        mergeObstacles(p, currentObs, getNeighboringObstacles(peds, ii, stripes));
        // time gap to pass the intersection ahead of a vehicle (MSPerson::getTimegapCrossing)
        const double tgc = types[persons[p.person].type].timegapCrossing;
        const double passingLength = pedLength(p) + (tgc >= 0 ? tgc : timegapCrossing) * speed;
        if (link >= 0 && dist - pedMinGap(p) < LOOKAHEAD_SAMEDIR * speed
                && (!opened(link, now - DELTA_T, speed, speed, passingLength, pedImpatience(p), speed, 0, -1, pedIgnoreRed(p, link),
                            nullptr, -1, p.person)
                    || pedStopForYellow(p, link))) {
            std::vector<PedObstacle> closedLink(stripes, explicitObstacle(p.edgePos + dir * (dist - NUMERICAL_EPS), 0, OBSTACLE_LINKCLOSED));
            mergeObstacles(p, currentObs, closedLink);
            // consider rerouting over another crossing
            if (p.wap >= 0) {
                const int oldNext = p.nli.lane;
                p.nli = getNextLane(p, p.lane, waPaths[p.wap].from);
                if (p.nli.lane != oldNext && isPriorityCrossing(oldNext)) {
                    unregisterCrossingApproach(p.person, oldNext);
                }
            }
        }
        if (net.lanes[lane].edge == stage.destination && stage.destinationStop >= 0) {
            const BusStop& b = busStops[stage.destinationStop];
            std::vector<PedObstacle> arrival;
            if (b.waitingCapacity > b.numWaitingPersons || !b.checkPersonCapacity) {
                arrival.assign(stripes, explicitObstacle(stage.arrivalPos + dir * pedMinGap(p), 0, OBSTACLE_ARRIVALPOS));
            } else {
                arrival.assign(stripes, explicitObstacle(stage.arrivalPos - dir * pedMinGap(p), 0, OBSTACLE_ARRIVALPOS));
            }
            mergeObstacles(p, currentObs, arrival);
        }
        if (laneVehicleNumberWithPartials(lane) > 0) {
            std::vector<PedObstacle> vehObs = getVehicleObstacles(lane, dir, &p);
            mergeObstacles(p, currentObs, vehObs);
        }
        if (hasCrossingVehObs) {
            mergeObstacles(p, currentObs, crossingVehs);
        }
        if (p.person == tracePerson) {
            std::fprintf(stderr, "%.2f ped %s lane=%s pos=%.3f lat=%.3f dir=%d next=%s link=%d dist=%.3f obs:", STEPS2TIME(now),
                         persons[p.person].id.c_str(), net.lanes[lane].id.c_str(), p.edgePos, p.posLat, p.dir,
                         nextLane >= 0 ? net.lanes[nextLane].id.c_str() : "-", link, dist);
            for (const PedObstacle& o : currentObs) {
                std::fprintf(stderr, " [%.2f,%.2f v=%.2f t=%d]", o.xBack, o.xFwd, o.speed, o.type);
            }
            std::fprintf(stderr, "\n");
        }
        pedWalk(p, currentObs);
        if (!p.waitingToEnter && !p.jammed) {
            const PedObstacle o = obstacleOf(p);
            const int s = pedStripe(p);
            const int os = pedOtherStripe(p);
            if (closer(o, obs[s], dir)) {
                obs[s] = o;
            }
            if (closer(o, obs[os], dir)) {
                obs[os] = o;
            }
        }
    }
}

int Sim::getReserved(int stripes, double factor) const {
    return std::min((int)std::floor(stripes * factor), (int)std::floor(striping.reserveOncomingMax / striping.stripeWidth));
}

// PState::walk
void Sim::pedWalk(PState& p, const std::vector<PedObstacle>& obs) {
    const int stripes = (int)obs.size();
    const int sMax = stripes - 1;
    const PStage& stage = walkStage(p);
    const Lane& L = net.lanes[p.lane];
    const double sw = striping.stripeWidth;
    const double vMax = stage.speed >= 0 ? stage.speed
                        : (striping.useNetSpeeds || L.isNormal() || L.internal ? laneMaxSpeedFor(p.lane, p.person) : walkMaxSpeed(p.person, stage));
    const int current = pedStripe(p);
    const int other = pedOtherStripe(p);
    std::vector<double> distance(stripes);
    for (int i = 0; i < stripes; ++i) {
        distance[i] = distanceTo(p, obs[i], obs[i].type == OBSTACLE_PED);
    }
    std::vector<double> utility(stripes, 0);
    for (int i = 0; i < stripes; ++i) {
        if (distance[i] == DIST_OVERLAP) {
            if (i == current && (!p.waitingToEnter || pedStripe(p) != stripeOf(p.posLat))) {
                utility[i] += OBSTRUCTED_PENALTY;
            }
            if (i < current) {
                for (int j = 0; j <= i; ++j) {
                    utility[j] += OBSTRUCTED_PENALTY;
                }
            }
            if (i > current) {
                for (int j = i; j < stripes; ++j) {
                    utility[j] += OBSTRUCTED_PENALTY;
                }
            }
        }
    }
    const bool onJunction = L.walkingArea || L.crossing;
    const int reserved = getReserved(stripes, onJunction ? striping.reserveOncomingJunctions : striping.reserveOncoming);
    if (p.dir == PED_FORWARD) {
        for (int i = 0; i < reserved; ++i) {
            utility[i] += INAPPROPRIATE_PENALTY * (i == current ? 0.5 : 1);
        }
    } else {
        for (int i = sMax; i > sMax - reserved; --i) {
            utility[i] += INAPPROPRIATE_PENALTY * (i == current ? 0.5 : 1);
        }
    }
    for (int i = 0; i < stripes; ++i) {
        if (obs[i].speed * p.dir < 0) {
            if ((p.dir == PED_FORWARD || obs[i].type == OBSTACLE_VEHICLE) && i > 0) {
                utility[i - 1] -= 0.5;
            } else if (p.dir == PED_BACKWARD && i < sMax) {
                utility[i + 1] -= 0.5;
            }
        }
        const double walkDist = std::max(0., distance[i]);
        const double lookAhead = obs[i].speed * p.dir >= 0 ? LOOKAHEAD_SAMEDIR : LOOKAHEAD_ONCOMING;
        const double expectedDist = std::min(vMax * LOOKAHEAD_SAMEDIR, walkDist + obs[i].speed * p.dir * lookAhead);
        if (expectedDist >= 0) {
            utility[i] += expectedDist;
        } else {
            utility[i] += ONCOMING_CONFLICT_PENALTY + distance[i];
        }
    }
    if (p.dir == PED_FORWARD && obs[0].speed < 0) {
        utility[0] += ONCOMING_CONFLICT_PENALTY;
    } else if (p.dir == PED_BACKWARD && obs[sMax].speed > 0) {
        utility[sMax] += ONCOMING_CONFLICT_PENALTY;
    }
    if (distance[current] > 0 && p.waitingTime == 0) {
        for (int i = 0; i < stripes; ++i) {
            utility[i] += std::abs(i - current) * LATERAL_PENALTY;
        }
    }
    if ((L.permissions & VC_ALL) != VC_PEDESTRIAN && p.dir == PED_BACKWARD) {
        for (int i = 0; i < stripes; ++i) {
            if (i <= current) {
                utility[i] += (sMax - i + 1) * LATERAL_PENALTY;
            }
        }
    }
    int chosen = current;
    for (int i = 0; i < stripes; ++i) {
        if (utility[i] > utility[chosen] && utility[i] >= INAPPROPRIATE_PENALTY * 0.5) {
            chosen = i;
        }
    }
    const int next = (chosen == current ? current : (chosen < current ? current - 1 : current + 1));
    double xDist = std::min(std::min(distance[current], distance[other]), distance[next]);
    if (next != chosen) {
        const int nextOther = chosen < current ? current - 2 : current + 2;
        xDist = std::min(xDist, distance[nextOther]);
    }
    const double preferredGap = NUMERICAL_EPS;
    double xSpeed = std::min(vMax, std::max(0., DIST2SPEED(xDist - preferredGap)));
    if (xSpeed < NUMERICAL_EPS) {
        xSpeed = 0.;
    }
    if (p.speed == 0 && xDist < MIN_STARTUP_DIST
            && !((xDist == distance[current] && obs[current].type >= OBSTACLE_END)
                 || (xDist == distance[other] && obs[other].type >= OBSTACLE_END)
                 || (xDist == distance[next] && obs[next].type >= OBSTACLE_END))) {
        xSpeed = 0;
    }
    if (xSpeed == 0) {
        const bool sharedWA = L.walkingArea && obs[current].vehicle >= 0 && veh[obs[current].vehicle].waitingTime > striping.jamTimeCrossing
                              && !waFoes[L.edge].empty();
        if (p.waitingTime > ((L.crossing || sharedWA) ? striping.jamTimeCrossing : striping.jamTime)
                || (sMax == 0 && obs[0].speed * p.dir < 0 && p.waitingTime > striping.jamTimeNarrow)
                || p.jammed) {
            if (!p.jammed) {
                __atomic_add_fetch(&nPedsJammed, 1, __ATOMIC_RELAXED);
                p.jammed = true;
            }
            xSpeed = vMax * striping.jamFactor;
        }
    } else if (p.jammed && stripeOf(p.posLat) >= 0 && stripeOf(p.posLat) <= sMax && xDist >= MIN_STARTUP_DIST) {
        p.jammed = false;
    }
    // dawdling
    const double dawdle = std::min(xSpeed, rand01(opt.seed, (uint64_t)p.person, (uint64_t)step, RP_PED_DAWDLE) * vMax * striping.dawdling);
    xSpeed -= dawdle;
    const double maxYSpeed = std::min(std::max(vMax * LATERAL_SPEED_FACTOR, vMax - xSpeed), sw);
    double ySpeed = 0;
    if (utility[next] > OBSTRUCTION_THRESHOLD && utility[chosen] > OBSTRUCTION_THRESHOLD) {
        const double yDist = (chosen * sw) - p.posLat;
        if (std::fabs(yDist) > NUMERICAL_EPS) {
            ySpeed = (yDist > 0 ? std::min(maxYSpeed, DIST2SPEED(yDist)) : std::max(-maxYSpeed, DIST2SPEED(yDist)));
        }
    } else if (utility[next] <= OBSTRUCTION_THRESHOLD && obs[next].type == OBSTACLE_VEHICLE
               && pedStripe(p) == stripeOf(p.posLat)
               && !(L.crossing || L.walkingArea)) {
        int stepAsideDir = p.dir;
        if (net.edges[L.edge].lanes.size() > 1 || current > sMax / 2) {
            stepAsideDir = PED_FORWARD;
        }
        p.jammed = true;
        ySpeed = stepAsideDir * vMax;
    }
    p.edgePos += SPEED2DIST(xSpeed * p.dir);
    p.posLat += SPEED2DIST(ySpeed);
    p.speedLat = ySpeed;
    p.speed = xSpeed;
    if (xSpeed >= HALTING_SPEED) {
        p.waitingToEnter = false;
        p.waitingTime = 0;
    } else {
        p.waitingTime += DELTA_T;
        p.totalWaitingTime += DELTA_T;
    }
    p.angle = std::numeric_limits<double>::max();
}

// PState::moveToNextLane (called when distToLaneEnd < 0)
void Sim::pedMoveToNextLane(int psi) {
    PState& p = pstates[psi];
    double dist = distToLaneEnd(p);
    const int oldDir = p.dir;
    const int oldLane = p.lane;
    p.lane = p.nli.lane;
    p.dir = p.nli.dir;
    PStage& stage = walkStageMut(p);
    const int nre = nextRouteEdge(stage);
    const bool normalLane = (p.lane < 0 || net.lanes[p.lane].isNormal() || net.lanes[p.lane].edge == nre);
    if (p.lane < 0) {
        p.edgePos = stage.arrivalPos;
    }
    if (stage.routeStep == (int)stage.route.size() - 1) {
        p.lane = -1;
    } else {
        stageMoveToNextEdge(p.person, oldDir, normalLane ? -1 : net.lanes[p.lane].edge);
        PState& q = pstates[psi];
        q.nli = getNextLane(q, q.lane, oldLane);
        if (net.lanes[q.lane].walkingArea) {
            if (q.nli.dir != PED_UNDEFINED) {
                q.wap = getWalkingAreaPath(net.lanes[q.lane].edge, oldLane, q.nli.lane);
            } else if (q.nli.link >= 0) {
                // direct connection instead of the walking area
                q.lane = q.nli.lane;
                stageMoveToNextEdge(q.person, q.dir, net.lanes[q.lane].edge);
                q.wap = -1;
                q.nli = getNextLane(q, q.lane, oldLane);
            } else {
                throw std::runtime_error("disconnected walk for person " + persons[q.person].id);
            }
        } else {
            q.wap = -1;
        }
        const double newLength = q.wap < 0 ? net.lanes[q.lane].length : waPaths[q.wap].length;
        if (-dist > newLength) {
            dist = -newLength;
        }
        if (q.dir == PED_BACKWARD) {
            q.edgePos = newLength + dist;
        } else {
            q.edgePos = -dist;
        }
        if (q.dir != oldDir) {
            q.posLat = (numStripes(oldLane) - 1) * striping.stripeWidth - q.posLat;
        }
        const int offset = getStripeOffset(numStripes(oldLane), numStripes(q.lane), oldDir != q.dir && numStripes(q.lane) < numStripes(oldLane));
        q.posLat += offset * striping.stripeWidth;
    }
    pstates[psi].angle = std::numeric_limits<double>::max();
}

// MSStageWalking::moveToNextEdge; returns whether the walk is finished
bool Sim::stageMoveToNextEdge(int pi, int prevDir, int nextInternal) {
    Person& person = persons[pi];
    PStage& s = person.plan[person.step];
    edgeRemovePerson(pi);
    const bool arrived = s.routeStep == (int)s.route.size() - 1;
    s.lastEdgeEntryTime = now;
    if (s.currentInternalEdge >= 0) {
        const double pathLength = s.pstate >= 0 && pstates[s.pstate].wap >= 0 ? waPaths[pstates[s.pstate].wap].length : 0;
        s.internalDistance += (pathLength == 0 ? net.edges[s.currentInternalEdge].length : pathLength);
    }
    (void)prevDir;
    if (arrived) {
        if (s.destinationStop >= 0) {
            stopAddPerson(s.destinationStop, pi);
        }
        if (!personProceed(pi)) {
            personErase(pi);
        }
        return true;
    }
    if (nextInternal < 0) {
        ++s.routeStep;
    }
    s.currentInternalEdge = nextInternal;
    edgeAddPerson(stageEdge(s), pi);
    return false;
}

// ---------------------------------------------------------------------------
// vehicles as obstacles
// ---------------------------------------------------------------------------

// MSPModel_Striping::addCrossingVehs: vehicles approaching the crossing (leader info of the crossing's exit link)
bool Sim::addCrossingVehs(int crossing, int stripes, double lateralOffset, int dir, std::vector<PedObstacle>& obs, bool prio, bool flipY) {
    bool hasCrossingVehObs = false;
    const int crossingExitLink = net.lanes[crossing].links.front();
    thread_local std::vector<LinkLeader> linkLeaders;
    leaderInfo(crossingExitLink, -1, net.lanes[crossing].length, linkLeaders, false);
    if (linkLeaders.empty()) {
        return false;
    }
    const double crossingLength = net.lanes[crossing].length;
    for (const LinkLeader& ll : linkLeaders) {
        const int v = ll.veh;
        if (v < 0) {
            continue;
        }
        const VType& t = vt(v);
        PedObstacle vo = explicitObstacle(ll.distToCrossing, 0, OBSTACLE_VEHICLE, t.width + 2 * striping.minGapToVehicle, v);
        // block entry to the crossing in walking direction but allow leaving it
        PedObstacle voBlock = vo;
        if (dir == PED_FORWARD) {
            voBlock.xBack = NUMERICAL_EPS;
        } else {
            voBlock.xFwd = crossingLength - NUMERICAL_EPS;
        }
        // when approaching a priority crossing, vehicles must be able to brake, otherwise the person must be able
        // to cross in time
        const double distToCrossBeforeVeh = dir == PED_FORWARD ? vo.xFwd : crossingLength - vo.xBack;
        const double bGap = prio ? cfBrakeGap(v, kin[v].speed, t.decel, 0) : kin[v].speed * distToCrossBeforeVeh;  // walking 1m/s
        double vehYmin, vehYmax;
        // relY increases from left to right (the other way around from vehicles)
        if ((ll.flags & LL_FROM_LEFT) != 0) {
            vehYmin = -ll.gap + lateralOffset;  // vehicle back
            vehYmax = vehYmin + t.length + bGap + striping.minGapToVehicle;
            vehYmin -= striping.minGapToVehicle;
        } else {
            vehYmax = net.lanes[crossing].width + ll.gap - lateralOffset;  // vehicle back
            vehYmin = vehYmax - t.length - bGap - striping.minGapToVehicle;
            vehYmax += striping.minGapToVehicle;
        }
        for (int st = std::max(0, stripeOf(vehYmin)); st < std::min(stripeOf(vehYmax), stripes); ++st) {
            if ((dir == PED_FORWARD && obs[st].xBack > vo.xBack) || (dir == PED_BACKWARD && obs[st].xFwd < vo.xFwd)) {
                if (!prio && kin[v].speed > HALTING_SPEED) {
                    // do not enter the crossing
                    obs[st] = voBlock;
                } else {
                    obs[st] = vo;
                }
                hasCrossingVehObs = true;
            }
        }
        if (flipY) {
            std::reverse(obs.begin(), obs.end());
        }
    }
    if (hasCrossingVehObs) {
        // check whether the crossing is fully blocked
        const int reserved = getReserved((int)obs.size(), striping.reserveOncomingJunctions);
        bool allBlocked = true;
        for (int i = 0; i < (int)obs.size(); i++) {
            if (obs[i].type != OBSTACLE_VEHICLE
                    && ((dir == PED_FORWARD && i >= reserved) || (dir == PED_BACKWARD && i < (int)obs.size() - reserved))) {
                allBlocked = false;
                break;
            }
        }
        if (allBlocked) {
            for (PedObstacle& o : obs) {
                if (dir == PED_FORWARD) {
                    o.xBack = NUMERICAL_EPS;
                } else {
                    o.xFwd = crossingLength - NUMERICAL_EPS;
                }
            }
        }
    }
    return hasCrossingVehObs;
}

// MSPModel_Striping::getVehicleObstacles: vehicles on a lane shared with pedestrians
std::vector<PedObstacle> Sim::getVehicleObstacles(int lane, int dir, const PState* ped) {
    constexpr double LOOKAROUND_VEHICLES = 60.0;
    const int stripes = numStripes(lane);
    std::vector<PedObstacle> vehObs(stripes, noObstacle(dir));
    int current = -1;
    double minX = 0.;
    double maxX = 0.;
    double pRelY = -1.;
    double pWidth = 0.;
    if (ped != nullptr) {
        current = pedStripe(*ped);
        minX = pedMinX(*ped, true);
        maxX = pedMaxX(*ped, true);
        pRelY = ped->posLat;
        pWidth = types[persons[ped->person].type].width;
    } else if (dir == PED_BACKWARD) {
        // checking vehicles on the next lane. Use entry point as reference
        minX = net.lanes[lane].length;
        maxX = net.lanes[lane].length;
    }
    thread_local std::vector<int> vs;
    vs.clear();
    anyVehicles(lane, [&](int u) {
        vs.push_back(u);
        return true;
    });
    if (dir == PED_FORWARD) {
        // MSLane::anyVehiclesUpstreamBegin
        std::reverse(vs.begin(), vs.end());
    }
    for (int u : vs) {
        const VType& t = vt(u);
        const double vehBack = backPositionOnLane(u, lane);
        const double vehFront = vehBack + t.length;
        // ensure that vehicles are not blocked
        const double vehNextSpeed = veh[u].waitingTime > DELTA_T ? 0 : std::max(kin[u].speed, 1.0);
        const double clearance = PED_SAFETY_GAP + vehNextSpeed * LOOKAHEAD_SAMEDIR;
        const double vehXMax = vehFront + clearance;
        const double vehXMin = vehBack - PED_SAFETY_GAP;
        double vehXMaxCheck, vehXMinCheck;
        if (dir == PED_FORWARD) {
            vehXMaxCheck = vehFront + clearance;
            vehXMinCheck = vehBack - LOOKAHEAD_SAMEDIR;
        } else {
            vehXMaxCheck = vehFront + LOOKAROUND_VEHICLES;
            vehXMinCheck = vehBack - NUMERICAL_EPS;
        }
        // (SUMO tests vehXMinCheck != 0 as a condition of its own)
        if (vehXMaxCheck > minX && vehXMinCheck != 0 && vehXMinCheck <= maxX) {
            PedObstacle vo = explicitObstacle(vehBack, kin[u].speed, OBSTACLE_VEHICLE, 0, u);
            // moving vehicles block space along their path
            vo.xFwd = vehXMax;
            vo.xBack = vehXMin;
            // relY increases from left to right (the other way around from vehicles); lateral position 0
            const double vehYmax = 0.5 * (net.lanes[lane].width + t.width - striping.stripeWidth);
            const double vehYmin = vehYmax - t.width;
            for (int st = std::max(0, stripeOf(vehYmin)); st < std::min(stripeOf(vehYmax) + 1, stripes); ++st) {
                const PedObstacle prior = vehObs[st];
                vehObs[st] = vo;
                if (st == current && vehFront + PED_SAFETY_GAP < minX) {
                    // ignore if already overlapping while vehicle is still behind
                    if (pRelY - pWidth < vehYmax && pRelY + pWidth > vehYmin && dir == PED_FORWARD) {
                        vehObs[st] = prior;
                    }
                }
            }
        }
    }
    return vehObs;
}

// MSVehicle::computeAngle (lateral position 0, no lane changing maneuver)
double Sim::vehAngle(int v) const {
    const Pos p1 = vehPosition(v);
    // MSVehicle::computeAngle: an articulated vehicle (a train) has the heading of its first part
    const VType& t = vt(v);
    Pos p2 = t.locomotiveLength > 0 ? vehPositionAtOffset(v, -std::min(t.locomotiveLength, t.length)) : vehBackPosition(v);
    if (!p2.valid()) {
        // the back reaches out of the network: the start of the last further lane (or of the lane)
        const Lane& B = net.lanes[kin[v].nFurther > 0 ? kin[v].further[kin[v].nFurther - 1] : kin[v].lane];
        p2 = geom::positionAtOffset(B.shape, 0, 0);
    }
    if (p1 != p2) {
        return p2.angleTo2D(p1);
    }
    const Lane& L = net.lanes[kin[v].lane];
    return geom::rotationAtOffset(L.shape, L.interpolateLanePosToGeometryPos(kin[v].pos));
}

// MSVehicle::getPosition(offset) with validatePosition: behind the front, on the further lanes if needed
// (INVALID_POS beyond them)
Pos Sim::vehPositionAtOffset(int v, double offset) const {
    const Kin& k = kin[v];
    const Lane& L = net.lanes[k.lane];
    if (k.pos + offset >= 0) {
        return geom::positionAtOffset(L.shape, L.interpolateLanePosToGeometryPos(k.pos + offset), 0);
    }
    double lastLength = k.pos;
    for (int i = 0; i < k.nFurther; ++i) {
        const Lane& F = net.lanes[k.further[i]];
        offset += lastLength;
        if (F.length + offset >= 0) {
            return geom::positionAtOffset(F.shape, F.interpolateLanePosToGeometryPos(F.length + offset), 0);
        }
        lastLength = F.length;
    }
    return INVALID_POS;
}

// MSVehicle::getPosition / getBackPosition (lateral position 0, not parking)
// MSVehicle::getPosition (with the lateral position in sublane mode)
Pos Sim::vehPosition(int v) const {
    const Kin& k = KV(v);
    const Lane& L = net.lanes[k.lane];
    return geom::positionAtOffset(L.shape, L.interpolateLanePosToGeometryPos(k.pos), (net.lefthand ? 1 : -1) * k.posLat);
}

Pos Sim::vehBackPosition(int v) const {
    const Kin& k = kin[v];
    const double length = vt(v).length;
    if (k.pos >= length) {
        const Lane& L = net.lanes[k.lane];
        return geom::positionAtOffset(L.shape, L.interpolateLanePosToGeometryPos(k.pos - length), 0);
    }
    if (k.nFurther > 0) {
        // truncate to 0 if the vehicle starts on an edge that is shorter than its length
        const int back = k.further[k.nFurther - 1];
        const double backPos = std::max(0.0, backPositionOnLane(v, back));
        const Lane& L = net.lanes[back];
        return geom::positionAtOffset(L.shape, L.interpolateLanePosToGeometryPos(backPos), 0);
    }
    const Lane& L = net.lanes[k.lane];
    return geom::positionAtOffset(L.shape, 0, 0);
}

// vehicles on lanes crossing a shared walking area as dummy pedestrians (MSPModel_Striping::moveInDirection)
void Sim::addWalkingAreaVehicleFoes(int lane, const WalkingAreaPath& path, int dir, double lateralOffset, double minY, double maxY,
                                     std::deque<PState>& tmp, std::vector<PState*>& transformed) {
    const std::vector<int>& foes = waFoes[net.lanes[lane].edge];
    auto addVehicleFoe = [&](int u, const Pos& relPos, double xWidth, double yWidth) {
        if (!relPos.valid()) {
            return false;
        }
        const double newY = relPos.y + lateralOffset;
        if (newY >= minY && newY <= maxY) {
            tmp.emplace_back();
            PState& tp = tmp.back();
            tp.person = -1;
            tp.vehicle = u;
            tp.lane = lane;
            tp.xWidth = xWidth;
            tp.yWidth = yWidth;
            // relX is the center but we want it to be the max value if the movement direction is forward
            tp.edgePos = relPos.x + xWidth / 2;
            tp.posLat = newY;
            tp.dir = PED_UNDEFINED;
            tp.waitingToEnter = false;
            transformed.push_back(&tp);
        }
        return true;
    };
    for (int foeLane : foes) {
        anyVehicles(foeLane, [&](int u) {
            const double vehWidth = vt(u).width;
            const Pos relFront = geom::transformToVectorCoordinates(path.shape, vehPosition(u), true);
            const Pos relBack = geom::transformToVectorCoordinates(path.shape, vehBackPosition(u), true);
            Shape relCenter{relFront, relBack};
            geom::move2side2(relCenter, vehWidth / 2);
            geom::Boundary relCorners;
            relCorners.add(relCenter[0]);
            relCorners.add(relCenter[1]);
            geom::move2side2(relCenter, -vehWidth);
            relCorners.add(relCenter[0]);
            relCorners.add(relCenter[1]);
            // persons should require less gap than the vehicles to prevent getting stuck when a vehicles moves towards them
            relCorners.xmin -= PED_SAFETY_GAP / 2;
            relCorners.xmax += PED_SAFETY_GAP / 2;
            const double xWidth = relCorners.xmax - relCorners.xmin;
            const double vehYmin = std::max(minY - lateralOffset, relCorners.ymin);
            const double vehYmax = std::min(maxY - lateralOffset, relCorners.ymax);
            const double xCenter = (relCorners.xmin + relCorners.xmax) / 2;
            const Pos yMinPos{xCenter, vehYmin};
            const Pos yMaxPos{xCenter, vehYmax};
            const bool addFront = addVehicleFoe(u, yMinPos, dir * xWidth, 0);
            const bool addBack = addVehicleFoe(u, yMaxPos, dir * xWidth, 0);
            if (addFront && addBack) {
                // add in-between positions
                const double yDist = vehYmax - vehYmin;
                for (double d = striping.stripeWidth; d < yDist; d += striping.stripeWidth) {
                    const double relDist = d / yDist;
                    const Pos between{yMinPos.x * relDist + yMaxPos.x * (1 - relDist), yMinPos.y * relDist + yMaxPos.y * (1 - relDist)};
                    addVehicleFoe(u, between, dir * xWidth, striping.stripeWidth);
                }
            }
            return true;
        });
    }
}

// ---------------------------------------------------------------------------
// output
// ---------------------------------------------------------------------------
// one row per walk (MSStageWalking::tripInfoOutput)
void Sim::writePersonInfo(int pi) {
    const Person& p = persons[pi];
    for (int si = 0; si < (int)p.plan.size(); ++si) {
        const PStage& s = p.plan[si];
        if (s.kind != PStage::WALKING || s.departed < 0) {
            continue;
        }
        const double distance = walkDistance(p, s, true);
        const double maxSpeed = walkMaxSpeed(pi, s);
        const Time duration = s.arrived >= 0 ? s.arrived - s.departed : now - s.departed;
        Time timeLoss = s.arrived < 0 ? 0 : duration - TIME2STEPS(distance / maxSpeed);
        if (timeLoss < 0 && timeLoss > TIME2STEPS(-0.1)) {
            timeLoss = 0;
        }
        const Time waiting = s.pstate >= 0 ? pstates[s.pstate].totalWaitingTime : 0;
        char buf[512];
        std::snprintf(buf, sizeof(buf), "%s,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", p.id.c_str(), si, STEPS2TIME(s.departed),
                      s.departPos, s.arrived >= 0 ? STEPS2TIME(s.arrived) : -1., s.arrived >= 0 ? s.arrivalPos : -1.,
                      STEPS2TIME(duration), s.arrived >= 0 ? distance : -1., STEPS2TIME(timeLoss), maxSpeed, STEPS2TIME(waiting));
        personRows.push_back(buf);
    }
}

// persons in fcd output: walking persons at their striping position, waiting persons at the roadside
void Sim::writePersonFcd() {
    std::vector<int> ps;
    for (int pi = 0; pi < (int)persons.size(); ++pi) {
        if (persons[pi].departed && !persons[pi].arrived && !persons[pi].container) {
            ps.push_back(pi);
        }
    }
    std::sort(ps.begin(), ps.end(), [&](int a, int b) { return persons[a].id < persons[b].id; });
    for (int pi : ps) {
        const Person& q = persons[pi];
        const PStage& s = q.plan[q.step];
        if (s.kind == PStage::DRIVING && s.vehicle >= 0 && veh[s.vehicle].status != VehStatus::RUNNING
                && veh[s.vehicle].status != VehStatus::PARKING) {
            continue;  // (riders of vehicles that are not on the road or parking are not written)
        }
        int edge;
        double pos, speed, angle;
        Pos xy;
        if (s.kind == PStage::WALKING && pedNonInteracting) {
            // MSPModel_NonInteracting::PState::getPosition / getAngle / getSpeed
            edge = stageEdge(s);
            pos = niEdgePos(s);
            speed = walkMaxSpeed(pi, s);
            int lane = getSidewalk(net, edge, types[q.type].vclass);
            if (lane < 0) {
                lane = net.edges[edge].lanes.front();
            }
            const Lane& L = net.lanes[lane];
            const double lateralOffset = L.allows(VC_PEDESTRIAN) ? 0 : 3.;  // MSPModel::SIDEWALK_OFFSET
            xy = geom::positionAtOffset(L.shape, L.interpolateLanePosToGeometryPos(pos), lateralOffset);
            angle = geom::rotationAtOffset(net.lanes[net.edges[edge].lanes[0]].shape, pos) + (s.niEndPos < s.niBeginPos ? M_PI : 0);
            if (angle > M_PI) {
                angle -= 2 * M_PI;
            }
        } else if (s.kind == PStage::WALKING && s.pstate >= 0) {
            const PState& p = pstates[s.pstate];
            edge = stageEdge(s);
            pos = p.edgePos;
            speed = p.speed;
            xy = pedPosition(p);
            angle = pedAngle(p);
        } else if (s.kind == PStage::DRIVING) {
            // MSStageDriving::getPosition / getAngle / getSpeed
            edge = stageEdge(s);
            pos = stageEdgePos(pi, q.step);
            if (s.vehicle >= 0) {
                const int v = s.vehicle;
                speed = kin[v].speed;
                xy = vehPositionParked(v);
                angle = vehAngleParked(v);
            } else {
                speed = 0;
                const Lane& L = net.lanes[net.edges[s.waitingEdge].lanes[0]];
                if (s.originStop >= 0 && net.lanes[busStops[s.originStop].lane].edge == s.waitingEdge) {
                    xy = stopWaitPosition(s.originStop, pi);
                    angle = geom::rotationAtOffset(L.shape, s.waitingPos) + busStops[s.originStop].angle;
                } else {
                    xy = geom::positionAtOffset(L.shape, L.interpolateLanePosToGeometryPos(s.waitingPos), 3.);
                    angle = geom::rotationAtOffset(L.shape, s.waitingPos) + M_PI / 2;
                }
            }
        } else if (s.kind == PStage::ACCESS) {
            // MSPersonStage_Access::getPosition / getAngle / getSpeed: along the straight path (on the stop's edge)
            edge = s.destinationStop >= 0 ? net.lanes[busStops[s.destinationStop].lane].edge : s.destination;
            pos = s.arrivalPos;
            const double len = s.accessFrom.distanceTo2D(s.accessTo);
            const Time dur = s.accessArrival - s.departed;
            xy = geom::positionAtOffset2D(s.accessFrom, s.accessTo, dur > 0 ? len * (double)(now - s.departed) / (double)dur : 0.);
            angle = s.accessFrom.angleTo2D(s.accessTo);
            speed = s.accessDist / STEPS2TIME(std::max((Time)1, dur));
        } else {
            // MSStageWaiting::getPosition / getAngle (at a stop: the waiting position there)
            edge = s.destination;
            pos = s.arrivalPos;
            speed = 0;
            const Lane& L = net.lanes[net.edges[edge].lanes[0]];
            if (s.kind == PStage::WAITING && s.destinationStop >= 0) {
                xy = stopWaitPosition(s.destinationStop, pi);
                angle = geom::rotationAtOffset(L.shape, pos) + busStops[s.destinationStop].angle;
            } else {
                xy = geom::positionAtOffset(L.shape, L.interpolateLanePosToGeometryPos(pos), 3.);
                angle = geom::rotationAtOffset(L.shape, pos) + (s.destinationStop >= 0 ? busStops[s.destinationStop].angle : M_PI / 2);
            }
        }
        std::fprintf(pfcdFile, "%.2f,%s,%s,%.2f,%.2f,%.2f,%.2f,%.2f\n", STEPS2TIME(now), q.id.c_str(), net.edges[edge].id.c_str(), pos,
                     xy.x, xy.y, geom::naviDegree(angle), speed);
    }
}

// ---------------------------------------------------------------------------
// non-interacting pedestrian model (MSPModel_NonInteracting): persons walk each edge of their route in a
// precomputed time, without obstacles and without effect on vehicles
// ---------------------------------------------------------------------------

// MSPModel_NonInteracting::PState::computeDuration
Time Sim::niComputeDuration(int pi, int prevEdge) {
    PStage& s = persons[pi].plan[persons[pi].step];
    s.niLastEntry = now;
    const Edge& edge = net.edges[stageEdge(s)];
    const int next = nextRouteEdge(s);
    int dir = PED_UNDEFINED;
    if (prevEdge < 0) {
        s.niBeginPos = s.departPos;
    } else {
        // default to forward if not connected
        const Edge& prev = net.edges[prevEdge];
        dir = (edge.toJ == prev.toJ || edge.toJ == prev.fromJ) ? PED_BACKWARD : PED_FORWARD;
        s.niBeginPos = dir == PED_FORWARD ? 0 : edge.length;
    }
    if (next < 0) {
        s.niEndPos = s.arrivalPos;
    } else {
        if (dir == PED_UNDEFINED) {
            const Edge& n = net.edges[next];
            dir = (edge.fromJ == n.fromJ || edge.fromJ == n.toJ) ? PED_BACKWARD : PED_FORWARD;
        }
        s.niEndPos = dir == PED_FORWARD ? edge.length : 0;
    }
    // a result > 0 even if the walk ends immediately, rounded to the next step (SUMO adds a whole step)
    s.niDuration = std::max((Time)1, TIME2STEPS(std::fabs(s.niEndPos - s.niBeginPos) / walkMaxSpeed(pi, s)));
    if (s.niDuration % DELTA_T > 0) {
        s.niDuration += DELTA_T;
    }
    return s.niDuration;
}

// MSPModel_NonInteracting::add: the first edge, as a begin-of-step event
void Sim::niAdd(int pi) {
    const Time d = niComputeDuration(pi, -1);
    niEvents[{now + d, niEventSeq++}] = pi;
}

// MSPModel_NonInteracting::MoveToNextEdge::execute for the events of this step (MSEventControl: events before
// the next step; a repeated event keeps its own time base)
void Sim::niStep() {
    while (!niEvents.empty() && niEvents.begin()->first.first < now + DELTA_T) {
        const Time t = niEvents.begin()->first.first;
        const int pi = niEvents.begin()->second;
        niEvents.erase(niEvents.begin());
        Person& person = persons[pi];
        const PStage& s = person.plan[person.step];
        const int old = stageEdge(s);
        const int dir = s.niBeginPos == s.niEndPos ? PED_UNDEFINED : (s.niBeginPos < s.niEndPos ? PED_FORWARD : PED_BACKWARD);
        if (stageMoveToNextEdge(pi, dir, -1)) {
            continue;  // arrived (the person proceeded to the next stage)
        }
        const Time d = std::max(DELTA_T, niComputeDuration(pi, old));
        niEvents[{t + d, niEventSeq++}] = pi;
    }
}

// MSPModel_NonInteracting::PState::getEdgePos
double Sim::niEdgePos(const PStage& s) const {
    return s.niBeginPos + (s.niEndPos - s.niBeginPos) / (double)s.niDuration * (double)(now - s.niLastEntry);
}

}  // namespace ps
