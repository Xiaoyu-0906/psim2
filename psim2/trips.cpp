// Person trips (SUMO 1.28 MSStageTrip::proceed / reroute / getVehicles / setArrived): the trip is routed by the
// intermodal router when it starts and replaced by walks, rides of public transport and rides of the person's
// own vehicle (built here, inserted when the person boards: a triggered departure).
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <unordered_map>

#include "model.hpp"

namespace ps {

// MSNet::getTravelTime for the vehicle of a trip (no vehicle-specific weights)
double Sim::imCarTravelTime(const void* owner, int e, const IMTrip& trip, double t) {
    const Sim& s = *static_cast<const Sim*>(owner);
    double loaded;
    if (!s.loadedWeights.empty() && s.loadedTravelTime(e, t, loaded)) {
        return loaded;
    }
    const Edge& ed = s.net.edges[e];
    const VType& T = s.types[trip.vtype];
    // MSEdge::getMinimumTravelTime (MSLane::getVehicleMaxSpeed of the first lane)
    const double vmax = std::min(T.maxSpeed, T.desiredMaxSpeed * trip.speedFactor);
    const double minTT = ed.length / std::min(vmax, s.net.lanes[ed.lanes[0]].speed * trip.speedFactor) + s.rtPenalty[e];
    const int mode = trip.routingMode;
    if ((mode & (1 | 4)) != 0) {
        // MSRoutingEngine::getEffort / getEffortBike (getEffortExtra: with the extras)
        const std::vector<double>& speeds = s.opt.bikeSpeeds && T.vclass == VC_BICYCLE ? s.rtBikeSpeed : s.rtSpeed;
        const double effort = speeds.empty() ? minTT : std::max(ed.length / std::max(speeds[e], NUMERICAL_EPS), minTT);
        return (mode & 4) != 0 ? s.routingExtrasFor(e, trip.randomSeed, trip.vtype, effort) : effort;
    }
    if (s.rtHaveExtras) {
        return s.routingExtrasFor(e, trip.randomSeed, trip.vtype, minTT);
    }
    return minTT;
}

// MSNet::getIntermodalRouter: the network with the access to all stopping places and the public transport
// schedules of the flows and vehicles known now (MSNet::adaptIntermodalRouter)
void Sim::ensureIntermodal() {
    if (imNet.built()) {
        return;
    }
    if (rtPenalty.empty()) {
        buildRoutingGraph();  // MSEdge::myTimePenalty
    }
    // SUMOVehicleParserHelper::parseCarWalkTransfer (taxis: not modelled)
    int carWalk = 0;
    for (const std::string& o : opt.carWalk) {
        carWalk |= o == "parkingAreas" ? IM_PARKING_AREAS : (o == "ptStops" ? IM_PT_STOPS : (o == "allJunctions" ? IM_ALL_JUNCTIONS : 0));
    }
    for (const std::string& o : opt.taxiWalk) {
        carWalk |= o == "parkingAreas" ? IM_TAXI_DROPOFF_PARKING_AREAS
                                       : (o == "ptStops" ? IM_TAXI_DROPOFF_PT : (o == "allJunctions" ? IM_TAXI_DROPOFF_ANYWHERE : 0));
    }
    for (const std::string& o : opt.walkTaxi) {
        carWalk |= o == "parkingAreas" ? IM_TAXI_PICKUP_PARKING_AREAS
                                       : (o == "ptStops" ? IM_TAXI_PICKUP_PT : (o == "allJunctions" ? IM_TAXI_PICKUP_ANYWHERE : 0));
    }
    imNet.carTravelTime = &Sim::imCarTravelTime;
    imNet.carTravelTimeOwner = this;
    imNet.walkOppositeFactor = opt.walkOppositeFactor;
    std::vector<std::pair<std::vector<int>, std::vector<int>>> zones;
    for (const Taz& z : tazs) {
        zones.push_back({z.sources, z.sinks});
    }
    imNet.build(net, carWalk, zones);
    imNet.addCarEdges(net, opt.taxiWait);
    // the stopping places by category (bus and train stops, container stops, charging stations, parking areas),
    // each sorted by id (NamedObjectCont)
    for (char cat : {'B', 'C', 'E'}) {
        std::vector<int> ids;
        for (int b = 0; b < (int)busStops.size(); ++b) {
            if (busStops[b].category == cat) {
                ids.push_back(b);
            }
        }
        std::sort(ids.begin(), ids.end(), [&](int a, int b) { return busStops[a].id < busStops[b].id; });
        const IMStopCategory category = cat == 'B' ? IM_BUS_STOP : (cat == 'C' ? IM_CONTAINER_STOP : IM_CHARGING_STATION);
        for (int b : ids) {
            const BusStop& s = busStops[b];
            imNet.addAccess(net, s.id, net.lanes[s.lane].edge, s.startPos, s.endPos, 0., category, false, opt.taxiWait);
            if (cat == 'B') {
                for (const BusStop::Access& a : s.accesses) {
                    imNet.addAccess(net, s.id, net.lanes[a.lane].edge, a.startPos, a.endPos, a.length, category, true, opt.taxiWait);
                }
            }
        }
    }
    std::vector<int> pas(parkingAreas.size());
    for (int i = 0; i < (int)pas.size(); ++i) {
        pas[i] = i;
    }
    std::sort(pas.begin(), pas.end(), [&](int a, int b) { return parkingAreas[a].id < parkingAreas[b].id; });
    for (int i : pas) {
        const ParkingArea& pa = parkingAreas[i];
        imNet.addAccess(net, pa.id, net.lanes[pa.lane].edge, pa.begPos, pa.endPos, 0., IM_PARKING_AREA, false, opt.taxiWait);
    }
    // MSInsertionControl::adaptIntermodalRouter (flows not finished yet), MSVehicleControl::adaptIntermodalRouter
    // (loaded vehicles which did not arrive)
    std::unordered_map<std::string, int> vehIdx;
    for (const PTSched& ps : ptScheds) {
        if (!ps.flow) {
            vehIdx[ps.id] = -1;
        }
    }
    for (int v = 0; v < (int)veh.size(); ++v) {
        auto it = vehIdx.find(veh[v].id);
        if (it != vehIdx.end()) {
            it->second = v;
        }
    }
    for (bool flows : {true, false}) {
        for (const PTSched& ps : ptScheds) {
            if (ps.flow != flows) {
                continue;
            }
            if (ps.flow ? ps.lastDepart <= now - DELTA_T : (vehIdx[ps.id] < 0 || veh[vehIdx[ps.id]].status == VehStatus::ARRIVED
                                                             || veh[vehIdx[ps.id]].status == VehStatus::REMOVED)) {
                continue;
            }
            std::vector<IntermodalNet::StopSched> stops;
            for (const auto& st : ps.stops) {
                IntermodalNet::StopSched s{st.first, st.second, 0., 0.};
                auto b = busStopIdx.find(st.first);
                if (b != busStopIdx.end()) {
                    // MSEdge::getStopPosition
                    const BusStop& bs = busStops[b->second];
                    const Lane& l = net.lanes[bs.lane];
                    const Pos p = geom::positionAtOffset(l.shape, l.interpolateLanePosToGeometryPos((bs.startPos + bs.endPos) / 2.), 0);
                    s.x = p.x;
                    s.y = p.y;
                }
                stops.push_back(s);
            }
            imNet.addSchedule(ps.id, ps.line, ps.flow ? ps.repetitionNumber : -1, ps.offset, stops);
        }
    }
}

// IntermodalEdge::getRoutingFactor: 1 / the routing preference of each edge for the persons' types (built at the
// start: read by the pedestrian model's tasks)
void Sim::initPersonPreferences() {
    if (preferenceLines.empty() || persons.empty()) {
        return;
    }
    if (rtPref.empty()) {
        buildRoutingGraph();
    }
    const size_t nRT = net.routingTypes.size();
    for (const Person& p : persons) {
        std::vector<double>& f = imPrefFactor[p.type];
        if (f.empty() && !rtPref.empty()) {
            f.assign(net.edges.size(), 1.);
            for (int e = 0; e < (int)net.edges.size(); ++e) {
                if (net.edges[e].routingType >= 0) {
                    f[e] = 1. / rtPref[(size_t)p.type * nRT + net.edges[e].routingType];
                }
            }
        }
    }
}

const std::vector<double>* Sim::prefFactor(int type) const {
    auto it = imPrefFactor.find(type);
    return it == imPrefFactor.end() || it->second.empty() ? nullptr : &it->second;
}

// a vehicle built while the simulation runs (MSVehicleControl::addVehicle, handleTriggeredDepart): the
// vehicle storage keeps reserve capacity for these (no reallocation)
int Sim::addRuntimeVehicle(Veh x, int type) {
    x.vclass = types[type].vclass;
    if (x.routingMode < 0) {
        x.routingMode = (int8_t)opt.routingMode;
    }
    x.randomSeed = sumoMurmur3(x.id, opt.sumoSeed);
    const int v = (int)veh.size();
    veh.push_back(std::move(x));
    Kin kk;
    kk.type = type;
    kin.push_back(kk);
    kinSnap.push_back(Kin());
    if (cfHaveVars) {
        cfVars.push_back(CFVars());
        initCarFollowVars(v);
    }
    if (sublane) {
        lat.push_back(LatState());
        latSnap.push_back(LatPos());
        if (latRes > 0) {
            sl.push_back(SLState());
            sl.back().impatience = vt(v).lcImpatience;
        }
        lcManeuverSnap.push_back(0);
        lcStratBlockedSnap.push_back(0);
        lcSpeedLatSnap.push_back(0);
    }
    lcSnapStage.push_back(-1);
    lcStateSnap.push_back(0);
    lcLeftSpaceSnap.push_back(0);
    resolveArrival(v);
    if (veh[v].departTriggered) {
        triggeredWaiting[routes[veh[v].route].front()].push_back(v);
        nTriggeredDepart++;
    }
    return v;
}

// MSStoppingPlace::getAccessPos (with the random number generator for random access positions)
double Sim::placeAccessPos(int stop, int parking, int edge, int pi, int stage, bool useRng) const {
    if (parking >= 0) {
        const ParkingArea& pa = parkingAreas[parking];
        return edge == net.lanes[pa.lane].edge ? (pa.begPos + pa.endPos) / 2. : -1.;
    }
    const BusStop& b = busStops[stop];
    if (edge == net.lanes[b.lane].edge) {
        return (b.startPos + b.endPos) / 2.;
    }
    for (const BusStop::Access& a : b.accesses) {
        if (edge == net.lanes[a.lane].edge) {
            if (!useRng || a.startPos == a.endPos) {
                return a.endPos;
            }
            return a.startPos + rand01(opt.seed, (uint64_t)pi, (uint64_t)stage, RP_ACCESS) * (a.endPos - a.startPos);
        }
    }
    return -1.;
}

// MSStageTrip::proceed / reroute / setArrived: the trip is replaced by the stages of the best route
void Sim::tripProceed(int pi, int priorIdx) {
    Person& p = persons[pi];
    const PStage prior = p.plan[priorIdx];
    const int tripIdx = p.step;
    PStage t = p.plan[tripIdx];
    p.plan[tripIdx].departPos = prior.arrivalPos;
    p.plan[tripIdx].originStop = prior.destinationStop;
    ensureIntermodal();
    const int nE = (int)net.edges.size();
    int origin = t.tripFrom;
    const int destination = t.destination;
    if (origin >= nE && (origin - nE) % 2 == 0) {
        // the previous stage ended at a taz sink
        origin = prior.kind == PStage::TRIP ? prior.tripFrom : prior.destination;
    }
    // MSStageTrip::getVehicles: the trip's vehicle types, otherwise a car and a bicycle of the modes
    struct Candidate {
        std::string id;
        int type;
    };
    std::vector<Candidate> cands;
    for (int type : t.tripVTypes) {
        cands.push_back({p.id + "_" + std::to_string(cands.size()), type});
    }
    if (cands.empty()) {
        if ((t.tripModes & VC_PASSENGER) != 0) {
            cands.push_back({p.id + "_0", typeIdx.at("DEFAULT_VEHTYPE")});
        }
        if ((t.tripModes & VC_BICYCLE) != 0) {
            cands.push_back({p.id + "_b0", typeIdx.at("DEFAULT_BIKETYPE")});
        }
    }
    // network edges and taz connectors (MSEdge ids and lengths)
    auto nodeName = [&](int n) {
        return n < nE ? net.edges[n].id : tazs[(n - nE) / 2].id + ((n - nE) % 2 == 0 ? "-sink" : "-source");
    };
    auto nodeLength = [&](int n) { return n < nE ? net.edges[n].length : 0.; };
    SVCPermissions originPerm = origin >= nE ? VC_ALL : 0;  // (taz connectors allow all)
    for (int l : origin >= nE ? std::vector<int>() : net.edges[origin].lanes) {
        originPerm |= net.lanes[l].permissions;
    }
    // vehicles not allowed on the start edge are ignored
    cands.erase(std::remove_if(cands.begin(), cands.end(), [&](const Candidate& c) {
        const SVCPermissions vc = types[c.type].vclass;
        return vc != 0 && (originPerm & vc) == 0;
    }), cands.end());
    const std::string originStop = prior.destinationStop >= 0 ? busStops[prior.destinationStop].id
                                   : (prior.destinationParking >= 0 ? parkingAreas[prior.destinationParking].id : "");
    const std::string destStop = t.destinationStop >= 0 ? busStops[t.destinationStop].id
                                 : (t.destinationParking >= 0 ? parkingAreas[t.destinationParking].id : "");
    IMTrip trip;
    trip.from = origin;
    trip.to = destination;
    trip.departPos = prior.arrivalPos;
    trip.arrivalPos = t.arrivalPos;
    trip.speed = personMaxSpeed(pi) * t.walkFactor;
    trip.departTime = now;
    trip.modeSet = (SVCPermissions)t.tripModes;
    trip.edgeFactor = prefFactor(p.type);
    trip.randomFactor = opt.weightsRandomFactor;
    trip.randomSeed64 = opt.seed;
    trip.randomKey = ((uint64_t)pi << 20) ^ (uint64_t)tripIdx ^ ((uint64_t)now << 32);
    double minCost = std::numeric_limits<double>::max();
    std::vector<IntermodalNet::TripItem> minResult;
    int minCand = -1;
    double minSpeedFactor = 1;
    for (int ci = -1; ci < (int)cands.size(); ++ci) {
        if (ci < 0 && !cands.empty()) {
            continue;  // walking only (with the access infrastructure) when there is no vehicle
        }
        double speedFactor = 1;
        if (ci >= 0) {
            const Candidate& c = cands[ci];
            const VType& T = types[c.type];
            trip.haveVehicle = true;
            trip.vclass = T.vclass;
            trip.vtype = c.type;
            // MSBaseVehicle: the chosen speed factor (psim2's random numbers)
            speedFactor = opt.deterministicSpeedFactor ? T.speedFactorMean
                          : sampleSpeedFactor(T, opt.seed, (int)(((uint64_t)pi << 8) + (uint64_t)ci), RP_TRIP_VEHICLE + 100 * (uint64_t)tripIdx);
            trip.speedFactor = speedFactor;
            trip.routingMode = opt.routingMode;
            trip.randomSeed = sumoMurmur3(c.id, opt.sumoSeed);
        } else {
            trip.haveVehicle = false;
            trip.vclass = VC_PEDESTRIAN;
        }
        std::vector<IntermodalNet::TripItem> result;
        if (imNet.compute(net, trip, originStop, destStop, result)) {
            double totalCost = 0;
            for (const IntermodalNet::TripItem& item : result) {
                totalCost += item.cost;
            }
            static const bool debug = std::getenv("PSIM2_DEBUG_TRIPS") != nullptr;
            if (debug) {
                std::fprintf(stderr, "trip %s t=%.2f cand=%s cost=%.3f\n", p.id.c_str(), STEPS2TIME(now), ci < 0 ? "-" : cands[ci].id.c_str(),
                             totalCost);
                for (const IntermodalNet::TripItem& item : result) {
                    std::string es;
                    for (int e : item.edges) {
                        es += net.edges[e].id + " ";
                    }
                    std::fprintf(stderr, "  line=%s dest=%s cost=%.3f edges=%s\n", item.line.c_str(), item.destStop.c_str(), item.cost, es.c_str());
                }
            }
            if (totalCost < minCost) {
                minCost = totalCost;
                minResult.swap(result);
                minCand = ci;
                minSpeedFactor = speedFactor;
            }
        }
    }
    // the stages (the previous stage of each is the last one created)
    std::vector<PStage> stages;
    struct Prev {
        double arrivalPos;
        int stop, parking, destination;
    } prev{prior.arrivalPos, prior.destinationStop, prior.destinationParking,
           prior.kind == PStage::TRIP ? prior.tripFrom : prior.destination};
    bool disconnected = false;
    if (minCost != std::numeric_limits<double>::max()) {
        for (size_t k = 0; k < minResult.size(); ++k) {
            const IntermodalNet::TripItem& it = minResult[k];
            if (it.edges.empty()) {
                continue;
            }
            // MSNet::getStoppingPlace (any kind)
            int bs = -1, bsParking = -1;
            if (!it.destStop.empty()) {
                auto b = busStopIdx.find(it.destStop);
                if (b != busStopIdx.end()) {
                    bs = b->second;
                } else {
                    auto pa = parkingAreaIdx.find(it.destStop);
                    bsParking = pa != parkingAreaIdx.end() ? pa->second : -1;
                }
            }
            double localArrivalPos = bs >= 0 || bsParking >= 0 ? placeAccessPos(bs, bsParking, it.edges.back(), pi, 0, false)
                                                               : it.arrivalPos;
            const int first = it.edges.front();
            // rides from a zone start on the first edge (MSStageDriving::myOrigin)
            const int rideOrigin = origin >= nE && stages.empty() ? first : -1;
            if (k + 1 == minResult.size() && t.hasArrivalPos) {
                localArrivalPos = t.arrivalPos;
            }
            PStage s;
            if (it.line.empty()) {
                // the walk's departure
                double depPos = prev.arrivalPos;
                if (prev.stop >= 0 || prev.parking >= 0) {
                    depPos = placeAccessPos(prev.stop, prev.parking, first, pi, tripIdx + (int)stages.size(), true);
                } else if (origin >= nE) {
                    // from a zone: the whole first edge (from its start if it is a source edge of the zone)
                    const std::vector<int>& srcs = tazs[(origin - nE) / 2].sources;
                    depPos = std::find(srcs.begin(), srcs.end(), first) != srcs.end() ? 0. : net.edges[first].length;
                } else if (prev.destination != first) {
                    const int prevTo = net.edges[prev.destination].toJ, prevFrom = net.edges[prev.destination].fromJ;
                    depPos = (prevTo == net.edges[first].toJ || prevFrom == net.edges[first].toJ) ? net.edges[first].length : 0.;
                }
                if (destination >= nE) {
                    // to a zone: the whole last edge
                    const std::vector<int>& sinks = tazs[(destination - nE) / 2].sinks;
                    localArrivalPos = std::find(sinks.begin(), sinks.end(), it.edges.back()) != sinks.end()
                                      ? net.edges[it.edges.back()].length : 0.;
                }
                s.kind = PStage::WALKING;
                s.route = it.edges;
                s.destinationStop = bs;
                s.destinationParking = bsParking;
                s.speed = t.speed;
                s.walkingTime = t.walkingTime;
                s.departPosLat = t.departPosLat;
                s.departPos = interpretEdgePos(depPos, net.edges[first].length);
                s.arrivalPos = interpretEdgePos(localArrivalPos, net.edges[it.edges.back()].length);
                s.destination = it.edges.back();
                if (s.walkingTime > 0) {
                    s.speed = walkDistance(p, s, false) / STEPS2TIME(s.walkingTime + 1);
                }
            } else if (it.line == "!car" && minCand >= 0) {
                // the ride in the trip's vehicle, which waits for the person on the first edge
                const Candidate& c = cands[minCand];
                Veh x;
                x.id = c.id;
                x.departTriggered = true;
                x.depart = TIME_MAX;
                routes.push_back(it.edges);
                x.route = (int)routes.size() - 1;
                char ap[32];
                std::snprintf(ap, sizeof(ap), "%.17g", localArrivalPos);
                char dp[32];
                std::snprintf(dp, sizeof(dp), "%.17g", prior.arrivalPos);
                parseVehicleDefinitions(x, vDefDepartLane, vDefDepartSpeed, prior.arrivalPos != 0 ? dp : "base", ap, "current", "0");
                x.speedFactor = minSpeedFactor;
                addRuntimeVehicle(std::move(x), c.type);
                s.kind = PStage::DRIVING;
                s.origin = rideOrigin;
                s.lines = {c.id};
                s.destination = it.edges.back();
                s.destinationStop = bs;
                s.destinationParking = bsParking;
                s.arrivalPos = localArrivalPos;
            } else {
                // public transport: any line stopping at the destination (or the intended line)
                s.kind = PStage::DRIVING;
                s.origin = rideOrigin;
                s.lines = {opt.ridePublicLine ? it.line : std::string("ANY")};
                s.intendedVeh = it.intended;
                s.destination = it.edges.back();
                s.destinationStop = bs;
                s.destinationParking = bsParking;
                s.arrivalPos = localArrivalPos;
            }
            prev = {s.arrivalPos, s.destinationStop, s.destinationParking, s.destination};
            stages.push_back(std::move(s));
        }
    } else {
        // no route: a walk from the origin to the destination (the person jumps)
        if (!opt.ignoreRouteErrors) {
            throw std::runtime_error("No connection found between " + (prior.destinationStop >= 0 ? "busStop '" + busStops[prior.destinationStop].id
                                                                                           : "edge '" + nodeName(origin))
                                     + "' and " + (t.destinationStop >= 0 ? "busStop '" + busStops[t.destinationStop].id
                                                                           : "edge '" + nodeName(destination))
                                     + "' for person '" + p.id + "'.");
        }
        std::fprintf(stderr, "Warning: No connection between edge '%s' and edge '%s' found.\n", nodeName(origin).c_str(),
                     nodeName(destination).c_str());
        disconnected = true;
    }
    if (stages.empty()) {
        if (!disconnected && prior.destinationStop >= 0 && prior.destinationStop == t.destinationStop) {
            // MSStageWaiting "sameStop"
            PStage w;
            w.kind = PStage::WAITING;
            w.destination = destination;
            w.destinationStop = t.destinationStop;
            w.duration = 0;
            w.until = -1;
            w.arrivalPos = prior.arrivalPos;
            w.actType = "sameStop";
            stages.push_back(w);
        } else {
            if (!disconnected && !opt.ignoreRouteErrors) {
                throw std::runtime_error("Empty route between edge '" + nodeName(origin) + "' and edge '" + nodeName(destination)
                                         + "' for person '" + p.id + "'.");
            }
            PStage w;
            w.kind = PStage::WALKING;
            w.route = {origin, destination};
            w.destinationStop = t.destinationStop;
            w.speed = t.speed;
            w.walkingTime = t.walkingTime;
            w.departPosLat = t.departPosLat;
            w.departPos = interpretEdgePos(prior.arrivalPos, nodeLength(origin));
            w.arrivalPos = interpretEdgePos(t.arrivalPos, nodeLength(destination));
            w.destination = destination;
            if (w.walkingTime > 0) {
                w.speed = walkDistance(p, w, false) / STEPS2TIME(w.walkingTime + 1);
            }
            stages.push_back(w);
        }
    }
    p.plan.insert(p.plan.begin() + tripIdx + 1, stages.begin(), stages.end());
    // the trip ends right away, the first of its stages starts
    personProceed(pi);
}

}  // namespace ps

namespace ps {

// MSPerson::checkAccess: an access stage between the stop of the prior stage and the access lane of its
// destination (waitAtStop) or of the next walk's first edge
bool Sim::checkAccess(int pi, int priorIdx, bool waitAtStop) {
    Person& p = persons[pi];
    const PStage& prior = p.plan[priorIdx];
    int prevStop = prior.destinationStop;
    if (!waitAtStop && prior.kind == PStage::TRIP) {
        prevStop = prior.originStop;
    }
    if (prevStop < 0) {
        return false;
    }
    const BusStop& b = busStops[prevStop];
    const int accessEdge = waitAtStop ? prior.destination : (p.plan[p.step].kind == PStage::WALKING ? p.plan[p.step].route[0]
                                                                                                    : p.plan[p.step].destination);
    const BusStop::Access* access = nullptr;
    for (const BusStop::Access& a : b.accesses) {
        if (net.lanes[a.lane].edge == accessEdge) {
            access = &a;
            break;
        }
    }
    if (access == nullptr) {
        return false;
    }
    const Lane& lane = net.lanes[net.edges[accessEdge].lanes[0]];
    const Lane& stopLane = net.lanes[b.lane];
    auto geomPos = [](const Lane& l, double pos, double lat) {
        return geom::positionAtOffset(l.shape, l.interpolateLanePosToGeometryPos(pos), lat);
    };
    PStage s;
    s.kind = PStage::ACCESS;
    s.destinationStop = prevStop;
    s.accessDist = access->length;
    if (waitAtStop) {
        // to the stop
        const double arrivalAtBs = (b.startPos + b.endPos) / 2;
        s.destination = stopLane.edge;
        s.arrivalPos = arrivalAtBs;
        s.accessExit = false;
        s.accessFrom = geomPos(lane, access->endPos, 0);
        s.accessTo = geomPos(stopLane, arrivalAtBs, 0);
    } else {
        // from the stop (the platform position or the nearest position to the vehicle's doors)
        const double startPos = prior.kind == PStage::TRIP ? prior.departPos : prior.arrivalPos;
        const Pos trainExit = geomPos(stopLane, startPos, 0);
        const bool useDoors = access->exit != 0;  // (carriage: as doors without jupedsim)
        const double arrivalPos = useDoors ? geom::nearestOffsetToPoint2D(lane.shape, trainExit, true) : access->endPos;
        Pos platformEntry = geomPos(lane, arrivalPos, 0);
        if (useDoors) {
            // the closer side of the platform
            const VType& t = types[p.type];
            const double halfWidth = lane.width / 2. - std::max(t.length, t.width) / 2. - POSITION_EPS;
            platformEntry = geomPos(lane, arrivalPos, halfWidth);
            const Pos plat2 = geomPos(lane, arrivalPos, -halfWidth);
            if (trainExit.distanceSquaredTo2D(plat2) < trainExit.distanceSquaredTo2D(platformEntry)) {
                platformEntry = plat2;
            }
        }
        s.destination = accessEdge;
        s.arrivalPos = arrivalPos;
        s.accessExit = true;
        s.accessFrom = trainExit;
        s.accessTo = platformEntry;
    }
    p.plan.insert(p.plan.begin() + p.step, s);
    return true;
}

// MSPersonStage_Access::ProceedCmd: the events before the given time in time order
void Sim::processAccessEnds(Time until) {
    while (!accessEvents.empty() && accessEvents.begin()->first < until) {
        const int pi = accessEvents.begin()->second;
        accessEvents.erase(accessEvents.begin());
        if (!personProceed(pi)) {
            personErase(pi);
        }
    }
}

}  // namespace ps
