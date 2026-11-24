// Vehicle routing, ported from SUMO 1.28: MSRoutingEngine (edge weights from smoothed edge speeds),
// MSDevice_Routing (rerouting before the insertion and periodically), DijkstraRouter and
// SUMOAbstractRouter (utils/router), MSBaseVehicle::reroute / replaceRouteEdges / replaceRoute.
// Only the default routing algorithm (dijkstra) without taz, weight files, random weight factors,
// edge priorities or turn-specific weights.
//
// Execution: the edge weights change only at the end of a step (adaptation), so the reroutes of a
// step are independent of each other. They are computed in parallel, one router workspace per
// thread; every vehicle with a routing device owns its route slot (Sim::routes), so a reroute only
// writes data of its own vehicle (also when a stop ends inside the parallel move planning).
#include <algorithm>
#include <queue>
#include <sstream>
#include <cmath>
#include <stdexcept>
#include <cstdio>

#include "model.hpp"

namespace ps {

namespace {
constexpr double UNREACHED = std::numeric_limits<double>::max();
}

// ---------------------------------------------------------------------------
// network data
// ---------------------------------------------------------------------------
// MSEdge::closeBuilding (successors with the via edge of the first link to them), myCombinedPermissions,
// MSEdge::recalcCache (myTimePenalty: minor links and turnarounds without traffic light)
void Sim::buildRoutingGraph() {
    const int nE = (int)net.edges.size();
    // MSRoutingEngine::initWeightConstants
    rtPriorityFactor = opt.weightsPriorityFactor;
    if (rtPriorityFactor != 0) {
        double minPrio = std::numeric_limits<double>::max(), maxPrio = -std::numeric_limits<double>::max();
        for (const Edge& ed : net.edges) {
            minPrio = std::min(minPrio, (double)ed.priority);
            maxPrio = std::max(maxPrio, (double)ed.priority);
        }
        rtMinPriority = minPrio;
        rtPriorityRange = maxPrio - minPrio;
        if (rtPriorityRange == 0) {
            rtPriorityFactor = 0;  // "does not take effect because all edges have the same priority"
        }
    }
    // MSNet::getPreference per vehicle type and routing type: the type's preference, its class's, the general one
    rtPref.clear();
    if (!preferenceLines.empty()) {
        const int nRT = (int)net.routingTypes.size();
        rtPref.assign(types.size() * (size_t)std::max(1, nRT), 1.);
        for (int t = 0; t < (int)types.size(); ++t) {
            for (int r = 0; r < nRT; ++r) {
                double byType = -1, byClass = -1, general = -1;
                for (const std::vector<std::string>& p : preferenceLines) {
                    if (p[1] != net.routingTypes[r]) {
                        continue;
                    }
                    const double prio = std::stod(p[4]);
                    if (p[2] != "-") {
                        std::stringstream ss(p[2]);
                        std::string c;
                        while (std::getline(ss, c, ',')) {
                            if ((SVCPermissions)vclassFromName(c) == (SVCPermissions)types[t].vclass) {
                                byClass = prio;
                            }
                        }
                    }
                    if (p[3] != "-") {
                        std::stringstream ss(p[3]);
                        std::string c;
                        while (std::getline(ss, c, ',')) {
                            if (c == types[t].id) {
                                byType = prio;
                            }
                        }
                    } else if (p[2] == "-") {
                        general = prio;
                    }
                }
                rtPref[(size_t)t * nRT + r] = byType > 0 ? byType : byClass > 0 ? byClass : general > 0 ? general : 1.;
            }
        }
    }
    rtHaveExtras = rtPriorityFactor != 0 || opt.weightsRandomFactor != 1. || !rtPref.empty();
    const int nNodes = nE + 2 * (int)tazs.size();
    rtViaSucc.assign(nNodes, {});
    rtPenalty.assign(nNodes, 0);
    rtPerm.assign(nNodes, VC_ALL);
    rtPermOrig.assign(nNodes, VC_ALL);
    for (int e = 0; e < nE; ++e) {
        rtPerm[e] = 0;
        rtPermOrig[e] = 0;
    }
    for (int e = 0; e < nE; ++e) {
        const Edge& ed = net.edges[e];
        for (int lane : ed.lanes) {
            rtPerm[e] |= net.lanes[lane].permissions;
            rtPermOrig[e] |= net.lanes[lane].origPermissions;
            for (int l : net.lanes[lane].links) {
                const Link& k = net.links[l];
                const int to = net.lanes[k.lane].edge;
                bool known = false;
                for (const std::pair<int, int>& s : rtViaSucc[e]) {
                    known |= s.first == to;
                }
                if (!known) {
                    rtViaSucc[e].push_back({to, k.via < 0 ? -1 : net.lanes[k.via].edge});
                }
            }
        }
        if (ed.internal && !net.lanes[ed.lanes[0]].incoming.empty()) {
            const Link& k = net.links[net.lanes[ed.lanes[0]].incoming[0].link];
            const bool priority = k.offState >= 'A' && k.offState <= 'Z';
            if (k.tls < 0 && !priority) {
                rtPenalty[e] = k.isTurnaround() ? opt.turnaroundPenalty : opt.minorPenalty;
            }
        }
    }
    // taz connectors (NLHandler::addDistrict, junction-taz): the source leads to its edges, the edges to the sink
    for (int i = 0; i < (int)tazs.size(); ++i) {
        for (int e : tazs[i].sources) {
            rtViaSucc[tazNode(i, true)].push_back({e, -1});
        }
        for (int e : tazs[i].sinks) {
            rtViaSucc[e].push_back({tazNode(i, false), -1});
        }
    }
    buildRailRouter();
}

// which vehicles get a routing device (MSDevice_Routing::buildVehicleDevices), their route slots and
// the first reroute events
void Sim::initRouting() {
    rtActive = false;
    for (size_t v = 0; v < veh.size(); ++v) {
        Veh& x = veh[v];
        // MSDevice::equippedByDefaultAssignmentOptions: by name, by parameter, by probability (psim2's random
        // numbers)
        bool equip = false;
        const double draw = rand01(opt.seed, (uint64_t)v, 0, RP_REROUTE_EQUIP);
        if (std::find(opt.rerouteExplicit.begin(), opt.rerouteExplicit.end(), x.id) != opt.rerouteExplicit.end()) {
            equip = true;
        } else if (x.deviceParam >= 0) {
            equip = x.deviceParam == 1;
        } else if (x.deviceProb >= 0) {
            equip = draw < x.deviceProb;
        } else if (opt.rerouteProb >= 0) {
            equip = draw < opt.rerouteProb;
        }
        if (!x.forceReroute && !equip) {
            if (hasRerouters()) {
                // rerouters may replace the route of any vehicle: own route slot
                routes.push_back(routes[x.route]);
                x.route = (int)routes.size() - 1;
            }
            continue;
        }
        x.routingDevice = true;
        rtActive = true;
        // MSDevice_Routing::buildVehicleDevices: trips reroute periodically unless a probability was given
        const double period = x.periodParam >= 0 ? x.periodParam : opt.reroutePeriod;
        x.reroutePeriod = (equip || (opt.rerouteProb < 0 && x.deviceProb < 0)) ? TIME2STEPS(period) : 0;
        // own route slot (rerouting replaces it in place)
        routes.push_back(routes[x.route]);
        x.route = (int)routes.size() - 1;
    }
    // MSBaseVehicle::initDevices: the aggregated routing modes (1, 4) use the device's edge weights
    bool aggregated = false;
    for (const Veh& x : veh) {
        aggregated |= x.routingMode > 0 && (x.routingMode & (1 | 4)) != 0;
    }
    rtAdapt = rtActive || aggregated;
    // MSRoutingEngine::isEnabled (initWeightUpdate): from the build of the first vehicle with a routing device
    // or an aggregated routing mode on (Sim::vehicleBuilt)
    rtEngineFrom = TIME_MAX;
    if (!rtAdapt) {
        if (hasRerouters()) {
            buildRoutingGraph();
        }
        return;
    }
    buildRoutingGraph();
    rtEdgeUpdates = opt.adaptWeight < 1. && opt.adaptInterval > 0;
    const Time prePeriod = std::max((Time)0, TIME2STEPS(opt.reroutePrePeriod));
    const Time load = TIME2STEPS(opt.begin);
    rtNextAdaptation = rtEdgeUpdates ? load : TIME_MAX;
    if (opt.weightsSeparateTurns > 0 && rtActive) {
        // MSRoutingEngine::myEdgeTravelTimes (written by the vehicles' tasks, read at the adaptation)
        rtTurnTT.reset(new std::atomic<long long>[net.edges.size()]);
        rtTurnCount.reset(new std::atomic<long long>[net.edges.size()]);
        for (size_t e = 0; e < net.edges.size(); ++e) {
            rtTurnTT[e].store(0);
            rtTurnCount[e].store(0);
        }
    }
    // (the weights are initialized by the first adaptation, before that the minimum travel times apply)
    if (!rtActive) {
        return;
    }
    (void)load;
    (void)prePeriod;
    // (the pre-insertion reroute events are added when the vehicles are built, see Sim::vehicleBuilt)
}

// MSNet::myInsertionEvents (MSEventControl::addEvent): a heap ordered by time only, equal times leave the order to
// the heap operations (as in SUMO)
void Sim::pushPreReroute(int v, Time t) {
    veh[v].rerouteNextPre = t;
    preRerouteQueue.push_back({t, v});
    std::push_heap(preRerouteQueue.begin(), preRerouteQueue.end(),
                   [](const std::pair<Time, int>& a, const std::pair<Time, int>& b) { return a.first > b.first; });
}

// ---------------------------------------------------------------------------
// edge weights (MSRoutingEngine)
// ---------------------------------------------------------------------------
// MSEdge::getMeanSpeed: lanes weighted by their vehicle number (empty lanes count once)
double Sim::edgeMeanSpeed(int e) const {
    double v = 0;
    double n = 0;
    for (int lane : net.edges[e].lanes) {
        int numVehs = (int)lanes[lane].vehs.size();
        if (numVehs == 0) {
            numVehs = 1;
        }
        v += numVehs * laneMeanSpeed(lane);
        n += numVehs;
    }
    const int bidi = net.edges[e].bidi;
    if (bidi >= 0) {
        for (int lane : net.edges[bidi].lanes) {
            if (!lanes[lane].vehs.empty()) {
                return 0;  // do not route across edges which are already occupied in reverse direction
            }
        }
    }
    if (n == 0) {
        return net.edgeSpeedLimit(e);
    }
    return v / n;
}

// MSRoutingEngine::_initEdgeWeights (first use)
void Sim::routingInitWeights() {
    if (!rtSpeed.empty()) {
        return;
    }
    const int nE = (int)net.edges.size();
    rtSpeed.resize(nE);
    rtSteps = opt.adaptWeight == 0 ? opt.adaptSteps : 0;
    rtPast.assign(rtSteps > 0 ? (size_t)nE * rtSteps : 0, 0);
    for (int e = 0; e < nE; ++e) {
        double tt;
        if (opt.initWithLoadedWeights) {
            // MSNet::getTravelTime(edge, nullptr, now): the loaded weight, otherwise the empty travel time
            rtSpeed[e] = net.edges[e].length / (loadedTravelTime(e, STEPS2TIME(now), tt) ? tt
                                                : net.edges[e].length / net.edgeSpeedLimit(e) + rtPenalty[e]);
        } else {
            rtSpeed[e] = edgeMeanSpeed(e);
        }
        for (int i = 0; i < rtSteps; ++i) {
            rtPast[(size_t)e * rtSteps + i] = rtSpeed[e];
        }
    }
    if (opt.bikeSpeeds) {
        rtBikeSpeed = rtSpeed;
        rtBikePast = rtPast;
    }
    rtLastAdaptation = now;
}

// MSRoutingEngine::adaptEdgeEfforts (end of step every adaptation interval); edges are independent
void Sim::adaptEdgeWeights() {
    if (!rtAdapt || rtNextAdaptation >= now + DELTA_T) {
        return;
    }
    rtNextAdaptation += TIME2STEPS(opt.adaptInterval);
    routingInitWeights();
    if (nInserted == 0) {
        return;
    }
    const int nE = (int)net.edges.size();
    const double newWeightFactor = 1. - opt.adaptWeight;
    auto adapt = [&](int e) {
        // MSEdge::isDelayed: vehicles were on this edge or on its bidi edge
        if (!edgeDelayed[e] && (net.edges[e].bidi < 0 || !edgeDelayed[net.edges[e].bidi])) {
            return;
        }
        double curr = edgeMeanSpeed(e);
        if (rtTurnTT != nullptr && rtViaSucc[e].size() > 1) {
            curr = patchSpeedForTurns(e, curr);
        }
        if (rtSteps > 0) {
            double& past = rtPast[(size_t)e * rtSteps + rtPastIdx];
            rtSpeed[e] += (curr - past) / rtSteps;
            past = curr;
            if (opt.bikeSpeeds) {
                const double currBike = edgeMeanSpeedBike(e);
                double& pastBike = rtBikePast[(size_t)e * rtSteps + rtPastIdx];
                rtBikeSpeed[e] += (currBike - pastBike) / rtSteps;
                pastBike = currBike;
            }
        } else {
            if (curr != rtSpeed[e]) {
                rtSpeed[e] = rtSpeed[e] * opt.adaptWeight + curr * newWeightFactor;
            }
            if (opt.bikeSpeeds) {
                const double currBike = edgeMeanSpeedBike(e);
                if (currBike != rtBikeSpeed[e]) {
                    rtBikeSpeed[e] = rtBikeSpeed[e] * opt.adaptWeight + currBike * newWeightFactor;
                }
            }
        }
    };
    if (rtTurnTT != nullptr) {
        // turn-specific speeds change the internal edges of an edge: in edge order (internal edges come first)
        for (int e = 0; e < nE; ++e) {
            adapt(e);
        }
    } else {
        pool.setTag(ThreadPool::T_OTHER);
        pool.parallel_for(nE, adapt, 256);
    }
    if (rtSteps > 0) {
        rtPastIdx = (rtPastIdx + 1) % rtSteps;
    }
    rtLastAdaptation = now;
}

// MSRoutingEngine::patchSpeedForTurns (--weights.separate-turns): the edge's speed is corrected towards the fastest
// turn, the slower turns get the difference as a time penalty on their internal edge
double Sim::patchSpeedForTurns(int e, double currSpeed) {
    const double length = net.edges[e].length;
    const double st = opt.weightsSeparateTurns;
    auto avgSpeed = [&](int ie) {
        const long long count = rtTurnCount[ie].load(std::memory_order_relaxed);
        return length / STEPS2TIME(rtTurnTT[ie].load(std::memory_order_relaxed) / count);
    };
    double maxSpeed = 0;
    for (const std::pair<int, int>& f : rtViaSucc[e]) {
        if (f.second >= 0 && rtTurnCount[f.second].load(std::memory_order_relaxed) > 0) {
            maxSpeed = std::max(avgSpeed(f.second), maxSpeed);
        }
    }
    if (maxSpeed <= 0) {
        return currSpeed;
    }
    const double correctedSpeed = st * maxSpeed + (1 - st) * currSpeed;
    for (const std::pair<int, int>& f : rtViaSucc[e]) {
        const int iid = f.second;
        if (iid < 0 || rtTurnCount[iid].load(std::memory_order_relaxed) == 0) {
            continue;
        }
        const double avg = avgSpeed(iid);
        if (avg < correctedSpeed) {
            const Edge& ie = net.edges[iid];
            double internalTT = ie.length / net.edgeSpeedLimit(iid);
            internalTT += (length / avg - length / correctedSpeed) * st;
            const double newInternalSpeed = ie.length / internalTT;
            if (rtSteps > 0) {
                // the past speeds get a virtual value consistent with the new speed (it reverts without updates)
                const double origInternalSpeed = rtSpeed[iid];
                double& past = rtPast[(size_t)iid * rtSteps + rtPastIdx];
                past = (newInternalSpeed - (origInternalSpeed - past / rtSteps)) * rtSteps;
            }
            rtSpeed[iid] = newInternalSpeed;
        }
        if (rtPastIdx == 0) {
            rtTurnTT[iid].store(0, std::memory_order_relaxed);
            rtTurnCount[iid].store(0, std::memory_order_relaxed);
        }
    }
    return correctedSpeed;
}

// MSRoutingEngine::getEffort: travel time at the smoothed speed, at least the minimum travel time;
// MSNet::getTravelTime (vehicles without a routing device): the minimum travel time
double Sim::routingEffort(int e, int v, const RouteWS& ws, double t) const {
    if (e >= (int)net.edges.size()) {
        return 0;  // taz connector (MSEdge::getMinimumTravelTime)
    }
    double loaded;
    if (ws.staticEffort && !loadedWeights.empty() && loadedTravelTime(e, t < 0 ? ws.queryTime : t, loaded)) {
        return loaded;  // MSNet::getTravelTime: the loaded weights come first
    }
    const Edge& ed = net.edges[e];
    const double minTT = ed.length / laneVehMaxSpeed(ed.lanes[0], v) + rtPenalty[e];
    // MSRoutingEngine's router (the routing device): smoothed weights; MSNet::getTravelTime: by routing mode
    const int mode = std::max(0, (int)veh[v].routingMode);
    const bool smoothed = !ws.staticEffort || (mode & (1 | 4)) != 0;
    const bool extras = (mode & 4) != 0 || ((mode & 1) == 0 && rtHaveExtras) || (!ws.staticEffort && rtHaveExtras);
    // MSRoutingEngine::getEffortBike: bicycles use their own speeds
    const std::vector<double>& speeds = opt.bikeSpeeds && veh[v].vclass == VC_BICYCLE ? rtBikeSpeed : rtSpeed;
    const double effort = !smoothed || speeds.empty() ? minTT : std::max(ed.length / std::max(speeds[e], NUMERICAL_EPS), minTT);
    return extras ? routingExtras(e, v, effort) : effort;
}

bool Sim::loadedTravelTime(int e, double t, double& value) const {
    for (auto it = loadedWeights[e].rbegin(); it != loadedWeights[e].rend(); ++it) {
        if (it->begin <= t && t < it->end) {
            value = it->value;
            return true;
        }
    }
    return false;
}

// MSEdge::getMeanSpeedBike (MSLane::getMeanSpeedBike divides the bicycle speeds by all vehicles of the lane)
double Sim::edgeMeanSpeedBike(int e) const {
    double v = 0;
    double total = 0;
    for (int lane : net.edges[e].lanes) {
        const std::vector<int>& vs = lanes[lane].vehs;
        double laneSpeed = net.lanes[lane].speed;
        if (!vs.empty()) {
            double sum = 0;
            int bikes = 0;
            for (int u : vs) {
                if (veh[u].vclass == VC_BICYCLE) {
                    sum += kin[u].speed;
                    bikes++;
                }
            }
            if (bikes > 0) {
                laneSpeed = sum / (double)vs.size();
            }
        }
        v += (double)vs.size() * laneSpeed;
        total += (double)vs.size();
    }
    return total == 0 ? net.edgeSpeedLimit(e) : v / total;
}

// MSRoutingEngine::applyExtras (the device's getEffortExtra, MSNet::getTravelTime): random factors from SUMO's
// vehicle and edge hash (the same values as SUMO), the edge priority factor
double Sim::routingExtras(int e, int v, double effort) const {
    return routingExtrasFor(e, veh[v].randomSeed, kin[v].type, effort);
}

double Sim::routingExtrasFor(int e, uint32_t randomSeed, int type, double effort) const {
    if (opt.weightsRandomFactor != 1.) {
        long long key = (long long)randomSeed ^ (long long)e;
        if (opt.weightsRandomDynamic) {
            key ^= now;
        }
        effort *= 1 + sumoRandHash(key) * (opt.weightsRandomFactor - 1);
    }
    if (rtPriorityFactor != 0) {
        // lower priority: higher effort (the edge with the minimum priority gets the factor 1 + priority-factor)
        const double relativeInversePrio = 1 - ((net.edges[e].priority - rtMinPriority) / rtPriorityRange);
        effort *= 1 + relativeInversePrio * rtPriorityFactor;
    }
    if (!rtPref.empty() && net.edges[e].routingType >= 0) {
        effort /= rtPref[(size_t)type * net.routingTypes.size() + net.edges[e].routingType];
    }
    return effort;
}

// ---------------------------------------------------------------------------
// router (DijkstraRouter, SUMOAbstractRouter)
// ---------------------------------------------------------------------------
// MSEdge::prohibits and the edges closed by a rerouter (SUMOAbstractRouter::isProhibited)
bool Sim::routeProhibited(int e, int v, const RouteWS& ws) const {
    const SVCPermissions vc = veh[v].vclass;
    // MSEdge::prohibits: routing mode 8 ignores the transient permission changes (rerouter closings)
    if (((veh[v].ignoreTransientPermissions() ? rtPermOrig[e] : rtPerm[e]) & vc) != vc) {
        return true;
    }
    if (ws.prohibited != nullptr) {
        for (const RouterProhibition& p : *ws.prohibited) {
            if (p.edge == e && (p.permissions & vc) != vc && p.end == std::numeric_limits<double>::infinity()) {
                return true;
            }
        }
    }
    return false;
}

// SUMOAbstractRouter::getEffort: an edge closed until a known time is passed after the closing ends
double Sim::prohibitionDelay(int e, int v, const RouteWS& ws, double t) const {
    if (ws.prohibited != nullptr) {
        const SVCPermissions vc = veh[v].vclass;
        for (const RouterProhibition& p : *ws.prohibited) {
            if (p.edge == e && p.end > t && t >= 0 && (p.permissions & vc) != vc) {
                return p.end - t;
            }
        }
    }
    return 0;
}

// effort of the junction-internal edges between an edge and its successor (updateViaEdgeCost)
double Sim::routeViaEffort(int via, int v, const RouteWS& ws, double t) const {
    double effort = 0;
    while (via >= 0 && net.edges[via].internal) {
        effort += routingEffort(via, v, ws, t < 0 ? -1 : t + effort);
        via = rtViaSucc[via].empty() ? -1 : rtViaSucc[via].front().second;
    }
    return effort;
}

bool Sim::routeCompute(int from, int to, int v, std::vector<int>& into, RouteWS& ws) const {
    if (useRailRouter(v, ws)) {
        return railRouteCompute(from, to, v, into, ws, false);  // MSRouterProvider::getVehicleRouter
    }
    if (routeProhibited(from, v, ws) || routeProhibited(to, v, ws)) {
        return false;
    }
    const int nE = (int)rtViaSucc.size();  // edges and taz connectors
    if ((int)ws.effort.size() != nE) {
        ws.effort.assign(nE, UNREACHED);
        ws.prev.assign(nE, -1);
        ws.visited.assign(nE, 0);
        ws.touched.clear();
    }
    for (int e : ws.touched) {
        ws.effort[e] = UNREACHED;
        ws.prev[e] = -1;
        ws.visited[e] = 0;
    }
    ws.touched.clear();
    ws.heap.clear();
    // EdgeInfoByEffortComparator: smallest effort first, ties by the smaller numerical id
    auto cmp = [&](int a, int b) {
        if (ws.effort[a] == ws.effort[b]) {
            return a > b;
        }
        return ws.effort[a] > ws.effort[b];
    };
    const SVCPermissions vc = veh[v].vclass;
    const bool ignoreTransient = veh[v].ignoreTransientPermissions();
    ws.effort[from] = 0;
    ws.touched.push_back(from);
    ws.heap.push_back(from);
    while (!ws.heap.empty()) {
        const int m = ws.heap.front();
        ws.visited[m] = 1;
        if (m == to) {
            const size_t first = into.size();
            for (int e = m; e >= 0; e = ws.prev[e]) {
                into.push_back(e);
            }
            std::reverse(into.begin() + (long)first, into.end());
            return true;
        }
        std::pop_heap(ws.heap.begin(), ws.heap.end(), cmp);
        ws.heap.pop_back();
        // (the effort is a travel time: the vehicle enters edge m at the query time plus its effort)
        const double base = ws.effort[m] + routingEffort(m, v, ws, ws.queryTime + ws.effort[m])
                            + prohibitionDelay(m, v, ws, ws.queryTime + ws.effort[m]);
        for (const std::pair<int, int>& f : rtViaSucc[m]) {
            // MSEdge::getViaSuccessors(vClass): successors reachable with lanes allowing the class
            const bool connector = m >= (int)net.edges.size() || f.first >= (int)net.edges.size();
            if ((!connector && net.allowedLanes(m, f.first, vc, ignoreTransient) == nullptr) || routeProhibited(f.first, v, ws)) {
                continue;
            }
            const double effort = base + routeViaEffort(f.second, v, ws, ws.queryTime + base);
            const int s = f.first;
            const double old = ws.effort[s];
            if (!ws.visited[s] && effort < old) {
                ws.effort[s] = effort;
                ws.prev[s] = m;
                if (old == UNREACHED) {
                    ws.touched.push_back(s);
                    ws.heap.push_back(s);
                    std::push_heap(ws.heap.begin(), ws.heap.end(), cmp);
                } else {
                    std::push_heap(ws.heap.begin(), std::find(ws.heap.begin(), ws.heap.end(), s) + 1, cmp);
                }
            }
        }
    }
    return false;
}

// SUMOAbstractRouter::compute with positions: computeLooped when the target lies behind the start
bool Sim::routeComputePos(int from, double fromPos, int to, double toPos, int v, std::vector<int>& into, RouteWS& ws) const {
    if (from != to || fromPos <= toPos) {
        return routeCompute(from, to, v, into, ws);
    }
    double minEffort = UNREACHED;
    std::vector<int> best, tmp;
    const SVCPermissions vc = veh[v].vclass;
    for (const std::pair<int, int>& f : rtViaSucc[from]) {
        if (net.allowedLanes(from, f.first, vc, veh[v].ignoreTransientPermissions()) == nullptr) {
            continue;
        }
        tmp.clear();
        routeCompute(f.first, to, v, tmp, ws);
        if (!tmp.empty()) {
            const double effort = routeCosts(tmp, v, ws);
            if (effort < minEffort) {
                minEffort = effort;
                best = tmp;
            }
        }
    }
    if (minEffort == UNREACHED) {
        return false;
    }
    into.push_back(from);
    into.insert(into.end(), best.begin(), best.end());
    return true;
}

// SUMOAbstractRouter::recomputeCosts
double Sim::routeCosts(const std::vector<int>& edges, int v, const RouteWS& ws) const {
    double effort = 0;
    int prev = -1;
    for (int e : edges) {
        if (prev >= 0) {
            for (const std::pair<int, int>& f : rtViaSucc[prev]) {
                if (f.first == e) {
                    effort += routeViaEffort(f.second, v, ws, ws.queryTime + effort);
                    break;
                }
            }
        }
        effort += routingEffort(e, v, ws, ws.queryTime + effort) + prohibitionDelay(e, v, ws, ws.queryTime + effort);
        prev = e;
    }
    if (useRailRouter(v, ws)) {
        // RailwayRouter::recomputeCosts: reversal corrections
        const double vehLength = vt(v).length;
        prev = -1;
        for (int e : edges) {
            if (prev >= 0 && e < (int)net.edges.size() && net.edges[e].bidi == prev) {
                if (net.edges[e].length > vehLength) {
                    // the train does not need to drive to the end of the edge
                    const double savingsFactor = 1 - vehLength / net.edges[e].length;
                    double correction = routingEffort(prev, v, ws) + routingEffort(e, v, ws);
                    for (const std::pair<int, int>& f : rtViaSucc[prev]) {
                        if (f.first == e) {
                            correction += routeViaEffort(f.second, v, ws);
                            break;
                        }
                    }
                    effort -= savingsFactor * correction;
                }
                effort += opt.reversalPenalty;
            }
            prev = e;
        }
    }
    return effort;
}

// ---------------------------------------------------------------------------
// rerouting a vehicle (MSBaseVehicle::reroute)
// ---------------------------------------------------------------------------
// MSStop::getArrivalFallback
static Time stopArrivalFallback(const Stop& s) {
    Time result = s.arrival;
    if (result < 0) {
        result = s.until;
        if (result >= 0 && s.duration >= 0) {
            result -= s.duration;
        }
    }
    return result;
}

// MSBaseVehicle::addStopPriority
static double addStopPriority(double p1, double p2) {
    return p1 < 0 || p2 < 0 ? p1 : p1 + p2;
}

double Sim::routeCostsPos(const std::vector<int>& edges, int v, RouteWS& ws, double fromPos, double toPos, double t) const {
    const double savedTime = ws.queryTime;
    ws.queryTime = t;
    double effort = routeCosts(edges, v, ws);
    if (!edges.empty()) {
        int first = edges.front();
        int last = edges.back();
        auto len = [&](int e) { return e < (int)net.edges.size() ? net.edges[e].length : 0.; };
        if (len(first) == 0) {
            first = edges.size() > 1 && len(edges[1]) > 0 ? edges[1] : -1;
        }
        if (first >= 0 && len(last) == 0) {
            last = edges.size() > 1 && len(edges[edges.size() - 2]) > 0 ? edges[edges.size() - 2] : -1;
        }
        if (first >= 0 && last >= 0) {
            const double firstEffort = routingEffort(first, v, ws, t) + prohibitionDelay(first, v, ws, t);
            const double lastEffort = routingEffort(last, v, ws, t) + prohibitionDelay(last, v, ws, t);
            effort -= firstEffort * fromPos / len(first);
            effort -= lastEffort * (len(last) - toPos) / len(last);
        }
    }
    ws.queryTime = savedTime;
    return effort;
}

const std::vector<int>& Sim::stopAlternatives(const Veh& x, int stop) const {
    static const std::vector<int> empty;
    if (stop < 0 || x.stops[stop].busStop < 0) {
        return empty;
    }
    const BusStop& b = busStops[x.stops[stop].busStop];
    auto it = b.name.empty() ? namedStops.end() : namedStops.find({b.category, b.name});
    return it == namedStops.end() ? empty : it->second;
}

// MSStopOptimizer::optimizeSkipped: the best sequence of reachable stops (fewest skipped priorities, then most
// reached priorities, fewest changes to alternative stopping places, lowest cost), searched lazily from each stop
std::vector<int> Sim::stopOptimizeSkipped(int v, int source, double sourcePos, std::vector<StopEdgeInfo>& stops,
                                          const std::vector<int>& edges, Time maxDelay, RouteWS& ws) const {
    const Veh& x = veh[v];
    struct Node {  // StopPathNode
        StopEdgeInfo info;
        double skippedPrio = 0, reachedPrio = 0;
        int reachedMandatory = 0, trackChanges = 0;
        double cost = 0;
        int stopIndex = -1, numSkipped = 0, altIndex = -1;
        bool checked = false;
        std::vector<int> edges;
        int prev = -1;
    };
    std::vector<Node> nodes;
    auto makeNode = [&](const StopEdgeInfo& o) {
        Node n;
        n.info = StopEdgeInfo{o.edge, o.pos, o.priority, o.arrival};
        n.info.stop = o.stop;
        n.info.delay = o.delay;
        n.info.routeIndex = o.routeIndex;
        return n;
    };
    // StopPathNode::operator< (for the best final node)
    auto less = [&](int ia, int ib) {
        const Node& a = nodes[ia];
        const Node& b = nodes[ib];
        if (a.reachedMandatory == b.reachedMandatory) {
            if (a.reachedPrio == b.reachedPrio) {
                if (a.trackChanges == b.trackChanges) {
                    return a.cost > b.cost;
                }
                return a.trackChanges > b.trackChanges;
            }
            return a.reachedPrio < b.reachedPrio;
        }
        return a.reachedMandatory < b.reachedMandatory;
    };
    // spnCompare: fewer skips are explored first
    auto spnCompare = [&](int ia, int ib) {
        const Node& a = nodes[ia];
        const Node& b = nodes[ib];
        if (a.skippedPrio == b.skippedPrio) {
            if (a.stopIndex == b.stopIndex) {
                return less(ia, ib);
            }
            return a.stopIndex < b.stopIndex;
        }
        return a.skippedPrio > b.skippedPrio;
    };
    const double t = STEPS2TIME(now);
    // reachableInTime
    auto reachable = [&](int from, double fromPos, int to, double toPos, Time arrival, std::vector<int>& into, double& cost) {
        routeComputePos(from, fromPos, to, toPos, v, into, ws);
        if (!into.empty()) {
            cost = routeCostsPos(into, v, ws, fromPos, toPos, t);
            return now + TIME2STEPS(cost) <= arrival + maxDelay;
        }
        return false;
    };
    // StopPathNode::getSuccessor
    auto getSuccessor = [&](int ni, double minSkipped) {
        if (!nodes[ni].checked) {
            nodes[ni].checked = true;
            double newCost = 0;
            const Node& p = nodes[nodes[ni].prev];
            std::vector<int> into;
            const bool ok = reachable(p.info.edge, p.info.pos, nodes[ni].info.edge, nodes[ni].info.pos, nodes[ni].info.arrival, into, newCost);
            nodes[ni].edges = into;
            if (!ok) {
                nodes[ni].reachedMandatory = -1;  // failure
                return -1;
            }
            nodes[ni].reachedPrio += std::max(0., nodes[ni].info.priority);
            nodes[ni].cost += newCost;
            if (nodes[ni].info.priority < 0) {
                nodes[ni].reachedMandatory++;
            }
        }
        int nextIndex = nodes[ni].stopIndex + nodes[ni].numSkipped + 1;
        while (nextIndex < (int)stops.size()) {
            const StopEdgeInfo& next = stops[nextIndex];
            // always try the default track first
            if (nodes[ni].altIndex == -1) {
                nodes[ni].altIndex++;
                Node succ = makeNode(next);
                const Node& p = nodes[ni];
                succ.skippedPrio = p.skippedPrio;
                succ.reachedPrio = p.reachedPrio;
                succ.reachedMandatory = p.reachedMandatory;
                succ.trackChanges = p.trackChanges;
                succ.cost = p.cost;
                succ.prev = ni;
                succ.stopIndex = nextIndex;
                nodes.push_back(std::move(succ));
                return (int)nodes.size() - 1;
            }
            const std::vector<int>& alternatives = stopAlternatives(x, next.stop);
            if (next.priority < 0) {
                return -1;  // the next stop can neither be skipped nor changed
            }
            while (nodes[ni].altIndex < (int)alternatives.size()) {
                const BusStop& alt = busStops[alternatives[nodes[ni].altIndex]];
                const int altEdge = net.lanes[alt.lane].edge;
                nodes[ni].altIndex++;
                if (altEdge == next.edge || !net.lanes[alt.lane].allows(x.vclass)) {
                    continue;
                }
                Node succ = makeNode(next);
                const Node& p = nodes[ni];
                succ.trackChanges = p.trackChanges + 1;
                succ.info.pos = alt.endPos;
                succ.skippedPrio = p.skippedPrio;
                succ.reachedPrio = p.reachedPrio;
                succ.reachedMandatory = p.reachedMandatory;
                succ.cost = p.cost;
                succ.prev = ni;
                succ.stopIndex = nextIndex;
                succ.info.origEdge = next.edge;
                succ.info.edge = altEdge;
                nodes.push_back(std::move(succ));
                return (int)nodes.size() - 1;
            }
            nodes[ni].skippedPrio += next.priority;
            if (nodes[ni].skippedPrio >= minSkipped) {
                return -1;  // cannot improve on the current best solution
            }
            nextIndex++;
            nodes[ni].altIndex = -1;
            nodes[ni].numSkipped++;
        }
        return -1;
    };
    // the initial solution is an upper bound on the acceptable total skipped priority
    double minSkipped = 0;
    double totalPrio = 0;
    std::priority_queue<int, std::vector<int>, decltype(spnCompare)> queue(spnCompare);
    {
        Node n = makeNode(StopEdgeInfo{source, sourcePos, -1, now});
        n.stopIndex = -1;
        n.info.routeIndex = 0;
        n.cost = routeCostsPos(edges, v, ws, sourcePos, 0, t);
        n.checked = true;
        nodes.push_back(std::move(n));
    }
    int prev = 0;
    int bestNode = 0;
    queue.push(0);
    for (int i = 0; i < (int)stops.size(); i++) {
        const StopEdgeInfo& stop = stops[i];
        if (stop.skipped) {
            minSkipped += stop.priority;
        } else {
            Node n = makeNode(stop);
            n.skippedPrio = minSkipped;
            n.reachedPrio = nodes[prev].reachedPrio;
            n.reachedMandatory = nodes[prev].reachedMandatory;
            if (stop.priority >= 0) {
                n.reachedPrio += stop.priority;
            } else {
                n.reachedMandatory += 1;
            }
            n.checked = true;
            n.prev = prev;
            n.stopIndex = i;
            const int ri = std::max(0, stop.routeIndex);
            const std::vector<int> subEdges(edges.begin(), edges.begin() + std::min(ri, (int)edges.size()));
            n.cost = routeCostsPos(subEdges, v, ws, sourcePos, 0, t);
            const int pri = std::max(0, nodes[prev].info.routeIndex);
            if (pri <= ri && ri < (int)edges.size()) {
                n.edges.assign(edges.begin() + pri, edges.begin() + ri + 1);
            }
            nodes.push_back(std::move(n));
            prev = (int)nodes.size() - 1;
            bestNode = prev;
            queue.push(prev);
        }
        totalPrio += std::max(0.0, stop.priority);
    }
    std::map<int, int> bestIntermediate;  // edge -> node
    while (!queue.empty()) {
        const int ptr = queue.top();
        queue.pop();
        const int succ = getSuccessor(ptr, minSkipped);
        if (less(bestNode, ptr)) {
            minSkipped = std::min(minSkipped, totalPrio - nodes[ptr].reachedPrio);
            bestNode = ptr;
        }
        if (succ >= 0) {
            if (minSkipped == 0 && nodes[ptr].trackChanges > nodes[bestNode].trackChanges) {
                continue;
            }
            auto it = bestIntermediate.find(nodes[ptr].info.edge);
            if (it == bestIntermediate.end()) {
                bestIntermediate[nodes[ptr].info.edge] = ptr;
            } else if (it->second != ptr) {
                if (!less(it->second, ptr)) {
                    continue;
                }
                it->second = ptr;
            }
            queue.push(ptr);
            queue.push(succ);
        }
    }
    for (StopEdgeInfo& stop : stops) {
        // init true and later set false for all used stops
        if (!stop.skipped) {
            stop.backtracked = true;
            stop.skipped = true;
        }
    }
    std::vector<int> bestEdges;
    if (nodes[bestNode].stopIndex < 0) {
        bestEdges.push_back(source);  // all stops were skipped: the route ends on the current edge
    }
    for (int n = bestNode; n >= 0 && nodes[n].stopIndex >= 0; n = nodes[n].prev) {
        Node& b = nodes[n];
        stops[b.stopIndex].skipped = false;
        stops[b.stopIndex].backtracked = false;
        stops[b.stopIndex].origEdge = b.info.origEdge;
        if (!bestEdges.empty() && !b.edges.empty()) {
            b.edges.pop_back();
        }
        bestEdges.insert(bestEdges.begin(), b.edges.begin(), b.edges.end());
    }
    return bestEdges;
}

// MSVehicle::getRerouteOrigin: the next edge when on a junction or too close to its end
int Sim::rerouteOriginIdx(int v) const {
    const Veh& x = veh[v];
    const Kin& k = kin[v];
    // (trains always start from their current edge)
    if (k.lane >= 0 && x.routeIdx + 1 < (int)routes[x.route].size() && !isRailway(x.vclass)) {
        if (isInternal(k.lane)) {
            return x.routeIdx + 1;
        }
        if (k.pos > net.lanes[k.lane].length - cfBrakeGap(v, k.speed, vt(v).decel, 0.)) {
            return x.routeIdx + 1;
        }
    }
    return x.routeIdx;
}

// device: info "device.rerouting" (savings check), sinkEdge: a new destination (rerouters), silent: no
// replacement when the destination cannot be reached
bool Sim::reroute(int v, bool onInit, RouteWS& ws, bool device, int sinkEdge, bool silent) {
    Veh& x = veh[v];
    const std::vector<int>& route = routes[x.route];
    const int originIdx = onInit ? 0 : rerouteOriginIdx(v);
    int source = route[originIdx];
    int sink = sinkEdge >= 0 ? sinkEdge : route.back();
    // trips between zones start at the source connector and end at the sink connector (the route given by a
    // taz trip ends at the sink connector until it is routed; with-taz keeps the zone for later reroutes)
    if (onInit && x.fromTaz >= 0) {
        source = tazNode(x.fromTaz, true);
    }
    if (sinkEdge < 0 && x.toTaz >= 0 && (onInit || opt.withTaz)) {
        sink = tazNode(x.toTaz, false);
    }
    const int curIdx = onInit ? 0 : x.routeIdx;
    std::vector<int> oldRemaining(route.begin() + (source == route[curIdx] ? curIdx : curIdx + 1), route.end());
    std::vector<int> edges;
    if (source != sink || !routeProhibited(sink, v, ws)) {
        edges.push_back(source);
    }
    double sourcePos = onInit ? 0 : kin[v].pos;
    if (onInit && x.departPosMode == DP_GIVEN) {
        sourcePos = x.departPos;
    } else if (!onInit && kin[v].lane >= 0 && source != net.lanes[kin[v].lane].edge) {
        sourcePos = 0;  // routing starts on the next edge
    }
    std::vector<StopEdgeInfo> stops;
    // stop indices whose segment is a jump (getStopEdges; like SUMO, not shifted when the first stop is skipped)
    std::vector<int> jumps;
    const std::vector<int>& via = x.exr().via;
    if (via.empty()) {
        // getStopEdges: one entry per stop edge occurrence (a loop when a stop lies behind the previous one)
        // (stops on junction-internal lanes are not supported)
        int prev = -1;
        double firstPos = INVALID_DOUBLE;
        for (int i = 0; i < (int)x.stops.size(); ++i) {
            const Stop& s = x.stops[i];
            if (s.reached) {
                if (s.jump >= 0) {
                    jumps.push_back((int)stops.size());
                }
                continue;
            }
            const double stopPos = stopEndPos(v, s);
            if (prev < 0 || x.stops[prev].routeIdx != s.routeIdx
                    || (x.stops[prev].lane == s.lane && stopEndPos(v, x.stops[prev]) > stopPos)) {
                StopEdgeInfo se{route[s.routeIdx], stopPos, s.priority, stopArrivalFallback(s)};
                se.stop = i;
                stops.push_back(se);
            } else if (x.stops[prev].routeIdx == s.routeIdx) {
                stops.back().priority = addStopPriority(stops.back().priority, s.priority);
            }
            prev = i;
            if (firstPos == INVALID_DOUBLE) {
                firstPos = s.parkingArea >= 0 ? std::max(0., stopPos) : stopPos;
            }
            if (s.jump >= 0) {
                jumps.push_back((int)stops.size() - 1);
            }
        }
        if (!stops.empty()) {
            if (!isStopped(v) && x.stops.front().speed == 0) {
                sourcePos += cfBrakeGap(v, kin[v].speed, vt(v).decel, 0.);
            }
            // avoid superfluous waypoints for the first and last edge
            const int curEdge = onInit ? route[0] : route[x.routeIdx];
            const bool skipFirst = stops.front().edge == source && (source != curEdge || sourcePos <= firstPos + NUMERICAL_EPS);
            if (stops.size() == 1 && skipFirst) {
                stops.clear();
            } else if (skipFirst) {
                sourcePos = stops.front().pos;
                stops.erase(stops.begin());
            }
        }
    } else {
        // the stops on via edges give their priority, arrival and position (the first stop per edge)
        std::map<int, StopEdgeInfo> stopsOnVia;
        for (const Stop& s : x.stops) {
            const int e = route[s.routeIdx];
            auto it = stopsOnVia.find(e);
            if (it == stopsOnVia.end()) {
                stopsOnVia.insert({e, StopEdgeInfo{e, stopEndPos(v, s), s.priority, stopArrivalFallback(s)}});
            } else {
                it->second.priority = addStopPriority(it->second.priority, s.priority);
            }
        }
        for (size_t i = 0; i < via.size(); ++i) {
            const int e = via[i];
            if ((e == source && i == 0) || (e == sink && i + 1 == via.size())) {
                continue;
            }
            auto it = stopsOnVia.find(e);
            stops.push_back(it == stopsOnVia.end() ? StopEdgeInfo{e, 0} : StopEdgeInfo{e, it->second.pos, it->second.priority, it->second.arrival});
            for (const Stop& s : x.stops) {
                if (s.jump >= 0 && route[s.routeIdx] == e) {
                    jumps.push_back((int)stops.size());  // a via edge with a jump stop: the next segment
                    break;
                }
            }
        }
    }
    if ((stops.empty() && (source != sink || sourcePos > x.arrivalPos))
            || (!stops.empty() && (stops.back().edge != sink || x.arrivalPos < stops.back().pos))) {
        StopEdgeInfo se{sink, x.arrivalPos};
        se.sink = true;
        stops.push_back(se);
    }
    std::vector<int> into;
    int stopIndex = -1;
    int stopIt = 0;  // the vehicle's stop of the current target
    Time startTime = now;
    bool hasSkipped = false;
    const double origSourcePos = sourcePos;
    const int origSource = source;
    const Time maxDelay = TIME2STEPS(x.exr().crMaxDelay >= 0 ? x.exr().crMaxDelay : 7200.);  // MSTriggeredRerouter::DEFAULT_MAXDELAY
    for (StopEdgeInfo& se : stops) {
        ++stopIndex;
        if (std::find(jumps.begin(), jumps.end(), stopIndex) != jumps.end()) {
            // a jump: the stop edge follows without a route in between
            edges.push_back(se.edge);
            source = se.edge;
            continue;
        }
        into.clear();
        routeComputePos(source, sourcePos, se.edge, se.pos, v, into, ws);
        if (!into.empty()) {
            while (stopIt < (int)x.stops.size() && net.lanes[x.stops[stopIt].lane].edge != se.edge) {
                stopIt++;
            }
            startTime += TIME2STEPS(routeCostsPos(into, v, ws, sourcePos, se.pos, STEPS2TIME(startTime)));
            if (stopIt < (int)x.stops.size()) {
                const Stop& s = x.stops[stopIt];
                if (s.priority >= 0 && !device && se.arrival > 0) {
                    // skip this stop when it cannot be reached in time
                    const Time delay = startTime - se.arrival;
                    if (delay > 0 && delay > maxDelay) {
                        se.skipped = true;
                        se.delay = delay;
                        hasSkipped = true;
                        continue;
                    }
                }
                sourcePos = se.pos;
                // MSStop::getMinDuration
                startTime += s.until >= 0 ? std::max(s.duration, s.until - startTime) : s.duration;
            }
            if (!edges.empty()) {
                edges.pop_back();
            }
            edges.insert(edges.end(), into.begin(), into.end());
            if (edges.back() >= (int)net.edges.size()) {
                edges.pop_back();  // a zone connector (a via edge)
            }
            source = edges.back();
            se.routeIndex = (int)edges.size() - 1;
        } else if (se.priority >= 0) {
            se.skipped = true;
            hasSkipped = true;
        } else if (se.sink) {
            edges.clear();
            if (onInit && source == se.edge && stops.size() == 1 && sourcePos > x.arrivalPos) {
                edges.push_back(se.edge);
            }
        } else if (source == se.edge && se.stop >= 0 && x.stops[se.stop].endPos >= sourcePos) {
            edges.clear();  // no failure on a dynamically computed stop position
        } else if (!device && (!opt.ignoreRouteErrors || silent)) {
            return false;  // "no valid route to stop edge" (the rerouter keeps the route)
        } else {
            // no route to a stop or via edge (SUMO warns and continues behind it)
            edges.push_back(source);
            source = se.edge;
        }
    }
    if (hasSkipped) {
        edges = stopOptimizeSkipped(v, origSource, origSourcePos, stops, edges, maxDelay, ws);
    }
    if (v == traceVeh) {
        std::fprintf(stderr, "%.2f reroute %s onInit=%d cost=%.4f old=%.4f route:", STEPS2TIME(now), x.id.c_str(), (int)onInit,
                     routeCosts(edges, v, ws), routeCosts(oldRemaining, v, ws));
        for (int e : edges) {
            std::fprintf(stderr, " %s(%.2f)", net.edges[e].id.c_str(), rtSpeed.empty() ? -1. : rtSpeed[e]);
        }
        std::fprintf(stderr, " old:");
        for (int e : oldRemaining) {
            std::fprintf(stderr, " %s(%.2f)", net.edges[e].id.c_str(), rtSpeed.empty() ? -1. : rtSpeed[e]);
        }
        std::fprintf(stderr, "\n");
    }
    // without the taz connectors
    if (!edges.empty() && edges.front() >= (int)net.edges.size()) {
        edges.erase(edges.begin());
    }
    if (!edges.empty() && edges.back() >= (int)net.edges.size()) {
        edges.pop_back();
    }
    if (edges.empty() && silent) {
        return false;
    }
    if (!onInit && device && opt.weightsRandomFactor == 1.) {
        // MSDevice_Routing::sufficientSaving (or the old route became unusable)
        const double routeCost = routeCosts(edges, v, ws);
        const double previousCost = routeCosts(oldRemaining, v, ws);
        bool ok = routeCost == 0
                  || (previousCost / routeCost > opt.rerouteThresholdFactor && previousCost - routeCost > opt.rerouteThresholdTime);
        for (int e : oldRemaining) {
            ok |= routeProhibited(e, v, ws);
        }
        if (!ok) {
            return !edges.empty();
        }
    }
    if (!edges.empty()) {
        replaceRouteEdges(v, edges, onInit, originIdx);
    } else if (onInit) {
        x.routeError = true;  // "Vehicle has no valid route" (reported after the parallel phase)
    }
    if (onInit) {
        resolveArrival(v);
    }
    return !edges.empty();
}

// MSBaseVehicle::replaceRouteEdges + replaceRoute, MSVehicle::replaceRoute
// patchStops: the stops changed (replaceParkingArea), their route positions are recomputed also when the
// route stays the same (MSBaseVehicle::haveValidStopEdges)
bool Sim::replaceRouteEdges(int v, std::vector<int>& edges, bool onInit, int originIdx, bool patchStops) {
    Veh& x = veh[v];
    if (x.exr().fullRoute >= 0) {
        x.ex().fullRoute = -1;  // (the arrivalEdge refers to the old route)
    }
    std::vector<int>& route = routes[x.route];
    int oldSize = (int)edges.size();
    if (!onInit) {
        const int origin = route[originIdx];
        if (origin != route[x.routeIdx] && edges.front() == origin) {
            edges.insert(edges.begin(), route[x.routeIdx]);
            oldSize = (int)edges.size();
        }
        edges.insert(edges.begin(), route.begin(), route.begin() + x.routeIdx);
    }
    if (edges == route && !patchStops) {
        if (onInit && x.initReplaced) {
            x.nReroutes++;
            x.initReplaced = false;  // (the insertion may route again)
        }
        return true;
    }
    int newIdx = 0;
    if (!onInit) {
        const int offset = (int)edges.size() - oldSize;
        auto it = std::find(edges.begin() + offset, edges.end(), route[x.routeIdx]);
        if (it == edges.end()) {
            return false;
        }
        newIdx = (int)(it - edges.begin());
        const int lane = kin[v].lane;
        if (lane >= 0) {
            if (isInternal(lane)) {
                const int next = net.lanes[net.normalSuccessorLane(lane)].edge;
                if (newIdx + 1 == (int)edges.size() || edges[newIdx + 1] != next) {
                    return false;  // on a junction leading elsewhere
                }
            } else if (kin[v].pos > net.lanes[lane].length && x.routeIdx + 1 < (int)route.size()
                       && newIdx + 1 < (int)edges.size() && route[x.routeIdx + 1] != edges[newIdx + 1]) {
                return false;  // moving past the junction, committed to another successor
            }
        }
    }
    route = edges;
    x.routeIdx = newIdx;
    x.initReplaced = false;
    if (!onInit) {
        resolveArrival(v);
    }
    x.nReroutes++;
    if (railActive) {
        railNewRoute(v);  // MSNet::VehicleState::NEWROUTE
    }
    // the stops on the new route (MSBaseVehicle::replaceRoute)
    if (onInit) {
        // not driven yet: reassigned from scratch (addStops)
        for (size_t i = 0; i < x.stops.size(); ++i) {
            const int idx = stopRouteIndex(x, kin[v].type, route, x.stops[i], i == 0 ? nullptr : &x.stops[i - 1]);
            x.stops[i].routeIdx = idx < 0 ? 0 : idx;
        }
    } else {
        // recheck the old stops: the next occurrence of the edge when a loop is needed to reach them
        int searchStart = x.routeIdx;
        double lastPos = kin[v].pos + cfBrakeGap(v, kin[v].speed, vt(v).decel, 0.);
        if (kin[v].lane >= 0 && isInternal(kin[v].lane) && !x.stops.empty() && !isInternal(x.stops.front().lane)) {
            lastPos += net.edges[route[x.routeIdx]].length;
        }
        for (auto it = x.stops.begin(); it != x.stops.end();) {
            const int e = net.lanes[it->lane].edge;
            const double endPos = stopEndPos(v, *it);
            if (searchStart >= (int)route.size() || route[searchStart] != e || endPos + NUMERICAL_EPS < lastPos
                    || (endPos - NUMERICAL_EPS < lastPos && it->reached)) {
                if (searchStart < (int)route.size() && !it->reached) {
                    searchStart++;
                }
            }
            lastPos = endPos;
            int found = -1;
            for (int i = searchStart; i < (int)route.size(); ++i) {
                if (route[i] == e) {
                    found = i;
                    break;
                }
            }
            if (found < 0 && it->priority >= 0) {
                // MSBaseVehicle::replaceWithAlternative: a stopping place of the same name on the new route
                for (int alt : stopAlternatives(x, (int)(it - x.stops.begin()))) {
                    const BusStop& b = busStops[alt];
                    const int altEdge = net.lanes[b.lane].edge;
                    if (altEdge == e || !net.lanes[b.lane].allows(x.vclass)) {
                        continue;
                    }
                    for (int i = searchStart; i < (int)route.size(); ++i) {
                        if (route[i] == altEdge) {
                            found = i;
                            break;
                        }
                    }
                    if (found >= 0) {
                        // MSStop::replaceStoppingPlace
                        it->lane = b.lane;
                        it->startPos = b.startPos;
                        it->endPos = b.endPos;
                        it->busStop = alt;
                        break;
                    }
                }
            }
            if (found < 0) {
                it = x.stops.erase(it);  // not on the new route
                continue;
            }
            it->routeIdx = found;
            searchStart = found;
            ++it;
        }
    }
    x.lastBestLanesEdge = -1;
    x.lastBestLanesInternalLane = -1;
    if (onInit) {
        updateBestLanes(v, true, net.edges[route[0]].lanes[0], false);
    } else {
        updateBestLanes(v, true);
    }
    if (x.stops.empty()) {
        x.stopDist = DMAX;
    }
    if (v == traceVeh) {
        std::fprintf(stderr, "   replaced: idx=%d route:", x.routeIdx);
        for (int e : route) {
            std::fprintf(stderr, " %s", net.edges[e].id.c_str());
        }
        std::fprintf(stderr, " bestCont:");
        for (int l : bestContinuation(v)) {
            std::fprintf(stderr, " %s", l >= 0 ? net.lanes[l].id.c_str() : "-");
        }
        std::fprintf(stderr, "\n");
    }
    return true;
}

// MSDevice_Routing::reroute
void Sim::deviceReroute(int v, bool onInit, RouteWS& ws) {
    Veh& x = veh[v];
    routingInitWeights();
    if (x.rerouteLast >= rtLastAdaptation) {
        return;
    }
    x.rerouteLast = now;
    ws.staticEffort = false;
    ws.queryTime = STEPS2TIME(now);
    ws.prohibited = nullptr;
    reroute(v, onInit, ws, true);
}

// ---------------------------------------------------------------------------
// events
// ---------------------------------------------------------------------------
// MSDevice_Routing::preInsertionReroute for the vehicles due in this step (insertion events)
void Sim::preInsertionReroutes() {
    if (!rtActive) {
        return;
    }
    std::vector<int>& due = rtDue;
    due.clear();
    auto cmp = [](const std::pair<Time, int>& a, const std::pair<Time, int>& b) { return a.first > b.first; };
    const Time prePeriod = std::max((Time)0, TIME2STEPS(opt.reroutePrePeriod));
    // MSEventControl::execute: the events due in this step in heap order; recurring ones are re-added at once
    while (!preRerouteQueue.empty()) {
        const Time t = preRerouteQueue.front().first < 0 ? now : preRerouteQueue.front().first;
        if (t >= now + DELTA_T) {
            break;
        }
        std::pop_heap(preRerouteQueue.begin(), preRerouteQueue.end(), cmp);
        const int v = preRerouteQueue.back().second;
        preRerouteQueue.pop_back();
        Veh& x = veh[v];
        if (x.status != VehStatus::PENDING || x.rerouteNextPre == NO_EVENT) {
            continue;  // (descheduled)
        }
        if (x.skipRoutingAt == now) {
            pushPreReroute(v, t + DELTA_T);  // MSDevice_Routing::preInsertionReroute: retried in the next step
            continue;
        }
        x.preRank = (int)due.size();
        x.preRankStep = now;
        due.push_back(v);
        // fixed departure edge and a departure lane independent of the route: once is enough
        if (prePeriod == 0 || x.departLane != DL_BEST) {
            x.rerouteNextPre = NO_EVENT;
        } else {
            pushPreReroute(v, t + prePeriod);
        }
    }
    if (due.empty()) {
        return;
    }
    routingInitWeights();
    pool.setTag(ThreadPool::T_OTHER);
    pool.parallel_for((int)due.size(), [&](int i) {
        thread_local RouteWS ws;
        deviceReroute(due[i], true, ws);
    }, 4);
    for (int v : due) {
        if (veh[v].routeError) {
            if (!opt.ignoreRouteErrors) {
                throw std::runtime_error("Vehicle '" + veh[v].id + "' has no valid route.");
            }
            veh[v].routeError = false;
            if (veh[v].fromTaz >= 0) {
                // MSBaseVehicle::reroute: a trip from a zone without a route is not inserted (descheduleDeparture)
                std::fprintf(stderr, "Warning: Removing vehicle '%s' which has no valid route.\n", veh[v].id.c_str());
                veh[v].status = VehStatus::REMOVED;
                releaseVehicle(v);
                continue;
            }
            // --ignore-route-errors: the vehicle keeps its route (it stops where the route breaks)
            std::fprintf(stderr, "Warning: Vehicle '%s' has no valid route.\n", veh[v].id.c_str());
        }
    }
}

// MSDevice_Routing::notifyEnter(NOTIFICATION_DEPARTED)
void Sim::routingDeparted(int v, RouteWS& ws) {
    Veh& x = veh[v];
    const Time prePeriod = std::max((Time)0, TIME2STEPS(opt.reroutePrePeriod));
    if (x.rerouteNextPre == NO_EVENT && prePeriod > 0 && now - x.depart > prePeriod) {
        // pre-insertion rerouting was disabled; reroute once since the insertion was delayed
        ws.staticEffort = false;
        ws.queryTime = STEPS2TIME(now);
        ws.prohibited = nullptr;
        reroute(v, true, ws, true);
    }
    x.rerouteNextPre = NO_EVENT;
    if (x.reroutePeriod > 0) {
        Time start = now + x.reroutePeriod;
        if (opt.rerouteSynchronize) {
            start -= start % x.reroutePeriod;
        }
        x.rerouteNext = start;
        rerouteQueue.push_back({start, v});
        std::push_heap(rerouteQueue.begin(), rerouteQueue.end(), std::greater<std::pair<Time, int>>());
    }
}

// MSDevice_Routing::wrappedRerouteCommandExecute (begin of step events)
void Sim::periodicReroutes() {
    if (!rtActive || rerouteQueue.empty()) {
        return;
    }
    std::vector<int>& due = rtDue;
    due.clear();
    auto later = std::greater<std::pair<Time, int>>();
    while (!rerouteQueue.empty() && rerouteQueue.front().first < now + DELTA_T) {
        std::pop_heap(rerouteQueue.begin(), rerouteQueue.end(), later);
        const std::pair<Time, int> ev = rerouteQueue.back();
        rerouteQueue.pop_back();
        Veh& x = veh[ev.second];
        if (x.rerouteNext != ev.first || (x.status != VehStatus::RUNNING && x.status != VehStatus::TELEPORTING
                                          && x.status != VehStatus::PARKING)) {
            continue;  // descheduled (arrived)
        }
        x.rerouteNext += x.reroutePeriod;
        rerouteQueue.push_back({x.rerouteNext, ev.second});
        std::push_heap(rerouteQueue.begin(), rerouteQueue.end(), later);
        if (isStopped(ev.second) || x.status != VehStatus::RUNNING) {
            x.rerouteAfterStop = x.status == VehStatus::RUNNING;
        } else {
            due.push_back(ev.second);
        }
    }
    if (due.empty()) {
        return;
    }
    // (MSEventControl: equal times by the command's priority, the vehicle's numerical id, highest first)
    std::sort(due.begin(), due.end(), std::greater<int>());
    for (int i = 0; i < (int)due.size(); ++i) {
        veh[due[i]].preRank = i;  // (the order of their new routes, MSRailSignalControl NEWROUTE)
        veh[due[i]].preRankStep = now;
    }
    routingInitWeights();
    pool.setTag(ThreadPool::T_OTHER);
    pool.parallel_for((int)due.size(), [&](int i) {
        thread_local RouteWS ws;
        deviceReroute(due[i], false, ws);
    }, 4);
}

// MSDevice_Routing::notifyStopEnded (called from the parallel move planning: own vehicle only)
// MSDevice_Routing::notifyStopEnded never runs: the device returns false from notifyEnter at the departure
// (without separate turn weights), so the vehicle drops it from its move reminders
void Sim::routingStopEnded(int v) {
    (void)v;
}

// ---------------------------------------------------------------------------
// railway router (utils/router/RailwayRouter.h, RailEdge.h): trains may reverse on bidirectional tracks
// ---------------------------------------------------------------------------
namespace {
constexpr double REVERSAL_SLACK = POSITION_EPS + NUMERICAL_EPS;
constexpr double REVERSAL_PENALTY_FACTOR = 0.2;  // RailwayRouter::myReversalPenaltyFactor (1/v)
}

// RailwayRouter::getRailEdges / RailEdge::init (rail edge i < number of edges is network edge i)
void Sim::buildRailRouter() {
    rrEdges.clear();
    bool haveBidi = false;
    for (const Edge& e : net.edges) {
        haveBidi |= e.bidi >= 0;
    }
    if (!haveBidi) {
        return;  // MSNet::hasBidiEdges
    }
    // the network edges and the taz connectors keep their indices (numerical ids), the turnarounds follow
    const int nE = (int)net.edges.size();
    const int nNodes = (int)rtViaSucc.size();
    rrEdges.resize(nNodes);
    for (int e = 0; e < nNodes; ++e) {
        rrEdges[e].original = e;
    }
    const bool permitReversal = opt.reversalPenalty >= 0;
    for (int e = 0; e < nNodes; ++e) {
        for (const std::pair<int, int>& f : rtViaSucc[e]) {
            if (e < nE && f.first == net.edges[e].bidi) {
                if (!permitReversal) {
                    continue;
                }
                if (rrEdges[e].turnaround < 0) {
                    RailRouteEdge t;
                    t.maxLength = t.startLength = net.edges[e].length - REVERSAL_SLACK;
                    t.via.push_back({f.first, -1});
                    rrEdges[e].turnaround = (int)rrEdges.size();
                    rrEdges[e].via.push_back({rrEdges[e].turnaround, -1});
                    rrEdges.push_back(t);
                }
                rrEdges[rrEdges[e].turnaround].isVirtual = false;
                // ensure at least one virtual turnaround (at the start of the edge)
                const double initialDist = std::max(opt.railMaxTrainLength - net.edges[e].length, POSITION_EPS);
                rrAddVirtualTurns(e, f.first, initialDist, net.edges[e].length, {e});
            } else {
                rrEdges[e].via.push_back({f.first, f.second});
            }
        }
    }
}

static bool edgeConnectedTo(const std::vector<std::vector<std::pair<int, int>>>& viaSucc, int e, int dest) {
    for (const std::pair<int, int>& f : viaSucc[e]) {
        if (f.first == dest) {
            return true;
        }
    }
    return false;
}

// RailEdge::addVirtualTurns: trains longer than the edge back up over the predecessors before reversing
void Sim::rrAddVirtualTurns(int forward, int backward, double dist, double maxTrainLength, const std::vector<int>& replacement) {
    if (dist <= 0) {
        return;
    }
    const std::vector<int> preds = net.edges[forward].predecessors;
    for (int prev : preds) {
        if (prev == backward) {
            continue;
        }
        const int bidi = net.edges[prev].bidi;
        if (bidi >= 0 && edgeConnectedTo(rtViaSucc, backward, bidi)) {
            if (rrEdges[prev].turnaround < 0) {
                RailRouteEdge t;
                t.maxLength = t.startLength = net.edges[prev].length - REVERSAL_SLACK;
                t.via.push_back({bidi, -1});
                rrEdges[prev].turnaround = (int)rrEdges.size();
                rrEdges[prev].via.push_back({rrEdges[prev].turnaround, -1});
                rrEdges.push_back(t);
            }
            if (std::find(replacement.begin(), replacement.end(), prev) == replacement.end()) {
                RailRouteEdge& t = rrEdges[rrEdges[prev].turnaround];
                const double maxLength = net.edges[prev].length + maxTrainLength - REVERSAL_SLACK;
                if (maxLength > t.maxLength) {  // RailEdge::update
                    t.maxLength = maxLength;
                    t.replacement = replacement;
                }
                std::vector<int> replacement2{prev};
                replacement2.insert(replacement2.end(), replacement.begin(), replacement.end());
                rrAddVirtualTurns(prev, bidi, dist - net.edges[prev].length, maxTrainLength + net.edges[prev].length, replacement2);
            }
        }
    }
}

// RailEdge::insertOriginalEdges
void Sim::rrInsertOriginalEdges(int re, double length, std::vector<int>& into) const {
    const RailRouteEdge& r = rrEdges[re];
    if (r.original >= 0) {
        into.push_back(r.original);
        return;
    }
    double seen = r.startLength;
    int nPushed = 0;
    if (seen >= length && !r.isVirtual) {
        return;
    }
    for (int edge : r.replacement) {
        into.push_back(edge);
        nPushed++;
        seen += net.edges[edge].length - REVERSAL_SLACK;
        if (seen >= length && edgeConnectedTo(rtViaSucc, edge, net.edges[edge].bidi)) {
            break;
        }
    }
    const int last = (int)into.size() - 1;
    for (int i = 0; i < nPushed; i++) {
        into.push_back(net.edges[into[last - i]].bidi);
    }
}

// RailwayRouter::getTravelTimeStatic (the effort of the routing device)
double Sim::rrEffort(int re, int v, const RouteWS& ws) const {
    const RailRouteEdge& r = rrEdges[re];
    if (r.original >= 0) {
        return routingEffort(r.original, v, ws);
    }
    if (r.isVirtual) {
        std::vector<int> repl;
        const double length = vt(v).length;
        rrInsertOriginalEdges(re, length, repl);
        repl.pop_back();  // the last edge is not used fully
        double seenDist = 0;
        double result = 0;
        for (int e : repl) {
            result += routingEffort(e, v, ws);
            seenDist += net.edges[e].length;
        }
        const double lengthOnLastEdge = std::max(0.0, length - seenDist);
        return result + opt.reversalPenalty + lengthOnLastEdge * REVERSAL_PENALTY_FACTOR;
    }
    return opt.reversalPenalty;
}

// DijkstraRouter on the rail edges (RailEdge::prohibits, RailEdge::getViaSuccessors(vClass))
bool Sim::rrDijkstra(int from, int to, int v, std::vector<int>& into, RouteWS& ws) const {
    const double vehLength = vt(v).length;
    auto prohibits = [&](int re) {
        const RailRouteEdge& r = rrEdges[re];
        return vehLength > r.maxLength || (r.original >= 0 && routeProhibited(r.original, v, ws));
    };
    if (prohibits(from) || prohibits(to)) {
        return false;
    }
    const int n = (int)rrEdges.size();
    if ((int)ws.rrEffort.size() != n) {
        ws.rrEffort.assign(n, UNREACHED);
        ws.rrPrev.assign(n, -1);
        ws.rrVisited.assign(n, 0);
        ws.rrTouched.clear();
    }
    for (int e : ws.rrTouched) {
        ws.rrEffort[e] = UNREACHED;
        ws.rrPrev[e] = -1;
        ws.rrVisited[e] = 0;
    }
    ws.rrTouched.clear();
    ws.rrHeap.clear();
    auto cmp = [&](int a, int b) {
        if (ws.rrEffort[a] == ws.rrEffort[b]) {
            return a > b;
        }
        return ws.rrEffort[a] > ws.rrEffort[b];
    };
    const SVCPermissions vc = veh[v].vclass;
    ws.rrEffort[from] = 0;
    ws.rrTouched.push_back(from);
    ws.rrHeap.push_back(from);
    while (!ws.rrHeap.empty()) {
        const int m = ws.rrHeap.front();
        ws.rrVisited[m] = 1;
        if (m == to) {
            const size_t first = into.size();
            for (int e = m; e >= 0; e = ws.rrPrev[e]) {
                into.push_back(e);
            }
            std::reverse(into.begin() + (long)first, into.end());
            return true;
        }
        std::pop_heap(ws.rrHeap.begin(), ws.rrHeap.end(), cmp);
        ws.rrHeap.pop_back();
        const double base = ws.rrEffort[m] + rrEffort(m, v, ws)
                            + (rrEdges[m].original >= 0 ? prohibitionDelay(rrEdges[m].original, v, ws, ws.queryTime + ws.rrEffort[m]) : 0.);
        const int orig = rrEdges[m].original;
        for (const std::pair<int, int>& f : rrEdges[m].via) {
            const int succOrig = rrEdges[f.first].original;
            if (orig >= 0 && succOrig >= 0 && orig < (int)net.edges.size() && succOrig < (int)net.edges.size()
                    && net.allowedLanes(orig, succOrig, vc, veh[v].ignoreTransientPermissions()) == nullptr) {
                continue;
            }
            if (prohibits(f.first)) {
                continue;
            }
            const double effort = base + routeViaEffort(f.second, v, ws, ws.queryTime + base);
            const int s = f.first;
            const double old = ws.rrEffort[s];
            if (!ws.rrVisited[s] && effort < old) {
                ws.rrEffort[s] = effort;
                ws.rrPrev[s] = m;
                if (old == UNREACHED) {
                    ws.rrTouched.push_back(s);
                    ws.rrHeap.push_back(s);
                    std::push_heap(ws.rrHeap.begin(), ws.rrHeap.end(), cmp);
                } else {
                    std::push_heap(ws.rrHeap.begin(), std::find(ws.rrHeap.begin(), ws.rrHeap.end(), s) + 1, cmp);
                }
            }
        }
    }
    return false;
}

// RailwayRouter::_compute: a train starting on a short edge may reverse behind it (the virtual turnaround
// lies backwards along the track)
bool Sim::railRouteCompute(int from, int to, int v, std::vector<int>& into, RouteWS& ws, bool avoidUnsafeBackTracking) const {
    const double vehLength = vt(v).length;
    std::vector<double> backLengths;
    const bool connector = from >= (int)net.edges.size();  // a taz connector (no length, no predecessors)
    double backDist = opt.reversalPenalty >= 0 && !connector ? vehLength - net.edges[from].length : 0;
    int start = from;
    while (backDist > 0) {
        // getStraightPredecessor
        int prev = -1;
        if ((int)into.size() > (int)backLengths.size()) {
            prev = into[into.size() - 1 - backLengths.size()];
        } else {
            for (int cand : net.edges[start].predecessors) {
                if (!net.edges[cand].internal && net.edges[cand].bidi != start) {
                    if (prev < 0) {
                        prev = cand;
                    } else {
                        prev = -2;
                        break;
                    }
                }
            }
        }
        if (prev < 0) {
            break;
        }
        backDist -= net.edges[prev].length;
        if (avoidUnsafeBackTracking && net.edges[prev].successors.size() > 1) {
            bool foundSwitch = false;
            for (int succ : net.edges[prev].successors) {
                if (succ != start && succ != net.edges[prev].bidi) {
                    foundSwitch = true;
                    break;
                }
            }
            if (foundSwitch) {
                break;
            }
        }
        backLengths.push_back(net.edges[prev].length + (backLengths.empty() ? std::min(vehLength, net.edges[from].length) : backLengths.back()));
        start = prev;
    }
    std::vector<int> intoTmp;
    bool success = rrDijkstra(start, to, v, intoTmp, ws);
    if (success) {
        const size_t intoSize = into.size();
        int backIndex = (int)backLengths.size();
        for (int re : intoTmp) {
            if (rrEdges[re].original >= 0) {
                backIndex--;
            }
            // prevent premature reversal on a back edge (extend the train length)
            const double length = backIndex >= 0 ? backLengths[backIndex] : vehLength;
            rrInsertOriginalEdges(re, length, into);
        }
        if (!backLengths.empty()) {
            // skip the virtual back edges
            into.erase(into.begin() + (long)intoSize, into.begin() + (long)(intoSize + backLengths.size()));
            if (into[intoSize] != from) {
                if (!avoidUnsafeBackTracking) {
                    // try again with more safety (unable to make use of turn-arounds on a short edge)
                    into.erase(into.begin() + (long)intoSize, into.end());
                    return railRouteCompute(from, to, v, into, ws, true);
                }
                std::fprintf(stderr, "Warning: Railway routing failure due to turn-around on short edge '%s' for vehicle '%s'.\n",
                             net.edges[from].id.c_str(), veh[v].id.c_str());
            }
        }
        if (ws.prohibited != nullptr) {
            for (size_t i = intoSize; i < into.size(); ++i) {
                for (const RouterProhibition& p : *ws.prohibited) {
                    if (p.edge == into[i]) {
                        into.erase(into.begin() + (long)intoSize, into.end());
                        return false;
                    }
                }
            }
        }
    }
    return success;
}

}  // namespace ps
