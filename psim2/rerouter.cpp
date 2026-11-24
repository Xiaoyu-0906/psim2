// Rerouters, ported from SUMO 1.28 MSTriggeredRerouter: closingReroute (edges closed for routing, with
// permissions also for driving), closingLaneReroute, destProbReroute (incl. keepDestination and
// terminateRoute), routeProbReroute, viaProbReroute and parkingAreaReroute (parking.cpp). Not supported:
// overtakingReroute, stationReroute, closings with 'until', pos/radius/optional rerouters.
//
// A rerouter is a move reminder of the lanes of its edges: it is consulted when a vehicle enters such a
// lane and in every step while the vehicle holds the reminder. This happens inside the parallel move
// phase; a reroute only changes the vehicle's own route slot (all vehicles own one when rerouters
// exist) and draws psim2's counter-based random numbers. Parking searches and the reroutes of vehicles
// heading for a parking area read parking areas of other lanes; they wait until the phase is over.
#include <algorithm>
#include <sstream>

#include "model.hpp"

namespace ps {

// REROUTER id prob off timeThreshold vTypes|- edge,edge...
// RRINTERVAL id begin end
// RRCLOSE id edge perm (-1: unspecified) [until]   RRCLOSELANE id lane perm
// RRDEST id edge|keepDestination|terminateRoute prob   RRROUTE id route prob   RRVIA id edge prob
// RRPARK id parkingArea prob visible
void Sim::parseRerouterLine(const std::vector<std::string>& tok) {
    auto split = [](const std::string& s) {
        std::vector<std::string> out;
        std::stringstream ss(s);
        std::string w;
        while (std::getline(ss, w, ',')) {
            out.push_back(w);
        }
        return out;
    };
    if (tok[0] == "REROUTER") {
        Rerouter r;
        r.id = tok[1];
        r.prob = std::stod(tok[2]);
        r.off = tok[3] == "1";
        r.timeThreshold = TIME2STEPS(std::stod(tok[4]));
        if (tok[5] != "-") {
            r.vTypes = split(tok[5]);
        }
        for (const std::string& e : split(tok[6])) {
            r.edges.push_back(net.edgeIdx.at(e));
        }
        rerouterIdx[r.id] = (int)rerouters.size();
        rerouters.push_back(r);
        return;
    }
    Rerouter& r = rerouters.at(rerouterIdx.at(tok[1]));
    if (tok[0] == "RRINTERVAL") {
        RerouteInterval iv;
        iv.begin = std::stod(tok[2]) < 0 ? -1 : TIME2STEPS(std::stod(tok[2]));
        const double end = std::stod(tok[3]);
        iv.end = end >= 1e15 ? TIME_MAX : TIME2STEPS(end);
        if (iv.begin < TIME2STEPS(opt.begin) && iv.end > TIME2STEPS(opt.begin)) {
            iv.begin = TIME2STEPS(opt.begin);  // started before the simulation, still active
        }
        r.intervals.push_back(iv);
        return;
    }
    RerouteInterval& iv = r.intervals.back();
    if (tok[0] == "RRCLOSE") {
        const long long perm = std::stoll(tok[3]);
        iv.closed.push_back({net.edgeIdx.at(tok[2]), perm < 0 ? (SVCPermissions)0 : (SVCPermissions)perm});
        iv.closedPermSet.push_back(perm >= 0);
        // RerouteInterval::getClosed: closed for all classes in routing unless permissions are given; 'until' ends it
        const double until = tok.size() > 4 ? std::stod(tok[4]) : -1;
        iv.prohibitions.push_back({iv.closed.back().first, iv.closed.back().second,
                                   until < 0 ? std::numeric_limits<double>::infinity() : until});
        r.haveClosingUntil |= until >= 0;
    } else if (tok[0] == "RRCLOSELANE") {
        const int lane = net.laneIdx.at(tok[2]);
        iv.closedLanes.push_back({lane, (SVCPermissions)std::stoll(tok[3])});
        if (std::find(iv.closedLanesAffected.begin(), iv.closedLanesAffected.end(), net.lanes[lane].edge) == iv.closedLanesAffected.end()) {
            iv.closedLanesAffected.push_back(net.lanes[lane].edge);
        }
    } else if (tok[0] == "RRDEST" || tok[0] == "RRVIA") {
        int target;
        if (tok[2] == "keepDestination") {
            target = RR_KEEP_DESTINATION;
        } else if (tok[2] == "terminateRoute") {
            target = RR_TERMINATE_ROUTE;
        } else {
            target = net.edgeIdx.at(tok[2]);
        }
        iv.edgeProbs.push_back({target, std::stod(tok[3])});
        iv.edgeProbSum += std::stod(tok[3]);
        iv.isVia |= tok[0] == "RRVIA";
    } else if (tok[0] == "RRROUTE") {
        iv.routeProbs.push_back({routeIdx.at(tok[2]), std::stod(tok[3])});
        iv.routeProbSum += std::stod(tok[3]);
    } else if (tok[0] == "RRPARK") {
        // RandomDistributor::add: a repeated (area, visible) adds to its probability
        const std::pair<int, bool> item(parkingAreaIdx.at(tok[2]), tok[4] == "1");
        const double prob = std::stod(tok[3]);
        auto it = std::find(iv.parkAreas.begin(), iv.parkAreas.end(), item);
        if (it == iv.parkAreas.end()) {
            iv.parkAreas.push_back(item);
            iv.parkProbs.push_back(prob);
        } else {
            iv.parkProbs[it - iv.parkAreas.begin()] += prob;
        }
        iv.parkProbSum += prob;
        r.haveParkProbs = true;
    } else if (tok[0] == "RROVERTAKE") {
        // RROVERTAKE id main,edges siding,edges minSaving defer|- (the signal at the siding end: after the rail init)
        RerouteInterval::OvertakeLocation o;
        for (const std::string& e : split(tok[2])) {
            o.main.push_back(net.edgeIdx.at(e));
        }
        for (const std::string& e : split(tok[3])) {
            o.siding.push_back(net.edgeIdx.at(e));
        }
        o.minSaving = std::stod(tok[4]);
        o.defer = tok[5] == "-" ? !iv.overtakeLocations.empty() : tok[5] == "1";
        iv.overtakeLocations.push_back(o);
    } else if (tok[0] == "RRSTATION") {
        iv.stopAlternatives.push_back(busStopIdx.at(tok[2]));
    } else {
        throw std::runtime_error("unknown rerouter line " + tok[0]);
    }
}

// the step in which a begin-of-step event at time t runs (t < step + DELTA_T)
static Time eventStep(Time t) {
    return t < 0 ? t : (t / DELTA_T) * DELTA_T;
}

// variable speed signs (MSLaneSpeedTrigger): VSS id lane,lane time:speed|- ...; as lane speed edits at the
// step times (a missing speed is the speed limit of the first lane)
void Sim::initSpeedSigns() {
    for (const std::vector<std::string>& tok : speedSignLines) {
        std::vector<int> signLanes;
        std::stringstream ss(tok[2]);
        std::string l;
        while (std::getline(ss, l, ',')) {
            signLanes.push_back(net.laneIdx.at(l));
        }
        const double defaultSpeed = net.lanes[signLanes[0]].speed;
        for (size_t i = 3; i < tok.size(); ++i) {
            const size_t colon = tok[i].find(':');
            const Time t = TIME2STEPS(std::stod(tok[i].substr(0, colon)));
            const std::string sp = tok[i].substr(colon + 1);
            for (int lane : signLanes) {
                EditRequest e;
                e.kind = EditRequest::LANE_SPEED;
                e.target = lane;
                e.value = sp == "-" ? defaultSpeed : std::stod(sp);
                scheduledEdits.push_back({eventStep(t), e});
            }
        }
    }
    speedSignLines.clear();
    std::stable_sort(scheduledEdits.begin(), scheduledEdits.end(),
                     [](const auto& a, const auto& b) { return a.first < b.first; });
}

// after loading: lane reminders, permission changes of the closings (begin-of-step events as edits)
void Sim::initRerouters() {
    if (rerouters.empty()) {
        return;
    }
    laneRerouters.assign(net.lanes.size(), {});
    for (int ri = 0; ri < (int)rerouters.size(); ++ri) {
        Rerouter& r = rerouters[ri];
        for (int e : r.edges) {
            for (int l : net.edges[e].lanes) {
                laneRerouters[l].push_back(ri);
            }
        }
        for (const std::string& t : r.vTypes) {
            if (typeIdx.count(t) != 0) {
                r.vTypeIdx.push_back(typeIdx.at(t));
            }
        }
        if (r.off) {
            r.prob = 0;  // setUserMode(true), setUserUsageProbability(0)
        }
        trackAccumulatedWaiting |= r.timeThreshold > 0;
        for (RerouteInterval& iv : r.intervals) {
            for (const std::pair<int, bool>& pa : iv.parkAreas) {
                // MSParkingArea::setNumAlternatives
                parkingAreas[pa.first].numAlternatives = std::max(parkingAreas[pa.first].numAlternatives, (int)iv.parkAreas.size() - 1);
            }
            iv.permissionsAllowAll = std::find(iv.closedPermSet.begin(), iv.closedPermSet.end(), true) == iv.closedPermSet.end();
            // MSTriggeredRerouter::setPermissions at the interval begin and end
            std::vector<std::pair<int, SVCPermissions>> lanePerms;
            for (size_t i = 0; i < iv.closed.size(); ++i) {
                if (iv.closedPermSet[i]) {
                    for (int l : net.edges[iv.closed[i].first].lanes) {
                        lanePerms.push_back({l, iv.closed[i].second});
                    }
                }
            }
            lanePerms.insert(lanePerms.end(), iv.closedLanes.begin(), iv.closedLanes.end());
            for (const std::pair<int, SVCPermissions>& lp : lanePerms) {
                EditRequest close;
                close.kind = EditRequest::LANE_PERMISSIONS;
                close.transient = true;
                close.target = lp.first;
                close.value = (double)lp.second;
                scheduledEdits.push_back({eventStep(iv.begin), close});
                if (iv.end < TIME_MAX) {
                    EditRequest reopen = close;
                    reopen.value = (double)net.lanes[lp.first].permissions;
                    scheduledEdits.push_back({eventStep(iv.end), reopen});
                }
            }
            if (!lanePerms.empty()) {
                for (int e : r.edges) {
                    // rebuildAllowedTargets of the rerouter edges (their vehicles update their best lanes)
                    EditRequest refresh;
                    refresh.kind = EditRequest::EDGE_REFRESH;
                    refresh.target = e;
                    scheduledEdits.push_back({eventStep(iv.begin), refresh});
                    if (iv.end < TIME_MAX) {
                        scheduledEdits.push_back({eventStep(iv.end), refresh});
                    }
                }
            }
        }
    }
    std::stable_sort(scheduledEdits.begin(), scheduledEdits.end(),
                     [](const auto& a, const auto& b) { return a.first < b.first; });
}

// MSTriggeredRerouter::getCurrentReroute(time, vehicle)
const Sim::RerouteInterval* Sim::currentReroute(const Rerouter& r, int v) const {
    for (const RerouteInterval& iv : r.intervals) {
        if (iv.begin <= now && iv.end > now) {
            if (iv.edgeProbSum > 0 || iv.routeProbSum > 0 || iv.parkProbSum > 0 || !iv.stopAlternatives.empty()) {
                return &iv;
            }
            if (!iv.closed.empty() || !iv.closedLanesAffected.empty() || !iv.overtakeLocations.empty()) {
                // getUpcomingEdgeIDs
                const std::vector<int>& route = routes[veh[v].route];
                for (int i = veh[v].routeIdx; i < (int)route.size(); ++i) {
                    for (const std::pair<int, SVCPermissions>& c : iv.closed) {
                        if (c.first == route[i]) {
                            return &iv;
                        }
                    }
                    if (std::find(iv.closedLanesAffected.begin(), iv.closedLanesAffected.end(), route[i]) != iv.closedLanesAffected.end()) {
                        return &iv;
                    }
                }
                // (an overtaking location applies when a main edge is upcoming)
                for (const RerouteInterval::OvertakeLocation& o : iv.overtakeLocations) {
                    for (int i = veh[v].routeIdx; i < (int)route.size(); ++i) {
                        if (std::find(o.main.begin(), o.main.end(), route[i]) != o.main.end()) {
                            return &iv;
                        }
                    }
                }
            }
        }
    }
    return nullptr;
}

// RandomDistributor::get
static int drawFrom(const std::vector<std::pair<int, double>>& items, double sum, double u) {
    double prob = u * sum;
    for (const std::pair<int, double>& it : items) {
        if (prob < it.second) {
            return it.first;
        }
        prob -= it.second;
    }
    return items.back().first;
}

// MSTriggeredRerouter::triggerRouting; returns whether the vehicle keeps the reminder. parkingReroute:
// MSMoveReminder::NOTIFICATION_PARKING_REROUTE (the vehicle found its parking area full)
bool Sim::rerouterTrigger(int v, int ri, bool laneChange, bool parkingReroute) {
    const Rerouter& r = rerouters[ri];
    Veh& x = veh[v];
    if (!r.vTypes.empty() && std::find(r.vTypeIdx.begin(), r.vTypeIdx.end(), kin[v].type) == r.vTypeIdx.end()) {
        return false;
    }
    const RerouteInterval* def = currentReroute(r, v);
    if (def == nullptr) {
        return true;  // an active interval could appear later
    }
    const uint64_t draw = (uint64_t)step * 4096 + (uint64_t)ri * 2;
    if (r.prob < 1 && rand01(opt.seed, (uint64_t)v, draw, parkingReroute ? RP_PARKING : RP_REROUTER) > r.prob) {
        return false;
    }
    if (r.timeThreshold > 0 && std::max(x.waitingTime, accumulatedWaitingTime(x)) < r.timeThreshold) {
        return true;  // waiting time may be reached later
    }
    if (laneChange) {
        return false;
    }
    const int interval = (int)(def - r.intervals.data());
    // (overtaking and station reroutes read the other trains: after the move phase)
    if (deferReroutes && (def->parkProbSum > 0 || parkingStops(v) || !def->overtakeLocations.empty() || !def->stopAlternatives.empty())) {
        DeferredReroute d{v, ri, interval, parkingReroute, kin[v], x.routeIdx, execTaskOrder, execTaskSeq++, {}};
        if (hasParkingAreas() && execTaskLane >= 0 && execTaskLane < (int)laneParkingAreas.size()) {  // (lanes added by edits have none)
            for (int pa : laneParkingAreas[execTaskLane]) {
                d.ownLaneAreas.push_back({pa, parkingAreas[pa]});
            }
        }
        std::lock_guard<std::mutex> lock(deferredMutex);
        deferredReroutes.push_back(std::move(d));
        return false;
    }
    rerouterApply(v, ri, interval);
    return false;
}

// the actions of triggerRouting after the checks
void Sim::rerouterApply(int v, int ri, int interval) {
    const RerouteInterval* def = &rerouters[ri].intervals[interval];
    if (def->parkProbSum > 0) {
        parkingReroute(v, ri, *def);
        return;
    }
    if (!def->overtakeLocations.empty()) {
        overtakingReroute(v, *def);
        return;
    }
    if (!def->stopAlternatives.empty()) {
        stationReroute(v, ri, *def);  // (then the other reroute definitions apply as well)
    }
    Veh& x = veh[v];
    const uint64_t draw = (uint64_t)step * 4096 + (uint64_t)ri * 2;
    thread_local RouteWS ws;
    ws.staticEffort = !x.routingDevice;  // MSNet::getRouterTT for vehicles without a routing device
    const std::vector<int>& route = routes[x.route];
    const int lastEdge = route.back();
    if (def->routeProbSum > 0) {
        // routeProbReroute: replaceRoute with the given route (it must contain the current edge)
        const int rid = drawFrom(def->routeProbs, def->routeProbSum, rand01(opt.seed, (uint64_t)v, draw + 1, RP_REROUTER));
        std::vector<int> edges = routes[rid];
        auto it = std::find(edges.begin(), edges.end(), route[x.routeIdx]);
        if (it != edges.end()) {
            std::vector<int> rest(it, edges.end());
            replaceRouteEdges(v, rest, false, x.routeIdx);
        }
        return;
    }
    int newEdge = lastEdge;
    double newArrivalPos = -1;
    std::vector<int> closedEdges;
    for (const std::pair<int, SVCPermissions>& c : def->closed) {
        closedEdges.push_back(c.first);
    }
    const bool destUnreachable = std::find(closedEdges.begin(), closedEdges.end(), lastEdge) != closedEdges.end();
    auto upcomingClosed = [&]() {
        for (int i = x.routeIdx; i < (int)route.size(); ++i) {
            if (std::find(closedEdges.begin(), closedEdges.end(), route[i]) != closedEdges.end()) {
                return true;
            }
        }
        return false;
    };
    bool keepDestination = false;
    // with a closingReroute only vehicles that cannot reach their destination get a new one
    if (closedEdges.empty() || destUnreachable || (def->isVia && upcomingClosed())) {
        newEdge = def->edgeProbSum > 0 ? drawFrom(def->edgeProbs, def->edgeProbSum, rand01(opt.seed, (uint64_t)v, draw + 1, RP_REROUTER))
                  : lastEdge;
        if (newEdge == RR_TERMINATE_ROUTE) {
            keepDestination = true;
            newEdge = route[x.routeIdx];
            newArrivalPos = kin[v].pos;  // instant arrival
        } else if (newEdge == RR_KEEP_DESTINATION || newEdge == lastEdge) {
            newEdge = destUnreachable && def->permissionsAllowAll ? route[x.routeIdx] : lastEdge;
        }
    }
    // the closed edges are prohibited for routing (closings without permissions for everybody)
    ws.prohibited = &def->prohibitions;
    ws.queryTime = STEPS2TIME(now);
    if (def->closed.empty() || destUnreachable || upcomingClosed()) {
        if (x.ignoreTransientPermissions() && !rerouters[ri].haveClosingUntil) {
            // after learning about network changes, the driver must no longer forget them
            x.routingMode = (int8_t)(x.routingMode & ~8);
        }
        const bool canChangeDest = def->edgeProbSum > 0;
        bool ok = reroute(v, false, ws, false, newEdge, canChangeDest);
        if (!ok && !keepDestination && canChangeDest) {
            // destination unreachable due to closed intermediate edges: try the other targets
            std::vector<std::pair<int, double>> alt = def->edgeProbs;
            double sum = def->edgeProbSum;
            uint64_t k = 2;
            auto removeTarget = [&](int t) {
                for (auto it = alt.begin(); it != alt.end(); ++it) {
                    if (it->first == t) {
                        sum -= it->second;
                        alt.erase(it);
                        break;
                    }
                }
            };
            removeTarget(newEdge);
            while (!ok && !alt.empty()) {
                newEdge = drawFrom(alt, sum, rand01(opt.seed, (uint64_t)v, draw + k++, RP_REROUTER));
                removeTarget(newEdge);
                if (newEdge == RR_TERMINATE_ROUTE) {
                    newEdge = routes[x.route][x.routeIdx];
                    newArrivalPos = kin[v].pos;
                    x.stops.clear();
                }
                if (newEdge == RR_KEEP_DESTINATION && !def->permissionsAllowAll) {
                    newEdge = lastEdge;
                    break;
                }
                if (newEdge == RR_KEEP_DESTINATION) {
                    continue;
                }
                ok = reroute(v, false, ws, false, newEdge, true);
            }
        }
        if (!def->isVia && ok && newArrivalPos != -1) {
            x.arrivalPos = newArrivalPos;
        }
    }
    if (def->isVia && newEdge != lastEdge && newEdge >= 0) {
        // only a via: route to the via edge, then on to the old destination
        std::vector<int> edges;
        routeCompute(newEdge, lastEdge, v, edges, ws);
        replaceRouteEdges(v, edges, false, rerouteOriginIdx(v));
    }
    ws.prohibited = nullptr;
}

// the lane reminders of a lane the vehicle entered (activateReminders); laneChange: entered by a lane change
void Sim::rerouterEnter(int v, int lane, bool laneChange) {
    if (laneRerouters.empty() || laneRerouters[lane].empty()) {
        return;
    }
    Veh& x = veh[v];
    for (int ri : laneRerouters[lane]) {
        if (rerouterTrigger(v, ri, laneChange)) {
            x.ex().rrActive.push_back(ri);
        }
    }
}

// MSVehicle::WaitingTimeCollector::passTime (--waiting-time-memory 100)
static constexpr Time WAITING_TIME_MEMORY = 100000;

void Sim::passWaitingTime(Veh& x, bool waiting) const {
    if (!waiting && x.exr().waitIntervals.empty()) {
        return;
    }
    std::vector<std::pair<Time, Time>>& w = x.ex().waitIntervals;
    const bool startNewInterval = w.empty() || w.back().first != 0;
    int cut = -1;
    for (int i = (int)w.size() - 1; i >= 0; --i) {
        w[i].first += DELTA_T;
        if (w[i].first >= WAITING_TIME_MEMORY) {
            cut = i;
            break;
        }
        w[i].second += DELTA_T;
    }
    if (cut >= 0) {
        w.erase(w.begin(), w.begin() + cut + 1);
    }
    if (!waiting) {
        return;
    }
    if (!startNewInterval) {
        w.back().first = 0;
    } else {
        w.push_back({0, DELTA_T});
    }
}

// MSVehicle::getAccumulatedWaitingTime (cumulatedWaitingTime)
Time Sim::accumulatedWaitingTime(const Veh& x) const {
    Time total = 0;
    for (auto it = x.exr().waitIntervals.rbegin(); it != x.exr().waitIntervals.rend(); ++it) {
        if (it->second >= WAITING_TIME_MEMORY) {
            if (it->first >= WAITING_TIME_MEMORY) {
                break;
            }
            total += WAITING_TIME_MEMORY - it->first;
        } else {
            total += it->second - it->first;
        }
    }
    return total;
}

// the deferred rerouter actions of the move phase, in vehicle order
void Sim::applyDeferredReroutes() {
    if (deferredReroutes.empty()) {
        return;
    }
    std::vector<DeferredReroute> todo;
    todo.swap(deferredReroutes);
    std::sort(todo.begin(), todo.end(), [](const DeferredReroute& a, const DeferredReroute& b) {
        return a.taskOrder != b.taskOrder ? a.taskOrder < b.taskOrder : a.seq < b.seq;
    });
    std::vector<const ParkingArea*> view(parkingAreas.size());
    for (const DeferredReroute& d : todo) {
        Veh& x = veh[d.veh];
        if (x.status != VehStatus::RUNNING) {
            continue;
        }
        for (int p = 0; p < (int)parkingAreas.size(); ++p) {
            const int lane = parkingAreas[p].lane;
            const int order = laneExecStep[lane] == step ? laneExecOrder[lane] : -1;
            view[p] = &parkingAreas[p];
            if (order > d.taskOrder && paStartStep[p] == step) {
                view[p] = &paStart[p];
            } else if (order == d.taskOrder) {
                for (const auto& own : d.ownLaneAreas) {
                    if (own.first == p) {
                        view[p] = &own.second;
                    }
                }
            }
        }
        paView = parkingAreas.empty() ? nullptr : &view;
        if (!d.preMove) {
            rerouterApply(d.veh, d.rerouter, d.interval);
            paView = nullptr;
            continue;
        }
        // SUMO reroutes in processNextStop, before the vehicle moves: the state of that moment, then the move
        // continues on the new route (MSVehicle::enterLaneAtMove advances the route index)
        const Kin post = kin[d.veh];
        const int advanced = x.routeIdx - d.routeIdx;
        const int nReroutes = x.nReroutes;
        kin[d.veh] = d.kin;
        x.routeIdx = d.routeIdx;
        rerouterApply(d.veh, d.rerouter, d.interval);
        paView = nullptr;
        kin[d.veh] = post;
        x.routeIdx = std::min(x.routeIdx + advanced, (int)routes[x.route].size() - 1);
        if (x.nReroutes != nReroutes) {
            // the best lanes of the lane after the move (updateBestLanes at the end of executeMove)
            x.lastBestLanesEdge = -1;
            x.lastBestLanesInternalLane = -1;
            updateBestLanes(d.veh, true);
        }
    }
}

// notifyMove of the rerouters the vehicle holds (after the move, every step)
void Sim::rerouterMove(int v) {
    std::vector<int>& rr = veh[v].ex().rrActive;
    for (size_t i = 0; i < rr.size();) {
        if (rerouterTrigger(v, rr[i], false)) {
            ++i;
        } else {
            rr.erase(rr.begin() + (long)i);
        }
    }
}

}  // namespace ps

// ---------------------------------------------------------------------------
// rail: overtakingReroute and stationReroute (MSTriggeredRerouter)
// ---------------------------------------------------------------------------
namespace ps {

// MSTriggeredRerouter::getRailSignal: the rail signal at the end of the edge (-1: none)
int Sim::edgeRailSignal(int e) const {
    const int j = net.edges[e].toJ;
    if (j < 0 || net.junctions[j].type != "rail_signal" || tlsRail.empty()) {
        return -1;
    }
    for (int l : net.lanes[net.edges[e].lanes.front()].links) {
        const Link& k = net.links[l];
        if (k.tls >= 0 && tlsRail[k.tls] >= 0) {
            return tlsRail[k.tls];
        }
    }
    return -1;
}

// MSTriggeredRerouter::findSignal: the last rail signal of edges[begin, end)
int Sim::findRailSignal(const std::vector<int>& edges, int begin, int end) const {
    for (int i = end - 1; i >= begin; --i) {
        const int s = edgeRailSignal(edges[i]);
        if (s >= 0) {
            return s;
        }
    }
    return -1;
}

// the signal at the end of each siding and its usable length (after the rail signals exist)
void Sim::initOvertakeLocations() {
    for (Rerouter& r : rerouters) {
        for (RerouteInterval& iv : r.intervals) {
            for (RerouteInterval::OvertakeLocation& o : iv.overtakeLocations) {
                o.sidingExit = findRailSignal(o.siding, 0, (int)o.siding.size());
                if (o.sidingExit < 0) {
                    throw std::runtime_error("The siding within rerouter '" + r.id + "' does not have a rail signal.");
                }
                o.sidingLength = 0;
                for (int e : o.siding) {
                    o.sidingLength += net.edges[e].length;
                    if (net.edges[e].to == railSignals[o.sidingExit].id) {
                        break;
                    }
                }
            }
        }
    }
}

// MSEdge::getMinimumTravelTime
double Sim::edgeMinTravelTime(int e, int v) const {
    return net.edges[e].length / laneVehMaxSpeed(net.edges[e].lanes[0], v) + (rtPenalty.empty() ? 0. : rtPenalty[e]);
}

// MSVehicle::estimateTimeToNextStop (INVALID_DOUBLE without stops)
double Sim::estimateTimeToNextStop(int v) const {
    return estimateTimeDistToNextStop(v).first;
}

// MSVehicle::estimateTimeToNextStop: the time and the distance
std::pair<double, double> Sim::estimateTimeDistToNextStop(int v) const {
    const Veh& x = veh[v];
    if (x.stops.empty()) {
        return {INVALID_DOUBLE, INVALID_DOUBLE};
    }
    const std::vector<int>& route = routes[x.route];
    const int lane = kin[v].lane >= 0 ? kin[v].lane : net.edges[curEdge(v)].lanes[0];
    const Stop& stop = x.stops.front();
    const Lane& L = net.lanes[lane];
    const Lane& SL = net.lanes[stop.lane];
    const double pos = kin[v].pos;
    // drive to the end of the current edge, then to the stop edge, then to the stop position
    double dist = L.length - pos;
    double travelTime = edgeMinTravelTime(L.edge, v) * dist / L.length;
    int it = x.routeIdx + 1;
    while (it < (int)route.size() && it < stop.routeIdx) {
        travelTime += edgeMinTravelTime(route[it], v);
        dist += net.edges[route[it]].length;
        it++;
    }
    const double stopEdgeDist = stop.endPos - (lane == stop.lane ? L.length : 0);
    dist += stopEdgeDist;
    travelTime += edgeMinTravelTime(SL.edge, v) * (stopEdgeDist / SL.length);
    // time loss by acceleration and deceleration (the maximum speed is limited by the distance)
    const VType& t = vt(v);
    const double a = t.accel;
    const double b = t.decel;
    const double c = kin[v].speed;
    const double d = dist;
    const double len = t.length;
    const double vs = std::min(std::max(stop.speed, 0.0), laneVehMaxSpeed(stop.lane, v));
    const double maxVD = std::max(c, ((std::sqrt(std::max(0.0, std::pow(2 * c * b, 2) + (4 * ((b * ((a * (2 * d * (b + a) + (vs * vs) - (c * c))) - (b * (c * c))))
                                                                                                 + std::pow((a * vs), 2))))) * 0.5) + (c * b)) / (b + a));
    it = x.routeIdx;
    double v0 = c;
    bool v0Stable = kin[v].accel == 0 && v0 > 0;
    double timeLossAccel = 0, timeLossDecel = 0, timeLossLength = 0;
    while (it < (int)route.size() && it <= stop.routeIdx) {
        const double vv = std::min(maxVD, laneVehMaxSpeed(net.edges[route[it]].lanes[0], v));
        const double edgeLength = (it == stop.routeIdx ? stop.endPos : net.edges[route[it]].length) - (it == x.routeIdx ? pos : 0);
        if (edgeLength <= len && v0Stable && v0 < vv) {
            const double lengthDist = std::min(len, edgeLength);
            timeLossLength += lengthDist / v0 - lengthDist / vv;
        }
        if (edgeLength > len) {
            const double dv = vv - v0;
            if (dv > 0) {
                timeLossAccel += dv / a - dv * (vv + v0) / (2 * a * vv);
            } else if (dv < 0) {
                timeLossDecel += -dv / b + dv * (vv + v0) / (2 * b * v0);
            }
            v0 = vv;
            v0Stable = true;
        }
        it++;
    }
    // the final deceleration to the stop (or the change to the waypoint speed)
    const double dv = vs - v0;
    if (dv > 0) {
        timeLossAccel += dv / a - dv * (vs + v0) / (2 * a * vs);
    } else if (dv < 0) {
        timeLossDecel += -dv / b + dv * (vs + v0) / (2 * b * v0);
    }
    return {std::max(0.0, travelTime + timeLossAccel + timeLossDecel + timeLossLength), dist};
}

// MSVehicle::slowDownForSchedule: the speed that arrives at the next stop at its planned arrival
// (speedFactorPremature)
double Sim::slowDownForSchedule(int v, double vMinComfortable) const {
    const VType& t = vt(v);
    const Stop& stop = veh[v].stops.front();
    const std::pair<double, double> timeDist = estimateTimeDistToNextStop(v);
    double arrivalDelay = STEPS2TIME(now) + timeDist.first - STEPS2TIME(stop.arrival);
    double tt = STEPS2TIME(stop.arrival - now);
    if (stop.flexArrival >= 0) {
        arrivalDelay += STEPS2TIME(stop.arrival - stop.flexArrival);
        tt = STEPS2TIME(stop.flexArrival - now);
    } else if (stop.startedGiven >= 0 && opt.useStopStarted) {
        arrivalDelay += STEPS2TIME(stop.arrival - stop.startedGiven);
        tt = STEPS2TIME(stop.startedGiven - now);
    }
    if (arrivalDelay < 0 && t.speedFactorPremature < veh[v].speedFactor) {
        const double vSlowDownMin = std::max(net.lanes[kin[v].lane].speed * t.speedFactorPremature, vMinComfortable);
        const double s = timeDist.second;
        const double b = t.decel;
        // x: the speed for arriving in tt seconds (constant speed, then braking)
        const double radicand = 4 * tt * tt * b * b - 8 * s * b;
        const double xs = radicand >= 0 ? tt * b - std::sqrt(radicand) * 0.5 : vSlowDownMin;
        return xs < vSlowDownMin ? vSlowDownMin : xs;
    }
    return vehMaxSpeed(v);
}

// MSVehicle::getStopDelay (-1: no stop with 'until')
double Sim::stopDelay(int v) const {
    const Veh& x = veh[v];
    if (x.stops.empty() || x.stops.front().until < 0) {
        return -1;
    }
    const Stop& stop = x.stops.front();
    Time estimatedDepart = now - DELTA_T;
    if (stop.reached) {
        return STEPS2TIME(estimatedDepart + stop.duration - stop.until);
    }
    if (stop.duration > 0) {
        estimatedDepart += stop.duration;
    }
    estimatedDepart += TIME2STEPS(estimateTimeToNextStop(v));
    return std::max(0.0, STEPS2TIME(estimatedDepart - stop.until));
}

// MSVehicle::getStopArrivalDelay (INVALID_DOUBLE: no stop with 'arrival')
double Sim::stopArrivalDelay(int v) const {
    const Veh& x = veh[v];
    if (x.stops.empty() || x.stops.front().arrival < 0) {
        return INVALID_DOUBLE;
    }
    const Stop& stop = x.stops.front();
    if (stop.reached) {
        return STEPS2TIME(stop.started - stop.arrival);
    }
    return STEPS2TIME(now) + estimateTimeToNextStop(v) - STEPS2TIME(stop.arrival);
}

// MSTriggeredRerouter::overtakingTrain: the faster train (with the signal it passes last before leaving the
// main track) for which waiting in the siding saves the most; vehicles in id order (MSVehicleControl's map)
std::pair<int, int> Sim::overtakingTrain(int v, int mainStart, const RerouteInterval::OvertakeLocation& oloc, double& netSaving) {
    const std::vector<int>& route = routes[veh[v].route];
    const std::vector<int>& main = oloc.main;
    const double vMax = vehMaxSpeed(v);
    const double prio = veh[v].exr().otPrio >= 0 ? veh[v].exr().otPrio : 0.001;  // DEFAULT_PRIO_OVERTAKEN
    if (vehIdOrder.size() != veh.size()) {
        vehIdOrder.resize(veh.size());
        for (int u = 0; u < (int)veh.size(); ++u) {
            vehIdOrder[u] = u;
        }
        std::sort(vehIdOrder.begin(), vehIdOrder.end(), [&](int a, int b) { return veh[a].id < veh[b].id; });
    }
    for (int u : vehIdOrder) {
        if (veh[u].status != VehStatus::RUNNING || vehMaxSpeed(u) <= vMax) {
            continue;
        }
        const double arrivalDelay = stopArrivalDelay(u);
        const double delay = std::max(stopDelay(u), arrivalDelay == INVALID_DOUBLE ? 0 : arrivalDelay);
        if (delay > (veh[u].exr().otMaxDelay >= 0 ? veh[u].exr().otMaxDelay : 7200.)) {  // DEFAULT_MAXDELAY
            continue;
        }
        const std::vector<int>& route2 = routes[veh[u].route];
        const int cur2 = veh[u].routeIdx;
        int itOnMain2 = (int)route2.size();
        int mainIndex = 0;
        for (int m : main) {
            itOnMain2 = (int)(std::find(route2.begin() + cur2, route2.end(), m) - route2.begin());
            if (itOnMain2 != (int)route2.size()) {
                break;
            }
            mainIndex++;
        }
        if (itOnMain2 == (int)route2.size() || itOnMain2 <= cur2) {
            continue;
        }
        int itOnMain = mainStart + mainIndex;
        double timeToMain = 0;
        for (int it = veh[v].routeIdx; it != itOnMain; it++) {
            timeToMain += edgeMinTravelTime(route[it], v);
        }
        // the other train may be anywhere on its current edge
        double timeToMain2 = -edgeMinTravelTime(route2[cur2], u) * kin[u].pos / net.edges[route2[cur2]].length;
        double timeToLastSignal2 = timeToMain2;
        for (int it = cur2; it != itOnMain2; it++) {
            timeToMain2 += edgeMinTravelTime(route2[it], u);
            if (edgeRailSignal(route2[it]) >= 0) {
                timeToLastSignal2 = timeToMain2;
            }
        }
        double exitMainTime = timeToMain;
        double exitMainBlockTime2 = timeToMain2;
        double commonTime = 0, commonTime2 = 0;
        int nCommon = 0;
        int exitMain2 = itOnMain2;
        int firstAfterMain = -1;
        int common = -1;
        double vMinCommon = laneVehMaxSpeed(net.edges[route[itOnMain]].lanes[0], v);
        double vMinCommon2 = laneVehMaxSpeed(net.edges[route2[itOnMain2]].lanes[0], u);
        while (itOnMain2 != (int)route2.size() && itOnMain != (int)route.size() && route[itOnMain] == route2[itOnMain2]) {
            common = route[itOnMain];
            commonTime += edgeMinTravelTime(common, v);
            commonTime2 += edgeMinTravelTime(common, u);
            vMinCommon = std::min(vMinCommon, laneVehMaxSpeed(net.edges[common].lanes[0], v));
            vMinCommon2 = std::min(vMinCommon2, laneVehMaxSpeed(net.edges[common].lanes[0], u));
            const bool onMain = nCommon < (int)main.size() - mainIndex;
            if (onMain) {
                exitMainTime = timeToMain + commonTime;
            }
            if (firstAfterMain < 0) {
                exitMainBlockTime2 = timeToMain2 + commonTime2;
            }
            const int signal = edgeRailSignal(common);
            if (signal >= 0 && !onMain && firstAfterMain < 0) {
                firstAfterMain = signal;
            }
            nCommon++;
            itOnMain++;
            itOnMain2++;
        }
        const double vMaxLast = laneVehMaxSpeed(net.edges[common].lanes[0], v);
        const double vMaxLast2 = laneVehMaxSpeed(net.edges[common].lanes[0], u);
        commonTime += vt(v).length / vMaxLast;
        exitMainBlockTime2 += vt(u).length / vMaxLast2;
        exitMain2 += std::min(nCommon, (int)main.size() - mainIndex);
        const double timeLoss2 = std::max(0.0, timeToMain + vt(v).length / laneVehMaxSpeed(net.edges[oloc.siding.front()].lanes[0], v)
                                          - timeToLastSignal2);
        const double saving = timeToMain + commonTime - (timeToMain2 + commonTime2) - timeLoss2;
        const double loss = exitMainBlockTime2 - exitMainTime;
        const double prio2 = veh[u].exr().otPrio >= 0 ? veh[u].exr().otPrio : 1.;  // DEFAULT_PRIO_OVERTAKER
        // losses from accelerating after stopping at a signal
        const double accelTimeLoss = loss > 0 ? 0.5 * vMinCommon / vt(v).accel : 0;
        const double accelTimeLoss2 = timeLoss2 > 0 ? 0.5 * vMinCommon2 / vt(u).accel : 0;
        netSaving = prio2 * (saving - accelTimeLoss2) - prio * (loss + accelTimeLoss);
        if (netSaving > oloc.minSaving) {
            const int s = findRailSignal(route2, cur2, exitMain2);
            if (s >= 0) {
                return {u, s};
            }
        }
    }
    return {-1, -1};
}

// MSTriggeredRerouter::checkStopSwitch (stationReroute): when the next stop is occupied at the train's
// (scheduled and estimated) arrival, it switches to the best free alternative (MSBaseVehicle::replaceStop)
void Sim::stationReroute(int v, int ri, const RerouteInterval& def) {
    Veh& x = veh[v];
    if (x.stops.empty()) {
        return;
    }
    const Stop& stop = x.stops.front();
    if (stop.reached || (stop.arrival < 0 && stop.until < 0)) {
        return;
    }
    const int cur = stop.busStop;
    if (cur < 0 || std::find(def.stopAlternatives.begin(), def.stopAlternatives.end(), cur) == def.stopAlternatives.end()) {
        return;
    }
    std::vector<int> blocked;  // myBlockedStoppingPlaces
    auto stopDuration = [&](int u) { return isStopped(u) ? veh[u].stops.front().duration : (Time)0; };  // getStopDuration
    Time stoppedDuration = -1;
    const BusStop& cs = busStops[cur];
    if (cs.endPositions.empty()) {
        // trains that will stop on this lane before ego arrives
        if (vehIdOrder.size() != veh.size()) {
            vehIdOrder.resize(veh.size());
            for (int u = 0; u < (int)veh.size(); ++u) {
                vehIdOrder[u] = u;
            }
            std::sort(vehIdOrder.begin(), vehIdOrder.end(), [&](int a, int b) { return veh[a].id < veh[b].id; });
        }
        for (int u : vehIdOrder) {
            if (veh[u].status == VehStatus::RUNNING && !veh[u].stops.empty() && veh[u].stops.front().lane == cs.lane) {
                if (std::find(blocked.begin(), blocked.end(), cur) == blocked.end()) {
                    blocked.push_back(cur);
                }
                if (isStopped(u)) {
                    stoppedDuration = std::max({(Time)0, stoppedDuration, stopDuration(u)});
                } else {
                    const Time timeTo = TIME2STEPS(estimateTimeToNextStop(u));
                    const Stop& us = veh[u].stops.front();
                    const Time minDur = us.until >= 0 ? std::max(us.duration, us.until - (now + timeTo)) : us.duration;  // getMinDuration
                    stoppedDuration = std::max({(Time)0, stoppedDuration, timeTo + minDur});
                }
            }
        }
    } else {
        stoppedDuration = 0;
        for (const auto& item : cs.endPositions) {
            stoppedDuration = std::max(stoppedDuration, stopDuration(item.first));
        }
    }
    if (stoppedDuration < 0) {
        return;
    }
    const Time stopFree = now + stoppedDuration;
    const Time scheduledArrival = stop.arrival >= 0 ? stop.arrival : stop.until - stop.duration;
    if (stopFree < scheduledArrival) {
        return;  // no conflict according to the schedule
    }
    const Time estimatedArrival = now + (stop.arrival >= 0 ? TIME2STEPS(stopArrivalDelay(v)) : TIME2STEPS(stopDelay(v)) - stop.duration);
    if (stopFree < estimatedArrival) {
        return;  // no conflict with the current delay
    }
    thread_local RouteWS ws;
    ws.staticEffort = !x.routingDevice;
    ws.prohibited = nullptr;
    ws.queryTime = STEPS2TIME(now);
    const int alternative = rerouteStation(v, ri, def, blocked, ws);
    if (alternative < 0 || alternative == cur) {
        return;  // (replaceStop on the same lane and position only replaces the stop attributes)
    }
    // MSBaseVehicle::replaceStop (nextStopIndex 0): to the new stop, then to the following stop or the destination
    const BusStop& nb = busStops[alternative];
    const std::vector<int> oldEdges = routes[x.route];
    const int n = (int)x.stops.size();
    const int junctionOffset = kin[v].lane >= 0 && isInternal(kin[v].lane) ? 1 : 0;
    const int itStart = x.routeIdx + junctionOffset;
    const double startPos = kin[v].pos;
    const int itEnd = n == 1 ? (int)oldEdges.size() - 1 : x.stops[1].routeIdx;
    const double endPos = n == 1 ? x.arrivalPos : x.stops[1].endPos;
    const int stopEdge = net.lanes[nb.lane].edge;
    const bool newDestination = n == 1 && stop.routeIdx == (int)oldEdges.size() - 1;
    std::vector<int> toNewStop, fromNewStop;
    routeComputePos(oldEdges[itStart], startPos, stopEdge, nb.endPos, v, toNewStop, ws);
    if (toNewStop.empty()) {
        return;
    }
    if (!newDestination) {
        routeComputePos(stopEdge, nb.endPos, oldEdges[itEnd], endPos, v, fromNewStop, ws);
        if (fromNewStop.empty()) {
            return;
        }
    }
    Stop& s = x.stops.front();
    s.lane = nb.lane;
    s.busStop = alternative;
    s.startPos = nb.startPos;
    s.endPos = nb.endPos;
    std::vector<int> newEdges(oldEdges.begin() + x.routeIdx, oldEdges.begin() + itStart);
    newEdges.insert(newEdges.end(), toNewStop.begin(), toNewStop.end() - 1);
    if (!newDestination) {
        newEdges.insert(newEdges.end(), fromNewStop.begin(), fromNewStop.end() - 1);
        newEdges.insert(newEdges.end(), oldEdges.begin() + itEnd, oldEdges.end());
    } else {
        newEdges.push_back(stopEdge);
    }
    replaceRouteEdges(v, newEdges, false, x.routeIdx, true);
}

// MSTriggeredRerouter::triggerRouting with overtakeLocations: the train waits in the siding until the faster one
// has passed (a predecessor constraint at the siding's exit signal)
void Sim::overtakingReroute(int v, const RerouteInterval& def) {
    Veh& x = veh[v];
    const std::vector<int> oldEdges = routes[x.route];
    double bestSavings = -std::numeric_limits<double>::max();
    double netSaving = 0;
    int bestIndex = -1, bestMainStart = -1;
    std::pair<int, int> best(-1, -1);
    // the locations by descending distance to the vehicle
    std::vector<std::pair<int, int>> sortedLocs;
    for (int index = 0; index < (int)def.overtakeLocations.size(); ++index) {
        const RerouteInterval::OvertakeLocation& o = def.overtakeLocations[index];
        if (vt(v).length > o.sidingLength) {
            continue;
        }
        const int mainStart = (int)(std::find(oldEdges.begin() + x.routeIdx, oldEdges.end(), o.main.front()) - oldEdges.begin());
        if (mainStart == (int)oldEdges.size() || mainStart + (int)o.main.size() > (int)oldEdges.size()
                || !std::equal(o.main.begin(), o.main.end(), oldEdges.begin() + mainStart)
                || (!x.stops.empty() && x.stops.front().routeIdx < mainStart + (int)o.main.size())) {
            continue;  // the route leaves the main track or a stop is on it
        }
        sortedLocs.push_back({-(mainStart - x.routeIdx), index});
    }
    std::sort(sortedLocs.begin(), sortedLocs.end());
    for (const auto& item : sortedLocs) {
        const RerouteInterval::OvertakeLocation& o = def.overtakeLocations[item.second];
        const int mainStart = x.routeIdx - item.first;
        const std::pair<int, int> overtaker = overtakingTrain(v, mainStart, o, netSaving);
        if (overtaker.first >= 0 && netSaving > bestSavings) {
            bestSavings = netSaving;
            bestIndex = item.second;
            best = overtaker;
            bestMainStart = mainStart;
        }
    }
    if (bestIndex < 0) {
        return;
    }
    const RerouteInterval::OvertakeLocation& o = def.overtakeLocations[bestIndex];
    if (o.defer) {
        return;
    }
    std::vector<int> newEdges(oldEdges.begin() + x.routeIdx, oldEdges.begin() + bestMainStart);
    newEdges.insert(newEdges.end(), o.siding.begin(), o.siding.end());
    newEdges.insert(newEdges.end(), oldEdges.begin() + bestMainStart + o.main.size(), oldEdges.end());
    replaceRouteEdges(v, newEdges, false, x.routeIdx);
    railAddConstraint(o.sidingExit, x.id, RailConstraint::PREDECESSOR, best.second, veh[best.first].id, 100, true);
}

}  // namespace ps
