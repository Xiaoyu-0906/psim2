// Vehicle insertion. Ported from SUMO 1.28 MSInsertionControl::emitVehicles,
// MSEdge::insertVehicle / getDepartLane / getFreeLane, MSLane::insertVehicle /
// isInsertionSuccess / safeInsertionSpeed / checkFailure / getDepartSpeed and
// MSVehicle::enterLaneAtInsertion / computeFurtherLanes.
//
// Insertion runs single-threaded after lane changing (as in SUMO).
#include "model.hpp"
#include "sublanequery.hpp"

namespace ps {

// MSLane::checkFailure: check is the InsertionCheck the failure belongs to
// (error: the failure deschedules the vehicle, MSLane::checkFailure with an error message)
bool Sim::checkFailure(int v, double& speed, double& dist, double nspeed, bool patchSpeed, int check, bool error) {
    if (nspeed < speed) {
        if (patchSpeed) {
            speed = std::min(nspeed, speed);
            dist = cfBrakeGap(v, speed) + vt(v).minGap;
        } else if (speed > 0) {
            if ((insertionChecksOf(v) & check) == 0) {
                return false;
            }
            if (opt.emergencyInsert && 0.5 * speed * speed / vt(v).emergencyDecel <= dist) {
                // the vehicle may stop in time with emergency deceleration
                std::fprintf(stderr, "Warning: Vehicle '%s' is inserted in an emergency situation, time=%.2f.\n", veh[v].id.c_str(), STEPS2TIME(now));
                return false;
            }
            if (error) {
                std::fprintf(stderr, "Error: Vehicle '%s' will not be able to depart with speed %.2f, time=%.2f.\n", veh[v].id.c_str(), speed,
                             STEPS2TIME(now));
                descheduled.push_back(v);
            }
            return true;
        }
    }
    return false;
}

double Sim::safeInsertionSpeed(int v, int lane, double seen, int leader, double speed) const {
    double nspeed = speed;
    if (leader >= 0) {
        const double gap = backPositionOnLane(leader, lane) + seen - vt(v).minGap;
        if (gap < 0) {
            return (insertionChecksOf(v) & IC_COLLISION) != 0 ? INVALID_SPEED : 0;
        }
        nspeed = std::min(nspeed, cfInsertionFollowSpeed(v, speed, gap, kin[leader].speed, vt(leader).decel, leader));
    }
    return nspeed;
}

// MSLane::safeInsertionSpeed with the leaders per sublane
double Sim::safeInsertionSpeedSL(int v, int lane, double seen, const LeaderInfo& leaders, double speed) const {
    double nspeed = speed;
    for (int i = 0; i < leaders.numSublanes(); ++i) {
        const int leader = leaders[i];
        if (leader >= 0) {
            const double gap = backPositionOnLane(leader, lane) + seen - vt(v).minGap;
            if (gap < 0) {
                return (insertionChecksOf(v) & IC_COLLISION) != 0 ? INVALID_SPEED : 0;
            }
            nspeed = std::min(nspeed, cfInsertionFollowSpeed(v, speed, gap, kin[leader].speed, vt(leader).decel, leader));
        }
    }
    return nspeed;
}

// MSLane::getBruttoOccupancy
static double bruttoOccupancy(const Sim& s, int lane) {
    // MSLane::getFractionalVehicleLength(brutto = true)
    double fractions = 0;
    const int bidi = s.net.lanes[lane].bidi;
    for (int u : s.lanes[lane].partials) {
        if (s.sublane && s.kin[u].shadowLane == lane) {
            continue;
        }
        if (bidi >= 0 && s.kin[u].lane == bidi) {
            fractions += s.vt(u).length + s.vt(u).minGap;
        } else {
            fractions += s.net.lanes[lane].length - s.backPositionOnLane(u, lane);
        }
    }
    const LaneRT& l = s.lanes[lane];
    if (!l.vehs.empty()) {
        const int last = l.vehs.front();
        if (s.kin[last].pos < s.vt(last).length) {
            fractions -= s.vt(last).length - s.kin[last].pos;
        }
    }
    return std::min(1., (l.brutto + fractions) / s.net.lanes[lane].length);
}

// MSEdge::getDepartPosBound
double Sim::departPosBound(int v, int edge, bool upper) const {
    const Veh& x = veh[v];
    const double length = net.edges[edge].length;
    double pos = length;
    switch (x.departPosMode) {
        case DP_GIVEN:
            pos = x.departPos;
            if (pos < 0.) {
                pos += length;
            }
            break;
        case DP_RANDOM:
        case DP_RANDOM_FREE:
        case DP_FREE:
            break;
        case DP_LAST:
            if (upper) {
                for (int l : net.edges[edge].lanes) {
                    if (!lanes[l].vehs.empty()) {
                        pos = std::min(pos, kin[lanes[l].vehs.front()].pos);
                    }
                }
            } else {
                pos = 0;
            }
            break;
        case DP_BASE:
            if (!upper) {
                pos = 0;
            }
            break;
        default:
            pos = std::min(pos, vt(v).length);
            break;
    }
    return pos;
}

// MSEdge::insertVehicle(checkOnly = true): whether the departure edge has room at all
bool Sim::insertCheckOnly(int v) const {
    const Veh& x = veh[v];
    const int e = routes[x.route][x.routeIdx];
    const Edge& edge = net.edges[e];
    if (!net.edgeAllows(e, x.vclass, x.ignoreTransientPermissions())) {
        return true;  // an invalid route start: tried (and removed) by the insertion
    }
    auto hasRoom = [&](int lane) {
        const double occupancy = bruttoOccupancy(*this, lane);
        return occupancy == 0 || occupancy * edge.length + vt(v).length + vt(v).minGap <= edge.length || x.departSplit;
    };
    if (x.departLane >= 0 || x.departLane == DL_FIRST) {
        // GIVEN, DEFAULT, FIRST_ALLOWED: the lane getDepartLane chooses
        int lane = -1;
        if (x.departLane >= 0) {
            if (x.departLane < (int)edge.lanes.size() && net.lanes[edge.lanes[x.departLane]].allows(x.vclass)) {
                lane = edge.lanes[x.departLane];
            }
        } else {
            for (int l : edge.lanes) {
                if (net.lanes[l].allows(x.vclass)) {
                    lane = l;
                    break;
                }
            }
        }
        return lane >= 0 && hasRoom(lane);
    }
    for (int l : edge.lanes) {
        if (hasRoom(l)) {
            return true;
        }
    }
    return false;
}

// MSEdge::getFreeLane
int Sim::freeLane(int edge, const std::vector<int>* allowed, SVCPermissions vclass, double departPos) const {
    std::vector<int> all;
    if (allowed == nullptr) {
        all = net.allowedLanesOf(edge, vclass);
        allowed = &all;
    }
    int res = -1;
    double largestGap = 0;
    int resByGap = -1;
    double leastOccupancy = DMAX;
    for (int lane : *allowed) {
        const double occupancy = bruttoOccupancy(*this, lane);
        if (occupancy < leastOccupancy) {
            res = lane;
            leastOccupancy = occupancy;
        }
        const int last = lanes[lane].vehs.empty() ? -1 : lanes[lane].vehs.front();
        const double lastGap = (last >= 0 ? kin[last].pos : net.edges[edge].length) - departPos;
        if (lastGap > largestGap) {
            largestGap = lastGap;
            resByGap = lane;
        }
    }
    if (resByGap >= 0) {
        res = resByGap;
    }
    return res;
}

// MSEdge::getProbableLane
int Sim::probableLane(int edge, const std::vector<int>* allowed, SVCPermissions vclass, double departPos, double maxSpeed) const {
    std::vector<int> all;
    if (allowed == nullptr) {
        all = net.allowedLanesOf(edge, vclass);
        allowed = &all;
    }
    int res = -1;
    double largestGap = 0;
    double largestSpeed = 0;
    int resByGap = -1;
    double leastOccupancy = DMAX;
    for (int lane : *allowed) {
        const double occupancy = bruttoOccupancy(*this, lane);
        if (occupancy < leastOccupancy) {
            res = lane;
            leastOccupancy = occupancy;
        }
        const int last = lanes[lane].vehs.empty() ? -1 : lanes[lane].vehs.front();
        const double lastGap = (last >= 0 ? kin[last].pos : net.edges[edge].length) - departPos;
        if (lastGap > largestGap && maxSpeed >= largestSpeed) {
            largestGap = lastGap;
            resByGap = lane;
        }
        if (last >= 0) {
            // MSEdge::getVehicleMaxSpeed(last): first lane of the edge
            largestSpeed = std::max(largestSpeed, laneVehMaxSpeed(net.edges[edge].lanes[0], last));
        }
    }
    if (resByGap >= 0) {
        res = resByGap;
    }
    return res;
}

// MSLane::getMeanSpeed
double Sim::laneMeanSpeed(int lane) const {
    const LaneRT& l = lanes[lane];
    if (l.vehs.empty()) {
        return net.lanes[lane].speed;
    }
    double v = 0;
    int n = 0;
    const bool lc = net.edges[net.lanes[lane].edge].hasLaneChanger;
    for (int u : l.vehs) {
        if (!isStopped(u) || !lc) {
            v += kin[u].speed;
            n++;
        }
    }
    return n == 0 ? net.lanes[lane].speed : v / n;
}

void Sim::incorporate(int v, int lane, double pos, double speed) {
    Veh& x = veh[v];
    Kin& k = kin[v];
    const VType& t = vt(v);
    std::vector<int>& vs = lanes[lane].vehs;
    if (vs.empty() && trackActiveList) {
        changedState.push_back(lane);  // MSEdgeControl::gotActive
    }
    auto it = std::find_if(vs.begin(), vs.end(), [&](int u) { return kin[u].pos >= pos; });
    vs.insert(it, v);
    lanes[lane].brutto += t.lengthWithGap();
    lanes[lane].netto += t.length;
    // enterLaneAtInsertion
    k.lane = lane;
    k.pos = pos;
    k.speed = speed;
    if (x.status == VehStatus::PENDING) {
        k.accel = 0;  // (vehicles ending a teleport or parking keep their acceleration)
    }
    x.lastActionTime = now + DELTA_T;  // the first action in the next step
    k.posLat = insertionPosLat;  // (departPosLat; a vehicle leaving a parking area keeps its lateral position)
    if (sublane) {
        k.shadowLane = -1;
        k.nShadowFurther = 0;
    }
    if (x.status == VehStatus::PENDING) {
        vehicleDeparted(v);
        x.prevSpeed = speed;
        x.departTime = STEPS2TIME(now);
        x.departSpeedActual = speed;
        x.departPosActual = pos;
        x.departLaneActual = net.lanes[lane].index;
        x.routeLength = -pos;
        x.lc = LCState();
    } else if (x.status == VehStatus::TELEPORTING) {
        // the end of a teleport (NOTIFICATION_TELEPORT)
        x.lastBestLanesEdge = -1;
        x.lastBestLanesInternalLane = -1;
        // MSLCM_LC2013::resetState
        LCState& lc = x.lc;
        lc.ownState = 0;
        lc.speedGainProbLeft = 0;
        lc.speedGainProbRight = 0;
        lc.keepRightProb = 0;
        lc.leadingBlockerLength = 0;
        lc.leftSpace = 0;
        lc.lookAheadSpeed = 0;  // LOOK_AHEAD_MIN_SPEED
        lc.advices.clear();
        lc.dontBrake = false;
        // stops already passed on this edge are skipped
        while (!x.stops.empty() && x.stops.front().routeIdx == x.routeIdx && net.lanes[x.stops.front().lane].edge == net.lanes[lane].edge
               && x.stops.front().endPos < pos) {
            x.stops.erase(x.stops.begin());
        }
        x.stopSpeed = DMAX;
    }
    const bool notifyReminders = x.status != VehStatus::TELEPORTING;
    const bool departed = x.status == VehStatus::PENDING;
    x.status = VehStatus::RUNNING;
    markEdgeDelayed(lane);
    if (departed && x.routingDevice) {
        // the vehicle's devices come first among its reminders (MSDevice_Routing::notifyEnter may reroute)
        static RouteWS ws;  // insertion is serial
        routingDeparted(v, ws);
        if (railActive && !railNewRoutes.empty() && std::find(railNewRoutes.begin(), railNewRoutes.end(), v) != railNewRoutes.end()) {
            // MSRailSignalControl::vehicleStateChanged (NEWROUTE) right away
            railNewRoutes.erase(std::remove(railNewRoutes.begin(), railNewRoutes.end(), v), railNewRoutes.end());
            if (!railSignals.empty() && isRailVeh(v) && isRailwayOrShared(rtPermOf(curEdge(v)))) {
                initDriveWays(v, true);
            }
            railRerouted.push_back(v);
        }
    }
    if (railActive && notifyReminders) {
        // enterLaneAtInsertion activates the lane reminders (departed, parking; teleports did at the start)
        trackerEnter(v, lane);
        dwLaneEntered(v, lane, x.routeIdx, departed ? DWR_DEPARTED : DWR_PARKING);  // MSDriveWay::notifyEnter
    }
    if (notifyReminders) {
        // NOTIFICATION_DEPARTED / NOTIFICATION_PARKING (teleports activated them at enterLaneAtMove)
        loopNotifyEnterNow(v, lane);
    }
    if (isInternal(lane) && x.junctionEntryTime == TIME_MAX) {
        x.junctionEntryTime = now;
        x.junctionEntryTimeNeverYield = now;
        if (net.links[net.lanes[lane].incoming.front().link].isConflictEntryLink()) {
            x.junctionConflictEntryTime = now;
        }
    }
    // computeFurtherLanes
    double leftLength = t.length - pos;
    int clane = lane;
    int routeIndex = x.routeIdx;
    int n = 0;
    while (leftLength > 0) {
        if (routeIndex > 0 && !isInternal(clane)) {
            routeIndex--;
            const int fromRouteEdge = routes[x.route][routeIndex];
            const int target = clane;
            clane = -1;
            for (const IncomingLane& ili : net.lanes[target].incoming) {
                // getNormalBefore of the incoming lane's edge
                const int normalBefore = net.lanes[net.normalPredecessorLane(ili.lane)].edge;
                if (normalBefore == fromRouteEdge) {
                    clane = ili.lane;
                    break;
                }
            }
        } else {
            clane = net.lanes[clane].logicalPred;
        }
        if (clane < 0 || clane == lane
                || (isInternal(clane) && net.links[net.lanes[clane].links[0]].isTurnaround())) {
            break;
        }
        if (n < MAX_FURTHER) {
            k.further[n++] = clane;
        } else {
            nFurtherOverflow++;
        }
        leftLength -= net.lanes[clane].length;
    }
    k.nFurther = n;
    setNoReminderFurther(v);
    k.backPos = -leftLength;
    if (sublane) {
        std::fill(lat[v].furtherPosLat, lat[v].furtherPosLat + n, k.posLat);
        updateShadowLane(v);
    }
    if (hasRerouters()) {
        if (VehExtra* e = x.exq()) {
            e->rrActive.clear();
        }
        rerouterEnter(v, lane, false);
    }
}

// MSLane::checkForPedestrians: pedestrians ahead on the lane, pedestrians on (or approaching) crossings the
// back of the vehicle would occupy
bool Sim::checkForPedestrians(int v, int lane, double& speed, double& dist, double pos, bool patchSpeed) {
    const VType& t = vt(v);
    if (hasPedestrians(lane)) {
        const std::pair<int, double> pl = nextBlocking(lane, pos - t.length, rightSideOnLane(v, lane), rightSideOnLane(v, lane) + t.width,
                                                       std::ceil(speed / t.decel));
        if (pl.first >= 0) {
            const double gap = pl.second - t.lengthWithGap();
            const double stopSpeed = cfStopSpeed(v, speed, gap, -1, cf::CalcReason::FUTURE);
            if ((gap < 0 && (insertionChecksOf(v) & (IC_COLLISION | IC_PEDESTRIAN)) != 0)
                    || checkFailure(v, speed, dist, stopSpeed, patchSpeed, IC_PEDESTRIAN)) {
                return false;
            }
        }
    }
    double backLength = t.length - pos;
    if (backLength > 0 && !persons.empty()) {
        // look upstream for pedestrian crossings
        int prev = net.lanes[lane].logicalPred;
        int cur = lane;
        while (backLength > 0 && prev >= 0) {
            const Link* k = net.linkTo(prev, cur);
            if (k->havePedestrianCrossingFoe) {
                for (int foe : k->foeLanes) {
                    if (net.lanes[foe].crossing
                            && (hasPedestrians(foe) || !approachingPersons[net.lanes[foe].incoming[0].link].empty())) {
                        return false;
                    }
                }
            }
            backLength -= net.lanes[prev].length;
            cur = prev;
            prev = net.lanes[prev].logicalPred;
        }
    }
    return true;
}

bool Sim::isInsertionSuccess(int v, int lane, double speed, double pos, bool patchSpeed, bool resuming) {
    Veh& x = veh[v];
    Kin& k = kin[v];
    const VType& t = vt(v);
    const double laneLength = net.lanes[lane].length;
    if (pos < 0 || pos > laneLength) {
        pos = laneLength;
    }
    // setTentativeLaneAndPosition + updateBestLanes
    const int oldLane = k.lane;
    const double oldPos = k.pos;
    k.lane = lane;
    k.pos = pos;
    k.posLat = insertionPosLat;
    if (!resuming && veh[v].status == VehStatus::PENDING) {
        k.speed = 0;  // (a teleporting vehicle keeps its speed: followers' gaps in freeInsertion)
    }
    k.nFurther = 0;
    if (sublane) {
        k.shadowLane = -1;
        k.nShadowFurther = 0;
    }
    // (the occupancies of the continuations are not needed here: lane changing recomputes them
    // before reading them, entering an internal lane too)
    updateBestLanes(v, false, lane, false);
    const std::vector<int>& conts = bestContinuationOf(v, lane);
    double seen = laneLength - pos;
    double dist = cfBrakeGap(v, speed) + t.minGap;
    const bool patchSpeedSpecial = patchSpeed || x.departSpeedMode != 0;
    auto fail = [&](int line = __builtin_LINE()) {
        if (v == traceVeh) {
            std::fprintf(stderr, "%.2f insertion of %s failed on %s at pos=%.2f speed=%.2f (line %d)\n", STEPS2TIME(now), veh[v].id.c_str(), net.lanes[lane].id.c_str(), pos, speed, line);
        }
        k.lane = oldLane;
        if (!resuming) {
            k.pos = oldPos;
        }  // (a parking vehicle keeps the tentative position, like SUMO)
        return false;
    };
    // trains: the departure driveway, oncoming trains on the bidirectional tracks
    const bool isRail = railActive && isRailVeh(v);
    if (isRail && insertionChecksOf(v) != IC_NONE && !veh[v].departSplit && railDepartBlocked(v, lane)) {
        return fail();
    }
    if (isRail && (insertionChecksOf(v) & IC_BIDI) != 0 && railBidiBlocked(v, lane, pos)) {
        return fail();
    }
    int firstRailSignal = -1;
    double firstRailSignalDist = -1;
    // stop on the insertion lane
    if (!x.stops.empty() && x.stops.front().lane == lane && !x.stops.front().reached) {
        const Stop& nextStop = x.stops.front();
        const bool waypoint = nextStop.speed > 0;
        const double distToStop = waypoint ? std::max(0.0, nextStop.startPos - pos) : nextStop.endPos - pos;
        const double safeSpeed = waypoint ? cfFreeSpeed(v, speed, distToStop, nextStop.speed, true)
                                          : cfStopSpeed(v, speed, distToStop, -1, cf::CalcReason::FUTURE);
        if (checkFailure(v, speed, dist, std::max(0.0, safeSpeed), patchSpeedSpecial, IC_STOP, true)) {
            return fail();
        }
    }
    // leader on the insertion lane
    if (latRes > 0) {
        const double nspeed = safeInsertionSpeedSL(v, lane, -pos, lastVehicleInformation(lane, v, 0, pos), speed);
        if (nspeed == INVALID_SPEED || checkFailure(v, speed, dist, nspeed, patchSpeed, IC_LEADER_GAP)) {
            return fail();
        }
    } else {
        const int leader = lastVehicleFrom(lane, pos, v);
        const double nspeed = safeInsertionSpeed(v, lane, -pos, leader, speed);
        if (nspeed == INVALID_SPEED || checkFailure(v, speed, dist, nspeed, patchSpeed, IC_LEADER_GAP)) {
            return fail();
        }
    }
    int nRouteSuccs = 1;
    int currentLane = lane;
    size_t ri = 0;
    Time arrivalTime = now + TIME2STEPS(seen / std::max(speed, HALTING_SPEED));
    while ((seen < dist || (isRail && firstRailSignal < 0)) && ri < conts.size()) {
        const int link = succLinkSec(v, nRouteSuccs, currentLane, conts);
        if (link < 0) {
            if (net.lanes[currentLane].edge != routes[x.route].back()) {
                if (checkFailure(v, speed, dist, cfInsertionStopSpeed(v, speed, seen), patchSpeedSpecial, IC_JUNCTION, true)) {
                    return fail();
                }
            }
            break;
        }
        const Link& L = net.links[link];
        const int nextLane = L.viaOrLane();
        if (isRail && firstRailSignal < 0 && hasInsertionConstraint(link, v)) {
            return fail();
        }
        // (may also be a regular traffic light instead of a rail signal)
        if (firstRailSignal < 0 && L.tls >= 0) {
            firstRailSignal = link;
            firstRailSignalDist = seen;
        }
        if (!opened(link, arrivalTime, speed, speed, t.length, impatience(v), t.decel, 0, v) || railSignalWasPassed(link)
                || !L.havePriority() || L.state == LS_ZIPPER) {
            // no sense in trying later (an error deschedules the vehicle): an unprioritized junction or a traffic light
            // that never switches to 'G' (MSTrafficLightLogic::getsMajorGreen; always for a rail signal)
            bool neverGreen = false;
            if (L.tls >= 0 && net.tls[L.tls].type != "rail_signal") {
                neverGreen = true;
                for (const Phase& ph : net.tls[L.tls].phases) {
                    if (L.tlIndex >= 0 && L.tlIndex < (int)ph.state.size() && ph.state[L.tlIndex] == 'G') {
                        neverGreen = false;
                        break;
                    }
                }
            }
            const bool error = L.state == LS_MINOR || L.state == LS_EQUAL || L.state == LS_STOP || L.state == LS_ALLWAY_STOP
                               || neverGreen;
            const double laneStopOffset = std::max(0., t.jmStoplineCrossingGap - L.distToFoePedCrossing);
            const double remaining = seen - laneStopOffset;
            if (checkFailure(v, speed, dist, cfInsertionStopSpeed(v, speed, remaining), patchSpeedSpecial, IC_JUNCTION, error)) {
                return fail();
            }
            if (seen >= t.minGap) {
                break;
            }
        } else if (isInternal(nextLane)) {
            double tmp = 0;
            bool dummyReq = true;
            double nSpeed = speed;
            checkLinkLeader(v, net.lanes[nextLane].links[0], nextLane, seen + net.lanes[nextLane].length, nullptr, nSpeed, tmp, tmp, dummyReq);
            if (checkFailure(v, speed, dist, nSpeed, patchSpeed, IC_LEADER_GAP)) {
                return fail();
            }
        }
        // no insertion when the bidirectional track is occupied before a rail signal
        if (firstRailSignal < 0 && net.lanes[nextLane].bidi >= 0 && laneVehicleNumberWithPartials(net.lanes[nextLane].bidi) > 0
                && (insertionChecksOf(v) & IC_ONCOMING_TRAIN) != 0) {
            return fail();
        }
        // stop on the next lane
        if (!x.stops.empty() && x.stops.front().lane == nextLane) {
            const Stop& nextStop = x.stops.front();
            const bool waypoint = nextStop.speed > 0;
            const double distToStop = seen + (waypoint ? nextStop.startPos : nextStop.endPos);
            const double safeSpeed = waypoint ? cfFreeSpeed(v, speed, distToStop, nextStop.speed, true)
                                              : cfInsertionStopSpeed(v, speed, distToStop);
            if (checkFailure(v, speed, dist, safeSpeed, patchSpeedSpecial, IC_STOP, true)) {
                return fail();
            }
        }
        // leader on the next lane
        if (latRes > 0) {
            const LeaderInfo nextLeaders = lastVehicleInformation(nextLane, v, 0, 0);
            if (nextLeaders.hasVehicles()) {
                const double nextLaneSpeed = safeInsertionSpeedSL(v, nextLane, seen, nextLeaders, speed);
                if (nextLaneSpeed == INVALID_SPEED || checkFailure(v, speed, dist, nextLaneSpeed, patchSpeed, IC_LEADER_GAP)) {
                    return fail();
                }
            }
        } else {
            const int nl = lastVehicleFrom(nextLane, 0, v);
            if (nl >= 0) {
                const double nextLaneSpeed = safeInsertionSpeed(v, nextLane, seen, nl, speed);
                if (nextLaneSpeed == INVALID_SPEED || checkFailure(v, speed, dist, nextLaneSpeed, patchSpeed, IC_LEADER_GAP)) {
                    return fail();
                }
            }
        }
        if (!checkForPedestrians(v, nextLane, speed, dist, -seen, patchSpeed)) {
            return fail();
        }
        const double freeSpeed = cfFreeSpeed(v, speed, seen, laneVehMaxSpeed(nextLane, v), true);
        if (freeSpeed < speed && patchSpeedSpecial) {
            speed = freeSpeed;
            dist = cfBrakeGap(v, speed) + t.minGap;
        }
        const Time lt = leaveTime(link, arrivalTime, speed, speed, t.length);
        if (hasApproachingFoe(link, arrivalTime, lt, speed, t.decel)) {
            if (checkFailure(v, speed, dist, cfInsertionStopSpeed(v, speed, seen), patchSpeed, IC_JUNCTION)) {
                return fail();
            }
        }
        arrivalTime += TIME2STEPS(net.lanes[nextLane].length / std::max(speed, NUMERICAL_EPS));
        seen += net.lanes[nextLane].length;
        currentLane = nextLane;
        if (L.via < 0) {
            nRouteSuccs++;
            ++ri;
        }
    }
    if (!checkForPedestrians(v, lane, speed, dist, pos, patchSpeed)) {
        return fail();
    }
    // followers
    const double backOffset = pos - t.length;
    if (latRes > 0) {
        // (the vehicle's current speed selects the followers: 0 before the departure)
        const double keepSpeed = k.speed;
        if (x.status == VehStatus::PENDING) {
            k.speed = 0;
        }
        auto anyVeh = [this](int l, auto f) { anyVehicles(l, f); };
        const CriticalFollowerDistanceInfo followers = followersOnConsecutiveT(lane, v, backOffset, false, false, anyVeh);
        bool blocked = false;
        auto tooClose = [&](const LeaderDistanceInfo& fl) {
            for (int i = 0; i < fl.numSublanes() && !blocked; ++i) {
                const std::pair<int, double> f = fl.at(i);
                if (f.first >= 0) {
                    const double backGapNeeded = cfSecureGap(f.first, v, kin[f.first].speed, speed, t.decel);
                    blocked = f.second < backGapNeeded
                              && ((insertionChecksOf(v) & IC_FOLLOWER_GAP) != 0 || (f.second < 0 && (insertionChecksOf(v) & IC_COLLISION) != 0));
                }
            }
        };
        tooClose(followers);
        // the shadow lane (MSAbstractLaneChangeModel::getShadowLane(lane))
        const int shadowLane = blocked ? -1 : shadowLaneFor(v, lane, k.posLat);
        if (shadowLane >= 0) {
            tooClose(followersOnConsecutiveT(shadowLane, v, backOffset, false, false, anyVeh));
            if (!blocked) {
                const LeaderInfo ahead = lastVehicleInformation(shadowLane, -1, 0, pos);
                for (int i = 0; i < ahead.numSublanes() && !blocked; ++i) {
                    const int u = ahead[i];
                    if (u >= 0) {
                        const double gap = backPositionOnLane(u, shadowLane) - pos - t.minGap;
                        const double gapNeeded = cfSecureGap(v, u, speed, kin[u].speed, vt(u).decel);
                        blocked = gap < gapNeeded
                                  && ((insertionChecksOf(v) & IC_LEADER_GAP) != 0 || (gap < 0 && (insertionChecksOf(v) & IC_COLLISION) != 0));
                    }
                }
            }
        }
        k.speed = keepSpeed;
        if (blocked) {
            return fail();
        }
    } else {
        // (MSCriticalFollowerDistanceInfo::addFollower selects with the vehicle's current speed: 0 before the
        // departure, a teleporting vehicle keeps its speed)
        const CritFollower f = followersOnConsecutive(lane, v, backOffset, x.status == VehStatus::PENDING ? 0.0 : k.speed, t.decel);
        if (f.veh >= 0) {
            const double backGapNeeded = cfSecureGap(f.veh, v, kin[f.veh].speed, speed, t.decel);
            if (f.dist < backGapNeeded
                    && ((insertionChecksOf(v) & IC_FOLLOWER_GAP) != 0 || (f.dist < 0 && (insertionChecksOf(v) & IC_COLLISION) != 0))) {
                return fail();
            }
        }
    }
    // (MSLane::getMissingRearGap repeats the same follower query and is 0 when no follower was found)
    if (insertionChecksOf(v) == IC_NONE) {
        speed = std::max(0.0, speed);
    }
    if (speed < 0) {
        return fail();
    }
    // LC2013::getExtraReservation
    const int blo = bestLaneOffset(v);
    double extraReservation = 0;
    if (blo < -1) {
        extraReservation = 20;
    } else if (blo > 1) {
        extraReservation = 40;
    }
    if (extraReservation > 0) {
        double distToStop = bestLaneDist(v) - pos - extraReservation - speed * actionStepLengthSecs(v);
        if (distToStop >= 0) {
            const double stopSpeed = cfStopSpeed(v, speed, distToStop, -1, cf::CalcReason::FUTURE);
            if (checkFailure(v, speed, distToStop, std::max(0.0, stopSpeed), patchSpeedSpecial, IC_LANECHANGE, true)) {
                return fail();
            }
        }
    }
    incorporate(v, lane, pos, speed);
    if (isRail && firstRailSignal >= 0 && net.links[firstRailSignal].junction >= 0
            && net.junctions[net.links[firstRailSignal].junction].type == "rail_signal") {
        // the rail signal needs the approach for switching at the start of the next step
        registerInsertionApproach(v, firstRailSignal, firstRailSignalDist);
    }
    return true;
}

// MSLane::getDepartPosLat
double Sim::departPosLatOf(int v, int lane, int attempt) const {
    const VehExtra* xe = veh[v].extra.p.get();
    if (xe == nullptr) {
        return 0;
    }
    const double w = net.lanes[lane].width;
    const double vw = vt(v).width;
    switch (xe->departPosLatProc) {
        case DPL_GIVEN:
            return xe->departPosLat;
        case DPL_RIGHT:
            return -w * 0.5 + vw * 0.5;
        case DPL_LEFT:
            return w * 0.5 - vw * 0.5;
        case DPL_RANDOM: {
            const double raw = rand01(opt.seed, (uint64_t)v, (uint64_t)step * 16 + attempt, RP_DEPART_POSLAT) * (w - vw) - w * 0.5 + vw * 0.5;
            return std::round(raw * 100.) / 100.;
        }
        default:
            return 0;
    }
}

bool Sim::tryInsert(int v) {
    Veh& x = veh[v];
    const int e = routes[x.route][x.routeIdx];  // (routeIdx: the departEdge)
    const Edge& edge = net.edges[e];
    if (insertFailedOnEdge[e] == (int)edge.lanes.size() && !opt.eagerInsert && !x.departSplit) {
        return false;  // whichever lane is chosen, it already refused a vehicle in this step
    }
    if (net.edgeVersionPermissions && !net.edgeAllows(e, x.vclass, x.ignoreTransientPermissions())) {
        // MSBaseVehicle::hasValidRouteStart: not allowed to depart on any lane of the edge (closed meanwhile)
        if (!opt.ignoreRouteErrors) {
            throw std::runtime_error("Vehicle '" + x.id + "' is not allowed to depart on any lane of edge '" + edge.id + "'.");
        }
        x.status = VehStatus::REMOVED;
        releaseVehicle(v);
        return false;
    }
    // MSEdge::getDepartLane
    int lane = -1;
    if (x.departLane >= 0) {
        if (x.departLane < (int)edge.lanes.size() && net.lanes[edge.lanes[x.departLane]].allows(x.vclass)) {
            lane = edge.lanes[x.departLane];
        }
    } else if (x.departLane == DL_RANDOM) {
        // RandHelper::getRandomFrom(allowedLanes(vclass)), drawn per insertion attempt
        const std::vector<int> allowed = net.allowedLanesOf(e, x.vclass);
        if (!allowed.empty()) {
            lane = allowed[(size_t)(rand01(opt.seed, (uint64_t)v, (uint64_t)step, RP_DEPARTLANE) * allowed.size())];
        }
    } else if (x.departLane == DL_ALLOWED) {
        // ALLOWED_FREE: the free lane among those that lead to the next route edge
        if (routes[x.route].size() == 1) {
            lane = freeLane(e, nullptr, x.vclass, departPosBound(v, e, false));
        } else {
            lane = freeLane(e, net.allowedLanes(e, routes[x.route][1], x.vclass), x.vclass, departPosBound(v, e, false));
        }
    } else if (x.departLane == DL_BEST || x.departLane == DL_BEST_PROB) {
        // BEST_FREE / BEST_PROB
        kin[v].lane = edge.lanes[0];
        kin[v].pos = 0;
        updateBestLanes(v, false, edge.lanes[0], false);
        const std::vector<LaneQ>& bl = bestLanesOf(v);
        double bestLength = -1;
        for (const LaneQ& q : bl) {
            bestLength = std::max(bestLength, q.length);
        }
        double departPos = 0;
        if (bestLength > 3000.0) {
            departPos = departPosBound(v, e, true);
            bestLength = std::min(bestLength - departPos, 3000.0);
        }
        std::vector<int> best;
        for (const LaneQ& q : bl) {
            if (q.length - departPos >= bestLength && net.lanes[q.lane].allows(x.vclass)) {
                best.push_back(q.lane);
            }
        }
        if (v == traceVeh) {
            for (const LaneQ& q : bl) {
                std::fprintf(stderr, "  best %s len=%.2f\n", net.lanes[q.lane].id.c_str(), q.length);
            }
        }
        if (x.departLane == DL_BEST) {
            lane = freeLane(e, &best, x.vclass, departPosBound(v, e, false));
        } else {
            lane = probableLane(e, &best, x.vclass, departPosBound(v, e, false), laneVehMaxSpeed(edge.lanes[0], v));
        }
    } else if (x.departLane == DL_FREE) {
        lane = freeLane(e, nullptr, x.vclass, departPosBound(v, e, false));
    } else {
        for (int l : edge.lanes) {
            if (net.lanes[l].allows(x.vclass)) {
                lane = l;
                break;
            }
        }
    }
    if (lane < 0) {
        return false;
    }
    if (insertFailed[lane] && !opt.eagerInsert && !x.departSplit) {
        return false;  // (--eager-insert: MSEdge::insertVehicle with forceCheck)
    }
    const VType& t = vt(v);
    double speed = 0;
    bool patchSpeed = true;
    const double vMax = laneVehMaxSpeed(lane, v);
    // MSLane::getDepartSpeed
    switch (x.departSpeedMode) {
        case DS_GIVEN:
            speed = x.departSpeed;
            patchSpeed = false;
            break;
        case DS_RANDOM:
            speed = std::round(rand01(opt.seed, (uint64_t)v, (uint64_t)step, RP_DEPARTSPEED) * vMax * 100.) / 100.;
            patchSpeed = true;
            break;
        case DS_MAX:
            speed = vMax;
            patchSpeed = true;
            break;
        case DS_DESIRED:
            speed = vMax;
            patchSpeed = false;
            break;
        case DS_LIMIT:
            speed = vMax / x.speedFactor;
            patchSpeed = false;
            break;
        case DS_LAST: {
            speed = vMax;
            const int last = lastAnyVehicle(lane);
            if (last >= 0) {
                speed = std::min(speed, kin[last].speed);
                patchSpeed = false;
            }
            break;
        }
        default:
            // AVG
            speed = std::min(vMax, laneMeanSpeed(lane));
            patchSpeed = lastAnyVehicle(lane) < 0;
            break;
    }
    // MSLane::insertVehicle: the departure position
    insertionPosLat = departPosLatOf(v, lane, 0);
    const double laneLength = net.lanes[lane].length;
    double pos;
    bool ok = false;
    bool decided = false;
    switch (x.departPosMode) {
        case DP_GIVEN:
            pos = x.departPos < 0 ? x.departPos + laneLength : x.departPos;
            break;
        case DP_RANDOM:
            pos = std::round(rand01(opt.seed, (uint64_t)v, (uint64_t)step * 16, RP_DEPARTPOS) * laneLength * 100.) / 100.;
            break;
        case DP_RANDOM_FREE:
            // a few random positions, then the free position
            for (int i = 0; i < 10 && !ok; ++i) {
                const double rpos = rand01(opt.seed, (uint64_t)v, (uint64_t)step * 16 + i, RP_DEPARTPOS) * laneLength;
                insertionPosLat = departPosLatOf(v, lane, i + 1);  // (could be random as well)
                ok = isInsertionSuccess(v, lane, speed, rpos, patchSpeed, false);
            }
            if (!ok) {
                ok = freeInsertion(v, lane, speed, true);
            }
            decided = true;
            break;
        case DP_FREE:
            ok = freeInsertion(v, lane, speed, true);
            decided = true;
            break;
        case DP_LAST:
            ok = lastInsertion(v, lane, speed, patchSpeed);
            decided = true;
            break;
        case DP_STOP:
            if (!x.stops.empty() && x.stops.front().routeIdx == x.routeIdx) {
                pos = std::max(0.0, stopEndPos(v, x.stops.front()));
                break;
            }
            [[fallthrough]];
        default:
            if (x.departSplit) {
                // behind (or in front of) the train it splits from, which stops on this lane
                pos = laneLength;
                for (int u : lanes[lane].vehs) {
                    if (isStopped(u) && veh[u].stops.front().splitVeh == v) {
                        pos = x.departPosMode == DP_SPLIT_FRONT ? kin[u].pos + vt(u).minGap + t.length
                                                                : backPositionOnLane(u, lane) - t.minGap;
                        break;
                    }
                }
                break;
            }
            // MSBaseVehicle::basePos
            pos = std::min(t.length + POSITION_EPS, edge.length);
            if (!x.stops.empty() && x.stops.front().routeIdx == x.routeIdx) {
                pos = std::min(pos, std::max(0.0, stopEndPos(v, x.stops.front())));
            }
            break;
    }
    if (!decided) {
        const VehExtra* xe = x.extra.p.get();
        const int dplProc = xe != nullptr ? xe->departPosLatProc : DPL_DEFAULT;
        if (latRes > 0 && (dplProc == DPL_FREE || dplProc == DPL_RANDOM_FREE)) {
            const double w = net.lanes[lane].width;
            if (dplProc == DPL_RANDOM_FREE) {
                // a few random lateral positions
                for (int i = 0; i < 10 && !ok; ++i) {
                    insertionPosLat = rand01(opt.seed, (uint64_t)v, (uint64_t)step * 16 + i, RP_DEPART_POSLAT + 8) * w - w * 0.5;
                    ok = isInsertionSuccess(v, lane, speed, pos, patchSpeed, false);
                }
            }
            // all lateral positions until a free one is found
            const double posLatMin = -w * 0.5 + t.width * 0.5;
            const double posLatMax = w * 0.5 - t.width * 0.5;
            for (double pl = posLatMin; pl < posLatMax && !ok; pl += latRes) {
                insertionPosLat = pl;
                ok = isInsertionSuccess(v, lane, speed, pos, patchSpeed, false);
            }
            decided = true;
        } else {
            ok = isInsertionSuccess(v, lane, speed, pos, patchSpeed, false);
        }
    }
    insertionPosLat = 0;
    const Time delay = now - x.depart;
    if (ok && !decided && opt.extrapolateDepartPos && delay > 0) {  // (not for the free and last positions)
        // MSLane::insertVehicle: move forward by the part of the step the vehicle should already have driven
        const double spd = kin[v].speed;
        double dist = spd * STEPS2TIME(std::min(DELTA_T, delay));
        const std::vector<int>& vs = lanes[lane].vehs;
        auto it = std::find(vs.begin(), vs.end(), v);
        if (it != vs.end() && it + 1 != vs.end()) {
            const int leader = *(it + 1);
            const double gap = backPositionOnLane(leader, lane) - kin[v].pos - t.minGap;
            dist = std::min(dist, gap - cfSecureGap(v, leader, spd, kin[leader].speed, vt(leader).decel));
        }
        if (dist > 0) {
            kin[v].pos = std::min(kin[v].pos + dist, net.lanes[lane].length - POSITION_EPS);
        }
    }
    if (!ok) {
        // MSEdge::insertVehicle: constraints may enforce explicit re-ordering, so other vehicles are tried after
        // a failure due to an insertion order (the lane keeps the mark of the vehicle)
        if (!insertFailed[lane] && !x.insertionOrderMark) {
            insertFailed[lane] = 1;
            insertFailedLanes.push_back(lane);
            insertFailedOnEdge[e]++;
        }
        kin[v].lane = -1;
    }
    return ok;
}

// MSNet::loadRoutes (SUMORouteLoaderControl::loadNext, SUMORouteLoader::loadUntil): the route file is read up to
// --route-steps ahead (at the start only up to the begin), up to and including the first vehicle departing later;
// the next reading happens at that vehicle's departure (the vehicles of flows are built at their departure)
void Sim::loadRoutesStep() {
    if (buildCursor >= veh.size() || nextBuildTime > now) {
        return;
    }
    const Time loadMax = opt.routeSteps <= 0 ? TIME_MAX
                         : std::max(nextBuildTime == -TIME_MAX ? -TIME_MAX : nextBuildTime + TIME2STEPS(opt.routeSteps), now);
    while (buildCursor < veh.size() && routeLastDepart <= loadMax) {
        const int v = (int)buildCursor++;
        if (veh[v].fromFlow) {
            continue;
        }
        // SUMORouteHandler::registerLastDepart: public transport (a line) is read in advance, its departure does
        // not count
        if (!veh[v].departTriggered && !veh[v].departSplit && veh[v].exr().line.empty()) {
            routeLastDepart = veh[v].depart;
        }
        vehicleBuilt(v);
    }
    nextBuildTime = buildCursor >= veh.size() ? TIME_MAX : routeLastDepart;
}

// a vehicle is built (MSVehicleControl::buildVehicle): its devices, MSNet::VehicleState::BUILT
void Sim::vehicleBuilt(int v) {
    Veh& x = veh[v];
    static const bool traceBuild = getenv("PSIM_TRACE_BUILD") != nullptr;
    if (traceBuild) {
        std::fprintf(stderr, "%.1f built %s (depart %.1f)\n", STEPS2TIME(now), x.id.c_str(), STEPS2TIME(x.depart));
    }
    if (rtActive && x.routingDevice && (TIME2STEPS(opt.reroutePrePeriod) > 0 || x.forceReroute)) {
        // MSDevice_Routing constructor: at the departure time if the weights are updated, else at the next execution
        pushPreReroute(v, x.departTriggered || !rtEdgeUpdates ? -1 : x.depart);
    }
    if ((x.routingDevice || (x.routingMode > 0 && (x.routingMode & (1 | 4)) != 0)) && !opt.withTaz && opt.adaptInterval >= 0) {
        rtEngineFrom = std::min(rtEngineFrom, now);  // MSRoutingEngine::initWeightUpdate
    }
    if (railActive) {
        railVehicleBuilt(v);
    }
}

void Sim::insertVehicles() {
    loadRoutesStep();
    // MSInsertionControl::checkCandidates: the vehicles waiting too long for their departure are skipped
    auto discard = [&](int v) {
        veh[v].status = VehStatus::REMOVED;
        releaseVehicle(v);
    };
    if (opt.maxDepartDelay >= 0 && !pending.empty()) {
        std::vector<int> keep;
        for (int v : pending) {
            if (veh[v].status == VehStatus::PENDING && now - veh[v].depart > opt.maxDepartDelay) {
                discard(v);
            } else {
                keep.push_back(v);
            }
        }
        pending.swap(keep);
    }
    while (nextToLoad < veh.size() - (size_t)nTriggeredDepart && veh[nextToLoad].depart <= now) {
        pending.push_back((int)nextToLoad);
        if (veh[nextToLoad].fromFlow) {
            vehicleBuilt((int)nextToLoad);  // flow vehicles are built when they depart
        }
        ++nextToLoad;
    }
    // vehicles whose triggered departure happened in this step (MSInsertionControl::add, behind the loaded ones)
    pending.insert(pending.end(), triggeredDue.begin(), triggeredDue.end());
    triggeredDue.clear();
    // MSInsertionControl::checkCandidates: with the routing engine enabled, only the vehicles whose departure
    // edge has room (MSEdge::insertVehicle(checkOnly)) are tried; the others skip their pre-insertion reroute
    const bool preCheck = now >= rtEngineFrom;
    std::vector<char> candidate;
    bool anyCandidate = false;
    if (preCheck) {
        candidate.assign(pending.size(), 0);
        for (size_t i = 0; i < pending.size(); ++i) {
            const int v = pending[i];
            if (veh[v].status != VehStatus::PENDING) {
                continue;
            }
            candidate[i] = insertCheckOnly(v) ? 1 : 0;
            anyCandidate |= candidate[i] != 0;
            if (!candidate[i] && veh[v].routingDevice) {
                veh[v].skipRoutingAt = now;
            }
        }
    }
    preInsertionReroutes();  // insertion events (MSDevice_Routing::preInsertionReroute)
    if (railActive) {
        processRailNewRoutes();
    }
    if (pending.empty() || (preCheck && !anyCandidate)) {
        return;
    }
    // reset the memory of the previous step (only the lanes that failed)
    if (insertFailed.size() != net.lanes.size() || insertFailedOnEdge.size() != net.edges.size()) {
        insertFailed.assign(net.lanes.size(), 0);
        insertFailedOnEdge.assign(net.edges.size(), 0);
    }
    for (int l : insertFailedLanes) {
        insertFailed[l] = 0;
        insertFailedOnEdge[net.lanes[l].edge] = 0;
    }
    insertFailedLanes.clear();
    std::vector<int> refused;
    // MSVehicleControl::getRunningVehicleNo (teleporting and parking vehicles included) for --max-num-vehicles
    int nRunning = (int)running.size() + (int)transfer.size();
    for (size_t i = 0; i < pending.size(); ++i) {
        const int v = pending[i];
        if (veh[v].status == VehStatus::REMOVED) {
            continue;  // (removed before its insertion)
        }
        if (preCheck && !candidate[i]) {
            refused.push_back(v);
            continue;
        }
        if ((opt.maxNumVehicles < 0 || nRunning < opt.maxNumVehicles) && tryInsert(v)) {
            running.push_back(v);
            nInserted++;
            nRunning++;
        } else if (!descheduled.empty() && std::find(descheduled.begin(), descheduled.end(), v) != descheduled.end()) {
            discard(v);
        } else if (veh[v].status == VehStatus::PENDING && opt.maxDepartDelay >= 0 && now - veh[v].depart > opt.maxDepartDelay) {
            discard(v);  // waited too long for its departure
        } else if (veh[v].status == VehStatus::PENDING) {
            refused.push_back(v);
        }
    }
    descheduled.clear();
    pending.swap(refused);
}

}  // namespace ps
