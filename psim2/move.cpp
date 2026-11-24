// Movement: planMove, junction leaders, checkRewindLinkLanes, link approach
// decisions and executeMove. Ported from SUMO 1.28 MSVehicle.cpp
// (planMoveInternal, adaptToLeader, adaptToJunctionLeader, checkLinkLeader,
// checkRewindLinkLanes, processLinkApproaches, processLaneAdvances,
// executeMove, updateFurtherLanes, updateDriveItems) without opposite
// driving, sublanes, rail, pedestrians, TraCI influence and waypoints.
//
// Every function here writes only the state of vehicle `v`; everything else
// is read from the state frozen at the start of the phase.
#include <cstdio>
#include <cstdlib>

#include "model.hpp"
#include "sublanequery.hpp"

namespace ps {

constexpr double DIST_TO_STOPLINE_EXPECT_PRIORITY = 1.0;
constexpr int CRLL_LOOK_AHEAD = 5;
constexpr double JUNCTION_BLOCKAGE_TIME = 5;  // s
constexpr double TLS_YELLOW_MIN_DECEL = 3.0;   // tls.yellow.min-decel

// MSAbstractLaneChangeModel::getCooperativeHelpSpeed
double Sim::cooperativeHelpSpeed(int v, int lane, double distToLaneEnd) const {
    const VType& t = vt(v);
    if (t.lcCooperativeHelpTime >= 0) {
        const Lane& ln = net.lanes[lane];
        const EdgeRT& e = edgesRT[ln.edge];
        const double backPos = e.lastBlockedBackPos[ln.index];
        const double waiting = e.lastBlockedWaiting[ln.index];
        if (waiting >= t.lcCooperativeHelpTime) {
            double gap = distToLaneEnd - ln.length + backPos - t.minGap - NUMERICAL_EPS;
            if (backPos < 0) {
                const int myLane = kin[v].lane;
                if (net.edges[net.lanes[myLane].edge].toJ == net.edges[ln.edge].fromJ) {
                    gap = isInternal(myLane) ? -1 : net.lanes[myLane].length - kin[v].pos;
                }
            }
            if (gap > 0) {
                const double ss = cfStopSpeed(v, kin[v].speed, gap);
                if (ss >= cfMinNextSpeed(v, kin[v].speed)) {
                    return ss;
                }
            }
        }
    }
    return DMAX;
}

// MSVehicle::adaptToLeader (Euler)
void Sim::adaptToLeader(int v, int pred, double gap, double seen, DriveItem* lastLink, double& vv, double& vLinkPass) const {
    if (cfIgnoresFoe(v, pred)) {
        return;
    }
    double vsafeLeader = BALLISTIC ? -DMAX : 0;
    bool backOnRoute = true;
    if (gap < 0 && lastLink != nullptr && lastLink->link >= 0) {
        backOnRoute = false;
        const int current = net.links[lastLink->link].viaOrLane();
        const int predBack = backLane(pred);
        if (predBack == current) {
            backOnRoute = true;
        } else {
            for (int lane : bestContinuation(v)) {
                if (lane == current) {
                    break;
                }
                if (predBack == lane) {
                    backOnRoute = true;
                }
            }
        }
        if (!backOnRoute) {
            double stopDist = seen - net.lanes[current].length - POSITION_EPS;
            const int ilb = net.links[lastLink->link].internalLaneBefore;
            if (ilb >= 0) {
                stopDist -= net.lanes[ilb].length;
            }
            vsafeLeader = cfStopSpeed(v, kin[v].speed, stopDist);
        }
    }
    const VType& pt = vt(pred);
    if (backOnRoute) {
        vsafeLeader = cfFollowSpeed(v, kin[v].speed, gap, kin[pred].speed, pt.apparentDecel, pred);
    }
    if (lastLink != nullptr) {
        const double futureVSafe = cfFollowSpeed(v, lastLink->accelV, gap, kin[pred].speed, pt.apparentDecel, pred, cf::CalcReason::FUTURE);
        lastLink->adaptLeaveSpeed(futureVSafe);
    }
    vv = std::min(vv, vsafeLeader);
    vLinkPass = std::min(vLinkPass, vsafeLeader);
}

// MSVehicle::adaptToJunctionLeader (Euler); leader == v: braking for a pedestrian
void Sim::adaptToJunctionLeader(int v, int leader, double gap, double seen, DriveItem* lastLink, int lane, double& vv,
                                double& vLinkPass, double distToCrossing) const {
    const VType& t = vt(v);
    const double speed = kin[v].speed;
    double vsafeLeader = BALLISTIC ? -DMAX : 0;
    const bool departed = veh[v].status != VehStatus::PENDING;  // MSBaseVehicle::hasDeparted (teleports too)
    if (leader != v && cfIgnoresFoe(v, leader)) {
        return;
    }
    if (leader == v) {
        // pedestrian: (gap < 0) no following, stop at the crossing
    } else if (gap >= 0) {
        if (departed) {
            vsafeLeader = cfFollowSpeed(v, speed, gap, kin[leader].speed, vt(leader).apparentDecel, leader);
        } else {
            vsafeLeader = cfInsertionFollowSpeed(v, speed, gap, kin[leader].speed, vt(leader).apparentDecel, leader);
        }
    } else if (leader != v) {
        vsafeLeader = cfStopSpeed(v, speed, seen - net.lanes[lane].length - POSITION_EPS);
    }
    if (distToCrossing >= 0) {
        const double vStop = cfStopSpeed(v, speed, distToCrossing - t.minGap);
        if (leader == v) {
            // braking for pedestrian
            vsafeLeader = cfStopSpeed(v, speed, distToCrossing);
            if (lastLink != nullptr) {
                lastLink->adaptStopSpeed(vsafeLeader);
            }
        } else if (gap == -DMAX) {
            vsafeLeader = std::max(vsafeLeader, vStop);
        } else {
            const double leaderDistToCrossing = distToCrossing - gap;
            const double leaderPastCPTime = leaderDistToCrossing / std::max(kin[leader].speed, HALTING_SPEED);
            const double vFinal = std::max(speed, 2 * (distToCrossing - t.minGap) / leaderPastCPTime - speed);
            const double v2 = speed + ACCEL2SPEED((vFinal - speed) / leaderPastCPTime);
            vsafeLeader = std::max(vsafeLeader, std::min(v2, vStop));
        }
    }
    if (lastLink != nullptr) {
        lastLink->adaptLeaveSpeed(vsafeLeader);
    }
    vv = std::min(vv, vsafeLeader);
    vLinkPass = std::min(vLinkPass, vsafeLeader);
}

// MSVehicle::checkLinkLeaderCurrentAndParallel: with a shadow lane also the leaders of the parallel link
void Sim::checkLinkLeader(int v, int link, int lane, double seen, DriveItem* lastLink, double& vv, double& vLinkPass,
                          double& vLinkWait, bool& setRequest) const {
    if (!net.usingInternalLanes) {
        return;  // (no junction leaders without internal lanes)
    }
    checkLinkLeader1(v, link, lane, seen, lastLink, vv, vLinkPass, vLinkWait, setRequest, false);
    if (sublane && kin[v].shadowLane >= 0) {
        const int dir = shadowDirection(v);
        const int parallelLink = dir == -1 ? net.links[link].parallelRight : dir == 1 ? net.links[link].parallelLeft : -1;
        if (parallelLink >= 0) {
            checkLinkLeader1(v, parallelLink, lane, seen, lastLink, vv, vLinkPass, vLinkWait, setRequest, true);
        }
    }
}

// MSVehicle::checkLinkLeader
void Sim::checkLinkLeader1(int v, int link, int lane, double seen, DriveItem* lastLink, double& vv, double& vLinkPass,
                           double& vLinkWait, bool& setRequest, bool isShadowLink) const {
    thread_local std::vector<LinkLeader> leaders;
    leaderInfo(link, v, seen, leaders, false, isShadowLink);
    const int myLane = kin[v].lane;
    const int myPred = net.lanes[myLane].logicalPred;
    for (const LinkLeader& ll : leaders) {
        const int leader = ll.veh;
        // jmIgnoreJunctionFoeProb
        const double ignoreProb = vt(v).jmIgnoreJunctionFoeProb;
        if (ignoreProb > 0 && ignoreProb >= rand01(opt.seed, (uint64_t)v * 1000003ULL + (uint64_t)(leader + 1), (uint64_t)step,
                                                   RP_JM_IGNORE + 16)) {
            continue;
        }
        if (leader < 0) {
            // leader is a pedestrian
            adaptToJunctionLeader(v, v, -1, seen, lastLink, lane, vv, vLinkPass, ll.distToCrossing);
            // if blocked by a pedestrian for too long we must yield our request
            if (vv < HALTING_SPEED && veh[v].waitingTime > TIME2STEPS(JUNCTION_BLOCKAGE_TIME)) {
                setRequest = false;
            }
        } else if (isLeader(v, link, leader, ll.gap) || (ll.flags & 1)) {
            if (latRes > 0 && net.lanes[kin[leader].lane].edge == net.lanes[lane].edge) {
                // sublane obstruction by a sibling link leader (assumed to have the geometry of the leader's lane)
                const int conflictLane = net.links[link].internalLaneBefore;
                LeaderInfo linkLeadersAhead(*this, net.lanes[conflictLane].width);
                linkLeadersAhead.addLeader(leader, false, 0);
                const double latOff = isShadowLink ? net.lanes[kin[v].lane].rightSideOnEdge - net.lanes[kin[v].shadowLane].rightSideOnEdge : 0;
                adaptToLeaders(v, linkLeadersAhead, latOff, seen, lastLink, kin[leader].lane, vv, vLinkPass);
            } else {
                adaptToJunctionLeader(v, leader, ll.gap, seen, lastLink, lane, vv, vLinkPass, ll.distToCrossing);
            }
            if (lastLink != nullptr) {
                vv = std::max(vv, lastLink->vLinkWait);
            }
            const int leaderPred = net.lanes[kin[leader].lane].logicalPred;
            if (vv < HALTING_SPEED
                    && (leaderPred == myPred || leaderPred == myLane || isStopped(leader)
                        || veh[leader].waitingTime > TIME2STEPS(JUNCTION_BLOCKAGE_TIME))) {
                setRequest = false;
                if (lastLink != nullptr && leaderPred == myLane) {
                    lastLink->setRequest = false;
                }
            }
        }
    }
    vLinkWait = std::min(vLinkWait, vv);
}

// MSVehicle::getArrivalTime (Euler)
Time Sim::arrivalTimeAt(int v, double seen, double vv, double arrivalSpeed) const {
    // (Euler: the vehicle moves with vv in this step)
    Time arrivalTime = now - DELTA_T + cf::getMinimalArrivalTime(vt(v), seen, BALLISTIC ? kin[v].speed : vv, arrivalSpeed);
    if (isStopped(v)) {
        arrivalTime += std::max((Time)0, veh[v].stops.front().duration);
    }
    return arrivalTime;
}

void Sim::planMove(int v, int leader, double lengthsInFront, const LeaderInfo* ahead) {
    Veh& x = veh[v];
    x.stopSpeed = cfMaxNextSpeed(v, x.stopSpeed);
    // MSVehicle::checkActionStep: between actions the plan is kept (the passed drive items are removed)
    x.actionStep = isActionStep(v, now);
    if (!x.actionStep) {
        x.lf.erase(x.lf.begin(), x.lf.begin() + std::min(x.nextDriveItem, (int)x.lf.size()));
        x.nextDriveItem = 0;
        return;
    }
    x.lastActionTime = now;
    planMoveInternal(v, leader, ahead, x.lf, x.stopDist, x.stopSpeed);
    checkRewindLinkLanes(v, lengthsInFront, x.lf);
    // MSVehicle::setApproachingForAllLinks: a random tie breaker at all-way stops
    for (int i = 0; i < (int)x.lf.size(); ++i) {
        DriveItem& d = x.lf[i];
        if (d.link >= 0 && net.links[d.link].state == LS_ALLWAY_STOP) {
            d.arrivalTime += rand01(opt.seed, (uint64_t)v * 1024 + (uint64_t)i, (uint64_t)step, RP_ALLWAY) < 0.5 ? 0 : 1;
        }
    }
    x.actionSpeed = kin[v].speed;
    x.actionWaitingTime = x.waitingTime;
    x.actionHaveStoppedFor = x.haveStoppedFor;
    if (sublane) {
        const Kin& k = kin[v];
        x.actionPosLat = k.posLat;
        x.actionShadowDir = (int8_t)(k.shadowLane >= 0 ? shadowDirection(v) : 0);
        x.actionShadowLatOffset = k.shadowLane >= 0 ? net.lanes[k.lane].rightSideOnEdge - net.lanes[k.shadowLane].rightSideOnEdge : 0;
    }
    if (v == traceVeh) {
        std::fprintf(stderr, "%.0f plan %s lane=%s pos=%.3f v=%.3f wait=%lld leader=%s items:", STEPS2TIME(now), x.id.c_str(),
                     net.lanes[kin[v].lane].id.c_str(), kin[v].pos, kin[v].speed, (long long)x.waitingTime,
                     leader < 0 ? "-" : veh[leader].id.c_str());
        for (const DriveItem& d : x.lf) {
            std::fprintf(stderr, " [%s pass=%.3f wait=%.3f req=%d at=%lld as=%.3f dist=%.2f space=%.2f stopped=%d]",
                         d.link < 0 ? "-" : net.lanes[net.links[d.link].viaOrLane()].id.c_str(), d.vLinkPass, d.vLinkWait,
                         (int)d.setRequest, (long long)d.arrivalTime, d.arrivalSpeed, d.distance, d.availableSpace,
                         (int)d.hadStoppedVehicle);
        }
        std::fprintf(stderr, "\n");
    }
    x.nextDriveItem = 0;
    x.lc.changed = false;
}

// MSVehicle::adaptToLeaders: the leaders of the sublanes the vehicle occupies (no opposite or bidi leaders)
void Sim::adaptToLeaders(int v, const LeaderInfo& ahead, double latOffset, double seen, DriveItem* lastLink, int lane, double& vv,
                         double& vLinkPass) const {
    int rightmost, leftmost;
    ahead.getSubLanes(v, latOffset, rightmost, leftmost);
    const Kin& k = kin[v];
    const VType& t = vt(v);
    for (int sublane = rightmost; sublane <= leftmost; ++sublane) {
        const int pred = ahead[sublane];
        if (pred >= 0 && pred != v) {
            const double predBack = backPositionOnLane(pred, lane);
            const double gap = lastLink == nullptr ? predBack - k.pos - t.minGap : predBack + seen - net.lanes[lane].length - t.minGap;
            adaptToLeader(v, pred, gap, seen, lastLink, vv, vLinkPass);
        }
    }
}

// MSLane::getLastVehicleInformation
LeaderInfo Sim::lastVehicleInformation(int lane, int ego, double latOff, double minPos, int ignore) const {
    return lastVehicleInformationT(lane, ego, latOff, minPos, ignore, [this](int l, auto f) { anyVehicles(l, f); });
}

void Sim::planMoveInternal(int v, int leader, const LeaderInfo* aheadInfo, std::vector<DriveItem>& lfLinks, double& newStopDist,
                           double& newStopSpeed) {
    struct Owner {  // the planning vehicle (cfOwner)
        explicit Owner(int v) { cfOwner = v; }
        ~Owner() { cfOwner = -1; }
    } owner(v);
    Veh& x = veh[v];
    const Kin& k = kin[v];
    const VType& t = vt(v);
    lfLinks.clear();
    newStopDist = DMAX;
    const double vehicleLength = t.length;
    const double maxV = cfMaxNextSpeed(v, k.speed);
    const double maxVD = std::max(vehMaxSpeed(v), std::min(maxV, t.desiredMaxSpeed));
    double laneMaxV = laneVehMaxSpeedCapped(k.lane, v, maxVD);
    const double vMinComfortable = cfMinNextSpeed(v, k.speed);
    const bool isRail = railActive && isRailVeh(v);
    if (isRail) {
        // speed limits must hold for the whole length of the train
        for (int i = 0; i < k.nFurther; ++i) {
            laneMaxV = std::min(laneMaxV, laneVehMaxSpeedCapped(k.further[i], v, maxVD));
        }
    }
    laneMaxV = std::max(laneMaxV, vMinComfortable);
    double vv = cfMaximumLaneSpeedCF(v, maxV, laneMaxV);
    const double dist = SPEED2DIST(maxV) + cfBrakeGap(v, maxV);
    const std::vector<int>& bestLaneConts = bestContinuation(v);
    bool hadNonInternal = false;
    double seen = net.lanes[k.lane].length - k.pos;
    double seenNonInternal = 0;
    double seenInternal = isInternal(k.lane) ? seen : 0;
    double vLinkPass = std::min(cf::estimateSpeedAfterDistance(t, seen, vv, t.accel), laneMaxV);
    // optionally slow down to match the arrival time of the next stop
    if (t.speedFactorPremature > 0 && vv > vMinComfortable && !x.stops.empty() && x.stops.front().arrival >= 0
            && vv > net.lanes[k.lane].speed * t.speedFactorPremature && !x.stops.front().reached) {
        vv = std::min(vv, slowDownForSchedule(v, vMinComfortable));
    }
    int view = 0;
    int lastLinkIdx = -1;  // index into lfLinks
    bool slowedDownForMinor = false;
    int lane = k.lane;
    int leaderLane = k.lane;
    int ahead = leader;
    // with a lateral resolution: the leaders per sublane, the lateral shift of the links passed, the next turn
    const bool subl = latRes > 0;
    LeaderInfo aheadSL = subl ? *aheadInfo : LeaderInfo(*this, 0.);
    double lateralShift = 0;
    bool encounteredTurn = !subl;
    if (subl) {
        sl[v].nextTurnDist = seen;
        sl[v].nextTurnLink = -1;
    }
    bool planningToStop = false;
    bool foundRailSignal = !isRail;
    double mustSeeBeforeReversal = 0;
    size_t stopIt = 0;
    while (true) {
        DriveItem* lastLink = lastLinkIdx >= 0 ? &lfLinks[lastLinkIdx] : nullptr;
        // adaptToLeaders
        if (subl) {
            if (k.shadowLane < 0) {
                // driving outside the lane bounds: potential leaders also outside the bounds
                const double laneW = net.lanes[lane].width;
                const double rightOL = rightSideOnLane(v, lane) + lateralShift;
                const double leftOL = leftSideOnLane(v, lane) + lateralShift;
                const bool outsideLeft = leftOL > laneW;
                if (rightOL < 0 || outsideLeft) {
                    LeaderInfo outsideLeaders(*this, laneW);
                    const int sublaneOffset = outsideLeft ? std::min(-1, -(int)std::ceil((leftOL - laneW) / latRes))
                                              : std::max(1, (int)std::ceil(-rightOL / latRes));
                    outsideLeaders.setSublaneOffset(sublaneOffset);
                    const double edgeWidth = net.edges[net.lanes[lane].edge].width;
                    for (int cand : lanes[lane].vehs) {
                        if ((lane != k.lane || kin[cand].pos > k.pos)
                                && ((!outsideLeft && leftSideOnEdge(cand) < 0) || (outsideLeft && leftSideOnEdge(cand) > edgeWidth))) {
                            outsideLeaders.addLeader(cand, true);
                        }
                    }
                    if (outsideLeaders.hasVehicles()) {
                        adaptToLeaders(v, outsideLeaders, lateralShift, seen, lastLink, leaderLane, vv, vLinkPass);
                    }
                }
            }
            adaptToLeaders(v, aheadSL, lateralShift, seen, lastLink, leaderLane, vv, vLinkPass);
        } else if (ahead >= 0 && ahead != v) {
            const double predBack = backPositionOnLane(ahead, leaderLane);
            const double gap = lastLink == nullptr ? predBack - k.pos - t.minGap
                               : predBack + seen - net.lanes[lane].length - t.minGap;
            adaptToLeader(v, ahead, gap, seen, lastLink, vv, vLinkPass);
        }
        if (lastLink != nullptr) {
            lastLink->vLinkWait = std::min(lastLink->vLinkWait, vv);
        }
        // also slow down for leaders on the shadow lane relative to the current lane (a continuous lane change
        // cannot be stopped, so the leader on the target lane matters before the midpoint)
        if (sublane && k.shadowLane >= 0) {
            const int shadowLane = shadowLaneFor(v, leaderLane, k.posLat);
            if (subl && shadowLane >= 0 && net.lanes[shadowLane].edge == net.lanes[leaderLane].edge) {
                const double latOff = net.lanes[k.lane].rightSideOnEdge - net.lanes[k.shadowLane].rightSideOnEdge;
                const LeaderInfo shadowLeaders = lastVehicleInformation(shadowLane, v, latOff, net.lanes[lane].length - seen);
                const double turningDifference = std::max(0.0, net.lanes[leaderLane].length - net.lanes[shadowLane].length);
                adaptToLeaders(v, shadowLeaders, latOff, seen - turningDifference, lastLink, shadowLane, vv, vLinkPass);
            } else if (shadowLane >= 0 && (lateralOverlap(v, k.posLat, k.lane) > POSITION_EPS || x.lc.completion < 0.5)
                    && net.lanes[shadowLane].edge == net.lanes[leaderLane].edge) {
                // MSLane::getLastVehicleInformation (one sublane) + adaptToLeaders
                const int pred = lastVehicleFrom(shadowLane, net.lanes[lane].length - seen, v);
                if (pred >= 0) {
                    const double turningDifference = std::max(0.0, net.lanes[leaderLane].length - net.lanes[shadowLane].length);
                    const double shadowSeen = seen - turningDifference;
                    const double predBack = backPositionOnLane(pred, shadowLane);
                    const double gap = lastLink == nullptr ? predBack - k.pos - t.minGap
                                       : predBack + shadowSeen - net.lanes[shadowLane].length - t.minGap;
                    adaptToLeader(v, pred, gap, shadowSeen, lastLink, vv, vLinkPass);
                }
            }
        }
        // adapt to pedestrians on the same lane
        if (hasPedestrians(lane)) {
            const double relativePos = net.lanes[lane].length - seen;
            const double stopTime = std::max(1.0, std::ceil(k.speed / t.decel));
            const std::pair<int, double> pl = nextBlocking(lane, relativePos, rightSideOnLane(v, lane),
                                                           rightSideOnLane(v, lane) + t.width, stopTime);
            if (pl.first >= 0) {
                vv = std::min(vv, cfStopSpeed(v, k.speed, pl.second - t.minGap));
            }
        }
        // adapt to vehicles blocked from (urgent) lane-changing
        if (net.edges[net.lanes[lane].edge].hasLaneChanger) {
            const double vHelp = cooperativeHelpSpeed(v, lane, seen);
            vv = std::min(vv, vHelp);
        }
        // stops on the current edge
        bool foundRealStop = false;
        while (stopIt < x.stops.size() && net.lanes[x.stops[stopIt].lane].edge == net.lanes[lane].edge
                && x.stops[stopIt].routeIdx == x.routeIdx + view) {
            const Stop& stop = x.stops[stopIt];
            const bool isFirstStop = stopIt == 0;
            stopIt++;
            if (!stop.reached || (stop.speed > 0 && keepStopping(v, false))) {
                // we are approaching a stop on the edge; must not drive further
                const bool isWaypoint = stop.speed > 0;
                double endPos = stopEndPos(v, stop) + NUMERICAL_EPS;
                if (stop.parkingArea >= 0) {
                    // a parking area: leave enough space so parking vehicles can exit
                    endPos = paLastFreePosWithReservation(stop.parkingArea, v, cfBrakeGap(v, k.speed, t.decel, 0.) + net.lanes[lane].length - seen);
                } else if (isWaypoint && !stop.reached) {
                    endPos = stop.startPos;
                }
                double stopDist = seen + endPos - net.lanes[lane].length;
                double stopSpeed = laneMaxV;
                if (isWaypoint) {
                    bool waypointWithStop = false;
                    if (stop.until > now) {
                        // check if we have to slow down or even stop
                        const Time time2end = stop.reached
                                              ? TIME2STEPS((stop.endPos - k.pos) / stop.speed)
                                              // time to reach the waypoint start and its end
                                              : TIME2STEPS(stopDist / ((k.speed + stop.speed) / 2) + (stop.endPos - stop.startPos) / stop.speed);
                        if (stop.until > now + time2end) {
                            double distToEnd = stopDist;
                            if (!stop.reached) {
                                distToEnd += stop.endPos - stop.startPos;
                            }
                            stopSpeed = std::max(cfStopSpeed(v, k.speed, distToEnd), vMinComfortable);
                            waypointWithStop = true;
                        }
                    }
                    if (stop.reached) {
                        stopSpeed = std::min(stop.speed, stopSpeed);
                        if (k.pos >= stop.endPos && !waypointWithStop) {
                            stopDist = DMAX;
                        }
                    } else {
                        stopSpeed = std::min(std::max(cfFreeSpeed(v, k.speed, stopDist, stop.speed), vMinComfortable), stopSpeed);
                        stopDist += stop.endPos - stop.startPos;
                        if (lastLink != nullptr) {
                            lastLink->adaptLeaveSpeed(cfFreeSpeed(v, vLinkPass, endPos, stop.speed));
                        }
                    }
                } else {
                    // regular stops are not emergencies
                    stopSpeed = std::max(cfStopSpeed(v, k.speed, stopDist), vMinComfortable);
                    if (lastLink != nullptr) {
                        lastLink->adaptLeaveSpeed(cfStopSpeed(v, vLinkPass, endPos, -1, cf::CalcReason::FUTURE));
                    }
                }
                if (stopSpeed < k.speed && k.speed > HALTING_SPEED) {
                    newStopSpeed = std::min(newStopSpeed, stopSpeed);
                } else if (k.speed < HALTING_SPEED) {
                    newStopSpeed = DMAX;
                }
                vv = std::min(vv, stopSpeed);
                if (isInternal(lane)) {
                    const int exitLink = succLinkSec(v, view + 1, lane, bestLaneConts);
                    bool dummyReq;
                    double dummyWait;
                    checkLinkLeader(v, exitLink, lane, seen, lastLink, vv, vLinkPass, dummyWait, dummyReq);
                }
                if (isFirstStop) {
                    newStopDist = stopDist;
                    // if the vehicle is going to stop we don't need to look further (except for trains, which
                    // make use of further link approach registrations for safety); waypoints are passed
                    if (isWaypoint) {
                        continue;
                    }
                    planningToStop = true;
                    if (!isRail) {
                        DriveItem d;
                        d.vLinkPass = d.vLinkWait = vv;
                        d.distance = stopDist;
                        lfLinks.push_back(d);
                        foundRealStop = true;
                        break;
                    }
                }
            }
        }
        if (foundRealStop) {
            break;
        }
        const int link = succLinkSec(v, view + 1, lane, bestLaneConts);
        // the next turn (for the lateral alignment of the sublane model)
        if (!encounteredTurn && link >= 0 && net.lanes[lane].links.size() > 1 && net.links[link].dir != 's') {
            sl[v].nextTurnDist = seen;
            sl[v].nextTurnLink = link;
            encounteredTurn = true;
        }
        // final edge
        if (x.routeIdx + view + 1 == (int)routes[x.route].size()) {
            const double arrivalSpeed = x.arrivalSpeed >= 0 ? x.arrivalSpeed : laneMaxV;
            const double distToArrival = seen + x.arrivalPos - net.lanes[lane].length - SPEED2DIST(arrivalSpeed);
            const double va = std::max(NUMERICAL_EPS, cfFreeSpeed(v, k.speed, distToArrival, arrivalSpeed));
            vv = std::min(vv, va);
            if (lastLink != nullptr) {
                lastLink->adaptLeaveSpeed(va);
            }
            DriveItem d;
            d.vLinkPass = d.vLinkWait = vv;
            d.distance = seen;
            d.availableSpace = net.edges[net.lanes[lane].edge].fringe ? 1000 : 0;
            lfLinks.push_back(d);
            break;
        }
        // dead end
        if (link < 0) {
            const double va = cfStopSpeed(v, k.speed, seen);
            if (lastLink != nullptr) {
                lastLink->adaptLeaveSpeed(va);
            }
            vv = subl && sl[v].committedSpeed > 0 ? std::min(sl[v].committedSpeed, vv) : std::min(va, vv);
            DriveItem d;
            d.vLinkPass = d.vLinkWait = vv;
            d.distance = seen;
            lfLinks.push_back(d);
            break;
        }
        if (sublane && brakeForOverlap(v, link, lane)) {
            // the shadow lane is a dead end
            const double va = cfStopSpeed(v, k.speed, seen);
            if (lastLink != nullptr) {
                lastLink->adaptLeaveSpeed(va);
            }
            vv = subl && sl[v].committedSpeed > 0 ? std::min(sl[v].committedSpeed, vv) : std::min(va, vv);
        }
        lateralShift += net.links[link].lateralShift;
        const Link& L = net.links[link];
        const bool yellowOrRed = L.haveRed() || L.haveYellow();
        double laneStopOffset;
        const double majorStopOffset = std::max(t.jmStoplineGap >= 0 ? t.jmStoplineGap : DIST_TO_STOPLINE_EXPECT_PRIORITY, 0.);
        const double stopDecel = yellowOrRed && !isRail ? std::max(std::min(TLS_YELLOW_MIN_DECEL, t.emergencyDecel), t.decel) : t.decel;
        const double brakeDist = cfBrakeGap(v, k.speed, stopDecel, 0);
        const bool canBrakeBeforeLaneEnd = seen >= brakeDist;
        const bool canBrakeBeforeStopLine = seen >= brakeDist;
        if (yellowOrRed) {
            laneStopOffset = majorStopOffset;
        } else if (L.havePriority()) {
            laneStopOffset = std::min(L.visibility - POSITION_EPS, majorStopOffset);
        } else {
            // stop before a pedestrian crossing that is a foe of the link
            double minorStopOffset = std::max(0., t.jmStoplineCrossingGap - L.distToFoePedCrossing);
            if (L.state == LS_ALLWAY_STOP) {
                minorStopOffset = std::max(minorStopOffset, t.jmStoplineGap >= 0 ? t.jmStoplineGap : 0.);
            } else {
                minorStopOffset = std::max(minorStopOffset, t.jmStoplineGapMinor >= 0 ? t.jmStoplineGapMinor : 0.);
            }
            laneStopOffset = std::min(L.visibility - POSITION_EPS, minorStopOffset);
        }
        if (canBrakeBeforeLaneEnd) {
            laneStopOffset = std::min(laneStopOffset, seen - brakeDist);
        }
        laneStopOffset = std::max(POSITION_EPS, laneStopOffset);
        double stopDist = std::max(0., seen - laneStopOffset);
        if (newStopDist != DMAX) {
            stopDist = std::max(stopDist, newStopDist);
        }
        if (isRail && !isInternal(lane)) {
            // check for train direction reversal
            if (net.lanes[lane].bidi >= 0 && net.lanes[L.lane].bidi == lane) {
                const double vMustReverse = cfStopSpeed(v, k.speed, seen - POSITION_EPS);
                if (seen < 1) {
                    mustSeeBeforeReversal = 2 * seen + t.length;
                }
                vv = std::min(vv, vMustReverse);
            }
            // a signal that is passed in the current step does not count
            foundRailSignal |= L.tls >= 0 && tlsRail[L.tls] >= 0 && seen > SPEED2DIST(vv);
        }
        // MSVehicle::checkReversal (for road vehicles the maximum speed: models that may exceed it, e.g. KraussPS
        // downhill, are capped here)
        if (isRail) {
            bool canReverseEventually = false;
            vv = std::min(vv, checkReversal(v, canReverseEventually, laneMaxV, seen));
        } else {
            vv = std::min(vv, vehMaxSpeed(v));
        }
        // slow down to finish a continuous lane change before a turn
        if (x.lc.isChangingLanes() && (L.dir == 'l' || L.dir == 'r')) {
            const double timeRemaining = STEPS2TIME(lcRemainingTime(v));
            const double va = std::max(cfStopSpeed(v, k.speed, seen - POSITION_EPS), (seen - POSITION_EPS) / timeRemaining);
            vv = std::min(va, vv);
        }
        // requests
        const bool leavingCurrentIntersection = isInternal(k.lane) && lastLink == nullptr;
        const bool abortRequestAfterMinor = slowedDownForMinor && L.internalLaneBefore < 0;
        bool setRequest = (vv > NUMERICAL_EPS_SPEED() && !abortRequestAfterMinor) || leavingCurrentIntersection;
        const double stopSpeed = cfStopSpeed(v, k.speed, stopDist, stopDecel, cf::CalcReason::CURRENT_WAIT);
        double vLinkWait = std::min(vv, stopSpeed);
        if (yellowOrRed && canBrakeBeforeStopLine && seen >= mustSeeBeforeReversal) {
            if (isInternal(lane)) {
                checkLinkLeader(v, link, lane, seen, lastLink, vv, vLinkPass, vLinkWait, setRequest);
            }
            const Time arrivalTime = arrivalTimeAt(v, seen, vv, vLinkPass);
            DriveItem d;
            d.link = link;
            d.vLinkPass = vv;
            d.vLinkWait = vLinkWait;
            d.setRequest = false;
            d.arrivalTime = arrivalTime;
            d.arrivalSpeed = vLinkWait;
            d.arrivalSpeedBraking = 0;
            d.distance = seen;
            d.accelV = -1;
            lfLinks.push_back(d);
            break;
        }
        checkLinkLeader(v, link, lane, seen, lastLink, vv, vLinkPass, vLinkWait, setRequest);
        lastLink = lastLinkIdx >= 0 ? &lfLinks[lastLinkIdx] : nullptr;
        if (lastLink != nullptr) {
            lastLink->adaptLeaveSpeed(laneMaxV);
        }
        double arrivalSpeed = vLinkPass;
        const double visibilityDistance = L.visibility;
        const bool determinedFoePresence = seen <= visibilityDistance;
        const bool couldBrakeForMinor = !L.havePriority() && brakeDist < seen && !net.lastWasContMajor(link);
        if (couldBrakeForMinor && !determinedFoePresence) {
            const double maxSpeedAtVisibilityDist = cf::maximumSafeStopSpeed(t, visibilityDistance, t.decel, k.speed, false, 0., false);
            const double maxArrivalSpeed = cf::estimateSpeedAfterDistance(t, visibilityDistance, maxSpeedAtVisibilityDist, t.accel);
            arrivalSpeed = std::min(vLinkPass, maxArrivalSpeed);
            slowedDownForMinor = true;
        } else if (L.state == LS_EQUAL && x.waitingTime > 0) {
            // check for deadlock (circular yielding)
            std::pair<int, int> blocker = firstApproachingFoe(link, link);
            int n = 100;
            while (blocker.second >= 0 && blocker.second != link && n > 0) {
                blocker = firstApproachingFoe(blocker.second, link);
                n--;
            }
            if (blocker.second == link) {
                if (getenv("PSIM_TRACE_DEADLOCK")) {
                    std::fprintf(stderr, "%.0f deadlock veh=%s link->%s\n", STEPS2TIME(now), x.id.c_str(), net.lanes[L.viaOrLane()].id.c_str());
                }
                const double threshold = L.dir == 's' ? 0.25 : 0.75;
                if (rand01(opt.seed, (uint64_t)v, (uint64_t)step, RP_DEADLOCK) < threshold) {
                    setRequest = false;
                }
            }
        }
        const Time arrivalTime = arrivalTimeAt(v, seen, vv, arrivalSpeed);
        if (couldBrakeForMinor && determinedFoePresence && net.edges[net.lanes[L.lane].edge].roundabout) {
            if (!opened(link, arrivalTime, arrivalSpeed, arrivalSpeed, t.length, impatience(v), t.decel, x.waitingTime, v)) {
                slowedDownForMinor = true;
            }
        }
        double arrivalSpeedBraking = 0;
        const double bGap = cfBrakeGap(v, vv);
        if (seen < bGap && !isStopped(v) && !planningToStop) {
            if (BALLISTIC) {
                arrivalSpeedBraking = cf::getMinimalArrivalSpeed(t, seen, k.speed);
            } else {
                arrivalSpeedBraking = cf::getMinimalArrivalSpeedEuler(t, seen, vv);
                // (braking must help despite the discrete / continuous mismatch)
                arrivalSpeedBraking = std::min(arrivalSpeedBraking, arrivalSpeed);
            }
        }
        const double estimatedLeaveSpeed = std::min(laneVehMaxSpeedCapped(L.viaOrLane(), v, maxVD),
                                                    cf::estimateSpeedAfterDistance(t, L.length, arrivalSpeed, t.accel));
        {
            DriveItem d;
            d.link = link;
            d.vLinkPass = vv;
            d.vLinkWait = vLinkWait;
            d.setRequest = setRequest;
            d.arrivalTime = arrivalTime;
            d.arrivalSpeed = arrivalSpeed;
            d.arrivalSpeedBraking = arrivalSpeedBraking;
            d.distance = seen;
            d.accelV = estimatedLeaveSpeed;
            lfLinks.push_back(d);
        }
        if (L.via < 0) {
            hadNonInternal = true;
            ++view;
        }
        if ((!setRequest || vv <= 0 || seen > dist) && hadNonInternal
                && seenNonInternal > std::max(vehicleLength * CRLL_LOOK_AHEAD, vehicleLength + seenInternal) && foundRailSignal) {
            break;
        }
        lane = L.viaOrLane();
        laneMaxV = laneVehMaxSpeedCapped(lane, v, maxVD);
        const double va = std::max(cfFreeSpeed(v, k.speed, seen, laneMaxV), vMinComfortable - NUMERICAL_EPS);
        vv = std::min(va, vv);
        if (isInternal(lane)) {
            seenInternal += net.lanes[lane].length;
        } else {
            seenNonInternal += net.lanes[lane].length;
        }
        leaderLane = lane;
        if (subl) {
            aheadSL = lastVehicleInformation(lane, -1, 0, 0);
        } else {
            ahead = lastAnyVehicle(lane);
        }
        seen += net.lanes[lane].length;
        vLinkPass = std::min(cf::estimateSpeedAfterDistance(t, net.lanes[lane].length, vv, t.accel), laneMaxV);
        lastLinkIdx = (int)lfLinks.size() - 1;
    }
}

// MSVehicle::checkRewindLinkLanes
void Sim::checkRewindLinkLanes(int v, double lengthsInFront, std::vector<DriveItem>& lfLinks) const {
    if (!net.usingInternalLanes || net.edges[net.lanes[kin[v].lane].edge].roundabout) {
        return;
    }
    const VType& t = vt(v);
    double seenSpace = -lengthsInFront;
    bool foundStopped = false;
    for (int i = 0; i < (int)lfLinks.size(); ++i) {
        DriveItem& item = lfLinks[i];
        if (item.link < 0 || foundStopped) {
            if (!foundStopped) {
                item.availableSpace += seenSpace;
            } else {
                item.availableSpace = seenSpace;
            }
            continue;
        }
        const Link& L = net.links[item.link];
        int approachedLane = L.via;
        const bool keepClear = L.hasFoes && L.keepClear;
        if (approachedLane >= 0) {
            if (keepClear) {
                seenSpace = seenSpace - lanes[approachedLane].brutto;
                if (approachedLane == kin[v].lane) {
                    seenSpace += t.lengthWithGap();
                }
            } else {
                seenSpace = seenSpace + spaceTillLastStanding(approachedLane, v, foundStopped);
            }
            item.availableSpace = seenSpace;
            continue;
        }
        approachedLane = L.lane;
        const int last = lastAnyVehicle(approachedLane);
        if (last < 0 || last == v) {
            if (net.lanes[approachedLane].length > t.length || keepClear) {
                seenSpace += net.lanes[approachedLane].length;
            }
            item.availableSpace = seenSpace;
        } else {
            bool foundStopped2 = false;
            const double spaceTill = spaceTillLastStanding(approachedLane, v, foundStopped2);
            seenSpace += spaceTill;
            if (foundStopped2) {
                foundStopped = true;
                item.hadStoppedVehicle = true;
            }
            item.availableSpace = seenSpace;
            if (veh[last].haveToWaitOnNextLink || isStopped(last)) {
                foundStopped = true;
                item.hadStoppedVehicle = true;
            }
        }
    }
    // check which links allow continuation and add pass available to the previous item
    for (int i = (int)lfLinks.size() - 1; i > 0; --i) {
        DriveItem& item = lfLinks[i - 1];
        DriveItem& nextItem = lfLinks[i];
        if (item.link < 0) {
            continue;  // SUMO dereferences item.myLink here; items without link only occur at the end
        }
        const Link& L = net.links[item.link];
        const bool canLeaveJunction = L.via < 0 || nextItem.link < 0 || nextItem.setRequest;
        const bool opn = canLeaveJunction
                         && (L.havePriority() || i == 1
                             || opened(item.link, item.arrivalTime, item.arrivalSpeed, item.leaveSpeed(), t.length,
                                       impatience(v), t.decel, veh[v].waitingTime, v));
        bool allowsContinuation = (L.isCont() || opn) && !item.hadStoppedVehicle;
        if (!opn) {
            foundStopped = true;
            if (i > 1) {
                DriveItem& item2 = lfLinks[i - 2];
                if (item2.link >= 0 && net.links[item2.link].isCont()) {
                    allowsContinuation = true;
                }
            }
        }
        if (allowsContinuation) {
            item.availableSpace = nextItem.availableSpace;
        }
    }
    // find removalBegin
    int removalBegin = -1;
    for (int i = 0; foundStopped && i < (int)lfLinks.size() && removalBegin < 0; ++i) {
        const DriveItem& item = lfLinks[i];
        if (item.link < 0) {
            continue;
        }
        const double leftSpace = item.availableSpace - t.lengthWithGap();
        const Link& L = net.links[item.link];
        if (leftSpace < 0 && L.hasFoes && L.keepClear) {
            removalBegin = i;
        }
    }
    if (removalBegin != -1 && !(removalBegin == 0 && isInternal(kin[v].lane))) {
        const double brakeGap = cfBrakeGap(v, kin[v].speed, t.decel, 0.);
        while (removalBegin < (int)lfLinks.size()) {
            DriveItem& dpi = lfLinks[removalBegin];
            if (dpi.link < 0) {
                break;
            }
            dpi.vLinkPass = dpi.vLinkWait;
            if (dpi.distance >= brakeGap + POSITION_EPS) {
                if (!net.links[dpi.link].isExitLink() || !lfLinks[removalBegin - 1].setRequest) {
                    dpi.setRequest = false;
                }
            }
            ++removalBegin;
        }
    }
}

// the further lanes of v (after a lane change or an insertion) whose reminders it does not hold
void Sim::setNoReminderFurther(int v) {
    Veh& x = veh[v];
    const Kin& k = kin[v];
    x.nNoRem = 0;
    for (int i = 0; i < k.nFurther; ++i) {
        x.noRem[x.nNoRem++] = k.further[i];
    }
}

bool Sim::holdsReminders(int v, int lane) const {
    const Veh& x = veh[v];
    return std::find(x.noRem, x.noRem + x.nNoRem, lane) == x.noRem + x.nNoRem;
}

// MSMeanData::MeanDataValues::notifyMove and MSMeanData_Net::notifyMoveInternal (Euler update,
// haltSpeed 0.1): the part of the step the vehicle spends on the lane, from the front entering to the
// back leaving. Positions are relative to `lane`.
void Sim::meanDataMove(int v, int lane, double oldPos, double newPos, double newSpeed, std::vector<EdSample>& eds) const {
    const double laneLength = net.lanes[lane].length;
    const double len = vt(v).length;
    double timeOnLane = TS;
    if (oldPos < 0 && newPos >= 0) {
        timeOnLane = TS - cf::passingTime(oldPos, 0, newPos, veh[v].prevSpeed, newSpeed);
    }
    const double oldBackPos = oldPos - len;
    const double newBackPos = newPos - len;
    if (newBackPos > laneLength && oldBackPos <= laneLength) {
        timeOnLane -= TS - cf::passingTime(oldBackPos, laneLength, newBackPos, veh[v].prevSpeed, newSpeed);
        if (std::fabs(timeOnLane) < NUMERICAL_EPS) {
            timeOnLane = 0.;
        }
    }
    if (timeOnLane <= 0) {
        return;
    }
    const double wait = !isStopped(v) && newSpeed < HALTING_SPEED ? timeOnLane : 0.;
    eds.push_back({lane, timeOnLane, timeOnLane * newSpeed, wait});
}

// ---------------------------------------------------------------------------
// executeMove
// ---------------------------------------------------------------------------
void Sim::processLinkApproaches(int v, double& vSafe, double& vSafeMin, double& vSafeMinDist) {
    Veh& x = veh[v];
    const Kin& k = kin[v];
    const VType& t = vt(v);
    x.haveToWaitOnNextLink = false;
    bool canBrakeVSafeMin = false;
    double vSafeZipper = DMAX;  // speed limit due to zipper merging
    for (const DriveItem& dpi : x.lf) {
        const int link = dpi.link;
        if (link >= 0 && dpi.setRequest) {
            const Link& L = net.links[link];
            const bool yellow = L.haveYellow();
            const bool canBrake = dpi.distance > cfBrakeGap(v, k.speed, t.decel, 0.) || (!BALLISTIC && k.speed < ACCEL2SPEED(t.decel));
            const bool beyondStopLine = dpi.distance < 0;  // lane stop offset is 0
            const bool ignoreRedLink = beyondStopLine;
            if (yellow && canBrake && !ignoreRedLink) {
                vSafe = dpi.vLinkWait;
                x.haveToWaitOnNextLink = true;
                break;
            }
            thread_local std::vector<int> collectFoes;
            collectFoes.clear();
            bool opened2 = yellow || opened(link, dpi.arrivalTime, dpi.arrivalSpeed, dpi.leaveSpeed(), t.length,
                                            canBrake ? impatience(v) : 1, t.decel, waitingTimeFor(v, link), v, ignoreRedLink,
                                            L.state == LS_ZIPPER ? &collectFoes : nullptr, dpi.distance);
            if (opened2 && sublane && kin[v].shadowLane >= 0) {
                // the parallel link of the shadow lane must be open as well
                const int dir = shadowDirection(v);
                const int parallelLink = dir == -1 ? L.parallelRight : dir == 1 ? L.parallelLeft : -1;
                if (parallelLink >= 0) {
                    const double shadowLatPos = kin[v].posLat - dir * 0.5 * (net.lanes[kin[v].lane].width + net.lanes[kin[v].shadowLane].width);
                    opened2 = yellow || opened(parallelLink, dpi.arrivalTime, dpi.arrivalSpeed, dpi.leaveSpeed(), t.length,
                                               canBrake ? impatience(v) : 1, t.decel, waitingTimeFor(v, link), v, ignoreRedLink,
                                               nullptr, dpi.distance, -1, &shadowLatPos);
                }
            }
            if (v == traceVeh) {
                std::fprintf(stderr, "%.0f exec %s link->%s opened=%d state=%c\n", STEPS2TIME(now), x.id.c_str(),
                             net.lanes[L.viaOrLane()].id.c_str(), (int)opened2, L.state);
                for (int foe : L.foeLinks) {
                    for (const Approach& a : approaching[foe]) {
                        std::fprintf(stderr, "    foe %s via %s at=%lld lt=%lld willPass=%d wt=%lld\n", veh[a.veh].id.c_str(),
                                     net.lanes[net.links[foe].viaOrLane()].id.c_str(), (long long)a.arrivalTime, (long long)a.leavingTime, (int)a.willPass, (long long)a.waitingTime);
                    }
                }
            }
            const double visibilityDistance = L.visibility;
            const bool determinedFoePresence = dpi.distance <= visibilityDistance;
            if (opened2 && !L.havePriority() && !net.lastWasContMajor(link) && !L.isCont() && !ignoreRedLink) {
                if (!determinedFoePresence && (canBrake || !yellow)) {
                    vSafe = dpi.vLinkWait;
                    x.haveToWaitOnNextLink = true;
                    break;
                } else {
                    vSafeMinDist = dpi.distance;
                    vSafeMin = std::min(std::min(BALLISTIC ? (double)DIST2SPEED(2 * vSafeMinDist + NUMERICAL_EPS) - k.speed
                                                 : (double)DIST2SPEED(vSafeMinDist + POSITION_EPS), dpi.vLinkPass),
                                        cfMaxNextSafeMin(v, k.speed));
                    canBrakeVSafeMin = canBrake;
                }
            }
            if (opened2) {
                vSafe = dpi.vLinkPass;
                if (vSafe < t.decel && vSafe <= dpi.vLinkWait && vSafe < cfMaxNextSpeed(v, k.speed)) {
                    x.haveToWaitOnNextLink = true;
                }
                if (L.mustStop() && determinedFoePresence && x.haveStoppedFor < 0) {
                    x.haveStoppedFor = link;
                }
            } else if (L.state == LS_ZIPPER) {
                vSafeZipper = std::min(vSafeZipper, zipperSpeed(v, link, dpi.distance, dpi.vLinkPass, dpi.arrivalTime, collectFoes));
            } else if (!canBrake && L.tls < 0 && dpi.distance < cfBrakeGap(v, k.speed, t.emergencyDecel, 0.)) {
                vSafe = dpi.vLinkPass;
            } else {
                vSafe = dpi.vLinkWait;
                x.haveToWaitOnNextLink = true;
                break;
            }
            if (isInternal(k.lane) && x.junctionEntryTime == TIME_MAX) {
                x.junctionEntryTime = now;
            }
        } else {
            if (link >= 0 && net.links[link].internalLaneBefore >= 0 && isInternal(k.lane)
                    && net.links[link].junction == net.edges[net.lanes[k.lane].edge].toJ) {
                x.junctionEntryTime = TIME_MAX;
                x.junctionConflictEntryTime = TIME_MAX;
            }
            vSafe = dpi.vLinkWait;
            if (link >= 0 || x.stopDist < (net.lanes[k.lane].length - k.pos)) {
                if (vSafe < k.speed) {
                    x.haveToWaitOnNextLink = true;
                }
            } else if (vSafe < HALTING_SPEED) {
                x.haveToWaitOnNextLink = true;
            }
            break;
        }
    }
    if (vSafe + NUMERICAL_EPS < vSafeMin && (!BALLISTIC || vSafeMin != 0)) {
        if (canBrakeVSafeMin && vSafe < k.speed) {
            vSafe = std::min(vSafe, std::max(cfMinNextSpeed(v, k.speed), cfStopSpeed(v, k.speed, vSafeMinDist)));
            vSafeMin = 0;
            x.haveToWaitOnNextLink = true;
        } else {
            vSafeMin = vSafe;
        }
    }
    // vehicles inside a roundabout should maintain their requests
    if (net.edges[net.lanes[k.lane].edge].roundabout) {
        x.haveToWaitOnNextLink = false;
    }
    vSafe = std::min(vSafe, vSafeZipper);
}

// MSCFModel::finalizeSpeed (cfmodels.cpp)
double Sim::finalizeSpeed(int v, double vPos, std::vector<StopEvent>& stopEv) {
    return cfFinalizeSpeed(v, vPos, stopEv);
}

bool Sim::hasArrivedInternal(int v) const {
    const Veh& x = veh[v];
    return (onLastEdge(v) || (x.extra.p != nullptr && x.extra.p->arrivalIdx >= 0 && x.routeIdx >= x.extra.p->arrivalIdx))
           && (x.stops.empty() || x.stops.front().routeIdx != x.routeIdx || x.stops.front().speed > 0)
           && kin[v].pos > x.arrivalPos - POSITION_EPS;
}

// MSVehicle::updateFurtherLanes; returns via kin[v].backPos
void Sim::updateFurtherLanes(int v, const std::vector<int>& passedLanes, const double* widthShiftPosLat) {
    Kin& k = kin[v];
    double backPosOnPreviousLane = k.pos - vt(v).length;
    int oldFurther[MAX_FURTHER];
    double oldPosLat[MAX_FURTHER];
    const int nOld = k.nFurther;
    if (sublane) {
        std::copy(k.further, k.further + nOld, oldFurther);
        std::copy(lat[v].furtherPosLat, lat[v].furtherPosLat + nOld, oldPosLat);
    }
    int n = 0;
    int fi = 0;
    if (passedLanes.size() > 1) {
        for (int i = (int)passedLanes.size() - 2; i >= 0 && backPosOnPreviousLane < 0; --i) {
            const int further = passedLanes[i];
            if (n < MAX_FURTHER) {
                if (sublane) {
                    // a known lateral position stays, a new lane gets the current one (or the previous further's)
                    double& fpl = lat[v].furtherPosLat[n];
                    if (fi < nOld && further == oldFurther[fi]) {
                        fpl = oldPosLat[fi];
                        ++fi;
                    } else if (n == 0) {
                        // (after a lateral shift on entering a lane: the position before the shift)
                        fpl = widthShiftPosLat != nullptr ? *widthShiftPosLat : k.posLat;
                    } else {
                        fpl = lat[v].furtherPosLat[n - 1];
                    }
                }
                k.further[n++] = further;
            } else {
                __atomic_add_fetch(&nFurtherOverflow, 1, __ATOMIC_RELAXED);
            }
            backPosOnPreviousLane += net.lanes[further].length;
        }
    }
    k.nFurther = n;
    k.backPos = backPosOnPreviousLane;
}

void Sim::executeMove(int v, std::vector<DetEvent>& det, std::vector<StopEvent>& stopEv, std::vector<EdSample>* eds,
                      std::vector<DriveWayEvent>* dwEv) {
    Veh& x = veh[v];
    Kin& k = kin[v];
    const VType& t = vt(v);
    x.oldLane = k.lane;
    x.oldPos = k.pos;
    x.jcPassed = -1;
    const size_t dwEvFirst = dwEv != nullptr ? dwEv->size() : 0;
    // lanes touched at the start of the step (for detectors): further lanes and the current lane
    int touched[MAX_FURTHER + 1];
    double touchedOffset[MAX_FURTHER + 1];
    int nTouched = 0;
    {
        double off = 0;
        for (int i = 0; i < k.nFurther; ++i) {
            off += net.lanes[k.further[i]].length;
            touched[nTouched] = k.further[i];
            touchedOffset[nTouched++] = off;
        }
        touched[nTouched] = k.lane;
        touchedOffset[nTouched++] = 0;
    }
    double vSafe = DMAX;
    double vSafeMin = -DMAX;
    double vSafeMinDist = 0;
    struct Owner {  // the moving vehicle (cfOwner)
        explicit Owner(int v) { cfOwner = v; }
        ~Owner() { cfOwner = -1; }
    } owner(v);
    if (x.actionStep) {
        processLinkApproaches(v, vSafe, vSafeMin, vSafeMinDist);
    } else {
        vSafe = k.speed + ACCEL2SPEED(k.accel);  // continue with the current acceleration
    }
    cfUpdateStartup(v, vSafe);
    double vNext = vSafe;
    if (x.actionStep) {
        vNext = finalizeSpeed(v, vSafe, stopEv);
        if (vNext > 0) {
            vNext = std::max(vNext, vSafeMin);
        }
    }
    if (std::fabs(vNext) < NUMERICAL_EPS_SPEED() && (x.stopDist > POSITION_EPS || (!x.stops.empty() && x.stops.front().routeIdx == x.routeIdx))) {
        vNext = 0.;
    }
    if (!BALLISTIC) {
        vNext = std::max(vNext, 0.);  // (ballistic: a negative speed means a stop within the step)
    }
    // updateState
    double deltaPos;
    if (!BALLISTIC) {
        deltaPos = SPEED2DIST(vNext);
    } else if (vNext >= 0) {
        deltaPos = SPEED2DIST(k.speed + 0.5 * (vNext - k.speed));  // MSVehicle::getDeltaPos: constant acceleration
    } else {
        deltaPos = -SPEED2DIST(0.5 * k.speed * k.speed / (vNext - k.speed));  // the stop within the step
    }
    vNext = std::max(vNext, 0.);
    k.accel = SPEED2ACCEL(vNext - k.speed);
    x.prevSpeed = k.speed;
    k.speed = vNext;
    k.pos += deltaPos;
    x.lastStepDist = deltaPos;
    if (latRes > 0) {
        sl[v].nextTurnDist -= deltaPos;  // (MSVehicle::updateState: myNextTurn.first)
    }
    // updateWaitingTime
    const bool lowSpeed = vNext <= HALTING_SPEED && k.accel <= 0.5 * t.accel;
    if (lowSpeed && !isStopped(v)) {
        x.waitingTime += DELTA_T;
    } else {
        x.waitingTime = 0;
    }
    if (trackAccumulatedWaiting) {
        passWaitingTime(x, lowSpeed && !isStopped(v));
    }
    // MSDevice_Tripinfo::notifyMove: a stop does not end a waiting period
    if (isStopped(v)) {
        if ((x.stops.front().speed > 0 || !x.actionStep) && vNext <= HALTING_SPEED) {
            // a waypoint counts as stopping time while halting (a stop between actions: processNextStop counts at them)
            x.stopTime += TS;
        }
    } else if (lowSpeed) {
        x.tripWaiting += TS;
        if (!x.tripAmWaiting) {
            x.waitingCount++;
            x.tripAmWaiting = true;
        }
    } else {
        x.tripAmWaiting = false;
    }
    // processLaneAdvances
    thread_local std::vector<int> passedLanes;
    passedLanes.clear();
    bool widthShift = false;  // the lateral position before the last lateral shift (lateral resolution)
    double posLatBeforeShift = 0;
    for (int i = k.nFurther - 1; i >= 0; --i) {
        passedLanes.push_back(k.further[i]);
    }
    passedLanes.push_back(k.lane);
    // let trains reverse direction
    bool reverseTrain = false;
    const bool railClass = (x.vclass & VC_RAIL_CLASSES) != 0;
    if (railClass) {
        checkReversal(v, reverseTrain, HALTING_SPEED, 0);
        if (reverseTrain) {
            // some slack ensures that the back of the train does not appear looped
            k.pos += 2 * (net.lanes[k.lane].length - k.pos) + t.length + NUMERICAL_EPS;
            k.speed = 0;
        }
    }
    const int oldLane = k.lane;
    const int oldBackLane = backLane(v);
    bool emergency = false;
    int entered[16];
    double enteredOffset[16];
    int nEntered = 0;
    double enterOffset = 0;
    if (k.pos > net.lanes[k.lane].length) {
        if (!onLastEdge(v)) {
            int approachedLane = k.lane;
            int item = 0;
            while (item < (int)x.lf.size() && approachedLane >= 0 && k.pos > net.lanes[approachedLane].length) {
                const DriveItem& dpi = x.lf[item];
                ++item;
                x.nextDriveItem = item;
                if (opt.checkJunctions && mustCheckJunctionCollisions(approachedLane)) {
                    // passed within a single step: still checked (MSEdgeControl::checkCollisionForInactive)
                    x.jcPassed = approachedLane;
                }
                int link = dpi.link;
                if (link >= 0) {
                    const Link& L = net.links[link];
                    if (railClass && !isInternal(k.lane) && net.lanes[k.lane].bidi >= 0 && net.lanes[L.lane].bidi == k.lane && !reverseTrain) {
                        emergency = true;  // "because it must reverse direction"
                        approachedLane = -1;
                        break;
                    }
                    if (railClass && k.pos < net.lanes[k.lane].length + NUMERICAL_EPS && !x.stops.empty()
                            && x.stops.front().routeIdx == x.routeIdx) {
                        // trains plan past their stops: do not skip a stop due to numerical instability
                        approachedLane = k.lane;
                        k.pos = net.lanes[k.lane].length;
                        break;
                    }
                    approachedLane = L.viaOrLane();
                    if (L.haveRed() && dpi.distance >= 0 && !reverseTrain) {
                        emergency = true;  // "because of a red traffic light"
                        break;
                    }
                    if (reverseTrain && isInternal(approachedLane)) {
                        // avoid getting stuck on a slow turn-around internal lane
                        k.pos += net.lanes[approachedLane].length;
                    }
                } else if (k.pos < net.lanes[k.lane].length + NUMERICAL_EPS) {
                    approachedLane = k.lane;
                    k.pos = net.lanes[k.lane].length;
                } else if (reverseTrain) {
                    approachedLane = net.edges[routeEdge(v, 1)].lanes[0];
                    link = net.linkIdxTo(k.lane, approachedLane);
                    while (net.links[link].via >= 0) {
                        link = net.lanes[net.links[link].via].links[0];
                    }
                    --item;
                    x.nextDriveItem = item;
                } else {
                    emergency = true;  // "no connection to the next edge"
                    approachedLane = -1;
                    break;
                }
                if (approachedLane != k.lane && approachedLane >= 0) {
                    // MSVehicle::leaveLane: stops on the lane that is left are skipped, waypoints are passed
                    while (!x.stops.empty() && x.stops.front().routeIdx == x.routeIdx
                           && net.lanes[x.stops.front().lane].edge == net.lanes[k.lane].edge) {
                        Stop& st = x.stops.front();
                        if (st.speed <= 0) {
                            if (st.reached) {
                                break;  // (a stopped vehicle does not leave its lane)
                            }
                            x.stops.erase(x.stops.begin());
                            continue;
                        }
                        if (!st.reached) {
                            st.reached = true;
                            st.started = now;
                            if (st.busStop >= 0) {
                                stopEv.push_back({v, st.busStop, true, k.pos});
                            }
                        }
                        resumeFromStopping(v, stopEv);
                    }
                    k.pos -= net.lanes[k.lane].length;
                    enterOffset += net.lanes[k.lane].length;
                    x.routeLength += net.lanes[k.lane].length;
                    k.lane = approachedLane;
                    x.lastBestLanesEdge = -1;  // myLastBestLanesEdge = nullptr
                    if (!isInternal(approachedLane)) {
                        x.routeIdx++;
                    }
                    const Link& L = net.links[link];
                    if (L.isEntryLink()) {
                        x.junctionEntryTime = now;
                        x.junctionEntryTimeNeverYield = now;
                        x.haveStoppedFor = -1;
                    }
                    if (L.isConflictEntryLink()) {
                        x.junctionConflictEntryTime = now;
                        x.junctionEntryTime = x.junctionEntryTimeNeverYield;
                    }
                    if (L.isExitLink()) {
                        x.junctionEntryTime = TIME_MAX;
                        x.junctionEntryTimeNeverYield = TIME_MAX;
                        x.junctionConflictEntryTime = TIME_MAX;
                    }
                    if (dwEv != nullptr && ((approachedLane < (int)laneDriveWays.size() && !laneDriveWays[approachedLane].empty())
                                            || laneHasTracker(approachedLane))) {
                        dwEv->push_back({v, approachedLane, x.routeIdx, true});  // MSDriveWay::notifyEnter (junction)
                    }
                    if (nEntered < 16) {
                        entered[nEntered] = approachedLane;
                        enteredOffset[nEntered++] = -enterOffset;
                    }
                    passedLanes.push_back(approachedLane);
                    // MSVehicle::enterLaneAtMove: the lateral position on the entered lane
                    if (latRes > 0) {
                        widthShift = true;
                        posLatBeforeShift = k.posLat;
                        k.posLat += L.lateralShift;
                    } else if (std::fabs(k.posLat) > NUMERICAL_EPS) {
                        const int oldLane = passedLanes[passedLanes.size() - 2];
                        const double overlap = std::max(0.0, lateralOverlap(v, k.posLat, oldLane));
                        const double range = (net.lanes[oldLane].width - t.width) * 0.5 + overlap;
                        const double range2 = (net.lanes[approachedLane].width - t.width) * 0.5 + overlap;
                        k.posLat *= range2 / range;
                    }
                    if (hasArrivedInternal(v)) {
                        break;
                    }
                    if (x.lc.isChangingLanes() && (L.dir == 'l' || L.dir == 'r')) {
                        // "could not finish continuous lane change (turn lane)": aborted
                        endLaneChangeManeuver(v);
                    }
                }
            }
        } else if (!hasArrivedInternal(v) && k.pos < net.lanes[k.lane].length + NUMERICAL_EPS) {
            k.pos = net.lanes[k.lane].length;
        }
    }
    x.nEnteredNormal = 0;
    for (int i = 0; i < nEntered && x.nEnteredNormal < 4; ++i) {
        if (!isInternal(entered[i])) {
            x.enteredNormal[x.nEnteredNormal++] = entered[i];
        }
    }
    // updateTimeLoss
    if (!isStopped(v)) {
        const double vmax = std::min(laneVehMaxSpeed(k.lane, v), std::max(x.stopSpeed, vNext));
        if (vmax > 0) {
            x.timeLoss += TS * (vmax - vNext) / vmax;
        }
    }
    // detectors: move reminders of all lanes touched in this step
    for (int i = 0; i < nTouched; ++i) {
        if (!loopsOf(v, touched[i]).empty() && holdsReminders(v, touched[i])) {
            loopNotifyMove(v, touched[i], x.oldPos + touchedOffset[i], x.oldPos + touchedOffset[i] + SPEED2DIST(vNext), vNext, det);
        }
    }
    for (int i = 0; i < nEntered; ++i) {
        if (!loopsOf(v, entered[i]).empty()) {
            loopNotifyMove(v, entered[i], x.oldPos + enteredOffset[i], x.oldPos + enteredOffset[i] + SPEED2DIST(vNext), vNext, det);
        }
    }
    if (eds != nullptr) {
        // edge data: the lane reminders of MSMeanData_Net (normal lanes only)
        for (int i = 0; i < nTouched; ++i) {
            if (!isInternal(touched[i]) && holdsReminders(v, touched[i])) {
                meanDataMove(v, touched[i], x.oldPos + touchedOffset[i], x.oldPos + touchedOffset[i] + SPEED2DIST(vNext), vNext, *eds);
            }
        }
        for (int i = 0; i < nEntered; ++i) {
            if (!isInternal(entered[i])) {
                meanDataMove(v, entered[i], x.oldPos + enteredOffset[i], x.oldPos + enteredOffset[i] + SPEED2DIST(vNext), vNext, *eds);
            }
        }
    }
    if (hasRerouters() && !hasArrivedInternal(v)) {
        // rerouters (lane move reminders): notifyEnter for the lanes entered, notifyMove for those held; the driveway
        // entries of the lanes entered before a rerouting one see the old route (the reminders of each lane run in turn)
        const size_t dwFirst = dwEv != nullptr ? dwEvFirst : 0;
        std::vector<int> routeBefore;
        bool haveDwEnter = false;
        if (dwEv != nullptr) {
            for (size_t j = dwFirst; j < dwEv->size(); ++j) {
                haveDwEnter |= (*dwEv)[j].veh == v && (*dwEv)[j].enter;
            }
        }
        if (haveDwEnter) {
            routeBefore = routes[x.route];
        }
        for (int i = 0; i < nEntered; ++i) {
            if (VehExtra* e = x.exq()) {
                e->rrActive.clear();  // the reminders of the lane left
            }
            rerouterEnter(v, entered[i], false);
            if (haveDwEnter && routes[x.route] != routeBefore) {
                haveDwEnter = false;
                for (size_t j = dwFirst; j < dwEv->size(); ++j) {
                    DriveWayEvent& ev = (*dwEv)[j];
                    // (a rerouter runs before the driveways of its own lane: loaded first)
                    if (ev.veh == v && ev.enter && std::find(entered, entered + i, ev.lane) != entered + i) {
                        ev.oldRoute = true;
                        x.ex().dwOldRoute = routeBefore;
                    }
                }
            }
        }
        if (!x.exr().rrActive.empty()) {
            rerouterMove(v);
        }
    }
    if (rtTurnTT != nullptr && x.routingDevice) {
        // MSDevice_Routing::notifyEnter(NOTIFICATION_JUNCTION): the travel time of the edge before an internal edge
        for (int i = 0; i < nEntered; ++i) {
            Time& last = x.ex().lastLaneEntryTime;
            if (last >= 0 && isInternal(entered[i])) {
                const int ie = net.lanes[entered[i]].edge;
                rtTurnTT[ie].fetch_add(now - last, std::memory_order_relaxed);
                rtTurnCount[ie].fetch_add(1, std::memory_order_relaxed);
            }
            last = now;
        }
    }
    if (hasArrivedInternal(v)) {
        x.routeLength += x.arrivalPos;
        return;
    }
    if (emergency || k.pos > net.lanes[k.lane].length) {
        k.pos = net.lanes[k.lane].length;
        k.speed = 0;
        k.accel = 0;
        x.emergencyStop = true;
    }
    updateFurtherLanes(v, passedLanes, widthShift ? &posLatBeforeShift : nullptr);
    if (dwEv != nullptr && passedLanes.size() > 1 && isRailVeh(v)) {
        // MSVehicle::leaveLaneBack: the lanes passed (or occupied before) that the train no longer touches
        const int back = backLane(v);
        for (int i = (int)passedLanes.size() - 1; i >= 0; --i) {
            const int pl = passedLanes[i];
            if (pl != k.lane && std::find(k.further, k.further + k.nFurther, pl) == k.further + k.nFurther) {
                dwEv->push_back({v, pl, back, false});
            }
        }
    }
    if (x.nNoRem > 0) {
        // lanes the back has left drop out
        int n = 0;
        for (int i = 0; i < x.nNoRem; ++i) {
            if (std::find(k.further, k.further + k.nFurther, x.noRem[i]) != k.further + k.nFurther) {
                x.noRem[n++] = x.noRem[i];
            }
        }
        x.nNoRem = n;
    }
    if (x.aliasLane >= 0 && k.lane != x.aliasLane
            && std::find(k.further, k.further + k.nFurther, x.aliasLane) == k.further + k.nFurther) {
        x.aliasLane = x.aliasLoopLane = -1;  // the replaced reminders are all passed
    }
    // on a normal lane LC2013 updates the occupancies before it uses them; on an internal lane it keeps
    // those computed when the vehicle entered the lane
    updateBestLanes(v, false, -1, isInternal(k.lane));
    if (sublane && (k.lane != oldLane || backLane(v) != oldBackLane)
            && (k.shadowLane >= 0 || lateralOverlap(v, k.posLat, k.lane) > POSITION_EPS)) {
        updateShadowLane(v);  // the front or the back lane changed
    }
    if (x.actionStep) {
        lcPrepareStep(v);  // (between actions the state is kept)
    } else if (opt.laneChangeDuration > 0 && !x.lc.isChangingLanes()) {
        x.lc.speedLat = 0;  // MSAbstractLaneChangeModel::resetSpeedLat
    }
}

// MSVehicle::updateDriveItems (right after a lane change, moves the link registrations)
void Sim::updateDriveItems(int v) {
    Veh& x = veh[v];
    if (x.lf.empty()) {
        return;
    }
    int nextPlannedLink = -1;
    for (int i = x.nextDriveItem; i < (int)x.lf.size() && nextPlannedLink < 0; ++i) {
        nextPlannedLink = x.lf[i].link;
    }
    if (nextPlannedLink < 0) {
        return;
    }
    const int lane = kin[v].lane;
    if (lane == net.links[nextPlannedLink].laneBefore) {
        return;
    }
    int parallelLink = net.links[nextPlannedLink].parallelLeft;
    if (!(parallelLink >= 0 && net.links[parallelLink].laneBefore == lane)) {
        parallelLink = net.links[nextPlannedLink].parallelRight;
        if (!(parallelLink >= 0 && net.links[parallelLink].laneBefore == lane)) {
            return;
        }
    }
    const std::vector<int>& conts = bestContinuation(v);
    size_t bestLaneIt = 1;
    int cur = lane;
    for (int i = x.nextDriveItem; i < (int)x.lf.size();) {
        DriveItem& d = x.lf[i];
        if (d.link < 0) {
            ++i;
            continue;
        }
        if (bestLaneIt >= conts.size()) {
            // remove the leftover link items
            for (int j = i; j < (int)x.lf.size();) {
                if (x.lf[j].link < 0) {
                    ++j;
                } else {
                    std::vector<Approach>& a = approaching[x.lf[j].link];
                    auto it = std::lower_bound(a.begin(), a.end(), v, [](const Approach& y, int id) { return y.veh < id; });
                    if (it != a.end() && it->veh == v) {
                        a.erase(it);
                    }
                    x.lf.erase(x.lf.begin() + j);
                }
            }
            break;
        }
        const int target = conts[bestLaneIt];
        int newLink = -1;
        for (int l : net.lanes[cur].links) {
            if (net.links[l].lane == target) {
                newLink = l;
                break;
            }
        }
        if (newLink == d.link) {
            break;
        }
        if (newLink < 0) {
            break;
        }
        moveApproach(v, d.link, newLink);
        d.link = newLink;
        cur = net.links[newLink].viaOrLane();
        ++i;
        if (!isInternal(cur)) {
            ++bestLaneIt;
        }
    }
}

}  // namespace ps
