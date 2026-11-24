// Teleporting and parking vehicles. Ported from SUMO 1.28 MSVehicleTransfer
// (add, checkInsertions), MSLane::freeInsertion, MSEdge::getCurrentTravelTime
// and MSVehicle::enterLaneAtMove (teleport case).
#include <cstring>

#include "model.hpp"

namespace ps {

constexpr double TELEPORT_MIN_SPEED = 1;

// MSEdge::getCurrentTravelTime (an edge is delayed once a vehicle entered it)
static double currentTravelTime(const Sim& s, int edge, double minSpeed) {
    const Edge& e = s.net.edges[edge];
    if (!s.edgeDelayed[edge]) {
        return e.length / std::max(s.net.edgeSpeedLimit(edge), NUMERICAL_EPS);
    }
    return e.length / std::max(minSpeed, s.edgeMeanSpeed(edge));
}

void Sim::removeFromNet(int v, int dwReason) {
    // MSVehicle::onRemovalFromNet / leaveLane: registrations at links are removed,
    // detectors see the vehicle leave
    removeApproaches(v);
    veh[v].waitingTime = 0;
    std::vector<DetEvent> ev;
    const Kin& k = kin[v];
    if (!loopsOf(v, k.lane).empty()) {
        loopNotifyLeave(v, k.lane, ev);
    }
    for (int i = 0; i < k.nFurther; ++i) {
        if (!loopsOf(v, k.further[i]).empty()) {
            loopNotifyLeave(v, k.further[i], ev);
        }
    }
    applyDetEvents(ev);
    veh[v].aliasLane = veh[v].aliasLoopLane = -1;
    veh[v].nNoRem = 0;
    if (!veh[v].exr().dwHeld.empty()) {
        dwLeaveAll(v, dwReason);  // notifyLeave (arrival, parking, teleport)
    }
}

void Sim::transferAdd(int v, bool parking) {
    Veh& x = veh[v];
    Kin& k = kin[v];
    if (parking) {
        x.parkingSignal = parkingSignalAtStop(v);
    }
    const int oldLane = k.lane;
    removeFromNet(v, parking ? DWR_PARKING : DWR_TELEPORT);
    x.lf.clear();
    endLaneChangeManeuver(v);
    if (parking) {
        x.status = VehStatus::PARKING;
    } else {
        if (!x.jumping) {
            nTeleported++;
            x.nTeleports++;
        }
        if (routeEdge(v, 1) < 0) {
            // teleports beyond the arrival edge (MSDevice_Tripinfo::notifyLeave, NOTIFICATION_TELEPORT_ARRIVED)
            x.routeLength += x.arrivalPos;
            x.status = VehStatus::ARRIVED;
            x.arrivalTime = STEPS2TIME(now);
            writeTripinfo(v);
            ridersVehicleArrived(v);
            releaseVehicle(v);
            nArrived++;
            return;
        }
        x.status = VehStatus::TELEPORTING;
        x.routeLength += net.lanes[oldLane].length;  // MSDevice_Tripinfo::notifyLeave (NOTIFICATION_TELEPORT)
        // enterLaneAtMove(succEdge(1)->getLanes()[0], true)
        x.routeIdx++;
        k.lane = net.edges[curEdge(v)].lanes[0];
        k.pos = 0;
        k.nFurther = 0;
        x.lastBestLanesEdge = -1;
        x.lastBestLanesInternalLane = -1;
        if (edFile != nullptr) {
            edEntered[k.lane]++;  // NOTIFICATION_TELEPORT
        }
        x.teleportLoopLane = k.lane;
        loopNotifyEnterNow(v, k.lane);
        if (railActive) {
            trackerEnter(v, k.lane);  // enterLaneAtMove(teleporting): activateReminders(NOTIFICATION_TELEPORT)
        }
        if (hasRerouters()) {
            if (VehExtra* e = x.exq()) {
                e->rrActive.clear();
            }
            rerouterEnter(v, k.lane, false);  // (rerouters also act on teleporting vehicles)
        }
    }
    x.transferTime = now;
    x.proceedTime = x.jumping ? x.exr().jumpProceed : -1;
    transfer.push_back(v);
}

// MSEdgeControl::detectCollisions -> MSLane::detectCollisions with the defaults
// collision.action teleport, collision.mingap-factor -1 (Krauss: 1) and no
// junction collision check: a follower closer to its leader than its minGap is
// teleported. Lanes are checked in parallel, the teleports applied in lane order.
void Sim::detectCollisions(const char* stage) {
    if (opt.collisionAction == "none") {
        return;
    }
    // lanes with vehicles; with --sumo-order in the order of the active-lane list
    // (MSEdgeControl::myActiveLanes, then myInactiveCheckCollisions)
    std::vector<int>& check = colCheck;
    collectOccupiedLanes(check);
    std::vector<int> passedJC;  // (MSEdgeControl::myInactiveCheckCollisions: after the active lanes)
    if (opt.checkJunctions && std::strcmp(stage, "move") == 0) {
        for (int v : running) {
            const int l = veh[v].jcPassed;
            if (l >= 0 && lanes[l].vehs.empty() && std::find(passedJC.begin(), passedJC.end(), l) == passedJC.end()) {
                passedJC.push_back(l);
            }
        }
    }
    if (opt.sumoOrder) {
        for (int l : check) {
            laneActive[l] = 1;
        }
        std::vector<int> ordered;
        for (int l : activeList) {
            if (laneActive[l] == 1) {
                laneActive[l] = 2;
                ordered.push_back(l);
            }
        }
        for (int l : check) {
            if (laneActive[l] == 1) {
                ordered.push_back(l);
            }
        }
        check.swap(ordered);
        for (int l : check) {
            laneActive[l] = 0;
        }
    }
    check.insert(check.end(), passedJC.begin(), passedJC.end());
    // per-lane results (persistent buffers: this runs three times per step)
    std::vector<std::vector<int>>& found = colFound;
    std::vector<std::vector<int>>& victims = colVictims;
    std::vector<std::vector<std::string>>& rows = colRows;
    if (found.size() < check.size()) {
        found.resize(check.size());
        victims.resize(check.size());
        rows.resize(check.size());
    }
    pool.setTag(ThreadPool::T_COLLISION);
    pool.parallel_for((int)check.size(), [&](int i) {
        const int l = check[i];
        const std::vector<int>& vs = lanes[l].vehs;
        std::vector<int>& f = found[i];
        std::vector<int>& vic = victims[i];
        f.clear();
        vic.clear();
        rows[i].clear();
        if (opt.checkJunctions && mustCheckJunctionCollisions(l)) {
            // collisions with the vehicles on the foe lanes inside the junction (gap -1)
            std::vector<std::pair<int, int>> jc;
            junctionCollisions(l, jc);
            for (const auto& cv : jc) {
                f.push_back(cv.first);
                vic.push_back(cv.second);
                char buf[256];
                std::snprintf(buf, sizeof(buf), "%.0f,%s,%s,%s,%s,%.2f\n", STEPS2TIME(now), stage, net.lanes[l].id.c_str(),
                              veh[cv.first].id.c_str(), veh[cv.second].id.c_str(), -1.0);
                rows[i].push_back(buf);
            }
        }
        const size_t nJunction = f.size();
        if (vs.empty()) {
            // (MSLane::detectCollisions returns before removing the colliders of a lane without vehicles of its own)
            f.clear();
            return;
        }
        // MSLane::detectCollisionBetween: minGap scaled by --collision.mingap-factor or the model's factor
        auto collisionGap = [&](int collider, int victim) {
            const double factor = opt.collisionMinGapFactor >= 0 ? opt.collisionMinGapFactor : vt(collider).collisionMinGapFactor;
            return backPositionOnLane(victim, l) - kin[collider].pos - factor * vt(collider).minGap;
        };
        auto collides = [&](int collider, int victim) {
            // (no self-collisions: a long train may loop back onto its lane)
            return collider != victim && collisionGap(collider, victim) < -NUMERICAL_EPS;
        };
        if (sublane) {
            // MSLane::detectCollisions in sublane mode: all pairs of vehicles touching the lane (several may be next
            // to each other), MSLane::detectCollisionBetween with the lateral gap
            auto sublaneGap = [&](int collider, int victim) {
                const double factor = opt.collisionMinGapFactor >= 0 ? opt.collisionMinGapFactor : vt(collider).collisionMinGapFactor;
                double victimBack = backPositionOnLane(victim, l);
                if (lateralOverlap(victim, kin[victim].posLat, kin[victim].lane) > 0
                        || lateralOverlap(collider, kin[collider].posLat, kin[collider].lane) > 0) {
                    const int cl = kin[collider].lane;
                    if (net.lanes[cl].edge == net.lanes[l].edge && net.lanes[cl].length > net.lanes[l].length) {
                        victimBack *= net.lanes[cl].length / net.lanes[l].length;  // the victim's position on the longer lane
                    }
                }
                return victimBack - positionOnLane(collider, l) - factor * vt(collider).minGap;
            };
            auto collidesSublane = [&](int collider, int victim) {
                const double gap = sublaneGap(collider, victim);
                if (gap >= -NUMERICAL_EPS) {
                    return false;
                }
                const double latGap = std::fabs(centerOnEdge(victim, l) - centerOnEdge(collider, l))
                                      - 0.5 * std::fabs(vt(victim).width + vt(collider).width);
                if (latGap + NUMERICAL_EPS > 0) {
                    return false;
                }
                const Edge& e = net.edges[net.lanes[l].edge];
                if (e.internal && e.lanes.size() > 1 && kin[victim].lane != kin[collider].lane) {
                    // the gap is ambiguous on internal lanes of different lengths
                    double gapDelta = 0;
                    const int other = kin[collider].lane == l ? victim : collider;
                    if (kin[other].shadowLane == l) {
                        gapDelta = net.lanes[l].length - net.lanes[kin[other].lane].length;
                    } else {
                        for (int j = 0; j < kin[other].nFurther; ++j) {
                            if (net.lanes[kin[other].further[j]].edge == net.lanes[l].edge) {
                                gapDelta = net.lanes[l].length - net.lanes[kin[other].further[j]].length;
                                break;
                            }
                        }
                    }
                    if (gap + gapDelta >= 0) {
                        return false;
                    }
                }
                if (opt.laneChangeDuration > DELTA_T && veh[collider].lc.isChangingLanes() && veh[victim].lc.isChangingLanes()
                        && kin[victim].lane != l) {
                    return false;  // a synchronous lane change maneuver
                }
                return true;
            };
            std::vector<int> any;
            anyVehicles(l, [&](int u) {
                any.push_back(u);
                return true;
            });
            for (int follow : any) {
                for (int lead : any) {
                    if (lead == follow || positionOnLane(lead, l) < positionOnLane(follow, l)) {
                        continue;
                    }
                    if (collidesSublane(follow, lead)) {
                        f.push_back(follow);
                        vic.push_back(lead);
                        break;
                    }
                }
            }
            for (size_t j = nJunction; j < f.size(); ++j) {
                char buf[256];
                std::snprintf(buf, sizeof(buf), "%.0f,%s,%s,%s,%s,%.2f\n", STEPS2TIME(now), stage, net.lanes[l].id.c_str(),
                              veh[f[j]].id.c_str(), veh[vic[j]].id.c_str(), sublaneGap(f[j], vic[j]));
                rows[i].push_back(buf);
            }
            std::sort(f.begin(), f.end());
            f.erase(std::unique(f.begin(), f.end()), f.end());
            return;
        }
        for (size_t j = vs.size() - 1; j > 0; --j) {
            if (collides(vs[j - 1], vs[j])) {
                f.push_back(vs[j - 1]);
                vic.push_back(vs[j]);
            }
        }
        // (SUMO compares the first partial occupant with the most upstream vehicle)
        if (!lanes[l].partials.empty() && collides(vs.front(), lanes[l].partials.front())) {
            f.push_back(vs.front());
            vic.push_back(lanes[l].partials.front());
        }
        const size_t nRegular = f.size();
        const int bidiLane = net.lanes[l].bidi;
        if (bidiLane >= 0 && laneVehicleNumberWithPartials(bidiLane) > 0) {
            // bidirectional railway: trains on the opposite track that overlap (the faster one is at fault)
            const bool moveStage = std::strcmp(stage, "move") == 0;
            const double len = net.lanes[l].length;
            for (int v1 : vs) {
                const double high = positionOnLane(v1, l);
                double low = backPositionOnLane(v1, l);
                if (moveStage) {
                    low -= SPEED2DIST(kin[v1].speed);  // catches trains that "jump" through each other
                }
                anyVehicles(bidiLane, [&](int v2) {
                    if (v1 == v2 && !isRailVeh(v1)) {
                        return true;  // (a long train may loop back onto itself)
                    }
                    if (kin[v1].lane == kin[v2].lane || kin[v1].lane == backLane(v2) || backLane(v1) == kin[v2].lane) {
                        return true;  // not in a bidi relation
                    }
                    const double low2 = len - positionOnLane(v2, bidiLane);
                    double high2 = len - backPositionOnLane(v2, bidiLane);
                    if (moveStage) {
                        high2 += SPEED2DIST(kin[v2].speed);
                    }
                    if (!(high < low2 || high2 < low)) {
                        const bool swap = kin[v1].speed < kin[v2].speed;
                        f.push_back(swap ? v2 : v1);
                        vic.push_back(swap ? v1 : v2);
                    }
                    return true;
                });
            }
        }
        for (size_t j = nJunction; j < f.size(); ++j) {
            char buf[256];
            std::snprintf(buf, sizeof(buf), "%.0f,%s,%s,%s,%s,%.2f\n", STEPS2TIME(now), stage, net.lanes[l].id.c_str(),
                          veh[f[j]].id.c_str(), veh[vic[j]].id.c_str(),
                          j < nRegular ? collisionGap(f[j], vic[j]) : -1.0);
            rows[i].push_back(buf);
        }
        // std::set<const MSVehicle*, ComparatorNumericalIdLess>
        std::sort(f.begin(), f.end());
        f.erase(std::unique(f.begin(), f.end()), f.end());
    }, 64);
    bool any = false;
    for (size_t i = 0; i < check.size(); ++i) {
        collisionRows.insert(collisionRows.end(), rows[i].begin(), rows[i].end());
        if (opt.collisionAction == "warn" || lanes[check[i]].vehs.empty()) {
            nCollisions += (int)rows[i].size();
            continue;
        }
        for (int v : found[i]) {
            if (veh[v].status != VehStatus::RUNNING) {
                continue;
            }
            LaneRT& L = lanes[kin[v].lane];
            L.vehs.erase(std::find(L.vehs.begin(), L.vehs.end(), v));
            L.brutto -= vt(v).lengthWithGap();
            L.netto -= vt(v).length;
            nCollisions++;
            transferAdd(v, false);
            any = true;
        }
    }
    if (any) {
        compactRunning();
        rebuildPartials();
    }
}

// MSLane::freeInsertion (teleporting vehicles end fully on the lane, departing ones may start at 0)
bool Sim::freeInsertion(int v, int lane, double mspeed, bool departure) {
    Veh& x = veh[v];
    const VType& t = vt(v);
    double maxPos = net.lanes[lane].length;
    if (!x.stops.empty() && x.stops.front().routeIdx == x.routeIdx) {
        maxPos = std::max(0.0, stopEndPos(v, x.stops.front()));
    }
    const double minPos = departure ? 0 : std::min(maxPos, t.length);
    kin[v].lane = lane;
    kin[v].pos = minPos;
    const LaneRT& L = lanes[lane];
    if (L.vehs.empty()) {
        const double backOffset = minPos - t.length;
        const double mrg = missingRearGap(lane, v, backOffset, mspeed);
        if (mrg > 0) {
            if (minPos + mrg <= maxPos) {
                return isInsertionSuccess(v, lane, mspeed, minPos + mrg, true, false);
            }
            return false;
        }
        return isInsertionSuccess(v, lane, mspeed, minPos, true, false);
    }
    {
        const int leader = L.vehs.back();
        const double leaderPos = backPositionOnLane(leader, lane);
        const double speed = kin[leader].speed;
        const double frontGapNeeded = cfSecureGap(v, leader, speed, kin[leader].speed, vt(leader).decel) + t.minGap;
        if (leaderPos >= frontGapNeeded) {
            const double tspeed = std::min(cfInsertionFollowSpeed(v, mspeed, frontGapNeeded, kin[leader].speed, vt(leader).decel, leader), mspeed);
            if (isInsertionSuccess(v, lane, tspeed, minPos, true, false)) {
                return true;
            }
        }
    }
    for (size_t i = 0; i < L.vehs.size(); ++i) {
        int leader = i + 1 < L.vehs.size() ? L.vehs[i + 1] : -1;
        if (leader < 0 && !L.partials.empty()) {
            leader = L.partials.front();
        }
        const int follower = L.vehs[i];
        double speed = mspeed;
        if (leader >= 0) {
            speed = std::min(kin[leader].speed, mspeed);
        }
        double frontMax = maxPos;
        if (leader >= 0) {
            const double leaderRearPos = backPositionOnLane(leader, lane);
            const double frontGapNeeded = cfSecureGap(v, leader, speed, kin[leader].speed, vt(leader).decel) + t.minGap;
            frontMax = std::min(maxPos, leaderRearPos - frontGapNeeded);
        }
        const double followPos = kin[follower].pos + vt(follower).minGap;
        const double backGapNeeded = cfSecureGap(follower, v, kin[follower].speed, kin[v].speed, t.decel);
        const double backMin = followPos + backGapNeeded + t.length;
        if (frontMax > minPos && backMin + POSITION_EPS < frontMax) {
            if (isInsertionSuccess(v, lane, speed, backMin + POSITION_EPS, true, false)) {
                return true;
            }
        }
    }
    return false;
}

void Sim::checkTransfers() {
    std::sort(transfer.begin(), transfer.end());
    std::vector<int> keep;
    std::vector<StopEvent> stopEv;
    for (int v : transfer) {
        Veh& x = veh[v];
        if (x.status == VehStatus::PARKING) {
            Kin& k = kin[v];
            if (now != x.transferTime) {
                processNextStop(v, 1, stopEv);
                // MSVehicle::updateParkingState
                k.accel = SPEED2ACCEL(0 - k.speed);
                x.prevSpeed = k.speed;
                k.speed = 0;
            }
            if (keepStopping(v, true)) {
                keep.push_back(v);
                continue;
            }
            const int lane = x.stops.front().lane;
            const int area = x.stops.front().parkingArea;
            const double departPos = area >= 0 ? paInsertionPosition(parkingAreas[area], v) : k.pos;
            if (v == traceVeh) {
                std::fprintf(stderr, "%.2f leave parking %s departPos=%.2f\n", STEPS2TIME(now), x.id.c_str(), departPos);
            }
            insertionPosLat = k.posLat;  // (the lateral position is kept)
            const bool inserted = isInsertionSuccess(v, lane, 0, departPos, false, true);
            insertionPosLat = 0;
            if (inserted) {
                running.push_back(v);
                if (edFile != nullptr) {
                    // MSMeanData_Net: entering from a parking stop counts as entered
                    edEntered[lane]++;
                }
            } else {
                // MSVehicle::workOnIdleReminders: blocked from entering the road, the vehicle waits (idling)
                if (k.accel <= 0.5 * vt(v).accel) {
                    x.waitingTime += DELTA_T;
                } else {
                    x.waitingTime = 0;
                }
                if (trackAccumulatedWaiting) {
                    passWaitingTime(x, k.accel <= 0.5 * vt(v).accel);
                }
                x.tripWaiting += TS;
                if (!x.tripAmWaiting) {
                    x.waitingCount++;
                    x.tripAmWaiting = true;
                }
                if (!x.parkingSignal) {
                    // signal the wish to re-enter the road; the parking area lets other vehicles help
                    x.parkingSignal = true;
                    if (area >= 0) {
                        paComputeLastFreePos(parkingAreas[area]);  // MSParkingArea::notifyEgressBlocked
                    }
                }
                keep.push_back(v);
            }
            continue;
        }
        // teleporting (or jumping: waits for its proceed time, then tries the insertion every step)
        if (x.jumping && x.proceedTime > now) {
            keep.push_back(v);
            continue;
        }
        const int e = curEdge(v);
        const int nextEdge = routeEdge(v, 1);
        int l;
        if (nextEdge >= 0) {
            l = freeLane(e, net.allowedLanes(e, nextEdge, x.vclass), x.vclass, 0);
        } else {
            l = freeLane(e, nullptr, x.vclass, 0);
        }
        // (no attempt while the bidirectional lane is occupied)
        const bool busyBidi = l >= 0 && net.lanes[l].bidi >= 0 && laneVehicleNumberWithPartials(net.lanes[l].bidi) > 0;
        if (l >= 0 && !busyBidi && freeInsertion(v, l, std::min(net.lanes[l].speed, vehMaxSpeed(v)))) {
            running.push_back(v);
            x.jumping = false;
            if (l != x.teleportLoopLane) {
                x.aliasLane = l;
                x.aliasLoopLane = x.teleportLoopLane;
            }
            x.teleportLoopLane = -1;
            continue;
        }
        if (x.jumping) {
            keep.push_back(v);  // try again later
            continue;
        }
        if (x.proceedTime < 0) {
            x.proceedTime = now + TIME2STEPS(currentTravelTime(*this, e, TELEPORT_MIN_SPEED));
        } else if (x.proceedTime < now) {
            if (!laneLoops[x.teleportLoopLane].empty()) {
                // leaveLane(NOTIFICATION_TELEPORT_CONTINUATION / NOTIFICATION_TELEPORT_ARRIVED)
                std::vector<DetEvent> ev;
                loopNotifyLeave(v, x.teleportLoopLane, ev);
                applyDetEvents(ev);
            }
            if (nextEdge < 0) {
                x.routeLength += x.arrivalPos;  // (NOTIFICATION_TELEPORT_ARRIVED)
                x.status = VehStatus::ARRIVED;
                x.arrivalTime = STEPS2TIME(now);
                writeTripinfo(v);
                ridersVehicleArrived(v);
                releaseVehicle(v);
                nArrived++;
                continue;
            }
            x.routeLength += net.lanes[kin[v].lane].length;  // (NOTIFICATION_TELEPORT_CONTINUATION)
            const std::vector<int> allowed = net.allowedLanesOf(nextEdge, x.vclass);
            x.routeIdx++;
            kin[v].lane = allowed.empty() ? net.edges[nextEdge].lanes[0] : allowed[0];
            kin[v].pos = 0;
            x.lastBestLanesEdge = -1;
            x.proceedTime = now + TIME2STEPS(currentTravelTime(*this, e, TELEPORT_MIN_SPEED));
            if (edFile != nullptr) {
                // MSVehicle::enterLaneAtMove(onTeleporting): reminders see NOTIFICATION_TELEPORT
                edEntered[kin[v].lane]++;
            }
            x.teleportLoopLane = kin[v].lane;
            loopNotifyEnterNow(v, kin[v].lane);
            if (railActive) {
                trackerEnter(v, kin[v].lane);
            }
            if (hasRerouters()) {
                if (VehExtra* e = x.exq()) {
                    e->rrActive.clear();
                }
                rerouterEnter(v, kin[v].lane, false);
            }
        }
        keep.push_back(v);
    }
    transfer.swap(keep);
    for (const StopEvent& ev : stopEv) {
        if (ev.enter) {
            busStopEnter(ev.busStop, ev.veh, ev.pos, veh[ev.veh].stops.empty() ? false : veh[ev.veh].stops.front().parking, ev.length);
        } else {
            busStopLeave(ev.busStop, ev.veh);
        }
    }
}

}  // namespace ps
