// Stops at bus stops and parking areas (no waypoints, triggers or containers).
// Ported from SUMO 1.28: MSVehicle::processNextStop, keepStopping,
// resumeFromStopping, MSStop::getEndPos / getMinDuration and
// MSStoppingPlace::enter / leaveFrom / getLastFreePos / fits.
//
// Bus stop occupancy is shared state; it is changed through StopEvents that
// the move phase collects and Sim applies afterwards in vehicle order.
#include "model.hpp"

namespace ps {

constexpr double STOPPING_PLACE_OFFSET = 0.5;

// MSStop::getMinDuration
static Time minDuration(const Stop& s, Time time) {
    if (s.until >= 0) {
        return std::max(s.duration, s.until - time);
    }
    return s.duration;
}

void Sim::busStopComputeLastFreePos(int bs) {
    BusStop& s = busStops[bs];
    s.lastFreePos = s.endPos;
    s.lastParking = -1;
    for (const auto& item : s.endPositions) {
        if (s.lastFreePos >= item.second.second || s.lastFreePos == s.endPos) {
            s.lastFreePos = item.second.second;
            if (isStopped(item.first) && veh[item.first].stops.front().parking) {
                s.lastParking = item.first;
            }
        }
    }
}

void Sim::busStopEnter(int bs, int v, double pos, bool parking, double length) {
    BusStop& s = busStops[bs];
    const VType& t = vt(v);
    const double beg = pos + t.minGap * (parking ? s.parkingFactor : 1);
    const double end = beg - ((length >= 0 ? length : t.length) + t.minGap) * (parking ? s.parkingFactor : 1);
    bool found = false;
    for (auto& item : s.endPositions) {
        if (item.first == v) {
            item.second = {beg, end};
            found = true;
        }
    }
    if (!found) {
        s.endPositions.push_back({v, {beg, end}});
        // SUMO keeps a std::map keyed by vehicle pointer; keep vehicle order
        std::sort(s.endPositions.begin(), s.endPositions.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    }
    busStopComputeLastFreePos(bs);
}

void Sim::busStopLeave(int bs, int v) {
    BusStop& s = busStops[bs];
    s.endPositions.erase(std::remove_if(s.endPositions.begin(), s.endPositions.end(),
                                        [&](const auto& item) { return item.first == v; }), s.endPositions.end());
    busStopComputeLastFreePos(bs);
}

bool Sim::busStopFits(int bs, double pos, int v) const {
    const BusStop& s = busStops[bs];
    return pos + POSITION_EPS >= s.endPos || (pos - s.startPos >= vt(v).length * s.parkingFactor / 2);
}

// MSStop::getEndPos -> MSStoppingPlace::getLastFreePos
double Sim::stopEndPos(int v, const Stop& stop) const {
    if (stop.parkingArea >= 0) {
        const bool onStopEdge = veh[v].status != VehStatus::PENDING && curEdge(v) == net.lanes[stop.lane].edge;
        const double brakePos = onStopEdge ? kin[v].pos + cfBrakeGap(v, kin[v].speed, vt(v).decel, 0.) : 0;
        return paLastFreePos(paRead(stop.parkingArea), v, brakePos, true);
    }
    if (stop.busStop < 0) {
        return stop.endPos;
    }
    return busStopLastFreePos(stop.busStop, v);
}

// MSStoppingPlace::getLastFreePos for the vehicle (whose next stop decides the parking factor)
double Sim::busStopLastFreePos(int bs, int v) const {
    const BusStop& s = busStops[bs];
    if (s.endPositions.empty()) {
        return s.lastFreePos;
    }
    const VType& t = vt(v);
    const double vehGap = t.minGap;
    double pos = s.lastFreePos - vehGap - NUMERICAL_EPS;
    const Veh& x = veh[v];
    if (s.parkingFactor < 1 && s.lastParking >= 0 && !x.stops.empty() && !x.stops.front().parking
            && veh[s.lastParking].stops.front().duration < minDuration(x.stops.front(), now)) {
        // stop far back enough so that the previous parking vehicle can leave (even if this vehicle fits, it will
        // be a blocker because it stops on the road)
        pos = std::min(pos, kin[s.lastParking].pos - vt(s.lastParking).length - vehGap - NUMERICAL_EPS);
    }
    if (!busStopFits(bs, pos, v)) {
        const double vehLength = t.length * s.parkingFactor;
        std::vector<std::pair<double, std::pair<double, int>>> spaces;
        for (const auto& it : s.endPositions) {
            spaces.push_back({it.second.first, {it.second.second, it.first}});
        }
        std::sort(spaces.begin(), spaces.end());
        std::reverse(spaces.begin(), spaces.end());
        double prev = s.endPos;
        for (const auto& it : spaces) {
            const int other = it.second.second;
            const bool otherParking = isParking(other);
            const Time remaining = isStopped(other) ? veh[other].stops.front().duration : 0;
            if (prev - it.first + NUMERICAL_EPS >= vehLength && (otherParking || remaining > TIME2STEPS(10))) {
                return prev;
            }
            prev = it.second.first - vehGap;
        }
        if (s.parkingFactor < 1 && s.lastParking >= 0) {
            pos = std::min(pos, kin[s.lastParking].pos - vt(s.lastParking).length - vehGap - NUMERICAL_EPS);
        }
    }
    return pos;
}

bool Sim::keepStopping(int v, bool afterProcessing) const {
    if (!isStopped(v)) {
        return false;
    }
    const Stop& s = veh[v].stops.front();
    const Kin& k = kin[v];
    // a waypoint keeps "stopping" until its end is passed
    return s.duration - (afterProcessing ? DELTA_T : 0) > 0 || s.triggered || s.containerTriggered || s.joinTriggered
           || (s.speed > 0 && k.pos < std::min(s.endPos, net.lanes[s.lane].length - POSITION_EPS)
               && (!s.parking || k.speed >= HALTING_SPEED));
}

void Sim::resumeFromStopping(int v, std::vector<StopEvent>& ev) {
    Veh& x = veh[v];
    if (!isStopped(v)) {
        return;
    }
    if (x.registeredWaiting) {
        x.registeredWaiting = false;
        nVehWaiting--;
    }
    const Stop& stop = x.stops.front();
    if (stop.busStop >= 0) {
        ev.push_back({v, stop.busStop, false, 0});
    }
    if (stop.posLat != INVALID_POSLAT && latRes <= 0) {
        kin[v].posLat = 0;  // the default lateral position again
    }
    if (stop.parkingArea >= 0 && stop.speed <= 0) {
        // (on the area's lane: the task of that lane changes the area)
        paLeave(stop.parkingArea, v);
    }
    if (opt.stopOutput) {
        // MSStopOut::stopEnded
        x.ex().stopLog.push_back({stop.busStop, stop.lane, kin[v].pos, stop.parking, stop.started, now});
    }
    if (stop.jump >= 0) {
        // MSVehicleTransfer::add: the vehicle leaves the network after the move and reappears on the next route edge
        x.jumpPending = true;
        x.ex().jumpEdge = net.lanes[stop.lane].edge;
        x.ex().jumpProceed = std::max(now + stop.jump, stop.jumpUntil);
    }
    x.stops.erase(x.stops.begin());
    x.stopDist = DMAX;
    x.waitingTime = 0;
    x.stopSpeed = cfMaxNextSpeed(v, kin[v].speed);
    updateBestLanes(v, true);
    if (x.routingDevice) {
        routingStopEnded(v);  // MSDevice_Routing::notifyStopEnded
    }
}

double Sim::processNextStop(int v, double currentVelocity, std::vector<StopEvent>& ev) {
    Veh& x = veh[v];
    if (x.stops.empty()) {
        return currentVelocity;
    }
    Stop& stop = x.stops.front();
    const Kin& k = kin[v];
    if (stop.reached) {
        stop.duration -= actionStepLength(v);  // (called at the actions)
        if (stop.speed <= 0) {
            x.stopTime += TS;  // (waypoints: counted after the move, see executeMove)
        }
        if (k.speed > 0 && stop.busStop >= 0) {
            ev.push_back({v, stop.busStop, true, k.pos});
        }
        if (stop.duration <= 0 && stop.joinVeh >= 0) {
            // join this train part to another one (the execute phase runs serially); then it arrives here
            const int t = stop.joinVeh;
            if (veh[t].status != VehStatus::PENDING && (joinTrainPart(t, v) || joinTrainPartFront(t, v))) {
                stop.joinTriggered = false;
                if (x.registeredWaiting) {
                    x.registeredWaiting = false;
                    nVehWaiting--;
                }
                x.arrivalPos = k.pos;
                x.ex().arrivalIdx = x.routeIdx;
                transferAtSplitOrJoin(v, t);
            }
        }
        boardTransportables(v);
        const bool onRoad = x.status == VehStatus::RUNNING;
        if (!keepStopping(v, false) && onRoad) {
            resumeFromStopping(v, ev);
        } else {
            stopTriggerWaiting(v);
            // joining only takes place after the stop duration (or until the extension is over)
            if (stop.joinTriggered && !x.registeredWaiting && stop.duration <= (stop.extension >= 0 ? -stop.extension : 0)) {
                if (stop.extension >= 0) {
                    std::fprintf(stderr, "Warning: Vehicle '%s' aborts joining after extension of %.2fs at time %.2f.\n", x.id.c_str(),
                                 STEPS2TIME(stop.extension), STEPS2TIME(now));
                    stop.joinTriggered = false;
                } else {
                    x.registeredWaiting = true;  // keeps stopping, the simulation may end
                    nVehWaiting++;
                }
            }
            if (stop.speed > 0) {
                // waypoint mode: the stop speed when the 'until' time is over, otherwise as computed in planMove
                return stop.duration == 0 ? stop.speed : currentVelocity;
            }
            return BALLISTIC ? kin[v].speed - vt(v).decel : 0;  // brake
        }
    } else {
        if (stop.onDemand && !stop.skipOnDemand
                && x.stopDist <= cfBrakeGap(v, laneVehMaxSpeed(k.lane, v), vt(v).decel, vt(v).tau)) {
            // nobody leaves or enters: the stop is passed (MSStop::getSpeed: as a waypoint at any speed)
            bool noExits = true;
            for (int pi : x.exr().riders) {
                noExits = noExits && !rideCanLeave(pi, v, stop);
            }
            bool noEntries = true;
            auto wait = waiting4Vehicle.find(net.lanes[stop.lane].edge);
            if (wait != waiting4Vehicle.end()) {
                for (int pi : wait->second) {
                    const PStage& s = persons[pi].plan[persons[pi].step];
                    noEntries = noEntries && !(rideWaitingFor(s, v) && vehAllowsBoarding(v, pi)
                                               && vehStoppedInRange(v, s.waitingPos, opt.stopTolerance, true));
                }
            }
            if (noExits && noEntries) {
                stop.skipOnDemand = true;
                stop.speed = std::numeric_limits<double>::max();
                updateBestLanes(v, true);  // the best lanes extend past this stop
            }
        }
        if (stop.routeIdx != x.routeIdx) {
            return currentVelocity;
        }
        const bool useStoppingPlace = stop.busStop >= 0 || stop.parkingArea >= 0;
        bool fits = stop.busStop < 0 || busStopFits(stop.busStop, k.pos, v);
        if (stop.parkingArea >= 0) {
            // a free position in the parking area; vehicles on other lanes of the edge (which cannot reach
            // the stop) only check whether the area is full, in the step-begin state
            const ParkingArea& pa = parkingAreas[stop.parkingArea];
            const bool ownLane = k.lane == pa.lane;
            fits = fits && k.pos > pa.begPos;
            if ((ownLane ? pa.occupancy() : pa.snapOccupancy) >= pa.capacity) {
                fits = false;
                // trigger a potential parkingAreaReroute (the reroute happens after the move phase)
                if (hasRerouters()) {
                    for (int ri : laneRerouters[k.lane]) {
                        if (rerouters[ri].haveParkProbs) {
                            rerouterTrigger(v, ri, false, true);
                        }
                    }
                }
            } else if (ownLane && paOccupancyIncludingReservations(pa, v) >= pa.capacity) {
                fits = false;
            } else if (ownLane && pa.onRoad && paLotIndex(pa, v) < 0) {
                fits = false;
            }
        }
        const double targetPos = k.pos + x.stopDist + (stop.speed > 0 ? stop.startPos - stop.endPos : 0.);
        // MSStop::getReachedThreshold: the start of the stop
        double reachedThreshold = (useStoppingPlace ? targetPos - STOPPING_PLACE_OFFSET : stop.startPos) - NUMERICAL_EPS;
        if (stop.busStop >= 0 && stop.speed <= 0 && x.waitingTime > DELTA_T && k.lane == stop.lane) {
            reachedThreshold = std::min(reachedThreshold, stop.startPos + vt(v).length);
        }
        const bool posReached = k.pos >= reachedThreshold && currentVelocity <= stop.speed + HALTING_SPEED && k.lane == stop.lane;
        if (fits && posReached) {
            stop.reached = true;
            stop.started = now;
            if (!stop.tripId.empty()) {
                x.tripId = stop.tripId;
            }
            stop.duration = minDuration(stop, now);
            stop.endBoarding = stop.extension >= 0 ? now + stop.duration + stop.extension : TIME_MAX;
            if (stop.speed > 0) {
                // ignore the duration in waypoint mode unless 'until' is set
                stop.duration = stop.until > now ? stop.until - now : 0;
            }
            if (stop.busStop >= 0) {
                ev.push_back({v, stop.busStop, true, k.pos});
            }
            if (stop.parkingArea >= 0 && stop.speed <= 0) {
                paEnter(stop.parkingArea, v);
            }
            if (!stop.split.empty()) {
                // split the train (the execute phase runs serially): the other part is inserted in this step
                const int sv = stop.splitVeh;
                if (sv < 0 || veh[sv].status != VehStatus::PENDING) {
                    std::fprintf(stderr, "Warning: Vehicle '%s' to split from vehicle '%s' is not known. time=%.2f.\n",
                                 stop.split.c_str(), x.id.c_str(), STEPS2TIME(now));
                } else {
                    // (the stopping place already knows the train with its length before the split)
                    for (auto it = ev.rbegin(); it != ev.rend(); ++it) {
                        if (it->veh == v && it->enter) {
                            it->length = vt(v).length;
                            break;
                        }
                    }
                    std::vector<int>& waiting = triggeredWaiting[routes[veh[sv].route][veh[sv].routeIdx]];
                    waiting.erase(std::remove(waiting.begin(), waiting.end(), sv), waiting.end());
                    veh[sv].depart = now;
                    triggeredDue.push_back(sv);
                    VType& own = types[kin[v].type];
                    own.length = std::max(own.length - vt(sv).length, own.locomotiveLength);
                    transferAtSplitOrJoin(v, sv);
                    if (veh[sv].departPosMode == DP_SPLIT_FRONT) {
                        const double backShift = vt(sv).length + own.minGap;
                        kin[v].pos -= backShift;
                        kin[v].backPos -= backShift;
                    }
                }
            }
            boardTransportables(v);
            if (stop.posLat != INVALID_POSLAT) {
                kin[v].posLat = stop.posLat;  // the lateral position of the stop
            }
        }
    }
    return currentVelocity;
}

}  // namespace ps

namespace ps {

// MSVehicle::joinTrainPart: v joins the rear of the stopped train t
bool Sim::joinTrainPart(int t, int v) {
    const Kin& kt = kin[t];
    const int backLane = kt.nFurther == 0 ? kt.lane : kt.further[kt.nFurther - 1];
    const double gap = (kt.nFurther == 0 ? kt.pos - vt(t).length : kt.backPos) - kin[v].pos;
    Veh& xt = veh[t];
    if (isStopped(t) && xt.stops.front().duration <= DELTA_T && xt.stops.front().joinTriggered && backLane == kin[v].lane && gap >= 0
            && gap <= vt(t).minGap + 1) {
        types[kt.type].length += vt(v).length;
        xt.stops.front().joinTriggered = false;
        if (xt.registeredWaiting) {
            xt.registeredWaiting = false;
            nVehWaiting--;
        }
        return true;
    }
    return false;
}

// MSVehicle::joinTrainPartFront: the stopped train t is joined at its front by v (t moves to the position of v)
bool Sim::joinTrainPartFront(int t, int v) {
    const Kin& kv = kin[v];
    const int backLane = kv.nFurther == 0 ? kv.lane : kv.further[kv.nFurther - 1];
    const double gap = (kv.nFurther == 0 ? kv.pos - vt(v).length : kv.backPos) - kin[t].pos;
    Veh& xt = veh[t];
    if (!(isStopped(t) && xt.stops.front().duration <= DELTA_T && xt.stops.front().joinTriggered && backLane == kin[t].lane && gap >= 0
          && gap <= vt(t).minGap + 1)) {
        return false;
    }
    double skippedLaneLengths = 0;
    if (kv.nFurther > 0) {
        // t is moved to the lane of v: the further lanes of v must be on t's route
        skippedLaneLengths += net.lanes[kin[t].lane].length;
        const std::vector<int>& route = routes[xt.route];
        int routeIndex = xt.routeIdx + (isInternal(kin[t].lane) ? 1 : 0);
        for (int i = kv.nFurther - 1; i >= 0; i--) {
            const int edge = net.lanes[kv.further[i]].edge;
            if (net.edges[edge].internal) {
                continue;
            }
            if (routeIndex >= (int)route.size() || edge != route[routeIndex]) {
                std::fprintf(stderr, "Warning: Cannot join vehicle '%s' to vehicle '%s' due to incompatible routes. time=%.2f.\n",
                             veh[v].id.c_str(), xt.id.c_str(), STEPS2TIME(now));
                return false;
            }
            routeIndex++;
        }
        const int vEdge = net.lanes[kv.lane].edge;
        const int normalSucc = net.edges[vEdge].internal ? net.lanes[net.normalSuccessorLane(kv.lane)].edge : vEdge;
        if (routeIndex >= (int)route.size() || normalSucc != route[routeIndex]) {
            std::fprintf(stderr, "Warning: Cannot join vehicle '%s' to vehicle '%s' due to incompatible routes. time=%.2f.\n",
                         veh[v].id.c_str(), xt.id.c_str(), STEPS2TIME(now));
            return false;
        }
        for (int i = kv.nFurther - 2; i >= 0; i--) {
            skippedLaneLengths += net.lanes[kv.further[i]].length;
        }
    }
    types[kin[t].type].length += vt(v).length;
    // the lane is advanced as in a regular move
    kin[t].pos = skippedLaneLengths + kv.pos;
    xt.stops.front().joinTriggered = false;
    if (xt.registeredWaiting) {
        xt.registeredWaiting = false;
        nVehWaiting--;
    }
    return true;
}

}  // namespace ps
