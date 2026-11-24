// Persons riding vehicles, ported from SUMO 1.28: MSStageDriving (the ride stage), MSTransportableControl
// (persons waiting for a vehicle, loadAnyWaiting), MSVehicle::boardTransportables and the stop triggers,
// MSDevice_Transportable (riders follow the vehicle and alight at their stop).
//
// Concurrency: boarding happens in processNextStop (move planning). Vehicles that may board this step (stop on
// their lane, persons waiting on its edge) are planned after the parallel loop, serially in lane order
// (planMovements). Alighting is decided in executeMove (vehicle state only); the persons' next stages start
// after the move phase, in lane order (applyAlightings).
#include <algorithm>
#include <cmath>
#include <cstdio>

#include "model.hpp"

namespace ps {

// MSStageDriving::isWaitingFor
bool Sim::rideWaitingFor(const PStage& s, int v) const {
    const Veh& x = veh[v];
    auto has = [&](const std::string& l) { return std::find(s.lines.begin(), s.lines.end(), l) != s.lines.end(); };
    if (has(x.id)) {
        return true;
    }
    if (!(has(x.exr().line) && !x.exr().line.empty()) && !has("ANY")) {
        return false;
    }
    // even if the line matches the vehicle has to stop at the destination
    if (s.destinationStop >= 0) {
        for (const Stop& st : x.stops) {
            if (st.busStop == s.destinationStop) {
                return true;
            }
        }
        return false;
    }
    for (const Stop& st : x.stops) {
        if (net.lanes[st.lane].edge == s.destination) {
            return true;
        }
    }
    return routes[x.route].back() == s.destination;  // MSBaseVehicle::stopsAtEdge
}

// MSBaseVehicle::allowsBoarding
bool Sim::vehAllowsBoarding(int v, int pi) const {
    const Veh& x = veh[v];
    if (persons[pi].container ? (int)x.exr().cargo.size() >= vt(v).containerCapacity
                              : (int)x.exr().riders.size() >= vt(v).personCapacity) {
        return false;
    }
    if (isStopped(v) && !x.stops.front().permitted.empty()) {
        const std::vector<std::string>& perm = x.stops.front().permitted;
        return std::find(perm.begin(), perm.end(), persons[pi].id) != perm.end();
    }
    return true;
}

// MSBaseVehicle::isStoppedInRange
bool Sim::vehStoppedInRange(int v, double pos, double tolerance, bool checkFuture) const {
    const Veh& x = veh[v];
    if (isStopped(v) || (checkFuture && !x.stops.empty())) {
        const Stop& stop = x.stops.front();
        return stop.startPos - tolerance <= pos && stop.endPos + tolerance >= pos;
    }
    return false;
}

// MSBaseVehicle::getOdometer
double Sim::vehOdometer(int v) const {
    const Veh& x = veh[v];
    return x.status == VehStatus::ARRIVED ? x.routeLength : x.routeLength + kin[v].pos;
}

// MSStageDriving::setVehicle
void Sim::rideSetVehicle(int pi, int v) {
    PStage& s = persons[pi].plan[persons[pi].step];
    s.vehicle = v;
    if (v >= 0) {
        s.vehicleId = veh[v].id;
        if (veh[v].status != VehStatus::PENDING) {
            s.vehicleDistance = vehOdometer(v);
            s.rideTimeLoss = veh[v].timeLoss;
        } else {
            // it probably got triggered by the person
            s.vehicleDistance = 0;
            s.rideTimeLoss = 0;
        }
    }
}

// MSBaseVehicle::addTransportable / MSVehicle::addTransportable: the awaited persons of a reached stop
void Sim::vehAddPerson(int v, int pi) {
    Veh& x = veh[v];
    const bool container = persons[pi].container;
    std::vector<int>& list = container ? x.ex().cargo : x.ex().riders;
    if (list.empty()) {
        // (MSDevice_Transportable: created with the stop state of the vehicle)
        (container ? x.ex().cargoStopped : x.ex().ridersStopped) = isStopped(v);
    }
    list.push_back(pi);
    if (!x.stops.empty() && x.stops.front().reached) {
        Stop& stop = x.stops.front();
        if (!container && stop.triggered && stop.numExpectedPerson > 0) {
            stop.numExpectedPerson -= (int)std::count(stop.awaitedPersons.begin(), stop.awaitedPersons.end(), persons[pi].id);
        }
        if (container && stop.containerTriggered && stop.numExpectedContainer > 0) {
            stop.numExpectedContainer -= (int)std::count(stop.awaitedContainers.begin(), stop.awaitedContainers.end(), persons[pi].id);
        }
    }
}

// MSTransportableControl::addWaiting + MSEdge::addTransportable
void Sim::rideRegisterWaiting(int pi) {
    PStage& s = persons[pi].plan[persons[pi].step];
    (persons[pi].container ? waiting4Container : waiting4Vehicle)[s.waitingEdge].push_back(pi);
    nWaitingForVehicle++;
    haveNewWaiting = true;
    if (!persons[pi].container) {
        edgeAddPerson(s.waitingEdge, pi);
    }
}

// MSStageDriving::proceed (without taxis and access stages)
void Sim::rideProceed(int pi, int priorIdx) {
    Person& p = persons[pi];
    PStage& s = p.plan[p.step];
    const PStage& prior = p.plan[priorIdx];
    s.originStop = prior.kind == PStage::TRIP ? prior.originStop : prior.destinationStop;
    s.waitingSince = now;
    if (p.triggered && p.step == 1) {
        // the first real stage of a triggered person: it starts in the vehicle
        int v = -1;
        for (int u = 0; u < (int)veh.size() && v < 0; ++u) {
            if (veh[u].id == s.lines.front()) {
                v = u;
            }
        }
        if (v < 0) {
            throw std::runtime_error("Vehicle '" + s.lines.front() + "' not found for triggered departure of person '" + p.id + "'.");
        }
        if ((int)veh[v].exr().riders.size() >= vt(v).personCapacity) {
            std::fprintf(stderr, "Warning: Vehicle '%s' exceeds personCapacity %d when placing triggered person '%s', time=%.2f\n",
                         veh[v].id.c_str(), vt(v).personCapacity, p.id.c_str(), STEPS2TIME(now));
        }
        s.departed = now;
        rideSetVehicle(pi, v);
        if (s.originStop >= 0) {
            stopRemovePerson(s.originStop, pi);
        }
        s.waitingEdge = stageEdge(prior);
        s.waitingPos = stageEdgePos(pi, priorIdx);
        vehAddPerson(v, pi);
        return;
    }
    if (s.originStop >= 0) {
        s.waitingEdge = net.lanes[busStops[s.originStop].lane].edge;
        s.waitingPos = stopWaitingPosOnLane(s.originStop, pi);
    } else {
        s.waitingEdge = stageEdge(prior);
        s.waitingPos = stageEdgePos(pi, priorIdx);
    }
    if (s.origin >= 0 && s.origin != s.waitingEdge && (s.originStop < 0 || stopAccessPos(s.originStop, s.origin) < 0)) {
        // transfer at a junction (rather than an access)
        s.waitingEdge = s.origin;
        s.waitingPos = 0;
    }
    // MSEdge::getWaitingVehicle: a vehicle waiting for its triggered departure
    auto tw = triggeredWaiting.find(s.waitingEdge);
    if (tw != triggeredWaiting.end()) {
        for (int v : tw->second) {
            if (veh[v].status == VehStatus::PENDING && !veh[v].departSplit && rideWaitingFor(s, v)) {
                rideSetVehicle(pi, v);
                if (s.originStop >= 0) {
                    stopRemovePerson(s.originStop, pi);
                }
                vehAddPerson(v, pi);
                // MSInsertionControl::add, MSVehicleControl::handleTriggeredDepart
                tw->second.erase(std::find(tw->second.begin(), tw->second.end(), v));
                triggerDeparture(v);
                return;
            }
        }
    }
    rideRegisterWaiting(pi);
}

// a vehicle with a triggered departure becomes due (it is inserted in this step)
void Sim::triggerDeparture(int v) {
    veh[v].depart = now;
    triggeredDue.push_back(v);
}

// MSTransportableControl::loadAnyWaiting
bool Sim::loadAnyWaiting(int edge, int v, Time& timeToLoadNext, Time& stopDuration, bool containers) {
    std::map<int, std::vector<int>>& waiting = containers ? waiting4Container : waiting4Vehicle;
    auto wait = waiting.find(edge);
    if (wait == waiting.end()) {
        return false;
    }
    bool ret = false;
    std::vector<int>& ps = wait->second;
    for (size_t i = 0; i < ps.size();) {
        const int pi = ps[i];
        const PStage& s = persons[pi].plan[persons[pi].step];
        if (rideWaitingFor(s, v) && vehAllowsBoarding(v, pi) && timeToLoadNext - DELTA_T <= now
                && vehStoppedInRange(v, s.waitingPos, opt.stopTolerance, false)) {
            edgeRemovePerson(pi);
            vehAddPerson(v, pi);
            // the time at which the next person may board
            const Time loadingDuration = (Time)((double)(containers ? vt(v).loadingDuration : vt(v).boardingDuration)
                                                * types[persons[pi].type].boardingFactor);
            if (timeToLoadNext > now - DELTA_T) {
                timeToLoadNext += loadingDuration;
            } else {
                timeToLoadNext = now + loadingDuration;
            }
            rideSetVehicle(pi, v);
            if (s.originStop >= 0) {
                stopRemovePerson(s.originStop, pi);
            }
            ps.erase(ps.begin() + (long)i);
            nWaitingForVehicle--;
            ret = true;
        } else {
            ++i;
        }
    }
    if (ps.empty()) {
        waiting.erase(wait);
    }
    if (ret) {
        stopDuration = std::max(stopDuration, timeToLoadNext - now);
    }
    return ret;
}

// MSVehicle::boardTransportables (persons only)
void Sim::boardTransportables(int v) {
    Veh& x = veh[v];
    Stop& stop = x.stops.front();
    if (stop.skipOnDemand) {
        return;
    }
    const bool boarded = now <= stop.endBoarding && !waiting4Vehicle.empty()
                         && loadAnyWaiting(net.lanes[stop.lane].edge, v, stop.timeToBoardNextPerson, stop.duration, false)
                         && stop.numExpectedPerson == 0;
    const bool loaded = now <= stop.endBoarding && !waiting4Container.empty()
                        && loadAnyWaiting(net.lanes[stop.lane].edge, v, stop.timeToLoadNextContainer, stop.duration, true)
                        && stop.numExpectedContainer == 0;
    if (now > stop.endBoarding) {
        stop.triggered = false;
        stop.containerTriggered = false;
        if (x.registeredWaiting) {
            x.registeredWaiting = false;
            nVehWaiting--;
        }
    }
    if (boarded) {
        // the triggering condition is fulfilled
        if (x.registeredWaiting) {
            x.registeredWaiting = false;
            nVehWaiting--;
        }
        stop.triggered = false;
    }
    if (loaded) {
        if (x.registeredWaiting) {
            x.registeredWaiting = false;
            nVehWaiting--;
        }
        stop.containerTriggered = false;
    }
}

// MSVehicle::processNextStop, the triggered stop after boarding: capacity, registration as waiting (one step later)
void Sim::stopTriggerWaiting(int v) {
    Veh& x = veh[v];
    Stop& stop = x.stops.front();
    if (stop.triggered) {
        if (vt(v).personCapacity == (int)x.exr().riders.size()) {
            std::fprintf(stderr, "Warning: Vehicle '%s' ignores triggered stop on lane '%s' due to capacity constraints.\n",
                         x.id.c_str(), net.lanes[stop.lane].id.c_str());
            stop.triggered = false;
        } else if (!x.registeredWaiting && stop.duration <= DELTA_T) {
            // registered after waiting for one step (otherwise a deadlock might be signalled falsely)
            x.registeredWaiting = true;
            nVehWaiting++;
        }
    }
    if (stop.containerTriggered) {
        if (vt(v).containerCapacity == (int)x.exr().cargo.size()) {
            std::fprintf(stderr, "Warning: Vehicle '%s' ignores container triggered stop on lane '%s' due to capacity constraints.\n",
                         x.id.c_str(), net.lanes[stop.lane].id.c_str());
            stop.containerTriggered = false;
        } else if (!x.registeredWaiting && stop.duration <= DELTA_T) {
            x.registeredWaiting = true;
            nVehWaiting++;
        }
    }
}

// MSDevice_Transportable::notifyMove (executeMove of the vehicle): the riders leave at their stop; their next
// stages start after the move phase (alight)
void Sim::ridersNotifyMove(int v, int laneVehicles, std::vector<Alighting>& alight) {
    // the person device, then the container device
    for (int kind = 0; kind < 2; ++kind) {
        VehExtra& e = veh[v].ex();
        std::vector<int>& riders = kind == 0 ? e.riders : e.cargo;
        bool& stopped = kind == 0 ? e.ridersStopped : e.cargoStopped;
        if (riders.empty() && !stopped) {
            continue;
        }
        transportablesNotifyMove(v, laneVehicles, riders, stopped, kind == 1, alight);
    }
}

void Sim::transportablesNotifyMove(int v, int laneVehicles, std::vector<int>& riders, bool& stopped, bool containers,
                                   std::vector<Alighting>& alight) {
    Veh& x = veh[v];
    if (stopped) {
        if (!isStopped(v)) {
            for (int pi : riders) {
                PStage& s = persons[pi].plan[persons[pi].step];
                if (s.departed < 0) {
                    s.departed = now;  // MSStage::setDeparted
                }
            }
            stopped = false;
        }
        return;
    }
    if (!isStopped(v)) {
        return;
    }
    stopped = true;
    Stop& stop = x.stops.front();
    const Time boardingDuration = containers ? vt(v).loadingDuration : vt(v).boardingDuration;
    for (size_t i = 0; i < riders.size();) {
        const int pi = riders[i];
        if (rideCanLeave(pi, v, stop) && !willTransferAtJoin(pi, stop.joinVeh)) {
            Time& timeForNext = containers ? stop.timeToLoadNextContainer : stop.timeToBoardNextPerson;
            if (timeForNext - DELTA_T > now) {
                // try deboarding again in the next step
                stopped = false;
                break;
            }
            const Time boardingTime = (Time)((double)boardingDuration * types[persons[pi].type].boardingFactor);
            if (timeForNext > now - DELTA_T) {
                timeForNext += boardingTime;
            } else {
                timeForNext = now + boardingTime;
            }
            // the vehicle stops long enough for the deboarding
            stop.duration = std::max(stop.duration, timeForNext - now);
            riders.erase(riders.begin() + (long)i);
            alight.push_back({pi, v, kin[v].lane, laneVehicles});
            continue;
        }
        ++i;
    }
}

// MSStageDriving::canLeaveVehicle (without access edges)
bool Sim::rideCanLeave(int pi, int v, const Stop& stop) const {
    const Person& p = persons[pi];
    const PStage& s = p.plan[p.step];
    const int stopEdge = net.lanes[stop.lane].edge;
    if (s.destination == stopEdge) {
        const bool last = p.step + 1 == (int)p.plan.size();
        const bool unspecifiedAP = s.arrivalPosUnspecified && (!last || !p.arrivalPosSet);
        const double arrivalPos = s.arrivalPosUnspecified ? interpretEdgePos(p.arrivalPos, net.edges[stopEdge].length)
                                  : s.arrivalPos;
        // MSStop::isInRange
        const double tolerance = vt(v).length + opt.stopTolerance;
        if (unspecifiedAP || (stop.startPos - tolerance <= arrivalPos && stop.endPos + tolerance >= arrivalPos)) {
            return true;
        }
    }
    if (s.destinationStop >= 0 || s.destinationParking >= 0) {
        const int lane = s.destinationStop >= 0 ? busStops[s.destinationStop].lane : parkingAreas[s.destinationParking].lane;
        const double beg = s.destinationStop >= 0 ? busStops[s.destinationStop].startPos : parkingAreas[s.destinationParking].begPos;
        const double end = s.destinationStop >= 0 ? busStops[s.destinationStop].endPos : parkingAreas[s.destinationParking].endPos;
        if (net.lanes[lane].edge == net.lanes[kin[v].lane].edge) {
            // with more tolerance due to the size of the stop (the access position is its middle)
            const double accessPos = (beg + end) / 2;
            const double tolerance = vt(v).length + opt.stopTolerance + (end - beg) / 2;
            return stop.startPos - tolerance <= accessPos && stop.endPos + tolerance >= accessPos;
        }
    }
    return false;
}

// the persons that left a vehicle in the move phase continue (MSTransportable::proceed), in lane order
void Sim::applyAlightings(const std::vector<Alighting>& alight) {
    for (const Alighting& a : alight) {
        const int pi = a.person;
        PStage& s = persons[pi].plan[persons[pi].step];
        if (s.destinationStop >= 0 && !persons[pi].container) {
            stopAddPerson(s.destinationStop, pi);
        }
        // (SUMO starts the next stage within the move of the vehicle: its lane still holds it)
        alightLane = a.lane;
        alightLaneVehicles = a.laneVehicles;
        if (!personProceed(pi)) {
            personErase(pi);
        }
        alightLane = -1;
    }
}

// MSDevice_Transportable::notifyLeave (vehicle arrival): the riders end their ride where the vehicle is
void Sim::ridersVehicleArrived(int v) {
    if (veh[v].exr().riders.empty() && veh[v].exr().cargo.empty()) {
        return;
    }
    const int edge = curEdge(v);
    std::vector<int> riders = std::move(veh[v].ex().riders);
    veh[v].ex().riders.clear();
    riders.insert(riders.end(), veh[v].ex().cargo.begin(), veh[v].ex().cargo.end());
    veh[v].ex().cargo.clear();
    for (int pi : riders) {
        const PStage& s = persons[pi].plan[persons[pi].step];
        if (s.destination != edge) {
            std::fprintf(stderr, "Warning: Teleporting person '%s' from vehicle destination edge '%s' to intended destination edge '%s' time=%.2f\n",
                         persons[pi].id.c_str(), net.edges[edge].id.c_str(), net.edges[s.destination].id.c_str(), STEPS2TIME(now));
        }
        rideArrivedWithVehicle = true;
        if (!personProceed(pi)) {
            personErase(pi);
        }
        rideArrivedWithVehicle = false;
    }
}

// MSTransportable::rerouteParkingArea: the vehicle drives to another parking area (parkingAreaReroute); a rider
// heading to the original one rides to the replacement, its next walk or stop is reached by a trip from there
void Sim::personRerouteParkingArea(int pi, int orig, int replacement) {
    Person& p = persons[pi];
    const int origEdge = net.lanes[parkingAreas[orig].lane].edge;
    PStage& s = p.plan[p.step];
    if (s.destination != origEdge) {
        return;
    }
    const ParkingArea& R = parkingAreas[replacement];
    s.destination = net.lanes[R.lane].edge;
    s.destinationStop = -1;
    s.destinationParking = replacement;
    s.arrivalPos = (R.begPos + R.endPos) / 2;
    s.arrivalPosUnspecified = false;
    auto trip = [&](int from, int to, int toStop, double arrivalPos) {
        // MSStageTrip(origin, nullptr, destination, toStop, -1, 0, "", -1, 1, id, 0, true, arrivalPos)
        PStage t;
        t.kind = PStage::TRIP;
        t.tripFrom = from;
        t.destination = to;
        t.destinationStop = toStop;
        t.arrivalPos = arrivalPos;
        t.hasArrivalPos = true;
        t.walkFactor = 1;
        t.departPosLat = 0;
        return t;
    };
    if (p.step + 1 == (int)p.plan.size()) {
        return;
    }
    PStage& next = p.plan[p.step + 1];
    if (next.kind == PStage::TRIP) {
        next.tripFrom = s.destination;  // MSStageTrip::setOrigin
        next.departPos = s.arrivalPos;
    } else if (next.kind == PStage::WALKING) {
        next = trip(s.destination, next.destination, next.destinationStop, next.arrivalPos);
    } else if (next.kind == PStage::WAITING) {
        p.plan.insert(p.plan.begin() + p.step + 1, trip(s.destination, next.destination, next.destinationStop, next.arrivalPos));
    }
    // a later ride with the same vehicle from the original parking area starts at the replacement
    for (size_t i = p.step + 2; i < p.plan.size(); ++i) {
        if (p.plan[i].kind != PStage::DRIVING) {
            continue;
        }
        p.plan[i].origin = -1;
        PStage& prev = p.plan[i - 1];
        if (p.plan[i].lines == s.lines && prev.destination == origEdge) {
            if (prev.kind == PStage::TRIP) {
                prev.destination = s.destination;
                prev.destinationStop = -1;
            } else if (prev.kind == PStage::WALKING) {
                prev = trip(prev.route.front(), s.destination, -1, s.arrivalPos);
            }
        }
    }
}

// MSStageDriving::setArrived
void Sim::rideSetArrived(int pi) {
    PStage& s = persons[pi].plan[persons[pi].step];
    if (s.vehicle >= 0) {
        const int v = s.vehicle;
        s.vehicleDistance = vehOdometer(v) - s.vehicleDistance;
        s.rideTimeLoss = veh[v].timeLoss - s.rideTimeLoss;
        s.arrivalPos = rideArrivedWithVehicle ? veh[v].arrivalPos : kin[v].pos;
        s.arrivalPosUnspecified = false;
    } else {
        s.vehicleDistance = -1;
        s.rideTimeLoss = -1;
    }
    s.vehicle = -1;
}

// MSTransportableControl::abortAnyWaitingForVehicle (the simulation ends without further vehicles)
void Sim::abortAnyWaitingForVehicle() {
    std::vector<std::pair<int, std::vector<int>>> all(waiting4Vehicle.begin(), waiting4Vehicle.end());
    all.insert(all.end(), waiting4Container.begin(), waiting4Container.end());
    for (auto& w : all) {
        for (int pi : w.second) {
            const PStage& s = persons[pi].plan[persons[pi].step];
            std::string lines;
            for (const std::string& l : s.lines) {
                lines += (lines.empty() ? "" : ",") + l;
            }
            std::fprintf(stderr, "Warning: %s '%s' aborted waiting for %s at edge '%s'.\n", persons[pi].container ? "Container" : "Person",
                         persons[pi].id.c_str(), lines.c_str(), net.edges[w.first].id.c_str());
            edgeRemovePerson(pi);
            personErase(pi);
        }
    }
    nWaitingForVehicle = 0;
    waiting4Vehicle.clear();
    waiting4Container.clear();
}

// a vehicle that may board persons in this step (the next stop on its lane, persons waiting on the edge)
bool Sim::mayBoard(int v) const {
    if ((waiting4Vehicle.empty() && waiting4Container.empty()) || veh[v].stops.empty()) {
        return false;
    }
    const Stop& stop = veh[v].stops.front();
    const int edge = net.lanes[stop.lane].edge;
    return stop.lane == kin[v].lane && (waiting4Vehicle.count(edge) != 0 || waiting4Container.count(edge) != 0);
}

// MSVehicle::getPosition including parked vehicles (beside the road, SUMO_const_laneWidth to the right of the first lane)
Pos Sim::vehPositionParked(int v) const {
    if (isParking(v) && veh[v].stops.front().parkingArea >= 0) {
        // MSParkingArea::getVehiclePosition
        for (const ParkingLot& lot : parkingAreas[veh[v].stops.front().parkingArea].lots) {
            if (lot.vehicle == v) {
                return lot.position;
            }
        }
    }
    if (isParking(v) && veh[v].stops.front().parkingArea < 0) {
        const Lane& L0 = net.lanes[net.edges[net.lanes[kin[v].lane].edge].lanes[0]];
        const Lane& L = net.lanes[kin[v].lane];
        return geom::positionAtOffset(L0.shape, L.interpolateLanePosToGeometryPos(kin[v].pos), 3.2);
    }
    return vehPosition(v);
}

// MSVehicle::computeAngle including parked vehicles
double Sim::vehAngleParked(int v) const {
    if (isParking(v)) {
        const Lane& L = net.lanes[kin[v].lane];
        if (veh[v].stops.front().parkingArea >= 0) {
            for (const ParkingLot& lot : parkingAreas[veh[v].stops.front().parkingArea].lots) {
                if (lot.vehicle == v) {
                    return (lot.rotation - 90.) * M_PI / 180.;  // MSParkingArea::getVehicleAngle
                }
            }
            return 0;
        }
        return geom::rotationAtOffset(L.shape, L.interpolateLanePosToGeometryPos(kin[v].pos));
    }
    return vehAngle(v);
}

// MSStoppingPlace::getWaitingPositionOnLane
double Sim::stopWaitingPosOnLane(int stop, int pi) const {
    const BusStop& b = busStops[stop];
    const Person& p = persons[pi];
    if (p.atStop == stop && p.stopSpot >= 0) {
        const double width = b.category == 'C' ? 2.5 : 0.8;  // SUMO_const_waitingContainerWidth / PersonWidth
        const int abreast = std::max(1, (int)std::floor((b.endPos - b.startPos) / width));
        return b.endPos - (0.5 + p.stopSpot % abreast) * width;
    }
    return (b.endPos + b.startPos) / 2;
}

// MSStoppingPlace::getWaitPosition (persons waiting at a bus stop stand in rows beside the lane)
Pos Sim::stopWaitPosition(int stop, int pi) const {
    const BusStop& b = busStops[stop];
    const Person& p = persons[pi];
    // SUMO_const_waitingContainerWidth / Depth at container stops, else SUMO_const_waitingPersonWidth / Depth
    const double width = b.category == 'C' ? 2.5 : 0.8, depth = b.category == 'C' ? 6.2 : 0.67;
    const int abreast = std::max(1, (int)std::floor((b.endPos - b.startPos) / width));
    int row = 0;
    if (p.atStop == stop) {
        row = p.stopSpot >= 0 ? p.stopSpot / abreast : 1 + b.waitingCapacity / abreast;
    }
    const Lane& L = net.lanes[b.lane];
    return geom::positionAtOffset(L.shape, L.interpolateLanePosToGeometryPos(stopWaitingPosOnLane(stop, pi)),
                                  L.width / 2 + row * depth + std::fabs(std::cos(b.angle)) * width / 2);
}

// MSDevice_Transportable::willTransferAtJoin: the next ride is in the train this one joins
bool Sim::willTransferAtJoin(int pi, int joinVeh) const {
    const Person& p = persons[pi];
    if (joinVeh < 0 || (int)p.plan.size() - p.step <= 1) {
        return false;
    }
    const PStage& s2 = p.plan[p.step + 1];
    return s2.kind == PStage::DRIVING && rideWaitingFor(s2, joinVeh);
}

// MSDevice_Transportable::transferAtSplitOrJoin: the riders whose next ride is in the other train part change
// into it (their ride ends here, the next one starts in the other part)
void Sim::transferAtSplitOrJoin(int v, int other) {
    if (veh[v].extra.p == nullptr) {
        return;
    }
    const Stop stop = veh[v].stops.front();
    for (bool containers : {false, true}) {
        size_t i = 0;
        while (true) {
            std::vector<int>& list = containers ? veh[v].ex().cargo : veh[v].ex().riders;
            if (i >= list.size()) {
                break;
            }
            const int pi = list[i];
            Person& p = persons[pi];
            if ((int)p.plan.size() - p.step > 1 && p.plan[p.step].kind == PStage::DRIVING && rideCanLeave(pi, v, stop)) {
                const PStage& s2 = p.plan[p.step + 1];
                if (s2.kind == PStage::DRIVING && rideWaitingFor(s2, other)) {
                    list.erase(list.begin() + (long)i);
                    // proceeding registers the transportable as waiting on the edge
                    personProceed(pi);
                    std::map<int, std::vector<int>>& waiting = containers ? waiting4Container : waiting4Vehicle;
                    auto wit = waiting.find(stageEdge(p.plan[p.step]));
                    if (wit != waiting.end()) {
                        auto it2 = std::find(wit->second.begin(), wit->second.end(), pi);
                        if (it2 != wit->second.end()) {
                            wit->second.erase(it2);
                            nWaitingForVehicle--;
                        }
                    }
                    edgeRemovePerson(pi);
                    vehAddPerson(other, pi);
                    rideSetVehicle(pi, other);
                    continue;
                }
            }
            i++;
        }
    }
}

}  // namespace ps
