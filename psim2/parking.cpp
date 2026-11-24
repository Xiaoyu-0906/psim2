// Parking areas, ported from SUMO 1.28 MSParkingArea (lots, on-road parking, reservations, egress), and the
// parking search of rerouters (parkingAreaReroute: MSTriggeredRerouter::rerouteParkingArea,
// MSStoppingPlaceRerouter::rerouteStoppingPlace / evaluateDestination, MSBaseVehicle::replaceParkingArea).
// Not supported: parking manoeuvres (--parking.maneuver), opportunistic parking, riders.
//
// A parking area is changed by the vehicles on its lane: reservations in planMove, entering and leaving in
// executeMove. Both phases run lane by lane in parallel, so these changes happen in the task of the area's
// lane only (in lane order, like SUMO); vehicles on other lanes read the snapshot taken at the step begin.
// Parking searches (and other rerouter actions of vehicles heading for a parking area) are deferred until
// the move phase is over (Sim::applyDeferredReroutes).
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>

#include "model.hpp"

namespace ps {

// MSStoppingPlaceRerouter's evaluation parameters, in the (alphabetical) order of its std::map
enum { P_ABSFREE, P_CAPACITY, P_DISTFROM, P_DISTTO, P_PROB, P_RELFREE, P_TIMEFROM, P_TIMETO, NPARAM };
using ParamValues = std::array<double, NPARAM>;
static const char* const PARAM_NAMES[NPARAM] = {"absfreespace", "capacity", "distancefrom", "distanceto",
                                                "probability", "relfreespace", "timefrom", "timeto"};
static const bool PARAM_INVERT[NPARAM] = {true, true, false, false, false, true, false, false};

// the state of one parking search (rerouteStoppingPlace)
struct ParkingSearch {
    int veh;
    uint64_t drawBase;
    uint64_t nDraws = 0;
    ParamValues weights{};
    ParamValues maxValues{};
    std::map<std::string, std::pair<int, ParamValues>> places;  // StoppingPlaceMap_t (ordered by id)
    std::map<int, std::vector<int>> newRoutes, approaches;
};

static constexpr double LANE_WIDTH = 3.2;  // SUMO_const_laneWidth

// PARKINGAREA id lane startPos endPos friendlyPos roadsideCapacity onRoad width length angle departPos reservable badges
// PASPACE id x y
void Sim::parseParkingAreaLine(const std::vector<std::string>& tok) {
    if (tok[0] == "PASPACE") {
        ParkingArea& pa = parkingAreas.at(parkingAreaIdx.at(tok[1]));
        if (pa.onRoad) {
            throw std::runtime_error("Cannot not add lot entry to on-road parking area.");
        }
        // MSParkingArea::addLotEntry: without parking manoeuvres, a lot ends at the end of the area
        ParkingLot lot;
        lot.endPos = pa.endPos;
        lot.position = Pos{std::stod(tok[2]), std::stod(tok[3])};
        lot.rotation = tok.size() > 4 && tok[4] != "-" ? std::stod(tok[4]) : pa.angle;
        pa.lots.push_back(lot);
        pa.capacity++;
        paComputeLastFreePos(pa);
        return;
    }
    ParkingArea pa;
    pa.id = tok[1];
    pa.lane = net.laneIdx.at(tok[2]);
    const double laneLength = net.lanes[pa.lane].length;
    double beg = std::stod(tok[3]);
    double end = tok[4] == "-" ? laneLength : std::stod(tok[4]);
    const bool friendlyPos = tok[5] == "1";
    // SUMORouteHandler::checkStopPos
    bool valid = POSITION_EPS <= laneLength;
    if (beg < 0) {
        beg += laneLength;
    }
    if (end < 0) {
        end += laneLength;
    }
    if (end < POSITION_EPS || end > laneLength) {
        valid = valid && friendlyPos;
        end = std::min(std::max(end, POSITION_EPS), laneLength);
    }
    if (beg < 0 || beg > end - POSITION_EPS) {
        valid = valid && friendlyPos;
        beg = std::min(std::max(beg, 0.0), end - POSITION_EPS);
    }
    if (!valid) {
        throw std::runtime_error("Invalid position for parking area '" + pa.id + "'.");
    }
    pa.begPos = beg;
    pa.endPos = end;
    pa.roadsideCapacity = std::stoi(tok[6]);
    pa.onRoad = tok[7] == "1";
    pa.width = std::stod(tok[8]);
    if (pa.width == 0) {
        pa.width = LANE_WIDTH;
    }
    pa.length = std::stod(tok[9]);
    pa.angle = std::stod(tok[10]);
    if (tok[11] != "-") {
        pa.departPos = std::stod(tok[11]);
        if (pa.departPos < 0 || pa.departPos > laneLength) {
            throw std::runtime_error("Invalid departPos for parkingArea '" + pa.id + "'");
        }
    }
    pa.reservable = tok[12] == "1";
    if (tok[13] != "-") {
        std::stringstream ss(tok[13]);
        std::string b;
        while (std::getline(ss, b, ',')) {
            pa.badges.push_back(b);
        }
    }
    paSetRoadsideCapacity(pa);
    parkingAreaIdx[pa.id] = (int)parkingAreas.size();
    parkingAreas.push_back(pa);
}

// MSParkingArea::setRoadsideCapacity: equal lots along the area
void Sim::paSetRoadsideCapacity(ParkingArea& pa) {
    const Lane& L = net.lanes[pa.lane];
    const double spaceOffset = pa.roadsideCapacity > 0 ? (pa.endPos - pa.begPos) / pa.roadsideCapacity : 7.5;
    // the lot geometry (MSParkingArea constructor: myShape; GeomHelper::calculateLotSpacePosition / Angle)
    const double spaceDim = pa.roadsideCapacity > 0 ? L.interpolateLanePosToGeometryPos(spaceOffset) : 7.5;
    const double width = pa.width == 0 ? 3.2 : pa.width;  // SUMO_const_laneWidth
    const double length = pa.length == 0 ? spaceDim : pa.length;
    Shape shape = geom::getSubpart(L.shape, L.interpolateLanePosToGeometryPos(pa.begPos),
                                         L.interpolateLanePosToGeometryPos(pa.endPos));
    if (!pa.onRoad) {
        geom::move2side(shape, L.width / 2. + width / 2.);
    }
    pa.lots.clear();
    pa.capacity = 0;
    for (int i = 0; i < pa.roadsideCapacity; ++i) {
        ParkingLot lot;
        lot.endPos = pa.endPos;
        const Pos start = geom::positionAtOffset2D(shape, spaceDim * i);
        const Pos end = geom::positionAtOffset2D(shape, spaceDim * (i + 1));
        if (pa.angle == 0) {
            lot.position = end;  // parallel to the road
        } else {
            double normAngle = std::fmod(pa.angle < 0 ? pa.angle + 360. * std::ceil(-pa.angle / 360.) : pa.angle, 360.);
            const double ra = normAngle / 180 * M_PI;
            const double spaceExtension = width * std::sin(ra) + length * std::cos(ra);
            const double hlpAngle = std::atan2(end.y - start.y, end.x - start.x);
            Pos p;
            p.x = start.x + 0.5 * width * std::sin(ra) - 0.5 * (spaceExtension - spaceDim) + length * std::cos(ra);
            if (normAngle <= 90) {
                p.y = start.y + 0.5 * width * (1 - std::cos(ra)) - length * std::sin(ra);
            } else if (normAngle <= 180) {
                p.y = start.y + 0.5 * width * (1 + std::cos(ra)) - length * std::sin(ra);
            } else if (pa.angle <= 270) {
                p.y = start.y + 0.5 * width * (1 - std::cos(ra - M_PI));
            } else {
                p.y = start.y + 0.5 * width * (1 + std::cos(ra - M_PI));
            }
            // rotateAround2D(hlpAngle, start)
            const double dx = p.x - start.x, dy = p.y - start.y;
            lot.position = Pos{start.x + dx * std::cos(hlpAngle) - dy * std::sin(hlpAngle),
                               start.y + dx * std::sin(hlpAngle) + dy * std::cos(hlpAngle)};
        }
        lot.rotation = std::atan2(end.x - start.x, start.y - end.y) * 180. / M_PI - pa.angle;
        pa.lots.push_back(lot);
        pa.capacity++;
        paComputeLastFreePos(pa);
        pa.lots.back().endPos = std::min(pa.endPos, pa.begPos + std::max(POSITION_EPS, spaceOffset * (i + 1)));
    }
    paComputeLastFreePos(pa);
}

// MSParkingArea::computeLastFreePos
void Sim::paComputeLastFreePos(ParkingArea& pa) {
    pa.lastFreeLot = -1;
    pa.lastFreePos = pa.begPos;
    pa.egressBlocked = false;
    for (int i = 0; i < (int)pa.lots.size(); ++i) {
        const int other = pa.lots[i].vehicle;
        // remainingStopDuration: the vehicles in the area are stopped there
        if (other < 0 || (pa.occupancy() == pa.capacity && veh[other].stops.front().duration <= 0)) {
            pa.lastFreeLot = i;
            if (other < 0) {
                pa.lastFreePos = pa.lots[i].endPos;
            } else {
                // the vehicle wants to exit the parking area
                pa.lastFreePos = pa.lots[i].endPos - vt(other).length - POSITION_EPS;
                pa.egressBlocked = true;
            }
            break;
        }
        pa.lastFreePos = std::min(pa.lastFreePos, pa.lots[i].endPos - vt(other).length - NUMERICAL_EPS);
    }
}

// MSParkingArea::mustAdvance: on-road parking that the vehicles cannot overtake
bool Sim::paMustAdvance(const ParkingArea& pa, SVCPermissions vc) const {
    if (!pa.onRoad) {
        return false;
    }
    const Lane& l = net.lanes[pa.lane];
    const std::vector<int>& lanes = net.edges[l.edge].lanes;
    const int left = l.index + 1 < (int)lanes.size() ? lanes[l.index + 1] : -1;
    const int right = l.index > 0 ? lanes[l.index - 1] : -1;
    return (left < 0 || !net.lanes[left].allows(vc) || (l.changeLeft & vc) != vc)
           && (right < 0 || !net.lanes[right].allows(vc) || (l.changeRight & vc) != vc);
}

// MSParkingArea::getLastFreePos; live: the reservations of the current step may be read (otherwise the
// caller is a vehicle on another lane in the parallel planMove phase)
double Sim::paLastFreePos(const ParkingArea& pa, int v, double brakePos, bool live) const {
    if (pa.capacity == (int)pa.parked.size()) {
        // keep enough space so that parking vehicles can leave
        return pa.lastFreePos - vt(v).minGap - POSITION_EPS;
    }
    if (paMustAdvance(pa, veh[v].vclass)) {
        // vehicles cannot overtake so we must fill from the downstream end
        int skipN = live && now == pa.reservationTime ? pa.reservations - 1 : 0;
        for (auto it = pa.lots.rbegin(); it != pa.lots.rend(); ++it) {
            if (it->vehicle < 0) {
                if (skipN > 0) {
                    skipN--;
                    continue;
                }
                return it->endPos;
            }
        }
        return brakePos;
    }
    const double minPos = std::min(pa.endPos, brakePos);
    if (pa.lastFreePos >= minPos) {
        return pa.lastFreePos;
    }
    for (const ParkingLot& lot : pa.lots) {
        if (lot.vehicle < 0 && lot.endPos >= minPos) {
            return lot.endPos;
        }
    }
    return brakePos;
}

// MSParkingArea::getLastFreePosWithReservation (planMove of a vehicle approaching the area)
double Sim::paLastFreePosWithReservation(int area, int v, double brakePos) {
    ParkingArea& pa = parkingAreas[area];
    const VType& t = vt(v);
    const bool ownLane = kin[v].lane == pa.lane;
    if (!ownLane || kin[v].pos < pa.begPos - pa.maxVehLength - t.minGap) {
        // no reservations from other lanes (lane-order dependency) or when far away
        if (pa.numAlternatives > 0 && pa.occupancy() == pa.capacity) {
            // ensure that the vehicle reaches the rerouter lane
            if (!pa.lots.empty() && pa.lots[0].vehicle >= 0) {
                const double waitPos = pa.lots[0].endPos - vt(pa.lots[0].vehicle).length - t.minGap - NUMERICAL_EPS;
                return std::max(waitPos, std::min(POSITION_EPS, pa.endPos));
            }
            return std::max(pa.begPos, std::min(POSITION_EPS, pa.endPos));
        }
        // a reservation from the last step
        int res = -1;
        double maxLen = 0;
        if (ownLane) {
            const Time last = now - DELTA_T;
            if (pa.lastReservationTime == last || pa.reservationTime == last) {
                res = pa.lastReservationTime == last ? pa.lastReservations : pa.reservations;
                maxLen = pa.lastReservationTime == last ? pa.lastReservationMaxLength : pa.reservationMaxLength;
            }
        } else {
            res = pa.snapReservations;
            maxLen = pa.snapReservationMaxLength;
        }
        if (res >= 0 && pa.capacity <= pa.occupancy() + res && !pa.lots.empty()) {
            return pa.lots[0].endPos - maxLen - t.minGap - NUMERICAL_EPS;
        }
        return paLastFreePos(pa, v, brakePos, ownLane);
    }
    if (now > pa.reservationTime) {
        pa.lastReservationTime = pa.reservationTime;
        pa.lastReservations = pa.reservations;
        pa.lastReservationMaxLength = pa.reservationMaxLength;
        pa.reservationTime = now;
        pa.reservations = 1;
        pa.reservationMaxLength = t.length;
        pa.reservedVehicles.assign(1, v);
        for (const ParkingLot& lot : pa.lots) {
            if (lot.vehicle >= 0) {
                pa.reservationMaxLength = std::max(pa.reservationMaxLength, vt(lot.vehicle).length);
            }
        }
        return paLastFreePos(pa, v, brakePos, true);
    }
    if (pa.capacity > pa.occupancy() + pa.reservations) {
        pa.reservations++;
        pa.reservationMaxLength = std::max(pa.reservationMaxLength, t.length);
        if (std::find(pa.reservedVehicles.begin(), pa.reservedVehicles.end(), v) == pa.reservedVehicles.end()) {
            pa.reservedVehicles.push_back(v);
        }
        return paLastFreePos(pa, v, brakePos, true);
    }
    if (pa.capacity == 0) {
        return paLastFreePos(pa, v, brakePos, true);
    }
    return pa.lots[0].endPos - pa.reservationMaxLength - t.minGap - NUMERICAL_EPS;
}

// MSParkingArea::getLotIndex
int Sim::paLotIndex(const ParkingArea& pa, int v) const {
    const double pos = kin[v].pos;
    if (pos > pa.lastFreePos) {
        // the vehicle has gone past lastFreePos and we need to find the actual lot
        int closestLot = -1;
        for (int i = 0; i < (int)pa.lots.size(); ++i) {
            if (pa.lots[i].vehicle < 0) {
                closestLot = i;
                if (pa.lots[i].endPos >= pos) {
                    return i;
                }
            }
        }
        return closestLot;
    }
    if (pa.onRoad && pa.lastFreePos - pos > POSITION_EPS) {
        // for on-road parking we need to be precise
        return -1;
    }
    return pa.lastFreeLot;
}

thread_local int execTaskOrder = -1;
thread_local int execTaskLane = -1;
thread_local int execTaskSeq = 0;

// after loading: the areas of each lane, the buffers of the move phase
void Sim::initParkingAreas() {
    if (parkingAreas.empty()) {
        return;
    }
    laneParkingAreas.assign(net.lanes.size(), {});
    for (int i = 0; i < (int)parkingAreas.size(); ++i) {
        laneParkingAreas[parkingAreas[i].lane].push_back(i);
    }
    paStart.resize(parkingAreas.size());
    paStartStep.assign(parkingAreas.size(), -1);
}

// the first change of an area in the move phase (by the task of its lane): keep the state before it
void Sim::paBeforeChange(int area) {
    if (deferReroutes && paStartStep[area] != step) {
        paStart[area] = parkingAreas[area];
        paStartStep[area] = (int)step;
    }
}

// MSParkingArea::enter
void Sim::paEnter(int area, int v) {
    paBeforeChange(area);
    ParkingArea& pa = parkingAreas[area];
    paRemoveReservation(pa, v);
    int lot = paLotIndex(pa, v);
    if (lot < 0) {
        std::fprintf(stderr, "Warning: Unsuitable parking position for vehicle '%s' at parkingArea '%s' time=%.2f.\n",
                     veh[v].id.c_str(), pa.id.c_str(), STEPS2TIME(now));
        lot = pa.lastFreeLot;
    }
    if (lot < 0 || lot >= (int)pa.lots.size() || pa.lots[lot].vehicle >= 0) {
        // (SUMO accesses an invalid lot here)
        for (lot = 0; lot < (int)pa.lots.size() && pa.lots[lot].vehicle >= 0; ++lot) {
        }
    }
    if (lot < (int)pa.lots.size()) {
        pa.lots[lot].vehicle = v;
    }
    if (v == traceVeh) {
        std::fprintf(stderr, "%.2f paEnter %s area=%s lot=%d pos=%.2f lastFreePos=%.2f\n", STEPS2TIME(now), veh[v].id.c_str(),
                     pa.id.c_str(), lot, kin[v].pos, pa.lastFreePos);
    }
    pa.parked.push_back(v);
    paComputeLastFreePos(pa);
    pa.maxVehLength = std::max(pa.maxVehLength, vt(v).length);
    // the current search ends here
    veh[v].parkingReroutes = 0;
}

// MSParkingArea::leaveFrom
void Sim::paLeave(int area, int v) {
    paBeforeChange(area);
    ParkingArea& pa = parkingAreas[area];
    pa.updatePending = true;
    for (ParkingLot& lot : pa.lots) {
        if (lot.vehicle == v) {
            lot.vehicle = -1;
            break;
        }
    }
    if (vt(v).length == pa.maxVehLength) {
        // (including the leaving vehicle, like SUMO)
        pa.maxVehLength = 0;
        for (int other : pa.parked) {
            pa.maxVehLength = std::max(pa.maxVehLength, vt(other).length);
        }
    }
    pa.parked.erase(std::remove(pa.parked.begin(), pa.parked.end(), v), pa.parked.end());
    paComputeLastFreePos(pa);
}

void Sim::paRemoveReservation(ParkingArea& pa, int v) {
    pa.remoteReserved.erase(std::remove(pa.remoteReserved.begin(), pa.remoteReserved.end(), v), pa.remoteReserved.end());
    pa.updatePending = true;
}

void Sim::paAddReservation(ParkingArea& pa, int v) {
    if (std::find(pa.remoteReserved.begin(), pa.remoteReserved.end(), v) == pa.remoteReserved.end()) {
        pa.remoteReserved.push_back(v);
    }
    pa.updatePending = true;
}

static bool contains(const std::vector<int>& c, int v) {
    return std::find(c.begin(), c.end(), v) != c.end();
}

int Sim::paOccupancyIncludingReservations(const ParkingArea& pa, int v) const {
    return (int)pa.parked.size() + (contains(pa.reservedVehicles, v) ? 0 : pa.reservations)
           + (contains(pa.remoteReserved, v) ? 0 : (int)pa.remoteReserved.size());
}

int Sim::paOccupancyIncludingRemoteReservations(const ParkingArea& pa, int v) const {
    return pa.occupancy() + (int)pa.remoteReserved.size() - (contains(pa.remoteReserved, v) ? 1 : 0);
}

int Sim::paLastStepOccupancyIncludingRemoteReservations(const ParkingArea& pa, int v) const {
    return pa.lastStepOccupancy + (int)pa.lastRemoteReserved.size() - (contains(pa.lastRemoteReserved, v) ? 1 : 0);
}

// MSParkingArea::getInsertionPosition
double Sim::paInsertionPosition(const ParkingArea& pa, int v) const {
    if (pa.departPos >= 0) {
        return pa.departPos;
    }
    for (const ParkingLot& lot : pa.lots) {
        if (lot.vehicle == v) {
            return lot.endPos;
        }
    }
    return -1;
}

// MSParkingArea::accepts
bool Sim::paAccepts(const ParkingArea& pa, int v) const {
    if (pa.badges.empty()) {
        return true;
    }
    for (const std::string& b : veh[v].exr().parkingBadges) {
        if (std::find(pa.badges.begin(), pa.badges.end(), b) != pa.badges.end()) {
            return true;
        }
    }
    return false;
}

// before planMove: the state read by vehicles on other lanes
void Sim::paSnapshot() {
    const Time last = now - DELTA_T;
    for (ParkingArea& pa : parkingAreas) {
        pa.snapOccupancy = pa.occupancy();
        pa.snapReservations = pa.reservationTime == last ? pa.reservations : -1;
        pa.snapReservationMaxLength = pa.reservationMaxLength;
    }
}

// MSParkingArea::updateOccupancy (end-of-step event of the areas that changed)
void Sim::paEndStep() {
    for (ParkingArea& pa : parkingAreas) {
        if (pa.updatePending) {
            pa.lastStepOccupancy = pa.occupancy();
            pa.lastRemoteReserved = pa.remoteReserved;
            pa.updatePending = false;
        }
    }
}

// MSVehicle::setBlinkerInformation at the end of the step in which the vehicle stopped to park: whether a
// blinker is on (MSVehicleTransfer signals the wish to leave only when none is)
bool Sim::parkingSignalAtStop(int v) const {
    const Veh& x = veh[v];
    const Kin& k = kin[v];
    const int state = x.lc.ownState;
    const bool blinkerManoeuvre = (state & LCA_SUBLANE) == 0 && ((state & LCA_KEEPRIGHT) == 0 || (state & LCA_BLOCKED) == 0);
    if (((state & LCA_LEFT) != 0 || (state & LCA_RIGHT) != 0) && blinkerManoeuvre) {
        return true;
    }
    const int link = succLinkSec(v, 1, k.lane, bestContinuation(v));
    if (link >= 0 && net.lanes[k.lane].length - k.pos < laneVehMaxSpeed(k.lane, v) * 7.) {
        const char dir = net.links[link].dir;
        if (dir == 't' || dir == 'l' || dir == 'L' || dir == 'r' || dir == 'R') {
            return true;
        }
    }
    if (!x.stops.empty()) {
        // stopping related signals (the stop is reached)
        const Lane& sl = net.lanes[x.stops.front().lane];
        if (sl.index > 0 && net.lanes[net.edges[sl.edge].lanes[sl.index - 1]].allows(x.vclass)) {
            return true;  // emergency blinkers
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// parking search (parkingAreaReroute)
// ---------------------------------------------------------------------------
bool Sim::parkingStops(int v) const {
    for (const Stop& s : veh[v].stops) {
        if (s.parkingArea >= 0) {
            return true;
        }
    }
    return false;
}

// MSStoppingPlaceRerouter::getWeight: the vehicle's parameter "parking.<key>", otherwise that of its type
double Sim::parkingWeight(int v, const std::string& key, double def) const {
    for (const auto& p : veh[v].exr().parkingParams) {
        if (p.first == key) {
            return p.second;
        }
    }
    return def;
}

void Sim::rememberBlockedParkingArea(int v, int area, bool local) {
    std::vector<ParkingMemory>& mem = veh[v].ex().parkingMemory;
    auto it = std::find_if(mem.begin(), mem.end(), [&](const ParkingMemory& m) { return m.area == area; });
    if (it == mem.end()) {
        mem.push_back({area});
        it = mem.end() - 1;
    }
    it->blockedAtTime = now;
    if (local) {
        it->blockedAtTimeLocal = now;
    }
}

Time Sim::sawBlockedParkingArea(int v, int area, bool local) const {
    for (const ParkingMemory& m : veh[v].exr().parkingMemory) {
        if (m.area == area) {
            return local ? m.blockedAtTimeLocal : m.blockedAtTime;
        }
    }
    return -1;
}

// MSEdge::getInternalFollowingLengthTo (SVC_IGNORING)
double Sim::internalFollowingLength(int from, int to) const {
    auto following = [&](int e) {
        for (int l : net.edges[e].lanes) {
            for (int k : net.lanes[l].links) {
                const Link& link = net.links[k];
                if (net.lanes[link.lane].edge == to) {
                    return link.via >= 0 ? net.lanes[link.via].edge : -1;
                }
            }
        }
        return -1;
    };
    double dist = 0;
    int e = following(from);
    while (e >= 0 && net.edges[e].internal) {
        dist += net.edges[e].length;
        e = following(e);
    }
    return dist;
}

// MSRoute::getDistanceBetween(fromPos, toPos, begin, end - 1, includeInternal)
double Sim::routeDistanceBetween(const std::vector<int>& edges, double fromPos, double toPos) const {
    if (edges.size() == 1) {
        return fromPos <= toPos ? toPos - fromPos : std::numeric_limits<double>::max();
    }
    double distance = -fromPos;
    for (size_t i = 0; i < edges.size(); ++i) {
        if (i + 1 == edges.size()) {
            distance += toPos;
            break;
        }
        distance += net.edges[edges[i]].length + internalFollowingLength(edges[i], edges[i + 1]);
    }
    return distance;
}

// MSStoppingPlaceRerouter::evaluateDestination
bool Sim::parkingEvaluate(int v, ParkingSearch& S, double brakeGap, bool newDestination, int alt, double occupancy, double prob,
                          RouteWS& ws) {
    const Veh& x = veh[v];
    const Kin& k = kin[v];
    const ParkingArea& pa = paRead(alt);
    const std::vector<int>& route = routes[x.route];
    const int paEdge = net.lanes[pa.lane].edge;
    const double targetPos = paLastFreePos(pa, v, 0, true);
    const int originIdx = rerouteOriginIdx(v);
    std::vector<int> edgesToStop;
    routeComputePos(route[originIdx], k.pos, paEdge, targetPos, v, edgesToStop, ws);
    if (edgesToStop.empty()) {
        return false;  // unreachable
    }
    if (route[originIdx] != route[x.routeIdx]) {
        edgesToStop.insert(edgesToStop.begin(), route[x.routeIdx]);
    }
    S.approaches[alt] = edgesToStop;
    int nextDestinationIndex = (int)route.size() - 1;
    std::vector<int> edgesFromStop;
    if (!newDestination) {
        int nextDestination = route.back();
        double nextPos = x.arrivalPos;
        // getStopIndices: the stop after the replaced one
        if (x.stops.size() > 1) {
            nextDestinationIndex = x.stops[1].routeIdx;
            nextDestination = route[nextDestinationIndex];
            nextPos = stopEndPos(v, x.stops[1]);
        }
        routeComputePos(paEdge, targetPos, nextDestination, nextPos, v, edgesFromStop, ws);
        if (edgesFromStop.empty()) {
            return false;  // unreachable
        }
    }
    ParamValues val{};
    val[P_PROB] = prob;
    val[P_CAPACITY] = pa.capacity;
    val[P_ABSFREE] = val[P_CAPACITY] - occupancy;
    val[P_RELFREE] = val[P_ABSFREE] / std::max(1.0, val[P_CAPACITY]);
    double toPos = pa.begPos;
    if (paEdge == curEdge(v)) {
        toPos = std::max(k.pos, toPos);
    }
    val[P_DISTTO] = routeDistanceBetween(edgesToStop, k.pos, toPos);
    const double endPos = paLastFreePos(pa, v, k.pos + brakeGap, true);
    const double distToEnd = val[P_DISTTO] - toPos + endPos;
    if (distToEnd < brakeGap) {
        return false;  // too close
    }
    const double correctionLastEdge = (net.lanes[pa.lane].length - pa.endPos) / laneVehMaxSpeed(pa.lane, v);
    const double correctionFirstEdge = k.pos / laneVehMaxSpeed(net.edges[edgesToStop.front()].lanes[0], v);
    val[P_TIMETO] = routeCosts(edgesToStop, v, ws) - correctionLastEdge - correctionFirstEdge;
    std::vector<int> newEdges = edgesToStop;
    if (!newDestination) {
        val[P_DISTFROM] = routeDistanceBetween(edgesFromStop, pa.begPos, net.edges[edgesFromStop.back()].length);
        val[P_TIMEFROM] = routeCosts(edgesFromStop, v, ws) - pa.endPos / net.lanes[pa.lane].speed;
        newEdges.insert(newEdges.end(), edgesFromStop.begin() + 1, edgesFromStop.end());
        newEdges.insert(newEdges.end(), route.begin() + nextDestinationIndex + 1, route.end());
    }
    for (int i = 0; i < NPARAM; ++i) {
        S.maxValues[i] = std::max(S.maxValues[i], val[i]);
    }
    S.places[pa.id] = {alt, val};
    S.newRoutes[alt] = newEdges;
    return true;
}

// MSStoppingPlaceRerouter::rerouteStoppingPlace for parking areas (MSTriggeredRerouter::rerouteParkingArea)
int Sim::rerouteStoppingPlace(int v, int ri, const RerouteInterval& def, int dest, bool& newDestination,
                              std::vector<int>& newRoute, RouteWS& ws) {
    Veh& x = veh[v];
    const Kin& k = kin[v];
    ParkingSearch S;
    S.veh = v;
    S.drawBase = ((uint64_t)step * 4096 + (uint64_t)ri) << 16;
    auto rand01v = [&]() { return rand01(opt.seed, (uint64_t)v, S.drawBase + S.nDraws++, RP_PARKING); };
    // the candidates the vehicle may use (the probabilities keep their original indices, like SUMO)
    std::vector<std::pair<int, bool>> cands;
    for (const std::pair<int, bool>& c : def.parkAreas) {
        if (paAccepts(paRead(c.first), v)) {
            cands.push_back(c);
        }
    }
    const std::vector<double>& probs = def.parkProbs;
    const int vehEdge = curEdge(v);
    auto edgeOf = [&](int area) { return net.lanes[paRead(area).lane].edge; };
    auto capacity = [&](int area) { return (double)paRead(area).capacity; };
    auto lastStepOcc = [&](int area) { return (double)paLastStepOccupancyIncludingRemoteReservations(paRead(area), v); };
    bool destVisible = edgeOf(dest) == vehEdge;
    for (const std::pair<int, bool>& c : cands) {
        if (c.first == dest && c.second) {
            destVisible = true;
            break;
        }
    }
    const std::vector<int>& route = routes[x.route];
    int onTheWay = -1;
    const int stopAnywhere = (int)parkingWeight(v, "anywhere", -1);
    const bool ignoreDest = parkingWeight(v, "ignoreDest", 0) != 0;
    if (stopAnywhere < 0 || stopAnywhere > x.parkingReroutes) {
        if (!destVisible) {
            for (const std::pair<int, bool>& c : cands) {
                if (c.second && lastStepOcc(c.first) >= capacity(c.first)) {
                    rememberBlockedParkingArea(v, c.first, edgeOf(c.first) == vehEdge);
                }
            }
        }
    } else {
        double bestDist = std::numeric_limits<double>::max();
        const double brakeGap = cfBrakeGap(v, k.speed, vt(v).decel, vt(v).tau);
        for (const std::pair<int, bool>& c : cands) {
            if (c.second && edgeOf(c.first) == vehEdge && lastStepOcc(c.first) < capacity(c.first)) {
                const double distToStart = paRead(c.first).begPos - k.pos;
                const double distToEnd = paRead(c.first).endPos - k.pos;
                if (distToEnd > brakeGap && distToStart < bestDist) {
                    bestDist = distToStart;
                    onTheWay = c.first;
                }
            }
        }
    }
    if (!ignoreDest && !destVisible && onTheWay < 0) {
        return -1;
    }
    const bool destIsFull = lastStepOcc(dest) >= capacity(dest);
    if (!(ignoreDest || destIsFull || onTheWay >= 0)) {
        return -1;
    }
    const ParkingArea& destPA = paRead(dest);
    newDestination = edgeOf(dest) == route.back() && x.arrivalPos >= destPA.begPos && x.arrivalPos <= destPA.endPos
                     && x.stops.size() == 1;
    // collectWeights (the default weight of distanceto is changed for all later vehicles, like SUMO)
    Rerouter& r = rerouters[ri];
    r.distanceWeight = parkingWeight(v, "distance.weight", r.distanceWeight);
    for (int i = 0; i < NPARAM; ++i) {
        S.weights[i] = parkingWeight(v, std::string(PARAM_NAMES[i]) + ".weight", i == P_DISTTO ? r.distanceWeight : 0.);
    }
    S.weights[P_PROB] = parkingWeight(v, "probability.weight", 0.);
    const double brakeGap = cfBrakeGap(v, k.speed, vt(v).decel, 0.);
    if (onTheWay >= 0) {
        if (newDestination) {
            newRoute.push_back(vehEdge);
        } else {
            if (!parkingEvaluate(v, S, brakeGap, newDestination, onTheWay, lastStepOcc(onTheWay), 1, ws)) {
                std::fprintf(stderr, "Warning: Stopping place '%s' along the way cannot be used by vehicle '%s' for unknown reason\n",
                             paRead(onTheWay).id.c_str(), x.id.c_str());
                return -1;
            }
            newRoute = S.newRoutes[onTheWay];
        }
        return onTheWay;
    }
    int numAlternatives = 0;
    std::vector<std::tuple<Time, std::string, int>> blockedTimes;
    if (destIsFull) {
        rememberBlockedParkingArea(v, dest, edgeOf(dest) == vehEdge);
    }
    const Time memory = TIME2STEPS(parkingWeight(v, "memory", 600));
    const double frustration = parkingWeight(v, "frustration", 100);
    const double knowledge = parkingWeight(v, "knowledge", 0);
    for (int i = 0; i < (int)cands.size(); ++i) {
        const int c = cands[i].first;
        const bool visible = cands[i].second || (c == dest && destVisible);
        double occupancy = paOccupancyIncludingRemoteReservations(paRead(c), v);
        if (!visible && (knowledge == 0 || knowledge < rand01v())) {
            const double cap = capacity(c);
            const double minOccupancy = std::min(cap - NUMERICAL_EPS, x.parkingReroutes * cap / frustration);
            occupancy = minOccupancy + (cap - minOccupancy) * rand01v();
            const Time blockedTime = sawBlockedParkingArea(v, c, false);
            if (blockedTime >= 0 && now - blockedTime < memory) {
                occupancy = cap;
                blockedTimes.emplace_back(blockedTime, paRead(c).id, i);
            }
        }
        if (v == traceVeh) {
            std::fprintf(stderr, "   cand %s visible=%d occupancy=%.2f capacity=%.0f\n", paRead(c).id.c_str(), (int)visible, occupancy, capacity(c));
        }
        if (occupancy < capacity(c)) {
            if (parkingEvaluate(v, S, brakeGap, newDestination, c, occupancy, probs[i], ws)) {
                numAlternatives++;
            }
        } else if (visible) {
            rememberBlockedParkingArea(v, c, edgeOf(c) == vehEdge);
        }
    }
    if (numAlternatives == 0) {
        std::sort(blockedTimes.begin(), blockedTimes.end());
        for (const auto& item : blockedTimes) {
            const int i = std::get<2>(item);
            const int c = cands[i].first;
            const double occupancy = capacity(c) * rand01v();
            if (parkingEvaluate(v, S, brakeGap, newDestination, c, occupancy, probs[i], ws)) {
                numAlternatives = 1;
                break;
            }
        }
        if (numAlternatives == 0) {
            std::vector<std::tuple<Time, std::string, int>> candidates;
            for (const std::pair<int, bool>& c : cands) {
                if (c.first == dest) {
                    continue;
                }
                Time t = sawBlockedParkingArea(v, c.first, true);
                if (t < 0) {
                    t = -(Time)(rand01v() * 1000000);
                }
                candidates.emplace_back(t, paRead(c.first).id, c.first);
            }
            std::sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
                return std::get<0>(a) < std::get<0>(b) || (std::get<0>(a) == std::get<0>(b) && std::get<1>(a) < std::get<1>(b));
            });
            for (const auto& item : candidates) {
                if (v == traceVeh) {
                    std::fprintf(stderr, "   fallback %s t=%lld\n", std::get<1>(item).c_str(), (long long)std::get<0>(item));
                }
                if (parkingEvaluate(v, S, brakeGap, newDestination, std::get<2>(item), 0, 1, ws)) {
                    break;
                }
            }
        }
    }
    int nearest = -1;
    double minCost = 0;
    for (auto& item : S.places) {
        const int area = item.second.first;
        ParamValues values = item.second.second;
        if (S.weights[P_PROB] > 0 && S.maxValues[P_PROB] > 0) {
            bool dominated = false;
            const std::vector<int>& to1 = S.approaches[area];
            for (const auto& other : S.places) {
                const int oArea = other.second.first;
                if (oArea == area) {
                    continue;
                }
                const std::vector<int>& to2 = S.approaches[oArea];
                if (to1.size() > to2.size()) {
                    if (std::equal(to2.begin(), to2.end(), to1.begin())) {
                        dominated = true;
                        break;
                    }
                } else if (to1 == to2 && paRead(area).endPos > paRead(oArea).endPos) {
                    dominated = true;
                    break;
                }
            }
            if (!dominated) {
                const double prob = values[P_PROB] * rand01v();
                values[P_PROB] = 1.0 - prob / S.maxValues[P_PROB];
            } else {
                values[P_PROB] = 1.0;
            }
        } else {
            values[P_PROB] = 0;
        }
        // getTargetValue
        double cost = 0;
        for (int i = 0; i < NPARAM; ++i) {
            double val = values[i];
            if (i != P_PROB && S.maxValues[i] > 0) {
                val /= S.maxValues[i];
            }
            cost += PARAM_INVERT[i] ? S.weights[i] * (1. - val) : S.weights[i] * val;
        }
        if (nearest < 0 || cost < minCost) {
            minCost = cost;
            nearest = area;
            newRoute = S.newRoutes[nearest];
        }
    }
    x.parkingReroutes++;
    return nearest;
}

// MSBaseVehicle::replaceParkingArea
bool Sim::replaceParkingArea(int v, int area) {
    Veh& x = veh[v];
    if (x.stops.empty() || x.stops.front().parkingArea < 0) {
        return false;
    }
    const ParkingArea& pa = parkingAreas[area];
    Stop& first = x.stops.front();
    const int oldEdge = net.lanes[first.lane].edge;
    // merge subsequent duplicate stops at the new area (SUMO only adds their duration to the stop parameters)
    while (x.stops.size() > 1 && x.stops[1].parkingArea == area) {
        x.stops.erase(x.stops.begin() + 1);
    }
    first.lane = pa.lane;
    first.parkingArea = area;
    first.startPos = pa.begPos;
    first.endPos = pa.endPos;
    // patch via edges
    const int newEdge = net.lanes[pa.lane].edge;
    if (!x.exr().via.empty() && x.exr().via.front() != newEdge) {
        std::vector<int>& via = x.ex().via;
        if (via.front() == oldEdge) {
            via.erase(via.begin());
        }
        via.insert(via.begin(), newEdge);
    }
    return true;
}

// the parking branch of MSTriggeredRerouter::triggerRouting
void Sim::parkingReroute(int v, int ri, const RerouteInterval& def) {
    Veh& x = veh[v];
    // getNextParkingArea
    const int oldArea = !x.stops.empty() && !x.stops.front().reached ? x.stops.front().parkingArea : -1;
    if (oldArea < 0) {
        return;  // not driving towards a parking area
    }
    thread_local RouteWS ws;
    ws.staticEffort = true;  // MSNet::getRouterTT
    ws.prohibited = &def.prohibitions;
    ws.queryTime = STEPS2TIME(now);
    bool newDestination = false;
    std::vector<int> newRoute;
    const int newArea = rerouteStoppingPlace(v, ri, def, oldArea, newDestination, newRoute, ws);
    ws.prohibited = nullptr;
    if (v == traceVeh) {
        std::fprintf(stderr, "%.2f parkingReroute %s rerouter=%s old=%s new=%s newDest=%d route:", STEPS2TIME(now), x.id.c_str(),
                     rerouters[ri].id.c_str(), parkingAreas[oldArea].id.c_str(), newArea < 0 ? "-" : parkingAreas[newArea].id.c_str(),
                     (int)newDestination);
        for (int e : newRoute) {
            std::fprintf(stderr, " %s", net.edges[e].id.c_str());
        }
        std::fprintf(stderr, "\n");
    }
    if (newArea < 0) {
        if (parkingAreas[oldArea].reservable) {
            paAddReservation(parkingAreas[oldArea], v);
        }
        return;
    }
    // adapt the plans of any riders
    for (int pi : x.exr().riders) {
        personRerouteParkingArea(pi, oldArea, newArea);
    }
    if (newDestination && x.arrivalPosMode != AP_MAX) {
        // update the arrival parameters (psim2 does not distinguish the default from "max")
        x.arrivalPosMode = AP_GIVEN;
        x.arrivalPosGiven = parkingAreas[newArea].endPos;
    }
    if (replaceParkingArea(v, newArea)) {
        replaceRouteEdges(v, newRoute, false, rerouteOriginIdx(v), true);
        if (parkingAreas[oldArea].reservable) {
            paRemoveReservation(parkingAreas[oldArea], v);
        }
        if (parkingAreas[newArea].reservable) {
            paAddReservation(parkingAreas[newArea], v);
        }
    }
}

}  // namespace ps

namespace ps {

// MSStoppingPlaceRerouter::evaluateDestination for a bus / train stop (stationReroute)
bool Sim::stationEvaluate(int v, ParkingSearch& S, double brakeGap, int bs, double occupancy, double capacity, double prob,
                          RouteWS& ws) {
    const Veh& x = veh[v];
    const Kin& k = kin[v];
    const BusStop& b = busStops[bs];
    const std::vector<int>& route = routes[x.route];
    const int stopEdge = net.lanes[b.lane].edge;
    const double targetPos = busStopLastFreePos(bs, v);
    const int originIdx = rerouteOriginIdx(v);
    std::vector<int> edgesToStop;
    routeComputePos(route[originIdx], k.pos, stopEdge, targetPos, v, edgesToStop, ws);
    if (edgesToStop.empty()) {
        return false;
    }
    if (route[originIdx] != route[x.routeIdx]) {
        edgesToStop.insert(edgesToStop.begin(), route[x.routeIdx]);
    }
    S.approaches[bs] = edgesToStop;
    // the stop after the replaced one (keepCurrentStop), otherwise the destination
    int nextDestinationIndex = (int)route.size() - 1;
    int nextDestination = route.back();
    double nextPos = x.arrivalPos;
    if (x.stops.size() > 1) {
        nextDestinationIndex = x.stops[1].routeIdx;
        nextDestination = route[nextDestinationIndex];
        nextPos = stopEndPos(v, x.stops[1]);
    }
    std::vector<int> edgesFromStop;
    routeComputePos(stopEdge, targetPos, nextDestination, nextPos, v, edgesFromStop, ws);
    if (edgesFromStop.empty()) {
        return false;
    }
    ParamValues val{};
    val[P_PROB] = prob;
    val[P_CAPACITY] = capacity;
    val[P_ABSFREE] = capacity - occupancy;
    val[P_RELFREE] = val[P_ABSFREE] / std::max(1.0, capacity);
    double toPos = b.startPos;
    if (stopEdge == curEdge(v)) {
        toPos = std::max(k.pos, toPos);
    }
    val[P_DISTTO] = routeDistanceBetween(edgesToStop, k.pos, toPos);
    const double distToEnd = val[P_DISTTO] - toPos + targetPos;
    if (distToEnd < brakeGap) {
        return false;  // too close
    }
    const double correctionLastEdge = (net.lanes[b.lane].length - b.endPos) / laneVehMaxSpeed(b.lane, v);
    const double correctionFirstEdge = k.pos / laneVehMaxSpeed(net.edges[edgesToStop.front()].lanes[0], v);
    val[P_TIMETO] = routeCosts(edgesToStop, v, ws) - correctionLastEdge - correctionFirstEdge;
    std::vector<int> newEdges = edgesToStop;
    val[P_DISTFROM] = routeDistanceBetween(edgesFromStop, b.startPos, net.edges[edgesFromStop.back()].length);
    val[P_TIMEFROM] = routeCosts(edgesFromStop, v, ws) - b.endPos / net.lanes[b.lane].speed;
    newEdges.insert(newEdges.end(), edgesFromStop.begin() + 1, edgesFromStop.end());
    newEdges.insert(newEdges.end(), route.begin() + nextDestinationIndex + 1, route.end());
    for (int i = 0; i < NPARAM; ++i) {
        S.maxValues[i] = std::max(S.maxValues[i], val[i]);
    }
    S.places[b.id] = {bs, val};
    S.newRoutes[bs] = newEdges;
    return true;
}

// MSStoppingPlaceRerouter::rerouteStoppingPlace without a destination stop for the stations of a stationReroute
// (MSTriggeredRerouter: capacity 1, or 0 for the stop that a train will occupy; occupancy: the stopped trains)
int Sim::rerouteStation(int v, int ri, const RerouteInterval& def, const std::vector<int>& blocked, RouteWS& ws) {
    Veh& x = veh[v];
    const Kin& k = kin[v];
    ParkingSearch S;
    S.veh = v;
    S.drawBase = ((uint64_t)step * 4096 + (uint64_t)ri) << 16;
    auto rand01v = [&]() { return rand01(opt.seed, (uint64_t)v, S.drawBase + S.nDraws++, RP_PARKING); };
    const int vehEdge = curEdge(v);
    // (the stops share the vehicle's parking memory, keyed apart from the parking areas)
    auto memKey = [](int bs) { return -2 - bs; };
    auto capacity = [&](int bs) { return std::find(blocked.begin(), blocked.end(), bs) != blocked.end() ? 0. : 1.; };
    auto occupancy = [&](int bs) { return (double)busStops[bs].endPositions.size(); };
    auto edgeOf = [&](int bs) { return net.lanes[busStops[bs].lane].edge; };
    const std::vector<int>& cands = def.stopAlternatives;
    const int stopAnywhere = (int)parkingWeight(v, "anywhere", -1);
    const bool ignoreDest = parkingWeight(v, "ignoreDest", 1) != 0;
    int onTheWay = -1;
    if (stopAnywhere < 0 || stopAnywhere > x.parkingReroutes) {
        // all candidates are visible
        for (int c : cands) {
            if (occupancy(c) >= capacity(c)) {
                rememberBlockedParkingArea(v, memKey(c), edgeOf(c) == vehEdge);
            }
        }
    } else {
        double bestDist = std::numeric_limits<double>::max();
        const double brakeGap = cfBrakeGap(v, k.speed, vt(v).decel, vt(v).tau);
        for (int c : cands) {
            if (edgeOf(c) == vehEdge && occupancy(c) < capacity(c)) {
                const double distToStart = busStops[c].startPos - k.pos;
                const double distToEnd = busStops[c].endPos - k.pos;
                if (distToEnd > brakeGap && distToStart < bestDist) {
                    bestDist = distToStart;
                    onTheWay = c;
                }
            }
        }
    }
    if (!ignoreDest && onTheWay < 0) {
        return -1;
    }
    // collectWeights (the default weight of distanceto is changed for all later vehicles, like SUMO)
    Rerouter& r = rerouters[ri];
    r.distanceWeight = parkingWeight(v, "distance.weight", r.distanceWeight);
    for (int i = 0; i < NPARAM; ++i) {
        S.weights[i] = parkingWeight(v, std::string(PARAM_NAMES[i]) + ".weight", i == P_DISTTO ? r.distanceWeight : 0.);
    }
    S.weights[P_PROB] = parkingWeight(v, "probability.weight", 0.);
    const double brakeGap = cfBrakeGap(v, k.speed, vt(v).decel, 0.);
    if (onTheWay >= 0) {
        return stationEvaluate(v, S, brakeGap, onTheWay, occupancy(onTheWay), capacity(onTheWay), 1, ws) ? onTheWay : -1;
    }
    int numAlternatives = 0;
    for (int c : cands) {
        if (occupancy(c) < capacity(c)) {
            if (stationEvaluate(v, S, brakeGap, c, occupancy(c), capacity(c), 1, ws)) {
                numAlternatives++;
            }
        } else {
            rememberBlockedParkingArea(v, memKey(c), edgeOf(c) == vehEdge);
        }
    }
    if (numAlternatives == 0) {
        // any target, preferring the ones not seen blocked
        std::vector<std::tuple<Time, std::string, int>> order;
        for (int c : cands) {
            Time t = sawBlockedParkingArea(v, memKey(c), true);
            if (t < 0) {
                t = -(Time)(rand01v() * 1000000);
            }
            order.emplace_back(t, busStops[c].id, c);
        }
        std::sort(order.begin(), order.end(), [](const auto& a, const auto& b) {
            return std::get<0>(a) < std::get<0>(b) || (std::get<0>(a) == std::get<0>(b) && std::get<1>(a) < std::get<1>(b));
        });
        for (const auto& item : order) {
            if (stationEvaluate(v, S, brakeGap, std::get<2>(item), 0, capacity(std::get<2>(item)), 1, ws)) {
                break;
            }
        }
    }
    int nearest = -1;
    double minCost = 0;
    for (auto& item : S.places) {
        ParamValues values = item.second.second;
        values[P_PROB] = 0;  // (the probability weight is 0 unless given)
        if (S.weights[P_PROB] > 0 && S.maxValues[P_PROB] > 0) {
            values[P_PROB] = 1.0 - values[P_PROB] * rand01v() / S.maxValues[P_PROB];
        }
        double cost = 0;
        for (int i = 0; i < NPARAM; ++i) {
            double val = values[i];
            if (i != P_PROB && S.maxValues[i] > 0) {
                val /= S.maxValues[i];
            }
            cost += PARAM_INVERT[i] ? S.weights[i] * (1. - val) : S.weights[i] * val;
        }
        if (nearest < 0 || cost < minCost) {
            minCost = cost;
            nearest = item.second.first;
        }
    }
    x.parkingReroutes++;
    return nearest;
}

}  // namespace ps
