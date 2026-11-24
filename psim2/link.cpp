// Right-of-way decisions at links. Ported from SUMO 1.28 MSLink.cpp
// (opened, blockedAtTime, blockedByFoe, getLeaderInfo) and MSVehicle::isLeader,
// without bidirectional lanes and sublane resolution. Pedestrians: persons approaching
// priority crossings, pedestrians on crossings (MSPModel_Interacting::blockedAtDist) and
// the queries of the pedestrian model (ego < 0, vehicles crossing a crossing).
//
// Approach registrations are read from Sim::approaching, which is rebuilt in
// vehicle order after the planning phase (MSEdgeControl::setJunctionApproaches).
// A lane change moves the registrations of the changed links right away
// (MSVehicle::updateDriveItems), a removed vehicle drops all of its own.
#include "model.hpp"

namespace ps {

constexpr Time LOOKAHEAD_TIME = 1000;  // MSLink::myLookaheadTime (jmTimegapMinor default)


Time Sim::leaveTime(int link, Time arrivalTime, double arrivalSpeed, double leaveSpeed, double vehLength) const {
    return arrivalTime == TIME_MAX ? TIME_MAX
           : arrivalTime + TIME2STEPS((net.links[link].length + vehLength) / std::max(0.5 * (arrivalSpeed + leaveSpeed), NUMERICAL_EPS));
}

const Approach* Sim::approachOf(int link, int v) const {
    const std::vector<Approach>& a = approaching[link];
    auto it = std::lower_bound(a.begin(), a.end(), v, [](const Approach& x, int id) { return x.veh < id; });
    return (it != a.end() && it->veh == v) ? &*it : nullptr;
}

// MSLink::setApproaching(veh, getApproaching(veh)) on the new link, then
// removeApproaching on the old one; emplace keeps an existing registration.
// (SUMO would register an invalid record if the old link had none, which only
// happens on looped routes; psim2 registers nothing then.)
void Sim::moveApproach(int v, int from, int to) {
    std::vector<Approach>& a = approaching[from];
    auto it = std::lower_bound(a.begin(), a.end(), v, [](const Approach& x, int id) { return x.veh < id; });
    if (it == a.end() || it->veh != v) {
        return;
    }
    const Approach ap = *it;
    a.erase(it);
    std::vector<Approach>& b = approaching[to];
    if (b.empty()) {
        approachMoved.push_back(to);  // cleared at the next registration
    }
    auto jt = std::lower_bound(b.begin(), b.end(), v, [](const Approach& x, int id) { return x.veh < id; });
    if (jt == b.end() || jt->veh != v) {
        b.insert(jt, ap);
    }
}

// MSVehicle::removeApproachingInformation(myLFLinkLanes)
void Sim::removeApproaches(int v) {
    for (const DriveItem& d : veh[v].lf) {
        if (d.link < 0) {
            continue;
        }
        std::vector<Approach>& a = approaching[d.link];
        auto it = std::lower_bound(a.begin(), a.end(), v, [](const Approach& x, int id) { return x.veh < id; });
        if (it != a.end() && it->veh == v) {
            a.erase(it);
        }
    }
}

bool Sim::opened(int link, Time arrivalTime, double arrivalSpeed, double leaveSpeed, double vehLength, double imp, double decel,
                 Time waitingTime, int ego, bool ignoreRed, std::vector<int>* collectFoes, double dist, int egoPerson,
                 const double* egoPosLat) const {
    const Link& k = net.links[link];
    if (k.haveRed() && !ignoreRed) {
        return false;
    }
    if (k.isCont()) {
        return true;
    }
    const Time lt = leaveTime(link, arrivalTime, arrivalSpeed, leaveSpeed, vehLength);
    const VType* wt = ego >= 0 ? &vt(ego) : nullptr;
    if (latRes > 0 && (!k.sublaneFoeLinks.empty() || !k.sublaneFoeLinks2.empty())) {
        // foes on the same lane with the same target edge (their paths cross)
        const double posLat = egoPosLat != nullptr ? *egoPosLat : ego >= 0 ? KV(ego).posLat : 0;
        const int myIndex = net.lanes[k.lane].index;
        for (int foeLink : k.sublaneFoeLinks) {
            const int foeIndex = net.lanes[net.links[foeLink].lane].index;
            for (const Approach& a : approaching[foeLink]) {
                const double foePosLat = KV(a.veh).posLat;
                if (((posLat < foePosLat + a.latOffset && myIndex > foeIndex) || (posLat > foePosLat + a.latOffset && myIndex < foeIndex))
                        // the vehicle that arrives later must yield (at the same time: the one to the left)
                        && (arrivalTime > a.arrivalTime || (arrivalTime == a.arrivalTime && posLat > foePosLat))
                        && blockedByFoe(foeLink, a, arrivalTime, lt, arrivalSpeed, leaveSpeed, false, imp, decel, waitingTime, wt)) {
                    if (collectFoes == nullptr) {
                        return false;
                    }
                    collectFoes->push_back(a.veh);
                }
            }
        }
        // foes on the same lane with a different target edge (straight movers take precedence if the paths cross)
        const int lhSign = net.lefthand ? -1 : 1;
        const double egoWidth = wt == nullptr ? 1.8 : wt->width;
        for (int foeLink : k.sublaneFoeLinks2) {
            for (const Approach& a : approaching[foeLink]) {
                const double foeLat = KV(a.veh).posLat + a.latOffset;
                if (!(std::fabs(foeLat - posLat) < (egoWidth + vt(a.veh).width) / 2)
                        && (((k.dir == 'r' || k.dir == 'R') && posLat * lhSign > foeLat * lhSign)
                            || ((k.dir == 'l' || k.dir == 'L') && posLat * lhSign < foeLat * lhSign))
                        && blockedByFoe(foeLink, a, arrivalTime, lt, arrivalSpeed, leaveSpeed, false, imp, decel, waitingTime, wt)) {
                    if (collectFoes == nullptr) {
                        return false;
                    }
                    collectFoes->push_back(a.veh);
                }
            }
        }
    }
    if ((k.havePriority() || net.lastWasContState(link, LS_GREEN_MAJOR)
            || (net.isExitLinkAfterInternalJunction(link) && net.links[net.correspondingEntryLink(link)].state == LS_GREEN_MAJOR))
            && k.state != LS_ZIPPER) {
        return collectFoes == nullptr || collectFoes->empty();
    }
    // the waiting time at stop signs (jmAllwayStopWait, jmStopSignWait; one step by default)
    if (k.state == LS_ALLWAY_STOP && waitingTime < TIME2STEPS(wt != nullptr && wt->jmAllwayStopWait >= 0 ? wt->jmAllwayStopWait : TS)) {
        return false;
    } else if (k.state == LS_STOP && waitingTime < TIME2STEPS(wt != nullptr && wt->jmStopSignWait >= 0 ? wt->jmStopSignWait : TS)) {
        return false;
    }
    const bool lastWasContRed = net.lastWasContState(link, LS_RED);
    for (int foe : k.foeLinks) {
        if (blockedAtTime(foe, arrivalTime, lt, arrivalSpeed, leaveSpeed, k.lane == net.links[foe].lane, imp, decel,
                          waitingTime, ego, lastWasContRed, collectFoes, dist, egoPerson)) {
            return false;
        }
    }
    return collectFoes == nullptr || collectFoes->empty();
}

bool Sim::blockedAtTime(int link, Time arrivalTime, Time lt, double arrivalSpeed, double leaveSpeed, bool sameTargetLane,
                        double imp, double decel, Time waitingTime, int ego, bool lastWasContRed,
                        std::vector<int>* collectFoes, double dist, int egoPerson) const {
    const VType* et = ego >= 0 ? &vt(ego) : (egoPerson >= 0 ? &types[persons[egoPerson].type] : nullptr);
    // jmIgnoreFoeProb / jmIgnoreFoeSpeed: whether a foe with the given speed is respected
    auto respects = [&](double foeSpeed, uint64_t foeKey) {
        if (et == nullptr || et->jmIgnoreFoeProb == 0 || et->jmIgnoreFoeSpeed < foeSpeed) {
            return true;
        }
        const uint64_t egoKey = ego >= 0 ? (uint64_t)ego : (1ULL << 40) + (uint64_t)egoPerson;
        return et->jmIgnoreFoeProb < rand01(opt.seed, egoKey * 1000003ULL + foeKey, (uint64_t)step, RP_JM_IGNORE);
    };
    for (const Approach& a : approaching[link]) {
        // SUMO reads the foe's current speed, which during the (sequential) move
        // phase may already be updated; in parallel the speed of the registration
        // keeps the result independent of the processing order
        const double foeSpeed = opt.sumoOrder ? kin[a.veh].speed : a.speed;
        if (a.veh != ego
                && respects(a.speed, (uint64_t)a.veh)
                && !jmIgnoresFoe(ego, a.veh)
                && (!lastWasContRed || foeSpeed > HALTING_SPEED)
                && blockedByFoe(link, a, arrivalTime, lt, arrivalSpeed, leaveSpeed, sameTargetLane, imp, decel, waitingTime, et)) {
            if (collectFoes == nullptr) {
                return true;
            }
            collectFoes->push_back(a.veh);
        }
    }
    // persons approaching a priority crossing
    if (!approachingPersons.empty() && !approachingPersons[link].empty() && !net.links[link].haveRed()) {
        const Time lookAhead = et == nullptr ? LOOKAHEAD_TIME : TIME2STEPS(et->jmTimegapMinor);
        for (const PersonApproach& pa : approachingPersons[link]) {
            const PStage& ps = persons[pa.person].plan[persons[pa.person].step];
            const double personSpeed = ps.pstate >= 0 ? pstates[ps.pstate].speed : 0;
            if (respects(personSpeed, (1ULL << 41) + (uint64_t)pa.person)
                    && !jmIgnoresFoe(ego, -1, pa.person)
                    && !((arrivalTime > pa.leavingTime + lookAhead) || (lt + lookAhead < pa.arrivalTime))) {
                if (ego < 0) {
                    // during insertion
                    if (net.junctions[net.links[link].junction].type == "rail_crossing") {
                        continue;
                    }
                    return true;
                }
                // check whether braking is feasible (ego might have started to accelerate already)
                if (dist > cfBrakeGap(ego, kin[ego].speed, vt(ego).decel, 0) && collectFoes == nullptr) {
                    return true;
                }
            }
        }
    }
    return false;
}

// MSVehicle::ignoreFoe: carFollowModel.ignoreIDs / ignoreTypes of the vehicle
bool Sim::cfIgnoresFoe(int v, int foe) const {
    const VehExtra* e = veh[v].extra.p.get();
    if (e == nullptr || foe < 0 || (e->cfIgnoreIDs.empty() && e->cfIgnoreTypes.empty())) {
        return false;
    }
    return std::find(e->cfIgnoreTypes.begin(), e->cfIgnoreTypes.end(), vt(foe).id) != e->cfIgnoreTypes.end()
           || std::find(e->cfIgnoreIDs.begin(), e->cfIgnoreIDs.end(), veh[foe].id) != e->cfIgnoreIDs.end();
}

// MSLink::ignoreFoe: junctionModel.ignoreIDs / ignoreTypes of the vehicle ego for a vehicle or a person
bool Sim::jmIgnoresFoe(int ego, int foeVeh, int foePerson) const {
    const VehExtra* e = ego >= 0 ? veh[ego].extra.p.get() : nullptr;
    if (e == nullptr || (e->jmIgnoreIDs.empty() && e->jmIgnoreTypes.empty())) {
        return false;
    }
    const std::string& type = foeVeh >= 0 ? vt(foeVeh).id : types[persons[foePerson].type].id;
    const std::string& id = foeVeh >= 0 ? veh[foeVeh].id : persons[foePerson].id;
    return std::find(e->jmIgnoreTypes.begin(), e->jmIgnoreTypes.end(), type) != e->jmIgnoreTypes.end()
           || std::find(e->jmIgnoreIDs.begin(), e->jmIgnoreIDs.end(), id) != e->jmIgnoreIDs.end();
}

// MSPModel_Interacting::blockedAtDist: a pedestrian on the crossing (foe lane) that is not past the crossing point
bool Sim::blockedAtDist(int lane, double vehCenter, double vehWidth, double oncomingGap, int ego) const {
    for (int psi : pedsOnLane[lane]) {
        const PState& p = pstates[psi];
        const double leaderFrontDist = (p.dir == PED_FORWARD ? vehCenter - p.edgePos : p.edgePos - vehCenter) - vehWidth * 0.5;
        const double leaderBackDist = leaderFrontDist + types[persons[p.person].type].length;
        if (leaderBackDist >= -vehWidth
                && (leaderFrontDist < 0
                    // give right of way to (close) approaching pedestrians unless they are standing
                    || (leaderFrontDist <= oncomingGap && p.waitingTime < TIME2STEPS(2.0)))) {
            if (jmIgnoresFoe(ego, -1, p.person)) {
                continue;
            }
            return true;
        }
    }
    return false;
}

// MSPModel_Interacting::nextBlocking: the closest pedestrian ahead on the lane overlapping [minRight, maxLeft]
// (person, distance); person -1 if none
std::pair<int, double> Sim::nextBlocking(int lane, double minPos, double minRight, double maxLeft, double stopTime) const {
    std::pair<int, double> result(-1, DMAX);
    for (int psi : pedsOnLane[lane]) {
        const PState& p = pstates[psi];
        const VType& t = types[persons[p.person].type];
        // account for distance covered by oncoming pedestrians
        const double relX2 = p.edgePos - (p.dir == PED_FORWARD ? 0 : stopTime * personMaxSpeed(p.person));
        const double dist = (relX2 - minPos) - (p.dir == PED_FORWARD ? t.length : 0);
        const bool aheadOfVehicle = p.edgePos > minPos;
        if (aheadOfVehicle && dist < result.second) {
            // PState::getLatOffset (posLatConversion) + half the lane width
            const double center = 0.5 * (net.lanes[lane].width - striping.stripeWidth) - p.posLat + 0.5 * net.lanes[lane].width;
            const double halfWidth = 0.5 * t.width;
            if (center + halfWidth > minRight && center - halfWidth < maxLeft) {
                result.first = p.person;
                result.second = dist;
            }
        }
    }
    return result;
}

bool Sim::hasPedestrians(int lane) const {
    return !pedsOnLane.empty() && !pedsOnLane[lane].empty();
}

// MSLink::couldBrakeForLeader
static inline bool couldBrakeForLeader(double followDist, double leaderDist, double followSpeed, double followDecel,
                                       double leaderSpeed) {
    return followDist > leaderDist && followDist - leaderDist > followSpeed - followDecel - leaderSpeed;
}

// MSCFModel::avoidArrivalAccel
static double avoidArrivalAccel(double dist, double time, double speed, double maxDecel) {
    if (dist <= 0) {
        return -maxDecel;
    } else if (time * speed > 2 * dist) {
        return -0.5 * speed * speed / dist;
    }
    return 2 * (dist / time - speed) / time;
}

// MSLink::getZipperSpeed. SUMO reads the foes' current speed and lane, which
// in its sequential move phase may already be updated; in parallel the values
// of the approach registration are used (with --sumo-order the current ones).
double Sim::zipperSpeed(int ego, int link, double dist, double vSafe, Time arrivalTime, const std::vector<int>& foes) const {
    const Link& k = net.links[link];
    if (k.foeLinks.empty()) {
        return vSafe;
    }
    const VType& t = vt(ego);
    const double egoSpeed = kin[ego].speed;
    if (dist > std::max(k.visibility, cfBrakeGap(ego, vSafe, t.decel, TS))) {
        return vSafe;
    }
    for (int foe : foes) {
        const Approach* a = nullptr;
        for (int fl : k.foeLinks) {
            a = approachOf(fl, foe);
            if (a != nullptr) {
                break;
            }
        }
        if (a == nullptr) {
            continue;
        }
        const VType& ft = vt(foe);
        // (the registration of a foe between its actions is older: its distance is extrapolated)
        const double foeDist = veh[foe].actionStep ? a->dist
                               : std::max(0.0, a->dist - STEPS2TIME(now - veh[foe].lastActionTime) * a->speed);
        const double foeSpeed = opt.sumoOrder ? kin[foe].speed : a->speed;
        const int foeLane = opt.sumoOrder ? kin[foe].lane : a->lane;
        if ((a->arrivalTime > arrivalTime && !couldBrakeForLeader(dist, foeDist, egoSpeed, t.decel, foeSpeed))
                || couldBrakeForLeader(foeDist, dist, foeSpeed, ft.decel, egoSpeed)
                || (a->arrivalTime == arrivalTime && foeDist == dist && net.lanes[kin[ego].lane].index < net.lanes[foeLane].index)) {
            continue;
        }
        const double uMax = laneVehMaxSpeed(foeLane, foe);
        const double uAccel = cf::estimateSpeedAfterDistance(ft, foeDist, a->speed, ft.accel);
        const double uEnd = std::min(uMax, uAccel);
        const double uAvg = (a->speed + uEnd) / 2;
        const double tf = std::max(1.0, std::ceil(foeDist / std::max(NUMERICAL_EPS, uAvg) / TS) * TS);
        const double vMax = laneVehMaxSpeed(kin[ego].lane, ego);
        const double vAccel = cf::estimateSpeedAfterDistance(t, dist, egoSpeed, t.accel);
        const double vDecel = cf::estimateSpeedAfterDistance(t, dist, egoSpeed, -t.decel);
        const double vEnd = std::min(std::min(vMax, vAccel), std::max(uEnd, vDecel));
        const double vAvg = (egoSpeed + vEnd) / 2;
        const double te = std::max(1.0, std::ceil(dist / std::max(NUMERICAL_EPS, vAvg) / TS) * TS);
        const double acc = avoidArrivalAccel(dist, tf + t.tau, vSafe, t.decel);
        const double gap = dist - ft.length - t.minGap - foeDist;
        const double vFollow = cfFollowSpeed(ego, egoSpeed, gap, a->speed, ft.decel, foe);
        const double vSafeGap = std::max(vFollow, egoSpeed + ACCEL2SPEED(acc));
        const double w = std::min(1.0, te / 10);
        const double maxDecel = w * t.decel + (1 - w) * t.emergencyDecel;
        const double vZipper = std::max(std::max(vFollow, egoSpeed - ACCEL2SPEED(maxDecel)), vSafeGap);
        vSafe = std::min(vSafe, vZipper);
    }
    return vSafe;
}

static inline bool unsafeMergeSpeeds(double leaderSpeed, double followerSpeed, double leaderDecel, double followerDecel) {
    return (leaderSpeed * leaderSpeed / leaderDecel) <= (followerSpeed * followerSpeed / followerDecel);
}

// MSLink::computeFoeArrivalTimeBraking
static Time computeFoeArrivalTimeBraking(Time arrivalTime, double foeMaxDecel, Time foeArrivalTime, double imp, double dist,
                                         Time now, double& fasb) {
    if (arrivalTime - arrivalTime % DELTA_T == foeArrivalTime - foeArrivalTime % DELTA_T) {
        return foeArrivalTime;
    }
    if (arrivalTime % DELTA_T > 0) {
        arrivalTime = arrivalTime - (arrivalTime % DELTA_T) + DELTA_T;
    }
    const double m = foeMaxDecel * imp;
    const double dt = STEPS2TIME(foeArrivalTime - arrivalTime);
    const double d = dt * m;
    const double a = dt * d / 2;
    const double v = dist / STEPS2TIME(foeArrivalTime - now + DELTA_T);
    const double dist2 = dist - v * STEPS2TIME(arrivalTime - now);
    if (0.5 * v * v / m <= dist2) {
        fasb = 0;
        return foeArrivalTime + TIME2STEPS(30);
    }
    const double x = (std::sqrt(4 * (v - d) * (v - d) - 8 * m * a) * -0.5 - d + v) / m;
    fasb = v - (dt + x) * m;
    return foeArrivalTime + TIME2STEPS(x);
}

bool Sim::blockedByFoe(int link, const Approach& a, Time arrivalTime, Time lt, double arrivalSpeed, double leaveSpeed,
                       bool sameTargetLane, double imp, double decel, Time waitingTime, const VType* et) const {
    const Link& k = net.links[link];
    if (!a.willPass) {
        return false;
    }
    if (k.state == LS_ALLWAY_STOP) {
        // (with action steps the registration of the foe may be outdated)
        const Time actionDelta = now - veh[a.veh].lastActionTime;
        if (waitingTime > a.waitingTime + actionDelta) {
            return false;
        }
        if (waitingTime == a.waitingTime + actionDelta && arrivalTime < a.arrivalTime + actionDelta) {
            return false;
        }
    }
    Time foeArrivalTime = a.arrivalTime;
    double foeArrivalSpeedBraking = a.arrivalSpeedBraking;
    const double foeDecel = types[kin[a.veh].type].decel;
    if (imp > 0 && arrivalTime < a.arrivalTime) {
        const Time fatb = computeFoeArrivalTimeBraking(arrivalTime, foeDecel, a.arrivalTime, imp, a.dist, now, foeArrivalSpeedBraking);
        foeArrivalTime = (Time)((1. - imp) * (double)a.arrivalTime + imp * (double)fatb);
    }
    const Time lookAhead = k.state == LS_ZIPPER ? 16000 : (et == nullptr ? LOOKAHEAD_TIME : TIME2STEPS(et->jmTimegapMinor));
    if (a.leavingTime < arrivalTime) {
        // ego wants to be follower
        if (sameTargetLane && (arrivalTime - a.leavingTime < lookAhead || unsafeMergeSpeeds(a.leaveSpeed, arrivalSpeed, foeDecel, decel))) {
            return true;
        }
    } else if (foeArrivalTime > lt + lookAhead) {
        // ego wants to be leader
        if (sameTargetLane && unsafeMergeSpeeds(leaveSpeed, foeArrivalSpeedBraking, decel, foeDecel)) {
            return true;
        }
    } else {
        return true;
    }
    return false;
}

bool Sim::hasApproachingFoe(int link, Time arrivalTime, Time lt, double speed, double decel) const {
    const Link& k = net.links[link];
    for (int foe : k.foeLinks) {
        if (blockedAtTime(foe, arrivalTime, lt, speed, speed, k.lane == net.links[foe].lane, 0, decel, 0, -1, false)) {
            return true;
        }
    }
    for (int lane : k.foeLanes) {
        if (!lanes[lane].vehs.empty() || !lanes[lane].partials.empty()) {
            return true;
        }
    }
    return false;
}

std::pair<int, int> Sim::firstApproachingFoe(int link, int wrapAround) const {
    double closest = DMAX;
    int closestVeh = -1, foeLink = -1;
    for (int l : net.links[link].foeLinks) {
        for (const Approach& a : approaching[l]) {
            if (net.links[l].laneBefore == net.links[wrapAround].laneBefore) {
                return {-1, wrapAround};
            } else if (a.dist < closest) {
                closest = a.dist;
                if (a.willPass) {
                    closestVeh = a.veh;
                    foeLink = l;
                }
            }
        }
    }
    return {closestVeh, foeLink};
}

// MSLink::getLeaderInfo. ego < 0: query of the pedestrian model (vehicles approaching the crossing point of
// a crossing's exit link; gap = distance of the vehicle's back to the crossing point). Pedestrians found
// for a vehicle are returned with veh = -1.
// the order of SUMO's LinkDirection enum (STRAIGHT, TURN, TURN_LEFTHAND, LEFT, RIGHT, PARTLEFT, PARTRIGHT, NODIR)
static int dirRank(char dir) {
    switch (dir) {
        case 's': return 0;
        case 't': return 1;
        case 'T': return 2;
        case 'l': return 3;
        case 'r': return 4;
        case 'L': return 5;
        case 'R': return 6;
        default: return 7;
    }
}

void Sim::leaderInfo(int link, int ego, double dist, std::vector<LinkLeader>& result, bool computeLC, bool isShadowLink) const {
    result.clear();
    const Link& k = net.links[link];
    if (ego >= 0 && !k.fromInternalLane()) {
        return;
    }
    const VType* et = ego >= 0 ? &vt(ego) : nullptr;
    if (computeLC && ego >= 0 && !isInternal(KV(ego).lane)) {
        const int junctionEntry = net.entryLink(k.laneBefore);
        if (net.links[junctionEntry].haveRed()) {
            return;
        }
    }
    const bool thisInternalJunctionLink = k.isInternalJunctionLink();
    const bool exitAfterInternalJunction = net.isExitLinkAfterInternalJunction(link);
    const bool withPersons = ego >= 0 && !persons.empty();
    for (int i = 0; i < (int)k.foeLanes.size(); ++i) {
        const int foeLane = k.foeLanes[i];
        // nothing to find on an empty foe lane (most foe lanes are)
        if (lanes[foeLane].vehs.empty() && lanes[foeLane].partials.empty()
                && (!withPersons || (pedsOnLane[foeLane].empty()
                                     && (!net.lanes[foeLane].crossing || approachingPersons[net.lanes[foeLane].incoming[0].link].empty())))) {
            continue;
        }
        const int foeExitLink = net.lanes[foeLane].links[0];
        const Link& fe = net.links[foeExitLink];
        double distToCrossing = dist - net.conflictLengthBehindCrossing(link, i);
        const double foeDistToCrossing = net.lanes[foeLane].length - net.conflictFoeLengthBehindCrossing(link, i);
        const bool sameTarget = k.lane == fe.lane && !thisInternalJunctionLink && !fe.isInternalJunctionLink();
        const bool sameSource = k.internalLaneBefore >= 0
                                && net.normalPredecessorLane(k.internalLaneBefore) == net.normalPredecessorLane(foeLane);
        const double crossingWidth = (sameTarget || sameSource) ? 0 : k.conflicts[i].conflictSize;
        const double foeCrossingWidth = (sameTarget || sameSource) ? 0 : net.conflictFoeSize(link, i);
        const bool contLane = isInternal(fe.viaOrLane()) && !(thisInternalJunctionLink || exitAfterInternalJunction);
        if (distToCrossing + crossingWidth < 0 && !sameTarget
                && (ego < 0 || !computeLC || distToCrossing + crossingWidth + et->length < 0)) {
            continue;
        }
        bool ignoreGreenCont = false;
        bool foeIndirect = false;
        if (contLane) {
            const int entry = net.entryLink(k.laneBefore);
            const int foeEntry = net.entryLink(foeLane);
            foeIndirect = net.links[foeEntry].indirect;
            if (entry >= 0 && net.links[entry].haveGreen() && foeEntry >= 0 && net.links[foeEntry].haveGreen()
                    && net.links[entry].laneBefore != net.links[foeEntry].laneBefore) {
                ignoreGreenCont = true;
            }
        }
        if (foeIndirect && distToCrossing >= NO_INTERSECTION) {
            continue;
        }
        const double l2 = ego >= 0 ? et->length + 2 : 0;
        const double sagitta = ego >= 0 && k.radius != DMAX ? k.radius - std::sqrt(k.radius * k.radius - 0.25 * l2 * l2) : 0;
        anyVehicles(foeLane, [&](int leader) {
            const VType& lt = vt(leader);
            const double leaderBack = backPositionOnLane(leader, foeLane);
            const double leaderBackDist = foeDistToCrossing - leaderBack;
            const bool pastTheCrossingPoint = leaderBackDist + foeCrossingWidth + sagitta < 0;
            const bool enteredTheCrossingPoint = leaderBackDist < lt.length;
            // a left-turning bicycle past the crossing point is part of an indirect turn
            const bool foeIsBicycleTurn = lt.vclass == VC_BICYCLE && net.links[net.lanes[foeLane].incoming.front().link].dir == 'l';
            const bool ignoreIndirectBicycleTurn = pastTheCrossingPoint && foeIsBicycleTurn;
            const bool cannotIgnore = ((contLane && !ignoreIndirectBicycleTurn) || sameTarget || (sameSource && !computeLC)) && ego >= 0;
            const bool inTheWay = ((!pastTheCrossingPoint && distToCrossing > 0) || (sameTarget && distToCrossing > leaderBackDist - lt.length))
                                  && (enteredTheCrossingPoint || (sameSource && !enteredTheCrossingPoint && foeDistToCrossing < distToCrossing))
                                  && (!(k.conflicts[i].flag == CONFLICT_DUMMY_MERGE) || foeIsBicycleTurn || sameSource);
            const Approach* avi = approachOf(foeExitLink, leader);
            const bool willPass = (avi != nullptr && avi->willPass) || (avi == nullptr && sameTarget);
            if (leader == ego) {
                return true;
            }
            if (!inTheWay && ignoreGreenCont) {
                return true;
            }
            if ((!computeLC || KV(ego).lane == foeLane) && distToCrossing < -POSITION_EPS && !inTheWay
                    && (ego < 0 || !computeLC || distToCrossing < -et->length)) {
                return true;
            }
            if (!computeLC && sameSource && ego >= 0 && net.lanes[KV(ego).lane].edge == net.lanes[k.internalLaneBefore].edge
                    && leaderBack + lt.length < KV(ego).pos - et->length) {
                return true;
            }
            if ((!cannotIgnore || isStopped(leader) || sameTarget) && !willPass
                    && (avi == nullptr || KV(leader).speed < HALTING_SPEED)
                    && isFrontOnLane(leader, foeLane) && !inTheWay && !willStop(leader)) {
                return true;
            }
            if (sublane && ego >= 0 && (sameSource || sameTarget)) {
                // check whether the foe is blocked and might need to change before leaving the junction
                const bool foeStrategicBlocked = strategicBlockedOf(leader)
                                                 && cfBrakeGap(leader, KV(leader).speed) <= net.lanes[foeLane].length - leaderBack;
                const bool sameInternalEdge = net.lanes[k.internalLaneBefore].edge == net.lanes[fe.internalLaneBefore].edge;
                if (!foeStrategicBlocked || sameInternalEdge) {
                    if (KV(ego).lane == KV(leader).lane) {
                        return true;
                    }
                    // ignore vehicles that are not in conflict sublane-wise
                    const double egoLatOffset = isShadowLink ? latOffset(ego, KV(ego).shadowLane) : 0;
                    const double posLat = KV(ego).posLat + egoLatOffset;
                    const double posLatLeader = KV(leader).posLat + latOffset(leader, foeLane);
                    const double latGap = std::fabs(posLat - posLatLeader) - 0.5 * (et->width + lt.width);
                    const double maneuverDist = maneuverDistOf(leader) * (posLat < posLatLeader ? -1 : 1);
                    if (latGap > 0 && (latGap > maneuverDist || !sameTarget || !computeLC)
                            // do not perform sublane changes that interfere with the leader vehicle
                            && (!computeLC || latGap > et->maxSpeedLat)) {
                        const Link& foeEntryLink = net.links[net.lanes[foeLane].incoming.front().link];
                        if (sameSource) {
                            // for lanes from the same edge, a higher index implies a connection further to the left
                            const bool leaderFromRight = k.index > foeEntryLink.index;
                            if ((posLat > posLatLeader) == leaderFromRight) {
                                return true;  // lanes diverge
                            }
                        } else if (sameTarget) {
                            if (k.dir != foeEntryLink.dir) {
                                bool leaderFromRight = dirRank(foeEntryLink.dir) < dirRank(k.dir);
                                if (net.lefthand) {
                                    leaderFromRight = !leaderFromRight;
                                }
                                const double leaderSpeedLat = speedLatOf(leader);
                                const double egoSpeedLat = speedLatOf(ego);
                                if ((posLat > posLatLeader) == leaderFromRight
                                        // leader should keep lateral position or move away from ego
                                        && (leaderSpeedLat == 0 || maneuverDistOf(leader) == 0 || leaderFromRight == (leaderSpeedLat < latGap))
                                        && (egoSpeedLat == 0 || maneuverDistOf(ego) == 0 || leaderFromRight == (egoSpeedLat > -latGap))) {
                                    return true;
                                }
                            }
                        } else {
                            return true;
                        }
                    }
                }
            }
            if (opt.ignoreJunctionBlocker < 0 || veh[leader].waitingTime < TIME2STEPS(opt.ignoreJunctionBlocker)) {
                double gap;
                bool fromLeft = true;
                if (ego < 0) {
                    // request from the pedestrian model: distance between the leader's back and the crossing point
                    gap = leaderBackDist;
                    // (SUMO adds this for every vehicle on the foe lane)
                    distToCrossing += k.conflicts[i].conflictSize / 2;
                    if (gap + foeCrossingWidth < 0) {
                        return true;
                    }
                    // whether the vehicle passes the crossing from the left or the right (heuristic)
                    fromLeft = foeDistToCrossing > 0.5 * net.lanes[foeLane].length;
                } else if (contLane && !sameSource && !ignoreIndirectBicycleTurn) {
                    gap = -DMAX;
                } else {
                    if (pastTheCrossingPoint && !sameTarget) {
                        return true;
                    }
                    double leaderBackDist2 = leaderBackDist;
                    if (sameTarget && leaderBackDist2 < 0) {
                        const double mismatch = net.conflictFoeLengthBehindCrossing(link, i) - net.conflictLengthBehindCrossing(link, i);
                        if (mismatch > 0) {
                            leaderBackDist2 += mismatch;
                        }
                    }
                    gap = distToCrossing - et->minGap - leaderBackDist2 - foeCrossingWidth;
                }
                const bool stopAsap = isFrontOnLane(leader, foeLane) ? cannotIgnore : (sameTarget || sameSource);
                const int flags = (fromLeft ? LL_FROM_LEFT : 0) | (inTheWay ? LL_IN_THE_WAY : 0) | (sameSource ? LL_SAME_SOURCE : 0)
                                  | (sameTarget ? LL_SAME_TARGET : 0);
                if (!jmIgnoresFoe(ego, leader)) {
                    result.push_back({leader, gap, stopAsap ? -1 : distToCrossing, flags, sublane ? latOffset(leader, foeLane) : 0.});
                }
            }
            return true;
        });
        if (ego >= 0 && !persons.empty()) {
            // crossing pedestrians (keep driving if already on top of the crossing)
            const double distToPeds = distToCrossing - et->jmStoplineCrossingGap;
            const double vehWidth = et->width + PED_SAFETY_GAP;
            const double vehCenter = foeDistToCrossing + net.lanes[k.laneBefore].width * 0.5;  // lateral position 0
            if (distToPeds >= -PED_SAFETY_GAP && blockedAtDist(foeLane, vehCenter, vehWidth, et->jmCrossingGap, ego)) {
                result.push_back({-1, -1, distToPeds, 0});
            } else if (net.lanes[foeLane].crossing && isInternal(KV(ego).lane)
                       && net.edges[net.lanes[KV(ego).lane].edge].toJ == k.junction) {
                const int crossingLink = net.lanes[foeLane].incoming[0].link;
                if (distToCrossing > 0 && net.links[crossingLink].havePriority() && !approachingPersons[crossingLink].empty()) {
                    // a person might step on the crossing at any moment, since ego is already on the junction,
                    // the opened() check is not done anymore
                    const double timeToEnterCrossing = distToCrossing / std::max(KV(ego).speed, 1.0);
                    for (const PersonApproach& pa : approachingPersons[crossingLink]) {
                        if (!jmIgnoresFoe(ego, -1, pa.person) && timeToEnterCrossing > STEPS2TIME(pa.arrivalTime - now)) {
                            result.push_back({-1, -1, distToPeds, 0});
                            break;
                        }
                    }
                }
            }
        }
    }
    if (ego >= 0 && !persons.empty()) {
        checkWalkingAreaFoe(ego, link, k.walkingAreaFoe, result);
        checkWalkingAreaFoe(ego, link, k.walkingAreaFoeExit, result);
    }
    if (latRes > 0 && ego >= 0 && !isShadowLink) {
        // foes on the same edge whose paths cross
        for (int foeLane : k.sublaneFoeLanes) {
            anyVehicles(foeLane, [&](int leader) {
                if (leader == ego || !isInternal(KV(leader).lane)) {
                    return true;  // (a leader on a normal lane is past the conflict point)
                }
                const double maxLength = std::max(net.lanes[k.internalLaneBefore].length, net.lanes[foeLane].length);
                const double gap = dist - maxLength - et->minGap + backPositionOnLane(leader, foeLane);
                if (gap < -(et->minGap + vt(leader).length)) {
                    return true;  // ego is ahead of the leader
                }
                const double posLat = KV(ego).posLat;
                const double posLatLeader = KV(leader).posLat + latOffset(leader, foeLane);
                if (((posLat < posLatLeader && net.lanes[k.internalLaneBefore].index > net.lanes[foeLane].index)
                        || (posLat > posLatLeader && net.lanes[k.internalLaneBefore].index < net.lanes[foeLane].index))
                        && !jmIgnoresFoe(ego, leader)) {
                    result.push_back({leader, gap, -1, LL_SAME_SOURCE});
                }
                return true;
            });
        }
    }
}

// MSLink::checkWalkingAreaFoe: pedestrians may be on an arbitrary path across the walking area that the vehicle
// crosses (shared space), keep enough distance
void Sim::checkWalkingAreaFoe(int ego, int link, int foeLane, std::vector<LinkLeader>& result) const {
    if (foeLane < 0 || !edgeHasPersons(net.lanes[foeLane].edge)) {
        return;
    }
    const VType& et = vt(ego);
    const Shape& egoPath = net.lanes[net.links[link].internalLaneBefore].shape;  // lateral position 0
    const Pos egoPos = vehPosition(ego);
    const double egoAngle = vehAngle(ego);
    // MSLink::isInFront
    auto isInFront = [&](const Pos& pPos) {
        const double angleDiff = std::fabs(geom::angleDiff(egoAngle, egoPos.angleTo2D(pPos)));
        return angleDiff < geom::rad(75) && geom::distance2D(egoPath, pPos, false) < et.width + PED_SAFETY_GAP;
    };
    double distToPeds = DMAX;
    for (int psi : pedsOnLane[foeLane]) {
        const PState& p = pstates[psi];
        const Pos pPos = pedPosition(p);
        const double pAngle = pedAngle(p);
        const double maxSpeed = personMaxSpeed(p.person);
        // MSLink::getFuturePosition (time horizon 1 s)
        const Pos future{pPos.x + std::cos(pAngle) * maxSpeed, pPos.y + std::sin(pAngle) * maxSpeed};
        double dist = egoPos.distanceTo2D(pPos) - types[persons[p.person].type].length;
        const bool inFront = isInFront(pPos) || isInFront(future);
        if (inFront) {
            dist -= std::max(et.minGap, PED_SAFETY_GAP);
        }
        if (dist < et.width / 2 || inFront) {
            if (jmIgnoresFoe(ego, -1, p.person)) {
                continue;
            }
            if (inFront) {
                // MSLink::isOnComingPed
                const double angleDiff = std::fabs(geom::angleDiff(pAngle, pPos.angleTo2D(egoPos)));
                const double oncomingFactor = angleDiff <= geom::rad(90) ? std::cos(angleDiff) : 0;
                if (oncomingFactor > 0) {
                    // account for pedestrian movement while closing in
                    const double timeToStop = std::sqrt(dist) / 2;
                    const double pedDist = maxSpeed * std::max(timeToStop, TS) * oncomingFactor;
                    dist = std::max(0.0, dist - pedDist);
                }
            }
            distToPeds = std::min(distToPeds, dist);
        }
    }
    if (distToPeds != DMAX) {
        // leave extra space in front
        result.push_back({-1, -1, distToPeds, 0});
    }
}

// MSVehicle::isLeader
bool Sim::isLeader(int ego, int link, int other, double gap) const {
    const Link& k = net.links[link];
    if (other < 0) {
        return false;
    }
    const int egoLane = KV(ego).lane;
    if (!isInternal(egoLane) || net.edges[net.lanes[egoLane].edge].toJ != k.junction) {
        return true;
    }
    const int foeLane = KV(other).lane;
    if (!isInternal(foeLane) || net.edges[net.lanes[foeLane].edge].fromJ != k.junction) {
        return true;
    }
    const Veh& e = veh[ego];
    const Veh& o = veh[other];
    Time egoET = e.junctionConflictEntryTime;
    Time foeET = o.junctionEntryTime;
    if (net.normalPredecessorLane(foeLane) == net.normalPredecessorLane(k.internalLaneBefore)) {
        egoET = e.junctionEntryTimeNeverYield;
        foeET = o.junctionEntryTimeNeverYield;
        if (net.isExitLinkAfterInternalJunction(link)
                && net.links[net.entryLink(net.lanes[k.internalLaneBefore].logicalPred)].indirect) {
            egoET = e.junctionConflictEntryTime;
        }
    } else {
        const int foeLink = net.lanes[foeLane].incoming[0].link;
        const Junction& J = net.junctions[k.junction];
        bool response, response2;
        const Link& entry = net.links[net.correspondingEntryLink(link)];
        const Link& foeEntry = net.links[net.correspondingEntryLink(foeLink)];
        if (entry.haveRed() || foeEntry.haveRed()) {
            if (!foeEntry.haveRed() && KV(other).speed > HALTING_SPEED && gap < 0) {
                const VType& ot = vt(other);
                const double foeNextSpeed = KV(other).speed + ACCEL2SPEED(ot.accel);
                const double foeBrakeGap = cfBrakeGap(other, foeNextSpeed, ot.decel, ot.tau);
                const double foeGap = -gap - ot.length - 2 * vt(ego).minGap;
                if (foeGap < foeBrakeGap) {
                    response = true;
                    response2 = false;
                } else {
                    response = false;
                    response2 = true;
                }
            } else {
                response = true;
                response2 = true;
            }
        } else if (entry.havePriority() != foeEntry.havePriority()) {
            response = !entry.havePriority();
            response2 = !foeEntry.havePriority();
        } else if (entry.haveYellow() && foeEntry.haveYellow()) {
            response = KV(other).speed >= KV(ego).speed;
            response2 = KV(ego).speed >= KV(other).speed;
        } else {
            response = J.responseTest(k.index, net.links[foeLink].index);
            response2 = J.responseTest(net.links[foeLink].index, k.index);
        }
        if (!response) {
            foeET = o.junctionConflictEntryTime;
            egoET = e.junctionEntryTime;
        } else if (response && response2) {
            foeET = o.junctionConflictEntryTime;
            egoET = e.junctionConflictEntryTime;
        }
    }
    if (egoET == foeET) {
        if (KV(ego).speed == KV(other).speed) {
            return e.id < o.id;
        }
        return KV(ego).speed < KV(other).speed;
    }
    return egoET > foeET;
}

}  // namespace ps
