// Lane changer shared by the LC2013 sweep (lc.cpp) and the sublane changer (sublane.cpp).
// Ported from SUMO 1.28 (MSLaneChanger.cpp, MSLCM_LC2013.cpp, MSAbstractLaneChangeModel.cpp, MSLCHelper.cpp).
// (see lc.cpp for the execution structure)
#pragma once

#include <cstdio>
#include <functional>

#include "model.hpp"

namespace ps {


namespace {
constexpr double LOOK_FORWARD = 10.;
constexpr double JAM_FACTOR = 1.;
constexpr double LCA_RIGHT_IMPATIENCE = -1.;
constexpr double CUT_IN_LEFT_SPEED_THRESHOLD = 27.;
constexpr double LOOK_AHEAD_MIN_SPEED = 0.0;
constexpr double LOOK_AHEAD_SPEED_MEMORY = 0.9;
constexpr double HELP_DECEL_FACTOR = 1.0;
constexpr double HELP_OVERTAKE = (10.0 / 3.6);
constexpr double MIN_FALLBEHIND = (7.0 / 3.6);
constexpr double RELGAIN_NORMALIZATION_MIN_SPEED = 10.0;
constexpr double URGENCY = 2.0;
constexpr double KEEP_RIGHT_TIME = 5.0;
constexpr double MAX_ONRAMP_LENGTH = 200.;
constexpr double TURN_LANE_DIST = 200.0;
constexpr double REACT_TO_STOPPED_DISTANCE = 100;
constexpr double BLOCKER_IS_BLOCKED_TIME_THRESHOLD = 5;
constexpr double HYST_PRECISION = 10000000;
}  // namespace

struct VehNeighbour {
    int veh = -1;
    double gap = -1;
};

struct ChangeElem {
    int lane = -1;
    int lead = -1;
    int hopped = -1;
    int lastBlocked = -1;
    int firstBlocked = -1;
    int lastStopped = -1;
    double dens = 0;
    bool mayChangeRight = false;
    bool mayChangeLeft = false;
    double zipperDist = 0;  // MSLaneChanger::ChangeElem::zipperDist
    std::vector<int> work;   // unprocessed vehicles, rear .. front
    std::vector<int> done;   // processed vehicles in processing order (front .. rear)
};

struct LaneChanger {
    Sim& s;
    const Network& net;
    const int edge;
    std::vector<ChangeElem>& ch;
    int candi = -1;
    bool allowsChanging = true;
    bool checkOpened = false;
    std::vector<LCMessage>& msgs;
    std::vector<LaneChangeEvent>& events;
    std::vector<DetEvent>& det;
    std::vector<int>& changedVehs;
    // vehicles of this edge whose partial occupations changed in this sweep: SUMO's setPartialOccupation /
    // resetPartialOccupation take effect at once, psim2 updates the lane lists at the end of the stage
    std::vector<int>& dirtyLC;

    LaneChanger(Sim& sim, int e, std::vector<LCMessage>& m, std::vector<LaneChangeEvent>& ev, std::vector<DetEvent>& d,
                std::vector<int>& changed)
        : s(sim), net(sim.net), edge(e), ch(changeElems(sim.net.edges[e].lanes.size())), msgs(m), events(ev), det(d), changedVehs(changed),
          dirtyLC(dirtyList()) {
        dirtyLC.clear();
    }

    static std::vector<int>& dirtyList() {
        thread_local std::vector<int> d;
        return d;
    }
    void markDirty(int v) {
        if (std::find(dirtyLC.begin(), dirtyLC.end(), v) == dirtyLC.end()) {
            dirtyLC.push_back(v);
        }
    }
    // whether a vehicle of this edge occupies a lane apart from its own one (live state)
    bool occupiesLive(int u, int lane) const {
        const Kin& k = s.kin[u];
        if (k.shadowLane == lane) {
            return true;
        }
        for (int i = 0; i < k.nFurther; ++i) {
            if (k.further[i] == lane) {
                return true;
            }
        }
        if (k.shadowLane >= 0) {
            for (int i = 0; i < k.nShadowFurther; ++i) {
                if (k.shadowFurther[i] == lane) {
                    return true;
                }
            }
        }
        return false;
    }
    // MSLane::myPartialVehicles as of now (buf: storage for a corrected list)
    const std::vector<int>& partialsLC(int lane, std::vector<int>& buf) const {
        const std::vector<int>& p = s.lanes[lane].partials;
        if (dirtyLC.empty()) {
            return p;
        }
        auto dirty = [&](int u) { return std::find(dirtyLC.begin(), dirtyLC.end(), u) != dirtyLC.end(); };
        bool changed = false;
        buf.clear();
        for (int u : p) {
            if (dirty(u) && !occupiesLive(u, lane)) {
                changed = true;
            } else {
                buf.push_back(u);
            }
        }
        for (int u : dirtyLC) {
            if (occupiesLive(u, lane) && std::find(p.begin(), p.end(), u) == p.end()) {
                const double pu = posOnLane(u, lane);
                auto it = buf.begin();
                while (it != buf.end() && (posOnLane(*it, lane) < pu || (posOnLane(*it, lane) == pu && *it < u))) {
                    ++it;
                }
                buf.insert(it, u);
                changed = true;
            }
        }
        return changed ? buf : p;
    }

    // the change elements of the thread's current edge, one persistent vector per lane count so the
    // vehicle lists keep their capacity from edge to edge (one lane changer per thread at a time)
    static std::vector<ChangeElem>& changeElems(size_t nLanes) {
        thread_local std::vector<std::vector<ChangeElem>> byCount;
        if (byCount.size() <= nLanes) {
            byCount.resize(nLanes + 1);
        }
        std::vector<ChangeElem>& v = byCount[nLanes];
        if (v.size() != nLanes) {
            v.resize(nLanes);
        }
        return v;
    }

    // ---- ownership-aware accessors ------------------------------------------
    bool isOwn(int u) const {
        const int l = s.kinSnap[u].lane;
        return l >= 0 && net.lanes[l].edge == edge;
    }
    // vehicles of this edge and of the edges of earlier stages: live state, the others: the copy
    bool live(int u) const { return isOwn(u) || s.lcSnapStage[u] < s.lcCurStage; }
    const Kin& K(int u) const { return live(u) ? s.kin[u] : s.kinSnap[u]; }
    int laneOf(int u) const { return K(u).lane; }
    double pos(int u) const { return s.kin[u].pos; }
    double speed(int u) const { return s.kin[u].speed; }
    double accel(int u) const { return s.kin[u].accel; }
    const VType& T(int u) const { return s.types[s.kin[u].type]; }
    double backPosOnLane(int u, int lane) const { return s.backPosOf(K(u), lane); }
    double posOnLane(int u, int lane) const { return backPosOnLane(u, lane) + T(u).length; }
    int ownStateOf(int u) const { return live(u) ? s.veh[u].lc.ownState : s.lcStateSnap[u]; }
    double safetyFactor(int u) const { return 1. / T(u).lcAssertive; }
    double laneVehMaxSpeed(int lane, int u) const { return s.laneVehMaxSpeed(lane, u); }
    bool congested(int u) const {
        return speed(u) < (60.0 / 3.6) || net.lanes[laneOf(u)].speed < (60.1 / 3.6);
    }
    bool isStopped(int u) const { return s.isStopped(u); }
    double brutto(int lane) const {
        return net.lanes[lane].edge == edge ? s.lanes[lane].brutto : s.bruttoSnap[lane];
    }
    ChangeElem* elemOf(int lane) {
        return net.lanes[lane].edge == edge ? &ch[net.lanes[lane].index] : nullptr;
    }
    // vehicles of a lane (rear to front) while lanes of this edge are being changed
    template <class F>
    void fullVehicles(int lane, F f) {
        ChangeElem* ce = elemOf(lane);
        if (ce == nullptr) {
            for (int u : s.lanes[lane].vehs) {
                f(u);
            }
            return;
        }
        for (int u : ce->work) {
            f(u);
        }
        for (auto it = ce->done.rbegin(); it != ce->done.rend(); ++it) {
            f(*it);
        }
    }
    // MSLane::getLastAnyVehicle
    int lastAnyVehicle(int lane) {
        int best = -1;
        fullVehicles(lane, [&](int u) {
            if (best < 0 || pos(u) < pos(best)) {
                best = u;
            }
        });
        if (best >= 0) {
            return best;
        }
        thread_local std::vector<int> buf;
        const std::vector<int>& p = partialsLC(lane, buf);
        return p.empty() ? -1 : p.front();
    }
    // MSLane::getFirstAnyVehicle (lanes of other edges)
    int firstAnyVehicle(int lane) const {
        const LaneRT& l = s.lanes[lane];
        int result = l.vehs.empty() ? -1 : l.vehs.back();
        std::vector<int> buf;
        const std::vector<int>& p = partialsLC(lane, buf);
        if (!p.empty() && (result < 0 || posOnLane(result, lane) < posOnLane(p.back(), lane))) {
            result = p.back();
        }
        return result;
    }
    double secureGap(int follower, int leader, double fSpeed, double lSpeed, double leaderMaxDecel) const {
        return s.cfSecureGap(follower, leader, fSpeed, lSpeed, leaderMaxDecel, true);
    }

    // ---- messages -------------------------------------------------------------
    void addLCSpeedAdvice(int u, double vSafe, int flag) {
        s.veh[u].lc.advices.push_back({SPEED2ACCEL(vSafe - speed(u)), flag});
    }
    // MSLCM_LC2013::inform
    void inform(int receiver, double vSafe, int flags) {
        if (veh(ch[candi]) == s.traceVeh || receiver == s.traceVeh) {
            std::fprintf(stderr, "      %.2f inform %s -> %s vSafe=%.3f flags=%x\n", STEPS2TIME(s.now),
                         veh(ch[candi]) >= 0 ? s.veh[veh(ch[candi])].id.c_str() : "-", s.veh[receiver].id.c_str(), vSafe, flags);
        }
        if (isOwn(receiver)) {
            addLCSpeedAdvice(receiver, vSafe, LCA_CHANGE_TO_HELP);
            s.veh[receiver].lc.ownState |= flags;
        } else {
            msgs.push_back({receiver, 0, vSafe, flags});
        }
    }

    int veh(const ChangeElem& ce) const { return ce.work.empty() ? -1 : ce.work.back(); }
    bool vehInChanger() const {
        for (const ChangeElem& ce : ch) {
            if (!ce.work.empty()) {
                return true;
            }
        }
        return false;
    }

    // ---- sweep ------------------------------------------------------------------
    void init() {
        const Edge& e = net.edges[edge];
        allowsChanging = e.allowsChanging;
        for (int i = 0; i < (int)e.lanes.size(); ++i) {
            ChangeElem& ce = ch[i];
            std::vector<int> work, done;
            work.swap(ce.work);
            done.swap(ce.done);
            ce = ChangeElem();
            ce.work.swap(work);
            ce.done.swap(done);
            ce.lane = e.lanes[i];
            ce.mayChangeRight = i > 0;
            ce.mayChangeLeft = i + 1 < (int)e.lanes.size();
            if (e.internal) {
                for (const IncomingLane& ili : net.lanes[ce.lane].incoming) {
                    if (net.links[ili.link].state == LS_ZIPPER) {
                        ce.zipperDist = net.lanes[ce.lane].length;
                        break;
                    }
                }
            } else {
                for (int l : net.lanes[ce.lane].links) {
                    if (net.links[l].state == LS_ZIPPER) {
                        ce.zipperDist = std::max(ce.zipperDist, net.links[l].visibility);
                    }
                }
            }
            if (e.internal) {
                const Lane& L = net.lanes[ce.lane];
                if (ce.mayChangeRight) {
                    const Lane& R = net.lanes[e.lanes[i - 1]];
                    if (L.logicalPred == R.logicalPred || L.length != R.length) {
                        ce.mayChangeRight = false;
                    }
                }
                if (ce.mayChangeLeft) {
                    const Lane& Lf = net.lanes[e.lanes[i + 1]];
                    if (L.logicalPred == Lf.logicalPred || L.length != Lf.length) {
                        ce.mayChangeLeft = false;
                    }
                }
            }
            ce.work = s.lanes[ce.lane].vehs;
            ce.done.clear();
            ce.done.reserve(ce.work.size() + 4);
        }
        checkOpened = s.lcCheckOpened[edge];
    }

    void laneChange() {
        struct ComputeLC {  // MSGlobals::gComputeLC (and the edge whose vehicles are live)
            explicit ComputeLC(int e) {
                cfComputeLC = true;
                lcOwnEdge = e;
            }
            ~ComputeLC() {
                cfComputeLC = false;
                lcOwnEdge = -1;
            }
        } computeLC(edge);
        init();
        while (vehInChanger()) {
            const bool changed = change();
            updateChanger(changed);
        }
        EdgeRT& ert = s.edgesRT[edge];
        for (int i = 0; i < (int)ch.size(); ++i) {
            ChangeElem& ce = ch[i];
            std::reverse(ce.done.begin(), ce.done.end());
            ert.lastBlockedBackPos[i] = -1;
            ert.lastBlockedWaiting[i] = -1;
            if (ce.firstBlocked >= 0) {
                ert.lastBlockedBackPos[i] = backPosOnLane(ce.firstBlocked, laneOf(ce.firstBlocked));
                ert.lastBlockedWaiting[i] = STEPS2TIME(s.veh[ce.firstBlocked].waitingTime);
            }
        }
    }

    int findCandidate() const {
        int max = -1;
        for (int i = 0; i < (int)ch.size(); ++i) {
            const int v = veh(ch[i]);
            if (v < 0) {
                continue;
            }
            if (max < 0 || pos(veh(ch[max])) < pos(v)) {
                max = i;
            }
        }
        return max;
    }

    bool mayChange(int direction) const {
        if (direction == 0) {
            return true;
        }
        if (!allowsChanging) {
            return false;
        }
        const int vc = s.veh[veh(ch[candi])].vclass;
        const Lane& cur = net.lanes[ch[candi].lane];
        if (direction == -1) {
            return ch[candi].mayChangeRight && net.lanes[ch[candi - 1].lane].allows(vc) && (cur.changeRight & vc) == vc;
        }
        return ch[candi].mayChangeLeft && net.lanes[ch[candi + 1].lane].allows(vc) && (cur.changeLeft & vc) == vc;
    }

    void updateChanger(bool vehHasChanged) {
        if (!vehHasChanged) {
            ch[candi].lead = veh(ch[candi]);
        }
        ch[candi].work.pop_back();
    }

    void registerHop(int target, int v) {
        ch[target].done.push_back(v);
        ch[target].dens += T(v).lengthWithGap();
        ch[target].hopped = v;
    }

    void registerUnchanged(int v) {
        ch[candi].done.push_back(v);
        ch[candi].dens += T(v).lengthWithGap();
    }

    void setOwnState(int v, int state) {
        s.veh[v].lc.previousState = state;
        s.veh[v].lc.ownState = state;
    }

    bool change() {
        candi = findCandidate();
        const int vehicle = veh(ch[candi]);
        if (s.veh[vehicle].lc.isChangingLanes() && !s.veh[vehicle].lc.changed) {
            return continueChange(vehicle);  // a continuous lane change goes on (no new decisions)
        }
        const bool stoppedOnLane = isStopped(vehicle) && s.veh[vehicle].stops.front().lane == laneOf(vehicle);
        if (!allowsChanging || stoppedOnLane) {
            registerUnchanged(vehicle);
            if (stoppedOnLane) {
                ch[candi].lastStopped = vehicle;
            }
            return false;
        }
        if (!s.veh[vehicle].actionStep) {
            registerUnchanged(vehicle);  // no change checks between actions
            return false;
        }
        const VehNeighbour leader = getRealLeader(candi);
        if (ch.size() == 1 || (!mayChange(-1) && !mayChange(1))) {
            registerUnchanged(vehicle);
            return false;
        }
        updateOccupancy(vehicle);
        std::vector<LaneQ>& preb = s.veh[vehicle].bestLanes.front();
        for (int i = 0; i < (int)ch.size() && i < (int)preb.size(); ++i) {
            preb[i].occupation = ch[i].dens + preb[i].nextOccupation;
        }
        int stateRight = 0;
        if (mayChange(-1)) {
            stateRight = checkChangeWithinEdge(-1, leader);
            if ((stateRight & LCA_RIGHT) != 0 && (stateRight & LCA_BLOCKED) == 0) {
                setOwnState(vehicle, stateRight);
                return startChange(vehicle, -1);
            }
            if ((stateRight & LCA_RIGHT) != 0 && (stateRight & LCA_URGENT) != 0) {
                ch[candi - 1].lastBlocked = vehicle;
                if (ch[candi - 1].firstBlocked < 0) {
                    ch[candi - 1].firstBlocked = vehicle;
                }
            }
        }
        int stateLeft = 0;
        if (mayChange(1)) {
            stateLeft = checkChangeWithinEdge(1, leader);
            if ((stateLeft & LCA_LEFT) != 0 && (stateLeft & LCA_BLOCKED) == 0) {
                setOwnState(vehicle, stateLeft);
                return startChange(vehicle, 1);
            }
            if ((stateLeft & LCA_LEFT) != 0 && (stateLeft & LCA_URGENT) != 0) {
                ch[candi + 1].lastBlocked = vehicle;
                if (ch[candi + 1].firstBlocked < 0) {
                    ch[candi + 1].firstBlocked = vehicle;
                }
            }
        }
        if ((stateRight & LCA_URGENT) != 0 && (stateLeft & LCA_URGENT) != 0) {
            stateLeft = 0;
        }
        setOwnState(vehicle, stateRight | stateLeft);
        registerUnchanged(vehicle);
        return false;
    }

    // MSVehicle::updateBestLanes during lane changing. Occupancies are read from
    // Sim::bruttoSnap, which holds the values published by the previous stages.
    // (On a junction-internal lane SUMO keeps the occupancy computed when the
    // vehicle entered the lane; updateBestLanes reproduces that.)
    void updateOccupancy(int v) { s.updateBestLanes(v, false, laneOf(v)); }

    // MSLaneChanger::startChange -> MSAbstractLaneChangeModel::primaryLaneChanged
    // MSLaneChanger::startChange: MSAbstractLaneChangeModel::startLaneChangeManeuver (continuous with
    // --lanechange.duration, otherwise the vehicle changes at once)
    bool startChange(int v, int direction) {
        if (s.opt.laneChangeDuration > DELTA_T) {
            LCState& lc = s.veh[v].lc;
            lc.completion = 0;
            lc.lcDirection = direction;
            lc.maneuverDist = (net.lanes[ch[candi + direction].lane].width + net.lanes[ch[candi].lane].width) * 0.5 * direction;
            return continueChange(v);
        }
        return primaryLaneChanged(v, direction);
    }

    // MSLaneChanger::continueChange: lateral movement; at the midpoint the vehicle changes its lane
    bool continueChange(int v) {
        LCState& lc = s.veh[v].lc;
        const int direction = lc.lcDirection;
        // MSAbstractLaneChangeModel::updateCompletion
        const bool pastBefore = lc.pastMidpoint();
        lc.speedLat = s.lcComputeSpeedLat(v, lc.maneuverDist, (lc.ownState & LCA_URGENT) != 0);
        lc.completion += SPEED2DIST(lc.speedLat) / lc.maneuverDist;
        const bool pastMidpoint = !pastBefore && lc.pastMidpoint();
        Kin& k = s.kin[v];
        k.posLat += SPEED2DIST(lc.speedLat);
        if (pastMidpoint && candi + direction >= 0 && candi + direction < (int)ch.size()) {
            k.posLat -= direction * 0.5 * (net.lanes[ch[candi].lane].width + net.lanes[ch[candi + direction].lane].width);
            primaryLaneChanged(v, direction);
        } else {
            registerHop(candi, v);
        }
        if (!lc.isChangingLanes()) {
            k.posLat = 0;
            s.endLaneChangeManeuver(v);
        }
        s.updateShadowLane(v);
        s.lcShadowPerEdge[edge].push_back(v);  // (its partial occupations follow at the end of the stage)
        markDirty(v);
        if (k.shadowLane >= 0 && net.lanes[k.shadowLane].edge == edge) {
            // found as leader on both lanes
            ch[net.lanes[k.shadowLane].index].hopped = v;
        }
        return pastMidpoint;
    }

    // MSAbstractLaneChangeModel::primaryLaneChanged: the change of the vehicle's lane
    bool primaryLaneChanged(int v, int direction) {
        const int target = candi + direction;
        const int source = ch[candi].lane;
        const int targetLane = ch[target].lane;
        Kin& k = s.kin[v];
        Veh& x = s.veh[v];
        // leaveLane(NOTIFICATION_LANE_CHANGE): detectors of the lanes the vehicle touches
        if (!s.loopsOf(v, source).empty()) {
            s.loopNotifyLeave(v, source, det);
        }
        for (int i = 0; i < k.nFurther; ++i) {
            if (!s.loopsOf(v, k.further[i]).empty()) {
                s.loopNotifyLeave(v, k.further[i], det);
            }
        }
        x.aliasLane = x.aliasLoopLane = -1;  // leaveLane removed all detector reminders
        s.lanes[source].brutto -= T(v).lengthWithGap();
        s.lanes[source].netto -= T(v).length;
        if (s.opt.lcOutput) {
            events.push_back({STEPS2TIME(s.now), v, source, targetLane, k.pos, k.speed, x.lc.ownState & ~(LCA_RIGHT | LCA_LEFT)});
        } else {
            events.push_back({STEPS2TIME(s.now), v, source, targetLane, 0, 0, 0});
        }
        // MSVehicle::enterLaneAtLaneChange
        k.lane = targetLane;
        s.updateBestLanes(v, false, targetLane);
        double leftLength = T(v).length - k.pos;
        int lane = targetLane;
        int deleteFurther = 0;
        for (int i = 0; i < k.nFurther; i++) {
            if (lane >= 0) {
                lane = net.predecessorOnEdge(lane, net.lanes[k.further[i]].edge);
            }
            if (leftLength > 0) {
                if (lane >= 0) {
                    k.further[i] = lane;
                    if (s.sublane) {
                        s.lat[v].furtherPosLat[i] = k.posLat;
                    }
                    leftLength -= net.lanes[lane].length;
                    k.backPos = -leftLength;
                } else {
                    if (s.sublane && x.lc.isChangingLanes()) {
                        s.lat[v].noShadow.push_back(k.further[i]);  // setNoShadowPartialOccupator
                    }
                    if (k.backPos < 0) {
                        k.backPos += net.lanes[k.further[i]].length;
                    }
                }
            } else {
                deleteFurther++;
            }
        }
        k.nFurther -= deleteFurther;
        s.setNoReminderFurther(v);
        if (!s.laneLoops[targetLane].empty()) {
            s.loopNotifyEnter(v, targetLane, det);
        }
        if (s.hasRerouters()) {
            s.rerouterEnter(v, targetLane, true);  // NOTIFICATION_LANE_CHANGE: never reroutes
        }
        updateOccupancy(v);
        s.lanes[targetLane].brutto += T(v).lengthWithGap();
        s.lanes[targetLane].netto += T(v).length;
        x.nLaneChanges++;
        if (s.latRes > 0 && s.isSL(v)) {
            s.slChanged(v);
        } else {
            lcChanged(v);
        }
        changedVehs.push_back(v);
        markDirty(v);
        registerHop(target, v);
        return true;
    }

    // MSLCM_LC2013::changed
    void lcChanged(int v) {
        LCState& lc = s.veh[v].lc;
        lc.ownState = 0;
        lc.speedGainProbLeft = 0;
        lc.speedGainProbRight = 0;
        lc.keepRightProb = 0;
        if (s.bestLaneOffset(v) == 0) {
            lc.leadingBlockerLength = 0;
            lc.leftSpace = 0;
        }
        lc.lookAheadSpeed = LOOK_AHEAD_MIN_SPEED;
        lc.advices.clear();
        lc.dontBrake = false;
        lc.leadingBlockerLength = 0;
    }

    // ---- neighbours -----------------------------------------------------------
    VehNeighbour getRealLeader(int target) {
        const int vehicle = veh(ch[candi]);
        int neighLead = ch[target].lead;
        if (ch[target].hopped >= 0) {
            const double hoppedPos = pos(ch[target].hopped);
            if (hoppedPos > pos(vehicle) && (neighLead < 0 || pos(neighLead) > hoppedPos)) {
                neighLead = ch[target].hopped;
            }
        }
        if (neighLead < 0) {
            const int targetLane = ch[target].lane;
            const double egoBack = pos(vehicle) - T(vehicle).length;
            double leaderBack = net.lanes[targetLane].length;
            std::vector<int> buf;
            for (int pl : partialsLC(targetLane, buf)) {
                const double plBack = backPosOnLane(pl, targetLane);
                const double plPos = plBack + T(pl).length;
                if (plBack < leaderBack && plPos + T(pl).minGap >= egoBack) {
                    neighLead = pl;
                    leaderBack = plBack;
                }
            }
            if (neighLead >= 0) {
                return {neighLead, leaderBack - pos(vehicle) - T(vehicle).minGap};
            }
            const double seen = net.lanes[ch[candi].lane].length - pos(vehicle);
            const double spd = speed(vehicle);
            const double dist = s.cfBrakeGap(vehicle, spd) + T(vehicle).minGap;
            if (seen > dist && !net.lanes[ch[candi].lane].internal) {
                return {-1, -1};
            }
            const std::vector<int>& conts = s.bestContinuationOf(vehicle, targetLane);
            return getLeaderOnConsecutive(targetLane, dist, seen, spd, vehicle, conts);
        }
        return {neighLead, backPosOnLane(neighLead, ch[target].lane) - pos(vehicle) - T(vehicle).minGap};
    }

    static int closerFollower(const LaneChanger& lc, double maxPos, int f1, int f2) {
        if (f1 < 0 || lc.pos(f1) > maxPos) {
            return f2;
        }
        if (f2 < 0 || lc.pos(f2) > maxPos) {
            return f1;
        }
        return lc.pos(f1) > lc.pos(f2) ? f1 : f2;
    }

    VehNeighbour getRealFollower(int target) {
        const int vehicle = veh(ch[candi]);
        const double candiPos = pos(vehicle);
        int neighFollow = -1;
        if (target != candi) {
            neighFollow = veh(ch[target]);
        } else if (ch[target].work.size() > 1) {
            neighFollow = ch[target].work[ch[target].work.size() - 2];
        }
        neighFollow = closerFollower(*this, candiPos, neighFollow, ch[target].hopped);
        // MSLane::getPartialBehind is always empty here (partial occupants never have
        // their front on the lane), but this second call also drops a hopped vehicle
        // that is ahead of the candidate
        neighFollow = closerFollower(*this, candiPos, neighFollow, -1);
        if (neighFollow < 0) {
            const Sim::CritFollower cf = getFollowersOnConsecutive(ch[target].lane, vehicle, pos(vehicle) - T(vehicle).length);
            return {cf.veh, cf.veh >= 0 ? cf.dist : -1};
        }
        return {neighFollow, pos(vehicle) - T(vehicle).length - pos(neighFollow) - T(neighFollow).minGap};
    }

    // MSLane::getLeaderOnConsecutive (gComputeLC, considerCrossingFoes = true)
    VehNeighbour getLeaderOnConsecutive(int lane, double dist, double seen, double spd, int v, const std::vector<int>& conts) {
        if (seen > dist && !net.lanes[lane].internal) {
            return {-1, -1};
        }
        const VType& t = T(v);
        int view = 1;
        std::vector<int> buf;
        const std::vector<int>& partials = partialsLC(lane, buf);
        if (!partials.empty()) {
            const int pred = partials.front();
            const double gap = seen - (net.lanes[lane].length - backPosOnLane(pred, lane)) - t.minGap;
            if (gap > 0) {
                return {pred, gap};
            }
        }
        thread_local std::vector<Sim::LinkLeader> leaders;
        int nextLane = lane;
        do {
            int link = s.succLinkSec(v, view, nextLane, conts);
            if (link < 0 && view < (int)s.routes[s.veh[v].route].size() - s.veh[v].routeIdx) {
                const int nextEdge = s.routeEdge(v, view);
                if (net.edges[nextEdge].lanes.size() == 1) {
                    // lanes are unambiguous on the next route edge, continue beyond the best lanes
                    for (int l : net.lanes[nextLane].links) {
                        if (net.links[l].lane == net.edges[nextEdge].lanes.front()) {
                            link = l;
                            break;
                        }
                    }
                }
            }
            if (link < 0) {
                break;
            }
            // link leaders: on another lane than the vehicle's own all candidates are leaders
            s.leaderInfo(link, v, seen, leaders, true);
            if (!leaders.empty()) {
                const bool laneChanging = s.KV(v).lane != lane;
                VehNeighbour result;
                double shortestGap = DMAX;
                for (const Sim::LinkLeader& ll : leaders) {
                    double gap = ll.gap;  // a pedestrian blocks with gap -1
                    if (ll.veh >= 0) {
                        if (!laneChanging && !s.isLeader(v, link, ll.veh, ll.gap)) {
                            continue;
                        }
                        gap += s.cfBrakeGap(ll.veh, speed(ll.veh), T(ll.veh).decel, 0);
                    }
                    if (gap < shortestGap) {
                        shortestGap = gap;
                        result = {ll.veh, ll.gap};
                    }
                }
                if (shortestGap != DMAX) {
                    return result;
                }
            }
            const bool nextInternal = net.links[link].via >= 0;
            nextLane = net.links[link].viaOrLane();
            const int leader = lastAnyVehicle(nextLane);
            if (leader >= 0) {
                return {leader, seen + backPosOnLane(leader, nextLane) - t.minGap};
            }
            if (laneVehMaxSpeed(nextLane, v) < spd) {
                dist = s.cfBrakeGap(v, laneVehMaxSpeed(nextLane, v));
            }
            seen += net.lanes[nextLane].length;
            if (!nextInternal) {
                view++;
            }
        } while (seen <= dist || net.lanes[nextLane].internal);
        return {-1, -1};
    }

    // MSVehicle::unsafeLinkAhead: zipper links within reach that have approaching
    // foes in the relevant time window, and unprioritized links the vehicle
    // could not stop for
    bool unsafeLinkAhead(int v, int lane, double zipperDist) {
        const Veh& x = s.veh[v];
        double seen = net.lanes[laneOf(v)].length - pos(v);
        const double dist = std::max(zipperDist, s.cfBrakeGap(v, speed(v), T(v).decel, 0));
        if (seen >= dist) {
            return false;
        }
        const std::vector<int>& conts = s.bestContinuationOf(v, lane);
        int view = 1;
        int link = s.succLinkSec(v, view, lane, conts);
        size_t di = 0;
        while (link >= 0 && seen <= dist) {
            const Link& L = net.links[link];
            const bool internal = net.lanes[lane].internal;
            if ((!internal && ((L.state == LS_ZIPPER && seen < L.visibility) || !L.havePriority())) || (internal && zipperDist > 0)) {
                // find the drive item corresponding to this link
                bool found = false;
                while (di < x.lf.size() && !found) {
                    if (x.lf[di].link >= 0 && net.lanes[net.links[x.lf[di].link].laneBefore].edge == net.lanes[lane].edge) {
                        found = true;
                    } else {
                        di++;
                    }
                }
                if (found) {
                    const DriveItem& d = x.lf[di];
                    const Time lt = s.leaveTime(link, d.arrivalTime, d.arrivalSpeed, d.leaveSpeed(), T(v).length);
                    if (s.hasApproachingFoe(net.correspondingEntryLink(link), d.arrivalTime, lt, d.arrivalSpeed, T(v).decel)) {
                        return true;
                    }
                }
            }
            lane = L.viaOrLane();
            if (!net.lanes[lane].internal) {
                view++;
            }
            seen += net.lanes[lane].length;
            link = s.succLinkSec(v, view, lane, conts);
        }
        return false;
    }

    // MSLane::getCriticalLeader
    VehNeighbour getCriticalLeader(int lane, double dist, double seen, double spd, int v) {
        const std::vector<int>& conts = s.bestContinuationOf(v, lane);
        VehNeighbour result{-1, -1};
        double safeSpeed = DMAX;
        int view = 1;
        int nextLane = lane;
        const VType& t = T(v);
        Time arrivalTime = s.now + TIME2STEPS(seen / std::max(spd, NUMERICAL_EPS));
        thread_local std::vector<Sim::LinkLeader> leaders;
        do {
            const int link = s.succLinkSec(v, view, nextLane, conts);
            if (link < 0 || !s.opened(link, arrivalTime, spd, spd, t.length, s.impatience(v), t.decel, 0, v)
                    || net.links[link].haveRed()) {
                return result;
            }
            s.leaderInfo(link, v, seen, leaders, true);
            for (const Sim::LinkLeader& ll : leaders) {
                if (ll.veh >= 0 && ll.veh != result.veh) {
                    double tmpSpeed = safeSpeed;
                    s.adaptToJunctionLeader(v, ll.veh, ll.gap, seen, nullptr, nextLane, tmpSpeed, tmpSpeed, ll.distToCrossing);
                    if (tmpSpeed < safeSpeed) {
                        safeSpeed = tmpSpeed;
                        result = {ll.veh, ll.gap};
                    }
                }
            }
            const bool nextInternal = net.links[link].via >= 0;
            nextLane = net.links[link].viaOrLane();
            const int leader = lastAnyVehicle(nextLane);
            if (leader >= 0 && leader != result.veh) {
                const double gap = seen + backPosOnLane(leader, nextLane) - t.minGap;
                // insertionFollowSpeed while MSGlobals::gComputeLC is set
                const double tmpSpeed = cf::maximumSafeFollowSpeed(t, gap, spd, speed(leader), T(leader).decel, true, true);
                if (tmpSpeed < safeSpeed) {
                    safeSpeed = tmpSpeed;
                    result = {leader, gap};
                }
            }
            if (laneVehMaxSpeed(nextLane, v) < spd) {
                dist = s.cfBrakeGap(v, laneVehMaxSpeed(nextLane, v));
            }
            seen += net.lanes[nextLane].length;
            if (seen <= dist) {
                arrivalTime += TIME2STEPS(net.lanes[nextLane].length / std::max(spd, NUMERICAL_EPS));
            }
            if (!nextInternal) {
                view++;
            }
        } while (seen <= dist || net.lanes[nextLane].internal);
        return result;
    }

    // MSLane::getFollowersOnConsecutive(ego, backOffset, allSublanes = true)[0]
    Sim::CritFollower getFollowersOnConsecutive(int lane, int ego, double backOffset) {
        Sim::CritFollower result;
        const double egoPos = backOffset + T(ego).length;
        auto required = [&](int u) { return secureGap(u, ego, speed(u), speed(ego), T(ego).decel); };
        auto consider = [&](int u) {
            if (u != ego && posOnLane(u, lane) < egoPos) {
                result.add(u, backOffset - posOnLane(u, lane) - T(u).minGap, required(u));
            }
        };
        // full and partial occupants merged by position (AnyVehicleIterator)
        fullVehicles(lane, consider);
        std::vector<int> pbuf;
        for (int u : partialsLC(lane, pbuf)) {
            consider(u);
        }
        if (result.veh >= 0) {
            return result;
        }
        double searchDist = net.lanes[lane].maxBrakeDist - backOffset;
        // edges of the further lanes of ego
        const Kin& ke = K(ego);
        auto egoFurtherHas = [&](int e) {
            for (int i = 0; i < ke.nFurther; ++i) {
                if (net.lanes[ke.further[i]].edge == e) {
                    return true;
                }
            }
            return false;
        };
        // search buffers, reused (not re-entrant)
        thread_local std::vector<int> visited;
        const std::vector<int>& edgeLanes = net.edges[net.lanes[lane].edge].lanes;
        visited.assign(edgeLanes.begin(), edgeLanes.end());
        struct Inc {
            int lane, link;
            double length;
        };
        thread_local std::vector<Inc> toExamine, newFound;
        toExamine.clear();
        newFound.clear();
        for (const IncomingLane& i : net.lanes[lane].incoming) {
            toExamine.push_back({i.lane, i.link, i.length});
        }
        thread_local std::vector<Sim::LinkLeader> leaders;
        const int egoEdge = net.lanes[laneOf(ego)].edge;
        while (!toExamine.empty()) {
            for (const Inc& it : toExamine) {
                const int next = it.lane;
                searchDist = std::min(searchDist, net.lanes[next].maxBrakeDist - backOffset);
                if (backOffset + it.length - net.lanes[next].length < 0 && egoFurtherHas(net.lanes[next].edge)) {
                    s.leaderInfo(it.link, ego, -backOffset, leaders, true);
                    for (const Sim::LinkLeader& ll : leaders) {
                        if (ll.veh < 0) {
                            continue;  // pedestrian
                        }
                        const bool egoIsLeader = s.isLeader(ll.veh, it.link, ego, ll.gap);
                        const double gap = egoIsLeader ? -ll.gap - T(ll.veh).lengthWithGap() - T(ego).minGap : ll.gap + T(ego).length;
                        result.add(ll.veh, gap, required(ll.veh));
                    }
                }
                ChangeElem* ce = elemOf(next);
                int firstAny, firstFront;
                if (ce != nullptr) {
                    firstFront = -1;
                    fullVehicles(next, [&](int u) {
                        if (firstFront < 0 || pos(u) > pos(firstFront)) {
                            firstFront = u;
                        }
                    });
                    firstAny = firstFront;
                    std::vector<int> buf;
                    const std::vector<int>& p = partialsLC(next, buf);
                    if (!p.empty() && (firstAny < 0 || posOnLane(firstAny, next) < posOnLane(p.back(), next))) {
                        firstAny = p.back();
                    }
                } else {
                    firstAny = firstAnyVehicle(next);
                    firstFront = s.lanes[next].vehs.empty() ? -1 : s.lanes[next].vehs.back();
                }
                int v = firstAny == ego ? firstFront : firstAny;
                if (v >= 0 && v != ego) {
                    double agap = 0;
                    if (laneOf(v) != next) {
                        agap = it.length - net.lanes[next].length + backOffset;
                        if (net.usingInternalLanes) {
                            agap -= T(v).minGap;  // (ego should have left the intersection still occupied by v)
                        }
                        const bool differentEdge = net.lanes[laneOf(v)].edge != egoEdge;
                        if (agap > 0 && differentEdge) {
                            v = firstFront;
                            if (v >= 0 && v != ego) {
                                agap = it.length - pos(v) + backOffset - T(v).minGap;
                            } else {
                                v = -1;
                            }
                        } else if (differentEdge && result.veh == v) {
                            agap = 0;
                        }
                    } else {
                        agap = it.length - pos(v) + backOffset - T(v).minGap;
                        if (!net.links[it.link].havePriority() && !egoFurtherHas(net.lanes[it.lane].edge)
                                && speed(v) < HALTING_SPEED) {
                            agap = std::max(agap, 0.0);
                        }
                    }
                    if (v >= 0) {
                        result.add(v, agap, required(v));
                    }
                }
                if (it.length < searchDist) {
                    for (const IncomingLane& j : net.lanes[next].incoming) {
                        if (std::find(visited.begin(), visited.end(), j.lane) == visited.end()) {
                            visited.push_back(j.lane);
                            newFound.push_back({j.lane, j.link, j.length + it.length});
                        }
                    }
                }
            }
            toExamine.clear();
            std::swap(newFound, toExamine);
        }
        return result;
    }

    // MSLane::getLeader (used by hasFreeLane)
    VehNeighbour getLeaderOn(int lane, int v, double vehPos, const std::vector<int>& conts, double dist) {
        VehNeighbour found;
        bool have = false;
        int best = -1;
        // vehicles of the lane rear to front: the first at or ahead of vehPos
        thread_local std::vector<int> cands;  // not re-entrant
        cands.clear();
        fullVehicles(lane, [&](int u) { cands.push_back(u); });
        std::vector<int> pbuf;
        for (int u : partialsLC(lane, pbuf)) {
            cands.push_back(u);
        }
        std::sort(cands.begin(), cands.end(), [&](int a, int b) { return posOnLane(a, lane) < posOnLane(b, lane); });
        for (int u : cands) {
            if (u != v && posOnLane(u, lane) >= vehPos) {
                best = u;
                have = true;
                break;
            }
        }
        if (have) {
            found = {best, backPosOnLane(best, lane) - T(v).minGap - vehPos};
            return found;
        }
        if (!conts.empty()) {
            const double seen = net.lanes[lane].length - vehPos;
            if (dist < 0) {
                dist = s.cfBrakeGap(v, speed(v)) + T(v).minGap;
            }
            if (seen > dist) {
                return {-1, -1};
            }
            return getLeaderOnConsecutive(lane, dist, seen, speed(v), v, conts);
        }
        return {-1, -1};
    }

    // ---- checkChange ------------------------------------------------------------
    int checkChangeWithinEdge(int laneOffset, const VehNeighbour& leader) {
        const VehNeighbour follower = getRealFollower(candi);
        const VehNeighbour neighLead = getRealLeader(candi + laneOffset);
        VehNeighbour neighFollow = getRealFollower(candi + laneOffset);
        if (neighLead.veh >= 0 && neighLead.veh == neighFollow.veh) {
            neighFollow.veh = -1;
        }
        const int state = checkChange(laneOffset, ch[candi + laneOffset].lane, leader, follower, neighLead, neighFollow);
        s.saveLCState(veh(ch[candi]), laneOffset, state);
        return state;
    }

    // MSLCM_LC2013::adaptSpeedToPedestrians
    void adaptSpeedToPedestrians(int v, int lane, double& vs) const {
        if (s.hasPedestrians(lane)) {
            const double right = s.rightSideOnLane(v, laneOf(v));
            const std::pair<int, double> pl = s.nextBlocking(lane, pos(v), right, right + T(v).width, std::ceil(speed(v) / T(v).decel));
            if (pl.first >= 0) {
                vs = std::min(vs, s.cfStopSpeed(v, speed(v), pl.second - T(v).minGap));
            }
        }
    }

    int checkChange(int laneOffset, int targetLane, const VehNeighbour& leader, const VehNeighbour& follower,
                    const VehNeighbour& neighLead, const VehNeighbour& neighFollow) {
        const int vehicle = veh(ch[candi]);
        int blocked = 0;
        const int blockedByLeader = (laneOffset == -1 ? LCA_BLOCKED_BY_RIGHT_LEADER : LCA_BLOCKED_BY_LEFT_LEADER);
        const int blockedByFollower = (laneOffset == -1 ? LCA_BLOCKED_BY_RIGHT_FOLLOWER : LCA_BLOCKED_BY_LEFT_FOLLOWER);
        if (neighFollow.veh >= 0 && neighFollow.gap < 0) {
            blocked |= (blockedByFollower | LCA_OVERLAPPING);
        }
        if (neighLead.veh >= 0 && neighLead.gap < 0) {
            blocked |= (blockedByLeader | LCA_OVERLAPPING);
        }
        // the speeds extrapolated over the rest of the reaction time (with action steps longer than the step)
        const double tauRemainder = s.actionStepLength(vehicle) == DELTA_T ? 0 : std::max(T(vehicle).tau - TS, 0.);
        if ((blocked & blockedByFollower) == 0 && neighFollow.veh >= 0) {
            const int nf = neighFollow.veh;
            const double vNextFollower = speed(nf) + std::max(0., tauRemainder * accel(nf));
            const double vNextLeader = speed(vehicle) + std::min(0., tauRemainder * accel(vehicle));
            const double secureBackGap = secureGap(nf, vehicle, vNextFollower, vNextLeader, T(vehicle).decel);
            if (neighFollow.gap < secureBackGap * safetyFactor(vehicle)) {
                blocked |= blockedByFollower;
            }
        }
        if ((blocked & blockedByLeader) == 0 && neighLead.veh >= 0) {
            const int nl = neighLead.veh;
            const double vNextFollower = speed(vehicle) + std::max(0., tauRemainder * accel(vehicle));
            const double vNextLeader = speed(nl) + std::min(0., tauRemainder * accel(nl));
            const double secureFrontGap = secureGap(vehicle, nl, vNextFollower, vNextLeader, T(nl).decel);
            if (neighLead.gap < secureFrontGap * safetyFactor(vehicle)) {
                blocked |= blockedByLeader;
            }
        }
        if (blocked == 0 && s.hasPedestrians(targetLane)) {
            const double right = s.rightSideOnLane(vehicle, targetLane);
            const std::pair<int, double> pl = s.nextBlocking(targetLane, pos(vehicle) - T(vehicle).length, right,
                                                             right + T(vehicle).width, std::ceil(speed(vehicle) / T(vehicle).decel));
            if (pl.first >= 0) {
                // the gap is relative to the back position
                const double gap = pl.second - T(vehicle).lengthWithGap();
                if (s.cfBrakeGap(vehicle, speed(vehicle)) > gap) {
                    blocked |= blockedByLeader;
                }
            }
        }
        int state = blocked | wantsChange(laneOffset, blocked, leader, follower, neighLead, neighFollow, targetLane,
                                           ch[candi].lastBlocked, ch[candi].firstBlocked);
        if (vehicle == s.traceVeh) {
            auto id = [&](int u) { return u < 0 ? std::string("-") : s.veh[u].id; };
            std::fprintf(stderr, "%.0f %s dir=%d state=%d blocked=%d lead=%s/%.2f fol=%s/%.2f nlead=%s/%.2f nfol=%s/%.2f "
                         "pos=%.3f v=%.3f la=%.3f sgR=%lld sgL=%lld kr=%lld\n",
                         STEPS2TIME(s.now), id(vehicle).c_str(), laneOffset, state, blocked, id(leader.veh).c_str(), leader.gap,
                         id(follower.veh).c_str(), follower.gap, id(neighLead.veh).c_str(), neighLead.gap,
                         id(neighFollow.veh).c_str(), neighFollow.gap, pos(vehicle), speed(vehicle),
                         s.veh[vehicle].lc.lookAheadSpeed, s.veh[vehicle].lc.speedGainProbRight,
                         s.veh[vehicle].lc.speedGainProbLeft, s.veh[vehicle].lc.keepRightProb);
        }
        if (blocked == 0 && (state & LCA_WANTS_LANECHANGE) != 0 && neighLead.veh >= 0) {
            const double seen = net.lanes[ch[candi].lane].length - pos(vehicle);
            const double spd = speed(vehicle);
            const double dist = s.cfBrakeGap(vehicle, spd) + T(vehicle).minGap;
            if (seen < dist || net.lanes[ch[candi].lane].internal) {
                const VehNeighbour neighLead2 = getCriticalLeader(targetLane, dist, seen, spd, vehicle);
                if (neighLead2.veh >= 0 && neighLead2.veh != neighLead.veh) {
                    const double sg = secureGap(vehicle, neighLead2.veh, speed(vehicle), speed(neighLead2.veh), T(neighLead2.veh).decel);
                    if (neighLead2.gap < sg * safetyFactor(vehicle)) {
                        state |= blockedByLeader;
                    }
                }
            }
        }
        if (blocked == 0 && (state & LCA_WANTS_LANECHANGE) != 0) {
            // ensure that merging is safe for any upcoming zipper links after changing
            double targetZipperDist = 0;
            if (laneOffset == 0) {
                targetZipperDist = ch[candi].zipperDist;
            } else if (laneOffset == 1 && candi + 1 < (int)ch.size()) {
                targetZipperDist = ch[candi + 1].zipperDist;
            } else if (laneOffset == -1 && candi > 0) {
                targetZipperDist = ch[candi - 1].zipperDist;
            }
            if (unsafeLinkAhead(vehicle, targetLane, targetZipperDist)) {
                state |= blockedByLeader;
            }
        }
        if ((state & LCA_BLOCKED) == 0 && (state & LCA_WANTS_LANECHANGE) != 0 && s.opt.laneChangeDuration > DELTA_T) {
            state |= continuousChangeBlocked(vehicle, laneOffset, targetLane, neighLead.veh, state, blockedByLeader);
        }
        if (checkOpened && (state & LCA_BLOCKED) == 0 && (state & LCA_WANTS_LANECHANGE) != 0
                && !net.lanes[laneOf(vehicle)].internal) {
            const std::vector<int>& conts = s.bestContinuation(vehicle);
            if (conts.size() > 1 && conts[1] >= 0) {
                const int link = net.linkIdxTo(laneOf(vehicle), conts[1]);
                if (link >= 0 && net.links[link].isEntryLink()) {
                    const int link2 = laneOffset < 0 ? net.links[link].parallelRight : net.links[link].parallelLeft;
                    if (link2 >= 0) {
                        const Approach* api = s.approachOf(link, vehicle);
                        if (api != nullptr) {
                            if (!s.opened(link2, api->arrivalTime, api->arrivalSpeed, api->leaveSpeed, T(vehicle).length,
                                          s.impatience(vehicle), T(vehicle).decel, s.veh[vehicle].waitingTime, vehicle, false,
                                          nullptr, api->dist)) {
                                state |= LCA_BLOCKED;
                            }
                        }
                    }
                }
            }
        }
        return state;
    }

    // MSLaneChanger::checkChange: a continuous lane change must be completed before the next turn (and the target
    // lane must continue over that distance without a dangerous leader)
    int continuousChangeBlocked(int vehicle, int laneOffset, int targetLane, int neighLead, int state, int blockedByLeader) {
        int result = 0;
        const double distToNeighLane = 0.5 * (net.lanes[laneOf(vehicle)].width + net.lanes[targetLane].width);
        const double assumedDecel = std::max(1.0, -accel(vehicle));  // getAssumedDecelForLaneChangeDuration
        const double estimatedLCDuration = s.estimateLCDuration(vehicle, speed(vehicle), distToNeighLane, assumedDecel,
                                                                (state & LCA_URGENT) != 0);
        if (estimatedLCDuration == -1) {
            return LCA_INSUFFICIENT_SPEED;
        }
        const VType& t = T(vehicle);
        const double decel = t.decel * estimatedLCDuration;
        const double avgSpeed = 0.5 * (std::max(0., speed(vehicle) - ACCEL2SPEED(t.decel)) + std::max(0., speed(vehicle) - decel));
        const double space2change = avgSpeed * estimatedLCDuration;
        double seen = net.lanes[ch[candi].lane].length - pos(vehicle);
        const std::vector<int>& conts = s.bestContinuation(vehicle);
        int view = 1;
        int nextLane = laneOf(vehicle);
        int link = s.succLinkSec(vehicle, view, nextLane, conts);
        while (link >= 0 && seen <= space2change) {
            const Link& L = net.links[link];
            if (L.dir == 'l' || L.dir == 'r' || (net.lanes[nextLane].internal && net.lanes[L.viaOrLane()].internal)) {
                result |= LCA_INSUFFICIENT_SPACE;
                break;
            }
            if (L.via < 0) {
                view++;
            }
            nextLane = L.viaOrLane();
            seen += net.lanes[nextLane].length;
            link = s.succLinkSec(vehicle, view, nextLane, conts);
        }
        if (link < 0 && seen < space2change) {
            result |= LCA_INSUFFICIENT_SPACE;
        }
        if (((state | result) & LCA_BLOCKED) == 0) {
            // dangerous leaders when the target lane changes laterally before the midpoint
            seen = net.lanes[ch[candi].lane].length - pos(vehicle);
            nextLane = laneOf(vehicle);
            view = 1;
            const double dist = s.cfBrakeGap(vehicle, speed(vehicle)) + t.minGap;
            int nextLink = s.succLinkSec(vehicle, view, nextLane, conts);
            while (nextLink >= 0 && seen <= space2change && seen <= dist) {
                nextLane = net.links[nextLink].viaOrLane();
                const Edge& ne = net.edges[net.lanes[nextLane].edge];
                const int idx = net.lanes[nextLane].index + laneOffset;
                if (idx < 0 || idx >= (int)ne.lanes.size()) {
                    result |= LCA_INSUFFICIENT_SPACE;
                    break;
                }
                const VehNeighbour neighLead2 = getLeaderOn(ne.lanes[idx], vehicle, -seen, std::vector<int>(), -1);
                if (neighLead2.veh >= 0 && neighLead2.veh != neighLead
                        && neighLead2.gap < s.cfSecureGap(vehicle, neighLead2.veh, speed(vehicle), speed(neighLead2.veh), T(neighLead2.veh).decel)) {
                    result |= blockedByLeader;
                    break;
                }
                if (net.links[nextLink].via < 0) {
                    view++;
                }
                seen += net.lanes[nextLane].length;
                nextLink = s.succLinkSec(vehicle, view, nextLane, conts);
            }
        }
        return result;
    }

    // ---- LC2013 -----------------------------------------------------------------
    int& canceled(int laneOffset) { return s.veh[veh(ch[candi])].lc.canceled[laneOffset + 1]; }

    bool cancelRequest(int state, int laneOffset) {
        canceled(laneOffset) |= state;
        return false;
    }

    // MSAbstractLaneChangeModel::avoidOvertakeRight (no bus stop exceptions); allowProb: lcOvertakeRight permits
    // overtaking on the right with its probability
    bool avoidOvertakeRight(int v, bool allowProb = false) const {
        const double p = T(v).lcOvertakeRight;
        return !s.opt.overtakeRight && !congested(v) && s.veh[v].vclass != VC_EMERGENCY
               && (!allowProb || p == 0 || p < rand01(s.opt.seed, (uint64_t)v, (uint64_t)s.step, RP_LC_OVERTAKE_RIGHT));
    }

    // MSLCHelper::divergentRoute
    bool divergentRoute(int v1, int v2) const {
        const int l1 = laneOf(v1), l2 = laneOf(v2);
        return net.lanes[l1].internal && net.lanes[l2].internal
               && net.edges[net.lanes[l1].edge].fromJ == net.edges[net.lanes[l2].edge].fromJ
               && net.lanes[l1].edge != net.lanes[l2].edge;
    }

    // MSAbstractLaneChangeModel::canOvertakeRight
    bool canOvertakeRight(int v, int nv, double dist, double maxSpeedDiff, double helpOvertakeSpeed, double& vSafe, double& deltaV) {
        deltaV = std::max(maxSpeedDiff, speed(v) - speed(nv));
        if (deltaV > 0) {
            const double vMaxDecel = s.cfSpeedAfterMaxDecel(v, speed(v));
            const double vSafeFollow = s.cfFollowSpeed(v, speed(v), dist, speed(nv), T(nv).decel, -1, cf::CalcReason::CURRENT, true);
            const double vStayBehind = speed(nv) - helpOvertakeSpeed;
            if (vSafeFollow >= vMaxDecel) {
                vSafe = vSafeFollow;
            } else {
                vSafe = std::max(vMaxDecel, vStayBehind);
            }
            return true;
        }
        return false;
    }

    bool unwillingToHelp(int ego, double plannedSpeed, int nv) const {
        const VType& tn = T(nv);
        if (tn.lcCooperativeHelpTime < 0 || STEPS2TIME(s.veh[ego].waitingTime) < tn.lcCooperativeHelpTime) {
            if (tn.lcCooperativeHelpThreshold >= 0 && (speed(nv) - plannedSpeed) > tn.lcCooperativeHelpThreshold) {
                return true;
            }
            const double nvLaneMax = laneVehMaxSpeed(laneOf(nv), nv);
            if (speed(nv) / nvLaneMax < tn.lcCooperativeMinSpeed) {
                return true;
            }
        }
        return false;
    }

    int slowDownForBlocked(int v, int blockedVeh, int state) {
        if (blockedVeh >= 0) {
            const double gap = pos(blockedVeh) - T(blockedVeh).length - pos(v) - T(v).minGap;
            if (gap > POSITION_EPS) {
                if (speed(v) < T(v).decel) {
                    if (speed(blockedVeh) < HALTING_SPEED) {
                        state |= LCA_AMBACKBLOCKER_STANDING;
                    } else {
                        state |= LCA_AMBACKBLOCKER;
                    }
                    addLCSpeedAdvice(v, s.cfFollowSpeed(v, speed(v), gap - POSITION_EPS, speed(blockedVeh), T(blockedVeh).decel, -1, cf::CalcReason::CURRENT, true),
                                     LCA_CHANGE_TO_HELP);
                }
            }
        }
        return state;
    }

    bool saveBlockerLength(int blocker, double length, double foeLeftSpace) {
        const double leftSpace = live(blocker) ? s.veh[blocker].lc.leftSpace : s.lcLeftSpaceSnap[blocker];
        const double potential = leftSpace - s.cfBrakeGap(blocker, speed(blocker), T(blocker).decel, s.actionStepLengthSecs(blocker));
        const bool canReserve = potential >= length;
        if (canReserve || leftSpace > foeLeftSpace) {
            if (isOwn(blocker)) {
                double& l = s.veh[blocker].lc.leadingBlockerLength;
                l = std::max(length, l);
            } else {
                msgs.push_back({blocker, 1, length, 0});
            }
            return true;
        }
        return false;
    }

    bool updateBlockerLength(int v, int blocker, int lcaCounter, double leftSpace, bool reliefConnection, double& leadingBlockerLength) {
        if (blocker >= 0 && (ownStateOf(blocker) & lcaCounter) != 0) {
            const double required = T(blocker).lengthWithGap() + T(v).minGap;
            const double potential = leftSpace - s.cfBrakeGap(v, speed(v), T(v).decel, 0);
            if (required <= potential) {
                leadingBlockerLength = std::max(required, leadingBlockerLength);
            } else {
                const double required2 = T(v).lengthWithGap() + T(blocker).minGap;
                const double foeLeftSpace = leftSpace - pos(v) + pos(blocker) - POSITION_EPS;
                const bool canReserve = saveBlockerLength(blocker, required2, foeLeftSpace);
                if (!canReserve && !reliefConnection) {
                    const int blockerState = ownStateOf(blocker);
                    if ((blockerState & LCA_STRATEGIC) != 0 && (blockerState & LCA_URGENT) != 0) {
                        leadingBlockerLength = std::max(required, leadingBlockerLength);
                    }
                }
                return canReserve;
            }
        }
        return true;
    }

    double getExtraReservation(int bestLaneOffset, double neighExtraDist, int v) const {
        if (neighExtraDist > T(v).lengthWithGap()) {
            return 0;
        }
        if (bestLaneOffset < -1) {
            return 20;
        } else if (bestLaneOffset > 1) {
            return 40;
        }
        return 0;
    }

    double overtakeDistance(int follower, int leader, double gap, double followerSpeed, double leaderSpeed) const {
        followerSpeed = followerSpeed == INVALID_SPEED ? speed(follower) : followerSpeed;
        leaderSpeed = leaderSpeed == INVALID_SPEED ? speed(leader) : leaderSpeed;
        const double overtakeDist = (gap + T(leader).lengthWithGap() + T(follower).length
                                     + secureGap(leader, follower, leaderSpeed, followerSpeed, T(follower).decel));
        return std::max(overtakeDist, 0.);
    }

    bool isNormalOrRoundabout(int v) const {
        const int l = laneOf(v);
        return !net.lanes[l].internal || net.edges[s.curEdge(v)].roundabout;
    }

    double informLeader(int v, int blocked, int dir, const VehNeighbour& neighLead, double remainingSeconds) {
        LCState& lc = s.veh[v].lc;
        const VType& t = T(v);
        double plannedSpeed = speed(v);
        plannedSpeed = std::min(plannedSpeed, s.cfStopSpeed(v, speed(v), lc.leftSpace - lc.leadingBlockerLength, -1, cf::CalcReason::CURRENT));
        for (const LCAdvice& a : lc.advices) {
            if (a.accel >= -t.decel) {
                plannedSpeed = std::min(plannedSpeed, speed(v) + ACCEL2SPEED(a.accel));
            }
        }
        const int nv = neighLead.veh;
        if (nv < 0) {
            return plannedSpeed;
        }
        const double neighNextSpeed = std::min(speed(nv) - ACCEL2SPEED(std::max(1.0, -accel(nv))),
                                               s.veh[nv].stopDist <= net.lanes[laneOf(nv)].length ? s.cfMinNextSpeed(nv, speed(nv)) : speed(nv));
        const double neighNextGap = BALLISTIC ? neighLead.gap + SPEED2DIST((speed(nv) + neighNextSpeed) / 2) - SPEED2DIST((speed(v) + plannedSpeed) / 2)
                                    : neighLead.gap + SPEED2DIST(neighNextSpeed - plannedSpeed);
        if ((blocked & LCA_BLOCKED_BY_LEADER) != 0) {
            if (divergentRoute(v, nv)) {
                return plannedSpeed;
            }
            double overtakeTime;
            const double overtakeDist = overtakeDistance(v, nv, neighLead.gap, INVALID_SPEED, INVALID_SPEED);
            const double dv = plannedSpeed - speed(nv);
            const double laneSpeedLimit = net.lanes[laneOf(v)].speed;
            if (dv > t.lcOvertakeDeltaSpeedFactor * laneSpeedLimit) {
                overtakeTime = overtakeDist / dv;
            } else if (STEPS2TIME(s.veh[nv].waitingTime) > BLOCKER_IS_BLOCKED_TIME_THRESHOLD
                       && (t.lengthWithGap() + T(nv).lengthWithGap()) <= lc.leftSpace) {
                overtakeTime = remainingSeconds - 1;
            } else {
                overtakeTime = remainingSeconds + 1;
            }
            if ((dv < t.lcOvertakeDeltaSpeedFactor * laneSpeedLimit
                    || (dir == LCA_MLEFT && avoidOvertakeRight(v))
                    || (!BALLISTIC && lc.leftSpace - lc.leadingBlockerLength - s.cfBrakeGap(v, speed(v)) < overtakeDist)
                    // (the lane end does not move: the brake gap without headway)
                    || (BALLISTIC && lc.leftSpace - lc.leadingBlockerLength - s.cfBrakeGap(v, speed(v), t.decel, 0.) < overtakeDist)
                    || (remainingSeconds < overtakeTime && (!BALLISTIC || !isStopped(nv))))
                    && !isStopped(nv)) {
                inform(nv, DMAX, dir | LCA_AMBLOCKINGLEADER);
                const double targetSpeed = std::max(std::max(s.cfMinNextSpeed(v, speed(v)),
                                                    s.cfFollowSpeed(v, speed(v), neighNextGap, neighNextSpeed, T(nv).decel, -1, cf::CalcReason::CURRENT, true)),
                                                    isNormalOrRoundabout(v) ? 0 : ACCEL2SPEED(t.accel));
                if (targetSpeed < speed(v)) {
                    const double decel = remainingSeconds == 0. ? t.decel :
                                         std::min(t.decel, std::max(MIN_FALLBEHIND, (speed(v) - targetSpeed) / remainingSeconds));
                    const double nextSpeed = std::min(plannedSpeed, std::max(0.0, speed(v) - ACCEL2SPEED(decel)));
                    addLCSpeedAdvice(v, nextSpeed, dir);
                    return nextSpeed;
                } else {
                    addLCSpeedAdvice(v, targetSpeed, dir);
                    return plannedSpeed;
                }
            } else {
                if (STEPS2TIME(s.veh[nv].waitingTime) <= BLOCKER_IS_BLOCKED_TIME_THRESHOLD) {
                    inform(nv, speed(nv), dir | LCA_AMBLOCKINGLEADER);
                }
                return -1;
            }
        } else {
            const double targetSpeed = std::max(s.cfMinNextSpeed(v, speed(v)),
                                                s.cfFollowSpeed(v, speed(v), neighNextGap, neighNextSpeed, T(nv).decel, -1, cf::CalcReason::CURRENT, true));
            addLCSpeedAdvice(v, targetSpeed, dir);
            return std::min(targetSpeed, plannedSpeed);
        }
    }

    double speedPreservingSecureGap(int leader, int follower, double currentGap, double leaderPlannedSpeed) {
        const double nextGap = currentGap + SPEED2DIST(leaderPlannedSpeed - speed(follower));
        const double sGap = secureGap(follower, leader, speed(follower), leaderPlannedSpeed, T(leader).decel);
        if (nextGap >= sGap) {
            const double nextGapMin = currentGap + SPEED2DIST(leaderPlannedSpeed - s.cfMaxNextSpeed(follower, speed(follower)));
            const double vSafe = s.cfFollowSpeed(follower, speed(follower), nextGapMin, leaderPlannedSpeed, T(leader).decel, -1, cf::CalcReason::CURRENT, true);
            return std::max(vSafe, speed(follower));
        } else {
            const double vSafe = s.cfFollowSpeed(follower, speed(follower), nextGap, leaderPlannedSpeed, T(leader).decel, -1, cf::CalcReason::CURRENT, true);
            return std::max(vSafe, s.cfMinNextSpeed(follower, speed(follower)));
        }
    }

    void informFollower(int v, int blocked, int dir, const VehNeighbour& neighFollow, double remainingSeconds, double plannedSpeed) {
        const VType& t = T(v);
        const int nv = neighFollow.veh;
        if (nv >= 0 && unwillingToHelp(v, plannedSpeed, nv)) {
            return;
        }
        if ((blocked & LCA_BLOCKED_BY_FOLLOWER) != 0 && nv >= 0) {
            if (divergentRoute(v, nv)) {
                return;
            }
            const VType& tn = T(nv);
            if (std::max(plannedSpeed, 0.) - speed(nv) >= HELP_OVERTAKE) {
                const double neededGap = secureGap(nv, v, speed(nv), plannedSpeed, t.decel);
                if ((neededGap - neighFollow.gap) / remainingSeconds < (std::max(plannedSpeed, 0.) - speed(nv))) {
                    inform(nv, std::max(plannedSpeed, 0.) - HELP_OVERTAKE, dir | LCA_AMBLOCKINGFOLLOWER);
                    return;
                }
            }
            const double helpDecel = tn.decel * HELP_DECEL_FACTOR;
            const double plannedAccel = SPEED2ACCEL(std::max(std::min(t.accel, plannedSpeed - speed(v)), -t.decel));
            const double vMaxEgo = laneVehMaxSpeed(laneOf(v), v);
            const double vMaxNeigh = laneVehMaxSpeed(laneOf(nv), nv);
            double neighNewSpeed, neighNewSpeed1s, dv, decelGap;
            if (!BALLISTIC) {
                neighNewSpeed = std::max(0., speed(nv) - ACCEL2SPEED(helpDecel));
                neighNewSpeed1s = std::max(0., speed(nv) - helpDecel);
                dv = plannedSpeed - neighNewSpeed1s;
                decelGap = neighFollow.gap + dv;
            } else {
                // (negative speeds: a stop within the next step)
                neighNewSpeed = speed(nv) - ACCEL2SPEED(helpDecel);
                neighNewSpeed1s = speed(nv) - helpDecel;
                dv = speed(v) - speed(nv);
                decelGap = cf::gapExtrapolation(1., neighFollow.gap, speed(v), speed(nv), plannedAccel, -helpDecel, vMaxEgo, vMaxNeigh);
            }
            const double sGap = secureGap(nv, v, std::max(neighNewSpeed1s, 0.), std::max(plannedSpeed, 0.), t.decel);
            const double onRampThreshold = net.lanes[laneOf(v)].speed * 0.8 * t.lcExperimental1 * (1 - s.impatience(v));
            if (dir == LCA_MLEFT && s.laneAccel(laneOf(v)) && neighNewSpeed1s < onRampThreshold) {
                return;
            }
            if (decelGap > 0 && decelGap >= sGap) {
                double vsafe;
                if (!BALLISTIC) {
                    // an upper bound on vsafe: the computation twice
                    const double vsafe1 = std::max(neighNewSpeed, s.cfFollowSpeed(nv, speed(nv), neighFollow.gap + SPEED2DIST(plannedSpeed), plannedSpeed, t.decel, -1, cf::CalcReason::CURRENT, true));
                    vsafe = std::max(neighNewSpeed, s.cfFollowSpeed(nv, speed(nv), neighFollow.gap + SPEED2DIST(plannedSpeed - vsafe1), plannedSpeed, t.decel, -1, cf::CalcReason::CURRENT, true));
                } else {
                    // the next step's gap without help deceleration
                    double nextGap = cf::gapExtrapolation(TS, neighFollow.gap, speed(v), speed(nv), plannedAccel, 0, vMaxEgo, vMaxNeigh);
                    const double vsafe1 = std::min(speed(nv), std::max(neighNewSpeed, s.cfFollowSpeed(nv, speed(nv), nextGap, std::max(0., plannedSpeed), t.decel, -1, cf::CalcReason::CURRENT, true)));
                    const double decel2 = SPEED2ACCEL(speed(nv) - vsafe1);
                    nextGap = cf::gapExtrapolation(TS, neighFollow.gap, speed(v), speed(nv), plannedAccel, -decel2, vMaxEgo, vMaxNeigh);
                    vsafe = std::min(speed(nv), std::max(neighNewSpeed, s.cfFollowSpeed(nv, speed(nv), nextGap, std::max(0., plannedSpeed), t.decel, -1, cf::CalcReason::CURRENT, true)));
                    // a secure gap as soon as possible
                    if (nextGap < secureGap(nv, v, vsafe, plannedSpeed, t.decel)) {
                        vsafe = neighNewSpeed;
                    }
                }
                inform(nv, vsafe, dir | LCA_AMBLOCKINGFOLLOWER);
            } else if ((!BALLISTIC && dv > 0 && dv * remainingSeconds > (sGap - decelGap + POSITION_EPS))
                       || (BALLISTIC && dv > 0 && dv * (remainingSeconds - 1) > sGap - decelGap + POSITION_EPS)) {
                inform(nv, neighNewSpeed, dir | LCA_AMBLOCKINGFOLLOWER);
            } else if (dir == LCA_MRIGHT && avoidOvertakeRight(nv)) {
                const double vhelp = std::max(neighNewSpeed, HELP_OVERTAKE);
                inform(nv, vhelp, dir | LCA_AMBLOCKINGFOLLOWER);
            } else {
                double vhelp = std::max(speed(nv), speed(v) + HELP_OVERTAKE);
                if (speed(nv) > speed(v) &&
                        ((dir == LCA_MRIGHT && STEPS2TIME(s.veh[v].waitingTime) > LCA_RIGHT_IMPATIENCE)
                         || (dir == LCA_MLEFT && plannedSpeed > CUT_IN_LEFT_SPEED_THRESHOLD)
                         || (dir == LCA_MLEFT && net.lanes[laneOf(v)].length > MAX_ONRAMP_LENGTH))) {
                    vhelp = std::max(neighNewSpeed, speed(v) + HELP_OVERTAKE);
                    if (!BALLISTIC) {
                        if ((speed(nv) - speed(v)) / helpDecel < remainingSeconds) {
                            inform(nv, vhelp, dir | LCA_AMBLOCKINGFOLLOWER);
                            return;
                        }
                    } else {
                        // the gap after remainingSeconds (the leader keeps its speed, the follower brakes)
                        const double gapAfter = cf::gapExtrapolation(remainingSeconds, neighFollow.gap, speed(v), speed(nv), 0, -helpDecel, vMaxEgo, vMaxNeigh);
                        const double secureGapAfter = secureGap(nv, v, std::max(speed(nv) - remainingSeconds * helpDecel, 0.), speed(v), t.decel);
                        if (gapAfter >= secureGapAfter) {
                            inform(nv, neighNewSpeed, dir | LCA_AMBLOCKINGFOLLOWER);
                            return;
                        }
                    }
                }
                inform(nv, vhelp, dir | LCA_AMBLOCKINGFOLLOWER);
                const double overtakeDist = overtakeDistance(nv, v, neighFollow.gap, vhelp, plannedSpeed);
                const double needDV = overtakeDist / remainingSeconds;
                addLCSpeedAdvice(v, std::max(vhelp - needDV, speed(v) - ACCEL2SPEED(t.decel)), dir);
            }
        } else if (nv >= 0 && (blocked & LCA_BLOCKED_BY_LEADER)) {
            const double vsafe = speedPreservingSecureGap(v, nv, neighFollow.gap, plannedSpeed);
            inform(nv, vsafe, dir);
        }
    }

    double anticipateFollowSpeed(int v, const VehNeighbour& leaderDist, double dist, double vMax, bool acceleratingLeader) {
        const VType& t = T(v);
        const int leader = leaderDist.veh;
        const double gap = leaderDist.gap;
        double futureSpeed;
        if (acceleratingLeader) {
            const double maxSpeed1s = (speed(v) + t.accel - ACCEL2SPEED(t.accel));
            if (leader < 0) {
                futureSpeed = s.cfFollowSpeed(v, maxSpeed1s, dist, 0, 0, -1, cf::CalcReason::CURRENT, true);
            } else {
                futureSpeed = s.cfFollowSpeed(v, maxSpeed1s, gap, speed(leader), T(leader).decel, -1, cf::CalcReason::CURRENT, true);
            }
        } else {
            if (leader < 0) {
                futureSpeed = cf::maximumSafeStopSpeed(t, dist, t.decel, speed(v), true, -1, true);
            } else {
                futureSpeed = cf::maximumSafeFollowSpeed(t, gap, speed(v), speed(leader), T(leader).decel, true, true);
            }
        }
        futureSpeed = std::min(vMax, futureSpeed);
        if (leader >= 0 && gap > 0 && t.lcSpeedGainLookahead > 0) {
            const double futureLeaderSpeed = acceleratingLeader ? laneVehMaxSpeed(laneOf(leader), leader) : speed(leader);
            const double deltaV = vMax - futureLeaderSpeed;
            if (deltaV > 0 && gap > 0) {
                const double secGap = secureGap(v, leader, futureSpeed, speed(leader), t.decel);
                const double fullSpeedGap = gap - secGap;
                if (fullSpeedGap / deltaV < t.lcSpeedGainLookahead) {
                    const double gapClosingTime = std::max(0.0, fullSpeedGap / deltaV);
                    const double foreCastTime = t.lcSpeedGainLookahead * 2;
                    futureSpeed = std::min(futureSpeed, (gapClosingTime * futureSpeed + (foreCastTime - gapClosingTime) * futureLeaderSpeed) / foreCastTime);
                }
            }
        }
        return futureSpeed;
    }

    // MSLCM_LC2013::hasFreeLane
    bool hasFreeLane(int v, int laneOffset, const VehNeighbour& neighLeadStopped) {
        if (neighLeadStopped.veh < 0) {
            return true;
        }
        const int dir = laneOffset > 0 ? 1 : -1;
        const int vc = s.veh[v].vclass;
        const int neigh = net.parallel(laneOf(v), laneOffset);
        if (neigh < 0) {
            return false;
        }
        if (dir > 0 && (net.lanes[neigh].changeLeft & vc) != vc) {
            return false;
        } else if (dir < 0 && (net.lanes[neigh].changeRight & vc) != vc) {
            return false;
        }
        const int nextOffset = laneOffset + dir;
        const int next = net.parallel(laneOf(v), nextOffset);
        if (next < 0 || !net.lanes[next].allows(vc)) {
            return false;
        }
        const double overtakeDist = neighLeadStopped.gap + T(neighLeadStopped.veh).lengthWithGap() + T(v).length + POSITION_EPS;
        const VehNeighbour nextLead = getLeaderOn(next, v, pos(v), s.bestContinuationOf(v, next), overtakeDist);
        return nextLead.veh < 0 || nextLead.gap >= overtakeDist || hasFreeLane(v, nextOffset, nextLead);
    }

    // MSLCHelper::getRoundaboutDistBonus
    double roundaboutDistBonus(int v, double bonusParam, const LaneQ& curr, const LaneQ& neigh, const LaneQ& best) {
        const LaneQ& inner = net.lanes[neigh.lane].index > net.lanes[curr.lane].index ? neigh : curr;
        if (neigh.lane == inner.lane && curr.bestContinuations.size() < neigh.bestContinuations.size()) {
            return 0;
        }
        int roundaboutJunctionsAhead = 0;
        bool enteredRoundabout = false;
        double seen = -pos(v);
        for (int i = 0; i < (int)best.bestContinuations.size(); i++) {
            int lane = best.bestContinuations[i];
            if (lane < 0) {
                lane = laneOf(v);
            }
            const bool rb = net.edges[net.lanes[lane].edge].roundabout;
            if ((!enteredRoundabout || rb) && i >= (int)inner.bestContinuations.size()) {
                return 0;
            }
            if (seen > 300) {
                return 0;
            }
            const int junction = net.edges[net.lanes[lane].edge].toJ;
            if (rb) {
                enteredRoundabout = true;
                if (junction >= 0 && net.junctions[junction].nIncoming + net.junctions[junction].nOutgoing > 2) {
                    roundaboutJunctionsAhead++;
                }
            } else if (enteredRoundabout) {
                break;
            }
            seen += net.lanes[lane].length;
        }
        if (roundaboutJunctionsAhead < 2) {
            return 0;
        }
        double occupancyOuter = 0;
        double occupancyInner = 0;
        double distanceInRoundabout = 0;
        int prevNormal = -1;
        int prevInner = -1;
        enteredRoundabout = false;
        for (int i = 0; i < (int)best.bestContinuations.size(); i++) {
            const int lane = best.bestContinuations[i];
            if (lane < 0) {
                continue;
            }
            if (net.edges[net.lanes[lane].edge].roundabout) {
                enteredRoundabout = true;
            } else if (enteredRoundabout) {
                break;
            }
            int via = -1;
            if (prevNormal >= 0) {
                for (int l : net.lanes[prevNormal].links) {
                    if (net.links[l].lane == lane) {
                        via = net.links[l].via;
                    }
                }
            }
            if (enteredRoundabout) {
                distanceInRoundabout += net.lanes[lane].length;
                if (via >= 0) {
                    distanceInRoundabout += net.lanes[via].length;
                }
            }
            const double upstreamDiscount = net.lanes[lane].edge == net.lanes[laneOf(v)].edge
                                            ? (net.lanes[lane].length - pos(v)) / net.lanes[lane].length : 1;
            prevNormal = lane;
            occupancyOuter += upstreamDiscount * brutto(lane);
            if (via >= 0) {
                occupancyOuter += brutto(via);
            }
            if (i < (int)inner.bestContinuations.size()) {
                const int innerLane = inner.bestContinuations[i];
                if (innerLane >= 0) {
                    occupancyInner += upstreamDiscount * brutto(innerLane);
                    if (prevInner >= 0) {
                        for (int l : net.lanes[prevInner].links) {
                            if (net.links[l].lane == innerLane && net.links[l].via >= 0) {
                                occupancyInner += brutto(net.links[l].via);
                            }
                        }
                    }
                }
                prevInner = innerLane;
            }
        }
        if (std::abs(curr.bestLaneOffset) > 1 && enteredRoundabout) {
            const VType& t = T(v);
            const double bGap = s.cfBrakeGap(v, speed(v) + ACCEL2SPEED(t.accel), t.decel, s.actionStepLengthSecs(v));
            const double reservation = getExtraReservation(curr.bestLaneOffset, 0, v);
            const double leftSpace = distanceInRoundabout - reservation - pos(v);
            if (bGap > leftSpace) {
                return 0;
            }
        }
        const double maxOccupancy = std::max(occupancyInner, occupancyOuter);
        const double bonus = roundaboutJunctionsAhead * 7.5;
        const double relativeJam = (occupancyOuter - occupancyInner + bonus) / (maxOccupancy + bonus);
        double jamFactor = std::max(0.0, relativeJam);
        if (net.edges[net.lanes[laneOf(v)].edge].roundabout && net.lanes[curr.lane].index > net.lanes[neigh.lane].index) {
            jamFactor = 1;
        }
        return distanceInRoundabout * jamFactor * bonusParam * 9;
    }

    int wantsChange(int laneOffset, int blocked, const VehNeighbour& leader, const VehNeighbour& follower,
                    const VehNeighbour& neighLead, const VehNeighbour& neighFollow, int neighLane,
                    int lastBlocked, int firstBlocked) {
        const int v = veh(ch[candi]);
        Veh& x = s.veh[v];
        LCState& lc = x.lc;
        const VType& t = T(v);
        const std::vector<LaneQ>& preb = x.bestLanes.front();
        // SUMO copies the three LaneQ (with their continuation vectors); references suffice
        static const LaneQ noLaneQ;
        const LaneQ* currP = &noLaneQ;
        const LaneQ* neighP = &noLaneQ;
        const LaneQ* bestP = &noLaneQ;
        int bestLaneOffset = 0;
        double currentDist = 0;
        double neighDist = 0;
        int currIdx = 0;
        int prebLane = laneOf(v);
        if (net.lanes[prebLane].internal) {
            prebLane = net.links[net.lanes[prebLane].links[0]].lane;
        }
        const int prebOffset = laneOffset;
        for (int p = 0; p < (int)preb.size(); ++p) {
            if (preb[p].lane == prebLane && p + laneOffset >= 0) {
                currP = &preb[p];
                neighP = &preb[p + prebOffset];
                currentDist = currP->length;
                neighDist = neighP->length;
                bestLaneOffset = currP->bestLaneOffset;
                if (bestLaneOffset == 0 && preb[p + prebOffset].bestLaneOffset == 0) {
                    bestLaneOffset = prebOffset;
                }
                bestP = &preb[p + bestLaneOffset];
                currIdx = p;
                break;
            }
        }
        const LaneQ& curr = *currP;
        const LaneQ& neigh = *neighP;
        const LaneQ& best = *bestP;
        const bool right = (laneOffset == -1);
        const double posOnLaneV = pos(v);
        double driveToNextStop = -DMAX;
        if (x.stopDist < DMAX && !x.stops.empty() && net.lanes[x.stops.front().lane].edge == net.lanes[laneOf(v)].edge) {
            driveToNextStop = x.stopDist;
            const double stopPos = posOnLaneV + x.stopDist - x.lastStepDist;
            currentDist = std::max(currentDist, stopPos);
            neighDist = std::max(neighDist, stopPos);
        }
        const int lca = (right ? LCA_RIGHT : LCA_LEFT);
        const int myLca = (right ? LCA_MRIGHT : LCA_MLEFT);
        const int lcaCounter = (right ? LCA_LEFT : LCA_RIGHT);
        bool changeToBest = (right && bestLaneOffset < 0) || (!right && bestLaneOffset > 0);
        int ret = (lc.ownState & 0xffff0000);
        int req = 0;
        ret = slowDownForBlocked(v, lastBlocked, ret);
        if (lastBlocked != firstBlocked) {
            ret = slowDownForBlocked(v, firstBlocked, ret);
        }
        if (speed(v) > lc.lookAheadSpeed) {
            lc.lookAheadSpeed = speed(v);
        } else {
            const double memoryFactor = 1. - (1. - LOOK_AHEAD_SPEED_MEMORY) * s.actionStepLengthSecs(v);
            lc.lookAheadSpeed = std::max(LOOK_AHEAD_MIN_SPEED, (memoryFactor * lc.lookAheadSpeed + (1 - memoryFactor) * speed(v)));
        }
        double laDist = lc.lookAheadSpeed * LOOK_FORWARD * t.lcStrategic * (right ? 1 : t.lcLookaheadLeft);
        laDist += t.lengthWithGap() * 2.;
        const bool hasStoppedLeader = leader.veh >= 0 && isStopped(leader.veh) && leader.gap < (currentDist - posOnLaneV);
        const bool hasJunctionLeader = leader.veh >= 0 && leader.veh == neighLead.veh;
        if (bestLaneOffset == 0 && hasStoppedLeader) {
            laDist = 0.5 * (t.lengthWithGap() + T(leader.veh).lengthWithGap() + leader.gap);
        } else if (bestLaneOffset == laneOffset && neighLead.veh >= 0 && isStopped(neighLead.veh) && neighLead.gap < (currentDist - posOnLaneV)) {
            if (!hasStoppedLeader
                    && (neighLead.gap + t.lengthWithGap() + T(neighLead.veh).lengthWithGap()) < (currentDist - posOnLaneV)) {
                changeToBest = false;
            }
        }
        if (t.lcStrategic < 0) {
            laDist = -1e3;
        }
        const double roundaboutBonus = roundaboutDistBonus(v, t.lcCooperativeRoundabout < 0 ? t.lcCooperative : t.lcCooperativeRoundabout, curr, neigh, best);
        currentDist += roundaboutBonus;
        neighDist += roundaboutBonus;
        const int bestIdx = currIdx + bestLaneOffset;
        const double usableDist = std::max(currentDist - posOnLaneV - preb[bestIdx].occupation * JAM_FACTOR, driveToNextStop);
        const double maxJam = std::max(preb[currIdx + prebOffset].occupation, preb[currIdx].occupation);
        const double vMax = laneVehMaxSpeed(laneOf(v), v);
        const double neighVMax = laneVehMaxSpeed(neighLane, v);
        double thisLaneVSafe = vMax;
        const bool checkOverTakeRight = avoidOvertakeRight(v, true);
        if (v == s.traceVeh) {
            std::fprintf(stderr, "   wants dir=%d currDist=%.2f neighDist=%.2f usable=%.2f laDist=%.2f blo=%d currBlo=%d occBest=%.2f preb:",
                         laneOffset, currentDist, neighDist, usableDist, laDist, bestLaneOffset, curr.bestLaneOffset, preb[bestIdx].occupation);
            for (const LaneQ& q : preb) {
                std::fprintf(stderr, " [%s len=%.1f off=%d occ=%.1f cont=%zu]", net.lanes[q.lane].id.c_str(), q.length, q.bestLaneOffset, q.occupation, q.bestContinuations.size());
            }
            std::fprintf(stderr, "\n");
        }
        double neighLeftPlace = std::max(0.0, neighDist - posOnLaneV - maxJam);
        if (neighLead.veh >= 0 && isStopped(neighLead.veh)) {
            neighLeftPlace = std::min(neighLeftPlace, neighLead.gap);
        }
        bool changeLeftToAvoidOvertakeRight = false;
        if (changeToBest && bestLaneOffset == curr.bestLaneOffset
                && usableDist / std::abs(bestLaneOffset) < laDist) {
            ret = ret | lca | LCA_STRATEGIC | LCA_URGENT;
        } else {
            if (neighLead.veh >= 0 && checkOverTakeRight && !right) {
                const int nv = neighLead.veh;
                double deltaV = 0.;
                double vSafe = 0.;
                if (canOvertakeRight(v, nv, neighLead.gap, vMax - laneVehMaxSpeed(neighLane, nv), HELP_OVERTAKE, vSafe, deltaV)) {
                    if (lc.speedGainProbLeft < t.changeProbThresholdLeft) {
                        vSafe = std::max(vSafe, speed(nv));
                    }
                    thisLaneVSafe = std::min(thisLaneVSafe, vSafe);
                    addLCSpeedAdvice(v, vSafe, myLca);
                    const double deltaGapFuture = deltaV * 8;
                    const double vSafeFuture = s.cfFollowSpeed(v, speed(v), neighLead.gap - deltaGapFuture, speed(nv), T(nv).decel, -1, cf::CalcReason::CURRENT, true);
                    if (vSafeFuture < vSafe) {
                        const double relativeGain = deltaV / std::max(vMax, RELGAIN_NORMALIZATION_MIN_SPEED);
                        lc.speedGainProbLeft += (long long)(s.actionStepLengthSecs(v) * relativeGain * HYST_PRECISION);
                        changeLeftToAvoidOvertakeRight = true;
                    }
                }
            }
            const bool currFreeUntilNeighEnd = leader.veh < 0 || neighDist - posOnLaneV <= leader.gap;
            const double overtakeDist = leader.veh < 0 ? -1 : leader.gap + t.length + T(leader.veh).lengthWithGap();
            const double overtakeDist2 = (neighLead.veh < 0 || !isStopped(neighLead.veh)) ? -1
                                         : neighLead.gap + t.length + T(neighLead.veh).lengthWithGap();
            if (leader.veh >= 0 && isStopped(leader.veh) && leader.gap < REACT_TO_STOPPED_DISTANCE
                    && std::min(neighDist, currentDist) - posOnLaneV > overtakeDist
                    && (!checkOverTakeRight || !right)
                    && t.lcStrategic >= 0
                    && (neighLead.veh < 0 || !isStopped(neighLead.veh)
                        || neighLead.gap > overtakeDist
                        || (overtakeDist2 > leader.gap && hasFreeLane(v, laneOffset, neighLead)))) {
                currentDist = posOnLaneV + leader.gap;
                ret = ret | lca | LCA_STRATEGIC | LCA_URGENT;
            } else if (!changeToBest && neighLeftPlace / (std::abs(bestLaneOffset) + 2) < laDist) {
                ret = ret | LCA_STAY | LCA_STRATEGIC;
            } else if (bestLaneOffset == 0 && (neighLeftPlace * 2. < laDist)) {
                ret = ret | LCA_STAY | LCA_STRATEGIC;
            } else if (bestLaneOffset == 0
                       && (leader.veh < 0 || !isStopped(leader.veh))
                       && !net.lanes[neigh.bestContinuations.back()].links.empty()
                       && roundaboutBonus == 0
                       && ((t.lcStrategic >= 0 && neighDist < TURN_LANE_DIST)
                           || (t.lcStrategic < 0 && currFreeUntilNeighEnd))) {
                ret = ret | LCA_STAY | LCA_STRATEGIC;
            }
        }
        canceled(laneOffset) |= ret | blocked;
        if ((ret & lcaCounter) != 0) {
            ret &= ~(LCA_TRACI | lcaCounter | LCA_URGENT);
        }
        if ((ret & LCA_STAY) != 0) {
            return canceled(laneOffset);
        }
        if ((ret & LCA_URGENT) != 0) {
            lc.leftSpace = currentDist - posOnLaneV;
            if (changeToBest && std::abs(bestLaneOffset) > 1 && s.routes[x.route].size() - x.routeIdx > 1) {
                lc.leadingBlockerLength = std::max(getExtraReservation(bestLaneOffset, neighDist - currentDist, v), lc.leadingBlockerLength);
            }
            const bool canContinue = curr.bestContinuations.size() > 1;
            bool canReserve = updateBlockerLength(v, neighLead.veh, lcaCounter, lc.leftSpace - POSITION_EPS, canContinue, lc.leadingBlockerLength);
            if (firstBlocked != neighLead.veh) {
                canReserve &= updateBlockerLength(v, firstBlocked, lcaCounter, lc.leftSpace - POSITION_EPS, canContinue, lc.leadingBlockerLength);
            }
            if (!canReserve) {
                lc.dontBrake = canContinue;
            }
            const int remainingLanes = std::max(1, std::abs(bestLaneOffset));
            // SUMO writes MAX2(STEPS2TIME(TS), ...): TS is already in seconds, so the lower bound is TS / 1000
            const double remainingSeconds = std::max(TS / 1000., lc.leftSpace / std::max(lc.lookAheadSpeed, NUMERICAL_EPS) / remainingLanes / URGENCY);
            if (!hasJunctionLeader) {
                const double plannedSpeed = informLeader(v, blocked, myLca, neighLead, remainingSeconds);
                // (ballistic: negative speeds may indicate a stop request, -1 means overtaking)
                if (plannedSpeed >= 0 || (BALLISTIC && plannedSpeed != -1)) {
                    informFollower(v, blocked, myLca, neighFollow, remainingSeconds, plannedSpeed);
                }
            }
            return canceled(laneOffset);
        }
        const bool acceleratingLeader = (neighLead.veh >= 0 && accel(neighLead.veh) > 0)
                                        || (leader.veh >= 0 && accel(leader.veh) > 0);
        double neighLaneVSafe = std::min(neighVMax, anticipateFollowSpeed(v, neighLead, neighDist, neighVMax, acceleratingLeader));
        thisLaneVSafe = std::min(thisLaneVSafe, anticipateFollowSpeed(v, leader, currentDist, vMax, acceleratingLeader));
        double inconvenience = laneOffset < 0
                               ? (double)lc.speedGainProbLeft / (double)t.changeProbThresholdRight
                               : (double)lc.speedGainProbRight / (double)t.changeProbThresholdLeft;
        const double relSpeedDiff = thisLaneVSafe == 0 ? 0 : (thisLaneVSafe - neighLaneVSafe) / std::max(thisLaneVSafe, neighLaneVSafe);
        inconvenience = std::max(relSpeedDiff, inconvenience);
        inconvenience = std::min(1.0, inconvenience);
        const bool speedGainInconvenient = inconvenience > t.lcCooperative;
        if (roundaboutBonus > 0) {
            // (ballistic: a cooperative request only when the occupancy allows it)
            const bool neighOccupancyInconvenient = std::min(1., brutto(neigh.lane) / net.lanes[neigh.lane].length)
                                                    > std::min(1., brutto(curr.lane) / net.lanes[curr.lane].length);
            if (lca == LCA_LEFT) {
                if (!BALLISTIC || !neighOccupancyInconvenient) {
                    req = ret | lca | LCA_COOPERATIVE;
                }
            } else if (!BALLISTIC || neighOccupancyInconvenient) {
                req = ret | LCA_STAY | LCA_COOPERATIVE;
            }
            if (!cancelRequest(req, laneOffset)) {
                return ret | req;
            }
        }
        if (right) {
            if (bestLaneOffset == 0 && net.lanes[laneOf(v)].speed > 80. / 3.6 && lc.lookAheadSpeed > HALTING_SPEED) {
                req = ret | LCA_STAY | LCA_STRATEGIC;
                if (!cancelRequest(req, laneOffset)) {
                    return ret | req;
                }
            }
        }
        if ((lc.ownState & (LCA_AMBLOCKINGFOLLOWER | LCA_AMBLOCKINGFOLLOWER_DONTBRAKE)) != 0
                && (!speedGainInconvenient)
                && ((lc.ownState & myLca) != 0)
                && (changeToBest || neighDist / std::abs(std::abs(bestLaneOffset) + 1) > laDist)) {
            req = ret | lca | LCA_COOPERATIVE | LCA_URGENT;
            if (!cancelRequest(req, laneOffset)) {
                if ((blocked & LCA_BLOCKED_BY_LEFT_FOLLOWER) && !right && lc.speedGainProbLeft > (long long)(t.lcSpeedGainUrgency * HYST_PRECISION)) {
                    const int nv = neighFollow.veh;
                    if (nv >= 0 && !unwillingToHelp(v, speed(v), nv)) {
                        const double helpSpeed = std::max(s.cfMinNextSpeed(nv, speed(nv)), speed(v) - 1);
                        inform(nv, helpSpeed, myLca | LCA_AMBLOCKINGFOLLOWER);
                    }
                }
                return ret | req;
            }
        }
        if (s.edgeHasPersons(net.lanes[neighLane].edge)) {
            // react to pedestrians
            adaptSpeedToPedestrians(v, laneOf(v), thisLaneVSafe);
            adaptSpeedToPedestrians(v, neighLane, neighLaneVSafe);
        }
        const double relativeGain = (neighLaneVSafe - thisLaneVSafe) / std::max(neighLaneVSafe, RELGAIN_NORMALIZATION_MIN_SPEED);
        if (right) {
            if (thisLaneVSafe - 5 / 3.6 > neighLaneVSafe) {
                lc.speedGainProbRight = (long long)((double)lc.speedGainProbRight * std::pow(0.5, s.actionStepLengthSecs(v)));
            } else {
                lc.speedGainProbRight += (long long)(s.actionStepLengthSecs(v) * relativeGain * HYST_PRECISION);
                const double roadSpeedFactor = vMax / net.lanes[laneOf(v)].speed;
                double acceptanceTime;
                if (t.lcKeepRightAcceptanceTime == -1) {
                    acceptanceTime = 7 * roadSpeedFactor * std::max(1.0, speed(v));
                } else {
                    acceptanceTime = t.lcKeepRightAcceptanceTime * roadSpeedFactor;
                    if (follower.veh >= 0 && follower.gap < 2 * s.cfBrakeGap(follower.veh, speed(follower.veh))) {
                        if (speed(follower.veh) >= speed(v)) {
                            acceptanceTime *= std::max(1.0, speed(v)) / std::max(1.0, speed(follower.veh));
                            const double fRSF = laneVehMaxSpeed(laneOf(follower.veh), follower.veh) / net.lanes[laneOf(follower.veh)].speed;
                            if (fRSF > roadSpeedFactor) {
                                acceptanceTime /= fRSF;
                            }
                        }
                    }
                }
                double fullSpeedGap = std::max(0., neighDist - s.cfBrakeGap(v, vMax));
                double fullSpeedDrivingSeconds = std::min(acceptanceTime, fullSpeedGap / vMax);
                if (neighLead.veh >= 0 && speed(neighLead.veh) < vMax) {
                    fullSpeedGap = std::max(0., std::min(fullSpeedGap,
                                                         neighLead.gap - secureGap(v, neighLead.veh, vMax, speed(neighLead.veh), T(neighLead.veh).decel)));
                    fullSpeedDrivingSeconds = std::min(fullSpeedDrivingSeconds, fullSpeedGap / (vMax - speed(neighLead.veh)));
                }
                if (checkOverTakeRight && leader.veh >= 0
                        && laneVehMaxSpeed(laneOf(leader.veh), leader.veh) < vMax) {
                    fullSpeedGap = std::min(fullSpeedGap, leader.gap);
                    fullSpeedDrivingSeconds = std::min(fullSpeedDrivingSeconds, fullSpeedGap / (vMax - speed(leader.veh)));
                }
                const double deltaProb = (t.changeProbThresholdRight == std::numeric_limits<long long>::max()) ? 0 :
                                         ((double)t.changeProbThresholdRight * (fullSpeedDrivingSeconds / acceptanceTime) / KEEP_RIGHT_TIME);
                lc.keepRightProb -= (long long)(s.actionStepLengthSecs(v) * deltaProb);
                if ((long long)((double)lc.keepRightProb * t.lcKeepRight) < -t.changeProbThresholdRight) {
                    req = ret | lca | LCA_KEEPRIGHT;
                    if (!cancelRequest(req, laneOffset)) {
                        return ret | req;
                    }
                }
            }
            if (lc.speedGainProbRight > t.changeProbThresholdRight
                    && neighDist / std::max(.1, speed(v)) > t.lcSpeedGainRemainTime) {
                req = ret | lca | LCA_SPEEDGAIN;
                if (lc.speedGainProbRight > (long long)(t.lcSpeedGainUrgency * HYST_PRECISION)) {
                    req |= LCA_URGENT;
                }
                if (!cancelRequest(req, laneOffset)) {
                    return ret | req;
                }
            }
        } else {
            if (thisLaneVSafe > neighLaneVSafe) {
                lc.speedGainProbLeft = (long long)((double)lc.speedGainProbLeft * std::pow(0.5, s.actionStepLengthSecs(v)));
            } else if (thisLaneVSafe == neighLaneVSafe) {
                lc.speedGainProbLeft = (long long)((double)lc.speedGainProbLeft * std::pow(0.8, s.actionStepLengthSecs(v)));
            } else {
                lc.speedGainProbLeft += (long long)(s.actionStepLengthSecs(v) * relativeGain * HYST_PRECISION);
            }
            if (lc.speedGainProbLeft > t.changeProbThresholdLeft
                    && (relativeGain > NUMERICAL_EPS || changeLeftToAvoidOvertakeRight)
                    && neighDist / std::max(.1, speed(v)) > t.lcSpeedGainRemainTime) {
                req = ret | lca | LCA_SPEEDGAIN;
                if (lc.speedGainProbLeft > (long long)(t.lcSpeedGainUrgency * HYST_PRECISION)) {
                    req |= LCA_URGENT;
                }
                if (!cancelRequest(req, laneOffset)) {
                    if ((req & LCA_URGENT) && (blocked & LCA_BLOCKED_BY_LEFT_FOLLOWER)) {
                        const int nv = neighFollow.veh;
                        if (nv >= 0 && !unwillingToHelp(v, speed(v), nv)) {
                            const double helpSpeed = std::max(s.cfMinNextSpeed(nv, speed(nv)), speed(v) - 1);
                            inform(nv, helpSpeed, myLca | LCA_AMBLOCKINGFOLLOWER);
                        }
                    }
                    return ret | req;
                }
            }
        }
        if (changeToBest && bestLaneOffset == curr.bestLaneOffset
                && t.lcStrategic >= 0
                && relativeGain >= 0
                && (right ? lc.speedGainProbRight : lc.speedGainProbLeft) > 0) {
            req = ret | lca | LCA_STRATEGIC;
            if (!cancelRequest(req, laneOffset)) {
                return ret | req;
            }
        }
        return ret;
    }
};

}  // namespace ps
