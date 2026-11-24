// Sublane lane changing (--lateral-resolution): MSLaneChangerSublane + MSLCM_SL2015 (SUMO 1.28), without opposite
// direction driving, bidirectional lanes, TraCI and maneuver reservations.
//
// The sweep, the stages and the snapshots of other edges' vehicles are those of the LC2013 lane changer (lc.cpp,
// lc.hpp). Per lane the changer keeps the leaders seen so far by sublane (MSLaneChangerSublane::ChangeElem::ahead)
// and the vehicles driving outside the lane bounds.
#include <cmath>

#include "lc.hpp"
#include "sublanequery.hpp"

namespace ps {

namespace {
constexpr double GAIN_PERCEPTION_THRESHOLD = 0.05;
constexpr double ARRIVALPOS_LAT_THRESHOLD = 100.0;
constexpr double LATGAP_SPEED_THRESHOLD = 50 / 3.6;
constexpr double LATGAP_SPEED_THRESHOLD2 = 50 / 3.6;
constexpr double SPEEDGAIN_DECAY_FACTOR = 0.5;
constexpr double SPEEDGAIN_MEMORY_FACTOR = 0.5;
constexpr double NO_NEIGHBOR = std::numeric_limits<double>::max();  // MSAbstractLaneChangeModel::NO_NEIGHBOR
constexpr double SUMO_LANE_WIDTH = 3.2;  // SUMO_const_laneWidth

using CLeaderDist = std::pair<int, double>;

// MSAbstractLaneChangeModel::StateAndDist
struct StateAndDist {
    int state = 0;
    double latDist = 0;
    double maneuverDist = 0;
    int dir = 0;
    bool sameDirection(const StateAndDist& other) const { return latDist * other.latDist > 0; }
};

// the sublane parts of MSLaneChangerSublane's ChangeElem
struct SubElem {
    LeaderInfo ahead;
    LeaderDistanceInfo aheadNext;
    std::vector<int> outsideBounds;
    std::vector<int> siblings;  // (index offsets of internal lanes with the same origin lane)
};
}  // namespace

struct SublaneChanger : LaneChanger {
    std::vector<SubElem>& se;

    SublaneChanger(Sim& sim, int e, std::vector<LCMessage>& m, std::vector<LaneChangeEvent>& ev, std::vector<DetEvent>& d,
                   std::vector<int>& changed)
        : LaneChanger(sim, e, m, ev, d, changed), se(subElems(sim.net.edges[e].lanes.size())) {}

    static std::vector<SubElem>& subElems(size_t nLanes) {
        thread_local std::vector<std::vector<SubElem>> byCount;
        if (byCount.size() <= nLanes) {
            byCount.resize(nLanes + 1);
        }
        std::vector<SubElem>& v = byCount[nLanes];
        if (v.size() != nLanes) {
            v.resize(nLanes);
        }
        return v;
    }

    double laneW(int lane) const { return net.lanes[lane].width; }
    int edgeOf(int lane) const { return net.lanes[lane].edge; }
    SLState& SL(int v) { return s.sl[v]; }
    const SLState& SL(int v) const { return s.sl[v]; }

    // ---- lane queries during lane changing ---------------------------------------------------
    // MSLane::AnyVehicleIterator (downstream): the lanes of this edge as they are being changed (unprocessed
    // vehicles, then the processed ones), merged with the partial occupants by position
    template <class F>
    void anyVehiclesLC(int lane, F f) {
        ChangeElem* ce = elemOf(lane);
        std::vector<int> buf;
        const std::vector<int>& p = partialsLC(lane, buf);
        const std::vector<int>& vehs = s.lanes[lane].vehs;
        const size_t nWork = ce == nullptr ? vehs.size() : ce->work.size();
        const size_t nFull = ce == nullptr ? vehs.size() : nWork + ce->done.size();
        auto full = [&](size_t i) {
            return ce == nullptr ? vehs[i] : (i < nWork ? ce->work[i] : ce->done[ce->done.size() - 1 - (i - nWork)]);
        };
        size_t i1 = 0, i2 = 0;
        while (i1 < nFull || i2 < p.size()) {
            bool takeFull;
            if (i1 == nFull) {
                takeFull = false;
            } else if (i2 == p.size()) {
                takeFull = true;
            } else {
                takeFull = pos(full(i1)) < posOnLane(p[i2], lane);
            }
            const int u = takeFull ? full(i1++) : p[i2++];
            if (!f(u)) {
                return;
            }
        }
    }

    // MSLane::getLastVehicleInformation, getFirstVehicleInformation, getFollowersOnConsecutive (allSublanes)
    LeaderInfo lastVehicleInfo(int lane, int ego, double latOffset, double minPos, int ignore = -1) {
        return s.lastVehicleInformationT(lane, ego, latOffset, minPos, ignore, [this](int l, auto f) { anyVehiclesLC(l, f); });
    }
    CriticalFollowerDistanceInfo followersOnConsecutive(int lane, int ego, double backOffset) {
        return s.followersOnConsecutiveT(lane, ego, backOffset, true, true, [this](int l, auto f) { anyVehiclesLC(l, f); });
    }

    // MSLane::getLeadersOnConsecutive (gComputeLC)
    void getLeadersOnConsecutive(int lane, double dist, double seen, double spd, int ego, const std::vector<int>& conts,
                                 LeaderDistanceInfo& result) {
        if (seen > dist && !net.lanes[lane].internal) {
            return;
        }
        // partial occupants (they might be on a different route)
        std::vector<int> buf;
        for (int u : partialsLC(lane, buf)) {
            if (!s.isFrontOnLane(u, lane)) {
                result.addLeader(u, seen, s.latOffset(u, lane));
            } else {
                break;
            }
        }
        thread_local std::vector<Sim::LinkLeader> linkLeaders;
        int nextLane = lane;
        int view = 1;
        while ((seen < dist && result.numFreeSublanes() > 0) || net.lanes[nextLane].internal) {
            if (nextLane != lane) {
                seen += net.lanes[nextLane].length;
            }
            const int link = s.succLinkSec(ego, view, nextLane, conts);
            if (link < 0) {
                break;
            }
            s.leaderInfo(link, ego, seen, linkLeaders, true);
            for (const Sim::LinkLeader& ll : linkLeaders) {
                const int u = ll.veh;
                if (u >= 0 && (s.isLeader(ego, link, u, ll.gap)
                               || s.vehPosition(u).distanceTo2D(s.vehPosition(ego)) - T(u).minGap - T(ego).length
                               < s.cfBrakeGap(u, speed(u)))) {
                    if ((ll.flags & (Sim::LL_SAME_TARGET | Sim::LL_SAME_SOURCE)) != 0) {
                        result.addLeader(u, ll.gap, ll.latOffset);
                    } else {
                        // add the link leader to all sublanes
                        for (int i = 0; i < result.numSublanes(); ++i) {
                            result.addLeader(u, ll.gap, 0, i);
                        }
                    }
                }
            }
            const bool nextInternal = net.links[link].via >= 0;
            nextLane = net.links[link].viaOrLane();
            const LeaderInfo leaders = lastVehicleInfo(nextLane, -1, 0, 0);
            const int iMax = std::min(leaders.numSublanes(), result.numSublanes());
            for (int i = 0; i < iMax; ++i) {
                const int u = leaders[i];
                if (u >= 0) {
                    result.addLeader(u, seen - T(ego).minGap + backPosOnLane(u, nextLane), 0, i);
                }
            }
            if (laneVehMaxSpeed(nextLane, ego) < spd) {
                dist = s.cfBrakeGap(ego, laneVehMaxSpeed(nextLane, ego));
            }
            if (!nextInternal) {
                view++;
            }
        }
    }

    // MSLane::addLeaders
    void addLeaders(int lane, int vehicle, double vehPos, LeaderDistanceInfo& result) {
        // vehicles on the target lane with the same position as ego may not have been added to 'ahead' yet
        const LeaderInfo aheadSamePos = lastVehicleInfo(lane, -1, 0, vehPos, vehicle);

        for (int i = 0; i < aheadSamePos.numSublanes(); ++i) {
            const int u = aheadSamePos[i];
            if (u >= 0 && u != vehicle) {
                result.addLeader(u, backPosOnLane(u, lane) - vehPos - T(vehicle).minGap, 0, i);
            }
        }
        // link leaders (getLeadersOnConsecutive) while on a junction
        if (result.numFreeSublanes() > 0 || net.lanes[lane].internal) {
            const double seen = net.lanes[laneOf(vehicle)].length - vehPos;
            const double spd = speed(vehicle);
            const double dist = std::max(s.cfBrakeGap(vehicle, spd), 10.0) + T(vehicle).minGap;
            if (seen > dist && !net.lanes[lane].internal) {
                return;
            }
            getLeadersOnConsecutive(lane, dist, seen, spd, vehicle, s.bestContinuationOf(vehicle, lane), result);
        }
    }

    // ---- the sweep ---------------------------------------------------------------------------
    // MSLaneChangerSublane::initChanger (with the sibling lanes of the constructor)
    void initSub() {
        init();
        const Edge& e = net.edges[edge];
        for (int i = 0; i < (int)ch.size(); ++i) {
            SubElem& x = se[i];
            const int lane = ch[i].lane;
            // MSLane::getPartialBeyond
            x.ahead = LeaderInfo(s, laneW(lane));
            for (int u : s.lanes[lane].partials) {
                if (!s.isFrontOnLane(u, lane)) {
                    x.ahead.addLeader(u, false, s.latOffset(u, lane));
                } else {
                    break;
                }
            }
            x.outsideBounds.clear();
            x.siblings.clear();
            if (e.internal) {
                for (int j = 0; j < (int)ch.size(); ++j) {
                    if (j != i && net.lanes[lane].incoming.front().lane == net.lanes[ch[j].lane].incoming.front().lane) {
                        x.siblings.push_back(j - i);
                    }
                }
            }
        }
    }

    void laneChangeSublane() {
        struct ComputeLC {
            explicit ComputeLC(int e) {
                cfComputeLC = true;
                lcOwnEdge = e;
            }
            ~ComputeLC() {
                cfComputeLC = false;
                lcOwnEdge = -1;
            }
        } computeLC(edge);
        initSub();
        while (vehInChanger()) {
            const bool changed = changeSublane();
            updateChangerSublane(changed);
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

    // MSLaneChangerSublane::updateChanger
    void updateChangerSublane(bool vehHasChanged) {
        updateChanger(vehHasChanged);
        if (!vehHasChanged) {
            const int lead = ch[candi].lead;
            const int lane = ch[candi].lane;
            if (s.leftSideOnLane(lead, lane) < 0 || s.rightSideOnLane(lead, lane) > laneW(lane)) {
                se[candi].outsideBounds.push_back(lead);
            } else {
                se[candi].ahead.addLeader(lead, false, 0);
            }
            const int shadowLane = K(lead).shadowLane;
            if (shadowLane >= 0 && edgeOf(shadowLane) == edgeOf(laneOf(lead))) {
                const double latOffset = net.lanes[laneOf(lead)].rightSideOnEdge - net.lanes[shadowLane].rightSideOnEdge;
                se[net.lanes[shadowLane].index].ahead.addLeader(lead, false, latOffset);
            }
        }
    }

    // MSLaneChangerSublane::change
    bool changeSublane() {
        candi = findCandidate();
        const int vehicle = veh(ch[candi]);
        static const bool traceOrder = getenv("PSIM2_TRACE_LC_ORDER") != nullptr;
        if (traceOrder) {
            std::fprintf(stderr, "LCORDER %.2f %s %s pos=%.6f posLat=%.3f\n", STEPS2TIME(s.now), net.edges[edge].id.c_str(), s.veh[vehicle].id.c_str(),
                         pos(vehicle), s.kin[vehicle].posLat);
        }
        Veh& x = s.veh[vehicle];
        const bool stoppedOnLane = isStopped(vehicle) && x.stops.front().lane == laneOf(vehicle);
        if (x.lc.changed || stoppedOnLane) {
            registerUnchanged(vehicle);
            if (stoppedOnLane) {
                ch[candi].lastStopped = vehicle;
            }
            return false;
        }
        if (!x.actionStep) {
            // resume the maneuver between actions
            const bool changed = continueChangeSublane(vehicle);
            if (!changed) {
                registerUnchanged(vehicle);
            }
            return changed;
        }
        updateOccupancy(vehicle);
        std::vector<LaneQ>& preb = x.bestLanes.front();
        for (int i = 0; i < (int)ch.size() && i < (int)preb.size(); ++i) {
            preb[i].occupation = ch[i].dens + preb[i].nextOccupation;
        }
        // the leaders beyond the current edge for all lanes
        for (int i = 0; i < (int)ch.size(); ++i) {
            se[i].aheadNext = getLeaders(i, vehicle);
        }
        // the expected speeds
        int sublaneIndex = 0;
        for (int i = 0; i < (int)ch.size(); ++i) {
            updateExpectedSublaneSpeeds(vehicle, se[i].aheadNext, sublaneIndex, i);
            for (int offset : se[i].siblings) {
                // sibling lanes (internal lanes with the same origin lane) are treated as if they had the same geometry
                if (net.lanes[ch[i + offset].lane].allows(x.vclass)) {
                    updateExpectedSublaneSpeeds(vehicle, se[i + offset].aheadNext, sublaneIndex, i + offset);
                }
            }
            sublaneIndex += se[i].ahead.numSublanes();
        }
        const int alternatives = (mayChange(-1) ? LCA_RIGHT : LCA_NONE) | (mayChange(1) ? LCA_LEFT : LCA_NONE);
        StateAndDist right = checkChangeHelper(vehicle, -1, alternatives);
        StateAndDist left = checkChangeHelper(vehicle, 1, alternatives);
        StateAndDist current = checkChangeHelper(vehicle, 0, alternatives);
        StateAndDist decision = decideDirection(vehicle, current, decideDirection(vehicle, right, left));
        if (vehicle == s.traceVeh) {
            std::fprintf(stderr, "%.2f SL %s lane=%s pos=%.3f posLat=%.3f speedLat=%.3f right=(%x %.3f %.3f) left=(%x %.3f %.3f) "
                         "current=(%x %.3f %.3f) decision=(%x %.3f %.3f) safeR=%.3f safeL=%.3f pR=%.4f pL=%.4f\n",
                         STEPS2TIME(s.now), x.id.c_str(), net.lanes[laneOf(vehicle)].id.c_str(), pos(vehicle), s.kin[vehicle].posLat,
                         x.lc.speedLat, right.state, right.latDist, right.maneuverDist, left.state, left.latDist, left.maneuverDist,
                         current.state, current.latDist, current.maneuverDist, decision.state, decision.latDist, decision.maneuverDist,
                         SL(vehicle).safeLatDistRight, SL(vehicle).safeLatDistLeft, SL(vehicle).speedGainProbRight,
                         SL(vehicle).speedGainProbLeft);
        }
        setOwnStateSL(vehicle, decision.state);
        if ((decision.state & LCA_WANTS_LANECHANGE) != 0 && (decision.state & LCA_BLOCKED) == 0) {
            const bool changed = startChangeSublane(vehicle, decision.latDist, decision.maneuverDist);
            if (!changed) {
                registerUnchanged(vehicle);
            }
            return changed;
        }
        abortLCManeuver(vehicle);
        registerUnchanged(vehicle);
        return false;
    }

    // MSLCM_SL2015::setOwnState
    void setOwnStateSL(int v, int state) {
        LCState& lc = s.veh[v].lc;
        lc.previousState = state;
        lc.ownState = state;
        SLState& st = SL(v);
        const VType& t = T(v);
        if (s.veh[v].actionStep) {
            if ((state & (LCA_STRATEGIC | LCA_SPEEDGAIN)) != 0 && (state & LCA_BLOCKED) != 0) {
                st.impatience = std::min(1.0, st.impatience + s.actionStepLengthSecs(v) / t.lcTimeToImpatience);
            } else {
                // impatience decays only to the driver-specific level
                st.impatience = std::max(t.lcImpatience, st.impatience - s.actionStepLengthSecs(v) / t.lcTimeToImpatience);
            }
            if ((state & LCA_STAY) != 0) {
                st.canChangeFully = true;
            }
        }
    }

    // MSLaneChangerSublane::abortLCManeuver (no lane change output for the ended maneuver)
    void abortLCManeuver(int v) {
        LCState& lc = s.veh[v].lc;
        setSpeedLat(v, 0);
        setManeuverDist(v, 0.);
        (void)lc;
    }

    // MSAbstractLaneChangeModel::setSpeedLat / setManeuverDist
    void setSpeedLat(int v, double speedLat) {
        LCState& lc = s.veh[v].lc;
        SL(v).accelerationLat = SPEED2ACCEL(speedLat - lc.speedLat);
        lc.speedLat = speedLat;
    }
    void setManeuverDist(int v, double dist) {
        LCState& lc = s.veh[v].lc;
        lc.maneuverDist = std::fabs(dist) < NUMERICAL_EPS ? 0. : dist;
        SL(v).previousManeuverDist = lc.maneuverDist;
    }

    // MSLaneChangerSublane::checkChangeHelper
    StateAndDist checkChangeHelper(int vehicle, int laneOffset, int alternatives) {
        StateAndDist result;
        if (mayChange(laneOffset)) {
            result.state = checkChangeSublane(vehicle, laneOffset, alternatives, result.latDist, result.maneuverDist);
            result.dir = laneOffset;
            if ((result.state & LCA_WANTS_LANECHANGE) != 0 && (result.state & LCA_URGENT) != 0 && (result.state & LCA_BLOCKED) != 0) {
                ch[candi + laneOffset].lastBlocked = vehicle;
                if (ch[candi + laneOffset].firstBlocked < 0) {
                    ch[candi + laneOffset].firstBlocked = vehicle;
                }
            }
        }
        return result;
    }

    // MSLaneChangerSublane::continueChangeSublane
    bool continueChangeSublane(int v) {
        LCState& lc = s.veh[v].lc;
        const double remLatDist = lc.maneuverDist;
        if (remLatDist == 0) {
            return false;
        }
        const bool urgent = (lc.ownState & LCA_URGENT) != 0;
        double maneuverDist = remLatDist;
        const double nextLatDist = SPEED2DIST(computeSpeedLat(v, remLatDist, maneuverDist, urgent));
        return startChangeSublane(v, nextLatDist, remLatDist);
    }

    // MSLaneChangerSublane::startChangeSublane
    bool startChangeSublane(int v, double latDist, double maneuverDist) {
        Kin& k = s.kin[v];
        LCState& lc = s.veh[v].lc;
        const int source = ch[candi].lane;
        const double distToRightLaneBorder = laneW(k.lane) * 0.5 + k.posLat - T(v).width * 0.5;
        const double distToLeftLaneBorder = laneW(k.lane) * 0.5 - k.posLat - T(v).width * 0.5;
        int direction = 0;
        if (latDist > 0 && latDist > distToLeftLaneBorder) {
            direction = 1;
        } else if (latDist < 0 && -latDist > distToRightLaneBorder) {
            direction = -1;
        }
        int to = candi;
        if (mayChange(direction)) {
            to = candi + direction;
        } else {
            // (may occur during maneuver continuation between actions)
            abortLCManeuver(v);
            return false;
        }
        k.posLat += latDist;
        LatState& ls = s.lat[v];
        for (int i = 0; i < k.nFurther; i++) {
            ls.furtherPosLat[i] += latDist;
        }
        setSpeedLat(v, DIST2SPEED(latDist));
        const bool completedManeuver = std::fabs(maneuverDist - latDist) < NUMERICAL_EPS;
        updateSafeLatDist(v, latDist);
        (void)lc;
        setManeuverDist(v, maneuverDist - latDist);
        const bool changedToNewLane = checkChangeToNewLane(v, direction, candi, to);
        const int oldShadowLane = k.shadowLane;
        s.updateShadowLane(v);
        const int shadowLane = k.shadowLane;
        if (v == s.traceVeh) {
            std::fprintf(stderr, "    startChange posLat=%.3f lane=%s old shadow=%d new=%d changed=%d\n", k.posLat, net.lanes[k.lane].id.c_str(),
                         oldShadowLane, shadowLane, (int)changedToNewLane);
        }
        if (shadowLane >= 0 && shadowLane != oldShadowLane && edgeOf(shadowLane) == edgeOf(source)) {
            const double latOffset = net.lanes[k.lane].rightSideOnEdge - net.lanes[shadowLane].rightSideOnEdge;
            se[net.lanes[shadowLane].index].ahead.addLeader(v, false, latOffset);
        }
        // (maneuver reservations of vehicles with longer action steps: not modelled)
        if (completedManeuver) {
            // MSVehicle::setAngle(.., straightenFurther): the further lanes are straightened
            int next = k.lane;
            for (int i = 0; i < k.nFurther; i++) {
                const Link* link = net.linkTo(k.further[i], next);
                if (link != nullptr) {
                    ls.furtherPosLat[i] = k.posLat - link->lateralShift;
                    next = k.further[i];
                } else {
                    break;
                }
            }
        }
        s.lcShadowPerEdge[edge].push_back(v);  // (the partial occupations follow at the end of the stage)
        markDirty(v);
        return changedToNewLane;
    }

    // MSLaneChangerSublane::checkChangeToNewLane
    bool checkChangeToNewLane(int v, int direction, int from, int to) {
        Kin& k = s.kin[v];
        const bool changedToNewLane = to != from && std::fabs(k.posLat) > 0.5 * laneW(k.lane) && mayChange(direction);
        if (changedToNewLane) {
            k.posLat -= direction * 0.5 * (laneW(ch[from].lane) + laneW(ch[to].lane));
            // MSAbstractLaneChangeModel::startLaneChangeManeuver -> primaryLaneChanged (registers the vehicle on the
            // target lane)
            primaryLaneChanged(v, direction);
            se[to].ahead.addLeader(v, false, 0);
        } else {
            se[from].ahead.addLeader(v, false, 0);
        }
        return changedToNewLane;
    }

    // MSLaneChangerSublane::getLeaders
    LeaderDistanceInfo getLeaders(int target, int vehicle) {
        const int targetLane = ch[target].lane;
        LeaderDistanceInfo result(s, laneW(targetLane), -1, 0);
        int sublaneShift = 0;
        if (targetLane == laneOf(vehicle)) {
            const double left = s.leftSideOnLane(vehicle, laneOf(vehicle));
            const double right = s.rightSideOnLane(vehicle, laneOf(vehicle));
            if (left < -s.latRes) {
                sublaneShift = int(-left / s.latRes);
            } else if (right > laneW(targetLane) + s.latRes) {
                sublaneShift = -int((right - laneW(targetLane)) / s.latRes);
            }
            result.setSublaneOffset(sublaneShift);
        }
        const LeaderInfo& ahead = se[target].ahead;
        for (int i = 0; i < ahead.numSublanes(); ++i) {
            const int u = ahead[i];
            if (u >= 0) {
                const double gap = backPosOnLane(u, targetLane) - pos(vehicle) - T(vehicle).minGap;
                if (i + sublaneShift >= 0 && i + sublaneShift < result.numSublanes()) {
                    result.addLeader(u, gap, 0, i + sublaneShift);
                }
            }
        }
        if (sublaneShift != 0) {
            for (int cand : se[target].outsideBounds) {
                const double gap = backPosOnLane(cand, laneOf(cand)) - pos(vehicle) - T(vehicle).minGap;
                result.addLeader(cand, gap);
            }
        }
        // (the exact position of getLastVehicleInformation: vehicles at the same position are not missed)
        addLeaders(targetLane, vehicle, posOnLane(vehicle, laneOf(vehicle)), result);
        if (vehicle == s.traceVeh) {
            std::fprintf(stderr, "    getLeaders %s:", net.lanes[targetLane].id.c_str());
            for (int i = 0; i < result.numSublanes(); ++i) {
                std::fprintf(stderr, " %s/%.2f", result[i] < 0 ? "-" : s.veh[result[i]].id.c_str(), result[i] < 0 ? 0. : result.at(i).second);
            }
            std::fprintf(stderr, " (ahead:");
            for (int i = 0; i < ahead.numSublanes(); ++i) {
                std::fprintf(stderr, " %s", ahead[i] < 0 ? "-" : s.veh[ahead[i]].id.c_str());
            }
            std::fprintf(stderr, ")\n");
        }
        return result;
    }

    // MSLaneChangerSublane::addOutsideLeaders
    void addOutsideLeaders(int vehicle, LeaderDistanceInfo& leaders) {
        if (K(vehicle).shadowLane >= 0) {
            return;
        }
        const int lane = laneOf(vehicle);
        const double rightOL = s.rightSideOnLane(vehicle, lane);
        const double leftOL = s.leftSideOnLane(vehicle, lane);
        const bool outsideLeft = rightOL > laneW(lane);
        if (leftOL < 0 || outsideLeft) {
            int sublaneOffset = 0;
            if (outsideLeft) {
                sublaneOffset = std::min(0, -(int)std::ceil((leftOL - laneW(lane)) / s.latRes) + 1);
            } else {
                sublaneOffset = std::max(0, (int)std::ceil(-rightOL / s.latRes) - 1);
            }
            if (sublaneOffset != 0) {
                leaders.setSublaneOffset(sublaneOffset);
                const double edgeWidth = net.edges[edgeOf(lane)].width;
                // MSLane::myTmpVehicles: the vehicles already processed on this lane
                for (int cand : ch[candi].done) {
                    if (pos(cand) > pos(vehicle)
                            && ((!outsideLeft && s.leftSideOnEdge(cand) < 0) || (outsideLeft && s.leftSideOnEdge(cand) > edgeWidth))) {
                        const double gap = pos(cand) - pos(vehicle) - T(cand).length - T(vehicle).minGap;
                        leaders.addLeader(cand, gap);
                    }
                }
            }
        }
    }

    // MSLaneChangerSublane::checkChangeSublane
    int checkChangeSublane(int vehicle, int laneOffset, int alternatives, double& latDist, double& maneuverDist) {
        const int target = candi + laneOffset;
        const int neighLane = ch[target].lane;
        int blocked = 0;
        LeaderDistanceInfo neighLeaders = se[target].aheadNext;
        LeaderDistanceInfo neighFollowers = followersOnConsecutive(neighLane, vehicle, pos(vehicle) - T(vehicle).length);
        LeaderDistanceInfo neighBlockers(s, laneW(neighLane), vehicle,
                                         net.lanes[laneOf(vehicle)].rightSideOnEdge - net.lanes[neighLane].rightSideOnEdge);
        LeaderDistanceInfo leaders = se[candi].aheadNext;
        addOutsideLeaders(vehicle, leaders);
        LeaderDistanceInfo followers = followersOnConsecutive(ch[candi].lane, vehicle, pos(vehicle) - T(vehicle).length);
        LeaderDistanceInfo blockers(s, laneW(laneOf(vehicle)), vehicle, 0);
        // sibling lanes of the origin and the target lane
        for (int offset : se[candi].siblings) {
            const LeaderDistanceInfo sibFollowers = followersOnConsecutive(ch[candi + offset].lane, vehicle, pos(vehicle) - T(vehicle).length);
            if (sibFollowers.hasVehicles()) {
                followers.addLeaders(sibFollowers);
            }
            if (se[candi + offset].aheadNext.hasVehicles()) {
                leaders.addLeaders(se[candi + offset].aheadNext);
            }
        }
        for (int offset : se[target].siblings) {
            const LeaderDistanceInfo sibFollowers = followersOnConsecutive(ch[target + offset].lane, vehicle, pos(vehicle) - T(vehicle).length);
            if (sibFollowers.hasVehicles()) {
                neighFollowers.addLeaders(sibFollowers);
            }
            if (se[target + offset].aheadNext.hasVehicles()) {
                neighLeaders.addLeaders(se[target + offset].aheadNext);
            }
        }
        if (vehicle == s.traceVeh) {
            std::fprintf(stderr, "    neighFollowers %s:", net.lanes[neighLane].id.c_str());
            for (int i = 0; i < neighFollowers.numSublanes(); ++i) {
                std::fprintf(stderr, " %s/%.2f", neighFollowers[i] < 0 ? "-" : s.veh[neighFollowers[i]].id.c_str(),
                             neighFollowers[i] < 0 ? 0. : neighFollowers.at(i).second);
            }
            std::fprintf(stderr, "\n");
        }
        // break leader symmetry
        if (laneOffset == -1 && neighLeaders.hasVehicles()) {
            neighLeaders.moveSamePosTo(vehicle, neighFollowers);
        }
        const int wish = wantsChangeSublane(vehicle, laneOffset, alternatives, leaders, followers, blockers, neighLeaders, neighFollowers,
                                            neighBlockers, neighLane, &ch[candi].lastBlocked, &ch[candi].firstBlocked, latDist, maneuverDist, blocked);
        if (checkOpened && (blocked & LCA_BLOCKED) == 0 && (wish & LCA_WANTS_LANECHANGE) != 0 && !net.lanes[laneOf(vehicle)].internal) {
            const std::vector<int>& conts = s.bestContinuation(vehicle);
            if (conts.size() > 1 && conts[1] >= 0) {
                const int link = net.linkIdxTo(laneOf(vehicle), conts[1]);
                if (link >= 0 && net.links[link].isEntryLink()) {
                    const int link2 = laneOffset < 0 ? net.links[link].parallelRight : laneOffset > 0 ? net.links[link].parallelLeft : -1;
                    if (link2 >= 0) {
                        const Approach* api = s.approachOf(link, vehicle);
                        if (api != nullptr) {
                            if (!s.opened(link2, api->arrivalTime, api->arrivalSpeed, api->leaveSpeed, T(vehicle).length, s.impatience(vehicle),
                                          T(vehicle).decel, s.veh[vehicle].waitingTime, vehicle, false, nullptr, api->dist)) {
                                blocked |= LCA_BLOCKED;
                            }
                        }
                    }
                }
            }
        }
        const int state = blocked | wish;
        canceled(laneOffset) |= blocked;
        s.saveLCState(vehicle, laneOffset, state);
        return state;
    }

    // ==========================================================================================
    // MSLCM_SL2015
    // ==========================================================================================
    double getWidthSL(int v) const { return T(v).width + NUMERICAL_EPS; }
    double maxSpeedLatStanding(int v) const { return T(v).lcMaxSpeedLatStanding >= 0 ? T(v).lcMaxSpeedLatStanding : T(v).maxSpeedLat; }
    double maxSpeedLatFactor(int v) const { return T(v).lcMaxSpeedLatFactor >= 0 ? T(v).lcMaxSpeedLatFactor : 1; }
    double getMaxSpeedLat2(int v) const { return std::max(T(v).maxSpeedLat, maxSpeedLatStanding(v)); }
    double safetyFactorSL(int v) const { return 1 / ((1 + 0.5 * SL(v).impatience) * T(v).lcAssertive); }
    double speedGainLookahead(int v) const { return T(v).lcSpeedGainLookaheadSet ? T(v).lcSpeedGainLookahead : 5; }
    double changeProbThresholdRight(int v) const {
        const VType& t = T(v);
        return t.lcSpeedGain <= 0 ? DMAX : (0.2 / t.lcSpeedGainRight) / t.lcSpeedGain;
    }
    double changeProbThresholdLeft(int v) const {
        const VType& t = T(v);
        return t.lcSpeedGain <= 0 ? DMAX : 0.2 / t.lcSpeedGain;
    }
    double speedLossProbThreshold(int v) const { return -0.1 + (1 - T(v).lcSublane); }
    double leftBorder(int v) const { return net.edges[edgeOf(laneOf(v))].width; }
    double vehicleCenter(int v) const { return s.centerOnEdge(v, -1); }
    double neighRight(int v, int neighLane) const {
        if (edgeOf(laneOf(v)) != edgeOf(neighLane)) {
            return net.lanes[laneOf(v)].rightSideOnEdge + laneW(laneOf(v));
        }
        return net.lanes[neighLane].rightSideOnEdge;
    }
    static bool overlap(double right, double left, double right2, double left2) {
        return left2 >= right + NUMERICAL_EPS && left >= right2 + NUMERICAL_EPS;
    }
    bool cfRelated(int v, int u) const {
        const std::vector<int>& r = SL(v).cfRelated;
        return std::find(r.begin(), r.end(), u) != r.end();
    }
    bool amBlockingFollowerPlusNB(int v) const {
        return (s.veh[v].lc.ownState & (LCA_AMBLOCKINGFOLLOWER | LCA_AMBLOCKINGFOLLOWER_DONTBRAKE)) != 0;
    }
    static bool currentDistDisallows(double dist, int laneOffset, double lookForwardDist) {
        return dist / std::abs(laneOffset) < lookForwardDist;
    }
    static bool currentDistAllows(double dist, int laneOffset, double lookForwardDist) {
        return dist / std::abs(laneOffset) > lookForwardDist;
    }
    static int getLCA(int state, double latDist) {
        return (latDist == 0 || (state & LCA_CHANGE_REASONS) == 0) ? LCA_NONE : (latDist < 0 ? LCA_RIGHT : LCA_LEFT);
    }

    // MSLCM_SL2015::updateSafeLatDist
    void updateSafeLatDist(int v, double travelledLatDist) {
        SLState& st = SL(v);
        st.safeLatDistLeft -= travelledLatDist;
        st.safeLatDistRight += travelledLatDist;
        if (std::fabs(st.safeLatDistLeft) < NUMERICAL_EPS) {
            st.safeLatDistLeft = 0.;
        }
        if (std::fabs(st.safeLatDistRight) < NUMERICAL_EPS) {
            st.safeLatDistRight = 0.;
        }
    }

    // MSLCM_SL2015::wantsChangeSublane
    int wantsChangeSublane(int v, int laneOffset, int alternatives, const LeaderDistanceInfo& leaders, const LeaderDistanceInfo& followers,
                           const LeaderDistanceInfo& blockers, const LeaderDistanceInfo& neighLeaders,
                           const LeaderDistanceInfo& neighFollowers, const LeaderDistanceInfo& neighBlockers, int neighLane,
                           int* lastBlocked, int* firstBlocked, double& latDist, double& maneuverDist, int& blocked) {
        int result = _wantsChangeSublane(v, laneOffset, alternatives, leaders, followers, blockers, neighLeaders, neighFollowers,
                                         neighBlockers, neighLane, lastBlocked, firstBlocked, latDist, maneuverDist, blocked);
        result = keepLatGap(v, result, leaders, followers, blockers, neighLeaders, neighFollowers, neighBlockers, neighLane, laneOffset,
                            latDist, maneuverDist, blocked);
        result |= getLCA(result, latDist);
        // the lateral acceleration
        latDist = SPEED2DIST(computeSpeedLat(v, latDist, maneuverDist, (result & LCA_URGENT) != 0));
        return result;
    }

    // MSLCM_SL2015::_wantsChangeSublane
    int _wantsChangeSublane(int v, int laneOffset, int alternatives, const LeaderDistanceInfo& leaders, const LeaderDistanceInfo& followers,
                            const LeaderDistanceInfo& blockers, const LeaderDistanceInfo& neighLeaders,
                            const LeaderDistanceInfo& neighFollowers, const LeaderDistanceInfo& neighBlockers, int neighLane,
                            int* lastBlocked, int* firstBlocked, double& latDist, double& maneuverDist, int& blocked) {
        Veh& x = s.veh[v];
        LCState& lc = x.lc;
        SLState& st = SL(v);
        const VType& t = T(v);
        if (laneOffset != 0) {
            // update mySafeLatDist w.r.t. the direction being checked
            const double halfWidth = getWidthSL(v) * 0.5;
            const double center = vehicleCenter(v);
            if (laneOffset < 0) {
                st.safeLatDistRight = center - halfWidth;
            } else {
                st.safeLatDistLeft = leftBorder(v) - center - halfWidth;
            }
        }
        // bestLaneOffset
        const std::vector<LaneQ>& preb = x.bestLanes.front();
        static const LaneQ noLaneQ;
        const LaneQ* currP = &noLaneQ;
        const LaneQ* neighP = &noLaneQ;
        const LaneQ* bestP = &noLaneQ;
        int bestLaneOffset = 0;
        double currentDist = 0;
        double neighDist = 0;
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
                break;
            }
        }
        const LaneQ& curr = *currP;
        const LaneQ& neigh = *neighP;
        const LaneQ& best = *bestP;
        if (x.stopDist < DMAX && !x.stops.empty() && edgeOf(x.stops.front().lane) == edgeOf(laneOf(v))) {
            // the vehicle can always drive up to the stop
            const double stopPos = pos(v) + x.stopDist - x.lastStepDist;
            currentDist = std::max(currentDist, stopPos);
            neighDist = std::max(neighDist, stopPos);
        }
        const bool right = laneOffset == -1;
        const bool left = laneOffset == 1;
        const int myLca = right ? LCA_MRIGHT : (left ? LCA_MLEFT : 0);
        const int lcaCounter = right ? LCA_LEFT : (left ? LCA_RIGHT : LCA_NONE);
        const bool changeToBest = (right && bestLaneOffset < 0) || (left && bestLaneOffset > 0) || (laneOffset == 0 && bestLaneOffset == 0);
        // keep the information about being a leader / follower but not the previous request or urgency
        int ret = lc.ownState & 0xffff0000;
        // the minimum distance to move the vehicle fully onto the new lane
        const double latLaneDist = laneOffset == 0 ? 0. : s.lateralDistanceToLane(v, laneOffset);
        ret = slowDownForBlocked(v, *lastBlocked, ret);
        if (lastBlocked != firstBlocked) {
            ret = slowDownForBlocked(v, *firstBlocked, ret);
        }
        if (speed(v) > lc.lookAheadSpeed) {
            lc.lookAheadSpeed = speed(v);
        } else {
            lc.lookAheadSpeed = std::max(LOOK_AHEAD_MIN_SPEED, LOOK_AHEAD_SPEED_MEMORY * lc.lookAheadSpeed + (1 - LOOK_AHEAD_SPEED_MEMORY) * speed(v));
        }
        double laDist = lc.lookAheadSpeed * LOOK_FORWARD * t.lcStrategic * (right ? 1 : t.lcLookaheadLeft);
        laDist += t.lengthWithGap() * 2.;
        // react to a stopped leader on the current lane
        if (bestLaneOffset == 0 && leaders.hasStoppedVehicle()) {
            laDist = t.lengthWithGap();
        }
        if (t.lcStrategic < 0) {
            laDist = -1e3;  // never a strategic change
        }
        const double roundaboutBonus = roundaboutDistBonus(v, t.lcCooperativeRoundabout < 0 ? t.lcCooperative : t.lcCooperativeRoundabout, curr, neigh, best);
        currentDist += roundaboutBonus;
        neighDist += roundaboutBonus;
        ret = checkStrategicChange(v, ret, neighLane, laneOffset, leaders, neighLeaders, curr, neigh, best, bestLaneOffset, changeToBest,
                                   currentDist, neighDist, laDist, roundaboutBonus, latLaneDist, latDist);
        if ((ret & LCA_STAY) != 0 && latDist == 0) {
            // ensure that mySafeLatDistLeft / mySafeLatDistRight are up to date for the check with laneOffset = 0
            const double center = s.centerOnEdge(v, -1);
            const double nr = neighRight(v, neighLane);
            updateGaps(v, neighLeaders, nr, center, 1.0, st.safeLatDistRight, st.safeLatDistLeft);
            updateGaps(v, neighFollowers, nr, center, 1.0, st.safeLatDistRight, st.safeLatDistLeft);
            return canceled(laneOffset);
        }
        if ((ret & LCA_URGENT) != 0) {
            // prepare the urgent lane change maneuver
            if (changeToBest && std::abs(bestLaneOffset) > 1 && curr.bestContinuations.back() >= 0
                    && !net.lanes[curr.bestContinuations.back()].links.empty()) {
                // a vehicle might need to counter-lane-change one lane further and we cannot see it yet
                const double reserve = std::min(lc.leftSpace - POSITION_EPS, getExtraReservation(bestLaneOffset, neighDist - currentDist, v));
                lc.leadingBlockerLength = std::max(reserve, lc.leadingBlockerLength);
            }
            // letting vehicles merge in at the end of the lane in case of counter-lane change, step#1
            const int neighLeadLongest = getLongest(v, neighLeaders).first;
            const bool canContinue = curr.bestContinuations.size() > 1;
            bool canReserve = updateBlockerLength(v, neighLeadLongest, lcaCounter, lc.leftSpace - POSITION_EPS, canContinue, lc.leadingBlockerLength);
            if (*firstBlocked != neighLeadLongest && tieBrakeLeader(v, *firstBlocked)) {
                canReserve &= updateBlockerLength(v, *firstBlocked, lcaCounter, lc.leftSpace - POSITION_EPS, canContinue, lc.leadingBlockerLength);
            }
            if (!canReserve) {
                // a low-priority relief connection
                lc.dontBrake = canContinue;
            }
            std::vector<CLeaderDist> collectLeadBlockers;
            std::vector<CLeaderDist> collectFollowBlockers;
            int blockedFully = 0;
            maneuverDist = latDist;
            const double gapFactor = computeGapFactor(v, LCA_STRATEGIC);
            blocked = checkBlocking(v, neighLane, latDist, maneuverDist, laneOffset, leaders, followers, blockers, neighLeaders,
                                    neighFollowers, neighBlockers, &collectLeadBlockers, &collectFollowBlockers, false, gapFactor, &blockedFully);
            const double absLaneOffset = std::fabs(bestLaneOffset != 0 ? bestLaneOffset : latDist / SUMO_LANE_WIDTH);
            // (STEPS2TIME(TS): TS / 1000)
            const double remainingSeconds = std::max(TS / 1000., lc.leftSpace / std::max(lc.lookAheadSpeed, NUMERICAL_EPS) / absLaneOffset / URGENCY);
            const double plannedSpeed = informLeadersSL(v, blocked, myLca, collectLeadBlockers, remainingSeconds);
            if (plannedSpeed >= 0) {
                informFollowersSL(v, blocked, myLca, collectFollowBlockers, remainingSeconds, plannedSpeed);
            }
            if (plannedSpeed > 0) {
                commitManoeuvre(v, blocked, blockedFully, leaders, neighLeaders, neighLane, maneuverDist);
            }
            return canceled(laneOffset);
        }
        if (roundaboutBonus > 0) {
            // use the inner lanes of a roundabout unless approaching the exit
            if (left) {
                ret |= LCA_COOPERATIVE;
                if (!cancelRequest(ret | LCA_LEFT, laneOffset)) {
                    if ((ret & LCA_STAY) == 0) {
                        latDist = latLaneDist;
                        maneuverDist = latLaneDist;
                        blocked = checkBlocking(v, neighLane, latDist, maneuverDist, laneOffset, leaders, followers, blockers, neighLeaders,
                                                neighFollowers, neighBlockers);
                    }
                    return ret;
                }
                ret &= ~LCA_COOPERATIVE;
            } else {
                st.keepRightProb = 0;
            }
        }
        // make place on the current lane if blocking a follower
        const double inconvenience = latLaneDist < 0 ? -st.speedGainProbRight / changeProbThresholdRight(v)
                                                     : -st.speedGainProbLeft / changeProbThresholdLeft(v);
        if (laneOffset != 0
                && ((amBlockingFollowerPlusNB(v) && (lc.ownState & myLca) != 0)
                    // continue a previous cooperative change
                    || ((lc.previousState & LCA_COOPERATIVE) != 0 && !st.canChangeFully && laneOffset * lc.maneuverDist > 0))
                && inconvenience < t.lcCooperative
                && (changeToBest || currentDistAllows(neighDist, std::abs(bestLaneOffset) + 1, laDist))) {
            ret |= LCA_COOPERATIVE | LCA_URGENT;
            if (!cancelRequest(ret | getLCA(ret, latLaneDist), laneOffset)) {
                latDist = amBlockingFollowerPlusNB(v) ? latLaneDist : lc.maneuverDist;
                maneuverDist = latDist;
                blocked = checkBlocking(v, neighLane, latDist, maneuverDist, laneOffset, leaders, followers, blockers, neighLeaders,
                                        neighFollowers, neighBlockers);
                return ret;
            }
            ret &= ~(LCA_COOPERATIVE | LCA_URGENT);
        }
        // higher speed: all combinations of sublanes the vehicle might cover
        const Edge& edgeE = net.edges[edgeOf(laneOf(v))];
        const std::vector<double>& sublaneSides = edgeE.sublaneSides;
        const double vehWidth = getWidthSL(v);
        const double rightVehSide = vehicleCenter(v) - 0.5 * vehWidth;
        const double leftVehSide = rightVehSide + vehWidth;
        // the next speed when staying where we are
        double defaultNextSpeed = DMAX;
        int leftmostOnEdge = (int)sublaneSides.size() - 1;
        while (leftmostOnEdge > 0 && sublaneSides[leftmostOnEdge] > leftVehSide) {
            leftmostOnEdge--;
        }
        int rightmostOnEdge = leftmostOnEdge;
        while (rightmostOnEdge > 0 && sublaneSides[rightmostOnEdge] > rightVehSide + NUMERICAL_EPS) {
            defaultNextSpeed = std::min(defaultNextSpeed, st.expectedSublaneSpeeds[rightmostOnEdge]);
            rightmostOnEdge--;
        }
        defaultNextSpeed = std::min(defaultNextSpeed, st.expectedSublaneSpeeds[rightmostOnEdge]);
        double maxGain = -DMAX;
        double maxGainRight = -DMAX;
        double maxGainLeft = -DMAX;
        double latDistNice = DMAX;
        const Lane& myL = net.lanes[laneOf(v)];
        const Lane& nL = net.lanes[neighLane];
        const int iMin = std::min(myL.rightmostSublane, nL.rightmostSublane);
        const double leftMax = std::max(myL.rightSideOnEdge + myL.width, nL.rightSideOnEdge + nL.width);
        const double rightMin = std::min(myL.rightSideOnEdge, nL.rightSideOnEdge);
        int sublaneCompact = std::max(iMin, rightmostOnEdge - 1);  // compactify to the right by default
        const double laneBoundary = laneOffset < 0 ? myL.rightSideOnEdge : nL.rightSideOnEdge;
        // with a neighboring lane, check the sublanes on all lanes of the edge but restrict the maneuver to the
        // currently visible lanes (current, neigh)
        const double maxLatDist = leftMax - leftVehSide;
        const double minLatDist = rightMin - rightVehSide;
        const int iStart = laneOffset == 0 ? iMin : 0;
        const double rightEnd = laneOffset == 0 ? leftMax : edgeE.width;
        const double asl = s.actionStepLengthSecs(v);
        for (int i = iStart; i < (int)sublaneSides.size(); ++i) {
            if (sublaneSides[i] + vehWidth < rightEnd) {
                // i is the rightmost sublane and the vehicle still fits on the edge: the min speed of all covered sublanes
                double vMin = st.expectedSublaneSpeeds[i];
                int j = i;
                while (vMin > 0 && j < (int)sublaneSides.size() && sublaneSides[j] < sublaneSides[i] + vehWidth) {
                    vMin = std::min(vMin, st.expectedSublaneSpeeds[j]);
                    ++j;
                }
                // between lanes
                if (laneOffset != 0 && overlap(sublaneSides[i], sublaneSides[i] + vehWidth, laneBoundary, laneBoundary)) {
                    vMin *= (1 - t.lcLaneDiscipline);
                }
                const double relativeGain = (vMin - defaultNextSpeed) / std::max(vMin, RELGAIN_NORMALIZATION_MIN_SPEED);
                double currentLatDist = sublaneSides[i] - rightVehSide;
                if ((laneOffset == 0 && (currentLatDist > maxLatDist || currentLatDist < minLatDist))
                        || (laneOffset < 0 && currentLatDist > maxLatDist)
                        || (laneOffset > 0 && currentLatDist < minLatDist)) {
                    continue;
                }
                currentLatDist = std::min(std::max(currentLatDist, minLatDist), maxLatDist);
                // (no bidi lanes)
                if (relativeGain > maxGain && currentLatDist * laneOffset >= 0) {
                    maxGain = relativeGain;
                    if (maxGain > GAIN_PERCEPTION_THRESHOLD) {
                        sublaneCompact = i;
                        latDist = currentLatDist;
                    }
                } else {
                    // if anticipated gains to the left are higher than to the right and current gains are equal, prefer left
                    if (currentLatDist > 0 && st.speedGainProbLeft > st.speedGainProbRight && relativeGain > GAIN_PERCEPTION_THRESHOLD
                            && maxGain - relativeGain < NUMERICAL_EPS) {
                        latDist = currentLatDist;
                    }
                }
                if (currentLatDist < -NUMERICAL_EPS * asl) {
                    maxGainRight = std::max(maxGainRight, relativeGain);
                } else if (currentLatDist > NUMERICAL_EPS * asl) {
                    maxGainLeft = std::max(maxGainLeft, relativeGain);
                }
                const double subAlignDist = sublaneSides[i] - rightVehSide;
                if (std::fabs(subAlignDist) < std::fabs(latDistNice)) {
                    latDistNice = subAlignDist;
                }
            }
        }
        if (v == s.traceVeh) {
            std::fprintf(stderr, "    gains off=%d default=%.3f maxGain=%.3f R=%.3f L=%.3f latDist=%.3f exp:", laneOffset, defaultNextSpeed, maxGain,
                         maxGainRight, maxGainLeft, latDist);
            for (double e : st.expectedSublaneSpeeds) {
                std::fprintf(stderr, " %.3f", e);
            }
            std::fprintf(stderr, "\n");
        }
        // the change probabilities
        if (maxGainRight != -DMAX) {
            st.speedGainProbRight += asl * maxGainRight;
        }
        if (maxGainLeft != -DMAX) {
            st.speedGainProbLeft += asl * maxGainLeft;
        }
        // decay if there is no reason for or against changing (only with enough information)
        if ((std::fabs(maxGainRight) < NUMERICAL_EPS || maxGainRight == -DMAX) && (right || (alternatives & LCA_RIGHT) == 0)) {
            st.speedGainProbRight *= std::pow(SPEEDGAIN_DECAY_FACTOR, asl);
        }
        if ((std::fabs(maxGainLeft) < NUMERICAL_EPS || maxGainLeft == -DMAX) && (left || (alternatives & LCA_LEFT) == 0)) {
            st.speedGainProbLeft *= std::pow(SPEEDGAIN_DECAY_FACTOR, asl);
        }
        if (!left) {
            // only for changing to the right: keepRight when no speed loss is expected (and continue a started
            // maneuver if the loss isn't too big)
            if (right && speed(v) > 0 && (maxGainRight >= 0 || ((lc.previousState & LCA_KEEPRIGHT) != 0 && maxGainRight >= -t.lcKeepRight))) {
                const double vMax = laneVehMaxSpeed(laneOf(v), v);
                const double roadSpeedFactor = vMax / myL.speed;
                double acceptanceTime;
                if (t.lcKeepRightAcceptanceTime == -1) {
                    acceptanceTime = 7 * roadSpeedFactor * std::max(1.0, speed(v));
                } else {
                    acceptanceTime = t.lcKeepRightAcceptanceTime * roadSpeedFactor;
                    if (followers.hasVehicles()) {
                        // reduce acceptanceTime if a follower vehicle is faster or wants to drive faster
                        double minFactor = 1.0;
                        for (int i = 0; i < followers.numSublanes(); ++i) {
                            const CLeaderDist follower = followers.at(i);
                            if (follower.first >= 0 && follower.second < 2 * s.cfBrakeGap(follower.first, speed(follower.first))) {
                                if (speed(follower.first) >= speed(v)) {
                                    double factor = std::max(1.0, speed(v)) / std::max(1.0, speed(follower.first));
                                    const double fRSF = laneVehMaxSpeed(laneOf(follower.first), follower.first) / net.lanes[laneOf(follower.first)].speed;
                                    if (fRSF > roadSpeedFactor) {
                                        factor /= fRSF;
                                    }
                                    if (factor < minFactor) {
                                        minFactor = factor;
                                    }
                                }
                            }
                        }
                        acceptanceTime *= minFactor;
                    }
                }
                double fullSpeedGap = std::max(0., neighDist - s.cfBrakeGap(v, vMax));
                double fullSpeedDrivingSeconds = std::min(acceptanceTime, fullSpeedGap / vMax);
                const CLeaderDist neighLead = getSlowest(neighLeaders);
                if (neighLead.first >= 0 && speed(neighLead.first) < vMax) {
                    fullSpeedGap = std::max(0., std::min(fullSpeedGap, neighLead.second - secureGap(v, neighLead.first, vMax, speed(neighLead.first),
                                                                                                         T(neighLead.first).decel)));
                    fullSpeedDrivingSeconds = std::min(fullSpeedDrivingSeconds, fullSpeedGap / (vMax - speed(neighLead.first)));
                }
                const double deltaProb = (changeProbThresholdRight(v) * (fullSpeedDrivingSeconds / acceptanceTime) / KEEP_RIGHT_TIME) * asl;
                const bool isSlide = preventSliding(v, latLaneDist);
                // stay below the threshold
                if (!isSlide || !wantsKeepRight(v, st.keepRightProb + deltaProb)) {
                    st.keepRightProb += deltaProb;
                }
                if (wantsKeepRight(v, st.keepRightProb)) {
                    ret |= LCA_KEEPRIGHT;
                    if (!cancelRequest(ret | LCA_RIGHT, laneOffset)) {
                        latDist = latLaneDist;
                        maneuverDist = latLaneDist;
                        blocked = checkBlocking(v, neighLane, latDist, maneuverDist, laneOffset, leaders, followers, blockers, neighLeaders,
                                                neighFollowers, neighBlockers);
                        return ret;
                    }
                    ret &= ~LCA_KEEPRIGHT;
                }
            }
            const double bidiRightFactor = 1;  // (no bidi lanes)
            if (latDist < 0 && st.speedGainProbRight >= changeProbThresholdRight(v) * bidiRightFactor
                    && neighDist / std::max(.1, speed(v)) > t.lcSpeedGainRemainTime) {
                ret |= LCA_SPEEDGAIN;
                if (!cancelRequest(ret | getLCA(ret, latDist), laneOffset)) {
                    int blockedFully = 0;
                    maneuverDist = latDist;
                    blocked = checkBlocking(v, neighLane, latDist, maneuverDist, laneOffset, leaders, followers, blockers, neighLeaders,
                                            neighFollowers, neighBlockers, nullptr, nullptr, false, 0, &blockedFully);
                    return ret;
                }
                latDist = 0;
                ret &= ~LCA_SPEEDGAIN;
            }
        }
        if (!right) {
            const bool stayInLane = s.kin[v].posLat + latDist < 0.5 * myL.width;
            if (latDist > 0 && st.speedGainProbLeft > changeProbThresholdLeft(v)
                    // when leaving the lane, the vehicle should be able to stay in the new lane for some time
                    && (stayInLane || neighDist / std::max(.1, speed(v)) > t.lcSpeedGainRemainTime)) {
                ret |= LCA_SPEEDGAIN;
                if (!cancelRequest(ret + getLCA(ret, latDist), laneOffset)) {
                    int blockedFully = 0;
                    maneuverDist = latDist;
                    blocked = checkBlocking(v, neighLane, latDist, maneuverDist, laneOffset, leaders, followers, blockers, neighLeaders,
                                            neighFollowers, neighBlockers, nullptr, nullptr, false, 0, &blockedFully);
                    return ret;
                }
                latDist = 0;
                ret &= ~LCA_SPEEDGAIN;
            }
        }
        double latDistSublane = 0.;
        const double halfLaneWidth = myL.width * 0.5;
        const double halfVehWidth = getWidthSL(v) * 0.5;
        const double posLat = s.kin[v].posLat;
        const VehExtra* xe = x.extra.p.get();
        const int arrivalProc = xe != nullptr ? xe->arrivalPosLatProc : APL_DEFAULT;
        if (arrivalProc != APL_DEFAULT && s.routeEdge(v, 1) < 0 && bestLaneOffset == 0
                && (x.arrivalPos - pos(v)) < ARRIVALPOS_LAT_THRESHOLD) {
            // on the final edge on the correct lane close to the arrival: the desired lateral position
            switch (arrivalProc) {
                case APL_GIVEN:
                    latDistSublane = xe->arrivalPosLat - posLat;
                    break;
                case APL_RIGHT:
                    latDistSublane = -halfLaneWidth + halfVehWidth - posLat;
                    break;
                case APL_CENTER:
                    latDistSublane = -posLat;
                    break;
                case APL_LEFT:
                    latDistSublane = halfLaneWidth - halfVehWidth - posLat;
                    break;
                default:
                    break;
            }
        } else {
            const double getPosLat = posLat + st.sigmaState;
            switch (getDesiredAlignment(v)) {
                case LA_RIGHT:
                    latDistSublane = -halfLaneWidth + halfVehWidth - getPosLat;
                    break;
                case LA_LEFT:
                    latDistSublane = halfLaneWidth - halfVehWidth - getPosLat;
                    break;
                case LA_CENTER:
                    latDistSublane = -getPosLat;
                    break;
                case LA_NICE:
                    latDistSublane = latDistNice;
                    break;
                case LA_COMPACT:
                    latDistSublane = sublaneSides[sublaneCompact] - rightVehSide;
                    break;
                case LA_ARBITRARY: {
                    latDistSublane = posLat - getPosLat;
                    if (std::fabs(posLat) > halfLaneWidth) {
                        // not within the lane
                        if (posLat > 0) {
                            latDistSublane -= (posLat - halfLaneWidth);
                        } else {
                            latDistSublane += (-posLat - halfLaneWidth);
                        }
                    } else {
                        const double edgeWidth = edgeE.width;
                        if (getWidthSL(v) < edgeWidth) {
                            if (rightVehSide < 0) {
                                latDistSublane -= rightVehSide;
                            } else if (leftVehSide > edgeWidth) {
                                latDistSublane -= leftVehSide - edgeWidth;
                            }
                        }
                    }
                    break;
                }
                case LA_GIVEN: {
                    // the alignment should not make the vehicle leave the lane
                    const double hw = myL.width / 2 - NUMERICAL_EPS;
                    const double offset = std::max(-hw, std::min(hw, t.latAlignmentOffset));
                    latDistSublane = -getPosLat + offset;
                    break;
                }
                default:
                    break;
            }
        }
        // the preferred lateral alignment only without speedGain motivation or in the same direction
        if (std::fabs(latDist) <= NUMERICAL_EPS * asl || latDistSublane * latDist > 0) {
            if ((latDistSublane < 0 && st.speedGainProbRight < speedLossProbThreshold(v))
                    || (latDistSublane > 0 && st.speedGainProbLeft < speedLossProbThreshold(v))
                    || computeSpeedGain(v, latDistSublane, defaultNextSpeed) < -t.lcSublane) {
                latDistSublane = 0;  // do not risk losing speed
            }
            // ignore the preferred alignment in the middle of an unfinished maneuver into the opposite direction
            if (!st.canChangeFully && (lc.previousState & (LCA_STRATEGIC | LCA_COOPERATIVE | LCA_KEEPRIGHT | LCA_SPEEDGAIN)) != 0
                    && ((lc.maneuverDist < 0 && latDistSublane > 0) || (lc.maneuverDist > 0 && latDistSublane < 0))) {
                latDistSublane = 0;
            }
            latDist = latDistSublane;
            if (std::fabs(latDist) >= NUMERICAL_EPS * asl) {
                ret |= LCA_SUBLANE;
                // include the prior motivation when the sublane change finishes an ongoing maneuver in the same direction
                if (st.previousManeuverDist * latDist > 0) {
                    const int priorReason = lc.previousState & LCA_CHANGE_REASONS & ~LCA_SUBLANE;
                    ret |= priorReason;
                }
                if (!cancelRequest(ret + getLCA(ret, latDist), laneOffset)) {
                    maneuverDist = latDist;
                    blocked = checkBlocking(v, neighLane, latDist, maneuverDist, laneOffset, leaders, followers, blockers, neighLeaders,
                                            neighFollowers, neighBlockers);
                    return ret;
                }
                ret &= ~LCA_SUBLANE;
            } else {
                return ret | LCA_SUBLANE | LCA_STAY;
            }
        }
        latDist = 0;
        return ret;
    }

    // MSLCM_SL2015::getDesiredAlignment (lcTurnAlignmentDistance)
    int getDesiredAlignment(int v) const {
        const VType& t = T(v);
        int align = t.latAlignment;
        const SLState& st = SL(v);
        if (t.lcTurnAlignmentDistance > 0 && st.nextTurnDist < t.lcTurnAlignmentDistance) {
            const char turnDir = st.nextTurnLink < 0 ? 0 : net.links[st.nextTurnLink].dir;
            const bool indirect = st.nextTurnLink >= 0 && net.links[st.nextTurnLink].indirect;
            switch (turnDir) {
                case 't':
                case 'l':
                case 'L':
                    align = net.lefthand != indirect ? LA_RIGHT : LA_LEFT;
                    break;
                case 'T':
                case 'r':
                case 'R':
                    align = net.lefthand != indirect ? LA_LEFT : LA_RIGHT;
                    break;
                default:
                    break;
            }
        }
        return align;
    }

    // MSLCM_SL2015::checkStrategicChange (no opposite lanes, no TraCI)
    int checkStrategicChange(int v, int ret, int neighLane, int laneOffset, const LeaderDistanceInfo& leaders,
                             const LeaderDistanceInfo& neighLeaders, const LaneQ& curr, const LaneQ& neigh, const LaneQ& best,
                             int bestLaneOffset, bool changeToBest, double& currentDist, double neighDist, double laDist,
                             double roundaboutBonus, double latLaneDist, double& latDist) {
        LCState& lc = s.veh[v].lc;
        SLState& st = SL(v);
        const VType& t = T(v);
        const bool right = laneOffset == -1;
        const bool left = laneOffset == 1;
        const double forwardPos = pos(v);
        if (laneOffset != 0) {
            lc.leftSpace = currentDist - forwardPos;
        }
        const double usableDist = currentDist - forwardPos - best.occupation * JAM_FACTOR;
        const double maxJam = std::max(neigh.occupation, curr.occupation);
        const double neighLeftPlace = std::max(0., neighDist - forwardPos - maxJam);
        const double overlapLat = s.lateralOverlap(v, s.kin[v].posLat, laneOf(v));
        if (laneOffset == 0) {
            const int shadow = K(v).shadowLane;
            if (overlapLat > std::max(POSITION_EPS, s.latRes)
                    && (shadow < 0 || !net.lanes[shadow].allows(s.veh[v].vclass))
                    && getWidthSL(v) < laneW(laneOf(v))) {
                // we urgently need to return to within lane bounds
                latDist = s.kin[v].posLat > 0 ? -overlapLat : overlapLat;
                ret |= LCA_STRATEGIC | LCA_URGENT;
            } else {
                const std::vector<int>& conts = s.bestContinuation(v);
                if (conts.size() > 1 && conts[1] >= 0 && laneW(laneOf(v)) > laneW(conts[1])) {
                    const int cur = laneOf(v);
                    const int next = conts[1];
                    const Link* link = net.linkTo(cur, next);
                    const double distOnLane = net.lanes[cur].length - pos(v);
                    if (link != nullptr && getWidthSL(v) < laneW(next) && distOnLane < 100) {
                        const double hwDiff = 0.5 * (laneW(cur) - laneW(next));
                        const double rightVehSide = s.rightSideOnLane(v, cur) + link->lateralShift - hwDiff;
                        const double leftVehSide = s.leftSideOnLane(v, cur) + link->lateralShift - hwDiff;
                        const double res = s.latRes > 0 ? s.latRes : laneW(next);
                        const int nextRight = s.parallelLane(next, -1);
                        const int nextLeft = s.parallelLane(next, 1);
                        if (rightVehSide < -res && (nextRight < 0 || !net.lanes[nextRight].allows(s.veh[v].vclass))) {
                            latDist = -rightVehSide;
                            lc.leftSpace = distOnLane;
                            ret |= LCA_STRATEGIC | LCA_URGENT;
                        } else if (leftVehSide > laneW(next) + res && (nextLeft < 0 || !net.lanes[nextLeft].allows(s.veh[v].vclass))) {
                            latDist = -(leftVehSide - laneW(next));
                            lc.leftSpace = distOnLane;
                            ret |= LCA_STRATEGIC | LCA_URGENT;
                        }
                    }
                }
            }
        } else if (changeToBest && bestLaneOffset == curr.bestLaneOffset && currentDistDisallows(usableDist, bestLaneOffset, laDist)) {
            // we urgently need to change lanes to follow our route
            if (!mustOvertakeStopped(v, false, neighLane, neighLeaders, leaders, forwardPos, neighDist, right, latLaneDist,
                                     neigh.bestContinuations, currentDist, latDist)) {
                latDist = latLaneDist;
                ret |= LCA_STRATEGIC | LCA_URGENT;
            }
        } else {
            // VARIANT_20 (noOvertakeRight)
            const int stoppedLeader = getStopped(neighLeaders);
            if (left && avoidOvertakeRight(v, true) && neighLeaders.hasVehicles()) {
                (void)stoppedLeader;
                // a slower leader on the left: do not overtake but rather move left (unless congested)
                const CLeaderDist cld = getSlowest(neighLeaders);
                const int nv = cld.first;
                double deltaV = 0.;
                double vSafe = 0.;
                if (canOvertakeRight(v, nv, cld.second, laneVehMaxSpeed(laneOf(v), v) - laneVehMaxSpeed(neighLane, nv), HELP_OVERTAKE, vSafe,
                                     deltaV)) {
                    addLCSpeedAdvice(v, vSafe, LCA_MLEFT);
                    if (vSafe < speed(v)) {
                        st.speedGainProbRight += s.actionStepLengthSecs(v) * changeProbThresholdLeft(v) / 3;
                    }
                }
            }
            if (laneOffset != 0 && t.lcStrategic >= 0
                    && mustOvertakeStopped(v, true, neighLane, leaders, neighLeaders, forwardPos, neighDist, right, latLaneDist,
                                           curr.bestContinuations, currentDist, latDist)) {
                if (latDist == 0) {
                    ret |= LCA_STAY | LCA_STRATEGIC;
                } else {
                    ret |= LCA_STRATEGIC | LCA_URGENT;
                }
            } else if (!changeToBest && currentDistDisallows(neighLeftPlace, std::abs(bestLaneOffset) + 2, laDist)) {
                // the opposite lane-changing direction should be done
                ret |= LCA_STAY | LCA_STRATEGIC;
            } else if (laneOffset != 0 && bestLaneOffset == 0 && !leaders.hasStoppedVehicle()
                       && neigh.bestContinuations.back() >= 0 && !net.lanes[neigh.bestContinuations.back()].links.empty()
                       && roundaboutBonus == 0 && neighDist < TURN_LANE_DIST && t.lcStrategic >= 0) {
                // VARIANT_21 (stayOnBest): do not leave the best lane for a lane which leads elsewhere
                ret |= LCA_STAY | LCA_STRATEGIC;
            } else if (right && bestLaneOffset == 0 && net.lanes[laneOf(v)].speed > 80. / 3.6 && lc.lookAheadSpeed > HALTING_SPEED) {
                // on a highway: do not get to the dead-end of an on-ramp
                ret |= LCA_STAY | LCA_STRATEGIC;
            }
        }
        if (laneOffset != 0 && (ret & LCA_URGENT) == 0 && K(v).shadowLane >= 0
                // ignore the overlap if it goes in the correct direction
                && bestLaneOffset * s.kin[v].posLat <= 0) {
            // stay within lane bounds in case the shadow lane ends
            const double requiredDist = 2 * overlapLat / SUMO_LANE_WIDTH * laDist;
            double currentShadowDist = -pos(v);
            int shadowPrev = -1;
            for (int cont : curr.bestContinuations) {
                if (cont < 0) {
                    continue;
                }
                const int shadow = s.shadowLaneFor(v, cont, s.kin[v].posLat);
                if (shadow < 0 || currentShadowDist >= requiredDist) {
                    break;
                }
                if (shadowPrev >= 0) {
                    currentShadowDist += internalFollowingLengthTo(edgeOf(shadowPrev), edgeOf(shadow), s.veh[v].vclass);
                }
                currentShadowDist += net.lanes[shadow].length;
                shadowPrev = shadow;
            }
            if (currentShadowDist < requiredDist && currentShadowDist < usableDist) {
                lc.leftSpace = currentShadowDist;
                latDist = s.kin[v].posLat < 0 ? overlapLat : -overlapLat;
                ret |= LCA_STRATEGIC | LCA_URGENT | LCA_STAY;
            }
        }
        canceled(laneOffset) |= ret;
        return ret;
    }

    // MSEdge::getInternalFollowingLengthTo
    double internalFollowingLengthTo(int fromEdge, int toEdge, long long vclass) const {
        for (int l : net.edges[fromEdge].lanes) {
            for (int li : net.lanes[l].links) {
                const Link& k = net.links[li];
                if (edgeOf(k.lane) == toEdge && (k.permissions & vclass) == vclass) {
                    double len = 0;
                    int via = k.via;
                    while (via >= 0) {
                        len += net.lanes[via].length;
                        via = net.links[net.lanes[via].links[0]].via;
                    }
                    return len;
                }
            }
        }
        return 0;
    }

    // MSLCM_SL2015::mustOvertakeStopped
    bool mustOvertakeStopped(int v, bool checkCurrent, int neighLane, const LeaderDistanceInfo& leaders, const LeaderDistanceInfo& neighLead,
                             double posOnLane, double neighDist, bool right, double latLaneDist, const std::vector<int>& conts,
                             double& currentDist, double& latDist) {
        bool mustOvertake = false;
        const int stoppedLeader = getStopped(leaders);
        const bool checkOverTakeRight = avoidOvertakeRight(v, true);
        int rightmost;
        int leftmost;
        const bool curHasStopped = stoppedLeader >= 0;
        (void)neighLane;
        (void)conts;
        if (curHasStopped) {
            leaders.getSubLanes(v, 0, rightmost, leftmost);
            for (int i = rightmost; i <= leftmost; i++) {
                const CLeaderDist leader = leaders.at(i);
                if (leader.first >= 0 && isStopped(leader.first) && leader.second < REACT_TO_STOPPED_DISTANCE) {
                    const double overtakeDist = leader.second + T(v).length + T(leader.first).lengthWithGap();
                    const double remaining = std::min(neighDist, currentDist) - posOnLane;
                    if (remaining > overtakeDist
                            // maybe do not overtake on the right at high speed
                            && (!checkCurrent || !checkOverTakeRight || !right)
                            && (!neighLead.hasStoppedVehicle() || neighLead.getMinDistToStopped() > overtakeDist)) {
                        // avoid becoming stuck behind a stopped leader
                        currentDist = pos(v) + leader.second;
                        s.veh[v].lc.leftSpace = currentDist - posOnLane;
                        latDist = latLaneDist;
                        mustOvertake = true;
                    }
                }
            }
        }
        if (!mustOvertake && !curHasStopped && neighLead.hasStoppedVehicle()) {
            const double offset = (latLaneDist < 0 ? -1 : 1) * laneW(laneOf(v));
            neighLead.getSubLanes(v, offset, rightmost, leftmost);
            for (int i = 0; i < leaders.numSublanes(); i++) {
                const CLeaderDist leader = leaders.at(i);
                if (leader.first >= 0 && isStopped(leader.first) && leader.second < REACT_TO_STOPPED_DISTANCE) {
                    mustOvertake = true;
                    if (i >= rightmost && i <= leftmost) {
                        latDist = s.lateralOverlap(v, s.kin[v].posLat, laneOf(v)) * (latLaneDist > 0 ? -1 : 1);
                        break;
                    }
                }
            }
        }
        return mustOvertake;
    }

    // MSLCM_SL2015::getStopped / getSlowest / getLongest / tieBrakeLeader
    int getStopped(const LeaderDistanceInfo& ldi) const {
        if (ldi.hasVehicles()) {
            for (int u : ldi.vehs) {
                if (u >= 0 && isStopped(u)) {
                    return u;
                }
            }
        }
        return -1;
    }
    CLeaderDist getSlowest(const LeaderDistanceInfo& ldi) const {
        int iMax = 0;
        double minSpeed = DMAX;
        for (int i = 0; i < ldi.numSublanes(); ++i) {
            if (ldi[i] >= 0) {
                const double sp = speed(ldi[i]);
                if (sp < minSpeed) {
                    minSpeed = sp;
                    iMax = i;
                }
            }
        }
        return ldi.at(iMax);
    }
    bool tieBrakeLeader(int v, int u) const {
        // tie braker if the leader is at the same lane position
        return u >= 0 && (pos(u) != pos(v) || speed(u) < speed(v) || edgeOf(laneOf(u)) != edgeOf(laneOf(v))
                          || net.lanes[laneOf(u)].index > net.lanes[laneOf(v)].index);
    }
    CLeaderDist getLongest(int v, const LeaderDistanceInfo& ldi) const {
        int iMax = -1;
        double maxLength = -1;
        for (int i = 0; i < ldi.numSublanes(); ++i) {
            const int u = ldi[i];
            if (u >= 0) {
                const double length = T(u).length;
                if (length > maxLength && tieBrakeLeader(v, u)) {
                    maxLength = length;
                    iMax = i;
                }
            }
        }
        return iMax >= 0 ? ldi.at(iMax) : CLeaderDist(-1, -1);
    }

    double computeGapFactor(int v, int state) const {
        return (state & LCA_STRATEGIC) != 0 ? std::max(0.0, (1.0 - T(v).lcPushy * (1 + 0.5 * SL(v).impatience))) : 1.0;
    }

    // MSLCM_SL2015::checkBlocking
    int checkBlocking(int v, int neighLane, double& latDist, double maneuverDist, int laneOffset, const LeaderDistanceInfo& leaders,
                      const LeaderDistanceInfo& followers, const LeaderDistanceInfo& /* blockers */, const LeaderDistanceInfo& neighLeaders,
                      const LeaderDistanceInfo& neighFollowers, const LeaderDistanceInfo& /* neighBlockers */,
                      std::vector<CLeaderDist>* collectLeadBlockers = nullptr, std::vector<CLeaderDist>* collectFollowBlockers = nullptr,
                      bool keepLatGapManeuver = false, double gapFactor = 0, int* retBlockedFully = nullptr) {
        SLState& st = SL(v);
        // truncate latDist according to maxSpeedLat
        const double maxDist = SPEED2DIST(getMaxSpeedLat2(v));
        latDist = std::max(std::min(latDist, maxDist), -maxDist);
        const double nr = neighRight(v, neighLane);
        const double laneRight = net.lanes[laneOf(v)].rightSideOnEdge;
        if (!st.cfRelatedReady) {
            updateCFRelated(v, followers, laneRight, false);
            updateCFRelated(v, leaders, laneRight, true);
            if (laneOffset != 0) {
                updateCFRelated(v, neighFollowers, nr, false);
                updateCFRelated(v, neighLeaders, nr, true);
            }
            st.cfRelatedReady = true;
        }
        // reduce latDist to avoid blockage with overlapping vehicles (no minGapLat constraints)
        const double center = s.centerOnEdge(v, -1);
        updateGaps(v, leaders, laneRight, center, gapFactor, st.safeLatDistRight, st.safeLatDistLeft, false, 0, latDist, collectLeadBlockers);
        updateGaps(v, followers, laneRight, center, gapFactor, st.safeLatDistRight, st.safeLatDistLeft, false, 0, latDist, collectFollowBlockers);
        if (laneOffset != 0) {
            updateGaps(v, neighLeaders, nr, center, gapFactor, st.safeLatDistRight, st.safeLatDistLeft, false, 0, latDist, collectLeadBlockers);
            updateGaps(v, neighFollowers, nr, center, gapFactor, st.safeLatDistRight, st.safeLatDistLeft, false, 0, latDist, collectFollowBlockers);
        }
        // if we can move at least a little bit in the desired direction, do so (rather than block)
        if (latDist < 0) {
            if (st.safeLatDistRight <= NUMERICAL_EPS) {
                return LCA_BLOCKED_RIGHT | LCA_OVERLAPPING;
            }
            latDist = std::max(latDist, -st.safeLatDistRight);
        } else {
            if (st.safeLatDistLeft <= NUMERICAL_EPS) {
                return LCA_BLOCKED_LEFT | LCA_OVERLAPPING;
            }
            latDist = std::min(latDist, st.safeLatDistLeft);
        }
        st.canChangeFully = (maneuverDist == 0 || latDist == maneuverDist);
        // destination sublanes must be safe, intermediate sublanes must not be blocked by overlapping vehicles
        int blocked = 0;
        blocked |= checkBlockingVehicles(v, leaders, laneOffset, latDist, laneRight, true, st.safeLatDistRight, st.safeLatDistLeft, collectLeadBlockers);
        blocked |= checkBlockingVehicles(v, followers, laneOffset, latDist, laneRight, false, st.safeLatDistRight, st.safeLatDistLeft, collectFollowBlockers);
        if (laneOffset != 0) {
            blocked |= checkBlockingVehicles(v, neighLeaders, laneOffset, latDist, nr, true, st.safeLatDistRight, st.safeLatDistLeft, collectLeadBlockers);
            blocked |= checkBlockingVehicles(v, neighFollowers, laneOffset, latDist, nr, false, st.safeLatDistRight, st.safeLatDistLeft, collectFollowBlockers);
        }
        int blockedFully = 0;
        blockedFully |= checkBlockingVehicles(v, leaders, laneOffset, maneuverDist, laneRight, true, st.safeLatDistRight, st.safeLatDistLeft, collectLeadBlockers);
        blockedFully |= checkBlockingVehicles(v, followers, laneOffset, maneuverDist, laneRight, false, st.safeLatDistRight, st.safeLatDistLeft, collectFollowBlockers);
        if (laneOffset != 0) {
            blockedFully |= checkBlockingVehicles(v, neighLeaders, laneOffset, maneuverDist, nr, true, st.safeLatDistRight, st.safeLatDistLeft, collectLeadBlockers);
            blockedFully |= checkBlockingVehicles(v, neighFollowers, laneOffset, maneuverDist, nr, false, st.safeLatDistRight, st.safeLatDistLeft, collectFollowBlockers);
        }
        if (retBlockedFully != nullptr) {
            *retBlockedFully = blockedFully;
        }
        if (blocked == 0 && !st.canChangeFully && T(v).lcPushy == 0 && !keepLatGapManeuver) {
            // cautious drivers check that the maneuver can be finished without encroaching on other vehicles
            blocked |= blockedFully;
        }
        if (collectFollowBlockers != nullptr && collectLeadBlockers != nullptr) {
            // no vehicle is both a leader and a follower
            for (const CLeaderDist& lb : *collectLeadBlockers) {
                for (auto it = collectFollowBlockers->begin(); it != collectFollowBlockers->end();) {
                    if (lb.first == it->first) {
                        it = collectFollowBlockers->erase(it);
                    } else {
                        ++it;
                    }
                }
            }
        }
        return blocked;
    }

    // MSLCM_SL2015::checkBlockingVehicles
    int checkBlockingVehicles(int ego, const LeaderDistanceInfo& vehicles, int laneOffset, double latDist, double foeOffset, bool leaders,
                              double& safeLatGapRight, double& safeLatGapLeft, std::vector<CLeaderDist>* collectBlockers) {
        const int blockType = laneOffset == 0 ? (leaders ? LCA_BLOCKED_BY_LEADER : LCA_BLOCKED_BY_FOLLOWER)
                              : (laneOffset > 0 ? (leaders ? LCA_BLOCKED_BY_LEFT_LEADER : LCA_BLOCKED_BY_LEFT_FOLLOWER)
                                 : (leaders ? LCA_BLOCKED_BY_RIGHT_LEADER : LCA_BLOCKED_BY_RIGHT_FOLLOWER));
        const double vehWidth = getWidthSL(ego);
        const double rightVehSide = s.rightSideOnEdge(ego, -1);
        const double leftVehSide = rightVehSide + vehWidth;
        const double rightVehSideDest = rightVehSide + latDist;
        const double leftVehSideDest = leftVehSide + latDist;
        const double rightNoOverlap = std::min(rightVehSideDest, rightVehSide);
        const double leftNoOverlap = std::max(leftVehSideDest, leftVehSide);
        int result = 0;
        for (int i = 0; i < vehicles.numSublanes(); ++i) {
            const CLeaderDist vehDist = vehicles.at(i);
            if (vehDist.first >= 0 && !cfRelated(ego, vehDist.first)) {
                int leader = vehDist.first;
                int follower = ego;
                if (!leaders) {
                    std::swap(leader, follower);
                }
                // only the current stripe occupied by the foe (in edge coordinates)
                double foeRight, foeLeft;
                vehicles.getSublaneBorders(i, foeOffset, foeRight, foeLeft);
                const bool overlapBefore = overlap(rightVehSide, leftVehSide, foeRight, foeLeft);
                const bool overlapDest = overlap(rightVehSideDest, leftVehSideDest, foeRight, foeLeft);
                const bool overlapAny = overlap(rightNoOverlap, leftNoOverlap, foeRight, foeLeft);
                if (ego == s.traceVeh) {
                    std::fprintf(stderr, "      checkBlockingVehicles %s i=%d foe=%s gap=%.3f foeR=%.3f foeL=%.3f vehR=%.3f vehL=%.3f latDist=%.3f oB=%d oD=%d oA=%d\n",
                                 leaders ? "lead" : "follow", i, s.veh[vehDist.first].id.c_str(), vehDist.second, foeRight, foeLeft,
                                 rightVehSide, leftVehSide, latDist, overlapBefore, overlapDest, overlapAny);
                }
                if (overlapAny) {
                    if (vehDist.second < 0) {
                        if (overlapBefore && !overlapDest && !s.outsideEdge(ego)) {
                            // moving away from an overlapping vehicle
                        } else {
                            result |= (blockType | LCA_OVERLAPPING);
                            if (collectBlockers == nullptr) {
                                return result;
                            }
                            collectBlockers->push_back(vehDist);
                        }
                    } else if (overlapDest || !SL(ego).canChangeFully) {
                        // the state after the action step (the follower may be accelerating)
                        const double timeTillAction = std::max(s.actionStepLengthSecs(follower), s.actionStepLengthSecs(leader)) - TS;
                        const double followerAccel = std::max(0., accel(follower));
                        const double leaderAccel = accel(leader);
                        const double expectedGap = cf::gapExtrapolation(timeTillAction, vehDist.second, speed(leader), speed(follower),
                                                                        leaderAccel, followerAccel, DMAX, DMAX);
                        const double followerExpectedSpeed = speed(follower) + timeTillAction * followerAccel;
                        const double leaderExpectedSpeed = std::max(0., speed(leader) + timeTillAction * leaderAccel);
                        const double expectedSecureGap = secureGap(follower, leader, followerExpectedSpeed, leaderExpectedSpeed, T(leader).decel);
                        const double secureGap2 = expectedSecureGap * safetyFactorSL(ego);
                        if (expectedGap < secureGap2) {
                            // a blocker: update the lateral safe gaps
                            if (foeRight > leftVehSide) {
                                safeLatGapLeft = std::min(safeLatGapLeft, foeRight - leftVehSide);
                            } else if (foeLeft < rightVehSide) {
                                safeLatGapRight = std::min(safeLatGapRight, rightVehSide - foeLeft);
                            }
                            result |= blockType;
                            if (collectBlockers == nullptr) {
                                return result;
                            }
                        }
                        if (collectBlockers != nullptr) {
                            // non-blocking followers as well (they should remain non-blocking)
                            collectBlockers->push_back(vehDist);
                        }
                    }
                }
            }
        }
        return result;
    }

    // MSLCM_SL2015::updateCFRelated
    void updateCFRelated(int v, const LeaderDistanceInfo& vehicles, double foeOffset, bool leaders) {
        // a slightly reduced width (numerical instability)
        const double vehWidth = T(v).width - NUMERICAL_EPS;
        const double rightVehSide = s.centerOnEdge(v, -1) - 0.5 * vehWidth;
        const double leftVehSide = rightVehSide + vehWidth;
        std::vector<int>& related = SL(v).cfRelated;
        for (int i = 0; i < vehicles.numSublanes(); ++i) {
            const CLeaderDist vehDist = vehicles.at(i);
            if (vehDist.first >= 0 && (!cfRelated(v, vehDist.first) || vehDist.second < 0)) {
                double foeRight, foeLeft;
                vehicles.getSublaneBorders(i, foeOffset, foeRight, foeLeft);
                const int u = vehDist.first;
                if (overlap(rightVehSide, leftVehSide, foeRight, foeLeft) && !s.outsideEdge(v)
                        // imprecise driving of some models
                        && (vehDist.second >= T(u).minGap * (1 - T(u).collisionMinGapFactor)
                            // avoid a deadlock (#3729)
                            || (!leaders && pos(v) >= T(v).length && speed(v) < HALTING_SPEED && speed(u) < HALTING_SPEED
                                && -vehDist.second < T(u).minGap && edgeOf(laneOf(v)) != edgeOf(laneOf(u))))) {
                    if (!cfRelated(v, u)) {
                        related.push_back(u);
                    }
                } else {
                    related.erase(std::remove(related.begin(), related.end(), u), related.end());
                }
            }
        }
    }

    // MSLCM_SL2015::updateGaps
    void updateGaps(int v, const LeaderDistanceInfo& others, double foeOffset, double oldCenter, double gapFactor, double& surplusGapRight,
                    double& surplusGapLeft, bool saveMinGap = false, double netOverlap = 0, double latDist = 0,
                    std::vector<CLeaderDist>* collectBlockers = nullptr) {
        if (!others.hasVehicles()) {
            return;
        }
        SLState& st = SL(v);
        const double halfWidth = getWidthSL(v) * 0.5 + NUMERICAL_EPS;
        const double baseMinGap = T(v).minGapLat;
        for (int i = 0; i < others.numSublanes(); ++i) {
            const CLeaderDist od = others.at(i);
            if (od.first >= 0 && od.second <= 0 && !cfRelated(v, od.first)
                    && (netOverlap == 0 || od.second + T(od.first).minGap < netOverlap)) {
                // the foe occupies full sublanes
                const int foe = od.first;
                const double res = s.latRes > 0 ? s.latRes : laneW(laneOf(foe));
                double foeRight, foeLeft;
                others.getSublaneBorders(i, foeOffset, foeRight, foeLeft);
                const double foeCenter = foeRight + 0.5 * res;
                const double gap = std::min(std::fabs(foeRight - oldCenter), std::fabs(foeLeft - oldCenter)) - halfWidth;
                const double deltaV = std::min(LATGAP_SPEED_THRESHOLD, std::max(std::max(LATGAP_SPEED_THRESHOLD2, speed(v)), std::fabs(speed(v) - speed(foe))));
                const double desiredMinGap = baseMinGap * deltaV / LATGAP_SPEED_THRESHOLD;
                const double currentMinGap = desiredMinGap * gapFactor;  // pushy vehicles may accept a lower gap temporarily
                // reserve additional distance if the foe is maneuvering towards ego
                if (foeCenter < oldCenter) {
                    const double foeManeuverDist = std::max(0., s.maneuverDistOf(foe));
                    surplusGapRight = std::min(std::min(surplusGapRight, gap - currentMinGap), std::max(currentMinGap, gap - foeManeuverDist));
                } else {
                    const double foeManeuverDist = -std::min(0., s.maneuverDistOf(foe));
                    surplusGapLeft = std::min(std::min(surplusGapLeft, gap - currentMinGap), std::max(currentMinGap, gap - foeManeuverDist));
                }
                if (saveMinGap) {
                    if (foeCenter < oldCenter) {
                        st.lastLateralGapRight = std::min(st.lastLateralGapRight, gap);
                    } else {
                        st.lastLateralGapLeft = std::min(st.lastLateralGapLeft, gap);
                    }
                }
                if (v == s.traceVeh) {
                    std::fprintf(stderr, "      updateGaps foe=%s gap2=%.3f foeRight=%.3f foeLeft=%.3f gap=%.3f minGap=%.3f sR=%.3f sL=%.3f\n",
                                 s.veh[foe].id.c_str(), od.second, foeRight, foeLeft, gap, currentMinGap, surplusGapRight, surplusGapLeft);
                }
                if (collectBlockers != nullptr) {
                    // a vehicle blocking a desired lane change
                    if ((foeCenter < oldCenter && latDist < 0 && gap < (desiredMinGap - latDist))
                            || (foeCenter > oldCenter && latDist > 0 && gap < (desiredMinGap + latDist))) {
                        collectBlockers->push_back(od);
                    }
                }
            }
        }
    }

    // MSLCM_SL2015::keepLatGap
    int keepLatGap(int v, int state, const LeaderDistanceInfo& leaders, const LeaderDistanceInfo& followers, const LeaderDistanceInfo& blockers,
                   const LeaderDistanceInfo& neighLeaders, const LeaderDistanceInfo& neighFollowers, const LeaderDistanceInfo& neighBlockers,
                   int neighLane, int laneOffset, double& latDist, double& maneuverDist, int& blocked) {
        SLState& st = SL(v);
        double gapFactor = computeGapFactor(v, state);
        const double oldLatDist = latDist;
        const double oldManeuverDist = maneuverDist;
        const int traciState = state;
        // the gaps after the maneuver
        const double halfWidth = getWidthSL(v) * 0.5;
        // if the current maneuver is blocked the vehicle stays where it is
        const double oldCenter = s.centerOnEdge(v, -1);
        // surplus gaps collect the constraints; stay within the current edge
        double surplusGapRight = oldCenter - halfWidth;
        double surplusGapLeft = leftBorder(v) - oldCenter - halfWidth;
        const bool stayInLane = laneOffset == 0
                                || ((traciState & LCA_STRATEGIC) != 0 && (traciState & LCA_STAY) != 0
                                    // wide vehicles may stay on the road
                                    && (surplusGapLeft >= 0 && surplusGapRight >= 0));
        // staying within the edge overrides all minGap considerations
        if (surplusGapLeft < 0 || surplusGapRight < 0) {
            gapFactor = 0;
        }
        // gaps to the vehicles on the current lane (ignore those too far behind)
        const double netOverlap = -T(v).length * 0.5;
        const double laneRight = net.lanes[laneOf(v)].rightSideOnEdge;
        updateGaps(v, leaders, laneRight, oldCenter, gapFactor, surplusGapRight, surplusGapLeft, true);
        updateGaps(v, followers, laneRight, oldCenter, gapFactor, surplusGapRight, surplusGapLeft, true, netOverlap);
        if (laneOffset != 0) {
            // gaps to the vehicles on the target lane
            const double nr = neighRight(v, neighLane);
            updateGaps(v, neighLeaders, nr, oldCenter, gapFactor, surplusGapRight, surplusGapLeft, true);
            updateGaps(v, neighFollowers, nr, oldCenter, gapFactor, surplusGapRight, surplusGapLeft, true, netOverlap);
        }
        // the physical gap in addition to the psychological gap
        double physicalGapLeft = st.lastLateralGapLeft == NO_NEIGHBOR ? surplusGapLeft : st.lastLateralGapLeft;
        double physicalGapRight = st.lastLateralGapRight == NO_NEIGHBOR ? surplusGapRight : st.lastLateralGapRight;
        const double halfLaneWidth = laneW(laneOf(v)) * 0.5;
        const double posLat = s.kin[v].posLat;
        if (stayInLane || laneOffset == 1) {
            // do not move past the right boundary of the current lane (but it is fine to be beyond already)
            surplusGapRight = std::min(surplusGapRight, std::max(0.0, halfLaneWidth + posLat - halfWidth));
            physicalGapRight = std::min(physicalGapRight, std::max(0.0, halfLaneWidth + posLat - halfWidth));
        }
        if (stayInLane || laneOffset == -1) {
            surplusGapLeft = std::min(surplusGapLeft, std::max(0.0, halfLaneWidth - posLat - halfWidth));
            physicalGapLeft = std::min(physicalGapLeft, std::max(0.0, halfLaneWidth - posLat - halfWidth));
        }
        if (v == s.traceVeh) {
            std::fprintf(stderr, "    keepLatGap off=%d state=%x latDist=%.3f man=%.3f sR=%.3f sL=%.3f pR=%.3f pL=%.3f gapFactor=%.3f\n", laneOffset,
                         state, latDist, maneuverDist, surplusGapRight, surplusGapLeft, physicalGapRight, physicalGapLeft, gapFactor);
        }
        if (surplusGapRight + surplusGapLeft < 0) {
            // insufficient lateral space: apportion the space proportionally
            if ((state & LCA_CHANGE_REASONS) == 0) {
                state |= LCA_SUBLANE;
            }
            const double equalDeficit = 0.5 * (surplusGapLeft + surplusGapRight);
            if (surplusGapRight < surplusGapLeft) {
                // shift further to the left but no further than there is physical space
                const double delta = std::min(equalDeficit - surplusGapRight, physicalGapLeft);
                latDist = delta;
                maneuverDist = delta;
            } else {
                const double delta = std::min(equalDeficit - surplusGapLeft, physicalGapRight);
                latDist = -delta;
                maneuverDist = -delta;
            }
        } else {
            // sufficient space: move as far as the gaps permit
            latDist = std::max(std::min(latDist, surplusGapLeft), -surplusGapRight);
            maneuverDist = std::max(std::min(maneuverDist, surplusGapLeft), -surplusGapRight);
            if ((state & LCA_KEEPRIGHT) != 0 && maneuverDist != oldManeuverDist) {
                // don't start keepRight unless it can be completed
                latDist = oldLatDist;
                maneuverDist = oldManeuverDist;
            }
        }
        const double asl = s.actionStepLengthSecs(v);
        // if we cannot move in the desired direction, consider the maneuver blocked anyway
        const bool nonSublaneChange = (state & (LCA_STRATEGIC | LCA_COOPERATIVE | LCA_SPEEDGAIN | LCA_KEEPRIGHT)) != 0;
        if (nonSublaneChange) {
            if ((latDist < NUMERICAL_EPS * asl) && (oldLatDist > 0)) {
                latDist = oldLatDist;  // restore the old request for decideDirection()
                blocked = LCA_OVERLAPPING | LCA_BLOCKED_LEFT;
            } else if ((latDist > -NUMERICAL_EPS * asl) && (oldLatDist < 0)) {
                latDist = oldLatDist;
                blocked = LCA_OVERLAPPING | LCA_BLOCKED_RIGHT;
            }
        }
        // moving even though the vehicle wishes to stay: update the change reason
        if (std::fabs(latDist) > NUMERICAL_EPS * asl && oldLatDist == 0) {
            state &= ~(LCA_CHANGE_REASONS | LCA_STAY) | LCA_TRACI;
        }
        // update the blocked status
        if (std::fabs(latDist - oldLatDist) > NUMERICAL_EPS * asl) {
            blocked = checkBlocking(v, neighLane, latDist, maneuverDist, laneOffset, leaders, followers, blockers, neighLeaders, neighFollowers,
                                    neighBlockers, nullptr, nullptr, nonSublaneChange);
        }
        if (std::fabs(latDist) > NUMERICAL_EPS * asl) {
            state = state & ~LCA_STAY;
            if ((state & LCA_CHANGE_REASONS) == 0) {
                state |= LCA_SUBLANE;
            }
        } else {
            if ((state & LCA_SUBLANE) != 0) {
                state |= LCA_STAY;
            }
            // (no blinker due to numerical issues)
            latDist = 0;
        }
        return state;
    }

    // MSLCM_SL2015::computeSpeedLat
    double computeSpeedLat(int v, double latDist, double& maneuverDist, bool urgent) {
        const LCState& lc = s.veh[v].lc;
        const SLState& st = SL(v);
        const VType& t = T(v);
        const double mySpeedLat = lc.speedLat;
        const int currentDirection = mySpeedLat >= 0 ? 1 : -1;
        const int directionWish = latDist >= 0 ? 1 : -1;
        double maxSpeedLat = t.maxSpeedLat;
        double accelLat = t.lcAccelLat;
        if (!urgent && (lc.leftSpace > POSITION_EPS || maxSpeedLatFactor(v) < 0)) {
            const double speedBound = maxSpeedLatStanding(v) + maxSpeedLatFactor(v) * speed(v);
            if (maxSpeedLatFactor(v) >= 0) {
                // the speed bound increases with speed and needs an upper bound
                maxSpeedLat = std::min(maxSpeedLat, speedBound);
            } else {
                // the speed bound decreases with speed and needs a lower bound
                maxSpeedLat = std::max(maxSpeedLat, speedBound);
                accelLat *= std::max(1.0, speedBound / t.maxSpeedLat);
            }
        }
        const double edgeWidth = net.edges[edgeOf(laneOf(v))].width;
        if (getWidthSL(v) < edgeWidth) {
            const double rightVehSide = s.rightSideOnEdge(v, -1);
            const double edgeOverlap = std::max(-rightVehSide, rightVehSide + t.width - edgeWidth);
            // outside the edge bounds: stronger lateral maneuvering
            accelLat = std::max(accelLat, 2 * edgeOverlap);
            maxSpeedLat = std::max(maxSpeedLat, edgeOverlap);
        }
        // reduced lateral speed (in the desired direction, no reversal against the wish)
        double speedDecel;
        if (directionWish == 1) {
            speedDecel = std::max(mySpeedLat - ACCEL2SPEED(accelLat), 0.);
        } else {
            speedDecel = std::min(mySpeedLat + ACCEL2SPEED(accelLat), 0.);
        }
        // increased lateral speed (in the desired direction)
        const double speedAccel = std::max(std::min(mySpeedLat + directionWish * ACCEL2SPEED(accelLat), maxSpeedLat), -maxSpeedLat);
        // can we reach the target distance in a single step?
        const double speedBound = DIST2SPEED(latDist);
        // for lat-gap keeping maneuvers myOrigLatDist may be 0
        const double fullLatDist = latDist > 0 ? std::min(st.safeLatDistLeft, std::max(maneuverDist, latDist))
                                   : std::max(-st.safeLatDistRight, std::min(maneuverDist, latDist));
        // update maneuverDist if safety constraints apply in its direction
        if (maneuverDist * latDist > 0) {
            maneuverDist = fullLatDist;
        }
        if (speedDecel * speedAccel <= 0 && ((latDist >= 0 && speedAccel >= speedBound && speedBound >= speedDecel)
                                              || (latDist <= 0 && speedAccel <= speedBound && speedBound <= speedDecel))) {
            // the desired value can be reached in this step
            return speedBound;
        }
        // moving in the wrong direction?
        if (latDist * mySpeedLat < 0) {
            return emergencySpeedLat(v, speedAccel);
        }
        // does the remaining distance allow to accelerate laterally?
        const double minDistAccel = SPEED2DIST(speedAccel) + currentDirection * brakeGapEuler(std::fabs(speedAccel), accelLat, 0);
        if ((std::fabs(minDistAccel) < std::fabs(fullLatDist)) || (std::fabs(minDistAccel - fullLatDist) < NUMERICAL_EPS)) {
            return speedAccel;
        }
        // does the remaining distance allow to keep the current lateral speed?
        const double minDistCurrent = SPEED2DIST(mySpeedLat) + currentDirection * brakeGapEuler(std::fabs(mySpeedLat), accelLat, 0);
        if ((std::fabs(minDistCurrent) < std::fabs(fullLatDist)) || (std::fabs(minDistCurrent - fullLatDist) < NUMERICAL_EPS)) {
            return mySpeedLat;
        }
        // reduce the lateral speed
        return emergencySpeedLat(v, speedDecel);
    }

    // MSCFModel::brakeGapEuler
    static double brakeGapEuler(double sp, double decel, double headwayTime) {
        const double speedReduction = ACCEL2SPEED(decel);
        const int steps = int(sp / speedReduction);
        return SPEED2DIST(steps * sp - speedReduction * steps * (steps + 1) / 2) + sp * headwayTime;
    }

    double emergencySpeedLat(int v, double speedLat) const {
        const SLState& st = SL(v);
        if (speedLat < 0 && SPEED2DIST(-speedLat) > st.safeLatDistRight) {
            speedLat = -DIST2SPEED(st.safeLatDistRight);
        } else if (speedLat > 0 && SPEED2DIST(speedLat) > st.safeLatDistLeft) {
            speedLat = DIST2SPEED(st.safeLatDistLeft);
        }
        return speedLat;
    }

    // MSLCM_SL2015::decideDirection
    StateAndDist decideDirection(int v, StateAndDist sd1, StateAndDist sd2) const {
        // ignore dummy decisions (mayChange() failed)
        if (sd1.state == 0) {
            return sd2;
        } else if (sd2.state == 0) {
            return sd1;
        }
        // LCA_STAY|LCA_SUBLANE may override another LCA_SUBLANE command
        const bool want1 = ((sd1.state & LCA_WANTS_LANECHANGE) != 0) || ((sd1.state & LCA_SUBLANE) != 0 && (sd1.state & LCA_STAY) != 0);
        const bool want2 = ((sd2.state & LCA_WANTS_LANECHANGE) != 0) || ((sd2.state & LCA_SUBLANE) != 0 && (sd2.state & LCA_STAY) != 0);
        const bool can1 = (sd1.state & LCA_BLOCKED) == 0;
        const bool can2 = (sd2.state & LCA_BLOCKED) == 0;
        const int reason1 = lowestBit(sd1.state & LCA_CHANGE_REASONS);
        const int reason2 = lowestBit(sd2.state & LCA_CHANGE_REASONS);
        if (want1) {
            if (want2) {
                // the lower value of LaneChangeAction has the higher priority
                if (reason1 < reason2) {
                    return (!can1 && can2 && sd1.sameDirection(sd2)) ? sd2 : sd1;
                } else if (reason1 > reason2) {
                    return (!can2 && can1 && sd1.sameDirection(sd2)) ? sd1 : sd2;
                } else {
                    // same priority
                    if ((sd1.state & LCA_SUBLANE) != 0) {
                        // prefer an action with dir != 0
                        if (sd1.dir == 0) {
                            return sd2;
                        } else if (sd2.dir == 0) {
                            return sd1;
                        } else {
                            // prefer the action that knows more about the desired direction (right is sd1)
                            if (sd1.latDist <= 0) {
                                return sd1;
                            } else if (sd2.latDist >= 0) {
                                return sd2;
                            }
                            // when in doubt, prefer moving to the right
                            return sd1.latDist <= sd2.latDist ? sd1 : sd2;
                        }
                    } else {
                        if (can1) {
                            if (can2) {
                                // break strategic ties with tactical concerns
                                const SLState& st = SL(v);
                                if (reason1 == LCA_STRATEGIC) {
                                    if (sd1.latDist <= sd2.latDist) {
                                        return st.speedGainProbRight > st.speedGainProbLeft ? sd1 : sd2;
                                    } else {
                                        return st.speedGainProbRight > st.speedGainProbLeft ? sd2 : sd1;
                                    }
                                } else {
                                    // finish the shorter maneuver (i.e. continue the current maneuver)
                                    return std::fabs(sd1.maneuverDist) < std::fabs(sd2.maneuverDist) ? sd1 : sd2;
                                }
                            } else {
                                return sd1;
                            }
                        } else {
                            return sd2;
                        }
                    }
                }
            } else {
                return sd1;
            }
        } else {
            return sd2;
        }
    }

    static int lowestBit(int changeReason) {
        if ((changeReason & LCA_STRATEGIC) != 0) {
            return LCA_STRATEGIC;
        }
        if ((changeReason & LCA_COOPERATIVE) != 0) {
            return LCA_COOPERATIVE;
        }
        if ((changeReason & LCA_SPEEDGAIN) != 0) {
            return LCA_SPEEDGAIN;
        }
        if ((changeReason & LCA_KEEPRIGHT) != 0) {
            return LCA_KEEPRIGHT;
        }
        if ((changeReason & LCA_TRACI) != 0) {
            return LCA_TRACI;
        }
        return changeReason;
    }

    // MSLCM_SL2015::preventSliding / wantsKeepRight
    bool preventSliding(int v, double maneuverDist) const {
        const VType& t = T(v);
        if (std::fabs(maneuverDist) > t.lcMaxDistLatStanding) {
            if (t.vclass == VC_EMERGENCY) {
                return false;
            }
            const double brakeGap = s.cfBrakeGap(v, speed(v));
            return std::fabs(maneuverDist) > t.lcMaxDistLatStanding + brakeGap * std::fabs(maxSpeedLatFactor(v));
        }
        return false;
    }
    bool wantsKeepRight(int v, double keepRightProb) const {
        return keepRightProb * T(v).lcKeepRight > std::max(changeProbThresholdRight(v), SL(v).speedGainProbLeft);
    }

    // MSLCM_SL2015::computeSpeedGain
    double computeSpeedGain(int v, double latDistSublane, double defaultNextSpeed) const {
        double result = DMAX;
        const Edge& e = net.edges[edgeOf(laneOf(v))];
        const std::vector<double>& sublaneSides = e.sublaneSides;
        const double vehWidth = getWidthSL(v);
        const double rightVehSide = s.centerOnEdge(v, -1) - vehWidth * 0.5 + latDistSublane;
        const double leftVehSide = rightVehSide + vehWidth;
        for (int i = 0; i < (int)sublaneSides.size(); ++i) {
            const double leftSide = i + 1 < (int)sublaneSides.size() ? sublaneSides[i + 1] : std::max(e.width, sublaneSides[i] + POSITION_EPS);
            if (overlap(rightVehSide, leftVehSide, sublaneSides[i], leftSide)) {
                result = std::min(result, SL(v).expectedSublaneSpeeds[i]);
            }
        }
        return result - defaultNextSpeed;
    }

    // MSLCM_SL2015::sublaneEnds
    bool sublaneEnds(int v, int i, int next, double shift) const {
        const double side = i * s.latRes + shift;
        const int r = s.parallelLane(next, -1);
        const int l = s.parallelLane(next, 1);
        return ((side < -NUMERICAL_EPS && (r < 0 || !net.lanes[r].allows(s.veh[v].vclass)))
                || (side + s.latRes > laneW(next) && (l < 0 || !net.lanes[l].allows(s.veh[v].vclass))));
    }

    // MSLCM_SL2015::forecastAverageSpeed
    double forecastAverageSpeed(int v, double vSafe, double vMax, double gap, double vLeader) const {
        const double deltaV = vMax - vLeader;
        const double lookahead = speedGainLookahead(v);
        if (deltaV > 0 && gap / deltaV < lookahead && lookahead > 0) {
            // anticipate future braking: the average speed over the next few seconds
            const double foreCastTime = lookahead * 2;
            const double gapClosingTime = std::max(0.0, gap / deltaV);
            vSafe = (gapClosingTime * vSafe + (foreCastTime - gapClosingTime) * vLeader) / foreCastTime;
        }
        return vSafe;
    }

    // MSLCM_SL2015::updateExpectedSublaneSpeeds (no bidi lanes)
    void updateExpectedSublaneSpeeds(int v, const LeaderDistanceInfo& ahead, int sublaneOffset, int laneIndex) {
        SLState& st = SL(v);
        const std::vector<LaneQ>& preb = s.veh[v].bestLanes.front();
        const int lane = ch[laneIndex].lane;
        const std::vector<int>& conts = s.bestContinuation(v);
        const int next = conts.size() > 1 ? conts[1] : -1;
        const Link* link = next >= 0 ? net.linkTo(lane, next) : nullptr;
        const double shift = link != nullptr ? link->lateralShift + 0.5 * (laneW(lane) - laneW(next)) : 0;
        const double vMax = laneVehMaxSpeed(lane, v);
        for (int sublane = 0; sublane < ahead.numSublanes(); ++sublane) {
            const int edgeSublane = sublane + sublaneOffset;
            if (edgeSublane >= (int)st.expectedSublaneSpeeds.size()) {
                continue;  // (a sibling lane wider than the changer lane)
            }
            if (link != nullptr && laneW(lane) > laneW(next) + NUMERICAL_EPS && s.latRes > 0 && sublaneEnds(v, sublane, next, shift)) {
                // the sublane does not continue
                st.expectedSublaneSpeeds[edgeSublane] = 0;
                continue;
            } else if (net.lanes[lane].allows(s.veh[v].vclass)) {
                // the lane is allowed: the safe speed behind the potential leaders
                const int leader = ahead[sublane];
                const double gap = ahead.at(sublane).second;
                double vSafe;
                if (leader < 0) {
                    const double dist = preb[laneIndex].length - pos(v);
                    vSafe = s.cfFollowSpeed(v, vMax, dist, 0, 0, -1, cf::CalcReason::CURRENT, true);
                } else {
                    if (accel(leader) > 0.5 * T(leader).accel) {
                        // the leader continues accelerating to its maximum speed
                        vSafe = laneVehMaxSpeed(laneOf(leader), leader);
                    } else {
                        vSafe = s.cfFollowSpeed(v, vMax, gap, speed(leader), T(leader).decel, leader, cf::CalcReason::CURRENT, true);
                        vSafe = forecastAverageSpeed(v, vSafe, vMax, gap, speed(leader));
                    }
                }
                // pedestrians
                if (s.hasPedestrians(lane)) {
                    double foeRight, foeLeft;
                    ahead.getSublaneBorders(sublane, 0, foeRight, foeLeft);
                    const std::pair<int, double> pedLeader = s.nextBlocking(lane, pos(v) - T(v).length, foeRight, foeLeft, 0);
                    if (pedLeader.first >= 0) {
                        const double pedGap = pedLeader.second - T(v).minGap - T(v).length;
                        vSafe = std::min(s.cfStopSpeed(v, vMax, pedGap), forecastAverageSpeed(v, vSafe, vMax, pedGap, 0));
                    }
                }
                vSafe = std::min(vMax, vSafe);
                const double memoryFactor = std::pow(SPEEDGAIN_MEMORY_FACTOR, s.actionStepLengthSecs(v));
                st.expectedSublaneSpeeds[edgeSublane] = memoryFactor * st.expectedSublaneSpeeds[edgeSublane] + (1 - memoryFactor) * vSafe;
            } else {
                // the lane is forbidden
                st.expectedSublaneSpeeds[edgeSublane] = -1;
            }
        }
    }

    // MSLCM_SL2015::commitManoeuvre
    void commitManoeuvre(int v, int blocked, int blockedFully, const LeaderDistanceInfo& leaders, const LeaderDistanceInfo& neighLeaders,
                         int neighLane, double maneuverDist) {
        SLState& st = SL(v);
        const LCState& lc = s.veh[v].lc;
        const VType& t = T(v);
        if (!blocked && !blockedFully && !st.canChangeFully) {
            const double asl = s.actionStepLengthSecs(v);
            double secondsToLeaveLane;
            if (!BALLISTIC) {
                // rounded to full action steps
                secondsToLeaveLane = std::ceil(std::fabs(maneuverDist) / t.maxSpeedLat / asl) * asl;
                st.committedSpeed = std::min(std::min(lc.leftSpace / secondsToLeaveLane, s.cfMaxNextSpeed(v, speed(v))),
                                             laneVehMaxSpeed(laneOf(v), v));
            } else {
                // the seconds to leave the lane from lateral speed zero (lat.accel == -lat.decel)
                secondsToLeaveLane = estimateArrivalTime(std::fabs(maneuverDist), t.maxSpeedLat, t.lcAccelLat);
                secondsToLeaveLane = std::ceil(secondsToLeaveLane / asl) * asl;
                const double timeTillActionStep = asl - TS;
                const double nextActionStepSpeed = std::max(0., speed(v) + timeTillActionStep * accel(v));
                double nextLeftSpace;
                if (nextActionStepSpeed > 0.) {
                    nextLeftSpace = lc.leftSpace - timeTillActionStep * (speed(v) + nextActionStepSpeed) * 0.5;
                } else if (accel(v) == 0) {
                    nextLeftSpace = lc.leftSpace;
                } else {
                    nextLeftSpace = lc.leftSpace + (speed(v) * speed(v) / accel(v)) * 0.5;
                }
                const double avoidArrivalSpeed = nextActionStepSpeed + ACCEL2SPEED(avoidArrivalAccel(
                                                     nextLeftSpace, secondsToLeaveLane - timeTillActionStep, nextActionStepSpeed, t.emergencyDecel));
                st.committedSpeed = std::min(std::min(avoidArrivalSpeed, speed(v) + t.accel * asl), laneVehMaxSpeed(laneOf(v), v));
            }
            st.committedSpeed = commitFollowSpeed(v, st.committedSpeed, maneuverDist, secondsToLeaveLane, leaders,
                                                  net.lanes[laneOf(v)].rightSideOnEdge);
            st.committedSpeed = commitFollowSpeed(v, st.committedSpeed, maneuverDist, secondsToLeaveLane, neighLeaders,
                                                  net.lanes[neighLane].rightSideOnEdge);
            if (st.committedSpeed < s.cfMinNextSpeed(v, speed(v))) {
                st.committedSpeed = 0;
            }
        }
    }

    // MSCFModel::estimateArrivalTime (stub: initial and arrival speed 0, accel == decel)
    static double estimateArrivalTime(double dist, double maxSpeed, double accel) {
        if (dist <= 0) {
            return 0.;
        }
        const double accelTime = maxSpeed / accel;
        const double accelDist = accelTime * (0.5 * maxSpeed);
        if (accelDist >= dist * 0.5) {
            return 4 * std::sqrt(dist / accel);
        }
        const double constSpeedTime = (dist - accelDist * 2) / maxSpeed;
        return accelTime + constSpeedTime;
    }

    // MSCFModel::avoidArrivalAccel
    static double avoidArrivalAccel(double dist, double time, double sp, double maxDecel) {
        if (dist <= 0) {
            return -maxDecel;
        } else if (time * sp > 2 * dist) {
            return -0.5 * sp * sp / dist;
        }
        return 2 * (dist / time - sp) / time;
    }

    // MSLCM_SL2015::commitFollowSpeed (with the maximum deceleration divided by the safety factor)
    double commitFollowSpeed(int v, double sp, double latDist, double secondsToLeaveLane, const LeaderDistanceInfo& leaders, double foeOffset) {
        if (!leaders.hasVehicles()) {
            return sp;
        }
        const double vehWidth = getWidthSL(v);
        const double rightVehSide = s.centerOnEdge(v, -1) - 0.5 * vehWidth;
        const double leftVehSide = rightVehSide + vehWidth;
        const double rightVehSideDest = rightVehSide + latDist;
        const double leftVehSideDest = leftVehSide + latDist;
        const double decel = T(v).decel / safetyFactorSL(v);
        for (int i = 0; i < leaders.numSublanes(); ++i) {
            const CLeaderDist vehDist = leaders.at(i);
            if (vehDist.first >= 0) {
                const int leader = vehDist.first;
                double foeRight, foeLeft;
                leaders.getSublaneBorders(i, foeOffset, foeRight, foeLeft);
                if (overlap(rightVehSideDest, leftVehSideDest, foeRight, foeLeft)) {
                    // overlap at the end of the maneuver: follow safely
                    const double vSafe = followSpeedWithDecel(v, decel, sp, vehDist.second, speed(leader), T(leader).decel, leader);
                    sp = std::min(sp, vSafe);
                } else if (overlap(rightVehSide, leftVehSide, foeRight, foeLeft)) {
                    // overlap at the start: avoid a collision within secondsToLeaveLane
                    const double vSafe = followSpeedTransient(v, decel, secondsToLeaveLane, vehDist.second, speed(leader), T(leader).decel);
                    sp = std::min(sp, vSafe);
                }
            }
        }
        return sp;
    }

    // the car-following model's followSpeed with a temporarily changed maximum deceleration (MSCFModel::setMaxDecel)
    double followSpeedWithDecel(int v, double decel, double sp, double gap, double predSpeed, double predMaxDecel, int pred) {
        return s.cfFollowSpeedDecel(v, decel, sp, gap, predSpeed, predMaxDecel, pred);
    }

    // MSCFModel::followSpeedTransient (with the given maximum deceleration)
    double followSpeedTransient(int v, double decel, double duration, double gap2pred, double predSpeed, double predMaxDecel) const {
        (void)v;
        // minimum distance covered by the leader if braking
        const double leaderMinDist = gap2pred + distAfterTime(duration, predSpeed, -predMaxDecel);
        if (!BALLISTIC) {
            const int a = (int)std::ceil(duration / TS - TS);
            if (cf::brakeGap(a * decel, decel, 0) <= leaderMinDist) {
                // braking continuously for duration
                const double b = TS * decel * 0.5 * (a * a - a);
                return (b + leaderMinDist) / duration;
            }
            double bg = 0;
            double sp = 0;
            while (bg < leaderMinDist) {
                sp += ACCEL2SPEED(decel);
                bg += SPEED2DIST(sp);
            }
            sp -= DIST2SPEED(bg - leaderMinDist);
            return sp;
        }
        const double fullBrakingSeconds = std::sqrt(leaderMinDist * 2 / decel);
        if (fullBrakingSeconds >= duration) {
            return leaderMinDist / duration + duration * decel / 2;
        }
        return fullBrakingSeconds * decel;
    }

    // MSCFModel::distAfterTime
    static double distAfterTime(double t, double sp, double acc) {
        if (acc >= 0.) {
            return (sp + 0.5 * acc * t) * t;
        }
        const double decel = -acc;
        if (sp <= decel * t) {
            return cf::brakeGap(sp, decel, 0);
        }
        if (!BALLISTIC) {
            double result = 0;
            while (t > 0) {
                sp -= ACCEL2SPEED(decel);
                result += std::max(0.0, SPEED2DIST(sp));
                t -= TS;
            }
            return result;
        }
        const double speed2 = sp - t * decel;
        return 0.5 * (sp + speed2) * t;
    }

    // ---- informing the neighbors ---------------------------------------------------------------
    // MSLCM_SL2015::informLeader
    double informLeaderSL(int v, int blocked, int dir, const CLeaderDist& neighLead, double remainingSeconds) {
        LCState& lc = s.veh[v].lc;
        const VType& t = T(v);
        double plannedSpeed = std::min(speed(v), s.cfStopSpeed(v, speed(v), lc.leftSpace - lc.leadingBlockerLength));
        for (const LCAdvice& a : lc.advices) {
            const double vv = speed(v) + ACCEL2SPEED(a.accel);
            if (vv >= speed(v) - ACCEL2SPEED(t.decel)) {
                plannedSpeed = std::min(plannedSpeed, vv);
            }
        }
        if ((blocked & LCA_BLOCKED_BY_LEADER) != 0 && neighLead.first >= 0) {
            const int nv = neighLead.first;
            if (divergentRoute(v, nv)) {
                return plannedSpeed;
            }
            // overtake the leader or follow it
            const double dv = plannedSpeed - speed(nv);
            const double overtakeDist = neighLead.second + T(nv).lengthWithGap() + t.length
                                        + secureGap(nv, v, speed(nv), speed(v), t.decel);
            if ((dv < t.lcOvertakeDeltaSpeedFactor * net.lanes[laneOf(v)].speed + NUMERICAL_EPS
                    // overtaking on the right on an uncongested highway is forbidden (noOvertakeLCLeft)
                    || (dir == LCA_MLEFT && !congested(v) && !s.opt.overtakeRight)
                    // not enough space to overtake?
                    || lc.leftSpace - lc.leadingBlockerLength - s.cfBrakeGap(v, speed(v)) < overtakeDist
                    // not enough time to overtake?
                    || dv * remainingSeconds < overtakeDist)
                    && !isStopped(nv)) {
                // cannot overtake
                inform(nv, -1, dir | LCA_AMBLOCKINGLEADER);
                // slow down smoothly to follow the leader
                const double targetSpeed = s.cfFollowSpeed(v, speed(v), neighLead.second, speed(nv), T(nv).decel, nv, cf::CalcReason::CURRENT, true);
                if (targetSpeed < speed(v)) {
                    const double decel = ACCEL2SPEED(std::min(t.decel, std::max(MIN_FALLBEHIND, (speed(v) - targetSpeed) / remainingSeconds)));
                    const double nextSpeed = std::min(plannedSpeed, std::max(0.0, speed(v) - decel));
                    addLCSpeedAdvice(v, nextSpeed, dir);
                    return nextSpeed;
                }
                // the leader is fast enough anyway
                addLCSpeedAdvice(v, targetSpeed, dir);
                return plannedSpeed;
            }
            // overtaking, the leader should not accelerate
            inform(nv, speed(nv), dir | LCA_AMBLOCKINGLEADER);
            return -1;
        } else if (neighLead.first >= 0) {
            // not blocked now: stay far enough from the leader
            const int nv = neighLead.first;
            double dv, nextNVSpeed;
            if (!BALLISTIC) {
                nextNVSpeed = speed(nv) - HELP_OVERTAKE;  // conservative
                dv = SPEED2DIST(speed(v) - nextNVSpeed);
            } else {
                nextNVSpeed = speed(nv) - HELP_OVERTAKE * s.actionStepLengthSecs(v);
                dv = SPEED2DIST(speed(v) - nextNVSpeed);
            }
            const double targetSpeed = s.cfFollowSpeed(v, speed(v), neighLead.second - dv, nextNVSpeed, T(nv).decel, nv, cf::CalcReason::CURRENT, true);
            addLCSpeedAdvice(v, targetSpeed, dir);
            return std::min(targetSpeed, plannedSpeed);
        }
        return plannedSpeed;
    }

    // MSLCM_SL2015::informFollower
    void informFollowerSL(int v, int blocked, int dir, const CLeaderDist& neighFollow, double remainingSeconds, double plannedSpeed) {
        const int nv = neighFollow.first;
        const VType& t = T(v);
        if (v == s.traceVeh) {
            std::fprintf(stderr, "    informFollower %s blocked=%x gap=%.3f remaining=%.3f planned=%.3f unwilling=%d\n", nv < 0 ? "-" : s.veh[nv].id.c_str(),
                         blocked, neighFollow.second, remainingSeconds, plannedSpeed, nv >= 0 ? (int)unwillingToHelp(v, plannedSpeed, nv) : -1);
        }
        // will we request help to cut in before the follower or allow to be overtaken?
        if (nv >= 0 && unwillingToHelp(v, plannedSpeed, nv)) {
            return;
        }
        if ((blocked & LCA_BLOCKED_BY_FOLLOWER) != 0 && nv >= 0) {
            if (divergentRoute(v, nv)) {
                return;
            }
            // fast enough to cut in without help?
            if (plannedSpeed - speed(nv) >= HELP_OVERTAKE) {
                const double neededGap = secureGap(nv, v, speed(nv), plannedSpeed, t.decel);
                if ((neededGap - neighFollow.second) / remainingSeconds < (plannedSpeed - speed(nv))) {
                    // the follower might even accelerate but not too much
                    inform(nv, plannedSpeed - HELP_OVERTAKE, dir | LCA_AMBLOCKINGFOLLOWER);
                    return;
                }
            }
            // the other vehicle is assumed to brake with its maximum deceleration for one second to help
            const double helpDecel = T(nv).decel * HELP_DECEL_FACTOR;
            const double neighNewSpeed = std::max(0., speed(nv) - ACCEL2SPEED(helpDecel));
            const double neighNewSpeed1s = std::max(0., speed(nv) - helpDecel);
            const double dv = plannedSpeed - neighNewSpeed1s;
            // the new gap between the follower and ego if the follower brakes for 1s
            const double decelGap = neighFollow.second + dv;
            const double sGap = secureGap(nv, v, neighNewSpeed1s, plannedSpeed, t.decel);
            if (decelGap > 0 && decelGap >= sGap) {
                // braking of the neighbor could help: how hard?
                const double vsafe1 = std::max(neighNewSpeed, s.cfFollowSpeed(nv, speed(nv), neighFollow.second + SPEED2DIST(plannedSpeed),
                                                                              plannedSpeed, t.decel, v, cf::CalcReason::CURRENT, true));
                const double vsafe = std::max(neighNewSpeed, s.cfFollowSpeed(nv, speed(nv), neighFollow.second + SPEED2DIST(plannedSpeed - vsafe1),
                                                                             plannedSpeed, t.decel, v, cf::CalcReason::CURRENT, true));
                inform(nv, vsafe, dir | LCA_AMBLOCKINGFOLLOWER);
            } else if (dv > 0 && dv * remainingSeconds > (sGap - decelGap + POSITION_EPS)) {
                // decelerating once is sufficient to open up a large enough gap in time
                inform(nv, neighNewSpeed, dir | LCA_AMBLOCKINGFOLLOWER);
            } else if (dir == LCA_MRIGHT && !s.opt.overtakeRight && !congested(nv)) {
                const double vhelp = std::max(neighNewSpeed, HELP_OVERTAKE);
                inform(nv, vhelp, dir | LCA_AMBLOCKINGFOLLOWER);
            } else {
                double vhelp = std::max(speed(nv), speed(v) + HELP_OVERTAKE);
                if (speed(nv) > speed(v)
                        && ((dir == LCA_MRIGHT && STEPS2TIME(s.veh[v].waitingTime) > LCA_RIGHT_IMPATIENCE)
                            || (dir == LCA_MLEFT && plannedSpeed > CUT_IN_LEFT_SPEED_THRESHOLD)
                            || (dir == LCA_MLEFT && s.veh[v].lc.leftSpace > MAX_ONRAMP_LENGTH))) {
                    // let the follower slow down to increase the likelyhood that later vehicles will be slow enough to help
                    vhelp = std::max(neighNewSpeed, speed(v) + HELP_OVERTAKE);
                    if ((speed(nv) - speed(v)) / helpDecel < remainingSeconds) {
                        inform(nv, vhelp, dir | LCA_AMBLOCKINGFOLLOWER);
                        return;
                    }
                }
                inform(nv, vhelp, dir | LCA_AMBLOCKINGFOLLOWER);
                // this follower is supposed to overtake us: slow down smoothly to allow this
                const double overtakeDist = neighFollow.second + t.lengthWithGap() + T(nv).length
                                            + secureGap(v, nv, plannedSpeed, vhelp, T(nv).decel);
                // the speed difference to create a sufficiently large gap
                const double needDV = overtakeDist / remainingSeconds;
                // the deceleration should not be too strong
                addLCSpeedAdvice(v, std::max(vhelp - needDV, speed(v) - ACCEL2SPEED(t.decel)), dir);
            }
        } else if (nv >= 0) {
            const double vsafe = speedPreservingSecureGap(v, nv, neighFollow.second, plannedSpeed);
            inform(nv, vsafe, dir | LCA_AMBLOCKINGFOLLOWER);
        }
    }

    // MSLCM_SL2015::informLeaders / informFollowers
    double informLeadersSL(int v, int blocked, int dir, const std::vector<CLeaderDist>& blockers, double remainingSeconds) {
        LCState& lc = s.veh[v].lc;
        double plannedSpeed = speed(v);
        double space = lc.leftSpace;
        if (lc.leadingBlockerLength != 0) {
            space -= lc.leadingBlockerLength - POSITION_EPS - T(v).minGap;
            if (space <= 0) {
                space = lc.leftSpace;  // ignore the leading blocker
            }
        }
        const double safe = s.cfStopSpeed(v, speed(v), space);
        plannedSpeed = std::min(plannedSpeed, safe);
        for (const CLeaderDist& b : blockers) {
            plannedSpeed = std::min(plannedSpeed, informLeaderSL(v, blocked, dir, b, remainingSeconds));
        }
        return plannedSpeed;
    }
    void informFollowersSL(int v, int blocked, int dir, const std::vector<CLeaderDist>& blockers, double remainingSeconds, double plannedSpeed) {
        for (const CLeaderDist& b : blockers) {
            informFollowerSL(v, blocked, dir, b, remainingSeconds, plannedSpeed);
        }
    }
};

// ---------------------------------------------------------------------------
// the sweep of an edge in sublane mode (called by Sim::changeLanes)
// ---------------------------------------------------------------------------
void Sim::laneChangeSublaneEdge(int e, std::vector<LCMessage>& m, std::vector<LaneChangeEvent>& ev, std::vector<DetEvent>& d,
                                std::vector<int>& changed, std::vector<std::vector<int>>& newLanes) {
    SublaneChanger c(*this, e, m, ev, d, changed);
    c.laneChangeSublane();
    newLanes.resize(c.ch.size());
    for (size_t j = 0; j < c.ch.size(); ++j) {
        newLanes[j].swap(c.ch[j].done);
    }
}

// ---------------------------------------------------------------------------
// vehicle-local parts of SL2015
// ---------------------------------------------------------------------------
// MSLCM_SL2015::prepareStep (with MSAbstractLaneChangeModel::prepareStep)
void Sim::slPrepareStep(int v) {
    LCState& lc = veh[v].lc;
    SLState& st = sl[v];
    lc.canceled[0] = lc.canceled[1] = lc.canceled[2] = 0;
    resetSavedLCStates(v);
    st.lastLateralGapRight = NO_NEIGHBOR;
    st.lastLateralGapLeft = NO_NEIGHBOR;
    st.committedSpeed = 0;
    // keep the information about the strategic change direction
    lc.ownState = (lc.ownState & (LCA_STRATEGIC | LCA_COOPERATIVE)) ? (lc.ownState & LCA_WANTS_LANECHANGE) : 0;
    lc.leadingBlockerLength = 0;
    lc.leftSpace = 0;
    lc.advices.clear();
    lc.dontBrake = false;
    st.cfRelated.clear();
    st.cfRelatedReady = false;
    const Kin& k = kin[v];
    const double halfWidth = (vt(v).width + NUMERICAL_EPS) * 0.5;
    // only changes within the lane bounds (opened up in _wantsChangeSublane)
    st.safeLatDistRight = net.lanes[k.lane].width * 0.5 + k.posLat - halfWidth;
    st.safeLatDistLeft = net.lanes[k.lane].width * 0.5 - k.posLat - halfWidth;
    // truncate to work around numerical instability between different builds
    st.speedGainProbRight = std::ceil(st.speedGainProbRight * 100000.0) * 0.00001;
    st.speedGainProbLeft = std::ceil(st.speedGainProbLeft * 100000.0) * 0.00001;
    st.keepRightProb = std::ceil(st.keepRightProb * 100000.0) * 0.00001;
    // the expected sublane speeds of a new edge
    const int curEdge = net.lanes[k.lane].edge;
    const Edge& e = net.edges[curEdge];
    if (st.expectedSublaneSpeeds.size() != e.sublaneSides.size()) {
        std::vector<double> newExpectedSpeeds;
        for (int lane : e.lanes) {
            const int subLanes = net.numSublanes(net.lanes[lane].width);
            for (int i = 0; i < subLanes; ++i) {
                newExpectedSpeeds.push_back(laneVehMaxSpeed(lane, v));
            }
        }
        if (!st.expectedSublaneSpeeds.empty() && st.lastEdge >= 0
                && net.edges[st.lastEdge].sublaneSides.size() == st.expectedSublaneSpeeds.size()) {
            // copy the old values
            const int subLaneShift = computeSublaneShift(st.lastEdge, curEdge);
            if (subLaneShift < std::numeric_limits<int>::max()) {
                for (int i = 0; i < (int)st.expectedSublaneSpeeds.size(); ++i) {
                    const int newI = i + subLaneShift;
                    if (newI > 0 && newI < (int)newExpectedSpeeds.size()) {
                        newExpectedSpeeds[newI] = st.expectedSublaneSpeeds[i];
                    }
                }
            }
        }
        st.expectedSublaneSpeeds = newExpectedSpeeds;
        st.lastEdge = curEdge;
    }
    const VType& t = vt(v);
    if (t.lcSigma > 0) {
        // MSLCM_SL2015::getLateralDrift: OUProcess::step scaled with the relative speed
        const double dt = actionStepLengthSecs(v);
        const double timeScale = std::max(NUMERICAL_EPS, (1 - t.lcSigma) * 100);
        uint64_t draw = 0;
        const double normal = randNormPolar(0, 1, [&]() { return rand01(opt.seed, (uint64_t)v * 1024 + draw++, (uint64_t)step, RP_LC_SIGMA); });
        const double newState = std::exp(-dt / timeScale) * st.sigmaState + t.lcSigma * std::sqrt(2 * dt / timeScale) * normal;
        const double deltaState = newState - st.sigmaState;
        st.sigmaState += deltaState * k.speed / net.lanes[k.lane].speed;
    }
}

// MSLCM_SL2015::computeSublaneShift
int Sim::computeSublaneShift(int prevEdge, int curEdge) const {
    // the first lane that targets the new edge
    int prevShift = 0;
    for (int lane : net.edges[prevEdge].lanes) {
        for (int li : net.lanes[lane].links) {
            if (net.lanes[net.links[li].lane].edge == curEdge) {
                int curShift = 0;
                const int target = net.links[li].lane;
                for (int lane2 : net.edges[curEdge].lanes) {
                    if (lane2 == target) {
                        return prevShift + curShift;
                    }
                    curShift += net.numSublanes(net.lanes[lane2].width);
                }
            }
        }
        prevShift -= net.numSublanes(net.lanes[lane].width);
    }
    return std::numeric_limits<int>::max();
}

// MSLCM_SL2015::changed
void Sim::slChanged(int v) {
    LCState& lc = veh[v].lc;
    SLState& st = sl[v];
    if (!st.canChangeFully) {
        // continue the maneuver but acknowledge a change to the right
        if (lc.maneuverDist < 0) {
            st.keepRightProb = 0;
        }
        return;
    }
    lc.ownState = 0;
    st.speedGainProbRight = 0;
    st.speedGainProbLeft = 0;
    st.keepRightProb = 0;
    if (bestLaneOffset(v) == 0) {
        lc.leadingBlockerLength = 0;
        lc.leftSpace = 0;
    }
    lc.lookAheadSpeed = LOOK_AHEAD_MIN_SPEED;
    lc.advices.clear();
    lc.dontBrake = false;
    lc.leadingBlockerLength = 0;
}

// MSLCM_SL2015::_patchSpeed
double Sim::slPatchSpeed(int v, double min, const double wanted, double max) {
    min = std::max(min, 0.0);
    if (wanted <= 0) {
        return wanted;
    }
    LCState& lc = veh[v].lc;
    const VType& t = vt(v);
    const int state = lc.ownState;
    const double speed = kin[v].speed;
    double nVSafe = wanted;
    bool gotOne = false;
    // letting vehicles merge in at the end of the lane in case of counter-lane change, step#2
    if (lc.leadingBlockerLength != 0) {
        const double space = lc.leftSpace - lc.leadingBlockerLength - POSITION_EPS;
        if (space >= 0) {
            // decelerate towards a place which allows the blocking leader to merge in in front
            const double safe = cfStopSpeed(v, speed, space, -1, cf::CalcReason::LANE_CHANGE);
            max = std::min(max, safe);
            if (safe < wanted) {
                if (safe < min) {
                    const double vMinEmergency = cfMinNextSpeedEmergency(v, speed);
                    if (safe >= vMinEmergency) {
                        // permit harder braking if needed and helpful
                        min = std::max(vMinEmergency, safe);
                    }
                }
                nVSafe = std::max(min, safe);
                gotOne = true;
            }
        }
    }
    const double coopWeight = std::max(0.0, std::min(1.0, t.lcCooperativeSpeed));
    if (v == traceVeh) {
        std::fprintf(stderr, "%.2f slPatchSpeed %s min=%.3f wanted=%.3f max=%.3f state=%x advices=%d lbl=%.3f\n", STEPS2TIME(now),
                     veh[v].id.c_str(), min, wanted, max, state, (int)lc.advices.size(), lc.leadingBlockerLength);
        for (const LCAdvice& a : lc.advices) {
            std::fprintf(stderr, "    advice accel=%.3f flag=%x\n", a.accel, a.flag);
        }
    }
    for (const LCAdvice& a : lc.advices) {
        const double vv = speed + ACCEL2SPEED(a.accel);
        if (vv >= min && vv <= max) {
            if (a.flag & LCA_CHANGE_TO_HELP) {
                nVSafe = std::min(vv * coopWeight + (1 - coopWeight) * wanted, nVSafe);
            } else {
                // own advice, no scaling needed
                nVSafe = std::min(vv, nVSafe);
            }
            gotOne = true;
        }
    }
    if (gotOne && !lc.dontBrake) {
        return nVSafe;
    }
    // is the vehicle blocked?
    if ((state & LCA_WANTS_LANECHANGE) != 0 && (state & LCA_BLOCKED) != 0) {
        if ((state & LCA_STRATEGIC) != 0) {
            // necessary decelerations are controlled via vSafe; without any, speed up
            return (max + wanted) / 2.0;
        } else if ((state & LCA_COOPERATIVE) != 0) {
            // only minor adjustments in speed
            if ((state & LCA_BLOCKED_BY_LEADER) != 0) {
                return (min + wanted) / 2.0;
            }
            if ((state & LCA_BLOCKED_BY_FOLLOWER) != 0) {
                return (max + wanted) / 2.0;
            }
        }
    }
    // accelerate if being a blocking leader (and not having to change lanes)
    if ((state & LCA_AMBLOCKINGLEADER) != 0) {
        return (max + wanted) / 2.0;
    }
    return wanted;
}

}  // namespace ps
