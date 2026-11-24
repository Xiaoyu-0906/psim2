// Lane changing: MSLaneChanger sweep + LC2013 decision model.
// Ported from SUMO 1.28 (MSLaneChanger.cpp, MSLCM_LC2013.cpp,
// MSAbstractLaneChangeModel.cpp, MSLCHelper.cpp) for instantaneous lane
// changes without sublanes, opposite driving, pedestrians or TraCI.
//
// Execution structure: every edge with more than one lane is swept by its own
// task. Inside an edge the sweep keeps SUMO's sequential front-to-back order.
// Edges are processed in stages ("levels", downstream first, see
// computeEdgeLevels); the edges of one stage run in parallel. Vehicles of edges
// of the same or later stages are read from a copy taken at the start of the
// phase (Sim::kinSnap), those of earlier stages from the live state
// (Sim::lcSnapStage), so a sweep sees the lane changes of the stages before
// it. Side effects on vehicles of other edges are messages;
// those for vehicles of later stages arrive before their edge is swept.
#include <chrono>
#include <cstdio>
#include <functional>

#include "model.hpp"

#include "lc.hpp"

namespace ps {


// ---------------------------------------------------------------------------
// vehicle-local parts of LC2013
// ---------------------------------------------------------------------------
double Sim::lcPatchSpeed(int v, double min, double wanted, double max) {
    if (latRes > 0 && isSL(v)) {
        return slPatchSpeed(v, min, wanted, max);
    }
    min = std::max(min, 0.0);
    Veh& x = veh[v];
    LCState& lc = x.lc;
    const VType& t = vt(v);
    const int state = lc.ownState;
    double nVSafe = wanted;
    bool gotOne = false;
    if (lc.leadingBlockerLength != 0) {
        const double space = lc.leftSpace - lc.leadingBlockerLength - POSITION_EPS;
        if (space > 0 && (!isInternal(kin[v].lane) || net.edges[curEdge(v)].roundabout)) {
            const double vMinEmergency = cfMinNextSpeedEmergency(v, kin[v].speed);
            const double safe = cfStopSpeed(v, kin[v].speed, space, -1, cf::CalcReason::LANE_CHANGE);
            max = std::min(max, std::max(safe, vMinEmergency));
            if (safe < wanted) {
                if (safe < min) {
                    if (safe >= vMinEmergency) {
                        min = std::max(vMinEmergency, safe);
                    }
                }
                nVSafe = std::max(min, safe);
                gotOne = true;
            }
        }
    }
    const double coopWeight = std::max(0.0, std::min(1.0, t.lcCooperativeSpeed));
    const int filter = (((lc.ownState & LCA_LEFT) != 0 ? LCA_MLEFT : 0) |
                        ((lc.ownState & LCA_RIGHT) != 0 ? LCA_MRIGHT : 0));
    for (const LCAdvice& i : lc.advices) {
        const double a = i.accel;
        const double vv = kin[v].speed + ACCEL2SPEED(a);
        // (ballistic: -1 means no restriction)
        if (vv >= min && vv <= max && (!BALLISTIC || vv != -1)) {
            if (i.flag & LCA_CHANGE_TO_HELP) {
                nVSafe = std::min(vv * coopWeight + (1 - coopWeight) * wanted, nVSafe);
            } else if ((i.flag & filter) != filter) {
                continue;
            } else {
                nVSafe = std::min(vv, nVSafe);
            }
            gotOne = true;
        }
    }
    if (gotOne && !lc.dontBrake) {
        return nVSafe;
    }
    if ((state & LCA_WANTS_LANECHANGE) != 0 && (state & LCA_BLOCKED) != 0) {
        if ((state & LCA_STRATEGIC) != 0) {
            return (max + wanted) / 2.0;
        } else if ((state & LCA_COOPERATIVE) != 0) {
            if ((state & LCA_BLOCKED_BY_LEADER) != 0) {
                if (wanted >= 0.) {
                    return (std::max(0., min) + wanted) / 2.0;
                }
                return wanted;
            }
            if ((state & LCA_BLOCKED_BY_FOLLOWER) != 0) {
                return (max + wanted) / 2.0;
            }
        }
    }
    if ((state & LCA_AMBLOCKINGLEADER) != 0 && t.lcCooperativeSpeed >= 0) {
        return (max + wanted) / 2.0;
    }
    if (!net.edges[net.lanes[kin[v].lane].edge].hasLaneChanger) {
        // MSLCM_LC2013::changed() on single-lane roads
        lc.ownState = 0;
        lc.speedGainProbLeft = 0;
        lc.speedGainProbRight = 0;
        lc.keepRightProb = 0;
        if (bestLaneOffset(v) == 0) {
            lc.leadingBlockerLength = 0;
            lc.leftSpace = 0;
        }
        lc.lookAheadSpeed = LOOK_AHEAD_MIN_SPEED;
        lc.advices.clear();
        lc.dontBrake = false;
        lc.leadingBlockerLength = 0;
    }
    return wanted;
}

// MSLCM_LC2013::prepareStep + MSAbstractLaneChangeModel::prepareStep
void Sim::lcPrepareStep(int v) {
    if (latRes > 0 && isSL(v)) {
        slPrepareStep(v);
        return;
    }
    LCState& lc = veh[v].lc;
    lc.canceled[0] = lc.canceled[1] = lc.canceled[2] = 0;
    resetSavedLCStates(v);
    if (!lc.isChangingLanes()) {
        lc.ownState = (lc.ownState & LCA_STRATEGIC) ? (lc.ownState & LCA_WANTS_LANECHANGE) : 0;
    }
    lc.leadingBlockerLength = 0;
    lc.leftSpace = 0;
    lc.advices.clear();
    lc.dontBrake = false;
}

// MSLCM_LC2013::computeSpeedLat (MSAbstractLaneChangeModel::computeSpeedLat bounded by lcMaxSpeedLatStanding and
// lcMaxSpeedLatFactor)
double Sim::lcComputeSpeedLat(int v, double maneuverDist, bool urgent) const {
    const VType& t = vt(v);
    const LCState& lc = veh[v].lc;
    double result;
    if (t.maxSpeedLatSet) {
        const int stepsToChange = (int)std::ceil(std::fabs(maneuverDist) / SPEED2DIST(t.maxSpeedLat));
        result = DIST2SPEED(maneuverDist / stepsToChange);
    } else {
        result = maneuverDist / STEPS2TIME(opt.laneChangeDuration);
    }
    if (lc.leftSpace > POSITION_EPS || !urgent) {
        const double standing = t.lcMaxSpeedLatStanding >= 0 ? t.lcMaxSpeedLatStanding : t.maxSpeedLat;
        const double factor = t.lcMaxSpeedLatFactor >= 0 ? t.lcMaxSpeedLatFactor : 1;
        double speedBound = standing + factor * kin[v].speed;
        if (lc.isChangingLanes()) {
            speedBound = std::max(0.5, speedBound);  // LC_RESOLUTION_SPEED_LAT
        }
        result = std::max(-speedBound, std::min(speedBound, result));
    }
    return result;
}

// MSAbstractLaneChangeModel::estimateLCDuration (-1: the change is not completed while braking to a stop)
double Sim::estimateLCDuration(int v, double speed, double remainingManeuverDist, double decel, bool urgent) const {
    const VType& t = vt(v);
    if (t.lcMaxSpeedLatStanding < 0 && t.lcMaxSpeedLatFactor < 0) {
        return t.maxSpeedLatSet ? remainingManeuverDist / t.maxSpeedLat : STEPS2TIME(opt.laneChangeDuration);
    }
    if (remainingManeuverDist == 0) {
        return 0;
    }
    const double v0 = speed;
    const double D = remainingManeuverDist;
    const double b = decel;
    const double wmin = t.lcMaxSpeedLatStanding >= 0 ? t.lcMaxSpeedLatStanding : t.maxSpeedLat;
    const double f = t.lcMaxSpeedLatFactor >= 0 ? t.lcMaxSpeedLatFactor : 1;
    const double wmax = t.maxSpeedLat;
    const double vm = (wmax - wmin) / f;
    double distSoFar = 0.;
    double timeSoFar = 0.;
    double vv = v0;
    if (vv > vm) {
        const double wmaxTime = (v0 - vm) / b;
        const double d1 = wmax * wmaxTime;
        if (d1 >= D) {
            return D / wmax;
        }
        distSoFar += d1;
        timeSoFar += wmaxTime;
        vv = vm;
    }
    if (vv > 0) {
        const double tt = vv / b;
        const double d2 = (wmin + 0.5 * f * vv) * tt;
        if (distSoFar + d2 >= D) {
            const double x = 0.5 * f * b;
            const double y = wmin + f * vv;
            const double p = 0.5 * y / x;
            const double q = (D - distSoFar) / x;
            return timeSoFar + p + std::sqrt(p * p - q);
        }
        distSoFar += d2;
        timeSoFar += tt;
    }
    if (wmin == 0) {
        double maneuverDist = remainingManeuverDist;
        const double vModel = lcComputeSpeedLat(v, maneuverDist, urgent);
        const double result = D / vModel;
        if (vModel > HALTING_SPEED && (vModel + kin[v].accel * result) > HALTING_SPEED) {
            return result;
        }
        return -1;
    }
    return timeSoFar + (D - distSoFar) / wmin;
}

// MSAbstractLaneChangeModel::remainingTime (of a continuous lane change)
Time Sim::lcRemainingTime(int v) const {
    const VType& t = vt(v);
    const LCState& lc = veh[v].lc;
    if (t.lcMaxSpeedLatStanding < 0 && t.lcMaxSpeedLatFactor < 0) {
        if (t.maxSpeedLatSet) {
            return TIME2STEPS((1. - lc.completion) * lc.maneuverDist / t.maxSpeedLat);
        }
        return (Time)((1. - lc.completion) * (double)opt.laneChangeDuration);
    }
    return TIME2STEPS(estimateLCDuration(v, kin[v].speed, std::fabs(lc.maneuverDist * (1 - lc.completion)), vt(v).decel,
                                         (lc.ownState & LCA_URGENT) != 0));
}

// MSAbstractLaneChangeModel::endLaneChangeManeuver (+ MSVehicle::fixPosition)
void Sim::endLaneChangeManeuver(int v) {
    LCState& lc = veh[v].lc;
    lc.completion = 1;
    lc.speedLat = 0;
    Kin& k = kin[v];
    k.shadowLane = -1;  // cleanupShadowLane (the partial occupations follow in rebuildPartials)
    k.nShadowFurther = 0;
    if (sublane) {
        lat[v].noShadow.clear();
    }
    if (opt.laneChangeDuration > 0) {
        k.posLat = 0;  // MSVehicle::fixPosition
    }
}

// ---------------------------------------------------------------------------
// phase driver
// ---------------------------------------------------------------------------
// Edge levels. SUMO sweeps edges in the order of its active-lane list, which in
// busy traffic puts downstream edges first. Here the edges with a lane changer
// form a graph (an arc leads to every lane-changing edge reachable through
// edges without lane changer); the level of an edge is the longest path to an
// edge without successors in that graph, cycles are broken where a depth-first
// search meets them. Edges of one level are processed in parallel.
void Sim::computeEdgeLevels() {
    const int n = (int)net.edges.size();
    lcEdges.clear();
    for (int e = 0; e < n; ++e) {
        if (net.edges[e].hasLaneChanger) {
            lcEdges.push_back(e);
        }
    }
    // successors in the graph of lane-changing edges
    std::vector<std::vector<int>> succ(n);
    auto nextEdges = [&](int e) {
        std::vector<int> out;
        for (int l : net.edges[e].lanes) {
            for (int k : net.lanes[l].links) {
                const int to = net.lanes[net.links[k].viaOrLane()].edge;
                if (std::find(out.begin(), out.end(), to) == out.end()) {
                    out.push_back(to);
                }
            }
        }
        return out;
    };
    for (int e : lcEdges) {
        std::vector<int> stack = nextEdges(e);
        std::vector<int> seen;
        while (!stack.empty()) {
            const int f = stack.back();
            stack.pop_back();
            if (std::find(seen.begin(), seen.end(), f) != seen.end() || f == e) {
                continue;
            }
            seen.push_back(f);
            if (net.edges[f].hasLaneChanger) {
                succ[e].push_back(f);
            } else if (seen.size() < 16) {
                for (int g : nextEdges(f)) {
                    stack.push_back(g);
                }
            }
        }
    }
    edgeLevel.assign(n, -1);
    std::vector<char> onStack(n, 0);
    std::function<int(int)> level = [&](int e) -> int {
        if (edgeLevel[e] >= 0) {
            return edgeLevel[e];
        }
        if (onStack[e]) {
            return 0;
        }
        onStack[e] = 1;
        int lv = 0;
        for (int sE : succ[e]) {
            lv = std::max(lv, level(sE) + 1);
        }
        onStack[e] = 0;
        edgeLevel[e] = lv;
        return lv;
    };
    int maxLevel = 0;
    for (int e : lcEdges) {
        maxLevel = std::max(maxLevel, level(e));
    }
    edgesByLevel.assign(maxLevel + 1, {});
    for (int e : lcEdges) {
        edgesByLevel[edgeLevel[e]].push_back(e);
    }
    // colors: lane-changing edges within LC_COLOR_HOPS edge hops (forward or backward,
    // through any edges) never share a stage, so that a vehicle partially on two
    // edges is never moved by two lane changers working on stale copies at once
    constexpr int LC_COLOR_HOPS = 3;
    std::vector<std::vector<int>> nbr(n);
    for (int e : lcEdges) {
        std::vector<int> frontier{e}, seen{e};
        for (int hop = 0; hop < LC_COLOR_HOPS && !frontier.empty(); ++hop) {
            std::vector<int> next;
            for (int f : frontier) {
                std::vector<int> adj = nextEdges(f);
                adj.insert(adj.end(), net.edges[f].predecessors.begin(), net.edges[f].predecessors.end());
                for (int g : adj) {
                    if (std::find(seen.begin(), seen.end(), g) == seen.end()) {
                        seen.push_back(g);
                        next.push_back(g);
                        if (net.edges[g].hasLaneChanger) {
                            nbr[e].push_back(g);
                        }
                    }
                }
            }
            frontier.swap(next);
        }
    }
    lcNbr = nbr;
    // greedy coloring, edges with many neighbours first (deterministic: ties by index)
    std::vector<int> byDegree = lcEdges;
    std::stable_sort(byDegree.begin(), byDegree.end(), [&](int a, int b) { return nbr[a].size() > nbr[b].size(); });
    std::vector<int> color(n, -1);
    int nColors = 0;
    for (int e : byDegree) {
        std::vector<char> used(nColors + 1, 0);
        for (int g : nbr[e]) {
            if (color[g] >= 0) {
                used[color[g]] = 1;
            }
        }
        int c = 0;
        while (used[c]) {
            ++c;
        }
        color[e] = c;
        nColors = std::max(nColors, c + 1);
    }
    edgesByColor.assign(nColors, {});
    lcEdgeColor.assign(n, -1);
    lcColorRank.assign(n, -1);
    for (int e : lcEdges) {
        lcEdgeColor[e] = color[e];
        lcColorRank[e] = (int)edgesByColor[color[e]].size();
        edgesByColor[color[e]].push_back(e);
    }
    // MSLaneChanger::postloadInitLC (checkOpened)
    lcCheckOpened.assign(n, 0);
    for (int e : lcEdges) {
        const Edge& E = net.edges[e];
        bool found = false;
        for (int lane : E.lanes) {
            for (int li : net.lanes[lane].links) {
                const Link& L = net.links[li];
                if (L.tls >= 0 || L.havePriority()) {
                    continue;
                }
                for (int lane2 : E.lanes) {
                    if (lane2 == lane) {
                        continue;
                    }
                    for (int li2 : net.lanes[lane2].links) {
                        const Link& L2 = net.links[li2];
                        if (net.lanes[L.lane].edge == net.lanes[L2.lane].edge && L.lane != L2.lane
                                && (net.lanes[lane].permissions & net.lanes[lane2].permissions & net.lanes[L.lane].permissions
                                    & net.lanes[L2.lane].permissions & net.lanes[L.viaOrLane()].permissions
                                    & net.lanes[L2.viaOrLane()].permissions & ~(VC_PEDESTRIAN | VC_BICYCLE)) != 0
                                && L.foeLinks != L2.foeLinks) {
                            found = true;
                        }
                    }
                }
            }
        }
        lcCheckOpened[e] = found;
    }
}

void Sim::applyLCMessage(const LCMessage& m) {
    LCState& lc = veh[m.receiver].lc;
    if (m.kind == 0) {
        lc.advices.push_back({SPEED2ACCEL(m.value - kin[m.receiver].speed), LCA_CHANGE_TO_HELP});
        lc.ownState |= m.flags;
        lcStateSnap[m.receiver] = lc.ownState;
    } else {
        lc.leadingBlockerLength = std::max(m.value, lc.leadingBlockerLength);
    }
}

static double lcClock() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void Sim::changeLanes() {
    const double c0 = lcClock();
    collectOccupiedLanes(lcOccupied);
    // stages: colors (default), one stage with all edges (--lc-single), levels
    // with vehicles (--lc-levels), or with --sumo-order one edge at a time in the
    // order of SUMO's active-lane list
    if (lcSchedPos.size() != net.edges.size()) {
        lcSchedPos.assign(net.edges.size(), -1);
        lcPending.assign(net.edges.size(), {});
    }
    std::vector<int>& schedPos = lcSchedPos;
    std::vector<std::vector<int>>& sched = lcSched;
    for (const std::vector<int>& todo : sched) {
        for (int e : todo) {
            schedPos[e] = -1;
        }
    }
    sched.clear();
    if (opt.sumoOrder) {
        for (int l : activeList) {
            const int e = net.lanes[l].edge;
            if (!net.edges[e].hasLaneChanger) {
                break;
            }
            if (schedPos[e] < 0) {
                schedPos[e] = (int)sched.size();
                sched.push_back({e});
            }
        }
    } else if (!opt.lcLevels && !opt.lcSingle) {
        // stages by color: each stage reads the edges of earlier stages as published,
        // the others as they were before the phase. The edges with vehicles come from
        // the lanes with vehicles; within a color they keep the order of edgesByColor.
        if (lcEdgeMark.size() != net.edges.size()) {
            lcEdgeMark.assign(net.edges.size(), 0);
        }
        for (int l : lcOccupied) {
            lcEdgeMark[net.lanes[l].edge] = 1;
        }
        std::vector<std::vector<int>> byColor(edgesByColor.size());
        for (size_t c = 0; c < edgesByColor.size(); ++c) {
            for (int e : edgesByColor[c]) {
                if (lcEdgeMark[e]) {
                    byColor[c].push_back(e);
                }
            }
        }
        for (int l : lcOccupied) {
            lcEdgeMark[net.lanes[l].edge] = 0;
        }
        if (opt.lcDynColor) {
            // colors of this step: greedy over the edges with vehicles (in the order of the static colors),
            // only neighbours with vehicles count
            std::vector<int> order;
            for (std::vector<int>& todo : byColor) {
                std::sort(todo.begin(), todo.end(), [&](int a, int b) { return lcColorRank[a] < lcColorRank[b]; });
                order.insert(order.end(), todo.begin(), todo.end());
                todo.clear();
            }
            std::vector<char> used;
            for (int e : order) {
                used.assign(byColor.size() + 1, 0);
                for (int g : lcNbr[e]) {
                    if (schedPos[g] >= 0) {
                        used[schedPos[g]] = 1;
                    }
                }
                int c = 0;
                while (used[c]) {
                    ++c;
                }
                schedPos[e] = c;
                if (c >= (int)byColor.size()) {
                    byColor.resize(c + 1);
                }
                byColor[c].push_back(e);
            }
        }
        for (std::vector<int>& todo : byColor) {
            if (todo.empty()) {
                continue;
            }
            if (opt.lcDynColor) {
                std::sort(todo.begin(), todo.end(), [&](int a, int b) { return lcColorRank[a] < lcColorRank[b]; });
            }
            for (int e : todo) {
                schedPos[e] = (int)sched.size();
            }
            sched.push_back(std::move(todo));
        }
    }
    if (opt.lcSingle) {
        // one stage: every lane changer reads the other edges as they were before
        // the phase (snapshot) and messages to their vehicles take effect afterwards
        std::vector<int> todo;
        for (int e : lcEdges) {
            for (int l : net.edges[e].lanes) {
                if (!lanes[l].vehs.empty()) {
                    schedPos[e] = 0;
                    todo.push_back(e);
                    break;
                }
            }
        }
        if (!todo.empty()) {
            sched.push_back(todo);
        }
    }
    for (const std::vector<int>& lvl : edgesByLevel) {
        if (opt.sumoOrder || !opt.lcLevels) {
            break;
        }
        std::vector<int> todo;
        for (int e : lvl) {
            bool any = false;
            for (int l : net.edges[e].lanes) {
                any |= !lanes[l].vehs.empty();
            }
            if (any) {
                schedPos[e] = (int)sched.size();
                todo.push_back(e);
            }
        }
        if (!todo.empty()) {
            sched.push_back(todo);
        }
    }
    // copy of the vehicle state for reads across edges, the stage of each vehicle's edge,
    // the check of the stage assumption
    pool.setTag(ThreadPool::T_LC_SNAP);
    pool.parallel_for((int)running.size(), [&](int i) {
        const int v = running[i];
        kinSnap[v] = kin[v];
        lcStateSnap[v] = veh[v].lc.ownState;
        lcLeftSpaceSnap[v] = veh[v].lc.leftSpace;
        if (sublane) {
            lcStratBlockedSnap[v] = veh[v].lc.strategicBlocked;
            latSnap[v] = lat[v];
            lcManeuverSnap[v] = veh[v].lc.maneuverDist;
            lcSpeedLatSnap[v] = veh[v].lc.speedLat;
        }
        lcSnapStage[v] = schedPos[net.lanes[kin[v].lane].edge];
        checkLCSchedule(v);
    }, 256);
    snapshotBrutto(lcOccupied);
    lcCurStage = 0;
    kview = kinSnap.data();
    tLcSched += lcClock() - c0;
    std::vector<std::vector<LCMessage>>& pendingMsgs = lcPending;
    std::vector<std::vector<std::vector<int>>> newLanes;
    std::vector<std::vector<DetEvent>> stageDet;
    std::vector<std::vector<int>> stageChanged;
    nLCStages += (long long)sched.size();
    for (size_t k = 0; k < sched.size(); ++k) {
        const std::vector<int>& todo = sched[k];
        for (int e : todo) {
            for (const LCMessage& m : pendingMsgs[e]) {
                applyLCMessage(m);
            }
            pendingMsgs[e].clear();
        }
        newLanes.resize(todo.size());
        stageDet.resize(todo.size());
        stageChanged.resize(todo.size());
        lcCurStage = (int)k;
        pool.setTag(ThreadPool::T_LC);
        pool.parallel_for((int)todo.size(), [&](int i) {
            const int e = todo[i];
            lcMessages[e].clear();
            lcEventsPerEdge[e].clear();
            if (sublane) {
                lcShadowPerEdge[e].clear();
            }
            stageDet[i].clear();
            stageChanged[i].clear();
            if (latRes > 0) {
                laneChangeSublaneEdge(e, lcMessages[e], lcEventsPerEdge[e], stageDet[i], stageChanged[i], newLanes[i]);
            } else {
                LaneChanger c(*this, e, lcMessages[e], lcEventsPerEdge[e], stageDet[i], stageChanged[i]);
                c.laneChange();
                newLanes[i].resize(c.ch.size());
                for (size_t j = 0; j < c.ch.size(); ++j) {
                    newLanes[i][j].swap(c.ch[j].done);
                }
            }
        }, 1);
        const double c1 = lcClock();
        // vehicles of this stage's edges count as published from here on (their copy still holds the
        // state before the phase, used just below)
        lcCurStage = (int)k + 1;
        // vehicles whose further lanes change (before the copies are published)
        std::vector<int> changed;
        for (size_t i = 0; i < todo.size(); ++i) {
            changed.insert(changed.end(), stageChanged[i].begin(), stageChanged[i].end());
        }
        std::vector<int> affected;
        std::vector<int> partialChanged = changed;
        if (sublane) {
            // vehicles whose shadow lanes changed (continuous lane changes)
            for (size_t i = 0; i < todo.size(); ++i) {
                partialChanged.insert(partialChanged.end(), lcShadowPerEdge[todo[i]].begin(), lcShadowPerEdge[todo[i]].end());
            }
            std::sort(partialChanged.begin(), partialChanged.end());
            partialChanged.erase(std::unique(partialChanged.begin(), partialChanged.end()), partialChanged.end());
        }
        for (int v : partialChanged) {
            const Kin& before = kinSnap[v];
            for (int i = 0; i < before.nFurther; ++i) {
                affected.push_back(before.further[i]);
            }
            for (int i = 0; i < kin[v].nFurther; ++i) {
                affected.push_back(kin[v].further[i]);
            }
            for (const Kin* kk : {&before, static_cast<const Kin*>(&kin[v])}) {
                if (kk->shadowLane >= 0) {
                    affected.push_back(kk->shadowLane);
                    affected.insert(affected.end(), kk->shadowFurther, kk->shadowFurther + kk->nShadowFurther);
                }
            }
        }
        // publish the lane lists of this stage (the vehicle states are live from now on, see lcCurStage)
        for (size_t i = 0; i < todo.size(); ++i) {
            const Edge& edge = net.edges[todo[i]];
            for (size_t j = 0; j < edge.lanes.size(); ++j) {
                const int l = edge.lanes[j];
                lanes[l].vehs.swap(newLanes[i][j]);
                newLanes[i][j].clear();
                bruttoSnap[l] = lanes[l].brutto;
                markBruttoSnap(l);
            }
        }
        std::vector<DetEvent> det;
        for (size_t i = 0; i < todo.size(); ++i) {
            const int e = todo[i];
            if (trackActiveList) {
                for (int l : net.edges[e].lanes) {
                    if (!lanes[l].vehs.empty() && !amActive[l]) {
                        amActive[l] = 1;
                        lcToAdd.push_back(l);
                    }
                }
            }
            for (const LaneChangeEvent& ev : lcEventsPerEdge[e]) {
                if (opt.lcOutput) {
                    lcEvents.push_back(ev);
                } else {
                    nLaneChangesTotal++;
                }
            }
            det.insert(det.end(), stageDet[i].begin(), stageDet[i].end());
        }
        applyDetEvents(det);
        // MSAbstractLaneChangeModel::primaryLaneChanged -> updateDriveItems: with
        // --sumo-order right after the edge, otherwise at the end of the stage
        // (lane changers of one stage read the registrations of the stage before)
        for (int v : changed) {
            updateDriveItems(v);
        }
        // messages to vehicles whose edge is still to be swept wait for that edge,
        // all others take effect now
        for (int e : todo) {
            for (const LCMessage& m : lcMessages[e]) {
                const int re = net.lanes[kinSnap[m.receiver].lane].edge;
                if (schedPos[re] > (int)k) {
                    pendingMsgs[re].push_back(m);
                } else {
                    applyLCMessage(m);
                }
            }
        }
        const double c2 = lcClock();
        tLcSerial += c2 - c1;
        // vehicles that changed lanes moved their back onto other lanes
        if (!partialChanged.empty()) {
            updatePartials(partialChanged, affected);
            nLcPartialRebuilds++;
        }
        tLcPartials += lcClock() - c2;
        lcStageEdges += (long long)todo.size();
    }
    kview = nullptr;
    for (int l : lcToAdd) {
        activeList.insert(activeList.begin(), l);
    }
    lcToAdd.clear();
}

// partial occupants of the lanes the vehicles that changed lanes left or entered with their back
// (the same lists as rebuildPartials: ordered by position on the lane, then by vehicle index)
void Sim::updatePartials(const std::vector<int>& changed, std::vector<int>& affected) {
    std::sort(affected.begin(), affected.end());
    affected.erase(std::unique(affected.begin(), affected.end()), affected.end());
    for (int l : affected) {
        std::vector<int>& p = lanes[l].partials;
        const bool wasEmpty = p.empty();
        p.erase(std::remove_if(p.begin(), p.end(), [&](int v) {
            return std::find(changed.begin(), changed.end(), v) != changed.end();
        }), p.end());
        for (int v : changed) {
            const Kin& k = kin[v];
            for (int i = 0; i < k.nFurther; ++i) {
                if (k.further[i] == l && std::find(p.begin(), p.end(), v) == p.end()) {
                    p.push_back(v);
                }
            }
            if (k.shadowLane >= 0 && std::find(p.begin(), p.end(), v) == p.end()
                    && (k.shadowLane == l || std::find(k.shadowFurther, k.shadowFurther + k.nShadowFurther, l) != k.shadowFurther + k.nShadowFurther)) {
                p.push_back(v);
            }
        }
        if (wasEmpty && !p.empty() && std::find(partialLanes.begin(), partialLanes.end(), l) == partialLanes.end()) {
            partialLanes.push_back(l);
        }
        std::sort(p.begin(), p.end());
        if (p.size() > 1) {
            std::stable_sort(p.begin(), p.end(), [&](int a, int b) { return positionOnLane(a, l) < positionOnLane(b, l); });
        }
    }
}

// bruttoSnap: the occupancy of all lanes at the start of a phase; only the given lanes (those with
// vehicles) are written, the others are reset from the previous snapshot (empty lanes have brutto 0)
void Sim::snapshotBrutto(const std::vector<int>& occupied) {
    for (int l : bruttoSnapLanes) {
        bruttoSnap[l] = 0;
        bruttoSnapMark[l] = 0;
    }
    bruttoSnapLanes.clear();
    for (int l : occupied) {
        bruttoSnap[l] = lanes[l].brutto;
        markBruttoSnap(l);
    }
}

// run-time check of the assumption behind the lane-changing stages: no vehicle reaches onto two
// lane-changing edges of the same stage (they would both read and move it). Called in parallel.
void Sim::checkLCSchedule(int v) {
    const Kin& k = kin[v];
    if (k.nFurther == 0) {
        return;
    }
    int edges[MAX_FURTHER + 1];
    int n = 0;
    int prev = -1;
    for (int i = -1; i < k.nFurther; ++i) {
        const int e = net.lanes[i < 0 ? k.lane : k.further[i]].edge;
        if (e != prev) {
            edges[n++] = e;
            prev = e;
        }
    }
    int span = lcMaxSpanEdges.load(std::memory_order_relaxed);
    while (n > span && !lcMaxSpanEdges.compare_exchange_weak(span, n, std::memory_order_relaxed)) {
    }
    for (int a = 0; a < n; ++a) {
        for (int b = a + 1; b < n; ++b) {
            if (edges[a] != edges[b] && lcSchedPos[edges[a]] >= 0 && lcSchedPos[edges[a]] == lcSchedPos[edges[b]]) {
                nLcScheduleConflicts.fetch_add(1, std::memory_order_relaxed);
            }
        }
    }
}

}  // namespace ps
