// Lateral state of vehicles in sublane mode (continuous lane changes with --lanechange.duration and the sublane
// model): MSVehicle::myState.myPosLat, shadow lanes (MSAbstractLaneChangeModel::updateShadowLane) and the lateral
// positions on the further lanes.
#include "model.hpp"

#include <algorithm>
#include <cmath>

namespace ps {

// MSVehicle::getLateralOverlap
double Sim::lateralOverlap(int v, double posLat, int lane) const {
    return std::fabs(posLat) + 0.5 * vt(v).width - 0.5 * net.lanes[lane].width;
}

// MSLane::getParallelLane
int Sim::parallelLane(int lane, int offset) const {
    const Edge& e = net.edges[net.lanes[lane].edge];
    const int idx = net.lanes[lane].index + offset;
    return idx >= 0 && idx < (int)e.lanes.size() ? e.lanes[idx] : -1;
}

// MSAbstractLaneChangeModel::getShadowLane(lane, posLat): the neighboring lane the vehicle overlaps with (a lane
// change that has not passed its midpoint reserves the target lane)
int Sim::shadowLaneFor(int v, int lane, double posLat) const {
    const std::vector<int>& noShadow = lat[v].noShadow;
    if (std::find(noShadow.begin(), noShadow.end(), lane) != noShadow.end()) {
        return -1;
    }
    if (lateralOverlap(v, posLat, lane) > NUMERICAL_EPS) {
        return parallelLane(lane, posLat < 0 ? -1 : 1);
    }
    const LCState& lc = veh[v].lc;
    if (lc.isChangingLanes() && lc.completion < 0.5) {
        return parallelLane(lane, lc.lcDirection);
    }
    return -1;
}

// MSAbstractLaneChangeModel::updateShadowLane (with MSVehicle::updateFurtherLanes for the shadow further lanes)
void Sim::updateShadowLane(int v) {
    Kin& k = kin[v];
    LatState& ls = lat[v];
    k.shadowLane = shadowLaneFor(v, k.lane, k.posLat);
    int passed[MAX_FURTHER + 1];
    int nPassed = 0;
    if (k.shadowLane >= 0) {
        passed[nPassed++] = k.shadowLane;
        for (int i = 0; i < k.nFurther; ++i) {
            const int shadowFurther = shadowLaneFor(v, k.further[i], ls.furtherPosLat[i]);
            if (shadowFurther >= 0 && net.linkTo(shadowFurther, passed[nPassed - 1]) != nullptr) {
                passed[nPassed++] = shadowFurther;
            }
        }
        std::reverse(passed, passed + nPassed);
    } else if (veh[v].lc.isChangingLanes() && lateralOverlap(v, k.posLat, k.lane) > NUMERICAL_EPS) {
        // "could not finish continuous lane change (lane disappeared)"
        endLaneChangeManeuver(v);
    }
    // the further lanes of the shadow (passed[nPassed - 1] is the shadow lane)
    int newFurther[MAX_FURTHER];
    double newPosLat[MAX_FURTHER];
    int n = 0;
    double backPosOnPreviousLane = k.pos - vt(v).length;
    int fi = 0;
    for (int pi = nPassed - 2; pi >= 0 && backPosOnPreviousLane < 0 && n < MAX_FURTHER; --pi) {
        const int further = passed[pi];
        newFurther[n] = further;
        backPosOnPreviousLane += net.lanes[further].length;
        if (fi < k.nShadowFurther && further == k.shadowFurther[fi]) {
            newPosLat[n] = ls.shadowFurtherPosLat[fi];  // the known lateral position stays
            ++fi;
        } else {
            newPosLat[n] = n == 0 ? k.posLat : newPosLat[n - 1];
        }
        ++n;
    }
    std::copy(newFurther, newFurther + n, k.shadowFurther);
    std::copy(newPosLat, newPosLat + n, ls.shadowFurtherPosLat);
    k.nShadowFurther = n;
}

// MSVehicle::brakeForOverlap: an overlapping vehicle must stop where its shadow lane ends
bool Sim::brakeForOverlap(int v, int link, int lane) const {
    const Kin& k = kin[v];
    const Link& L = net.links[link];
    const double futurePosLat = k.posLat + (lane != k.lane && net.lanes[lane].internal
                                            ? net.links[net.lanes[lane].incoming[0].link].lateralShift : 0);
    const double overlap = lateralOverlap(v, futurePosLat, lane);
    const Edge& e = net.edges[net.lanes[lane].edge];
    const double edgeWidth = net.lanes[e.lanes.back()].rightSideOnEdge + net.lanes[e.lanes.back()].width;
    if (overlap <= POSITION_EPS || vt(v).width > edgeWidth || L.via >= 0) {
        return false;
    }
    const int shadow = shadowLaneFor(v, L.lane, k.posLat);
    const bool laneInternalAfterInternal = net.lanes[lane].internal && !net.lanes[lane].incoming.empty()
                                           && net.lanes[net.lanes[lane].incoming[0].lane].internal;
    if (!(shadow < 0 || (net.lanes[shadow].permissions & veh[v].vclass) != veh[v].vclass || laneInternalAfterInternal)) {
        return false;
    }
    // (situations where the shadow lane is part of a double connection with the current lane do not count)
    return k.shadowLane < 0 || net.lanes[k.shadowLane].links.empty()
           || net.links[net.lanes[k.shadowLane].links.front()].lane != L.lane;
}

// MSVehicle::getCenterOnEdge
double Sim::centerOnEdge(int v, int lane) const {
    const Kin& k = KV(v);
    const Lane& M = net.lanes[k.lane];
    if (lane < 0 || net.lanes[lane].edge == M.edge) {
        return M.rightSideOnEdge + k.posLat + 0.5 * M.width;
    }
    const Lane& L = net.lanes[lane];
    if (lane == k.shadowLane) {
        if (shadowDirection(v) == -1) {
            return L.rightSideOnEdge + L.width + k.posLat + 0.5 * M.width;
        }
        return L.rightSideOnEdge - M.width + k.posLat + 0.5 * M.width;
    }
    const LatPos& ls = LV(v);
    for (int i = 0; i < k.nFurther; ++i) {
        if (k.further[i] == lane) {
            return L.rightSideOnEdge + ls.furtherPosLat[i] + 0.5 * L.width;
        }
    }
    for (int i = 0; i < k.nShadowFurther; ++i) {
        if (k.shadowFurther[i] == lane) {
            const Lane& S = net.lanes[k.shadowLane];
            return L.rightSideOnEdge + ls.shadowFurtherPosLat[i] + 0.5 * L.width
                   + (M.rightSideOnEdge + 0.5 * M.width - (S.rightSideOnEdge + 0.5 * S.width));
        }
    }
    return M.rightSideOnEdge + k.posLat + 0.5 * M.width;
}

// MSVehicle::getLatOffset: the offset to add to the vehicle's lateral position on its lane to obtain it on the lane
// (no opposite direction lanes)
double Sim::latOffset(int v, int lane) const {
    const Kin& k = KV(v);
    const Lane& M = net.lanes[k.lane];
    const Lane& L = net.lanes[lane];
    if (L.edge == M.edge) {
        return M.rightSideOnEdge - L.rightSideOnEdge;
    }
    const LatPos& ls = LV(v);
    for (int i = 0; i < k.nFurther; ++i) {
        if (k.further[i] == lane) {
            return ls.furtherPosLat[i] - k.posLat;
        }
    }
    for (int i = 0; i < k.nShadowFurther; ++i) {
        if (k.shadowFurther[i] == lane) {
            return latOffset(v, k.shadowLane) + ls.shadowFurtherPosLat[i] - k.posLat;
        }
    }
    // (maneuver reservations on the further lanes' neighbors: not modelled)
    return 0;
}

// MSVehicle::lateralDistanceToLane: the minimum lateral distance to move fully onto the neighboring lane
double Sim::lateralDistanceToLane(int v, int offset) const {
    const Kin& k = KV(v);
    const double halfCurrentLaneWidth = 0.5 * net.lanes[k.lane].width;
    const double halfVehWidth = 0.5 * (vt(v).width + NUMERICAL_EPS);
    const double latPos = k.posLat;
    const double leftLimit = halfCurrentLaneWidth - halfVehWidth - latPos;
    const double rightLimit = -halfCurrentLaneWidth + halfVehWidth - latPos;
    double latLaneDist = 0;
    if (offset == 0) {
        if (latPos + halfVehWidth > halfCurrentLaneWidth) {
            latLaneDist = halfCurrentLaneWidth - latPos - halfVehWidth;
        } else if (latPos - halfVehWidth < -halfCurrentLaneWidth) {
            latLaneDist = -halfCurrentLaneWidth - latPos + halfVehWidth;
        }
    } else if (offset == -1) {
        latLaneDist = rightLimit - (vt(v).width + NUMERICAL_EPS);
    } else if (offset == 1) {
        latLaneDist = leftLimit + (vt(v).width + NUMERICAL_EPS);
    }
    return latLaneDist;
}

// MSLCM_SL2015::outsideEdge
bool Sim::outsideEdge(int v) const {
    return leftSideOnEdge(v) < 0 || rightSideOnEdge(v) > net.edges[net.lanes[KV(v).lane].edge].width;
}

// MSAbstractLaneChangeModel::getShadowDirection
int Sim::shadowDirection(int v) const {
    const LCState& lc = veh[v].lc;
    const Kin& k = KV(v);
    if (lc.isChangingLanes()) {
        return lc.pastMidpoint() ? -lc.lcDirection : lc.lcDirection;
    }
    if (k.shadowLane < 0) {
        return 0;
    }
    if (net.lanes[k.shadowLane].edge == net.lanes[k.lane].edge) {
        return net.lanes[k.shadowLane].index - net.lanes[k.lane].index;
    }
    return 1;  // (overlap with an opposite direction lane)
}

}  // namespace ps
