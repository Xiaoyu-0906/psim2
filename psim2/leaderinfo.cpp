// Leaders and followers per sublane (MSLeaderInfo.cpp)
#include "model.hpp"

#include <cmath>

namespace ps {

LeaderInfo::LeaderInfo(const Sim& s, double laneWidth, int ego, double latOffset)
    : sim(&s), width(laneWidth),
      // (without a lateral resolution: one slot)
      vehs(s.latRes > 0 ? std::max(1, (int)std::ceil(laneWidth / s.latRes)) : 1, -1),
      freeSublanes((int)vehs.size()) {
    if (ego >= 0) {
        getSubLanes(ego, latOffset, egoRightMost, egoLeftMost);
        freeSublanes -= egoRightMost;
        freeSublanes -= (int)vehs.size() - 1 - egoLeftMost;
    }
}

int LeaderInfo::addLeader(int v, bool beyond, double latOffset) {
    if (v < 0) {
        return freeSublanes;
    }
    if (vehs.size() == 1) {
        if (!beyond || vehs[0] < 0) {
            vehs[0] = v;
            freeSublanes = 0;
            hasVehs = true;
        }
        return freeSublanes;
    }
    int rightmost, leftmost;
    getSubLanes(v, latOffset, rightmost, leftmost);
    for (int sublane = rightmost; sublane <= leftmost; ++sublane) {
        if ((egoRightMost < 0 || (egoRightMost <= sublane && sublane <= egoLeftMost)) && (!beyond || vehs[sublane] < 0)) {
            if (vehs[sublane] < 0) {
                freeSublanes--;
            }
            vehs[sublane] = v;
            hasVehs = true;
        }
    }
    return freeSublanes;
}

void LeaderInfo::clear() {
    vehs.assign(vehs.size(), -1);
    freeSublanes = (int)vehs.size();
    if (egoRightMost >= 0) {
        freeSublanes -= egoRightMost;
        freeSublanes -= (int)vehs.size() - 1 - egoLeftMost;
    }
}

void LeaderInfo::getSubLanes(int v, double latOffset, int& rightmost, int& leftmost) const {
    if (vehs.size() == 1) {
        rightmost = 0;
        leftmost = 0;
        return;
    }
    const Sim& s = *sim;
    const double res = s.latRes;
    const double vehCenter = s.KV(v).posLat + 0.5 * width + latOffset + offset * res;
    const double vehHalfWidth = 0.5 * s.vt(v).width;
    double rightVehSide = vehCenter - vehHalfWidth;
    double leftVehSide = vehCenter + vehHalfWidth;
    if (s.actionStepLength(v) != DELTA_T) {
        // the lateral movement until the next action
        const double maneuverDist = s.maneuverDistOf(v);
        const double speedLat = s.speedLatOf(v);
        if (maneuverDist < 0. || speedLat < 0.) {
            rightVehSide -= std::min(s.vt(v).maxSpeedLat * s.actionStepLengthSecs(v), -std::min(0., maneuverDist));
        }
        if (maneuverDist > 0. || speedLat > 0.) {
            leftVehSide += std::min(s.vt(v).maxSpeedLat * s.actionStepLengthSecs(v), std::max(0., maneuverDist));
        }
    }
    if (rightVehSide > width || leftVehSide < 0) {
        rightmost = -1000;
        leftmost = -2000;
    } else {
        rightmost = std::max(0, (int)std::floor((rightVehSide + NUMERICAL_EPS) / res));
        leftmost = std::min((int)vehs.size() - 1, (int)std::floor(std::max(0.0, leftVehSide - NUMERICAL_EPS) / res));
    }
}

void LeaderInfo::getSublaneBorders(int sublane, double latOffset, double& rightSide, double& leftSide) const {
    const double res = sim->latRes > 0 ? sim->latRes : width;
    rightSide = sublane * res + latOffset - offset * sim->latRes;
    leftSide = std::min((sublane + 1) * res, width) + latOffset - offset * sim->latRes;
}

bool LeaderInfo::hasStoppedVehicle() const {
    if (!hasVehs) {
        return false;
    }
    for (int v : vehs) {
        if (v >= 0 && sim->isStopped(v)) {
            return true;
        }
    }
    return false;
}

bool LeaderInfo::hasVehicle(int v) const {
    if (!hasVehs) {
        return false;
    }
    for (int u : vehs) {
        if (u == v) {
            return true;
        }
    }
    return false;
}

LeaderDistanceInfo::LeaderDistanceInfo(const Sim& s, int v, double dist, double laneWidth)
    : LeaderInfo(s, laneWidth, -1, 0.), dists(1, dist) {
    vehs.assign(1, v);
    freeSublanes = v >= 0 ? 0 : 1;
    hasVehs = v >= 0;
}

int LeaderDistanceInfo::addLeader(int v, double gap, double latOffset, int sublane) {
    if (v < 0) {
        return freeSublanes;
    }
    if (vehs.size() == 1) {
        sublane = 0;
    }
    if (sublane >= 0 && sublane < (int)vehs.size()) {
        if (gap < dists[sublane]) {
            if (vehs[sublane] < 0) {
                freeSublanes--;
            }
            vehs[sublane] = v;
            dists[sublane] = gap;
            hasVehs = true;
        }
        return freeSublanes;
    }
    int rightmost, leftmost;
    getSubLanes(v, latOffset, rightmost, leftmost);
    for (int i = rightmost; i <= leftmost; ++i) {
        if ((egoRightMost < 0 || (egoRightMost <= i && i <= egoLeftMost)) && gap < dists[i]) {
            if (vehs[i] < 0) {
                freeSublanes--;
            }
            vehs[i] = v;
            dists[i] = gap;
            hasVehs = true;
        }
    }
    return freeSublanes;
}

void LeaderDistanceInfo::addLeaders(const LeaderDistanceInfo& other) {
    const int maxSubLane = std::min(numSublanes(), other.numSublanes());
    for (int i = 0; i < maxSubLane; i++) {
        addLeader(other.vehs[i], other.dists[i], 0, i);
    }
}

void LeaderDistanceInfo::clear() {
    LeaderInfo::clear();
    dists.assign(vehs.size(), std::numeric_limits<double>::max());
}

void LeaderDistanceInfo::patchGaps(double amount) {
    for (size_t i = 0; i < vehs.size(); ++i) {
        if (vehs[i] >= 0) {
            dists[i] += amount;
        }
    }
}

std::pair<int, double> LeaderDistanceInfo::getClosest() const {
    double minGap = -1;
    int v = -1;
    if (hasVehs) {
        minGap = std::numeric_limits<double>::max();
        for (size_t i = 0; i < vehs.size(); ++i) {
            if (vehs[i] >= 0 && dists[i] < minGap) {
                minGap = dists[i];
                v = vehs[i];
            }
        }
    }
    return {v, minGap};
}

double LeaderDistanceInfo::getMinDistToStopped() const {
    double result = std::numeric_limits<double>::max();
    if (!hasVehs) {
        return result;
    }
    for (size_t i = 0; i < vehs.size(); ++i) {
        if (vehs[i] >= 0 && sim->isStopped(vehs[i])) {
            result = std::min(result, dists[i]);
        }
    }
    return result;
}

int CriticalFollowerDistanceInfo::addFollower(int v, int ego, double gap, double latOffset, int sublane) {
    if (v < 0) {
        return freeSublanes;
    }
    const Sim& s = *sim;
    const double requiredGap = haveOppositeLeaders ? 0
                               : s.cfSecureGap(v, ego, s.kin[v].speed, s.kin[ego].speed, s.vt(ego).decel);
    const double missingGap = requiredGap - gap;
    if (vehs.size() == 1) {
        sublane = 0;
    }
    auto better = [&](int i) {
        // overlapping vehicles are stored preferably, among those with a missing gap the closer ones
        return (missingGap > missingGaps[i] || (missingGap > 0 && gap < dists[i]) || (gap < 0 && dists[i] > 0))
               && !(gap > 0 && dists[i] < 0) && !(missingGaps[i] > 0 && dists[i] < gap);
    };
    if (sublane >= 0 && sublane < (int)vehs.size()) {
        if (better(sublane)) {
            if (vehs[sublane] < 0) {
                freeSublanes--;
            }
            vehs[sublane] = v;
            dists[sublane] = gap;
            missingGaps[sublane] = missingGap;
            hasVehs = true;
        }
        return freeSublanes;
    }
    int rightmost, leftmost;
    getSubLanes(v, latOffset, rightmost, leftmost);
    for (int i = rightmost; i <= leftmost; ++i) {
        if ((egoRightMost < 0 || (egoRightMost <= i && i <= egoLeftMost)) && better(i)) {
            if (vehs[i] < 0) {
                freeSublanes--;
            }
            vehs[i] = v;
            dists[i] = gap;
            missingGaps[i] = missingGap;
            hasVehs = true;
        }
    }
    return freeSublanes;
}

void CriticalFollowerDistanceInfo::clear() {
    LeaderDistanceInfo::clear();
    missingGaps.assign(vehs.size(), -std::numeric_limits<double>::max());
}

// MSLeaderDistanceInfo::moveSamePosTo: leaders at the same position as ego (on its edge) become followers
// (the other's vehicle flag is not updated, as in SUMO)
void LeaderDistanceInfo::moveSamePosTo(int ego, LeaderDistanceInfo& other) {
    const Sim& s = *sim;
    const double pos = s.KV(ego).pos;
    const int egoEdge = s.net.lanes[s.KV(ego).lane].edge;
    for (int i = 0; i < (int)vehs.size(); ++i) {
        const int u = vehs[i];
        if (u >= 0 && dists[i] < 0 && s.KV(u).pos == pos && s.net.lanes[s.KV(u).lane].edge == egoEdge) {
            other.vehs[i] = u;
            other.dists[i] = dists[i];
            vehs[i] = -1;
            dists[i] = -1;
        }
    }
}

}  // namespace ps
