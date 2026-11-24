// Sublane lane queries shared by the lane changer and the insertion (MSLane::getLastVehicleInformation,
// getFirstVehicleInformation, getFollowersOnConsecutive). anyVeh(lane, f) iterates the vehicles of a lane rear to
// front (full and partial occupants merged by position, MSLane::AnyVehicleIterator) until f returns false.
#pragma once

#include "model.hpp"

namespace ps {

// MSLane::getLastVehicleInformation
template <class AnyVeh>
LeaderInfo Sim::lastVehicleInformationT(int lane, int ego, double latOff, double minPos, int ignore, AnyVeh anyVeh) const {
    LeaderInfo result(*this, net.lanes[lane].width, ego, latOff);
    int freeSublanes = 1;
    anyVeh(lane, [&](int u) {
        if (freeSublanes <= 0) {
            return false;
        }
        if (u != ego && u != ignore && std::max(0.0, positionOnLane(u, lane)) >= minPos) {
            freeSublanes = result.addLeader(u, true, latOffset(u, lane));
        }
        return true;
    });
    return result;
}

// MSLane::getFirstVehicleInformation (front to rear)
template <class AnyVeh>
LeaderInfo Sim::firstVehicleInformationT(int lane, int ego, double latOff, bool onlyFrontOnLane, double maxPos, AnyVeh anyVeh) const {
    LeaderInfo result(*this, net.lanes[lane].width, ego, latOff);
    thread_local std::vector<int> all;
    all.clear();
    anyVeh(lane, [&](int u) {
        all.push_back(u);
        return true;
    });
    int freeSublanes = 1;
    for (auto it = all.rbegin(); it != all.rend() && freeSublanes > 0; ++it) {
        const int u = *it;
        if (u != ego && positionOnLane(u, lane) <= maxPos && (!onlyFrontOnLane || isFrontOnLane(u, lane))) {
            freeSublanes = result.addLeader(u, true, latOffset(u, lane));
        }
    }
    return result;
}

// MSLane::getFollowersOnConsecutive (MinorLinkMode::FOLLOW_ALWAYS); egoOnRoad: MSVehicle::isOnRoad (false on insertion)
template <class AnyVeh>
CriticalFollowerDistanceInfo Sim::followersOnConsecutiveT(int lane, int ego, double backOffset, bool allSublanes, bool egoOnRoad,
                                                          AnyVeh anyVeh) const {
    const Kin& ke = KV(ego);
    const VType& te = vt(ego);
    const double egoPos = backOffset + te.length;
    const double egoLatDist = net.lanes[ke.lane].rightSideOnEdge - net.lanes[lane].rightSideOnEdge;
    const double width = net.lanes[lane].width;
    CriticalFollowerDistanceInfo result(*this, width, allSublanes ? -1 : ego, allSublanes ? 0 : egoLatDist, false);
    if (latRes > 0 && egoLatDist == 0) {
        // ego outside the lane bounds far enough for another vehicle between itself and the first sublane
        const double left = leftSideOnLane(ego, ke.lane);
        const double right = rightSideOnLane(ego, ke.lane);
        if (left < -latRes) {
            result.setSublaneOffset(int(-left / latRes));
        } else if (right > width + latRes) {
            result.setSublaneOffset(-int((right - width) / latRes));
        }
    }
    anyVeh(lane, [&](int u) {
        if (u != ego && positionOnLane(u, lane) < egoPos) {
            const double latOff = latOffset(u, lane);
            const double dist = backOffset - positionOnLane(u, lane) - vt(u).minGap;
            result.addFollower(u, ego, dist, latOff);
        }
        return true;
    });
    if (result.numFreeSublanes() <= 0) {
        return result;
    }
    double searchDist = net.lanes[lane].maxBrakeDist - backOffset;
    thread_local std::vector<int> egoFurther;
    egoFurther.clear();
    for (int i = 0; i < ke.nFurther; ++i) {
        egoFurther.push_back(net.lanes[ke.further[i]].edge);
    }
    if (ke.pos < te.length && ke.nFurther == 0 && net.lanes[ke.lane].logicalPred >= 0) {
        egoFurther.push_back(net.lanes[net.lanes[ke.lane].logicalPred].edge);  // on insertion
    }
    auto egoFurtherHas = [&](int e) { return std::find(egoFurther.begin(), egoFurther.end(), e) != egoFurther.end(); };
    std::vector<int> visited(net.edges[net.lanes[lane].edge].lanes);
    struct Inc {
        int lane, link;
        double length;
    };
    std::vector<Inc> toExamine, newFound;
    for (const IncomingLane& i : net.lanes[lane].incoming) {
        toExamine.push_back({i.lane, i.link, i.length});
    }
    std::vector<LinkLeader> leaders;
    const int egoEdge = net.lanes[ke.lane].edge;
    while (!toExamine.empty()) {
        for (const Inc& it : toExamine) {
            const int next = it.lane;
            searchDist = std::min(searchDist, net.lanes[next].maxBrakeDist - backOffset);
            const LeaderInfo first = firstVehicleInformationT(next, -1, 0, false, DMAX, anyVeh);
            const LeaderInfo firstFront = firstVehicleInformationT(next, -1, 0, true, DMAX, anyVeh);
            if (backOffset + it.length - net.lanes[next].length < 0 && egoFurtherHas(net.lanes[next].edge)) {
                // junction foes that would interfere with lane changing
                leaderInfo(it.link, ego, -backOffset, leaders, cfComputeLC);
                for (const LinkLeader& ll : leaders) {
                    if (ll.veh >= 0) {
                        const bool egoIsLeader = isLeader(ll.veh, it.link, ego, ll.gap);
                        const double gap = egoIsLeader ? -ll.gap - vt(ll.veh).lengthWithGap() - te.minGap : ll.gap + te.length;
                        result.addFollower(ll.veh, ego, gap);
                    }
                }
            }
            for (int i = 0; i < first.numSublanes(); ++i) {
                int u = first[i] == ego ? firstFront[i] : first[i];
                double agap = 0;
                if (u >= 0 && u != ego) {
                    if (!isFrontOnLane(u, next)) {
                        // the front of u is already on a divergent trajectory: the gap is between its flank and ego's back
                        agap = it.length - net.lanes[next].length + backOffset;
                        if (net.usingInternalLanes) {
                            agap -= vt(u).minGap;  // (ego should have left the intersection still occupied by u)
                        }
                        const bool differentEdge = net.lanes[KV(u).lane].edge != egoEdge;
                        if (agap > 0 && differentEdge) {
                            u = firstFront[i];
                            if (u >= 0 && u != ego) {
                                agap = it.length - KV(u).pos + backOffset - vt(u).minGap;
                            } else {
                                u = -1;
                            }
                        } else if (differentEdge && result.hasVehicle(u)) {
                            agap = 0;  // already seen on another lane
                        }
                    } else {
                        agap = it.length - KV(u).pos + backOffset - vt(u).minGap;
                        if (!net.links[it.link].havePriority() && !egoFurtherHas(net.lanes[it.lane].edge) && egoOnRoad
                                && KV(u).speed < HALTING_SPEED) {
                            agap = std::max(agap, 0.0);  // a stopped vehicle on a minor road does not block
                        }
                    }
                    result.addFollower(u, ego, agap, 0, i);
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

}  // namespace ps
