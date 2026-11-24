// Lane queries (MSLane) and best lanes (MSVehicle::updateBestLanes).
#include "model.hpp"

namespace ps {

// ---------------------------------------------------------------------------
// positions
// ---------------------------------------------------------------------------
// MSVehicle::getBackPositionOnLane (no shadow/target lanes, no bidi)
double Sim::backPosOf(const Kin& k, int lane) const {
    const double len = types[k.type].length;
    if (lane == k.lane) {
        return k.pos - len;
    }
    const Lane& L = net.lanes[lane];
    const Lane& M = net.lanes[k.lane];
    if (L.edge != M.edge) {
        if (k.nFurther > 0 && lane == k.further[k.nFurther - 1]) {
            return k.backPos;
        }
        if (k.nShadowFurther > 0 && lane == k.shadowFurther[k.nShadowFurther - 1] && k.nFurther > 0) {
            const double furtherLength = net.lanes[k.further[k.nFurther - 1]].length;
            return L.length == furtherLength ? k.backPos : k.backPos / furtherLength * L.length;
        }
        double leftLength = len - k.pos;
        for (int i = 0; i < k.nFurther && leftLength > 0; ++i) {
            leftLength -= net.lanes[k.further[i]].length;
            if (k.further[i] == lane) {
                return -leftLength;
            }
        }
        leftLength = len - k.pos;
        for (int i = 0; i < k.nShadowFurther && leftLength > 0; ++i) {
            leftLength -= net.lanes[k.shadowFurther[i]].length;
            if (k.shadowFurther[i] == lane) {
                return -leftLength;
            }
        }
        // not a further lane (callers ensure this does not happen; SUMO warns here)
        return L.length - k.pos + len;
    }
    // parallel lane of the same edge
    return k.pos - len + std::min(0.0, L.length - M.length);
}

// ---------------------------------------------------------------------------
// lane queries
// ---------------------------------------------------------------------------
int Sim::succLinkSec(int v, int nRouteSuccs, int fromLane, const std::vector<int>& conts) const {
    const int nRouteEdge = routeEdge(v, nRouteSuccs);
    if (nRouteEdge < 0) {
        return -1;
    }
    const Lane& src = net.lanes[fromLane];
    if (src.internal) {
        return src.links[0];
    }
    if (nRouteSuccs < (int)conts.size()) {
        const int vc = veh[v].vclass;
        for (int l : src.links) {
            const Link& k = net.links[l];
            if (net.lanes[k.lane].edge == nRouteEdge && net.lanes[k.lane].allows(vc) && (k.via < 0 || net.lanes[k.via].allows(vc))) {
                if (k.lane == conts[nRouteSuccs]) {
                    return l;
                }
            }
        }
    }
    return -1;
}

int Sim::lastAnyVehicle(int lane) const {
    const LaneRT& l = lanes[lane];
    if (!l.vehs.empty()) {
        return l.vehs.front();
    }
    if (!l.partials.empty()) {
        return l.partials.front();
    }
    return -1;
}

int Sim::firstAnyVehicle(int lane) const {
    const LaneRT& l = lanes[lane];
    int result = l.vehs.empty() ? -1 : l.vehs.back();
    if (!l.partials.empty() && (result < 0 || positionOnLane(result, lane) < positionOnLane(l.partials.back(), lane))) {
        result = l.partials.back();
    }
    return result;
}

// MSLane::getLastVehicleInformation(ego, 0, minPos) without sublanes: the
// rearmost vehicle (full or partial) at or ahead of minPos
int Sim::lastVehicleFrom(int lane, double minPos, int ego) const {
    int found = -1;
    anyVehicles(lane, [&](int v) {
        if (v != ego && std::max(0.0, positionOnLane(v, lane)) >= minPos) {
            found = v;
            return false;
        }
        return true;
    });
    return found;
}

double Sim::spaceTillLastStanding(int lane, int ego, bool& foundStopped) const {
    double lengths = 0;
    for (int last : lanes[lane].vehs) {
        if (kin[last].speed < HALTING_SPEED && !net.edges[net.lanes[kin[last].lane].edge].roundabout && last != ego
                && isFrontOnLane(last, lane)) {
            foundStopped = true;
            const double lastBrakeGap = cfBrakeGap(last, kin[last].speed);
            return backPositionOnLane(last, kin[last].lane) + lastBrakeGap - lengths;
        }
        lengths += vt(last).lengthWithGap();
    }
    return net.lanes[lane].length - lengths;
}

bool Sim::appropriate(int lane, int v) const {
    if (isInternal(lane)) {
        return true;
    }
    if (routeEdge(v, 1) < 0) {
        return bestLaneOffset(v) == 0;
    }
    return succLinkSec(v, 1, lane, bestContinuation(v)) >= 0;
}

// ---------------------------------------------------------------------------
// best lanes
// ---------------------------------------------------------------------------
int Sim::nextLinkPriority(const std::vector<int>& conts) const {
    if (conts.size() < 2 || conts[0] < 0 || conts[1] < 0) {
        return -1;
    }
    const Link* l = net.linkTo(conts[0], conts[1]);
    if (l == nullptr) {
        return -1;
    }
    return l->havePriority() ? 1 : 0;
}

const std::vector<int>& Sim::bestContinuation(int v) const {
    static const std::vector<int> empty;
    const Veh& x = veh[v];
    if (x.bestLanes.empty() || x.bestLanes[0].empty()) {
        return empty;
    }
    return x.bestLanes[0][x.currentLaneInBestLanes].bestContinuations;
}

const std::vector<int>& Sim::bestContinuationOf(int v, int lane) const {
    static const std::vector<int> empty;
    if (isInternal(lane)) {
        lane = net.links[net.lanes[lane].links[0]].lane;
    }
    const Veh& x = veh[v];
    if (x.bestLanes.empty()) {
        return empty;
    }
    for (const LaneQ& q : x.bestLanes[0]) {
        if (q.lane == lane) {
            return q.bestContinuations;
        }
    }
    return empty;
}

int Sim::bestLaneOffset(int v) const {
    const Veh& x = veh[v];
    if (x.bestLanes.empty() || x.bestLanes[0].empty()) {
        return 0;
    }
    return x.bestLanes[0][x.currentLaneInBestLanes].bestLaneOffset;
}

double Sim::bestLaneDist(int v) const {
    const Veh& x = veh[v];
    if (x.bestLanes.empty() || x.bestLanes[0].empty()) {
        return -1;
    }
    return x.bestLanes[0][x.currentLaneInBestLanes].length;
}

void Sim::updateOccupancyAndCurrentBestLane(int v, int startLane, bool occupancy) {
    Veh& x = veh[v];
    std::vector<LaneQ>& cur = x.bestLanes.front();
    if (!occupancy) {
        for (int i = 0; i < (int)cur.size(); ++i) {
            if (cur[i].lane == startLane) {
                x.currentLaneInBestLanes = i;
            }
        }
        return;
    }
    for (int i = 0; i < (int)cur.size(); ++i) {
        double nextOccupation = 0;
        const std::vector<int>& c = cur[i].bestContinuations;
        for (size_t j = 1; j < c.size(); ++j) {
            if (c[j] >= 0) {
                nextOccupation += bruttoSnap[c[j]];
            }
        }
        cur[i].nextOccupation = nextOccupation;
        if (cur[i].lane == startLane) {
            x.currentLaneInBestLanes = i;
        }
    }
}

// MSVehicle::betterContinuation (no bidi lanes, lcContRight = 1)
static bool betterContinuation(const LaneQ* bestConnectedNext, const LaneQ& m) {
    if (bestConnectedNext == nullptr) {
        return true;
    }
    if (bestConnectedNext->length < m.length) {
        return true;
    }
    if (bestConnectedNext->length == m.length) {
        if (std::abs(bestConnectedNext->bestLaneOffset) > std::abs(m.bestLaneOffset)) {
            return true;
        }
    }
    return false;
}

void Sim::updateBestLanes(int v, bool forceRebuild, int startLane, bool occupancy) {
    Veh& x = veh[v];
    if (startLane < 0) {
        startLane = kin[v].lane;
    }
    if (forceRebuild) {
        x.lastBestLanesEdge = -1;
        x.lastBestLanesInternalLane = -1;
    }
    const int startEdge = net.lanes[startLane].edge;
    if (!x.bestLanes.empty() && !forceRebuild && x.lastBestLanesEdge == startEdge) {
        updateOccupancyAndCurrentBestLane(v, startLane, occupancy);
        return;
    }
    if (isInternal(startLane)) {
        if (x.bestLanes.empty() || forceRebuild) {
            updateBestLanes(v, true, net.lanes[startLane].logicalPred);
        }
        if (x.lastBestLanesInternalLane == startLane && !forceRebuild) {
            return;
        }
        // adapt best lanes to fit the current internal edge
        const int nextEdge = net.lanes[net.normalSuccessorLane(startLane)].edge;
        for (auto it = x.bestLanes.begin(); it != x.bestLanes.end();) {
            std::vector<LaneQ>& ql = *it;
            if (net.lanes[ql[0].lane].edge == nextEdge) {
                std::vector<LaneQ> oldLanes = ql;
                ql.clear();
                for (int src : net.edges[startEdge].lanes) {
                    for (LaneQ& q : oldLanes) {
                        if (net.links[net.lanes[src].links[0]].lane == q.lane) {
                            ql.push_back(q);
                            break;
                        }
                    }
                }
                for (int i = 0; i < (int)ql.size(); ++i) {
                    if (i + ql[i].bestLaneOffset < 0) {
                        ql[i].bestLaneOffset = -i;
                    }
                    if (i + ql[i].bestLaneOffset >= (int)ql.size()) {
                        ql[i].bestLaneOffset = (int)ql.size() - i - 1;
                    }
                    if (ql[i].bestContinuations[0] != -1) {
                        ql[i].bestContinuations.insert(ql[i].bestContinuations.begin(), -1);
                    }
                    if (net.links[net.lanes[startLane].links[0]].lane == ql[i].lane) {
                        x.currentLaneInBestLanes = i;
                    }
                }
                x.lastBestLanesInternalLane = startLane;
                updateOccupancyAndCurrentBestLane(v, startLane, occupancy);
                return;
            }
            it = x.bestLanes.erase(it);
        }
        return;  // should not happen
    }
    // rebuild
    x.lastBestLanesInternalLane = -1;
    x.lastBestLanesEdge = startEdge;
    x.bestLanes.clear();
    const std::vector<int>& route = routes[x.route];
    const VType& t = vt(v);
    const int vc = x.vclass;
    // next stop
    int nextStopRouteIdx = -1;
    int nextStopLane = -1;
    double nextStopPos = 0;
    if (!x.stops.empty()) {
        const Stop& s = x.stops.front();
        nextStopLane = s.lane;
        nextStopRouteIdx = s.routeIdx;
        nextStopPos = s.startPos;
    } else if (x.arrivalLaneMode != AL_CURRENT && x.arrivalLane >= 0) {
        // a given arrival lane acts like a stop at the arrival position
        nextStopRouteIdx = (int)route.size() - 1;
        nextStopLane = net.edges[route.back()].lanes[x.arrivalLane];
        nextStopPos = x.arrivalPos;
    }
    if (nextStopLane >= 0) {
        nextStopPos = std::max(POSITION_EPS, std::min(nextStopPos, net.lanes[nextStopLane].length - 2 * POSITION_EPS));
    }
    int seen = 0;
    double seenLength = 0;
    bool progress = true;
    const double maxSpeed = vehMaxSpeed(v);
    const double maxBrakeDist = net.lanes[startLane].length + t.tau * maxSpeed + cfBrakeGap(v, maxSpeed) + t.minGap;
    const double lookahead = t.lcStrategicLookahead;
    for (int ce = x.routeIdx; progress;) {
        std::vector<LaneQ> currentLanes;
        const std::vector<int>* allowed = nullptr;
        bool haveNext = ce + 1 < (int)route.size();
        if (haveNext) {
            allowed = net.allowedLanes(route[ce], route[ce + 1], vc);
        }
        const Edge& e = net.edges[route[ce]];
        for (int cl : e.lanes) {
            LaneQ q;
            q.lane = cl;
            q.bestContinuations.push_back(cl);
            q.bestLaneOffset = 0;
            q.length = net.lanes[cl].allows(vc) ? e.length : 0;
            q.currentLength = q.length;
            // MSEdge::allowedLanes returns nullptr for unreachable successors, which allows every lane
            q.allowsContinuation = allowed == nullptr || std::find(allowed->begin(), allowed->end(), cl) != allowed->end();
            currentLanes.push_back(q);
        }
        if (nextStopRouteIdx == ce && !(ce == x.routeIdx && kin[v].lane >= 0 && isInternal(kin[v].lane))) {
            const int normalStopLane = net.normalPredecessorLane(nextStopLane);
            for (LaneQ& q : currentLanes) {
                if (normalStopLane != q.lane) {
                    q.allowsContinuation = false;
                    q.length = nextStopPos;
                    q.currentLength = q.length;
                }
            }
        }
        x.bestLanes.push_back(currentLanes);
        ++seen;
        seenLength += net.lanes[currentLanes[0].lane].length;
        ++ce;
        if (lookahead >= 0) {
            progress &= (seen <= 2 || seenLength < lookahead);
        } else {
            progress &= (seen <= 4 || seenLength < std::max(maxBrakeDist, 3000.0));
            progress &= (seen <= 8 || seenLength < std::max(maxBrakeDist, 200.0));
        }
        progress &= ce != (int)route.size();
    }
    auto changeRightAllowed = [&](int lane) { return (net.lanes[lane].changeRight & vc) == vc; };
    auto changeLeftAllowed = [&](int lane) { return (net.lanes[lane].changeLeft & vc) == vc; };
    // last edge
    {
        double bestLength = -1;
        int bestThisIndex = 0, bestThisMaxIndex = 0;
        std::vector<LaneQ>& last = x.bestLanes.back();
        for (int index = 0; index < (int)last.size(); ++index) {
            if (last[index].length > bestLength) {
                bestLength = last[index].length;
                bestThisIndex = index;
                bestThisMaxIndex = index;
            } else if (last[index].length == bestLength) {
                bestThisMaxIndex = index;
            }
        }
        bool requiredChangeRightForbidden = false;
        int requireChangeToLeftForbidden = -1;
        for (int index = 0; index < (int)last.size(); ++index) {
            LaneQ& j = last[index];
            if (j.length < bestLength) {
                if (std::abs(bestThisIndex - index) < std::abs(bestThisMaxIndex - index)) {
                    j.bestLaneOffset = bestThisIndex - index;
                } else {
                    j.bestLaneOffset = bestThisMaxIndex - index;
                }
                if (!j.allowsContinuation) {
                    const int right = net.parallel(j.lane, -1), left = net.parallel(j.lane, 1);
                    if (j.bestLaneOffset < 0 && (!changeRightAllowed(j.lane) || right < 0 || !net.lanes[right].allows(vc) || requiredChangeRightForbidden)) {
                        requiredChangeRightForbidden = true;
                        j.length = 0;
                    } else if (j.bestLaneOffset > 0 && (!changeLeftAllowed(j.lane) || left < 0 || !net.lanes[left].allows(vc))) {
                        requireChangeToLeftForbidden = net.lanes[j.lane].index;
                    }
                }
            }
        }
        for (int i = requireChangeToLeftForbidden; i >= 0; i--) {
            if (last[i].bestLaneOffset > 0) {
                last[i].length = 0;
            }
        }
    }
    // backwards
    for (int i = (int)x.bestLanes.size() - 2; i >= 0; --i) {
        std::vector<LaneQ>& nextLanes = x.bestLanes[i + 1];
        std::vector<LaneQ>& clanes = x.bestLanes[i];
        const int cE = net.lanes[clanes[0].lane].edge;
        double bestConnectedLength = -1;
        double bestLength = -1;
        for (const LaneQ& j : nextLanes) {
            if (net.approachedFromEdge(j.lane, cE) && bestConnectedLength < j.length) {
                bestConnectedLength = j.length;
            }
            if (bestLength < j.length) {
                bestLength = j.length;
            }
        }
        int bestThisIndex = 0, bestThisMaxIndex = 0;
        if (bestConnectedLength > 0) {
            int index = 0;
            for (LaneQ& j : clanes) {
                const LaneQ* bestConnectedNext = nullptr;
                if (j.allowsContinuation) {
                    for (const LaneQ& m : nextLanes) {
                        if (net.lanes[m.lane].allows(vc) && net.approachedFromLane(m.lane, j.lane, vc)) {
                            if (betterContinuation(bestConnectedNext, m)) {
                                bestConnectedNext = &m;
                            }
                        }
                    }
                    if (bestConnectedNext != nullptr) {
                        if (bestConnectedNext->length == bestConnectedLength && std::abs(bestConnectedNext->bestLaneOffset) < 2) {
                            j.length += bestLength;
                        } else {
                            j.length += bestConnectedNext->length;
                        }
                        j.bestLaneOffset = bestConnectedNext->bestLaneOffset;
                    }
                }
                if (bestConnectedNext != nullptr && (bestConnectedNext->allowsContinuation || bestConnectedNext->length > 0)) {
                    j.bestContinuations.insert(j.bestContinuations.end(), bestConnectedNext->bestContinuations.begin(), bestConnectedNext->bestContinuations.end());
                } else {
                    j.allowsContinuation = false;
                }
                const LaneQ& bt = clanes[bestThisIndex];
                if (bt.length < j.length
                        || (bt.length == j.length && std::abs(bt.bestLaneOffset) > std::abs(j.bestLaneOffset))
                        || (bt.length == j.length && std::abs(bt.bestLaneOffset) == std::abs(j.bestLaneOffset)
                            && nextLinkPriority(bt.bestContinuations) < nextLinkPriority(j.bestContinuations))) {
                    bestThisIndex = index;
                    bestThisMaxIndex = index;
                } else if (bt.length == j.length && std::abs(bt.bestLaneOffset) == std::abs(j.bestLaneOffset)
                           && nextLinkPriority(bt.bestContinuations) == nextLinkPriority(j.bestContinuations)) {
                    bestThisMaxIndex = index;
                }
                index++;
            }
        } else {
            int bestNextIndex = 0;
            int bestDistToNeeded = (int)clanes.size();
            for (int index = 0; index < (int)clanes.size(); ++index) {
                if (clanes[index].allowsContinuation) {
                    for (int nextIndex = 0; nextIndex < (int)nextLanes.size(); ++nextIndex) {
                        if (net.approachedFromLane(nextLanes[nextIndex].lane, clanes[index].lane, vc)) {
                            if (bestDistToNeeded > std::abs(nextLanes[nextIndex].bestLaneOffset)) {
                                bestDistToNeeded = std::abs(nextLanes[nextIndex].bestLaneOffset);
                                bestThisIndex = index;
                                bestThisMaxIndex = index;
                                bestNextIndex = nextIndex;
                            }
                        }
                    }
                }
            }
            clanes[bestThisIndex].length += nextLanes[bestNextIndex].length;
            clanes[bestThisIndex].bestContinuations.insert(clanes[bestThisIndex].bestContinuations.end(),
                    nextLanes[bestNextIndex].bestContinuations.begin(), nextLanes[bestNextIndex].bestContinuations.end());
        }
        const LaneQ bt = clanes[bestThisIndex];
        bool requiredChangeRightForbidden = false;
        int requireChangeToLeftForbidden = -1;
        for (int index = 0; index < (int)clanes.size(); ++index) {
            LaneQ& j = clanes[index];
            if (j.length < bt.length
                    || (j.length == bt.length && std::abs(j.bestLaneOffset) > std::abs(bt.bestLaneOffset))
                    || nextLinkPriority(j.bestContinuations) < nextLinkPriority(bt.bestContinuations)) {
                if (std::abs(bestThisIndex - index) < std::abs(bestThisMaxIndex - index)) {
                    j.bestLaneOffset = bestThisIndex - index;
                } else {
                    j.bestLaneOffset = bestThisMaxIndex - index;
                }
                if (nextLinkPriority(j.bestContinuations) < nextLinkPriority(bt.bestContinuations)) {
                    j.length = j.currentLength;
                }
                if (!j.allowsContinuation) {
                    const int right = net.parallel(j.lane, -1), left = net.parallel(j.lane, 1);
                    if (j.bestLaneOffset < 0 && (!changeRightAllowed(j.lane) || right < 0 || !net.lanes[right].allows(vc) || requiredChangeRightForbidden)) {
                        requiredChangeRightForbidden = true;
                        if (j.length == j.currentLength) {
                            j.length = 0;
                        }
                    } else if (j.bestLaneOffset > 0 && (!changeLeftAllowed(j.lane) || left < 0 || !net.lanes[left].allows(vc))) {
                        requireChangeToLeftForbidden = net.lanes[j.lane].index;
                    }
                }
            } else {
                j.bestLaneOffset = 0;
            }
        }
        for (int idx = requireChangeToLeftForbidden; idx >= 0; idx--) {
            if (clanes[idx].length == clanes[idx].currentLength) {
                clanes[idx].length = 0;
            }
        }
    }
    updateOccupancyAndCurrentBestLane(v, startLane, occupancy);
}

}  // namespace ps

namespace ps {

// MSLane::getFollowersOnConsecutive(ego, backOffset, false) outside of lane
// changing (insertion, leaving a parking stop). `ego` is not on the network.
Sim::CritFollower Sim::followersOnConsecutive(int lane, int ego, double backOffset, double egoSpeed, double egoDecel) const {
    CritFollower result;
    auto required = [&](int u) { return cfSecureGap(u, ego, kin[u].speed, egoSpeed, egoDecel); };
    const double egoPos = backOffset + vt(ego).length;
    anyVehicles(lane, [&](int u) {
        if (u != ego && positionOnLane(u, lane) < egoPos) {
            result.add(u, backOffset - positionOnLane(u, lane) - vt(u).minGap, required(u));
        }
        return true;
    });
    forBidiOccupants(lane, [&](int u, double pos) {
        if (u != ego && pos < egoPos) {
            result.add(u, backOffset - pos - vt(u).minGap - vt(u).length, required(u));  // (isBidiOn)
        }
    });
    if (result.veh >= 0) {
        return result;
    }
    double searchDist = net.lanes[lane].maxBrakeDist - backOffset;
    // ego is being inserted: its further lanes are not set yet
    int egoFurtherEdge = -1;
    if (kin[ego].pos < vt(ego).length && net.lanes[lane].logicalPred >= 0) {
        egoFurtherEdge = net.lanes[net.lanes[lane].logicalPred].edge;
    }
    std::vector<int> visited = net.edges[net.lanes[lane].edge].lanes;
    const int bidiEdge = net.edges[net.lanes[lane].edge].bidi;
    if (bidiEdge >= 0) {
        visited.insert(visited.end(), net.edges[bidiEdge].lanes.begin(), net.edges[bidiEdge].lanes.end());
    }
    struct Inc {
        int lane, link;
        double length;
    };
    std::vector<Inc> toExamine, newFound;
    for (const IncomingLane& i : net.lanes[lane].incoming) {
        toExamine.push_back({i.lane, i.link, i.length});
    }
    thread_local std::vector<LinkLeader> leaders;
    const int egoEdge = net.lanes[kin[ego].lane].edge;
    while (!toExamine.empty()) {
        for (const Inc& it : toExamine) {
            const int next = it.lane;
            searchDist = std::min(searchDist, net.lanes[next].maxBrakeDist - backOffset);
            if (backOffset + it.length - net.lanes[next].length < 0 && net.lanes[next].edge == egoFurtherEdge) {
                leaderInfo(it.link, ego, -backOffset, leaders, false);
                for (const LinkLeader& ll : leaders) {
                    if (ll.veh < 0) {
                        continue;  // pedestrian
                    }
                    const bool egoIsLeader = isLeader(ll.veh, it.link, ego, ll.gap);
                    const double gap = egoIsLeader ? -ll.gap - vt(ll.veh).lengthWithGap() - vt(ego).minGap : ll.gap + vt(ego).length;
                    result.add(ll.veh, gap, required(ll.veh));
                }
            }
            // first vehicle (any / front on lane)
            const int firstAny = firstAnyVehicle(next);
            const int firstFront = lanes[next].vehs.empty() ? -1 : lanes[next].vehs.back();
            int v = firstAny == ego ? firstFront : firstAny;
            // (the vehicles on the bidirectional lane are partial occupants of `next` as well)
            int bidiFirst = -1;
            double bidiFirstPos = 0;
            forBidiOccupants(next, [&](int u, double pos) {
                if (bidiFirst < 0 || pos > bidiFirstPos) {
                    bidiFirst = u;
                    bidiFirstPos = pos;
                }
            });
            if (bidiFirst >= 0 && bidiFirst != ego && (v < 0 || bidiFirstPos > positionOnLane(v, next))) {
                v = bidiFirst;
            }
            if (v >= 0 && v != ego && v == bidiFirst) {
                result.add(v, kin[v].pos + backOffset - vt(v).lengthWithGap(), required(v));
            } else if (v >= 0 && v != ego) {
                double agap = 0;
                if (!isFrontOnLane(v, next)) {
                    agap = it.length - net.lanes[next].length + backOffset;
                    if (net.usingInternalLanes) {
                        agap -= vt(v).minGap;  // (ego should have left the intersection still occupied by v)
                    }
                    const bool differentEdge = net.lanes[kin[v].lane].edge != egoEdge;
                    if (agap > 0 && differentEdge) {
                        v = firstFront;
                        if (v >= 0 && v != ego) {
                            agap = it.length - kin[v].pos + backOffset - vt(v).minGap;
                        } else {
                            v = -1;
                        }
                    } else if (differentEdge && result.veh == v) {
                        agap = 0;
                    }
                } else {
                    agap = it.length - kin[v].pos + backOffset - vt(v).minGap;
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

// MSLane::getMissingRearGap
double Sim::missingRearGap(int lane, int leader, double backOffset, double leaderSpeed) const {
    // (the critical follower is selected with the leader's current speed: 0 before its departure)
    const double currentSpeed = veh[leader].status == VehStatus::PENDING ? 0.0 : kin[leader].speed;
    const CritFollower f = followersOnConsecutive(lane, leader, backOffset, currentSpeed, vt(leader).decel);
    if (leader == traceVeh) {
        std::fprintf(stderr, "%.2f missingRearGap %s lane=%s follower=%s dist=%.2f\n", STEPS2TIME(now), veh[leader].id.c_str(),
                     net.lanes[lane].id.c_str(), f.veh >= 0 ? veh[f.veh].id.c_str() : "-", f.dist);
    }
    if (f.veh >= 0) {
        return cfSecureGap(f.veh, leader, kin[f.veh].speed, leaderSpeed, vt(leader).decel) - f.dist;
    }
    return 0;
}

}  // namespace ps
