// Pedestrian router, see pedrouter.hpp
#include "pedrouter.hpp"

#include <functional>
#include <limits>
#include <map>
#include <queue>

namespace ps {

int getSidewalk(const Network& net, int edge, SVCPermissions svc) {
    if (edge < 0) {
        return -1;
    }
    const std::vector<int>& lanes = net.edges[edge].lanes;
    for (int l : lanes) {
        if ((net.lanes[l].permissions & VC_ALL) == svc) {
            return l;
        }
    }
    for (int l : lanes) {
        if (net.lanes[l].allows(svc)) {
            return l;
        }
    }
    if (svc != VC_PEDESTRIAN) {
        // persons should always be able to use the sidewalk
        for (int l : lanes) {
            if ((net.lanes[l].permissions & VC_ALL) == VC_PEDESTRIAN) {
                return l;
            }
        }
        for (int l : lanes) {
            if (net.lanes[l].allows(VC_PEDESTRIAN)) {
                return l;
            }
        }
    }
    return -1;
}

// IntermodalNetwork(edges, pedestrianOnly = true)
void PedRouter::build(const Network& net) {
    const int nE = (int)net.edges.size();
    myNodes.clear();
    myBidi.assign(nE, {-1, -1});
    myDepart.assign(nE, -1);
    myArrival.assign(nE, -1);
    std::vector<std::vector<int>> viaOf;  // per node: via node of each successor (parallel to succ)
    auto addNode = [&](Kind k, int edge, bool fwd, double startPos, int lane) {
        Node n;
        n.kind = k;
        n.edge = edge;
        n.forward = fwd;
        n.startPos = startPos;
        n.length = edge >= 0 ? net.edges[edge].length : 0;
        n.lane = lane;
        // PedestrianEdge::myIsOpposite: backwards on a sidewalk that allows other traffic
        if (k == PED && !fwd && edge >= 0 && (net.edges[edge].isNormal() || net.edges[edge].internal)) {
            const int sidewalk = getSidewalk(net, edge);
            n.opposite = sidewalk >= 0 && net.lanes[sidewalk].permissions != VC_PEDESTRIAN;
        }
        myNodes.push_back(n);
        viaOf.emplace_back();
        return (int)myNodes.size() - 1;
    };
    bool haveSeenWalkingArea = false;
    for (int e = 0; e < nE; ++e) {
        const Edge& E = net.edges[e];
        const int lane = getSidewalk(net, e);
        if (lane >= 0) {
            if (E.isWalkingArea()) {
                const int n = addNode(PED, e, true, 0, lane);
                myBidi[e] = {n, n};
                myDepart[e] = n;
                myArrival[e] = n;
                haveSeenWalkingArea = true;
            } else {
                const int f = addNode(PED, e, true, 0, lane);
                const int b = addNode(PED, e, false, E.length, lane);
                myBidi[e] = {f, b};
            }
        }
        if (!E.isWalkingArea()) {
            myDepart[e] = addNode(DEPART_CONN, e, true, 0, -1);
            myArrival[e] = addNode(ARRIVAL_CONN, e, true, 0, -1);
        }
    }
    // walking connectors at nodes (networks without walking areas)
    std::map<int, int> walkConn;
    for (int e = 0; e < nE; ++e) {
        const Edge& E = net.edges[e];
        if (E.internal || haveSeenWalkingArea) {
            continue;
        }
        for (int node : {E.fromJ, E.toJ}) {
            if (walkConn.count(node) == 0) {
                walkConn[node] = addNode(WALK_CONN, -1, true, 0, -1);
            }
        }
    }
    auto addSucc = [&](int from, int to, int via) {
        myNodes[from].succ.push_back(to);
        viaOf[from].push_back(via);
    };
    for (int e = 0; e < nE; ++e) {
        const Edge& E = net.edges[e];
        const int sidewalk = getSidewalk(net, e);
        if (sidewalk < 0) {
            continue;
        }
        const std::pair<int, int> pair = myBidi[e];
        // MSLane::getOutgoingViaLanes
        std::vector<std::pair<int, int>> outgoing;
        for (int l : net.lanes[sidewalk].links) {
            const Link& k = net.links[l];
            outgoing.push_back({k.lane, k.via >= 0 ? net.lanes[k.via].edge : -1});
        }
        if (haveSeenWalkingArea) {
            bool hasWalkingArea = false;
            for (const auto& t : outgoing) {
                if (net.lanes[t.first].walkingArea) {
                    hasWalkingArea = true;
                    break;
                }
            }
            for (const auto& t : outgoing) {
                const int targetEdge = net.lanes[t.first].edge;
                const bool used = t.first == getSidewalk(net, targetEdge) && (!hasWalkingArea || net.edges[targetEdge].isWalkingArea());
                if (used) {
                    const std::pair<int, int> tp = myBidi[targetEdge];
                    addSucc(pair.first, tp.first, -1);
                    addSucc(tp.second, pair.second, -1);
                }
            }
        }
        auto connOf = [&](int node) {
            auto it = walkConn.find(node);
            return it == walkConn.end() ? -1 : it->second;
        };
        const int toNodeConn = E.toJ >= 0 ? connOf(E.toJ) : -1;
        if (toNodeConn >= 0) {
            double minViaLength = std::numeric_limits<double>::max();
            int minVia = -1;
            for (const auto& t : outgoing) {
                if (t.second >= 0 && net.edges[t.second].length < minViaLength) {
                    minViaLength = net.edges[t.second].length;
                    minVia = t.second;
                }
            }
            std::pair<int, int> interVia = {-1, -1};
            if (minVia >= 0 && myBidi[minVia].first >= 0) {
                interVia = myBidi[minVia];
            }
            if (!haveSeenWalkingArea) {
                addSucc(pair.first, toNodeConn, interVia.first);
            }
            addSucc(toNodeConn, pair.second, interVia.second);
        }
        const int fromNodeConn = E.fromJ >= 0 ? connOf(E.fromJ) : -1;
        if (fromNodeConn >= 0) {
            if (!haveSeenWalkingArea) {
                addSucc(pair.second, fromNodeConn, -1);
            }
            addSucc(fromNodeConn, pair.first, -1);
        }
        if (!E.isWalkingArea()) {
            const int dep = myDepart[e];
            const int arr = myArrival[e];
            addSucc(dep, pair.first, -1);
            addSucc(dep, pair.second, -1);
            addSucc(pair.first, arr, -1);
            addSucc(pair.second, arr, -1);
        }
    }
    myVia = std::move(viaOf);
}

// PedestrianEdge::prohibits and the prohibition of the edge walked before
bool PedRouter::prohibits(const Network& net, const Node& n, const Trip& trip) const {
    if (n.kind != PED) {
        return false;
    }
    if (n.edge == trip.prohibitedEdge) {
        return true;
    }
    if (trip.onlyNode < 0) {
        return false;
    }
    const Edge& E = net.edges[n.edge];
    return E.fromJ != trip.onlyNode && E.toJ != trip.onlyNode;
}

// PedestrianEdge::getTravelTime (connectors take no time)
double PedRouter::effort(const Network& net, const Node& n, const Trip& trip, double time) const {
    if (n.kind != PED) {
        return 0.;
    }
    // getPartialLength
    double length = n.length;
    if (n.edge == trip.from && !n.forward && trip.departPos < n.startPos) {
        length = trip.departPos - (n.startPos - n.length);
    }
    if (n.edge == trip.to && n.forward && trip.arrivalPos < n.startPos + n.length) {
        length = trip.arrivalPos - n.startPos;
    }
    if (n.edge == trip.from && n.forward && trip.departPos > n.startPos) {
        length -= (trip.departPos - n.startPos);
    }
    if (n.edge == trip.to && !n.forward && trip.arrivalPos > n.startPos - n.length) {
        length -= (trip.arrivalPos - (n.startPos - n.length));
    }
    length = std::max(length, NUMERICAL_EPS);
    double tlsDelay = 0;
    if (net.edges[n.edge].isCrossing()) {
        const double timeToCrossing = time;  // the trip departs at time 0
        const Lane& L = net.lanes[n.lane];
        // MSLane::getIncomingLinkState
        const char state = L.logicalPred < 0 ? char{LS_DEADEND} : net.linkTo(L.logicalPred, n.lane)->state;
        if (state == LS_RED) {
            tlsDelay += std::max(0., 20. - timeToCrossing);  // TL_RED_PENALTY
        } else if (state == LS_GREEN_MAJOR && timeToCrossing > 20. / 2) {
            tlsDelay += 20.;
        }
    }
    double tt = length / (trip.speed * (n.opposite ? walkOppositeFactor : 1.)) + tlsDelay;
    if (trip.randomFactor > 1) {
        tt *= 1. + rand01(trip.randomSeed, trip.randomKey, (uint64_t)(&n - myNodes.data()), RP_PERSON_ROUTE) * (trip.randomFactor - 1.);
    }
    return trip.edgeFactor != nullptr ? tt * (*trip.edgeFactor)[n.edge] : tt;  // IntermodalEdge::getRoutingFactor
}

std::vector<int> PedRouter::compute(const Network& net, const Trip& trip) const {
    std::vector<int> into;
    const int start = trip.from >= 0 ? myDepart[trip.from] : -1;
    const int target = trip.to >= 0 ? myArrival[trip.to] : -1;
    if (start < 0 || target < 0 || prohibits(net, myNodes[start], trip) || prohibits(net, myNodes[target], trip)) {
        return into;
    }
    const int n = (int)myNodes.size();
    thread_local std::vector<double> eff, leave;
    thread_local std::vector<int> prev;
    thread_local std::vector<char> visited;
    eff.assign(n, std::numeric_limits<double>::max());
    leave.assign(n, 0.);
    prev.assign(n, -1);
    visited.assign(n, 0);
    // min-heap by (effort, numerical id): DijkstraRouter::EdgeInfoByEffortComparator
    using Entry = std::pair<double, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> frontier;
    eff[start] = 0;
    leave[start] = 0;
    frontier.push({0., start});
    while (!frontier.empty()) {
        const Entry top = frontier.top();
        frontier.pop();
        const int cur = top.second;
        if (visited[cur] || top.first != eff[cur]) {
            continue;
        }
        visited[cur] = 1;
        if (cur == target) {
            std::vector<int> path;
            for (int x = cur; x >= 0; x = prev[x]) {
                path.push_back(x);
            }
            for (auto it = path.rbegin(); it != path.rend(); ++it) {
                if (myNodes[*it].kind == PED) {
                    into.push_back(myNodes[*it].edge);
                }
            }
            return into;
        }
        const Node& N = myNodes[cur];
        const double effortDelta = effort(net, N, trip, leave[cur]);
        const double leaveTime = leave[cur] + effortDelta;
        for (size_t i = 0; i < N.succ.size(); ++i) {
            const int f = N.succ[i];
            if (prohibits(net, myNodes[f], trip)) {
                continue;
            }
            double e = eff[cur] + effortDelta;
            double time = leaveTime;
            // updateViaEdgeCost: internal edges walked between the two
            for (int via = myVia[cur][i]; via >= 0 && net.edges[myNodes[via].edge].internal;) {
                const double d = effort(net, myNodes[via], trip, time);
                time += d;
                e += d;
                via = myVia[via].empty() ? -1 : myVia[via].front();
            }
            if (!visited[f] && e < eff[f]) {
                eff[f] = e;
                leave[f] = time;
                prev[f] = cur;
                frontier.push({e, f});
            }
        }
    }
    return into;
}

}  // namespace ps
