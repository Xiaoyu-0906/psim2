// Intermodal router, see intermodal.hpp (ported from SUMO 1.28 utils/router: IntermodalNetwork.h,
// IntermodalEdge.h, PedestrianEdge.h, CarEdge.h, PublicTransportEdge.h, StopEdge.h, AccessEdge.h,
// IntermodalRouter.h, DijkstraRouter.h)
#include "intermodal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

#include "pedrouter.hpp"

namespace ps {

namespace {
constexpr SVCPermissions TAXI_CLASSES = VC_TAXI;  // gTaxiClasses
constexpr double TL_RED_PENALTY = 20;
}  // namespace

void IntermodalNet::reset() {
    std::vector<IEdge>().swap(myEdges);
    std::vector<std::multimap<Time, Schedule>>().swap(mySchedules);
    std::vector<std::string>().swap(myNames);
    std::vector<std::pair<int, int>>().swap(myBidi);
    std::vector<std::vector<int>>().swap(myDepart);
    std::vector<std::vector<int>>().swap(myArrival);
    std::vector<int>().swap(myCar);
    myWalkingConnector.clear();
    myStops.clear();
    myAccessSplits.clear();
    myPTLines.clear();
    myLoopedLines.clear();
}

size_t IntermodalNet::memoryBytes() const {
    size_t b = myEdges.capacity() * sizeof(IEdge);
    for (const IEdge& e : myEdges) {
        b += e.succ.capacity() * sizeof(std::pair<int, int>);
    }
    for (const auto& d : myDepart) {
        b += d.capacity() * sizeof(int);
    }
    for (const auto& d : myArrival) {
        b += d.capacity() * sizeof(int);
    }
    return b + myBidi.capacity() * sizeof(std::pair<int, int>) + myCar.capacity() * sizeof(int);
}

int IntermodalNet::nameIndex(const std::string& name) {
    auto it = std::find(myNames.begin(), myNames.end(), name);
    if (it != myNames.end()) {
        return (int)(it - myNames.begin());
    }
    myNames.push_back(name);
    return (int)myNames.size() - 1;
}

int IntermodalNet::addEdge(const IEdge& e) {
    myEdges.push_back(e);
    return (int)myEdges.size() - 1;
}

// IntermodalEdge::removeSuccessor
bool IntermodalNet::removeSucc(int from, int to) {
    std::vector<std::pair<int, int>>& s = myEdges[from].succ;
    const size_t before = s.size();
    s.erase(std::remove_if(s.begin(), s.end(), [to](const std::pair<int, int>& p) { return p.first == to; }), s.end());
    return s.size() != before;
}

int IntermodalNet::newConnector(int edge) {
    IEdge c;
    c.kind = CONNECTOR;
    c.edge = edge;
    return addEdge(c);
}

// PedestrianEdge
int IntermodalNet::newPed(const Network& net, int edge, int lane, bool forward, double pos) {
    const Edge& E = net.edges[edge];
    IEdge p;
    p.kind = PED;
    p.edge = edge;
    p.lane = lane;
    p.forward = forward;
    p.length = E.length;
    p.startPos = pos >= 0 ? pos : (forward ? 0. : E.length);
    if (!forward && (E.isNormal() || E.internal)) {
        const int sidewalk = getSidewalk(net, edge);
        if (sidewalk >= 0 && net.lanes[sidewalk].permissions != VC_PEDESTRIAN) {
            p.opposite = true;
        }
    }
    return addEdge(p);
}

// CarEdge
int IntermodalNet::newCar(const Network& net, int edge, double pos) {
    IEdge c;
    c.kind = CAR;
    c.edge = edge;
    c.length = net.edges[edge].length;
    c.startPos = pos >= 0 ? pos : 0.;
    return addEdge(c);
}

// AccessEdge (from an intermodal edge to 'out')
int IntermodalNet::newAccess(int out, double length, SVCPermissions modeRestriction, SVCPermissions vehicleRestriction,
                             double traveltime) {
    IEdge a;
    a.kind = ACCESS;
    a.edge = myEdges[out].edge;
    a.length = length > 0. ? length : NUMERICAL_EPS;
    a.traveltime = traveltime;
    a.modeRestriction = modeRestriction;
    a.vehicleRestriction = vehicleRestriction;
    return addEdge(a);
}

// MSEdge::getFromJunction / getToJunction (internal edges, crossings and walking areas: the junction of the id)
int IntermodalNet::junctionOf(const Network& net, int edge, bool to) const {
    const Edge& E = net.edges[edge];
    const int j = to ? E.toJ : E.fromJ;
    if (j >= 0 || E.isNormal() || E.id.empty() || E.id[0] != ':') {
        return j;
    }
    const size_t sep = E.id.rfind('_');
    auto it = net.junctionIdx.find(E.id.substr(1, sep == std::string::npos ? std::string::npos : sep - 1));
    return it == net.junctionIdx.end() ? -1 : it->second;
}

// IntermodalNetwork(edges, pedestrianOnly = false, carWalkTransfer) (psim2's zones are no network edges)
void IntermodalNet::build(const Network& net, int carWalkTransfer,
                          const std::vector<std::pair<std::vector<int>, std::vector<int>>>& tazs) {
    reset();
    myCarWalkTransfer = carWalkTransfer;
    const int nE = (int)net.edges.size();
    myNumNetEdges = nE;
    const int nAll = nE + 2 * (int)tazs.size();
    myBidi.assign(nAll, {-1, -1});
    myDepart.assign(nAll, {});
    myArrival.assign(nAll, {});
    myCar.assign(nAll, -1);
    myEdges.reserve((size_t)nE * 5);
    bool haveSeenWalkingArea = false;
    for (int e = 0; e < nE; ++e) {
        const Edge& E = net.edges[e];
        const int lane = getSidewalk(net, e);
        if (lane >= 0) {
            if (E.isWalkingArea()) {
                const int n = newPed(net, e, lane, true);
                myBidi[e] = {n, n};
                myDepart[e].push_back(n);
                myArrival[e].push_back(n);
                haveSeenWalkingArea = true;
            } else {
                const int f = newPed(net, e, lane, true);
                const int b = newPed(net, e, lane, false);
                myBidi[e] = {f, b};
            }
        }
        if (!E.isWalkingArea()) {
            const int dep = newConnector(e);
            const int arr = newConnector(e);
            myDepart[e].insert(myDepart[e].begin(), dep);
            myArrival[e].insert(myArrival[e].begin(), arr);
        }
    }
    // the taz connectors: a single access edge each
    for (int c = nE; c < nAll; ++c) {
        IEdge a;
        a.kind = ACCESS;
        a.edge = c;
        a.length = NUMERICAL_EPS;
        const int ae = addEdge(a);
        myDepart[c].push_back(ae);
        myArrival[c].push_back(ae);
    }
    // the walking connectors (with walking areas only at the end of edges without sidewalks)
    for (int e = 0; e < nE; ++e) {
        const Edge& E = net.edges[e];
        if (E.internal) {
            continue;
        }
        if (haveSeenWalkingArea) {
            const int node = junctionOf(net, e, true);
            if (getSidewalk(net, e) < 0 && myWalkingConnector.count(node) == 0) {
                myWalkingConnector[node] = newConnector(-1);
            }
        } else {
            for (int node : {junctionOf(net, e, false), junctionOf(net, e, true)}) {
                if (myWalkingConnector.count(node) == 0) {
                    myWalkingConnector[node] = newConnector(-1);
                }
            }
        }
    }
    // the connections
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
            // an outgoing walking area must be used, all other connections are ignored then
            bool hasWalkingArea = false;
            for (const auto& t : outgoing) {
                hasWalkingArea = hasWalkingArea || net.edges[net.lanes[t.first].edge].isWalkingArea();
            }
            for (const auto& t : outgoing) {
                const int targetEdge = net.lanes[t.first].edge;
                if (t.first == getSidewalk(net, targetEdge) && (!hasWalkingArea || net.edges[targetEdge].isWalkingArea())) {
                    const std::pair<int, int> tp = myBidi[targetEdge];
                    addSucc(pair.first, tp.first);
                    addSucc(tp.second, pair.second);
                }
            }
        }
        auto it = myWalkingConnector.find(junctionOf(net, e, true));
        if (it != myWalkingConnector.end()) {
            // the shortest outgoing via edge as an approximation
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
                addSucc(pair.first, it->second, interVia.first);
            }
            addSucc(it->second, pair.second, interVia.second);
        }
        auto it2 = myWalkingConnector.find(junctionOf(net, e, false));
        if (it2 != myWalkingConnector.end()) {
            if (!haveSeenWalkingArea) {
                addSucc(pair.second, it2->second);
            }
            addSucc(it2->second, pair.first);
        }
        if (!E.isWalkingArea()) {
            const int dep = myDepart[e][0];
            const int arr = myArrival[e][0];
            addSucc(dep, pair.first);
            addSucc(dep, pair.second);
            addSucc(pair.first, arr);
            addSucc(pair.second, arr);
        }
    }
    // persons may depart at sinks and arrive at sources as well
    for (int i = 0; i < (int)tazs.size(); ++i) {
        const int sink = myDepart[nE + 2 * i][0], source = myDepart[nE + 2 * i + 1][0];
        for (int in : tazs[i].second) {
            if (net.edges[in].isNormal()) {
                addSucc(myArrival[in][0], sink);
                addSucc(source, myDepart[in][0]);
            }
        }
        for (int out : tazs[i].first) {
            if (net.edges[out].isNormal()) {
                addSucc(source, myDepart[out][0]);
                addSucc(myArrival[out][0], sink);
            }
        }
    }
}

// IntermodalNetwork::getWalkingConnector: the constructor's lookups have registered the junctions of all edges
// with sidewalks (as null entries), so the walking area fallback is never reached
int IntermodalNet::walkingConnector(const Network& net, int e) const {
    auto it = myWalkingConnector.find(junctionOf(net, e, true));
    return it != myWalkingConnector.end() ? it->second : -1;
}

void IntermodalNet::addRestrictedCarExit(int from, int to, SVCPermissions vehicleRestriction) {
    const int access = newAccess(to, 0, 0, vehicleRestriction);
    addSucc(from, access);
    addSucc(access, to);
}

// IntermodalNetwork::addCarEdges (myCarLookup is iterated in edge order)
void IntermodalNet::addCarEdges(const Network& net, double taxiWait) {
    const int nE = (int)net.edges.size();
    for (int e = 0; e < nE; ++e) {
        if (net.edges[e].isNormal() || net.edges[e].internal) {
            myCar[e] = newCar(net, e);
        }
    }
    // MSJunction::getOutgoing / getIncoming: all edges of the junction graph, sorted by id (NLHandler)
    std::vector<int> byId(nE);
    for (int e = 0; e < nE; ++e) {
        byId[e] = e;
    }
    std::sort(byId.begin(), byId.end(), [&](int a, int b) { return net.edges[a].id < net.edges[b].id; });
    std::map<int, std::vector<int>> outOf, inOf;
    for (int e : byId) {
        outOf[junctionOf(net, e, false)].push_back(e);
        inOf[junctionOf(net, e, true)].push_back(e);
    }
    std::vector<std::pair<int, bool>> targets;
    for (int e = 0; e < nE; ++e) {
        const int carEdge = myCar[e];
        if (carEdge < 0) {
            continue;
        }
        // MSEdge::getViaSuccessors: the first link to every successor
        std::vector<std::pair<int, int>> vias;
        for (int lane : net.edges[e].lanes) {
            for (int l : net.lanes[lane].links) {
                const Link& k = net.links[l];
                const int to = net.lanes[k.lane].edge;
                bool known = false;
                for (const auto& v : vias) {
                    known = known || v.first == to;
                }
                if (!known) {
                    vias.push_back({to, k.via < 0 ? -1 : net.lanes[k.via].edge});
                }
            }
        }
        for (const auto& suc : vias) {
            const int sucCar = myCar[suc.first];
            if (sucCar >= 0) {
                addSucc(carEdge, sucCar, suc.second >= 0 ? myCar[suc.second] : -1);
            }
        }
        if (!net.edges[e].isNormal()) {
            continue;
        }
        // an edge where pedestrians are forbidden: continue on an arbitrary pedestrian edge
        const int toJ = junctionOf(net, e, true);
        targets.clear();
        for (int out : outOf[toJ]) {
            if (!net.edges[out].internal && getSidewalk(net, out) >= 0) {
                targets.push_back({out, true});
            }
        }
        for (int in : inOf[toJ]) {
            if (!net.edges[in].internal && getSidewalk(net, in) >= 0) {
                targets.push_back({in, false});
            }
        }
        if ((myCarWalkTransfer & IM_ALL_JUNCTIONS) != 0) {
            const int walkCon = walkingConnector(net, e);
            if (walkCon >= 0) {
                addSucc(carEdge, walkCon);
            } else {
                for (const auto& t : targets) {
                    addSucc(carEdge, t.second ? myBidi[t.first].first : myBidi[t.first].second);
                }
            }
        }
        if ((myCarWalkTransfer & IM_ALL_JUNCTIONS) == 0 && (myCarWalkTransfer & IM_TAXI_DROPOFF_ANYWHERE) != 0) {
            const int walkCon = walkingConnector(net, e);
            if (walkCon >= 0) {
                addRestrictedCarExit(carEdge, walkCon, TAXI_CLASSES);
            } else {
                for (const auto& t : targets) {
                    addRestrictedCarExit(carEdge, t.second ? myBidi[t.first].first : myBidi[t.first].second, TAXI_CLASSES);
                }
            }
        }
        // an intermediate access edge that prevents the taxi departure
        const int departConn = myDepart[e][0];
        const int access = newAccess(carEdge, 0, VC_ALL & ~VC_TAXI);
        addSucc(departConn, access);
        addSucc(access, carEdge);
        if ((myCarWalkTransfer & IM_TAXI_PICKUP_ANYWHERE) != 0) {
            // taxis may depart anywhere with a time penalty
            const int taxiAccess = newAccess(carEdge, 0, VC_TAXI, 0, taxiWait);
            addSucc(departConn, taxiAccess);
            addSucc(taxiAccess, carEdge);
        }
        if ((myCarWalkTransfer & IM_TAXI_DROPOFF_ANYWHERE) != 0) {
            addSucc(carEdge, myArrival[e][0]);
        } else {
            addRestrictedCarExit(carEdge, myArrival[e][0], VC_ALL & ~TAXI_CLASSES);
        }
    }
}

// IntermodalNetwork::addCarAccess
void IntermodalNet::addCarAccess(const Network& net, int edge, SVCPermissions svc, double traveltime) {
    const std::pair<int, int> ped = myBidi[edge];
    const int carEdge = myCar[edge];
    if (ped.first < 0 || carEdge < 0) {
        return;
    }
    const int access = newAccess(carEdge, 0, svc, 0, traveltime);
    addSucc(ped.first, access);
    addSucc(ped.second, access);
    addSucc(access, carEdge);
}

// IntermodalNetwork::findSplitIndex
int IntermodalNet::findSplitIndex(int toSplit, double pos, double& relPos, bool& needSplit) const {
    relPos = pos;
    needSplit = true;
    int splitIndex = 0;
    auto it = myAccessSplits.find(toSplit);
    if (it != myAccessSplits.end() && !it->second.empty()) {
        for (int split : it->second) {
            if (relPos < myEdges[split].length + POSITION_EPS) {
                break;
            }
            relPos -= myEdges[split].length;
            splitIndex++;
        }
        if (splitIndex + 1 < (int)it->second.size() && std::fabs(relPos - myEdges[it->second[splitIndex]].length) < POSITION_EPS) {
            needSplit = false;
        }
    }
    return splitIndex;
}

// IntermodalNetwork::splitEdge (afterSplit was created by the caller when a split is needed)
void IntermodalNet::splitEdge(int toSplit, int splitIndex, int afterSplit, double relPos, double length, bool needSplit,
                              int stopConn, bool forward, bool addExit, bool addEntry) {
    std::vector<int>& splitList = myAccessSplits[toSplit];
    if (splitList.empty()) {
        splitList.push_back(toSplit);
    }
    if (!forward) {
        splitIndex = (int)splitList.size() - 1 - splitIndex;
        if (!needSplit) {
            splitIndex--;
        }
    }
    const int beforeSplit = splitList[splitIndex];
    if (needSplit) {
        // IntermodalEdge::transferSuccessors (without the via edges)
        IEdge& b = myEdges[beforeSplit];
        IEdge& a = myEdges[afterSplit];
        for (const auto& s : b.succ) {
            a.succ.push_back({s.first, -1});
        }
        b.succ.clear();
        addSucc(beforeSplit, afterSplit);
        if (forward) {
            myEdges[afterSplit].length = std::max(0.0, myEdges[beforeSplit].length - relPos);
            myEdges[beforeSplit].length = relPos;
        } else {
            myEdges[afterSplit].length = relPos;
            myEdges[beforeSplit].length = std::max(0.0, myEdges[beforeSplit].length - relPos);
        }
        splitList.insert(splitList.begin() + splitIndex + 1, afterSplit);
    } else {
        afterSplit = splitList[splitIndex + 1];
    }
    if (addEntry) {
        const int access = newAccess(stopConn, length);
        addSucc(beforeSplit, access);
        addSucc(access, stopConn);
    }
    if (addExit) {
        // exit from public transport to walking
        const int exit = newAccess(afterSplit, length);
        addSucc(stopConn, exit);
        addSucc(exit, afterSplit);
    }
}

// IntermodalNetwork::addAccess
void IntermodalNet::addAccess(const Network& net, const std::string& stopId, int stopEdge, double startPos, double endPos,
                              double length, IMStopCategory category, bool isAccess, double taxiWait) {
    const bool parkingArea = category == IM_PARKING_AREA;
    const bool busStop = category == IM_BUS_STOP;
    const bool transferCarWalk = (parkingArea && (myCarWalkTransfer & IM_PARKING_AREAS) != 0)
                                 || (busStop && (myCarWalkTransfer & IM_PT_STOPS) != 0);
    const bool transferTaxiWalk = (parkingArea && (myCarWalkTransfer & IM_TAXI_DROPOFF_PARKING_AREAS) != 0)
                                  || (busStop && (myCarWalkTransfer & IM_TAXI_DROPOFF_PT) != 0);
    const bool transferWalkTaxi = (parkingArea && (myCarWalkTransfer & IM_TAXI_PICKUP_PARKING_AREAS) != 0)
                                  || (busStop && (myCarWalkTransfer & IM_TAXI_PICKUP_PT) != 0);
    const double pos = (startPos + endPos) / 2.;
    if (myStops.count(stopId) == 0) {
        IEdge s;
        s.kind = STOP;
        s.edge = stopEdge;
        s.ref = nameIndex(stopId);
        s.startPos = startPos;
        s.endPos = endPos;
        myStops[stopId] = addEdge(s);
    }
    const int stopConn = myStops[stopId];
    const int lane = getSidewalk(net, stopEdge);
    if (lane >= 0) {
        const std::pair<int, int> pair = myBidi[stopEdge];
        double relPos;
        bool needSplit;
        const int splitIndex = findSplitIndex(pair.first, pos, relPos, needSplit);
        const int fwdSplit = needSplit ? newPed(net, stopEdge, lane, true, pos) : -1;
        splitEdge(pair.first, splitIndex, fwdSplit, relPos, length, needSplit, stopConn);
        const int backSplit = needSplit ? newPed(net, stopEdge, lane, false, pos) : -1;
        splitEdge(pair.second, splitIndex, backSplit, relPos, length, needSplit, stopConn, false);
        int carSplit = -1;
        if (myCar[stopEdge] >= 0) {
            if (needSplit) {
                carSplit = newCar(net, stopEdge, pos);
            }
            splitEdge(myCar[stopEdge], splitIndex, carSplit, relPos, length, needSplit, stopConn, true, false, transferCarWalk);
        }
        if (needSplit) {
            if (carSplit >= 0 && (transferCarWalk || transferTaxiWalk)) {
                // access from car to walk
                const int beforeSplit = myAccessSplits[myCar[stopEdge]][splitIndex];
                for (int conn : {fwdSplit, backSplit}) {
                    if (transferCarWalk) {
                        const int access = newAccess(conn, length);
                        addSucc(beforeSplit, access);
                        addSucc(access, conn);
                    } else if (transferTaxiWalk) {
                        addRestrictedCarExit(beforeSplit, stopConn, TAXI_CLASSES);
                    }
                }
            }
            if (carSplit >= 0 && transferWalkTaxi && !isAccess) {
                const int access = newAccess(carSplit, 0, VC_TAXI, 0, taxiWait);
                addSucc(stopConn, access);
                addSucc(access, carSplit);
            }
            // the depart connections of the forward and backward pedestrian edges and the car edge
            const int prevDep = myDepart[stopEdge][splitIndex];
            const std::vector<int>& backSplitList = myAccessSplits[pair.second];
            const int backBeforeSplit = backSplitList[backSplitList.size() - 2 - splitIndex];
            const int depConn = newConnector(stopEdge);
            addSucc(depConn, fwdSplit);
            addSucc(depConn, backBeforeSplit);
            myEdges[depConn].length = myEdges[fwdSplit].length;
            removeSucc(prevDep, backBeforeSplit);
            addSucc(prevDep, backSplit);
            myEdges[prevDep].length = myEdges[backSplit].length;
            if (carSplit >= 0) {
                addSucc(depConn, carSplit);
            }
            // the arrival connections
            const int prevArr = myArrival[stopEdge][splitIndex];
            const int fwdBeforeSplit = myAccessSplits[pair.first][splitIndex];
            const int arrConn = newConnector(stopEdge);
            addSucc(fwdSplit, arrConn);
            addSucc(backBeforeSplit, arrConn);
            myEdges[arrConn].length = myEdges[fwdSplit].length;
            removeSucc(fwdSplit, prevArr);
            addSucc(fwdBeforeSplit, prevArr);
            myEdges[prevArr].length = myEdges[backSplit].length;
            if (carSplit >= 0) {
                if (removeSucc(carSplit, prevArr)) {
                    addSucc(carSplit, arrConn);
                    addSucc(myAccessSplits[myCar[stopEdge]][splitIndex], prevArr);
                } else {
                    // restricted access
                    const std::vector<std::pair<int, int>> outs = myEdges[carSplit].succ;
                    for (const auto& out : outs) {
                        if (myEdges[out.first].kind == ACCESS && removeSucc(out.first, prevArr)) {
                            addSucc(out.first, arrConn);
                            addRestrictedCarExit(myAccessSplits[myCar[stopEdge]][splitIndex], prevArr,
                                                 myEdges[out.first].vehicleRestriction);
                            break;
                        }
                    }
                }
            }
            myDepart[stopEdge].insert(myDepart[stopEdge].begin() + splitIndex + 1, depConn);
            myArrival[stopEdge].insert(myArrival[stopEdge].begin() + splitIndex + 1, arrConn);
        }
    } else {
        // pedestrians cannot walk here: the stop edge is a depart connector (persons may start at the stop)
        std::vector<int>& splitList = myDepart[stopEdge];
        auto it = splitList.begin();
        while (it != splitList.end() && startPos > endPosOf(myEdges[*it])) {
            ++it;
        }
        splitList.insert(it, stopConn);
        if (!isAccess && (transferWalkTaxi || transferCarWalk || transferTaxiWalk) && myCar[stopEdge] >= 0) {
            const int carEdge = myCar[stopEdge];
            double relPos;
            bool needSplit;
            const int splitIndex = findSplitIndex(carEdge, pos, relPos, needSplit);
            if (needSplit) {
                const int carSplit = newCar(net, stopEdge, pos);
                splitEdge(carEdge, splitIndex, carSplit, relPos, length, needSplit, stopConn, true, false, false);
                if (transferCarWalk || transferTaxiWalk) {
                    // access from car to walk
                    const int beforeSplit = myAccessSplits[myCar[stopEdge]][splitIndex];
                    if (transferCarWalk) {
                        const int access = newAccess(stopConn, length);
                        addSucc(beforeSplit, access);
                        addSucc(access, stopConn);
                    } else if (transferTaxiWalk) {
                        addRestrictedCarExit(beforeSplit, stopConn, TAXI_CLASSES);
                    }
                }
                if (transferWalkTaxi) {
                    const int access = newAccess(carSplit, 0, VC_TAXI, 0, taxiWait);
                    addSucc(stopConn, access);
                    addSucc(access, carSplit);
                }
            }
        }
    }
}

// IntermodalNetwork::addSchedule (the stops have absolute until times of the first vehicle, -1 if not given)
void IntermodalNet::addSchedule(const std::string& vehId, const std::string& line, long long repetitionNumber,
                                Time repetitionOffset, const std::vector<StopSched>& stops) {
    std::vector<const StopSched*> valid;
    Time lastUntil = 0;
    for (const StopSched& s : stops) {
        if (myStops.count(s.busStop) > 0 && s.until >= lastUntil) {
            valid.push_back(&s);
            lastUntil = s.until;
        }
    }
    if (valid.size() < 2 && line != "taxi") {
        return;  // "Not using public transport line ... It has less than two usable stops."
    }
    // PublicTransportEdge::addSchedule
    auto addToEdge = [&](int ptEdge, Time begin, Time travelTime) {
        std::multimap<Time, Schedule>& schedules = mySchedules[myEdges[ptEdge].sched];
        bool found = false;
        for (auto& it : schedules) {
            Schedule& s = it.second;
            if (travelTime == s.travelTime) {
                if (repetitionNumber == -1 && s.repetitionNumber == 1) {
                    if (begin > s.begin) {
                        s.period = begin - s.begin;
                        found = true;
                    }
                } else if (begin == s.begin + s.repetitionNumber * s.period) {
                    found = true;
                }
                if (found) {
                    s.repetitionNumber += std::max(repetitionNumber, 1LL);
                    s.ids.push_back(vehId);
                    break;
                }
            }
        }
        if (!found) {
            schedules.insert({begin, Schedule{{vehId}, begin, std::max(repetitionNumber, 1LL), std::max<Time>(repetitionOffset, 1),
                                              travelTime}});
        }
    };
    std::vector<int>& lineEdges = myPTLines[line];
    if (lineEdges.empty()) {
        const int lineRef = nameIndex(line);
        int lastStop = -1;
        double lastX = 0, lastY = 0;
        Time lastTime = 0;
        for (const StopSched* s : valid) {
            const int currStop = myStops[s->busStop];
            if (lastStop >= 0) {
                IEdge p;
                p.kind = PT;
                p.edge = myEdges[currStop].edge;
                p.ref = lineRef;
                p.length = std::sqrt((s->x - lastX) * (s->x - lastX) + (s->y - lastY) * (s->y - lastY));
                p.entryStop = lastStop;
                p.sched = (int)mySchedules.size();
                mySchedules.emplace_back();
                const int pe = addEdge(p);
                addToEdge(pe, lastTime, s->until - lastTime);
                addSucc(lastStop, pe);
                addSucc(pe, currStop);
                lineEdges.push_back(pe);
            }
            lastTime = s->until;
            lastStop = currStop;
            lastX = s->x;
            lastY = s->y;
        }
        if (line != "taxi" && valid.front()->busStop == valid.back()->busStop) {
            myLoopedLines.insert(lineRef);
        }
    } else {
        if (valid.size() != lineEdges.size() + 1 || myEdges[lineEdges.front()].entryStop != myStops[valid.front()->busStop]) {
            return;  // a different stop sequence than the earlier definitions: ignored
        }
        for (size_t i = 1; i < valid.size(); ++i) {
            if (myEdges[lineEdges[i - 1]].succ[0].first != myStops[valid[i]->busStop]) {
                return;
            }
        }
        Time lastTime = valid.front()->until;
        for (size_t i = 0; i < lineEdges.size(); ++i) {
            addToEdge(lineEdges[i], lastTime, valid[i + 1]->until - lastTime);
            lastTime = valid[i + 1]->until;
        }
    }
}

// IntermodalNetwork::getDepartEdge
int IntermodalNet::departEdge(const Network& net, int edge, double pos) const {
    const std::vector<int>& splits = myDepart[edge];
    if (splits.empty()) {
        return -1;
    }
    if (edge >= myNumNetEdges) {
        return splits[0];  // a taz connector
    }
    bool pedestrians = false;
    for (int l : net.edges[edge].lanes) {
        pedestrians = pedestrians || (net.lanes[l].permissions & VC_PEDESTRIAN) != 0;
    }
    if (!pedestrians) {
        // the most specific split (best train stop)
        double bestDist = std::numeric_limits<double>::max();
        int best = -1;
        for (int split : splits) {
            const IEdge& s = myEdges[split];
            if (pos >= startPosOf(s) - POSITION_EPS && pos <= endPosOf(s) + POSITION_EPS) {
                const double dist = endPosOf(s) - startPosOf(s);
                if (dist < bestDist) {
                    bestDist = dist;
                    best = split;
                }
            }
        }
        return best;
    }
    size_t i = 0;
    double totalLength = 0.;
    while (i + 1 < splits.size() && totalLength + myEdges[splits[i]].length < pos) {
        totalLength += myEdges[splits[i]].length;
        ++i;
    }
    return splits[i];
}

// IntermodalNetwork::getArrivalEdge
int IntermodalNet::arrivalEdge(int edge, double pos) const {
    const std::vector<int>& splits = myArrival[edge];
    if (splits.empty()) {
        return -1;
    }
    double totalLength = 0.;
    for (int split : splits) {
        if (totalLength + myEdges[split].length >= pos) {
            return split;
        }
        totalLength += myEdges[split].length;
    }
    return splits.back();
}

bool IntermodalNet::includeInRoute(const Network& net, const IEdge& e, bool allEdges) const {
    switch (e.kind) {
        case PED: {
            const Edge& E = net.edges[e.edge];
            return allEdges || (!E.isCrossing() && !E.isWalkingArea() && !E.internal);
        }
        case CAR:
        case PT:
        case STOP:
            return true;
        default:
            return false;
    }
}

double IntermodalNet::startPosOf(const IEdge& e) const {
    return (e.kind == PED || e.kind == CAR || e.kind == STOP) ? e.startPos : 0.;
}

double IntermodalNet::endPosOf(const IEdge& e) const {
    switch (e.kind) {
        case PED:
            return e.forward ? e.startPos + e.length : e.startPos - e.length;
        case CAR:
            return e.startPos + e.length;
        case STOP:
            return e.endPos;
        default:
            return e.length;
    }
}

// getPartialLength of pedestrian and car edges
double IntermodalNet::partialLength(const IEdge& e, const IMTrip& trip) const {
    double length = e.length;
    if (e.kind == PED) {
        if (e.edge == trip.from && !e.forward && trip.departPos < e.startPos) {
            length = trip.departPos - (e.startPos - e.length);
        }
        if (e.edge == trip.to && e.forward && trip.arrivalPos < e.startPos + e.length) {
            length = trip.arrivalPos - e.startPos;
        }
        if (e.edge == trip.from && e.forward && trip.departPos > e.startPos) {
            length -= (trip.departPos - e.startPos);
        }
        if (e.edge == trip.to && !e.forward && trip.arrivalPos > e.startPos - e.length) {
            length -= (trip.arrivalPos - (e.startPos - e.length));
        }
        return std::max(length, NUMERICAL_EPS);
    }
    if (e.kind == CAR) {
        // checking the arrival first to have it right for identical depart and arrival edges
        if (e.edge == trip.to && trip.arrivalPos >= e.startPos && trip.arrivalPos < e.startPos + e.length) {
            length = trip.arrivalPos - e.startPos;
        }
        if (e.edge == trip.from && trip.departPos >= e.startPos && trip.departPos < e.startPos + e.length) {
            length -= trip.departPos - e.startPos;
        }
    }
    return length;
}

double IntermodalNet::travelTime(const Network& net, const IEdge& e, const IMTrip& trip, double time) const {
    double tt = travelTimeRaw(net, e, trip, time);
    if (trip.randomFactor > 1) {
        // IntermodalEdge::getTravelTimeStaticRandomized: RandHelper::rand(1, factor)
        const uint64_t id = (uint64_t)(&e - myEdges.data());
        tt *= 1. + rand01(trip.randomSeed64, trip.randomKey, id, RP_PERSON_ROUTE) * (trip.randomFactor - 1.);
    }
    // IntermodalEdge::getTravelTimeStatic: the routing preference of the network edge for the person's type
    if (trip.edgeFactor != nullptr && e.edge >= 0 && e.edge < myNumNetEdges) {
        tt *= (*trip.edgeFactor)[e.edge];
    }
    return tt;
}

double IntermodalNet::travelTimeRaw(const Network& net, const IEdge& e, const IMTrip& trip, double time) const {
    switch (e.kind) {
        case PED: {
            const double length = partialLength(e, trip);
            double tlsDelay = 0;
            if (net.edges[e.edge].isCrossing()) {
                const double timeToCrossing = time - STEPS2TIME(trip.departTime);
                const Lane& L = net.lanes[e.lane];
                // MSLane::getIncomingLinkState (red traffic lights later on the route may be green on arrival)
                const char state = L.logicalPred < 0 ? char{LS_DEADEND} : net.linkTo(L.logicalPred, e.lane)->state;
                if (state == LS_RED) {
                    tlsDelay += std::max(0., TL_RED_PENALTY - timeToCrossing);
                } else if (state == LS_GREEN_MAJOR && timeToCrossing > TL_RED_PENALTY / 2) {
                    tlsDelay += TL_RED_PENALTY;
                }
            }
            return length / (trip.speed * (e.opposite ? walkOppositeFactor : 1.)) + tlsDelay;
        }
        case CAR: {
            // CarEdge::getPartialTravelTime
            const double full = carTravelTime(carTravelTimeOwner, e.edge, trip, time);
            return full * partialLength(e, trip) / net.edges[e.edge].length;
        }
        case PT: {
            // PublicTransportEdge::getTravelTime: waiting for the next departure and riding
            Time minArrival = std::numeric_limits<Time>::max();
            const Time step = TIME2STEPS(time);
            for (const auto& it : mySchedules[e.sched]) {
                const Schedule& s = it.second;
                if (it.first > minArrival) {
                    break;
                }
                const Time offset = std::max<Time>(0, step - s.begin);
                int running = (int)(offset / s.period);
                if (offset % s.period != 0) {
                    running++;
                }
                if (running < s.repetitionNumber) {
                    minArrival = std::min(s.begin + running * s.period + s.travelTime, minArrival);
                }
            }
            return minArrival != std::numeric_limits<Time>::max() ? STEPS2TIME(minArrival - step) : std::numeric_limits<double>::max();
        }
        case ACCESS:
            return e.traveltime > 0 ? e.traveltime : e.length / trip.speed;
        default:
            return 0.;
    }
}

// PublicTransportEdge::getIntended
double IntermodalNet::intended(const IEdge& e, double time, std::string& intended) const {
    if (e.kind != PT) {
        return 0.;
    }
    Time minArrival = std::numeric_limits<Time>::max();
    double bestDepartTime = std::numeric_limits<double>::max();
    const Time step = TIME2STEPS(time);
    for (const auto& it : mySchedules[e.sched]) {
        const Schedule& s = it.second;
        if (it.first > minArrival) {
            break;
        }
        const Time offset = std::max<Time>(0, step - s.begin);
        int running = (int)(offset / s.period);
        if (offset % s.period != 0) {
            running++;
        }
        if (running < s.repetitionNumber) {
            const Time nextDepart = s.begin + running * s.period;
            if (nextDepart + s.travelTime < minArrival) {
                minArrival = nextDepart + s.travelTime;
                bestDepartTime = STEPS2TIME(nextDepart);
                // the naming scheme of MSInsertionControl::determineCandidates
                if (s.ids.size() == 1 || running >= (int)s.ids.size()) {
                    intended = s.repetitionNumber == 1 ? s.ids[0] : s.ids[0] + "." + std::to_string(running);
                } else {
                    intended = s.ids[running];
                }
            }
        }
    }
    return bestDepartTime;
}

bool IntermodalNet::prohibits(const Network& net, const IEdge& e, const IMTrip& trip) const {
    switch (e.kind) {
        case CAR: {
            if (!trip.haveVehicle) {
                return true;
            }
            // MSEdge::prohibits (combined permissions)
            SVCPermissions combined = 0;
            for (int l : net.edges[e.edge].lanes) {
                combined |= net.lanes[l].permissions;
            }
            return (combined & trip.vclass) != trip.vclass;
        }
        case PT:
            return (trip.modeSet & VC_BUS) == 0;
        case ACCESS:
            return (e.modeRestriction != 0 && (trip.modeSet & e.modeRestriction) == 0)
                   || (e.vehicleRestriction != 0 && (trip.vclass & e.vehicleRestriction) == 0);
        default:
            return false;
    }
}

// CarEdge::getViaSuccessors(vClass): connectors, the splits of the edge and the car edges usable by the class
const std::vector<std::pair<int, int>>& IntermodalNet::viaSuccessors(const Network& net, const IEdge& e, SVCPermissions vclass,
                                                                     std::vector<std::pair<int, int>>& buf) const {
    if (e.kind != CAR || vclass == 0) {
        return e.succ;
    }
    buf.clear();
    for (const auto& f : e.succ) {
        const IEdge& fe = myEdges[f.first];
        if (!includeInRoute(net, fe, false) || fe.edge == e.edge
                || (fe.edge >= 0 && net.allowedLanes(e.edge, fe.edge, vclass) != nullptr)) {
            buf.push_back(f);
        }
    }
    return buf;
}

// IntermodalRouter::loopedLineTransfer
bool IntermodalNet::loopedLineTransfer(int prev, int cur, double prevTime, double time) const {
    if (myEdges[cur].kind == PT && myLoopedLines.count(myEdges[cur].ref) != 0) {
        std::string intended1, intended2;
        intended(myEdges[prev], prevTime, intended1);
        intended(myEdges[cur], time, intended2);
        return intended1 != intended2;
    }
    return false;
}

// IntermodalRouter::compute with DijkstraRouter (efforts: travel times; ties by numerical id)
bool IntermodalNet::compute(const Network& net, const IMTrip& trip, const std::string& originStop, const std::string& destStop,
                            std::vector<TripItem>& into) const {
    auto stopEdge = [&](const std::string& id) {
        auto it = myStops.find(id);
        return it == myStops.end() ? -1 : it->second;
    };
    const int iFrom = !originStop.empty() ? stopEdge(originStop) : departEdge(net, trip.from, trip.departPos);
    const int iTo = !destStop.empty() ? stopEdge(destStop) : arrivalEdge(trip.to, trip.arrivalPos);
    if (iFrom < 0 || iTo < 0 || prohibits(net, myEdges[iFrom], trip) || prohibits(net, myEdges[iTo], trip)) {
        return false;
    }
    const int n = (int)myEdges.size();
    // per-thread search state, reset through the list of touched edges
    thread_local std::vector<double> eff, leave;
    thread_local std::vector<int> prev, touched;
    thread_local std::vector<char> visited;
    if ((int)eff.size() < n) {
        eff.assign(n, std::numeric_limits<double>::max());
        leave.assign(n, 0.);
        prev.assign(n, -1);
        visited.assign(n, 0);
        touched.clear();
    }
    for (int t : touched) {
        eff[t] = std::numeric_limits<double>::max();
        prev[t] = -1;
        visited[t] = 0;
    }
    touched.clear();
    using Entry = std::pair<double, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> frontier;
    const SVCPermissions vclass = trip.haveVehicle ? trip.vclass : VC_PEDESTRIAN;
    eff[iFrom] = 0;
    leave[iFrom] = STEPS2TIME(trip.departTime);
    touched.push_back(iFrom);
    frontier.push({0., iFrom});
    std::vector<std::pair<int, int>> buf;
    bool found = false;
    while (!frontier.empty()) {
        const Entry top = frontier.top();
        frontier.pop();
        const int cur = top.second;
        if (visited[cur] || top.first != eff[cur]) {
            continue;
        }
        visited[cur] = 1;
        if (cur == iTo) {
            found = true;
            break;
        }
        const IEdge& E = myEdges[cur];
        const double effortDelta = travelTime(net, E, trip, leave[cur]);
        const double leaveTime = leave[cur] + effortDelta;
        for (const auto& f : viaSuccessors(net, E, vclass, buf)) {
            if (prohibits(net, myEdges[f.first], trip)) {
                continue;
            }
            double e = eff[cur] + effortDelta;
            double time = leaveTime;
            // updateViaEdgeCost
            for (int via = f.second; via >= 0 && myEdges[via].edge >= 0 && net.edges[myEdges[via].edge].internal;) {
                const double d = travelTime(net, myEdges[via], trip, time);
                time += d;
                e += d;
                via = myEdges[via].succ.empty() ? -1 : myEdges[via].succ.front().second;
            }
            if (!visited[f.first] && e < eff[f.first]) {
                if (eff[f.first] == std::numeric_limits<double>::max()) {
                    touched.push_back(f.first);
                }
                eff[f.first] = e;
                leave[f.first] = time;
                prev[f.first] = cur;
                frontier.push({e, f.first});
            }
        }
    }
    if (!found) {
        return false;
    }
    std::vector<int> intoEdges;
    for (int x = iTo; x >= 0; x = prev[x]) {
        intoEdges.push_back(x);
    }
    std::reverse(intoEdges.begin(), intoEdges.end());
    // the trip items (IntermodalRouter::compute); lines: -1 none, -2 walking, -3 car, otherwise the line name
    int lastLine = -1;
    int lastLineEdge = -1;
    double lastLineTime = STEPS2TIME(trip.departTime);
    double time = STEPS2TIME(trip.departTime);
    double effort = 0., length = 0.;
    int prevEdge = -1;
    for (int ie : intoEdges) {
        const IEdge& E = myEdges[ie];
        if (includeInRoute(net, E, false)) {
            if (E.kind == STOP) {
                if (!into.empty()) {
                    // the previous item ends at the stop
                    into.back().destStop = myNames[E.ref];
                    if (lastLine == -2) {
                        lastLine = -1;  // a stop always starts a new trip item
                    }
                } else {
                    // the trip starts at the stop
                    lastLine = -1;
                    TripItem t;
                    t.line = "!stop";
                    t.destStop = myNames[E.ref];
                    into.push_back(t);
                }
            } else {
                const int line = E.kind == PED ? -2 : (E.kind == CAR ? -3 : E.ref);
                if (line != lastLine || (lastLineEdge >= 0 && loopedLineTransfer(lastLineEdge, ie, lastLineTime, time))) {
                    lastLine = line;
                    lastLineEdge = ie;
                    lastLineTime = time;
                    TripItem t;
                    if (line == -3) {
                        t.line = "!car";
                    } else if (line >= 0) {
                        t.line = myNames[line];
                        t.depart = intended(E, time, t.intended);
                    }
                    t.departPos = startPosOf(E);
                    into.push_back(t);
                }
                if (into.back().edges.empty() || into.back().edges.back() != E.edge) {
                    into.back().edges.push_back(E.edge);
                    into.back().arrivalPos = endPosOf(E);
                }
            }
        }
        const double prevTime = time, prevEffort = effort, prevLength = length;
        // SUMOAbstractRouter::updateViaCost
        if (prevEdge >= 0) {
            for (const auto& f : myEdges[prevEdge].succ) {
                if (f.first == ie) {
                    for (int via = f.second; via >= 0 && myEdges[via].edge >= 0 && net.edges[myEdges[via].edge].internal;) {
                        const double d = travelTime(net, myEdges[via], trip, time);
                        time += d;
                        effort += d;
                        length += myEdges[via].length;
                        via = myEdges[via].succ.empty() ? -1 : myEdges[via].succ.front().second;
                    }
                    break;
                }
            }
        }
        const double d = travelTime(net, E, trip, time);
        effort += d;
        time += d;
        length += E.length;
        // the intermodal length
        length += partialLength(E, trip) - E.length;
        prevEdge = ie;
        if (!into.empty()) {
            into.back().traveltime += time - prevTime;
            into.back().cost += effort - prevEffort;
            into.back().length += length - prevLength;
            if (into.back().depart < 0) {
                into.back().depart = prevTime;
            }
        }
    }
    if (!into.empty()) {
        into.back().arrivalPos = trip.arrivalPos;
    }
    return true;
}

}  // namespace ps
