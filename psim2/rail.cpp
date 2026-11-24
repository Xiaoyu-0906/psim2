// Rail signals and driveways, ported from SUMO 1.28 (MSRailSignal, MSDriveWay, MSRailSignalControl).
//
// A driveway (Fahrstraße) is the sequence of tracks a train uses from a rail signal (or its
// departure position) up to the next signal, together with the tracks that protect it (bidirectional
// tracks up to the next switch, flanks). Driveways are built when a train's route becomes known and
// remember which trains are on them; a signal is green for the closest approaching train when no foe
// driveway is occupied and no train with higher priority approaches a conflicting signal.
//
// Not supported: signal constraints, moving block, deadlock definitions, joining/splitting trains,
// rail rerouting at signals.
//
// Execution: driveways are built and signals switched single-threaded (begin of step, vehicle
// loading, rerouting); the trains that enter or leave driveways during the parallel move phase are
// recorded per lane and applied in lane order afterwards (nothing reads the train sets in between).
#include <algorithm>
#include <cstdio>
#include <set>

#include "model.hpp"
#include "railtypes.hpp"

namespace ps {

// ---------------------------------------------------------------------------
// network helpers (MSLane / MSLink / MSEdge)
// ---------------------------------------------------------------------------
int Sim::rlLinkTo(int lane, int target) const {
    const bool internal = isInternal(target);
    for (int l : net.lanes[lane].links) {
        const Link& k = net.links[l];
        if ((internal && k.via == target) || (!internal && k.lane == target)) {
            return l;
        }
    }
    return -1;
}

int Sim::rlEntryLink(int link) const {
    while (isInternal(net.links[link].laneBefore)) {
        link = net.lanes[net.links[link].laneBefore].incoming.front().link;
    }
    return link;
}

int Sim::rlExitLink(int link) const {
    int lane = net.links[link].via;
    while (lane >= 0) {
        link = net.lanes[lane].links[0];
        lane = net.links[link].via;
    }
    return link;
}

int Sim::rlNextNormal(int lane) const {
    return isInternal(lane) ? net.lanes[net.normalSuccessorLane(lane)].edge : net.lanes[lane].edge;
}

bool Sim::rlIsRailSignalLink(int link) const {
    const Link& k = net.links[link];
    return k.tls >= 0 && k.tlIndex >= 0 && net.tls[k.tls].type == "rail_signal";
}

bool Sim::rlEdgeIsRailSignal(int e) const {
    const Edge& ed = net.edges[e];
    return ed.toJ >= 0 && net.junctions[ed.toJ].type == "rail_signal";
}

// MSLink::getFoeLinks equivalent through the junction logic
bool Sim::rlJunctionFoes(int junction, int index, int other) const {
    const Junction& J = net.junctions[junction];
    return J.logic && index >= 0 && other >= 0 && index < (int)J.foes.size() && J.foesTest(index, other);
}

bool Sim::rlIsEmpty(int lane) const {
    return lanes[lane].vehs.empty() && lanes[lane].partials.empty();
}

// MSEdge::isConnectedTo
bool Sim::rlConnected(int edge, int next, SVCPermissions vc) const {
    for (int lane : net.edges[edge].lanes) {
        for (int l : net.lanes[lane].links) {
            if (net.lanes[net.links[l].lane].edge == next && (net.links[l].permissions & vc) == vc) {
                return true;
            }
        }
    }
    return false;
}

// SUMOVehicleClass.cpp
bool isRailway(SVCPermissions p) {
    return (p & VC_RAIL_CLASSES) != 0 && (p & ~(VC_RAIL_CLASSES | VC_TAXI)) == 0;
}

bool isRailwayOrShared(SVCPermissions p) {
    return (p & VC_RAIL_CLASSES) != 0 && (p & (VC_SHIP | VC_AIRCRAFT | VC_DRONE | VC_CONTAINER)) == 0;
}

// ---------------------------------------------------------------------------
// driveway structure (MSDriveWay::buildRoute and the foe checks)
// ---------------------------------------------------------------------------
static void appendMapIndex(std::map<int, int>& visited, int lane) {
    const int tmp = (int)visited.size();
    visited[lane] = tmp;
}

// a link is a switch when its lanes are reached from or lead to other lanes as well
bool Sim::dwIsSwitch(int link) const {
    const Link& k = net.links[link];
    const int predLane = net.normalPredecessorLane(k.laneBefore);
    for (int o : net.lanes[predLane].links) {
        if (net.links[o].lane != k.lane && !net.links[o].isTurnaround()) {
            return true;
        }
    }
    for (const IncomingLane& ili : net.lanes[k.lane].incoming) {
        if (ili.link != link && !net.links[ili.link].isTurnaround()) {
            return true;
        }
    }
    const int bidi = net.lanes[k.lane].bidi;
    if (bidi >= 0) {
        const int predBidi = net.lanes[predLane].bidi;
        for (int o : net.lanes[bidi].links) {
            if (net.links[o].lane != predBidi && !net.links[o].isTurnaround()) {
                return true;
            }
        }
    }
    return false;
}

void Sim::dwBuildRoute(DriveWay& dw, int origin, const std::vector<int>& route, int next, std::map<int, int>& visited,
                       FlankSet& flankSwitches) {
    const int end = (int)route.size();
    double length = 0;
    bool seekForwardSignal = true;
    bool seekBidiSwitch = true;  // (SUMO's loop only ends by returning: myBidiEnded stays false)
    bool foundUnsafeSwitch = false;
    int toLane = origin >= 0 ? net.links[origin].viaOrLane() : net.edges[route[next]].lanes[0];
    while (true) {
        if (length > opt.railMaxBlockLength) {
            dw.abortedBuild = true;
            return;
        }
        const int current = net.lanes[toLane].edge;
        if (net.edges[current].isNormal()) {
            dw.route.push_back(current);
            if (next != end) {
                next++;
            }
        }
        appendMapIndex(visited, toLane);
        length += net.lanes[toLane].length;
        const int bidi = net.lanes[toLane].bidi;
        if (seekForwardSignal) {
            if (!foundUnsafeSwitch) {
                dw.forward.push_back(toLane);
                if (net.lanes[toLane].isNormal()) {
                    dw.forwardEdgeCount++;
                }
                if (dw.forward.size() == 1) {
                    dw.lane = toLane;
                    if ((int)laneDriveWays.size() < (int)net.lanes.size()) {
                        laneDriveWays.resize(net.lanes.size());
                    }
                    laneDriveWays[toLane].push_back(dw.index);
                }
            }
        } else if (bidi < 0) {
            if (!(isInternal(toLane) && net.links[net.lanes[toLane].incoming.front().link].isTurnaround())) {
                seekBidiSwitch = false;
            }
        }
        if (bidi >= 0) {
            if (!seekForwardSignal && !foundUnsafeSwitch && net.lanes[bidi].isNormal()) {
                // look for a switch that could protect from oncoming vehicles
                for (int l : net.lanes[bidi].links) {
                    const Link& k = net.links[l];
                    if (k.dir == 't') {
                        continue;
                    }
                    if (!dw.bidi.empty() && k.viaOrLane() != dw.bidi.back()) {
                        dw.coreSize = (int)dw.route.size() - 1;
                        const int used = rlLinkTo(bidi, dw.bidi.back());
                        // trains along our route beyond this switch might create deadlock
                        foundUnsafeSwitch = true;
                        // the switch itself must still be guarded to ensure safety
                        if (used >= 0) {
                            flankSwitches.insert(used);
                        }
                        break;
                    }
                }
            }
            if (foundUnsafeSwitch) {
                dw.bidiExtended.push_back(bidi);
            } else {
                dw.bidi.push_back(bidi);
            }
        }
        const std::vector<int>& lks = net.lanes[toLane].links;
        toLane = -1;
        for (int l : lks) {
            const Link& k = net.links[l];
            if (next != end && net.lanes[k.lane].edge == route[next]) {
                toLane = k.viaOrLane();
                if (rlIsRailSignalLink(l)) {
                    if (l == origin) {
                        dw.abortedBuild = true;  // circular block
                        return;
                    }
                    seekForwardSignal = false;
                    dw.foundSignal = true;
                    seekBidiSwitch = bidi >= 0;
                }
                if (dwIsSwitch(l)) {
                    switchDriveWays[l].push_back(dw.index);
                }
                if (net.lanes[k.lane].bidi >= 0 && net.lanes[k.lane].edge == net.edges[current].bidi) {
                    // reversal on driveway
                    reversalDriveWays[current].push_back(dw.index);
                    dw.reversals.push_back(current);
                }
                break;
            }
        }
        (void)seekBidiSwitch;
        if (toLane < 0) {
            if (next != end) {
                dw.foundJump = true;  // turn-around or jump
                return;
            }
            dw.terminateRoute = true;
            return;
        }
    }
}

bool Sim::dwFlankConflict(const DriveWay& a, const DriveWay& other) const {
    for (int lane : a.forward) {
        for (int lane2 : other.forward) {
            if (lane == lane2) {
                return true;
            }
        }
        for (int lane2 : other.bidi) {
            if (lane == lane2 && dwBidiBlockedBy(a, other)) {
                return true;
            }
        }
        for (int lane2 : other.bidiExtended) {
            if (lane == lane2 && dwBidiBlockedBy(a, other)) {
                return true;
            }
        }
    }
    return false;
}

bool Sim::dwCrossingConflict(const DriveWay& a, const DriveWay& other) const {
    for (int lane : a.forward) {
        for (int lane2 : other.forward) {
            if (net.lanes[lane].isNormal() && net.lanes[lane2].isNormal()
                    && net.edges[net.lanes[lane].edge].toJ == net.edges[net.lanes[lane2].edge].toJ) {
                return true;
            }
        }
    }
    if (a.origin >= 0 && other.origin >= 0) {
        const Link& o1 = net.links[a.origin];
        const Link& o2 = net.links[other.origin];
        if (o1.junction == o2.junction && o1.junction >= 0 && rlJunctionFoes(o1.junction, o1.index, o2.index)) {
            // switch/crossing is also a rail_signal (direct control)
            if (!(isInternal(a.forward.front()) && isInternal(other.forward.front()))) {
                return false;
            }
            return true;
        }
    }
    if (other.origin >= 0 && isInternal(other.forward.front())) {
        const Link& o2 = net.links[other.origin];
        for (int i = 0; i < (int)a.forward.size() - 1; i++) {
            const int lane = a.forward[i];
            if (net.edges[net.lanes[lane].edge].toJ == o2.junction) {
                const int link = rlLinkTo(lane, a.forward[i + 1]);
                if (link >= 0 && net.links[link].tls < 0) {
                    // switch/crossing is also a rail_signal (direct control) but own link is uncontrolled
                    if (rlJunctionFoes(o2.junction, net.links[link].index, o2.index)) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

bool Sim::dwOverlap(const DriveWay& a, const DriveWay& other) const {
    for (int i = 0; i < a.coreSize; i++) {
        for (int j = 0; j < other.coreSize; j++) {
            const Edge& e1 = net.edges[a.route[i]];
            const Edge& e2 = net.edges[other.route[j]];
            if (e1.toJ == e2.toJ || e1.toJ == e2.fromJ || e1.fromJ == e2.fromJ) {
                return true;
            }
        }
    }
    return false;
}

bool Sim::dwBidiBlockedBy(const DriveWay& a, const DriveWay& other) const {
    for (int lane : a.bidi) {
        for (int lane2 : other.forward) {
            if (lane == lane2) {
                return true;
            }
        }
    }
    for (int lane : a.bidiExtended) {
        for (int lane2 : other.forward) {
            if (lane == lane2 && dwOverlap(a, other)) {
                return true;
            }
        }
    }
    return false;
}

bool Sim::dwBidiBlockedByEnd(const DriveWay& a, const DriveWay& other) const {
    const int end = other.forward.back();
    for (int lane : a.bidi) {
        if (lane == end) {
            return true;
        }
    }
    for (int lane : a.bidiExtended) {
        if (lane == end && dwOverlap(a, other)) {
            return true;
        }
    }
    return false;
}

bool Sim::dwForwardRouteConflict(const DriveWay& a, const std::set<int>& forward, const DriveWay& other, bool secondCheck) const {
    int i = 0;
    for (int edge2 : other.route) {
        if (i == other.coreSize) {
            return false;
        }
        i++;
        if (edge2 == rlNextNormal(a.forward.front()) && !secondCheck) {
            // foe should not pass from behind through our own forward section
            return false;
        }
        if (net.edges[edge2].bidi >= 0 && forward.count(net.edges[edge2].bidi) != 0) {
            return true;
        }
    }
    return false;
}

void Sim::dwCheckFlanks(const DriveWay& dw, int originLink, const std::vector<int>& lanesList, const std::map<int, int>& visited,
                        bool allFoes, bool movingBlock, FlankSet& flankSwitches) const {
    int reverseOriginLink = -1;
    if (originLink >= 0) {
        const Link& o = net.links[originLink];
        const int lb = net.lanes[o.lane].bidi;
        const int bb = net.lanes[o.laneBefore].bidi;
        if (lb >= 0 && bb >= 0) {
            reverseOriginLink = rlLinkTo(lb, bb);
            if (reverseOriginLink >= 0) {
                reverseOriginLink = rlExitLink(reverseOriginLink);
            }
        }
    }
    for (int i = 0; i < (int)lanesList.size(); i++) {
        const int lane = lanesList[i];
        const int prev = i > 0 ? lanesList[i - 1] : -1;
        const int nxt = i + 1 < (int)lanesList.size() ? lanesList[i + 1] : -1;
        if (isInternal(lane)) {
            continue;
        }
        for (const IncomingLane& ili : net.lanes[lane].incoming) {
            const Link& k = net.links[ili.link];
            if (ili.link == originLink || ili.link == reverseOriginLink || k.dir == 't' || k.dir == 'T'
                    || (originLink < 0 && i == 0 && movingBlock)) {
                continue;
            }
            if (ili.lane != prev && ili.lane != nxt) {
                flankSwitches.insert(ili.link);
            } else if (allFoes) {
                // link is part of the driveway, find foes that cross the driveway without entering
                dwCheckCrossingFlanks(ili.link, visited, flankSwitches);
            }
        }
    }
    (void)dw;
}

void Sim::dwCheckCrossingFlanks(int dwLink, const std::map<int, int>& visited, FlankSet& flankSwitches) const {
    const Link& d = net.links[dwLink];
    if (d.junction < 0 || !net.junctions[d.junction].logic) {
        return;  // unregulated junction
    }
    const Junction& J = net.junctions[d.junction];
    std::vector<int> incomingEdges;
    for (int inc : J.incLanes) {
        const int e = net.lanes[inc].edge;
        if (std::find(incomingEdges.begin(), incomingEdges.end(), e) == incomingEdges.end()) {
            incomingEdges.push_back(e);
        }
    }
    for (int in : incomingEdges) {
        if (net.edges[in].internal) {
            continue;
        }
        for (int inLane : net.edges[in].lanes) {
            const int inBidi = net.lanes[inLane].bidi;
            if (isRailwayOrShared(net.lanes[inLane].permissions) && visited.count(inLane) == 0 && (inBidi < 0 || visited.count(inBidi) == 0)) {
                for (int l : net.lanes[inLane].links) {
                    const Link& k = net.links[l];
                    if (k.index >= 0 && rlJunctionFoes(d.junction, d.index, k.index) && visited.count(k.lane) == 0) {
                        flankSwitches.insert(k.via < 0 ? l : net.lanes[k.via].links.front());
                    }
                }
            }
        }
    }
}

void Sim::dwAddFoes(DriveWay& dw, int link) {
    const Link& k = net.links[link];
    if (k.tls < 0 || net.tls[k.tls].type != "rail_signal") {
        return;
    }
    const RailSignal& rs = railSignals[tlsRail[k.tls]];
    for (int fi : rs.links[k.tlIndex].driveways) {
        DriveWay& foe = *driveWays[fi];
        if (&foe != &dw && (dwFlankConflict(dw, foe) || dwFlankConflict(foe, dw) || dwCrossingConflict(dw, foe) || dwCrossingConflict(foe, dw))) {
            dwAddFoe(dw, fi);
        }
    }
}

void Sim::dwAddSwitchFoes(DriveWay& dw, int link) {
    auto it = switchDriveWays.find(link);
    if (it != switchDriveWays.end()) {
        for (int fi : it->second) {
            DriveWay& foe = *driveWays[fi];
            if (&foe != &dw && (dwFlankConflict(dw, foe) || dwFlankConflict(foe, dw) || dwCrossingConflict(dw, foe) || dwCrossingConflict(foe, dw))) {
                dwAddFoe(dw, fi);
            }
        }
    }
}

void Sim::dwFindFlankProtection(DriveWay& dw, int link, int origLink, std::vector<int>& flank) {
    const int entry = rlEntryLink(link);
    const Link& k = net.links[link];
    if (net.links[entry].tls >= 0 && k.junction >= 0 && net.junctions[k.junction].type == "rail_signal") {
        // guarded by signal
        dw.conflictLinks.push_back(entry);
        dwAddFoes(dw, entry);
    } else {
        const int lane = k.laneBefore;
        std::vector<int> predLinks;
        for (const IncomingLane& ili : net.lanes[lane].incoming) {
            if (!net.links[ili.link].isTurnaround()) {
                predLinks.push_back(ili.link);
            }
        }
        if (predLinks.size() > 1) {
            // this is a switch
            for (int pred : predLinks) {
                dwAddSwitchFoes(dw, pred);
            }
        } else if (predLinks.size() == 1) {
            if (dwIsSwitch(link)) {
                dwAddSwitchFoes(dw, link);
            } else {
                // continue upstream via single predecessor
                dwFindFlankProtection(dw, predLinks.front(), origLink, flank);
            }
        }
        // check for insertions
        auto dit = departureDriveways.find(net.lanes[lane].edge);
        if (dit != departureDriveways.end()) {
            for (int fi : dit->second) {
                DriveWay& foe = *driveWays[fi];
                if (dwFlankConflict(dw, foe) || dwCrossingConflict(dw, foe)) {
                    dwAddFoe(dw, fi);
                }
            }
        }
    }
}

void Sim::dwAddBidiFoes(DriveWay& dw, int ownSignal, bool extended) {
    const std::vector<int> bidiLanes = extended ? dw.bidiExtended : dw.bidi;
    for (int bidi : bidiLanes) {
        for (const IncomingLane& ili : net.lanes[bidi].incoming) {
            const Link& k = net.links[ili.link];
            const int rs = k.tls >= 0 && net.tls[k.tls].type == "rail_signal" ? tlsRail[k.tls] : -1;
            if (rs >= 0 && rs != ownSignal && std::find(bidiLanes.begin(), bidiLanes.end(), ili.lane) != bidiLanes.end()) {
                dwAddFoes(dw, ili.link);
            }
        }
        const int bidiEdge = net.lanes[bidi].edge;
        auto dit = departureDriveways.find(bidiEdge);
        if (dit != departureDriveways.end()) {
            for (int fi : dit->second) {
                if (dwFlankConflict(dw, *driveWays[fi])) {
                    dwAddFoe(dw, fi);
                }
            }
        }
        auto eit = departureDrivewaysEnds.find(bidiEdge);
        if (eit != departureDrivewaysEnds.end()) {
            for (int fi : eit->second) {
                if (dwFlankConflict(dw, *driveWays[fi])) {
                    dwAddFoe(dw, fi);
                }
            }
        }
    }
}

void Sim::dwAddParallelFoes(DriveWay& dw, int link, int first) {
    if (link >= 0) {
        dwAddFoes(dw, link);
    } else {
        auto it = departureDriveways.find(first);
        if (it != departureDriveways.end()) {
            for (int fi : it->second) {
                dwAddFoe(dw, fi);
            }
        }
    }
}

void Sim::dwAddReversalFoes(DriveWay& dw, bool movingBlock) {
    std::set<int> forward;
    for (int lane : dw.forward) {
        if (net.lanes[lane].isNormal()) {
            forward.insert(net.lanes[lane].edge);
        }
    }
    int i = 0;
    for (int e : dw.route) {
        if (forward.count(e) != 0 && !movingBlock) {
            continue;  // reversals in our own forward section: each driveway is its own foe
        }
        if (i == dw.coreSize) {
            break;
        }
        i++;
        auto it = reversalDriveWays.find(e);
        if (it != reversalDriveWays.end()) {
            const std::vector<int> cands = it->second;
            for (int fi : cands) {
                DriveWay& foe = *driveWays[fi];
                if (dwForwardRouteConflict(dw, forward, foe, false)) {
                    std::set<int> foeForward;
                    for (int lane : foe.forward) {
                        if (net.lanes[lane].isNormal()) {
                            foeForward.insert(net.lanes[lane].edge);
                            if (net.lanes[lane].bidi >= 0) {
                                foeForward.insert(net.edges[net.lanes[lane].edge].bidi);
                            }
                        }
                    }
                    if (dwForwardRouteConflict(foe, foeForward, dw, true)) {
                        dwAddFoe(dw, fi);
                    }
                } else if (movingBlock && &foe == &dw) {
                    dwAddFoe(dw, fi);
                }
            }
        }
    }
}

bool Sim::dwForwardEndOnRoute(const DriveWay& a, const DriveWay& foe) const {
    const int foeForwardEnd = net.lanes[net.normalPredecessorLane(foe.forward.back())].edge;
    return std::find(a.route.begin(), a.route.end(), foeForwardEnd) != a.route.end();
}

bool Sim::dwHasRS(int cur, int next) const {
    if (rlEdgeIsRailSignal(cur)) {
        for (int lane : net.edges[cur].lanes) {
            for (int l : net.lanes[lane].links) {
                if (net.lanes[net.links[l].lane].edge == next && net.links[l].tls >= 0) {
                    return true;
                }
            }
        }
    }
    return false;
}

int Sim::dwFoeOrSubFoe(const DriveWay& a, int dw) const {
    dwSpecRead(a.index);
    dwSpecRead(dw);
    if (std::find(a.foes.begin(), a.foes.end(), dw) != a.foes.end()) {
        return dw;
    }
    for (int sub : driveWays[dw]->subs) {
        const int foe = dwFoeOrSubFoe(a, sub);
        if (foe >= 0) {
            return foe;
        }
    }
    return -1;
}

void Sim::dwAddSidings(DriveWay& dw, DriveWay& foe, bool addToFoe) {
    dwMutated(dw.index);
    dwMutated(foe.index);
    const std::vector<int>& foeRoute = foe.parent >= 0 ? driveWays[foe.parent]->route : foe.route;
    int foeEndBidi = net.edges[net.lanes[foe.forward.back()].edge].bidi;
    int foeForwardNormals = 0;
    for (int lane : foe.forward) {
        if (net.lanes[lane].isNormal()) {
            foeForwardNormals++;
        }
    }
    if (foeForwardNormals == (int)foeRoute.size()) {
        return;
    }
    auto foeSearchBeg = foeRoute.begin() + foeForwardNormals;
    auto foeSearchEnd = foeRoute.end();
    if (foeEndBidi < 0) {
        throw std::runtime_error("checkSiding " + dw.id + " foe=" + foe.id + " noBidi");
    }
    // if foe is a subDriveway, the forward section may end on an internal edge which would not be found on the route
    if (net.edges[foeEndBidi].internal) {
        foeEndBidi = net.lanes[net.normalSuccessorLane(net.edges[foeEndBidi].lanes[0])].edge;
    }
    std::set<int> foeForwardEdges(foeRoute.begin(), foeRoute.begin() + foeForwardNormals);
    int forwardNormals = 0;
    for (int lane : dw.forward) {
        if (net.lanes[lane].isNormal()) {
            forwardNormals++;
            if (foeForwardEdges.count(net.lanes[lane].edge) != 0) {
                return;  // forward edge on the foe forward section (sidings unsafe)
            }
        }
    }
    int i;
    std::vector<int> start;
    std::vector<double> length;
    std::vector<std::vector<int>> intermediateRS;
    for (i = 0; i < (int)dw.route.size(); i++) {
        if (dw.route[i] == foeEndBidi) {
            break;
        }
    }
    if (i == (int)dw.route.size()) {
        throw std::runtime_error("checkSiding " + dw.id + " foe=" + foe.id + " foeEndBidi not on route");
    }
    int next = dw.route[i];
    i--;
    for (; i >= 0; i--) {
        const int cur = dw.route[i];
        const bool curHasRS = dwHasRS(cur, next);
        const int curBidi = net.edges[cur].bidi;
        if (curHasRS) {
            if (std::find(foeSearchBeg, foeSearchEnd, curBidi) == foeSearchEnd) {
                start.push_back(i);
                length.push_back(0);
                intermediateRS.push_back({});
            }
        }
        if (!start.empty()) {
            auto itFind = std::find(foeSearchBeg, foeSearchEnd, curBidi);
            if (itFind != foeSearchEnd) {
                const int firstIndex = i + 1;
                if (addToFoe) {
                    std::vector<Siding>& foeSidings = foe.sidings[dw.index];
                    const int first = dw.route[firstIndex];
                    auto itFirst = std::find(foeRoute.begin(), foeRoute.end(), first);
                    if (itFirst != foeRoute.end()) {
                        for (int j = 0; j < (int)length.size(); j++) {
                            const int last = dw.route[start[j]];
                            auto itLast = std::find(itFirst, foeRoute.end(), last);
                            if (itLast != foeRoute.end()) {
                                foeSidings.insert(foeSidings.begin(), Siding{(int)(itFirst - foeRoute.begin()), (int)(itLast - foeRoute.begin()), length[j], {}});
                                if (foeSidings.size() > 2) {
                                    foeSidings.pop_back();
                                }
                            }
                        }
                    }
                } else {
                    std::vector<int> furtherRS;
                    if (curHasRS) {
                        furtherRS.push_back(i);
                    }
                    int next2 = cur;
                    for (int i2 = i - 1; i2 >= forwardNormals; i2--) {
                        const int cur2 = dw.route[i2];
                        if (dwHasRS(cur2, next2)) {
                            furtherRS.push_back(i2);
                        }
                        next2 = cur2;
                    }
                    std::vector<Siding>& foeSidings = dw.sidings[foe.index];
                    for (int j = 0; j < (int)length.size(); j++) {
                        intermediateRS[j].insert(intermediateRS[j].end(), furtherRS.begin(), furtherRS.end());
                        foeSidings.insert(foeSidings.begin(), Siding{firstIndex, start[j], length[j], intermediateRS[j]});
                        if (foeSidings.size() > 2) {
                            foeSidings.pop_back();
                        }
                    }
                }
                start.clear();
                length.clear();
                intermediateRS.clear();
                foeSearchBeg = itFind;
            } else {
                for (int j = 0; j < (int)length.size(); j++) {
                    length[j] += net.edges[cur].length;
                    if (curHasRS && i != start[j]) {
                        intermediateRS[j].push_back(i);
                    }
                }
            }
        }
        next = cur;
    }
}

bool Sim::dwBuildSubFoe(DriveWay& dw, DriveWay& foe, bool movingBlock) {
    if (dw.forward.size() < foe.forward.size()
            && std::equal(dw.forward.begin(), dw.forward.end(), foe.forward.begin())) {
        dwAddFoe(foe, dw.index);  // subpart of foe
        return true;
    }
    int subLast = (int)dw.forward.size() - 2;
    if (movingBlock && dw.forward.back() == foe.forward.back()) {
        subLast++;
    }
    bool foundConflict = false;
    bool zipperC = false;
    bool crossC = false;
    auto forwardDistance = [&](int lastIndex) {
        double result = 0;
        for (int i = 0; i <= lastIndex; i++) {
            result += net.lanes[dw.forward[i]].length;
        }
        return result;
    };
    auto isZipper = [&](int j) { return j >= 0 && net.junctions[j].type == "zipper"; };
    while (subLast >= 0) {
        const int lane = dw.forward[subLast];
        DriveWay tmp;
        tmp.origin = subLast > 0 ? rlLinkTo(dw.forward[subLast - 1], lane) : dw.origin;
        tmp.forward.push_back(lane);
        tmp.bidi = dw.bidi;
        tmp.bidiExtended = dw.bidiExtended;
        tmp.route.push_back(rlNextNormal(lane));
        tmp.coreSize = 1;
        const bool flankC = dwFlankConflict(tmp, foe);
        const bool bidiConflict = std::find(foe.bidi.begin(), foe.bidi.end(), lane) != foe.bidi.end();
        crossC = dwCrossingConflict(tmp, foe);
        if (flankC || crossC || bidiConflict) {
            foundConflict = true;
            if (!movingBlock || bidiConflict) {
                break;
            }
            const Edge& le = net.edges[net.lanes[lane].edge];
            if (((flankC && isZipper(le.fromJ)) || (!flankC && isZipper(le.toJ)))
                    && (dw.origin < 0 || forwardDistance(flankC ? subLast - 1 : subLast) > opt.railMovingBlockMaxDist)) {
                // a zipper merge resolves the conflict of moving-block trains (unless the signal is close to it)
                zipperC = true;
                foundConflict = false;
                if (!flankC && crossC) {
                    return false;
                }
            }
            if (!flankC && crossC) {
                break;
            }
        } else if (foundConflict) {
            break;
        }
        subLast--;
    }
    if (dwBidiBlockedByEnd(dw, foe) && dwBidiBlockedBy(dw, dw) && dwForwardEndOnRoute(foe, dw)) {
        std::vector<int> forward(dw.route.begin(), dw.route.begin() + dw.forwardEdgeCount);
        const int fb = foe.forwardEdgeCount;
        std::vector<int> foeAfterForward(foe.route.begin() + fb,
                                         foe.route.begin() + std::min((int)foe.route.size(), fb + (int)forward.size()));
        if (forward == foeAfterForward) {
            dwAddFoe(foe, dw.index);
            dwAddSidings(dw, foe, true);  // foe will get the sidings
            return true;
        }
    }
    if (subLast < 0) {
        if (movingBlock && zipperC) {
            return false;
        } else if (net.lanes[dw.forward.back()].edge == dw.route.back() && dwForwardEndOnRoute(foe, dw)) {
            // driveway ends in the middle of the block and only the final edge overlaps with the foe
            dwAddFoe(foe, dw.index);
        } else if (foe.terminateRoute) {
            if (dwBidiBlockedByEnd(dw, foe) && dwBidiBlockedBy(dw, dw) && dwForwardEndOnRoute(foe, dw)) {
                dwAddFoe(foe, dw.index);
                dwAddSidings(dw, foe, true);
            }
        } else if (dw.terminateRoute && dw.bidi.size() <= dw.forward.size()) {
            dwAddFoe(foe, dw.index);
            return true;
        }
        return false;
    }
    const int subSize = subLast + 1;
    for (int ci : dw.subs) {
        DriveWay& cand = *driveWays[ci];
        if ((int)cand.forward.size() == subSize) {
            // re-use existing sub-driveway
            dwAddFoe(foe, ci);
            if (dwBidiBlockedByEnd(foe, cand)) {
                dwAddSidings(foe, cand, false);
            }
            dwAddFoe(cand, foe.index);
            return true;
        }
    }
    std::vector<int> forward(dw.forward.begin(), dw.forward.begin() + subSize);
    std::vector<int> route;
    for (int lane : forward) {
        if (net.lanes[lane].isNormal()) {
            route.push_back(net.lanes[lane].edge);
        }
    }
    if (route.empty()) {
        const int fromJ = net.edges[net.lanes[forward.front()].edge].fromJ;
        if (subSize == 1 && crossC && fromJ == net.edges[net.lanes[foe.forward.front()].edge].fromJ
                && net.junctions[fromJ].type == "rail_signal") {
            // sub-driveway ends after a single internal lane: add the next edge
            route.push_back(rlNextNormal(foe.forward.front()));
        } else {
            return false;
        }
    }
    if (dw.route.size() > route.size()) {
        // route continues: the subDriveway must not end with a reversal
        if (net.edges[route.back()].bidi == dw.route[route.size()]) {
            return false;
        }
    }
    const int si = newDriveWay(dw.origin, dw.id + "." + std::to_string(dw.subs.size()));
    DriveWay& sub = *driveWays[si];
    DriveWay& parent = *driveWays[dw.index];  // (the table may have grown)
    sub.lane = parent.lane;
    sub.parent = parent.index;
    sub.forward = forward;
    sub.route = route;
    sub.coreSize = (int)sub.route.size();
    laneDriveWays[sub.lane].push_back(si);
    // copy trains that are currently on this driveway
    for (int v : parent.trains) {
        const int e = routes[veh[v].route][veh[v].routeIdx];
        if (std::find(sub.route.begin(), sub.route.end(), e) != sub.route.end()) {
            dwTrainAdd(si, v);
            veh[v].ex().dwHeld.push_back(si);
            for (const DwVehEvent& ev : parent.events) {
                if (ev.veh == v) {
                    sub.events.push_back(ev);  // (with the associated entry events)
                }
            }
        }
    }
    DriveWay& foe2 = *driveWays[foe.index];
    dwAddFoe(foe2, si);
    if (dwBidiBlockedByEnd(foe2, sub)) {
        dwAddSidings(foe2, sub, false);
    }
    dwAddFoe(sub, foe2.index);
    parent.subs.push_back(si);
    dwMutated(parent.index);
    return true;
}

int Sim::newDriveWay(int origin, const std::string& id) {
    std::unique_ptr<DriveWay> dw(new DriveWay());
    dw->index = (int)driveWays.size();
    dw->origin = origin;
    dw->id = id;
    driveWays.push_back(std::move(dw));
    dwHot.push_back(DwHot{0, origin, 0});
    dwFoeOf.emplace_back();
    dwTouched.push_back(0);
    return (int)driveWays.size() - 1;
}

// MSDriveWay::buildDriveWay
int Sim::buildDriveWay(const std::string& id, int link, const std::vector<int>& route, int first) {
    const int di = newDriveWay(link, id);
    std::map<int, int> visited;
    std::vector<int> before;
    int fromBidi = -1;
    if (link >= 0) {
        appendMapIndex(visited, net.links[link].laneBefore);
        fromBidi = net.lanes[net.links[link].laneBefore].bidi;
    }
    FlankSet flankSwitches(FlankLess{&net});
    if (fromBidi >= 0) {
        before.push_back(fromBidi);
    }
    dwBuildRoute(*driveWays[di], link, route, first, visited, flankSwitches);
    DriveWay& dw = *driveWays[di];
    dw.coreSize = (int)dw.route.size();
    const int rs = link >= 0 ? tlsRail[net.links[link].tls] : -1;
    const bool movingBlock = (rs >= 0 && railSignals[rs].movingBlock)
                             || (rs < 0 && (opt.railMovingBlock || isMovingBlockClass(rtPermOf(route[first]))));
    dwCheckFlanks(dw, link, dw.forward, visited, true, movingBlock, flankSwitches);
    dwCheckFlanks(dw, link, dw.bidi, visited, false, movingBlock, flankSwitches);
    dwCheckFlanks(dw, link, before, visited, true, movingBlock, flankSwitches);
    for (int fsLink : flankSwitches) {
        dwFindFlankProtection(*driveWays[di], fsLink, fsLink, driveWays[di]->flank);
    }
    FlankSet flankSwitchesBidiExtended(FlankLess{&net});
    dwCheckFlanks(*driveWays[di], link, driveWays[di]->bidiExtended, visited, false, movingBlock, flankSwitchesBidiExtended);
    for (int fl : flankSwitchesBidiExtended) {
        dwFindFlankProtection(*driveWays[di], fl, fl, driveWays[di]->bidiExtended);
    }
    DriveWay& d = *driveWays[di];
    if (rs < 0 || !railSignals[rs].movingBlock) {
        d.conflictLanes.insert(d.conflictLanes.end(), d.forward.begin(), d.forward.end());
    }
    d.conflictLanes.insert(d.conflictLanes.end(), d.bidi.begin(), d.bidi.end());
    d.conflictLanes.insert(d.conflictLanes.end(), d.flank.begin(), d.flank.end());
    dwAddBidiFoes(d, rs, false);
    dwAddBidiFoes(d, rs, true);
    if (!movingBlock) {
        // add driveways that start on the same signal / lane
        dwAddParallelFoes(d, link, route[first]);
    }
    // add driveways that reverse along this driveway's route
    dwAddReversalFoes(d, movingBlock);
    // make foes unique and symmetrical (sorted by numerical id)
    std::set<int> uniqueFoes(d.foes.begin(), d.foes.end());
    dwClearFoes(d);
    // check for a self-intersecting forward section in moving-block mode
    if (movingBlock && uniqueFoes.count(di) == 0) {
        std::map<int, std::vector<int>> forwardJunctions;
        const int iLast = (int)d.forward.size() - 1;
        bool selfIntersect = false;
        for (int i = 0; i < iLast && !selfIntersect; i++) {
            const int fw = d.forward[i];
            if (net.lanes[fw].isNormal()) {
                const int fwTo = net.edges[net.lanes[fw].edge].toJ;
                const int fwLink = rlLinkTo(fw, d.forward[i + 1]);
                if (fwLink >= 0) {
                    const Link& fk = net.links[fwLink];
                    for (int link2 : forwardJunctions[fwTo]) {
                        const Link& k2 = net.links[link2];
                        if (std::find(fk.foeLinks.begin(), fk.foeLinks.end(), link2) != fk.foeLinks.end()
                                || std::find(k2.foeLinks.begin(), k2.foeLinks.end(), fwLink) != k2.foeLinks.end()
                                || net.lanes[fk.lane].bidi == k2.laneBefore || net.lanes[k2.lane].bidi == fk.laneBefore) {
                            dwAddFoe(d, di);
                            selfIntersect = true;
                            break;
                        }
                    }
                    forwardJunctions[fwTo].push_back(fwLink);
                }
            }
        }
    }
    std::set<int, FlankLess> uniqueCLink(d.conflictLinks.begin(), d.conflictLinks.end(), FlankLess{&net});
    const int lastEdge = net.lanes[d.forward.back()].edge;
    for (int fi : uniqueFoes) {
        DriveWay& foe = *driveWays[fi];
        DriveWay& self = *driveWays[di];
        const int foeLastEdge = net.lanes[foe.forward.back()].edge;
        const bool sameLast = foeLastEdge == lastEdge;
        if (sameLast && !movingBlock) {
            dwAddFoe(self, fi);
            if (fi != di) {
                dwAddFoe(foe, di);
            }
        } else {
            if (dwBidiBlockedByEnd(foe, self)) {
                dwAddFoe(foe, di);
                dwAddSidings(foe, self, false);
            } else {
                dwBuildSubFoe(self, foe, movingBlock);
            }
            if (fi != di) {
                DriveWay& self2 = *driveWays[di];
                DriveWay& foe2 = *driveWays[fi];
                if (dwBidiBlockedByEnd(self2, foe2)) {
                    dwAddFoe(self2, fi);
                    dwAddSidings(self2, foe2, false);
                } else {
                    dwBuildSubFoe(foe2, self2, movingBlock);
                }
            }
        }
        DriveWay& foe3 = *driveWays[fi];
        DriveWay& self3 = *driveWays[di];
        if (link >= 0) {
            dwAddConflictLink(foe3, link);
        }
        // ignore links that have the same start junction
        if (net.edges[foe3.route.front()].fromJ != net.edges[self3.route.front()].fromJ) {
            for (const IncomingLane& ili : net.lanes[foe3.forward.front()].incoming) {
                if (net.links[ili.link].tls >= 0) {
                    // ignore links that originate on our bidi lanes
                    const int origin = net.links[ili.link].laneBefore;
                    if (std::find(self3.bidi.begin(), self3.bidi.end(), origin) == self3.bidi.end()) {
                        uniqueCLink.insert(ili.link);
                    }
                }
            }
        }
    }
    DriveWay& fin = *driveWays[di];
    fin.conflictLinks.assign(uniqueCLink.begin(), uniqueCLink.end());
    endingDriveways[lastEdge].push_back(di);
    if (!movingBlock) {
        // every driveway is its own foe (also all driveways that depart in the same block)
        for (int se : endingDriveways[lastEdge]) {
            if (uniqueFoes.count(se) == 0) {
                dwAddFoe(fin, se);
                if (se != di) {
                    dwAddFoe(*driveWays[se], di);
                }
            }
        }
    }
    return di;
}

void Sim::dwAddConflictLink(DriveWay& dw, int link) {
    dwMutated(dw.index);
    const Link& k = net.links[link];
    if (k.tls >= 0) {
        // ignore links that originate on the bidi lanes and links from the same junction as the own link
        if (std::find(dw.bidi.begin(), dw.bidi.end(), k.laneBefore) == dw.bidi.end()) {
            if (k.junction != net.edges[dw.route.front()].fromJ) {
                if (std::find(dw.conflictLinks.begin(), dw.conflictLinks.end(), link) == dw.conflictLinks.end()) {
                    dw.conflictLinks.push_back(link);
                }
            }
        }
    }
}

// MSDriveWay::match
bool Sim::dwMatch(const DriveWay& dw, const std::vector<int>& route, int firstIt) const {
    int itRoute = firstIt;
    const int endIt = (int)route.size();
    size_t itDw = 0;
    bool match = true;
    while (itRoute != endIt && itDw != dw.route.size()) {
        if (route[itRoute] != dw.route[itDw]) {
            match = false;
            break;
        }
        itRoute++;
        itDw++;
    }
    if (match && itDw == dw.route.size()
            && (itRoute == endIt || dw.abortedBuild || dw.bidiEnded || dw.foundJump || dw.parent >= 0)) {
        if (itRoute != endIt) {
            const int next = route[itRoute];
            const int prev = dw.route.back();
            const int prevBidi = net.edges[prev].bidi;
            if (dw.foundJump && prevBidi != next && prevBidi >= 0
                    && rlConnected(prev, next, VC_RAIL_CLASSES & rtPermOf(prev))) {
                return false;
            }
            if (!dw.foundJump && prevBidi == next && prev == net.lanes[dw.forward.back()].edge) {
                return false;  // must not leave driveway via reversal
            }
            if (isInternal(dw.forward.back()) && rlNextNormal(dw.forward.back()) != route[itRoute]) {
                return false;  // part of a direct-control conflict and continues elsewhere
            }
        }
        return dw.parent < 0 || dwMatch(*driveWays[dw.parent], route, firstIt);
    }
    return false;
}

SVCPermissions Sim::rtPermOf(int edge) const {
    SVCPermissions p = 0;
    for (int l : net.edges[edge].lanes) {
        p |= net.lanes[l].permissions;
    }
    return p;
}

// ---------------------------------------------------------------------------
// occupation (MSDriveWay::foeDriveWayOccupied, reserve, hasLinkConflict)
// ---------------------------------------------------------------------------
std::pair<bool, int> Sim::dwCanUseSiding(const DriveWay& dw, int ego, const DriveWay& foe, int recurseSidingEnd) const {
    dwSpecRead(dw.index);
    dwSpecRead(foe.index);
    auto it = dw.sidings.find(foe.index);
    if (it != dw.sidings.end()) {
        for (const Siding& siding : it->second) {
            // assume siding is usable when computing state for unapproached signal (ego < 0)
            if (ego < 0 || siding.length >= vt(ego).length) {
                const int sidingEnd = dw.route[siding.end];
                bool checkNext = false;
                auto eit = endingDriveways.find(sidingEnd);
                const std::vector<int> empty;
                const std::vector<int>& approaches = eit == endingDriveways.end() ? empty : eit->second;
                for (int sa : approaches) {
                    const DriveWay& sidingApproach = *driveWays[sa];
                    if (!sidingApproach.trains.empty() && *sidingApproach.trains.begin() == ego && sidingEnd == recurseSidingEnd) {
                        checkNext = true;
                        continue;
                    }
                    if (!sidingApproach.trains.empty() && *sidingApproach.trains.begin() != ego) {
                        // possibly the foe vehicle can use the other part of the siding
                        if (recurseSidingEnd < 0) {
                            int foeVeh = -1;
                            if (!foe.trains.empty()) {
                                foeVeh = *foe.trains.begin();
                            } else if (foe.origin >= 0 && !approaching[foe.origin].empty()) {
                                foeVeh = closestApproach(foe.origin).veh;
                            }
                            if (foeVeh < 0) {
                                continue;
                            }
                            const DriveWay& foe2 = foe.parent >= 0 ? *driveWays[foe.parent] : foe;
                            const int this2 = dwFoeOrSubFoe(foe2, dw.index);
                            if (this2 >= 0 && dwCanUseSiding(foe2, foeVeh, *driveWays[this2], sidingEnd).first) {
                                continue;
                            }
                        }
                        return std::make_pair(false, sa);
                    }
                }
                // vehicles approaching intermediate driveways could also make the siding unusable
                for (int i : siding.intermediateEnds) {
                    const int intermediateEnd = dw.route[i];
                    auto iit = endingDriveways.find(intermediateEnd);
                    if (iit == endingDriveways.end()) {
                        continue;
                    }
                    for (int ia : iit->second) {
                        const DriveWay& intermediateApproach = *driveWays[ia];
                        if (!intermediateApproach.trains.empty()
                                && (*intermediateApproach.trains.begin() != ego || intermediateEnd == recurseSidingEnd)) {
                            const int onApproach = *intermediateApproach.trains.begin();
                            const std::vector<int>& r = routes[veh[onApproach].route];
                            if (std::find(r.begin() + veh[onApproach].routeIdx, r.end(), sidingEnd) == r.end()) {
                                continue;  // intermediate vehicle does not make use of the siding
                            }
                            if (recurseSidingEnd < 0) {
                                int foeVeh = -1;
                                if (!foe.trains.empty()) {
                                    foeVeh = *foe.trains.begin();
                                } else if (foe.origin >= 0 && !approaching[foe.origin].empty()) {
                                    foeVeh = closestApproach(foe.origin).veh;
                                }
                                if (foeVeh < 0) {
                                    continue;
                                }
                                const DriveWay& foe2 = foe.parent >= 0 ? *driveWays[foe.parent] : foe;
                                const int this2 = dwFoeOrSubFoe(foe2, dw.index);
                                if (this2 >= 0 && dwCanUseSiding(foe2, foeVeh, *driveWays[this2], intermediateEnd).first) {
                                    continue;
                                }
                            }
                            return std::make_pair(false, ia);
                        }
                    }
                }
                if (checkNext) {
                    continue;
                }
                return std::make_pair(true, -1);
            }
        }
    }
    return std::make_pair(false, -1);
}

// MSDriveWay::hasJoin: the foe is the stopped train that ego joins, or the stopped train that joins ego
bool Sim::railHasJoin(int ego, int foe) const {
    if (ego < 0) {
        return false;
    }
    const Veh& x = veh[ego];
    std::string joinVehicle = x.stops.empty() ? "" : x.stops.front().join;
    if (joinVehicle.empty() && x.status == VehStatus::PENDING && x.stops.size() > 1) {
        joinVehicle = x.stops[1].join;  // check one more stop
    }
    if (!joinVehicle.empty() && veh[foe].id == joinVehicle && isStopped(foe)) {
        return true;
    }
    return isStopped(foe) && veh[foe].stops.front().join == x.id;
}

bool Sim::dwFoeOccupied(const DriveWay& dw, int ego, std::vector<int>* occupied) {
    dwSpecRead(dw.index);
    // (no occupied foe: only departure driveways need to look at the foes; an occupied foe without ego and
    // without sidings to evade it: occupied)
    if (ego < 0 && occupied == nullptr && dwHot[dw.index].occFoes > 0 && dw.sidings.empty()) {
        return true;
    }
    const bool scan = dwHot[dw.index].occFoes > 0 || dw.origin < 0;
    for (int fi : dw.foes) {
        if (!scan) {
            break;
        }
        const DwHot& foeHot = dwHot[fi];
        if (foeHot.nTrains == 0 && (dw.origin >= 0 || fi == dw.index || foeHot.origin < 0 || approaching[foeHot.origin].empty())) {
            continue;  // (the common case, without touching the driveway)
        }
        const DriveWay& foeDW = *driveWays[fi];
        if (!foeDW.trains.empty()) {
            if (foeDW.trains.size() == 1 && *foeDW.trains.begin() == ego) {
                continue;  // ignore ego as foe
            }
            if (foeDW.trains.size() == 1 && haveJoinSplit && railHasJoin(ego, *foeDW.trains.begin())) {
                continue;
            }
            const std::pair<bool, int> useSiding = dwCanUseSiding(dw, ego, foeDW, -1);
            if (useSiding.first) {
                continue;
            }
            for (int foe : foeDW.trains) {
                if (occupied != nullptr) {
                    const int e = routes[veh[foe].route][veh[foe].routeIdx];
                    occupied->push_back(e);
                    if (net.edges[e].bidi >= 0) {
                        occupied->push_back(net.edges[e].bidi);
                    }
                }
            }
            if (ego >= 0 && opt.timeToTeleportRSDeadlock > 0
                    && (veh[ego].waitingTime > 0 || veh[ego].status != VehStatus::RUNNING)) {
                // if there is an occupied siding, it becomes part of the wait relation
                const DriveWay& blocker = useSiding.second < 0 ? foeDW : *driveWays[useSiding.second];
                const int rs = dw.origin >= 0 ? tlsRail[net.links[dw.origin].tls] : -1;
                if (rsSpec != nullptr) {
                    rsSpec->waits.push_back({ego, RailWaitRelation{rs, *blocker.trains.begin(), -1}});
                } else {
                    railWaitRelations[ego] = RailWaitRelation{rs, *blocker.trains.begin(), -1};
                }
            }
            return true;
        } else if (fi != dw.index && dw.origin < 0 && foeDW.origin >= 0) {
            // departure driveway: foes approaching the signal of their driveway that cannot brake anymore
            if (!approaching[foeDW.origin].empty()) {
                const Approach& foeA = closestApproach(foeDW.origin);
                const int foe = foeA.veh;
                const VType& ft = vt(foe);
                if (foeA.dist < cfBrakeGap(foe, kin[foe].speed, ft.decel, ft.tau)) {
                    const std::vector<int>& fr = routes[veh[foe].route];
                    auto firstIt = std::find(fr.begin() + veh[foe].routeIdx, fr.end(), foeDW.route.front());
                    if (firstIt != fr.end() && dwMatch(foeDW, fr, (int)(firstIt - fr.begin()))) {
                        if (dwCanUseSiding(dw, ego, foeDW, -1).first) {
                            continue;
                        }
                        return true;
                    }
                }
            }
        }
    }
    for (const std::set<int>& dlFoes : dw.deadlocks) {
        bool allOccupied = true;
        for (int dlFoe : dlFoes) {
            if (driveWays[dlFoe]->trains.empty()) {
                allOccupied = false;
                break;
            }
        }
        if (allOccupied) {
            return true;
        }
    }
    return false;
}

const Approach& Sim::closestApproach(int link) const {
    const std::vector<Approach>& a = approaching[link];
    size_t best = 0;
    double minDist = DMAX;
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i].dist < minDist) {
            minDist = a[i].dist;
            best = i;
        }
    }
    return a[best];
}

bool Sim::dwFoeApproached(const DriveWay& dw) const {
    dwSpecRead(dw.index);
    for (int fi : dw.foes) {
        const int origin = dwHot[fi].origin;
        if (origin >= 0 && !approaching[origin].empty()) {
            return true;
        }
    }
    return false;
}

// MSDriveWay::mustYield
bool Sim::dwMustYield(const Approach& ego, const Approach& foe) const {
    if (foe.arrivalSpeedBraking == ego.arrivalSpeedBraking) {
        if (foe.arrivalTime == ego.arrivalTime) {
            if (kin[foe.veh].speed == kin[ego.veh].speed) {
                if (foe.dist == ego.dist) {
                    if (veh[foe.veh].waitingTime == veh[ego.veh].waitingTime) {
                        return foe.veh < ego.veh;
                    }
                    return veh[foe.veh].waitingTime > veh[ego.veh].waitingTime;
                }
                return foe.dist < ego.dist;
            }
            return kin[foe.veh].speed > kin[ego.veh].speed;
        }
        return foe.arrivalTime < ego.arrivalTime;
    }
    return foe.arrivalSpeedBraking > ego.arrivalSpeedBraking;
}

bool Sim::dwHasLinkConflict(const DriveWay& dw, const Approach& ego, int foeLink) {
    if (approaching[foeLink].empty()) {
        return false;
    }
    const Approach foe = closestApproach(foeLink);
    if (foe.veh == ego.veh) {
        return false;
    }
    const Link& fk = net.links[foeLink];
    if (fk.tls < 0 || net.tls[fk.tls].type != "rail_signal") {
        return false;
    }
    const int foeRS = tlsRail[fk.tls];
    DriveWay& foeDriveWay = *driveWays[rsGetDriveWay(foeRS, fk.tlIndex, foe.veh, -1)];
    const int foeOrSub = dwFoeOrSubFoe(dw, foeDriveWay.index);
    if (dwFoeOccupied(foeDriveWay, foe.veh, nullptr) || !constraintsAllow(foeRS, foe.veh, false) || !dwOverlap(dw, foeDriveWay) || foeOrSub < 0
            || dwCanUseSiding(dw, ego.veh, *driveWays[foeOrSub], -1).first) {
        return false;
    }
    return dwMustYield(ego, foe);
}

bool Sim::dwReserve(DriveWay& dw, const Approach& closest, std::vector<int>& occupied) {
    dwSpecRead(dw.index);
    if (dwFoeOccupied(dw, closest.veh, &occupied)) {
        return false;
    }
    const std::vector<int> conflictLinks = dw.conflictLinks;
    for (int foeLink : conflictLinks) {
        if (dwHasLinkConflict(*driveWays[dw.index], closest, foeLink)) {
            return false;
        }
    }
    if (rsSpec != nullptr) {
        rsSpec->activeWrites.push_back({dw.index, closest.veh});
    } else {
        driveWays[dw.index]->active = closest.veh;
    }
    return true;
}

// ---------------------------------------------------------------------------
// rail signals (MSRailSignal)
// ---------------------------------------------------------------------------
void Sim::initRailSignals() {
    tlsRail.assign(net.tls.size(), -1);
    railSignals.clear();
    railSignalized = opt.railSignalizedClasses;
    for (int t = 0; t < (int)net.tls.size(); ++t) {
        const TLSDef& d = net.tls[t];
        if (d.type != "rail_signal") {
            continue;
        }
        RailSignal rs;
        rs.tls = t;
        rs.id = d.id;
        SVCPermissions outgoing = 0;
        for (int i = 0; i < (int)d.links.size(); ++i) {
            if (d.links[i].size() != 1) {
                throw std::runtime_error("At railSignal '" + d.id + "' found " + std::to_string(d.links[i].size())
                                         + " links controlled by index " + std::to_string(i));
            }
            RailLinkInfo li;
            li.link = d.links[i][0];
            const Link& k = net.links[li.link];
            li.controlled = (net.lanes[k.viaOrLane()].permissions & net.lanes[k.lane].permissions & VC_RAIL_CLASSES) != 0;
            rs.links.push_back(li);
            outgoing |= k.permissions;
            // MSRailSignalControl::addSignal: moving-block classes on signalized tracks are signalized
            railSignalized |= k.permissions & opt.railMovingBlockClasses;
        }
        rs.movingBlock = opt.railMovingBlock || isMovingBlockClass(outgoing);
        const std::string mb = d.param("moving-block", "");  // MSRailSignal::setParameter (additional files)
        if (!mb.empty()) {
            rs.movingBlock = mb == "true" || mb == "1" || mb == "yes" || mb == "on" || mb == "x";
        }
        tlsRail[t] = (int)railSignals.size();
        railSignals.push_back(rs);
    }
    for (int ri = 0; ri < (int)railSignals.size(); ++ri) {
        rsUpdateCurrentPhase(ri);
        rsSetSignals(ri);
    }
    // deadlock hints (NLHandler::addDeadlock, MSRailSignalControl::addDeadlockCheck)
    railDeadlockChecks.clear();
    for (const std::vector<std::string>& ids : railDeadlockLines) {
        std::vector<int> signals;
        for (const std::string& id : ids) {
            auto it = net.tlsIdx.find(id);
            if (it == net.tlsIdx.end() || tlsRail[it->second] < 0) {
                throw std::runtime_error("Rail signal '" + id + "' in deadlock is not known");
            }
            signals.push_back(tlsRail[it->second]);
        }
        const int n = (int)signals.size();
        for (int i = 0; i < n; i++) {
            std::vector<int> others;
            for (int j = 0; j < n; j++) {
                others.push_back(signals[(i + j + 1) % n]);
            }
            railDeadlockChecks[signals[i]] = others;
        }
    }
    initRailConstraints();
}

// MSRailSignalControl::isMovingBlock
bool Sim::isMovingBlockClass(SVCPermissions svc) const {
    return (opt.railMovingBlockClasses & svc & VC_RAIL_CLASSES) == (svc & VC_RAIL_CLASSES);
}

void Sim::rsSetSignals(int ri) {
    RailSignal& rs = railSignals[ri];
    for (int i = 0; i < (int)rs.links.size(); ++i) {
        Link& k = net.links[rs.links[i].link];
        const char ls = rs.state[i];
        if (k.state != ls) {
            k.lastStateChange = now;
        }
        k.state = ls;
        if (k.haveGreen()) {
            k.lastGreenState = k.state;
        }
    }
}

// MSRailSignal::LinkInfo::getDriveWay(veh, searchStart)
int Sim::rsGetDriveWay(int ri, int tlIndex, int v, int searchStart) {
    RailLinkInfo& li = railSignals[ri].links[tlIndex];
    const int first = net.lanes[net.links[li.link].lane].edge;
    const std::vector<int>& route = dwRoute(v);
    int firstIt = (int)route.size();
    for (int i = searchStart < 0 ? veh[v].routeIdx : searchStart; i < (int)route.size(); ++i) {
        if (route[i] == first) {
            firstIt = i;
            break;
        }
    }
    if (firstIt == (int)route.size()) {
        // possibly the vehicle has already gone past the first edge: look backward along the route
        double lookBack = SPEED2DIST(kin[v].speed + 10);
        int routeIndex = veh[v].routeIdx - 1;
        while (lookBack > 0 && routeIndex > 0) {
            if (route[routeIndex] == first) {
                firstIt = routeIndex;
                break;
            }
            lookBack -= net.edges[route[routeIndex]].length;
            routeIndex--;
        }
    }
    if (firstIt == (int)route.size() && rsSpec != nullptr) {
        rsSpec->abort = true;  // (evaluated in order)
        return li.driveways.empty() ? 0 : li.driveways.front();
    }
    if (firstIt == (int)route.size()) {
        std::fprintf(stderr, "Warning: Invalid approach information to rail signal '%s' after rerouting for vehicle '%s'.\n",
                     railSignals[ri].id.c_str(), veh[v].id.c_str());
        if (li.driveways.empty()) {
            const std::vector<int> dummy{first};
            const int dw = buildDriveWay(rsNewDriveWayID(ri), li.link, dummy, 0);
            railSignals[ri].links[tlIndex].driveways.push_back(dw);
            rsMarkTouched(ri);
        }
        return railSignals[ri].links[tlIndex].driveways.front();
    }
    // the first matching driveway of the link stays the same while the route ahead is the same (driveways are only
    // appended)
    uint64_t h = 1469598103934665603ULL;
    for (int i = firstIt; i < (int)route.size(); ++i) {
        h = (h ^ (uint64_t)(uint32_t)route[i]) * 1099511628211ULL;
    }
    if (v >= (int)dwLookup.size()) {
        dwLookup.resize(veh.size());
    }
    const int tailLen = (int)route.size() - firstIt;
    for (const DwLookup& c : dwLookup[v]) {
        if (c.link == li.link && c.firstIt == firstIt && c.tailLen == tailLen && c.tailHash == h) {
            return c.dw;
        }
    }
    if (rsSpec != nullptr) {
        // (no building and no cache update in the speculative evaluation)
        for (int dw : li.driveways) {
            if (dwMatch(*driveWays[dw], route, firstIt)) {
                return dw;
            }
        }
        rsSpec->abort = true;
        return li.driveways.empty() ? 0 : li.driveways.front();
    }
    const int dw = rsGetDriveWayByRoute(ri, tlIndex, route, firstIt, veh[v].id);
    std::array<DwLookup, DW_LOOKUPS>& cache = dwLookup[v];
    std::move_backward(cache.begin(), cache.end() - 1, cache.end());  // (the oldest entry is dropped)
    cache[0] = DwLookup{railSignals[ri].links[tlIndex].link, firstIt, tailLen, dw, h};
    return dw;
}

int Sim::rsGetDriveWayByRoute(int ri, int tlIndex, const std::vector<int>& route, int firstIt, const std::string& info) {
    for (int dw : railSignals[ri].links[tlIndex].driveways) {
        if (dwMatch(*driveWays[dw], route, firstIt)) {
            return dw;
        }
    }
    const int dw = buildDriveWay(rsNewDriveWayID(ri), railSignals[ri].links[tlIndex].link, route, firstIt);
    driveWays[dw]->firstVehicle = info;
    rsMarkTouched(ri);
    railSignals[ri].links[tlIndex].driveways.push_back(dw);
    return dw;
}

std::string Sim::rsNewDriveWayID(int ri) {
    RailSignal& rs = railSignals[ri];
    return rs.id + "." + std::to_string(rs.driveWayIndex++);
}

// MSRailSignal::LinkInfo::reroute: the train waiting for occupied foe driveways looks for another route
// around them (each train once, or with the rerouting period)
void Sim::rsReroute(int ri, int linkIndex, int v, const std::vector<int>& occupied) {
    RailLinkInfo& li = railSignals[ri].links[linkIndex];
    Veh& x = veh[v];
    const double period = x.periodParam >= 0 ? x.periodParam : opt.reroutePeriod;
    if (!x.routingDevice || !opt.rerouteRailSignal
            || (li.lastRerouteVehicle == v && !(period > 0 && li.lastRerouteTime + TIME2STEPS(period) <= now))) {
        return;
    }
    li.lastRerouteVehicle = v;
    li.lastRerouteTime = now;
    // indefinite occupation because the vehicles might be in a deadlock on their current routes
    std::vector<RouterProhibition> prohibited;
    for (int e : occupied) {
        prohibited.push_back({e, 0, std::numeric_limits<double>::infinity()});
    }
    thread_local RouteWS ws;
    ws.staticEffort = false;
    ws.prohibited = &prohibited;
    ws.queryTime = STEPS2TIME(now);
    reroute(v, false, ws, false, -1, true);  // MSRoutingEngine::reroute (silent)
    ws.prohibited = nullptr;
}

// MSRailSignal::updateCurrentPhase
bool Sim::rsUpdateCurrentPhase(int ri) {
    bool keepActive = false;
    RailSignal& rs = railSignals[ri];
    std::string state(rs.links.size(), 'G');  // green by default so vehicles can be inserted at the borders
    for (int i = 0; i < (int)rs.links.size(); ++i) {
        RailLinkInfo& li = railSignals[ri].links[i];
        const int tlIndex = net.links[li.link].tlIndex;
        if (!approaching[li.link].empty() && li.controlled) {
            keepActive = true;
            const Approach closest = closestApproach(li.link);
            DriveWay& driveway = *driveWays[rsGetDriveWay(ri, i, closest.veh, -1)];
            std::vector<int> occupied;
            const bool mustWait = !constraintsAllow(ri, closest.veh, true);
            if (mustWait || !dwReserve(driveway, closest, occupied)) {
                state[tlIndex] = 'r';
                if (!occupied.empty()) {
                    rsReroute(ri, i, closest.veh, occupied);
                }
            } else {
                state[tlIndex] = 'G';
            }
            static const char* traceRS = getenv("PSIM_TRACE_RS");
            if (traceRS != nullptr && rs.id == traceRS && rsSpec == nullptr) {
                std::fprintf(stderr, "%.1f rs %s link %d veh %s dw %s -> %c foes:", STEPS2TIME(now), rs.id.c_str(), i,
                             veh[closest.veh].id.c_str(), driveway.id.c_str(), state[tlIndex]);
                for (int fi : driveWays[driveway.index]->foes) {
                    if (!driveWays[fi]->trains.empty()) {
                        std::fprintf(stderr, " %s(%s)", driveWays[fi]->id.c_str(), veh[*driveWays[fi]->trains.begin()].id.c_str());
                    }
                }
                std::fprintf(stderr, "\n");
            }
        } else if (li.controlled) {
            if (li.driveways.empty()) {
                state[tlIndex] = 'r';  // uninitialized signal (no driveways yet)
            } else {
                const DriveWay& driveway = *driveWays[li.driveways.front()];
                if (dwFoeOccupied(driveway, -1, nullptr) || dwFoeApproached(driveway)) {
                    keepActive = true;
                    state[tlIndex] = 'r';
                }
                static const char* traceRS = getenv("PSIM_TRACE_RS");
                if (traceRS != nullptr && rs.id == traceRS && rsSpec == nullptr) {
                    std::fprintf(stderr, "%.1f rs %s link %d unapproached dw %s -> %c foes:", STEPS2TIME(now), rs.id.c_str(), i,
                                 driveway.id.c_str(), state[tlIndex]);
                    for (int fi : driveway.foes) {
                        if (!driveWays[fi]->trains.empty()) {
                            std::fprintf(stderr, " %s(%s)", driveWays[fi]->id.c_str(), veh[*driveWays[fi]->trains.begin()].id.c_str());
                        } else if (driveWays[fi]->origin >= 0 && !approaching[driveWays[fi]->origin].empty()) {
                            const Approach& a = approaching[driveWays[fi]->origin].front();
                            std::fprintf(stderr, " %s(approached by %s dist=%.1f from %s)", driveWays[fi]->id.c_str(),
                                         veh[a.veh].id.c_str(), a.dist, net.lanes[net.links[driveWays[fi]->origin].laneBefore].id.c_str());
                        }
                    }
                    std::fprintf(stderr, "\n");
                }
            }
        } else {
            state[tlIndex] = 'O';
        }
    }
    if (rsSpec != nullptr) {
        rsSpec->state = state;
    } else {
        railSignals[ri].state = state;
    }
    return keepActive;
}

thread_local Sim::RsSpec* Sim::rsSpec = nullptr;

void Sim::rsMarkTouched(int ri) {
    if (rsCommitting && !rsTouched[ri]) {
        rsTouched[ri] = 1;
        rsTouchedList.push_back(ri);
    }
}

// whether the rail signals may be evaluated speculatively (features whose evaluation writes state are excluded)
bool Sim::rsSpeculate() const {
    static const bool force = getenv("PSIM_RS_SPEC") != nullptr;  // (exercises the path with one thread)
    static const bool trace = getenv("PSIM_TRACE_RS") != nullptr;
    return (pool.size() > 1 || force) && !trace && activeRailSignals.size() >= (force ? 1u : 8u) && !driveWays.empty()
           && railConstraints.empty() && railDeadlockChecks.empty() && !opt.rerouteRailSignal;
}

// MSRailSignalControl::updateSignals (begin of step, after the begin-of-step events)
void Sim::updateRailSignals() {
    if (railSignals.empty()) {
        return;
    }
    if (rsSpeculate()) {
        const std::vector<int> order(activeRailSignals.begin(), activeRailSignals.end());
        nRsUpdates += (long long)order.size();
        if (dwLookup.size() < veh.size()) {
            dwLookup.resize(veh.size());
        }
        rsTouched.resize(railSignals.size(), 0);
        rsSpecResults.resize(std::max(rsSpecResults.size(), order.size()));
        pool.setTag(ThreadPool::T_OTHER);
        pool.parallel_for((int)order.size(), [&](int i) {
            RsSpec& r = rsSpecResults[i];
            r.abort = false;
            r.reads.clear();
            r.activeWrites.clear();
            r.waits.clear();
            rsSpec = &r;
            r.keep = rsUpdateCurrentPhase(order[i]);
            rsSpec = nullptr;
        }, 4);
        // in SUMO's order: builds of driveways happen here
        rsCommitting = true;
        for (int i = 0; i < (int)order.size(); ++i) {
            const int ri = order[i];
            const RsSpec& r = rsSpecResults[i];
            bool valid = !r.abort && !rsTouched[ri];
            if (valid && !dwTouchedList.empty()) {
                for (int d : r.reads) {
                    if (dwTouched[d]) {
                        valid = false;
                        break;
                    }
                }
            }
            bool keepActive;
            nRsSpeculative++;
            if (valid) {
                railSignals[ri].state = r.state;
                for (const auto& aw : r.activeWrites) {
                    driveWays[aw.first]->active = aw.second;
                }
                for (const auto& w : r.waits) {
                    railWaitRelations[w.first] = w.second;
                }
                keepActive = r.keep;
            } else {
                nRsReevaluated++;
                keepActive = rsUpdateCurrentPhase(ri);
            }
            rsSetSignals(ri);
            if (!keepActive) {
                activeRailSignals.erase(ri);
            }
        }
        rsCommitting = false;
        for (int d : dwTouchedList) {
            dwTouched[d] = 0;
        }
        dwTouchedList.clear();
        for (int ri : rsTouchedList) {
            rsTouched[ri] = 0;
        }
        rsTouchedList.clear();
        return;
    }
    nRsUpdates += (long long)activeRailSignals.size();
    for (auto it = activeRailSignals.begin(); it != activeRailSignals.end();) {
        const int ri = *it;
        const bool keepActive = rsUpdateCurrentPhase(ri);
        rsSetSignals(ri);
        if (!keepActive) {
            it = activeRailSignals.erase(it);
        } else {
            ++it;
        }
    }
}

void Sim::railNotifyApproach(int link) {
    const Link& k = net.links[link];
    if (k.tls >= 0 && tlsRail[k.tls] >= 0) {
        static const char* traceRS = getenv("PSIM_TRACE_RS");
        if (traceRS != nullptr && railSignals[tlsRail[k.tls]].id == traceRS && activeRailSignals.count(tlsRail[k.tls]) == 0) {
            std::fprintf(stderr, "%.1f rs %s activated via link from %s\n", STEPS2TIME(now), traceRS, net.lanes[k.laneBefore].id.c_str());
        }
        activeRailSignals.insert(tlsRail[k.tls]);
    }
}

// MSDriveWay::enterDriveWay
void Sim::dwEnter(int dwi, int v, int reason) {
    DriveWay& dw = *driveWays[dwi];
    dwTrainAdd(dwi, v);
    dwEvent(dwi, v, true, reason);
    if (dw.origin >= 0) {
        railNotifyApproach(dw.origin);
    }
    for (int fi : dw.foes) {
        if (driveWays[fi]->origin >= 0) {
            railNotifyApproach(driveWays[fi]->origin);
        }
    }
}

// MSDriveWay::getDepartureDriveway
int Sim::departureDriveWay(int v, bool init) {
    const std::vector<int>& route = dwRoute(v);
    const int departIdx = init ? std::min(veh[v].departEdge, (int)route.size() - 1) : veh[v].routeIdx;
    const int edge = route[departIdx];
    const int fromJ = net.edges[edge].fromJ;
    if (fromJ >= 0 && net.junctions[fromJ].type == "rail_signal") {
        for (int lane : net.edges[edge].lanes) {
            for (const IncomingLane& ili : net.lanes[lane].incoming) {
                const int entry = rlEntryLink(ili.link);
                const Link& k = net.links[entry];
                if (k.tls >= 0 && tlsRail[k.tls] >= 0) {
                    const int dw = rsGetDriveWay(tlsRail[k.tls], k.tlIndex, v, -1);
                    if (net.lanes[driveWays[dw]->forward.front()].edge == edge) {
                        return dw;
                    }
                }
            }
        }
    }
    for (int dw : departureDriveways[edge]) {
        if (dwMatch(*driveWays[dw], route, departIdx)) {
            return dw;
        }
    }
    const std::string id = net.junctions[fromJ].id + ".d" + std::to_string(departDrivewayIndex[fromJ]++);
    const int dw = buildDriveWay(id, -1, route, veh[v].routeIdx);
    departureDriveways[edge].push_back(dw);
    departureDrivewaysEnds[net.lanes[driveWays[dw]->forward.back()].edge].push_back(dw);
    driveWays[dw]->firstVehicle = veh[v].id;
    return dw;
}

// MSRailSignal::initDriveWays (when a train is built, update: after rerouting)
void Sim::initDriveWays(int v, bool update) {
    if (railSignals.empty()) {
        return;
    }
    const std::vector<int>& edges = dwRoute(v);
    const int endIndex = (int)routes[veh[v].route].size() - 1;  // (getParameter().arrivalEdge)
    int departIndex = std::min(veh[v].departEdge, endIndex);  // (getParameter().departEdge)
    int prev = -1;
    if (update && veh[v].status != VehStatus::PENDING) {
        // find last rail signal on the route and obtain the driveway
        int next = edges[veh[v].routeIdx];
        for (int i = veh[v].routeIdx - 1; i > departIndex; i--) {
            const int e = edges[i];
            if (rlEdgeIsRailSignal(e)) {
                const int link = rlLinkTo(net.edges[e].lanes.front(), net.edges[next].lanes.front());
                if (link >= 0 && net.links[link].tls >= 0) {
                    const int ri = tlsRail[net.links[link].tls];
                    if (ri >= 0) {
                        prev = rsGetDriveWay(ri, net.links[link].tlIndex, v, i);
                        departIndex = veh[v].routeIdx;
                        break;
                    }
                }
            }
            next = e;
        }
    }
    if (prev < 0) {
        prev = departureDriveWay(v, true);
    }
    if (update && veh[v].status != VehStatus::PENDING) {
        if (driveWays[prev]->trains.count(v) == 0 && dwNotifyReroute(prev, v)) {
            if (std::find(veh[v].ex().dwHeld.begin(), veh[v].ex().dwHeld.end(), prev) == veh[v].ex().dwHeld.end()) {
                veh[v].ex().dwHeld.push_back(prev);
            }
        }
    }
    for (int i = departIndex; i <= endIndex - 1; i++) {
        const int e = edges[i];
        if (net.edges[e].isNormal() && rlEdgeIsRailSignal(e)) {
            const int e2 = edges[i + 1];
            for (int lane : net.edges[e].lanes) {
                for (int l : net.lanes[lane].links) {
                    if (net.lanes[net.links[l].lane].edge == e2 && net.links[l].tls >= 0 && tlsRail[net.links[l].tls] >= 0) {
                        const int ri = tlsRail[net.links[l].tls];
                        const int dw = rsGetDriveWay(ri, net.links[l].tlIndex, v, i);
                        dwSucc[prev].insert(dw);  // MSRailSignalControl::addDrivewayFollower
                        dwPred[dw].insert(prev);
                        railAddDWDeadlockChecks(ri, prev);
                        railNotifyApproach(l);
                        prev = dw;
                        if (update && veh[v].status == VehStatus::PENDING) {
                            // the vehicle may have rerouted its initial trip after the states have been set
                            rsUpdateCurrentPhase(ri);
                            rsSetSignals(ri);
                        } else if (update && i <= veh[v].routeIdx) {
                            if (driveWays[dw]->trains.count(v) == 0 && dwNotifyReroute(dw, v)) {
                                veh[v].ex().dwHeld.push_back(dw);
                                for (int sub : driveWays[dw]->subs) {
                                    if (driveWays[sub]->trains.count(v) == 0 && dwNotifyReroute(sub, v)) {
                                        veh[v].ex().dwHeld.push_back(sub);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    (void)prev;
}

// MSDriveWay::matchesPastRoute
int Sim::dwMatchesPastRoute(int dwi, int v) const {
    const DriveWay& dw = *driveWays[dwi];
    const std::vector<int>& r = dwRoute(v);
    const int first = rlNextNormal(dw.lane);
    for (int i = veh[v].routeIdx; i >= 0; i--) {
        if (r[i] == first) {
            if (dwMatch(dw, r, i)) {
                return veh[v].routeIdx - i;
            }
            break;
        }
    }
    return -1;
}

// notifyEnter with NOTIFICATION_REROUTE
bool Sim::dwNotifyReroute(int dwi, int v) {
    const int movedPast = dwMatchesPastRoute(dwi, v);
    if (movedPast >= 0 && movedPast < driveWays[dwi]->forwardEdgeCount) {
        dwEnter(dwi, v, DWR_REROUTE);
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// driveway move reminders (notifyEnter / notifyLeave / notifyLeaveBack)
// ---------------------------------------------------------------------------
// MSBaseVehicle::isRail
bool Sim::isRailVeh(int v) const {
    return isRailway(veh[v].vclass) || isRailway(rtPermOf(curEdge(v)));
}

// MSRailSignalControl::isUsingDriveWays: signalized and moving-block classes
bool Sim::usesDriveWays(int v) const {
    const SVCPermissions svc = veh[v].vclass;
    return ((railSignalized | opt.railMovingBlockClasses) & svc) == svc;
}

// MSRailSignalControl::isSignalized
bool Sim::isSignalizedVeh(int v) const {
    return (railSignalized & veh[v].vclass) == veh[v].vclass;
}

// the vehicle entered `lane` (departure, junction, parking) with route index routeIdx: driveways starting there
void Sim::dwLaneEntered(int v, int lane, int routeIdx, int reason, const std::vector<int>* route) {
    if (lane >= (int)laneDriveWays.size() || laneDriveWays[lane].empty() || !usesDriveWays(v)) {
        return;
    }
    Veh& x = veh[v];
    const std::vector<int>& r = route != nullptr ? *route : dwRoute(v);
    const std::vector<int> dws = laneDriveWays[lane];
    for (int dwi : dws) {
        const DriveWay& dw = *driveWays[dwi];
        int firstIt = (int)r.size();
        const int first = rlNextNormal(dw.lane);
        for (int i = routeIdx; i < (int)r.size(); ++i) {
            if (r[i] == first) {
                firstIt = i;
                break;
            }
        }
        if (dwMatch(dw, r, firstIt)) {
            if (dw.trains.count(v) == 0) {
                dwEnter(dwi, v, reason);
            }
            if (std::find(x.ex().dwHeld.begin(), x.ex().dwHeld.end(), dwi) == x.ex().dwHeld.end()) {
                x.ex().dwHeld.push_back(dwi);
            }
        }
    }
}

// the back of the vehicle left `lane` (notifyLeaveBack); back: the vehicle's back lane afterwards
void Sim::dwLaneLeftBack(int v, int lane, int back) {
    Veh& x = veh[v];
    if (x.exr().dwHeld.empty()) {
        return;
    }
    for (size_t i = 0; i < x.ex().dwHeld.size();) {
        DriveWay& dw = *driveWays[x.ex().dwHeld[i]];
        if (lane == dw.forward.back() && back != net.lanes[lane].bidi) {
            dwTrainRemove(x.ex().dwHeld[i], v);
            dwEvent(x.ex().dwHeld[i], v, false, DWR_JUNCTION);
            x.ex().dwHeld.erase(x.ex().dwHeld.begin() + (long)i);
        } else {
            ++i;
        }
    }
}

// the vehicle left the network (arrival, teleport, parking): notifyLeave with a reason other than junction
void Sim::dwLeaveAll(int v, int reason) {
    std::vector<int>& held = veh[v].ex().dwHeld;
    for (int dwi : held) {
        dwTrainRemove(dwi, v);
        dwEvent(dwi, v, false, reason);
    }
    held.clear();
}

// the driveway events recorded in the move phase (per lane task), applied in task order
void Sim::applyDriveWayEvents(std::vector<std::vector<DriveWayEvent>>& events) {
    for (std::vector<DriveWayEvent>& evs : events) {
        for (const DriveWayEvent& ev : evs) {
            if (ev.enter) {
                trackerEnter(ev.veh, ev.lane);
                const VehExtra& ex = veh[ev.veh].exr();
                dwLaneEntered(ev.veh, ev.lane, ev.aux, DWR_JUNCTION, ev.oldRoute && !ex.dwOldRoute.empty() ? &ex.dwOldRoute : nullptr);
            } else {
                dwLaneLeftBack(ev.veh, ev.lane, ev.aux);
            }
        }
        for (const DriveWayEvent& ev : evs) {
            if (ev.oldRoute) {
                veh[ev.veh].ex().dwOldRoute.clear();
            }
        }
        evs.clear();
    }
}

// ---------------------------------------------------------------------------
// insertion (MSLane::isInsertionSuccess, MSVehicle::registerInsertionApproach)
// ---------------------------------------------------------------------------
// the departure driveway of a signalized train has an occupied foe
bool Sim::railDepartBlocked(int v, int lane) {
    if (!isRailVeh(v) || !isSignalizedVeh(v) || !isRailwayOrShared(net.lanes[lane].permissions)) {
        return false;
    }
    const int dw = departureDriveWay(v, false);
    const bool blocked = dwFoeOccupied(*driveWays[dw], v, nullptr);
    if (blocked && getenv("PSIM_TRACE_RAIL")) {
        const DriveWay& d = *driveWays[dw];
        std::fprintf(stderr, "%.1f departBlocked %s dw=%s foes:", STEPS2TIME(now), veh[v].id.c_str(), d.id.c_str());
        for (int fi : d.foes) {
            std::fprintf(stderr, " %s(%d)", driveWays[fi]->id.c_str(), (int)driveWays[fi]->trains.size());
        }
        std::fprintf(stderr, "\n");
    }
    return blocked;
}

// the train would be inserted next to an occupied bidirectional lane (also along its back)
bool Sim::railBidiBlocked(int v, int lane, double pos) const {
    const int bidi = net.lanes[lane].bidi;
    if (bidi < 0 || !isRailVeh(v)) {
        return false;
    }
    if (laneVehicleNumberWithPartials(bidi) > 0) {
        return true;
    }
    double backLength = vt(v).length - pos;
    int pred = net.lanes[lane].logicalPred;
    int b = pred < 0 ? -1 : net.lanes[pred].bidi;
    while (backLength > 0 && b >= 0) {
        if (laneVehicleNumberWithPartials(b) > 0) {
            return true;
        }
        backLength -= net.lanes[b].length;
        pred = net.lanes[pred].logicalPred;
        b = pred < 0 ? -1 : net.lanes[pred].bidi;
    }
    return false;
}

// MSLink::railSignalWasPassed
bool Sim::railSignalWasPassed(int link) const {
    const Link& k = net.links[link];
    if (k.junction >= 0 && net.junctions[k.junction].type == "rail_signal") {
        for (const Approach& a : approaching[link]) {
            if (a.dist < SPEED2DIST(kin[a.veh].speed)) {
                return true;
            }
        }
    }
    return false;
}

// rail_signal junctions need the approach of an inserted train at the start of the next step
void Sim::registerInsertionApproach(int v, int link, double dist) {
    Approach ap;
    ap.veh = v;
    ap.arrivalTime = TIME_MAX;
    ap.leavingTime = TIME_MAX;
    ap.arrivalSpeed = 0;
    ap.leaveSpeed = 0;
    ap.willPass = false;
    ap.arrivalSpeedBraking = cf::getMinimalArrivalSpeedEuler(vt(v), dist, kin[v].speed);
    ap.waitingTime = 0;
    ap.dist = dist;
    ap.speed = kin[v].speed;
    ap.lane = kin[v].lane;
    std::vector<Approach>& a = approaching[link];
    auto it = std::lower_bound(a.begin(), a.end(), v, [](const Approach& x, int id) { return x.veh < id; });
    if (it != a.end() && it->veh == v) {
        *it = ap;
    } else {
        a.insert(it, ap);
    }
    approachMoved.push_back(link);  // removed when the approaches are rebuilt
    railNotifyApproach(link);
}


// ---------------------------------------------------------------------------
// simulation hooks
// ---------------------------------------------------------------------------
void Sim::initRail() {
    railActive = false;
    for (const Veh& x : veh) {
        if ((x.vclass & VC_RAIL_CLASSES) != 0) {
            railActive = true;
            break;
        }
    }
    initRailSignals();
    if (!railActive) {
        return;
    }
    // (the vehicles report themselves when they are built, see Sim::vehicleBuilt)
}

// MSRailSignalControl::vehicleStateChanged (BUILT): the driveways along the route (only when the network
// has rail signals: the control is created by the first signal)
void Sim::railVehicleBuilt(int v) {
    if (railSignals.empty() || !isRailVeh(v)) {
        return;
    }
    if (veh[v].forceReroute) {
        // MSRailSignalControl::vehicleStateChanged(BUILT): a trip only if its unrouted route (from, via, to) is
        // valid (MSBaseVehicle::hasValidRoute)
        const std::vector<int>& route = routes[veh[v].route];
        const SVCPermissions vc = veh[v].vclass;
        for (int i = veh[v].routeIdx; i < (int)route.size(); ++i) {
            if (i + 1 < (int)route.size()) {
                const std::vector<int>* allowed = net.allowedLanes(route[i], route[i + 1], vc);
                if (allowed == nullptr || allowed->empty()) {
                    return;
                }
            }
            if (!net.edgeAllows(route[i], vc)) {
                return;
            }
        }
    }
    if (isRailwayOrShared(rtPermOf(curEdge(v)))) {
        initDriveWays(v, false);
    }
}

// a route was replaced (MSNet::VehicleState::NEWROUTE); called from any thread
void Sim::railNewRoute(int v) {
    std::lock_guard<std::mutex> lock(railMutex);
    railNewRoutes.push_back(v);
}

// MSRailSignalControl::vehicleStateChanged (NEWROUTE), in vehicle order; departed trains check their
// driveway reminders at the start of the next step (MSBaseVehicle::activateRemindersOnReroute)
void Sim::processRailNewRoutes() {
    if (railNewRoutes.empty()) {
        return;
    }
    // (the pre-insertion reroutes in the order of their events, then by vehicle)
    auto rank = [&](int v) { return veh[v].preRankStep == now ? veh[v].preRank : std::numeric_limits<int>::max(); };
    std::sort(railNewRoutes.begin(), railNewRoutes.end(), [&](int a, int b) {
        return rank(a) != rank(b) ? rank(a) < rank(b) : a < b;
    });
    railNewRoutes.erase(std::unique(railNewRoutes.begin(), railNewRoutes.end()), railNewRoutes.end());
    for (int v : railNewRoutes) {
        if (!railSignals.empty() && isRailVeh(v) && isRailwayOrShared(rtPermOf(curEdge(v)))) {
            initDriveWays(v, true);
        }
        if (veh[v].status != VehStatus::PENDING && isRailVeh(v)) {
            railRerouted.push_back(v);
        }
    }
    railNewRoutes.clear();
}

void Sim::railBeginStep() {
    processRailNewRoutes();
    for (int v : railRerouted) {
        Veh& x = veh[v];
        // MSDriveWay::notifyReroute
        for (size_t i = 0; i < x.ex().dwHeld.size();) {
            if (usesDriveWays(v) && dwMatchesPastRoute(x.ex().dwHeld[i], v) < 0) {
                dwTrainRemove(x.ex().dwHeld[i], v);
                dwEvent(x.ex().dwHeld[i], v, false, DWR_REROUTE);
                x.ex().dwHeld.erase(x.ex().dwHeld.begin() + (long)i);
            } else {
                ++i;
            }
        }
        // MSVehicle::resetApproachOnReroute
        if (x.status == VehStatus::RUNNING) {
            bool diverged = false;
            const std::vector<int>& route = routes[x.route];
            int ri = x.routeIdx;
            for (const DriveItem& d : x.lf) {
                if (d.link < 0) {
                    continue;
                }
                if (!diverged) {
                    if (ri + 1 >= (int)route.size() || net.lanes[net.links[d.link].lane].edge != route[ri + 1]) {
                        diverged = true;
                    } else if (net.links[d.link].via < 0) {
                        ri++;
                    }
                }
                if (diverged) {
                    std::vector<Approach>& a = approaching[d.link];
                    a.erase(std::remove_if(a.begin(), a.end(), [&](const Approach& ap) { return ap.veh == v; }), a.end());
                }
            }
        }
    }
    railRerouted.clear();
    updateRailSignals();
}

// MSVehicle::setApproachingForAllLinks: trains activate the rail signals they approach
void Sim::railApproaches() {
    if (railSignals.empty()) {
        return;
    }
    for (int v : running) {
        if (veh[v].status != VehStatus::RUNNING || (veh[v].vclass & VC_RAIL_CLASSES) == 0 || !isRailVeh(v)) {
            continue;
        }
        for (const DriveItem& d : veh[v].lf) {
            if (d.link >= 0 && net.links[d.link].tls >= 0 && tlsRail[net.links[d.link].tls] >= 0) {
                railNotifyApproach(d.link);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// reversal (MSVehicle::checkReversal)
// ---------------------------------------------------------------------------
// the speed that lets the train reverse on the bidirectional edge that follows on its route (vehMaxSpeed
// when it cannot); canReverse: it may reverse now
double Sim::checkReversal(int v, bool& canReverse, double speedThreshold, double seen) const {
    const Veh& x = veh[v];
    const Kin& k = kin[v];
    const std::vector<int>& route = routes[x.route];
    const bool stopOk = x.stops.empty() || x.stops.front().routeIdx != x.routeIdx
                        || (x.stops.front().speed > 0 && k.pos > x.stops.front().endPos - 2 * POSITION_EPS);  // a passed waypoint
    if ((x.vclass & VC_RAIL_CLASSES) == 0 || x.prevSpeed > speedThreshold || k.pos > net.lanes[k.lane].length || isInternal(k.lane)
            || x.routeIdx + 1 >= (int)route.size() || net.edges[net.lanes[k.lane].edge].bidi != route[x.routeIdx + 1] || !stopOk) {
        return vehMaxSpeed(v);
    }
    const VType& t = vt(v);
    // the train must be fully on bidirectional edges that allow the reversal
    // Routes contain normal edges; internal junction lanes do not consume route entries.
    int neededFutureRoute = 1;
    for (int i = 0; i < k.nFurther; ++i) {
        if (!isInternal(k.further[i])) ++neededFutureRoute;
    }
    const int remainingRoute = (int)route.size() - x.routeIdx - 1;
    if (remainingRoute < neededFutureRoute) {
        return vehMaxSpeed(v);
    }
    // the turn-around connection from the current edge to its bidi edge must exist
    const Edge& cur = net.edges[net.lanes[k.lane].edge];
    if (std::find(cur.successors.begin(), cur.successors.end(), cur.bidi) == cur.successors.end()) {
        return vehMaxSpeed(v);
    }
    // the front must not move past a stop on the bidi edge of the current edge
    if (!x.stops.empty() && x.stops.front().routeIdx == x.routeIdx + 1) {
        const double stopPos = stopEndPos(v, x.stops.front());
        const double brakeDist = cfBrakeGap(v, k.speed, t.decel, 0);
        const double newPos = net.lanes[k.lane].length - (backPositionOnLane(v, k.lane) + brakeDist);
        if (newPos > stopPos && seen > std::max(brakeDist, 1.0)) {
            return vehMaxSpeed(v);
        }
    }
    // bidi edges for all further edges, no stops skipped, no red rail signal on top of the train afterwards
    int bidi = net.lanes[k.lane].bidi;
    int view = 2;
    for (int i = 0; i < k.nFurther; ++i) {
        const int further = k.further[i];
        if (isInternal(further)) {
            continue;
        }
        if (x.routeIdx + view >= (int)route.size() || net.edges[net.lanes[further].edge].bidi != route[x.routeIdx + view]) {
            return vehMaxSpeed(v);
        }
        const int nextBidi = net.lanes[further].bidi;
        const Link* toNext = bidi < 0 || nextBidi < 0 ? nullptr : net.linkTo(bidi, nextBidi);
        if (toNext == nullptr || toNext->haveRed()) {
            return vehMaxSpeed(v);
        }
        bidi = nextBidi;
        if (!x.stops.empty() && x.stops.front().routeIdx == x.routeIdx + view) {
            const double brakeDist = cfBrakeGap(v, k.speed, t.decel, 0);
            const double stopPos = stopEndPos(v, x.stops.front());
            const double newPos = net.lanes[further].length - (backPositionOnLane(v, further) + brakeDist);
            if (newPos > stopPos && seen > std::max(brakeDist, 1.0)) {
                canReverse = false;
                return vehMaxSpeed(v);
            }
        }
        view++;
    }
    // reverse as soon as comfortably possible
    canReverse = true;
    return cfMinNextSpeed(v, k.speed);
}

// ---------------------------------------------------------------------------
// the Rail car-following model (MSCFModel_Rail constructor)
// ---------------------------------------------------------------------------
// RAILTYPE id trainType key=value...: flags lengthSet, maxSpeedSet, decelSet, emergencyDecelSet, massSet, the
// values mass, massFactor, maxPower, maxTraction, resCoef_*, curveResistance, curveIntegration, roeckl_*,
// speedTable, tractionTable, resistanceTable (comma separated)
void Sim::setRailType(VType& t, const std::vector<std::string>& tok) {
    using namespace railtypes;
    const std::string& trainType = tok[2];
    TrainParams p;
    if (trainType == "RB425") {
        p = initRB425Params();
    } else if (trainType == "RB628") {
        p = initRB628Params();
    } else if (trainType == "NGT400") {
        p = initNGT400Params();
    } else if (trainType == "NGT400_16") {
        p = initNGT400_16Params();
    } else if (trainType == "ICE1") {
        p = initICE1Params();
    } else if (trainType == "REDosto7") {
        p = initREDosto7Params();
    } else if (trainType == "Freight") {
        p = initFreightParams();
    } else if (trainType == "ICE3") {
        p = initICE3Params();
    } else if (trainType == "MireoPlusB") {
        p = initMireoPlusB2TParams();
    } else if (trainType == "MireoPlusH") {
        p = initMireoPlusH2TParams();
    } else if (trainType == "custom") {
        p = initCustomParams();
    } else {
        throw std::runtime_error("Unknown train type: " + trainType);
    }
    std::map<std::string, std::string> kv;
    for (size_t i = 3; i < tok.size(); ++i) {
        const size_t eq = tok[i].find('=');
        kv[tok[i].substr(0, eq)] = tok[i].substr(eq + 1);
    }
    auto flag = [&](const char* k) { return kv.count(k) != 0 && kv[k] == "1"; };
    auto num = [&](const char* k, double def) { return kv.count(k) != 0 ? std::stod(kv[k]) : def; };
    auto table = [&](const char* k) {
        std::vector<double> r;
        std::string cur;
        for (char c : kv[k] + ",") {
            if (c == ',') {
                if (!cur.empty()) {
                    r.push_back(std::stod(cur));
                }
                cur.clear();
            } else {
                cur += c;
            }
        }
        return r;
    };
    // override with the user values
    if (flag("maxSpeedSet")) {
        p.vmax = t.maxSpeed;
    }
    if (flag("lengthSet")) {
        p.length = t.length;
    }
    std::shared_ptr<RailCF> r = std::make_shared<RailCF>();
    r->mf = num("massFactor", p.mf);
    r->decl = flag("decelSet") ? t.decel : p.decl;
    r->vmax = p.vmax;
    t.decel = r->decl;  // setMaxDecel
    if (!flag("emergencyDecelSet")) {
        t.emergencyDecel = r->decl + 0.3;
    }
    t.maxSpeed = p.vmax;
    t.length = p.length;
    r->weight = flag("massSet") ? num("mass", 0) / 1000 : p.weight;  // kg to tons
    // tabular curves (SUMOVTypeParameter::getCFProfile)
    r->traction = p.traction;
    r->resistance = p.resistance;
    for (const char* k : {"tractionTable", "resistanceTable"}) {
        if (kv.count(k) != 0) {
            const std::vector<double> speeds = table("speedTable");
            const std::vector<double> values = table(k);
            if (values.size() == 1 || speeds.size() != values.size()) {
                throw std::runtime_error(std::string("Invalid size of ") + k + " for vType '" + t.id + "'");
            }
            std::map<double, double>& m = std::string(k) == "tractionTable" ? r->traction : r->resistance;
            m.clear();
            for (size_t i = 0; i < speeds.size(); ++i) {
                m[speeds[i]] = values[i];
            }
        }
    }
    // parametric curves
    r->maxPower = num("maxPower", RailCF::INVALID);
    r->maxTraction = num("maxTraction", RailCF::INVALID);
    r->resCoefConstant = num("resCoef_constant", RailCF::INVALID);
    r->resCoefLinear = num("resCoef_linear", RailCF::INVALID);
    r->resCoefQuadratic = num("resCoef_quadratic", RailCF::INVALID);
    r->curveResistance = num("curveResistance", r->curveResistance);
    r->curveIntegration = num("curveIntegration", r->curveIntegration);
    r->roecklSharpRadius = num("roeckl_sharp_radius", r->roecklSharpRadius);
    r->roecklNumerator = num("roeckl_numerator", r->roecklNumerator);
    r->roecklNumeratorSharp = num("roeckl_numerator_sharp", r->roecklNumeratorSharp);
    r->roecklOffset = num("roeckl_offset", r->roecklOffset);
    r->roecklOffsetSharp = num("roeckl_offset_sharp", r->roecklOffsetSharp);
    if (r->maxPower != RailCF::INVALID && r->maxTraction == RailCF::INVALID) {
        throw std::runtime_error("Undefined maxTraction for vType '" + t.id + "'");
    }
    const bool someRes = r->resCoefConstant != RailCF::INVALID || r->resCoefLinear != RailCF::INVALID
                         || r->resCoefQuadratic != RailCF::INVALID;
    const bool allRes = r->resCoefConstant != RailCF::INVALID && r->resCoefLinear != RailCF::INVALID
                        && r->resCoefQuadratic != RailCF::INVALID;
    if (someRes && !allRes) {
        throw std::runtime_error("Some undefined resistance coefficients for vType '" + t.id + "'");
    }
    if (r->traction.empty() && r->maxPower == RailCF::INVALID) {
        throw std::runtime_error("Either tractionTable or maxPower must be defined for vType '" + t.id + "'");
    }
    if (r->resistance.empty() && r->resCoefConstant == RailCF::INVALID) {
        throw std::runtime_error("Either resistanceTable or resCoef_constant must be defined for vType '" + t.id + "'");
    }
    t.rail = r;
    t.cfm = CFM::RAIL;
}

// ---------------------------------------------------------------------------
// deadlocks (MSRailSignalControl::haveDeadlock)
// ---------------------------------------------------------------------------
// the trains waiting for each other (the wait relations of this step) form a circle through v; constraint:
// the first rail signal constraint in the circle (-1: none), seen: the trains in the circle
bool Sim::railHaveDeadlock(int v, int& constraint, std::set<int>& seen, std::vector<std::pair<int, RailWaitRelation>>* circle) const {
    constraint = -1;
    seen.clear();
    if (circle != nullptr) {
        circle->clear();
    }
    int cur = v;
    while (seen.count(cur) == 0) {
        auto it = railWaitRelations.find(cur);
        if (it == railWaitRelations.end()) {
            return false;
        }
        if (it->second.constraint >= 0 && constraint < 0) {
            constraint = it->second.constraint;
        }
        seen.insert(cur);
        if (circle != nullptr) {
            circle->push_back({cur, it->second});
        }
        cur = it->second.foe;
    }
    return cur == v;
}

// deadlock-output (MSRailSignalControl::haveDeadlock): a new deadlock, the constraint deactivated to resolve it
void Sim::writeDeadlock(const RailDeadlockFound& d) {
    std::string signals, vehicles, tripIds;
    bool anyConstraint = false;
    for (const auto& item : d.circle) {
        const RailWaitRelation& w = item.second;
        signals += (signals.empty() ? "" : " ") + (w.railSignal < 0 ? std::string("INSERTION") : railSignals[w.railSignal].id);
        vehicles += (vehicles.empty() ? "" : " ") + veh[w.foe].id;
        tripIds += (tripIds.empty() ? "" : " ") + vehTripId(w.foe);
        anyConstraint |= w.constraint >= 0;
    }
    const char* tag = anyConstraint ? "constraintDeadlock" : "deadlock";
    char time[32];
    std::snprintf(time, sizeof(time), "%.2f", STEPS2TIME(now));  // (time2string)
    std::string row = "    <" + std::string(tag) + " time=\"" + time + "\" signals=\"" + signals + "\" vehicles=\"" + vehicles + "\"";
    if (anyConstraint) {
        row += " tripIds=\"" + tripIds + "\"";
    }
    if (d.resolved < 0) {
        deadlockRows.push_back(row + "/>\n");
        return;
    }
    // the first constraint of the circle: its signal and the train it blocked
    for (const auto& item : d.circle) {
        if (item.second.constraint == d.resolved) {
            static const char* tags[] = {"predecessor", "insertionPredecessor", "foeInsertion", "insertionOrder", "bidiPredecessor"};
            const RailConstraint& c = railConstraints[d.resolved];
            row += ">\n        <resolvedConstraint id=\"" + railSignals[item.second.railSignal].id + "\">\n";
            row += "            <" + std::string(tags[c.type]) + " tripId=\"" + vehTripId(item.first) + "\" tl=\""
                   + railSignals[c.foeSignal].id + "\" foes=\"" + c.foeTrip + "\"";
            if (c.limit > 1) {
                row += " limit=\"" + std::to_string(c.limit) + "\"";
            }
            if (!c.active) {
                row += " active=\"0\"";
            }
            row += "/>\n        </resolvedConstraint>\n    </" + std::string(tag) + ">\n";
            break;
        }
    }
    deadlockRows.push_back(row);
}

// the driveway could be part of a deadlock: is it blocked by the next signal in the circle
void Sim::railAddDWDeadlockChecks(int ri, int dw) {
    auto it = railDeadlockChecks.find(ri);
    if (it != railDeadlockChecks.end()) {
        railFindDeadlockFoes(dw, it->second, {});
    }
}

void Sim::railFindDeadlockFoes(int dw, const std::vector<int>& others, std::vector<int> deadlockFoes) {
    const int circleIndex = (int)deadlockFoes.size();
    if (circleIndex < (int)others.size()) {
        const std::string& other = railSignals[others[circleIndex]].id;
        deadlockFoes.push_back(dw);
        const std::set<int> succ = dwSucc[dw];
        for (int follower : succ) {
            const std::vector<int> foes = driveWays[follower]->foes;
            for (int foe : foes) {
                const int toJ = net.edges[net.lanes[driveWays[foe]->forward.back()].edge].toJ;
                if (toJ >= 0 && net.junctions[toJ].id == other) {
                    railFindDeadlockFoes(foe, others, deadlockFoes);
                }
            }
        }
    } else {
        auto addDWDeadlock = [&](int target) {
            DriveWay& t = *driveWays[target];
            std::set<int> filtered;
            for (int foe : deadlockFoes) {
                if (std::find(t.foes.begin(), t.foes.end(), foe) == t.foes.end()) {
                    filtered.insert(foe);
                }
            }
            if (std::find(t.deadlocks.begin(), t.deadlocks.end(), filtered) == t.deadlocks.end()) {
                t.deadlocks.push_back(filtered);
            }
        };
        for (int dldw : deadlockFoes) {
            if (driveWays[dldw]->origin < 0) {
                addDWDeadlock(dldw);
            } else {
                const std::set<int> preds = dwPred[dldw];
                for (int pred : preds) {
                    const int origin = driveWays[pred]->origin;
                    if (origin >= 0) {
                        const int predRS = tlsRail[net.links[origin].tls];
                        if (std::find(others.begin(), others.end(), predRS) != others.end()) {
                            // driveways that participate in the deadlock cannot prevent it (unless departure driveways)
                            continue;
                        }
                        addDWDeadlock(pred);
                    }
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// rail signal constraints (MSRailSignalConstraint, NLHandler::addPredecessorConstraint)
// ---------------------------------------------------------------------------
// RSC signal kind tripId foeSignal limit active foeTrip...
void Sim::initRailConstraints() {
    railConstraints.clear();
    passedTrackers.clear();
    laneTracker.clear();
    for (const std::vector<std::string>& t : railConstraintLines) {
        auto railSignalOf = [&](const std::string& id) {
            auto it = net.tlsIdx.find(id);
            if (it == net.tlsIdx.end() || tlsRail[it->second] < 0) {
                throw std::runtime_error("Rail signal '" + id + "' in railSignalConstraints is not known");
            }
            return tlsRail[it->second];
        };
        const int ri = railSignalOf(t[1]);
        static const std::map<std::string, int> kinds = {
            {"predecessor", RailConstraint::PREDECESSOR}, {"insertionPredecessor", RailConstraint::INSERTION_PREDECESSOR},
            {"foeInsertion", RailConstraint::FOE_INSERTION}, {"insertionOrder", RailConstraint::INSERTION_ORDER},
            {"bidiPredecessor", RailConstraint::BIDI_PREDECESSOR}};
        const int type = kinds.at(t[2]);
        const std::string& tripId = t[3];
        const int foeSignal = railSignalOf(t[4]);
        const int limit = std::stoi(t[5]);
        const bool active = t[6] == "1";
        for (size_t i = 7; i < t.size(); ++i) {
            railAddConstraint(ri, tripId, type, foeSignal, t[i], limit, active);
        }
    }
}

// MSRailSignal::addConstraint with a MSRailSignalConstraint_Predecessor (its PassedTrackers on the lanes behind
// the foe signal); also while the simulation runs (overtakingReroute)
void Sim::railAddConstraint(int ri, const std::string& tripId, int type, int foeSignal, const std::string& foeTrip, int limit,
                            bool active) {
    RailConstraint c;
    c.type = type;
    c.foeTrip = foeTrip;
    c.foeSignal = foeSignal;
    c.limit = limit;
    c.active = active;
    for (const RailLinkInfo& li : railSignals[foeSignal].links) {
        const int lane = net.links[li.link].viaOrLane();
        if ((int)laneTracker.size() < (int)net.lanes.size()) {
            laneTracker.resize(net.lanes.size(), -1);
        }
        if (laneTracker[lane] < 0) {
            PassedTracker pt;
            pt.lane = lane;
            laneTracker[lane] = (int)passedTrackers.size();
            passedTrackers.push_back(pt);
        }
        PassedTracker& pt = passedTrackers[laneTracker[lane]];
        while (limit > (int)pt.passed.size()) {  // PassedTracker::raiseLimit
            pt.passed.insert(pt.passed.begin() + (pt.lastIndex + 1), "");
        }
        c.trackers.push_back(laneTracker[lane]);
    }
    railSignals[ri].constraints[tripId].push_back((int)railConstraints.size());
    railConstraints.push_back(c);
}

// MSRailSignalConstraint_Predecessor::cleared, PassedTracker::hasPassed
bool Sim::rcCleared(int ci) const {
    const RailConstraint& c = railConstraints[ci];
    if (!c.active) {
        return true;
    }
    for (int ti : c.trackers) {
        const PassedTracker& pt = passedTrackers[ti];
        if (pt.lastIndex < 0) {
            continue;
        }
        int i = pt.lastIndex;
        for (int limit = c.limit; limit > 0; limit--) {
            if (pt.passed[i] == c.foeTrip) {
                return true;
            }
            i = i == 0 ? (int)pt.passed.size() - 1 : i - 1;
        }
    }
    return false;
}

// MSRailSignalConstraint::getVeh(tripId, true): the loaded vehicle with the foe trip id (-1: none)
int Sim::rcFoe(int ci) const {
    const std::string& trip = railConstraints[ci].foeTrip;
    auto alive = [&](int u) {
        return veh[u].status != VehStatus::ARRIVED && veh[u].status != VehStatus::REMOVED && (!veh[u].fromFlow || veh[u].depart <= now);
    };
    auto it = railTripIdLookup.find(trip);
    if (it != railTripIdLookup.end()) {
        for (int u = 0; u < (int)veh.size(); ++u) {
            if (veh[u].id == it->second && alive(u)) {
                return u;
            }
        }
    }
    int best = -1;
    for (int u = 0; u < (int)veh.size(); ++u) {
        // (SUMO's loaded vehicles are ordered by id)
        if (alive(u) && (veh[u].tripId == trip || veh[u].id == trip) && (best < 0 || veh[u].id < veh[best].id)) {
            best = u;
        }
    }
    return best;
}

// MSRailSignal::constraintsAllow
bool Sim::constraintsAllow(int ri, int v, bool storeWaitRelation) {
    const RailSignal& rs = railSignals[ri];
    if (rs.constraints.empty()) {
        return true;
    }
    auto it = rs.constraints.find(vehTripId(v));
    if (it != rs.constraints.end()) {
        for (int ci : it->second) {
            if (!railConstraints[ci].isInsertionConstraint() && !rcCleared(ci)) {
                if (storeWaitRelation && opt.timeToTeleportRSDeadlock > 0 && veh[v].waitingTime > 0) {
                    const int foe = rcFoe(ci);
                    if (foe >= 0) {
                        railWaitRelations[v] = RailWaitRelation{ri, foe, ci};
                    }
                }
                return false;
            }
        }
    }
    return true;
}

// MSRailSignal::hasInsertionConstraint
bool Sim::hasInsertionConstraint(int link, int v) {
    const Link& k = net.links[link];
    if (railConstraints.empty() || k.junction < 0 || net.junctions[k.junction].type != "rail_signal" || k.tls < 0 || tlsRail[k.tls] < 0) {
        return false;
    }
    const int ri = tlsRail[k.tls];
    auto it = railSignals[ri].constraints.find(vehTripId(v));
    if (it != railSignals[ri].constraints.end()) {
        for (int ci : it->second) {
            if (railConstraints[ci].isInsertionConstraint() && !rcCleared(ci)) {
                if (railConstraints[ci].type == RailConstraint::INSERTION_ORDER) {
                    veh[v].insertionOrderMark = true;
                }
                if (opt.timeToTeleportRSDeadlock > 0) {
                    const int foe = rcFoe(ci);
                    if (foe >= 0) {
                        railWaitRelations[v] = RailWaitRelation{ri, foe, ci};
                    }
                }
                return true;
            }
        }
    }
    return false;
}

// PassedTracker::notifyEnter
void Sim::trackerEnter(int v, int lane) {
    if (!laneHasTracker(lane)) {
        return;
    }
    PassedTracker& pt = passedTrackers[laneTracker[lane]];
    pt.lastIndex = (pt.lastIndex + 1) % (int)pt.passed.size();
    pt.passed[pt.lastIndex] = vehTripId(v);
}

// ---------------------------------------------------------------------------
// railsignal-block-output (MSNet::writeRailSignalBlocks, MSRailSignal::writeBlocks, MSDriveWay::writeBlocks,
// MSDriveWay::writeDepatureBlocks)
// ---------------------------------------------------------------------------
namespace {
std::string joinSorted(std::vector<std::string> v) {
    std::sort(v.begin(), v.end());
    std::string r;
    for (const std::string& x : v) {
        r += (r.empty() ? "" : " ") + x;
    }
    return r;
}
}  // namespace

void Sim::writeDriveWayBlocks(FILE* f, int d, int indent) const {
    const DriveWay& dw = *driveWays[d];
    const std::string in(indent * 4, ' '), in1((indent + 1) * 4, ' ');
    auto lanesOf = [&](const std::vector<int>& lanes) {
        std::string r;
        for (int l : lanes) {
            r += (r.empty() ? "" : " ") + net.lanes[l].id;
        }
        return r;
    };
    const bool sub = dw.parent >= 0;
    std::string edges;
    for (int e : dw.route) {
        edges += (edges.empty() ? "" : " ") + net.edges[e].id;
    }
    std::fprintf(f, "%s<%s id=\"%s\" vehicle=\"%s\" edges=\"%s\"", in.c_str(), sub ? "subDriveWay" : "driveWay", dw.id.c_str(),
                 dw.firstVehicle.c_str(), edges.c_str());
    if (dw.coreSize != (int)dw.route.size()) {
        std::fprintf(f, " core=\"%d\"", dw.coreSize);
    }
    std::fprintf(f, ">\n%s<forward lanes=\"%s\"/>\n", in1.c_str(), lanesOf(dw.forward).c_str());
    if (!sub) {
        std::fprintf(f, "%s<bidi lanes=\"%s\"", in1.c_str(), lanesOf(dw.bidi).c_str());
        if (!dw.bidiExtended.empty()) {
            std::fprintf(f, "\n                    deadlockCheck=\"%s\"", lanesOf(dw.bidiExtended).c_str());
        }
        std::fprintf(f, "/>\n%s<flank lanes=\"%s\"/>\n", in1.c_str(), lanesOf(dw.flank).c_str());
        std::vector<std::string> signals;
        for (int l : dw.conflictLinks) {
            signals.push_back(net.tls[net.links[l].tls].id + "_" + std::to_string(net.links[l].tlIndex));
        }
        std::fprintf(f, "%s<conflictLinks signals=\"%s\"/>\n", in1.c_str(), joinSorted(signals).c_str());
        if (!dw.foes.empty()) {
            std::vector<std::string> foes;
            for (int foe : dw.foes) {
                foes.push_back(driveWays[foe]->id);
            }
            std::fprintf(f, "%s<foes driveWays=\"%s\"/>\n", in1.c_str(), joinSorted(foes).c_str());
        }
        // (MSDriveWay::mySidings: ordered by the foe's id)
        std::vector<std::pair<std::string, const std::vector<Siding>*>> sidings;
        for (const auto& item : dw.sidings) {
            sidings.push_back({driveWays[item.first]->id, &item.second});
        }
        std::sort(sidings.begin(), sidings.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
        const std::string in2((indent + 2) * 4, ' ');
        for (const auto& item : sidings) {
            std::fprintf(f, "%s<sidings foe=\"%s\">\n", in1.c_str(), item.first.c_str());
            for (const Siding& sd : *item.second) {
                std::fprintf(f, "%s<siding start=\"%s\" end=\"%s\" length=\"%.2f\"", in2.c_str(), net.edges[dw.route[sd.start]].id.c_str(),
                             net.edges[dw.route[sd.end]].id.c_str(), sd.length);
                if (!sd.intermediateEnds.empty()) {
                    std::string ends;
                    for (int i : sd.intermediateEnds) {
                        ends += (ends.empty() ? "" : " ") + net.edges[dw.route[i]].id;
                    }
                    std::fprintf(f, " intermediateEnds=\"%s\"", ends.c_str());
                }
                std::fprintf(f, "/>\n");
            }
            std::fprintf(f, "%s</sidings>\n", in1.c_str());
        }
        for (const std::set<int>& dl : dw.deadlocks) {
            std::vector<std::string> ids;
            for (int x : dl) {
                ids.push_back(driveWays[x]->id);
            }
            std::fprintf(f, "%s<deadlock foes=\"%s\"/>\n", in1.c_str(), joinSorted(ids).c_str());
        }
    }
    std::fprintf(f, "%s</%s>\n", in.c_str(), sub ? "subDriveWay" : "driveWay");
    for (int s2 : dw.subs) {
        writeDriveWayBlocks(f, s2, indent);
    }
}

void Sim::writeDriveWayVehicles(FILE* f, int d, int indent) const {
    const DriveWay& dw = *driveWays[d];
    static const char* reasons[] = {"departed", "junction", "teleport", "parking", "reroute", "arrived", "teleportArrived"};
    const std::string in(indent * 4, ' ');
    const char* tag = dw.parent >= 0 ? "subDriveWay" : "driveWay";
    if (dw.events.empty()) {
        std::fprintf(f, "%s<%s id=\"%s\"/>\n", in.c_str(), tag, dw.id.c_str());
    } else {
        std::fprintf(f, "%s<%s id=\"%s\">\n", in.c_str(), tag, dw.id.c_str());
        for (const DwVehEvent& ev : dw.events) {
            std::fprintf(f, "%s    <%s id=\"%s\" time=\"%.2f\" reason=\"%s\"/>\n", in.c_str(), ev.entry ? "entry" : "exit",
                         veh[ev.veh].id.c_str(), STEPS2TIME(ev.time), reasons[ev.reason]);
        }
        std::fprintf(f, "%s</%s>\n", in.c_str(), tag);
    }
    for (int s2 : dw.subs) {
        writeDriveWayVehicles(f, s2, indent);
    }
}

void Sim::writeRailSignalBlocks(bool vehicles) const {
    FILE* f = openOutput(vehicles ? "railsignal_vehicles.xml" : "railsignal_blocks.xml");
    const char* root = vehicles ? "railsignal-vehicle-output" : "railsignal-block-output";
    std::fprintf(f, "<%s>\n", root);
    // (MSTLLogicControl::getAllLogics: by id)
    std::vector<int> order(railSignals.size());
    for (size_t i = 0; i < order.size(); ++i) {
        order[i] = (int)i;
    }
    std::sort(order.begin(), order.end(), [&](int a, int b) { return railSignals[a].id < railSignals[b].id; });
    for (int i : order) {
        const RailSignal& rs = railSignals[i];
        if (rs.links.empty()) {
            std::fprintf(f, "    <railSignal id=\"%s\"/>\n", rs.id.c_str());
            continue;
        }
        std::fprintf(f, "    <railSignal id=\"%s\">\n", rs.id.c_str());
        for (const RailLinkInfo& li : rs.links) {
            const Link& k = net.links[li.link];
            std::fprintf(f, "        <link linkIndex=\"%d\" from=\"%s\" to=\"%s\"", k.tlIndex, net.lanes[k.laneBefore].id.c_str(),
                         net.lanes[k.via >= 0 ? k.via : k.lane].id.c_str());
            if (li.driveways.empty()) {
                std::fprintf(f, "/>\n");
                continue;
            }
            std::fprintf(f, ">\n");
            for (int d : li.driveways) {
                if (vehicles) {
                    writeDriveWayVehicles(f, d, 3);
                } else {
                    writeDriveWayBlocks(f, d, 3);
                }
            }
            std::fprintf(f, "        </link>\n");
        }
        std::fprintf(f, "    </railSignal>\n");
    }
    // (MSDriveWay::myDepartureDriveways: by the edge's numerical id)
    for (const auto& item : departureDriveways) {
        if (item.second.empty()) {
            continue;
        }
        std::fprintf(f, "    <departJunction id=\"%s\">\n", net.edges[item.first].from.c_str());
        for (int d : item.second) {
            if (vehicles) {
                writeDriveWayVehicles(f, d, 2);
            } else {
                writeDriveWayBlocks(f, d, 2);
            }
        }
        std::fprintf(f, "    </departJunction>\n");
    }
    std::fprintf(f, "</%s>\n", root);
    std::fclose(f);
}

}  // namespace ps
