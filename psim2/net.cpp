// Network loading and junction initialisation (see net.hpp).
#include "net.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <stdexcept>

namespace ps {

// SUMO's vehicle class names in bit order
SVCPermissions vclassFromName(const std::string& name) {
    if (name == "ignoring") {
        return 0;  // SVC_IGNORING
    }
    static const char* names[] = {
        "private",
        "emergency",
        "authority",
        "army",
        "vip",
        "pedestrian",
        "passenger",
        "hov",
        "taxi",
        "bus",
        "coach",
        "delivery",
        "truck",
        "trailer",
        "motorcycle",
        "moped",
        "bicycle",
        "evehicle",
        "tram",
        "rail_urban",
        "rail",
        "rail_electric",
        "rail_fast",
        "ship",
        "container",
        "cable_car",
        "subway",
        "aircraft",
        "wheelchair",
        "scooter",
        "drone",
        "custom1",
        "custom2"};
    for (int i = 0; i < (int)(sizeof(names) / sizeof(names[0])); ++i) {
        if (name == names[i]) {
            return 1LL << i;
        }
    }
    throw std::runtime_error("unknown vehicle class " + name);
}


static double toD(const std::string& s) { return std::strtod(s.c_str(), nullptr); }
static int toI(const std::string& s) { return (int)std::strtol(s.c_str(), nullptr, 10); }

static std::vector<std::string> splitBy(const std::string& s, char c) {
    std::vector<std::string> out;
    if (s == "-" || s.empty()) {
        return out;
    }
    size_t start = 0;
    while (true) {
        const size_t p = s.find(c, start);
        out.push_back(s.substr(start, p == std::string::npos ? std::string::npos : p - start));
        if (p == std::string::npos) {
            break;
        }
        start = p + 1;
    }
    return out;
}

namespace {
// connection lines refer to a traffic light that is defined later in the file
struct PendingTL {
    int link;
    std::string tls;
    int index;
};
std::vector<PendingTL> pendingTL;
}  // namespace

int Network::linkIdxTo(int from, int target) const {
    const bool internal = lanes[target].internal;
    for (int l : lanes[from].links) {
        if ((internal && links[l].via == target) || (!internal && links[l].lane == target)) {
            return l;
        }
    }
    return -1;
}

const Link* Network::linkTo(int from, int target) const {
    const int l = linkIdxTo(from, target);
    return l < 0 ? nullptr : &links[l];
}

void Network::loadLine(const std::vector<std::string>& t) {
    const std::string& k = t[0];
    if (k == "EDGE") {
        Edge e;
        e.id = t[1];
        e.func = t[2] == "internal" ? EF_INTERNAL : t[2] == "crossing" ? EF_CROSSING : t[2] == "walkingarea" ? EF_WALKINGAREA : EF_NORMAL;
        if (e.func == EF_NORMAL && t[2] != "normal") {
            throw std::runtime_error("unknown edge function " + t[2]);
        }
        e.internal = e.func == EF_INTERNAL;
        e.from = t[3];
        e.to = t[4];
        e.priority = toI(t[5]);
        if (t.size() > 6 && t[6] != "-") {
            pendingBidi.push_back({(int)edges.size(), t[6]});
        }
        edgeIdx[e.id] = (int)edges.size();
        edges.push_back(e);
    } else if (k == "ETYPE") {
        auto it = routingTypeIdx.find(t[2]);
        if (it == routingTypeIdx.end()) {
            it = routingTypeIdx.emplace(t[2], (int)routingTypes.size()).first;
            routingTypes.push_back(t[2]);
        }
        edges[edgeIdx.at(t[1])].routingType = it->second;
    } else if (k == "LANE") {
        Lane l;
        l.id = t[1];
        l.edge = edgeIdx.at(t[2]);
        l.index = toI(t[3]);
        l.speed = toD(t[4]);
        l.length = toD(t[5]);
        l.width = toD(t[6]);
        l.permissions = std::stoll(t[7]);
        l.origPermissions = l.permissions;
        l.changeLeft = std::stoll(t[8]);
        l.changeRight = std::stoll(t[9]);
        l.internal = edges[l.edge].internal;
        l.crossing = edges[l.edge].isCrossing();
        l.walkingArea = edges[l.edge].isWalkingArea();
        for (const std::string& p : splitBy(t[10], ';')) {
            const size_t c = p.find(',');
            l.shape.push_back(Pos{toD(p.substr(0, c)), toD(p.substr(c + 1))});
        }
        l.lengthGeometryFactor = std::max(POSITION_EPS, geom::length2D(l.shape)) / l.length;
        const int idx = (int)lanes.size();
        laneIdx[l.id] = idx;
        Edge& e = edges[l.edge];
        if ((int)e.lanes.size() != l.index) {
            throw std::runtime_error("lanes of edge " + e.id + " out of order");
        }
        e.lanes.push_back(idx);
        lanes.push_back(l);
    } else if (k == "LANEZ") {
        Lane& l = lanes[laneIdx.at(t[1])];
        for (const std::string& z : splitBy(t[2], ',')) {
            l.z.push_back(toD(z));
        }
        // MSLane::myLengthGeometryFactor: the length of the 3D shape
        double len3D = 0;
        for (size_t i = 0; i + 1 < l.shape.size(); ++i) {
            const double d2 = l.shape[i].distanceTo2D(l.shape[i + 1]);
            len3D += std::sqrt(d2 * d2 + (l.z[i + 1] - l.z[i]) * (l.z[i + 1] - l.z[i]));
        }
        l.lengthGeometryFactor = std::max(POSITION_EPS, len3D) / l.length;
        haveElevation = true;
    } else if (k == "LBIDI") {
        pendingLaneBidi.push_back({t[1], t[2]});
    } else if (k == "JUNC") {
        Junction j;
        j.id = t[1];
        j.type = t[2];
        for (const std::string& s : splitBy(t[3], ',')) {
            j.incLanes.push_back(laneIdx.at(s));
        }
        for (const std::string& s : splitBy(t[4], ',')) {
            j.intLanes.push_back(laneIdx.at(s));
        }
        static const char* logicTypes[] = {"traffic_light", "traffic_light_right_on_red", "right_before_left",
                                           "left_before_right", "priority", "priority_stop", "allway_stop", "zipper",
                                           "rail_signal", "rail_crossing"};
        for (const char* lt : logicTypes) {
            if (j.type == lt) {
                j.logic = true;
            }
        }
        j.internalJ = j.type == "internal";
        static const char* supported[] = {"dead_end", "priority", "priority_stop", "right_before_left", "left_before_right",
                                          "allway_stop", "traffic_light", "traffic_light_unregulated", "unregulated",
                                          "internal", "zipper", "rail_signal", "rail_crossing"};
        bool ok = false;
        for (const char* st : supported) {
            ok |= j.type == st;
        }
        if (!ok) {
            throw std::runtime_error("unsupported junction type " + j.type + " at " + j.id);
        }
        junctionIdx[j.id] = (int)junctions.size();
        junctions.push_back(j);
    } else if (k == "REQ") {
        Junction& j = junctions[junctionIdx.at(t[1])];
        if (toI(t[2]) != (int)j.response.size()) {
            throw std::runtime_error("requests of junction " + j.id + " out of order");
        }
        j.response.push_back(t[3]);
        j.foes.push_back(t[4]);
        j.cont.push_back(t[5] == "1");
    } else if (k == "CONN") {
        Link l;
        l.laneBefore = laneIdx.at(t[1]);
        l.lane = laneIdx.at(t[2]);
        l.via = t[3] == "-" ? -1 : laneIdx.at(t[3]);
        l.dir = t[4][0];
        l.state = l.offState = t[5][0];
        l.visibility = t[8] == "-" ? (l.state == LS_ZIPPER ? 100 : 4.5) : toD(t[8]);
        l.keepClear = t[9] != "0";
        l.indirect = t[10] == "1";
        if (l.via >= 0) {
            l.length = lanes[l.via].length;
        } else {
            l.length = lanes[l.laneBefore].shape.back().distanceTo2D(lanes[l.lane].shape.front());
        }
        const int idx = (int)links.size();
        if (t[6] != "-") {
            pendingTL.push_back({idx, t[6], toI(t[7])});
        }
        links.push_back(l);
        const int next = l.via >= 0 ? l.via : l.lane;
        lanes[next].incoming.push_back({l.laneBefore, idx, lanes[l.laneBefore].length});
        // MSLane::addApproachingLane
        std::vector<std::pair<int, std::vector<int>>>& ap = lanes[l.lane].approaching;
        const int fromEdge = lanes[l.laneBefore].edge;
        bool found = false;
        for (auto& a : ap) {
            if (a.first == fromEdge) {
                a.second.push_back(l.laneBefore);
                found = true;
            }
        }
        if (!found) {
            ap.push_back({fromEdge, {l.laneBefore}});
        }
        lanes[l.laneBefore].links.push_back(idx);
    } else if (k == "TLS") {
        TLSDef d;
        d.id = t[1];
        d.type = t[2];
        d.programID = t[3];
        d.offset = TIME2STEPS(toD(t[4]));
        if (d.type != "static" && d.type != "actuated" && d.type != "rail_signal" && d.type != "rail_crossing" && d.type != "off") {
            throw std::runtime_error("unsupported traffic light type " + d.type);
        }
        tlsIdx[d.id] = (int)tls.size();
        tls.push_back(d);
    } else if (k == "PHASE") {
        TLSDef& d = tls[tlsIdx.at(t[1])];
        Phase p;
        p.duration = TIME2STEPS(toD(t[2]));
        p.minDur = TIME2STEPS(toD(t[3]));
        p.maxDur = TIME2STEPS(toD(t[4]));
        p.state = t[5];
        d.phases.push_back(p);
    } else if (k == "NETVERSION") {
        netVersion = toI(t[1]) * 1000 + toI(t[2]);
    } else if (k == "LEFTHAND") {
        lefthand = true;
    } else if (k == "NOINTERNAL") {
        usingInternalLanes = false;
    } else if (k == "ROUNDABOUT") {
        edges[edgeIdx.at(t[1])].roundabout = true;
    } else if (k == "TLPARAM") {
        tls[tlsIdx.at(t[1])].params.push_back({t[2], t[3]});
    }
}

bool Network::approachedFromLane(int lane, int from, SVCPermissions vclass) const {
    for (int l : lanes[from].links) {
        if (links[l].lane == lane && (links[l].permissions & vclass) == vclass) {
            return true;
        }
    }
    return false;
}

int Network::entryLink(int lane) const {
    if (!lanes[lane].internal) {
        return -1;
    }
    int internal = lane;
    int l = lanes[lane].canonicalPred;
    while (lanes[l].internal) {
        internal = l;
        l = lanes[l].canonicalPred;
    }
    return linkIdxTo(l, internal);
}

int Network::correspondingEntryLink(int link) const {
    int l = link;
    while (lanes[links[l].laneBefore].internal) {
        l = lanes[links[l].laneBefore].incoming.front().link;
    }
    return l;
}

int Network::correspondingExitLink(int link) const {
    int lane = links[link].via;
    int l = link;
    while (lane >= 0) {
        l = lanes[lane].links[0];
        lane = links[l].via;
    }
    return l;
}

bool Network::isExitLinkAfterInternalJunction(int link) const {
    const Link& k = links[link];
    if (k.internalLaneBefore < 0) {
        return false;
    }
    const Lane& before = lanes[k.internalLaneBefore];
    return before.incoming.size() == 1 && links[before.incoming.front().link].isInternalJunctionLink();
}

bool Network::lastWasContMajor(int link) const {
    const Link& k = links[link];
    if (isExitLinkAfterInternalJunction(link)) {
        return lastWasContMajor(lanes[k.internalLaneBefore].incoming[0].link);
    }
    if (k.via < 0 || k.amCont) {
        return false;
    }
    const int pred = lanes[k.via].logicalPred;
    if (!lanes[pred].internal) {
        return false;
    }
    const int pred2 = lanes[pred].logicalPred;
    const Link* predLink = linkTo(pred2, pred);
    if (predLink->havePriority()) {
        return true;
    }
    if (k.havePedestrianCrossingFoe) {
        return predLink->lastGreenState == LS_GREEN_MAJOR;
    }
    return predLink->haveYellow();
}

bool Network::lastWasContState(int link, char st) const {
    const Link& k = links[link];
    if (k.via < 0 || k.amCont || k.havePedestrianCrossingFoe) {
        return false;
    }
    const int pred = lanes[k.via].logicalPred;
    if (!lanes[pred].internal) {
        return false;
    }
    const int pred2 = lanes[pred].logicalPred;
    return linkTo(pred2, pred)->state == st;
}

double Network::internalLengthsAfter(int link) const {
    double len = 0;
    int lane = links[link].via;
    while (lane >= 0 && lanes[lane].internal) {
        len += lanes[lane].length;
        lane = links[lanes[lane].links[0]].via;
    }
    return len;
}

double Network::conflictLengthBehindCrossing(int link, int i) const {
    const Link& k = links[link];
    const ConflictInfo& ci = k.conflicts[i];
    if (ci.flag == CONFLICT_STOP_AT_INTERNAL_JUNCTION) {
        return lanes[k.internalLaneBefore].length;
    }
    return ci.lengthBehindCrossing;
}

double Network::conflictFoeLengthBehindCrossing(int link, int i) const {
    const Link& k = links[link];
    const ConflictInfo& ci = k.conflicts[i];
    if (ci.flag == CONFLICT_DUMMY_MERGE) {
        return 0;
    }
    if (ci.foeConflictIndex >= 0) {
        const int foeExit = lanes[k.foeLanes[i]].links[0];
        return links[foeExit].conflicts[ci.foeConflictIndex].lengthBehindCrossing;
    }
    return -NO_INTERSECTION;
}

double Network::conflictFoeSize(int link, int i) const {
    const Link& k = links[link];
    const ConflictInfo& ci = k.conflicts[i];
    if (ci.foeConflictIndex >= 0) {
        const int foeExit = lanes[k.foeLanes[i]].links[0];
        return links[foeExit].conflicts[ci.foeConflictIndex].conflictSize;
    }
    return 0;
}

double Network::lengthBeforeCrossing(int link, int foeLane) const {
    const Link& k = links[link];
    int foeIx = 0;
    for (; foeIx < (int)k.foeLanes.size(); ++foeIx) {
        if (k.foeLanes[foeIx] == foeLane) {
            break;
        }
    }
    if (foeIx == (int)k.foeLanes.size()) {
        return INVALID_DOUBLE;
    }
    const double dist = lanes[k.internalLaneBefore].length - conflictLengthBehindCrossing(link, foeIx);
    if (dist == -10000.) {
        return INVALID_DOUBLE;
    }
    return dist;
}

double Network::lengthsBeforeCrossing(int link, int foeLane) const {
    int via = links[link].via;
    double total = 0;
    while (via >= 0) {
        const int l = lanes[via].links[0];
        const double d = lengthBeforeCrossing(l, foeLane);
        if (d != INVALID_DOUBLE) {
            return total + d;
        }
        total += lanes[via].length;
        via = links[l].via;
    }
    return INVALID_DOUBLE;
}

const std::vector<int>* Network::allowedLanes(int edge, int destEdge, SVCPermissions vclass, bool ignoreTransient) const {
    for (const AllowedSet& a : (ignoreTransient && haveTransientPermissions ? myOrigAllowed : myAllowed)[edge]) {
        if (a.dest == destEdge && a.vclass == vclass) {
            return a.lanes.empty() ? nullptr : &a.lanes;
        }
    }
    return nullptr;
}

void Network::computeAllowedLanes() {
    computeAllowedSets(myAllowed, false);
    haveTransientPermissions = false;
    for (const Lane& l : lanes) {
        haveTransientPermissions |= l.permissions != l.origPermissions;
    }
    if (haveTransientPermissions) {
        computeAllowedSets(myOrigAllowed, true);
    } else {
        myOrigAllowed.clear();
    }
}

// MSEdge::rebuildAllowedTargets (myAllowedTargets, or myOrigAllowedTargets with the original permissions)
void Network::computeAllowedSets(std::vector<std::vector<AllowedSet>>& into, bool original) const {
    auto allows = [&](int lane, SVCPermissions vclass) {
        return ((original ? lanes[lane].origPermissions : lanes[lane].permissions) & vclass) == vclass;
    };
    into.assign(edges.size(), {});
    for (int e = 0; e < (int)edges.size(); ++e) {
        std::vector<int> dests;
        for (int lane : edges[e].lanes) {
            for (int l : lanes[lane].links) {
                const int d = lanes[links[l].lane].edge;
                if (std::find(dests.begin(), dests.end(), d) == dests.end()) {
                    dests.push_back(d);
                }
            }
        }
        for (int d : dests) {
            for (SVCPermissions vclass = usedIgnoring ? 0 : 1; vclass <= VC_CUSTOM2; vclass = vclass == 0 ? 1 : vclass << 1) {
                if (vclass != 0 && (usedClasses & vclass) == 0) {
                    continue;
                }
                AllowedSet a{d, vclass, {}};
                for (int lane : edges[e].lanes) {
                    if (!allows(lane, vclass)) {
                        continue;
                    }
                    for (int l : lanes[lane].links) {
                        const Link& k = links[l];
                        if (lanes[k.lane].edge == d && allows(k.lane, vclass) && (k.via < 0 || allows(k.via, vclass))) {
                            a.lanes.push_back(lane);
                            break;
                        }
                    }
                }
                into[e].push_back(std::move(a));
            }
        }
    }
}

std::vector<int> Network::allowedLanesOf(int edge, SVCPermissions vclass) const {
    std::vector<int> res;
    for (int lane : edges[edge].lanes) {
        if (lanes[lane].allows(vclass)) {
            res.push_back(lane);
        }
    }
    return res;
}

// ---------------------------------------------------------------------------
// postload initialisation
// ---------------------------------------------------------------------------
void Network::finish() {
    // bidirectional edges and lanes (MSEdge::checkAndRegisterBiDirEdge)
    for (const auto& b : pendingBidi) {
        edges[b.first].bidi = edgeIdx.at(b.second);
    }
    for (const auto& b : pendingLaneBidi) {
        lanes[laneIdx.at(b.first)].bidi = laneIdx.at(b.second);
    }
    pendingBidi.clear();
    pendingLaneBidi.clear();
    // traffic light links
    for (const PendingTL& p : pendingTL) {
        Link& l = links[p.link];
        l.tls = tlsIdx.at(p.tls);
        l.tlIndex = p.index;
        TLSDef& d = tls[l.tls];
        if (p.index < 0) {
            d.railLinks.push_back(p.link);  // the tracks of a rail crossing (MSRailCrossing::addLink)
            continue;
        }
        while ((int)d.links.size() <= p.index) {
            d.links.emplace_back();
            d.lanes.emplace_back();
        }
        d.links[p.index].push_back(p.link);
        d.lanes[p.index].push_back(l.laneBefore);
    }
    pendingTL.clear();
    for (Link& l : links) {
        l.permissions = lanes[l.laneBefore].permissions & lanes[l.lane].permissions & (l.via < 0 ? VC_ALL : lanes[l.via].permissions);
    }
    // MSEdge::closeBuilding
    for (int e = 0; e < (int)edges.size(); ++e) {
        Edge& ed = edges[e];
        ed.length = lanes[ed.lanes[0]].length;
        ed.hasLaneChanger = ed.lanes.size() > 1 || allEdgesChangeLanes;
        double rightSide = 0;  // MSEdge::initialize
        ed.sublaneSides.clear();
        for (int l : ed.lanes) {
            lanes[l].rightSideOnEdge = rightSide;
            lanes[l].rightmostSublane = (int)ed.sublaneSides.size();
            const int nSub = numSublanes(lanes[l].width);
            for (int j = 0; j < nSub; ++j) {
                ed.sublaneSides.push_back(rightSide + j * lateralResolution);
            }
            rightSide += lanes[l].width;
        }
        ed.width = rightSide;
        if (!ed.internal) {
            auto f = junctionIdx.find(ed.from);
            auto t = junctionIdx.find(ed.to);
            ed.fromJ = f == junctionIdx.end() ? -1 : f->second;
            ed.toJ = t == junctionIdx.end() ? -1 : t->second;
        } else {
            ed.fromJ = ed.toJ = junctionIdx.at(ed.from);
        }
        for (int lane : ed.lanes) {
            for (int li : lanes[lane].links) {
                const Link& k = links[li];
                const int to = lanes[k.lane].edge;
                if (std::find(ed.successors.begin(), ed.successors.end(), to) == ed.successors.end()) {
                    ed.successors.push_back(to);
                }
                if (k.dir != 't') {
                    ed.fringe = false;
                }
                std::vector<int>& p = edges[to].predecessors;
                if (std::find(p.begin(), p.end(), e) == p.end()) {
                    p.push_back(e);
                }
                if (k.via >= 0) {
                    std::vector<int>& p2 = edges[lanes[k.via].edge].predecessors;
                    if (std::find(p2.begin(), p2.end(), e) == p2.end()) {
                        p2.push_back(e);
                    }
                }
            }
        }
    }
    // NLEdgeControlBuilder::build: an internal edge next to a roundabout edge is
    // part of the roundabout (edges in net order, so marks can chain)
    for (Edge& ed : edges) {
        if (ed.internal && ((!ed.successors.empty() && edges[ed.successors[0]].roundabout)
                            || (!ed.predecessors.empty() && edges[ed.predecessors[0]].roundabout))) {
            ed.roundabout = true;
        }
    }
    // MSLink::initParallelLinks
    for (Link& k : links) {
        for (int dir : {-1, 1}) {
            const int before = parallel(k.laneBefore, dir);
            const int after = parallel(k.lane, dir);
            int found = -1;
            if (before >= 0 && after >= 0) {
                for (int l : lanes[before].links) {
                    if (links[l].lane == after) {
                        found = l;
                        break;
                    }
                }
            }
            (dir < 0 ? k.parallelRight : k.parallelLeft) = found;
        }
    }
    // NLHandler (SUMO_TAG_NET): every edge, internal ones included, is outgoing of its from- and incoming
    // of its to-junction
    for (const Edge& ed : edges) {
        if (ed.fromJ >= 0) {
            junctions[ed.fromJ].nOutgoing++;
        }
        if (ed.toJ >= 0) {
            junctions[ed.toJ].nIncoming++;
        }
    }
    for (Edge& ed : edges) {
        std::sort(ed.successors.begin(), ed.successors.end());  // MSEdge::by_id_sorter: numerical ids
    }
    // MSLane::getLogicalPredecessorLane (by_connections_to_sorter)
    for (int li = 0; li < (int)lanes.size(); ++li) {
        Lane& lane = lanes[li];
        if (lane.incoming.empty()) {
            continue;
        }
        std::vector<int> pred;
        for (int pe : edges[lane.edge].predecessors) {
            for (const IncomingLane& in : lane.incoming) {
                if (lanes[in.lane].edge == pe) {
                    pred.push_back(pe);
                    break;
                }
            }
        }
        if (pred.empty()) {
            continue;
        }
        const double laneDir = geom::angleAt2D(lanes[edges[lane.edge].lanes[0]].shape, 0);
        auto score = [&](int e1) {
            // allowedLanes(*myEdge) without class restriction
            std::vector<int> ae;
            for (int l : edges[e1].lanes) {
                for (int k : lanes[l].links) {
                    if (lanes[links[k].lane].edge == lane.edge) {
                        ae.push_back(l);
                        break;
                    }
                }
            }
            if (ae.empty()) {
                return 0.;
            }
            return (double)ae.size() + std::fabs(geom::angleDiff(geom::angleAt2D(lanes[ae[0]].shape, 0), laneDir)) / M_PI / 2.;
        };
        std::stable_sort(pred.begin(), pred.end(), [&](int a, int b) { return score(a) < score(b); });
        for (const IncomingLane& in : lane.incoming) {
            if (lanes[in.lane].edge == pred.front()) {
                lane.logicalPred = in.lane;
                break;
            }
        }
    }
    // internal lanes have exactly one predecessor; getEntryLink() is needed while
    // the junctions are initialised
    for (Lane& lane : lanes) {
        if (lane.internal && lane.incoming.size() == 1) {
            lane.canonicalPred = lane.incoming[0].lane;
        }
    }
    // edges: lane changing on internal edges (MSEdge::allowsLaneChanging)
    for (Edge& ed : edges) {
        if (ed.internal) {
            for (int l : ed.lanes) {
                const Link* k = linkTo(lanes[l].logicalPred, l);
                const char st = k->state;
                if (st == LS_MINOR || st == LS_EQUAL || st == LS_STOP || st == LS_ALLWAY_STOP || st == LS_DEADEND) {
                    ed.allowsChanging = false;
                }
            }
        }
    }
    // junction logics: normal junctions before internal ones, each sorted by id
    std::vector<int> order(junctions.size());
    for (int i = 0; i < (int)order.size(); ++i) {
        order[i] = i;
    }
    std::sort(order.begin(), order.end(), [&](int a, int b) { return junctions[a].id < junctions[b].id; });
    // MSBitSetLogic::hasFoes looks at the response bits
    for (Junction& j : junctions) {
        for (const std::string& r : j.response) {
            if (r.find('1') != std::string::npos) {
                j.hasFoes = true;
            }
        }
    }
    for (int j : order) {
        if (junctions[j].logic) {
            junctionPostload(j);
        } else if (!junctions[j].internalJ) {
            // MSNoLogicJunction::postloadInit: links without right-of-way information
            for (int lane : junctions[j].incLanes) {
                for (int l : lanes[lane].links) {
                    setRequestInformation(l, -1, false, false, {}, {}, -1);
                }
            }
        }
    }
    for (int j : order) {
        if (junctions[j].internalJ) {
            internalJunctionPostload(j);
        }
    }
    recheck();
    initSublaneLinks();
    computeAllowedLanes();
    // canonical predecessor / successor (need foe links)
    for (int li = 0; li < (int)lanes.size(); ++li) {
        Lane& lane = lanes[li];
        if (!lane.incoming.empty()) {
            const double laneDir = geom::angleAt2D(lane.shape, 0);
            auto better = [&](const IncomingLane& a, const IncomingLane& b) {
                // incoming_lane_priority_sorter: true if a ranks before b
                int n1 = a.lane, n2 = b.lane;
                while (lanes[n1].internal) {
                    n1 = lanes[n1].incoming[0].lane;
                }
                while (lanes[n2].internal) {
                    n2 = lanes[n2].incoming[0].lane;
                }
                const int l1 = linkIdxTo(n1, li), l2 = linkIdxTo(n2, li);
                bool p1 = true, p2 = true;
                if (l1 >= 0 && l2 >= 0) {
                    for (int f : links[l1].foeLinks) {
                        if (f == l2) {
                            p1 = false;
                        }
                    }
                    for (int f : links[l2].foeLinks) {
                        if (f == l1) {
                            p2 = false;
                        }
                    }
                }
                if (p1 != p2) {
                    return p1;
                }
                const double d1 = std::fabs(geom::angleDiff(geom::angleAt2D(lanes[n1].shape, 0), laneDir));
                const double d2 = std::fabs(geom::angleDiff(geom::angleAt2D(lanes[n2].shape, 0), laneDir));
                return d2 > d1;
            };
            const IncomingLane* best = &lane.incoming[0];
            for (const IncomingLane& in : lane.incoming) {
                if (better(in, *best)) {
                    best = &in;
                }
            }
            lane.canonicalPred = best->lane;
        }
        if (!lane.links.empty()) {
            const double laneDir = geom::angleAt2D(lane.shape, 0);
            std::vector<int> cand = lane.links;
            std::stable_sort(cand.begin(), cand.end(), [&](int a, int b) {
                const int t1 = links[a].lane, t2 = links[b].lane;
                const int p1 = edges[lanes[t1].edge].priority, p2 = edges[lanes[t2].edge].priority;
                if (p1 != p2) {
                    return p1 > p2;
                }
                const double d1 = std::fabs(geom::angleDiff(geom::angleAt2D(lanes[t1].shape, 0), laneDir));
                const double d2 = std::fabs(geom::angleDiff(geom::angleAt2D(lanes[t2].shape, 0), laneDir));
                return d2 > d1;
            });
            lane.canonicalSucc = links[cand[0]].viaOrLane();
        }
    }
}

bool Network::contIntersect(int lane, int foe) const {
    if (links[lanes[foe].links[0]].via >= 0) {
        return !geom::intersectsAtLengths2D(lanes[lane].shape, lanes[foe].shape).empty();
    }
    return false;
}

double Network::computeDistToDivergence(int laneI, int siblingI, double minDist, bool sameSource) const {
    const Lane& lane = lanes[laneI];
    const Lane& sibling = lanes[siblingI];
    double lbcSibling = 0;
    double lbcLane = 0;
    Shape l = lane.shape;
    Shape s = sibling.shape;
    const double length = geom::length2D(l);
    const double sibLength = geom::length2D(s);
    if (!sameSource) {
        std::reverse(l.begin(), l.end());
        std::reverse(s.begin(), s.end());
    } else if (links[entryLink(siblingI)].indirect) {
        lbcSibling += s[s.size() - 1].distanceTo2D(s[s.size() - 2]);
        s.pop_back();
    } else if (links[entryLink(laneI)].indirect) {
        lbcLane += l[l.size() - 1].distanceTo2D(l[l.size() - 2]);
        l.pop_back();
    }
    if (l.back().distanceTo2D(s.back()) > minDist) {
        const std::vector<double> distances = geom::distances(l, s);
        if (distances.back() > minDist && distances[l.size() - 1] > minDist) {
            for (int j = (int)s.size() - 2; j >= 0; j--) {
                const int i = j + (int)l.size();
                const double segLength = s[j].distanceTo2D(s[j + 1]);
                if (distances[i] > minDist) {
                    lbcSibling += segLength;
                } else {
                    lbcSibling += segLength - (minDist - distances[i]) * segLength / (distances[i + 1] - distances[i]);
                    break;
                }
            }
            for (int i = (int)l.size() - 2; i >= 0; i--) {
                const double segLength = l[i].distanceTo2D(l[i + 1]);
                if (distances[i] > minDist) {
                    lbcLane += segLength;
                } else {
                    lbcLane += segLength - (minDist - distances[i]) * segLength / (distances[i + 1] - distances[i]);
                    break;
                }
            }
        }
    }
    const double d1 = sibling.length - lbcSibling;
    const double d2 = lane.length - lbcLane;
    return std::min(std::min(std::max(d1, d2), sibLength), length);
}

// MSLink constructor (lateral shift between lane geometries) and MSLink::setRequestInformation (sublane foes)
void Network::initSublaneLinks() {
    if (lateralResolution <= 0) {
        return;
    }
    for (int li = 0; li < (int)links.size(); ++li) {
        Link& k = links[li];
        k.lateralShift = 0;
        k.sublaneFoeLinks.clear();
        k.sublaneFoeLinks2.clear();
        k.sublaneFoeLanes.clear();
        const Shape& from = lanes[k.laneBefore].shape;
        const Shape& to = lanes[k.viaOrLane()].shape;
        if ((k.via >= 0 || lanes[k.laneBefore].internal) && !from.empty() && !to.empty()
                && !(from.back() == to.front())) {
            const double dist = from.back().distanceTo2D(to.front());
            // PositionVector::move2side(dist) of the end point (to the right for a positive amount)
            Pos moved = from.back();
            int i = (int)from.size() - 1;
            while (i > 0 && from[i - 1].distanceTo2D(from.back()) < POSITION_EPS) {
                --i;
            }
            if (i > 0) {
                const Pos& a = from[i - 1];
                const Pos& b = from.back();
                const double scale = dist / a.distanceTo2D(b);
                moved.x -= (a.y - b.y) * scale;
                moved.y -= (b.x - a.x) * scale;
            }
            k.lateralShift = moved.distanceTo2D(to.front()) < dist ? dist : -dist;
            if (lefthand) {
                k.lateralShift *= -1;
            }
        }
        const int myTarget = lanes[k.lane].edge;
        for (int l2 : lanes[k.laneBefore].links) {
            if (l2 == li) {
                continue;
            }
            if (lanes[links[l2].lane].edge == myTarget) {
                k.sublaneFoeLinks.push_back(l2);
            } else if (k.dir != 's' && links[l2].dir == 's') {
                k.sublaneFoeLinks2.push_back(l2);
            }
        }
        if (k.internalLaneBefore >= 0) {
            for (int l2 : lanes[lanes[k.laneBefore].incoming.front().lane].links) {
                if (links[l2].via != k.internalLaneBefore && links[l2].via >= 0 && lanes[links[l2].lane].edge == myTarget) {
                    k.sublaneFoeLanes.push_back(links[l2].via);
                }
            }
        }
    }
}

void Network::setRequestInformation(int linkI, int index, bool hasFoes, bool isCont, const std::vector<int>& foeLinks,
                                    const std::vector<int>& foeLanes, int internalLaneBefore) {
    Link& k = links[linkI];
    k.index = index;
    k.hasFoes = hasFoes;
    k.amCont = isCont;
    k.foeLinks = foeLinks;
    for (int f : foeLanes) {
        k.foeLanes.push_back(f);
    }
    k.junction = edges[lanes[k.lane].edge].fromJ;
    if (isCont && k.tls >= 0 && internalLaneBefore < 0) {
        // checkContOff
        for (int cand : edges[lanes[k.laneBefore].edge].lanes) {
            for (int l : lanes[cand].links) {
                if (links[l].offState == LS_OFF_NOSIGNAL) {
                    k.amContOff = true;
                }
            }
        }
    }
    k.internalLaneBefore = internalLaneBefore;
    if (internalLaneBefore >= 0) {
        // exit link: crossing points with all foe lanes
        const int lane = internalLaneBefore;
        const Lane& L = lanes[lane];
        const bool beforeInternalJunction = lanes[links[L.links[0]].viaOrLane()].internal;
        if (L.incoming.size() != 1) {
            throw std::runtime_error("internal lane " + L.id + " has several predecessors");
        }
        for (size_t fi = 0; fi < foeLanes.size(); ++fi) {
            const int foeLane = foeLanes[fi];
            const Lane& F = lanes[foeLane];
            k.havePedestrianCrossingFoe = k.havePedestrianCrossingFoe || F.crossing;
            const bool sameTarget = k.lane == links[F.links[0]].lane;
            if (sameTarget && !beforeInternalJunction && !contIntersect(lane, foeLane)) {
                const double minDist = std::min(DIVERGENCE_MIN_WIDTH, 0.5 * (L.width + F.width));
                if (L.shape.back().distanceTo2D(F.shape.back()) >= minDist) {
                    ConflictInfo ci;
                    if (links[entryLink(foeLane)].indirect) {
                        ci.lengthBehindCrossing = -NO_INTERSECTION;
                        ci.conflictSize = 0;
                    } else {
                        ci.lengthBehindCrossing = 0;
                        ci.conflictSize = F.width;
                        ci.flag = CONFLICT_DUMMY_MERGE;
                    }
                    k.conflicts.push_back(ci);
                } else {
                    const double distAfterDivergence = computeDistToDivergence(lane, foeLane, minDist, false);
                    ConflictInfo ci;
                    ci.lengthBehindCrossing = L.interpolateGeometryPosToLanePos(distAfterDivergence);
                    ci.conflictSize = F.width;
                    k.conflicts.push_back(ci);
                }
            } else {
                std::vector<double> i1 = geom::intersectsAtLengths2D(L.shape, F.shape);
                bool haveIntersection = true;
                if (i1.empty()) {
                    i1.push_back(-NO_INTERSECTION);
                    haveIntersection = false;
                } else if (i1.size() > 1) {
                    std::sort(i1.begin(), i1.end());
                }
                std::vector<double> i2 = geom::intersectsAtLengths2D(F.shape, L.shape);
                if (i2.empty()) {
                    i2.push_back(0);
                } else if (i2.size() > 1) {
                    std::sort(i2.begin(), i2.end());
                }
                if (!haveIntersection && links[F.links[0]].via >= 0) {
                    const Pos waitPos = F.shape.back();
                    const double dist = geom::distance2D(L.shape, waitPos, true);
                    if (dist != INVALID_OFFSET && dist < L.width / 2) {
                        i1.clear();
                        i2.clear();
                        i1.push_back(geom::nearestOffsetToPoint2D(L.shape, waitPos, false));
                        i2.push_back(geom::length2D(F.shape));
                        haveIntersection = true;
                    }
                }
                double conflictSize = F.width;
                int flag = CONFLICT_NO_INTERSECTION;
                if (haveIntersection) {
                    flag = CONFLICT_DEFAULT;
                    const double angle1 = geom::naviDegree(geom::rotationAtOffset(L.shape, i1.back()));
                    const double angle2 = geom::naviDegree(geom::rotationAtOffset(F.shape, i2.back()));
                    const double angleDiff = geom::getMinAngleDiff(angle1, angle2);
                    const double widthFactor = 1 / std::max(std::sin(geom::rad(angleDiff)), 0.2) * 2 - 1;
                    conflictSize *= widthFactor;
                    conflictSize = std::min(conflictSize, L.length);
                    i1.back() -= conflictSize / 2;
                    i1.back() = std::max(0.0, i1.back());
                    i1.back() = L.interpolateGeometryPosToLanePos(i1.back());
                    if (lanes[L.logicalPred].internal && !F.crossing) {
                        flag = CONFLICT_STOP_AT_INTERNAL_JUNCTION;
                    }
                    if (F.crossing) {
                        // MSLink::updateDistToFoePedCrossing of the link onto this internal lane
                        Link& before = links[linkIdxTo(lanes[internalLaneBefore].canonicalPred, internalLaneBefore)];
                        before.distToFoePedCrossing = std::min(before.distToFoePedCrossing, i1.back());
                    }
                }
                ConflictInfo ci;
                ci.lengthBehindCrossing = L.length - i1.back();
                ci.conflictSize = conflictSize;
                ci.flag = flag;
                k.conflicts.push_back(ci);
            }
        }
        // overlap with internal lanes from the same source lane
        const int pred = L.logicalPred;
        for (int pl : lanes[pred].links) {
            const int sibling = links[pl].via;
            if (sibling == lane || sibling < 0) {
                continue;
            }
            const Lane& S = lanes[sibling];
            const double minDist = std::min(DIVERGENCE_MIN_WIDTH, 0.5 * (L.width + S.width));
            if (L.shape.front().distanceTo2D(S.shape.front()) >= minDist) {
                continue;
            }
            const double distToDivergence = computeDistToDivergence(lane, sibling, minDist, true);
            double lbcLane;
            if (L.length == S.length && L.edge == S.edge) {
                lbcLane = L.length - distToDivergence;
            } else {
                lbcLane = std::max(0.0, L.length - L.interpolateGeometryPosToLanePos(distToDivergence));
            }
            ConflictInfo ci;
            ci.lengthBehindCrossing = lbcLane;
            ci.conflictSize = S.width;
            auto it = std::find(k.foeLanes.begin(), k.foeLanes.end(), sibling);
            if (it != k.foeLanes.end()) {
                k.conflicts[it - k.foeLanes.begin()] = ci;
            } else {
                k.conflicts.push_back(ci);
                k.foeLanes.push_back(sibling);
            }
            const int siblingCont = links[S.links.front()].viaOrLane();
            if (lanes[siblingCont].internal && geom::distance2D(L.shape, lanes[siblingCont].shape.front(), false) < minDist) {
                const double maxCommonLength = std::min(L.length, S.length + lanes[siblingCont].length);
                ConflictInfo ci2;
                ci2.lengthBehindCrossing = std::max(0.0, L.length - maxCommonLength);
                ci2.conflictSize = lanes[siblingCont].width;
                ci2.flag = CONFLICT_SIBLING_CONTINUATION;
                k.conflicts.push_back(ci2);
                k.foeLanes.push_back(siblingCont);
                myRecheck.push_back({linkI, lanes[siblingCont].links.front()});
            }
        }
        // symmetric conflicts: the link that comes second sets the pointers
        for (int i = 0; i < (int)k.foeLanes.size(); i++) {
            const int foeExit = lanes[k.foeLanes[i]].links[0];
            Link& fe = links[foeExit];
            int found = -1;
            for (int i2 = 0; i2 < (int)fe.foeLanes.size(); i2++) {
                if (fe.foeLanes[i2] == lane) {
                    k.conflicts[i].foeConflictIndex = i2;
                    fe.conflicts[i2].foeConflictIndex = i;
                    myRecheck.erase(std::remove(myRecheck.begin(), myRecheck.end(), std::make_pair(foeExit, linkI)), myRecheck.end());
                    found = i2;
                    break;
                }
            }
            if (found < 0 && k.conflicts[i].flag != CONFLICT_NO_INTERSECTION) {
                myRecheck.push_back({linkI, foeExit});
            }
        }
    }
    if (internalLaneBefore >= 0 && k.dir != 's' && k.dir != 'r') {
        const double angle = std::fabs(geom::angleDiff(geom::angleAt2D(lanes[normalPredecessorLane(k.laneBefore)].shape, -2),
                                                       geom::angleAt2D(lanes[k.lane].shape, 0)));
        if (angle > 0) {
            const Lane& before = lanes[internalLaneBefore];
            double length = geom::length2D(before.shape);
            if (before.incoming.size() == 1 && lanes[before.incoming[0].lane].internal) {
                length += geom::length2D(lanes[before.incoming[0].lane].shape);
            } else if (k.via >= 0) {
                length += geom::length2D(lanes[k.via].shape);
            }
            k.radius = length / angle;
        }
    }
}

void Network::recheck() {
    // MSLink::recheckSetRequestInformation (std::set order: by pointer, here by link index)
    std::sort(myRecheck.begin(), myRecheck.end());
    myRecheck.erase(std::unique(myRecheck.begin(), myRecheck.end()), myRecheck.end());
    for (const auto& item : myRecheck) {
        Link& link = links[item.first];
        Link& foeExit = links[item.second];
        const int lane = link.internalLaneBefore;
        const int foeLane = foeExit.internalLaneBefore;
        int ci = -1;
        for (int i = 0; i < (int)link.foeLanes.size(); i++) {
            if (link.foeLanes[i] == foeLane) {
                ci = i;
                break;
            }
        }
        if (ci == -1) {
            continue;
        }
        ConflictInfo& c = link.conflicts[ci];
        if (c.flag == CONFLICT_SIBLING_CONTINUATION) {
            const int intLane = lane;
            const int siblingCont = foeLane;
            const int sibling = lanes[siblingCont].logicalPred;
            const double distToDivergence = lanes[intLane].length - c.lengthBehindCrossing;
            const double lbc = std::min(lanes[siblingCont].length,
                                        std::max(0.0, lanes[sibling].length + lanes[siblingCont].length - distToDivergence));
            ConflictInfo c2;
            c2.lengthBehindCrossing = lbc;
            c2.conflictSize = lanes[intLane].width;
            c2.foeConflictIndex = ci;
            c.foeConflictIndex = (int)foeExit.conflicts.size();
            foeExit.foeLanes.push_back(intLane);
            foeExit.conflicts.push_back(c2);
            continue;
        }
        std::vector<double> i1 = geom::intersectsAtLengths2D(lanes[foeLane].shape, lanes[lane].shape);
        if (i1.empty()) {
            continue;
        }
        const double widthFactor = c.conflictSize / lanes[foeLane].width;
        const double conflictSize2 = lanes[lane].width * widthFactor;
        std::sort(i1.begin(), i1.end());
        i1.back() -= conflictSize2 / 2;
        i1.back() = std::max(0.0, i1.back());
        c.foeConflictIndex = (int)foeExit.conflicts.size();
        ConflictInfo c2;
        c2.lengthBehindCrossing = lanes[foeLane].length - i1.back();
        c2.conflictSize = conflictSize2;
        foeExit.conflicts.push_back(c2);
    }
    myRecheck.clear();
}

void Network::junctionPostload(int ji) {
    const Junction& J = junctions[ji];
    // links onto walking areas and links from walking areas onto sidewalks are not part of the logic
    auto inLogic = [&](int lane, int l) {
        return !(lanes[links[l].lane].walkingArea || (lanes[lane].walkingArea && !lanes[links[l].lane].crossing));
    };
    std::vector<int> sorted;  // links in request order
    for (int lane : J.incLanes) {
        for (int l : lanes[lane].links) {
            if (inLogic(lane, l)) {
                sorted.push_back(l);
            }
        }
    }
    const int maxNo = (int)sorted.size();
    int requestPos = 0;
    for (int lane : J.incLanes) {
        int walkingAreaFoe = -1;
        for (int linkI : lanes[lane].links) {
            if (lanes[links[linkI].lane].walkingArea) {
                if (lanes[lane].permissions != VC_PEDESTRIAN) {
                    walkingAreaFoe = links[linkI].lane;  // a vehicular lane connects to a walking area (shared space)
                }
                continue;
            }
            if (!inLogic(lane, linkI)) {
                continue;
            }
            if (requestPos >= (int)J.response.size()) {
                throw std::runtime_error("invalid logic position at junction " + J.id);
            }
            const bool cont = J.cont[requestPos] && usingInternalLanes;  // (MSLink::setRequestInformation)
            std::vector<int> foeLinks;
            for (int c = 0; c < maxNo; ++c) {
                if (J.responseTest(requestPos, c)) {
                    const int foe = sorted[c];
                    foeLinks.push_back(foe);
                    if (links[foe].via >= 0) {
                        const int foeExit = lanes[links[foe].via].links[0];
                        if (links[foeExit].via >= 0) {
                            foeLinks.push_back(foeExit);
                        }
                    }
                }
            }
            std::vector<int> foeInternal;
            if (!J.intLanes.empty()) {
                int li = 0;
                for (int c = 0; c < maxNo; ++c) {
                    if (J.foesTest(requestPos, c)) {
                        foeInternal.push_back(J.intLanes[li]);
                        if (lanes[links[linkI].lane].crossing || J.responseTest(requestPos, c) || links[sorted[c]].indirect) {
                            const std::vector<IncomingLane>& inc = lanes[J.intLanes[li]].incoming;
                            if (inc.size() == 1 && lanes[inc[0].lane].internal) {
                                foeInternal.push_back(inc[0].lane);
                            }
                        }
                    }
                    ++li;
                }
            }
            setRequestInformation(linkI, requestPos, J.hasFoes, cont, foeLinks, foeInternal, -1);
            if (links[linkI].via >= 0 && !cont) {
                const int exitLink = lanes[links[linkI].via].links[0];
                setRequestInformation(exitLink, requestPos, false, false, {}, foeInternal, links[linkI].via);
                setWalkingAreaFoeExit(exitLink);
            }
            // the exit link of a crossing is needed by the pedestrian model (vehicles crossing it)
            if (lanes[links[linkI].lane].crossing) {
                const int exitLink = lanes[links[linkI].lane].links[0];
                setRequestInformation(exitLink, requestPos, false, false, {}, foeInternal, links[linkI].lane);
            }
            requestPos++;
        }
        if (walkingAreaFoe >= 0 && lanes[lane].links.size() > 1) {
            for (int l : lanes[lane].links) {
                if (!lanes[links[l].lane].walkingArea) {
                    links[lanes[links[l].via].links[0]].walkingAreaFoe = walkingAreaFoe;
                }
            }
        }
    }
}

// MSLink::addWalkingAreaFoeExit: the exit link of a junction leads onto a lane that is also entered from a walking area
void Network::setWalkingAreaFoeExit(int exitLink) {
    for (const IncomingLane& ili : lanes[links[exitLink].lane].incoming) {
        if (lanes[ili.lane].walkingArea) {
            links[exitLink].walkingAreaFoeExit = ili.lane;
            break;
        }
    }
}

// MSInternalJunction::indirectBicycleTurn: two left-turning bicycle paths that
// intersect on their second / first parts
bool Network::indirectBicycleTurn(int specialLane, int thisLink, int foeFirstPart, int foeLink) const {
    const Link& t = links[thisLink];
    return lanes[specialLane].permissions == VC_BICYCLE && lanes[foeFirstPart].permissions == VC_BICYCLE
           && t.dir == 'l' && links[foeLink].dir == 'l' && t.via >= 0
           && geom::intersects(lanes[t.via].shape, lanes[foeFirstPart].shape);
}

void Network::internalJunctionPostload(int ji) {
    const Junction& J = junctions[ji];
    if (J.incLanes.empty()) {
        throw std::runtime_error("internal junction without incoming lanes");
    }
    const int specialLane = J.incLanes[0];
    const int thisLink = lanes[specialLane].links[0];
    const int parentJ = edges[lanes[specialLane].edge].toJ;
    if (parentJ < 0 || !junctions[parentJ].logic) {
        return;
    }
    const Junction& P = junctions[parentJ];
    const int ownLinkIndex = links[lanes[specialLane].incoming[0].link].index;
    std::vector<int> laneFoes, linkFoes;
    for (int lane : J.intLanes) {
        for (int l : lanes[lane].links) {
            if (links[l].via >= 0) {
                const int foeIndex = links[lanes[lane].incoming[0].link].index;
                if (P.responseTest(ownLinkIndex, foeIndex) || indirectBicycleTurn(specialLane, thisLink, lane, l)) {
                    laneFoes.push_back(lane);
                }
                if (std::find(laneFoes.begin(), laneFoes.end(), links[l].via) == laneFoes.end()) {
                    laneFoes.push_back(links[l].via);
                }
            } else {
                if (std::find(laneFoes.begin(), laneFoes.end(), lane) == laneFoes.end()) {
                    laneFoes.push_back(lane);
                    if (lanes[lane].crossing) {
                        // the walking area before the crossing is not an incoming lane of the internal junction
                        linkFoes.push_back(lanes[lane].incoming[0].link);
                    }
                }
            }
        }
    }
    for (size_t i = 1; i < J.incLanes.size(); ++i) {
        for (int l : lanes[J.incLanes[i]].links) {
            const int linkIndex = links[correspondingEntryLink(l)].index;
            if (linkIndex != -1 && P.responseTest(ownLinkIndex, linkIndex)) {
                linkFoes.push_back(l);
                const int via = links[l].via;
                if (via >= 0 && links[lanes[via].links.front()].via >= 0) {
                    linkFoes.push_back(lanes[via].links.front());
                }
            }
        }
    }
    const int viaLane = links[thisLink].via;
    setRequestInformation(thisLink, ownLinkIndex, true, false, linkFoes, laneFoes, lanes[viaLane].logicalPred);
    const int exitLink = lanes[viaLane].links[0];
    setRequestInformation(exitLink, ownLinkIndex, false, false, {}, laneFoes, viaLane);
    setWalkingAreaFoeExit(exitLink);
}

}  // namespace ps
