// Static road network: lanes (normal and junction-internal), links with their
// right-of-way relations and conflict points, junction logics and traffic
// light programs. Built once at load time and read-only afterwards (except for
// the signal state of traffic-light controlled links, which is written by the
// traffic-light phase at the start of a step).
//
// Runtime editing: elements are never removed from the arrays (indices stay
// valid); removed elements are marked dead and new ones are appended. Every
// edit increments `version`, which invalidates derived data (allowed lanes,
// lane-change schedule, vehicles' best lanes). Junction initialisation works
// per junction so that an edited junction can be re-initialised alone.
//
// Initialisation follows SUMO 1.28: NLHandler::addConnection,
// MSRightOfWayJunction::postloadInit, MSInternalJunction::postloadInit,
// MSLink::setRequestInformation and MSLink::recheckSetRequestInformation.
#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "common.hpp"
#include "geom.hpp"

namespace ps {

// LinkState (SUMOXMLDefinitions.h)
enum : char {
    LS_GREEN_MAJOR = 'G', LS_GREEN_MINOR = 'g', LS_RED = 'r', LS_REDYELLOW = 'u', LS_YELLOW_MAJOR = 'Y',
    LS_YELLOW_MINOR = 'y', LS_OFF_BLINKING = 'o', LS_OFF_NOSIGNAL = 'O', LS_MAJOR = 'M', LS_MINOR = 'm',
    LS_EQUAL = '=', LS_STOP = 's', LS_ALLWAY_STOP = 'w', LS_ZIPPER = 'Z', LS_DEADEND = '-',
};

// vehicle classes: SUMO's bit masks (SUMOVehicleClass.h)
using SVCPermissions = long long;
constexpr SVCPermissions VC_PRIVATE = 1LL << 0, VC_EMERGENCY = 1LL << 1, VC_AUTHORITY = 1LL << 2, VC_ARMY = 1LL << 3,
                         VC_VIP = 1LL << 4, VC_PEDESTRIAN = 1LL << 5, VC_PASSENGER = 1LL << 6, VC_HOV = 1LL << 7,
                         VC_TAXI = 1LL << 8, VC_BUS = 1LL << 9, VC_COACH = 1LL << 10, VC_DELIVERY = 1LL << 11,
                         VC_TRUCK = 1LL << 12, VC_TRAILER = 1LL << 13, VC_MOTORCYCLE = 1LL << 14, VC_MOPED = 1LL << 15,
                         VC_BICYCLE = 1LL << 16, VC_E_VEHICLE = 1LL << 17, VC_TRAM = 1LL << 18, VC_RAIL_URBAN = 1LL << 19,
                         VC_RAIL = 1LL << 20, VC_RAIL_ELECTRIC = 1LL << 21, VC_RAIL_FAST = 1LL << 22, VC_SHIP = 1LL << 23,
                         VC_CONTAINER = 1LL << 24, VC_CABLE_CAR = 1LL << 25, VC_SUBWAY = 1LL << 26, VC_AIRCRAFT = 1LL << 27,
                         VC_WHEELCHAIR = 1LL << 28, VC_SCOOTER = 1LL << 29, VC_DRONE = 1LL << 30, VC_CUSTOM1 = 1LL << 31,
                         VC_CUSTOM2 = 1LL << 32;
constexpr SVCPermissions VC_ALL = (1LL << 33) - 1;
constexpr SVCPermissions VC_RAIL_CLASSES = VC_RAIL_ELECTRIC | VC_RAIL_FAST | VC_RAIL | VC_RAIL_URBAN | VC_TRAM | VC_SUBWAY | VC_CABLE_CAR;
SVCPermissions vclassFromName(const std::string& name);  // throws for unknown names
bool isRailway(SVCPermissions p);          // rail classes only (and taxi)
bool isRailwayOrShared(SVCPermissions p);  // rail classes, also shared with road classes

enum ConflictFlag { CONFLICT_DEFAULT, CONFLICT_DUMMY_MERGE, CONFLICT_NO_INTERSECTION, CONFLICT_STOP_AT_INTERNAL_JUNCTION,
                    CONFLICT_SIBLING_CONTINUATION };

constexpr double NO_INTERSECTION = 10000;
constexpr double DIVERGENCE_MIN_WIDTH = 2.5;

struct ConflictInfo {
    int foeConflictIndex = -1;
    double lengthBehindCrossing = 0;
    double conflictSize = 0;
    int flag = CONFLICT_DEFAULT;
};

struct Link {
    int laneBefore = -1;  // MSLink::myLaneBefore
    int lane = -1;        // target lane (always a normal lane): MSLink::myLane
    int via = -1;         // first junction-internal lane on the way, -1 if none
    char dir = 's';
    char state = LS_MAJOR;      // current state (changes with traffic lights)
    char offState = LS_MAJOR;   // state from the net file
    char lastGreenState = LS_GREEN_MINOR;
    Time lastStateChange = std::numeric_limits<Time>::min() / 2;
    int tls = -1;
    int tlIndex = -1;
    double visibility = 4.5;
    double length = 0;
    bool keepClear = true;
    bool indirect = false;
    SVCPermissions permissions = 0;
    // request information (setRequestInformation)
    int index = -1;
    bool hasFoes = false;
    bool amCont = false;
    bool amContOff = false;
    int internalLaneBefore = -1;
    int junction = -1;
    std::vector<int> foeLinks;
    std::vector<int> foeLanes;
    std::vector<ConflictInfo> conflicts;
    double radius = DMAX;
    int parallelRight = -1, parallelLeft = -1;  // MSLink::myParallelRight / myParallelLeft
    // sublane model: MSLink::myLateralShift, mySublaneFoeLinks (same origin lane and target edge),
    // mySublaneFoeLinks2 (same origin lane, straight foes of a turn), mySublaneFoeLanes (exit links)
    double lateralShift = 0;
    std::vector<int> sublaneFoeLinks, sublaneFoeLinks2, sublaneFoeLanes;
    // pedestrians (MSLink::myWalkingAreaFoe, myWalkingAreaFoeExit, myHavePedestrianCrossingFoe, myDistToFoePedCrossing)
    int walkingAreaFoe = -1, walkingAreaFoeExit = -1;
    bool havePedestrianCrossingFoe = false;
    double distToFoePedCrossing = DMAX;

    bool havePriority() const { return state >= 'A' && state <= 'Z'; }
    bool haveRed() const { return state == LS_RED || state == LS_REDYELLOW; }
    bool haveYellow() const { return state == LS_YELLOW_MINOR || state == LS_YELLOW_MAJOR; }
    bool haveGreen() const { return state == LS_GREEN_MAJOR || state == LS_GREEN_MINOR; }
    bool mustStop() const { return state == LS_STOP || state == LS_ALLWAY_STOP; }
    bool isTurnaround() const { return dir == 't' || dir == 'T'; }
    bool fromInternalLane() const { return internalLaneBefore >= 0; }
    bool isCont() const {
        return (state == LS_OFF_BLINKING || state == LS_ALLWAY_STOP || state == LS_STOP) ? amContOff : amCont;
    }
    bool isEntryLink() const { return via >= 0 && internalLaneBefore < 0; }
    bool isExitLink() const { return internalLaneBefore >= 0 && via < 0; }
    bool isInternalJunctionLink() const { return internalLaneBefore >= 0 && via >= 0; }
    bool isConflictEntryLink() const { return !amCont && (isEntryLink() || (internalLaneBefore >= 0 && via >= 0)); }
    int viaOrLane() const { return via >= 0 ? via : lane; }
};

// SumoXMLEdgeFunc: crossings and walking areas lie inside a junction but are not "internal" (MSEdge::isInternal)
enum EdgeFunc : uint8_t { EF_NORMAL, EF_INTERNAL, EF_CROSSING, EF_WALKINGAREA };

struct IncomingLane {
    int lane;
    int link;  // the link that leads onto the lane (MSLane::IncomingLaneInfo::viaLink)
    double length;
};

struct Lane {
    std::string id;
    int edge = -1;
    int index = 0;
    double speed = 0;
    double length = 0;
    double width = 3.2;
    double rightSideOnEdge = 0;  // MSLane::myRightSideOnEdge (the widths of the lanes to the right)
    int rightmostSublane = 0;    // MSLane::myRightmostSublane (index into Edge::sublaneSides)
    SVCPermissions permissions = VC_ALL;
    SVCPermissions origPermissions = VC_ALL;  // MSLane::myOriginalPermissions (without transient changes: rerouters)
    SVCPermissions changeLeft = VC_ALL;
    SVCPermissions changeRight = VC_ALL;
    bool internal = false;      // MSLane::isInternal (function internal only)
    bool crossing = false;      // MSLane::isCrossing
    bool walkingArea = false;   // MSLane::isWalkingArea
    bool accel = false;  // acceleration lane (MSLane::isAccelLane)
    bool dead = false;  // removed by a runtime edit
    Shape shape;
    std::vector<double> z;  // the elevation of the shape points (empty: flat)
    double lengthGeometryFactor = 1;
    std::vector<int> links;              // outgoing links in net-file order
    std::vector<IncomingLane> incoming;  // MSLane::myIncomingLanes
    std::vector<std::pair<int, std::vector<int>>> approaching;  // (edge, lanes): MSLane::myApproachingLanes
    int logicalPred = -1;    // getLogicalPredecessorLane()
    int canonicalPred = -1;  // getCanonicalPredecessorLane()
    int canonicalSucc = -1;  // getCanonicalSuccessorLane()
    double maxBrakeDist = 0;

    int bidi = -1;  // MSLane::getBidiLane
    // MSLane::allowsVehicleClass (SVC_IGNORING = 0 is allowed everywhere)
    bool allows(SVCPermissions vclass) const { return (permissions & vclass) == vclass; }
    bool isNormal() const { return !internal && !crossing && !walkingArea; }
    double interpolateGeometryPosToLanePos(double g) const { return g / lengthGeometryFactor; }
    double interpolateLanePosToGeometryPos(double p) const { return p * lengthGeometryFactor; }
    // PositionVector::positionAtOffset with 3D segment lengths: the elevation at a geometry offset and the
    // horizontal offset of that point along the shape
    void elevationAtOffset(double pos, double& elevation, double& offset2D) const {
        double seen = 0, seen2D = 0;
        for (size_t i = 0; i + 1 < shape.size(); ++i) {
            const double d2 = shape[i].distanceTo2D(shape[i + 1]);
            const double dz = z[i + 1] - z[i];
            const double d3 = std::sqrt(d2 * d2 + dz * dz);
            if (seen + d3 >= pos || i + 2 == shape.size()) {
                const double f = d3 > 0 ? std::min(1., std::max(0., (pos - seen) / d3)) : 0;
                elevation = z[i] + f * dz;
                offset2D = seen2D + f * d2;
                return;
            }
            seen += d3;
            seen2D += d2;
        }
        elevation = z.empty() ? 0 : z.front();
        offset2D = 0;
    }
};

struct Edge {
    std::string id;
    EdgeFunc func = EF_NORMAL;
    bool internal = false;  // func == EF_INTERNAL (MSEdge::isInternal)
    std::string from, to;  // junction ids
    int fromJ = -1, toJ = -1;
    int priority = -1;
    int routingType = -1;    // index into Network::routingTypes (MSEdge::getRoutingType), -1: none
    std::vector<int> lanes;  // by index (0 = rightmost)
    double length = 0;
    std::vector<int> successors;    // successor edges (sorted by numerical id)
    std::vector<int> predecessors;  // MSEdge::myPredecessors
    bool hasLaneChanger = false;    // more than one lane
    bool roundabout = false;
    bool fringe = true;  // MSEdge::isFringe: no outgoing link other than turnarounds
    bool allowsChanging = true;     // MSEdge::allowsLaneChanging
    double width = 0;               // MSEdge::myWidth
    std::vector<double> sublaneSides;  // MSEdge::mySublaneSides (the right sides of all sublanes)
    int bidi = -1;                  // MSEdge::getBidiEdge
    bool isNormal() const { return func == EF_NORMAL; }
    bool isCrossing() const { return func == EF_CROSSING; }
    bool isWalkingArea() const { return func == EF_WALKINGAREA; }
};

struct Junction {
    std::string id;
    std::string type;
    std::vector<int> incLanes;
    std::vector<int> intLanes;
    // right-of-way logic, bit strings as in the net file (last character = index 0)
    std::vector<std::string> response, foes;
    std::vector<char> cont;
    bool hasFoes = false;
    int nIncoming = 0, nOutgoing = 0;  // all edges incl. internal ones (MSJunction::getIncoming / getOutgoing)
    bool logic = false;     // MSRightOfWayJunction
    bool internalJ = false; // MSInternalJunction

    bool test(const std::string& bits, int c) const {
        return c >= 0 && c < (int)bits.size() && bits[bits.size() - 1 - c] == '1';
    }
    bool responseTest(int index, int c) const { return test(response[index], c); }
    bool foesTest(int index, int c) const { return test(foes[index], c); }
};

struct Phase {
    Time duration, minDur, maxDur;
    std::string state;
    bool isGreenPhase() const {
        return state.find_first_of("gG") != std::string::npos && state.find_first_of("yY") == std::string::npos;
    }
    bool isActuated() const { return minDur != maxDur; }
};

struct TLSDef {
    std::string id;
    std::string type;  // static | actuated
    std::string programID;
    Time offset = 0;
    std::vector<Phase> phases;
    std::vector<std::pair<std::string, std::string>> params;
    std::vector<std::vector<int>> links;  // per tl index
    std::vector<std::vector<int>> lanes;  // per tl index (the approach lanes)
    std::vector<int> railLinks;           // links with tl index -1: the incoming tracks of a rail crossing
    std::string param(const std::string& key, const std::string& def) const {
        for (const auto& p : params) {
            if (p.first == key) {
                return p.second;
            }
        }
        return def;
    }
};

class Network {
public:
    std::vector<Lane> lanes;
    std::vector<Edge> edges;
    std::vector<Link> links;
    std::vector<Junction> junctions;
    std::vector<TLSDef> tls;
    std::unordered_map<std::string, int> laneIdx, edgeIdx, junctionIdx, tlsIdx;

    long long version = 0;  // incremented by every runtime edit
    int netVersion = 0;     // net file version, major * 1000 + minor (MSNet::getNetworkVersion)
    bool allEdgesChangeLanes = false;  // continuous lane changing: every edge has a lane changer
    double lateralResolution = 0;      // MSGlobals::gLateralResolution
    bool lefthand = false;             // MSGlobals::gLefthand
    bool usingInternalLanes = true;    // MSGlobals::gUsingInternalLanes (--no-internal-links)

    void loadLine(const std::vector<std::string>& tok);
    void finish();  // postload initialisation
    void invalidateDerived() {
        computeAllowedLanes();
        ++version;
    }
    void computeAllowedLanes();

    // ---- queries (MSLane / MSLink helpers) -----------------------------------
    const Link* linkTo(int from, int target) const;  // MSLane::getLinkTo
    int linkIdxTo(int from, int target) const;
    // MSLeaderInfo: the number of sublanes of a lane
    int numSublanes(double width) const {
        return lateralResolution > 0 ? std::max(1, (int)std::ceil(width / lateralResolution)) : 1;
    }
    int parallel(int lane, int offset) const {
        const Edge& e = edges[lanes[lane].edge];
        const int idx = lanes[lane].index + offset;
        return (idx < 0 || idx >= (int)e.lanes.size()) ? -1 : e.lanes[idx];
    }
    int normalPredecessorLane(int lane) const {
        while (lanes[lane].internal) {
            lane = lanes[lane].logicalPred;
        }
        return lane;
    }
    int normalSuccessorLane(int lane) const {
        while (lanes[lane].internal) {
            lane = lanes[lane].canonicalSucc;
        }
        return lane;
    }
    int predecessorOnEdge(int lane, int edge) const {  // MSLane::getLogicalPredecessorLane(fromEdge)
        for (const IncomingLane& i : lanes[lane].incoming) {
            if (lanes[i.lane].edge == edge) {
                return i.lane;
            }
        }
        return -1;
    }
    bool approachedFromEdge(int lane, int edge) const {  // MSLane::isApproachedFrom(MSEdge*)
        for (const auto& a : lanes[lane].approaching) {
            if (a.first == edge) {
                return true;
            }
        }
        return false;
    }
    bool approachedFromLane(int lane, int from, SVCPermissions vclass) const;  // MSLane::isApproachedFrom(MSLane*, svc)
    int entryLink(int lane) const;            // MSLane::getEntryLink
    int correspondingEntryLink(int link) const;
    int correspondingExitLink(int link) const;
    bool isExitLinkAfterInternalJunction(int link) const;
    bool lastWasContMajor(int link) const;
    bool lastWasContState(int link, char st) const;
    double internalLengthsAfter(int link) const;
    double lengthBeforeCrossing(int link, int foeLane) const;   // INVALID_DOUBLE if none
    double lengthsBeforeCrossing(int link, int foeLane) const;
    double conflictLengthBehindCrossing(int link, int i) const;  // ConflictInfo::getLengthBehindCrossing
    double conflictFoeLengthBehindCrossing(int link, int i) const;
    double conflictFoeSize(int link, int i) const;
    // nullptr = none; ignoreTransient: with the original permissions (routing mode 8)
    const std::vector<int>* allowedLanes(int edge, int destEdge, SVCPermissions vclass, bool ignoreTransient = false) const;
    std::vector<int> allowedLanesOf(int edge, SVCPermissions vclass) const;
    // MSEdge::getSpeedLimit: the speed limit of the first lane
    double edgeSpeedLimit(int e) const { return edges[e].lanes.empty() ? 1. : lanes[edges[e].lanes[0]].speed; }
    SVCPermissions usedClasses = VC_PASSENGER;  // classes of the loaded vehicle types (allowed-lane tables)
    bool usedIgnoring = false;                  // a vehicle type with vClass "ignoring" (SVC_IGNORING = 0)
    bool edgeVersionPermissions = false;
    std::vector<std::pair<int, std::string>> pendingBidi;            // edge, bidi edge id
    std::vector<std::pair<std::string, std::string>> pendingLaneBidi;  // lane, bidi lane
    bool haveTransientPermissions = false;  // a lane's permissions differ from its original ones
    bool haveElevation = false;             // some lanes are not flat (LANEZ)
    std::vector<std::string> routingTypes;   // routing types (edge types) of the edges
    std::unordered_map<std::string, int> routingTypeIdx;
    bool edgeAllows(int e, SVCPermissions vclass, bool ignoreTransient = false) const {
        for (int l : edges[e].lanes) {
            if (((ignoreTransient ? lanes[l].origPermissions : lanes[l].permissions) & vclass) == vclass) {
                return true;
            }
        }
        return false;
    }

private:
    void junctionPostload(int j);
    void initSublaneLinks();
    void internalJunctionPostload(int j);
    void setWalkingAreaFoeExit(int exitLink);
    bool indirectBicycleTurn(int specialLane, int thisLink, int foeFirstPart, int foeLink) const;
    void setRequestInformation(int link, int index, bool hasFoes, bool isCont, const std::vector<int>& foeLinks,
                               const std::vector<int>& foeLanes, int internalLaneBefore);
    double computeDistToDivergence(int lane, int sibling, double minDist, bool sameSource) const;
    bool contIntersect(int lane, int foe) const;
    void recheck();
    std::vector<std::pair<int, int>> myRecheck;
    // allowed lanes per (edge, successor edge, vclass): MSEdge::myAllowedLanes / myAllowed,
    // computed before the simulation (and after edits) so that threads only read them
    struct AllowedSet {
        int dest;
        SVCPermissions vclass;
        std::vector<int> lanes;
    };
    std::vector<std::vector<AllowedSet>> myAllowed;  // per edge
    std::vector<std::vector<AllowedSet>> myOrigAllowed;  // with the original permissions (only with transient changes)
    void computeAllowedSets(std::vector<std::vector<AllowedSet>>& into, bool original) const;
};

}  // namespace ps
