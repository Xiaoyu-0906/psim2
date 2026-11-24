// Pedestrian router (SUMO 1.28 PedestrianRouter on the pedestrian-only IntermodalNetwork with
// DijkstraRouter): used by the striping model to pick the way across a junction and the initial
// walking direction. The graph and the order of the numerical ids follow IntermodalNetwork, the
// queue order (effort, then numerical id) follows DijkstraRouter::EdgeInfoByEffortComparator, so
// that ties are broken as in SUMO.
#pragma once

#include <vector>

#include "net.hpp"

namespace ps {

class PedRouter {
public:
    // trip parameters (IntermodalTrip)
    struct Trip {
        int from = -1, to = -1;    // network edges
        double departPos = 0, arrivalPos = 0;
        double speed = 1;
        int onlyNode = -1;         // junction the route must stay at (-1: any)
        int prohibitedEdge = -1;   // network edge that may not be used (both directions)
        const std::vector<double>* edgeFactor = nullptr;  // routing preferences of the person's type
        // getTravelTimeStaticRandomized (--weights.random-factor): psim2's random numbers per query and edge
        double randomFactor = 1;
        uint64_t randomSeed = 0, randomKey = 0;
    };

    void build(const Network& net);
    double walkOppositeFactor = 1;  // --persontrip.walk-opposite-factor (gWeightsWalkOppositeFactor)
    // edges of the route (pedestrian edges only, PedestrianEdge::includeInRoute(true)); empty if none
    std::vector<int> compute(const Network& net, const Trip& trip) const;

private:
    enum Kind { PED, DEPART_CONN, ARRIVAL_CONN, WALK_CONN };
    struct Node {
        Kind kind;
        int edge = -1;        // network edge (-1 for walking connectors)
        bool forward = true;
        double startPos = 0;  // PedestrianEdge::myStartPos
        double length = 0;
        int lane = -1;        // the sidewalk (crossing lane for crossings)
        bool opposite = false;  // against the traffic direction (PedestrianEdge::myIsOpposite)
        std::vector<int> succ;
    };
    std::vector<Node> myNodes;
    std::vector<std::vector<int>> myVia;      // per node: the via node of each successor (-1 if none)
    std::vector<std::pair<int, int>> myBidi;  // per network edge: (forward, backward) node, -1 if none
    std::vector<int> myDepart, myArrival;     // per network edge
    double effort(const Network& net, const Node& n, const Trip& trip, double time) const;
    bool prohibits(const Network& net, const Node& n, const Trip& trip) const;
};

// getSidewalk<MSEdge, MSLane>: prefer lanes exclusive to the class, otherwise the first allowing it
int getSidewalk(const Network& net, int edge, SVCPermissions svc = VC_PEDESTRIAN);

}  // namespace ps
