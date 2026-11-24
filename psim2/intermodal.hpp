// Intermodal router for person trips (SUMO 1.28 IntermodalNetwork with car, public transport and stop
// access edges, IntermodalRouter, DijkstraRouter). The edges are numbered in creation order like SUMO's
// numerical ids, the queue breaks ties by them, so that routes and their ties are those of SUMO.
// Derived data: built when the first trip needs it (serial phase), dropped when the network changes.
#pragma once

#include <map>
#include <set>
#include <string>
#include <vector>

#include "common.hpp"
#include "net.hpp"

namespace ps {

// IntermodalTrip (the vehicle is described by its type: a routing vehicle of the personTrip modes)
struct IMTrip {
    int from = -1, to = -1;
    double departPos = 0, arrivalPos = 0;
    double speed = 1;         // walking speed
    Time departTime = 0;
    SVCPermissions modeSet = 0;
    bool haveVehicle = false;  // a car of the trip (CarEdge needs it)
    SVCPermissions vclass = VC_PEDESTRIAN;  // the vehicle's class (or pedestrian)
    // the vehicle for MSNet::getTravelTime
    int vtype = -1;
    double speedFactor = 1;
    int routingMode = 0;
    uint32_t randomSeed = 0;
    const std::vector<double>* edgeFactor = nullptr;  // routing preferences of the person's type (per network edge)
    // IntermodalEdge::getTravelTimeStaticRandomized (--weights.random-factor): psim2's random numbers per query and edge
    double randomFactor = 1;
    uint64_t randomSeed64 = 0, randomKey = 0;
};

// ModeChangeOptions
enum IMTransfer {
    IM_PARKING_AREAS = 1, IM_PT_STOPS = 2, IM_ALL_JUNCTIONS = 4, IM_TAXI_DROPOFF_PARKING_AREAS = 8,
    IM_TAXI_DROPOFF_PT = 16, IM_TAXI_DROPOFF_ANYWHERE = 32, IM_TAXI_PICKUP_PARKING_AREAS = 64,
    IM_TAXI_PICKUP_PT = 128, IM_TAXI_PICKUP_ANYWHERE = 256
};

// stopping place categories (SumoXMLTag order of MSNet::myStoppingPlaces)
enum IMStopCategory { IM_BUS_STOP, IM_CONTAINER_STOP, IM_CHARGING_STATION, IM_PARKING_AREA };

class IntermodalNet {
public:
    // IntermodalRouter::TripItem
    struct TripItem {
        std::string line;       // "" walking, "!stop", "!car" (the trip's vehicle) or a public transport line
        std::string destStop;   // the stopping place where the item ends
        std::string intended;   // intended public transport vehicle
        double depart = -1;
        std::vector<int> edges;
        double traveltime = 0, cost = 0, length = 0;
        double departPos = INVALID_DOUBLE, arrivalPos = INVALID_DOUBLE;
    };
    struct StopSched {
        std::string busStop;
        Time until;
        double x, y;  // MSEdge::getStopPosition (the center of the stop, for the length of the line edge)
    };

    // edge travel time of a car edge for the trip's vehicle (MSEdge::getTravelTimeStatic): set by the owner
    double (*carTravelTime)(const void* owner, int edge, const IMTrip& trip, double time) = nullptr;
    const void* carTravelTimeOwner = nullptr;
    double walkOppositeFactor = 1;  // --persontrip.walk-opposite-factor (gWeightsWalkOppositeFactor)

    // zones: (source edges, sink edges) per taz; their connectors follow the network edges (sink, source per zone)
    void build(const Network& net, int carWalkTransfer, const std::vector<std::pair<std::vector<int>, std::vector<int>>>& tazs);
    void addCarEdges(const Network& net, double taxiWait);
    void addAccess(const Network& net, const std::string& stopId, int stopEdge, double startPos, double endPos, double length,
                   IMStopCategory category, bool isAccess, double taxiWait);
    void addSchedule(const std::string& vehId, const std::string& line, long long repetitionNumber, Time repetitionOffset,
                     const std::vector<StopSched>& stops);
    void addCarAccess(const Network& net, int edge, SVCPermissions svc, double traveltime);
    bool compute(const Network& net, const IMTrip& trip, const std::string& originStop, const std::string& destStop,
                 std::vector<TripItem>& into) const;
    bool built() const { return !myEdges.empty(); }
    void reset();
    size_t memoryBytes() const;

private:
    enum Kind : uint8_t { CONNECTOR, PED, CAR, PT, STOP, ACCESS };
    struct Schedule {
        std::vector<std::string> ids;
        Time begin;
        long long repetitionNumber;
        Time period;
        Time travelTime;
    };
    struct IEdge {
        Kind kind = CONNECTOR;
        bool forward = true;     // pedestrian edge direction
        bool opposite = false;   // pedestrian edge against the traffic (weights.walk-opposite-factor)
        int edge = -1;           // network edge (-1: none)
        int lane = -1;           // pedestrian edge: the sidewalk
        int ref = -1;            // stop edge: the stop name; public transport edge: the line; access: the schedule
        int entryStop = -1;      // public transport edge: the stop edge it starts at
        int sched = -1;          // public transport edge: its schedules
        double length = 0;
        double startPos = 0;     // pedestrian / car / stop start
        double endPos = 0;       // stop end
        double traveltime = -1;  // access edge
        SVCPermissions modeRestriction = 0, vehicleRestriction = 0;
        std::vector<std::pair<int, int>> succ;  // (successor, via) (myFollowingViaEdges)
    };
    std::vector<IEdge> myEdges;
    std::vector<std::multimap<Time, Schedule>> mySchedules;
    std::vector<std::string> myNames;          // stop names and line names (IEdge::ref)
    std::vector<std::pair<int, int>> myBidi;   // per network edge (forward, backward), -1: none
    std::vector<std::vector<int>> myDepart, myArrival;  // per network edge: the connectors of its splits
    std::map<int, int> myWalkingConnector;     // junction -> walking connector
    std::vector<int> myCar;                    // per network edge: car edge (-1: none)
    std::map<std::string, int> myStops;        // stopping place -> stop edge
    std::map<int, std::vector<int>> myAccessSplits;  // edge -> its splits (the edge first)
    std::map<std::string, std::vector<int>> myPTLines;
    std::set<int> myLoopedLines;               // line name indices
    int myCarWalkTransfer = 0;
    int myNumNetEdges = 0;

    int addEdge(const IEdge& e);
    void addSucc(int from, int to, int via = -1) { myEdges[from].succ.push_back({to, via}); }
    bool removeSucc(int from, int to);
    int newConnector(int edge);
    int newPed(const Network& net, int edge, int lane, bool forward, double pos = -1);
    int newCar(const Network& net, int edge, double pos = -1);
    int newAccess(int out, double length, SVCPermissions modeRestriction = 0, SVCPermissions vehicleRestriction = 0,
                  double traveltime = -1);
    int junctionOf(const Network& net, int edge, bool to) const;
    int walkingConnector(const Network& net, int edge) const;
    void addRestrictedCarExit(int from, int to, SVCPermissions vehicleRestriction);
    int findSplitIndex(int toSplit, double pos, double& relPos, bool& needSplit) const;
    void splitEdge(int toSplit, int splitIndex, int afterSplit, double relPos, double length, bool needSplit, int stopConn,
                   bool forward = true, bool addExit = true, bool addEntry = true);
    int departEdge(const Network& net, int edge, double pos) const;
    int arrivalEdge(int edge, double pos) const;
    bool includeInRoute(const Network& net, const IEdge& e, bool allEdges) const;
    double partialLength(const IEdge& e, const IMTrip& trip) const;
    double travelTime(const Network& net, const IEdge& e, const IMTrip& trip, double time) const;
    double travelTimeRaw(const Network& net, const IEdge& e, const IMTrip& trip, double time) const;
    bool prohibits(const Network& net, const IEdge& e, const IMTrip& trip) const;
    const std::vector<std::pair<int, int>>& viaSuccessors(const Network& net, const IEdge& e, SVCPermissions vclass,
                                                          std::vector<std::pair<int, int>>& buf) const;
    double startPosOf(const IEdge& e) const;
    double endPosOf(const IEdge& e) const;
    double intended(const IEdge& e, double time, std::string& intended) const;
    bool loopedLineTransfer(int prev, int cur, double prevTime, double time) const;
    int nameIndex(const std::string& name);
};

}  // namespace ps
