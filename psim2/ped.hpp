// Persons and the striping pedestrian model (SUMO 1.28 MSTransportable, MSStageWalking,
// MSStageWaiting, MSPModel_Striping, MSPModel_Interacting). See PEDESTRIANS.md.
#pragma once

#include <deque>
#include <limits>
#include <string>
#include <vector>

#include "common.hpp"
#include "geom.hpp"

namespace ps {

// MSPModel
constexpr int PED_FORWARD = 1;
constexpr int PED_BACKWARD = -1;
constexpr int PED_UNDEFINED = 0;
constexpr double PED_SAFETY_GAP = 1.0;
constexpr double PED_SIDEWALK_OFFSET = 3;
constexpr double UNSPECIFIED_POS_LAT = std::numeric_limits<double>::max();
constexpr double RANDOM_POS_LAT = -std::numeric_limits<double>::max();
constexpr double DEFAULT_PEDESTRIAN_SPEED = 5. / 3.6;

// MSPModel_Striping parameters (options pedestrian.striping.*)
struct StripingParams {
    double stripeWidth = 0.64;
    double dawdling = 0.2;
    double minGapToVehicle = 0.25;
    int walkingAreaDetail = 4;
    Time jamTime = 300000;
    Time jamTimeCrossing = 10000;
    Time jamTimeNarrow = 1000;
    double jamFactor = 0.25;
    bool legacyPosLat = false;
    double reserveOncoming = 0.0;
    double reserveOncomingJunctions = 0.34;
    double reserveOncomingMax = 1.28;
    bool useNetSpeeds = true;  // networks of version >= 1.20
};

// MSPModel_Striping::WalkingAreaPath
struct WalkingAreaPath {
    int from = -1, to = -1;  // lanes before and after
    int lane = -1;           // the walking area
    Shape shape;
    int dir = PED_FORWARD;   // direction when entering this path
    double angleOverride = INVALID_DOUBLE;
    double length = 0;
};

// MSPModel_Striping::NextLaneInfo
struct PedNextLane {
    int lane = -1;
    int link = -1;
    int dir = PED_UNDEFINED;
};

enum PedObstacleType {
    OBSTACLE_NONE = 0,
    OBSTACLE_PED = 1,
    OBSTACLE_VEHICLE = 3,
    OBSTACLE_END = 4,
    OBSTACLE_NEXTEND = 5,
    OBSTACLE_LINKCLOSED = 6,
    OBSTACLE_ARRIVALPOS = 7
};

// MSPModel_Striping::Obstacle (the description is the person index for pedestrians)
struct PedObstacle {
    double xFwd = 0, xBack = 0;
    double speed = 0;
    int type = OBSTACLE_NONE;
    int person = -1;
    int vehicle = -1;
};

// MSStage (walking, waiting and the initial waiting-for-departure stage)
struct PStage {
    enum Kind { WAITING_FOR_DEPART, WALKING, WAITING, TRIP, DRIVING, ACCESS, TRANSHIP } kind = WALKING;
    int destination = -1;      // edge
    int destinationStop = -1;  // bus stop
    double arrivalPos = 0;
    bool arrivalPosUnspecified = false;  // waiting stage without position (MSStage::unspecifiedArrivalPos)
    // walking
    std::vector<int> route;
    double speed = -1;            // configured speed (or average speed for a given duration), -1 = vType speed
    Time walkingTime = -1;        // walk duration
    double departPos = 0;
    double departPosLat = UNSPECIFIED_POS_LAT;
    int departLane = -1;
    // trip (MSStageTrip): routed when it starts
    int tripFrom = -1;
    uint64_t tripModes = 0;        // SVCPermissions of the modes (car, bicycle, taxi, public = bus)
    std::vector<int> tripVTypes;   // vehicle types of the trip
    double walkFactor = 0.75;
    bool hasArrivalPos = false;
    // waiting
    Time duration = -1, until = -1;
    std::string actType;
    // runtime
    Time departed = -1, arrived = -1;
    int routeStep = 0;                // index into route (MSStageMoving::myRouteStep)
    int currentInternalEdge = -1;     // walking area / crossing / internal edge being walked on
    double internalDistance = 0;
    int pstate = -1;
    bool walkCounted = false;         // counted in Sim::nWalking
    Time lastEdgeEntryTime = 0;
    Time stopEndTime = -1;
    // non-interacting model (MSPModel_NonInteracting::PState): the walk on the current edge
    double niBeginPos = 0, niEndPos = 0;
    Time niDuration = 1, niLastEntry = 0;
    // driving (MSStageDriving): a ride in a vehicle of the given lines (vehicle ids, line names or "ANY")
    std::vector<std::string> lines;
    std::string intendedVeh;
    int origin = -1;             // the 'from' edge (myOrigin)
    int destinationParking = -1;  // a parking area as the destination stop of a ride
    int vehicle = -1;            // while riding
    int waitingEdge = -1;
    double waitingPos = -1;
    Time waitingSince = -1;
    int originStop = -1;
    double vehicleDistance = -1;  // the vehicle's odometer at boarding, the ride distance after the ride
    double rideTimeLoss = -1;
    std::string vehicleId;
    // access (MSPersonStage_Access): from a stop to an access lane or back, along a straight path
    double accessDist = 0;
    bool accessExit = false;
    Pos accessFrom, accessTo;
    Time accessArrival = 0;
};

// MSLink::ApproachingPersonInformation (persons approaching a priority crossing)
struct PersonApproach {
    int person;
    Time arrivalTime;
    Time leavingTime;
};

// MSPerson / MSTransportable
struct Person {
    std::string id;
    int type = -1;
    Time depart = 0;
    double speedFactor = 1;  // getChosenSpeedFactor
    std::vector<PStage> plan;
    int step = 0;            // current stage (myStep)
    bool departed = false;
    bool arrived = false;
    double arrivalTime = -1;
    int atStop = -1;         // bus stop the person is registered at as waiting
    int stopSpot = -1;       // its waiting spot there (MSStoppingPlace::myWaitingTransportables), -1: none
    int onEdge = -1;         // edge the person is registered at (MSEdge::addTransportable)
    bool triggered = false;  // depart="triggered": starts in the vehicle of its first ride
    bool container = false;  // a container (MSTransportable::isPerson false): transported, not walking
    double arrivalPos = 0;   // the person's arrivalPos parameter (used by a ride ending without arrivalPos)
    bool arrivalPosSet = false;
};

// MSPModel_Striping::PState (also used for the temporary copies and the vehicle obstacles of a walking area)
struct PState {
    int person = -1;            // -1 for a vehicle obstacle
    int vehicle = -1;           // PStateVehicle
    double xWidth = 0, yWidth = 0;
    int lane = -1;
    double edgePos = 0;
    double posLat = 0;
    int dir = PED_UNDEFINED;
    double speed = 0;
    double speedLat = 0;
    bool waitingToEnter = true;
    Time waitingTime = 0;
    Time totalWaitingTime = 0;
    bool jammed = false;
    PedNextLane nli;
    int wap = -1;               // walking area path or -1
    mutable double angle = std::numeric_limits<double>::max();
    bool dead = false;          // finished walk
};

}  // namespace ps
