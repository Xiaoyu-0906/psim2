// Runtime data layout of psim2.
//
// Design differences to SUMO (behavioural logic is kept):
//  * vehicles, lanes and links live in contiguous arrays and refer to each
//    other by index; per-vehicle data is split into hot kinematic state (Kin)
//    and the rest (Veh)
//  * every step runs in phases separated by barriers; inside a phase a task
//    writes only data it owns (a vehicle, a lane, an edge or a traffic light)
//    and reads everything else from state frozen at the start of the phase
//  * junction approach registrations are rebuilt per link from the vehicles'
//    drive items in vehicle order instead of being written into shared maps
//  * cross-edge side effects of lane changing are sent as messages and
//    applied in a fixed order
//  * random numbers are counter based (see common.hpp)
#pragma once

#include <array>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <unordered_map>

#include "cf.hpp"
#include "net.hpp"
#include "ped.hpp"
#include "pedrouter.hpp"
#include "intermodal.hpp"
#include "threadpool.hpp"

namespace ps {

// ---------------------------------------------------------------------------
// best lanes (MSVehicle::LaneQ)
// ---------------------------------------------------------------------------
struct LaneQ {
    int lane = -1;
    double length = 0;
    double currentLength = 0;
    double occupation = 0;
    double nextOccupation = 0;
    int bestLaneOffset = 0;
    bool allowsContinuation = true;
    std::vector<int> bestContinuations;  // -1 stands for SUMO's nullptr entry
};
constexpr int MAX_LANES = 8;
constexpr int MAX_FURTHER = 12;

// MSVehicle::DriveProcessItem
struct DriveItem {
    int link = -1;
    double vLinkPass = 0;
    double vLinkWait = 0;
    bool setRequest = false;
    Time arrivalTime = 0;
    double arrivalSpeed = 0;
    double arrivalSpeedBraking = 0;
    double distance = 0;
    double accelV = -1;
    bool hadStoppedVehicle = false;
    double availableSpace = 0;

    void adaptLeaveSpeed(double v) { accelV = accelV < 0 ? v : std::min(accelV, v); }
    void adaptStopSpeed(double v) { vLinkWait = std::min(vLinkWait, v); }
    double leaveSpeed() const { return accelV < 0 ? vLinkPass : accelV; }
};

// MSLink::ApproachingVehicleInformation
struct Approach {
    int veh;
    Time arrivalTime;
    Time leavingTime;
    double arrivalSpeed;
    double leaveSpeed;
    bool willPass;
    double arrivalSpeedBraking;
    Time waitingTime;
    double dist;
    double speed;
    int lane;  // lane of the vehicle when registering (zipper merging reads it in parallel)
    double latOffset = 0;  // MSLink::ApproachingVehicleInformation::latOffset (sublane model)
};

// hot state touched by every neighbour lookup
struct Kin {
    double pos = 0;
    double speed = 0;
    double accel = 0;    // MSVehicle::myAcceleration
    double backPos = 0;  // MSVehicle::State::myBackPos (position of the back on the last further lane)
    int lane = -1;
    int type = 0;
    int nFurther = 0;
    int further[MAX_FURTHER];  // MSVehicle::myFurtherLanes (nearest first)
    // lateral state (continuous lane changes and the sublane model): MSVehicle::State::myPosLat (offset of the
    // center from the lane center, left positive), the shadow lane and its further lanes
    double posLat = 0;
    int shadowLane = -1;
    int nShadowFurther = 0;
    int shadowFurther[MAX_FURTHER];
};

// lateral positions only the vehicle itself needs (allocated in sublane mode): MSVehicle::myFurtherLanesPosLat,
// MSAbstractLaneChangeModel::myShadowFurtherLanesPosLat, myNoPartiallyOccupatedByShadow
struct LatPos {
    double furtherPosLat[MAX_FURTHER];
    double shadowFurtherPosLat[MAX_FURTHER];
};
struct LatState : LatPos {
    std::vector<int> noShadow;
};

struct LCAdvice {
    double accel;
    int flag;
};

// LC2013 + MSAbstractLaneChangeModel state
struct LCState {
    int ownState = 0;
    int previousState = 0;
    int canceled[3] = {0, 0, 0};
    bool dontBrake = false;
    bool changed = false;          // MSAbstractLaneChangeModel::myAlreadyChanged
    bool strategicBlocked = false; // isStrategicBlocked (from the saved states of the last right and left checks)
    int savedStateRight = 0, savedStateLeft = 0;  // MSAbstractLaneChangeModel::mySavedStateRight / Left .second
    long long speedGainProbRight = 0;
    long long speedGainProbLeft = 0;
    long long keepRightProb = 0;
    double leadingBlockerLength = 0;
    double leftSpace = 0;
    double lookAheadSpeed = 0;
    std::vector<LCAdvice> advices;
    // continuous lane changing (--lanechange.duration): MSAbstractLaneChangeModel::myLaneChangeCompletion (1: not
    // changing), the direction, the full lateral distance, the lateral speed and the lateral position on the lane
    double completion = 1;
    int lcDirection = 0;
    double maneuverDist = 0, speedLat = 0;
    bool isChangingLanes() const { return completion < 1 - NUMERICAL_EPS; }
    bool pastMidpoint() const { return completion >= 0.5; }
};

// MSLCM_SL2015 state and the parts of MSAbstractLaneChangeModel / MSVehicle only the sublane model uses
// (Sim::sl, allocated with a lateral resolution)
struct SLState {
    double speedGainProbRight = 0, speedGainProbLeft = 0, keepRightProb = 0;
    int lastEdge = -1;                // myLastEdge (of myExpectedSublaneSpeeds)
    bool canChangeFully = true;
    double safeLatDistRight = 0, safeLatDistLeft = 0;
    double impatience = 0;            // myImpatience (starts at lcImpatience)
    double sigmaState = 0;            // lateral drift (lcSigma)
    double committedSpeed = 0;        // MSAbstractLaneChangeModel::myCommittedSpeed
    double previousManeuverDist = 0;  // myPreviousManeuverDist
    double lastLateralGapLeft = 0, lastLateralGapRight = 0;
    double accelerationLat = 0;
    bool cfRelatedReady = false;
    std::vector<int> cfRelated;       // myCFRelated (vehicles in a car-following relation)
    std::vector<double> expectedSublaneSpeeds;
    double nextTurnDist = 0;          // MSVehicle::myNextTurn (planMoveInternal)
    int nextTurnLink = -1;
};

enum class VehStatus : uint8_t { PENDING, RUNNING, ARRIVED, TELEPORTING, PARKING, REMOVED };

constexpr double INVALID_POSLAT = std::numeric_limits<double>::max();  // (INVALID_DOUBLE)

// MSStop (bus stops only)
struct Stop {
    int lane = -1;
    int routeIdx = -1;      // route index of the stop edge
    double startPos = 0, endPos = 0;
    Time duration = 0;
    Time until = -1;
    bool parking = false;
    int busStop = -1;
    int parkingArea = -1;
    double speed = 0;       // a waypoint: passed at this speed (MSStop::getSpeed)
    Time jump = -1;         // after the stop the vehicle jumps to the next route edge (SUMOVehicleParameter::Stop::jump)
    Time jumpUntil = -1;
    // transportables (MSStop): the stop waits for persons / containers, awaited and permitted ids
    bool triggered = false, containerTriggered = false;
    Time extension = -1;    // boarding ends this long after the stop duration (endBoarding)
    bool onDemand = false;  // skipped when nobody leaves or enters (then passed like a waypoint at any speed)
    bool skipOnDemand = false;
    std::vector<std::string> awaitedPersons, permitted, awaitedContainers;
    int numExpectedPerson = 0, numExpectedContainer = 0;
    Time timeToBoardNextPerson = 0, timeToLoadNextContainer = 0;
    bool reached = false;
    Time endBoarding = 0;
    Time started = -1;      // time the stop was reached (stop-output)
    std::string tripId;     // the trip id of the vehicle from this stop on (rail signal constraints)
    Time arrival = -1;      // the planned arrival (SUMOVehicleParameter::Stop::arrival)
    std::string split, join;  // the train part split off here / the train this one joins
    int splitVeh = -1, joinVeh = -1;  // (their vehicle indices, resolved after loading)
    bool joinTriggered = false;  // waits until another train joins (MSStop::joinTriggered)
    double priority = -1;   // >= 0: may be skipped or moved to a stopping place of the same name after rerouting
    int skipLooped = 0;     // occurrences of the stop edge in a looped route to skip (an index beyond the stops)
    Time startedGiven = -1, flexArrival = -1;  // the given 'started' attribute and flexArrival parameter
    double posLat = INVALID_POSLAT;  // the lateral position while stopped (SUMOVehicleParameter::Stop::posLat)
};

// MSCFModel::VehicleVariables of the car-following models with state, and MSVehicle::myTimeSinceStartup
// (Sim::cfVars, allocated when a vehicle type needs them; written only by the vehicle's own task)
// the vehicle whose own planning or speed decision runs on this thread: models with state that SUMO also
// changes in followSpeed (SmartSK) change it only for this vehicle
extern thread_local int cfOwner;
// MSGlobals::gComputeLC: true while this thread computes lane changes
extern thread_local bool cfComputeLC;
// the edge whose lane changer runs on this thread (its vehicles are read live, see Sim::snapRead)
extern thread_local int lcOwnEdge;

struct CFVars {
    Time timeSinceStartup = 86400000;  // MSVehicle::myTimeSinceStartup (startupDelay)
    double accelDawdle = 1e6;          // Krauss with sigmaStep
    Time updateOffset = 0;
    double levelOfService = 1;         // IDMM
    double accelSign = 1;              // Wiedemann
    double rnd = 0;                    // BKerner
    double aOld = 0;                   // PWagner2009
    double gOld = 0, headway = 0;      // SmartSK
    std::map<int, double> ggOld;
    int accControlMode = 0, caccControlMode = 0, caccCommMode = 0;  // ACC, CACC
    Time accLastUpdate = 0;
    // EIDM
    double minaccel = 100, wouldacc = 100, lastacc = 0, realacc = 100, lastrealacc = 0, realleaderacc = 100, lastleaderacc = 0;
    double v0_int = 0, v0_old = 0, t_off = -10., w_gap = 0, w_speed = 0, w_error = 0, v_est_l = 0, v_est = 0, s_est = 0;
    bool respectMinGap = true;
    int ap_update = 0;
    std::vector<std::pair<double, double>> stop;
    std::vector<std::pair<double, double>> odometerAngles;  // Rail curve resistance
};

// one finished stop (SUMO stop-output)
struct StopRecord {
    int busStop = -1;
    int lane = -1;
    double pos = 0;
    bool parking = false;
    Time started = 0, ended = 0;
};

// InsertionCheck (SUMOXMLDefinitions.h)
enum InsertionCheck : int {
    IC_NONE = 0, IC_COLLISION = 1 << 0, IC_LEADER_GAP = 1 << 1, IC_FOLLOWER_GAP = 1 << 2, IC_JUNCTION = 1 << 3, IC_STOP = 1 << 4,
    IC_ARRIVAL_SPEED = 1 << 5, IC_ONCOMING_TRAIN = 1 << 6, IC_SPEED_LIMIT = 1 << 7, IC_PEDESTRIAN = 1 << 8, IC_BIDI = 1 << 9,
    IC_LANECHANGE = 1 << 10, IC_ALL = (1 << 11) - 1
};

// SUMO's departure / arrival procedures (SUMOVehicleParameter)
enum DepartLane : int { DL_FREE = -1, DL_BEST = -2, DL_FIRST = -3, DL_BEST_PROB = -4, DL_RANDOM = -5, DL_ALLOWED = -6 };
enum DepartSpeed : int { DS_GIVEN, DS_MAX, DS_DESIRED, DS_AVG, DS_RANDOM, DS_LIMIT, DS_LAST };
enum DepartPos : int { DP_BASE, DP_GIVEN, DP_RANDOM, DP_RANDOM_FREE, DP_FREE, DP_LAST, DP_STOP, DP_SPLIT_FRONT };
enum ArrivalPos : int { AP_MAX, AP_GIVEN, AP_RANDOM, AP_CENTER };
enum ArrivalLane : int { AL_CURRENT, AL_GIVEN, AL_RANDOM, AL_FIRST };

// parking search memory of a vehicle (StoppingPlaceMemory)
struct ParkingMemory {
    int area;
    Time blockedAtTime = -1, blockedAtTimeLocal = -1;
};

// rarely used per-vehicle data, allocated on first use: large demands keep Veh small. Only the vehicle's own
// task creates or changes it (like the other per-vehicle state).
struct VehExtra {
    std::vector<int> via;                    // via edges of a trip
    std::vector<StopRecord> stopLog;         // finished stops (only with --stop-output)
    std::vector<int> rrActive;               // rerouters whose lane reminder the vehicle holds
    std::vector<int> dwHeld;                 // driveways whose reminder the train holds (rail.cpp)
    std::vector<std::string> parkingBadges;  // parking areas (parking.cpp)
    std::vector<std::pair<std::string, double>> parkingParams;  // parking.* parameters of the vehicle or its type
    std::vector<ParkingMemory> parkingMemory;
    // MSVehicle::WaitingTimeCollector (only kept when needed): waiting intervals (time since the end, time
    // since the begin), the newest last
    std::vector<std::pair<Time, Time>> waitIntervals;
    Time lastLaneEntryTime = -1;  // MSDevice_Routing::myLastLaneEntryTime (--weights.separate-turns)
    int jumpEdge = -1;            // the edge of the last stop with jump and the time the jump ends
    Time jumpProceed = -1;
    std::vector<int> riders;      // MSDevice_Transportable::myTransportables (persons)
    bool ridersStopped = false;   // MSDevice_Transportable::myStopped
    std::vector<int> cargo;       // the containers (the container device)
    bool cargoStopped = false;
    std::string line;             // SUMOVehicleParameter::line
    double otPrio = -1, otMaxDelay = -1;  // overtakingReroute.prio / .maxDelay parameters (-1: not given)
    double crMaxDelay = -1;               // closingReroute.maxDelay (-1: not given)
    int arrivalIdx = -1;          // arrives on this route index (a train part that joined another one)
    int fullRoute = -1;           // with arrivalEdge: the whole route (driveways are built along it), until rerouted
    std::vector<int> dwOldRoute;  // the route before a rerouter changed it in this move (driveway entries before it)
    int insertionChecks = -1;     // the vehicle's insertionChecks (-1: the option)
    // carFollowModel.ignoreIDs / ignoreTypes, junctionModel.ignoreIDs / ignoreTypes (vehicle parameters)
    std::vector<std::string> cfIgnoreIDs, cfIgnoreTypes, jmIgnoreIDs, jmIgnoreTypes;
    // departPosLat / arrivalPosLat (DepartPosLatDefinition, ArrivalPosLatDefinition)
    int departPosLatProc = 0, arrivalPosLatProc = 0;
    double departPosLat = 0, arrivalPosLat = 0;
};
enum DepartPosLatDef { DPL_DEFAULT, DPL_GIVEN, DPL_RIGHT, DPL_CENTER, DPL_LEFT, DPL_RANDOM, DPL_FREE, DPL_RANDOM_FREE };
enum ArrivalPosLatDef { APL_DEFAULT, APL_GIVEN, APL_RIGHT, APL_CENTER, APL_LEFT };

// owning pointer to VehExtra that copies deeply (vehicles of a flow are copies of its prototype)
struct VehExtraPtr {
    std::unique_ptr<VehExtra> p;
    VehExtraPtr() = default;
    VehExtraPtr(const VehExtraPtr& o) : p(o.p ? new VehExtra(*o.p) : nullptr) {}
    VehExtraPtr(VehExtraPtr&&) noexcept = default;
    VehExtraPtr& operator=(const VehExtraPtr& o) {
        p.reset(o.p ? new VehExtra(*o.p) : nullptr);
        return *this;
    }
    VehExtraPtr& operator=(VehExtraPtr&&) noexcept = default;
};
inline const VehExtra NO_VEH_EXTRA;

struct Veh {
    // (fields ordered to avoid padding: Veh exists for every vehicle of the demand)
    // static
    std::string id;
    Time depart = 0;
    int route = -1;
    // departure / arrival definitions (demand.cpp); SUMO 1.28 defaults
    int departLane = DL_BEST_PROB;    // >= 0 given lane index, otherwise a DepartLane value
    int departSpeedMode = DS_AVG;
    int departPosMode = DP_BASE;
    double departSpeed = 0;
    double departPos = 0;
    double speedFactor = 1;
    double arrivalPosGiven = 0;
    double arrivalPos = -1;           // resolved after loading
    double arrivalSpeed = -1;         // given arrival speed, -1 = current
    int arrivalPosMode = AP_MAX;
    int arrivalLaneMode = AL_CURRENT;
    int arrivalLane = -1;             // resolved lane index on the arrival edge, -1 = any
    SVCPermissions vclass = VC_PASSENGER;
    std::vector<Stop> stops;   // remaining stops (front is the next one)
    // dynamic
    VehStatus status = VehStatus::PENDING;
    bool haveToWaitOnNextLink = false;
    int routeIdx = 0;          // MSVehicle::myCurrEdge
    int departEdge = 0;        // SUMOVehicleParameter::departEdge (the route index of the departure)
    double prevSpeed = 0;
    Time waitingTime = 0;      // MSVehicle::myWaitingTime
    double timeLoss = 0;
    double stopSpeed = DMAX;   // myStopSpeed
    double stopDist = DMAX;    // myStopDist
    Time junctionEntryTime = TIME_MAX;
    Time junctionEntryTimeNeverYield = TIME_MAX;
    Time junctionConflictEntryTime = TIME_MAX;
    std::vector<DriveItem> lf;  // myLFLinkLanes
    int nextDriveItem = 0;
    int jcPassed = -1;  // an internal lane checked for junction collisions the front passed in this step
    // action steps (MSVehicle::myLastActionTime, myActionStep); the speed, waiting time and link stopped for at the
    // last action (the link approaches registered then stay until the next action)
    Time lastActionTime = 0;
    bool actionStep = true;
    int actionHaveStoppedFor = -1;
    Time actionWaitingTime = 0;
    double actionSpeed = 0;
    // sublane model: the lateral position at the last action (the approaches' latOffset) and the shadow lane's
    // direction and lateral offset (approaches of the parallel links)
    double actionPosLat = 0, actionShadowLatOffset = 0;
    int8_t actionShadowDir = 0;
    int haveStoppedFor = -1;   // link
    int nEnteredNormal = 0;  // normal lanes entered in the last move (edge data)
    int enteredNormal[4];
    // best lanes
    std::vector<std::vector<LaneQ>> bestLanes;
    int lastBestLanesEdge = -1;          // edge index
    int lastBestLanesInternalLane = -1;
    int currentLaneInBestLanes = 0;
    // movement bookkeeping of the current step
    int oldLane = -1;
    double oldPos = 0;
    // lane changing
    LCState lc;
    // teleport / parking bookkeeping (MSVehicleTransfer)
    Time transferTime = -1;
    Time proceedTime = -1;
    // statistics
    double departTime = -1, arrivalTime = -1, routeLength = 0, tripWaiting = 0;
    double departSpeedActual = 0;
    double departPosActual = 0;
    double stopTime = 0;
    double lastStepDist = 0;     // MSVehicle::State::myLastCoveredDist
    int waitingCount = 0;
    int departLaneActual = -1;
    int nLaneChanges = 0;
    int nTeleports = 0;
    bool tripAmWaiting = false;  // MSDevice_Tripinfo::myAmWaiting
    bool emergencyStop = false;  // set in the move phase, counted afterwards
    // Detectors are move reminders of the vehicle in SUMO. They only differ from "the detectors of the
    // lanes the vehicle touches" around teleports: a teleporting vehicle holds the reminders of the lane
    // it entered virtually (teleportLoopLane); when the teleport ends on another lane of that edge, these
    // reminders replace those of the new lane (aliasLane) until the vehicle leaves it (enterLaneAtInsertion
    // with NOTIFICATION_TELEPORT adds no reminders).
    int teleportLoopLane = -1;
    int aliasLane = -1, aliasLoopLane = -1;
    // further lanes whose reminders (detectors, edge data) the vehicle does not hold: after a lane change
    // (enterLaneAtLaneChange) or an insertion (enterLaneAtInsertion) only the reminders of the entered lane
    // are added, not those of the lanes the back occupies
    int nNoRem = 0;
    int noRem[MAX_FURTHER];
    // vehicle routing (routing.cpp, MSDevice_Routing)
    double deviceProb = -1;      // device.rerouting.probability of the type (-1: not given)
    double periodParam = -1;     // device.rerouting.period of the vehicle or its type (-1: not given)
    Time reroutePeriod = 0;
    Time rerouteNextPre = NO_EVENT;  // the pre-insertion reroute event's time (-1: when built, NO_EVENT: none)
    int preRank = -1;            // position among this step's pre-insertion reroutes (rail driveway updates)
    Time preRankStep = -1;
    Time skipRoutingAt = -1;     // MSDevice_Routing::skipRouting: no pre-insertion reroute in this step
    Time rerouteNext = -1;       // time of the next periodic reroute (-1: none)
    Time rerouteLast = -1;       // MSDevice_Routing::myLastRouting
    int deviceParam = -1;        // has.rerouting.device of the vehicle or its type (-1: not given)
    int nReroutes = 0;           // MSBaseVehicle::myNumberReroutes
    int fromTaz = -1, toTaz = -1;  // trips between zones (fromTaz / toTaz, fromJunction / toJunction)
    bool forceReroute = false;   // VEHPARS_FORCE_REROUTE: a trip (from, via, to), routed before the insertion
    bool tripinfoDevice = true;  // has.tripinfo.device (tripinfo-output)
    bool initReplaced = false;   // SUMO's loaded route of the trip differs from any computed one (counts in rerouteNo)
    bool fromFlow = false;       // generated at its departure (a flow), not loaded in advance
    bool routingDevice = false;
    bool rerouteAfterStop = false;
    bool routeError = false;
    bool insertionOrderMark = false;  // an insertion order constraint failed (the lane parameter "insertionOrder<id>")
    int8_t routingMode = -1;     // device.rerouting.mode of the vehicle or its type (libsumo::ROUTING_MODE_*, -1: the option)
    bool jumpPending = false;    // a stop with jump ended in this step (MSBaseVehicle::isJumping)
    bool departTriggered = false;  // depart="triggered": inserted when a person boards
    bool departSplit = false;      // depart="split": inserted when the train it splits from reaches its stop
    bool registeredWaiting = false;  // MSVehicle::myAmRegisteredAsWaiting (a triggered stop waits)
    bool jumping = false;        // in the vehicle transfer after a stop with jump
    uint32_t randomSeed = 0;     // MSBaseVehicle::myRandomSeed (murmur3 of the id and SUMO's seed)
    std::string tripId;          // the tripId parameter (empty: the vehicle id)
    // parking areas (parking.cpp)
    int parkingReroutes = 0;     // MSBaseVehicle::myNumberParkingReroutes
    bool parkingSignal = false;  // a blinker is on while parking (MSVehicleTransfer: egress wish signalled)
    VehExtraPtr extra;
    VehExtra& ex() {  // created on first use
        if (!extra.p) {
            extra.p.reset(new VehExtra());
        }
        return *extra.p;
    }
    const VehExtra& exr() const { return extra.p ? *extra.p : NO_VEH_EXTRA; }
    VehExtra* exq() { return extra.p.get(); }
    // MSBaseVehicle::ignoreTransientPermissions (ROUTING_MODE_IGNORE_TRANSIENT_PERMISSIONS)
    bool ignoreTransientPermissions() const { return routingMode > 0 && (routingMode & 8) != 0; }
};

// ---------------------------------------------------------------------------
// lanes at runtime
// ---------------------------------------------------------------------------
struct LaneRT {
    std::vector<int> vehs;      // rear (0) to front (back()), like MSLane::myVehicles
    std::vector<int> partials;  // vehicles whose back reaches onto this lane, sorted by position on this lane
    double brutto = 0;          // MSLane::myBruttoVehicleLengthSum
    double netto = 0;
    std::vector<int> outbox;    // vehicles that moved onto another lane in the move phase
    std::vector<int> stay;      // vehicles that stay on the lane (rear to front)
    std::vector<int> arrived;
    std::vector<int> arrivedLaneVehicles;  // the lane's vehicle number when each arrived (MSLane::executeMovements)
    std::vector<int> parked;
    std::vector<int> incoming;  // vehicles entering from other lanes (MSLane::myVehBuffer)
    int teleportVeh = -1;       // vehicle to teleport (decided in the move phase)
    int nEmergency = 0;         // vehicles with an emergency stop (counted in parallel)
    char teleportReason = 0;    // 'w' wrong lane, 'y' yield, 'j' jam
};

struct EdgeRT {
    double lastBlockedBackPos[MAX_LANES];
    double lastBlockedWaiting[MAX_LANES];
    EdgeRT() {
        for (int i = 0; i < MAX_LANES; ++i) {
            lastBlockedBackPos[i] = -1;
            lastBlockedWaiting[i] = -1;
        }
    }
};

// induction loop of an actuated traffic light (MSInductLoop, the parts used for actuation)
struct InductLoop {
    int lane = -1;
    double pos = 0;
    double endPos = 0;
    std::vector<std::pair<int, double>> onDet;  // (vehicle, entry time)
    double lastLeaveTime = 0;
};

// a vehicle's contribution to the edge data of a lane in one step (MSMeanData_Net)
struct EdSample {
    int lane;
    double sampled, dist, wait;
};

struct DetEvent {
    int det;
    int veh;
    int kind;     // 0 enter, 1 leave (passed), 2 leave (left the lane), 3 erase
    double time;
};

struct InductLoopInfo {
    int loop;
    int lane;
    double maxGap;
    double jamThreshold;
    Time lastGreenTime = 0;
    std::vector<bool> servedPhase;
};

// traffic light program state (MSSimpleTrafficLightLogic / MSActuatedTrafficLightLogic)
struct TLSRT {
    int step = 0;
    std::vector<Time> lastSwitch;
    std::vector<Time> lastEnd;
    Time nextSwitch = 0;
    bool actuated = false;
    double maxGap = 3.0, passingTime = 1.9, detectorGap = 2.0, jamThreshold = -1;
    Time inactiveThreshold = 180000;
    std::vector<InductLoopInfo> loops;
    std::vector<std::vector<int>> loopsForPhase;  // indices into loops
    Time lastTrySwitchTime = 0;
    // rail crossing (MSRailCrossing): phases G, y, r, u
    bool railCrossing = false;
    Time timeGap = 15000, minGreenTime = 5000, openingDelay = 3000, openingTime = 3000, yellowTime = 5000;
    double spaceGap = -1;
};

struct LCMessage {
    int receiver;
    int kind;  // 0 = inform (advice + state flags), 1 = blocker length
    double value;
    int flags;
};

struct LaneChangeEvent {
    double time;
    int veh;
    int from;
    int to;
    double pos;
    double speed;
    int reason;
};

struct BusStop {
    std::string id;
    int lane = -1;
    double startPos = 0, endPos = 0;
    double parkingLength = 0;
    double parkingFactor = 1;  // MSStoppingPlace::myParkingFactor
    // occupied intervals (vehicle -> (begin, end)) of stopped vehicles: MSStoppingPlace::myEndPositions
    std::vector<std::pair<int, std::pair<double, double>>> endPositions;
    double lastFreePos = 0;
    int lastParking = -1;
    // persons waiting at the stop (MSStoppingPlace::myWaitingTransportables, myTransportableCapacity)
    int waitingCapacity = 6;
    double angle = M_PI / 2;  // MSStoppingPlace::myAngle (waiting persons face this way relative to the lane)
    int numWaitingPersons = 0;
    std::vector<char> spotUsed;  // MSStoppingPlace::myWaitingSpots (free spots), built with waitingCapacity
    bool checkPersonCapacity = true;  // bus and train stops
    char category = 'B';  // B: bus / train stop, C: container stop, E: charging station (MSNet::myStoppingPlaces)
    struct Access {       // MSStoppingPlace::Access (the access from another pedestrian lane)
        int lane;
        double startPos, endPos, length;
        int exit = 0;     // AccessExit: 0 platform, 1 doors, 2 carriage
    };
    std::vector<Access> accesses;
    std::string name;      // MSStoppingPlace::myName (alternatives with the same name)
    int loadOrder = 0;     // position in the additional files
};

// MSParkingArea. The state is changed by the vehicles on the parking area's lane (planMove: reservations,
// executeMove: entering and leaving), so the parallel move phase changes it only in the task of that lane;
// vehicles on other lanes read the step-begin snapshot (snap*), the other changes happen in serial phases.
struct ParkingLot {      // MSParkingArea::LotSpaceDefinition
    int vehicle = -1;
    double endPos = 0;
    Pos position;           // where a parked vehicle is drawn, and its rotation in degrees
    double rotation = 0;
};

struct ParkingArea {
    std::string id;
    int lane = -1;
    double begPos = 0, endPos = 0;
    int roadsideCapacity = 0;
    int capacity = 0;
    bool onRoad = false;
    bool reservable = false;
    double width = 0, length = 0, angle = 0;
    double departPos = -1;             // DepartPosDefinition::GIVEN (-1: not given)
    std::vector<std::string> badges;   // acceptedBadges (empty: all vehicles)
    std::vector<ParkingLot> lots;      // mySpaceOccupancies
    std::vector<int> parked;           // myEndPositions (the vehicles in the area)
    int lastFreeLot = -1;
    double lastFreePos = 0;
    bool egressBlocked = false;
    double maxVehLength = 0;
    int numAlternatives = 0;
    // getLastFreePosWithReservation
    Time reservationTime = -1, lastReservationTime = -1;
    int reservations = 0, lastReservations = 0;
    double reservationMaxLength = 0, lastReservationMaxLength = 0;
    std::vector<int> reservedVehicles;          // myReservedVehicles
    std::vector<int> remoteReserved;            // myRemoteReservedVehicles (rerouters)
    std::vector<int> lastRemoteReserved;
    int lastStepOccupancy = 0;
    bool updatePending = false;                 // myUpdateEvent (updateOccupancy at the end of the step)
    // step-begin snapshot for the vehicles on other lanes
    int snapOccupancy = 0;
    int snapReservations = -1;                  // reservations of the previous step (-1: none)
    double snapReservationMaxLength = 0;
    int occupancy() const { return (int)parked.size() - (egressBlocked ? 1 : 0); }
};

// stop events of the move phase, applied afterwards in vehicle order
struct StopEvent {
    int veh;
    int busStop;
    bool enter;   // enter or leave
    double pos;   // vehicle position when entering
    double length = -1;  // vehicle length when entering (-1: the length of its type; a train may split meanwhile)
};

// SUMOVehicleParameter::parseInsertionChecks (demand.cpp)
int parseInsertionChecks(const std::string& s);

// the departure / arrival definitions of a vehicle (demand.cpp)
void parseVehicleDefinitions(Veh& x, const std::string& dl, const std::string& ds, const std::string& dp,
                             const std::string& ap, const std::string& al, const std::string& as);

struct Options {
    uint64_t seed = 42;
    int threads = 1;
    double begin = 0;
    double end = -1;
    double timeToTeleport = 300;
    double timeToTeleportHighways = 0;
    double ignoreJunctionBlocker = -1;  // seconds, -1 = never
    bool checkJunctions = false;         // --collision.check-junctions
    double checkJunctionsMinGap = 0;     // --collision.check-junctions.mingap
    double timeToImpatience = 180;
    std::string outPrefix = "out";
    bool lcOutput = true;
    bool gzip = false;  // the streamed outputs (fcd, person fcd, lane changes) are written gzip-compressed
    bool fcd = false;
    bool tripinfoUnfinished = false;
    bool withTaz = false;  // --device.rerouting.with-taz (--with-taz)  // --tripinfo-unfinished (SUMO --tripinfo-output.write-unfinished)
    bool stopOutput = false;  // stopinfo.csv (SUMO stop-output)
    bool railsignalBlockOutput = false;  // railsignal_blocks.xml (SUMO railsignal-block-output)
    bool deadlockOutput = false;         // deadlocks.xml (SUMO deadlock-output)
    bool railsignalVehicleOutput = false;  // railsignal_vehicles.xml (SUMO railsignal-vehicle-output)
    bool sumoOrder = false;
    bool lcLevels = false;  // lane changing in stages by edge level (downstream first)
    bool lcSingle = false;  // lane changing in one stage (fastest, neighbouring edges may act on stale partial occupants)
    bool lcDynColor = false;  // colors of the lane-changing stages per step, among the edges with vehicles
    std::string trace;
    bool deterministicSpeedFactor = false;  // use the mean speed factor (validation runs)
    double edgedataPeriod = 0;  // seconds, 0 = no edge data (SUMO edgeData)
    std::string editsFile;      // runtime edits by time (see edit.cpp)
    bool extrapolateDepartPos = false;  // --extrapolate-departpos: compensate sub-step departure delays
    double summaryPeriod = 60;  // seconds between rows of summary.csv (SUMO summary-output)
    // SUMO options of the pedestrian model (--pedestrian.striping.*, --pedestrian.timegap-crossing)
    std::vector<std::pair<std::string, std::string>> pedOptions;
    // vehicle routing (--device.rerouting.*, --weights.*)
    double rerouteProb = -1;        // equip probability, -1: only trips
    std::vector<std::string> rerouteExplicit;  // --device.rerouting.explicit
    double reroutePeriod = 0;       // seconds, 0: no periodic rerouting
    bool rerouteRailSignal = false;  // --device.rerouting.railsignal: trains waiting at a red signal reroute
    double reroutePrePeriod = 60;
    double routeSteps = 200;  // --route-steps (vehicles are built that far ahead; <= 0: all at the start)
    double adaptInterval = 1;
    int adaptSteps = 180;
    double adaptWeight = 0;
    double rerouteThresholdFactor = 1;
    int routingMode = 0;  // --device.rerouting.mode
    int sumoSeed = 23423;  // SUMO's --seed (its hash randomness: vehicle random seeds)
    double weightsRandomFactor = 1;  // --weights.random-factor
    bool weightsRandomDynamic = false;  // --weights.random-factor.dynamic
    double weightsPriorityFactor = 0;  // --weights.priority-factor
    double weightsSeparateTurns = 0;   // --weights.separate-turns
    double stopTolerance = 10;         // --ride.stop-tolerance
    bool initWithLoadedWeights = false;  // --device.rerouting.init-with-loaded-weights
    bool bikeSpeeds = false;           // --device.rerouting.bike-speeds
    double rerouteThresholdTime = 0;
    bool rerouteSynchronize = false;
    bool ignoreRouteErrors = false;  // --ignore-route-errors
    bool emergencyInsert = false;     // --emergency-insert
    double defaultActionStepLength = 0;       // --default.action-step-length (0: the step length)
    bool defaultActionStepLengthSet = false;
    bool ballisticSet = false;                 // --step-method.ballistic was given
    bool overtakeRight = false;                // --lanechange.overtake-right
    Time maxDepartDelay = -1;                  // --max-depart-delay (-1: vehicles are never skipped)
    int maxNumVehicles = -1;                   // --max-num-vehicles (-1: no limit)
    bool tlsAllOff = false;                    // --tls.all-off
    bool useStopStarted = false;               // --use-stop-started
    bool removeGridlocked = false;             // --time-to-teleport.remove
    Time laneChangeDuration = 0;               // --lanechange.duration (continuous lane changing)
    double lateralResolution = 0;              // --lateral-resolution (the sublane model)
    Time startupWaitThreshold = 2000;  // --startup-wait-threshold (MSGlobals::gStartupWaitThreshold)
    double collisionMinGapFactor = -1;  // --collision.mingap-factor (-1: the car-following model's)
    // intermodal routing of person trips (--persontrip.*)
    std::vector<std::string> carWalk{"parkingAreas"};  // --persontrip.transfer.car-walk
    std::vector<std::string> taxiWalk, walkTaxi;        // --persontrip.transfer.taxi-walk / walk-taxi
    double taxiWait = 300;                              // --persontrip.taxi.waiting-time
    bool ridePublicLine = false;                        // --persontrip.ride-public-line
    double walkFactor = 0.75;                           // --persontrip.walkfactor
    double walkOppositeFactor = 1;                      // --persontrip.walk-opposite-factor
    double minorPenalty = 1.5;
    double turnaroundPenalty = 5.0;
    std::string collisionAction = "teleport";  // --collision.action: teleport, warn, none
    int insertionChecks = 2047;                // --insertion-checks (InsertionCheck bits, all)
    // rail signals (--railsignal.*, --time-to-teleport.railsignal-deadlock)
    double railMaxBlockLength = 2e4;
    double railMovingBlockMaxDist = 200;  // --railsignal.moving-block.max-dist
    double railMaxTrainLength = 1000;     // --railway.max-train-length
    double reversalPenalty = 60;          // --weights.reversal-penalty (negative: no reversals)
    double timeToTeleportRSDeadlock = -1;
    bool removeConstraint = false;  // --time-to-teleport.remove-constraint
    bool eagerInsert = false;       // --eager-insert
    bool railMovingBlock = false;
    SVCPermissions railSignalizedClasses = VC_RAIL | VC_RAIL_FAST | VC_RAIL_ELECTRIC | VC_RAIL_URBAN | VC_SUBWAY;
    SVCPermissions railMovingBlockClasses = VC_TRAM | VC_CABLE_CAR;
};

// ---------------------------------------------------------------------------
// rail signals (rail.cpp)
// ---------------------------------------------------------------------------
// MSDriveWay::Siding: a section of the route where a foe train can wait
struct Siding {
    int start, end;  // route indices
    double length;
    std::vector<int> intermediateEnds;  // route indices of intermediate rail signals
};

// MSDriveWay::VehicleEvent (railsignal-vehicle-output); reason: MSMoveReminder::Notification
enum DwReason : uint8_t { DWR_DEPARTED, DWR_JUNCTION, DWR_TELEPORT, DWR_PARKING, DWR_REROUTE, DWR_ARRIVED, DWR_TELEPORT_ARRIVED };
struct DwVehEvent {
    int veh;
    Time time;
    bool entry;
    uint8_t reason;
};

// MSDriveWay: the tracks used by a train from a rail signal (or its departure) to the next signal,
// with the tracks protecting it. Driveways are only created (never removed); `index` is the
// numerical id.
struct DriveWay {
    int index = -1;
    std::string id;
    int origin = -1;    // link of the rail signal, -1 for a departure driveway
    int lane = -1;      // first lane of the forward section (the reminder lane)
    int parent = -1;    // the driveway a sub-driveway belongs to
    std::vector<int> route;  // normal edges
    int coreSize = 0;
    int forwardEdgeCount = 0;
    bool foundSignal = false, foundJump = false, terminateRoute = false, abortedBuild = false, bidiEnded = false;
    std::vector<int> forward, bidi, bidiExtended, flank, conflictLanes;  // lanes
    std::vector<int> conflictLinks;
    std::vector<int> foes, subs;  // driveways
    std::vector<int> reversals;   // edges
    std::map<int, std::vector<Siding>> sidings;  // by foe driveway
    std::vector<std::set<int>> deadlocks;        // sets of driveways that must not all be occupied (deadlock hints)
    std::set<int> trains;         // vehicles on the driveway (by numerical id)
    int active = -1;
    std::string firstVehicle;
    std::vector<DwVehEvent> events;  // entries and exits (railsignal-vehicle-output)
};

struct RailLinkInfo {
    int link = -1;
    bool controlled = false;
    std::vector<int> driveways;
    int lastRerouteVehicle = -1;  // rerouting at the signal (--device.rerouting.railsignal)
    Time lastRerouteTime = 0;
};

struct RailSignal {
    int tls = -1;
    std::string id;
    std::vector<RailLinkInfo> links;  // per tl index
    std::string state;
    int driveWayIndex = 0;
    bool movingBlock = false;
    std::map<std::string, std::vector<int>> constraints;  // by constrained trip id (MSRailSignal::myConstraints)
};

// MSRailSignalConstraint_Predecessor: the train may only pass after the foe trip passed the foe signal
// (among the last `limit` trains)
struct RailConstraint {
    enum Type { PREDECESSOR, INSERTION_PREDECESSOR, FOE_INSERTION, INSERTION_ORDER, BIDI_PREDECESSOR };
    int type = PREDECESSOR;
    std::string foeTrip;
    int foeSignal = -1;
    int limit = 1;
    bool active = true;
    std::vector<int> trackers;
    bool isInsertionConstraint() const { return type == INSERTION_PREDECESSOR || type == INSERTION_ORDER; }
};

// MSRailSignalConstraint_Predecessor::PassedTracker: the trips that entered a lane (ring buffer)
struct PassedTracker {
    int lane = -1;
    std::vector<std::string> passed{""};
    int lastIndex = -1;
};

// MSRailSignalControl::WaitRelation
struct RailWaitRelation {
    int railSignal = -1;
    int foe = -1;
    int constraint = -1;
};

// a rail signal deadlock found while moving (MSRailSignalControl::haveDeadlock): the circle of wait relations
// (waiting train, relation), the constraint to deactivate (-1: none)
struct RailDeadlockFound {
    int resolved = -1;
    std::set<int> seen;
    std::vector<std::pair<int, RailWaitRelation>> circle;
};

// a train entering or leaving a driveway lane in the move phase (aux: route index when entering,
// the back lane afterwards when leaving)
struct DriveWayEvent {
    int veh;
    int lane;
    int aux;
    bool enter;
    bool oldRoute = false;  // entered before a rerouter on a later lane of the same move changed the route
};

// MSLink::ComparatorNumericalLaneIdLess
struct FlankLess {
    const Network* net;
    bool operator()(int a, int b) const {
        const Link& x = net->links[a];
        const Link& y = net->links[b];
        return x.lane < y.lane || (x.lane == y.lane && x.index < y.index);
    }
};
using FlankSet = std::set<int, FlankLess>;

struct LaneChanger;

// a flow whose vehicles psim2 generates itself (Poisson or probability flows)
struct FlowDef {
    std::string id;
    int type = -1;
    std::string typeName;  // a type or a type distribution (sampled per vehicle)
    int route = -1;
    std::string speedFactor = "-";  // given speedFactor of the vehicles ("-": from the type)
    Time begin = 0;
    Time end = 0;
    bool poisson = true;  // period="exp(value)", otherwise probability="value"
    double value = 0;
    long long number = -1;
    Veh proto;            // departure / arrival definitions
    std::vector<std::vector<std::string>> stops;
};

// normal distribution capped by resampling (Distribution_Parameterized::sample), demand.cpp
double sampleSpeedFactor(const VType& t, uint64_t seed, int id, uint64_t purpose);

// RouterProhibition (SUMOAbstractRouter): an edge closed for routing; with an end time the edge may be used, the
// waiting until the end counts as effort
struct RouterProhibition {
    int edge;
    SVCPermissions permissions;  // classes still allowed
    double end;                  // seconds (infinity: closed for good)
};

// a traffic assignment zone (taz, junction-taz): its source and sink connectors are routing nodes after
// the edges (MSEdge with SumoXMLEdgeFunc::CONNECTOR, numerical ids after the network's edges)
struct Taz {
    std::string id;
    std::vector<int> sources, sinks;  // edges reachable from the source / leading to the sink
};

struct ParkingSearch;  // parking.cpp
// the lane task of executeMovements running on this thread (position in activeLanes, trigger counter)
extern thread_local int execTaskOrder;
extern thread_local int execTaskLane;
extern thread_local int execTaskSeq;

struct LeaderInfo;
struct CriticalFollowerDistanceInfo;

class Sim {
public:
    explicit Sim(const Options& o) : opt(o), pool(o.threads) {}
    void load(const std::string& file);
    void run();

    Options opt;
    Network net;
    std::vector<VType> types;
    std::vector<std::vector<int>> routes;  // edge sequences
    std::vector<BusStop> busStops;
    std::unordered_map<std::string, int> busStopIdx;
    // MSNet::myNamedStoppingPlaces: (category, name) -> stopping places in load order
    std::map<std::pair<char, std::string>, std::vector<int>> namedStops;
    std::unordered_map<std::string, int> typeIdx, routeIdx;
    // demand (demand.cpp)
    std::vector<FlowDef> flows;
    int lastFlow = -1;  // STOP lines after a FLOW belong to it
    void addVehicle(Veh x, int type, const std::string& speedFactor);
    // vTypeDistribution: (type, probability) per distribution id
    std::unordered_map<std::string, std::vector<std::pair<int, double>>> typeDists;
    int resolveType(const std::string& id, uint64_t a, uint64_t b) const;
    void parseVehicleLine(const std::vector<std::string>& tok);
    void parseFlowLine(const std::vector<std::string>& tok);
    void parseStopLine(const std::vector<std::string>& tok);
    void parseVehicleOptions(const std::vector<std::string>& tok);
    void addStop(Veh& x, int type, const std::vector<std::string>& tok, Time departDesired);
    int stopRouteIndex(const Veh& x, int type, const std::vector<int>& r, const Stop& s, const Stop* prevStop) const;
    void generateFlows();
    void resolveArrival(int v);
    bool lastInsertion(int v, int lane, double speed, bool patchSpeed);
    std::vector<Veh> veh;
    std::vector<Kin> kin;
    std::vector<LaneRT> lanes;
    std::vector<EdgeRT> edgesRT;
    std::vector<std::vector<Approach>> approaching;  // per link, sorted by vehicle
    struct ApproachRec {
        int link;
        Approach ap;
    };
    std::vector<int> approachMoved;                   // links that received a moved registration
    std::vector<std::vector<ApproachRec>> approachBuckets;  // per (chunk, link partition)
    std::vector<std::vector<int>> approachTouchedOf;  // per link partition: links with registrations
    int approachParts = 0, approachNLinks = 0;
    std::vector<TLSRT> tlsRT;
    std::vector<InductLoop> loops;
    std::vector<std::vector<int>> laneLoops;  // induction loops per lane
    ThreadPool pool;

    Time now = 0;  // MSNet::myStep (start of the current step)
    int step = 0;

    // MSVehicleControl: running extremes over departed vehicles, they bound
    // MSLane::getMaximumBrakeDist
    double maxSpeedFactor = 1;
    double minDecel = 4.5;  // SUMOVTypeParameter::getDefaultDecel(SVC_IGNORING)
    double maxMinGap = 0;
    void vehicleDeparted(int v);
    void detectCollisions(const char* stage);
    // collisions inside junctions (jcollision.cpp)
    bool mustCheckJunctionCollisions(int lane) const;
    Shape vehBoundingBox(int v, double offset) const;
    Shape vehBoundingPoly(int v, double offset) const;
    void junctionCollisions(int lane, std::vector<std::pair<int, int>>& out) const;
    std::vector<int> colCheck;
    std::vector<std::vector<int>> colFound, colVictims;
    std::vector<std::vector<std::string>> colRows;
    std::vector<std::string> collisionRows;  // collisions.csv
    void updateMaxBrakeDist();

    std::vector<int> pending;   // vehicles waiting for insertion (depart order)
    size_t nextToLoad = 0;
    // SUMORouteLoaderControl::loadNext: the next vehicle to build (route file order), when the next load happens
    size_t buildCursor = 0;
    Time nextBuildTime = -TIME_MAX;
    Time routeLastDepart = -1;  // SUMORouteHandler::myLastDepart
    void loadRoutesStep();
    void vehicleBuilt(int v);
    std::vector<int> running;   // vehicles on the network (any order)
    std::vector<int> transfer;  // teleporting or parking vehicles (MSVehicleTransfer)
    std::vector<char> insertFailed;  // MSEdge::myFailedInsertionMemory (per lane, current step)
    std::vector<int> insertFailedLanes;
    std::vector<int> insertFailedOnEdge;  // failed lanes per edge (current step)
    std::vector<int> activeLanes;    // lanes with vehicles (sorted)
    std::vector<char> laneActive;
    std::vector<std::vector<int>> scanChunks;
    void collectOccupiedLanes(std::vector<int>& out);
    void compactRunning();
    std::vector<int> partialLanes;   // lanes with partial occupants
    std::vector<std::atomic_flag> laneLock;
    std::vector<char> laneTouched;
    std::vector<std::vector<int>> touchedOfChunk;
    std::vector<std::vector<DetEvent>> laneDet;
    std::vector<std::vector<StopEvent>> laneStop;
    std::vector<std::vector<EdSample>> laneEd;
    void updateActiveLanes();

    // statistics
    int nArrived = 0, nTeleported = 0, nTeleJam = 0, nTeleYield = 0, nTeleWrongLane = 0, nEmergencyStops = 0;
    int nCollisions = 0;
    long long nLCStages = 0;  // parallel lane-changing stages over all steps  // collisions (all teleported, collision.action teleport)
    int nInserted = 0;
    std::vector<LaneChangeEvent> lcEvents;
    std::vector<std::string> tripRows;
    double tTLS = 0, tPlan = 0, tApproach = 0, tExec = 0, tIntegrate = 0, tLC = 0, tInsert = 0, tOther = 0;
    double tLcSched = 0, tLcSerial = 0, tLcPartials = 0;  // parts of the lane-changing phase run by the main thread
    long long nLcPartialRebuilds = 0, lcStageEdges = 0;

    // ---- car-following models (cfmodels.cpp): MSCFModel's virtual methods for vehicle v ----
    std::vector<CFVars> cfVars;  // per vehicle when a type has a model with state or a startup delay
    std::vector<LatState> lat;   // per vehicle in sublane mode
    std::vector<SLState> sl;     // per vehicle with a lateral resolution
    // MSAbstractLaneChangeModel::saveLCState (the state after TraCI) and isStrategicBlocked
    void saveLCState(int v, int laneOffset, int state) {
        LCState& lc = veh[v].lc;
        if (laneOffset == -1) {
            lc.savedStateRight = state;
        } else if (laneOffset == 1) {
            lc.savedStateLeft = state;
        }
        const int blk = LCA_STRATEGIC | LCA_BLOCKED;
        lc.strategicBlocked = (lc.savedStateRight & blk) == blk || (lc.savedStateLeft & blk) == blk;
    }
    void resetSavedLCStates(int v) {
        LCState& lc = veh[v].lc;
        lc.savedStateRight = lc.savedStateLeft = 0;
        lc.strategicBlocked = false;
    }
    std::vector<char> lcStratBlockedSnap;  // (copy for reads across edges in the lane-change phase)
    bool strategicBlockedOf(int v) const {
        return snapRead(v) ? lcStratBlockedSnap[v] != 0 : veh[v].lc.strategicBlocked;
    }
    // sublane lane changing (sublane.cpp)
    void laneChangeSublaneEdge(int e, std::vector<LCMessage>& m, std::vector<LaneChangeEvent>& ev, std::vector<DetEvent>& d,
                               std::vector<int>& changed, std::vector<std::vector<int>>& newLanes);
    void slPrepareStep(int v);
    double slPatchSpeed(int v, double min, double wanted, double max);
    void slChanged(int v);
    int computeSublaneShift(int prevEdge, int curEdge) const;
    bool isSL(int v) const { return vt(v).lcModel == 2 || (vt(v).lcModel == 0 && latRes > 0); }  // MSLCM_SL2015
    std::vector<LatPos> latSnap;  // their copy for reads across edges in the lane-change phase (as kinSnap)
    std::vector<double> lcManeuverSnap, lcSpeedLatSnap;  // (the lateral maneuver of the vehicles, ditto)
    const LatPos& LV(int v) const { return snapRead(v) ? latSnap[v] : lat[v]; }
    double maneuverDistOf(int v) const {
        return snapRead(v) ? lcManeuverSnap[v] : veh[v].lc.maneuverDist;
    }
    double speedLatOf(int v) const { return snapRead(v) ? lcSpeedLatSnap[v] : veh[v].lc.speedLat; }
    bool sublane = false;        // MSGlobals::gSublane: continuous lane changes or the sublane model
    double latRes = 0;           // MSGlobals::gLateralResolution (--lateral-resolution)
    bool cfHaveVars = false;
    CFVars* cfv(int v) const { return cfHaveVars ? const_cast<CFVars*>(&cfVars[v]) : nullptr; }
    void initCarFollowVars(int v);  // MSCFModel::createVehicleVariables (at loading)
    bool cfNeedsVars(const VType& t) const;
    double cfRand(int v, uint64_t salt) const { return rand01(opt.seed, (uint64_t)v, (uint64_t)step, RP_CF ^ mix64(salt)); }
    double cfFinalizeSpeed(int v, double vPos, std::vector<StopEvent>& stopEv);
    void cfUpdateStartup(int v, double vSafe);  // MSVehicle::executeMove: myTimeSinceStartup
    bool passingMinor(int v) const;
    double cfSlope(int v) const;  // MSVehicle::getSlope (degrees)
    double lcComputeSpeedLat(int v, double maneuverDist, bool urgent) const;
    double estimateLCDuration(int v, double speed, double remainingManeuverDist, double decel, bool urgent) const;
    Time lcRemainingTime(int v) const;
    void endLaneChangeManeuver(int v);
    // lateral state (lateral.cpp)
    double lateralOverlap(int v, double posLat, int lane) const;
    int parallelLane(int lane, int offset) const;
    int shadowLaneFor(int v, int lane, double posLat) const;
    void updateShadowLane(int v);
    double centerOnEdge(int v, int lane) const;
    double latOffset(int v, int lane) const;               // MSVehicle::getLatOffset
    double rightSideOnEdge(int v, int lane = -1) const { return centerOnEdge(v, lane) - 0.5 * vt(v).width; }
    double leftSideOnEdge(int v, int lane = -1) const { return centerOnEdge(v, lane) + 0.5 * vt(v).width; }
    double leftSideOnLane(int v, int lane) const { return KV(v).posLat + 0.5 * net.lanes[lane].width + 0.5 * vt(v).width; }
    double lateralDistanceToLane(int v, int offset) const;  // MSVehicle::lateralDistanceToLane
    bool outsideEdge(int v) const;                          // MSLCM_SL2015::outsideEdge
    int shadowDirection(int v) const;
    bool brakeForOverlap(int v, int link, int lane) const;
    double railCurveResistance(int v) const;
    double cfInteractionGap(int v, double vL) const;
    double wiedemannV(int v, double predSpeed, double gap, double predAccel) const;
    double kernerV(int v, double speed, double vfree, double gap, double predSpeed) const;
    double smartSKVsafe(const VType& t, const CFVars* vars, double gap, double predSpeed) const;
    double naschVsafe(int v, double speed, double gap) const;
    double naschRoundToCell(const VType& t, double speed) const;
    double accV(int v, int block, double gap, double speed, double predSpeed, double desSpeed) const;
    double caccV(int v, int pred, double gap, double speed, double predSpeed, double desSpeed) const;
    double cfMaximumLaneSpeedCF(int v, double maxSpeed, double maxSpeedLane) const;
    double eidmV(int v, double gap2pred, double egoSpeed, double predSpeed, double desSpeed, bool respectMinGap, int update,
                 cf::CalcReason usage) const;
    double eidmFreeSpeed(int v, double speed, double seen, double maxSpeed, bool onInsertion, cf::CalcReason usage) const;
    double eidmFinalizeSpeed(int v, double vPos, std::vector<StopEvent>& stopEv);
    void eidmInternalSpeedLimit(int v, double oldV);
    std::pair<int, double> vehicleLeader(int v, double dist) const;  // MSVehicle::getLeader (without crossing foes)
    void setCarFollowModel(VType& t, const std::vector<std::string>& tok);
    void initCarFollowModel(VType& t);
    double cfParam(const VType& t, const char* key, double deflt) const;
    double cfFollowSpeed(int v, double speed, double gap, double predSpeed, double predMaxDecel, int pred = -1,
                         cf::CalcReason usage = cf::CalcReason::CURRENT, bool computeLC = false, int lane = -1) const;
    double cfFollowSpeedT(const VType& t, int v, double speed, double gap, double predSpeed, double predMaxDecel, int pred,
                          cf::CalcReason usage, bool computeLC, int lane) const;
    double cfFollowSpeedDecel(int v, double decel, double speed, double gap, double predSpeed, double predMaxDecel, int pred) const;
    double cfStopSpeed(int v, double speed, double gap, double decel = -1, cf::CalcReason usage = cf::CalcReason::CURRENT,
                       int lane = -1) const;
    double cfFreeSpeed(int v, double speed, double seen, double maxSpeed, bool onInsertion = false, int lane = -1) const;
    double cfInsertionFollowSpeed(int v, double speed, double gap, double predSpeed, double predMaxDecel, int pred) const;
    double cfInsertionStopSpeed(int v, double speed, double gap, int lane = -1) const;
    double cfMaxNextSpeed(int v, double speed) const;
    double cfMaxNextSafeMin(int v, double speed) const;  // MSCFModel::maxNextSafeMin
    double cfMinNextSpeed(int v, double speed) const;
    double cfMinNextSpeedEmergency(int v, double speed) const;
    double cfBrakeGap(int v, double speed) const;  // brakeGap(speed, decel, tau)
    double cfBrakeGap(int v, double speed, double decel, double headway) const;
    double cfSecureGap(int v, int pred, double speed, double leaderSpeed, double leaderMaxDecel, bool computeLC = false) const;
    double cfSpeedAfterMaxDecel(int v, double speed) const;

    // ---- helpers -----------------------------------------------------------
    const VType& vt(int v) const { return types[kin[v].type]; }
    int insertionChecksOf(int v) const {
        return veh[v].extra.p != nullptr && veh[v].extra.p->insertionChecks >= 0 ? veh[v].extra.p->insertionChecks : opt.insertionChecks;
    }
    bool isInternal(int lane) const { return net.lanes[lane].internal; }
    // MSLane::getVehicleMaxSpeed
    double laneVehMaxSpeed(int lane, int v) const {
        return std::min(vehMaxSpeed(v), net.lanes[lane].speed * veh[v].speedFactor);
    }
    double laneVehMaxSpeedCapped(int lane, int v, double maxSpeed) const {
        return std::min(maxSpeed, net.lanes[lane].speed * veh[v].speedFactor);
    }
    double vehMaxSpeed(int v) const {
        const VType& t = vt(v);
        return std::min(t.maxSpeed, t.desiredMaxSpeed * veh[v].speedFactor);
    }
    int routeEdge(int v, int offset) const {  // succEdge
        const std::vector<int>& r = routes[veh[v].route];
        const int i = veh[v].routeIdx + offset;
        return (i < 0 || i >= (int)r.size()) ? -1 : r[i];
    }
    int curEdge(int v) const { return routes[veh[v].route][veh[v].routeIdx]; }
    bool onLastEdge(int v) const { return veh[v].routeIdx == (int)routes[veh[v].route].size() - 1; }
    int backLane(int v) const {
        const Kin& k = kin[v];
        return k.nFurther > 0 ? k.further[k.nFurther - 1] : k.lane;
    }
    // vehicle state used by neighbour queries: the live state, or during lane changing the copy
    // taken at the start of the phase until the stage of the vehicle's edge is over (lcSnapStage:
    // the stage of the vehicle's edge, -1 for edges without lane changing; lcCurStage: the first
    // stage not yet published)
    const Kin* kview = nullptr;
    // in the lane-change phase: vehicles of edges of the same or later stages are read from the copy, except those
    // of the edge being changed by this thread (LaneChanger::live)
    bool snapRead(int v) const {
        return kview != nullptr && lcSnapStage[v] >= lcCurStage && net.lanes[kview[v].lane].edge != lcOwnEdge;
    }
    const Kin& KV(int v) const { return snapRead(v) ? kview[v] : kin[v]; }
    double backPosOf(const Kin& k, int lane) const;     // MSVehicle::getBackPositionOnLane
    double backPositionOnLane(int v, int lane) const { return backPosOf(KV(v), lane); }
    double positionOnLane(int v, int lane) const { return backPositionOnLane(v, lane) + vt(v).length; }
    bool isFrontOnLane(int v, int lane) const { return KV(v).lane == lane || KV(v).shadowLane == lane; }
    bool isStopped(int v) const { return !veh[v].stops.empty() && veh[v].stops.front().reached; }
    bool willStop(int v) const {
        return !isStopped(v) && !veh[v].stops.empty() && net.lanes[veh[v].stops.front().lane].edge == net.lanes[kin[v].lane].edge;
    }
    double impatience(int v) const {
        return opt.timeToImpatience > 0 ? std::max(0., std::min(1., STEPS2TIME(veh[v].waitingTime) / opt.timeToImpatience)) : 0.;
    }
    Time waitingTimeFor(int v, int link) const { return link == veh[v].haveStoppedFor ? TIME_MAX : veh[v].waitingTime; }
    // SUMOVehicleParserHelper::processActionStepLength: a positive multiple of the step length
    static Time processActionStepLength(double given) {
        Time result = TIME2STEPS(given);
        if (result <= 0) {
            return DELTA_T;
        }
        if (result % DELTA_T != 0) {
            result = std::max(DELTA_T, (Time)((double)DELTA_T * std::floor(double(result) / double(DELTA_T))));
        }
        return result;
    }
    Time typeActionStepLength(const VType& t) const {
        return processActionStepLength(t.actionStepLength > 0 ? t.actionStepLength : opt.defaultActionStepLength);
    }
    Time actionStepLength(int v) const { return typeActionStepLength(vt(v)); }
    double actionStepLengthSecs(int v) const { return STEPS2TIME(actionStepLength(v)); }
    bool isActionStep(int v, Time t) const { return (t - veh[v].lastActionTime) % actionStepLength(v) == 0; }
    bool congested(int v) const {
        return kin[v].speed < (60.0 / 3.6) || net.lanes[kin[v].lane].speed < (60.1 / 3.6);
    }

    // best lanes (bestlanes.cpp)
    // occupancy = false: the occupancies of the continuations (LaneQ::nextOccupation) are not updated; only
    // lane changing reads them, and it updates them first
    void updateBestLanes(int v, bool forceRebuild, int startLane = -1, bool occupancy = true);
    void updateOccupancyAndCurrentBestLane(int v, int startLane, bool occupancy = true);
    const std::vector<LaneQ>& bestLanesOf(int v) const { return veh[v].bestLanes.front(); }
    const LaneQ& curLaneQ(int v) const { return veh[v].bestLanes.front()[veh[v].currentLaneInBestLanes]; }
    const std::vector<int>& bestContinuation(int v) const;
    const std::vector<int>& bestContinuationOf(int v, int lane) const;
    int bestLaneOffset(int v) const;
    double bestLaneDist(int v) const;
    int nextLinkPriority(const std::vector<int>& conts) const;

    // MSCriticalFollowerDistanceInfo with a single sublane
    struct CritFollower {
        int veh = -1;
        double dist = DMAX;
        double missingGap = -DMAX;
        void add(int v, double gap, double requiredGap) {
            const double mg = requiredGap - gap;
            if ((mg > missingGap || (mg > 0 && gap < dist) || (gap < 0 && dist > 0))
                    && !(gap > 0 && dist < 0) && !(missingGap > 0 && dist < gap)) {
                veh = v;
                dist = gap;
                missingGap = mg;
            }
        }
    };
    // MSLane::getFollowersOnConsecutive (FOLLOW_ALWAYS) outside of lane changing
    CritFollower followersOnConsecutive(int lane, int ego, double backOffset, double egoSpeed, double egoDecel) const;
    // the vehicles on the bidirectional lane of `lane` (front or further lanes there): SUMO registers them as partial
    // occupants of `lane` (setPartialOccupation on the bidi lane); f(vehicle, MSVehicle::getPositionOnLane(lane))
    template <class F>
    void forBidiOccupants(int lane, F f) const {
        const int b = net.lanes[lane].bidi;
        if (b < 0) {
            return;
        }
        const double len = net.lanes[lane].length;
        // (railways do not register on purely railway bidi lanes: MSVehicle::enterLaneAtMove / updateFurtherLanes)
        const bool railOnly = (net.lanes[b].permissions & ~VC_RAIL_CLASSES) == 0;
        for (int u : lanes[b].vehs) {
            if (!(railOnly && (veh[u].vclass & VC_RAIL_CLASSES) != 0)) {
                f(u, len - kin[u].pos);
            }
        }
        for (int u : lanes[b].partials) {
            if (railOnly && (veh[u].vclass & VC_RAIL_CLASSES) != 0) {
                continue;
            }
            const Kin& k = kin[u];
            const double vlen = vt(u).length;
            double leftLength = vlen - k.pos;
            for (int i = 0; i < k.nFurther && leftLength > 0; ++i) {
                leftLength -= net.lanes[k.further[i]].length;
                if (k.further[i] == b) {
                    f(u, len + leftLength - vlen);  // (getBackPositionOnLane(lane, true) + length)
                    break;
                }
            }
        }
    }
    double missingRearGap(int lane, int leader, double backOffset, double leaderSpeed) const;

    // lane queries (lanes.cpp)
    int succLinkSec(int v, int nRouteSuccs, int fromLane, const std::vector<int>& conts) const;  // -1 = link end
    int lastAnyVehicle(int lane) const;
    int firstAnyVehicle(int lane) const;
    int lastVehicleFrom(int lane, double minPos, int ego) const;  // getLastVehicleInformation(ego, 0, minPos)
    double spaceTillLastStanding(int lane, int ego, bool& foundStopped) const;
    bool appropriate(int lane, int v) const;
    template <class F> void anyVehicles(int lane, F f) const;  // rear to front, stops when f returns false

    // links (link.cpp)
    Time leaveTime(int link, Time arrivalTime, double arrivalSpeed, double leaveSpeed, double vehLength) const;
    // ego -1: no vehicle (insertion checks, pedestrians); dist: distance of ego to the link (-1 = unknown)
    // egoPerson: the person asking (ego -1)
    // posLat: the lateral position of ego to check (sublane model; default: its current one)
    bool opened(int link, Time arrivalTime, double arrivalSpeed, double leaveSpeed, double vehLength, double impatience,
                double decel, Time waitingTime, int ego, bool ignoreRed = false, std::vector<int>* collectFoes = nullptr,
                double dist = -1, int egoPerson = -1, const double* posLat = nullptr) const;
    bool blockedAtTime(int link, Time arrivalTime, Time leaveTime, double arrivalSpeed, double leaveSpeed, bool sameTargetLane,
                       double impatience, double decel, Time waitingTime, int ego, bool lastWasContRed,
                       std::vector<int>* collectFoes = nullptr, double dist = -1, int egoPerson = -1) const;
    bool blockedAtDist(int lane, double vehCenter, double vehWidth, double oncomingGap, int ego = -1) const;
    bool cfIgnoresFoe(int v, int foe) const;
    bool jmIgnoresFoe(int ego, int foeVeh, int foePerson = -1) const;
    std::pair<int, double> nextBlocking(int lane, double minPos, double minRight, double maxLeft, double stopTime) const;
    bool hasPedestrians(int lane) const;
    double rightSideOnLane(int v, int lane) const { return KV(v).posLat + 0.5 * net.lanes[lane].width - 0.5 * vt(v).width; }
    double zipperSpeed(int ego, int link, double dist, double vSafe, Time arrivalTime, const std::vector<int>& foes) const;
    bool blockedByFoe(int link, const Approach& a, Time arrivalTime, Time leaveTime, double arrivalSpeed, double leaveSpeed,
                      bool sameTargetLane, double impatience, double decel, Time waitingTime, const VType* egoType) const;
    bool hasApproachingFoe(int link, Time arrivalTime, Time leaveTime, double speed, double decel) const;
    const Approach* approachOf(int link, int v) const;
    void moveApproach(int v, int from, int to);
    void removeApproaches(int v);
    std::pair<int, int> firstApproachingFoe(int link, int wrapAround) const;  // (vehicle, link)
    enum : int { LL_IN_THE_WAY = 1, LL_FROM_LEFT = 2, LL_SAME_SOURCE = 4, LL_SAME_TARGET = 8 };
    struct LinkLeader {
        int veh;  // -1: a pedestrian
        double gap;
        double distToCrossing;
        int flags;
        double latOffset = 0;  // the leader's lateral offset on the foe lane (sublane model)
    };
    void leaderInfo(int link, int ego, double dist, std::vector<LinkLeader>& result, bool computeLC, bool isShadowLink = false) const;
    bool isLeader(int ego, int link, int other, double gap) const;

    // movement (move.cpp)
    // ahead: the leaders per sublane with a lateral resolution (MSLane::planMovements), otherwise leader
    void planMove(int v, int leader, double lengthsInFront, const LeaderInfo* ahead = nullptr);
    void planMoveInternal(int v, int leader, const LeaderInfo* ahead, std::vector<DriveItem>& lfLinks, double& newStopDist,
                          double& newStopSpeed);
    void adaptToLeaders(int v, const LeaderInfo& ahead, double latOffset, double seen, DriveItem* lastLink, int lane, double& vv,
                        double& vLinkPass) const;
    // MSLane::getLastVehicleInformation (ego, ignore: -1 for none)
    LeaderInfo lastVehicleInformation(int lane, int ego, double latOffset, double minPos, int ignore = -1) const;
    // sublane lane queries with a vehicle iteration (sublanequery.hpp)
    template <class AnyVeh>
    LeaderInfo lastVehicleInformationT(int lane, int ego, double latOff, double minPos, int ignore, AnyVeh anyVeh) const;
    template <class AnyVeh>
    LeaderInfo firstVehicleInformationT(int lane, int ego, double latOff, bool onlyFrontOnLane, double maxPos, AnyVeh anyVeh) const;
    template <class AnyVeh>
    CriticalFollowerDistanceInfo followersOnConsecutiveT(int lane, int ego, double backOffset, bool allSublanes, bool egoOnRoad,
                                                         AnyVeh anyVeh) const;
    void adaptToLeader(int v, int pred, double gap, double seen, DriveItem* lastLink, double& vv, double& vLinkPass) const;
    void adaptToJunctionLeader(int v, int leader, double gap, double seen, DriveItem* lastLink, int lane, double& vv,
                               double& vLinkPass, double distToCrossing) const;
    void checkLinkLeader(int v, int link, int lane, double seen, DriveItem* lastLink, double& vv, double& vLinkPass,
                         double& vLinkWait, bool& setRequest) const;
    void checkLinkLeader1(int v, int link, int lane, double seen, DriveItem* lastLink, double& vv, double& vLinkPass,
                          double& vLinkWait, bool& setRequest, bool isShadowLink) const;
    void checkRewindLinkLanes(int v, double lengthsInFront, std::vector<DriveItem>& lfLinks) const;
    double cooperativeHelpSpeed(int v, int lane, double distToLaneEnd) const;
    Time arrivalTimeAt(int v, double seen, double vv, double arrivalSpeed) const;
    void processLinkApproaches(int v, double& vSafe, double& vSafeMin, double& vSafeMinDist);
    void executeMove(int v, std::vector<DetEvent>& det, std::vector<StopEvent>& stopEv, std::vector<EdSample>* eds,
                     std::vector<DriveWayEvent>* dwEv);
    void setNoReminderFurther(int v);
    bool holdsReminders(int v, int lane) const;
    void meanDataMove(int v, int lane, double oldPos, double newPos, double newSpeed, std::vector<EdSample>& eds) const;
    double finalizeSpeed(int v, double vPos, std::vector<StopEvent>& stopEv);
    bool hasArrivedInternal(int v) const;
    void updateFurtherLanes(int v, const std::vector<int>& passedLanes, const double* widthShiftPosLat = nullptr);
    void updateDriveItems(int v);
    double processNextStop(int v, double currentVelocity, std::vector<StopEvent>& ev);
    double stopEndPos(int v, const Stop& s) const;   // MSStop::getEndPos
    double busStopLastFreePos(int bs, int v) const;
    bool busStopFits(int bs, double pos, int v) const;
    void busStopEnter(int bs, int v, double pos, bool parking, double length = -1);
    void busStopLeave(int bs, int v);
    void busStopComputeLastFreePos(int bs);
    bool keepStopping(int v, bool afterProcessing) const;
    void resumeFromStopping(int v, std::vector<StopEvent>& ev);
    // MSBaseVehicle::isParking: off the road (not at an on-road parking area)
    bool isParking(int v) const {
        if (!isStopped(v) || !veh[v].stops.front().parking) {
            return false;
        }
        const Stop& s = veh[v].stops.front();
        return (s.parkingArea < 0 || !parkingAreas[s.parkingArea].onRoad) && (s.speed == 0 || kin[v].speed < HALTING_SPEED);
    }
    double lcPatchSpeed(int v, double min, double wanted, double max);
    void lcPrepareStep(int v);

    // detectors and traffic lights (tls.cpp)
    void initTLS();
    void tlsStep();
    Time tlsTrySwitch(int t);
    void initRailCrossing(int t);
    Time railCrossingUpdate(int t);
    void setTLSignals(int t);
    void loopNotifyMove(int v, int lane, double oldPos, double newPos, double newSpeed, std::vector<DetEvent>& ev) const;
    const std::vector<int>& loopsOf(int v, int lane) const;
    void loopNotifyEnter(int v, int lane, std::vector<DetEvent>& ev) const;
    void loopNotifyEnterNow(int v, int lane);
    void loopNotifyLeave(int v, int lane, std::vector<DetEvent>& ev) const;
    void applyDetEvents(std::vector<DetEvent>& ev);
    double timeSinceLastDetection(int loop) const;

    // insertion (insert.cpp)
    void insertVehicles();
    bool tryInsert(int v);
    bool isInsertionSuccess(int v, int lane, double speed, double pos, bool patchSpeed, bool resuming);
    double safeInsertionSpeedSL(int v, int lane, double seen, const LeaderInfo& leaders, double speed) const;
    double insertionPosLat = 0;  // the lateral position of the vehicle being inserted (insertion is serial)
    double departPosLatOf(int v, int lane, int attempt) const;  // MSLane::getDepartPosLat
    bool checkFailure(int v, double& speed, double& dist, double nspeed, bool patchSpeed, int check, bool error = false);
    std::vector<int> descheduled;  // MSInsertionControl::descheduleDeparture (removed after the insertion attempts)
    bool checkForPedestrians(int v, int lane, double& speed, double& dist, double pos, bool patchSpeed);
    double safeInsertionSpeed(int v, int lane, double seen, int leader, double speed) const;
    void incorporate(int v, int lane, double pos, double speed);
    int freeLane(int edge, const std::vector<int>* allowed, SVCPermissions vclass, double departPos) const;
    double departPosBound(int v, int edge, bool upper) const;
    bool insertCheckOnly(int v) const;
    int probableLane(int edge, const std::vector<int>* allowed, SVCPermissions vclass, double departPos, double maxSpeed) const;
    double laneMeanSpeed(int lane) const;
    bool freeInsertion(int v, int lane, double speed, bool departure = false);

    // teleports and parking (transfer.cpp)
    void transferAdd(int v, bool parking);
    void checkTransfers();

    // vehicle routing (routing.cpp)
    struct RouteWS {  // per-thread Dijkstra state and the query settings
        bool staticEffort = false;  // MSNet's router (minimum travel times) instead of the routing device's
        const std::vector<RouterProhibition>* prohibited = nullptr;  // closed edges (SUMOAbstractRouter::prohibit)
        double queryTime = 0;  // the time of the query in seconds (prohibitions with an end)
        std::vector<double> effort;
        std::vector<int> prev;
        std::vector<char> visited;
        std::vector<int> touched;
        std::vector<int> heap;
        // the same for the railway router graph
        std::vector<double> rrEffort;
        std::vector<int> rrPrev;
        std::vector<char> rrVisited;
        std::vector<int> rrTouched;
        std::vector<int> rrHeap;
    };
    // RailEdge (utils/router/RailEdge.h): a network edge or a train reversal onto the bidi edge
    struct RailRouteEdge {
        int original = -1;              // network edge, -1 for a reversal
        int turnaround = -1;            // the reversal edge after this one
        bool isVirtual = true;          // reversal without a turn-around connection (the train backs up)
        std::vector<int> replacement;   // the edges a virtual reversal backs up on
        double maxLength = DMAX;        // longest train for the reversal
        double startLength = 0;
        std::vector<std::pair<int, int>> via;  // (successor, via edge), rail edge indices
    };
    std::vector<RailRouteEdge> rrEdges;  // RailwayRouter graph (empty when the network has no bidi edges)
    void buildRailRouter();
    double rrEffort(int re, int v, const RouteWS& ws) const;
    void rrInsertOriginalEdges(int re, double length, std::vector<int>& into) const;
    void rrAddVirtualTurns(int forward, int backward, double dist, double maxTrainLength, const std::vector<int>& replacement);
    bool rrDijkstra(int from, int to, int v, std::vector<int>& into, RouteWS& ws) const;
    bool railRouteCompute(int from, int to, int v, std::vector<int>& into, RouteWS& ws, bool avoidUnsafeBackTracking) const;
    // the routing device's router for trains when the network has bidi edges (MSNet's static router is a
    // plain Dijkstra)
    bool useRailRouter(int v, const RouteWS& ws) const { return !rrEdges.empty() && !ws.staticEffort && isRailway(veh[v].vclass); }
    bool rtActive = false;       // any vehicle with a routing device
    bool rtAdapt = false;        // edge weights are adapted (a routing device or an aggregated routing mode)
    Time rtEngineFrom = TIME_MAX;  // MSRoutingEngine::isEnabled from this step on (the first device is built)
    bool rtEdgeUpdates = false;  // MSRoutingEngine::hasEdgeUpdates
    std::vector<std::vector<std::pair<int, int>>> rtViaSucc;  // (node count: edges and taz connectors)
    std::vector<Taz> tazs;
    std::unordered_map<std::string, int> tazIdx;
    int tazNode(int taz, bool source) const { return (int)net.edges.size() + 2 * taz + (source ? 1 : 0); }  // MSEdge::myViaSuccessors (successor, via edge)
    std::vector<double> rtPenalty;      // MSEdge::myTimePenalty
    std::vector<SVCPermissions> rtPerm; // MSEdge::myCombinedPermissions
    std::vector<SVCPermissions> rtPermOrig;  // MSEdge::myOriginalCombinedPermissions
    bool rtHaveExtras = false;               // MSRoutingEngine::myHaveExtras
    std::vector<std::vector<std::string>> preferenceLines;  // PREFERENCE routingType vClasses vTypes priority
    std::vector<double> rtPref;              // per vehicle type and routing type: MSNet::getPreference
    double rtPriorityFactor = 0, rtMinPriority = 0, rtPriorityRange = 0;
    double routingExtras(int e, int v, double effort) const;
    double routingExtrasFor(int e, uint32_t randomSeed, int type, double effort) const;
    std::vector<double> rtSpeed;        // MSRoutingEngine::myEdgeSpeeds (empty until first used)
    std::vector<double> rtPast;         // myPastEdgeSpeeds (rtSteps per edge)
    std::vector<double> rtBikeSpeed, rtBikePast;  // myEdgeBikeSpeeds, myPastEdgeBikeSpeeds (--device.rerouting.bike-speeds)
    double edgeMeanSpeedBike(int e) const;
    // MSRoutingEngine::myEdgeTravelTimes (--weights.separate-turns): travel time sum and count per internal edge
    std::unique_ptr<std::atomic<long long>[]> rtTurnTT, rtTurnCount;
    double patchSpeedForTurns(int e, double currSpeed);
    int rtSteps = 0, rtPastIdx = 0;
    Time rtLastAdaptation = -1, rtNextAdaptation = TIME_MAX;
    std::vector<char> edgeDelayed;      // MSEdge::myAmDelayed (a vehicle entered the edge)
    std::vector<std::pair<Time, int>> preRerouteQueue, rerouteQueue;  // min-heaps (step, vehicle)
    std::vector<int> rtDue;
    void initRouting();
    void buildRoutingGraph();
    void pushPreReroute(int v, Time t);
    // the route driveways are built and matched along (MSDriveWay: the vehicle's whole route)
    const std::vector<int>& dwRoute(int v) const {
        const int fr = veh[v].exr().fullRoute;
        return routes[fr >= 0 ? fr : veh[v].route];
    }
    double edgeMeanSpeed(int e) const;
    void routingInitWeights();
    void adaptEdgeWeights();
    // t: the time the edge is entered (default: the query time; loaded weights depend on it)
    double routingEffort(int e, int v, const RouteWS& ws, double t = -1) const;
    bool routeProhibited(int e, int v, const RouteWS& ws) const;
    double prohibitionDelay(int e, int v, const RouteWS& ws, double t) const;
    double routeViaEffort(int via, int v, const RouteWS& ws, double t = -1) const;
    // loaded edge weights (--weight-files): per edge (begin, end, travel time), the last loaded wins
    struct LoadedWeight {
        double begin, end, value;
    };
    std::vector<std::vector<LoadedWeight>> loadedWeights;
    bool loadedTravelTime(int e, double t, double& value) const;  // MSEdgeWeightsStorage::retrieveExistingTravelTime
    bool routeCompute(int from, int to, int v, std::vector<int>& into, RouteWS& ws) const;
    bool routeComputePos(int from, double fromPos, int to, double toPos, int v, std::vector<int>& into, RouteWS& ws) const;
    double routeCosts(const std::vector<int>& edges, int v, const RouteWS& ws) const;
    // SUMOAbstractRouter::recomputeCostsPos at time t (seconds)
    double routeCostsPos(const std::vector<int>& edges, int v, RouteWS& ws, double fromPos, double toPos, double t) const;
    // MSBaseVehicle::StopEdgeInfo: the targets of a reroute
    struct StopEdgeInfo {
        int edge;
        double pos;
        double priority = -1;
        Time arrival = -1;
        bool sink = false;
        int stop = -1;          // the vehicle's stop (stopPar, the stopping place name)
        int routeIndex = -1;    // set during routing, used by the optimization
        bool skipped = false, backtracked = false;
        Time delay = 0;
        int origEdge = -1;      // set when replaced by an alternative
    };
    // MSStopOptimizer::optimizeSkipped
    std::vector<int> stopOptimizeSkipped(int v, int source, double sourcePos, std::vector<StopEdgeInfo>& stops,
                                         const std::vector<int>& edges, Time maxDelay, RouteWS& ws) const;
    const std::vector<int>& stopAlternatives(const Veh& x, int stop) const;  // MSNet::getStoppingPlaceAlternatives
    int rerouteOriginIdx(int v) const;
    bool reroute(int v, bool onInit, RouteWS& ws, bool device, int sinkEdge = -1, bool silent = false);
    bool replaceRouteEdges(int v, std::vector<int>& edges, bool onInit, int originIdx, bool patchStops = false);
    void deviceReroute(int v, bool onInit, RouteWS& ws);
    void preInsertionReroutes();
    void periodicReroutes();
    void routingDeparted(int v, RouteWS& ws);
    void routingStopEnded(int v);
    // rerouters (rerouter.cpp)
    static constexpr int RR_KEEP_DESTINATION = -2, RR_TERMINATE_ROUTE = -3;
    struct RerouteInterval {
        Time begin = -1, end = TIME_MAX;
        std::vector<std::pair<int, SVCPermissions>> closed;  // edge, classes still allowed (0 if not given)
        std::vector<RouterProhibition> prohibitions;           // the closed edges for routing (closingReroute until)
        std::vector<bool> closedPermSet;                     // permissions given (they also apply to driving)
        std::vector<std::pair<int, SVCPermissions>> closedLanes;
        std::vector<int> closedLanesAffected;                // edges of the closed lanes
        std::vector<std::pair<int, double>> edgeProbs;       // destProbReroute / viaProbReroute
        double edgeProbSum = 0;
        bool isVia = false;
        std::vector<std::pair<int, double>> routeProbs;      // routeProbReroute
        double routeProbSum = 0;
        std::vector<std::pair<int, bool>> parkAreas;         // parkingAreaReroute: (area, visible)
        std::vector<double> parkProbs;
        double parkProbSum = 0;
        bool permissionsAllowAll = true;
        // overtakingReroute (MSTriggeredRerouter::OvertakeLocation)
        struct OvertakeLocation {
            std::vector<int> main, siding;
            int sidingExit = -1;      // rail signal at the end of the siding
            double sidingLength = 0;
            double minSaving = 300;
            bool defer = false;
        };
        std::vector<OvertakeLocation> overtakeLocations;
        std::vector<int> stopAlternatives;  // stationReroute: bus / train stops
    };
    struct Rerouter {
        std::string id;
        std::vector<int> edges;
        double prob = 1;
        bool off = false;
        Time timeThreshold = 0;
        std::vector<std::string> vTypes;
        std::vector<int> vTypeIdx;
        std::vector<RerouteInterval> intervals;
        bool haveParkProbs = false;  // isParkingRerouter
        double distanceWeight = 1;   // MSStoppingPlaceRerouter::myEvalParams["distanceto"] (changed by collectWeights)
        bool haveClosingUntil = false;  // myHaveClosingUntil
    };
    std::vector<Rerouter> rerouters;
    std::unordered_map<std::string, int> rerouterIdx;
    std::vector<std::vector<int>> laneRerouters;  // lane move reminders
    void parseRerouterLine(const std::vector<std::string>& tok);
    std::vector<std::vector<std::string>> speedSignLines;
    void initSpeedSigns();
    void initRerouters();
    const RerouteInterval* currentReroute(const Rerouter& r, int v) const;
    bool rerouterTrigger(int v, int ri, bool laneChange, bool parkingReroute = false);
    void rerouterApply(int v, int ri, int interval);
    void rerouterEnter(int v, int lane, bool laneChange);
    void rerouterMove(int v);
    bool hasRerouters() const { return !laneRerouters.empty(); }
    bool trackAccumulatedWaiting = false;  // a rerouter has a time threshold (getWaitingTime(true))
    void passWaitingTime(Veh& x, bool waiting) const;
    // rail rerouters (overtakingReroute, stationReroute)
    int edgeRailSignal(int e) const;
    int findRailSignal(const std::vector<int>& edges, int begin, int end) const;
    void initOvertakeLocations();
    double edgeMinTravelTime(int e, int v) const;
    double estimateTimeToNextStop(int v) const;
    std::pair<double, double> estimateTimeDistToNextStop(int v) const;
    double slowDownForSchedule(int v, double vMinComfortable) const;
    double stopDelay(int v) const;
    double stopArrivalDelay(int v) const;
    std::pair<int, int> overtakingTrain(int v, int mainStart, const RerouteInterval::OvertakeLocation& oloc, double& netSaving);
    void overtakingReroute(int v, const RerouteInterval& def);
    void stationReroute(int v, int ri, const RerouteInterval& def);
    bool stationEvaluate(int v, ParkingSearch& S, double brakeGap, int bs, double occupancy, double capacity, double prob, RouteWS& ws);
    int rerouteStation(int v, int ri, const RerouteInterval& def, const std::vector<int>& blocked, RouteWS& ws);
    std::vector<int> vehIdOrder;  // the vehicles sorted by id (MSVehicleControl's map)
    Time accumulatedWaitingTime(const Veh& x) const;
    // rerouter actions of the parallel move phase that touch shared state (parking areas) wait until the
    // phase is over and are applied in vehicle order
    struct DeferredReroute {
        int veh, rerouter, interval;
        bool preMove;  // triggered before the move (processNextStop): evaluated in that state
        Kin kin;
        int routeIdx;
        int taskOrder, seq;  // SUMO's order: the lane in executeMovements, then the trigger within the lane
        std::vector<std::pair<int, ParkingArea>> ownLaneAreas;  // the parking areas of that lane at the trigger
    };
    bool deferReroutes = false;
    std::mutex deferredMutex;
    std::vector<DeferredReroute> deferredReroutes;
    void applyDeferredReroutes();

    // parking areas (parking.cpp)
    std::vector<ParkingArea> parkingAreas;
    std::unordered_map<std::string, int> parkingAreaIdx;
    std::vector<std::vector<int>> laneParkingAreas;
    // deferred reroutes see the parking areas as SUMO's sequential executeMovements would: the final state
    // of areas on lanes executed before the vehicle's lane, the state before the phase (paStart, saved at
    // the first change) of areas on later lanes, the areas of its own lane as they were at the trigger
    std::vector<ParkingArea> paStart;
    std::vector<int> paStartStep;
    std::vector<int> laneExecOrder, laneExecStep;  // position of the lane in executeMovements (this step)
    const std::vector<const ParkingArea*>* paView = nullptr;
    const ParkingArea& paRead(int i) const { return paView != nullptr ? *(*paView)[i] : parkingAreas[i]; }
    void paBeforeChange(int area);
    void initParkingAreas();
    void parseParkingAreaLine(const std::vector<std::string>& tok);
    void paSetRoadsideCapacity(ParkingArea& pa);
    void paComputeLastFreePos(ParkingArea& pa);
    bool paMustAdvance(const ParkingArea& pa, SVCPermissions vc) const;
    double paLastFreePos(const ParkingArea& pa, int v, double brakePos, bool ownLane) const;
    double paLastFreePosWithReservation(int area, int v, double brakePos);
    int paLotIndex(const ParkingArea& pa, int v) const;
    void paEnter(int area, int v);
    void paLeave(int area, int v);
    void paRemoveReservation(ParkingArea& pa, int v);
    void paAddReservation(ParkingArea& pa, int v);
    int paOccupancyIncludingReservations(const ParkingArea& pa, int v) const;
    int paOccupancyIncludingRemoteReservations(const ParkingArea& pa, int v) const;
    int paLastStepOccupancyIncludingRemoteReservations(const ParkingArea& pa, int v) const;
    double paInsertionPosition(const ParkingArea& pa, int v) const;
    bool paAccepts(const ParkingArea& pa, int v) const;
    void paSnapshot();
    void paEndStep();
    bool hasParkingAreas() const { return !parkingAreas.empty(); }
    bool parkingSignalAtStop(int v) const;
    double parkingWeight(int v, const std::string& key, double def) const;
    void rememberBlockedParkingArea(int v, int area, bool local);
    Time sawBlockedParkingArea(int v, int area, bool local) const;
    void parkingReroute(int v, int ri, const RerouteInterval& def);
    int rerouteStoppingPlace(int v, int ri, const RerouteInterval& def, int dest, bool& newDestination,
                             std::vector<int>& newRoute, RouteWS& ws);
    bool parkingStops(int v) const;
    bool parkingEvaluate(int v, ParkingSearch& S, double brakeGap, bool newDestination, int alt, double occupancy, double prob,
                         RouteWS& ws);
    bool replaceParkingArea(int v, int area);
    double routeDistanceBetween(const std::vector<int>& edges, double fromPos, double toPos) const;
    double internalFollowingLength(int from, int to) const;

    // rail signals and driveways (rail.cpp)
    std::vector<std::unique_ptr<DriveWay>> driveWays;
    // per driveway: the number of trains on it and its signal link (contiguous for the foe scans of the signals)
    // and the number of its foe entries that are occupied (dwFoeOf: the driveways listing it as foe, per entry)
    struct DwHot {
        int nTrains = 0;
        int origin = -1;
        int occFoes = 0;
    };
    std::vector<DwHot> dwHot;
    std::vector<std::vector<int>> dwFoeOf;
    void dwTrainAdd(int dwi, int v) {
        if (driveWays[dwi]->trains.insert(v).second && dwHot[dwi].nTrains++ == 0) {
            for (int d : dwFoeOf[dwi]) {
                dwHot[d].occFoes++;
                dwMutated(d);
            }
        }
    }
    void dwTrainRemove(int dwi, int v) {
        if (driveWays[dwi]->trains.erase(v) > 0 && --dwHot[dwi].nTrains == 0) {
            for (int d : dwFoeOf[dwi]) {
                dwHot[d].occFoes--;
                dwMutated(d);
            }
        }
    }
    void dwAddFoe(DriveWay& d, int f) {
        d.foes.push_back(f);
        dwFoeOf[f].push_back(d.index);
        if (dwHot[f].nTrains > 0) {
            dwHot[d.index].occFoes++;
        }
        dwMutated(d.index);
    }
    void dwClearFoes(DriveWay& d) {
        for (int f : d.foes) {
            std::vector<int>& of = dwFoeOf[f];
            of.erase(std::find(of.rbegin(), of.rend(), d.index).base() - 1);  // (added recently: search from the end)
        }
        d.foes.clear();
        dwHot[d.index].occFoes = 0;
        dwMutated(d.index);
    }
    // the rail signals are evaluated in parallel (speculatively) and taken over in SUMO's order unless a driveway
    // built by an earlier signal of the step changed a driveway the evaluation read (foes, subs, sidings,
    // conflict links) or the signal's own driveway lists; then the signal is evaluated again
    struct RsSpec {
        bool abort = false;
        bool keep = false;
        std::string state;
        std::vector<int> reads;                                 // driveways
        std::vector<std::pair<int, int>> activeWrites;          // driveway, vehicle
        std::vector<std::pair<int, RailWaitRelation>> waits;    // by waiting vehicle
    };
    static thread_local RsSpec* rsSpec;
    std::vector<RsSpec> rsSpecResults;
    long long nRsUpdates = 0, nRsSpeculative = 0, nRsReevaluated = 0;  // statistics
    bool rsCommitting = false;
    std::vector<char> dwTouched, rsTouched;
    std::vector<int> dwTouchedList, rsTouchedList;
    void dwMutated(int d) {
        if (rsCommitting && !dwTouched[d]) {
            dwTouched[d] = 1;
            dwTouchedList.push_back(d);
        }
    }
    static void dwSpecRead(int d) {
        if (rsSpec != nullptr) {
            rsSpec->reads.push_back(d);
        }
    }
    bool rsSpeculate() const;
    void rsMarkTouched(int ri);
    // the last driveways found for a train at signal links (MSRailSignal::LinkInfo::getDriveWay; a train approaches
    // several signals at once)
    struct DwLookup {
        int link = -1, firstIt = -1, tailLen = -1, dw = -1;
        uint64_t tailHash = 0;
    };
    static constexpr int DW_LOOKUPS = 4;
    std::vector<std::array<DwLookup, DW_LOOKUPS>> dwLookup;
    std::vector<RailSignal> railSignals;
    std::vector<int> tlsRail;                        // rail signal of each traffic light (-1: none)
    std::vector<std::vector<int>> laneDriveWays;     // driveways whose forward section starts on the lane
    std::map<int, std::vector<int>> switchDriveWays;  // by link
    std::map<int, std::vector<int>> reversalDriveWays, departureDriveways, departureDrivewaysEnds, endingDriveways;  // by edge
    std::map<int, int> departDrivewayIndex;          // by junction
    std::set<int> activeRailSignals;
    std::map<int, RailWaitRelation> railWaitRelations;  // by waiting vehicle
    // rail signal constraints
    std::vector<std::vector<std::string>> railConstraintLines;  // RSC lines
    std::vector<RailConstraint> railConstraints;
    std::vector<PassedTracker> passedTrackers;
    std::vector<int> laneTracker;                           // tracker of each lane (-1: none; empty: no trackers)
    std::unordered_map<std::string, std::string> railTripIdLookup;  // MSRailSignalConstraint::myTripIdLookup
    std::set<std::set<int>> railWrittenDeadlocks;           // MSRailSignalControl::myWrittenDeadlocks
    std::vector<std::vector<RailDeadlockFound>> railResolvedDeadlocks;  // per move task
    std::vector<std::string> deadlockRows;  // deadlock-output
    void initRailConstraints();
    void railAddConstraint(int ri, const std::string& tripId, int type, int foeSignal, const std::string& foeTrip, int limit, bool active);
    const std::string& vehTripId(int v) const { return veh[v].tripId.empty() ? veh[v].id : veh[v].tripId; }
    bool rcCleared(int c) const;
    int rcFoe(int c) const;
    bool constraintsAllow(int ri, int v, bool storeWaitRelation);
    bool hasInsertionConstraint(int link, int v);
    void trackerEnter(int v, int lane);
    bool laneHasTracker(int lane) const { return lane < (int)laneTracker.size() && laneTracker[lane] >= 0; }
    std::vector<std::vector<DriveWayEvent>> dwEvents;     // per move task
    SVCPermissions railSignalized = 0;               // MSRailSignalControl::mySignalizedClasses
    bool railActive = false;                         // any rail vehicle
    int rlLinkTo(int lane, int target) const;
    int rlEntryLink(int link) const;
    int rlExitLink(int link) const;
    int rlNextNormal(int lane) const;
    bool rlIsRailSignalLink(int link) const;
    bool rlEdgeIsRailSignal(int e) const;
    bool rlJunctionFoes(int junction, int index, int other) const;
    bool rlIsEmpty(int lane) const;
    bool rlConnected(int edge, int next, SVCPermissions vc) const;
    SVCPermissions rtPermOf(int edge) const;
    bool dwIsSwitch(int link) const;
    void dwBuildRoute(DriveWay& dw, int origin, const std::vector<int>& route, int next, std::map<int, int>& visited,
                      FlankSet& flankSwitches);
    bool dwFlankConflict(const DriveWay& a, const DriveWay& other) const;
    bool dwCrossingConflict(const DriveWay& a, const DriveWay& other) const;
    bool dwOverlap(const DriveWay& a, const DriveWay& other) const;
    bool dwBidiBlockedBy(const DriveWay& a, const DriveWay& other) const;
    bool dwBidiBlockedByEnd(const DriveWay& a, const DriveWay& other) const;
    bool dwForwardRouteConflict(const DriveWay& a, const std::set<int>& forward, const DriveWay& other, bool secondCheck) const;
    void dwCheckFlanks(const DriveWay& dw, int originLink, const std::vector<int>& lanesList, const std::map<int, int>& visited,
                       bool allFoes, bool movingBlock, FlankSet& flankSwitches) const;
    void dwCheckCrossingFlanks(int dwLink, const std::map<int, int>& visited, FlankSet& flankSwitches) const;
    void dwAddFoes(DriveWay& dw, int link);
    void dwAddSwitchFoes(DriveWay& dw, int link);
    void dwFindFlankProtection(DriveWay& dw, int link, int origLink, std::vector<int>& flank);
    void dwAddBidiFoes(DriveWay& dw, int ownSignal, bool extended);
    void dwAddParallelFoes(DriveWay& dw, int link, int first);
    void dwAddReversalFoes(DriveWay& dw, bool movingBlock);
    bool dwForwardEndOnRoute(const DriveWay& a, const DriveWay& foe) const;
    bool dwHasRS(int cur, int next) const;
    int dwFoeOrSubFoe(const DriveWay& a, int dw) const;
    void dwAddSidings(DriveWay& dw, DriveWay& foe, bool addToFoe);
    bool dwBuildSubFoe(DriveWay& dw, DriveWay& foe, bool movingBlock);
    int newDriveWay(int origin, const std::string& id);
    int buildDriveWay(const std::string& id, int link, const std::vector<int>& route, int first);
    void dwAddConflictLink(DriveWay& dw, int link);
    bool dwMatch(const DriveWay& dw, const std::vector<int>& route, int firstIt) const;
    std::pair<bool, int> dwCanUseSiding(const DriveWay& dw, int ego, const DriveWay& foe, int recurseSidingEnd) const;
    bool dwFoeOccupied(const DriveWay& dw, int ego, std::vector<int>* occupied);
    const Approach& closestApproach(int link) const;
    bool dwFoeApproached(const DriveWay& dw) const;
    bool dwMustYield(const Approach& ego, const Approach& foe) const;
    bool dwHasLinkConflict(const DriveWay& dw, const Approach& ego, int foeLink);
    bool dwReserve(DriveWay& dw, const Approach& closest, std::vector<int>& occupied);
    void initRailSignals();
    bool isMovingBlockClass(SVCPermissions svc) const;
    void rsSetSignals(int ri);
    int rsGetDriveWay(int ri, int tlIndex, int v, int searchStart);
    int rsGetDriveWayByRoute(int ri, int tlIndex, const std::vector<int>& route, int firstIt, const std::string& info);
    std::string rsNewDriveWayID(int ri);
    bool rsUpdateCurrentPhase(int ri);
    void rsReroute(int ri, int linkIndex, int v, const std::vector<int>& occupied);
    void updateRailSignals();
    void railNotifyApproach(int link);
    void dwEnter(int dwi, int v, int reason);
    void dwEvent(int dwi, int v, bool entry, int reason) {
        if (opt.railsignalVehicleOutput) {
            driveWays[dwi]->events.push_back({v, now, entry, (uint8_t)reason});
        }
    }
    int departureDriveWay(int v, bool init);
    void initDriveWays(int v, bool update);
    int dwMatchesPastRoute(int dwi, int v) const;
    bool dwNotifyReroute(int dwi, int v);
    bool isRailVeh(int v) const;
    bool usesDriveWays(int v) const;
    bool isSignalizedVeh(int v) const;
    void dwLaneEntered(int v, int lane, int routeIdx, int reason, const std::vector<int>* route = nullptr);
    void dwLaneLeftBack(int v, int lane, int back);
    void dwLeaveAll(int v, int reason);
    void applyDriveWayEvents(std::vector<std::vector<DriveWayEvent>>& events);
    bool railDepartBlocked(int v, int lane);
    bool railBidiBlocked(int v, int lane, double pos) const;
    bool railSignalWasPassed(int link) const;
    void registerInsertionApproach(int v, int link, double dist);
    std::mutex railMutex;
    std::vector<int> railNewRoutes, railRerouted;
    void initRail();
    static void setRailType(VType& t, const std::vector<std::string>& tok);
    void railVehicleBuilt(int v);
    void railNewRoute(int v);
    void processRailNewRoutes();
    void railBeginStep();
    void railApproaches();
    double checkReversal(int v, bool& canReverse, double speedThreshold, double seen) const;
    bool railHaveDeadlock(int v, int& constraint, std::set<int>& seen,
                          std::vector<std::pair<int, RailWaitRelation>>* circle = nullptr) const;
    // deadlock hints (MSRailSignalControl::addDeadlockCheck, addDrivewayFollower, findDeadlockFoes)
    std::vector<std::vector<std::string>> railDeadlockLines;  // DEADLOCK lines (signal ids)
    std::map<int, std::vector<int>> railDeadlockChecks;       // rail signal -> the other signals of the circle
    std::map<int, std::set<int>> dwSucc, dwPred;
    void railAddDWDeadlockChecks(int ri, int dw);
    void railFindDeadlockFoes(int dw, const std::vector<int>& others, std::vector<int> deadlockFoes);

    void markEdgeDelayed(int lane) {
        edgeDelayed[net.lanes[lane].edge] = 1;
    }

    // lane changing (lc.cpp)
    friend struct LaneChanger;
    std::vector<Kin> kinSnap;
    std::vector<int> lcSnapStage;
    int lcCurStage = 0;
    std::vector<int> lcStateSnap;
    std::vector<double> lcLeftSpaceSnap;
    std::vector<double> bruttoSnap;
    std::vector<std::vector<LCMessage>> lcMessages;
    std::vector<std::vector<LaneChangeEvent>> lcEventsPerEdge;
    std::vector<std::vector<int>> lcShadowPerEdge;  // vehicles whose shadow lanes changed in the lane-change phase
    std::vector<int> edgeLevel;
    std::vector<std::vector<int>> edgesByLevel;
    std::vector<std::vector<int>> edgesByColor;  // no two lane-changing edges within a few hops share a color
    std::vector<int> lcEdges;  // edges with a lane changer
    std::vector<char> lcCheckOpened;  // MSLaneChanger::checkOpened per edge
    size_t nLaneChangesTotal = 0;
    bool laneAccel(int lane) const { return net.lanes[lane].accel; }
    void computeEdgeLevels();
    void changeLanes();
    std::vector<int> lcEdgeColor, lcColorRank;   // static color of each lane-changing edge, position in it
    std::vector<std::vector<int>> lcNbr;         // lane-changing edges within the color distance
    std::vector<int> lcSchedPos;                 // stage of each edge in the current phase (-1: none)
    std::vector<std::vector<int>> lcSched;
    std::vector<std::vector<LCMessage>> lcPending;
    std::atomic<long long> nLcScheduleConflicts{0};  // vehicles reaching onto two edges of one stage
    std::atomic<int> lcMaxSpanEdges{0};              // most edges a vehicle occupies at once
    long long nFurtherOverflow = 0;              // further lanes beyond MAX_FURTHER (not recorded)
    void checkLCSchedule(int v);
    std::vector<int> lcOccupied;                 // lanes with vehicles at the start of the phase
    std::vector<char> lcEdgeMark;
    void updatePartials(const std::vector<int>& changed, std::vector<int>& affected);
    std::vector<int> bruttoSnapLanes;
    std::vector<char> bruttoSnapMark;
    void snapshotBrutto(const std::vector<int>& occupied);
    void markBruttoSnap(int l) {
        if (!bruttoSnapMark[l]) {
            bruttoSnapMark[l] = 1;
            bruttoSnapLanes.push_back(l);
        }
    }
    void applyLCMessage(const LCMessage& m);

    // runtime edits (edit.cpp). Requests are queued from any thread and applied
    // single-threaded at the start of the next step, before any parallel phase.
    struct EditRequest {
        enum Kind { LANE_SPEED, LANE_PERMISSIONS, TLS_PROGRAM, EDGE_REFRESH } kind;
        int target;
        double value = 0;
        bool transient = false;  // a permission change of a rerouter (MSLane::setPermissions with a transient id)
        std::vector<Phase> phases;
    };
    std::vector<EditRequest> pendingEdits;
    std::vector<std::pair<Time, EditRequest>> scheduledEdits;  // --edits file, by time
    size_t nextScheduledEdit = 0;
    long long seenNetVersion = 0;
    void loadEdits(const std::string& file);
    void applyEdits();

    // emulation of MSEdgeControl::myActiveLanes (only with --sumo-order, for
    // step-by-step comparison with SUMO)
    std::vector<int> activeList;
    bool trackActiveList = false;  // keep activeList (SUMO's order of the active lanes)
    std::vector<char> amActive;
    std::vector<int> changedState;
    std::vector<int> lcToAdd;
    void activePatch();
    void activeRemoveEmpty();
    void activeAdd(int lane);

    // phases (sim.cpp)
    void stepOnce();
    void planMovements();
    void setJunctionApproaches();
    void executeMovements();
    void integrate();
    void rebuildPartials();
    void writeTripinfo(int v, bool unfinished = false);
    void removeFromNet(int v, int dwReason = DWR_ARRIVED);
    void releaseVehicle(int v);
    void writeOutputs();
    void writeRailSignalBlocks(bool vehicles) const;
    void writeDeadlock(const RailDeadlockFound& d);
    void writeDriveWayBlocks(FILE* f, int dw, int indent) const;
    void writeDriveWayVehicles(FILE* f, int dw, int indent) const;

    // output
    int traceVeh = -1;
    int tracePerson = -1;
    FILE* fcdFile = nullptr;
    void writeFcd();
    // edge data per lane (normal lanes), summed to edges per interval
    std::vector<double> edSampled, edDist, edWait;
    std::vector<int> edEntered;
    FILE* edFile = nullptr;
    Time edLastEnd = 0;  // end of the last written edge data interval
    FILE* sumFile = nullptr;
    void writeEdgeData(double begin, double end);
    void writeSummaryRow();
    // compressible: written gzip-compressed (name + ".gz") with --gzip
    FILE* openOutput(const std::string& name, bool compressible = false) const;
    FILE* lcFile = nullptr;  // lane changes, written at the end of every step
    void flushLaneChanges();

    // ---- persons and the striping pedestrian model (peds.cpp) --------------
    std::vector<Person> persons;
    std::vector<std::string> personDepartPos;  // departPos as given (resolved with the first stage)
    std::vector<std::vector<std::string>> personLines;  // PERSON / PWALK / PSTOP lines until the network is complete
    std::map<Time, std::vector<int>> waiting4Departure;  // MSTransportableControl::myWaiting4Departure
    std::map<Time, std::vector<int>> waitingUntil;       // MSTransportableControl::myWaitingUntil
    int nPersonsRunning = 0, nPersonsArrived = 0, nPersonsEnded = 0;
    int nPedsActive = 0, nPedsJammed = 0;
    double timegapCrossing = 2.0;  // --pedestrian.timegap-crossing
    StripingParams striping;
    PedRouter pedRouter;
    bool pedUsingInternalLanes = false;  // MSPModel::usingInternalLanes
    // intermodal routing (MSTransportableRouter): built when the first trip is routed, with the public transport
    // schedules known then (MSNet::adaptIntermodalRouter)
    IntermodalNet imNet;
    struct PTSched {
        bool flow;
        std::string id, line;
        Time begin, offset, lastDepart;
        long long repetitionNumber;
        std::vector<std::pair<std::string, Time>> stops;  // (bus stop, until), "" if no bus stop
    };
    std::vector<PTSched> ptScheds;
    std::map<int, std::vector<double>> imPrefFactor;  // per person type: 1 / routing preference of each edge
    void initPersonPreferences();
    const std::vector<double>* prefFactor(int type) const;
    std::string vDefDepartLane = "best_prob", vDefDepartSpeed = "avg";  // --default.departlane / departspeed
    void ensureIntermodal();
    static double imCarTravelTime(const void* owner, int edge, const IMTrip& trip, double time);
    int addRuntimeVehicle(Veh x, int type);
    double placeAccessPos(int stop, int parking, int edge, int pi, int stage, bool useRng) const;
    std::deque<PState> pstates;              // all walks (a deque keeps references valid)
    std::vector<std::vector<int>> pedsOnLane;  // MSPModel_Interacting::myActiveLanes
    std::set<int> pedActiveLanes;              // lanes that ever had pedestrians, by numerical id
    std::vector<std::vector<PersonApproach>> approachingPersons;  // per link (MSLink::myApproachingPersons)
    std::vector<WalkingAreaPath> waPaths;
    std::map<std::pair<int, int>, int> waPathIdx;  // (from lane, to lane) -> path
    std::vector<double> minNextLength;             // per walking area lane
    std::vector<std::vector<int>> waFoes;          // per walking area edge: vehicle lanes crossing it
    std::vector<int> edgePersons;                  // persons on each edge (MSEdge::myPersons, walking or waiting)
    bool edgeHasPersons(int edge) const { return !edgePersons.empty() && edgePersons[edge] > 0; }
    void edgeAddPerson(int edge, int pi);
    void edgeRemovePerson(int pi);
    std::vector<std::string> personRows;           // personinfo.csv
    FILE* pfcdFile = nullptr;

    void loadPersonLines();
    void parsePersonLine(const std::vector<std::string>& tok);
    void parsePersonStageLine(const std::vector<std::string>& tok);
    double interpretEdgePos(double pos, double maximumValue) const;
    double stopAccessPos(int stop, int edge) const;
    double parsePersonPosLat(const std::string& value, int edge, int departLane) const;
    void applyPedOptions();
    void initPersons();
    bool personsDone() const { return nPersonsEnded == (int)persons.size(); }
    // persons riding vehicles (transport.cpp)
    std::map<int, std::vector<int>> waiting4Vehicle;  // MSTransportableControl::myWaiting4Vehicle (per edge)
    std::map<int, std::vector<int>> waiting4Container;  // the same for containers (the container control)
    int nWaitingForVehicle = 0;
    int nWalking = 0;                                 // persons in a walking stage (MSTransportableControl::getMovingNumber)
    bool haveNewWaiting = false;                      // MSTransportableControl::myHaveNewWaiting
    std::map<int, std::vector<int>> triggeredWaiting;  // vehicles waiting for their triggered departure (MSEdge::myWaiting)
    int nTriggeredDepart = 0;                         // such vehicles (sorted to the end of the vehicles)
    std::vector<int> triggeredDue;                    // triggered in this step, inserted from this step on
    std::atomic<int> nVehWaiting{0};                  // MSVehicleControl::myWaitingForTransportable
    bool rideArrivedWithVehicle = false;              // (MSStageDriving::setArrived: vehicleArrived)
    struct Alighting {
        int person, vehicle;
        int lane, laneVehicles;  // the vehicle's lane and its vehicle number during the move (MSLane::executeMovements)
    };
    std::vector<std::vector<Alighting>> laneAlight;  // persons leaving vehicles in the move phase, per lane task
    int alightLane = -1, alightLaneVehicles = 0;     // (laneVehicleNumberWithPartials while a person alights)
    struct DeferredPlan {
        int veh, leader;
        double lengthsInFront;
        std::shared_ptr<LeaderInfo> ahead;  // (sublane model)
    };
    std::vector<std::vector<DeferredPlan>> planDeferred;  // per lane task: vehicles planned after the parallel loop
    bool rideWaitingFor(const PStage& s, int v) const;
    bool vehAllowsBoarding(int v, int pi) const;
    bool vehStoppedInRange(int v, double pos, double tolerance, bool checkFuture) const;
    double vehOdometer(int v) const;
    void rideSetVehicle(int pi, int v);
    void vehAddPerson(int v, int pi);
    void rideRegisterWaiting(int pi);
    void rideProceed(int pi, int priorIdx);
    void triggerDeparture(int v);
    bool loadAnyWaiting(int edge, int v, Time& timeToLoadNext, Time& stopDuration, bool containers);
    void boardTransportables(int v);
    void stopTriggerWaiting(int v);
    void ridersNotifyMove(int v, int laneVehicles, std::vector<Alighting>& alight);
    void transportablesNotifyMove(int v, int laneVehicles, std::vector<int>& riders, bool& stopped, bool containers,
                                  std::vector<Alighting>& alight);
    bool rideCanLeave(int pi, int v, const Stop& stop) const;
    void applyAlightings(const std::vector<Alighting>& alight);
    void ridersVehicleArrived(int v);
    void rideSetArrived(int pi);
    void personRerouteParkingArea(int pi, int orig, int replacement);
    void abortAnyWaitingForVehicle();
    double stopWaitingPosOnLane(int stop, int pi) const;
    Pos stopWaitPosition(int stop, int pi) const;
    Pos vehPositionParked(int v) const;
    double vehAngleParked(int v) const;
    bool mayBoard(int v) const;  // boarding is possible in this step: planned serially
    // trains joining and splitting at stops (planned serially: they change the other train part)
    bool haveJoinSplit = false;
    bool railHasJoin(int ego, int foe) const;
    bool joinTrainPart(int t, int v);
    bool joinTrainPartFront(int t, int v);
    bool willTransferAtJoin(int pi, int joinVeh) const;
    void transferAtSplitOrJoin(int v, int other);
    // MSTransportableControl / MSTransportable / MSStage*
    void personCheckWaiting();
    bool personProceed(int pi);
    void stageProceed(int pi, int priorIdx);
    void tripProceed(int pi, int priorIdx);
    bool checkAccess(int pi, int priorIdx, bool waitAtStop);
    void processAccessEnds(Time until);
    std::multimap<Time, int> accessEvents;  // the ends of access stages (MSPersonStage_Access::ProceedCmd)
    void personErase(int pi);
    void stopAddPerson(int stop, int pi);
    void stopRemovePerson(int stop, int pi);
    int stageEdge(const PStage& s) const;
    double stageEdgePos(int pi, int si) const;
    double personMaxSpeed(int pi) const;
    double walkMaxSpeed(int pi, const PStage& s) const;
    double laneMaxSpeedFor(int lane, int pi) const;
    int canTraverse(int dir, const std::vector<int>& route, int& passedEdges) const;
    double walkDistance(const Person& p, const PStage& s, bool partial) const;
    bool stageMoveToNextEdge(int pi, int prevDir, int nextInternal);
    // striping model: geometry and walking area paths
    void initWalkingAreaPaths();
    int getArbitraryPath(int walkingAreaEdge) const;
    int getWalkingAreaPath(int walkingAreaEdge, int before, int after) const;
    int numStripes(int lane) const;
    int stripeOf(double relY) const;
    int otherStripeOf(const PState& p, double relY) const;
    double pedWidth(const PState& p) const;
    double pedLength(const PState& p) const;
    double pedMinGap(const PState& p) const;
    int pedStripe(const PState& p) const;
    int pedOtherStripe(const PState& p) const;
    double pedMinX(const PState& p, bool includeMinGap) const;
    double pedMaxX(const PState& p, bool includeMinGap) const;
    PedObstacle obstacleOf(const PState& p) const;
    double distanceTo(const PState& p, const PedObstacle& obs, bool includeMinGap) const;
    void mergeObstacles(const PState& p, std::vector<PedObstacle>& into, const std::vector<PedObstacle>& obs2) const;
    static void mergeObstaclesDir(std::vector<PedObstacle>& into, const std::vector<PedObstacle>& obs2, int dir, int offset);
    const PStage& walkStage(const PState& p) const;
    PStage& walkStageMut(const PState& p);
    int nextRouteEdge(const PStage& s) const;
    double distToLaneEnd(const PState& p) const;
    double pedImpatience(const PState& p) const;
    bool pedIgnoreRed(const PState& p, int link) const;
    bool pedStopForYellow(const PState& p, int link) const;
    Pos pedPosition(const PState& p) const;
    double pedAngle(const PState& p) const;
    const std::string& pedID(const PState& p) const;
    // next lane, crossings
    int getNextWalkingArea(int currentLane, int dir, int& link) const;
    int connectedDirection(int from, int to) const;
    PedNextLane getNextLane(const PState& p, int currentLane, int prevLane);
    int internalFollowingLane(int from, int target) const;
    bool isPriorityCrossing(int lane) const;
    void registerCrossingApproach(const PState& p, int crossing, int beforeWA);
    void unregisterCrossingApproach(int person, int crossing);
    // movement
    void pedAdd(int pi);
    void pedLaneAdd(int lane, int psi);
    int laneVehicleNumberWithPartials(int lane) const;
    std::vector<PedObstacle> getNeighboringObstacles(const std::vector<PState*>& peds, int egoIndex, int stripes) const;
    static int getStripeOffset(int origStripes, int destStripes, bool addRemainder);
    static void transformToCurrentLanePositions(std::vector<PedObstacle>& obs, int currentDir, int nextDir, double currentLength,
                                                double nextLength);
    static void addCloserObstacle(std::vector<PedObstacle>& obs, double x, int stripe, int numStripes, int person, double width,
                                  int dir, int type);
    void sortPeds(std::vector<int>& v, int dir) const;
    void sortPedPtrs(std::vector<PState*>& v, int dir) const;
    const std::vector<PedObstacle>& getNextLaneObstacles(std::map<int, std::vector<PedObstacle>>& nextLanesObs, int lane, int nextLane,
                                                         int stripes, int nextDir, double currentLength, int currentDir);
    void movePedestrians();
    void moveInDirection(int dir);
    void moveLaneInDirection(int lane, int dir);
    void arriveAndAdvance(int lane, int dir);
    void moveInDirectionOnLane(std::vector<PState*>& peds, int lane, int dir);
    // effects of a lane task on other lanes and shared data, applied in lane order after a phase
    struct PedEvent {
        enum Kind { REGISTER, UNREGISTER, ADD, FINISH } kind = ADD;
        int person = -1;
        int a = -1;    // REGISTER / UNREGISTER: crossing link, ADD: lane, FINISH: walking direction
        int psi = -1;  // ADD
        Time arrivalTime = 0, leavingTime = 0;  // REGISTER
    };
    void applyPedEvent(const PedEvent& ev);
    std::vector<std::vector<PedEvent>> pedEventsOfTask;
    std::vector<PState> pedSnap;                 // pedestrian states at the start of a phase (by pstates index)
    std::vector<std::vector<int>> pedSnapOnLane;  // pedestrians per lane at the start of a wave
    std::vector<char> pedInWave;                  // per lane: belongs to the running wave
    std::vector<char> pedChanged;                 // per person: changed the lane in this step (only an obstacle)
    bool pedParallel = false;                     // inside the parallel lane tasks
    int getReserved(int stripes, double factor) const;
    void pedWalk(PState& p, const std::vector<PedObstacle>& obs);
    void pedMoveToNextLane(int psi);
    // interaction with vehicles
    bool addCrossingVehs(int crossing, int stripes, double lateralOffset, int dir, std::vector<PedObstacle>& obs, bool prio, bool flipY);
    std::vector<PedObstacle> getVehicleObstacles(int lane, int dir, const PState* ped);
    void addWalkingAreaVehicleFoes(int lane, const WalkingAreaPath& path, int dir, double lateralOffset, double minY, double maxY,
                                   std::deque<PState>& tmp, std::vector<PState*>& transformed);
    Pos vehPosition(int v) const;
    Pos vehBackPosition(int v) const;
    double vehAngle(int v) const;
    Pos vehPositionAtOffset(int v, double offset) const;
    void checkWalkingAreaFoe(int ego, int link, int foeLane, std::vector<LinkLeader>& result) const;
    // output
    void writePersonInfo(int pi);
    void writePersonFcd();
    // non-interacting pedestrian model (--pedestrian.model nonInteracting)
    bool pedNonInteracting = false;
    std::map<std::pair<Time, uint64_t>, int> niEvents;  // MoveToNextEdge events (time, insertion order) -> person
    uint64_t niEventSeq = 0;
    Time niComputeDuration(int pi, int prevEdge);
    void niAdd(int pi);
    void niStep();
    double niEdgePos(const PStage& s) const;
};

// rear to front over full and partial occupants, merged by position
// (MSLane::AnyVehicleIterator in downstream direction)
template <class F>
void Sim::anyVehicles(int lane, F f) const {
    const LaneRT& l = lanes[lane];
    size_t i1 = 0, i2 = 0;
    while (i1 < l.vehs.size() || i2 < l.partials.size()) {
        bool takeFull;
        if (i1 == l.vehs.size()) {
            takeFull = false;
        } else if (i2 == l.partials.size()) {
            takeFull = true;
        } else {
            takeFull = kin[l.vehs[i1]].pos < positionOnLane(l.partials[i2], lane);
        }
        const int v = takeFull ? l.vehs[i1++] : l.partials[i2++];
        if (!f(v)) {
            return;
        }
    }
}

// Leaders (or followers) of a lane per sublane (MSLeaderInfo, MSLeaderDistanceInfo,
// MSCriticalFollowerDistanceInfo); one slot without a lateral resolution
struct LeaderInfo {
    const Sim* sim;
    double width;
    int offset = 0;             // shifts the interpretation of the sublane borders (in sublanes)
    std::vector<int> vehs;      // per sublane, -1: none
    int freeSublanes;
    int egoRightMost = -1, egoLeftMost = -1;
    bool hasVehs = false;

    LeaderInfo() : sim(nullptr), width(0), freeSublanes(0) {}
    LeaderInfo(const Sim& s, double laneWidth, int ego = -1, double latOffset = 0.);
    virtual ~LeaderInfo() = default;
    int addLeader(int v, bool beyond, double latOffset = 0.);
    virtual void clear();
    void getSubLanes(int v, double latOffset, int& rightmost, int& leftmost) const;
    void getSublaneBorders(int sublane, double latOffset, double& rightSide, double& leftSide) const;
    int operator[](int sublane) const { return vehs[sublane]; }
    int numSublanes() const { return (int)vehs.size(); }
    bool hasVehicles() const { return hasVehs; }
    bool hasStoppedVehicle() const;
    bool hasVehicle(int v) const;
    void setSublaneOffset(int o) { offset = o; }
    int numFreeSublanes() const { return freeSublanes; }
};

struct LeaderDistanceInfo : LeaderInfo {
    std::vector<double> dists;

    LeaderDistanceInfo() = default;
    LeaderDistanceInfo(const Sim& s, double laneWidth, int ego = -1, double latOffset = 0.)
        : LeaderInfo(s, laneWidth, ego, latOffset), dists(vehs.size(), std::numeric_limits<double>::max()) {}
    // a single leader (MSLeaderDistanceInfo(CLeaderDist, laneWidth))
    LeaderDistanceInfo(const Sim& s, int v, double dist, double laneWidth);
    int addLeader(int v, double gap, double latOffset = 0, int sublane = -1);
    void addLeaders(const LeaderDistanceInfo& other);
    void clear() override;
    std::pair<int, double> at(int sublane) const { return {vehs[sublane], dists[sublane]}; }
    void patchGaps(double amount);
    std::pair<int, double> getClosest() const;
    double getMinDistToStopped() const;
    void moveSamePosTo(int ego, LeaderDistanceInfo& other);
};

struct CriticalFollowerDistanceInfo : LeaderDistanceInfo {
    std::vector<double> missingGaps;
    bool haveOppositeLeaders;

    CriticalFollowerDistanceInfo(const Sim& s, double laneWidth, int ego = -1, double latOffset = 0., bool haveOpposite = false)
        : LeaderDistanceInfo(s, laneWidth, ego, latOffset), missingGaps(vehs.size(), -std::numeric_limits<double>::max()),
          haveOppositeLeaders(haveOpposite) {}
    int addFollower(int v, int ego, double gap, double latOffset = 0, int sublane = -1);
    void clear() override;
};

}  // namespace ps
