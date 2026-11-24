// psim2: a parallel-by-design re-implementation of Eclipse SUMO's microscopic
// simulation (Krauss car-following, LC2013 lane changing, junctions with
// internal lanes and right-of-way, traffic lights, stops, teleports). Logic
// follows SUMO 1.28 (src/microsim); execution structure, data layout and
// inter-vehicle dependencies are redesigned.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace ps {

// ---- time ------------------------------------------------------------------
// Time is SUMO's SUMOTime (integer milliseconds). DELTA_T is the step length in
// ms and TS the same in seconds (SUMO's TS macro). Both are set at load time.
using Time = long long;
extern Time DELTA_T;
extern double TS;
// --step-method.ballistic (MSGlobals::gSemiImplicitEulerUpdate == !BALLISTIC), set at load time
extern bool BALLISTIC;
constexpr Time TIME_MAX = std::numeric_limits<Time>::max() - 1000;  // SUMOTime_MAX
constexpr Time NO_EVENT = std::numeric_limits<Time>::min();  // (no event scheduled)
inline constexpr Time TIME2STEPS(double x) { return static_cast<Time>(x * 1000. + (x >= 0 ? 0.5 : -0.5)); }
inline constexpr double STEPS2TIME(Time x) { return static_cast<double>(x) / 1000.; }

inline double ACCEL2SPEED(double x) { return x * TS; }
inline double SPEED2DIST(double x) { return x * TS; }
inline double DIST2SPEED(double x) { return x / TS; }
inline double ACCEL2DIST(double x) { return x * TS * TS; }
inline double SPEED2ACCEL(double x) { return x / TS; }

// ---- constants (SUMO config.h / StdDefs.h / MSVehicle.cpp) -----------------
constexpr double NUMERICAL_EPS = 0.001;
constexpr double POSITION_EPS = 0.1;
constexpr double SPEED_EPS = 0.01;
constexpr double HALTING_SPEED = 0.1;  // SUMO_const_haltingSpeed
constexpr double INVALID_SPEED = 299792458.0 + 1;
constexpr double EMERGENCY_DECEL_AMPLIFIER = 1.2;
constexpr double DMAX = std::numeric_limits<double>::max();
inline double NUMERICAL_EPS_SPEED() { return 0.1 * NUMERICAL_EPS * TS; }

// ---- lane change action flags (SUMOXMLDefinitions.h) -----------------------
enum : int {
    LCA_NONE = 0,
    LCA_STAY = 1 << 0,
    LCA_LEFT = 1 << 1,
    LCA_RIGHT = 1 << 2,
    LCA_STRATEGIC = 1 << 3,
    LCA_COOPERATIVE = 1 << 4,
    LCA_SPEEDGAIN = 1 << 5,
    LCA_KEEPRIGHT = 1 << 6,
    LCA_TRACI = 1 << 7,
    LCA_URGENT = 1 << 8,
    LCA_BLOCKED_BY_LEFT_LEADER = 1 << 9,
    LCA_BLOCKED_BY_LEFT_FOLLOWER = 1 << 10,
    LCA_BLOCKED_BY_RIGHT_LEADER = 1 << 11,
    LCA_BLOCKED_BY_RIGHT_FOLLOWER = 1 << 12,
    LCA_OVERLAPPING = 1 << 13,
    LCA_INSUFFICIENT_SPACE = 1 << 14,
    LCA_SUBLANE = 1 << 15,
    LCA_AMBLOCKINGLEADER = 1 << 16,
    LCA_AMBLOCKINGFOLLOWER = 1 << 17,
    LCA_MRIGHT = 1 << 18,
    LCA_MLEFT = 1 << 19,
    LCA_AMBLOCKINGFOLLOWER_DONTBRAKE = 1 << 21,
    LCA_CHANGE_TO_HELP = 1 << 23,
    LCA_AMBACKBLOCKER = 1 << 26,
    LCA_AMBACKBLOCKER_STANDING = 1 << 27,
    LCA_INSUFFICIENT_SPEED = 1 << 28,
    LCA_UNKNOWN = 1 << 30,
    LCA_WANTS_LANECHANGE = LCA_LEFT | LCA_RIGHT,
    LCA_BLOCKED_LEFT = LCA_BLOCKED_BY_LEFT_LEADER | LCA_BLOCKED_BY_LEFT_FOLLOWER,
    LCA_BLOCKED_RIGHT = LCA_BLOCKED_BY_RIGHT_LEADER | LCA_BLOCKED_BY_RIGHT_FOLLOWER,
    LCA_BLOCKED_BY_LEADER = LCA_BLOCKED_BY_LEFT_LEADER | LCA_BLOCKED_BY_RIGHT_LEADER,
    LCA_BLOCKED_BY_FOLLOWER = LCA_BLOCKED_BY_LEFT_FOLLOWER | LCA_BLOCKED_BY_RIGHT_FOLLOWER,
    LCA_BLOCKED = LCA_BLOCKED_LEFT | LCA_BLOCKED_RIGHT | LCA_INSUFFICIENT_SPACE | LCA_INSUFFICIENT_SPEED,
    LCA_CHANGE_REASONS = LCA_STRATEGIC | LCA_COOPERATIVE | LCA_SPEEDGAIN | LCA_KEEPRIGHT | LCA_SUBLANE | LCA_TRACI,
};

// ---- counter-based random numbers ------------------------------------------
// SUMO draws from per-lane Mersenne Twisters in processing order. Here every
// draw is a pure function of (seed, vehicle, step, purpose), so results do not
// depend on thread count or processing order.
inline uint64_t mix64(uint64_t z) {
    z += 0x9e3779b97f4a7c15ULL;
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
    return z ^ (z >> 31);
}

inline double rand01(uint64_t seed, uint64_t a, uint64_t b, uint64_t c) {
    const uint64_t h = mix64(seed ^ mix64(a ^ mix64(b ^ mix64(c + 0x632be59bd9b4e019ULL))));
    return (double)(h >> 11) * (1.0 / 9007199254740992.0);
}

enum RandPurpose : uint64_t {
    RP_DAWDLE = 1,
    RP_SPEEDFACTOR = 2,  // drawn once per vehicle
    RP_DEADLOCK = 3,     // right_before_left deadlock resolution
    RP_DEPARTLANE = 4,   // departLane="random", per insertion attempt
    RP_DEPARTPOS = 5,    // departPos="random" / "random_free", per insertion attempt
    RP_DEPARTSPEED = 6,  // departSpeed="random", per insertion attempt
    RP_ARRIVAL = 7,      // arrivalPos / arrivalLane="random", once per vehicle
    RP_FLOW = 8,         // Poisson and probability flows
    RP_PERSON_DEPARTPOS = 9,   // person departPos="random"
    RP_PERSON_ARRIVALPOS = 10, // walk arrivalPos="random"
    RP_PERSON_POSLAT = 11,     // walk departPosLat="random"
    RP_PED_DAWDLE = 12,        // striping model dawdling, per person and step
    RP_PERSON_SPEEDFACTOR = 13,
    RP_JM_IGNORE = 14,          // jmIgnoreFoeProb / jmIgnoreJunctionFoeProb, per decision
    RP_PERSON_FLOW = 15,        // probabilistic and Poisson person flows
    RP_REROUTE_EQUIP = 16,      // device.rerouting.probability, once per vehicle
    RP_REROUTER = 17,           // rerouter probability and destination choice, per vehicle and step
    RP_PARKING = 18,            // parking search of rerouters (occupancy guesses, probabilities)
    RP_TYPEDIST = 19,           // vTypeDistribution, once per vehicle
    RP_ACCESS = 20,             // random access positions of stopping places (person and stage)
    RP_TRIP_VEHICLE = 21,       // speed factors of the vehicles of person trips (person and stage)
    RP_PERSON_ROUTE = 22,       // --weights.random-factor of person routes (query and edge)
    RP_CF = 23,                 // car-following models' own draws (vehicle, step and call)
    RP_CF_STATE = 24,           // car-following model state drawn once per step (BKerner, EIDM)
    RP_LC_OVERTAKE_RIGHT = 25,  // lcOvertakeRight (vehicle and step)
    RP_ALLWAY = 26,             // all-way stop tie breaker of the arrival times (vehicle, drive item and step)
    RP_LC_SIGMA = 27,           // lcSigma lateral drift of the sublane model (vehicle and step)
    RP_DEPART_POSLAT = 28,      // departPosLat="random" / "random_free", per insertion attempt
};

// RandHelper::randNorm (polar method), u() draws uniformly from [0, 1)
template <class U>
inline double randNormPolar(double mean, double variance, U u) {
    double x, q;
    do {
        x = 2 * u() - 1;
        const double y = 2 * u() - 1;
        q = x * x + y * y;
    } while (q == 0.0 || q >= 1.0);
    const double logRounded = std::ceil(std::log(q) * 1e14) / 1e14;
    return mean + variance * x * std::sqrt(-2 * logRounded / q);
}

// RandHelper::randExp
inline double randExp(double rate, double u) {
    return -std::log(1.0 - u) / rate;
}

// SUMO's hash randomness (same values as SUMO): RandHelper::randHash, RandHelper::murmur3_32
inline double sumoRandHash(long long x) {
    return (double)(mix64((uint64_t)x) >> 11) / (double)(1ULL << 53);
}

inline uint32_t sumoMurmur3(const std::string& key, int seed) {
    auto scramble = [](uint32_t k) {
        k *= 0xcc9e2d51;
        k = (k << 15) | (k >> 17);
        k *= 0x1b873593;
        return k;
    };
    const uint8_t* p = reinterpret_cast<const uint8_t*>(key.data());
    const size_t len = key.size();
    uint32_t h = (uint32_t)seed;
    uint32_t k;
    for (size_t i = len >> 2; i; i--) {
        std::memcpy(&k, p, sizeof(uint32_t));
        p += sizeof(uint32_t);
        h ^= scramble(k);
        h = (h << 13) | (h >> 19);
        h = h * 5 + 0xe6546b64;
    }
    k = 0;
    for (size_t i = len & 3; i; i--) {
        k <<= 8;
        k |= p[i - 1];
    }
    h ^= scramble(k);
    h ^= (uint32_t)len;
    h ^= h >> 16;
    h *= 0x85ebca6b;
    h ^= h >> 13;
    h *= 0xc2b2ae35;
    h ^= h >> 16;
    return h;
}

}  // namespace ps
