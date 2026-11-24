// Krauss and Rail car-following models with semi-implicit Euler update.
// Ported from SUMO 1.28: MSCFModel.cpp, MSCFModel_Krauss.cpp, MSCFModel_Rail.cpp.
#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "common.hpp"

namespace ps {

// MSCFModel_Rail::TrainParams (flat tracks, no curve resistance)
struct RailCF {
    static constexpr double INVALID = std::numeric_limits<double>::max();
    double weight = 0;  // tons
    double mf = 1;      // mass factor
    double decl = 0;
    double vmax = 0;
    std::map<double, double> traction, resistance;  // m/s -> kN
    double maxPower = INVALID, maxTraction = INVALID;  // kN
    double resCoefConstant = INVALID, resCoefLinear = INVALID, resCoefQuadratic = INVALID;
    // curve resistance (Röckl): smoothed over curveIntegration m of track
    double curveResistance = 0, curveIntegration = 100, roecklSharpRadius = 300, roecklNumerator = 6380,
           roecklNumeratorSharp = 4910, roecklOffset = 55, roecklOffsetSharp = 30;

    // LinearApproxHelpers::getInterpolatedValue
    static double interpolated(const std::map<double, double>& map, double x) {
        auto low = map.lower_bound(x);
        if (low == map.end()) {
            return map.rbegin()->second;
        }
        if (low == map.begin()) {
            return low->second;
        }
        auto prev = std::prev(low);
        const double weight = (x - prev->first) / (low->first - prev->first);
        return (1 - weight) * prev->second + weight * low->second;
    }
    double getResistance(double speed) const {  // kN
        if (resCoefConstant != INVALID) {
            return resCoefQuadratic * speed * speed + resCoefLinear * speed + resCoefConstant;
        }
        return interpolated(resistance, speed);
    }
    double getTraction(double speed) const {  // kN
        if (maxPower != INVALID) {
            return std::min(maxPower / speed, maxTraction);
        }
        return interpolated(traction, speed);
    }
    double rotWeight() const { return weight * mf; }
};

// the car-following model of a vehicle type (MSVehicleType::build)
enum class CFM : uint8_t {
    KRAUSS, KRAUSS_ORIG1, KRAUSS_PS, KRAUSSX, SMART_SK, DANIEL1, PWAGNER2009, IDM, IDMM, EIDM, BKERNER, WIEDEMANN, W99,
    RAIL, ACC, CACC, CC, NASCH
};

// LatAlignmentDefinition (the codes of tools/psim2scen.py)
enum LatAlign { LA_RIGHT = 0, LA_CENTER = 1, LA_ARBITRARY = 2, LA_NICE = 3, LA_COMPACT = 4, LA_LEFT = 5, LA_GIVEN = 6 };

struct VType {
    std::string id;
    CFM cfm = CFM::KRAUSS;
    std::map<std::string, std::string> cfAttrs;  // the model's own attributes as given (SUMOVTypeParameter::cfParameter)
    std::vector<double> cfp;                      // the model's parameters with their defaults (Sim::initCarFollowModel)
    double startupDelay = 0;                      // seconds (MSCFModel::myStartupDelay)
    double actionStepLength = 0;                  // seconds as given (0: --default.action-step-length)
    double speedFactorPremature = -1;             // slow down to arrive at stops not before their arrival
    std::map<double, double> maxAccelProfile, desAccelProfile;  // speed -> acceleration (MSCFModel::getCurrentAccel)
    double collisionMinGapFactor = 1;             // MSCFModel::myCollisionMinGapFactor
    long long vclass = 1LL << 6;  // SUMO SVC_PASSENGER
    // speed factor distribution normc(mean, dev, min, max)
    double speedFactorMean = 1, speedFactorDev = 0.1, speedFactorMin = 0.2, speedFactorMax = 2;
    // car-following
    double accel = 2.6, decel = 4.5, emergencyDecel = 9.0, apparentDecel = 4.5;
    double tau = 1.0, sigma = 0.5;
    double length = 5.0, minGap = 2.5, maxSpeed = 55.55, desiredMaxSpeed = 10000.0 / 3.6;
    double width = 1.8;
    double locomotiveLength = -1;  // SUMOVTypeParameter::locomotiveLength (the shortest train after a split)
    int shapeKind = 0;             // guiShape for the outline: 0 box, 1 passenger car (cut corners), 2 along the further lanes
    // transportables (SUMOVTypeParameter: personCapacity, containerCapacity, boardingDuration, loadingDuration,
    // boardingFactor of the transportable's type)
    int personCapacity = 4, containerCapacity = 0;
    long long boardingDuration = 500, loadingDuration = 90000;  // milliseconds
    double boardingFactor = 1;
    // junction model / pedestrians (jmDriveAfterRedTime, jmDriveAfterYellowTime, impatience,
    // pedestrian.timegap-crossing; -1 = not set)
    double impatience = 0, jmDriveAfterRedTime = -1, jmDriveAfterYellowTime = -1, timegapCrossing = -1;
    // junction model parameters (-1 = not set, the default depends on the use)
    double jmIgnoreFoeProb = 0, jmIgnoreFoeSpeed = 0, jmIgnoreJunctionFoeProb = 0, jmCrossingGap = 10;
    double jmStoplineGap = -1, jmStoplineGapMinor = -1, jmStoplineCrossingGap = 1.0, jmTimegapMinor = 1.0;
    double jmAllwayStopWait = -1, jmStopSignWait = -1;  // seconds (-1: the step length)
    // LC2013
    double lcStrategic = 1, lcCooperative = 1, lcSpeedGain = 1, lcKeepRight = 1;
    double lcLookaheadLeft = 2.0, lcSpeedGainRight = 0.1, lcSpeedGainLookahead = 0;
    double lcSpeedGainRemainTime = 20, lcSpeedGainUrgency = 50;
    double lcCooperativeSpeed = -1;  // defaults to lcCooperative
    double lcKeepRightAcceptanceTime = -1, lcOvertakeDeltaSpeedFactor = 0;
    double lcAssertive = 1, lcCooperativeHelpTime = 60, lcCooperativeHelpThreshold = -1;
    double lcCooperativeMinSpeed = 0, lcStrategicLookahead = -1, lcExperimental1 = 0;
    double lcCooperativeRoundabout = -1;  // defaults to lcCooperative
    double lcOvertakeRight = 0;
    double maxSpeedLat = 1.0;             // lateral speed (continuous lane changes)
    bool maxSpeedLatSet = false;
    double lcMaxSpeedLatStanding = -1, lcMaxSpeedLatFactor = -1;  // -1: not given (maxSpeedLat, 1)
    // SL2015 (sublane model); lcModel 0: the default (LC2013, SL2015 with a lateral resolution), 1: LC2013, 2: SL2015
    int lcModel = 0;
    bool lcSpeedGainLookaheadSet = false;  // (SL2015 default: 5)
    double lcSublane = 1, lcPushy = -1, lcPushyGap = -1, lcImpatience = 0, lcTimeToImpatience = DMAX, lcAccelLat = 1.0;
    double lcTurnAlignmentDistance = 0, lcLaneDiscipline = 0, lcSigma = 0, lcOpposite = 1;
    double lcMaxDistLatStanding = -1;  // -1: not given (two-wheelers: unlimited, others 1.6)
    double minGapLat = -1;             // -1: not given (bicycles, scooters 0.35, others 0.6)
    // SUMOVTypeParameter::latAlignmentProcedure (LatAlignmentDefinition codes, -1: the class default)
    int latAlignment = -1;
    double latAlignmentOffset = 0;
    // derived
    long long changeProbThresholdRight = 0, changeProbThresholdLeft = 0;
    std::shared_ptr<const RailCF> rail;  // the Rail car-following model (Krauss when null)

    double lengthWithGap() const { return length + minGap; }

    void initDerived() {
        if (lcCooperativeSpeed < 0) {
            lcCooperativeSpeed = lcCooperative;
        }
        constexpr long long BICYCLE = 1LL << 16, SCOOTER = 1LL << 29, MOTORCYCLE = 1LL << 14, MOPED = 1LL << 15;
        if (minGapLat < 0) {
            minGapLat = (vclass & (BICYCLE | SCOOTER)) != 0 ? 0.35 : 0.6;
        }
        if (latAlignment < 0) {
            latAlignment = (vclass & (BICYCLE | SCOOTER)) != 0 ? LA_RIGHT : LA_CENTER;
        }
        if (lcMaxDistLatStanding < 0) {
            lcMaxDistLatStanding = (vclass & (BICYCLE | MOTORCYCLE | MOPED)) != 0 ? DMAX : 1.6;
        }
        if (lcPushy < 0) {
            // MSLCM_SL2015: lcPushy, otherwise derived from lcPushyGap
            const double gapLat = std::max(NUMERICAL_EPS, minGapLat);
            lcPushy = 1 - (lcPushyGap >= 0 ? lcPushyGap : gapLat) / gapLat;
        }
        constexpr double HYST_PRECISION = 10000000;
        if (lcSpeedGain <= 0) {
            changeProbThresholdRight = std::numeric_limits<long long>::max();
            changeProbThresholdLeft = std::numeric_limits<long long>::max();
        } else {
            changeProbThresholdRight = (long long)((0.2 / lcSpeedGainRight) / lcSpeedGain * HYST_PRECISION);
            changeProbThresholdLeft = (long long)(0.2 / lcSpeedGain * HYST_PRECISION);
        }
    }
};

namespace cf {

// MSCFModel::passingTime
inline double passingTime(double lastPos, double passedPos, double currentPos, double lastSpeed, double currentSpeed) {
    if (passedPos > currentPos || passedPos < lastPos) {
        const double lastCoveredDist = currentPos - lastPos;
        return passedPos > currentPos ? TS * (passedPos - lastPos) / lastCoveredDist : TS * (currentPos - passedPos) / lastCoveredDist;
    }
    const double distanceOldToPassed = passedPos - lastPos;
    if (!BALLISTIC) {
        // constantly moving with currentSpeed
        if (currentSpeed == 0) {
            return TS;
        }
        const double t = distanceOldToPassed / currentSpeed;
        return std::min(TS, std::max(0., t));
    }
    // ballistic: constant acceleration, except for a stop within the step
    const double a = currentSpeed > 0 ? SPEED2ACCEL(currentSpeed - lastSpeed) : lastSpeed * lastSpeed / (2 * (lastPos - currentPos));
    if (std::fabs(a) < NUMERICAL_EPS) {
        const double t = 2 * distanceOldToPassed / (lastSpeed + currentSpeed);
        return std::min(TS, std::max(0., t));
    } else if (a > 0) {
        const double va = lastSpeed / a;
        return -va + std::sqrt(va * va + 2 * distanceOldToPassed / a);
    }
    const double va = lastSpeed / a;
    const double t = -va - std::sqrt(va * va + 2 * distanceOldToPassed / a);
    return std::min(TS, std::max(0., t));
}

// MSCFModel_Rail::maxNextSpeed (traction and resistance; slope in degrees)
// (curveRes: MSCFModel_Rail::getCurveResistance in kN/t)
inline double railMaxNextSpeed(const RailCF& r, double speed, double slope = 0, double curveRes = 0) {
    if (speed >= r.vmax) {
        return r.vmax;
    }
    const double gr = r.weight * 9.80665 * std::sin(slope * M_PI / 180.);  // kN
    const double totalRes = r.getResistance(speed) + gr + r.weight * curveRes;  // kN
    const double trac = r.getTraction(speed);  // kN
    const double a = (trac - totalRes) / r.rotWeight();  // kN/t == N/kg
    return std::min(r.vmax, speed + ACCEL2SPEED(a));
}

// MSCFModel_Rail::minNextSpeed: braking is supported by the resistance
inline double railMinNextSpeed(const RailCF& r, double speed, double slope = 0, double curveRes = 0) {
    const double gr = r.weight * 9.80665 * std::sin(slope * M_PI / 180.);
    const double a = r.decl + (r.getResistance(speed) + gr + r.weight * curveRes) / r.rotWeight();
    return BALLISTIC ? speed - ACCEL2SPEED(a) : std::max(speed - ACCEL2SPEED(a), 0.);
}

// MSCFModel::getCurrentAccel: the acceleration profiles limit the acceleration by the speed
inline double currentAccel(const VType& t, double speed) {
    double result = t.accel;
    if (!t.desAccelProfile.empty()) {
        result = std::min(result, RailCF::interpolated(t.desAccelProfile, speed));
    }
    if (!t.maxAccelProfile.empty()) {
        result = std::min(result, RailCF::interpolated(t.maxAccelProfile, speed));
    }
    return result;
}

inline double maxNextSpeed(const VType& t, double speed) {
    if (t.rail) {
        return railMaxNextSpeed(*t.rail, speed);
    }
    return std::min(speed + ACCEL2SPEED(currentAccel(t, speed)), t.maxSpeed);
}

// (the ballistic update allows negative speeds to indicate a stop within the next step)
inline double minNextSpeed(const VType& t, double speed) {
    if (t.rail) {
        // MSCFModel_Rail::minNextSpeed: braking is supported by the resistance
        const RailCF& r = *t.rail;
        const double a = r.decl + r.getResistance(speed) / r.rotWeight();
        return BALLISTIC ? speed - ACCEL2SPEED(a) : std::max(speed - ACCEL2SPEED(a), 0.);
    }
    return BALLISTIC ? speed - ACCEL2SPEED(t.decel) : std::max(speed - ACCEL2SPEED(t.decel), 0.);
}

inline double minNextSpeedEmergency(const VType& t, double speed) {
    if (t.rail) {
        return minNextSpeed(t, speed);
    }
    return BALLISTIC ? speed - ACCEL2SPEED(t.emergencyDecel) : std::max(speed - ACCEL2SPEED(t.emergencyDecel), 0.);
}

inline double getSpeedAfterMaxDecel(const VType& t, double v) {
    return std::max(0., v - ACCEL2SPEED(t.decel));
}

inline double brakeGap(double speed, double decel, double headwayTime) {
    if (BALLISTIC) {
        return speed <= 0 ? 0. : speed * (headwayTime + 0.5 * speed / decel);
    }
    const double speedReduction = ACCEL2SPEED(decel);
    const int steps = int(speed / speedReduction);
    return SPEED2DIST(steps * speed - speedReduction * steps * (steps + 1) / 2) + speed * headwayTime;
}

inline double brakeGap(const VType& t, double speed) {
    return brakeGap(speed, t.decel, t.tau);
}

inline double maximumSafeStopSpeedEuler(const VType& t, double gap, double decel, double headway) {
    const double g = gap - NUMERICAL_EPS;
    if (g < 0.) {
        return 0.;
    }
    const double b = ACCEL2SPEED(decel);
    const double tt = headway >= 0 ? headway : t.tau;
    const double s = TS;
    const double n = std::floor(.5 - ((tt + (std::sqrt(((s * s) + (4.0 * ((s * (2.0 * g / b - tt)) + (tt * tt))))) * -0.5)) / s));
    const double h = 0.5 * n * (n - 1) * b * s + n * b * tt;
    const double r = (g - h) / (n * s + tt);
    const double x = n * b + r;
    return x;
}

inline double calculateEmergencyDeceleration(const VType& t, double gap, double egoSpeed, double predSpeed, double predMaxDecel) {
    if (gap <= 0.) {
        return t.emergencyDecel;
    }
    const double predBrakeDist = 0.5 * predSpeed * predSpeed / predMaxDecel;
    const double b1 = 0.5 * egoSpeed * egoSpeed / (gap + predBrakeDist);
    if (b1 <= predMaxDecel) {
        return b1;
    }
    const double b2 = 0.5 * (egoSpeed * egoSpeed - predSpeed * predSpeed) / gap;
    return b2;
}

// MSCFModel::maximumSafeStopSpeedBallistic
inline double maximumSafeStopSpeedBallistic(const VType& t, double gap, double decel, double currentSpeed, bool onInsertion,
                                            double headway) {
    const double g = std::max(0., gap - NUMERICAL_EPS);
    headway = headway >= 0 ? headway : t.tau;
    if (onInsertion) {
        // no distance is covered until the next step
        const double btau = decel * headway;
        return -btau + std::sqrt(btau * btau + 2 * decel * g);
    }
    const double tau = headway == 0 ? TS : headway;
    const double v0 = std::max(0., currentSpeed);
    if (g <= v0 * tau * 0.5) {
        // a stop within tau
        if (g == 0.) {
            return v0 > 0. ? -ACCEL2SPEED(t.emergencyDecel) : 0.;
        }
        const double a = -v0 * v0 / (2 * g);
        return v0 + a * TS;
    }
    const double btau2 = decel * tau / 2;
    const double v1 = -btau2 + std::sqrt(btau2 * btau2 + decel * (2 * g - tau * v0));
    const double a = (v1 - v0) / tau;
    return v0 + a * TS;
}

inline double maximumSafeStopSpeed(const VType& t, double gap, double decel, double currentSpeed, bool onInsertion,
                                   double headway, bool relaxEmergency) {
    double vsafe = BALLISTIC ? maximumSafeStopSpeedBallistic(t, gap, decel, currentSpeed, onInsertion, headway)
                   : maximumSafeStopSpeedEuler(t, gap, decel, headway);
    if (relaxEmergency && t.decel != t.emergencyDecel) {
        const double origSafeDecel = SPEED2ACCEL(currentSpeed - vsafe);
        if (origSafeDecel > t.decel + NUMERICAL_EPS) {
            double safeDecel = EMERGENCY_DECEL_AMPLIFIER * calculateEmergencyDeceleration(t, gap, currentSpeed, 0., 1);
            safeDecel = std::max(safeDecel, t.decel);
            safeDecel = std::min(safeDecel, origSafeDecel);
            vsafe = currentSpeed - ACCEL2SPEED(safeDecel);
            if (!BALLISTIC) {
                vsafe = std::max(vsafe, 0.);
            }
        }
    }
    return vsafe;
}

// computeLC mirrors MSGlobals::gComputeLC (true while lanes are being changed)
inline double maximumSafeFollowSpeed(const VType& t, double gap, double egoSpeed, double predSpeed, double predMaxDecel,
                                     bool onInsertion, bool computeLC) {
    const double headway = t.tau;
    double x;
    if (gap >= 0 || computeLC) {
        x = maximumSafeStopSpeed(t, gap + brakeGap(predSpeed, std::max(t.decel, predMaxDecel), 0), t.decel, egoSpeed, onInsertion, headway, false);
    } else {
        x = egoSpeed - ACCEL2SPEED(t.emergencyDecel);
        if (!BALLISTIC) {
            x = std::max(x, 0.);
        }
    }
    if (t.decel != t.emergencyDecel && !onInsertion && !computeLC) {
        const double origSafeDecel = SPEED2ACCEL(egoSpeed - x);
        if (origSafeDecel > t.decel + NUMERICAL_EPS) {
            double safeDecel = EMERGENCY_DECEL_AMPLIFIER * calculateEmergencyDeceleration(t, gap, egoSpeed, predSpeed, predMaxDecel);
            safeDecel = std::max(safeDecel, t.decel);
            safeDecel = std::min(safeDecel, origSafeDecel);
            x = egoSpeed - ACCEL2SPEED(safeDecel);
            if (!BALLISTIC) {
                x = std::max(x, 0.);
            }
        }
    }
    return x;
}

// MSCFModel_Krauss::followSpeed (Euler)
inline double followSpeed(const VType& t, double speed, double gap, double predSpeed, double predMaxDecel, bool computeLC) {
    if (t.rail) {
        // MSCFModel_Rail::followSpeed (moving block): a safety distance similar to CIR-ELKE (LZB) from 30 km/h
        if (speed >= 30 / 3.6) {
            gap = std::max(0.0, gap + t.minGap - 50);
        }
        const double vsafe = maximumSafeStopSpeed(t, gap, t.decel, speed, false, TS, false);
        return std::min(vsafe, maxNextSpeed(t, speed));
    }
    const double vsafe = maximumSafeFollowSpeed(t, gap, speed, predSpeed, predMaxDecel, false, computeLC);
    const double vmax = maxNextSpeed(t, speed);
    if (BALLISTIC) {
        // (the Euler variant can brake as hard as it wishes immediately)
        return std::max(std::min(vsafe, vmax), minNextSpeedEmergency(t, speed));
    }
    return std::min(vsafe, vmax);
}

enum class CalcReason { CURRENT, FUTURE, CURRENT_WAIT, LANE_CHANGE };

// MSCFModel_Krauss::stopSpeed (Euler, action step == TS)
inline double stopSpeed(const VType& t, double speed, double gap, double decel, CalcReason usage = CalcReason::CURRENT) {
    if (t.rail) {
        // MSCFModel_Rail::stopSpeed
        return std::min(maximumSafeStopSpeed(t, gap, decel, speed, false, TS, false), maxNextSpeed(t, speed));
    }
    const bool relaxEmergency = usage != CalcReason::FUTURE;
    return std::min(maximumSafeStopSpeed(t, gap, decel, speed, false, TS, relaxEmergency), maxNextSpeed(t, speed));
}

inline double stopSpeed(const VType& t, double speed, double gap, CalcReason usage = CalcReason::CURRENT) {
    return stopSpeed(t, speed, gap, t.decel, usage);
}

inline double freeSpeed(double currentSpeed, double decel, double dist, double targetSpeed, bool onInsertion,
                        double actionStepLength = -1) {
    if (BALLISTIC) {
        // the maximum next speed vN that can be adjusted to targetSpeed after dist with the deceleration
        const double asl = actionStepLength < 0 ? TS : actionStepLength;
        const double dt = onInsertion ? 0 : asl;
        const double v0 = currentSpeed;
        const double vT = targetSpeed;
        const double b = decel;
        const double d = dist - NUMERICAL_EPS;
        if (0.5 * (v0 + vT) * dt >= d) {
            return v0 + TS * (vT - v0) / asl;  // attain vT after the action step
        }
        const double q = (dt * v0 - 2 * d) * b - vT * vT;
        const double p = 0.5 * b * dt;
        const double vN = -p + std::sqrt(p * p - q);
        return v0 + TS * (vN - v0) / asl;
    }
    const double v = SPEED2DIST(targetSpeed);
    if (dist < v) {
        return targetSpeed;
    }
    const double b = ACCEL2DIST(decel);
    const double y = std::max(0.0, ((std::sqrt((b + 2.0 * v) * (b + 2.0 * v) + 8.0 * b * dist) - b) * 0.5 - v) / b);
    const double yFull = std::floor(y);
    const double exactGap = (yFull * yFull + yFull) * 0.5 * b + yFull * v + (y > yFull ? v : 0.0);
    const double fullSpeedGain = (yFull + (onInsertion ? 1. : 0.)) * ACCEL2SPEED(decel);
    return DIST2SPEED(std::max(0.0, dist - exactGap) / (yFull + 1)) + fullSpeedGain + targetSpeed;
}

inline double freeSpeed(const VType& t, double speed, double seen, double maxSpeed, bool onInsertion = false) {
    if (t.rail) {
        // MSCFModel_Rail::freeSpeed: the same braking scheme with the train's deceleration
        return freeSpeed(speed, t.rail->decl, seen, maxSpeed, onInsertion);
    }
    if (maxSpeed < 0.) {
        return maxSpeed;
    }
    return freeSpeed(speed, t.decel, seen, maxSpeed, onInsertion);
}

// MSCFModel::getSecureGap. egoAccel/safetyFactor are only used while computing lane changes
inline double getSecureGap(const VType& t, double speed, double leaderSpeed, double leaderMaxDecel,
                           bool computeLC = false, double egoAccel = 0, double lcSafetyFactor = 1) {
    const double maxDecel = std::max(t.decel, leaderMaxDecel);
    const double bgLeader = brakeGap(leaderSpeed, maxDecel, 0);
    double secureGap = std::max(0.0, brakeGap(speed, t.decel, t.tau) - bgLeader);
    if (computeLC && egoAccel < -NUMERICAL_EPS) {
        const double secureGapDecel = std::max(0.0, brakeGap(speed, std::min(-egoAccel, t.decel), 0) - bgLeader);
        secureGap = std::min(secureGap, secureGapDecel / lcSafetyFactor);
    }
    return secureGap;
}

inline double insertionFollowSpeed(const VType& t, double speed, double gap2pred, double predSpeed, double predMaxDecel) {
    // (ballistic: the current speed is irrelevant at insertion)
    return maximumSafeFollowSpeed(t, gap2pred, BALLISTIC ? 0. : speed, predSpeed, predMaxDecel, true, false);
}

inline double insertionStopSpeed(const VType& t, double speed, double gap) {
    if (BALLISTIC) {
        return std::min(maximumSafeStopSpeed(t, gap, t.decel, 0., true, 0., false), t.maxSpeed);
    }
    return stopSpeed(t, speed, gap, CalcReason::FUTURE);
}

// MSCFModel::getMinimalArrivalSpeed (ballistic)
inline double getMinimalArrivalSpeed(const VType& t, double dist, double currentSpeed);

inline double estimateSpeedAfterDistance(const VType& t, double dist, double v, double accel) {
    return std::min(t.maxSpeed, (double)std::sqrt(std::max(0., 2 * dist * accel + v * v)));
}

// MSCFModel::getMinimalArrivalTime
inline Time getMinimalArrivalTime(const VType& t, double dist, double currentSpeed, double arrivalSpeed) {
    if (dist <= 0.) {
        return 0;
    }
    arrivalSpeed = std::max(0.0, arrivalSpeed);
    const double accel = (arrivalSpeed >= currentSpeed) ? t.accel : -t.decel;
    const double accelTime = accel == 0. ? 0. : (arrivalSpeed - currentSpeed) / accel;
    const double accelWay = accelTime * (arrivalSpeed + currentSpeed) * 0.5;
    if (dist >= accelWay) {
        const double nonAccelWay = dist - accelWay;
        const double nonAccelSpeed = std::max(std::max(currentSpeed, arrivalSpeed), HALTING_SPEED);
        return TIME2STEPS(accelTime + nonAccelWay / nonAccelSpeed);
    }
    return TIME2STEPS(-(currentSpeed - std::sqrt(currentSpeed * currentSpeed + 2 * accel * dist)) / accel);
}

// MSCFModel::getMinimalArrivalSpeedEuler
inline double getMinimalArrivalSpeedEuler(const VType& t, double dist, double currentSpeed) {
    if (dist < currentSpeed) {
        return INVALID_SPEED;
    } else if (2 * (dist - currentSpeed * t.tau) * -t.decel + currentSpeed * currentSpeed >= 0) {
        return estimateSpeedAfterDistance(t, dist - currentSpeed * t.tau, currentSpeed, -t.decel);
    }
    return t.decel;
}

// MSCFModel_KraussOrig1::vsafe (also Daniel1::_vsafe)
inline double kraussOrig1Vsafe(const VType& t, double tauDecel, double gap, double predSpeed) {
    if (predSpeed == 0 && gap < 0.01) {
        return 0;
    } else if (predSpeed == 0 && gap <= ACCEL2SPEED(t.decel)) {
        return std::min(ACCEL2SPEED(t.decel), DIST2SPEED(gap));  // workaround for #2310
    }
    return -1. * tauDecel + std::sqrt(tauDecel * tauDecel + predSpeed * predSpeed + 2. * t.decel * gap);
}

// MSCFModel::gapExtrapolation: the gap after duration with constant accelerations (bounded speeds)
inline double gapExtrapolation(double duration, double currentGap, double v1, double v2, double a1, double a2, double maxV1,
                               double maxV2) {
    double newGap = currentGap;
    if (!BALLISTIC) {
        for (unsigned int steps = 1; steps * TS <= duration; ++steps) {
            v1 = std::min(std::max(v1 + a1, 0.), maxV1);
            v2 = std::min(std::max(v2 + a2, 0.), maxV2);
            newGap += TS * (v1 - v2);
        }
        return newGap;
    }
    // the times at which the vehicles stop (t1, t2) or reach their maximum speed (t3, t4)
    double t1 = 0, t2 = 0, t3 = 0, t4 = 0;
    if (a1 < 0 && v1 > 0) {
        t1 = std::min(-v1 / a1, duration);
    } else if (a1 >= 0) {
        t1 = duration;
    }
    if (a2 < 0 && v2 > 0) {
        t2 = std::min(-v2 / a2, duration);
    } else if (a2 >= 0) {
        t2 = duration;
    }
    if (a1 > 0 && v1 < maxV1) {
        t3 = std::min((maxV1 - v1) / a1, duration);
    } else if (a1 <= 0) {
        t3 = duration;
    }
    if (a2 > 0 && v2 < maxV2) {
        t4 = std::min((maxV2 - v2) / a2, duration);
    } else if (a2 <= 0) {
        t4 = duration;
    }
    double l[4] = {t1, t2, t3, t4};
    std::sort(l, l + 4);
    double tLast = 0.;
    for (double ti : l) {
        if (ti != tLast) {
            const double dt = std::min(ti, duration) - tLast;
            const double dv = v1 - v2;
            const double da = a1 - a2;
            newGap += dv * dt + da * dt * dt / 2.;
            v1 += dt * a1;
            v2 += dt * a2;
        }
        if (ti == t1 || ti == t3) {
            a1 = 0.;
        }
        if (ti == t2 || ti == t4) {
            a2 = 0.;
        }
        tLast = std::min(ti, duration);
        if (tLast == duration) {
            break;
        }
    }
    if (duration != tLast) {
        newGap += (v1 - v2) * (duration - tLast);  // (both vehicles have zero acceleration)
    }
    return newGap;
}

// MSCFModel::getMinimalArrivalSpeed (ballistic)
inline double getMinimalArrivalSpeed(const VType& t, double dist, double currentSpeed) {
    return estimateSpeedAfterDistance(t, dist - currentSpeed * t.tau, currentSpeed, -t.decel);
}

// MSCFModel_Krauss::dawdle2
inline double dawdle2(const VType& t, double speed, double sigma, double random) {
    if (BALLISTIC && speed < 0) {
        return speed;  // (a desired stop within the next step)
    }
    if (speed < t.accel) {
        speed -= ACCEL2SPEED(sigma * speed * random);
    } else {
        speed -= ACCEL2SPEED(sigma * t.accel * random);
    }
    return std::max(0., speed);
}

}  // namespace cf
}  // namespace ps
