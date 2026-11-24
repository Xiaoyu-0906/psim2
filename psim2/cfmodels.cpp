// Car-following models: the virtual methods of SUMO 1.28's MSCFModel and its subclasses (microsim/cfmodels),
// dispatched by the vehicle's type. The Krauss and Rail formulas live in cf.hpp, the other models here.
// A model reads the vehicle like SUMO's MSCFModel methods read their MSVehicle argument: its type, speed factor,
// the speed limit of its lane (lane: the vehicle's lane unless the caller passes the lane it sees, e.g. a
// neighbour in the lane-changing phase) and its model state (Sim::cfVars, only written by the vehicle's own task).
// Random numbers are psim2's counter-based draws (vehicle, step, call), not SUMO's per-vehicle generator.
#include <cmath>
#include <sstream>

#include "model.hpp"

namespace ps {

thread_local int cfOwner = -1;
thread_local bool cfComputeLC = false;
thread_local int lcOwnEdge = -1;

namespace {
CFM cfModelByName(const std::string& name) {
    static const std::map<std::string, CFM> names = {
        {"Krauss", CFM::KRAUSS}, {"KraussOrig1", CFM::KRAUSS_ORIG1}, {"KraussPS", CFM::KRAUSS_PS},
        {"KraussX", CFM::KRAUSSX}, {"SmartSK", CFM::SMART_SK}, {"Daniel1", CFM::DANIEL1},
        {"PWagner2009", CFM::PWAGNER2009}, {"IDM", CFM::IDM}, {"IDMM", CFM::IDMM}, {"EIDM", CFM::EIDM},
        {"BKerner", CFM::BKERNER}, {"Wiedemann", CFM::WIEDEMANN}, {"W99", CFM::W99}, {"Rail", CFM::RAIL},
        {"ACC", CFM::ACC}, {"CACC", CFM::CACC}, {"CC", CFM::CC}, {"NaSch", CFM::NASCH}};
    auto it = names.find(name);
    if (it == names.end()) {
        throw std::runtime_error("unknown car-following model " + name);
    }
    return it->second;
}

// VType::cfp indices of the models
enum KraussP { K_DAWDLE_STEP, K_SIGMA_MINOR, K_TAU_DECEL, K_TMP1, K_TMP2, K_N };   // Krauss family
enum IdmP { IDM_DELTA, IDM_ADAPT_FACTOR, IDM_ADAPT_TIME, IDM_ITER, IDM_TWO_SQRT, IDM_N };
enum WiedP { WD_SECURITY, WD_ESTIMATION, WD_AX, WD_CX, WD_MIN_ACCEL, WD_MAX_APPROACH_DECEL, WD_N };
enum KernerP { BK_K, BK_PHI, BK_TAU_DECEL, BK_N };
enum PWagP { PW_TAU_DECEL, PW_DECEL_DIV_TAU, PW_TAU_LAST_DECEL, PW_AP_PROB, PW_N };
enum SmartSKP { SK_TAU_DECEL, SK_TMP2, SK_TMP3, SK_S2S, SK_MAX_DELTA_GAP, SK_N };
enum NaSchP { NS_CELL_SPEED, NS_N };
// ACC (also the ACC block of CACC, at offset 0), then the CACC block
enum AccP { ACC_SC, ACC_GCC_SPEED, ACC_GCC_SPACE, ACC_GC_SPEED, ACC_GC_SPACE, ACC_CA_SPEED, ACC_CA_SPACE, ACC_EMERGENCY,
            ACC_HEADWAY, ACC_N };
enum EidmP { E_DELTA, E_ITER, E_TPERS_DRIVE, E_TREACTION, E_TPREVIEW, E_TPERS_ESTIMATE, E_CCOOLNESS, E_SIG_LEADER,
             E_SIG_GAP, E_SIG_ERROR, E_JERK_MAX, E_EPSILON_ACC, E_TACC_MAX, E_M_FLATNESS, E_M_BEGIN, E_N };
constexpr double EIDM_EST_REAC_THRESHOLD = 3.;
constexpr double EIDM_CLUTCH_ENGAGE_SPEED = 0.5;
constexpr double EIDM_POS_ACC_EPS = 0.05;
enum CaccP { C_SC = ACC_N, C_GCC_GAP, C_GCC_GAP_DOT, C_GC_GAP, C_GC_GAP_DOT, C_CA_GAP, C_CA_GAP_DOT, C_EMERGENCY, C_SC_MIN_GAP,
             C_N };

constexpr double W99_INTERACTION_GAP = 150;
constexpr double WIEDEMANN_D_MAX = 150;
constexpr double GRAVITY = 9.80665;

bool isKraussFamily(CFM m) {
    return m == CFM::KRAUSS || m == CFM::KRAUSS_ORIG1 || m == CFM::KRAUSS_PS || m == CFM::KRAUSSX;
}
}  // namespace

// CFTYPE type model key=value ... (the model and its own attributes)
void Sim::setCarFollowModel(VType& t, const std::vector<std::string>& tok) {
    t.cfm = cfModelByName(tok[2]);
    for (size_t i = 3; i < tok.size(); ++i) {
        const size_t eq = tok[i].find('=');
        std::string value = tok[i].substr(eq + 1);
        std::replace(value.begin(), value.end(), ';', ' ');
        t.cfAttrs[tok[i].substr(0, eq)] = value;
    }
    initCarFollowModel(t);
}

double Sim::cfParam(const VType& t, const char* key, double deflt) const {
    auto it = t.cfAttrs.find(key);
    return it == t.cfAttrs.end() ? deflt : std::stod(it->second);
}

// the model constructors (parameters with the model's defaults)
void Sim::initCarFollowModel(VType& t) {
    t.cfp.clear();
    t.startupDelay = cfParam(t, "startupDelay", 0);
    // SUMOVTypeParameter::getCFProfile: the values per speed of the speedTable
    auto table = [&](const char* key) {
        std::vector<double> r;
        auto it = t.cfAttrs.find(key);
        if (it != t.cfAttrs.end()) {
            std::istringstream ss(it->second);
            double d;
            while (ss >> d) {
                r.push_back(d);
            }
        }
        return r;
    };
    const std::vector<double> speeds = table("speedTable");
    for (const char* key : {"maxAccelProfile", "desAccelProfile"}) {
        const std::vector<double> values = table(key);
        if (!values.empty()) {
            if (values.size() != speeds.size()) {
                throw std::runtime_error(std::string("mismatching size of speedTable and ") + key + " of vType " + t.id);
            }
            std::map<double, double>& profile = key[0] == 'm' ? t.maxAccelProfile : t.desAccelProfile;
            profile.clear();
            for (size_t i = 0; i < speeds.size(); ++i) {
                profile[speeds[i]] = values[i];
            }
        }
    }
    // the imprecise models may violate minGap on occasion
    const bool imprecise = t.cfm != CFM::RAIL && !isKraussFamily(t.cfm) && t.cfm != CFM::DANIEL1 && t.cfm != CFM::SMART_SK
                           && t.cfm != CFM::NASCH;
    t.collisionMinGapFactor = cfParam(t, "collisionMinGapFactor", imprecise ? 0.1 : 1);
    std::vector<double>& p = t.cfp;
    switch (t.cfm) {
        case CFM::RAIL:
            break;
        case CFM::KRAUSS:
        case CFM::KRAUSS_ORIG1:
        case CFM::KRAUSS_PS:
        case CFM::KRAUSSX: {
            p.resize(K_N);
            // MSCFModel_Krauss: sigmaStep rounded to whole steps
            Time step = TIME2STEPS(cfParam(t, "sigmaStep", TS));
            if (step % DELTA_T != 0) {
                const Time rem = step % DELTA_T;
                step += rem < DELTA_T / 2 ? -rem : DELTA_T - rem;
            }
            p[K_DAWDLE_STEP] = (double)step;
            p[K_SIGMA_MINOR] = cfParam(t, "jmSigmaMinor", t.sigma);
            p[K_TAU_DECEL] = t.decel * t.tau;
            p[K_TMP1] = cfParam(t, "tmp1", 0);
            p[K_TMP2] = cfParam(t, "tmp2", 0);
            break;
        }
        case CFM::IDM:
        case CFM::IDMM: {
            const bool idmm = t.cfm == CFM::IDMM;
            p.resize(IDM_N);
            p[IDM_DELTA] = idmm ? 4.0 : cfParam(t, "delta", 4.);
            p[IDM_ADAPT_FACTOR] = idmm ? cfParam(t, "adaptFactor", 1.8) : 1.0;
            p[IDM_ADAPT_TIME] = idmm ? cfParam(t, "adaptTime", 600.0) : 0.0;
            p[IDM_ITER] = std::max(1, int(TS / cfParam(t, "stepping", .25) + .5));
            p[IDM_TWO_SQRT] = 2 * std::sqrt(t.accel * t.decel);
            break;
        }
        case CFM::W99:
            p.resize(9);
            p[0] = cfParam(t, "cc1", 1.30);
            p[1] = cfParam(t, "cc2", 8.00);
            p[2] = cfParam(t, "cc3", -12.00);
            p[3] = cfParam(t, "cc4", -0.25);
            p[4] = cfParam(t, "cc5", 0.35);
            p[5] = cfParam(t, "cc6", 6.00);
            p[6] = cfParam(t, "cc7", 0.25);
            p[7] = cfParam(t, "cc8", 2.00);
            p[8] = cfParam(t, "cc9", 1.50);
            // (the model's headway time and acceleration)
            t.tau = p[0];
            t.accel = p[7];
            break;
        case CFM::WIEDEMANN:
            p.resize(WD_N);
            p[WD_SECURITY] = cfParam(t, "security", 0.5);
            p[WD_ESTIMATION] = cfParam(t, "estimation", 0.5);
            p[WD_AX] = t.length + 1. + 2. * p[WD_SECURITY];
            p[WD_CX] = 25. * (1. + p[WD_SECURITY] + p[WD_ESTIMATION]);
            p[WD_MIN_ACCEL] = 0.2 * t.accel;
            p[WD_MAX_APPROACH_DECEL] = (t.decel + t.emergencyDecel) / 2;
            break;
        case CFM::BKERNER:
            p.resize(BK_N);
            p[BK_K] = cfParam(t, "k", 0.5);
            p[BK_PHI] = cfParam(t, "phi", 5.0);
            p[BK_TAU_DECEL] = t.decel * t.tau;
            break;
        case CFM::PWAGNER2009:
            p.resize(PW_N);
            p[PW_TAU_DECEL] = t.decel * t.tau;
            p[PW_DECEL_DIV_TAU] = t.decel / t.tau;
            p[PW_TAU_LAST_DECEL] = t.decel * cfParam(t, "tauLast", 0.3);
            p[PW_AP_PROB] = cfParam(t, "apProb", 0.5);
            break;
        case CFM::SMART_SK: {
            p.resize(SK_N);
            p[SK_TAU_DECEL] = t.decel * t.tau;
            const double tmp1 = cfParam(t, "tmp1", 1.0);
            double s2s = -p[SK_TAU_DECEL] + std::sqrt(p[SK_TAU_DECEL] * p[SK_TAU_DECEL] + t.accel * (t.accel + t.decel) * tmp1 * tmp1
                                                       + t.accel * t.decel * tmp1 * TS);
            p[SK_S2S] = std::min(s2s, 5.0);
            p[SK_MAX_DELTA_GAP] = -0.5 * (t.decel + t.accel) * TS * TS;
            p[SK_TMP2] = TS / cfParam(t, "tmp2", 1.0);
            p[SK_TMP3] = std::sqrt(TS) * cfParam(t, "tmp3", 1.0);
            break;
        }
        case CFM::DANIEL1:
            p.resize(1);
            p[0] = t.decel * t.tau;
            break;
        case CFM::NASCH:
            p.resize(NS_N);
            p[NS_CELL_SPEED] = ACCEL2SPEED(t.accel);
            break;
        case CFM::EIDM:
            p.resize(E_N);
            p[E_DELTA] = cfParam(t, "delta", 4.);
            p[E_ITER] = std::max(1, int(TS / cfParam(t, "stepping", .25) + .5));
            p[E_TPERS_DRIVE] = cfParam(t, "tPersDrive", 3);
            p[E_TREACTION] = cfParam(t, "treaction", 0.5);
            p[E_TPREVIEW] = cfParam(t, "tpreview", 4);
            p[E_TPERS_ESTIMATE] = cfParam(t, "tPersEstimate", 10);
            p[E_CCOOLNESS] = cfParam(t, "ccoolness", 0.99);
            p[E_SIG_LEADER] = cfParam(t, "sigmaleader", 0.02);
            p[E_SIG_GAP] = cfParam(t, "sigmagap", 0.1);
            p[E_SIG_ERROR] = cfParam(t, "sigmaerror", 0.04);
            p[E_JERK_MAX] = cfParam(t, "jerkmax", 3.);
            p[E_EPSILON_ACC] = cfParam(t, "epsilonacc", 1.);
            p[E_TACC_MAX] = cfParam(t, "taccmax", 1.2);
            p[E_M_FLATNESS] = cfParam(t, "Mflatness", 2.);
            p[E_M_BEGIN] = cfParam(t, "Mbegin", 0.7);
            break;
        case CFM::ACC:
        case CFM::CACC:
            p.resize(t.cfm == CFM::ACC ? int{ACC_N} : int{C_N});
            p[ACC_SC] = cfParam(t, "speedControlGain", -0.4);
            p[ACC_GCC_SPEED] = cfParam(t, "gapClosingControlGainSpeed", 0.8);
            p[ACC_GCC_SPACE] = cfParam(t, "gapClosingControlGainSpace", 0.04);
            p[ACC_GC_SPEED] = cfParam(t, "gapControlGainSpeed", 0.07);
            p[ACC_GC_SPACE] = cfParam(t, "gapControlGainSpace", 0.23);
            p[ACC_CA_SPEED] = cfParam(t, "collisionAvoidanceGainSpeed", 0.23);
            p[ACC_CA_SPACE] = cfParam(t, "collisionAvoidanceGainSpace", 0.8);
            p[ACC_EMERGENCY] = cfParam(t, "collisionAvoidanceOverride", 2.0);
            // (the ACC model inside CACC: setHeadwayTime(tauCACCToACC))
            p[ACC_HEADWAY] = t.cfm == CFM::ACC ? t.tau : cfParam(t, "tauCACCToACC", 1.0);
            if (t.cfm == CFM::CACC) {
                p[C_SC] = cfParam(t, "speedControlGainCACC", -0.4);
                p[C_GCC_GAP] = cfParam(t, "gapClosingControlGainGap", 0.005);
                p[C_GCC_GAP_DOT] = cfParam(t, "gapClosingControlGainGapDot", 0.05);
                p[C_GC_GAP] = cfParam(t, "gapControlGainGap", 0.45);
                p[C_GC_GAP_DOT] = cfParam(t, "gapControlGainGapDot", 0.0125);
                p[C_CA_GAP] = cfParam(t, "collisionAvoidanceGainGap", 0.45);
                p[C_CA_GAP_DOT] = cfParam(t, "collisionAvoidanceGainGapDot", 0.05);
                p[C_EMERGENCY] = cfParam(t, "collisionAvoidanceOverride", 2.0);
                p[C_SC_MIN_GAP] = cfParam(t, "speedControlMinGap", 1.66);
            }
            break;
        default:
            throw std::runtime_error("car-following model of type " + t.id + " not supported yet");
    }
}

bool Sim::cfNeedsVars(const VType& t) const {
    switch (t.cfm) {
        case CFM::KRAUSS:
        case CFM::KRAUSS_PS:
            return (Time)t.cfp[K_DAWDLE_STEP] > DELTA_T;
        case CFM::IDMM:
        case CFM::WIEDEMANN:
        case CFM::BKERNER:
        case CFM::PWAGNER2009:
        case CFM::SMART_SK:
        case CFM::ACC:
        case CFM::CACC:
        case CFM::EIDM:
            return true;
        case CFM::RAIL:
            return t.rail->curveResistance > 0;
        default:
            return false;
    }
}

// MSCFModel::createVehicleVariables, MSVehicle::myTimeSinceStartup
void Sim::initCarFollowVars(int v) {
    if (!cfHaveVars) {
        return;
    }
    CFVars& c = cfVars[v];
    c = CFVars();
    const VType& t = vt(v);
    if (isKraussFamily(t.cfm)) {
        // (at loading: SUMO builds the vehicle when its route is read)
        const Time dawdleStep = (Time)t.cfp[K_DAWDLE_STEP];
        c.updateOffset = std::max<Time>(0, veh[v].depart) % dawdleStep + DELTA_T;
    } else if (t.cfm == CFM::SMART_SK) {
        c.headway = t.tau;
    } else if (t.cfm == CFM::BKERNER) {
        c.rnd = rand01(opt.seed, (uint64_t)v, 0, RP_CF_STATE);
    }
}

bool Sim::passingMinor(int v) const {
    const int lane = kin[v].lane;
    if (lane < 0) {
        return false;
    }
    if (isInternal(lane)) {
        const IncomingLane& in = net.lanes[lane].incoming.front();
        return !net.links[in.link].havePriority();
    }
    const Veh& x = veh[v];
    if (!x.lf.empty() && x.lf.front().link >= 0) {
        const Link& l = net.links[x.lf.front().link];
        return !l.havePriority() && x.lf.front().distance <= l.visibility;
    }
    return false;
}

// MSCFModel_Rail::getCurveResistance (kN/t) with RailVehicleVariables::getIntegratedRadius: the radius over the
// (odometer, angle) points of the last curveIntegration m. The vehicle's own calls record a point when the odometer
// changed; the calls of others see the result of recording it without changing the history
double Sim::railCurveResistance(int v) const {
    const RailCF& r = *vt(v).rail;
    if (r.curveResistance <= 0) {
        return 0;
    }
    std::vector<std::pair<double, double>>& h = cfv(v)->odometerAngles;
    const double odo = vehOdometer(v);
    const double inf = std::numeric_limits<double>::max();
    double radius = inf;
    std::pair<double, double> back, front;
    bool haveRadius = false;
    if ((h.empty() || h.back().first != odo) && veh[v].departTime >= 0) {
        const std::pair<double, double> point(odo, vehAngle(v));
        if (cfOwner == v) {
            h.push_back(point);
            while (h.size() > 2 && h.back().first - h[1].first >= r.curveIntegration) {
                h.erase(h.begin());
            }
        } else if (!h.empty()) {
            size_t f = 0;
            while (h.size() - f > 1 && f + 1 < h.size() && point.first - h[f + 1].first >= r.curveIntegration) {
                ++f;
            }
            back = point;
            front = h[f];
            haveRadius = true;
        }
    }
    if (!haveRadius && h.size() > 1) {
        back = h.back();
        front = h.front();
        haveRadius = true;
    }
    if (haveRadius) {
        const double angleDiff = geom::angleDiff(back.second, front.second);
        radius = angleDiff == 0 ? inf : (back.first - front.first) / std::fabs(angleDiff);
    }
    // (with fewer points MSVehicle::getCurveRadius: the vehicle did not move since its departure)
    if (radius == inf) {
        return 0;
    } else if (radius >= r.roecklSharpRadius) {
        return 0.001 * r.curveResistance * r.roecklNumerator / (radius - r.roecklOffset);
    } else if (radius > r.roecklOffsetSharp) {
        return 0.001 * r.curveResistance * r.roecklNumeratorSharp / (radius - r.roecklOffsetSharp);
    }
    return 0;  // (SUMO warns: cannot compute the curve resistance)
}

// MSVehicle::getSlope: from the back to the front position
double Sim::cfSlope(int v) const {
    const Kin& k = kin[v];
    if (!net.haveElevation || k.lane < 0 || veh[v].status != VehStatus::RUNNING) {
        return 0;
    }
    auto point = [&](int lane, double pos, double& z, Pos& xy) {
        const Lane& L = net.lanes[lane];
        const double g = L.interpolateLanePosToGeometryPos(pos);
        if (L.z.empty()) {
            z = 0;
            xy = geom::positionAtOffset2D(L.shape, g);
        } else {
            double off2D;
            L.elevationAtOffset(g, z, off2D);
            xy = geom::positionAtOffset2D(L.shape, off2D);
        }
    };
    double z1, z2;
    Pos p1, p2;
    point(k.lane, k.pos, z1, p1);
    const double backOnLane = k.pos - vt(v).length;
    if (backOnLane >= 0 || k.nFurther == 0) {
        point(k.lane, std::max(0., backOnLane), z2, p2);
    } else {
        point(k.further[k.nFurther - 1], std::max(0., k.backPos), z2, p2);
    }
    const double d = p1.distanceTo2D(p2);
    if (d == 0 && z1 == z2) {
        return 0;
    }
    return std::atan2(z1 - z2, d) * 180. / M_PI;
}

// ---------------------------------------------------------------------------
// IDM / IDMM (MSCFModel_IDM)
// ---------------------------------------------------------------------------
namespace {
double idmV(const VType& t, const CFVars* vars, double gap2pred, double egoSpeed, double predSpeed, double desSpeed,
            bool respectMinGap = true) {
    const std::vector<double>& p = t.cfp;
    double headwayTime = t.tau;
    if (p[IDM_ADAPT_FACTOR] != 1. && vars != nullptr) {
        headwayTime *= p[IDM_ADAPT_FACTOR] + vars->levelOfService * (1. - p[IDM_ADAPT_FACTOR]);
    }
    double newSpeed = egoSpeed;
    double gap = gap2pred;
    if (respectMinGap) {
        gap += t.minGap;  // gap2pred comes with minGap already subtracted
    }
    const int iterations = (int)p[IDM_ITER];
    for (int i = 0; i < iterations; i++) {
        const double deltaV = newSpeed - predSpeed;
        double s = std::max(0., newSpeed * headwayTime + newSpeed * deltaV / p[IDM_TWO_SQRT]);
        if (respectMinGap) {
            s += t.minGap;
        }
        gap = std::max(NUMERICAL_EPS, gap);
        const double acc = t.accel * (1. - std::pow(newSpeed / std::max(NUMERICAL_EPS, desSpeed), p[IDM_DELTA]) - (s * s) / (gap * gap));
        newSpeed = std::max(0.0, newSpeed + ACCEL2SPEED(acc) / iterations);
        gap -= std::max(0., SPEED2DIST(newSpeed - predSpeed) / iterations);
    }
    return std::max(0., newSpeed);
}

double idmSecureGap(const VType& t, double speed, double leaderSpeed) {
    const double deltaV = speed - leaderSpeed;
    return std::max(0.0, speed * t.tau + speed * deltaV / t.cfp[IDM_TWO_SQRT]);
}

// MSCFModel_EIDM::maximumSafeFollowSpeed (computeLC: MSGlobals::gComputeLC)
double eidmMaximumSafeFollowSpeed(const VType& t, double gap, double egoSpeed, double predSpeed, double predMaxDecel,
                                  bool onInsertion, bool computeLC) {
    double x;
    if (gap >= 0 || computeLC) {
        const double a = cf::currentAccel(t, egoSpeed);
        const double b = t.tau * (2 * std::sqrt(a * t.decel)) - predSpeed;
        const double c = -std::sqrt(1 + t.decel / (2 * a)) * gap * (2 * std::sqrt(a * t.decel));
        x = (-b + std::sqrt(b * b - 4. * c)) / 2.;
    } else {
        x = egoSpeed - ACCEL2SPEED(t.emergencyDecel);
        if (!BALLISTIC) {
            x = std::max(x, 0.);
        }
    }
    if (t.decel != t.emergencyDecel && !onInsertion && !computeLC) {
        const double origSafeDecel = SPEED2ACCEL(egoSpeed - x);
        if (origSafeDecel > t.decel + NUMERICAL_EPS) {
            double safeDecel = EMERGENCY_DECEL_AMPLIFIER * cf::calculateEmergencyDeceleration(t, gap, egoSpeed, predSpeed, predMaxDecel);
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

// MSCFModel_EIDM::maximumSafeStopSpeed (Euler)
double eidmMaximumSafeStopSpeed(const VType& t, double gap, double decel, double currentSpeed, double headway) {
    if (BALLISTIC) {
        return cf::maximumSafeStopSpeedBallistic(t, gap, decel, currentSpeed, false, headway);
    }
    const double g = gap - NUMERICAL_EPS;
    if (g < 0) {
        return 0;
    }
    const double a = cf::currentAccel(t, currentSpeed);
    const double b = headway * (2 * std::sqrt(a * t.decel));
    const double c = -std::sqrt(1 + decel / (2 * a)) * g * (2 * std::sqrt(a * t.decel));
    return (-b + std::sqrt(b * b - 4. * c)) / 2.;
}
}  // namespace

// ---------------------------------------------------------------------------
// MSCFModel virtual methods
// ---------------------------------------------------------------------------
double Sim::cfFollowSpeed(int v, double speed, double gap, double predSpeed, double predMaxDecel, int pred,
                          cf::CalcReason usage, bool computeLC, int lane) const {
    return cfFollowSpeedT(vt(v), v, speed, gap, predSpeed, predMaxDecel, pred, usage, computeLC, lane);
}

// followSpeed with a temporarily changed maximum deceleration (MSCFModel::setMaxDecel, MSCFModel_KraussOrig1::setMaxDecel)
double Sim::cfFollowSpeedDecel(int v, double decel, double speed, double gap, double predSpeed, double predMaxDecel, int pred) const {
    thread_local VType tmp;
    tmp = vt(v);
    tmp.decel = decel;
    if (tmp.cfm == CFM::KRAUSS_ORIG1 || tmp.cfm == CFM::KRAUSS || tmp.cfm == CFM::KRAUSS_PS || tmp.cfm == CFM::KRAUSSX) {
        if ((int)tmp.cfp.size() > K_TAU_DECEL) {
            tmp.cfp[K_TAU_DECEL] = decel * tmp.tau;
        }
    }
    return cfFollowSpeedT(tmp, v, speed, gap, predSpeed, predMaxDecel, pred, cf::CalcReason::CURRENT, true, -1);
}

double Sim::cfFollowSpeedT(const VType& t, int v, double speed, double gap, double predSpeed, double predMaxDecel, int pred,
                           cf::CalcReason usage, bool computeLC, int lane) const {
    switch (t.cfm) {
        case CFM::KRAUSS:
        case CFM::KRAUSS_PS:
        case CFM::KRAUSSX:
            return cf::followSpeed(t, speed, gap, predSpeed, predMaxDecel, computeLC);
        case CFM::RAIL: {
            // MSCFModel_Rail::followSpeed (moving block): a safety distance similar to CIR-ELKE (LZB) from 30 km/h
            if (speed >= 30 / 3.6) {
                gap = std::max(0.0, gap + t.minGap - 50);
            }
            const double vsafe = cf::maximumSafeStopSpeed(t, gap, t.decel, speed, false, TS, false);
            const double vmax = cfMaxNextSpeed(v, speed);
            return BALLISTIC ? std::max(std::min(vsafe, vmax), cfMinNextSpeed(v, speed)) : std::min(vsafe, vmax);
        }
        case CFM::KRAUSS_ORIG1:
            if (BALLISTIC) {
                return std::max(std::min(cf::maximumSafeFollowSpeed(t, gap, speed, predSpeed, predMaxDecel, false, computeLC || cfComputeLC),
                                         cfMaxNextSpeed(v, speed)), cfMinNextSpeed(v, speed));
            }
            return std::min(cf::kraussOrig1Vsafe(t, t.cfp[K_TAU_DECEL], gap, predSpeed), cfMaxNextSpeed(v, speed));
        case CFM::IDM:
        case CFM::IDMM:
            return idmV(t, cfv(v), gap, speed, predSpeed, laneVehMaxSpeed(lane >= 0 ? lane : kin[v].lane, v));
        case CFM::W99: {
            const std::vector<double>& p = t.cfp;
            const double dx = gap + t.minGap;
            const double dv = predSpeed - speed;
            const double cc0 = t.minGap;
            const double leaderAccel = pred >= 0 ? kin[pred].accel : 0;
            const double oldAccel = kin[v].accel;
            const uint64_t salt = std::hash<double>()(gap) ^ std::hash<double>()(predSpeed);
            const double rndVal = speed > 0 ? cfRand(v, salt) - 0.5 : 0.5;
            // computeThresholds
            const double vSlower = (dv >= 0 || leaderAccel < 1) ? speed : predSpeed + dv * rndVal;
            double sdxc = cc0;
            if (predSpeed > 0) {
                sdxc += p[0] * std::max(0.0, vSlower);
            }
            const double sdxo = sdxc + p[1];
            const double sdxv = sdxo + p[2] * (dv - p[3]);
            const double sdv = p[5] * dx * dx / 10000;
            const double sdvc = speed > 0 ? p[3] - sdv : 0;
            const double sdvo = predSpeed > p[4] ? sdv + p[4] : sdv;
            double accel = 0;
            if (dv < sdvo && dx <= sdxc) {
                // decelerate - increase distance
                accel = 0;
                if (dx - SPEED2DIST(speed) < t.minGap * t.collisionMinGapFactor) {
                    accel = -SPEED2ACCEL(speed);  // prevent crashing in the next step
                }
                if (predSpeed > 0) {
                    if (dv < 0) {
                        if (dx > cc0) {
                            accel = std::min(leaderAccel + dv * dv / (cc0 - dx), 0.0);
                        } else {
                            accel = std::min(leaderAccel + 0.5 * (dv - sdvo), 0.0);
                        }
                    }
                    if (accel > -p[6]) {
                        accel = -p[6];
                    } else {
                        accel = std::max(accel, -10 + 0.5 * std::sqrt(speed));
                    }
                }
            } else if (dv < sdvc && dx < sdxv) {
                // decelerate - decrease distance
                accel = 0.5 * dv * dv / (sdxc - dx - 0.1);
            } else if (dv < sdvo && dx < sdxo) {
                // accelerate / decelerate - keep distance
                accel = oldAccel <= 0 ? std::min(oldAccel, -p[6]) : std::max(oldAccel, p[6]);
            } else if (dx > sdxc) {
                // accelerate / relax - increase / keep speed
                const double accelMax = p[7] + p[8] * std::min(speed, 80 / 3.6) + cfRand(v, salt + 1);
                accel = dx < sdxo ? std::min(dv * dv / (sdxo - dx), accelMax) : accelMax;
            }
            const double vNew = speed + ACCEL2SPEED(accel);
            return BALLISTIC ? vNew : std::max(0.0, vNew);
        }
        case CFM::WIEDEMANN:
            return wiedemannV(v, predSpeed, gap, pred >= 0 ? kin[pred].accel : 0);
        case CFM::BKERNER: {
            const double vMax = cfMaxNextSpeed(v, speed);
            return std::min(kernerV(v, speed, vMax, gap, predSpeed), vMax);
        }
        case CFM::PWAGNER2009: {
            if (predSpeed == 0 && gap < 0.01) {
                return 0;
            }
            const std::vector<double>& p = t.cfp;
            const double vsafe = -p[PW_TAU_LAST_DECEL] + std::sqrt(p[PW_TAU_LAST_DECEL] * p[PW_TAU_LAST_DECEL] + predSpeed * predSpeed + 2.0 * t.decel * gap);
            const double asafe = SPEED2ACCEL(vsafe - speed);
            const CFVars* vars = cfv(v);
            double apref = vars->aOld;
            const uint64_t salt = std::hash<double>()(gap) ^ std::hash<double>()(predSpeed);
            if (apref <= asafe && cfRand(v, salt) <= p[PW_AP_PROB] * TS) {
                apref = p[PW_DECEL_DIV_TAU] * (gap + (predSpeed - speed) * t.tau - speed * t.tau) / (speed + p[PW_TAU_DECEL]);
                apref = std::min(apref, t.accel);
                apref = std::max(apref, -t.decel);
                apref += t.sigma * (2 * cfRand(v, salt + 1) - 1);
            }
            if (apref > asafe) {
                apref = asafe;
            }
            return std::max(0., speed + ACCEL2SPEED(apref));
        }
        case CFM::SMART_SK: {
            CFVars* vars = cfv(v);
            const std::vector<double>& p = t.cfp;
            const bool own = v == cfOwner;
            if (own && (gap - vars->gOld) < p[SK_MAX_DELTA_GAP]) {
                const double tTauTest = gap / speed;
                if (tTauTest < vars->headway && tTauTest > TS) {
                    vars->headway = tTauTest;
                }
            }
            double vsafe = smartSKVsafe(t, vars, gap, predSpeed);
            if (speed <= 0.0 && vsafe < p[SK_S2S]) {
                vsafe = 0;
            }
            const double vNew = std::max(cfSpeedAfterMaxDecel(v, speed), std::min(vsafe, cfMaxNextSpeed(v, speed)));
            if (own) {
                vars->gOld = gap;
                vars->ggOld[(int)vNew] = gap;
            }
            return vNew;
        }
        case CFM::DANIEL1:
            return std::min(cf::kraussOrig1Vsafe(t, t.cfp[0], gap, predSpeed), cfMaxNextSpeed(v, speed));
        case CFM::NASCH:
            return naschVsafe(v, speed, gap);
        case CFM::EIDM: {
            cf::CalcReason u = computeLC || cfComputeLC ? cf::CalcReason::LANE_CHANGE : usage;
            const int update = u == cf::CalcReason::LANE_CHANGE || u == cf::CalcReason::FUTURE ? 0 : 1;
            return eidmV(v, gap, speed, predSpeed, cfv(v)->v0_int, true, update, u);
        }
        case CFM::ACC: {
            const int l = lane >= 0 ? lane : kin[v].lane;
            const double desSpeed = std::min(net.lanes[l].speed, vehMaxSpeed(v));
            const double vACC = accV(v, 0, gap, speed, predSpeed, desSpeed);
            const double vSafe = cf::maximumSafeFollowSpeed(t, gap, speed, predSpeed, predMaxDecel, false, computeLC);
            return vSafe + t.cfp[ACC_EMERGENCY] < vACC ? vSafe + t.cfp[ACC_EMERGENCY] : vACC;
        }
        case CFM::CACC: {
            const double desSpeed = laneVehMaxSpeed(lane >= 0 ? lane : kin[v].lane, v);
            const double vCACC = caccV(v, pred, gap, speed, predSpeed, desSpeed);
            const double vSafe = cf::maximumSafeFollowSpeed(t, gap, speed, predSpeed, predMaxDecel, true, computeLC);
            const double speedOverride = std::min(t.cfp[C_EMERGENCY], gap);
            return vSafe + speedOverride < vCACC ? std::max(0.0, vSafe + speedOverride) : vCACC;
        }
        default:
            break;
    }
    (void)usage;
    return cf::followSpeed(t, speed, gap, predSpeed, predMaxDecel, computeLC);
}

double Sim::cfStopSpeed(int v, double speed, double gap, double decel, cf::CalcReason usage, int lane) const {
    const VType& t = vt(v);
    if (decel < 0) {
        decel = t.decel;
    }
    switch (t.cfm) {
        case CFM::KRAUSS_ORIG1:
            if (BALLISTIC) {
                return std::min(cf::maximumSafeStopSpeedBallistic(t, gap, decel, speed, false, -1), cfMaxNextSpeed(v, speed));
            }
            return std::min(cf::kraussOrig1Vsafe(t, t.cfp[K_TAU_DECEL], gap, 0.), cfMaxNextSpeed(v, speed));
        case CFM::IDM:
        case CFM::IDMM: {
            if (gap < 0.01) {
                return 0;
            }
            double result = idmV(t, cfv(v), gap, speed, 0, laneVehMaxSpeed(lane >= 0 ? lane : kin[v].lane, v), false);
            if (gap > 0 && speed < NUMERICAL_EPS && result < NUMERICAL_EPS) {
                // ensure that stops can be reached
                result = cf::maximumSafeStopSpeed(t, gap, decel, speed, false, actionStepLengthSecs(v), true);
            }
            if (gap >= 0) {
                result = std::min(result, DIST2SPEED(gap));  // avoid overshooting the stop location
            }
            return result;
        }
        case CFM::EIDM: {
            cf::CalcReason u = cfComputeLC ? cf::CalcReason::LANE_CHANGE : usage;
            const int update = u == cf::CalcReason::LANE_CHANGE || u == cf::CalcReason::FUTURE || usage == cf::CalcReason::CURRENT_WAIT ? 0 : 1;
            return eidmV(v, gap, speed, 0, cfv(v)->v0_int, false, update, u);
        }
        case CFM::W99:
        case CFM::WIEDEMANN:
        case CFM::ACC:
        case CFM::CACC:
            return std::min(cf::maximumSafeStopSpeed(t, gap, decel, speed, false, actionStepLengthSecs(v), true), cfMaxNextSpeed(v, speed));
        case CFM::BKERNER: {
            const double vMax = cfMaxNextSpeed(v, speed);
            return std::min(kernerV(v, speed, vMax, gap, 0), vMax);
        }
        case CFM::PWAGNER2009: {
            if (gap < 0.01) {
                return 0.;
            }
            const std::vector<double>& p = t.cfp;
            const double vsafe = -p[PW_TAU_DECEL] + std::sqrt(p[PW_TAU_DECEL] * p[PW_TAU_DECEL] + 2.0 * t.decel * gap);
            const double asafe = SPEED2ACCEL(vsafe - speed);
            double apref = p[PW_DECEL_DIV_TAU] * (gap - 2 * speed * t.tau) / (speed + p[PW_TAU_DECEL]);
            if (apref <= asafe) {
                apref = std::min(apref, t.accel);
                apref = std::max(apref, -t.decel);
            } else {
                apref = asafe;
            }
            return std::max(0., vsafe + ACCEL2SPEED(apref));
        }
        case CFM::SMART_SK: {
            CFVars* vars = cfv(v);
            if (v == cfOwner && (gap - vars->gOld) < t.cfp[SK_MAX_DELTA_GAP]) {
                const double tTauTest = gap / speed;
                if (tTauTest < vars->headway && tTauTest > TS) {
                    vars->headway = tTauTest;
                }
            }
            return std::max(cfSpeedAfterMaxDecel(v, speed), std::min(smartSKVsafe(t, vars, gap, 0), cfMaxNextSpeed(v, speed)));
        }
        case CFM::DANIEL1:
            return std::min(cf::kraussOrig1Vsafe(t, t.cfp[0], gap, 0), cfMaxNextSpeed(v, speed));
        case CFM::NASCH:
            return naschVsafe(v, speed, gap);
        default:
            // Krauss, KraussPS, KraussX (Krauss::stopSpeed), Rail
            if (t.cfm == CFM::RAIL) {
                return std::min(cf::maximumSafeStopSpeed(t, gap, decel, speed, false, TS, false), cfMaxNextSpeed(v, speed));
            }
            return std::min(cf::maximumSafeStopSpeed(t, gap, decel, speed, false, actionStepLengthSecs(v), usage != cf::CalcReason::FUTURE),
                            cfMaxNextSpeed(v, speed));
    }
}

double Sim::cfFreeSpeed(int v, double speed, double seen, double maxSpeed, bool onInsertion, int lane) const {
    const VType& t = vt(v);
    if (t.cfm == CFM::EIDM) {
        return eidmFreeSpeed(v, speed, seen, maxSpeed, onInsertion, cf::CalcReason::CURRENT);
    }
    if ((t.cfm == CFM::IDM || t.cfm == CFM::IDMM) && maxSpeed >= 0.) {
        // MSCFModel_IDM::freeSpeed
        const double secGap = idmSecureGap(t, maxSpeed, 0);
        const double laneMax = laneVehMaxSpeed(lane >= 0 ? lane : kin[v].lane, v);
        double vSafe;
        if (speed <= maxSpeed) {
            vSafe = idmV(t, cfv(v), 1e6, speed, maxSpeed, laneMax, false);  // accelerate
        } else {
            // decelerate (relax the gap to avoid emergency braking, the transition point does not move)
            vSafe = idmV(t, cfv(v), std::max(seen, secGap), speed, 0, laneMax, false);
        }
        if (seen < secGap) {
            vSafe = std::min(vSafe, maxSpeed);  // avoid overshoot when close to a change in the speed limit
        }
        return vSafe;
    }
    if (t.rail || maxSpeed < 0.) {
        return cf::freeSpeed(t, speed, seen, maxSpeed, onInsertion);
    }
    return cf::freeSpeed(speed, t.decel, seen, maxSpeed, onInsertion, actionStepLengthSecs(v));
}

double Sim::cfInsertionFollowSpeed(int v, double speed, double gap, double predSpeed, double predMaxDecel, int pred) const {
    const VType& t = vt(v);
    switch (t.cfm) {
        case CFM::IDM:
        case CFM::IDMM: {
            const double s = std::max(0., speed * t.tau + speed * (speed - predSpeed) / t.cfp[IDM_TWO_SQRT]);
            if (gap >= s) {
                return speed;
            }
            // the safe speed of an equilibrium state (followSpeed assumes 'speed' to be the current speed)
            const double speed2 = cfFollowSpeed(v, speed, gap, predSpeed, predMaxDecel, pred, cf::CalcReason::FUTURE);
            const double speed3 = cfFollowSpeed(v, speed2, gap, predSpeed, predMaxDecel, pred, cf::CalcReason::FUTURE);
            if (speed2 - speed3 < ACCEL2SPEED(1)) {
                return speed2;
            }
            return cfInsertionFollowSpeed(v, speed2, gap, predSpeed, predMaxDecel, pred);
        }
        case CFM::NASCH:
            return naschVsafe(v, speed, gap);
        case CFM::EIDM:
            return eidmMaximumSafeFollowSpeed(t, gap, speed, predSpeed, predMaxDecel, true, false);
        case CFM::ACC: {
            // a stationary value of followSpeed
            double res = speed;
            for (int i = 0; i < 50; ++i) {
                const double a = SPEED2ACCEL(cfFollowSpeed(v, res, gap, predSpeed, predMaxDecel, -1, cf::CalcReason::FUTURE) - res);
                res = res + 0.1 * a;
                if (std::fabs(a) < 0.1) {
                    break;
                }
            }
            return res;
        }
        case CFM::CACC: {
            double res = speed;
            double damping = 0.8;
            for (int i = 0; i < 50; ++i) {
                const double vCACC = caccV(v, pred, gap, res, predSpeed, speed);
                const double vSafe = cf::maximumSafeFollowSpeed(t, gap, res, predSpeed, predMaxDecel, true, false);
                const double a = std::min(vCACC, vSafe) - res;
                res = res + damping * a;
                damping *= 0.9;
                if (std::fabs(a) < 0.1) {
                    break;
                }
            }
            return res;
        }
        default:
            return cf::insertionFollowSpeed(t, speed, gap, predSpeed, predMaxDecel);
    }
}

double Sim::cfInsertionStopSpeed(int v, double speed, double gap, int lane) const {
    const VType& t = vt(v);
    if (t.cfm == CFM::IDM || t.cfm == CFM::IDMM) {
        // an equilibrium state (MSCFModel::insertionStopSpeed iterated)
        auto base = [&](double sp) {
            return BALLISTIC ? std::min(cf::maximumSafeStopSpeed(t, gap, t.decel, 0., true, 0., false), t.maxSpeed)
                   : cfStopSpeed(v, sp, gap, -1, cf::CalcReason::FUTURE, lane);
        };
        double result = base(speed);
        int i = 0;
        while (result - speed < -ACCEL2SPEED(t.decel) && ++i < 10) {
            speed = result;
            result = base(speed);
        }
        return result;
    }
    if (t.cfm == CFM::KRAUSS || t.cfm == CFM::RAIL) {
        return cf::insertionStopSpeed(t, speed, gap);
    }
    if (t.cfm == CFM::EIDM) {
        const double v = eidmMaximumSafeStopSpeed(t, gap, t.decel, speed, t.tau);
        return BALLISTIC ? std::min(v, t.maxSpeed) : v;
    }
    return cfStopSpeed(v, speed, gap, -1, cf::CalcReason::FUTURE, lane);
}

double Sim::cfMaxNextSpeed(int v, double speed) const {
    const VType& t = vt(v);
    if (t.cfm == CFM::KRAUSS_PS) {
        // MSCFModel_KraussPS::maxNextSpeed: the slope reduces the acceleration
        const double aBound = cf::currentAccel(t, speed);
        const double aMax = std::max(0., aBound - GRAVITY * std::sin(cfSlope(v) * M_PI / 180.));
        const double typeMax = t.maxSpeed * (veh[v].vclass == VC_BICYCLE ? veh[v].speedFactor : 1);
        const double vMax = std::max(std::sqrt(aMax / t.accel) * typeMax, speed - ACCEL2SPEED(t.decel));
        return std::max(aBound / 2, std::min(speed + ACCEL2SPEED(aMax), vMax));
    }
    if (t.cfm == CFM::RAIL) {
        return cf::railMaxNextSpeed(*t.rail, speed, cfSlope(v), railCurveResistance(v));
    }
    return cf::maxNextSpeed(t, speed);
}

double Sim::cfMaxNextSafeMin(int v, double speed) const {
    return vt(v).cfm == CFM::EIDM ? 0 : cfMaxNextSpeed(v, speed);
}

double Sim::cfMinNextSpeed(int v, double speed) const {
    const VType& t = vt(v);
    if (t.cfm == CFM::IDM || t.cfm == CFM::IDMM) {
        // permit exceeding decel when approaching stops
        const double decel = std::max(t.decel, std::min(t.emergencyDecel, 1.5));
        return BALLISTIC ? speed - ACCEL2SPEED(decel) : std::max(speed - ACCEL2SPEED(decel), 0.);
    }
    if (t.cfm == CFM::RAIL) {
        return cf::railMinNextSpeed(*t.rail, speed, cfSlope(v), railCurveResistance(v));
    }
    return cf::minNextSpeed(t, speed);
}

double Sim::cfMinNextSpeedEmergency(int v, double speed) const {
    const VType& t = vt(v);
    if (t.cfm == CFM::NASCH) {
        return BALLISTIC ? -DMAX : 0.;  // (the emergency deceleration is not bounded)
    }
    if (t.cfm == CFM::RAIL) {
        return cfMinNextSpeed(v, speed);
    }
    return cf::minNextSpeedEmergency(t, speed);
}

double Sim::cfBrakeGap(int v, double speed) const {
    const VType& t = vt(v);
    return cfBrakeGap(v, speed, t.decel, t.tau);
}

double Sim::cfBrakeGap(int v, double speed, double decel, double headway) const {
    const VType& t = vt(v);
    if (t.cfm == CFM::EIDM && !cfComputeLC) {
        return cf::brakeGap(speed, std::max(decel, t.decel + 1.0), headway);
    }
    return cf::brakeGap(speed, decel, headway);
}

double Sim::cfMaximumLaneSpeedCF(int v, double maxSpeed, double maxSpeedLane) const {
    return vt(v).cfm == CFM::EIDM ? maxSpeed : std::min(maxSpeed, maxSpeedLane);
}

double Sim::cfSecureGap(int v, int pred, double speed, double leaderSpeed, double leaderMaxDecel, bool computeLC) const {
    (void)pred;
    const VType& t = vt(v);
    if (t.cfm == CFM::IDM || t.cfm == CFM::IDMM) {
        return idmSecureGap(t, speed, leaderSpeed);
    }
    if (t.cfm == CFM::ACC) {
        return t.cfp[ACC_GC_SPEED] * (speed - leaderSpeed) / t.cfp[ACC_GC_SPACE] + t.tau * speed;
    }
    if (t.cfm == CFM::EIDM) {
        const double a = cf::currentAccel(t, speed);
        const double s = std::max(0.0, speed * t.tau + speed * (speed - leaderSpeed) / (2 * std::sqrt(a * t.decel)));
        return std::min(s, std::sqrt((s * s) / (t.decel / a + 1.0)));
    }
    // (MSGlobals::gComputeLC: every secure gap computed during the lane-change stage)
    const double base = cf::getSecureGap(t, speed, leaderSpeed, leaderMaxDecel, computeLC || cfComputeLC, kin[v].accel, 1. / t.lcAssertive);
    if (t.cfm == CFM::CACC) {
        const double desSpacing = pred < 0 || vt(pred).cfm != CFM::CACC
                                  ? t.cfp[ACC_GC_SPEED] * (speed - leaderSpeed) / t.cfp[ACC_GC_SPACE] + t.cfp[ACC_HEADWAY] * speed
                                  : t.tau * speed;
        return std::max(desSpacing, base);
    }
    if (t.cfm == CFM::WIEDEMANN) {
        const double bx = (1 + 7 * t.cfp[WD_SECURITY]) * std::sqrt(speed);
        const double abx = t.cfp[WD_AX] + bx - t.length;  // the brutto gap
        return std::max(abx, base);
    }
    return base;
}

double Sim::cfSpeedAfterMaxDecel(int v, double speed) const {
    return cf::getSpeedAfterMaxDecel(vt(v), speed);
}

double Sim::cfInteractionGap(int v, double vL) const {
    const VType& t = vt(v);
    const double speed = kin[v].speed;
    switch (t.cfm) {
        case CFM::W99:
            return W99_INTERACTION_GAP;
        case CFM::WIEDEMANN:
            return WIEDEMANN_D_MAX;
        case CFM::ACC:
        case CFM::CACC:
            return 250;  // the radar range
        case CFM::IDM:
        case CFM::IDMM:
        case CFM::EIDM: {
            const double laneMax = laneVehMaxSpeed(kin[v].lane, v);
            const double acc = cf::currentAccel(t, speed) * (1. - std::pow(speed / laneMax, t.cfp[t.cfm == CFM::EIDM ? (int)E_DELTA : (int)IDM_DELTA]));
            const double vNext = speed + acc;
            const double gap = (vNext - vL) * (speed + vL) / (2 * t.decel) + vL;
            return std::max(gap, SPEED2DIST(vNext));
        }
        default: {
            // MSCFModel::interactionGap
            const double vNext = std::min(cfMaxNextSpeed(v, speed), laneVehMaxSpeed(kin[v].lane, v));
            const double gap = (vNext - vL) * ((speed + vL) / (2. * t.decel) + t.tau) + vL * t.tau;
            return std::max(gap, SPEED2DIST(vNext));
        }
    }
}

// ---------------------------------------------------------------------------
// model helpers with vehicle state
// ---------------------------------------------------------------------------
// MSCFModel_Wiedemann::_v
double Sim::wiedemannV(int v, double predSpeed, double gap, double predAccel) const {
    const VType& t = vt(v);
    const std::vector<double>& p = t.cfp;
    const CFVars* vars = cfv(v);
    const double dx = gap + t.length;  // Wiedemann uses the brutto gap
    const double speed = kin[v].speed;
    const double vpref = vehMaxSpeed(v);
    const double dv = speed - predSpeed;
    const double bx = (1 + 7 * p[WD_SECURITY]) * std::sqrt(speed);
    const double abx = p[WD_AX] + bx;
    const double ex = 2 - p[WD_ESTIMATION];
    const double sdx = p[WD_AX] + ex * bx;
    const double sdvRoot = (dx - p[WD_AX]) / p[WD_CX];
    const double sdv = sdvRoot * sdvRoot;
    const double cldv = sdv * ex * ex;
    uint64_t k = std::hash<double>()(gap) ^ std::hash<double>()(predSpeed);
    const double opdv = cldv * (-1 - 2 * randNormPolar(0.5, 0.15, [&]() { return cfRand(v, k++); }));
    const double dmax = std::max(WIEDEMANN_D_MAX, cf::brakeGap(speed, t.decel, 0));
    auto fullspeed = [&]() {
        const double bmax = 0.2 + 0.8 * t.accel * (7 - std::sqrt(speed));
        double accel = dx <= 2 * abx ? std::min(p[WD_MIN_ACCEL], bmax * (dx - abx) / abx) : bmax;
        return speed > vpref ? -accel : accel;
    };
    auto approaching = [&]() {
        return std::max(0.5 * dv * dv / (abx - dx) + predAccel * 0.5, -p[WD_MAX_APPROACH_DECEL]);
    };
    double accel;
    if (dx <= abx) {
        // emergency
        if (dx > p[WD_AX]) {
            const double bmin = -1;
            accel = 0.5 * dv * dv / (p[WD_AX] - dx) + predAccel * 0.55 + bmin * (abx - gap) / bx;
        } else {
            accel = -t.emergencyDecel;
        }
    } else if (dx < sdx) {
        if (dv > cldv) {
            accel = approaching();
        } else if (dv > opdv) {
            accel = p[WD_MIN_ACCEL] * vars->accelSign;  // following
        } else {
            accel = fullspeed();
        }
    } else {
        accel = dv > sdv && dx < dmax ? approaching() : fullspeed();
    }
    accel = std::max(std::min(accel, t.accel), -t.emergencyDecel);
    return std::max(0., speed + ACCEL2SPEED(accel));
}

// MSCFModel_Kerner::_v
double Sim::kernerV(int v, double speed, double vfree, double gap, double predSpeed) const {
    if (predSpeed == 0 && gap < 0.01) {
        return 0;
    }
    const VType& t = vt(v);
    const std::vector<double>& p = t.cfp;
    const double G = std::max(0., SPEED2DIST(p[BK_K] * speed) + p[BK_PHI] / t.accel * speed * (speed - predSpeed));
    const double vcond = gap > G ? speed + ACCEL2SPEED(t.accel)
                         : speed + std::max(ACCEL2SPEED(-t.decel), std::min(ACCEL2SPEED(t.accel), predSpeed - speed));
    const double td = p[BK_TAU_DECEL];
    const double vsafe = -1. * td + std::sqrt(td * td + predSpeed * predSpeed + 2. * t.decel * gap);
    const double va = std::max(0., std::min({vfree, vsafe, vcond})) + cfv(v)->rnd;
    return std::max(0., std::min({vfree, va, speed + ACCEL2SPEED(t.accel), vsafe}));
}

double Sim::smartSKVsafe(const VType& t, const CFVars* vars, double gap, double predSpeed) const {
    if (predSpeed == 0 && gap < 0.01) {
        return 0;
    }
    const double bTau = t.decel * vars->headway;
    return -1. * bTau + std::sqrt(bTau * bTau + predSpeed * predSpeed + 2. * t.decel * gap);
}

// MSCFModel_ACC::_v (block: the parameters of the ACC block, the headway at ACC_HEADWAY)
double Sim::accV(int v, int block, double gap, double speed, double predSpeed, double desSpeed) const {
    const VType& t = vt(v);
    const double* p = t.cfp.data() + block;
    CFVars* vars = cfv(v);
    const double vErr = speed - desSpeed;
    bool setControlMode = false;
    if (vars->accLastUpdate != now && v == cfOwner) {
        vars->accLastUpdate = now;
        setControlMode = true;
    }
    auto gapControl = [&]() {
        const double deltaVel = predSpeed - speed;
        // dynamic gap margin (Xiao et al. 2018)
        const double d0 = std::max(0., std::min(75. / speed - 5., 2.));
        const double spacingErr = gap - p[ACC_HEADWAY] * speed - d0;
        if (std::fabs(spacingErr) < 0.2 && std::fabs(vErr) < 0.1) {
            return p[ACC_GC_SPEED] * deltaVel + p[ACC_GC_SPACE] * spacingErr;  // gap mode
        } else if (spacingErr < 0) {
            return p[ACC_CA_SPEED] * deltaVel + p[ACC_CA_SPACE] * spacingErr;  // collision avoidance mode
        }
        return p[ACC_GCC_SPEED] * deltaVel + p[ACC_GCC_SPACE] * spacingErr;  // gap closing mode
    };
    double accel;
    if (gap > 120) {
        accel = p[ACC_SC] * vErr;  // speed control
        if (setControlMode) {
            vars->accControlMode = 0;
        }
    } else if (gap < 100) {
        accel = gapControl();
        if (setControlMode) {
            vars->accControlMode = 1;
        }
    } else {
        accel = vars->accControlMode == 0 ? p[ACC_SC] * vErr : gapControl();  // the previous law
    }
    return std::max(0., speed + ACCEL2SPEED(accel));
}

// MSCFModel_CACC::_v
double Sim::caccV(int v, int pred, double gap, double speed, double predSpeed, double desSpeed) const {
    const VType& t = vt(v);
    const std::vector<double>& p = t.cfp;
    CFVars* vars = cfv(v);
    const double vErr = speed - desSpeed;
    bool setControlMode = false;
    if (vars->accLastUpdate != now && v == cfOwner) {
        vars->accLastUpdate = now;
        setControlMode = true;
    }
    auto speedControl = [&]() { return speed + ACCEL2SPEED(p[C_SC] * vErr); };
    auto cavGapControl = [&]() {
        const double spacingErr = gap - t.tau * speed;
        const double speedErr = predSpeed - speed - t.tau * kin[v].accel;
        if (spacingErr > 0 && spacingErr < 0.2 && vErr < 0.1) {
            return speed + p[C_GC_GAP] * spacingErr + p[C_GC_GAP_DOT] * speedErr;
        } else if (spacingErr < 0) {
            return speed + p[C_CA_GAP] * spacingErr + p[C_CA_GAP_DOT] * speedErr;
        }
        return speed + p[C_GCC_GAP] * spacingErr + p[C_GCC_GAP_DOT] * speedErr;
    };
    // speedGapControl: ACC behind vehicles without CACC
    auto speedGapControl = [&]() {
        if (pred < 0) {
            return speedControl();
        }
        if (vt(pred).cfm != CFM::CACC) {
            return accV(v, 0, gap, speed, predSpeed, desSpeed);
        }
        return cavGapControl();
    };
    double newSpeed = 0;
    switch (vars->caccCommMode) {
        case 0: {  // rely on the time gap to the predecessor
            const double timeGap = gap / std::max(NUMERICAL_EPS, speed);
            const double spacingErr = gap - t.tau * speed;
            if (timeGap > 2 && spacingErr > p[C_SC_MIN_GAP]) {
                newSpeed = speedControl();
                if (setControlMode) {
                    vars->caccControlMode = 0;
                }
            } else if (timeGap < 1.5) {
                newSpeed = speedGapControl();
                if (setControlMode) {
                    vars->caccControlMode = 1;
                }
            } else {
                newSpeed = vars->caccControlMode == 0 ? speedControl() : speedGapControl();
            }
            break;
        }
        case 1:  // no leader
            newSpeed = speedControl();
            break;
        case 2:  // leader without communication
            newSpeed = accV(v, 0, gap, speed, predSpeed, desSpeed);
            break;
        default:  // leader with communication
            newSpeed = cavGapControl();
            break;
    }
    if (DELTA_T < 100) {
        newSpeed = speed + ACCEL2SPEED((newSpeed - speed) * 10);
    }
    return std::max(0., newSpeed);
}

// MSCFModel_NaSch::vsafe
double Sim::naschVsafe(int v, double speed, double gap) const {
    if (gap < NUMERICAL_EPS) {
        return 0.;
    }
    const double vGapLimited = std::min(cfMaxNextSpeed(v, speed), DIST2SPEED(gap));
    return naschRoundToCell(vt(v), std::max(0., vGapLimited));
}

double Sim::naschRoundToCell(const VType& t, double speed) const {
    const double cell = t.cfp[NS_CELL_SPEED];
    if (speed <= 0. || cell <= 0.) {
        return 0.;
    }
    return std::floor(speed / cell + NUMERICAL_EPS) * cell;
}

// ---------------------------------------------------------------------------
// MSCFModel::finalizeSpeed and the models' patchSpeedBeforeLC / finalizeSpeed
// ---------------------------------------------------------------------------
// MSVehicle::executeMove: myTimeSinceStartup (before finalizeSpeed)
void Sim::cfUpdateStartup(int v, double vSafe) {
    if (!cfHaveVars) {
        return;
    }
    CFVars& c = cfVars[v];
    const VType& t = vt(v);
    const double rawAccel = SPEED2ACCEL(std::max(vSafe, 0.) - kin[v].speed);
    if (vSafe <= HALTING_SPEED * TS && veh[v].waitingTime > opt.startupWaitThreshold && rawAccel <= 0.5 * t.accel
            && veh[v].actionStep) {
        c.timeSinceStartup = 0;
    } else if (isStopped(v)) {
        // (no startup delay for waypoints)
        if (t.cfm == CFM::RAIL && veh[v].stops.front().speed <= 0) {
            c.timeSinceStartup = DELTA_T;
        } else {
            c.timeSinceStartup = TIME2STEPS(t.startupDelay) + DELTA_T;
        }
    } else {
        c.timeSinceStartup += DELTA_T;
    }
}

double Sim::cfFinalizeSpeed(int v, double vPos, std::vector<StopEvent>& stopEv) {
    const Kin& k = kin[v];
    const VType& t = vt(v);
    const double oldV = k.speed;
    const int prevOwner = cfOwner;
    cfOwner = v;
    if (t.cfm == CFM::EIDM) {
        const double vNext = eidmFinalizeSpeed(v, vPos, stopEv);
        cfOwner = prevOwner;
        return vNext;
    }
    if (t.cfm == CFM::DANIEL1) {
        // MSCFModel_Daniel1::finalizeSpeed
        const double vSafe = std::min(vPos, processNextStop(v, vPos, stopEv));
        const double vMin = cfSpeedAfterMaxDecel(v, oldV);
        const double vMax = std::min({laneVehMaxSpeed(k.lane, v), cfMaxNextSpeed(v, oldV), vSafe});
        const double vDawdle = std::max(0., vMax - ACCEL2SPEED(t.sigma * t.accel * cfRand(v, 0)));
        cfOwner = prevOwner;
        return lcPatchSpeed(v, vMin, std::max(vMin, vDawdle), vMax);
    }
    const double vStop = std::min(vPos, processNextStop(v, vPos, stopEv));
    const double vMinEmergency = cfMinNextSpeedEmergency(v, oldV);
    const double vMin = std::min(cfMinNextSpeed(v, oldV), std::max(vPos, vMinEmergency));
    const double aMax = (std::max(laneVehMaxSpeed(k.lane, v), vPos) - oldV) / actionStepLengthSecs(v);
    double vMax = std::min(std::min(oldV + ACCEL2SPEED(aMax), cfMaxNextSpeed(v, oldV)), vStop);
    vMax = std::max(vMin, vMax);
    // patchSpeedBeforeLC
    double vNext = vMax;
    switch (t.cfm) {
        case CFM::KRAUSS:
        case CFM::KRAUSS_PS: {
            const double sigma = t.cfp[K_SIGMA_MINOR] != t.sigma && passingMinor(v) ? t.cfp[K_SIGMA_MINOR] : t.sigma;
            const Time dawdleStep = (Time)t.cfp[K_DAWDLE_STEP];
            const double rnd = rand01(opt.seed, (uint64_t)v, (uint64_t)step, RP_DAWDLE);
            if (dawdleStep > DELTA_T) {
                CFVars* vars = cfv(v);
                if (now % dawdleStep == vars->updateOffset) {
                    const double vD = std::max(vMin, cf::dawdle2(t, vMax, sigma, rnd));
                    const double a1 = SPEED2ACCEL(vMax - oldV);
                    const double a2 = SPEED2ACCEL(vD - vMax);
                    const double accelMax = (laneVehMaxSpeed(k.lane, v) - oldV) / STEPS2TIME(dawdleStep);
                    vars->accelDawdle = std::min(a1, accelMax) + a2;
                    vNext = oldV + ACCEL2SPEED(vars->accelDawdle);
                } else {
                    const double accel = std::min(SPEED2ACCEL(vMax - oldV), vars->accelDawdle);
                    vNext = std::max(vMin, std::min(vMax, oldV + ACCEL2SPEED(accel)));
                }
            } else {
                vNext = std::max(vMin, cf::dawdle2(t, vMax, sigma, rnd));
            }
            break;
        }
        case CFM::KRAUSS_ORIG1: {
            const double rnd = rand01(opt.seed, (uint64_t)v, (uint64_t)step, RP_DAWDLE);
            const double dawdled = BALLISTIC && vMax < 0 ? vMax : std::max(0., vMax - ACCEL2SPEED(t.sigma * t.accel * rnd));
            vNext = std::max(vMin, dawdled);
            break;
        }
        case CFM::KRAUSSX: {
            // MSCFModel_KraussX::dawdleX
            double speed = vMax;
            if (BALLISTIC && speed < 0) {
                vNext = speed;  // (a desired stop within the next step)
                break;
            }
            if (oldV < t.accel) {
                speed -= ACCEL2SPEED(t.cfp[K_TMP1] * t.accel);  // extra slow to start
            }
            const double rnd = rand01(opt.seed, (uint64_t)v, (uint64_t)step, RP_DAWDLE);
            speed -= ACCEL2SPEED(t.sigma * t.accel * rnd);
            if (oldV > vMax) {
                speed -= ACCEL2SPEED(t.cfp[K_TMP2] * t.accel * rnd);  // overbraking
                if (!BALLISTIC) {
                    speed = std::max(0.0, speed);
                }
            }
            vNext = std::max(vMin, speed);
            break;
        }
        case CFM::SMART_SK:
            // (dawdling from the current speed)
            vNext = std::max(0., oldV - ACCEL2SPEED(t.sigma * t.accel * rand01(opt.seed, (uint64_t)v, (uint64_t)step, RP_DAWDLE)));
            break;
        case CFM::NASCH:
            if (vMax > 0. && t.sigma > 0. && rand01(opt.seed, (uint64_t)v, (uint64_t)step, RP_DAWDLE) < t.sigma) {
                vNext = std::max(vMin, naschRoundToCell(t, vMax - t.cfp[NS_CELL_SPEED]));
            }
            break;
        default:
            break;  // MSCFModel::patchSpeedBeforeLC (Rail, IDM, W99, Wiedemann, BKerner, PWagner2009)
    }
    vNext = lcPatchSpeed(v, vMin, vNext, vMax);
    // MSCFModel::applyStartupDelay
    if (cfHaveVars && t.startupDelay > 0) {
        const Time since = cfVars[v].timeSinceStartup;
        const Time delay = TIME2STEPS(t.startupDelay);
        if (since > 0 && since - DELTA_T < delay) {
            const Time remaining = delay - (since - DELTA_T);
            vNext = remaining >= DELTA_T ? 0 : (double)(DELTA_T - remaining) / (double)DELTA_T * vNext;
        }
    }
    // the models' finalizeSpeed
    switch (t.cfm) {
        case CFM::IDMM: {
            CFVars* vars = cfv(v);
            vars->levelOfService += (vNext / laneVehMaxSpeed(k.lane, v) - vars->levelOfService) / t.cfp[IDM_ADAPT_TIME] * TS;
            break;
        }
        case CFM::WIEDEMANN:
            cfv(v)->accelSign = vNext > oldV ? 1. : -1.;
            break;
        case CFM::BKERNER:
            cfv(v)->rnd = rand01(opt.seed, (uint64_t)v, (uint64_t)step, RP_CF_STATE);
            break;
        case CFM::PWAGNER2009:
            cfv(v)->aOld = SPEED2ACCEL(vNext - oldV);
            break;
        case CFM::SMART_SK: {
            CFVars* vars = cfv(v);
            // updateMyHeadway: the preferred headway changes slowly
            double tTau = vars->headway;
            tTau = tTau + (t.tau - tTau) * t.cfp[SK_TMP2] + t.cfp[SK_TMP3] * tTau * (2 * cfRand(v, 7) - 1);
            vars->headway = std::max(tTau, TS);
            vars->gOld = vars->ggOld[(int)vPos];
            vars->ggOld.clear();
            break;
        }
        default:
            break;
    }
    cfOwner = prevOwner;
    return vNext;
}

// ---------------------------------------------------------------------------
// EIDM (MSCFModel_EIDM): IIDM with reaction time (action points), estimation errors (Wiener processes),
// coolness, drive-off term and a look-ahead internal desired speed
// ---------------------------------------------------------------------------
// MSVehicle::getLeader (the junction foes of getLeaderOnConsecutive are not considered)
std::pair<int, double> Sim::vehicleLeader(int v, double dist) const {
    const Kin& k = kin[v];
    const std::vector<int>& vs = lanes[k.lane].vehs;
    auto it = std::find(vs.begin(), vs.end(), v);
    if (it != vs.end() && it + 1 != vs.end()) {
        const int lead = *(it + 1);
        return {lead, backPositionOnLane(lead, k.lane) - k.pos - vt(v).minGap};
    }
    double seen = net.lanes[k.lane].length - k.pos;
    for (int lane : bestContinuation(v)) {
        if (lane == k.lane || lane < 0) {
            continue;
        }
        if (seen > dist) {
            break;
        }
        int best = -1;
        double bestBack = DMAX;
        anyVehicles(lane, [&](int u) {
            const double b = backPositionOnLane(u, lane);
            if (u != v && b < bestBack) {
                bestBack = b;
                best = u;
            }
            return true;
        });
        if (best >= 0) {
            return {best, seen + bestBack - vt(v).minGap};
        }
        seen += net.lanes[lane].length;
    }
    return {-1, -1};
}

double Sim::eidmV(int v, double gap2pred, double egoSpeed, double predSpeed, double desSpeed, bool respectMinGap, int update,
                  cf::CalcReason usage) const {
    const VType& t = vt(v);
    const std::vector<double>& p = t.cfp;
    CFVars* vars = cfv(v);
    const bool own = v == cfOwner;
    const int iterations = (int)p[E_ITER];
    double v0 = std::max(NUMERICAL_EPS, desSpeed);
    if (vars->v0_old == 0) {
        // the first iteration
        v0 = std::max(NUMERICAL_EPS, laneVehMaxSpeed(kin[v].lane, v));
        if (own) {
            vars->v0_old = v0;
            vars->v0_int = v0;
        }
    }
    const bool current = usage == cf::CalcReason::CURRENT && own;
    const double simTime = STEPS2TIME(now);
    double wantedacc = 0., a_free;
    double wouldacc = 0., woulds, woulda_free;
    double estSpeed, estleaderSpeed, estGap;
    double current_estSpeed, current_estGap, current_estleaderSpeed;
    double acc = 0.;
    double a_leader = NUMERICAL_EPS;  // default without a leader, not 0
    double newSpeed = egoSpeed;
    bool lastrespectMinGap = respectMinGap;
    const double minGapStop_EPS = 0.05 + 0.20 * std::max(0.25, cf::currentAccel(t, 0.0));
    // (the follow calculation subtracts minGap from the gap)
    const double current_gap = respectMinGap ? std::max(NUMERICAL_EPS, gap2pred + std::max(NUMERICAL_EPS, t.minGap - 0.25))
                               : std::max(NUMERICAL_EPS, gap2pred + minGapStop_EPS);
    double newGap = current_gap;
    for (int i = 0; i < iterations; i++) {
        const double subStep = TS * (iterations - i - 1.) / iterations;
        current_estSpeed = newSpeed;
        current_estleaderSpeed = respectMinGap ? std::max(predSpeed - newGap * p[E_SIG_LEADER] * vars->w_speed, 0.0) : predSpeed;
        if (update == 2) {
            current_estGap = newGap;  // (freeSpeed)
        } else if (respectMinGap) {
            current_estGap = newGap * std::exp(p[E_SIG_GAP] * vars->w_gap);
        } else {
            current_estGap = newGap * std::exp(p[E_SIG_GAP] * vars->w_gap * std::min(current_estSpeed / EIDM_EST_REAC_THRESHOLD, 1.0));
        }
        if (vars->ap_update == 0 && usage == cf::CalcReason::CURRENT) {
            // update with the current observation
            estSpeed = current_estSpeed;
            estleaderSpeed = current_estleaderSpeed;
            estGap = current_estGap;
        } else if (usage == cf::CalcReason::CURRENT) {
            // the stored variables (reaction time)
            const double since = vars->ap_update * TS - subStep;
            estSpeed = std::max(vars->v_est + vars->lastrealacc * since, 0.0);
            if (update == 2) {
                estGap = newGap;
                estleaderSpeed = std::max(vars->v_est_l + vars->lastleaderacc * since - vars->s_est * p[E_SIG_LEADER] * vars->w_speed, 0.0);
            } else {
                lastrespectMinGap = vars->respectMinGap;
                if (lastrespectMinGap) {
                    estleaderSpeed = std::max(vars->v_est_l + vars->lastleaderacc * since - vars->s_est * p[E_SIG_LEADER] * vars->w_speed, 0.0);
                    estGap = vars->s_est * std::exp(p[E_SIG_GAP] * vars->w_gap)
                             - ((vars->v_est + estSpeed) / 2. - (vars->v_est_l + estleaderSpeed) / 2.) * since;
                } else {
                    estleaderSpeed = vars->v_est_l;
                    estGap = vars->s_est * std::exp(p[E_SIG_GAP] * vars->w_gap * std::min(current_estSpeed / EIDM_EST_REAC_THRESHOLD, 1.0))
                             - ((vars->v_est + estSpeed) / 2. - (vars->v_est_l + estleaderSpeed) / 2.) * since;
                }
            }
        } else {
            estSpeed = current_estSpeed;
            estleaderSpeed = current_estleaderSpeed;
            estGap = current_estGap;
        }
        const double headwayTime = t.tau;
        const double Accel_estSpeed = cf::currentAccel(t, estSpeed);
        const double Accel_current_estSpeed = cf::currentAccel(t, current_estSpeed);
        double s = std::max(0., estSpeed * headwayTime + estSpeed * (estSpeed - estleaderSpeed) / (2 * std::sqrt(Accel_estSpeed * t.decel)));
        s += lastrespectMinGap ? t.minGap + EIDM_POS_ACC_EPS : minGapStop_EPS + EIDM_POS_ACC_EPS;
        // no oscillations of the acceleration at small gaps
        if (lastrespectMinGap) {
            if (estGap < t.minGap + (TS * 10 + 1) * EIDM_POS_ACC_EPS && estSpeed < EIDM_EST_REAC_THRESHOLD
                    && s < estGap * std::sqrt(1 + 2 * EIDM_POS_ACC_EPS / Accel_estSpeed)) {
                s = estGap * std::sqrt(1 + 2 * EIDM_POS_ACC_EPS / Accel_estSpeed);
            }
        } else if (estGap < minGapStop_EPS + 2 * EIDM_POS_ACC_EPS && s < estGap * std::sqrt(1 + EIDM_POS_ACC_EPS / Accel_estSpeed)) {
            s = estGap * std::sqrt(1 + EIDM_POS_ACC_EPS / Accel_estSpeed);
        }
        // IIDM (Treiber/Kesting) with the variables of the last action point
        auto iidm = [&](double sp, double ss, double gap, double accel) {
            double af, result;
            if (sp <= v0) {
                af = accel * (1. - std::pow(sp / v0, p[E_DELTA]));
                result = ss >= gap ? accel * (1. - (ss * ss) / (gap * gap)) : af * (1. - std::pow(ss / gap, 2 * accel / af));
            } else {
                af = -t.decel * (1. - std::pow(v0 / sp, accel * p[E_DELTA] / t.decel));
                result = ss >= gap ? af + accel * (1. - (ss * ss) / (gap * gap)) : af;
            }
            return result;
        };
        wantedacc = iidm(estSpeed, s, estGap, Accel_estSpeed);
        wantedacc = SPEED2ACCEL(std::max(0.0, estSpeed + ACCEL2SPEED(wantedacc)) - estSpeed);
        // the acceleration without reaction time
        if (usage == cf::CalcReason::CURRENT) {
            woulds = std::max(0., current_estSpeed * headwayTime + current_estSpeed * (current_estSpeed - current_estleaderSpeed)
                              / (2 * std::sqrt(Accel_current_estSpeed * t.decel)));
            woulds += respectMinGap ? t.minGap + EIDM_POS_ACC_EPS : minGapStop_EPS + EIDM_POS_ACC_EPS;
            wouldacc = iidm(current_estSpeed, woulds, current_estGap, Accel_current_estSpeed);
            wouldacc = SPEED2ACCEL(std::max(0.0, current_estSpeed + ACCEL2SPEED(wouldacc)) - current_estSpeed);
        }
        (void)woulda_free;
        (void)a_free;
        // the resulting acceleration with the drive-off term
        acc = iidm(estSpeed, s, estGap, Accel_estSpeed);
        // coolness (EIDM) when the gap jumps to a smaller value
        if (vars->minaccel > wantedacc - NUMERICAL_EPS && usage == cf::CalcReason::CURRENT) {
            const int leader = vehicleLeader(v, estGap + 25).first;
            if (leader >= 0 && lastrespectMinGap && estleaderSpeed >= SPEED_EPS) {
                a_leader = std::min(kin[leader].accel, Accel_estSpeed);
                if (estGap > s * 3 / 2) {
                    a_leader = a_leader * (s * 3 / 2) / estGap;  // far leaders influence less
                }
            }
            if (estleaderSpeed < SPEED_EPS) {
                estleaderSpeed = SPEED_EPS;
            }
            if (vars->t_off + p[E_TACC_MAX] + NUMERICAL_EPS < simTime - subStep && egoSpeed > HALTING_SPEED) {
                double a_cah;
                if (estleaderSpeed * (estSpeed - estleaderSpeed) <= -2 * estGap * a_leader) {
                    a_cah = (estSpeed * estSpeed * a_leader) / (estleaderSpeed * estleaderSpeed - 2 * estGap * a_leader);
                } else if (estSpeed - estleaderSpeed >= 0) {
                    a_cah = a_leader - ((estSpeed - estleaderSpeed) * (estSpeed - estleaderSpeed)) / (2 * estGap);
                } else {
                    a_cah = a_leader;
                }
                if (acc < a_cah) {
                    acc = (1 - p[E_CCOOLNESS]) * acc + p[E_CCOOLNESS] * (a_cah + t.decel * std::tanh((acc - a_cah) / t.decel));
                }
            }
        }
        newSpeed = std::max(0.0, current_estSpeed + ACCEL2SPEED(acc) / iterations);
        newGap = std::max(NUMERICAL_EPS, newGap - std::max(0., SPEED2DIST(newSpeed - predSpeed) / iterations));
        // always reach stops with large time steps (Euler)
        if (iterations > 1 && newSpeed < EIDM_EST_REAC_THRESHOLD * TS && !lastrespectMinGap) {
            newSpeed = std::max(0.0, predSpeed + DIST2SPEED(current_gap - newGap) * iterations / (i + 1.));
        }
    }
    // the acceleration after the iterations
    acc = SPEED2ACCEL(std::min(newSpeed, cfMaxNextSpeed(v, egoSpeed)) - kin[v].speed);
    if (current) {
        if (vars->minaccel > wantedacc - NUMERICAL_EPS && vars->ap_update == 0) {
            // store the observation for the time steps until the next action point
            vars->v_est_l = predSpeed;
            vars->v_est = egoSpeed;
            vars->s_est = update == 2 ? current_gap + p[E_TREACTION] * egoSpeed : current_gap;
            vars->respectMinGap = respectMinGap;
        }
        if (vars->wouldacc > wouldacc) {
            vars->wouldacc = wouldacc;
        }
    }
    if (usage == cf::CalcReason::CURRENT && vars->realacc > acc && vars->minaccel <= wantedacc - NUMERICAL_EPS) {
        acc = vars->realacc;
        newSpeed = std::max(0.0, egoSpeed + ACCEL2SPEED(acc));
    }
    if (current && vars->minaccel > wantedacc - NUMERICAL_EPS) {
        // this call determines the acceleration update
        vars->minaccel = wantedacc;
        if (vars->realacc > acc) {
            vars->realacc = acc;
        }
        vars->realleaderacc = a_leader;
    }
    // for a potential update in finalizeSpeed when stopSpeed determined the speed
    if (own && vars->minaccel > wantedacc - NUMERICAL_EPS && usage == cf::CalcReason::CURRENT_WAIT && !respectMinGap) {
        vars->stop.push_back({acc, gap2pred});
    }
    return std::max(0., newSpeed);
}

double Sim::eidmFreeSpeed(int v, double speed, double seen, double maxSpeed, bool onInsertion, cf::CalcReason usage) const {
    if (maxSpeed < 0.) {
        return maxSpeed;
    }
    const VType& t = vt(v);
    const std::vector<double>& p = t.cfp;
    cf::CalcReason u = cfComputeLC ? cf::CalcReason::LANE_CHANGE : usage;
    int update = u == cf::CalcReason::LANE_CHANGE || u == cf::CalcReason::FUTURE ? 0 : 1;
    const CFVars* vars = cfv(v);
    if (onInsertion) {
        // whether insertion at this speed can reach maxSpeed within seen (MSCFModel_EIDM::freeSpeed)
        if (BALLISTIC) {
            // (no movement until the next step)
            const double d = seen - NUMERICAL_EPS;
            if (0.5 * (speed + maxSpeed) * 0 >= d) {
                return maxSpeed;
            }
            const double q = -2 * d * t.decel - maxSpeed * maxSpeed;
            return std::sqrt(-q);
        }
        return cf::freeSpeed(speed, t.decel, seen, maxSpeed, true);
    }
    if (maxSpeed < speed && seen < speed * (p[E_TPREVIEW] + p[E_TREACTION])) {
        // brake to the speed limit of the next lane / event
        update = update == 0 ? 0 : 2;
        const double remaining = std::max({(seen - speed * p[E_TREACTION]) / speed, p[E_TREACTION] / 2, TS});
        double targetDecel = (speed - maxSpeed) / remaining;
        if (remaining > p[E_TPREVIEW] - targetDecel / p[E_JERK_MAX]) {
            targetDecel = (p[E_TPREVIEW] - remaining) * p[E_JERK_MAX];
        }
        // internalsecuregap
        auto internalSecureGap = [&](double sp) {
            const double a = cf::currentAccel(t, sp);
            const double s = std::max(0.0, sp * t.tau + sp * sp / (2 * std::sqrt(a * t.decel)));
            if (sp <= vars->v0_int) {
                return std::sqrt((s * s) / std::max(targetDecel / a + 1.0, 1.0));
            }
            const double aFree = -t.decel * (1.0 - std::pow(vars->v0_int / sp, a * p[E_DELTA] / t.decel));
            return std::sqrt((s * s) / std::max(targetDecel / a + 1.0 + aFree / a, 1.0));
        };
        const double secGap = vars->ap_update == 0 || update == 0 ? internalSecureGap(speed)
                              : internalSecureGap(vars->v_est + vars->lastrealacc * vars->ap_update * TS);
        return eidmV(v, std::max(seen, secGap), speed, 0., vars->v0_int, true, update, u);
    }
    return eidmV(v, 500., speed, maxSpeed, vars->v0_int, true, update, u);
}

// MSCFModel_EIDM::internalspeedlimit: the internal desired speed adapts slowly to the upcoming speed limits and turns
void Sim::eidmInternalSpeedLimit(int v, double oldV) {
    const VType& t = vt(v);
    const double tpreview = t.cfp[E_TPREVIEW];
    CFVars* vars = cfv(v);
    double vLimCurr = vars->v0_int;
    const double vLimPrev = vars->v0_old;
    const Kin& k = kin[v];
    const std::vector<int>& conts = bestContinuation(v);
    int view = 1;
    int lane = k.lane;
    int link = succLinkSec(v, view, lane, conts);
    double seen = net.lanes[lane].length - k.pos;
    double v0 = laneVehMaxSpeed(lane, v);
    if (link >= 0 && (seen < oldV * tpreview || seen < v0 * tpreview / 2)) {
        double speedlim = 200;  // (kept from turn to turn)
        while (link >= 0 && (seen < oldV * tpreview || seen < v0 * tpreview / 2)) {
            switch (net.links[link].dir) {
                case 't':
                case 'T':
                    speedlim = 4;
                    break;
                case 'l':
                    speedlim = 8;
                    break;
                case 'r':
                    speedlim = 6;
                    break;
                case 'L':
                case 'R':
                    speedlim = 12;
                    break;
                default:
                    break;
            }
            if (v0 > speedlim * veh[v].speedFactor + NUMERICAL_EPS) {
                v0 = speedlim * veh[v].speedFactor;
            }
            if (net.links[link].via < 0) {
                ++view;
            }
            lane = net.links[link].viaOrLane();
            v0 = std::min(v0, laneVehMaxSpeed(lane, v));
            seen += net.lanes[lane].length;
            link = succLinkSec(v, view, lane, conts);
        }
    }
    auto eq = [](double a, double b) { return a < b + NUMERICAL_EPS && a > b - NUMERICAL_EPS; };
    if (!eq(vLimPrev, v0) || (eq(vLimPrev, v0) && !eq(vLimPrev, vLimCurr))) {
        if ((vLimCurr < vLimPrev + NUMERICAL_EPS && vLimCurr < v0 + NUMERICAL_EPS && vLimPrev > v0 - NUMERICAL_EPS)
                || (vLimCurr > vLimPrev - NUMERICAL_EPS && vLimCurr > v0 - NUMERICAL_EPS && vLimPrev < v0 + NUMERICAL_EPS)) {
            vars->v0_old = vLimCurr;  // the change of direction
        } else if (vLimCurr >= v0 - NUMERICAL_EPS) {
            vLimCurr = std::max(v0, vLimCurr - (vLimPrev - v0) * TS / tpreview);
        } else {
            vLimCurr = std::min(v0, vLimCurr - (vLimPrev - v0) * TS / tpreview);
        }
        if (eq(vLimCurr, v0)) {
            vars->v0_old = v0;
            vars->v0_int = v0;
        } else {
            vars->v0_int = vLimCurr;
        }
    }
}

double Sim::eidmFinalizeSpeed(int v, double vPos, std::vector<StopEvent>& stopEv) {
    const VType& t = vt(v);
    const std::vector<double>& p = t.cfp;
    CFVars* vars = cfv(v);
    const double oldV = kin[v].speed;
    const double accel = kin[v].accel;
    const double simTime = STEPS2TIME(now);
    const int iterations = (int)p[E_ITER];
    // the speed from a stopSpeed calculation: run _v again with its saved variables
    double vPosE = vPos;
    if ((vPos <= HALTING_SPEED && vPos <= oldV)
            || !(vPos > oldV + ACCEL2SPEED(vars->realacc) - NUMERICAL_EPS && vPos < oldV + ACCEL2SPEED(vars->realacc) + NUMERICAL_EPS)) {
        for (const std::pair<double, double>& st : vars->stop) {
            if (vPos > oldV + ACCEL2SPEED(st.first) - NUMERICAL_EPS && vPos < oldV + ACCEL2SPEED(st.first) + NUMERICAL_EPS) {
                vPosE = eidmV(v, st.second, oldV, 0, vars->v0_int, false, 1, cf::CalcReason::CURRENT);
            }
        }
    }
    const double vStop = std::min(vPosE, processNextStop(v, vPosE, stopEv));
    const double vMinEmergency = cfMinNextSpeedEmergency(v, oldV);
    const double vMin = std::min(cfMinNextSpeed(v, oldV), std::max(vPosE, vMinEmergency));
    double vMax = std::min(cfMaxNextSpeed(v, oldV), vStop);
    vMax = std::max(vMin, vMax);
    // patchSpeedBeforeLCEIDM: the driving error depends on the criticality of the situation
    const double Accel_vMax = cf::currentAccel(t, vMax);
    double s = std::max(0., vars->v_est * t.tau + vars->v_est * (vars->v_est - vars->v_est_l) / (2 * std::sqrt(Accel_vMax * t.decel)));
    s += vars->respectMinGap ? t.minGap + EIDM_POS_ACC_EPS : 0.05 + 0.20 * std::max(0.25, cf::currentAccel(t, 0.0)) + EIDM_POS_ACC_EPS;
    const double intensity = std::min({Accel_vMax, 1.5, std::max(vMax - 0.5 * Accel_vMax, 0.0)});
    const double criticality = std::min(std::max(s / vars->s_est - 0.5, -0.4), 0.0);
    const double drivingError = p[E_SIG_ERROR] * vars->w_error * intensity * (2.75 * 2.75 * criticality * criticality + 1.0);
    double vNext = std::max(vMin, vMax + ACCEL2SPEED(drivingError));
    if (BALLISTIC) {
        // (negative speeds indicate a stop within the step)
        vNext = std::max(lcPatchSpeed(v, vMin, vMax, vMax), vMin);
    } else {
    // adapt slowly to vNext
    vNext = lcPatchSpeed(v, vMin, vNext, vMax);
    if (vNext > oldV && oldV > EIDM_CLUTCH_ENGAGE_SPEED * 2 && vars->t_off + p[E_TACC_MAX] + NUMERICAL_EPS < simTime) {
        // the jerk is bounded (twice when still decelerating)
        const double jerk = p[E_JERK_MAX] * TS + accel < 0. ? p[E_JERK_MAX] * 2 : p[E_JERK_MAX];
        vNext = std::max(oldV + std::min(vNext - oldV, (jerk * TS + accel) * TS), 0.);
    } else if (vNext <= oldV && vNext < vMax - NUMERICAL_EPS && oldV > EIDM_CLUTCH_ENGAGE_SPEED * 2) {
        // braking for lane changes is limited, braking for safety reasons is not
        vNext = std::max(oldV + std::min(vMax - oldV, std::max(vNext - oldV, (-p[E_JERK_MAX] * TS + accel) * TS)), 0.);
    }
    }
    // startup delay before driving off, then the slow-to-start term
    const Time delay = TIME2STEPS(t.startupDelay);
    const Time since = cfVars[v].timeSinceStartup;
    Time addTime = vars->ap_update * DELTA_T;
    if (delay + addTime - (since - DELTA_T) < DELTA_T) {
        addTime = 0;
    }
    double vDelay = vNext;
    if (since > 0 && since - DELTA_T < delay + addTime) {
        const Time remaining = delay + addTime - (since - DELTA_T);
        vDelay = remaining >= DELTA_T ? 0 : (double)(DELTA_T - remaining) / (double)DELTA_T * vNext;
    }
    if (v == traceVeh) {
        std::fprintf(stderr, "%.2f eidm %s vPos=%.4f vMin=%.4f vMax=%.4f vNext=%.4f since=%lld ap=%d add=%lld vDelay=%.4f\n", STEPS2TIME(now),
                     veh[v].id.c_str(), vPos, vMin, vMax, vNext, (long long)since, vars->ap_update, (long long)addTime, vDelay);
    }
    // slowToStartTerm
    double result = vDelay;
    if (!(vDelay == 0 || vDelay <= oldV)) {
        const double remainingDelay = vDelay != vNext ? STEPS2TIME(DELTA_T - (delay - (since - DELTA_T))) : 0.0;
        double vCorr = oldV;
        for (int i = 0; i < iterations; i++) {
            const double tt = simTime - remainingDelay - TS * (iterations - i - 1.) / iterations;
            if (oldV < EIDM_CLUTCH_ENGAGE_SPEED && vars->t_off + 4. - NUMERICAL_EPS < tt && vars->ap_update == 0
                    && accel < std::min(cf::currentAccel(t, oldV) / 4, 0.2)) {
                vars->t_off = tt;  // activate the drive-off term
            }
            if (vars->t_off + p[E_TACC_MAX] + NUMERICAL_EPS > tt) {
                vCorr += (vDelay - oldV) / iterations * (std::tanh(((tt - vars->t_off) * 2. / p[E_TACC_MAX] - p[E_M_BEGIN]) * p[E_M_FLATNESS]) + 1.) / 2.;
            } else {
                vCorr += (vDelay - oldV) / iterations;
            }
        }
        result = vCorr;
    }
    vNext = result;
    eidmInternalSpeedLimit(v, oldV);
    if (vNext > EIDM_EST_REAC_THRESHOLD) {
        // the Wiener processes (RandHelper::randNorm of the global generator)
        uint64_t key = 0;
        auto u = [&]() { return rand01(opt.seed, (uint64_t)v, (uint64_t)step, RP_CF_STATE ^ mix64(++key)); };
        vars->w_gap = std::exp(-TS / p[E_TPERS_ESTIMATE]) * vars->w_gap + std::sqrt(2 * TS / p[E_TPERS_ESTIMATE]) * randNormPolar(0, 0.5, u);
        vars->w_speed = std::exp(-TS / p[E_TPERS_ESTIMATE]) * vars->w_speed + std::sqrt(2 * TS / p[E_TPERS_ESTIMATE]) * randNormPolar(0, 0.5, u);
        vars->w_error = std::exp(-TS / p[E_TPERS_DRIVE]) * vars->w_error + std::sqrt(2 * TS / p[E_TPERS_DRIVE]) * randNormPolar(0, 1, u);
    }
    // the action point (reaction time)
    if (vars->ap_update == 0) {
        vars->lastacc = vars->minaccel;
        vars->wouldacc = vars->minaccel;
        vars->lastrealacc = vars->realacc;
        vars->lastleaderacc = vars->realleaderacc;
    }
    if (double(vars->ap_update) >= double(p[E_TREACTION] / TS - 1 - NUMERICAL_EPS)) {
        vars->ap_update = 0;
    } else {
        vars->ap_update++;
    }
    // react immediately to strong braking needs
    if ((vars->wouldacc - vars->lastacc) < -p[E_EPSILON_ACC] || vars->wouldacc < -t.emergencyDecel || (oldV < EIDM_EST_REAC_THRESHOLD && vNext < oldV)) {
        vars->ap_update = 0;
    }
    vars->minaccel = 100;
    vars->realacc = 100;
    vars->realleaderacc = 100;
    vars->stop.clear();
    return vNext;
}

}  // namespace ps
