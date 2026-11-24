// psim2 command line.
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>

#include "model.hpp"

// a list of vehicle classes (comma or space separated)
static ps::SVCPermissions parseClasses(const std::string& s) {
    ps::SVCPermissions p = 0;
    std::string cur;
    for (char c : s + ",") {
        if (c == ',' || c == ' ') {
            if (!cur.empty()) {
                p |= ps::vclassFromName(cur);
            }
            cur.clear();
        } else {
            cur += c;
        }
    }
    return p;
}


int main(int argc, char** argv) {
    auto printUsage = [](FILE* out) {
        std::fprintf(out, "usage: psim2 <scenario> [--seed N] [--threads N] [--begin S] [--end S] [--out PREFIX]\n"
                          "  [--time-to-teleport S] [--ignore-junction-blocker S] [--fcd] [--stop-output] [--no-lc-output] [--gzip]\n"
                          "  [--deterministic-speedfactor] [--trace VEH] [--sumo-order]\n"
                          "  [--edgedata-period S] [--summary-period S (0 = off)] [--edits FILE] [--lc-levels|--lc-single]\n"
                          "  [--pedestrian.<SUMO option> VALUE] [--device.rerouting.<option> VALUE] [--weights.minor-penalty S]\n"
                          "  [--weights.turnaround-penalty S]\n");
    };
    if (argc < 2) {
        printUsage(stderr);
        return 1;
    }
    if (std::strcmp(argv[1], "-h") == 0 || std::strcmp(argv[1], "--help") == 0) {
        printUsage(stdout);
        return 0;
    }
    try {
        ps::Options o;
        for (int i = 2; i < argc; ++i) {
            const std::string a = argv[i];
            auto next = [&]() {
                if (i + 1 >= argc) {
                    throw std::runtime_error("missing value for option " + a);
                }
                return std::string(argv[++i]);
            };
            if (a == "--seed") o.seed = std::stoull(next());
            else if (a == "--threads") o.threads = std::stoi(next());
            else if (a == "--begin") o.begin = std::stod(next());
            else if (a == "--end") o.end = std::stod(next());
            else if (a == "--out") o.outPrefix = next();
            else if (a == "--time-to-teleport") o.timeToTeleport = std::stod(next());
            else if (a == "--ignore-junction-blocker") o.ignoreJunctionBlocker = std::stod(next());
            else if (a == "--fcd") o.fcd = true;
            else if (a == "--tripinfo-unfinished") o.tripinfoUnfinished = true;
            else if (a == "--with-taz") o.withTaz = true;
            else if (a == "--stop-output") o.stopOutput = true;
            else if (a == "--railsignal-block-output") o.railsignalBlockOutput = true;
            else if (a == "--deadlock-output") o.deadlockOutput = true;
            else if (a == "--railsignal-vehicle-output") o.railsignalVehicleOutput = true;
            else if (a == "--no-internal-links") {}  // (the converted network has no internal lanes, see psim2net.py)
            else if (a == "--no-lc-output") o.lcOutput = false;
            else if (a == "--gzip") o.gzip = true;
            else if (a == "--deterministic-speedfactor") o.deterministicSpeedFactor = true;
            else if (a == "--trace") o.trace = next();
            else if (a == "--sumo-order") o.sumoOrder = true;
            else if (a == "--step-method.ballistic") {
                ps::BALLISTIC = true;
                o.ballisticSet = true;
            } else if (a == "--default.action-step-length") {
                o.defaultActionStepLength = std::stod(next());
                o.defaultActionStepLengthSet = true;
            }
            else if (a == "--lc-levels") o.lcLevels = true;
            else if (a == "--lc-single") o.lcSingle = true;
            else if (a == "--lc-dyncolor") o.lcDynColor = true;
            else if (a == "--edgedata-period") o.edgedataPeriod = std::stod(next());
            else if (a == "--edits") o.editsFile = next();
            else if (a == "--extrapolate-departpos") o.extrapolateDepartPos = true;
            else if (a == "--summary-period") o.summaryPeriod = std::stod(next());
            else if (a.rfind("--pedestrian.", 0) == 0) o.pedOptions.emplace_back(a.substr(2), next());
            else if (a == "--device.rerouting.probability") o.rerouteProb = std::stod(next());
            else if (a == "--device.rerouting.period") o.reroutePeriod = std::stod(next());
            else if (a == "--device.rerouting.railsignal") o.rerouteRailSignal = true;
            else if (a == "--device.rerouting.explicit") {
                std::stringstream ss(next());
                std::string id;
                while (std::getline(ss, id, ',')) {
                    o.rerouteExplicit.push_back(id);
                }
            }
            else if (a == "--device.rerouting.pre-period") o.reroutePrePeriod = std::stod(next());
            else if (a == "--route-steps") o.routeSteps = std::stod(next());
            else if (a == "--device.rerouting.adaptation-interval") o.adaptInterval = std::stod(next());
            else if (a == "--device.rerouting.adaptation-steps") o.adaptSteps = std::stoi(next());
            else if (a == "--device.rerouting.adaptation-weight") o.adaptWeight = std::stod(next());
            else if (a == "--device.rerouting.threshold.factor") o.rerouteThresholdFactor = std::stod(next());
            else if (a == "--device.rerouting.mode") o.routingMode = std::stoi(next());
            else if (a == "--sumo-seed") o.sumoSeed = std::stoi(next());
            else if (a == "--weights.random-factor") o.weightsRandomFactor = std::stod(next());
            else if (a == "--weights.random-factor.dynamic") o.weightsRandomDynamic = true;
            else if (a == "--weights.priority-factor") o.weightsPriorityFactor = std::stod(next());
            else if (a == "--weights.separate-turns") o.weightsSeparateTurns = std::stod(next());
            else if (a == "--device.rerouting.init-with-loaded-weights") o.initWithLoadedWeights = true;
            else if (a == "--device.rerouting.bike-speeds") o.bikeSpeeds = true;
            else if (a == "--device.rerouting.threshold.constant") o.rerouteThresholdTime = std::stod(next());
            else if (a == "--device.rerouting.synchronize") o.rerouteSynchronize = next() == "true";
            else if (a == "--weights.minor-penalty") o.minorPenalty = std::stod(next());
            else if (a == "--ignore-route-errors") o.ignoreRouteErrors = true;
            else if (a == "--emergency-insert") o.emergencyInsert = true;
            else if (a == "--lanechange.overtake-right") o.overtakeRight = true;
            else if (a == "--tls.all-off") o.tlsAllOff = true;
            else if (a == "--use-stop-started") o.useStopStarted = true;
            else if (a == "--time-to-teleport.remove") o.removeGridlocked = true;
            else if (a == "--lanechange.duration") o.laneChangeDuration = ps::TIME2STEPS(std::stod(next()));
            else if (a == "--lateral-resolution") o.lateralResolution = std::stod(next());
            else if (a == "--max-depart-delay") o.maxDepartDelay = ps::TIME2STEPS(std::stod(next()));
            else if (a == "--max-num-vehicles") o.maxNumVehicles = std::stoi(next());
            else if (a == "--collision.mingap-factor") o.collisionMinGapFactor = std::stod(next());
            else if (a == "--collision.check-junctions") o.checkJunctions = true;
            else if (a == "--collision.check-junctions.mingap") o.checkJunctionsMinGap = std::stod(next());
            else if (a == "--startup-wait-threshold") o.startupWaitThreshold = ps::TIME2STEPS(std::stod(next()));
            else if (a == "--persontrip.transfer.car-walk" || a == "--persontrip.transfer.taxi-walk"
                     || a == "--persontrip.transfer.walk-taxi") {
                std::vector<std::string>& list = a == "--persontrip.transfer.car-walk" ? o.carWalk
                                                 : (a == "--persontrip.transfer.taxi-walk" ? o.taxiWalk : o.walkTaxi);
                list.clear();
                std::string v = next();
                std::replace(v.begin(), v.end(), ',', ' ');
                std::stringstream ss(v);
                std::string item;
                while (ss >> item) {
                    list.push_back(item);
                }
            }
            else if (a == "--persontrip.taxi.waiting-time") o.taxiWait = std::stod(next());
            else if (a == "--persontrip.walkfactor") o.walkFactor = std::stod(next());
            else if (a == "--persontrip.walk-opposite-factor") o.walkOppositeFactor = std::stod(next());
            else if (a == "--persontrip.ride-public-line") o.ridePublicLine = next() == "true";
            else if (a == "--weights.turnaround-penalty") o.turnaroundPenalty = std::stod(next());
            else if (a == "--time-to-teleport.railsignal-deadlock") o.timeToTeleportRSDeadlock = std::stod(next());
            else if (a == "--time-to-teleport.remove-constraint") o.removeConstraint = true;
            else if (a == "--eager-insert") o.eagerInsert = true;
            else if (a == "--insertion-checks") o.insertionChecks = ps::parseInsertionChecks(next());
            else if (a == "--collision.action") {
                o.collisionAction = next();
                if (o.collisionAction != "teleport" && o.collisionAction != "warn" && o.collisionAction != "none") {
                    throw std::runtime_error("unsupported collision.action " + o.collisionAction);
                }
            }
            else if (a == "--railsignal.max-block-length") o.railMaxBlockLength = std::stod(next());
            else if (a == "--railsignal.moving-block.max-dist") o.railMovingBlockMaxDist = std::stod(next());
            else if (a == "--railway.max-train-length") o.railMaxTrainLength = std::stod(next());
            else if (a == "--weights.reversal-penalty") o.reversalPenalty = std::stod(next());
            else if (a == "--railsignal-moving-block") o.railMovingBlock = true;
            else if (a == "--railsignal.default-classes") o.railSignalizedClasses = parseClasses(next());
            else if (a == "--railsignal.moving-block.default-classes") o.railMovingBlockClasses = parseClasses(next());
            else {
                std::fprintf(stderr, "unknown option %s\n", a.c_str());
                return 1;
            }
        }
        if (o.sumoOrder) {
            o.threads = 1;  // validation mode: SUMO's sequential processing order
        }
        ps::Sim sim(o);
        sim.load(argv[1]);
        sim.run();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    return 0;
}
