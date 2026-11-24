// Traffic lights (static and actuated) and the induction loops that drive
// actuation. Ported from SUMO 1.28: NLJunctionControlBuilder::closeTrafficLightLogic
// (initial phase from the offset), MSTrafficLightLogic::SwitchCommand,
// MSSimpleTrafficLightLogic::trySwitch, MSActuatedTrafficLightLogic (init,
// trySwitch, gapControl, duration; the default algorithm without 'next'
// phases, switching rules or per-link min/max green) and MSInductLoop
// (notifyMove, notifyEnter, notifyLeave, getTimeSinceLastDetection).
//
// Detector updates are produced as events in the move phase (each lane task
// writes its own buffer) and applied afterwards in a fixed order.
#include <cstdio>
#include <set>

#include "model.hpp"

namespace ps {

constexpr double DEFAULT_LENGTH_WITH_GAP = 7.5;

void Sim::setTLSignals(int t) {
    const TLSDef& d = net.tls[t];
    const std::string& state = d.phases[tlsRT[t].step].state;
    for (int i = 0; i < (int)d.links.size(); ++i) {
        for (int l : d.links[i]) {
            Link& k = net.links[l];
            const char ls = state[i];
            if (k.state != ls) {
                k.lastStateChange = now;
            }
            k.state = ls;
            if (k.haveGreen()) {
                k.lastGreenState = k.state;
            }
        }
    }
}

static bool weakConflict(const Network& net, const TLSDef& d, int tlIndex, const std::string& state) {
    for (int link : d.links[tlIndex]) {
        const int linkIndex = net.links[link].index;
        const int junction = net.links[link].junction;
        for (int i = 0; i < (int)d.links.size(); i++) {
            if (i == tlIndex) {
                continue;
            }
            if (state[i] == LS_GREEN_MAJOR || state[i] == LS_GREEN_MINOR) {
                for (int foe : d.links[i]) {
                    if (junction == net.links[foe].junction) {
                        const Junction& J = net.junctions[junction];
                        if (J.foesTest(linkIndex, net.links[foe].index) && net.links[foe].permissions != 0
                                && net.lanes[net.links[foe].laneBefore].edge != net.lanes[net.links[link].laneBefore].edge) {
                            return false;
                        }
                    }
                }
            }
        }
    }
    return true;
}

void Sim::initTLS() {
    {
        // MSTLLogicControl::switchOffAll (--tls.all-off: all logics but rail signals) and the program "off":
        // MSOffTrafficLightLogic, whose single phase has the 'off' states of the links (blinking for minor links,
        // all-way stop, otherwise no signal)
        for (TLSDef& d : net.tls) {
            if (d.type == "rail_signal" || (!opt.tlsAllOff && d.type != "off")) {
                continue;
            }
            std::string state;
            for (const std::vector<int>& ls : d.links) {
                bool minor = false, allway = false;
                for (int l : ls) {
                    minor |= net.links[l].offState == LS_OFF_BLINKING;
                    allway |= net.links[l].offState == LS_ALLWAY_STOP;
                }
                state += allway ? (char)LS_ALLWAY_STOP : minor ? (char)LS_OFF_BLINKING : (char)LS_OFF_NOSIGNAL;
            }
            d.type = "static";
            d.offset = 0;
            d.phases = {Phase{TIME2STEPS(3600), TIME2STEPS(3600), TIME2STEPS(3600), state}};
        }
    }
    tlsRT.assign(net.tls.size(), TLSRT());
    laneLoops.assign(net.lanes.size(), {});
    for (int t = 0; t < (int)net.tls.size(); ++t) {
        const TLSDef& d = net.tls[t];
        TLSRT& r = tlsRT[t];
        if (d.type == "rail_signal") {
            r.nextSwitch = TIME_MAX;  // switched by rail.cpp
            continue;
        }
        if (d.type == "rail_crossing") {
            initRailCrossing(t);
            continue;
        }
        r.lastSwitch.assign(d.phases.size(), 0);
        r.lastEnd.assign(d.phases.size(), 0);
        // initial step and first switch (NLJunctionControlBuilder::closeTrafficLightLogic)
        Time absDuration = 0;
        for (const Phase& p : d.phases) {
            absDuration += p.duration;
        }
        Time offset;
        if (d.offset >= 0) {
            offset = (now + absDuration - (d.offset % absDuration)) % absDuration;
        } else {
            offset = (now + ((-d.offset) % absDuration)) % absDuration;
        }
        int step = 0;
        while (offset >= d.phases[step].duration) {
            offset -= d.phases[step].duration;
            step++;
        }
        r.step = step;
        r.nextSwitch = d.phases[step].duration - offset + now;
        r.lastSwitch[step] = now;
        // MSTLLogicControl::TLSLogicVariants::addLogic: links whose state in the net file differs from the
        // initial phase change state at the begin time (MSLink::setTLState)
        setTLSignals(t);
        if (d.type != "actuated") {
            continue;
        }
        // MSActuatedTrafficLightLogic constructor / init; the first switch is
        // scheduled after the minimum duration of the initial phase
        r.nextSwitch = d.phases[step].minDur + now;
        r.actuated = true;
        r.maxGap = std::stod(d.param("max-gap", "3.0"));
        r.jamThreshold = std::stod(d.param("jam-threshold", "-1"));
        r.passingTime = std::stod(d.param("passing-time", "1.9"));
        r.detectorGap = std::stod(d.param("detector-gap", "2.0"));
        r.inactiveThreshold = TIME2STEPS(std::stod(d.param("inactive-threshold", "180")));
        const double detLength = std::stod(d.param("detector-length", "0"));
        const int numLinks = (int)d.links.size();
        auto minimumMinDuration = [&](int lane) {
            Time result = std::numeric_limits<Time>::max();
            for (int pI = 0; pI < (int)d.phases.size(); pI++) {
                const Phase& ph = d.phases[pI];
                for (int i = 0; i < (int)ph.state.size(); i++) {
                    if ((ph.state[i] == LS_GREEN_MAJOR || ph.state[i] == LS_GREEN_MINOR) && i < numLinks) {
                        for (int cand : d.lanes[i]) {
                            if (cand == lane && ph.isActuated()) {
                                result = std::min(result, ph.minDur);
                            }
                        }
                    }
                }
            }
            return result;
        };
        std::vector<std::pair<int, int>> laneLoop;  // (lane, loop info index)
        auto loopOfLane = [&](int lane) {
            for (const auto& p : laneLoop) {
                if (p.first == lane) {
                    return p.second;
                }
            }
            return -1;
        };
        for (int i = 0; i < numLinks; ++i) {
            for (int lane : d.lanes[i]) {
                // noVehicles: no detectors on green verges, sidewalks and walking areas
                if ((net.lanes[lane].permissions & VC_ALL) == 0 || (net.lanes[lane].permissions & VC_ALL) == VC_PEDESTRIAN) {
                    continue;
                }
                if (loopOfLane(lane) >= 0) {
                    continue;
                }
                const Time minDur = minimumMinDuration(lane);
                if (minDur == std::numeric_limits<Time>::max()) {
                    continue;
                }
                const double length = net.lanes[lane].length;
                const double speed = net.lanes[lane].speed;
                const double inductLoopPosition = std::min(r.detectorGap * speed, (STEPS2TIME(minDur) / r.passingTime + 0.5) * DEFAULT_LENGTH_WITH_GAP);
                double ilpos = length - inductLoopPosition;
                int placementLane = lane;
                while (ilpos < 0 && net.lanes[placementLane].incoming.size() == 1
                        && net.links[net.correspondingEntryLink(net.lanes[placementLane].incoming.front().link)].tls < 0) {
                    placementLane = net.lanes[placementLane].logicalPred;
                    ilpos += net.lanes[placementLane].length;
                }
                if (ilpos < 0) {
                    ilpos = 0;
                }
                InductLoop loop;
                loop.lane = placementLane;
                loop.pos = ilpos;
                loop.endPos = ilpos + detLength;
                loop.lastLeaveTime = STEPS2TIME(now);
                const int li = (int)loops.size();
                loops.push_back(loop);
                laneLoops[placementLane].push_back(li);
                InductLoopInfo info;
                info.loop = li;
                info.lane = lane;
                info.maxGap = r.maxGap;
                info.jamThreshold = r.jamThreshold;
                info.servedPhase.assign(d.phases.size(), false);
                laneLoop.push_back({lane, (int)r.loops.size()});
                r.loops.push_back(info);
            }
        }
        // assign loops to phases
        std::vector<bool> neverMajor(numLinks, true);
        for (const Phase& ph : d.phases) {
            for (int i = 0; i < numLinks; i++) {
                if (ph.state[i] == LS_GREEN_MAJOR) {
                    neverMajor[i] = false;
                }
            }
        }
        std::vector<bool> oneLane(numLinks, false);
        std::vector<bool> turnaround(numLinks, true);
        for (int i = 0; i < numLinks; i++) {
            for (int lane : d.lanes[i]) {
                int numMotorized = 0;
                for (int l : net.edges[net.lanes[lane].edge].lanes) {
                    if (net.lanes[l].permissions != 0) {
                        numMotorized++;
                    }
                }
                if (numMotorized == 1) {
                    oneLane[i] = true;
                    break;
                }
            }
            for (int link : d.links[i]) {
                if (!net.links[link].isTurnaround()) {
                    turnaround[i] = false;
                    break;
                }
            }
        }
        auto hasMajor = [&](const std::string& state, const std::vector<int>& lanesAt) {
            for (int i = 0; i < (int)state.size() && i < numLinks; i++) {
                if (state[i] == LS_GREEN_MAJOR) {
                    for (int cand : d.lanes[i]) {
                        for (int lane : lanesAt) {
                            if (lane == cand) {
                                return true;
                            }
                        }
                    }
                }
            }
            return false;
        };
        for (int phaseIndex = 0; phaseIndex < (int)d.phases.size(); ++phaseIndex) {
            const Phase& ph = d.phases[phaseIndex];
            std::vector<int> loopsForPhase;
            if (ph.isActuated()) {
                std::set<int> greenLinks, greenLinksPermissive;
                std::vector<std::pair<int, std::set<int>>> loopLinks;  // (loop info index, links)
                auto loopLinksOf = [&](int li) -> std::set<int>& {
                    for (auto& p : loopLinks) {
                        if (p.first == li) {
                            return p.second;
                        }
                    }
                    loopLinks.push_back({li, {}});
                    return loopLinks.back().second;
                };
                for (int i = 0; i < numLinks; i++) {
                    if (ph.state[i] == LS_GREEN_MAJOR) {
                        greenLinks.insert(i);
                    } else if (ph.state[i] == LS_GREEN_MINOR) {
                        if (((neverMajor[i] || turnaround[i]) && hasMajor(ph.state, d.lanes[i])) || oneLane[i] || weakConflict(net, d, i, ph.state)) {
                            greenLinks.insert(i);
                        } else {
                            greenLinksPermissive.insert(i);
                        }
                    }
                    for (int lane : d.lanes[i]) {
                        const int li = loopOfLane(lane);
                        if (li >= 0) {
                            loopLinksOf(li).insert(i);
                        }
                    }
                }
                for (auto& item : loopLinks) {
                    const InductLoopInfo& info = r.loops[item.first];
                    bool usable = true;
                    bool foundUsable = false;
                    for (int j : item.second) {
                        if (greenLinks.count(j) == 0 && (info.jamThreshold <= 0 || greenLinksPermissive.count(j) == 0)) {
                            usable = false;
                        } else {
                            foundUsable = true;
                        }
                    }
                    if (!usable && foundUsable && info.jamThreshold > 0) {
                        usable = true;
                    }
                    if (usable && info.jamThreshold <= 0) {
                        for (int link : net.lanes[info.lane].links) {
                            if (net.links[link].isTurnaround()) {
                                continue;
                            }
                            const int next = net.links[link].lane;
                            const int nextLoop = loopOfLane(next);
                            if (nextLoop >= 0) {
                                for (int j : loopLinksOf(nextLoop)) {
                                    if (greenLinks.count(j) == 0) {
                                        usable = false;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                    if (usable) {
                        loopsForPhase.push_back(item.first);
                    }
                }
                // MSActuatedTrafficLightLogic iterates a std::set<MSInductLoop*>; use loop creation order
                std::sort(loopsForPhase.begin(), loopsForPhase.end());
            }
            for (int li : loopsForPhase) {
                r.loops[li].servedPhase[phaseIndex] = true;
            }
            r.loopsForPhase.push_back(loopsForPhase);
        }
    }
}

double Sim::timeSinceLastDetection(int loop) const {
    const InductLoop& l = loops[loop];
    if (!l.onDet.empty()) {
        return 0;
    }
    return STEPS2TIME(now) - l.lastLeaveTime;
}

// MSActuatedTrafficLightLogic::trySwitch / MSSimpleTrafficLightLogic::trySwitch
Time Sim::tlsTrySwitch(int t) {
    const TLSDef& d = net.tls[t];
    TLSRT& r = tlsRT[t];
    if (r.railCrossing) {
        return railCrossingUpdate(t);
    }
    if (!r.actuated) {
        r.step = (r.step + 1) % (int)d.phases.size();
        r.lastSwitch[r.step] = now;
        return d.phases[r.step].duration;
    }
    r.lastTrySwitchTime = now;
    const int origStep = r.step;
    Time actDuration = now - r.lastSwitch[r.step];
    const Phase& cur = d.phases[r.step];
    // gapControl
    double detectionGap = DMAX;
    if (cur.isGreenPhase() && !(actDuration >= cur.maxDur)) {
        for (int li : r.loopsForPhase[r.step]) {
            const double actualGap = timeSinceLastDetection(r.loops[li].loop);
            if (actualGap < r.loops[li].maxGap) {
                detectionGap = std::min(detectionGap, actualGap);
            }
        }
    }
    if (detectionGap < DMAX) {
        // duration()
        Time newDuration = cur.minDur - actDuration;
        newDuration = std::max(std::max(newDuration, TIME2STEPS(r.detectorGap - detectionGap)), (Time)1);
        if (newDuration % 1000 != 0) {
            const Time totalDur = newDuration + actDuration;
            newDuration = (totalDur / 1000 + 1) * 1000 - actDuration;
        }
        newDuration = std::min(newDuration, cur.maxDur - actDuration);
        return newDuration;
    }
    const int nextStep = (r.step + 1) % (int)d.phases.size();
    r.step = nextStep;
    if (r.step != origStep) {
        r.lastEnd[origStep] = now;
        r.lastSwitch[r.step] = now;
        actDuration = 0;
    }
    if (d.phases[r.step].isGreenPhase()) {
        for (int li : r.loopsForPhase[r.step]) {
            r.loops[li].lastGreenTime = now;
        }
    }
    const Time minRetry = r.step != origStep ? 0 : TIME2STEPS(1);
    return std::max(minRetry, d.phases[r.step].minDur - actDuration);
}

// begin-of-step traffic light switches (MSTrafficLightLogic::SwitchCommand::execute)
// the switch commands are begin-of-step events: MSEventControl::execute runs an event scheduled within the
// current step and reschedules it at its scheduled time plus the returned offset
void Sim::tlsStep() {
    for (int t = 0; t < (int)tlsRT.size(); ++t) {
        TLSRT& r = tlsRT[t];
        while (r.nextSwitch < now + DELTA_T) {
            const int step1 = r.step;
            Time next = tlsTrySwitch(t);
            while (next == 0) {
                next = tlsTrySwitch(t);
            }
            if (r.step != step1) {
                setTLSignals(t);
            }
            r.nextSwitch += next;
        }
    }
}

// ---------------------------------------------------------------------------
// induction loops
// ---------------------------------------------------------------------------
// the detectors of `lane` that are move reminders of vehicle v (see Veh::aliasLane)
const std::vector<int>& Sim::loopsOf(int v, int lane) const {
    const Veh& x = veh[v];
    return lane == x.aliasLane ? laneLoops[x.aliasLoopLane] : laneLoops[lane];
}

// MSInductLoop::notifyMove; positions are relative to `lane`. The move phase
// runs at simulation time `now` (SIMTIME)
void Sim::loopNotifyMove(int v, int lane, double oldPos, double newPos, double newSpeed, std::vector<DetEvent>& ev) const {
    const double len = vt(v).length;
    for (int li : loopsOf(v, lane)) {
        const InductLoop& d = loops[li];
        if (newPos < d.pos) {
            continue;
        }
        if (newPos >= d.pos && oldPos < d.pos) {
            const double timeBeforeEnter = cf::passingTime(oldPos, d.pos, newPos, veh[v].prevSpeed, newSpeed);
            ev.push_back({li, v, 0, STEPS2TIME(now) + timeBeforeEnter});
        }
        const double oldBackPos = oldPos - len;
        const double newBackPos = newPos - len;
        if (newBackPos > d.endPos) {
            if (oldBackPos <= d.endPos) {
                ev.push_back({li, v, 1, STEPS2TIME(now) + cf::passingTime(oldBackPos, d.endPos, newBackPos, veh[v].prevSpeed, newSpeed)});
            } else {
                ev.push_back({li, v, 3, 0});
            }
        }
    }
}

// MSInductLoop::notifyEnter for reasons other than passing a junction (lane change, departure,
// leaving a parking stop, teleporting onto the next edge); the vehicle state is the new one
void Sim::loopNotifyEnter(int v, int lane, std::vector<DetEvent>& ev) const {
    for (int li : loopsOf(v, lane)) {
        const InductLoop& d = loops[li];
        if (backPositionOnLane(v, lane) >= d.pos) {
            continue;
        }
        if (kin[v].pos >= d.pos) {
            ev.push_back({li, v, 0, STEPS2TIME(now)});
        }
    }
}

// MSInductLoop::notifyLeave for reasons other than passing a junction
void Sim::loopNotifyLeave(int v, int lane, std::vector<DetEvent>& ev) const {
    for (int li : loopsOf(v, lane)) {
        ev.push_back({li, v, 2, STEPS2TIME(now) + TS});
    }
}

// loopNotifyEnter in sequential phases (insertion, vehicle transfer)
void Sim::loopNotifyEnterNow(int v, int lane) {
    if (loopsOf(v, lane).empty()) {
        return;
    }
    std::vector<DetEvent> ev;
    loopNotifyEnter(v, lane, ev);
    applyDetEvents(ev);
}

void Sim::applyDetEvents(std::vector<DetEvent>& ev) {
    for (const DetEvent& e : ev) {
        InductLoop& d = loops[e.det];
        auto it = std::find_if(d.onDet.begin(), d.onDet.end(), [&](const std::pair<int, double>& p) { return p.first == e.veh; });
        if (e.kind == 0) {
            if (it == d.onDet.end()) {
                d.onDet.push_back({e.veh, e.time});
            } else {
                it->second = e.time;
            }
        } else if (e.kind == 1 || e.kind == 2) {
            if (it != d.onDet.end()) {
                d.onDet.erase(it);
                d.lastLeaveTime = e.time;
            }
        } else if (it != d.onDet.end()) {
            d.onDet.erase(it);
        }
    }
    ev.clear();
}

// ---------------------------------------------------------------------------
// rail crossings (MSRailCrossing)
// ---------------------------------------------------------------------------
void Sim::initRailCrossing(int t) {
    TLSDef& d = net.tls[t];
    TLSRT& r = tlsRT[t];
    r.railCrossing = true;
    r.timeGap = TIME2STEPS(std::stod(d.param("time-gap", "15")));
    r.spaceGap = std::stod(d.param("space-gap", "-1"));  // the time gap is used by default
    r.minGreenTime = TIME2STEPS(std::stod(d.param("min-green", "5")));
    r.openingDelay = TIME2STEPS(std::stod(d.param("opening-delay", "3")));
    r.openingTime = TIME2STEPS(std::stod(d.param("opening-time", "3")));  // red-yellow while opening
    r.yellowTime = TIME2STEPS(std::stod(d.param("yellow-time", "5")));
    const size_t n = d.links.size();
    d.phases = {Phase{1, 1, 1, std::string(n, LS_GREEN_MAJOR)}, Phase{r.yellowTime, r.yellowTime, r.yellowTime, std::string(n, LS_YELLOW_MINOR)},
                Phase{1, 1, 1, std::string(n, LS_RED)}, Phase{r.openingTime, r.openingTime, r.openingTime, std::string(n, LS_REDYELLOW)}};
    r.step = 0;
    r.lastSwitch.assign(d.phases.size(), now);
    r.lastEnd.assign(d.phases.size(), 0);
    railCrossingUpdate(t);
    setTLSignals(t);
    r.nextSwitch = now;  // the switch command starts at the begin
}

// MSRailCrossing::updateCurrentPhase: the time until the next try
Time Sim::railCrossingUpdate(int t) {
    const TLSDef& d = net.tls[t];
    TLSRT& r = tlsRT[t];
    Time stayRedUntil = now;
    // the approaching trains decide whether and how long the crossing must stay closed
    for (int link : d.railLinks) {
        for (const Approach& a : approaching[link]) {
            // (an unknown leaving time, SUMOTime_MAX, overflows in SUMO and has no effect)
            if (a.arrivalTime != TIME_MAX && a.arrivalTime - r.yellowTime - now < r.timeGap && a.leavingTime != TIME_MAX) {
                stayRedUntil = std::max(stayRedUntil, a.leavingTime + r.openingDelay);
            }
            if (r.spaceGap >= 0 && a.dist < r.spaceGap && a.leavingTime != TIME_MAX) {
                stayRedUntil = std::max(stayRedUntil, a.leavingTime + r.openingDelay);
            }
        }
        const int via = net.links[link].via;
        if (via >= 0 && laneVehicleNumberWithPartials(via) > 0) {
            // no opening while a train is on the crossing
            stayRedUntil = std::max(stayRedUntil, now + DELTA_T + r.openingDelay);
        }
    }
    const Time wait = stayRedUntil - now;
    const int oldStep = r.step;
    Time next;
    if (r.step == 0) {
        // 'G': can the crossing stay open
        if (wait == 0) {
            next = DELTA_T;
        } else {
            r.step++;
            next = r.yellowTime;
        }
    } else if (r.step == 1) {
        // 'y': the yellow time is over
        r.step++;
        next = std::max(DELTA_T, wait);
    } else if (r.step == 2) {
        // 'r': may it open again
        if (wait == 0) {
            r.step++;
            next = r.openingTime;
        } else {
            next = wait;
        }
    } else {
        // 'u': the opening time is over
        if (wait == 0) {
            r.step = 0;
            next = r.minGreenTime;
        } else {
            // a train approached during the opening, close again
            r.step = 2;
            next = wait;
        }
    }
    if (r.step != oldStep) {
        r.lastSwitch[r.step] = now;
    }
    static const bool traceRC = getenv("PSIM_TRACE_RC") != nullptr;
    if (traceRC) {
        std::fprintf(stderr, "%.1f rc %s step %d->%d wait=%lld next=%lld\n", STEPS2TIME(now), d.id.c_str(), oldStep, r.step, (long long)wait, (long long)next);
    }
    return next;
}

}  // namespace ps
