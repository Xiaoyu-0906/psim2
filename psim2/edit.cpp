// Runtime network edits, applied single-threaded at the start of a step
// (before any parallel phase), like TraCI commands between two SUMO steps.
// Supported so far: lane speed limit (traci.lane.setMaxSpeed), lane
// permissions (traci.lane.setAllowed) and the traffic light program.
// Edits that change junction topology need the right-of-way logic of the
// edited junction to be recomputed (netconvert's job in SUMO) and are not
// supported yet.
//
// Edits file (--edits), one edit per line:
//   TIME LANE_SPEED lane speed
//   TIME LANE_PERMISSIONS lane class[,class...]   (or "none")
#include <fstream>
#include <sstream>

#include "model.hpp"

namespace ps {

static SVCPermissions classMask(const std::string& list) {
    if (list == "none") {
        return 0;
    }
    if (list == "all") {
        return VC_ALL;
    }
    SVCPermissions m = 0;
    std::stringstream ss(list);
    std::string c;
    while (std::getline(ss, c, ',')) {
        m |= vclassFromName(c);
    }
    return m;
}

void Sim::loadEdits(const std::string& file) {
    std::ifstream in(file);
    if (!in) {
        throw std::runtime_error("cannot open " + file);
    }
    std::string line;
    while (std::getline(in, line)) {
        std::istringstream ls(line);
        std::string t, kind, target, value;
        if (!(ls >> t) || t[0] == '#') {
            continue;
        }
        ls >> kind >> target >> value;
        EditRequest e;
        e.target = net.laneIdx.at(target);
        if (kind == "LANE_SPEED") {
            e.kind = EditRequest::LANE_SPEED;
            e.value = std::stod(value);
        } else if (kind == "LANE_PERMISSIONS") {
            e.kind = EditRequest::LANE_PERMISSIONS;
            e.value = classMask(value);
        } else {
            throw std::runtime_error("unsupported edit " + kind);
        }
        scheduledEdits.push_back({TIME2STEPS(std::stod(t)), e});
    }
    std::stable_sort(scheduledEdits.begin(), scheduledEdits.end(),
                     [](const auto& a, const auto& b) { return a.first < b.first; });
}

void Sim::applyEdits() {
    if (pendingEdits.empty()) {
        return;
    }
    bool permissions = false;
    std::vector<int> refreshEdges;
    for (const EditRequest& e : pendingEdits) {
        switch (e.kind) {
            case EditRequest::LANE_SPEED:
                // MSLane::setMaxSpeed: no derived data besides the brake distance bound
                net.lanes[e.target].speed = e.value;
                break;
            case EditRequest::LANE_PERMISSIONS: {
                net.lanes[e.target].permissions = (SVCPermissions)e.value;
                if (!e.transient) {
                    net.lanes[e.target].origPermissions = (SVCPermissions)e.value;
                }
                net.edgeVersionPermissions = true;
                permissions = true;
                // MSEdge::rebuildAllowedLanes(false, true): the edge and its predecessors
                // rebuild their allowed targets and the best lanes of their vehicles
                const int edge = net.lanes[e.target].edge;
                refreshEdges.push_back(edge);
                for (int p : net.edges[edge].predecessors) {
                    refreshEdges.push_back(p);
                }
                break;
            }
            case EditRequest::EDGE_REFRESH:
                refreshEdges.push_back(e.target);
                permissions = true;
                break;
            case EditRequest::TLS_PROGRAM: {
                net.tls[e.target].phases = e.phases;
                TLSRT& r = tlsRT[e.target];
                r.step = 0;
                r.lastSwitch.assign(e.phases.size(), now);
                r.lastEnd.assign(e.phases.size(), now);
                r.nextSwitch = now + e.phases[0].duration;
                setTLSignals(e.target);
                break;
            }
        }
    }
    pendingEdits.clear();
    if (permissions) {
        // MSLink::updatePermissions: link permissions only ever increase (transient reductions would make
        // the best lanes invalid)
        for (Link& k : net.links) {
            k.permissions |= net.lanes[k.laneBefore].permissions & net.lanes[k.lane].permissions
                             & (k.via < 0 ? VC_ALL : net.lanes[k.via].permissions);
        }
        net.computeAllowedLanes();
        std::sort(refreshEdges.begin(), refreshEdges.end());
        refreshEdges.erase(std::unique(refreshEdges.begin(), refreshEdges.end()), refreshEdges.end());
        for (int e : refreshEdges) {
            for (int l : net.edges[e].lanes) {
                for (int v : lanes[l].vehs) {
                    updateBestLanes(v, true);
                }
            }
        }
        computeEdgeLevels();
        if (rtActive || hasRerouters()) {
            buildRoutingGraph();
        }
    }
    ++net.version;
    updateMaxBrakeDist();
    seenNetVersion = net.version;
}

}  // namespace ps
