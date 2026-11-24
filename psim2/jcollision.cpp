// Collisions inside junctions (--collision.check-junctions): the outlines of vehicles on internal lanes and on
// their foe lanes are intersected (MSLane::detectCollisions with mustCheckJunctionCollisions,
// MSVehicle::getBoundingBox / getBoundingPoly, PositionVector::overlapsWith).
#include <cmath>

#include "model.hpp"

namespace ps {

namespace {
// GeomHelper::angle2D
double angle2D(const Pos& p1, const Pos& p2) {
    return geom::angleDiff(std::atan2(p1.y, p1.x), std::atan2(p2.y, p2.x));
}

// PositionVector::around (offset 0)
bool around(const Shape& poly, const Pos& p) {
    if (poly.size() < 2) {
        return false;
    }
    double angle = 0;
    for (size_t i = 0; i + 1 < poly.size(); ++i) {
        angle += angle2D(poly[i] - p, poly[i + 1] - p);
    }
    angle += angle2D(poly.back() - p, poly.front() - p);
    return !(std::fabs(angle) < M_PI);
}

// PositionVector::partialWithin
bool partialWithin(const Shape& s, const Shape& poly) {
    if (s.size() < 2) {
        return false;
    }
    for (const Pos& p : s) {
        if (around(poly, p)) {
            return true;
        }
    }
    return false;
}

// PositionVector::intersects(p1, p2) (crosses)
bool crosses(const Shape& poly, const Pos& p1, const Pos& p2) {
    if (poly.size() < 2) {
        return false;
    }
    for (size_t i = 0; i + 1 < poly.size(); ++i) {
        if (geom::intersects(poly[i], poly[i + 1], p1, p2, nullptr, nullptr)) {
            return true;
        }
    }
    return false;
}

// PositionVector::overlapsWith (offset 0)
bool overlapsWith(const Shape& a, const Shape& b) {
    if (partialWithin(a, b) || partialWithin(b, a)) {
        return true;
    }
    if (a.size() >= 2) {
        for (size_t i = 0; i + 1 < a.size(); ++i) {
            if (crosses(b, a[i], a[i + 1])) {
                return true;
            }
        }
        if (a.size() > 2 && crosses(b, a.back(), a.front())) {
            return true;
        }
    }
    return false;
}

// PositionVector::extrapolate2D
void extrapolate2D(Shape& s, double val) {
    if (s.size() > 1) {
        Pos& p1 = s[0];
        const Pos p2 = s[1];
        if (p1.distanceTo2D(p2) > 0) {
            const Pos offset = (p2 - p1) * (val / p1.distanceTo2D(p2));
            p1 = p1 - offset;
            if (s.size() == 2) {
                s[1] = s[1] + offset;
            } else {
                const Pos e1 = s[s.size() - 2];
                Pos& e2 = s.back();
                e2 = e2 - (e1 - e2) * (val / e1.distanceTo2D(e2));
            }
        }
    }
}

// PositionVector::removeDoublePoints(POSITION_EPS, assertLength = true)
void removeDoublePoints(Shape& s) {
    if (s.size() < 2) {
        return;
    }
    size_t last = 0;
    size_t i = 1;
    while (i < s.size() && s.size() > 2) {
        if (s[last].almostSame(s[i], POSITION_EPS)) {
            if (i + 1 == s.size()) {
                s.erase(s.begin() + (long)last);  // keep the last point
                break;
            }
            s.erase(s.begin() + (long)i);
        } else {
            last = i;
            ++i;
        }
    }
}

// PositionVector::getCentroid (of a line: the mean of its points)
Pos centroid(const Shape& s) {
    if (s.size() < 3) {
        Pos c;
        for (const Pos& p : s) {
            c = c + p;
        }
        return s.empty() ? c : c * (1. / (double)s.size());
    }
    double area = 0, x = 0, y = 0;
    for (size_t i = 0; i < s.size(); ++i) {
        const Pos& p = s[i];
        const Pos& q = s[(i + 1) % s.size()];
        const double f = p.x * q.y - q.x * p.y;
        area += f;
        x += (p.x + q.x) * f;
        y += (p.y + q.y) * f;
    }
    area /= 2;
    return area == 0 ? s.front() : Pos{x / (6 * area), y / (6 * area)};
}
}  // namespace

// MSLane::mustCheckJunctionCollisions
bool Sim::mustCheckJunctionCollisions(int lane) const {
    if (!opt.checkJunctions || !isInternal(lane) || net.lanes[lane].links.empty()) {
        return false;
    }
    const Link& L = net.links[net.lanes[lane].links.front()];
    return !L.foeLanes.empty() || L.walkingAreaFoe >= 0 || L.walkingAreaFoeExit >= 0;
}

// MSVehicle::getBoundingBox
Shape Sim::vehBoundingBox(int v, double offset) const {
    const Kin& k = kin[v];
    const VType& t = vt(v);
    Shape centerLine;
    const Pos pos = vehPosition(v);
    centerLine.push_back(pos);
    if (t.shapeKind == 2) {
        // long vehicles (trains, trailers): along the lanes they occupy
        for (int i = 0; i < k.nFurther; ++i) {
            centerLine.push_back(net.lanes[k.further[i]].shape.back());
        }
    }
    Pos backPos = vehBackPosition(v);
    if (pos.distanceTo2D(backPos) > t.length + NUMERICAL_EPS) {
        // the back position may not match the visual back in networks without internal lanes
        const double a = vehAngle(v) + M_PI;
        backPos = pos + Pos{t.length * std::cos(a), t.length * std::sin(a)};
    }
    centerLine.push_back(backPos);
    if (offset != 0) {
        extrapolate2D(centerLine, offset);
    }
    removeDoublePoints(centerLine);
    Shape result = centerLine;
    geom::move2side(result, std::max(0.0, 0.5 * t.width + offset));
    geom::move2side(centerLine, std::min(0.0, -0.5 * t.width - offset));
    // PositionVector::append(reverse, POSITION_EPS)
    std::reverse(centerLine.begin(), centerLine.end());
    if (!result.empty() && !centerLine.empty() && result.back().distanceTo2D(centerLine.front()) < POSITION_EPS) {
        result.insert(result.end(), centerLine.begin() + 1, centerLine.end());
    } else {
        result.insert(result.end(), centerLine.begin(), centerLine.end());
    }
    return result;
}

// MSVehicle::getBoundingPoly
Shape Sim::vehBoundingPoly(int v, double offset) const {
    const VType& t = vt(v);
    if (t.shapeKind != 1) {
        return vehBoundingBox(v, 0);
    }
    // passenger cars: a box with the corners cut off
    Shape centerLine{vehPosition(v), vehBackPosition(v)};
    if (offset != 0) {
        extrapolate2D(centerLine, offset);
    }
    Shape line1 = centerLine;
    Shape line2 = centerLine;
    geom::move2side(line1, std::max(0.0, 0.3 * t.width + offset));
    geom::move2side(line2, std::max(0.0, 0.5 * t.width + offset));
    const Pos c = centroid(line2);
    for (Pos& p : line2) {
        p = c + (p - c) * 0.8;  // scaleRelative(0.8)
    }
    Shape result;
    if (line1.size() < 2 || line2.size() < 2) {
        return vehBoundingBox(v, 0);
    }
    result.push_back(line1[0]);
    result.push_back(line2[0]);
    result.push_back(line2[1]);
    result.push_back(line1[1]);
    geom::move2side(line1, std::min(0.0, -0.6 * t.width - offset));
    geom::move2side(line2, std::min(0.0, -1.0 * t.width - offset));
    result.push_back(line1[1]);
    result.push_back(line2[1]);
    result.push_back(line2[0]);
    result.push_back(line1[0]);
    return result;
}

// MSLane::detectCollisions with mustCheckJunctionCollisions: (collider, victim) pairs of one internal lane
void Sim::junctionCollisions(int lane, std::vector<std::pair<int, int>>& out) const {
    const Link& exitLink = net.links[net.lanes[lane].links.front()];
    std::vector<int> colliders;
    anyVehicles(lane, [&](int u) {
        colliders.push_back(u);
        return true;
    });
    const Time ignoreBlocker = opt.ignoreJunctionBlocker >= 0 ? TIME2STEPS(opt.ignoreJunctionBlocker) : TIME_MAX;
    for (int collider : colliders) {
        const Shape colliderBoundary = vehBoundingBox(collider, opt.checkJunctionsMinGap);
        for (int foeLane : exitLink.foeLanes) {
            std::vector<int> victims;
            anyVehicles(foeLane, [&](int u) {
                victims.push_back(u);
                return true;
            });
            for (int victim : victims) {
                if (victim == collider) {
                    continue;  // (the vehicle's lane and shadow lane may be siblings)
                }
                if (ignoreBlocker < TIME_MAX && (veh[collider].waitingTime >= ignoreBlocker || veh[victim].waitingTime >= ignoreBlocker)) {
                    continue;  // ignored vehicles do not collide
                }
                if (!overlapsWith(colliderBoundary, vehBoundingBox(victim, 0))) {
                    continue;
                }
                if (overlapsWith(vehBoundingPoly(collider, opt.checkJunctionsMinGap), vehBoundingPoly(victim, 0))) {
                    // the junction leader is the victim (the collider is still on the junction)
                    if (isInternal(kin[victim].lane) && isLeader(victim, net.lanes[lane].links.front(), collider, -1)) {
                        out.push_back({victim, collider});
                    } else {
                        out.push_back({collider, victim});
                    }
                }
            }
        }
    }
}

}  // namespace ps
