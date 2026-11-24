// 2D geometry for junction conflict points and the pedestrian model.
// Ported from SUMO 1.28 utils/geom (Position, PositionVector, GeomHelper).
// Shapes are treated as 2D (net files of the scenarios used here have no z).
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace ps {

constexpr double INVALID_OFFSET = -1;  // GeomHelper::INVALID_OFFSET
constexpr double INVALID_DOUBLE = std::numeric_limits<double>::max();

struct Pos {
    double x = 0, y = 0;
    double distanceTo2D(const Pos& p) const { return std::sqrt(distanceSquaredTo2D(p)); }
    double distanceSquaredTo2D(const Pos& p) const { return (x - p.x) * (x - p.x) + (y - p.y) * (y - p.y); }
    double angleTo2D(const Pos& o) const { return std::atan2(o.y - y, o.x - x); }
    Pos operator+(const Pos& o) const { return Pos{x + o.x, y + o.y}; }
    Pos operator-(const Pos& o) const { return Pos{x - o.x, y - o.y}; }
    Pos operator*(double f) const { return Pos{x * f, y * f}; }
    bool operator==(const Pos& o) const { return x == o.x && y == o.y; }
    bool operator!=(const Pos& o) const { return !(*this == o); }
    bool almostSame(const Pos& o, double maxDiv = 0.1) const { return distanceTo2D(o) < maxDiv; }  // POSITION_EPS
    bool valid() const { return x != INVALID_DOUBLE; }
};

constexpr Pos INVALID_POS{INVALID_DOUBLE, INVALID_DOUBLE};  // Position::INVALID

using Shape = std::vector<Pos>;

namespace geom {

inline double deg(double rad) { return rad * 180. / M_PI; }
inline double rad(double d) { return d * M_PI / 180.; }

inline double length2D(const Shape& s) {
    double len = 0;
    for (size_t i = 0; i + 1 < s.size(); ++i) {
        len += s[i].distanceTo2D(s[i + 1]);
    }
    return len;
}

inline Pos positionAtOffset2D(const Pos& p1, const Pos& p2, double pos) {
    const double dist = p1.distanceTo2D(p2);
    if (dist == 0 || pos == 0.) {
        return p1;
    }
    return Pos{p1.x + (p2.x - p1.x) * (pos / dist), p1.y + (p2.y - p1.y) * (pos / dist)};
}

inline Pos positionAtOffset2D(const Shape& s, double pos) {
    if (s.size() == 1) {
        return s.front();
    }
    double seen = 0;
    for (size_t i = 0; i + 1 < s.size(); ++i) {
        const double next = s[i].distanceTo2D(s[i + 1]);
        if (seen + next > pos) {
            return positionAtOffset2D(s[i], s[i + 1], pos - seen);
        }
        seen += next;
    }
    return s.back();
}

inline double rotationAtOffset(const Shape& s, double pos) {
    if (s.size() < 2) {
        return INVALID_DOUBLE;
    }
    if (pos < 0) {
        pos += length2D(s);
    }
    double seen = 0;
    for (size_t i = 0; i + 1 < s.size(); ++i) {
        const double next = s[i].distanceTo2D(s[i + 1]);
        if (seen + next > pos) {
            return s[i].angleTo2D(s[i + 1]);
        }
        seen += next;
    }
    return s[s.size() - 2].angleTo2D(s.back());
}

inline double angleAt2D(const Shape& s, int pos) {
    if (pos < 0) {
        pos += (int)s.size();
    }
    if (pos + 1 < (int)s.size()) {
        return s[pos].angleTo2D(s[pos + 1]);
    }
    return INVALID_DOUBLE;
}

// PositionVector::intersects(p11, p12, p21, p22, withinDist = 0, ...)
inline bool intersects(const Pos& p11, const Pos& p12, const Pos& p21, const Pos& p22, double* x, double* y) {
    const double eps = std::numeric_limits<double>::epsilon();
    const double denominator = (p22.y - p21.y) * (p12.x - p11.x) - (p22.x - p21.x) * (p12.y - p11.y);
    const double numera = (p22.x - p21.x) * (p11.y - p21.y) - (p22.y - p21.y) * (p11.x - p21.x);
    const double numerb = (p12.x - p11.x) * (p11.y - p21.y) - (p12.y - p11.y) * (p11.x - p21.x);
    if (std::fabs(numera) < eps && std::fabs(numerb) < eps && std::fabs(denominator) < eps) {
        double a1, a2, a3, a4;
        double a = -1e12;
        if (p11.x != p12.x) {
            a1 = p11.x < p12.x ? p11.x : p12.x;
            a2 = p11.x < p12.x ? p12.x : p11.x;
            a3 = p21.x < p22.x ? p21.x : p22.x;
            a4 = p21.x < p22.x ? p22.x : p21.x;
        } else {
            a1 = p11.y < p12.y ? p11.y : p12.y;
            a2 = p11.y < p12.y ? p12.y : p11.y;
            a3 = p21.y < p22.y ? p21.y : p22.y;
            a4 = p21.y < p22.y ? p22.y : p21.y;
        }
        if (a1 <= a3 && a3 <= a2) {
            a = a4 < a2 ? (a3 + a4) / 2 : (a2 + a3) / 2;
        }
        if (a3 <= a1 && a1 <= a4) {
            a = a2 < a4 ? (a1 + a2) / 2 : (a1 + a4) / 2;
        }
        if (a != -1e12) {
            if (x != nullptr) {
                if (p11.x != p12.x) {
                    const double mu = (a - p11.x) / (p12.x - p11.x);
                    *x = a;
                    *y = p11.y + mu * (p12.y - p11.y);
                } else {
                    *x = p11.x;
                    *y = a;
                }
            }
            return true;
        }
        return false;
    }
    if (std::fabs(denominator) < eps) {
        return false;
    }
    double mua = numera / denominator;
    if (std::fabs(p12.x - p22.x) < eps && std::fabs(p12.y - p22.y) < eps) {
        mua = 1.;
    } else {
        const double mub = numerb / denominator;
        if (mua < 0 || mua > 1 || mub < 0 || mub > 1) {
            return false;
        }
    }
    if (x != nullptr) {
        *x = p11.x + mua * (p12.x - p11.x);
        *y = p11.y + mua * (p12.y - p11.y);
    }
    return true;
}

inline bool intersects(const Shape& a, const Shape& b) {
    for (size_t i = 0; i + 1 < a.size(); ++i) {
        for (size_t j = 0; j + 1 < b.size(); ++j) {
            if (intersects(b[j], b[j + 1], a[i], a[i + 1], nullptr, nullptr)) {
                return true;
            }
        }
    }
    return false;
}

// PositionVector::intersectsAtLengths2D(other): offsets along `s` where `other` crosses it
inline std::vector<double> intersectsAtLengths2D(const Shape& s, const Shape& other) {
    std::vector<double> ret;
    for (size_t j = 0; j + 1 < other.size(); ++j) {
        double pos = 0;
        for (size_t i = 0; i + 1 < s.size(); ++i) {
            double x, y;
            if (intersects(s[i], s[i + 1], other[j], other[j + 1], &x, &y)) {
                ret.push_back(Pos{x, y}.distanceTo2D(s[i]) + pos);
            }
            pos += s[i].distanceTo2D(s[i + 1]);
        }
    }
    return ret;
}

inline double nearestOffsetOnLine(const Pos& a, const Pos& b, const Pos& p, bool perpendicular) {
    const double len = a.distanceTo2D(b);
    if (len == 0.) {
        return 0.;
    }
    const double u = ((p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y)) / len;
    if (u < 0. || u > len) {
        if (perpendicular) {
            return INVALID_OFFSET;
        }
        return u < 0. ? 0. : len;
    }
    return u;
}

inline double nearestOffsetToPoint2D(const Shape& s, const Pos& p, bool perpendicular) {
    double minDist = std::numeric_limits<double>::max();
    double nearestPos = INVALID_OFFSET;
    double seen = 0;
    for (size_t i = 0; i + 1 < s.size(); ++i) {
        const double pos = nearestOffsetOnLine(s[i], s[i + 1], p, perpendicular);
        const double dist2 = pos == INVALID_OFFSET ? minDist : p.distanceSquaredTo2D(positionAtOffset2D(s[i], s[i + 1], pos));
        if (dist2 < minDist) {
            nearestPos = pos + seen;
            minDist = dist2;
        }
        if (perpendicular && i != 0 && pos == INVALID_OFFSET) {
            const double cornerDist2 = p.distanceSquaredTo2D(s[i]);
            if (cornerDist2 < minDist) {
                const double pos1 = nearestOffsetOnLine(s[i - 1], s[i], p, false);
                const double pos2 = nearestOffsetOnLine(s[i], s[i + 1], p, false);
                if (pos1 == s[i - 1].distanceTo2D(s[i]) && pos2 == 0.) {
                    nearestPos = seen;
                    minDist = cornerDist2;
                }
            }
        }
        seen += s[i].distanceTo2D(s[i + 1]);
    }
    return nearestPos;
}

inline double distance2D(const Shape& s, const Pos& p, bool perpendicular) {
    if (s.empty()) {
        return std::numeric_limits<double>::max();
    }
    if (s.size() == 1) {
        return s.front().distanceTo2D(p);
    }
    const double off = nearestOffsetToPoint2D(s, p, perpendicular);
    if (off == INVALID_OFFSET) {
        return INVALID_OFFSET;
    }
    return p.distanceTo2D(positionAtOffset2D(s, off));
}

inline std::vector<double> distances(const Shape& a, const Shape& b) {
    std::vector<double> ret;
    for (const Pos& p : a) {
        const double d = distance2D(b, p, false);
        if (d != INVALID_OFFSET) {
            ret.push_back(d);
        }
    }
    for (const Pos& p : b) {
        const double d = distance2D(a, p, false);
        if (d != INVALID_OFFSET) {
            ret.push_back(d);
        }
    }
    return ret;
}

// ---- pedestrian geometry (PositionVector, SUMO 1.28) ------------------------------------------

// PositionVector::sideOffset
inline Pos sideOffset(const Pos& beg, const Pos& end, double amount) {
    const double scale = amount / beg.distanceTo2D(end);
    return Pos{(beg.y - end.y) * scale, (end.x - beg.x) * scale};
}

// PositionVector::positionAtOffset(p1, p2, pos, lateralOffset)
inline Pos positionAtOffset(const Pos& p1, const Pos& p2, double pos, double lateralOffset) {
    const double dist = p1.distanceTo2D(p2);
    if (pos < 0. || dist < pos) {
        return INVALID_POS;
    }
    if (lateralOffset != 0) {
        if (dist == 0.) {
            return INVALID_POS;
        }
        const Pos offset = sideOffset(p1, p2, -lateralOffset);
        if (pos == 0.) {
            return p1 + offset;
        }
        return p1 + (p2 - p1) * (pos / dist) + offset;
    }
    if (pos == 0.) {
        return p1;
    }
    return p1 + (p2 - p1) * (pos / dist);
}

// PositionVector::positionAtOffset(pos, lateralOffset)
inline Pos positionAtOffset(const Shape& s, double pos, double lateralOffset) {
    if (s.empty()) {
        return INVALID_POS;
    }
    if (s.size() == 1) {
        return s.front();
    }
    double seen = 0;
    for (size_t i = 0; i + 1 < s.size(); ++i) {
        const double next = s[i].distanceTo2D(s[i + 1]);
        if (seen + next > pos) {
            return positionAtOffset(s[i], s[i + 1], pos - seen, lateralOffset);
        }
        seen += next;
    }
    if (lateralOffset == 0) {
        return s.back();
    }
    return positionAtOffset(s[s.size() - 2], s.back(), s[s.size() - 2].distanceTo2D(s.back()), lateralOffset);
}

// PositionVector::sidePositionAtAngle
inline Pos sidePositionAtAngle(const Pos& p1, const Pos& p2, double pos, double lateralOffset, double angle) {
    const double dist = p1.distanceTo2D(p2);
    if (pos < 0. || dist < pos || dist == 0) {
        return INVALID_POS;
    }
    angle -= rad(90);
    const Pos offset{std::cos(angle) * lateralOffset, std::sin(angle) * lateralOffset};
    return p1 + (p2 - p1) * (pos / dist) + offset;
}

inline Pos sidePositionAtAngle(const Shape& s, double pos, double lateralOffset, double angle) {
    if (s.empty()) {
        return INVALID_POS;
    }
    if (s.size() == 1) {
        return s.front();
    }
    double seen = 0;
    for (size_t i = 0; i + 1 < s.size(); ++i) {
        const double next = s[i].distanceTo2D(s[i + 1]);
        if (seen + next > pos) {
            return sidePositionAtAngle(s[i], s[i + 1], pos - seen, lateralOffset, angle);
        }
        seen += next;
    }
    return sidePositionAtAngle(s[s.size() - 2], s.back(), s[s.size() - 2].distanceTo2D(s.back()), lateralOffset, angle);
}

// PositionVector::extrapolate (onlyFirst / onlyLast false)
inline void extrapolate(Shape& s, double val) {
    if (s.size() > 1) {
        Pos& p1 = s[0];
        const Pos p2 = s[1];
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

// PositionVector::isLeft
inline double isLeft(const Pos& p0, const Pos& p1, const Pos& p2) {
    return (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y);
}

// PositionVector::transformToVectorCoordinates: (offset along the shape, signed distance; right is positive)
inline Pos transformToVectorCoordinates(const Shape& s, const Pos& p, bool extend) {
    if (s.empty()) {
        return INVALID_POS;
    }
    if (extend) {
        Shape extended = s;
        const double dist = 2 * distance2D(s, p, false);
        extrapolate(extended, dist);
        const Pos r = transformToVectorCoordinates(extended, p, false);
        return r.valid() ? Pos{r.x - dist, r.y} : r;
    }
    double minDist = std::numeric_limits<double>::max();
    double nearestPos = -1;
    double seen = 0;
    int sign = 1;
    for (size_t i = 0; i + 1 < s.size(); ++i) {
        const double pos = nearestOffsetOnLine(s[i], s[i + 1], p, true);
        const double dist = pos < 0 ? minDist : p.distanceTo2D(positionAtOffset(s[i], s[i + 1], pos, 0));
        if (dist < minDist) {
            nearestPos = pos + seen;
            minDist = dist;
            sign = isLeft(s[i], s[i + 1], p) >= 0 ? -1 : 1;
        }
        if (i != 0 && pos == INVALID_OFFSET) {
            const double cornerDist = p.distanceTo2D(s[i]);
            if (cornerDist < minDist) {
                const double pos1 = nearestOffsetOnLine(s[i - 1], s[i], p, false);
                const double pos2 = nearestOffsetOnLine(s[i], s[i + 1], p, false);
                if (pos1 == s[i - 1].distanceTo2D(s[i]) && pos2 == 0.) {
                    nearestPos = seen;
                    minDist = cornerDist;
                    sign = isLeft(s[i - 1], s[i], p) >= 0 ? -1 : 1;
                }
            }
        }
        seen += s[i].distanceTo2D(s[i + 1]);
    }
    if (nearestPos != -1) {
        return Pos{nearestPos, sign * minDist};
    }
    return INVALID_POS;
}

// PositionVector::push_back_noDoublePos
inline void pushBackNoDoublePos(Shape& s, const Pos& p) {
    if (s.empty() || !p.almostSame(s.back())) {
        s.push_back(p);
    }
}

// PositionVector::bezier
inline Shape bezier(const Shape& s, int numPoints) {
    static const double fac[33] = {
        1.0, 1.0, 2.0, 6.0, 24.0, 120.0, 720.0, 5040.0, 40320.0, 362880.0, 3628800.0, 39916800.0, 479001600.0,
        6227020800.0, 87178291200.0, 1307674368000.0, 20922789888000.0, 355687428096000.0, 6402373705728000.0,
        121645100408832000.0, 2432902008176640000.0, 51090942171709440000.0, 1124000727777607680000.0,
        25852016738884976640000.0, 620448401733239439360000.0, 15511210043330985984000000.0,
        403291461126605635584000000.0, 10888869450418352160768000000.0, 304888344611713860501504000000.0,
        8841761993739701954543616000000.0, 265252859812191058636308480000000.0,
        8222838654177922817725562880000000.0, 263130836933693530167218012160000000.0};
    Shape ret;
    const int npts = (int)s.size();
    const double step = 1.0 / (numPoints - 1);
    double t = 0.;
    Pos prev;
    for (int i1 = 0; i1 < numPoints; i1++) {
        if ((1.0 - t) < 5e-6) {
            t = 1.0;
        }
        double x = 0., y = 0.;
        for (int i = 0; i < npts; i++) {
            const double ti = (i == 0) ? 1.0 : std::pow(t, i);
            const double tni = (npts == i + 1) ? 1.0 : std::pow(1 - t, npts - i - 1);
            const double basis = fac[npts - 1] / (fac[i] * fac[npts - 1 - i]) * ti * tni;
            x += basis * s[i].x;
            y += basis * s[i].y;
        }
        t += step;
        const Pos current{x, y};
        if (prev != current && !std::isnan(x) && !std::isnan(y)) {
            ret.push_back(current);
        }
        prev = current;
    }
    return ret;
}

// PositionVector::move2side for a two-point vector (the only use in the pedestrian model;
// removeDoublePoints keeps both points of a two-point vector)
inline void move2side2(Shape& s, double amount) {
    if (s.size() < 2 || s[0].distanceTo2D(s[1]) == 0 || amount == 0) {
        return;
    }
    const Pos off = sideOffset(s[0], s[1], amount);
    s = Shape{s[0] - off, s[1] - off};
}

// PositionVector::getSubpart (2D): the part between two offsets along the shape
inline Shape getSubpart(const Shape& s, double beginOffset, double endOffset) {
    const double len = length2D(s);
    beginOffset = std::max(0.0, std::min(beginOffset, len));
    endOffset = std::max(beginOffset, std::min(endOffset, len));
    Shape ret;
    ret.push_back(positionAtOffset2D(s, beginOffset));
    double seen = 0;
    for (size_t i = 0; i + 1 < s.size(); ++i) {
        seen += s[i].distanceTo2D(s[i + 1]);
        if (seen > beginOffset && seen < endOffset) {
            pushBackNoDoublePos(ret, s[i + 1]);
        }
    }
    pushBackNoDoublePos(ret, positionAtOffset2D(s, endOffset));
    return ret;
}

// PositionVector::move2side (2D, without the sharp angle warnings): shifted by amount to the right
inline void move2side(Shape& s, double amount) {
    if (s.size() < 2 || length2D(s) == 0 || amount == 0) {
        return;
    }
    Shape shape;
    for (size_t i = 0; i < s.size(); ++i) {
        if (i == 0) {
            if (s[0] != s[1]) {
                shape.push_back(s[0] - sideOffset(s[0], s[1], amount));
            }
        } else if (i + 1 == s.size()) {
            if (s[i - 1] != s[i]) {
                shape.push_back(s[i] - sideOffset(s[i - 1], s[i], amount));
            }
        } else {
            const Pos& from = s[i - 1];
            const Pos& me = s[i];
            const Pos& to = s[i + 1];
            const Pos o1 = sideOffset(from, me, amount);
            const Pos o2 = sideOffset(me, to, amount);
            double x, y;
            if (intersects(from - o1, me - o1, me - o2, to - o2, &x, &y)) {
                shape.push_back(Pos{x, y});
            } else {
                shape.push_back(me - sideOffset(from, to, amount));  // (parallel segments)
            }
        }
    }
    s = shape;
}

// Boundary
struct Boundary {
    double xmin = std::numeric_limits<double>::max(), xmax = std::numeric_limits<double>::lowest();
    double ymin = std::numeric_limits<double>::max(), ymax = std::numeric_limits<double>::lowest();
    void add(const Pos& p) {
        xmin = std::min(xmin, p.x);
        xmax = std::max(xmax, p.x);
        ymin = std::min(ymin, p.y);
        ymax = std::max(ymax, p.y);
    }
    void growWidth(double by) {
        xmin -= by;
        xmax += by;
    }
    double width() const { return xmax - xmin; }
    Pos center() const { return Pos{(xmin + xmax) / 2., (ymin + ymax) / 2.}; }
};

inline double naviDegree(double angle) {
    double degree = deg(M_PI / 2. - angle);
    if (std::isinf(degree)) {
        return 0;
    }
    while (degree >= 360.) {
        degree -= 360.;
    }
    while (degree < 0.) {
        degree += 360.;
    }
    return degree;
}

inline double getMinAngleDiff(double a1, double a2) {
    double cw = a1 - a2;
    if (cw < 0) {
        cw = 360 + cw;
    }
    double ccw = a2 - a1;
    if (ccw < 0) {
        ccw = 360 + ccw;
    }
    return std::min(cw, ccw);
}

inline double angleDiff(double a1, double a2) {
    double d = a2 - a1;
    while (d > M_PI) {
        d -= 2.0 * M_PI;
    }
    while (d < -M_PI) {
        d += 2.0 * M_PI;
    }
    return d;
}

}  // namespace geom
}  // namespace ps
