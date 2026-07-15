#include "alignment.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {

// Trace a canonical spiral–arc–spiral fillet from the origin heading +X:
// curvature ramps 0 -> 1/R over Ls (clothoid), holds 1/R over the arc, ramps
// back to 0. Returns the sampled points, the end heading (== the corner's
// deflection), and fills tanIn/tanOut with the distances from the curve's
// start/end to the intersection of its entry/exit tangent lines (the corner
// point) — everything the placement step needs, measured, not closed-form
// (no Fresnel integrals to get subtly wrong).
struct CanonicalFillet {
    std::vector<Vec2> pts;      // from origin, heading +X at pts[0]
    std::vector<Real> curv;     // per point
    Real deflection = 0;        // total heading change (rad, positive left)
    Real tanIn = 0, tanOut = 0; // corner distances along entry/exit tangents
};

CanonicalFillet traceFillet(Real R, Real Ls, Real arcAngle, Real step) {
    CanonicalFillet out;
    const Real k = 1.0 / R;
    const Real arcLen = arcAngle * R;
    const Real total = 2 * Ls + arcLen;
    const int n = std::max(4, static_cast<int>(std::ceil(total / step)));
    const Real ds = total / n;
    Vec2 p(0, 0);
    Real heading = 0;
    out.pts.push_back(p);
    out.curv.push_back(0);
    for (int i = 0; i < n; ++i) {
        const Real s0 = i * ds;
        // curvature at the segment midpoint (2nd-order accurate heading)
        const Real sm = s0 + ds * 0.5;
        Real km;
        if (sm < Ls) km = k * (sm / Ls);
        else if (sm < Ls + arcLen) km = k;
        else km = k * ((total - sm) / Ls);
        const Real hMid = heading + km * ds * 0.5;
        p = p + Vec2(std::cos(hMid), std::sin(hMid)) * ds;
        heading += km * ds;
        out.pts.push_back(p);
        const Real s1 = s0 + ds;
        Real k1;
        if (s1 < Ls) k1 = k * (s1 / Ls);
        else if (s1 < Ls + arcLen) k1 = k;
        else k1 = k * std::max(Real(0), (total - s1) / Ls);
        out.curv.push_back(k1);
    }
    out.deflection = heading;
    // The corner point is where the entry tangent line (the +X axis) meets
    // the exit tangent line (through pts.back() at `heading`). Solve
    // pts.back() - t * exitDir hitting y = 0.
    const Vec2 e = out.pts.back();
    const Vec2 ed(std::cos(heading), std::sin(heading));
    if (std::fabs(ed.y) > 1e-12) {
        const Real t = e.y / ed.y;              // back along the exit tangent
        out.tanOut = t;
        out.tanIn = e.x - t * ed.x;             // corner x on the entry axis
    } else {
        out.tanIn = e.x;                        // degenerate: straight-through
        out.tanOut = 0;
    }
    return out;
}

}  // namespace

Alignment Alignment::fromPolyline(const std::vector<Vec2>& control,
                                  Real radius, Real spiralLen, Real step,
                                  Real minRadius) {
    Alignment out;
    if (control.size() < 2) return out;

    // Emit a straight run of samples from `from` to `to` (exclusive of `to`).
    auto emitTangent = [&](const Vec2& from, const Vec2& to) {
        const Vec2 d = to - from;
        const Real len = d.length();
        if (len < 1e-9) return;
        const Vec2 u = d * (1.0 / len);
        const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
        for (int i = 0; i < n; ++i) {
            AlignmentSample s;
            s.pos = from + u * (len * i / n);
            s.tangent = u;
            s.curvature = 0;
            out.samples_.push_back(s);
        }
    };

    Vec2 cursor = control.front();
    for (std::size_t i = 1; i + 1 < control.size(); ++i) {
        const Vec2& A = cursor;
        const Vec2& B = control[i];
        const Vec2& C = control[i + 1];
        Vec2 inD = B - A, outD = C - B;
        const Real inLen = inD.length(), outLen = outD.length();
        if (inLen < 1e-9 || outLen < 1e-9) continue;
        inD = inD * (1.0 / inLen);
        outD = outD * (1.0 / outLen);
        const Real cosd = std::max(Real(-1), std::min(Real(1), dot(inD, outD)));
        Real defl = std::acos(cosd);                       // magnitude
        const Real side = cross(inD, outD) >= 0 ? 1.0 : -1.0;   // left positive
        if (defl < 0.02) {                                 // effectively straight
            emitTangent(cursor, B);
            cursor = B;
            continue;
        }
        // Fit the fillet: the spiral pair consumes 2*(Ls/2R) of deflection;
        // shrink the spirals when the corner is too gentle for the full pair,
        // and shrink the radius when the tangent lengths don't fit the legs.
        Real R = radius, Ls = spiralLen;
        for (int guard = 0; guard < 8; ++guard) {
            Real spiralDefl = Ls / (2 * R);
            if (2 * spiralDefl > defl * 0.9)
                Ls = defl * 0.9 * R;                       // gentle corner
            spiralDefl = Ls / (2 * R);
            const Real arcAngle = defl - 2 * spiralDefl;
            CanonicalFillet f = traceFillet(R, Ls, arcAngle, step);
            // Room on both legs? (Reserve half of each leg so consecutive
            // corners can both fit their fillets.)
            if (f.tanIn <= inLen * 0.5 + 1e-6 && f.tanOut <= outLen * 0.5 + 1e-6) {
                // Place: entry tangent ends tanIn before B along inD.
                const Vec2 start = B - inD * f.tanIn;
                emitTangent(cursor, start);
                // Map the canonical trace (origin, +X) onto (start, inD),
                // mirrored across the entry axis for right-hand corners.
                const Vec2 ux = inD, uy(-inD.y, inD.x);
                for (std::size_t k = 0; k + 1 < f.pts.size(); ++k) {
                    const Vec2& cp = f.pts[k];
                    const Vec2& cn = f.pts[k + 1];
                    AlignmentSample s;
                    s.pos = start + ux * cp.x + uy * (side * cp.y);
                    Vec2 d = cn - cp;
                    const Real dl = d.length();
                    if (dl < 1e-12) continue;
                    d = d * (1.0 / dl);
                    s.tangent = normalize(Vec2(ux.x * d.x + uy.x * (side * d.y),
                                               ux.y * d.x + uy.y * (side * d.y)));
                    s.curvature = side * f.curv[k];
                    out.samples_.push_back(s);
                }
                cursor = B + outD * f.tanOut;
                break;
            }
            // Floor the radius at the design minimum: never shrink into a
            // hairpin. If the min-radius fillet still won't fit the 0.5-leg
            // reservation, take it anyway on the LAST guard step (a bounded
            // bend that borrows a little more of the legs beats a hairpin or
            // a raw kink).
            if (minRadius > 0 && R * 0.72 < minRadius) {
                R = minRadius;
                Real spiralDefl = Ls / (2 * R);
                if (2 * spiralDefl > defl * 0.9) Ls = defl * 0.9 * R;
                spiralDefl = Ls / (2 * R);
                const Real arcAngle = defl - 2 * spiralDefl;
                CanonicalFillet f = traceFillet(R, Ls, arcAngle, step);
                if (f.tanIn <= inLen * 0.9 + 1e-6 &&
                    f.tanOut <= outLen * 0.9 + 1e-6) {
                    const Vec2 start = B - inD * f.tanIn;
                    emitTangent(cursor, start);
                    const Vec2 ux = inD, uy(-inD.y, inD.x);
                    for (std::size_t k = 0; k + 1 < f.pts.size(); ++k) {
                        const Vec2& cp = f.pts[k];
                        const Vec2& cn = f.pts[k + 1];
                        AlignmentSample s;
                        s.pos = start + ux * cp.x + uy * (side * cp.y);
                        Vec2 d = cn - cp;
                        const Real dl = d.length();
                        if (dl < 1e-12) continue;
                        d = d * (1.0 / dl);
                        s.tangent = normalize(Vec2(ux.x * d.x + uy.x * (side * d.y),
                                                   ux.y * d.x + uy.y * (side * d.y)));
                        s.curvature = side * f.curv[k];
                        out.samples_.push_back(s);
                    }
                    cursor = B + outD * f.tanOut;
                    break;
                }
            }
            R *= 0.72;                                     // tighten and retry
            R = std::max(R, minRadius);                    // but never below the floor
            Ls = std::min(Ls, R);                          // keep spirals sane
        }
        if ((cursor - B).length() < 1e-9) {                // fit never succeeded
            emitTangent(cursor, B);
            cursor = B;
        }
    }
    emitTangent(cursor, control.back());
    {   // closing sample AT the endpoint
        AlignmentSample s;
        s.pos = control.back();
        const Vec2 d = control.back() - control[control.size() - 2];
        s.tangent = d.length() > 1e-9 ? normalize(d)
                                      : (out.samples_.empty()
                                             ? Vec2(1, 0)
                                             : out.samples_.back().tangent);
        s.curvature = 0;
        out.samples_.push_back(s);
    }
    // Stations from the sampled chain.
    Real acc = 0;
    for (std::size_t i = 0; i < out.samples_.size(); ++i) {
        if (i > 0) acc += (out.samples_[i].pos - out.samples_[i - 1].pos).length();
        out.samples_[i].station = acc;
    }
    return out;
}

std::size_t Alignment::indexFor(Real s) const {
    if (samples_.size() < 2) return 0;
    // binary search for the interval [station[i], station[i+1]] containing s
    std::size_t lo = 0, hi = samples_.size() - 1;
    s = std::max(Real(0), std::min(s, samples_.back().station));
    while (lo + 1 < hi) {
        const std::size_t mid = (lo + hi) / 2;
        if (samples_[mid].station <= s) lo = mid;
        else hi = mid;
    }
    return lo;
}

Vec2 Alignment::pos(Real s) const {
    if (samples_.empty()) return Vec2(0, 0);
    if (samples_.size() == 1) return samples_[0].pos;
    const std::size_t i = indexFor(s);
    const AlignmentSample& a = samples_[i];
    const AlignmentSample& b = samples_[i + 1];
    const Real span = b.station - a.station;
    const Real t = span > 1e-12
        ? (std::max(Real(0), std::min(s, samples_.back().station)) - a.station) / span
        : 0;
    return a.pos + (b.pos - a.pos) * t;
}

Vec2 Alignment::tangent(Real s) const {
    if (samples_.empty()) return Vec2(1, 0);
    if (samples_.size() == 1) return samples_[0].tangent;
    const std::size_t i = indexFor(s);
    const AlignmentSample& a = samples_[i];
    const AlignmentSample& b = samples_[i + 1];
    const Real span = b.station - a.station;
    const Real t = span > 1e-12
        ? (std::max(Real(0), std::min(s, samples_.back().station)) - a.station) / span
        : 0;
    const Vec2 mixed = a.tangent * (1 - t) + b.tangent * t;
    const Real len = mixed.length();
    return len > 1e-9 ? mixed * (1.0 / len) : a.tangent;
}

Vec2 Alignment::normal(Real s) const {
    const Vec2 t = tangent(s);
    return Vec2(-t.y, t.x);   // left of travel
}

Real Alignment::curvature(Real s) const {
    if (samples_.empty()) return 0;
    if (samples_.size() == 1) return samples_[0].curvature;
    const std::size_t i = indexFor(s);
    const AlignmentSample& a = samples_[i];
    const AlignmentSample& b = samples_[i + 1];
    const Real span = b.station - a.station;
    const Real t = span > 1e-12
        ? (std::max(Real(0), std::min(s, samples_.back().station)) - a.station) / span
        : 0;
    return a.curvature * (1 - t) + b.curvature * t;
}

Real VerticalProfile::elevation(Real s) const {
    if (pvis.empty()) return 0;
    if (pvis.size() == 1) return pvis[0].elevation;
    s = std::max(pvis.front().station, std::min(s, pvis.back().station));
    // Find the segment [i, i+1] containing s.
    std::size_t i = 0;
    while (i + 2 < pvis.size() && pvis[i + 1].station <= s) ++i;
    const Pvi& a = pvis[i];
    const Pvi& b = pvis[i + 1];
    const Real gAB = (b.elevation - a.elevation) /
                     std::max(Real(1e-9), b.station - a.station);
    // Parabolic curve centred on PVI b, blending grade gAB into the next
    // segment's grade — only if there IS a next segment and s falls inside
    // the transition window. Same for the curve around PVI a.
    auto inCurve = [&](const Pvi& p, std::size_t pi) {
        return p.curveLength > 1e-6 && pi > 0 && pi + 1 < pvis.size() &&
               std::fabs(s - p.station) < p.curveLength * 0.5;
    };
    if (inCurve(b, i + 1)) {
        const Pvi& c = pvis[i + 2];
        const Real gBC = (c.elevation - b.elevation) /
                         std::max(Real(1e-9), c.station - b.station);
        const Real L = b.curveLength;
        const Real x = s - (b.station - L * 0.5);           // 0..L through the curve
        const Real zStart = b.elevation - gAB * (L * 0.5);
        return zStart + gAB * x + (gBC - gAB) * x * x / (2 * L);
    }
    if (inCurve(a, i)) {
        const Pvi& p0 = pvis[i - 1];
        const Real gPA = (a.elevation - p0.elevation) /
                         std::max(Real(1e-9), a.station - p0.station);
        const Real L = a.curveLength;
        const Real x = s - (a.station - L * 0.5);
        const Real zStart = a.elevation - gPA * (L * 0.5);
        return zStart + gPA * x + (gAB - gPA) * x * x / (2 * L);
    }
    return a.elevation + gAB * (s - a.station);
}

Real VerticalProfile::grade(Real s) const {
    const Real h = 0.5;
    return (elevation(s + h) - elevation(s - h)) / (2 * h);
}

Real LaneSchedule::auxFraction(const AuxSpan& a, Real s) {
    if (s < a.s0 || s > a.s1) return 0.0;
    const Real t = std::min(
        Real(1), (a.taperAtStart ? s - a.s0 : a.s1 - s) /
                     std::max(Real(1), a.taper));
    return t * t * (3.0 - 2.0 * t);   // smoothstep: reads as a curve, not a wedge
}

int LaneSchedule::lanesAt(Real s, bool up) const {
    // The aux lane COUNTS (separator dashes, gore offsets) only while it is
    // at least half-width: past that the dashes stop and the narrowing
    // pavement reads as the merge zone (device: gradual merge).
    int n = throughLanes;
    for (const AuxSpan& a : aux)
        if (a.upStation == up && auxFraction(a, s) >= 0.5) ++n;
    return n;
}

Real CorridorDef::halfWidthAt(Real s, int sideSign) const {
    // Aux lanes fade in over ~35 m (deceleration-lane taper) and END at the
    // gore station sharply — the ramp continues the pavement from there.
    const bool up = sideSign < 0;   // up-station drives the negative offsets
    Real extra = 0;
    for (const LaneSchedule::AuxSpan& a : lanes.aux)
        if (a.upStation == up)
            extra += laneWidth * LaneSchedule::auxFraction(a, s);
    Real w = medianWidth * 0.5 + shoulderIn + lanes.throughLanes * laneWidth +
             extra + shoulderOut;
    if (taperEnds) {
        // R1.3: funnel each end down to an arterial half-width over 50 m
        const Real Lc = horizontal.length();
        const Real endDist = std::min(s, Lc - s);
        const Real endHalf = 7.0;
        if (endDist < 50.0 && w > endHalf)
            w = endHalf + (w - endHalf) * (endDist / 50.0);
    }
    return w;
}

Real CorridorDef::halfWidthAt(Real s) const {
    return std::max(halfWidthAt(s, -1), halfWidthAt(s, 1));
}

Real CorridorDef::superelevationAt(Real s) const {
    // e = v^2 / (g R), signed by the curve direction, capped. Straightaways
    // stay flat (the sweep adds its own drainage crown if it wants one).
    const Real k = horizontal.curvature(s);
    const Real e = designSpeed * designSpeed * std::fabs(k) / 9.81;
    return std::min(e, superelevationMax) * (k >= 0 ? 1.0 : -1.0);
}

}  // namespace engine
