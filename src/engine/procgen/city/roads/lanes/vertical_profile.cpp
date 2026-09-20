#include "engine/procgen/city/roads/lanes/vertical_profile.h"
#include "engine/procgen/city/roads/lanes/polyline_ops.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace engine {
namespace roads::lanes {

namespace {

std::vector<double> smoothed(const std::vector<double>& z, const std::vector<double>& s, double win) {
    if (win <= 0) return z;
    std::vector<double> out(z.size()); size_t lo = 0, hi = 0;
    for (size_t i = 0; i < z.size(); ++i) {
        while (lo < z.size() && s[lo] < s[i] - win / 2) ++lo;
        while (hi < z.size() && s[hi] <= s[i] + win / 2) ++hi;
        double sum = 0; for (size_t k = lo; k < hi; ++k) sum += z[k]; out[i] = sum / static_cast<double>(hi - lo);
    }
    return out;
}

std::vector<double> limitDescents(std::vector<double> z, const std::vector<double>& s, double gmax) {
    for (size_t i = 1; i < z.size(); ++i) z[i] = std::min(z[i], z[i-1] + gmax * (s[i] - s[i-1]));
    for (size_t i = z.size() - 1; i-- > 0;) z[i] = std::min(z[i], z[i+1] + gmax * (s[i+1] - s[i]));
    return z;
}

std::vector<double> gradeLimit(const std::vector<double>& z, const std::vector<double>& s, double gmax) {
    std::vector<double> a = limitDescents(z, s, gmax), neg(z.size()); for (size_t i = 0; i < z.size(); ++i) neg[i] = -z[i];
    std::vector<double> b = limitDescents(neg, s, gmax);
    for (size_t i = 0; i < z.size(); ++i) a[i] = std::max(a[i], -b[i]);
    return a;
}

double smoothstep(double u) { u = std::clamp(u, 0.0, 1.0); return u * u * (3 - 2 * u); }

}  // namespace

void throughProfile(EdgeSpec& e, const HeightField& terrain, const RoadClassSpec& c) {
    e.s = stations(e.xy); e.t.resize(e.xy.size());
    for (size_t i = 0; i < e.xy.size(); ++i) e.t[i] = terrain(e.xy[i].x, e.xy[i].y);
    std::vector<double> z = smoothed(e.t, e.s, c.window); double gd = kDesignGrade * c.gMax;
    if (e.hasZMin) for (double& v : z) v = std::max(v, e.zMin);
    for (const auto& fp : e.floorPts) {                       // a tent at design grade: clears fp.z at its station, grade-consistent by construction
        double sc = project(e.xy, e.s, Vec2(fp[0], fp[1])).station;
        for (size_t i = 0; i < z.size(); ++i) z[i] = std::max(z[i], fp[2] - gd * std::max(0.0, std::fabs(e.s[i] - sc) - fp[3]));
    }
    e.z = gradeLimit(z, e.s, gd);
    // An open end is pinned to the ground there, and nothing may sit higher than the
    // grade allows from it: gradeLimit is the max of two monotone envelopes, so it
    // fills and never cuts. The cone has slope gd exactly, so the result is still
    // grade-legal. A floor (a crossing's hold) within reach of an open end loses to
    // it — a road cannot both start at ground level and clear a bridge 50 m later,
    // and the clearance invariant says which crossings that cost.
    for (int k = 0; k < 2 && !e.z.empty(); ++k) {
        if (!e.openEnd[k]) continue;
        const double zEnd = k ? e.t.back() : e.t.front();
        const double sEnd = k ? e.s.back() : e.s.front();
        for (std::size_t i = 0; i < e.z.size(); ++i)
            e.z[i] = std::min(e.z[i], zEnd + gd * std::fabs(e.s[i] - sEnd));
    }
}

void applyFloors(EdgeSpec& e, const RoadClassSpec& c) {
    if (e.floorPts.empty() || e.z.size() != e.s.size()) return; const double gd = kDesignGrade * c.gMax;
    for (const auto& fp : e.floorPts) {
        const double sc = project(e.xy, e.s, Vec2(fp[0], fp[1])).station;
        for (size_t i = 0; i < e.z.size(); ++i) e.z[i] = std::max(e.z[i], fp[2] - gd * std::max(0.0, std::fabs(e.s[i] - sc) - fp[3]));
    }
}

std::vector<double> profileAlong(const std::vector<Vec2>& xy, const HeightField& terrain, double window, double gMax) {
    std::vector<double> s = stations(xy), t(xy.size()); for (size_t i = 0; i < xy.size(); ++i) t[i] = terrain(xy[i].x, xy[i].y);
    return gradeLimit(smoothed(t, s, window), s, kDesignGrade * gMax);
}

double projectZ(const EdgeSpec& e, const Vec2& p) { return interp(e.s, e.z, project(e.xy, e.s, p).station); }

double maxGrade(const EdgeSpec& e) {
    double m = 0; for (size_t i = 1; i < e.z.size(); ++i) m = std::max(m, std::fabs(e.z[i] - e.z[i-1]) / std::max(e.s[i] - e.s[i-1], 1e-9)); return m;
}

double nodeConsistency(RoadLabGraph& g, double tol, double maxDz) {
    std::vector<EdgeSpec*> thr; for (EdgeSpec& e : g.edges) if (!e.isRamp() && e.z.size() > 1) thr.push_back(&e);
    double worst = 0; std::vector<std::array<double, 2>> corr(thr.size(), {0.0, 0.0});
    for (size_t ei = 0; ei < thr.size(); ++ei) {
        EdgeSpec& e = *thr[ei];
        for (int k = 0; k < 2; ++k) {
            Vec2 p = k == 0 ? e.xy.front() : e.xy.back(); double own = k == 0 ? e.z.front() : e.z.back(); std::vector<double> ends, ints;
            for (EdgeSpec* h : thr) {
                if (h == &e) continue; Projection pr = project(h->xy, h->s, p); if (pr.distance > tol) continue;
                double zh = interp(h->s, h->z, pr.station); bool atEnd = std::min(distance(h->xy.front(), p), distance(h->xy.back(), p)) <= tol;
                if (!atEnd && std::fabs(zh - own) >= maxDz) continue;                    // an endpoint under a viaduct is not a node
                (atEnd ? ends : ints).push_back(zh);
            }
            double tgt;
            if (!ints.empty()) { tgt = 0; for (double v : ints) tgt += v; tgt /= static_cast<double>(ints.size()); }
            else if (!ends.empty()) { tgt = own; for (double v : ends) tgt += v; tgt /= static_cast<double>(ends.size() + 1); }
            else continue;
            worst = std::max(worst, std::fabs(tgt - own)); corr[ei][static_cast<size_t>(k)] = tgt - own;
        }
    }
    for (size_t ei = 0; ei < thr.size(); ++ei) {
        double d0 = corr[ei][0], d1 = corr[ei][1]; if (d0 == 0 && d1 == 0) continue; EdgeSpec& e = *thr[ei]; double L = std::max(e.s.back(), 1e-9);
        for (size_t i = 0; i < e.z.size(); ++i) { double u = e.s[i] / L; e.z[i] += (1 - u) * d0 + u * d1; }
    }
    return worst;
}

double nodeMismatch(const RoadLabGraph& g, double tol, double maxDz, NodeMismatchWhere* where) {
    std::vector<const EdgeSpec*> thr; for (const EdgeSpec& e : g.edges) if (!e.isRamp() && e.z.size() > 1) thr.push_back(&e);
    double worst = 0;
    for (const EdgeSpec* e : thr) for (int k = 0; k < 2; ++k) {
        Vec2 p = k == 0 ? e->xy.front() : e->xy.back(); double own = k == 0 ? e->z.front() : e->z.back();
        for (const EdgeSpec* h : thr) {
            if (h == e) continue; Projection pr = project(h->xy, h->s, p); if (pr.distance > tol) continue;
            double dz = std::fabs(interp(h->s, h->z, pr.station) - own); bool atEnd = std::min(distance(h->xy.front(), p), distance(h->xy.back(), p)) <= tol;
            if (!atEnd && dz >= maxDz) continue;
            if (dz > worst) { worst = dz; if (where) { where->at = p; where->a = e->id; where->b = h->id; where->atEnd = atEnd; } }
        }
    }
    return worst;
}

double crossingConsistency(RoadLabGraph& g, double maxDz, double rampMaxDz, double radius) {
    // Ramps take part as the side that never moves: a ramp crossing a street below bridge height is a level
    // crossing the STREET rises or dips to meet (Glenn's exit ramps sat a metre over local streets; the
    // ramp is pinned to its anchors, so only the street can give). Two ramps, or a ramp and its own host, do not pair.
    std::vector<EdgeSpec*> thr; for (EdgeSpec& e : g.edges) if (e.z.size() > 1 && e.laneCount() > 0) thr.push_back(&e);
    double worst = 0;
    for (size_t i = 0; i < thr.size(); ++i) for (size_t j = i + 1; j < thr.size(); ++j) {
        EdgeSpec& a = *thr[i]; EdgeSpec& b = *thr[j];
        if (a.isRamp() && b.isRamp()) continue;
        if (a.isRamp() && (a.from.edge == b.id || a.to.edge == b.id)) continue;
        if (b.isRamp() && (b.from.edge == a.id || b.to.edge == a.id)) continue;
        // The SAME notion of a crossing the clearance lift uses (lanes.cpp): two
        // centrelines meeting, AND either road ending under the other's paved band.
        // They used to differ, and that gap was a dead zone — a street ending 1.1 m
        // under another was levelled by nobody (consistency never saw it), lifted by
        // nobody (the lift only fires above bridge_h), and then failed by the
        // clearance invariant, which needs 7.8 m. metro_hills, with no freeway at all,
        // failed 9 of 9 pairs that way.
        std::vector<Vec2> meet = crossings(a.xy, b.xy);
        for (int side = 0; side < 2; ++side) {
            const EdgeSpec& e = side ? b : a;
            const EdgeSpec& o = side ? a : b;
            for (const Vec2& q : {e.xy.front(), e.xy.back()}) {
                const Projection pr = project(o.xy, o.s, q);
                if (pr.distance > g.hw(o) + g.hw(e)) continue;
                // Leave nodeConsistency its own cases, or the two passes pull the same
                // T against each other and the agree loop stalls 35 cm short: it owns
                // an end within endpointTol of the other centreline (and a shared node).
                if (pr.distance <= g.rules.endpointTol) continue;
                if (std::min(distance(o.xy.front(), q), distance(o.xy.back(), q)) <= g.rules.endpointTol) continue;
                bool dup = false;
                for (const Vec2& m : meet) if (distance(m, q) < 1.0) dup = true;
                if (!dup) meet.push_back(q);
            }
        }
        for (const Vec2& p : meet) {
            // Only a SHARED node belongs to nodeConsistency — both roads ending here.
            // One road ending mid-span of the other is a T, and nodeConsistency only
            // recognises it within endpointTol (0.5 m) of the centreline; a metre out
            // it saw nothing, this pass skipped it as "a node", and the pair went on
            // to fail clearance at 1.1 m. A T is a level crossing: it is levelled here.
            const double aEnd = std::min(distance(a.xy.front(), p), distance(a.xy.back(), p));
            const double bEnd = std::min(distance(b.xy.front(), p), distance(b.xy.back(), p));
            if (aEnd < g.rules.endpointTol && bEnd < g.rules.endpointTol) continue;          // a shared node
            bool aWins = a.isRamp() ? true : b.isRamp() ? false : std::make_pair(g.cls(a).rank, -g.index.at(a.id)) >= std::make_pair(g.cls(b).rank, -g.index.at(b.id));
            EdgeSpec& hi = aWins ? a : b; EdgeSpec& lo = aWins ? b : a;
            double dz = projectZ(hi, p) - projectZ(lo, p);
            if (std::fabs(dz) >= ((a.isRamp() || b.isRamp()) ? rampMaxDz : maxDz)) continue;   // grade separation (a ramp on its embankment is structure, not a street to meet)
            worst = std::max(worst, std::fabs(dz)); if (std::fabs(dz) < 1e-4) continue;
            static const bool trace = std::getenv("LANELAB_TRACE_CLEARANCE") != nullptr;
            if (trace && (a.isRamp() || b.isRamp() || g.cls(lo).rank >= 3)) std::fprintf(stderr, "crossing-level: %s moves %.2f to meet %s at (%.0f, %.0f)\n", lo.id.c_str(), dz, hi.id.c_str(), p.x, p.y);
            double R = std::max(radius, 1.5 * std::fabs(dz) / ((1 - kDesignGrade) * g.cls(lo).gMax / 4));   // four passes share the design headroom
            double sx = project(lo.xy, lo.s, p).station;
            for (size_t k = 0; k < lo.z.size(); ++k) { double u = std::clamp(1 - std::fabs(lo.s[k] - sx) / R, 0.0, 1.0); lo.z[k] += dz * u * u * (3 - 2 * u); }
        }
    }
    return worst;
}

void rampProfile(RoadLabGraph& g, EdgeSpec& e, const HeightField& terrain) {
    const EdgeSpec* A = e.from.edge.empty() ? nullptr : g.find(e.from.edge); const EdgeSpec* B = e.to.edge.empty() ? nullptr : g.find(e.to.edge);
    e.s = stations(e.xy); e.t.resize(e.xy.size()); for (size_t i = 0; i < e.xy.size(); ++i) e.t[i] = terrain(e.xy[i].x, e.xy[i].y);
    size_t n = e.s.size(); std::vector<double> zA(n), zB(n); int i0 = e.anchorIdx[0], i1 = e.anchorIdx[1]; double hwR = g.hw(e);
    if (A) {
        for (size_t i = 0; i < n; ++i) zA[i] = projectZ(*A, e.xy[i]);
        if (i0 < 0) {   // departure = the end of the INITIAL contiguous run within reach of the host (a ramp may cross its host again later)
            i0 = 0; double th = g.hw(*A) + hwR + 1.0;
            for (size_t i = 0; i < n; ++i) { if (project(A->xy, A->s, e.xy[i]).distance <= th) i0 = static_cast<int>(i); else break; }
        }
    } else { for (size_t i = 0; i < n; ++i) zA[i] = e.t.front(); i0 = 0; }
    if (B) {
        for (size_t i = 0; i < n; ++i) zB[i] = projectZ(*B, e.xy[i]);
        if (i1 < 0) {   // arrival = the start of the FINAL contiguous run within reach of the host
            i1 = static_cast<int>(n) - 1; double th = g.hw(*B) + hwR + 1.0;
            for (size_t i = n; i-- > 0;) { if (project(B->xy, B->s, e.xy[i]).distance <= th) i1 = static_cast<int>(i); else break; }
        }
    } else { for (size_t i = 0; i < n; ++i) zB[i] = e.t.back(); i1 = static_cast<int>(n) - 1; }
    i0 = std::clamp(i0, 0, static_cast<int>(n) - 1); i1 = std::clamp(i1, 0, static_cast<int>(n) - 1); if (i1 <= i0) i1 = std::min(static_cast<int>(n) - 1, i0 + 1);
    double s0 = e.s[static_cast<size_t>(i0)], s1 = e.s[static_cast<size_t>(i1)], za = zA[static_cast<size_t>(i0)], zb = zB[static_cast<size_t>(i1)];
    e.z.resize(n);
    for (size_t i = 0; i < n; ++i) {
        double w = smoothstep((e.s[i] - s0) / std::max(s1 - s0, 1e-6));
        double holdA = e.s[i] <= s0 ? zA[i] : za, holdB = e.s[i] >= s1 ? zB[i] : zb;
        e.z[i] = (1 - w) * holdA + w * holdB;
    }
    // floor tents on a ramp: lift under a design-grade cone, but never inside the held runs at the anchors,
    // and never above what the class's maximum grade can reach from either anchor — a hold the ramp cannot
    // afford is met as far as the grade allows and left for the clearance invariant to name, not a cliff.
    if (!e.floorPts.empty()) {
        const double gd = kDesignGrade * g.cls(e).gMax, gm = g.cls(e).gMax;
        for (const auto& fp : e.floorPts) {
            const double sc = project(e.xy, e.s, Vec2(fp[0], fp[1])).station;
            for (size_t i = 0; i < n; ++i) {
                if (e.s[i] <= s0 || e.s[i] >= s1) continue;
                const double tent = fp[2] - gd * std::max(0.0, std::fabs(e.s[i] - sc) - fp[3]);
                const double cap = std::min(za + gm * (e.s[i] - s0), zb + gm * (s1 - e.s[i]));
                e.z[i] = std::max(e.z[i], std::min(tent, cap));
            }
        }
    }
    const RoadClassSpec& c = g.cls(e); e.ramp.valid = true; e.ramp.departS = s0; e.ramp.touchS = s1; e.ramp.length = e.s.back();
    e.ramp.climb = zb - za; e.ramp.freeLen = s1 - s0; e.ramp.requiredLen = std::fabs(e.ramp.climb) / c.gMax * 1.5; e.ramp.ok = e.ramp.freeLen + 1e-6 >= e.ramp.requiredLen;
}

}  // namespace roads::lanes
}  // namespace engine
