#ifndef RAYTRACER_TESTS_DRIVE_PROBE_H
#define RAYTRACER_TESTS_DRIVE_PROBE_H

// DRIVE PROBE — the gate that mesh-level and sim-level tests both miss.
//
// The citysim drives an abstract NAV GRAPH; the player drives the collision
// MESH. Nothing checked that they agree, so a route the sim happily plans can
// run through a wall, over a cliff, or across a hole in the asphalt — which is
// exactly what shipped: a parapet standing across every freeway gore, 164% steps
// in junction pads, open wedges at ramp landings. Every one of those passed a
// green suite and an aerial screenshot.
//
// So: take the centreline the nav graph says is drivable, and interrogate the
// geometry actually under and around it.
//   - HOLE   : no drivable surface beneath the centreline.
//   - STEP   : the surface jumps between adjacent samples (a bump you hit).
//   - GRADE  : the surface is too steep to be a road.
//   - BLOCKED: something solid stands in the driving corridor at car height.
//
// Header-only so any test TU can drive any mesh it builds.

#include "../src/renderer/renderer.h"
#include "../src/engine/procgen/city/polygon.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace driveprobe {

using engine::Vec2;
using engine::Vec3;
using engine::RenderMesh;
using engine::Vertex;

struct Defect {
    std::string kind;      // "hole" | "step" | "grade" | "blocked"
    Vec3 where{0, 0, 0};
    double amount = 0;     // step metres / grade fraction / blocker height
};

struct Report {
    std::vector<Defect> defects;
    int samples = 0;
    double worstStep = 0, worstGrade = 0;
    int holes = 0, steps = 0, grades = 0, blocked = 0;

    void print(const char* tag) const {
        std::printf("[drive:%s] samples=%d  holes=%d steps=%d grades=%d blocked=%d"
                    "  worstStep=%.2fm worstGrade=%.0f%%\n",
                    tag, samples, holes, steps, grades, blocked, worstStep,
                    worstGrade * 100.0);
        int shown = 0;
        for (const Defect& d : defects) {
            if (shown++ >= 8) { std::printf("   ... %zu more\n", defects.size() - 8); break; }
            std::printf("   %-8s at (%8.1f,%6.2f,%8.1f)  %.2f\n", d.kind.c_str(),
                        (double)d.where.x, (double)d.where.y, (double)d.where.z, d.amount);
        }
    }
    bool clean() const { return defects.empty(); }
};

struct DriveParams {
    double step = 2.0;        // sample spacing along the centreline (m)
    double maxStep = 0.15;    // surface jump between samples that reads as a bump
    double maxGrade = 0.25;   // steeper than this is a wall, not a road
    double rideLo = 0.35;     // bumper band above the surface: ignore kerb lips...
    double rideHi = 1.80;     // ...but catch parapets, walls, tree trunks
    double searchY = 6.0;     // how far around the expected height to look for the road
};

// Möller-Trumbore, two-sided: a wall is a wall from either face.
inline bool rayHitsTri(const Vec3& o, const Vec3& d, double maxT, const Vec3& a,
                       const Vec3& b, const Vec3& c, double& tOut) {
    const Vec3 e1 = b - a, e2 = c - a;
    const Vec3 p = cross(d, e2);
    const double det = dot(e1, p);
    if (std::fabs(det) < 1e-12) return false;
    const double inv = 1.0 / det;
    const Vec3 tv = o - a;
    const double u = dot(tv, p) * inv;
    if (u < -1e-9 || u > 1 + 1e-9) return false;
    const Vec3 q = cross(tv, e1);
    const double v = dot(d, q) * inv;
    if (v < -1e-9 || u + v > 1 + 1e-9) return false;
    const double t = dot(e2, q) * inv;
    if (t < 1e-6 || t > maxT) return false;
    tOut = t;
    return true;
}

// Up-facing surface heights under (x,z), ascending.
inline std::vector<double> surfacesAt(const RenderMesh& m, double x, double z) {
    std::vector<double> hits;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Vertex& A = m.vertices[m.indices[t]];
        const Vertex& B = m.vertices[m.indices[t + 1]];
        const Vertex& C = m.vertices[m.indices[t + 2]];
        if ((A.normal.y + B.normal.y + C.normal.y) / 3.0 < 0.5) continue;
        const double ax = A.position.x, az = A.position.z, bx = B.position.x,
                     bz = B.position.z, cx = C.position.x, cz = C.position.z;
        const double d = (bz - cz) * (ax - cx) + (cx - bx) * (az - cz);
        if (std::fabs(d) < 1e-9) continue;
        const double l1 = ((bz - cz) * (x - cx) + (cx - bx) * (z - cz)) / d;
        const double l2 = ((cz - az) * (x - cx) + (ax - cx) * (z - cz)) / d;
        const double l3 = 1.0 - l1 - l2;
        if (l1 < -1e-6 || l2 < -1e-6 || l3 < -1e-6) continue;
        hits.push_back(l1 * A.position.y + l2 * B.position.y + l3 * C.position.y);
    }
    std::sort(hits.begin(), hits.end());
    return hits;
}

// Drive `path` (a centreline with absolute heights) across `m` and report what a
// car would hit. `expectY` seeds the search so a ramp over a street picks the
// ramp, not the ground under it.
inline void drivePath(const RenderMesh& m, const std::vector<Vec3>& path,
                      Report& rep, const DriveParams& p = {}) {
    if (path.size() < 2) return;
    double prevY = 0; bool havePrev = false;
    double prevDelta = 0; bool haveDelta = false;
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        const Vec3 a = path[i], b = path[i + 1];
        const double segLen = (Vec2(b.x, b.z) - Vec2(a.x, a.z)).length();
        if (segLen < 1e-6) continue;
        const int n = std::max(1, (int)std::ceil(segLen / p.step));
        // Consecutive path segments share an endpoint: sampling k=0 again
        // would DOUBLE-SAMPLE it — a ghost delta of zero that alternates
        // with the real per-sample delta and reads as a kink/staircase on
        // any grade (and double-counts samples and defects at seams).
        for (int k = havePrev ? 1 : 0; k <= n; ++k) {
            const double t = (double)k / n;
            const Vec3 q = a + (b - a) * t;
            ++rep.samples;
            // the drivable surface nearest where the route says we should be
            std::vector<double> hits = surfacesAt(m, q.x, q.z);
            double best = 1e30; bool found = false;
            for (double h : hits)
                if (std::fabs(h - q.y) < p.searchY &&
                    std::fabs(h - q.y) < std::fabs(best - q.y)) { best = h; found = true; }
            if (!found) {
                rep.defects.push_back({"hole", q, 0});
                ++rep.holes; havePrev = false; continue;
            }
            if (havePrev) {
                const double delta = best - prevY;
                const double jump = std::fabs(delta);
                const double run = segLen / n;
                const double g = run > 1e-6 ? jump / run : 0.0;
                rep.worstGrade = std::max(rep.worstGrade, g);
                // A STEP is a surface DISCONTINUITY, not a steep-but-smooth
                // grade: fire on the KINK (second difference — the change in
                // per-sample delta) or on an outright cliff. Raw |delta| >
                // 0.15 also fired on a smooth 11% ramp knee sampled every
                // ~1.4 m (0.15/sample) — a drivable grade, which the grade
                // check below owns. On flat/gentle fixtures deltas are ~0,
                // so the kink rule matches the old rule there exactly.
                const double kink = haveDelta ? std::fabs(delta - prevDelta)
                                              : jump;
                rep.worstStep = std::max(rep.worstStep, kink);
                if (kink > p.maxStep || jump > 0.6) {
                    rep.defects.push_back({"step", q, std::max(kink, jump)});
                    ++rep.steps;
                } else if (g > p.maxGrade) {
                    rep.defects.push_back({"grade", q, g});
                    ++rep.grades;
                }
                prevDelta = delta; haveDelta = true;
            } else {
                haveDelta = false;
            }
            prevY = best; havePrev = true;

            // Anything solid in the bumper band, along the direction of travel?
            if (k < n) {
                Vec3 dir = b - a;
                const double L = std::sqrt(dir.x * dir.x + dir.z * dir.z);
                dir = Vec3(dir.x / L, 0, dir.z / L);
                const Vec3 o(q.x, best + p.rideLo, q.z);
                const double reach = segLen / n + 0.05;   // to the next sample
                for (std::size_t tri = 0; tri + 2 < m.indices.size(); tri += 3) {
                    const Vec3& A = m.vertices[m.indices[tri]].position;
                    const Vec3& B = m.vertices[m.indices[tri + 1]].position;
                    const Vec3& C = m.vertices[m.indices[tri + 2]].position;
                    // only faces that stand UP in the band can block us
                    const double lo = std::min({A.y, B.y, C.y}), hi = std::max({A.y, B.y, C.y});
                    if (hi < best + p.rideLo || lo > best + p.rideHi) continue;
                    double tt;
                    if (rayHitsTri(o, dir, reach, A, B, C, tt)) {
                        rep.defects.push_back({"blocked", Vec3(q.x, best, q.z), hi - best});
                        ++rep.blocked;
                        break;
                    }
                }
            }
        }
    }
}

}  // namespace driveprobe

#endif
