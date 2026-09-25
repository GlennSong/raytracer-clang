#include "engine/procgen/city/roads/lanes/deck_mesh.h"
#include "engine/mesh_builder.h"
#include "log.h"
#include "engine/procgen/city/roads/lanes/polyline_ops.h"
#include "engine/procgen/city/roads/lanes/geom2d.h"

#include <algorithm>
#include <cstdlib>
#include <unordered_map>
#include <cmath>
#include <functional>
#include <tuple>
#include <map>
#include <set>
#include <cstdio>

namespace engine {
namespace roads::lanes {

namespace {

Vec3 world(const Vec2& p, double h) { return Vec3(static_cast<Real>(p.x), static_cast<Real>(h), static_cast<Real>(p.y)); }

// kAsphalt was 0.045: near black, and the Asphalt surface's grain is invisible on it. Real asphalt sits
// around 0.08-0.12 albedo. kConcrete was a mid grey; a barrier and a pier read as eggshell white, matte.
// kGuardrail is galvanised steel: light, slightly cool, and barely metallic — a high metallic value with no
// environment to reflect just renders black, which is what a first pass at 0.65 did.
const Vec3 kAsphalt(0.085f, 0.085f, 0.090f), kConcrete(0.80f, 0.79f, 0.75f), kSidewalk(0.62f, 0.60f, 0.55f), kShoulder(0.115f, 0.115f, 0.120f),
           kGuardrail(0.66f, 0.68f, 0.70f),
           kMedian(0.22f, 0.34f, 0.14f), kPaintWhite(0.9f, 0.9f, 0.85f), kPaintYellow(0.95f, 0.75f, 0.15f), kGrass(0.20f, 0.30f, 0.13f);

// WELDED, AND NO UNDERSIDE WHERE NOBODY CAN SEE ONE (ADR-0095). Every deck triangle used to be
// emitted with three vertices of its own, top AND bottom face: metro_planned's asphalt alone was
// 779 MB of GPU mesh. A top vertex is now shared by every triangle that meets it with the same
// UV (one lane's triangles; two lanes' UVs differ and keep their own), and the underside is
// emitted only where it stands clear of the ground (`groundAt`, 0.3 m) -- a bridge, a viaduct,
// a ramp; a street lies on the ground. Null groundAt: every underside, as before.
void slab(RenderMesh& top, RenderMesh& side, const std::vector<DeckVertex>& verts, const std::vector<std::array<int, 3>>& tris,
          const std::vector<std::pair<int, int>>& boundary, double thick, double lift, const Vec3& color, const Vec3& sideColor,
          const std::vector<std::array<float, 6>>* uv, const std::function<bool(const Vec2&, const Vec2&, double)>* seam = nullptr,
          const std::function<double(const Vec2&)>* groundAt = nullptr,
          const std::function<bool(const Vec2&, double)>* hidden = nullptr) {
    std::map<std::tuple<int, float, float, int>, uint32_t> shared;   // (deck vertex, u, v, face) -> mesh vertex
    auto vertexFor = [&](int vi, const Vec3& p, float u, float v, const Vec3& normal, const Vec3& tan, int face) {
        auto [it, fresh] = shared.try_emplace(std::make_tuple(vi, u, v, face), static_cast<uint32_t>(top.vertices.size()));
        if (fresh) {
            Vertex vx(p, normal, tan, u, v);
            vx.color = color;
            top.vertices.push_back(vx);
        }
        return it->second;
    };
    auto emit = [&](const int (&vi)[3], const Vec3 (&P)[3], const float (&u)[6], const Vec3& normal, int face) {
        const Vec3 edge = P[1] - P[0];
        const Vec3 tan = edge.lengthSquared() > 1e-12 ? normalize(edge) : Vec3(1, 0, 0);
        const uint32_t a = vertexFor(vi[0], P[0], u[0], u[1], normal, tan, face);
        const uint32_t b = vertexFor(vi[1], P[1], u[2], u[3], normal, tan, face);
        const uint32_t c = vertexFor(vi[2], P[2], u[4], u[5], normal, tan, face);
        // the front face points the way the shading normal does (as MeshBuilder::emitTri)
        if (dot(cross(P[2] - P[0], P[1] - P[0]), normal) >= 0) top.indices.insert(top.indices.end(), {a, b, c});
        else top.indices.insert(top.indices.end(), {a, c, b});
    };
    static const float kNoUV[6] = {0, 0, 0, 0, 0, 0};
    for (size_t ti = 0; ti < tris.size(); ++ti) {
        const auto& t = tris[ti]; const DeckVertex& a = verts[static_cast<size_t>(t[0])]; const DeckVertex& b = verts[static_cast<size_t>(t[1])]; const DeckVertex& c = verts[static_cast<size_t>(t[2])];
        const int vi[3] = {t[0], t[1], t[2]};
        const Vec3 P[3] = {world(a.xy, a.z + lift), world(b.xy, b.z + lift), world(c.xy, c.z + lift)};
        float u[6];
        for (int k = 0; k < 6; ++k) u[k] = uv ? (*uv)[ti][static_cast<size_t>(k)] : 0.0f;
        emit(vi, P, u, Vec3(0, 1, 0), 0);
        bool under = true;
        if (groundAt) {
            under = false;
            for (const DeckVertex* dv : {&a, &b, &c})
                if (dv->z + lift - thick - (*groundAt)(dv->xy) > 0.3) under = true;
        }
        if (under) {
            const Vec3 Q[3] = {world(a.xy, a.z + lift - thick), world(b.xy, b.z + lift - thick), world(c.xy, c.z + lift - thick)};
            emit(vi, Q, kNoUV, Vec3(0, -1, 0), 1);
        }
    }
    // THE SIDE ONLY WHERE IT SHOWS (ADR-0095). A side is dropped where the surface beyond the edge
    // stands at or above its top (`hidden`: asphalt under a sidewalk, a shoulder against the lane it
    // continues), and where it does not rise above the ground outside it (`groundAt`); one that shows
    // stops 5 cm below that ground instead of the slab's full thickness. Then consecutive sides along
    // one straight line become ONE quad: a kerb is a polyline tessellated for its curves, and on the
    // straights every one of those vertices was a quad of its own. Null groundAt/hidden: every side,
    // full depth, as before (still merged).
    struct Side { int a, b; double topA, topB, botA, botB; };
    std::vector<Side> sides; sides.reserve(boundary.size());
    for (const auto& e : boundary) {
        const DeckVertex& a = verts[static_cast<size_t>(e.first)]; const DeckVertex& b = verts[static_cast<size_t>(e.second)];
        Vec2 d = b.xy - a.xy; if (d.length() < 1e-9) continue; Vec2 out = Vec2(d.y, -d.x) / d.length();   // interior on the left => outward is the right normal
        if (seam && (*seam)(a.xy, b.xy, 0.5 * (a.z + b.z) + lift)) continue;                          // an internal seam: the asphalt continues at this level, no side face
        const double topA = a.z + lift, topB = b.z + lift;
        if (hidden && (*hidden)((a.xy + b.xy) * 0.5 + out * 0.15, 0.5 * (topA + topB))) continue;
        double botA = topA - thick, botB = topB - thick;
        if (groundAt) {
            const double gA = (*groundAt)(a.xy + out * 0.3), gB = (*groundAt)(b.xy + out * 0.3);
            if (topA - gA < 0.03 && topB - gB < 0.03) continue;
            botA = std::max(botA, std::min(topA, gA) - 0.05);
            botB = std::max(botB, std::min(topB, gB) - 0.05);
        }
        sides.push_back({e.first, e.second, topA, topB, botA, botB});
    }
    // chain: side i continues at the side that starts where it ends (only where that is unambiguous)
    std::unordered_map<int, int> startsAt; startsAt.reserve(sides.size());
    for (size_t i = 0; i < sides.size(); ++i) { auto [it, fresh] = startsAt.try_emplace(sides[i].a, static_cast<int>(i)); if (!fresh) it->second = -1; }
    std::vector<char> used(sides.size(), 0);
    auto emitRun = [&](const std::vector<int>& run) {
        const Side& f = sides[static_cast<size_t>(run.front())]; const Side& l = sides[static_cast<size_t>(run.back())];
        const Vec2 A = verts[static_cast<size_t>(f.a)].xy, B = verts[static_cast<size_t>(l.b)].xy;
        Vec2 d = B - A; const Vec2 out = Vec2(d.y, -d.x) / d.length();
        MeshBuilder::emitQuad(side, world(A, f.topA), world(B, l.topB), world(B, l.botB), world(A, f.botA),
                              Vec3(static_cast<Real>(out.x), 0, static_cast<Real>(out.y)), sideColor);
    };
    for (size_t i0 = 0; i0 < sides.size(); ++i0) {
        if (used[i0]) continue;
        std::vector<int> run{static_cast<int>(i0)}; used[i0] = 1;
        while (run.size() < 256) {
            const Side& last = sides[static_cast<size_t>(run.back())];
            auto it = startsAt.find(last.b); if (it == startsAt.end() || it->second < 0 || used[static_cast<size_t>(it->second)]) break;
            const Side& nx = sides[static_cast<size_t>(it->second)];
            // would the run's single quad, start to nx's end, still pass through every vertex it covers?
            const Side& f = sides[static_cast<size_t>(run.front())];
            const Vec2 A = verts[static_cast<size_t>(f.a)].xy, B = verts[static_cast<size_t>(nx.b)].xy;
            const Vec2 d = B - A; const double len = d.length(); if (len < 1e-9) break;
            const Vec2 u = d / len;
            bool ok = true;
            auto check = [&](int vi, double top, double bot) {
                const Vec2 q = verts[static_cast<size_t>(vi)].xy - A; const double t = dot(q, u) / len;
                if (t <= 0.0 || t >= 1.0 || std::fabs(cross(u, q)) > 0.01) return false;
                const double topL = f.topA + (nx.topB - f.topA) * t, botL = f.botA + (nx.botB - f.botA) * t;
                return std::fabs(topL - top) < 0.01 && botL <= bot + 0.01 && botL >= bot - 0.5;   // the top exact, the bottom no shallower
            };
            for (size_t k = 0; k < run.size() && ok; ++k) {
                const Side& s0 = sides[static_cast<size_t>(run[k])];
                const Side& s1 = k + 1 < run.size() ? sides[static_cast<size_t>(run[k + 1])] : nx;
                ok = check(s0.b, s0.topB, std::min(s0.botB, s1.botA));
            }
            if (!ok) break;
            used[static_cast<size_t>(it->second)] = 1; run.push_back(it->second);
        }
        emitRun(run);
    }
}

// A WALL swept along a chain of points: mitred corners (clamped at 2x for sharp ones), closed end caps,
// per-point lateral offsets (positive = the chain's right normal = outward on a CCW deck boundary) so a
// run can pass from a freeway edge (2.5 m shoulder) onto a ramp edge (1 m) without a step. Outer face,
// inner face, top and caps; the bottom sits on the slab. Every wall in the lab should come through here
// (Glenn: "missing end caps and there are cracks between the walls").
// `taperStart` / `taperEnd`: that end ramps down to nothing over this many metres instead of stopping in a
// full-height vertical face. A parapet stops where its deck drops under the 1.5 m rule, which lands
// mid-carriageway on a descending viaduct; a blunt 0.9 m block there is a wall a car drives into ("the
// freeway walls end up blocking the car", Glenn, 2026-09-07). A real barrier ends that way, sloped down.
// PER END, not per run: tapering every open end made continuous walls read as a row of wedges ("the walls
// now randomly taper instead of being consistently straight", 2026-09-08). Only an end that would otherwise
// leave a face standing on drivable pavement is tapered; a run that ends at its deck's own end stays square.
void sweepWall(RenderMesh& mesh, const std::vector<Vec2>& pts, const std::vector<double>& z, const std::vector<double>& offOuter,
               const std::vector<double>& offInner, double height, const Vec3& color, bool closed, double taperStart = 0.0, double taperEnd = 0.0) {
    const size_t n = pts.size(); if (n < 2) return;
    std::vector<double> hv(n, height);
    if (!closed && (taperStart > 0.0 || taperEnd > 0.0)) {
        std::vector<double> st(n, 0.0); for (size_t i = 1; i < n; ++i) st[i] = st[i - 1] + (pts[i] - pts[i - 1]).length();
        const double S = st.back(), cap = S * 0.4;   // a short run tapers over its own length, never more than 40 % each end
        const double t0 = std::min(taperStart, cap), t1 = std::min(taperEnd, cap);
        for (size_t i = 0; i < n; ++i) {
            double f = 1.0;
            if (t0 > 1e-6) f = std::min(f, st[i] / t0);
            if (t1 > 1e-6) f = std::min(f, (S - st[i]) / t1);
            hv[i] = height * std::min(1.0, f);
        }
    }
    auto unit = [](Vec2 v) { const double l = v.length(); return l > 1e-9 ? v / l : Vec2(1, 0); };
    std::vector<Vec2> nrm(n);
    for (size_t i = 0; i < n; ++i) {
        const Vec2 dPrev = unit(i > 0 ? pts[i] - pts[i - 1] : (closed ? pts[0] - pts[n - 1] : pts[1] - pts[0]));
        const Vec2 dNext = unit(i + 1 < n ? pts[i + 1] - pts[i] : (closed ? pts[0] - pts[n - 1] : pts[n - 1] - pts[n - 2]));
        const Vec2 nPrev(dPrev.y, -dPrev.x), nNext(dNext.y, -dNext.x); Vec2 m = nPrev + nNext; const double ml = m.length();
        if (ml < 1e-6) nrm[i] = nNext; else { m = m / ml; const double c = dot(m, nNext); nrm[i] = m * (c > 0.5 ? 1.0 / c : 2.0); }
    }
    auto P = [&](size_t i, double off, double dz) { return world(pts[i] + nrm[i] * off, z[i] + dz); };
    const size_t segs = closed ? n : n - 1;
    for (size_t i = 0; i < segs; ++i) {
        const size_t j = (i + 1) % n; const Vec2 out = unit(Vec2((pts[j] - pts[i]).y, -(pts[j] - pts[i]).x));
        const Vec3 nOut(static_cast<Real>(out.x), 0, static_cast<Real>(out.y));
        if (hv[i] < 1e-4 && hv[j] < 1e-4) continue;
        MeshBuilder::emitQuad(mesh, P(i, offOuter[i], 0), P(j, offOuter[j], 0), P(j, offOuter[j], hv[j]), P(i, offOuter[i], hv[i]), nOut, color);
        MeshBuilder::emitQuad(mesh, P(j, offInner[j], 0), P(i, offInner[i], 0), P(i, offInner[i], hv[i]), P(j, offInner[j], hv[j]), nOut * -1.0f, color);
        MeshBuilder::emitQuad(mesh, P(i, offOuter[i], hv[i]), P(i, offInner[i], hv[i]), P(j, offInner[j], hv[j]), P(j, offOuter[j], hv[j]), Vec3(0, 1, 0), color);
    }
    if (!closed && (hv.front() > 1e-4 || hv.back() > 1e-4)) {
        const Vec2 t0 = unit(pts[1] - pts[0]), t1 = unit(pts[n - 1] - pts[n - 2]);
        if (hv.front() > 1e-4) MeshBuilder::emitQuad(mesh, P(0, offInner[0], 0), P(0, offOuter[0], 0), P(0, offOuter[0], hv.front()), P(0, offInner[0], hv.front()), Vec3(static_cast<Real>(-t0.x), 0, static_cast<Real>(-t0.y)), color);
        if (hv.back() > 1e-4) MeshBuilder::emitQuad(mesh, P(n - 1, offOuter[n - 1], 0), P(n - 1, offInner[n - 1], 0), P(n - 1, offInner[n - 1], hv.back()), P(n - 1, offOuter[n - 1], hv.back()), Vec3(static_cast<Real>(t1.x), 0, static_cast<Real>(t1.y)), color);
    }
}

// A steel guardrail: a beam swept along the run and a post every `postGap` metres down to the deck. Used
// where a freeway or ramp sits AT GRADE and there is nothing to fall off — a solid parapet there reads as a
// wall through a field, and 13 km of it on metro would (Glenn, 2026-09-08).
// The mitred offset of a polyline, the same construction sweepWall uses: at each joint the normal is the
// bisector scaled by 1/cos, so an offset line keeps pace with a curve instead of stepping in and out. Using
// the raw per-segment normal made the guardrail zigzag round the ring ("I was hoping those walls would
// follow the curve of the road and not go crazy", Glenn, 2026-09-08).
std::vector<Vec2> miterOffset(const std::vector<Vec2>& pts, const std::vector<double>& off) {
    const size_t n = pts.size(); std::vector<Vec2> out(n);
    auto unit = [](Vec2 v) { const double l = v.length(); return l > 1e-9 ? v / l : Vec2(1, 0); };
    for (size_t i = 0; i < n; ++i) {
        const Vec2 dPrev = unit(i > 0 ? pts[i] - pts[i - 1] : pts[1] - pts[0]);
        const Vec2 dNext = unit(i + 1 < n ? pts[i + 1] - pts[i] : pts[n - 1] - pts[n - 2]);
        const Vec2 nPrev(dPrev.y, -dPrev.x), nNext(dNext.y, -dNext.x); Vec2 m = nPrev + nNext; const double ml = m.length();
        Vec2 nrm = nNext;
        if (ml >= 1e-6) { m = m / ml; const double c = dot(m, nNext); nrm = m * (c > 0.5 ? 1.0 / c : 2.0); }
        out[i] = pts[i] + nrm * off[i];
    }
    return out;
}

// A steel guardrail: a beam swept along the run and a post every `postGap` metres down to the deck. Used
// where a freeway or ramp sits AT GRADE and there is nothing to fall off — a solid parapet there reads as a
// wall through a field, and 13 km of it on metro would (Glenn, 2026-09-08). The beam is swept from the run's
// own points so sweepWall does the mitring; only the posts need the offset line.
void sweepGuardrail(RenderMesh& mesh, const std::vector<Vec2>& pts, const std::vector<double>& z, const std::vector<double>& off,
                    double height, const Vec3& color, double postGap = 4.0) {
    const size_t n = pts.size(); if (n < 2) return;
    const double beamTop = height, beamBot = std::max(0.25, height - 0.32), halfT = 0.05;
    std::vector<double> oo(n), oi(n), zb(n);
    for (size_t i = 0; i < n; ++i) { oo[i] = off[i] + halfT; oi[i] = off[i] - halfT; zb[i] = z[i] + beamBot; }
    sweepWall(mesh, pts, zb, oo, oi, beamTop - beamBot, color, false);       // the beam, mitred by sweepWall
    const std::vector<Vec2> line = miterOffset(pts, off);                    // posts stand on the same line
    double acc = 0;
    for (size_t i = 0; i + 1 < n; ++i) {
        const double seg = (line[i + 1] - line[i]).length(); if (seg < 1e-9) continue;
        for (double t = std::fmod(postGap - acc, postGap); t < seg; t += postGap) {
            const double u = t / seg; const Vec2 p = line[i] + (line[i + 1] - line[i]) * u;
            const double zt = z[i] + (z[i + 1] - z[i]) * u;
            RenderMesh post = MeshBuilder::box(Vec3(0.11f, static_cast<float>(beamTop), 0.11f));
            MeshBuilder::appendTransformed(mesh, post, Mat4::translate(static_cast<Real>(p.x), static_cast<Real>(zt + beamTop / 2), static_cast<Real>(p.y)));
        }
        acc = std::fmod(acc + seg, postGap);
    }
}

void strip(RenderMesh& mesh, const std::vector<Vec2>& pts, const std::vector<double>& z, double halfWidth, double dash, double gap, const Vec3& color) {
    double acc = 0;
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        Vec2 d = pts[i+1] - pts[i]; double L = d.length(); if (L < 1e-9) continue; Vec2 n = perp(d / L) * halfWidth;
        if (dash <= 0 || std::fmod(acc, dash + gap) < dash)
            MeshBuilder::emitQuad(mesh, world(pts[i] - n, z[i]), world(pts[i+1] - n, z[i+1]), world(pts[i+1] + n, z[i+1]), world(pts[i] + n, z[i]), Vec3(0, 1, 0), color);
        acc += L;
    }
}

}  // namespace

std::vector<NamedMesh> buildMeshes(const Result& r) {
    const RoadLabGraph& g = r.graph; const LaneSet& L = r.lanes; const DeckHeight& H = *r.heights;
    RenderMesh asphalt, concrete, sidewalk, shoulder, median, paintW, paintY, terrain, guardrail;
    // The ground under a deck, for the underside rule (slab): the conformed grid, or 0 on a flat level.
    const std::function<double(const Vec2&)> groundUnder = [&r](const Vec2& q) {
        return r.hasTerrain ? r.terrain.sample(q.x, q.y) : 0.0;
    };
    // decks with lane-local UVs
    // A boundary edge is only a deck EDGE if the pavement ends there. A weld crack is also "used by one
    // triangle" and so also a boundary edge — but the asphalt continues on its far side, and a parapet
    // or a slab side face on it is a thin vertical wall in the middle of a lane (Glenn, driving: "a very
    // thin vertical edge in the middle of a freeway lane"). Test a point just beyond the edge.
    // Level-aware: the pavement beyond the edge must be at the SAME height — a street passing under
    // the viaduct is pavement in plan but not a continuation of this deck.
    // Every point query below asks "which lanes' footprints contain q": the boxes once, a grid of them, never
    // a bounds() over a footprint's vertices per call (that was a 100 s stall on metro).
    const std::vector<Box2> laneBox = laneBoxes(r.pavement.footprints); const LaneGrid laneGrid(laneBox, r.pavement.footprints); const std::vector<PreparedSet> footprint = prepareAll(r.pavement.footprints);
    auto sameLevelPavedAt = [&](const Vec2& q, double z) {
        for (int ojI : laneGrid.at(q)) {
            const size_t oj = static_cast<size_t>(ojI); const Box2& bb = laneBox[oj];
            if (q.x < bb.minX || q.x > bb.maxX || q.y < bb.minY || q.y > bb.maxY || !footprint[oj].contains(q)) continue;
            if (std::fabs(H.deck(ojI, q) - z) < 1.0) return true;
        }
        return false;
    };
    auto internalSeam = [&](const Vec2& a, const Vec2& b, double z) {
        Vec2 d = b - a; const double len = d.length(); if (len < 1e-9) return false; d = d / len; const Vec2 out(d.y, -d.x);
        return sameLevelPavedAt((a + b) * 0.5 + out * 0.35, z);
    };
    const std::function<bool(const Vec2&, const Vec2&, double)> seamFn = internalSeam;
    // What stands beyond a slab's edge, for slab()'s side rule: a surface whose top at q is at or up to
    // half a metre above `top` hides the side (asphalt under a sidewalk's edge, a shoulder's edge against
    // the lane it continues). One much higher is a bridge over it and hides nothing. Each layer is split
    // into its polygons so a point tests only the few whose box covers it.
    struct Cover { std::vector<PolySet> polys; std::vector<Box2> box; std::vector<PreparedSet> prep; std::vector<int> roads; double lift; };
    auto coverOf = [&](const PolySet& set, double lift, bool anyRoad) {
        Cover c; for (const Polygon2& p : set) c.polys.push_back(PolySet{p});
        c.box = laneBoxes(c.polys); c.prep = prepareAll(c.polys); c.roads = layerRoads(g, anyRoad); c.lift = lift; return c;
    };
    const Cover covers[3] = {coverOf(r.pavement.sidewalk, kSidewalkLift, false), coverOf(r.pavement.shoulder, 0.0, false), coverOf(r.pavement.median, 0.10, true)};
    const LaneGrid coverGrid[3] = {LaneGrid(covers[0].box, covers[0].polys), LaneGrid(covers[1].box, covers[1].polys), LaneGrid(covers[2].box, covers[2].polys)};
    const std::function<bool(const Vec2&, double)> hiddenFn = [&](const Vec2& q, double top) {
        auto hides = [&](double z) { return z >= top - 0.02 && z <= top + 0.5; };
        for (int li : laneGrid.at(q)) {
            const Box2& bb = laneBox[static_cast<size_t>(li)];
            if (q.x < bb.minX || q.x > bb.maxX || q.y < bb.minY || q.y > bb.maxY || !footprint[static_cast<size_t>(li)].contains(q)) continue;
            if (hides(H.deck(li, q))) return true;
        }
        for (int k = 0; k < 3; ++k) {
            const Cover& c = covers[k];
            for (int pi : coverGrid[k].at(q)) {
                const Box2& bb = c.box[static_cast<size_t>(pi)];
                if (q.x < bb.minX || q.x > bb.maxX || q.y < bb.minY || q.y > bb.maxY || !c.prep[static_cast<size_t>(pi)].contains(q)) continue;
                if (hides(H.layerHeight(c.roads, q) + c.lift)) return true;
            }
        }
        return false;
    };
    // RT_DECK_STATS=1: what each stage adds to the concrete mesh (triangles), to see where it goes.
    static const bool deckStats = std::getenv("RT_DECK_STATS") != nullptr;
    std::vector<std::pair<const char*, std::size_t>> concreteBy;
    auto mark = [&](const char* stage) { if (deckStats) concreteBy.push_back({stage, concrete.indices.size() / 3}); };
    mark("start");
    for (const Surface& s : r.pavement.decks) {
        std::vector<std::array<float, 6>> uv(s.tris.size());
        for (size_t ti = 0; ti < s.tris.size(); ++ti) {
            const Lane& owner = L.lanes[static_cast<size_t>(s.triOwner[ti])];
            for (int k = 0; k < 3; ++k) {
                const DeckVertex& v = s.verts[static_cast<size_t>(s.tris[ti][static_cast<size_t>(k)])]; Projection pr = project(owner.xy, owner.s, v.xy);
                Vec2 q = pointAt(owner.xy, owner.s, pr.station); Vec2 t = tangentAtStation(owner.xy, owner.s, pr.station);
                double lateral = cross(t, v.xy - q);
                uv[ti][static_cast<size_t>(2 * k)] = static_cast<float>(lateral); uv[ti][static_cast<size_t>(2 * k + 1)] = static_cast<float>(pr.station);
            }
        }
        slab(asphalt, concrete, s.verts, s.tris, s.boundary, s.thick, 0.0, kAsphalt, kConcrete, &uv, &seamFn, &groundUnder, &hiddenFn);
    }
    // layers
    auto layer = [&](const PolySet& poly, RenderMesh& top, const Vec3& color, double lift, double thick, bool anyRoad) {
        const std::vector<int> roads = layerRoads(g, anyRoad);
        for (const FlatMesh& m : layerMeshes(g, H, poly, roads)) slab(top, concrete, m.verts, m.tris, m.boundary, thick, lift, color, kConcrete, nullptr, nullptr, &groundUnder, &hiddenFn);
    };
    mark("deck sides");
    layer(r.pavement.sidewalk, sidewalk, kSidewalk, kSidewalkLift, 0.42, false); mark("sidewalk sides");
    layer(r.pavement.shoulder, shoulder, kShoulder, 0.0, 0.3, false); mark("shoulder sides");
    layer(r.pavement.median, median, kMedian, 0.10, 0.40, true); mark("median sides");
    // Parapets and girders on elevated decks (Glenn: "the elevated ones need walls so you don't drive
    // off", "an undercarriage"). The runs come from parapetRuns() so a diagnostic sees exactly what is swept;
    // every elevated freeway/ramp lane also gets a box girder hung from the slab.
    for (const ParapetRun& run : parapetRuns(r)) {
        if (run.kind == BarrierKind::Guardrail) sweepGuardrail(guardrail, run.pts, run.z, run.offOuter, run.height, kGuardrail);
        else sweepWall(concrete, run.pts, run.z, run.offOuter, run.offInner, run.height, kConcrete, run.closed,
                       run.capOnPavement[0] ? 8.0 : 0.0, run.capOnPavement[1] ? 8.0 : 0.0);   // 8 m terminal only where a cap would stand on road
    }
    mark("parapet walls");
    auto groundAt = [&](const Vec2& p) { return r.hasTerrain ? r.terrain.sample(p.x, p.y) : 0.0; };
    auto isDeckLane = [&](int li) {
        if (li < 0 || li >= static_cast<int>(L.lanes.size())) return false; const Lane& l = L.lanes[static_cast<size_t>(li)];
        if (l.isConnector() || l.parent < 0) return false; const EdgeSpec& e = r.graph.edges[static_cast<size_t>(l.parent)]; return e.isRamp() || e.cls == "freeway";
    };
    for (size_t li = 0; li < L.lanes.size(); ++li) {
        if (!isDeckLane(static_cast<int>(li))) continue; const Lane& l = L.lanes[li]; if (l.s.size() < 2 || l.s.back() < 8) continue;
        const double slabThick = r.graph.cls(r.graph.edges[static_cast<size_t>(l.parent)]).thick, gw = 0.7, gd = 1.6;
        int n = std::max(2, static_cast<int>(l.s.back() / 4.0) + 1); bool prevOn = false; Vec2 pp, pn; double pz = 0;
        for (int k = 0; k < n; ++k) {
            const double st = l.s.back() * k / (n - 1); const Vec2 p = pointAt(l.xy, l.s, st); const Vec2 nrm = perp(tangentAtStation(l.xy, l.s, st));
            const double zTop = H.deck(static_cast<int>(li), p), z = zTop - slabThick; const bool on = zTop - groundAt(p) > 1.5;
            if (on && prevOn) {
                MeshBuilder::emitQuad(concrete, world(pp + pn * gw, pz), world(p + nrm * gw, z), world(p + nrm * gw, z - gd), world(pp + pn * gw, pz - gd), Vec3(static_cast<Real>(nrm.x), 0, static_cast<Real>(nrm.y)), kConcrete);
                MeshBuilder::emitQuad(concrete, world(p - nrm * gw, z), world(pp - pn * gw, pz), world(pp - pn * gw, pz - gd), world(p - nrm * gw, z - gd), Vec3(static_cast<Real>(-nrm.x), 0, static_cast<Real>(-nrm.y)), kConcrete);
                MeshBuilder::emitQuad(concrete, world(pp - pn * gw, pz - gd), world(p - nrm * gw, z - gd), world(p + nrm * gw, z - gd), world(pp + pn * gw, pz - gd), Vec3(0, -1, 0), kConcrete);
            }
            prevOn = on; pp = p; pn = nrm; pz = z;
        }
    }
    mark("girders");
    // piers
    for (const Pier& p : r.piers) {
        double h = p.z1 - p.z0; if (h <= 0) continue; RenderMesh box = MeshBuilder::box(Vec3(2.6f, static_cast<float>(h), 2.6f));
        MeshBuilder::appendTransformed(concrete, box, Mat4::translate(static_cast<Real>(p.xy.x), static_cast<Real>(p.z0 + h / 2), static_cast<Real>(p.xy.y)));
    }
    mark("piers");
    if (deckStats) {
        std::string o;
        for (std::size_t k = 1; k < concreteBy.size(); ++k)
            o += std::string(" ") + concreteBy[k].first + " " + std::to_string(concreteBy[k].second - concreteBy[k - 1].second);
        LOG_INFO << "[deck stats] concrete triangles by stage:" << o << "; asphalt " << asphalt.indices.size() / 3
                 << " (" << asphalt.vertices.size() << " verts), hasTerrain " << r.hasTerrain;
    }
    // Junction boxes: a point on a street lane that lies inside a CROSSING road's lane at the same level
    // is inside an intersection. Lane paint stops there (lane lines running through a junction read as
    // a mess), and the spans of it along each street get a zebra crosswalk and a stop bar per approach.
    // Parallel overlaps (a gore, an aux lane beside its host) and grade-separated overlaps (a street
    // under a viaduct) are not junctions. Returns the crossing road's rank, or -1.
    auto inBox = [&](size_t li, const Vec2& p, const Vec2& t) {
        const Lane& l = L.lanes[li]; if (l.isConnector()) return -1;
        const double zl = H.deck(static_cast<int>(li), p); int best = -1;
        for (int ojI : laneGrid.at(p)) {
            const size_t oj = static_cast<size_t>(ojI); const Lane& o = L.lanes[oj];
            if (oj == li || o.isConnector() || o.parent == l.parent || o.parent < 0 || r.pavement.footprints[oj].empty()) continue;
            const Box2& b = laneBox[oj];
            if (p.x < b.minX || p.x > b.maxX || p.y < b.minY || p.y > b.maxY || !footprint[oj].contains(p)) continue;
            if (std::fabs(H.deck(static_cast<int>(oj), p) - zl) > 1.0) continue;
            Projection pr = project(o.xy, o.s, p);
            if (std::fabs(dot(t, tangentAtStation(o.xy, o.s, pr.station))) > 0.7) continue;
            best = std::max(best, r.graph.cls(r.graph.edges[static_cast<size_t>(o.parent)]).rank);
        }
        return best;
    };
    // Crosswalks and stop bars: per street, the parent-station spans where its lanes run through a box.
    struct Span { double s0, s1; int crossRank; };
    std::map<int, std::vector<Span>> spans;
    for (size_t li = 0; li < L.lanes.size(); ++li) {
        const Lane& l = L.lanes[li]; if (l.isConnector() || l.parent < 0 || l.s.size() < 2) continue;
        const EdgeSpec& e = r.graph.edges[static_cast<size_t>(l.parent)]; if (e.isRamp() || e.cls == "freeway" || r.graph.cls(e).rank < 1) continue;
        int n = std::max(2, static_cast<int>(l.s.back() / 2.0) + 1); double in0 = -1, inLast = 0; int rankMax = -1;
        for (int k = 0; k <= n; ++k) {
            int cr = -1; double st = 0;
            if (k < n) { double ls = l.s.back() * k / (n - 1); Vec2 p = pointAt(l.xy, l.s, ls); Vec2 t = tangentAtStation(l.xy, l.s, ls); cr = inBox(li, p, t); st = project(e.xy, e.s, p).station; }
            if (cr >= 0) { if (in0 < 0) { in0 = st; rankMax = cr; } inLast = st; rankMax = std::max(rankMax, cr); }
            else if (in0 >= 0) { spans[l.parent].push_back({std::min(in0, inLast), std::max(in0, inLast), rankMax}); in0 = -1; }
        }
    }
    std::map<int, std::vector<Span>> boxes;   // parent edge -> merged junction spans
    for (auto& [pe, raw] : spans) {
        std::sort(raw.begin(), raw.end(), [](const Span& a, const Span& b) { return a.s0 < b.s0; });
        std::vector<Span>& merged = boxes[pe];
        for (const Span& sp : raw) { if (!merged.empty() && sp.s0 <= merged.back().s1 + 12.0 /* a divided road is ONE junction: its carriageways sit a median apart */) { merged.back().s1 = std::max(merged.back().s1, sp.s1); merged.back().crossRank = std::max(merged.back().crossRank, sp.crossRank); } else merged.push_back(sp); }
    }
    // a point on a street lane inside one of its road's merged boxes (the median gap of a divided cross road included)
    auto inSpan = [&](const Lane& l, const Vec2& p) {
        auto it = boxes.find(l.parent); if (it == boxes.end()) return false;
        const EdgeSpec& e = r.graph.edges[static_cast<size_t>(l.parent)]; const double st = project(e.xy, e.s, p).station;
        for (const Span& sp : it->second) if (st >= sp.s0 - 0.5 && st <= sp.s1 + 0.5) return true;
        return false;
    };
    // markings: each lane edge classified by what lies just outside it
    for (size_t li = 0; li < L.lanes.size(); ++li) {
        const Lane& l = L.lanes[li];
        for (int side = 0; side < 2; ++side) {
            const std::vector<Vec2>& E = side == 0 ? l.left : l.right; std::vector<Vec2> pts; std::vector<double> z; std::vector<int> style;   // 0 edge, 1 lane, 2 centre
            std::vector<double> Es = stations(E); int n = std::max(2, static_cast<int>(Es.back() / 4.0) + 1);
            for (int k = 0; k < n; ++k) {
                double st = Es.back() * k / (n - 1); Vec2 p = pointAt(E, Es, st); Vec2 t = tangentAtStation(E, Es, st); Vec2 nrm = perp(t) * (side == 0 ? 1.0 : -1.0); Vec2 q = p + nrm * 0.3;
                int sty = (inBox(li, p, t) >= 0 || inSpan(l, p)) ? 3 : 0;   // 3: inside a junction box, no paint
                for (int ojI : laneGrid.at(q)) {
                    if (sty != 0) break; const size_t oj = static_cast<size_t>(ojI);
                    if (oj == li || r.pavement.footprints[oj].empty()) continue; const Box2& b = laneBox[oj];
                    if (q.x < b.minX || q.x > b.maxX || q.y < b.minY || q.y > b.maxY || !footprint[oj].contains(q)) continue;
                    const Lane& o = L.lanes[oj]; Projection pr = project(o.xy, o.s, q);
                    double same = dot(t * static_cast<double>(l.dir), tangentAtStation(o.xy, o.s, pr.station) * static_cast<double>(o.dir)); sty = same > 0 ? 1 : 2;
                }
                pts.push_back(p); z.push_back(H.deck(static_cast<int>(li), p) + 0.02); style.push_back(sty);
            }
            size_t i = 0;
            while (i < pts.size()) {
                size_t j = i; while (j + 1 < pts.size() && style[j + 1] == style[i]) ++j;
                if (j > i) {
                    std::vector<Vec2> seg(pts.begin() + static_cast<long>(i), pts.begin() + static_cast<long>(j) + 1); std::vector<double> sz(z.begin() + static_cast<long>(i), z.begin() + static_cast<long>(j) + 1);
                    if (style[i] == 3) {} else if (style[i] == 1) strip(paintW, seg, sz, 0.06, 3.0, 6.0, kPaintWhite); else if (style[i] == 2) strip(paintY, seg, sz, 0.07, 0, 0, kPaintYellow); else strip(paintW, seg, sz, 0.08, 0, 0, kPaintWhite);
                }
                i = j + 1;
            }
        }
    }
    for (auto& [pe, merged] : boxes) {
        const EdgeSpec& e = r.graph.edges[static_cast<size_t>(pe)]; const double Lr = e.s.back(); const int ownRank = r.graph.cls(e).rank;
        auto paintSide = [&](double sBox, int sign, int crossRank) {   // sign -1: the approach at stations below the box; +1: above
            const double sw0 = sBox + sign * 1.0, sw1 = sBox + sign * 4.0;
            if (std::min(sw0, sw1) < 2.0 || std::max(sw0, sw1) > Lr - 2.0) return;   // the road ends in this box: no leg here
            const double stMid = 0.5 * (sw0 + sw1); Vec2 P = pointAt(e.xy, e.s, stMid); Vec2 T = tangentAtStation(e.xy, e.s, stMid); Vec2 N = perp(T);
            struct LL { int li; double lat, w; bool approaching; }; std::vector<LL> ls;
            for (size_t li = 0; li < L.lanes.size(); ++li) {
                const Lane& l = L.lanes[li]; if (l.parent != pe || l.isConnector() || l.s.size() < 2) continue;
                Projection pr = project(l.xy, l.s, P); if (pr.station < 1.0 || pr.station > l.s.back() - 1.0 || pr.distance > 30.0) continue;
                Vec2 c = pointAt(l.xy, l.s, pr.station); double lat = dot(c - P, N);
                bool towardIncreasing = dot(tangentAtStation(l.xy, l.s, pr.station) * static_cast<double>(l.dir), T) > 0;
                ls.push_back({static_cast<int>(li), lat, l.w, sign < 0 ? towardIncreasing : !towardIncreasing});
            }
            if (ls.empty()) return;
            double latMin = 1e9, latMax = -1e9; for (const LL& x : ls) { latMin = std::min(latMin, x.lat - x.w / 2); latMax = std::max(latMax, x.lat + x.w / 2); }
            auto laneAt = [&](double lat) { int best = ls[0].li; double bd = 1e300; for (const LL& x : ls) { double d = std::fabs(x.lat - lat); if (d < bd) { bd = d; best = x.li; } } return best; };
            auto zAt = [&](int li, const Vec2& q) { return H.deck(li, q) + 0.025; };
            // zebra: stripes along the road, 0.5 m on a 1 m pitch, across the paved width
            Vec2 A = pointAt(e.xy, e.s, sw0), B = pointAt(e.xy, e.s, sw1);
            for (double lat = latMin + 0.55; lat + 0.25 <= latMax - 0.3; lat += 1.0) {
                int li = laneAt(lat); Vec2 a = A + N * lat, b = B + N * lat;
                strip(paintW, std::vector<Vec2>{a, b}, std::vector<double>{zAt(li, a), zAt(li, b)}, 0.25, 0, 0, kPaintWhite);
            }
            // LANE ARROWS (Glenn, 2026-09-25: "lane signs like arrows for where to turn ... they should be
            // a part of the road"): what each approaching lane may do at this junction, painted in it twice
            // -- 7 m and 30 m before the stop line. The junction's LEGS are the street edges that leave its
            // box at this level (a road through it gives two, one ending in it one); each leg's angle from
            // the approach's heading makes it a left, a straight or a right. The movements go to the lanes
            // by position, the usual way: the leftmost lane turns left (and goes straight, with a lane
            // beside it that does too), the rightmost turns right, the middle ones go straight; a turn
            // pocket turns only. Painted where there is a choice of lanes (two or more approaching). The
            // glyphs are built in plan coordinates pointing at the real legs, so the world's mirror
            // (interchange.h) flips them with the roads.
            {
                std::vector<const LL*> app;
                for (const LL& x : ls) if (x.approaching) app.push_back(&x);
                const double sStop = sBox + sign * 5.2;
                const Vec2 D = T * static_cast<double>(-sign);   // travel into the box
                const Vec2 Lft = perp(D);                         // the plan's left of travel
                // the approach lies at stations on the `sign` side of the box: back from the stop line is +sign
                if (app.size() >= 2 && std::min(sStop, sStop + sign * 40.0) > 2.0 && std::max(sStop, sStop + sign * 40.0) < Lr - 2.0) {
                    // the box, from just inside it
                    const Vec2 C = pointAt(e.xy, e.s, sBox) + D * 6.0;   // 6 m into the box
                    const double zC = H.deck(ls[0].li, pointAt(e.xy, e.s, sStop));
                    std::set<int> parents;
                    for (const Vec2& q : {C, C + D * 12.0, C + Lft * 12.0, C - Lft * 12.0, C + D * 6.0})
                        for (int ojI : laneGrid.at(q)) { const Lane& o = L.lanes[static_cast<size_t>(ojI)]; if (!o.isConnector() && o.parent >= 0) parents.insert(o.parent); }
                    bool hasL = false, hasS = false, hasR = false;
                    for (int pe2 : parents) {
                        const EdgeSpec& e2 = r.graph.edges[static_cast<size_t>(pe2)];
                        if (e2.isRamp() || e2.cls == "freeway" || e2.s.size() < 2) continue;
                        const Projection pr2 = project(e2.xy, e2.s, C);
                        if (pr2.distance > 25.0) continue;
                        // at this level (not a viaduct overhead or a road underneath)
                        int anyLane = -1;
                        for (size_t lj = 0; lj < L.lanes.size() && anyLane < 0; ++lj) if (L.lanes[lj].parent == pe2) anyLane = static_cast<int>(lj);
                        if (anyLane < 0 || std::fabs(H.deck(anyLane, pointAt(e2.xy, e2.s, pr2.station)) - zC) > 1.5) continue;
                        for (int w2 : {-1, +1}) {
                            const double sLeg = pr2.station + w2 * 30.0;
                            if (sLeg < 0.0 || sLeg > e2.s.back()) continue;   // the edge ends in this junction that way
                            const Vec2 V = pointAt(e2.xy, e2.s, sLeg) - C;
                            if (V.length() < 12.0) continue;
                            const Vec2 v = V / V.length();
                            const double a = std::atan2(cross(D, v), dot(D, v)) * 180.0 / 3.14159265358979;
                            if (std::fabs(a) > 150.0) continue;          // the way it came
                            if (std::fabs(a) < 35.0) hasS = true; else if (a > 0) hasL = true; else hasR = true;
                        }
                    }
                    // the lanes, plan-left first
                    std::sort(app.begin(), app.end(), [&](const LL* a, const LL* b) { return dot(N * a->lat, Lft) > dot(N * b->lat, Lft); });
                    const std::size_t n = app.size();
                    const std::vector<LaneMoves> moves = assignLaneMoves(n, hasL, hasS, hasR,
                        L.lanes[static_cast<size_t>(app.front()->li)].kind == "turn", L.lanes[static_cast<size_t>(app.back()->li)].kind == "turn");
                    for (std::size_t i = 0; i < n; ++i) {
                        const bool mL = moves[i].left, mS = moves[i].straight, mR = moves[i].right;
                        if (!mL && !mS && !mR) continue;
                        // the glyph in its lane's frame: u forward from the tail, v to the plan's left
                        // the second arrow only where the block is long enough that the next junction's
                        // arrows (painted back from ITS stop line) don't meet it mid-block
                        double gap = sign < 0 ? sBox : Lr - sBox;
                        for (const Span& o : merged) {
                            if (sign < 0 && o.s1 < sBox - 1.0) gap = std::min(gap, sBox - o.s1);
                            if (sign > 0 && o.s0 > sBox + 1.0) gap = std::min(gap, o.s0 - sBox);
                        }
                        for (double back : {7.0, 30.0}) {
                            if (back > 10.0 && gap < 110.0) continue;
                            const double sTail = sStop + sign * (back + 5.0);
                            const Vec2 base = pointAt(e.xy, e.s, sTail) + N * app[i]->lat;
                            auto P = [&](double u, double v) { return base + D * u + Lft * v; };
                            auto Z = [&](const Vec2& q) { return H.deck(app[i]->li, q) + 0.025; };
                            auto shaft = [&](std::initializer_list<std::pair<double, double>> uv) {
                                std::vector<Vec2> pts; std::vector<double> z;
                                for (const auto& [u, v] : uv) { pts.push_back(P(u, v)); z.push_back(Z(pts.back())); }
                                strip(paintW, pts, z, 0.15, 0, 0, kPaintWhite);
                            };
                            auto head = [&](double u, double v, double du, double dv, double len) {   // base centre, pointing (du, dv)
                                const Vec2 b = P(u, v), dir = D * du + Lft * dv, side = perp(dir) * 0.45, tip = b + dir * len;
                                MeshBuilder::emitQuad(paintW, world(b - side, Z(b - side)), world(tip, Z(tip)), world(tip, Z(tip)), world(b + side, Z(b + side)), Vec3(0, 1, 0), kPaintWhite);
                            };
                            const double turnAt = mS ? 1.6 : 2.4;   // a turn branches low when a straight arrow carries on above it
                            shaft({{0.0, 0.0}, {mS ? 3.8 : turnAt, 0.0}});
                            if (mS) head(3.8, 0.0, 1.0, 0.0, 1.2);
                            for (int t2 : {+1, -1}) {
                                if ((t2 > 0 && !mL) || (t2 < 0 && !mR)) continue;
                                shaft({{turnAt, 0.0}, {turnAt + 0.55, 0.22 * t2}, {turnAt + 0.8, 0.5 * t2}});
                                head(turnAt + 0.8, 0.5 * t2, 0.0, static_cast<double>(t2), 0.95);
                            }
                        }
                    }
                }
            }
            // stop bar across the approaching lanes, a metre before the crosswalk — the minor road stops
            // for the major one (equal ranks: every approach stops)
            if (crossRank < ownRank) return;
            double lo = 1e300, hi = -1e300; for (const LL& x : ls) if (x.approaching) { lo = std::min(lo, x.lat - x.w / 2); hi = std::max(hi, x.lat + x.w / 2); }
            if (lo < hi) { Vec2 Q = pointAt(e.xy, e.s, sBox + sign * 5.2); Vec2 a = Q + N * (lo + 0.15), b = Q + N * (hi - 0.15);
                strip(paintW, std::vector<Vec2>{a, b}, std::vector<double>{zAt(laneAt(lo), a), zAt(laneAt(hi), b)}, 0.25, 0, 0, kPaintWhite); }
        };
        for (const Span& sp : merged) { paintSide(sp.s0, -1, sp.crossRank); paintSide(sp.s1, +1, sp.crossRank); }
    }
    // terrain
    if (r.hasTerrain) {
        const HeightGrid& G = r.terrain;
        for (int j = 0; j < G.ny; ++j) for (int i = 0; i < G.nx; ++i) {
            Vertex v; v.position = world(Vec2(G.x0 + i * G.res, G.y0 + j * G.res), G.at(i, j)); v.normal = Vec3(0, 1, 0); v.color = kGrass; v.u = static_cast<float>(i); v.v = static_cast<float>(j); terrain.vertices.push_back(v);
        }
        MeshBuilder::gridIndices(terrain, G.nx, G.ny, 0); MeshBuilder::recomputeNormals(terrain);
    }
    std::vector<NamedMesh> out;
    auto add = [&](const char* n, RenderMesh& m, const Vec3& c) { if (!m.indices.empty()) out.push_back({n, std::move(m), c}); };
    add("asphalt", asphalt, kAsphalt); add("concrete", concrete, kConcrete); add("guardrail", guardrail, kGuardrail); add("sidewalk", sidewalk, kSidewalk); add("shoulder", shoulder, kShoulder); add("median", median, kMedian);
    add("paint_white", paintW, kPaintWhite); add("paint_yellow", paintY, kPaintYellow); add("terrain", terrain, kGrass);
    return out;
}

std::vector<LaneMoves> assignLaneMoves(std::size_t n, bool hasL, bool hasS, bool hasR, bool leftPocket, bool rightPocket) {
    std::vector<LaneMoves> m(n);
    if (n == 1) { m[0] = {hasL, hasS, hasR}; return m; }
    for (std::size_t i = 0; i < n; ++i) {
        if (i == 0) { m[i].left = hasL; m[i].straight = hasS && (!leftPocket || !hasL); }                  // leftmost: left (+ straight)
        else if (i + 1 == n) { m[i].right = hasR; m[i].straight = hasS && (!rightPocket || !hasR); }      // rightmost: right (+ straight)
        else { m[i].straight = hasS; if (!hasS) { m[i].left = i * 2 < n && hasL; m[i].right = !m[i].left && hasR; } }   // middle
        // an end lane with nothing its own way still carries on or turns the other (a T's stem: two lanes, left and right)
        if (!m[i].left && !m[i].straight && !m[i].right) { m[i].straight = hasS; if (!hasS) { m[i].left = hasL && i == 0; m[i].right = hasR && i + 1 == n; } }
    }
    return m;
}

const char* edgeRoleName(EdgeRole role) {
    switch (role) {
        case EdgeRole::Seam: return "seam";
        case EdgeRole::Median: return "median";
        case EdgeRole::VsStreet: return "vs street";
        case EdgeRole::Elevated: return "elevated";
        case EdgeRole::AtGrade: return "at grade";
        default: return "?";
    }
}

// The class table decides what an edge carries; these are the defaults when the graph JSON says nothing.
// A median is walled whether the road is elevated or not — opposing traffic is on the far side, and half
// the metro median was bare because it happened to sit at grade. A city-facing edge gets a noise wall. An
// at-grade outward edge gets a guardrail, not a concrete wall through a field.
BarrierSpec barrierFor(const RoadClassSpec& c, EdgeRole role) {
    const BarrierSpec& authored = c.edges[static_cast<size_t>(role)];
    if (authored.set) return authored;   // authored, even when it says none
    if (c.name != "freeway" && c.name != "ramp") return {};   // streets are the lot pass's business
    switch (role) {   // kind, height, offset outboard of the outline, body thickness
        case EdgeRole::Median:   return {BarrierKind::Wall, 1.05, 0.0, 0.4};
        case EdgeRole::VsStreet: return {BarrierKind::Wall, 2.50, 0.0, 0.4};
        case EdgeRole::Elevated: return {BarrierKind::Wall, 0.90, 0.0, 0.4};
        case EdgeRole::AtGrade:  return {BarrierKind::Guardrail, 0.75, 0.0, 0.1};
        default:                 return {};
    }
}

std::vector<ParapetRun> parapetRuns(const Result& r, ParapetCensus* census) {
    const LaneSet& L = r.lanes;
    std::vector<ParapetRun> out;
    const std::vector<Box2> laneBox = laneBoxes(r.pavement.footprints); const LaneGrid laneGrid(laneBox, r.pavement.footprints); const std::vector<PreparedSet> footprint = prepareAll(r.pavement.footprints);
    auto isDeckLane = [&](int li) {
        if (li < 0 || li >= static_cast<int>(L.lanes.size())) return false; const Lane& l = L.lanes[static_cast<size_t>(li)];
        if (l.isConnector() || l.parent < 0) return false; const EdgeSpec& e = r.graph.edges[static_cast<size_t>(l.parent)]; return e.isRamp() || e.cls == "freeway";
    };
    auto pavedLaneAt = [&](const Vec2& q, double z, bool deckOnly) {
        for (int ojI : laneGrid.at(q)) {
            const size_t oj = static_cast<size_t>(ojI); const Box2& bb = laneBox[oj];
            if (deckOnly && !isDeckLane(ojI)) continue;
            if (q.x < bb.minX || q.x > bb.maxX || q.y < bb.minY || q.y > bb.maxY || !footprint[oj].contains(q)) continue;
            if (std::fabs(r.heights->deck(ojI, q) - z) < 1.0) return ojI;
        }
        return -1;
    };
    // which deck lane's footprint covers q, height irrelevant: used to find the road the outline belongs to
    auto deckLaneAt = [&](const Vec2& q) {
        for (int ojI : laneGrid.at(q)) {
            const size_t oj = static_cast<size_t>(ojI); const Box2& bb = laneBox[oj];
            if (!isDeckLane(ojI)) continue;
            if (q.x < bb.minX || q.x > bb.maxX || q.y < bb.minY || q.y > bb.maxY || !footprint[oj].contains(q)) continue;
            return ojI;
        }
        return -1;
    };
    auto groundAt = [&](const Vec2& p) { return r.hasTerrain ? r.terrain.sample(p.x, p.y) : 0.0; };

    // THE BARRIER LINE IS THE ROAD'S OWN OUTLINE (2026-09-08, Glenn: "it's like sidewalks but they're walls").
    // A sidewalk is a ribbon offset from the road, unioned and clipped by Clipper, which is why it never
    // fragments and never twists. A barrier was assembled instead from the deck's boundary TRIANGLE EDGES,
    // chained by vertex index: any edge that failed a test ended the chain (775 runs over 25 km), and a sharp
    // joint had to be mitred by hand. Offsetting the freeway and ramp pavement gives the same line as one
    // polygon, already mitred and closed, and its boundary rings are the ordered curves a sweep wants.
    PolySet deckPaved;
    for (size_t li = 0; li < L.lanes.size(); ++li) if (isDeckLane(static_cast<int>(li)) && !r.pavement.footprints[li].empty()) deckPaved = unionSets(deckPaved, r.pavement.footprints[li]);
    if (deckPaved.empty()) return out;
    const double standOff = 0.05;                    // the barrier hugs the pavement edge; its body lies outside it
    const PolySet band = offsetSet(deckPaved, standOff);
    std::vector<Ring> rings;
    for (const Polygon2& pg : band) { if (pg.outer.size() >= 8) rings.push_back(pg.outer); for (const Ring& h : pg.holes) if (h.size() >= 8) rings.push_back(h); }

    int walled = 0; double bandM = 0;
    for (const Ring& ring : rings) {
        std::vector<Vec2> pts = resample(ring, 1.0); if (pts.size() < 8) continue;
        const size_t n = pts.size();
        // Per point: which deck lane owns it, its height, what the edge faces, and so what to build.
        std::vector<int> lane(n, -1); std::vector<double> z(n, 0.0); std::vector<EdgeRole> role(n, EdgeRole::Seam); std::vector<BarrierSpec> spec(n);
        for (size_t i = 0; i < n; ++i) {
            const Vec2 t = normalize(pts[(i + 1) % n] - pts[(i + n - 1) % n]); const Vec2 nrm(t.y, -t.x);
            // the pavement is on one side; "beyond" is the other
            int inLane = deckLaneAt(pts[i] - nrm * 0.6), outLane = -1; double sign = -1.0;
            if (inLane < 0) { inLane = deckLaneAt(pts[i] + nrm * 0.6); sign = +1.0; }
            if (inLane < 0) continue;                                   // no deck lane either side: not our line
            lane[i] = inLane; z[i] = r.heights->deck(inLane, pts[i]);
            const Vec2 outward = nrm * -sign;
            // what lies beyond, walking out
            double nd = 1e300; int nb = -1;
            for (const double d : {1.0, 2.5, 5.0, 8.0, 12.0, 18.0, 25.0, 32.0}) {
                const int hit = pavedLaneAt(pts[i] + outward * d, z[i], false);
                if (hit >= 0 && hit != inLane) { nb = hit; nd = d; break; }
            }
            outLane = nb;
            const EdgeSpec& me = r.graph.edges[static_cast<size_t>(L.lanes[static_cast<size_t>(inLane)].parent)];
            const EdgeSpec* ne = outLane >= 0 && L.lanes[static_cast<size_t>(outLane)].parent >= 0 ? &r.graph.edges[static_cast<size_t>(L.lanes[static_cast<size_t>(outLane)].parent)] : nullptr;
            const double hAbove = z[i] - groundAt(pts[i]);
            // A MOUTH IS NOT AN EDGE. The outline is the boundary of the freeway and ramp pavement alone, so it
            // wraps around the END of a ramp where it meets a street — and a barrier laid there seals the exit
            // ("no way to get off the exit ramp!", Glenn, 2026-09-08). Wherever the outline stands on, or right
            // beside, pavement that is not ours, the road carries on and nothing is built.
            const bool onOtherPavement = [&] {
                for (const Vec2& q : {pts[i], pts[i] + nrm * 0.6, pts[i] - nrm * 0.6}) {
                    const int hit = pavedLaneAt(q, z[i], false);
                    if (hit >= 0 && !isDeckLane(hit)) return true;
                }
                return false;
            }();
            if (onOtherPavement) role[i] = EdgeRole::Seam;
            else if (ne && nd <= 1.2) role[i] = EdgeRole::Seam;          // pavement right there: the outline runs through a junction
            else if (ne && ne->cls == "freeway" && me.cls == "freeway" && ne != &me && nd <= 14.0) role[i] = EdgeRole::Median;
            else if (ne && ne->cls != "freeway" && !ne->isRamp() && nd <= 32.0) role[i] = EdgeRole::VsStreet;
            else role[i] = hAbove >= 1.5 ? EdgeRole::Elevated : EdgeRole::AtGrade;
            spec[i] = barrierFor(r.graph.cls(me), role[i]);
        }
        // census over the ring
        for (size_t i = 0; i < n; ++i) {
            const double seg = distance(pts[i], pts[(i + 1) % n]); bandM += seg;
            if (lane[i] < 0) continue;
            if (census) { census->metres[static_cast<size_t>(role[i])] += seg; if (spec[i].kind != BarrierKind::None) census->built[static_cast<size_t>(role[i])] += seg; }
        }
        // split the ring into arcs of one spec and emit a run for each
        auto same = [&](size_t i, size_t j) {
            return lane[i] >= 0 && lane[j] >= 0 && spec[i].kind == spec[j].kind && std::fabs(spec[i].h - spec[j].h) < 1e-6
                   && std::fabs(spec[i].offset - spec[j].offset) < 1e-6 && spec[i].kind != BarrierKind::None;
        };
        size_t start = 0; bool allSame = true;
        for (size_t i = 1; i < n; ++i) if (!same(0, i)) { allSame = false; start = i; break; }
        if (allSame && spec[0].kind != BarrierKind::None && lane[0] >= 0) {   // a whole closed ring of one barrier
            ParapetRun run; run.closed = true; run.lane = lane[0]; run.role = role[0]; run.kind = spec[0].kind; run.height = spec[0].h;
            for (size_t i = 0; i < n; ++i) { run.pts.push_back(pts[i]); run.z.push_back(z[i]); run.offOuter.push_back(spec[i].offset + spec[i].thick); run.offInner.push_back(spec[i].offset); }
            out.push_back(std::move(run)); ++walled; continue;
        }
        // Only a TERMINAL tapers: an arc that stops because the road wants no barrier there (a junction mouth,
        // a seam) ends beside drivable ground and needs a ramp down. An arc that stops because the next
        // stretch is a different barrier is continuous and stays square, or every transition grows a wedge.
        auto wantsNothing = [&](size_t k) { return lane[k] < 0 || spec[k].kind == BarrierKind::None; };
        size_t i = start; ParapetRun run; bool open = false;
        for (size_t k = 0; k <= n; ++k, i = (i + 1) % n) {
            const bool build = k < n && lane[i] >= 0 && spec[i].kind != BarrierKind::None;
            const bool cont = open && build && same(i, (i + n - 1) % n);
            if (open && !cont) {
                if (run.pts.size() >= 2) { run.capOnPavement[1] = wantsNothing(i); out.push_back(std::move(run)); ++walled; }
                run = ParapetRun(); open = false;
            }
            if (!build || k == n) continue;
            if (!open) {
                run = ParapetRun(); run.closed = false; run.lane = lane[i]; run.role = role[i]; run.kind = spec[i].kind; run.height = spec[i].h;
                run.capOnPavement[0] = wantsNothing((i + n - 1) % n); open = true;
            }
            run.pts.push_back(pts[i]); run.z.push_back(z[i]); run.offOuter.push_back(spec[i].offset + spec[i].thick); run.offInner.push_back(spec[i].offset);
        }
        if (open && run.pts.size() >= 2) { run.capOnPavement[1] = wantsNothing(i); out.push_back(std::move(run)); ++walled; }
    }
    std::fprintf(stderr, "lanelab: barrier line %.0f m of road outline, %d runs swept\n", bandM, walled);
    return out;
}

}  // namespace roads::lanes
}  // namespace engine
