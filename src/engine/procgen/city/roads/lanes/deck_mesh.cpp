#include "engine/procgen/city/roads/lanes/deck_mesh.h"
#include "engine/mesh_builder.h"
#include "log.h"
#include "engine/procgen/city/roads/lanes/polyline_ops.h"
#include "engine/procgen/city/roads/lanes/geom2d.h"
#include "engine/procgen/city/rail_track.h"

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
           kBallast(0.30f, 0.28f, 0.25f), kSleeper(0.55f, 0.53f, 0.49f), kRailSteel(0.42f, 0.40f, 0.38f), kPlatform(0.70f, 0.69f, 0.66f),
           kCanopySheet(0.60f, 0.62f, 0.64f), kStationSteel(0.20f, 0.27f, 0.29f), kStationLamp(0.92f, 0.94f, 0.95f),
           kMedian(0.22f, 0.34f, 0.14f), kPaintWhite(0.9f, 0.9f, 0.85f), kPaintYellow(0.95f, 0.75f, 0.15f), kGrass(0.20f, 0.30f, 0.13f);

// WELDED, AND NO UNDERSIDE WHERE NOBODY CAN SEE ONE (ADR-0095). Every deck triangle used to be
// emitted with three vertices of its own, top AND bottom face: metro_planned's asphalt alone was
// 779 MB of GPU mesh. A top vertex is now shared by every triangle that meets it with the same
// UV (one lane's triangles; two lanes' UVs differ and keep their own), and the underside is
// emitted only where it stands clear of the ground (`groundAt`, 0.3 m) -- a bridge, a viaduct,
// a ramp; a street lies on the ground. Null groundAt: every underside, as before.
// the layers' slab thicknesses (sidewalk, shoulder, median): what they are built with, and how far down each one
// covers a neighbour's side (slab's side rule)
constexpr double kSidewalkThick = 0.42, kShoulderThick = 0.3, kMedianThick = 0.40;

void slab(RenderMesh& top, RenderMesh& side, const std::vector<DeckVertex>& verts, const std::vector<std::array<int, 3>>& tris,
          const std::vector<std::pair<int, int>>& boundary, double thick, double lift, const Vec3& color, const Vec3& sideColor,
          const std::vector<std::array<float, 6>>* uv, const std::function<bool(const Vec2&, const Vec2&, double)>* seam = nullptr,
          const std::function<double(const Vec2&)>* groundAt = nullptr,
          const std::function<double(const Vec2&, double)>* hidden = nullptr) {
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
        MeshBuilder::triFacing(top, a, b, c, normal);
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
            // THE UNDERSIDE IS THE SLAB'S CONCRETE (#97, Glenn: "the elevated roads have no bottom to them"):
            // emitted into the top mesh it took the road surface's material, whose procedural asphalt sets its
            // own near-black colour, and a dark face pointing down where no sun reaches read as a void. It
            // goes into the SIDE mesh (concrete, as the girders and slab edges are), wound to face down.
            const Vec3 Q[3] = {world(a.xy, a.z + lift - thick), world(b.xy, b.z + lift - thick), world(c.xy, c.z + lift - thick)};
            const uint32_t base = static_cast<uint32_t>(side.vertices.size());
            const Vec3 down(0, -1, 0), tan = normalize(Q[1] - Q[0]).lengthSquared() > 0 ? normalize(Q[1] - Q[0]) : Vec3(1, 0, 0);
            for (const Vec3& q : Q) { Vertex vx(q, down, tan, 0.0f, 0.0f); vx.color = sideColor; side.vertices.push_back(vx); }
            MeshBuilder::triFacing(side, base, base + 1, base + 2, down);
        }
    }
    // THE SIDE ONLY WHERE IT SHOWS (ADR-0095). A side is covered where the surface beyond the edge
    // stands at or above its top (`hidden`: asphalt under a sidewalk, a shoulder against the lane it
    // continues) -- but only DOWN TO THAT SURFACE'S OWN UNDERSIDE, which `hidden` returns: a 1.2 m
    // carriageway slab beside a 0.3 m shoulder hangs 0.9 m below it, and that step is a face (Glenn,
    // 2026-10-14: "a strip of polygons missing along the side ... the hole is clearly one way": from
    // below you looked into the slab, through the road's culled back, at the sky). And it is dropped where it does not rise above the ground outside it (`groundAt`); one that shows
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
        double botA = topA - thick, botB = topB - thick;
        double coverTopA = topA, coverTopB = topB;   // where this side's exposed part ends, up
        if (hidden) {
            const double under = (*hidden)((a.xy + b.xy) * 0.5 + out * 0.15, 0.5 * (topA + topB));
            if (!std::isnan(under)) {   // a neighbour stands at the top: only the step below its underside shows
                const double drop = 0.5 * (topA + topB) - under;   // how deep the neighbour reaches
                if (drop >= thick - 0.02) continue;
                coverTopA = topA - drop; coverTopB = topB - drop;
            }
        }
        if (groundAt) {
            const double gA = (*groundAt)(a.xy + out * 0.3), gB = (*groundAt)(b.xy + out * 0.3);
            if (coverTopA - gA < 0.03 && coverTopB - gB < 0.03) continue;
            botA = std::max(botA, std::min(coverTopA, gA) - 0.05);
            botB = std::max(botB, std::min(coverTopB, gB) - 0.05);
        }
        sides.push_back({e.first, e.second, coverTopA, coverTopB, botA, botB});
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
// inner face, top, caps and bottom (it may stand on the seam between two slabs). Every wall in the lab should come through here
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
        // and its BOTTOM: a median wall stands on the seam between two carriageways' slabs, and from under the
        // viaduct the slit between their soffits looked up through the wall to the sky (rt_bake --audit, undersides;
        // Glenn: "the elevated freeway undercarriage when viewed from below can be seen through"). On a slab the
        // bottom faces the slab's top and is culled.
        MeshBuilder::emitQuad(mesh, P(i, offOuter[i], 0), P(j, offOuter[j], 0), P(j, offInner[j], 0), P(i, offInner[i], 0), Vec3(0, -1, 0), color);
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

bool trackClass(const RoadLabGraph& g, const std::string& cls) {
    const auto it = g.classes.find(cls);
    return it != g.classes.end() && it->second.track;
}

bool trackLane(const Result& r, int li) {
    if (li < 0 || li >= static_cast<int>(r.lanes.lanes.size())) return false;
    const Lane& l = r.lanes.lanes[static_cast<size_t>(li)];
    return !l.isConnector() && l.parent >= 0 && trackClass(r.graph, r.graph.edges[static_cast<size_t>(l.parent)].cls);
}

// A lane's left-normal offset from its parent spine (laneSlots' convention), measured where it starts.
double laneOffset(const Result& r, const Lane& l) {
    const EdgeSpec& e = r.graph.edges[static_cast<size_t>(l.parent)];
    if (e.xy.size() < 2 || l.xy.empty()) return 0;
    return dot(l.xy.front() - e.xy.front(), perp(tangentAtStation(e.xy, e.s, 0.0)));
}

// One closed box of a track kit, standing on the deck: centre `c`, `t` along the track, `hx` half its length
// ACROSS the track and `ht` half along it, from y0 to y1. No bottom face: it lies on the formation.
// A box on the track's frame, closed: a stair's block, a column, a railing -- each may stand over air (rt_bake --audit,
// undersides). `bottom` false only where it lies on something (a sleeper on its ballast).
void trackBox(RenderMesh& m, const Vec2& c, const Vec2& t, double hx, double ht, double y0, double y1, const Vec3& color, bool bottom = true) {
    // the mesh library's box on the track's frame: across (n), up, along (t)
    const Vec2 n = perp(t), corner = c - n * hx - t * ht;
    MeshFrame f;
    f.o = world(corner, y0);
    f.x = Vec3(static_cast<Real>(n.x * 2 * hx), 0, static_cast<Real>(n.y * 2 * hx));
    f.y = Vec3(0, static_cast<Real>(y1 - y0), 0);
    f.z = Vec3(static_cast<Real>(t.x * 2 * ht), 0, static_cast<Real>(t.y * 2 * ht));
    MeshBuilder::emitBox(m, f, color, bottom ? kFacesAll : static_cast<uint8_t>(kFacesAll & ~kFaceBottom));
}

// THE TRACK on one lane: a sleeper every 0.65 m and two rails swept along the lane's centreline at gauge.
void trackKit(RenderMesh& sleepers, RenderMesh& rails, const Result& r, int li) {
    const Lane& l = r.lanes.lanes[static_cast<size_t>(li)]; const DeckHeight& H = *r.heights;
    if (l.s.size() < 2 || l.s.back() < 1.0) return;
    const double len = l.s.back();
    for (double st = 0.3; st < len; st += 0.65) {
        const Vec2 p = pointAt(l.xy, l.s, st), t = tangentAtStation(l.xy, l.s, st); const double z = H.deck(li, p);
        trackBox(sleepers, p, t, 1.3, 0.13, z - 0.04, z + kSleeperTop, kSleeper, false);
    }
    const int n = std::max(2, static_cast<int>(std::ceil(len / 2.0)) + 1);
    const double half = kRailGauge / 2 + 0.035, hw = 0.035;
    for (int side = -1; side <= 1; side += 2) {
        std::vector<Vec2> c(static_cast<size_t>(n)), nr(static_cast<size_t>(n)); std::vector<double> z(static_cast<size_t>(n));
        for (int k = 0; k < n; ++k) {
            const double st = len * k / (n - 1); const Vec2 p = pointAt(l.xy, l.s, st);
            nr[static_cast<size_t>(k)] = perp(tangentAtStation(l.xy, l.s, st)); c[static_cast<size_t>(k)] = p + nr[static_cast<size_t>(k)] * (half * side);
            z[static_cast<size_t>(k)] = H.deck(li, p);
        }
        for (int k = 0; k + 1 < n; ++k) {
            const size_t i = static_cast<size_t>(k), j = i + 1;
            const Vec2 ai = c[i] - nr[i] * hw, bi = c[i] + nr[i] * hw, aj = c[j] - nr[j] * hw, bj = c[j] + nr[j] * hw;
            const double ti = z[i] + kRailHeadAbove, tj = z[j] + kRailHeadAbove, fi = z[i] + kSleeperTop, fj = z[j] + kSleeperTop;
            MeshBuilder::emitQuad(rails, world(ai, ti), world(aj, tj), world(bj, tj), world(bi, ti), Vec3(0, 1, 0), kRailSteel);
            const Vec3 nn(static_cast<Real>(nr[i].x), 0, static_cast<Real>(nr[i].y));
            MeshBuilder::emitQuad(rails, world(bi, fi), world(bj, fj), world(bj, tj), world(bi, ti), nn, kRailSteel);
            MeshBuilder::emitQuad(rails, world(ai, fi), world(aj, fj), world(aj, tj), world(ai, ti), nn * Real(-1), kRailSteel);
        }
    }
}

// The outermost track lane of edge `ei` on spine side `side` (+1 left, -1 right), or -1.
int outerTrack(const Result& r, int ei, int side, double* offOut) {
    int best = -1; double bestOff = 0;
    for (size_t li = 0; li < r.lanes.lanes.size(); ++li) {
        const Lane& l = r.lanes.lanes[li];
        if (l.parent != ei || l.isConnector()) continue;
        const double off = laneOffset(r, l);
        if (off * side <= 0 && !(off == 0 && side > 0)) continue;   // a centred single track serves both sides
        if (best < 0 || off * side > bestOff * side) { best = static_cast<int>(li); bestOff = off; }
    }
    if (offOut) *offOut = bestOff;
    return best;
}

bool footwayClass(const RoadLabGraph& g, const std::string& cls) {
    const auto it = g.classes.find(cls);
    return it != g.classes.end() && it->second.footway;
}

bool footwayLane(const Result& r, int li) {
    if (li < 0 || li >= static_cast<int>(r.lanes.lanes.size())) return false;
    const Lane& l = r.lanes.lanes[static_cast<size_t>(li)];
    return !l.isConnector() && l.parent >= 0 && footwayClass(r.graph, r.graph.edges[static_cast<size_t>(l.parent)].cls);
}

// A STATION STAIR (Glenn, 2026-10-11: "make it accessible"): where a footway crosses a track over its platforms
// (a footbridge), a flight from the footway's deck down onto each platform -- along the platform, against its back
// edge, the two flights running opposite ways. The parapet opens where a flight leaves the deck, and the canopy
// stops over the flight and under the bridge.
// UP, where the footway passes UNDER the track (a city's light rail, held up over its streets): a flight from the
// footway's edge up onto each platform, behind its back edge, landing level with it at the top. `top` is then the
// landing beside the platform and `dir` runs down toward the footway, as on a footbridge's flight.
struct StationStair {
    int edge = -1; const PlatformSpec* platform = nullptr; int side = 0;
    Vec2 top, dir, across;        // the top of the flight on the deck's edge (its middle); down the flight; across it
    double zTop = 0, sTop = 0;    // the deck there; the top's station along the track edge
    double width = 1.9;           // walls included
    int steps = 0; double riser = 0, tread = 0.28;
    double bridge0 = 0, bridge1 = 0;   // the bridge over this platform, as track-edge stations
    bool up = false; double zFoot = 0; // a flight up from a footway under the track; the footway's height at its foot
    double length() const { return (steps - 1) * tread; }
};

std::vector<StationStair> stationStairs(const Result& r) {
    std::vector<StationStair> out;
    const RoadLabGraph& g = r.graph;
    for (size_t fi = 0; fi < g.edges.size(); ++fi) {
        const EdgeSpec& f = g.edges[fi];
        if (!footwayClass(g, f.cls) || f.xy.size() < 2) continue;
        int fl = -1;
        for (size_t li = 0; li < r.lanes.lanes.size(); ++li)
            if (r.lanes.lanes[li].parent == static_cast<int>(fi) && !r.lanes.lanes[li].isConnector()) { fl = static_cast<int>(li); break; }
        if (fl < 0) continue;
        const double hwF = g.hw(f);
        for (size_t ei = 0; ei < g.edges.size(); ++ei) {
            const EdgeSpec& e = g.edges[ei];
            if (!trackClass(g, e.cls) || e.platforms.empty() || e.xy.size() < 2) continue;
            std::vector<Vec2> xs;   // (a spine vertex on the other line is found by both its segments: once each)
            for (const Vec2& X : crossings(e.xy, f.xy)) { bool dup = false; for (const Vec2& q : xs) if (distance(q, X) < 1.0) dup = true; if (!dup) xs.push_back(X); }
            for (const Vec2& X : xs) {
                const double sX = project(e.xy, e.s, X).station;
                const Vec2 te = tangentAtStation(e.xy, e.s, sX), ne = perp(te);
                const Vec2 tf = tangentAtStation(f.xy, f.s, project(f.xy, f.s, X).station);
                const double sinT = std::fabs(cross(te, tf));
                if (sinT < 0.5) continue;   // a flight beside a badly skewed bridge would not meet its edge
                const double cotT = dot(tf, te) / dot(tf, ne), halfDeck = hwF / sinT;
                for (const PlatformSpec& P : e.platforms) {
                    if (sX < P.s0 + 4 || sX > P.s1 - 4) continue;
                    for (int side = -1; side <= 1; side += 2) {
                        if (P.side != 0 && P.side != side) continue;
                        double off = 0; const int li = outerTrack(r, static_cast<int>(ei), side, &off); if (li < 0) continue;
                        StationStair st; st.edge = static_cast<int>(ei); st.platform = &P; st.side = side;
                        const double platTop = r.heights->deck(li, pointAt(e.xy, e.s, sX) + ne * off) + kRailHeadAbove + P.h;
                        st.up = r.heights->deck(fl, X) < platTop - 0.4;   // the footway under the track, not over it
                        const double lateral = off + side * (kPlatformGap + P.w) + side * (st.up ? 0.02 + st.width / 2 : -(0.15 + st.width / 2));
                        const double alongC = lateral * cotT;
                        st.bridge0 = sX + alongC - halfDeck - 0.6; st.bridge1 = sX + alongC + halfDeck + 0.6;
                        const Vec2 onWay = pointAt(e.xy, e.s, sX + alongC) + ne * lateral;
                        if (st.up) {
                            st.zFoot = r.heights->deck(fl, onWay); st.zTop = platTop;
                            const double rise = platTop - st.zFoot; if (rise < 0.4) continue;
                            st.steps = static_cast<int>(std::ceil(rise / 0.175)); st.riser = rise / st.steps;
                            // the flight climbs away from the footway, its landing (one tread) on the platform's level
                            int d = side;
                            auto fits = [&](int dd) { const double s1 = sX + alongC + dd * (halfDeck + st.length() + st.tread); return s1 > P.s0 + 2 && s1 < P.s1 - 2; };
                            if (!fits(d)) d = -d;
                            if (!fits(d)) continue;
                            st.sTop = sX + alongC + d * (halfDeck + st.length());
                            st.dir = te * static_cast<double>(-d); st.across = ne * static_cast<double>(side);
                            st.top = pointAt(e.xy, e.s, st.sTop) + ne * lateral;
                            out.push_back(st);
                            continue;
                        }
                        st.zTop = r.heights->deck(fl, onWay);
                        const double rise = st.zTop - platTop; if (rise < 0.4) continue;
                        st.steps = static_cast<int>(std::ceil(rise / 0.175)); st.riser = rise / st.steps;
                        int d = side;   // the two flights run opposite ways; either turns round if its end would leave the platform
                        auto fits = [&](int dd) { const double s1 = sX + alongC + dd * (halfDeck + st.length()); return s1 > P.s0 + 2 && s1 < P.s1 - 2; };
                        if (!fits(d)) d = -d;
                        if (!fits(d)) continue;
                        st.sTop = sX + alongC + d * halfDeck;
                        st.dir = te * static_cast<double>(d); st.across = ne * static_cast<double>(side);
                        st.top = pointAt(e.xy, e.s, st.sTop) + ne * lateral;
                        out.push_back(st);
                    }
                }
            }
        }
    }
    return out;
}

// FOOTWAY STAIRS: wherever a footway lane climbs steeper than kFootwayStairGrade, steps on its deck -- a riser each
// ~0.17 m of height, each tread a solid block standing on the slope (its top the slope's height at the tread's upper
// end), the lane's width less a hand's breadth each side. The gentle stretches stay the paved ramp under them.
void footwayStairs(RenderMesh& steps, const Result& r, int li) {
    const Lane& l = r.lanes.lanes[static_cast<size_t>(li)];
    if (l.s.size() < 2 || l.s.back() < 2.0) return;
    const double len = l.s.back(), ds = 0.05, hx = std::max(0.5, l.w / 2 - 0.1);
    auto zAt = [&](double st) { return r.heights->deck(li, pointAt(l.xy, l.s, std::clamp(st, 0.0, len))); };
    // the steep stretches, on a 1 m grade read
    std::vector<std::pair<double, double>> runs;
    double runFrom = -1;
    for (double st = 0; st <= len; st += 0.5) {
        const bool steep = std::fabs(zAt(st + 0.5) - zAt(st - 0.5)) > kFootwayStairGrade * 1.0;
        if (steep && runFrom < 0) runFrom = st;
        if (!steep && runFrom >= 0) { if (st - runFrom >= 1.0) runs.push_back({runFrom, st}); runFrom = -1; }
    }
    if (runFrom >= 0 && len - runFrom >= 1.0) runs.push_back({runFrom, len});
    for (const auto& [a, b] : runs) {
        const double z0 = zAt(a), z1 = zAt(b);
        const int n = std::max(1, static_cast<int>(std::lround(std::fabs(z1 - z0) / 0.17)));
        const double riser = (z1 - z0) / n;
        // the step edges: where the deck reaches each riser's height
        std::vector<double> edge{a};
        double st = a;
        for (int k = 1; k < n; ++k) {
            const double target = z0 + riser * k;
            while (st < b && (zAt(st) - target) * (riser > 0 ? 1 : -1) < 0) st += ds;
            edge.push_back(st);
        }
        edge.push_back(b);
        for (std::size_t k = 0; k + 1 < edge.size(); ++k) {
            const double s0 = edge[k], s1 = edge[k + 1]; if (s1 - s0 < 0.05) continue;
            const double top = std::max(zAt(s0), zAt(s1)), foot = std::min(zAt(s0), zAt(s1)) - 0.05;
            const Vec2 c = pointAt(l.xy, l.s, (s0 + s1) / 2), t = tangentAtStation(l.xy, l.s, (s0 + s1) / 2);
            trackBox(steps, c, t, hx, (s1 - s0) / 2, foot, top, kSidewalk);
        }
    }
}

// One flight: a solid block per step from the platform up, a wall each side a metre over the treads. Up from a
// footway: the blocks stand on the footway's level, the landing is a tread too, and the wall on the platform's side
// stops at the platform's level (stepped over onto it at the top).
void stairKit(RenderMesh& treads, RenderMesh& concrete, const Result& r, const StationStair& st) {
    const EdgeSpec& e = r.graph.edges[static_cast<size_t>(st.edge)];
    double off = 0; const int li = outerTrack(r, st.edge, st.side, &off); if (li < 0) return;
    if (st.up) {
        for (int i = 0; i < st.steps; ++i) {
            const Vec2 c = st.top + st.dir * (i * st.tread);
            const double y = st.zTop - i * st.riser, floor = st.zFoot - 0.3;
            trackBox(treads, c, st.dir, st.width / 2 - 0.15, st.tread / 2, floor, y, kPlatform);
            for (int w = -1; w <= 1; w += 2)
                trackBox(concrete, c + st.across * (w * (st.width / 2 - 0.075)), st.dir, 0.075, st.tread / 2, floor, w < 0 ? std::min(y + 1.0, st.zTop) : y + 1.0, kConcrete);
        }
        // the landing's far end walled (walked straight on, the flight's top led off it into the air)
        trackBox(concrete, st.top - st.dir * (st.tread / 2 + 0.075), st.dir, st.width / 2, 0.075, st.zFoot - 0.3, st.zTop + 1.0, kConcrete);
        return;
    }
    for (int i = 1; i < st.steps; ++i) {
        const double a = (i - 0.5) * st.tread; const Vec2 c = st.top + st.dir * a;
        const double sAt = project(e.xy, e.s, c).station; const Vec2 q = pointAt(e.xy, e.s, sAt), nr = perp(tangentAtStation(e.xy, e.s, sAt));
        const double floor = r.heights->deck(li, q + nr * off) + kRailHeadAbove + st.platform->h - 0.05, y = st.zTop - i * st.riser;
        trackBox(treads, c, st.dir, st.width / 2 - 0.15, st.tread / 2, floor, y, kPlatform);
        for (int w = -1; w <= 1; w += 2) trackBox(concrete, c + st.across * (w * (st.width / 2 - 0.075)), st.dir, 0.075, st.tread / 2, floor, y + 1.0, kConcrete);
    }
}

// Where a platform's canopy lamps hang: every 6 m down its middle, the first 3 m in. A stop's night lights stand
// under every other one (railTracks: RailStop::lamps).
std::vector<double> canopyLampStations(double s0, double s1) {
    std::vector<double> out;
    for (double st = s0 + 3.0; st <= s1 - 2.0; st += 6.0) out.push_back(st);
    return out;
}

// A STATION PLATFORM: a slab from the ground up to `h` over the railhead, its edge kPlatformGap off the outer
// track, `w` wide, with a canopy down its middle (posts every 12 m, a roof 3.4 m over the platform) -- stopped
// under a footbridge and over its stair, which `stairs` holds.
// HELD UP (a light rail's, over its streets): an underside, a railing along its back edge -- open beside the top of a
// flight up to it -- and columns under its middle every 12 m, clear of the footway under it and any street.
void platformKit(RenderMesh& plat, RenderMesh& concrete, RenderMesh& canopy, RenderMesh& steel, RenderMesh& lamp, const Result& r, const EdgeSpec& e, int ei, const PlatformSpec& P,
                 const std::vector<StationStair>& stairs, const std::function<double(const Vec2&)>& groundUnder,
                 const std::function<bool(const Vec2&)>& pavedAt) {
    for (int side = -1; side <= 1; side += 2) {
        if (P.side != 0 && P.side != side) continue;
        double off = 0; const int li = outerTrack(r, ei, side, &off); if (li < 0) continue;
        const double inner = off + side * kPlatformGap, outer = inner + side * P.w;
        std::vector<std::pair<double, double>> gaps;   // track-edge stations the canopy keeps out of
        std::vector<std::pair<double, double>> railGaps, colGaps;   // held up: the railing's openings; no columns
        for (const StationStair& st : stairs) {
            if (st.edge != ei || st.platform != &P || st.side != side) continue;
            const double sEnd = st.sTop + dot(st.dir, tangentAtStation(e.xy, e.s, st.sTop)) * st.length();
            if (st.up) {
                // open where the treads are within a metre of the platform (their wall is no higher than it), and the landing
                const double ds = dot(st.dir, tangentAtStation(e.xy, e.s, st.sTop)), k = std::ceil(1.0 / st.riser) + 0.5;
                const double a = st.sTop - ds * 0.5 * st.tread, b = st.sTop + ds * k * st.tread;
                railGaps.push_back({std::min(a, b), std::max(a, b)});
                colGaps.push_back({st.bridge0 - 1.0, st.bridge1 + 1.0});
                colGaps.push_back({std::min(st.sTop, sEnd) - 1.0, std::max(st.sTop, sEnd) + 1.0});
                stairKit(plat, concrete, r, st);
                continue;
            }
            gaps.push_back({st.bridge0, st.bridge1}); gaps.push_back({std::min(st.sTop, sEnd) - 0.4, std::max(st.sTop, sEnd) + 0.4});
            stairKit(plat, concrete, r, st);
        }
        auto inGap = [&](double a, double b) { for (const auto& gp : gaps) if (b > gp.first && a < gp.second) return true; return false; };
        const double s0 = std::max(0.0, P.s0), s1 = std::min(e.s.back(), P.s1); if (s1 - s0 < 2) continue;
        const int n = std::max(2, static_cast<int>(std::ceil((s1 - s0) / 2.0)) + 1);
        std::vector<Vec2> a(static_cast<size_t>(n)), b(static_cast<size_t>(n)); std::vector<double> top(static_cast<size_t>(n)), foot(static_cast<size_t>(n));
        const Lane& l = r.lanes.lanes[static_cast<size_t>(li)];
        for (int k = 0; k < n; ++k) {
            const double st = s0 + (s1 - s0) * k / (n - 1); const Vec2 q = pointAt(e.xy, e.s, st), nr = perp(tangentAtStation(e.xy, e.s, st));
            a[static_cast<size_t>(k)] = q + nr * inner; b[static_cast<size_t>(k)] = q + nr * outer;
            const double rail = r.heights->deck(li, q + nr * off) + kRailHeadAbove;
            top[static_cast<size_t>(k)] = rail + P.h; foot[static_cast<size_t>(k)] = rail - kRailHeadAbove - 0.3;
        }
        (void)l;
        // HELD UP (over a street: an underside, a railing, columns) where its foot stands 2 m clear of the ground;
        // anywhere else it is a slab from the GROUND up, its sides taken down to the ground -- it was a box from
        // under the track ballast, so a platform on a low embankment floated over an open bottom (rt_bake --audit).
        bool heldUp = false;
        for (int k = 0; k < n; ++k) if (foot[static_cast<size_t>(k)] - groundUnder(b[static_cast<size_t>(k)]) > 2.0) heldUp = true;
        if (!heldUp)
            for (int k = 0; k < n; ++k) {
                const size_t i = static_cast<size_t>(k);
                foot[i] = std::min(foot[i], std::min(groundUnder(a[i]), groundUnder(b[i])) - 0.3);
            }
        for (int k = 0; k + 1 < n; ++k) {
            const size_t i = static_cast<size_t>(k), j = i + 1;
            MeshBuilder::emitQuad(plat, world(a[i], top[i]), world(a[j], top[j]), world(b[j], top[j]), world(b[i], top[i]), Vec3(0, 1, 0), kPlatform);
            Vec2 d = a[j] - a[i]; const double dl = d.length(); if (dl < 1e-9) continue; d = d / dl;
            const Vec2 toTrack = perp(d) * (side > 0 ? -1.0 : 1.0);   // the platform edge faces its track
            const Vec3 fn(static_cast<Real>(toTrack.x), 0, static_cast<Real>(toTrack.y));
            MeshBuilder::emitQuad(concrete, world(a[i], foot[i]), world(a[j], foot[j]), world(a[j], top[j]), world(a[i], top[i]), fn, kConcrete);
            MeshBuilder::emitQuad(concrete, world(b[i], foot[i]), world(b[j], foot[j]), world(b[j], top[j]), world(b[i], top[i]), fn * Real(-1), kConcrete);
        }
        for (int end = 0; end < 2; ++end) {
            const size_t i = end == 0 ? 0 : static_cast<size_t>(n - 1);
            const Vec2 d = end == 0 ? a[1] - a[0] : a[i] - a[i - 1]; const double dl = d.length(); if (dl < 1e-9) continue;
            const Vec3 en(static_cast<Real>(d.x / dl * (end == 0 ? -1 : 1)), 0, static_cast<Real>(d.y / dl * (end == 0 ? -1 : 1)));
            MeshBuilder::emitQuad(concrete, world(a[i], foot[i]), world(b[i], foot[i]), world(b[i], top[i]), world(a[i], top[i]), en, kConcrete);
        }
        if (heldUp) {
            auto inSpans = [](const std::vector<std::pair<double, double>>& sp, double a0, double b0) { for (const auto& g0 : sp) if (b0 > g0.first && a0 < g0.second) return true; return false; };
            for (int k = 0; k + 1 < n; ++k) {
                const size_t i = static_cast<size_t>(k), j = i + 1;
                MeshBuilder::emitQuad(concrete, world(a[i], foot[i]), world(b[i], foot[i]), world(b[j], foot[j]), world(a[j], foot[j]), Vec3(0, -1, 0), kConcrete);
                const double st0 = s0 + (s1 - s0) * k / (n - 1), st1 = s0 + (s1 - s0) * (k + 1) / (n - 1);
                if (inSpans(railGaps, st0, st1)) continue;
                const Vec2 mid2 = (b[i] + b[j]) * 0.5, d = b[j] - b[i]; const double dl = d.length(); if (dl < 1e-9) continue;
                const Vec2 inward = (a[i] - b[i]) * (1.0 / std::max(1e-9, (a[i] - b[i]).length()));
                trackBox(concrete, mid2 + inward * 0.08, d / dl, 0.08, dl / 2, 0.5 * (top[i] + top[j]) - 0.05, 0.5 * (top[i] + top[j]) + 1.1, kConcrete);
            }
            const double mid = (inner + outer) / 2;
            for (double st = s0 + 6; st <= s1 - 6; st += 12) {
                if (inSpans(colGaps, st - 0.5, st + 0.5)) continue;
                const Vec2 q = pointAt(e.xy, e.s, st), t = tangentAtStation(e.xy, e.s, st), c = q + perp(t) * mid;
                bool paved = false;
                for (double dx : {-0.6, 0.6}) for (double dy : {-0.6, 0.6}) if (pavedAt(c + Vec2(dx, dy))) paved = true;
                if (paved) continue;
                const double y = r.heights->deck(li, q + perp(t) * off) - 0.3;
                trackBox(concrete, c, t, 0.3, 0.3, groundUnder(c) - 0.5, y, kConcrete);
            }
        }
        // THE CANOPY (Glenn, 2026-10-12: "the overhangs are thin and need dimension. Maybe they should be the
        // corrugated metal?"): a ribbed steel sheet falling 8 % away from the track, on a steel frame -- a post down
        // the middle every 12 m carrying a beam across, a deep fascia along both edges -- and a lamp every 6 m under it.
        const double mid = (inner + outer) / 2, roofHalf = std::max(1.0, P.w / 2 - 0.4);
        constexpr double kRoof = 3.3, kFall = 0.08, kBeam = 0.3, kRib = 0.2, kRibH = 0.04;
        auto platTop = [&](double st) { const Vec2 q = pointAt(e.xy, e.s, st); return r.heights->deck(li, q + perp(tangentAtStation(e.xy, e.s, st)) * off) + kRailHeadAbove + P.h; };
        // a point under the roof: station st, `u` across from the middle (- toward the track), height over the beams' foot
        auto roofPt = [&](double st, double u, double dy) {
            const Vec2 q = pointAt(e.xy, e.s, st), nr = perp(tangentAtStation(e.xy, e.s, st));
            return world(q + nr * (mid + side * u), platTop(st) + kRoof - kFall * u + dy);
        };
        // a box from four corners at its foot and four at its head: the mesh library's block
        auto box8 = [](RenderMesh& m, const Vec3 (&lo)[4], const Vec3 (&hi)[4], const Vec3& col) { MeshBuilder::emitBlock(m, lo, hi, col); };
        const double ss0 = s0 + 0.3, ss1 = s1 - 0.3;
        // posts and beams
        for (double st = s0 + 6; st <= s1 - 6; st += 12) {
            if (inGap(st - 0.3, st + 0.3)) continue;
            const Vec2 q = pointAt(e.xy, e.s, st), t = tangentAtStation(e.xy, e.s, st), nr = perp(t);
            const double y = platTop(st);
            trackBox(steel, q + nr * mid, t, 0.11, 0.11, y, y + kRoof + 0.05, kStationSteel);
            const Vec3 lo[4] = {roofPt(st - 0.08, -roofHalf, 0), roofPt(st + 0.08, -roofHalf, 0), roofPt(st + 0.08, roofHalf, 0), roofPt(st - 0.08, roofHalf, 0)};
            const Vec3 hi[4] = {roofPt(st - 0.08, -roofHalf, kBeam), roofPt(st + 0.08, -roofHalf, kBeam), roofPt(st + 0.08, roofHalf, kBeam), roofPt(st - 0.08, roofHalf, kBeam)};
            // the beam tapers to its tips: deep over the post, a third of it at the edges
            Vec3 lo2[4]; for (int i = 0; i < 4; ++i) lo2[i] = lo[i] + Vec3(0, static_cast<Real>(kBeam * 0.65), 0);
            const Vec3 midLo[4] = {roofPt(st - 0.08, -0.15, 0), roofPt(st + 0.08, -0.15, 0), roofPt(st + 0.08, 0.15, 0), roofPt(st - 0.08, 0.15, 0)};
            const Vec3 midHi[4] = {roofPt(st - 0.08, -0.15, kBeam), roofPt(st + 0.08, -0.15, kBeam), roofPt(st + 0.08, 0.15, kBeam), roofPt(st - 0.08, 0.15, kBeam)};
            box8(steel, midLo, midHi, kStationSteel);
            const Vec3 aLo[4] = {lo2[0], lo2[1], midLo[1], midLo[0]}, aHi[4] = {hi[0], hi[1], midHi[1], midHi[0]};
            const Vec3 bLo[4] = {midLo[3], midLo[2], lo2[2], lo2[3]}, bHi[4] = {midHi[3], midHi[2], hi[2], hi[3]};
            box8(steel, aLo, aHi, kStationSteel); box8(steel, bLo, bHi, kStationSteel);
        }
        // the sheet: ribs across the roof (they run the way it drains), a trapezoid every kRib along the platform,
        // seen from above and below
        const double prof[5][2] = {{0.0, 0.0}, {0.3, 0.0}, {0.4, kRibH}, {0.9, kRibH}, {1.0, 0.0}};
        for (double st = ss0; st + kRib <= ss1 + 1e-6; st += kRib) {
            if (inGap(st, st + kRib)) continue;
            for (int k = 0; k < 4; ++k) {
                const double a = st + prof[k][0] * kRib, b = st + prof[k + 1][0] * kRib;
                const double ya = kBeam + prof[k][1], yb = kBeam + prof[k + 1][1];
                const Vec3 A = roofPt(a, -roofHalf - 0.15, ya), B = roofPt(b, -roofHalf - 0.15, yb), C = roofPt(b, roofHalf + 0.15, yb), D = roofPt(a, roofHalf + 0.15, ya);
                const bool up = cross(B - A, D - A).y > 0;
                const Vec3 sheetUp = normalize(cross(B - A, D - A)) * (up ? Real(1) : Real(-1));
                MeshBuilder::emitQuad(canopy, A, B, C, D, sheetUp, kCanopySheet);           // the top
                MeshBuilder::emitQuad(canopy, A, B, C, D, sheetUp * Real(-1), kCanopySheet);   // and its underside
            }
        }
        // the fascias, 0.4 deep, hiding the sheet's ribbed edge; and the lamps
        for (int k = 0; k + 1 < n; ++k) {
            const double st0 = std::max(ss0, s0 + (s1 - s0) * k / (n - 1)), st1 = std::min(ss1, s0 + (s1 - s0) * (k + 1) / (n - 1));
            if (st1 - st0 < 0.05 || inGap(st0, st1)) continue;
            for (int edge = -1; edge <= 1; edge += 2) {
                const double u0 = edge * (roofHalf + 0.15), u1 = u0 + edge * 0.05;
                const Vec3 lo[4] = {roofPt(st0, u0, kBeam - 0.3), roofPt(st1, u0, kBeam - 0.3), roofPt(st1, u1, kBeam - 0.3), roofPt(st0, u1, kBeam - 0.3)};
                const Vec3 hi[4] = {roofPt(st0, u0, kBeam + 0.1), roofPt(st1, u0, kBeam + 0.1), roofPt(st1, u1, kBeam + 0.1), roofPt(st0, u1, kBeam + 0.1)};
                box8(steel, lo, hi, kStationSteel);
            }
        }
        for (double st : canopyLampStations(s0, s1)) {
            if (inGap(st - 0.7, st + 0.7)) continue;
            // a linear fitting under the sheet between the beams: its housing, and the diffuser under it that lights
            const Vec3 hLo[4] = {roofPt(st - 0.6, -0.08, kBeam - 0.08), roofPt(st + 0.6, -0.08, kBeam - 0.08), roofPt(st + 0.6, 0.08, kBeam - 0.08), roofPt(st - 0.6, 0.08, kBeam - 0.08)};
            const Vec3 hHi[4] = {roofPt(st - 0.6, -0.08, kBeam), roofPt(st + 0.6, -0.08, kBeam), roofPt(st + 0.6, 0.08, kBeam), roofPt(st - 0.6, 0.08, kBeam)};
            box8(steel, hLo, hHi, kStationSteel);
            const Vec3 dLo[4] = {roofPt(st - 0.57, -0.06, kBeam - 0.12), roofPt(st + 0.57, -0.06, kBeam - 0.12), roofPt(st + 0.57, 0.06, kBeam - 0.12), roofPt(st - 0.57, 0.06, kBeam - 0.12)};
            const Vec3 dHi[4] = {roofPt(st - 0.57, -0.06, kBeam - 0.08), roofPt(st + 0.57, -0.06, kBeam - 0.08), roofPt(st + 0.57, 0.06, kBeam - 0.08), roofPt(st - 0.57, 0.06, kBeam - 0.08)};
            box8(lamp, dLo, dHi, kStationLamp);
        }
    }
}

}  // namespace

std::vector<RailTrack> railTracks(const Result& r) {
    std::vector<RailTrack> out;
    for (size_t li = 0; li < r.lanes.lanes.size(); ++li) {
        if (!trackLane(r, static_cast<int>(li))) continue;
        const Lane& l = r.lanes.lanes[li]; if (l.s.size() < 2) continue;
        const EdgeSpec& e = r.graph.edges[static_cast<size_t>(l.parent)];
        RailTrack t; t.edge = e.id; t.cls = e.cls; t.dir = l.dir >= 0 ? +1 : -1;
        const double len = l.s.back(); const int n = std::max(2, static_cast<int>(std::ceil(len / 2.0)) + 1);
        for (int k = 0; k < n; ++k) {
            const double st = len * k / (n - 1); const Vec2 p = pointAt(l.xy, l.s, st);
            t.pts.push_back(world(p, r.heights->deck(static_cast<int>(li), p) + kRailHeadAbove));
        }
        if (t.dir < 0) std::reverse(t.pts.begin(), t.pts.end());
        // the platforms beside it: on the side it is outermost on, in its own running stations
        for (const PlatformSpec& P : e.platforms)
            for (int side = -1; side <= 1; side += 2) {
                if (P.side != 0 && P.side != side) continue;
                if (outerTrack(r, l.parent, side, nullptr) != static_cast<int>(li)) continue;
                const Vec2 q = pointAt(e.xy, e.s, std::clamp((P.s0 + P.s1) / 2, 0.0, e.s.back()));
                RailStop st; st.name = P.name;
                const double a = project(l.xy, l.s, pointAt(e.xy, e.s, std::clamp(P.s0, 0.0, e.s.back()))).station;
                const double b = project(l.xy, l.s, pointAt(e.xy, e.s, std::clamp(P.s1, 0.0, e.s.back()))).station;
                st.s0 = t.dir > 0 ? std::min(a, b) : len - std::max(a, b);
                st.s1 = t.dir > 0 ? std::max(a, b) : len - std::min(a, b);
                // The train's left is its body's +x (rail_sim CarPose: cross(up, forward)). The plan's left normal,
                // perp(), is the OTHER side once plan y is world z -- the doors opened away from the platform
                // (Glenn, 2026-10-10: "the doors did not slide open for me ... It may have opened on the wrong side").
                st.side = -side * t.dir;
                st.top = r.heights->deck(static_cast<int>(li), pointAt(l.xy, l.s, project(l.xy, l.s, q).station)) + kRailHeadAbove + P.h;
                // its night lights: under every other canopy lamp, a little below the fitting
                {
                    double off = 0; outerTrack(r, l.parent, side, &off);
                    const double midLat = off + side * (kPlatformGap + P.w / 2);
                    const std::vector<double> ls = canopyLampStations(std::max(0.0, P.s0), std::min(e.s.back(), P.s1));
                    for (std::size_t k = 0; k < ls.size(); k += 2) {
                        const Vec2 pq = pointAt(e.xy, e.s, ls[k]), nr = perp(tangentAtStation(e.xy, e.s, ls[k]));
                        st.lamps.push_back(world(pq + nr * midLat, r.heights->deck(static_cast<int>(li), pq + nr * off) + kRailHeadAbove + P.h + 3.1));
                    }
                }
                // the way in: a footway over this platform, at its end away from the line
                for (const EdgeSpec& f : r.graph.edges) {
                    if (!footwayClass(r.graph, f.cls) || f.xy.size() < 2) continue;
                    bool over = false;
                    for (const Vec2& X : crossings(e.xy, f.xy)) { const double sx = project(e.xy, e.s, X).station; if (sx >= P.s0 && sx <= P.s1) over = true; }
                    if (!over) continue;
                    const bool frontFar = project(e.xy, e.s, f.xy.front()).distance >= project(e.xy, e.s, f.xy.back()).distance;
                    st.hasAccess = true; st.access = frontFar ? f.xy.front() : f.xy.back();
                }
                t.stops.push_back(st);
            }
        out.push_back(std::move(t));
    }
    return out;
}

std::vector<NamedMesh> buildMeshes(const Result& r) {
    const RoadLabGraph& g = r.graph; const LaneSet& L = r.lanes; const DeckHeight& H = *r.heights;
    RenderMesh asphalt, concrete, sidewalk, shoulder, median, paintW, paintY, terrain, guardrail, ballast, sleepers, rails, platform;
    RenderMesh canopy, stationSteel, stationLamp;
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
    // ...and it returns how far down that surface reaches (its underside), NaN when nothing covers: the side is a
    // face below that. A deck's slab is its surface's thickness (laneThick), a layer's the thickness it is built with.
    std::vector<double> laneThick(L.lanes.size(), 0.0);
    for (const Surface& s : r.pavement.decks)
        for (int o : s.triOwner) if (o >= 0 && static_cast<size_t>(o) < laneThick.size()) laneThick[static_cast<size_t>(o)] = std::max(laneThick[static_cast<size_t>(o)], s.thick);
    const double coverThick[3] = {kSidewalkThick, kShoulderThick, kMedianThick};
    const std::function<double(const Vec2&, double)> hiddenFn = [&](const Vec2& q, double top) {
        auto hides = [&](double z) { return z >= top - 0.02 && z <= top + 0.5; };
        double under = std::nan("");   // the deepest underside among the surfaces covering the side
        auto take = [&](double bottom) { if (std::isnan(under) || bottom < under) under = bottom; };
        for (int li : laneGrid.at(q)) {
            const Box2& bb = laneBox[static_cast<size_t>(li)];
            if (q.x < bb.minX || q.x > bb.maxX || q.y < bb.minY || q.y > bb.maxY || !footprint[static_cast<size_t>(li)].contains(q)) continue;
            const double z = H.deck(li, q);
            if (hides(z)) take(z - laneThick[static_cast<size_t>(li)]);
        }
        for (int k = 0; k < 3; ++k) {
            const Cover& c = covers[k];
            for (int pi : coverGrid[k].at(q)) {
                const Box2& bb = c.box[static_cast<size_t>(pi)];
                if (q.x < bb.minX || q.x > bb.maxX || q.y < bb.minY || q.y > bb.maxY || !c.prep[static_cast<size_t>(pi)].contains(q)) continue;
                const double z = H.layerHeight(c.roads, q) + c.lift;
                if (hides(z)) take(z - coverThick[k]);
            }
        }
        return under;
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
        // a deck all of whose lanes are track is a ballast bed, not asphalt
        bool track = !s.triOwner.empty(), foot = !s.triOwner.empty();
        for (int o : s.triOwner) if (!trackLane(r, o)) { track = false; break; }
        for (int o : s.triOwner) if (!footwayLane(r, o)) { foot = false; break; }   // ...and one all of footway is paved like a sidewalk
        slab(track ? ballast : foot ? sidewalk : asphalt, concrete, s.verts, s.tris, s.boundary, s.thick, 0.0, track ? kBallast : foot ? kSidewalk : kAsphalt, kConcrete, &uv, &seamFn, &groundUnder, &hiddenFn);
    }
    // layers
    auto layer = [&](const PolySet& poly, RenderMesh& top, const Vec3& color, double lift, double thick, bool anyRoad) {
        const std::vector<int> roads = layerRoads(g, anyRoad);
        for (const FlatMesh& m : layerMeshes(g, H, poly, roads)) slab(top, concrete, m.verts, m.tris, m.boundary, thick, lift, color, kConcrete, nullptr, nullptr, &groundUnder, &hiddenFn);
    };
    mark("deck sides");
    layer(r.pavement.sidewalk, sidewalk, kSidewalk, kSidewalkLift, kSidewalkThick, false); mark("sidewalk sides");
    layer(r.pavement.shoulder, shoulder, kShoulder, 0.0, kShoulderThick, false); mark("shoulder sides");
    layer(r.pavement.median, median, kMedian, 0.10, kMedianThick, true); mark("median sides");
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
    // the roads that carry barriers: freeways and ramps, and any class whose table AUTHORS one (the
    // island's mountain road, ADR-0117) -- streets without are the lot pass's business
    auto barriered = [&](const std::string& cls) {
        if (cls == "freeway") return true;
        const RoadClassSpec& c = r.graph.classes.count(cls) ? r.graph.classes.at(cls) : r.graph.classes.begin()->second;
        for (const BarrierSpec& b : c.edges) if (b.set && b.kind != BarrierKind::None) return true;
        return false;
    };
    auto isDeckLane = [&](int li) {
        if (li < 0 || li >= static_cast<int>(L.lanes.size())) return false; const Lane& l = L.lanes[static_cast<size_t>(li)];
        if (l.isConnector() || l.parent < 0) return false; const EdgeSpec& e = r.graph.edges[static_cast<size_t>(l.parent)]; return e.isRamp() || barriered(e.cls);
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
    // RAIL: sleepers and rails on every track lane, platforms where the graph puts a station
    for (size_t li = 0; li < L.lanes.size(); ++li)
        if (trackLane(r, static_cast<int>(li))) trackKit(sleepers, rails, r, static_cast<int>(li));
    for (size_t li = 0; li < L.lanes.size(); ++li)
        if (footwayLane(r, static_cast<int>(li))) footwayStairs(sidewalk, r, static_cast<int>(li));
    const std::vector<StationStair> stairs = stationStairs(r);
    const std::function<bool(const Vec2&)> anyPavedAt = [&](const Vec2& q) {
        for (int ojI : laneGrid.at(q)) {
            const size_t oj = static_cast<size_t>(ojI); const Box2& bb = laneBox[oj];
            if (q.x >= bb.minX && q.x <= bb.maxX && q.y >= bb.minY && q.y <= bb.maxY && footprint[oj].contains(q)) return true;
        }
        return false;
    };
    for (size_t ei = 0; ei < g.edges.size(); ++ei)
        if (trackClass(g, g.edges[ei].cls))
            for (const PlatformSpec& P : g.edges[ei].platforms) platformKit(platform, concrete, canopy, stationSteel, stationLamp, r, g.edges[ei], static_cast<int>(ei), P, stairs, groundUnder, anyPavedAt);
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
        const EdgeSpec& e = r.graph.edges[static_cast<size_t>(l.parent)]; if (e.isRamp() || e.cls == "freeway" || r.graph.cls(e).rank < 1 || trackClass(r.graph, e.cls)) continue;
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
        if (trackLane(r, static_cast<int>(li)) || footwayLane(r, static_cast<int>(li))) continue;   // a track is not painted, nor a path
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
    add("ballast", ballast, kBallast); add("sleeper", sleepers, kSleeper); add("rail", rails, kRailSteel); add("platform", platform, kPlatform);
    add("canopy", canopy, kCanopySheet); add("station_steel", stationSteel, kStationSteel); add("station_lamp", stationLamp, kStationLamp);
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
    // freeways and ramps -- the pavement whose outline is a barrier line by default; anything else a ramp
    // meets is a road it carries on into (a mouth, never walled)
    auto isFreewayLane = [&](int li) {
        if (li < 0 || li >= static_cast<int>(L.lanes.size())) return false; const Lane& l = L.lanes[static_cast<size_t>(li)];
        if (l.isConnector() || l.parent < 0) return false; const EdgeSpec& e = r.graph.edges[static_cast<size_t>(l.parent)]; return e.isRamp() || e.cls == "freeway";
    };
    // ...and every road whose class AUTHORS a barrier (the island's mountain road, ADR-0117)
    auto authored = [&](const std::string& cls) {
        auto it = r.graph.classes.find(cls); if (it == r.graph.classes.end()) return false;
        for (const BarrierSpec& b : it->second.edges) if (b.set && b.kind != BarrierKind::None) return true;
        return false;
    };
    auto isDeckLane = [&](int li) {
        if (isFreewayLane(li)) return true;
        if (li < 0 || li >= static_cast<int>(L.lanes.size())) return false; const Lane& l = L.lanes[static_cast<size_t>(li)];
        return !l.isConnector() && l.parent >= 0 && authored(r.graph.edges[static_cast<size_t>(l.parent)].cls);
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
    int ringFoot = 0;   // the ring being walked is a footway's (1) or the other decks' (0): its owner is one of those
    auto deckLaneAt = [&](const Vec2& q) {
        for (int ojI : laneGrid.at(q)) {
            const size_t oj = static_cast<size_t>(ojI); const Box2& bb = laneBox[oj];
            if (!isDeckLane(ojI) || static_cast<int>(footwayLane(r, ojI)) != ringFoot) continue;
            if (q.x < bb.minX || q.x > bb.maxX || q.y < bb.minY || q.y > bb.maxY || !footprint[oj].contains(q)) continue;
            return ojI;
        }
        return -1;
    };
    auto groundAt = [&](const Vec2& p) { return r.hasTerrain ? r.terrain.sample(p.x, p.y) : 0.0; };
    // where a station stair leaves a footbridge: no parapet across its top
    const std::vector<StationStair> stairs = stationStairs(r);
    auto stairTop = [&](const Vec2& q) {
        for (const StationStair& st : stairs) {
            if (st.up) continue;   // (its top is on the platform, not the footway)
            const Vec2 a = st.top - st.across * (st.width / 2), b = st.top + st.across * (st.width / 2), ab = b - a;
            const double u = std::clamp(dot(q - a, ab) / std::max(1e-9, dot(ab, ab)), 0.0, 1.0);
            if (distance(q, a + ab * u) < 0.4) return true;
        }
        return false;
    };

    // THE BARRIER LINE IS THE ROAD'S OWN OUTLINE (2026-09-08, Glenn: "it's like sidewalks but they're walls").
    // A sidewalk is a ribbon offset from the road, unioned and clipped by Clipper, which is why it never
    // fragments and never twists. A barrier was assembled instead from the deck's boundary TRIANGLE EDGES,
    // chained by vertex index: any edge that failed a test ended the chain (775 runs over 25 km), and a sharp
    // joint had to be mitred by hand. Offsetting the freeway and ramp pavement gives the same line as one
    // polygon, already mitred and closed, and its boundary rings are the ordered curves a sweep wants.
    // A FOOTWAY's outline is its own: a footbridge crosses the tracks it clears, and in one plan union with them its
    // edges over the line vanished inside theirs -- no parapet the whole way across (the rail lab's first bridge)
    PolySet deckPaved, footPaved;
    for (size_t li = 0; li < L.lanes.size(); ++li)
        if (isDeckLane(static_cast<int>(li)) && !r.pavement.footprints[li].empty()) {
            PolySet& into = footwayLane(r, static_cast<int>(li)) ? footPaved : deckPaved;
            into = unionSets(into, r.pavement.footprints[li]);
        }
    if (deckPaved.empty() && footPaved.empty()) return out;
    const double standOff = 0.05;                    // the barrier hugs the pavement edge; its body lies outside it
    std::vector<Ring> rings; std::vector<int> ringIsFoot;
    for (const PolySet* set : {&deckPaved, &footPaved}) {
        if (set->empty()) continue;
        const int foot = set == &footPaved ? 1 : 0;
        for (const Polygon2& pg : offsetSet(*set, standOff)) {
            if (pg.outer.size() >= 8) { rings.push_back(pg.outer); ringIsFoot.push_back(foot); }
            for (const Ring& h : pg.holes) if (h.size() >= 8) { rings.push_back(h); ringIsFoot.push_back(foot); }
        }
    }

    int walled = 0; double bandM = 0;
    for (size_t ri = 0; ri < rings.size(); ++ri) {
        const Ring& ring = rings[ri]; ringFoot = ringIsFoot[ri];
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
                    // another road's pavement (not a freeway or ramp, not this road's own lanes): it carries on
                    if (hit >= 0 && !isFreewayLane(hit) && L.lanes[static_cast<size_t>(hit)].parent != L.lanes[static_cast<size_t>(inLane)].parent) return true;
                }
                return false;
            }();
            if (onOtherPavement) role[i] = EdgeRole::Seam;
            else if (ne && nd <= 1.2) role[i] = EdgeRole::Seam;
            else if (footwayClass(r.graph, me.cls)) role[i] = stairTop(pts[i]) ? EdgeRole::Seam : hAbove >= 1.0 ? EdgeRole::Elevated : EdgeRole::AtGrade;   // a path: walled where it can be fallen off          // pavement right there: the outline runs through a junction
            else if (ne && ne->cls == "freeway" && me.cls == "freeway" && ne != &me && nd <= 14.0) role[i] = EdgeRole::Median;
            else if (ne && ne->cls != "freeway" && !ne->isRamp() && nd <= 32.0) role[i] = EdgeRole::VsStreet;
            else role[i] = hAbove >= 1.5 ? EdgeRole::Elevated : EdgeRole::AtGrade;
            spec[i] = barrierFor(r.graph.cls(me), role[i]);
            if (spec[i].kind != BarrierKind::None && spec[i].minDrop > 0.0 && z[i] - groundAt(pts[i] + outward * 5.0) < spec[i].minDrop)
                spec[i] = BarrierSpec{};   // nowhere to fall here: no rail
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
