#include "city_lots.h"

#include "parcel.h"          // subdivideBlock, Lot, ParcelParams
#include "road_net.h"        // RoadEntity + navRoadGraph (growLotBuildingsOnNets)
#include "road_network.h"    // RoadGraph (edge blocks walk its chains)
#include "architect.h"       // DistrictMap + archetype tables (the architect pass)
#include "shape_grammar.h"   // scopeFromFootprint, growBuilding — REAL buildings
#include "road_mesh.h"       // triangulatePolygon (lot-shaped park pads)
#include "street_kit.h"      // streetLamp (plaza lamp posts)
#include "block_grade.h"     // gradeBlocks (in-pass block terracing)
#include "../../../log.h"    // plaza site report (find them on the map)
#include "../../mesh_builder.h"   // MeshBuilder::append (merge parts by PartId)
#include <algorithm>
#include <array>
#include <cmath>
#include <map>

namespace engine {

namespace {
// A small deterministic hash RNG so the whole pass reproduces from `seed` (no
// global rng, no Math.random) — one stream per lot, mixed from stable inputs.
struct Hash {
    uint32_t s;
    explicit Hash(uint32_t seed) : s(seed ? seed : 0x9e3779b9u) {}
    uint32_t next() {
        s ^= s << 13; s ^= s >> 17; s ^= s << 5;
        return s;
    }
    Real unit() { return (next() & 0xffffff) / static_cast<Real>(0x1000000); }
    Real range(Real a, Real b) { return a + (b - a) * unit(); }
};
uint32_t mix(uint32_t a, uint32_t b) {
    uint32_t h = a * 0x85ebca6bu ^ (b + 0x9e3779b9u + (a << 6) + (a >> 2));
    h ^= h >> 15; h *= 0xc2b2ae35u; h ^= h >> 13;
    return h;
}

Vec3 colorFor(const std::string& t) {
    if (t == "home")   return {0.72, 0.55, 0.45};
    if (t == "shop")   return {0.82, 0.70, 0.42};
    if (t == "office") return {0.55, 0.62, 0.72};
    if (t == "civic")  return {0.80, 0.80, 0.85};
    if (t == "park")   return {0.35, 0.60, 0.35};
    return {0.72, 0.70, 0.64};
}

// Distance from p to segment [a,b] (shared by the sculptors below).
Real distToSeg(const Vec2& p, const Vec2& a, const Vec2& b) {
    Vec2 ab = b - a;
    Real len2 = ab.lengthSquared();
    Real t = len2 > 1e-12 ? dot(p - a, ab) / len2 : 0.0;
    t = t < 0 ? 0 : (t > 1 ? 1 : t);
    return (p - Vec2(a.x + ab.x * t, a.y + ab.y * t)).length();
}

// Merge a grown kit's parts into the by-PartId output array.
void appendKit(const BuildingMesh& kit, std::vector<RenderMesh>* outParts) {
    if (!outParts) return;
    for (const RenderMesh& part : kit.parts) {
        const int mi = part.materialIndex;
        if (mi >= 0 && mi < static_cast<int>(outParts->size()))
            MeshBuilder::append((*outParts)[mi], part);
    }
}

// PARK SCULPTING (device: "sculpting building lots to include landscaping
// and walking paths"): a park lot is a designed green, not a bare pad — a
// centre PLAZA, walking-path SPOKES out to the surrounding streets, a stone
// fountain, benches facing it, planter hedges at the path mouths, and
// deterministic TREE SPOTS the hosts plant real trees at. Lawn and path
// colours are BAKED into the pad's vertices (the lot's `color` goes white so
// the bake reads as-is in both the viewer and the offline tracer).
void sculptPark(LotBuilding& g, const Poly2& poly, Real h,
                const std::function<Real(Real, Real)>& ground, uint32_t seed,
                std::vector<RenderMesh>* outParts) {
    const Vec3 grass(0.30, 0.50, 0.26);
    const Vec3 pathCol(0.72, 0.68, 0.60);        // decomposed granite
    auto gy = [&](const Vec2& v) { return ground ? ground(v.x, v.y) : Real(0); };
    Hash rng(mix(seed, 0x9A46B1u));

    (void)grass;
    RenderMesh m;
    // NO LAWN SLAB (device: "could we use the actual terrain for the park
    // grounds?"): the pad plane only sampled terrain at its boundary corners,
    // so on any real slope its interior hovered or sank — and everything
    // placed on its plane (the old planting) went under the ground with it.
    // The park floor IS the terrain now; the lot reads as a park because of
    // what stands on it — paths, fence, furniture, planting — every piece
    // sampling the ground under its own feet.

    const Vec2 c = centroid(poly);
    const Real A = area(poly);
    const Real py = h + 0.03;                    // paths float just off the lawn
    const Real r0 = std::min(Real(4.5), std::max(Real(2.2), std::sqrt(A) * 0.12));

    // The centre PLAZA: a paved disc fan, draped on the terrain.
    const int PN = 14;
    Vec2 rim[PN];
    for (int k = 0; k < PN; ++k) {
        const Real a2 = 6.283185307179586 * k / PN;
        rim[k] = c + Vec2(std::cos(a2), std::sin(a2)) * r0;
    }
    for (int k = 0; k < PN; ++k) {
        const Vec2& a = rim[k];
        const Vec2& b = rim[(k + 1) % PN];
        MeshBuilder::emitTri(m, Vec3(c.x, gy(c) + py, c.y),
                             Vec3(a.x, gy(a) + py, a.y),
                             Vec3(b.x, gy(b) + py, b.y), Vec3(0, 1, 0), pathCol);
        // Curb skirt: the plaza is a raised slab, not a floating decal — its
        // rim drops to below the lawn so slopes never open a gap under it.
        Vec2 n2 = normalize(Vec2(b.y - a.y, a.x - b.x));
        MeshBuilder::emitQuad(m, Vec3(a.x, gy(a) + py - 0.45, a.y),
                              Vec3(b.x, gy(b) + py - 0.45, b.y),
                              Vec3(b.x, gy(b) + py, b.y),
                              Vec3(a.x, gy(a) + py, a.y),
                              Vec3(n2.x, 0, n2.y), pathCol * 0.85);
    }

    // Walking-path SPOKES: from the plaza out to the midpoints of the longest
    // lot edges — the desire lines to the surrounding sidewalks.
    struct Spoke { Vec2 a, b; };
    std::vector<Spoke> spokes;
    for (std::size_t i = 0; i < poly.size() && spokes.size() < 4; ++i) {
        const Vec2& ea = poly[i];
        const Vec2& eb = poly[(i + 1) % poly.size()];
        if ((eb - ea).length() < 12.0) continue;
        Vec2 mouth = (ea + eb) * 0.5;
        Vec2 to = mouth - c;
        const Real len = to.length();
        if (len < r0 + 2.5) continue;
        Vec2 dir = to * (1.0 / len);
        Vec2 p0 = c + dir * (r0 * 0.85);
        Vec2 perp(-dir.y, dir.x);
        const Real hw = 0.8;                     // path half-width
        const int segs = std::max(1, static_cast<int>((len - r0 * 0.85) / 3.0));
        for (int s = 0; s < segs; ++s) {
            Vec2 q0 = p0 + dir * ((len - r0 * 0.85) * s / segs);
            Vec2 q1 = p0 + dir * ((len - r0 * 0.85) * (s + 1) / segs);
            const Vec3 L0(q0.x - perp.x * hw, gy(q0 - perp * hw) + py, q0.y - perp.y * hw);
            const Vec3 R0(q0.x + perp.x * hw, gy(q0 + perp * hw) + py, q0.y + perp.y * hw);
            const Vec3 L1(q1.x - perp.x * hw, gy(q1 - perp * hw) + py, q1.y - perp.y * hw);
            const Vec3 R1(q1.x + perp.x * hw, gy(q1 + perp * hw) + py, q1.y + perp.y * hw);
            MeshBuilder::emitQuad(m, L0, R0, R1, L1, Vec3(0, 1, 0), pathCol);
            // Curb skirts: the path is a slab with thickness — its long edges
            // drop below the lawn so a terrain dip never leaves it hovering
            // (device: "the walkway is floating").
            const Vec3 drop(0, -0.45, 0);
            const Vec3 pL(-perp.x, 0, -perp.y), pR(perp.x, 0, perp.y);
            MeshBuilder::emitQuad(m, L0 + drop, L1 + drop, L1, L0, pL, pathCol * 0.85);
            MeshBuilder::emitQuad(m, R0 + drop, R1 + drop, R1, R0, pR, pathCol * 0.85);
        }
        spokes.push_back({p0, mouth});
    }

    // The park is FURNISHED, not painted: every item below sits on the
    // terrain under its own feet, and every item CLAIMS its footprint in a
    // shared registry — nothing may stand where a path runs or another item
    // already stands (device: "spaced out and placed correctly instead of
    // over one another").
    std::vector<std::pair<Vec2, Real>> claimed;
    auto clearAt = [&](const Vec2& p2, Real r) {
        for (const Spoke& s : spokes)
            if (distToSeg(p2, s.a, s.b) < r + 1.1) return false;   // path + margin
        for (const auto& [q, qr] : claimed)
            if ((q - p2).length() < r + qr) return false;
        return true;
    };
    auto claim = [&](const Vec2& p2, Real r) { claimed.emplace_back(p2, r); };

    if (outParts) {
        BuildingMesh kit;
        const Real fy = gy(c) + py;
        const Vec3 stone(0.72, 0.70, 0.66);
        const Vec3 up(0, 1, 0);
        if (r0 > 2.5) {
            // The stone FOUNTAIN: basin ring + column + bowl (one lathe), a
            // still water disc (glass) inside the basin.
            std::vector<Vec2> basin = {
                {r0 * 0.42, 0.0},  {r0 * 0.44, 0.42}, {r0 * 0.36, 0.50},
                {r0 * 0.34, 0.14}, {0.14, 0.14},      {0.11, 1.05},
                {0.30, 1.18},      {0.24, 1.32},      {0.0, 1.40}};
            MeshBuilder::append(
                (*outParts)[static_cast<std::size_t>(PartId::Trim)],
                latheMesh(Vec3(c.x, fy, c.y), basin, 14, stone));
            std::vector<Vec2> water = {{r0 * 0.33, 0.36}, {0.0, 0.36}};
            MeshBuilder::append(
                (*outParts)[static_cast<std::size_t>(PartId::Glass)],
                latheMesh(Vec3(c.x, fy, c.y), water, 14, Vec3(0.20, 0.34, 0.40)));
            claim(c, r0 * 0.5);
        }
        // BENCHES around the plaza, facing the centre.
        const Vec3 wood(0.45, 0.34, 0.22);
        const int nb = 3 + static_cast<int>(rng.unit() * 3);
        for (int k = 0; k < nb; ++k) {
            const Real a2 = 6.283185307179586 * (k + rng.range(0.05, 0.3)) / nb;
            Vec2 dir(std::cos(a2), std::sin(a2));
            Vec2 bp = c + dir * (r0 + 0.6);
            if (!pointInPolygon(poly, bp) || !clearAt(bp, 1.0)) continue;
            claim(bp, 1.0);
            Vec2 t2(-dir.y, dir.x);
            Vec3 t3(t2.x, 0, t2.y), n3(dir.x, 0, dir.y);
            const Real by = gy(bp) + 0.02;
            Vec3 o = Vec3(bp.x, by + 0.42, bp.y) - t3 * 0.8 - n3 * 0.22;
            emitBox(kit, Scope{o, {t3, up, n3}, Vec3(1.6, 0.07, 0.44)},
                    PartId::Wood, wood);
            for (int lg = 0; lg < 2; ++lg)
                emitBox(kit, Scope{Vec3(bp.x, by, bp.y) +
                                       t3 * (lg ? 0.55 : -0.65) - n3 * 0.18,
                                   {t3, up, n3}, Vec3(0.10, 0.42, 0.36)},
                        PartId::Metal, Vec3(0.20, 0.21, 0.22));
        }
        // PLANTER hedges where each path meets the street: a stone curb box
        // with clipped greenery on top, one per side of the mouth.
        for (const Spoke& s : spokes) {
            Vec2 dir = normalize(s.b - s.a);
            Vec2 perp(-dir.y, dir.x);
            for (Real side : {Real(1), Real(-1)}) {
                Vec2 pp = s.b - dir * 1.2 + perp * (side * 1.6);
                if (!pointInPolygon(poly, pp)) continue;
                for (const auto& [q, qr] : claimed)   // planters flank paths by
                    if ((q - pp).length() < 0.9 + qr) { pp.x = 1e30; break; }
                if (pp.x > 1e29) continue;            // design; only item claims
                claim(pp, 0.9);
                const Real by = gy(pp) + 0.02;
                Vec3 t3(perp.x, 0, perp.y), n3(dir.x, 0, dir.y);
                Vec3 o = Vec3(pp.x, by, pp.y) - t3 * 0.5 - n3 * 0.5;
                emitBox(kit, Scope{o, {t3, up, n3}, Vec3(1.0, 0.35, 1.0)},
                        PartId::Trim, stone * 0.9);
                const Vec3 hedge(0.24 + rng.range(0, 0.05), 0.40, 0.20);
                emitBox(kit, Scope{o + Vec3(0, 0.35, 0) + t3 * 0.1 + n3 * 0.1,
                                   {t3, up, n3}, Vec3(0.8, 0.55, 0.8)},
                        PartId::Foliage, hedge);
            }
        }
        // FLOWER BEDS: a stone curb with a bright planted top, on the plaza
        // ring between the benches.
        const Vec3 bloom[3] = {{0.72, 0.22, 0.26},    // red
                               {0.82, 0.68, 0.20},    // marigold
                               {0.56, 0.32, 0.66}};   // violet
        const int nf = 2 + static_cast<int>(rng.unit() * 2);
        for (int k = 0; k < nf * 3; ++k) {
            const Real a2 = rng.unit() * 6.283185307179586;
            Vec2 dir(std::cos(a2), std::sin(a2));
            Vec2 fp = c + dir * (r0 + 1.1);
            if (!pointInPolygon(poly, fp) || !clearAt(fp, 0.8)) continue;
            claim(fp, 0.8);
            const Real by = gy(fp) + 0.02;
            Vec3 t3(-dir.y, 0, dir.x), n3(dir.x, 0, dir.y);
            Vec3 o = Vec3(fp.x, by, fp.y) - t3 * 0.55 - n3 * 0.55;
            emitBox(kit, Scope{o, {t3, up, n3}, Vec3(1.1, 0.22, 1.1)},
                    PartId::Trim, stone * 0.92);
            emitBox(kit, Scope{o + Vec3(0, 0.22, 0) + t3 * 0.12 + n3 * 0.12,
                               {t3, up, n3}, Vec3(0.86, 0.14, 0.86)},
                    PartId::Foliage, bloom[static_cast<int>(rng.unit() * 2.99)]);
        }
        // SHRUBS: loose clipped bushes scattered on the lawn, off everything.
        Real mnx = 1e30, mnz = 1e30, mxx = -1e30, mxz = -1e30;
        for (const Vec2& v : poly) {
            mnx = std::min(mnx, v.x); mxx = std::max(mxx, v.x);
            mnz = std::min(mnz, v.y); mxz = std::max(mxz, v.y);
        }
        Poly2 shrubIn = inset(poly, 1.5);
        const int ns = std::min(8, std::max(2, static_cast<int>(A / 130)));
        for (int k = 0, placed = 0; k < ns * 5 && placed < ns; ++k) {
            Vec2 sp(mnx + rng.unit() * (mxx - mnx), mnz + rng.unit() * (mxz - mnz));
            if (shrubIn.size() < 3 || !pointInPolygon(shrubIn, sp)) continue;
            if (!clearAt(sp, 0.9)) continue;
            claim(sp, 0.9);
            ++placed;
            const Real by = gy(sp) + 0.0;
            const Real a2 = rng.unit() * 6.283185307179586;
            Vec3 t3(std::cos(a2), 0, std::sin(a2)), n3(-t3.z, 0, t3.x);
            const Real sw = rng.range(0.7, 1.15), sh = rng.range(0.5, 0.85);
            const Vec3 bush(0.20 + rng.range(0, 0.06), 0.38 + rng.range(0, 0.06), 0.18);
            emitBox(kit, Scope{Vec3(sp.x, by, sp.y) - t3 * (sw * 0.5) - n3 * (sw * 0.5),
                               {t3, up, n3}, Vec3(sw, sh, sw)},
                    PartId::Foliage, bush);
        }
        // The FENCE: post-and-rail around the lot line, with OPENINGS where
        // the walking paths meet the street (and one gap on tiny greens with
        // no paths, so every park can be entered). Small pocket greens skip
        // the fence entirely.
        if (A > 300.0) {
            Poly2 ring = inset(poly, 0.5);
            std::vector<Vec2> mouths;
            for (const Spoke& s : spokes) mouths.push_back(s.b);
            if (mouths.empty() && !ring.empty()) {
                // no paths: open the middle of the longest edge
                std::size_t bi = 0; Real bl = 0;
                for (std::size_t i = 0; i < ring.size(); ++i) {
                    Real l = (ring[(i + 1) % ring.size()] - ring[i]).length();
                    if (l > bl) { bl = l; bi = i; }
                }
                mouths.push_back((ring[bi] + ring[(bi + 1) % ring.size()]) * 0.5);
            }
            const Vec3 rail(0.30, 0.26, 0.22);   // stained timber
            for (std::size_t i = 0; i < ring.size(); ++i) {
                const Vec2& a = ring[i];
                const Vec2& b2 = ring[(i + 1) % ring.size()];
                const Real len = (b2 - a).length();
                if (len < 1.0) continue;
                Vec2 dir = (b2 - a) * (1.0 / len);
                Vec3 t3(dir.x, 0, dir.y), n3(-dir.y, 0, dir.x);
                const int posts = std::max(1, static_cast<int>(len / 2.4));
                const Real step = len / posts;
                for (int pi2 = 0; pi2 <= posts; ++pi2) {
                    Vec2 pp = a + dir * (step * pi2);
                    bool gap = false;
                    for (const Vec2& mo : mouths)
                        if ((mo - pp).length() < 2.6) { gap = true; break; }
                    if (gap) continue;
                    const Real by = gy(pp);
                    emitBox(kit, Scope{Vec3(pp.x, by, pp.y) - t3 * 0.06 - n3 * 0.06,
                                       {t3, up, n3}, Vec3(0.12, 0.95, 0.12)},
                            PartId::Wood, rail * 0.9);
                    if (pi2 == posts) continue;
                    {   // this post->next span is fenced: record for colliders
                        Vec2 np2 = a + dir * (step * (pi2 + 1));
                        bool ngap = false;
                        for (const Vec2& mo : mouths)
                            if ((mo - np2).length() < 2.6) { ngap = true; break; }
                        if (!ngap) g.fenceSegs.push_back({ pp, np2 });
                    }
                    // rails to the next post, skipped when it sits in a gap
                    Vec2 np = a + dir * (step * (pi2 + 1));
                    bool ngap = false;
                    for (const Vec2& mo : mouths)
                        if ((mo - np).length() < 2.6) { ngap = true; break; }
                    if (ngap) continue;
                    const Real ny = gy(np);
                    for (Real rh2 : {Real(0.34), Real(0.76)}) {
                        // rail follows the ground: a thin box laid corner to
                        // corner reads fine at this scale
                        Vec3 p0(pp.x, by + rh2, pp.y), p1(np.x, ny + rh2, np.y);
                        Vec3 mid = (p0 + p1) * 0.5;
                        Vec3 along = p1 - p0;
                        const Real alen = along.length();
                        if (alen < 1e-4) continue;
                        along = along / alen;
                        emitBox(kit, Scope{mid - along * (alen * 0.5) - n3 * 0.035 -
                                               up * 0.04,
                                           {along, up, n3}, Vec3(alen, 0.08, 0.07)},
                                PartId::Wood, rail);
                    }
                }
            }
        }
        appendKit(kit, outParts);
    }

    // TREE SPOTS: a loose ring between the plaza and the boundary, kept off
    // the paths AND off everything already claimed. (x, trunk scale, z) —
    // the hosts grow the real trees.
    const int want = std::min(12, std::max(3, static_cast<int>(A / 140)));
    for (int k = 0; k < want * 4 &&
                    static_cast<int>(g.treeSpots.size()) < want; ++k) {
        const Real a2 = rng.unit() * 6.283185307179586;
        const Real rr = r0 + 2.0 + rng.unit() * std::sqrt(A) * 0.35;
        Vec2 tp = c + Vec2(std::cos(a2), std::sin(a2)) * rr;
        Poly2 inner = inset(poly, 1.8);
        if (inner.size() < 3 || !pointInPolygon(inner, tp)) continue;
        if (!clearAt(tp, 1.4)) continue;
        bool clear = true;
        for (const Spoke& s : spokes)
            if (distToSeg(tp, s.a, s.b) < 2.2) { clear = false; break; }
        for (const Vec3& other : g.treeSpots)
            if ((Vec2(other.x, other.z) - tp).length() < 3.2) {
                clear = false;
                break;
            }
        if (!clear) continue;
        claim(tp, 1.4);
        g.treeSpots.push_back(Vec3(tp.x, rng.range(0.8, 1.3), tp.y));
    }

    g.padMesh = std::move(m);
    g.color = Vec3(1, 1, 1);   // vertex colours carry the lawn/path look
}

// A paved pad that FITS beneath a freeway deck: an asphalt lot with painted
// stalls for a PARKING lot, or a concrete yard with equipment cabinets + a post
// fence for a UTILITY lot. Draped on the terrain with a curb skirt so it never
// floats on a slope; the "furniture" is what makes the land-use read. Stored in
// g.padMesh like a park, so the host renders it on the terrain (no prism).
static void sculptUnderPad(LotBuilding& g, const Poly2& poly,
                           const std::function<Real(Real, Real)>& ground,
                           uint32_t seed, bool utility) {
    if (poly.size() < 3) return;
    Hash rng(mix(seed, utility ? 0xC0FFEEu : 0x5A1A5Au));
    auto gy = [&](const Vec2& v) { return ground ? ground(v.x, v.y) : Real(0); };
    const Real lift = 0.05;
    const Vec3 slab = utility ? Vec3(0.50, 0.50, 0.48)     // concrete
                              : Vec3(0.24, 0.24, 0.26);    // asphalt
    RenderMesh m;
    Poly2 pad = inset(poly, 1.4);
    if (pad.size() < 3) pad = poly;
    for (const auto& t : triangulatePolygon(pad)) {                    // slab
        const Vec2& a = pad[t[0]]; const Vec2& b = pad[t[1]]; const Vec2& c = pad[t[2]];
        MeshBuilder::emitTri(m, Vec3(a.x, gy(a) + lift, a.y),
                             Vec3(b.x, gy(b) + lift, b.y),
                             Vec3(c.x, gy(c) + lift, c.y), Vec3(0, 1, 0), slab);
    }
    for (std::size_t i = 0; i < pad.size(); ++i) {                     // curb skirt
        const Vec2& a = pad[i]; const Vec2& b = pad[(i + 1) % pad.size()];
        Vec2 n2 = normalize(Vec2(b.y - a.y, a.x - b.x));
        MeshBuilder::emitQuad(m, Vec3(a.x, gy(a) + lift - 0.4, a.y),
                              Vec3(b.x, gy(b) + lift - 0.4, b.y),
                              Vec3(b.x, gy(b) + lift, b.y),
                              Vec3(a.x, gy(a) + lift, a.y),
                              Vec3(n2.x, 0, n2.y), slab * 0.8);
    }
    const OBB2 ob = orientedBoundingBox(pad);
    const Vec2 ax = ob.axis[0], ay = ob.axis[1], ctr = ob.center;
    const Real hx = ob.half[0], hy = ob.half[1];
    auto emitBoxRM = [&](const Vec2& p, Real bw, Real bd, Real bh, const Vec3& col) {
        const Vec2 cs[4] = { p - ax * bw - ay * bd, p + ax * bw - ay * bd,
                             p + ax * bw + ay * bd, p - ax * bw + ay * bd };
        const Real g0 = gy(p), top = g0 + bh;
        MeshBuilder::emitQuad(m, Vec3(cs[0].x, top, cs[0].y), Vec3(cs[1].x, top, cs[1].y),
                              Vec3(cs[2].x, top, cs[2].y), Vec3(cs[3].x, top, cs[3].y),
                              Vec3(0, 1, 0), col);
        for (int k = 0; k < 4; ++k) {
            const Vec2& a = cs[k]; const Vec2& b = cs[(k + 1) % 4];
            Vec2 n2 = normalize(Vec2(b.y - a.y, a.x - b.x));
            MeshBuilder::emitQuad(m, Vec3(a.x, g0, a.y), Vec3(b.x, g0, b.y),
                                  Vec3(b.x, top, b.y), Vec3(a.x, top, a.y),
                                  Vec3(n2.x, 0, n2.y), col);
        }
    };
    if (utility) {
        const int nb = 2 + static_cast<int>(rng.unit() * 3);          // cabinets
        for (int i = 0; i < nb; ++i) {
            const Vec2 p = ctr + ax * rng.range(-hx * 0.6, hx * 0.6) +
                                 ay * rng.range(-hy * 0.6, hy * 0.6);
            emitBoxRM(p, rng.range(0.8, 1.6), rng.range(0.6, 1.2),
                      rng.range(1.4, 2.2), Vec3(0.42, 0.44, 0.46));
        }
        for (const Vec2& c : pad) emitBoxRM(c, 0.12, 0.12, 1.1, Vec3(0.30, 0.30, 0.32));
    } else {
        const Vec3 white(0.82, 0.82, 0.82);                           // parking stalls
        const Real stall = 2.7;
        const int rows = std::max(1, static_cast<int>(2 * hx / stall));
        for (int r = 0; r <= rows; ++r) {
            const Vec2 base = ctr - ax * hx + ax * (2 * hx * r / rows);
            const Vec2 a = base - ay * (hy * 0.8), b = base + ay * (hy * 0.8);
            const Vec2 w = ax * 0.12;
            const Real y0 = gy(base) + lift + 0.02;
            MeshBuilder::emitQuad(m, Vec3((a - w).x, y0, (a - w).y),
                                  Vec3((a + w).x, y0, (a + w).y),
                                  Vec3((b + w).x, y0, (b + w).y),
                                  Vec3((b - w).x, y0, (b - w).y), Vec3(0, 1, 0), white);
        }
    }
    g.padMesh = std::move(m);
    g.color = Vec3(1, 1, 1);
}

// YARD SCULPTING: a house lot earns a FRONT WALK from its door to the street
// boundary, a clipped hedge along the front lot line (with a gap where the
// walk crosses), and a back-yard tree spot or two.
void sculptYard(LotBuilding& b, const Poly2& lotPoly, const Poly2& house,
                const Vec2& face, Real walkTopY,
                const std::function<Real(Real, Real)>& ground, uint32_t seed,
                std::vector<RenderMesh>* outParts) {
    if (!outParts || house.size() < 3 || lotPoly.size() < 3 ||
        face.length() < 1e-6)
        return;
    Hash rng(mix(seed, 0xF00D5EEDu));
    auto gy = [&](const Vec2& v) {
        return ground ? ground(v.x, v.y) : Real(0);
    };
    const Vec2 f = normalize(face);
    // The DOOR: midpoint of the house edge whose outward normal best faces
    // the street (the same rule growPlanBuilding uses for the entrance).
    Real bestDot = -1e30;
    Vec2 door = centroid(house);
    for (std::size_t i = 0; i < house.size(); ++i) {
        const Vec2& a = house[i];
        const Vec2& e = house[(i + 1) % house.size()];
        Vec2 d = e - a;
        const Real len = d.length();
        if (len < 2.0) continue;
        Vec2 nrm(d.y / len, -d.x / len);
        const Real sc = dot(nrm, f);
        if (sc > bestDot) { bestDot = sc; door = (a + e) * 0.5; }
    }
    // Ray-exit from the door along the street direction to the lot boundary.
    Real tExit = -1;
    for (std::size_t i = 0; i < lotPoly.size(); ++i) {
        const Vec2& a = lotPoly[i];
        const Vec2& e = lotPoly[(i + 1) % lotPoly.size()];
        Vec2 d = e - a;
        const Real den = cross(f, d);
        if (std::fabs(den) < 1e-9) continue;
        const Real t = cross(a - door, d) / den;
        const Real u = cross(a - door, f) / den;
        if (t > 0.2 && u >= 0 && u <= 1) tExit = std::max(tExit, t);
    }
    Vec2 walkEnd = door + f * std::max(tExit, Real(0));
    if (tExit > 0.6) {
        // The front WALK: a draped pavement ribbon, door to lot line.
        RenderMesh& path = (*outParts)[static_cast<std::size_t>(PartId::Path)];
        Vec2 perp(-f.y, f.x);
        const Real hw = 0.55;
        const int segs = std::max(1, static_cast<int>(tExit / 2.5));
        for (int s = 0; s < segs; ++s) {
            Vec2 q0 = door + f * (tExit * s / segs);
            Vec2 q1 = door + f * (tExit * (s + 1) / segs);
            const Real y0 = ground ? gy(q0) + 0.06 : walkTopY + 0.06;
            const Real y1 = ground ? gy(q1) + 0.06 : walkTopY + 0.06;
            MeshBuilder::emitQuad(
                path, Vec3(q0.x - perp.x * hw, y0, q0.y - perp.y * hw),
                Vec3(q0.x + perp.x * hw, y0, q0.y + perp.y * hw),
                Vec3(q1.x + perp.x * hw, y1, q1.y + perp.y * hw),
                Vec3(q1.x - perp.x * hw, y1, q1.y - perp.y * hw),
                Vec3(0, 1, 0), Vec3(0.72, 0.70, 0.65));
        }
    }
    // The front HEDGE: clipped boxes along the street-facing lot edge, a gap
    // where the walk crosses. Roughly half the houses keep one.
    if (rng.unit() < 0.55) {
        BuildingMesh kit;
        const Vec3 up(0, 1, 0);
        Real bestEdge = -1e30;
        std::size_t fe = lotPoly.size();
        for (std::size_t i = 0; i < lotPoly.size(); ++i) {
            const Vec2& a = lotPoly[i];
            const Vec2& e = lotPoly[(i + 1) % lotPoly.size()];
            Vec2 d = e - a;
            const Real len = d.length();
            if (len < 6.0) continue;
            Vec2 nrm(d.y / len, -d.x / len);
            const Real sc = dot(nrm, f) + len * 0.005;
            if (sc > bestEdge) { bestEdge = sc; fe = i; }
        }
        if (fe < lotPoly.size() && bestEdge > 0.5) {
            const Vec2& a = lotPoly[fe];
            const Vec2& e = lotPoly[(fe + 1) % lotPoly.size()];
            Vec2 d = e - a;
            const Real len = d.length();
            Vec2 dir = d * (1.0 / len);
            Vec2 nrm(dir.y, -dir.x);              // outward (CCW poly)
            const Vec3 hedgeCol(0.24 + rng.range(0, 0.05), 0.42, 0.21);
            const int n = static_cast<int>((len - 1.6) / 2.0);
            for (int k = 0; k < n; ++k) {
                Vec2 hc = a + dir * (0.8 + 2.0 * k + 1.0) - nrm * 0.6;
                if (distToSeg(hc, door, walkEnd) < 1.4) continue;   // walk gap
                if (!pointInPolygon(lotPoly, hc)) continue;
                Vec3 t3(dir.x, 0, dir.y), n3(nrm.x, 0, nrm.y);
                emitBox(kit, Scope{Vec3(hc.x, gy(hc), hc.y) - t3 * 0.9 -
                                       n3 * 0.3,
                                   {t3, up, n3}, Vec3(1.8, 0.65, 0.6)},
                        PartId::Foliage, hedgeCol);
            }
        }
        appendKit(kit, outParts);
    }
    // BACK-YARD trees: a spot or two behind the house.
    const Vec2 hc = centroid(house);
    const int want = 1 + (rng.unit() < 0.5 ? 1 : 0);
    Poly2 inner = inset(lotPoly, 1.2);
    for (int k = 0; k < 8 && static_cast<int>(b.treeSpots.size()) < want; ++k) {
        const Real a2 = rng.unit() * 6.283185307179586;
        Vec2 tp = hc + Vec2(std::cos(a2), std::sin(a2)) *
                           rng.range(4.0, 9.0);
        if (dot(tp - hc, f) > 0) continue;               // not the front yard
        if (inner.size() < 3 || !pointInPolygon(inner, tp)) continue;
        if (pointInPolygon(house, tp)) continue;
        b.treeSpots.push_back(Vec3(tp.x, rng.range(0.8, 1.2), tp.y));
    }
}

// PLAZA SCULPTING (device: "concrete plazas, walking paths, decorative
// fencing and staircases between elevations ... like building a building
// structure without the building"): the fitted plan becomes a raised paver
// PODIUM at the lot's graded plane — the pad under it flattens exactly like a
// building's (the recipe is type "civic" so the park/green flatten skip does
// not apply) — dressed with STAIRS where its edges meet lower ground, guard
// FENCING over bigger drops, and a fountain / planters / benches / flower
// beds that each CLAIM their footprint on the flat deck.
void sculptPlaza(LotBuilding& b, const Poly2& planIn,
                 const std::function<Real(Real, Real)>& ground, uint32_t seed,
                 std::vector<RenderMesh>* outParts, const RoadGraph* roads) {
    if (!outParts || planIn.size() < 3) return;
    Poly2 plan = planIn;
    ensureCCW(plan);   // area() is |signedArea| — the old `area()<0` guard was dead
    Hash rng(mix(seed, 0x9A7A5EEDu));
    auto gy = [&](const Vec2& v) { return ground ? ground(v.x, v.y) : Real(0); };
    const Real slabY = b.groundY + 0.35;          // podium reveal over the pad
    const Real A = area(plan);
    const Vec2 c = centroid(plan);
    const Vec3 up(0, 1, 0);
    const Vec3 white(1, 1, 1);                    // surfaces carry the look
    BuildingMesh kit;

    // The paver DECK: one flat plate (the pad below is graded to b.groundY,
    // so the reveal is constant); Pavement surface via PartId::Path.
    {
        RenderMesh deck;
        for (const std::array<int, 3>& t : triangulatePolygon(plan))
            MeshBuilder::emitTri(deck,
                                 Vec3(plan[t[0]].x, slabY, plan[t[0]].y),
                                 Vec3(plan[t[1]].x, slabY, plan[t[1]].y),
                                 Vec3(plan[t[2]].x, slabY, plan[t[2]].y),
                                 up, white);
        MeshBuilder::append((*outParts)[static_cast<std::size_t>(PartId::Path)],
                            deck);
    }
    // Concrete SKIRT: deck edge down past the surrounding ground, so a
    // downhill boundary never opens a gap under the slab.
    {
        RenderMesh skirt;
        for (std::size_t i = 0; i < plan.size(); ++i) {
            const Vec2& a = plan[i];
            const Vec2& e = plan[(i + 1) % plan.size()];
            Vec2 d = e - a;
            if (d.length() < 1e-6) continue;
            Vec2 n2 = normalize(Vec2(d.y, -d.x));   // CCW: right normal = outward
            MeshBuilder::emitQuad(skirt,
                                  Vec3(a.x, gy(a) - 0.5, a.y),
                                  Vec3(e.x, gy(e) - 0.5, e.y),
                                  Vec3(e.x, slabY, e.y),
                                  Vec3(a.x, slabY, a.y),
                                  Vec3(n2.x, 0, n2.y), white);
        }
        MeshBuilder::append(
            (*outParts)[static_cast<std::size_t>(PartId::Concrete)], skirt);
    }

    // STAIRS at the plaza's mouths: the longest edges whose outside ground
    // sits a walkable drop below the deck get a full stair run — the
    // "staircases between elevations". Each mouth also opens the fence, and
    // each records its FOOT (where a walk to the street starts).
    struct Mouth { Vec2 p; Real halfW; Vec2 foot; };
    std::vector<Mouth> mouths;
    {
        std::vector<std::pair<Real, std::size_t>> ranked;
        for (std::size_t i = 0; i < plan.size(); ++i)
            ranked.emplace_back(
                (plan[(i + 1) % plan.size()] - plan[i]).length(), i);
        std::sort(ranked.begin(), ranked.end(),
                  [](const auto& x, const auto& y) { return x.first > y.first; });
        for (const auto& [len, i] : ranked) {
            if (mouths.size() >= 3 || len < 6.0) break;
            const Vec2& a = plan[i];
            const Vec2& e = plan[(i + 1) % plan.size()];
            Vec2 dir = (e - a) * (1.0 / len);
            Vec2 nOut(dir.y, -dir.x);
            Vec2 m = (a + e) * 0.5;
            const Real go = gy(m + nOut * 2.2);
            const Real drop = slabY - go;
            if (drop > 2.6) continue;
            if (drop < 0.18) {
                // FLUSH mouth: level with the outside ground — no stairs
                // needed, but it still opens the fence and takes a walk.
                mouths.push_back({m, std::min(len * 0.45, Real(6.0)) * 0.5,
                                  m + nOut * 0.5});
                continue;
            }
            const Real w = std::min(len * 0.45, Real(6.0));
            const int steps = std::max(1, static_cast<int>(std::ceil(drop / 0.17)));
            const Real tread = 0.34;
            Vec3 u3(dir.x, 0, dir.y), n3(nOut.x, 0, nOut.y);
            for (int s = 1; s <= steps; ++s) {
                const Real topY = slabY - s * (drop / steps);
                const Real botY = go - 0.4;
                if (topY <= botY) break;
                emitBox(kit,
                        Scope{Vec3(m.x, botY, m.y) - u3 * (w * 0.5) +
                                  n3 * ((s - 1) * tread),
                              {u3, up, n3}, Vec3(w, topY - botY, tread)},
                        PartId::Path, white);   // paver steps: walk surface
            }
            mouths.push_back({m, w * 0.5, m + nOut * (steps * tread + 0.3)});
        }
    }

    // WALKS to the street (P6.3, device: "walking paths"): from each mouth's
    // foot to the nearest carriageway edge — a paver ribbon that follows the
    // terrain with side skirts (the same construction the park spokes use),
    // so the plaza is HOOKED to its sidewalks instead of floating in a lot.
    if (roads) {
        RenderMesh walk;
        for (const Mouth& mo : mouths) {
            Real best = 1e30;
            Vec2 curb{};
            for (const RoadEdge& e : roads->edges) {
                if (e.a < 0 || e.b < 0 ||
                    e.a >= static_cast<int>(roads->nodes.size()) ||
                    e.b >= static_cast<int>(roads->nodes.size())) continue;
                const Vec2& ra = roads->nodes[e.a].pos;
                const Vec2& rb = roads->nodes[e.b].pos;
                Vec2 ab = rb - ra;
                Real len2 = ab.lengthSquared();
                Real t = len2 > 1e-12 ? dot(mo.foot - ra, ab) / len2 : 0.0;
                t = t < 0 ? 0 : (t > 1 ? 1 : t);
                Vec2 q(ra.x + ab.x * t, ra.y + ab.y * t);
                const Real d = (q - mo.foot).length();
                if (d >= best || d < 1e-6) continue;
                best = d;
                // stop the walk at the carriageway edge, on the mouth's side
                curb = q + (mo.foot - q) * ((e.width * 0.5 + 0.4) / d);
            }
            const Real len = (curb - mo.foot).length();
            if (best > 1e29 || len < 1.2 || len > 28.0) continue;
            Vec2 dir = (curb - mo.foot) * (1.0 / len);
            Vec2 perp(-dir.y, dir.x);
            const Real hw = 1.0;
            const int segs = std::max(1, static_cast<int>(len / 3.0));
            for (int s = 0; s < segs; ++s) {
                Vec2 q0 = mo.foot + dir * (len * s / segs);
                Vec2 q1 = mo.foot + dir * (len * (s + 1) / segs);
                const Real e0 = 0.08, e1 = 0.08;
                const Vec3 L0(q0.x - perp.x * hw, gy(q0 - perp * hw) + e0, q0.y - perp.y * hw);
                const Vec3 R0(q0.x + perp.x * hw, gy(q0 + perp * hw) + e0, q0.y + perp.y * hw);
                const Vec3 L1(q1.x - perp.x * hw, gy(q1 - perp * hw) + e1, q1.y - perp.y * hw);
                const Vec3 R1(q1.x + perp.x * hw, gy(q1 + perp * hw) + e1, q1.y + perp.y * hw);
                MeshBuilder::emitQuad(walk, L0, R0, R1, L1, up, white);
                const Vec3 drop3(0, -0.45, 0);
                const Vec3 pL(-perp.x, 0, -perp.y), pR(perp.x, 0, perp.y);
                MeshBuilder::emitQuad(walk, L0 + drop3, L1 + drop3, L1, L0, pL, white);
                MeshBuilder::emitQuad(walk, R0 + drop3, R1 + drop3, R1, R0, pR, white);
            }
        }
        MeshBuilder::append((*outParts)[static_cast<std::size_t>(PartId::Path)],
                            walk);
    }

    // Decorative GUARD FENCE: iron posts + one rail along deck edges that
    // stand above the outside ground, opened at every stair mouth.
    {
        const Vec3 iron(0.13, 0.13, 0.15);
        for (std::size_t i = 0; i < plan.size(); ++i) {
            const Vec2& a = plan[i];
            const Vec2& e = plan[(i + 1) % plan.size()];
            const Real len = (e - a).length();
            if (len < 1.6) continue;
            Vec2 dir = (e - a) * (1.0 / len);
            Vec2 nOut(dir.y, -dir.x);
            Vec3 u3(dir.x, 0, dir.y), n3(nOut.x, 0, nOut.y);
            const int posts = std::max(1, static_cast<int>(len / 2.2));
            const Real step = len / posts;
            auto keep = [&](const Vec2& pp) {
                for (const Mouth& mo : mouths)
                    if ((mo.p - pp).length() < mo.halfW + 0.9) return false;
                // guard only where there is something to guard against
                return slabY - gy(pp + nOut * 1.4) > 0.55;
            };
            for (int pi2 = 0; pi2 <= posts; ++pi2) {
                Vec2 pp = a + dir * (step * pi2) - nOut * 0.22;   // inboard of edge
                if (!keep(pp + nOut * 0.22)) continue;
                emitBox(kit,
                        Scope{Vec3(pp.x, slabY, pp.y) - u3 * 0.05 - n3 * 0.05,
                              {u3, up, n3}, Vec3(0.10, 0.95, 0.10)},
                        PartId::Metal, iron);
                if (pi2 == posts) continue;
                Vec2 np = a + dir * (step * (pi2 + 1)) - nOut * 0.22;
                if (!keep(np + nOut * 0.22)) continue;
                emitBox(kit,
                        Scope{Vec3(pp.x, slabY + 0.82, pp.y) - n3 * 0.03,
                              {u3, up, n3}, Vec3(step, 0.07, 0.06)},
                        PartId::Metal, iron);
            }
        }
    }

    // FURNITURE on the flat deck — claim-registry spacing, same discipline as
    // the parks (device: "spaced out and placed correctly").
    std::vector<std::pair<Vec2, Real>> claimed;
    auto clearAt = [&](const Vec2& p2, Real r) {
        for (const auto& [q, qr] : claimed)
            if ((q - p2).length() < r + qr) return false;
        return true;
    };
    auto claim = [&](const Vec2& p2, Real r) { claimed.emplace_back(p2, r); };
    Poly2 innerPoly = inset(plan, 1.2);
    auto onDeck = [&](const Vec2& p2) {
        return innerPoly.size() >= 3 && pointInPolygon(innerPoly, p2);
    };
    const Vec3 stone(0.72, 0.70, 0.66);

    // The FOUNTAIN: centrepiece of a roomy plaza.
    Real r0 = 0;
    if (A > 380 && onDeck(c)) {
        r0 = std::min(Real(4.5), std::max(Real(2.2), std::sqrt(A) * 0.11));
        std::vector<Vec2> basin = {
            {r0 * 0.42, 0.0},  {r0 * 0.44, 0.42}, {r0 * 0.36, 0.50},
            {r0 * 0.34, 0.14}, {0.14, 0.14},      {0.11, 1.05},
            {0.30, 1.18},      {0.24, 1.32},      {0.0, 1.40}};
        MeshBuilder::append((*outParts)[static_cast<std::size_t>(PartId::Trim)],
                            latheMesh(Vec3(c.x, slabY, c.y), basin, 16, stone));
        std::vector<Vec2> water = {{r0 * 0.33, 0.36}, {0.0, 0.36}};
        MeshBuilder::append((*outParts)[static_cast<std::size_t>(PartId::Glass)],
                            latheMesh(Vec3(c.x, slabY, c.y), water, 16,
                                      Vec3(0.20, 0.34, 0.40)));
        claim(c, r0 * 0.5 + 0.4);
    }

    // Plaza LAMPS: the street-kit lamp, standing on the deck.
    {
        const RenderMesh lampProto = streetLamp();
        const int nl = A > 500 ? 4 : 2;
        for (int k = 0, placed = 0; k < nl * 4 && placed < nl; ++k) {
            const Real a2 = rng.unit() * 6.283185307179586;
            const Real rr = (r0 > 0 ? r0 + 2.5 : 3.0) +
                            rng.unit() * std::sqrt(A) * 0.22;
            Vec2 lp = c + Vec2(std::cos(a2), std::sin(a2)) * rr;
            if (!onDeck(lp) || !clearAt(lp, 0.6)) continue;
            claim(lp, 0.6);
            ++placed;
            RenderMesh lamp = lampProto;
            MeshBuilder::transform(lamp, Mat4::translate(lp.x, slabY, lp.y));
            MeshBuilder::append(
                (*outParts)[static_cast<std::size_t>(PartId::Metal)], lamp);
        }
    }

    // BENCHES facing the centre.
    {
        const Vec3 wood(0.45, 0.34, 0.22);
        const int nb = 3 + static_cast<int>(rng.unit() * 3);
        for (int k = 0; k < nb * 2; ++k) {
            const Real a2 = rng.unit() * 6.283185307179586;
            Vec2 dir(std::cos(a2), std::sin(a2));
            Vec2 bp = c + dir * ((r0 > 0 ? r0 + 1.1 : 2.8) + rng.range(0, 1.5));
            if (!onDeck(bp) || !clearAt(bp, 1.0)) continue;
            claim(bp, 1.0);
            Vec2 t2(-dir.y, dir.x);
            Vec3 t3(t2.x, 0, t2.y), n3(dir.x, 0, dir.y);
            Vec3 o = Vec3(bp.x, slabY + 0.42, bp.y) - t3 * 0.8 - n3 * 0.22;
            emitBox(kit, Scope{o, {t3, up, n3}, Vec3(1.6, 0.07, 0.44)},
                    PartId::Wood, wood);
            for (int lg = 0; lg < 2; ++lg)
                emitBox(kit,
                        Scope{Vec3(bp.x, slabY, bp.y) +
                                  t3 * (lg ? 0.55 : -0.65) - n3 * 0.18,
                              {t3, up, n3}, Vec3(0.10, 0.42, 0.36)},
                        PartId::Metal, Vec3(0.20, 0.21, 0.22));
        }
    }

    // Stone PLANTERS with trees — the plaza's canopy, rooted in boxes.
    {
        const int np = std::min(6, std::max(2, static_cast<int>(A / 220)));
        Real mnx = 1e30, mnz = 1e30, mxx = -1e30, mxz = -1e30;
        for (const Vec2& v : plan) {
            mnx = std::min(mnx, v.x); mxx = std::max(mxx, v.x);
            mnz = std::min(mnz, v.y); mxz = std::max(mxz, v.y);
        }
        for (int k = 0, placed = 0; k < np * 5 && placed < np; ++k) {
            Vec2 sp(mnx + rng.unit() * (mxx - mnx),
                    mnz + rng.unit() * (mxz - mnz));
            if (!onDeck(sp) || !clearAt(sp, 1.7)) continue;
            claim(sp, 1.7);
            ++placed;
            const Real a2 = rng.unit() * 6.283185307179586;
            Vec3 t3(std::cos(a2), 0, std::sin(a2)), n3(-t3.z, 0, t3.x);
            emitBox(kit,
                    Scope{Vec3(sp.x, slabY, sp.y) - t3 * 0.85 - n3 * 0.85,
                          {t3, up, n3}, Vec3(1.7, 0.55, 1.7)},
                    PartId::Trim, stone * 0.9);
            b.treeSpots.push_back(Vec3(sp.x, rng.range(0.65, 0.9), sp.y));
        }
    }

    // FLOWER BEDS: a bright ring accent between the benches.
    {
        const Vec3 bloom[3] = {{0.72, 0.22, 0.26},
                               {0.82, 0.68, 0.20},
                               {0.56, 0.32, 0.66}};
        const int nf = 2 + static_cast<int>(rng.unit() * 2);
        for (int k = 0; k < nf * 3; ++k) {
            const Real a2 = rng.unit() * 6.283185307179586;
            Vec2 dir(std::cos(a2), std::sin(a2));
            Vec2 fp = c + dir * ((r0 > 0 ? r0 + 1.6 : 3.2) + rng.range(0, 1.0));
            if (!onDeck(fp) || !clearAt(fp, 0.8)) continue;
            claim(fp, 0.8);
            Vec3 t3(-dir.y, 0, dir.x), n3(dir.x, 0, dir.y);
            Vec3 o = Vec3(fp.x, slabY, fp.y) - t3 * 0.55 - n3 * 0.55;
            emitBox(kit, Scope{o, {t3, up, n3}, Vec3(1.1, 0.22, 1.1)},
                    PartId::Trim, stone * 0.92);
            emitBox(kit,
                    Scope{o + Vec3(0, 0.22, 0) + t3 * 0.12 + n3 * 0.12,
                          {t3, up, n3}, Vec3(0.86, 0.14, 0.86)},
                    PartId::Foliage,
                    bloom[static_cast<int>(rng.unit() * 2.99)]);
        }
    }

    appendKit(kit, outParts);
    b.color = Vec3(1, 1, 1);   // the surfaces carry the plaza's look
}
}  // namespace

std::vector<LotBuilding> growLotBuildings(const std::vector<Poly2>& blocks,
                                          const LotParams& pIn, LotPlanDebug* debug,
                                          std::vector<RenderMesh>* outParts,
                                          const RoadGraph* roads, Real roadClearance,
                                          std::vector<RenderMesh>* outFlatParts,
                                          std::vector<TerrainFlatten>* outGrade) {
    // Mutable copy: after PASS A the ground sampler is wrapped with the block
    // grades (see the grading step below), so PASS B/C grow on terraced ground.
    LotParams p = pIn;
    std::vector<LotBuilding> out;
    // The plan counters always run (the build-end density line below reads
    // them); `debug` just decides whether the caller sees them too.
    LotPlanDebug localDbg;
    LotPlanDebug* dbg = debug ? debug : &localDbg;
    // The ARCHITECT's district map (P5): radial rings + seeded old-town and
    // industrial quarters. Every lot asks the architect what belongs here.
    DistrictMap districts;
    districts.center = p.center;
    districts.innerRadius = p.innerRadius;
    districts.midRadius = p.midRadius;
    districts.hubs = p.hubs;
    districts.hubRadius = p.hubRadius;
    districts.seed = p.seed;
    // Stage-10 ALLEYS live in their own small graph BESIDE the (const) sampled
    // graph: the frontage gate counts them as street surface, and the clearance
    // passes keep buildings just off the pavement. An Alley-class edge takes a
    // small FIXED clearance — a garage may abut a service lane; a tower may
    // not abut an arterial — so the lane doesn't sterilize the rows it exists
    // to serve.
    RoadGraph alleyGraph;
    // Abutting: a service lane wants buildings AT its edge (garage doors open
    // onto an alley). The lane is centred on the rows' SHARED lot boundary and
    // buildings already stand lotSetback (1.4 m) off that line — so a 2.8 m
    // pavement with epsilon clearance costs the existing rows NOTHING, which
    // is the whole trick: 0.7 m of clearance measurably shrank the inner rows
    // and COST coverage (living_city 48.6% -> 47.3%); 0.35 m broke even.
    constexpr Real kAlleyClear = 0.05;
    // Road-clearance corner test (device: buildings poking onto the street): a
    // building box corner must stay `edge width/2 + roadClearance` from every
    // road centreline. Checked against the SAMPLED graph the asphalt is meshed
    // from, so a curvy road that bows into a straight-edged block still pushes
    // the building back. Folded into scopeFromFootprint's shrink-to-fit below.
    auto clearOfRoads = [&](const Vec2& c) {
        if (!roads) return true;
        const RoadGraph* gs[2] = {roads, &alleyGraph};
        for (int gi = 0; gi < 2; ++gi) {
            const RoadGraph& g = *gs[gi];
            const Real clear = gi == 0 ? roadClearance : kAlleyClear;
            for (const RoadEdge& e : g.edges) {
                if (e.a < 0 || e.b < 0 || e.a >= static_cast<int>(g.nodes.size()) ||
                    e.b >= static_cast<int>(g.nodes.size())) continue;
                const Vec2& a = g.nodes[e.a].pos;
                const Vec2& b = g.nodes[e.b].pos;
                Vec2 ab = b - a;
                Real len2 = ab.lengthSquared();
                Real t = len2 > 1e-12 ? dot(c - a, ab) / len2 : 0.0;
                t = t < 0 ? 0 : (t > 1 ? 1 : t);
                Vec2 q(a.x + ab.x * t, a.y + ab.y * t);
                if ((c - q).length() < e.width * 0.5 + clear) return false;
            }
        }
        return true;
    };
    // Distance from a point to the nearest road SURFACE edge (centreline
    // distance minus half-width, floored at 0). Frontage gate: a lot whose
    // whole footprint sits farther than the frontage bound from every road
    // goes GREEN instead of building — stage 4 of the city contract ("lots
    // touch the road; the middle of a block is parks and plazas").
    auto roadSurfaceDist = [&](const Vec2& c) {
        if (!roads) return Real(0);
        Real best = Real(1e30);
        const RoadGraph* gs[2] = {roads, &alleyGraph};
        for (const RoadGraph* gp : gs) {
            const RoadGraph& g = *gp;
            for (const RoadEdge& e : g.edges) {
                if (e.a < 0 || e.b < 0 || e.a >= static_cast<int>(g.nodes.size()) ||
                    e.b >= static_cast<int>(g.nodes.size())) continue;
                const Vec2& a = g.nodes[e.a].pos;
                const Vec2& b = g.nodes[e.b].pos;
                Vec2 ab = b - a;
                Real len2 = ab.lengthSquared();
                Real t = len2 > 1e-12 ? dot(c - a, ab) / len2 : 0.0;
                t = t < 0 ? 0 : (t > 1 ? 1 : t);
                Vec2 q(a.x + ab.x * t, a.y + ab.y * t);
                Real d = (c - q).length() - e.width * 0.5;
                best = std::min(best, d < 0 ? Real(0) : d);
            }
        }
        return best;
    };
    // PUSH a polygon clear of the sampled road ribbons (roads-v2.1 R4-6c,
    // drive feedback: "parks don't sit on their lots and explode into the
    // street including fences"). Buildings shrink-to-fit via clearOfRoads;
    // parks/greens used their raw footprint and a uniform inset that assumed
    // 4 m half-width roads — arterials and curved streets overran them, so
    // fences and trees stood in the carriageway. Edges are subdivided first
    // so a road bowing into a LONG park edge still gets pushed.
    auto pushPolyClearOfRoads = [&](Poly2 poly) {
        if (!roads || poly.size() < 3) return poly;
        Poly2 dense;
        for (std::size_t i = 0; i < poly.size(); ++i) {
            const Vec2& a2 = poly[i];
            const Vec2& b2 = poly[(i + 1) % poly.size()];
            dense.push_back(a2);
            const Real len = (b2 - a2).length();
            const int div = static_cast<int>(len / 8.0);
            for (int k = 1; k <= div; ++k)
                dense.push_back(a2 + (b2 - a2) * (static_cast<Real>(k) / (div + 1)));
        }
        for (Vec2& v : dense) {
            for (int guard = 0; guard < 4; ++guard) {
                Real worst = 0;
                Vec2 away(0, 0);
                const RoadGraph* gs[2] = {roads, &alleyGraph};
                for (int gi = 0; gi < 2; ++gi) {
                    const RoadGraph& g = *gs[gi];
                    const Real clear = gi == 0 ? roadClearance : kAlleyClear;
                    for (const RoadEdge& e : g.edges) {
                        if (e.a < 0 || e.b < 0 ||
                            e.a >= static_cast<int>(g.nodes.size()) ||
                            e.b >= static_cast<int>(g.nodes.size()))
                            continue;
                        const Vec2& a2 = g.nodes[e.a].pos;
                        const Vec2& b2 = g.nodes[e.b].pos;
                        Vec2 ab = b2 - a2;
                        Real len2 = ab.lengthSquared();
                        Real t = len2 > 1e-12 ? dot(v - a2, ab) / len2 : 0.0;
                        t = t < 0 ? 0 : (t > 1 ? 1 : t);
                        const Vec2 q(a2.x + ab.x * t, a2.y + ab.y * t);
                        const Real need = e.width * 0.5 + clear;
                        const Real d = (v - q).length();
                        if (need - d > worst) {
                            worst = need - d;
                            away = d > 1e-6 ? (v - q) * (1.0 / d) : Vec2(1, 0);
                        }
                    }
                }
                if (worst <= 0) break;
                v = v + away * (worst + 0.2);
            }
        }
        // INFEASIBLE lot: vertices that still violate after the pushes sit in
        // a sliver no park can occupy (a leftover strip between two
        // carriageways). Such ground is street verge, not a lot — return
        // EMPTY so the caller skips the park/green entirely rather than
        // furnishing the roadway.
        int bad = 0;
        std::vector<char> badAt(dense.size(), 0);
        for (std::size_t vi = 0; vi < dense.size(); ++vi) {
            const Vec2& v = dense[vi];
            const RoadGraph* gs[2] = {roads, &alleyGraph};
            for (const RoadGraph* gp : gs) {
                const RoadGraph& g = *gp;
                for (const RoadEdge& e : g.edges) {
                    if (e.a < 0 || e.b < 0 ||
                        e.a >= static_cast<int>(g.nodes.size()) ||
                        e.b >= static_cast<int>(g.nodes.size()))
                        continue;
                    const Vec2& a2 = g.nodes[e.a].pos;
                    const Vec2& b2 = g.nodes[e.b].pos;
                    Vec2 ab = b2 - a2;
                    Real len2 = ab.lengthSquared();
                    Real t = len2 > 1e-12 ? dot(v - a2, ab) / len2 : 0.0;
                    t = t < 0 ? 0 : (t > 1 ? 1 : t);
                    if ((v - (a2 + ab * t)).length() < e.width * 0.5 + 0.3) {
                        ++bad;
                        badAt[vi] = 1;
                        break;
                    }
                }
                if (badAt[vi]) break;
            }
        }
        if (bad * 5 > static_cast<int>(dense.size()))   // > 20% hopeless
            return Poly2{};
        if (bad > 0) {
            // A few stragglers (opposing pushes couldn't settle them): the
            // polygon simply loses those corners — a slightly smaller park
            // beats a fence post in the carriageway.
            Poly2 kept;
            for (std::size_t vi = 0; vi < dense.size(); ++vi)
                if (!badAt[vi]) kept.push_back(dense[vi]);
            if (kept.size() < 3) return Poly2{};
            return kept;
        }
        return dense;
    };
    // Under a FREEWAY/RAMP deck? The clearance graph carries the freeway
    // right-of-way as RoadClass::Freeway / ::Ramp edges whose width is the deck
    // SHADOW (set by the loader). A point within that shadow can't host a normal
    // building (it would clip the elevated structure) — the lot pass turns it
    // into deck-fitting open space instead (see PASS C).
    auto underFreeway = [&](const Vec2& c, bool* nearestIsRamp) {
        if (!roads) return false;
        bool under = false; Real best = 1e30;
        for (const RoadEdge& e : roads->edges) {
            if (e.klass != RoadClass::Freeway && e.klass != RoadClass::Ramp) continue;
            if (e.a < 0 || e.b < 0 || e.a >= static_cast<int>(roads->nodes.size()) ||
                e.b >= static_cast<int>(roads->nodes.size())) continue;
            const Vec2& a = roads->nodes[e.a].pos;
            const Vec2& b = roads->nodes[e.b].pos;
            Vec2 ab = b - a;
            Real len2 = ab.lengthSquared();
            Real t = len2 > 1e-12 ? dot(c - a, ab) / len2 : 0.0;
            t = t < 0 ? 0 : (t > 1 ? 1 : t);
            Vec2 q(a.x + ab.x * t, a.y + ab.y * t);
            const Real d = (c - q).length();
            if (d < e.width * 0.5) {                             // within the deck shadow
                under = true;
                if (d < best) { best = d; if (nearestIsRamp) *nearestIsRamp = e.klass == RoadClass::Ramp; }
            }
        }
        return under;
    };
    int underFwCount = 0;
    // TERRAIN base for a plan: the LOWEST ground under its vertices so the
    // downhill corner never floats, embedded slightly on real slopes so the
    // uphill side beds in instead of hovering behind a knife-edge gap.
    auto baseYFor = [&](const Poly2& pl) -> Real {
        if (!p.ground || pl.empty()) return 0;
        Real lo = 1e30, hi = -1e30;
        for (const Vec2& v : pl) {
            const Real g = p.ground(v.x, v.y);
            lo = std::min(lo, g);
            hi = std::max(hi, g);
        }
        return lo - ((hi - lo) > 0.05 ? Real(0.25) : Real(0));
    };
    // TERRAIN pad plane (device: "the terrain should be flat under the
    // building" + "the entrance should be as level as possible with the
    // sidewalk it's next to"): the grade at the ENTRANCE side — cast a ray
    // from the plan centroid along the face direction to the boundary, step a
    // couple of metres toward the street (onto the road-conformed apron, i.e.
    // the sidewalk's own grade), and sample there. The host stamps a flatten
    // pad at this plane, so the walls meet FLAT graded earth and the front
    // door meets the sidewalk. Falls back to the vertex average when the ray
    // finds no boundary (degenerate plans).
    auto padPlaneFor = [&](const Poly2& pl, const Vec2& face) -> Real {
        if (!p.ground || pl.empty()) return 0;
        const Vec2 c = centroid(pl);
        Real tExit = -1;
        if (face.length() > Real(1e-6)) {
            const Vec2 f = normalize(face);
            const std::size_t n = pl.size();
            for (std::size_t i = 0; i < n; ++i) {
                const Vec2& a = pl[i];
                const Vec2& b = pl[(i + 1) % n];
                const Vec2 e = b - a;
                const Real den = cross(f, e);
                if (std::fabs(den) < Real(1e-9)) continue;
                const Real t = cross(a - c, e) / den;       // along the ray
                const Real u = cross(a - c, f) / den;       // along the edge
                if (t > 0 && u >= 0 && u <= 1) tExit = std::max(tExit, t);
            }
            if (tExit > 0) {
                const Vec2 E = c + f * (tExit + Real(2.0));
                return p.ground(E.x, E.y);
            }
        }
        Real sum = 0;
        for (const Vec2& v : pl) sum += p.ground(v.x, v.y);
        return sum / static_cast<Real>(pl.size());
    };
    // Plinth reveal: walls start this far above the graded pad, on a visible
    // FOUNDATION course (device: "there should be some kind of a base for the
    // building and steps to get up to the front door"). Host-tunable.
    const Real plinth = p.ground ? std::max(Real(0), p.plinth) : Real(0);
    // The foundation course: the plan outset slightly, extruded from below the
    // pad up to the wall base — a concrete band that grounds the massing and
    // hides the terrain seam. Emitted into the Concrete part like any other
    // grammar element.
    auto emitFoundation = [&](const Poly2& plIn, Real planeY, Real topY) {
        if (!outParts || plIn.size() < 3 || !p.ground) return;
        Poly2 pl = plIn;
        // ensureCCW, NOT `if (area(pl) < 0) reverse`: area() is |signedArea|,
        // so that guard can never fire — and roughly half the plan sources
        // (box fallback, rowhouse strips, courtyard carves) arrive CW. A CW
        // plan INSETS the ring under the walls with inward normals, which the
        // backface-culling viewer draws as a see-through hole (Glenn: "the
        // skirt is very broken looking with inverse normal faces").
        ensureCCW(pl);
        const std::size_t nv = pl.size();
        Poly2 o(nv);
        for (std::size_t i = 0; i < nv; ++i) {
            const Vec2& a = pl[(i + nv - 1) % nv];
            const Vec2& b = pl[i];
            const Vec2& c = pl[(i + 1) % nv];
            Vec2 e0 = b - a, e1 = c - b;
            auto rn = [](Vec2 e) {
                Vec2 n(e.y, -e.x); Real l = n.length();
                return l < Real(1e-9) ? Vec2(0, 0) : n * (1 / l);
            };
            Vec2 n0 = rn(e0), n1 = rn(e1);
            Vec2 bis = n0 + n1; Real bl = bis.length();
            Vec2 mm = bl < Real(1e-9) ? n1 : bis * (1 / bl);
            const Real cosH = std::max(Real(0.35), dot(mm, n1));
            o[i] = b + mm * (Real(0.14) / cosH);
        }
        RenderMesh& m = (*outParts)[static_cast<std::size_t>(PartId::Concrete)];
        const Vec3 col(1, 1, 1);   // Concrete's surface maps carry the look
        // The skirt bottom DRAPES: each ring vertex drops to the terrain
        // sampled under it, minus a bed-in margin — a fixed planeY-0.6 left
        // the ring hanging in air wherever the downhill side fell more than
        // 0.6 m below the pad (the falloff apron does exactly that on hills).
        std::vector<Real> bot(nv);
        for (std::size_t i = 0; i < nv; ++i) {
            const Real g = p.ground(o[i].x, o[i].y);
            bot[i] = std::min(planeY, g) - Real(0.4);
        }
        for (std::size_t i = 0; i < nv; ++i) {
            const std::size_t j = (i + 1) % nv;
            Vec2 e = o[j] - o[i];
            if (e.length() < Real(1e-9)) continue;
            Vec2 n = normalize(Vec2(e.y, -e.x));
            MeshBuilder::emitQuad(m, Vec3(o[i].x, bot[i], o[i].y),
                                  Vec3(o[j].x, bot[j], o[j].y),
                                  Vec3(o[j].x, topY, o[j].y),
                                  Vec3(o[i].x, topY, o[i].y),
                                  Vec3(n.x, 0, n.y), col);
            // The exposed ledge between the foundation's outer lip and the wall.
            MeshBuilder::emitQuad(m, Vec3(o[i].x, topY, o[i].y),
                                  Vec3(o[j].x, topY, o[j].y),
                                  Vec3(pl[j].x, topY, pl[j].y),
                                  Vec3(pl[i].x, topY, pl[i].y),
                                  Vec3(0, 1, 0), col);
        }
    };
    if (outParts) {
        outParts->assign(static_cast<std::size_t>(PartId::Count), RenderMesh{});
        for (std::size_t i = 0; i < outParts->size(); ++i)
            (*outParts)[i].materialIndex = static_cast<int>(i);
    }
    if (outFlatParts) {
        outFlatParts->assign(static_cast<std::size_t>(PartId::Count), RenderMesh{});
        for (std::size_t i = 0; i < outFlatParts->size(); ++i)
            (*outFlatParts)[i].materialIndex = static_cast<int>(i);
    }
    // Merge one grown building's parts into a PartId-indexed set (the same
    // fold the LOD0 path does inline below — shared so the flat set cannot
    // diverge on indexing).
    auto mergeParts = [](std::vector<RenderMesh>* dst, const BuildingMesh& bm) {
        if (!dst) return;
        for (const RenderMesh& part : bm.parts) {
            const int mi = part.materialIndex;
            if (mi >= 0 && mi < static_cast<int>(dst->size()))
                MeshBuilder::append((*dst)[mi], part);
        }
    };
    // Block footprints kept for the in-pass grading step below (dbg->blocks
    // is the debug overlay's copy; grading must not depend on debug wiring).
    std::vector<Poly2> blockFoots;

    // ---- PASS A: parcel every block and COLLECT the viable lots ------------
    // The landmark planner (pass B) needs to see the whole city before any lot
    // builds — a courthouse goes on the BEST financial lot, not the first one.
    struct BlockInfo {
        ParcelParams pp;
        DistrictTag tag;
        Real lotSetback, buildChance;
        Poly2 foot;   // the parcelled interior (the alley pass clips to it)
    };
    struct LotCand {
        std::size_t li;      // lot index within its block (the rng stream id)
        int block;           // index into binfos
        Lot lot;
        int landmark = -1;   // LandmarkKind once the planner assigns one
    };
    std::vector<BlockInfo> binfos;
    std::vector<LotCand> cands;
    // Level-authored parcel grain (citysim.parcel, 8km-city P3): the override
    // RESCALES the district tuning below relative to the stock defaults —
    // absent (<= 0) every factor is 1 and the tuning is exactly today's.
    const ParcelParams stockPP;
    const Real ppAreaScale = p.parcelTargetArea > 0
        ? p.parcelTargetArea / stockPP.targetArea : Real(1);
    const Real ppFrontScale = p.parcelFrontWidth > 0
        ? p.parcelFrontWidth / stockPP.frontWidth : Real(1);
    const Real ppDepthScale = p.parcelLotDepth > 0
        ? p.parcelLotDepth / stockPP.lotDepth : Real(1);
    const Real ppMinArea = p.parcelMinArea > 0 ? p.parcelMinArea : p.minLotArea;
    int blocksAllCarriageway = 0;
    for (std::size_t bi = 0; bi < blocks.size(); ++bi) {
        const Poly2& block = blocks[bi];
        if (block.size() < 3) continue;
        // Pull in from the road edge to the buildable interior (road + sidewalk).
        Poly2 foot = inset(block, p.roadMargin);
        if (foot.size() < 3) continue;
        // ...then push clear of the SAMPLED ribbons, because that inset alone is
        // measured against the wrong road twice over:
        //   - p.roadMargin is ONE scalar, derived from a nominal road half-width
        //     (level_params.cpp), so it is short for anything wider than that;
        //   - the faces were walked over the CONTROL CHORDS (extractBlocks needs a
        //     planar graph), while the asphalt is meshed from the sampled spline,
        //     and a curvy road bows off its chord — see growLotBuildingsOnNets.
        // Where those bite, the "buildable" interior reaches into the carriageway
        // and its lots are cut and then thrown away downstream as rejClear, so the
        // symptom is missing density rather than a building in the road.
        // Same helper the park pads use: per-edge width-aware, and it subdivides
        // long edges first so a bowing road still moves a long block edge.
        // Gate: tests/test_lot_road_clearance.cpp
        //       lot_block_interiors_clear_every_carriageway_on_a_mixed_width_net
        foot = pushPolyClearOfRoads(foot);
        if (foot.size() < 3) { ++blocksAllCarriageway; continue; }
        if (area(foot) < p.minLotArea * 1.5) continue;
        dbg->blocks.push_back(foot);
        blockFoots.push_back(foot);

        BlockInfo bf;
        bf.pp.seed = mix(static_cast<uint32_t>(bi), p.seed);
        bf.pp.targetArea = 480;
        bf.pp.minArea = ppMinArea;
        bf.pp.minEdge = p.parcelMinEdge > 0 ? p.parcelMinEdge : p.minShort;
        if (p.parcelCourtMinArea > 0) bf.pp.courtMinArea = p.parcelCourtMinArea;
        // DENSITY is a district decision too (device: "feels like a small
        // town"): urban quarters parcel small and build nearly wall-to-wall;
        // the financial core keeps big tower plates; suburbs keep their yards.
        const Vec2 footC = centroid(foot);
        bf.tag = districts.tagAt(footC);
        bf.lotSetback = p.lotSetback;
        bf.buildChance = p.buildChance;
        // Lot DIMENSIONS drive the row-split parceler now (frontWidth along
        // the street, lotDepth back from it), so each district reads as its
        // own grain: narrow-deep retail on Main St, wide-shallow tower plates
        // downtown, big industrial parcels, house lots in the suburbs. (The
        // town/city scale knob is p.blockSize upstream — a town parcels the
        // same district a touch bigger.)
        switch (bf.tag) {
            case DistrictTag::Financial: {  // SKYSCRAPER pads (density round)
                // Big plates, bigger still toward the hub: the core mints
                // 42-55 m frontages x 48-60 m depths, so glass/podium towers
                // stand on real floor plates instead of rowhouse slots.
                const Real cness = std::sqrt(std::max(
                    Real(0), 1.0 - (footC - p.center).length() /
                                       std::max(Real(1), p.innerRadius)));
                const Real ct = std::max(Real(0), (cness - 0.5) * 2.0);
                bf.pp.frontWidth = 42 + 13 * ct;   // 42 -> 55 at the hub
                bf.pp.lotDepth = 48 + 12 * ct;     // 48 -> 60
                bf.pp.targetArea = 2400;           // the OBB fallback matches
                bf.lotSetback = 1.0;
                bf.buildChance = std::min(Real(1), p.buildChance + 0.06);
                break; }
            case DistrictTag::Commercial:  // narrow, deep retail frontage
                bf.pp.frontWidth = 13; bf.pp.lotDepth = 30;
                bf.pp.targetArea = 300; bf.lotSetback = 0.7;
                bf.buildChance = std::min(Real(1), p.buildChance + 0.06); break;
            case DistrictTag::OldTown:     // small, tight, narrow
                bf.pp.frontWidth = 11; bf.pp.lotDepth = 22;
                bf.pp.targetArea = 210;
                bf.pp.minArea = std::min(ppMinArea, Real(80));
                bf.lotSetback = 0.5; bf.buildChance = 0.98; break;
            case DistrictTag::Industrial:  // big parcels
                bf.pp.frontWidth = 42; bf.pp.lotDepth = 52;
                bf.pp.targetArea = 700; bf.lotSetback = 1.2; break;
            case DistrictTag::Residential: // house lots with yards
                bf.pp.frontWidth = 18; bf.pp.lotDepth = 27;
                bf.pp.targetArea = 400; break;
        }
        // The parcel override rescales the district grain it just chose.
        bf.pp.targetArea *= ppAreaScale;
        bf.pp.frontWidth *= ppFrontScale;
        bf.pp.lotDepth *= ppDepthScale;
        // Sometimes a WHOLE small block is a park (device: "the green space
        // doesn't conform to the city block"): the pad is the block's own
        // road-inset interior, so its edges follow the surrounding streets
        // exactly — a real city square, not a leftover parcel.
        if ((bf.tag == DistrictTag::Commercial ||
             bf.tag == DistrictTag::Residential) &&
            area(foot) < 2600.0) {
            Hash blockRng(mix(bf.pp.seed, 0xB10Cu));
            if (blockRng.unit() < 0.10) {
                OBB2 gb = orientedBoundingBox(foot);
                LotBuilding g;
                g.site = centroid(foot);
                g.width = 2 * gb.half[0];
                g.depth = 2 * gb.half[1];
                g.height = 0.25;
                g.yaw = std::atan2(gb.axis[0].y, gb.axis[0].x);
                g.type = "park";
                g.recipe = "park_block";
                g.color = colorFor("park");
                g.pad = pushPolyClearOfRoads(foot);
                if (g.pad.empty()) continue;   // road-locked block: no square
                // The city square is DESIGNED: plaza, paths, fountain,
                // benches, hedges, tree spots (not a bare green pad).
                sculptPark(g, g.pad, g.height, p.ground,
                           mix(bf.pp.seed, 0xB10C2u), outParts);
                out.push_back(std::move(g));
                continue;
            }
        }
        std::vector<Lot> lots = subdivideBlock(foot, bf.pp);
        // RT_PARCEL_DEBUG: the per-block view the [citylots] summary cannot give.
        // The summary says how many lots a city produced; this says which blocks
        // produced them, at what district grain, on what shape of interior — which
        // is the difference between "the parcel grain is wrong" and "the blocks are
        // the wrong size", two diagnoses that look identical in the totals. Note
        // `edges`: a block arrives as the road graph's face with the roads' spline
        // samples still in it (living_city: 27 edges on a 77x28 m block), so an
        // "edge" here is a polyline segment, not a street frontage.
        if (std::getenv("RT_PARCEL_DEBUG")) {
            int courts = 0, tiny = 0;
            for (const Lot& l : lots) {
                if (l.court) ++courts;
                else if (area(l.footprint) < bf.pp.minArea) ++tiny;
            }
            const OBB2 fb2 = orientedBoundingBox(foot);
            std::fprintf(stderr,
                         "[parcel] %-11s area=%7.0f obb=%5.1fx%5.1f edges=%2zu "
                         "fw=%4.1f ld=%4.1f -> %2zu lots (%d court, %d tiny)\n",
                         districtName(bf.tag), area(foot), 2 * fb2.half[0],
                         2 * fb2.half[1], foot.size(), bf.pp.frontWidth,
                         bf.pp.lotDepth, lots.size(), courts, tiny);
        }
        for (const Lot& lot : lots) dbg->lots.push_back(lot.footprint);
        bf.foot = foot;
        binfos.push_back(bf);
        for (std::size_t li = 0; li < lots.size(); ++li) {
            // A block-interior COURT (frontage parceler, v2 step 10) is open
            // space, never a building lot — nothing landlocked gets built.
            // But it IS emitted: a sculpted mid-block green. On the big-block
            // geometry the court can be most of the block, and dropping it
            // silently rendered as broken bare terrain (Glenn's report).
            if (lots[li].court) {
                if (area(lots[li].footprint) < 60.0) continue;
                LotBuilding g;
                const OBB2 gb = orientedBoundingBox(lots[li].footprint);
                g.site = centroid(lots[li].footprint);
                g.width = 2 * gb.half[0];
                g.depth = 2 * gb.half[1];
                g.height = 0.25;
                g.yaw = std::atan2(gb.axis[0].y, gb.axis[0].x);
                g.type = "park";
                g.recipe = "court_green";
                g.color = colorFor("park");
                g.pad = pushPolyClearOfRoads(lots[li].footprint);
                if (g.pad.empty()) continue;
                sculptPark(g, g.pad, g.height, p.ground,
                           mix(bf.pp.seed, 0xC0947u + static_cast<uint32_t>(li)),
                           outParts);
                out.push_back(std::move(g));
                continue;
            }
            if (area(lots[li].footprint) < bf.pp.minArea) continue;
            cands.push_back({li, static_cast<int>(binfos.size()) - 1,
                             lots[li], -1});
        }
    }

    // ---- STAGE-10 ALLEYS (courts-with-alleys round) -------------------------
    // The frontage gate below refuses any lot whose whole footprint sits beyond
    // 14 m of a street surface — and the margin round proved it refuses REAL,
    // parcelled land: rim-block outer rows sit 15-21 m out (rejFrontage 1 -> 13
    // when the road-half term was dropped). Stage 10's contract says that land
    // "connects out" by an alley. So: for each block whose candidates FAIL the
    // reach, cut ONE service alley along the block's long axis through the
    // failing rows' front line — into the CLEARANCE graph, not the parcel
    // geometry. The rows already exist and are correctly shaped; they only
    // lack legal frontage. The gate then passes them, buildings keep just off
    // the pavement (Alley class, kAlleyClear), and a draped Path strip makes
    // the lane visible. Measured on living_city this fires on rim rectangles;
    // a deep enclosed block whose parcel walk orphans a row takes the same cut.
    const Real kFrontReach = 14.0;
    if (p.alleys && roads && p.alleyWidth > 0) {
        // Failing candidates, grouped by block: lot -> its front distance and
        // the vertex that attains it (the alley must pass near those fronts).
        struct BlockFail { std::vector<Vec2> fronts; };
        std::map<int, BlockFail> failing;
        for (const LotCand& c : cands) {
            Real front = Real(1e30);
            Vec2 at = c.lot.footprint.front();
            for (const Vec2& v : c.lot.footprint) {
                const Real d = roadSurfaceDist(v);
                if (d < front) { front = d; at = v; }
            }
            if (front > kFrontReach) failing[c.block].fronts.push_back(at);
        }
        for (const auto& [bi2, bfail] : failing) {
            const Poly2& foot = binfos[bi2].foot;
            if (foot.size() < 3 || bfail.fronts.empty()) continue;
            // The lane: block long axis, through the failing lots' front line.
            const OBB2 ob = orientedBoundingBox(foot);
            const Vec2 axis = ob.half[0] >= ob.half[1] ? ob.axis[0] : ob.axis[1];
            Vec2 through(0, 0);
            for (const Vec2& v : bfail.fronts) through = through + v;
            through = through * (Real(1) / bfail.fronts.size());
            // Clip the infinite line to the block interior (min/max parameter
            // over all boundary-edge crossings). A block the line barely
            // crosses (or crosses oddly) takes no alley — fail-safe: the lots
            // stay green exactly as today.
            Real tMin = Real(1e30), tMax = Real(-1e30);
            const int fn = static_cast<int>(foot.size());
            for (int i = 0; i < fn; ++i) {
                const Vec2& a = foot[i];
                const Vec2& b = foot[(i + 1) % fn];
                const Vec2 d2 = b - a;
                const Real den = cross(axis, d2);
                if (std::fabs(den) < Real(1e-9)) continue;
                const Real t = cross(a - through, d2) / den;
                const Real u = cross(a - through, axis) / den;
                if (u < Real(-0.05) || u > Real(1.05)) continue;
                tMin = std::min(tMin, t);
                tMax = std::max(tMax, t);
            }
            if (tMin > tMax) {
                // Degenerate clip (the lane runs along a boundary edge, or the
                // rim sliver's edges all filtered as near-parallel): span the
                // lane from the VERTEX projections instead — always defined,
                // and the clearance pass keeps buildings off any overreach.
                for (const Vec2& v : foot) {
                    const Real t = dot(v - through, axis);
                    tMin = std::min(tMin, t);
                    tMax = std::max(tMax, t);
                }
            }
            if (tMax - tMin < Real(12)) {           // too short to be a lane
                if (std::getenv("RT_PARCEL_DEBUG"))
                    std::fprintf(stderr,
                                 "[alley-skip] block %d span=%.1f at (%.1f, %.1f)\n",
                                 bi2, static_cast<double>(tMax - tMin),
                                 through.x, through.y);
                continue;
            }
            // Connect out: overshoot each end a little into the road verge, so
            // the pavement visually meets the street's sidewalk band.
            const Vec2 A = through + axis * (tMin - Real(2.5));
            const Vec2 B = through + axis * (tMax + Real(2.5));
            const int na = static_cast<int>(alleyGraph.nodes.size());
            alleyGraph.nodes.push_back(RoadNode{A});
            alleyGraph.nodes.push_back(RoadNode{B});
            RoadEdge ae;
            ae.a = na; ae.b = na + 1;
            ae.width = static_cast<Real>(p.alleyWidth);
            ae.klass = RoadClass::Alley;
            alleyGraph.edges.push_back(ae);
            dbg->alleys.push_back({A, B});
            // The PAVEMENT: a draped ribbon into the Path part (the same idiom
            // as the yards' front walks), asphalt-grey so it reads as a
            // service lane between the rows, not a garden path.
            if (outParts) {
                RenderMesh& path =
                    (*outParts)[static_cast<std::size_t>(PartId::Path)];
                auto gy = [&](const Vec2& v) {
                    return p.ground ? p.ground(v.x, v.y) : Real(0);
                };
                const Vec2 dirN = normalize(B - A);
                const Vec2 perp(-dirN.y, dirN.x);
                const Real hw = p.alleyWidth * Real(0.5);
                const Real L = (B - A).length();
                const int segs = std::max(1, static_cast<int>(L / 3.0));
                for (int s = 0; s < segs; ++s) {
                    const Vec2 q0 = A + dirN * (L * s / segs);
                    const Vec2 q1 = A + dirN * (L * (s + 1) / segs);
                    const Real y0 = gy(q0) + Real(0.06);
                    const Real y1 = gy(q1) + Real(0.06);
                    MeshBuilder::emitQuad(
                        path, Vec3(q0.x - perp.x * hw, y0, q0.y - perp.y * hw),
                        Vec3(q0.x + perp.x * hw, y0, q0.y + perp.y * hw),
                        Vec3(q1.x + perp.x * hw, y1, q1.y + perp.y * hw),
                        Vec3(q1.x - perp.x * hw, y1, q1.y - perp.y * hw),
                        Vec3(0, 1, 0), Vec3(0.46, 0.455, 0.44));
                }
            }
        }
    }

    // ---- GRADE between parcelling and growth (buried-buildings fix) --------
    // Fit the block planes/terraces NOW and wrap the ground sampler with them,
    // so PASS B/C — pad planes (padPlaneFor), plinths, skirts, park drapes on
    // real buildings — all sample the TERRACED ground the terrain will show.
    // The old order graded after buildings were meshed: pads froze pre-terrace
    // heights, and a house in a band the grade later raised sat buried to its
    // eaves. The host consumes outGrade instead of re-deriving (identical
    // input would give an identical fit, but one derivation is one truth).
    if (p.ground && outGrade && !blockFoots.empty()) {
        std::vector<Poly2> gradePolys;
        gradePolys.reserve(blockFoots.size());
        for (const Poly2& b2 : blockFoots) {
            Vec2 c(0, 0);
            for (const Vec2& v : b2) c = c + v;
            c = c * (Real(1) / b2.size());
            Poly2 grown;
            grown.reserve(b2.size());
            for (const Vec2& v : b2) {
                const Vec2 d = v - c;
                const Real l = d.length();
                // +4.5 m so the grade overlaps the road conform band (the
                // road's higher priority wins inside its own footprint) —
                // the level_loader idiom, moved here with the derivation.
                grown.push_back(l > Real(1e-6) ? c + d * ((l + Real(4.5)) / l)
                                               : v);
            }
            gradePolys.push_back(std::move(grown));
        }
        *outGrade = gradeBlocks(gradePolys, p.ground);
        if (!outGrade->empty()) {
            auto base = p.ground;
            auto flat = std::make_shared<std::vector<TerrainFlatten>>(*outGrade);
            p.ground = [base, flat](Real x, Real z) {
                return applyFlatten(*flat, x, z, base(x, z));
            };
        }
    }

    // ---- PASS B: the LANDMARK planner (architect, per hub cluster) ----------
    // Civic anchors are PLACED, never rolled: quotas filled by the best-
    // scoring eligible lot — biggest, and for the courthouse most central.
    // 8km-city P3: the quotas run PER HUB CLUSTER now (nearest-hub assignment
    // via hubClusters): the primary city keeps the full civic table; every
    // satellite town is guaranteed its own school + church. No hubs (or no
    // cluster ids) = one city, exactly the old plan.
    {
        auto clusterOf = [&](const Vec2& q) -> int {
            if (p.hubs.empty() || p.hubClusters.size() != p.hubs.size())
                return 0;
            std::size_t best = 0;
            Real bd = 1e30;
            for (std::size_t i = 0; i < p.hubs.size(); ++i) {
                const Real dd = (q - p.hubs[i].first).lengthSquared();
                if (dd < bd) { bd = dd; best = i; }
            }
            return p.hubClusters[best];
        };
        std::vector<LandmarkCand> lmc(cands.size());
        for (std::size_t ci = 0; ci < cands.size(); ++ci) {
            const LotCand& c = cands[ci];
            OBB2 ob = orientedBoundingBox(c.lot.footprint);
            lmc[ci].tag = binfos[c.block].tag;
            lmc[ci].shortSide = 2 * std::min(ob.half[0], ob.half[1]);
            lmc[ci].area = area(c.lot.footprint);
            lmc[ci].pos = centroid(c.lot.footprint);
            lmc[ci].cluster = clusterOf(lmc[ci].pos);
        }
        const std::vector<int> placed =
            planLandmarks(lmc, p.center, p.innerRadius);
        for (std::size_t ci = 0; ci < cands.size(); ++ci)
            cands[ci].landmark = placed[ci];
    }

    // ---- PASS C: grow every lot (landmarks use their PLACED recipes) --------
    for (const LotCand& cand : cands) {
        {
            const BlockInfo& bf = binfos[cand.block];
            const ParcelParams& pp = bf.pp;
            const DistrictTag blockTag = bf.tag;
            const Real lotSetback = bf.lotSetback;
            const Real buildChance = bf.buildChance;
            const std::size_t li = cand.li;
            const Lot& lot = cand.lot;
            Hash rng(mix(pp.seed, static_cast<uint32_t>(li) + 1));
            // An unbuilt lot is reported as a GREEN (device: "empty lots had
            // vegetation like trees and grass"), not silently dropped — the
            // caller plants grass + trees on it. Same for lots the sliver /
            // fill gates reject below.
            auto emitGreen = [&]() {
                OBB2 gb = orientedBoundingBox(lot.footprint);
                LotBuilding g;
                g.site = centroid(lot.footprint);
                g.width = 2 * gb.half[0];
                g.depth = 2 * gb.half[1];
                g.height = 0.12;
                g.yaw = std::atan2(gb.axis[0].y, gb.axis[0].x);
                g.type = "green";
                g.recipe = "green";
                g.color = Vec3(0.32, 0.52, 0.30);
                g.pad = pushPolyClearOfRoads(lot.footprint);
                if (g.pad.empty()) return;     // road-locked sliver: no green
                // NO pad mesh (device: "I still get the green pads here and
                // there. We should remove them"): the terrain is the green's
                // ground; the lot polygon still marks it for planting.
                out.push_back(std::move(g));
            };
            // FRONTAGE GATE (city contract stage 4): a building must front a
            // street. Ring lots front by construction, so the bound is TIGHT
            // now (14 m — a sidewalk plus a setback, not half a block), and
            // it applies to EVERYONE: landmarks are chosen from ring lots
            // that already front, and fallback-path lots (synthesized
            // frontage) must pass the same bar — no more corner-grazing
            // mid-block builds. Courts are green by design; this catches
            // the strays.
            {
                Real front = Real(1e30);
                for (const Vec2& v : lot.footprint)
                    front = std::min(front, roadSurfaceDist(v));
                if (front > 14.0) {
                    dbg->rejFrontage++;
                    if (std::getenv("RT_PARCEL_DEBUG")) {
                        const Vec2 c = centroid(lot.footprint);
                        std::fprintf(stderr,
                                     "[frontage-rej] at (%7.1f, %7.1f) front=%5.1f m "
                                     "area=%6.0f district=%d\n",
                                     c.x, c.y, front, area(lot.footprint),
                                     lot.district);
                    }
                    emitGreen();
                    continue;
                }
            }
            // UNDER A FREEWAY DECK: a normal building would clip the elevated
            // structure, so this lot becomes something that FITS beneath it. The
            // block/lot pipeline thus builds the RIGHT thing under the freeway
            // instead of a rejected stub. Landmarks are exempt (placed on
            // purpose). A high FREEWAY deck also admits a single-storey building;
            // a variable-height RAMP shadow gets only low uses.
            int ufShort = 0;   // build a one-storey mass that clears the deck
            {
                bool nearestIsRamp = false;
                if (cand.landmark < 0 &&
                    underFreeway(centroid(lot.footprint), &nearestIsRamp)) {
                    const uint32_t s =
                        mix(pp.seed, static_cast<uint32_t>(li) * 17u + 11u);
                    const Real roll = Hash(s).unit();
                    // 0 open space, 1 parking, 2 utility, 3 short building
                    const int kind = nearestIsRamp
                        ? (roll < 0.45 ? 0 : (roll < 0.75 ? 1 : 2))
                        : (roll < 0.30 ? 0 : (roll < 0.55 ? 1 : (roll < 0.80 ? 2 : 3)));
                    if (kind == 3) {
                        ufShort = 1;   // fall through to the building path, capped
                    } else {
                        OBB2 gb = orientedBoundingBox(lot.footprint);
                        LotBuilding u;
                        u.site = centroid(lot.footprint);
                        u.width = 2 * gb.half[0];
                        u.depth = 2 * gb.half[1];
                        u.yaw = std::atan2(gb.axis[0].y, gb.axis[0].x);
                        u.height = 0.16;         // low — tucks under the deck lift
                        u.type = "park";         // host: terrain ground + padMesh (no prism)
                        u.recipe = kind == 0 ? "underfreeway_open"
                                 : kind == 1 ? "underfreeway_parking"
                                             : "underfreeway_utility";
                        u.pad = lot.footprint;   // deliberately IN the ROW
                        u.color = Vec3(0.30, 0.50, 0.26);
                        if (kind == 0) {         // landscaped open space, no canopy
                            sculptPark(u, u.pad, u.height, p.ground, s, outParts);
                            u.treeSpots.clear();
                        } else {                 // paved parking / utility yard
                            sculptUnderPad(u, u.pad, p.ground, s, /*utility=*/kind == 2);
                        }
                        ++underFwCount;
                        out.push_back(std::move(u));
                        continue;
                    }
                }
            }
            if (cand.landmark < 0 && rng.unit() > buildChance) {
                dbg->rejChance++;
                emitGreen(); continue;   // plaza / gap (landmarks always build)
            }

            // RELIEF gate (device: buildings "floating off the ground" on the
            // island brink): a lot whose ground falls away more than a storey
            // across its footprint cannot be seated by a single pad — the pad
            // plane leaves one side hovering and the other buried, and the
            // grade skirt becomes a cliff-sized wall. Such land is HILLSIDE,
            // not a parcel: it goes green. Sampled at the footprint corners +
            // centroid on the same ground the pad plane would use.
            if (p.ground) {
                Real lo = 1e30, hi = -1e30;
                auto sampleG = [&](const Vec2& v) {
                    const Real g = p.ground(v.x, v.y);
                    lo = std::min(lo, g);
                    hi = std::max(hi, g);
                };
                for (const Vec2& v : lot.footprint) sampleG(v);
                sampleG(centroid(lot.footprint));
                if (hi - lo > p.maxPadRelief) {
                    dbg->rejRelief++;
                    emitGreen(); continue;
                }
            }

            // Building set back from its own lot lines (district-tuned).
            Poly2 site = inset(lot.footprint, lotSetback);
            if (site.size() < 3 || area(site) < 30) site = lot.footprint;

            OBB2 obb = orientedBoundingBox(site);
            Real w = 2 * obb.half[0], d = 2 * obb.half[1];
            Real shortSide = std::min(w, d), longSide = std::max(w, d);
            // Dense districts parcel small — an 8 m rowhouse plan is CORRECT
            // in old town — so the sliver floor relaxes there.
            const Real minShort =
                (blockTag == DistrictTag::OldTown ||
                 blockTag == DistrictTag::Commercial)
                    ? std::min(p.minShort, p.minShortUrban) : p.minShort;
            if (shortSide < minShort) {                                         // sliver
                dbg->rejSliver++;
                emitGreen(); continue;
            }
            // A rescued site is the sub-rectangle the building actually
            // takes; recompute the derived frame from it.
            auto retakeSite = [&](Poly2 rect) {
                site = std::move(rect);
                obb = orientedBoundingBox(site);
                w = 2 * obb.half[0];
                d = 2 * obb.half[1];
                shortSide = std::min(w, d);
                longSide = std::max(w, d);
            };
            // Is `rect` usable ground: corners + edge midpoints on the lot
            // and clear of the road corridors (concave-safe enough for the
            // convex-ish ring lots this sees).
            auto rectFits = [&](const Poly2& rect, const Poly2& host) {
                const Vec2 c0 = centroid(rect);
                for (std::size_t vi = 0; vi < rect.size(); ++vi) {
                    const Vec2& v = rect[vi];
                    const Vec2 m2 = (v + rect[(vi + 1) % rect.size()]) * 0.5;
                    if (!pointInPolygon(host, v + (c0 - v) * 0.02)) return false;
                    if (!pointInPolygon(host, m2 + (c0 - m2) * 0.02)) return false;
                    if (roads && (!clearOfRoads(v) || !clearOfRoads(m2)))
                        return false;
                }
                return true;
            };
            if (longSide > shortSide * p.maxAspect) {                           // knife blade
                // RECOVERABLE (density round): a too-long lot still holds a
                // fine building on PART of its length — clamp the site to a
                // maxAspect rectangle before giving up and greening.
                bool rescued = false;
                const int la = obb.longAxis();
                const Vec2 U = obb.axis[la], V = obb.axis[1 - la];
                const Real hs = obb.half[1 - la] * 0.96;
                Real hl = shortSide * p.maxAspect * 0.5 * 0.95;
                for (int attempt = 0; attempt < 4 && hl > hs; ++attempt, hl *= 0.85) {
                    Poly2 rect{obb.center - U * hl - V * hs,
                               obb.center + U * hl - V * hs,
                               obb.center + U * hl + V * hs,
                               obb.center - U * hl + V * hs};
                    if (rectFits(rect, site)) {
                        retakeSite(std::move(rect));
                        rescued = true;
                        break;
                    }
                }
                if (!rescued) {
                    dbg->rejAspect++;
                    emitGreen(); continue;
                }
            }
            // How much of its oriented box the lot actually fills. PLAN massing
            // takes the polygon itself, so off-cut lots (L / triangle /
            // flatiron wedges) are buildable now — the very shapes that make
            // interesting buildings (device: "take advantage of the weirder
            // lots"). Only truly degenerate slivers go green here; the BOX
            // fallback below still demands the old 0.72 fill, since a box on a
            // low-fill lot is the "malformed overhanging mass" bug.
            Real fill = area(site) / std::max(Real(1e-6), w * d);
            if (fill < 0.45) {
                // RECOVERABLE: shrink-fit a box INSIDE the polygon (the same
                // mechanism as the box fallback below) — a wedge or L off-cut
                // still builds on its fat part instead of greening whole.
                Scope sc = scopeFromFootprint(site, 0.0, 10.0, clearOfRoads);
                const Real fitShort = std::min(sc.size.x, sc.size.z);
                const Real fitLong = std::max(sc.size.x, sc.size.z);
                Vec2 r2(sc.axis[0].x, sc.axis[0].z);
                Vec2 f2(sc.axis[2].x, sc.axis[2].z);
                Vec2 o2(sc.origin.x, sc.origin.z);
                Poly2 rect{o2, o2 + r2 * sc.size.x,
                           o2 + r2 * sc.size.x + f2 * sc.size.z,
                           o2 + f2 * sc.size.z};
                if (fitShort >= minShort && fitLong <= fitShort * p.maxAspect &&
                    rectFits(rect, site)) {
                    retakeSite(std::move(rect));
                    fill = 1.0;
                } else {
                    dbg->rejFill++;
                    emitGreen(); continue;
                }
            }

            LotBuilding b;
            b.site = centroid(site);
            b.width = w;
            b.depth = d;
            b.yaw = std::atan2(obb.axis[0].y, obb.axis[0].x);
            // A DISTRICT ENDS AT A STREET (city-pipeline.md stage 9, "district
            // refinement"). This used to re-derive the tag per LOT from the lot's
            // own centroid, while the block above already resolved one at its
            // centroid — so the two disagreed wherever a zone boundary crossed a
            // block. `tagAt` is a continuous function of position (radial rings,
            // or distance to the nearest hub); nothing in it knows where the
            // streets are, so the boundary fell mid-block and one row of a
            // commercial block built houses while the row opposite built shops.
            // Worse, it was incoherent with the block's own decisions: the parcel
            // GRAIN, setback and buildChance all come from `bf.tag`, so a block
            // could be cut into 13x30 m retail slots and then have cottages placed
            // on them.
            //
            // Take the block's tag. Block faces are road-graph faces, so their
            // edges ARE streets by construction — zoning boundaries now land on
            // them, and every lot in a block agrees with the grain it was cut at.
            const DistrictTag tag = blockTag;
            // Coreness peaks the skyline: 1 at the city centre, 0 at the
            // financial district's rim — the architect grows the skyscraper
            // cluster from it. sqrt widens the peak so the cluster is a
            // CLUSTER, not one tall building at the exact centre.
            const Real coreness = std::sqrt(std::max(
                Real(0), 1.0 - (b.site - p.center).length() /
                                   std::max(Real(1), p.innerRadius)));
            BuildingRecipe rec =
                cand.landmark >= 0
                    ? architectLandmark(static_cast<LandmarkKind>(cand.landmark),
                                        shortSide, area(site),
                                        mix(pp.seed,
                                            static_cast<uint32_t>(li) * 7u + 3u))
                    : architectPick(tag, shortSide, area(site),
                                    mix(pp.seed,
                                        static_cast<uint32_t>(li) * 7u + 3u),
                                    coreness,
                                    p.archetypeBook.empty() ? nullptr
                                                            : &p.archetypeBook);
            b.type = rec.placeType;
            b.recipe = rec.name;
            b.block = cand.block;
            b.district = districtName(tag);
            b.color = colorFor(b.type);
            if (rec.massing == BuildingRecipe::Massing::Park) {
                b.height = 0.18;  // low green pad — stays UNDER the road deck
                                  // lift so park edges tuck below the asphalt
                b.pad = pushPolyClearOfRoads(lot.footprint);
                if (b.pad.empty()) continue;   // road-locked sliver: no park
                sculptPark(b, b.pad, b.height, p.ground,
                           mix(pp.seed, static_cast<uint32_t>(li) * 13u + 5u),
                           outParts);
                out.push_back(std::move(b));
                continue;
            }
            // Grow a REAL building that FITS the lot: its oriented footprint IS the
            // scope — shrunk until it sits inside the lot AND clear of every road
            // corridor (clearOfRoads), so no mass overhangs a sidewalk. Height
            // comes from floors. Parts keep their shape-grammar PartId, merged
            // into outParts so the caller binds the SAME PBR material recipes the
            // shape:"city" pipeline uses — not a flattened vertex-colour blob.
            BuildingParams bp = rec.params;   // the architect's recipe
            // Under a high freeway deck: one storey only, so the mass clears the
            // structure (a low outbuilding beneath the viaduct, not a tower).
            if (ufShort) { bp.floors = 1; b.type = "underfreeway"; ++underFwCount; }
            // The STYLE BOOK (Lua data layer) overlays look overrides by
            // recipe name — cladding, windows, colours — before growth.
            if (p.styleHook) p.styleHook(rec.name, bp);
            // The lot record carries the building's REAL facade colour (post
            // style book), not the zoning palette — the HLOD mass-box proxy
            // bakes from it, and a proxy that pops in wearing the zone colour
            // instead of the building's own reads as a different building
            // (device: "the cubes don't look like they have the color").
            b.color = bp.wallColor;
            // The door (and the retail front) faces the nearest STREET, not a
            // fixed +Z: aim faceDir at the closest point on the road network.
            if (roads) {
                Real best = 1e30;
                Vec2 q = b.site;
                for (const RoadEdge& e : roads->edges) {
                    if (e.a < 0 || e.b < 0 ||
                        e.a >= static_cast<int>(roads->nodes.size()) ||
                        e.b >= static_cast<int>(roads->nodes.size())) continue;
                    const Vec2& ra = roads->nodes[e.a].pos;
                    const Vec2& rb = roads->nodes[e.b].pos;
                    Vec2 ab = rb - ra;
                    Real len2 = ab.lengthSquared();
                    Real t = len2 > 1e-12 ? dot(b.site - ra, ab) / len2 : 0.0;
                    t = t < 0 ? 0 : (t > 1 ? 1 : t);
                    Vec2 cp(ra.x + ab.x * t, ra.y + ab.y * t);
                    Real dd = (cp - b.site).length();
                    if (dd < best) { best = dd; q = cp; }
                }
                Vec2 dir = q - b.site;
                if (dir.length() > 1e-6) {
                    dir = normalize(dir);
                    bp.faceDir = Vec3(dir.x, 0, dir.y);
                }
            }

            // PLAN massing (P3.c): the building takes the LOT'S OWN SHAPE — the
            // simplified site polygon (short edges merged so no micro-facades),
            // inset progressively until every vertex clears the road corridors.
            // Falls back to the shrink-fit box when the plan can't clear.
            Poly2 plan = site;
            {   // merge sub-2.5 m edges + drop near-collinear vertices
                Poly2 s;
                for (std::size_t vi = 0; vi < plan.size(); ++vi) {
                    const Vec2& prev = s.empty() ? plan.back() : s.back();
                    if ((plan[vi] - prev).length() < 2.5 && !s.empty()) continue;
                    s.push_back(plan[vi]);
                }
                Poly2 s2;
                for (std::size_t vi = 0; vi < s.size(); ++vi) {
                    Vec2 a = s[(vi + s.size() - 1) % s.size()], m2 = s[vi],
                         c2 = s[(vi + 1) % s.size()];
                    // 0.08: shallow bends inherited from the block subdivision
                    // used to survive into long facades as a visible KINK
                    // partway along a wall (device feedback) — straighten
                    // anything under ~4.6 deg; real corners keep their turn.
                    if (std::fabs(cross(normalize(m2 - a), normalize(c2 - m2))) < 0.08)
                        continue;
                    s2.push_back(m2);
                }
                // FLATIRON prows: a very acute corner would grow a knife-edge
                // facade — ROUND it into a short chord arc instead (device:
                // "rounded at the ends ... rather than becoming a perfect
                // corner"), the classic flatiron nose. Quadratic bezier
                // through the cut points with the sharp corner as control.
                Poly2 s3;
                for (std::size_t vi = 0; vi < s2.size(); ++vi) {
                    Vec2 a = s2[(vi + s2.size() - 1) % s2.size()], m2 = s2[vi],
                         c2 = s2[(vi + 1) % s2.size()];
                    Vec2 d0 = normalize(m2 - a), d1 = normalize(c2 - m2);
                    const Real cut = 3.0;
                    if (dot(d0, d1) < -0.45 && (m2 - a).length() > cut * 2 &&
                        (c2 - m2).length() > cut * 2) {
                        Vec2 p0 = m2 - d0 * cut, p1 = m2 + d1 * cut;
                        for (int k = 0; k <= 4; ++k) {
                            Real t = k / 4.0, mt = 1 - t;
                            s3.push_back(p0 * (mt * mt) + m2 * (2 * mt * t) +
                                         p1 * (t * t));
                        }
                    } else {
                        s3.push_back(m2);
                    }
                }
                if (s3.size() >= 3) plan = s3;
            }
            bool planOk = plan.size() >= 3 && area(plan) > p.minLotArea * 0.5;
            if (planOk && roads) {
                bool fit = false;
                for (Real t : {Real(0), Real(0.8), Real(1.6), Real(2.6), Real(3.6)}) {
                    Poly2 cand = t > 0 ? inset(plan, t) : plan;
                    if (cand.size() < 3 || area(cand) < 40) break;
                    // The MESH is the guarantee (lot buildings are visual-only
                    // — no box collider since the invisible-walls fix), so the
                    // plan's own vertices clearing the corridors is what keeps
                    // facades off the street.
                    bool clear = true;
                    for (const Vec2& v : cand)
                        if (!clearOfRoads(v)) { clear = false; break; }
                    if (clear) { plan = cand; fit = true; break; }
                }
                // Corner-pocket rescue: a curvy road bows into ONE corner of
                // the lot, and a global inset shrinks the whole plan to
                // nothing before that corner clears. Nudge just the offending
                // vertices toward the centroid instead — the rest of the lot
                // keeps its footprint (this was ~20% of all lots going green).
                if (!fit) {
                    Poly2 cand = plan;
                    const Vec2 c0 = centroid(cand);
                    bool ok = cand.size() >= 3;
                    for (Vec2& v : cand) {
                        int guard = 0;
                        while (!clearOfRoads(v) && guard++ < 8)
                            v = v + (c0 - v) * 0.18;
                        if (!clearOfRoads(v)) { ok = false; break; }
                    }
                    if (ok && area(cand) > std::max(Real(40), area(plan) * 0.5)) {
                        plan = cand;
                        fit = true;
                    }
                }
                if (!fit) dbg->rejClear++;   // (box fallback may still build)
                planOk = fit;
            }
            // COURTYARD massing (device: "a lot of same-y looking ones"): a
            // big boxy mid-rise lot carves a court into its BACK side — away
            // from the street — so the mass reads as an L / U / T plan with
            // wings instead of yet another extruded rectangle. The carve is
            // inward-only, so road clearance established above still holds.
            if (planOk && rec.massing == BuildingRecipe::Massing::LotPlan &&
                rec.params.floors >= 3 && area(plan) > 280 && rng.unit() < 0.5) {
                OBB2 cb = orientedBoundingBox(plan);
                if (area(plan) > 0.85 * (4 * cb.half[0] * cb.half[1])) {
                    Vec2 face(bp.faceDir.x, bp.faceDir.z);
                    const Real da = dot(cb.axis[0], face), db = dot(cb.axis[1], face);
                    const int backAxis = std::fabs(da) >= std::fabs(db) ? 0 : 1;
                    const Real backSign = (backAxis == 0 ? da : db) > 0 ? -1.0 : 1.0;
                    Vec2 v = cb.axis[backAxis] * backSign;      // toward the back
                    Vec2 u = cb.axis[1 - backAxis];             // along the back edge
                    const Real hu = cb.half[1 - backAxis], hv = cb.half[backAxis];
                    const Real w = 2 * hu * rng.range(0.30, 0.45);
                    const Real dpt = std::min(2 * hv * 0.35, rng.range(4.5, 7.5));
                    // Wings must stay walls, not slivers.
                    if (2 * hv - dpt > 6.0 && 2 * hu - w > 6.0 && dpt > 3.0) {
                        // Court at a corner (an L) or centred-ish (a U/T).
                        Real s0 = rng.unit() < 0.4
                                      ? (rng.unit() < 0.5 ? -hu + 2.5 : hu - w - 2.5)
                                      : -w * 0.5 + rng.range(-0.15, 0.15) * hu;
                        s0 = std::max(-hu + 2.5, std::min(hu - w - 2.5, s0));
                        const Vec2 C = cb.center;
                        Poly2 cand{C - v * hv - u * hu, C - v * hv + u * hu,
                                   C + v * hv + u * hu, C + v * hv + u * (s0 + w),
                                   C + v * (hv - dpt) + u * (s0 + w),
                                   C + v * (hv - dpt) + u * s0,
                                   C + v * hv + u * s0, C + v * hv - u * hu};
                        ensureCCW(cand);   // area() is |signedArea| — old guard was dead
                        // The court rect is the plan's OBB — on a not-quite-
                        // rect plan its corners can poke past the cleared
                        // polygon, so re-check them against the roads.
                        bool ok = true;
                        if (roads)
                            for (const Vec2& q : cand)
                                if (!clearOfRoads(q)) { ok = false; break; }
                        if (ok) plan = cand;
                    }
                }
            }
            // BOX-MASS recipes (pagoda / cylinder shapes) must reach
            // growBuilding — the plan path can't dispatch a BuildingShape —
            // so a roomy rect-ish lot takes the shrink-fit box fallback
            // below. A cramped/off-cut lot grows through the plan grammar
            // instead (the shape can't dispatch, but the lot still builds).
            if (rec.massing == BuildingRecipe::Massing::BoxMass &&
                fill >= 0.72 && shortSide >= 10.0)
                planOk = false;
            // A HOUSE sits on a small centred rectangle, not the whole lot —
            // the rest of the parcel reads as its yard (residential realism).
            bool yardApplied = false;
            if (planOk && rec.massing == BuildingRecipe::Massing::RectYard) {
                Real hw2 = std::min(obb.half[0] - 1.6, rec.yardHalfWMax);
                Real hd2 = std::min(obb.half[1] - 1.6, rec.yardHalfDMax);
                // Frontage-first lots are TRAPEZOIDS (v2 step 10), so a house
                // sized to the OBB pokes past the tapered edges. SHRINK-TO-FIT:
                // retry smaller (centred on the lot centroid, which lies inside
                // a trapezoid) until every corner sits on the lot and off the
                // road — a tapered lot just gets a smaller house + more yard.
                Vec2 c = centroid(site);
                if (!pointInPolygon(site, c)) c = obb.center;
                for (int attempt = 0; attempt < 5 && hw2 > 3.5 && hd2 > 3.5;
                     ++attempt) {
                    Poly2 house{c - obb.axis[0] * hw2 - obb.axis[1] * hd2,
                                c + obb.axis[0] * hw2 - obb.axis[1] * hd2,
                                c + obb.axis[0] * hw2 + obb.axis[1] * hd2,
                                c - obb.axis[0] * hw2 + obb.axis[1] * hd2};
                    bool clear = true;
                    for (const Vec2& v : house)
                        if (!pointInPolygon(site, v) ||
                            (roads && !clearOfRoads(v))) { clear = false; break; }
                    if (clear) { plan = house; yardApplied = true; break; }
                    hw2 *= 0.82;
                    hd2 *= 0.82;
                }
            }
            // The architect's ROUND towers: a chord-tessellated circle plan
            // inscribed in the lot (a real drum, cornices and tiers included).
            if (planOk && rec.massing == BuildingRecipe::Massing::Circle &&
                shortSide > 17) {
                const Real rad = shortSide * 0.5 - 1.6;
                Poly2 circ;
                for (int k = 0; k < 16; ++k) {
                    Real a2 = 2.0 * 3.14159265358979323846 * k / 16;
                    circ.push_back(b.site + Vec2(std::cos(a2), std::sin(a2)) * rad);
                }
                bool clear = true;
                if (roads)
                    for (const Vec2& v : circ)
                        if (!clearOfRoads(v)) { clear = false; break; }
                if (clear) plan = circ;
            }
            // ROWHOUSES (device: "town homes ... packed side by side"): the
            // lot becomes a terrace of narrow townhome UNITS sharing party
            // walls — each its own plan building (door, stoop, cladding,
            // sometimes its own gable) grown side by side with zero gaps.
            if (planOk && rec.massing == BuildingRecipe::Massing::RowStrip) {
                OBB2 sb = orientedBoundingBox(plan);
                const int la = sb.longAxis(), sa2 = 1 - la;
                const Real len = 2 * sb.half[la];
                const Real dep = std::min(2 * sb.half[sa2], Real(11.0));
                const int units =
                    static_cast<int>(len / rng.range(5.6, 7.0));
                Vec2 u = sb.axis[la], v = sb.axis[sa2];
                bool stripOk = units >= 3 && dep > 6.5;
                if (stripOk && roads)
                    for (int sx = -1; sx <= 1 && stripOk; sx += 2)
                        for (int sy = -1; sy <= 1; sy += 2) {
                            Vec2 corner = sb.center + u * (len * 0.5 * sx) +
                                          v * (dep * 0.5 * sy);
                            if (!clearOfRoads(corner)) { stripOk = false; break; }
                        }
                if (stripOk) {
                    const Real uw = len / units;
                    const Vec2 c0 = sb.center - u * (len * 0.5);
                    // One shared pad plane for the whole terrace: the strip is
                    // graded flat as ONE pad, so the party-wall units sit flush
                    // on it instead of staggering into the cut.
                    b.groundY = padPlaneFor(plan, Vec2(bp.faceDir.x, bp.faceDir.z));
                    const Real stripBase =
                        p.ground ? b.groundY + plinth : baseYFor(plan);
                    for (int k = 0; k < units; ++k) {
                        Poly2 up4{c0 + u * (uw * k) - v * (dep * 0.5),
                                  c0 + u * (uw * (k + 1)) - v * (dep * 0.5),
                                  c0 + u * (uw * (k + 1)) + v * (dep * 0.5),
                                  c0 + u * (uw * k) + v * (dep * 0.5)};
                        BuildingParams upar = architectRowUnit(
                            mix(bp.seed, static_cast<uint32_t>(k) * 31u + 7u),
                            bp.floors);
                        upar.faceDir = bp.faceDir;
                        if (p.styleHook) p.styleHook("rowhouse_unit", upar);
                        BuildingMesh um = growPlanBuilding(up4, upar, stripBase);
                        mergeParts(outParts, um);
                        if (outFlatParts)
                            mergeParts(outFlatParts,
                                       growPlanBuilding(up4, upar, stripBase,
                                                        FacadeDetail::Flat));
                        b.height = std::max(b.height, um.height);
                    }
                    b.site = sb.center;
                    b.baseY = stripBase;
                    b.width = 2 * sb.half[0];
                    b.depth = 2 * sb.half[1];
                    b.yaw = std::atan2(sb.axis[0].y, sb.axis[0].x);
                    b.plan = {sb.center - u * (len * 0.5) - v * (dep * 0.5),
                              sb.center + u * (len * 0.5) - v * (dep * 0.5),
                              sb.center + u * (len * 0.5) + v * (dep * 0.5),
                              sb.center - u * (len * 0.5) + v * (dep * 0.5)};
                    emitFoundation(b.plan, b.groundY, b.baseY);
                    out.push_back(std::move(b));
                    continue;
                }
                // Too short/shallow for a terrace: build as one plan building.
            }

            // PLAN QUALITY (device: "a really degenerate triangle with sharp
            // edges and barely no space"): the OBB short side overestimates a
            // wedge's usable width, so gauge the finished plan by its
            // inradius-ish 4*area/perimeter. Too pinched → green; merely
            // wedge-shaped → the recipe shrinks to what the floor plate can
            // actually carry (no skyscraper on a knife of a lot).
            if (planOk) {
                Real per = 0;
                for (std::size_t vi = 0; vi < plan.size(); ++vi)
                    per += (plan[(vi + 1) % plan.size()] - plan[vi]).length();
                const Real effShort = std::min(
                    shortSide, 4 * area(plan) / std::max(per, Real(1e-6)));
                if (effShort < 5.5) {
                    dbg->rejPlan++;
                    emitGreen(); continue;
                }
                const Real qq = effShort / std::max(shortSide, Real(1e-6));
                if (qq < 0.8)
                    bp.floors = std::max(1, static_cast<int>(bp.floors * qq));
            }

            BuildingMesh bm;
            BuildingMesh bmFlat;   // the LOD1 twin, same massing decisions (R2)
            // On terrain the building rises from its graded pad plane (plus the
            // plinth reveal); every walk-up entrance earns steps to the door —
            // porticos and bay-door fronts already bring their own.
            b.groundY = padPlaneFor(planOk ? plan : site,
                                    Vec2(bp.faceDir.x, bp.faceDir.z));
            b.baseY = p.ground ? b.groundY + plinth
                               : baseYFor(planOk ? plan : site);
            // PLAZA massing (device: "a building structure without the
            // building"): the fitted plan becomes a raised paver podium —
            // graded + flattened like any building pad (type "civic", so the
            // park/green flatten skip doesn't apply), then dressed with
            // stairs, fencing, fountain, planters, benches. No storeys grown.
            if (rec.massing == BuildingRecipe::Massing::Plaza) {
                if (!planOk) { emitGreen(); continue; }
                b.plan = plan;               // pad flatten + walkable prism
                OBB2 pb = orientedBoundingBox(plan);
                b.site = pb.center;
                b.width = 2 * pb.half[0];
                b.depth = 2 * pb.half[1];
                b.yaw = std::atan2(pb.axis[0].y, pb.axis[0].x);
                b.baseY = p.ground ? b.groundY : baseYFor(plan);   // no plinth
                b.height = 0.35;             // the podium IS the massing
                sculptPlaza(b, plan, p.ground,
                            mix(pp.seed, static_cast<uint32_t>(li) * 31u + 17u),
                            outParts, roads);
                LOG_INFO << "[plaza] at (" << b.site.x << ", " << b.site.y
                         << ") area " << static_cast<int>(area(plan)) << " m2";
                out.push_back(std::move(b));
                continue;
            }
            if (p.ground && !bp.entranceSteps && bp.portico == 0 &&
                bp.groundBays == 0 && !bp.porch && bp.floors > 0)
                bp.entranceSteps = true;
            // PODIUM TOWER massing (density round, "more varied building
            // shapes"): a few-storey podium takes the WHOLE lot plan (street
            // wall, ground retail), and a slender curtain-wall tower rises
            // from its roof — the modern downtown block. Falls through to a
            // single mass when the plan can't stand a tower.
            if (planOk && rec.massing == BuildingRecipe::Massing::PodiumTower &&
                rec.podiumFloors > 0 && bp.floors > rec.podiumFloors + 4) {
                OBB2 pb = orientedBoundingBox(plan);
                Real thw = std::min(pb.half[0] * 0.62, Real(15.0));
                Real thd = std::min(pb.half[1] * 0.62, Real(15.0));
                Poly2 tplan;
                bool towerOk = false;
                for (int attempt = 0; attempt < 3 && !towerOk; ++attempt) {
                    if (std::min(thw, thd) < 5.5) break;
                    tplan = {pb.center - pb.axis[0] * thw - pb.axis[1] * thd,
                             pb.center + pb.axis[0] * thw - pb.axis[1] * thd,
                             pb.center + pb.axis[0] * thw + pb.axis[1] * thd,
                             pb.center - pb.axis[0] * thw + pb.axis[1] * thd};
                    towerOk = true;
                    for (const Vec2& v : tplan)
                        if (!pointInPolygon(plan, v) ||
                            (roads && !clearOfRoads(v))) {
                            towerOk = false;
                            break;
                        }
                    if (!towerOk) { thw *= 0.85; thd *= 0.85; }
                }
                if (towerOk) {
                    BuildingParams pod = bp;   // the street-wall base
                    pod.floors = rec.podiumFloors;
                    pod.setbackFloors = 0;
                    pod.spire = false;
                    pod.dome = false;
                    BuildingMesh pm = growPlanBuilding(plan, pod, b.baseY);
                    BuildingParams twr = bp;   // the shaft: no street-level kit
                    twr.floors = std::max(1, bp.floors - rec.podiumFloors);
                    twr.groundRetail = false;
                    twr.walkableGround = false;
                    twr.baseCourse = false;
                    twr.awning = false;
                    twr.entranceSteps = false;
                    twr.groundHeight = twr.floorHeight;
                    if (twr.floors > 18) {
                        twr.setbackFloors = 6;   // tall shafts keep tiers
                        twr.setbackEvery = 1.5;
                    }
                    // The tower roots just below the podium parapet, so the
                    // joint never shows a gap.
                    const Real towerBase =
                        b.baseY +
                        std::max(Real(0), pm.height - human::PARAPET - 0.15);
                    BuildingMesh tm = growPlanBuilding(tplan, twr, towerBase);
                    if (outParts) {
                        for (const RenderMesh& part : pm.parts) {
                            const int mi = part.materialIndex;
                            if (mi >= 0 &&
                                mi < static_cast<int>(outParts->size()))
                                MeshBuilder::append((*outParts)[mi], part);
                        }
                        for (const RenderMesh& part : tm.parts) {
                            const int mi = part.materialIndex;
                            if (mi >= 0 &&
                                mi < static_cast<int>(outParts->size()))
                                MeshBuilder::append((*outParts)[mi], part);
                        }
                    }
                    b.site = pb.center;
                    b.width = 2 * pb.half[0];
                    b.depth = 2 * pb.half[1];
                    b.yaw = std::atan2(pb.axis[0].y, pb.axis[0].x);
                    b.plan = plan;
                    b.height = (towerBase - b.baseY) + tm.height;
                    emitFoundation(b.plan, b.groundY, b.baseY);
                    out.push_back(std::move(b));
                    continue;
                }
            }
            if (planOk) {
                bm = growPlanBuilding(plan, bp, b.baseY);
                if (outFlatParts)
                    bmFlat = growPlanBuilding(plan, bp, b.baseY,
                                              FacadeDetail::Flat);
                OBB2 pb = orientedBoundingBox(plan);
                b.site = pb.center;
                b.width = 2 * pb.half[0];
                b.depth = 2 * pb.half[1];
                b.yaw = std::atan2(pb.axis[0].y, pb.axis[0].x);
                b.plan = plan;   // the collider prism follows the massing
            } else {
                // The box fallback fills the OBB: on a low-fill lot that IS
                // the overhanging-mass bug, so those go green instead.
                if (fill < 0.72) { dbg->rejBox++; emitGreen(); continue; }
                Scope scope = scopeFromFootprint(site, b.baseY, 10.0, clearOfRoads);
                const Real fitShort = std::min(scope.size.x, scope.size.z);
                // The urban sliver floor holds for the box path too: a
                // shrink-fit below minShortUrban is a knife blade, not a mass.
                if (fitShort < std::max(p.minShort * 0.75,
                                        std::min(p.minShort, p.minShortUrban))) {
                    dbg->rejBox++;
                    emitGreen(); continue;
                }
                Vec3 sc = scope.center();
                b.site = Vec2(sc.x, sc.z);
                b.width = scope.size.x;
                b.depth = scope.size.z;
                b.yaw = std::atan2(scope.axis[0].z, scope.axis[0].x);
                bm = growBuilding(scope, bp);
                if (outFlatParts)
                    bmFlat = growBuilding(scope, bp, FacadeDetail::Flat);
                // The box scope IS the plan here (shrunk-fit inside the lot).
                Vec2 r2(scope.axis[0].x, scope.axis[0].z);
                Vec2 f2(scope.axis[2].x, scope.axis[2].z);
                Vec2 o2(scope.origin.x, scope.origin.z);
                b.plan = {o2, o2 + r2 * scope.size.x,
                          o2 + r2 * scope.size.x + f2 * scope.size.z,
                          o2 + f2 * scope.size.z};
            }
            if (bm.parts.empty()) continue;
            mergeParts(outParts, bm);
            mergeParts(outFlatParts, bmFlat);
            b.height = bm.height > 0 ? bm.height : 8.0;
            emitFoundation(b.plan, b.groundY, b.baseY);
            // A yarded house earns its LANDSCAPING: front walk to the street,
            // a hedge along the front lot line, back-yard tree spots.
            if (yardApplied)
                sculptYard(b, lot.footprint, plan,
                           Vec2(bp.faceDir.x, bp.faceDir.z), b.groundY,
                           p.ground,
                           mix(pp.seed, static_cast<uint32_t>(li) * 29u + 11u),
                           outParts);
            out.push_back(std::move(b));
        }
    }
    if (underFwCount > 0)
        LOG_INFO << "[citylots] under-freeway lots: " << underFwCount
                 << " re-zoned to fit beneath the deck (open space / parking / "
                    "utility / short building)";
    // The DENSITY line (Glenn's "why did lots fail" dial): one glance says how
    // much of the parcelled city actually built and where the rest went.
    {
        int nBuilt = 0, nGreen = 0, nCourt = 0;
        for (const LotBuilding& lb : out) {
            if (lb.type == "green") ++nGreen;
            else if (lb.recipe == "court_green") ++nCourt;
            else if (lb.type != "park") ++nBuilt;
        }
        // The trailing "blocks all carriageway" is not a silent drop: it counts
        // faces whose interior was mostly roadway after the clear-push. A number
        // climbing there means the road graph is eating its own blocks — a road
        // bug, not a lot-tuning one.
        // COVERAGE, not just counts. "The blocks look empty" is a statement about
        // FOOTPRINT AREA, and the two come apart: a round that raised the lot count
        // while shrinking every lot moves this number the wrong way, and one that
        // built the same number of bigger buildings moves it the right way. Counts
        // alone sent us chasing the parcel grain twice (see commit d5db9a7).
        double builtArea = 0, blockArea = 0;
        for (const LotBuilding& lb : out)
            if (lb.type != "park" && lb.recipe != "court_green" &&
                lb.plan.size() >= 3)
                builtArea += std::fabs(area(lb.plan));
        for (const Poly2& b2 : dbg->blocks) blockArea += std::fabs(area(b2));
        const double coverPct =
            blockArea > 1.0 ? 100.0 * builtArea / blockArea : 0.0;
        LOG_INFO << "[citylots] " << dbg->blocks.size() << " blocks -> "
                 << dbg->lots.size() << " lots, " << nBuilt << " built, "
                 << nGreen << " green, " << nCourt << " courts | COVER "
                 << static_cast<int>(builtArea) << " m2 of "
                 << static_cast<int>(blockArea) << " m2 buildable ("
                 << (static_cast<int>(coverPct * 10) / 10.0) << "%) | rej: chance "
                 << dbg->rejChance << ", sliver " << dbg->rejSliver
                 << ", aspect " << dbg->rejAspect << ", fill " << dbg->rejFill
                 << ", plan " << dbg->rejPlan << ", clear " << dbg->rejClear
                 << ", box " << dbg->rejBox << ", frontage "
                 << dbg->rejFrontage << ", relief " << dbg->rejRelief
                 << " | alleys " << dbg->alleys.size()
                 << " | " << blocksAllCarriageway
                 << " blocks all carriageway";
    }
    return out;
}


namespace {
// Distance from p to segment [a,b].
Real segDist(const Vec2& p, const Vec2& a, const Vec2& b) {
    Vec2 ab = b - a;
    Real len2 = ab.lengthSquared();
    Real t = len2 > 1e-12 ? dot(p - a, ab) / len2 : 0.0;
    t = t < 0 ? 0 : (t > 1 ? 1 : t);
    Vec2 q(a.x + ab.x * t, a.y + ab.y * t);
    return (p - q).length();
}
}  // namespace

std::vector<Poly2> edgeBlocks(const RoadGraph& roads,
                              const std::vector<Poly2>& closedBlocks,
                              const EdgeBlockParams& p) {
    std::vector<Poly2> out;
    const int n = static_cast<int>(roads.nodes.size());
    if (n == 0) return out;

    // Adjacency + degree, then walk maximal CHAINS between non-degree-2 ends
    // (each chain is one road run between junctions / dead ends).
    std::vector<std::vector<std::pair<int, int>>> adj(n);   // node -> {edge, other}
    for (std::size_t ei = 0; ei < roads.edges.size(); ++ei) {
        const RoadEdge& e = roads.edges[ei];
        if (e.a < 0 || e.b < 0 || e.a >= n || e.b >= n || e.a == e.b) continue;
        adj[e.a].push_back({static_cast<int>(ei), e.b});
        adj[e.b].push_back({static_cast<int>(ei), e.a});
    }
    std::vector<uint8_t> used(roads.edges.size(), 0);

    // A candidate rectangle is kept only if its centre is truly OPEN ground:
    // inside no closed block, and no OTHER road passes near it.
    auto isOpen = [&](const Vec2& c) {
        for (const Poly2& b : closedBlocks)
            if (pointInPolygon(b, c)) return false;
        for (const RoadEdge& e : roads.edges) {
            if (e.a < 0 || e.b < 0 || e.a >= n || e.b >= n) continue;
            if (segDist(c, roads.nodes[e.a].pos, roads.nodes[e.b].pos) <
                p.margin + p.depth * 0.35)
                return false;   // some road runs through/near this ground
        }
        return true;
    };

    for (int start = 0; start < n; ++start) {
        if (adj[start].size() == 2) continue;   // chain interior, not an end
        for (const auto& [e0, n0] : adj[start]) {
            if (used[e0]) continue;
            // Walk the chain from `start` through degree-2 nodes.
            std::vector<Vec2> line{roads.nodes[start].pos};
            int prev = start, cur = n0, edge = e0;
            used[edge] = 1;
            line.push_back(roads.nodes[cur].pos);
            while (adj[cur].size() == 2) {
                const auto& [ea, na] = adj[cur][0];
                const auto& [eb, nb] = adj[cur][1];
                int nextEdge = (na == prev && !used[eb]) ? eb
                             : (nb == prev && !used[ea]) ? ea
                             : -1;
                if (nextEdge < 0) break;
                prev = cur;
                cur = roads.edges[nextEdge].a == cur ? roads.edges[nextEdge].b
                                                     : roads.edges[nextEdge].a;
                used[nextEdge] = 1;
                line.push_back(roads.nodes[cur].pos);
            }
            // Arc length; subdivide into pieces within [minLen, maxLen].
            Real total = 0;
            for (std::size_t i = 1; i < line.size(); ++i)
                total += (line[i] - line[i - 1]).length();
            if (total < p.minLen) continue;
            const int pieces = std::max(1, static_cast<int>(total / p.maxLen) + 
                                           (std::fmod(total, p.maxLen) > p.minLen ? 1 : 0));
            const Real pieceLen = total / pieces;
            // Point at arc-length s along the polyline.
            auto at = [&](Real s) {
                for (std::size_t i = 1; i < line.size(); ++i) {
                    Real seg = (line[i] - line[i - 1]).length();
                    if (s <= seg || i + 1 == line.size())
                        return line[i - 1] + (line[i] - line[i - 1]) *
                                                 (seg > 1e-9 ? s / seg : 0.0);
                    s -= seg;
                }
                return line.back();
            };
            for (int k = 0; k < pieces; ++k) {
                Vec2 a = at(k * pieceLen + 2.0);          // small end setbacks so
                Vec2 b = at((k + 1) * pieceLen - 2.0);    // neighbours don't touch
                Vec2 d = b - a;
                Real len = d.length();
                if (len < p.minLen * 0.6) continue;
                d = d / len;
                Vec2 nrm(-d.y, d.x);
                for (Real side : {Real(1), Real(-1)}) {
                    Vec2 c = (a + b) * 0.5 + nrm * side * (p.margin + p.depth * 0.5);
                    if (!isOpen(c)) continue;
                    Vec2 i0 = a + nrm * side * p.margin;
                    Vec2 i1 = b + nrm * side * p.margin;
                    Vec2 o1 = b + nrm * side * (p.margin + p.depth);
                    Vec2 o0 = a + nrm * side * (p.margin + p.depth);
                    Poly2 rect{i0, i1, o1, o0};
                    ensureCCW(rect);
                    // Rim blocks from DIFFERENT chains can land on the same open
                    // ground near a corner — two overlapping blocks grew two
                    // buildings through each other (device: "buildings
                    // intersecting with one another"). First-come wins; a rect
                    // overlapping an accepted one (SAT on the convex quads) is
                    // dropped.
                    bool overlaps = false;
                    const Poly2& rectRef = rect;
                    for (const Poly2& q : out) {
                        bool separated = false;
                        for (const Poly2* poly : {&rectRef, &q}) {
                            for (std::size_t ei = 0; ei < poly->size() && !separated; ++ei) {
                                Vec2 ed = (*poly)[(ei + 1) % poly->size()] - (*poly)[ei];
                                Vec2 ax(-ed.y, ed.x);
                                Real lo0 = 1e30, hi0 = -1e30, lo1 = 1e30, hi1 = -1e30;
                                for (const Vec2& v : rect) {
                                    Real t = dot(ax, v);
                                    lo0 = std::min(lo0, t); hi0 = std::max(hi0, t);
                                }
                                for (const Vec2& v : q) {
                                    Real t = dot(ax, v);
                                    lo1 = std::min(lo1, t); hi1 = std::max(hi1, t);
                                }
                                if (hi0 < lo1 || hi1 < lo0) separated = true;
                            }
                            if (separated) break;
                        }
                        if (!separated) { overlaps = true; break; }
                    }
                    if (overlaps) continue;
                    out.push_back(std::move(rect));
                }
            }
        }
    }
    return out;
}

NetLotResult growLotBuildingsOnNets(const std::vector<RoadEntity>& nets,
                                    const LotParams& params,
                                    const EdgeBlockParams& edgeParams,
                                    Real roadClearance,
                                    const RoadGroundFn& ground,
                                    const RoadGraph* freewayROW,
                                    bool wantFlatParts) {
    NetLotResult r;
    // One combined raw planar graph across every net (the sampled navRoadGraph
    // loses faces, so the block extraction uses the nets' own nodes/edges) —
    // plus the SAMPLED centrelines (what the asphalt is actually meshed from;
    // a curvy road bows off its control chords) with real per-edge widths, for
    // the building road-clearance check.
    RoadGraph rg, rgSampled;
    for (const RoadEntity& net : nets) {
        const int base = static_cast<int>(rg.nodes.size());
        for (const RoadNode& n : net.graph.nodes) rg.nodes.push_back({n.pos});
        for (const RoadEdge& e : net.graph.edges) {
            // #19 (semantic layer S6): block faces and rim-rectangle lots
            // derive from the WALKABLE STREET subgraph only. A baked corridor
            // edge (freeway/ramp) must never be a block boundary or a lot
            // frontage — else houses grow fronting the freeway with no street
            // to reach them (Glenn's "orphaned houses along the elevated
            // freeway"). The freeway ROW stays a keep-out band via rgSampled/
            // freewayROW below; it is simply not FRONTAGE.
            if (e.baked || e.klass == RoadClass::Freeway ||
                e.klass == RoadClass::Ramp)
                continue;   // corridor edge: not a block/lot source
            rg.edges.push_back(RoadEdge{base + e.a, base + e.b, e.width,
                                        RoadClass::Local, 0});
        }
        RoadGraph s = navRoadGraph(net, ground);
        const int sBase = static_cast<int>(rgSampled.nodes.size());
        for (const auto& n : s.nodes) rgSampled.nodes.push_back(n);
        for (const auto& e : s.edges)
            rgSampled.edges.push_back(RoadEdge{sBase + e.a, sBase + e.b,
                                               e.width, e.klass, e.layer});
    }
    // Keep buildings clear of the FREEWAY corridor, not just the streets
    // (device: "buildings and the freeway overlapping ... if the road graph was
    // true that wouldn't happen"). Feed the freeway right-of-way into the SAME
    // clearance graph the building pass reads (rgSampled) as wide edges, so the
    // city builds AROUND it (and, under a deck, re-zones — see the under-freeway
    // pass in growLotBuildings) instead of placing masses under/through it.
    int fwKeep = 0;
    if (freewayROW && !freewayROW->edges.empty()) {
        // PREFERRED: the real, routed ROW — dual carriageways AND ramps/gores,
        // from the unified graph. Each edge keeps its true class so the
        // under-deck re-zoning can tell freeway shadow from street frontage;
        // the keep width is the deck/ramp SHADOW (wider than the drivable lane).
        for (const RoadEdge& e : freewayROW->edges) {
            const bool ramp = e.klass == RoadClass::Ramp;
            const float keepW = ramp ? 14.0f : 24.0f;   // shadow half ~7 / ~12 m
            const int a = static_cast<int>(rgSampled.nodes.size());
            rgSampled.nodes.push_back({freewayROW->nodes[e.a].pos});
            const int b = static_cast<int>(rgSampled.nodes.size());
            rgSampled.nodes.push_back({freewayROW->nodes[e.b].pos});
            rgSampled.edges.push_back(RoadEdge{a, b, keepW, e.klass, 0});
            ++fwKeep;
        }
        LOG_INFO << "[citylots] freeway keep-out: " << fwKeep
                 << " ROW segments (carriageways + ramps) from the unified graph";
    } else {
        // FALLBACK: the mainline centreline proxy (net.freewayPlans) — used when
        // no corridor was routed (e.g. a level that authors only freewayPlans).
        const float kFreewayKeepWidth = 34.0f;   // deck + shoulders + a shy margin
        for (const RoadEntity& net : nets) {
            for (const std::vector<Vec2>& plan : net.plan.freewayPlans) {
                for (std::size_t i = 0; i + 1 < plan.size(); ++i) {
                    const int a = static_cast<int>(rgSampled.nodes.size());
                    rgSampled.nodes.push_back({plan[i]});
                    const int b = static_cast<int>(rgSampled.nodes.size());
                    rgSampled.nodes.push_back({plan[i + 1]});
                    rgSampled.edges.push_back(
                        RoadEdge{a, b, kFreewayKeepWidth, RoadClass::Freeway, 0});
                    ++fwKeep;
                }
            }
        }
        LOG_INFO << "[citylots] freeway keep-out: " << fwKeep
                 << " mainline-proxy segments (no routed ROW)";
    }
    std::vector<Poly2> blocks = extractBlocks(rg);
    // Rim blocks: the town edge has no enclosed faces — synthesize rectangles
    // on the boundary roads' open sides so the outskirts build up too.
    std::vector<Poly2> rim = edgeBlocks(rg, blocks, edgeParams);
    blocks.insert(blocks.end(), rim.begin(), rim.end());
    // Per-hub-cluster landmark quotas (8km-city P3): the loader forwards hubs
    // as {pos, kind} only, so recover each hub's SITE id (0 = the primary
    // city, 1+ = satellite towns) from the nets' own cityHubs by position —
    // both lists come from the same CityHub records. A caller that already
    // filled hubClusters keeps its own mapping.
    LotParams lp = params;
    if (!lp.hubs.empty() && lp.hubClusters.size() != lp.hubs.size()) {
        lp.hubClusters.assign(lp.hubs.size(), 0);
        for (std::size_t i = 0; i < lp.hubs.size(); ++i) {
            bool found = false;
            for (const RoadEntity& net : nets) {
                for (const CityHub& h : net.plan.cityHubs)
                    if ((h.pos - lp.hubs[i].first).lengthSquared() < 1e-6) {
                        lp.hubClusters[i] = h.site;
                        found = true;
                        break;
                    }
                if (found) break;
            }
        }
    }
    r.lots = growLotBuildings(blocks, lp, &r.plan, &r.parts, &rgSampled,
                              roadClearance,
                              wantFlatParts ? &r.flatParts : nullptr,
                              &r.gradeFlatten);
    return r;
}

void appendLotMassBox(RenderMesh& out, const LotBuilding& lot,
                      const Vec3& sideColor, const Vec3& roofColor) {
    const Real hw = lot.width * 0.5, hd = lot.depth * 0.5;
    const Vec3 ax(std::cos(lot.yaw), 0, std::sin(lot.yaw));
    const Vec3 az(-std::sin(lot.yaw), 0, std::cos(lot.yaw));
    const Vec3 base(lot.site.x, lot.baseY, lot.site.y);
    const Vec3 top = base + Vec3(0, lot.height, 0);
    // Corners run CCW in XZ: (-,-) -> (+,-) -> (+,+) -> (-,+).
    const Vec3 c[4] = {ax * -hw + az * -hd, ax * hw + az * -hd,
                       ax * hw + az * hd, ax * -hw + az * hd};
    for (int e = 0; e < 4; ++e) {
        const Vec3 &a = c[e], &b = c[(e + 1) % 4];
        // OUTWARD normal of edge a->b. For this CCW winding that is the RIGHT
        // normal (d.z, 0, -d.x); the LEFT normal (-d.z, 0, d.x) points back into
        // the box. Getting this backwards builds every distant building
        // inside-out — and it hides from the facing debug view, because emitQuad
        // winds geometry to agree with the normal it is handed, so the near wall
        // is culled and you see the far wall with its flipped normal aimed at you.
        const Vec3 d = b - a;
        const Vec3 n = normalize(Vec3(d.z, 0, -d.x));
        MeshBuilder::emitQuad(out, base + a, base + b, top + b, top + a, n, sideColor);
    }
    MeshBuilder::emitQuad(out, top + c[0], top + c[1], top + c[2], top + c[3],
                          Vec3(0, 1, 0), roofColor);
}

}  // namespace engine
