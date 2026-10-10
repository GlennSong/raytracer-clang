#include "city_lots.h"
#include "trades.h"

#include <unordered_map>
#include <memory>
#include <functional>
#include "parcel.h"          // subdivideBlock, Lot, ParcelParams
#include "roads/road_entity.h"        // RoadEntity + navRoadGraph (growLotBuildingsOnNets)
#include "road_network.h"    // RoadGraph (edge blocks walk its chains)
#include "architect.h"       // DistrictMap + archetype tables (the architect pass)
#include "shape_grammar.h"   // scopeFromFootprint, growBuilding — REAL buildings
#include "road_mesh.h"       // triangulatePolygon (lot-shaped park pads)
#include "street_kit.h"      // streetLamp (plaza lamp posts)
#include "block_grade.h"     // gradeBlocks (in-pass block terracing)
#include "site_plan.h"       // siteFrame + largestAlignedRect: the rectilinear buildable
#include "core_plan.h"       // coreFor: which tall buildings open (M5)
#include "../furniture_kit.h"   // Piece: outdoor library furniture (M2)
#include "../../../log.h"    // plaza site report (find them on the map)
#include "../../mesh_builder.h"   // MeshBuilder::append (merge parts by PartId)
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
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


namespace {
// A stable hash of a point (an outdoor piece's variant, without drawing from a pass's own dice).
uint32_t posHash(const Vec2& p) {
    uint32_t h = static_cast<uint32_t>(std::lround(p.x * 13.0)) * 73856093u ^ static_cast<uint32_t>(std::lround(p.y * 7.0)) * 19349663u;
    h ^= h >> 13; h *= 0x5bd1e995u; h ^= h >> 15;
    return h;
}
// An outdoor library piece at `at` (its back's centre), its front facing `face`; the variant's wood from `h`.
OutdoorPiece outdoorPiece(Piece pc, uint32_t h, const Vec2& at, const Vec2& face, Real y, bool draped) {
    OutdoorPiece o;
    o.piece = static_cast<uint8_t>(pc);
    o.variant = (h % 4u) << 3;   // the wood (furniture_kit.h variant bits 3-4)
    o.at = at;
    o.yaw = std::atan2(face.x, face.y);
    o.y = y;
    o.draped = draped;
    return o;
}
}  // namespace

// Merge a grown kit's parts into the by-PartId output array.
void appendKit(const BuildingMesh& kit, std::vector<RenderMesh>* outParts,
               bool draped = false) {
    if (!outParts) return;
    for (const RenderMesh& part : kit.parts) {
        const int mi = part.materialIndex;
        if (mi < 0) continue;
        const std::size_t slot = draped ? drapedSlot(static_cast<PartId>(mi))
                                        : static_cast<std::size_t>(mi);
        if (slot < outParts->size()) MeshBuilder::append((*outParts)[slot], part);
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
                std::vector<RenderMesh>* outParts,
                std::vector<TerrainFlatten>* outFlatten = nullptr,
                Real meshCell = 3.0) {
    const Vec3 grass(0.30, 0.50, 0.26);
    const Vec3 pathCol(0.72, 0.68, 0.60);        // decomposed granite
    // GROUND-RELATIVE (kDrapedPartBase): everything this emits — the padMesh and
    // the furnishing kit — is measured from the ground under it, and the host
    // drapes it on the finished terrain. Only the path RAMPS, which grade the
    // terrain itself, need the absolute ground.
    auto gy = [](const Vec2&) { return Real(0); };
    auto gyAbs = [&](const Vec2& v) { return ground ? ground(v.x, v.y) : Real(0); };
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
    // paths stand just off the lawn: a kerb's height at most, whatever the lot's own pad height (a block park's
    // 0.25 m put its plaza and walks 0.28 m up -- a step, not a path)
    const Real py = std::min(h, Real(0.18)) + 0.03;
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

    g.sealed.push_back(Poly2(rim, rim + PN));   // the plaza: no grass on it
    {   // THE LAWN as an activity area (a picnic, catch, a chat, the sun): the park's box drawn in off its edges
        const OBB2 ob = orientedBoundingBox(poly);
        const int la = ob.longAxis();
        LotBuilding::Area ar;
        ar.kind = "lawn";
        ar.center = ob.center;
        ar.axis = ob.axis[la];
        ar.halfL = ob.half[la] - 3.0;
        ar.halfW = ob.half[1 - la] - 3.0;
        if (ar.halfL > 4.0 && ar.halfW > 3.0 && A > 300.0) g.areas.push_back(ar);
    }
    for (int k = 0; k < PN; ++k)                // ...and a ring across it for the walkers' network
        g.walks.push_back({c + (rim[k] - c) * 0.6, c + (rim[(k + 1) % PN] - c) * 0.6, 1.2});
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
        // Segment length tied to the RENDERED terrain cell (walkway-lab
        // round, device: "the walkway itself needs more geometry so it can
        // conform"): the tile is piecewise-bilinear per cell, so a strip can
        // only follow it if it has a joint inside every cell it crosses. A
        // fixed 3 m span bridged the bends of metro's 2.7 m cells.
        const Real segLen =
            std::min(Real(3.0), std::max(Real(1.2), meshCell * Real(0.75)));
        const int segs =
            std::max(1, static_cast<int>((len - r0 * 0.85) / segLen));
        {   // sealed: no grass on the walk; and a line for the walkers' network, from the plaza's ring to the street
            const Vec2 pe = p0 + dir * (len - r0 * 0.85);
            g.sealed.push_back({p0 - perp * hw, p0 + perp * hw, pe + perp * hw, pe - perp * hw});
            g.walks.push_back({c + dir * (r0 * 0.6), pe, hw * 2});
        }
        for (int s = 0; s < segs; ++s) {
            Vec2 q0 = p0 + dir * ((len - r0 * 0.85) * s / segs);
            Vec2 q1 = p0 + dir * ((len - r0 * 0.85) * (s + 1) / segs);
            const Vec3 L0(q0.x - perp.x * hw, gy(q0 - perp * hw) + py, q0.y - perp.y * hw);
            const Vec3 R0(q0.x + perp.x * hw, gy(q0 + perp * hw) + py, q0.y + perp.y * hw);
            const Vec3 L1(q1.x - perp.x * hw, gy(q1 - perp * hw) + py, q1.y - perp.y * hw);
            const Vec3 R1(q1.x + perp.x * hw, gy(q1 + perp * hw) + py, q1.y + perp.y * hw);
            MeshBuilder::emitQuad(m, L0, R0, R1, L1, Vec3(0, 1, 0), pathCol);
            // The path STAMPS its band into the terrain flatten set: the mesh
            // samples the analytic ground at its corners, but the RENDERED
            // CDLOD tile is free to disagree between samples — coarse LODs
            // bulged natural ground through (or dropped it out from under)
            // the thin ribbon, which is where the surviving "floating
            // walkway" sightings lived after the grade-ordering fix. A ramp
            // per segment grades every LOD to the path's own plane — the
            // same anti-straddle answer road corridors use.
            if (outFlatten)
                outFlatten->push_back(makeFlattenRamp(
                    Vec3(q0.x, 0, q0.y), Vec3(q1.x, 0, q1.y),
                    gyAbs(q0) + py - 0.05, gyAbs(q1) + py - 0.05, hw + 0.6, 2.5));
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
                latheMesh(Vec3(c.x, fy, c.y), water, 14, Vec3(0.036, 0.092, 0.136)));
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
            // A PARK BENCH from the library (M2: sat on, lain on), facing the fountain, on the terrain.
            (void)wood;
            g.furniture.push_back(outdoorPiece(Piece::Bench, posHash(bp), bp + dir * 0.33, Vec2(-dir.x, -dir.y), 0.02, true));
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
                emitSoftBox(kit, Scope{o + Vec3(0, 0.35, 0) + t3 * 0.1 + n3 * 0.1,
                                       {t3, up, n3}, Vec3(0.8, 0.55, 0.8)},
                            PartId::Foliage, hedge, 0u);
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
            emitSoftBox(kit, Scope{o + Vec3(0, 0.22, 0) + t3 * 0.12 + n3 * 0.12,
                                   {t3, up, n3}, Vec3(0.86, 0.18, 0.86)},
                        PartId::Foliage, bloom[static_cast<int>(rng.unit() * 2.99)], 0u);
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
            emitSoftBox(kit, Scope{Vec3(sp.x, by, sp.y) - t3 * (sw * 0.5) - n3 * (sw * 0.5),
                                   {t3, up, n3}, Vec3(sw, sh, sw)},
                        PartId::Foliage, bush, 0u);
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
        appendKit(kit, outParts, /*draped=*/true);
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

// THE QUAD (campus gap pass; Glenn: "make it look cool"): the green between the halls as a collegiate quad, not a
// park -- a paved round at its centre with a FOUNTAIN, a broad stone walk down its length and one across it (out
// through the gaps between the halls to the streets), diagonal walks to its corners, an avenue of trees down the
// long walk, lamps between them, benches round the fountain and along the walk, flower beds in the round's corners.
// No fence. Ground-relative like a park; the walks and the round are SEALED (no grass on them).
static void sculptQuad(LotBuilding& g, const Poly2& poly, uint32_t seed, std::vector<RenderMesh>* outParts, Real meshCell,
                       const std::function<bool(const Vec2&)>& street, bool crossWalks,
                       const std::vector<std::pair<Vec2, Vec2>>& hallDoors) {
    const OBB2 ob = orientedBoundingBox(poly);
    const int la = ob.longAxis();
    const Vec2 ua = ob.axis[la], va(ua.y, -ua.x), c = ob.center;
    const Real hl = ob.half[la], hw = ob.half[1 - la];
    Hash rng(mix(seed, 0x0A0Du));
    RenderMesh m;
    BuildingMesh kit;
    const Vec3 up(0, 1, 0);
    // (a weathered limestone: the first, lighter cut read as paper in the sun)
    const Vec3 stone(0.56, 0.53, 0.48), stoneDark(0.42, 0.40, 0.37), water(0.16, 0.27, 0.32);
    const Real py = 0.05;   // the walks stand just proud of the lawn
    auto Q = [&](Real u, Real v) { return c + ua * u + va * v; };
    auto inside = [&](const Vec2& q) { return pointInPolygon(poly, q); };
    const Real segLen = std::min(Real(3.0), std::max(Real(1.2), meshCell * Real(0.75)));
    // a walk from a to b, `w` wide: a ribbon jointed every terrain cell (it is draped), sealed
    auto walk = [&](Vec2 a, Vec2 b, Real w, const Vec3& col) {
        const Vec2 d = b - a;
        const Real L = d.length();
        if (L < 0.5) return;
        const Vec2 dir = d * (1.0 / L), perp(-dir.y, dir.x);
        const int n = std::max(1, static_cast<int>(L / segLen));
        for (int i = 0; i < n; ++i) {
            const Vec2 q0 = a + dir * (L * i / n), q1 = a + dir * (L * (i + 1) / n);
            MeshBuilder::emitQuad(m, Vec3(q0.x - perp.x * w / 2, py, q0.y - perp.y * w / 2), Vec3(q0.x + perp.x * w / 2, py, q0.y + perp.y * w / 2),
                                  Vec3(q1.x + perp.x * w / 2, py, q1.y + perp.y * w / 2), Vec3(q1.x - perp.x * w / 2, py, q1.y - perp.y * w / 2), up, col);
        }
        g.sealed.push_back({a - perp * (w / 2), a + perp * (w / 2), b + perp * (w / 2), b - perp * (w / 2)});
        g.walks.push_back({a, b, w});
    };
    // a paved disc, sealed -- in RINGS a terrain cell apart: it is draped, and a fan from its centre to a 7 m rim
    // hung its middle 0.4 m over a sloping courtyard
    auto discInto = [&](RenderMesh& m, const Vec2& o, Real r, Real y, const Vec3& col, bool seal) {
        const int n = 32, rings = std::max(1, static_cast<int>(std::ceil(r / segLen)));
        Poly2 rimP;
        for (int k = 0; k < n; ++k) {
            const Real a0 = 6.2831853 * k / n, a1 = 6.2831853 * (k + 1) / n;
            const Vec2 d0(std::cos(a0), std::sin(a0)), d1(std::cos(a1), std::sin(a1));
            for (int ri = 0; ri < rings; ++ri) {
                const Real r0 = r * ri / rings, r1 = r * (ri + 1) / rings;
                const Vec2 p00 = o + d0 * r0, p01 = o + d1 * r0, p10 = o + d0 * r1, p11 = o + d1 * r1;
                if (ri == 0) MeshBuilder::emitTri(m, Vec3(o.x, y, o.y), Vec3(p10.x, y, p10.y), Vec3(p11.x, y, p11.y), up, col);
                else MeshBuilder::emitQuad(m, Vec3(p00.x, y, p00.y), Vec3(p10.x, y, p10.y), Vec3(p11.x, y, p11.y), Vec3(p01.x, y, p01.y), up, col);
            }
            rimP.push_back(o + d0 * r);
        }
        if (seal) g.sealed.push_back(rimP);
    };
    auto disc = [&](const Vec2& o, Real r, Real y, const Vec3& col, bool seal) { discInto(m, o, r, y, col, seal); };
    auto box = [&](const Vec2& at, Real y, const Vec2& along, const Vec3& size, PartId part, const Vec3& col) {
        const Vec3 t(along.x, 0, along.y), n(-along.y, 0, along.x);
        emitBox(kit, Scope{Vec3(at.x, y, at.y) - t * (size.x * 0.5) - n * (size.z * 0.5), {t, up, n}, size}, part, col);
    };

    // THE ROUND and its FOUNTAIN
    const Real R = std::clamp(std::min(hw, hl) * 0.42, Real(5.5), Real(8.5));
    disc(c, R, py + 0.005, stone, true);
    disc(c, R - 0.5, py + 0.01, stoneDark * 1.08, false);   // a darker inlaid band ...
    disc(c, R - 0.9, py + 0.015, stone, false);             // ... inside the round's edge
    const Real fr = R * 0.42;   // the basin
    for (int k = 0; k < 28; ++k) {   // its rim: stone blocks round a circle
        const Real a = 6.2831853 * (k + 0.5) / 28;
        const Vec2 d(std::cos(a), std::sin(a));
        box(c + d * fr, py, Vec2(-d.y, d.x), Vec3(6.2831853 * fr / 28 + 0.04, 0.48, 0.38), PartId::Trim, stone * 0.95);
    }
    // the water: with the fountain's stone (Trim), not the walks' surface -- 0.36 m up inside its rim it is not a
    // walk standing proud of the lawn
    RenderMesh pool;
    discInto(pool, c, fr - 0.18, py + 0.36, water, false);
    // THE RIM is a place to sit too (a hip on the stone, looking out): the citysim's seats like the benches'
    for (int k = 0; k < 12; ++k) {
        const Real a = 6.2831853 * (k + 0.25) / 12;
        const Vec2 d(std::cos(a), std::sin(a));
        g.sitSpots.push_back({c + d * (fr + 0.12), d, py + 0.48});
    }
    box(c, py, ua, Vec3(1.3, 0.55, 1.3), PartId::Trim, stone * 0.9);          // the plinth
    box(c, py + 0.55, ua, Vec3(0.5, 1.35, 0.5), PartId::Trim, stone);        // the column
    box(c, py + 1.9, ua, Vec3(1.9, 0.16, 1.9), PartId::Trim, stone * 0.95);   // the upper bowl ...
    box(c, py + 2.06, ua, Vec3(1.6, 0.03, 1.6), PartId::Trim, water * 1.3);   // ... its water
    box(c, py + 2.06, ua, Vec3(0.22, 0.55, 0.22), PartId::Trim, stone);      // the finial

    // the ROUND as a walk too (for the walkers' network): a ring of chords where the walks meet it
    {
        const int n = 12;
        for (int k = 0; k < n; ++k) {
            const Real a0 = 6.2831853 * k / n, a1 = 6.2831853 * (k + 1) / n;
            g.walks.push_back({c + Vec2(std::cos(a0), std::sin(a0)) * (R - 0.6), c + Vec2(std::cos(a1), std::sin(a1)) * (R - 0.6), 1.2});
        }
    }
    // THE WALKS run on past the quad until they meet the SIDEWALK (Glenn: "those ribbons should go to the edges where
    // it meets the sidewalk"): from `from` along `dir`, at least to `minLen`, then on in half-metre steps until the
    // next would be street, 30 m at most
    auto toStreet = [&](const Vec2& from, const Vec2& dir, Real minLen) {
        Real t = minLen;
        if (street)
            while (t < minLen + 30.0 && !street(from + dir * (t + 0.5))) t += 0.5;
        return from + dir * t;
    };
    const Real wMain = 3.0, wDiag = 2.0;
    // down the length, and across through the halls' gap -- not in a dorm block's courtyard, whose halls run its whole
    // length (the walks went through them)
    for (const Vec2& d : {ua, ua * -1.0, va, va * -1.0}) {
        if (!crossWalks && std::fabs(dot(d, va)) > 0.5) continue;
        const Vec2 from = c + d * (R - 0.3);
        const Real span = std::fabs(dot(d, ua)) > 0.5 ? hl : hw;
        walk(from, toStreet(from, d, span - (R - 0.3)), wMain, stone);
    }
    // THE FAN (Glenn liked it): two walks each way from the round, spreading toward the quad's ends -- on a long
    // narrow quad a sunburst about the long walk -- out to the street there
    std::vector<std::pair<Vec2, Vec2>> diags;
    for (int su : {-1, 1})
        for (int sv : {-1, 1}) {
            const Vec2 aim = Q(su * hl, sv * hw * 0.65);
            const Vec2 dir = normalize(aim - c);
            const Vec2 from = c + dir * (R - 0.3);
            const Vec2 to = toStreet(from, dir, (aim - from).length());
            walk(from, to, wDiag, stone * 0.97);
            diags.push_back({from, to});
        }
    // TO EVERY HALL'S QUAD DOOR (its back door, in the middle of the wall facing the quad): a walk straight out from
    // the door to the long walk
    for (const auto& hd : hallDoors) {
        const Vec2 foot = hd.first + hd.second * 0.4, axis = c + ua * dot(hd.first - c, ua);
        const Vec2 d = axis - foot;
        if (d.length() < 2.0 || d.length() > hw + 30.0) continue;
        const Vec2 dir = normalize(d);
        walk(foot, axis - dir * (wMain / 2 - 0.1), wDiag, stone * 0.97);
        g.walks.back().streetEnds = false;
        diags.push_back({foot, axis});
    }
    auto offWalks = [&](const Vec2& q, Real clear) {
        if ((q - c).length() < R + clear) return false;
        if (std::fabs(dot(q - c, va)) < wMain / 2 + clear) return false;   // the long walk
        if (crossWalks && std::fabs(dot(q - c, ua)) < wMain / 2 + clear) return false;   // the cross walk
        for (const auto& d : diags) if (distToSeg(q, d.first, d.second) < wDiag / 2 + clear) return false;
        return inside(q);
    };

    // BENCHES round the fountain, between the walks, facing it
    for (int k = 0; k < 16; ++k) {
        const Real a = 6.2831853 * (k + 0.5) / 16;
        const Vec2 d(std::cos(a), std::sin(a));
        const Vec2 bp = c + d * (R - 0.75);
        bool clear = std::fabs(dot(bp - c, va)) > wMain / 2 + 1.1 && (!crossWalks || std::fabs(dot(bp - c, ua)) > wMain / 2 + 1.1);
        for (const auto& dg : diags) clear = clear && distToSeg(bp, dg.first, dg.second) > wDiag / 2 + 1.1;
        if (clear) g.furniture.push_back(outdoorPiece(Piece::Bench, posHash(bp) ^ seed, bp + d * 0.33, Vec2(-d.x, -d.y), 0.02, true));
    }
    // FLOWER BEDS in the round's corners, just outside it between the walks
    const Vec3 bloom[4] = {{0.72, 0.22, 0.26}, {0.82, 0.68, 0.20}, {0.56, 0.32, 0.66}, {0.90, 0.88, 0.84}};
    for (int k = 0; k < 8; ++k) {
        const Real a = 6.2831853 * (k + 0.5) / 8;
        const Vec2 d(std::cos(a), std::sin(a));
        const Vec2 fp = c + d * (R + 1.6);
        if (!offWalks(fp, 1.0)) continue;
        const Vec2 t(-d.y, d.x);
        box(fp, 0.0, t, Vec3(2.6, 0.22, 1.1), PartId::Trim, stone * 0.9);
        emitSoftBox(kit, Scope{Vec3(fp.x, 0.22, fp.y) - Vec3(t.x, 0, t.y) * 1.2 - Vec3(d.x, 0, d.y) * 0.45,
                               {Vec3(t.x, 0, t.y), up, Vec3(d.x, 0, d.y)}, Vec3(2.4, 0.3, 0.9)},
                    PartId::Foliage, bloom[(k + (rng.next() & 3u)) & 3u], 0u);
    }
    // THE AVENUE: trees down both sides of the long walk, lamps between them, benches facing the walk
    const Real off = wMain / 2 + 2.2;
    // THE LAWNS (activity areas: a picnic, a game of catch, a circle chatting, someone in the sun): the four quarters
    // past the round, out beyond the avenue's trees
    {
        const Real l0 = R + 2.5, l1 = hl - 2.0, w0 = off + 1.8, w1 = hw - 1.5;
        if (l1 - l0 > 6.0 && w1 - w0 > 4.0)
            for (int su : {-1, 1})
                for (int sv : {-1, 1}) {
                    LotBuilding::Area ar;
                    ar.kind = "lawn";
                    ar.center = Q(su * (l0 + l1) * 0.5, sv * (w0 + w1) * 0.5);
                    ar.axis = ua;
                    ar.halfL = (l1 - l0) * 0.5;
                    ar.halfW = (w1 - w0) * 0.5;
                    g.areas.push_back(ar);
                }
    }
    const Real step = 8.0;
    int idx = 0;
    for (Real u = R + 4.0; u < hl - 2.5; u += step, ++idx)
        for (int su : {-1, 1})
            for (int sv : {-1, 1}) {
                const Vec2 tp = Q(su * u, sv * off);
                if (offWalks(tp, 1.2)) g.treeSpots.push_back(Vec3(tp.x, rng.range(1.0, 1.25), tp.y));
                const Vec2 mid = Q(su * (u + step / 2), sv * (off - 0.4));
                if (u + step / 2 > hl - 3.0 || !offWalks(mid, 0.8)) continue;
                if (idx % 2 == 0) {   // a lamp
                    box(mid, 0.0, ua, Vec3(0.14, 3.4, 0.14), PartId::Metal, Vec3(0.07, 0.07, 0.08));
                    box(mid, 3.4, ua, Vec3(0.34, 0.46, 0.34), PartId::Trim, Vec3(0.95, 0.88, 0.66));
                    box(mid, 3.86, ua, Vec3(0.44, 0.06, 0.44), PartId::Metal, Vec3(0.07, 0.07, 0.08));
                } else {              // a bench, its back to the trees, facing the walk
                    const Vec2 face = va * static_cast<Real>(-sv);
                    g.furniture.push_back(outdoorPiece(Piece::Bench, posHash(mid) ^ seed, mid - face * 0.33, face, 0.02, true));
                }
            }
    // a few trees out on the lawns, clear of the walks
    for (int k = 0; k < 40 && g.treeSpots.size() < 40; ++k) {
        const Vec2 tp = Q((rng.unit() * 2 - 1) * (hl - 3), (rng.unit() * 2 - 1) * (hw - 3));
        if (!offWalks(tp, 2.5)) continue;
        bool clear = true;
        for (const Vec3& o : g.treeSpots) clear = clear && (Vec2(o.x, o.z) - tp).length() > 7.0;
        if (clear && rng.unit() < 0.5) g.treeSpots.push_back(Vec3(tp.x, rng.range(0.9, 1.3), tp.y));
    }
    appendKit(kit, outParts, /*draped=*/true);
    if (outParts) {
        const std::size_t slot = drapedSlot(PartId::Trim);
        if (slot < outParts->size()) MeshBuilder::append((*outParts)[slot], pool);
    }
    g.padMesh = std::move(m);
    g.color = Vec3(1, 1, 1);
}

// THE PASEO (~/.claude/plans/nightlife-and-malls.md stage 3; Glenn: "outdoor pedestrian walkways like they do at these
// upscale malls"): the open-air mall's car-free walk between its two rows of shops. Paved end to end in banded stone;
// down each side a row of trees in planters with benches between; overhead, strings of lights zigzagging pole to pole
// across it; a fountain in a round at the middle. Its walks: down the middle, along each row of shopfronts, and out
// through every PASSAGE between the buildings (`passages`: their positions along the paseo) to the street.
static void sculptPaseo(LotBuilding& g, const Poly2& poly, uint32_t seed, std::vector<RenderMesh>* outParts, Real meshCell,
                        const std::function<bool(const Vec2&)>& street, const std::vector<Real>& passages) {
    const OBB2 ob = orientedBoundingBox(poly);
    const int la = ob.longAxis();
    const Vec2 ua = ob.axis[la], va(ua.y, -ua.x), c = ob.center;
    const Real hl = ob.half[la], hw = ob.half[1 - la];
    Hash rng(mix(seed, 0x9A5E0u));
    RenderMesh m;
    BuildingMesh kit;
    const Vec3 up(0, 1, 0);
    // (contrasty: the sun lifts pale stone to one white sheet -- the first cut, 0.66/0.55/0.42, read as plain concrete)
    const Vec3 paveA(0.60, 0.52, 0.42), paveB(0.44, 0.38, 0.32), paveC(0.26, 0.23, 0.21), water(0.16, 0.27, 0.32);
    const Real py = 0.06;
    auto Q = [&](Real u, Real v) { return c + ua * u + va * v; };
    const Real segLen = std::min(Real(3.0), std::max(Real(1.2), meshCell * Real(0.75)));
    auto box = [&](const Vec2& at, Real y, const Vec2& along, const Vec3& size, PartId part, const Vec3& col) {
        const Vec3 t(along.x, 0, along.y), n(-along.y, 0, along.x);
        emitBox(kit, Scope{Vec3(at.x, y, at.y) - t * (size.x * 0.5) - n * (size.z * 0.5), {t, up, n}, size}, part, col);
    };
    // THE PAVING: bands across the walk -- two stones, a dark course every fourth -- jointed a terrain cell at a
    // time along the bands (it is draped), a darker border along each row's shopfronts
    {
        const Real band = 1.2;
        int k = 0;
        for (Real u = -hl; u < hl - 1e-3; u += band, ++k) {
            const Real u1 = std::min(hl, u + band);
            const Vec3 col = k % 4 == 3 ? paveC : (k & 1) ? paveB : paveA;
            const int nv = std::max(1, static_cast<int>(std::ceil(2 * hw / segLen)));
            for (int j = 0; j < nv; ++j) {
                const Real v0 = -hw + 2 * hw * j / nv, v1 = -hw + 2 * hw * (j + 1) / nv;
                const bool edge = v0 < -hw + 0.8 || v1 > hw - 0.8;
                const Vec2 a = Q(u, v0), b = Q(u1, v0), cc = Q(u1, v1), d = Q(u, v1);
                MeshBuilder::emitQuad(m, Vec3(a.x, py, a.y), Vec3(b.x, py, b.y), Vec3(cc.x, py, cc.y), Vec3(d.x, py, d.y), up,
                                      edge ? paveC * 1.1 : col);
            }
        }
        g.sealed.push_back(poly);
    }
    // THE WALKS (the walkers' network): down the middle and along each row of fronts, to the street at both ends
    auto toStreet = [&](const Vec2& from, const Vec2& dir, Real minLen) {
        Real t = minLen;
        if (street) while (t < minLen + 30.0 && !street(from + dir * (t + 0.5))) t += 0.5;
        return from + dir * t;
    };
    auto walkLine = [&](Vec2 a, Vec2 b, Real w, bool paint) {
        g.walks.push_back({a, b, w});
        if (!paint) return;
        // out past the paseo (an end, a passage): paved too, a ribbon jointed every terrain cell
        const Vec2 d = b - a;
        const Real L = d.length();
        if (L < 0.5) return;
        const Vec2 dir = d * (1.0 / L), perp(-dir.y, dir.x);
        const int n = std::max(1, static_cast<int>(L / segLen));
        for (int i = 0; i < n; ++i) {
            const Vec2 q0 = a + dir * (L * i / n), q1 = a + dir * (L * (i + 1) / n);
            MeshBuilder::emitQuad(m, Vec3(q0.x - perp.x * w / 2, py, q0.y - perp.y * w / 2), Vec3(q0.x + perp.x * w / 2, py, q0.y + perp.y * w / 2),
                                  Vec3(q1.x + perp.x * w / 2, py, q1.y + perp.y * w / 2), Vec3(q1.x - perp.x * w / 2, py, q1.y - perp.y * w / 2),
                                  up, (i & 1) ? paveA : paveB);
        }
        g.sealed.push_back({a - perp * (w / 2), a + perp * (w / 2), b + perp * (w / 2), b - perp * (w / 2)});
    };
    for (Real v : {Real(0), hw - 1.4, -(hw - 1.4)}) {
        walkLine(Q(-hl, v), Q(hl, v), v == 0 ? 4.0 : 2.4, false);
        for (int su : {-1, 1}) {
            const Vec2 from = Q(su * hl, v);
            walkLine(from, toStreet(from, ua * static_cast<Real>(su), 0.5), v == 0 ? 4.0 : 2.4, true);
        }
    }
    // THE PASSAGES out to the streets between the buildings, paved through
    for (Real pu : passages)
        for (int sv : {-1, 1}) {
            const Vec2 from = Q(pu, sv * (hw - 0.2));
            walkLine(Q(pu, 0), from, 3.0, false);
            walkLine(from, toStreet(from, va * static_cast<Real>(sv), 8.0), 4.5, true);
        }
    auto clearOfPassages = [&](Real u, Real margin) {
        for (Real pu : passages) if (std::fabs(u - pu) < 3.0 + margin) return false;
        return true;
    };
    // THE ROUND at the middle and its FOUNTAIN (a low basin you can sit on the rim of)
    const Real R = std::clamp(hw * 0.75, Real(4.0), Real(5.5));
    {
        const int n = 32;
        for (int k = 0; k < n; ++k) {
            const Real a0 = 6.2831853 * k / n, a1 = 6.2831853 * (k + 1) / n;
            const Vec2 p0 = c + Vec2(std::cos(a0), std::sin(a0)) * R, p1 = c + Vec2(std::cos(a1), std::sin(a1)) * R;
            MeshBuilder::emitTri(m, Vec3(c.x, py + 0.01, c.y), Vec3(p0.x, py + 0.01, p0.y), Vec3(p1.x, py + 0.01, p1.y), up, paveC * 1.15);
        }
        const Real fr = R * 0.55;
        for (int k = 0; k < 24; ++k) {
            const Real a = 6.2831853 * (k + 0.5) / 24;
            const Vec2 d(std::cos(a), std::sin(a));
            box(c + d * fr, py, Vec2(-d.y, d.x), Vec3(6.2831853 * fr / 24 + 0.04, 0.46, 0.36), PartId::Trim, paveA * 0.95);
        }
        for (int k = 0; k < 10; ++k) {
            const Real a = 6.2831853 * (k + 0.25) / 10;
            const Vec2 d(std::cos(a), std::sin(a));
            g.sitSpots.push_back({c + d * (fr + 0.12), d, py + 0.46});
        }
        box(c, py + 0.30, ua, Vec3(2 * fr - 0.3, 0.02, 2 * fr - 0.3), PartId::Trim, water);
        box(c, py, ua, Vec3(0.9, 0.9, 0.9), PartId::Trim, paveA * 0.9);
        box(c, py + 0.9, ua, Vec3(1.5, 0.12, 1.5), PartId::Trim, paveA);
        box(c, py + 1.02, ua, Vec3(1.2, 0.02, 1.2), PartId::Trim, water * 1.3);
    }
    // THE TREES in square planters down each side (between the fronts' walk and the middle), BENCHES between them
    // facing across, the STRING LIGHTS' poles at the planters: every 9 m, clear of the round and the passages
    const Real tv = std::max(Real(2.6), hw - 3.4);
    const Real step = 9.0;
    std::vector<Real> poleU;
    for (Real u = -hl + 4.0; u <= hl - 4.0; u += step) {
        if (std::fabs(u) < R + 2.5 || !clearOfPassages(u, 1.5)) continue;
        poleU.push_back(u);
        for (int sv : {-1, 1}) {
            const Vec2 tp = Q(u, sv * tv);
            box(tp, py, ua, Vec3(1.7, 0.5, 1.7), PartId::Trim, paveC * 1.2);   // the planter
            emitSoftBox(kit, Scope{Vec3(tp.x, py + 0.5, tp.y) - Vec3(ua.x, 0, ua.y) * 0.75 - Vec3(va.x, 0, va.y) * 0.75,
                                   {Vec3(ua.x, 0, ua.y), up, Vec3(va.x, 0, va.y)}, Vec3(1.5, 0.12, 1.5)},
                        PartId::Foliage, Vec3(0.20, 0.30, 0.14), 0u);
            g.treeSpots.push_back(Vec3(tp.x, rng.range(0.85, 1.05), tp.y));
            // a bench each side of the planter, facing the middle
            const Vec2 bpos = Q(u + step * 0.5, sv * tv);
            if (u + step * 0.5 < hl - 3.0 && std::fabs(u + step * 0.5) > R + 2.0 && clearOfPassages(u + step * 0.5, 1.0)) {
                const Vec2 face = va * static_cast<Real>(-sv);
                g.furniture.push_back(outdoorPiece(Piece::Bench, posHash(bpos) ^ seed, bpos - face * 0.33, face, 0.02, true));
            }
        }
    }
    // THE STRING LIGHTS: a pole beside each planter, and from each pole a strand across to the far pole one bay on
    // (a zigzag down the paseo), sagging, warm bulbs every 0.7 m -- lit at night (LitBand: tinted emission)
    {
        const Real poleH = 5.2, pv = tv + 1.1;
        const Vec3 iron(0.08, 0.08, 0.09), bulb(1.0, 0.78, 0.45);
        for (Real u : poleU)
            for (int sv : {-1, 1}) {
                box(Q(u, sv * pv), py, ua, Vec3(0.12, poleH, 0.12), PartId::Metal, iron);
                box(Q(u, sv * pv), py + poleH, ua, Vec3(0.2, 0.08, 0.2), PartId::Metal, iron);
            }
        for (std::size_t i = 0; i + 1 < poleU.size(); ++i) {
            if (poleU[i + 1] - poleU[i] > step * 1.6) continue;   // (across the round or a passage: no strand)
            for (int sv : {-1, 1}) {
                const Vec2 a = Q(poleU[i], sv * pv), b = Q(poleU[i + 1], -sv * pv);
                const Vec2 d = b - a;
                const Real L = d.length();
                const Vec2 dir = d * (1.0 / L);
                const int nb = static_cast<int>(L / 0.7);
                auto yAt = [&](Real t) { return py + poleH - 0.1 - 1.1 * 4 * t * (1 - t); };   // the sag
                for (int k = 0; k < 8; ++k) {   // the wire
                    const Real t0 = k / 8.0, t1 = (k + 1) / 8.0;
                    const Vec2 q0 = a + d * t0, q1 = a + d * t1;
                    const Vec2 mid = (q0 + q1) * 0.5;
                    box(mid, (yAt(t0) + yAt(t1)) * 0.5, dir, Vec3(L / 8 + 0.02, 0.015, 0.015), PartId::Metal, iron);
                }
                for (int k = 1; k < nb; ++k) {   // the bulbs
                    const Real t = Real(k) / nb;
                    box(a + d * t, yAt(t) - 0.09, dir, Vec3(0.07, 0.09, 0.07), PartId::LitBand, bulb);
                }
            }
        }
    }
    // PLACES TO BE (the citysim's activity areas): the round, and each stretch of paseo between the trees
    {
        LotBuilding::Area ar;
        ar.kind = "plaza";
        ar.center = c;
        ar.axis = ua;
        ar.halfL = hl - 2.0;
        ar.halfW = std::max(Real(1), tv - 1.5);
        g.areas.push_back(ar);
    }
    appendKit(kit, outParts, /*draped=*/true);
    g.padMesh = std::move(m);
    g.color = Vec3(1, 1, 1);
}

// THE CAMPUS SPORTS FIELD (Glenn: "sports fields"): a football pitch on the block's grass -- mown in stripes, its
// lines painted (touch and goal lines, halfway line and centre circle, penalty and goal areas, the spots), goals at
// each end -- and along one touchline BLEACHERS you can sit on. Ground-relative, like a park: the host lays it on the
// finished terrain. False (nothing laid) when the block cannot hold a 50 x 34 m pitch.
static bool sculptSportsField(LotBuilding& g, const Poly2& poly, uint32_t seed, std::vector<RenderMesh>* outParts) {
    const OBB2 ob = orientedBoundingBox(poly);
    const int la = ob.longAxis();
    const Vec2 ua = ob.axis[la], va = ob.axis[1 - la], c = ob.center;
    // the pitch, as big as the block takes (a full one is 105 x 68), with room for the stand on one side
    Real PL = std::min(Real(105), 2 * ob.half[la] - 14), PW = std::min(Real(68), 2 * ob.half[1 - la] - 22);
    // a point on the pitch: x along its length, z across, from the centre; the pitch sits 3 m off-centre, away from
    // the stand
    Vec2 pc = c - va * 3.0;
    auto P = [&](Real x, Real z, Real y) { const Vec2 q = pc + ua * x + va * z; return Vec3(q.x, y, q.y); };
    // smaller until it fits: a block's corners are its streets' rounded junctions
    auto fits = [&]() {
        for (const Vec3& q : {P(-PL / 2 - 3, -PW / 2 - 3, 0), P(PL / 2 + 3, -PW / 2 - 3, 0), P(PL / 2 + 3, PW / 2 + 9, 0), P(-PL / 2 - 3, PW / 2 + 9, 0)})
            if (!pointInPolygon(poly, Vec2(q.x, q.z))) return false;
        return true;
    };
    const Real PL0 = PL, PW0 = PW;
    // THE RUNNING TRACK (the campus gap pass): four lanes round the pitch where the block takes them -- a stadium
    // oval, its curves centred on the pitch's axis, radius R from there to the inside of lane 1, the straights S
    // long; the pitch's corners a metre inside the oval. The stand then stands beyond the track.
    constexpr Real kLane = 1.22, kLanes = 4;
    const Real trackW = kLane * kLanes;
    auto trackR = [&]() { return PW / 2 + 3.0; };
    auto trackS = [&]() { return std::max(Real(0), PL - 2 * std::sqrt(2 * PW + 8)); };
    // the track's OWN outline (a stadium has no corners -- a box round it failed every block's rounded corners), a
    // metre out, and the stand's footprint beyond the home straight
    auto fitsTrack = [&]() {
        const Real ro = trackR() + trackW + 1.0, hs = trackS() / 2;
        for (int end : {-1, 1})
            for (int i = 0; i <= 12; ++i) {
                const Real a = -1.5707963 + 3.14159265 * i / 12;
                const Vec3 q = P(end * (hs + std::cos(a) * ro), std::sin(a) * ro, 0);
                if (!pointInPolygon(poly, Vec2(q.x, q.z))) return false;
            }
        const Real sz = trackR() + trackW + 1.5 + 2.6 + 1.0, sx = std::max(hs, Real(3.2)) + 0.5;
        for (const Vec3& q : {P(-sx, sz, 0), P(sx, sz, 0)})
            if (!pointInPolygon(poly, Vec2(q.x, q.z))) return false;
        return true;
    };
    bool track = false;
    {
        // a smaller pitch inside a track reads more like a campus's field than a bigger bare one: the widest pitch
        // that fits with its track, then the longest (the curves' radius grows with the width, so the two are
        // searched apart -- shrinking both together ran out of length first)
        // ...and slid along the block up to 8 m, a block's ends being seldom square to its box
        const Real tl = PL, tw = PW;
        const Vec2 pc0 = pc;
        for (Real w = tw; w >= 34 && !track; w -= 2.0)
            for (Real l = tl; l >= 50 && !track; l -= 3.0)
                for (Real sh : {0.0, 2.0, -2.0, 4.0, -4.0, 6.0, -6.0, 8.0, -8.0}) {
                    PL = l; PW = w;
                    pc = pc0 + ua * sh;
                    track = l >= w * 1.3 && fitsTrack();
                    if (track) break;
                }
        if (!track) pc = pc0;
        if (!track) { PL = tl; PW = tw; }
    }
    if (!track)
        while ((PL >= 50 && PW >= 34) && !fits()) { PL -= 3.0; PW -= 2.0; }
    if (std::getenv("RT_CAMPUS_DEBUG"))
        std::printf("[sports field] pad %zu verts, box %.1f x %.1f: pitch %.1f x %.1f -> %.1f x %.1f, track %d\n", poly.size(),
                    2 * ob.half[la], 2 * ob.half[1 - la], PL0, PW0, PL, PW, track ? 1 : 0);
    if (PL < 50 || PW < 34) return false;   // a training pitch at the least
    const Real s = PW / 68.0;   // the markings scale with the width (a small pitch, a small box)
    RenderMesh m;
    const Vec3 up(0, 1, 0);
    auto quad = [&](Real x0, Real z0, Real x1, Real z1, Real y, const Vec3& col) {
        MeshBuilder::emitQuad(m, P(x0, z0, y), P(x1, z0, y), P(x1, z1, y), P(x0, z1, y), up, col);
    };
    // the turf: a margin round the pitch, then the pitch mown in stripes across its length
    // (raised a little: from a distance the terrain's coarser tiles stand a few centimetres proud of the leaf mesh
    // the turf is laid on)
    quad(-PL / 2 - 3, -PW / 2 - 3, PL / 2 + 3, PW / 2 + 3, 0.06, Vec3(0.26, 0.47, 0.22));
    const int stripes = 12;
    for (int i = 0; i < stripes; ++i) {
        const Real x0 = -PL / 2 + PL * i / stripes, x1 = -PL / 2 + PL * (i + 1) / stripes;
        quad(x0, -PW / 2, x1, PW / 2, 0.07, (i % 2) ? Vec3(0.30, 0.55, 0.26) : Vec3(0.27, 0.50, 0.23));
    }
    // THE TRACK: the infield's grass out to the inside of lane 1, then the lanes in a terracotta band, white lane
    // lines between them, the start/finish line across the home straight
    const Real tR = trackR(), tS = trackS();
    auto stadium = [&](Real r0, Real r1, Real y, const Vec3& col) {   // the band between two stadium outlines
        const Real hs = tS / 2;
        quad(-hs, -r1, hs, -r0, y, col);
        quad(-hs, r0, hs, r1, y, col);
        const int n = 28;
        for (int end : {-1, 1})
            for (int i = 0; i < n; ++i) {
                const Real a0 = -1.5707963 + 3.14159265 * i / n, a1 = -1.5707963 + 3.14159265 * (i + 1) / n;
                const Vec2 d0(std::cos(a0) * end, std::sin(a0)), d1(std::cos(a1) * end, std::sin(a1));
                const Real cx = end * hs;
                MeshBuilder::emitQuad(m, P(cx + d0.x * r0, d0.y * r0, y), P(cx + d1.x * r0, d1.y * r0, y),
                                      P(cx + d1.x * r1, d1.y * r1, y), P(cx + d0.x * r1, d0.y * r1, y), up, col);
            }
    };
    if (track) {
        const Vec3 infield(0.26, 0.47, 0.22), clay(0.62, 0.27, 0.19), lane(0.92, 0.90, 0.86);
        // the infield beyond the pitch's margin: the curves' segments, as a band from the axis out to the track
        stadium(0.0, tR, 0.055, infield);
        stadium(tR, tR + trackW, 0.065, clay);
        for (int k = 0; k <= static_cast<int>(kLanes); ++k)
            stadium(tR + k * kLane - 0.025, tR + k * kLane + 0.025, 0.075, lane);
        quad(tS / 2 - 0.05, -(tR + trackW), tS / 2 + 0.05, -tR, 0.075, lane);   // the finish line
        // THE LOOP a jogger runs: the middle of lane 1, anticlockwise from the finish line
        Poly2 loop;
        const Real r = tR + kLane * 0.5, hs = tS / 2;
        auto at = [&](Real x, Real z) { const Vec3 q = P(x, z, 0); loop.push_back(Vec2(q.x, q.z)); };
        for (int i = 0; i < 6; ++i) at(hs - tS * i / 6.0, -r);                 // the home straight
        for (int i = 0; i <= 12; ++i) { const Real a = -1.5707963 - 3.14159265 * i / 12; at(-hs + std::cos(a) * r, std::sin(a) * r); }
        for (int i = 1; i < 6; ++i) at(-hs + tS * i / 6.0, r);                 // the back straight
        for (int i = 0; i < 12; ++i) { const Real a = 1.5707963 - 3.14159265 * i / 12; at(hs + std::cos(a) * r, std::sin(a) * r); }
        g.loops.push_back(std::move(loop));
    }
    // the lines: 12 cm white
    const Real lw = 0.12, ly = 0.08;
    const Vec3 white(0.93, 0.93, 0.90);
    auto lineX = [&](Real x0, Real x1, Real z) { quad(x0, z - lw / 2, x1, z + lw / 2, ly, white); };
    auto lineZ = [&](Real x, Real z0, Real z1) { quad(x - lw / 2, z0, x + lw / 2, z1, ly, white); };
    lineX(-PL / 2, PL / 2, -PW / 2); lineX(-PL / 2, PL / 2, PW / 2);   // the touchlines
    lineZ(-PL / 2, -PW / 2, PW / 2); lineZ(PL / 2, -PW / 2, PW / 2);   // the goal lines
    lineZ(0, -PW / 2, PW / 2);                                        // halfway
    auto ring = [&](Real cx, Real r, Real a0, Real a1) {
        const int n = 40;
        for (int i = 0; i < n; ++i) {
            const Real t0 = a0 + (a1 - a0) * i / n, t1 = a0 + (a1 - a0) * (i + 1) / n;
            const Vec2 d0(std::cos(t0), std::sin(t0)), d1(std::cos(t1), std::sin(t1));
            MeshBuilder::emitQuad(m, P(cx + d0.x * (r - lw / 2), d0.y * (r - lw / 2), ly), P(cx + d1.x * (r - lw / 2), d1.y * (r - lw / 2), ly),
                                  P(cx + d1.x * (r + lw / 2), d1.y * (r + lw / 2), ly), P(cx + d0.x * (r + lw / 2), d0.y * (r + lw / 2), ly), up, white);
        }
    };
    ring(0, 9.15 * s, 0, 6.2832);                                     // the centre circle
    quad(-0.15, -0.15, 0.15, 0.15, ly, white);                         // the centre spot
    for (int end : {-1, 1}) {
        const Real gx = end * PL / 2;
        for (const auto& [depth, half] : {std::pair<Real, Real>{16.5 * s, 20.15 * s}, std::pair<Real, Real>{5.5 * s, 9.16 * s}}) {
            const Real ix = gx - end * depth;
            lineX(std::min(gx, ix), std::max(gx, ix), -half);
            lineX(std::min(gx, ix), std::max(gx, ix), half);
            lineZ(ix, -half, half);
        }
        const Real spot = gx - end * 11.0 * s;
        quad(spot - 0.12, -0.12, spot + 0.12, 0.12, ly, white);
        // the arc at the edge of the box (outside it): centred on the spot, radius 9.15
        const Real a = std::acos(std::clamp((16.5 * s - 11.0 * s) / (9.15 * s), Real(-1), Real(1)));
        if (end < 0) ring(spot, 9.15 * s, -a, a); else ring(spot, 9.15 * s, 3.14159265 - a, 3.14159265 + a);
    }
    // THE PITCH as an activity area (a kickabout's two halves): its centre, long axis and half sizes
    {
        LotBuilding::Area ar;
        ar.kind = "pitch";
        ar.center = pc;
        ar.axis = ua;
        ar.halfL = PL / 2;
        ar.halfW = PW / 2;
        g.areas.push_back(ar);
    }
    // THE GOALS: white posts 7.32 m apart, a 2.44 m crossbar, a net frame sloping back
    BuildingMesh kit;
    const Vec3 post(0.95, 0.95, 0.93), net(0.82, 0.82, 0.80);
    for (int end : {-1, 1}) {
        const Real gx = end * PL / 2;
        const Vec3 along(va.x, 0, va.y), out(ua.x * end, 0, ua.y * end);
        auto box = [&](const Vec3& o, const Vec3& size, const Vec3& col) {
            emitBox(kit, Scope{o, {along, up, out}, size}, PartId::Trim, col);
        };
        const Vec3 g0 = P(gx, -3.66, 0);
        box(g0 + along * -0.06, Vec3(0.12, 2.44, 0.12), post);
        box(P(gx, 3.66, 0) + along * -0.06, Vec3(0.12, 2.44, 0.12), post);
        box(g0 + along * -0.06 + up * 2.44, Vec3(7.44, 0.12, 0.12), post);
        box(g0 + out * 1.8 + up * 0.0, Vec3(0.05, 1.2, 0.05), net);              // the net's back posts
        box(P(gx, 3.66, 0) + out * 1.8, Vec3(0.05, 1.2, 0.05), net);
        box(g0 + out * 1.8 + up * 1.15, Vec3(7.32, 0.05, 0.05), net);
    }
    appendKit(kit, outParts, /*draped=*/true);
    // THE STAND: bleachers along the +va touchline, facing the pitch, set back 4 m from the line -- or, round a
    // track, 1.5 m beyond its outer lane along the home straight
    const Real standOff = track ? tR + trackW + 1.5 : PW / 2 + 4.0;
    const int stands = std::max(1, static_cast<int>((track ? std::max(tS, Real(6.4)) : PL) * 0.5 / 6.4));
    for (int k = 0; k < stands; ++k) {
        const Real x = -6.4 * (stands - 1) * 0.5 + 6.4 * k;
        const Vec2 back = pc + ua * x + va * (standOff + 2.6);
        g.furniture.push_back(outdoorPiece(Piece::Bleacher, posHash(back) ^ seed, back, va * -1.0, 0.02, true));
    }
    // TREES round the field, never on it: spots along the ends and the far touchline, outside the pitch and its
    // margin (a park without spots gets trees scattered anywhere by the host)
    Hash rng(seed ^ 0x5b07u);
    for (int k = 0; k < 40 && g.treeSpots.size() < 10; ++k) {
        const Real side = rng.unit();
        Vec2 tp;
        const Real endOff = track ? tS / 2 + tR + trackW + 3 : PL / 2 + 6, sideOff = track ? tR + trackW + 3 : PW / 2 + 6;
        if (side < 0.5) tp = pc + ua * ((rng.unit() < 0.5 ? -1 : 1) * (endOff + rng.unit() * 4)) + va * ((rng.unit() - 0.5) * PW);
        else tp = pc + ua * ((rng.unit() - 0.5) * PL) - va * (sideOff + rng.unit() * 3);
        if (!pointInPolygon(poly, tp)) continue;
        bool clear = true;
        for (std::size_t i = 0; i < poly.size() && clear; ++i)   // 2.5 m in from the block's edge
            if (distToSeg(tp, poly[i], poly[(i + 1) % poly.size()]) < 2.5) clear = false;
        for (const Vec3& o : g.treeSpots) if ((Vec2(o.x, o.z) - tp).length() < 5.0) clear = false;
        if (clear) g.treeSpots.push_back(Vec3(tp.x, rng.range(0.9, 1.3), tp.y));
    }
    g.padMesh = std::move(m);
    g.color = Vec3(1, 1, 1);
    return true;
}

// THE BUS DEPOT (Glenn, 2026-10-08: "buses shouldn't park there. There should probably be a dedicated bus depot they
// return to at end of day"): the yard, the biggest rectangle square to the gate that the block holds, fenced, its
// gate on the street. Inside, from the gate in: a drive aisle, then rows of bus bays (4.2 x 14 m, nose in) each with
// the aisle it is driven in from, and at the back the MAINTENANCE SHED -- a steel shed, a roll-up door for every
// 9 m, a lit clerestory under its eaves -- whose apron is the last aisle. Lamp poles stand at the rows' ends.
// Ground-relative, like a park (the block was picked near-flat). The bays, the gate and the shed are recorded as
// areas ("bus_bay", "bus_gate", "bus_shed") for the host: the citysim parks its buses in the bays.
// False when the block cannot hold a row of four bays.
static bool sculptBusDepot(LotBuilding& g, const Poly2& poly, Vec2 frontage, uint32_t seed, std::vector<RenderMesh>* outParts) {
    if (!outParts || poly.size() < 3 || frontage.length() < 1e-6 ||
        outParts->size() < kLotPartSlots) return false;
    const Vec2 fo = normalize(frontage);              // out through the gate, to the street
    const Vec2 v = fo * -1.0, u(v.y, -v.x);           // v in from the gate, u along the street
    const Vec2 c = centroid(poly);
    const Poly2 inner = inset(poly, 1.2);
    if (inner.size() < 3) return false;
    auto W2 = [&](Real x, Real y) { return c + u * x + v * y; };
    // THE YARD: the largest rectangle square to the gate inside the block -- the block rasterised on a 1 m grid in the
    // frame (a cell is in when its centre is 0.5 m inside the inset outline), then the largest all-in rectangle by the
    // histogram method, row by row
    Real x0 = 1e30, x1 = -1e30, y0 = 1e30, y1 = -1e30;
    for (const Vec2& w : inner) {
        const Real a = dot(w - c, u), b = dot(w - c, v);
        x0 = std::min(x0, a); x1 = std::max(x1, a); y0 = std::min(y0, b); y1 = std::max(y1, b);
    }
    auto insideCount = [&](Real a0, Real b0, Real a1, Real b1) {
        int n = 0;
        for (int i = 0; i <= 8; ++i)
            for (int j = 0; j <= 8; ++j)
                if (pointInPolygon(inner, W2(a0 + (a1 - a0) * i / 8, b0 + (b1 - b0) * j / 8))) ++n;
        return n;
    };
    {
        Poly2 cellIn = inset(poly, 1.9);
        if (cellIn.size() < 3) cellIn = inner;   // (an inset can fail on a ragged outline; the yard is checked on `inner` below)
        const int nx = static_cast<int>(x1 - x0), ny = static_cast<int>(y1 - y0);
        if (cellIn.size() < 3 || nx < 30 || ny < 30) {
            if (std::getenv("RT_DEPOT_DEBUG"))
                std::printf("[depot] at %.0f %.0f: block %d x %d in the gate's frame, inset %zu verts\n", c.x, c.y, nx, ny, cellIn.size());
            return false;
        }
        std::vector<int> h(static_cast<std::size_t>(nx), 0);
        Real best = 0;
        Real bx0 = 0, bx1 = 0, by0 = 0, by1 = 0;
        for (int j = 0; j < ny; ++j) {
            for (int i = 0; i < nx; ++i)
                h[static_cast<std::size_t>(i)] = pointInPolygon(cellIn, W2(x0 + i + 0.5, y0 + j + 0.5)) ? h[static_cast<std::size_t>(i)] + 1 : 0;
            std::vector<int> st;   // the largest rectangle under this row's histogram
            for (int i = 0; i <= nx; ++i) {
                const int hi = i < nx ? h[static_cast<std::size_t>(i)] : 0;
                while (!st.empty() && h[static_cast<std::size_t>(st.back())] >= hi) {
                    const int top = h[static_cast<std::size_t>(st.back())];
                    st.pop_back();
                    const int left = st.empty() ? 0 : st.back() + 1;
                    const Real ar = static_cast<Real>(top) * (i - left);
                    if (ar > best && i - left >= 30 && top >= 30) {
                        best = ar;
                        bx0 = x0 + left; bx1 = x0 + i; by0 = y0 + j + 1 - top; by1 = y0 + j + 1;
                    }
                    if (i == nx && st.empty()) break;
                }
                if (i < nx) st.push_back(i);
            }
        }
        if (best <= 0) { x1 = x0; y1 = y0; }
        else { x0 = bx0; x1 = bx1; y0 = by0; y1 = by1; }
    }
    const Real W = x1 - x0, D = y1 - y0, cx = (x0 + x1) * 0.5;
    const bool dbgOn = std::getenv("RT_DEPOT_DEBUG") != nullptr;
    if (insideCount(x0, y0, x1, y1) < 81 || W < 30 || D < 36) {
        if (dbgOn) std::printf("[depot] at %.0f %.0f: no yard (%.0f x %.0f)\n", c.x, c.y, W, D);
        return false;
    }
    // A BAY is a LANE two buses long (they park nose to tail, as a real yard packs them) where the yard is deep enough,
    // else one bus long
    constexpr Real bayW = 4.2, aisle = 15.0, shedD = 24.0, shedW = 22.0;
    const Real bayL = D >= aisle + 26.0 + 1.0 ? 26.0 : 14.0;
    const int perBay = bayL > 20 ? 2 : 1;
    // THE SHED: across the back where the yard is deep enough for a row of bays and the shed's own apron, else at
    // one end, its doors on the drive aisle (most blocks are 45-65 m deep and longer than that)
    const bool backShed = D >= aisle + bayL + aisle + shedD;
    const bool endShed = !backShed && W >= 6.0 + 4 * bayW + shedW + 4.0 && D >= aisle + 16.0;
    const bool shed = backShed || endShed;
    std::vector<Real> rows;   // each row's near edge (its aisle in front of it)
    for (Real y = y0 + aisle; y + bayL + (backShed ? aisle + shedD : 1.0) <= y1 + 1e-6; y += bayL + aisle) rows.push_back(y);
    if (!rows.empty()) {   // the rows against the back (or the shed's apron): the spare depth goes to the gate's aisle
        const Real spare = (y1 - (backShed ? aisle + shedD : 1.0)) - (rows.back() + bayL);
        for (Real& r : rows) r += std::max(Real(0), spare);
    }
    const Real bx0 = x0 + 3.0, bx1 = endShed ? x1 - 1.0 - shedW - 3.0 : x1 - 3.0;
    const int nBay = static_cast<int>((bx1 - bx0) / bayW);
    if (rows.empty() || nBay < 4) {
        if (dbgOn) std::printf("[depot] at %.0f %.0f: yard %.0f x %.0f holds no row\n", c.x, c.y, W, D);
        return false;
    }
    const Real sx0 = (bx0 + bx1) * 0.5 - nBay * bayW * 0.5;
    // the shed's rectangle (doors on its y = sya face, toward the gate)
    const Real sxa = backShed ? cx - std::min(W - 8.0, Real(96)) * 0.5 : x1 - 1.0 - shedW;
    const Real sxb = backShed ? cx + std::min(W - 8.0, Real(96)) * 0.5 : x1 - 1.0;
    const Real sya = backShed ? y1 - 1.0 - shedD : y0 + aisle, syb = y1 - 1.0;

    const Vec3 up(0, 1, 0), u3(u.x, 0, u.y), v3(v.x, 0, v.y);
    auto P = [&](Real x, Real y, Real h) { const Vec2 w = W2(x, y); return Vec3(w.x, h, w.y); };
    RenderMesh m;   // the yard's surface and paint (ground-relative, vertex colours)
    auto quad = [&](Real a0, Real b0, Real a1, Real b1, Real h, const Vec3& col) {
        MeshBuilder::emitQuad(m, P(a0, b0, h), P(a0, b1, h), P(a1, b1, h), P(a1, b0, h), up, col);
    };
    const Vec3 asphalt(0.07, 0.07, 0.075), white(0.90, 0.90, 0.87), yellow(0.90, 0.76, 0.16);   // (linear: the drawn lot reads dark)
    quad(x0, y0, x1, y1, 0.05, asphalt);
    // the drive out of the gate to the block's edge (the pavement beyond)
    Real yOut = y0;
    while (yOut > y0 - 12 && pointInPolygon(poly, W2(cx, yOut - 0.25))) yOut -= 0.25;
    if (yOut < y0) quad(cx - 8, yOut, cx + 8, y0, 0.05, asphalt);
    g.sealed.push_back({W2(x0, y0), W2(x1, y0), W2(x1, y1), W2(x0, y1)});
    if (yOut < y0) g.sealed.push_back({W2(cx - 8, yOut), W2(cx + 8, yOut), W2(cx + 8, y0), W2(cx - 8, y0)});
    for (Poly2& s : g.sealed) ensureCCW(s);
    // THE BAYS: white dividers, a yellow line across their noses, each bay an area for the host
    for (Real ry : rows) {
        for (int k = 0; k <= nBay; ++k) {
            const Real x = sx0 + k * bayW;
            quad(x - 0.06, ry, x + 0.06, ry + bayL, 0.07, white);
        }
        quad(sx0, ry + bayL - 0.15, sx0 + nBay * bayW, ry + bayL, 0.07, yellow);
        for (int k = 0; k < nBay; ++k)
            for (int q = 0; q < perBay; ++q) {   // the far slot first: the first bus in drives to the end of the lane
                LotBuilding::Area ar;
                ar.kind = "bus_bay";
                ar.center = W2(sx0 + (k + 0.5) * bayW, ry + bayL - (q + 0.5) * (bayL / perBay));
                ar.axis = v;   // nose in, away from its aisle
                ar.halfL = bayL / perBay * 0.5;
                ar.halfW = bayW * 0.5;
                g.areas.push_back(ar);
            }
    }
    {   // the gate: where a bus leaves the street (the host anchors the bays on the street in front of it)
        LotBuilding::Area ar;
        ar.kind = "bus_gate";
        ar.center = W2(cx, yOut);
        ar.axis = fo;
        ar.halfL = 8;
        ar.halfW = 1;
        g.areas.push_back(ar);
    }
    BuildingMesh kit;
    // THE FENCE round the yard, the gate left open: posts every 3 m, three rails
    {
        const Vec3 steel(0.42, 0.44, 0.45);
        auto fence = [&](Vec2 a, Vec2 b) {   // frame points
            const Vec2 d = b - a;
            const Real L = d.length();
            if (L < 0.5) return;
            const Vec2 t = d * (1.0 / L);
            const Vec3 ax(u.x * t.x + v.x * t.y, 0, u.y * t.x + v.y * t.y), lat(-ax.z, 0, ax.x);
            const Vec3 o = P(a.x, a.y, 0);
            for (Real h : {Real(0.15), Real(1.1), Real(2.05)})
                emitBox(kit, Scope{o + up * h - lat * 0.02, {ax, up, lat}, Vec3(L, 0.05, 0.04)}, PartId::Metal, steel);
            const int nPost = std::max(1, static_cast<int>(L / 3.0));
            for (int k = 0; k <= nPost; ++k)
                emitBox(kit, Scope{o + ax * (L * k / nPost - 0.04) - lat * 0.04, {ax, up, lat}, Vec3(0.08, 2.2, 0.08)},
                        PartId::Metal, steel);
            g.fenceSegs.push_back({W2(a.x, a.y), W2(b.x, b.y)});
        };
        fence({x0, y0}, {cx - 8, y0});
        fence({cx + 8, y0}, {x1, y0});
        fence({x1, y0}, {x1, y1});
        fence({x1, y1}, {x0, y1});
        fence({x0, y1}, {x0, y0});
        for (Real gx : {cx - 8.0, cx + 8.0})   // the gate's posts
            emitBox(kit, Scope{P(gx - 0.15, y0 - 0.15, 0), {u3, up, v3}, Vec3(0.3, 2.6, 0.3)}, PartId::Metal, Vec3(0.30, 0.32, 0.34));
    }
    // LAMP POLES at the rows' ends (their lamps glow at night)
    RenderMesh lamps;
    const Vec3 lampCol(1.0, 0.92, 0.75), poleCol(0.38, 0.39, 0.41);
    auto lampPole = [&](Real xs, Real ys) {
        emitBox(kit, Scope{P(xs - 0.12, ys - 0.12, 0), {u3, up, v3}, Vec3(0.24, 10.0, 0.24)}, PartId::Metal, poleCol);
        emitBox(kit, Scope{P(xs - 0.3, ys - 1.4, 10.0), {u3, up, v3}, Vec3(0.6, 0.25, 2.8)}, PartId::Metal, poleCol);
        for (Real dy : {Real(-1.25), Real(0.55)})
            MeshBuilder::emitQuad(lamps, P(xs - 0.25, ys + dy, 9.99), P(xs + 0.25, ys + dy, 9.99), P(xs + 0.25, ys + dy + 0.7, 9.99),
                                  P(xs - 0.25, ys + dy + 0.7, 9.99), up * -1.0, lampCol);
    };
    for (Real ry : rows) {
        if (sx0 - 1.5 > x0 + 0.5) lampPole(sx0 - 1.5, ry + bayL * 0.5);
        if (sx0 + nBay * bayW + 1.5 < (endShed ? sxa - 0.5 : x1 - 0.5)) lampPole(sx0 + nBay * bayW + 1.5, ry + bayL * 0.5);
    }
    // THE MAINTENANCE SHED at the back (or, on a shallow yard, the dispatcher's cabin in a back corner)
    const Vec3 clad(0.66, 0.68, 0.66), trim(0.26, 0.38, 0.52), doorCol(0.52, 0.54, 0.56), roofCol(0.46, 0.47, 0.48);
    if (shed) {
        const Real sw = sxb - sxa, sd = syb - sya;
        constexpr Real eave = 7.5, ridge = 9.3, t = 0.25;
        emitBox(kit, Scope{P(sxa, sya, 0), {u3, up, v3}, Vec3(sw, eave, t)}, PartId::Metal, clad);              // front
        emitBox(kit, Scope{P(sxa, syb - t, 0), {u3, up, v3}, Vec3(sw, eave, t)}, PartId::Metal, clad);          // back
        emitBox(kit, Scope{P(sxa, sya, 0), {u3, up, v3}, Vec3(t, eave, sd)}, PartId::Metal, clad);           // ends
        emitBox(kit, Scope{P(sxa + sw - t, sya, 0), {u3, up, v3}, Vec3(t, eave, sd)}, PartId::Metal, clad);
        emitBox(kit, Scope{P(sxa - 0.1, sya - 0.1, eave - 0.6), {u3, up, v3}, Vec3(sw + 0.2, 0.6, 0.12)}, PartId::Trim, trim);   // fascia
        // the roof: two slopes to a ridge along the shed, the gables closed
        RenderMesh roof, gable, lit;
        const Real ym = (sya + syb) * 0.5;
        const Real o = 0.4;   // the eaves' overhang
        MeshBuilder::emitQuad(roof, P(sxa - o, sya - o, eave), P(sxa + sw + o, sya - o, eave), P(sxa + sw + o, ym, ridge),
                              P(sxa - o, ym, ridge), normalize(Vec3(0, 1, 0) - v3 * ((ridge - eave) / (ym - sya))), roofCol);
        MeshBuilder::emitQuad(roof, P(sxa - o, ym, ridge), P(sxa + sw + o, ym, ridge), P(sxa + sw + o, syb + o, eave),
                              P(sxa - o, syb + o, eave), normalize(Vec3(0, 1, 0) + v3 * ((ridge - eave) / (syb - ym))), roofCol);
        for (Real gx : {sxa, sxa + sw}) {
            const Vec3 n = gx == sxa ? u3 * -1.0 : u3;
            MeshBuilder::emitTri(gable, P(gx, sya, eave), P(gx, syb, eave), P(gx, ym, ridge), n, clad);
            MeshBuilder::emitTri(gable, P(gx, syb, eave), P(gx, sya, eave), P(gx, ym, ridge), n * -1.0, clad);
        }
        // the roll-up doors, one for every 9 m of the front, and the lit clerestory over them
        const int nDoor = std::max(2, static_cast<int>(sw / 9.0));
        for (int k = 0; k < nDoor; ++k) {
            const Real dc = sxa + sw * (k + 0.5) / nDoor;
            emitBox(kit, Scope{P(dc - 2.6, sya - 0.12, 0), {u3, up, v3}, Vec3(5.2, 5.4, 0.12)}, PartId::Metal, doorCol);
            for (int s = 1; s < 9; ++s)   // its slats
                emitBox(kit, Scope{P(dc - 2.6, sya - 0.16, s * 0.6), {u3, up, v3}, Vec3(5.2, 0.04, 0.04)}, PartId::Metal,
                        doorCol * 0.8);
            emitBox(kit, Scope{P(dc - 2.8, sya - 0.18, 5.4), {u3, up, v3}, Vec3(5.6, 0.3, 0.2)}, PartId::Trim, trim);   // the head
            emitBox(kit, Scope{P(dc - 2.8, sya - 0.18, 0), {u3, up, v3}, Vec3(0.2, 5.4, 0.2)}, PartId::Trim, trim);
            emitBox(kit, Scope{P(dc + 2.6, sya - 0.18, 0), {u3, up, v3}, Vec3(0.2, 5.4, 0.2)}, PartId::Trim, trim);
        }
        MeshBuilder::emitQuad(lit, P(sxa + 0.5, sya - 0.02, 6.0), P(sxa + sw - 0.5, sya - 0.02, 6.0), P(sxa + sw - 0.5, sya - 0.02, 6.7),
                              P(sxa + 0.5, sya - 0.02, 6.7), v3 * -1.0, Vec3(1.0, 0.95, 0.82));
        MeshBuilder::append((*outParts)[drapedSlot(PartId::Roof)], roof);
        MeshBuilder::append((*outParts)[drapedSlot(PartId::Metal)], gable);
        MeshBuilder::append((*outParts)[drapedSlot(PartId::LitBand)], lit);
        quad(sxa - 1, sya - 3, sxa + sw + 1, syb, 0.06, Vec3(0.20, 0.20, 0.19));   // the concrete apron and floor
        LotBuilding::Area ar;
        ar.kind = "bus_shed";
        ar.center = W2((sxa + sxb) * 0.5, ym);
        ar.axis = u;
        ar.halfL = sw * 0.5;
        ar.halfW = sd * 0.5;
        g.areas.push_back(ar);
    } else {
        const Real ax = x1 - 9.0, ay = y1 - 4.5;
        emitBox(kit, Scope{P(ax, ay, 0), {u3, up, v3}, Vec3(7.0, 3.0, 3.5)}, PartId::Metal, clad);
        emitBox(kit, Scope{P(ax - 0.2, ay - 0.2, 3.0), {u3, up, v3}, Vec3(7.4, 0.2, 3.9)}, PartId::Trim, trim);
        RenderMesh lit;
        MeshBuilder::emitQuad(lit, P(ax + 1.0, ay - 0.02, 1.2), P(ax + 4.0, ay - 0.02, 1.2), P(ax + 4.0, ay - 0.02, 2.2),
                              P(ax + 1.0, ay - 0.02, 2.2), v3 * -1.0, Vec3(1.0, 0.95, 0.82));
        MeshBuilder::append((*outParts)[drapedSlot(PartId::LitBand)], lit);
    }
    MeshBuilder::append((*outParts)[drapedSlot(PartId::LitBand)], lamps);
    appendKit(kit, outParts, /*draped=*/true);
    (void)seed;
    if (std::getenv("RT_DEPOT_DEBUG"))
        std::printf("[depot] yard %.0f x %.0f at %.0f %.0f: %zu rows of %d bays x %d buses, shed %s\n", W, D, c.x, c.y, rows.size(), nBay, perBay,
                    backShed ? "back" : endShed ? "end" : "none");
    g.padMesh = std::move(m);
    g.color = Vec3(1, 1, 1);
    return true;
}

// ---- OPEN LOTS (Glenn, 2026-10-09: "free rein" on the city's next ten; #1 "the empty blocks") ----------------------
// A lot that does not build used to be a bare GREEN: 2,223 of the island's 13,156 lots, flat grass between the houses.
// Real cities put those lots to use. The open-lot program gives each one a use by district, size and whether it meets
// a street: a CAR PARK, a PLAYGROUND, a COMMUNITY GARDEN or a POCKET PARK (sculptPark). Hillsides, road-locked slivers
// and lots too small for any of them stay green. Every sculptor below is ground-relative (kDrapedPartBase) like a
// park or the bus depot, works in the lot's street frame (u along the street, v in from it) on the largest rectangle
// square to that frame, and returns false when the lot cannot hold it (the caller then tries the next use).

// The largest rectangle inside `poly` square to the frame (c, u, v), on a `step` grid (cells whose centre lies inside
// `inner`), by the histogram method row by row. False when nothing of at least minW x minD fits.
static bool largestFrameRect(const Poly2& inner, Vec2 c, Vec2 u, Vec2 v, Real step, Real minW, Real minD,
                             Real& rx0, Real& rx1, Real& ry0, Real& ry1) {
    if (inner.size() < 3) return false;
    Real x0 = 1e30, x1 = -1e30, y0 = 1e30, y1 = -1e30;
    for (const Vec2& w : inner) {
        const Real a = dot(w - c, u), b = dot(w - c, v);
        x0 = std::min(x0, a); x1 = std::max(x1, a); y0 = std::min(y0, b); y1 = std::max(y1, b);
    }
    const int nx = static_cast<int>((x1 - x0) / step), ny = static_cast<int>((y1 - y0) / step);
    if (nx < 2 || ny < 2 || nx > 2000 || ny > 2000) return false;
    std::vector<int> h(static_cast<std::size_t>(nx), 0);
    Real best = 0;
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const Vec2 q = c + u * (x0 + (i + 0.5) * step) + v * (y0 + (j + 0.5) * step);
            h[static_cast<std::size_t>(i)] = pointInPolygon(inner, q) ? h[static_cast<std::size_t>(i)] + 1 : 0;
        }
        std::vector<int> st;
        for (int i = 0; i <= nx; ++i) {
            const int hi = i < nx ? h[static_cast<std::size_t>(i)] : 0;
            while (!st.empty() && h[static_cast<std::size_t>(st.back())] >= hi) {
                const int top = st.back();
                st.pop_back();
                const int left = st.empty() ? 0 : st.back() + 1;
                const Real wR = (i - left) * step, dR = h[static_cast<std::size_t>(top)] * step;
                if (wR >= minW && dR >= minD && wR * dR > best) {
                    best = wR * dR;
                    rx0 = x0 + left * step; rx1 = x0 + i * step;
                    ry1 = y0 + (j + 1) * step; ry0 = ry1 - dR;
                }
            }
            st.push_back(i);
        }
    }
    return best > 0;
}

// The lot's street frame: v in from the street (against the lot's frontage), u along it.
struct OpenFrame {
    Vec2 c, u, v, fo;
    Real x0 = 0, x1 = 0, y0 = 0, y1 = 0;
    Vec2 W(Real x, Real y) const { return c + u * x + v * y; }
    Vec3 P(Real x, Real y, Real h) const { const Vec2 w = W(x, y); return Vec3(w.x, h, w.y); }
};
static bool openFrameOf(const Poly2& poly, Vec2 frontage, Real inset0, Real minW, Real minD, OpenFrame& f) {
    if (poly.size() < 3 || frontage.length() < 1e-6) return false;
    f.fo = normalize(frontage);
    {   // square to the lot: the frontage is toward the nearest street point (diagonal on a corner lot), so take the
        // outward normal of the edge that faces it best, weighted by length
        Poly2 ccw = poly;
        ensureCCW(ccw);
        Real best = 0;
        Vec2 bn = f.fo;
        for (std::size_t i = 0; i < ccw.size(); ++i) {
            const Vec2 e = ccw[(i + 1) % ccw.size()] - ccw[i];
            const Real L = e.length();
            if (L < 4.0) continue;
            const Vec2 n(e.y / L, -e.x / L);   // outward, for a CCW ring
            const Real s = dot(n, f.fo) * std::min(L, Real(30));
            if (dot(n, f.fo) > 0.4 && s > best) { best = s; bn = n; }
        }
        f.fo = bn;
    }
    f.v = f.fo * -1.0;
    f.u = Vec2(f.v.y, -f.v.x);
    f.c = centroid(poly);
    Poly2 inner = inset(poly, inset0);
    if (inner.size() < 3) return false;
    const bool ok = largestFrameRect(inner, f.c, f.u, f.v, 0.5, minW, minD, f.x0, f.x1, f.y0, f.y1);
    if (!ok && std::getenv("RT_OPENLOT_DEBUG")) {
        Real a0 = 1e30, a1 = -1e30, b0 = 1e30, b1 = -1e30;
        for (const Vec2& w : inner) { a0 = std::min(a0, dot(w - f.c, f.u)); a1 = std::max(a1, dot(w - f.c, f.u)); b0 = std::min(b0, dot(w - f.c, f.v)); b1 = std::max(b1, dot(w - f.c, f.v)); }
        std::printf("[openlot]   no %.1f x %.1f rect in a %.1f x %.1f frame box (%zu verts, frontage %.2f %.2f)\n", minW, minD, a1 - a0, b1 - b0, inner.size(), f.fo.x, f.fo.y);
    }
    return ok;
}

// A walk from the frame rectangle's street edge at x straight out to the lot line (the pavement is beyond it):
// sealed, recorded for the walkers, drawn into `m` at kerb height.
static void openWalkOut(LotBuilding& g, RenderMesh& m, const Poly2& poly, const OpenFrame& f, Real x, Real hw,
                        const Vec3& col) {
    Real yOut = f.y0;
    while (yOut > f.y0 - 14 && pointInPolygon(poly, f.W(x, yOut - 0.25))) yOut -= 0.25;
    if (yOut >= f.y0 - 0.2) return;
    const int segs = std::max(1, static_cast<int>((f.y0 - yOut) / 1.0));
    for (int s = 0; s < segs; ++s) {
        const Real ya = yOut + (f.y0 - yOut) * s / segs, yb = yOut + (f.y0 - yOut) * (s + 1) / segs;
        MeshBuilder::emitQuad(m, f.P(x - hw, ya, 0.05), f.P(x - hw, yb, 0.05), f.P(x + hw, yb, 0.05), f.P(x + hw, ya, 0.05),
                              Vec3(0, 1, 0), col);
    }
    Poly2 s{f.W(x - hw, yOut), f.W(x + hw, yOut), f.W(x + hw, f.y0), f.W(x - hw, f.y0)};
    ensureCCW(s);
    g.sealed.push_back(s);
    g.walks.push_back({f.W(x, yOut), f.W(x, f.y0 + 1.0), hw * 2});
}

// A fence round the frame rectangle with a gap of `gate` metres at x = gx on the street side. `kind` 0: low painted
// steel railing (a playground); 1: white picket (a garden).
static void openFence(LotBuilding& g, BuildingMesh& kit, const OpenFrame& f, Real gx, Real gate, int kind) {
    const Vec3 up(0, 1, 0);
    const Vec3 col = kind == 0 ? Vec3(0.16, 0.34, 0.22) : Vec3(0.86, 0.85, 0.80);
    const Real H = kind == 0 ? 1.1 : 0.95;
    auto run = [&](Vec2 a, Vec2 b) {   // frame points
        const Vec2 wa = f.W(a.x, a.y), wb = f.W(b.x, b.y);
        const Vec2 d = wb - wa;
        const Real L = d.length();
        if (L < 0.4) return;
        const Vec2 t = d * (1.0 / L);
        const Vec3 ax(t.x, 0, t.y), lat(-t.y, 0, t.x);
        const Vec3 o(wa.x, 0, wa.y);
        if (kind == 0) {
            for (Real h : {Real(0.12), Real(H - 0.05)})
                emitBox(kit, Scope{o + up * h - lat * 0.025, {ax, up, lat}, Vec3(L, 0.05, 0.05)}, PartId::Metal, col);
            const int nb = std::max(1, static_cast<int>(L / 0.14));   // the bars
            for (int k = 0; k <= nb; k += 1)
                emitBox(kit, Scope{o + ax * (L * k / nb - 0.012) - lat * 0.012, {ax, up, lat}, Vec3(0.024, H, 0.024)},
                        PartId::Metal, col);
        } else {
            for (Real h : {Real(0.25), Real(0.70)})
                emitBox(kit, Scope{o + up * h - lat * 0.02, {ax, up, lat}, Vec3(L, 0.07, 0.025)}, PartId::Wood, col);
            const int nb = std::max(1, static_cast<int>(L / 0.13));   // the pickets
            for (int k = 0; k < nb; ++k)
                emitBox(kit, Scope{o + ax * (L * (k + 0.5) / nb - 0.04) + lat * 0.01, {ax, up, lat}, Vec3(0.08, H, 0.02)},
                        PartId::Wood, col);
        }
        const int np = std::max(1, static_cast<int>(L / 2.4));   // posts
        for (int k = 0; k <= np; ++k)
            emitBox(kit, Scope{o + ax * (L * k / np - 0.05) - lat * 0.05, {ax, up, lat}, Vec3(0.10, H + 0.1, 0.10)},
                    kind == 0 ? PartId::Metal : PartId::Wood, col * 0.9);
        g.fenceSegs.push_back({wa, wb});
    };
    run({f.x0, f.y0}, {gx - gate * 0.5, f.y0});
    run({gx + gate * 0.5, f.y0}, {f.x1, f.y0});
    run({f.x1, f.y0}, {f.x1, f.y1});
    run({f.x1, f.y1}, {f.x0, f.y1});
    run({f.x0, f.y1}, {f.x0, f.y0});
}

// THE CAR PARK: a surface lot off the street. A cross aisle along the street edge (the gate in its middle), aisles
// running in from it, perpendicular stalls 2.5 x 5 m both sides of each aisle; asphalt, white stall lines, a hedge
// along the street and the sides, a lamp pole at each aisle's far end, a pay machine by the gate, trees in the
// corners. Every stall is an area "car_bay" (axis: nose in), the gate an area "car_gate" (axis: out to the street):
// the host makes them a garage the citysim's drivers park in, as it does a depot's bays. Type "depot" for the host:
// paved and sealed, ground-relative, no place for the schedules.
static bool sculptCarPark(LotBuilding& g, const Poly2& poly, Vec2 frontage, uint32_t seed, std::vector<RenderMesh>* outParts) {
    if (!outParts || outParts->size() < kLotPartSlots) return false;
    constexpr Real stallW = 2.5, stallL = 5.0, aisleW = 6.0, frontAisle = 6.0, verge = 1.4;
    OpenFrame f;
    if (!openFrameOf(poly, frontage, 0.8, stallL + aisleW + 2 * verge, frontAisle + 2 * stallW + verge, f)) return false;
    Hash rng(mix(seed, 0xCA7FA2Cu));
    const Real W = f.x1 - f.x0;
    // double-loaded aisles where the lot is wide enough for one, else ONE row of stalls beside a single aisle
    const bool single = W < 2 * stallL + aisleW + 2 * verge;
    const Real module = single ? stallL + aisleW : 2 * stallL + aisleW;
    const int nMod = std::max(1, static_cast<int>((W - 2 * verge) / module));
    const Real mx0 = (f.x0 + f.x1) * 0.5 - nMod * module * 0.5;   // the modules centred across the lot
    const Real sy0 = f.y0 + verge + frontAisle;                       // the stall rows start behind the cross aisle
    const Real sy1 = f.y1 - verge;
    const int nAlong = static_cast<int>((sy1 - sy0) / stallW);
    if (nAlong * (single ? 1 : 2) * nMod < 6) return false;   // two or three stalls is a driveway, not a car park
    const Vec3 up(0, 1, 0);
    const Vec3 asphalt(0.075, 0.075, 0.08), white(0.90, 0.90, 0.87);
    RenderMesh m;
    auto quad = [&](Real a0, Real b0, Real a1, Real b1, Real h, const Vec3& col) {
        MeshBuilder::emitQuad(m, f.P(a0, b0, h), f.P(a0, b1, h), f.P(a1, b1, h), f.P(a1, b0, h), up, col);
    };
    const Real ax0 = mx0 - 0.3, ax1 = mx0 + nMod * module + 0.3;
    quad(ax0, f.y0 + verge, ax1, sy0 + nAlong * stallW + 0.3, 0.05, asphalt);
    const Real gx = (ax0 + ax1) * 0.5;
    // the drive out to the street, through the verge
    Real yOut = f.y0 + verge;
    while (yOut > f.y0 - 14 && pointInPolygon(poly, f.W(gx, yOut - 0.25))) yOut -= 0.25;
    quad(gx - 3.0, yOut, gx + 3.0, f.y0 + verge, 0.05, asphalt);
    {
        Poly2 a{f.W(ax0, f.y0 + verge), f.W(ax1, f.y0 + verge), f.W(ax1, sy0 + nAlong * stallW + 0.3), f.W(ax0, sy0 + nAlong * stallW + 0.3)};
        Poly2 d{f.W(gx - 3.0, yOut), f.W(gx + 3.0, yOut), f.W(gx + 3.0, f.y0 + verge), f.W(gx - 3.0, f.y0 + verge)};
        ensureCCW(a); ensureCCW(d);
        g.sealed.push_back(a);
        g.sealed.push_back(d);
    }
    int stalls = 0;
    for (int k = 0; k < nMod; ++k) {
        const Real ac = single ? mx0 + aisleW * 0.5 : mx0 + k * module + module * 0.5;   // the aisle's centre line
        for (int side = single ? 1 : -1; side <= 1; side += 2) {
            const Real xa = ac + side * aisleW * 0.5, xb = ac + side * (aisleW * 0.5 + stallL);
            for (int s = 0; s <= nAlong; ++s) {   // the stall lines
                const Real y = sy0 + s * stallW;
                quad(std::min(xa, xb) + (side < 0 ? 0.4 : 0), y - 0.06, std::max(xa, xb) - (side > 0 ? 0.4 : 0), y + 0.06, 0.07, white);
            }
            for (int s = 0; s < nAlong; ++s) {
                LotBuilding::Area ar;
                ar.kind = "car_bay";
                ar.center = f.W((xa + xb) * 0.5, sy0 + (s + 0.5) * stallW);
                ar.axis = f.u * static_cast<Real>(side);   // nose in, away from the aisle
                ar.halfL = stallL * 0.5;
                ar.halfW = stallW * 0.5;
                g.areas.push_back(ar);
                ++stalls;
            }
        }
    }
    {
        LotBuilding::Area ar;
        ar.kind = "car_gate";
        ar.center = f.W(gx, yOut);
        ar.axis = f.fo;
        ar.halfL = 3;
        ar.halfW = 1;
        g.areas.push_back(ar);
    }
    BuildingMesh kit;
    // the hedge: along the street (the gate open) and the two sides, a clipped box on the verge
    const Vec3 hedge(0.22, 0.38, 0.19);
    auto hedgeRun = [&](Real a0, Real b0, Real a1, Real b1) {   // frame rect of the hedge
        if (a1 - a0 < 0.6 || b1 - b0 < 0.4) return;
        const Vec3 u3(f.u.x, 0, f.u.y), v3(f.v.x, 0, f.v.y);
        emitSoftBox(kit, Scope{f.P(a0, b0, 0), {u3, up, v3}, Vec3(a1 - a0, 0.9, b1 - b0)}, PartId::Foliage, hedge, 0u);
    };
    const Real hy0 = f.y0 + 0.25, hy1 = f.y0 + verge - 0.25;
    hedgeRun(f.x0 + 0.3, hy0, gx - 4.0, hy1);
    hedgeRun(gx + 4.0, hy0, f.x1 - 0.3, hy1);
    hedgeRun(f.x0 + 0.25, f.y0 + verge + 1.0, ax0 - 0.2, f.y1 - 0.3);
    hedgeRun(ax1 + 0.2, f.y0 + verge + 1.0, f.x1 - 0.25, f.y1 - 0.3);
    // lamp poles at each aisle's far end; the lenses glow at night
    RenderMesh lamps;
    const Vec3 poleCol(0.38, 0.39, 0.41), lampCol(1.0, 0.92, 0.75);
    const Vec3 u3(f.u.x, 0, f.u.y), v3(f.v.x, 0, f.v.y);
    for (int k = 0; k < nMod; ++k) {
        const Real xs = single ? mx0 + aisleW * 0.5 : mx0 + k * module + module * 0.5, ys = sy0 + nAlong * stallW + 0.05;
        if (ys + 0.4 > f.y1) continue;
        emitBox(kit, Scope{f.P(xs - 0.1, ys, 0), {u3, up, v3}, Vec3(0.2, 7.5, 0.2)}, PartId::Metal, poleCol);
        emitBox(kit, Scope{f.P(xs - 0.25, ys - 1.6, 7.5), {u3, up, v3}, Vec3(0.5, 0.2, 1.8)}, PartId::Metal, poleCol);
        MeshBuilder::emitQuad(lamps, f.P(xs - 0.2, ys - 1.5, 7.49), f.P(xs + 0.2, ys - 1.5, 7.49), f.P(xs + 0.2, ys - 0.9, 7.49),
                              f.P(xs - 0.2, ys - 0.9, 7.49), up * -1.0, lampCol);
    }
    MeshBuilder::append((*outParts)[drapedSlot(PartId::LitBand)], lamps);
    {   // the pay machine by the gate, and a P sign on a post at the street
        const Real px = gx + 3.6, py = f.y0 + verge + 0.6;
        emitBox(kit, Scope{f.P(px, py, 0), {u3, up, v3}, Vec3(0.6, 1.6, 0.45)}, PartId::Metal, Vec3(0.20, 0.32, 0.52));
        emitBox(kit, Scope{f.P(px + 0.1, py - 0.02, 1.0), {u3, up, v3}, Vec3(0.4, 0.3, 0.02)}, PartId::Trim, Vec3(0.10, 0.12, 0.14));
        const Real sx = gx - 4.2, sy = f.y0 + 0.4;
        emitBox(kit, Scope{f.P(sx, sy, 0), {u3, up, v3}, Vec3(0.1, 2.6, 0.1)}, PartId::Metal, poleCol);
        emitBox(kit, Scope{f.P(sx - 0.35, sy - 0.03, 2.6), {u3, up, v3}, Vec3(0.8, 0.8, 0.06)}, PartId::Trim, Vec3(0.10, 0.25, 0.62));
        emitBox(kit, Scope{f.P(sx - 0.12, sy - 0.05, 2.75), {u3, up, v3}, Vec3(0.08, 0.5, 0.02)}, PartId::Trim, Vec3(0.95, 0.95, 0.95));
        emitBox(kit, Scope{f.P(sx - 0.12, sy - 0.05, 3.1), {u3, up, v3}, Vec3(0.3, 0.08, 0.02)}, PartId::Trim, Vec3(0.95, 0.95, 0.95));
        emitBox(kit, Scope{f.P(sx - 0.12, sy - 0.05, 2.92), {u3, up, v3}, Vec3(0.3, 0.08, 0.02)}, PartId::Trim, Vec3(0.95, 0.95, 0.95));
        emitBox(kit, Scope{f.P(sx + 0.13, sy - 0.05, 2.92), {u3, up, v3}, Vec3(0.08, 0.26, 0.02)}, PartId::Trim, Vec3(0.95, 0.95, 0.95));
    }
    // trees in the corners of the verge, off the asphalt
    for (const Vec2 t : {Vec2(f.x0 + 0.8, f.y1 - 0.8), Vec2(f.x1 - 0.8, f.y1 - 0.8)})
        if (t.x < ax0 - 0.5 || t.x > ax1 + 0.5) g.treeSpots.push_back(Vec3(f.W(t.x, t.y).x, rng.range(0.7, 1.0), f.W(t.x, t.y).y));
    appendKit(kit, outParts, /*draped=*/true);
    g.padMesh = std::move(m);
    g.color = Vec3(1, 1, 1);
    if (std::getenv("RT_OPENLOT_DEBUG"))
        std::printf("[openlot] car park %.0f x %.0f at %.0f %.0f: %d aisles, %d stalls\n", W, f.y1 - f.y0, f.c.x, f.c.y, nMod, stalls);
    return true;
}

// THE PLAYGROUND: a fenced play area off the street, its gate on a walk from the pavement. Inside, on a rubber
// safety surface: a swing set, a slide tower, a climbing frame, a seesaw and a sandpit as the rectangle has room for
// them, laid out along it; benches for the grown-ups inside the fence, trees outside it. Type "park": a place to go.
static bool sculptPlayground(LotBuilding& g, const Poly2& poly, Vec2 frontage, uint32_t seed, std::vector<RenderMesh>* outParts) {
    if (!outParts || outParts->size() < kLotPartSlots) return false;
    OpenFrame f;
    if (!openFrameOf(poly, frontage, 1.2, 10.5, 8.0, f)) return false;
    Hash rng(mix(seed, 0x91A76u));
    // the play area: the rectangle, capped at 34 x 26 m and centred on it (a big lot keeps a lawn round it)
    {
        const Real W = std::min(f.x1 - f.x0, Real(34)), D = std::min(f.y1 - f.y0, Real(26));
        const Real cx = (f.x0 + f.x1) * 0.5;
        f.x0 = cx - W * 0.5; f.x1 = cx + W * 0.5;
        f.y1 = f.y0 + D;
    }
    const Real W = f.x1 - f.x0, D = f.y1 - f.y0;
    const Vec3 up(0, 1, 0), u3(f.u.x, 0, f.u.y), v3(f.v.x, 0, f.v.y);
    RenderMesh m;
    auto quad = [&](Real a0, Real b0, Real a1, Real b1, Real h, const Vec3& col) {
        MeshBuilder::emitQuad(m, f.P(a0, b0, h), f.P(a0, b1, h), f.P(a1, b1, h), f.P(a1, b0, h), up, col);
    };
    const Vec3 rubber = rng.unit() < 0.5 ? Vec3(0.42, 0.13, 0.09) : Vec3(0.10, 0.22, 0.38);
    const Vec3 rubber2 = rng.unit() < 0.5 ? Vec3(0.12, 0.30, 0.14) : Vec3(0.55, 0.40, 0.08);
    quad(f.x0 + 0.3, f.y0 + 0.3, f.x1 - 0.3, f.y1 - 0.3, 0.04, rubber);
    {
        Poly2 s{f.W(f.x0, f.y0), f.W(f.x1, f.y0), f.W(f.x1, f.y1), f.W(f.x0, f.y1)};
        ensureCCW(s);
        g.sealed.push_back(s);
    }
    const Real gx = (f.x0 + f.x1) * 0.5;
    openWalkOut(g, m, poly, f, gx, 0.9, Vec3(0.72, 0.68, 0.60));
    g.walks.push_back({f.W(gx, f.y0), f.W(gx, f.y0 + D * 0.5), 1.8, false});
    BuildingMesh kit;
    openFence(g, kit, f, gx, 1.8, 0);
    // the equipment, in bright powder-coated colours, in slots along the rectangle behind a strip for the benches
    const Vec3 paint[4] = {{0.80, 0.18, 0.12}, {0.95, 0.72, 0.10}, {0.12, 0.40, 0.75}, {0.15, 0.55, 0.25}};
    const Vec3 steel(0.55, 0.57, 0.60), timber(0.42, 0.30, 0.18);
    auto box = [&](Real x, Real y, Real h, Real sx, Real sy, Real sh, PartId part, const Vec3& col) {
        emitBox(kit, Scope{f.P(x, y, h), {u3, up, v3}, Vec3(sx, sh, sy)}, part, col);
    };
    // a bar between two frame points at heights (the swing's A-frame legs, the slide's chute)
    auto bar = [&](Real xa, Real ya, Real ha, Real xb, Real yb, Real hb, Real t, PartId part, const Vec3& col) {
        const Vec3 A = f.P(xa, ya, ha), B = f.P(xb, yb, hb);
        Vec3 d = B - A;
        const Real L = d.length();
        if (L < 1e-3) return;
        d = d / L;
        Vec3 side = cross(d, up);
        if (side.length() < 1e-3) side = u3;
        side = normalize(side);
        const Vec3 n = cross(side, d);
        emitBox(kit, Scope{A - side * (t * 0.5) - n * (t * 0.5), {d, n, side}, Vec3(L, t, t)}, part, col);
    };
    std::vector<int> items = {0, 1, 2, 3, 4};   // swings, slide, climbing frame, seesaw, sandpit
    for (std::size_t i = items.size(); i > 1; --i) std::swap(items[i - 1], items[static_cast<std::size_t>(rng.unit() * i) % i]);
    const Real by0 = f.y0 + 2.1;   // the benches' strip by the gate
    Real cursor = f.x0 + 1.2, rowY = by0, rowDeep = 0;   // laid in rows along the street, a second row behind on a deep lot
    int placed = 0;
    std::vector<std::pair<Vec2, Vec2>> slots;   // what went where (frame), for the trees
    for (int it : items) {
        const Real need = it == 0 ? 6.0 : it == 1 ? 6.4 : it == 2 ? 4.6 : it == 3 ? 4.6 : 4.0;
        const Real deep = it == 0 ? 5.0 : it == 1 ? 6.0 : it == 2 ? 4.0 : it == 3 ? 2.4 : 4.0;
        if (cursor + need > f.x1 - 0.8 && rowDeep > 0) { rowY += rowDeep + 1.4; cursor = f.x0 + 1.2; rowDeep = 0; }
        if (cursor + need > f.x1 - 0.8 || rowY + deep > f.y1 - 0.6) continue;
        const Real x = cursor, y = rowY;
        const Vec3 col = paint[(placed + static_cast<int>(rng.unit() * 4)) % 4];
        if (it == 0) {   // SWINGS: two A-frames, a beam, two seats on chains, a mat under each
            const Real bx0 = x + 0.5, bx1 = x + need - 0.5, cyy = y + deep * 0.5;
            for (Real ex : {bx0, bx1}) {
                bar(ex, cyy - 1.2, 0, ex, cyy, 2.5, 0.09, PartId::Metal, col);
                bar(ex, cyy + 1.2, 0, ex, cyy, 2.5, 0.09, PartId::Metal, col);
            }
            bar(bx0, cyy, 2.5, bx1, cyy, 2.5, 0.11, PartId::Metal, col);
            for (Real sx : {bx0 + (bx1 - bx0) * 0.3, bx0 + (bx1 - bx0) * 0.7}) {
                for (Real dx : {Real(-0.22), Real(0.22)}) bar(sx + dx, cyy, 2.45, sx + dx, cyy, 0.55, 0.02, PartId::Metal, steel);
                box(sx - 0.25, cyy - 0.12, 0.5, 0.5, 0.24, 0.05, PartId::Furniture, Vec3(0.08, 0.08, 0.09));
                quad(sx - 0.7, cyy - 2.0, sx + 0.7, cyy + 2.0, 0.055, rubber2);
            }
        } else if (it == 1) {   // SLIDE TOWER: four posts, a deck at 1.5 m, a roof, a ladder, a chute down
            const Real tx = x + 0.4, ty = y + deep * 0.5 - 0.8;
            for (Real dx : {Real(0), Real(1.5)})
                for (Real dy : {Real(0), Real(1.5)}) box(tx + dx, ty + dy, 0, 0.12, 0.12, 2.9, PartId::Metal, col);
            box(tx, ty, 1.45, 1.62, 1.62, 0.08, PartId::Wood, timber);
            box(tx - 0.1, ty - 0.1, 2.9, 1.82, 1.82, 0.12, PartId::Trim, paint[(placed + 1) % 4]);
            box(tx - 0.05, ty + 1.5, 1.53, 1.6, 0.05, 0.6, PartId::Metal, col);   // the guard
            for (int r = 0; r < 5; ++r) box(tx + 0.4, ty - 0.55 - 0.02, 0.3 * (r + 1), 0.7, 0.05, 0.05, PartId::Metal, steel);   // ladder
            for (Real dx : {Real(0.38), Real(1.12)}) bar(tx + dx, ty - 0.6, 0, tx + dx, ty, 1.5, 0.06, PartId::Metal, steel);
            bar(tx + 1.62, ty + 0.4, 1.45, tx + 5.6, ty + 0.4, 0.3, 0.06, PartId::Metal, steel);   // the chute's rails
            bar(tx + 1.62, ty + 1.2, 1.45, tx + 5.6, ty + 1.2, 0.3, 0.06, PartId::Metal, steel);
            {
                RenderMesh chute;   // the bed, polished steel
                MeshBuilder::emitQuad(chute, f.P(tx + 1.62, ty + 0.42, 1.42), f.P(tx + 1.62, ty + 1.18, 1.42),
                                      f.P(tx + 5.6, ty + 1.18, 0.27), f.P(tx + 5.6, ty + 0.42, 0.27), up, Vec3(0.75, 0.76, 0.78));
                MeshBuilder::emitQuad(chute, f.P(tx + 5.6, ty + 0.42, 0.27), f.P(tx + 5.6, ty + 1.18, 0.27),
                                      f.P(tx + 1.62, ty + 1.18, 1.42), f.P(tx + 1.62, ty + 0.42, 1.42), up * -1.0, Vec3(0.75, 0.76, 0.78));
                MeshBuilder::append((*outParts)[drapedSlot(PartId::Metal)], chute);
            }
        } else if (it == 2) {   // CLIMBING FRAME: a lattice cube of bars
            const Real cx0 = x + 0.3, cy0 = y;
            for (int i = 0; i <= 3; ++i)
                for (int j = 0; j <= 3; ++j) box(cx0 + i * 1.3 - 0.04, cy0 + j * 1.3 - 0.04, 0, 0.08, 0.08, 2.4, PartId::Metal, col);
            for (int lv = 1; lv <= 3; ++lv)
                for (int i = 0; i <= 3; ++i) {
                    box(cx0 + i * 1.3 - 0.03, cy0, lv * 0.8, 0.06, 3.9, 0.06, PartId::Metal, steel);
                    box(cx0, cy0 + i * 1.3 - 0.03, lv * 0.8, 3.9, 0.06, 0.06, PartId::Metal, steel);
                }
        } else if (it == 3) {   // SEESAW: a pivot, a plank tipped one way, two handles
            const Real cx = x + need * 0.5, cyy = y + deep * 0.5;
            box(cx - 0.2, cyy - 0.3, 0, 0.4, 0.6, 0.45, PartId::Metal, steel);
            bar(cx - 1.9, cyy, 0.18, cx + 1.9, cyy, 0.78, 0.22, PartId::Metal, col);
            for (Real dx : {Real(-1.5), Real(1.5)}) bar(cx + dx, cyy, 0.48 + dx * 0.16, cx + dx, cyy, 0.9 + dx * 0.16, 0.04, PartId::Metal, steel);
        } else {   // SANDPIT: a timber kerb and the sand
            box(x, y, 0, 4.0, 0.2, 0.3, PartId::Wood, timber);
            box(x, y + 3.8, 0, 4.0, 0.2, 0.3, PartId::Wood, timber);
            box(x, y, 0, 0.2, 4.0, 0.3, PartId::Wood, timber);
            box(x + 3.8, y, 0, 0.2, 4.0, 0.3, PartId::Wood, timber);
            quad(x + 0.2, y + 0.2, x + 3.8, y + 3.8, 0.2, Vec3(0.72, 0.62, 0.42));
        }
        slots.push_back({Vec2(x, y), Vec2(x + need, y + deep)});
        cursor += need + 1.4;
        rowDeep = std::max(rowDeep, deep);
        ++placed;
    }
    if (placed < 2) return false;   // a fence round one swing is not a playground
    // the benches, inside the fence by the gate, facing the play area (sat on by the citysim)
    for (Real bx : {f.x0 + W * 0.22, f.x1 - W * 0.22}) {
        if (std::fabs(bx - gx) < 2.0) continue;
        const Vec2 at = f.W(bx, f.y0 + 1.0);
        g.furniture.push_back(outdoorPiece(Piece::Bench, posHash(at), at, f.v, 0.04, true));
        g.sitSpots.push_back({at + f.v * 0.33, f.v, 0.45});
    }
    // trees round the outside of the fence, where the lot has room
    for (int k = 0; k < 8; ++k) {
        const Vec2 t = f.W(k < 4 ? (k % 2 ? f.x1 + 2.2 : f.x0 - 2.2) : f.x0 + W * (0.2 + 0.6 * (k % 2)),
                           k < 4 ? f.y0 + D * (k < 2 ? 0.3 : 0.75) : f.y1 + 2.4);
        Poly2 in = inset(poly, 1.6);
        if (in.size() >= 3 && pointInPolygon(in, t)) g.treeSpots.push_back(Vec3(t.x, rng.range(0.8, 1.2), t.y));
    }
    {   // a lawn round it on a big lot, for the citysim's lawn activities
        LotBuilding::Area ar;
        ar.kind = "lawn";
        ar.center = f.W(gx, f.y0 + D * 0.5);
        ar.axis = f.u;
        ar.halfL = W * 0.5 - 1.0;
        ar.halfW = D * 0.5 - 1.0;
        if (ar.halfL > 4 && ar.halfW > 3) g.areas.push_back(ar);
    }
    appendKit(kit, outParts, /*draped=*/true);
    g.padMesh = std::move(m);
    g.color = Vec3(1, 1, 1);
    if (std::getenv("RT_OPENLOT_DEBUG"))
        std::printf("[openlot] playground %.0f x %.0f at %.0f %.0f: %d pieces\n", W, D, f.c.x, f.c.y, placed);
    return true;
}

// THE COMMUNITY GARDEN: allotment beds behind a picket fence. Gravel paths between rows of raised timber beds (1.2 x
// 3.0 m), each planted with one crop (leafy rows, bean canes, flowers, squash); a tool shed in a back corner, a water
// butt beside it, compost bins, a bench by the gate. Type "park": a place to go.
static bool sculptCommunityGarden(LotBuilding& g, const Poly2& poly, Vec2 frontage, uint32_t seed, std::vector<RenderMesh>* outParts) {
    if (!outParts || outParts->size() < kLotPartSlots) return false;
    OpenFrame f;
    if (!openFrameOf(poly, frontage, 1.0, 9.0, 7.5, f)) return false;
    Hash rng(mix(seed, 0x6A2DE1u));
    const Real W = f.x1 - f.x0, D = f.y1 - f.y0;
    const Vec3 up(0, 1, 0), u3(f.u.x, 0, f.u.y), v3(f.v.x, 0, f.v.y);
    RenderMesh m;
    auto quad = [&](Real a0, Real b0, Real a1, Real b1, Real h, const Vec3& col) {
        MeshBuilder::emitQuad(m, f.P(a0, b0, h), f.P(a0, b1, h), f.P(a1, b1, h), f.P(a1, b0, h), up, col);
    };
    const Vec3 gravel(0.30, 0.27, 0.22), soil(0.17, 0.12, 0.08), timber(0.40, 0.29, 0.18);   // (linear: a pale gravel read white)
    quad(f.x0 + 0.2, f.y0 + 0.2, f.x1 - 0.2, f.y1 - 0.2, 0.03, gravel);
    {
        Poly2 s{f.W(f.x0, f.y0), f.W(f.x1, f.y0), f.W(f.x1, f.y1), f.W(f.x0, f.y1)};
        ensureCCW(s);
        g.sealed.push_back(s);
    }
    const Real gx = (f.x0 + f.x1) * 0.5;
    openWalkOut(g, m, poly, f, gx, 0.8, gravel);
    BuildingMesh kit;
    openFence(g, kit, f, gx, 1.4, 1);
    auto box = [&](Real x, Real y, Real h, Real sx, Real sy, Real sh, PartId part, const Vec3& col) {
        emitBox(kit, Scope{f.P(x, y, h), {u3, up, v3}, Vec3(sx, sh, sy)}, part, col);
    };
    // the shed's corner is kept clear of beds
    const bool shedLeft = rng.unit() < 0.5;
    const Real shx = shedLeft ? f.x0 + 0.8 : f.x1 - 0.8 - 2.6, shy = f.y1 - 0.8 - 2.0;
    // THE BEDS: columns of 1.2 m beds along v, 0.9 m paths between them; the middle column left as the main path
    const Real bedW = 1.2, bedL = D >= 12.0 ? 3.0 : 2.0, path = 0.9;
    const int cols = static_cast<int>((W - 1.0) / (bedW + path));
    const int rows = static_cast<int>((D - 2.6) / (bedL + path));
    const Real bx0 = gx - cols * (bedW + path) * 0.5 + path * 0.5;
    const Vec3 crops[6] = {{0.20, 0.42, 0.16}, {0.30, 0.50, 0.18}, {0.16, 0.34, 0.20},
                           {0.70, 0.28, 0.32}, {0.82, 0.64, 0.18}, {0.36, 0.46, 0.14}};
    int beds = 0;
    for (int i = 0; i < cols; ++i) {
        const Real x = bx0 + i * (bedW + path);
        if (std::fabs(x + bedW * 0.5 - gx) < 1.2) continue;   // the main path from the gate
        for (int j = 0; j < rows; ++j) {
            const Real y = f.y0 + 1.8 + j * (bedL + path);
            if (x + bedW > shx - 0.6 && x < shx + 2.6 + 0.6 && y + bedL > shy - 0.6) continue;   // the shed's corner
            box(x, y, 0, bedW, bedL, 0.35, PartId::Wood, timber);
            box(x + 0.06, y + 0.06, 0.33, bedW - 0.12, bedL - 0.12, 0.03, PartId::Detail, soil);   // (a bed's top is no ground)
            const int crop = static_cast<int>(rng.unit() * 7.99);
            if (crop == 7) { ++beds; continue; }   // a bed lying fallow
            if (crop == 6) {   // BEAN CANES: wigwams of thin poles, leafy at the foot
                for (int k = 0; k < 2; ++k) {
                    const Real cy = y + bedL * (0.28 + 0.44 * k), cx = x + bedW * 0.5;
                    for (int p2 = 0; p2 < 4; ++p2) {
                        const Real a2 = 1.5707963 * p2 + 0.4;
                        const Vec3 A = f.P(cx + std::cos(a2) * 0.45, cy + std::sin(a2) * 0.45, 0.36), B = f.P(cx, cy, 2.0);
                        Vec3 d = B - A; const Real L = d.length(); d = d / L;
                        const Vec3 s = normalize(cross(d, up)), n = cross(s, d);
                        emitBox(kit, Scope{A - s * 0.015 - n * 0.015, {d, n, s}, Vec3(L, 0.03, 0.03)}, PartId::Wood, Vec3(0.55, 0.45, 0.30));
                    }
                    emitSoftBox(kit, Scope{f.P(cx - 0.4, cy - 0.4, 0.36), {u3, up, v3}, Vec3(0.8, 0.9, 0.8)}, PartId::Foliage, crops[1], 0u);
                }
            } else {   // ROWS of a crop across the bed (flowers and squash in fewer, fuller clumps)
                const int n = crop >= 3 && crop <= 4 ? 3 : 5;
                const Real hgt = crop == 5 ? 0.55 : crop >= 3 ? 0.45 : rng.range(0.22, 0.38);
                for (int k = 0; k < n; ++k) {
                    const Real cy = y + 0.15 + (bedL - 0.3) * (k + 0.5) / n;
                    emitSoftBox(kit, Scope{f.P(x + 0.12, cy - 0.18, 0.36), {u3, up, v3}, Vec3(bedW - 0.24, hgt, 0.36)},
                                PartId::Foliage, crops[crop], 0u);
                }
            }
            ++beds;
        }
    }
    if (beds < 3) return false;
    // THE SHED: a timber box under a pent roof, a door on its gate side
    box(shx, shy, 0, 2.6, 2.0, 2.1, PartId::Wood, Vec3(0.30, 0.38, 0.30));
    {
        RenderMesh roof;
        MeshBuilder::emitQuad(roof, f.P(shx - 0.15, shy - 0.2, 2.35), f.P(shx + 2.75, shy - 0.2, 2.35), f.P(shx + 2.75, shy + 2.2, 2.05),
                              f.P(shx - 0.15, shy + 2.2, 2.05), up, Vec3(0.20, 0.20, 0.21));
        MeshBuilder::emitQuad(roof, f.P(shx - 0.15, shy + 2.2, 2.0), f.P(shx + 2.75, shy + 2.2, 2.0), f.P(shx + 2.75, shy - 0.2, 2.3),
                              f.P(shx - 0.15, shy - 0.2, 2.3), up * -1.0, Vec3(0.20, 0.20, 0.21));
        MeshBuilder::append((*outParts)[drapedSlot(PartId::Roof)], roof);
        box(shx, shy - 0.05, 2.1, 2.6, 0.05, 0.25, PartId::Wood, Vec3(0.30, 0.38, 0.30));   // the gable over the door
        box(shx + 0.9, shy - 0.06, 0, 0.85, 0.04, 1.85, PartId::Door, Vec3(0.24, 0.18, 0.12));
    }
    {   // the water butt (a green drum) and two compost bins (slatted boxes)
        const Real wx = shedLeft ? shx + 3.1 : shx - 0.8, wy = shy + 1.0;
        MeshBuilder::append((*outParts)[drapedSlot(PartId::Metal)],
                            latheMesh(f.P(wx, wy, 0), {{0.32, 0.0}, {0.34, 0.9}, {0.30, 0.95}, {0.0, 0.95}}, 12, Vec3(0.12, 0.28, 0.14)));
        for (int k = 0; k < 2; ++k) {
            const Real cx = shedLeft ? f.x1 - 1.0 - 1.1 * (k + 1) : f.x0 + 1.0 + 1.1 * k, cy = f.y1 - 1.1;
            for (int s = 0; s < 4; ++s) box(cx, cy, 0.05 + s * 0.24, 1.0, 1.0, 0.16, PartId::Wood, timber * 0.8);
            box(cx + 0.05, cy + 0.05, 0.86, 0.9, 0.9, 0.03, PartId::Detail, Vec3(0.20, 0.15, 0.08));
        }
    }
    {   // the bench by the gate, looking up the main path
        const Vec2 at = f.W(gx + 1.6, f.y0 + 0.9);
        g.furniture.push_back(outdoorPiece(Piece::Bench, posHash(at), at, f.v, 0.03, true));
        g.sitSpots.push_back({at + f.v * 0.33, f.v, 0.45});
    }
    g.walks.push_back({f.W(gx, f.y0), f.W(gx, f.y1 - 1.5), 1.6, false});
    for (const Vec2 t : {Vec2(f.x0 - 2.0, f.y1 - 1.0), Vec2(f.x1 + 2.0, f.y1 - 1.0)}) {
        const Vec2 w = f.W(t.x, t.y);
        Poly2 in = inset(poly, 1.6);
        if (in.size() >= 3 && pointInPolygon(in, w)) g.treeSpots.push_back(Vec3(w.x, rng.range(0.8, 1.2), w.y));
    }
    appendKit(kit, outParts, /*draped=*/true);
    g.padMesh = std::move(m);
    g.color = Vec3(1, 1, 1);
    if (std::getenv("RT_OPENLOT_DEBUG"))
        std::printf("[openlot] garden %.0f x %.0f at %.0f %.0f: %d beds\n", W, D, f.c.x, f.c.y, beds);
    return true;
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
    (void)ground;
    auto gy = [](const Vec2&) { return Real(0); };   // ground-relative padMesh (kDrapedPartBase)
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

// A WALK FROM EVERY DOOR TO THE PAVEMENT (Glenn, 2026-09-21: "If it can't [face the
// street] it would be nice to have a walk way to the door and the walk way should be on
// the ground leading to the house's front door").
//
// sculptYard below already lays one — "a draped pavement ribbon, door to lot line" — but
// only for a house with a yard, and only as far as the LOT LINE, which is not where the
// pavement is: the block is inset from the kerb and a gap of ground lies between the two.
// This runs for every door that does not already open onto pavement, and walks from the
// threshold along the door's own outward normal until it reaches the sidewalk band's
// outer edge — `roadHalfWidth + sidewalkWidth` from the nearest centreline, the same line
// a paved lot's plate is pushed out to. A door already on the pavement (a tower built to
// the lot line) gets nothing: a 20 cm stub through the kerb is worse than no walk.
//
// The ribbon DRAPES: a joint every metre, each vertex ground-relative (kDrapedPartBase),
// so once the host lays it on the finished terrain it follows it instead of floating
// over a dip or cutting into a rise.
// WHAT A BUILDING MAY FRONT (Glenn, 2026-09-29: "buildings shouldn't face a freeway as a road since that's
// not an accessible road"): a door, its walk and a plaza's mouths reach for city streets. Freeway and ramp
// edges stay in the graph for what needs them (the deck-shadow test below) but are never frontage.
// ...and ON THE GROUND (Glenn, 2026-09-30: "assuming the city road is on the ground in front of the house"):
// a city street carried over the block on a bridge is no one's front door either.
static bool faces(const RoadEdge& e) { return e.klass != RoadClass::Freeway && e.klass != RoadClass::Ramp && e.layer == 0; }

void sculptDoorWalks(const LotBuilding& b, const RoadGraph* roads, Real sidewalkWidth,
                     const Poly2* block, std::vector<RenderMesh>* outParts) {
    if (!outParts || !roads || roads->edges.empty() || sidewalkWidth <= 0) return;
    // How far past the sidewalk band this point still is (0 = on it or beyond it).
    auto pastBand = [&](const Vec2& q) {
        Real best = Real(1e30), bestHw = 0;
        for (const RoadEdge& e : roads->edges) {
            if (e.a < 0 || e.b < 0 || e.a >= static_cast<int>(roads->nodes.size()) ||
                e.b >= static_cast<int>(roads->nodes.size())) continue;
            if (!faces(e)) continue;   // a building fronts a street, never a freeway or a ramp
            const Vec2& ra = roads->nodes[e.a].pos;
            const Vec2& rb = roads->nodes[e.b].pos;
            const Vec2 ab = rb - ra;
            const Real len2 = ab.lengthSquared();
            Real t = len2 > 1e-12 ? dot(q - ra, ab) / len2 : 0.0;
            t = std::max(Real(0), std::min(Real(1), t));
            const Real d = (q - (ra + ab * t)).length();
            if (d < best) { best = d; bestHw = e.width * 0.5; }
        }
        return std::max(Real(0), best - (bestHw + sidewalkWidth));
    };
    // GROUND-RELATIVE (kDrapedPartBase): y is the lift over the ground; the host
    // lays the ribbon on the finished terrain.
    auto gy = [](const Vec2&) { return Real(0); };
    RenderMesh& path = (*outParts)[drapedSlot(PartId::Path)];
    constexpr Real kHalfWidth = 0.6, kStep = 1.0, kMaxWalk = 30.0, kLift = 0.05;
    for (const BuildingUnit& u : b.units)
        for (const DoorSpec& d : u.doors) {
            if (d.normal.length() < Real(1e-6)) continue;
            const Vec2 f = normalize(d.normal);
            // Walk out until the ground under the next step is the sidewalk: past the sidewalk
            // band's inner edge AND out of the block. The band is measured from the graph's
            // centreline and half-width, which on a lane-built city is the lot-clearance width
            // (padded) — it reads the sidewalk as starting ~1.5 m inside a block that now begins
            // right behind the drawn sidewalk, and a walk stopped there left a strip of grass
            // between its end and the pavement. The block edge IS the back of the sidewalk.
            auto inBlock = [&](const Vec2& q) { return block && block->size() >= 3 && pointInPolygon(*block, q); };
            auto onPavement = [&](const Vec2& q) { return pastBand(q) <= Real(0.05) && !inBlock(q); };
            if (onPavement(d.foot + f * Real(0.8))) continue;   // already at the pavement
            Real len = 0;
            while (len < kMaxWalk && !onPavement(d.foot + f * len)) len += Real(0.25);
            if (len < Real(0.8) || len >= kMaxWalk) continue;   // never reaches a street
            len += Real(0.3);                                    // tuck the end under the sidewalk's edge
            const Vec2 perp(-f.y, f.x);
            const int segs = std::max(1, static_cast<int>(std::ceil(len / kStep)));
            for (int si = 0; si < segs; ++si) {
                const Vec2 q0 = d.foot + f * (len * si / segs);
                const Vec2 q1 = d.foot + f * (len * (si + 1) / segs);
                const Vec2 a0 = q0 - perp * kHalfWidth, b0 = q0 + perp * kHalfWidth;
                const Vec2 a1 = q1 - perp * kHalfWidth, b1 = q1 + perp * kHalfWidth;
                MeshBuilder::emitQuad(path, Vec3(a0.x, gy(a0) + kLift, a0.y), Vec3(b0.x, gy(b0) + kLift, b0.y),
                                      Vec3(b1.x, gy(b1) + kLift, b1.y), Vec3(a1.x, gy(a1) + kLift, a1.y),
                                      Vec3(0, 1, 0), Vec3(0.72, 0.70, 0.65));
            }
        }
}

// YARD SCULPTING: a house lot earns a FRONT WALK from its door to the street
// boundary, a clipped hedge along the front lot line (with a gap where the
// walk crosses), and a back-yard tree spot or two.
void sculptYard(LotBuilding& b, const Poly2& lotPoly, const Poly2& house,
                const Vec2& face, uint32_t seed, std::vector<RenderMesh>* outParts) {
    if (!outParts || house.size() < 3 || lotPoly.size() < 3 ||
        face.length() < 1e-6)
        return;
    Hash rng(mix(seed, 0xF00D5EEDu));
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
        // The front WALK: a pavement ribbon, door to lot line, GROUND-RELATIVE
        // (kDrapedPartBase) — the host lays it on the finished terrain. A joint
        // every metre so it can bend with the mesh's 2.7 m cells.
        RenderMesh& path = (*outParts)[drapedSlot(PartId::Path)];
        Vec2 perp(-f.y, f.x);
        const Real hw = 0.55;
        const int segs = std::max(1, static_cast<int>(std::ceil(tExit / 1.0)));
        for (int s = 0; s < segs; ++s) {
            Vec2 q0 = door + f * (tExit * s / segs);
            Vec2 q1 = door + f * (tExit * (s + 1) / segs);
            const Real y0 = 0.06, y1 = 0.06;
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
                // Bedded 5 cm: ground-relative, so the base sits in the grass
                // at every corner once draped.
                emitSoftBox(kit, Scope{Vec3(hc.x, -0.05, hc.y) - t3 * 0.9 -
                                           n3 * 0.3,
                                       {t3, up, n3}, Vec3(1.8, 0.70, 0.6)},
                            PartId::Foliage, hedgeCol, 0u);
            }
        }
        appendKit(kit, outParts, /*draped=*/true);
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
// PAVING for an urban lot (ADR-0086): the whole lot becomes one flat concrete
// plate at `paveY` — the sidewalk's top — with a concrete skirt down past the
// surrounding ground, exactly as the plaza's deck is built. The plate runs
// under the building too (hidden by its foundation and floor), so a drum, a
// court notch or a short row strip never shows grass inside its own site.
// Into BOTH LOD tiers: a paving that exists only up close is a lot that turns
// to grass at a distance.
void sculptPaving(const Poly2& lotIn, Real paveY,
                  const std::function<Real(Real, Real)>& ground,
                  std::vector<RenderMesh>* outParts, std::vector<RenderMesh>* outFlatParts) {
    Poly2 lot = lotIn;
    if (lot.size() < 3) return;
    ensureCCW(lot);
    auto gy = [&](const Vec2& v) { return ground ? ground(v.x, v.y) : paveY; };
    const Vec3 up(0, 1, 0), white(1, 1, 1);
    RenderMesh deck, skirt;
    for (const std::array<int, 3>& t : triangulatePolygon(lot))
        MeshBuilder::emitTri(deck, Vec3(lot[t[0]].x, paveY, lot[t[0]].y),
                             Vec3(lot[t[1]].x, paveY, lot[t[1]].y),
                             Vec3(lot[t[2]].x, paveY, lot[t[2]].y), up, white);
    for (std::size_t i = 0; i < lot.size(); ++i) {
        const Vec2& a = lot[i];
        const Vec2& e = lot[(i + 1) % lot.size()];
        const Vec2 d = e - a;
        if (d.length() < 1e-6) continue;
        const Vec2 n2 = normalize(Vec2(d.y, -d.x));   // CCW: right normal = outward
        // The skirt is UNCONDITIONAL and deep: the plate is a one-sided sheet, and
        // wherever the ground dips between two corners (a neighbour's lower pad,
        // the 1 m feather, a cross-street strip) an edge with no side under it
        // hangs in the air as a paper-thin slab — the owner's "wafer-thin walls".
        // A metre below the lower corner's ground, sampled at the mid-edge too.
        const Real lo = std::min({gy(a), gy(e), gy((a + e) * 0.5)}) - 1.0;
        MeshBuilder::emitQuad(skirt, Vec3(a.x, lo, a.y), Vec3(e.x, lo, e.y),
                              Vec3(e.x, paveY, e.y), Vec3(a.x, paveY, a.y),
                              Vec3(n2.x, 0, n2.y), white);
    }
    for (std::vector<RenderMesh>* parts : {outParts, outFlatParts}) {
        if (!parts || parts->size() <= static_cast<std::size_t>(PartId::Concrete)) continue;
        MeshBuilder::append((*parts)[static_cast<std::size_t>(PartId::Path)], deck);
        MeshBuilder::append((*parts)[static_cast<std::size_t>(PartId::Concrete)], skirt);
    }
}

// The FORECOURT of a tower in a plaza (ADR-0086 point 5, the 1961 New York
// model): the paved strip between the avenue and the tower, dressed the way
// Seagram's is — two square reflecting pools toward the front corners, a row
// of stone planters with trees along the tower's foot, benches between. The
// paving itself is the lot's plate (already laid at the sidewalk's height);
// this only stands things on it. `plaza` is the strip in world XZ, `f` the
// site frame (u along the avenue, v inward), `y` the plate's top.
void sculptForecourt(LotBuilding& b, const Poly2& plaza, const SiteFrame& f, Real y,
                     uint32_t seed, std::vector<RenderMesh>* outParts) {
    if (!outParts || plaza.size() < 3) return;
    Hash rng(mix(seed, 0x51A7A9EDu));
    Real x0 = 1e30, x1 = -1e30, y0 = 1e30, y1 = -1e30;
    for (const Vec2& w : plaza) {
        const Vec2 q = f.toFrame(w);
        x0 = std::min(x0, q.x); x1 = std::max(x1, q.x);
        y0 = std::min(y0, q.y); y1 = std::max(y1, q.y);
    }
    const Real W = x1 - x0, P = y1 - y0;
    if (W < 16 || P < 10) return;
    const Vec3 up(0, 1, 0);
    const Vec3 stone(0.72, 0.70, 0.66);
    const Vec3 u3(f.u.x, 0, f.u.y), v3(f.v.x, 0, f.v.y);
    BuildingMesh kit;
    auto at = [&](Real fx, Real fy) { const Vec2 w = f.toWorld({fx, fy}); return Vec3(w.x, y, w.y); };
    // Two square POOLS: a 0.42 m stone rim, water inside.
    const Real s = std::min(Real(7.5), std::min(W * 0.22, P * 0.36));
    if (s >= 3.0) {
        for (int side = 0; side < 2; ++side) {
            const Real cx0 = side ? x1 - W * 0.22 : x0 + W * 0.22;
            const Real cy0 = y0 + P * 0.46;
            const Vec3 o = at(cx0 - s * 0.5, cy0 - s * 0.5);
            const Real rim = 0.45;
            // Four rim walls around the basin.
            emitBox(kit, Scope{o, {u3, up, v3}, Vec3(s, 0.42, rim)}, PartId::Trim, stone);
            emitBox(kit, Scope{o + v3 * (s - rim), {u3, up, v3}, Vec3(s, 0.42, rim)}, PartId::Trim, stone);
            emitBox(kit, Scope{o + v3 * rim, {u3, up, v3}, Vec3(rim, 0.42, s - 2 * rim)}, PartId::Trim, stone);
            emitBox(kit, Scope{o + u3 * (s - rim) + v3 * rim, {u3, up, v3}, Vec3(rim, 0.42, s - 2 * rim)}, PartId::Trim, stone);
            // The water: a dark glassy sheet 0.3 m up the rim.
            RenderMesh water;
            const Vec3 w0 = o + u3 * rim + v3 * rim + up * 0.30;
            const Vec3 w1 = w0 + u3 * (s - 2 * rim), w2 = w1 + v3 * (s - 2 * rim), w3 = w0 + v3 * (s - 2 * rim);
            MeshBuilder::emitQuad(water, w0, w1, w2, w3, up, Vec3(0.036, 0.092, 0.136));
            MeshBuilder::append((*outParts)[static_cast<std::size_t>(PartId::Glass)], water);
        }
    }
    // PLANTERS along the tower's foot (the plaza's rear edge), trees in them.
    {
        const int np = std::max(2, std::min(6, static_cast<int>(W / 9)));
        for (int k = 0; k < np; ++k) {
            const Real fx = x0 + W * (k + 0.5) / np;
            const Real fy = y1 - 2.4;
            const Vec3 o = at(fx - 0.85, fy - 0.85);
            emitBox(kit, Scope{o, {u3, up, v3}, Vec3(1.7, 0.55, 1.7)}, PartId::Trim, stone * 0.9);
            const Vec2 w = f.toWorld({fx, fy});
            b.treeSpots.push_back(Vec3(w.x, rng.range(0.6, 0.85), w.y));
        }
    }
    // BENCHES facing the avenue, between the pools.
    {
        const Vec3 wood(0.45, 0.34, 0.22);
        const int nb = std::max(2, std::min(5, static_cast<int>(W / 10)));
        for (int k = 0; k < nb; ++k) {
            const Real fx = x0 + W * (k + 0.5) / nb;
            const Real fy = y0 + P * 0.62;
            // A backless PLAZA BENCH from the library (M2), facing the avenue.
            (void)wood;
            const Vec2 face(-f.v.x, -f.v.y);
            b.furniture.push_back(outdoorPiece(Piece::PlazaBench, posHash(f.toWorld({fx, fy})), f.toWorld({fx, fy}) - face * 0.275, face, y, false));
        }
    }
    appendKit(kit, outParts);
}

// A PARKING LOT'S PLAN, in its frame (x along the street, y in from it to the store): what sculptParking paints and
// surfaceStallsOf hands the drivers -- one plan, so a striped stall is a stall a car can take. Two layouts, the one
// with more stalls wins: rows running IN toward the doors (a deep lot: modules of [row][aisle][row] across it), or
// rows running ALONG the store (a shallow lot: the modules stacked from the street, a walk to the doors kept clear).
struct LotPlan {
    Real ax0 = 0, ax1 = 0, ay0 = 0, ys0 = 0, ys1 = 0, cx = 0;   // the asphalt, the street-side aisle, the fire lane
    struct Rect { Real x0, y0, x1, y1; };
    std::vector<Rect> stripes, islands;                       // painted lines; planted islands (a curb round)
    std::vector<Vec2> trees, poles;                           // in the islands; light poles
    struct Stall { Vec2 at, face; };
    std::vector<Stall> stalls;
    struct Corral { Vec2 at; bool alongX; };                  // a cart corral, in a stall
    std::vector<Corral> corrals;
};
LotPlan lotPlan(Real x0, Real x1, Real y0, Real y1) {
    constexpr Real row = 5.4, aisle = 7.2, mod = row * 2 + aisle, stall = 2.7, isl = 2.4;
    auto planIn = [&]() {
        LotPlan L;
        L.ax0 = x0 + 1.5; L.ax1 = x1 - 1.5; L.ay0 = y0 + 2.5;
        L.ys1 = y1 - 8.0;          // the fire lane along the storefront
        L.ys0 = L.ay0 + 6.5;       // a drive aisle along the street
        L.cx = (x0 + x1) * 0.5;
        const int nMod = static_cast<int>((L.ax1 - L.ax0 - 1.0) / mod);
        if (nMod < 1 || L.ys1 - L.ys0 <= 2 * isl + 3 * stall) return L;
        const Real fx0 = L.cx - nMod * mod * 0.5;
        const Real r0 = L.ys0 + isl, r1 = L.ys1 - isl;
        const int nStall = static_cast<int>((r1 - r0) / stall);
        const Real s0 = (r0 + r1) * 0.5 - nStall * stall * 0.5;
        for (int m = 0; m < nMod; ++m) {
            const Real xm = fx0 + m * mod;
            for (int side = 0; side < 2; ++side) {
                const Real ra = side == 0 ? xm : xm + row + aisle, rb = ra + row;
                for (int k = 0; k <= nStall; ++k) L.stripes.push_back({ra + 0.3, s0 + k * stall - 0.06, rb, s0 + k * stall + 0.06});
                for (int end = 0; end < 2; ++end) {
                    const Real b0 = end == 0 ? s0 - isl + 0.2 : s0 + nStall * stall + 0.2;
                    L.islands.push_back({ra + 0.2, b0, rb - 0.2, b0 + isl - 0.4});
                    if (end == 0 || m % 2 == side) L.trees.push_back({(ra + rb) * 0.5, b0 + (isl - 0.4) * 0.5});
                }
                const Vec2 face(side == 0 ? -1.0 : 1.0, 0.0);   // nose in, off the aisle
                for (int k = 0; k < nStall; ++k) {
                    if (side == 1 && k == nStall / 2) {          // the module's cart corral
                        L.corrals.push_back({{ra + row * 0.5, s0 + (k + 0.5) * stall}, false});
                        continue;
                    }
                    L.stalls.push_back({{ra + row * 0.5 + 0.15, s0 + (k + 0.5) * stall}, face});
                }
            }
        }
        for (int m = 0; m <= nMod; ++m) {   // light poles on every module seam
            const int nPole = std::max(1, static_cast<int>((r1 - r0) / 26.0));
            for (int k = 0; k < nPole; ++k) L.poles.push_back({fx0 + m * mod, r0 + (r1 - r0) * (k + 0.5) / nPole});
        }
        return L;
    };
    auto planAlong = [&]() {
        LotPlan L;
        L.ax0 = x0 + 1.5; L.ax1 = x1 - 1.5; L.ay0 = y0 + 2.5;
        L.ys1 = y1 - 8.0;
        L.ys0 = L.ay0 + 6.5;
        L.cx = (x0 + x1) * 0.5;
        const Real r0 = L.ax0 + isl, r1 = L.ax1 - isl;
        if (r1 - r0 < 6 * stall) return L;
        const int nStall = static_cast<int>((r1 - r0) / stall);
        const Real s0 = (r0 + r1) * 0.5 - nStall * stall * 0.5;
        // the rows, from the street-side aisle in: two back to back (the first noses away from the aisle before it,
        // the second from the aisle after it), then that aisle -- the last pair's aisle is the fire lane
        struct R { Real y0; int nose; };
        std::vector<R> rows;
        for (Real y = L.ys0; y + 2 * row <= L.ys1 + 1e-6; y += 2 * row + aisle) {
            rows.push_back({y, 1});
            rows.push_back({y + row, -1});
        }
        if (!rows.empty() && L.ys1 - (rows.back().y0 + row) >= aisle + row) rows.push_back({L.ys1 - row, -1});   // one more on the lane
        int ri = 0;
        for (const R& r : rows) {
            const Real ya = r.y0, yb = r.y0 + row;
            // stripes from the aisle side in, 0.3 short of the back
            const Real sy0 = r.nose > 0 ? ya : ya + 0.3, sy1 = r.nose > 0 ? yb - 0.3 : yb;   // (0.3 short of the back: the nose end)
            for (int k = 0; k <= nStall; ++k) {
                const Real x = s0 + k * stall;
                if (std::fabs(x - L.cx) < 3.5) continue;
                L.stripes.push_back({x - 0.06, sy0, x + 0.06, sy1});
            }
            for (int end = 0; end < 2; ++end) {
                const Real b0 = end == 0 ? s0 - isl + 0.2 : s0 + nStall * stall + 0.2;
                L.islands.push_back({b0, ya + 0.2, b0 + isl - 0.4, yb - 0.2});
                L.trees.push_back({b0 + (isl - 0.4) * 0.5, (ya + yb) * 0.5});
            }
            const Vec2 face(0.0, static_cast<Real>(r.nose));
            for (int k = 0; k < nStall; ++k) {
                const Real x = s0 + (k + 0.5) * stall;
                if (std::fabs(x - L.cx) < 3.5 + stall * 0.5) continue;   // the walk to the doors
                if (k == nStall / 4 && (ri & 1)) { L.corrals.push_back({{x, (ya + yb) * 0.5}, true}); continue; }
                L.stalls.push_back({{x, (ya + yb) * 0.5 + (r.nose > 0 ? 0.15 : -0.15)}, face});
            }
            ++ri;
        }
        // the walk to the doors: a crossing at the centre over every aisle between rows, poles on the back-to-back seams
        for (std::size_t j = 0; j + 1 < rows.size(); ++j)
            if (rows[j].nose < 0 && rows[j + 1].nose > 0 && rows[j + 1].y0 - (rows[j].y0 + row) > 3)
                for (int k = -2; k <= 2; ++k)
                    L.stripes.push_back({L.cx + k * 0.9 - 0.25, rows[j].y0 + row + 0.5, L.cx + k * 0.9 + 0.25, rows[j + 1].y0 - 0.5});
        const int nPole = std::max(1, static_cast<int>((r1 - r0) / 26.0));
        for (std::size_t j = 0; j + 1 < rows.size(); ++j) {
            if (!(rows[j].nose > 0 && rows[j + 1].nose < 0)) continue;   // a seam: two rows back to back
            const Real ys = rows[j].y0 + row;
            for (int k = 0; k < nPole; ++k) L.poles.push_back({r0 + (r1 - r0) * (k + 0.5) / nPole, ys});
        }
        if (rows.size() < 2)   // no seam: poles in the islands at the row's ends
            for (const LotPlan::Rect& is : L.islands) L.poles.push_back({(is.x0 + is.x1) * 0.5, (is.y0 + is.y1) * 0.5 + 1.0});
        return L;
    };
    LotPlan a = planIn(), b = planAlong();
    return b.stalls.size() > a.stalls.size() ? b : a;
}

// THE BIG BOX'S PARKING LOT (Glenn, 2026-10-01: "Big box stores like Costco or Bestbuy"): the strip between the
// street and the store, on the lot's plate. Asphalt; double-loaded aisles running toward the doors, stalls 2.7 m
// wide and 5.4 deep striped white; planted islands at every row's ends; light poles on the seams between modules
// (their lamps glow at night); a cart corral in each module; a fire lane along the storefront with a crosswalk
// to the doors; and the chain's pylon sign by the street. `f` the site frame (u along the street, v inward),
// `y` the plate's top, `bp` the store's params (its chain colour).
void sculptParking(LotBuilding& b, const Poly2& lotP, const SiteFrame& f, Real y, const BuildingParams& bp,
                   uint32_t seed, std::vector<RenderMesh>* outParts) {
    if (!outParts || lotP.size() < 3 || outParts->size() <= static_cast<std::size_t>(PartId::LitBand)) return;
    Hash rng(mix(seed, 0x9A4C1A7u));
    Real x0 = 1e30, x1 = -1e30, y0 = 1e30, y1 = -1e30;
    for (const Vec2& w : lotP) {
        const Vec2 q = f.toFrame(w);
        x0 = std::min(x0, q.x); x1 = std::max(x1, q.x);
        y0 = std::min(y0, q.y); y1 = std::max(y1, q.y);
    }
    const Real W = x1 - x0, P = y1 - y0;
    if (W < 20 || P < 18) return;
    const Vec3 up(0, 1, 0);
    const Vec3 u3(f.u.x, 0, f.u.y), v3(f.v.x, 0, f.v.y);
    auto at = [&](Real fx, Real fy, Real dy = 0) { const Vec2 w = f.toWorld({fx, fy}); return Vec3(w.x, y + dy, w.y); };
    BuildingMesh kit;
    RenderMesh asphalt, paint;
    auto flat = [&](RenderMesh& m, Real a0, Real b0, Real a1, Real b1, Real dy, const Vec3& col) {
        MeshBuilder::emitQuad(m, at(a0, b0, dy), at(a0, b1, dy), at(a1, b1, dy), at(a1, b0, dy), up, col);
    };
    const LotPlan L = lotPlan(x0, x1, y0, y1);
    // the asphalt: all but a planted border along the street and the sides
    flat(asphalt, L.ax0, L.ay0, L.ax1, y1, 0.02, Vec3(0.16, 0.16, 0.18));   // the vertex colour IS the asphalt
    // the fire lane along the storefront, its crosswalk to the doors
    for (int k = -5; k <= 5; ++k)
        flat(paint, L.cx + k * 0.9 - 0.25, L.ys1 + 0.5, L.cx + k * 0.9 + 0.25, y1 - 0.3, 0.04, Vec3(0.92, 0.92, 0.90));
    flat(paint, L.ax0, L.ys1 + 0.2, L.ax1, L.ys1 + 0.35, 0.04, Vec3(0.92, 0.80, 0.15));   // the fire lane's yellow line
    const Vec3 white(0.92, 0.92, 0.90), curb(0.62, 0.61, 0.58), soil(0.24, 0.30, 0.16);
    for (const LotPlan::Rect& r : L.stripes) flat(paint, r.x0, r.y0, r.x1, r.y1, 0.04, white);
    for (const LotPlan::Rect& r : L.islands) {   // a curb, planted
        emitBox(kit, Scope{at(r.x0, r.y0), {u3, up, v3}, Vec3(r.x1 - r.x0, 0.15, r.y1 - r.y0)}, PartId::Concrete, curb);
        flat(paint, r.x0 + 0.15, r.y0 + 0.15, r.x1 - 0.15, r.y1 - 0.15, 0.16, soil);
    }
    for (const Vec2& t : L.trees) {
        const Vec2 w = f.toWorld(t);
        b.treeSpots.push_back(Vec3(w.x, rng.range(0.6, 0.85), w.y));
    }
    // CART CORRALS: two rails and an open end toward the aisle
    for (const LotPlan::Corral& c : L.corrals) {
        const Vec3 rail(0.55, 0.57, 0.60);
        const Vec3 ax = c.alongX ? v3 : u3, lat = c.alongX ? u3 : v3;   // the rails run along the stall's depth
        const Vec3 o = at(c.at.x, c.at.y) - ax * 2.1 - lat * 0.9;
        for (Real dv : {Real(0), Real(1.8)}) {
            emitBox(kit, Scope{o + lat * dv + up * 0.9, {ax, up, lat}, Vec3(4.2, 0.06, 0.06)}, PartId::Metal, rail);
            for (Real dx : {Real(0.0), Real(2.1), Real(4.14)})
                emitBox(kit, Scope{o + lat * dv + ax * dx, {ax, up, lat}, Vec3(0.06, 0.9, 0.06)}, PartId::Metal, rail);
        }
        emitBox(kit, Scope{o, {ax, up, lat}, Vec3(0.06, 0.96, 1.86)}, PartId::Metal, rail);
    }
    // LIGHT POLES (their lamps glow at night)
    const Vec3 pole(0.38, 0.39, 0.41);
    for (const Vec2& pp : L.poles) {
        const Real xs = pp.x, vs = pp.y;
        emitBox(kit, Scope{at(xs - 0.12, vs - 0.12), {u3, up, v3}, Vec3(0.24, 9.0, 0.24)}, PartId::Metal, pole);
        emitBox(kit, Scope{at(xs - 1.4, vs - 0.3, 9.0), {u3, up, v3}, Vec3(2.8, 0.25, 0.6)}, PartId::Metal, pole);
        RenderMesh lamp;   // the lenses, facing down
        for (Real dx : {Real(-1.25), Real(0.55)})
            MeshBuilder::emitQuad(lamp, at(xs + dx, vs - 0.25, 8.99), at(xs + dx + 0.7, vs - 0.25, 8.99),
                                  at(xs + dx + 0.7, vs + 0.25, 8.99), at(xs + dx, vs - 0.25 + 0.5, 8.99),
                                  up * -1.0, Vec3(1.0, 0.92, 0.75));
        MeshBuilder::append((*outParts)[static_cast<std::size_t>(PartId::LitBand)], lamp);
    }
    // THE PYLON SIGN by the street, near a corner: the chain's colour, lit both faces
    {
        const Real px = x1 - 6.0, pv = y0 + 1.0;
        const Vec3 brand = bp.trimColor;
        emitBox(kit, Scope{at(px - 0.6, pv - 0.4), {u3, up, v3}, Vec3(1.2, 9.0, 0.8)}, PartId::Trim, Vec3(0.30, 0.30, 0.32));
        emitBox(kit, Scope{at(px - 2.4, pv - 0.5, 9.0), {u3, up, v3}, Vec3(4.8, 3.2, 1.0)}, PartId::Trim, Vec3(0.12, 0.12, 0.13));
        // the panel in the chain's colour, a lit strip of name across each face (it glows at night)
        emitBox(kit, Scope{at(px - 2.25, pv - 0.53, 9.15), {u3, up, v3}, Vec3(4.5, 2.9, 1.06)}, PartId::Trim, brand);
        RenderMesh lit;
        for (int face = 0; face < 2; ++face) {
            const Real fv = face == 0 ? pv - 0.55 : pv + 0.55;
            const Vec3 n = face == 0 ? v3 * -1.0 : v3;
            Vec3 A = at(px - 1.9, fv, 10.1), B = at(px + 1.9, fv, 10.1), C = at(px + 1.9, fv, 11.1), D = at(px - 1.9, fv, 11.1);
            if (face == 0) MeshBuilder::emitQuad(lit, A, B, C, D, n, Vec3(1.0, 0.96, 0.88));
            else MeshBuilder::emitQuad(lit, B, A, D, C, n, Vec3(1.0, 0.96, 0.88));
        }
        MeshBuilder::append((*outParts)[static_cast<std::size_t>(PartId::LitBand)], lit);
    }
    // plain parts, not the paving's tiled surface: asphalt in Ground, the paint in Detail
    MeshBuilder::append((*outParts)[static_cast<std::size_t>(PartId::Ground)], asphalt);
    MeshBuilder::append((*outParts)[static_cast<std::size_t>(PartId::Detail)], paint);
    appendKit(kit, outParts);
}

void sculptPlaza(LotBuilding& b, const Poly2& planIn,
                 const std::function<Real(Real, Real)>& ground, uint32_t seed,
                 std::vector<RenderMesh>* outParts, const RoadGraph* roads,
                 std::vector<TerrainFlatten>* outFlatten = nullptr,
                 Real meshCell = 3.0) {
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
                if (!faces(e)) continue;   // a building fronts a street, never a freeway or a ramp
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
            // Cell-tied joints, same rule as the park spokes.
            const Real segLen =
                std::min(Real(3.0), std::max(Real(1.2), meshCell * Real(0.75)));
            const int segs = std::max(1, static_cast<int>(len / segLen));
            for (int s = 0; s < segs; ++s) {
                Vec2 q0 = mo.foot + dir * (len * s / segs);
                Vec2 q1 = mo.foot + dir * (len * (s + 1) / segs);
                const Real e0 = 0.08, e1 = 0.08;
                const Vec3 L0(q0.x - perp.x * hw, gy(q0 - perp * hw) + e0, q0.y - perp.y * hw);
                const Vec3 R0(q0.x + perp.x * hw, gy(q0 + perp * hw) + e0, q0.y + perp.y * hw);
                const Vec3 L1(q1.x - perp.x * hw, gy(q1 - perp * hw) + e1, q1.y - perp.y * hw);
                const Vec3 R1(q1.x + perp.x * hw, gy(q1 + perp * hw) + e1, q1.y + perp.y * hw);
                MeshBuilder::emitQuad(walk, L0, R0, R1, L1, up, white);
                // Same flatten stamp as the park spokes (see sculptPark): the
                // rendered terrain must agree with the ribbon at EVERY LOD.
                if (outFlatten)
                    outFlatten->push_back(makeFlattenRamp(
                        Vec3(q0.x, 0, q0.y), Vec3(q1.x, 0, q1.y),
                        gy(q0) + 0.03, gy(q1) + 0.03, hw + 0.6, 2.5));
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
                                      Vec3(0.036, 0.092, 0.136)));
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
            // A PARK BENCH from the library (M2), facing the plaza's centre, on the deck.
            (void)wood;
            b.furniture.push_back(outdoorPiece(Piece::Bench, posHash(bp), bp + dir * 0.33, Vec2(-dir.x, -dir.y), slabY, false));
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

std::vector<ParkingStall> surfaceStallsOf(const Poly2& lotP, Vec2 facing, Vec2* portal, Vec2* out) {
    std::vector<ParkingStall> st;
    if (lotP.size() < 3 || facing.length() < 1e-6) return st;
    SiteFrame f;
    f.v = normalize(facing) * -1.0;
    f.u = Vec2(f.v.y, -f.v.x);
    Real x0 = 1e30, x1 = -1e30, y0 = 1e30, y1 = -1e30;
    for (const Vec2& w : lotP) {
        const Vec2 q = f.toFrame(w);
        x0 = std::min(x0, q.x); x1 = std::max(x1, q.x);
        y0 = std::min(y0, q.y); y1 = std::max(y1, q.y);
    }
    const Real W = x1 - x0, P = y1 - y0;
    if (portal) *portal = f.toWorld({(x0 + x1) * 0.5, y0});
    if (out) *out = f.v * -1.0;
    if (W < 20 || P < 18) return st;
    for (const LotPlan::Stall& k : lotPlan(x0, x1, y0, y1).stalls) st.push_back({f.toWorld(k.at), f.u * k.face.x + f.v * k.face.y, 0});
    return st;
}


namespace {
Vertex lerpVertex(const Vertex& a, const Vertex& b, Real t) {
    Vertex v = a;
    v.position = a.position + (b.position - a.position) * t;
    v.normal = a.normal + (b.normal - a.normal) * t;
    v.tangent = a.tangent + (b.tangent - a.tangent) * t;
    v.u = a.u + (b.u - a.u) * static_cast<float>(t);
    v.v = a.v + (b.v - a.v) * static_cast<float>(t);
    v.color = a.color + (b.color - a.color) * t;
    return v;
}

}  // namespace

void drapeOnGround(RenderMesh& m, const std::function<Real(Real, Real)>& ground, Real gridStep,
                   Vec2 gridOrigin, bool followCreases) {
    if (!ground) return;
    static const bool coarseOnly = std::getenv("RT_DRAPE_COARSE") != nullptr;   // A/B: the cell-sized floor everywhere
    if (coarseOnly) followCreases = false;
    if (gridStep > Real(1e-6) && !m.indices.empty()) {
        const Real inv = Real(1) / gridStep;
        const Real ox = gridOrigin.x, oz = gridOrigin.y;
        // ONLY AS FINE AS THE GROUND NEEDS (ADR-0095). Cutting every draped mesh along every
        // grid line shredded each lawn and path into cell-sized pieces -- at the baked 1 m cell,
        // +950 MB of dressing on metro_planned. Instead a triangle is kept whole when the drawn
        // ground under it (checked at every grid node of the cells it touches) lies within
        // kDrapeTol of the plane through its corners, and otherwise split at the middle of its
        // longest edge and checked again, down to the grid cell. Flat pads stay one piece, a
        // gentle hill a few metre-scale pieces, a kerb crease its cell. Where neighbours split
        // differently the seam is off the ground by at most kDrapeTol.
        constexpr Real kDrapeTol = 0.03;
        // The drawn ground at GRID NODES, remembered: neighbouring triangles (and every level of a split) test the
        // same nodes, and the ground (lodSurfaceHeight, its flatten set) is the whole cost of a drape.
        std::unordered_map<long long, Real> nodeGround;
        auto nodeAt = [&](long i, long j) {
            const long long key = (static_cast<long long>(i) << 32) ^ static_cast<long long>(static_cast<uint32_t>(j));
            auto it = nodeGround.find(key);
            if (it != nodeGround.end()) return it->second;
            const Real g = ground(ox + i * gridStep, oz + j * gridStep);
            nodeGround.emplace(key, g);
            return g;
        };
        auto withinTol = [&](const Vertex& A, const Vertex& B, const Vertex& C) {
            const Real ga = ground(A.position.x, A.position.z), gb = ground(B.position.x, B.position.z),
                       gc = ground(C.position.x, C.position.z);
            const Real x1 = B.position.x - A.position.x, z1 = B.position.z - A.position.z;
            const Real x2 = C.position.x - A.position.x, z2 = C.position.z - A.position.z;
            const Real det = x1 * z2 - x2 * z1;
            if (std::fabs(det) < Real(1e-9)) return true;   // degenerate: nothing to follow
            const Real bx = ((gb - ga) * z2 - (gc - ga) * z1) / det, bz = ((gc - ga) * x1 - (gb - ga) * x2) / det;
            const Real mnx = std::min({A.position.x, B.position.x, C.position.x}), mxx = std::max({A.position.x, B.position.x, C.position.x});
            const Real mnz = std::min({A.position.z, B.position.z, C.position.z}), mxz = std::max({A.position.z, B.position.z, C.position.z});
            const long i0 = static_cast<long>(std::floor((mnx - ox) * inv)), i1 = static_cast<long>(std::ceil((mxx - ox) * inv));
            const long j0 = static_cast<long>(std::floor((mnz - oz) * inv)), j1 = static_cast<long>(std::ceil((mxz - oz) * inv));
            for (long j = j0; j <= j1; ++j)
                for (long i = i0; i <= i1; ++i) {
                    const Real x = ox + i * gridStep, z = oz + j * gridStep;
                    if (std::fabs(nodeAt(i, j) - (ga + bx * (x - A.position.x) + bz * (z - A.position.z))) > kDrapeTol) return false;
                }
            return true;
        };
        RenderMesh out;
        out.materialIndex = m.materialIndex;
        std::vector<std::array<Vertex, 3>> work;
        for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3)
            work.push_back({m.vertices[m.indices[t]], m.vertices[m.indices[t + 1]], m.vertices[m.indices[t + 2]]});
        while (!work.empty()) {
            std::array<Vertex, 3> tri = work.back();
            work.pop_back();
            auto len2 = [&](int u, int w) {
                const Real dx = tri[u].position.x - tri[w].position.x, dz = tri[u].position.z - tri[w].position.z;
                return dx * dx + dz * dz;
            };
            int e = 0;   // longest edge: (e, e+1)
            for (int k = 1; k < 3; ++k) if (len2(k, (k + 1) % 3) > len2(e, (e + 1) % 3)) e = k;
            // A triangle SMALLER than a cell is checked too (lot_dressing_is_planted_on_the_ground, 2026-10-03): a park
            // plaza's 2 m fan triangles were kept whole under the old "down to the grid cell" floor and bridged an
            // earthwork lip the ground climbs 1.3 m over 1 m -- the plaza 0.6 m off the ground mid-triangle. The
            // floor is a sixteenth of a cell, so a crease through a small triangle is followed to ~0.1 m.
            // (a paved lot's PLATE -- white: its texture carries the look -- is a slab standing 0.35 m proud by
            // design and covers whole lots; following every crease under it was most of the cost, for nothing)
            auto white = [](const Vertex& v) { return v.color.x > Real(0.99) && v.color.y > Real(0.99) && v.color.z > Real(0.99); };
            const bool walking = followCreases && tri[0].normal.y > Real(0.5) && tri[1].normal.y > Real(0.5) &&
                                 tri[2].normal.y > Real(0.5) && !(white(tri[0]) && white(tri[1]) && white(tri[2]));
            // A WALKING SURFACE IS CUT, NOT BISECTED. Inside one cell the drawn ground is two planar triangles split on
            // the (0,0)-(1,1) diagonal (lodSurfaceHeight), so a walk triangle cut along the cell edges and diagonals
            // it crosses lies on the ground exactly, in a handful of pieces. (Bisecting toward a crease took a walk
            // down to 1/16-cell slivers along every line it crossed: +50 s on the island load. The plane-vs-grid-
            // nodes test below is no use under a cell: on any slope the cell corners are off a small triangle's plane.)
            if (walking) {
                std::vector<std::vector<Vertex>> polys{{tri[0], tri[1], tri[2]}};
                auto gxOf = [&](const Vertex& v) { return (v.position.x - ox) * inv; };
                auto gzOf = [&](const Vertex& v) { return (v.position.z - oz) * inv; };
                // split every polygon by the line f(v) = 0
                auto cut = [&](const std::function<Real(const Vertex&)>& f) {
                    std::vector<std::vector<Vertex>> next;
                    for (const std::vector<Vertex>& poly : polys) {
                        std::vector<Vertex> lo, hi;
                        for (std::size_t k = 0; k < poly.size(); ++k) {
                            const Vertex& p0 = poly[k];
                            const Vertex& p1 = poly[(k + 1) % poly.size()];
                            const Real f0 = f(p0), f1 = f(p1);
                            if (f0 <= 0) lo.push_back(p0);
                            if (f0 >= 0) hi.push_back(p0);
                            if ((f0 < 0 && f1 > 0) || (f0 > 0 && f1 < 0)) {
                                const Vertex m2 = lerpVertex(p0, p1, f0 / (f0 - f1));
                                lo.push_back(m2);
                                hi.push_back(m2);
                            }
                        }
                        if (lo.size() >= 3) next.push_back(std::move(lo));
                        if (hi.size() >= 3) next.push_back(std::move(hi));
                    }
                    polys = std::move(next);
                };
                Real mnx = 1e30, mxx = -1e30, mnz = 1e30, mxz = -1e30, mnd = 1e30, mxd = -1e30;
                for (const Vertex& v : tri) {
                    const Real gx = gxOf(v), gz = gzOf(v);
                    mnx = std::min(mnx, gx); mxx = std::max(mxx, gx); mnz = std::min(mnz, gz); mxz = std::max(mxz, gz);
                    mnd = std::min(mnd, gx - gz); mxd = std::max(mxd, gx - gz);
                }
                for (long k = static_cast<long>(std::floor(mnx)) + 1; k < mxx; ++k)
                    cut([&, k](const Vertex& v) { return gxOf(v) - Real(k); });
                for (long k = static_cast<long>(std::floor(mnz)) + 1; k < mxz; ++k)
                    cut([&, k](const Vertex& v) { return gzOf(v) - Real(k); });
                for (long k = static_cast<long>(std::floor(mnd)) + 1; k < mxd; ++k)
                    cut([&, k](const Vertex& v) { return gxOf(v) - gzOf(v) - Real(k); });
                for (const std::vector<Vertex>& poly : polys) {
                    const uint32_t base = static_cast<uint32_t>(out.vertices.size());
                    for (const Vertex& v : poly) out.vertices.push_back(v);
                    for (uint32_t k = 1; k + 1 < poly.size(); ++k) out.indices.insert(out.indices.end(), {base, base + k, base + k + 1});
                }
                continue;
            }
            const Real l2 = len2(e, (e + 1) % 3);
            const bool keep = l2 <= gridStep * gridStep || withinTol(tri[0], tri[1], tri[2]);
            if (keep) {
                const uint32_t base = static_cast<uint32_t>(out.vertices.size());
                for (const Vertex& v : tri) out.vertices.push_back(v);
                out.indices.insert(out.indices.end(), {base, base + 1, base + 2});
                continue;
            }
            const int e1 = (e + 1) % 3, o = (e + 2) % 3;
            const Vertex mid = lerpVertex(tri[e], tri[e1], Real(0.5));
            work.push_back({tri[e], mid, tri[o]});
            work.push_back({mid, tri[e1], tri[o]});
        }
        m = std::move(out);
    }
    for (Vertex& v : m.vertices)
        v.position.y += ground(v.position.x, v.position.z);
}

TerrainFlatten lotPadFlatten(const LotBuilding& lb, double apron, double falloff) {
    // A paved lot (ADR-0086) is flat to its lot line: the plate covers the
    // whole lot, so the pad does too, with only a hair of apron past it.
    // THE PAD IS THE PARCEL (skyscrapers v2). It used to be the plan plus a 2.2 m
    // apron, which reached the lot line only because the plan stood a metre
    // inside it. With real yards the apron stops short, and the strip between
    // the road and the building shows the block plane instead — which on a
    // hillside stands above the road deck (the poke gate went 108 -> 275 when
    // the yards landed, and clipping the pad to the parcel moved it by 20).
    // So a lot with a recorded bound flattens its WHOLE parcel at groundY —
    // its yards and paving included, the cross-street strips excluded (see
    // LotBuilding::padBound) — and only a legacy record keeps the apron.
    const bool paved = !lb.pavedLot.empty();
    const bool bounded = lb.padBound.size() >= 3;
    const Poly2& src = bounded ? lb.padBound : (paved ? lb.pavedLot : lb.plan);
    if (bounded) apron = 0.0;
    else if (paved) apron = std::min(apron, 0.3);
    Vec2 c2(0, 0); for (const Vec2& v : src) c2 = c2 + v; if (!src.empty()) c2 = c2 * (1.0 / static_cast<double>(src.size()));
    Poly2 grown; grown.reserve(src.size());
    for (const Vec2& v : src) { const Vec2 d = v - c2; const double l = d.length(); grown.push_back(apron > 0 && l > 1e-6 ? c2 + d * ((l + apron) / l) : v); }
    // A PAD NEVER CROSSES ITS PARCEL LINE. The lot line is only ~1.3 m behind
    // the sidewalk's outer edge; a pad that spills past it (the 2.2 m apron
    // and the 5 m feather did, on every rectified corner lot hugging a cross
    // street) sits at the FRONT street's level and lifts the terrain through a
    // lower cross street's deck — metro_road_decks_are_never_poked_by_the_
    // drawn_terrain went 119 -> 276 pokes when the rectangles landed. Clip the
    // pad to the parcel (+0.3 m) and end the feather inside the strip.
    // The pad is clipped to LotBuilding::padBound where the pass recorded one (the
    // parcel, 1 m inside every street-facing edge but the frontage — see the
    // field), else to the parcel itself; the bound is convex-ish, so a half-plane
    // clip per edge is exact enough and never needs a boolean library.
    const Poly2& boundSrc = lb.padBound.size() >= 3 ? lb.padBound : lb.lot;
    if (boundSrc.size() >= 3) {
        Poly2 bound = boundSrc;
        ensureCCW(bound);
        const Real slack = lb.padBound.size() >= 3 ? 0.0 : 0.3;
        Poly2 clipped = grown;
        // A pad that IS its bound (the parcel, apron 0) is already inside it — and the per-edge
        // half-plane clip below is only right for a CONVEX bound: a whole-block site's parcel is a
        // concave polygon of 40-80 edges, and clipping it by each of its own edges cut the pad to a
        // sliver, so the landmark stood on the bare block grade (floorplan census: a 3768 m2 civic
        // hall 3.2 m under its uphill ground, no pad at the point at all).
        // The same holds for any CONCAVE bound: there the pad keeps to its own source polygon (no
        // apron — it cannot spill past a lot line it never crosses) and skips the clip.
        bool concave = false;
        {
            const std::size_t nb = bound.size();
            for (std::size_t i = 0; i < nb && !concave; ++i) {
                const Vec2 e0 = bound[(i + 1) % nb] - bound[i];
                const Vec2 e1 = bound[(i + 2) % nb] - bound[(i + 1) % nb];
                if (cross(e0, e1) < -1e-6 * e0.length() * e1.length()) concave = true;   // a right turn on a CCW ring
            }
        }
        if (concave && !bounded) grown = src;
        if (bounded || concave) clipped.clear();
        for (std::size_t i = 0; i < bound.size() && clipped.size() >= 3; ++i) {
            const Vec2 a = bound[i], b = bound[(i + 1) % bound.size()];
            const Vec2 d = b - a;
            const Real len = d.length();
            if (len < 1e-9) continue;
            const Vec2 n(d.y / len, -d.x / len);   // CCW: right normal = outward
            clipped = clipHalfPlane(clipped, n, dot(n, a) + slack);
        }
        if (clipped.size() >= 3) grown = std::move(clipped);   // never clip a pad to nothing
        falloff = std::min(falloff, 1.0);
    }
    std::vector<Vec3> poly; poly.reserve(grown.size());
    for (const Vec2& v : grown) poly.push_back(Vec3(v.x, 0, v.y));
    return makeFlattenPad(std::move(poly), lb.groundY, falloff);
}

std::vector<LotBuilding> growLotBuildings(const std::vector<Poly2>& blocks,
                                          const LotParams& pIn, LotPlanDebug* debug,
                                          std::vector<RenderMesh>* outParts,
                                          const RoadGraph* roads, Real roadClearance,
                                          std::vector<RenderMesh>* outFlatParts,
                                          std::vector<TerrainFlatten>* outGrade) {
    std::vector<Poly2> paseoGrades;   // the open-air malls' footprints, graded flat after PASS A
    // Mutable copy: after PASS A the ground sampler is wrapped with the block
    // grades (see the grading step below), so PASS B/C grow on terraced ground.
    LotParams p = pIn;
    std::vector<LotBuilding> out;
    // CORENESS (skyscrapers v2 M3 — the owner: "go as big as we can"): 1 on a
    // PLATEAU over the inner half of the downtown radius, easing linearly to 0
    // at the midtown radius — a cluster with shoulders, not a spike at the one
    // lot nearest the centre. The financial grain, the commercial grain near
    // downtown and every recipe's height lift read this one function; before,
    // it was sqrt(1 - d/innerRadius) in two places and 0 for every lot outside
    // the financial rim, which is why a "downtown" was seven buildings.
    // From the NEAREST downtown: every financial hub zones a downtown of its own (DistrictMap), and
    // measuring height from `center` alone left a second one — metro_planned's mountain city — a
    // financial district of low-rise.
    // EACH TOWN'S NIGHTLIFE STRIP: a seeded point in its commercial ring (out past the skyscraper plateau, short of
    // the residential edge); the lots within reach of it are bars, restaurants and clubs in low buildings.
    std::vector<Vec2> nightCentres;
    {
        std::vector<Vec2> towns{p.center};
        for (const auto& h : p.hubs) if (h.second == 0) towns.push_back(h.first);
        for (std::size_t t = 0; t < towns.size(); ++t) {
            const uint32_t hn = mix(p.seed, 0x9e37u + static_cast<uint32_t>(t) * 977u);
            const Real ang = (hn & 0xffffu) / 65536.0 * 6.283185307179586;
            const Real r = 0.5 * (std::max(Real(1), p.innerRadius) + std::max(p.midRadius, p.innerRadius + 1));
            nightCentres.push_back(towns[t] + Vec2(std::cos(ang), std::sin(ang)) * r);
        }
    }
    const Real nightReach = std::max(Real(90), 0.6 * std::max(Real(1), p.innerRadius));
    auto nightlifeAt = [&](const Vec2& q) {
        for (const Vec2& c : nightCentres) if ((q - c).length() < nightReach) return true;
        return false;
    };
    auto corenessAt = [&](const Vec2& q) {
        Real d = (q - p.center).length();
        for (const auto& h : p.hubs) if (h.second == 0) d = std::min(d, (q - h.first).length());
        const Real plateau = 0.5 * std::max(Real(1), p.innerRadius);
        const Real edge = std::max(p.midRadius, p.innerRadius + Real(1));
        if (d <= plateau) return Real(1);
        if (d >= edge) return Real(0);
        return (edge - d) / (edge - plateau);
    };
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

    // TREES IN THE ROAD (Glenn, 2026-09-17: "one lot being built in the middle
    // of a street which is placing trees in the road"). The BUILDING is shrunk
    // to clear the carriageway -- measured, 0 of 1397 plans stand in a lane --
    // but landscaping is planted across the LOT, which was never inset. So a
    // parcel that runs into the street grows its trees there while its facade
    // politely stops at the kerb. Prune any spot inside a carriageway; a lot
    // keeps whatever spots are genuinely on its own ground.
    int treeSpotsPruned = 0;
    // MEASURED, not assumed: pads that come back from pushPolyClearOfRoads
    // still covering carriageway. The relaxation pushes to width/2 +
    // roadClearance (4.6 m) but VERIFIES at width/2 + 0.3, and when up to
    // 20%% of vertices still fail it DELETES them and returns the rest --
    // and a polygon missing the vertex that poked into a street can still
    // span it, because the edge across the gap is never re-tested.
    int padsStillInRoad = 0, padsChecked = 0;
    int padsOnCarriageway = 0;   // the STRICT question: on the asphalt itself
    int lotsInCarriageway = 0;   // PARCELS that reach into a lane
    std::vector<Vec2> rejectedPadAt;   // WHERE the road-locked pads were
    auto pruneTreeSpotsIntoRoad = [&](std::vector<Vec3>& spots) {
        if (!roads || spots.empty()) return;
        const std::size_t before = spots.size();
        spots.erase(std::remove_if(spots.begin(), spots.end(),
                                   [&](const Vec3& sp) {
                                       return !clearOfRoads(Vec2(sp.x, sp.z));
                                   }),
                    spots.end());
        treeSpotsPruned += static_cast<int>(before - spots.size());
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
    // Does a finished pad still cover roadway? Samples the RING and the
    // centroid against the same predicate the buildings use, so this asks the
    // question at the clearance the pass actually intends.
    // ON THE ASPHALT. clearOfRoads() carries the 4.6 m building clearance, so a
    // park touching the pavement trips it without being in the road at all.
    // This asks the question Glenn actually asked: is the point on the
    // carriageway surface?
    auto onCarriageway = [&](const Vec2& c) {
        if (!roads) return false;
        const RoadGraph* gs[2] = {roads, &alleyGraph};
        for (const RoadGraph* gp : gs) {
            const RoadGraph& g = *gp;
            for (const RoadEdge& e : g.edges) {
                if (e.a < 0 || e.b < 0 || e.a >= static_cast<int>(g.nodes.size()) ||
                    e.b >= static_cast<int>(g.nodes.size())) continue;
                const Vec2& a2 = g.nodes[e.a].pos;
                const Vec2& b2 = g.nodes[e.b].pos;
                Vec2 ab = b2 - a2;
                Real len2 = ab.lengthSquared();
                Real t = len2 > 1e-12 ? dot(c - a2, ab) / len2 : 0.0;
                t = t < 0 ? 0 : (t > 1 ? 1 : t);
                const Vec2 q(a2.x + ab.x * t, a2.y + ab.y * t);
                if ((c - q).length() < e.width * 0.5) return true;
            }
        }
        return false;
    };
    auto padCoversRoad = [&](const Poly2& pad) {
        if (pad.size() < 3) return false;
        ++padsChecked;
        for (const Vec2& v : pad)
            if (!clearOfRoads(v)) { ++padsStillInRoad; return true; }
        if (!clearOfRoads(centroid(pad))) { ++padsStillInRoad; return true; }
        // Edge midpoints: the case a deleted vertex leaves behind.
        bool near = false;
        for (std::size_t i = 0; i < pad.size(); ++i) {
            const Vec2 m = (pad[i] + pad[(i + 1) % pad.size()]) * 0.5;
            if (!clearOfRoads(m)) { near = true; break; }
        }
        if (near) ++padsStillInRoad;
        return near;
    };
    // Same sampling, strict predicate: ring, centroid and edge midpoints (the
    // midpoint being exactly what a DELETED vertex leaves spanning a street).
    // A pad that LIES ACROSS a street is not a pad. Vertex relaxation cannot
    // catch this case: every corner of a big green can sit clear of the kerb
    // while the carriageway runs straight through its middle, so the ring test
    // passes and the polygon still furnishes the roadway (Glenn: "one lot being
    // built in the middle of a street which is placing trees in the road").
    // Measured: 4 of 301 pads, all of them bad == 0 -- they never went near the
    // trim path. Sampled on the ring, the centroid AND along every edge, since
    // a road crossing a polygon must cross its boundary somewhere.
    auto padClearOrEmpty = [&](const Poly2& pad) -> Poly2 {
        if (pad.size() < 3) return pad;
        auto hit = [&](const Vec2& c) {
            if (!roads) return false;
            const RoadGraph* gs2[2] = {roads, &alleyGraph};
            for (const RoadGraph* gp : gs2)
                for (const RoadEdge& e : gp->edges) {
                    const RoadGraph& g = *gp;
                    if (e.a < 0 || e.b < 0 || e.a >= static_cast<int>(g.nodes.size()) ||
                        e.b >= static_cast<int>(g.nodes.size())) continue;
                    const Vec2& a3 = g.nodes[e.a].pos;
                    const Vec2& b3 = g.nodes[e.b].pos;
                    Vec2 ab3 = b3 - a3;
                    Real l2 = ab3.lengthSquared();
                    Real t3 = l2 > 1e-12 ? dot(c - a3, ab3) / l2 : 0.0;
                    t3 = t3 < 0 ? 0 : (t3 > 1 ? 1 : t3);
                    const Vec2 q3(a3.x + ab3.x * t3, a3.y + ab3.y * t3);
                    if ((c - q3).length() < e.width * 0.5) return true;
                }
            return false;
        };
        const Vec2 c0 = centroid(pad);
        if (hit(c0)) { rejectedPadAt.push_back(c0); return Poly2{}; }
        for (std::size_t i2 = 0; i2 < pad.size(); ++i2) {
            const Vec2& A = pad[i2];
            const Vec2& B = pad[(i2 + 1) % pad.size()];
            const Real len = (B - A).length();
            const int steps = std::max(1, static_cast<int>(len / 2.0));
            for (int k2 = 0; k2 <= steps; ++k2)
                if (hit(A + (B - A) * (static_cast<Real>(k2) / steps))) {
                    rejectedPadAt.push_back(c0);
                    return Poly2{};
                }
        }
        return pad;
    };
    auto padOnCarriageway = [&](const Poly2& pad) {
        if (pad.size() < 3) return false;
        for (const Vec2& v : pad) if (onCarriageway(v)) { ++padsOnCarriageway; return true; }
        if (onCarriageway(centroid(pad))) { ++padsOnCarriageway; return true; }
        for (std::size_t i = 0; i < pad.size(); ++i)
            if (onCarriageway((pad[i] + pad[(i + 1) % pad.size()]) * 0.5)) {
                ++padsOnCarriageway; return true;
            }
        return false;
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
    int sitesRectified = 0, sitesNoRect = 0, sitesRectUnfit = 0, pavedLots = 0;   // site-plan ledger (M1)
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
        // The FRONT EDGE, not one point (floorplan-conformance round): the
        // old single sample 2 m past the entrance sat on the centroid ray
        // only — on a street sloping ALONG the frontage the front corners
        // deviated by slope x halfFrontage (up to metres), and whichever
        // corner drew the short straw hovered. The pad anchors to the MIN
        // across the whole front edge: no front corner may hover; the uphill
        // corner cuts in (a retaining edge — standard hillside practice),
        // and the entrance rise stays small by construction, which is what
        // keeps the entry steps short and the tall foundation faces on the
        // sides/back.
        if (face.length() > Real(1e-6)) {
            const Vec2 f = normalize(face);
            const std::size_t n = pl.size();
            // The plan edge most aligned with the entrance direction (its
            // outward right-normal vs `face`), i.e. the frontage edge.
            Real bestDot = Real(0.2);   // demand rough agreement
            std::size_t bestI = n;
            for (std::size_t i = 0; i < n; ++i) {
                const Vec2 e = pl[(i + 1) % n] - pl[i];
                const Real l = e.length();
                if (l < Real(1e-6)) continue;
                const Vec2 rn(e.y / l, -e.x / l);   // outward for CCW plans
                const Real d = dot(rn, f);
                if (d > bestDot) { bestDot = d; bestI = i; }
            }
            if (bestI < n) {
                const Vec2& a = pl[bestI];
                const Vec2& b = pl[(bestI + 1) % n];
                Real lo = Real(1e30);
                for (Real t : {Real(0.0), Real(0.5), Real(1.0)}) {
                    const Vec2 q = a + (b - a) * t + f * Real(2.0);
                    // THE STREET IT FACES (Glenn: buildings "sunk under their roads ... sitting high above
                    // their roads with no access"): the deck in front, where a lane city knows it -- the
                    // lawn 2 m out can be metres off the street on a hillside.
                    Real sy;
                    if (p.streetHeight && p.streetHeight(q.x, q.y, &sy)) lo = std::min(lo, sy);
                    else lo = std::min(lo, p.ground(q.x, q.y));
                }
                return lo;
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
    // The FOUNDATION BLOCK (Glenn's design — the daylight/stem-wall
    // foundation of real hillside construction): the plan outset slightly,
    // extruded as a solid concrete body from the wall base DOWN past the
    // terrain under every perimeter vertex. The grammar stays terrain-free;
    // this lot layer owns both plan and ground, so it pours the concrete the
    // building stands on. Emitted into EVERY tier that draws the building —
    // LOD0 and, when grown, the LOD1 flat twin — because a foundation that
    // exists only up close is a building that floats at a distance (the
    // census's mechanism 4).
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
        const Vec3 col(1, 1, 1);   // Concrete's surface maps carry the look
        // The block's bottom DRAPES + BEDS: each ring vertex drops past the
        // terrain sampled under it (0.5 m bed-in) — and also past a mid-edge
        // sample, so a dip BETWEEN two vertices can't open daylight under a
        // long wall (the census walks the perimeter at 1 m; the block must
        // beat it everywhere, and sampling only corners lost mid-edge sag).
        std::vector<Real> bot(nv);
        for (std::size_t i = 0; i < nv; ++i) {
            const std::size_t j = (i + 1) % nv;
            const Vec2 mid = (o[i] + o[j]) * Real(0.5);
            Real g = std::min(p.ground(o[i].x, o[i].y),
                              p.ground(mid.x, mid.y));
            g = std::min(g, p.ground(o[j].x, o[j].y));
            bot[i] = std::min(planeY, g) - Real(0.5);
        }
        // Every edge's bottom uses the LOWER of its two ends, so adjacent
        // side quads always share their bottom corner (watertight sides).
        auto emitInto = [&](std::vector<RenderMesh>* parts) {
            if (!parts) return;
            RenderMesh& m = (*parts)[static_cast<std::size_t>(PartId::Concrete)];
            for (std::size_t i = 0; i < nv; ++i) {
                const std::size_t j = (i + 1) % nv;
                Vec2 e = o[j] - o[i];
                if (e.length() < Real(1e-9)) continue;
                Vec2 n = normalize(Vec2(e.y, -e.x));
                const Real eb = std::min(bot[i], bot[j]);
                MeshBuilder::emitQuad(m, Vec3(o[i].x, eb, o[i].y),
                                      Vec3(o[j].x, eb, o[j].y),
                                      Vec3(o[j].x, topY, o[j].y),
                                      Vec3(o[i].x, topY, o[i].y),
                                      Vec3(n.x, 0, n.y), col);
                // The exposed ledge between the block's outer lip and the wall.
                MeshBuilder::emitQuad(m, Vec3(o[i].x, topY, o[i].y),
                                      Vec3(o[j].x, topY, o[j].y),
                                      Vec3(pl[j].x, topY, pl[j].y),
                                      Vec3(pl[i].x, topY, pl[i].y),
                                      Vec3(0, 1, 0), col);
            }
        };
        emitInto(outParts);
        emitInto(outFlatParts);   // the LOD1 twin stands on the same concrete
    };
    if (outParts) {
        outParts->assign(kLotPartSlots, RenderMesh{});   // base + draped slots
        for (std::size_t i = 0; i < outParts->size(); ++i)
            (*outParts)[i].materialIndex = static_cast<int>(baseSlot(i));
    }
    if (outFlatParts) {   // same slot layout as outParts (the LOD1 tier draws no dressing)
        outFlatParts->assign(kLotPartSlots, RenderMesh{});
        for (std::size_t i = 0; i < outFlatParts->size(); ++i)
            (*outFlatParts)[i].materialIndex = static_cast<int>(baseSlot(i));
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
    // Doors + regen key for one grown unit (ADR-0080): the "entrance"
    // attaches mergeParts is about to discard become DoorSpecs on a
    // BuildingUnit the runtime can rebuild interiors from.
    auto collectUnit = [](LotBuilding& b, const Poly2& uplan,
                          const BuildingParams& upar, Real ubase,
                          const BuildingMesh& um) {
        BuildingUnit u;
        u.plan = uplan;
        u.baseY = ubase;
        u.params = upar;
        for (const AttachPoint& ap : um.attaches) {
            if (ap.tag == "entrance")
                u.doors.push_back({Vec2(ap.position.x, ap.position.z),
                                   Vec2(ap.normal.x, ap.normal.z), ap.width,
                                   ap.height});
            else if (ap.tag == "beacon")
                u.beacons.push_back(ap.position);
        }
        // The SHOPS' street doors and the BACK door after the building's own entrance: doors.front() stays the
        // front door (the citysim's place entrance); every door gets its collider gap and its leaf.
        for (const AttachPoint& ap : um.attaches)
            if (ap.tag == "shopdoor" || ap.tag == "backdoor")
                u.doors.push_back({Vec2(ap.position.x, ap.position.z), Vec2(ap.normal.x, ap.normal.z), ap.width,
                                   ap.height, ap.tag == "backdoor"});
        // Enterable requires an ACTUAL collected door: recipes that never
        // take FacadeMode::Entrance (cylinders, pagodas, bay fronts) can
        // carry openDoorway without ever emitting an aperture. Evaluated
        // AFTER the doors loop -- the first cut sat before it, doors was
        // always empty, and the citywide enable produced 0 enterable
        // buildings (the level door gate caught it on the first run).
        u.enterable = upar.openDoorway && !u.doors.empty();
        b.units.push_back(std::move(u));
    };
    // Does an enterableAt point land in -- or stand beside -- this unit's
    // plan? Proximity matters because a SAFE spawn is authored OUTSIDE every
    // prism (the Phase-0 guard walks it out), a couple of metres off the
    // building it should open. 8 m reaches across a sidewalk, never across
    // a street (>= 12 m).
    auto wantsDoorway = [&p](const Poly2& uplan, const BuildingParams& bp) {
        if (uplan.size() < 3) return false;
        // CITYWIDE (device: "could we apply this to buildings of this size
        // or less?"): every walk-in building up to 3 upper storeys grows
        // open. Curtain-wall shafts keep painted doors (their look is
        // unresolved), as do units with no walkable ground. Taller
        // buildings wait for elevators (roadmap).
        if (bp.walkableGround && bp.groundBays <= 0) {
            if (!bp.curtainWall && bp.floors <= 3) return true;
            // THE UNIVERSITY's halls climb by their own stair (campusPlan), however many storeys: the residence
            // hall's four or five were shut, so its students had nowhere to be seen (campus M4)
            if (bp.campus) return true;
            // TALL buildings open when a CORE fits (skyscrapers v2 M5): the
            // stairwells and the elevator bank make every floor reachable,
            // and a curtain wall is as enterable as any other (its entrance
            // bay is a real aperture; the leaf is the DoorSystem's).
            if (wantsCore(bp) && coreFor(uplan, bp, entranceEdgeFor(uplan, bp)).valid) return true;
            // ITS SHOPS, whatever stands above them: a ground-floor shop is a room of its own behind its own door, no
            // stair needed (storefronts plan, stage 5; Glenn: "What about the interior does it look like a shop would
            // look?" -- it had never been seen: a tower over shops with no core never streamed its interior, so the
            // shops' fit-outs were never drawn)
            if (bp.groundRetail && !shopFrontsOf(uplan, bp).empty()) return true;
        }
        for (const Vec2& pt : p.enterableAt) {
            if (pointInPolygon(uplan, pt)) return true;
            for (std::size_t i = 0; i < uplan.size(); ++i) {
                const Vec2 a = uplan[i], b = uplan[(i + 1) % uplan.size()];
                const Vec2 ab = b - a;
                const Real L2 = ab.lengthSquared();
                const Real t = L2 < Real(1e-12)
                                   ? Real(0)
                                   : std::max(Real(0),
                                              std::min(Real(1),
                                                       dot(pt - a, ab) / L2));
                if ((a + ab * t - pt).length() < 8.0) return true;
            }
        }
        return false;
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
        Yards yards;  // front / side / rear setbacks the site plan honours
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
    std::vector<Vec2> bigBoxCentres;   // the big-box blocks chosen so far (they keep 700 m apart)
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
    // Parks/greens are SCULPTED LATE (after the grade rebind below) so their
    // paths and furniture sample the TERRACED ground the terrain will show —
    // sculpting during parcelling sampled the pre-grade hillside, and the
    // walking paths floated over the block-graded terraces (device:
    // "walkways float ... as part of plazas and parks"). The entries only
    // reserve the lot and record the seed; the sculpt runs post-rebind.
    struct DeferredPark { std::size_t lot; uint32_t seed; };
    std::vector<DeferredPark> deferredParks;
    struct DeferredAlley { Vec2 a, b; };
    std::vector<DeferredAlley> deferredAlleys;

    // WHICH BLOCK EDGES ARE STREETS: a block edge whose nearest road is a freeway or a ramp gets no lots
    // (ParcelParams::isFrontage). The road segments are binned once on a 32 m grid; an edge is asked at its
    // quarter points and is a street when most of them lie nearest a street (a merged edge can run past a
    // ramp's end). No road within 60 m of the kerb: a street. A graph with no freeway: every edge a street.
    std::function<bool(const Vec2&, const Vec2&)> isFrontage;
    // ...and a lot's clearance from them: no lot within kFreewayClear of a freeway or ramp carriageway, even
    // from behind (island_8_nature: a shop's back wall 0.3 m from an on-ramp, 2026-09-30).
    std::function<bool(const Vec2&)> nearFreeway;
    if (p.nearFreeway) { auto f = p.nearFreeway; nearFreeway = [f](const Vec2& q) { return f(q.x, q.y); }; }
    // 6 m from the lot graph's carriageway edge: its ramp widths run ~2 m narrower than the drawn deck, and at 4 m
    // an office still stood 1.8 m from a ramp's asphalt
    constexpr Real kFreewayClear = 6.0;
    if (roads && !roads->edges.empty()) {
        constexpr Real kCell = 32;
        auto segGrid = std::make_shared<std::unordered_map<long long, std::vector<int>>>();
        auto key = [](int cx, int cz) { return (static_cast<long long>(cx) << 32) ^ static_cast<uint32_t>(cz); };
        bool anyFreeway = false;
        for (std::size_t ei = 0; ei < roads->edges.size(); ++ei) {
            const RoadEdge& e = roads->edges[ei];
            if (e.a < 0 || e.b < 0 || e.a >= static_cast<int>(roads->nodes.size()) || e.b >= static_cast<int>(roads->nodes.size())) continue;
            anyFreeway = anyFreeway || !faces(e);
            const Vec2& ra = roads->nodes[e.a].pos; const Vec2& rb = roads->nodes[e.b].pos;
            const int x0 = static_cast<int>(std::floor(std::min(ra.x, rb.x) / kCell)), x1 = static_cast<int>(std::floor(std::max(ra.x, rb.x) / kCell));
            const int z0 = static_cast<int>(std::floor(std::min(ra.y, rb.y) / kCell)), z1 = static_cast<int>(std::floor(std::max(ra.y, rb.y) / kCell));
            if ((x1 - x0 + 1) * (z1 - z0 + 1) > 4096) continue;   // a degenerate span; never on a real net
            for (int cx = x0; cx <= x1; ++cx) for (int cz = z0; cz <= z1; ++cz) (*segGrid)[key(cx, cz)].push_back(static_cast<int>(ei));
        }
        if (anyFreeway) {
            if (!nearFreeway) {
                const RoadGraph* g = roads;
                nearFreeway = [g, segGrid, key](const Vec2& q) {
                    const int qx = static_cast<int>(std::floor(q.x / 32)), qz = static_cast<int>(std::floor(q.y / 32));
                    for (int cx = qx - 1; cx <= qx + 1; ++cx) for (int cz = qz - 1; cz <= qz + 1; ++cz) {
                        auto it = segGrid->find(key(cx, cz)); if (it == segGrid->end()) continue;
                        for (int ei : it->second) {
                            const RoadEdge& e = g->edges[static_cast<std::size_t>(ei)];
                            if (faces(e) || e.layer != 0) continue;   // at-grade freeway and ramp carriageways
                            const Vec2& ra = g->nodes[e.a].pos; const Vec2& rb = g->nodes[e.b].pos;
                            const Vec2 ab = rb - ra; const Real l2 = ab.lengthSquared();
                            Real u = l2 > 1e-12 ? dot(q - ra, ab) / l2 : Real(0); u = std::max(Real(0), std::min(Real(1), u));
                            if ((q - (ra + ab * u)).length() - e.width * Real(0.5) < kFreewayClear) return true;
                        }
                    }
                    return false;
                };
            }
            const RoadGraph* g = roads;
            isFrontage = [g, segGrid, key](const Vec2& a, const Vec2& b) {
                int street = 0, freeway = 0;
                for (Real t : {Real(0.25), Real(0.5), Real(0.75)}) {
                    const Vec2 q = a + (b - a) * t;
                    Real best = Real(60); int bestE = -1;
                    const int qx = static_cast<int>(std::floor(q.x / 32)), qz = static_cast<int>(std::floor(q.y / 32));
                    for (int cx = qx - 2; cx <= qx + 2; ++cx) for (int cz = qz - 2; cz <= qz + 2; ++cz) {
                        auto it = segGrid->find(key(cx, cz)); if (it == segGrid->end()) continue;
                        for (int ei : it->second) {
                            const RoadEdge& e = g->edges[static_cast<std::size_t>(ei)];
                            const Vec2& ra = g->nodes[e.a].pos; const Vec2& rb = g->nodes[e.b].pos;
                            const Vec2 ab = rb - ra; const Real l2 = ab.lengthSquared();
                            Real u = l2 > 1e-12 ? dot(q - ra, ab) / l2 : Real(0); u = std::max(Real(0), std::min(Real(1), u));
                            const Real d = (q - (ra + ab * u)).length() - e.width * Real(0.5);
                            if (d < best) { best = d; bestE = ei; }
                        }
                    }
                    if (bestE < 0 || faces(g->edges[static_cast<std::size_t>(bestE)])) ++street;
                    else ++freeway;
                }
                return street >= freeway;
            };
        }
    }
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
        const Poly2 footBeforePush = foot;
        foot = pushPolyClearOfRoads(foot);
        if (foot.size() < 3) { ++blocksAllCarriageway; continue; }
        if (area(foot) < p.minLotArea * 1.5) continue;
        dbg->blocks.push_back(foot);
        blockFoots.push_back(foot);

        BlockInfo bf;
        bf.pp.seed = mix(static_cast<uint32_t>(bi), p.seed);
        bf.pp.isFrontage = isFrontage;
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
                const Real cness = corenessAt(footC);
                const Real ct = std::max(Real(0), (cness - 0.5) * 2.0);
                bf.pp.frontWidth = 42 + 13 * ct;   // 42 -> 55 at the hub
                bf.pp.lotDepth = 48 + 12 * ct;     // 48 -> 60
                bf.pp.targetArea = 2400;           // the OBB fallback matches
                bf.lotSetback = 1.0;
                bf.buildChance = std::min(Real(1), p.buildChance + 0.06);
                // YARDS (site plan): a downtown street wall stands on the
                // lot line; the party-wall sides keep a hair so two
                // neighbours' facades never share a plane; a short rear yard.
                bf.yards = Yards{0.0, 0.3, 3.0};
                break; }
            case DistrictTag::Commercial: {  // narrow, deep retail frontage...
                // ...that widens toward downtown into MIDTOWN plates (skyscrapers v2 M3):
                // the shoulders of the skyline are commercial towers, and a tower needs
                // a plate the slenderness cap will let it rise on. 13 x 30 m at the rim,
                // 35 x 45 m where coreness is full.
                const Real cc = corenessAt(footC);
                bf.pp.frontWidth = 13 + 22 * cc; bf.pp.lotDepth = 30 + 15 * cc;
                bf.pp.targetArea = 300 + 1200 * cc; bf.lotSetback = 0.7;
                bf.yards = Yards{0.0, 0.3, 3.0};
                bf.buildChance = std::min(Real(1), p.buildChance + 0.06); break; }
            case DistrictTag::OldTown:     // small, tight, narrow
                bf.pp.frontWidth = 11; bf.pp.lotDepth = 22;
                bf.pp.targetArea = 210;
                bf.pp.minArea = std::min(ppMinArea, Real(80));
                bf.yards = Yards{0.0, 0.3, 2.0};
                bf.lotSetback = 0.5; bf.buildChance = 0.98; break;
            case DistrictTag::Industrial:  // big parcels
                bf.pp.frontWidth = 42; bf.pp.lotDepth = 52;
                bf.yards = Yards{2.0, 2.0, 2.0};
                bf.pp.targetArea = 700; bf.lotSetback = 1.2; break;
            case DistrictTag::Residential: // house lots with yards
                bf.pp.frontWidth = 18; bf.pp.lotDepth = 27;
                bf.yards = Yards{3.0, 1.5, 5.0};
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
                g.pad = pushPolyClearOfRoads(foot); g.pad = padClearOrEmpty(g.pad);
                (void)padCoversRoad(g.pad); (void)padOnCarriageway(g.pad);
                if (g.pad.empty()) continue;   // road-locked block: no square
                // The city square is DESIGNED: plaza, paths, fountain,
                // benches, hedges, tree spots (not a bare green pad).
                // Sculpted AFTER the grade rebind below (deferredParks) —
                // parks sampled the pre-terrace hillside here, so their
                // walking paths floated over the block-graded ground the
                // terrain actually shows (device: "walkways float ... as
                // part of plazas and parks"). Same ordering fix the pads
                // got in the buried-buildings round.
                out.push_back(std::move(g));
                deferredParks.push_back({out.size() - 1, mix(bf.pp.seed, 0xB10C2u)});
                continue;
            }
        }
        bool bisected = false;
        ParcelReject prj;
        std::vector<Lot> lots = subdivideBlock(foot, bf.pp, 0, &bisected, &prj);
        // EVERY LOT INSIDE ITS BLOCK -- the block as it CAME, before the road push. The push moves each
        // vertex away from its nearest road, and in a narrow neck between two roads the pushed vertices
        // cross: a self-intersecting footprint, whose ray casts and containment tests say yes to ground
        // outside the block. island_8_nature's block 406 grew 214,226 -> 217,287 m2 through the push with no
        // vertex outside it, and eight lots stood 11-15 m past its edge on the freeway's embankment (buried
        // 7 m, their pads clipped to the real block). A lot with a corner outside the block is dropped.
        {
            const std::size_t before = lots.size();
            lots.erase(std::remove_if(lots.begin(), lots.end(), [&](const Lot& L) {
                if (L.footprint.size() < 3) return false;
                const Vec2 c = centroid(L.footprint);
                for (const Vec2& v : L.footprint)
                    if (!pointInPolygon(footBeforePush, v + (c - v) * Real(0.02))) return true;
                if (nearFreeway)
                    for (const Vec2& v : L.footprint)
                        if (nearFreeway(v)) return true;
                return false;
            }), lots.end());
            prj.escaped += static_cast<int>(before - lots.size());
        }
        // A PARCEL IN A LANE (Glenn: "That's a road, there should be no lot
        // there", after being told twice it was fixed).
        //
        // The block interior is pushed clear of roads, but pushPolyClearOfRoads
        // DELETES up to 20% of vertices it cannot settle and returns the rest --
        // so `foot` can still cross a carriageway, and every lot parcelled out
        // of it inherits that. The BUILDING on such a lot is shrunk clear, which
        // is why every building-based check reported clean; the lot itself never
        // was, and landscaping scatters across the lot. Measured from the city's
        // own SVG (tools/city_svg_audit.py): 5 of 1702 lots reached into a lane,
        // worst 5.99 m.
        //
        // Dropping the lot is proportionate where dropping the BLOCK was not --
        // this is five parcels, not five city blocks.
        {
            const std::size_t before = lots.size();
            lots.erase(std::remove_if(lots.begin(), lots.end(),
                                      [&](const Lot& L) {
                                          return padOnCarriageway(L.footprint);
                                      }),
                       lots.end());
            lotsInCarriageway += static_cast<int>(before - lots.size());
        }
        if (bisected) ++dbg->bisectedBlocks;
        dbg->pEdgeShort += prj.edgeShort; dbg->pShallow += prj.shallow; dbg->pNotStreet += prj.notStreet;
        dbg->pMitered   += prj.mitered;   dbg->pOverlap += prj.overlap;
        dbg->pEscaped   += prj.escaped;   dbg->pTiny    += prj.tiny;
        dbg->pThin      += prj.thin;      dbg->pPlaced  += prj.placed;
        dbg->pClips     += prj.clips;     dbg->pLeftOverlapping += prj.leftOverlapping;
        dbg->pSameEdge  += prj.sameEdgeOverlap; dbg->pAtInsert += prj.overlapAtInsert; dbg->pConcave += prj.concavePiece + prj.concaveQ; dbg->pClipFailed += prj.clipFailed;
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
        // ONE BLOCK, ONE BUILDING (Glenn, 2026-09-21: "Some of the city blocks only create a lot
        // or two ... if only one lot can fit for whatever reason then it should take up the entire
        // city block ... I would accept having 1 lot with a large massive building on it rather
        // than a city block that has just a dinky lot"). A block the parcel walk left with two or
        // fewer building lots covering under half of it — an odd-shaped block between curving
        // streets, a wedge by a ramp, a block too shallow for the grain — is ONE site instead:
        // the whole interior, a landmark on it (architectBlockLandmark), always built.
        {
            constexpr Real kWholeBlockMinArea = 900.0;   // a block big enough to be worth a landmark
            constexpr Real kWholeBlockMinShort = 14.0;   // and wide enough to hold one
            const Real blockA = std::fabs(area(foot));
            int viable = 0;
            Real viableA = 0;
            for (const Lot& L : lots)
                if (!L.court && std::fabs(area(L.footprint)) >= bf.pp.minArea) {
                    ++viable;
                    viableA += std::fabs(area(L.footprint));
                }
            const OBB2 fb = orientedBoundingBox(foot);
            // ...on ground one pad can seat: the relief limit every lot's pad already obeys
            // (LotParams::maxPadRelief). Without it three of the lattice metro's whole-block towers
            // stood on blocks falling more than that and the floorplan census found them buried;
            // metro_lanes' candidates fall 5-6.6 m along 150 m blocks and seat fine. The block
            // grade fits a plane to this boundary, so the boundary's relief is the test.
            const Real kWholeBlockMaxRelief = p.maxPadRelief;
            Real lo = Real(1e30), hi = Real(-1e30);
            if (p.ground)
                for (const Vec2& v : foot) {
                    const Real g = p.ground(v.x, v.y);
                    lo = std::min(lo, g);
                    hi = std::max(hi, g);
                }
            const bool seatable = !p.ground || hi - lo <= kWholeBlockMaxRelief;
            // ...and on a block with a STREET to face: one walled in by freeway and ramps has no front
            // door anywhere (ParcelParams::isFrontage), so it stays open ground.
            bool hasStreet = !bf.pp.isFrontage;
            for (std::size_t i = 0; i < foot.size() && !hasStreet; ++i) hasStreet = bf.pp.isFrontage(foot[i], foot[(i + 1) % foot.size()]);
            // ...clear of any freeway or ramp, as every parcelled lot is
            if (hasStreet && nearFreeway)
                for (const Vec2& v : foot) if (nearFreeway(v)) { hasStreet = false; break; }
            if (!seatable && viable <= 2 && viableA < Real(0.5) * blockA && blockA >= kWholeBlockMinArea &&
                dbg->wholeBlocks < 24)
                LOG_INFO << "[citylots] whole block SKIPPED (relief " << (hi - lo) << " m across its edge) "
                         << static_cast<int>(blockA) << " m2 at " << static_cast<int>(centroid(foot).x) << " "
                         << static_cast<int>(centroid(foot).y);
            if (viable <= 2 && viableA < Real(0.5) * blockA && blockA >= kWholeBlockMinArea &&
                2 * std::min(fb.half[0], fb.half[1]) >= kWholeBlockMinShort && !padOnCarriageway(foot) &&
                seatable && hasStreet) {
                Lot whole;
                // the block AS IT CAME when the road push moved a vertex out of it (a folded push -- see the
                // parcelled lots' containment test above): a landmark never stands past its own block
                bool pushedOut = false;
                for (const Vec2& v : foot) if (!pointInPolygon(footBeforePush, v)) { pushedOut = true; break; }
                whole.footprint = pushedOut ? footBeforePush : foot;
                whole.area = blockA;
                whole.wholeBlock = true;
                // Faces its longest street edge (the door rule re-aims it at the nearest road).
                const Real sgn = signedArea(foot) >= 0 ? Real(1) : Real(-1);
                Real longest = -1;
                for (std::size_t i = 0; i < foot.size(); ++i) {
                    const Vec2 d = foot[(i + 1) % foot.size()] - foot[i];
                    const Real len = d.length();
                    if (len > longest && len > Real(1e-6)) {
                        longest = len;
                        whole.frontage = Vec2(d.y, -d.x) * (sgn / len);   // outward for this winding
                    }
                }
                // WHY it could not be parcelled, in the parcel walk's own words (this block only).
                if (dbg->wholeBlocks < 24)
                    LOG_INFO << "[citylots] whole block " << districtName(bf.tag) << " "
                             << static_cast<int>(blockA) << " m2, "
                             << static_cast<int>(2 * fb.half[0]) << " x " << static_cast<int>(2 * fb.half[1])
                             << " m, " << foot.size() << " edges: parcel walk made " << viable
                             << " lot(s) covering " << static_cast<int>(100 * viableA / blockA)
                             << "% (rejected edgeShort " << prj.edgeShort << ", shallow " << prj.shallow
                             << ", tiny " << prj.tiny << ", thin " << prj.thin << "; grain "
                             << static_cast<int>(bf.pp.frontWidth) << " x " << static_cast<int>(bf.pp.lotDepth)
                             << " m) at " << static_cast<int>(centroid(foot).x) << " "
                             << static_cast<int>(centroid(foot).y) << " relief " << (hi - lo);
                lots.clear();
                lots.push_back(std::move(whole));
                ++dbg->wholeBlocks;
            }
        }
        // BIG-BOX STORES (Glenn, 2026-10-01: "Big box stores like Costco or Bestbuy"): out from downtown, a big
        // commercial or industrial block -- 55 m and more across -- may be ONE store's site instead of a street of
        // lots: the box at the back, its parking lot in front (Massing::BigBox in pass C). About one such block in
        // three, a dozen a city at most.
        if ((bf.tag == DistrictTag::Commercial || bf.tag == DistrictTag::Industrial) && dbg->bigBoxBlocks < 16 &&
            !(lots.size() == 1 && lots.front().wholeBlock)) {
            const OBB2 fb = orientedBoundingBox(foot);
            const Real shortS = 2 * std::min(fb.half[0], fb.half[1]), longS = 2 * std::max(fb.half[0], fb.half[1]);
            const Real blockA = std::fabs(area(foot));
            Hash bbRng(mix(bf.pp.seed, 0xB16B0Bu));
            // (62 across: after the setbacks the store layout needs 54 -- a 30 m store and its 24 m lot; at 55 the
            // site came out ~50 deep, the layout failed, and the box fallback filled the block with no parking)
            bool ok = shortS >= 62 && longS >= 70 && longS <= 260 && blockA >= 4500 && blockA <= 40000 &&
                      blockA > 0.6 * shortS * longS && corenessAt(centroid(foot)) < 0.35 && bbRng.unit() < 0.35 &&
                      !padOnCarriageway(foot);
            // ...and spread out: a store's catchment, not a strip of them (700 m between big boxes)
            for (const Vec2& c : bigBoxCentres) if (ok && (c - centroid(foot)).length() < 700) ok = false;
            if (ok && p.ground) {
                // THE STREETS ROUND IT: 2 m outside each block vertex, on the street's own height where the host
                // knows it (a lane city's decks), else the ground there
                const Vec2 fcen = centroid(foot);
                Real lo = 1e30, hi = -1e30;
                for (const Vec2& v : foot) {
                    const Vec2 out = v + normalize(v - fcen) * 2.0;
                    Real g = 0;
                    if (!(p.streetHeight && p.streetHeight(out.x, out.y, &g))) g = p.ground(out.x, out.y);
                    lo = std::min(lo, g); hi = std::max(hi, g);
                }
                ok = hi - lo <= std::min(p.maxBigBoxRelief, p.maxPadRelief);
            }

            for (std::size_t i = 0; i < foot.size() && ok && nearFreeway; ++i) if (nearFreeway(foot[i])) ok = false;
            {   // inside the block as it came (a vertex ON its line is inside: nudge it toward the middle)
                const Vec2 fc = centroid(foot);
                for (const Vec2& v : foot) if (ok && !pointInPolygon(footBeforePush, v + (fc - v) * Real(0.02))) ok = false;
            }
            // FACES the long side with the most street along it: a lane city's block edge is its street's curve,
            // sampled every few metres, so the frontage is summed per side of the block's box, not one edge
            Vec2 front(0, 0);
            Real longest = -1;
            {
                const int la = fb.longAxis();
                const Vec2 nrm = fb.axis[1 - la];   // the long sides face +/- this
                const Real sgn = signedArea(foot) >= 0 ? Real(1) : Real(-1);
                Real run[2] = {0, 0};
                for (std::size_t i = 0; i < foot.size() && ok; ++i) {
                    const Vec2 a = foot[i], c = foot[(i + 1) % foot.size()];
                    if (bf.pp.isFrontage && !bf.pp.isFrontage(a, c)) continue;
                    const Vec2 d = c - a;
                    const Real len = d.length();
                    if (len < 1e-6) continue;
                    const Vec2 out = Vec2(d.y, -d.x) * (sgn / len);
                    const Real al = dot(out, nrm);
                    if (al > 0.8) run[0] += len;
                    else if (al < -0.8) run[1] += len;
                }
                const int k = run[0] >= run[1] ? 0 : 1;
                longest = run[k];
                front = k == 0 ? nrm : nrm * -1.0;
            }
            if (std::getenv("RT_BIGBOX_DEBUG"))
                std::printf("[bigbox?] %s short %.0f long %.0f area %.0f core %.2f frontage %.0f -> %d\n", districtName(bf.tag),
                            shortS, longS, blockA, corenessAt(centroid(foot)), longest, ok ? 1 : 0);
            if (ok && longest > 40) {
                Lot whole;
                whole.footprint = foot;
                whole.area = blockA;
                whole.wholeBlock = true;
                whole.bigBox = true;
                // AN INDOOR MALL instead of a store (stage 2): on a block deep enough for its concourse and long
                // enough for its units, one in 1.5 km
                {
                    bool m = shortS >= 70 && longS >= 100 && bbRng.unit() < 0.6;
                    for (const Vec2& c0 : dbg->mallAt) if ((c0 - centroid(foot)).length() < 1500) m = false;
                    if (m) { whole.mall = true; ++dbg->mallBlocks; dbg->mallAt.push_back(centroid(foot)); }
                    // ...the AMERICAN MALL where the block is big enough for its wings (a court and two 40 m wings)
                    if (m && shortS >= 70 && longS >= 150) whole.mallWings = true;
                }
                whole.frontage = front;
                whole.district = lots.empty() ? 0 : lots.front().district;
                lots.clear();
                lots.push_back(std::move(whole));
                ++dbg->bigBoxBlocks;
                bigBoxCentres.push_back(centroid(foot));
            }
        }
        // THE BUS DEPOT (Glenn, 2026-10-08: "There should probably be a dedicated bus depot they return to at end of
        // day"): out at the edge of town, an industrial or commercial block -- 48 m and more across, near-flat -- may be
        // the buses' yard: rows of bus bays, a maintenance shed at the back, a fence round it (sculptBusDepot). One in
        // 1.8 km, so each town has its own; the citysim sends each route's buses to the nearest when service ends.
        // A TOWN WITH NO DEPOT YET (the city's next ten #6: four of the island's towns' routes had none): a block 3 km
        // from any depot may hold one in a residential district too, and is not rolled away
        bool depotAlone = true;
        {
            const Vec2 fc0 = centroid(foot);
            for (const Vec2& c : dbg->depotAt) if ((c - fc0).length() < 3000.0) depotAlone = false;
        }
        if ((bf.tag == DistrictTag::Industrial || bf.tag == DistrictTag::Commercial || (depotAlone && bf.tag == DistrictTag::Residential)) &&
            dbg->depotBlocks < 10 && !(lots.size() == 1 && (lots.front().wholeBlock || lots.front().bigBox))) {
            const OBB2 fb = orientedBoundingBox(foot);
            const Real shortS = 2 * std::min(fb.half[0], fb.half[1]), longS = 2 * std::max(fb.half[0], fb.half[1]);
            const Real blockA = std::fabs(area(foot));
            const Vec2 fc = centroid(foot);
            Hash dRng(mix(bf.pp.seed, 0xB05D3B07u));
            const char* why = nullptr;
            if (shortS < 48 || longS < 60 || longS > 150 || shortS > 110) why = "size";
            else if (blockA < 0.65 * shortS * longS) why = "shape";
            else if (corenessAt(fc) > (bf.tag == DistrictTag::Industrial ? 0.45 : 0.25)) why = "central";
            else if (padOnCarriageway(foot)) why = "road";
            for (const Vec2& c : dbg->depotAt) if (!why && (c - fc).length() < 1800) why = "near another";
            for (std::size_t i = 0; i < foot.size() && !why && nearFreeway; ++i) if (nearFreeway(foot[i])) why = "freeway";
            for (const Vec2& v : foot) if (!why && !pointInPolygon(footBeforePush, v + (fc - v) * Real(0.02))) why = "pushed";
            if (!why && p.ground) {   // near-flat: the yard is laid on the ground, not graded
                Real lo = 1e30, hi = -1e30;
                for (const Vec2& v : foot) { const Real g = p.ground(v.x, v.y); lo = std::min(lo, g); hi = std::max(hi, g); }
                const Real g = p.ground(fc.x, fc.y); lo = std::min(lo, g); hi = std::max(hi, g);
                if (hi - lo > 1.6) why = "slope";
            }
            if (!why && bf.tag == DistrictTag::Commercial && !depotAlone && dRng.unit() < 0.5) why = "roll";
            // its GATE on the long side with the most street along it
            Vec2 front(0, 0);
            if (!why) {
                const Vec2 nrm = fb.axis[1 - fb.longAxis()];
                const Real sgn = signedArea(foot) >= 0 ? Real(1) : Real(-1);
                Real run[2] = {0, 0};
                for (std::size_t i = 0; i < foot.size(); ++i) {
                    const Vec2 a = foot[i], c = foot[(i + 1) % foot.size()];
                    if (bf.pp.isFrontage && !bf.pp.isFrontage(a, c)) continue;
                    const Vec2 d = c - a;
                    const Real len = d.length();
                    if (len < 1e-6) continue;
                    const Real al = dot(Vec2(d.y, -d.x) * (sgn / len), nrm);
                    if (al > 0.8) run[0] += len; else if (al < -0.8) run[1] += len;
                }
                if (std::max(run[0], run[1]) < 30) why = "frontage";
                front = run[0] >= run[1] ? nrm : nrm * -1.0;
            }
            if (std::getenv("RT_DEPOT_DEBUG"))
                std::printf("[depot?] %s at %.0f %.0f: %.0f x %.0f core %.2f -> %s\n", districtName(bf.tag), fc.x, fc.y, longS,
                            shortS, corenessAt(fc), why ? why : "yes");
            if (!why) {
                Lot yard;
                yard.footprint = foot;
                yard.area = blockA;
                yard.wholeBlock = true;
                yard.depot = true;
                yard.frontage = front;
                yard.district = lots.empty() ? 0 : lots.front().district;
                lots.clear();
                lots.push_back(std::move(yard));
                ++dbg->depotBlocks;
                dbg->depotAt.push_back(fc);
            }
        }
        // THE UNIVERSITY CAMPUS (Glenn: "a university campus over blocks (library, classrooms, offices, dorms, quads,
        // sports fields)"): one a city, on a big, near-rectangular residential or commercial block between downtown
        // and the edge. Its halls stand in a strip along each long side -- teaching halls, the library, a residence
        // hall -- and between them the QUAD, a green with paths, trees and benches.
        if ((bf.tag == DistrictTag::Residential || bf.tag == DistrictTag::Commercial) && dbg->campusBlocks < 1 &&
            !(lots.size() == 1 && (lots.front().wholeBlock || lots.front().bigBox))) {
            const OBB2 fb = orientedBoundingBox(foot);
            const int la = fb.longAxis();
            const Vec2 ua = fb.axis[la], va = fb.axis[1 - la];
            const Real L = fb.half[la], W = fb.half[1 - la];
            const Real blockA = std::fabs(area(foot));
            const Real core = corenessAt(centroid(foot));
            Hash cRng(mix(bf.pp.seed, 0xCA4905u));
            const char* why = 2 * W < 72 ? "narrow" : 2 * L < 90 ? "short" : 2 * L > 260 ? "long"
                            : blockA <= 0.70 * 4 * L * W ? "not rectangular" : core >= 0.7 ? "downtown"
                            : padOnCarriageway(foot) ? "carriageway" : "";
            (void)cRng;
            bool ok = !*why;
            if (ok && p.ground) {
                Real lo = 1e30, hi = -1e30;
                for (const Vec2& v : foot) { const Real g = p.ground(v.x, v.y); lo = std::min(lo, g); hi = std::max(hi, g); }
                ok = hi - lo <= p.maxPadRelief;
                if (!ok) why = "relief";
            }
            for (std::size_t i = 0; i < foot.size() && ok && nearFreeway; ++i) if (nearFreeway(foot[i])) { ok = false; why = "freeway"; }
            std::vector<Lot> campus;
            // pulled in from the block's box until every hall lies inside it (a lane block's edges are its streets'
            // curves, so the box's corners are often outside)
            for (Real m : {3.0, 6.0, 9.5, 13.0}) if (ok) {
                campus.clear();
                if (2 * W - 2 * m < 60 || 2 * L - 2 * m < 80) break;
                const Vec2 c = fb.center;
                const Real gap = 7.0;
                const Real d = std::clamp(2 * W * 0.27, Real(17), Real(24));
                auto rect = [&](Real u0, Real u1, Real v0, Real v1) {
                    Poly2 r = {c + ua * u0 + va * v0, c + ua * u1 + va * v0, c + ua * u1 + va * v1, c + ua * u0 + va * v1};
                    ensureCCW(r);
                    return r;
                };
                // a hall at the block's end SHORTENS its outer end until it is inside -- the block's corners are its
                // streets' rounded junctions, which clip a rectangle's outer corner however far it is pulled in
                auto in = [&](const Vec2& q) { return pointInPolygon(foot, q) && pointInPolygon(footBeforePush, q); };
                auto add = [&](Real u0, Real u1, Real v0, Real v1, uint8_t role, const Vec2& front) {
                    for (int k = 0; k < 15; ++k) {
                        const bool end0 = in(c + ua * u0 + va * v0) && in(c + ua * u0 + va * v1);
                        const bool end1 = in(c + ua * u1 + va * v0) && in(c + ua * u1 + va * v1);
                        if (end0 && end1) break;
                        if (!end0) u0 += 2.0;
                        if (!end1) u1 -= 2.0;
                    }
                    const Poly2 r = rect(u0, u1, v0, v1);
                    for (const Vec2& q : r) if (!in(q)) return false;
                    if (u1 - u0 < 24.0) return false;   // a hall too short for its rooms
                    Lot l;
                    l.footprint = r;
                    l.area = std::fabs(area(r));
                    l.campus = role;
                    l.frontage = front;
                    l.district = lots.empty() ? 0 : lots.front().district;
                    campus.push_back(std::move(l));
                    return true;
                };
                const Real uA = -L + m, uB = L - m;
                // the strip along +va: a teaching hall and the library; along -va: a teaching hall and a residence hall
                const bool fits = add(uA, -gap * 0.5, W - m - d, W - m, 1, va) && add(gap * 0.5, uB, W - m - d, W - m, 2, va) &&
                                  add(uA, -gap * 0.5, -W + m, -W + m + d, 1, va * -1.0) &&
                                  add(gap * 0.5, uB, -W + m, -W + m + d, 3, va * -1.0) &&
                                  add(uA, uB, -W + m + d + 2.0, W - m - d - 2.0, 4, va);
                if (fits) break;
            }
            if (ok && campus.size() != 5) { ok = false; why = "a hall would leave the block"; }
            if (std::getenv("RT_CAMPUS_DEBUG"))
                std::printf("[campus?] %s %.0f x %.0f core %.2f -> %s\n", districtName(bf.tag), 2 * L, 2 * W, core, ok ? "YES" : why);
            if (ok) {
                lots = std::move(campus);
                ++dbg->campusBlocks;
                dbg->campusAt.push_back(centroid(foot));
                dbg->campusWhat.push_back("campus");
            }
        }
        // THE OPEN-AIR MALL (~/.claude/plans/nightlife-and-malls.md stage 3; Glenn: "Some nicer shopping malls like
        // Santana row" / "outdoor pedestrian walkways like they do at these upscale malls"): on a near-rectangular,
        // flat commercial block, a car-free PASEO down its length, a row of two- and three-storey shop buildings each
        // side facing it, PASSAGES between them through to the streets. One a town, 1.2 km apart.
        if (bf.tag == DistrictTag::Commercial && dbg->paseoBlocks < 6 &&
            !(lots.size() == 1 && (lots.front().wholeBlock || lots.front().bigBox)) && !(lots.size() > 0 && lots.front().campus)) {
            const OBB2 fb = orientedBoundingBox(foot);
            const int la = fb.longAxis();
            const Vec2 ua = fb.axis[la], va = fb.axis[1 - la];
            const Real L = fb.half[la], W = fb.half[1 - la];
            const Real core = corenessAt(centroid(foot));
            const char* why = 2 * W < 60 ? "narrow" : 2 * L < 90 ? "short" : 2 * L > 240 ? "long"
                            : std::fabs(area(foot)) <= 0.72 * 4 * L * W ? "not rectangular" : core >= 0.8 ? "downtown"
                            : padOnCarriageway(foot) ? "carriageway" : "";
            bool ok = !*why;
            for (const Vec2& c0 : dbg->paseoAt) if (ok && (c0 - centroid(foot)).length() < 1200) { ok = false; why = "another near"; }
            for (const Vec2& c0 : dbg->campusAt) if (ok && (c0 - centroid(foot)).length() < 400) { ok = false; why = "the campus"; }
            if (ok && p.ground) {
                Real lo = 1e30, hi = -1e30;
                for (const Vec2& v : foot) { const Real g = p.ground(v.x, v.y); lo = std::min(lo, g); hi = std::max(hi, g); }
                ok = hi - lo <= p.maxPadRelief;
                if (!ok) why = "relief";
            }
            for (std::size_t i = 0; i < foot.size() && ok && nearFreeway; ++i) if (nearFreeway(foot[i])) { ok = false; why = "freeway"; }
            std::vector<Lot> mall;
            auto in = [&](const Vec2& q) { return pointInPolygon(foot, q) && pointInPolygon(footBeforePush, q); };
            for (Real m : {3.0, 6.0, 9.5}) if (ok) {
                mall.clear();
                const Real P = 14.0;                                       // the paseo's width
                const Real d = std::clamp(W - m - P * 0.5, Real(13), Real(22));   // the rows' depth
                if (W - m < P * 0.5 + 13 || 2 * (L - m) < 80) break;
                const Vec2 c = fb.center;
                auto rect = [&](Real u0, Real u1, Real v0, Real v1) {
                    Poly2 r = {c + ua * u0 + va * v0, c + ua * u1 + va * v0, c + ua * u1 + va * v1, c + ua * u0 + va * v1};
                    ensureCCW(r);
                    return r;
                };
                const Real uA = -L + m, uB = L - m, gap = 6.0;
                const int n = 2 * (L - m) > 150 ? 3 : 2;               // buildings a row; the passages between them
                const Real bl = (uB - uA - gap * (n - 1)) / n;
                bool fits = true;
                for (int side : {1, -1})
                    for (int k = 0; k < n && fits; ++k) {
                        Real u0 = uA + k * (bl + gap), u1 = u0 + bl;
                        const Real v0 = side > 0 ? P * 0.5 : -P * 0.5 - d, v1 = side > 0 ? P * 0.5 + d : -P * 0.5;
                        // an end building SHORTENS its outer end until it is inside (the junctions round the corners)
                        for (int t = 0; t < 12; ++t) {
                            const bool e0 = in(c + ua * u0 + va * v0) && in(c + ua * u0 + va * v1);
                            const bool e1 = in(c + ua * u1 + va * v0) && in(c + ua * u1 + va * v1);
                            if (e0 && e1) break;
                            if (!e0 && k == 0) u0 += 2.0; else if (!e1 && k == n - 1) u1 -= 2.0; else break;
                        }
                        const Poly2 r = rect(u0, u1, v0, v1);
                        for (const Vec2& q : r) if (!in(q)) fits = false;
                        if (u1 - u0 < 16.0) fits = false;
                        Lot l;
                        l.footprint = r;
                        l.area = std::fabs(area(r));
                        l.paseo = 1;
                        l.frontage = va * static_cast<Real>(-side);   // the front faces the paseo
                        l.district = lots.empty() ? 0 : lots.front().district;
                        mall.push_back(std::move(l));
                    }
                // the paseo: the strip between the rows, end to end
                Lot w;
                w.footprint = rect(uA, uB, -P * 0.5 + 0.3, P * 0.5 - 0.3);
                for (const Vec2& q : w.footprint) if (!in(q)) fits = false;
                w.area = std::fabs(area(w.footprint));
                w.paseo = 2;
                w.frontage = ua;
                w.district = lots.empty() ? 0 : lots.front().district;
                mall.push_back(std::move(w));
                if (fits) break;
                mall.clear();
            }
            if (ok && mall.empty()) { ok = false; why = "the rows would leave the block"; }
            if (std::getenv("RT_PASEO_DEBUG"))
                std::printf("[paseo?] %.0f x %.0f core %.2f -> %s\n", 2 * L, 2 * W, core, ok ? "YES" : why);
            if (ok) {
                // its footprint (the rows and the walk), graded to ONE level after the parcelling: on a sloping block
                // the walk draped the slope while every building stood flat at its own height, and the paving rose
                // over one end of each shop (Glenn: "the ground is raised up so some buildings are sunken")
                Poly2 whole;
                for (const Lot& l : mall) for (const Vec2& q : l.footprint) whole.push_back(q);
                paseoGrades.push_back(convexHull(whole));
                lots = std::move(mall);
                ++dbg->paseoBlocks;
                dbg->paseoAt.push_back(centroid(foot));
            }
        }
        // THE CAMPUS SPORTS FIELD: once the campus stands, the first block within 500 m that holds a pitch (and its
        // stand) becomes the field -- one lot, the pitch sculpted in pass C (sculptSportsField).
        if (dbg->campusBlocks > 0 && dbg->sportsBlocks < 1 && !(lots.size() == 1 && (lots.front().wholeBlock || lots.front().bigBox)) &&
            !(lots.size() > 0 && (lots.front().campus || lots.front().paseo)) && (bf.tag == DistrictTag::Residential || bf.tag == DistrictTag::Commercial)) {
            const OBB2 fb = orientedBoundingBox(foot);
            const Real shortS = 2 * std::min(fb.half[0], fb.half[1]), longS = 2 * std::max(fb.half[0], fb.half[1]);
            bool ok = (centroid(foot) - dbg->campusAt.front()).length() < 500 && shortS >= 66 && longS >= 78 &&
                      std::fabs(area(foot)) > 0.70 * shortS * longS && !padOnCarriageway(foot);
            if (ok && p.ground) {
                Real lo = 1e30, hi = -1e30;
                for (const Vec2& v : foot) { const Real gr = p.ground(v.x, v.y); lo = std::min(lo, gr); hi = std::max(hi, gr); }
                ok = hi - lo <= p.maxPadRelief;
            }
            for (std::size_t i = 0; i < foot.size() && ok && nearFreeway; ++i) if (nearFreeway(foot[i])) ok = false;
            if (std::getenv("RT_CAMPUS_DEBUG"))
                std::printf("[sports?] %.0f x %.0f, %.0f m from the campus -> %d\n", longS, shortS,
                            (centroid(foot) - dbg->campusAt.front()).length(), ok ? 1 : 0);
            if (ok) {
                Lot field;
                field.footprint = foot;
                field.area = std::fabs(area(foot));
                field.campus = 5;
                field.district = lots.empty() ? 0 : lots.front().district;
                lots.clear();
                lots.push_back(std::move(field));
                ++dbg->sportsBlocks;
                dbg->campusAt.push_back(centroid(foot));
                dbg->campusWhat.push_back("sports field");
            }
        }
        // THE DORM BLOCK (the campus gap pass): once the campus stands, the first block within 350 m that holds two
        // residence halls -- one along each long side, facing its street -- with a COURTYARD between them (a quad:
        // lawn, paths, benches). Its students live here as well as in the quad block's hall.
        if (dbg->campusBlocks > 0 && dbg->dormBlocks < 1 && !(lots.size() == 1 && (lots.front().wholeBlock || lots.front().bigBox)) &&
            !(lots.size() > 0 && (lots.front().campus || lots.front().paseo)) && (bf.tag == DistrictTag::Residential || bf.tag == DistrictTag::Commercial)) {
            const OBB2 fb = orientedBoundingBox(foot);
            const int la = fb.longAxis();
            const Vec2 ua = fb.axis[la], va = fb.axis[1 - la];
            const Real L = fb.half[la], W = fb.half[1 - la];
            const Real dist = (centroid(foot) - dbg->campusAt.front()).length();
            const char* why = dist > 350 ? "far" : 2 * W < 48 ? "narrow" : 2 * L < 60 ? "short" : 2 * L > 220 ? "long"
                            : std::fabs(area(foot)) <= 0.70 * 4 * L * W ? "not rectangular" : corenessAt(centroid(foot)) >= 0.7 ? "downtown"
                            : padOnCarriageway(foot) ? "carriageway" : "";
            bool ok = !*why;
            if (ok && p.ground) {
                Real lo = 1e30, hi = -1e30;
                for (const Vec2& v : foot) { const Real gr = p.ground(v.x, v.y); lo = std::min(lo, gr); hi = std::max(hi, gr); }
                ok = hi - lo <= p.maxPadRelief;
                if (!ok) why = "relief";
            }
            for (std::size_t i = 0; i < foot.size() && ok && nearFreeway; ++i) if (nearFreeway(foot[i])) { ok = false; why = "freeway"; }
            std::vector<Lot> dorms;
            auto in = [&](const Vec2& q) { return pointInPolygon(foot, q) && pointInPolygon(footBeforePush, q); };
            for (Real m : {3.0, 6.0, 9.5, 13.0}) if (ok) {
                dorms.clear();
                if (2 * W - 2 * m < 44 || 2 * L - 2 * m < 50) break;
                const Vec2 c = fb.center;
                const Real d = std::clamp(2 * W * 0.3, Real(15), Real(19));
                auto add = [&](Real u0, Real u1, Real v0, Real v1, uint8_t role, const Vec2& front) {
                    for (int k = 0; k < 15; ++k) {   // a hall at the block's end shortens to stay inside (rounded corners)
                        const bool end0 = in(c + ua * u0 + va * v0) && in(c + ua * u0 + va * v1);
                        const bool end1 = in(c + ua * u1 + va * v0) && in(c + ua * u1 + va * v1);
                        if (end0 && end1) break;
                        if (!end0) u0 += 2.0;
                        if (!end1) u1 -= 2.0;
                    }
                    Poly2 r = {c + ua * u0 + va * v0, c + ua * u1 + va * v0, c + ua * u1 + va * v1, c + ua * u0 + va * v1};
                    ensureCCW(r);
                    for (const Vec2& q : r) if (!in(q)) return false;
                    if (role == 3 && u1 - u0 < 30.0) return false;
                    Lot l;
                    l.footprint = r;
                    l.area = std::fabs(area(r));
                    l.campus = role;
                    l.frontage = front;
                    l.district = lots.empty() ? 0 : lots.front().district;
                    dorms.push_back(std::move(l));
                    return true;
                };
                const Real uA = -L + m, uB = L - m;
                if (add(uA, uB, W - m - d, W - m, 3, va) && add(uA, uB, -W + m, -W + m + d, 3, va * -1.0) &&
                    add(uA, uB, -W + m + d + 2.0, W - m - d - 2.0, 6, va))
                    break;
            }
            if (ok && dorms.size() != 3) { ok = false; why = "a hall would leave the block"; }
            if (std::getenv("RT_CAMPUS_DEBUG"))
                std::printf("[dorms?] %.0f x %.0f, %.0f m from the campus -> %s\n", 2 * L, 2 * W, dist, ok ? "YES" : why);
            if (ok) {
                lots = std::move(dorms);
                ++dbg->dormBlocks;
                dbg->campusAt.push_back(centroid(foot));
                dbg->campusWhat.push_back("dorms");
            }
        }
        // ATTACHED BUILDINGS (Glenn, 2026-10-01: "in the dense part of town ... buildings right next to each
        // other"): in an old town or a commercial block, a lot's SIDE line that a neighbouring lot shares (the
        // same street run's next slot) is a party line -- the building stands on it, blank against its
        // neighbour, instead of a side yard. Front and rear lines never are.
        if ((bf.tag == DistrictTag::OldTown || bf.tag == DistrictTag::Commercial) && lots.size() >= 2)
            for (std::size_t ai = 0; ai < lots.size(); ++ai) {
                Lot& A = lots[ai];
                if (A.court || A.wholeBlock || A.campus || A.paseo) continue;
                Poly2 pa = A.footprint;
                ensureCCW(pa);
                for (std::size_t i = 0; i < pa.size() && A.partyCount < 2; ++i) {
                    const Vec2 a = pa[i], b = pa[(i + 1) % pa.size()];
                    const Vec2 d = b - a;
                    const Real len = d.length();
                    if (len < 3.0) continue;
                    const Vec2 dn = d * (1.0 / len), nrm(dn.y, -dn.x);
                    if (std::fabs(dot(nrm, normalize(A.frontage))) > 0.5) continue;   // a front or rear line
                    bool shared = false;
                    for (std::size_t bi = 0; bi < lots.size() && !shared; ++bi) {
                        if (bi == ai || lots[bi].court || lots[bi].wholeBlock || lots[bi].campus || lots[bi].paseo) continue;
                        const Poly2& pb = lots[bi].footprint;
                        for (std::size_t j = 0; j < pb.size() && !shared; ++j) {
                            const Vec2 c = pb[j], e = pb[(j + 1) % pb.size()];
                            if (std::fabs(cross(dn, c - a)) > 0.15 || std::fabs(cross(dn, e - a)) > 0.15) continue;
                            const Real t0 = dot(c - a, dn), t1 = dot(e - a, dn);
                            const Real ov = std::min(len, std::max(t0, t1)) - std::max(Real(0), std::min(t0, t1));
                            shared = ov >= std::min(Real(4.0), len * 0.6);
                        }
                    }
                    if (!shared) continue;
                    A.partyN[A.partyCount] = nrm;
                    A.partyAt[A.partyCount] = dot(a, nrm);
                    ++A.partyCount;
                }
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
                g.pad = pushPolyClearOfRoads(lots[li].footprint); g.pad = padClearOrEmpty(g.pad);
                (void)padCoversRoad(g.pad); (void)padOnCarriageway(g.pad);
                if (g.pad.empty()) continue;
                // Deferred like the city square above: sculpt on the graded
                // ground, not the pre-terrace hill.
                out.push_back(std::move(g));
                deferredParks.push_back(
                    {out.size() - 1,
                     mix(bf.pp.seed, 0xC0947u + static_cast<uint32_t>(li))});
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
            // The PAVEMENT is emitted LATE (deferredAlleys, after the grade
            // rebind): the inline emit sampled the pre-terrace ground — the
            // same ordering bug the parks had — and stamped no flatten, so
            // the ribbon floated over graded hillsides and coarse-LOD tiles
            // bulged through it (device: "floating walkways", round 3 — the
            // scored hillside ribbons were ALLEY pavements between rows
            // whose houses the relief gate had rejected).
            deferredAlleys.push_back({A, B});
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
    if (p.ground && outGrade && !blockFoots.empty() && !p.smoothGround) {
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
            if (p.groundWith) {
                // PRIORITY-CORRECT rebind (floorplan-conformance round): the
                // host rebuilds the sampler with the grades folded into ONE
                // region set next to the roads, so road-vs-grade priority
                // resolves the way the final terrain will. The old wrapper
                // (applyFlatten(grades) over the road-carved base) let the
                // grade plane override the road deck inside its +4.5 m
                // overlap — padPlaneFor and the foundation drape then read a
                // surface the mesh never shows, exactly at the frontage.
                auto dilated = p.groundWith(*outGrade);
                p.groundDilatedFn = dilated;
                p.ground = [dilated](Real x, Real z) { return dilated(x, z, Real(0)); };
            } else {
                // Legacy composition for hosts without the hook (flat levels,
                // old callers): correct only where grades and roads don't
                // overlap.
                auto base = p.ground;
                auto flat =
                    std::make_shared<std::vector<TerrainFlatten>>(*outGrade);
                p.ground = [base, flat](Real x, Real z) {
                    return applyFlatten(*flat, x, z, base(x, z));
                };
            }
        }
    }

    // MESH-CONFORMING ground for everything that must visually SIT on the
    // terrain: corner samples on the finest rendered CDLOD grid, bilinear
    // across the cell — the tile's own interpolation. The analytic sampler is
    // exact between grid points where the mesh is not; walkways placed by it
    // floated over every cell the mesh cut differently (measured 5.8 m worst
    // at terrace lips by the tile-fidelity gate). Sculptors keep calling a
    // plain ground fn; the fn they get IS the mesh's surface.
    std::function<Real(Real, Real)> meshGround = p.ground;
    if (p.ground && p.groundMeshCell > 0.5) {
        const Real cell = p.groundMeshCell;
        // The tile's own corner query: terrain_lod samples terrainHeight with
        // flattenDilate = step * 1.45, so a corner near a narrow band or a
        // terrace lip resolves to the SAME plane the rendered mesh shows.
        // Without the dilation the conforming sampler still diverged by whole
        // planes exactly at lips (the 5.8 m tile-gate measurement).
        const Real dil = cell * Real(1.45);
        auto corner = p.groundDilatedFn
                          ? std::function<Real(Real, Real)>(
                                [f = p.groundDilatedFn, dil](Real x, Real z) {
                                    return f(x, z, dil);
                                })
                          : p.ground;   // legacy hosts: best effort, undilated
        meshGround = [corner, cell](Real x, Real z) -> Real {
            const Real gx = std::floor(x / cell) * cell;
            const Real gz = std::floor(z / cell) * cell;
            const Real fx = (x - gx) / cell, fz = (z - gz) / cell;
            const Real h00 = corner(gx, gz), h10 = corner(gx + cell, gz);
            const Real h01 = corner(gx, gz + cell), h11 = corner(gx + cell, gz + cell);
            return h00 * (1 - fx) * (1 - fz) + h10 * fx * (1 - fz) +
                   h01 * (1 - fx) * fz + h11 * fx * fz;
        };
    }

    // Deferred park sculpting (see the parcelling loop): now that p.ground is
    // the grade-aware sampler, parks drape their plazas, paths and furniture
    // on the same terraced ground the terrain will show.
    for (const DeferredPark& dp : deferredParks) {
        LotBuilding& g = out[dp.lot];
        sculptPark(g, g.pad, g.height, meshGround, dp.seed, outParts, outGrade,
                   p.groundMeshCell > 0.5 ? p.groundMeshCell : Real(3.0));
    }
    // Deferred alley pavements: draped on the SAME terraced ground as the
    // parks, and each segment stamps its band into the flatten set so the
    // rendered terrain agrees at every LOD (see the sculptPark spoke ramps).
    if (outParts && !deferredAlleys.empty()) {
        // GROUND-RELATIVE ribbon (kDrapedPartBase); the RAMP it stamps grades the
        // terrain and so reads the absolute ground.
        RenderMesh& path = (*outParts)[drapedSlot(PartId::Path)];
        auto gy = [&](const Vec2& v) {
            return meshGround ? meshGround(v.x, v.y) : Real(0);
        };
        for (const DeferredAlley& da : deferredAlleys) {
            const Vec2 dirN = normalize(da.b - da.a);
            const Vec2 perp(-dirN.y, dirN.x);
            const Real hw = p.alleyWidth * Real(0.5);
            const Real L = (da.b - da.a).length();
            const Real segLen = p.groundMeshCell > 0.5
                                    ? std::min(Real(3.0), std::max(Real(1.2),
                                          p.groundMeshCell * Real(0.75)))
                                    : Real(3.0);
            const int segs = std::max(1, static_cast<int>(L / segLen));
            for (int sgi = 0; sgi < segs; ++sgi) {
                const Vec2 q0 = da.a + dirN * (L * sgi / segs);
                const Vec2 q1 = da.a + dirN * (L * (sgi + 1) / segs);
                // Lateral samples too: an alley crossing a cross-slope BANKS
                // with it instead of hanging its downhill edge in the air.
                const Real y0L = gy(q0 - perp * hw) + Real(0.06);
                const Real y0R = gy(q0 + perp * hw) + Real(0.06);
                const Real y1L = gy(q1 - perp * hw) + Real(0.06);
                const Real y1R = gy(q1 + perp * hw) + Real(0.06);
                const Real lift = Real(0.06);
                MeshBuilder::emitQuad(
                    path,
                    Vec3(q0.x - perp.x * hw, lift, q0.y - perp.y * hw),
                    Vec3(q0.x + perp.x * hw, lift, q0.y + perp.y * hw),
                    Vec3(q1.x + perp.x * hw, lift, q1.y + perp.y * hw),
                    Vec3(q1.x - perp.x * hw, lift, q1.y - perp.y * hw),
                    Vec3(0, 1, 0), Vec3(0.42, 0.41, 0.40));
                if (outGrade)
                    outGrade->push_back(makeFlattenRamp(
                        Vec3(q0.x, 0, q0.y), Vec3(q1.x, 0, q1.y),
                        (y0L + y0R) * Real(0.5) - Real(0.05),
                        (y1L + y1R) * Real(0.5) - Real(0.05), hw + 0.6, 2.5));
            }
        }
    }

    // ---- THE PASEOS, graded flat (one level for the walk and every shop on it), on top of the block grades ----
    if (p.ground && !paseoGrades.empty()) {
        auto flats = std::make_shared<std::vector<TerrainFlatten>>();
        for (const Poly2& pg : paseoGrades) {
            if (pg.size() < 3) continue;
            // the level: the mean of the ground over it (a grid of samples inside)
            const OBB2 ob = orientedBoundingBox(pg);
            Real sum = 0;
            int n = 0;
            for (int i = 0; i <= 6; ++i)
                for (int j = 0; j <= 4; ++j) {
                    const Vec2 q = ob.center + ob.axis[0] * (ob.half[0] * (i / 3.0 - 1.0) * 0.9) +
                                   ob.axis[1] * (ob.half[1] * (j / 2.0 - 1.0) * 0.9);
                    if (!pointInPolygon(pg, q)) continue;
                    sum += p.ground(q.x, q.y);
                    ++n;
                }
            if (n == 0) continue;
            std::vector<Vec3> poly;
            for (const Vec2& v : pg) poly.push_back(Vec3(v.x, 0, v.y));
            flats->push_back(makeFlattenPad(poly, sum / n, 5.0));
        }
        if (!flats->empty()) {
            if (outGrade) outGrade->insert(outGrade->end(), flats->begin(), flats->end());
            auto base = p.ground;
            p.ground = [base, flats](Real x, Real z) { return applyFlatten(*flats, x, z, base(x, z)); };
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
            cands[ci].landmark = cands[ci].lot.bigBox || cands[ci].lot.campus || cands[ci].lot.depot ? -1 : placed[ci];   // a big-box block keeps its store, a campus its halls
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
            // `natural`: hillside or road-locked ground, which stays green; any other lot that does not build is
            // first offered the OPEN-LOT program (a car park, a playground, a community garden, a pocket park).
            // `street`: whether the lot meets a street (a car park needs a way in for cars).
            auto emitGreen = [&](bool natural = false, bool street = true) {
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
                g.pad = pushPolyClearOfRoads(lot.footprint); g.pad = padClearOrEmpty(g.pad);
                (void)padCoversRoad(g.pad); (void)padOnCarriageway(g.pad);
                if (g.pad.empty()) return;     // road-locked sliver: no green
                if (!natural && p.openLots && outParts && area(g.pad) >= 140.0) {
                    // THE OPEN-LOT PROGRAM: the uses to try, in order, by district; ~15% stay a vacant green
                    Hash orng(mix(pp.seed, static_cast<uint32_t>(li) * 29u + 7u));
                    const Real roll = orng.unit();
                    std::vector<int> order;   // 0 car park, 1 playground, 2 community garden, 3 pocket park
                    if (orng.unit() >= 0.15) {
                        switch (blockTag) {
                            case DistrictTag::Financial: case DistrictTag::Commercial: case DistrictTag::OldTown:
                                order = roll < 0.45 ? std::vector<int>{0, 3, 2} : roll < 0.78 ? std::vector<int>{3, 2, 0} : std::vector<int>{2, 3, 0};
                                break;
                            case DistrictTag::Industrial:
                                order = roll < 0.7 ? std::vector<int>{0, 3} : std::vector<int>{3, 0};
                                break;
                            default:
                                order = roll < 0.35 ? std::vector<int>{1, 3, 2} : roll < 0.65 ? std::vector<int>{2, 1, 3} : std::vector<int>{3, 1, 2};
                                break;
                        }
                    }
                    const Vec2 fr = lot.frontage.length() > 1e-6 ? lot.frontage : Vec2(0, 1);
                    const uint32_t s = mix(pp.seed, static_cast<uint32_t>(li) * 13u + 5u);
                    for (int use : order) {
                        if (use == 0 && !street) continue;
                        LotBuilding o = g;
                        bool ok = false;
                        if (use == 0) {
                            o.type = "depot";   // paved and levelled, no place for the schedules: the host's yard path
                            o.recipe = "car_park";
                            ok = sculptCarPark(o, o.pad, fr, s, outParts);
                            if (ok) {
                                o.pavedLot = o.sealed.front();
                                if (p.ground) {
                                    Real sum = 0;
                                    for (const Vec2& q : o.pavedLot) sum += p.ground(q.x, q.y);
                                    const Vec2 qc = centroid(o.pavedLot);
                                    sum += p.ground(qc.x, qc.y);
                                    o.groundY = sum / static_cast<Real>(o.pavedLot.size() + 1);
                                }
                                o.paveY = o.groundY;
                                ++dbg->openCarParks;
                            }
                        } else if (use == 1) {
                            o.type = "park"; o.recipe = "playground"; o.height = 0.18;
                            ok = sculptPlayground(o, o.pad, fr, s, outParts);
                            if (ok) ++dbg->openPlaygrounds;
                        } else if (use == 2) {
                            o.type = "park"; o.recipe = "community_garden"; o.height = 0.18;
                            ok = sculptCommunityGarden(o, o.pad, fr, s, outParts);
                            if (ok) ++dbg->openGardens;
                        } else {
                            if (std::getenv("RT_OPENLOT_DEBUG")) {
                                const OBB2 ob = orientedBoundingBox(o.pad);
                                std::printf("[openlot] fallback park: %s area %.0f obb %.1f x %.1f street %d order0 %d\n", districtName(blockTag),
                                            area(o.pad), 2 * ob.half[0], 2 * ob.half[1], street ? 1 : 0, order.front());
                            }
                            o.type = "park"; o.recipe = "pocket_park"; o.height = 0.18;
                            sculptPark(o, o.pad, o.height, meshGround, s, outParts, nullptr,
                                       p.groundMeshCell > 0.5 ? p.groundMeshCell : Real(3.0));
                            ok = true;
                            ++dbg->openParks;
                        }
                        if (!ok) continue;
                        o.block = cand.block;
                        o.district = districtName(blockTag);
                        pruneTreeSpotsIntoRoad(o.treeSpots);
                        out.push_back(std::move(o));
                        return;
                    }
                }
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
                    emitGreen(/*natural=*/false, /*street=*/false);
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
                            sculptPark(u, u.pad, u.height, meshGround, s, outParts,
                                       nullptr,
                                       p.groundMeshCell > 0.5 ? p.groundMeshCell : Real(3.0));
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
            if (cand.landmark < 0 && !lot.wholeBlock && !lot.campus && rng.unit() > buildChance) {
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
                    emitGreen(/*natural=*/true); continue;
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
            // SITE PLAN (skyscrapers v2 M1, ADR-0086): the building stands on
            // the largest frontage-aligned RECTANGLE the lot holds with the
            // district's yards — buildings are rectilinear, lots are not, and
            // a trapezoid's leftovers are ground, not walls. The rectangle is
            // searched on the lot, then on the lot inset by the ladder the
            // plan clearance uses, until its corners and edge midpoints clear
            // the road corridors (the raster itself never consults the roads:
            // clearOfRoads walks every edge, far too slow per cell).
            bool siteRectified = false;
            const SiteFrame siteFrameOf = siteFrame(lot.footprint, lot.frontage);
            // RT_SITE_PLAN=0 skips the rectification, RT_SITE_PLAN=inset rectifies with
            // uniform 1 m yards — a bisect switch for the terrain gates, not a feature.
            static const char* kSitePlanMode = std::getenv("RT_SITE_PLAN");
            const bool sitePlanOff = kSitePlanMode && std::string(kSitePlanMode) == "0";
            Yards siteYards = bf.yards;
            if (kSitePlanMode && std::string(kSitePlanMode) == "inset") siteYards = Yards{1.0, 1.0, 1.0};
            if (!sitePlanOff) {
                bool rectified = false, anyRect = false;
                const SiteFrame& frame = siteFrameOf;
                for (Real t : {Real(0), Real(0.8), Real(1.6), Real(2.6), Real(3.6)}) {
                    const Poly2 host = t > 0 ? inset(lot.footprint, t) : lot.footprint;
                    if (host.size() < 3 || area(host) < 40) break;
                    // The raster spans cell centres, so a rectangle comes out
                    // up to a cell short of the exact one; a lot that clears
                    // the sliver floor by less than that must not fail here.
                    Poly2 rect = largestAlignedRect(host, frame, siteYards, 0.5,
                                                    std::max(Real(4), minShort - 0.5));
                    if (rect.size() != 4) continue;
                    anyRect = true;
                    if (!rectFits(rect, lot.footprint)) continue;
                    retakeSite(std::move(rect));
                    rectified = true;
                    break;
                }
                if (rectified) ++sitesRectified;
                else if (anyRect) ++sitesRectUnfit;
                else ++sitesNoRect;
                if (!rectified && std::getenv("RT_SITE_DEBUG")) {
                    const OBB2 lo = orientedBoundingBox(lot.footprint);
                    std::printf("[site-fail] %s at (%.0f, %.0f) area %.0f obb %.1fx%.1f n %zu frontage (%.2f, %.2f) minShort %.1f %s\n",
                                districtName(blockTag), centroid(lot.footprint).x, centroid(lot.footprint).y,
                                area(lot.footprint), 2 * lo.half[0], 2 * lo.half[1], lot.footprint.size(),
                                lot.frontage.x, lot.frontage.y, minShort, anyRect ? "rect-unfit" : "no-rect");
                    std::printf("[site-fail]   yards %.1f/%.1f/%.1f poly", bf.yards.front, bf.yards.side, bf.yards.rear);
                    for (const Vec2& v : lot.footprint) std::printf(" (%.2f,%.2f)", v.x, v.y);
                    std::printf("\n");
                }
                siteRectified = rectified;
            }
            // ATTACHED (Glenn, 2026-10-01): on a lot with shared side lines, the rectangle's sides run out to them
            // -- 5 cm short, so neighbouring walls never coincide -- and the building stands wall to wall with its
            // neighbours (the party flags follow the final plan, below). Landmarks keep their yards.
            if (siteRectified && site.size() == 4 && lot.partyCount > 0 && cand.landmark < 0 && !lot.wholeBlock) {
                const SiteFrame& fm = siteFrameOf;
                Real x0 = 1e30, x1 = -1e30, y0 = 1e30, y1 = -1e30;
                for (const Vec2& v : site) {
                    const Vec2 f = fm.toFrame(v);
                    x0 = std::min(x0, f.x); x1 = std::max(x1, f.x);
                    y0 = std::min(y0, f.y); y1 = std::max(y1, f.y);
                }
                Real nx0 = x0, nx1 = x1;
                for (int k = 0; k < lot.partyCount; ++k) {
                    const Vec2 N = lot.partyN[k];
                    const Real nu = dot(N, fm.u), nv = dot(N, fm.v);
                    if (std::fabs(nu) < 0.99) continue;   // a slanted side line: keep the yard
                    // where the line crosses the rectangle's front and back edges, in frame x
                    const Real base = lot.partyAt[k] - dot(fm.origin, N);
                    const Real xa = (base - y0 * nv) / nu, xb = (base - y1 * nv) / nu;
                    if (nu < 0) nx0 = std::min(nx0, std::max(xa, xb) + 0.05);
                    else nx1 = std::max(nx1, std::min(xa, xb) - 0.05);
                }
                if ((nx0 < x0 - 0.02 || nx1 > x1 + 0.02) && nx1 - nx0 < (x1 - x0) + 4.0) {
                    Poly2 rect{fm.toWorld({nx0, y0}), fm.toWorld({nx1, y0}), fm.toWorld({nx1, y1}), fm.toWorld({nx0, y1})};
                    if (signedArea(rect) < 0) std::reverse(rect.begin(), rect.end());
                    if (rectFits(rect, lot.footprint)) {
                        retakeSite(std::move(rect));
                        ++dbg->attachedSites;
                    }
                }
            }
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
            b.lot = lot.footprint;
            ensureCCW(b.lot);
            {   // The pad bound: the parcel, 1 m inside every street-facing edge but the frontage.
                auto distToBlock = [&](const Vec2& q) {
                    Real best = 1e30;
                    const Poly2& f = bf.foot;
                    for (std::size_t i = 0; i < f.size(); ++i) {
                        const Vec2 a = f[i], c = f[(i + 1) % f.size()];
                        const Vec2 ac = c - a;
                        const Real l2 = ac.lengthSquared();
                        Real t = l2 > 1e-12 ? dot(q - a, ac) / l2 : 0.0;
                        t = std::max(Real(0), std::min(Real(1), t));
                        best = std::min(best, (q - (a + ac * t)).length());
                    }
                    return best;
                };
                Poly2 bound = b.lot;
                // A CONCAVE parcel (a whole-block site: 40-80 edges round a curving block) cannot be
                // cut by a half-plane per edge — each of its own edges' half-planes slices away the
                // rest of it, and the bound collapsed to four coincident points, so the landmark on
                // it got no pad and stood 3-4 m under its uphill ground. There each edge MOVES by its
                // own amount instead (offsetPolygonEdges); a convex parcel keeps the exact clip.
                bool lotConcave = false;
                {
                    const std::size_t nl = b.lot.size();
                    const Real sgn = signedArea(b.lot) >= 0 ? Real(1) : Real(-1);
                    for (std::size_t i = 0; i < nl && !lotConcave; ++i) {
                        const Vec2 e0 = b.lot[(i + 1) % nl] - b.lot[i];
                        const Vec2 e1 = b.lot[(i + 2) % nl] - b.lot[(i + 1) % nl];
                        if (sgn * cross(e0, e1) < -1e-6 * e0.length() * e1.length()) lotConcave = true;
                    }
                }
                std::vector<Real> outward(b.lot.size(), Real(0));
                for (std::size_t i = 0; i < b.lot.size(); ++i) {
                    const Vec2 a = b.lot[i], c = b.lot[(i + 1) % b.lot.size()];
                    const Vec2 d = c - a;
                    const Real len = d.length();
                    if (len < 1e-9) continue;
                    const Vec2 n(d.y / len, -d.x / len);   // CCW: outward
                    const bool onStreet = bf.foot.size() >= 3 && distToBlock(a) < 0.6 && distToBlock(c) < 0.6;
                    const bool front = lotSideOf(n, lot.frontage) == LotSide::Front;
                    Real pullIn = (onStreet && !front) ? Real(1.0) : Real(-0.3);
                    // ...but never past the building's own wall: a side yard is 0.3 m
                    // downtown, and a pad ending inside the plan leaves that wall on
                    // unflattened ground (the lab census: one wall buried 1.4 m).
                    if (pullIn > 0 && site.size() >= 3) {
                        Real planGap = 1e30;
                        for (const Vec2& v : site) planGap = std::min(planGap, -dot(n, v - a));
                        pullIn = std::max(Real(0), std::min(pullIn, planGap - Real(0.05)));
                    }
                    // Only ever IN: the convex clip below cannot grow a polygon (a half-plane 0.3 m
                    // outside an edge cuts nothing), so its "slack" never widened a pad — and a pad
                    // grown past its lot line outranks the road beside it (the deck-poke gate).
                    outward[i] = std::min(Real(0), -pullIn);
                    if (lotConcave) continue;
                    const Poly2 cut = clipHalfPlane(bound, n, dot(n, a) - pullIn);
                    if (cut.size() >= 3) bound = cut;
                }
                if (lotConcave) {
                    // b.lot is CCW here (the loop's outward normals assume it), so the per-edge
                    // amounts index the same edges offsetPolygonEdges walks.
                    Poly2 moved = offsetPolygonEdges(b.lot, outward);
                    if (moved.size() >= 3 && area(moved) > Real(0.5) * area(b.lot)) bound = std::move(moved);
                }
                b.padBound = bound;
            }
            // RT_LOT_AT=x,z prints the lot whose parcel holds the point: its recipe, plane,
            // frontage and pad bound — the probe for "what stands at this poke".
            if (const char* at = std::getenv("RT_LOT_AT")) {
                double px = 0, pz = 0;
                auto nearParcel = [&](double qx, double qz) {
                    const Vec2 q(qx, qz);
                    if (pointInPolygon(b.lot, q)) return true;
                    for (std::size_t i = 0; i < b.lot.size(); ++i) {
                        const Vec2 a = b.lot[i], c = b.lot[(i + 1) % b.lot.size()];
                        const Vec2 ac = c - a; const Real l2 = ac.lengthSquared();
                        Real t = l2 > 1e-12 ? dot(q - a, ac) / l2 : 0.0; t = std::max(Real(0), std::min(Real(1), t));
                        if ((q - (a + ac * t)).length() < 3.0) return true;
                    }
                    return false;
                };
                if (std::sscanf(at, "%lf,%lf", &px, &pz) == 2 && nearParcel(px, pz)) {
                    std::printf("[lot-at] %s at (%.1f, %.1f) frontage (%.2f, %.2f) parcel", districtName(blockTag), b.site.x, b.site.y,
                                lot.frontage.x, lot.frontage.y);
                    for (const Vec2& v : b.lot) std::printf(" (%.1f,%.1f)", v.x, v.y);
                    std::printf("\n[lot-at]   padBound");
                    for (const Vec2& v : b.padBound) std::printf(" (%.1f,%.1f)", v.x, v.y);
                    std::printf("\n");
                }
            }
            // The ground around the buildable: the lot minus the rectangle the
            // gates settled on (an aspect or fill rescue may have re-taken it).
            if (siteRectified) b.open = openSpacePieces(lot.footprint, site, siteFrameOf);
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
            const Real coreness = corenessAt(b.site);
            BuildingRecipe rec =
                lot.bigBox && lot.mallWings ? architectMallWings(mix(pp.seed, static_cast<uint32_t>(li) * 7u + 3u)) :
                lot.bigBox && lot.mall ? architectMall(mix(pp.seed, static_cast<uint32_t>(li) * 7u + 3u)) :
                lot.bigBox ? architectBigBox(mix(pp.seed, static_cast<uint32_t>(li) * 7u + 3u)) :
                lot.depot ? architectBusDepot() :
                lot.campus ? architectCampus(lot.campus, mix(pp.seed, static_cast<uint32_t>(li) * 7u + 3u)) :
                lot.paseo ? architectPaseo(lot.paseo, mix(pp.seed, static_cast<uint32_t>(li) * 7u + 3u)) :
                cand.landmark >= 0
                    ? architectLandmark(static_cast<LandmarkKind>(cand.landmark),
                                        shortSide, area(site),
                                        mix(pp.seed,
                                            static_cast<uint32_t>(li) * 7u + 3u))
                : lot.wholeBlock
                    ? architectBlockLandmark(tag, shortSide, area(site),
                                             mix(pp.seed, static_cast<uint32_t>(li) * 7u + 3u),
                                             coreness)
                    : architectPick(tag, shortSide, area(site),
                                    mix(pp.seed,
                                        static_cast<uint32_t>(li) * 7u + 3u),
                                    coreness,
                                    p.archetypeBook.empty() ? nullptr
                                                            : &p.archetypeBook);
            // THE SHOP MIX (~/.claude/plans/nightlife-and-malls.md): a residential street's everyday shops, a high
            // street's eating and drinking, and -- round each town's NIGHTLIFE STRIP -- low-rise buildings of bars,
            // restaurants and clubs (Glenn: "these all don't need to be located on skyscrapers").
            {
                const bool urban = tag == DistrictTag::Commercial || tag == DistrictTag::Financial || tag == DistrictTag::OldTown;
                const bool night = urban && nightlifeAt(b.site);
                rec.params.shopMix = lot.paseo ? 3 : night ? 2 : urban ? 1 : 0;
                // JAPANESE TOWERS: a downtown tower of 12+ floors may be a department store over its lobby, and one of
                // 20+ may have restaurant floors at the top (Glenn: "malls like this in the skyscrapers like in Japan")
                if ((tag == DistrictTag::Financial || tag == DistrictTag::Commercial) && !lot.wholeBlock && !lot.bigBox &&
                    !lot.campus && !lot.paseo && rec.params.floors >= 12 && !rec.params.parkingDecks) {
                    const uint32_t ht = mix(pp.seed, static_cast<uint32_t>(li) * 131u + 17u);
                    if (ht % 4u == 0) rec.params.storeFloors = static_cast<uint8_t>(3 + (ht >> 4) % 3u);
                    if (rec.params.floors >= 20 && (ht >> 8) % 3u == 0) rec.params.dineFloors = static_cast<uint8_t>(1 + (ht >> 12) % 2u);
                    if (rec.params.storeFloors || rec.params.dineFloors) ++dbg->towerVenues;
                }
                if (night && !lot.paseo && rec.massing == BuildingRecipe::Massing::LotPlan && !lot.wholeBlock && !lot.bigBox &&
                    !lot.campus && cand.landmark < 0) {
                    const uint32_t hn = mix(pp.seed, static_cast<uint32_t>(li) * 29u + 11u);
                    rec.params.floors = std::min(rec.params.floors, 1 + static_cast<int>(hn % 2u));
                    rec.params.groundRetail = true;
                    rec.params.curtainWall = false;
                    ++dbg->nightlifeBuildings;
                }
            }
            b.type = rec.placeType;
            b.recipe = rec.name;
            b.block = cand.block;
            b.district = districtName(tag);
            b.color = colorFor(b.type);
            if (rec.massing == BuildingRecipe::Massing::Park) {
                b.height = 0.18;  // low green pad — stays UNDER the road deck
                                  // lift so park edges tuck below the asphalt
                b.pad = pushPolyClearOfRoads(lot.footprint); b.pad = padClearOrEmpty(b.pad);
                (void)padCoversRoad(b.pad); (void)padOnCarriageway(b.pad);
                if (b.pad.empty()) continue;   // road-locked sliver: no park
                if (rec.name == "paseo") {
                    // its passages: the gaps between the shop buildings already grown along it
                    const OBB2 ob = orientedBoundingBox(b.pad);
                    const int la = ob.longAxis();
                    const Vec2 ua = ob.axis[la];
                    std::vector<std::pair<Real, Real>> spans;
                    for (const LotBuilding& h : out) {
                        if (h.recipe != "paseo_shops" || (h.site - ob.center).length() > 160.0) continue;
                        Real lo = 1e30, hi = -1e30;
                        for (const Vec2& q : h.lot) { const Real t = dot(q - ob.center, ua); lo = std::min(lo, t); hi = std::max(hi, t); }
                        spans.push_back({lo, hi});
                    }
                    std::sort(spans.begin(), spans.end());
                    std::vector<Real> passages;
                    for (std::size_t k = 0; k + 1 < spans.size(); ++k)
                        if (spans[k + 1].first - spans[k].second > 3.0 && spans[k + 1].first - spans[k].second < 12.0)
                            passages.push_back((spans[k].second + spans[k + 1].first) * 0.5);
                    std::sort(passages.begin(), passages.end());
                    passages.erase(std::unique(passages.begin(), passages.end(), [](Real x, Real y) { return std::fabs(x - y) < 2.0; }),
                                   passages.end());
                    sculptPaseo(b, b.pad, mix(pp.seed, static_cast<uint32_t>(li) * 13u + 5u), outParts,
                                p.groundMeshCell > 0.5 ? p.groundMeshCell : Real(3.0),
                                [&](const Vec2& q) { return !clearOfRoads(q); }, passages);
                } else if (rec.name == "bus_depot") {
                    // THE BUS DEPOT's yard; a block that cannot hold a row of bays is a park after all
                    if (sculptBusDepot(b, b.pad, lot.frontage, mix(pp.seed, static_cast<uint32_t>(li) * 13u + 5u), outParts)) {
                        // the yard is PAVED and levelled: flattened to its mean ground (which keeps the forest's trees
                        // and the meadow off it); the drape lays the yard on that
                        b.pavedLot = b.sealed.front();
                        if (p.ground) {
                            Real sum = 0;
                            for (const Vec2& q : b.pavedLot) sum += p.ground(q.x, q.y);
                            const Vec2 qc = centroid(b.pavedLot);
                            sum += p.ground(qc.x, qc.y);
                            b.groundY = sum / static_cast<Real>(b.pavedLot.size() + 1);
                        }
                        b.paveY = b.groundY;
                    } else {
                        b.type = "park";
                        b.recipe = "park";
                        --dbg->depotBlocks;
                        sculptPark(b, b.pad, b.height, meshGround, mix(pp.seed, static_cast<uint32_t>(li) * 13u + 5u), outParts,
                                   nullptr, p.groundMeshCell > 0.5 ? p.groundMeshCell : Real(3.0));
                    }
                } else if (rec.name == "campus_quad" || rec.name == "dorm_courtyard") {
                    // the halls round it are grown before it (the quad is the block's last lot): their quad doors
                    std::vector<std::pair<Vec2, Vec2>> hallDoors;
                    const Vec2 qc = centroid(b.pad);
                    for (const LotBuilding& h : out) {
                        if (h.recipe != "teaching_hall" && h.recipe != "campus_library" && h.recipe != "residence_hall") continue;
                        for (const BuildingUnit& u : h.units)
                            for (const DoorSpec& d : u.doors)
                                if (d.back && (d.foot - qc).length() < 90.0 && dot(qc - d.foot, d.normal) > 0)
                                    hallDoors.push_back({d.foot, d.normal});
                    }
                    sculptQuad(b, b.pad, mix(pp.seed, static_cast<uint32_t>(li) * 13u + 5u), outParts,
                               p.groundMeshCell > 0.5 ? p.groundMeshCell : Real(3.0),
                               [&](const Vec2& q) { return !clearOfRoads(q); }, rec.name == "campus_quad", hallDoors);
                    if (std::getenv("RT_CAMPUS_DEBUG")) {
                        std::printf("[quad] %s: %zu hall doors onto it\n", rec.name.c_str(), hallDoors.size());
                        for (const LotBuilding& h : out) {
                            if (h.recipe != "teaching_hall" && h.recipe != "campus_library" && h.recipe != "residence_hall") continue;
                            for (const BuildingUnit& u : h.units) {
                                std::printf("[quad]   %s at %.0f %.0f: backDoor %d walkable %d, %zu doors:", h.recipe.c_str(), h.site.x, h.site.y,
                                            u.params.backDoor ? 1 : 0, u.params.walkableGround ? 1 : 0, u.doors.size());
                                for (const DoorSpec& d : u.doors) std::printf(" (%s %.0f %.0f n %.2f %.2f, %.0f m)", d.back ? "back" : "front", d.foot.x, d.foot.y, d.normal.x, d.normal.y, (d.foot - qc).length());
                                std::printf("\n");
                            }
                        }
                    }
                }
                else if (rec.name != "sports_field" ||
                    !sculptSportsField(b, b.pad, mix(pp.seed, static_cast<uint32_t>(li) * 13u + 5u), outParts))
                    sculptPark(b, b.pad, b.height, meshGround,
                               mix(pp.seed, static_cast<uint32_t>(li) * 13u + 5u),
                               outParts, nullptr,
                               p.groundMeshCell > 0.5 ? p.groundMeshCell : Real(3.0));
                pruneTreeSpotsIntoRoad(b.treeSpots);
                if (std::getenv("RT_CAMPUS_DEBUG") && lot.campus)
                    std::printf("[campus lot] role %d %s padMesh %zu verts\n", lot.campus, rec.name.c_str(), b.padMesh.vertices.size());
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
                    if (!faces(e)) continue;   // a building fronts a street, never a freeway or a ramp
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
            // a paseo's shops front the PASEO, not the street behind them (they keep shopfronts there too)
            if (lot.paseo == 1 && lot.frontage.length() > 1e-6) {
                const Vec2 fd = normalize(lot.frontage);
                bp.faceDir = Vec3(fd.x, 0, fd.y);
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
                    // The MESH is the guarantee: the collider IS this plan,
                    // extruded to a prism by the loader (ADR-0080; the old
                    // "visual-only, no collider" note predated that), so the
                    // plan's own vertices clearing the corridors keeps facade
                    // AND collider off the street.
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
                if (!fit) {
                    dbg->rejClear++;
                    // A LOT IN THE ROAD (Glenn, 2026-09-17: "one lot being
                    // built in the middle of a street which is placing trees in
                    // the road"). Failing the fit used to fall through with
                    // planOk = false -- and EVERY downstream consumer reads
                    // `planOk ? plan : site`: the building's own plan (epl), its
                    // pad plane, its base Y. `site` is the raw parcel polygon,
                    // which has NEVER been clearance-checked. So the rejection
                    // was counted and then ignored, and the lot built across the
                    // carriageway with its landscaping following it there.
                    //
                    // A plan that cannot be made to clear the road is GREEN. On
                    // metro_v2 that is 3 lots of 1,413, which is the right price
                    // for never putting a facade or a collider in a traffic lane.
                    emitGreen(/*natural=*/true);
                    continue;
                }
                planOk = fit;
            }
            // COURTYARD massing (device: "a lot of same-y looking ones"): a
            // big boxy mid-rise lot carves a court into its BACK side — away
            // from the street — so the mass reads as an L / U / T plan with
            // wings instead of yet another extruded rectangle. The carve is
            // inward-only, so road clearance established above still holds.
            Poly2 courtNotch;   // the carve's court, recorded for the site plan (ADR-0086)
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
                        // rect plan its corners poke past the cleared polygon.
                        // The old check tested the ROADS ONLY, never the lot,
                        // so a courtyard mass could stand off its own parcel
                        // and into the neighbour's building (device: "I see
                        // some lots or buildings intersecting each other").
                        // Corners must clear the roads AND stay on the lot.
                        bool ok = true;
                        const Vec2 cc = centroid(cand);
                        for (const Vec2& q : cand) {
                            if (roads && !clearOfRoads(q)) { ok = false; break; }
                            if (!pointInPolygon(plan, q + (cc - q) * 0.02)) {
                                ok = false; break;
                            }
                        }
                        if (ok) {
                            plan = cand;
                            courtNotch = {C + v * hv + u * (s0 + w), C + v * (hv - dpt) + u * (s0 + w),
                                          C + v * (hv - dpt) + u * s0, C + v * hv + u * s0};
                            ensureCCW(courtNotch);
                        }
                    }
                }
            }
            // TOWER IN THE PLAZA (ADR-0086 point 5; the 1961 New York model —
            // Seagram: a 29 x 44 m slab on a 60 x 90 m lot behind a 27 m plaza).
            // On a rectified site deep enough for it, the tower takes 55-70 % of
            // the width and stands 32-42 % of the depth back from the avenue;
            // the strip in front is the plaza, paved by the lot's plate and
            // dressed by sculptForecourt. The tower is slender by design, so it
            // is re-capped on ITS plate and the plan-quality scaling (which
            // reads a small plan on a big site as a pinched wedge) is skipped.
            Poly2 plazaPoly;
            if (std::getenv("RT_SITE_DEBUG") && rec.massing == BuildingRecipe::Massing::TowerInPlaza)
                std::printf("[plaza-probe] %s planOk %d rectified %d site %zu\n", rec.name.c_str(), planOk ? 1 : 0, siteRectified ? 1 : 0, site.size());
            if (planOk && siteRectified && rec.massing == BuildingRecipe::Massing::TowerInPlaza) {
                const SiteFrame& f = siteFrameOf;
                Real fx0 = 1e30, fx1 = -1e30, fy0 = 1e30, fy1 = -1e30;
                for (const Vec2& v : site) {
                    const Vec2 q = f.toFrame(v);
                    fx0 = std::min(fx0, q.x); fx1 = std::max(fx1, q.x);
                    fy0 = std::min(fy0, q.y); fy1 = std::max(fy1, q.y);
                }
                const Real W = fx1 - fx0, D = fy1 - fy0;
                if (std::getenv("RT_SITE_DEBUG")) std::printf("[plaza-probe]   W %.1f D %.1f\n", W, D);
                // A downtown lot is 48-60 m deep; a 28 m one still holds a tower
                // with a 10 m forecourt (a plaza the width of the lot and the depth
                // of a small square).
                if (W >= 26 && D >= 28) {
                    const Real P = std::clamp(D * rng.range(0.32, 0.42), Real(10), Real(30));
                    const Real tw = std::clamp(W * rng.range(0.55, 0.70), Real(20), Real(46));
                    const Real td = D - P;
                    if (td >= 16) {
                        const Real cx0 = (fx0 + fx1) * 0.5;
                        Poly2 tower{f.toWorld({cx0 - tw * 0.5, fy0 + P}), f.toWorld({cx0 + tw * 0.5, fy0 + P}),
                                    f.toWorld({cx0 + tw * 0.5, fy1}), f.toWorld({cx0 - tw * 0.5, fy1})};
                        ensureCCW(tower);
                        plan = tower;
                        plazaPoly = {f.toWorld({fx0, fy0}), f.toWorld({fx1, fy0}), f.toWorld({fx1, fy0 + P}), f.toWorld({fx0, fy0 + P})};
                        ensureCCW(plazaPoly);
                        // Re-cap on the tower's own plate (the recipe capped on the site's).
                        const int maxFloors = std::max(1, static_cast<int>(std::min(tw, td) * 6.0 / std::max(Real(2.4), bp.floorHeight)) - 1);
                        bp.floors = std::min(bp.floors, maxFloors);
                        bp.setbackFloors = 0;   // a slab in a plaza rises sheer
                        bp.setbackEvery = 0;
                        bp.envelope = BuildingParams::Envelope::None;
                    }
                }
            }
            // THE BIG BOX (Glenn, 2026-10-01: "Big box stores like Costco or Bestbuy"): on the block's rectified
            // site the store stands at the back -- 55-65 % of the depth, most of the width -- and the strip in front,
            // to the street it faces, is its PARKING LOT (sculptParking dresses it on the lot's plate).
            Poly2 parkingPoly;
            Poly2 parkingBack;   // a second lot behind the building (the mall's, out to the back street)
            if (planOk && siteRectified && rec.massing == BuildingRecipe::Massing::BigBox) {
                const SiteFrame& f = siteFrameOf;
                Real fx0 = 1e30, fx1 = -1e30, fy0 = 1e30, fy1 = -1e30;
                for (const Vec2& v : site) {
                    const Vec2 q = f.toFrame(v);
                    fx0 = std::min(fx0, q.x); fx1 = std::max(fx1, q.x);
                    fy0 = std::min(fy0, q.y); fy1 = std::max(fy1, q.y);
                }
                const Real W = fx1 - fx0, D = fy1 - fy0;
                // THE AMERICAN MALL (bigBox 6): wings off a court at the site's middle -- along the front both ways,
                // and back / forward where the site is deep enough -- its outline the wings' union
                if (bp.bigBox == 6) {
                    const Real a = 20.0, edgeM = 4.0;
                    const Real cx = (fx0 + fx1) * 0.5;
                    // ITS PARKING LOT (Glenn: "the parking round it"): in front, between the street and the court's
                    // doors, where the block is deep enough for a row module (26 m) -- the wings pushed back behind
                    // it. Tried in turn, the first that fits the site: a wing back and a 44 m lot; no wing back and the
                    // lot as deep as the block allows; the wings in the middle with no lot (as before) -- each with
                    // the long wings as long as they fit (a block's ends are often its streets' curves)
                    const Real spare = D - 2 * edgeM - 2 * a;
                    auto layout = [&](Real P, bool back, bool forward, Real Ax) -> bool {
                        const Real cy = P > 0 ? fy0 + P + 1.0 + a : (fy0 + fy1) * 0.5;
                        const Real room = P > 0 ? fy1 - edgeM - cy : D * 0.5 - edgeM;
                        uint8_t arms = 0x3;   // +u, -u
                        Real Ab = 0, Af = 0;
                        if (back) { if (room < a + 28) return false; arms |= 0x4; Ab = std::min(room, Real(90)); }
                        if (forward) { if (P > 0 || room < a + 28) return false; arms |= 0x8; Af = std::min(room, Real(70)); }
                        std::vector<Vec2> q;   // CCW in the frame (x along the front, y back from it)
                        q.push_back({cx + a, cy - a});
                        q.push_back({cx + Ax, cy - a}); q.push_back({cx + Ax, cy + a});
                        if (arms & 0x4) { q.push_back({cx + a, cy + a}); q.push_back({cx + a, cy + Ab}); q.push_back({cx - a, cy + Ab}); }
                        q.push_back({cx - a, cy + a});
                        q.push_back({cx - Ax, cy + a}); q.push_back({cx - Ax, cy - a});
                        if (arms & 0x8) { q.push_back({cx - a, cy - a}); q.push_back({cx - a, cy - Af}); q.push_back({cx + a, cy - Af}); }
                        Poly2 mp;
                        for (const Vec2& v : q) mp.push_back(f.toWorld(v));
                        ensureCCW(mp);
                        for (const Vec2& v : mp) if (!pointInPolygon(site, v)) return false;
                        Poly2 lotP;
                        if (P > 0) {   // the lot: the front strip, as wide as the site is there
                            lotP = {f.toWorld({fx0, fy0}), f.toWorld({fx1, fy0}), f.toWorld({fx1, fy0 + P}), f.toWorld({fx0, fy0 + P})};
                            ensureCCW(lotP);
                        }
                        plan = mp;
                        parkingPoly = lotP;
                        bp.mallArms = arms;
                        bp.mallCourt = f.toWorld({cx, cy});
                        bp.faceDir = Vec3(-f.v.x, 0, -f.v.y);   // the main doors face the street
                        return true;
                    };
                    const bool fwd = rng.unit() < 0.5;
                    bool fit = false;
                    for (Real Ax = std::min(W * 0.5 - edgeM, Real(120)); !fit && Ax >= a + 40; Ax -= 10) {
                        if (spare >= 26 + 28) fit = layout(std::min(spare - 28, Real(44)), true, false, Ax);
                        if (!fit && spare >= 26) fit = layout(std::min(spare, Real(64)), false, false, Ax);
                        if (!fit) fit = layout(0, true, fwd, Ax) || layout(0, true, false, Ax) || layout(0, false, false, Ax);
                    }
                    planOk = planOk && fit;
                    if (std::getenv("RT_BIGBOX_DEBUG"))
                        std::printf("[mall] W %.0f D %.0f spare %.0f -> %s arms %u lot %zu\n", W, D, spare, planOk ? "wings" : "box",
                                    static_cast<unsigned>(bp.mallArms), parkingPoly.size());
                    if (planOk && dbg->bigBoxAt.size() < 4) dbg->bigBoxAt.push_back(bp.mallCourt);
                    // THE LOT BEHIND (Glenn: "the parking round it"): the block often runs on past the site's rectangle
                    // to a street behind -- with no wing back in the way, the biggest rectangle between the mall's
                    // back wall and that street, 26 m deep at least, is parking too, entered from the back
                    if (planOk && !(bp.mallArms & 0x4)) {
                        const Poly2 host = inset(lot.footprint, 3.6);
                        const Real yb = f.toFrame(bp.mallCourt).y + a + 1.0;
                        const Real cxm = f.toFrame(bp.mallCourt).x;
                        Real bestA = 0;
                        for (Real off = -60; off <= 60 && host.size() >= 3; off += 10)   // (slid along: a block's corners slant)
                        for (Real dp = 26; dp <= 90; dp += 2)
                            for (Real hw = 20; hw <= 130; hw += 5) {
                                const Real xc = cxm + off;
                                bool in = true;
                                for (Real gx : {-1.0, -0.5, 0.0, 0.5, 1.0})
                                    for (Real gy : {0.0, 0.5, 1.0})
                                        if (in && !pointInPolygon(host, f.toWorld({xc + gx * hw, yb + gy * dp}))) in = false;
                                if (!in) break;   // (wider only gets worse)
                                if (2 * hw * dp > bestA) {
                                    bestA = 2 * hw * dp;
                                    parkingBack = {f.toWorld({xc - hw, yb}), f.toWorld({xc + hw, yb}), f.toWorld({xc + hw, yb + dp}),
                                                   f.toWorld({xc - hw, yb + dp})};
                                }
                            }
                        if (bestA < 26 * 40) parkingBack.clear();
                        else ensureCCW(parkingBack);
                        if (std::getenv("RT_BIGBOX_DEBUG")) std::printf("[mall]   lot behind %.0f m2\n", bestA);
                    }
                    // the wings do not fit the site: the box mall instead (the block was big enough for a mall)
                    if (!planOk) { bp.bigBox = 5; bp.mallArms = 0; planOk = true; parkingPoly.clear(); parkingBack.clear(); }
                }
                if (bp.bigBox != 6) {
                const bool mallBox = bp.bigBox == 5;
                Real sd = std::clamp(D * rng.range(0.52, 0.62), Real(mallBox ? 36 : 30), Real(80));
                if (D - sd < 24) sd = D - 24;
                const Real sw = std::clamp(W * rng.range(mallBox ? 0.80 : 0.70, 0.85), Real(36), Real(mallBox ? 160 : 140));
                if (sd >= (mallBox ? 34 : 30) && W >= (mallBox ? 70 : 40)) {
                    const Real cx0 = (fx0 + fx1) * 0.5, P = D - sd;
                    Poly2 store{f.toWorld({cx0 - sw * 0.5, fy0 + P}), f.toWorld({cx0 + sw * 0.5, fy0 + P}),
                                f.toWorld({cx0 + sw * 0.5, fy1 - 0.5}), f.toWorld({cx0 - sw * 0.5, fy1 - 0.5})};
                    ensureCCW(store);
                    plan = store;
                    parkingPoly = {f.toWorld({fx0, fy0}), f.toWorld({fx1, fy0}), f.toWorld({fx1, fy0 + P - 0.5}),
                                   f.toWorld({fx0, fy0 + P - 0.5})};
                    ensureCCW(parkingPoly);
                    bp.faceDir = Vec3(-f.v.x, 0, -f.v.y);   // the doors face the lot and the street beyond it
                    if (dbg->bigBoxAt.size() < 4) dbg->bigBoxAt.push_back(centroid(store));
                } else {
                    planOk = false;   // too shallow for a store and its lot
                }
                }   // (bigBox 6 above)
            }
            // A STORE WITH NO LOT IS NOT A STORE: the box fallback filled the whole block with a parking-less box,
            // seated at the block's lowest corner (Glenn: "a massive sunken building"). A block whose site cannot take
            // the store and its parking stays open ground.
            if (rec.massing == BuildingRecipe::Massing::BigBox && parkingPoly.empty() && !(bp.bigBox == 6 && planOk)) continue;
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
                // The drum is sized off the OBB short side and centred on
                // b.site — on a TAPERED lot that circle runs past the sloping
                // edges, and the old test checked the ROADS ONLY, so the tower
                // could stand partly on the neighbour's parcel. SHRINK-TO-FIT,
                // as the house path above already does: retry smaller until
                // the whole rim sits on the lot and off the road.
                Vec2 cc = b.site;
                if (!pointInPolygon(plan, cc)) cc = centroid(plan);
                for (Real rad = shortSide * 0.5 - 1.6; rad > 6.0; rad *= 0.85) {
                    Poly2 circ;
                    for (int k = 0; k < 16; ++k) {
                        Real a2 = 2.0 * 3.14159265358979323846 * k / 16;
                        circ.push_back(cc + Vec2(std::cos(a2), std::sin(a2)) * rad);
                    }
                    bool clear = true;
                    for (const Vec2& v : circ)
                        if ((roads && !clearOfRoads(v)) ||
                            !pointInPolygon(plan, v)) { clear = false; break; }
                    if (clear) { plan = circ; break; }
                }
            }
            // ROWHOUSES (device: "town homes ... packed side by side"): the
            // lot becomes a terrace of narrow townhome UNITS sharing party
            // walls — each its own plan building (door, stoop, cladding,
            // sometimes its own gable) grown side by side with zero gaps.
            if (planOk && rec.massing == BuildingRecipe::Massing::RowStrip) {
                OBB2 sb = orientedBoundingBox(plan);
                const int la = sb.longAxis(), sa2 = 1 - la;
                Real len = 2 * sb.half[la];
                Real dep = std::min(2 * sb.half[sa2], Real(11.0));
                Vec2 u = sb.axis[la], v = sb.axis[sa2];
                // The terrace spans the plan's OBB, and a frontage-first lot
                // is a TRAPEZOID — so the row's ends ran past the tapered
                // edges onto the neighbour, because the old test checked the
                // ROADS ONLY and never the lot. This was the WIDEST spill of
                // the three (a whole terrace, not one mass). SHRINK-TO-FIT
                // like the house path: pull the row in until all four corners
                // sit on the lot and off the road — a tapered lot gets a
                // SHORTER row rather than one that overhangs.
                bool stripOk = false;
                for (int attempt = 0; attempt < 5 && dep > 6.5; ++attempt) {
                    bool fits = true;
                    for (int sx = -1; sx <= 1 && fits; sx += 2)
                        for (int sy = -1; sy <= 1; sy += 2) {
                            Vec2 corner = sb.center + u * (len * 0.5 * sx) +
                                          v * (dep * 0.5 * sy);
                            if ((roads && !clearOfRoads(corner)) ||
                                !pointInPolygon(plan, corner)) {
                                fits = false; break;
                            }
                        }
                    if (fits) { stripOk = true; break; }
                    len *= 0.88;
                    dep *= 0.94;
                }
                const int units = static_cast<int>(len / rng.range(5.6, 7.0));
                stripOk = stripOk && units >= 3 && dep > 6.5;
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
                        if (wantsDoorway(up4, upar)) upar.openDoorway = true;
                        BuildingMesh um = growPlanBuilding(up4, upar, stripBase);
                        collectUnit(b, up4, upar, stripBase, um);
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
                    pruneTreeSpotsIntoRoad(b.treeSpots);
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
                if (qq < 0.8 && plazaPoly.size() < 3 && parkingPoly.size() < 3)
                    bp.floors = std::max(1, static_cast<int>(bp.floors * qq));
            }

            BuildingMesh bm;
            BuildingMesh bmFlat;   // the LOD1 twin, same massing decisions (R2)
            // On terrain the building rises from its graded pad plane (plus the
            // plinth reveal); every walk-up entrance earns steps to the door —
            // porticos and bay-door fronts already bring their own.
            // The pad plane is the FRONT STREET's carve, probed 2 m beyond the
            // PARCEL's frontage edge — not the plan's. With a real front yard
            // the plan's edge stands metres inside the lot, its 2 m probe lands
            // on the block plane above the road, and every pad along the street
            // then stands proud of the deck (the poke gate: 108 -> 1660 once the
            // pads grew to their parcels). Lots without a parcel keep the plan.
            // ...and the street is the NEAREST ROAD where a road graph exists (bp.faceDir), the
            // parcel's recorded frontage only where none does (the lab: every block edge is a
            // street). A rim lot's frontage can face the open side of the city, and probing 2 m
            // out there read the hillside: a pad 4.8 m above the road beside it.
            {
                const Vec2 nearestRoad(bp.faceDir.x, bp.faceDir.z);
                const Vec2 probeDir = (roads && nearestRoad.length() > Real(1e-6)) ? nearestRoad
                                      : (lot.frontage.length() > Real(1e-6) ? lot.frontage : nearestRoad);
                b.groundY = padPlaneFor(lot.footprint.size() >= 3 ? lot.footprint : (planOk ? plan : site),
                                        probeDir);
                // A WHOLE-BLOCK site is a street on every side: at its front edge's grade the pad stood
                // above the streets falling away behind it, and the mesher's footprint dilation lifted
                // the terrain through their decks (the deck-poke gate: 126 -> 400, worst 4.2 m). Seat
                // it at the lowest ground on its boundary, below every street around it.
                if (lot.wholeBlock && p.ground && lot.footprint.size() >= 3) {
                    Real lowest = b.groundY;
                    for (std::size_t i = 0; i < lot.footprint.size(); ++i) {
                        const Vec2 a = lot.footprint[i], c = lot.footprint[(i + 1) % lot.footprint.size()];
                        lowest = std::min({lowest, p.ground(a.x, a.y), p.ground((a.x + c.x) * 0.5, (a.y + c.y) * 0.5)});
                    }
                    b.groundY = lowest;
                }
            }
            if (const char* at = std::getenv("RT_LOT_AT")) {
                double px = 0, pz = 0;
                const Vec2 q0(px, pz);
                bool near = false;
                if (std::sscanf(at, "%lf,%lf", &px, &pz) == 2) {
                    near = pointInPolygon(b.lot, Vec2(px, pz));
                    for (std::size_t i = 0; i < b.lot.size() && !near; ++i) {
                        const Vec2 a = b.lot[i], c = b.lot[(i + 1) % b.lot.size()];
                        const Vec2 ac = c - a; const Real l2 = ac.lengthSquared();
                        Real t = l2 > 1e-12 ? dot(Vec2(px, pz) - a, ac) / l2 : 0.0; t = std::max(Real(0), std::min(Real(1), t));
                        near = (Vec2(px, pz) - (a + ac * t)).length() < 3.0;
                    }
                }
                (void)q0;
                if (near)
                    std::printf("[lot-at]   %s faceDir (%.2f, %.2f) groundY %.2f (ground at the probe point %.2f) plan corners %zu\n", rec.name.c_str(),
                                bp.faceDir.x, bp.faceDir.z, b.groundY, p.ground ? p.ground(px, pz) : 0.0, (planOk ? plan : site).size());
            }
            b.baseY = p.ground ? b.groundY + plinth
                               : baseYFor(planOk ? plan : site);
            // PAVED URBAN LOT (ADR-0086; the owner's rule: a downtown building
            // stands on concrete at the sidewalk's height, not on grass). The
            // whole lot is paved at groundY + sidewalkRise — the sidewalk's
            // top by the road carve's own constants — and the building's base
            // IS that paving, so the threshold meets it flush. Houses on
            // yards, parks and plazas keep their own ground.
            const bool urbanTag = tag == DistrictTag::Financial || tag == DistrictTag::Commercial ||
                                  tag == DistrictTag::OldTown || tag == DistrictTag::Industrial;
            const bool paved = siteRectified && urbanTag && !ufShort && planOk &&
                               rec.massing != BuildingRecipe::Massing::Park &&
                               rec.massing != BuildingRecipe::Massing::Plaza &&
                               rec.massing != BuildingRecipe::Massing::RectYard;
            if (paved) {
                b.paveY = p.ground ? b.groundY + (p.sidewalkRise > 0 ? p.sidewalkRise : plinth) : b.baseY;
                b.baseY = b.paveY;
                b.pavedLot = lot.footprint;
                ensureCCW(b.pavedLot);
                // The plate reaches the SIDEWALK: the block inset leaves a strip of
                // grass between the lot line and the band's outer edge (1.3-2.3 m on
                // the lattice). Along every parcel edge that lies on the block
                // boundary and beside a road, push the plate's edge out to
                // (halfWidth + sidewalkWidth) from that road's centreline. The
                // parcel, pad and yards stay where they are: the strip's ground
                // is the road's own carve, and the plate's skirt covers the rise.
                if (roads && p.sidewalkWidth > 0 && bf.foot.size() >= 3) {
                    auto distToBlock = [&](const Vec2& q) {
                        Real best = 1e30;
                        for (std::size_t i = 0; i < bf.foot.size(); ++i) {
                            const Vec2 a = bf.foot[i], c = bf.foot[(i + 1) % bf.foot.size()];
                            const Vec2 ac = c - a; const Real l2 = ac.lengthSquared();
                            Real t = l2 > 1e-12 ? dot(q - a, ac) / l2 : 0.0; t = std::max(Real(0), std::min(Real(1), t));
                            best = std::min(best, (q - (a + ac * t)).length());
                        }
                        return best;
                    };
                    std::vector<Real> apron(b.pavedLot.size(), Real(0));
                    for (std::size_t i = 0; i < b.pavedLot.size(); ++i) {
                        const Vec2 a = b.pavedLot[i], c = b.pavedLot[(i + 1) % b.pavedLot.size()];
                        if (distToBlock(a) > 0.6 || distToBlock(c) > 0.6) continue;   // not a block edge
                        const Vec2 m = (a + c) * 0.5;
                        Real best = 1e30, bestHw = 0;
                        for (const RoadEdge& e : roads->edges) {
                            if (e.a < 0 || e.b < 0 || e.a >= static_cast<int>(roads->nodes.size()) ||
                                e.b >= static_cast<int>(roads->nodes.size())) continue;
                            const Vec2& ra = roads->nodes[e.a].pos;
                            const Vec2& rb = roads->nodes[e.b].pos;
                            const Vec2 ab = rb - ra; const Real len2 = ab.lengthSquared();
                            Real t = len2 > 1e-12 ? dot(m - ra, ab) / len2 : 0.0; t = std::max(Real(0), std::min(Real(1), t));
                            const Real d = (m - (ra + ab * t)).length();
                            if (d < best) { best = d; bestHw = e.width * 0.5; }
                        }
                        const Real bandEdge = bestHw + p.sidewalkWidth;
                        // Only an edge that really lies beside that road (within a few
                        // metres of its band): an alley-side edge finds a far street.
                        if (best < bandEdge || best > bandEdge + 4.0) continue;
                        const Real reach = std::max(Real(0), best - bandEdge - Real(0.05));
                        // ...and only where the ground it would cover is not ABOVE the plate. Uphill
                        // of a sloping street the sidewalk's grading stands higher than the lot's
                        // pad, and a plate stretched over it went under the grass (measured: 28 of
                        // the lattice metro's 35 buried plates were apron). There the lot line keeps
                        // its edge and the skirt meets the rising ground.
                        if (reach > Real(0.05) && p.ground) {
                            const Vec2 dd = c - a;
                            const Real dl = dd.length();
                            if (dl > Real(1e-6)) {
                                const Vec2 out(dd.y / dl, -dd.x / dl);   // CCW: right normal is outward
                                const Vec2 q = m + out * reach;
                                if (p.ground(q.x, q.y) > b.paveY + Real(0.1)) continue;
                            }
                        }
                        apron[i] = reach;
                    }
                    bool any = false;
                    for (Real v : apron) any = any || v > 0.05;
                    if (any) b.pavedLot = offsetPolygonEdges(b.pavedLot, apron);
                }
                // ...and on a lane-built city, pulled IN to where the pad is flat. A lane-built
                // pad is inset by its feather and clipped to the block (clipPadsToBlocks), so
                // the last metres of a plate that reached the lot line stood on the ramp from
                // the pad up to the sidewalk — and uphill that ground climbed over the plaza
                // (metro_lanes: 173 plates under the grass once blocks began right behind the
                // sidewalk). The plate stops where the flat pad stops; the feather between it
                // and the sidewalk is graded ground.
                if (p.padFeatherInside > 0 && bf.foot.size() >= 3 && b.pavedLot.size() >= 3) {
                    ensureCCW(b.pavedLot);
                    auto blockDist = [&](const Vec2& q) {
                        Real best = Real(1e30);
                        for (std::size_t i = 0; i < bf.foot.size(); ++i) {
                            const Vec2 a = bf.foot[i], c = bf.foot[(i + 1) % bf.foot.size()];
                            const Vec2 ac = c - a; const Real l2 = ac.lengthSquared();
                            Real t = l2 > 1e-12 ? dot(q - a, ac) / l2 : 0.0; t = std::max(Real(0), std::min(Real(1), t));
                            best = std::min(best, (q - (a + ac * t)).length());
                        }
                        return best;
                    };
                    // Corners on the block line; the middle may sag off a CURVED block edge by the
                    // chord's sagitta (a lane city's blocks follow curving streets).
                    auto onBlockEdge = [&](const Vec2& q, Real tol) { return blockDist(q) < tol; };
                    std::vector<Real> pull(b.pavedLot.size(), Real(0));
                    bool anyPull = false;
                    for (std::size_t i = 0; i < b.pavedLot.size(); ++i) {
                        const Vec2 a = b.pavedLot[i], c = b.pavedLot[(i + 1) % b.pavedLot.size()];
                        if (onBlockEdge(a, Real(0.6)) && onBlockEdge(c, Real(0.6)) && onBlockEdge((a + c) * 0.5, Real(2.5))) {
                            pull[i] = -p.padFeatherInside;
                            anyPull = true;
                        }
                    }
                    if (anyPull) {
                        Poly2 pulled = offsetPolygonEdges(b.pavedLot, pull);
                        if (pulled.size() >= 3 && area(pulled) > Real(0.3) * area(b.pavedLot)) b.pavedLot = std::move(pulled);
                    }
                }
                if (courtNotch.size() >= 3) b.open.push_back({courtNotch, OpenKind::Courtyard});
                if (parkingPoly.size() >= 3) b.open.push_back({parkingPoly, OpenKind::Parking});
                if (parkingBack.size() >= 3) b.open.push_back({parkingBack, OpenKind::Parking});
                if (plazaPoly.size() >= 3) {
                    b.open.push_back({plazaPoly, OpenKind::Plaza});
                    if (std::getenv("RT_SITE_DEBUG"))
                        std::printf("[plaza-at] %s at (%.1f, %.1f) plaza %.0f m2 floors %d\n", rec.name.c_str(),
                                    b.site.x, b.site.y, area(plazaPoly), bp.floors);
                }
                ++pavedLots;
            }
            // Entrance steps meet the REAL ground (Glenn's foundation-block
            // design): sample the ground at the entrance-edge middle, 2 m
            // out, and hand the grammar the drop from the storey base down
            // to it — the stoop grows the extra steps. F3's min-front-edge
            // anchoring keeps this small by construction; the clamp stops a
            // pathological sample from growing a staircase tower.
            if (p.ground && !paved) {
                const Poly2& epl = planOk ? plan : site;
                const Vec2 f2(bp.faceDir.x, bp.faceDir.z);
                Real eg = b.groundY;
                if (f2.length() > Real(1e-6) && epl.size() >= 3) {
                    const Vec2 fn = normalize(f2);
                    Real bestDot = Real(0.2);
                    std::size_t bi = epl.size();
                    for (std::size_t ei = 0; ei < epl.size(); ++ei) {
                        const Vec2 e = epl[(ei + 1) % epl.size()] - epl[ei];
                        const Real l = e.length();
                        if (l < Real(1e-6)) continue;
                        const Real d = dot(Vec2(e.y / l, -e.x / l), fn);
                        if (d > bestDot) { bestDot = d; bi = ei; }
                    }
                    if (bi < epl.size()) {
                        const Vec2 mid =
                            (epl[bi] + epl[(bi + 1) % epl.size()]) * Real(0.5) +
                            fn * Real(2.0);
                        eg = p.ground(mid.x, mid.y);
                    }
                }
                // ...and never short of its own PAD (#94): the stoop stands on the building's graded pad, whose
                // plane is groundY; the 2 m sample was taken before the pad exists, and where it came out above
                // the storey base the drop clamped to 0 and the stoop hung a plinth (0.45 m) over the pad --
                // island_8_nature's cluster of stoops floating at exactly that height.
                bp.entranceDropBelow =
                    std::clamp(std::max(b.baseY - eg, b.baseY - b.groundY), Real(0), Real(2.4));
            }
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
                sculptPlaza(b, plan, meshGround,
                            mix(pp.seed, static_cast<uint32_t>(li) * 31u + 17u),
                            outParts, roads, outGrade,
                            p.groundMeshCell > 0.5 ? p.groundMeshCell : Real(3.0));
                LOG_INFO << "[plaza] at (" << b.site.x << ", " << b.site.y
                         << ") area " << static_cast<int>(area(plan)) << " m2";
                pruneTreeSpotsIntoRoad(b.treeSpots);
                out.push_back(std::move(b));
                continue;
            }
            if (paved) bp.entranceSteps = false;   // a downtown door is flush with the paving
            if (p.ground && !paved && !bp.entranceSteps && bp.portico == 0 &&
                bp.groundBays == 0 && !bp.porch && bp.floors > 0)
                bp.entranceSteps = true;
            // PODIUM TOWER massing (density round, "more varied building
            // shapes"): a few-storey podium takes the WHOLE lot plan (street
            // wall, ground retail), and a slender curtain-wall tower rises
            // from its roof — the modern downtown block. Falls through to a
            // single mass when the plan can't stand a tower.
            // ONE building, not two (Glenn's second walk, 2026-09-14: "very
            // tall but only 4 floors on the elevator"): the podium and the
            // tower used to be two grows and two records, so the lobby's core
            // was the podium's — four floors — and the tower above had no
            // way in. The mass stack already knows this shape (the street
            // wall's shaft tier): the podium is the base, the tower the shaft
            // from podiumFloors up, and one record carries the whole height —
            // one core seated in the shaft and fitting every tier, one
            // elevator bank to the top, one interior streamed per storey on
            // the tier's plan. The tower's size check stays the guard: a plan
            // that cannot stand a tower falls through to a single mass.
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
                    // The street wall's envelope with no first setback and no
                    // steps: base = the podium, shaft = the tower, sized to
                    // the same area the two-mass path drew (the mass stack
                    // shrinks it until it sits inside the podium).
                    bp.envelope = BuildingParams::Envelope::StreetWallSetback;
                    bp.baseFloors = rec.podiumFloors;
                    bp.setback1 = 0;
                    bp.stepFloors = 0;
                    bp.stepDepth = 0;
                    bp.towerFloor = rec.podiumFloors;
                    bp.towerFrac = std::min(Real(0.9), (4.0 * thw * thd) / std::max(Real(1), area(plan)));
                    if (bp.floors - rec.podiumFloors > 18) {
                        bp.setbackFloors = 6;   // tall shafts keep tiers
                        bp.setbackEvery = 1.5;
                    } else {
                        bp.setbackFloors = 0;
                    }
                }
            }
            if (planOk && lot.partyCount > 0 && cand.landmark < 0 && !lot.wholeBlock) {
                // PARTY WALLS where the final plan still stands on a shared side line (a clearance inset or a
                // massing rescue pulls it off, and then the wall keeps its windows), and the attached building's
                // way out the back: a service door and, on a walk-up, a fire escape.
                BuildingParams tp = bp;
                tp.partyWalls = 0;
                for (int k = 0; k < lot.partyCount; ++k) {
                    BuildingParams one = bp;
                    one.partyWalls = 1;
                    one.partyN[0] = lot.partyN[k];
                    one.partyAt[0] = lot.partyAt[k];
                    Poly2 cp = plan;
                    ensureCCW(cp);
                    bool on = false;
                    for (std::size_t e = 0; e < cp.size() && !on; ++e) on = partyEdge(cp, one, e);
                    if (!on) continue;
                    tp.partyN[tp.partyWalls] = lot.partyN[k];
                    tp.partyAt[tp.partyWalls] = lot.partyAt[k];
                    ++tp.partyWalls;
                }
                if (tp.partyWalls > 0) {
                    bp = tp;
                    bp.backDoor = bp.walkableGround && bp.groundBays <= 0;
                    bp.fireEscape = !bp.curtainWall && !bp.solidFacade && bp.floors >= 2 && bp.floors <= 6 &&
                                    bp.envelope == BuildingParams::Envelope::None && bp.setbackFloors == 0;
                    ++dbg->attachedBuilt;
                    if (bp.fireEscape && dbg->attachedAt.size() < 4) dbg->attachedAt.push_back(centroid(plan));
                }
            }
            if (planOk) {
                if (wantsDoorway(plan, bp)) bp.openDoorway = true;
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
            collectUnit(b, b.plan, bp, b.baseY, bm);
            mergeParts(outParts, bm);
            mergeParts(outFlatParts, bmFlat);
            b.height = bm.height > 0 ? bm.height : 8.0;
            if (!paved) emitFoundation(b.plan, b.groundY, b.baseY);   // the plate is the foundation of a paved lot
            else sculptPaving(b.pavedLot, b.paveY, meshGround, outParts, outFlatParts);
            if (paved && plazaPoly.size() >= 3)
                sculptForecourt(b, plazaPoly, siteFrameOf, b.paveY,
                                mix(pp.seed, static_cast<uint32_t>(li) * 37u + 23u), outParts);
            if (paved && parkingPoly.size() >= 3)
                sculptParking(b, parkingPoly, siteFrameOf, b.paveY, bp,
                              mix(pp.seed, static_cast<uint32_t>(li) * 41u + 29u), outParts);
            if (paved && parkingBack.size() >= 3) {   // (its frame turned round: the back street is its street)
                SiteFrame back = siteFrameOf;
                back.u = back.u * -1.0;
                back.v = back.v * -1.0;
                sculptParking(b, parkingBack, back, b.paveY, bp, mix(pp.seed, static_cast<uint32_t>(li) * 43u + 31u), outParts);
            }
            // CAFÉ TERRACES (the furniture library, M2): in front of every café and bakery on the storefront, where
            // the paving reaches far enough (2.8 m), bistro sets along the glass -- a table, a chair either side
            // facing across it -- clear of the shop's door. Sat on like any chair.
            if (paved && planOk && b.pavedLot.size() >= 3) {
                for (const ShopFront& sf : shopFrontsOf(plan, bp)) {
                    { const TradeInfo* tr = tradeById(sf.trade); if (sf.indoor || !tr || !tr->terrace) continue; }   // cafe, bakery, restaurant (trades.h); a mall's are indoors
                    const Vec2 d0 = sf.b - sf.a;
                    const Real L = d0.length();
                    if (L < 2.4) continue;
                    const Vec2 d = d0 * (1.0 / L);
                    auto onPaving = [&](const Vec2& q) { return pointInPolygon(b.pavedLot, q) && clearOfRoads(q); };
                    Real room = 0;
                    for (Real t = 0.3; t <= 8.0; t += 0.25) {
                        if (!onPaving((sf.a + sf.b) * 0.5 + sf.n * t)) break;
                        room = t;
                    }
                    if (room < 2.8) continue;
                    const Real out = std::min(room - 1.1, Real(1.9));   // the tables' line, out from the glass
                    for (Real x = 1.1; x + 1.1 <= L + 1e-6; x += 2.3) {
                        const Vec2 T = sf.a + d * x + sf.n * out;
                        if (std::fabs(dot(T - sf.door, d)) < 1.3) continue;   // the door stays clear
                        if (!onPaving(T - d * 0.85) || !onPaving(T + d * 0.85)) continue;
                        b.furniture.push_back(outdoorPiece(Piece::BistroTable, posHash(T), T - sf.n * 0.35, sf.n, b.paveY, false));
                        b.furniture.push_back(outdoorPiece(Piece::BistroChair, posHash(T), T - d * 0.80, d, b.paveY, false));
                        b.furniture.push_back(outdoorPiece(Piece::BistroChair, posHash(T), T + d * 0.80, d * -1.0, b.paveY, false));
                    }
                }
            }
            // SHOPFRONTS BY TRADE (the city's next ten #4): what stands on the pavement in front of the glass says what
            // the shop is -- a grocer's fruit and vegetable stands either side of the door, a boutique's or a bookshop's
            // A-frame board (and the bookshop's cart of books), a restaurant's planters and menu stand at its door, a
            // bar's high tables and stools along its window, a club's rope line with brass posts. On the paving, clear of
            // the door, the terrace and the road; absolute at the paving's height like the terraces.
            if (paved && planOk && b.pavedLot.size() >= 3 && outParts) {
                BuildingMesh kit;
                const Vec3 up(0, 1, 0);
                auto onPaving = [&](const Vec2& q) { return pointInPolygon(b.pavedLot, q) && clearOfRoads(q); };
                auto boxAt = [&](const Vec2& c, const Vec2& d, const Vec2& n, Real w, Real dep, Real y0, Real h, PartId part, const Vec3& col) {
                    const Vec3 u3(d.x, 0, d.y), n3(n.x, 0, n.y);
                    const Vec3 o = Vec3(c.x, b.paveY + y0, c.y) - u3 * (w * 0.5) - n3 * (dep * 0.5);
                    emitBox(kit, Scope{o, {u3, up, n3}, Vec3(w, h, dep)}, part, col);
                };
                auto softAt = [&](const Vec2& c, const Vec2& d, const Vec2& n, Real w, Real dep, Real y0, Real h, const Vec3& col) {
                    const Vec3 u3(d.x, 0, d.y), n3(n.x, 0, n.y);
                    const Vec3 o = Vec3(c.x, b.paveY + y0, c.y) - u3 * (w * 0.5) - n3 * (dep * 0.5);
                    emitSoftBox(kit, Scope{o, {u3, up, n3}, Vec3(w, h, dep)}, PartId::Foliage, col, 0u);
                };
                // an A-frame board: two leaning panels, its face toward the passers-by along the street
                auto aFrame = [&](const Vec2& c, const Vec2& d, const Vec3& face) {
                    for (Real s : {Real(-1), Real(1)}) {
                        const Vec3 base(c.x + d.x * s * 0.22, b.paveY, c.y + d.y * s * 0.22), top(c.x, b.paveY + 0.95, c.y);
                        Vec3 ax = normalize(top - base);
                        const Vec3 side(-d.y, 0, d.x);
                        const Vec3 nn = normalize(cross(side, ax));
                        emitBox(kit, Scope{base - side * 0.3, {side, ax, nn}, Vec3(0.6, (top - base).length(), 0.03)}, PartId::Wood, Vec3(0.20, 0.14, 0.09));
                        emitBox(kit, Scope{base - side * 0.25 + ax * 0.1 - nn * 0.005, {side, ax, nn}, Vec3(0.5, (top - base).length() - 0.2, 0.01)},
                                PartId::Trim, face);
                    }
                };
                for (const ShopFront& sf : shopFrontsOf(plan, bp)) {
                    if (sf.indoor) continue;
                    const Vec2 d0 = sf.b - sf.a;
                    const Real L = d0.length();
                    if (L < 3.0) continue;
                    const Vec2 d = d0 * (1.0 / L);
                    const Real dx = dot(sf.door - sf.a, d);   // the door, along the front
                    Real room = 0;
                    for (Real tt = 0.3; tt <= 6.0; tt += 0.25) { if (!onPaving((sf.a + sf.b) * 0.5 + sf.n * tt)) break; room = tt; }
                    if (room < 1.6) continue;
                    const uint32_t hh = posHash(sf.door);
                    auto at = [&](Real x, Real out) { return sf.a + d * x + sf.n * out; };
                    auto clear = [&](const Vec2& q, Real r) { return onPaving(q + d * r) && onPaving(q - d * r) && onPaving(q + sf.n * r) && onPaving(q - sf.n * r); };
                    if (std::getenv("RT_FRONT_DEBUG"))
                        std::printf("[front] trade %d door %.1f %.1f n %.2f %.2f room %.1f\n", sf.trade, sf.door.x, sf.door.y, sf.n.x, sf.n.y, room);
                    switch (sf.trade) {
                        case 1: {   // GROCERY: tiered produce stands, two each side of the door
                            const Vec3 produce[6] = {{0.75, 0.12, 0.08}, {0.95, 0.60, 0.10}, {0.40, 0.65, 0.15}, {0.85, 0.80, 0.20}, {0.55, 0.15, 0.35}, {0.25, 0.45, 0.12}};
                            int k = 0;
                            for (Real side : {Real(-1), Real(1)})
                                for (int s = 0; s < 2; ++s) {
                                    const Real x = dx + side * (1.6 + s * 1.5);
                                    if (x < 0.8 || x > L - 0.8) continue;
                                    const Vec2 c = at(x, 0.65);
                                    if (!clear(c, 0.6)) continue;
                                    boxAt(c, d, sf.n, 1.3, 0.9, 0, 0.55, PartId::Wood, Vec3(0.45, 0.33, 0.20));        // the stand
                                    for (int tier = 0; tier < 2; ++tier) {
                                        const Vec2 tc = c + sf.n * (tier == 0 ? 0.18 : -0.18);
                                        boxAt(tc, d, sf.n, 1.2, 0.4, 0.55 + tier * 0.22, 0.12, PartId::Wood, Vec3(0.55, 0.42, 0.26));   // a crate
                                        softAt(tc, d, sf.n, 1.1, 0.34, 0.66 + tier * 0.22, 0.10, produce[(hh + k++) % 6]);
                                    }
                                }
                            break;
                        }
                        case 2: case 3: case 4: {   // BOUTIQUE / BOOKSHOP / ELECTRONICS: a board by the door
                            const Real x = dx + ((hh & 1) ? 1.5 : -1.5);
                            const Vec2 c = at(std::clamp(x, Real(0.6), L - 0.6), std::min(room - 0.5, Real(1.4)));
                            if (clear(c, 0.4)) aFrame(c, d, sf.trade == 2 ? Vec3(0.92, 0.88, 0.84) : Vec3(0.06, 0.08, 0.07));
                            if (sf.trade == 3) {   // ...and a cart of books on the other side
                                const Vec2 cc = at(std::clamp(dx + ((hh & 1) ? -1.7 : 1.7), Real(0.8), L - 0.8), 0.55);
                                if (clear(cc, 0.5)) {
                                    boxAt(cc, d, sf.n, 1.2, 0.5, 0.0, 0.75, PartId::Wood, Vec3(0.35, 0.22, 0.12));
                                    for (int r = 0; r < 8; ++r)
                                        boxAt(cc + d * (-0.52 + r * 0.15), d, sf.n, 0.12, 0.38, 0.75, 0.18 + (r % 3) * 0.03, PartId::Trim,
                                              Vec3(0.2 + 0.1 * (r % 4), 0.15 + 0.08 * ((r + 1) % 3), 0.25 + 0.1 * ((r + 2) % 3)));
                                }
                            }
                            break;
                        }
                        case 13: {   // RESTAURANT: planters flanking the door, a menu stand
                            for (Real side : {Real(-1), Real(1)}) {
                                const Vec2 c = at(std::clamp(dx + side * 1.1, Real(0.5), L - 0.5), 0.45);
                                if (!clear(c, 0.35)) continue;
                                boxAt(c, d, sf.n, 0.6, 0.6, 0, 0.55, PartId::Concrete, Vec3(0.30, 0.30, 0.31));
                                softAt(c, d, sf.n, 0.5, 0.5, 0.55, 0.65, Vec3(0.18, 0.34, 0.16));
                            }
                            const Vec2 m = at(std::clamp(dx + 1.9, Real(0.5), L - 0.5), std::min(room - 0.4, Real(1.0)));
                            if (clear(m, 0.3)) {
                                boxAt(m, d, sf.n, 0.05, 0.05, 0, 1.2, PartId::Metal, Vec3(0.12, 0.12, 0.12));
                                boxAt(m + sf.n * 0.02, d, sf.n, 0.42, 0.04, 1.0, 0.55, PartId::Trim, Vec3(0.08, 0.08, 0.08));
                            }
                            break;
                        }
                        case 14: {   // BAR: high tables and stools along the window, a chalk board
                            for (Real x = 1.2; x + 1.0 <= L; x += 2.6) {
                                if (std::fabs(x - dx) < 1.6) continue;
                                const Vec2 T = at(x, 0.7);
                                if (!clear(T, 0.6)) continue;
                                b.furniture.push_back(outdoorPiece(Piece::HighTable, posHash(T), T - sf.n * 0.3, sf.n, b.paveY, false));
                                b.furniture.push_back(outdoorPiece(Piece::BarStool, posHash(T) ^ 1u, T - d * 0.6, d, b.paveY, false));
                                b.furniture.push_back(outdoorPiece(Piece::BarStool, posHash(T) ^ 2u, T + d * 0.6, d * -1.0, b.paveY, false));
                            }
                            const Vec2 c = at(std::clamp(dx + 1.2, Real(0.6), L - 0.6), std::min(room - 0.5, Real(1.6)));
                            if (clear(c, 0.4)) aFrame(c, d, Vec3(0.05, 0.06, 0.05));
                            break;
                        }
                        case 15: {   // CLUB: a rope line from the door along the front, brass posts every 1.4 m
                            const Real dir = (hh & 1) ? 1.0 : -1.0;
                            std::vector<Vec2> posts;
                            for (int k = 0; k < 6; ++k) {
                                const Vec2 q = at(dx + dir * (0.9 + k * 1.4), 1.0);
                                const Real along = dot(q - sf.a, d);
                                if (along < 0.4 || along > L - 0.4 || !clear(q, 0.25)) break;
                                posts.push_back(q);
                            }
                            for (std::size_t k = 0; k < posts.size(); ++k) {
                                boxAt(posts[k], d, sf.n, 0.3, 0.3, 0, 0.04, PartId::Metal, Vec3(0.75, 0.58, 0.22));
                                boxAt(posts[k], d, sf.n, 0.06, 0.06, 0.04, 0.95, PartId::Metal, Vec3(0.80, 0.62, 0.25));
                                if (k + 1 < posts.size()) {   // the velvet rope, sagging a little
                                    const Vec2 mid = (posts[k] + posts[k + 1]) * 0.5;
                                    boxAt(mid, d, sf.n, 1.35, 0.05, 0.80, 0.05, PartId::Trim, Vec3(0.45, 0.04, 0.08));
                                }
                            }
                            if (posts.size() >= 2) {   // the queue's line, for the host: where the night's crowd waits to get in
                                LotBuilding::Area ar;
                                ar.kind = "club_queue";
                                ar.center = (posts.front() + posts.back()) * 0.5 - sf.n * 0.5;
                                ar.axis = d * dir;
                                ar.halfL = (posts.back() - posts.front()).length() * 0.5;
                                ar.halfW = 0.4;
                                b.areas.push_back(ar);
                            }
                            break;
                        }
                        default: break;
                    }
                }
                appendKit(kit, outParts);
            }
            // A yarded house earns its LANDSCAPING: front walk to the street,
            // a hedge along the front lot line, back-yard tree spots.
            if (yardApplied)
                sculptYard(b, lot.footprint, plan,
                           Vec2(bp.faceDir.x, bp.faceDir.z),
                           mix(pp.seed, static_cast<uint32_t>(li) * 29u + 11u),
                           outParts);
            else
                sculptDoorWalks(b, roads, p.sidewalkWidth, &bf.foot, outParts);
            pruneTreeSpotsIntoRoad(b.treeSpots);
            out.push_back(std::move(b));
        }
    }
    if (underFwCount > 0)
        LOG_INFO << "[citylots] under-freeway lots: " << underFwCount
                 << " re-zoned to fit beneath the deck (open space / parking / "
                    "utility / short building)";
    LOG_INFO << "[citylots] site plans: " << sitesRectified
             << " lots rectified to a frontage-aligned rectangle; kept their lot shape: "
             << sitesNoRect << " (no rectangle of minShort in the lot), "
             << sitesRectUnfit << " (rectangle found but a corner or edge was on a road)"
             << "; " << pavedLots << " urban lots paved at the sidewalk datum";
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
        for (const Vec2& a : dbg->bigBoxAt)
            LOG_INFO << "[citylots] big-box store -> teleport " << static_cast<int>(a.x) << " " << static_cast<int>(a.y);
        for (const Vec2& a : dbg->mallAt)
            LOG_INFO << "[citylots] indoor mall -> teleport " << static_cast<int>(a.x) << " " << static_cast<int>(a.y);
        for (const Vec2& a : dbg->paseoAt)
            LOG_INFO << "[citylots] open-air mall (paseo) -> teleport " << static_cast<int>(a.x) << " " << static_cast<int>(a.y);
        for (std::size_t i = 0; i < dbg->campusAt.size(); ++i)
            LOG_INFO << "[citylots] university " << (i < dbg->campusWhat.size() ? dbg->campusWhat[i] : std::string("?")) << " -> teleport "
                     << static_cast<int>(dbg->campusAt[i].x) << " " << static_cast<int>(dbg->campusAt[i].y);
        for (const Vec2& a : dbg->attachedAt)
            LOG_INFO << "[citylots] attached walk-up with a fire escape -> teleport " << static_cast<int>(a.x) << " "
                     << static_cast<int>(a.y);
        LOG_INFO << "[citylots] " << dbg->blocks.size() << " blocks -> "
                 << dbg->lots.size() << " lots (" << dbg->wholeBlocks << " whole-block landmark sites), " << nBuilt << " built ("
                 << dbg->attachedBuilt << " attached of " << dbg->attachedSites << " sites run to a party line, "
                 << dbg->bigBoxBlocks << " big-box blocks, " << dbg->nightlifeBuildings << " nightlife buildings, " << dbg->towerVenues << " towers with stores or sky dining), "
                 << nGreen << " green, " << nCourt << " courts | open lots: "
                 << dbg->openCarParks << " car parks, " << dbg->openPlaygrounds << " playgrounds, "
                 << dbg->openGardens << " community gardens, " << dbg->openParks << " pocket parks | COVER "
                 << static_cast<int>(builtArea) << " m2 of "
                 << static_cast<int>(blockArea) << " m2 buildable ("
                 << (static_cast<int>(coverPct * 10) / 10.0) << "%) | rej: chance "
                 << dbg->rejChance << ", sliver " << dbg->rejSliver
                 << ", aspect " << dbg->rejAspect << ", fill " << dbg->rejFill
                 << ", plan " << dbg->rejPlan << ", clear " << dbg->rejClear
                 << " | tree spots pruned from carriageways " << treeSpotsPruned
                 << " | pads still covering road " << padsStillInRoad
                 << " (ON THE CARRIAGEWAY " << padsOnCarriageway << ")"
                 << " of " << padsChecked
                 << ", box " << dbg->rejBox << ", frontage "
                 << dbg->rejFrontage << ", relief " << dbg->rejRelief
                 << " | alleys " << dbg->alleys.size()
                 << " | " << blocksAllCarriageway
                 << " blocks all carriageway"
                 << " | " << dbg->bisectedBlocks
                 << " blocks BLIND-BISECTED (no street reference)"
                 << " | lots dropped for reaching into a lane " << lotsInCarriageway;
    for (std::size_t k = 0; k < rejectedPadAt.size(); ++k)
        LOG_INFO << "[citylots] road-locked lot " << k << " REMOVED at ("
                 << rejectedPadAt[k].x << ", " << rejectedPadAt[k].y
                 << ") -- this ground is street again";
    LOG_INFO << "[citylots] frontage walk: placed " << dbg->pPlaced
             << " lots; freeway/ramp edges (no frontage) " << dbg->pNotStreet
             << "; rejected edgeShort " << dbg->pEdgeShort
             << ", shallow " << dbg->pShallow << ", mitered " << dbg->pMitered
             << ", overlap " << dbg->pOverlap << ", escaped " << dbg->pEscaped
             << ", tiny " << dbg->pTiny << ", thin " << dbg->pThin
             << " | backstop clipped " << dbg->pClips << ", pairs STILL overlapping "
             << dbg->pLeftOverlapping << " (" << dbg->pSameEdge << " from ONE edge, " << dbg->pAtInsert << " already overlapping AT INSERT, " << dbg->pConcave << " NON-CONVEX, " << dbg->pClipFailed << " CLIPS THAT DID NOT SEPARATE)";
    }
    // The SKYLINE lines: storeys, towers, footprint shape and open space —
    // the same census the loader prints on a warm load (which grows nothing).
    {
        const SkylineCensus sc = skylineCensus(out);
        LOG_INFO << sc.line();
        LOG_INFO << sc.districtsLine();
    }
    return out;
}

int buildingStoreys(const LotBuilding& lot) {
    if (lot.type == "park" || lot.type == "green" || lot.units.empty()) return 0;
    Real base = std::numeric_limits<Real>::infinity();
    for (const BuildingUnit& u : lot.units) base = std::min(base, u.baseY);
    int onBase = 0, stacked = 0;
    for (const BuildingUnit& u : lot.units) {
        if (u.baseY <= base + 0.5) onBase = std::max(onBase, u.params.floors + 1);
        else stacked += u.params.floors + 1;
    }
    return onBase + stacked;
}

namespace {
// Count the plan's corners and how many are oblique (neither a right angle
// nor straight, within `tol` radians). Duplicate samples are skipped.
void classifyCorners(const Poly2& plan, Real tol, int& corners, int& oblique) {
    const std::size_t n = plan.size();
    if (n < 3) return;
    constexpr Real kRight = 1.5707963267948966;
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = plan[(i + n - 1) % n], b = plan[i], c = plan[(i + 1) % n];
        if ((b - a).length() < 1e-6 || (c - b).length() < 1e-6) continue;
        const Vec2 d0 = normalize(b - a), d1 = normalize(c - b);
        const Real turn = std::fabs(std::atan2(cross(d0, d1), dot(d0, d1)));
        if (turn <= tol) continue;               // straight: a sample, not a corner
        ++corners;
        if (std::fabs(turn - kRight) > tol) ++oblique;
    }
}
std::string fmt1(double v) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.1f", v);
    return buf;
}
}  // namespace

SkylineCensus skylineCensus(const std::vector<LotBuilding>& lots, Real rightTolDeg) {
    SkylineCensus c;
    const Real tol = rightTolDeg * 3.14159265358979323846 / 180.0;
    std::map<std::string, double> open;
    std::map<std::string, SkylineCensus::District> districts;
    for (const LotBuilding& lb : lots) {
        if (lb.type == "green") {
            ++c.greens;
            open["green"] += area(lb.pad);
            continue;
        }
        if (lb.type == "park") {
            ++c.parks;
            open[lb.recipe.empty() ? "park" : lb.recipe] += area(lb.pad);
            continue;
        }
        if (lb.recipe == "plaza") {   // a podium, not a building — open space
            open["plaza"] += area(lb.plan);
            continue;
        }
        if (lb.recipe == "car_park") {   // an open lot's car park (sculptCarPark)
            open["car_park"] += area(lb.pad);
            continue;
        }
        if (lb.units.empty()) continue;
        ++c.built;
        for (const OpenSpace& o : lb.open) open[openKindName(o.kind)] += area(o.poly);
        if (!lb.pavedLot.empty()) ++c.paved;
        const int storeys = buildingStoreys(lb);
        int bin = 0;
        for (int i = 0; i < SkylineCensus::kBins; ++i)
            if (storeys >= SkylineCensus::kBinLo[i]) bin = i;
        ++c.storeyBins[bin];
        if (storeys > 20) ++c.over20;
        if (storeys > 40) ++c.over40;
        if (storeys > 60) ++c.over60;
        if (storeys > c.tallestStoreys ||
            (storeys == c.tallestStoreys && lb.height > c.tallestHeight)) {
            c.tallestStoreys = storeys;
            c.tallestHeight = lb.height;
            c.tallestAt = lb.site;
            c.tallestRecipe = lb.recipe;
            c.tallestDistrict = lb.district;
        }
        int corners = 0, oblique = 0;
        classifyCorners(lb.plan, tol, corners, oblique);
        c.corners += corners;
        c.obliqueCorners += oblique;
        if (oblique == 0 && corners > 0) ++c.rectilinear;
        const double a = area(lb.plan);
        c.builtArea += a;
        SkylineCensus::District& d = districts[lb.district];
        d.name = lb.district;
        ++d.built;
        d.builtArea += a;
        d.maxStoreys = std::max(d.maxStoreys, storeys);
        if (storeys > 20) ++d.over20;
    }
    for (const auto& kv : open) c.openByKind.push_back(kv);
    for (const auto& kv : districts) c.districts.push_back(kv.second);
    return c;
}

std::string SkylineCensus::line() const {
    std::string s = "[skyline] " + std::to_string(built) + " buildings (paved " + std::to_string(paved) +
                    ", parks " + std::to_string(parks) + ", greens " + std::to_string(greens) + ") | storeys";
    static const char* kBinName[kBins] = {"1-3", "4-8", "9-20", "21-40", "41-60", "61+"};
    for (int i = 0; i < kBins; ++i)
        s += std::string(i ? ", " : " ") + kBinName[i] + ": " + std::to_string(storeyBins[i]);
    s += " | towers >20: " + std::to_string(over20) + ", >40: " + std::to_string(over40) +
         ", >60: " + std::to_string(over60);
    s += " | tallest " + std::to_string(tallestStoreys) + " storeys " +
         std::to_string(static_cast<int>(tallestHeight)) + " m " + tallestRecipe + " (" +
         tallestDistrict + ") at (" + std::to_string(static_cast<int>(tallestAt.x)) + ", " +
         std::to_string(static_cast<int>(tallestAt.y)) + ")";
    const double rectPct = built > 0 ? 100.0 * rectilinear / built : 0.0;
    const double oblPct = corners > 0 ? 100.0 * obliqueCorners / corners : 0.0;
    s += " | rectilinear " + fmt1(rectPct) + "% of buildings, oblique corners " + fmt1(oblPct) +
         "% of " + std::to_string(corners);
    s += " | built " + std::to_string(static_cast<long long>(builtArea)) + " m2 | open:";
    if (openByKind.empty()) s += " none";
    for (std::size_t i = 0; i < openByKind.size(); ++i)
        s += std::string(i ? ", " : " ") + openByKind[i].first + " " +
             std::to_string(static_cast<long long>(openByKind[i].second)) + " m2";
    return s;
}

std::string SkylineCensus::districtsLine() const {
    std::string s = "[skyline] districts:";
    if (districts.empty()) s += " none";
    for (std::size_t i = 0; i < districts.size(); ++i) {
        const District& d = districts[i];
        s += std::string(i ? "; " : " ") + (d.name.empty() ? "(untagged)" : d.name) + " " +
             std::to_string(d.built) + " built, " +
             std::to_string(static_cast<long long>(d.builtArea)) + " m2, max " +
             std::to_string(d.maxStoreys) + " storeys, >20: " + std::to_string(d.over20);
    }
    return s;
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
                                    bool wantFlatParts,
                                    bool wantParts) {
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
    r.lots = growLotBuildings(blocks, lp, &r.plan, wantParts ? &r.parts : nullptr, &rgSampled,
                              roadClearance,
                              (wantParts && wantFlatParts) ? &r.flatParts : nullptr,
                              &r.gradeFlatten);
    return r;
}

void appendLotMassBox(RenderMesh& out, const LotBuilding& lot,
                      const Vec3& sideColor, const Vec3& roofColor,
                      Real bottomY) {
    const Real hw = lot.width * 0.5, hd = lot.depth * 0.5;
    const Vec3 ax(std::cos(lot.yaw), 0, std::sin(lot.yaw));
    const Vec3 az(-std::sin(lot.yaw), 0, std::cos(lot.yaw));
    // The distant tier IS the building past detailDistance, so it carries its
    // own foundation reach: walls start at bottomY (min perimeter ground -
    // bed-in, computed by the host) instead of baseY. NAN/unset = legacy
    // baseY (flat levels).
    const Real wallBase = std::isfinite(bottomY)
                              ? std::min(bottomY, lot.baseY)
                              : lot.baseY;
    const Vec3 base(lot.site.x, wallBase, lot.site.y);
    const Vec3 top(lot.site.x, lot.baseY + lot.height, lot.site.y);
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
        // UVs in WINDOW CELLS (kMassBoxCell m per cell, kMassBoxTile cells per
        // texture repeat), so the loader's lit-window emissive map lands one
        // window per bay and storey on the far tier at night.
        const float uw = static_cast<float>(d.length() / (kMassBoxCell * kMassBoxTile));
        const float vh = static_cast<float>((top.y - base.y) / (kMassBoxCell * kMassBoxTile));
        MeshBuilder::emitQuadUV(out, base + a, base + b, top + b, top + a, n, sideColor,
                                0, 0, uw, 0, uw, vh, 0, vh);
    }
    // The roof cap samples the map's dark corner cell (its first cell is
    // always unlit by construction).
    const float rc = static_cast<float>(0.5 / kMassBoxTile);
    MeshBuilder::emitQuadUV(out, top + c[0], top + c[1], top + c[2], top + c[3],
                            Vec3(0, 1, 0), roofColor, rc, rc, rc, rc, rc, rc, rc, rc);
}

Mat4 outdoorPieceXform(const OutdoorPiece& p, Real y) {
    const Real c = std::cos(p.yaw), s = std::sin(p.yaw);
    Mat4 m;
    // +x (c, 0, -s), +y up, +z (s, 0, c): right-handed, the piece's front along +z
    m.m[0][0] = c;  m.m[0][1] = 0; m.m[0][2] = s;  m.m[0][3] = p.at.x;
    m.m[1][0] = 0;  m.m[1][1] = 1; m.m[1][2] = 0;  m.m[1][3] = y;
    m.m[2][0] = -s; m.m[2][1] = 0; m.m[2][2] = c;  m.m[2][3] = p.at.y;
    return m;
}

}  // namespace engine
