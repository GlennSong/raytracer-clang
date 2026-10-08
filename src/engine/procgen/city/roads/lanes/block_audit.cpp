#include "engine/procgen/city/roads/lanes/deck_mesh.h"
#include "engine/procgen/city/roads/lanes/road_graph_spec.h"
#include "engine/procgen/city/roads/lanes/block_audit.h"

#include "engine/level_params.h"
#include "engine/procgen/city/roads/lanes/geom2d.h"
#include "engine/procgen/city/roads/lanes/road_twin.h"
#include "engine/procgen/city/roads/lanes/polyline_ops.h"
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <set>
#include <unordered_map>
#include <sstream>

namespace engine {
namespace roads::lanes {

namespace {
std::vector<size_t> dpKeep(const Ring& pts, double tol) {
    std::vector<char> keep(pts.size(), 0); if (pts.size() < 3) return {};
    keep.front() = 1; keep.back() = 1; std::vector<std::pair<size_t, size_t>> stack{{0, pts.size() - 1}};
    while (!stack.empty()) {
        auto [a, b] = stack.back(); stack.pop_back(); if (b <= a + 1) continue;
        const Vec2 d = pts[b] - pts[a]; const double L = std::hypot(d.x, d.y); size_t best = a; double bd = -1;
        for (size_t i = a + 1; i < b; ++i) { const Vec2 v = pts[i] - pts[a]; const double dist = L > 1e-9 ? std::fabs(v.x * d.y - v.y * d.x) / L : std::hypot(v.x, v.y); if (dist > bd) { bd = dist; best = i; } }
        if (bd > tol) { keep[best] = 1; stack.push_back({a, best}); stack.push_back({best, b}); }
    }
    std::vector<size_t> out; for (size_t i = 0; i < pts.size(); ++i) if (keep[i]) out.push_back(i); return out;
}
}  // namespace

std::vector<Ring> pavementHoles(const Result& r, double minArea) {
    // The buildable space is what the pavement's holes enclose MINUS any pavement inside them. A hole is not a
    // block when other paved polygons sit inside it: a ground-level freeway ring whose frontage road never
    // touches it has one hole — the whole city — and taking it as a block parcelled lots over every street
    // (the slip-ramp metro, 2026-09-07). Hole-shaped pieces (an annulus: the band between the ring and the
    // frontage road) are verge, not blocks, and are dropped; the inner polygon's own holes are the blocks.
    // The SIDEWALK is pavement too (Glenn, 2026-09-21: blocks sat inset from the street by a
    // strip of grass). Holes used to stop at the kerb and the loader then inset every block by
    // the level's `citysim.sidewalk` — a lattice number, 5 m on metro_lanes against a 3.5 m
    // sidewalk actually drawn. With the drawn sidewalk in the union, a hole stops at the back
    // of whatever sidewalk each street really has, and the block begins there.
    std::vector<Ring> holes; const PolySet paved = unionSets(unionSets(r.pavement.surface, r.pavement.shoulder), r.pavement.sidewalk);
    PolySet enclosed;
    for (const Polygon2& p : paved) for (const Ring& h : p.holes) if (std::fabs(ringArea(h)) >= minArea) for (const Polygon2& q : fromRing(h)) enclosed.push_back(q);
    if (enclosed.empty()) return holes;
    for (const Polygon2& piece : differenceSets(unionSets(enclosed, PolySet{}), paved)) {
        if (!piece.holes.empty() || std::fabs(ringArea(piece.outer)) < minArea) continue;
        holes.push_back(piece.outer);
    }
    return holes;
}

std::vector<Poly2> blocksFromHoles(const std::vector<Ring>& holes, double simplify, double insetBy, double minWidth,
                                   const std::vector<Ring>* water) {
    std::vector<Poly2> blocks; std::vector<Ring> rings;
    for (const Ring& h : holes) {
        if (insetBy <= 0) { rings.push_back(h); continue; }
        // A hole may split when inset. A piece that vanishes under a further 3 m inset is a strip under 2*(inset+3) m
        // wide — the verge between a freeway and its frontage road, a median — not a block (2026-09-07).
        // The strip test is an absolute width (default 2 * (inset + 3), the old rule): a hole that
        // already excludes its sidewalks is inset by centimetres, and "vanishes under 3 m more"
        // would then keep 7 m verges as blocks.
        const double stripHalf = std::max(3.0, (minWidth > 0 ? minWidth : 2.0 * (insetBy + 3.0)) * 0.5 - insetBy);
        for (const Polygon2& q : offsetSet(fromRing(h), -insetBy)) if (std::fabs(ringArea(q.outer)) >= 135.0 && !offsetSet(PolySet{q}, -stripHalf).empty()) rings.push_back(q.outer);
    }
    // THE WATER (rivers and lakes, ADR-0104): a block the river runs through is two blocks, one each
    // side; a block only grazed by it loses the wet strip. Pieces under 135 m2 are not blocks.
    if (water && !water->empty()) {
        const PolySet wet = unionRings(*water);
        std::vector<Ring> dry;
        for (const Ring& h : rings)
            for (const Polygon2& q : differenceSets(fromRing(h), wet))
                if (std::fabs(ringArea(q.outer)) >= 135.0) dry.push_back(q.outer);
        rings = std::move(dry);
    }
    for (const Ring& h : rings) {
        Ring closed = h; closed.push_back(h.front());   // DP wants an open run: split the loop at its first point
        std::vector<size_t> keep = dpKeep(closed, simplify);
        Poly2 poly; for (size_t i : keep) if (i + 1 < closed.size()) poly.push_back(closed[i]);
        if (poly.size() < 3) continue;
        double a = 0; for (size_t i = 0; i < poly.size(); ++i) { const Vec2& u = poly[i]; const Vec2& v = poly[(i + 1) % poly.size()]; a += u.x * v.y - v.x * u.y; }
        if (a < 0) std::reverse(poly.begin(), poly.end());   // CCW, as extractBlocks would hand them
        blocks.push_back(std::move(poly));
    }
    return blocks;
}

std::vector<Poly2> sceneBlocks(const Result& r, double simplify, double minArea, double insetBy) { return blocksFromHoles(pavementHoles(r, minArea), simplify, insetBy); }

std::vector<TerrainFlatten> lanesEarthworkPins(const RoadDeckField& deck, double sidewalk, double below, double falloff) {
    std::vector<TerrainFlatten> out;
    // THE ELEVATED DECKS, binned: a street's strip must never reach over a LOWER structure beside it -- a ramp
    // descending 13 m off a collector 2.3 m above it lay under the collector's strip, pinned to the collector's
    // height (metro_lanes: 396 ramp samples under up to 2.2 m of ground, 2026-09-30).
    struct ESeg { Vec2 a, b; double ya, yb, hw; };
    std::vector<ESeg> elev;
    for (const UnionSpine& sp : deck.spines) {
        if (!(sp.authoredDeck || sp.layer != 0) || sp.points.size() < 2 || sp.yAbs.size() != sp.points.size()) continue;
        for (std::size_t i = 0; i + 1 < sp.points.size(); ++i)
            elev.push_back({sp.points[i], sp.points[i + 1], sp.yAbs[i], sp.yAbs[i + 1], i < sp.hw.size() ? sp.hw[i] : sp.halfWidth});
    }
    constexpr double kCell = 48;
    auto key = [](int cx, int cz) { return (static_cast<long long>(cx) << 32) ^ static_cast<uint32_t>(cz); };
    std::unordered_map<long long, std::vector<int>> egrid;
    for (std::size_t k = 0; k < elev.size(); ++k) {
        const ESeg& e = elev[k]; const double pad = e.hw + 30.0;
        for (int cx = static_cast<int>(std::floor((std::min(e.a.x, e.b.x) - pad) / kCell)); cx <= static_cast<int>(std::floor((std::max(e.a.x, e.b.x) + pad) / kCell)); ++cx)
            for (int cz = static_cast<int>(std::floor((std::min(e.a.y, e.b.y) - pad) / kCell)); cz <= static_cast<int>(std::floor((std::max(e.a.y, e.b.y) + pad) / kCell)); ++cz)
                egrid[key(cx, cz)].push_back(static_cast<int>(k));
    }
    // how far this strip may reach from its centreline at q before it would cover a deck lower than y
    auto reachAllowed = [&](const Vec2& q, double y) {
        double allowed = 1e30;
        auto it = egrid.find(key(static_cast<int>(std::floor(q.x / kCell)), static_cast<int>(std::floor(q.y / kCell))));
        if (it == egrid.end()) return allowed;
        for (int k : it->second) {
            const ESeg& e = elev[static_cast<std::size_t>(k)];
            const Vec2 ab = e.b - e.a; const double l2 = dot(ab, ab);
            double t = l2 > 1e-12 ? dot(q - e.a, ab) / l2 : 0.0; t = std::clamp(t, 0.0, 1.0);
            const double ey = e.ya + (e.yb - e.ya) * t;
            if (ey > y - 0.3) continue;   // level with or above this street: not buried by it
            allowed = std::min(allowed, (q - (e.a + ab * t)).length() - e.hw - 1.0);
        }
        return allowed;
    };
    for (const UnionSpine& sp : deck.spines) {
        if (sp.authoredDeck || sp.layer != 0 || sp.points.size() < 2 || sp.yAbs.size() != sp.points.size()) continue;
        for (std::size_t i = 0; i + 1 < sp.points.size(); ++i) {
            const Vec2 a = sp.points[i], b = sp.points[i + 1];
            if ((b - a).length() < 0.05) continue;
            double hw = (i < sp.hw.size() ? std::max(sp.hw[i], i + 1 < sp.hw.size() ? sp.hw[i + 1] : sp.hw[i]) : sp.halfWidth) + std::max(0.0, sidewalk);
            if (!elev.empty()) {
                const double y = std::max(sp.yAbs[i], sp.yAbs[i + 1]);
                // the strip AND its feather stop short of the lower deck (the finish's 6 m feather buried the ramp
                // once its strip was clipped)
                hw = std::min({hw, reachAllowed(a, y) - falloff, reachAllowed(b, y) - falloff, reachAllowed((a + b) * 0.5, y) - falloff});
                if (hw < 1.0) continue;   // a lower structure runs right beside or under this piece: no strip here
            }
            TerrainFlatten f = makeFlattenRamp(Vec3(a.x, 0, a.y), Vec3(b.x, 0, b.y), sp.yAbs[i] - below, sp.yAbs[i + 1] - below, hw, falloff);
            f.priority = kRoadFlattenPriority;
            out.push_back(std::move(f));
        }
    }
    return out;
}

double lanesMinSidewalk(const nlohmann::json& level, double fallback) {
    double best = 1e30;
    if (level.contains("entities") && level["entities"].is_array())
        for (const nlohmann::json& e : level["entities"]) {
            const nlohmann::json* road = e.contains("road") && e["road"].is_object() ? &e["road"] : e.contains("lanelab") && e["lanelab"].is_object() ? &e["lanelab"] : nullptr;
            if (!road || !road->contains("sidewalks")) continue;
            const nlohmann::json& sw = (*road)["sidewalks"];
            if (sw.is_number()) best = std::min(best, sw.get<double>());
            else if (sw.is_object()) for (const auto& kv : sw.items()) if (kv.value().is_number()) best = std::min(best, kv.value().get<double>());
        }
    return best < 1e29 ? std::min(best, fallback > 0 ? fallback : best) : fallback;
}

std::function<bool(double, double)> lanesNearFreeway(const RoadDeckField& deck, double clear) {
    struct Seg { Vec2 a, b; double hw; };
    auto segs = std::make_shared<std::vector<Seg>>();
    for (const UnionSpine& sp : deck.spines) {
        if (sp.klass != RoadClass::Freeway && sp.klass != RoadClass::Ramp) continue;
        for (std::size_t i = 0; i + 1 < sp.points.size(); ++i) segs->push_back({sp.points[i], sp.points[i + 1], i < sp.hw.size() ? sp.hw[i] : sp.halfWidth});
    }
    if (segs->empty()) return {};
    constexpr double kCell = 32;
    auto key = [](int cx, int cz) { return (static_cast<long long>(cx) << 32) ^ static_cast<uint32_t>(cz); };
    auto grid = std::make_shared<std::unordered_map<long long, std::vector<int>>>();
    for (std::size_t k = 0; k < segs->size(); ++k) {
        const Seg& g = (*segs)[k]; const double pad = g.hw + clear;
        for (int cx = static_cast<int>(std::floor((std::min(g.a.x, g.b.x) - pad) / kCell)); cx <= static_cast<int>(std::floor((std::max(g.a.x, g.b.x) + pad) / kCell)); ++cx)
            for (int cz = static_cast<int>(std::floor((std::min(g.a.y, g.b.y) - pad) / kCell)); cz <= static_cast<int>(std::floor((std::max(g.a.y, g.b.y) + pad) / kCell)); ++cz)
                (*grid)[key(cx, cz)].push_back(static_cast<int>(k));
    }
    return [segs, grid, key, clear](double x, double z) {
        auto it = grid->find(key(static_cast<int>(std::floor(x / kCell)), static_cast<int>(std::floor(z / kCell))));
        if (it == grid->end()) return false;
        for (int k : it->second) {
            const Seg& g = (*segs)[static_cast<std::size_t>(k)];
            const Vec2 ab = g.b - g.a; const double l2 = dot(ab, ab);
            double t = l2 > 1e-12 ? dot(Vec2(x, z) - g.a, ab) / l2 : 0.0; t = std::clamp(t, 0.0, 1.0);
            if ((Vec2(x, z) - (g.a + ab * t)).length() - g.hw < clear) return true;
        }
        return false;
    };
}

std::function<bool(double, double, double*)> lanesStreetHeight(const RoadDeckField& deck, double reach) {
    struct Seg { Vec2 a, b; double ya, yb, hw; };
    auto segs = std::make_shared<std::vector<Seg>>();
    for (const UnionSpine& sp : deck.spines) {
        if (sp.authoredDeck || sp.layer != 0 || sp.klass == RoadClass::Freeway || sp.klass == RoadClass::Ramp) continue;
        if (sp.points.size() < 2 || sp.yAbs.size() != sp.points.size()) continue;
        for (std::size_t i = 0; i + 1 < sp.points.size(); ++i)
            segs->push_back({sp.points[i], sp.points[i + 1], sp.yAbs[i], sp.yAbs[i + 1], i < sp.hw.size() ? sp.hw[i] : sp.halfWidth});
    }
    if (segs->empty()) return {};
    constexpr double kCell = 32;
    auto key = [](int cx, int cz) { return (static_cast<long long>(cx) << 32) ^ static_cast<uint32_t>(cz); };
    auto grid = std::make_shared<std::unordered_map<long long, std::vector<int>>>();
    const double pad = reach + 16.0;
    for (std::size_t k = 0; k < segs->size(); ++k) {
        const Seg& g = (*segs)[k];
        const int x0 = static_cast<int>(std::floor((std::min(g.a.x, g.b.x) - pad) / kCell)), x1 = static_cast<int>(std::floor((std::max(g.a.x, g.b.x) + pad) / kCell));
        const int z0 = static_cast<int>(std::floor((std::min(g.a.y, g.b.y) - pad) / kCell)), z1 = static_cast<int>(std::floor((std::max(g.a.y, g.b.y) + pad) / kCell));
        for (int cx = x0; cx <= x1; ++cx) for (int cz = z0; cz <= z1; ++cz) (*grid)[key(cx, cz)].push_back(static_cast<int>(k));
    }
    return [segs, grid, key, reach](double x, double z, double* out) {
        auto it = grid->find(key(static_cast<int>(std::floor(x / kCell)), static_cast<int>(std::floor(z / kCell))));
        if (it == grid->end()) return false;
        double best = reach; bool found = false;
        for (int k : it->second) {
            const Seg& g = (*segs)[static_cast<std::size_t>(k)];
            const Vec2 ab = g.b - g.a; const double l2 = dot(ab, ab);
            double t = l2 > 1e-12 ? dot(Vec2(x, z) - g.a, ab) / l2 : 0.0; t = std::clamp(t, 0.0, 1.0);
            const double d = (Vec2(x, z) - (g.a + ab * t)).length() - g.hw;
            if (d < best) { best = d; *out = g.ya + (g.yb - g.ya) * t; found = true; }
        }
        return found;
    };
}

std::shared_ptr<const std::function<double(double, double)>> lanesEarthworkField(
    const RoadDeckField& deck, double sidewalk, const std::function<double(double, double)>& natural,
    const EarthworkParams& params, double seaLevel, EarthworkStats* stats) {
    const std::vector<TerrainFlatten> pins = lanesEarthworkPins(deck, sidewalk);
    if (pins.empty()) return nullptr;
    // HOLD the ground under every elevated deck (layer > 0 or an authored deck), its shadow plus 6 m: its
    // piers and underside were built at bake time on the ground there (#96 pillars floating, #95 ramps
    // sunk, 2026-09-30 -- the street pins pulled the field up or down beneath the structures).
    std::vector<TerrainFlatten> holds;
    static const bool holdOff = [] { const char* e = std::getenv("RT_EARTHWORK_HOLD"); return e && e[0] == '0'; }();   // A/B
    for (const UnionSpine& sp : deck.spines) {
        if (holdOff) break;
        if (!(sp.authoredDeck || sp.layer != 0) || sp.points.size() < 2) continue;
        for (std::size_t i = 0; i + 1 < sp.points.size(); ++i) {
            const Vec2 a = sp.points[i], b = sp.points[i + 1];
            if ((b - a).length() < 0.05) continue;
            const double hw = (i < sp.hw.size() ? sp.hw[i] : sp.halfWidth) + 6.0;
            holds.push_back(makeFlattenRamp(Vec3(a.x, 0, a.y), Vec3(b.x, 0, b.y), 0.0, 0.0, hw, 0.0));
        }
    }
    return buildEarthworkField(pins, natural, params, seaLevel, stats, &holds);
}

double lanesSidewalkRise() { return Rules{}.skirtDrop + kSidewalkLift; }

// The feather lives INSIDE the block: footprints are inset by it, so the ramp ends at the block line.
// 1 m (was 2): the streets' own strips now carry the ground up to the sidewalk (lanesStreetFinish), and a
// pad clipped 2 m short left its building's side walls standing on those strips' feathers -- at the SIDE
// street's height, above a pad set by the front one (69 buried on island_8_nature, 2026-09-29). A pad
// outranks every feather, so a wall on its pad cannot be buried; a higher side street shows a stepped base.
double lanesPadFalloff(double sidewalk) { (void)sidewalk; return 1.0; }

namespace {
// A flatten footprint shrunk by its own feather and clipped to the block, so the graded plane plus its
// ramp never leaves the block: the block edge is the back of the sidewalk, and anything past it lifts
// pavement (the flatten outranks the road). Returns the pieces (a footprint can split).
// Every block inset by the feather, as its pieces with their bounds -- the clip set a footprint is cut to.
// (A concave block's centroid can lie outside it, so there is no per-lot "its block" lookup.) The pieces are
// NOT unioned: blocks are disjoint and an inset of a simple ring has no holes, and Clipper's NonZero clip
// already treats a list of pieces as their union. The first cut unioned them one block at a time
// (quadratic) and then cut every lot against all of them at once: on island_8_nature's 12,856 lots that
// was about half of a 100 s load. A footprint is now cut against the pieces its bounds touch -- the same
// region, since the rest cannot overlap it.
struct BlockInset { PolySet pieces; std::vector<Box2> box; };
BlockInset blocksInset(const std::vector<Poly2>& blocks, double falloff) {
    BlockInset in;
    for (const Poly2& b : blocks)
        for (Polygon2& pg : offsetSet(fromRing(b), -falloff)) { in.box.push_back(bounds(pg)); in.pieces.push_back(std::move(pg)); }
    return in;
}
std::vector<std::vector<Vec3>> insideBlock(const std::vector<Vec3>& polygon, const BlockInset& blocks, double falloff, bool shrinkSelf = true) {
    Ring ring; for (const Vec3& v : polygon) ring.emplace_back(v.x, v.z);
    const PolySet self = shrinkSelf ? offsetSet(fromRing(ring), -falloff) : fromRing(ring);
    if (self.empty()) return {};
    const Box2 sb = bounds(self);
    PolySet near;
    for (std::size_t i = 0; i < blocks.pieces.size(); ++i) {
        const Box2& b = blocks.box[i];
        if (b.maxX < sb.minX || b.minX > sb.maxX || b.maxY < sb.minY || b.minY > sb.maxY) continue;
        near.push_back(blocks.pieces[i]);
    }
    if (near.empty()) return {};
    PolySet ps = intersectSets(self, near);   // the blocks inset too: a terrace wider than its block would otherwise feather from the block line outward
    std::vector<std::vector<Vec3>> out;
    for (const Polygon2& pg : ps) { if (pg.outer.size() < 3 || std::fabs(ringArea(pg.outer)) < 1.0) continue; std::vector<Vec3> poly; for (const Vec2& q : pg.outer) poly.push_back(Vec3(q.x, 0, q.y)); out.push_back(std::move(poly)); }
    return out;
}
}  // namespace

std::vector<TerrainFlatten> clipPadsToBlocks(const std::vector<LotBuilding>& lots, const std::vector<Poly2>& blocks, double sidewalk) {
    std::vector<TerrainFlatten> out; const double falloff = lanesPadFalloff(sidewalk); const BlockInset inside = blocksInset(blocks, falloff);
    for (const LotBuilding& lb : lots) {
        if (lb.type == "park" || lb.type == "green" || (lb.plan.size() < 3 && lb.type != "depot")) continue;   // (a bus depot's yard is paved)
        // THE PAD HOLDS ITS WHOLE PARCEL; only the BLOCK EDGE gives up the feather. Shrinking the pad
        // by its feather on every side (it was) left a 4 m strip at every party line that neither
        // neighbour's pad owned — the outer 2 m of every plate and plaza stood on a ramp, and a
        // higher neighbour's feather climbed over it (147 of metro_lanes' 162 buried plates, inside
        // their blocks). Clipped to the block inset by the feather, the ramp still ends at the
        // block line — behind the sidewalk, never in the road — and neighbours meet at their lot line.
        const TerrainFlatten raw = lotPadFlatten(lb, 2.2 + falloff, falloff);
        for (std::vector<Vec3>& poly : insideBlock(raw.polygon, inside, falloff, /*shrinkSelf=*/false)) out.push_back(makeFlattenPad(std::move(poly), lb.groundY, falloff));
    }
    return out;
}

std::vector<TerrainFlatten> lanesTerraces(const std::vector<TerrainFlatten>& grades, const std::vector<Poly2>& blocks, double sidewalk) {
    std::vector<TerrainFlatten> out; const double falloff = lanesPadFalloff(sidewalk); const BlockInset inside = blocksInset(blocks, falloff);
    for (const TerrainFlatten& g : grades) {
        if (g.polygon.size() < 3) continue;
        for (std::vector<Vec3>& poly : insideBlock(g.polygon, inside, falloff)) { TerrainFlatten f = g; f.polygon = std::move(poly); f.falloff = falloff; f.minX = f.minZ = 1e300; f.maxX = f.maxZ = -1e300; for (const Vec3& v : f.polygon) { f.minX = std::min(f.minX, v.x); f.maxX = std::max(f.maxX, v.x); f.minZ = std::min(f.minZ, v.z); f.maxZ = std::max(f.maxZ, v.z); } out.push_back(std::move(f)); }
    }
    return out;
}

std::vector<PadConflict> padsOnPavement(const Result& r, const std::vector<TerrainFlatten>& pads, double kerb) {
    std::vector<PadConflict> out; if (!r.hasTerrain || r.terrain.nx < 2 || r.conform.nodeOwner.size() != r.terrain.z.size()) return out;
    const HeightGrid& G = r.terrain;
    auto smooth = [](double u) { u = std::clamp(u, 0.0, 1.0); return u * u * (3 - 2 * u); };
    for (size_t pi = 0; pi < pads.size(); ++pi) {
        const TerrainFlatten& f = pads[pi]; if (f.polygon.size() < 3) continue;
        Ring ring; for (const Vec3& v : f.polygon) ring.emplace_back(v.x, v.z);
        const int i0 = std::max(0, static_cast<int>(std::floor((f.minX - f.falloff - G.x0) / G.res))), i1 = std::min(G.nx - 1, static_cast<int>(std::ceil((f.maxX + f.falloff - G.x0) / G.res)));
        const int j0 = std::max(0, static_cast<int>(std::floor((f.minZ - f.falloff - G.y0) / G.res))), j1 = std::min(G.ny - 1, static_cast<int>(std::ceil((f.maxZ + f.falloff - G.y0) / G.res)));
        PadConflict pc; pc.pad = static_cast<int>(pi);
        for (int j = j0; j <= j1; ++j) for (int i = i0; i <= i1; ++i) {
            const size_t n = static_cast<size_t>(j) * G.nx + i; if (r.conform.nodeOwner[n] == "-" || r.conform.nodeDist[n] > 0.01) continue;   // not on pavement or sidewalk
            const Vec2 q(G.x0 + i * G.res, G.y0 + j * G.res); double w = 1.0;
            if (!pointInRing(ring, q)) {
                double d = 1e300; for (size_t k = 0; k < ring.size(); ++k) { const Vec2& a = ring[k]; const Vec2& b = ring[(k + 1) % ring.size()]; const Vec2 ab = b - a; const double L2 = dot(ab, ab); const double t = L2 > 1e-12 ? std::clamp(dot(q - a, ab) / L2, 0.0, 1.0) : 0.0; d = std::min(d, distance(q, a + ab * t)); }
                if (d >= f.falloff) continue; w = smooth(1.0 - d / f.falloff);
            }
            const double plane = f.c + f.dx * q.x + f.dz * q.y, ground = G.z[n], rise = (plane - ground) * w;
            if (rise <= kerb) continue;
            pc.area += G.cellArea(); if (rise > pc.rise) { pc.rise = rise; pc.at = q; pc.what = r.conform.nodeOwner[n]; }
        }
        if (pc.area > 0) out.push_back(pc);
    }
    return out;
}

BlockAudit auditBlocks(const Result& r, const nlohmann::json& citysim, bool fromScene) {
    BlockAudit a;
    RoadEntity twin = roadTwin(r);
    EdgeBlockParams ep; LotParams lp;
    readLotGrowParams(citysim.is_object() ? citysim : nlohmann::json::object(), ep, lp);
    const HeightGrid* grid = r.hasTerrain ? &r.terrain : nullptr;
    RoadGroundFn ground = [grid](double x, double z) { return grid ? grid->sample(x, z) : 0.0; };
    lp.ground = [grid](Real x, Real z) { return static_cast<Real>(grid ? grid->sample(x, z) : 0.0); };
    RoadGraph row;   // the freeway right-of-way, as the loader hands it: freeway + ramp edges of the twin
    for (const RoadNode& n : twin.graph.nodes) row.nodes.push_back(n);
    for (const RoadEdge& e : twin.graph.edges) if (e.klass == RoadClass::Freeway || e.klass == RoadClass::Ramp) row.edges.push_back(e);
    const double roadClear = citysim.is_object() ? citysim.value("sidewalk", 4.0) + 0.6 : 4.6;
    if (fromScene) {
        // ONE representation: the blocks ARE the pavement's holes, exact to the kerb, so the pass needs no
        // road graph to keep them off the asphalt — no clearance graph, no right-of-way band, no twin.
        const double sidewalk = lp.roadMargin; lp.roadMargin = 0;   // inset here, robustly; the pass's own inset is a no-op
        std::vector<Poly2> blocks = sceneBlocks(r, 1.5, 2000.0, sidewalk);
        a.holesGiven = blocks.size();
        const bool wantParts = std::getenv("LANELAB_AUDIT_PARTS") != nullptr;   // grow the building meshes too (slow): `lanelab_tool blocks --at` lists them
        a.grown.lots = growLotBuildings(blocks, lp, &a.grown.plan, wantParts ? &a.grown.parts : nullptr, nullptr, 0.0, wantParts ? &a.grown.flatParts : nullptr, &a.grown.gradeFlatten);
        {   // pads and terraces against the pavement: as the loader built them before, and as it builds them now
            std::vector<TerrainFlatten> raw, now;   // `sidewalk` from the block inset above
            for (const LotBuilding& lb : a.grown.lots) { if (lb.type == "park" || lb.type == "green" || lb.plan.size() < 3) continue; raw.push_back(lotPadFlatten(lb)); }
            raw.insert(raw.end(), a.grown.gradeFlatten.begin(), a.grown.gradeFlatten.end());
            now = clipPadsToBlocks(a.grown.lots, blocks, sidewalk);
            for (TerrainFlatten& f : lanesTerraces(a.grown.gradeFlatten, blocks, sidewalk)) now.push_back(std::move(f));
            a.padConflictsRaw = padsOnPavement(r, raw); a.padConflicts = padsOnPavement(r, now);
            if (std::getenv("LANELAB_TRACE_PADS")) for (const PadConflict& pc : a.padConflicts) {
                const TerrainFlatten& f = now[static_cast<size_t>(pc.pad)]; const bool terrace = pc.pad >= static_cast<int>(now.size() - lanesTerraces(a.grown.gradeFlatten, blocks, sidewalk).size());
                std::fprintf(stderr, "pad-conflict: %s #%d bbox [%.0f %.0f]-[%.0f %.0f] falloff %.1f plane c %.2f dx %.4f dz %.4f; +%.2f m over %.0f m2 at (%.0f, %.0f) on %s\n", terrace ? "terrace" : "pad", pc.pad, f.minX, f.minZ, f.maxX, f.maxZ, f.falloff, f.c, f.dx, f.dz, pc.rise, pc.area, pc.at.x, pc.at.y, pc.what.c_str());
            }
            for (const PadConflict& pc : a.padConflictsRaw) a.padAreaRaw += pc.area; for (const PadConflict& pc : a.padConflicts) a.padArea += pc.area;
        }
        // holes the pass returned nothing for: a block foot's centroid inside the hole claims it
        for (const Poly2& h : blocks) {
            bool claimed = false;
            for (const Poly2& f : a.grown.plan.blocks) { Vec2 c; for (const Vec2& q : f) c += q; c = c / static_cast<double>(f.size()); if (pointInRing(h, c)) { claimed = true; break; } }
            if (claimed) continue;
            double ar = 0, shortest = 1e300; Vec2 c;
            for (size_t i = 0; i < h.size(); ++i) { const Vec2& u = h[i]; const Vec2& v = h[(i + 1) % h.size()]; ar += u.x * v.y - v.x * u.y; shortest = std::min(shortest, (v - u).length()); c += u; }
            Poly2 foot = h; double fa = 0; for (size_t i = 0; i < foot.size(); ++i) { const Vec2& u = foot[i]; const Vec2& v = foot[(i + 1) % foot.size()]; fa += u.x * v.y - v.x * u.y; }
            a.dropped.push_back({std::fabs(ar / 2), h.size(), shortest, c / static_cast<double>(h.size()), foot.size() < 3 ? std::string("inset rejected the polygon") : std::fabs(fa / 2) < lp.minLotArea * 1.5 ? "too small after the inset" : "parceller found no lot"});
        }
    } else {
        std::vector<RoadEntity> nets{twin};
        a.grown = growLotBuildingsOnNets(nets, lp, ep, roadClear, ground, row.edges.empty() ? nullptr : &row, false, false);
    }
    // the audit
    const PolySet& paved = r.pavement.surface;
    for (const Poly2& foot : a.grown.plan.blocks) {
        if (foot.size() < 3) continue;
        BlockReport b; b.foot = foot; b.vertices = foot.size();
        Ring ring(foot.begin(), foot.end()); PolySet fs = fromRing(ring);
        b.area = std::fabs(ringArea(ring));
        const PolySet opened = offsetSet(offsetSet(fs, -4.0), 4.0);
        b.thinArea = std::max(0.0, b.area - setArea(opened));
        for (const Polygon2& piece : differenceSets(fs, opened)) {   // each thin piece: a spike if it is long
            double longest = 0; const Ring& o = piece.outer;
            for (size_t i = 0; i < o.size(); ++i) for (size_t j = i + 1; j < o.size(); ++j) longest = std::max(longest, (o[i] - o[j]).length());
            if (longest > 25.0) b.spikeArea += std::fabs(ringArea(piece.outer));
        }
        b.pavedArea = setArea(intersectSets(fs, paved));
        {   // convex hull (monotone chain) for the wedge-vs-spike call
            std::vector<Vec2> P(foot.begin(), foot.end()); std::sort(P.begin(), P.end(), [](const Vec2& u, const Vec2& v) { return u.x < v.x || (u.x == v.x && u.y < v.y); });
            auto cross2 = [](const Vec2& o, const Vec2& u, const Vec2& v) { return (u.x - o.x) * (v.y - o.y) - (u.y - o.y) * (v.x - o.x); };
            std::vector<Vec2> H(2 * P.size()); size_t k = 0;
            for (size_t i = 0; i < P.size(); ++i) { while (k >= 2 && cross2(H[k - 2], H[k - 1], P[i]) <= 0) --k; H[k++] = P[i]; }
            for (size_t i = P.size() - 1, t = k + 1; i > 0; --i) { while (k >= t && cross2(H[k - 2], H[k - 1], P[i - 1]) <= 0) --k; H[k++] = P[i - 1]; }
            H.resize(k > 1 ? k - 1 : 0); double ha = 0; for (size_t i = 0; i < H.size(); ++i) { const Vec2& u = H[i]; const Vec2& v = H[(i + 1) % H.size()]; ha += u.x * v.y - v.x * u.y; }
            ha = std::fabs(ha / 2); b.convexity = ha > 1e-6 ? b.area / ha : 1.0;
        }
        for (const Vec2& q : foot) b.centroid += q; b.centroid = b.centroid / static_cast<double>(foot.size());
        if (b.pavedArea > 1.0) {
            std::set<std::string> roads;
            for (size_t li = 0; li < r.lanes.lanes.size(); ++li) {
                const Lane& l = r.lanes.lanes[li]; if (l.parent < 0 || r.pavement.footprints[li].empty()) continue;
                if (setArea(intersectSets(fs, r.pavement.footprints[li])) > 0.5) roads.insert(r.graph.edges[static_cast<size_t>(l.parent)].id);
            }
            for (const std::string& s : roads) b.roads += (b.roads.empty() ? "" : " ") + s;
        }
        if (!b.ok()) ++a.broken;
        a.thinTotal += b.thinArea; a.pavedTotal += b.pavedArea; a.spikeTotal += b.spikeArea;
        a.blocks.push_back(std::move(b));
    }
    return a;
}

void writeBlocksSvg(const Result& r, const BlockAudit& a, const std::string& path) {
    const std::array<double, 4> b = r.graph.bounds(); const double W = b[1] - b[0], H = b[3] - b[2], s = 1600.0 / std::max(W, H);
    auto X = [&](double x) { return (x - b[0]) * s; }; auto Y = [&](double y) { return (b[3] - y) * s; };
    std::ofstream f(path);
    f << "<svg xmlns='http://www.w3.org/2000/svg' width='" << W * s << "' height='" << H * s << "' viewBox='0 0 " << W * s << " " << H * s << "'>\n";
    f << "<rect width='100%' height='100%' fill='#eef0e6'/>\n";
    auto ring = [&](const Ring& rg) { for (size_t i = 0; i < rg.size(); ++i) f << (i ? " L" : "M") << X(rg[i].x) << " " << Y(rg[i].y); f << " Z"; };
    f << "<path fill='#8d8d8d' fill-rule='evenodd' stroke='none' d='";
    for (const Polygon2& p : r.pavement.surface) { ring(p.outer); for (const Ring& h : p.holes) ring(h); }
    f << "'/>\n";
    for (const BlockReport& blk : a.blocks) {
        f << "<polygon fill='" << (blk.ok() ? "#cfe3c4" : "#f4b5b0") << "' fill-opacity='0.75' stroke='" << (blk.ok() ? "#3f6b3a" : "#c0392b") << "' stroke-width='" << 0.8 * s << "' points='";
        for (const Vec2& q : blk.foot) f << X(q.x) << "," << Y(q.y) << " ";
        f << "'/>\n";
    }
    for (const Poly2& l : a.grown.plan.lots) { f << "<polygon fill='none' stroke='#8c5a3c' stroke-width='" << 0.35 * s << "' points='"; for (const Vec2& q : l) f << X(q.x) << "," << Y(q.y) << " "; f << "'/>\n"; }
    for (const LotBuilding& lb : a.grown.lots) { if (lb.plan.size() < 3) continue; f << "<polygon fill='#2b2b2b' stroke='none' points='"; for (const Vec2& q : lb.plan) f << X(q.x) << "," << Y(q.y) << " "; f << "'/>\n"; }
    for (const BlockReport& blk : a.blocks) if (!blk.ok()) f << "<text x='" << X(blk.centroid.x) << "' y='" << Y(blk.centroid.y) << "' font-size='" << 6 * s << "' fill='#c0392b' text-anchor='middle'>thin " << static_cast<int>(blk.thinArea) << " / paved " << static_cast<int>(blk.pavedArea) << "</text>\n";
    f << "</svg>\n";
}

std::string summary(const BlockAudit& a) {
    std::ostringstream o;
    o << "pads: " << a.padConflicts.size() << " lift pavement or sidewalk (" << static_cast<int>(a.padArea) << " m²); unclipped they were " << a.padConflictsRaw.size() << " (" << static_cast<int>(a.padAreaRaw) << " m²)";
    for (size_t i = 0; i < a.padConflicts.size() && i < 6; ++i) { const PadConflict& pc = a.padConflicts[i]; o << (i == 0 ? ": " : "; ") << pc.what << " +" << std::fixed << std::setprecision(2) << pc.rise << " m at (" << static_cast<int>(pc.at.x) << ", " << static_cast<int>(pc.at.y) << ")"; }
    o << "\n";
    o << "blocks: " << a.blocks.size() << " of " << a.holesGiven << " pavement holes (" << a.grown.plan.lots.size() << " lots, " << a.grown.lots.size() << " built); broken " << a.broken
      << " — spikes " << static_cast<int>(a.spikeTotal) << " m² (thin " << static_cast<int>(a.thinTotal) << "), on pavement " << static_cast<int>(a.pavedTotal) << " m²\n";
    std::vector<const BlockReport*> worst; for (const BlockReport& b : a.blocks) if (!b.ok()) worst.push_back(&b);
    std::sort(worst.begin(), worst.end(), [](const BlockReport* x, const BlockReport* y) { return x->spikeArea + x->pavedArea > y->spikeArea + y->pavedArea; });
    for (size_t i = 0; i < worst.size() && i < 12; ++i) {
        const BlockReport& b = *worst[i];
        o << "  block " << static_cast<int>(b.area) << " m², " << b.vertices << " vertices at (" << static_cast<int>(b.centroid.x) << ", " << static_cast<int>(b.centroid.y) << "): spikes " << static_cast<int>(b.spikeArea) << " m² (thin " << static_cast<int>(b.thinArea) << "), paved " << static_cast<int>(b.pavedArea) << " m², convexity " << static_cast<int>(b.convexity * 100) << "%" << (b.roads.empty() ? "" : " under " + b.roads) << "\n";
    }
    if (!a.dropped.empty()) {
        o << "  holes that became no block: " << a.dropped.size() << "\n";
        std::vector<const DroppedHole*> d; for (const DroppedHole& x : a.dropped) d.push_back(&x);
        std::sort(d.begin(), d.end(), [](const DroppedHole* x, const DroppedHole* y) { return x->area > y->area; });
        for (size_t i = 0; i < d.size() && i < 12; ++i) o << "    " << static_cast<int>(d[i]->area) << " m², " << d[i]->vertices << " vertices, shortest edge " << std::fixed << std::setprecision(1) << d[i]->shortestEdge << " m at (" << static_cast<int>(d[i]->centroid.x) << ", " << static_cast<int>(d[i]->centroid.y) << "): " << d[i]->why << "\n";
    }
    return o.str();
}

}  // namespace roads::lanes
}  // namespace engine
