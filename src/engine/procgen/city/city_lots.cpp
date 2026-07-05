#include "city_lots.h"

#include "parcel.h"          // subdivideBlock, Lot, ParcelParams
#include "shape_grammar.h"   // scopeFromFootprint, growBuilding — REAL buildings
#include <cmath>

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

// Radial zoning + weighted pick — downtown skews commercial, the edge residential.
const char* typeFor(Real dist, const LotParams& p, Hash& rng) {
    const Real r = rng.unit();
    if (dist < p.innerRadius) {                 // downtown
        if (r < 0.45) return "office";
        if (r < 0.75) return "shop";
        if (r < 0.90) return "civic";
        return "home";
    }
    if (dist < p.midRadius) {                    // midtown, mixed
        if (r < 0.06) return "park";
        if (r < 0.28) return "shop";
        if (r < 0.48) return "office";
        if (r < 0.55) return "civic";
        return "home";
    }
    if (r < 0.10) return "park";                 // outskirts, residential
    if (r < 0.20) return "shop";
    return "home";
}

// A shape-grammar building recipe per place type, scaled a little to the lot's
// short side so a small lot doesn't sprout a tower. `wallColor` tints the facade.
BuildingParams paramsFor(const std::string& t, Real shortSide, const Vec3& color,
                         Hash& rng) {
    BuildingParams bp;
    bp.seed = rng.next();
    bp.wallColor = color;
    bp.faceDir = Vec3(0, 0, 1);
    const bool roomy = shortSide > 16.0;   // big enough to carry height
    if (t == "office") {
        bp.floors = roomy ? static_cast<int>(rng.range(6, 15)) : static_cast<int>(rng.range(4, 7));
        bp.curtainWall = rng.unit() < 0.5;
        bp.groundRetail = true;
    } else if (t == "civic") {
        bp.floors = static_cast<int>(rng.range(2, 5));
        bp.groundRetail = false;
        bp.pilasters = true;
    } else if (t == "shop") {
        bp.floors = static_cast<int>(rng.range(1, 3));
        bp.groundRetail = true;
        bp.awning = true;
    } else {   // home
        bp.floors = static_cast<int>(rng.range(1, roomy ? 4 : 3));
        bp.groundRetail = false;
        bp.baseCourse = true;
    }
    // Slenderness cap: total height stays under ~1.8x the footprint's short side,
    // so a small lot can't sprout a pencil tower (device: "very tall skinny and
    // completely malformed"). Floors ~3.2 m each; always at least one.
    const int maxFloors = std::max(1, static_cast<int>(shortSide * 1.8 / 3.2) - 1);
    bp.floors = std::min(bp.floors, maxFloors);
    return bp;
}

Vec3 colorFor(const std::string& t) {
    if (t == "home")   return {0.72, 0.55, 0.45};
    if (t == "shop")   return {0.82, 0.70, 0.42};
    if (t == "office") return {0.55, 0.62, 0.72};
    if (t == "civic")  return {0.80, 0.80, 0.85};
    if (t == "park")   return {0.35, 0.60, 0.35};
    return {0.72, 0.70, 0.64};
}
}  // namespace

std::vector<LotBuilding> growLotBuildings(const std::vector<Poly2>& blocks,
                                          const LotParams& p, LotPlanDebug* debug) {
    std::vector<LotBuilding> out;
    for (std::size_t bi = 0; bi < blocks.size(); ++bi) {
        const Poly2& block = blocks[bi];
        if (block.size() < 3) continue;
        // Pull in from the road edge to the buildable interior (road + sidewalk).
        Poly2 foot = inset(block, p.roadMargin);
        if (foot.size() < 3 || area(foot) < p.minLotArea * 1.5) continue;
        if (debug) debug->blocks.push_back(foot);

        ParcelParams pp;
        pp.seed = mix(static_cast<uint32_t>(bi), p.seed);
        pp.targetArea = 480;
        pp.minArea = p.minLotArea;
        pp.minEdge = p.minShort;
        std::vector<Lot> lots = subdivideBlock(foot, pp);
        if (debug)
            for (const Lot& lot : lots) debug->lots.push_back(lot.footprint);

        for (std::size_t li = 0; li < lots.size(); ++li) {
            const Lot& lot = lots[li];
            if (area(lot.footprint) < p.minLotArea) continue;
            Hash rng(mix(pp.seed, static_cast<uint32_t>(li) + 1));
            if (rng.unit() > p.buildChance) continue;   // plaza / gap

            // Building set back from its own lot lines.
            Poly2 site = inset(lot.footprint, p.lotSetback);
            if (site.size() < 3 || area(site) < 30) site = lot.footprint;

            OBB2 obb = orientedBoundingBox(site);
            const Real w = 2 * obb.half[0], d = 2 * obb.half[1];
            const Real shortSide = std::min(w, d), longSide = std::max(w, d);
            if (shortSide < p.minShort) continue;              // sliver
            if (longSide > shortSide * p.maxAspect) continue;  // knife blade
            // The building FILLS the site's oriented bounding box (that is the
            // grammar's scope), so a triangular / L-shaped off-cut whose polygon
            // covers little of its OBB would grow a mass that OVERHANGS the lot —
            // the "completely malformed" buildings (device feedback). Require the
            // lot to actually fill its box before building on it.
            if (area(site) < 0.72 * w * d) continue;

            LotBuilding b;
            b.site = centroid(site);
            b.width = w;
            b.depth = d;
            b.yaw = std::atan2(obb.axis[0].y, obb.axis[0].x);
            const Real dist = (b.site - p.center).length();
            b.type = typeFor(dist, p, rng);
            b.color = colorFor(b.type);
            if (b.type == "park") {
                b.height = 0.3;   // a low green pad; the caller draws it as a box
                out.push_back(std::move(b));
                continue;
            }
            // Grow a REAL building that FITS the lot: its oriented footprint IS the
            // scope, so the mass is contained by the lot; height comes from floors.
            BuildingParams bp = paramsFor(b.type, shortSide, b.color, rng);
            Scope scope = scopeFromFootprint(site, 0.0, 10.0 /* height set by floors */);
            BuildingMesh bm = growBuilding(scope, bp);
            if (bm.parts.empty()) continue;
            b.mesh = bm.merged();
            b.height = bm.height > 0 ? bm.height : 8.0;
            out.push_back(std::move(b));
        }
    }
    return out;
}

}  // namespace engine
