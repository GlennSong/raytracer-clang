#include "city_lots.h"

#include "../../engine/procgen/city/parcel.h"   // subdivideBlock, Lot, ParcelParams
#include <cmath>

namespace citysim {

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
PlaceType typeFor(Real dist, const LotParams& p, Hash& rng) {
    const Real r = rng.unit();
    if (dist < p.innerRadius) {                 // downtown
        if (r < 0.45) return PlaceType::Office;
        if (r < 0.75) return PlaceType::Shop;
        if (r < 0.90) return PlaceType::Civic;
        return PlaceType::Home;
    }
    if (dist < p.midRadius) {                    // midtown, mixed
        if (r < 0.06) return PlaceType::Park;
        if (r < 0.28) return PlaceType::Shop;
        if (r < 0.48) return PlaceType::Office;
        if (r < 0.55) return PlaceType::Civic;
        return PlaceType::Home;
    }
    if (r < 0.10) return PlaceType::Park;        // outskirts, residential
    if (r < 0.20) return PlaceType::Shop;
    return PlaceType::Home;
}

Real heightFor(PlaceType t, Hash& rng) {
    switch (t) {
        case PlaceType::Office: return rng.range(18, 42);
        case PlaceType::Civic:  return rng.range(10, 18);
        case PlaceType::Shop:   return rng.range(5, 9);
        case PlaceType::Home:   return rng.range(6, 12);
        case PlaceType::Park:   return 0.3;          // a low green pad, not a building
        default:                return 8.0;
    }
}

Vec3 colorFor(PlaceType t) {
    switch (t) {
        case PlaceType::Home:   return {0.72, 0.55, 0.45};
        case PlaceType::Shop:   return {0.82, 0.70, 0.42};
        case PlaceType::Office: return {0.55, 0.62, 0.72};
        case PlaceType::Civic:  return {0.80, 0.80, 0.85};
        case PlaceType::Park:   return {0.35, 0.60, 0.35};
        default:                return {0.72, 0.70, 0.64};
    }
}
}  // namespace

std::vector<LotBuilding> growLotBuildings(const std::vector<Poly2>& blocks,
                                          const LotParams& p) {
    std::vector<LotBuilding> out;
    for (std::size_t bi = 0; bi < blocks.size(); ++bi) {
        const Poly2& block = blocks[bi];
        if (block.size() < 3) continue;
        // Pull in from the road edge to the buildable interior (road + sidewalk).
        Poly2 foot = engine::inset(block, p.roadMargin);
        if (foot.size() < 3 || engine::area(foot) < p.minLotArea * 1.5) continue;

        engine::ParcelParams pp;
        pp.seed = mix(static_cast<uint32_t>(bi), p.seed);
        pp.targetArea = 480;
        pp.minArea = p.minLotArea;
        pp.minEdge = p.minShort;
        std::vector<engine::Lot> lots = engine::subdivideBlock(foot, pp);

        for (std::size_t li = 0; li < lots.size(); ++li) {
            const engine::Lot& lot = lots[li];
            if (engine::area(lot.footprint) < p.minLotArea) continue;
            Hash rng(mix(pp.seed, static_cast<uint32_t>(li) + 1));
            if (rng.unit() > p.buildChance) continue;   // plaza / gap

            // Building set back from its own lot lines.
            Poly2 site = engine::inset(lot.footprint, p.lotSetback);
            if (site.size() < 3 || engine::area(site) < 30) site = lot.footprint;

            engine::OBB2 obb = engine::orientedBoundingBox(site);
            const Real w = 2 * obb.half[0], d = 2 * obb.half[1];
            const Real shortSide = std::min(w, d), longSide = std::max(w, d);
            if (shortSide < p.minShort) continue;              // sliver
            if (longSide > shortSide * p.maxAspect) continue;  // knife blade

            LotBuilding b;
            b.site = engine::centroid(site);
            b.width = w;
            b.depth = d;
            b.yaw = std::atan2(obb.axis[0].y, obb.axis[0].x);
            const Real dist = (b.site - p.center).length();
            b.type = typeFor(dist, p, rng);
            b.height = heightFor(b.type, rng);
            b.color = colorFor(b.type);
            out.push_back(b);
        }
    }
    return out;
}

}  // namespace citysim
