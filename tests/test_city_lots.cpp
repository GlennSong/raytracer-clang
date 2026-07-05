#include "test_framework.h"

#include "../src/engine/procgen/city/city_lots.h"

using namespace engine;

// Living City, ADR-0066: buildings grown on the road network's blocks. The pass
// is pure geometry (blocks → lots → typed buildings), so its guarantees — real
// buildings come out, none are slivers, tagged for schedules, deterministic — are
// pinned headless; the visible spawn is device-gated.

namespace {
// A few big square blocks the parcel step can actually subdivide.
std::vector<Poly2> squareBlocks() {
    std::vector<Poly2> blocks;
    for (int i = 0; i < 4; ++i) {
        Real cx = (i % 2) * 120.0 - 60.0, cz = (i / 2) * 120.0 - 60.0;
        Real h = 45.0;   // 90 m block
        blocks.push_back({{cx - h, cz - h}, {cx + h, cz - h},
                          {cx + h, cz + h}, {cx - h, cz + h}});
    }
    return blocks;
}
bool isKnownType(const std::string& t) {
    return t == "home" || t == "shop" || t == "office" || t == "civic" || t == "park";
}
}  // namespace

TEST_CASE(lot_buildings_are_grown_and_not_slivers) {
    LotParams p;
    p.center = {0, 0};
    p.seed = 3;
    std::vector<LotBuilding> b = growLotBuildings(squareBlocks(), p);
    CHECK(!b.empty());   // real blocks yield buildings
    for (const LotBuilding& lb : b) {
        const Real shortSide = std::min(lb.width, lb.depth);
        const Real longSide = std::max(lb.width, lb.depth);
        CHECK(shortSide >= p.minShort - 1e-6);          // no knife blades
        CHECK(longSide <= shortSide * p.maxAspect + 1e-6);
        CHECK(lb.height > 0.0);                         // has mass (park = low pad)
        CHECK(isKnownType(lb.type));                    // a valid schedule tag
        // A real building carries grown geometry (floors/windows/roof); only a
        // park is a bare pad with no mesh.
        if (lb.type != "park") CHECK(!lb.mesh.vertices.empty());
    }
}

TEST_CASE(lot_buildings_are_deterministic) {
    LotParams p;
    p.seed = 7;
    std::vector<LotBuilding> a = growLotBuildings(squareBlocks(), p);
    std::vector<LotBuilding> c = growLotBuildings(squareBlocks(), p);
    CHECK(a.size() == c.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        CHECK(a[i].type == c[i].type);
        CHECK_APPROX(a[i].site.x, c[i].site.x, 1e-9);
        CHECK_APPROX(a[i].height, c[i].height, 1e-9);
    }
    LotParams q = p; q.seed = 99;
    std::vector<LotBuilding> d = growLotBuildings(squareBlocks(), q);
    CHECK(!d.empty());
}

TEST_CASE(lot_buildings_zone_radially) {
    // Downtown (near the centre) should skew commercial/civic, not all houses.
    LotParams p;
    p.center = {-60, -60};   // put downtown on the first block's centre
    p.seed = 5;
    std::vector<LotBuilding> b = growLotBuildings(squareBlocks(), p);
    int nearNonHome = 0;
    for (const LotBuilding& lb : b) {
        if ((lb.site - p.center).length() > p.innerRadius) continue;
        if (lb.type != "home") nearNonHome++;
    }
    CHECK(nearNonHome > 0);
}
