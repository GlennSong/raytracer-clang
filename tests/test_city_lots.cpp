#include "test_framework.h"

#include "../src/engine/procgen/city/architect.h"
#include "../src/engine/procgen/city/city_lots.h"
#include "../src/engine/procgen/city/road_network.h"
#include "../src/engine/procgen/city/shape_grammar.h"   // PartId (surfaced walls)

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
    return t == "home" || t == "shop" || t == "office" || t == "civic" ||
           t == "park" || t == "green";
}
}  // namespace

TEST_CASE(lot_buildings_are_grown_and_not_slivers) {
    LotParams p;
    p.center = {0, 0};
    p.seed = 3;
    std::vector<RenderMesh> parts;
    std::vector<LotBuilding> b = growLotBuildings(squareBlocks(), p, nullptr, &parts);
    CHECK(!b.empty());   // real blocks yield buildings
    for (const LotBuilding& lb : b) {
        const Real shortSide = std::min(lb.width, lb.depth);
        const Real longSide = std::max(lb.width, lb.depth);
        CHECK(shortSide >= p.minShort - 1e-6);          // no knife blades
        CHECK(longSide <= shortSide * p.maxAspect + 1e-6);
        CHECK(lb.height > 0.0);                         // has mass (park = low pad)
        CHECK(isKnownType(lb.type));                    // a valid schedule tag
    }
    // The buildings' geometry lands in the per-PartId meshes with the part id set
    // (the loader binds the shape-grammar PBR recipes onto exactly these).
    CHECK(!parts.empty());
    int filled = 0;
    for (std::size_t i = 0; i < parts.size(); ++i) {
        if (parts[i].vertices.empty()) continue;
        ++filled;
        CHECK(parts[i].materialIndex == static_cast<int>(i));
    }
    CHECK(filled >= 2);   // at least walls + one more class (glass/roof/trim)
    // Facades must land in the SURFACED wall parts (Brick/Concrete/Stucco/Metal
    // carry the procedural PBR texture recipes) — leaving every wall in the flat
    // PartId::Wall is the "buildings are colour-only" device bug.
    std::size_t surfacedVerts = 0;
    for (PartId id : {PartId::Brick, PartId::Concrete, PartId::Stucco, PartId::Metal})
        surfacedVerts += parts[static_cast<std::size_t>(id)].vertices.size();
    CHECK(surfacedVerts > 0);
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

TEST_CASE(edge_blocks_build_the_town_rim) {
    // A single long straight boundary road, no enclosed faces anywhere: edge
    // blocks must appear on BOTH open sides, sized within [minLen, maxLen] and
    // set back by the margin, and the usual lot pass grows buildings on them.
    RoadGraph roads;
    roads.nodes = { {Vec2(0, 0)}, {Vec2(150, 0)} };
    roads.edges = { RoadEdge{0, 1, 8, RoadClass::Local, 0} };
    EdgeBlockParams ep;   // margin 9, depth 34, len 26..58
    std::vector<Poly2> rim = edgeBlocks(roads, {}, ep);
    CHECK(!rim.empty());
    CHECK(rim.size() % 2 == 0u);   // both sides of a road with open ground
    for (const Poly2& r : rim) {
        CHECK(r.size() == 4u);
        Vec2 lo, hi; bounds(r, lo, hi);
        const Real len = hi.x - lo.x, depth = hi.y - lo.y;
        CHECK(len >= ep.minLen * 0.6 - 1e-6);
        CHECK(len <= ep.maxLen + 1e-6);
        CHECK_APPROX(depth, ep.depth, 1e-6);
        // Set back from the road (y=0) by the margin: no corner nearer than it.
        CHECK(std::min(std::abs(lo.y), std::abs(hi.y)) >= ep.margin - 1e-6);
    }
    // The rim rectangles grow buildings through the ordinary lot pass.
    LotParams lp; lp.seed = 9; lp.roadMargin = 2.0;   // rim blocks are pre-set-back
    CHECK(!growLotBuildings(rim, lp).empty());
}

TEST_CASE(edge_blocks_skip_covered_ground) {
    // The same road, but one side is an enclosed city block: only the open side
    // gets rim rectangles.
    RoadGraph roads;
    roads.nodes = { {Vec2(0, 0)}, {Vec2(150, 0)} };
    roads.edges = { RoadEdge{0, 1, 8, RoadClass::Local, 0} };
    Poly2 block{{-10, 5}, {160, 5}, {160, 80}, {-10, 80}};   // covers +y side
    EdgeBlockParams ep;
    std::vector<Poly2> rim = edgeBlocks(roads, {block}, ep);
    CHECK(!rim.empty());
    for (const Poly2& r : rim) {
        Vec2 lo, hi; bounds(r, lo, hi);
        CHECK(hi.y < 0);   // every rim block sits on the OPEN (-y) side
    }
}

// Buildings keep clear of road corridors (device: "buildings overlapping
// sidewalks and poking out onto the street"): with the sampled road graph +
// clearance passed in, every grown box corner stays at least half the road
// width + clearance from the centreline, even when the lot hugs the road.
TEST_CASE(lot_buildings_keep_clear_of_roads) {
    LotParams p;
    p.seed = 3;
    p.roadMargin = 6.0;   // deliberately TIGHT: the block reaches into the verge
    RoadGraph roads;
    roads.nodes.push_back({Vec2(-105, -200)});
    roads.nodes.push_back({Vec2(-105, 200)});
    roads.edges.push_back({0, 1, 13, RoadClass::Arterial, 0});   // wide arterial
    std::vector<Poly2> blocks = {
        {{-101, -45}, {-11, -45}, {-11, 45}, {-101, 45}}};       // hugs the road
    const Real clearance = 4.6;
    std::vector<LotBuilding> b =
        growLotBuildings(blocks, p, nullptr, nullptr, &roads, clearance);
    CHECK(!b.empty());
    const Real minDist = 13.0 * 0.5 + clearance;                 // 11.1 m
    for (const LotBuilding& lb : b) {
        if (lb.type == "park") continue;
        Vec2 a0(std::cos(lb.yaw), std::sin(lb.yaw));
        Vec2 a1(-a0.y, a0.x);
        for (int sx = -1; sx <= 1; sx += 2)
            for (int sy = -1; sy <= 1; sy += 2) {
                Vec2 c = lb.site + a0 * (lb.width * 0.5 * sx)
                                 + a1 * (lb.depth * 0.5 * sy);
                CHECK(std::fabs(c.x + 105.0) >= minDist - 0.2);
            }
    }
}

// --- The ARCHITECT pass (building-grammar-plan.md P5) ------------------------

TEST_CASE(district_map_places_quarters_deterministically) {
    DistrictMap dm;
    dm.center = {0, 0};
    dm.innerRadius = 55;
    dm.midRadius = 135;
    dm.seed = 5;
    CHECK(dm.tagAt({10, 5}) == DistrictTag::Financial);   // the core
    int oldTown = 0, commercial = 0, industrial = 0, residential = 0;
    for (int k = 0; k < 64; ++k) {                        // ring samples
        Real a = 6.28318 * k / 64;
        DistrictTag mid = dm.tagAt({std::cos(a) * 95, std::sin(a) * 95});
        if (mid == DistrictTag::OldTown) ++oldTown;
        if (mid == DistrictTag::Commercial) ++commercial;
        DistrictTag outer = dm.tagAt({std::cos(a) * 170, std::sin(a) * 170});
        if (outer == DistrictTag::Industrial) ++industrial;
        if (outer == DistrictTag::Residential) ++residential;
    }
    CHECK(oldTown > 3);          // the old-town pocket exists...
    CHECK(commercial > oldTown); // ...inside a commercial ring
    CHECK(industrial > 5);       // the industrial wedge exists...
    CHECK(residential > industrial);   // ...on a residential ring
    for (int k = 0; k < 16; ++k) {     // deterministic
        Vec2 q(std::cos(k * 0.7) * 120, std::sin(k * 0.7) * 120);
        CHECK(dm.tagAt(q) == dm.tagAt(q));
    }
}

TEST_CASE(architect_tables_are_coherent) {
    // The financial table holds no cottages; the residential table holds no
    // towers; the industrial wedge builds solid sheds; old town has ONE look.
    int finTall = 0, finPitched = 0;
    int resLow = 0, resPitched = 0, resTall = 0;
    int indSolid = 0;
    for (uint32_t s = 1; s <= 40; ++s) {
        BuildingRecipe f = architectPick(DistrictTag::Financial, 22, 460, s);
        if (f.params.floors >= 4) ++finTall;
        if (f.params.roofStyle != BuildingParams::RoofStyle::Flat) ++finPitched;
        BuildingRecipe r = architectPick(DistrictTag::Residential, 14, 220, s);
        if (r.massing != BuildingRecipe::Massing::Park) {
            if (r.params.floors <= 4) ++resLow; else ++resTall;
            if (r.params.roofStyle != BuildingParams::RoofStyle::Flat) ++resPitched;
        }
        BuildingRecipe ind = architectPick(DistrictTag::Industrial, 24, 500, s);
        if (ind.params.solidFacade) ++indSolid;
        BuildingRecipe ot = architectPick(DistrictTag::OldTown, 12, 160, s);
        CHECK(ot.params.floors <= 3);                               // low
        CHECK(ot.params.roofStyle == BuildingParams::RoofStyle::Hip);   // one look
        CHECK(ot.params.window.head == OpeningStyle::Head::Round);
    }
    CHECK(finTall >= 30);      // downtown is tall
    CHECK(finPitched == 0);    // and never a cottage roof
    CHECK(resTall == 0);       // the outskirts never grow a tower
    CHECK(resLow >= 30);
    CHECK(resPitched >= 12);   // pitched roofs are common out there
    CHECK(indSolid >= 20);     // the wedge is mostly sheds
}
