#include "test_framework.h"

#include "../src/engine/procgen/city/architect.h"
#include "../src/engine/procgen/city/city_lots.h"
#include "../src/engine/procgen/city/road_network.h"
#include "../src/engine/procgen/city/shape_grammar.h"   // PartId (surfaced walls)
#include "../src/engine/procgen/city/metro.h"
#include "../src/engine/procgen/city/road_constraints.h"

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
        // The no-sliver guarantee is about BUILDINGS: parks and greens are
        // ground scenery shaped by the lot itself (dense districts parcel
        // small, so a leftover green can be a narrow lawn strip — fine).
        // Dense districts (OldTown/Commercial) build down to minShortUrban:
        // an 8 m rowhouse plan is correct there, not a knife blade.
        if (lb.type != "park" && lb.type != "green") {
            CHECK(shortSide >= std::min(p.minShort, p.minShortUrban) - 1e-6);
            CHECK(longSide <= shortSide * p.maxAspect + 1e-6);
        }
        CHECK(lb.height > 0.0);                         // has mass (park = low pad)
        CHECK(isKnownType(lb.type));                    // a valid schedule tag
        // Unbuilt lots carry their OWN polygon (device: "square green lots
        // don't fit the blocks they're in"). The TERRAIN is their ground now
        // (device: "remove the green pads"): greens bake no pad mesh at all,
        // and a park's padMesh holds only its plaza + walking paths.
        if (lb.type == "park" || lb.type == "green") {
            CHECK(lb.pad.size() >= 3u);
            if (lb.type == "green") CHECK(lb.padMesh.vertices.empty());
        } else {
            CHECK(lb.pad.empty());
        }
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
// clearance passed in, every vertex of the grown building MESHES stays at
// least half the road width + clearance from the centreline, even when the
// lot hugs the road. (The mesh is the guarantee — lot buildings are visual-
// only since the invisible-walls fix, so there is no collider to test.)
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
    std::vector<RenderMesh> parts;
    std::vector<LotBuilding> b =
        growLotBuildings(blocks, p, nullptr, &parts, &roads, clearance);
    CHECK(!b.empty());
    int built = 0;
    for (const LotBuilding& lb : b)
        if (lb.type != "park" && lb.type != "green") ++built;
    CHECK(built > 0);
    const Real minDist = 13.0 * 0.5 + clearance;                 // 11.1 m
    // Every building vertex — walls, trim, roofs — stays behind the corridor.
    // Facade elements protrude from the plan (awnings ~0.9 m, hood bands), so
    // the tolerance is the element depth, not a rounding epsilon. The Path
    // part is EXEMPT by design: plaza walks and stair runs exist to reach the
    // sidewalk, so they legitimately run right up to the carriageway edge.
    for (std::size_t pi = 0; pi < parts.size(); ++pi) {
        if (pi == static_cast<std::size_t>(PartId::Path)) continue;
        for (const Vertex& v : parts[pi].vertices)
            CHECK(std::fabs(v.position.x + 105.0) >= minDist - 1.2);
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

    // The CORE is a SKYSCRAPER cluster: full coreness lifts the glass towers
    // well past the ring's 10-16 floors (the relaxed slender cap lets them
    // stand), while the district rim (coreness 0) keeps its old heights.
    int core18 = 0;
    for (uint32_t s = 1; s <= 40; ++s) {
        BuildingRecipe c = architectPick(DistrictTag::Financial, 24, 520, s, 1.0);
        if (c.params.floors >= 18) ++core18;
        BuildingRecipe rim = architectPick(DistrictTag::Financial, 24, 520, s, 0.0);
        CHECK(rim.params.floors <= 16);
    }
    CHECK(core18 >= 8);
}

TEST_CASE(landmark_recipes_are_coherent) {
    // Each civic archetype keeps its signature across seeds: a school is low
    // with a schoolyard, a fire station has vehicle bays, the market is one
    // tall arched hall, the courthouse is formal (pilasters + tall ground).
    for (uint32_t s = 1; s <= 12; ++s) {
        BuildingRecipe school =
            architectLandmark(LandmarkKind::School, 18, 420, s);
        CHECK(school.params.floors <= 3);
        CHECK(school.massing == BuildingRecipe::Massing::RectYard);
        CHECK(school.yardHalfWMax > 8.0);            // a real yard, not a lawn
        CHECK(school.name == "school");
        BuildingRecipe fire = architectLandmark(LandmarkKind::Fire, 16, 300, s);
        CHECK(fire.params.groundBays >= 2);
        CHECK(fire.params.floors <= 2);
        BuildingRecipe market =
            architectLandmark(LandmarkKind::Market, 14, 260, s);
        CHECK(market.params.floors == 0);            // one tall hall
        CHECK(market.params.window.head == OpeningStyle::Head::Round);
        CHECK(market.params.roofStyle == BuildingParams::RoofStyle::Gable);
        BuildingRecipe court =
            architectLandmark(LandmarkKind::Courthouse, 18, 400, s);
        CHECK(court.params.pilasters);
        CHECK(court.params.groundHeight > 5.0);
        CHECK(court.placeType == "civic");
    }
}

TEST_CASE(capitol_university_and_rowhouses_are_coherent) {
    // The capitol carries the full classical kit; the university keeps a
    // campus green; rowhouse strips pack Residential lots; mixed-use fills
    // commercial streets with shops-below-homes-above.
    for (uint32_t s = 1; s <= 10; ++s) {
        BuildingRecipe cap = architectLandmark(LandmarkKind::Capitol, 22, 480, s);
        CHECK(cap.params.dome);
        CHECK(cap.params.portico >= 6);
        CHECK(cap.params.entranceSteps);
        CHECK(cap.placeType == "civic");
        BuildingRecipe uni =
            architectLandmark(LandmarkKind::University, 20, 440, s);
        CHECK(uni.massing == BuildingRecipe::Massing::RectYard);
        CHECK(uni.yardHalfWMax > 10.0);   // the campus green
        CHECK(uni.params.portico >= 4);
        BuildingParams unit = architectRowUnit(s * 977u, 3);
        CHECK(unit.floors == 3);
        CHECK(unit.entranceSteps);        // the stoop
        CHECK(!unit.quoins);              // party walls, no corner masonry
    }
    int rowhouses = 0, mixed = 0;
    for (uint32_t s = 1; s <= 60; ++s) {
        BuildingRecipe r = architectPick(DistrictTag::Residential, 14, 300, s);
        if (r.massing == BuildingRecipe::Massing::RowStrip) ++rowhouses;
        BuildingRecipe c = architectPick(DistrictTag::Commercial, 12, 220, s);
        if (c.name == "mixed_use") ++mixed;
    }
    CHECK(rowhouses >= 4);   // the terrace archetype exists in the table
    CHECK(mixed >= 4);
}

TEST_CASE(style_book_hook_overlays_looks) {
    // The styleHook (the Lua data layer's engine-side seam) restyles by
    // recipe name without touching structure: force every yard house to a
    // hip roof and check the massing/floors survive untouched.
    LotParams p;
    p.center = {0, 0};
    p.seed = 11;
    p.styleHook = [](const std::string& recipe, BuildingParams& bp) {
        if (recipe == "yard_house") {
            bp.roofStyle = BuildingParams::RoofStyle::Hip;
            bp.roofPitch = 0.5;
        }
    };
    std::vector<Poly2> blocks;
    for (int gx = -1; gx <= 1; ++gx)
        for (int gz = -1; gz <= 1; ++gz) {
            Real cx = gx * 110.0, cz = gz * 110.0, h = 48.0;
            blocks.push_back({{cx - h, cz - h}, {cx + h, cz - h},
                              {cx + h, cz + h}, {cx - h, cz + h}});
        }
    std::vector<LotBuilding> b = growLotBuildings(blocks, p);
    std::vector<LotBuilding> plainB;
    {
        LotParams q = p;
        q.styleHook = nullptr;
        plainB = growLotBuildings(blocks, q);
    }
    CHECK(b.size() == plainB.size());   // looks changed, structure identical
    for (std::size_t i = 0; i < b.size(); ++i) {
        CHECK(b[i].recipe == plainB[i].recipe);
        CHECK(b[i].type == plainB[i].type);
    }
}

TEST_CASE(buildings_grow_from_terrain_base) {
    // City-on-terrain: with a ground sampler set, every building records its
    // graded PAD PLANE (groundY — the mid-slope average under the plan, which
    // the host flattens the terrain to) and grows from a small plinth reveal
    // above it. Park/green pads still drape per-vertex.
    LotParams p;
    p.center = {0, 0};
    p.seed = 3;
    p.ground = [](Real x, Real z) { return 0.04 * x + 0.02 * z; };
    std::vector<LotBuilding> b = growLotBuildings(squareBlocks(), p);
    CHECK(!b.empty());
    int sloped = 0;
    for (const LotBuilding& lb : b) {
        if (lb.type == "park" || lb.type == "green") {
            // Draped pads: vertex heights track the sampler, not a flat 0.
            for (const Vertex& v : lb.padMesh.vertices)
                CHECK(std::fabs(v.position.y -
                                (0.04 * v.position.x + 0.02 * v.position.z)) <
                      1.5);
            continue;
        }
        if (lb.plan.size() < 3) continue;
        if (lb.recipe == "plaza") continue;   // a podium SITS AT the pad grade
                                              // (no plinth reveal — by design)
        Real lo = 1e30, hi = -1e30;
        for (const Vec2& v : lb.plan) {
            const Real g = 0.04 * v.x + 0.02 * v.y;
            lo = std::min(lo, g);
            hi = std::max(hi, g);
        }
        // The pad plane is the ENTRANCE-side grade (sampled a couple of metres
        // toward the street), so it stays within the plan's own grades plus a
        // small slope allowance for that overstep.
        CHECK(lb.groundY >= lo - 0.5);
        CHECK(lb.groundY <= hi + 0.5);
        CHECK(lb.baseY > lb.groundY);             // walls sit proud of the pad
        CHECK(lb.baseY <= lb.groundY + 0.3);      // ...by a small plinth only
        if (std::fabs(lb.baseY) > 0.5) ++sloped;
    }
    CHECK(sloped > 0);   // the slope actually moved buildings off y=0
}

TEST_CASE(parcel_overrides_rescale_the_lot_grain) {
    // citysim.parcel (8km-city P3): a piedmont-scale metro lays 150 m+ blocks,
    // so the level can ask for bigger lots. The override rescales the per-
    // district grain; unset fields keep today's tuning exactly.
    LotParams p;
    p.seed = 3;
    LotPlanDebug base;
    growLotBuildings(squareBlocks(), p, &base);
    CHECK(!base.lots.empty());

    LotParams big = p;
    big.parcelTargetArea = 840;    // 2x the stock 420
    big.parcelFrontWidth = 32;     // 2x the stock 16
    big.parcelLotDepth = 42;       // 1.5x the stock 28
    big.parcelMinArea = 220;
    LotPlanDebug bigDbg;
    growLotBuildings(squareBlocks(), big, &bigDbg);
    CHECK(!bigDbg.lots.empty());
    CHECK(bigDbg.lots.size() < base.lots.size());   // bigger grain = fewer lots
    auto medianArea = [](const std::vector<Poly2>& lots) {
        std::vector<Real> a;
        for (const Poly2& l : lots) a.push_back(area(l));
        std::sort(a.begin(), a.end());
        return a.empty() ? Real(0) : a[a.size() / 2];
    };
    CHECK(medianArea(bigDbg.lots) > medianArea(base.lots) * 1.3);
    // Explicitly-unset overrides (< 0) change NOTHING: same plan as default.
    LotParams same = p;
    same.parcelTargetArea = -1;
    same.parcelMinEdge = -1;
    LotPlanDebug sameDbg;
    growLotBuildings(squareBlocks(), same, &sameDbg);
    CHECK(sameDbg.lots.size() == base.lots.size());
}

TEST_CASE(landmark_quotas_run_per_hub_cluster) {
    // 8km-city P3: multi-site metros plan their civic anchors PER HUB CLUSTER
    // — the primary city (cluster 0) keeps the global table (one courthouse),
    // and every satellite town is guaranteed its own school and church when it
    // has enough candidate lots. Planned, never rolled; deterministic.
    std::vector<LandmarkCand> cands;
    auto addLots = [&](int cluster, DistrictTag tag, int n, Vec2 base) {
        for (int i = 0; i < n; ++i) {
            LandmarkCand c;
            c.tag = tag;
            c.shortSide = 16 + (i % 5);
            c.area = 360 + 8 * i;
            c.pos = base + Vec2((i % 6) * 40.0, (i / 6) * 40.0);
            c.cluster = cluster;
            cands.push_back(c);
        }
    };
    addLots(0, DistrictTag::Financial, 8, {0, 0});        // the city core...
    addLots(0, DistrictTag::Commercial, 12, {150, 0});
    addLots(0, DistrictTag::Residential, 14, {0, 200});
    addLots(1, DistrictTag::Residential, 14, {2000, 0});  // a residential town
    addLots(2, DistrictTag::OldTown, 12, {0, 2000});      // an old-town town
    const std::vector<int> plan = planLandmarks(cands, Vec2(0, 0), 60.0);
    CHECK(plan.size() == cands.size());
    auto countIn = [&](int cluster, LandmarkKind k) {
        int n = 0;
        for (std::size_t i = 0; i < plan.size(); ++i)
            if (cands[i].cluster == cluster &&
                plan[i] == static_cast<int>(k)) ++n;
        return n;
    };
    CHECK(countIn(0, LandmarkKind::Courthouse) == 1);   // the city's table holds
    CHECK(countIn(1, LandmarkKind::School) >= 1);       // each town keeps its
    CHECK(countIn(1, LandmarkKind::Church) >= 1);       // guaranteed anchors
    CHECK(countIn(2, LandmarkKind::School) >= 1);
    CHECK(countIn(2, LandmarkKind::Church) >= 1);
    CHECK(countIn(2, LandmarkKind::Market) == 1);       // old town: market hall
    for (std::size_t i = 0; i < plan.size(); ++i)       // city-only anchors
        if (cands[i].cluster != 0) {                    // never leave the city
            CHECK(plan[i] != static_cast<int>(LandmarkKind::Courthouse));
            CHECK(plan[i] != static_cast<int>(LandmarkKind::Capitol));
        }
    CHECK(plan == planLandmarks(cands, Vec2(0, 0), 60.0));   // deterministic

    // ...and the guarantee holds through the FULL lot pass: blocks around a
    // residential hub far from the city (cluster 1) grow their own school and
    // church, deterministically across two runs.
    LotParams p;
    p.seed = 11;
    p.center = {0, 0};
    p.hubs = {{Vec2(0, 0), 0}, {Vec2(900, 0), 2}};
    p.hubClusters = {0, 1};
    p.hubRadius = 260;
    std::vector<Poly2> blocks;
    auto grid = [&](Vec2 c) {
        for (int gx = -1; gx <= 1; ++gx)
            for (int gz = -1; gz <= 1; ++gz) {
                Real cx = c.x + gx * 110.0, cz = c.y + gz * 110.0, h = 48.0;
                blocks.push_back({{cx - h, cz - h}, {cx + h, cz - h},
                                  {cx + h, cz + h}, {cx - h, cz + h}});
            }
    };
    grid({0, 0});
    grid({900, 0});
    std::vector<LotBuilding> b = growLotBuildings(blocks, p);
    int townSchools = 0, townChurches = 0;
    for (const LotBuilding& lb : b) {
        if (lb.site.x < 450) continue;   // the town's half of the world
        if (lb.recipe == "school") ++townSchools;
        if (lb.recipe == "church") ++townChurches;
    }
    CHECK(townSchools >= 1);
    CHECK(townChurches >= 1);
    std::vector<LotBuilding> b2 = growLotBuildings(blocks, p);
    CHECK(b.size() == b2.size());
    for (std::size_t i = 0; i < b.size(); ++i) CHECK(b[i].recipe == b2[i].recipe);
}

TEST_CASE(landmarks_are_planned_not_rolled) {
    // The planner fills civic quotas on the BEST lots: a city grown over real
    // blocks gets exactly one courthouse (on the most central financial lot),
    // at most the quota of schools, and every landmark name is recorded.
    LotParams p;
    p.center = {0, 0};
    p.seed = 11;
    // 3x3 grid of blocks spanning core -> outskirts so every district exists.
    std::vector<Poly2> blocks;
    for (int gx = -1; gx <= 1; ++gx)
        for (int gz = -1; gz <= 1; ++gz) {
            Real cx = gx * 110.0, cz = gz * 110.0, h = 48.0;
            blocks.push_back({{cx - h, cz - h}, {cx + h, cz - h},
                              {cx + h, cz + h}, {cx - h, cz + h}});
        }
    std::vector<LotBuilding> b = growLotBuildings(blocks, p);
    int courthouse = 0, schools = 0, named = 0;
    for (const LotBuilding& lb : b) {
        if (lb.recipe == "courthouse") ++courthouse;
        if (lb.recipe == "school") ++schools;
        if (!lb.recipe.empty()) ++named;
    }
    CHECK(courthouse == 1);       // one per city, never rolled
    CHECK(schools >= 1);          // the residential ring got its school
    CHECK(schools <= 3);          // ...but not one per parcel
    CHECK(named == static_cast<int>(b.size()));   // every lot knows its recipe
    // Determinism holds through the planner.
    std::vector<LotBuilding> c = growLotBuildings(blocks, p);
    CHECK(b.size() == c.size());
    for (std::size_t i = 0; i < b.size(); ++i) CHECK(b[i].recipe == c[i].recipe);
}

// FRONTAGE GATE (ADR-0066, stage 4): every built lot must touch a road. A lot
// "fronts" a street when its nearest road edge is within a reasonable setback
// distance (max 30 m). This gate verifies that the lot-placement algorithm keeps
// buildings from floating in the middle of the block, far from any street face.
TEST_CASE(every_built_lot_touches_a_road) {
    // Build a piedmont-style metro using the same multi-site params as test_metro_sites.
    MetroParams p;
    p.seed = 7;
    p.freeways = true;
    p.backboneClass = RoadClass::Arterial;
    p.freewayWidth = 17;
    p.arteryWidth = 17;
    p.collectorWidth = 13;
    p.streetWidth = 11;
    p.blockSize = 220;
    p.minBlockEdge = 150;
    p.segLength = 120;
    p.influence = 800;
    p.killRadius = 300;
    p.mergeRadius = 200;
    p.corridorSpacing = 240;
    p.ambientPer500 = 5;
    p.loopMin = 550;
    p.loopMax = 1300;
    p.interchangeSpacing = 600;

    MetroSite city;
    city.center = {900, 900};
    city.radius = 1400;
    city.hotspots = 9;
    city.blockSize = 220;
    MetroSite east;
    east.center = {3050, 400};
    east.radius = 420;
    east.hotspots = 3;
    east.blockSize = 180;
    east.density = 0.55;
    east.kindBias = 3;
    MetroSite south;
    south.center = {300, 3050};
    south.radius = 380;
    south.hotspots = 3;
    south.blockSize = 180;
    south.density = 0.5;
    south.kindBias = 2;
    p.sites = {city, east, south};

    RoadGraph roads = buildMetro(p);

    // Post-process the graph as test_metro_sites does (two rounds of consolidation).
    RoadRules rules;
    rules.autoRoundabout = false;
    roads = capDegree(planarize(applyConstraints(roads, rules), 1.0), rules);
    for (int round = 0; round < 2; ++round) {
        roads = dropParallelEdges(roads);
        roads = consolidateJunctionSpans(roads, 150.0, rules.maxDegree);
        roads = capDegree(planarize(roads, 1.0), rules);
        roads = mergeShortEdges(roads, 30.0, rules.maxDegree);
        roads = relaxSharpBends(roads, 0.5, 64);
    }

    // Extract the blocks and grow buildings on them.
    std::vector<Poly2> blocks = extractBlocks(roads, 200.0);
    CHECK(!blocks.empty());

    LotParams lotParams;
    lotParams.center = {900, 900};
    lotParams.seed = 11;
    std::vector<LotBuilding> buildings = growLotBuildings(blocks, lotParams);
    CHECK(!buildings.empty());

    // For each built (non-park/non-green) lot, compute the minimum distance from
    // the lot's OBB to any road edge in the graph. The lot's rectangle is defined
    // by site (centroid), width, depth (half-extents), and yaw (rotation).
    //
    // Helper: distance from a point to a line segment (a,b).
    auto distPointToSegment = [](const Vec2& p, const Vec2& a, const Vec2& b) -> Real {
        Vec2 ab = b - a;
        Real len2 = ab.lengthSquared();
        if (len2 < 1e-12) return (p - a).length();
        Real t = std::clamp(dot(p - a, ab) / len2, 0.0, 1.0);
        Vec2 closest = a + ab * t;
        return (p - closest).length();
    };

    // Helper: get the 4 corners of the OBB.
    auto getCorners = [](const Vec2& center, Real w, Real d, Real yaw) -> std::vector<Vec2> {
        Real cx = std::cos(yaw), sy = std::sin(yaw);
        Vec2 ax{cx, sy};      // long axis
        Vec2 ay{-sy, cx};     // short axis (perpendicular)
        return {
            center + ax * w + ay * d,
            center + ax * w - ay * d,
            center - ax * w - ay * d,
            center - ax * w + ay * d
        };
    };

    // Helper: distance from the lot rectangle to the nearest road edge.
    auto distToRoads = [&](const LotBuilding& lot) -> Real {
        std::vector<Vec2> corners = getCorners(lot.site, lot.width, lot.depth, lot.yaw);
        Real minDist = 1e30;

        // For each road edge, compute the distance from each corner to the line segment.
        for (const RoadEdge& edge : roads.edges) {
            Vec2 a = roads.nodes[edge.a].pos;
            Vec2 b = roads.nodes[edge.b].pos;
            for (const Vec2& corner : corners) {
                Real d = distPointToSegment(corner, a, b);
                minDist = std::min(minDist, d);
            }
        }
        return minDist;
    };

    // Measure frontage distances and report the distribution.
    std::vector<Real> distances;
    int nonCourtLots = 0;
    Real maxFrontage = 0;
    for (const LotBuilding& b : buildings) {
        // Skip parks and greens (they're ground scenery, not buildings).
        if (b.type == "park" || b.type == "green") continue;

        Real dist = distToRoads(b);
        distances.push_back(dist);
        nonCourtLots++;
        maxFrontage = std::max(maxFrontage, dist);
    }

    // Report the measured maximum frontage distance.
    if (!distances.empty()) {
        std::sort(distances.begin(), distances.end());
        Real median = distances[distances.size() / 2];
        std::printf("        [frontage] %d buildings, median %.1f m, max %.1f m\n",
                    nonCourtLots, median, maxFrontage);
    }

    // Assert: every non-court lot must touch a road within 30 m.
    // This is the frontage setback guarantee — lots are "adjacent" to the street.
    for (const LotBuilding& b : buildings) {
        if (b.type == "park" || b.type == "green") continue;
        Real dist = distToRoads(b);
        CHECK(dist <= 30.0);
    }
}
