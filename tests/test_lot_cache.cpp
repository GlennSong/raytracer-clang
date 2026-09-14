// Lot pass products through the bundle codecs (ADR-0084, milestone B): every field of BuildingParams and
// OpeningStyle set to a non-default value and compared after the round trip, plus lots with units, doors,
// pads, trees, fences, the plan record, terraces and part meshes.
#include "test_framework.h"

#include "../src/engine/bundle/bundle.h"
#include "../src/engine/mesh_builder.h"
#include "../src/engine/procgen/city/lot_cache.h"

#include <cstdio>
#include <cstring>

using namespace engine;
using namespace engine::lotcache;

namespace {
BuildingParams everyFieldSet() {
    BuildingParams p;
    p.floors = 12; p.groundRetail = false; p.floorHeight = 3.7; p.groundHeight = 5.1; p.bayWidth = 4.25; p.windowInset = 0.31; p.walkableGround = false; p.wallThickness = 0.45;
    p.openDoorway = true; p.setbackEvery = 2.5; p.setbackFloors = 4; p.parapet = 1.15; p.wallColor = Vec3(0.1, 0.2, 0.3); p.curtainWall = true; p.solidFacade = true;
    p.wallPart = PartId::Glass; p.faceDir = Vec3(1, 0, 0); p.entranceDropBelow = 0.75; p.baseCourse = false; p.stringCourse = false; p.pilasters = true; p.awning = false;
    p.window.head = OpeningStyle::Head::Round; p.window.archRise = 0.41; p.window.frameWidth = 0.17; p.window.lightsX = 3; p.window.lightsY = 2; p.window.sill = false; p.window.hood = OpeningStyle::Hood::Arch; p.window.frameColor = Vec3(0.4, 0.5, 0.6);
    p.quoins = true; p.groundBays = 7; p.portico = 4; p.entranceSteps = true; p.dome = true; p.roofStyle = BuildingParams::RoofStyle::Sawtooth; p.roofPitch = 0.83; p.retailStreetOnly = true;
    p.balconies = true; p.porch = true; p.chimney = true; p.spire = true; p.steeple = true; p.parkingDecks = true; p.sideBays = 5; p.trimColor = Vec3(0.7, 0.8, 0.9);
    p.shape = BuildingShape::Cylinder; p.tiers = 9; p.sides = 48; p.seed = 0xDEADBEEFu;
    p.envelope = BuildingParams::Envelope::StreetWallSetback; p.baseFloors = 6; p.setback1 = 7.5; p.stepFloors = 9; p.stepDepth = 2.25; p.towerFrac = 0.4; p.towerFloor = 24;
    return p;
}
bool sameVec3(const Vec3& a, const Vec3& b) { return a.x == b.x && a.y == b.y && a.z == b.z; }
bool sameParams(const BuildingParams& a, const BuildingParams& b) {
    return a.floors == b.floors && a.groundRetail == b.groundRetail && a.floorHeight == b.floorHeight && a.groundHeight == b.groundHeight && a.bayWidth == b.bayWidth && a.windowInset == b.windowInset
        && a.walkableGround == b.walkableGround && a.wallThickness == b.wallThickness && a.openDoorway == b.openDoorway && a.setbackEvery == b.setbackEvery && a.setbackFloors == b.setbackFloors && a.parapet == b.parapet
        && sameVec3(a.wallColor, b.wallColor) && a.curtainWall == b.curtainWall && a.solidFacade == b.solidFacade && a.wallPart == b.wallPart && sameVec3(a.faceDir, b.faceDir) && a.entranceDropBelow == b.entranceDropBelow
        && a.baseCourse == b.baseCourse && a.stringCourse == b.stringCourse && a.pilasters == b.pilasters && a.awning == b.awning
        && a.window.head == b.window.head && a.window.archRise == b.window.archRise && a.window.frameWidth == b.window.frameWidth && a.window.lightsX == b.window.lightsX && a.window.lightsY == b.window.lightsY && a.window.sill == b.window.sill && a.window.hood == b.window.hood && sameVec3(a.window.frameColor, b.window.frameColor)
        && a.quoins == b.quoins && a.groundBays == b.groundBays && a.portico == b.portico && a.entranceSteps == b.entranceSteps && a.dome == b.dome && a.roofStyle == b.roofStyle && a.roofPitch == b.roofPitch && a.retailStreetOnly == b.retailStreetOnly
        && a.balconies == b.balconies && a.porch == b.porch && a.chimney == b.chimney && a.spire == b.spire && a.steeple == b.steeple && a.parkingDecks == b.parkingDecks && a.sideBays == b.sideBays && sameVec3(a.trimColor, b.trimColor)
        && a.shape == b.shape && a.tiers == b.tiers && a.sides == b.sides && a.seed == b.seed
        && a.envelope == b.envelope && a.baseFloors == b.baseFloors && a.setback1 == b.setback1 && a.stepFloors == b.stepFloors && a.stepDepth == b.stepDepth && a.towerFrac == b.towerFrac && a.towerFloor == b.towerFloor;
}
bool samePoly(const Poly2& a, const Poly2& b) { if (a.size() != b.size()) return false; for (size_t i = 0; i < a.size(); ++i) if (a[i].x != b[i].x || a[i].y != b[i].y) return false; return true; }
}  // namespace

TEST_CASE(lot_cache_round_trips_every_building_parameter) {
    const BuildingParams p = everyFieldSet(); const BuildingParams d;
    CHECK(!sameParams(p, d));   // the fixture really differs from the defaults everywhere it matters
    bundle::BinWriter w; putBuildingParams(w, p); BuildingParams q; bundle::BinReader r(w.bytes.data(), w.bytes.size());
    CHECK(getBuildingParams(r, q)); CHECK(sameParams(p, q));
    bundle::BinReader bad(w.bytes.data(), w.bytes.size() - 5); BuildingParams q2; CHECK(!getBuildingParams(bad, q2));
    std::printf("    sizeof(BuildingParams) = %zu (a new field must be added to lot_cache.cpp and kLotsFormatVersion bumped)\n", sizeof(BuildingParams));
}

TEST_CASE(lot_cache_round_trips_a_lot_result_with_every_part_of_it_populated) {
    NetLotResult res;
    LotBuilding a; a.site = Vec2(10, -20); a.width = 12.5; a.depth = 8.25; a.height = 17; a.yaw = 0.7; a.type = "shop"; a.block = 3; a.district = "oldtown"; a.recipe = "corner_shop"; a.baseY = 4.5; a.groundY = 4.25;
    a.plan = {Vec2(0, 0), Vec2(12, 0), Vec2(12, 8), Vec2(0, 8)}; a.color = Vec3(0.3, 0.2, 0.1); a.pad = {Vec2(-1, -1), Vec2(13, -1), Vec2(13, 9), Vec2(-1, 9)};
    a.padMesh = MeshBuilder::box(Vec3(3, 0.2, 2)); a.padMesh.vertices[0].color = Vec3(0.9, 0.1, 0.1);   // a varying vertex colour survives
    a.treeSpots = {Vec3(1, 2, 3), Vec3(4, 5, 6)}; a.fenceSegs = {{Vec2(0, 0), Vec2(5, 0)}, {Vec2(5, 0), Vec2(5, 5)}};
    a.open = {{{Vec2(-1, -1), Vec2(13, -1), Vec2(13, 0), Vec2(-1, 0)}, OpenKind::Forecourt}, {{Vec2(12, 0), Vec2(13, 0), Vec2(13, 9)}, OpenKind::SideYard}};
    a.pavedLot = a.pad; a.paveY = 4.8; a.lot = {Vec2(-2, -2), Vec2(14, -2), Vec2(14, 10), Vec2(-2, 10)}; a.padBound = {Vec2(-1, -2), Vec2(13, -2), Vec2(13, 9)};
    BuildingUnit u; u.plan = {Vec2(1, 1), Vec2(11, 1), Vec2(11, 7), Vec2(1, 7)}; u.baseY = 4.6; u.params = everyFieldSet(); DoorSpec d; d.foot = Vec2(6, 1); d.normal = Vec2(0, -1); d.width = 1.2; d.height = 2.4; u.doors = {d}; u.enterable = true;
    BuildingUnit u2; u2.plan = {Vec2(2, 2), Vec2(4, 2), Vec2(4, 4)}; u2.baseY = 5; a.units = {u, u2};
    LotBuilding park; park.type = "park"; park.site = Vec2(-50, 50); park.height = 0.25; park.pad = {Vec2(-60, 40), Vec2(-40, 40), Vec2(-40, 60), Vec2(-60, 60)}; park.treeSpots = {Vec3(-55, 0, 45)};
    res.lots = {a, park};
    res.plan.blocks = {a.pad}; res.plan.lots = {a.plan, park.pad}; res.plan.alleys = {{Vec2(0, 0), Vec2(1, 1)}};
    res.plan.rejChance = 1; res.plan.rejSliver = 2; res.plan.rejAspect = 3; res.plan.rejFill = 4; res.plan.rejPlan = 5; res.plan.rejClear = 6; res.plan.rejBox = 7; res.plan.rejFrontage = 8; res.plan.bisectedBlocks = 9;
    res.plan.pEdgeShort = 10; res.plan.pShallow = 11; res.plan.pMitered = 12; res.plan.pOverlap = 13; res.plan.pEscaped = 14; res.plan.pTiny = 15; res.plan.pThin = 16; res.plan.pPlaced = 17;
    res.plan.pClips = 18; res.plan.pLeftOverlapping = 19; res.plan.pSameEdge = 20; res.plan.pAtInsert = 21; res.plan.pConcave = 22; res.plan.pClipFailed = 23; res.plan.rejRelief = 24;
    res.parts.resize(3); res.parts[0] = MeshBuilder::box(Vec3(4, 9, 4)); res.parts[0].materialIndex = 7; res.parts[2] = MeshBuilder::box(Vec3(1, 1, 1)); for (Vertex& v : res.parts[2].vertices) v.color = Vec3(0.2, 0.2, 0.2); res.parts[2].vertices[1].color = Vec3(1, 0, 0);
    res.flatParts.resize(1); res.flatParts[0] = MeshBuilder::box(Vec3(4, 9, 4)); res.flatParts[0].materialIndex = 2;
    TerrainFlatten f1; f1.polygon = {Vec3(0, 0, 0), Vec3(10, 0, 0), Vec3(10, 0, 10)}; f1.c = 3.5; f1.dx = -0.01; f1.dz = 0.02; f1.falloff = 2; f1.minX = 0; f1.minZ = 0; f1.maxX = 10; f1.maxZ = 10; f1.falloffMode = TerrainFlatten::Falloff::DaylightBatter; f1.cutBatter = 1.2; f1.fillBatter = 0.5; f1.priority = -1; f1.owner = 4;
    TerrainFlatten f2; f2.polygon = {Vec3(1, 0, 1), Vec3(2, 0, 1), Vec3(2, 0, 2), Vec3(1, 0, 2)}; f2.c = 9; res.gradeFlatten = {f1, f2};

    bundle::BundleWriter w; w.openMemory(); writeLotResult(w, "lots/e0/", res); nlohmann::json manifest = nlohmann::json::object(); std::string err; CHECK(w.finish(manifest, &err));
    std::unique_ptr<bundle::Bundle> b = bundle::Bundle::fromMemory(w.memory(), &err); CHECK(b != nullptr); if (!b) return;
    CHECK(b->has("lots/e0/lots") && b->has("lots/e0/plan") && b->has("lots/e0/grade") && b->has("lots/e0/parts/0") && !b->has("lots/e0/parts/1") && b->has("lots/e0/parts/2") && b->has("lots/e0/flat/0"));
    NetLotResult back; CHECK(readLotResult(*b, "lots/e0/", back, &err)); if (!err.empty()) std::printf("    %s\n", err.c_str());
    CHECK(back.lots.size() == 2);
    if (back.lots.size() == 2) {
        const LotBuilding& x = back.lots[0];
        CHECK(x.site.x == 10 && x.site.y == -20 && x.width == 12.5 && x.depth == 8.25 && x.height == 17 && x.yaw == 0.7 && x.type == "shop" && x.block == 3 && x.district == "oldtown" && x.recipe == "corner_shop" && x.baseY == 4.5 && x.groundY == 4.25);
        CHECK(samePoly(x.plan, a.plan) && samePoly(x.pad, a.pad) && sameVec3(x.color, a.color));
        CHECK(x.padMesh.vertices.size() == a.padMesh.vertices.size() && x.padMesh.indices == a.padMesh.indices);
        CHECK_APPROX(x.padMesh.vertices[0].color.x, 0.9, 1e-6); CHECK_APPROX(x.padMesh.vertices[3].position.x, static_cast<float>(a.padMesh.vertices[3].position.x), 0.0);
        CHECK(x.treeSpots.size() == 2 && x.treeSpots[1].z == 6 && x.fenceSegs.size() == 2 && x.fenceSegs[1].second.y == 5);
        CHECK(x.open.size() == 2 && x.open[0].kind == OpenKind::Forecourt && samePoly(x.open[0].poly, a.open[0].poly) && x.open[1].kind == OpenKind::SideYard && x.open[1].poly.size() == 3);
        CHECK(samePoly(x.pavedLot, a.pad) && x.paveY == 4.8 && samePoly(x.lot, a.lot) && samePoly(x.padBound, a.padBound));
        CHECK(x.units.size() == 2 && x.units[0].enterable && !x.units[1].enterable && x.units[0].baseY == 4.6 && samePoly(x.units[0].plan, u.plan) && samePoly(x.units[1].plan, u2.plan));
        CHECK(x.units.size() == 2 && sameParams(x.units[0].params, u.params) && sameParams(x.units[1].params, BuildingParams()));
        CHECK(x.units.size() == 2 && x.units[0].doors.size() == 1 && x.units[0].doors[0].foot.x == 6 && x.units[0].doors[0].normal.y == -1 && x.units[0].doors[0].width == 1.2 && x.units[0].doors[0].height == 2.4);
        const LotBuilding& y = back.lots[1]; CHECK(y.type == "park" && y.height == 0.25 && y.plan.empty() && samePoly(y.pad, park.pad) && y.treeSpots.size() == 1 && y.padMesh.vertices.empty() && y.units.empty());
    }
    CHECK(back.plan.blocks.size() == 1 && back.plan.lots.size() == 2 && back.plan.alleys.size() == 1 && back.plan.alleys[0].second.x == 1);
    CHECK(back.plan.rejChance == 1 && back.plan.rejFrontage == 8 && back.plan.bisectedBlocks == 9 && back.plan.pPlaced == 17 && back.plan.pClipFailed == 23 && back.plan.rejRelief == 24);
    CHECK(back.parts.size() == 3 && back.flatParts.size() == 1);
    if (back.parts.size() == 3) {
        CHECK(back.parts[0].materialIndex == 7 && back.parts[0].vertices.size() == res.parts[0].vertices.size() && back.parts[0].indices == res.parts[0].indices && back.parts[1].vertices.empty());
        CHECK_APPROX(back.parts[0].vertices[5].position.y, static_cast<float>(res.parts[0].vertices[5].position.y), 0.0);
        CHECK(back.parts[2].vertices.size() == res.parts[2].vertices.size()); CHECK_APPROX(back.parts[2].vertices[1].color.x, 1.0, 1e-6); CHECK_APPROX(back.parts[2].vertices[0].color.x, 0.2, 1e-6);
    }
    if (back.flatParts.size() == 1) CHECK(back.flatParts[0].materialIndex == 2 && back.flatParts[0].indices == res.flatParts[0].indices);
    CHECK(back.gradeFlatten.size() == 2);
    if (back.gradeFlatten.size() == 2) {
        const TerrainFlatten& g = back.gradeFlatten[0];
        CHECK(g.polygon.size() == 3 && g.polygon[2].z == 10 && g.c == 3.5 && g.dx == -0.01 && g.dz == 0.02 && g.falloff == 2 && g.maxX == 10 && g.maxZ == 10 && g.falloffMode == TerrainFlatten::Falloff::DaylightBatter && g.cutBatter == 1.2 && g.fillBatter == 0.5 && g.priority == -1 && g.owner == 4);
        CHECK(back.gradeFlatten[1].polygon.size() == 4 && back.gradeFlatten[1].c == 9 && back.gradeFlatten[1].falloffMode == TerrainFlatten::Falloff::Smoothstep);
    }
    CHECK(b->json("lots/e0/meta").value("units", 0) == 2 && b->json("lots/e0/meta").value("built", 0) == 1);
}

TEST_CASE(lot_cache_stores_parts_per_render_cell_and_reassembles_them) {
    using namespace engine::bundle; using namespace engine::lotcache;
    NetLotResult res; res.parts.resize(2);
    // one part spanning two 100 m cells: a box in cell (0,0) and a smaller one in cell (1,1)
    RenderMesh a = MeshBuilder::box(Vec3(4, 9, 4)); for (Vertex& v : a.vertices) v.position = v.position + Vec3(50, 0, 50);
    RenderMesh b2 = MeshBuilder::box(Vec3(2, 3, 2)); for (Vertex& v : b2.vertices) v.position = v.position + Vec3(150, 0, 150);
    res.parts[1] = a; const uint32_t base = static_cast<uint32_t>(a.vertices.size());
    res.parts[1].vertices.insert(res.parts[1].vertices.end(), b2.vertices.begin(), b2.vertices.end()); for (uint32_t i : b2.indices) res.parts[1].indices.push_back(base + i);
    res.parts[1].materialIndex = 5;
    res.flatParts.resize(1); res.flatParts[0] = a; res.flatParts[0].materialIndex = 3;
    BundleWriter w; w.openMemory(); writeLotResult(w, "lots/", res, 100.0); nlohmann::json manifest = nlohmann::json::object(); std::string err; CHECK(w.finish(manifest, &err));
    std::unique_ptr<Bundle> b = Bundle::fromMemory(w.memory(), &err); CHECK(b != nullptr); if (!b) return;
    CHECK(lotCellSize(*b, "lots/") == 100.0);
    std::vector<LotCellPart> cps; CHECK(listLotCellParts(*b, "lots/", cps, &err));
    CHECK(cps.size() == 3);   // part 1 in (0,0) and (1,1); flat 0 in (0,0)
    int flats = 0; size_t tris = 0;
    for (const LotCellPart& cp : cps) { RenderMesh m; CHECK(readLotPart(*b, cp.section, m)); CHECK(m.materialIndex == (cp.flat ? 3 : 5)); if (cp.flat) ++flats; else tris += m.indices.size() / 3; }
    CHECK(flats == 1 && tris == res.parts[1].indices.size() / 3);
    NetLotResult back; CHECK(readLotResult(*b, "lots/", back, &err)); if (!err.empty()) std::printf("    %s\n", err.c_str());
    CHECK(back.parts.size() == 2 && back.parts[0].vertices.empty() && back.parts[1].indices.size() == res.parts[1].indices.size() && back.parts[1].vertices.size() == res.parts[1].vertices.size() && back.parts[1].materialIndex == 5);
    CHECK(back.flatParts.size() == 1 && back.flatParts[0].indices.size() == a.indices.size() && back.flatParts[0].materialIndex == 3);
    NetLotResult noParts; CHECK(readLotResult(*b, "lots/", noParts, &err, false)); CHECK(noParts.parts.size() == 2 && noParts.parts[1].vertices.empty() && noParts.lots.empty());
    // whole layout (cell 0) still lists no cell parts
    BundleWriter w0; w0.openMemory(); writeLotResult(w0, "lots/", res); nlohmann::json m0 = nlohmann::json::object(); CHECK(w0.finish(m0, &err));
    std::unique_ptr<Bundle> b0 = Bundle::fromMemory(w0.memory(), &err); CHECK(b0 != nullptr); if (!b0) return;
    std::vector<LotCellPart> none; CHECK(lotCellSize(*b0, "lots/") == 0.0 && listLotCellParts(*b0, "lots/", none, &err) && none.empty());
}
