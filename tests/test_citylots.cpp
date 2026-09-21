// The lattice city's lot pre-pass as a bundle producer (ADR-0084, milestone C): which levels it takes,
// what its key follows, and that a bake grows the same city a direct grow does — parts split per render
// cell carrying exactly the grown triangles. city.json is the subject: small, terrain, buildLots.
#include "test_framework.h"

#include "../src/engine/bundle/bake.h"
#include "../src/engine/bundle/bundle.h"
#include "../src/engine/city_prepass.h"
#include "../src/engine/lot_grow_setup.h"
#include "../src/engine/level_params.h"
#include "../src/engine/procgen/city/architect.h"
#include "../src/engine/procgen/city/citylots_producer.h"
#include "../src/engine/procgen/city/lot_cache.h"

#include <cstdio>
#include <filesystem>

using namespace engine;

namespace {
// Level paths are CWD-relative (like the viewer), so every case runs from the source tree.
struct AtSourceDir {
    std::filesystem::path before = std::filesystem::current_path();
    AtSourceDir() { std::filesystem::current_path(RT_SOURCE_DIR); }
    ~AtSourceDir() { std::error_code ec; std::filesystem::current_path(before, ec); }
};

bundle::LevelInputs cityInputs() {
    bundle::LevelInputs in; std::string err;
    CHECK(bundle::loadLevelInputs("assets/levels/city.json", in, &err));
    if (!err.empty()) std::printf("    %s\n", err.c_str());
    return in;
}
}  // namespace

TEST_CASE(citylots_applies_only_where_the_json_determines_the_lot_prepass) {
    AtSourceDir here; registerCityLotsProducer();
    const bundle::BundleProducer* p = bundle::findProducer(kCityLotsProducerName);
    CHECK(p != nullptr); if (!p) return;
    const bundle::LevelInputs city = cityInputs();
    CHECK(p->applies(city));

    // A Lua script pre-pass shapes the ground at load: not reproducible from the JSON, so refused.
    bundle::LevelInputs scripted = city;
    scripted.level["entities"].push_back({{"shape", "script"}, {"script", "whatever.lua"}});
    CHECK(!p->applies(scripted));

    // A recipe that plans corridor freeways is solved and baked by the loader: refused.
    bundle::LevelInputs corridor = city;
    for (nlohmann::json& e : corridor.level["entities"])
        if (e.value("shape", std::string()) == "road" && e.contains("road") && e["road"].contains("generate"))
            e["road"]["generate"]["corridor_freeways"] = true;
    CHECK(!p->applies(corridor));

    // No terrain, no lot pre-pass; no lots wanted, nothing to cache.
    bundle::LevelInputs flat = city; flat.level.erase("terrain");
    CHECK(!p->applies(flat));
    bundle::LevelInputs noLots = city; noLots.level["citysim"]["buildLots"] = false; noLots.level["citysim"]["planOnly"] = false;
    CHECK(!p->applies(noLots));
}

TEST_CASE(citylots_identity_follows_what_the_city_is_grown_from) {
    AtSourceDir here; registerCityLotsProducer();
    const bundle::BundleProducer* p = bundle::findProducer(kCityLotsProducerName);
    CHECK(p != nullptr); if (!p) return;
    const bundle::LevelInputs base = cityInputs();
    const uint64_t k0 = p->identity(base).key;
    CHECK(k0 == p->identity(base).key);   // deterministic

    // The water block moves the recipe's sea level (propagateWaterSeaLevel) — the input behind the
    // coast_city divergence — so it must move the key.
    bundle::LevelInputs water = base; water.level["water"] = {{"seaLevel", 3.5}};
    CHECK(p->identity(water).key != k0);
    bundle::LevelInputs cs = base; cs.level["citysim"]["seed"] = base.level["citysim"].value("seed", 0) + 1;
    CHECK(p->identity(cs).key != k0);
    bundle::LevelInputs terrain = base; terrain.level["terrain"]["seed"] = base.level["terrain"].value("seed", 0u) + 1u;
    CHECK(p->identity(terrain).key != k0);

    // Presentation the lot pass never reads leaves the key alone, so a lighting tweak is still a hit.
    bundle::LevelInputs lit = base; lit.level["lighting"] = {{"sunIntensity", 7.0}};
    CHECK(p->identity(lit).key == k0);
}

TEST_CASE(citylots_bake_grows_the_same_city_as_a_direct_grow) {
    using namespace engine::lotcache;
    AtSourceDir here; registerCityLotsProducer();
    const bundle::LevelInputs in = cityInputs();

    // The direct grow, twice, out of the shared pre-pass world: byte-identical sections.
    auto grow = [&]() {
        CityPrePass pre; CHECK(cityPrePassForLevel(in.level, pre));
        Vec2 spawn; const bool haveSpawn = authoredSpawnXZ(in.level, spawn);
        LotGrowSetup s = lotGrowSetupForLevel(in.level["citysim"], in.levelDir, pre.lotGround, pre.nets,
                                              pre.lotGroundWith, pre.lotMeshCell, haveSpawn ? &spawn : nullptr);
        return growLotBuildingsOnNets(pre.nets, s.lp, s.ep, s.roadClear, pre.naturalGround,
                                      pre.freewayROWOrNull(), s.wantFlat, !s.planOnly);
    };
    const NetLotResult ga = grow(), gb = grow();
    CHECK(!ga.lots.empty() && !ga.parts.empty());
    bundle::BundleWriter wa, wb; wa.openMemory(); wb.openMemory();
    writeLotResult(wa, kCityLotsSectionPrefix, ga); writeLotResult(wb, kCityLotsSectionPrefix, gb);
    CHECK(wa.sections().size() == wb.sections().size());
    size_t differing = 0;
    for (size_t i = 0; i < wa.sections().size() && i < wb.sections().size(); ++i)
        if (wa.sections()[i].name != wb.sections()[i].name || wa.sections()[i].fnv != wb.sections()[i].fnv) ++differing;
    CHECK(differing == 0);

    // The real bake, in memory, read back: the same lots and plan, parts per render cell.
    bundle::BakeRequest req; req.levelPath = "assets/levels/city.json"; req.toMemory = true; req.only = {kCityLotsProducerName};
    const bundle::BakeReport rep = bundle::bakeLevel(req);
    CHECK(rep.ok); if (!rep.ok) { std::printf("    bake: %s\n", rep.error.c_str()); return; }
    std::string err;
    std::unique_ptr<bundle::Bundle> b = bundle::Bundle::fromMemory(rep.memory, &err);
    CHECK(b != nullptr); if (!b) return;
    NetLotResult back; CHECK(readLotResult(*b, kCityLotsSectionPrefix, back, &err));
    CHECK(back.lots.size() == ga.lots.size() && back.plan.lots.size() == ga.plan.lots.size() &&
          back.plan.blocks.size() == ga.plan.blocks.size() && back.gradeFlatten.size() == ga.gradeFlatten.size());

    CHECK(lotCellSize(*b, kCityLotsSectionPrefix) > 0.0);
    std::vector<LotCellPart> cps; CHECK(listLotCellParts(*b, kCityLotsSectionPrefix, cps, &err)); CHECK(!cps.empty());
    size_t cellTris = 0, wholeTris = 0;
    for (const LotCellPart& cp : cps) { RenderMesh m; CHECK(readLotPart(*b, cp.section, m)); cellTris += m.indices.size() / 3; }
    for (const RenderMesh& m : ga.parts) wholeTris += m.indices.size() / 3;
    for (const RenderMesh& m : ga.flatParts) wholeTris += m.indices.size() / 3;
    CHECK(cellTris == wholeTris);

    const nlohmann::json entry = bundle::manifestProducer(b->manifest(), kCityLotsProducerName);
    CHECK(!entry.is_null() &&
          entry.value("key", std::string()) == bundle::hex16(bundle::findProducer(kCityLotsProducerName)->identity(in).key));
    std::printf("    %s; %zu cell parts\n",
                rep.reports.at(kCityLotsProducerName).report.value("summary", std::string()).c_str(), cps.size());
}

// DISTRICTS THE LEVEL AUTHORS (Glenn, 2026-09-20: a lane-built metro with "more
// skyscrapers like in metro_test_v2"). Hubs used to come only from a metro RECIPE,
// left on the road entity — so a city built from a baked lane graph had no districts
// at all: one radial core, and half the towers of the recipe-planned city beside it.
TEST_CASE(a_level_can_author_its_own_district_hubs) {
    const nlohmann::json cs = nlohmann::json::parse(R"({
        "downtownRadius": 420, "midtownRadius": 750,
        "districts": { "hubRadius": 260, "hubs": [
            {"at": [-150.0, 950.0],  "kind": "commercial"},
            {"at": [0.0, 0.0],       "kind": "financial"},
            {"at": [-600.0, -900.0], "kind": "residential"},
            {"at": [2000.0, 2000.0], "kind": "no-such-quarter"}
        ] } })");
    EdgeBlockParams ep;
    LotParams lp;
    readLotGrowParams(cs, ep, lp);
    CHECK(lp.hubs.size() == 4);
    if (lp.hubs.size() != 4) return;
    CHECK(lp.hubRadius == 260.0);
    CHECK(lp.innerRadius == 420.0 && lp.midRadius == 750.0);
    CHECK(lp.hubs[0].second == 1 && lp.hubs[1].second == 0 && lp.hubs[2].second == 2);
    CHECK(lp.hubs[3].second == 2);                       // an unknown quarter is housing
    // Coreness — height grading and the landmark quotas measure from here — is the
    // FINANCIAL hub, not merely the first one authored.
    CHECK(lp.center.x == 0.0 && lp.center.y == 0.0);

    // And the zoning those hubs describe: downtown is the financial hub's own disc,
    // a commercial collar sits around a flavoured hub, and the far corner is housing.
    DistrictMap dm;
    dm.hubs = lp.hubs;
    dm.hubRadius = lp.hubRadius;
    dm.innerRadius = lp.innerRadius;
    dm.midRadius = lp.midRadius;
    CHECK(dm.tagAt(Vec2(0, 0)) == DistrictTag::Financial);
    CHECK(dm.tagAt(Vec2(300, 0)) == DistrictTag::Financial);        // inside downtownRadius
    CHECK(dm.tagAt(Vec2(2000, 2000)) == DistrictTag::Residential);  // the unknown quarter, housing
    CHECK(dm.tagAt(Vec2(-150, 950)) == DistrictTag::Commercial);    // the flavoured hub
    CHECK(dm.tagAt(Vec2(-600, -900)) == DistrictTag::Residential);

    // A level that authors nothing is unchanged: no hubs, the radial reading.
    LotParams plain;
    EdgeBlockParams ep2;
    readLotGrowParams(nlohmann::json::parse(R"({"downtownRadius": 300})"), ep2, plain);
    CHECK(plain.hubs.empty());
    CHECK(plain.center.x == 0.0 && plain.center.y == 0.0);
}
