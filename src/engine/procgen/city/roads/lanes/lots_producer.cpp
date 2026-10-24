#include "engine/procgen/city/roads/lanes/lots_producer.h"

#include <set>

#include "engine/city_grow.h"
#include "engine/level_params.h"   // readTerrainParams, levelDrawnGroundCell, readErodedBase
#include "engine/lot_grow_setup.h"
#include "engine/procgen/city/lot_cache.h"
#include "engine/procgen/city/roads/lanes/block_audit.h"
#include "engine/procgen/city/roads/lanes/city_producer.h"
#include "engine/procgen/city/roads/lanes/terrain_recipe.h"
#include "engine/script_assets.h"
#include "log.h"

#include <chrono>
#include <memory>
#include <thread>
#include <algorithm>
#include <cmath>

namespace engine {
namespace roads::lanes {

// Bump whenever the lot pass's output changes for the same inputs (the key cannot see code).
const char* const kLotsBuildTag = "2026-10-23.2";   // party walls blank only as high as the neighbour against them (ADR-0227); history: git log -G kLotsBuildTag
//   // .2-.4: soft foliage (emitSoftBox hedges, bushes, beds)   // 2026-09-24.1: grows on terrain, on the loader's ground (laneErodedBase + lotGroundFor, ADR-0095);   // ground-relative (draped) dressing slots; blocks behind the drawn sidewalk; door walks reach it   // 2026-09-22.1: ONE grow (engine::growCity) — the bake gets the streets and the paved band the loader always had

namespace {
using bundle::BinReader;

double secondsSince(const std::chrono::steady_clock::time_point& t) { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count(); }

// A road entity whose lots belong to the OTHER pipeline (the lattice's terrain pre-pass).
// A shape:"road" entity that names the lanes builder is this very city (ADR-0089) and
// disqualifies nothing — that is the whole point of the second spelling.
// A Lua script entity that grades the terrain before the lots grow (shape:"script",
// onTerrain): its footprints join the ground only the loader assembles.
bool hasTerrainScripts(const nlohmann::json& level) {
    for (const nlohmann::json& e : level.value("entities", nlohmann::json::array()))
        if (e.is_object() && e.value("shape", std::string()) == "script" && e.value("onTerrain", false)) return true;
    return false;
}

bool hasRoadEntities(const nlohmann::json& level) {
    std::set<int> city;
    for (const CityEntity& c : cityEntities(level)) city.insert(c.entityIndex);
    int i = -1;
    for (const nlohmann::json& e : level.value("entities", nlohmann::json::array())) {
        ++i;
        if (e.is_object() && e.value("shape", std::string()) == "road" && city.count(i) == 0) return true;
    }
    return false;
}
bool lotsWanted(const nlohmann::json& level) {
    const nlohmann::json cs = level.value("citysim", nlohmann::json::object());
    return cs.is_object() && (cs.value("buildLots", false) || cs.value("planOnly", false));
}

class LotsProducer : public bundle::BundleProducer {
public:
    std::string name() const override { return kLotsProducerName; }
    bool applies(const bundle::LevelInputs& in) const override {
        // On terrain the lots grow on the loader's ground; this producer builds the SAME ground from
        // the level JSON and the city's grid (laneErodedBase + lotGroundFor, ADR-0095) -- unless a Lua
        // pre-pass shapes the terrain, which only the loader can run.
        return !cityEntities(in.level).empty() && lotsWanted(in.level) && !hasRoadEntities(in.level) &&
               !hasTerrainScripts(in.level);
    }
    double weight() const override { return 35.0; }

    bundle::ProducerIdentity identity(const bundle::LevelInputs& in) const override {
        bundle::ProducerIdentity id; id.tag = kLotsBuildTag;
#ifdef RT_ENABLE_SCRIPTING
        const int scripting = 1;
#else
        const int scripting = 0;
#endif
        uint64_t k = bundle::fnv1aStr(std::string(kLotsProducerName) + "|" + std::to_string(lotcache::kLotsFormatVersion) + "|" + kLotsBuildTag + "|real" + std::to_string(sizeof(Real)) + "|scripting" + std::to_string(scripting));
        if (const bundle::BundleProducer* city = bundle::findProducer(kCityProducerName)) {
            const bundle::ProducerIdentity c = city->identity(in);
            k = bundle::fnv1a(&c.key, sizeof(c.key), k); id.inputs = c.inputs;
        } else k = bundle::fnv1aStr("nocity", k);
        k = bundle::fnv1aStr(in.level.value("citysim", nlohmann::json::object()).dump(), k);
        // The ground the lots stand on: the terrain and water blocks, and the cell it is drawn at -- the terrain
        // without its forest, which the loader places at load and the lots never read (a species mix tweak on the
        // island must not re-bake its 8.8 GB of city; a level with no forest keys exactly as before)
        nlohmann::json terrainKey = in.level.value("terrain", nlohmann::json());
        if (terrainKey.is_object()) terrainKey.erase("forest");
        k = bundle::fnv1aStr(terrainKey.dump(), k);
        k = bundle::fnv1aStr(in.level.value("water", nlohmann::json()).dump(), k);
        const double drawnCell = levelDrawnGroundCell(in.level);
        k = bundle::fnv1a(&drawnCell, sizeof(drawnCell), k);
        Vec2 spawn;
        if (authoredSpawnXZ(in.level, spawn)) { k = bundle::fnv1a(&spawn.x, sizeof(spawn.x), k); k = bundle::fnv1a(&spawn.y, sizeof(spawn.y), k); }
        else k = bundle::fnv1aStr("nospawn", k);
        for (const char* book : {"style_book.lua", "archetype_book.lua"}) {
            const std::string p = resolveScriptPath(book, in.levelDir);
            if (p.empty()) { k = bundle::fnv1aStr(std::string("nobook:") + book, k); continue; }
            bundle::InputFile f; f.path = p; uint64_t fk = bundle::kFnvOffset;
            if (bundle::fnv1aFile(p, fk, &f.bytes, &f.mtime)) { f.fnv = fk; k = bundle::fnv1a(&fk, sizeof(fk), k); } else k = bundle::fnv1aStr("missing:" + p, k);
            id.inputs.push_back(f);
        }
        id.key = k; return id;
    }

    bundle::ProducerReport produce(const bundle::LevelInputs& in, bundle::BundleWriter& out, const bundle::ProgressFn* progress) const override {
        bundle::ProducerReport rep; const auto t0 = std::chrono::steady_clock::now();
        auto fail = [&](const std::string& why) { rep.ok = false; rep.error = why; return rep; };
        auto report = [&](double local, const std::string& stage, const std::string& msg) {
            if (!progress) return true; bundle::Progress p; p.stage = stage; p.message = msg; p.fraction = local; return (*progress)(p);
        };
        const std::vector<CityEntity> ents = cityEntities(in.level);
        if (ents.empty()) return fail("no lanelab entity");
        // The city's products, out of the bundle being written (the city producer runs first: 'c' < 'l').
        const std::string pre = citySectionPrefix(ents.back().ordinal);
        LotsCityInputs city; std::vector<uint8_t> bytes;
        if (!out.readBack(pre + "meta", bytes)) return fail("city products are not in this bundle (" + pre + "meta); the city producer must run before lots");
        nlohmann::json meta; try { meta = nlohmann::json::parse(bytes.begin(), bytes.end()); } catch (const std::exception&) { return fail(pre + "meta: malformed"); }
        if (meta.value("format", 0) != kCityFormatVersion) return fail(pre + "meta: city format " + std::to_string(meta.value("format", 0)));
        city.hasTerrain = meta.value("hasTerrain", false);
        if (city.hasTerrain) {
            if (!out.readBack(pre + "ground", bytes)) return fail(pre + "ground: missing");
            BinReader r(bytes.data(), bytes.size()); if (!bundle::getHeightGrid(r, city.ground)) return fail(pre + "ground: unreadable");
        }
        if (!out.readBack(pre + "blocks/holes", bytes)) return fail(pre + "blocks/holes: missing");
        { BinReader r(bytes.data(), bytes.size()); if (!bundle::getRings(r, city.holes)) return fail(pre + "blocks/holes: unreadable"); }
        // The streets a door faces and the pavement it walks out to — what the loader hands the
        // grow when it grows in place. Without them the BAKED city's doors pointed whichever way
        // the block's plan ran, so a level looked different warm than cold.
        // at the LOT clearance widths (roads/lotnav, ADR-0115) -- roads/nav is the sim's, at travel widths,
        // and growing on it moved buildings a metre and a half closer to every street
        if (!out.readBack(pre + "roads/lotnav", bytes)) return fail(pre + "roads/lotnav: missing");
        { BinReader r(bytes.data(), bytes.size()); if (!bundle::getRoadGraph(r, city.nav)) return fail(pre + "roads/lotnav: unreadable"); }
        if (!out.readBack(pre + "roads/deck", bytes)) return fail(pre + "roads/deck: missing");
        { BinReader r(bytes.data(), bytes.size()); if (!bundle::getDeckField(r, city.deck)) return fail(pre + "roads/deck: unreadable"); }
        if (!out.readBack(pre + "roads/bands", bytes)) return fail(pre + "roads/bands: missing");
        { BinReader r(bytes.data(), bytes.size()); CurbBandAudit b; if (!bundle::getCurbBands(r, b)) return fail(pre + "roads/bands: unreadable"); city.pavedSidewalk = b.sidewalkWidth; }
        // the rail lines, when the city has any: buildings stand clear of them
        if (out.readBack(pre + "rail/tracks", bytes)) {
            try { city.rails = railTracksFromJson(nlohmann::json::parse(bytes.begin(), bytes.end())); } catch (const std::exception&) { return fail(pre + "rail/tracks: malformed"); }
        }
        if (!report(0.03, "grow", std::to_string(city.holes.size()) + " pavement holes")) return fail("cancelled");
        const auto tg = std::chrono::steady_clock::now();
        nlohmann::json counts;
        NetLotResult r = growLotsForLevel(in, city, &counts);
        rep.timings["grow"] = secondsSince(tg);
        if (!report(0.95, "write", std::to_string(r.lots.size()) + " buildings")) return fail("cancelled");
        const auto tw = std::chrono::steady_clock::now();
        lotcache::writeLotResult(out, kLotsSectionPrefix, r, cityRenderCell(in.level));   // parts per render cell: the loader never chunks at load
        rep.timings["write"] = secondsSince(tw);
        counts["cell"] = cityRenderCell(in.level); rep.report = counts; rep.seconds = secondsSince(t0);
        LOG_INFO << "[lots] " << counts.value("summary", std::string()) << " in " << rep.seconds << " s";
        return rep;
    }
};
}  // namespace

void registerLotsProducer() { bundle::registerProducer(std::make_unique<LotsProducer>()); }

std::shared_ptr<const std::function<double(double, double)>> laneErodedBase(
    const nlohmann::json& root, std::shared_ptr<const HeightGrid> grid,
    std::shared_ptr<const std::function<double(double, double)>> levelEroded, const RoadDeckField* deck) {
    auto fbTp = std::make_shared<TerrainParams>(readTerrainParams(root["terrain"]));
    fbTp->erodedBase = std::move(levelEroded);   // the fallback keeps whatever base the level had; no recursion
    auto fbNoise = std::make_shared<Noise>(root["terrain"].value("seed", 0u));
    const double bx1 = grid->x0 + grid->res * (grid->nx - 1), by1 = grid->y0 + grid->res * (grid->ny - 1);
    // THE ROADS KEEP THEIR GROUND, THE HILLS GET THEIRS (ADR-0221). The road build conforms its own copy of the ground
    // at the grid's cell (20 m on the island), and the terrain drawn from that copy threw away everything finer: the
    // weathered ground is 2.44 m, with its refine detail and rock relief, and island_8_nature's hills came out in 20 m
    // blocks of cover (Glenn: "the hills aren't smooth even after all this work to increase the terrain detail").
    // Laying the whole natural ground back under the roads was measured: the detail between the grid's nodes rose
    // through the decks (the island's poke census 3713 -> 120203 samples, worst 20 m), and by the mountain road the
    // build's copy is up to 349 m off today's peaks (it was sampled before the weathering grew them). So the grid
    // stays the ground in a CORRIDOR along the roads -- every node a deck reaches or the build changed (the cities'
    // graded ground among them) -- and the natural ground's detail (natural minus its own bilinear at the nodes)
    // fades in beyond it, all of it by kDetailFull. The roads stand on exactly the ground they were built on; the
    // rest of the island gets the full ground. (Holding every city block to the grid too took 43% of the island
    // out of the detail and moved the poke census not one sample: 4486 either way.)
    constexpr double kCorridorPad = 40.0, kDetailFull = 100.0;   // metres from the corridor: no detail / all of it
    const int nx = grid->nx, ny = grid->ny;
    const std::size_t nn = static_cast<std::size_t>(nx) * ny;
    auto natNodes = std::make_shared<std::vector<float>>(nn);
    auto detailW = std::make_shared<std::vector<float>>(nn);
    {
        std::vector<char> road(nn, 0);
        const unsigned nt = std::max(1u, std::min(16u, std::thread::hardware_concurrency()));
        std::vector<std::thread> workers;
        for (unsigned t = 0; t < nt; ++t)
            workers.emplace_back([&, t] {
                for (int j = static_cast<int>(t); j < ny; j += static_cast<int>(nt))
                    for (int i = 0; i < nx; ++i) {
                        const std::size_t k = static_cast<std::size_t>(j) * nx + i;
                        const double x = grid->x0 + i * grid->res, z = grid->y0 + j * grid->res;
                        (*natNodes)[k] = static_cast<float>(terrainHeight(*fbTp, *fbNoise, x, z));
                        road[k] = std::fabs(grid->z[k] - (*natNodes)[k]) > 0.05 || (deck && deck->coversAny(x, z, 0.75 * grid->res));
                    }
            });
        for (auto& th : workers) th.join();
        // distance to the corridor, in metres (a two-pass chamfer on the grid: 1 and sqrt 2 cells)
        std::vector<float> dist(nn, 1e9f);
        for (std::size_t k = 0; k < nn; ++k) if (road[k]) dist[k] = 0.0f;
        const float a = static_cast<float>(grid->res), b = static_cast<float>(grid->res * std::sqrt(2.0));
        auto relax = [&](int i, int j, int di, int dj, float w) {
            const int u = i + di, v = j + dj;
            if (u < 0 || v < 0 || u >= nx || v >= ny) return;
            float& d = dist[static_cast<std::size_t>(j) * nx + i];
            d = std::min(d, dist[static_cast<std::size_t>(v) * nx + u] + w);
        };
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i) { relax(i, j, -1, 0, a); relax(i, j, 0, -1, a); relax(i, j, -1, -1, b); relax(i, j, 1, -1, b); }
        for (int j = ny - 1; j >= 0; --j)
            for (int i = nx - 1; i >= 0; --i) { relax(i, j, 1, 0, a); relax(i, j, 0, 1, a); relax(i, j, 1, 1, b); relax(i, j, -1, 1, b); }
        std::size_t corridor = 0, full = 0, changed = 0, far = 0; double worst = 0.0, worstFar = 0.0;
        for (std::size_t k = 0; k < nn; ++k) {
            const double u = std::clamp((dist[k] - kCorridorPad) / (kDetailFull - kCorridorPad), 0.0, 1.0);
            (*detailW)[k] = static_cast<float>(u * u * (3 - 2 * u));
            const double c = std::fabs(grid->z[k] - (*natNodes)[k]);
            if (road[k]) ++corridor;
            if ((*detailW)[k] >= 1.0f) ++full;
            if (c > 0.05) ++changed;
            worst = std::max(worst, c);
            if ((*detailW)[k] > 0.0f && c > 1.0) { ++far; worstFar = std::max(worstFar, c); }
        }
        LOG_INFO << "[lanelab] road ground: " << nx << " x " << ny << " @ " << grid->res << " m; the build changed "
                 << changed << " nodes by > 5 cm (" << 100.0 * changed / std::max<std::size_t>(1, nn) << "%, worst " << worst
                 << " m); corridor " << corridor << " nodes (" << 100.0 * corridor / std::max<std::size_t>(1, nn)
                 << "%), full natural detail on " << 100.0 * full / std::max<std::size_t>(1, nn) << "%; " << far
                 << " nodes off the build's ground by > 1 m where detail shows (worst " << worstFar << " m)";
    }
    auto bilinear = [grid](const std::vector<float>& v, double x, double z) {
        const double fx = std::clamp((x - grid->x0) / grid->res, 0.0, grid->nx - 1.001), fz = std::clamp((z - grid->y0) / grid->res, 0.0, grid->ny - 1.001);
        const int i = static_cast<int>(fx), j = static_cast<int>(fz);
        const double u = fx - i, w = fz - j;
        auto C = [&](int p, int q) { return static_cast<double>(v[static_cast<std::size_t>(q) * grid->nx + p]); };
        return (C(i, j) * (1 - u) + C(i + 1, j) * u) * (1 - w) + (C(i, j + 1) * (1 - u) + C(i + 1, j + 1) * u) * w;
    };
    return std::make_shared<const std::function<double(double, double)>>(
        [fbTp, fbNoise, bx1, by1, grid, natNodes, detailW, bilinear](double x, double z) {
            const double band = 60.0;   // blend to the level's own terrain over the last 60 m of the grid
            const double inset = std::min(std::min(x - grid->x0, bx1 - x), std::min(z - grid->y0, by1 - z));
            if (inset <= 0.0) return terrainHeight(*fbTp, *fbNoise, x, z);
            double lab = grid->sample(x, z);
            const double w = bilinear(*detailW, x, z);
            const bool needNatural = w > 0.0 || inset < band;
            const double natural = needNatural ? terrainHeight(*fbTp, *fbNoise, x, z) : 0.0;
            if (w > 0.0) lab += w * (natural - bilinear(*natNodes, x, z));   // the detail finer than the grid
            if (inset >= band) return lab;
            const double u = inset / band, s = u * u * (3 - 2 * u);
            return natural * (1 - s) + lab * s;
        });
}

NetLotResult growLotsForLevel(const bundle::LevelInputs& in, const LotsCityInputs& city, nlohmann::json* report) {
    const nlohmann::json cs = in.level.value("citysim", nlohmann::json::object());
    // The blocks are the city's pavement holes; there are no nets. The ground: on a level with
    // terrain, the loader's -- the level's terrain with the city's grid blended in, sampled and
    // re-graded exactly as the loader's lot pass does; without, the city's own grid.
    HeightField ground;
    LotGroundWithFn groundWith;
    double groundMeshCell = 0.0;
    bool smoothGround = false;   // the earthwork field carries the streets: no block terraces
    if (city.hasTerrain) {
        auto grid = std::make_shared<HeightGrid>();
        grid->x0 = city.ground.x0; grid->y0 = city.ground.y0; grid->res = city.ground.res; grid->nx = city.ground.nx; grid->ny = city.ground.ny; grid->z = city.ground.z;
        if (in.level.contains("terrain")) {
            auto tp = std::make_shared<TerrainParams>(readTerrainParams(in.level["terrain"]));
            tp->erodedBase = laneErodedBase(in.level, grid, readErodedBase(in.level), &city.deck);
            // THE STREETS SET THE GROUND: the same earthwork field the loader installs (lanesEarthworkField),
            // over the same natural ground, so every pad is fitted to the ground the terrain will show.
            {
                auto natTp = std::make_shared<TerrainParams>(*tp);
                auto natNoise = std::make_shared<Noise>(in.level["terrain"].value("seed", 0u));
                const std::function<double(double, double)> natural = [natTp, natNoise](double x, double z) { return terrainHeight(*natTp, *natNoise, x, z); };
                tp->earthwork = lanesEarthworkField(city.deck, city.pavedSidewalk, natural, tp->earthworkParams,
                                                     in.level.contains("water") ? in.level["water"].value("seaLevel", tp->seaLevel) : tp->seaLevel);
                smoothGround = static_cast<bool>(tp->earthwork);
                if (smoothGround) {   // the loader's finish, so the pads see the ground the terrain will show
                    const std::vector<TerrainFlatten> finish = lanesStreetFinish(city.deck, lanesMinSidewalk(in.level, city.pavedSidewalk));
                    tp->flatten.insert(tp->flatten.end(), finish.begin(), finish.end());
                    rebuildFlattenIndex(*tp);
                }
            }
            const LotGround lg = lotGroundFor(tp, in.level["terrain"].value("seed", 0u));
            ground = lg.ground;
            groundWith = lg.groundWith;
            groundMeshCell = levelDrawnGroundCell(in.level);
        } else {
            ground = [grid](double x, double z) { return grid->sample(x, z); };
        }
    }
    Vec2 spawn; const bool haveSpawn = authoredSpawnXZ(in.level, spawn);
    // ONE GROW (engine/city_grow.h): the same call the loader makes, from the same city products.
    CityGrowInputs gin;
    gin.citysim = cs;
    gin.levelDir = in.levelDir;
    gin.padGround = ground;
    gin.groundWith = groundWith;
    gin.groundMeshCell = groundMeshCell;
    gin.holes = &city.holes;
    const std::vector<std::vector<Vec2>> water = levelWaterKeepOut(in.level);   // blocks stand back from rivers (ADR-0104)
    gin.water = &water;
    gin.streets = &city.nav;
    gin.pavedSidewalk = city.pavedSidewalk;
    gin.smoothGround = smoothGround;
    if (auto nf = lanesNearFreeway(city.deck, 6.0)) gin.nearFreeway = [nf](Real x, Real z) { return nf(x, z); };
    gin.trackKeepOut = lanesTrackKeepOut(city.deck, city.rails);
    if (smoothGround) {   // the pads meet the street they face (30 m: across a sidewalk and a front yard)
        auto sh = lanesStreetHeight(city.deck, 30.0);
        if (sh) gin.streetHeight = [sh](Real x, Real z, Real* y) { double d; if (!sh(x, z, &d)) return false; *y = static_cast<Real>(d); return true; };
    }
    if (haveSpawn) gin.enterableAt = &spawn;
    const auto t0 = std::chrono::steady_clock::now();
    LotGrowSetup s;
    NetLotResult r = growCity(gin, &s);
    const std::size_t blockCount = r.plan.blocks.size();
    if (report) {
        size_t units = 0; for (const LotBuilding& b : r.lots) units += b.units.size();
        size_t partTris = 0; for (const RenderMesh& m : r.parts) partTris += m.indices.size() / 3;
        const double seconds = secondsSince(t0);
        *report = {{"summary", std::to_string(blockCount) + " blocks, " + std::to_string(r.plan.lots.size()) + " lots, " + std::to_string(r.lots.size()) + " buildings, " + std::to_string(units) + " units, " + std::to_string(partTris) + " part triangles"},
                   {"blocks", blockCount}, {"lots", r.plan.lots.size()}, {"buildings", r.lots.size()}, {"units", units}, {"parts", r.parts.size()}, {"partTriangles", partTris},
                   {"flatParts", r.flatParts.size()}, {"terraces", r.gradeFlatten.size()}, {"planOnly", s.planOnly}, {"lod1", s.wantFlat && !s.planOnly}, {"hasTerrain", city.hasTerrain}, {"spawn", haveSpawn}, {"seconds", seconds}};
    }
    return r;
}

}  // namespace roads::lanes
}  // namespace engine
