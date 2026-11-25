#include "engine/procgen/city/roads/lanes/lots_producer.h"

#include <set>

#include "engine/city_grow.h"
#include "engine/level_params.h"   // readTerrainParams, levelDrawnGroundCell, readErodedBase
#include "engine/lot_grow_setup.h"
#include "engine/procgen/city/lot_cache.h"
#include "engine/procgen/city/roads/lanes/block_audit.h"
#include "engine/procgen/city/roads/lanes/city_producer.h"
#include "engine/procgen/city/roads/lanes/road_model_cache.h"   // obtainLevelRoadModel: the level's one road model
#include "engine/procgen/city/roads/lanes/terrain_recipe.h"
#include "engine/script_assets.h"
#include "log.h"

#include <chrono>
#include <fstream>
#include <map>
#ifdef __GLIBC__
#include <malloc.h>   // malloc_trim
#endif
#include <memory>
#include "../../../../../job_system.h"   // parallelRows: the shared pool (ADR-0288)
#include <algorithm>
#include <cmath>

namespace engine {
namespace roads::lanes {

// Bump whenever the lot pass's output changes for the same inputs (the key cannot see code).
const char* const kLotsBuildTag = "2026-11-25.2";   // grown, cut and written a 1 km tile at a time, each tile kept from the last bake when its key is the same: every lot cell changes once (ADR-0504); .1 every block's and lot's dice from its own outline, the blocks taken in that order: every lot changes once (ADR-0503); 2026-11-22.1 two new trades, the barber's and the gym: every street's shops dealt again (Glenn's word); .20.2 a lot's paths stated as walkways with their ends, read from the planned road model (ADR-0476); .1 every building and door records the street it fronts, its station and kerb (ADR-0474); 2026-11-14.1 the lot pass reads the road model's streets and the drawn deck, not a copy of the road graph (ADR-0430); 2026-11-06.3 the furniture upgraded with soft shapes (ADR-0347); .2 the furniture modelled by its Lua recipes (ADR-0346); .1 one annulus / band / fill / skirt for every disc, ring and paved plate (ADR-0345); 2026-11-03.1 a landmark is never planned onto a lot that is already a store, campus, depot or car park (ADR-0314); .3 beach car parks: a curved lot's open frame from its simplified outline; .2 whole blocks of homes rise, courtyard blocks uncapped (ADR-0299); denser housing (ADR-0298); history: git log -G kLotsBuildTag
//   // .2-.4: soft foliage (emitSoftBox hedges, bushes, beds)   // 2026-09-24.1: grows on terrain, on the loader's ground (laneErodedBase + lotGroundFor, ADR-0095);   // ground-relative (draped) dressing slots; blocks behind the drawn sidewalk; door walks reach it   // 2026-09-22.1: ONE grow (engine::growCity) — the bake gets the streets and the paved band the loader always had

namespace {

// THE PROCESS'S MEMORY NOW AND AT ITS PEAK SO FAR (GB; Linux /proc, 0 elsewhere): logged at the lots stage's
// boundaries so a bake's report says which part of the stage holds the most (#317: the island's lots, 37.6 GB).
void memoryNow(double& rssGb, double& peakGb) {
    rssGb = peakGb = 0;
    std::ifstream f("/proc/self/status");
    std::string key;
    while (f >> key) {
        double kb = 0;
        if (key == "VmRSS:" && f >> kb) rssGb = kb / (1024.0 * 1024.0);
        else if (key == "VmHWM:" && f >> kb) peakGb = kb / (1024.0 * 1024.0);
        std::getline(f, key);
    }
}

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
    const LotCitysim cs = LotCitysim::ofLevel(level);
    return cs.value("buildLots", false) || cs.value("planOnly", false);
}

// EVERYTHING THE LOT PASS READS BUT THE CITY AND ITS STREETS (the producer's key adds those; a tile's key adds the
// city it reaches, ADR-0504): the tag and format, the city sim's block as the pass reads it, the terrain and water, the
// cell the ground is drawn at, the spawn, the style and archetype books
uint64_t lotsInputsKey(const bundle::LevelInputs& in, std::vector<bundle::InputFile>* inputs) {
#ifdef RT_ENABLE_SCRIPTING
    const int scripting = 1;
#else
    const int scripting = 0;
#endif
    uint64_t k = bundle::fnv1aStr(std::string(kLotsProducerName) + "|" + std::to_string(::engine::lotcache::kLotsFormatVersion) + "|" + kLotsBuildTag + "|real" + std::to_string(sizeof(Real)) + "|scripting" + std::to_string(scripting));
    // THE CITY SIM'S BLOCK AS THE LOT PASS READS IT (ADR-0457): exactly the fields it reads (lot_citysim.h
    // kLotCitysimFields), the same LotCitysim the grow is handed below; the fields only the running sim reads
    // (people, driving, crowds, buses, ...) re-bake nothing.
    k = bundle::fnv1aStr("lotcitysim|" + LotCitysim::ofLevel(in.level).keyText(), k);
    // The ground the lots stand on: the terrain and water blocks, and the cell it is drawn at -- the terrain
    // without its forest, which the loader places at load and the lots never read (a species mix tweak on the
    // island must not re-bake its 8.8 GB of city; a level with no forest keys exactly as before)
    nlohmann::json terrainKey = in.level.value("terrain", nlohmann::json());
    if (terrainKey.is_object()) terrainKey.erase("forest");
    // (nor the cover's heath, ADR-0331: it paints and plants, the lots never see it -- erased only where it is, so
    // every level without one keeps its key)
    if (terrainKey.is_object() && terrainKey.contains("groundCover") && terrainKey["groundCover"].is_object())
        terrainKey["groundCover"].erase("heath");
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
        if (inputs) inputs->push_back(f);
    }
    return k;
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
        uint64_t k = lotsInputsKey(in, &id.inputs);
        if (const bundle::BundleProducer* city = bundle::findProducer(kCityProducerName)) {
            const bundle::ProducerIdentity c = city->identity(in);
            k = bundle::fnv1a(&c.key, sizeof(c.key), k); id.inputs.insert(id.inputs.begin(), c.inputs.begin(), c.inputs.end());
        } else k = bundle::fnv1aStr("nocity", k);
        // The streets the lots front, by id (lotRoadsKeyText): the city's key leaves the road file's streets and
        // junctions out (no road mesh reads them), and the lot pass reads them -- editing a street re-grows the lots.
        if (const std::vector<CityEntity> ents = cityEntities(in.level); !ents.empty()) {
            const CityEntity& e = ents.back();
            nlohmann::json spec = e.block;
            if (!e.inlineGraph) {
                std::ifstream gf(e.graphPath);
                try { gf >> spec; } catch (const std::exception&) { spec = nlohmann::json(); }
            }
            k = bundle::fnv1aStr("lotroads|" + lotRoadsKeyText(spec), k);
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
        // The pavement a door walks out to and the drawn deck — what the loader hands the grow when it grows in
        // place. (The streets a door faces come from the level's one road model: growLotsForLevel, lotRoadInputs.)
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
        // THE LAST BAKE'S LOTS, when they were grown by this pass (the same tag): its unchanged tiles are kept (ADR-0504)
        std::unique_ptr<bundle::Bundle> previous;
        if (!in.reuseFrom.empty()) {
            std::string err;
            previous = bundle::Bundle::open(in.reuseFrom + "/" + bundle::kBundleFile, &err);
            if (previous && bundle::manifestProducer(previous->manifest(), kLotsProducerName).value("tag", std::string()) != kLotsBuildTag) previous.reset();
        }
        ::engine::lotcache::TileCellWriter cells(out, kLotsSectionPrefix, cityRenderCell(in.level), previous.get());
        NetLotResult r = growLotsForLevel(in, city, &counts, &cells);
        rep.timings["grow"] = secondsSince(tg);
#ifdef __GLIBC__
        // the memory the growing freed, handed back before writing: the pool's threads each keep their own heap, and
        // what a lot grown on one of them freed stays in it otherwise (#317 step 5)
        malloc_trim(0);
#endif
        double rss = 0, peak = 0;
        memoryNow(rss, peak);
        rep.timings["grownRssGb"] = rss;
        rep.timings["grownPeakGb"] = peak;
        LOG_INFO << "[lots] grown: memory " << rss << " GB, the peak so far " << peak << " GB";
        if (!report(0.95, "write", std::to_string(r.lots.size()) + " buildings")) return fail("cancelled");
        const auto tw = std::chrono::steady_clock::now();
        cells.finish(r);   // (the cells are written already, tile by tile; the lots, plan, grades, tiles and meta now)
        rep.timings["write"] = secondsSince(tw);
        rep.timings["tilesGrown"] = cells.grownTiles();
        rep.timings["tilesKept"] = cells.keptTiles();
        LOG_INFO << "[lots] tiles: " << cells.grownTiles() << " grown, " << cells.keptTiles() << " kept from "
                 << (previous ? in.reuseFrom : std::string("(no last bake)"));
        memoryNow(rss, peak);
        rep.timings["writtenPeakGb"] = peak;
        LOG_INFO << "[lots] written: memory " << rss << " GB, the peak so far " << peak << " GB";
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
        parallelRows(ny, [&](int j) {
            for (int i = 0; i < nx; ++i) {
                const std::size_t k = static_cast<std::size_t>(j) * nx + i;
                const double x = grid->x0 + i * grid->res, z = grid->y0 + j * grid->res;
                (*natNodes)[k] = static_cast<float>(terrainHeight(*fbTp, *fbNoise, x, z));
                road[k] = std::fabs(grid->z[k] - (*natNodes)[k]) > 0.05 || (deck && deck->coversAny(x, z, 0.75 * grid->res));
            }
        });
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
        return bilinearGrid(grid->nx, grid->ny, (x - grid->x0) / grid->res, (z - grid->y0) / grid->res,
                            [&](int p, int q) { return static_cast<double>(v[static_cast<std::size_t>(q) * grid->nx + p]); });
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

LotsCityInputs lotsCityInputs(const CityProducts& p) {
    LotsCityInputs city;
    city.hasTerrain = p.hasTerrain;
    city.ground = p.ground;
    city.holes = p.holes;
    city.pavedSidewalk = p.bands.sidewalkWidth;
    city.deck = p.deck;
    city.rails = p.rails;
    return city;
}

int lotsCity(const std::vector<std::shared_ptr<RoadModel>>& cities) { return static_cast<int>(cities.size()) - 1; }

LotRoadInputs lotRoadInputs(const std::vector<std::shared_ptr<RoadModel>>& cities, const RoadDeckField& deck) {
    const int c = lotsCity(cities);
    return lotRoadInputs(c < 0 || !cities[static_cast<std::size_t>(c)] ? RoadModel() : *cities[static_cast<std::size_t>(c)], deck);
}

std::vector<WalkwayProblem> stateLotWalkways(RoadModel& built, const std::vector<LotBuilding>& lots) {
    std::vector<ModelWalkway> all;
    for (const LotBuilding& lb : lots) all.insert(all.end(), lb.walkways.begin(), lb.walkways.end());
    if (all.empty()) return {};
    return stateWalkways(built, std::move(all));
}

std::vector<WalkwayProblem> stateLotWalkways(const std::vector<std::shared_ptr<RoadModel>>& cities, const std::vector<LotBuilding>& lots) {
    const int c = lotsCity(cities);
    if (c >= 0 && cities[static_cast<std::size_t>(c)]) return stateLotWalkways(*cities[static_cast<std::size_t>(c)], lots);
    bool any = false;
    for (const LotBuilding& lb : lots) any = any || !lb.walkways.empty();
    if (!any) return {};
    WalkwayProblem p;
    p.fault = WalkwayFault::NoLaneLevel;
    p.why = std::string(walkwayFaultWord(p.fault)) + ": the level has no road model for the city its lots grow on";
    return {p};
}

std::string describeWalkwayProblems(const std::vector<WalkwayProblem>& problems) {
    std::map<std::string, int> byKind;
    for (const WalkwayProblem& p : problems) ++byKind[walkwayFaultWord(p.fault)];
    std::string out;
    for (const std::pair<const std::string, int>& k : byKind) out += (out.empty() ? "" : "; ") + std::to_string(k.second) + " " + k.first;
    return out;
}

LotRoadInputs lotRoadInputs(const RoadModel& planned, const RoadDeckField& deck) {
    LotRoadInputs in;
    // THE MODEL'S OWN LIST of the roads a search by position goes over (RoadModel::roadLines: each street, then each
    // junction's inside stretches; no track, no footway), in its order -- the list and the order RoadModel::frontageAt
    // searches, so "the street nearest this door" is one answer for the lot pass and for the model (ADR-0439).
    for (const RoadLine& l : planned.roadLines()) {
        const ModelSection& section = *l.section;
        LotRoad r;
        r.id = l.street.valid() ? planned.streets[l.street.index].id : planned.junctions[l.junction.index].id;
        r.line = planned.centreLine(l.line);
        if (l.line.end() <= planned.lineStations.size())   // its stations as the model states them (ADR-0474)
            r.stations.assign(planned.lineStations.begin() + l.line.first, planned.lineStations.begin() + l.line.end());
        r.insideJunction = !l.street.valid();
        if (l.street.valid()) {   // where its sidewalks run: from kerb end to kerb end (the model's kerbEndStation, ADR-0476)
            const ModelStreet& S = planned.streets[l.street.index];
            r.sidewalkFrom = S.from;
            r.sidewalkTo = S.to;
            for (int end = 0; end < 2; ++end) {
                const int arm = planned.armAt(l.street, end);
                const Real at = arm >= 0 ? kerbEndStation(planned, S.junctions[static_cast<std::size_t>(end)], static_cast<uint32_t>(arm)) : kNotStated;
                if (stated(at)) (end == 0 ? r.sidewalkFrom : r.sidewalkTo) = at;
            }
        }
        r.halfWidth = section.carriagewayHalfWidth();   // (as the deck is drawn: city_producer.cpp deckFromResult)
        r.sidewalk = section.sidewalk;
        r.klass = section.klass;
        in.roads.push_back(std::move(r));
    }
    std::shared_ptr<RoadDeckField> drawn = std::make_shared<RoadDeckField>(deck);
    drawn->buildIndex();
    in.deck = std::move(drawn);
    return in;
}

std::string lotRoadsKeyText(const nlohmann::json& roadFile) {
    std::string text;
    auto stretch = [&](const std::string& id, const nlohmann::json& s) {
        if (!s.is_object()) return;
        text += id + "|" + s.value("road", std::string()) + "|" + s.value("from", nlohmann::json()).dump() + "|" + s.value("to", nlohmann::json()).dump() + ";";
    };
    if (!roadFile.is_object()) return text;
    if (roadFile.contains("streets") && roadFile["streets"].is_array())
        for (const nlohmann::json& s : roadFile["streets"]) stretch(s.is_object() ? s.value("id", std::string()) : std::string(), s);
    if (roadFile.contains("junctions") && roadFile["junctions"].is_array())
        for (const nlohmann::json& j : roadFile["junctions"]) {
            if (!j.is_object() || !j.contains("inside") || !j["inside"].is_array()) continue;
            for (const nlohmann::json& s : j["inside"]) stretch(j.value("id", std::string()), s);
        }
    return text;
}

NetLotResult growLotsForLevel(const bundle::LevelInputs& in, const LotsCityInputs& city, nlohmann::json* report, ::engine::lotcache::TileCellWriter* cells) {
    const LotCitysim cs = LotCitysim::ofLevel(in.level);   // (what the key hashes, and nothing else: ADR-0457)
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
    // the streets by id, from the level's PLANNED road model (read from its road file: the lots grow before the built
    // model is assembled, ADR-0476), and the drawn deck
    std::vector<std::string> plannedNotes;
    const LotRoadInputs roads = lotRoadInputs(levelPlannedRoadModels(in, &plannedNotes), city.deck);
    for (const std::string& n : plannedNotes) LOG_WARN << "[lots] the level's road model: " << n;
    gin.roads = &roads;
    gin.pavedSidewalk = city.pavedSidewalk;
    gin.smoothGround = smoothGround;
    if (auto nf = lanesNearFreeway(city.deck, 6.0)) gin.nearFreeway = [nf](Real x, Real z) { return nf(x, z); };
    gin.trackKeepOut = lanesTrackKeepOut(city.deck, city.rails);
    if (smoothGround) {   // the pads meet the street they face (30 m: across a sidewalk and a front yard)
        auto sh = lanesStreetHeight(city.deck, 30.0);
        if (sh) gin.streetHeight = [sh](Real x, Real z, Real* y) { double d; if (!sh(x, z, &d)) return false; *y = static_cast<Real>(d); return true; };
    }
    if (haveSpawn) gin.enterableAt = &spawn;
    // THE TILES' HOST (ADR-0504): each tile's key is the hash of its slice of the plan, of everything the pass reads but
    // the city (lotsInputsKey), and of the city where the tile reaches -- kTileKeyHalo past its reach: the ground grid's
    // nodes, the drawn deck's spines and pads, the streets it may front, the rails and the water
    if (cells) {
        constexpr double kTileKeyHalo = 200.0;
        const uint64_t inputsKey = lotsInputsKey(in, nullptr);
        std::shared_ptr<LotTileHooks> hooks = std::make_shared<LotTileHooks>();
        hooks->key = [cells, inputsKey, &city, &roads, &water](const LotTilePlan& t) {
            const double x0 = t.lo.x - kTileKeyHalo, x1 = t.hi.x + kTileKeyHalo, z0 = t.lo.y - kTileKeyHalo, z1 = t.hi.y + kTileKeyHalo;
            auto meets = [&](const std::vector<Vec2>& pts) {
                for (const Vec2& v : pts) if (v.x >= x0 && v.x <= x1 && v.y >= z0 && v.y <= z1) return true;
                double a = 1e300, b = -1e300, c = 1e300, d = -1e300;
                for (const Vec2& v : pts) { a = std::min(a, v.x); b = std::max(b, v.x); c = std::min(c, v.y); d = std::max(d, v.y); }
                return !pts.empty() && b >= x0 && a <= x1 && d >= z0 && c <= z1;
            };
            uint64_t k = bundle::fnv1a(&inputsKey, sizeof inputsKey, bundle::kFnvOffset);
            k = bundle::fnv1a(&t.plan, sizeof t.plan, k);
            auto num = [&k](double v) { k = bundle::fnv1a(&v, sizeof v, k); };
            num(city.pavedSidewalk);
            if (city.hasTerrain) {   // the ground grid's nodes under the reach
                const bundle::HeightGridBlob& g = city.ground;
                num(g.x0); num(g.y0); num(g.res); num(static_cast<double>(g.nx)); num(static_cast<double>(g.ny));
                const int i0 = std::max(0, static_cast<int>(std::floor((x0 - g.x0) / g.res))), i1 = std::min(g.nx - 1, static_cast<int>(std::ceil((x1 - g.x0) / g.res)));
                const int j0 = std::max(0, static_cast<int>(std::floor((z0 - g.y0) / g.res))), j1 = std::min(g.ny - 1, static_cast<int>(std::ceil((z1 - g.y0) / g.res)));
                for (int j = j0; j <= j1; ++j)
                    for (int i = i0; i <= i1; ++i) num(static_cast<double>(g.z[static_cast<std::size_t>(j) * static_cast<std::size_t>(g.nx) + static_cast<std::size_t>(i)]));
            }
            for (const UnionSpine& sp : city.deck.spines) {   // the drawn deck
                if (!meets(sp.points)) continue;
                for (const Vec2& v : sp.points) { num(v.x); num(v.y); }
                num(sp.halfWidth); num(sp.closed ? 1 : 0); num(static_cast<double>(sp.klass)); num(sp.access); num(sp.accessBack);
                num(sp.authoredDeck ? 1 : 0); num(sp.layer); num(static_cast<double>(sp.use)); num(sp.travelEdgeFrac);
                for (const std::vector<double>* a : {&sp.yAbs, &sp.hw, &sp.crossSlope, &sp.yLeft, &sp.yRight}) { num(static_cast<double>(a->size())); for (double v : *a) num(v); }
            }
            for (const RoadDeckField::Tri& tri : city.deck.pads) {
                if (!meets({Vec2(tri.a.x, tri.a.z), Vec2(tri.b.x, tri.b.z), Vec2(tri.c.x, tri.c.z)})) continue;
                for (const Vec3& v : {tri.a, tri.b, tri.c}) { num(v.x); num(v.y); num(v.z); }
            }
            for (const LotRoad& rd : roads.roads) {   // the streets it may front
                if (!meets(rd.line)) continue;
                k = bundle::fnv1aStr(rd.id, k);
                for (const Vec2& v : rd.line) { num(v.x); num(v.y); }
                for (Real v : rd.stations) num(v);
                num(rd.insideJunction ? 1 : 0); num(rd.sidewalkFrom); num(rd.sidewalkTo); num(rd.halfWidth); num(rd.sidewalk);
                num(static_cast<double>(rd.klass)); num(rd.overGround ? 1 : 0);
            }
            for (const RailTrack& rt : city.rails) {
                std::vector<Vec2> plan; for (const Vec3& v : rt.pts) plan.push_back(Vec2(v.x, v.z));
                if (meets(plan)) k = bundle::fnv1aStr(railTracksToJson({rt}).dump(), k);
            }
            for (const std::vector<Vec2>& wpoly : water) if (meets(wpoly)) for (const Vec2& v : wpoly) { num(v.x); num(v.y); }
            return k;
        };
        hooks->keep = [cells](const LotTilePlan& t, uint64_t key, LotTile& kept) { return cells->keep(t, key, kept); };
        hooks->done = [cells](LotTile& t) { cells->done(t); };
        gin.tiles = hooks;
    }
    const auto t0 = std::chrono::steady_clock::now();
    LotGrowSetup s;
    NetLotResult r = growCity(gin, &s);
    const std::size_t blockCount = r.plan.blocks.size();
    // THE PATHS STATED ON THE BUILT MODEL (ADR-0476), here only to say what does not land: the loader states them again
    // on the model it publishes (the saved road model holds none)
    std::size_t walkwayFaults = 0;
    {
        const LevelBuiltRoads levelRoads = obtainLevelRoadModel(in, levelRoadModelCacheDir());
        for (const std::string& n : levelRoads.notes) LOG_WARN << "[lots] the level's road model: " << n;
        const std::vector<WalkwayProblem> faults = stateLotWalkways(levelRoads.cities, r.lots);
        walkwayFaults = faults.size();
        if (!faults.empty()) LOG_WARN << "[lots] paths that do not land on the road model: " << describeWalkwayProblems(faults);
    }
    if (report) {
        size_t units = 0; for (const LotBuilding& b : r.lots) units += b.units.size();
        size_t partTris = 0; for (const RenderMesh& m : r.parts) partTris += m.indices.size() / 3;
        const double seconds = secondsSince(t0);
        *report = {{"summary", std::to_string(blockCount) + " blocks, " + std::to_string(r.plan.lots.size()) + " lots, " + std::to_string(r.lots.size()) + " buildings, " + std::to_string(units) + " units, " + std::to_string(partTris) + " part triangles"},
                   {"blocks", blockCount}, {"lots", r.plan.lots.size()}, {"buildings", r.lots.size()}, {"units", units}, {"parts", r.parts.size()}, {"partTriangles", partTris},
                   {"flatParts", r.flatParts.size()}, {"terraces", r.gradeFlatten.size()}, {"planOnly", s.planOnly}, {"lod1", s.wantFlat && !s.planOnly}, {"hasTerrain", city.hasTerrain}, {"spawn", haveSpawn}, {"seconds", seconds}, {"walkwayFaults", walkwayFaults}};
    }
    return r;
}

}  // namespace roads::lanes
}  // namespace engine
