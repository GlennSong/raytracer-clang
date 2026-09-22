#include "engine/procgen/city/roads/lanes/lots_producer.h"

#include <set>

#include "engine/lot_grow_setup.h"
#include "engine/procgen/city/lot_cache.h"
#include "engine/procgen/city/roads/lanes/block_audit.h"
#include "engine/procgen/city/roads/lanes/city_producer.h"
#include "engine/procgen/city/roads/lanes/terrain_recipe.h"
#include "engine/script_assets.h"
#include "log.h"

#include <chrono>
#include <memory>

namespace engine {
namespace roads::lanes {

// Bump whenever the lot pass's output changes for the same inputs (the key cannot see code).
const char* const kLotsBuildTag = "2026-09-21.17";   // ground-relative (draped) dressing slots; blocks behind the drawn sidewalk; door walks reach it

namespace {
using bundle::BinReader;

double secondsSince(const std::chrono::steady_clock::time_point& t) { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count(); }

// A road entity whose lots belong to the OTHER pipeline (the lattice's terrain pre-pass).
// A shape:"road" entity that names the lanes builder is this very city (ADR-0089) and
// disqualifies nothing — that is the whole point of the second spelling.
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
        return !cityEntities(in.level).empty() && lotsWanted(in.level) && !in.level.contains("terrain") && !hasRoadEntities(in.level);
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

NetLotResult growLotsForLevel(const bundle::LevelInputs& in, const LotsCityInputs& city, nlohmann::json* report) {
    const nlohmann::json cs = in.level.value("citysim", nlohmann::json::object());
    const double sidewalk = cs.is_object() ? cs.value("sidewalk", 4.0) : 4.0;
    // The loader's rules, one for one (LevelLoader::loadLanesEntity + growCityLots's lanelab branch):
    // the lab's grid IS the ground, the blocks are the pavement holes (sidewalks included) inset by a margin, no nets.
    HeightField ground;
    if (city.hasTerrain) {
        auto grid = std::make_shared<HeightGrid>();
        grid->x0 = city.ground.x0; grid->y0 = city.ground.y0; grid->res = city.ground.res; grid->nx = city.ground.nx; grid->ny = city.ground.ny; grid->z = city.ground.z;
        ground = [grid](double x, double z) { return grid->sample(x, z); };
    }
    const std::vector<Poly2> blocks = blocksFromHoles(city.holes, 1.5, kBlockMarginBehindSidewalk, kMinBlockWidth);
    Vec2 spawn; const bool haveSpawn = authoredSpawnXZ(in.level, spawn);
    LotGrowSetup s = lotGrowSetupForLevel(cs, in.levelDir, ground, {}, nullptr, 0.0, haveSpawn ? &spawn : nullptr);
    s.lp.roadMargin = 0;   // the lab's blocks already begin behind the drawn sidewalk
    s.lp.padFeatherInside = static_cast<Real>(lanesPadFalloff(sidewalk));   // plates stand on the flat pad (loader rule)
    s.lp.sidewalkRise = engine::roads::lanes::lanesSidewalkRise();   // paving meets the lab's sidewalk (ADR-0086)
    const auto t0 = std::chrono::steady_clock::now();
    NetLotResult r;
    r.lots = growLotBuildings(blocks, s.lp, &r.plan, s.planOnly ? nullptr : &r.parts, nullptr, 0.0,
                              (s.wantFlat && !s.planOnly) ? &r.flatParts : nullptr, &r.gradeFlatten);
    if (report) {
        size_t units = 0; for (const LotBuilding& b : r.lots) units += b.units.size();
        size_t partTris = 0; for (const RenderMesh& m : r.parts) partTris += m.indices.size() / 3;
        const double seconds = secondsSince(t0);
        *report = {{"summary", std::to_string(blocks.size()) + " blocks, " + std::to_string(r.plan.lots.size()) + " lots, " + std::to_string(r.lots.size()) + " buildings, " + std::to_string(units) + " units, " + std::to_string(partTris) + " part triangles"},
                   {"blocks", blocks.size()}, {"lots", r.plan.lots.size()}, {"buildings", r.lots.size()}, {"units", units}, {"parts", r.parts.size()}, {"partTriangles", partTris},
                   {"flatParts", r.flatParts.size()}, {"terraces", r.gradeFlatten.size()}, {"planOnly", s.planOnly}, {"lod1", s.wantFlat && !s.planOnly}, {"hasTerrain", city.hasTerrain}, {"spawn", haveSpawn}, {"seconds", seconds}};
    }
    return r;
}

}  // namespace roads::lanes
}  // namespace engine
