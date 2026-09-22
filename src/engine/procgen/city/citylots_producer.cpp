#include "engine/procgen/city/citylots_producer.h"

#include "engine/city_prepass.h"
#include "engine/lot_grow_setup.h"
#include "engine/procgen/city/lot_cache.h"
#include "engine/script_assets.h"
#include "log.h"

#include <chrono>

namespace engine {

const char* const kCityLotsProducerName = "citylots";
const char* const kCityLotsSectionPrefix = "citylots/";
// Bump whenever the lot pass's output changes for the same inputs (the key cannot see code).
const char* const kCityLotsBuildTag = "2026-09-21.17";   // 2026-09-21: ground-relative (draped) dressing slots (.1-.3 carried lot-depth and door-rule experiments, both reverted)   // .5: door walks end at the back of the sidewalk   // .6: a block the parcel walk cannot fill is one landmark site   // .1: a lot that cannot clear the carriageway is green, not built from the raw site polygon   // .2: propagateWaterSeaLevel, so a level with water grows the loader's graph   // .3: measure pads that still cover carriageway

namespace {

double secondsSince(const std::chrono::steady_clock::time_point& t) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count();
}

bool lotsWanted(const nlohmann::json& level) {
    const nlohmann::json cs = level.value("citysim", nlohmann::json::object());
    return cs.is_object() && (cs.value("buildLots", false) || cs.value("planOnly", false));
}

double renderCellOf(const nlohmann::json& level) {
    if (level.contains("citysim") && level["citysim"].is_object())
        return level["citysim"].value("renderCell", 250.0);
    return 250.0;
}

class CityLotsProducer : public bundle::BundleProducer {
    std::string name() const override { return kCityLotsProducerName; }

    bool applies(const bundle::LevelInputs& in) const override {
        return lotsWanted(in.level) && cityPrePassApplies(in.level);
    }

    double weight() const override { return 40.0; }   // measured: piedmont's pre-pass is ~41 s

    bundle::ProducerIdentity identity(const bundle::LevelInputs& in) const override {
        bundle::ProducerIdentity id; id.tag = kCityLotsBuildTag;
#ifdef RT_ENABLE_SCRIPTING
        const int scripting = 1;
#else
        const int scripting = 0;
#endif
        uint64_t k = bundle::fnv1aStr(std::string(kCityLotsProducerName) + "|" +
                                      std::to_string(lotcache::kLotsFormatVersion) + "|" + kCityLotsBuildTag +
                                      "|real" + std::to_string(sizeof(Real)) + "|scripting" + std::to_string(scripting));
        // Everything the pre-pass world is derived from: the terrain block (erosion params included),
        // every road entity's block, the citysim knobs, the water block (the earthwork's sea pin), and
        // the render cell the parts are split on.
        k = bundle::fnv1aStr(in.level.value("terrain", nlohmann::json::object()).dump(), k);
        k = bundle::fnv1aStr(in.level.value("citysim", nlohmann::json::object()).dump(), k);
        k = bundle::fnv1aStr(in.level.value("water", nlohmann::json::object()).dump(), k);
        for (const nlohmann::json& e : in.level.value("entities", nlohmann::json::array()))
            if (e.is_object() && e.value("shape", std::string()) == "road")
                k = bundle::fnv1aStr(e.value("road", nlohmann::json::object()).dump(), k);
        const double cell = renderCellOf(in.level);
        k = bundle::fnv1a(&cell, sizeof(cell), k);
        Vec2 spawn;
        if (authoredSpawnXZ(in.level, spawn)) {
            k = bundle::fnv1a(&spawn.x, sizeof(spawn.x), k);
            k = bundle::fnv1a(&spawn.y, sizeof(spawn.y), k);
        } else k = bundle::fnv1aStr("nospawn", k);
        for (const char* book : {"style_book.lua", "archetype_book.lua"}) {
            const std::string p = resolveScriptPath(book, in.levelDir);
            if (p.empty()) { k = bundle::fnv1aStr(std::string("nobook:") + book, k); continue; }
            bundle::InputFile f; f.path = p; uint64_t fk = bundle::kFnvOffset;
            if (bundle::fnv1aFile(p, fk, &f.bytes, &f.mtime)) { f.fnv = fk; k = bundle::fnv1a(&fk, sizeof(fk), k); }
            else k = bundle::fnv1aStr("missing:" + p, k);
            id.inputs.push_back(f);
        }
        id.key = k; return id;
    }

    bundle::ProducerReport produce(const bundle::LevelInputs& in, bundle::BundleWriter& out,
                                   const bundle::ProgressFn* progress) const override {
        bundle::ProducerReport rep;
        const auto t0 = std::chrono::steady_clock::now();
        auto report = [&](double f, const char* stage) {
            if (!progress || !*progress) return true;
            bundle::Progress p; p.producer = kCityLotsProducerName; p.stage = stage; p.fraction = f;
            return (*progress)(p);
        };

        if (!report(0.02, "roads")) { rep.ok = false; rep.error = "cancelled"; return rep; }
        CityPrePass pre;
        if (!cityPrePassForLevel(in.level, pre)) {
            rep.ok = false; rep.error = "the level's lot pre-pass world is not derivable from its JSON";
            return rep;
        }
        const double tRoads = secondsSince(t0);

        if (!report(0.10, "lots")) { rep.ok = false; rep.error = "cancelled"; return rep; }
        const auto tGrow0 = std::chrono::steady_clock::now();
        Vec2 spawn; const bool haveSpawn = authoredSpawnXZ(in.level, spawn);
        LotGrowSetup s = lotGrowSetupForLevel(in.level.value("citysim", nlohmann::json::object()), in.levelDir,
                                              pre.lotGround, pre.nets, pre.lotGroundWith, pre.lotMeshCell,
                                              haveSpawn ? &spawn : nullptr);
        NetLotResult r = growLotBuildingsOnNets(pre.nets, s.lp, s.ep, s.roadClear, pre.naturalGround,
                                                pre.freewayROWOrNull(), s.wantFlat, !s.planOnly);
        const double tGrow = secondsSince(tGrow0);

        if (!report(0.90, "write")) { rep.ok = false; rep.error = "cancelled"; return rep; }
        const auto tWrite0 = std::chrono::steady_clock::now();
        lotcache::writeLotResult(out, kCityLotsSectionPrefix, r, renderCellOf(in.level));
        const double tWrite = secondsSince(tWrite0);

        rep.timings["roads"] = tRoads;
        rep.timings["grow"] = tGrow;
        rep.timings["write"] = tWrite;
        rep.report["lots"] = r.plan.lots.size();
        rep.report["buildings"] = r.lots.size();
        rep.report["blocks"] = r.plan.blocks.size();
        rep.report["parts"] = r.parts.size();
        rep.report["grade"] = r.gradeFlatten.size();
        rep.report["summary"] = std::to_string(r.plan.blocks.size()) + " blocks, " +
                                std::to_string(r.plan.lots.size()) + " lots, " +
                                std::to_string(r.lots.size()) + " buildings (roads " +
                                std::to_string(tRoads).substr(0, 4) + " s, grow " +
                                std::to_string(tGrow).substr(0, 5) + " s)";
        rep.seconds = secondsSince(t0);
        report(1.0, "done");
        return rep;
    }
};

}  // namespace

void registerCityLotsProducer() {
    if (bundle::findProducer(kCityLotsProducerName)) return;
    bundle::registerProducer(std::make_unique<CityLotsProducer>());
}

}  // namespace engine
