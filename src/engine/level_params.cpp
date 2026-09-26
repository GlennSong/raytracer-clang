#include "level_params.h"
#include "procgen/terrain_weather.h"   // the weathered ground (ADR-0126)
#include "procgen/terrain_maps.h"      // its maps (ADR-0128)
#include "procgen/forest.h"            // its forest (ADR-0129)
#include "procgen/trails.h"            // its trails (ADR-0134)
#include "procgen/terrain_lod.h"   // kBakedCell0 (ADR-0095)

#include "procgen/erosion.h"
#include "procgen/ground_cover.h"
#include "procgen/hydrology.h"
#include "../log.h"
#include <chrono>
#include <map>
#include <mutex>
#include "procgen/noise.h"

using json = nlohmann::json;

namespace engine {

std::vector<std::vector<Vec2>> levelWaterKeepOut(const json& rootIn, double margin) {
    if (!rootIn.contains("terrain") || !rootIn["terrain"].is_object() || !rootIn["terrain"].contains("rivers")) return {};
    json root = rootIn;
    propagateWaterSeaLevel(root);   // the hydrology ends its rivers at the sea
    const TerrainParams p = readTerrainParams(root["terrain"]);
    return p.hydro ? p.hydro->corridorRings(margin) : std::vector<std::vector<Vec2>>{};
}

void propagateWaterSeaLevel(json& root) {
    if (!root.contains("water")) return;
    const json& w = root["water"];
    double sea = w.value("seaLevel", 0.0);
    double beach = w.value("beachRise", 2.5);
    // The terrain colours its coast by this same sea level (beach/rock/sea floor).
    if (root.contains("terrain") && root["terrain"].is_object() &&
        !root["terrain"].contains("seaLevel"))
        root["terrain"]["seaLevel"] = sea;
    // Every road recipe gates buildability on it (roads/blocks stay on land).
    if (!root.contains("entities") || !root["entities"].is_array()) return;
    for (auto& ent : root["entities"]) {
        if (ent.value("shape", std::string()) != "road" || !ent.contains("road"))
            continue;
        json& road = ent["road"];
        if (!road.contains("generate") || !road["generate"].is_object()) continue;
        json& g = road["generate"];
        if (!g.contains("sea_level")) g["sea_level"] = sea;
        if (!g.contains("beach_rise")) g["beach_rise"] = beach;
    }
}

std::shared_ptr<const std::function<double(double, double)>> erodedForTerrain(const json& tj);

TerrainParams readTerrainParams(const json& t) {
    TerrainParams p;
    p.size        = t.value("size", p.size);
    p.resolution  = t.value("resolution", p.resolution);
    p.heightScale = t.value("heightScale", p.heightScale);
    p.noiseScale  = t.value("noiseScale", p.noiseScale);
    p.octaves     = t.value("octaves", p.octaves);
    p.warp        = t.value("warp", p.warp);
    p.mountainHeight = t.value("mountainHeight", p.mountainHeight);
    p.mountainScale  = t.value("mountainScale", p.mountainScale);
    p.mountainMaskScale = t.value("mountainMaskScale", p.mountainMaskScale);
    p.mountainMaskLo = t.value("mountainMaskLo", p.mountainMaskLo);
    p.mountainMaskHi = t.value("mountainMaskHi", p.mountainMaskHi);
    p.mountainAlongRange = t.value("mountainAlongRange", p.mountainAlongRange);
    p.tiltX = t.value("tiltX", p.tiltX);
    p.tiltZ = t.value("tiltZ", p.tiltZ);
    // AUTHORED flatten records (walkway-lab round): the height stack's record
    // types, written directly in the level instead of arriving only via the
    // road/lot passes — so an isolated scene can stage pads, ramps, and
    // priority overlaps one at a time and prove each against the mesh.
    //   {"type":"pad","poly":[[x,z],...],"y":H,"falloff":F,"priority":P}
    //   {"type":"ramp","a":[x,z],"b":[x,z],"ya":H,"yb":H,
    //    "halfWidth":W,"falloff":F,"priority":P}
    if (t.contains("flatten") && t["flatten"].is_array()) {
        for (const auto& f : t["flatten"]) {
            const std::string kind = f.value("type", std::string("pad"));
            TerrainFlatten r;
            if (kind == "ramp") {
                const auto& a = f["a"];
                const auto& b = f["b"];
                r = makeFlattenRamp(
                    Vec3(a[0].get<double>(), 0, a[1].get<double>()),
                    Vec3(b[0].get<double>(), 0, b[1].get<double>()),
                    f.value("ya", 0.0), f.value("yb", 0.0),
                    f.value("halfWidth", 4.0), f.value("falloff", 6.0));
            } else {
                std::vector<Vec3> poly;
                for (const auto& v : f.value("poly", json::array()))
                    if (v.is_array() && v.size() >= 2)
                        poly.push_back(Vec3(v[0].get<double>(), 0, v[1].get<double>()));
                if (poly.size() < 3) continue;
                r = makeFlattenPad(std::move(poly), f.value("y", 0.0),
                                   f.value("falloff", 6.0));
            }
            r.priority = f.value("priority", 0);
            p.flatten.push_back(std::move(r));
        }
    }
    p.seaLevel = t.value("seaLevel", p.seaLevel);   // loaders may override from the water block
    p.erodeLandOnly = t.value("erodeLandOnly", p.erodeLandOnly);
    // The earthwork field's knobs (procgen/earthwork.h):
    //   "earthwork": { "enabled": true, "reach": 100, "cell": 4, "margin": 0 }
    if (t.contains("earthwork") && t["earthwork"].is_object()) {
        const auto& e = t["earthwork"];
        p.earthworkParams.enabled = e.value("enabled", p.earthworkParams.enabled);
        p.earthworkParams.reach = e.value("reach", p.earthworkParams.reach);
        p.earthworkParams.cell = e.value("cell", p.earthworkParams.cell);
        p.earthworkParams.margin = e.value("margin", p.earthworkParams.margin);
    }
    // RT_EARTHWORK=0|1 overrides the level for an A/B on the same file: the
    // bank census with and without the field, nothing else changed.
    if (const char* ov = std::getenv("RT_EARTHWORK"))
        p.earthworkParams.enabled = std::atoi(ov) != 0;
    p.snowLine = t.value("snowLine", p.snowLine);   // colour-band scaling
    p.rockLine = t.value("rockLine", p.rockLine);
    if (t.contains("rangeSpine") && t["rangeSpine"].is_array()) {
        std::vector<Vec3> ctl;
        for (const auto& pt : t["rangeSpine"])
            if (pt.is_array() && pt.size() >= 2)
                ctl.push_back(Vec3(pt[0].get<double>(), 0.0, pt[1].get<double>()));
        p.rangeSpine = sampleRangeSpine(ctl);
    }
    p.rangeWidth = t.value("rangeWidth", p.rangeWidth);
    p.rangeHeight = t.value("rangeHeight", p.rangeHeight);
    p.rangeVariation = t.value("rangeVariation", p.rangeVariation);
    if (t.contains("island") && t["island"].is_object()) {   // ADR-0105
        const json& il = t["island"];
        TerrainParams::Island& I = p.island;
        I.on = il.value("on", true);
        if (il.contains("center") && il["center"].is_array()) { I.cx = il["center"][0].get<double>(); I.cz = il["center"][1].get<double>(); }
        I.radius = il.value("radius", I.radius); I.aspect = il.value("aspect", I.aspect); I.angleDeg = il.value("angle", I.angleDeg);
        I.coastNoise = il.value("coastNoise", I.coastNoise); I.coastScale = il.value("coastScale", I.coastScale);
        if (il.contains("peninsula") && il["peninsula"].is_object()) {
            const json& pn = il["peninsula"];
            I.penDeg = pn.value("bearing", 0.0); I.penLength = pn.value("length", I.penLength); I.penWidth = pn.value("width", I.penWidth);
        }
        if (il.contains("cliffs") && il["cliffs"].is_object()) {
            const json& c = il["cliffs"];
            I.cliffFromDeg = c.value("from", 0.0); I.cliffToDeg = c.value("to", 0.0); I.cliffHeight = c.value("height", 0.0);
        }
        I.plainHeight = il.value("plainHeight", I.plainHeight); I.shelfDepth = il.value("shelfDepth", I.shelfDepth);
    }
    if (t.contains("range") && t["range"].is_object()) {
        const auto& r = t["range"];
        p.rangeRidges = buildRangeRidges(
            r.value("length", 60.0f), r.value("branchAngle", 38.0f),
            r.value("falloff", 0.55f), r.value("leaderFalloff", 0.92f),
            r.value("iterations", 5), r.value("height", 130.0f),
            r.value("depthFalloff", 0.62f), r.value("angleJitter", 12.0f),
            r.value("seed", 0u));
        p.rangeWidth = r.value("width", p.rangeWidth);
    }
    // THE GROUND-COVER MAP (procgen/ground_cover.h): "groundCover": {...} in the terrain block.
    // Its sea level is the terrain's (the water block's, via propagateWaterSeaLevel).
    if (t.contains("groundCover") && t["groundCover"].is_object()) {
        const json& g = t["groundCover"];
        GroundCoverParams gp;
        gp.seaLevel = p.seaLevel;
        gp.beachHeight = g.value("beachHeight", gp.beachHeight);
        gp.uplandHeight = g.value("uplandHeight", gp.uplandHeight);
        gp.mountainHeight = g.value("mountainHeight", gp.mountainHeight);
        gp.snowHeight = g.value("snowHeight", gp.snowHeight);
        gp.rockSlopeDeg = g.value("rockSlopeDeg", gp.rockSlopeDeg);
        gp.dirtPatches = g.value("dirtPatches", gp.dirtPatches);
        gp.seed = g.value("seed", gp.seed);
        auto col = [&](const char* key, Vec3& into) {
            if (g.contains(key) && g[key].is_array() && g[key].size() == 3)
                into = Vec3(g[key][0].get<double>(), g[key][1].get<double>(), g[key][2].get<double>());
        };
        col("grass", gp.grass); col("grassDry", gp.grassDry); col("dirt", gp.dirt);
        col("sand", gp.sand); col("rock", gp.rock); col("snow", gp.snow);
        if (t.value("erode", false)) gp.maps = weatheredMapsFor(t);   // ADR-0128: nullptr unless weathered
        if (t.contains("trails") && t["trails"].is_array())                 // ADR-0134: the hiking trails
            gp.trails = std::make_shared<const TrailNetwork>(TrailNetwork::fromJson(t["trails"]));
        if (t.contains("forest") && t["forest"].is_object())                // ADR-0129: litter under the canopy
            gp.forest = std::make_shared<const ForestParams>(forestFromJson(t["forest"]));
        p.cover = std::make_shared<const GroundCover>(gp);
        p.coverWeights = g.value("layered", true);   // textured layers (the loader binds them) or a flat colour
    }
    // HYDROLOGY (procgen/hydrology.h, ADR-0099): "rivers": {...} in the terrain block. The network
    // is computed on the terrain's own base relief, once per process for a given terrain block
    // (readTerrainParams runs many times a load, and every copy shares the network).
    if (t.contains("rivers") && t["rivers"].is_object()) {
        static std::mutex memoMutex;
        static std::map<std::string, std::shared_ptr<const Hydrology>> memo;
        const std::string key = t.dump();
        std::lock_guard<std::mutex> lock(memoMutex);
        auto it = memo.find(key);
        if (it == memo.end()) {
            const json& r = t["rivers"];
            HydroParams hp;
            hp.half = r.value("region", static_cast<double>(p.size)) * 0.5;
            hp.cell = r.value("cell", hp.cell);
            hp.seaLevel = p.seaLevel;
            hp.riverArea = r.value("riverArea", hp.riverArea);
            hp.widthMin = r.value("widthMin", hp.widthMin);
            hp.widthMax = r.value("widthMax", hp.widthMax);
            hp.widthK = r.value("widthK", hp.widthK);
            hp.widthVariation = r.value("widthVariation", hp.widthVariation);
            hp.depthMin = r.value("depthMin", hp.depthMin);
            hp.depthMax = r.value("depthMax", hp.depthMax);
            hp.depthK = r.value("depthK", hp.depthK);
            hp.bankSlope = r.value("bankSlope", hp.bankSlope);
            hp.incisionMin = r.value("incisionMin", hp.incisionMin);
            hp.incisionMax = r.value("incisionMax", hp.incisionMax);
            hp.incisionK = r.value("incisionK", hp.incisionK);
            hp.bankSteep = r.value("bankSteep", hp.bankSteep);
            hp.lakeMinArea = r.value("lakeMinArea", hp.lakeMinArea);
            hp.autoRivers = r.value("auto", hp.autoRivers);
            if (r.contains("courses") && r["courses"].is_array())
                for (const json& c : r["courses"]) {
                    HydroParams::Course course;
                    for (const json& q : c.value("points", json::array())) course.points.emplace_back(q[0].get<double>(), q[1].get<double>());
                    if (c.contains("width") && c["width"].is_array() && c["width"].size() == 2) {
                        course.width0 = c["width"][0].get<double>();
                        course.width1 = c["width"][1].get<double>();
                    }
                    course.meander = c.value("meander", course.meander);
                    hp.courses.push_back(std::move(course));
                }
            hp.lakeMinDepth = r.value("lakeMinDepth", hp.lakeMinDepth);
            TerrainParams base = p;   // the relief the water runs over: no flatten, no earthwork
            base.flatten.clear();
            base.earthwork.reset();
            base.hydro.reset();
            const Noise n(t.value("seed", 0u));
            const auto t0 = std::chrono::steady_clock::now();
            // the ground the water runs over: the ERODED relief when the terrain erodes (its valleys
            // are where the rivers belong), else the raw relief
            std::function<double(double, double)> over = [base, n](double x, double z) { return terrainBaseHeight(base, n, x, z); };
            if (t.value("erode", false))
                if (auto eb = erodedForTerrain(t)) over = [eb](double x, double z) { return (*eb)(x, z); };
            auto hy = Hydrology::build(over, hp);
            LOG_INFO << "[hydrology] " << hy->rivers().size() << " rivers, " << hy->lakes().size() << " lakes on a "
                     << hy->gridSize() << "^2 grid (" << hp.cell << " m) in "
                     << std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() << " s";
            it = memo.emplace(key, std::move(hy)).first;
        }
        p.hydro = it->second;
    }
    return p;
}

TreeParams readTreeParams(const json& ent, uint32_t& seedOut) {
    TreeParams tp;
    seedOut = 0;
    if (!ent.contains("tree")) return tp;
    const auto& j = ent["tree"];
    tp.iterations      = j.value("iterations", tp.iterations);
    tp.trunkLength     = j.value("trunkLength", tp.trunkLength);
    tp.lengthFalloff   = j.value("lengthFalloff", tp.lengthFalloff);
    tp.leaderFalloff   = j.value("leaderFalloff", tp.leaderFalloff);
    tp.branchAngle     = j.value("branchAngle", tp.branchAngle);
    tp.angleJitter     = j.value("angleJitter", tp.angleJitter);
    tp.branchesPerNode = j.value("branchesPerNode", tp.branchesPerNode);
    tp.phyllotaxis     = j.value("phyllotaxis", tp.phyllotaxis);
    tp.terminalFraction = j.value("terminalFraction", tp.terminalFraction);
    tp.terminalForks   = j.value("terminalForks", tp.terminalForks);
    tp.droop           = j.value("droop", tp.droop);
    tp.wander          = j.value("wander", tp.wander);
    tp.rootCount       = j.value("rootCount", tp.rootCount);
    tp.rootSpread      = j.value("rootSpread", tp.rootSpread);
    tp.leafClump       = j.value("leafClump", tp.leafClump);
    tp.maxLeafCards    = j.value("maxLeafCards", tp.maxLeafCards);
    tp.tipRadius       = j.value("tipRadius", tp.tipRadius);
    tp.pipeExponent    = j.value("pipeExponent", tp.pipeExponent);
    tp.radiusScale     = j.value("radiusScale", tp.radiusScale);
    tp.ringSegments    = j.value("ringSegments", tp.ringSegments);
    tp.leaves          = j.value("leaves", tp.leaves);
    tp.leafSize        = j.value("leafSize", tp.leafSize);
    tp.leavesPerTip    = j.value("leavesPerTip", tp.leavesPerTip);
    tp.leafThickness   = j.value("leafThickness", tp.leafThickness);
    tp.barkColor       = parseVec3(j.value("barkColor", json()), tp.barkColor);
    tp.leafColor       = parseVec3(j.value("leafColor", json()), tp.leafColor);
    seedOut            = j.value("seed", 0u);
    return tp;
}

double levelDrawnGroundCell(const json& root) {
    if (!root.contains("terrain") || !root["terrain"].contains("cdlod")) return 0.0;
    const json& cj = root["terrain"]["cdlod"];
    const double worldHalf = cj.is_object() ? cj.value("worldHalf", 1024.0) : 1024.0;
    const int numLods = cj.is_object() ? cj.value("numLods", 6) : 6;
    const int gridRes = cj.is_object() ? cj.value("gridRes", 32) : 32;
    const char* bakedEnv = std::getenv("RT_BAKED_TERRAIN");
    if (cj.is_object() && cj.value("baked", false) && !(bakedEnv && bakedEnv[0] == '0')) return kBakedCell0;
    return (worldHalf * 2.0 / double(1 << (numLods - 1))) / std::max(1, gridRes);
}

std::shared_ptr<const std::function<double(double, double)>>
readErodedBase(const json& root) {
    if (!root.contains("terrain") || !root["terrain"].value("erode", false))
        return nullptr;
    return erodedForTerrain(root["terrain"]);
}

std::shared_ptr<const std::function<double(double, double)>> erodedForTerrain(const json& tjIn) {
    // once per terrain block (a load reads it several times; the island planner and the level agree).
    // Its "rivers" are left out: they are computed ON this, and reading them here would recurse.
    json tj = tjIn;
    tj.erase("rivers");
    static std::mutex memoMutex;
    static std::map<std::string, std::shared_ptr<const std::function<double(double, double)>>> memo;
    const std::string key = tj.dump();
    std::lock_guard<std::mutex> lock(memoMutex);
    if (auto it = memo.find(key); it != memo.end()) return it->second;
    // THE WEATHERED GROUND (ADR-0126): grown from uplift and weathered by water offline, cached on disk
    if (tj.contains("weather") && tj["weather"].is_object()) {
        auto grid = std::make_shared<const Heightmap>(weatheredTerrainCached(tj));
        auto f = std::make_shared<const std::function<double(double, double)>>(
            [grid](double x, double z) { return static_cast<double>(grid->sampleWorld(static_cast<float>(x), static_cast<float>(z))); });
        memo[key] = f;
        return f;
    }
    TerrainParams eb = readTerrainParams(tj);
    Noise en(tj.value("seed", 0u));
    ErosionParams ep;
    ep.seed = tj.value("seed", 0u) + 1234u;
    ep.droplets = tj.value("erodeDroplets", ep.droplets);
    ep.erodeRadius = tj.value("erodeRadius", ep.erodeRadius);
    ep.thermalIterations = tj.value("erodeThermal", ep.thermalIterations);
    ep.talus = tj.value("erodeTalus", ep.talus);
    ep.vulkan = tj.value("erodeGpu", false);   // ADR-0122: opt-in on Vulkan builds
    bakeErodedTerrain(eb, en, eb.size, tj.value("erodeRes", 512), ep);
    memo[key] = eb.erodedBase;
    return eb.erodedBase;
}

void readLotGrowParams(const json& cityJson,
                       EdgeBlockParams& edges, LotParams& lots) {
    edges.depth = cityJson.value("edgeBlockDepth", edges.depth);
    edges.minLen = cityJson.value("edgeBlockMinLen", edges.minLen);
    edges.maxLen = cityJson.value("edgeBlockMaxLen", edges.maxLen);
    edges.margin = 4.0 + cityJson.value("sidewalk", 4.0);
    lots.seed = cityJson.value("seed", 1u) ^ 0x10c5u;
    lots.buildChance = cityJson.value("buildChance", 0.9);
    // ROAD MARGIN — sidewalk only, no road-half term (measured 2026-08-16).
    //
    // This used to be `4.0 + sidewalk`, where the 4.0 stood in for a road's half
    // width. That was a guess at ONE road's half width (an 8 m street), applied
    // uniformly — too little for a 17 m arterial, too much for everything narrow —
    // and it is now redundant: `pushPolyClearOfRoads` (added in b167c1d) pushes
    // every block vertex clear of every carriageway using that edge's OWN width,
    // per edge, against the sampled centreline. The scalar cannot do better than a
    // guess and the per-edge pass cannot do worse than exact, so the guess only
    // costs buildable area.
    //
    // It costs a lot of it: block interiors totalled 24 252 m² inside a ~90 000 m²
    // city on living_city, and buildings already covered 60.5% of that interior.
    // The city reads empty because ~73% of it is road and margin, not because the
    // blocks are under-built.
    //
    // Gate: tests/test_lot_road_clearance.cpp
    //       lot_block_interiors_clear_every_carriageway_on_a_mixed_width_net
    // fails if this lets a block interior reach into a carriageway.
    lots.roadMargin = cityJson.value("sidewalk", 4.0);
    lots.innerRadius = cityJson.value("downtownRadius", 55.0);
    lots.midRadius = cityJson.value("midtownRadius", 135.0);
    lots.plinth = cityJson.value("plinth", lots.plinth);   // base height above the pad
    lots.hubRadius = cityJson.value("hubRadius", lots.hubRadius);
    // Level-authored parcel grain ("parcel", 8km-city P3): piedmont-scale metros
    // lay 150 m+ blocks, so the level can ask for bigger lots. Six knobs only;
    // absent = the compiled-in district tuning, untouched (city_lots rescales
    // its per-district grain from these).
    //
    // These live HERE, in the shared reader, on purpose: they were duplicated
    // into both loaders, and the copy in level_scene silently fell behind —
    // the editor grew a different city than the game (no parcel grain, no
    // polycentric zoning, no tower core). That is the whole reason this
    // function exists, so a field added to one loader cannot go missing in the
    // other.
    //
    // DISTRICTS THE LEVEL AUTHORS ("districts", 2026-09-20). `hubs` used to come
    // only from the road nets — a metro RECIPE plans them and leaves them on the
    // entity — which left a city built from a baked lane graph with no districts
    // at all: one radial core, and half the towers of the recipe-planned city
    // beside it. A level can now say where its quarters are:
    //
    //   "districts": { "hubRadius": 220,
    //                  "hubs": [ {"at": [x, z], "kind": "financial"}, ... ] }
    //
    // kind is financial | commercial | residential | oldtown | industrial (the
    // DistrictTag order). Lots zone by the NEAREST hub; a kind-0 (financial) hub
    // reads as a downtown CORE — the radial rings are measured from it, so
    // downtownRadius/midtownRadius size its financial and commercial discs, and
    // several of them make several downtowns. When a level authors hubs they are
    // the city's districts; the nets' own are not merged in, or a recipe would
    // silently outvote the author. `road_products_probe <level>` prints what a
    // recipe planned in exactly this form, to copy across.
    if (cityJson.contains("districts") && cityJson["districts"].is_object()) {
        const auto& dj = cityJson["districts"];
        lots.hubRadius = dj.value("hubRadius", lots.hubRadius);
        if (dj.contains("hubs") && dj["hubs"].is_array()) {
            auto kindOf = [](const std::string& n) {
                if (n == "financial") return 0;
                if (n == "commercial") return 1;
                if (n == "residential") return 2;
                if (n == "oldtown") return 3;
                if (n == "industrial") return 4;
                return 2;                                  // an unknown quarter is housing
            };
            bool haveCore = false;
            for (const auto& hj : dj["hubs"]) {
                if (!hj.is_object() || !hj.contains("at") || !hj["at"].is_array() || hj["at"].size() < 2)
                    continue;
                const Vec2 at(hj["at"][0].get<double>(), hj["at"][1].get<double>());
                const int kind = hj["kind"].is_number()
                                     ? hj.value("kind", 2)
                                     : kindOf(hj.value("kind", std::string("residential")));
                lots.hubs.push_back({at, kind});
                // Coreness (height grading, landmark quotas) measures from `center`:
                // the first financial hub is downtown, else the first hub at all.
                if (!haveCore && (kind == 0 || lots.hubs.size() == 1)) {
                    lots.center = at;
                    haveCore = kind == 0;
                }
            }
        }
    }
    if (cityJson.contains("parcel") && cityJson["parcel"].is_object()) {
        const auto& pj = cityJson["parcel"];
        lots.parcelTargetArea = pj.value("targetArea", lots.parcelTargetArea);
        lots.parcelMinArea = pj.value("minArea", lots.parcelMinArea);
        lots.parcelMinEdge = pj.value("minEdge", lots.parcelMinEdge);
        lots.parcelFrontWidth = pj.value("frontWidth", lots.parcelFrontWidth);
        lots.parcelLotDepth = pj.value("lotDepth", lots.parcelLotDepth);
        lots.parcelCourtMinArea = pj.value("courtMinArea", lots.parcelCourtMinArea);
    }
    // Stage-10 alleys (courts-with-alleys round): service lanes cut into blocks
    // whose parcelled lots would otherwise fail the frontage gate. On by
    // default; a level can opt out ("alleys": false) or retune the pavement.
    lots.alleys = cityJson.value("alleys", lots.alleys);
    lots.alleyWidth = cityJson.value("alleyWidth", lots.alleyWidth);
}

WaterMeshParams readWaterParams(const json& w) {
    WaterMeshParams wp;
    wp.seaLevel = w.value("seaLevel", 0.0);
    if (double region = w.value("region", 0.0); region > 0.0) {
        wp.lo = {-region, -region};
        wp.hi = {region, region};
    }
    if (w.contains("lo") && w["lo"].is_array())
        wp.lo = {w["lo"][0].get<double>(), w["lo"][1].get<double>()};
    if (w.contains("hi") && w["hi"].is_array())
        wp.hi = {w["hi"][0].get<double>(), w["hi"][1].get<double>()};
    wp.cell = w.value("cell", wp.cell);
    wp.foamBand = w.value("foamBand", wp.foamBand);
    return wp;
}

}  // namespace engine
