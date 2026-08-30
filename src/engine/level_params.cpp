#include "level_params.h"

#include "procgen/erosion.h"
#include "procgen/noise.h"

using json = nlohmann::json;

namespace engine {

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

std::shared_ptr<const std::function<double(double, double)>>
readErodedBase(const json& root) {
    if (!root.contains("terrain") || !root["terrain"].value("erode", false))
        return nullptr;
    const json& tj = root["terrain"];
    TerrainParams eb = readTerrainParams(tj);
    Noise en(tj.value("seed", 0u));
    ErosionParams ep;
    ep.seed = tj.value("seed", 0u) + 1234u;
    ep.droplets = tj.value("erodeDroplets", ep.droplets);
    ep.erodeRadius = tj.value("erodeRadius", ep.erodeRadius);
    ep.thermalIterations = tj.value("erodeThermal", ep.thermalIterations);
    ep.talus = tj.value("erodeTalus", ep.talus);
    bakeErodedTerrain(eb, en, eb.size, tj.value("erodeRes", 512), ep);
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
    // other. `hubs` and `center` cannot join them: they come from the road
    // nets, not the JSON, so each caller still fills those from its own nets.
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
