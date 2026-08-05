#include "level_params.h"

#include "procgen/erosion.h"
#include "procgen/noise.h"

using json = nlohmann::json;

namespace engine {

namespace {
Vec3 readVec3(const json& j, Vec3 fallback = Vec3()) {
    if (!j.is_array() || j.size() != 3) return fallback;
    return Vec3(j[0].get<Real>(), j[1].get<Real>(), j[2].get<Real>());
}
}  // namespace

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
    p.seaLevel = t.value("seaLevel", p.seaLevel);   // loaders may override from the water block
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
    tp.barkColor       = readVec3(j.value("barkColor", json()), tp.barkColor);
    tp.leafColor       = readVec3(j.value("leafColor", json()), tp.leafColor);
    seedOut            = j.value("seed", 0u);
    return tp;
}

CityParams readCityParams(const json& ent, const json& root) {
    CityParams cp;
    Vec3 pos = readVec3(ent.value("position", json()));
    cp.center = {pos.x, pos.z};
    cp.baseY = pos.y;
    bool onTerrain = false;
    if (ent.contains("city")) {
        const auto& j = ent["city"];
        cp.extent         = j.value("extent", cp.extent);
        cp.cellSize       = j.value("cellSize", cp.cellSize);
        cp.roadJitter     = j.value("roadJitter", cp.roadJitter);
        cp.sidewalk       = j.value("sidewalk", cp.sidewalk);
        cp.downtownRadius = j.value("downtownRadius", cp.downtownRadius);
        cp.midtownRadius  = j.value("midtownRadius", cp.midtownRadius);
        cp.parkFraction   = j.value("parkFraction", cp.parkFraction);
        cp.buildChance    = j.value("buildChance", cp.buildChance);
        cp.scatterTrees   = j.value("scatterTrees", cp.scatterTrees);
        cp.seed           = j.value("seed", cp.seed);
        onTerrain         = j.value("onTerrain", false);
        // District road tech (ADR-0066): real arterials + irregular streets whose
        // blocks feed the lot/building pipeline, instead of the regular grid.
        cp.districtRoads  = j.value("districtRoads", cp.districtRoads);
        cp.arterials      = j.value("arterials", cp.arterials);
        cp.blockSizeMin   = j.value("blockSizeMin", cp.blockSizeMin);
        cp.blockSizeMax   = j.value("blockSizeMax", cp.blockSizeMax);
        cp.arteryWidth    = j.value("arteryWidth", cp.arteryWidth);
        cp.streetWidth    = j.value("streetWidth", cp.streetWidth);
        cp.irregular      = j.value("irregular", cp.irregular);
    }
    // Drape onto the level's terrain (ADR-0038 §6). The shared_ptrs keep the
    // params/noise alive for the sampler closure the model may hold; it reads
    // the *base* terrain (no flatten) so the city decides its grades from
    // natural ground and its flatten footprints then cut the mesh.
    if (onTerrain && root.contains("terrain")) {
        auto tp = std::make_shared<TerrainParams>(readTerrainParams(root["terrain"]));
        auto noise = std::make_shared<Noise>(root["terrain"].value("seed", 0u));
        Real base = cp.baseY;
        cp.groundAt = [tp, noise, base](const Vec2& p) {
            return base + terrainHeight(*tp, *noise, p.x, p.y);
        };
    }
    return cp;
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
    lots.roadMargin = 4.0 + cityJson.value("sidewalk", 4.0);   // road half + sidewalk
    lots.innerRadius = cityJson.value("downtownRadius", 55.0);
    lots.midRadius = cityJson.value("midtownRadius", 135.0);
    lots.plinth = cityJson.value("plinth", lots.plinth);   // base height above the pad
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
