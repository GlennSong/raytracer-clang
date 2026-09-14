#include "engine/city_prepass.h"

#include "engine/level_params.h"
#include "engine/procgen/earthwork.h"

namespace engine {
namespace {

bool hasShape(const nlohmann::json& root, const char* shape) {
    if (!root.contains("entities") || !root["entities"].is_array()) return false;
    for (const nlohmann::json& e : root["entities"])
        if (e.value("shape", std::string()) == shape) return true;
    return false;
}

// A recipe that plans corridor freeways is solved and baked by the loader (corridorAuthor +
// bakeCorridorIntoNet), which is not reproducible from the JSON alone.
bool wantsCorridors(const nlohmann::json& root) {
    if (hasShape(root, "corridor")) return true;
    if (!root.contains("entities") || !root["entities"].is_array()) return false;
    for (const nlohmann::json& e : root["entities"]) {
        if (e.value("shape", std::string()) != "road" || !e.contains("road")) continue;
        const nlohmann::json& r = e["road"];
        if (r.contains("generate") && r["generate"].is_object() &&
            r["generate"].value("corridor_freeways", false))
            return true;
    }
    return false;
}

}  // namespace

bool cityPrePassApplies(const nlohmann::json& root) {
    if (!root.contains("terrain")) return false;         // the lot pre-pass only runs with terrain
    if (hasShape(root, "script")) return false;          // a Lua pre-pass shapes the ground; not JSON-derivable
    if (hasShape(root, "lanelab")) return false;         // the lanelab `lots` producer owns those
    if (wantsCorridors(root)) return false;              // the corridor solve happens in the loader
    bool generated = false;
    if (root.contains("entities") && root["entities"].is_array())
        for (const nlohmann::json& e : root["entities"])
            if (e.value("shape", std::string()) == "road" && e.contains("road") &&
                e["road"].contains("generate"))
                generated = true;
    return generated;
}

bool cityPrePassForLevel(const nlohmann::json& rootIn, CityPrePass& out) {
    if (!cityPrePassApplies(rootIn)) return false;

    // 0. The loader MUTATES the level before the road pre-pass: propagateWaterSeaLevel defaults every
    //    road recipe's buildability sea_level from the top-level water block, so the waterline that
    //    draws is the waterline the city respects. Both pipelines run it "so they gate on an identical
    //    graph" (level_params.h) — and so must this one, or a level with water grows a DIFFERENT city
    //    here than at load. coast_city caught exactly that: one building 1.11 m under the drawn ground.
    nlohmann::json root = rootIn;
    propagateWaterSeaLevel(root);

    // 1. The NATURAL surface. erodedBase is the shared erosion bake (cache/terrain/<hash>.bin), so
    //    roads, lots and the drawn ground all conform to one eroded surface.
    out.natural = std::make_shared<TerrainParams>(readTerrainParams(root["terrain"]));
    out.natural->erodedBase = readErodedBase(root);
    out.noise = std::make_shared<Noise>(root["terrain"].value("seed", 0u));
    {
        auto tp = out.natural; auto nz = out.noise;
        out.naturalGround = [tp, nz](double x, double z) { return terrainHeight(*tp, *nz, x, z); };
    }

    // The finest rendered CDLOD cell: the walkway sculptors sample ground through the tile's own
    // interpolation on this grid, so ribbons sit on the MESH and not on the analytic function between.
    if (const nlohmann::json& tj = root["terrain"]; tj.contains("cdlod")) {
        const nlohmann::json& cj = tj["cdlod"];
        const double worldHalf = cj.is_object() ? cj.value("worldHalf", 1024.0) : 1024.0;
        const int numLods = cj.is_object() ? cj.value("numLods", 6) : 6;
        const int gridRes = cj.is_object() ? cj.value("gridRes", 32) : 32;
        out.lotMeshCell = (worldHalf * 2.0 / double(1 << (numLods - 1))) / std::max(1, gridRes);
    }

    // 2. The road nets, recipes run against the natural ground (which gates the metro's
    //    terrain-aware layout), and the carve regions they ask the terrain for.
    if (root.contains("entities") && root["entities"].is_array())
        for (const nlohmann::json& ent : root["entities"]) {
            if (ent.value("shape", std::string()) != "road") continue;
            const nlohmann::json roadBlock = ent.contains("road") ? ent["road"] : nlohmann::json::object();
            RoadEntity net = roadNetFromJson(roadBlock);
            if (roadBlock.contains("generate"))
                applyGenerateRecipe(net, roadBlock["generate"], out.naturalGround);
            std::vector<TerrainFlatten> r = roadNetConformRegions(net, out.naturalGround);
            out.roadFlatten.insert(out.roadFlatten.end(), r.begin(), r.end());
            out.nets.push_back(std::move(net));
        }
    if (out.nets.empty()) return false;

    // 3. The freeway right-of-way the lot pass builds (and re-zones) around.
    for (const RoadEntity& net : out.nets) {
        RoadGraph g = navRoadGraph(net, out.naturalGround);
        const int base = static_cast<int>(out.freewayROW.nodes.size());
        for (const RoadNode& n : g.nodes) out.freewayROW.nodes.push_back(n);
        for (RoadEdge e : g.edges) {
            if (e.klass != RoadClass::Freeway && e.klass != RoadClass::Ramp) continue;
            e.a += base; e.b += base;
            out.freewayROW.edges.push_back(e);
        }
    }

    // 4. The CARVED params the lots stand on: the level's own flatten records, then the road carve,
    //    then the earthwork field fitted to that carve against NATURAL ground. The earthwork goes
    //    into these params only — never into the natural sampler behind the road solve, or the deck
    //    and the carve would stop agreeing.
    out.lotParams = std::make_shared<TerrainParams>(readTerrainParams(root["terrain"]));
    out.lotParams->erodedBase = out.natural->erodedBase;
    out.lotParams->flatten.insert(out.lotParams->flatten.end(), out.roadFlatten.begin(), out.roadFlatten.end());
    {
        const double sea = root.contains("water")
                               ? root["water"].value("seaLevel", out.lotParams->seaLevel)
                               : out.lotParams->seaLevel;
        out.lotParams->earthwork =
            buildEarthworkField(out.roadFlatten, out.naturalGround, out.lotParams->earthworkParams, sea, nullptr);
    }
    rebuildFlattenIndex(*out.lotParams);
    {
        auto tp = out.lotParams; auto nz = out.noise;
        out.lotGround = [tp, nz](double x, double z) { return terrainHeight(*tp, *nz, x, z); };
        // Fold the in-pass block grades into the SAME region list as the roads, so priorities
        // resolve the way the final terrain will.
        out.lotGroundWith = [tp, nz](const std::vector<TerrainFlatten>& extra) {
            auto t2 = std::make_shared<TerrainParams>(*tp);
            t2->flatten.insert(t2->flatten.end(), extra.begin(), extra.end());
            rebuildFlattenIndex(*t2);
            return [t2, nz](Real x, Real z, Real dilate) {
                return terrainHeight(*t2, *nz, x, z, static_cast<double>(dilate));
            };
        };
    }
    return true;
}

}  // namespace engine
