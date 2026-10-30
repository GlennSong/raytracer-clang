#include "engine/lot_grow_setup.h"

#include "engine/level_params.h"
#include "engine/script_assets.h"
#include "log.h"
#ifdef RT_ENABLE_SCRIPTING
#include "engine/scripting/procgen_bindings.h"
#include "engine/scripting/script_vm.h"
#endif

namespace engine {

bool authoredSpawnXZ(const nlohmann::json& root, Vec2& out) {
    if (!root.contains("player")) return false;
    const nlohmann::json& pj = root["player"];
    if (!pj.contains("position") || !pj["position"].is_array() || pj["position"].size() < 3) return false;
    out = Vec2(pj["position"][0].get<double>(), pj["position"][2].get<double>());
    return true;
}

LotGrowSetup lotGrowSetupForLevel(const nlohmann::json& cs, const std::string& levelDir, const HeightField& ground,
                                  const std::vector<RoadEntity>& nets, LotGroundWithFn groundWith, double groundMeshCell,
                                  const Vec2* enterableAt) {
    LotGrowSetup s;
    readLotGrowParams(cs, s.lp);
    // Polycentric zoning: the level names its districts (citysim.districts, read above).
    // Terrain: buildings grow from their graded pad plane, park/green pads drape per-vertex.
    s.lp.groundWith = std::move(groundWith);
    s.lp.groundMeshCell = static_cast<Real>(groundMeshCell);
    // Enterable buildings (ADR-0080): the spawn building only, for now.
    if (enterableAt) s.lp.enterableAt.push_back(*enterableAt);
    if (ground) s.lp.ground = [ground](Real x, Real z) { return static_cast<Real>(ground(x, z)); };
#ifdef RT_ENABLE_SCRIPTING
    {   // The STYLE BOOK (the architect's Lua DATA layer): per-recipe look overrides. The vm must outlive
        // growLotBuildings (the hook holds it), so the setup carries it.
        const std::string sb = loadScriptCode("style_book.lua", levelDir);
        if (const std::string p = resolveScriptPath("style_book.lua", levelDir); !p.empty()) s.scriptFiles.push_back(p);
        if (!sb.empty()) {
            auto vm = std::make_shared<ScriptVM>();
            openProcgenLibrary(*vm);
            std::string err;
            auto hook = makeStyleBook(*vm, sb, &err);
            if (hook) { s.lp.styleHook = std::move(hook); s.styleVm = vm; }
            else if (!err.empty()) LOG_WARN << "style_book.lua: " << err;
        }
    }
    {   // The ARCHETYPE BOOK (the architect's Lua SELECTION layer), resolved to plain data here.
        // ALL-OR-NOTHING: a book with any invalid entry is rejected, never half applied.
        const std::string ab = loadScriptCode("archetype_book.lua", levelDir);
        if (const std::string p = resolveScriptPath("archetype_book.lua", levelDir); !p.empty()) s.scriptFiles.push_back(p);
        if (!ab.empty()) {
            ScriptVM vm;
            openProcgenLibrary(vm);
            std::string err;
            ArchetypeBook book = makeArchetypeBook(vm, ab, &err);
            if (!err.empty()) LOG_ERROR << "archetype_book.lua REJECTED (all-or-nothing): " << err;
            else s.lp.archetypeBook = std::move(book);
        }
    }
#else
    (void)levelDir;
#endif
    // The sidewalk datum for urban paving (ADR-0086): the lattice carves the ground kRoadConformStep under
    // the deck and the sidewalk slab stands `curb` over it. lanelab sets its own (lanesSidewalkRise).
    if (!nets.empty()) s.lp.sidewalkRise = static_cast<Real>(kRoadConformStep + nets.front().look.curb);
    if (!nets.empty()) s.lp.sidewalkWidth = static_cast<Real>(nets.front().look.sidewalk);   // the plate reaches the band
    s.wantFlat = cs.value("facadeDistance", 0.0) > cs.value("detailDistance", 700.0);
    s.planOnly = !cs.value("buildLots", false) && cs.value("planOnly", false);
    return s;
}

LotGround lotGroundFor(std::shared_ptr<TerrainParams> params, unsigned seed) {
    LotGround g;
    g.params = params;
    g.noise = std::make_shared<Noise>(seed);
    auto tp0 = params;
    auto nz = g.noise;
    g.ground = [tp0, nz](double x, double z) { return terrainHeight(*tp0, *nz, x, z); };
    g.groundWith = [tp0, nz](const std::vector<TerrainFlatten>& extra) {
        auto tp = std::make_shared<TerrainParams>(*tp0);
        tp->flatten.insert(tp->flatten.end(), extra.begin(), extra.end());
        rebuildFlattenIndex(*tp);
        return [tp, nz](Real x, Real z, Real dilate) {
            return terrainHeight(*tp, *nz, x, z, static_cast<double>(dilate));
        };
    };
    return g;
}

}  // namespace engine
