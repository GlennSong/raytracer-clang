#ifndef RAYTRACER_ENGINE_CITY_PLANNER_H
#define RAYTRACER_ENGINE_CITY_PLANNER_H

#include "world.h"
#include "../renderer/renderer.h"   // MeshHandle
#include <string>

namespace engine {

class AssetManager;
class CameraSystem;

// What a planner regenerate reports back to the panel, so tuning is measured
// rather than eyeballed (P7.3 phase 1). `spans` counts junction-to-junction
// chains walked through degree-2 curve nodes — the same metric the metro span
// audits gate on — not raw edges (an arterial is a chain of curve samples).
struct CityPlannerStats {
    int nodes = 0;
    int edges = 0;
    int spans = 0;
    int hubs = 0;
    double regenMs = 0.0;
    bool ok = false;   // false: no generated road entity / unparsable recipe
};

// The engine side of the City Planner dock (P7.3 phase 1) — host-agnostic:
// the Qt panel talks to it through EditorBridge, and nothing here knows a
// widget. The fast-iteration contract:
//   applyRecipe  -> applyGenerateRecipe GRAPH-ONLY (+ corridor re-bake) and
//                   rebuild the schematic overlay layers. Never meshes the
//                   carriageway — regenerateRoad's per-tick uploadMesh path
//                   leaks (editor_system.cpp TECH_DEBT) and meshing an 8km
//                   city per slider tick is the slow loop this tool retires.
//   bakeMesh     -> the one explicit mesh build, routed through
//                   AssetManager::acquireMesh/releaseMesh so repeated bakes
//                   swap the GPU mesh instead of leaking it.
//   overlays     -> merged strokeRibbon meshes per layer (hubs / arterials /
//                   nodes) on InstanceGroup entities with huge bounds — the
//                   city_render block-outline pattern; rebuilt ONLY on
//                   regenerate, toggled by clearing/restoring the group's one
//                   identity instance.
//   cameraPreset -> Top / Iso plan views on the ORBIT controller (the ortho-
//                   capable one: orthoHeight scales with distance), Free
//                   restores the perspective fly camera.
class CityPlanner {
public:
    // Wired by EditorSystem::onStart alongside the bridge attach; same-thread
    // by construction (the shell's UI and the engine frame interleave on one
    // event loop). `assets`/`renderer`/`cameras` may be null in headless
    // hosts — every consumer degrades (no overlays / no bake / no presets).
    void attach(World* world, AssetManager* assets, Renderer* renderer,
                CameraSystem* cameras);
    void detach();
    bool attached() const { return world_ != nullptr; }

    // The generate-block JSON of the planner's road entity ("" if the level
    // has no generated road).
    std::string recipe();

    // Write `generateJson` back into the road's SourceSpec recipe (preserving
    // the round-trip — roadRecipeForSave keeps the generate block, never bakes
    // nodes), regenerate the graph, and rebuild the overlays.
    CityPlannerStats applyRecipe(const std::string& generateJson);

    // The explicit full carriageway mesh build (buildRoadNetMesh), on demand.
    bool bakeMesh();

    // Start fresh on the terrain: empty the road's graph (the same hygiene
    // set a regenerate clears), release the baked carriageway mesh, and drop
    // every overlay — bare ground. The SourceSpec recipe SURVIVES, so the
    // next Regenerate rebuilds from the dials, not from nothing.
    bool clearRoads();

    // Layer visibility: "footprint" | "hubs" | "arterials" | "nodes". State
    // persists across regenerates; overlays are only REBUILT by applyRecipe.
    void setLayer(const std::string& name, bool on);

    // 0 = Top (orbit ortho, straight down), 1 = Iso (orbit ortho, yaw 45 /
    // pitch 35.264), 2 = Free (fly, perspective).
    void cameraPreset(int preset);

    // The road entity the planner drives: the first RoadNet whose SourceSpec
    // recipe carries a "generate" block.
    Entity roadEntity();

private:
    struct Layer {
        Entity group;        // InstanceGroup entity (created on first rebuild)
        MeshHandle mesh;     // the acquired merged ribbon mesh (planner-owned ref)
        bool visible = true; // desired state; survives detach/reattach
    };

    Layer* layerByName(const std::string& name);
    void rebuildOverlays();
    void publishLayer(Layer& layer, const RenderMesh& mesh, const char* name);

    World* world_ = nullptr;
    AssetManager* assets_ = nullptr;
    Renderer* renderer_ = nullptr;
    CameraSystem* cameras_ = nullptr;

    Layer footprint_, hubs_, arterials_, nodes_;
    MeshHandle bakedMesh_;     // last bake we acquired (released on the next one)
    uint64_t rev_ = 0;         // uniquifies acquireMesh keys across rebuilds
};

}  // namespace engine

#endif
