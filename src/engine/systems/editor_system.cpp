#include "editor_system.h"
#include <cstdlib>

#include "../components.h"
#include "../procgen/city/road_net.h"   // editable road regen (ADR-0049)
#include "../procgen/city/corridor_plan.h"   // Regenerate re-bakes freeways (roads-v2 S3)
#include "../imgui_properties.h"
#include "../mesh_builder.h"
#include "../level_writer.h"
#include "../model_importer.h"
#include "../../log.h"
#include <algorithm>
#include <cstdio>
#include <cstring>

#ifdef RT_ENABLE_IMGUI
#include <imgui.h>
// Gizmos degrade gracefully when the ImGuizmo submodule isn't fetched (the
// build warns); everything else in the editor still works.
#if __has_include(<ImGuizmo.h>)
#include <ImGuizmo.h>
#define RT_HAS_IMGUIZMO 1
#endif
#endif

namespace engine {

namespace {

// Half-size of the clickable box / drawn marker for a group (null object),
// in world units at the group's origin.
constexpr Real GROUP_MARKER_HALF = 0.3;

// Mouse (window coords) -> world-space pick ray. NDC depth follows the
// projection's [0,1] convention (ADR-0009): the near plane unprojects at
// z = 0 and the far plane at z = 1, so the ray runs eye -> scene. (Getting
// this backwards puts the origin on the FAR plane looking back at the
// camera, which selects the furthest of any overlapping objects.)
struct PickRay {
    Vec3 origin;
    Vec3 direction;
};

// The projection the ACTUAL camera uses — perspective OR orthographic. Every
// screen-space overlay (group markers, camera frustums, path handles,
// selection rings, gizmo) must project through this; hardcoding perspective
// misprojected them all in the City Planner's ortho plan views (P7.3).
Mat4 cameraProjection(const CameraState& cam) {
    return (cam.projection == CameraProjection::Perspective)
        ? Mat4::perspective(degreesToRadians(cam.fovDegrees), cam.aspectRatio,
                            cam.nearPlane, cam.farPlane)
        : Mat4::orthographic(cam.orthoHeight, cam.aspectRatio,
                             cam.nearPlane, cam.farPlane);
}

PickRay rayThroughCursor(const FrameContext& ctx) {
    const CameraState& cam = ctx.view.camera;
    Mat4 view = Mat4::lookAt(cam.position, cam.target, cam.up);
    Mat4 proj = cameraProjection(cam);
    Mat4 invVP = (proj * view).inverse();

    double w = std::max(ctx.windowWidth, 1);
    double h = std::max(ctx.windowHeight, 1);
    double ndcX = 2.0 * ctx.input.mouseX / w - 1.0;
    double ndcY = 1.0 - 2.0 * ctx.input.mouseY / h;

    Vec3 nearP = invVP.transformPoint(Vec3(ndcX, ndcY, 0.0));
    Vec3 farP = invVP.transformPoint(Vec3(ndcX, ndcY, 1.0));
    return {nearP, normalize(farP - nearP)};
}

// Smallest positive t where the ray enters the sphere; negative on miss.
double raySphere(const PickRay& ray, const Vec3& center, double radius) {
    Vec3 oc = ray.origin - center;
    double b = dot(oc, ray.direction);
    double c = oc.lengthSquared() - radius * radius;
    double disc = b * b - c;
    if (disc < 0.0) return -1.0;
    double t = -b - std::sqrt(disc);
    if (t < 0.0) t = -b + std::sqrt(disc);   // origin inside the sphere
    return t;
}

// Ray vs the mesh's model-space box (slab test in local space — the full
// inverse handles rotation and non-uniform scale). Direction is NOT
// re-normalized after the transform, so t stays the world-space parameter
// and is comparable across entities. Negative on miss. The sphere stays as
// the broad phase; this is what makes flat things (planes!) pickable only
// where they actually are.
double rayBox(const PickRay& ray, const Mat4& model, const BoundingSphere& b) {
    Mat4 inv = model.inverse();
    Vec3 o = inv.transformPoint(ray.origin);
    Vec3 d = inv.transformDirection(ray.direction);

    const double pad = 0.02;   // grace for zero-thickness shapes
    const double lo[3] = {b.boxMin.x - pad, b.boxMin.y - pad, b.boxMin.z - pad};
    const double hi[3] = {b.boxMax.x + pad, b.boxMax.y + pad, b.boxMax.z + pad};
    const double orig[3] = {o.x, o.y, o.z};
    const double dir[3] = {d.x, d.y, d.z};

    double tmin = 0.0, tmax = 1e30;
    for (int axis = 0; axis < 3; axis++) {
        if (std::abs(dir[axis]) < 1e-12) {
            if (orig[axis] < lo[axis] || orig[axis] > hi[axis]) return -1.0;
            continue;
        }
        double t1 = (lo[axis] - orig[axis]) / dir[axis];
        double t2 = (hi[axis] - orig[axis]) / dir[axis];
        if (t1 > t2) std::swap(t1, t2);
        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);
        if (tmin > tmax) return -1.0;
    }
    return tmin > 0.0 ? tmin : (tmax > 0.0 ? tmax : -1.0);
}

#ifdef RT_ENABLE_IMGUI
// Our Mat4 is row-major with column-vector convention; ImGuizmo wants
// GL-style float16 column-major. The conversion is a transpose.
void toGizmo(const Mat4& m, float* out) {
    for (int c = 0; c < 4; c++)
        for (int r = 0; r < 4; r++)
            out[c * 4 + r] = static_cast<float>(m.m[r][c]);
}

Mat4 fromGizmo(const float* in) {
    Mat4 m;
    for (int c = 0; c < 4; c++)
        for (int r = 0; r < 4; r++)
            m.m[r][c] = in[c * 4 + r];
    return m;
}

// The selected entity's parent world matrix (identity if unparented) — the
// gizmo works in world space, so a manipulated world matrix converts back to
// the entity's LOCAL Transform via this matrix's inverse.
Mat4 parentWorldMatrix(World& world, Entity e) {
    SourceSpec* s = world.get<SourceSpec>(e);
    if (!s || s->parentId == 0) return Mat4();
    Entity parent = findByDocumentId(world, s->parentId);
    return parent.valid() ? worldMatrix(world, parent) : Mat4();
}

// Project a world point to viewport pixels for the background draw list.
// Returns false (and leaves `out` untouched) if the point is behind the
// camera, so callers can skip line segments that would wrap.
bool projectToScreen(const Mat4& view, const Mat4& proj,
                     const ImGuiViewport* vp, const Vec3& world, ImVec2& out) {
    Vec3 viewPos = view.transformPoint(world);
    if (viewPos.z > -1e-3) return false;   // -Z is forward
    Vec3 ndc = proj.transformPoint(viewPos);
    out.x = vp->Pos.x + (static_cast<float>(ndc.x) * 0.5f + 0.5f) * vp->Size.x;
    out.y = vp->Pos.y + (0.5f - static_cast<float>(ndc.y) * 0.5f) * vp->Size.y;
    return true;
}

// Draw a world-space line into the background draw list, clipped to points in
// front of the camera (skipped entirely if either end is behind).
void drawWorldLine(const Mat4& view, const Mat4& proj, const ImGuiViewport* vp,
                   const Vec3& a, const Vec3& b, ImU32 color, float thickness) {
    ImVec2 pa, pb;
    if (projectToScreen(view, proj, vp, a, pa) &&
        projectToScreen(view, proj, vp, b, pb))
        ImGui::GetBackgroundDrawList()->AddLine(pa, pb, color, thickness);
}
#endif

// Sensible default sizes per shape (loader semantics: x = radius, y = height).
Vec3 defaultShapeSize(const std::string& shape) {
    if (shape == "sphere")   return Vec3(0.5, 0, 0);
    if (shape == "cylinder") return Vec3(0.5, 1, 0);
    if (shape == "cone")     return Vec3(0.5, 1, 0);
    if (shape == "capsule")  return Vec3(0.4, 1, 0);
    if (shape == "torus")    return Vec3(0.6, 0.2, 0);
    if (shape == "plane")    return Vec3(2, 2, 0);
    return Vec3(1, 1, 1);   // box, wedge
}

// Rebuild an editable road's carriageway from its RoadNet and keep the saved recipe in sync, after a
// property edit or a node drag (ADR-0049). A generated road keeps its recipe — roadRecipeForSave
// preserves the "generate" block instead of baking the nodes (road-network-v2-plan T2.1).
void regenerateRoad(World& world, Entity e, Renderer& renderer) {
    RoadNet* net = world.get<RoadNet>(e);
    if (!net) return;
    if (Renderable* r = world.get<Renderable>(e)) {
        RenderMesh mesh = buildRoadNetMesh(*net);
        if (!mesh.vertices.empty()) r->mesh = renderer.uploadMesh(mesh);
    }
    if (SourceSpec* spec = world.get<SourceSpec>(e))
        spec->recipe = roadRecipeForSave(spec->recipe, *net).dump();
}

// Rebuild a generated road from its (edited) generate recipe — the tuning panel's Regenerate. Re-runs
// buildDistrict from the updated params (which the caller has written into spec.recipe), re-meshes,
// and keeps the recipe a generate recipe. No-op for a hand-authored road. (road-network-v2-plan T2.3)
void regenerateRoadFromRecipe(World& world, Entity e, Renderer& renderer) {
    RoadNet* net = world.get<RoadNet>(e);
    SourceSpec* spec = net ? world.get<SourceSpec>(e) : nullptr;
    if (!spec) return;
    nlohmann::json recipe = nlohmann::json::parse(spec->recipe, nullptr, false);
    if (!recipe.is_object() || !recipe.contains("generate")) return;
    applyGenerateRecipe(*net, recipe["generate"]);
    // Roads-v2 S3: a metro that grew freeway plans gets them RE-BAKED into the
    // fresh graph (the same plan -> land -> author -> bake pipeline the loader
    // runs) — without this, Regenerate on a baked level silently dropped the
    // freeway and its ramps from the editable graph. The corridor's RENDERED
    // deck entities are load-time and stay stale until reload (like terrain,
    // lots and signals do on a regen); the GRAPH stays whole and editable.
    if (const int baked = rebakeNetCorridors(
            *net, recipe["generate"].value("interchange_spacing", 700.0)))
        LOG_INFO << "[bake] regenerate: " << baked
                 << " corridor(s) re-baked into the editable graph";
    if (Renderable* r = world.get<Renderable>(e)) {
        RenderMesh mesh = buildRoadNetMesh(*net);
        if (!mesh.vertices.empty()) r->mesh = renderer.uploadMesh(mesh);
    }
    spec->recipe = roadRecipeForSave(spec->recipe, *net).dump();
}

}  // namespace

EditorSystem::EditorSystem(CameraSystem& cameras, std::string levelFile,
                           PlayFactory makePlayState, EditorBridge* bridge)
    : cameras(cameras), levelFile(std::move(levelFile)),
      makePlayState(std::move(makePlayState)), bridge(bridge) {}

// "Conform terrain to roads" (G): re-grade the CDLOD terrain so the ground meets every
// road, then re-drape the roads on the carved result (ADR-0044 corridor conforming).
void EditorSystem::conformTerrainToRoads(FrameContext& ctx) {
    TerrainLodConfig* cfg = nullptr;
    ctx.world.each<TerrainLodConfig>([&](Entity, TerrainLodConfig& c) { if (!cfg) cfg = &c; });
    if (!cfg) { LOG_WARN << "Conform: no CDLOD terrain to grade"; return; }

    auto noise = std::make_shared<Noise>(cfg->seed);
    // Natural ground = the level's non-road grading only; road profiles are measured
    // against THIS (never against terrain a previous conform already cut for the road).
    TerrainParams naturalParams = cfg->params;
    naturalParams.flatten = cfg->baseFlatten;
    rebuildFlattenIndex(naturalParams);   // index must match this flatten, not the copy's (ADR-0075 P0)
    std::function<double(double, double)> natural =
        [naturalParams, noise](double x, double z) {
            return terrainHeight(naturalParams, *noise, x, z);
        };

    // Fresh cut/fill footprints from every road, measured on the natural ground.
    std::vector<TerrainFlatten> roads;
    ctx.world.each<RoadNet>([&](Entity, RoadNet& net) {
        net.heightAt = natural;
        std::vector<TerrainFlatten> r = roadNetConformRegions(net);
        roads.insert(roads.end(), r.begin(), r.end());
    });

    // Re-grade: terrain flatten = base + fresh roads; bump the revision so the LOD system
    // rebuilds its tiles + collider window from the new params.
    cfg->params.flatten = cfg->baseFlatten;
    cfg->params.flatten.insert(cfg->params.flatten.end(), roads.begin(), roads.end());
    rebuildFlattenIndex(cfg->params);   // re-index the re-conformed set (ADR-0075 P0)
    ++cfg->revision;

    // Re-drape every road on the freshly carved terrain so it sits exactly on its profile.
    TerrainParams carvedParams = cfg->params;
    std::function<double(double, double)> carved =
        [carvedParams, noise](double x, double z) {
            return terrainHeight(carvedParams, *noise, x, z);
        };
    ctx.world.each<RoadNet>([&](Entity e, RoadNet& net) {
        net.heightAt = carved;
        regenerateRoad(ctx.world, e, ctx.renderer);
    });
    LOG_INFO << "Conformed terrain to " << roads.size() << " road footprint(s)";
}

void EditorSystem::onStart(FrameContext& ctx) {
    ctx.actions.bindButton("gizmo_translate", KeyCode::Num1);
    ctx.actions.bindButton("gizmo_rotate", KeyCode::Num2);
    ctx.actions.bindButton("gizmo_scale", KeyCode::Num3);
    ctx.actions.bindButton("editor_frame", KeyCode::F);   // detach is disabled here
    // Held while road topology gestures are armed (Ctrl): see updatePathEdit.
    ctx.actions.bindButton("road_edit_modifier", KeyCode::LeftControl);
    ctx.actions.bindButton("road_edit_modifier", KeyCode::RightControl);
    ctx.actions.bindButton("editor_conform", KeyCode::G);   // grade terrain to the roads
    if (bridge) {
        bridge->attach(&ctx.world, this, levelFile);
        // City Planner (P7.3): the planner needs services that never cross
        // the bridge otherwise — the asset manager (leak-free overlay/bake
        // meshes via acquire/release) and the camera system (Top/Iso/Free).
        bridge->attachPlanner(&ctx.assets, &ctx.renderer, &cameras);
    }

    ComponentRegistry& registry = componentRegistry();
    undoStack();   // session log exists before any panel asks for it

    // Post-edit hook: a Size (or bulk/undo) write to a shape rebuilds its
    // mesh — what makes Size editable from EVERY inspector, not just the one
    // that knows the renderer. (The replaced upload leaks; see the mesh-cache
    // item in docs/TECH_DEBT.md.) The renderer outlives editor states, so the
    // captured pointer stays valid across sessions.
    Renderer* renderer = &ctx.renderer;
    if (ComponentRegistry::Entry* shape = registry.find("Shape")) {
        shape->onEdited = [renderer](World& world, Entity e, const char* label) {
            if (label && std::strcmp(label, "Size") != 0) return;
            SourceSpec* spec = world.get<SourceSpec>(e);
            Renderable* r = world.get<Renderable>(e);
            if (!spec || !r || !spec->meshFile.empty()) return;
            if (!spec->recipe.empty()) return;   // recipe entities (trees) don't rebuild from Size
            RenderMesh mesh = MeshBuilder::shape(spec->shape, spec->size);
            if (!mesh.vertices.empty()) r->mesh = renderer->uploadMesh(mesh);
        };
    }
    // Editable road (ADR-0049): any inspector edit (Width, sidewalk, ...) — or an
    // undo/paste (label == null) — regenerates the carriageway and re-syncs the
    // saved recipe, so widening a road is live and round-trips.
    if (ComponentRegistry::Entry* road = registry.find("Road")) {
        road->onEdited = [renderer](World& world, Entity e, const char*) {
            regenerateRoad(world, e, *renderer);
        };
    }
}

void EditorSystem::frameSelected(FrameContext& ctx) {
    Transform* t = ctx.world.get<Transform>(selected);
    if (!t) return;
    Mat4 wm = worldMatrix(ctx.world, selected);
    Vec3 worldPos = Vec3(wm.m[0][3], wm.m[1][3], wm.m[2][3]);
    Renderable* r = ctx.world.get<Renderable>(selected);
    Real radius = 1.0;
    if (r) {
        BoundingSphere bounds = ctx.renderer.getMeshBounds(r->mesh);
        Vec3 cx(wm.m[0][0], wm.m[1][0], wm.m[2][0]);
        Vec3 cy(wm.m[0][1], wm.m[1][1], wm.m[2][1]);
        Vec3 cz(wm.m[0][2], wm.m[1][2], wm.m[2][2]);
        Real maxScale = std::max({cx.length(), cy.length(), cz.length()});
        radius = std::max(bounds.radius * maxScale, Real(0.5));
    }
    // Keep the current view direction; back off far enough to see the object.
    FlyCameraController& fly = cameras.flyController();
    fly.eye = worldPos - fly.forward() * (radius * 3.5);
}

void EditorSystem::onStop(FrameContext&) {
    // Play (or app shutdown) tears the editor state down: the shell's panels
    // see a detached bridge and gray out rather than touching gameplay state.
    if (bridge) bridge->detach();
}

void EditorSystem::update(FrameContext& ctx) {
    if (ctx.actions.pressed("gizmo_translate")) gizmoOp = 0;
    if (ctx.actions.pressed("gizmo_rotate")) gizmoOp = 1;
    if (ctx.actions.pressed("gizmo_scale")) gizmoOp = 2;

    if (ctx.actions.pressed("editor_frame") && ctx.world.alive(selected))
        frameSelected(ctx);

    if (ctx.actions.pressed("editor_conform"))
        conformTerrainToRoads(ctx);

    bool click = ctx.input.mouseLeftDown && !prevMouseLeft;
    prevMouseLeft = ctx.input.mouseLeftDown;
    bool active = !ctx.input.uiWantsMouse && !gizmoBusy;
    // A road's node/tangent handles get first refusal on the click; only a miss falls
    // through to object picking, so dragging a handle never re-selects under it.
    bool pathGrabbed = updatePathEdit(ctx, click && active);
    if (click && active && !pathGrabbed)
        pickAtCursor(ctx);

    processShellRequests(ctx);

    // Drop dead entities from the selection set (deletes, undo of a create).
    selection.erase(std::remove_if(selection.begin(), selection.end(),
                                   [&](Entity e) { return !ctx.world.alive(e); }),
                    selection.end());
    if (!ctx.world.alive(selected))
        selected = selection.empty() ? Entity{} : selection.back();

    // One chokepoint catches every way selection moves (pick, hierarchy,
    // undo, delete): the shell reacts on its next drain instead of polling.
    // A signature folds in the whole set, not just the primary, so adding to
    // a multi-selection still wakes the panels.
    std::size_t sig = selection.size();
    for (Entity e : selection) sig = sig * 1000003u + e.index;
    if (bridge && (!(selected == lastNoticedSelection) || sig != selectionSig)) {
        lastNoticedSelection = selected;
        selectionSig = sig;
        bridge->notify(EditorNotice::SelectionChanged);
    }
}

void EditorSystem::processShellRequests(FrameContext& ctx) {
    for (const std::string& what : pendingAdds) {
        if (what == "playerspawn") {
            // Only one spawn is meaningful: addPlayerSpawn reuses an existing one,
            // so record a create (undo) only when we actually made a new one.
            bool had = false;
            ctx.world.each<Transform, PlayerSpawn>(
                [&](Entity, Transform&, PlayerSpawn&) { had = true; });
            Entity e = addPlayerSpawn(ctx);
            if (e.valid()) {
                selected = e;
                if (!had) undoStack()->recordCreate(e);
            }
            continue;
        }
        Entity e = (what == "camera") ? cameras.placeCameraAtView(ctx)
                   : (what == "group") ? addGroup(ctx)
                                       : addPrimitive(ctx, what);
        if (e.valid()) {
            selected = e;
            undoStack()->recordCreate(e);
        }
    }
    pendingAdds.clear();

    if (pendingDuplicate) {
        pendingDuplicate = false;
        Entity copy = duplicateSelected(ctx);
        if (copy.valid()) {
            selected = copy;
            undoStack()->recordCreate(copy);
        }
    }
}

void EditorSystem::pickAtCursor(FrameContext& ctx) {
    PickRay ray = rayThroughCursor(ctx);

    Entity best;
    double bestT = 1e30;
    ctx.world.each<Transform, Renderable>(
        [&](Entity e, Transform&, Renderable& r) {
            BoundingSphere bounds = ctx.renderer.getMeshBounds(r.mesh);
            // World matrix so parented children are hit where they're drawn.
            Mat4 wm = worldMatrix(ctx.world, e);
            Vec3 center = wm.transformPoint(bounds.center);
            Vec3 cx(wm.m[0][0], wm.m[1][0], wm.m[2][0]);
            Vec3 cy(wm.m[0][1], wm.m[1][1], wm.m[2][1]);
            Vec3 cz(wm.m[0][2], wm.m[1][2], wm.m[2][2]);
            Real maxScale = std::max({cx.length(), cy.length(), cz.length()});
            // Broad phase: cheap sphere reject. Narrow: tight box hit.
            if (raySphere(ray, center, bounds.radius * maxScale) <= 0.0) return;
            double hit = rayBox(ray, wm, bounds);
            if (hit > 0.0 && hit < bestT) {
                bestT = hit;
                best = e;
            }
        });

    // Groups carry no mesh, so they're invisible to the pass above; test a
    // small fixed box at each group's world origin (matches the drawn marker)
    // so they're clickable in the viewport, not only from the hierarchy.
    BoundingSphere groupBox;
    groupBox.boxMin = Vec3(-GROUP_MARKER_HALF, -GROUP_MARKER_HALF, -GROUP_MARKER_HALF);
    groupBox.boxMax = Vec3(GROUP_MARKER_HALF, GROUP_MARKER_HALF, GROUP_MARKER_HALF);
    ctx.world.each<Transform, SourceSpec>(
        [&](Entity e, Transform&, SourceSpec& spec) {
            if (!spec.isGroup()) return;
            double hit = rayBox(ray, worldMatrix(ctx.world, e), groupBox);
            if (hit > 0.0 && hit < bestT) {
                bestT = hit;
                best = e;
            }
        });

    // Shift-click extends the selection (toggle the hit); plain click
    // replaces it. Shift-clicking empty space leaves the set alone.
    if (ctx.input.keyShift) {
        if (!best.valid()) return;
        std::vector<Entity> next = selection;
        auto it = std::find(next.begin(), next.end(), best);
        if (it != next.end()) {
            next.erase(it);
            setSelection(next, next.empty() ? Entity{} : next.back());
        } else {
            next.push_back(best);
            setSelection(next, best);
        }
    } else {
        if (best.valid()) setSelection({best}, best);
        else setSelection({}, Entity{});
    }
}

// Drive the path-edit tool for the selected road this frame (ADR-0050). Returns true
// when a handle was grabbed on this click, so update() skips object picking. Pure logic
// (no ImGui) — the matching draw lives in drawPathHandles().
bool EditorSystem::updatePathEdit(FrameContext& ctx, bool click) {
    if (std::getenv("RT_SELECT_ROAD") &&
        (!ctx.world.alive(selected) || !ctx.world.get<RoadNet>(selected)))
        ctx.world.each<RoadNet>([&](Entity e, RoadNet&) { selected = e; });   // headless handle capture
    RoadNet* net = ctx.world.alive(selected) ? ctx.world.get<RoadNet>(selected) : nullptr;
    if (!net) {
        if (pathTool.bound()) pathTool.bind(nullptr);
        roadSource.reset();
        pathEditEntity = Entity{};
        pathRoadNet = nullptr;
        pathWasDragging = false;
        return false;
    }
    // (Re)seat the handle source when the road — or its storage slot — changes, but never
    // mid-drag: the world isn't structurally mutated while dragging, so the pointer holds.
    if (!pathTool.dragging() && (!(pathEditEntity == selected) || pathRoadNet != net)) {
        roadSource = std::make_unique<RoadHandleSource>(*net);
        pathTool.bind(roadSource.get());
        pathEditEntity = selected;
        pathRoadNet = net;
    }
    // Rebuild the carriageway after each applied move (ctx is only touched synchronously,
    // inside drag() below, so capturing it by reference is safe).
    pathTool.onEdit([this, &ctx] { regenerateRoad(ctx.world, selected, ctx.renderer); });

    PickRay pr = rayThroughCursor(ctx);
    EditRay ray{pr.origin, pr.direction};
    const CameraState& cam = ctx.view.camera;
    Vec3 viewDir = normalize(cam.target - cam.position);
    // Pick fatness scales with view distance so the dots stay grabbable when zoomed out.
    double radius = 0.06 * (cam.position - cam.target).length() + 0.4;

    // Always refresh hover first, so the click handlers below know what's under the cursor
    // on the very frame of the click (updateHover self-suppresses while dragging).
    int hov = pathTool.updateHover(ray, radius);

    // Topology gestures (ADR-0049 ops, finally wired up). The carriageway height varies,
    // so land the cursor on the ground plane through the active node for a stable XZ.
    if (click && !pathTool.dragging()) {
        double planeY = 0.0;
        for (const EditHandle& h : pathTool.handles())
            if (h.index == pathTool.selectedNode() && h.kind == HandleKind::Knot)
                planeY = h.position.y;
        Vec3 ground = projectDrag(ray, Vec3(0, planeY, 0), DragPlane::Ground, viewDir);

        // Shift+click a node -> delete it.
        if (ctx.input.keyShift && hov >= 0) {
            int node = pathTool.handles()[hov].index;
            if (deleteRoadNode(ctx.world, selected, node, ctx.renderer)) pathTool.clearSelection();
            return true;
        }
        // Ctrl+click -> subdivide the edge under the cursor, or (over empty ground, with a
        // node selected) extend a new connected segment to the cursor. Either way, select
        // the new node so you can immediately drag or chain another extend.
        if (ctx.actions.held("road_edit_modifier") && hov < 0) {
            int edge = nearestRoadEdge(ctx.world, selected, ground, std::max(2.5, radius));
            int newNode = (edge >= 0)
                ? splitRoadEdge(ctx.world, selected, edge, ground, ctx.renderer)
                : (pathTool.selectedNode() >= 0
                       ? extendRoad(ctx.world, selected, pathTool.selectedNode(), ground, ctx.renderer)
                       : -1);
            if (newNode >= 0) pathTool.selectNode(newNode);
            if (edge >= 0 || newNode >= 0) return true;
        }
    }

    bool grabbed = false;
    if (click) grabbed = pathTool.beginDrag(ray, radius);
    else if (pathTool.dragging() && ctx.input.mouseLeftDown) pathTool.drag(ray, viewDir);
    if (pathWasDragging && !ctx.input.mouseLeftDown) pathTool.endDrag();
    pathWasDragging = pathTool.dragging();
    return grabbed;
}

Vec3 EditorSystem::spawnPoint(FrameContext& ctx) const {
    // A few meters in front of the view, so new objects appear where you look.
    const CameraState& cam = ctx.view.camera;
    return cam.position + normalize(cam.target - cam.position) * 5.0;
}

Entity EditorSystem::addPrimitive(FrameContext& ctx, const std::string& shape) {
    SourceSpec spec;
    spec.shape = shape;
    spec.size = defaultShapeSize(shape);

    RenderMesh mesh = MeshBuilder::shape(shape, spec.size);
    if (mesh.vertices.empty()) return Entity{};

    spec.id = nextDocumentId(ctx.world);
    Entity e = ctx.world.create();
    Transform t;
    t.position = spawnPoint(ctx);
    ctx.world.add<Transform>(e, t);
    ctx.world.add<PrevTransform>(e, {t});
    ctx.world.add<SourceSpec>(e, spec);

    Renderable r;
    r.mesh = ctx.renderer.uploadMesh(mesh);
    r.material.albedo = Vec3(0.7, 0.7, 0.7);
    r.material.roughness = 0.6f;
    ctx.world.add<Renderable>(e, r);
    return e;
}

Entity EditorSystem::addGroup(FrameContext& ctx) {
    // A null object: a named transform, no mesh. Parent things under it to
    // move them together. Drawn and pickable in the viewport as a small box.
    SourceSpec spec;
    spec.shape = "";   // marks it a group (isGroup())
    spec.name = "Group " + std::to_string(nextDocumentId(ctx.world));
    spec.id = nextDocumentId(ctx.world);

    Entity e = ctx.world.create();
    Transform t;
    t.position = spawnPoint(ctx);
    ctx.world.add<Transform>(e, t);
    ctx.world.add<PrevTransform>(e, {t});
    ctx.world.add<SourceSpec>(e, spec);
    return e;
}

Entity EditorSystem::addPlayerSpawn(FrameContext& ctx) {
    // The player start: a single, pickable green capsule. Its Transform is the
    // spawn — the LevelWriter syncs it into the level's "player" block (so it
    // round-trips via that block, not a SourceSpec, exactly like the loader's
    // loadPlayerSpawn). Only one spawn is meaningful, so if one already exists we
    // select it rather than scatter duplicates the writer would fight over.
    Entity existing{};
    ctx.world.each<Transform, PlayerSpawn>([&](Entity e, Transform&, PlayerSpawn&) {
        if (!existing.valid()) existing = e;
    });
    if (existing.valid()) return existing;

    Entity e = ctx.world.create();
    Transform t;
    t.position = spawnPoint(ctx);
    ctx.world.add<Transform>(e, t);
    ctx.world.add<PrevTransform>(e, {t});
    ctx.world.add<PlayerSpawn>(e);

    Renderable gizmo;
    gizmo.mesh = ctx.renderer.uploadMesh(MeshBuilder::capsule(0.3f, 0.8f));
    gizmo.material.albedo = Vec3(0.2, 0.8, 0.3);   // green = "you start here"
    gizmo.material.roughness = 0.5f;
    ctx.world.add<Renderable>(e, gizmo);
    return e;
}

Entity EditorSystem::duplicateSelected(FrameContext& ctx) {
    Transform* t = ctx.world.get<Transform>(selected);
    SourceSpec* spec = ctx.world.get<SourceSpec>(selected);
    if (!t || !spec) return Entity{};

    Entity e = ctx.world.create();
    Transform copy = *t;
    copy.position += Vec3(spec->size.x + 0.5, 0, 0);   // beside the original
    ctx.world.add<Transform>(e, copy);
    ctx.world.add<PrevTransform>(e, {copy});

    SourceSpec dup = *spec;
    dup.id = nextDocumentId(ctx.world);   // fresh id; keep parentId (a sibling)
    ctx.world.add<SourceSpec>(e, dup);

    if (Renderable* r = ctx.world.get<Renderable>(selected))
        ctx.world.add<Renderable>(e, *r);   // shares the uploaded mesh
    return e;
}

#ifdef RT_ENABLE_IMGUI

void EditorSystem::render(FrameContext& ctx) {
    // No ImGui context (e.g. a backend without debug-UI support): stay inert.
    if (ImGui::GetCurrentContext() == nullptr) return;
    // Headless visual capture (RT_FRAME_DUMP batteries like tools/car_shots.sh)
    // needs the exact fly camera that edit mode gives, but not the chrome drawn
    // over it — the toolbar covers the top-left third of the frame, which is
    // most of a parked car. Off by default; capture scripts set it and restore
    // settings.json afterwards.
    if (ctx.settings.getBool("hideEditorUi", false)) return;
#ifdef RT_HAS_IMGUIZMO
    ImGuizmo::BeginFrame();
#endif
    drawGrid(ctx);
    // Inside the Qt shell its panels carry the toolbar/inspector duties (and
    // hosted mode doesn't bridge ImGui keyboard input anyway) — only the
    // viewport overlays draw: grid, gizmo, selection ring.
    if (!bridge) {
        drawToolbar(ctx);
        drawInspector(ctx);
    }
    drawCameraFrustums(ctx);
    drawGroupMarkers(ctx);
    drawGizmo(ctx);
    drawSelectionMarker(ctx);
    drawPathHandles(ctx);
}

void EditorSystem::drawGroupMarkers(FrameContext& ctx) const {
    const CameraState& cam = ctx.view.camera;
    Mat4 view = Mat4::lookAt(cam.position, cam.target, cam.up);
    Mat4 proj = cameraProjection(cam);   // respect ortho plan views (P7.3)
    const ImGuiViewport* vp = ImGui::GetMainViewport();

    // A small axis-aligned wireframe box at each group's world origin, so a
    // null object reads as a thing you can click and grab.
    const Real h = GROUP_MARKER_HALF;
    const Vec3 corners[8] = {
        {-h, -h, -h}, {h, -h, -h}, {h, h, -h}, {-h, h, -h},
        {-h, -h, h},  {h, -h, h},  {h, h, h},  {-h, h, h}};
    const int edges[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
                              {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};

    ctx.world.each<Transform, SourceSpec>(
        [&](Entity e, Transform&, SourceSpec& spec) {
            if (!spec.isGroup()) return;
            Mat4 wm = worldMatrix(ctx.world, e);
            Vec3 w[8];
            for (int i = 0; i < 8; i++) w[i] = wm.transformPoint(corners[i]);
            ImU32 color = isSelected(e) ? IM_COL32(120, 220, 255, 235)
                                        : IM_COL32(120, 220, 255, 140);
            for (auto& edge : edges)
                drawWorldLine(view, proj, vp, w[edge[0]], w[edge[1]], color,
                              isSelected(e) ? 2.0f : 1.3f);
        });
}

void EditorSystem::drawCameraFrustums(FrameContext& ctx) const {
    // Show the framing pyramid for selected placed cameras: apex at the
    // camera, opening to a rectangle whose half-angles come from the lens.
    // Drawn to a capped distance (the real far plane is uselessly large) so
    // it reads as "what this camera sees".
    const CameraState& cam = ctx.view.camera;
    Mat4 view = Mat4::lookAt(cam.position, cam.target, cam.up);
    Mat4 proj = cameraProjection(cam);   // respect ortho plan views (P7.3)
    const ImGuiViewport* vp = ImGui::GetMainViewport();

    for (Entity e : selection) {
        SceneCamera* sc = ctx.world.get<SceneCamera>(e);
        Transform* t = ctx.world.get<Transform>(e);
        if (!sc || !t) continue;
        if (e == ctx.view.activeCameraEntity) continue;   // we're inside it

        Mat4 wm = worldMatrix(ctx.world, e);
        Real fov = degreesToRadians(sc->lens.verticalFovDegrees());
        Real aspect = cam.aspectRatio > 0 ? cam.aspectRatio : 16.0 / 9.0;
        Real dist = std::clamp(sc->lens.focusDistance, Real(2), Real(12));
        Real hh = dist * std::tan(fov * 0.5);
        Real hw = hh * aspect;

        Vec3 apex = wm.transformPoint(Vec3(0, 0, 0));
        Vec3 c[4] = {wm.transformPoint(Vec3(-hw, -hh, -dist)),
                     wm.transformPoint(Vec3(hw, -hh, -dist)),
                     wm.transformPoint(Vec3(hw, hh, -dist)),
                     wm.transformPoint(Vec3(-hw, hh, -dist))};
        ImU32 color = IM_COL32(255, 220, 90, 220);
        for (int i = 0; i < 4; i++) {
            drawWorldLine(view, proj, vp, apex, c[i], color, 1.5f);
            drawWorldLine(view, proj, vp, c[i], c[(i + 1) % 4], color, 1.5f);
        }
    }
}

void EditorSystem::drawGrid(FrameContext& ctx) const {
#ifndef RT_HAS_IMGUIZMO
    (void)ctx;
#else
    if (!ctx.settings.getBool("editorGrid", true)) return;

    const CameraState& cam = ctx.view.camera;
    Mat4 view = Mat4::lookAt(cam.position, cam.target, cam.up);
    Mat4 proj = cameraProjection(cam);   // respect ortho plan views (P7.3)
    float viewF[16], projF[16];
    toGizmo(view, viewF);
    toGizmo(proj, projF);
    static const float IDENTITY[16] = {1, 0, 0, 0, 0, 1, 0, 0,
                                       0, 0, 1, 0, 0, 0, 0, 1};
    ImGuiIO& io = ImGui::GetIO();
    ImGuizmo::SetOrthographic(cam.projection == CameraProjection::Orthographic);
    ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
    // Ground-plane reference (y = 0), drawn into ImGuizmo's overlay window —
    // over the scene, under every panel.
    ImGuizmo::DrawGrid(viewF, projF, IDENTITY, 24.0f);
#endif
}

void EditorSystem::drawToolbar(FrameContext& ctx) {
    ImGui::Begin("Editor");

    if (ImGui::Button("  Play  ")) {
        // Save first: Play loads the document fresh, so what you built is
        // exactly what runs (and nothing is lost if the game crashes).
        LevelWriter::save(levelFile, ctx.world);
        ctx.transition.replaceWith(makePlayState());
    }
    ImGui::SameLine();
    if (ImGui::Button("Play Here")) {
        // Same loop, but spawn the player at this view (one-shot flag the
        // play state consumes) — skip the walk from the level's spawn point.
        LevelWriter::save(levelFile, ctx.world);
        ctx.settings.setBool("playFromHere", true);
        ctx.transition.replaceWith(makePlayState());
    }
    ImGui::SameLine();
    if (ImGui::Button("Save")) LevelWriter::save(levelFile, ctx.world);
    ImGui::SameLine();
    if (ImGui::Button("Conform Terrain"))   // grade the ground to the roads (key: G)
        conformTerrainToRoads(ctx);
    ImGui::SameLine();
    ImGui::TextDisabled("%s", levelFile.c_str());

    // Undo/redo: buttons plus the universal chords (Ctrl+Z / Ctrl+Shift+Z).
    const ImGuiIO& io = ImGui::GetIO();
    bool wantUndo = io.KeyCtrl && !io.KeyShift &&
                    ImGui::IsKeyPressed(ImGuiKey_Z) && !io.WantTextInput;
    bool wantRedo = io.KeyCtrl && io.KeyShift &&
                    ImGui::IsKeyPressed(ImGuiKey_Z) && !io.WantTextInput;
    ImGui::BeginDisabled(!undo || !undo->canUndo());
    wantUndo |= ImGui::Button("Undo");
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!undo || !undo->canRedo());
    wantRedo |= ImGui::Button("Redo");
    ImGui::EndDisabled();
    if (undo && wantUndo && undo->canUndo()) selected = undo->undo(ctx.world);
    if (undo && wantRedo && undo->canRedo()) selected = undo->redo(ctx.world);

    ImGui::SeparatorText("Add");
    static const char* SHAPES[] = {"box", "sphere", "cylinder", "plane",
                                   "cone", "wedge", "torus", "capsule"};
    for (int i = 0; i < 8; i++) {
        if (ImGui::Button(SHAPES[i])) {
            selected = addPrimitive(ctx, SHAPES[i]);
            if (undo && selected.valid()) undo->recordCreate(selected);
        }
        if (i % 4 != 3) ImGui::SameLine();
    }
    if (ImGui::Button("camera")) {
        selected = cameras.placeCameraAtView(ctx);
        if (undo && selected.valid()) undo->recordCreate(selected);
    }
    ImGui::SameLine();
    if (ImGui::Button("group")) {
        selected = addGroup(ctx);
        if (undo && selected.valid()) undo->recordCreate(selected);
    }
    // Its own line: "player spawn" is wide and would clip off a narrow toolbar.
    if (ImGui::Button("player spawn")) {
        // Only one spawn may exist; addPlayerSpawn reuses it if present, so undo
        // records a create only when we actually made a new one.
        bool had = false;
        ctx.world.each<Transform, PlayerSpawn>(
            [&](Entity, Transform&, PlayerSpawn&) { had = true; });
        selected = addPlayerSpawn(ctx);
        if (undo && !had && selected.valid()) undo->recordCreate(selected);
    }

    ImGui::SetNextItemWidth(180);
    ImGui::InputText("##gltf", modelPath, sizeof(modelPath));
    ImGui::SameLine();
    if (ImGui::Button("Add glTF")) {
        // Resolve relative to the level's directory, like the loader does.
        std::string dir = levelFile;
        std::size_t slash = dir.find_last_of("/\\");
        dir = (slash == std::string::npos) ? "" : dir.substr(0, slash + 1);
        ImportedModel model = ModelImporter::load(dir + modelPath, ctx.renderer);
        if (model.meshes.empty()) {
            LOG_WARN << "No meshes in " << modelPath;
        } else {
            Transform t;
            t.position = spawnPoint(ctx);
            for (size_t i = 0; i < model.meshes.size(); i++) {
                Entity e = ctx.world.create();
                ctx.world.add<Transform>(e, t);
                ctx.world.add<PrevTransform>(e, {t});
                Renderable r;
                r.mesh = model.meshes[i].meshHandle;
                r.material = model.meshes[i].material;
                ctx.world.add<Renderable>(e, r);
                if (i == 0) {
                    SourceSpec spec;
                    spec.shape = "";
                    spec.meshFile = modelPath;
                    ctx.world.add<SourceSpec>(e, spec);
                    selected = e;
                    // Undo covers the document entity (the first sub-mesh);
                    // sibling sub-meshes share the known multi-mesh limits.
                    if (undo) undo->recordCreate(e);
                }
            }
        }
    }

    ImGui::SeparatorText("Gizmo");
    ImGui::RadioButton("Move (1)", &gizmoOp, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Rotate (2)", &gizmoOp, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Scale (3)", &gizmoOp, 2);

    ImGui::End();
}

void EditorSystem::drawInspector(FrameContext& ctx) {
    Transform* t = ctx.world.get<Transform>(selected);
    if (!t) {
        pendingEdit.active = false;
        return;
    }

    ImGui::Begin("Editor");
    ImGui::SeparatorText("Selected");

    if (SceneCamera* cam = ctx.world.get<SceneCamera>(selected))
        ImGui::Text("Camera: %s", cam->name.c_str());
    else if (SourceSpec* spec = ctx.world.get<SourceSpec>(selected))
        ImGui::Text("%s", spec->meshFile.empty() ? spec->shape.c_str()
                                                 : spec->meshFile.c_str());

    // Road Generation (road-network-v2-plan T2.2/T2.3): for a GENERATED road, sliders over the
    // district recipe with a LIVE regenerate, so you can watch the network rebuild as you tune it.
    // First in the inspector — it's the primary control for a generated city.
    if (RoadNet* rnet = ctx.world.get<RoadNet>(selected))
        if (SourceSpec* rspec = ctx.world.get<SourceSpec>(selected)) {
            nlohmann::json recipe = nlohmann::json::parse(rspec->recipe, nullptr, false);
            if (recipe.is_object() && recipe.contains("generate") && recipe["generate"].is_object()) {
                ImGui::SeparatorText("Road Generation");
                nlohmann::json& g = recipe["generate"];
                const bool metro = g.value("kind", std::string("district")) == "metro";
                float arteryW = static_cast<float>(g.value("artery_width", rnet->width * 1.6));
                float streetW = static_cast<float>(g.value("street_width", rnet->width));
                int   seed    = static_cast<int>(g.value("seed", 1u));
                bool  changed = false;

                if (metro) {
                    // A whole-metro network: hotspots linked by organic arterials,
                    // each block between them filled by grid/radial subdivision.
                    float radius   = static_cast<float>(g.value("radius", 700.0));
                    int   hotspots = g.value("hotspots", 6);
                    float blockSz  = static_cast<float>(g.value("block_size", 70.0));
                    bool  ringRoad = g.value("ring_road", false);
                    changed |= ImGui::SliderFloat("Radius", &radius, 200.0f, 1500.0f, "%.0f m");
                    changed |= ImGui::SliderInt("Hotspots", &hotspots, 2, 12);
                    changed |= ImGui::SliderFloat("Block size", &blockSz, 30.0f, 140.0f, "%.0f m");
                    changed |= ImGui::Checkbox("Ring road", &ringRoad);
                    changed |= ImGui::SliderFloat("Artery width", &arteryW, 4.0f, 30.0f, "%.1f m");
                    changed |= ImGui::SliderFloat("Street width", &streetW, 3.0f, 20.0f, "%.1f m");
                    changed |= ImGui::SliderInt("Seed", &seed, 1, 9999);
                    ImGui::SameLine();
                    if (ImGui::SmallButton("Reseed")) { seed = (seed * 1103515 + 12345) & 0x7fff; changed = true; }
                    if (changed) {
                        g["radius"] = radius;           g["hotspots"] = hotspots;
                        g["block_size"] = blockSz;      g["ring_road"] = ringRoad;
                        g["artery_width"] = arteryW;    g["street_width"] = streetW;
                        g["seed"] = seed;
                    }
                } else {
                    float bsNominal = static_cast<float>(g.value("block_size", 36.0));
                    float radius    = static_cast<float>(g.value("radius", 130.0));
                    int   arterials = g.value("arterials", 3);
                    float blockMax  = static_cast<float>(g.value("block_size_max", static_cast<double>(bsNominal)));
                    float blockMin  = static_cast<float>(g.value("block_size_min", bsNominal * 0.55));
                    float irregular = static_cast<float>(g.value("irregular", 0.22));
                    float jitter    = static_cast<float>(g.value("jitter", 0.16));
                    changed |= ImGui::SliderFloat("Radius", &radius, 40.0f, 1500.0f, "%.0f m");
                    changed |= ImGui::SliderInt("Arterials", &arterials, 0, 8);
                    changed |= ImGui::SliderFloat("Block max", &blockMax, 12.0f, 120.0f, "%.0f m");
                    changed |= ImGui::SliderFloat("Block min", &blockMin, 6.0f, 100.0f, "%.0f m");
                    changed |= ImGui::SliderFloat("Irregular", &irregular, 0.0f, 1.0f, "%.2f");
                    changed |= ImGui::SliderFloat("Jitter", &jitter, 0.0f, 0.5f, "%.2f");
                    changed |= ImGui::SliderFloat("Artery width", &arteryW, 4.0f, 30.0f, "%.1f m");
                    changed |= ImGui::SliderFloat("Street width", &streetW, 3.0f, 20.0f, "%.1f m");
                    changed |= ImGui::SliderInt("Seed", &seed, 1, 9999);
                    ImGui::SameLine();
                    if (ImGui::SmallButton("Reseed")) { seed = (seed * 1103515 + 12345) & 0x7fff; changed = true; }
                    if (changed) {
                        if (blockMin > blockMax) blockMin = blockMax; // keep the bracket sane
                        g.erase("block_size");                        // normalise to explicit min/max
                        g["radius"] = radius;            g["arterials"] = arterials;
                        g["block_size_max"] = blockMax;  g["block_size_min"] = blockMin;
                        g["irregular"] = irregular;      g["jitter"] = jitter;
                        g["artery_width"] = arteryW;     g["street_width"] = streetW;
                        g["seed"] = seed;
                    }
                }

                if (changed) {
                    rspec->recipe = recipe.dump();                    // panel writes the params...
                    regenerateRoadFromRecipe(ctx.world, selected, ctx.renderer);   // ...then rebuilds live
                }
            }
        }

    // Every section below is generated from describeProperties via the
    // registry — the same single source the Qt inspector renders from.
    ComponentRegistry& registry = componentRegistry();
    const auto& entries = registry.entries();
    for (std::size_t i = 0; i < entries.size(); i++) {
        const ComponentRegistry::Entry& entry = entries[i];
        if (!entry.has(ctx.world, selected)) continue;
        ImGui::PushID(static_cast<int>(i));
        ImGui::SeparatorText(entry.name.c_str());

        nlohmann::json preVisit =
            undo ? undo->componentState(ctx.world, selected, entry.name)
                 : nlohmann::json();

        ImGuiPropertyVisitor ui;
        entry.visit(ctx.world, selected, ui);

        if (ui.edited() && entry.onEdited)
            entry.onEdited(ctx.world, selected, ui.editedLabel());

        // Undo brackets one continuous interaction: state at activation,
        // recorded when the widget commits (drag release, typing done).
        if (undo) {
            if (ui.interactionStarted()) {
                pendingEdit = {true, selected, entry.name, std::move(preVisit)};
            }
            if (ui.interactionFinished() && pendingEdit.active &&
                pendingEdit.entity == selected &&
                pendingEdit.component == entry.name) {
                undo->recordFieldEdit(
                    selected, entry.name, ui.editedLabel(),
                    std::move(pendingEdit.before),
                    undo->componentState(ctx.world, selected, entry.name));
                pendingEdit.active = false;
            }
        }

        if (entry.removeFrom && ImGui::SmallButton("Remove")) {
            if (undo) undo->recordComponentRemove(ctx.world, selected, entry.name);
            entry.removeFrom(ctx.world, selected);
        }
        ImGui::PopID();
    }

    // Add Component: every registered type the entity lacks and that allows
    // menu attachment.
    if (ImGui::Button("Add Component..."))
        ImGui::OpenPopup("editor_add_component");
    if (ImGui::BeginPopup("editor_add_component")) {
        for (const auto& entry : entries) {
            if (!entry.addTo || entry.has(ctx.world, selected)) continue;
            if (ImGui::Selectable(entry.name.c_str())) {
                entry.addTo(ctx.world, selected);
                if (undo)
                    undo->recordComponentAdd(ctx.world, selected, entry.name);
            }
        }
        ImGui::EndPopup();
    }

    if (ImGui::Button("Duplicate")) {
        selected = duplicateSelected(ctx);
        if (undo && selected.valid()) undo->recordCreate(selected);
    }
    ImGui::SameLine();
    if (ImGui::Button("Delete")) {
        if (undo) undo->recordDelete(ctx.world, selected);
        ctx.world.destroy(selected);
        selected = Entity{};
    }

    // Cameras are stationary; keep the interpolation pair in sync so edits
    // don't smear across frames (true for all editor objects: nothing moves).
    if (ctx.world.alive(selected))
        if (auto* prev = ctx.world.get<PrevTransform>(selected))
            prev->value = *ctx.world.get<Transform>(selected);

    ImGui::End();
}

void EditorSystem::drawGizmo(FrameContext& ctx) {
#ifndef RT_HAS_IMGUIZMO
    (void)ctx;
    gizmoBusy = false;
    return;
#else
    Transform* t = ctx.world.get<Transform>(selected);
    if (!t) {
        gizmoBusy = false;
        gizmoWasUsing = false;
        return;
    }

    const CameraState& cam = ctx.view.camera;
    Mat4 view = Mat4::lookAt(cam.position, cam.target, cam.up);
    Mat4 proj = cameraProjection(cam);   // respect ortho plan views (P7.3)

    float viewF[16], projF[16], modelF[16];
    toGizmo(view, viewF);
    toGizmo(proj, projF);
    // Manipulate in WORLD space so parented entities drag where they're drawn.
    toGizmo(worldMatrix(ctx.world, selected), modelF);

    ImGuiIO& io = ImGui::GetIO();
    ImGuizmo::SetOrthographic(cam.projection == CameraProjection::Orthographic);
    ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

    ImGuizmo::OPERATION op = (gizmoOp == 1)   ? ImGuizmo::ROTATE
                             : (gizmoOp == 2) ? ImGuizmo::SCALE
                                              : ImGuizmo::TRANSLATE;

    // Shift-drag snaps (Unity-style): translate to a grid step, rotate in
    // whole increments, scale in ticks. Steps overridable via settings.
    float snapValues[3] = {0, 0, 0};
    const float* snap = nullptr;
    if (ctx.input.keyShift) {
        double step = (op == ImGuizmo::ROTATE)
                          ? ctx.settings.getDouble("editor.snapRotateDeg", 15.0)
                      : (op == ImGuizmo::SCALE)
                          ? ctx.settings.getDouble("editor.snapScale", 0.1)
                          : ctx.settings.getDouble("editor.snapTranslate", 0.5);
        snapValues[0] = snapValues[1] = snapValues[2] =
            static_cast<float>(step);
        snap = snapValues;
    }
    ImGuizmo::Manipulate(viewF, projF, op, ImGuizmo::WORLD, modelF, nullptr,
                         snap);
    gizmoBusy = ImGuizmo::IsOver() || ImGuizmo::IsUsing();

    bool usingNow = ImGuizmo::IsUsing();
    if (usingNow) {
        if (!gizmoWasUsing) {
            // Drag begins: snapshot the primary's world and every selected
            // entity's start state, so the group moves rigidly with the
            // primary and one undo entry covers it all.
            dragStartPrimaryWorld = worldMatrix(ctx.world, selected);
            dragEntities.clear();
            dragStartLocals.clear();
            dragStartWorlds.clear();
            for (Entity e : selection) {
                Transform* et = ctx.world.get<Transform>(e);
                if (!et) continue;
                dragEntities.push_back(e);
                dragStartLocals.push_back(*et);
                dragStartWorlds.push_back(worldMatrix(ctx.world, e));
            }
        }
        // The accumulated world-space delta the gizmo applied to the primary.
        Mat4 delta = fromGizmo(modelF) * dragStartPrimaryWorld.inverse();
        for (std::size_t i = 0; i < dragEntities.size(); i++) {
            Entity e = dragEntities[i];
            Mat4 newWorld = delta * dragStartWorlds[i];
            Mat4 local = parentWorldMatrix(ctx.world, e).inverse() * newWorld;
            Transform flat = transformFromMatrix(local);
            if (Transform* et = ctx.world.get<Transform>(e)) *et = flat;
            if (auto* prev = ctx.world.get<PrevTransform>(e)) prev->value = flat;
        }
    } else if (gizmoWasUsing && undo && !dragEntities.empty()) {
        // Drag ended: one compound undo entry across the whole selection.
        std::vector<Transform> after;
        after.reserve(dragEntities.size());
        for (Entity e : dragEntities)
            after.push_back(*ctx.world.get<Transform>(e));
        undo->recordTransformEditMulti(dragEntities, dragStartLocals, after);
    }
    gizmoWasUsing = usingNow;
#endif
}

void EditorSystem::drawSelectionMarker(FrameContext& ctx) const {
    if (selection.empty()) return;

    // Project each selected entity's bounding sphere to the screen and ring
    // it — selection feedback without touching materials. The primary (gizmo
    // anchor) rings brighter so it's distinguishable in a multi-selection.
    const CameraState& cam = ctx.view.camera;
    Mat4 view = Mat4::lookAt(cam.position, cam.target, cam.up);
    Mat4 proj = cameraProjection(cam);   // respect ortho plan views (P7.3)
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const bool ortho = cam.projection == CameraProjection::Orthographic;
    float fovScale =
        vp->Size.y / (2.0f * std::tan(degreesToRadians(cam.fovDegrees) * 0.5f));

    for (Entity e : selection) {
        Renderable* r = ctx.world.get<Renderable>(e);
        if (!r) continue;   // groups have no mesh; nothing to ring (yet)
        BoundingSphere bounds = ctx.renderer.getMeshBounds(r->mesh);
        Mat4 wm = worldMatrix(ctx.world, e);
        Vec3 center = wm.transformPoint(bounds.center);

        Vec3 viewPos = view.transformPoint(center);
        if (viewPos.z > -1e-3) continue;   // behind the camera (-Z is forward)
        Vec3 ndc = proj.transformPoint(viewPos);

        float sx = vp->Pos.x + (static_cast<float>(ndc.x) * 0.5f + 0.5f) * vp->Size.x;
        float sy = vp->Pos.y + (0.5f - static_cast<float>(ndc.y) * 0.5f) * vp->Size.y;

        Vec3 wcx(wm.m[0][0], wm.m[1][0], wm.m[2][0]);
        Vec3 wcy(wm.m[0][1], wm.m[1][1], wm.m[2][1]);
        Vec3 wcz(wm.m[0][2], wm.m[1][2], wm.m[2][2]);
        Real maxScale = std::max({wcx.length(), wcy.length(), wcz.length()});
        float dist = static_cast<float>(-viewPos.z);
        // Screen radius: perspective shrinks with distance; ortho is a fixed
        // world-units-per-pixel scale (orthoHeight = full vertical extent).
        float radius =
            ortho ? static_cast<float>(bounds.radius * maxScale) /
                        std::max(cam.orthoHeight, 1e-3f) * vp->Size.y
                  : static_cast<float>(bounds.radius * maxScale) / dist *
                        fovScale;
        radius = std::clamp(radius * 1.15f, 12.0f, 0.6f * vp->Size.y);

        bool primary = (e == selected);
        ImU32 color = primary ? IM_COL32(255, 170, 40, 220)
                              : IM_COL32(255, 170, 40, 130);
        ImGui::GetBackgroundDrawList()->AddCircle(
            ImVec2(sx, sy), radius, color, 0, primary ? 2.5f : 1.5f);
    }
}

// The selected road's edit overlay (ADR-0050): the centreline skeleton plus a dot per
// node and a hollow ring per tangent handle, the grabbed one lit. Projection mirrors
// drawSelectionMarker; the handle geometry comes from the bound HandleSource.
void EditorSystem::drawPathHandles(FrameContext& ctx) const {
    if (!pathTool.bound()) return;
    const CameraState& cam = ctx.view.camera;
    Mat4 view = Mat4::lookAt(cam.position, cam.target, cam.up);
    Mat4 proj = cameraProjection(cam);   // respect ortho plan views (P7.3)
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    auto toScreen = [&](const Vec3& w, ImVec2& out) -> bool {
        Vec3 vpos = view.transformPoint(w);
        if (vpos.z > -1e-3) return false;          // behind the camera (-Z forward)
        Vec3 ndc = proj.transformPoint(vpos);
        out = ImVec2(vp->Pos.x + (static_cast<float>(ndc.x) * 0.5f + 0.5f) * vp->Size.x,
                     vp->Pos.y + (0.5f - static_cast<float>(ndc.y) * 0.5f) * vp->Size.y);
        return true;
    };
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    ImVec2 a, b;
    for (const std::pair<Vec3, Vec3>& seg : pathTool.previewSegments())
        if (toScreen(seg.first, a) && toScreen(seg.second, b))
            dl->AddLine(a, b, IM_COL32(80, 200, 255, 180), 2.0f);

    int hovered = pathTool.hoveredIndex();
    int selNode = pathTool.selectedNode();
    std::vector<EditHandle> hs = pathTool.handles();
    for (int i = 0; i < static_cast<int>(hs.size()); ++i) {
        ImVec2 s;
        if (!toScreen(hs[i].position, s)) continue;
        bool grabbed = (i == pathTool.grabbedIndex());
        bool hover = (i == hovered);
        bool onSelected = (hs[i].index == selNode);   // a handle of the active node
        bool tangent = hs[i].kind != HandleKind::Knot;
        // Grabbed > hovered > on the selected node > idle, brightest first.
        ImU32 col = grabbed     ? IM_COL32(255, 240, 80, 255)
                    : hover     ? IM_COL32(255, 255, 255, 255)
                    : onSelected ? IM_COL32(255, 210, 90, 255)
                    : tangent   ? IM_COL32(120, 220, 120, 230)
                                : IM_COL32(255, 170, 40, 230);
        float r = (grabbed || hover) ? 7.5f : onSelected ? 6.0f : 5.0f;
        if (tangent) dl->AddCircle(s, r, col, 0, 2.0f);
        else dl->AddCircleFilled(s, r, col);
        // A thin ring around the hovered dot makes the grab target unmistakable.
        if (hover) dl->AddCircle(s, r + 3.0f, IM_COL32(255, 255, 255, 200), 0, 1.5f);
    }

    // Controls cheat-sheet for the selected road, so the gestures are discoverable.
    const char* help =
        "Road edit:  drag dot = move node/tangent\n"
        "Shift+click dot = delete node\n"
        "Ctrl+click road = subdivide   |   Ctrl+click ground = extend from selected node";
    ImVec2 at(vp->Pos.x + 12.0f, vp->Pos.y + vp->Size.y - 64.0f);
    dl->AddText(ImVec2(at.x + 1, at.y + 1), IM_COL32(0, 0, 0, 200), help);   // shadow
    dl->AddText(at, IM_COL32(235, 235, 235, 230), help);
}

#else  // !RT_ENABLE_IMGUI

void EditorSystem::render(FrameContext& ctx) {
    // No UI without ImGui; picking still works (selection marker doesn't draw).
    (void)ctx;
    (void)cameras;
    (void)gizmoOp;
    (void)modelPath;
    (void)gizmoWasUsing;
    (void)gizmoDragStart;
    (void)pendingEdit;
    (void)dragStartPrimaryWorld;
}
void EditorSystem::drawToolbar(FrameContext&) {}
void EditorSystem::drawInspector(FrameContext&) {}
void EditorSystem::drawGizmo(FrameContext&) {}
void EditorSystem::drawGrid(FrameContext&) const {}
void EditorSystem::drawSelectionMarker(FrameContext&) const {}
void EditorSystem::drawPathHandles(FrameContext&) const {}
void EditorSystem::drawGroupMarkers(FrameContext&) const {}
void EditorSystem::drawCameraFrustums(FrameContext&) const {}

#endif

// --- Road sub-object editing (ADR-0049) ------------------------------------
std::vector<Vec3> roadNodeHandles(World& world, Entity e) {
    std::vector<Vec3> handles;
    RoadNet* net = world.get<RoadNet>(e);
    if (!net) return handles;
    handles.reserve(net->nodes.size());
    for (int i = 0; i < static_cast<int>(net->nodes.size()); ++i) {
        const Vec2& n = net->nodes[i];
        // An authored ELEVATED node (nodeElev, an absolute world Y) puts its
        // drag-handle ON the deck, not on the ground beneath the viaduct.
        double y = (i < static_cast<int>(net->nodeElev.size()) && std::isfinite(net->nodeElev[i]))
                       ? net->nodeElev[i] + 0.05
                       : (net->heightAt ? net->heightAt(n.x, n.y) : 0.0) + net->lift + 0.05;
        handles.push_back(Vec3(n.x, y, n.y));
    }
    return handles;
}

bool moveRoadNode(World& world, Entity e, int node, const Vec3& worldPos, Renderer& renderer) {
    RoadNet* net = world.get<RoadNet>(e);
    if (!net || !roadNetMoveNode(*net, node, Vec2(worldPos.x, worldPos.z))) return false;
    regenerateRoad(world, e, renderer);
    return true;
}

std::vector<Vec3> roadTangentHandles(World& world, Entity e) {
    std::vector<Vec3> handles;
    RoadNet* net = world.get<RoadNet>(e);
    if (!net) return handles;
    handles.reserve(net->nodes.size());
    for (int i = 0; i < static_cast<int>(net->nodes.size()); ++i) {
        Vec2 h = net->nodes[i] + roadNetTangentAt(*net, i);   // node + tangent vector
        double y = (i < static_cast<int>(net->nodeElev.size()) && std::isfinite(net->nodeElev[i]))
                       ? net->nodeElev[i] + 0.05
                       : (net->heightAt ? net->heightAt(h.x, h.y) : 0.0) + net->lift + 0.05;
        handles.push_back(Vec3(h.x, y, h.y));
    }
    return handles;
}

bool moveRoadTangent(World& world, Entity e, int node, const Vec3& worldPos, Renderer& renderer) {
    RoadNet* net = world.get<RoadNet>(e);
    if (!net || node < 0 || node >= static_cast<int>(net->nodes.size())) return false;
    Vec2 tangent = Vec2(worldPos.x, worldPos.z) - net->nodes[node];   // handle - node
    if (!roadNetSetTangent(*net, node, tangent)) return false;
    regenerateRoad(world, e, renderer);
    return true;
}

int nearestRoadEdge(World& world, Entity e, const Vec3& worldPos, double maxDist) {
    RoadNet* net = world.get<RoadNet>(e);
    if (!net) return -1;
    return roadNetNearestEdge(*net, Vec2(worldPos.x, worldPos.z), maxDist);
}

bool setRoadEdgeWidth(World& world, Entity e, int edge, double width, Renderer& renderer) {
    RoadNet* net = world.get<RoadNet>(e);
    if (!net || !roadNetSetEdgeWidth(*net, edge, width)) return false;
    regenerateRoad(world, e, renderer);
    return true;
}

int splitRoadEdge(World& world, Entity e, int edge, const Vec3& worldPos, Renderer& renderer) {
    RoadNet* net = world.get<RoadNet>(e);
    if (!net) return -1;
    int ni = roadNetSplitEdge(*net, edge, Vec2(worldPos.x, worldPos.z));
    if (ni >= 0) regenerateRoad(world, e, renderer);
    return ni;
}

int extendRoad(World& world, Entity e, int fromNode, const Vec3& worldPos, Renderer& renderer) {
    RoadNet* net = world.get<RoadNet>(e);
    if (!net) return -1;
    int ni = roadNetExtend(*net, fromNode, Vec2(worldPos.x, worldPos.z));
    if (ni >= 0) regenerateRoad(world, e, renderer);
    return ni;
}

bool deleteRoadNode(World& world, Entity e, int node, Renderer& renderer) {
    RoadNet* net = world.get<RoadNet>(e);
    if (!net || !roadNetDeleteNode(*net, node)) return false;
    regenerateRoad(world, e, renderer);
    return true;
}

}  // namespace engine
