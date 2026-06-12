#include "test_framework.h"

#include "../src/engine/undo_stack.h"
#include "../src/engine/component_registry.h"
#include "../src/engine/components.h"
#include "../src/engine/camera/scene_camera.h"
#include <nlohmann/json.hpp>

using namespace engine;  // namespace migration (ADR-0015)
using json = nlohmann::json;

// The editor command log: every edit chokepoint records a reversible command;
// undo/redo re-apply them through the property layer.

namespace {

struct Fixture {
    World world;
    ComponentRegistry registry;
    UndoStack undo{registry};

    Fixture() { registerEngineComponents(registry); }

    Entity makeBox() {
        Entity e = world.create();
        world.add<Transform>(e);
        world.add<PrevTransform>(e);
        SourceSpec spec;
        spec.shape = "box";
        world.add<SourceSpec>(e, spec);
        world.add<Renderable>(e);
        return e;
    }

    json materialState(Entity e) {
        return undo.componentState(world, e, "Material");
    }
};

}  // namespace

TEST_CASE(undo_field_edit_round_trips_through_property_layer) {
    Fixture f;
    Entity e = f.makeBox();
    RenderMaterial& mat = f.world.get<Renderable>(e)->material;

    json before = f.materialState(e);
    mat.metallic = 0.9f;
    f.undo.recordFieldEdit(e, "Material", "Metallic", before, f.materialState(e));
    CHECK(f.undo.canUndo());

    Entity affected = f.undo.undo(f.world);
    CHECK(affected == e);
    CHECK_APPROX(mat.metallic, 0.0, 1e-6);   // back to default

    f.undo.redo(f.world);
    CHECK_APPROX(mat.metallic, 0.9, 1e-6);
}

TEST_CASE(undo_coalesces_consecutive_edits_to_one_field) {
    Fixture f;
    Entity e = f.makeBox();
    RenderMaterial& mat = f.world.get<Renderable>(e)->material;

    json start = f.materialState(e);
    // Three spinner ticks on the same field -> one command.
    for (float v : {0.1f, 0.2f, 0.3f}) {
        json before = f.materialState(e);
        mat.metallic = v;
        f.undo.recordFieldEdit(e, "Material", "Metallic", before,
                               f.materialState(e));
    }
    CHECK(f.undo.depth() == 1);

    f.undo.undo(f.world);
    CHECK_APPROX(mat.metallic, 0.0, 1e-6);   // the FIRST before wins
    f.undo.redo(f.world);
    CHECK_APPROX(mat.metallic, 0.3, 1e-6);   // the LAST after wins

    // Edits that return to the starting value cancel the whole entry.
    f.undo.clear();
    json b1 = f.materialState(e);
    mat.metallic = 0.7f;
    f.undo.recordFieldEdit(e, "Material", "Metallic", b1, f.materialState(e));
    json b2 = f.materialState(e);
    mat.metallic = 0.3f;
    f.undo.recordFieldEdit(e, "Material", "Metallic", b2, f.materialState(e));
    CHECK(f.undo.depth() == 0);

    // Identical before/after never records at all.
    f.undo.recordFieldEdit(e, "Material", "Metallic", f.materialState(e),
                           f.materialState(e));
    CHECK(!f.undo.canUndo());
}

TEST_CASE(undo_transform_edit_restores_pose_and_prev) {
    Fixture f;
    Entity e = f.makeBox();
    Transform& t = *f.world.get<Transform>(e);

    Transform before = t;
    t.position = Vec3(5, 6, 7);
    t.orientation = Quat::fromAxisAngle(Vec3(0, 1, 0), 1.0);
    f.undo.recordTransformEdit(e, before, t);

    f.undo.undo(f.world);
    CHECK_APPROX(t.position.x, 0.0, 1e-9);
    CHECK_APPROX(f.world.get<PrevTransform>(e)->value.position.x, 0.0, 1e-9);

    f.undo.redo(f.world);
    CHECK_APPROX(t.position.z, 7.0, 1e-9);
    // Orientation round-trips even though it is not a described property.
    Vec3 fwd = t.orientation.rotate(Vec3(0, 0, -1));
    CHECK(std::abs(fwd.x) > 0.1);
}

TEST_CASE(undo_create_and_delete_round_trip_with_remap) {
    Fixture f;
    Entity e = f.makeBox();
    f.world.get<Transform>(e)->position = Vec3(1, 2, 3);
    f.world.get<Renderable>(e)->material.albedo = Vec3(0.9, 0.1, 0.1);
    f.undo.recordCreate(e);

    // Undo the create: the entity is gone.
    f.undo.undo(f.world);
    CHECK(!f.world.alive(e));

    // Redo brings it back — under a fresh handle, components intact.
    Entity back = f.undo.redo(f.world);
    CHECK(back.valid());
    CHECK(f.world.alive(back));
    CHECK_APPROX(f.world.get<Transform>(back)->position.y, 2.0, 1e-9);
    CHECK_APPROX(f.world.get<Renderable>(back)->material.albedo.x, 0.9, 1e-6);
    CHECK(f.world.has<SourceSpec>(back));
    CHECK(f.world.has<PrevTransform>(back));

    // Now delete it; undo restores, and LATER commands were remapped to the
    // restored handle, so the whole log stays applicable.
    json before = f.materialState(back);
    f.world.get<Renderable>(back)->material.metallic = 0.8f;
    f.undo.recordFieldEdit(back, "Material", "Metallic", before,
                           f.materialState(back));

    f.undo.recordDelete(f.world, back);
    f.world.destroy(back);
    CHECK(!f.world.alive(back));

    Entity restored = f.undo.undo(f.world);          // un-delete
    CHECK(f.world.alive(restored));
    CHECK_APPROX(f.world.get<Renderable>(restored)->material.metallic, 0.8,
                 1e-6);
    f.undo.undo(f.world);                            // un-edit (remapped)
    CHECK_APPROX(f.world.get<Renderable>(restored)->material.metallic, 0.0,
                 1e-6);
}

TEST_CASE(undo_delete_restores_camera_and_spawn_markers) {
    Fixture f;
    Entity e = f.world.create();
    Transform t;
    t.position = Vec3(0, 4, 0);
    f.world.add<Transform>(e, t);
    SceneCamera cam;
    cam.name = "Crane";
    cam.lens.focalLength = 85.0;
    f.world.add<SceneCamera>(e, cam);
    f.world.add<PlayerSpawn>(e);

    f.undo.recordDelete(f.world, e);
    f.world.destroy(e);

    Entity restored = f.undo.undo(f.world);
    CHECK(f.world.alive(restored));
    CHECK(f.world.get<SceneCamera>(restored)->name == "Crane");
    CHECK_APPROX(f.world.get<SceneCamera>(restored)->lens.focalLength, 85.0,
                 1e-9);
    CHECK(f.world.has<PlayerSpawn>(restored));
}

TEST_CASE(undo_component_add_remove_round_trips) {
    Fixture f;
    Entity e = f.makeBox();
    ComponentRegistry::Entry* camera = f.registry.find("Camera");
    CHECK(camera != nullptr);

    // Menu add, then undo/redo it.
    camera->addTo(f.world, e);
    f.world.get<SceneCamera>(e)->name = "Added";
    f.undo.recordComponentAdd(f.world, e, "Camera");

    f.undo.undo(f.world);
    CHECK(!f.world.has<SceneCamera>(e));
    f.undo.redo(f.world);
    CHECK(f.world.has<SceneCamera>(e));
    CHECK(f.world.get<SceneCamera>(e)->name == "Added");   // state restored

    // Menu remove: undo restores the component with its edited fields.
    f.world.get<SceneCamera>(e)->lens.fStop = 1.4;
    f.undo.recordComponentRemove(f.world, e, "Camera");
    camera->removeFrom(f.world, e);
    CHECK(!f.world.has<SceneCamera>(e));

    f.undo.undo(f.world);
    CHECK(f.world.has<SceneCamera>(e));
    CHECK_APPROX(f.world.get<SceneCamera>(e)->lens.fStop, 1.4, 1e-9);
}

TEST_CASE(undo_field_edit_fires_post_edit_hook_on_restore) {
    Fixture f;
    Entity e = f.makeBox();

    int hookCalls = 0;
    const char* hookLabel = "sentinel";
    f.registry.find("Shape")->onEdited = [&](World&, Entity,
                                             const char* label) {
        hookCalls++;
        hookLabel = label;
    };

    SourceSpec& spec = *f.world.get<SourceSpec>(e);
    json before = f.undo.componentState(f.world, e, "Shape");
    spec.size = Vec3(3, 3, 3);
    f.undo.recordFieldEdit(e, "Shape", "Size", before,
                           f.undo.componentState(f.world, e, "Shape"));

    f.undo.undo(f.world);
    CHECK_APPROX(spec.size.x, 1.0, 1e-9);
    // Bulk restores dispatch the hook with a null label.
    CHECK(hookCalls == 1);
    CHECK(hookLabel == nullptr);
}

TEST_CASE(undo_revision_tracks_every_change) {
    Fixture f;
    Entity e = f.makeBox();
    RenderMaterial& mat = f.world.get<Renderable>(e)->material;
    CHECK(f.undo.revision() == 0);

    json before = f.materialState(e);
    mat.metallic = 0.5f;
    f.undo.recordFieldEdit(e, "Material", "Metallic", before,
                           f.materialState(e));
    uint64_t afterEdit = f.undo.revision();
    CHECK(afterEdit > 0);

    // Undo and redo both move the document away from whatever was saved, so
    // they bump the revision too (the dirty flag is conservative).
    f.undo.undo(f.world);
    CHECK(f.undo.revision() > afterEdit);
    uint64_t afterUndo = f.undo.revision();
    f.undo.redo(f.world);
    CHECK(f.undo.revision() > afterUndo);

    // Coalesced edits also count as changes.
    uint64_t beforeCoalesce = f.undo.revision();
    json b2 = f.materialState(e);
    mat.metallic = 0.6f;
    f.undo.recordFieldEdit(e, "Material", "Metallic", b2, f.materialState(e));
    CHECK(f.undo.revision() > beforeCoalesce);
}

TEST_CASE(undo_survives_stale_entities_and_empty_stacks) {
    Fixture f;
    CHECK(!f.undo.canUndo());
    CHECK(!f.undo.canRedo());
    CHECK(!f.undo.undo(f.world).valid());
    CHECK(!f.undo.redo(f.world).valid());

    // A command whose entity died outside the log degrades to a no-op.
    Entity e = f.makeBox();
    json before = f.materialState(e);
    f.world.get<Renderable>(e)->material.metallic = 0.5f;
    f.undo.recordFieldEdit(e, "Material", "Metallic", before,
                           f.materialState(e));
    f.world.destroy(e);
    f.undo.undo(f.world);   // must not crash; nothing to apply
    CHECK(!f.world.alive(e));
}
