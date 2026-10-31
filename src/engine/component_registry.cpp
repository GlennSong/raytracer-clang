#include "component_registry.h"

#include "components.h"
#include "camera/scene_camera.h"

namespace engine {

void registerEngineComponents(ComponentRegistry& registry) {
    registry.add<Transform>("Transform");
    registry.add<SourceSpec>("Shape");
    // Renderable's editable surface is its material; mesh handles are runtime.
    registry.addWithAccessor<Renderable>(
        "Material", [](Renderable& r, PropertyVisitor& v) {
            describeProperties(r.material, v);
        });
    // Add/remove is offered only where attach/detach is safe without extra
    // services: a camera or spawn marker on any entity. Transform and Shape
    // are the document's identity; Material rides on Renderable (needs a
    // mesh) — none of those make sense to toggle from a menu.
    registry.allowAddRemove<SceneCamera>(registry.add<SceneCamera>("Camera"));
    registry.allowAddRemove<PlayerSpawn>(
        registry.addMarker<PlayerSpawn>("Player Spawn"));

    // Runtime-state rows, for WATCHING a playtest through the observer
    // bridge: never present on document entities, never authored here
    // (physics is authored via SourceSpec's block; locked fields are
    // display-only and the JSON read visitor skips them by contract).
    registry.add<Velocity>("Velocity");
    registry.addWithAccessor<RigidBody>(
        "Rigid Body", [](RigidBody& rb, PropertyVisitor& v) {
            // Transient lvalues are fine: visitors only hold references for
            // the duration of the visit.
            std::string motion = rb.motion == BodyMotion::Dynamic ? "dynamic"
                                 : rb.motion == BodyMotion::Kinematic
                                     ? "kinematic"
                                     : "static";
            v.field(FieldMeta("Motion").id("motion").locked(), motion);
            bool lockRotation = rb.lockRotation;
            v.field(FieldMeta("Lock Rotation").id("lockRotation").locked(),
                    lockRotation);
        });
    registry.addWithAccessor<ControlledBy>(
        "Controlled By", [](ControlledBy& c, PropertyVisitor& v) {
            Real player = c.playerIndex;
            v.field(FieldMeta("Player").id("player").locked(), player);
        });
    // Deliberately absent: PrevTransform (interpolation plumbing), Collider
    // (runtime physics state; authored via SourceSpec's block).
}

}  // namespace engine
