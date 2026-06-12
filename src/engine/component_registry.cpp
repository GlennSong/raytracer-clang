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
    // Deliberately absent: PrevTransform (interpolation plumbing), RigidBody/
    // Collider (runtime physics state; authored via SourceSpec's block).
}

}  // namespace engine
