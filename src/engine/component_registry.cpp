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
    registry.add<SceneCamera>("Camera");
    registry.addMarker<PlayerSpawn>("Player Spawn");
    // Deliberately absent: PrevTransform (interpolation plumbing), RigidBody/
    // Collider (runtime physics state; authored via SourceSpec's block).
}

}  // namespace engine
