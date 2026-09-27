#include "flashlight_system.h"

#include "../components.h"
#include "../world.h"
#include "../../log.h"

#include <cmath>

namespace engine {

void FlashlightSystem::onStart(FrameContext& ctx) {
    ctx.actions.bindButton("slot_4", KeyCode::Num4);   // the flashlight (a tool)
}

void FlashlightSystem::update(FrameContext& ctx) {
    if (ctx.actions.pressed("slot_4")) {
        on_ = !on_;
        LOG_INFO << "[flashlight] " << (on_ ? "on" : "off");
    }
}

// the beam is staged in RENDER, after the fixed steps, into its own list (SceneLighting::toolSpots), rebuilt
// every frame here; RenderSystem, registered after this, merges it
void FlashlightSystem::render(FrameContext& ctx) {
    ctx.view.lighting.toolSpots.clear();
    if (!on_) return;
    // on foot only: at the wheel the car's own headlights are the light
    bool driving = false;
    ctx.world.each<ControlledBy, InVehicle>([&](Entity, ControlledBy&, InVehicle& iv) { if (iv.vehicle.valid()) driving = true; });
    if (driving) return;
    const CameraState& c = ctx.view.camera;
    Vec3 fwd = c.target - c.position;
    const Real len = fwd.length();
    if (len < 1e-6) return;
    fwd = fwd * (1.0 / len);
    const Vec3 right = normalize(cross(fwd, Vec3(0, 1, 0)));
    // held in the right hand: a little right of and below the eye, so the beam's edge reads in the view
    const Vec3 at = c.position + right * 0.25 - Vec3(0, 0.3, 0) + fwd * 0.3;
    SpotLight beam(at, fwd, Vec3(1.0, 0.93, 0.8), /*intensity=*/140.0f, /*inner=*/0.2f, /*outer=*/0.42f);
    beam.range = 60.0f;
    ctx.view.lighting.toolSpots.push_back(beam);
}

}  // namespace engine
