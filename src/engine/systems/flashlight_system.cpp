#include "flashlight_system.h"

#include "../components.h"
#include "../player/tools.h"
#include "../player_pose.h"
#include "../world.h"
#include "../../log.h"

#include <cmath>

namespace engine {

void FlashlightSystem::onStart(FrameContext& ctx) {
    ctx.actions.bindButton("light", KeyCode::L);   // (in a car L is its headlights: vehicle_lights)
    ctx.actions.bindButton("light", GamepadButton::DpadUp);
    ctx.actions.setActionContext("light", InputContext::OnFoot);
}

void FlashlightSystem::update(FrameContext& ctx) {
    if (ctx.actions.pressed("light")) {
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
    Vec3 body;
    bool haveBody = false;
    Tool tool = Tool::Hands;
    ctx.world.each<ControlledBy, Transform>([&](Entity e, ControlledBy&, Transform& t) {
        if (haveBody) return;
        haveBody = true;
        body = t.position;
        if (const InVehicle* iv = ctx.world.get<InVehicle>(e)) driving = iv->vehicle.valid();
        if (const PlayerPose* p = ctx.world.get<PlayerPose>(e)) tool = static_cast<Tool>(p->tool);
    });
    if (driving) return;
    const CameraState& c = ctx.view.camera;
    Vec3 fwd = c.target - c.position;
    const Real len = fwd.length();
    if (len < 1e-6) return;
    fwd = fwd * (1.0 / len);
    const Vec3 right = normalize(cross(fwd, Vec3(0, 1, 0)));
    // WHERE IT RIDES, from the eye: first person, the camera's; third person, the body's (the camera is behind it)
    const bool third = ctx.settings.getBool("playerThirdPerson", false) && haveBody;
    const Vec3 eye = third ? body + Vec3(0, 0.7, 0) : c.position;
    Vec3 at;
    if (tool == Tool::Hands) at = eye + right * 0.25 - Vec3(0, 0.3, 0) + fwd * 0.3;        // in the right hand
    else if (tool == Tool::Gun) at = eye + right * 0.17 - Vec3(0, 0.24, 0) + fwd * 0.6;    // under the barrel
    else at = eye + Vec3(0, 0.09, 0) + fwd * 0.1;                                         // a headlamp
    SpotLight beam(at, fwd, Vec3(1.0, 0.93, 0.8), /*intensity=*/140.0f, /*inner=*/0.2f, /*outer=*/0.42f);
    beam.range = 60.0f;
    ctx.view.lighting.toolSpots.push_back(beam);
}

}  // namespace engine
