#include "city_spectate.h"

#include "city_render.h"
#include "../../engine/camera/view_distance.h"
#include "../../renderer/event.h"   // KeyCode

#include <cmath>

namespace citysim {

using engine::Real;
using engine::Vec2;
using engine::Vec3;

bool isSpectatable(const Agent& agent) {
    // A released agent's ghost is parked (its physical car is player-driven), and
    // the player ghost isn't an NPC to watch — skip both when cycling.
    return !agent.released && !agent.playerControlled;
}

int nextSpectatable(const std::vector<Agent>& agents, int current, int dir) {
    const int n = static_cast<int>(agents.size());
    if (n == 0) return -1;
    const int step = dir < 0 ? -1 : 1;
    // Normalize the starting point so the first index examined is `current`'s
    // neighbour, or index 0 / n-1 when nothing valid is selected yet.
    const int base = (current >= 0 && current < n) ? current
                                                   : (step > 0 ? -1 : n);
    for (int i = 1; i <= n; ++i) {
        const int idx = ((base + step * i) % n + n) % n;
        if (isSpectatable(agents[idx])) return idx;
    }
    return -1;
}

int clampSpectateIndex(int index, int count) {
    if (count <= 0) return -1;
    if (index < 0) return -1;
    if (index >= count) return count - 1;
    return index;
}

Real spectateHeadingDegrees(Vec2 heading) {
    // forward = (heading.x, 0, heading.y); FlyCamera yaw = atan2(fwd.x, -fwd.z).
    return engine::radiansToDegrees(std::atan2(heading.x, -heading.y));
}

void CitySpectateSystem::onStart(engine::FrameContext& ctx) {
    ctx.actions.bindButton("spectate_toggle", engine::KeyCode::K);
    ctx.actions.bindButton("spectate_prev", engine::KeyCode::LeftBracket);
    ctx.actions.bindButton("spectate_next", engine::KeyCode::RightBracket);
}

void CitySpectateSystem::configureFollowFor(Agent::Mode mode) {
    // Behind-and-above framing; look input never reaches it (zoom-only), so the
    // agent's heading owns the yaw and the offsets stay glued to its back.
    follow_.orbitYaw = 0.0;
    follow_.lateralOffset = 0.0;
    if (mode == Agent::Mode::Driver) {
        follow_.distance = 8.0;
        follow_.minDistance = 3.0;
        follow_.maxDistance = 22.0;
        follow_.targetHeight = 1.5;
        follow_.orbitPitch = -14.0;
    } else {
        follow_.distance = 5.0;
        follow_.minDistance = 2.0;
        follow_.maxDistance = 12.0;
        follow_.targetHeight = 1.2;
        follow_.orbitPitch = -12.0;
    }
}

void CitySpectateSystem::stopSpectating(engine::FrameContext& /*ctx*/) {
    spectating_ = false;
    render_.setDebugWidgets(widgetsWereOn_);   // hand the HUD back as we found it
    // Keep current_ so a re-toggle resumes near the last agent watched.
}

void CitySpectateSystem::update(engine::FrameContext& ctx) {
    const std::vector<Agent>& agents = render_.sim().agents();
    const int count = static_cast<int>(agents.size());
    follow_.farPlane = engine::worldViewFar(ctx.world);

    // K toggles spectate. Turning ON forces the debug widgets on (so the followed
    // agent shows its ring/cone) after stashing the prior state; a city with no
    // spectatable agent stays off (a no-op).
    if (ctx.actions.pressed("spectate_toggle")) {
        if (!spectating_) {
            const int sel = nextSpectatable(agents, current_, +1);
            if (sel >= 0) {
                spectating_ = true;
                current_ = sel;
                widgetsWereOn_ = render_.debugWidgets();
                render_.setDebugWidgets(true);
                configureFollowFor(agents[current_].mode);
            }
        } else {
            stopSpectating(ctx);
        }
    }

    if (!spectating_) return;   // OFF: PlayerSystem / CameraSystem own the view.

    // [ / ] cycle prev / next (skips released agents, wraps).
    if (ctx.actions.pressed("spectate_next")) {
        const int sel = nextSpectatable(agents, current_, +1);
        if (sel >= 0) { current_ = sel; configureFollowFor(agents[current_].mode); }
    }
    if (ctx.actions.pressed("spectate_prev")) {
        const int sel = nextSpectatable(agents, current_, -1);
        if (sel >= 0) { current_ = sel; configureFollowFor(agents[current_].mode); }
    }

    // Stale-selection guard: the agent vector may have changed, or the followed
    // agent may have been released. Clamp, then move to the next spectatable one;
    // if none remain, drop spectate and hand the view back.
    current_ = clampSpectateIndex(current_, count);
    if (current_ < 0 || !isSpectatable(agents[current_])) {
        const int sel = nextSpectatable(agents, current_, +1);
        if (sel < 0) { stopSpectating(ctx); return; }
        current_ = sel;
        configureFollowFor(agents[current_].mode);
    }

    // Resolve the followed agent's WORLD pose — the real external body when one is
    // reported (cars/peds owned by the physics bridges), else the sim ghost. A
    // frame with no reported body (e.g. just released to the player) leaves the
    // camera where it was and advances next frame.
    Vec3 worldPos;
    Vec2 heading;
    if (!render_.agentWorldPose(current_, worldPos, heading)) {
        const int sel = nextSpectatable(agents, current_, +1);
        if (sel >= 0) { current_ = sel; configureFollowFor(agents[current_].mode); }
        return;
    }

    // Chase the pose. Feed ZOOM ONLY (mirrors PlayerSystem's third-person branch):
    // the heading owns the yaw, so a full update() would double-apply look input.
    follow_.setTarget(worldPos, spectateHeadingDegrees(heading));
    engine::CameraInput zoom;
    zoom.zoomDelta = ctx.input.scrollDelta;
    follow_.update(zoom, ctx.frameDelta);

    const float aspect = (ctx.framebufferHeight > 0)
        ? static_cast<float>(ctx.framebufferWidth) / ctx.framebufferHeight
        : 1.0f;
    ctx.view.camera = follow_.cameraState(aspect);
    ctx.view.activeCameraEntity = engine::Entity{};   // a controller view, not a placed cam
}

}  // namespace citysim
