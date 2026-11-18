#include "city_spectate.h"

#include "city_render.h"
#include "../../engine/camera/view_distance.h"
#include "../../renderer/event.h"   // KeyCode

#include <algorithm>
#include <cmath>
#include <utility>

namespace citysim {

using engine::Real;
using engine::Vec2;
using engine::Vec3;

namespace {
// How the camera frames an agent: as a car when they drive one, or sit in a household's car (ADR-0444: they are a
// walker in it, and the car is what is seen); as a walker otherwise.
Agent::Mode framing(const Agent& a) { return a.inCar >= 0 ? Agent::Mode::Driver : a.mode; }
}  // namespace

bool isSpectatable(const Agent& agent) {
    // A released agent's ghost is parked (its physical car is player-driven), and
    // the player ghost isn't an NPC to watch — skip both when cycling.
    return !agent.released && !agent.playerControlled;
}

int nextSpectatable(const std::vector<Agent>& agents, int current, int dir) {
    return nextSpectateTarget(agents, 0, current, dir);
}

int nextSpectateTarget(const std::vector<Agent>& agents, std::size_t cars, int current, int dir) {
    const int agentCount = static_cast<int>(agents.size());
    const int n = agentCount + static_cast<int>(cars);
    if (n == 0) return -1;
    const int step = dir < 0 ? -1 : 1;
    // Normalize the starting point so the first index examined is `current`'s
    // neighbour, or index 0 / n-1 when nothing valid is selected yet.
    const int base = (current >= 0 && current < n) ? current
                                                   : (step > 0 ? -1 : n);
    for (int i = 1; i <= n; ++i) {
        const int idx = ((base + step * i) % n + n) % n;
        if (idx >= agentCount || isSpectatable(agents[idx])) return idx;
    }
    return -1;
}

bool isSpectatableCar(engine::World& world, engine::Entity car) {
    const CityCarLink* link = world.get<CityCarLink>(car);
    return link && world.has<CityCarBody>(car) && world.has<CityCarRoams>(car) && link->driver < 0 && !link->away &&
           !carNowhere(world, car);
}

std::vector<engine::Entity> CitySpectateSystem::spectatableCars(engine::World& world) const {
    std::vector<std::pair<uint32_t, engine::Entity>> byNumber;
    world.each<CityCarBody>([&](engine::Entity e, CityCarBody& body) {
        if (isSpectatableCar(world, e)) byNumber.push_back({body.id, e});
    });
    std::sort(byNumber.begin(), byNumber.end(),
              [](const std::pair<uint32_t, engine::Entity>& a, const std::pair<uint32_t, engine::Entity>& b) {
                  return a.first < b.first;
              });
    std::vector<engine::Entity> out;
    for (const std::pair<uint32_t, engine::Entity>& c : byNumber) out.push_back(c.second);
    return out;
}

bool CitySpectateSystem::advance(engine::World& world, int dir) {
    const std::vector<Agent>& agents = render_.sim().agents();
    const std::vector<engine::Entity> cars = spectatableCars(world);
    // where the followed target stands in the cycle: an agent by its index, a car after every agent
    int at = current_;
    if (car_.valid()) {
        at = -1;
        for (std::size_t k = 0; k < cars.size(); ++k)
            if (cars[k] == car_) at = static_cast<int>(agents.size() + k);
    }
    const int sel = nextSpectateTarget(agents, cars.size(), at, dir);
    if (sel < 0) return false;
    if (sel < static_cast<int>(agents.size())) {
        current_ = sel;
        car_ = engine::Entity{};
        configureFollowFor(framing(agents[static_cast<std::size_t>(sel)]));
    } else {
        current_ = -1;
        car_ = cars[static_cast<std::size_t>(sel) - agents.size()];
        configureFollowFor(Agent::Mode::Driver);
    }
    return true;
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
    framedFor_ = mode;
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
            if (advance(ctx.world, +1)) {
                spectating_ = true;
                widgetsWereOn_ = render_.debugWidgets();
                render_.setDebugWidgets(true);
            }
        } else {
            stopSpectating(ctx);
        }
        held_ = false;
    }
    // follow(): this agent, from now. Unlike K it leaves the debug widgets as they are: the caller draws its own
    // (the trip's plan line), and the nav graph's lines all round would bury it -- J still shows them.
    if (pendingFollow_ >= 0 && pendingFollow_ < count) {
        if (!spectating_) {
            spectating_ = true;
            widgetsWereOn_ = render_.debugWidgets();
        }
        current_ = pendingFollow_;
        car_ = engine::Entity{};
        configureFollowFor(framing(agents[static_cast<std::size_t>(current_)]));
        held_ = true;
    }
    pendingFollow_ = -1;
    // letGo(): as K, when something is followed.
    if (pendingLetGo_) {
        if (spectating_) stopSpectating(ctx);
        held_ = false;
        pendingLetGo_ = false;
    }

    if (!spectating_) return;   // OFF: PlayerSystem / CameraSystem own the view.

    // [ / ] cycle prev / next (skips released agents, wraps).
    if (ctx.actions.pressed("spectate_next")) {
        advance(ctx.world, +1);
        held_ = false;
    }
    if (ctx.actions.pressed("spectate_prev")) {
        advance(ctx.world, -1);
        held_ = false;
    }

    // Stale-selection guard: the agent vector may have changed, the followed
    // agent may have been released, or the followed car taken off the road or by
    // the player. Clamp, then move to the next spectatable one; if none remain,
    // drop spectate and hand the view back.
    if (car_.valid()) {
        if (!isSpectatableCar(ctx.world, car_) && !advance(ctx.world, +1)) { stopSpectating(ctx); return; }
    } else {
        current_ = clampSpectateIndex(current_, count);
        if ((current_ < 0 || !isSpectatable(agents[current_])) && !advance(ctx.world, +1)) {
            stopSpectating(ctx);
            return;
        }
    }

    // Resolve the followed target's WORLD pose. A car with nobody in it is where its entity is (city_car.h). An agent:
    // the real external body when one is reported (cars/peds owned by the physics bridges), else the sim ghost. A
    // frame with no reported body (e.g. just released to the player) leaves the camera where it was and advances
    // next frame.
    Vec3 worldPos;
    Vec2 heading;
    if (car_.valid()) {
        const engine::Transform* t = ctx.world.get<engine::Transform>(car_);
        if (!t) return;
        worldPos = t->position;
        heading = carHeading(*t);
    } else if (!render_.agentWorldPose(current_, worldPos, heading)) {
        if (held_) return;   // the agent we were asked to follow: wait for its body
        advance(ctx.world, +1);
        return;
    } else if (framing(agents[static_cast<std::size_t>(current_)]) != framedFor_) {
        // In or out of a car: the chase framed for what it is now.
        configureFollowFor(framing(agents[static_cast<std::size_t>(current_)]));
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
