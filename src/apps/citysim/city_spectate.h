#ifndef RAYTRACER_APPS_CITYSIM_CITY_SPECTATE_H
#define RAYTRACER_APPS_CITYSIM_CITY_SPECTATE_H

#include "../../engine/system.h"
#include "../../engine/camera/follow_camera_controller.h"
#include "city_sim.h"   // Agent

#include <vector>

namespace citysim {

class CityRenderSystem;

// Spectate mode (ADR-0064 follow-up): cycle the view through the living city's
// agents and chase one, so you watch an NPC drive its commute or walk its route
// with its debug ring + vision cone lit. It only READS agent world poses (from
// CityRenderSystem) and drives ctx.view.camera with a FollowCameraController; it
// never owns motion, so it needs no physics and lives outside the Jolt block.
//
// Controls (bound in onStart): K toggles spectate on/off, [ and ] cycle the
// followed agent (prev / next). While spectating it forces the debug widgets on
// so the followed agent shows its ring/cone, and restores the prior widget state
// when it turns off. When off it writes nothing — PlayerSystem/CameraSystem own
// the view exactly as before.

// --- pure, headless-testable core --------------------------------------------
// An agent worth spectating: alive in the sim (not released to the player) and
// not the host-driven player ghost. Released/player agents are skipped.
bool isSpectatable(const Agent& agent);

// The next spectatable agent from `current` scanning in `dir` (+1 next / -1
// prev), wrapping around the vector and skipping non-spectatable agents.
// `current` may be -1 / out of range (nothing selected yet): then the scan
// begins at the first (dir>0) or last (dir<0) index. Returns -1 when no agent
// is spectatable at all. If `current` is the only spectatable agent it is
// returned (cycling is a no-op rather than losing the target).
int nextSpectatable(const std::vector<Agent>& agents, int current, int dir);

// Clamp a possibly-stale selection to a valid index for a vector of `count`
// agents: -1 (no selection) when the city is empty, the last index when the
// selection ran off the end (the count shrank), else the index unchanged. The
// caller still re-checks isSpectatable() on the clamped index.
int clampSpectateIndex(int index, int count);

// Convert an agent XZ heading (Vec2, .x = world +x, .y = world +z) to the
// FlyCamera yaw convention FollowCameraController expects (yaw 0 looks down -Z):
// the agent's forward is (heading.x, 0, heading.y), so yaw = atan2(fx, -fz).
engine::Real spectateHeadingDegrees(engine::Vec2 heading);

// -----------------------------------------------------------------------------

class CitySpectateSystem : public engine::System {
public:
    explicit CitySpectateSystem(CityRenderSystem& render) : render_(render) {}

    void onStart(engine::FrameContext& ctx) override;
    void update(engine::FrameContext& ctx) override;

    // Inspection (tests / debug): the current spectate state.
    bool spectating() const { return spectating_; }
    int followedAgent() const { return current_; }

private:
    // Frame the chase camera for the followed agent's mode: a longer arm for a
    // driver, a tighter frame for a walker. Called when the target changes.
    void configureFollowFor(Agent::Mode mode);
    // Stop spectating and hand the view back (restores the prior widget state).
    void stopSpectating(engine::FrameContext& ctx);

    CityRenderSystem& render_;
    engine::FollowCameraController follow_;
    bool spectating_ = false;
    int current_ = -1;             // index into sim().agents(), or -1
    bool widgetsWereOn_ = false;   // debug-widget state captured on spectate-on
};

}  // namespace citysim

#endif
