#ifndef RAYTRACER_APPS_CITYSIM_CITY_SPECTATE_H
#define RAYTRACER_APPS_CITYSIM_CITY_SPECTATE_H

#include "../../engine/system.h"
#include "../../engine/camera/follow_camera_controller.h"
#include "../../engine/world.h"
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
// How the chase camera frames what it follows: a car (a car followed as itself, or someone sitting in a household's car,
// ADR-0444: the car is what is seen) or a person on foot.
enum class SpectateFraming : uint8_t { OnFoot, Car };
SpectateFraming spectateFraming(const Agent& agent);

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
// ...and over the agents then `cars` cars after them (indices agents.size() .. + cars - 1, every one spectatable): the
// cycle K and [ ] go round when the city has cars with nobody in them to follow (ADR-0453).
int nextSpectateTarget(const std::vector<Agent>& agents, std::size_t cars, int current, int dir);

// A car followed as itself: one with nobody in it that drives round for ever (city_car.h CityCarRoams), while it is
// somewhere (on the road, or a near physics car) and not the player's. A car with a driver is followed through them.
bool isSpectatableCar(engine::World& world, engine::Entity car);

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
    engine::Entity followedCar() const { return car_; }   // a car followed as itself (isSpectatableCar), or none
    // Follow this agent from the next frame, as if K and [ ] had picked it (the trip lab: a person sent somewhere is
    // followed there) but leaving the debug widgets as they are, and stay on it while it has no body to show
    // (indoors, between car and pavement) rather than moving on to someone else. K, [ or ] let go of it as before.
    void follow(int agentIndex) { pendingFollow_ = agentIndex; }
    // Stop following from the next frame, as K does: the view goes back to the camera and player systems (the control
    // channel's `camera` verb can then place it). Harmless when nothing is followed.
    void letGo() { pendingLetGo_ = true; }

private:
    // Frame the chase camera for what is followed: a longer arm for a car, a tighter frame for a walker. Called when
    // the target changes.
    void configureFollowFor(SpectateFraming framing);
    // Stop spectating and hand the view back (restores the prior widget state).
    void stopSpectating(engine::FrameContext& ctx);
    // The cars followed as themselves now, in the order of their numbers.
    std::vector<engine::Entity> spectatableCars(engine::World& world) const;
    // Move to the next (`dir` +1) or previous (-1) target of the cycle (nextSpectateTarget), framed for it; false
    // when there is none.
    bool advance(engine::World& world, int dir);

    CityRenderSystem& render_;
    engine::FollowCameraController follow_;
    bool spectating_ = false;
    int current_ = -1;             // index into sim().agents(), or -1
    engine::Entity car_;           // ...or the car followed as itself (current_ is then -1)
    bool widgetsWereOn_ = false;   // debug-widget state captured on spectate-on
    // What the chase is framed for: a followed walker who gets into a car (or out of one) is re-framed.
    SpectateFraming framedFor_ = SpectateFraming::OnFoot;
    int pendingFollow_ = -1;       // follow()'s agent, taken up by the next update
    bool held_ = false;            // following follow()'s agent: kept through frames it has no body
    bool pendingLetGo_ = false;    // letGo(), taken up by the next update
};

}  // namespace citysim

#endif
