#ifndef RAYTRACER_APPS_CITYSIM_CITY_PLAYER_TRANSIT_H
#define RAYTRACER_APPS_CITYSIM_CITY_PLAYER_TRANSIT_H

// THE PLAYER RIDES (Glenn, 2026-09-18: "We should let the player ride a bus or
// taxi", and earlier "they could hitch a ride that way").
//
// NPCs already ride: RideBook carries a passenger on its driver's pose. The
// player cannot, because the player is not a sim agent -- it is a character
// capsule the host drives, and `possess` only ever made it the DRIVER of a car
// or the body of a walker. Nobody had ever been a passenger.
//
// So this is the passenger seat: find the nearest bus or cab, sit the capsule
// on its pose each step, and put the player back on the kerb when they get off.
// It writes the character position directly rather than going through RideBook,
// because RideBook indexes AGENTS and the player is not one -- forcing the
// player into that table would mean giving them an agent slot, which is a much
// larger change for no gain here.
//
// Deliberately its own system: a bus, a cab and (later) a train differ only in
// which vehicle is chosen, so the seat is one mechanism and the choosing is a
// policy on top.

#include "../../engine/system.h"
#include "../../rt_math.h"   // engine::Vec3

#include <vector>

#include "../../engine/components.h"   // Transform, CharacterController
#include "../../engine/world.h"

namespace engine {
class PhysicsSystem;
class FlyCameraController;
}

namespace citysim {

class CityRenderSystem;

class CityPlayerTransitSystem : public engine::System {
public:
    // `fly` is the first-person view's controller: aboard a bus the view turns
    // with the bus, so a corner swings the saloon rather than the world.
    CityPlayerTransitSystem(CityRenderSystem& city, engine::PhysicsSystem& physics,
                            engine::FlyCameraController& fly)
        : city_(city), physics_(physics), fly_(fly) {}

    void onStart(engine::FrameContext& ctx) override;
    void fixedUpdate(engine::FrameContext& ctx) override;

    // --- testable core (no FrameContext), the CityRenderSystem pattern ------
    // One fixed step of the ride. `request`: -1 none, 0 get off, 1 bus, 2 cab,
    // 3 either (the `ride` verb); `interact` is E; forward/right the move axes.
    struct RideInput {
        double request = -1.0;
        bool interact = false;
        engine::Real forward = 0, right = 0;
    };
    void step(engine::World& world, engine::Real dt, const RideInput& in);
    int riding() const { return riding_; }
    void update(engine::FrameContext& ctx) override;
    void render(engine::FrameContext& ctx) override;

    // WHERE IS THE BUS (Glenn, 2026-09-18: "I couldn't find one and I walked
    // around for a while"). What the on-screen line says, refreshed every
    // frame. Public so a test can read it without drawing anything.
    struct Hud {
        enum class Mode { None, OnFoot, Riding } mode = Mode::None;
        engine::Real stopDistance = 0;   // to the nearest stop
        const char* stopDirection = "";  // relative to where you are looking
        // stopsAway -1 = no bus on the route. 0 = this is the bus's NEXT stop,
        // which can still be a few hundred metres off -- hence busDistance.
        struct Serving {
            int route = -1;
            int stopsAway = -1;
            engine::Real busDistance = -1;
            bool atStop = false;
        };
        std::vector<Serving> serving;    // every route calling at that stop
        bool inService = true;
        const char* boardable = nullptr; // "bus" / "cab" when E would board
        int boardableRoute = -1;
        int ridingRoute = -1;            // -1 in a cab
        engine::Real nextStopDistance = -1;
    };
    const Hud& hud() const { return hud_; }
    int seatHeld() const { return seatIdx_; }            // -1 standing
    const engine::Vec3& aboardAt() const { return local_; }   // bus-frame floor point

private:
    void rideBus(engine::World& world, engine::Real dt, const RideInput& in,
                 engine::Entity player, engine::Transform& t,
                 engine::CharacterController& cc, const engine::Mat4& pose, bool interact);
    void pin(engine::World& world, engine::Entity player, engine::Transform& t,
             engine::CharacterController& cc, const engine::Vec3& pos);
    void leave(engine::World& world, engine::Entity player, const engine::Vec3& where);

    CityRenderSystem& city_;
    engine::PhysicsSystem& physics_;
    engine::FlyCameraController& fly_;
    // Aboard a bus: where the player stands in the bus's OWN frame (a floor
    // point), the seat they hold (-1 standing), and the bus's yaw last step
    // (so the view can turn with it).
    engine::Vec3 local_{0, 0, 0};
    int seatIdx_ = -1;
    bool haveBusYaw_ = false;
    engine::Real lastBusYaw_ = 0;
    const char* busPrompt_ = "";
    int riding_ = -1;          // agent index of the vehicle we are aboard, or -1
    engine::Vec3 seat_{0, 0, 0};   // where we put the player last step
    bool seated_ = false;          // seat_ is meaningful
    bool wasAboard_ = false;   // for the one-shot log on boarding/alighting
    Hud hud_;
    bool boardEdge_ = false;   // E pressed since the last fixed step
};

}  // namespace citysim

#endif
