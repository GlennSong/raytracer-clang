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

namespace engine { class PhysicsSystem; }

namespace citysim {

class CityRenderSystem;

class CityPlayerTransitSystem : public engine::System {
public:
    CityPlayerTransitSystem(CityRenderSystem& city, engine::PhysicsSystem& physics)
        : city_(city), physics_(physics) {}

    void onStart(engine::FrameContext& ctx) override;
    void fixedUpdate(engine::FrameContext& ctx) override;
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

private:
    CityRenderSystem& city_;
    engine::PhysicsSystem& physics_;
    int riding_ = -1;          // agent index of the vehicle we are aboard, or -1
    engine::Vec3 seat_{0, 0, 0};   // where we put the player last step
    bool seated_ = false;          // seat_ is meaningful
    bool wasAboard_ = false;   // for the one-shot log on boarding/alighting
    Hud hud_;
    bool boardEdge_ = false;   // E pressed since the last fixed step
};

}  // namespace citysim

#endif
