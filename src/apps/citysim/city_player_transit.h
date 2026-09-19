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

namespace engine { class PhysicsSystem; }

namespace citysim {

class CityRenderSystem;

class CityPlayerTransitSystem : public engine::System {
public:
    CityPlayerTransitSystem(CityRenderSystem& city, engine::PhysicsSystem& physics)
        : city_(city), physics_(physics) {}

    void onStart(engine::FrameContext& ctx) override;
    void fixedUpdate(engine::FrameContext& ctx) override;

private:
    CityRenderSystem& city_;
    engine::PhysicsSystem& physics_;
    int riding_ = -1;          // agent index of the vehicle we are aboard, or -1
    engine::Vec3 seat_{0, 0, 0};   // where we put the player last step
    bool seated_ = false;          // seat_ is meaningful
    bool wasAboard_ = false;   // for the one-shot log on boarding/alighting
};

}  // namespace citysim

#endif
