#ifndef RAYTRACER_APPS_CITYSIM_CITY_VEHICLES_H
#define RAYTRACER_APPS_CITYSIM_CITY_VEHICLES_H

#include "../../engine/system.h"
#include "../../engine/systems/physics_system.h"
#include "../../renderer/renderer.h"   // engine::MeshHandle
#include "city_render.h"

#include <vector>

namespace citysim {

// PROMOTION-on-interaction (ADR-0062, motion-authority rethink). Ambient NPC
// traffic is moved by ONE authority — the CitySim planner, drawn instanced and
// collided via kinematic proxies (CityPhysicsSystem). Full vehicle dynamics is
// an INTERACTION response, not the default gait of forty cars: when the player
// walks up to an ambient car and presses enter, this system PROMOTES it — the
// sim releases the agent (its instanced car vanishes), a real engine Vehicle
// with the same fleet body spawns in its place (wheel-less mesh; VehicleSystem
// adds physics wheels + lamps), and VehicleSystem's normal enter flow seats the
// player in it that same frame. One car system at the component level; two
// motion regimes, each owned by the thing that's good at it.
//
// (The earlier all-dynamic bridge — every NPC car a Jolt vehicle chasing a
// planner ghost through tether/pursuit/station control — is retired: two clocks
// over one body needed endless reconciliation and jammed junctions. The
// pursuit/sensing controller stack remains in engine/ai, harness-tested, for
// promoted DYNAMIC cars that need AI driving later, e.g. hit reactions.)
//
// Viewer/editor only (needs Jolt via PhysicsSystem/VehicleSystem), not
// engine_core. UNVERIFIED on device, like the rest of the Jolt path.
class CityVehicleSystem : public engine::System {
public:
    CityVehicleSystem(CityRenderSystem& city, engine::PhysicsSystem& physics)
        : city_(city), physics_(physics) {}

    void update(engine::FrameContext& ctx) override;   // commandeer check (per frame)
    void onStop(engine::FrameContext& ctx) override;

private:
    struct PromotedCar {
        engine::Entity entity;   // the real Vehicle that replaced the ambient car
        int agentId = -1;        // the released sim agent
    };

    CityRenderSystem& city_;
    engine::PhysicsSystem& physics_;
    std::vector<PromotedCar> promoted_;
};

}  // namespace citysim

#endif
