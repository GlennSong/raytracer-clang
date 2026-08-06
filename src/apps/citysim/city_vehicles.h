#ifndef RAYTRACER_APPS_CITYSIM_CITY_VEHICLES_H
#define RAYTRACER_APPS_CITYSIM_CITY_VEHICLES_H

#include "../../engine/system.h"
#include "../../renderer/renderer.h"   // engine::MeshHandle
#include "city_render.h"

#include <vector>

namespace citysim {

// A lens a PROMOTED car should carry, derived from its fleet slot's lamp
// markers. Kept pure and Jolt-free (this header's system class is viewer-only)
// so the mapping itself is directly testable: it is the thing that stops
// VehicleSystem from falling back to four lenses at guessed chassis corners,
// which is what drew a second pair of taillights beyond the bodywork.
struct PromotedLamp {
    engine::Vec3 local;
    bool front = true;   // headlight end vs tail end
    bool left = false;   // indicator side
};

// Split a slot's markers into lamp lenses. Markers that name neither a
// headlight nor a taillight are skipped, except `driver_seat`: when one is
// present its position is written to `driverSeat` and the function returns true
// through `hasDriverSeat` (it is where the seated occupant belongs; without it
// that too is guessed off the chassis box). Presence is reported explicitly
// rather than inferred from a non-zero position — a seat marker AT the body
// origin is unusual but legal, and must not read as "absent".
inline std::vector<PromotedLamp> promotedLamps(
    const std::vector<CityRenderSystem::LampMarker>& markers,
    engine::Vec3* driverSeat = nullptr, bool* hasDriverSeat = nullptr) {
    if (hasDriverSeat) *hasDriverSeat = false;
    std::vector<PromotedLamp> out;
    for (const CityRenderSystem::LampMarker& m : markers) {
        if (m.name.rfind("driver_seat", 0) == 0) {
            if (driverSeat) *driverSeat = m.pos;
            if (hasDriverSeat) *hasDriverSeat = true;
            continue;
        }
        const bool front = m.name.rfind("headlight", 0) == 0;
        const bool rear = m.name.rfind("taillight", 0) == 0;
        if (!front && !rear) continue;
        out.push_back({m.pos, front, !m.name.empty() && m.name.back() == 'l'});
    }
    return out;
}

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
    // No PhysicsSystem: promotion spawns a Vehicle COMPONENT and VehicleSystem
    // does every Jolt call (body, wheels, lamps). This system held a
    // PhysicsSystem& for symmetry with its walker counterpart, which really does
    // need one — here it was never once read.
    explicit CityVehicleSystem(CityRenderSystem& city) : city_(city) {}

    void update(engine::FrameContext& ctx) override;   // commandeer check (per frame)
    void onStop(engine::FrameContext& ctx) override;

private:
    struct PromotedCar {
        engine::Entity entity;   // the real Vehicle that replaced the ambient car
        int agentId = -1;        // the released sim agent
    };

    CityRenderSystem& city_;
    std::vector<PromotedCar> promoted_;
};

}  // namespace citysim

#endif
