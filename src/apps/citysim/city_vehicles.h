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

// PROMOTION-on-interaction (ADR-0062, motion-authority rethink). Ambient traffic
// is moved by ONE authority — the driving, drawn instanced and collided via
// kinematic proxies (CityPhysicsSystem). Full vehicle dynamics is an INTERACTION
// response, not the default gait of forty cars: when the player walks up to an
// ambient car and presses enter, this system PROMOTES it, and VehicleSystem's
// normal enter flow seats the player in it that same frame.
//
// Every ambient car is a CAR ENTITY (a household's, ADR-0445, or one that drives round for ever, ADR-0453; city_car.h),
// so nothing spawns in its place: it is taken the one way (CitySim::playerTakesCar, ADR-0455: its driver gets out on
// the kerb side, it leaves the driving), and the same entity gains the Vehicle. The city bake stops drawing it;
// VehicleSystem draws, moves and voices it from then. When the player gets out it stays a Vehicle where they left it
// (VehicleSystem parks it) until the near tier gives a household's car back once it has stopped (#293,
// CitySim::playerLeftCar). (The old driving's cars, whose drivers were released and a new Vehicle spawned in their
// place, went with it: ADR-0461.)
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
        engine::Entity entity;   // the real Vehicle: the car entity itself
        int car = -1;            // its number (ADR-0445)
    };
    // AN AMBIENT CAR the player could take: a car entity, by its number (`car`; ADR-0445, ADR-0453).
    struct Ambient {
        int car = -1;
        bool valid() const { return car >= 0; }
    };

    CityRenderSystem& city_;
    engine::World* world_ = nullptr;   // this frame's (update)
    std::vector<PromotedCar> promoted_;
    // the ambient car offered to the interaction broker (rescanned four times a second)
    engine::Real scanT_ = 0;
    Ambient ambient_;   // the nearest ambient car in reach, rescanned four times a second
    // (ADR-0356) make an ambient car a real Vehicle -- the takeover, and a parked car's hood or trunk opened on foot
    engine::Entity promote(engine::FrameContext& ctx, const Ambient& target);
    // The nearest ambient car within kCommandeerRadius of `p` (city_car.h carsNear: the car entities).
    Ambient nearestAmbient(engine::World& world, engine::Vec2 p) const;
    // Where it is (on the ground, as agentWorldPose answers), the model it is drawn as, and whether it is on a trip.
    bool ambientPose(const Ambient& a, engine::Vec3& pos, engine::Vec2& heading) const;
    int ambientSlot(const Ambient& a) const;
    bool ambientMoving(const Ambient& a) const;
    bool haveAmbient_ = false;
    engine::Vec3 ambientAt_{0, 0, 0};
};

}  // namespace citysim

#endif
