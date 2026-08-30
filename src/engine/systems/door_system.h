#ifndef RAYTRACER_ENGINE_SYSTEMS_DOOR_SYSTEM_H
#define RAYTRACER_ENGINE_SYSTEMS_DOOR_SYSTEM_H

#include "../asset_manager.h"
#include "../system.h"
#include "../procgen/city/building_records.h"
#include <cstdint>
#include <unordered_map>

namespace engine {

// Which way a double-acting leaf swings (ADR-0080 Phase 3, Glenn's rule:
// "doors need to swing in the direction the player/npc is moving to get out
// of the way of them"). A mover heading through the aperture pushes the leaf
// AHEAD of themselves (velocity decides); a loiterer at the threshold gets
// it swung to the side they are NOT on. Returns -1 = swing to the inside
// (-normal side), +1 = to the outside. Pure — unit-tested headless.
Real doorSwingSign(const Vec2& moverPos, const Vec2& moverVel,
                   const Vec2& foot, const Vec2& normal);

// The door LEAF: visual-only, over a physically OPEN aperture — no body, no
// interact verb, so it can never trap or push anyone (the collider notch is
// the passage; the leaf is theatre). One leaf entity per door of an
// enterable building near the player (ACTIVE_M), opening as a mover comes
// within TRIGGER_M — eased, closing CLOSE_S after the doorway is clear.
// Entities come and go like walkers (city_walkers.cpp): created when the
// player nears, destroyed when they leave.
class DoorSystem : public System {
public:
    void fixedUpdate(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

    static constexpr double TRIGGER_M = 2.5;
    static constexpr double ACTIVE_M = 60.0;
    static constexpr double CLOSE_S = 2.0;
    static constexpr double SWING_RAD = 1.48;      // ~85 deg, clears the jamb
    static constexpr double RATE_RAD_S = 4.0;      // swing speed

private:
    struct Leaf {
        Entity entity;
        MeshHandle mesh;
        Real angle = 0;        // signed swing, radians
        Real target = 0;
        double clearFor = 1e9; // seconds since a mover was near
        Vec2 foot, normal;
        Real width = 2.0, height = 2.7;
        Real footY = 0;
    };
    std::unordered_map<int64_t, Leaf> leaves_;   // (record << 8) | door
};

}  // namespace engine

#endif
