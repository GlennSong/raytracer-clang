#ifndef RAYTRACER_ENGINE_SYSTEMS_LADDER_SYSTEM_H
#define RAYTRACER_ENGINE_SYSTEMS_LADDER_SYSTEM_H

#include "../ladder.h"
#include "../system.h"
#include "../asset_manager.h"
#include <unordered_map>

namespace engine {

// LADDERS as an interaction PROVIDER (engine/ladder.h). Builds the world's ladder list from the building records once
// a city has loaded (roof-access ladders: procgen/city/ladders.h), draws the ones near the player, and OFFERS the
// broker "climb" at a ladder's foot and "climb down" at its top -- the buildings' and the vehicles' alike. On the
// command it puts CLIMBING on the player; the PlayerSystem does the climbing.
class PhysicsSystem;
class LadderSystem : public System {
public:
    explicit LadderSystem(PhysicsSystem* physics = nullptr) : physics_(physics) {}
    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

    static constexpr Real DRAW_M = 140.0;      // a building's ladder is drawn within this of the player
    static constexpr Real RELEASE_M = 180.0;   // ...and let go beyond this
    static constexpr Real REACH = 1.1;         // from its foot, or its top, horizontally

private:
    PhysicsSystem* physics_ = nullptr;
    Entity laddersEntity_;
    struct Drawn { Entity e; MeshHandle mesh; };
    std::unordered_map<std::size_t, Drawn> drawn_;
    // THE FIRE ESCAPES' colliders round the player (record index -> its body): their stairs climb whatever is streamed
    std::unordered_map<std::size_t, uint64_t> escapes_;
    static constexpr Real ESCAPE_M = 120.0, ESCAPE_RELEASE_M = 170.0;
};

// Which ladder end, if any, a player standing with feet at `feet` can use: 0 none, 1 the foot (climb), 2 the top
// (climb down). Pure: headless-tested.
int ladderEndWithinReach(const LadderSpec& l, const Vec3& feet, Real reach = LadderSystem::REACH);

}  // namespace engine

#endif
