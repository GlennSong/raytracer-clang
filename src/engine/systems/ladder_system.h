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
class LadderSystem : public System {
public:
    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

    static constexpr Real DRAW_M = 140.0;      // a building's ladder is drawn within this of the player
    static constexpr Real RELEASE_M = 180.0;   // ...and let go beyond this
    static constexpr Real REACH = 1.1;         // from its foot, or its top, horizontally

private:
    Entity laddersEntity_;
    struct Drawn { Entity e; MeshHandle mesh; };
    std::unordered_map<std::size_t, Drawn> drawn_;
};

// Which ladder end, if any, a player standing with feet at `feet` can use: 0 none, 1 the foot (climb), 2 the top
// (climb down). Pure: headless-tested.
int ladderEndWithinReach(const LadderSpec& l, const Vec3& feet, Real reach = LadderSystem::REACH);

}  // namespace engine

#endif
