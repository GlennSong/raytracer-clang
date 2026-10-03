#ifndef RAYTRACER_ENGINE_INTERACT_BROKER_H
#define RAYTRACER_ENGINE_INTERACT_BROKER_H

// THE INTERACTION BROKER (Glenn, 2026-10-01: "E sounds like it's a universal interaction button ... some way to
// highlight things close to you or if you're in a volume ... it could display the right interaction"). E belongs to
// ONE owner. Every system that can be used -- furniture, a lift, a bus -- OFFERS what it can do from where the
// player stands, each frame; the broker picks ONE focus (the offer the player looks at, standing in its volume
// counts most), marks it on screen, and on E (tap) or E held hands that offer back to its provider as a command.
// Nothing else reads E, so one press can never do two things.
//
//   offers   InteractOffers on the player, filled during update() by the providers and cleared by the broker
//   command  InteractCommand on the player, written by the broker, read (and removed) by the named provider the
//            next frame -- a one-frame hand-off, so the order systems run in does not matter
//
// Two kinds of offer: a POINT (within `reach` of the feet, roughly in view, nothing solid in between) or a VOLUME
// the provider says the player stands in (a lift's call spot, a bus aisle) -- no reach or look test. An EXCLUSIVE
// offer (stand up while seated) hides every other. More than one candidate: Tab, or the mouse wheel in first
// person, moves the focus through them.

#include "../rt_math.h"
#include "world.h"
#include <cstdint>
#include <string>
#include <vector>

namespace engine {

struct InteractOffer {
    std::string provider;      // "furniture", "elevator", "transit", ...: who acts on it
    Entity entity;             // the provider's own handle on the thing (optional)
    uint64_t key = 0;          // ...and its own id within it
    Vec3 anchor{0, 0, 0};      // where the marker goes, world
    std::string tap;           // what a tap does ("sit"); empty = nothing
    std::string hold;          // what a hold does ("lie down"); empty = nothing
    std::string name;          // what it is ("sofa", "elevator"), for the prompt
    Real reach = 1.8;          // POINT: horizontal metres from the feet
    bool inVolume = false;     // VOLUME: the player stands in it (no reach / look test)
    bool exclusive = false;    // while offered, the only one (and no marker)
    bool needsSight = true;    // POINT: nothing solid between the eye and the anchor
};

struct InteractOffers {
    std::vector<InteractOffer> list;
};

struct InteractCommand {
    std::string provider;
    Entity entity;
    uint64_t key = 0;
    bool hold = false;
};

// Add an offer to the player (makes the component on first use: structural, so not inside a World::each).
void offerInteraction(World& world, Entity player, InteractOffer offer);
// The command addressed to `provider`, if one is waiting: copies it out and removes it.
bool takeInteractCommand(World& world, Entity player, const std::string& provider, InteractCommand& out);

// THE CHOICE (pure; headless-tested). Ranks the offers a player at `feet` with the eye at `eye` looking along
// `forward` could use, best first. `blocked(eye, anchor)` answers the sight test (nullptr: never blocked).
struct RankedOffer {
    std::size_t index = 0;   // into the offers
    Real score = 0;          // lower is better
};
std::vector<RankedOffer> rankInteractions(const std::vector<InteractOffer>& offers, const Vec3& feet,
                                          const Vec3& eye, const Vec3& forward,
                                          bool (*blocked)(const Vec3&, const Vec3&, void*) = nullptr,
                                          void* blockedCtx = nullptr);

// Where a world point lands on screen, in pixels, for a camera at `pos` looking at `target` (fov vertical, deg).
// False behind the camera.
bool projectToScreen(const Vec3& p, const Vec3& pos, const Vec3& target, const Vec3& up, Real fovDeg, Real aspect,
                     Real width, Real height, Real& sx, Real& sy);

}  // namespace engine

#endif
