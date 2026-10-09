#ifndef RAYTRACER_ENGINE_FOOTSTEP_SYSTEM_H
#define RAYTRACER_ENGINE_FOOTSTEP_SYSTEM_H

#include "../system.h"
#include "../audio/footsteps.h"
#include "../audio/audio_engine.h"
#include "../procgen/noise.h"

#include <array>
#include <cstdint>
#include <memory>
#include <random>

namespace engine {

class PhysicsSystem;
struct TerrainLodConfig;

// The player's feet, heard (#62 footsteps by surface, #63 jump and landing, #64 grass swish). Each fixed
// step it asks physics whether the player stands, how fast it moves and on WHAT (the ground body's
// surface tag; terrain asks the ground-cover map at the foot), feeds FootstepTracker, and plays the
// procedural clip (sfx::footstep / jumpPush / landing) the tracker calls for at the feet. A stride
// through TALL grass adds a swish, as loud as the grass there is thick and knee-high; a mown lawn is silent.
// Silent while the player is in a vehicle. All clips are synthesized at start (no sound files).
class FootstepSystem : public System {
public:
    explicit FootstepSystem(PhysicsSystem& physics) : physics_(physics) {}

    void onStart(FrameContext& ctx) override;
    void fixedUpdate(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

    // The ground a foot at (x, y, z) standing on a body tagged `surface` makes: the tag, or for terrain
    // the level's ground cover there. Public for tests and the debug overlay.
    // `slopeCos` (optional) gets the ground's normal y there (1 = flat; 1 off terrain).
    sfx::Ground groundAt(World& world, uint8_t surface, double x, double y, double z, double* slopeCos = nullptr);

private:
    static constexpr int kGrounds = static_cast<int>(sfx::Ground::Count);
    static constexpr int kStepVariants = 4;
    static constexpr int kLandLevels = 3;
    PhysicsSystem& physics_;
    bool ready_ = false;
    std::array<std::array<AudioClipHandle, kStepVariants>, kGrounds> steps_{};
    std::array<AudioClipHandle, kGrounds> jumps_{};
    std::array<std::array<AudioClipHandle, kLandLevels>, kGrounds> lands_{};
    std::array<AudioClipHandle, kStepVariants> swishes_{};
    // water (#43): a plunge, a stroke; wading uses sfx::Ground::Water steps
    std::array<AudioClipHandle, 3> splashes_{};   // stroke, wade in, plunge
    bool wasSwimming_ = false;
    double strokeTimer_ = 0.0;
    double fallBeforeWater_ = 0.0;
    double swishLevel_ = 0.0;         // 0..1: how much tall grass the legs are wading through here
    FootstepTracker tracker_;
    std::mt19937 rng_{0x5eed};
    int lastVariant_ = -1;
    Vec3 lastFeet_{};                 // where the feet were last tick: pace is what they MOVED, not what was asked
    bool haveLastFeet_ = false;
    uint8_t lastSurface_ = 0;         // the ground last stood on (a push-off happens as the feet leave it)
    sfx::Ground lastGround_ = sfx::Ground::Concrete;
    // the ground under the standing foot, re-read ~10x a second for the swish (#85)
    double hereTimer_ = 0.0;
    sfx::Ground groundHere_ = sfx::Ground::Concrete;
    double slopeHere_ = 1.0;
    std::unique_ptr<Noise> noise_;   // the terrain's, for levels drawn from the field (not a baked pyramid)
    uint32_t noiseSeed_ = 0;
    double groundHeight(const TerrainLodConfig& cfg, double x, double z);
};

}  // namespace engine

#endif
