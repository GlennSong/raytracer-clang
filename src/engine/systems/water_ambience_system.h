#ifndef RAYTRACER_ENGINE_WATER_AMBIENCE_SYSTEM_H
#define RAYTRACER_ENGINE_WATER_AMBIENCE_SYSTEM_H

#include "../system.h"
#include "../audio/audio_engine.h"

#include <memory>
#include <vector>

namespace engine {

struct TerrainLodConfig;

// The water you can hear (#65): surf where the land meets the sea, running water by a river. Two
// looping POSITIONAL voices on the Ambient bus. The surf voice sits at the nearest point of the
// coast found around the listener (ground below sea level on rings out to ~160 m, a few samples a
// frame); the river voice at the nearest river bank (Hydrology::distanceToRiver). Their positions
// glide, so the sound moves with the shore rather than jumping, and the mixer's distance falloff
// fades them as you walk away. Lakes stay quiet (still water). A level with no sea and no rivers
// makes no sound and costs nothing.
class WaterAmbienceSystem : public System {
public:
    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

    static constexpr double kSurfRange = 170.0;    // audible out to here from the coast point
    static constexpr double kRiverRange = 80.0;

private:
    bool ready_ = false;
    AudioClipHandle surfClip_{}, riverClip_{}, bigRiverClip_{};
    AudioVoiceHandle surfVoice_{}, riverVoice_{}, bigRiverVoice_{};
    Vec3 surfAt_{}, riverAt_{}, bigRiverAt_{};
    bool haveSurf_ = false, haveRiver_ = false, haveBigRiver_ = false;
    double surfGain_ = 0, riverGain_ = 0, bigRiverGain_ = 0;
    // #87: a narrow stream babbles, a broad river roars -- two loops crossfaded by the nearest river's width
    double riverSize_ = 0.5;          // 0 stream .. 1 broad river
    double riverSizeTimer_ = 0.0;
    // UNDER the water (SceneLighting::underwater, from UnderwaterSystem): the surface sounds duck away and a
    // muffled deep bubbling surrounds you (Glenn, of the muffled river: "works good for underwater")
    AudioClipHandle underClip_{};
    AudioVoiceHandle underVoice_{};
    double underGain_ = 0.0;
    bool surfFound_ = false;        // the last finished sweep's answer
    Vec3 surfTarget_{};
    double surfLevel_ = 0;
    // the coast search, spread over frames: sample index into the ring pattern, and this sweep's best
    int sweep_ = 0;
    double bestD2_ = 1e30;
    Vec3 best_{};
    Vec3 sweepCentre_{};
};

}  // namespace engine

#endif
