#ifndef RAYTRACER_ENGINE_MOTION_AUDIO_SYSTEM_H
#define RAYTRACER_ENGINE_MOTION_AUDIO_SYSTEM_H

#include "../system.h"
#include "../audio/audio_engine.h"
#include "../player_pose.h"

#include <cstdint>
#include <random>

namespace engine {

// THE PLAYER'S MOVES, heard (Glenn, 2026-10-24: "different sfx would help sell these types of motions too. Like
// skateboarding, parachuting, grapple, etc."). Reads PlayerPose (engine/player_pose.h) each frame and plays the
// procedural clips of sfx_motion.cpp at the body:
//  - LOOPS held by gain and pitch: air rushing past (by speed through the air, in every mode on foot), the canopy's
//    flutter, the wingsuit's buzz, the grapple's winch while reeling, a board's wheels rolling;
//  - ONE-SHOTS from the pose's counters: the hook fired, biting, the rope snapping taut, the canopy opening, an
//    ollie, a board landing; and a board's trucks over each pavement joint as it rolls.
// FootstepSystem stays silent while these modes own the body. Vehicles' own sounds are VehicleSystem's.
class MotionAudioSystem : public System {
public:
    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

private:
    struct Loop {
        AudioClipHandle clip;
        AudioVoiceHandle voice;
        float gain = 0;
    };
    // ease a loop toward `target` gain at `pitch`, starting its voice when it first sounds and stopping it when silent
    void drive(FrameContext& ctx, Loop& loop, float target, float pitch, const Vec3& at, float rate, float dt);
    void oneShot(FrameContext& ctx, AudioClipHandle clip, const Vec3& at, float volume, float pitch, Real range = 30.0);

    bool ready_ = false;
    Loop wind_, chute_, wingsuit_, winch_, roll_;
    AudioClipHandle fire_, bite_, twang_, open_, pop_, land_, clack_, kick_;
    // what the pose's counters said last frame (a jump in a counter is a sound)
    bool seen_ = false;
    uint32_t shots_ = 0, bites_ = 0, snaps_ = 0, opens_ = 0, ollies_ = 0, lands_ = 0;
    // the body's speed through the air, from where it went (smoothed)
    Vec3 lastPos_;
    bool havePos_ = false;
    Real airspeed_ = 0;
    // a board over pavement joints: metres to the next joint, and the back truck's clack still to come
    Real toJoint_ = 1.0;
    Real backTruckIn_ = -1;
    Real lastKick_ = 0;
    std::mt19937 rng_{0x6d6f7665};
};

}  // namespace engine

#endif
