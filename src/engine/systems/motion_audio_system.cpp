#include "motion_audio_system.h"

#include "../audio/sfx.h"
#include "../components.h"
#include "../interaction.h"   // Seated

#include <algorithm>
#include <cmath>

namespace engine {

namespace {
AudioClipHandle makeClip(AudioEngine& audio, const std::vector<float>& pcm) {
    return audio.createClip(pcm.data(), pcm.size(), 1, audio.sampleRate());
}
float clamp01(Real x) { return static_cast<float>(std::clamp(x, Real(0), Real(1))); }
constexpr Real kJoint = 1.6;   // m between pavement joints (sidewalk slabs)
}  // namespace

void MotionAudioSystem::onStart(FrameContext& ctx) {
    ready_ = ctx.audio.ready();
    if (!ready_) return;
    const uint32_t r = ctx.audio.sampleRate();
    wind_.clip = makeClip(ctx.audio, sfx::windRush(r, 1));
    chute_.clip = makeClip(ctx.audio, sfx::clothFlutter(9.0, r, 2));
    wingsuit_.clip = makeClip(ctx.audio, sfx::clothFlutter(24.0, r, 3));
    winch_.clip = makeClip(ctx.audio, sfx::reelWinch(r, 1));
    roll_.clip = makeClip(ctx.audio, sfx::boardRoll(r, 1));
    fire_ = makeClip(ctx.audio, sfx::hookFire(r, 1));
    bite_ = makeClip(ctx.audio, sfx::hookBite(r, 1));
    twang_ = makeClip(ctx.audio, sfx::ropeTwang(r, 1));
    open_ = makeClip(ctx.audio, sfx::canopyOpen(r, 1));
    pop_ = makeClip(ctx.audio, sfx::ollie(r, 1));
    land_ = makeClip(ctx.audio, sfx::boardLand(r, 1));
    clack_ = makeClip(ctx.audio, sfx::boardClack(r, 1));
    kick_ = makeClip(ctx.audio, sfx::jumpPush(sfx::Ground::Concrete, r, 9));   // a shoe pushing off the pavement
    grab_ = makeClip(ctx.audio, sfx::handGrab(r, 1));
    hold_ = makeClip(ctx.audio, sfx::handHold(r, 1));
    metal_ = makeClip(ctx.audio, sfx::metalGrab(r, 1));
}

void MotionAudioSystem::onStop(FrameContext& ctx) {
    for (Loop* l : {&wind_, &chute_, &wingsuit_, &winch_, &roll_}) {
        if (l->voice.valid()) ctx.audio.stop(l->voice);
        l->voice = AudioVoiceHandle{};
        l->gain = 0;
    }
    seen_ = false;
    havePos_ = false;
}

void MotionAudioSystem::drive(FrameContext& ctx, Loop& loop, float target, float pitch, const Vec3& at, float rate,
                              float dt) {
    loop.gain += (target - loop.gain) * std::min(1.0f, rate * dt);
    if (!loop.voice.valid() || !ctx.audio.isPlaying(loop.voice)) {
        loop.voice = AudioVoiceHandle{};
        if (loop.gain < 0.005f || !loop.clip.valid()) return;
        AudioPlayParams p;
        p.loop = true;
        p.volume = 0.0f;
        p.pitch = pitch;
        loop.voice = ctx.audio.playAt(loop.clip, at, 40.0, p);
        if (!loop.voice.valid()) return;
    }
    if (loop.gain < 0.003f && target <= 0.0f) {
        ctx.audio.stop(loop.voice);
        loop.voice = AudioVoiceHandle{};
        loop.gain = 0;
        return;
    }
    ctx.audio.setVoiceVolume(loop.voice, loop.gain);
    ctx.audio.setVoicePitch(loop.voice, std::max(0.05f, pitch));
    ctx.audio.setVoicePosition(loop.voice, at);
}

void MotionAudioSystem::oneShot(FrameContext& ctx, AudioClipHandle clip, const Vec3& at, float volume, float pitch,
                                Real range) {
    if (!clip.valid() || volume <= 0.01f) return;
    std::uniform_real_distribution<float> vary(0.94f, 1.06f);
    AudioPlayParams p;
    p.volume = std::min(1.0f, volume);
    p.pitch = pitch * vary(rng_);
    ctx.audio.playAt(clip, at, range, p);
}

void MotionAudioSystem::update(FrameContext& ctx) {
    if (!ready_) return;
    const float dt = static_cast<float>(std::clamp(ctx.frameDelta, 0.0, 0.1));
    Entity player;
    ctx.world.each<Transform, CharacterController, ControlledBy>(
        [&](Entity e, Transform&, CharacterController&, ControlledBy&) {
            if (!player.valid()) player = e;
        });
    const PlayerPose* pose = player.valid() ? ctx.world.get<PlayerPose>(player) : nullptr;
    const Transform* body = player.valid() ? ctx.world.get<Transform>(player) : nullptr;
    const bool onFoot = pose && body && !ctx.world.has<InVehicle>(player) && !ctx.world.has<Seated>(player) &&
                        !ctx.world.has<Passenger>(player) && pose->mode != PlayerPose::Mode::Hidden;
    if (!onFoot) {
        for (Loop* l : {&wind_, &chute_, &wingsuit_, &winch_, &roll_}) drive(ctx, *l, 0, 1, ctx.view.camera.position, 10, dt);
        havePos_ = false;
        if (pose) {   // the counters move on without us: no burst of stale sounds on the way back out
            shots_ = pose->hookShots; bites_ = pose->hookBites; snaps_ = pose->ropeSnaps;
            opens_ = pose->chuteOpens; ollies_ = pose->ollies; lands_ = pose->boardLands; grabs_ = pose->grabs;
            seen_ = true;
        }
        return;
    }
    const Vec3 at = body->position;
    using M = PlayerPose::Mode;
    // SPEED THROUGH THE AIR, from where the body went (a teleport is not a gale)
    if (havePos_ && dt > 0) {
        const Real v = (at - lastPos_).length() / dt;
        if (v < 150) airspeed_ += (v - airspeed_) * std::min(1.0, 6.0 * dt);
    }
    lastPos_ = at;
    havePos_ = true;

    // THE WIND: nothing at a run, a roar in a dive (the wingsuit at 50 m/s is near full)
    const Real w = std::clamp((airspeed_ - 6.0) / 44.0, 0.0, 1.0);
    const float windGain = pose->underwater ? 0.0f : static_cast<float>(0.85 * std::pow(w, 1.2));
    drive(ctx, wind_, windGain, static_cast<float>(0.75 + 0.6 * w), at, 4, dt);
    // THE CANOPY flapping overhead, and the WINGSUIT's buzz at the arms
    drive(ctx, chute_, pose->mode == M::Parachute ? 0.45f * clamp01(pose->canopyOpen) : 0.0f,
          static_cast<float>(0.9 + 0.02 * airspeed_), at + Vec3(0, 3.5, 0), 5, dt);
    drive(ctx, wingsuit_, pose->mode == M::Wingsuit ? 0.15f + 0.45f * clamp01(airspeed_ / 50.0) : 0.0f,
          static_cast<float>(0.75 + 0.5 * std::clamp(airspeed_ / 50.0, 0.0, 1.0)), at, 5, dt);
    // THE WINCH while the rope reels: higher as it goes faster, lower letting out
    const Real reel = pose->mode == M::Grapple ? pose->reel : 0;
    drive(ctx, winch_, std::fabs(reel) > 0.5 ? 0.5f : 0.0f,
          static_cast<float>(reel > 0 ? 0.6 + reel / 12.0 : 0.55 + 0.6 * -reel / 12.0), at + Vec3(0, 0.6, 0), 14, dt);
    // A BOARD'S WHEELS on the ground (the scooter's small hard wheels higher), silent in the air
    const bool rolling = pose->mode == M::Board && pose->grounded && pose->speed > 0.3;
    const Real sp = pose->speed;
    const Real scooter = pose->board == 1 ? 1.35 : 1.0;
    drive(ctx, roll_, rolling ? static_cast<float>(0.7 * std::pow(std::clamp(sp / 7.0, 0.0, 1.0), 0.7)) : 0.0f,
          static_cast<float>((0.6 + sp / 9.0) * scooter), at - Vec3(0, 0.9, 0), rolling ? 12 : 25, dt);
    // ...and its trucks over each pavement joint: the front, then the back a wheelbase later
    if (rolling) {
        toJoint_ -= sp * dt;
        if (toJoint_ <= 0) {
            toJoint_ += kJoint;
            oneShot(ctx, clack_, at - Vec3(0, 0.9, 0), clamp01(sp / 8.0) * 0.6f, static_cast<float>(scooter));
            backTruckIn_ = (pose->board == 1 ? 0.8 : 0.55) / std::max(sp, Real(0.5));
        }
    }
    if (backTruckIn_ >= 0 && (backTruckIn_ -= dt) < 0)
        oneShot(ctx, clack_, at - Vec3(0, 0.9, 0), clamp01(sp / 8.0) * 0.5f, static_cast<float>(scooter * 0.95));
    // a kick: the shoe pushing off
    if (pose->mode == M::Board && pose->kick > 0 && lastKick_ <= 0) oneShot(ctx, kick_, at - Vec3(0, 0.9, 0), 0.5f, 1.0f);
    lastKick_ = pose->mode == M::Board ? pose->kick : 0;

    // ONE-SHOTS, from the counters
    if (!seen_) {
        shots_ = pose->hookShots; bites_ = pose->hookBites; snaps_ = pose->ropeSnaps;
        opens_ = pose->chuteOpens; ollies_ = pose->ollies; lands_ = pose->boardLands; grabs_ = pose->grabs;
        seen_ = true;
    }
    const Vec3 hand = at + Vec3(0, 0.75, 0);
    const Vec3 tip = pose->ropePoints > 0 ? pose->rope[pose->ropePoints - 1] : hand;
    if (pose->hookShots != shots_) oneShot(ctx, fire_, hand, 0.8f, 1.0f);
    if (pose->hookBites != bites_) oneShot(ctx, bite_, tip, 1.0f, 1.0f, 80.0);
    if (pose->ropeSnaps != snaps_) oneShot(ctx, twang_, hand, clamp01(pose->snapSpeed / 8.0), 1.0f);
    if (pose->chuteOpens != opens_) oneShot(ctx, open_, at + Vec3(0, 2.0, 0), 1.0f, 1.0f, 60.0);
    if (pose->ollies != ollies_) oneShot(ctx, pop_, at - Vec3(0, 0.9, 0), 0.9f, pose->board == 1 ? 1.2f : 1.0f);
    if (pose->boardLands != lands_)
        oneShot(ctx, land_, at - Vec3(0, 0.9, 0), clamp01(0.3 + pose->landSpeed / 6.0), pose->board == 1 ? 1.2f : 1.0f);
    // THE HANDS: catching hold (both, the weight coming on), and climbing, each hand set on the next hold
    const bool metal = pose->mode == M::Ladder || pose->mode == M::Bars;
    const Vec3 hands = (pose->handL + pose->handR) * 0.5;
    const Vec3 handsAt = (hands - at).lengthSquared() < 9.0 ? hands : at + Vec3(0, 0.8, 0);
    if (pose->grabs != grabs_) oneShot(ctx, metal ? metal_ : grab_, handsAt, metal ? 0.7f : 0.9f, 1.0f);
    const bool climbing = pose->mode == M::Climb || pose->mode == M::Ladder;
    if (climbing && std::floor(pose->climbCycle * 2.0) != std::floor(lastClimb_ * 2.0)) {
        const bool left = static_cast<long>(std::floor(pose->climbCycle * 2.0)) % 2 == 0;
        oneShot(ctx, pose->mode == M::Ladder ? metal_ : hold_, left ? pose->handL : pose->handR,
                pose->mode == M::Ladder ? 0.35f : 0.55f, pose->mode == M::Ladder ? 1.1f : 1.0f);
    }
    lastClimb_ = pose->climbCycle;
    shots_ = pose->hookShots; bites_ = pose->hookBites; snaps_ = pose->ropeSnaps;
    opens_ = pose->chuteOpens; ollies_ = pose->ollies; lands_ = pose->boardLands; grabs_ = pose->grabs;
}

}  // namespace engine
