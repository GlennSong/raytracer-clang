// The engine-sound decision core (engine/vehicle_audio.h): one predicate the
// player's car and the city's traffic both speak, so they rev alike. Pure —
// no audio device, no ECS.
#include "test_framework.h"

#include "../src/engine/vehicle_audio.h"

#include <cmath>

using namespace engine;  // namespace migration (ADR-0015)

TEST_CASE(engine_sound_is_silent_when_not_running) {
    // A parked, driverless car makes no sound — that is what makes a street of
    // stopped traffic read differently from an empty one.
    EngineSound off = engineSoundFor(0.0, 0.0, false);
    CHECK(off.gain == 0.0f);
    // ...even if it is somehow rolling.
    CHECK(engineSoundFor(18.0, 1.0, false).gain == 0.0f);
}

TEST_CASE(engine_sound_idles_audibly_and_low) {
    EngineSound idle = engineSoundFor(0.0, 0.0, true);
    CHECK(idle.gain > 0.05f);                             // audible
    CHECK(idle.gain < 0.35f);                             // but quiet
    CHECK(std::fabs(idle.pitch - static_cast<float>(
              kEnginePitchIdle + (kEnginePitchMax - kEnginePitchIdle) *
                                     kEngineIdleRev)) < 1e-5f);
    // Revving at a standstill is louder than trailing throttle at the same
    // revs (load), without changing pitch.
    EngineSound revved = engineSoundFor(0.0, 1.0, true);
    CHECK(revved.gain > idle.gain);
    CHECK(std::fabs(revved.pitch - idle.pitch) < 1e-5f);
}

TEST_CASE(engine_pitch_climbs_within_a_gear) {
    // Inside first gear, faster is higher — the ordinary accelerating note.
    EngineSound slow = engineSoundFor(3.0, 1.0, true);
    EngineSound mid = engineSoundFor(7.0, 1.0, true);
    EngineSound fast = engineSoundFor(10.5, 1.0, true);
    CHECK(mid.pitch > slow.pitch);
    CHECK(fast.pitch > mid.pitch);
    CHECK(fast.gain >= mid.gain);
}

TEST_CASE(engine_pitch_drops_across_every_shift) {
    // THE characteristic: revs fall as the box hands over. Without this an
    // engine is a siren whose pitch tracks the speedometer.
    for (int g = 0; g + 1 < kEngineGearCount; ++g) {
        const Real top = kEngineGearTops[g];
        EngineSound before = engineSoundFor(top - 0.05, 1.0, true);
        EngineSound after = engineSoundFor(top + 0.05, 1.0, true);
        CHECK(after.pitch < before.pitch);
    }
}

TEST_CASE(engine_sound_stays_in_band_across_the_whole_speed_range) {
    // Whatever the speed — including past the top gear and in reverse — the
    // playback rate stays inside the band one sample can cover without
    // artefacts, and the gain stays a legal voice volume.
    for (Real v = -30.0; v <= 90.0; v += 0.25) {
        for (Real pedal : {Real(-1.0), Real(0.0), Real(0.6), Real(1.0)}) {
            EngineSound s = engineSoundFor(v, pedal, true);
            CHECK(s.pitch >= static_cast<float>(kEnginePitchIdle) - 1e-5f);
            CHECK(s.pitch <= static_cast<float>(kEnginePitchMax) + 1e-5f);
            CHECK(s.gain >= 0.0f);
            CHECK(s.gain <= 1.0f);
        }
    }
    // Reverse revs like forward (speed magnitude, pedal magnitude).
    CHECK(std::fabs(engineSoundFor(-8.0, -1.0, true).pitch -
                    engineSoundFor(8.0, 1.0, true).pitch) < 1e-5f);
}

TEST_CASE(engine_creep_eases_off_idle_instead_of_jumping) {
    // Pulling away must not snap from idle revs to the bottom of first gear.
    EngineSound idle = engineSoundFor(0.0, 0.5, true);
    EngineSound creep = engineSoundFor(1.0, 0.5, true);
    EngineSound rolling = engineSoundFor(kEngineCreepSpeed, 0.5, true);
    CHECK(creep.pitch > idle.pitch);
    CHECK(creep.pitch < rolling.pitch);
}

TEST_CASE(engine_wobble_hunts_within_a_safe_band_and_never_syncs) {
    // The rev hunt that stops a loop reading as a hum. It must stay a nudge
    // (a wandering pitch is a broken engine, not a running one)...
    for (Real t = 0.0; t < 12.0; t += 0.01) {
        for (Real rev : {Real(0.16), Real(0.5), Real(1.0)}) {
            const float w = engineWobble(t, 0.0, rev);
            CHECK(w > 0.94f);
            CHECK(w < 1.06f);
        }
    }
    // ...deeper at idle than at revs (a lumpy tickover, a smooth top end)...
    auto swing = [](Real rev) {
        float lo = 2.0f, hi = 0.0f;
        for (Real t = 0.0; t < 20.0; t += 0.005) {
            const float w = engineWobble(t, 0.0, rev);
            lo = std::min(lo, w);
            hi = std::max(hi, w);
        }
        return hi - lo;
    };
    CHECK(swing(kEngineIdleRev) > swing(1.0));
    // ...and two cars must not hunt in step, or a street becomes a chord.
    bool differs = false;
    for (Real t = 0.0; t < 5.0; t += 0.05)
        if (std::fabs(engineWobble(t, 0.0, 0.5) - engineWobble(t, 0.37, 0.5)) >
            0.002f)
            differs = true;
    CHECK(differs);
}

TEST_CASE(engine_gets_louder_with_speed_overall) {
    // Gear drops dip the pitch, but a fast car is never quieter than a
    // crawling one — the gain floor rides revs, not the shift pattern.
    EngineSound crawl = engineSoundFor(1.0, 0.5, true);
    EngineSound cruise = engineSoundFor(25.0, 0.5, true);
    EngineSound flat = engineSoundFor(55.0, 1.0, true);
    CHECK(cruise.gain > crawl.gain);
    CHECK(flat.gain > cruise.gain);
}
