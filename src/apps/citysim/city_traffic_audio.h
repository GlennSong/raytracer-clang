#ifndef RAYTRACER_APPS_CITYSIM_CITY_TRAFFIC_AUDIO_H
#define RAYTRACER_APPS_CITYSIM_CITY_TRAFFIC_AUDIO_H

#include "../../engine/system.h"
#include "../../engine/audio/engine_voices.h"   // EngineVoicePool
#include "city_render.h"

#include <vector>

namespace citysim {

// Engine note for AMBIENT traffic — the citysim adapter over
// engine::engineSoundFor, exactly as car_lamps.h adapts the lamp core. The
// city's cars are planner ghosts, not physics bodies, so nothing in the
// engine-side VehicleSystem can voice them; but they are cars, and a street
// with forty of them should sound like one. Same core, same clip, same rev
// curve — the difference is only where the speed comes from.
//
// The nearest 8 driving agents within 70 m, voiced by the engine pool (engine/audio/engine_voices.h) the physics
// cars share; seed 2's rasp, so the player's own engine (seed 1) still reads as theirs. The planner gives no pedal,
// so the pool reads each car's load from its change of speed.
//
// Commandeered agents (`released`) are skipped: the player's car is a real
// physics Vehicle by then, and VehicleSystem voices it — one car, one engine.
class CityTrafficAudioSystem : public engine::System {
public:
    explicit CityTrafficAudioSystem(CityRenderSystem& city) : city_(city) {}

    void update(engine::FrameContext& ctx) override;
    void onStop(engine::FrameContext& ctx) override;

    // How many ambient cars can be heard at once, and how far a car carries.
    static constexpr int kMaxVoices = 8;
    static constexpr engine::Real kAudibleRange = 70.0;

private:
    CityRenderSystem& city_;
    engine::EngineVoicePool engines_{kMaxVoices, kAudibleRange, 2};
};

}  // namespace citysim

#endif
