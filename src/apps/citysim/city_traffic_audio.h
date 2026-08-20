#ifndef RAYTRACER_APPS_CITYSIM_CITY_TRAFFIC_AUDIO_H
#define RAYTRACER_APPS_CITYSIM_CITY_TRAFFIC_AUDIO_H

#include "../../engine/system.h"
#include "../../engine/audio/audio_engine.h"
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
// A VOICE POOL, not a voice per car: the nearest few driving agents get the
// mixer's slots and everything else is silence you would not have picked out
// anyway. Voices follow their agent across frames (so a car keeps its note as
// it passes) and fade rather than cut when they lose their slot.
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
    struct Voice {
        int agent = -1;
        engine::AudioVoiceHandle voice;
        float gain = 0.0f;
        engine::Real prevSpeed = 0;   // for the load (accelerating?) estimate
        bool wanted = false;
    };

    CityRenderSystem& city_;
    engine::AudioClipHandle clip_;
    std::vector<Voice> voices_;
};

}  // namespace citysim

#endif
