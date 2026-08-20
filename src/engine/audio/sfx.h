#ifndef RAYTRACER_ENGINE_AUDIO_SFX_H
#define RAYTRACER_ENGINE_AUDIO_SFX_H

#include <cstdint>
#include <vector>

namespace engine {
namespace sfx {

// Procedural sound effects (ADR-0069/0071): pure `(parameters, seed) -> PCM`
// generators, the audio counterpart of the procgen mesh builders (ADR-0021).
// Deterministic from the seed, mono float32 in [-1, 1], ready for
// AudioEngine::createClip — no sound files anywhere. Headless-tested.

// A gunshot: a fast noise crack over a decaying low-frequency thump.
std::vector<float> gunshot(uint32_t sampleRate = 48000, uint32_t seed = 1);

// A physical impact knock: damped inharmonic partials + a touch of contact
// noise. Different seeds vary the strike pitch, so repeated hits don't sound
// machine-gunned.
std::vector<float> impact(uint32_t sampleRate = 48000, uint32_t seed = 1);

// A car horn: the classic two-note dyad with brassy harmonics, as ONE steady
// loop-clean period (~0.25 s) — every partial completes an integer number of
// cycles in the buffer BY CONSTRUCTION, so a looping voice can hold the honk
// for as long as the key is down with no seam. The caller shapes attack and
// release with voice volume (see VehicleSystem's horn); baking an envelope in
// would make it repeat every loop. Seeds pick slightly different horn pitches,
// so not every car honks the same note.
std::vector<float> horn(uint32_t sampleRate = 48000, uint32_t seed = 1);

// A running engine, as ONE steady loop-clean period at a REFERENCE firing
// rate (kEngineRefHz below). It is never played at 1.0 for long: the caller
// holds a looping voice and shifts its PITCH with revs (engine/vehicle_audio.h
// turns speed into that multiplier), which is how a single sample covers idle
// through redline. Like the horn, bake no envelope — the loop would repeat it.
// Seeds vary the rasp partials so two cars in one street aren't clones.
std::vector<float> engine(uint32_t sampleRate = 48000, uint32_t seed = 1);

// The firing rate the clip is baked at: pitch 1.0 sounds like this many
// combustion events per second (~1700 rpm on a four-cylinder four-stroke, two
// firings per revolution). Callers multiply it; the mapping core owns the band.
constexpr double kEngineRefHz = 56.0;

}  // namespace sfx
}  // namespace engine

#endif
