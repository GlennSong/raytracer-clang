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

}  // namespace sfx
}  // namespace engine

#endif
