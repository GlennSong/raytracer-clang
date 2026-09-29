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

// ---- The ground under a foot (#62/#63/#64/#65) --------------------------------------------------
// What a footstep lands on, as far as the ear cares. The surface resolver (engine/audio/footsteps.h)
// picks one from the collider's tag (roads) or the ground-cover map (terrain).
enum class Ground : uint8_t { Asphalt = 0, Concrete, Grass, Dirt, Sand, Rock, Snow, Wood, Metal, Count };
const char* groundName(Ground g);   // "asphalt", "concrete", ... (clip names: "sfx/step/<name>/<n>")

// One footstep on `ground`: the heel's contact, the body's weight, and the surface's own texture --
// grit scraped on asphalt, a clean click on concrete, a swish on grass, a dull crumble on dirt, a soft
// shush on sand, loose stones on rock, the compressed crunch of snow. ~0.1-0.35 s, one-shot. Seeds vary
// it so a walk is never the same step twice.
std::vector<float> footstep(Ground ground, uint32_t sampleRate = 48000, uint32_t seed = 1);
// Pushing off for a jump: the same surface scuffed harder and shorter, no weight landing.
std::vector<float> jumpPush(Ground ground, uint32_t sampleRate = 48000, uint32_t seed = 1);
// Landing: `heavy` 0..1 from a hop to a drop off a wall -- a bigger, lower body thud under the
// surface's texture, longer as it grows. The caller also scales volume by impact speed.
std::vector<float> landing(Ground ground, double heavy, uint32_t sampleRate = 48000, uint32_t seed = 1);

// Ambience LOOPS, seamless by construction (circular filtering, integer-cycle envelopes), for a
// looping voice whose volume the caller drives:
std::vector<float> grassRustle(uint32_t sampleRate = 48000, uint32_t seed = 1);   // walking through tall grass, ~2 s
std::vector<float> surf(uint32_t sampleRate = 48000, uint32_t seed = 1);          // waves on a beach, two swells, ~11 s
// Running water, ~4 s. `size` 0..1: 0 a narrow, fast stream (a dense babble of small bright bubbles),
// 1 a broad river (fewer, deeper bubbles over a low roar). Physically based: each bubble rings at its
// Minnaert frequency (~3.3 m / radius) with the damping a real bubble has, rising in pitch as it closes.
std::vector<float> river(uint32_t sampleRate = 48000, uint32_t seed = 1, double size = 0.5);
// Heard from UNDER the water: the same deep bubbling, muffled -- no surface sounds, nothing above ~900 Hz.
std::vector<float> underwaterRiver(uint32_t sampleRate = 48000, uint32_t seed = 1);

// The firing rate the clip is baked at: pitch 1.0 sounds like this many
// combustion events per second (~1700 rpm on a four-cylinder four-stroke, two
// firings per revolution). Callers multiply it; the mapping core owns the band.
constexpr double kEngineRefHz = 56.0;

}  // namespace sfx
}  // namespace engine

#endif
