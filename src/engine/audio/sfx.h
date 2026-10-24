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
enum class Ground : uint8_t { Asphalt = 0, Concrete, Grass, Dirt, Sand, Rock, Snow, Wood, Metal, Water, Count };   // Water: wading
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

// One stride through tall grass (~0.4 s, a one-shot played with each step): the shin brushing the stems.
std::vector<float> grassSwish(uint32_t sampleRate = 48000, uint32_t seed = 1);

// Ambience LOOPS, seamless by construction (circular filtering, integer-cycle envelopes), for a
// looping voice whose volume the caller drives:
// Water lapping at a lake's edge: small irregular wavelets arriving every 1.5-3 s, ~12 s loop (lakes are still;
// this is the quiet sound of their shore, not surf).
std::vector<float> lap(uint32_t sampleRate = 48000, uint32_t seed = 1);
std::vector<float> surf(uint32_t sampleRate = 48000, uint32_t seed = 1);          // waves on a beach, two swells, ~11 s
// Running water heard from the bank, ~4 s loop: a dark, soft, steady noise. `size` 0..1 narrow .. broad.
std::vector<float> river(uint32_t sampleRate = 48000, uint32_t seed = 1, double size = 0.5);
// Something COOKING (a pot simmering to a boil, `boil` 0..1): layered low bubbling, loop. Found tuning the
// river -- Glenn: "It sounds like something cooking ... mark the sounds for that". Not used in a level yet.
std::vector<float> simmer(uint32_t sampleRate = 48000, uint32_t seed = 1, double boil = 0.5);
// A SPLASH (#43): `strength` 0..1 from a swimming stroke to jumping in -- a slap on the surface, a spray of
// droplets falling back, and a gulp of air pulled under. One-shot.
std::vector<float> splash(double strength, uint32_t sampleRate = 48000, uint32_t seed = 1);
// Heard from UNDER the water (river or sea): deep bubbling, muffled -- nothing above ~900 Hz. Glenn: "works good
// for underwater".
std::vector<float> underwaterRiver(uint32_t sampleRate = 48000, uint32_t seed = 1);

// ---- ELEVATORS (Glenn, 2026-09-30: "a ding when it reaches the floor, noise when it opens, clicking when
// selecting a floor and then a noise showing that it's moving") ----------------------------------------------
// The CAR-ARRIVAL chime: a struck bell -- a bright partial over the fundamental, a slow decay. `up` rings the
// rising two-note signal (an up car), else the falling one. ~1.6 s, one-shot.
std::vector<float> elevatorDing(bool up, uint32_t sampleRate = 48000, uint32_t seed = 1);
// A floor-button press: a short plastic tick with a little spring, ~0.06 s.
std::vector<float> elevatorButton(uint32_t sampleRate = 48000, uint32_t seed = 1);
// The doors sliding: an operator motor's whir rising and falling over a rolling rumble, the leaves meeting with
// a soft thunk at the end. ~1.3 s (the door travel), one-shot; played for opening and closing alike.
std::vector<float> elevatorDoors(uint32_t sampleRate = 48000, uint32_t seed = 1);
// The car MOVING, heard inside: a low traction-motor hum, a rush of shaft air, a faint rail whisper. A seamless
// ~2 s loop; the caller fades it with the cab's speed.
std::vector<float> elevatorHum(uint32_t sampleRate = 48000, uint32_t seed = 1);

// TRAINS (Glenn, 2026-10-11: "there's no sound on the train. No train track clacking. No whistle or bell when it stops
// or when the doors open").
// One wheelset over a rail joint, ~0.2 s: a thump in the rail bed, the steel's ring, a scuffed burst. Played per axle
// as each crosses a joint, so a car makes the "ta-dum ... ta-dum" of its two bogies.
std::vector<float> railClack(uint32_t sampleRate = 48000, uint32_t seed = 1);
// The roll under it, a seamless loop (~1.5 s): low wheel-on-rail roar and a faint steel hiss. Gain and pitch by speed.
std::vector<float> railRumble(uint32_t sampleRate = 48000, uint32_t seed = 1);
// A locomotive's two-tone horn, high then low, ~1.7 s: sounded coming into a station.
std::vector<float> trainHorn(uint32_t sampleRate = 48000, uint32_t seed = 1);
// The doors: opening, a falling two-note chime (~1.3 s); closing, three short warning beeps (~0.9 s).
std::vector<float> trainDoorChime(bool opening, uint32_t sampleRate = 48000, uint32_t seed = 1);

// THE BODY'S MOVES (Glenn, 2026-10-24: "different sfx would help sell these types of motions too. Like skateboarding,
// parachuting, grapple, etc. The bike sounds motorized and it shouldn't. The drifting needs some tire squeal").
// sfx_motion.cpp. LOOPS (seamless; the caller drives gain and pitch):
std::vector<float> windRush(uint32_t sampleRate = 48000, uint32_t seed = 1);      // air past the ears, ~4 s: by airspeed
std::vector<float> clothFlutter(double flapsHz, uint32_t sampleRate = 48000, uint32_t seed = 1);   // canopy ~9, wingsuit ~24
std::vector<float> reelWinch(uint32_t sampleRate = 48000, uint32_t seed = 1);     // the grapple reeling: ratchet + motor, 1 s
std::vector<float> boardRoll(uint32_t sampleRate = 48000, uint32_t seed = 1);     // skateboard wheels on concrete, 2 s
std::vector<float> freewheel(uint32_t sampleRate = 48000, uint32_t seed = 1);     // a coasting bicycle's pawls, 40 clicks/s at 1.0
std::vector<float> tyreRoll(uint32_t sampleRate = 48000, uint32_t seed = 1);      // a pneumatic tyre's hum, 2 s
std::vector<float> tyreSqueal(uint32_t sampleRate = 48000, uint32_t seed = 1);    // rubber sliding: a drift, a lock-up, 2 s
// ONE-SHOTS:
std::vector<float> canopyOpen(uint32_t sampleRate = 48000, uint32_t seed = 1);    // rustle, WHUMP, lines snapping taut, ~1.5 s
std::vector<float> hookFire(uint32_t sampleRate = 48000, uint32_t seed = 1);      // the launcher's thunk + the line zipping out
std::vector<float> hookBite(uint32_t sampleRate = 48000, uint32_t seed = 1);      // the hook's clank as it bites
std::vector<float> ropeTwang(uint32_t sampleRate = 48000, uint32_t seed = 1);     // a slack rope snapping taut
std::vector<float> boardClack(uint32_t sampleRate = 48000, uint32_t seed = 1);    // one truck over a pavement joint
std::vector<float> ollie(uint32_t sampleRate = 48000, uint32_t seed = 1);         // the tail's pop + the grip-tape drag
std::vector<float> boardLand(uint32_t sampleRate = 48000, uint32_t seed = 1);     // both trucks back down, a rattle

// The firing rate the clip is baked at: pitch 1.0 sounds like this many
// combustion events per second (~1700 rpm on a four-cylinder four-stroke, two
// firings per revolution). Callers multiply it; the mapping core owns the band.
constexpr double kEngineRefHz = 56.0;

}  // namespace sfx
}  // namespace engine

#endif
