#ifndef RAYTRACER_APPS_CITYSIM_CITY_MESHES_H
#define RAYTRACER_APPS_CITYSIM_CITY_MESHES_H

#include "../../renderer/renderer.h"   // RenderMesh, RenderMaterial
#include "city_sim.h"                  // VehicleType, Agent::State, fleet table
#include "traffic_signal.h"            // SignalState

namespace citysim {

// Procedural meshes + materials for the citysim render bridge: everything here
// is pure construction (vertex-coloured boxes, flat widget geometry, material
// constants) with no ECS or sim-state dependency.

// A simple articulated PERSON (device ask: "people should look like people"):
// six vertex-coloured boxes — head, torso, two legs, two arms — centred like the
// old walker box (y in [-0.9, 0.9], facing +Z). `swing` pitches the limbs about
// hip/shoulder pivots (radians; legs opposed, arms counter-swing), so a small
// set of these meshes IS the walk cycle — walkers hop between shared pose
// meshes by walk phase, no skeleton needed. `outfit` picks a deterministic
// shirt/pants/skin combination.
engine::RenderMesh buildPersonMesh(engine::Real swing, int outfit);

// Outfits a WALKER may be dealt (deterministic per walker). The table holds one
// extra slot past this count — the player's fixed, distinctive outfit
// (playerOutfit()) — so walkers dealing `hash % personOutfitCount()` can never
// dress like the player.
int personOutfitCount();
int playerOutfit();

// --- the shared walk cycle ---------------------------------------------------
// ONE walk cycle for everyone who walks (city walkers AND the third-person
// player body): a handful of pre-built limb-swing pose meshes, hopped between
// as a body's stride advances with its REAL horizontal speed — the legs move
// exactly as fast as the ground goes by. Pose 0 is standing.
constexpr int kWalkPoses = 6;                    // poses per stride cycle
constexpr engine::Real kMaxSwing = 0.55;         // peak limb pitch (radians, ~32 deg)
constexpr engine::Real kStrideLength = 1.5;      // metres per full walk cycle
constexpr engine::Real kWalkAnimMinSpeed = 0.3;  // below this the body stands

// The limb swing baked into pose `pose` (sinusoidal over the cycle; pose 0
// stands) — feed it to buildPersonMesh.
engine::Real walkPoseSwing(int pose);

// Advance `stridePhase` (kept wrapped to [0,1)) by the distance walked this
// tick and return the pose index to draw. Below kWalkAnimMinSpeed the phase
// holds and the body stands (pose 0) — a held body never moonwalks.
int walkPoseIndex(engine::Real& stridePhase, engine::Real speed, engine::Real dt);

// A ground-PROJECTED ring of unit outer radius: a flat glowing band lying just
// above the pavement, like painted road markings — always visible from any
// camera the way lane paint is (regular depth, no reliance on an overlay depth
// state). Wide enough (20% of radius) to read at street-view distance.
// Instanced with scale (radius, 1, radius).
engine::RenderMesh ringXZ(engine::Real innerFrac = 0.80, int segs = 40);

// A flat arrow along +Z from the origin to z=1 (a shaft + a head), facing +Y —
// the debug trajectory vector. Instanced with scale (width, 1, length) + yaw.
engine::RenderMesh arrowXZ();

// A flat ground strip: a unit quad along +Z (x in [-0.5, 0.5], z in [0, 1]),
// facing +Y — the debug navgraph lane line. Instanced with scale
// (width, 1, length) + yaw.
engine::RenderMesh stripXZ();

// A ground-facing OUTLINE wedge of unit radius centred on +Z: the two straight
// edges from the origin plus the arc between them, built from thin quads (band
// ~4% of radius) like ringXZ's band so it draws with the same widgetMaterial
// technique — the debug vision-cone view. The half-angle is baked into the
// mesh; instanced with scale (range, 1, range) + yaw.
engine::RenderMesh wedgeXZ(engine::Real halfAngleRad, int segs = 12);

// Materials the citysim instance groups draw with (hue mostly carried in each
// mesh's vertex colours).
engine::RenderMaterial carMaterial();
engine::RenderMaterial pedMaterial();
engine::RenderMaterial signalMaterial(SignalState s);      // emissive lit lens
engine::RenderMaterial signalPostMaterial();               // pole/arm/head assembly
engine::RenderMaterial widgetMaterial(engine::Vec3 color); // debug ground paint
// An emissive CAR lamp lens (ADR-0065 follow-up): a dark lens body with strong
// self-illumination in `emission` so headlights (white), brake lights (red), and
// turn signals (amber) glow like the traffic-signal lenses. Same material recipe
// as signalMaterial (FLAG none, regular depth), so it renders on device exactly
// the way the signal lenses do.
engine::RenderMaterial lampMaterial(engine::Vec3 emission);

// Traffic-light semantics (user-requested): GREEN = going, RED = stopped,
// AMBER = braking/avoiding, violet = turning, teal = following a leader.
engine::Vec3 stateColor(Agent::State s);

}  // namespace citysim

#endif
