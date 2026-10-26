#ifndef RAYTRACER_ENGINE_PROCGEN_PERSON_H
#define RAYTRACER_ENGINE_PROCGEN_PERSON_H

#include "../../renderer/renderer.h"
#include "../../rt_math.h"

namespace engine {

// THE STANDING PERSON: the city's walkers' body (citysim buildPersonMesh, the walk cycle's poses), here in the engine
// so anything can stand one -- a scale reference (Glenn, 2026-10-18, of the scale lab's box figure: "why not use one of
// the agent models?"; Lua mesh.person) as much as a crowd. Six vertex-coloured boxes -- head, torso, two legs, two
// arms -- proportioned to a 1.8 m body, CENTRED at mid-height (y in [-0.9, 0.9]), facing +Z. `swing` pitches the
// limbs about the hip and shoulder (radians; legs opposed, arms counter-swinging), `outfit` picks the clothes.
struct PersonOutfit { Vec3 shirt, pants, skin; };
// The outfits: personOutfitTotal() of them, the last the PLAYER's (a red jacket no walker is dealt).
int personOutfitTotal();
const PersonOutfit& personOutfit(int outfit);   // (any int: wrapped round)
RenderMesh buildStandingPerson(Real swing, int outfit);

struct BodyPose;
// THE JOINTED PERSON (engine/block_body.h): the same body with knees and elbows, posed -- the torso and head as
// blocks, each limb one skinned tube bent at its fold. About the pose's position (give it the origin for a mesh
// centred like the one above).
RenderMesh buildPosedPerson(const BodyPose& pose, int outfit);
// THE STRIDE with knees and elbows, for a walk cycle's `swing` (buildStandingPerson's: radians, + the L leg back):
// legs opposed and arms counter-swinging as there, the trailing leg's knee folding as it pushes off, the arms'
// elbows soft and bending as they come forward.
BodyPose walkBodyPose(Real swing);

}  // namespace engine

#endif
