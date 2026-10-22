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

}  // namespace engine

#endif
