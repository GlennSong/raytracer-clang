#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ENVELOPE_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ENVELOPE_H

// THE ENVELOPE (architectural toolkit, issue #105, step 6; ADR-0178): the skin of a mass when it is NOT the
// floors. A form (`BuildingParams::envelope`: a twist, a bulge, the curves, a wave, a mesh) can be the floors --
// every storey its slice, as the stack and the field and the mesh give them -- or it can be a SKIN round straight
// floors: the storeys the plan's prism (straight glass, rectangular rooms, the planners untroubled), and the form
// drawn outside them as a second glass skin, or as the floor plates running out to its contour, or both. Glenn,
// 2026-10-15: "the interior of those offices should just be a straight edge versus the exterior which can be
// wavy" -- the Wave did it first (its plates); this is that rule as a word for every form.
//
//   skin 0  the form IS the floors (every storey its slice)
//   skin 1  DOUBLE SKIN: straight floors `skinGap` inside the form, the form a glass veil round them
//   skin 2  PLATES: straight floors, each floor's plate running out to the form's contour (Aqua Tower)
//   skin 3  both: the plates, and the veil round them
//
// The floors inside a skin are the plan scaled about its centre until, pushed out by the gap, they lie inside the
// form at every storey above the base; the Wave's floors are the plan itself (its wave runs past the plan). Floors
// narrower than the core needs (kCoreTierMin) cannot stand, and the form is the floors as if `skin` were 0. The
// skin draws from the base storeys' roof to the top, ruled storey to storey; it is scenery (no collider): the
// floors' own glass bounds the walkable space, and a plate's edge past it carries a railing.

#include "mass.h"
#include "mass_stack.h"
#include "shape_grammar.h"
#include <vector>

namespace engine {

constexpr uint8_t kSkinDouble = 1, kSkinPlates = 2;

// The skin mode in force: `params.skin`, with the Wave's default of plates (its storeys were always the plan).
uint8_t skinOf(const BuildingParams& params);
// THE FLOORS INSIDE THE SKIN: empty when no skin is in force or the floors would be too narrow for the core.
Poly2 skinFloorsPlan(const Poly2& plan, const BuildingParams& params);
// THE FORM'S OWN STOREYS, skin ignored: what the form would be as floors (the stack's, the field's, the mesh's).
std::vector<StoreyPlan> formStoreys(const Poly2& plan, const BuildingParams& params);
// THE SKIN's outline per storey (index = storey, as storeyPlans gives them): empty below the first storey above the
// base and when no veil is drawn; the form's storey outlines, resampled to one corner count when they differ
// (resampleLoop), so the glass rules from each storey to the next.
std::vector<Poly2> skinSlices(const Poly2& plan, const BuildingParams& params);
// THE PLATES per storey (index = storey): the Wave's field contour at the floor line (wavePlates), another form's
// storey outline; empty below the base and when the skin has no plates.
std::vector<Poly2> skinPlates(const Poly2& plan, const BuildingParams& params);
// The floors as a stack when a skin is in force and the floors are not the plan itself: the plan up to the base
// storeys, the inner plan nested above. False (and `out` untouched) when today's path applies.
bool skinFloorsStack(const Poly2& plan, const BuildingParams& params, Stack& out);

}  // namespace engine

#endif
