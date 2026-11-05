#ifndef RAYTRACER_ENGINE_PROCGEN_REGRADE_H
#define RAYTRACER_ENGINE_PROCGEN_REGRADE_H

// THE REGRADE (ADR-0323): a town's patch of mountain levelled enough to carry its streets and lots, the mountain's
// shape kept. Glenn, 2026-11-04: "a way to terraform the mountain area. Not to make it entirely flat but to level it
// enough to place buildings and roads. And somewhat retain the mountain shape."
//
// Over the outline the ground is
//   1. blurred over `blur` metres: the crags and gullies go, the shoulder and the fall of the hill stay;
//   2. capped at `maxSlope`: no steeper than that anywhere, the cut (the highest such ground under it) and the fill
//      (the lowest over it) averaged, so the earth taken off the knolls about fills the hollows;
//   3. stepped into shelves `terrace` metres apart, each easing up its last `bank` of the rise to the next (0: none);
// and eased back to the raw ground over `feather` metres outside it. It is GROUND, not earthwork: it is read in the
// terrain block's "regrade" with the weathered relief (erodedForTerrain), so the rivers, the planner, the road solve's
// natural ground, the lots and the drawn tiles all stand on it. The streets' and pads' own cut and fill come after.

#include "city/polygon.h"   // Vec2
#include "terrain_field.h"  // HeightField
#include <nlohmann/json.hpp>
#include <vector>

namespace engine {

struct Regrade {
    std::vector<Vec2> outline;   // the ground levelled (x, z)
    double blur = 100.0;         // m: relief finer than this is smoothed away
    double maxSlope = 0.12;      // rise per metre, at most
    double terrace = 0.0;        // m between shelves; 0 = a smooth slope
    double bank = 0.35;          // fraction of a shelf's rise that is bank
    double feather = 100.0;      // m outside the outline back to the raw ground
    double cell = 4.0;           // m: the regraded ground's grid
};

// The terrain block's "regrade": [ { "outline": [[x, z], ...], "blur", "maxSlope", "terrace", "bank", "feather",
// "cell" }, ... ]. Empty without one.
std::vector<Regrade> regradesFromJson(const nlohmann::json& terrainBlock);

// `base` with each regrade laid over it in turn. The levelled patches are computed here, once.
HeightField regradeField(HeightField base, const std::vector<Regrade>& regrades);

}  // namespace engine

#endif
