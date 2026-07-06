#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ARCHITECT_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ARCHITECT_H

#include "polygon.h"        // Vec2
#include "shape_grammar.h"  // BuildingParams (the recipe the architect fills)
#include <cstdint>
#include <string>

namespace engine {

// The ARCHITECT pass (building-grammar-plan.md P5): a deterministic planning
// stage between the city's districts and its lots. The city is divided into
// named DISTRICTS; each district owns an ARCHETYPE TABLE — the weighted set of
// building recipes that belong there — and every lot asks the architect what
// to build. Coherence lives here: the financial district's table simply
// contains no cottages, and an industrial shed never lands on a old-town lane.

enum class DistrictTag : uint8_t {
    Financial,     // the downtown core: office towers, glass, concrete slabs
    Commercial,    // the midtown ring: masonry mid-rises, shops, mixed use
    Residential,   // the outer ring: houses, duplexes, low walk-ups, parks
    OldTown,       // a dense pocket: narrow stucco, round arches, hip roofs
    Industrial,    // an edge wedge: metal sheds, solid facades, wide lots
};
const char* districtName(DistrictTag t);

// v1 district GEOGRAPHY: the proven radial rings (downtown core, midtown,
// outskirts) plus two seeded WEDGES — an industrial sector on the outer ring
// and an old-town pocket in midtown, both placed by angle from `seed` — so a
// city has recognizable quarters, not just gradients. A polygon-region map
// replaces this when districts become authorable.
struct DistrictMap {
    Vec2 center{0, 0};
    Real innerRadius = 55.0;    // < this: Financial
    Real midRadius = 135.0;     // < this: Commercial (or OldTown); else outer
    uint32_t seed = 1;
    DistrictTag tagAt(const Vec2& p) const;
};

// What the architect hands back for one lot: a fully-populated grammar recipe,
// the place type agents schedule around, and the massing choice.
struct BuildingRecipe {
    enum class Massing : uint8_t {
        LotPlan,     // the lot's own polygon is the floorplan (urban infill)
        RectYard,    // a small centred rectangle — a house with a YARD
        Circle,      // a chord-tessellated drum (roomy curtain-wall lots)
        Park,        // no building: a green with trees
    };
    Massing massing = Massing::LotPlan;
    BuildingParams params;      // style/windows/roof/floors, ready to grow
    std::string placeType;      // "home" | "shop" | "office" | "civic" | "park"
};

// Pick a recipe for a lot in `tag`'s district. `shortSide`/`area` describe the
// buildable footprint (the table filters recipes the lot can't carry — no
// tower on a cottage lot). Deterministic in `seed`.
BuildingRecipe architectPick(DistrictTag tag, Real shortSide, Real area,
                             uint32_t seed);

}  // namespace engine

#endif
