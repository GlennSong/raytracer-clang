#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ARCHITECT_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ARCHITECT_H

#include "polygon.h"        // Vec2
#include "shape_grammar.h"  // BuildingParams (the recipe the architect fills)
#include <array>
#include <cstdint>
#include <string>
#include <vector>

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
    // POLYCENTRIC geography (the metropolis tier): secondary centres with a
    // dominant flavor. Non-empty = tagAt zones by the NEAREST hub — the centre
    // hub keeps the radial-ring reading (Financial core, Commercial ring),
    // a flavored hub grades from its flavor out to Residential. Each entry is
    // {position, kind} with kind mirroring DistrictTag's order
    // (0 financial, 1 commercial, 2 residential, 3 oldtown, 4 industrial).
    std::vector<std::pair<Vec2, int>> hubs;
    Real hubRadius = 220.0;     // a hub's flavor reach before Residential wins
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
        RowStrip,    // the lot packed with side-by-side ROWHOUSE units
        BoxMass,     // force the shrink-fit BOX scope + growBuilding, so the
                     // non-box BuildingShapes (pagoda, cylinder) can dispatch
        Plaza,       // no storeys: a raised paver podium with stairs, fencing,
                     // fountain and planting ("a building without the building")
        PodiumTower, // a full-lot PODIUM of a few floors carrying a slender
                     // TOWER above it — the modern downtown block (density
                     // round: "more varied building shapes")
    };
    Massing massing = Massing::LotPlan;
    BuildingParams params;      // style/windows/roof/floors, ready to grow
    std::string placeType;      // "home" | "shop" | "office" | "civic" | "park"
    std::string name;           // the RECIPE that made it ("school", "hotel",
                                // "glass_tower", ...) — debug + tests
    // RectYard massing caps (a house keeps a small pad; a school a big one).
    Real yardHalfWMax = 7.0;
    Real yardHalfDMax = 6.0;
    // PodiumTower massing: how many of params.floors belong to the full-lot
    // podium (the rest rise in the tower). 0 elsewhere.
    int podiumFloors = 0;
};

// LANDMARK archetypes: placed by the PLANNER (best-lot picks with quotas —
// one courthouse per city, one school per residential quarter), never rolled
// per lot. A city has civic anchors, not a 6% chance of one per parcel.
enum class LandmarkKind : uint8_t {
    School,        // 2-3 fl masonry, classroom window grid, a real schoolyard
    Hospital,      // near-white concrete slab, wings, roof plant
    Courthouse,    // the government hall: pilasters, tall ground, quoins
    Police,        // squat concrete, dark trim
    Fire,          // brick, VEHICLE BAY doors on the street face
    Market,        // one tall arched masonry hall (shop anchor)
    Capitol,       // THE town hall: portico + steps + dome rotunda, dead centre
    University,    // the campus hall: brick, portico, a real green (RectYard)
    Church,        // one tall gabled nave with a BELL TOWER steeple
    Library,       // sandstone civic reading hall: pilasters, arched windows
    Museum,        // the gallery: formal portico + steps on a plaza (RectYard)
    Count
};
const char* landmarkName(LandmarkKind k);
BuildingRecipe architectLandmark(LandmarkKind kind, Real shortSide, Real area,
                                 uint32_t seed);

// The LANDMARK QUOTA PLAN (8km-city P3, per-hub-cluster): civic anchors are
// placed on the best-scoring lots, never rolled — and the quotas are now
// evaluated PER HUB CLUSTER. Cluster 0 (the primary city) keeps the full
// city-wide table (one capitol, courthouse, hospital, a school per
// residential quarter, ...); every other cluster is a satellite TOWN
// guaranteed at least its own school and church (and, for an old-town
// settlement, a market hall) once it has enough candidate lots — a town is a
// settlement with its own anchors, not a suburb of the city's. Purely
// best-score (no rng), so the plan is deterministic in the candidates' order.
struct LandmarkCand {
    DistrictTag tag = DistrictTag::Residential;  // the lot's district
    Real shortSide = 0;    // lot OBB short side (m)
    Real area = 0;         // lot polygon area (m²)
    Vec2 pos{0, 0};        // lot centroid (world XZ)
    int cluster = 0;       // hub-cluster id (0 = the city; 1+ = towns)
};
// One entry per candidate: the assigned LandmarkKind as int, -1 = none.
// `cityCenter`/`innerRadius` feed the centrality scoring of the wantCore
// picks (capitol, courthouse, museum), exactly as the city-only planner did.
std::vector<int> planLandmarks(const std::vector<LandmarkCand>& cands,
                               const Vec2& cityCenter, Real innerRadius);

// One ROWHOUSE unit's dressed params for the RowStrip massing: the lot pass
// splits the strip and calls this per unit, so neighbours vary in cladding
// and colour while sharing the strip's floor count and eave line.
BuildingParams architectRowUnit(uint32_t seed, int floors);

// ---- THE ARCHETYPE BOOK (city-lots-v2 item 3): the SELECTION layer as data.
//
// The architect has three layers. Zoning (tagAt) and the 56 recipe BODIES stay
// C++ — the bodies are not data (they branch on coreness/roomy and feed the
// floor cap back through RecipeCtx::slender; porting them is what the
// abandoned Lua branch attempted and failed). The layer worth authoring is
// SELECTION: each district's weighted table over named recipes, today written
// as if/else ladders inside architectPick. A book replaces those weights.
//
// Recipes are named by the SAME strings the style book keys on (out.name:
// "glass_tower", "brick_shop", ...), so the two authoring surfaces share one
// vocabulary. A district absent from the book keeps its compiled ladder.
// The book is resolved to registry indices ONCE at load (makeArchetypeBook,
// procgen_bindings.h) — no per-lot Lua — and the pick consumes the same
// single rng roll in the same order, so a book carrying the ladder's own
// weights reproduces today's cities EXACTLY.
struct ArchetypeBook {
    // Per DistrictTag (same order): ascending {cumulative threshold, recipe
    // registry index}, last threshold 1.0. Empty vector = compiled ladder.
    std::array<std::vector<std::pair<Real, int>>, 5> tables;
    bool empty() const {
        for (const auto& t : tables)
            if (!t.empty()) return false;
        return true;
    }
};

// The recipe REGISTRY: every recipe the architect can produce, rolled and
// landmark alike, by its out.name string. Index is stable within a build.
// Unknown name = -1 — the caller must HARD-ERROR, never skip (the
// courtMinArea false-knob rule, parcel.h).
int architectRecipeIndex(const std::string& name);
int architectRecipeCount();
const char* architectRecipeName(int index);   // nullptr out of range

// Pick a recipe for a lot in `tag`'s district. `shortSide`/`area` describe the
// buildable footprint (the table filters recipes the lot can't carry — no
// tower on a cottage lot). `coreness` is how deep into the city centre the
// lot sits (0 = at the district rim, 1 = dead centre): the financial table
// uses it to peak the skyline — a SKYSCRAPER cluster downtown, shoulders
// around it. Deterministic in `seed`. A non-null `book` with a non-empty
// table for `tag` replaces the district's compiled weight ladder.
BuildingRecipe architectPick(DistrictTag tag, Real shortSide, Real area,
                             uint32_t seed, Real coreness = 0,
                             const ArchetypeBook* book = nullptr);

}  // namespace engine

#endif
