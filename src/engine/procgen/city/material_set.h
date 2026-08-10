#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_MATERIAL_SET_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_MATERIAL_SET_H

#include "facade_plan.h"
#include "shape_grammar.h"   // PartId, FacadeStyle — the shared vocabulary
#include <cstdint>
#include <string>
#include <vector>

namespace engine {

// What a building is MADE OF (docs/lot-system-plan.md §7, §17.7).
//
// The brief: "buildings should be made out of sets of materials." A MaterialSet
// is a COHERENT BUNDLE drawn as a unit — wall, trim, roof, base and accent
// together with the window style that belongs to that palette — rather than
// five independent dice rolls. That coherence is the whole point: today's
// complaint is "same-y but random" from both directions at once, which is what
// independent per-part randomness produces.
//
// Sets are picked per DISTRICT and per LOT PROGRAM, then perturbed per
// building, so a street reads as one place with variation on it.

struct MaterialSet {
    std::string name;

    PartId wall = PartId::Brick;
    PartId trim = PartId::Trim;
    PartId roof = PartId::Shingle;
    PartId base = PartId::Concrete;
    PartId accent = PartId::Metal;

    Vec3 wallColor{0.72, 0.66, 0.60};
    Vec3 trimColor{0.92, 0.90, 0.86};
    Vec3 roofColor{0.24, 0.22, 0.22};
    Vec3 accentColor{0.45, 0.40, 0.36};

    FacadeStyle style = FacadeStyle::Brick;
    FenestrationStyle window;

    // ADR-0040's rule, carried as data rather than as a branch in dress():
    // cladding follows structure follows height. A set declares the band it is
    // credible in, and the picker never puts a timber cottage palette on a
    // forty-storey tower.
    int minStoreys = 1, maxStoreys = 8;
};

// Era is the axis that makes "Edwardian street" or "post-war estate" a data
// choice rather than another branch.
enum class Era : std::uint8_t { Victorian, Edwardian, Interwar, PostWar, Modern, Contemporary };

// The district character a palette belongs to. Kept deliberately coarse — the
// fine variation is the per-building perturbation, not more categories.
enum class Character : std::uint8_t {
    Suburban, OldTown, Terraced, Commercial, Downtown, Civic, Industrial, Upscale, RunDown,
};

const char* eraName(Era e);
const char* characterName(Character c);

// The palette library. Sets live here as data; adding "upscale area" or
// "run-down area" is appending rows, not another branch in dress().
struct PaletteLibrary {
    std::vector<MaterialSet> sets;
    // Parallel to `sets`: which character/era each belongs to.
    std::vector<Character> character;
    std::vector<Era> era;

    // Every set credible for this character and storey count. Era is a
    // preference rather than a filter, so a district is never left with
    // nothing to wear.
    std::vector<int> candidates(Character c, int storeys) const;

    // The street's FAMILY — the set this district wears. Exposed in its own
    // right because "what does this district look like" is a real question
    // (the editor asks it, and so does any test), and answering it by building
    // a house and seeing what it wore would be absurd.
    int districtFamily(Character c, Era preferred, int storeys,
                       std::uint32_t districtSeed) const;

    // Pick coherently: the DISTRICT seed chooses the street's palette family,
    // and the BUILDING seed only decides whether this building is one of the
    // minority that steps outside it. That split is what makes a street read as
    // one place — a per-building choice from the whole library is the "same-y
    // but random" failure, and a street with no outliers at all is the other.
    int pick(Character c, Era preferred, int storeys, std::uint32_t districtSeed,
             std::uint32_t buildingSeed) const;
};

PaletteLibrary stockPalettes();

// Perturb a set for one building: small hue/value shifts within the family, so
// neighbours differ without the street losing its identity. Never changes which
// PARTS are used — that is what keeps the bundle coherent.
MaterialSet perturb(const MaterialSet& base, std::uint32_t seed, Real amount = 1.0);

// A tier may re-clad legitimately (a glass tower on a stone podium). ADR-0040's
// bound: ONE family per mass, with a second treatment allowed at the BASE.
// Returns the base palette for a podium given the shaft's palette.
int basePaletteFor(const PaletteLibrary& lib, int shaftSet, Character c,
                   std::uint32_t seed);

// The material assignment for one building: a set per tier, plus the sided part
// mapping. Interiors stay out of scope; the side is recorded so they become a
// binding later rather than a rewrite.
struct BuildingMaterials {
    std::vector<MaterialSet> perTier;
    MaterialSet siteSet;              // fences, paths, planters

    const MaterialSet& forTier(int tier) const;
    // The part a given surface should use, honouring the (part, side) split.
    SidedPart partFor(int tier, PartId logical, Side side = Side::Exterior) const;
};

BuildingMaterials assignMaterials(const PaletteLibrary& lib, Character c, Era era,
                                  const std::vector<int>& tierStoreys,
                                  std::uint32_t districtSeed,
                                  std::uint32_t buildingSeed);

}  // namespace engine

#endif
