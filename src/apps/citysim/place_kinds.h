#ifndef RAYTRACER_CITYSIM_PLACE_KINDS_H
#define RAYTRACER_CITYSIM_PLACE_KINDS_H

// WHAT KIND OF PLACE, AND HOW MANY HOMES IT HOLDS (#172, kinds are data; ~/.claude/plans/kinds-are-data-assign-places.md).
//
// A place's TYPE (PlaceType) is its coarse category, the one menus and venues read. Its KIND is the finer name the
// rules speak -- "teaching", "library", "residence", "quad", "field", and one kind named after each type -- with TAGS
// the sim asks instead of naming a kind ("does it have beds?", "is it campus?"). A kind is picked by the recipe that
// grew the building; a recipe no kind lists takes the kind named after its type.
//
// A HOME holds DWELLINGS: one household each (Glenn, 2026-11-01: "apartments, condos, family houses actually just
// house all family together and single people would have their own apartment"). How a building's floor area splits
// into dwellings is keyed by its USE (building_uses.lua: "house", "apartments"...), the same answer its interior is
// planned from: a house's units are one dwelling each, sized by their area; a block of flats is cut into a mix.
//
// Authored in assets/scripts/places.lua (scripting/place_kinds_lua.h reads it); defaultPlaceKindCatalog is the same
// catalog in C++ (a test keeps the two describing identically).

#include "places.h"

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace citysim {

struct PlaceKind {
    std::string name;
    PlaceType type = PlaceType::Home;
    std::vector<std::string> recipes;    // the recipes that grow this kind
    std::vector<std::string> tagNames;
    uint32_t tags = 0;                   // tagNames as PlaceKindCatalog::tag bits (internTags)
    double m2PerBed = 0;                 // a hall's beds: one per this much floor (0: not a hall)
};

// One size of dwelling: its bedrooms (0 a studio), its floor area, and its share of a block of flats.
struct DwellingSize {
    int beds = 1;
    double m2 = 0;
    double share = 0;
};

// How a home's floor area becomes dwellings. perUnit: each grown unit is one dwelling (a house, each house of a
// row), its bedrooms the largest of the catalog's houseSizes its area reaches. Else the floor (less `usable`'s
// corridors, stairs and walls) is cut into `mix`, in its shares.
struct DwellingRule {
    bool perUnit = false;
    double usable = 0.75;
    std::vector<DwellingSize> mix;
};

struct PlaceKindCatalog {
    std::vector<PlaceKind> kinds;
    std::map<std::string, DwellingRule> dwellings;   // by use (building_uses.lua)
    std::vector<DwellingSize> houseSizes;            // ascending m2: a house's bedrooms by its floor area
    std::vector<std::string> tagNames;               // bit i = tagNames[i] (internTags)

    int find(const std::string& name) const;   // -1 when unknown
    // The kind of a place grown by `recipe` (or authored with no recipe) of type `type`.
    int kindFor(const std::string& recipe, PlaceType type) const;
    // The bit of a tag; 0 when no kind carries it (so it matches nothing).
    uint32_t tag(const std::string& name) const;
    const char* nameOf(int kind) const { return kind >= 0 && kind < static_cast<int>(kinds.size()) ? kinds[static_cast<std::size_t>(kind)].name.c_str() : "?"; }
    // Number every tag the kinds name, and set each kind's bits. Call after filling `kinds`.
    void internTags();
    // What is wrong with it ("" when nothing): a type with no kind of its name, a recipe two kinds claim, a mix with
    // no shares, more than 32 tags.
    std::string check() const;
    // Canonical text: two catalogs that describe identically make the same places.
    std::string describe() const;
    // The dwelling rule for a home of this use; the "apartments" rule when the use has none.
    const DwellingRule* dwellingRule(const std::string& use) const;
};

// The built-in catalog: places.lua, in C++.
PlaceKindCatalog defaultPlaceKindCatalog();

// The dwellings of a home with `floorArea` m2 over `units` grown units, by `rule`: each one's bedrooms, appended to
// `beds`. Never none: a home too small to cut is one dwelling of the smallest size.
void dwellingsOf(const PlaceKindCatalog& c, const DwellingRule& rule, double floorArea, int units, std::vector<int>& beds);

}  // namespace citysim

#endif
