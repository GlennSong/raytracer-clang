#ifndef RAYTRACER_CITYSIM_HOUSEHOLDS_H
#define RAYTRACER_CITYSIM_HOUSEHOLDS_H

// WHO LIVES WHERE: ages, life stages and households (#172; ~/.claude/plans/kinds-are-data-assign-places.md).
//
// Glenn, 2026-11-01: "a family would be expressed by like a unit"; "single people would have their own apartment ...
// and single people could have roommates as well"; time frozen (nobody ages, nobody is born). So a person has an AGE
// in a BAND (child, young, adult, senior), a LIFE STAGE drawn from the band (working, at home, retired, a child, a
// student in a hall) and a HOUSEHOLD: a single, a couple, roommates, a single parent, a family, three generations,
// seniors -- one household to a dwelling (place_kinds.h), each type fitting dwellings of so many bedrooms.
//
// planHouseholds fills the city's dwellings with households, in an order of the seed's, until the people run out --
// so the POPULATION IS CAPPED by the housing (Glenn: "Do a cap for now"), and what could not be housed is reported,
// not invented a home. The hall beds (a place tagged beds) are filled first, one student each.
//
// Authored in assets/scripts/people.lua (scripting/people_lua.h reads it); defaultPeopleCatalog is the same catalog
// in C++ (a test keeps the two describing identically).

#include "places.h"

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace citysim {

// What a person's days are for. `job`: they look for work (their role is the job's, roles.lua); else `role` names the
// role they hold. `walks`: they never drive (a child, a student in a hall); `beds`: they live in a hall's beds, not a
// dwelling.
struct LifeStage {
    std::string name;
    bool job = false;
    bool walks = false;
    bool beds = false;
    std::string role;
};

// An age band: its years and the life stages people of those ages lead, with shares (stage index, share).
struct AgeBand {
    std::string name;
    int lo = 0, hi = 0;
    std::vector<std::pair<int, double>> stages;
};

// Some members of a household, all of one band (drawn once for the group: a couple is two adults, not an adult and
// a senior): lo..hi of them, or one per bedroom (`perBed`, roommates).
struct MemberGroup {
    int lo = 1, hi = 1;
    bool perBed = false;
    std::vector<std::pair<int, double>> bands;   // (band index, share)
};

// HOW TWO PEOPLE OF A HOUSEHOLD ARE TIED, as each sees it: a tie and the one its other side holds (parent / child;
// partner / partner). A tie that is its own other side may tie a group to itself (a couple, the roommates).
struct Tie {
    std::string name;
    int other = 0;   // the tie the other side holds (an index into PeopleCatalog::ties)
    int older = 0;   // years this side is at least older than the other (a parent 18); the planner keeps it
};
// Group `from`'s members are the `tie` of group `to`'s (each pair once; within one group, every pair).
struct HouseholdTie {
    int from = 0, to = 0, tie = 0;
};

// A kind of household: its share of the households, the dwellings it fits (bedrooms bedsLo..bedsHi), who it is and
// how they are tied.
struct HouseholdType {
    std::string name;
    double share = 0;
    int bedsLo = 0, bedsHi = 0;
    std::vector<MemberGroup> groups;
    std::vector<HouseholdTie> ties;
};

struct PeopleCatalog {
    std::vector<LifeStage> stages;
    std::vector<AgeBand> bands;
    std::vector<HouseholdType> households;
    std::vector<Tie> ties;
    // Who fills a hall's beds: people of this band and stage (a stage with `beds`), at most this share of the walkers.
    int hallBand = -1, hallStage = -1;
    double hallShare = 0.25;
    // LODGERS: anyone past the housing (a level's authored count) is one of these bands, on their own. No band of
    // theirs may lead a walk-only stage (a lodger may be a driver).
    std::vector<std::pair<int, double>> lodgers;

    int stage(const std::string& name) const;
    int band(const std::string& name) const;
    int household(const std::string& name) const;
    int tie(const std::string& name) const;
    // What is wrong with it ("" when nothing): a share-less band or type, a dwelling size no type fits, a hall rule
    // that names no beds stage, a stage with no job and no role, a lodger band with a walk-only stage, a tie whose
    // other side does not answer it, a household tie naming a group it has not got.
    std::string check() const;
    std::string describe() const;
};

PeopleCatalog defaultPeopleCatalog();

// One person of the plan.
struct Resident {
    uint8_t band = 0, stage = 0, age = 0;
    uint8_t group = 0;   // which of its household type's member groups (its ties)
    bool walks = false;
};
// One household: its type (kHallHousehold: a student in a hall bed), its home, the dwelling it has (an index into
// PlaceMap::dwellings(); kNoDwelling for a hall bed) and its members, people[first, first + count).
struct Household {
    static constexpr uint16_t kHall = 0xFFFF;
    static constexpr uint32_t kNoDwelling = 0xFFFFFFFFu;
    uint16_t type = 0;
    PlaceId place = kNoPlace;
    uint32_t dwelling = kNoDwelling;
    uint32_t first = 0, count = 0;
};
struct HouseholdPlan {
    std::vector<Household> households;
    std::vector<Resident> people;
    int asked = 0;               // the people it was asked to house
    int walkersOnly = 0;         // of people: those who must be walkers
    int dwellings = 0, dwellingsFilled = 0, hallBeds = 0, hallFilled = 0;
    // Everyone the city's housing could hold, had it been asked for more: the people of a household drawn for every
    // dwelling and every hall bed (the hall share aside).
    int capacity = 0;
};

// Fill `places`' dwellings and hall beds (places tagged `bedsTag`) with households, until `people` are housed, no
// more than `walkers` of whom may be walk-only (children, students). Deterministic in `seed`.
HouseholdPlan planHouseholds(const PeopleCatalog& cat, const PlaceMap& places, uint32_t bedsTag, int people, int walkers,
                             uint32_t seed);

// A lodger (PeopleCatalog::lodgers) from `key`'s bits: their band, age and stage.
Resident drawLodger(const PeopleCatalog& cat, uint32_t key);

// One line: households by type, people by band and stage, dwellings filled, and the people the housing turned away.
std::string describePlan(const PeopleCatalog& cat, const HouseholdPlan& plan);

}  // namespace citysim

#endif
