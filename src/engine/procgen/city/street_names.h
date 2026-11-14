#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_STREET_NAMES_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_STREET_NAMES_H

// NAMING THE STREETS (Glenn, 2026-09-18: "we should name all of the streets. A
// street should have a start and end point ... I think that also helps with bus
// routes and agents addresses"; 2026-09-19: "It would be nice to have in world
// signs for streets").
//
// THE PLANNER'S NAME GENERATOR (ADR-0420). What a person calls a street -- you stay
// on Elm when Elm crosses Oak -- is a RUN of the road model's streets that are one
// street carrying on, as the road file states it (a junction's `through` pairs, a
// street's `carries_on`, a divided road's two carriageways: road_layout.h
// streetRuns). This file only gives each run a name: from a deterministic bank,
// suffixed by the run's width (a wide road is a Boulevard or Avenue, a narrow one a
// Lane or Court) or its class, unique across the city. The planner writes the names
// into the road file; a level reads them from there (the road model), the same on
// every load. Nothing here looks at a road's shape.

#include <nlohmann/json.hpp>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace engine {

// THE CONTENT, AS DATA (Glenn, 2026-09-20: "for the street names you've put
// them in code. I'm starting to wonder if some of this shouldn't be data and
// not baked into the C++"). assets/data/streets.json holds the words, what
// suffix a road of a given width carries, which road classes carry no name, and
// how a sign shop shortens a name; this file holds only the algorithm.
struct StreetNameBook {
    std::vector<std::vector<std::string>> banks;   // word pools, drawn from evenly
    struct SuffixRule {
        double minWidth = 0;
        std::vector<std::string> choices;
    };
    std::vector<SuffixRule> suffixes;                   // widest first
    std::map<std::string, std::string> classSuffix;     // "alley" -> "Alley"
    std::map<std::string, std::string> abbreviations;   // "Boulevard" -> "Blvd"
    std::set<std::string> unnamed;                      // road classes that carry no street name ("freeway", "ramp")
    uint32_t seed = 20260918;
    bool fromAsset = false;                             // false = the built-in fallback
    bool usable() const { return !banks.empty() && !suffixes.empty(); }
};

// Parse the asset's "names" object. An empty/!usable book means the caller
// should fall back.
StreetNameBook parseStreetNameBook(const nlohmann::json& names);
// assets/data/streets.json, read once. Falls back to a small built-in book
// (with a warning) so a stripped install still names its streets.
const StreetNameBook& streetNameBook();

// WHAT A RUN TO BE NAMED IS, to the generator: how long (the longest runs draw
// first, so the main roads take from the full bank), how wide its carriageway is
// (the suffix rule), and its road class (a class suffix, when the book has one).
struct RunToName {
    double length = 0;
    double width = 0;
    std::string roadClass;
};

// A NAME FOR EACH RUN, in the runs' order: deterministic in (runs, book, seed);
// unique, and no word used twice while the banks last. `taken`: names already in
// use (kept from a road file), never given again and their words not reused.
// seed 0: the book's own.
std::vector<std::string> nameRuns(const std::vector<RunToName>& runs, const StreetNameBook& book, uint32_t seed = 0,
                                  const std::set<std::string>& taken = {});

// The sign-shop short form of a street's suffix ("Boulevard" -> "Blvd"), from
// the book's abbreviations. Unchanged when there is nothing to shorten.
std::string abbreviateStreetName(const std::string& name, const StreetNameBook& book);
std::string abbreviateStreetName(const std::string& name);

}  // namespace engine

#endif
