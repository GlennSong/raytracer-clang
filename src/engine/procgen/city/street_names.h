#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_STREET_NAMES_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_STREET_NAMES_H

// NAMING THE STREETS (Glenn, 2026-09-18: "we should name all of the streets. A
// street should have a start and end point ... I think that also helps with bus
// routes and agents addresses"; 2026-09-19: "It would be nice to have in world
// signs for streets"). The engine port of tools/city_street_names.py.
//
// A road graph has EDGES, not streets. What a person calls a street is a CHAIN
// of edges that carries straight on: you stay on Elm when Elm crosses Oak. So
// each street grows from an edge both ways -- through every plain (degree-2)
// node, since a bend is still the same street, and through a junction only
// onto the straightest continuation of the same width class, within a limit.
// Then each chain gets a name from a deterministic bank, suffixed by its class
// (a wide arterial is a Boulevard or Avenue, a narrow one a Lane or Court),
// unique across the city. Same graph + seed -> same names.
//
// Freeways and ramps are not streets here (they carry route numbers, not
// street names): their edges map to -1.

#include "road_network.h"

#include <nlohmann/json.hpp>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace engine {

// THE CONTENT, AS DATA (Glenn, 2026-09-20: "for the street names you've put
// them in code. I'm starting to wonder if some of this shouldn't be data and
// not baked into the C++"). assets/data/streets.json holds the words, what
// suffix a road of a given width carries, and how a sign shop shortens it;
// this file holds only the algorithm that uses them.
struct StreetNameBook {
    std::vector<std::vector<std::string>> banks;   // word pools, drawn from evenly
    struct SuffixRule {
        double minWidth = 0;
        std::vector<std::string> choices;
    };
    std::vector<SuffixRule> suffixes;                   // widest first
    std::map<std::string, std::string> classSuffix;     // "alley" -> "Alley"
    std::map<std::string, std::string> abbreviations;   // "Boulevard" -> "Blvd"
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

struct Street {
    std::string name;
    RoadClass klass = RoadClass::Local;
    double width = 0;                // mean carriageway width
    std::vector<int> edges;          // RoadGraph edge ids, end to end
    std::vector<int> nodes;          // the node chain (edges.size() + 1)
    double length = 0;
};

struct StreetNaming {
    std::vector<Street> streets;
    std::vector<int> streetOfEdge;   // edge id -> street index, -1 for none
    int streetOf(int edge) const {
        return edge >= 0 && edge < static_cast<int>(streetOfEdge.size()) ? streetOfEdge[edge] : -1;
    }
};

struct StreetNamingParams {
    // (The naming CONTENT lives in the book; these are the geometry rules.)
    // Carrying on THROUGH A JUNCTION: the straightest continuation within this
    // many degrees, of a width within widthTolerance. (Plain nodes always
    // continue: a curve is not a new street.)
    double junctionTurnDeg = 28.0;
    double widthTolerance = 2.5;   // a width change this big (or more) is a new street
    // Through a PLAIN node: a curve is still the street, a kink sharper than
    // 60 degrees at a single node is a corner between two.
    double cosPlainKink = 0.5;
    uint32_t seed = 0;   // 0 = the book's own seed
};

StreetNaming nameStreets(const RoadGraph& g, const StreetNameBook& book,
                         const StreetNamingParams& p = {});
// ...with the shipped book (streetNameBook()).
StreetNaming nameStreets(const RoadGraph& g, const StreetNamingParams& p = {});

// The sign-shop short form of a street's suffix ("Boulevard" -> "Blvd"), from
// the book's abbreviations. Unchanged when there is nothing to shorten.
std::string abbreviateStreetName(const std::string& name, const StreetNameBook& book);
std::string abbreviateStreetName(const std::string& name);

}  // namespace engine

#endif
