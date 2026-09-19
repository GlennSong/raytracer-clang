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

#include <cstdint>
#include <string>
#include <vector>

namespace engine {

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
    // Carrying on THROUGH A JUNCTION: the straightest continuation within this
    // many degrees, of a width within widthTolerance. (Plain nodes always
    // continue: a curve is not a new street.)
    double junctionTurnDeg = 28.0;
    double widthTolerance = 2.5;   // a width change this big (or more) is a new street
    // Through a PLAIN node: a curve is still the street, a kink sharper than
    // 60 degrees at a single node is a corner between two.
    double cosPlainKink = 0.5;
    uint32_t seed = 20260918u;
};

StreetNaming nameStreets(const RoadGraph& g, const StreetNamingParams& p = {});

// The sign-shop short form of a street's suffix ("Boulevard" -> "Blvd").
// Unchanged when there is nothing to shorten.
std::string abbreviateStreetName(const std::string& name);

}  // namespace engine

#endif
