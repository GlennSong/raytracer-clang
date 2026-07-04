#ifndef RAYTRACER_APPS_CITYSIM_PLACES_H
#define RAYTRACER_APPS_CITYSIM_PLACES_H

#include "../../engine/ai/nav_graph.h"   // engine::NavGraph, Vec2, Real
#include <cstdint>
#include <string>
#include <vector>

namespace citysim {

using engine::Real;
using engine::Vec2;

// The PLACES layer (Living City, ADR-0066 Phase 1). Buildings become
// destinations agents can reason about: each Place has a TYPE (what it is for),
// an ENTRANCE snapped onto the pedestrian network (so a resolved place is
// already routable), and light metadata (open hours, capacity). The PlaceMap
// indexes places by type so an agent can ask "a shop near me" and get back a
// concrete, walkable target.
//
// Pure data + pure queries — no ECS, no rendering, no Lua, no sim state — so it
// builds and is unit-tested headless like the rest of the living-world
// substrate. The generator (phase 6) will emit a PlaceMap; the vertical-slice
// level (phase 4) authors one by hand. Everything here is deterministic.

// Stable place UID. Handed out densely (0..n-1) in insertion order, so it also
// serves as the index into PlaceMap::places(). Reproducible from the build
// order (ADR-0002).
using PlaceId = uint32_t;
constexpr PlaceId kNoPlace = ~PlaceId(0);

// What a building is FOR. Kept deliberately small in v1 (⟨A4⟩): the handful of
// roles the vertical slice needs. `Count` sizes the per-type index; keep last.
enum class PlaceType : uint8_t {
    Home,     // where an agent sleeps / departs from
    Shop,     // retail — a shopkeeper holds it; customers come and go
    Office,   // workplace for commuters
    Park,     // open green space — a stroll destination
    Civic,    // town hall / services (catch-all public building)
    Count
};

// Stable lowercase name of a place type ("home", "shop", …); "?" if out of
// range. Used by level authoring / debug, and as the JSON tag in the slice.
const char* placeTypeName(PlaceType type);

// Parse a place-type tag ("home", "shop", …). Returns false (leaving `out`
// untouched) for an unrecognized tag, so a level loader can report it.
bool parsePlaceType(const std::string& tag, PlaceType& out);

// One place in the world. `site` is the building footprint centroid; `entrance`
// is the door, snapped to the nearest walkable point so routing to the place is
// routing to a real graph position.
struct Place {
    PlaceId id = kNoPlace;
    PlaceType type = PlaceType::Home;
    Vec2 site;             // building footprint centroid (world XZ)
    Vec2 entrance;         // door on the pedestrian network (sidewalk point)
    int entranceLink = -1; // NavGraph link the entrance sits on (-1 = unsnapped)
    Real entranceT = 0;    // parameter along that link, [0,1]
    // Light metadata (surface-level ⟨A6⟩). Hours are in-world 0..24; the default
    // window [0,24) is "always open". `capacity` 0 means unlimited.
    Real openHour = 0;
    Real closeHour = 24;
    int capacity = 0;
    std::string name;      // optional authored label ("Al's Diner"); may be empty

    // Is this place open at in-world hour `clock` (0..24)? Handles a window that
    // wraps midnight (openHour > closeHour, e.g. a 20→4 late shop).
    bool openAt(Real clock) const;
};

// Indexes places by type and answers nearest-of-type queries. Build it by
// add()-ing each building; the entrance is snapped against the NavGraph at add
// time so a place is routable the moment it exists.
class PlaceMap {
public:
    // Register a building as a place, snapping its entrance to the nearest
    // walkable point of `graph` (the sidewalk in front of the nearest road).
    // Returns the assigned PlaceId (== its index in places()). If the graph is
    // empty the entrance falls back to the site and entranceLink stays -1.
    PlaceId add(PlaceType type, Vec2 site, const engine::NavGraph& graph,
                Real openHour = 0, Real closeHour = 24, int capacity = 0,
                std::string name = {});

    int size() const { return static_cast<int>(places_.size()); }
    bool empty() const { return places_.empty(); }
    const std::vector<Place>& places() const { return places_; }

    // Access by UID. Out-of-range asserts in debug; callers hold valid ids.
    const Place& operator[](PlaceId id) const { return places_[id]; }

    // All place ids of a given type, in insertion order. Empty if none.
    const std::vector<PlaceId>& ofType(PlaceType type) const;

    // Count of places of a type — a cheap "does the city have any shops?" check.
    int countOfType(PlaceType type) const {
        return static_cast<int>(ofType(type).size());
    }

    // Nearest place of `type` to world point `from`, by straight-line distance
    // to the ENTRANCE (what a walker actually routes to). kNoPlace if there are
    // none of that type. Deterministic: ties break to the lower PlaceId.
    PlaceId nearest(PlaceType type, Vec2 from) const;

private:
    std::vector<Place> places_;
    // places_ indices grouped by type; byType_[t] is the id list for type t.
    std::vector<PlaceId> byType_[static_cast<int>(PlaceType::Count)];
};

// Snap a world point to the nearest walkable point of `graph`: finds the nearest
// link, projects onto its centreline for the parameter t, and returns the
// sidewalk point there (just outside the kerb). Fills `outLink`/`outT`. Returns
// the input point unchanged with outLink=-1 when the graph has no links.
// Exposed so the level loader / generator can snap doors with the same logic the
// tests pin. `verge` matches NavGraph::sidewalkPoint's kerb offset.
Vec2 snapToSidewalk(const engine::NavGraph& graph, Vec2 site,
                    int& outLink, Real& outT, Real verge = 1.0);

}  // namespace citysim

#endif
