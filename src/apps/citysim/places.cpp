#include "places.h"
#include "../../hash.h"   // lowbias32
#include "../../engine/procgen/city/roads/road_model.h"   // walkJoinNear: where a door joins the walks

#include <cmath>
#include <cstring>

namespace citysim {

const char* placeTypeName(PlaceType type) {
    switch (type) {
        case PlaceType::Home:   return "home";
        case PlaceType::Shop:   return "shop";
        case PlaceType::Office: return "office";
        case PlaceType::Park:   return "park";
        case PlaceType::Civic:  return "civic";
        case PlaceType::Cafe:        return "cafe";
        case PlaceType::Restaurant:  return "restaurant";
        case PlaceType::Supermarket: return "supermarket";
        case PlaceType::Bar:         return "bar";
        case PlaceType::Club:        return "club";
        default:                return "?";
    }
}

bool parsePlaceType(const std::string& tag, PlaceType& out) {
    for (int t = 0; t < static_cast<int>(PlaceType::Count); ++t) {
        const auto pt = static_cast<PlaceType>(t);
        if (tag == placeTypeName(pt)) {
            out = pt;
            return true;
        }
    }
    return false;
}

RetailKind retailKindFor(Vec2 site) {
    // Hash the site on a 0.5 m lattice: stable across builds, and two lots
    // never share one.
    uint32_t h = static_cast<uint32_t>(static_cast<int32_t>(std::lround(site.x * 2.0))) * 73856093u ^
                 static_cast<uint32_t>(static_cast<int32_t>(std::lround(site.y * 2.0))) * 19349663u;
    h = engine::lowbias32(h);
    const uint32_t roll = h % 100u;
    RetailKind k;
    if (roll < 22)      { k.type = PlaceType::Cafe;        k.openHour = 6.5; k.closeHour = 19.0; }
    else if (roll < 40) { k.type = PlaceType::Restaurant;  k.openHour = 11.0; k.closeHour = 23.5; }
    else if (roll < 49) { k.type = PlaceType::Supermarket; k.openHour = 7.0; k.closeHour = 22.0; }
    else if (roll < 57) { k.type = PlaceType::Bar;         k.openHour = 16.0; k.closeHour = 2.0; }
    else if (roll < 60) { k.type = PlaceType::Club;        k.openHour = 21.0; k.closeHour = 3.0; }
    else                { k.type = PlaceType::Shop;        k.openHour = 9.0; k.closeHour = 20.0; }
    return k;
}

bool Place::openAt(Real clock) const {
    if (openHour == closeHour) return true;          // degenerate → always open
    if (openHour < closeHour)                        // ordinary daytime window
        return clock >= openHour && clock < closeHour;
    return clock >= openHour || clock < closeHour;   // window wraps midnight
}

Vec2 snapToSidewalk(const engine::RoadModel* roads, Vec2 site) {
    if (!roads || !roads->built) return site;
    const engine::WalkJoin j = roads->walkJoinNear(site);   // the one fallback by position (ADR-0468)
    return j.valid() ? j.point : site;
}

PlaceId PlaceMap::add(PlaceType type, Vec2 site, const engine::RoadModel* roads,
                      Real openHour, Real closeHour, int capacity,
                      std::string name, const Vec2* entranceHint) {
    Place p;
    p.id = static_cast<PlaceId>(places_.size());
    p.type = type;
    p.site = site;
    // ADR-0080: when the caller knows the real door, snap from IT -- the
    // entrance lands on the sidewalk in front of the door, not wherever is
    // nearest the footprint centroid (which can be around a corner).
    p.door = entranceHint ? *entranceHint : site;
    p.entrance = snapToSidewalk(roads, p.door);
    p.openHour = openHour;
    p.closeHour = closeHour;
    p.capacity = capacity;
    p.name = std::move(name);

    const int ti = static_cast<int>(type);
    if (ti >= 0 && ti < static_cast<int>(PlaceType::Count))
        byType_[ti].push_back(p.id);
    places_.push_back(std::move(p));
    return static_cast<PlaceId>(places_.size() - 1);
}

const std::vector<PlaceId>& PlaceMap::ofType(PlaceType type) const {
    const int ti = static_cast<int>(type);
    static const std::vector<PlaceId> kEmpty;
    if (ti < 0 || ti >= static_cast<int>(PlaceType::Count)) return kEmpty;
    return byType_[ti];
}

PlaceId PlaceMap::nearest(PlaceType type, Vec2 from) const {
    const std::vector<PlaceId>& ids = ofType(type);
    PlaceId best = kNoPlace;
    Real bestDist2 = -1;
    for (PlaceId id : ids) {
        const Real d2 = (places_[id].entrance - from).lengthSquared();
        // Strict `<` keeps the first (lower-id) place on a tie — deterministic.
        if (bestDist2 < 0 || d2 < bestDist2) {
            bestDist2 = d2;
            best = id;
        }
    }
    return best;
}

}  // namespace citysim
