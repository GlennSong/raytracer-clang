#include "places.h"

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

bool Place::openAt(Real clock) const {
    if (openHour == closeHour) return true;          // degenerate → always open
    if (openHour < closeHour)                        // ordinary daytime window
        return clock >= openHour && clock < closeHour;
    return clock >= openHour || clock < closeHour;   // window wraps midnight
}

Vec2 snapToSidewalk(const engine::NavGraph& graph, Vec2 site,
                    int& outLink, Real& outT, Real verge) {
    outLink = -1;
    outT = 0;
    if (graph.linkCount() == 0) return site;

    // Scan every link, project the site onto its centreline segment, and keep
    // the sidewalk point closest to the building. Scanning (rather than trusting
    // NavGraph::nearestLink alone) is what puts the door on the FRONTAGE side: a
    // two-way road is two opposite-direction links whose sidewalks sit on
    // opposite kerbs, so the nearer sidewalk is the one the building faces.
    Real bestDist2 = -1;
    for (int li = 0; li < graph.linkCount(); ++li) {
        const engine::NavLink& link = graph.links[li];
        const Vec2 a = graph.nodes[link.from];
        const Vec2 b = graph.nodes[link.to];
        const Vec2 ab = b - a;
        const Real len2 = ab.lengthSquared();
        Real t = 0;
        if (len2 > 1e-9) {
            t = dot(site - a, ab) / len2;
            if (t < 0) t = 0;
            else if (t > 1) t = 1;
        }
        const Vec2 door = graph.sidewalkPoint(li, t, verge);
        const Real d2 = (door - site).lengthSquared();
        if (bestDist2 < 0 || d2 < bestDist2) {
            bestDist2 = d2;
            outLink = li;
            outT = t;
        }
    }
    return graph.sidewalkPoint(outLink, outT, verge);
}

PlaceId PlaceMap::add(PlaceType type, Vec2 site, const engine::NavGraph& graph,
                      Real openHour, Real closeHour, int capacity,
                      std::string name) {
    Place p;
    p.id = static_cast<PlaceId>(places_.size());
    p.type = type;
    p.site = site;
    p.entrance = snapToSidewalk(graph, site, p.entranceLink, p.entranceT);
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
