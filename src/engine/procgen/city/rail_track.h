#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_RAIL_TRACK_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_RAIL_TRACK_H

// RAIL TRACKS, as the road builder hands them on (Glenn, 2026-10-10: "it should define rail lines and we should
// probably build them like we build lanes of the freeway"). A track class's lanes are built by the lanes builder
// like any road's -- profile, deck, bridges, terrain conform -- and each one leaves as a RailTrack: its railhead
// line in the direction trains run on it, and the platforms beside it. Whatever runs trains reads these and
// nothing else, so a train stands on exactly the rails that were drawn.

#include "polygon.h"            // Vec2
#include "../../../rt_math.h"   // Vec3
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace engine {

// Track geometry over the formation (the deck top): sleepers sit on it, rails on them. A carriage floor and a
// platform top are measured from the railhead.
constexpr double kSleeperTop = 0.18;          // deck -> sleeper top
constexpr double kRailHeadAbove = 0.35;       // deck -> railhead
constexpr double kRailGauge = 1.435;          // standard gauge, between the rails' inner faces
constexpr double kPlatformGap = 1.55;         // track centre -> platform edge (a 2.9 m body, 0.1 m step gap)

// A platform beside a track, in the track's own stations (metres along `pts`).
struct RailStop {
    std::string name;
    double s0 = 0, s1 = 0;
    int side = +1;            // which side of the train it is on, looking the way the train runs: +1 left, -1 right
    double top = 0;           // platform top, world height
    // THE WAY IN: where the station's footway leaves the street (plan x, z) -- what a bus line serves
    bool hasAccess = false;
    Vec2 access{0, 0};
};

struct RailTrack {
    std::string edge, cls;    // the graph edge it was built on, and its class
    int dir = +1;             // +1 along the edge's spine, -1 against it
    std::vector<Vec3> pts;    // railhead centreline (world x, height, z), in running order, ~2 m apart
    std::vector<RailStop> stops;
};

inline nlohmann::json railTracksToJson(const std::vector<RailTrack>& tracks) {
    nlohmann::json a = nlohmann::json::array();
    for (const RailTrack& t : tracks) {
        nlohmann::json pts = nlohmann::json::array();
        for (const Vec3& p : t.pts) pts.push_back({p.x, p.y, p.z});
        nlohmann::json stops = nlohmann::json::array();
        for (const RailStop& s : t.stops) {
            nlohmann::json j = {{"name", s.name}, {"s0", s.s0}, {"s1", s.s1}, {"side", s.side}, {"top", s.top}};
            if (s.hasAccess) j["access"] = {s.access.x, s.access.y};
            stops.push_back(j);
        }
        a.push_back({{"edge", t.edge}, {"cls", t.cls}, {"dir", t.dir}, {"pts", pts}, {"stops", stops}});
    }
    return a;
}

inline std::vector<RailTrack> railTracksFromJson(const nlohmann::json& a) {
    std::vector<RailTrack> out;
    if (!a.is_array()) return out;
    for (const nlohmann::json& j : a) {
        RailTrack t;
        t.edge = j.value("edge", std::string());
        t.cls = j.value("cls", std::string());
        t.dir = j.value("dir", 1);
        for (const nlohmann::json& p : j.value("pts", nlohmann::json::array()))
            t.pts.push_back(Vec3(p.at(0).get<Real>(), p.at(1).get<Real>(), p.at(2).get<Real>()));
        for (const nlohmann::json& s : j.value("stops", nlohmann::json::array())) {
            RailStop st{s.value("name", std::string()), s.value("s0", 0.0), s.value("s1", 0.0), s.value("side", 1), s.value("top", 0.0)};
            if (s.contains("access")) { st.hasAccess = true; st.access = Vec2(s["access"].at(0).get<double>(), s["access"].at(1).get<double>()); }
            t.stops.push_back(st);
        }
        out.push_back(std::move(t));
    }
    return out;
}

}  // namespace engine

#endif
