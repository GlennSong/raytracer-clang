#ifndef RAYTRACER_APPS_CITYSIM_STATION_ENTRANCE_H
#define RAYTRACER_APPS_CITYSIM_STATION_ENTRANCE_H

// THE STATION ENTRANCE (Glenn, 2026-10-13: "Okay do that" -- station buildings, on the list after the hotel lobby).
// A station's platforms stand up on the line, its footway climbing to them from a street; where the footway leaves
// the street (RailStop::access, the "way in") there was nothing to say a station was there. Now a PAVILION stands over
// the footway's first metres: a roof on columns with the station's name on it facing the street, glass side walls,
// ticket machines down one side, a kiosk and a bench down the other, and a line of ticket gates across the way up.
// Built at load from the rail lines (nothing baked), on the ground the city reports under each piece.

#include "../../renderer/renderer.h"
#include "rail_sim.h"

#include <functional>
#include <string>
#include <vector>

namespace citysim {

struct StationEntrance {
    std::string name, cls;     // the station, its line's class (rail, light_rail)
    engine::Vec3 foot;         // 2.5 m up the footway from the way in (where it leaves the street), on the ground
    engine::Vec3 along;        // horizontal unit: from the way in toward the platforms (the way up)
};

// One per station that has a way in (a station's two platforms share one).
std::vector<StationEntrance> stationEntrances(const RailSim& sim, const std::function<double(double, double)>& ground);

struct EntranceMeshes {
    engine::RenderMesh body;      // columns, roof, machines, gates, kiosk, bench: vertex coloured, world space
    engine::RenderMesh glass;     // the side walls' glazing, world space
    engine::RenderMesh collider;  // what a walker bumps into (no roof): world space
    engine::Mat4 board;           // the name board's frame on the roof (station_boards.h), facing the street
};

// The pavilion: 8.6 m across the footway, 7.5 m along it from the way in; each piece on the ground under it.
EntranceMeshes stationEntranceMeshes(const StationEntrance& e, const std::function<double(double, double)>& ground);

}  // namespace citysim

#endif
