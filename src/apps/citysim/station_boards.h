#ifndef RAYTRACER_APPS_CITYSIM_STATION_BOARDS_H
#define RAYTRACER_APPS_CITYSIM_STATION_BOARDS_H

// THE STATION'S NAME ON ITS PLATFORMS (Glenn, 2026-10-11: "Train stations could use the name of the station so you
// know what station it is"). A running-in board at each end of every platform: white lettering on a blue board on two
// posts near the platform's back edge, facing the track -- read from a train pulling in and from the platform.
// Lettered at load (the lanes bundle has no textures), from the RailSim's lines: each line's stop is one platform.

#include "../../engine/text/font.h"
#include "../../renderer/renderer.h"
#include "rail_sim.h"

#include <string>
#include <vector>

namespace citysim {

constexpr double kBoardW = 4.2, kBoardH = 0.7;    // the lettered face, metres
constexpr double kBoardBottom = 2.0;              // above the platform top
constexpr double kBoardBack = 3.6;                // in from the platform's edge (4.5 m wide: clear of the edge, the stair
                                                  // runs along the back near the bridge, the boards stand at the ends)
constexpr double kBoardFromEnd = 16.0;            // along from each end of the platform

struct StationBoard {
    std::string name;
    engine::Mat4 m;   // the board's frame: +x along the platform, +y up, +z toward the track; origin at its foot
};

// Two per platform, one in from each end.
std::vector<StationBoard> stationBoards(const RailSim& sim);

// The posts and the board's edge (vertex coloured), and its two lettered faces (u 0..1 across, v 0 at the top; the
// back face mirrored so it reads from behind too). Both in the board's frame.
engine::RenderMesh stationBoardFrame();
engine::RenderMesh stationBoardFaces();

// The lettering for `name`: centred, as large as the board allows, white on the board's blue.
void stationBoardImage(const engine::Font& font, const std::string& name, engine::TextImage& out);

}  // namespace citysim

#endif
