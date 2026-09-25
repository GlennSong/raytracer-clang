#ifndef RAYTRACER_ENGINE_PROCGEN_WORLD_ROAD_SIGNS_H
#define RAYTRACER_ENGINE_PROCGEN_WORLD_ROAD_SIGNS_H

// ROAD SIGNS FOR A WORLD'S ROADS (Glenn, 2026-09-25: "freeway signs. Like how far each town is ... we
// can name the towns and show which direction they are. Also! We need signs for which side of the
// freeway we should enter to go in the right direction.").
//
// Two stages, both pure data, both offline:
//   1. THE PLAN (planIslandSigns): where every sign stands, which way the traffic reading it travels,
//      and what it says -- from the freeway, its interchanges and the places they serve:
//        on the freeway, per exit and direction: advance signs 2 km and 1 km out, the exit direction
//        sign overhead where the exit lane opens, a gore sign at the split; after each on-ramp a
//        route marker and a distance sign (the next three places, road distance);
//        on the road at each interchange: an entrance sign before each on-ramp (route, direction,
//        the city that way, an arrow), DO NOT ENTER and WRONG WAY at each off-ramp's end;
//        town-limit signs where a road enters a place; route trailblazers on the pass and the
//        mountain road.
//      A ring freeway has no north or south: its directions are the INNER LOOP (clockwise) and the
//      OUTER LOOP, each signed with the next city that way (the Capital Beltway's convention).
//   2. THE FACE (layoutSign): the panel in metres -- size, colours, rows of text measured with the
//      sign font (text/font.h, the same metrics the texture baker uses), shields, arrows. One layout,
//      drawn two ways: the SVG sign sheet now, the baked texture atlas when the world is built in 3D.
//
// Handedness: plan coordinates are the world mirrored (roads/lanes/interchange.h) -- traffic keeps
// right in the plan and LEFT in the world. Every left/right a driver sees (an arrow, the side of an
// exit tab) is the plan's, mirrored.

#include "island_world.h"

#include <string>
#include <vector>

namespace engine {

// A piece of a sign face, in metres from its top-left corner.
struct SignElem {
    std::string kind;          // "rect", "text", "shield", "arrow", "disc"
    double x = 0, y = 0, w = 0, h = 0;
    std::string text;          // text; a shield's number
    double cap = 0;            // text: cap height (m); y is its baseline
    std::string color = "#ffffff";
    std::string anchor = "start";   // text: start, middle, end
    double angle = 0;          // arrow: degrees clockwise from straight up (-45: up and left)
    double radius = 0;         // rect: corner radius
};

struct SignFace {
    double w = 1, h = 1;
    std::string background = "#00693f";   // the US guide-sign green
    std::vector<SignElem> elems;
};

// Fill w.signs (the island's interchanges, roads, places and limits must be in place).
void planIslandSigns(IslandWorld& w);

// The face of a sign.
SignFace layoutSign(const IslandSign& s);

// Every sign face on one sheet, grouped by interchange, each captioned with where it stands.
bool writeSignSheetSvg(const IslandWorld& w, const std::string& svgPath);

// The face as SVG elements (for the sheet, and for a map that wants to show one).
std::string signFaceSvg(const SignFace& f, double x, double y, double scale);

}  // namespace engine

#endif
