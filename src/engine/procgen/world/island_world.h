#ifndef RAYTRACER_ENGINE_PROCGEN_WORLD_ISLAND_WORLD_H
#define RAYTRACER_ENGINE_PROCGEN_WORLD_ISLAND_WORLD_H

// AN ISLAND WORLD, PLANNED ON THE MAP FIRST (ADR-0105). Glenn, 2026-09-25: "an island with a mountain
// range in the middle ... two cities on either side of the mountains and then some small towns
// scattered throughout and all of it is connected by a large ring freeway system ... a beach,
// mountains, rivers, deltas, inlets ... a peninsula ... cliffs where you can drive up a windy
// mountain road and get to a small town up there."
//
// The terrain comes first; the cities go where the land lets them:
//   1. the TERRAIN: an island (TerrainParams::Island) with a range along its long axis, and its rivers
//      from the drainage (ADR-0099);
//   2. BUILDABLE LAND: above the beach, gentle, dry;
//   3. SITES: a distance transform finds where the flat land is deepest -- a CITY on the widest plain
//      each side of the range, TOWNS on other good ground spread around, one of them the highest
//      (the mountain town);
//   4. ROADS, grade-limited over the ground (terrain_route.h): a RING freeway through the coastal
//      sites in order round the island, a PASS road over the range between the cities, a winding
//      MOUNTAIN road up to the high town.
// Then a MAP (and a report) to judge it by -- before anything is built in 3D.

#include "../city/polygon.h"   // Vec2
#include "../hydrology.h"

#include <nlohmann/json.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace engine {

struct IslandSite {
    std::string kind;          // "city", "town", "mountain town"
    std::string name;          // place_names.h: what the signs call it
    Vec2 at;
    double radius = 0.0;       // the flat ground it has (m): a city's footprint, a town's
    double elevation = 0.0;
    double flatArea = 0.0;     // m^2 of buildable ground in its patch
    bool coastal = false;
    std::vector<std::vector<Vec2>> limits;   // its planned limits (routeFreewayRoundCities), closed
    int population = 0;                      // estimated from its planned buildings (the town-limit sign)
};

// An INTERCHANGE on the island freeway: a diamond where a road crosses it (islandInterchanges).
struct IslandInterchange {
    int road = -1;             // the crossing road (w.roads index)
    int site = -1;             // the place it serves (-1: none, a country road)
    double station = 0.0;      // along w.freewayRoute
    Vec2 at;                   // where the road crosses
    std::string exit;          // the exit number: km along the Inner Loop from the zero point ("12", "12A")
    struct Ramp {
        bool off = false;
        bool withRoute = false;   // on the carriageway that runs with increasing station
        double gore = 0.0;        // its gore's station
        Vec2 gorePt, terminal;    // where it leaves/joins the freeway, and lands on the road
        std::vector<Vec2> path;
    };
    std::vector<Ramp> ramps;
};

// A BUS LINE (Glenn: "bus routes in local regions. Maybe a bus to go between towns?"): a local loop
// in one place, or an intercity line between places' centres over the island's roads.
struct IslandBusLine {
    std::string name;          // "Saltwood 2", "X1 Island Ring"
    std::string kind;          // "local", "intercity"
    int site = -1;             // a local line's place
    std::vector<Vec2> path;    // as driven
    std::vector<Vec2> stops;
    std::vector<std::string> stopNames;
};

// A SIGN (road_signs.h): where it stands, which way the traffic reading it travels, and what it says.
struct IslandSign {
    std::string kind;          // advance, exit, gore, distance, route, entrance, do-not-enter, wrong-way, limit, trailblazer
    Vec2 at;                   // its post (or a gantry's middle)
    Vec2 facing;               // the direction of travel of the traffic that reads it (unit)
    std::string mount;         // "roadside", "overhead"
    nlohmann::json legend;     // what it says (road_signs.h lays the face out from this)
};

struct IslandRoad {
    std::string kind;          // "freeway", "pass", "mountain", "link"
    std::vector<Vec2> points;
    double length = 0.0, climb = 0.0, worstGrade = 0.0;
    int from = -1, to = -1;    // site indices
    std::string street;        // a link: the city street it lands on (its exit is signed with it)
};

struct IslandWorld {
    nlohmann::json terrain;    // the level terrain block (island, range, rivers)
    double half = 10000.0, cell = 20.0;
    int n = 0;                 // samples a side
    std::vector<float> height; // n x n, row-major from (-half, -half)
    std::vector<uint8_t> buildable;
    std::shared_ptr<const Hydrology> hydro;
    std::vector<IslandSite> sites;
    std::vector<IslandRoad> roads;
    Vec2 axis{1, 0};           // the island's long axis (the range runs along it)
    std::vector<std::vector<Vec2>> ramps;   // the interchanges' ramps (islandInterchanges), centrelines
    std::vector<nlohmann::json> rampEdges;  // ...and as the lanes-scene edges diamondRamps wrote (anchors on fw0_a / fw0_b)
    std::vector<Vec2> freewayRoute;         // the freeway as the one route the diamonds were laid on
    bool routeClockwise = false;            // increasing station runs clockwise (seen on the map, north up)
    std::vector<IslandInterchange> interchanges;   // in station order
    std::vector<IslandSign> signs;
    std::vector<IslandBusLine> busLines;
    std::vector<Vec2> centres;              // per site: its centre stop (the transit centre), set by the caller
    std::vector<std::vector<Vec2>> trails;  // hiking trails (planTrails): trailheads to summits and lakes, town to town
    std::vector<std::string> trailNames;    // what each leads to
    nlohmann::json report;
    double heightAt(double x, double z) const;   // bilinear
    // Where a road may not go on the ground: the sea, or a lake and 12 m round it (Saltwood's pass ran
    // its last stretch along lake 2's bed, 3.5 m under the water -- "you drove into the lake")
    bool water(double x, double z) const;
    // the ground a MOUNTAIN road is routed and graded on: the height blurred over ~60 m, the way the road
    // will meet it once the builder has cut the spurs and filled the gullies. On the raw eroded ground no
    // route under 12% existed; on it, switchbacks do (ADR-0112)
    std::vector<float> heightSmooth;
    double smoothAt(double x, double z) const;
};

// A variant's terrain block from a seed: the island's shape, bearing, peninsula and cliffs, and
// the range, all drawn from the seed within sensible bounds.
nlohmann::json islandTerrainBlock(uint32_t seed, double half = 10000.0);

IslandWorld planIsland(const nlohmann::json& terrainBlock, double half = 10000.0, double cell = 20.0);

// HIKING TRAILS (ADR-0134): footpaths routed on the ground (gentle where they can, switchbacking where
// they must, never over water) from each place's TRAILHEAD -- the edge of its limits facing the range --
// to its nearest summits and mountain lakes, and between neighbouring places along the coast (a
// long-distance path). Needs the places' limits (routeFreewayRoundCities). Fills w.trails.
void planTrails(IslandWorld& w);

// The pass's (and mountain road's) ends run on into their towns: each end, trimmed at its town's limits,
// joins the nearest of that town's arterial nodes (ADR-0135). Call after routeFreewayRoundCities.
void joinPassToTowns(IslandWorld& w, const std::vector<std::pair<int, std::vector<Vec2>>>& arterialNodes);

// A CITY BRIEF FOR A SITE (ADR-0106): the city planner's brief (city_plan.h, as JSON) scaled to the
// site's flat ground -- a city gets a core grid, midtown, outskirts and a freeway ring; a town a small
// grid and outskirts, no ring -- its grid squared to the coast (one axis runs down to the sea), and
// its world the island's own terrain block with `land` set, so the plan is cut back to the ground it
// may stand on and a ring that meets the sea opens into a C.
nlohmann::json islandSiteBrief(const IslandWorld& w, int site);

// Route the island's roads that end at `site` onto that city's own ring: each is cut back to where it
// first comes within the ring's corridor and joined to the ring there (a system interchange, when it
// is built) -- so the island freeway runs round the city's inland side on the ring, never through it.
void joinFreewayToRing(IslandWorld& w, int site, const std::vector<Vec2>& ring, bool closed);

// THE FREEWAY ROUND THE CITIES, NOT THROUGH THEM (ADR-0106). Once each site is planned, its limits
// (closed outlines, CityPlan::limits) are no-go for the freeway: it is routed again from site to site,
// each waypoint moved just outside its city on the inland side, so it runs along the foot of the hills
// behind the towns -- the region's road, which the cities' arterials meet. The pass and the mountain
// roads stop at the limits they reach, and the mountain road climbs from the new freeway.
void routeFreewayRoundCities(IslandWorld& w, const std::vector<std::pair<int, std::vector<std::vector<Vec2>>>>& cityLimits);

// LINK ROADS from a city's arterials to the freeway (an interchange at each): the arterial ends nearest
// the freeway, up to `maxLinks`, their freeway ends at least `spacing` apart, none longer than maxLength.
// `streets` names each node's street (parallel to arterialNodes; empty: unnamed).
void linkCityToFreeway(IslandWorld& w, int site, const std::vector<Vec2>& arterialNodes, int maxLinks, double spacing, double maxLength = 700.0,
                       const std::vector<std::string>& streets = {});

// THE INTERCHANGES: every road that meets the freeway (a city's links, the pass, the mountain road)
// CROSSES it -- carried on past, to a T on the first street it meets if there is one within 500 m, else
// just far enough for the far ramps to land -- and at each crossing the one
// diamond generator (roads/lanes/interchange.h, as the level importer and the city planner use it)
// lays the ramps: off from each carriageway before the crossing, on after it, landing on the road
// either side. A crossing it refuses (too oblique, too close to the last, no room) is reported, not
// forced. Fills w.ramps and w.report["interchanges"].
// `cityStreets`: the places' streets as segments; a road carried on past the freeway ends in a T on the
// first one it meets.
void islandInterchanges(IslandWorld& w, const std::vector<std::pair<Vec2, Vec2>>& cityStreets = {});

// INTERCITY BUSES over the island's roads, between the places' centres (w.centres): the RING line
// calls at every place round the freeway (in and out of each on its link road and streets); the PASS
// line runs city to city over the range by Route 2; a SHUTTLE climbs Route 3 to the mountain town from
// the place nearest its foot. `streets(site, from, to)` is a street path inside a place (the caller
// has its graph); the freeway, the links, the pass and the mountain road are the island's.
void planIntercityBuses(IslandWorld& w, const std::function<std::vector<Vec2>(int site, const Vec2& from, const Vec2& to)>& streets);

// The map: shaded relief and water, buildable land tinted, sites and roads drawn. `px` a side.
// A view may zoom to a window (centre, half-width; 0 = the whole island) and lay extra lines over
// it (a planned city's streets), each drawn `widthM` wide but never thinner than `minPx`.
struct IslandMapLayer {
    std::string name;                      // the SVG group's id ("streets-local", ...)
    std::vector<std::vector<Vec2>> lines;
    float rgb[3] = {0.1f, 0.1f, 0.1f};
    double widthM = 10.0, minPx = 1.0;
    bool casing = false;                   // a dark edge round each line
};
struct IslandMapView {
    Vec2 centre{0, 0};
    double half = 0.0;
    bool sites = true;                     // the site circles
    std::vector<IslandMapLayer> layers;    // drawn after the rivers, before the island roads
};
bool writeIslandMap(const IslandWorld& w, const std::string& pngPath, int px = 1400, const IslandMapView& view = {});

// The same map as SVG (Glenn: "We should favor that over raster for maps"), in metres, north up, one
// file to zoom into instead of a close-up per site. Everything planned is VECTOR -- streets at their
// widths, the freeway and its ramps, the pass, links, rivers at their widths, city limits, labels --
// and so is the terrain's line work: contours every 20 m (100 m heavier) and the coastline. The
// shaded relief under it is one embedded image at the terrain's own resolution (a 20 m grid: vectors
// would add nothing). Each layer is a group (<g id=...>), to toggle in Inkscape or a browser.
bool writeIslandSvg(const IslandWorld& w, const std::string& svgPath, const IslandMapView& view = {}, int reliefPx = 2000);

}  // namespace engine

#endif
