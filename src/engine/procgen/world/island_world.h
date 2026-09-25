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
#include <memory>
#include <string>
#include <vector>

namespace engine {

struct IslandSite {
    std::string kind;          // "city", "town", "mountain town"
    Vec2 at;
    double radius = 0.0;       // the flat ground it has (m): a city's footprint, a town's
    double elevation = 0.0;
    double flatArea = 0.0;     // m^2 of buildable ground in its patch
    bool coastal = false;
};

struct IslandRoad {
    std::string kind;          // "freeway", "pass", "mountain", "link"
    std::vector<Vec2> points;
    double length = 0.0, climb = 0.0, worstGrade = 0.0;
    int from = -1, to = -1;    // site indices
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
    nlohmann::json report;
    double heightAt(double x, double z) const;   // bilinear
};

// A variant's terrain block from a seed: the island's shape, bearing, peninsula and cliffs, and
// the range, all drawn from the seed within sensible bounds.
nlohmann::json islandTerrainBlock(uint32_t seed, double half = 10000.0);

IslandWorld planIsland(const nlohmann::json& terrainBlock, double half = 10000.0, double cell = 20.0);

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
void linkCityToFreeway(IslandWorld& w, int site, const std::vector<Vec2>& arterialNodes, int maxLinks, double spacing, double maxLength = 700.0);

// The map: shaded relief and water, buildable land tinted, sites and roads drawn. `px` a side.
// A view may zoom to a window (centre, half-width; 0 = the whole island) and lay extra lines over
// it (a planned city's streets), each drawn `widthM` wide but never thinner than `minPx`.
struct IslandMapLayer {
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

}  // namespace engine

#endif
