#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_CITY_SVG_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_CITY_SVG_H

#include "road_network.h"
#include "polygon.h"
#include "street_furniture.h"
#include "../../ai/nav_graph.h"

#include <string>
#include <vector>

namespace engine {

// THE CITY MAP (device: "an svg of all the sidewalks and how they hug the
// roads ... along with all of the planted street objects ... layered so we
// can turn on and off elements (including buildings, lots, districts)").
// One writer, every element its own <g id="layer-…">: chosen at write time
// (CityMapLayers, RT_CITY_SVG_LAYERS) and switchable inside the file (the
// legend entries toggle their layer when the SVG is opened in a browser).
// 1 unit = 1 m; y = world Z, so it reads like the viewer's top-down view.
//
// Every layer is drawn from what the generators actually BUILT, never from
// a re-derivation: the curb loops are the mesher's own band outlines (its
// CurbBandAudit), the buildings are the collider prisms' plan polygons, the
// poles are the furniture plan, the trees are the spawned instance groups.
// Where two layers disagree, the disagreement is the finding.
struct CityMapData {
    RoadGraph roads;                       // the unified street graph
    struct Hub { Vec2 pos; int kind = 0; std::string name; };
    std::vector<Hub> hubs;                 // district hubs (polycentric zoning)
    double hubRadius = 220.0;
    NavGraph nav;                          // the planner's graph (lanes)
    StreetFurniturePlan furniture;         // signal poles + lamp posts
    // The mesher's curb/sidewalk band: closed loops around every asphalt
    // union (exterior outlines and block-interior holes), the band riding
    // each loop's right normal outward by sidewalkWidth; mouthGaps are where
    // the band is suppressed (a non-street mouth).
    std::vector<Poly2> curbLoops;
    std::vector<std::pair<Vec2, Vec2>> mouthGaps;
    double sidewalkWidth = 3.5;
    std::vector<Poly2> blocks, lots;
    struct Building { Poly2 plan; std::string district, type; };
    std::vector<Building> buildings;
    struct Place { Vec2 pos; std::string type, name; };
    std::vector<Place> places;
    // Everything else planted: trees/rocks (Scenery) and benches/signs/other
    // furniture instance groups, as positions.
    enum class ObjectKind { Scenery, Furniture };
    struct Object { Vec2 pos; ObjectKind kind = ObjectKind::Scenery; };
    std::vector<Object> objects;
};

struct CityMapLayers {
    bool roads = true, curbs = true, sidewalks = true, gaps = true, nav = true,
         furniture = true, objects = true, blocks = true, lots = true, buildings = true,
         districts = true, places = true, legend = true;
    // "roads,sidewalks,furniture" → only those (unknown names are ignored,
    // reported by the writer's log line); "all" or "" → everything.
    static CityMapLayers fromList(const std::string& csv);
    std::string toList() const;
    static const char* const* names(int* count);
};

// The furniture-centric preset the old RT_FURNITURE_SVG drew: streets, the
// nav graph, poles and lamps.
CityMapLayers furnitureMapLayers();

// The sidewalk band as a drawable centreline: each curb loop offset outward
// by half the sidewalk width (stroke it sidewalkWidth wide). Exposed for
// tests: the band must sit OUTSIDE the asphalt everywhere.
std::vector<Poly2> sidewalkBandCentrelines(const std::vector<Poly2>& curbLoops,
                                           double sidewalkWidth);

bool writeCityMapSvg(const std::string& path, const CityMapData& data,
                     const CityMapLayers& layers);

}  // namespace engine

#endif
