#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_CITY_SVG_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_CITY_SVG_H

#include "road_network.h"
#include "road_mesh.h"   // RoadDeckField: the built asphalt the conflicts census measures against
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
    // Real 3-D doors (ADR-0080): the grammar's entrance apertures, from the
    // CityBuildings records. Drawn as a tick along the outward normal.
    struct Door { Vec2 foot; Vec2 normal; bool enterable = false; };
    std::vector<Door> doors;
    // Everything else planted: trees/rocks (Scenery) and benches/signs/other
    // furniture instance groups, as positions.
    enum class ObjectKind { Scenery, Furniture };
    struct Object { Vec2 pos; ObjectKind kind = ObjectKind::Scenery; };
    std::vector<Object> objects;
};

struct CityMapLayers {
    bool roads = true, curbs = true, sidewalks = true, gaps = true, nav = true,
         furniture = true, objects = true, blocks = true, lots = true, buildings = true,
         districts = true, places = true, doors = true, conflicts = true, legend = true;
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

// WHERE THE SIDEWALK CUTS ACROSS A ROAD (device: "can you find places where
// the sidewalk cuts across the road?"). The band centreline, sampled every
// metre, is measured against the BUILT asphalt — each road's RoadDeckField
// (the spines and junction pads its mesh rode; RoadDeckField::depthInside)
// — not the graph: the first cut measured graph edges and reported every
// cul-de-sac cap and every road whose graph edge ran on past its ribbon.
// A sample deeper than `tolerance` inside a deck is a hit; hits cluster
// into places within `mergeRadius`, deepest first. Drawn as the
// `conflicts` layer (red X + number) and logged as a census.
struct SidewalkCrossing {
    Vec2 pos;              // the deepest sample of the cluster
    int edge = -1;         // the nearest road edge (a label, from the graph)
    RoadClass klass = RoadClass::Local;
    double width = 0;      // that edge's carriageway width
    double depth = 0;      // metres inside the built asphalt
    int samples = 0;       // band samples in the cluster
    double spanMetres = 0; // extent of the cluster along the band
    // The nearest degree-1 graph node within 25 m (-1 = none): a stub whose
    // end lies inside another road's asphalt is a T the graph never joined.
    double deadEndDist = -1;
    // The BUILT chain the band lies on (the deck spine): its class and width,
    // and the nearest graph node within 15 m with its degree (a T the graph
    // knows has degree 3 there; a stub abutting another road does not).
    RoadClass deckClass = RoadClass::Local;
    double deckWidth = 0;
    int nodeDegree = -1;
    double nodeDist = -1;
    // COINCIDENT nodes: how many graph nodes sit within 2 m of the nearest
    // one, with their degrees — a T recorded as a bend plus a dead end (two
    // nodes where one belongs) is the shape the top metro places have.
    int coincident = 0;
    std::string coincidentDegrees;
};
std::vector<SidewalkCrossing> findSidewalkRoadCrossings(
    const CityMapData& data, const std::vector<const RoadDeckField*>& decks,
    double tolerance = 0.5, double mergeRadius = 15.0);

// `decks`: the RoadDeck fields of the level's roads (empty = the conflicts
// layer is drawn empty and says so).
bool writeCityMapSvg(const std::string& path, const CityMapData& data,
                     const CityMapLayers& layers,
                     const std::vector<const RoadDeckField*>& decks = {});

}  // namespace engine

#endif
