#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_CITY_PRODUCER_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_CITY_PRODUCER_H

// The procedural city as the first bundle producer (ADR-0084). For every shape:"lanelab" entity of a
// level it runs the lane lab and writes what the loader instantiates — the material meshes split by
// render cell, the conformed ground, the road twin (nav graph, freeway right-of-way) and the un-inset
// pavement holes the lot pass turns into blocks — under `city/e<ordinal>/...`. The loader reads exactly
// these sections back (readCityProducts) and instantiates from them whether they came from disk or from a
// build a moment ago, so a cold level and a cached level are the same geometry to the byte.
//
// Identity: the bytes of each entity's graph file (or its inline graph) and of the terrain grid file the
// graph references, the render cell size, sizeof(Real), kCityFormatVersion and kLanesBuildTag — the tag
// is bumped by hand whenever the lab's output changes (that is the rule; the key cannot see code).

#include "engine/bundle/bake.h"
#include "engine/bundle/codecs.h"
#include "engine/procgen/city/roads/lanes/lanes.h"
#include "engine/procgen/city/roads/lanes/road_graph_spec.h"

#include <string>
#include <vector>

namespace engine {
namespace roads::lanes {

constexpr int kCityFormatVersion = 1;
constexpr const char* kCityProducerName = "city";
extern const char* const kLanesBuildTag;

void registerCityProducer();   // idempotent

struct CityEntity {
    int ordinal = 0;        // n-th lanelab entity of the level: the section namespace
    int entityIndex = 0;    // position in the level's entities array
    nlohmann::json block;   // the entity's "lanelab" block
    std::string graphPath;  // empty when the graph is inline ("edges" in the block)
    bool inlineGraph = false;
};
std::vector<CityEntity> cityEntities(const nlohmann::json& level);
double cityRenderCell(const nlohmann::json& level);   // citysim.renderCell, default 250 m
std::string citySectionPrefix(int ordinal);          // "city/e<ordinal>/"
RoadLabGraph loadCityGraph(const CityEntity& e);      // the loader's rule: inline "edges" or the "graph" path

struct CityCellMesh { int cx = 0, cz = 0; bundle::PackedMesh mesh; };
struct CityProducts {
    bool hasTerrain = false;
    bundle::HeightGridBlob ground;
    RoadEntity twin;                 // class-faithful roadTwin()
    RoadGraph nav;                   // navRoadGraph(twin, ground): the level's unified road graph
    RoadGraph row;                   // twin nodes + Freeway/Ramp edges: the lot pass's keep-out
    std::vector<Ring> holes;         // un-inset pavement holes >= 2000 m² (blocks after the level's sidewalk inset)
    double cell = 250;
    std::vector<CityCellMesh> cells; // one per (render cell, material), material name inside
    nlohmann::json report;           // summary, invariants, timings, counts
};

// `withInvariants`: also run the lab's invariant sweep into the report (minutes on metro; the loader never
// needs it — rt_bake --check and the tests ask for it).
CityProducts cityProductsFromResult(const Result& r, double renderCell, bool withInvariants = false);
void writeCityProducts(bundle::BundleWriter& w, int ordinal, const CityProducts& p);
bool readCityProducts(const bundle::Bundle& b, int ordinal, CityProducts& p, std::string* err);

}  // namespace roads::lanes
}  // namespace engine

#endif
