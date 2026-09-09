#ifndef RAYTRACER_ENGINE_PROCGEN_LANELAB_LANELAB_H
#define RAYTRACER_ENGINE_PROCGEN_LANELAB_LANELAB_H

// lanelab pipeline (ADR-0083): graph -> lanes -> profiles -> footprints -> arrangement ->
// surfaces -> terrain conform -> stats and invariants. Headless, engine types in and out.

#include "engine/procgen/lanelab/pavement.h"
#include "engine/procgen/lanelab/terrain_conform.h"
#include <functional>
#include <stdexcept>
#include <memory>
#include <string>
#include <vector>

namespace engine {
namespace lanelab {

struct Pier { int edge; Vec2 xy; double z0, z1; };

struct Check { std::string name, detail; bool ok; };

struct Result {
    RoadLabGraph graph;
    LaneSet lanes;
    std::unique_ptr<DeckHeight> heights;
    Pavement pavement;
    std::vector<Pier> piers;
    std::map<std::string, double> bridgeLen;   // per edge
    bool hasTerrain = false;
    HeightGrid terrain;                        // conformed
    ConformStats conform;
    std::vector<std::pair<int, int>> adjacent;
    double nodeMismatchBefore = 0, nodeMismatchAfter = 0, seconds = 0;
    std::string nodeMismatchWhere;             // the worst node, when it is worth naming
    std::map<std::string, double> timings;
};

// A step in the driving surface: a deck boundary edge (a slab side face) standing more than kerb height
// above or below the pavement just outside it, or above ground that terrain conform should have raised to
// it — at less than bridge height, so viaducts and true bridges are not steps. `a` owns the edge; `b` is
// the pavement lane outside it, or empty when the edge faces bare ground.
struct SurfaceStep { Vec2 at; double dz = 0; std::string a, b; double length = 0; int kind = 0; };   // kind: 0 step against pavement, 1 lip over bare ground, 2 a layer triangle that bridged two levels (dropped from the mesh)
std::vector<SurfaceStep> surfaceSteps(const Result& r, double kerb = 0.3);

// Progress of one build, 0..1 overall, weighted by measured stage seconds; the callback runs on the calling
// thread only. Returning false cancels: build() then throws BuildCancelled.
struct BuildProgress { const char* stage = ""; double fraction = 0; };
using BuildProgressFn = std::function<bool(const BuildProgress&)>;
struct BuildCancelled : std::runtime_error { BuildCancelled() : std::runtime_error("lanelab build cancelled") {} };
struct BuildOptions { const BuildProgressFn* progress = nullptr; unsigned threads = 0; };   // threads 0 = LANELAB_THREADS or every hardware thread

std::unique_ptr<Result> build(RoadLabGraph graph, const BuildOptions& opts = BuildOptions());   // heap: DeckHeight holds references into the result
std::vector<Check> invariants(const Result& r);
std::string summary(const Result& r);

}  // namespace lanelab
}  // namespace engine

#endif
