#ifndef RAYTRACER_ENGINE_PROCGEN_HYDROLOGY_H
#define RAYTRACER_ENGINE_PROCGEN_HYDROLOGY_H

// HYDROLOGY (ADR-0099): rivers and lakes that the land itself makes. Rain falls everywhere and
// runs downhill; where water from enough land gathers, a river runs, cutting its channel;
// basins fill into lakes until they spill; everything reaches the sea or the map's edge.
//
// Computed once on a drainage grid over the terrain's base relief:
//   1. PRIORITY-FLOOD depression filling (from the map's edge and the sea inward): the filled
//      surface says where lakes stand (and at what level they spill), and each cell's
//      RECEIVER -- the neighbour it drains into -- so water always finds a way out;
//   2. FLOW ACCUMULATION: the land area draining through every cell;
//   3. RIVERS where that area passes a threshold, traced from their sources to a confluence,
//      a lake, the sea or the edge, smoothed, and given width, depth and a water LEVEL that only
//      falls downstream;
//   4. the CARVE: the ground lowered to a rounded channel and sloping banks along every river
//      (only ever cut, never raised), applied inside terrainHeight (TerrainParams::hydro), so
//      CDLOD, colliders, placement and the baked pyramid all see the channels.
// The water SURFACES are built from the same network (hydrology meshes: river ribbons, lakes).

#include "../../rt_math.h"
#include "../../renderer/renderer.h"   // RenderMesh
#include "city/polygon.h"               // Vec2 (planar x, z)

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace engine {

struct HydroParams {
    double half = 1500.0;           // the square region [-half, half]^2 the drainage grid covers
    double cell = 8.0;              // drainage grid spacing (m)
    double seaLevel = -1e30;        // cells below it are sea (outlets); -1e30 = no sea
    double riverArea = 300000.0;    // upstream area (m^2) at which a channel is a river
    double widthMin = 5.0, widthMax = 70.0, widthK = 0.035;   // width = min + K * sqrt(area)
    double depthMin = 0.8, depthMax = 5.0, depthK = 0.0022;   // depth = min + K * sqrt(area)
    double bankSlope = 0.35;        // the banks rise this much per metre beyond the channel
    double lakeMinArea = 12000.0;   // m^2 of filled basin before it is a lake
    double lakeMinDepth = 0.6;      // m of fill before a cell is lake
    int smoothIterations = 3;       // Chaikin passes on each river's path (grid staircase -> curve)
};

struct RiverNode {
    Vec2 p;             // x, z
    double level = 0;   // water surface height
    double width = 0, depth = 0;
    double area = 0;    // upstream area (m^2)
    double fade = 1;    // 1 upstream, 0 at a mouth: the surface fades into the sea or lake it meets
};
struct River { std::vector<RiverNode> nodes; bool mouth = false; };   // source -> end; mouth: ends in the sea or a lake

struct Lake {
    double level = 0;
    std::vector<int> cells;   // drainage-grid cells under the lake
    double minX = 0, minZ = 0, maxX = 0, maxZ = 0;
};

class Hydrology {
public:
    static std::shared_ptr<const Hydrology> build(const std::function<double(double, double)>& ground,
                                                  const HydroParams& p);

    // The ground at (x, z) with the channels cut into it: min(h, channel profile).
    double carve(double x, double z, double h) const;
    // Distance to the nearest river channel's edge (<= 0 inside it), capped at `maxDist`.
    double distanceToRiver(double x, double z, double maxDist) const;

    const std::vector<River>& rivers() const { return rivers_; }
    const std::vector<Lake>& lakes() const { return lakes_; }
    const HydroParams& params() const { return p_; }
    int gridSize() const { return n_; }
    Vec2 cellCenter(int idx) const;

    // The water surfaces: one ribbon per river (u across 0..1, v the distance along in metres,
    // colour r the flow speed 0..1 from the surface's slope, b its fade into a mouth) and one flat
    // sheet per lake.
    RenderMesh riverMesh() const;
    RenderMesh lakeMesh() const;

private:
    HydroParams p_;
    int n_ = 0;
    std::vector<River> rivers_;
    std::vector<Lake> lakes_;
    // the channel segments, binned on a coarse grid for the carve queries
    struct Seg { Vec2 a, b; double la, lb, wa, wb, da, db; };
    std::vector<Seg> segs_;
    static constexpr double kBankReach = 30.0;   // how far past the water the banks may be cut (m)
    double binSize_ = 64.0, reach_ = 0.0;
    int bins_ = 1;
    std::vector<std::vector<int>> bin_;
    void index();
};

}  // namespace engine

#endif
