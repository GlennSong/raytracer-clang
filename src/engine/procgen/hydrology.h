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
    double bankSlope = 0.35;        // the outer banks rise this much per metre, out to the natural ground
    // INCISION: the water sits this far below the LOWEST natural ground across its corridor (so it
    // never hangs above a bank), and the inner bank climbs that height steeply (bankSteep per
    // metre) before the outer slope -- a river runs in a trench, not flush with the meadow.
    double incisionMin = 1.0, incisionMax = 3.5, incisionK = 0.05;   // incision = min + K * width
    double bankSteep = 1.4;
    double lakeMinArea = 12000.0;   // m^2 of filled basin before it is a lake
    double lakeMinDepth = 0.6;      // m of fill before a cell is lake
    int smoothIterations = 3;       // Chaikin passes on each river's path (grid staircase -> curve)
    // The drainage's own rivers (false: only the authored courses below are rivers; lakes still form
    // where basins fill -- raise lakeMinArea to keep them out too).
    bool autoRivers = true;
    // AUTHORED COURSES (ADR-0104): rivers a level lays down through waypoints -- a source up in the
    // range, a gap between two towns, the sea -- widening from width0 to width1, with a gentle meander
    // (metres) between the waypoints. Levels and depth as for any river.
    struct Course { std::vector<Vec2> points; double width0 = 12.0, width1 = 40.0, meander = 14.0; };
    std::vector<Course> courses;
};

struct RiverNode {
    Vec2 p;             // x, z
    double level = 0;   // water surface height
    double width = 0, depth = 0;
    double area = 0;    // upstream area (m^2)
    double fade = 1;    // 1 upstream, 0 at a mouth: the surface fades into the sea or lake it meets
};
struct River {
    std::vector<RiverNode> nodes; bool mouth = false; int intoLake = -1;
    bool authored = false; double width0 = 0.0, width1 = 0.0;   // an authored course: its width, source to mouth
    int shelf = -1;   // a sea mouth's channel carried across the shelf: the node it starts at (-1: none)
};   // source -> end; mouth: ends in the sea or a lake

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
    // Distance from (x, z) to the nearest river's BANK (0 at the water's edge, negative in the water),
    // capped at maxDist (no river within kFarReach: maxDist); `level`, when given, gets that river's
    // water level there (NaN when none). Indexed: cheap per sample.
    double distanceToRiver(double x, double z, double maxDist, double* level = nullptr) const;
    // The rivers' and lakes' outline grown by `margin` metres past each river bank, as rings (outer
    // counter-clockwise, holes clockwise) -- what a city cuts its blocks back from.
    std::vector<std::vector<Vec2>> corridorRings(double margin) const;
    // QUAYS (ADR-0104): a stone wall along the rivers' edges where `where` says (a city's blocks),
    // from under the water up to a parapet `parapet` above the bank (`ground`, the drawn ground):
    // one two-sided strip per stretch, u along the wall (m), v up. Lakes get none.
    RenderMesh quayMesh(const std::function<bool(double, double)>& where,
                        const std::function<double(double, double)>& ground, double parapet = 0.6) const;

    const std::vector<River>& rivers() const { return rivers_; }
    const std::vector<Lake>& lakes() const { return lakes_; }
    const HydroParams& params() const { return p_; }
    int gridSize() const { return n_; }
    Vec2 cellCenter(int idx) const;

    // THE WATER SURFACE (ADR-0099): every river's corridor unioned with the lakes into one
    // polygon set and triangulated (constrained Delaunay, the river centre lines as interior
    // points), so mouths and confluences simply merge. Each vertex sits at its water level (a
    // lake's own inside a lake, else the nearest river's) and carries the flow: the tangent is
    // the flow direction, colour r the speed (0..1) and g the fade into a mouth, u the distance
    // to the bank (0 at the bank, 0.5 well inside).
    // `ground` (the DRAWN ground, carve included): each vertex's water depth / 8 m (clamped to 1) goes in colour b, so
    // the shader can make deep water opaque and shallows clear.
    // Under open water: inside a lake, or within a river's width (plus `margin` metres). Binned:
    // cheap enough for a scatter to ask per placement, and safe on any thread.
    bool isWet(double x, double z, double margin = 0.0) const;
    // On a sea mouth's SHELF channel (the carve carried past the coast, River::shelf) or its banks: where
    // the ocean surface must cover the carved ground, though the uncarved ground stands above the sea.
    bool onShelf(double x, double z) const;
    RenderMesh waterMesh(const std::vector<std::vector<Vec2>>& sea = {},
                         const std::function<double(double, double)>& ground = {}) const;   // sea: cells it stops at

private:
    HydroParams p_;
    int n_ = 0;
    std::vector<River> rivers_;
    std::vector<Lake> lakes_;
    std::vector<int> lakeOfCell_;   // drainage cell -> lake index (-1 none)
    // the channel segments, binned on a coarse grid for the carve queries
    struct Seg { Vec2 a, b; double la, lb, wa, wb, da, db; };
    std::vector<Seg> segs_;
    static constexpr double kBankReach = 30.0;   // how far past the water the banks may be cut (m)
    double binSize_ = 64.0, reach_ = 0.0;
    int bins_ = 1;
    std::vector<std::vector<int>> bin_;
    static constexpr double kFarBin = 128.0, kFarReach = 250.0;
    int farBins_ = 0;
    std::vector<std::vector<int>> farBin_;
    void index();
    std::vector<std::vector<Vec2>> outlineRings(double margin, std::vector<Vec2>* interior) const;
};

}  // namespace engine

#endif
