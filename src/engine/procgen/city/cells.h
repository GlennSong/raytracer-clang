#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_CELLS_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_CELLS_H

// THE COMPLEX (architectural toolkit, issue #105, step 5; ADR-0176): a building made of CELLS, each a mass in its
// own frame with its own params, standing against one another. Where two cells touch, the touching faces are an
// ADJACENCY: a shared wall, blank on both sides up to the lower cell's roof (the taller cell keeps its windows
// above it), with OPENINGS through it where the cells connect. A cell with no outer face is entered through such
// an opening from a neighbour's ground floor. No booleans: a complex's slice is its cells' slices side by side, and
// each cell is grown as the plan building it is, the shared faces as party walls (shape_grammar.h partyN /
// partyAt / partyTop / partyDoors, which this generalises). The first preset is the BUNDLE: nx by ny rectangular
// tubes on one frame, each its own height -- the Sears Tower's nine.

#include "mass.h"
#include "mass_stack.h"
#include <memory>
#include <string>
#include <vector>

namespace engine {

struct Opening {
    Real u0 = 0, u1 = 0;   // along the shared run from its start
    int k0 = 0, k1 = 0;    // the storeys it spans
};

struct Adjacency {
    int a = -1, b = -1;    // the cells
    Vec2 n;                // out of a, into b
    Real at = 0;           // dot(n, p) on the shared plane
    Vec2 p0, p1;           // the shared run, world, along a's edge
    int k0 = 0, k1 = 0;    // the storeys both cells have (k1: the lower cell's top storey)
    std::vector<Opening> openings;
};

struct Bridge {
    int a = -1, b = -1;
    int storey = 0;
    Real width = 0;
};

struct CellFrame {
    SiteFrame frame;
    std::shared_ptr<Mass> mass;
    BuildingParams params;
    Real baseY = 0;
    std::string role;
    Poly2 plan;                // the base outline, world CCW (the mass's base slice)
    bool outerFace = false;    // an edge no neighbour stands against (resolve)
    bool connected = false;    // reachable from the street: its own outer face, or an opening from a connected cell
};

struct Complex : Mass {
    std::vector<CellFrame> cells;
    std::vector<Adjacency> adj;
    std::vector<Bridge> bridges;   // (owed: a bridge between cells above the ground)
    Slice slice(Real y) const override;
    Real height() const override;
    MassBounds bounds() const override;
    RenderMesh shell(int lod, const Vec3& colour) const override;
    // THE SHARED FACES: every pair of cells whose edges lie on one line facing each other and overlap along it by
    // `minRun` is an adjacency; each cell's params take the wall as a party wall (up to four), blank up to the
    // lower cell's top storey. Sets every cell's `outerFace`.
    void resolve(Real tol = 0.25, Real minRun = 2.0);
    // A WAY IN FOR EVERY CELL: a cell with an outer face enters from it (its `faceDir` turned to that face, the one
    // nearest the street's `faceDir`); a cell without one is entered through a shared wall from a connected
    // neighbour -- the neighbour's wall takes a doorway in its middle bay (partyDoors), the cell's entrance faces it.
    // Sets every cell's `connected`.
    void connectLobbies(const Vec2& faceDir);
};

// THE BUNDLE: nx by ny rectangular cells filling `box` (a CCW rectangle; its first edge is the x direction), each a
// stack of its own floors (`floorsPerCell`, row-major, y rows of x cells; short lists repeat the last), the touching
// faces shared, every cell with a way in. The base params are `params`.
Complex bundledTubes(const Poly2& box, const BuildingParams& params, int nx, int ny, const std::vector<int>& floorsPerCell);

}  // namespace engine

#endif
