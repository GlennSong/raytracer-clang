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

// A BRIDGE between two cells that stand apart (ADR-0179): at `storey`, `width` wide, between the faces of `a` and
// `b` that face each other. resolveBridges writes it onto both cells' params as a LINK (shape_grammar.h: a doorway
// in each face, the bridge drawn from a's), and records the span it found.
struct Bridge {
    int a = -1, b = -1;
    int storey = 0;
    Real width = 0;
    Real span = 0;         // the gap it crosses (resolveBridges)
    bool placed = false;   // both faces found, both cells tall enough
};

struct CellFrame {
    SiteFrame frame;
    std::shared_ptr<Mass> mass;
    BuildingParams params;
    Real baseY = 0;
    std::string role;
    Poly2 plan;                // the base outline, world CCW (the mass's base slice)
    bool outerFace = false;    // an edge no neighbour stands against (resolve): a street face, or a court's
    bool streetFace = false;   // an outer face on the complex's own boundary (connectLobbies): where a door can open
    bool connected = false;    // reachable from the street: its own street face, or an opening from a connected cell
};

struct Complex : Mass {
    std::vector<CellFrame> cells;
    std::vector<Adjacency> adj;
    std::vector<Bridge> bridges;   // the sky bridges between cells standing apart (resolveBridges)
    Slice slice(Real y) const override;
    Real height() const override;
    MassBounds bounds() const override;
    RenderMesh shell(int lod, const Vec3& colour) const override;
    // THE SHARED FACES: every pair of cells whose edges lie on one line facing each other and overlap along it by
    // `minRun` is an adjacency; each cell's params take the wall as a party wall (up to four), blank up to the
    // lower cell's top storey. Sets every cell's `outerFace`.
    void resolve(Real tol = 0.25, Real minRun = 2.0);
    // A WAY IN FOR EVERY CELL: a cell with a STREET face -- an outer face on the complex's own boundary (a face
    // onto an enclosed court is outer but opens onto nothing) -- enters from it (its `faceDir` turned to that
    // face, the one nearest the street's `faceDir`); a cell without one is entered through a shared wall from a
    // connected neighbour -- the neighbour's wall takes a doorway in its middle bay (partyDoors), the cell's
    // entrance faces it. Sets every cell's `streetFace` and `connected`.
    void connectLobbies(const Vec2& faceDir);
    // THE BRIDGES: for every bridge, the two faces that face each other across a gap (parallel, overlapping along
    // their run); both cells take a link at the bridge's storey (clamped under both roofs) at the middle of the
    // overlap, `a`'s with the span to draw. A bridge with no such faces, or on cells too low, stays unplaced.
    void resolveBridges();
    // A WIDE OPENING (ADR-0180) through the shared wall of adjacency `ai`: the wall absent from `u0` to `u1` along
    // the shared run (from its start, in a's edge direction) on storeys k0..k1, on BOTH cells (params.partyOpen*);
    // recorded as an Opening. A ground opening across the wall's middle bay makes its party door redundant.
    void openWall(std::size_t ai, Real u0, Real u1, int k0, int k1);
};

// THE BUNDLE: nx by ny rectangular cells filling `box` (a CCW rectangle; its first edge is the x direction), each a
// stack of its own floors (`floorsPerCell`, row-major, y rows of x cells; short lists repeat the last), the touching
// faces shared, every cell with a way in. The base params are `params`.
Complex bundledTubes(const Poly2& box, const BuildingParams& params, int nx, int ny, const std::vector<int>& floorsPerCell);

// THE COURTYARD BLOCK: eight cells round an open court on `box` -- four corners `depth` square and four wings
// `depth` deep between them, every cell with a street face, the corners shared with the wings at their ends (no
// edge half shared: a party wall is a whole edge). `floorsPerWing`: the four wings then the four corners, in
// order (south wing, east, north, west; SW corner, SE, NE, NW); short lists repeat the last. The court must keep
// `minCourt` across, or the result is empty.
Complex courtyardWings(const Poly2& box, const BuildingParams& params, Real depth, const std::vector<int>& floorsPerWing, Real minCourt = 8.0);

// THE RING: one cell per edge of a convex `outline` (CCW), each the trapezoid between that edge and the outline
// offset `depth` inward, neighbours sharing their radial faces whole -- a round block round a court (Apple Park on
// a sixteen-gon; four cells with mitred corners on a rectangle). `floorsPerCell` per edge (short lists repeat the
// last). Empty when the court inside would be narrower than `minCourt`.
Complex ringCells(const Poly2& outline, const BuildingParams& params, Real depth, const std::vector<int>& floorsPerCell, Real minCourt = 6.0);

// THE TWINS: two towers on `box` (CCW; its first edge the long axis they stand along), `gap` apart, each the box's
// full depth, joined by a SKY BRIDGE `bridgeW` wide at `bridgeStorey` (clamped under both roofs; 0 = two thirds
// of the way up the lower one). `floorsPerCell`: the two heights (short lists repeat the last). Empty when either
// tower would be narrower than its core needs. Petronas.
Complex twinTowers(const Poly2& box, const BuildingParams& params, Real gap, const std::vector<int>& floorsPerCell, int bridgeStorey = 0,
                   Real bridgeW = 4.5);

// THE MALL AS CELLS (ADR-0180; the plan's Westfield): on `box` (CCW), with the street toward `faceDir`, a SPINE cell
// along the box's long axis at its street side -- the American mall (`mall`: bigBox 6, its wings +u / -u ending
// OPEN, its court in the middle, its own escalators and lift) 40 m deep -- an ANCHOR cell at each end of the spine
// (`anchor`: a big-box store, given the spine's floors and storey heights so the levels meet), the concourse open
// onto it on every level through a wide opening the concourse's width; and, where the box is deep enough behind
// the spine, a GARAGE cell (`garage`: parking decks) against the back of the court, open onto it at the ground
// across 8 m. Every cell its own building, entered from the street as well. Empty when the spine's wings would be
// too short (the box under 142 m long or 40 m deep).
Complex mallCells(const Poly2& box, const Vec2& faceDir, const BuildingParams& mall, const BuildingParams& anchor,
                  const BuildingParams& garage, bool withGarage = true, Real anchorDepth = 30.0);

}  // namespace engine

#endif
