#ifndef ROADLAB_TESSELLATE_H
#define ROADLAB_TESSELLATE_H

// The mesh is deliberately dumb. Its whole job is to be a surface that carries
// (s, t) in metres per vertex; everything that makes a road look like a road is
// computed from those two numbers in surface.h. That split is the point: the
// same geometry serves any paint scheme, LOD costs nothing in authoring, and
// there is no marking mesh to z-fight with.
//
// One rule keeps the surface watertight: adjacent strips share their boundary t
// EXACTLY, and a junction's arms snap to the pad's own boundary points. Cracks
// in a road mesh almost always come from two pieces computing the same edge
// slightly differently, so only one piece is ever allowed to compute it.

#include "network.h"
#include "junction.h"
#include <vector>

namespace roadlab {

enum class MatKind : uint8_t {
    RoadSurface,   // shade through shadeRoadSurface using (s,t)
    JunctionPad,   // shade through shadeJunctionPad using the world plan point
    Flat,          // vertex colour only: structures, props, terrain
};

struct Vertex {
    Vec3 pos{0, 0, 0};
    Vec3 normal{0, 1, 0};
    double s = 0, t = 0;
    Vec3 color{0.5, 0.5, 0.5};
    int road = -1;
    int junction = -1;
    MatKind material = MatKind::Flat;
    double roughness = 0.9;
};

struct Mesh {
    std::vector<Vertex> verts;
    std::vector<uint32_t> indices;

    uint32_t push(const Vertex& v) {
        verts.push_back(v);
        return uint32_t(verts.size() - 1);
    }
    void tri(uint32_t a, uint32_t b, uint32_t c) {
        indices.push_back(a);
        indices.push_back(b);
        indices.push_back(c);
    }
    void quad(uint32_t a, uint32_t b, uint32_t c, uint32_t d) {
        tri(a, b, c);
        tri(a, c, d);
    }
    void append(const Mesh& other);
    size_t triangleCount() const { return indices.size() / 3; }
};

struct TessParams {
    double maxStationStep = 4.0;    // longest step along s on a straight
    double minStationStep = 0.6;
    double curvatureTolerance = 0.12;   // radians of heading change per step
    bool skipZeroWidth = true;
};

// One quad strip per (lane section, strip). Strips are tessellated separately so
// that a lane appearing or disappearing changes only its own quads, and so the
// shared boundary offsets are computed exactly once.
void tessellateRoad(const Road& road, Mesh& out, const TessParams& p = {});

// The pad: the boundary polygon fanned from its centroid, with vertex heights
// interpolated from the arms so the surface meets every approach at its own
// grade. Shaded in junction-local coordinates.
void tessellateJunction(const Network& net, const Junction& j, Mesh& out);

// Surface height inside a junction pad, blended from the arms. Used both by the
// tessellator and by anything that needs to stand something on the pad.
double junctionElevationAt(const Network& net, const Junction& j, Vec2 planPoint);

// Everything at once, including structures (structure.h) if `withStructures`.
void tessellateNetwork(const Network& net, Mesh& out, const TessParams& p = {},
                       bool withStructures = true);

// --- helper geometry (structures, props) ----------------------------------

void emitBox(Mesh& m, Vec3 centre, Vec3 half, double yaw, Vec3 color, double roughness = 0.85);
void emitCylinder(Mesh& m, Vec3 base, double radius, double height, int sides, Vec3 color,
                  double roughness = 0.7);
// A flat plate standing in the world, used for sign faces and barrier webs.
void emitPlate(Mesh& m, Vec3 centre, Vec3 right, Vec3 up, Vec3 color, double roughness = 0.5);
// A closed polygon fan (sign faces: octagon, triangle, rectangle).
void emitPolygonFace(Mesh& m, Vec3 centre, Vec3 right, Vec3 up, int sides, double startAngle,
                     Vec3 color);
// Sweep a 2D profile (in the road's (t, h) plane) along an s-range: parapets,
// jersey barriers, kerb faces and tunnel bores are all this one operation.
void sweepProfile(const Road& road, double s0, double s1, const std::vector<Vec2>& profile,
                  Vec3 color, Mesh& out, double step = 2.0, bool closeEnds = false);

}  // namespace roadlab

#endif
