#ifndef RAYTRACER_ENGINE_MESH_BUILDER_H
#define RAYTRACER_ENGINE_MESH_BUILDER_H

#include "../renderer/renderer.h"
#include <string>
#include <vector>

namespace engine {

struct MeshBuilder {
    // Level-format shape names ("box", "sphere", ...) -> mesh, with the same
    // size semantics as the level loader (x = radius for sphere/cylinder/...,
    // y = height where applicable). Empty mesh for unknown names.
    static RenderMesh shape(const std::string& name, Vec3 size);

    static RenderMesh box(Vec3 size);
    static RenderMesh sphere(float radius, int stacks = 32, int slices = 64);
    static RenderMesh cylinder(float radius, float height, int slices = 32);
    static RenderMesh plane(float width, float depth);
    static RenderMesh cone(float radius, float height, int slices = 32);
    static RenderMesh wedge(Vec3 size);
    static RenderMesh torus(float majorRadius, float minorRadius,
                            int majorSegments = 32, int minorSegments = 16);
    static RenderMesh capsule(float radius, float height,
                              int stacks = 16, int slices = 32);

    // --- Procgen-grade assembly ops (ROADMAP 3.3) ------------------------
    // These operate on RenderMesh as a value type so generators (L-systems,
    // terrain, scatter) compose geometry the same way loaded meshes are built —
    // no special procgen path. Pure data; unit-tested headless.

    // Append `src` into `dst`, offsetting src's indices so the two merge into
    // one vertex/index buffer (dst keeps its materialIndex).
    static void append(RenderMesh& dst, const RenderMesh& src);

    // Append `src` transformed by `xform`: positions by the matrix, normals by
    // its inverse-transpose (correct under non-uniform scale), renormalized.
    // The building block for kit-bashing parts — L-system segments, props.
    static void appendTransformed(RenderMesh& dst, const RenderMesh& src,
                                  const Mat4& xform);

    // Transform a mesh in place (positions + normals/tangents as above).
    static void transform(RenderMesh& mesh, const Mat4& xform);

    // Merge several meshes into one.
    static RenderMesh merged(const std::vector<RenderMesh>& parts);

    // Recompute smooth vertex normals from face geometry (area-weighted by the
    // face cross product), e.g. after displacing vertices (noise terrain).
    static void recomputeNormals(RenderMesh& mesh);

    // Planar UVs projected along an axis (0=x, 1=y, 2=z): the two perpendicular
    // position components, scaled, become (u, v). A cheap default mapping for
    // terrain (axis=1) and generated geometry that ships without UVs.
    static void generatePlanarUVs(RenderMesh& mesh, int axis = 1, float scale = 1.0f);

    // Tint per-vertex color by height: `low` at the mesh's lowest vertex, `high`
    // at its highest, linearly between. A cheap strata/canopy effect — e.g. a
    // brown trunk fading to green leaves on a tree (use a white material so the
    // vertex color shows). No-op on an empty mesh.
    static void bakeHeightColor(RenderMesh& mesh, const Vec3& low, const Vec3& high);
};

}  // namespace engine

#endif
