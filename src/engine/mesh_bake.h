#ifndef RAYTRACER_ENGINE_MESH_BAKE_H
#define RAYTRACER_ENGINE_MESH_BAKE_H

// THE BAKER (#191; Glenn, 2026-11-06: "projecting or unwrapping a mesh so we could bake these maps ... keep building
// out the baker"): a detailed mesh's surface baked into the atlas of a cheaper one (unwrapped by mesh_unwrap.h).
// Every texel the cheap mesh's triangles cover is found on its surface -- its point, its normal, its tangent frame as
// the shader builds it -- and the NEAREST detailed surface within `cage` metres either way, facing the same way and
// (with part ids) of the same PART, is what the texel stands for. From there:
//
//   normal     the detailed normal in the cheap surface's tangent frame (x +u, y +v, z out): what a normal map holds
//   world      the same in the mesh's own space (an object-space normal map)
//   color      the detailed surface's vertex colour
//   ao         the share of `aoRays` cosine rays from the detailed surface that escape within `aoDistance`
//   curvature  the detailed surface's mean curvature (1/m; + convex, - concave): edge wear, dirt in the creases
//   height     how far the detailed surface stands out from the cheap one (m, + out): parallax, displacement
//   thickness  how far a ray into the detailed surface goes before it leaves again (m, capped): light through leaves
//
// Each texel averages `samples` x `samples` points spread over it (no aliasing on fine detail); rings of empty
// texels round each chart are filled from their neighbours (`dilate`) so filtering at a seam reads the chart's own
// values. Tagged faces (u <= -999) are skipped, as the unwrapper skips them. Deterministic.

#include "../renderer/renderer.h"
#include "mesh_unwrap.h"

#include <cstdint>
#include <vector>

namespace engine {

struct BakeOptions {
    int size = 512;            // the atlas's side in texels (the unwrap report's)
    double cage = 0.05;        // metres either side of the cheap surface the detailed one is looked for
    int samples = 1;           // points per texel along each axis (samples^2 in all)
    bool normal = true, color = false, curvature = false, height = false, thickness = false;
    int aoRays = 0;            // 0: no occlusion map
    double aoDistance = 0.5;   // metres an occluder counts within
    double thicknessMax = 0.5; // metres a thickness ray looks
    int dilate = 4;            // rings of texels filled past each chart's edge
    // the PART of each face (one entry per face of low / of high; both null: any surface counts): a texel of a low
    // part only takes the high surface of the same part (the body's paint, not the window gasket in front of it)
    const std::vector<int>* lowParts = nullptr;
    const std::vector<int>* highParts = nullptr;
};

struct BakedMaps {
    int size = 0;
    std::vector<Vec3> normal;      // tangent space, unit (0, 0, 1 where nothing was found)
    std::vector<Vec3> color;
    std::vector<float> ao;         // 1 open .. 0 shut
    std::vector<float> curvature;  // 1/m
    std::vector<float> height;     // m
    std::vector<float> thickness;  // m
    std::vector<uint8_t> covered;  // 1 a chart's own texel, 2 filled by dilation, 0 empty
    std::vector<Vec3> position;    // a chart texel's point on the cheap surface (zero elsewhere)
    std::vector<Vec3> world;       // the detailed surface's normal there, in the mesh's own space (zero: none found)
    int texels = 0, hits = 0;      // chart texels, and how many found the detailed surface
};

BakedMaps bakeMaps(const RenderMesh& low, const RenderMesh& high, const BakeOptions& opts = {});

// A MODEL BAKED PART BY PART into one atlas: each part's cheap mesh unwrapped with the others (no chart crossing from
// one part into the next) and baked from its own detailed mesh only. A part with `bake` false (glass, a lens) takes
// no atlas space and keeps its own uvs: it is drawn smooth by its material.
struct BakePart {
    RenderMesh low, high;
    bool bake = true;
};
struct BakedModel {
    RenderMesh mesh;               // the parts' cheap meshes merged in order, the baked ones on the atlas
    std::vector<std::size_t> firstIndex;   // where each part's indices start in `mesh`
    UnwrapReport unwrap;
    BakedMaps maps;
};
BakedModel bakeParts(const std::vector<BakePart>& parts, const UnwrapOptions& unwrap = {}, BakeOptions bake = {});

// The maps as bytes: normals (tangent or object space) as RGB n * 0.5 + 0.5, colour as RGB, occlusion as grey,
// curvature as grey 0.5 + k * radius / 2 (radius: the curvature that reads white, 1/m), height as grey 0.5 + h /
// (2 range), thickness as grey t / max
std::vector<uint8_t> normalMapBytes(const BakedMaps& maps);
std::vector<uint8_t> worldNormalMapBytes(const BakedMaps& maps);
std::vector<uint8_t> colorMapBytes(const BakedMaps& maps);
std::vector<uint8_t> aoMapBytes(const BakedMaps& maps);
std::vector<uint8_t> curvatureMapBytes(const BakedMaps& maps, double radius = 0.05);
std::vector<uint8_t> heightMapBytes(const BakedMaps& maps, double range);
std::vector<uint8_t> thicknessMapBytes(const BakedMaps& maps, double max);

}  // namespace engine

#endif
