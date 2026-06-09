#ifndef RAYTRACER_ENGINE_MESH_BUILDER_H
#define RAYTRACER_ENGINE_MESH_BUILDER_H

#include "../renderer/renderer.h"

namespace engine {

struct MeshBuilder {
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
};

}  // namespace engine

#endif
