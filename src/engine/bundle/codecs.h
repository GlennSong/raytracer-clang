#ifndef RAYTRACER_ENGINE_BUNDLE_CODECS_H
#define RAYTRACER_ENGINE_BUNDLE_CODECS_H

// Section codecs for engine types (ADR-0084). Each codec writes a magic + version first and reads back
// only what it understands; a foreign or newer section reads as false, never as garbage.
//
// PackedMesh is the on-disk and in-flight shape of a RenderMesh: float32 planes (position, normal, tangent,
// uv; colour only when it varies across the mesh) and u32 indices — what the GPU and Jolt keep anyway.
// The cold path packs and unpacks too, so a level instantiated from a fresh build and one instantiated
// from a bundle are bit-identical.

#include "engine/bundle/binary_stream.h"
#include "engine/components.h"
#include "engine/procgen/city/road_net.h"
#include "renderer/renderer.h"
#include "rt_math.h"

#include <cstdint>
#include <string>
#include <vector>

namespace engine {
namespace bundle {

struct PackedMesh {
    enum : uint32_t { kColorUniform = 1u, kCollidable = 2u, kPaint = 4u };
    std::string name;            // material name (asphalt, concrete, …) — load-bearing for the loader's rules
    uint32_t flags = kColorUniform;
    float albedo[3] = {1, 1, 1};       // the Renderable's material colour
    float vertexColor[3] = {1, 1, 1};  // the one vertex colour when kColorUniform
    float roughness = 0.93f;
    double friction = 0.85;
    std::vector<float> pos, nrm, tan, uv, col;   // 3n, 3n, 3n, 2n, 3n (col empty when uniform)
    std::vector<uint32_t> idx;
    size_t vertexCount() const { return pos.size() / 3; }
    size_t triangleCount() const { return idx.size() / 3; }
    bool colorUniform() const { return (flags & kColorUniform) != 0; }
    bool collidable() const { return (flags & kCollidable) != 0; }
    bool paint() const { return (flags & kPaint) != 0; }
    size_t bytes() const { return (pos.size() + nrm.size() + tan.size() + uv.size() + col.size()) * sizeof(float) + idx.size() * sizeof(uint32_t); }
};

PackedMesh packMesh(const RenderMesh& m, const std::string& name, const Vec3& albedo, uint32_t flags, float roughness, double friction);
RenderMesh unpackMesh(const PackedMesh& p);
void colliderFromPacked(const PackedMesh& p, MeshCollider& mc);   // positions + indices only, no Vertex materialised
void putPackedMesh(BinWriter& w, const PackedMesh& p);
bool getPackedMesh(BinReader& r, PackedMesh& p);

// A regular height grid over the plan (x, y = world z), doubles, row-major ny rows of nx.
struct HeightGridBlob { double x0 = 0, y0 = 0, res = 1; int32_t nx = 0, ny = 0; std::vector<double> z; };
void putHeightGrid(BinWriter& w, const HeightGridBlob& g);
bool getHeightGrid(BinReader& r, HeightGridBlob& g);

void putRoadGraph(BinWriter& w, const RoadGraph& g);
bool getRoadGraph(BinReader& r, RoadGraph& g);
void putRoadEntity(BinWriter& w, const RoadEntity& e);   // graph + look + plan (hubs, freeway plans)
bool getRoadEntity(BinReader& r, RoadEntity& e);

void putRings(BinWriter& w, const std::vector<std::vector<Vec2>>& rings);
bool getRings(BinReader& r, std::vector<std::vector<Vec2>>& rings);

}  // namespace bundle
}  // namespace engine

#endif
