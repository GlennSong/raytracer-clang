#ifndef RAYTRACER_RENDERER_H
#define RAYTRACER_RENDERER_H

#include "../rt_math.h"
#include "../handle.h"
#include <vector>
#include <memory>
#include <cstdint>

struct Vertex {
    Vec3 position;
    Vec3 normal;
    float u, v;

    Vertex() : u(0), v(0) {}
    Vertex(const Vec3& pos, const Vec3& norm, float u = 0, float v = 0)
        : position(pos), normal(norm), u(u), v(v) {}
};

struct RenderMaterial {
    Vec3 albedo;
    float metallic;
    float roughness;
    float opacity;
    Vec3 emission;

    RenderMaterial()
        : albedo(0.8, 0.8, 0.8), metallic(0.0), roughness(0.5),
          opacity(1.0), emission(0, 0, 0) {}
};

struct RenderMesh {
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    int materialIndex;

    RenderMesh() : materialIndex(0) {}
};

struct PointLight {
    Vec3 position;
    Vec3 color;
    float intensity;

    PointLight() : color(1, 1, 1), intensity(1.0f) {}
    PointLight(const Vec3& pos, const Vec3& col, float intensity)
        : position(pos), color(col), intensity(intensity) {}
};

enum class CameraProjection { Perspective, Orthographic };

struct CameraState {
    Vec3 position;
    Vec3 target;
    Vec3 up;
    CameraProjection projection = CameraProjection::Perspective;
    float fovDegrees;       // vertical field of view (perspective)
    float orthoHeight;      // vertical world units visible (orthographic)
    float aspectRatio;
    float nearPlane;
    float farPlane;

    CameraState()
        : position(0, 0, 3), target(0, 0, 0), up(0, 1, 0),
          fovDegrees(60.0f), orthoHeight(10.0f), aspectRatio(1.0f),
          nearPlane(0.1f), farPlane(1000.0f) {}
};

// Generation-checked GPU-resource identities (ADR-0007). Distinct tag types so
// a MeshHandle can't be passed where a BufferHandle is expected, and a stale
// handle (its slot freed and reused) is detected rather than silently aliasing.
// A default-constructed handle is null (valid() == false).
struct MeshTag {};
struct BufferTag {};
using MeshHandle = Handle<MeshTag>;
using BufferHandle = Handle<BufferTag>;

class Renderer {
public:
    virtual ~Renderer() = default;

    // windowHandle is an opaque native OS window pointer (e.g. NSWindow* on
    // macOS) from Window::nativeWindowHandle() — never a windowing-library type.
    virtual bool initialize(void* windowHandle, int width, int height) = 0;
    virtual void shutdown() = 0;
    virtual void resize(int width, int height) = 0;

    virtual MeshHandle uploadMesh(const RenderMesh& mesh) = 0;
    virtual void removeMesh(MeshHandle handle) = 0;

    virtual void beginFrame() = 0;
    virtual void setCamera(const CameraState& camera) = 0;
    virtual void setLights(const std::vector<PointLight>& lights, float exposure = 1.0f) = 0;
    virtual void drawMesh(MeshHandle handle, const Mat4& transform,
                          const RenderMaterial& material) = 0;
    virtual void endFrame() = 0;

    // Debug-UI (Dear ImGui) backend hooks — see ADR-0011. No-ops unless a
    // backend implements them and the build defines RT_ENABLE_IMGUI; engine
    // code never sees ImGui types. The per-frame new-frame/submit are handled
    // internally inside beginFrame()/endFrame(); only setup/teardown are here.
    // windowHandle is the same opaque native pointer passed to initialize().
    virtual void initDebugUi(void* /*windowHandle*/) {}
    virtual void shutdownDebugUi() {}

    static std::unique_ptr<Renderer> create();
};

#endif
