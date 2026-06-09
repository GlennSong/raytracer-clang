#ifndef RAYTRACER_RENDERER_H
#define RAYTRACER_RENDERER_H

#include "../rt_math.h"
#include "../handle.h"
#include <vector>
#include <memory>
#include <cstdint>

namespace engine {

struct MeshTag {};
struct BufferTag {};
struct TextureTag {};
using MeshHandle = Handle<MeshTag>;
using BufferHandle = Handle<BufferTag>;
using TextureHandle = Handle<TextureTag>;

struct Vertex {
    Vec3 position;
    Vec3 normal;
    Vec3 tangent;
    float u, v;

    Vertex() : u(0), v(0) {}
    Vertex(const Vec3& pos, const Vec3& norm, float u = 0, float v = 0)
        : position(pos), normal(norm), u(u), v(v) {}
    Vertex(const Vec3& pos, const Vec3& norm, const Vec3& tan, float u = 0, float v = 0)
        : position(pos), normal(norm), tangent(tan), u(u), v(v) {}
};

inline BoundingSphere computeBoundingSphere(const Vertex* vertices, size_t count) {
    if (count == 0) return {};
    Vec3 center;
    for (size_t i = 0; i < count; i++) center += vertices[i].position;
    center /= static_cast<Real>(count);
    Real maxDistSq = 0;
    for (size_t i = 0; i < count; i++) {
        Real dSq = (vertices[i].position - center).lengthSquared();
        if (dSq > maxDistSq) maxDistSq = dSq;
    }
    return {center, std::sqrt(maxDistSq)};
}

struct RenderMaterial {
    Vec3 albedo;
    float metallic;
    float roughness;
    float opacity;
    Vec3 emission;
    uint32_t flags = 0;

    static constexpr uint32_t FLAG_CHECKERBOARD = 1;

    TextureHandle albedoMap;
    TextureHandle normalMap;
    TextureHandle metallicRoughnessMap;
    TextureHandle emissiveMap;
    TextureHandle aoMap;

    RenderMaterial()
        : albedo(0.8, 0.8, 0.8), metallic(0.0), roughness(0.5),
          opacity(1.0), emission(0, 0, 0), flags(0) {}
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

struct DirectionalLight {
    Vec3 direction;     // toward the light source (sun direction)
    Vec3 color;
    float intensity;
    bool castsShadow;

    DirectionalLight()
        : direction(0.5, 0.7, -0.3), color(1, 1, 1), intensity(1.0f),
          castsShadow(true) {}
    DirectionalLight(const Vec3& dir, const Vec3& col, float intensity,
                     bool shadow = true)
        : direction(dir), color(col), intensity(intensity),
          castsShadow(shadow) {}
};

struct SpotLight {
    Vec3 position;
    Vec3 direction;     // toward target
    Vec3 color;
    float intensity;
    float innerConeAngle;   // radians
    float outerConeAngle;   // radians
    bool castsShadow;

    SpotLight()
        : color(1, 1, 1), intensity(1.0f),
          innerConeAngle(0.3f), outerConeAngle(0.5f),
          castsShadow(false) {}
    SpotLight(const Vec3& pos, const Vec3& dir, const Vec3& col,
              float intensity, float inner, float outer,
              bool shadow = false)
        : position(pos), direction(dir), color(col), intensity(intensity),
          innerConeAngle(inner), outerConeAngle(outer),
          castsShadow(shadow) {}
};

struct ShadowConfig {
    float bias = 0.005f;
    float normalBias = 0.02f;
    int resolution = 2048;
    bool enabled = true;
};

// Procedural sky parameters (ADR-0016). Drive the analytic skybox and, via the
// reflection-probe bake, image-based lighting. A DayNightCycle writes these each
// frame; the defaults reproduce the original fixed daytime sky, so behavior is
// unchanged when nothing drives them.
struct ProceduralSky {
    Vec3  sunDirection{0.4, 0.8, -0.3};   // toward the sun (world space)
    Vec3  sunColor{1.0, 0.95, 0.8};       // sun disc / glow tint
    float sunDiscIntensity = 1.0f;        // disc brightness scale (0 at night)
    Vec3  zenithColor{0.25, 0.45, 0.85};
    Vec3  horizonColor{0.6, 0.75, 0.9};
    Vec3  groundColor{0.35, 0.3, 0.25};
};

struct SceneLighting {
    DirectionalLight sun;
    std::vector<PointLight> pointLights;
    std::vector<SpotLight> spotLights;
    ShadowConfig shadow;
    ProceduralSky sky;
    float exposure = 1.0f;
    float ambientMultiplier = 0.3f;
};

struct ReflectionProbe {
    Vec3 position;
    float influenceRadius;
    Vec3 boxMin, boxMax;    // AABB for parallax correction
    int priority = 0;       // higher = preferred when overlapping

    ReflectionProbe() : influenceRadius(10.0f) {}
    ReflectionProbe(const Vec3& pos, float radius, const Vec3& bMin, const Vec3& bMax)
        : position(pos), influenceRadius(radius), boxMin(bMin), boxMax(bMax) {}
};

struct RenderStats {
    uint32_t drawCalls = 0;
    uint32_t instancedDrawCalls = 0;
    uint32_t totalInstances = 0;
    uint32_t entitiesSubmitted = 0;
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
    virtual BoundingSphere getMeshBounds(MeshHandle handle) const = 0;
    virtual TextureHandle uploadTexture(int width, int height, int channels,
                                        const uint8_t* data) = 0;
    // Float (HDR) texture upload — e.g. equirectangular environment maps decoded
    // from Radiance .hdr. `data` is linear RGB(A), `channels` 3 or 4. Stored as a
    // half-float texture. Default no-op so non-HDR-capable backends stay valid.
    virtual TextureHandle uploadTextureHDR(int /*width*/, int /*height*/,
                                           int /*channels*/, const float* /*data*/) {
        return TextureHandle{};
    }
    virtual void removeTexture(TextureHandle handle) = 0;

    // Bind an equirectangular HDR texture as the scene environment, driving the
    // skybox and (via the reflection-probe bake) image-based lighting — see
    // ADR-0016. An invalid handle restores the procedural sky. No-op by default.
    virtual void setEnvironmentMap(TextureHandle /*equirect*/) {}
    virtual RenderStats getRenderStats() const = 0;

    virtual void beginFrame() = 0;
    virtual void setCamera(const CameraState& camera) = 0;
    virtual void setLights(const SceneLighting& lighting) = 0;
    virtual void drawMesh(MeshHandle handle, const Mat4& transform,
                          const RenderMaterial& material) = 0;
    virtual void setReflectionProbes(const std::vector<ReflectionProbe>& probes) {}
    virtual void endFrame() = 0;

    // Debug-UI (Dear ImGui) backend hooks — see ADR-0011. No-ops unless a
    // backend implements them and the build defines RT_ENABLE_IMGUI; engine
    // code never sees ImGui types. The per-frame new-frame/submit are handled
    // internally inside beginFrame()/endFrame(); only setup/teardown are here.
    // windowHandle is the same opaque native pointer passed to initialize().
    virtual void initDebugUi(void* /*windowHandle*/) {}
    virtual void shutdownDebugUi() {}

    // Runtime toggles for post-processing effects (debug/tuning)
    bool ssaoEnabled = true;
    bool ssrEnabled = true;
    bool reflectionProbesEnabled = true;

    // Debug visualization: 0=normal, 1=AO only, 2=SSR only, 3=depth, 4=normals
    int debugView = 0;

    // Bloom
    bool bloomEnabled = true;
    struct BloomParams {
        float threshold = 1.0f;
        float knee      = 0.5f;
        float intensity = 0.3f;
    } bloomParams;

    // Compact stats HUD visible during gameplay
    bool showHud = false;

    // Frame rate cap (0 = uncapped, otherwise target FPS like 30 or 60)
    int targetFps = 0;

    // SSR tuning parameters
    struct SSRParams {
        float maxRayDist    = 20.0f;
        float thickness     = 0.3f;
        float thicknessFar  = 2.0f;
        float stride        = 2.0f;
        float blendStrength = 0.5f;
    } ssrParams;

    // SSAO tuning parameters
    struct SSAOParams {
        float radius    = 1.5f;
        float intensity = 0.8f;
        float bias      = 0.05f;
        int directions  = 4;
        int steps       = 4;
    } ssaoParams;

    static std::unique_ptr<Renderer> create();
};


}  // namespace engine

#endif
