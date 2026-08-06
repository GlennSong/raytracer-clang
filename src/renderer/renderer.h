#ifndef RAYTRACER_RENDERER_H
#define RAYTRACER_RENDERER_H

#include "../rt_math.h"
#include "../handle.h"
#include "../lens_params.h"
#include <vector>
#include <memory>
#include <cstdint>
#include <string>

namespace engine {

class Window;
class XrBackend;  // engine/xr/xr_backend.h — owned by headset-capable renderers

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
    // Per-vertex tint, multiplied with the material albedo in the shader.
    // Default white = no tint, so existing meshes are unaffected. Generators
    // bake it (e.g. terrain height/slope coloration).
    Vec3 color{1, 1, 1};

    Vertex() : u(0), v(0) {}
    Vertex(const Vec3& pos, const Vec3& norm, float u = 0, float v = 0)
        : position(pos), normal(norm), u(u), v(v) {}
    Vertex(const Vec3& pos, const Vec3& norm, const Vec3& tan, float u = 0, float v = 0)
        : position(pos), normal(norm), tangent(tan), u(u), v(v) {}
};

inline BoundingSphere computeBoundingSphere(const Vertex* vertices, size_t count) {
    if (count == 0) return {};
    Vec3 center;
    Vec3 lo = vertices[0].position, hi = vertices[0].position;
    for (size_t i = 0; i < count; i++) {
        const Vec3& p = vertices[i].position;
        center += p;
        lo = Vec3(std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z));
        hi = Vec3(std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z));
    }
    center /= static_cast<Real>(count);
    Real maxDistSq = 0;
    for (size_t i = 0; i < count; i++) {
        Real dSq = (vertices[i].position - center).lengthSquared();
        if (dSq > maxDistSq) maxDistSq = dSq;
    }
    return {center, std::sqrt(maxDistSq), lo, hi};
}

struct RenderMaterial {
    Vec3 albedo;
    float metallic;
    float roughness;
    float opacity;
    Vec3 emission;
    uint32_t flags = 0;

    static constexpr uint32_t FLAG_CHECKERBOARD = 1;
    // Alpha-cut foliage: discard fragments where the albedo map's alpha is
    // below the threshold, so leaf cards keep crisp silhouettes in the opaque
    // pass (depth-correct, no transparency sorting). Needs an albedoMap.
    static constexpr uint32_t FLAG_ALPHA_TEST = 2;
    // Wind sway: the vertex shader displaces this mesh by a per-frame wind
    // function, weighted by height above the instance origin (base planted, tips
    // move). For grass/foliage; applied on the instanced draw path.
    static constexpr uint32_t FLAG_WIND = 4;
    // Debug overlay: draw ON TOP of everything (depth test always passes, no depth
    // write) and unlit/emissive, for gizmos like the agent footprints/trajectory
    // vectors that must stay visible through world geometry. Backend-honoured.
    static constexpr uint32_t FLAG_OVERLAY = 8;
    // Two-sided: draw both faces instead of culling backfaces. For SURFACES that
    // are genuinely a thin sheet with no inside — glass panes, foliage cards.
    // Without it such a mesh has to be emitted TWICE at identical coordinates to
    // be visible from either side, and for a transparent pane that is actively
    // wrong: the transparent pass does not write depth, so both coincident faces
    // blend and the pane comes out far more opaque than authored (car glass at
    // 0.32 read as 0.54 per pane, and ~79% through a whole car). One two-sided
    // pane blends once. Backend-honoured via cull mode, so it costs a state
    // change per batch, not per triangle.
    static constexpr uint32_t FLAG_TWO_SIDED = 16;

    // World-space procedural surface library (applySurface in surfaces.metal /
    // scene.cpp): an analytic material — brick, concrete, roof tiles, asphalt,
    // ... — chosen by an id packed into bits 8..15 of `flags`, so it rides the
    // existing material path with no texture maps. 0 = none. Keep the ids in
    // lockstep with material.h's Surface and the shaders' SURFACE_* constants.
    enum class Surface : uint32_t {
        None = 0, Brick, Concrete, Stucco, RoofTile, RoofShingle,
        CorrugatedMetal, Asphalt, Pavement, Cobblestone, WoodSiding,
        // Road carriageway lane paint composited from road-local mesh UV (u = lateral
        // [-1,1] offset by +2 so non-carriageway u=0 is excluded; v = arc-length for
        // dashes) instead of baked stripe geometry (ADR-0044 / road-geometry-plan
        // Problem 3): conforms to terrain for free, no z-fight, crisp at any distance.
        RoadMarkings,
        // Water body (ocean/river/pond). Depth is baked into mesh UV.x
        // (seaLevel - floor) and shore distance into UV.y, so the shader grades
        // colour by depth and lays a foam band at the waterline without a depth
        // buffer; waves + foam animate on windTime. Rides the material's low
        // roughness + <1 opacity for SSR reflection and fresnel transparency.
        Water,
        // Natural ground (terrain). The biome colour is already baked into the
        // vertex colour; this surface adds procedural MICRO-RELIEF: a slope-scaled
        // normal perturbation + roughness variation (rougher/bumpier on steep rock,
        // smoother on flat sand/snow) so the ground isn't a flat-shaded plane.
        // Albedo grain only; the normal/roughness work is surfaceReliefTerrain
        // (same file, surface_terrain.metal).
        TerrainGround,
        // Rooftop HVAC kit (device: "procedural recipe … UV wrap around the
        // HVAC unit"). Baked PBR sets like the facade surfaces:
        //  VentGrille   — capsule intake holes: matte dark holes punched in a
        //                 metallic casing, normal-mapped rims (long faces).
        //  UtilityPanel — panelled metal box with seams/rivets/hatch (short faces).
        //  FanTop       — radial fan blades under a hub, smooth casing ring
        //                 (cowl top; the disc bakes its own centred UVs).
        VentGrille, UtilityPanel, FanTop,
    };
    static constexpr uint32_t SURFACE_SHIFT = 8;
    static constexpr uint32_t SURFACE_MASK = 0xFF00u;
    static constexpr uint32_t surfaceBits(Surface s) {
        return static_cast<uint32_t>(s) << SURFACE_SHIFT;
    }
    Surface surface() const {
        return static_cast<Surface>((flags & SURFACE_MASK) >> SURFACE_SHIFT);
    }
    void setSurface(Surface s) {
        flags = (flags & ~SURFACE_MASK) | surfaceBits(s);
    }

    TextureHandle albedoMap;
    TextureHandle normalMap;
    TextureHandle metallicRoughnessMap;
    TextureHandle emissiveMap;
    TextureHandle aoMap;

    RenderMaterial()
        : albedo(0.8, 0.8, 0.8), metallic(0.0), roughness(0.5),
          opacity(1.0), emission(0, 0, 0), flags(0) {}
};

// Map a level-format surface name ("brick", "concrete", "asphalt", ...) to its
// Surface id. Shared by both loaders so the offline tracer and the viewer agree.
// Returns Surface::None for an unknown name.
inline RenderMaterial::Surface surfaceFromName(const std::string& s) {
    using S = RenderMaterial::Surface;
    if (s == "brick") return S::Brick;
    if (s == "concrete") return S::Concrete;
    if (s == "stucco" || s == "plaster") return S::Stucco;
    if (s == "rooftile" || s == "roof_tile" || s == "tile") return S::RoofTile;
    if (s == "shingle" || s == "roofshingle") return S::RoofShingle;
    if (s == "corrugated" || s == "corrugatedmetal" || s == "metal") return S::CorrugatedMetal;
    if (s == "vent" || s == "ventgrille") return S::VentGrille;
    if (s == "utilitypanel") return S::UtilityPanel;
    if (s == "fantop") return S::FanTop;
    if (s == "asphalt") return S::Asphalt;
    if (s == "pavement" || s == "sidewalk") return S::Pavement;
    if (s == "cobblestone" || s == "cobble") return S::Cobblestone;
    if (s == "wood" || s == "woodsiding" || s == "siding") return S::WoodSiding;
    if (s == "roadmarkings" || s == "road_markings" || s == "lanes") return S::RoadMarkings;
    if (s == "water" || s == "ocean" || s == "river") return S::Water;
    if (s == "terrain" || s == "ground" || s == "terrainground") return S::TerrainGround;
    return S::None;
}

// Canonical name for a surface, so an authored material's "surface" round-trips
// through save/load (the inverse of surfaceFromName).
inline const char* surfaceName(RenderMaterial::Surface s) {
    using S = RenderMaterial::Surface;
    switch (s) {
        case S::Brick:           return "brick";
        case S::Concrete:        return "concrete";
        case S::Stucco:          return "stucco";
        case S::RoofTile:        return "rooftile";
        case S::RoofShingle:     return "shingle";
        case S::CorrugatedMetal: return "corrugated";
        case S::VentGrille: return "vent";
        case S::UtilityPanel: return "utilitypanel";
        case S::FanTop: return "fantop";
        case S::Asphalt:         return "asphalt";
        case S::Pavement:        return "pavement";
        case S::Cobblestone:     return "cobblestone";
        case S::WoodSiding:      return "wood";
        case S::RoadMarkings:    return "roadmarkings";
        case S::Water:           return "water";
        case S::TerrainGround:   return "terrain";
        default:                 return "";
    }
}

struct RenderMesh {
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    int materialIndex;

    RenderMesh() : materialIndex(0) {}
};

// Light units (ADR-0017 Phase 1): for the sun, color * intensity is the
// illuminance arriving from its direction. For point/spot lights it is the
// illuminance at 1 m; falloff is inverse-square, windowed to zero at `range`.
struct PointLight {
    Vec3 position;
    Vec3 color;
    float intensity;
    float range = 25.0f;    // falloff window radius (world units)

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
    float range = 25.0f;    // falloff window radius (world units)
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
    float bias = 0.005f;        // rasterization depth bias (shadow pass encoder)
    float normalBias = 0.02f;   // world-space lookup offset along the normal
    float pcfRadius = 1.0f;     // PCF tap spread in shadow-map texels
    int resolution = 2048;
    bool enabled = true;
    // Per-level overrides for the cascade fit (ShadowParams). 0 = unset (keep the
    // settings-driven default). A large world (CDLOD terrain) needs a much longer
    // shadow distance than the 150 m default, so the level can raise it.
    float distance = 0.0f;
    int   cascadeCount = 0;
};

// Artistic shadow response (ADR-0017 Phase 2). Shadow visibility becomes a
// color: occluded regions lerp toward `tint` (a deep blue reads richer than
// black), `strength` scales how dark full shadow gets on direct light, and
// `ambientStrength` is how much shadow also occludes the environment/IBL
// terms — the knob that makes shadows read under an HDR sky, whose energy
// otherwise arrives entirely through the unshadowed ambient path.
struct ShadowArtistic {
    float strength = 1.0f;
    Vec3 tint{0, 0, 0};
    float ambientStrength = 0.5f;
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

    // Procedural FBM clouds (ADR-0016 step 3). A sky-dome visual overlay, never
    // baked into reflection probes. cloudTime is the drift phase in seconds.
    bool  cloudsEnabled  = true;
    float cloudCoverage  = 0.5f;   // higher = more open sky (threshold)
    float cloudDensity   = 1.0f;   // overlay opacity
    float cloudScale     = 1.5f;   // noise frequency
    float cloudTime      = 0.0f;   // animation phase (advanced by the cycle)
};

// Physically-based scattering sky (cinematic-sky phase). Opt-in per level via
// "environment": {"sky": {"model": "scattering", ...}}. The backend computes a
// Hillaire-style transmittance LUT + sky-view LUT (recomputed only when the sun
// or these params change), the skybox samples them, IBL is baked from the same
// sky, and the lit pass swaps the exp height fog for per-channel Rayleigh+Mie
// aerial perspective. The sun disc/color follow SceneLighting::sun. Defaults are
// an Earth-like clear day; `turbidity` is the one-knob haze control.
struct SkyScatteringParams {
    bool  enabled = false;
    float planetRadius = 6360000.0f;      // ground radius (world meters)
    float atmosphereHeight = 100000.0f;   // shell thickness above ground (m)
    Vec3  rayleighBeta{5.802e-6f, 13.558e-6f, 33.1e-6f};   // scatter /m (λ⁻⁴ blue)
    float rayleighScaleHeight = 8000.0f;
    float mieBeta = 3.996e-6f;            // scatter /m (extinction = 1.11x)
    float mieScaleHeight = 1200.0f;
    float mieG = 0.8f;                    // forward-scatter lobe (sun haze glow)
    float turbidity = 1.0f;               // scales the Mie coefficients (haze)
    Vec3  groundAlbedo{0.30f, 0.30f, 0.30f};
    float brightness = 1.0f;              // artistic scale on the sky radiance
    float multiScatter = 1.0f;            // strength of the multi-scatter approx
    float aerialDensity = 1.0f;           // aerial-perspective distance scale
    float sunAngularRadius = 0.0047f;     // radians (~0.27°, the real sun)
};

// Volumetric cloud layer (cinematic-sky phase). Opt-in per level via
// "environment": {"clouds": {...}}. A slab [bottom, top] of raymarched clouds
// rendered at half resolution after the scene (depth-occluded) and upsampled
// bilaterally; replaces the 2D FBM sky overlay while active. Altitudes are
// world meters; `wind` drifts the field along +X in m/s.
struct VolumetricCloudParams {
    bool  enabled = false;
    float coverage = 0.35f;      // 0 = clear, 1 = overcast
    float bottom = 900.0f;
    float top = 1600.0f;
    // Extinction per metre inside the slab. 0.55 was not a volume coefficient —
    // at a ~17.5 m step it puts the per-step optical depth near 10, so the very
    // first non-empty sample saturated the ray and the march broke out of its
    // loop after ONE iteration. Visible brightness was then whatever density
    // that single jittered sample happened to hit, which is where the per-pixel
    // sparkle came from: neighbouring pixels sampled different points and
    // differed severalfold. At 0.05 a step is ~0.4-0.9 optical depths, so the
    // ray actually integrates across many samples and the noise averages out.
    // (metropolis_sky.json already authored 0.05 by hand; this makes the
    // default agree with the one level that had been tuned.)
    float density = 0.05f;       // extinction scale inside the slab
    float noiseScale = 0.0011f;  // base noise frequency (1/m)
    float wind = 12.0f;
    int   steps = 40;            // view-march steps (perf/quality)
    int   lightSteps = 6;        // sun-march steps per lit sample
    float phaseG = 0.45f;        // Henyey-Greenstein silver-lining strength
    float farDistance = 30000.0f;
    float ambient = 0.5f;        // sky ambient reaching cloud interiors
    // Perlin-Worley detail erosion: how deep the 32^3 worley detail texture
    // eats into cloud edges (0 = solid billows, ~0.6 = ragged wisps). Matches
    // the 0.35 the old inline-fbm erosion hardcoded.
    float detailStrength = 0.35f;
};

// Aerial-perspective fog (ADR-0016 environment). Lit color lerps toward `color`
// by 1-exp(-density*dist) from the camera, so distant terrain dissolves into
// atmosphere — a depth cue that also hides the far clip / LOD seams. Mirrors the
// offline tracer's Scene::fog so both render paths match. density 0 = off.
struct FogParams {
    bool  enabled = false;
    Vec3  color{0.6, 0.7, 0.82};
    float density = 0.0f;
    // > 0: density decays with altitude (exp(-heightFalloff * y)) — low-lying
    // haze that aerial cameras see through instead of a distance white-out.
    float heightFalloff = 0.0f;
};

struct SceneLighting {
    DirectionalLight sun;
    std::vector<PointLight> pointLights;
    std::vector<SpotLight> spotLights;
    ShadowConfig shadow;
    ShadowArtistic shadowArtistic;
    ProceduralSky sky;
    FogParams fog;
    SkyScatteringParams skyScattering;      // cinematic-sky opt-in (per level)
    VolumetricCloudParams volumetricClouds; // cloud-slab opt-in (per level)
    float exposure = 1.0f;
    float ambientMultiplier = 0.3f;
    Vec3 ambientTint{1, 1, 1};   // grades the ambient/irradiance term only
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
    uint32_t trianglesDrawn = 0;   // color pass only (excludes shadow casters)
    // Instance-capacity overflow, per frame. Non-zero shadow/foliage counts mean
    // geometry was DROPPED this frame; instanceOverflow counts instances that
    // fell back to per-draw submission (correct image, draw-call cliff). Raise
    // the capacities (setInstanceCapacities) when these are non-zero.
    uint32_t instanceOverflow = 0;
    uint32_t shadowOverflow = 0;
    uint32_t foliageOverflow = 0;
    // Casters actually submitted to the shadow pass, summed over all cascades,
    // and terrain nodes likewise. The shadow pass re-submits the colour pass's
    // geometry per cascade and NOTHING counted it, so "shadows cost ~3x the
    // colour pass" was an inference rather than a measurement.
    uint32_t shadowCasters = 0;
    uint32_t shadowTerrainNodes = 0;
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

    // Physical lens parameters of the view (docs/virtual-camera-plan.md).
    // Backends approximate them in post (DOF, distortion, CA, vignette); the
    // defaults are visually inert, so views that never touch this lose nothing.
    LensParams lens;

    CameraState()
        : position(0, 0, 3), target(0, 0, 0), up(0, 1, 0),
          fovDegrees(60.0f), orthoHeight(10.0f), aspectRatio(1.0f),
          nearPlane(0.1f), farPlane(1000.0f) {}
};

// One explicit XR view: matrices the renderer uses AS-IS. CameraState cannot
// express a headset's off-axis eye projection, so XR hands the renderer the
// finished matrices instead of a look-at description.
struct RenderViewDesc {
    Mat4 view;            // world -> eye
    Mat4 projection;      // reverse-Z, off-axis, GPU-ready
    int targetIndex = 0;  // color slice / texture index
};

// Per-frame XR render info, produced by XrCameraSystem (engine/systems) and
// handed to the renderer via Renderer::setXrViews. Contract: a compositor-
// owned backend MAY refine each view's ORIGIN-space eye pose/projection with
// a same-frame late latch (fresher anchor, exact current-drawable projection);
// it must NEVER touch worldFromOrigin — the locomotion base belongs to the
// game. That split is what makes "the renderer second-guessed the camera"
// structurally impossible.
struct XrRenderInfo {
    bool active = false;
    Mat4 worldFromOrigin;  // locomotion base: tracking origin in world space
    int viewCount = 0;
    RenderViewDesc views[2];
};

// Planetary atmosphere (procedural-planet-plan P3). A single-scattering Rayleigh+Mie
// glow the backend raymarches as a fullscreen pass and adds over the HDR scene. The
// values mirror the tested CPU reference (engine/procgen/atmosphere.cpp); build them
// with atmosphereParamsFor() for an Earth-like body. Sun direction/colour come from
// the scene's directional light, so they are not carried here. Backends without the
// pass ignore it (default no-op) — Metal is the first to implement it.
struct AtmosphereRenderParams {
    bool  enabled = false;
    Vec3  planetCenter{0, 0, 0};
    float planetRadius = 20.0f;
    float atmosphereRadius = 21.0f;
    Vec3  rayleighCoeff{20.0f, 47.0f, 115.0f};   // per length (already ∝ λ⁻⁴)
    float mieCoeff = 42.0f;
    float mieG = 0.76f;
    float rayleighScaleHeight = 0.25f;           // absolute world units
    float mieScaleHeight = 0.07f;
    float sunIntensity = 22.0f;
    int   viewSamples = 24;
    int   lightSamples = 8;
};

// Earth-like atmosphere for a body of `radius` at `center` (mirrors the CPU
// atmosphereEarth preset, scaled to the render-space radius). `thicknessFrac` is the
// shell height as a fraction of the planet radius (0.08 ≈ a clearly visible limb
// halo); `density` scales the scattering coefficients (brighter/deeper blue); and
// `sunIntensity` the overall glow strength. The level's "atmosphere" block overrides
// all three so the halo can be dialled without a recompile.
inline AtmosphereRenderParams atmosphereParamsFor(const Vec3& center, float radius,
                                                  float thicknessFrac = 0.08f,
                                                  float density = 1.0f,
                                                  float sunIntensity = 32.0f) {
    AtmosphereRenderParams a;
    a.enabled = true;
    a.planetCenter = center;
    a.planetRadius = radius;
    a.atmosphereRadius = radius * (1.0f + thicknessFrac);
    float thickness = a.atmosphereRadius - a.planetRadius;
    a.rayleighScaleHeight = thickness * 0.25f;
    a.mieScaleHeight = thickness * 0.07f;
    a.rayleighCoeff = Vec3(20.0f, 47.0f, 115.0f) * (density / radius);
    a.mieCoeff = 42.0f * density / radius;
    a.mieG = 0.76f;
    a.sunIntensity = sunIntensity;
    return a;
}

class Renderer {
public:
    virtual ~Renderer() = default;

    // Hand the windowing seam to backends that create their own surface from it
    // rather than from the opaque native handle. Vulkan needs this: the native
    // handle is null on Linux (window.cpp), so the backend asks the Window for a
    // surface via Window::createVulkanSurface (ADR-0057). The Application calls
    // this before initialize(); backends that bind via the native handle (Metal)
    // ignore it. Default no-op so NullRenderer and other backends stay valid.
    virtual void setWindow(Window* /*window*/) {}

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

    // Per-frame instance buffer capacities (general/shadow/foliage). A big level
    // (8 km city) raises these at load; small levels keep the lean defaults.
    // Values below the backend's defaults are clamped up — shrinking buys
    // nothing and risks overflow. Safe between frames; no-op by default.
    virtual void setInstanceCapacities(uint32_t /*instances*/, uint32_t /*shadow*/,
                                       uint32_t /*foliage*/) {}

    virtual void beginFrame() = 0;
    virtual void setCamera(const CameraState& camera) = 0;

    // XR (engine/xr/): the headset backend this renderer owns, or nullptr —
    // the visionOS Metal backend returns its CompositorServices adapter; all
    // other backends (and macOS Metal) inherit nullptr and XR stays inert.
    virtual XrBackend* xrBackend() { return nullptr; }

    // Explicit per-view matrices for XR frames. Called by RenderSystem after
    // setCamera when XR is active; setCamera still runs every frame and
    // remains the mono truth (culling, audio, effects). Default: ignore.
    virtual void setXrViews(const XrRenderInfo& /*info*/) {}

    // The GAMEPLAY camera's position — the locomotion-base hint for
    // head-tracked surfaces. XrCameraSystem calls this each frame BEFORE it
    // rewrites the shared camera to follow the head; the base-follow must
    // track gameplay movement (teleports, vehicles), never head motion.
    virtual void setXrBaseHint(const Vec3& /*worldPosition*/) {}

    virtual void setLights(const SceneLighting& lighting) = 0;
    virtual void drawMesh(MeshHandle handle, const Mat4& transform,
                          const RenderMaterial& material) = 0;
    // Draw many instances of one mesh (world-space transforms, one material for
    // all). Default loops drawMesh: backends that batch by mesh handle (the Metal
    // backend) already coalesce these into a single instanced draw, so the win
    // here is CPU-side (one InstanceGroup vs. thousands of entities). A backend
    // may override for a more direct path. (std::span is C++20; we are C++17.)
    virtual void drawMeshInstanced(MeshHandle handle,
                                   const std::vector<Mat4>& transforms,
                                   const RenderMaterial& material) {
        for (const Mat4& m : transforms) drawMesh(handle, m, material);
    }
    virtual void setReflectionProbes(const std::vector<ReflectionProbe>& /*probes*/) {}

    // Set (or clear, via `enabled = false`) the planetary atmosphere glow pass
    // (procedural-planet-plan P3). Backends without the pass ignore it. No-op default.
    virtual void setAtmosphere(const AtmosphereRenderParams& /*atmosphere*/) {}

    // Draw one CDLOD terrain node (ADR-0036): a world-space mesh (identity
    // transform) whose vertices carry a baked morph target in Vertex::tangent.
    // morphStart/morphEnd is the camera-distance band over which the terrain
    // vertex shader morphs the node toward the coarser level. The default falls
    // back to a plain draw (no morph) so backends without a terrain pipeline still
    // render the node.
    virtual void drawTerrain(MeshHandle handle, const RenderMaterial& material,
                             float morphStart, float morphEnd) {
        (void)morphStart;
        (void)morphEnd;
        drawMesh(handle, Mat4(), material);
    }

    virtual void endFrame() = 0;

    // Debug-UI (Dear ImGui) backend hooks — see ADR-0011. No-ops unless a
    // backend implements them and the build defines RT_ENABLE_IMGUI; engine
    // code never sees ImGui types. The per-frame new-frame/submit are handled
    // internally inside beginFrame()/endFrame(); only setup/teardown are here.
    // windowHandle is the same opaque native pointer passed to initialize().
    virtual void initDebugUi(void* /*windowHandle*/) {}
    virtual void shutdownDebugUi() {}

    // Runtime toggles for post-processing effects (debug/tuning)
    // XR world scale: how many world units the user traverses per real
    // meter. >1 makes the user FEEL larger (wider virtual IPD, longer
    // strides — the world reads smaller); 1 = life-size. Head-tracking
    // backends apply it to head/eye translations; ignored elsewhere.
    float xrWorldScale = 1.0f;

    bool ssaoEnabled = true;
    bool ssrEnabled = true;
    bool reflectionProbesEnabled = true;

    // Cinematic sky runtime gates (AND-ed with the level's opt-in params, so a
    // HUD toggle can kill either without reloading). cloudStepsOverride > 0
    // replaces the level's view-march step count (quality/perf knob).
    bool skyScatteringEnabled = true;
    bool volumetricCloudsEnabled = true;
    int  cloudStepsOverride = 0;

    // Resolution scale for the screen-space effect buffers (SSAO + SSR), a
    // fraction of the framebuffer. These effects are low/medium frequency, so
    // rendering them smaller and upscaling is a large perf win at little visual
    // cost. 1.0 = full res; 0.5 (default) = quarter the pixels. Backends that
    // support it clamp to a sane range and recreate the buffers when it changes.
    float postEffectScale = 0.5f;

    // HDR environment map (baked cubemap + IBL). When off, the renderer ignores a
    // bound HDR and falls back to the procedural sky + analytic IBL. Levels with no
    // "hdr" key clear it on load; this is the live override.
    bool environmentMapEnabled = true;

    // Depth prepass for alpha-cut foliage (perf). Foliage uses discard, which
    // forces late depth testing — every overlapping leaf card runs the full lit
    // shader (huge overdraw close-up). When on, foliage is drawn twice: a cheap
    // depth-only prepass that writes the nearest leaf depth, then the lit pass
    // with early depth tests + an Equal/no-write depth state, so only the
    // front-most leaf per pixel is shaded. Off = legacy single-pass foliage.
    bool depthPrepassEnabled = true;

    // Vegetation draw-distance override (live tuning, to balance against fog).
    // 0 = use each scatter group's level-set drawDistance; >0 overrides all
    // groups this session. Applied by RenderSystem's per-instance cull.
    float vegetationDrawDistance = 0.0f;

    // Debug LAYER visibility mask (device: "turn layers on/off ... to debug
    // individual layers ... see the terrain underneath"). A Renderable /
    // InstanceGroup tagged with a RenderLayer bit is SKIPPED by RenderSystem
    // when that bit is set here. Layer 0 (untagged — terrain, sky, props) is
    // always drawn. Runtime-only; never serialized.
    uint32_t hiddenLayers = 0;

    // Debug visualization: 0=normal, 1=AO only, 2=SSR only, 3=depth, 4=normals,
    // 5=shadow, 6=albedo, 7=facing (green=front / red=back), 8=shadow cascades
    int debugView = 0;

    // Wireframe: 0=off, 1=wireframe only, 2=wireframe overlaid on the shaded image
    int wireframe = 0;
    // Colour of the wireframe lines (both modes). Lines bypass lighting and draw
    // this flat colour, so they read clearly against the shaded surface.
    Vec3 wireframeColor{0.1f, 1.0f, 0.5f};

    // Lens effects of the active view's LensParams (docs/virtual-camera-plan.md):
    // a final image-space warp pass (distortion + chromatic aberration +
    // vignette) and a depth-based depth-of-field pass. DOF defaults off until
    // verified on-device.
    bool lensEffectsEnabled = true;
    bool dofEnabled = false;

    // Bloom
    bool bloomEnabled = true;
    struct BloomParams {
        float threshold = 1.0f;
        float knee      = 0.5f;
        float intensity = 0.3f;
    } bloomParams;

    // Tone mapping + color grade (the composite "Look" + view transform). The
    // grade (contrast/saturation) runs in scene-linear before the tone map, so it
    // is display-agnostic (HDR-ready). Tone map: 0 = ACES, 1 = AgX.
    int tonemapOperator = 0;
    struct GradeParams {
        float contrast   = 1.0f;   // log-space contrast around middle grey
        float saturation = 1.0f;   // saturation around luma
    } gradeParams;

    // Compact stats HUD visible during gameplay
    bool showHud = false;

    // Frame rate cap (0 = uncapped, otherwise target FPS like 30 or 60)
    int targetFps = 0;

    // Log-average luminance of the bound HDR environment (0 if none / procedural).
    // Set by EnvironmentLoader at load; drives the debug "Auto Exposure" button.
    float environmentAvgLuminance = 0.0f;

    // SSR tuning parameters
    struct SSRParams {
        float maxRayDist    = 20.0f;
        float thickness     = 0.3f;
        float thicknessFar  = 2.0f;
        float stride        = 2.0f;
        float blendStrength = 0.5f;
        float maxRoughness  = 0.6f;   // surfaces rougher than this don't reflect
    } ssrParams;

    // SSAO tuning parameters
    struct SSAOParams {
        float radius    = 1.5f;
        float intensity = 0.8f;
        float bias      = 0.05f;
        float aoFloor   = 0.15f;   // darkest the composite AO multiply can go
        int directions  = 4;       // angular samples -> less banding (live-tunable)
        int steps       = 5;       // radial samples -> smoother (live-tunable)
        float temporal  = 0.9f;    // history blend weight: higher = steadier but
                                   // more ghosting in motion; 0 disables temporal AO
    } ssaoParams;

    // Cascaded shadow maps (sun). The view frustum is split into `cascadeCount`
    // ranges out to `distance`; each gets its own shadow-map slice fit tightly to
    // it, so near geometry is crisp and far geometry stays covered (no slice).
    struct ShadowParams {
        float distance    = 150.0f;  // furthest range that receives sun shadows (m)
        int   cascadeCount = 3;      // 1..RT_MAX_CASCADES
        float splitLambda  = 0.6f;   // 0 = uniform splits, 1 = logarithmic
    } shadowParams;

    static std::unique_ptr<Renderer> create();
};


}  // namespace engine

#endif
