#ifndef RAYTRACER_SCENE_H
#define RAYTRACER_SCENE_H

#include "rt_math.h"
#include "geometry.h"
#include "material.h"
#include "kdtree.h"
#include <vector>
#include <limits>
#include <cstdint>

namespace engine {

// A delta light, sampled explicitly with shadow rays (next-event estimation).
// Types, units, and falloff mirror the realtime renderer (lighting.metal):
// directional intensity is illuminance; point/spot color*intensity is the
// illuminance at 1m with inverse-square falloff windowed to zero at `range`.
// Directional `direction` points TOWARD the light (engine convention).
struct SceneLight {
    enum class Type { Directional, Point, Spot };
    Type type = Type::Directional;
    Vec3 direction{0.5, 0.7, -0.3};   // toward the light / spot aim (toward target)
    Vec3 position;
    Vec3 color{1, 1, 1};
    double intensity = 1.0;
    double range = 25.0;
    double innerConeAngle = 0.3;      // radians (spot)
    double outerConeAngle = 0.5;
    double angularRadius = 0.03;      // radians; directional soft-shadow size
};

// Decoded equirectangular HDR environment (linear RGB), sampled on ray miss —
// the path-traced equivalent of the viewer's image-based lighting (ADR-0016).
struct EnvironmentMap {
    int width = 0;
    int height = 0;
    std::vector<float> pixels;   // width*height*3

    bool valid() const { return width > 0 && height > 0 && !pixels.empty(); }
    Vec3 sample(const Vec3& unitDir) const;   // bilinear, equirect mapping

    // When the dominant light is extracted into an explicit SceneLight
    // (mirroring ADR-0017's HDR-drives-the-sun), the disc must come out of the
    // map or sunlit surfaces count it twice. Disc texels (>= half the peak
    // luminance, the extraction's own threshold) are replaced with the average
    // of the circumsolar ring around them.
    void suppressSunDisc();
};

// Sky for rays that leave the scene. Disabled by default, which returns black —
// exactly the tracer's historical miss behavior, so existing scenes (the
// Cornell box) render unchanged. Imported levels enable it: the HDR map when
// the level has one, a horizon/zenith gradient otherwise. The sun is NOT part
// of this — it is an explicit SceneLight, sampled with shadow rays.
struct EnvironmentLight {
    bool enabled = false;
    Vec3 skyHorizon{0.5, 0.6, 0.7};
    Vec3 skyZenith{0.15, 0.3, 0.55};
    double skyIntensity = 1.0;
    EnvironmentMap map;   // when valid, replaces the gradient

    Vec3 radiance(const Vec3& unitDir) const;
};

// A CPU image whose alpha channel drives alpha-cut foliage. Row-major,
// `channels` bytes per texel; alpha is the last channel.
struct Texture {
    int width = 0, height = 0, channels = 0;
    std::vector<uint8_t> pixels;
    double sampleAlpha(double u, double v) const;   // clamped, nearest
};

// Distance (aerial-perspective) fog: surfaces fade toward `color` with distance
// from the camera, so far terrain dissolves into atmosphere — depth cue, and it
// hides the far clip / LOD seams. `density` is the exponential rate (0 = off).
struct Fog {
    bool   enabled = false;
    Vec3   color{0.6, 0.7, 0.82};
    double density = 0.0;
};

class Scene {
public:
    std::vector<Sphere> spheres;
    std::vector<Triangle> triangles;
    std::vector<Quad> quads;
    std::vector<Material> materials;
    std::vector<Texture> textures;
    KdTree kdTree;
    EnvironmentLight environment;
    Fog fog;
    std::vector<SceneLight> lights;

    int addMaterial(const Material& mat);
    int addTexture(Texture tex);
    void addSphere(const Vec3& center, double radius, int matIdx);
    void addTriangle(const Vec3& v0, const Vec3& v1, const Vec3& v2, int matIdx);
    void addTriangle(const Triangle& tri) { triangles.push_back(tri); }
    void addQuad(const Vec3& corner, const Vec3& edge1, const Vec3& edge2, int matIdx);
    void addMeshSphere(const Vec3& center, double radius, int matIdx,
                       int stacks = 16, int slices = 32);

    void buildAccelerator();
    bool intersect(const Ray& ray, double tMin, double tMax, HitRecord& rec) const;
    // Nearest hit that survives alpha-cut testing: a hit on a sub-cutoff texel
    // is skipped and the ray continues. Use for camera, bounce, and shadow rays
    // so leaf cards read as silhouettes (and cast dappled shadows).
    bool intersectVisible(const Ray& ray, double tMin, double tMax,
                          HitRecord& rec) const;
    Vec3 tracePath(const Ray& ray, int maxBounces) const;

    // Direct light at a surface point: one light sampled uniformly, BRDF
    // evaluated with the viewer's GGX model, occlusion by shadow ray. Returns
    // black with no lights (the Cornell box), keeping legacy renders identical.
    Vec3 sampleDirectLight(const Vec3& point, const Vec3& normal,
                           const Vec3& viewDir, const Vec3& albedo,
                           double metallic, double roughness) const;
};


}  // namespace engine

#endif
