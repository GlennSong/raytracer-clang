#include "scene.h"

#include <algorithm>

namespace engine {

// --- BRDF + light falloff, mirroring the realtime renderer ----------------
// These are line-for-line ports of lighting.metal so direct light, shadows,
// and materials match the viewer (ADR-0017): GGX NDF, height-correlated Smith
// visibility (with the 1/(4 NdotL NdotV) folded in), Schlick Fresnel with
// f0 = lerp(0.04, albedo, metallic), and UE-style windowed inverse-square
// falloff. Roughness is perceptual (squared into alpha, floored at 0.002).
namespace {

double luminanceOf(const Vec3& c) {
    return 0.2126 * c.x + 0.7152 * c.y + 0.0722 * c.z;
}

double distributionGGX(double NdotH, double a2) {
    double d = NdotH * NdotH * (a2 - 1.0) + 1.0;
    return a2 / std::max(PI * d * d, 1e-9);
}

double visibilitySmithGGX(double NdotV, double NdotL, double a2) {
    double gv = NdotL * std::sqrt(NdotV * NdotV * (1.0 - a2) + a2);
    double gl = NdotV * std::sqrt(NdotL * NdotL * (1.0 - a2) + a2);
    return 0.5 / std::max(gv + gl, 1e-7);
}

Vec3 fresnelSchlick(double cosTheta, const Vec3& f0) {
    double f = std::pow(std::clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
    return f0 + (Vec3(1, 1, 1) - f0) * f;
}

double distanceAttenuation(double dist, double range) {
    double ratio2 = (dist * dist) / std::max(range * range, 1e-4);
    double window = std::clamp(1.0 - ratio2 * ratio2, 0.0, 1.0);
    return window * window / std::max(dist * dist, 1e-4);
}

Vec3 f0For(const Vec3& albedo, double metallic) {
    return Vec3(0.04, 0.04, 0.04) * (1.0 - metallic) + albedo * metallic;
}

// lighting.metal's per-light evaluation: specular D*Vis*F plus an
// energy-balanced Lambert term ((1-F)(1-metallic) albedo / pi).
Vec3 evalBRDF(const Vec3& n, const Vec3& v, const Vec3& l,
              const Vec3& albedo, double metallic, double roughness) {
    double a = std::max(roughness * roughness, 0.002);
    double a2 = a * a;
    double NdotV = std::max(dot(n, v), 1e-4);
    double NdotL = std::max(dot(n, l), 0.0);
    Vec3 h = normalize(l + v);
    double NdotH = std::max(dot(n, h), 0.0);
    double VdotH = std::max(dot(v, h), 0.0);

    Vec3 F = fresnelSchlick(VdotH, f0For(albedo, metallic));
    Vec3 specular = F * (distributionGGX(NdotH, a2) *
                         visibilitySmithGGX(NdotV, NdotL, a2));
    Vec3 diffuse = (Vec3(1, 1, 1) - F) * ((1.0 - metallic) / PI) * albedo;
    return diffuse + specular;
}

void buildBasis(const Vec3& n, Vec3& t, Vec3& b) {
    Vec3 up = std::abs(n.y) < 0.99 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
    t = normalize(cross(up, n));
    b = cross(n, t);
}

// common.metal's applyCheckerboard: 1m world-space squares, dark = 0.3x.
Vec3 applyCheckerboard(const Vec3& albedo, const Vec3& worldPos) {
    int cx = static_cast<int>(std::floor(worldPos.x));
    int cz = static_cast<int>(std::floor(worldPos.z));
    return (((cx + cz) & 1) != 0) ? albedo * 0.3 : albedo;
}

}  // namespace

Vec3 EnvironmentMap::sample(const Vec3& unitDir) const {
    // Inverse of the loader's equirect mapping (model_importer.cpp):
    // dir = (sin(theta) cos(phi), cos(theta), sin(theta) sin(phi)).
    double theta = std::acos(std::clamp(unitDir.y, -1.0, 1.0));
    double phi = std::atan2(unitDir.z, unitDir.x);
    double x = (phi / (2.0 * PI) + 0.5) * width - 0.5;
    double y = (theta / PI) * height - 0.5;

    int x0 = static_cast<int>(std::floor(x));
    int y0 = static_cast<int>(std::floor(y));
    double fx = x - x0, fy = y - y0;
    auto texel = [&](int xi, int yi) {
        xi = ((xi % width) + width) % width;          // wrap in azimuth
        yi = std::clamp(yi, 0, height - 1);           // clamp at the poles
        const float* p = &pixels[(static_cast<size_t>(yi) * width + xi) * 3];
        return Vec3(p[0], p[1], p[2]);
    };
    return texel(x0, y0) * ((1 - fx) * (1 - fy)) +
           texel(x0 + 1, y0) * (fx * (1 - fy)) +
           texel(x0, y0 + 1) * ((1 - fx) * fy) +
           texel(x0 + 1, y0 + 1) * (fx * fy);
}

void EnvironmentMap::suppressSunDisc() {
    if (!valid()) return;
    const size_t count = static_cast<size_t>(width) * height;

    float maxLum = 0.0f;
    for (size_t i = 0; i < count; i++) {
        const float* p = &pixels[i * 3];
        maxLum = std::max(maxLum,
                          0.2126f * p[0] + 0.7152f * p[1] + 0.0722f * p[2]);
    }
    const float disc = 0.5f * maxLum;   // the extraction's own disc threshold

    // Fill with the circumsolar ring (bright sky just below disc level), so
    // the patched region blends instead of leaving a dark hole.
    Vec3 ring;
    size_t ringCount = 0;
    for (size_t i = 0; i < count; i++) {
        const float* p = &pixels[i * 3];
        float lum = 0.2126f * p[0] + 0.7152f * p[1] + 0.0722f * p[2];
        if (lum >= 0.1f * disc && lum < disc) {
            ring += Vec3(p[0], p[1], p[2]);
            ringCount++;
        }
    }
    if (ringCount == 0) return;   // no meaningful ring: leave the map alone
    Vec3 fill = ring / static_cast<double>(ringCount);

    for (size_t i = 0; i < count; i++) {
        float* p = &pixels[i * 3];
        float lum = 0.2126f * p[0] + 0.7152f * p[1] + 0.0722f * p[2];
        if (lum >= disc) {
            p[0] = static_cast<float>(fill.x);
            p[1] = static_cast<float>(fill.y);
            p[2] = static_cast<float>(fill.z);
        }
    }
}

Vec3 EnvironmentLight::radiance(const Vec3& unitDir) const {
    if (map.valid()) return map.sample(unitDir) * skyIntensity;
    // Gradient sky fallback: horizon -> zenith above, horizon color below
    // (ground-bounce stand-in).
    double t = std::max(unitDir.y, 0.0);
    return ((1.0 - t) * skyHorizon + t * skyZenith) * skyIntensity;
}

Vec3 Scene::sampleDirectLight(const Vec3& point, const Vec3& normal,
                              const Vec3& viewDir, const Vec3& albedo,
                              double metallic, double roughness) const {
    if (lights.empty()) return Vec3(0, 0, 0);

    // One light per sample, picked uniformly (estimator scales by count).
    size_t idx = std::min(static_cast<size_t>(randomDouble() * lights.size()),
                          lights.size() - 1);
    const SceneLight& light = lights[idx];

    Vec3 lightDir;
    double attenuation;
    double maxDist = 1e19;

    if (light.type == SceneLight::Type::Directional) {
        // Soft sun: jitter the shadow ray within the angular radius, the
        // path-traced analog of the viewer's PCF penumbra.
        Vec3 d = normalize(light.direction);
        Vec3 t, b;
        buildBasis(d, t, b);
        double r = light.angularRadius * std::sqrt(randomDouble());
        double phi = 2.0 * PI * randomDouble();
        lightDir = normalize(d + (t * std::cos(phi) + b * std::sin(phi)) *
                                     std::tan(r));
        attenuation = light.intensity;
    } else {
        Vec3 toLight = light.position - point;
        double dist = toLight.length();
        if (dist <= 1e-6) return Vec3(0, 0, 0);
        lightDir = toLight / dist;
        attenuation = light.intensity * distanceAttenuation(dist, light.range);

        if (light.type == SceneLight::Type::Spot) {
            // smoothstep(outerCos, innerCos, theta), as in the shader.
            double theta = dot(-lightDir, normalize(light.direction));
            double innerCos = std::cos(light.innerConeAngle);
            double outerCos = std::cos(light.outerConeAngle);
            double e = std::clamp((theta - outerCos) /
                                      std::max(innerCos - outerCos, 1e-6),
                                  0.0, 1.0);
            attenuation *= e * e * (3.0 - 2.0 * e);
        }
        maxDist = dist - 1e-3;
    }

    double NdotL = dot(normal, lightDir);
    if (NdotL <= 0.0 || attenuation <= 0.0) return Vec3(0, 0, 0);

    // Shadow ray; opaque occluders count, alpha-cut leaves let light through
    // (dappled shade), like the viewer's alpha-tested shadow pass.
    HitRecord shadowRec;
    if (intersectVisible(Ray(point, lightDir), 1e-3, maxDist, shadowRec))
        return Vec3(0, 0, 0);

    Vec3 brdf = evalBRDF(normal, viewDir, lightDir, albedo, metallic, roughness);
    return brdf * light.color *
           (attenuation * NdotL * static_cast<double>(lights.size()));
}

double Texture::sampleAlpha(double u, double v) const {
    if (pixels.empty() || channels < 1) return 1.0;
    int x = std::clamp(static_cast<int>(u * width), 0, width - 1);
    int y = std::clamp(static_cast<int>(v * height), 0, height - 1);
    size_t i = (static_cast<size_t>(y) * width + x) * channels + (channels - 1);
    return pixels[i] / 255.0;
}

int Scene::addMaterial(const Material& mat) {
    materials.push_back(mat);
    return static_cast<int>(materials.size()) - 1;
}

int Scene::addTexture(Texture tex) {
    textures.push_back(std::move(tex));
    return static_cast<int>(textures.size()) - 1;
}

void Scene::addSphere(const Vec3& center, double radius, int matIdx) {
    spheres.push_back(Sphere(center, radius, matIdx));
}

void Scene::addTriangle(const Vec3& v0, const Vec3& v1, const Vec3& v2, int matIdx) {
    triangles.push_back(Triangle(v0, v1, v2, matIdx));
}

void Scene::addQuad(const Vec3& corner, const Vec3& edge1, const Vec3& edge2, int matIdx) {
    quads.push_back(Quad(corner, edge1, edge2, matIdx));
}

void Scene::addMeshSphere(const Vec3& center, double radius, int matIdx,
                          int stacks, int slices) {
    for (int i = 0; i < stacks; i++) {
        double theta0 = PI * i / stacks;
        double theta1 = PI * (i + 1) / stacks;
        for (int j = 0; j < slices; j++) {
            double phi0 = 2.0 * PI * j / slices;
            double phi1 = 2.0 * PI * (j + 1) / slices;

            Vec3 p00 = center + radius * Vec3(
                std::sin(theta0) * std::cos(phi0),
                std::cos(theta0),
                std::sin(theta0) * std::sin(phi0));
            Vec3 p10 = center + radius * Vec3(
                std::sin(theta1) * std::cos(phi0),
                std::cos(theta1),
                std::sin(theta1) * std::sin(phi0));
            Vec3 p01 = center + radius * Vec3(
                std::sin(theta0) * std::cos(phi1),
                std::cos(theta0),
                std::sin(theta0) * std::sin(phi1));
            Vec3 p11 = center + radius * Vec3(
                std::sin(theta1) * std::cos(phi1),
                std::cos(theta1),
                std::sin(theta1) * std::sin(phi1));

            if (i != 0) addTriangle(p00, p10, p01, matIdx);
            if (i != stacks - 1) addTriangle(p10, p11, p01, matIdx);
        }
    }
}

void Scene::buildAccelerator() {
    if (!triangles.empty()) {
        kdTree.build(triangles);
    }
}

bool Scene::intersect(const Ray& ray, double tMin, double tMax, HitRecord& rec) const {
    HitRecord tempRec;
    bool hitAnything = false;
    double closest = tMax;

    for (const auto& sphere : spheres) {
        if (sphere.intersect(ray, tMin, closest, tempRec)) {
            hitAnything = true;
            closest = tempRec.t;
            rec = tempRec;
        }
    }

    if (!kdTree.isEmpty()) {
        if (kdTree.intersect(triangles, ray, tMin, closest, tempRec)) {
            hitAnything = true;
            closest = tempRec.t;
            rec = tempRec;
        }
    } else {
        for (const auto& tri : triangles) {
            if (tri.intersect(ray, tMin, closest, tempRec)) {
                hitAnything = true;
                closest = tempRec.t;
                rec = tempRec;
            }
        }
    }

    for (const auto& quad : quads) {
        if (quad.intersect(ray, tMin, closest, tempRec)) {
            hitAnything = true;
            closest = tempRec.t;
            rec = tempRec;
        }
    }

    return hitAnything;
}

bool Scene::intersectVisible(const Ray& ray, double tMin, double tMax,
                             HitRecord& rec) const {
    double start = tMin;
    for (int i = 0; i < 64; i++) {   // cap: bounded leaf overdraw per ray
        if (!intersect(ray, start, tMax, rec)) return false;
        const Material& mat = materials[rec.materialIndex];
        if (mat.alphaTex >= 0 &&
            textures[mat.alphaTex].sampleAlpha(rec.u, rec.v) < mat.alphaCutoff) {
            start = rec.t + 1e-4;   // transparent texel: continue along the ray
            continue;
        }
        return true;
    }
    return false;
}

Vec3 Scene::tracePath(const Ray& ray, int maxBounces) const {
    Vec3 throughput(1.0, 1.0, 1.0);
    Vec3 radiance(0.0, 0.0, 0.0);
    Ray currentRay = ray;
    double firstDist = -1.0;

    for (int bounce = 0; bounce < maxBounces; bounce++) {
        HitRecord rec;
        if (!intersectVisible(currentRay, 0.001, 1e20, rec)) {
            if (environment.enabled)
                radiance += throughput *
                            environment.radiance(normalize(currentRay.direction));
            break;
        }
        if (bounce == 0) firstDist = rec.t;

        const Material& mat = materials[rec.materialIndex];

        radiance += throughput * mat.emission;

        if (mat.type == MaterialType::EMISSIVE) {
            break;
        }

        if (mat.type == MaterialType::DIFFUSE) {
            // Delta lights can't be hit by chance, so explicit sampling here
            // never double-counts the scattered path below.
            Vec3 albedo = mat.albedo * rec.color;
            radiance += throughput *
                        sampleDirectLight(rec.point, rec.normal,
                                          -normalize(currentRay.direction),
                                          albedo, 0.0, 1.0);
            Vec3 scatterDir = randomCosineHemisphere(rec.normal);
            currentRay = Ray(rec.point, scatterDir);
            throughput = throughput * albedo;
        } else if (mat.type == MaterialType::PBR) {
            // The viewer's surface model (ADR-0017), path-traced: explicit
            // light sampling with the same GGX BRDF for direct light, then a
            // one-sample lobe pick (GGX specular vs cosine diffuse) for the
            // indirect bounce.
            Vec3 n = rec.normal;
            Vec3 v = -normalize(currentRay.direction);
            Vec3 albedo = (mat.checkerboard
                               ? applyCheckerboard(mat.albedo, rec.point)
                               : mat.albedo) *
                          rec.color;

            radiance += throughput * sampleDirectLight(rec.point, n, v, albedo,
                                                       mat.metallic,
                                                       mat.roughness);

            Vec3 f0 = f0For(albedo, mat.metallic);
            double specWeight = luminanceOf(f0);
            double diffWeight = luminanceOf(albedo) * (1.0 - mat.metallic);
            double pSpec = std::clamp(
                specWeight / std::max(specWeight + diffWeight, 1e-6), 0.05, 1.0);

            double a = std::max(mat.roughness * mat.roughness, 0.002);
            double a2 = a * a;
            if (randomDouble() < pSpec) {
                // Sample the GGX NDF for a half-vector around the normal.
                double u1 = randomDouble(), u2 = randomDouble();
                double cosTheta =
                    std::sqrt((1.0 - u1) / (1.0 + (a2 - 1.0) * u1));
                double sinTheta =
                    std::sqrt(std::max(1.0 - cosTheta * cosTheta, 0.0));
                double phi = 2.0 * PI * u2;
                Vec3 t, b;
                buildBasis(n, t, b);
                Vec3 h = normalize(t * (sinTheta * std::cos(phi)) +
                                   b * (sinTheta * std::sin(phi)) +
                                   n * cosTheta);
                Vec3 l = reflect(-v, h);
                double NdotL = dot(n, l);
                if (NdotL <= 0.0) break;
                double NdotV = std::max(dot(n, v), 1e-4);
                double NdotH = std::max(dot(n, h), 1e-6);
                double VdotH = std::max(dot(v, h), 1e-6);
                // NDF-sampling estimator: f*cos/pdf = F * Vis * 4 VdotH NdotL
                // / NdotH; clamped to tame fireflies at grazing angles.
                double w = std::min(visibilitySmithGGX(NdotV, NdotL, a2) * 4.0 *
                                        VdotH * NdotL / NdotH,
                                    8.0);
                throughput = throughput * fresnelSchlick(VdotH, f0) * (w / pSpec);
                currentRay = Ray(rec.point, l);
            } else {
                // (1-F) is folded into the direct term only — the indirect
                // diffuse keeps the plain albedo weight, like most realtime-
                // parity tracers.
                Vec3 l = randomCosineHemisphere(n);
                throughput = throughput * albedo *
                             ((1.0 - mat.metallic) / (1.0 - pSpec));
                currentRay = Ray(rec.point, l);
            }
        } else if (mat.type == MaterialType::METAL) {
            Vec3 reflected = reflect(normalize(currentRay.direction), rec.normal);
            if (mat.roughness > 0) {
                reflected = normalize(reflected + mat.roughness * randomInUnitSphere());
            }
            if (dot(reflected, rec.normal) <= 0) break;
            currentRay = Ray(rec.point, reflected);
            throughput = throughput * mat.albedo;
        } else if (mat.type == MaterialType::GLASS) {
            Vec3 unitDir = normalize(currentRay.direction);
            double etaRatio = rec.frontFace ? (1.0 / mat.ior) : mat.ior;

            double cosTheta = std::min(dot(-unitDir, rec.normal), 1.0);
            double sinTheta = std::sqrt(1.0 - cosTheta * cosTheta);

            bool cannotRefract = etaRatio * sinTheta > 1.0;
            Vec3 direction;

            if (cannotRefract || schlick(cosTheta, mat.ior) > randomDouble()) {
                direction = reflect(unitDir, rec.normal);
            } else {
                direction = refract(unitDir, rec.normal, etaRatio);
            }

            currentRay = Ray(rec.point, direction);
            throughput = throughput * mat.albedo;
        }

        // Russian roulette after 3 bounces
        if (bounce > 3) {
            double p = std::max({throughput.x, throughput.y, throughput.z});
            if (randomDouble() > p) break;
            throughput = throughput / p;
        }
    }

    // Aerial-perspective fog: fade the shaded result toward the fog color with
    // primary-ray distance (sky rays already are the horizon color, so skip them).
    if (fog.enabled && fog.density > 0.0 && firstDist > 0.0) {
        double f = 1.0 - std::exp(-fog.density * firstDist);
        radiance = radiance * (1.0 - f) + fog.color * f;
    }

    return radiance;
}

}  // namespace engine

