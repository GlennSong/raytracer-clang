#include "scene.h"

namespace engine {

Vec3 EnvironmentLight::radiance(const Vec3& unitDir) const {
    // Simple gradient sky: horizon -> zenith on the upward half, horizon color
    // mirrored below (ground bounce stand-in).
    double t = std::max(unitDir.y, 0.0);
    Vec3 sky = ((1.0 - t) * skyHorizon + t * skyZenith) * skyIntensity;

    if (sunIntensity > 0.0) {
        // The disc's radiance is the sun's total strength spread over its
        // solid angle, so ground-plane irradiance tracks sunIntensity
        // regardless of the padded disc size.
        double cosSun = dot(unitDir, -normalize(sunDirection));
        if (cosSun > std::cos(sunAngularRadius)) {
            double solidAngle = PI * sunAngularRadius * sunAngularRadius;
            sky += sunColor * (sunIntensity / solidAngle);
        }
    }
    return sky;
}

int Scene::addMaterial(const Material& mat) {
    materials.push_back(mat);
    return static_cast<int>(materials.size()) - 1;
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
        if (kdTree.intersect(ray, tMin, closest, tempRec)) {
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

Vec3 Scene::tracePath(const Ray& ray, int maxBounces) const {
    Vec3 throughput(1.0, 1.0, 1.0);
    Vec3 radiance(0.0, 0.0, 0.0);
    Ray currentRay = ray;

    for (int bounce = 0; bounce < maxBounces; bounce++) {
        HitRecord rec;
        if (!intersect(currentRay, 0.001, 1e20, rec)) {
            if (environment.enabled)
                radiance += throughput *
                            environment.radiance(normalize(currentRay.direction));
            break;
        }

        const Material& mat = materials[rec.materialIndex];

        radiance += throughput * mat.emission;

        if (mat.type == MaterialType::EMISSIVE) {
            break;
        }

        if (mat.type == MaterialType::DIFFUSE) {
            Vec3 scatterDir = randomCosineHemisphere(rec.normal);
            currentRay = Ray(rec.point, scatterDir);
            throughput = throughput * mat.albedo;
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

    return radiance;
}

}  // namespace engine

