#ifndef RAYTRACER_MATERIAL_H
#define RAYTRACER_MATERIAL_H

#include "rt_math.h"

namespace engine {

enum class MaterialType {
    DIFFUSE,
    METAL,
    EMISSIVE,
    GLASS,
    PBR     // metallic/roughness, evaluated with the viewer's GGX model
};

struct Material {
    MaterialType type;
    Vec3 albedo;
    Vec3 emission;
    double roughness;
    double ior;
    double metallic;
    bool checkerboard;   // world-space albedo checker (viewer material flag)

    Material()
        : type(MaterialType::DIFFUSE), albedo(0.8, 0.8, 0.8),
          emission(0, 0, 0), roughness(0), ior(1.5), metallic(0),
          checkerboard(false) {}

    static Material diffuse(const Vec3& color) {
        Material mat;
        mat.type = MaterialType::DIFFUSE;
        mat.albedo = color;
        return mat;
    }

    // Mirrors the realtime renderer's surface model (lighting.metal): albedo +
    // perceptual roughness + metallic, f0 = lerp(0.04, albedo, metallic).
    static Material pbr(const Vec3& color, double metallic, double roughness) {
        Material mat;
        mat.type = MaterialType::PBR;
        mat.albedo = color;
        mat.metallic = metallic;
        mat.roughness = roughness;
        return mat;
    }

    static Material metal(const Vec3& color, double roughness = 0.0) {
        Material mat;
        mat.type = MaterialType::METAL;
        mat.albedo = color;
        mat.roughness = roughness;
        return mat;
    }

    static Material emissive(const Vec3& color, double strength = 1.0) {
        Material mat;
        mat.type = MaterialType::EMISSIVE;
        mat.emission = color * strength;
        mat.albedo = Vec3(0, 0, 0);
        return mat;
    }

    static Material glass(double ior = 1.5) {
        Material mat;
        mat.type = MaterialType::GLASS;
        mat.albedo = Vec3(1.0, 1.0, 1.0);
        mat.ior = ior;
        return mat;
    }
};


}  // namespace engine

#endif
