#ifndef RAYTRACER_MATERIAL_H
#define RAYTRACER_MATERIAL_H

#include "math.h"

enum class MaterialType {
    DIFFUSE,
    METAL,
    EMISSIVE,
    GLASS
};

struct Material {
    MaterialType type;
    Vec3 albedo;
    Vec3 emission;
    double roughness;
    double ior;

    Material()
        : type(MaterialType::DIFFUSE), albedo(0.8, 0.8, 0.8),
          emission(0, 0, 0), roughness(0), ior(1.5) {}

    static Material diffuse(const Vec3& color) {
        Material mat;
        mat.type = MaterialType::DIFFUSE;
        mat.albedo = color;
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

#endif
