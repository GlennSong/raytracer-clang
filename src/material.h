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
    // World-space procedural surface from the library (brick / concrete / roof
    // tile / asphalt / ...), evaluated at the hit point like checkerboard.
    // Mirrors the viewer's RenderMaterial::Surface and applySurface() so the
    // offline tracer previews the same facades. 0 = none. Keep these ids in
    // lockstep with renderer.h Surface and the shaders' SURFACE_* constants.
    int surface = 0;
    // Baked PBR texture set (indices into Scene::textures; -1 = none), sampled
    // with a world-planar tiling UV = worldCoord * texScale. When albedoTex is
    // set the surface is textured (mirrors the viewer's albedo/normal/MR/AO
    // maps) and the analytic `surface` path is skipped.
    int albedoTex = -1, normalTex = -1, mrTex = -1, aoTex = -1, emissiveTex = -1;
    double texScale = 1.0;
    // Sample the maps with the mesh's own UVs (rec.u/rec.v) — for imported glTF
    // models that ship authored UVs + tangents. The procedural bake leaves this
    // false and uses the world-planar tiling frame instead. The two paths share
    // the same texture slots, so the source (glTF vs bake) is invisible downstream.
    bool meshUV = false;
    // Alpha-cut foliage (leaf cards): index into Scene::textures whose alpha is
    // sampled at the hit's uv; below `alphaCutoff` the surface is transparent
    // (the ray passes through). -1 = opaque. Mirrors the viewer's alpha test.
    int alphaTex = -1;
    double alphaCutoff = 0.5;

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

    // Mirrors the realtime renderer's surface model (lighting_surface.metal): albedo +
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
