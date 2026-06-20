#ifndef RAYTRACER_MODEL_IMPORTER_H
#define RAYTRACER_MODEL_IMPORTER_H

#include "../renderer/renderer.h"
#include <string>
#include <vector>
#include <unordered_map>

namespace engine {

struct ImportedMesh {
    MeshHandle meshHandle;
    RenderMaterial material;
};

struct ImportedModel {
    std::vector<ImportedMesh> meshes;
};

// CPU-side import result (no GPU upload): geometry + decoded texture images, the
// single source of truth both the realtime upload and the offline path tracer
// consume, so a glTF model uses the same material/texture system regardless of
// renderer (ADR-0039).
struct CpuImage {
    int width = 0, height = 0, channels = 0;
    std::vector<uint8_t> pixels;   // row-major, `channels` bytes per texel
    bool valid() const { return width > 0 && height > 0 && !pixels.empty(); }
};
struct CpuMaterial {
    Vec3 baseColor{1, 1, 1};
    float metallic = 1.0f, roughness = 1.0f, opacity = 1.0f;
    Vec3 emission{0, 0, 0};
    CpuImage baseColorTex, normalTex, metallicRoughnessTex, aoTex, emissiveTex;
};
struct CpuMesh {
    RenderMesh geometry;   // positions/normals/uvs/tangents in object space
    CpuMaterial material;
};
struct CpuModel {
    std::vector<CpuMesh> meshes;
};

class ModelImporter {
public:
    static ImportedModel load(const std::string& path, Renderer& renderer);

    // Parse to CPU geometry + decoded images (no GPU). The offline tracer uses
    // this directly; load() uploads it. Empty model on failure.
    static CpuModel loadCpu(const std::string& path);

    // Parse-only check (no GPU upload) — the editor's asset import validates
    // files before copying them into the project. False fills `error`.
    static bool validate(const std::string& path, std::string& error);

    static ImportedModel* getCached(const std::string& path);

    static void clearCache();

private:
    static std::unordered_map<std::string, ImportedModel> cache;
};

// Decoded equirectangular HDR environment map: linear RGB, row-major (ADR-0016).
struct HdrImage {
    int width = 0;
    int height = 0;
    std::vector<float> pixels;  // width*height*3, linear RGB
    float avgLuminance = 0.0f;  // log-average luminance, for auto-exposure

    // Dominant light ("sun") extracted from the brightest region of the map
    // (ADR-0017 Phase 2), so a shadow-casting directional light can match the
    // sun baked into the image. hasSun stays false when no clear peak exists
    // (e.g. an overcast capture).
    bool hasSun = false;
    Vec3 sunDirection;          // toward the sun, world space
    Vec3 sunColor{1, 1, 1};     // chromaticity, normalized to luminance 1
    float sunIntensity = 0.0f;  // illuminance from the sun region (lighting units)

    bool valid() const { return width > 0 && height > 0 && !pixels.empty(); }
};

// Loads/uploads HDR environment maps. Decoding reuses the vendored stb_image
// (already built into the glTF importer TU) — `stbi_loadf` handles Radiance
// .hdr (RGBE) plus other float-decodable formats. See ADR-0016.
class EnvironmentLoader {
public:
    // Decode an equirectangular .hdr to linear RGB. Empty image on failure.
    static HdrImage loadHdr(const std::string& path);

    // Decode + upload as a float environment texture. Invalid handle on failure.
    // If outSun is given and the map has a clear dominant light, the extracted
    // direction/color/intensity overwrite it (castsShadow is left as-is).
    static TextureHandle loadEnvironmentMap(const std::string& path,
                                            Renderer& renderer,
                                            DirectionalLight* outSun = nullptr);
};

}  // namespace engine

#endif
