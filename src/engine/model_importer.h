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

class ModelImporter {
public:
    static ImportedModel load(const std::string& path, Renderer& renderer);

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
    static TextureHandle loadEnvironmentMap(const std::string& path,
                                            Renderer& renderer);
};

}  // namespace engine

#endif
