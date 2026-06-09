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

}  // namespace engine

#endif
