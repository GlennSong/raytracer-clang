#ifndef RAYTRACER_ENGINE_MESH_UPLOADER_H
#define RAYTRACER_ENGINE_MESH_UPLOADER_H

#include "../renderer/renderer.h"   // RenderMesh, MeshHandle, BoundingSphere

namespace engine {

// The narrow slice of the renderer the AssetManager drives — just enough to own
// GPU mesh lifetime. The real Renderer satisfies it through the adapter below;
// tests implement a tiny stub. Keeping the manager off the full (many-method,
// backend-bound) Renderer is what makes it headless-testable.
class MeshUploader {
public:
    virtual ~MeshUploader() = default;
    virtual MeshHandle uploadMesh(const RenderMesh& mesh) = 0;
    // Two-phase upload (renderer.h, PreparedMesh): prepare on any thread, upload on the render thread.
    virtual PreparedMesh prepareMesh(RenderMesh&& mesh) const { return prepareRawMesh(std::move(mesh)); }
    virtual MeshHandle uploadPrepared(PreparedMesh&& mesh) { return uploadMesh(mesh.raw); }
    virtual void removeMesh(MeshHandle handle) = 0;
    virtual BoundingSphere getMeshBounds(MeshHandle handle) const = 0;
};

// Production wiring: forward the seam to a real Renderer (whose mesh API has the
// identical signatures). Lets Application hand the AssetManager a renderer
// without the manager ever depending on the concrete backend.
class RendererMeshUploader : public MeshUploader {
public:
    explicit RendererMeshUploader(Renderer& renderer) : renderer_(renderer) {}
    MeshHandle uploadMesh(const RenderMesh& mesh) override {
        return renderer_.uploadMesh(mesh);
    }
    PreparedMesh prepareMesh(RenderMesh&& mesh) const override {
        return static_cast<const Renderer&>(renderer_).prepareMesh(std::move(mesh));
    }
    MeshHandle uploadPrepared(PreparedMesh&& mesh) override { return renderer_.uploadPrepared(std::move(mesh)); }
    void removeMesh(MeshHandle handle) override { renderer_.removeMesh(handle); }
    BoundingSphere getMeshBounds(MeshHandle handle) const override {
        return renderer_.getMeshBounds(handle);
    }

private:
    Renderer& renderer_;
};

}  // namespace engine

#endif
