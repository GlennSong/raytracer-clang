#ifndef RAYTRACER_METAL_RENDERER_H
#define RAYTRACER_METAL_RENDERER_H

#include "../renderer.h"

#ifdef __APPLE__

#ifdef __OBJC__
#import <Metal/Metal.h>
#import <QuartzCore/CAMetalLayer.h>
#endif

namespace engine {

class MetalRenderer : public Renderer {
public:
    MetalRenderer();
    ~MetalRenderer() override;

    bool initialize(void* windowHandle, int width, int height) override;
    void shutdown() override;
    void resize(int width, int height) override;

    MeshHandle uploadMesh(const RenderMesh& mesh) override;
    void removeMesh(MeshHandle handle) override;
    BoundingSphere getMeshBounds(MeshHandle handle) const override;
    TextureHandle uploadTexture(int width, int height, int channels,
                                const uint8_t* data) override;
    void removeTexture(TextureHandle handle) override;
    RenderStats getRenderStats() const override;

    void beginFrame() override;
    void setCamera(const CameraState& camera) override;
    void setLights(const SceneLighting& lighting) override;
    void drawMesh(MeshHandle handle, const Mat4& transform,
                  const RenderMaterial& material) override;
    void setReflectionProbes(const std::vector<ReflectionProbe>& probes) override;
    void endFrame() override;

    void initDebugUi(void* windowHandle) override;
    void shutdownDebugUi() override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};

}  // namespace engine

#endif // __APPLE__
#endif
