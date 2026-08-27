#ifndef RAYTRACER_VULKAN_RENDERER_H
#define RAYTRACER_VULKAN_RENDERER_H

#include "../renderer.h"
#include <memory>

namespace engine {

// The Vulkan backend (Linux + Windows) — the second implementation of the
// Renderer seam (ADR-0057). All Vulkan state lives behind a pimpl so this header
// pulls in no Vulkan headers; engine code sees only the backend-neutral Renderer
// interface. See src/renderer/vulkan/AGENTS.md for the design and phase plan.
//
// Phase 0 (current): instance/device/swapchain bring-up and a cleared frame.
// Resource upload and the draw/post pipeline land in later phases; the upload
// and draw entry points here are valid no-ops/stubs until then.
class VulkanRenderer final : public Renderer {
public:
    VulkanRenderer();
    ~VulkanRenderer() override;

    void setWindow(Window* window) override;
    bool initialize(void* windowHandle, int width, int height) override;
    void shutdown() override;
    void resize(int width, int height) override;

    MeshHandle uploadMesh(const RenderMesh& mesh) override;
    void removeMesh(MeshHandle handle) override;
    BoundingSphere getMeshBounds(MeshHandle handle) const override;
    TextureHandle uploadTexture(int width, int height, int channels,
                                const uint8_t* data) override;
    TextureHandle uploadTextureHDR(int width, int height, int channels,
                                   const float* data) override;
    void setEnvironmentMap(TextureHandle equirect) override;
    void setTerrainHorizon(TextureHandle map, float originX, float originZ, float extent,
                           float encodeLo, float encodeHi) override;
    void removeTexture(TextureHandle handle) override;
    RenderStats getRenderStats() const override;

    void beginFrame() override;
    void setCamera(const CameraState& camera) override;
    void setLights(const SceneLighting& lighting) override;
    void drawMesh(MeshHandle handle, const Mat4& transform,
                  const RenderMaterial& material) override;
    void drawTerrain(MeshHandle handle, const RenderMaterial& material,
                     float morphStart, float morphEnd) override;
    void endFrame() override;

    // Headless frame capture: arm a one-shot PNG of the next composited frame
    // (the control channel's `shot`). Same seam as the Metal backend.
    bool requestFrameDump(const std::string& path) override;

    // Dear ImGui (ADR-0011). No-ops unless RT_ENABLE_IMGUI is defined.
    void initDebugUi(void* windowHandle) override;
    void shutdownDebugUi() override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};

}  // namespace engine

#endif
