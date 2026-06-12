#include "renderer.h"

namespace engine {

namespace {

// A renderer that renders nothing. Stands in on platforms with no GPU backend
// yet (the Qt editor shell on Linux builds and shows its UI with a dead
// viewport until the Vulkan backend exists) and doubles as a test stand-in.
class NullRenderer final : public Renderer {
public:
    bool initialize(void*, int, int) override { return true; }
    void shutdown() override {}
    void resize(int, int) override {}

    MeshHandle uploadMesh(const RenderMesh&) override {
        MeshHandle handle;
        handle.index = nextMesh++;
        handle.generation = 1;
        return handle;
    }
    void removeMesh(MeshHandle) override {}
    BoundingSphere getMeshBounds(MeshHandle) const override {
        // Unit-ish bounds so headless picking/culling code has something
        // real to test against.
        return BoundingSphere{Vec3(0, 0, 0), 1.0f, Vec3(-1, -1, -1),
                              Vec3(1, 1, 1)};
    }
    TextureHandle uploadTexture(int, int, int, const uint8_t*) override {
        return TextureHandle{};
    }
    void removeTexture(TextureHandle) override {}
    RenderStats getRenderStats() const override { return RenderStats{}; }

    void beginFrame() override {}
    void setCamera(const CameraState&) override {}
    void setLights(const SceneLighting&) override {}
    void drawMesh(MeshHandle, const Mat4&, const RenderMaterial&) override {}
    void endFrame() override {}

private:
    uint32_t nextMesh = 1;
};

}  // namespace

// This translation unit provides the factory only where no platform backend
// does (the Metal backend defines it on Apple) — link exactly one of them.
std::unique_ptr<Renderer> Renderer::create() {
    return std::make_unique<NullRenderer>();
}

}  // namespace engine
