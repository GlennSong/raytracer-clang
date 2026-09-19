#ifndef RAYTRACER_APPS_CITYSIM_CITY_MAP_TOOL_H
#define RAYTRACER_APPS_CITYSIM_CITY_MAP_TOOL_H

// THE MAP, A TOOL IN THE PLAYER'S HAND (Glenn, 2026-09-19: "make a minimap
// using the svg map we have and use it to pan and zoom around. We should be
// able to see where we are, which direction we are facing, bus stops and bus
// lines. It probably shouldn't pause the city simulation ... for now maybe we
// should stick it as a number entry since it's akin to a tool", and "It can't
// be imgui. But should show the svg map and allow panning and zooming").
//
// Slot 3 raises it (slots 1 and 2 put it away, like any tool change). It is the
// city SVG (CityMapRaster) drawn through the renderer's GAME UI layer
// (Renderer::submitUi), not the debug overlay. While it is up the mouse is the
// map's: drag pans, the wheel zooms, C centres it back on you. The city keeps
// running and you can still walk.
//
// Rasterizing a view takes 50-200 ms, so it runs on a worker; the last picture
// is shown moved and scaled until the new one lands.

#include "../../engine/system.h"
#include "../../rt_math.h"
#include "../../renderer/renderer.h"
#include "city_map_raster.h"

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace engine {
class FlyCameraController;
struct CityMapData;
}

namespace citysim {

class CityRenderSystem;

class CityMapToolSystem : public engine::System {
public:
    CityMapToolSystem(CityRenderSystem& city, engine::FlyCameraController& fly)
        : city_(city), fly_(fly) {}
    ~CityMapToolSystem() override;

    void onStart(engine::FrameContext& ctx) override;
    void update(engine::FrameContext& ctx) override;
    void render(engine::FrameContext& ctx) override;
    void onStop(engine::FrameContext& ctx) override;

    bool open() const { return open_; }

    // --- testable core (no FrameContext, no GPU) ----------------------------
    // Everything the map draws this frame, in framebuffer pixels: frame,
    // paper, the picture, stops, live buses, you, the legend.
    std::vector<engine::Renderer::UiQuad> composeFrame(engine::World& world, int fbW,
                                                        int fbH) const;
    struct Rect { float x0 = 0, y0 = 0, x1 = 0, y1 = 0; };
    static Rect panel(int fbW, int fbH);
    // Put the map up at a view (tests; the game opens it with slot 3).
    void showAt(double cx, double cz, double metresPerPixel) {
        open_ = true; follow_ = false; sized_ = true;
        cx_ = cx; cz_ = cz; mpp_ = metresPerPixel;
    }
    // The tints composeFrame gives its markers (so a test can find them).
    static const float* youColour();
    static const float* frameColour();

private:
    void startWorker(engine::FrameContext& ctx);
    void stopWorker();
    void requestRaster(const CityMapRaster::View& v);
    void ensureSprites(engine::Renderer& r);

    CityRenderSystem& city_;
    engine::FlyCameraController& fly_;

    bool open_ = false;
    bool follow_ = true;              // the view tracks the player until panned
    double cx_ = 0, cz_ = 0;          // world point at the panel centre
    double mpp_ = 1.0;                // metres per framebuffer pixel
    bool sized_ = false;              // mpp_ fitted to the city on first open

    // The picture on the GPU and the view it was drawn for.
    engine::TextureHandle tex_;
    CityMapRaster::View texView_;
    // Sprites: a round dot (stops, buses) and the you-are-here arrow.
    engine::TextureHandle dot_, arrow_;

    // --- the worker ---------------------------------------------------------
    std::thread worker_;
    std::mutex mu_;
    std::condition_variable cv_;
    bool quit_ = false;
    bool loading_ = false;            // SVG not parsed yet
    bool haveJob_ = false;
    CityMapRaster::View job_;
    bool haveResult_ = false;
    CityMapRaster::View resultView_;
    std::vector<uint8_t> result_;
    bool rasterBusy_ = false;
    CityMapRaster::View lastAsked_;
    bool askedAny_ = false;
    double settle_ = 0;               // seconds since the view last changed
};

}  // namespace citysim

#endif
