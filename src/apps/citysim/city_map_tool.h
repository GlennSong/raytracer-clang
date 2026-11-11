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
// The picture is tiles (ADR-0394): rasterizing a whole screen took 0.5 s on one
// thread (metro_v2_test), so the city is drawn as 256-pixel tiles at power-of-two
// zooms by a few workers, nearest the centre first, kept on the GPU, with the
// overview drawn as soon as the level is up so the first look is never blank.

#include "../../engine/system.h"
#include "../../rt_math.h"
#include "../../renderer/renderer.h"
#include "city_map_raster.h"

#include <condition_variable>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <set>
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
    // Take the transit lines from the sim (update does, when the city is built and when the trains arrive).
    void takeLines() {
        lines_ = transitLines();
        linesTaken_ = true;
    }
    // The tints composeFrame gives its markers (so a test can find them).
    static const float* youColour();
    static const float* frameColour();

private:
    // What the picture's transit overlay draws: the rail lines (under) and the bus lines. Main thread only.
    std::vector<TransitLine> transitLines() const;
    void startWorker(engine::FrameContext& ctx);
    void stopWorker();
    void ensureSprites(engine::Renderer& r);

    CityRenderSystem& city_;
    engine::FlyCameraController& fly_;

    bool open_ = false;
    bool layered_ = false;            // the map's input layer is pushed (open_, as the input map knows it)
    bool follow_ = true;              // the view tracks the player until panned
    double cx_ = 0, cz_ = 0;          // world point at the panel centre
    double mpp_ = 1.0;                // metres per framebuffer pixel
    bool sized_ = false;              // mpp_ fitted to the city on first open

    // Sprites: a round dot (stops, buses) and the you-are-here arrow.
    engine::TextureHandle dot_, arrow_;

    // --- THE TILES (ADR-0394) ------------------------------------------------
    // The picture is CityMapRaster tiles, 256 px at power-of-two zooms, kept on the GPU. Drawn coarsest first, so
    // a view is never blank: the overview level (the whole city in a few tiles, drawn as soon as the level is up,
    // map open or not) shows under the sharper ones while they come.
    struct TileKey {
        int z = 0, ix = 0, iy = 0;
        bool operator<(const TileKey& o) const {
            return z != o.z ? z < o.z : (ix != o.ix ? ix < o.ix : iy < o.iy);
        }
    };
    struct Tile {
        engine::TextureHandle tex;
        uint64_t used = 0;   // the frame it was last wanted (the cache drops the oldest)
    };
    std::map<TileKey, Tile> tiles_;
    uint64_t frame_ = 0;
    // the city's frame, copied from the parsed raster on the main thread once it is ready
    int overviewZ_ = -1;
    double cityX_ = 0, cityZ_ = 0, cityW_ = 0, cityH_ = 0;
    void wantTiles(double panelW, double panelH);   // main thread: what to draw next, coarse and central first
    void takeTiles(engine::Renderer& r);           // main thread: upload what the workers drew; drop the oldest

    // the loader (writes and parses the SVG) and the tile workers, each with its own rasterizer
    std::thread loader_;
    std::vector<std::thread> workers_;
    std::mutex mu_;
    std::condition_variable cv_;
    bool quit_ = false;
    std::shared_ptr<const CityMapRaster> raster_;   // set once, read-only after (guarded for the handoff)
    std::vector<TileKey> want_;                     // to draw, first first
    std::set<TileKey> drawing_;                     // taken by a worker, not back yet
    std::vector<std::pair<TileKey, std::vector<uint8_t>>> done_;

    // The transit lines, drawn live over the tiles (a few pixels wide at any zoom). The trains start a few frames
    // after the city: the lines are taken again once the rail is there.
    std::vector<TransitLine> lines_;
    bool linesTaken_ = false, railInLines_ = false;
};

}  // namespace citysim

#endif
