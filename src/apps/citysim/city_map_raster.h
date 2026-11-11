#ifndef RAYTRACER_APPS_CITYSIM_CITY_MAP_RASTER_H
#define RAYTRACER_APPS_CITYSIM_CITY_MAP_RASTER_H

// THE MAP'S PICTURE (Glenn, 2026-09-19: "make a minimap using the svg map we
// have and use it to pan and zoom around", then "It can't be imgui. But
// should show the svg map and allow panning and zooming of the map").
//
// The city map already exists as an SVG (engine/procgen/city/city_svg.h: the
// streets, blocks and buildings the generators actually built, 1 unit = 1 m,
// y = world Z). This parses that file once and rasterizes any view of it on
// demand -- centre, metres per pixel, size -- with nanosvg.
//
// TILES (ADR-0394): the map tool draws it as 256-pixel tiles at power-of-two zooms (tileView), several at once,
// each worker with its own Rasterizer. After loadCity the parsed city is read-only: rasterizeTile never touches
// it, so any number of threads may draw from one CityMapRaster. The transit lines are not part of the picture:
// the tool draws them live, as lines over the tiles.
//
// Pure: no ECS, no renderer, no threads of its own -- a test runs it headless.

#include <cstdint>
#include <string>
#include <vector>

struct NSVGimage;
struct NSVGrasterizer;

namespace citysim {

// The city layers the map shows: the ground plan a person reads a street map
// by -- blocks, streets and their kerbs, buildings. (The SVG's debug layers --
// the nav graph, conflicts, furniture, the text legend -- stay off.)
constexpr const char* kCityMapLayers = "blocks,roads,curbs,sidewalks,buildings";

class CityMapRaster {
public:
    CityMapRaster() = default;
    ~CityMapRaster();
    CityMapRaster(const CityMapRaster&) = delete;
    CityMapRaster& operator=(const CityMapRaster&) = delete;

    // The city layer: an SVG file from writeCityMapSvg. Reads its viewBox so
    // world metres map to image units exactly.
    bool loadCity(const std::string& svgPath);
    bool loaded() const { return city_ != nullptr; }

    // The city's extent in world metres (x, z), from the SVG's viewBox.
    double minX() const { return minX_; }
    double minZ() const { return minZ_; }
    double width() const { return w_; }
    double height() const { return h_; }

    struct View {
        double cx = 0, cz = 0;         // world point at the image centre
        double metresPerPixel = 2.0;
        int w = 0, h = 0;              // pixels
    };
    // RGBA8, row-major, top row = smaller world Z. Outside the city is the
    // `paper` colour. Not thread-safe (it uses the raster's own rasterizer); a test's call.
    void rasterize(const View& v, std::vector<uint8_t>& rgba) const;
    // Shapes the last rasterize() actually drew (culling check for tests).
    int lastShapesDrawn() const { return lastDrawn_; }

    // A rasterizer of one's own: one per thread that calls rasterizeTile.
    class Rasterizer {
    public:
        Rasterizer();
        ~Rasterizer();
        Rasterizer(const Rasterizer&) = delete;
        Rasterizer& operator=(const Rasterizer&) = delete;
        NSVGrasterizer* get() const { return r_; }
    private:
        NSVGrasterizer* r_ = nullptr;
    };
    // The same picture as rasterize(), drawn with the caller's rasterizer, reading the city only: safe from many
    // threads at once. Returns the shapes drawn.
    int rasterizeTile(const View& v, std::vector<uint8_t>& rgba, Rasterizer& with) const;

    // THE TILES: tile (z, ix, iy) is kTilePx square at kFinestMpp x 2^z metres per pixel, counted from the city's
    // (minX, minZ) corner.
    static constexpr int kTilePx = 256;
    static constexpr double kFinestMpp = 0.125;
    static double levelMpp(int z);
    // The level to show at `mpp`: the coarsest whose tiles are at least as sharp (shown at 0.5..1x).
    static int levelFor(double mpp);
    // The level where the whole city fits in about `across` tiles each way (the overview, drawn first).
    int overviewLevel(int across = 4) const;
    View tileView(int z, int ix, int iy) const;

private:
    NSVGimage* city_ = nullptr;
    NSVGrasterizer* rast_ = nullptr;
    double minX_ = 0, minZ_ = 0, w_ = 0, h_ = 0;
    mutable int lastDrawn_ = 0;
};

// A transit line the map draws over its tiles (CityMapToolSystem: live, a few pixels wide at any zoom).
struct TransitLine {
    std::vector<std::pair<double, double>> path;   // world x, z
    float r = 1, g = 0, b = 0;
    float width = 1.0f;     // times the map's line width (a rail line's casing is wider than a bus line)
    bool closed = true;     // a loop (a bus route is one); an open line ends where it ends
};

}  // namespace citysim

#endif
