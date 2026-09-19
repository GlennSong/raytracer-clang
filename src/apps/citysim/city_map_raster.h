#ifndef RAYTRACER_APPS_CITYSIM_CITY_MAP_RASTER_H
#define RAYTRACER_APPS_CITYSIM_CITY_MAP_RASTER_H

// THE MAP'S PICTURE (Glenn, 2026-09-19: "make a minimap using the svg map we
// have and use it to pan and zoom around", then "It can't be imgui. But
// should show the svg map and allow panning and zooming of the map").
//
// The city map already exists as an SVG (engine/procgen/city/city_svg.h: the
// streets, blocks and buildings the generators actually built, 1 unit = 1 m,
// y = world Z). This parses that file once and rasterizes any view of it on
// demand -- centre, metres per pixel, size -- with nanosvg, plus a transit
// overlay (the bus lines) whose strokes stay a few PIXELS wide at every zoom,
// so a line reads at city scale without paving over a street up close.
//
// Pure: no ECS, no renderer, no threads -- the map tool runs it on a worker
// and uploads the pixels; a test runs it headless. One rasterize() at a time.

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
    // The transit overlay, in the SAME world frame (see transitSvg).
    bool loadTransit(const std::string& svgText);
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
    // `paper` colour. Transit strokes are `linePx` pixels wide.
    void rasterize(const View& v, std::vector<uint8_t>& rgba, float linePx = 4.0f) const;

    // Shapes the last rasterize() actually drew (culling check for tests).
    int lastShapesDrawn() const { return lastDrawn_; }

private:
    NSVGimage* city_ = nullptr;
    NSVGimage* transit_ = nullptr;
    NSVGrasterizer* rast_ = nullptr;
    double minX_ = 0, minZ_ = 0, w_ = 0, h_ = 0;
    mutable int lastDrawn_ = 0;
    mutable std::vector<uint8_t> overlay_;   // scratch for the transit pass
};

// The transit overlay as SVG text in the city map's frame: one closed line per
// route in its sign colour (routeColour), for CityMapRaster::loadTransit.
// `routes` holds each route's driven path (world x, z); `colours` its RGB 0..1.
struct TransitLine {
    std::vector<std::pair<double, double>> path;
    float r = 1, g = 0, b = 0;
};
std::string transitSvg(const std::vector<TransitLine>& lines, double minX, double minZ,
                       double w, double h);

}  // namespace citysim

#endif
