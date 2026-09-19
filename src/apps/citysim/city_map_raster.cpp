#include "city_map_raster.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>

// nanosvg (third_party/nanosvg, zlib licence): implemented in this one unit.
#define NANOSVG_IMPLEMENTATION
#define NANOSVGRAST_IMPLEMENTATION
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wall"
#pragma clang diagnostic ignored "-Wextra"
#pragma clang diagnostic ignored "-Wpedantic"
#pragma clang diagnostic ignored "-Wconversion"
#pragma clang diagnostic ignored "-Wsign-conversion"
#pragma clang diagnostic ignored "-Wshadow"
#pragma clang diagnostic ignored "-Wdouble-promotion"
#pragma clang diagnostic ignored "-Wimplicit-float-conversion"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wextra"
#endif
#include "../../../third_party/nanosvg/nanosvg.h"
#include "../../../third_party/nanosvg/nanosvgrast.h"
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

namespace citysim {

namespace {

// The paper the city sits on, beyond its own background rect.
constexpr uint8_t kPaper[4] = {214, 208, 196, 255};

// viewBox='minX minZ w h' from the file's <svg> tag.
bool readViewBox(const std::string& path, double& x, double& z, double& w, double& h) {
    std::ifstream in(path);
    if (!in) return false;
    char buf[4096];
    in.read(buf, sizeof(buf) - 1);
    buf[in.gcount()] = 0;
    const char* v = std::strstr(buf, "viewBox=");
    if (!v) return false;
    v += 9;   // past viewBox= and the quote
    return std::sscanf(v, "%lf %lf %lf %lf", &x, &z, &w, &h) == 4 && w > 0 && h > 0;
}

// Draw only the shapes whose bounds meet the window [x0,x1]x[y0,y1] (image
// units): relink the list for the call, restore it after. The rasterizer
// flattens every shape it is handed, so a street-level view of a whole city
// was paying for the whole city.
struct Culled {
    NSVGimage* img;
    NSVGshape* head;
    int drawn = 0;
    Culled(NSVGimage* image, double x0, double y0, double x1, double y1) : img(image), head(image->shapes) {
        NSVGshape* first = nullptr;
        NSVGshape** tail = &first;
        std::vector<NSVGshape*> all;
        for (NSVGshape* s = head; s; s = s->next) all.push_back(s);
        for (NSVGshape* s : all) {
            // Stroke half-width pads the bounds (a road's stroke spills past
            // its centreline).
            const double pad = s->strokeWidth * 0.5;
            if (s->bounds[2] + pad < x0 || s->bounds[0] - pad > x1 ||
                s->bounds[3] + pad < y0 || s->bounds[1] - pad > y1)
                continue;
            *tail = s;
            tail = &s->next;
            ++drawn;
        }
        *tail = nullptr;
        // Keep the original chain recoverable: store it before relinking.
        order = std::move(all);
        img->shapes = first;
    }
    ~Culled() {
        for (std::size_t i = 0; i < order.size(); ++i)
            order[i]->next = i + 1 < order.size() ? order[i + 1] : nullptr;
        img->shapes = head;
    }
    std::vector<NSVGshape*> order;
};

}  // namespace

CityMapRaster::~CityMapRaster() {
    if (city_) nsvgDelete(city_);
    if (transit_) nsvgDelete(transit_);
    if (rast_) nsvgDeleteRasterizer(rast_);
}

bool CityMapRaster::loadCity(const std::string& svgPath) {
    if (!readViewBox(svgPath, minX_, minZ_, w_, h_)) return false;
    NSVGimage* img = nsvgParseFromFile(svgPath.c_str(), "px", 96.0f);
    if (!img) return false;
    if (city_) nsvgDelete(city_);
    city_ = img;
    if (!rast_) rast_ = nsvgCreateRasterizer();
    return rast_ != nullptr;
}

bool CityMapRaster::loadTransit(const std::string& svgText) {
    std::vector<char> text(svgText.begin(), svgText.end());
    text.push_back(0);
    NSVGimage* img = nsvgParse(text.data(), "px", 96.0f);
    if (!img) return false;
    if (transit_) nsvgDelete(transit_);
    transit_ = img;
    return true;
}

void CityMapRaster::rasterize(const View& v, std::vector<uint8_t>& rgba, float linePx) const {
    const int w = std::max(1, v.w), h = std::max(1, v.h);
    rgba.assign(static_cast<std::size_t>(w) * h * 4, 0);
    lastDrawn_ = 0;
    if (!city_ || !rast_) return;
    const double mpp = std::max(1e-3, v.metresPerPixel);
    const double left = v.cx - w * 0.5 * mpp, top = v.cz - h * 0.5 * mpp;
    // Image units are world metres less the viewBox origin (the SVG's width
    // equals its viewBox width, so nanosvg's scale is 1).
    const double ix0 = left - minX_, iy0 = top - minZ_;
    const double ix1 = ix0 + w * mpp, iy1 = iy0 + h * mpp;
    const float scale = static_cast<float>(1.0 / mpp);
    const float tx = static_cast<float>(-ix0 / mpp), ty = static_cast<float>(-iy0 / mpp);
    {
        Culled c(city_, ix0, iy0, ix1, iy1);
        lastDrawn_ = c.drawn;
        nsvgRasterize(rast_, city_, tx, ty, scale, rgba.data(), w, h, w * 4);
    }
    // Paper under anything the city did not cover (off its edge).
    for (std::size_t p = 0; p < rgba.size(); p += 4) {
        const unsigned a = rgba[p + 3];
        if (a == 255) continue;
        for (int c = 0; c < 3; ++c)
            rgba[p + c] = static_cast<uint8_t>((rgba[p + c] * a + kPaper[c] * (255u - a)) / 255u);
        rgba[p + 3] = 255;
    }
    if (!transit_) return;
    // Lines a constant few pixels wide: the width in image units is px x mpp.
    for (NSVGshape* s = transit_->shapes; s; s = s->next)
        s->strokeWidth = static_cast<float>(linePx * mpp);
    overlay_.assign(rgba.size(), 0);
    {
        Culled c(transit_, ix0, iy0, ix1, iy1);
        nsvgRasterize(rast_, transit_, tx, ty, scale, overlay_.data(), w, h, w * 4);
    }
    // nanosvg writes premultiplied-looking straight colour with coverage in
    // alpha; blend it over.
    for (std::size_t p = 0; p < rgba.size(); p += 4) {
        const unsigned a = overlay_[p + 3];
        if (a == 0) continue;
        for (int c = 0; c < 3; ++c)
            rgba[p + c] = static_cast<uint8_t>((overlay_[p + c] * a + rgba[p + c] * (255u - a)) / 255u);
    }
}

std::string transitSvg(const std::vector<TransitLine>& lines, double minX, double minZ,
                       double w, double h) {
    std::ostringstream out;
    out.precision(10);
    out << "<svg xmlns='http://www.w3.org/2000/svg' viewBox='" << minX << " " << minZ << " "
        << w << " " << h << "' width='" << w << "' height='" << h << "'>\n";
    for (const TransitLine& l : lines) {
        if (l.path.size() < 2) continue;
        char col[16];
        std::snprintf(col, sizeof(col), "#%02x%02x%02x",
                      static_cast<int>(std::clamp(l.r, 0.0f, 1.0f) * 255.0f + 0.5f),
                      static_cast<int>(std::clamp(l.g, 0.0f, 1.0f) * 255.0f + 0.5f),
                      static_cast<int>(std::clamp(l.b, 0.0f, 1.0f) * 255.0f + 0.5f));
        out << "<polygon fill='none' stroke='" << col
            << "' stroke-width='4' stroke-linejoin='round' stroke-opacity='0.9' points='";
        for (const auto& p : l.path) out << p.first << "," << p.second << " ";
        out << "'/>\n";
    }
    out << "</svg>\n";
    return out.str();
}

}  // namespace citysim
