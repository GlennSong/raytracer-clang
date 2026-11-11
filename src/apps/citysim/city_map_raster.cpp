#include "city_map_raster.h"

#include <algorithm>
#include <cstdlib>
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

// Draw only the shapes whose bounds meet the window [x0,x1]x[y0,y1] (image units): the rasterizer flattens every
// shape it is handed, so a street-level view of a whole city was paying for the whole city. The shapes are COPIED
// (shallow: their paths are shared, read-only) and linked into a list of the caller's own, so the parsed image is
// never written and several threads can draw from it at once.
int culledRasterize(NSVGrasterizer* r, const NSVGimage* img, double x0, double y0, double x1, double y1, float tx,
                    float ty, float scale, unsigned char* dst, int w, int h) {
    std::vector<NSVGshape> keep;
    for (const NSVGshape* s = img->shapes; s; s = s->next) {
        // Stroke half-width pads the bounds (a road's stroke spills past its centreline).
        const double pad = s->strokeWidth * 0.5;
        if (s->bounds[2] + pad < x0 || s->bounds[0] - pad > x1 || s->bounds[3] + pad < y0 || s->bounds[1] - pad > y1)
            continue;
        keep.push_back(*s);
    }
    for (std::size_t k = 0; k < keep.size(); ++k) keep[k].next = k + 1 < keep.size() ? &keep[k + 1] : nullptr;
    NSVGimage local = *img;
    local.shapes = keep.empty() ? nullptr : keep.data();
    nsvgRasterize(r, &local, tx, ty, scale, dst, w, h, w * 4);
    return static_cast<int>(keep.size());
}

// LONG STROKES CUT SHORT (ADR-0394). A city's kerb lines and sidewalk bands are stroked loops kilometres long --
// on the island one has 66,618 points -- and the rasterizer strokes every point of every shape it is handed, so a
// tile that touched one paid for the whole loop (240 ms a tile). A stroke, unlike a fill, can be cut into runs that
// draw the same picture: each run of `maxSegs` segments becomes its own shape with its own bounds, so a tile meets
// only the runs near it. Runs get round caps, so a cut is a round join (the sidewalks' own; on a 0.5 m kerb line
// the difference is under a pixel at the finest zoom). Fills, dashes and gradients are left whole.
void splitLongStrokes(NSVGimage* img, int maxSegs) {
    NSVGshape* head = nullptr;
    NSVGshape** tail = &head;
    for (NSVGshape* s = img->shapes; s;) {
        NSVGshape* next = s->next;
        int segs = 0;
        for (const NSVGpath* p = s->paths; p; p = p->next) segs += (p->npts - 1) / 3;
        const bool strokeOnly = s->fill.type == NSVG_PAINT_NONE && s->stroke.type == NSVG_PAINT_COLOR &&
                                s->strokeDashCount == 0;
        if (!strokeOnly || segs <= maxSegs) {
            *tail = s;
            tail = &s->next;
            s = next;
            continue;
        }
        for (const NSVGpath* p = s->paths; p; p = p->next) {
            if (p->npts < 4) continue;
            std::vector<float> pts(p->pts, p->pts + 2 * p->npts);
            if (p->closed) {   // the closing segment, made explicit (a straight cubic), so every run is open
                const float x0 = pts[pts.size() - 2], y0 = pts[pts.size() - 1], x1 = pts[0], y1 = pts[1];
                for (const float t : {1.0f / 3.0f, 2.0f / 3.0f, 1.0f}) {
                    pts.push_back(x0 + (x1 - x0) * t);
                    pts.push_back(y0 + (y1 - y0) * t);
                }
            }
            const int n = static_cast<int>((pts.size() / 2 - 1) / 3);
            for (int first = 0; first < n; first += maxSegs) {
                const int count = std::min(maxSegs, n - first);
                NSVGpath* run = static_cast<NSVGpath*>(std::malloc(sizeof(NSVGpath)));
                NSVGshape* piece = static_cast<NSVGshape*>(std::malloc(sizeof(NSVGshape)));
                if (!run || !piece) {
                    std::free(run);
                    std::free(piece);
                    continue;
                }
                run->npts = 1 + 3 * count;
                run->pts = static_cast<float*>(std::malloc(sizeof(float) * 2 * run->npts));
                std::memcpy(run->pts, pts.data() + 2 * 3 * first, sizeof(float) * 2 * run->npts);
                run->closed = 0;
                run->next = nullptr;
                run->bounds[0] = run->bounds[2] = run->pts[0];
                run->bounds[1] = run->bounds[3] = run->pts[1];
                for (int k = 1; k < run->npts; ++k) {
                    run->bounds[0] = std::min(run->bounds[0], run->pts[2 * k]);
                    run->bounds[1] = std::min(run->bounds[1], run->pts[2 * k + 1]);
                    run->bounds[2] = std::max(run->bounds[2], run->pts[2 * k]);
                    run->bounds[3] = std::max(run->bounds[3], run->pts[2 * k + 1]);
                }
                *piece = *s;   // colours only (checked above): nothing shared that nsvgDelete would free twice
                piece->strokeLineCap = NSVG_CAP_ROUND;
                std::memcpy(piece->bounds, run->bounds, sizeof(piece->bounds));
                piece->paths = run;
                piece->next = nullptr;
                *tail = piece;
                tail = &piece->next;
            }
        }
        nsvg__deletePaths(s->paths);
        std::free(s);
        s = next;
    }
    *tail = nullptr;
    img->shapes = head;
}

}  // namespace

CityMapRaster::~CityMapRaster() {
    if (city_) nsvgDelete(city_);
    if (rast_) nsvgDeleteRasterizer(rast_);
}

CityMapRaster::Rasterizer::Rasterizer() : r_(nsvgCreateRasterizer()) {}
CityMapRaster::Rasterizer::~Rasterizer() {
    if (r_) nsvgDeleteRasterizer(r_);
}

bool CityMapRaster::loadCity(const std::string& svgPath) {
    if (!readViewBox(svgPath, minX_, minZ_, w_, h_)) return false;
    NSVGimage* img = nsvgParseFromFile(svgPath.c_str(), "px", 96.0f);
    if (!img) return false;
    splitLongStrokes(img, 128);
    if (city_) nsvgDelete(city_);
    city_ = img;
    if (!rast_) rast_ = nsvgCreateRasterizer();
    return rast_ != nullptr;
}

namespace {

int drawView(NSVGrasterizer* r, const NSVGimage* city, double minX, double minZ, const CityMapRaster::View& v,
             std::vector<uint8_t>& rgba) {
    const int w = std::max(1, v.w), h = std::max(1, v.h);
    rgba.assign(static_cast<std::size_t>(w) * h * 4, 0);
    if (!city || !r) return 0;
    const double mpp = std::max(1e-3, v.metresPerPixel);
    const double left = v.cx - w * 0.5 * mpp, top = v.cz - h * 0.5 * mpp;
    // Image units are world metres less the viewBox origin (the SVG's width
    // equals its viewBox width, so nanosvg's scale is 1).
    const double ix0 = left - minX, iy0 = top - minZ;
    const double ix1 = ix0 + w * mpp, iy1 = iy0 + h * mpp;
    const int drawn = culledRasterize(r, city, ix0, iy0, ix1, iy1, static_cast<float>(-ix0 / mpp),
                                      static_cast<float>(-iy0 / mpp), static_cast<float>(1.0 / mpp), rgba.data(), w, h);
    // Paper under anything the city did not cover (off its edge).
    for (std::size_t p = 0; p < rgba.size(); p += 4) {
        const unsigned a = rgba[p + 3];
        if (a == 255) continue;
        for (int c = 0; c < 3; ++c)
            rgba[p + c] = static_cast<uint8_t>((rgba[p + c] * a + kPaper[c] * (255u - a)) / 255u);
        rgba[p + 3] = 255;
    }
    return drawn;
}

}  // namespace

void CityMapRaster::rasterize(const View& v, std::vector<uint8_t>& rgba) const {
    lastDrawn_ = drawView(rast_, city_, minX_, minZ_, v, rgba);
}

int CityMapRaster::rasterizeTile(const View& v, std::vector<uint8_t>& rgba, Rasterizer& with) const {
    return drawView(with.get(), city_, minX_, minZ_, v, rgba);
}

double CityMapRaster::levelMpp(int z) { return std::ldexp(kFinestMpp, z); }

int CityMapRaster::levelFor(double mpp) {
    const int z = static_cast<int>(std::floor(std::log2(std::max(mpp, kFinestMpp) / kFinestMpp) + 1e-9));
    return std::clamp(z, 0, 20);
}

int CityMapRaster::overviewLevel(int across) const {
    int z = 0;
    while (z < 20 && std::max(w_, h_) > across * kTilePx * levelMpp(z)) ++z;
    return z;
}

CityMapRaster::View CityMapRaster::tileView(int z, int ix, int iy) const {
    const double m = levelMpp(z);
    View v;
    v.metresPerPixel = m;
    v.w = v.h = kTilePx;
    v.cx = minX_ + (ix + 0.5) * kTilePx * m;
    v.cz = minZ_ + (iy + 0.5) * kTilePx * m;
    return v;
}

}  // namespace citysim
