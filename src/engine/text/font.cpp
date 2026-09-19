#include "font.h"

#include "../asset_root.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <mutex>

#define STB_TRUETYPE_IMPLEMENTATION
#define STBTT_STATIC
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
#pragma clang diagnostic ignored "-Wcast-qual"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wextra"
#endif
#include "../../../third_party/stb/stb_truetype.h"
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

namespace engine {

namespace {

// UTF-8 -> code points (street names are ASCII today; this keeps an accent
// from becoming three garbage glyphs).
std::vector<int> codepoints(const std::string& s) {
    std::vector<int> out;
    for (std::size_t i = 0; i < s.size();) {
        const unsigned char c = static_cast<unsigned char>(s[i]);
        int cp = c, extra = 0;
        if (c >= 0xF0) { cp = c & 0x07; extra = 3; }
        else if (c >= 0xE0) { cp = c & 0x0F; extra = 2; }
        else if (c >= 0xC0) { cp = c & 0x1F; extra = 1; }
        ++i;
        for (int k = 0; k < extra && i < s.size(); ++k, ++i)
            cp = (cp << 6) | (static_cast<unsigned char>(s[i]) & 0x3F);
        out.push_back(cp);
    }
    return out;
}

}  // namespace

void TextImage::resize(int width, int height, const uint8_t fill[4]) {
    w = std::max(0, width);
    h = std::max(0, height);
    rgba.assign(static_cast<std::size_t>(w) * h * 4, 0);
    for (std::size_t p = 0; p < rgba.size(); p += 4)
        for (int c = 0; c < 4; ++c) rgba[p + c] = fill[c];
}

void TextImage::fillRect(int x0, int y0, int x1, int y1, const uint8_t c[4]) {
    x0 = std::clamp(x0, 0, w); x1 = std::clamp(x1, 0, w);
    y0 = std::clamp(y0, 0, h); y1 = std::clamp(y1, 0, h);
    for (int y = y0; y < y1; ++y)
        for (int x = x0; x < x1; ++x) {
            uint8_t* d = &rgba[(static_cast<std::size_t>(y) * w + x) * 4];
            for (int k = 0; k < 4; ++k) d[k] = c[k];
        }
}

Font::Font() = default;
Font::~Font() = default;

bool Font::loadFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(in)),
                               std::istreambuf_iterator<char>());
    return loadBytes(std::move(bytes));
}

bool Font::loadBytes(std::vector<uint8_t> bytes) {
    data_ = std::move(bytes);
    info_.reset();
    if (data_.empty()) return false;
    auto info = std::make_unique<stbtt_fontinfo>();
    const int offset = stbtt_GetFontOffsetForIndex(data_.data(), 0);
    if (offset < 0 || !stbtt_InitFont(info.get(), data_.data(), offset)) return false;
    info_ = std::move(info);
    return true;
}

float Font::ascent(float pixelHeight) const {
    if (!info_) return 0;
    int a, d, g;
    stbtt_GetFontVMetrics(info_.get(), &a, &d, &g);
    return a * stbtt_ScaleForPixelHeight(info_.get(), pixelHeight);
}

float Font::descent(float pixelHeight) const {
    if (!info_) return 0;
    int a, d, g;
    stbtt_GetFontVMetrics(info_.get(), &a, &d, &g);
    return -d * stbtt_ScaleForPixelHeight(info_.get(), pixelHeight);
}

float Font::capHeight(float pixelHeight) const {
    if (!info_) return 0;
    int x0, y0, x1, y1;
    if (!stbtt_GetCodepointBox(info_.get(), 'H', &x0, &y0, &x1, &y1)) return 0;
    return (y1 - y0) * stbtt_ScaleForPixelHeight(info_.get(), pixelHeight);
}

float Font::measure(const std::string& text, float pixelHeight, float xScale) const {
    if (!info_) return 0;
    const float s = stbtt_ScaleForPixelHeight(info_.get(), pixelHeight) * xScale;
    const std::vector<int> cps = codepoints(text);
    float x = 0;
    for (std::size_t i = 0; i < cps.size(); ++i) {
        int adv, lsb;
        stbtt_GetCodepointHMetrics(info_.get(), cps[i], &adv, &lsb);
        x += adv * s;
        if (i + 1 < cps.size())
            x += stbtt_GetCodepointKernAdvance(info_.get(), cps[i], cps[i + 1]) * s;
    }
    return x;
}

void Font::draw(TextImage& img, const std::string& text, float x, float baselineY,
                float pixelHeight, const uint8_t rgba[4], float xScale) const {
    if (!info_ || img.w <= 0 || img.h <= 0) return;
    const float sy = stbtt_ScaleForPixelHeight(info_.get(), pixelHeight);
    const float sx = sy * xScale;
    const std::vector<int> cps = codepoints(text);
    std::vector<uint8_t> glyph;
    for (std::size_t i = 0; i < cps.size(); ++i) {
        const float fx = std::floor(x), shift = x - fx;
        int x0, y0, x1, y1;
        stbtt_GetCodepointBitmapBoxSubpixel(info_.get(), cps[i], sx, sy, shift, 0.0f,
                                            &x0, &y0, &x1, &y1);
        const int gw = x1 - x0, gh = y1 - y0;
        if (gw > 0 && gh > 0) {
            glyph.assign(static_cast<std::size_t>(gw) * gh, 0);
            stbtt_MakeCodepointBitmapSubpixel(info_.get(), glyph.data(), gw, gh, gw, sx, sy,
                                              shift, 0.0f, cps[i]);
            const int ox = static_cast<int>(fx) + x0;
            const int oy = static_cast<int>(std::lround(baselineY)) + y0;
            for (int gy = 0; gy < gh; ++gy) {
                const int py = oy + gy;
                if (py < 0 || py >= img.h) continue;
                for (int gx = 0; gx < gw; ++gx) {
                    const int px = ox + gx;
                    if (px < 0 || px >= img.w) continue;
                    const unsigned cov = glyph[static_cast<std::size_t>(gy) * gw + gx] * rgba[3] / 255u;
                    if (cov == 0) continue;
                    uint8_t* d = &img.rgba[(static_cast<std::size_t>(py) * img.w + px) * 4];
                    for (int c = 0; c < 3; ++c)
                        d[c] = static_cast<uint8_t>((rgba[c] * cov + d[c] * (255u - cov)) / 255u);
                    d[3] = static_cast<uint8_t>(std::min(255u, d[3] + cov * (255u - d[3]) / 255u));
                }
            }
        }
        int adv, lsb;
        stbtt_GetCodepointHMetrics(info_.get(), cps[i], &adv, &lsb);
        x += adv * sx;
        if (i + 1 < cps.size())
            x += stbtt_GetCodepointKernAdvance(info_.get(), cps[i], cps[i + 1]) * sx;
    }
}

const Font* signFont() {
    static std::once_flag once;
    static Font font;
    static bool ok = false;
    std::call_once(once, [] {
        ok = font.loadFile(assetPath("assets/fonts/Overpass-Bold.ttf"));
    });
    return ok ? &font : nullptr;
}

}  // namespace engine
