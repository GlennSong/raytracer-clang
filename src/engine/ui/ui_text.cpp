#include "ui_text.h"

#include <algorithm>
#include <cmath>

namespace engine {

UiText::Atlas* UiText::atlas(int px) {
    px = std::max(6, px);
    const auto it = atlases_.find(px);
    if (it != atlases_.end()) return &it->second;
    const Font* f = font_ ? font_ : signFont();
    if (!f) return nullptr;
    Atlas a;
    if (!f->bakeAtlas(static_cast<float>(px), a.glyphs)) return nullptr;
    return &atlases_.emplace(px, std::move(a)).first->second;
}

float UiText::measure(const std::string& text, float size) {
    const Atlas* a = atlas(static_cast<int>(std::lround(size)));
    return a ? a->glyphs.measure(text) : 0.0f;
}

float UiText::ascent(float size) {
    const Atlas* a = atlas(static_cast<int>(std::lround(size)));
    return a ? a->glyphs.ascent : size * 0.8f;
}

float UiText::descent(float size) {
    const Atlas* a = atlas(static_cast<int>(std::lround(size)));
    return a ? a->glyphs.descent : size * 0.2f;
}

void UiText::text(Renderer& r, std::vector<Quad>& out, float x, float baseline, const std::string& text, float size,
                  const float rgba[4]) {
    Atlas* a = atlas(static_cast<int>(std::lround(size)));
    if (!a) return;
    if (!a->texture.valid())
        a->texture = r.uploadTexture(a->glyphs.image.w, a->glyphs.image.h, 4, a->glyphs.image.rgba.data());
    const GlyphAtlas& g = a->glyphs;
    const std::vector<int> cps = utf8Codepoints(text);
    const GlyphAtlas::Glyph* space = g.glyph(' ');
    // pixel-aligned pen: the atlas was baked at whole pixels, so the glyphs stay crisp
    float pen = std::round(x);
    const float base = std::round(baseline);
    for (std::size_t i = 0; i < cps.size(); ++i) {
        const GlyphAtlas::Glyph* gl = g.glyph(cps[i]);
        if (gl && gl->x1 > gl->x0) {
            const float gx = std::round(pen);
            Quad q = Quad::rect(gx + gl->x0, base + gl->y0, gx + gl->x1, base + gl->y1);
            q.texture = a->texture;
            q.u[0] = gl->u0; q.u[1] = gl->u1; q.u[2] = gl->u1; q.u[3] = gl->u0;
            q.v[0] = gl->v0; q.v[1] = gl->v0; q.v[2] = gl->v1; q.v[3] = gl->v1;
            q.r = rgba[0]; q.g = rgba[1]; q.b = rgba[2]; q.a = rgba[3];
            out.push_back(q);
        }
        pen += gl ? gl->advance : space ? space->advance : 0;
        if (i + 1 < cps.size()) pen += g.kern(cps[i], cps[i + 1]);
    }
}

UiText::Box UiText::label(Renderer& r, std::vector<Quad>& out, float cx, float y, const std::string& t, float size,
                          const float fg[4], const float bg[4], float pad) {
    const float w = measure(t, size), asc = ascent(size), desc = descent(size);
    Box b;
    b.x0 = std::round(cx - w * 0.5f - pad);
    b.x1 = std::round(cx + w * 0.5f + pad);
    b.y0 = std::round(y);
    b.y1 = std::round(y + asc + desc + pad);
    rect(out, b.x0, b.y0, b.x1, b.y1, bg);
    text(r, out, b.x0 + pad, b.y0 + pad * 0.5f + asc, t, size, fg);
    return b;
}

void UiText::rect(std::vector<Quad>& out, float x0, float y0, float x1, float y1, const float rgba[4]) {
    Quad q = Quad::rect(x0, y0, x1, y1);
    q.r = rgba[0]; q.g = rgba[1]; q.b = rgba[2]; q.a = rgba[3];
    out.push_back(q);
}

namespace {
// a 128 px sprite: white, alpha the coverage of the annulus between inner and outer (in radii, 4x4 supersampled)
std::vector<uint8_t> annulus(Real inner) {
    constexpr int n = 128;
    std::vector<uint8_t> px(static_cast<std::size_t>(n) * n * 4, 255);
    for (int y = 0; y < n; ++y)
        for (int x = 0; x < n; ++x) {
            int hits = 0;
            for (int sy = 0; sy < 4; ++sy)
                for (int sx = 0; sx < 4; ++sx) {
                    const Real u = ((x + (sx + 0.5) / 4.0) / n) * 2.0 - 1.0;
                    const Real v = ((y + (sy + 0.5) / 4.0) / n) * 2.0 - 1.0;
                    const Real d = std::sqrt(u * u + v * v);
                    if (d <= 1.0 && d >= inner) ++hits;
                }
            px[(static_cast<std::size_t>(y) * n + x) * 4 + 3] = static_cast<uint8_t>(hits * 255 / 16);
        }
    return px;
}
}  // namespace

void UiText::disc(Renderer& r, std::vector<Quad>& out, float cx, float cy, float radius, const float rgba[4]) {
    if (!disc_.valid()) {
        const std::vector<uint8_t> px = annulus(0.0);
        disc_ = r.uploadTexture(128, 128, 4, px.data());
    }
    Quad q = Quad::rect(cx - radius, cy - radius, cx + radius, cy + radius);
    q.texture = disc_;
    q.r = rgba[0]; q.g = rgba[1]; q.b = rgba[2]; q.a = rgba[3];
    out.push_back(q);
}

void UiText::ring(Renderer& r, std::vector<Quad>& out, float cx, float cy, float radius, float width,
                  const float rgba[4]) {
    const int key = std::clamp(static_cast<int>(std::lround(width / std::max(radius, 1.0f) * 64.0f)), 1, 64);
    TextureHandle& t = rings_[key];
    if (!t.valid()) {
        const std::vector<uint8_t> px = annulus(1.0 - key / 64.0);
        t = r.uploadTexture(128, 128, 4, px.data());
    }
    Quad q = Quad::rect(cx - radius, cy - radius, cx + radius, cy + radius);
    q.texture = t;
    q.r = rgba[0]; q.g = rgba[1]; q.b = rgba[2]; q.a = rgba[3];
    out.push_back(q);
}

void UiText::release(Renderer& r) {
    for (auto& [px, a] : atlases_)
        if (a.texture.valid()) r.removeTexture(a.texture);
    atlases_.clear();
    if (disc_.valid()) r.removeTexture(disc_);
    disc_ = TextureHandle{};
    for (auto& [k, t] : rings_)
        if (t.valid()) r.removeTexture(t);
    rings_.clear();
}

}  // namespace engine
