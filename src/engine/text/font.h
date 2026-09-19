#ifndef RAYTRACER_ENGINE_TEXT_FONT_H
#define RAYTRACER_ENGINE_TEXT_FONT_H

// TEXT INTO PICTURES (Glenn, 2026-09-19: "Can we embed a font or use a font
// atlas? It would be nice to have in world signs for streets").
//
// A TrueType font (stb_truetype, third_party/stb) that measures a string and
// draws it into an RGBA image -- the CPU half of every piece of in-world and
// in-game text: street-name blades composited into a sign atlas, labels on the
// map. Nothing here touches a renderer; the caller uploads the image it built.
//
// Pixel heights are the font's em height in pixels (stbtt_ScaleForPixelHeight);
// capHeight() is what a legibility check wants (how tall an 'H' comes out).

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

struct stbtt_fontinfo;

namespace engine {

// RGBA8, row-major, top row first, straight alpha.
struct TextImage {
    int w = 0, h = 0;
    std::vector<uint8_t> rgba;
    void resize(int width, int height, const uint8_t fill[4]);
    // Blend a solid rectangle (for sign faces, borders).
    void fillRect(int x0, int y0, int x1, int y1, const uint8_t c[4]);
};

class Font {
public:
    Font();
    ~Font();
    Font(const Font&) = delete;
    Font& operator=(const Font&) = delete;

    bool loadFile(const std::string& path);
    bool loadBytes(std::vector<uint8_t> bytes);
    bool loaded() const { return info_ != nullptr; }

    // Pixels above / below the baseline at `pixelHeight` (descent positive).
    float ascent(float pixelHeight) const;
    float descent(float pixelHeight) const;
    // How tall a capital H stands, in pixels.
    float capHeight(float pixelHeight) const;
    // Advance width of `text` (kerned), condensed by xScale.
    float measure(const std::string& text, float pixelHeight, float xScale = 1.0f) const;
    // Draw with the baseline's left end at (x, baselineY), blended over img.
    void draw(TextImage& img, const std::string& text, float x, float baselineY,
              float pixelHeight, const uint8_t rgba[4], float xScale = 1.0f) const;

private:
    std::vector<uint8_t> data_;
    std::unique_ptr<stbtt_fontinfo> info_;
};

// The engine's sign/UI face (assets/fonts/Overpass-Bold.ttf), loaded once on
// first use from the asset root. nullptr if the file is missing.
const Font* signFont();

}  // namespace engine

#endif
