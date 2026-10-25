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

// A GLYPH ATLAS (the game UI's text, ADR-0250): printable ASCII and the degree sign, each glyph's coverage baked
// once at one pixel height into a white image's alpha, with where it sits from the pen and the kerning between
// them -- so a line of text is one quad per glyph on one texture, crisp at the height it was baked for.
struct GlyphAtlas {
    TextImage image;              // white; alpha = coverage
    float pixelHeight = 0, ascent = 0, descent = 0;
    struct Glyph {
        float u0 = 0, v0 = 0, u1 = 0, v1 = 0;   // in the image (0..1)
        float x0 = 0, y0 = 0, x1 = 0, y1 = 0;   // pixels from the pen at the baseline (y down)
        float advance = 0;
    };
    // the glyph for a code point (nullptr: not in the atlas -- drawn as nothing, advanced as a space)
    const Glyph* glyph(int codepoint) const;
    float kern(int a, int b) const;
    float measure(const std::string& text) const;
    std::vector<int> codepoints;  // sorted, glyphs[i] is codepoints[i]'s
    std::vector<Glyph> glyphs;
    std::vector<float> kerning;   // n x n, pixels
};

// UTF-8 to code points (an accent is one glyph, not three).
std::vector<int> utf8Codepoints(const std::string& s);

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

    // The string's OUTLINES as SVG path data (y down, baseline's left end at (x, baselineY)), in the
    // same units and metrics as draw(): text on a map or a sign sheet that looks the same in any
    // viewer, with no font to install or embed (a browser or Inkscape ignoring an embedded font laid
    // wider text over a panel sized for this one).
    std::string svgPath(const std::string& text, float x, float baselineY, float pixelHeight, float xScale = 1.0f) const;

    // The glyph atlas at `pixelHeight` (false: no font loaded).
    bool bakeAtlas(float pixelHeight, GlyphAtlas& out) const;

private:
    std::vector<uint8_t> data_;
    std::unique_ptr<stbtt_fontinfo> info_;
};

// The engine's sign/UI face (assets/fonts/Overpass-Bold.ttf), loaded once on
// first use from the asset root. nullptr if the file is missing.
const Font* signFont();
// NEON (storefronts stage 3): a script for bars' signs (Pacifico) and a face drawn as bent tube (Tilt Neon) for
// clubs' and the OPEN signs in shop windows. Both SIL OFL; nullptr if missing.
const Font* neonFont();
const Font* tubeFont();

}  // namespace engine

#endif
