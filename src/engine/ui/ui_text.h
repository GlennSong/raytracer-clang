#ifndef RAYTRACER_ENGINE_UI_TEXT_H
#define RAYTRACER_ENGINE_UI_TEXT_H

// THE GAME UI'S WORDS (ADR-0250): text, panels and labels as Renderer::UiQuads, for anything a player sees -- the
// tool row, the prompts, the watch. Not ImGui (the debug overlay, which can be hidden and is a developer's).
//
// Text is the sign face (Overpass Bold) baked into a glyph atlas per pixel height on first use and uploaded once:
// a string is one quad per glyph on one texture, crisp at the size it was asked for.

#include "../../renderer/renderer.h"
#include "../text/font.h"

#include <map>
#include <string>
#include <vector>

namespace engine {

class UiText {
public:
    using Quad = Renderer::UiQuad;
    explicit UiText(const Font* font = nullptr) : font_(font) {}

    // How wide `text` comes out at `size` px (the em height).
    float measure(const std::string& text, float size);
    float ascent(float size);
    float descent(float size);
    // `text` with its baseline's left end at (x, baseline).
    void text(Renderer& r, std::vector<Quad>& out, float x, float baseline, const std::string& text, float size,
              const float rgba[4]);
    // A LABEL: `text` centred on cx, its box's top at y, on a backing box `pad` px round it. Returns the box.
    struct Box { float x0 = 0, y0 = 0, x1 = 0, y1 = 0; };
    Box label(Renderer& r, std::vector<Quad>& out, float cx, float y, const std::string& text, float size,
              const float fg[4], const float bg[4], float pad = 8.0f);
    // A plain rectangle in a colour.
    static void rect(std::vector<Quad>& out, float x0, float y0, float x1, float y1, const float rgba[4]);
    // A filled disc, or a ring `width` px thick, of radius r round (cx, cy) (antialiased sprites, made once).
    void disc(Renderer& r, std::vector<Quad>& out, float cx, float cy, float radius, const float rgba[4]);
    void ring(Renderer& r, std::vector<Quad>& out, float cx, float cy, float radius, float width, const float rgba[4]);
    // Give back the atlases' textures (on shutdown, before the renderer goes).
    void release(Renderer& r);

private:
    struct Atlas {
        GlyphAtlas glyphs;
        TextureHandle texture;
    };
    Atlas* atlas(int px);
    const Font* font_;
    std::map<int, Atlas> atlases_;   // by pixel height
    TextureHandle disc_;
    std::map<int, TextureHandle> rings_;   // by thickness, in 1/64ths of the radius
};

}  // namespace engine

#endif
