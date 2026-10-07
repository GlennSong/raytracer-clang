#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_SHOP_SIGNS_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_SHOP_SIGNS_H

// SHOP SIGNS (~/.claude/plans/storefronts-identity.md, stage 2; Glenn: "Maybe we also need signage?" / "all those
// places need to look unique from one another and like the actual places"). Every business (a shop unit of one
// trade: trades.h) gets a NAME of its trade's kind -- invented, never a real chain; Hawaiian, Japanese and American by
// turns for an island at Honolulu's latitude -- lettered in its trade's colour on a dark board over its fascia.
//
// Pure, like street_signs.h: plan the boards from the shopfronts, composite the names into atlas pages
// (font -> TextImage, shelf-packed), build the board quads merged per cell and page. The loader uploads.

#include "../../../renderer/renderer.h"   // RenderMesh, Vertex
#include "../../text/font.h"

#include <cstdint>
#include <string>
#include <vector>

namespace engine {

// A business's name: its trade's patterns, filled from the word lists by `seed` (a hash of its door).
std::string shopName(uint8_t trade, uint32_t seed);

struct ShopSign {
    std::string text;
    uint8_t trade = 0;
    Vec3 centre;        // the board's centre, world
    Vec3 right, up, n;  // the board's axes: along the facade (reads left to right from the street), up, outward
    Real width = 0, height = 0;
};

struct ShopSignAtlas {
    std::vector<TextImage> pages;
    struct Board { int page = 0; float u0 = 0, v0 = 0, u1 = 0, v1 = 0; float widthFrac = 1; };   // widthFrac: of the fascia
    std::vector<Board> boards;   // one per sign, in order
    float smallestCapPx = 0;
};
ShopSignAtlas buildShopSignAtlas(const Font& font, const std::vector<ShopSign>& signs, int pagePx = 2048);

struct ShopSignMeshes {
    struct Cell {
        int page = 0;
        RenderMesh mesh;   // boards, world coordinates
        Vec3 centre;
        Real radius = 0;
    };
    std::vector<Cell> cells;
};
ShopSignMeshes buildShopSignMeshes(const std::vector<ShopSign>& signs, const ShopSignAtlas& atlas, Real cellSize = 200.0);

}  // namespace engine

#endif
