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
// An indoor mall's name ("Kona Galleria", "The Shops at Juniper").
std::string mallName(uint32_t seed);
// A department store's name (an anchor): "Whitfield's", "Kaneshiro & Co.".
std::string anchorName(uint32_t seed);
// A tower's restaurant floors' name ("Sora Sky Dining", "The Hilo Terrace").
std::string skyName(uint32_t seed);

struct ShopSign {
    std::string text;
    uint8_t trade = 0;
    // NEON (storefronts stage 3): 0 lettered board; 1 neon script (a bar's); 2 neon tube letters (a club's). A neon
    // sign is pale glass tube on a dark board by day and its `neon` colour, haloed, by night.
    uint8_t style = 0;
    Vec3 neon{1.0, 0.25, 0.6};
    Vec3 centre;        // the board's centre, world
    Vec3 right, up, n;  // the board's axes: along the facade (reads left to right from the street), up, outward
    Real width = 0, height = 0;
};

struct ShopSignAtlas {
    std::vector<TextImage> pages;
    std::vector<TextImage> glow;   // NEON's emissive pages, laid out as `pages` (empty when no sign is neon)
    struct Board { int page = 0; float u0 = 0, v0 = 0, u1 = 0, v1 = 0; float widthFrac = 1; };   // widthFrac: of the fascia
    std::vector<Board> boards;   // one per sign, in order
    float smallestCapPx = 0;
};
// `tube`: the face of style-2 signs (the script `font` when null).
ShopSignAtlas buildShopSignAtlas(const Font& font, const std::vector<ShopSign>& signs, int pagePx = 2048,
                                 const Font* tube = nullptr);

// A neon sign's colour, by trade and seed: pinks, reds and amber for bars, violet, cyan and pink for clubs.
Vec3 neonColour(uint8_t trade, uint32_t seed);

// THE OPEN SIGN in a shop window: red tube letters in a blue tube border on a dark plate. Its albedo (pale tube by
// day, `alpha` cut round the plate) and its glow (the emissive map, lit only while someone is on shift), and its quad
// (kOpenSignW x kOpenSignH, centred, facing +Z).
constexpr Real kOpenSignW = 0.62, kOpenSignH = 0.27;
void openSignImages(const Font& tube, TextImage& albedo, TextImage& glow);
RenderMesh openSignMesh();

// THE BUS'S DESTINATION SIGN (Glenn, 2026-10-08: "Buses should probably show what line they are and whether they are
// in service on an electric sign on front of the bus"): an amber LED dot-matrix board, kBusSignDotsW x kBusSignDotsH
// dots. One atlas row per label: "12|CLOCKWISE" is the line number big at the left and the words after it; a label
// without a '|' ("NOT IN SERVICE") is the words alone, centred. Albedo: the dark board, its unlit dots; glow: the lit
// dots (the emissive map).
constexpr int kBusSignDotsW = 160, kBusSignDotsH = 16, kBusSignDotPx = 4;
constexpr Real kBusSignW = 2.0, kBusSignH = 0.2;
void busSignImages(const Font& font, const std::vector<std::string>& labels, TextImage& albedo, TextImage& glow);
// The board (kBusSignW x kBusSignH, centred, facing +Z) showing atlas row `row` of `rows`.
RenderMesh busSignMesh(int row, int rows);

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
