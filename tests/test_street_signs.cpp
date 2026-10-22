#include "test_framework.h"

#include "../src/engine/procgen/city/road_network.h"
#include "../src/engine/procgen/city/street_names.h"
#include "../src/engine/procgen/city/street_signs.h"
#include "../src/engine/text/font.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <map>
#include <set>
#include <string>

using namespace engine;

// STREET NAMES AND THEIR SIGNS (Glenn, 2026-09-19: "It would be nice to have in
// world signs for streets ... composite textures and store a list of all the
// street sign textures and map them to the right signs and make sure they fit
// and are legible").

namespace {

// A grid of streets sampled every `step` metres, like metro's polylines: a
// junction every `pitch`, plain nodes between. Rows 12 m wide, columns 9 m.
RoadGraph sampledGrid(int n, Real pitch, Real step) {
    RoadGraph g;
    std::map<std::pair<long, long>, int> at;
    auto node = [&](Real x, Real z) {
        const auto key = std::make_pair(std::lround(x * 10), std::lround(z * 10));
        auto it = at.find(key);
        if (it != at.end()) return it->second;
        g.nodes.push_back(RoadNode{Vec2(x, z)});
        at[key] = static_cast<int>(g.nodes.size()) - 1;
        return static_cast<int>(g.nodes.size()) - 1;
    };
    const int k = static_cast<int>(std::lround(pitch / step));
    for (int j = 0; j < n; ++j)
        for (int i = 0; i + 1 < n; ++i)
            for (int s = 0; s < k; ++s) {
                g.addEdge(node(i * pitch + s * step, j * pitch),
                          node(i * pitch + (s + 1) * step, j * pitch), 12.0);
                g.addEdge(node(j * pitch, i * pitch + s * step),
                          node(j * pitch, i * pitch + (s + 1) * step), 9.0);
            }
    return g;
}

}  // namespace

TEST_CASE(street_names_follow_a_street_through_its_junctions) {
    const RoadGraph g = sampledGrid(4, 80.0, 4.0);
    const StreetNaming n = nameStreets(g);
    // 4 rows + 4 columns, each one street end to end -- not 3 stubs apiece.
    CHECK(n.streets.size() == 8);
    std::set<std::string> names;
    for (const Street& s : n.streets) {
        names.insert(s.name);
        CHECK(std::fabs(s.length - 240.0) < 1e-6);
    }
    CHECK(names.size() == n.streets.size());   // unique
    for (int e = 0; e < static_cast<int>(g.edges.size()); ++e) CHECK(n.streetOf(e) >= 0);
    // Rows (12 m) and columns (9 m) never share a street.
    for (const Street& s : n.streets)
        for (int e : s.edges) CHECK(g.edges[static_cast<std::size_t>(e)].width == s.width);
    // Deterministic.
    const StreetNaming again = nameStreets(g);
    for (std::size_t i = 0; i < n.streets.size(); ++i)
        CHECK(again.streets[i].name == n.streets[i].name);
    CHECK(abbreviateStreetName("Oak Boulevard") == "Oak Blvd");
    CHECK(abbreviateStreetName("Market Street") == "Market St");
    CHECK(abbreviateStreetName("Mill") == "Mill");
}

// THE NAMES ARE CONTENT, NOT CODE (Glenn, 2026-09-20: "I'm starting to wonder
// if some of this shouldn't be data and not baked into the C++").
// assets/data/streets.json holds the words, the suffix a width carries and the
// sign shop's abbreviations; the C++ holds only the algorithm.
TEST_CASE(street_names_come_from_the_streets_asset) {
    // The shipped book really loaded (not the built-in fallback).
    const StreetNameBook& shipped = streetNameBook();
    CHECK(shipped.fromAsset);
    CHECK(shipped.banks.size() >= 2);
    CHECK(shipped.abbreviations.count("Boulevard") == 1);

    // A book authored here names the city from ITS words and suffixes.
    const nlohmann::json doc = nlohmann::json::parse(R"({
        "seed": 7,
        "banks": { "cats": ["Tabby", "Calico", "Tortoise", "Siamese"] },
        "suffixes": [ { "minWidth": 11, "choices": ["Causeway"] },
                      { "minWidth": 0,  "choices": ["Mews"] } ],
        "classSuffix": { "alley": "Ginnel" },
        "abbreviations": { "Causeway": "Cswy" }
    })");
    const StreetNameBook book = parseStreetNameBook(doc);
    CHECK(book.usable());
    const RoadGraph g = sampledGrid(4, 80.0, 4.0);
    const StreetNaming n = nameStreets(g, book);
    CHECK(n.streets.size() == 8);
    for (const Street& s : n.streets) {
        const std::string word = s.name.substr(0, s.name.find(' '));
        const std::string suffix = s.name.substr(s.name.find(' ') + 1);
        CHECK(word == "Tabby" || word == "Calico" || word == "Tortoise" || word == "Siamese");
        // Rows are 12 m (Causeway), columns 9 m (Mews).
        CHECK(suffix == (s.width >= 11 ? "Causeway" : "Mews"));
    }
    CHECK(abbreviateStreetName("Calico Causeway", book) == "Calico Cswy");
    // The shipped book does not know "Causeway"; a book is self-contained.
    CHECK(abbreviateStreetName("Calico Causeway", shipped) == "Calico Causeway");
}

TEST_CASE(street_signs_stand_on_the_corner_with_the_right_names) {
    const RoadGraph g = sampledGrid(4, 80.0, 4.0);
    const StreetNaming n = nameStreets(g);
    StreetSignParams p;
    const std::vector<StreetSignPost> posts =
        planStreetSigns(g, n, [](Real, Real) { return 0.0; }, p);
    // Every junction (degree >= 3) gets a post: 4 corners are degree 2, so
    // 16 - 4 = 12 junctions.
    CHECK(posts.size() == 12);
    for (const StreetSignPost& post : posts) {
        const Vec2 o = g.nodes[static_cast<std::size_t>(post.node)].pos;
        // The blades name exactly the streets that meet here.
        std::set<int> want, have;
        for (int e = 0; e < static_cast<int>(g.edges.size()); ++e) {
            const RoadEdge& ed = g.edges[static_cast<std::size_t>(e)];
            if (ed.a == post.node || ed.b == post.node) want.insert(n.streetOf(e));
        }
        for (const StreetSignBlade& b : post.blades) {
            have.insert(b.street);
            // Parallel to its street: rows run along X, columns along Z.
            const bool row = n.streets[static_cast<std::size_t>(b.street)].width == 12.0;
            CHECK(row ? std::fabs(b.along.y) < 1e-6 : std::fabs(b.along.x) < 1e-6);
        }
        CHECK(have == want);
        // On the sidewalk corner: clear of both carriageways, not in the block.
        const Real dx = std::fabs(post.base.x - o.x), dz = std::fabs(post.base.z - o.y);
        CHECK(dz >= 6.0 + p.kerbGap * 0.9);   // the 12 m row's half-width + gap
        CHECK(dx >= 4.5 + p.kerbGap * 0.9);   // the 9 m column's
        CHECK(dx < 4.5 + p.sidewalkWidth + 3.0 && dz < 6.0 + p.sidewalkWidth + 3.0);
    }
}

TEST_CASE(street_sign_blades_fit_and_are_legible) {
    const Font* font = signFont();
    CHECK(font != nullptr);
    if (!font) return;
    // A name no blade holds at full size: it must abbreviate / condense,
    // never clip.
    RoadGraph g = sampledGrid(3, 80.0, 4.0);
    StreetNaming n = nameStreets(g);
    n.streets[0].name = "Seventeenth Boulevard";
    n.streets[1].name = "Exchange Parkway";
    StreetSignParams p;
    const auto posts = planStreetSigns(g, n, [](Real, Real) { return 0.0; }, p);
    const SignAtlas atlas = buildSignAtlas(*font, n, posts, p);
    CHECK(!atlas.pages.empty());
    const int pad = static_cast<int>(std::lround(p.bladePx * 0.28));
    for (const auto& [s, b] : atlas.blades) {
        std::printf("    [sign] %-24s -> \"%s\" %d px wide, capitals %.1f px%s%s\n",
                    n.streets[static_cast<std::size_t>(s)].name.c_str(), b.text.c_str(), b.wPx,
                    b.capPx, b.abbreviated ? ", abbreviated" : "",
                    b.xScale < 0.999f ? ", condensed" : "");
        CHECK(b.textPx + 2 * pad <= b.wPx + 1);            // the lettering fits the blade
        CHECK(b.capPx >= 0.40f * p.bladePx);               // and is not shrunk illegible
        CHECK(b.widthM() <= p.maxBladeWidth + 1e-6);
        CHECK(b.u1 > b.u0 && b.v1 > b.v0 && b.u1 <= 1.0f && b.v1 <= 1.0f);
    }
    const SignBlade& longest = atlas.blades.at(0);
    CHECK(longest.abbreviated);                              // "Seventeenth Blvd"
    CHECK(longest.text == "Seventeenth Blvd");
    // The lettering is really there: the blade's pixels hold white ink.
    const TextImage& page = atlas.pages[static_cast<std::size_t>(longest.page)];
    const int x0 = static_cast<int>(longest.u0 * page.w), x1 = static_cast<int>(longest.u1 * page.w);
    const int y0 = static_cast<int>(longest.v0 * page.h), y1 = static_cast<int>(longest.v1 * page.h);
    long white = 0;
    for (int y = y0 + p.bladePx / 5; y < y1 - p.bladePx / 5; ++y)
        for (int x = x0 + pad; x < x1 - pad; ++x)
            if (page.rgba[(static_cast<std::size_t>(y) * page.w + x) * 4] > 200) ++white;
    CHECK(white > 200);

    // Meshes: two faces per blade, each a quad.
    const StreetSignMeshes m = buildStreetSignMeshes(posts, atlas, p);
    std::size_t quads = 0;
    for (const auto& c : m.blades) quads += c.mesh.indices.size() / 6;
    std::size_t blades = 0;
    for (const auto& post : posts) blades += post.blades.size();
    CHECK(quads == blades * 2);
    CHECK(m.posts.size() == posts.size());
}

// NEON (storefronts stage 3): a bar's name in script tube, a club's in tube letters -- pale glass on a dark plate by
// day, by night (the glow page) a hot core with its colour's halo round it and the plate black; and the window's OPEN
// sign, red letters in a blue border. RT_SIGN_PNG=<prefix> writes the pages to look at.
#include "../src/engine/procgen/city/shop_signs.h"
#include "../src/engine/procgen/city/trades.h"
#include <tinygltf/stb_image_write.h>
#include <cstdlib>

TEST_CASE(neon_signs_glow_their_colour_and_the_open_sign_is_red_in_blue) {
    const engine::Font* script = engine::neonFont();
    const engine::Font* tube = engine::tubeFont();
    CHECK(script != nullptr);
    CHECK(tube != nullptr);
    if (!script || !tube) return;
    std::vector<engine::ShopSign> signs(2);
    signs[0].text = "The Copper Fox"; signs[0].trade = 14; signs[0].style = 1; signs[0].neon = engine::Vec3(1.0, 0.18, 0.55);
    signs[1].text = "Velvet Room";    signs[1].trade = 15; signs[1].style = 2; signs[1].neon = engine::Vec3(0.2, 0.9, 1.0);
    for (engine::ShopSign& s : signs) { s.width = 6.0; s.height = 0.7; }
    const engine::ShopSignAtlas atlas = engine::buildShopSignAtlas(*script, signs, 2048, tube);
    CHECK((atlas.pages.size()) == (std::size_t(1)));
    CHECK((atlas.glow.size()) == (atlas.pages.size()));
    if (atlas.glow.empty()) return;
    const engine::TextImage& day = atlas.pages[0];
    const engine::TextImage& night = atlas.glow[0];
    CHECK((day.w) == (night.w));
    CHECK((day.h) == (night.h));
    // per board: its tube pixels (the night's brightest), its halo (lit, not tube) and its dark plate
    for (std::size_t i = 0; i < signs.size(); ++i) {
        const engine::ShopSignAtlas::Board& b = atlas.boards[i];
        const int x0 = static_cast<int>(b.u0 * day.w), x1 = static_cast<int>(b.u1 * day.w);
        const int y0 = static_cast<int>(b.v0 * day.h), y1 = static_cast<int>(b.v1 * day.h);
        int core = 0, haloPx = 0, dark = 0;
        double hue[3] = {0, 0, 0};
        for (int y = y0; y < y1; ++y)
            for (int x = x0; x < x1; ++x) {
                const uint8_t* n = &night.rgba[(static_cast<std::size_t>(y) * night.w + x) * 4];
                const int m = std::max({n[0], n[1], n[2]});
                if (m > 200) ++core;
                else if (m > 30) { ++haloPx; for (int k = 0; k < 3; ++k) hue[k] += n[k]; }
                else ++dark;
            }
        const engine::Vec3 c = signs[i].neon;
        std::printf("  neon \"%s\": %d tube px, %d halo px, %d dark; halo rgb %.0f %.0f %.0f\n", signs[i].text.c_str(), core, haloPx, dark,
                    hue[0] / std::max(1, haloPx), hue[1] / std::max(1, haloPx), hue[2] / std::max(1, haloPx));
        CHECK(core > 300);
        CHECK(haloPx > core);          // the halo spreads wider than the tube
        CHECK(dark > haloPx);          // ...and the plate is mostly dark
        // the halo is the sign's colour: its strongest channel is the colour's
        const int want = c.x >= c.y && c.x >= c.z ? 0 : c.y >= c.z ? 1 : 2;
        const int got = hue[0] >= hue[1] && hue[0] >= hue[2] ? 0 : hue[1] >= hue[2] ? 1 : 2;
        CHECK((got) == (want));
    }
    engine::TextImage openDay, openNight;
    engine::openSignImages(*tube, openDay, openNight);
    CHECK(openDay.w > 0 && openNight.w == openDay.w);
    long red = 0, blue = 0;
    for (std::size_t p = 0; p + 3 < openNight.rgba.size(); p += 4) {
        const uint8_t* n = &openNight.rgba[p];
        if (n[0] > 150 && n[0] > n[2] + 60) ++red;
        if (n[2] > 150 && n[2] > n[0] + 60) ++blue;
    }
    std::printf("  OPEN sign: %ld red px, %ld blue px of %d\n", red, blue, openNight.w * openNight.h);
    CHECK(red > 600);
    CHECK(blue > 600);
    if (const char* png = std::getenv("RT_SIGN_PNG")) {
        const std::string pre = png;
        stbi_write_png((pre + "_neon_day.png").c_str(), day.w, day.h, 4, day.rgba.data(), day.w * 4);
        stbi_write_png((pre + "_neon_night.png").c_str(), night.w, night.h, 4, night.rgba.data(), night.w * 4);
        stbi_write_png((pre + "_open_day.png").c_str(), openDay.w, openDay.h, 4, openDay.rgba.data(), openDay.w * 4);
        stbi_write_png((pre + "_open_night.png").c_str(), openNight.w, openNight.h, 4, openNight.rgba.data(), openNight.w * 4);
    }
}
