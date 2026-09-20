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
