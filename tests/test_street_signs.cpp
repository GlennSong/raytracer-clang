#include "test_framework.h"

#include "../src/engine/procgen/city/road_network.h"
#include "../src/engine/procgen/city/street_names.h"
#include "../src/engine/procgen/city/street_signs.h"
#include "../src/engine/procgen/city/roads/lanes/lane_expand.h"
#include "../src/engine/procgen/city/roads/lanes/road_layout.h"
#include "../src/engine/procgen/city/roads/road_model.h"
#include "../src/engine/text/font.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <iterator>
#include <map>
#include <set>
#include <string>

using namespace engine;

// STREET NAMES AND THEIR SIGNS (Glenn, 2026-09-19: "It would be nice to have in
// world signs for streets ... composite textures and store a list of all the
// street sign textures and map them to the right signs and make sure they fit
// and are legible"). Since ADR-0420 a street's name is the road file's (a run of
// streets that are one street carrying on, named by the planner) and a sign stands
// at a junction of the road model where two named streets meet.

namespace {

// A GRID OF ROADS as a road file: n rows (east-west, 12 m: one 6 m lane each way) and n columns (north-south, 9 m:
// 4.5 m lanes), `pitch` apart, each row and column a road per block, meeting at the grid's points, and running on half a
// block past the grid's edge (so its outer corners are crossings, not bends where a row would carry on as a column).
// Its layout derived as the one-off tool does, and named with `book`.
nlohmann::json gridScene(int n, Real pitch, const StreetNameBook& book) {
    nlohmann::json g;
    g["name"] = "grid";
    g["classes"]["row"] = {{"w", 6.0}, {"fwd", 1}, {"back", 1}, {"sidewalk", 0.0}, {"g_max", 0.1}, {"window", 40}, {"thick", 0.5}};
    g["classes"]["col"] = {{"w", 4.5}, {"fwd", 1}, {"back", 1}, {"sidewalk", 0.0}, {"g_max", 0.1}, {"window", 40}, {"thick", 0.5}};
    nlohmann::json edges = nlohmann::json::array();
    for (int j = 0; j < n; ++j)
        for (int i = -1; i < n; ++i) {
            const Real a = i < 0 ? -0.5 * pitch : i * pitch, b = i + 1 >= n ? (n - 0.5) * pitch : (i + 1) * pitch;
            edges.push_back({{"id", "r" + std::to_string(j) + "_" + std::to_string(i + 1)}, {"class", "row"},
                             {"path", {{"points", {{a, j * pitch}, {b, j * pitch}}}}}});
            edges.push_back({{"id", "c" + std::to_string(j) + "_" + std::to_string(i + 1)}, {"class", "col"},
                             {"path", {{"points", {{j * pitch, a}, {j * pitch, b}}}}}});
        }
    g["edges"] = edges;
    const roads::lanes::RoadLabGraph graph = roads::lanes::RoadLabGraph::fromJson(g);
    roads::lanes::RoadLayout layout = roads::lanes::deriveLayout(graph, roads::lanes::LayoutSource::Inferred);
    roads::lanes::nameStreetRuns(layout.streets, layout.junctions, layout.divided, graph, book);
    roads::lanes::writeLayout(g, layout);
    return g;
}

// The scene's planned road model (as a level loads it).
RoadModel modelOf(const nlohmann::json& scene) {
    roads::lanes::RoadLabGraph g = roads::lanes::RoadLabGraph::fromJson(scene);
    (void)roads::lanes::expand(g);
    return plannedRoadModel(g);
}

// The model's streets as drawn carriageways (what a post keeps clear of): each centre line, its travel width.
RoadGraph carriageways(const RoadModel& m) {
    RoadGraph g;
    for (const ModelStreet& s : m.streets)
        for (uint32_t k = s.line.first; k + 1 < s.line.end(); ++k) {
            g.nodes.push_back(RoadNode{m.linePoints[k]});
            g.nodes.push_back(RoadNode{m.linePoints[k + 1]});
            g.addEdge(static_cast<int>(g.nodes.size()) - 2, static_cast<int>(g.nodes.size()) - 1, s.travelWidth());
        }
    return g;
}

// Every junction of the model where named streets meet, as the loader hands them to the sign plan.
std::vector<SignCorner> cornersOf(const RoadModel& m) {
    std::vector<SignCorner> out;
    for (std::size_t j = 0; j < m.junctions.size(); ++j) {
        SignCorner c;
        c.city = 0;
        c.junction = static_cast<int>(j);
        for (const NamedArm& a : namedArmsAt(m, ModelJunctionId(static_cast<uint32_t>(j))))
            c.arms.push_back({a.arm, a.point, a.dir, a.travelWidth * 0.5, a.name});
        out.push_back(c);
    }
    return out;
}

}  // namespace

TEST_CASE(street_names_follow_a_street_through_its_junctions) {
    const RoadModel m = modelOf(gridScene(4, 80.0, streetNameBook()));
    // 4 rows + 4 columns, each one street end to end (its through pairs) -- not 3 stubs apiece: 8 names
    std::map<std::string, Real> lengthOf;
    for (std::size_t s = 0; s < m.streets.size(); ++s) {
        const std::string name = streetName(m, ModelStreetId(static_cast<uint32_t>(s)));
        CHECK(!name.empty());
        lengthOf[name] += m.streets[s].to - m.streets[s].from;
    }
    CHECK(lengthOf.size() == 8u);   // unique per run
    for (const auto& [name, len] : lengthOf) CHECK(std::fabs(len - 320.0) < 1.0);   // three blocks and two half blocks
    // rows (12 m) and columns (9 m) never share a name
    std::map<std::string, std::set<std::string>> classesOf;
    for (std::size_t s = 0; s < m.streets.size(); ++s)
        classesOf[streetName(m, ModelStreetId(static_cast<uint32_t>(s)))].insert(m.streets[s].section.roadClass);
    for (const auto& [name, cls] : classesOf) CHECK(cls.size() == 1u);
    // deterministic
    const RoadModel again = modelOf(gridScene(4, 80.0, streetNameBook()));
    for (std::size_t s = 0; s < m.streets.size(); ++s)
        CHECK(streetName(again, ModelStreetId(static_cast<uint32_t>(s))) == streetName(m, ModelStreetId(static_cast<uint32_t>(s))));
    CHECK(abbreviateStreetName("Oak Boulevard") == "Oak Blvd");
    CHECK(abbreviateStreetName("Market Street") == "Market St");
    CHECK(abbreviateStreetName("Mill") == "Mill");
}

// THE NAMES ARE CONTENT, NOT CODE (Glenn, 2026-09-20: "I'm starting to wonder
// if some of this shouldn't be data and not baked into the C++").
// assets/data/streets.json holds the words, the suffix a width carries, the classes
// that carry no name and the sign shop's abbreviations; the C++ holds only the algorithm.
TEST_CASE(street_names_come_from_the_streets_asset) {
    // The shipped book really loaded (not the built-in fallback).
    const StreetNameBook& shipped = streetNameBook();
    CHECK(shipped.fromAsset);
    CHECK(shipped.banks.size() >= 2);
    CHECK(shipped.abbreviations.count("Boulevard") == 1);
    CHECK(shipped.unnamed.count("freeway") == 1 && shipped.unnamed.count("ramp") == 1);

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
    const RoadModel m = modelOf(gridScene(4, 80.0, book));
    std::set<std::string> names;
    for (std::size_t s = 0; s < m.streets.size(); ++s) {
        const std::string name = streetName(m, ModelStreetId(static_cast<uint32_t>(s)));
        names.insert(name);
        const std::string word = name.substr(0, name.find(' '));
        const std::string suffix = name.substr(name.find(' ') + 1);
        CHECK(word == "Tabby" || word == "Calico" || word == "Tortoise" || word == "Siamese");
        // Rows are 12 m (Causeway), columns 9 m (Mews).
        CHECK(suffix == (m.streets[s].travelWidth() >= 11 ? "Causeway" : "Mews"));
    }
    CHECK(names.size() == 8u);
    // the generator alone: a class suffix wins over the width; a kept name is never given again
    const std::vector<std::string> two = nameRuns({{100, 20, "alley"}, {50, 5, "local"}}, book, 0, {"Tabby Mews"});
    CHECK(two[0].substr(two[0].find(' ') + 1) == "Ginnel");
    CHECK(two[1] != "Tabby Mews" && two[1].rfind("Tabby", 0) != 0);
    CHECK(abbreviateStreetName("Calico Causeway", book) == "Calico Cswy");
    // The shipped book does not know "Causeway"; a book is self-contained.
    CHECK(abbreviateStreetName("Calico Causeway", shipped) == "Calico Causeway");
}

TEST_CASE(street_signs_stand_on_the_corner_with_the_right_names) {
    const RoadModel m = modelOf(gridScene(4, 80.0, streetNameBook()));
    StreetSignParams p;
    const std::vector<StreetSignPost> posts = planStreetSigns(cornersOf(m), carriageways(m), [](Real, Real) { return 0.0; }, p);
    // Every junction gets a post: the grid's 16 crossings
    CHECK(m.junctions.size() == 16u);
    CHECK(posts.size() == 16u);
    for (const StreetSignPost& post : posts) {
        const ModelJunction& J = m.junctions[static_cast<std::size_t>(post.junction)];
        const Vec2 o = J.at;
        // The blades name exactly the streets that meet here, each attached to an arm that is its street.
        std::set<std::string> want, have;
        for (const NamedArm& a : namedArmsAt(m, ModelJunctionId(static_cast<uint32_t>(post.junction)))) want.insert(a.name);
        for (const StreetSignBlade& b : post.blades) {
            have.insert(b.name);
            CHECK(b.arm >= 0 && streetName(m, J.arms[static_cast<std::size_t>(b.arm)].street) == b.name);
            // Parallel to its street: rows run along X, columns along Z.
            const bool row = m.streets[J.arms[static_cast<std::size_t>(b.arm)].street.index].section.roadClass == "row";
            CHECK(row ? std::fabs(b.along.y) < 1e-6 : std::fabs(b.along.x) < 1e-6);
        }
        CHECK(have == want);
        // On the sidewalk corner: clear of both carriageways, not in the block.
        const Real dx = std::fabs(post.base.x - o.x), dz = std::fabs(post.base.z - o.y);
        CHECK(dz >= 6.0 + p.kerbGap * 0.9);   // the 12 m row's half-width + gap
        CHECK(dx >= 4.5 + p.kerbGap * 0.9);   // the 9 m column's
        CHECK(dx < 4.5 + p.sidewalkWidth + 3.0 && dz < 6.0 + p.sidewalkWidth + 3.0);
    }
    // each blade attached to the model, at the arm it names
    RoadModel withSigns = m;
    int attached = 0;
    for (const StreetSignPost& post : posts)
        for (const StreetSignBlade& b : post.blades) {
            ModelAttachment a;
            a.kind = attachmentKinds().use("name_sign");
            a.junction = ModelJunctionId(static_cast<uint32_t>(post.junction));
            a.arm = b.arm;
            a.text = b.name;
            attached += attach(withSigns, a) ? 1 : 0;
        }
    std::size_t blades = 0;
    for (const StreetSignPost& post : posts) blades += post.blades.size();
    CHECK(attached == static_cast<int>(blades));
}

TEST_CASE(street_sign_blades_fit_and_are_legible) {
    const Font* font = signFont();
    CHECK(font != nullptr);
    if (!font) return;
    // A name no blade holds at full size: it must abbreviate / condense, never clip.
    RoadModel m = modelOf(gridScene(3, 80.0, streetNameBook()));
    std::vector<SignCorner> corners = cornersOf(m);
    std::set<std::string> names;
    for (const SignCorner& c : corners) for (const SignCorner::Arm& a : c.arms) names.insert(a.name);
    CHECK(names.size() >= 2u);
    const std::string first = *names.begin(), second = *std::next(names.begin());
    for (SignCorner& c : corners)
        for (SignCorner::Arm& a : c.arms) {
            if (a.name == first) a.name = "Seventeenth Boulevard";
            else if (a.name == second) a.name = "Exchange Parkway";
        }
    StreetSignParams p;
    const auto posts = planStreetSigns(corners, carriageways(m), [](Real, Real) { return 0.0; }, p);
    const SignAtlas atlas = buildSignAtlas(*font, posts, p);
    CHECK(!atlas.pages.empty());
    const int pad = static_cast<int>(std::lround(p.bladePx * 0.28));
    for (const auto& [s, b] : atlas.blades) {
        std::printf("    [sign] %-24s -> \"%s\" %d px wide, capitals %.1f px%s%s\n", s.c_str(), b.text.c_str(), b.wPx, b.capPx,
                    b.abbreviated ? ", abbreviated" : "", b.xScale < 0.999f ? ", condensed" : "");
        CHECK(b.textPx + 2 * pad <= b.wPx + 1);            // the lettering fits the blade
        CHECK(b.capPx >= 0.40f * p.bladePx);               // and is not shrunk illegible
        CHECK(b.widthM() <= p.maxBladeWidth + 1e-6);
        CHECK(b.u1 > b.u0 && b.v1 > b.v0 && b.u1 <= 1.0f && b.v1 <= 1.0f);
    }
    CHECK(atlas.blades.count("Seventeenth Boulevard") == 1u);
    if (!atlas.blades.count("Seventeenth Boulevard")) return;
    const SignBlade& longest = atlas.blades.at("Seventeenth Boulevard");
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
    const StreetSignMeshes sm = buildStreetSignMeshes(posts, atlas, p);
    std::size_t quads = 0;
    for (const auto& c : sm.blades) quads += c.mesh.indices.size() / 6;
    std::size_t blades = 0;
    for (const auto& post : posts) blades += post.blades.size();
    CHECK(quads == blades * 2);
    CHECK(sm.posts.size() == posts.size());
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
