#include "test_framework.h"
#include "../src/engine/procgen/world/place_names.h"
#include "../src/engine/procgen/world/road_signs.h"
#include "../src/engine/text/font.h"

#include <cmath>
#include <map>
#include <set>

using namespace engine;

// PLACES AND SIGNS (Glenn, 2026-09-25: "freeway signs. Like how far each town is ... signs for which
// side of the freeway we should enter to go in the right direction"). Names are the world's, the
// same every time; a sign's lettering fits its panel; and on a ring freeway the two ramps at an
// interchange are signed for two different places, the advance signs never reach back past the
// interchange before, and the distances count up.

TEST_CASE(place_names_are_the_same_every_time_unique_and_shaped_by_their_site) {
    const PlaceNameBook& book = placeNameBook();
    CHECK(book.usable());
    std::set<std::string> a, b;
    std::vector<std::string> first;
    for (uint32_t k = 0; k < 9; ++k) {
        PlaceTraits t;
        t.mountain = k == 8;
        t.coastal = k % 2 == 0;
        first.push_back(placeName(book, t, 8, k, a));
    }
    for (uint32_t k = 0; k < 9; ++k) {
        PlaceTraits t;
        t.mountain = k == 8;
        t.coastal = k % 2 == 0;
        CHECK(placeName(book, t, 8, k, b) == first[k]);   // same world, same names
    }
    CHECK(a.size() == 9);                                  // no two places share a name
    // over many mountain towns, most take a mountain form (the book's chance is 0.8)
    int shaped = 0;
    std::set<std::string> used;
    for (uint32_t k = 0; k < 40; ++k) {
        PlaceTraits t;
        t.mountain = true;
        const std::string n = placeName(book, t, 99, k, used);
        for (const std::string& end : book.mountain.suffix) if (n.size() > end.size() && n.compare(n.size() - end.size(), end.size(), end) == 0) ++shaped;
        for (const std::string& end : book.mountain.endings) if (n.size() > end.size() && n.compare(n.size() - end.size(), end.size(), end) == 0) ++shaped;
    }
    CHECK(shaped > 20);
}

TEST_CASE(road_sign_lettering_fits_its_panel) {
    const Font* font = signFont();
    CHECK(font != nullptr);
    if (!font) return;
    IslandSign s;
    s.kind = "exit";
    s.legend = {{"exit", "26A"}, {"dests", nlohmann::json::array({"Carrcombe Harbor Crossing", "Saltwood"})}, {"arrow", "up-left"}};
    for (const char* kind : {"exit", "advance", "distance", "entrance", "limit", "trailblazer", "gore"}) {
        s.kind = kind;
        if (s.kind == "advance") s.legend["dist"] = "2 km";
        if (s.kind == "distance") s.legend = {{"rows", nlohmann::json::array({nlohmann::json::array({"Carrcombe Harbor", "12"}), nlohmann::json::array({"Weyby", "4"})})}};
        if (s.kind == "entrance") s.legend = {{"route", "1"}, {"dir", "Outer Loop"}, {"city", "Carrcombe Harbor"}, {"arrow", "left"}};
        if (s.kind == "limit") s.legend = {{"name", "Carrcombe Harbor"}, {"pop", 32000}, {"kind", "city"}};
        if (s.kind == "trailblazer") s.legend = {{"route", "2"}, {"dest", "Carrcombe Harbor"}, {"dist", "9 km"}};
        if (s.kind == "gore") s.legend = {{"exit", "126B"}};
        const SignFace f = layoutSign(s);
        for (const SignElem& e : f.elems) {
            if (e.kind != "text") continue;
            const double w = font->measure(e.text, 100.0f) * e.cap / font->capHeight(100.0f);
            const double x0 = e.anchor == "middle" ? e.x - w / 2 : e.anchor == "end" ? e.x - w : e.x;
            CHECK(x0 >= 0.0);
            CHECK(x0 + w <= f.w + 1e-6);
            CHECK(e.y <= f.h);
        }
    }
}

namespace {
// A ring freeway, 4 km across, three interchanges on it with a place each, their ramps a diamond's:
// on each carriageway the off-ramp leaves 250 m before the crossing and the on-ramp joins 250 m after.
IslandWorld ringWorld() {
    IslandWorld w;
    const double R = 2000.0, kPi = 3.14159265358979;
    for (int i = 0; i <= 400; ++i) w.freewayRoute.push_back(Vec2(R * std::cos(2 * kPi * i / 400), R * std::sin(2 * kPi * i / 400)));
    w.routeClockwise = false;   // counter-clockwise with increasing station
    const double L = 2 * kPi * R;
    const char* kinds[3] = {"city", "town", "city"};
    const char* names[3] = {"Ashford", "Wickham", "Oakby"};
    for (int k = 0; k < 3; ++k) {
        const double s = L * (k + 0.2) / 3.0, th = s / R;
        const Vec2 dir(std::cos(th), std::sin(th)), t(-std::sin(th), std::cos(th));
        IslandSite site;
        site.kind = kinds[k];
        site.name = names[k];
        site.at = dir * (R - 900.0);
        w.sites.push_back(site);
        IslandRoad road;
        road.kind = "link";
        road.from = k;
        road.street = std::string("Main St");
        road.points = {dir * (R - 500.0), dir * R, dir * (R + 90.0)};
        w.roads.push_back(road);
        IslandInterchange ic;
        ic.road = k;
        ic.site = k;
        ic.station = s;
        ic.at = dir * R;
        ic.exit = std::to_string(k * 4 + 1);
        for (int d : {+1, -1})
            for (bool off : {true, false}) {
                IslandInterchange::Ramp rp;
                rp.off = off;
                rp.withRoute = d > 0;
                rp.gore = s + (off ? -250.0 : 250.0) * d;
                const double gth = rp.gore / R;
                rp.gorePt = Vec2(R * std::cos(gth), R * std::sin(gth));
                // a ramp runs from its gore to the road, on its carriageway's side (+1: the -normal side,
                // which on this counter-clockwise ring is outside)
                const Vec2 side = dir * (d > 0 ? 1.0 : -1.0);
                rp.terminal = dir * R + side * 35.0;
                // an off-ramp runs gore -> road, an on-ramp road -> gore (as diamondRamps lays them)
                const Vec2 a = off ? rp.gorePt : rp.terminal, b = off ? rp.terminal : rp.gorePt;
                for (int q = 0; q <= 10; ++q) rp.path.push_back(a + (b - a) * (q / 10.0));
                ic.ramps.push_back(rp);
            }
        w.interchanges.push_back(ic);
    }
    return w;
}
}  // namespace

TEST_CASE(ring_freeway_signs_say_which_ramp_goes_where_and_how_far) {
    IslandWorld w = ringWorld();
    planIslandSigns(w);
    CHECK(!w.signs.empty());
    int entrances = 0, advances = 0, distances = 0;
    std::map<int, std::set<std::string>> entranceCities;   // interchange -> the places its ramps are signed for
    for (const IslandSign& s : w.signs) {
        if (s.kind == "entrance") {
            ++entrances;
            // the interchange it stands at
            int k = 0;
            for (int j = 1; j < 3; ++j) if ((w.interchanges[j].at - s.at).length() < (w.interchanges[k].at - s.at).length()) k = j;
            entranceCities[k].insert(s.legend.value("city", std::string()) + "|" + s.legend.value("dir", std::string()));
            CHECK(!s.legend.value("city", std::string()).empty());
        }
        if (s.kind == "advance") ++advances;
        if (s.kind == "distance") {
            ++distances;
            int last = 0;
            for (const auto& r : s.legend["rows"]) { const int d = std::stoi(r[1].get<std::string>()); CHECK(d >= last); last = d; }
        }
    }
    CHECK(entrances == 6);    // one per on-ramp
    CHECK(distances == 6);    // one after each on-ramp
    CHECK(advances > 0);
    // at each interchange its two ramps are signed for two directions AND two different places
    for (const auto& [k, set] : entranceCities) {
        CHECK(set.size() == 2);
        std::set<std::string> cities;
        for (const std::string& e : set) cities.insert(e.substr(0, e.find('|')));
        CHECK(cities.size() == 2);
    }
    // exits 4.2 km apart: both the 2 km and the 1 km sign fit before each, each way
    CHECK(advances == 12);
}

// IN 3D (ADR-0110): the faces bake into atlas pages that are cached by content -- the second bake of
// the same signs is read back, not lettered again -- and each panel's texture runs to its READER's
// right in the world (u along cross(forward, up)), so the plan/world mirror never reverses a word.
#include "../src/engine/procgen/world/road_sign_build.h"
#include <filesystem>
TEST_CASE(road_sign_atlas_is_cached_and_panels_read_left_to_right) {
    const Font* font = signFont();
    CHECK(font != nullptr);
    if (!font) return;
    IslandWorld w = ringWorld();
    planIslandSigns(w);
    const std::string dir = (std::filesystem::temp_directory_path() / "rt_roadsign_cache_test").string();
    std::filesystem::remove_all(dir);
    const RoadSignAtlas a = bakeRoadSignAtlas(w.signs, *font, dir, 40.0, 1024);
    CHECK(!a.fromCache);
    CHECK(a.slots.size() == w.signs.size());
    for (const RoadSignAtlas::Slot& s : a.slots) { CHECK(s.page >= 0); CHECK(s.u1 > s.u0); CHECK(s.v1 > s.v0); CHECK(s.w > 0.5); }
    const RoadSignAtlas b = bakeRoadSignAtlas(w.signs, *font, dir, 40.0, 1024);
    CHECK(b.fromCache);
    CHECK(b.key == a.key);
    CHECK(b.pages.size() == a.pages.size());
    CHECK(b.slots.size() == a.slots.size());
    if (!b.slots.empty()) CHECK(std::fabs(b.slots[0].u0 - a.slots[0].u0) < 1e-6f);
    std::filesystem::remove_all(dir);
    // one sign, read by traffic heading -z: its panel faces +z, and u grows toward +x (the reader's right)
    IslandSign s;
    s.kind = "limit";
    s.at = Vec2(0, 0);
    s.facing = Vec2(0, -1);
    s.mount = "roadside";
    s.legend = {{"name", "Ashford"}, {"pop", 1200}, {"kind", "town"}};
    const RoadSignAtlas one = bakeRoadSignAtlas({s}, *font, {}, 40.0, 1024);
    const RoadSignMeshes m = buildRoadSignMeshes({s}, one, [](double, double) { return 0.0; });
    CHECK(m.panels.size() == 1);
    if (m.panels.size() == 1) {
        const auto& vs = m.panels[0].mesh.vertices;
        CHECK(vs.size() >= 4);
        double xAtU0 = 0, xAtU1 = 0;
        for (const auto& v : vs) {
            if (std::fabs(v.u - one.slots[0].u0) < 1e-5f) xAtU0 = v.position.x;
            if (std::fabs(v.u - one.slots[0].u1) < 1e-5f) xAtU1 = v.position.x;
            CHECK(v.normal.z > 0.9f);   // it faces the traffic coming at it
        }
        CHECK(xAtU1 > xAtU0);
    }
    CHECK(!m.steel.empty());
}
