// THE GAME UI'S WORDS (ADR-0250) and THE WATCH's readouts (ADR-0251).

#include "test_framework.h"

#include "../src/engine/systems/watch_system.h"
#include "../src/engine/text/font.h"
#include "../src/engine/ui/ui_text.h"

#include <cmath>
#include <memory>

using namespace engine;

TEST_CASE(a_glyph_atlas_holds_every_printable_glyph_and_measures_like_the_font) {
    const Font* f = signFont();
    CHECK(f != nullptr);
    if (!f) return;
    GlyphAtlas a;
    CHECK(f->bakeAtlas(24.0f, a));
    CHECK(a.image.w == 512 && a.image.h >= 16);
    for (int c = 33; c <= 126; ++c) {
        const GlyphAtlas::Glyph* g = a.glyph(c);
        CHECK(g != nullptr);
        if (!g) continue;
        CHECK(g->x1 > g->x0 && g->y1 > g->y0);              // ink
        CHECK(g->u1 > g->u0 && g->v1 > g->v0 && g->u1 <= 1.0f && g->v1 <= 1.0f);
        CHECK(g->y0 < 0.0f || c == '_' || c == ',' || c == '.');   // above the baseline (y down), but for the low ones
    }
    CHECK(a.glyph(0xB0) != nullptr);   // the degree sign
    CHECK(a.glyph(0x4E00) == nullptr);   // (not every glyph there is)
    // the atlas's measure is the font's (the same advances and kerning)
    for (const char* s : {"N", "340 m", "1.2 km", "14:05", "Space   open the parachute", "AVAWAY"})
        CHECK_APPROX(a.measure(s), f->measure(s, 24.0f), 0.01);
    // glyphs don't overlap in the image (2 px gutters)
    for (std::size_t i = 0; i < a.glyphs.size(); ++i)
        for (std::size_t j = i + 1; j < a.glyphs.size(); ++j) {
            const GlyphAtlas::Glyph &p = a.glyphs[i], &q = a.glyphs[j];
            if (p.u1 <= p.u0 || q.u1 <= q.u0) continue;
            const bool apart = p.u1 <= q.u0 || q.u1 <= p.u0 || p.v1 <= q.v0 || q.v1 <= p.v0;
            CHECK(apart);
        }
}

TEST_CASE(ui_text_lays_out_a_line_as_glyph_quads_on_one_texture) {
    std::unique_ptr<Renderer> r = Renderer::create();
    UiText ui;
    std::vector<Renderer::UiQuad> q;
    const float white[4] = {1, 1, 1, 1};
    ui.text(*r, q, 100.0f, 200.0f, "Hi there", 20.0f, white);
    CHECK(q.size() == 7);   // the space has no quad
    for (const Renderer::UiQuad& g : q) {
        CHECK(g.texture == q.front().texture);
        CHECK(g.y[0] < 200.0f && g.y[2] <= 205.0f);   // on the baseline
    }
    for (std::size_t i = 1; i < q.size(); ++i) CHECK(q[i].x[0] > q[i - 1].x[0]);   // left to right
    CHECK_APPROX(q.back().x[1] - 100.0f, ui.measure("Hi there", 20.0f), 3.0);
    // a label: its box round the words, then the words
    q.clear();
    const float back[4] = {0, 0, 0, 0.5f};
    const UiText::Box b = ui.label(*r, q, 400.0f, 50.0f, "jump", 18.0f, white, back);
    CHECK(q.size() == 5);
    CHECK(!q.front().texture.valid());
    CHECK_APPROX((b.x0 + b.x1) * 0.5f, 400.0f, 1.0);
    for (std::size_t i = 1; i < q.size(); ++i) CHECK(q[i].x[0] >= b.x0 && q[i].x[1] <= b.x1 && q[i].y[2] <= b.y1);
    ui.release(*r);
}

TEST_CASE(the_watch_reads_distance_and_time_as_a_person_would) {
    CHECK(WatchSystem::distanceText(0) == "0 m");
    CHECK(WatchSystem::distanceText(340.4) == "340 m");
    CHECK(WatchSystem::distanceText(994) == "994 m");
    CHECK(WatchSystem::distanceText(1234) == "1.2 km");
    CHECK(WatchSystem::distanceText(12600) == "13 km");
    CHECK(WatchSystem::clockText(14.09) == "14:05");
    CHECK(WatchSystem::clockText(0) == "00:00");
    CHECK(WatchSystem::clockText(23.999) == "23:59");
    CHECK(WatchSystem::clockText(25.5) == "01:30");
}

TEST_CASE(the_watch_dial_turns_with_the_heading_and_puts_the_waypoint_on_its_edge) {
    std::unique_ptr<Renderer> r = Renderer::create();
    WatchSystem w;
    // the waypoint's dot: the gold disc the dial draws, found by its colour and size
    auto dot = [&](Real heading, Real bearing, float& x, float& y) {
        std::vector<Renderer::UiQuad> q;
        WatchSystem::Face f;
        f.headingDeg = heading;
        f.bearingDeg = bearing;
        f.distance = 300;
        w.drawDial(*r, q, 200.0f, f);
        for (const Renderer::UiQuad& g : q)
            if (g.r == 1.0f && g.g > 0.7f && g.g < 0.8f && g.b < 0.2f && std::fabs(g.x[1] - g.x[0] - 15.0f) < 0.01f &&
                std::fabs(g.y[2] - g.y[0] - 15.0f) < 0.01f) {   // (the 7.5 px dot, not the gold digits)
                x = (g.x[0] + g.x[1]) * 0.5f;
                y = (g.y[0] + g.y[2]) * 0.5f;
                return true;
            }
        return false;
    };
    float x = 0, y = 0;
    CHECK(dot(0, 0, x, y));        // facing north, the waypoint north: straight up the dial
    CHECK_APPROX(x, 100.0, 0.5);
    CHECK(y < 30.0f);
    CHECK(dot(90, 0, x, y));       // facing east, it is to the left
    CHECK(x < 30.0f);
    CHECK_APPROX(y, 100.0, 0.5);
    CHECK(dot(0, 180, x, y));      // behind: the bottom
    CHECK(y > 170.0f);
}
