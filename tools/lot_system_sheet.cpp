// Draw the NEW Lot System's real output as SVG — no reconstruction, no
// prototype. Every polygon on this sheet comes from the shipping modules:
// lot_program measures the parcel, site_plan allocates the zones, building_
// recipe picks and builds the mass, facade_plan lays out the bays, and
// lot_fixtures decides the gates and lights.
//
//   sh tools/lot_system_build.sh   (link set)
//   c++ ... tools/lot_system_sheet.cpp ... -o /tmp/lot_sheet && OUT=/tmp /tmp/lot_sheet
//
// The point of drawing it is that this environment has no GPU: judging the
// design on paper is the difference between reviewing it and guessing.

#include "../src/engine/procgen/city/lot_fixtures.h"
#include "../src/engine/procgen/city/building_recipe.h"
#include "../src/engine/procgen/city/site_plan.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace engine;

namespace {

struct Svg {
    FILE* f = nullptr;
    Real w = 0, h = 0;
};

void open(Svg& s, const std::string& path, Real w, Real h, const char* title) {
    s.f = fopen(path.c_str(), "w");
    s.w = w;
    s.h = h;
    fprintf(s.f,
            "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%.0f\" "
            "height=\"%.0f\" viewBox=\"0 0 %.0f %.0f\">\n",
            w, h, w, h);
    fprintf(s.f, "<rect width=\"%.0f\" height=\"%.0f\" fill=\"#12161d\"/>\n", w, h);
    fprintf(s.f,
            "<text x=\"28\" y=\"42\" fill=\"#e8edf5\" font-family=\"Helvetica,"
            "Arial,sans-serif\" font-size=\"22\" font-weight=\"600\">%s</text>\n",
            title);
}

void close(Svg& s) {
    fprintf(s.f, "</svg>\n");
    fclose(s.f);
    s.f = nullptr;
}

void label(Svg& s, Real x, Real y, const char* t, const char* fill = "#93a4bd",
           int size = 13) {
    fprintf(s.f,
            "<text x=\"%.1f\" y=\"%.1f\" fill=\"%s\" font-family=\"Helvetica,"
            "Arial,sans-serif\" font-size=\"%d\">%s</text>\n",
            x, y, fill, size, t);
}

// A view transform: world metres -> panel pixels, y flipped so north is up.
struct View {
    Real px = 0, py = 0, sc = 1, minx = 0, miny = 0, h = 0;
    Vec2 map(const Vec2& p) const {
        return Vec2(px + (p.x - minx) * sc, py + h - (p.y - miny) * sc);
    }
};

View fitView(const Shape2& s, Real px, Real py, Real w, Real h, Real pad = 10) {
    Vec2 lo, hi;
    bounds(s, lo, hi);
    View v;
    const Real sx = (w - pad * 2) / std::max(Real(0.01), hi.x - lo.x);
    const Real sy = (h - pad * 2) / std::max(Real(0.01), hi.y - lo.y);
    v.sc = std::min(sx, sy);
    v.px = px + pad;
    v.py = py + pad;
    v.minx = lo.x;
    v.miny = lo.y;
    v.h = h - pad * 2;
    return v;
}

void poly(Svg& s, const View& v, const Poly2& p, const char* fill, Real fillOp,
          const char* stroke, Real sw) {
    if (p.size() < 3) return;
    fprintf(s.f, "<path d=\"");
    for (std::size_t i = 0; i < p.size(); ++i) {
        const Vec2 q = v.map(p[i]);
        fprintf(s.f, "%s%.2f %.2f ", i ? "L" : "M", q.x, q.y);
    }
    fprintf(s.f,
            "Z\" fill=\"%s\" fill-opacity=\"%.2f\" stroke=\"%s\" "
            "stroke-opacity=\"0.95\" stroke-width=\"%.2f\"/>\n",
            fill, fillOp, stroke, sw);
}

void shape(Svg& s, const View& v, const Shape2& sh, const char* fill, Real op,
           const char* stroke, Real sw = 1.2) {
    const TessShape t = tessellate(sh, 0.05);
    poly(s, v, t.outer, fill, op, stroke, sw);
    for (const Poly2& hole : t.holes)
        poly(s, v, hole, "#12161d", 1.0, stroke, sw);
}

void dot(Svg& s, const View& v, const Vec2& p, Real r, const char* fill) {
    const Vec2 q = v.map(p);
    fprintf(s.f, "<circle cx=\"%.2f\" cy=\"%.2f\" r=\"%.2f\" fill=\"%s\"/>\n",
            q.x, q.y, r, fill);
}

void line(Svg& s, const View& v, const Vec2& a, const Vec2& b, const char* col,
          Real sw, const char* dash = nullptr) {
    const Vec2 p = v.map(a), q = v.map(b);
    fprintf(s.f,
            "<line x1=\"%.2f\" y1=\"%.2f\" x2=\"%.2f\" y2=\"%.2f\" stroke=\"%s\" "
            "stroke-width=\"%.2f\"%s/>\n",
            p.x, p.y, q.x, q.y, col, sw,
            dash ? " stroke-dasharray=\"4 3\"" : "");
}

const char* zoneColor(Zone z) {
    switch (z) {
        case Zone::Building:    return "#e07a5f";
        case Zone::Frontage:    return "#81b29a";
        case Zone::Circulation: return "#c9ada7";
        case Zone::Open:        return "#5f8d4e";
        case Zone::Parking:     return "#6b7280";
        case Zone::Service:     return "#8d6e63";
    }
    return "#888";
}

Shape2 streetLot(Real w, Real d, bool corner) {
    Shape2 lot = rectShape(0, 0, w, d);
    lot.outer.edges[0].tag = EdgeTag::Street;
    lot.outer.edges[1].tag = corner ? EdgeTag::Street : EdgeTag::Side;
    lot.outer.edges[2].tag = EdgeTag::Rear;
    lot.outer.edges[3].tag = EdgeTag::Side;
    return lot;
}

}  // namespace

int main(int argc, char** argv) {
    const char* out = std::getenv("OUT") ? std::getenv("OUT") : ".";
    const std::uint32_t seed = argc > 1 ? std::atoi(argv[1]) : 7;

    PaletteLibrary palettes = stockPalettes();
    RecipeBook book = stockRecipes();

    // ---------------------------------------------------------------- sheet 1
    // Six real lots, each measured, programmed, zoned, built and furnished by
    // the shipping modules.
    {
        Svg s;
        const Real PW = 470, PH = 410;
        open(s, std::string(out) + "/lotsystem_sites.svg", PW * 3 + 40, PH * 2 + 110,
             "Lot System — real output: measure, program, zone, build, furnish");
        label(s, 28, 66,
              "Every polygon drawn by lot_program + site_plan + building_recipe + "
              "lot_fixtures. Orange = building envelope, green = open, "
              "pale = paths, grey = parking.");

        struct Case { Real w, d; bool corner; ProgramSet (*mix)(); const char* note; };
        const Case cases[6] = {
            {26, 34, false, residentialPrograms, "mid-block residential"},
            {26, 34, true,  residentialPrograms, "CORNER (same size)"},
            {16, 30, false, residentialPrograms, "narrow terrace"},
            {34, 26, false, commercialPrograms,  "high street"},
            {58, 62, false, downtownPrograms,    "downtown plate"},
            {120, 90, false, rimPrograms,        "rim block (campus scale)"},
        };

        for (int i = 0; i < 6; ++i) {
            const Real px = 20 + (i % 3) * (PW + 5);
            const Real py = 90 + (i / 3) * (PH + 10);
            const Case& c = cases[i];
            Shape2 lot = streetLot(c.w, c.d, c.corner);
            LotTags tags = measureLot(lot, {}, StreetClass::Street, c.w < 100,
                                      i == 4 ? 0.9 : 0.3);
            ProgramSet set = c.mix();
            const int pi = set.pick(tags, seed + i);

            View v = fitView(lot, px, py, PW, PH - 46, 22);
            shape(s, v, lot, "#1c2230", 1.0, "#3d4a60", 1.4);

            char head[256];
            if (pi < 0) {
                snprintf(head, sizeof head, "%s — no program: OPEN SPACE by design",
                         c.note);
                label(s, px + 12, py + PH - 16, head, "#e8edf5", 14);
                continue;
            }
            const LotProgram& prog = set.programs[pi];
            SitePlan site = layoutSite(lot, tags, prog, seed + i);
            SiteFurnishing furn = furnishSite(site, prog, seed + i);

            for (const ZoneArea& z : site.zones)
                for (const Shape2& part : z.parts)
                    shape(s, v, part, zoneColor(z.zone), 0.42, zoneColor(z.zone), 1.0);

            // The height budget is the SMALLER of what the plate can carry and
            // what the program asked for.
            const int budget = std::max(1, std::min((int)tags.maxStoreys,
                                                    prog.maxStoreys));
            const int ri = book.pick(prog, budget, seed + i);
            int storeys = 0;
            if (ri >= 0) {
                LotTags capped = tags;
                capped.maxStoreys = budget;
                BuiltBuilding b = buildFromRecipe(book.recipes[ri], site.buildEnvelope,
                                                  capped, palettes, 5, seed + i);
                storeys = b.storeys;
                // The building's actual ground-floor plan, drawn on the lot.
                if (!b.surfaces.levels.empty())
                    for (const Shape2& p : b.surfaces.levels[0].plans)
                        shape(s, v, p, "#f2cc8f", 0.85, "#2b3140", 1.3);
                LotFixtures fx = fixturesFor(site, furn, prog, b, 0, seed + i);
                for (const InteractableSpec& it : fx.interactables) {
                    const Vec2 at(it.at.x, it.at.z);
                    if (it.kind == InteractKind::Gate) {
                        dot(s, v, at, 4.5, "#ffd166");
                        dot(s, v, Vec2(it.hinge.anchor.x, it.hinge.anchor.z), 2.5,
                            "#ef476f");
                    } else if (it.kind == InteractKind::Door) {
                        dot(s, v, at, 4.0, "#06d6a0");
                    }
                }
                for (const LightSpec& l : fx.lights)
                    if (!l.emissiveOnly) dot(s, v, Vec2(l.at.x, l.at.z), 2.0, "#ffe8a3");
            }
            for (const BoundaryRun& b : site.boundaries)
                line(s, v, b.a, b.b, "#8ecae6", 1.6, "dash");
            for (const PropInstance& p : furn.props) {
                const char* col = p.kind == PropKind::Tree ? "#2d6a4f"
                                : p.kind == PropKind::Car ? "#9aa5b1"
                                : p.kind == PropKind::Bin ? "#6d597a" : "#52796f";
                dot(s, v, p.at, std::max(Real(1.5), p.radius * v.sc), col);
            }

            snprintf(head, sizeof head, "%s  |  %s / %s, %d storeys", c.note,
                     prog.name.c_str(),
                     ri >= 0 ? book.recipes[ri].name.c_str() : "-", storeys);
            label(s, px + 12, py + PH - 30, head, "#e8edf5", 13);
            snprintf(head, sizeof head, "%zu zones, %zu props, %zu gates",
                     site.zones.size(), furn.props.size(), site.gates.size());
            label(s, px + 12, py + PH - 13, head, "#93a4bd", 12);
        }
        close(s);
    }

    // ---------------------------------------------------------------- sheet 2
    // Plan templates and mass stacks: what the grammar can actually draw.
    {
        Svg s;
        const Real PW = 240, PH = 230;
        std::vector<PlanTemplate> tpl = allPlanTemplates();
        const std::size_t COLS = 6;
        const std::size_t rows = (tpl.size() + COLS - 1) / COLS;
        open(s, std::string(out) + "/lotsystem_plans.svg", PW * COLS + 40,
             PH * (rows + 1) + 120,
             "Lot System — plan templates and mass stacks");
        label(s, 28, 66,
              "Every named plan template (plan_grammar), each on a 56 x 40 m "
              "frame. The last row is mass stacks drawn as stacked floor plates "
              "(mass_stack). The COMPOSED plans — from radial-wings on — are "
              "scripts in the grammar's own ops rather than single shapes.");

        for (std::size_t i = 0; i < tpl.size(); ++i) {
            const Real px = 20 + (i % COLS) * PW;
            const Real py = 90 + (i / COLS) * PH;
            Shape2 p = planTemplate(tpl[i], 56, 40, seed + (std::uint32_t)i);
            View v = fitView(p, px, py, PW, PH - 34, 20);
            shape(s, v, p, "#f2cc8f", 0.8, "#2b3140", 1.4);
            char t[128];
            snprintf(t, sizeof t, "%s  (%.0f m2)", planTemplateName(tpl[i]), area(p));
            label(s, px + 10, py + PH - 12, t, "#e8edf5", 12);
        }

        // Mass stacks: draw every level's plan, faintly, so the silhouette reads.
        struct Stack { const char* name; MassStack (*make)(); };
        const Real rowY = 90 + rows * PH;
        Shape2 base = rectShape(0, 0, 34, 26);
        Shape2 disc = circleShape(Vec2(17, 13), 15);
        struct Item { const char* name; MassStack st; };
        std::vector<Item> items;
        items.push_back({"simple (6)", simpleStack(base, 6)});
        items.push_back({"wedding cake", weddingCake(base, 4, 4)});
        items.push_back({"gherkin (profile)", gherkin(disc, 26)});
        items.push_back({"pyramid (taper)", pyramid(base, 14)});
        items.push_back({"arch (loft merges)",
                         archTower(rectShape(0, 0, 10, 24),
                                   rectShape(30, 0, 40, 24), 14)});
        for (std::size_t i = 0; i < items.size(); ++i) {
            const Real px = 20 + i * PW;
            std::vector<Level> lv = expandLevels(items[i].st);
            if (lv.empty()) continue;
            Shape2 widest = lv[0].plans[0];
            for (const Level& L : lv)
                for (const Shape2& p : L.plans)
                    if (area(p) > area(widest)) widest = p;
            View v = fitView(widest, px, rowY, PW, PH - 34, 24);
            for (std::size_t k = 0; k < lv.size(); ++k) {
                const Real t = lv.size() > 1 ? (Real)k / (lv.size() - 1) : 0;
                char col[16];
                snprintf(col, sizeof col, "#%02x%02x%02x", (int)(90 + t * 150),
                         (int)(140 + t * 80), (int)(200 - t * 60));
                for (const Shape2& p : lv[k].plans)
                    shape(s, v, p, col, 0.16, col, 0.7);
            }
            char t[160];
            snprintf(t, sizeof t, "%s — %zu levels, %.0f m", items[i].name, lv.size(),
                     items[i].st.height());
            label(s, px + 10, rowY + PH - 12, t, "#e8edf5", 12);
        }
        close(s);
    }

    // ---------------------------------------------------------------- sheet 3
    // Kernel proofs: the operations that everything above rests on.
    {
        Svg s;
        const Real PW = 300, PH = 270;
        open(s, std::string(out) + "/lotsystem_kernel.svg", PW * 4 + 40, PH * 2 + 120,
             "Lot System — 2-D kernel (shape2 / shape_ops)");
        label(s, 28, 66,
              "Arcs survive booleans and offsets as TRUE circles. Curves here are "
              "SVG arc commands, not chord fans.");

        auto panel = [&](int idx, const char* title,
                         const std::vector<Shape2>& shapes,
                         const std::vector<Shape2>& ghosts) {
            const Real px = 20 + (idx % 4) * PW;
            const Real py = 90 + (idx / 4) * PH;
            Shape2 all = shapes.empty() ? Shape2{} : shapes[0];
            Vec2 lo(1e30, 1e30), hi(-1e30, -1e30);
            auto grow = [&](const Shape2& sh) {
                Vec2 a, b;
                bounds(sh, a, b);
                lo.x = std::min(lo.x, a.x); lo.y = std::min(lo.y, a.y);
                hi.x = std::max(hi.x, b.x); hi.y = std::max(hi.y, b.y);
            };
            for (const Shape2& sh : shapes) grow(sh);
            for (const Shape2& sh : ghosts) grow(sh);
            Shape2 frame = rectShape(lo.x, lo.y, hi.x, hi.y);
            View v = fitView(frame, px, py, PW, PH - 34, 22);
            for (const Shape2& g : ghosts) shape(s, v, g, "#3d4a60", 0.25, "#4a5876", 0.9);
            for (const Shape2& sh : shapes) shape(s, v, sh, "#81b29a", 0.55, "#e8edf5", 1.5);
            label(s, px + 10, py + PH - 12, title, "#e8edf5", 12);
        };

        Shape2 discA = circleShape(Vec2(0, 0), 12);
        Shape2 tab = rectShape(10, -3, 18, 3);
        panel(0, "unite: 3 quarter-arcs untouched", unite(discA, tab), {discA, tab});
        panel(1, "subtract: sub-arc, same circle",
              subtract(discA, rectShape(-20, -20, 20, 0)), {discA});
        Shape2 plate = rectShape(0, 0, 40, 40);
        panel(2, "subtract: a real hole",
              subtract(plate, rectShape(14, 14, 26, 26)), {plate});
        panel(3, "offset +3: concentric circle", offsetShape(discA, 3), {discA});

        Shape2 uShape = subtract(rectShape(0, 0, 40, 30),
                                 rectShape(12, 8, 28, 31))[0];
        panel(4, "offset -4.5: U SPLITS in two", offsetShape(uShape, -4.5), {uShape});
        Shape2 sq = rectShape(0, 0, 20, 20);
        panel(5, "roundToCircle(1.0) = pi r^2", {roundToCircle(sq, 1.0)}, {sq});
        panel(6, "filletAll(4): true tangent arcs", {filletAll(sq, 4)}, {sq});
        Shape2 barA = rectShape(0, 0, 20, 10), barB = rectShape(18, 0, 38, 10);
        panel(7, "smoothUnion(k=3): blended join",
              smoothUnion(barA, barB, 3.0, 0.2), {barA, barB});
        close(s);
    }

    printf("wrote %s/lotsystem_{sites,plans,kernel}.svg\n", out);
    return 0;
}
