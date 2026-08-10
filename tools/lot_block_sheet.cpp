// A CITY BLOCK, end to end, drawn from the shipping modules.
//
// Roads are a hand-authored net run through the REAL engine pipeline
// (RoadGraph -> extractBlocks), so the block faces on this sheet are the same
// faces the engine would produce from those streets. Everything after that is
// the new Lot System: parcel_block cuts the lots, lot_program measures and
// programs them, site_plan allocates the zones, building_recipe builds the
// masses, lot_fixtures places the gates and lights.
//
// This is the test scene, reviewable on paper before there is a mesher to put
// it on screen. The SAME street definition becomes the in-engine level, which
// is the 1:1 property the whole tooling argument rests on.

#include "../src/engine/procgen/city/parcel_block.h"
#include "../src/engine/procgen/city/lot_fixtures.h"
#include "../src/engine/procgen/city/building_recipe.h"
#include "../src/engine/procgen/city/site_plan.h"
#include "../src/engine/procgen/city/road_network.h"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using namespace engine;

namespace {

struct Svg { FILE* f = nullptr; };

void open(Svg& s, const std::string& path, Real w, Real h, const char* title,
          const char* sub) {
    s.f = fopen(path.c_str(), "w");
    fprintf(s.f,
            "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%.0f\" height=\"%.0f\""
            " viewBox=\"0 0 %.0f %.0f\">\n", w, h, w, h);
    fprintf(s.f, "<rect width=\"%.0f\" height=\"%.0f\" fill=\"#11151c\"/>\n", w, h);
    fprintf(s.f, "<text x=\"30\" y=\"44\" fill=\"#e9eef6\" font-family=\"Helvetica,"
                 "Arial,sans-serif\" font-size=\"23\" font-weight=\"600\">%s</text>\n",
            title);
    fprintf(s.f, "<text x=\"30\" y=\"68\" fill=\"#8fa1ba\" font-family=\"Helvetica,"
                 "Arial,sans-serif\" font-size=\"13\">%s</text>\n", sub);
}
void close(Svg& s) { fprintf(s.f, "</svg>\n"); fclose(s.f); }

void label(Svg& s, Real x, Real y, const char* t, const char* fill, int size) {
    fprintf(s.f, "<text x=\"%.1f\" y=\"%.1f\" fill=\"%s\" font-family=\"Helvetica,"
                 "Arial,sans-serif\" font-size=\"%d\">%s</text>\n", x, y, fill, size, t);
}

struct View {
    Real px = 0, py = 0, sc = 1, minx = 0, miny = 0, h = 0;
    Vec2 map(const Vec2& p) const {
        return Vec2(px + (p.x - minx) * sc, py + h - (p.y - miny) * sc);
    }
};

void polyPath(Svg& s, const View& v, const Poly2& p, const char* fill, Real op,
              const char* stroke, Real sw) {
    if (p.size() < 3) return;
    fprintf(s.f, "<path d=\"");
    for (std::size_t i = 0; i < p.size(); ++i) {
        const Vec2 q = v.map(p[i]);
        fprintf(s.f, "%s%.2f %.2f ", i ? "L" : "M", q.x, q.y);
    }
    fprintf(s.f, "Z\" fill=\"%s\" fill-opacity=\"%.2f\" stroke=\"%s\" "
                 "stroke-opacity=\"0.95\" stroke-width=\"%.2f\"/>\n",
            fill, op, stroke, sw);
}

void drawShape(Svg& s, const View& v, const Shape2& sh, const char* fill, Real op,
               const char* stroke, Real sw) {
    const TessShape t = tessellate(sh, 0.06);
    polyPath(s, v, t.outer, fill, op, stroke, sw);
    for (const Poly2& h : t.holes) polyPath(s, v, h, "#11151c", 1.0, stroke, sw);
}

void dot(Svg& s, const View& v, const Vec2& p, Real r, const char* fill) {
    const Vec2 q = v.map(p);
    fprintf(s.f, "<circle cx=\"%.2f\" cy=\"%.2f\" r=\"%.2f\" fill=\"%s\"/>\n",
            q.x, q.y, r, fill);
}

void seg(Svg& s, const View& v, const Vec2& a, const Vec2& b, const char* col,
         Real sw, bool dash = false) {
    const Vec2 p = v.map(a), q = v.map(b);
    fprintf(s.f, "<line x1=\"%.2f\" y1=\"%.2f\" x2=\"%.2f\" y2=\"%.2f\" stroke=\"%s\""
                 " stroke-width=\"%.2f\"%s stroke-linecap=\"round\"/>\n",
            p.x, p.y, q.x, q.y, col, sw, dash ? " stroke-dasharray=\"5 4\"" : "");
}

const char* zoneColor(Zone z) {
    switch (z) {
        case Zone::Building:    return "#e07a5f";
        case Zone::Frontage:    return "#82b29a";
        case Zone::Circulation: return "#cbb4a4";
        case Zone::Open:        return "#4f7c46";
        case Zone::Parking:     return "#6a7480";
        case Zone::Service:     return "#8d6e63";
    }
    return "#888";
}

const char* tagColor(EdgeTag t) {
    switch (t) {
        case EdgeTag::Street: return "#ffd166";
        case EdgeTag::Side:   return "#5b7fa6";
        case EdgeTag::Rear:   return "#7a6f9b";
        case EdgeTag::Party:  return "#ef476f";
        case EdgeTag::Court:  return "#06d6a0";
        case EdgeTag::Lane:   return "#f4a261";
        case EdgeTag::None:   break;
    }
    return "#555";
}

// ---------------------------------------------------------------------------
// The scene: four blocks from eight streets.
// ---------------------------------------------------------------------------
//
// Deliberately hand-authored rather than generated. The metro generator's
// smallest sensible output is ~77 blocks; tuning it down to four would mean
// hunting parameters until it accidentally produced the layout, and it would
// drift the next time anyone retuned it. Eight nodes state the intent exactly.
//
// The four blocks are chosen to exercise DECISIONS, not to look like a city:
// a tight terrace block, a standard residential block, a big downtown plate,
// and a coarse rim block outside the grid.

struct SceneBlock {
    Shape2 shape;
    ProgramSet (*mix)();
    bool enclosed;
    Real coreness;
    StreetClass klass;
    const char* note;
};

RoadGraph buildStreets() {
    RoadGraph g;
    // A 3 x 3 lattice of intersections, with deliberately UNEVEN spacing so the
    // four faces differ in size and each one tests a different program mix.
    const Real xs[3] = {0, 150, 470};
    const Real ys[3] = {0, 115, 360};
    int id[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) id[i][j] = g.addNode(Vec2(xs[i], ys[j]));
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            // The middle horizontal is the ARTERIAL — street class changes
            // retail eligibility, so one row of the grid gets shops.
            if (i + 1 < 3)
                g.addEdge(id[i][j], id[i + 1][j], j == 1 ? 17.0 : 12.0,
                          j == 1 ? RoadClass::Arterial : RoadClass::Local);
            if (j + 1 < 3) g.addEdge(id[i][j], id[i][j + 1], 12.0, RoadClass::Local);
        }
    return g;
}

}  // namespace

int main(int argc, char** argv) {
    const char* out = std::getenv("OUT") ? std::getenv("OUT") : ".";
    const std::uint32_t seed = argc > 1 ? std::atoi(argv[1]) : 3;

    PaletteLibrary palettes = stockPalettes();
    RecipeBook book = stockRecipes();

    // --- roads -> blocks, through the REAL engine pipeline -------------------
    RoadGraph streets = buildStreets();
    std::vector<Poly2> faces = extractBlocks(streets, 200.0);
    printf("streets: %zu nodes, %zu edges -> %zu block faces\n",
           streets.nodes.size(), streets.edges.size(), faces.size());

    // A rim block outside the grid: no far-side street, so `enclosed` is false
    // and the campus-scale programs become eligible (§17.6).
    Shape2 rim = rectShape(520, 0, 760, 200);

    std::vector<SceneBlock> blocks;
    for (const Poly2& f : faces) {
        if (f.size() < 3) continue;
        Shape2 b = shapeFromPoly(f);
        // Pull the block off the carriageway centrelines by a half-width.
        std::vector<Shape2> inner = offsetShape(b, -7.0);
        if (inner.empty()) continue;
        b = inner[0];
        const Real a = area(b);
        SceneBlock sb;
        sb.shape = b;
        sb.enclosed = true;
        sb.coreness = 0.35;
        sb.klass = StreetClass::Street;
        if (a > 40000) {
            sb.mix = downtownPrograms; sb.coreness = 0.9;
            sb.klass = StreetClass::Arterial; sb.note = "downtown plate";
        } else if (a < 14000) {
            sb.mix = residentialPrograms; sb.note = "tight terrace block";
        } else {
            sb.mix = commercialPrograms; sb.klass = StreetClass::Arterial;
            sb.note = "high street (arterial)";
        }
        printf("  face %.0f m2 -> %s\n", a, sb.note);
        blocks.push_back(sb);
    }
    blocks.push_back({rim, rimPrograms, false, 0.1, StreetClass::Street,
                      "rim block (no far-side street)"});

    // --- run the whole new stack on every lot --------------------------------
    struct BuiltLot {
        ParcelledLot parcel;
        SitePlan site;
        SiteFurnishing furn;
        LotFixtures fx;
        std::string program, recipe;
        int storeys = 0;
        bool built = false;
    };
    std::vector<ParcelledBlock> parcelled;
    std::vector<std::vector<BuiltLot>> built;
    int totalLots = 0, totalGates = 0, totalProps = 0, totalStoreys = 0;
    int totalLanes = 0, totalPassages = 0;
    Real totalOpen = 0, totalBlockArea = 0;

    for (std::size_t bi = 0; bi < blocks.size(); ++bi) {
        SceneBlock& sb = blocks[bi];
        ProgramSet set = sb.mix();
        ParcelParams pp;
        ParcelledBlock pb = parcelBlock(sb.shape, set, pp, sb.enclosed, sb.coreness,
                                        sb.klass, seed + (std::uint32_t)bi * 17);
        std::vector<BuiltLot> lots;
        for (std::size_t li = 0; li < pb.lots.size(); ++li) {
            const ParcelledLot& pl = pb.lots[li];
            BuiltLot bl;
            bl.parcel = pl;
            if (pl.program < 0) { lots.push_back(bl); continue; }
            const LotProgram& prog = set.programs[pl.program];
            bl.program = prog.name;
            const std::uint32_t s = seed + (std::uint32_t)(bi * 1000 + li) * 7u + 1u;
            bl.site = layoutSite(pl.shape, pl.tags, prog, s);
            bl.furn = furnishSite(bl.site, prog, s);
            const int budget = std::max(1, std::min((int)pl.tags.maxStoreys,
                                                    prog.maxStoreys));
            const int ri = book.pick(prog, budget, s);
            if (ri >= 0) {
                LotTags capped = pl.tags;
                capped.maxStoreys = budget;
                BuiltBuilding bb = buildFromRecipe(book.recipes[ri],
                                                   bl.site.buildEnvelope, capped,
                                                   palettes, (std::uint32_t)bi + 1, s);
                bl.recipe = bb.recipeName;
                bl.storeys = bb.storeys;
                bl.fx = fixturesFor(bl.site, bl.furn, prog, bb, 0, s);
                bl.built = true;
                totalStoreys += bb.storeys;
            }
            totalGates += (int)bl.site.gates.size();
            totalProps += (int)bl.furn.props.size();
            lots.push_back(std::move(bl));
        }
        totalLots += (int)pb.lots.size();
        for (const BlockLane& l : pb.lanes) (l.passage ? totalPassages : totalLanes)++;
        totalOpen += pb.openArea();
        totalBlockArea += area(sb.shape);
        if (!pb.openSpaceIsReachable())
            printf("  WARNING: block %zu has an unreachable green\n", bi);
        parcelled.push_back(std::move(pb));
        built.push_back(std::move(lots));
    }
    printf("parcelled %zu blocks -> %d lots, %d gates, %d props, %d storeys total\n",
           parcelled.size(), totalLots, totalGates, totalProps, totalStoreys);

    // --- draw ----------------------------------------------------------------
    Vec2 lo(1e30, 1e30), hi(-1e30, -1e30);
    for (const auto& n : streets.nodes) {
        lo.x = std::min(lo.x, n.pos.x); lo.y = std::min(lo.y, n.pos.y);
        hi.x = std::max(hi.x, n.pos.x); hi.y = std::max(hi.y, n.pos.y);
    }
    Vec2 rl, rh;
    bounds(rim, rl, rh);
    lo.x = std::min(lo.x, rl.x); lo.y = std::min(lo.y, rl.y);
    hi.x = std::max(hi.x, rh.x); hi.y = std::max(hi.y, rh.y);
    lo = lo - Vec2(25, 25);
    hi = hi + Vec2(25, 25);

    const Real W = 1900, H = 1240;
    auto makeView = [&](Real px, Real py, Real w, Real h) {
        View v;
        v.sc = std::min((w) / (hi.x - lo.x), (h) / (hi.y - lo.y));
        v.px = px; v.py = py; v.minx = lo.x; v.miny = lo.y; v.h = h;
        return v;
    };

    // SHEET A — the site plan: streets, blocks, lots, zones, buildings.
    {
        Svg s;
        open(s, std::string(out) + "/block_site.svg", W, H,
             "One city block scene — streets to buildings, all shipping modules",
             "extractBlocks on a hand-authored street net, then parcel_block / "
             "lot_program / site_plan / building_recipe / lot_fixtures. "
             "Orange = building, green = open, pale = paths, grey = parking, "
             "yellow dot = gate, teal dot = door.");
        View v = makeView(40, 96, W - 80, H - 190);

        for (const auto& e : streets.edges)
            seg(s, v, streets.nodes[e.a].pos, streets.nodes[e.b].pos, "#2b3442",
                std::max(Real(2), e.width * v.sc));
        for (std::size_t bi = 0; bi < parcelled.size(); ++bi) {
            drawShape(s, v, parcelled[bi].block, "#171d27", 1.0, "#39445a", 1.2);
            for (const Shape2& o : parcelled[bi].openSpace)
                drawShape(s, v, o, "#3d5c38", 0.55, "#4f7c46", 0.8);
            for (const BlockLane& l : parcelled[bi].lanes)
                drawShape(s, v, l.area, l.paved ? "#4a4f5a" : "#6b7f5a", 0.95,
                          l.paved ? "#6c7482" : "#8aa06f", 0.8);
            for (const BuiltLot& bl : built[bi]) {
                for (const ZoneArea& z : bl.site.zones)
                    for (const Shape2& part : z.parts)
                        drawShape(s, v, part, zoneColor(z.zone), 0.5,
                                  zoneColor(z.zone), 0.6);
                drawShape(s, v, bl.parcel.shape, "#000000", 0.0, "#7d8aa0", 0.9);
                for (const PropInstance& p : bl.furn.props) {
                    const char* c = p.kind == PropKind::Tree ? "#2d6a4f"
                                  : p.kind == PropKind::Car ? "#9aa5b1"
                                  : p.kind == PropKind::Bin ? "#6d597a" : "#52796f";
                    dot(s, v, p.at, std::max(Real(1.2), p.radius * v.sc), c);
                }
                for (const InteractableSpec& it : bl.fx.interactables) {
                    if (it.kind == InteractKind::Gate)
                        dot(s, v, Vec2(it.at.x, it.at.z), 3.0, "#ffd166");
                    else if (it.kind == InteractKind::Door)
                        dot(s, v, Vec2(it.at.x, it.at.z), 2.4, "#06d6a0");
                }
            }
        }
        char t[300];
        snprintf(t, sizeof t,
                 "%zu blocks   %d lots   %d storeys   %d service lanes   "
                 "%d passages   %d gates   %.0f%% open space — every shared green "
                 "reachable, every lot passing its program's minimum",
                 parcelled.size(), totalLots, totalStoreys, totalLanes,
                 totalPassages, totalGates,
                 100.0 * totalOpen / std::max(Real(1), totalBlockArea));
        label(s, 40, H - 58, t, "#e9eef6", 15);
        label(s, 40, H - 34,
              "Blocks: tight terrace / high street (arterial) / downtown plate / "
              "rim block. Same streets, different program mixes.", "#8fa1ba", 13);
        close(s);
    }

    // SHEET B — the parcelling, with every lot edge coloured by its TAG.
    {
        Svg s;
        open(s, std::string(out) + "/block_parcels.svg", W, H,
             "The same scene, showing what the parceller decided",
             "Lot fill = its program. Edge colour = its TAG, computed once at cut "
             "time and only read afterwards: yellow street, blue side, "
             "purple rear, RED PARTY WALL (blank by construction), teal court. "
             "Dark green = open space by design, where no program fits.");
        View v = makeView(40, 96, W - 80, H - 190);

        auto progColor = [](const std::string& n) {
            if (n.find("cottage") != std::string::npos) return "#8ab17d";
            if (n.find("villa") != std::string::npos) return "#a3b18a";
            if (n.find("town") != std::string::npos) return "#c9a227";
            if (n.find("walkup") != std::string::npos) return "#d08c60";
            if (n.find("shop") != std::string::npos) return "#e07a5f";
            if (n.find("mixed") != std::string::npos) return "#bc4749";
            if (n.find("office") != std::string::npos) return "#577590";
            if (n.find("tower") != std::string::npos) return "#43aa8b";
            if (n.find("cathedral") != std::string::npos) return "#9d4edd";
            if (n.find("civic") != std::string::npos) return "#7b2cbf";
            if (n.find("campus") != std::string::npos) return "#4d908e";
            if (n.find("box") != std::string::npos) return "#f9c74f";
            if (n.find("works") != std::string::npos) return "#6d6875";
            if (n.find("bank") != std::string::npos) return "#f3722c";
            return "#90a0b0";
        };

        for (const auto& e : streets.edges)
            seg(s, v, streets.nodes[e.a].pos, streets.nodes[e.b].pos, "#2b3442",
                std::max(Real(2), e.width * v.sc));
        for (std::size_t bi = 0; bi < parcelled.size(); ++bi) {
            drawShape(s, v, parcelled[bi].block, "#171d27", 1.0, "#39445a", 1.2);
            for (const Shape2& o : parcelled[bi].openSpace)
                drawShape(s, v, o, "#2f4a2c", 0.85, "#4f7c46", 0.8);
            for (const BlockLane& l : parcelled[bi].lanes)
                drawShape(s, v, l.area, l.paved ? "#4a4f5a" : "#6b7f5a", 0.95,
                          l.paved ? "#6c7482" : "#8aa06f", 0.8);
            for (const BuiltLot& bl : built[bi]) {
                drawShape(s, v, bl.parcel.shape, progColor(bl.program), 0.60,
                          "#0d1117", 0.5);
                // Edge tags: the vocabulary every later layer reads.
                const Loop2& L = bl.parcel.shape.outer;
                for (std::size_t i = 0; i < L.size(); ++i)
                    seg(s, v, L.start(i), L.end(i), tagColor(L.edges[i].tag), 2.2);
            }
        }
        // Legend.
        const char* names[6] = {"street", "side", "rear", "PARTY (blank wall)",
                                "court", "LANE (service back)"};
        const EdgeTag tags[6] = {EdgeTag::Street, EdgeTag::Side, EdgeTag::Rear,
                                 EdgeTag::Party, EdgeTag::Court, EdgeTag::Lane};
        for (int i = 0; i < 6; ++i) {
            const Real x = 40 + i * 210;
            fprintf(s.f, "<line x1=\"%.0f\" y1=\"%.0f\" x2=\"%.0f\" y2=\"%.0f\" "
                         "stroke=\"%s\" stroke-width=\"4\"/>\n",
                    x, H - 60, x + 26, H - 60, tagColor(tags[i]));
            label(s, x + 34, H - 55, names[i], "#c8d3e2", 13);
        }
        int party = 0, court = 0;
        for (const auto& blk : built)
            for (const BuiltLot& bl : blk)
                for (const Edge2& e : bl.parcel.shape.outer.edges) {
                    if (e.tag == EdgeTag::Party) ++party;
                    if (e.tag == EdgeTag::Court) ++court;
                }
        char t[240];
        snprintf(t, sizeof t,
                 "%d party walls and %d court edges found by geometry — not one "
                 "of them chosen by a die.", party, court);
        label(s, 40, H - 30, t, "#8fa1ba", 13);
        close(s);
    }

    printf("wrote %s/block_{site,parcels}.svg\n", out);
    return 0;
}
