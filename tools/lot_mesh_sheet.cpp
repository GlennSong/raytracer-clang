// THE MESHED BUILDINGS, drawn — a software render of the Lot System bridge.
//
// There is no GPU on the machine this is generated on, and "it compiles and the
// triangle count is plausible" is not a check that a building looks like a
// building. So: project every triangle isometrically, sort back to front, flat
// shade by normal, write SVG. It catches inverted winding, missing walls,
// floating slabs and z-fighting bands — all the things a headless test cannot
// see and a screenshot would show instantly.
//
// This is the same argument the SVG plan sheets already won: reviewable on
// paper beats unreviewable in principle.

#include "../src/engine/procgen/city/lot_mesh.h"
#include "../src/engine/procgen/city/parcel_block.h"
#include "../src/engine/procgen/city/material_set.h"

#include <algorithm>
#include <cmath>
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
    fprintf(s.f, "<rect width=\"%.0f\" height=\"%.0f\" fill=\"#12161d\"/>\n", w, h);
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

// A face ready to draw: screen-space corners, depth, and its shaded colour.
struct Face {
    float x[3], y[3];
    float depth;
    unsigned char r, g, b;
};

// Isometric camera: yaw around Y, then tilt. Depth is distance along the view
// direction, which is all a painter's sort needs.
struct Cam {
    Real yaw = 0.72, tilt = 0.62, scale = 1.0;
    Vec2 origin{0, 0};
    // TOWARD the camera. A face is visible when its normal points this way —
    // getting the sign backwards culls exactly the faces you meant to keep and
    // draws the inside of the building instead, which looks convincingly like a
    // meshing bug until you check the camera.
    Vec3 toEye() const {
        return Vec3(std::sin(yaw) * std::cos(tilt), std::sin(tilt),
                    std::cos(yaw) * std::cos(tilt));
    }
    void project(const Vec3& p, float& sx, float& sy, float& d) const {
        const Real cx = std::cos(yaw), sxx = std::sin(yaw);
        const Real rx = p.x * cx - p.z * sxx;           // right
        const Real rz = p.x * sxx + p.z * cx;           // into the screen
        const Real up = p.y * std::cos(tilt) - rz * std::sin(tilt);
        sx = static_cast<float>(origin.x + rx * scale);
        sy = static_cast<float>(origin.y - up * scale);
        d = static_cast<float>(rz * std::cos(tilt) + p.y * std::sin(tilt));
    }
};

void collect(const RenderMesh& m, const Cam& cam, std::vector<Face>& out) {
    const Vec3 sun = normalize(Vec3(0.45, 0.78, 0.35));
    for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
        const Vertex& a = m.vertices[m.indices[i]];
        const Vertex& b = m.vertices[m.indices[i + 1]];
        const Vertex& c = m.vertices[m.indices[i + 2]];
        // BACKFACE CULL exactly as the engine would: its winding convention has
        // cross(b - a, c - a) pointing AGAINST the shading normal (see
        // MeshBuilder::emitQuad), so the face normal is the negated cross
        // product. Culling the other way round draws the inside of every
        // building — which looks convincingly like a meshing bug for about an
        // hour.
        const Vec3 gn = cross(b.position - a.position, c.position - a.position);
        if (gn.lengthSquared() < 1e-12) continue;
        const Vec3 n = normalize(gn) * Real(-1);
        if (dot(n, cam.toEye()) <= 0) continue;
        Face f;
        float d0, d1, d2;
        cam.project(a.position, f.x[0], f.y[0], d0);
        cam.project(b.position, f.x[1], f.y[1], d1);
        cam.project(c.position, f.x[2], f.y[2], d2);
        f.depth = (d0 + d1 + d2) / 3.0f;
        const Real lam = std::max(Real(0.18), dot(n, sun));
        const Vec3 col = a.color;
        auto ch = [&](Real v) {
            return static_cast<unsigned char>(
                std::max(Real(0), std::min(Real(255), v * lam * 255.0 * 1.15)));
        };
        f.r = ch(col.x); f.g = ch(col.y); f.b = ch(col.z);
        out.push_back(f);
    }
}

void draw(Svg& s, std::vector<Face>& faces) {
    std::sort(faces.begin(), faces.end(),
              [](const Face& a, const Face& b) { return a.depth > b.depth; });
    for (const Face& f : faces)
        fprintf(s.f,
                "<path d=\"M%.1f %.1f L%.1f %.1f L%.1f %.1f Z\" fill=\"#%02x%02x%02x\""
                " shape-rendering=\"crispEdges\"/>\n",
                f.x[0], f.y[0], f.x[1], f.y[1], f.x[2], f.y[2], f.r, f.g, f.b);
}

int triCount(const BuildingMesh& m) {
    int t = 0;
    for (const RenderMesh& p : m.parts) t += static_cast<int>(p.indices.size() / 3);
    return t;
}

// Fit a camera so a building of this height and footprint fills its cell.
Cam fitCam(const BuildingMesh& m, Real px, Real py, Real cw, Real ch) {
    Vec3 lo(1e30, 1e30, 1e30), hi(-1e30, -1e30, -1e30);
    for (const RenderMesh& p : m.parts)
        for (const Vertex& v : p.vertices) {
            lo.x = std::min(lo.x, v.position.x); hi.x = std::max(hi.x, v.position.x);
            lo.y = std::min(lo.y, v.position.y); hi.y = std::max(hi.y, v.position.y);
            lo.z = std::min(lo.z, v.position.z); hi.z = std::max(hi.z, v.position.z);
        }
    Cam cam;
    if (lo.x > hi.x) return cam;
    const Real spanXZ = std::max(hi.x - lo.x, hi.z - lo.z) * 1.42;
    const Real spanY = (hi.y - lo.y) + spanXZ * 0.5;
    cam.scale = std::min(cw / std::max(spanXZ, Real(1)),
                         ch / std::max(spanY, Real(1))) * 0.92;
    // Re-centre on the projected bounds.
    Cam probe = cam;
    probe.origin = Vec2(0, 0);
    Real minx = 1e30, maxx = -1e30, miny = 1e30, maxy = -1e30;
    const Vec3 corners[8] = {{lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z}, {lo.x, lo.y, hi.z},
                             {hi.x, lo.y, hi.z}, {lo.x, hi.y, lo.z}, {hi.x, hi.y, lo.z},
                             {lo.x, hi.y, hi.z}, {hi.x, hi.y, hi.z}};
    for (const Vec3& c : corners) {
        float sx, sy, d;
        probe.project(c, sx, sy, d);
        minx = std::min(minx, (Real)sx); maxx = std::max(maxx, (Real)sx);
        miny = std::min(miny, (Real)sy); maxy = std::max(maxy, (Real)sy);
    }
    cam.origin = Vec2(px + cw * 0.5 - (minx + maxx) * 0.5,
                      py + ch * 0.5 - (miny + maxy) * 0.5);
    return cam;
}

}  // namespace

int main(int argc, char** argv) {
    const char* out = std::getenv("OUT") ? std::getenv("OUT") : ".";
    const std::uint32_t seed = argc > 1 ? std::atoi(argv[1]) : 5;

    PaletteLibrary palettes = stockPalettes();
    RecipeBook book = stockRecipes();

    // A representative spread: a house, a terrace, a shop, a block, towers, a
    // landmark, a shed — and the composed plans, which are the ones whose
    // geometry has never been through a mesher.
    struct Pick { const char* recipe; Real w, d; int storeys; };
    const Pick picks[] = {
        {"cottage_house", 11, 13, 2},   {"townhouse", 7, 16, 3},
        {"shopfront", 10, 18, 2},       {"corner_shopfront", 16, 16, 3},
        {"walkup", 20, 22, 5},          {"apartment_slab", 26, 16, 8},
        {"office_block", 34, 30, 9},    {"mixed_use", 24, 30, 6},
        {"office_tower", 40, 38, 24},   {"glass_tower", 46, 44, 38},
        {"stepped_tower", 42, 40, 30},  {"profile_tower", 40, 40, 34},
        {"church", 24, 40, 2},          {"cathedral", 40, 60, 3},
        {"civic_hall", 46, 52, 4},      {"museum", 44, 46, 3},
        {"hotel", 36, 34, 12},          {"school", 44, 36, 3},
        {"hospital", 54, 46, 8},        {"university_campus", 70, 60, 4},
        {"factory", 44, 34, 2},         {"warehouse", 40, 30, 1},
        {"market_hall", 36, 30, 2},     {"pyramid_landmark", 40, 40, 8},
    };
    const int N = static_cast<int>(sizeof(picks) / sizeof(picks[0]));
    const int COLS = 6;
    const int ROWS = (N + COLS - 1) / COLS;
    const Real CW = 330, CH = 340;

    Svg s;
    open(s, std::string(out) + "/lotmesh_buildings.svg", CW * COLS + 40,
         CH * ROWS + 110, "The Lot System, meshed",
         "Every building below is BuiltBuilding -> meshBuilding -> triangles, "
         "drawn by a painter's-algorithm software render (no GPU here). "
         "Backfaces are culled against the GEOMETRIC normal, so a wall wound "
         "the wrong way vanishes instead of hiding.");

    int totalTris = 0;
    for (int i = 0; i < N; ++i) {
        const Pick& p = picks[i];
        const LotRecipe* rec = book.byName(p.recipe);
        if (!rec) continue;
        Shape2 envelope = rectShape(0, 0, p.w, p.d);
        for (Edge2& e : envelope.outer.edges) e.tag = EdgeTag::Side;
        envelope.outer.edges[0].tag = EdgeTag::Street;
        LotTags tags = measureLot(envelope, {}, StreetClass::Street, true, 0.7);
        tags.maxStoreys = p.storeys;
        BuiltBuilding bb = buildFromRecipe(*rec, envelope, tags, palettes, 3,
                                           seed + static_cast<std::uint32_t>(i) * 17u);
        LotMeshParams mp;
        mp.seed = seed + i;
        mp.retail = rec->placeType == "shop";
        BuildingMesh bm = meshBuilding(bb, mp);
        totalTris += triCount(bm);

        const Real px = 20 + (i % COLS) * CW;
        const Real py = 92 + (i / COLS) * CH;
        Cam cam = fitCam(bm, px, py, CW - 20, CH - 46);
        std::vector<Face> faces;
        for (const RenderMesh& part : bm.parts) collect(part, cam, faces);
        draw(s, faces);

        char t[200];
        snprintf(t, sizeof t, "%s — %d storeys, %s, %d tris", p.recipe, bb.storeys,
                 planTemplateName(bb.plan), triCount(bm));
        label(s, px + 12, py + CH - 16, t, "#dbe4f0", 12);
    }
    close(s);

    // Sheet 2: the LOD twin and the HLOD proxy, side by side with the full
    // detail — the R2 swap has to be invisible at distance and much cheaper.
    {
        Svg l;
        const char* names[3] = {"full detail", "flat LOD (R2)", "HLOD proxy"};
        open(l, std::string(out) + "/lotmesh_lod.svg", CW * 3 + 40, CH * 2 + 110,
             "The same building at three detail levels",
             "Full punches openings and reveals; flat reads the SAME opening "
             "list as bands; the proxy is the ground plate extruded. The "
             "triangle counts are the point.");
        const char* two[2] = {"glass_tower", "walkup"};
        const Real dims[2][3] = {{46, 44, 34}, {20, 22, 5}};
        for (int r = 0; r < 2; ++r) {
            const LotRecipe* rec = book.byName(two[r]);
            if (!rec) continue;
            Shape2 envelope = rectShape(0, 0, dims[r][0], dims[r][1]);
            for (Edge2& e : envelope.outer.edges) e.tag = EdgeTag::Side;
            envelope.outer.edges[0].tag = EdgeTag::Street;
            LotTags tags = measureLot(envelope, {}, StreetClass::Street, true, 0.8);
            tags.maxStoreys = static_cast<int>(dims[r][2]);
            BuiltBuilding bb = buildFromRecipe(*rec, envelope, tags, palettes, 3, seed);
            for (int c = 0; c < 3; ++c) {
                LotMeshParams mp;
                mp.detail = c == 1 ? FacadeDetail::Flat : FacadeDetail::Full;
                BuildingMesh bm = meshBuilding(bb, mp);
                const Real px = 20 + c * CW, py = 92 + r * CH;
                std::vector<Face> faces;
                if (c == 2) {
                    Cam cam = fitCam(bm, px, py, CW - 20, CH - 46);
                    collect(bm.proxy, cam, faces);
                    draw(l, faces);
                    char t[160];
                    snprintf(t, sizeof t, "%s — %s, %d tris", two[r], names[c],
                             static_cast<int>(bm.proxy.indices.size() / 3));
                    label(l, px + 12, py + CH - 16, t, "#dbe4f0", 12);
                } else {
                    Cam cam = fitCam(bm, px, py, CW - 20, CH - 46);
                    for (const RenderMesh& part : bm.parts) collect(part, cam, faces);
                    draw(l, faces);
                    char t[160];
                    snprintf(t, sizeof t, "%s — %s, %d tris", two[r], names[c],
                             triCount(bm));
                    label(l, px + 12, py + CH - 16, t, "#dbe4f0", 12);
                }
            }
        }
        close(l);
    }

    printf("wrote %s/lotmesh_{buildings,lod}.svg — %d triangles across %d buildings\n",
           out, totalTris, N);
    return 0;
}
