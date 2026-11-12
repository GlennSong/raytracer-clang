// sign_tool -- the sign kit (procgen/world/sign_kit.h) without a window.
//   sign_tool gallery [--out DIR] [--catalogue FILE] [--px N]
//       every sign in the catalogue at true size on its post, in rows, each labelled on the ground in front of it:
//         the catalogue's examples (rows of ten), a row of STOP and PEDESTRIAN CROSSING worn 0 -> 1, the same examples
//         again as their FAR versions, and a row of mounts (post, two posts, mast arm, overhead span).
//       Writes DIR/sign_gallery.glb (open in Blender: File > Import > glTF 2.0), DIR/sign_sheet.svg (every face drawn
//       flat, a contact sheet), DIR/sign_faces_<n>.png and DIR/sign_shine_<n>.png (the atlas pages: colour, and the
//       night-shine grey the renderer will read), and DIR/sign_gallery.json (each sign's triangles, near and far).
//       DIR defaults to out/signlab.
//   sign_tool list [--catalogue FILE]   the catalogue's signs and examples
#include "engine/asset_root.h"
#include "engine/glb_export.h"
#include "engine/mesh_audit.h"
#include "engine/mesh_builder.h"
#include "engine/procgen/world/road_sign_build.h"
#include "engine/procgen/world/sign_kit.h"
#include "engine/scripting/sign_kit_lua.h"
#include "engine/text/font.h"

#include <tinygltf/stb_image_write.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <vector>

using namespace engine;

namespace {

std::size_t triangles(const SignModel& m) {
    std::size_t n = 0;
    for (const SignModelPart& p : m.parts) n += p.mesh.indices.size() / 3;
    return n;
}

void bounds(const SignModel& m, double& x0, double& x1) {
    x0 = 1e30; x1 = -1e30;
    for (const SignModelPart& p : m.parts)
        for (const Vertex& v : p.mesh.vertices) { x0 = std::min(x0, static_cast<double>(v.position.x)); x1 = std::max(x1, static_cast<double>(v.position.x)); }
    if (x0 > x1) x0 = x1 = 0;
}

int usage() {
    std::fprintf(stderr, "usage: sign_tool gallery [--out DIR] [--catalogue FILE] [--px N]\n       sign_tool list [--catalogue FILE]\n");
    return 2;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) return usage();
    const std::string verb = argv[1];
    std::string out = "out/signlab", catalogue = "signs_us.lua";
    SignKitOptions opt;
    for (int i = 2; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--out") && i + 1 < argc) out = argv[++i];
        else if (!std::strcmp(argv[i], "--catalogue") && i + 1 < argc) catalogue = argv[++i];
        else if (!std::strcmp(argv[i], "--px") && i + 1 < argc) opt.pxPerMetre = std::atof(argv[++i]);
        else return usage();
    }
    SignKit kit;
    std::string err;
    if (!kit.load(catalogue, std::string(), &err)) { std::fprintf(stderr, "sign_tool: %s\n", err.c_str()); return 1; }
    const std::vector<nlohmann::json> examples = kit.examples();
    if (verb == "list") {
        for (const std::string& n : kit.names()) std::printf("%s\n", n.c_str());
        for (const nlohmann::json& e : examples) std::printf("  %s\n", e.dump().c_str());
        return 0;
    }
    if (verb != "gallery") return usage();
    const Font* font = signFont();
    if (!font) { std::fprintf(stderr, "sign_tool: the sign font (assets/fonts/Overpass-Bold.ttf) is missing\n"); return 1; }

    // THE ROWS: each a title and its requests
    struct Row { std::string title; std::vector<nlohmann::json> requests; };
    std::vector<Row> rows;
    for (std::size_t i = 0; i < examples.size(); i += 10) {
        Row r{"catalogue " + std::to_string(i / 10 + 1), {}};
        for (std::size_t k = i; k < std::min(examples.size(), i + 10); ++k) r.requests.push_back(examples[k]);
        rows.push_back(r);
    }
    {
        Row r{"wear 0 -> 1 (seed 3)", {}};
        for (const char* sign : {"stop", "ped_crossing"})
            for (double w : {0.0, 0.25, 0.5, 0.75, 1.0}) r.requests.push_back({{"sign", sign}, {"wear", w}, {"seed", 3}});
        rows.push_back(r);
    }
    for (std::size_t i = 0; i < examples.size(); i += 10) {
        Row r{"far versions " + std::to_string(i / 10 + 1), {}};
        for (std::size_t k = i; k < std::min(examples.size(), i + 10); ++k) {
            nlohmann::json q = examples[k];
            q["far"] = true;
            r.requests.push_back(q);
        }
        rows.push_back(r);
    }
    rows.push_back({"mounts: post, two posts, mast arm, overhead span",
                    {{{"sign", "speed_limit"}, {"value", 35}, {"mount", "post"}},
                     {{"sign", "guide"}, {"lines", {"SALTWOOD", "HARBOR RD"}}, {"exit", 12}, {"arrow", "exit"}, {"mount", "two_post"}},
                     {{"sign", "lane_use"}, {"moves", {"far"}}, {"mount", "mast_arm"}},
                     {{"sign", "street_name"}, {"name", "HARBOR RD"}, {"mount", "mast_arm"}},
                     {{"sign", "guide"}, {"lines", {"DUNWYN"}}, {"arrow", "up"}, {"mount", "overhead"}}}});

    std::vector<nlohmann::json> all;
    for (const Row& r : rows) all.insert(all.end(), r.requests.begin(), r.requests.end());
    SignKitBuild build;
    if (!kit.build(all, *font, build, &err, opt)) { std::fprintf(stderr, "sign_tool: %s\n", err.c_str()); return 1; }

    std::filesystem::create_directories(out);
    for (std::size_t p = 0; p < build.pages.size(); ++p) {
        const std::string a = out + "/sign_faces_" + std::to_string(p) + ".png", b = out + "/sign_shine_" + std::to_string(p) + ".png";
        stbi_write_png(a.c_str(), build.pages[p].w, build.pages[p].h, 4, build.pages[p].rgba.data(), build.pages[p].w * 4);
        stbi_write_png(b.c_str(), build.shinePages[p].w, build.shinePages[p].h, 4, build.shinePages[p].rgba.data(), build.shinePages[p].w * 4);
    }

    // LABELS: a white card on the ground in front of each sign (and each row's title), lettered with the sign font
    std::vector<SignFace> labelFaces;
    auto labelFace = [&](const std::string& line1, const std::string& line2, double w) {
        SignFace f;
        f.w = w; f.h = line2.empty() ? 0.16 : 0.26;
        f.background = "none";   // (the panel's own rounded corners are too round for a card this low)
        SignElem card;
        card.kind = "rect"; card.w = w; card.h = f.h; card.color = "#ffffff";
        f.elems.push_back(card);
        auto text = [&](const std::string& s, double baseline, double cap, const char* color) {
            SignElem t;
            t.kind = "text"; t.x = w / 2; t.y = baseline; t.text = s; t.cap = cap; t.color = color; t.anchor = "middle";
            f.elems.push_back(t);
        };
        text(line1, 0.11, 0.065, "#141414");
        if (!line2.empty()) text(line2, 0.21, 0.05, "#555555");
        labelFaces.push_back(f);
        return static_cast<int>(labelFaces.size()) - 1;
    };

    std::deque<RenderMesh> meshes;   // stable addresses for the GLB entries
    std::vector<GlbEntry> entries;
    RenderMesh labels, ground;
    struct LabelAt { int face; double x, z, w, h; };
    std::vector<LabelAt> labelAt;
    nlohmann::json report = nlohmann::json::array();
    const double rowGap = 9.0, gap = 1.4;
    std::size_t m = 0;
    double widest = 0;
    for (std::size_t r = 0; r < rows.size(); ++r) {
        const double z = -static_cast<double>(r) * rowGap;
        labelAt.push_back({labelFace(rows[r].title, "", 3.2), -4.2, z + 1.0, 3.2, 0.16});
        double cursor = 0;
        for (std::size_t k = 0; k < rows[r].requests.size(); ++k, ++m) {
            SignModel& model = build.models[m];
            double x0 = 0, x1 = 0;
            bounds(model, x0, x1);
            const double dx = cursor - x0;
            const Mat4 at = Mat4::translate(static_cast<Real>(dx), 0, static_cast<Real>(z));
            std::size_t faceTris = 0, metalTris = 0;
            for (const SignModelPart& p : model.parts) {
                meshes.push_back(p.mesh);
                MeshBuilder::transform(meshes.back(), at);
                GlbEntry e;
                e.name = model.label + " / " + p.name;
                e.mesh = &meshes.back();
                // the part's own material, as the catalogue gave it (signs_us.lua `materials`)
                e.color = p.color;
                e.metallic = p.material.metallic;
                e.roughness = p.material.roughness;
                e.materialExtras = {{"rt_material", p.materialName}};
                if (p.page >= 0) {
                    e.texture = &build.pages[static_cast<std::size_t>(p.page)];
                    // the night-shine input (not drawn by anything yet): the grey page with the same layout, and the unit
                    e.materialExtras["rt_retroreflective"] = {{"shine_page", "sign_shine_" + std::to_string(p.page) + ".png"},
                                                              {"ra_white", build.sheeting.value("ra_white", 0.0)}};
                    faceTris += p.mesh.indices.size() / 3;
                } else {
                    metalTris += p.mesh.indices.size() / 3;
                }
                e.extras = model.info;
                entries.push_back(e);
            }
            const double w = std::max(1.0, x1 - x0);
            const std::string tris = std::to_string(triangles(model)) + " triangles" + (model.info.value("far", false) ? " (far)" : "");
            labelAt.push_back({labelFace(model.label, tris, w), cursor, z + 1.0, w, 0.26});
            report.push_back({{"label", model.label}, {"request", all[m]}, {"triangles", triangles(model)}, {"face_triangles", faceTris},
                              {"metal_triangles", metalTris}, {"info", model.info}});
            cursor += (x1 - x0) + gap;
        }
        widest = std::max(widest, cursor);
    }
    // the label cards: packed, then laid flat on the ground, reading from the near (+z) side
    std::vector<TextImage> labelImages;
    for (const SignFace& f : labelFaces) labelImages.push_back(rasterizeSignFace(f, *font, 400.0));
    std::vector<SignImagePlace> places;
    std::vector<TextImage> labelPages = packSignImages(labelImages, 2048, places);
    std::vector<RenderMesh> labelMeshes(labelPages.size());
    for (const LabelAt& l : labelAt) {
        const SignImagePlace& p = places[static_cast<std::size_t>(l.face)];
        if (p.page < 0) continue;
        const float P = 2048.0f;
        const float u0 = (p.x + 2) / P, v0 = (p.y + 2) / P;
        const float u1 = u0 + static_cast<float>(l.w * 400.0) / P, v1 = v0 + static_cast<float>(l.h * 400.0) / P;
        const Real y = 0.01f, x0 = static_cast<Real>(l.x), x1 = static_cast<Real>(l.x + l.w);
        const Real zFar = static_cast<Real>(l.z), zNear = static_cast<Real>(l.z + l.h);
        MeshBuilder::emitQuadUV(labelMeshes[static_cast<std::size_t>(p.page)], Vec3(x0, y, zFar), Vec3(x1, y, zFar), Vec3(x1, y, zNear),
                                Vec3(x0, y, zNear), Vec3(0, 1, 0), Vec3(1, 1, 1), u0, v0, u1, v0, u1, v1, u0, v1);
    }
    for (std::size_t p = 0; p < labelMeshes.size(); ++p) {
        if (labelMeshes[p].indices.empty()) continue;
        GlbEntry e;
        e.name = "labels " + std::to_string(p);
        e.mesh = &labelMeshes[p];
        e.texture = &labelPages[p];
        e.color = Vec3(1, 1, 1);
        entries.push_back(e);
    }
    // a pale ground under it all, 2 cm down
    MeshBuilder::emitBox(ground, Vec3(-6, -0.04, -static_cast<Real>(rows.size()) * rowGap), Vec3(static_cast<Real>(widest + 2), -0.02, 4),
                         Vec3(0.8f, 0.8f, 0.78f));
    GlbEntry floor;
    floor.name = "ground";
    floor.mesh = &ground;
    floor.color = Vec3(1, 1, 1);   // (the gallery's floor and label cards take GlbEntry's plain defaults: not sign materials)
    entries.push_back(floor);

    const std::string glb = out + "/sign_gallery.glb";
    if (!writeGlb(entries, glb, {{"what", "sign kit gallery (sign_tool)"}, {"kerb_side", "left"}}, &err)) {
        std::fprintf(stderr, "sign_tool: %s\n", err.c_str());
        return 1;
    }
    std::ofstream(out + "/sign_gallery.json") << report.dump(1);

    // THE CONTACT SHEET: every distinct face, flat, at 120 px a metre
    {
        const double k = 120.0, pageW = 2000;
        std::string body;
        double x = 20, y = 20, rowH = 0;
        for (std::size_t i = 0; i < build.faces.size(); ++i) {
            const SignFace& f = build.faces[i];
            const double w = f.w * k, h = f.h * k;
            if (x + std::max(w, 160.0) > pageW - 20) { x = 20; y += rowH + 50; rowH = 0; }
            body += signFaceSvg(f, x, y + 20, k) + "\n";
            body += "<text x=\"" + std::to_string(x) + "\" y=\"" + std::to_string(y + 20 + h + 16) +
                    "\" font-size=\"12\" font-family=\"sans-serif\" fill=\"#444\">" + build.faceLabels[i] + "</text>\n";
            x += std::max(w, 160.0) + 30;
            rowH = std::max(rowH, h + 30);
        }
        std::ofstream svg(out + "/sign_sheet.svg");
        svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 " << pageW << ' ' << y + rowH + 60 << "\" width=\"" << pageW
            << "\" height=\"" << y + rowH + 60 << "\">\n<rect width=\"100%\" height=\"100%\" fill=\"#d9d6cf\"/>\n" << body << "</svg>\n";
    }

    // the summary: every model's triangles, and the mesh audit over its parts
    long bad = 0;
    for (std::size_t i = 0; i < build.models.size(); ++i) {
        MeshAudit a;
        for (const SignModelPart& p : build.models[i].parts) a.add(auditMesh(p.mesh));
        if (!a.clean()) ++bad;
        std::printf("%-46s %5zu triangles%s%s\n", build.models[i].label.c_str(), triangles(build.models[i]),
                    build.models[i].info.value("far", false) ? "  far" : "", a.clean() ? "" : "  AUDIT FAILS");
    }
    std::printf("wrote %s (%zu signs, %zu faces on %zu pages), %s/sign_sheet.svg, %s/sign_faces_*.png, %s/sign_shine_*.png\n", glb.c_str(),
                build.models.size(), build.faces.size(), build.pages.size(), out.c_str(), out.c_str(), out.c_str());
    return bad ? 1 : 0;
}
