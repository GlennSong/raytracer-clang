#include "test_framework.h"
#include "../src/engine/mesh_audit.h"
#include "../src/engine/mesh_builder.h"
#include "../src/engine/procgen/world/sign_kit.h"
#include "../src/engine/scripting/sign_kit_lua.h"
#include "../src/engine/text/font.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace engine;

// THE SIGN KIT (procgen/world/sign_kit.h; the catalogue assets/scripts/signs_us.lua over sign_kit.lua). Every sign in
// the catalogue builds, near and far; its meshes pass the engine's mesh audit and its plates are closed; the same wear
// and seed give the same bytes; a sign added in Lua alone builds; the side a turn sign shows comes from the data.

namespace {

std::vector<nlohmann::json> everyRequest(SignKit& kit) {
    std::vector<nlohmann::json> out;
    for (const nlohmann::json& e : kit.examples()) {
        out.push_back(e);
        nlohmann::json far = e;
        far["far"] = true;
        out.push_back(far);
    }
    for (const std::string& n : kit.names()) out.push_back({{"sign", n}});   // every sign alone, plaques too
    out.push_back({{"sign", "speed_limit"}, {"value", 45}, {"mount", "mast_arm"}});
    out.push_back({{"sign", "guide"}, {"lines", {"DUNWYN"}}, {"arrow", "up"}, {"mount", "overhead"}});
    out.push_back({{"sign", "no_turn"}, {"turn", "far"}, {"wear", 1.0}, {"seed", 11}});
    out.push_back({{"sign", "stop"}, {"wear", 0.8}, {"seed", 4}, {"plaques", {{{"sign", "all_way"}}}}});
    return out;
}

std::size_t tris(const SignModel& m) {
    std::size_t n = 0;
    for (const SignModelPart& p : m.parts) n += p.mesh.indices.size() / 3;
    return n;
}

bool sameBytes(const RenderMesh& a, const RenderMesh& b) {
    return a.vertices.size() == b.vertices.size() && a.indices == b.indices &&
           std::memcmp(a.vertices.data(), b.vertices.data(), a.vertices.size() * sizeof(Vertex)) == 0;
}

}  // namespace

TEST_CASE(sign_kit_every_catalogue_sign_builds_near_and_far) {
    const Font* font = signFont();
    CHECK(font != nullptr);
    if (!font) return;
    SignKit kit;
    std::string err;
    CHECK(kit.load("signs_us.lua", std::string(), &err));
    if (!err.empty()) std::printf("    %s\n", err.c_str());
    const std::vector<std::string> names = kit.names();
    CHECK(names.size() >= 20);
    const std::vector<nlohmann::json> reqs = everyRequest(kit);
    SignKitBuild b;
    CHECK(kit.build(reqs, *font, b, &err));
    if (!err.empty()) std::printf("    %s\n", err.c_str());
    CHECK(b.models.size() == reqs.size());
    CHECK(!b.pages.empty() && b.pages.size() == b.shinePages.size());
    for (std::size_t i = 0; i < b.models.size(); ++i) {
        const SignModel& m = b.models[i];
        int faces = 0, backs = 0, posts = 0;
        for (const SignModelPart& p : m.parts) {
            if (p.page >= 0) ++faces;
            backs += p.materialName == "plate_back" ? 1 : 0;
            posts += p.materialName == "post" ? 1 : 0;
            CHECK(!p.mesh.indices.empty());
            CHECK(!p.materialName.empty());
            if (p.page >= 0) CHECK(p.materialName == "face");
        }
        CHECK(faces >= 1);
        CHECK(backs == 1);
        CHECK(posts == 1);
        CHECK(!m.label.empty());
        CHECK(m.info.contains("means"));   // what it stands for: the planting finds it by this
    }
    // every example's far version is cheaper than its near one
    const std::size_t n = kit.examples().size();
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t near = tris(b.models[2 * i]), far = tris(b.models[2 * i + 1]);
        if (far >= near) std::printf("    %s: far %zu >= near %zu\n", b.models[2 * i].label.c_str(), far, near);
        CHECK(far < near);
    }
}

TEST_CASE(sign_kit_meshes_pass_the_mesh_audit_and_plates_are_closed) {
    const Font* font = signFont();
    if (!font) { CHECK(false); return; }
    SignKit kit;
    std::string err;
    CHECK(kit.load("signs_us.lua", std::string(), &err));
    const std::vector<nlohmann::json> reqs = everyRequest(kit);
    SignKitBuild b;
    CHECK(kit.build(reqs, *font, b, &err));
    for (std::size_t i = 0; i < b.models.size(); ++i) {
        const SignModel& m = b.models[i];
        MeshAudit a;
        RenderMesh all;
        for (const SignModelPart& p : m.parts) {
            a.add(auditMesh(p.mesh));
            MeshBuilder::append(all, p.mesh);
        }
        if (!a.clean()) printMeshAudit(m.label.c_str(), a);
        CHECK(a.clean());
        // WELDED: every plate (its face, rim and back) and every post, clamp and arm is a closed solid
        const std::size_t open = openEdges(all);
        if (open) std::printf("    %s: %zu open edges\n", m.label.c_str(), open);
        CHECK(open == 0);
        // the faces look at the reader (+z) unless worn
        if (reqs[i].value("wear", 0.0) == 0.0)
            for (const SignModelPart& p : m.parts) {
                if (p.page < 0) continue;
                for (const Vertex& v : p.mesh.vertices) CHECK(v.normal.z > 0.99);
            }
    }
}

TEST_CASE(sign_kit_same_seed_same_bytes) {
    const Font* font = signFont();
    if (!font) { CHECK(false); return; }
    const std::vector<nlohmann::json> reqs = {
        {{"sign", "stop"}, {"wear", 0.7}, {"seed", 5}},
        {{"sign", "ped_crossing"}, {"wear", 0.4}, {"seed", 9}},
        {{"sign", "speed_limit"}, {"value", 30}, {"wear", 1.0}, {"seed", 2}},
    };
    SignKitBuild a, b, c;
    std::string err;
    {
        SignKit kit;
        CHECK(kit.load("signs_us.lua", std::string(), &err));
        CHECK(kit.build(reqs, *font, a, &err));
    }
    {
        SignKit kit;
        CHECK(kit.load("signs_us.lua", std::string(), &err));
        CHECK(kit.build(reqs, *font, b, &err));
        std::vector<nlohmann::json> other = reqs;
        other[0]["seed"] = 6;
        CHECK(kit.build(other, *font, c, &err));
    }
    CHECK(a.pages.size() == b.pages.size());
    for (std::size_t p = 0; p < a.pages.size() && p < b.pages.size(); ++p) {
        CHECK(a.pages[p].rgba == b.pages[p].rgba);
        CHECK(a.shinePages[p].rgba == b.shinePages[p].rgba);
    }
    CHECK(a.models.size() == b.models.size());
    for (std::size_t i = 0; i < a.models.size() && i < b.models.size(); ++i) {
        CHECK(a.models[i].parts.size() == b.models[i].parts.size());
        for (std::size_t k = 0; k < a.models[i].parts.size() && k < b.models[i].parts.size(); ++k) {
            CHECK(sameBytes(a.models[i].parts[k].mesh, b.models[i].parts[k].mesh));
            CHECK(a.models[i].parts[k].material.roughness == b.models[i].parts[k].material.roughness);
            CHECK(a.models[i].parts[k].color.x == b.models[i].parts[k].color.x);
        }
    }
    // another seed wears it differently: its pixels, and its plate's bend or lean
    bool pixelsDiffer = a.pages.size() != c.pages.size();
    for (std::size_t p = 0; !pixelsDiffer && p < a.pages.size(); ++p) pixelsDiffer = a.pages[p].rgba != c.pages[p].rgba;
    CHECK(pixelsDiffer);
    CHECK(!sameBytes(a.models[0].parts.back().mesh, c.models[0].parts.back().mesh));
    // wear 0 leaves the face exactly as drawn: no texel of a new sign is touched
    TextImage img;
    const uint8_t red[4] = {184, 28, 34, 255};
    img.resize(8, 8, red);
    const TextImage before = img;
    wearSignFace(img, nullptr, 0.0, 77, SignWearLook(), 256.0);
    CHECK(img.rgba == before.rgba);
}

TEST_CASE(sign_kit_a_sign_defined_only_in_lua_builds) {
    const Font* font = signFont();
    if (!font) { CHECK(false); return; }
    // a new warning sign, written here and nowhere else: no C++ knows its name
    const std::string source = R"(
        local C = require "signs_us"
        table.insert(C.signs, {
          name = "test_deer_crossing", code = "W11-3", title = "DEER CROSSING", means = { warns = "animals" },
          shape = "diamond", size = { 30 }, corner = 1.875, plate = "yellow",
          border = { color = "black", width = 0.75, inset = 0.5 },
          legend = function(f) f:text("DEER", { y = 3, cap = 5, series = "C" }); f:text("XING", { y = -4, cap = 5, series = "C" }) end,
        })
        return C
    )";
    SignKit kit;
    std::string err;
    CHECK(kit.loadSource(source, std::string(), &err));
    if (!err.empty()) std::printf("    %s\n", err.c_str());
    bool listed = false;
    for (const std::string& n : kit.names()) listed = listed || n == "test_deer_crossing";
    CHECK(listed);
    SignKitBuild b;
    CHECK(kit.build({{{"sign", "test_deer_crossing"}}, {{"sign", "test_deer_crossing"}, {"far", true}}}, *font, b, &err));
    CHECK(b.models.size() == 2);
    if (b.models.size() == 2) {
        CHECK(b.models[0].label == "DEER CROSSING (W11-3)");
        CHECK(b.models[0].info.value("code", std::string()) == "W11-3");
        MeshAudit a;
        RenderMesh all;
        for (const SignModelPart& p : b.models[0].parts) { a.add(auditMesh(p.mesh)); MeshBuilder::append(all, p.mesh); }
        CHECK(a.clean());
        CHECK(openEdges(all) == 0);
    }
}

namespace {
const SignModelPart* partNamed(const SignModel& m, const std::string& material) {
    for (const SignModelPart& p : m.parts) if (p.materialName == material) return &p;
    return nullptr;
}
}  // namespace

TEST_CASE(sign_kit_materials_come_from_the_catalogue_and_metal_ages) {
    const Font* font = signFont();
    if (!font) { CHECK(false); return; }
    const std::vector<nlohmann::json> reqs = {{{"sign", "stop"}}, {{"sign", "stop"}, {"wear", 1.0}, {"seed", 3}}};
    SignKitBuild a;
    std::string err;
    {
        SignKit kit;
        CHECK(kit.load("signs_us.lua", std::string(), &err));
        CHECK(kit.build(reqs, *font, a, &err));
    }
    CHECK(a.models.size() == 2);
    if (a.models.size() != 2) return;
    const SignModelPart* post = partNamed(a.models[0], "post");
    const SignModelPart* back = partNamed(a.models[0], "plate_back");
    const SignModelPart* face = partNamed(a.models[0], "face");
    CHECK(post && back && face);
    if (!post || !back || !face) return;
    // a DULL metal: metallic, with a high roughness (not a mirror); the face is not a metal
    for (const SignModelPart* p : {post, back}) {
        CHECK(p->material.metallic >= 0.9f);
        CHECK(p->material.roughness >= 0.6f && p->material.roughness <= 0.75f);
        // the base colour rides on the mesh too, as every procgen part's does
        CHECK(std::fabs(p->mesh.vertices[0].color.x - p->color.x) < 1e-5);
    }
    CHECK(face->material.metallic == 0.0f);
    // wear ages the metal: rougher and darker, from the same 0..1 number
    const SignModelPart* oldPost = partNamed(a.models[1], "post");
    CHECK(oldPost != nullptr);
    if (oldPost) {
        CHECK(oldPost->material.roughness > post->material.roughness);
        CHECK(oldPost->color.x < post->color.x);
    }
    // a material changed in Lua alone changes the result: no C++ involved
    const std::string source = R"(
        local C = require "signs_us"
        C.materials.post = { color = { 0.2, 0.3, 0.4 }, metallic = 0.5, roughness = 0.31, source = "test" }
        return C
    )";
    SignKit kit;
    CHECK(kit.loadSource(source, std::string(), &err));
    SignKitBuild b;
    CHECK(kit.build({reqs[0]}, *font, b, &err));
    const SignModelPart* p = b.models.empty() ? nullptr : partNamed(b.models[0], "post");
    CHECK(p != nullptr);
    if (p) {
        CHECK(std::fabs(p->material.roughness - 0.31f) < 1e-6);
        CHECK(std::fabs(p->material.metallic - 0.5f) < 1e-6);
        CHECK(std::fabs(p->color.z - 0.4) < 1e-6);
        CHECK(std::fabs(p->mesh.vertices[0].color.z - 0.4) < 1e-5);
    }
}

namespace {
// How many of `solid`'s closed solids hold `p`: the faces a ray from p crosses, counted +1 leaving a solid and -1
// entering one (a point inside two overlapping solids, a clamp round a post, counts 2; outside, 0).
int solidsHolding(const RenderMesh& solid, const Vec3& lo, const Vec3& hi, const Vec3& p) {
    if (p.x < lo.x || p.y < lo.y || p.z < lo.z || p.x > hi.x || p.y > hi.y || p.z > hi.z) return 0;
    const Vec3 d = normalize(Vec3(0.1234, 0.9771, 0.1713));
    int w = 0;
    for (std::size_t t = 0; t + 2 < solid.indices.size(); t += 3) {
        const Vec3& a = solid.vertices[solid.indices[t]].position;
        const Vec3& b = solid.vertices[solid.indices[t + 1]].position;
        const Vec3& c = solid.vertices[solid.indices[t + 2]].position;
        Real hit = 0;
        if (rayTriangle(p, d, a, b, c, hit) && hit > 1e-9) w += dot(cross(c - a, b - a), d) > 0 ? 1 : -1;
    }
    return w;
}
}  // namespace

// A WORN PLATE STAYS OUT OF ITS POST (Glenn, in the gallery: the bent STOP signs went through the post). Every sign, new
// and worn, over several seeds and on every mount: no point of a plate (its corners, and a grid over each of its
// triangles, taken 1 mm inside the plate so a plate's back resting on the post's face is not counted) lies inside the
// post, a clamp, an arm or a span.
TEST_CASE(sign_kit_worn_plates_stay_out_of_their_posts) {
    const Font* font = signFont();
    if (!font) { CHECK(false); return; }
    SignKit kit;
    std::string err;
    CHECK(kit.load("signs_us.lua", std::string(), &err));
    std::vector<nlohmann::json> reqs;
    for (const std::string& n : kit.names()) {
        reqs.push_back({{"sign", n}});
        for (double w : {0.5, 1.0})
            for (int seed : {1, 2, 3, 4}) reqs.push_back({{"sign", n}, {"wear", w}, {"seed", seed}});
    }
    for (const char* mount : {"post", "two_post", "mast_arm", "overhead"})
        for (const char* sign : {"stop", "speed_limit", "guide"})
            for (double w : {0.0, 0.5, 1.0})
                for (int seed : {1, 2, 3}) reqs.push_back({{"sign", sign}, {"mount", mount}, {"wear", w}, {"seed", seed}});
    reqs.push_back({{"sign", "stop"}, {"wear", 1.0}, {"seed", 5}, {"plaques", {{{"sign", "all_way"}}}}});
    SignKitBuild b;
    CHECK(kit.build(reqs, *font, b, &err));
    CHECK(b.models.size() == reqs.size());
    long clipping = 0;
    for (std::size_t i = 0; i < b.models.size(); ++i) {
        const SignModel& m = b.models[i];
        const SignModelPart* post = nullptr;
        for (const SignModelPart& p : m.parts) if (p.materialName == "post") post = &p;
        if (!post) { CHECK(false); continue; }
        Vec3 lo(1e30, 1e30, 1e30), hi(-1e30, -1e30, -1e30);
        for (const Vertex& v : post->mesh.vertices) {
            lo = Vec3(std::min(lo.x, v.position.x), std::min(lo.y, v.position.y), std::min(lo.z, v.position.z));
            hi = Vec3(std::max(hi.x, v.position.x), std::max(hi.y, v.position.y), std::max(hi.z, v.position.z));
        }
        long bad = 0;
        Vec3 first;
        for (const SignModelPart& p : m.parts) {
            if (&p == post) continue;
            const RenderMesh& pm = p.mesh;
            for (std::size_t t = 0; t + 2 < pm.indices.size(); t += 3) {
                const Vec3& a = pm.vertices[pm.indices[t]].position;
                const Vec3& bb = pm.vertices[pm.indices[t + 1]].position;
                const Vec3& c = pm.vertices[pm.indices[t + 2]].position;
                const Vec3 n = normalize(cross(c - a, bb - a));
                constexpr int N = 5;
                for (int u = 0; u <= N; ++u)
                    for (int v = 0; u + v <= N; ++v) {
                        const Vec3 q = a + (bb - a) * (static_cast<Real>(u) / N) + (c - a) * (static_cast<Real>(v) / N) - n * 0.001;
                        if (solidsHolding(post->mesh, lo, hi, q) > 0 && bad++ == 0) first = q;
                    }
            }
        }
        if (bad) std::printf("    %s %s: %ld plate points inside the post, first at (%.3f %.3f %.3f)\n", m.label.c_str(),
                             reqs[i].dump().c_str(), bad, first.x, first.y, first.z);
        clipping += bad ? 1 : 0;
    }
    CHECK(clipping == 0);
}

TEST_CASE(sign_kit_the_side_a_sign_shows_comes_from_the_data) {
    const Font* font = signFont();
    if (!font) { CHECK(false); return; }
    // the engine drives on the left: a near turn is a LEFT turn, the mast arm reaches right over the road
    SignKit left;
    std::string err;
    CHECK(left.load("signs_us.lua", std::string(), &err));
    const std::vector<nlohmann::json> reqs = {{{"sign", "no_turn"}, {"turn", "near"}},
                                              {{"sign", "keep_side"}, {"side", "near"}},
                                              {{"sign", "speed_limit"}, {"mount", "mast_arm"}}};
    SignKitBuild a;
    CHECK(left.build(reqs, *font, a, &err));
    CHECK(a.models.size() == 3);
    if (a.models.size() == 3) {
        CHECK(a.models[0].label == "NO LEFT TURN (R3-2)");
        CHECK(a.models[1].label == "KEEP LEFT (R4-8)");
        double cx = 0;
        for (const Vertex& v : a.models[2].parts[0].mesh.vertices) cx += v.position.x / a.models[2].parts[0].mesh.vertices.size();
        CHECK(cx > 1.0);   // the sign hangs out over the road, to the reader's right
    }
    // the same catalogue for a road whose kerb is on the right: one line of data, no code
    SignKit right;
    CHECK(right.loadSource("local C = require \"signs_us\"\nC.kerb_side = \"right\"\nreturn C", std::string(), &err));
    SignKitBuild b;
    CHECK(right.build(reqs, *font, b, &err));
    if (b.models.size() == 3) {
        CHECK(b.models[0].label == "NO RIGHT TURN (R3-1)");
        CHECK(b.models[1].label == "KEEP RIGHT (R4-7)");
        double cx = 0;
        for (const Vertex& v : b.models[2].parts[0].mesh.vertices) cx += v.position.x / b.models[2].parts[0].mesh.vertices.size();
        CHECK(cx < -1.0);
    }
}
