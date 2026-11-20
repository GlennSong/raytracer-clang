// NODES IN A MODEL (#191, model step 1).
//
// A model is a tree of named nodes: the door under the car, the handle under the door. Each node has a position and
// a turn under its parent, and a part (a mesh with a material) hangs on one node and is drawn there. The root, node
// 0, is the model's own space, so a model that adds no node is the flat model every recipe built before.
//
// What is held here:
//   - the record: the root is always there, names are unique, a node's place in the model is its own frame under
//     its parents', and merging two models keeps both trees;
//   - Lua: `m:node`, `n:node`, `n:add`, turns in degrees, and a wrong name stopping the recipe;
//   - the three readers agree: the offline bake, `m:flatten`, and the level loader's entities all put a child node's
//     mesh at the same place;
//   - a recipe that uses no node is untouched: one node, every part on it, the triangles its vertices say.
#include "test_framework.h"

#include "../src/engine/asset_manager.h"
#include "../src/engine/components.h"
#include "../src/engine/level_loader.h"
#include "../src/engine/mesh_builder.h"
#include "../src/engine/mesh_uploader.h"
#include "../src/engine/procgen/proc_model.h"
#include "../src/engine/scripting/procgen_bindings.h"
#include "../src/engine/scripting/script_vm.h"
#include "../src/level_scene.h"
#include "../src/renderer/renderer.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

using namespace engine;

namespace {

// A slab on the root, a post 3 m along x turned a quarter turn about the vertical, and an arm 2 m up the post and
// 1 m along the post's own z. The quarter turn carries the post's z onto the model's x, so the arm's centre is at
// (4, 2, 0) in the model.
const char* kTreeRecipe = R"LUA(
    local m = model.new()
    m:add(mesh.box{ 2, 1, 2 })
    local post = m:node("post", { pos = { 3, 0, 0 }, turn = { 0, 90, 0 } })
    post:add(mesh.box{ 1, 1, 1 })
    local arm = post:node("arm", { pos = { 0, 2, 1 } })
    arm:add(mesh.box{ 0.5, 0.5, 0.5 })
    return m
)LUA";
const Vec3 kArmCentre(4, 2, 0);

struct Box {
    Vec3 lo{1e30, 1e30, 1e30}, hi{-1e30, -1e30, -1e30};
    void add(const Vec3& p) {
        lo = Vec3(std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z));
        hi = Vec3(std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z));
    }
    Vec3 centre() const { return (lo + hi) * 0.5; }
    Vec3 size() const { return hi - lo; }
};

bool near(const Vec3& a, const Vec3& b, Real eps = 1e-9) { return (a - b).length() < eps; }

bool runModel(const std::string& code, ProcModel& out, std::string* error = nullptr) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    return runProcgenModelValue(vm, code, out, error);
}

struct TmpFile {   // a scratch file, removed even when a check fails
    std::string path;
    TmpFile(const std::string& p, const std::string& contents) : path(p) { std::ofstream(path) << contents; }
    ~TmpFile() { std::remove(path.c_str()); }
};

}  // namespace

TEST_CASE(a_model_always_has_its_root_and_names_are_unique) {
    ProcModel m;
    CHECK(m.nodes.size() == 1);
    CHECK(m.nodes[0].name == "root");
    CHECK(m.nodes[0].parent == -1);
    CHECK(m.findNode("root") == 0);
    CHECK(m.findNode("door") == -1);
    CHECK(ProcPart{}.node == 0);                         // a part that names no node is on the root

    const int door = m.addNode("door", 0, ProcFrame{Vec3(1, 0, 0), Quat()});
    CHECK(door == 1);
    CHECK(m.addNode("handle", door, ProcFrame{}) == 2);
    CHECK(m.addNode("door", 0, ProcFrame{}) == -1);      // the name is taken
    CHECK(m.addNode("root", 0, ProcFrame{}) == -1);
    CHECK(m.addNode("", 0, ProcFrame{}) == -1);          // a node needs a name
    CHECK(m.addNode("lost", 7, ProcFrame{}) == -1);      // no such parent
    CHECK(m.addNode("lost", -1, ProcFrame{}) == -1);     // only the root has no parent
    CHECK(m.nodes.size() == 3);                          // the refused ones added nothing
}

TEST_CASE(a_nodes_place_in_the_model_is_its_frame_under_its_parents) {
    ProcModel m;
    // The root is no move and no turn, exactly: this is what keeps a flat model's triangles bit for bit.
    const ProcFrame root = m.nodeFrame(0);
    CHECK(root.position.x == 0 && root.position.y == 0 && root.position.z == 0);
    CHECK(root.rotation.x == 0 && root.rotation.y == 0 && root.rotation.z == 0 && root.rotation.w == 1);

    const Quat quarter = Quat::fromAxisAngle(Vec3(0, 1, 0), 90 * DEG_TO_RAD);
    const int post = m.addNode("post", 0, ProcFrame{Vec3(3, 0, 0), quarter});
    const int arm = m.addNode("arm", post, ProcFrame{Vec3(0, 2, 1), Quat()});
    const int tip = m.addNode("tip", arm, ProcFrame{Vec3(0, 0, 1), quarter});
    CHECK(near(m.nodeFrame(post).position, Vec3(3, 0, 0)));
    CHECK(near(m.nodeFrame(arm).position, kArmCentre));
    CHECK(near(m.nodeFrame(tip).position, Vec3(5, 2, 0)));
    // Turns add up down the tree: two quarter turns face the tip's z backwards.
    CHECK(near(m.nodeFrame(tip).rotation.rotate(Vec3(0, 0, 1)), Vec3(0, 0, -1)));
    // The frame as a matrix says the same as the frame.
    CHECK(near(m.nodeFrame(arm).matrix().transformPoint(Vec3(0, 0, 1)), m.nodeFrame(arm).point(Vec3(0, 0, 1))));
}

TEST_CASE(merging_two_models_keeps_both_trees) {
    ProcModel a;
    a.parts.push_back(ProcPart{});
    const int aDoor = a.addNode("door", 0, ProcFrame{Vec3(1, 0, 0), Quat()});
    { ProcPart p; p.node = aDoor; a.parts.push_back(p); }

    ProcModel b;
    b.parts.push_back(ProcPart{});                                           // on b's root
    const int bBoom = b.addNode("boom", 0, ProcFrame{Vec3(0, 5, 0), Quat()});
    const int bHook = b.addNode("hook", bBoom, ProcFrame{Vec3(0, 0, 4), Quat()});
    { ProcPart p; p.node = bHook; b.parts.push_back(p); }

    CHECK(a.merge(b));
    CHECK(a.nodes.size() == 4);                          // one root, not two
    CHECK(a.parts.size() == 4);
    CHECK(a.parts[2].node == 0);                         // b's root part is on a's root
    const int hook = a.findNode("hook");
    CHECK(hook > 0);
    CHECK(a.parts[3].node == hook);
    CHECK(a.nodes[static_cast<std::size_t>(hook)].parent == a.findNode("boom"));
    CHECK(a.nodes[static_cast<std::size_t>(a.findNode("boom"))].parent == 0);
    CHECK(near(a.nodeFrame(hook).position, Vec3(0, 5, 4)));   // where it was in b

    // Two nodes of one name: refused, and nothing is changed.
    ProcModel c;
    c.addNode("door", 0, ProcFrame{});
    c.parts.push_back(ProcPart{});
    std::string clash;
    CHECK(!a.merge(c, &clash));
    CHECK(clash == "door");
    CHECK(a.nodes.size() == 4);
    CHECK(a.parts.size() == 4);
}

TEST_CASE(a_recipe_builds_a_tree_of_nodes) {
    ProcModel m;
    std::string err;
    CHECK(runModel(kTreeRecipe, m, &err));
    if (!err.empty()) std::printf("    script error: %s\n", err.c_str());
    CHECK(m.nodes.size() == 3);
    CHECK(m.parts.size() == 3);
    const int post = m.findNode("post"), arm = m.findNode("arm");
    CHECK(post == 1 && arm == 2);
    CHECK(m.nodes[2].parent == post);
    CHECK(m.parts[0].node == 0 && m.parts[1].node == post && m.parts[2].node == arm);
    CHECK(near(m.nodeFrame(arm).position, kArmCentre, 1e-9));   // `turn` was read as degrees

    // The words a recipe asks the tree with.
    ScriptVM vm;
    openProcgenLibrary(vm);
    CHECK(vm.doString(R"LUA(
        local m = model.new()
        assert(m:node_count() == 1)
        local post = m:node("post", { pos = { 3, 0, 0 }, turn = { 0, 90, 0 } })
        local arm = m:node("arm", { parent = "post", pos = { 0, 2, 1 } })     -- the parent by name
        assert(m:node_count() == 3)
        assert(arm:name() == "arm" and arm:parent():name() == "post")
        assert(post:parent():name() == "root" and post:parent():parent() == nil)
        assert(m:find("arm"):name() == "arm" and m:find("nothing") == nil)
        local p = arm:point{ 0, 0, 1 }                   -- 1 m along the arm's z, which is the model's x
        assert(math.abs(p[1] - 5) < 1e-9 and math.abs(p[2] - 2) < 1e-9 and math.abs(p[3]) < 1e-9)
        assert(arm:add(mesh.box{ 1, 1, 1 }) == arm)      -- chainable, like m:add
        assert(m:part_count() == 1)
    )LUA", &err));
    if (!err.empty()) std::printf("    script error: %s\n", err.c_str());
}

TEST_CASE(a_wrong_node_name_stops_the_recipe) {
    ProcModel m;
    std::string err;
    CHECK(!runModel(R"LUA(local m = model.new() m:node("door") m:node("door") return m)LUA", m, &err));
    CHECK(err.find("already has a node named 'door'") != std::string::npos);
    err.clear();
    CHECK(!runModel(R"LUA(local m = model.new() m:node("root") return m)LUA", m, &err));
    CHECK(err.find("already has a node named 'root'") != std::string::npos);
    err.clear();
    CHECK(!runModel(R"LUA(local m = model.new() m:node("handle", { parent = "door" }) return m)LUA", m, &err));
    CHECK(err.find("no node named 'door'") != std::string::npos);
    err.clear();
    CHECK(!runModel(R"LUA(local m = model.new() m:node("") return m)LUA", m, &err));
    CHECK(err.find("needs a name") != std::string::npos);
    err.clear();
    CHECK(!runModel(R"LUA(
        local a, b = model.new(), model.new()
        a:node("door") b:node("door")
        return a:merge(b))LUA", m, &err));
    CHECK(err.find("both models have a node named 'door'") != std::string::npos);
    err.clear();
    CHECK(!runModel(R"LUA(local a = model.new() return a:merge(a))LUA", m, &err));
    CHECK(err.find("into itself") != std::string::npos);
}

TEST_CASE(a_recipe_with_no_nodes_is_the_flat_model) {
    ProcModel m;
    CHECK(runModel(R"LUA(
        local m = model.new()
        m:add(mesh.translate(mesh.box{ 1, 2, 3 }, { 5, 6, 7 }))
        m:add_solid(mesh.box{ 4, 4, 4 })
        local other = model.new()
        other:add(mesh.box{ 1, 1, 1 })
        m:merge(other)
        return m
    )LUA", m));
    CHECK(m.nodes.size() == 1);
    CHECK(m.parts.size() == 3);
    for (const ProcPart& p : m.parts) CHECK(p.node == 0);

    // Baked, every triangle's corner is the vertex the mesh holds, bit for bit, in the mesh's order.
    Scene scene;
    bakeProcModel(m, scene);
    std::size_t t = 0;
    bool same = true;
    for (const ProcPart& p : m.parts)
        for (std::size_t i = 0; i + 2 < p.mesh.indices.size(); i += 3, ++t) {
            const Vec3& a = p.mesh.vertices[p.mesh.indices[i]].position;
            const Vec3& b = p.mesh.vertices[p.mesh.indices[i + 1]].position;
            const Vec3& c = p.mesh.vertices[p.mesh.indices[i + 2]].position;
            const Triangle& tri = scene.triangles[t];
            same = same && tri.v0.x == a.x && tri.v0.y == a.y && tri.v0.z == a.z && tri.v1.x == b.x && tri.v1.y == b.y &&
                   tri.v1.z == b.z && tri.v2.x == c.x && tri.v2.y == c.y && tri.v2.z == c.z;
        }
    CHECK(t == scene.triangles.size());
    CHECK(same);
}

TEST_CASE(a_child_nodes_mesh_is_drawn_in_place_by_the_bake_and_by_flatten) {
    ProcModel m;
    CHECK(runModel(kTreeRecipe, m));

    // The offline bake: three boxes of 12 triangles, in the parts' order. The last is the arm's.
    Scene scene;
    bakeProcModel(m, scene, Vec3(10, 0, 0));             // and the entity's own offset still moves the whole model
    CHECK(scene.triangles.size() == 36);
    Box slab, post, arm;
    for (std::size_t i = 0; i < scene.triangles.size(); ++i) {
        Box& b = i < 12 ? slab : i < 24 ? post : arm;
        b.add(scene.triangles[i].v0); b.add(scene.triangles[i].v1); b.add(scene.triangles[i].v2);
    }
    CHECK(near(slab.centre(), Vec3(10, 0, 0)));
    CHECK(near(post.centre(), Vec3(13, 0, 0)));
    CHECK(near(arm.centre(), kArmCentre + Vec3(10, 0, 0)));
    CHECK(near(arm.size(), Vec3(0.5, 0.5, 0.5)));
    // A face's normal turns with its node: the arm's +z face looks along the model's +x.
    bool facesX = false;
    for (std::size_t i = 24; i < 36; ++i) facesX = facesX || near(scene.triangles[i].n0, Vec3(1, 0, 0));
    CHECK(facesX);

    // m:flatten gives one mesh with the same three boxes in the same places.
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::shared_ptr<RenderMesh> flat;
    std::string err;
    std::string code = kTreeRecipe;
    code.replace(code.rfind("return m"), 8, "return m:flatten()");
    CHECK(runProcgenMesh(vm, code, flat, &err));
    CHECK(flat != nullptr);
    if (flat != nullptr) {
        CHECK(flat->indices.size() == 36 * 3);
        Box flatArm;
        for (std::size_t i = 24 * 3; i < flat->indices.size(); ++i) flatArm.add(flat->vertices[flat->indices[i]].position);
        CHECK(near(flatArm.centre(), kArmCentre));
        CHECK(near(flatArm.size(), Vec3(0.5, 0.5, 0.5)));
    }
}

TEST_CASE(a_level_draws_a_child_nodes_mesh_in_place_offline_and_in_the_game) {
    // One level, one recipe, both loaders: the path tracer's scene and the game's entities put the arm in one place.
    const TmpFile recipe("test_model_nodes_tmp.lua", kTreeRecipe);
    const TmpFile level("test_model_nodes_tmp.json", R"({
      "version": 1,
      "entities": [ { "name": "tree", "shape": "script", "file": "test_model_nodes_tmp.lua", "position": [10, 0, 0] } ]
    })");
    const TmpFile sidecar("test_model_nodes_tmp.json.cameras.json", "");   // a load may write one; clear it after

    Scene scene;
    CHECK(LevelScene::load(level.path, scene));
    CHECK(scene.triangles.size() == 36);
    Box offlineArm;
    for (std::size_t i = 24; i < scene.triangles.size(); ++i) {
        offlineArm.add(scene.triangles[i].v0); offlineArm.add(scene.triangles[i].v1); offlineArm.add(scene.triangles[i].v2);
    }
    CHECK(near(offlineArm.centre(), kArmCentre + Vec3(10, 0, 0)));

    std::unique_ptr<Renderer> renderer = Renderer::create();
    RendererMeshUploader uploader(*renderer);
    AssetManager assets(uploader);
    World world;
    RenderView view;
    CHECK(LevelLoader::load(level.path, world, *renderer, view, assets, false));
    // Each part is an entity standing at its node's frame; its mesh is still in the node's own space.
    std::vector<Transform> drawn;
    world.each<Renderable, Transform>([&](Entity, Renderable&, Transform& t) { drawn.push_back(t); });
    CHECK(drawn.size() == 3);
    int found = 0;
    for (const Transform& t : drawn) {
        if (near(t.position, Vec3(10, 0, 0))) ++found;
        if (near(t.position, Vec3(13, 0, 0))) ++found;
        if (near(t.position, offlineArm.centre())) {
            ++found;
            CHECK(near(t.orientation.rotate(Vec3(0, 0, 1)), Vec3(1, 0, 0)));   // turned with the post
        }
    }
    CHECK(found == 3);
    world.destroyAll();
    assets.clear();
}
