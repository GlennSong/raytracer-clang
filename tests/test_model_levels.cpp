// LEVELS OF DETAIL IN A MODEL (#191, model step 5).
//
// A model may have several versions of itself for different distances: the full one close up, simpler ones farther
// away. Each part and each use belongs to one level. Every level hangs on the same nodes and has the same markers.
// A far level may be simpler in structure (one mesh on the root). A used piece has levels of its own.
//
// What is held here:
//   - the record: the full level alone unless more are added; each new level farther than the last; which level a
//     distance draws;
//   - the one walk draws by distance: the model's own level and, separately, each used piece's;
//   - a level made from the full one by the toolkit's simplify: fewer triangles, the same nodes and materials;
//   - Lua: `m:level`, `{ level = n }`, `m:levels`, and what stops a recipe;
//   - the readers: the bake and flatten draw the level asked for; the game draws the full level only, for now;
//   - `mesh.simplify` and a model's levels are one piece of code (MeshBuilder::simplifiedSurface).
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

bool runModel(const std::string& code, ProcModel& out, std::string* error = nullptr) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    return runProcgenModelValue(vm, code, out, error);
}

std::size_t triangles(const RenderMesh& m) { return m.indices.size() / 3; }

ProcPart partOf(const RenderMesh& mesh, int level, int node = 0) {
    ProcPart p;
    p.mesh = mesh;
    p.level = level;
    p.node = node;
    return p;
}

struct TmpFile {   // a scratch file, removed even when a check fails
    std::string path;
    TmpFile(const std::string& p, const std::string& contents) : path(p) { std::ofstream(path) << contents; }
    ~TmpFile() { std::remove(path.c_str()); }
};

}  // namespace

TEST_CASE(a_model_has_its_full_level_and_each_new_level_is_farther) {
    ProcModel m;
    CHECK(m.levelFrom.size() == 1 && m.levelFrom[0] == 0);
    CHECK(ProcPart{}.level == 0 && ProcUse{}.level == 0);
    CHECK(m.levelAt(0) == 0 && m.levelAt(1e9) == 0);       // one level: it is drawn from anywhere

    CHECK(m.addLevel(40) == 1);
    CHECK(m.addLevel(150) == 2);
    CHECK(m.addLevel(150) == -1);                           // not farther
    CHECK(m.addLevel(60) == -1);
    CHECK(m.addLevel(0) == -1);
    CHECK(m.levelFrom.size() == 3);
    CHECK(m.levelAt(0) == 0 && m.levelAt(39.9) == 0);
    CHECK(m.levelAt(40) == 1 && m.levelAt(149) == 1);
    CHECK(m.levelAt(150) == 2 && m.levelAt(1e9) == 2);
}

TEST_CASE(what_is_drawn_follows_the_distance_in_the_model_and_in_each_piece) {
    // A lamp: a detailed head near, a plain one from 30 m. A street: lamps at two verges, and from 100 m no lamps at
    // all, only a strip. The lamp switches at ITS distance and the street at its own.
    std::shared_ptr<ProcModel> lamp = std::make_shared<ProcModel>();
    lamp->addLevel(30);
    lamp->parts.push_back(partOf(MeshBuilder::sphere(0.3, 16, 12), 0));
    lamp->parts.push_back(partOf(MeshBuilder::box(Vec3(0.4, 0.4, 0.4)), 1));
    const std::size_t fine = triangles(lamp->parts[0].mesh), plain = triangles(lamp->parts[1].mesh);

    ProcModel street;
    street.addLevel(100);
    street.parts.push_back(partOf(MeshBuilder::box(Vec3(10, 0.1, 2)), 0));
    street.parts.push_back(partOf(MeshBuilder::box(Vec3(10, 0.1, 2)), 1));
    ProcUse lamps;
    lamps.piece = lamp;
    lamps.placements.push_back(ProcPlacement{Vec3(-4, 0, 0), Quat(), 1});
    lamps.placements.push_back(ProcPlacement{Vec3(4, 0, 0), Quat(), 1});
    street.uses.push_back(lamps);                           // level 0 of the street

    // near: the street's full level, the lamp's fine head
    std::vector<ProcDrawnPiece> drawn = drawnPieces(street, 0);
    CHECK(drawn.size() == 1 && drawn[0].part == &lamp->parts[0] && drawn[0].transforms.size() == 2);
    CHECK(triangles(flattenModel(street, 0)) == 12 + 2 * fine);
    // 50 m: still the street's full level, but the lamp has switched
    drawn = drawnPieces(street, 50);
    CHECK(drawn.size() == 1 && drawn[0].part == &lamp->parts[1] && drawn[0].transforms.size() == 2);
    CHECK(triangles(flattenModel(street, 50)) == 12 + 2 * plain);
    // 120 m: the street's far level has no lamps
    CHECK(drawnPieces(street, 120).empty());
    CHECK(placedUses(street, 120).empty());
    CHECK(triangles(flattenModel(street, 120)) == 12);
    // no distance said: the full model, as everything read before levels existed
    CHECK(triangles(flattenModel(street)) == 12 + 2 * fine);
    CHECK(street.instanceCount() == 2);

    // The bake draws the level asked for.
    Scene near, far;
    bakeProcModel(street, near);
    bakeProcModel(street, far, Vec3(), 120);
    CHECK(near.triangles.size() == 12 && near.instances.size() == 2);
    CHECK(far.triangles.size() == 12 && far.instances.empty());
}

TEST_CASE(a_level_made_from_the_full_one_is_simpler_on_the_same_nodes) {
    ProcModel m;
    const int arm = m.addNode("arm", 0, ProcFrame{Vec3(0, 2, 0), Quat()});
    std::shared_ptr<ProcMaterial> steel = std::make_shared<ProcMaterial>();
    const int made = m.addMaterial("steel", steel);
    m.parts.push_back(partOf(MeshBuilder::sphere(1.0, 32, 24), 0));
    { ProcPart p = partOf(MeshBuilder::sphere(0.5, 32, 24), 0, arm); p.material = made; m.parts.push_back(p); }
    ProcUse bolt;
    bolt.piece = ProcModel::ofMesh(MeshBuilder::box(Vec3(0.1, 0.1, 0.1)));
    bolt.node = arm;
    bolt.placements.push_back(ProcPlacement{});
    m.uses.push_back(bolt);

    CHECK(addSimplifiedLevel(m, 40, 0.25) == 1);
    CHECK(addSimplifiedLevel(m, 30, 0.25) == -1);           // not farther: nothing added
    CHECK(m.parts.size() == 4 && m.uses.size() == 2);
    for (std::size_t i = 0; i < 2; ++i) {
        const ProcPart& full = m.parts[i];
        const ProcPart& simple = m.parts[i + 2];
        CHECK(simple.level == 1 && simple.node == full.node && simple.material == full.material);
        // a sphere's ring of triangles at each pole has no area and goes with the weld, so count from the welded one
        MeshBuilder::WeldTolerance tol;
        tol.positionOnly = true;
        const std::size_t have = triangles(MeshBuilder::weld(full.mesh, tol));
        CHECK(triangles(simple.mesh) <= triangles(full.mesh) / 4 + 2);
        CHECK(triangles(simple.mesh) >= have / 8);           // simpler, not gone
        // still the same shape: every point of the simpler sphere is near the sphere
        const Real radius = i == 0 ? 1.0 : 0.5;
        Real worst = 0;
        for (const Vertex& v : simple.mesh.vertices) worst = std::max(worst, std::abs(v.position.length() - radius));
        CHECK(worst < 0.12 * radius);
    }
    CHECK(m.uses[1].level == 1 && m.uses[1].piece == m.uses[0].piece && m.uses[1].node == arm);   // the same bolt, again
    CHECK(drawnPieces(m, 50).size() == 1);
    // The second from the full one again, not from the first lower level.
    CHECK(addSimplifiedLevel(m, 120, 0.05) == 2);
    CHECK(m.parts.size() == 6);
    CHECK(triangles(m.parts[4].mesh) < triangles(m.parts[2].mesh));

    // One piece of code: the toolkit's simplified surface is the weld, the simplify, the weld and the creasing a
    // recipe's mesh.simplify always did.
    const RenderMesh ball = MeshBuilder::sphere(1.0, 24, 16);
    MeshBuilder::WeldTolerance tol;
    tol.positionOnly = true;
    RenderMesh byHand = MeshBuilder::weld(ball, tol);
    const std::size_t left = MeshBuilder::simplify(byHand, 100);
    byHand = MeshBuilder::weld(byHand);
    MeshBuilder::autosmooth(byHand, 50.0);
    std::size_t leftToo = 0;
    const RenderMesh shared = MeshBuilder::simplifiedSurface(ball, 100, 1e30, true, 50.0, &leftToo);
    CHECK(left == leftToo && shared.indices == byHand.indices && shared.vertices.size() == byHand.vertices.size());
    bool same = true;
    for (std::size_t i = 0; i < shared.vertices.size() && same; ++i)
        same = shared.vertices[i].position.x == byHand.vertices[i].position.x && shared.vertices[i].normal.y == byHand.vertices[i].normal.y;
    CHECK(same);
}

TEST_CASE(merging_keeps_levels_that_agree_and_refuses_ones_that_do_not) {
    ProcModel a;
    a.parts.push_back(partOf(MeshBuilder::box(Vec3(1, 1, 1)), 0));
    ProcModel b;
    b.addLevel(40);
    b.parts.push_back(partOf(MeshBuilder::box(Vec3(1, 1, 1)), 0));
    b.parts.push_back(partOf(MeshBuilder::box(Vec3(1, 1, 1)), 1));
    CHECK(a.merge(b));                                      // a had only the full level: it takes b's
    CHECK(a.levelFrom.size() == 2 && a.levelFrom[1] == 40);
    CHECK(a.parts.size() == 3 && a.parts[2].level == 1);

    ProcModel same;
    same.addLevel(40);
    same.parts.push_back(partOf(MeshBuilder::box(Vec3(1, 1, 1)), 1));
    CHECK(a.merge(same));
    CHECK(a.parts.size() == 4 && a.levelFrom.size() == 2);

    ProcModel other;
    other.addLevel(60);
    std::string clash;
    CHECK(!a.merge(other, &clash));
    CHECK(clash.find("levels of detail") != std::string::npos);
    CHECK(a.parts.size() == 4);

    ProcModel flat;                                         // one level merges into many: its parts are the full level's
    flat.parts.push_back(partOf(MeshBuilder::box(Vec3(1, 1, 1)), 0));
    CHECK(a.merge(flat));
    CHECK(a.parts.back().level == 0 && a.levelFrom.size() == 2);
}

TEST_CASE(a_recipe_adds_levels_and_says_what_belongs_to_each) {
    ProcModel m;
    std::string err;
    CHECK(runModel(R"LUA(
        local tire = model.new()
        tire:add(mesh.sphere(0.3, 24, 16))
        assert(tire:level(40, { simplify = 0.25 }) == 2)      -- made from the full one

        local car = model.new()
        car:material("paint", material.new{ roughness = 0.3 })
        car:add(mesh.sphere(1, 32, 24), "paint")
        local wheel = car:node("wheel", { pos = { 1, 0, 0 } })
        wheel:use(tire)
        local mid = car:level(40, { simplify = 0.3, max_error = 0.05 })
        local far = car:level(150)
        assert(mid == 2 and far == 3)
        car:add(mesh.box{ 2, 1, 4 }, "paint", { level = far })                -- simpler in structure: one box
        car:add(mesh.box{ 0.2, 0.2, 0.2 }, { level = far })                   -- the plain material, with a level
        wheel:add(mesh.box{ 0.3, 0.6, 0.6 }, "paint", { level = far })        -- on a node, at a level
        car:use(mesh.box{ 0.1, 0.1, 0.1 }, { { pos = { 0, 1, 0 } } }, { level = far })
        local from = car:levels()
        assert(#from == 3 and from[1] == 0 and from[2] == 40 and from[3] == 150)
        assert(#tire:levels() == 2)
        return car
    )LUA", m, &err));
    if (!err.empty()) std::printf("    script error: %s\n", err.c_str());
    CHECK(m.levelFrom.size() == 3);
    CHECK(m.parts.size() == 5 && m.uses.size() == 3);
    if (m.levelFrom.size() != 3 || m.parts.size() != 5 || m.uses.size() != 3) return;   // the recipe failed: nothing to read
    int byLevel[3] = {0, 0, 0};
    for (const ProcPart& p : m.parts) ++byLevel[p.level];
    CHECK(byLevel[0] == 1 && byLevel[1] == 1 && byLevel[2] == 3);
    CHECK(m.parts[1].level == 1 && m.parts[1].material == m.findMaterial("paint"));
    CHECK(triangles(m.parts[1].mesh) < triangles(m.parts[0].mesh) / 2);
    CHECK(m.parts[3].material == 0 && m.parts[3].level == 2);
    CHECK(m.parts[4].node == m.findNode("wheel") && m.parts[4].level == 2);
    // the wheel's use: at the full level, repeated at the middle one, not at the far one; the far one has its own
    CHECK(m.uses.size() == 3 && m.uses[0].level == 0 && m.uses[1].level == 1 && m.uses[2].level == 2);
    CHECK(m.uses[1].piece == m.uses[0].piece);
    // At 60 m: the car's middle level, and the tire's own simpler level.
    const std::vector<ProcDrawnPiece> at60 = drawnPieces(m, 60);
    CHECK(at60.size() == 1 && at60[0].part->level == 1);

    struct Wrong { const char* code; const char* says; };
    const Wrong wrongs[] = {
        {R"LUA(local m = model.new() m:level(40) m:level(40) return m)LUA", "farther than the last"},
        {R"LUA(local m = model.new() m:level(0) return m)LUA", "farther than the last"},
        {R"LUA(local m = model.new() m:add(mesh.box{1,1,1}, { level = 2 }) return m)LUA", "the model has no level 2"},
        {R"LUA(local m = model.new() m:level(40) m:add(mesh.box{1,1,1}, nil, { level = 0 }) return m)LUA", "the model has no level 0"},
        {R"LUA(local m = model.new() m:use(mesh.box{1,1,1}, { level = 3 }) return m)LUA", "the model has no level 3"},
        {R"LUA(local m = model.new() m:add(mesh.box{1,1,1}) m:level(40, { simplify = 1.5 }) return m)LUA", "0 to 1"},
    };
    for (const Wrong& w : wrongs) {
        err.clear();
        CHECK(!runModel(w.code, m, &err));
        if (err.find(w.says) == std::string::npos) std::printf("    wanted '%s', got: %s\n", w.says, err.c_str());
        CHECK(err.find(w.says) != std::string::npos);
    }
}

TEST_CASE(the_game_draws_a_models_full_level_only_for_now) {
    // Picking a level by distance is the prefab's drawing (step 7): a model's separate part entities would each
    // switch at their own distance. Until then the loader draws the full level and nothing of the others.
    const TmpFile recipe("test_model_levels_tmp.lua", R"LUA(
        local m = model.new()
        m:add(mesh.box{ 1, 1, 1 })
        m:add(mesh.box{ 2, 2, 2 })
        m:use(mesh.box{ 0.2, 0.2, 0.2 }, { { pos = { 3, 0, 0 } } })
        local far = m:level(50)
        m:add(mesh.box{ 3, 3, 3 }, { level = far })
        m:use(mesh.box{ 0.5, 0.5, 0.5 }, { { pos = { 3, 0, 0 } }, { pos = { 6, 0, 0 } } }, { level = far })
        return m
    )LUA");
    const TmpFile level("test_model_levels_tmp.json", R"({
      "version": 1,
      "entities": [ { "name": "thing", "shape": "script", "file": "test_model_levels_tmp.lua" } ]
    })");
    const TmpFile sidecar("test_model_levels_tmp.json.cameras.json", "");

    std::unique_ptr<Renderer> renderer = Renderer::create();
    RendererMeshUploader uploader(*renderer);
    AssetManager assets(uploader);
    World world;
    RenderView view;
    CHECK(LevelLoader::load(level.path, world, *renderer, view, assets, false));
    int parts = 0, groups = 0, copies = 0;
    world.each<Renderable>([&](Entity, Renderable&) { ++parts; });
    world.each<InstanceGroup>([&](Entity, InstanceGroup& g) { ++groups; copies += static_cast<int>(g.transforms.size()); });
    CHECK(parts == 2);
    CHECK(groups == 1 && copies == 1);
    world.destroyAll();
    assets.clear();

    Scene scene;                                            // and the offline loader the same
    CHECK(LevelScene::load(level.path, scene));
    CHECK(scene.triangles.size() == 24 && scene.instances.size() == 1);
}
