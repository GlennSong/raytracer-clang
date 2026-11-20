// USES: ONE MODEL PLACED INSIDE ANOTHER, STORED ONCE (#191, model step 3).
//
// A wheel used four times, a bolt forty, a lamp at every verge. A use names a piece (a whole model, with its own
// parts, materials, nodes, markers and uses), the node whose space it is placed in, and where each copy goes. This
// is what the instance group grew into; `m:add_instances` is the older word for it and still works.
//
// What is held here:
//   - the record: a use on the root has exactly its placements' matrices; a use on a node stands at that node; a use
//     inside a piece is drawn once for every copy of that piece; a piece may not reach back to the model using it;
//   - the one walk: a piece used from four nodes is one instanced draw per part of the piece, four copies each;
//   - what is inside a piece is reached through the node that uses it ("wheel_fl/valve"), scale and all;
//   - Lua: `n:use`, `m:use`, placements in degrees, and `m:add_instances` unchanged;
//   - the readers: the offline bake, flatten and the game's loader draw the same copies in the same places, and the
//     loader stamps a use's colliders at every copy.
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

bool near(const Vec3& a, const Vec3& b, Real eps = 1e-9) { return (a - b).length() < eps; }
Vec3 origin(const Mat4& m) { return Vec3(m.m[0][3], m.m[1][3], m.m[2][3]); }

bool same(const Mat4& a, const Mat4& b) {
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            if (a.m[r][c] != b.m[r][c]) return false;
    return true;
}

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

// A tire: a rim and a wall in two materials, a valve marker, and five bolts (one bolt piece, used five times).
std::shared_ptr<ProcModel> makeTire(const std::shared_ptr<ProcModel>& bolt) {
    std::shared_ptr<ProcModel> tire = std::make_shared<ProcModel>();
    std::shared_ptr<ProcMaterial> rubber = std::make_shared<ProcMaterial>();
    rubber->roughness = 0.9f;
    std::shared_ptr<ProcMaterial> alloy = std::make_shared<ProcMaterial>();
    alloy->metallic = 0.9f;
    { ProcPart p; p.mesh = MeshBuilder::box(Vec3(0.2, 0.6, 0.6)); p.material = tire->addMaterial("rubber", rubber); tire->parts.push_back(p); }
    { ProcPart p; p.mesh = MeshBuilder::box(Vec3(0.22, 0.4, 0.4)); p.material = tire->addMaterial("alloy", alloy); tire->parts.push_back(p); }
    ProcMarker valve;
    valve.name = "valve";
    valve.frame.position = Vec3(0.1, 0.2, 0);
    tire->addMarker(0, valve);
    ProcUse bolts;
    bolts.piece = bolt;
    for (int i = 0; i < 5; ++i) bolts.placements.push_back(ProcPlacement{Vec3(0.12, 0.1 * std::cos(i * 1.2566), 0.1 * std::sin(i * 1.2566)), Quat(), 1});
    tire->uses.push_back(bolts);
    return tire;
}

// A car: a body, and the one tire used at four nodes.
const Vec3 kWheelAt[4] = {Vec3(0.8, 0.3, 1.3), Vec3(-0.8, 0.3, 1.3), Vec3(0.8, 0.3, -1.3), Vec3(-0.8, 0.3, -1.3)};
const char* kWheelName[4] = {"wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"};
ProcModel makeCar(const std::shared_ptr<ProcModel>& tire) {
    ProcModel car;
    { ProcPart p; p.mesh = MeshBuilder::box(Vec3(1.6, 0.6, 4)); car.parts.push_back(p); }
    for (int w = 0; w < 4; ++w) {
        ProcUse use;
        use.piece = tire;
        use.node = car.addNode(kWheelName[w], 0, ProcFrame{kWheelAt[w], Quat()});
        use.placements.push_back(ProcPlacement{});
        car.uses.push_back(use);
    }
    return car;
}

}  // namespace

TEST_CASE(a_use_on_the_root_has_exactly_its_placements_matrices) {
    // What the instance group was: a mesh placed by matrices in the model's own space. Bit for bit, so every recipe
    // written with m:add_instances draws what it drew.
    ProcModel m;
    ProcUse lamps;
    lamps.piece = ProcModel::ofMesh(MeshBuilder::box(Vec3(0.2, 4, 0.2)));
    lamps.placements.push_back(ProcPlacement{Vec3(5, 0, 1), Quat::fromAxisAngle(Vec3(0, 1, 0), 0.7), 1.3});
    lamps.placements.push_back(ProcPlacement{Vec3(-3, 0.5, 2), Quat(), 1});
    m.uses.push_back(lamps);

    const std::vector<ProcPlacedUse> placed = placedUses(m);
    CHECK(placed.size() == 1);
    CHECK(placed[0].use == &m.uses[0]);
    CHECK(placed[0].transforms.size() == 2);
    CHECK(same(placed[0].transforms[0], Mat4::trs(Vec3(5, 0, 1), Quat::fromAxisAngle(Vec3(0, 1, 0), 0.7), Vec3(1.3, 1.3, 1.3))));
    CHECK(same(placed[0].transforms[1], Mat4::trs(Vec3(-3, 0.5, 2), Quat(), Vec3(1, 1, 1))));
    const std::vector<ProcDrawnPiece> drawn = drawnPieces(m);
    CHECK(drawn.size() == 1);
    CHECK(same(drawn[0].transforms[0], placed[0].transforms[0]));
    CHECK(m.instanceCount() == 2);
}

TEST_CASE(a_piece_used_from_four_nodes_is_stored_and_drawn_once) {
    const std::shared_ptr<ProcModel> bolt = ProcModel::ofMesh(MeshBuilder::box(Vec3(0.02, 0.02, 0.02)));
    const std::shared_ptr<ProcModel> tire = makeTire(bolt);
    const ProcModel car = makeCar(tire);

    // Four uses of the tire, and inside the tire one use of the bolt that is met once per tire.
    const std::vector<ProcPlacedUse> placed = placedUses(car);
    CHECK(placed.size() == 8);
    int tires = 0, bolts = 0;
    for (const ProcPlacedUse& p : placed) {
        if (p.use->piece == tire) { ++tires; CHECK(p.transforms.size() == 1); }
        if (p.use->piece == bolt) { ++bolts; CHECK(p.transforms.size() == 5); }
    }
    CHECK(tires == 4 && bolts == 4);
    CHECK(car.instanceCount() == 4 + 20);

    // Drawn: the wall, the rim and the bolt, each ONE instanced draw.
    const std::vector<ProcDrawnPiece> drawn = drawnPieces(car);
    CHECK(drawn.size() == 3);
    CHECK(drawn[0].part == &tire->parts[0] && drawn[0].transforms.size() == 4);
    CHECK(drawn[1].part == &tire->parts[1] && drawn[1].transforms.size() == 4);
    CHECK(drawn[2].part == &bolt->parts[0] && drawn[2].transforms.size() == 20);
    for (int w = 0; w < 4; ++w) CHECK(near(origin(drawn[0].transforms[static_cast<std::size_t>(w)]), kWheelAt[w]));
    // A bolt of the second wheel: the wheel's place plus the bolt's place in the tire.
    CHECK(near(origin(drawn[2].transforms[5]), kWheelAt[1] + Vec3(0.12, 0.1, 0)));
    CHECK(&drawn[1].piece->materialOf(*drawn[1].part) == tire->materials[static_cast<std::size_t>(tire->findMaterial("alloy"))].material.get());
}

TEST_CASE(a_use_stands_at_its_node_and_a_pieces_part_at_its_own_node) {
    // The piece has a part on a node of its own; the use is on a turned node and the copy is scaled.
    std::shared_ptr<ProcModel> lamp = std::make_shared<ProcModel>();
    const int head = lamp->addNode("head", 0, ProcFrame{Vec3(0, 4, 0.5), Quat()});
    { ProcPart p; p.mesh = MeshBuilder::box(Vec3(0.3, 0.1, 0.3)); p.node = head; lamp->parts.push_back(p); }

    ProcModel street;
    const int verge = street.addNode("verge", 0, ProcFrame{Vec3(10, 0, 0), Quat::fromAxisAngle(Vec3(0, 1, 0), 90 * DEG_TO_RAD)});
    ProcUse use;
    use.piece = lamp;
    use.node = verge;
    use.placements.push_back(ProcPlacement{Vec3(0, 0, 2), Quat(), 2});
    street.uses.push_back(use);

    // verge at (10,0,0), its z along the model's x: the copy at (12,0,0); the head 8 up and 1 along (scale 2).
    const std::vector<ProcDrawnPiece> drawn = drawnPieces(street);
    CHECK(drawn.size() == 1 && drawn[0].transforms.size() == 1);
    CHECK(near(origin(drawn[0].transforms[0]), Vec3(13, 8, 0)));
    CHECK(near(drawn[0].transforms[0].transformPoint(Vec3(0, 0, 0.15)), Vec3(13.3, 8, 0)));   // scaled and turned
}

TEST_CASE(what_is_inside_a_piece_is_reached_through_the_node_that_uses_it) {
    const std::shared_ptr<ProcModel> tire = makeTire(ProcModel::ofMesh(MeshBuilder::box(Vec3(0.02, 0.02, 0.02))));
    ProcModel car = makeCar(tire);
    ProcFrame at;
    CHECK(car.markerFrame("wheel_fl/valve", at));
    CHECK(near(at.position, kWheelAt[0] + Vec3(0.1, 0.2, 0)));
    CHECK(car.markerFrame("wheel_rr/valve", at));
    CHECK(near(at.position, kWheelAt[3] + Vec3(0.1, 0.2, 0)));
    CHECK(!car.markerFrame("valve", at));                  // the car itself has no valve
    CHECK(!car.markerFrame("wheel_fl/nothing", at));
    CHECK(!car.markerFrame("nothing/valve", at));
    CHECK(tire->markerFrame("valve", at));                 // the tire's own name, asked of the tire

    // A copy's scale and turn carry the marker with it, and the path goes as deep as the pieces do.
    std::shared_ptr<ProcModel> axle = std::make_shared<ProcModel>();
    ProcUse big;
    big.piece = tire;
    big.node = axle->addNode("left", 0, ProcFrame{Vec3(0, 0, 0), Quat()});
    big.placements.push_back(ProcPlacement{Vec3(1, 0, 0), Quat::fromAxisAngle(Vec3(0, 1, 0), 90 * DEG_TO_RAD), 2});
    axle->uses.push_back(big);
    CHECK(axle->markerFrame("left/valve", at));
    CHECK(near(at.position, Vec3(1, 0.4, -0.2)));           // (0.1, 0.2, 0) doubled, then x carried onto -z
    ProcModel truck;
    ProcUse rear;
    rear.piece = axle;
    rear.node = truck.addNode("rear", 0, ProcFrame{Vec3(0, 1, -3), Quat()});
    rear.placements.push_back(ProcPlacement{});
    truck.uses.push_back(rear);
    CHECK(truck.markerFrame("rear/left/valve", at));
    CHECK(near(at.position, Vec3(1, 1.4, -3.2)));
}

TEST_CASE(a_model_may_not_use_a_piece_that_reaches_back_to_it) {
    std::shared_ptr<ProcModel> a = std::make_shared<ProcModel>(), b = std::make_shared<ProcModel>();
    CHECK(a->reaches(a.get()));
    CHECK(!a->reaches(b.get()));
    ProcUse use;
    use.piece = b;
    use.placements.push_back(ProcPlacement{});
    a->uses.push_back(use);
    CHECK(a->reaches(b.get()));
    CHECK(!b->reaches(a.get()));

    // Merging carries uses across with their nodes, and refuses a model that holds this one.
    ProcModel host;
    host.addNode("yard", 0, ProcFrame{});
    ProcModel guest;
    ProcUse onNode;
    onNode.piece = b;
    onNode.node = guest.addNode("gate", 0, ProcFrame{Vec3(4, 0, 0), Quat()});
    onNode.placements.push_back(ProcPlacement{});
    guest.uses.push_back(onNode);
    CHECK(host.merge(guest));
    CHECK(host.uses.size() == 1);
    CHECK(host.uses[0].node == host.findNode("gate"));
    CHECK(host.uses[0].piece == b);

    std::string clash;
    CHECK(!b->merge(*a, &clash));                          // a holds b
    CHECK(clash == "model 'itself'");
}

TEST_CASE(a_recipe_uses_one_tire_at_four_nodes) {
    ProcModel car;
    std::string err;
    CHECK(runModel(R"LUA(
        local tire = model.new()
        tire:material("rubber", material.new{ roughness = 0.9 })
        tire:material("alloy", material.new{ metallic = 0.9 })
        tire:add(mesh.box{ 0.2, 0.6, 0.6 }, "rubber")
        tire:add(mesh.box{ 0.22, 0.4, 0.4 }, "alloy")
        tire:marker("valve", { pos = { 0.1, 0.2, 0 } })
        local ring = {}
        for i = 0, 4 do ring[#ring + 1] = { pos = { 0.12, 0.1 * math.cos(i * 1.2566), 0.1 * math.sin(i * 1.2566) }, turn = { i * 72, 0, 0 } } end
        tire:use(mesh.box{ 0.02, 0.02, 0.02 }, ring, { metallic = 1.0 })      -- five bolts, one mesh

        local car = model.new()
        car:add(mesh.box{ 1.6, 0.6, 4 })
        for name, at in pairs{ wheel_fl = { 0.8, 0.3, 1.3 }, wheel_fr = { -0.8, 0.3, 1.3 },
                               wheel_rl = { 0.8, 0.3, -1.3 }, wheel_rr = { -0.8, 0.3, -1.3 } } do
            assert(car:node(name, { pos = at }):use(tire):name() == name)     -- chainable
        end
        assert(car:instance_count() == 24)
        local at = car:marker_at("wheel_rl/valve")
        assert(math.abs(at[1] - 0.9) < 1e-9 and math.abs(at[2] - 0.5) < 1e-9 and math.abs(at[3] + 1.3) < 1e-9)
        assert(car:marker_at("valve") == nil)
        return car
    )LUA", car, &err));
    if (!err.empty()) std::printf("    script error: %s\n", err.c_str());
    CHECK(car.uses.size() == 4);
    for (const ProcUse& u : car.uses) {
        CHECK(u.piece == car.uses[0].piece);               // one tire, held once
        CHECK(u.node > 0 && u.placements.size() == 1);
    }
    const std::vector<ProcDrawnPiece> drawn = drawnPieces(car);
    CHECK(drawn.size() == 3);
    if (drawn.size() == 3) {
        CHECK(drawn[0].transforms.size() == 4 && drawn[1].transforms.size() == 4 && drawn[2].transforms.size() == 20);
        CHECK_APPROX(drawn[2].piece->materialOf(*drawn[2].part).metallic, 1.0, 1e-6);   // the bare mesh's look
        CHECK_APPROX(drawn[0].piece->materialOf(*drawn[0].part).roughness, 0.9, 1e-6);
    }
    // The fourth bolt is turned 216 degrees about x with the ring (placements turn in degrees).
    const ProcUse& bolts = car.uses[0].piece->uses[0];
    CHECK(near(bolts.placements[3].rotation.rotate(Vec3(0, 1, 0)), Vec3(0, std::cos(216 * DEG_TO_RAD), std::sin(216 * DEG_TO_RAD))));

    // What stops a recipe.
    CHECK(!runModel(R"LUA(local m = model.new() m:use(m) return m)LUA", car, &err));
    CHECK(err.find("cannot use itself") != std::string::npos);
    CHECK(!runModel(R"LUA(local a, b = model.new(), model.new() a:use(b) b:node("n"):use(a) return a)LUA", car, &err));
    CHECK(err.find("cannot use itself") != std::string::npos);
}

TEST_CASE(the_older_word_places_copies_as_it_always_did) {
    ProcModel m;
    std::string err;
    CHECK(runModel(R"LUA(
        local m = model.new()
        m:add_instances(mesh.box{ 0.2, 4, 0.2 },
            { { pos = { 5, 0, 1 }, yaw = 0.7, scale = 1.3 }, { pos = { -3, 0.5, 2 } } },
            { roughness = 0.4, metallic = 0.7, collide = { shape = "capsule", radius = 0.1, height = 4 } })
        m:add_instances(mesh.box{ 1, 1, 0.02 }, { { pos = { 0, 0, 0 } } }, { alpha_foliage = true })
        m:add_instances(mesh.box{ 1, 1, 1 }, { { pos = { 9, 0, 0 } } })
        assert(m:instance_count() == 4)
        return m
    )LUA", m, &err));
    if (!err.empty()) std::printf("    script error: %s\n", err.c_str());
    CHECK(m.uses.size() == 3);
    const std::vector<ProcPlacedUse> placed = placedUses(m);
    CHECK(placed.size() == 3);
    // yaw in RADIANS about the vertical, as the word always read it.
    CHECK(same(placed[0].transforms[0], Mat4::trs(Vec3(5, 0, 1), Quat::fromAxisAngle(Vec3(0, 1, 0), 0.7), Vec3(1.3, 1.3, 1.3))));
    const ProcModel& lamp = *m.uses[0].piece;
    CHECK(lamp.parts.size() == 1 && lamp.nodes.size() == 1);
    CHECK_APPROX(lamp.materialOf(lamp.parts[0]).roughness, 0.4, 1e-6);
    CHECK_APPROX(lamp.materialOf(lamp.parts[0]).metallic, 0.7, 1e-6);
    CHECK(m.uses[0].collision == InstanceCollision::Capsule);
    CHECK_APPROX(m.uses[0].colliderHalfHeight, 2.0, 1e-12);
    CHECK(m.uses[1].piece->materialOf(m.uses[1].piece->parts[0]).alphaFoliage);
    CHECK(m.uses[2].piece->parts[0].material == 0);        // no look given: the plain material
}

TEST_CASE(the_bake_and_flatten_draw_every_copy_of_a_used_piece) {
    const std::shared_ptr<ProcModel> bolt = ProcModel::ofMesh(MeshBuilder::box(Vec3(0.02, 0.02, 0.02)));
    const ProcModel car = makeCar(makeTire(bolt));

    Scene scene;
    bakeProcModel(car, scene, Vec3(100, 0, 0));
    scene.buildAccelerator();
    CHECK(scene.triangles.size() == 12);                    // the body; everything else is instanced
    CHECK(scene.protos.size() == 3);                        // wall, rim, bolt: one each, whatever the copies
    CHECK(scene.instances.size() == 4 + 4 + 20);
    // A ray down the axle line from outside hits the front-left tire's wall at its outer face.
    HitRecord rec;
    CHECK(scene.intersect(Ray(Vec3(105, 0.55, 1.3), Vec3(-1, 0, 0)), 0.001, 1e9, rec));
    CHECK_APPROX(rec.point.x, 100.8 + 0.1, 1e-6);
    // And the rear-right from the other side.
    CHECK(scene.intersect(Ray(Vec3(95, 0.55, -1.3), Vec3(1, 0, 0)), 0.001, 1e9, rec));
    CHECK_APPROX(rec.point.x, 100 - 0.8 - 0.1, 1e-6);

    const RenderMesh flat = flattenModel(car);
    CHECK(flat.indices.size() == static_cast<std::size_t>((12 + 4 * 12 + 4 * 12 + 20 * 12) * 3));
    Real reach = 0;
    for (const Vertex& v : flat.vertices) reach = std::max(reach, std::abs(v.position.x));
    CHECK_APPROX(reach, 0.8 + 0.12 + 0.01, 1e-9);           // the bolts stand proud of the rim
}

TEST_CASE(the_game_draws_one_group_per_part_of_a_piece_and_stamps_its_colliders) {
    const TmpFile recipe("test_model_uses_tmp.lua", R"LUA(
        local tire = model.new()
        tire:add(mesh.box{ 0.2, 0.6, 0.6 }, material.new{ roughness = 0.91 })
        tire:add(mesh.box{ 0.22, 0.4, 0.4 }, material.new{ metallic = 0.92 })
        local car = model.new()
        car:add(mesh.box{ 1.6, 0.6, 4 })
        car:node("wheel_fl", { pos = { 0.8, 0.3, 1.3 } }):use(tire, { collide = { shape = "sphere", radius = 0.3 } })
        car:node("wheel_fr", { pos = { -0.8, 0.3, 1.3 } }):use(tire, { collide = { shape = "sphere", radius = 0.3 } })
        car:node("wheel_rl", { pos = { 0.8, 0.3, -1.3 } }):use(tire)
        car:node("wheel_rr", { pos = { -0.8, 0.3, -1.3 } }):use(tire)
        return car
    )LUA");
    const TmpFile level("test_model_uses_tmp.json", R"({
      "version": 1,
      "entities": [ { "name": "car", "shape": "script", "file": "test_model_uses_tmp.lua" } ]
    })");
    const TmpFile sidecar("test_model_uses_tmp.json.cameras.json", "");

    std::unique_ptr<Renderer> renderer = Renderer::create();
    RendererMeshUploader uploader(*renderer);
    AssetManager assets(uploader);
    World world;
    RenderView view;
    CHECK(LevelLoader::load(level.path, world, *renderer, view, assets, false));
    int groups = 0, walls = 0, rims = 0;
    world.each<InstanceGroup>([&](Entity, InstanceGroup& g) {
        ++groups;
        CHECK(g.transforms.size() == 4);
        if (std::abs(g.material.roughness - 0.91f) < 1e-6f) ++walls;
        if (std::abs(g.material.metallic - 0.92f) < 1e-6f) ++rims;
        bool frontLeft = false;
        for (const Mat4& t : g.transforms) frontLeft = frontLeft || near(origin(t), Vec3(0.8, 0.3, 1.3));
        CHECK(frontLeft);
    });
    CHECK(groups == 2 && walls == 1 && rims == 1);
    // A sphere collider at each of the two front wheels, lifted by its radius as a placed prop's always was.
    int spheres = 0;
    world.each<Collider, Transform>([&](Entity, Collider& c, Transform& t) {
        if (c.shape != ColliderShape::Sphere) return;
        ++spheres;
        CHECK(near(t.position, Vec3(0.8, 0.6, 1.3)) || near(t.position, Vec3(-0.8, 0.6, 1.3)));
    });
    CHECK(spheres == 2);
    world.destroyAll();
    assets.clear();
}
