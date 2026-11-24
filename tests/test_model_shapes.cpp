// COLLISION SHAPES ON NODES (#191, model step 6, first piece).
//
// A collision shape is a simple solid that stands in for the model when things bump into it: a box, a sphere, a
// capsule, the convex hull of some points, or a mesh's own triangles. It has a name and sits on a node, in the
// node's own space, so a boom's box goes with the boom. This is what a model's colliders grew into: those were
// unnamed and in the model's own space, which is now the case "on the root".
//
// What is held here:
//   - the record: a shape's name is unique in the model; a shape on a node is placed through that node, in a pose
//     too; merging and mirroring carry shapes;
//   - a shape as triangles (`shapeMesh`): what the model sheet draws and what a hull is given to the physics as;
//   - Lua: `m:shape`, `n:shape`, and the older words (`m:add_collider`, `m:collide`, `m:add_solid`) unchanged;
//   - the game: a shape on a node becomes a collider standing where the node stands, turned as it is turned.
#include "test_framework.h"

#include "../src/engine/asset_manager.h"
#include "../src/engine/components.h"
#include "../src/engine/level_loader.h"
#include "../src/engine/mesh_builder.h"
#include "../src/engine/mesh_uploader.h"
#include "../src/engine/procgen/proc_model.h"
#include "../src/engine/scripting/procgen_bindings.h"
#include "../src/engine/scripting/script_vm.h"
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

bool runModel(const std::string& code, ProcModel& out, std::string* error = nullptr) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    return runProcgenModelValue(vm, code, out, error);
}

struct Box {
    Vec3 lo{1e30, 1e30, 1e30}, hi{-1e30, -1e30, -1e30};
    void add(const Vec3& p) {
        lo = Vec3(std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z));
        hi = Vec3(std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z));
    }
    Vec3 centre() const { return (lo + hi) * 0.5; }
    Vec3 size() const { return hi - lo; }
};
Box boundsOf(const RenderMesh& m) {
    Box b;
    for (const Vertex& v : m.vertices) b.add(v.position);
    return b;
}

ProcCollider boxShape(const std::string& name, int node, const Vec3& size, const Vec3& at) {
    ProcCollider c;
    c.kind = ProcCollider::Kind::Box;
    c.name = name;
    c.node = node;
    c.halfExtent = size * 0.5;
    c.center = at;
    return c;
}

struct TmpFile {   // a scratch file, removed even when a check fails
    std::string path;
    TmpFile(const std::string& p, const std::string& contents) : path(p) { std::ofstream(path) << contents; }
    ~TmpFile() { std::remove(path.c_str()); }
};

}  // namespace

TEST_CASE(a_collision_shape_has_a_name_and_sits_on_a_node) {
    ProcModel crane;
    const int boom = crane.addNode("boom", 0, ProcFrame{Vec3(0, 5, 0), Quat::fromAxisAngle(Vec3(0, 1, 0), 90 * DEG_TO_RAD)});
    ProcMotion lift;
    lift.kind = ProcMotion::Kind::Turn;
    lift.axis = Vec3(-1, 0, 0);
    crane.nodes[static_cast<std::size_t>(boom)].motion = lift;

    CHECK(crane.addShape(boxShape("base", 0, Vec3(2, 0.4, 2), Vec3(0, 0.2, 0))));
    CHECK(crane.addShape(boxShape("boom", boom, Vec3(0.3, 0.3, 5), Vec3(0, 0, 2.5))));   // a shape may share a node's name
    CHECK(!crane.addShape(boxShape("boom", 0, Vec3(1, 1, 1), Vec3(0, 0, 0))));            // but not another shape's
    CHECK(!crane.addShape(boxShape("lost", 9, Vec3(1, 1, 1), Vec3(0, 0, 0))));
    CHECK(crane.addShape(boxShape("", 0, Vec3(1, 1, 1), Vec3(3, 0, 0))));                 // unnamed, as colliders were
    CHECK(crane.addShape(boxShape("", 0, Vec3(1, 1, 1), Vec3(4, 0, 0))));                 // and as many as wanted
    CHECK(crane.colliders.size() == 4);
    CHECK(crane.findShape("boom") != nullptr && crane.findShape("boom")->node == boom);
    CHECK(crane.findShape("") == nullptr && crane.findShape("jib") == nullptr);

    // Where the boom's box stands in the model: the boom's quarter turn carries its z onto the model's x.
    const ProcCollider& shape = *crane.findShape("boom");
    CHECK(near((crane.nodeFrame(boom) * shape.frame()).position, Vec3(2.5, 5, 0)));
    // In a pose it goes with the node: the boom lifted 90 degrees stands the box straight up.
    ProcPose pose;
    pose.values["boom"] = 90 * DEG_TO_RAD;
    CHECK(near((crane.nodeFrame(boom, pose) * shape.frame()).position, Vec3(0, 7.5, 0)));
    const std::shared_ptr<ProcModel> held = posedModel(crane, pose);
    CHECK(held != nullptr);
    if (held != nullptr) CHECK(near((held->nodeFrame(boom) * held->findShape("boom")->frame()).position, Vec3(0, 7.5, 0)));
}

TEST_CASE(a_collision_shape_as_triangles_is_the_solid_it_names) {
    ProcCollider box = boxShape("b", 0, Vec3(2, 1, 4), Vec3(1, 2, 3));
    box.rotation = Quat::fromAxisAngle(Vec3(0, 1, 0), 90 * DEG_TO_RAD);
    const Box b = boundsOf(shapeMesh(box));
    CHECK(near(b.centre(), Vec3(1, 2, 3)));
    CHECK(near(b.size(), Vec3(4, 1, 2)));                   // turned a quarter: its length lies along x

    ProcCollider ball;
    ball.kind = ProcCollider::Kind::Sphere;
    ball.radius = 0.5;
    ball.center = Vec3(0, 3, 0);
    const Box s = boundsOf(shapeMesh(ball));
    CHECK(near(s.centre(), Vec3(0, 3, 0), 1e-6));
    CHECK(std::abs(s.size().y - 1.0) < 1e-6);

    ProcCollider pill;
    pill.kind = ProcCollider::Kind::Capsule;
    pill.radius = 0.2;
    pill.halfHeight = 1.0;
    pill.rotation = Quat::fromAxisAngle(Vec3(0, 0, 1), 90 * DEG_TO_RAD);   // its length (its own y) laid along x
    const Box p = boundsOf(shapeMesh(pill));
    CHECK(std::abs(p.size().x - 2.4) < 1e-5);
    CHECK(std::abs(p.size().y - 0.4) < 1e-2);

    // A hull is the convex shape round the mesh's points: a box's points and a point above it make a roofed box.
    ProcCollider hull;
    hull.kind = ProcCollider::Kind::Hull;
    hull.mesh = MeshBuilder::box(Vec3(2, 2, 2));
    Vertex peak;
    peak.position = Vec3(0, 3, 0);
    hull.mesh.vertices.push_back(peak);
    hull.center = Vec3(10, 0, 0);
    const RenderMesh faces = shapeMesh(hull);
    CHECK(faces.indices.size() / 3 == 10 + 4);              // the box less its top (10 triangles), and four roof faces
    const Box h = boundsOf(faces);
    CHECK(near(h.lo, Vec3(9, -1, -1)) && near(h.hi, Vec3(11, 3, 1)));

    // A mesh shape is its own triangles.
    ProcCollider exact;
    exact.kind = ProcCollider::Kind::Mesh;
    exact.mesh = MeshBuilder::box(Vec3(1, 1, 1));
    CHECK(shapeMesh(exact).indices == exact.mesh.indices);
}

TEST_CASE(merging_and_mirroring_carry_collision_shapes) {
    ProcModel car;
    car.addShape(boxShape("body", 0, Vec3(2, 1, 4), Vec3(0, 0.5, 0)));
    ProcModel door;
    const int hinge = door.addNode("hinge", 0, ProcFrame{Vec3(0.9, 0.5, 1.0), Quat()});
    door.addShape(boxShape("door", hinge, Vec3(0.1, 1, 1), Vec3(0.05, 0, -0.5)));
    CHECK(car.merge(door));
    CHECK(car.colliders.size() == 2);
    CHECK(car.findShape("door") != nullptr && car.findShape("door")->node == car.findNode("hinge"));

    std::string clash;
    ProcModel twinDoor;
    twinDoor.addShape(boxShape("door", 0, Vec3(1, 1, 1), Vec3(0, 0, 0)));
    CHECK(!car.merge(twinDoor, &clash));
    CHECK(clash == "collision shape 'door'");
    ProcModel unnamed;                                      // unnamed shapes never clash
    unnamed.addShape(boxShape("", 0, Vec3(1, 1, 1), Vec3(0, 0, 0)));
    CHECK(car.merge(unnamed) && car.merge(unnamed));
    CHECK(car.colliders.size() == 4);

    // Mirrored left to right, the door's box is on the other side, still on its hinge.
    const std::shared_ptr<ProcModel> twin = mirroredModel(car);
    CHECK(twin != nullptr);
    if (twin != nullptr) {
        const ProcCollider* was = car.findShape("door");
        const ProcCollider* is = twin->findShape("door");
        CHECK(is != nullptr && is->node == was->node);
        if (is != nullptr) {
            const Vec3 a = (car.nodeFrame(was->node) * was->frame()).position, b = (twin->nodeFrame(is->node) * is->frame()).position;
            CHECK(near(b, Vec3(-a.x, a.y, a.z)));
        }
    }
    // A hull's points are mirrored with it.
    ProcModel rock;
    ProcCollider hull;
    hull.kind = ProcCollider::Kind::Hull;
    hull.name = "rock";
    hull.mesh = MeshBuilder::box(Vec3(1, 1, 1));
    MeshBuilder::transform(hull.mesh, Mat4::translate(3, 0, 0));
    rock.addShape(hull);
    const std::shared_ptr<ProcModel> rockTwin = mirroredModel(rock);
    CHECK(rockTwin != nullptr && near(boundsOf(shapeMesh(*rockTwin->findShape("rock"))).centre(), Vec3(-3, 0, 0)));
}

TEST_CASE(a_recipe_gives_a_model_collision_shapes) {
    ProcModel m;
    std::string err;
    CHECK(runModel(R"LUA(
        local m = model.new()
        assert(m:shape("base", { box = { 2, 0.4, 2 }, pos = { 0, 0.2, 0 }, friction = 0.9 }) == m)      -- chainable
        local boom = m:node("boom", { pos = { 0, 5, 0 }, turn = { 0, 90, 0 } })
        assert(boom:shape("boom", { capsule = { radius = 0.2, height = 5 }, pos = { 0, 0, 2.5 }, turn = { 90, 0, 0 } }) == boom)
        boom:shape("ball", { sphere = 0.6, pos = { 0, 0, 5 } })
        boom:shape("hook", { hull = mesh.box{ 0.4, 0.8, 0.4 }, pos = { 0, -1, 5 } })
        m:shape("ground", { mesh = mesh.box{ 20, 0.1, 20 } })
        -- the older words still say what they said: unnamed shapes on the root
        m:add_collider{ shape = "box", size = { 1, 1, 1 }, center = { 5, 0, 0 }, yaw = 0.5 }
        m:collide(mesh.box{ 1, 1, 1 })
        m:add_solid(mesh.box{ 1, 1, 1 })
        assert(m:collider_count() == 8)
        return m
    )LUA", m, &err));
    if (!err.empty()) std::printf("    script error: %s\n", err.c_str());
    CHECK(m.colliders.size() == 8);
    if (m.colliders.size() != 8) return;
    const ProcCollider* base = m.findShape("base");
    CHECK(base != nullptr && base->kind == ProcCollider::Kind::Box && base->node == 0);
    if (base != nullptr) {
        CHECK(near(base->halfExtent, Vec3(1, 0.2, 1)));
        CHECK_APPROX(base->friction, 0.9, 1e-12);
    }
    const ProcCollider* boom = m.findShape("boom");
    CHECK(boom != nullptr && boom->kind == ProcCollider::Kind::Capsule && boom->node == m.findNode("boom"));
    if (boom != nullptr) {
        CHECK_APPROX(boom->halfHeight, 2.5, 1e-12);          // `height` is the straight part
        CHECK(near(boom->rotation.rotate(Vec3(0, 1, 0)), Vec3(0, 0, 1)));   // turned 90 degrees about x: its length lies along the boom
    }
    CHECK(m.findShape("ball") != nullptr && m.findShape("ball")->kind == ProcCollider::Kind::Sphere);
    CHECK(m.findShape("hook") != nullptr && m.findShape("hook")->kind == ProcCollider::Kind::Hull);
    CHECK(m.findShape("ground") != nullptr && m.findShape("ground")->kind == ProcCollider::Kind::Mesh);
    const ProcCollider& older = m.colliders[5];
    CHECK(older.name.empty() && older.node == 0 && older.kind == ProcCollider::Kind::Box && near(older.center, Vec3(5, 0, 0)));
    CHECK(near(older.rotation.rotate(Vec3(0, 0, 1)), Vec3(std::sin(0.5), 0, std::cos(0.5))));   // yaw, in radians, as it was

    struct Wrong { const char* code; const char* says; };
    const Wrong wrongs[] = {
        {R"LUA(local m = model.new() m:shape("a", { box = {1,1,1} }) m:node("n"):shape("a", { sphere = 1 }) return m)LUA", "already has a collision shape named 'a'"},
        {R"LUA(local m = model.new() m:shape("", { box = {1,1,1} }) return m)LUA", "needs a name"},
        {R"LUA(local m = model.new() m:shape("a", { pos = {0,0,0} }) return m)LUA", "give one of box, sphere, capsule, hull, mesh"},
        {R"LUA(local m = model.new() m:shape("a", { box = {1,1,1}, sphere = 1 }) return m)LUA", "give one of box, sphere, capsule, hull, mesh"},
        {R"LUA(local m = model.new() m:shape("a", { hull = mesh.plane(2, 2) }) return m)LUA", "too few or all in one plane"},
    };
    for (const Wrong& w : wrongs) {
        err.clear();
        CHECK(!runModel(w.code, m, &err));
        if (err.find(w.says) == std::string::npos) std::printf("    wanted '%s', got: %s\n", w.says, err.c_str());
        CHECK(err.find(w.says) != std::string::npos);
    }
}

TEST_CASE(the_game_stands_a_shape_where_its_node_stands) {
    const TmpFile recipe("test_model_shapes_tmp.lua", R"LUA(
        local m = model.new()
        m:add(mesh.box{ 2, 0.4, 2 })
        m:shape("base", { box = { 2, 0.4, 2 }, pos = { 0, 0.2, 0 } })
        local boom = m:node("boom", { pos = { 0, 5, 0 }, turn = { 0, 90, 0 } })
        boom:shape("boom", { capsule = { radius = 0.2, height = 5 }, pos = { 0, 0, 2.5 }, turn = { 90, 0, 0 } })
        boom:shape("ball", { sphere = 0.6, pos = { 0, 0, 5 } })
        boom:shape("plate", { mesh = mesh.box{ 1, 0.1, 1 } })
        boom:shape("hook", { hull = mesh.box{ 0.4, 0.8, 0.4 }, pos = { 0, -1, 5 } })
        return m
    )LUA");
    const TmpFile level("test_model_shapes_tmp.json", R"({
      "version": 1,
      "entities": [ { "name": "crane", "shape": "script", "file": "test_model_shapes_tmp.lua" } ]
    })");
    const TmpFile sidecar("test_model_shapes_tmp.json.cameras.json", "");

    std::unique_ptr<Renderer> renderer = Renderer::create();
    RendererMeshUploader uploader(*renderer);
    AssetManager assets(uploader);
    World world;
    RenderView view;
    CHECK(LevelLoader::load(level.path, world, *renderer, view, assets, false));
    int boxes = 0, capsules = 0, spheres = 0;
    world.each<Collider, Transform>([&](Entity, Collider& c, Transform& t) {
        if (c.shape == ColliderShape::Box) {
            ++boxes;
            CHECK(near(t.position, Vec3(0, 0.2, 0)));
        } else if (c.shape == ColliderShape::Capsule) {
            ++capsules;
            CHECK(near(t.position, Vec3(2.5, 5, 0)));                         // half way along the boom, which points along x
            CHECK(near(t.orientation.rotate(Vec3(0, 1, 0)), Vec3(1, 0, 0)));   // and its length lies along it
            CHECK_APPROX(c.halfHeight, 2.5, 1e-12);
        } else {
            ++spheres;
            CHECK(near(t.position, Vec3(5, 5, 0)));
        }
    });
    CHECK(boxes == 1 && capsules == 1 && spheres == 1);
    // The plate's triangles and the hull's faces are static triangles, standing where the boom stands.
    Box tris;
    std::size_t count = 0;
    world.each<MeshCollider>([&](Entity, MeshCollider& mc) {
        for (const Vec3& v : mc.vertices) tris.add(v);
        count += mc.indices.size() / 3;
    });
    CHECK(count == 12 + 12);
    // (to a micron: the box primitive's corners are single precision)
    CHECK(near(tris.lo, Vec3(-0.5, 3.6, -0.5), 1e-6));       // the hook hangs 1 m under the boom's tip, 0.8 tall
    CHECK(near(tris.hi, Vec3(5.2, 5.05, 0.5), 1e-6));
    world.destroyAll();
    assets.clear();
}
