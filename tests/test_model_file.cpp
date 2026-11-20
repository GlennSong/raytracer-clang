// A MODEL AS A glTF FILE, AND BACK (#191, model step 6).
//
// A model made outside the engine says what its pieces are by their names (AGENTS.md "Model file names"); a model made
// by a recipe is written out the same way, so it opens in Blender as the tree it is and reads back as the model it
// was.
//
// What is held here:
//   - the round trip: a model with everything in it (nodes that move, markers, each kind of collision shape,
//     materials, levels of detail, a used piece with a piece inside it) written to a file and read back is the same
//     model, to the precision of a file (single-precision positions);
//   - a file as Blender makes one: no `root`, an object scaled, a level by a `_lod1` name, a `col_box_` object, an
//     empty: read into nodes, parts, a shape and a marker, the scale put into the mesh;
//   - levels of detail as separate files, matched to the first file's nodes by name;
//   - Lua: `model.from_file`, `model.from_files`, a file's name and never a path, and a host that loads none;
//   - what a file cannot say is refused, not dropped.
#include "test_framework.h"

#include "../src/engine/glb_export.h"
#include "../src/engine/mesh_builder.h"
#include "../src/engine/model_importer.h"
#include "../src/engine/procgen/model_file.h"
#include "../src/engine/procgen/proc_model.h"
#include "../src/engine/scripting/procgen_bindings.h"
#include "../src/engine/scripting/script_vm.h"

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

using namespace engine;

namespace {

bool near(const Vec3& a, const Vec3& b, Real eps = 1e-5) { return (a - b).length() < eps; }
bool sameTurn(const Quat& a, const Quat& b) { return std::abs(std::abs(a.dot(b)) - 1.0) < 1e-9; }
bool sameFrame(const ProcFrame& a, const ProcFrame& b) { return near(a.position, b.position, 1e-9) && sameTurn(a.rotation, b.rotation); }

struct Removed {   // a scratch file, removed even when a check fails
    std::string path;
    explicit Removed(const std::string& p) : path(p) {}
    ~Removed() { std::remove(path.c_str()); }
};

RenderMesh painted(RenderMesh m, const Vec3& colour) {
    for (Vertex& v : m.vertices) v.color = colour;
    return m;
}

struct Sums { Vec3 sum, cubes, colours; std::size_t points = 0, triangles = 0; long facing = 0; };
Sums sumsOf(const RenderMesh& mesh) {
    Sums s;
    for (const Vertex& v : mesh.vertices) {
        s.sum = s.sum + v.position;
        s.cubes = s.cubes + Vec3(v.position.x * v.position.x * v.position.x, v.position.y * v.position.y * v.position.y, v.position.z * v.position.z * v.position.z);
        s.colours = s.colours + v.color;
        ++s.points;
    }
    s.triangles = mesh.indices.size() / 3;
    for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        const Vertex& a = mesh.vertices[mesh.indices[i]];
        const Vertex& b = mesh.vertices[mesh.indices[i + 1]];
        const Vertex& c = mesh.vertices[mesh.indices[i + 2]];
        if (dot(cross(b.position - a.position, c.position - a.position), a.normal) > 0) ++s.facing;
    }
    return s;
}
bool sameSums(const Sums& a, const Sums& b) {
    return a.points == b.points && a.triangles == b.triangles && a.facing == b.facing && near(a.sum, b.sum, 1e-3) && near(a.cubes, b.cubes, 1e-2) &&
           near(a.colours, b.colours, 1e-3);
}

// A tire: two materials, a marker, and three bolts that are one bolt piece.
std::shared_ptr<ProcModel> makeTire() {
    std::shared_ptr<ProcModel> bolt = ProcModel::ofMesh(painted(MeshBuilder::box(Vec3(0.02, 0.03, 0.03)), Vec3(0.2, 0.2, 0.25)));
    bolt->name = "bolt";
    std::shared_ptr<ProcModel> tire = std::make_shared<ProcModel>();
    tire->name = "tire";
    std::shared_ptr<ProcMaterial> rubber = std::make_shared<ProcMaterial>();
    rubber->roughness = 0.9f;
    std::shared_ptr<ProcMaterial> alloy = std::make_shared<ProcMaterial>();
    alloy->metallic = 0.9f;
    alloy->roughness = 0.25f;
    { ProcPart p; p.mesh = painted(MeshBuilder::box(Vec3(0.22, 0.66, 0.66)), Vec3(0.05, 0.05, 0.05)); p.material = tire->addMaterial("rubber", rubber); tire->parts.push_back(p); }
    { ProcPart p; p.mesh = painted(MeshBuilder::box(Vec3(0.04, 0.4, 0.4)), Vec3(0.75, 0.76, 0.78)); MeshBuilder::transform(p.mesh, Mat4::translate(0.1, 0, 0));
      p.material = tire->addMaterial("alloy", alloy); tire->parts.push_back(p); }
    ProcMarker valve;
    valve.name = "valve";
    valve.frame.position = Vec3(0.12, 0.17, 0);
    tire->addMarker(0, valve);
    ProcUse bolts;
    bolts.piece = bolt;
    for (int i = 0; i < 3; ++i) bolts.placements.push_back(ProcPlacement{Vec3(0.125, 0.09 * std::cos(i * 2.0944), 0.09 * std::sin(i * 2.0944)), Quat::fromEuler(i * 0.7, 0, 0), 1});
    tire->uses.push_back(bolts);
    return tire;
}

// A small car with everything a model can hold.
std::shared_ptr<ProcModel> makeCar() {
    std::shared_ptr<ProcModel> car = std::make_shared<ProcModel>();
    car->name = "car";
    std::shared_ptr<ProcMaterial> paint = std::make_shared<ProcMaterial>();
    paint->roughness = 0.3f;
    paint->metallic = 0.55f;
    std::shared_ptr<ProcMaterial> glass = std::make_shared<ProcMaterial>();
    glass->roughness = 0.05f;
    glass->opacity = 0.3f;
    glass->twoSided = true;
    std::shared_ptr<ProcMaterial> lamp = std::make_shared<ProcMaterial>();
    lamp->emission = Vec3(2, 1.5, 1);
    std::shared_ptr<ProcMaterial> brick = std::make_shared<ProcMaterial>();   // a material with a map
    brick->textured = true;
    brick->tile = 2.5;
    brick->albedo.width = 2; brick->albedo.height = 2; brick->albedo.channels = 3;
    brick->albedo.pixels = {200, 10, 10, 10, 200, 10, 10, 10, 200, 90, 90, 90};
    const int mPaint = car->addMaterial("paint", paint), mGlass = car->addMaterial("glass", glass);
    const int mLamp = car->addMaterial("", lamp), mBrick = car->addMaterial("brick", brick);

    car->addLevel(40);
    car->addLevel(150);
    { ProcPart p; p.mesh = painted(MeshBuilder::box(Vec3(1.5, 0.5, 4)), Vec3(0.6, 0.08, 0.06)); MeshBuilder::transform(p.mesh, Mat4::translate(0, 0.6, 0)); p.material = mPaint; car->parts.push_back(p); }
    { ProcPart p; p.mesh = MeshBuilder::box(Vec3(0.2, 0.1, 0.05)); MeshBuilder::transform(p.mesh, Mat4::translate(0.5, 0.6, 2)); p.material = mLamp; car->parts.push_back(p); }
    { ProcPart p; p.mesh = MeshBuilder::box(Vec3(0.3, 0.3, 0.3)); MeshBuilder::transform(p.mesh, Mat4::translate(0, 1.2, 0)); car->parts.push_back(p); }   // plain
    { ProcPart p; p.mesh = MeshBuilder::box(Vec3(0.4, 0.2, 0.4)); MeshBuilder::transform(p.mesh, Mat4::translate(0, 0.2, -1.8)); p.material = mBrick; car->parts.push_back(p); }
    { ProcPart p; p.mesh = painted(MeshBuilder::box(Vec3(1.5, 0.9, 4)), Vec3(0.6, 0.08, 0.06)); MeshBuilder::transform(p.mesh, Mat4::translate(0, 0.6, 0)); p.material = mPaint; p.level = 2; car->parts.push_back(p); }

    // a door on a hinge that turns, with glass, a handle marker and its own box to bump into
    const int door = car->addNode("door_front_l", 0, ProcFrame{Vec3(0.775, 0.4, 0.75), Quat::fromEuler(0.02, 0.05, -0.03)});
    ProcMotion swing;
    swing.kind = ProcMotion::Kind::Turn;
    swing.axis = Vec3(0, -1, 0);
    swing.limited = true;
    swing.max = 70 * DEG_TO_RAD;
    car->nodes[static_cast<std::size_t>(door)].motion = swing;
    { ProcPart p; p.mesh = painted(MeshBuilder::box(Vec3(0.05, 0.42, 1.0)), Vec3(0.75, 0.12, 0.08)); MeshBuilder::transform(p.mesh, Mat4::translate(0, 0.22, -0.5)); p.node = door; p.material = mPaint; car->parts.push_back(p); }
    { ProcPart p; p.mesh = MeshBuilder::box(Vec3(0.04, 0.32, 0.8)); MeshBuilder::transform(p.mesh, Mat4::translate(-0.05, 0.68, -0.45)); p.node = door; p.material = mGlass; car->parts.push_back(p); }
    { ProcPart p; p.mesh = painted(MeshBuilder::box(Vec3(0.05, 0.42, 1.0)), Vec3(0.75, 0.12, 0.08)); MeshBuilder::transform(p.mesh, Mat4::translate(0, 0.22, -0.5)); p.node = door; p.material = mPaint; p.level = 1; car->parts.push_back(p); }
    ProcMarker handle;
    handle.name = "handle";
    handle.role = "grip";
    handle.size = 0.15;
    handle.frame = ProcFrame{Vec3(0.07, 0.32, -0.85), Quat::fromEuler(0, 1.2, 0)};
    car->addMarker(door, handle);
    ProcMarker seat;
    seat.name = "driver";
    seat.frame.position = Vec3(-0.4, 0.54, -0.58);
    car->addMarker(0, seat);

    // a wheel: a suspension that slides, a spin with no limits under it, and the tire used there
    const int susp = car->addNode("susp_front_l", 0, ProcFrame{Vec3(0.8, 0.33, 1.3), Quat()});
    ProcMotion travel;
    travel.kind = ProcMotion::Kind::Slide;
    travel.limited = true;
    travel.min = -0.1;
    travel.max = 0.08;
    car->nodes[static_cast<std::size_t>(susp)].motion = travel;
    const int spin = car->addNode("spin_front_l", susp, ProcFrame{});
    ProcMotion roll;
    roll.kind = ProcMotion::Kind::Turn;
    roll.axis = Vec3(1, 0, 0);
    car->nodes[static_cast<std::size_t>(spin)].motion = roll;
    const int far = car->addNode("susp_front_r", 0, ProcFrame{Vec3(-0.8, 0.33, 1.3), Quat::fromAxisAngle(Vec3(0, 1, 0), 180 * DEG_TO_RAD)});
    const std::shared_ptr<ProcModel> tire = makeTire();
    for (const int node : {spin, far}) {
        ProcUse use;
        use.piece = tire;
        use.node = node;
        use.placements.push_back(ProcPlacement{Vec3(0, 0, 0), Quat(), node == far ? 1.25 : 1.0});
        car->uses.push_back(use);
    }
    {   // the tire again at the far level, twice on the root, with a collider per copy
        ProcUse use;
        use.piece = tire;
        use.level = 2;
        use.placements.push_back(ProcPlacement{Vec3(0.8, 0.33, -1.3), Quat(), 1});
        use.placements.push_back(ProcPlacement{Vec3(-0.8, 0.33, -1.3), Quat::fromAxisAngle(Vec3(0, 1, 0), 180 * DEG_TO_RAD), 1});
        use.collision = InstanceCollision::Sphere;
        use.colliderRadius = 0.33;
        use.colliderFriction = 0.9;
        car->uses.push_back(use);
    }

    // one of each kind of collision shape
    ProcCollider box;
    box.kind = ProcCollider::Kind::Box;
    box.name = "body";
    box.halfExtent = Vec3(0.75, 0.25, 2);
    box.center = Vec3(0, 0.6, 0);
    box.friction = 0.8;
    car->addShape(box);
    ProcCollider panel = box;
    panel.name = "door_front_l";
    panel.node = door;
    panel.halfExtent = Vec3(0.03, 0.21, 0.5);
    panel.center = Vec3(0, 0.22, -0.5);
    panel.rotation = Quat::fromEuler(0, 0.1, 0);
    car->addShape(panel);
    ProcCollider ball;
    ball.kind = ProcCollider::Kind::Sphere;
    ball.name = "wheel_front_l";
    ball.node = susp;
    ball.radius = 0.33;
    car->addShape(ball);
    ProcCollider pill;
    pill.kind = ProcCollider::Kind::Capsule;
    pill.name = "axle";
    pill.radius = 0.05;
    pill.halfHeight = 0.7;
    pill.center = Vec3(0, 0.33, 1.3);
    pill.rotation = Quat::fromAxisAngle(Vec3(0, 0, 1), 90 * DEG_TO_RAD);
    car->addShape(pill);
    ProcCollider hull;
    hull.kind = ProcCollider::Kind::Hull;
    hull.name = "cabin";
    hull.mesh = MeshBuilder::box(Vec3(1.2, 0.4, 2));
    Vertex peak;
    peak.position = Vec3(0, 0.5, 0);
    hull.mesh.vertices.push_back(peak);
    hull.center = Vec3(0, 1.05, -0.2);
    car->addShape(hull);
    ProcCollider tris;
    tris.kind = ProcCollider::Kind::Mesh;
    tris.node = door;                                     // unnamed, on a node
    tris.mesh = MeshBuilder::box(Vec3(0.02, 0.3, 0.3));
    car->addShape(tris);
    return car;
}

// The two models are the same model, to a file's precision.
void checkSameModel(const ProcModel& a, const ProcModel& b, const char* what) {
    std::printf("    comparing %s\n", what);
    CHECK(a.name == b.name);
    CHECK(a.nodes.size() == b.nodes.size());
    CHECK(a.levelFrom == b.levelFrom);
    for (const ProcNode& n : a.nodes) {
        const int i = a.findNode(n.name), j = b.findNode(n.name);
        CHECK(j >= 0);
        if (j < 0) continue;
        const ProcNode& o = b.nodes[static_cast<std::size_t>(j)];
        CHECK((n.parent < 0) == (o.parent < 0));
        if (n.parent >= 0 && o.parent >= 0) CHECK(a.nodes[static_cast<std::size_t>(n.parent)].name == b.nodes[static_cast<std::size_t>(o.parent)].name);
        CHECK(sameFrame(n.frame, o.frame));
        CHECK(n.motion.kind == o.motion.kind && n.motion.limited == o.motion.limited);
        if (n.motion.kind != ProcMotion::Kind::None) {
            CHECK(near(n.motion.axis, o.motion.axis, 1e-9));
            CHECK_APPROX(n.motion.min, o.motion.min, 1e-9);
            CHECK_APPROX(n.motion.max, o.motion.max, 1e-9);
        }
        CHECK(n.markers.size() == o.markers.size());
        for (const ProcMarker& mk : n.markers) {
            int on = -1;
            const ProcMarker* other = b.findMarker(mk.name, &on);
            CHECK(other != nullptr && on == j);
            if (other == nullptr) continue;
            CHECK(other->role == mk.role);
            CHECK_APPROX(other->size, mk.size, 1e-9);
            CHECK(sameFrame(other->frame, mk.frame));
        }
        // the parts on this node, level by level: the same points, colours and facing; made of the same things
        for (std::size_t level = 0; level < a.levelFrom.size(); ++level) {
            RenderMesh mine, theirs;
            std::vector<std::string> madeMine, madeTheirs;
            for (const ProcPart& p : a.parts)
                if (p.node == i && p.level == static_cast<int>(level)) { MeshBuilder::append(mine, p.mesh); madeMine.push_back(a.materials[static_cast<std::size_t>(p.material)].name); }
            for (const ProcPart& p : b.parts)
                if (p.node == j && p.level == static_cast<int>(level)) { MeshBuilder::append(theirs, p.mesh); madeTheirs.push_back(b.materials[static_cast<std::size_t>(p.material)].name); }
            CHECK(madeMine == madeTheirs);
            CHECK(sameSums(sumsOf(mine), sumsOf(theirs)));
        }
    }
    // materials: each part's partner in `b` is the part on the same node and level with the same points
    for (const ProcPart& part : a.parts) {
        const ProcMaterial& x = a.materialOf(part);
        const Sums mine = sumsOf(part.mesh);
        bool found = false;
        for (const ProcPart& q : b.parts) {
            if (b.nodes[static_cast<std::size_t>(q.node)].name != a.nodes[static_cast<std::size_t>(part.node)].name || q.level != part.level) continue;
            if (!sameSums(mine, sumsOf(q.mesh))) continue;
            found = true;
            const ProcMaterial& y = b.materialOf(q);
            CHECK(b.materials[static_cast<std::size_t>(q.material)].name == a.materials[static_cast<std::size_t>(part.material)].name);
            CHECK_APPROX(x.roughness, y.roughness, 1e-6);
            CHECK_APPROX(x.metallic, y.metallic, 1e-6);
            CHECK_APPROX(x.opacity, y.opacity, 1e-6);
            CHECK(near(x.emission, y.emission, 1e-6));
            CHECK(x.twoSided == y.twoSided && x.textured == y.textured);
            if (x.textured) {
                CHECK_APPROX(x.tile, y.tile, 1e-9);
                CHECK(x.albedo.width == y.albedo.width && x.albedo.height == y.albedo.height);
                CHECK(y.albedo.pixels.size() >= 3 && y.albedo.pixels[0] == x.albedo.pixels[0] && y.albedo.pixels[1] == x.albedo.pixels[1]);
            }
            break;
        }
        CHECK(found);
    }
    // collision shapes: named ones by name
    CHECK(a.colliders.size() == b.colliders.size());
    for (const ProcCollider& c : a.colliders) {
        if (c.name.empty()) continue;
        const ProcCollider* o = b.findShape(c.name);
        CHECK(o != nullptr);
        if (o == nullptr) continue;
        CHECK(o->kind == c.kind);
        CHECK(a.nodes[static_cast<std::size_t>(c.node)].name == b.nodes[static_cast<std::size_t>(o->node)].name);
        CHECK_APPROX(o->friction, c.friction, 1e-9);
        // the same solid in the same place: its triangles' points agree
        CHECK(sameSums(sumsOf(shapeMesh(c)), sumsOf(shapeMesh(*o))));
        if (c.kind == ProcCollider::Kind::Box) CHECK(near(o->halfExtent, c.halfExtent));
        if (c.kind == ProcCollider::Kind::Sphere || c.kind == ProcCollider::Kind::Capsule) CHECK_APPROX(o->radius, c.radius, 1e-5);
        if (c.kind == ProcCollider::Kind::Capsule) CHECK_APPROX(o->halfHeight, c.halfHeight, 1e-5);
    }
    // everything drawn, at each level's distance: the same points in the same places, used pieces and all
    for (const Real distance : a.levelFrom) CHECK(sameSums(sumsOf(flattenModel(a, distance)), sumsOf(flattenModel(b, distance))));
    CHECK(a.instanceCount() == b.instanceCount());
}

}  // namespace

TEST_CASE(a_model_written_to_a_file_reads_back_as_the_model_it_was) {
    const Removed file("test_model_file_tmp.glb");
    const std::shared_ptr<ProcModel> car = makeCar();
    std::string err;
    CHECK(writeModelGlb(*car, file.path, &err));
    if (!err.empty()) std::printf("    write: %s\n", err.c_str());
    std::vector<std::string> notes;
    const std::shared_ptr<ProcModel> back = readModelGlb(file.path, &err, &notes);
    CHECK(back != nullptr);
    if (back == nullptr) { std::printf("    read: %s\n", err.c_str()); return; }
    for (const std::string& n : notes) std::printf("    note: %s\n", n.c_str());
    CHECK(notes.empty());                                  // nothing had to be put right

    checkSameModel(*car, *back, "the car");
    // The tire is ONE piece again, used where it was: the same model behind all four copies.
    CHECK(back->uses.size() == 3);
    const ProcModel* tire = nullptr;
    int copies = 0;
    for (const ProcUse& u : back->uses) {
        CHECK(u.piece != nullptr);
        if (tire == nullptr) tire = u.piece.get();
        CHECK(u.piece.get() == tire);
        copies += static_cast<int>(u.placements.size());
        if (u.level == 2) {
            CHECK(u.collision == InstanceCollision::Sphere);
            CHECK_APPROX(u.colliderRadius, 0.33, 1e-9);
            CHECK_APPROX(u.colliderFriction, 0.9, 1e-9);
        }
    }
    CHECK(copies == 4);
    if (tire != nullptr) {
        checkSameModel(*makeTire(), *tire, "the tire, read from its first copy");
        CHECK(tire->uses.size() == 1 && tire->uses[0].placements.size() == 3 && tire->uses[0].piece->name == "bolt");
    }
    // What is inside a piece is where it was, through the node that uses it: the far side's tire is turned and 1.25 x.
    ProcFrame was, is;
    CHECK(car->markerFrame("susp_front_r/valve", was) && back->markerFrame("susp_front_r/valve", is));
    CHECK(near(was.position, is.position, 1e-9));
    CHECK(car->markerFrame("door_front_l/handle", was) == false);          // (the handle is the car's own, on the door)
    CHECK(car->markerFrame("handle", was) && back->markerFrame("handle", is) && sameFrame(was, is));
    // In a pose the two still agree: the joints came back with their limits.
    ProcPose pose;
    pose.values["door_front_l"] = 40 * DEG_TO_RAD;
    pose.values["susp_front_l"] = 0.05;
    pose.values["spin_front_l"] = 2.0;
    const std::shared_ptr<ProcModel> a = posedModel(*car, pose), b = posedModel(*back, pose, &err);
    CHECK(a != nullptr && b != nullptr);
    if (a != nullptr && b != nullptr) CHECK(sameSums(sumsOf(flattenModel(*a)), sumsOf(flattenModel(*b))));

    // Written again from what was read, and read again, it is the same once more (nothing drifts).
    const Removed again("test_model_file_tmp2.glb");
    CHECK(writeModelGlb(*back, again.path, &err));
    const std::shared_ptr<ProcModel> twice = readModelGlb(again.path, &err);
    CHECK(twice != nullptr);
    if (twice != nullptr) checkSameModel(*back, *twice, "the car, written and read a second time");

    // The file is true glTF: the engine's plain importer, which knows nothing of models, reads the same triangles,
    // facing out.
    const CpuModel plain = ModelImporter::loadCpu(file.path);
    CHECK(!plain.meshes.empty());
    long facing = 0, triangles = 0;
    for (const CpuMesh& m : plain.meshes) { const Sums s = sumsOf(m.geometry); facing += s.facing; triangles += static_cast<long>(s.triangles); }
    const Sums ours = sumsOf(flattenModel(*car));
    CHECK(triangles > 0);
    CHECK((facing == 0) == (ours.facing == 0));            // wound the way the engine's own meshes are
}

TEST_CASE(a_file_as_blender_makes_one_is_read_by_its_names) {
    // No `root`: two top objects. `body` is scaled 2 x; under it a level of detail by name, a box to bump into, and
    // an empty. `mast` is an empty with a child, so it is a node, not a marker.
    const Removed file("test_model_file_blender_tmp.glb");
    const RenderMesh unit = painted(MeshBuilder::box(Vec3(1, 1, 1)), Vec3(0.2, 0.6, 0.9));
    const RenderMesh low = MeshBuilder::box(Vec3(1, 1, 1));
    const RenderMesh colBox = MeshBuilder::box(Vec3(1, 0.5, 2));
    const RenderMesh lampMesh = MeshBuilder::box(Vec3(0.2, 0.2, 0.2));
    std::vector<GlbTreeMaterial> materials(1);
    materials[0].name = "steel";
    materials[0].metallic = 0.7f;
    materials[0].roughness = 0.4f;
    std::vector<GlbTreeNode> nodes;
    { GlbTreeNode n; n.name = "body"; n.translation = Vec3(0, 1, 0); n.scale = 2; n.primitives.push_back(GlbTreePrimitive{&unit, 0}); nodes.push_back(n); }
    { GlbTreeNode n; n.name = "body_lod1"; n.parent = 0; n.primitives.push_back(GlbTreePrimitive{&low, 0}); nodes.push_back(n); }
    { GlbTreeNode n; n.name = "col_box_hull"; n.parent = 0; n.translation = Vec3(0, 0.25, 0); n.primitives.push_back(GlbTreePrimitive{&colBox, -1}); nodes.push_back(n); }
    { GlbTreeNode n; n.name = "hook"; n.parent = 0; n.translation = Vec3(0.5, 0.5, 0); n.extras = {{"role", "hook"}}; nodes.push_back(n); }
    { GlbTreeNode n; n.name = "mast"; n.translation = Vec3(5, 0, 0); n.rotation = Quat::fromAxisAngle(Vec3(0, 1, 0), 90 * DEG_TO_RAD); nodes.push_back(n); }
    { GlbTreeNode n; n.name = "lamp"; n.parent = 4; n.translation = Vec3(0, 3, 1); n.primitives.push_back(GlbTreePrimitive{&lampMesh, -1}); nodes.push_back(n); }
    std::string err;
    CHECK(writeGlbTree(nodes, materials, file.path, nlohmann::json::object(), &err));

    std::vector<std::string> notes;
    const std::shared_ptr<ProcModel> m = readModelGlb(file.path, &err, &notes);
    CHECK(m != nullptr);
    if (m == nullptr) { std::printf("    read: %s\n", err.c_str()); return; }
    for (const std::string& n : notes) std::printf("    note: %s\n", n.c_str());

    CHECK(m->nodes.size() == 4);                           // root, body, mast, lamp
    const int body = m->findNode("body"), mast = m->findNode("mast"), lamp = m->findNode("lamp");
    CHECK(body > 0 && mast > 0 && lamp > 0);
    if (body < 0 || mast < 0 || lamp < 0) return;
    CHECK(m->nodes[static_cast<std::size_t>(lamp)].parent == mast);
    CHECK(near(m->nodeFrame(body).position, Vec3(0, 1, 0)));
    // the lamp is 1 m along the mast's own z, which the mast's quarter turn carries onto x
    CHECK(near(m->nodeFrame(lamp).position, Vec3(6, 3, 0)));

    // the body's scale is in its mesh: a 1 m box is 2 m, on a node with no scale of its own
    CHECK(m->parts.size() == 3);
    Vec3 lo(1e30, 1e30, 1e30), hi(-1e30, -1e30, -1e30);
    for (const Vertex& v : m->parts[0].mesh.vertices) {
        lo = Vec3(std::min(lo.x, v.position.x), std::min(lo.y, v.position.y), std::min(lo.z, v.position.z));
        hi = Vec3(std::max(hi.x, v.position.x), std::max(hi.y, v.position.y), std::max(hi.z, v.position.z));
    }
    CHECK(m->parts[0].node == body && near(hi - lo, Vec3(2, 2, 2)));
    CHECK(near(m->parts[0].mesh.vertices[0].color, Vec3(0.2, 0.6, 0.9)));
    CHECK(m->materialOf(m->parts[0]).metallic > 0.69f && m->findMaterial("steel") == m->parts[0].material);
    bool scaledNoted = false, levelNoted = false;
    for (const std::string& n : notes) {
        scaledNoted = scaledNoted || n.find("'body' is scaled") != std::string::npos;
        levelNoted = levelNoted || n.find("no distance for a level") != std::string::npos;
    }
    CHECK(scaledNoted && levelNoted);

    // `body_lod1` is the body's second level, not a node; with no distance said it starts at 40 m
    CHECK(m->findNode("body_lod1") == -1);
    CHECK(m->levelFrom.size() == 2 && m->levelFrom[1] == 40);
    int lower = 0;
    for (const ProcPart& p : m->parts)
        if (p.level == 1) { ++lower; CHECK(p.node == body); }
    CHECK(lower == 1);

    // `col_box_hull` is a box named "hull" on the body, sized by the object's mesh (and the body's scale)
    const ProcCollider* shape = m->findShape("hull");
    CHECK(shape != nullptr && shape->kind == ProcCollider::Kind::Box && shape->node == body);
    if (shape != nullptr) {
        CHECK(near(shape->halfExtent, Vec3(1, 0.5, 2)));
        CHECK(near(shape->center, Vec3(0, 0.5, 0)));
    }
    CHECK(m->findNode("col_box_hull") == -1);
    // `hook` is a marker on the body with its role; `mast`, an empty with a child, is a node
    int on = -1;
    const ProcMarker* hook = m->findMarker("hook", &on);
    CHECK(hook != nullptr && on == body);
    if (hook != nullptr) {
        CHECK(hook->role == "hook");
        CHECK(near(hook->frame.position, Vec3(1, 1, 0)));   // 0.5 in the scaled body is 1 m
    }
    CHECK(m->findMarker("mast") == nullptr);
}

TEST_CASE(levels_of_detail_as_separate_files_are_matched_to_the_first_files_nodes) {
    const Removed high("test_model_file_high_tmp.glb"), mid("test_model_file_mid_tmp.glb"), low("test_model_file_low_tmp.glb");
    auto crane = [](Real boom, bool cab, const char* extraNode) {
        ProcModel m;
        { ProcPart p; p.mesh = MeshBuilder::box(Vec3(2, 0.4, 2)); m.parts.push_back(p); }
        const int cabin = m.addNode("cabin", 0, ProcFrame{Vec3(0, 4, 0), Quat()});
        if (cab) { ProcPart p; p.mesh = MeshBuilder::box(Vec3(1, 1, 1)); p.node = cabin; m.parts.push_back(p); }
        const int arm = m.addNode("boom", cabin, ProcFrame{Vec3(0, 1, 0), Quat()});
        { ProcPart p; p.mesh = MeshBuilder::box(Vec3(0.3, 0.3, boom)); p.node = arm; m.parts.push_back(p); }
        if (extraNode != nullptr) {
            const int extra = m.addNode(extraNode, 0, ProcFrame{Vec3(9, 0, 0), Quat()});
            ProcPart p; p.mesh = MeshBuilder::box(Vec3(0.5, 0.5, 0.5)); p.node = extra; m.parts.push_back(p);
        }
        ProcMarker hook;
        hook.name = "hook";
        hook.frame.position = Vec3(0, 0, boom * 0.5);
        m.addMarker(arm, hook);
        return m;
    };
    std::string err;
    CHECK(writeModelGlb(crane(5, true, nullptr), high.path, &err));
    CHECK(writeModelGlb(crane(4.9, true, nullptr), mid.path, &err));
    CHECK(writeModelGlb(crane(4.8, false, "ballast"), low.path, &err));   // simpler, and with a node the first file lacks

    std::vector<std::string> notes;
    const std::shared_ptr<ProcModel> m = loadModelFiles({high.path, mid.path, low.path}, {0, 60, 200}, &err, &notes);
    CHECK(m != nullptr);
    if (m == nullptr) { std::printf("    load: %s\n", err.c_str()); return; }
    CHECK(m->levelFrom.size() == 3 && m->levelFrom[1] == 60 && m->levelFrom[2] == 200);
    CHECK(m->nodes.size() == 3);                           // the first file's rig, and only it
    int perLevel[3] = {0, 0, 0};
    for (const ProcPart& p : m->parts) ++perLevel[p.level];
    CHECK(perLevel[0] == 3 && perLevel[1] == 3 && perLevel[2] == 3);
    // each level's boom is on the boom
    const int boom = m->findNode("boom");
    for (int level = 0; level < 3; ++level) {
        int onBoom = 0;
        for (const ProcPart& p : m->parts) onBoom += p.level == level && p.node == boom ? 1 : 0;
        CHECK(onBoom == 1);
    }
    // the low file's `ballast` is not in the rig: its box hangs on the root where it stood, and is noted
    bool noted = false;
    for (const std::string& n : notes) noted = noted || n.find("'ballast' is not a node") != std::string::npos;
    CHECK(noted);
    bool found = false;
    for (const ProcPart& p : m->parts)
        if (p.level == 2 && p.node == 0 && p.mesh.vertices.size() == 24) {
            Vec3 c(0, 0, 0);
            for (const Vertex& v : p.mesh.vertices) c = c + v.position;
            c = c / 24.0;
            if (near(c, Vec3(9, 0, 0))) found = true;
        }
    CHECK(found);
    // one set of markers: the first file's
    CHECK(m->markerCount() == 1);
    ProcFrame hook;
    CHECK(m->markerFrame("hook", hook) && near(hook.position, Vec3(0, 5, 2.5)));

    // what stops a load
    CHECK(loadModelFiles({high.path, mid.path}, {0, 60, 200}, &err) == nullptr);
    CHECK(err.find("one entry per level") != std::string::npos);
    CHECK(loadModelFiles({high.path, mid.path}, {0, 0}, &err) == nullptr);
    CHECK(loadModelFiles({}, {}, &err) == nullptr);
    CHECK(loadModelFiles({"no_such_file_tmp.glb"}, {}, &err) == nullptr);
    // without distances: 40 m and doubling, and a note
    notes.clear();
    const std::shared_ptr<ProcModel> guessed = loadModelFiles({high.path, mid.path, low.path}, {}, &err, &notes);
    CHECK(guessed != nullptr && guessed->levelFrom.size() == 3 && guessed->levelFrom[1] == 40 && guessed->levelFrom[2] == 80);
}

TEST_CASE(a_recipe_loads_a_model_file_by_name_and_never_by_path) {
    const Removed file("test_model_file_lua_tmp.glb"), low("test_model_file_lua_low_tmp.glb");
    std::string err;
    CHECK(writeModelGlb(*makeCar(), file.path, &err));
    ProcModel simple;
    { ProcPart p; p.mesh = MeshBuilder::box(Vec3(1.5, 0.9, 4)); simple.parts.push_back(p); }
    CHECK(writeModelGlb(simple, low.path, &err));

    auto run = [&](const std::string& code, ProcModel& out, bool withFiles) {
        ScriptVM vm;
        openProcgenLibrary(vm);
        // this host keeps its model files in the working directory, under the names the test gave them
        if (withFiles) openModelFiles(vm, [](const std::string& name) { std::FILE* f = std::fopen(name.c_str(), "rb"); if (f == nullptr) return std::string(); std::fclose(f); return name; });
        err.clear();
        return runProcgenModelValue(vm, code, out, &err);
    };
    ProcModel m;
    CHECK(run(R"LUA(
        local car = model.from_file("test_model_file_lua_tmp.glb")
        assert(car:name() == "car")
        assert(car:node_count() == 5 and #car:joints() == 3 and #car:levels() == 3)
        assert(car:has_material("paint") and car:has_material("glass"))
        local at = car:marker_at("susp_front_r/valve")
        assert(at ~= nil)
        local ajar = car:posed{ door_front_l = 40 }           -- a loaded model is a model: it poses
        assert(ajar:marker_at("handle")[1] > car:marker_at("handle")[1] + 0.3)
        -- and it is used like any other: two cars in a lot
        local lot = model.new("lot")
        lot:use(car, { { pos = { 0, 0, 0 } }, { pos = { 3, 0, 0 }, turn = { 0, 180, 0 } } })
        return lot
    )LUA", m, true));
    if (!err.empty()) std::printf("    script error: %s\n", err.c_str());
    CHECK(m.name == "lot" && m.uses.size() == 1 && m.uses[0].placements.size() == 2);
    if (m.uses.size() == 1) CHECK(m.uses[0].piece->name == "car" && m.uses[0].piece->nodes.size() == 5);

    CHECK(run(R"LUA(
        return model.from_files{ name = "van", levels = { "test_model_file_lua_tmp.glb" } }
    )LUA", m, true));
    CHECK(m.name == "van" && m.levelFrom.size() == 3);

    struct Wrong { const char* code; const char* says; bool files; };
    const Wrong wrongs[] = {
        {R"LUA(return model.from_file("../secrets.glb"))LUA", "is not a model file's name", true},
        {R"LUA(return model.from_file("/etc/passwd"))LUA", "is not a model file's name", true},
        {R"LUA(return model.from_file("car.blend"))LUA", "is not a model file's name", true},
        {R"LUA(return model.from_file("no_such_model.glb"))LUA", "no model file named 'no_such_model.glb'", true},
        {R"LUA(return model.from_files{ levels = {} })LUA", "no files given", true},
        {R"LUA(return model.from_files{ distances = { 0 } })LUA", "levels =", true},
        {R"LUA(return model.from_files{ levels = { "test_model_file_lua_tmp.glb", "test_model_file_lua_low_tmp.glb" } })LUA", "already hold 3 levels", true},
        {R"LUA(return model.from_file("test_model_file_lua_tmp.glb"))LUA", "this host loads no model files", false},
    };
    for (const Wrong& w : wrongs) {
        CHECK(!run(w.code, m, w.files));
        if (err.find(w.says) == std::string::npos) std::printf("    wanted '%s', got: %s\n", w.says, err.c_str());
        CHECK(err.find(w.says) != std::string::npos);
    }
}

TEST_CASE(what_a_model_file_cannot_say_is_refused_not_dropped) {
    const Removed file("test_model_file_refused_tmp.glb");
    std::string err;
    ProcModel graded;
    graded.flatten.push_back(TerrainFlatten{});
    CHECK(!writeModelGlb(graded, file.path, &err));
    CHECK(err.find("grades terrain") != std::string::npos);

    ProcModel lamps;
    ProcUse use;
    use.piece = ProcModel::ofMesh(MeshBuilder::box(Vec3(0.2, 4, 0.2)));
    use.placements.push_back(ProcPlacement{});
    use.collision = InstanceCollision::Mesh;
    use.collisionProto = MeshBuilder::box(Vec3(0.3, 4, 0.3));
    lamps.uses.push_back(use);
    CHECK(!writeModelGlb(lamps, file.path, &err));
    CHECK(err.find("collision mesh of its own") != std::string::npos);

    // A file that says two nodes of one name is not a model.
    const RenderMesh box = MeshBuilder::box(Vec3(1, 1, 1));
    std::vector<GlbTreeNode> nodes(2);
    nodes[0].name = "door";
    nodes[0].primitives.push_back(GlbTreePrimitive{&box, -1});
    nodes[1] = nodes[0];
    CHECK(writeGlbTree(nodes, {}, file.path, nlohmann::json::object(), &err));
    CHECK(readModelGlb(file.path, &err) == nullptr);
    CHECK(err.find("two nodes named 'door'") != std::string::npos);
}
