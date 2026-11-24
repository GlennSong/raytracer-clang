// MOTION ON NODES, POSES AND RIG TABLES (#191, model step 4).
//
// A node may move: turn about an axis or slide along one, between limits (a wheel turns without end). The model says
// only what MAY move; a pose gives values, and everything under a moved node goes with it. A rig table (rigs.lua)
// lists a kind's joints so every car has a `door_front_l` that opens the same way.
//
// What is held here:
//   - the record: a move stops at its limits; a child, a marker and a used piece follow a moved node; the model with
//     no pose is exactly the model as built;
//   - a model held in a pose is a still, and holds a used piece apart only where the pose reaches into it;
//   - Lua: `n:motion`, `motion =` on a node, `m:posed` in degrees and metres, `m:joints`, and what stops a recipe;
//   - the mirrored twin moves the mirrored way;
//   - the car rig: the same value is the same action on both sides (steering, spin, doors), and motion sits above
//     the flip of a right-hand tire.
#include "test_framework.h"

#include "../src/engine/mesh_builder.h"
#include "../src/engine/procgen/proc_model.h"
#include "../src/engine/script_assets.h"
#include "../src/engine/scripting/procgen_bindings.h"
#include "../src/engine/scripting/script_modules.h"
#include "../src/engine/scripting/script_vm.h"

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

using namespace engine;

namespace {

bool near(const Vec3& a, const Vec3& b, Real eps = 1e-9) { return (a - b).length() < eps; }

ProcMotion turn(const Vec3& axis, Real minDeg, Real maxDeg) {
    ProcMotion m;
    m.kind = ProcMotion::Kind::Turn;
    m.axis = axis;
    m.limited = true;
    m.min = minDeg * DEG_TO_RAD;
    m.max = maxDeg * DEG_TO_RAD;
    return m;
}
ProcMotion spin(const Vec3& axis) {
    ProcMotion m;
    m.kind = ProcMotion::Kind::Turn;
    m.axis = axis;
    return m;
}
ProcMotion slide(const Vec3& axis, Real min, Real max) {
    ProcMotion m;
    m.kind = ProcMotion::Kind::Slide;
    m.axis = axis;
    m.limited = true;
    m.min = min;
    m.max = max;
    return m;
}

// Runs a recipe with `require` reaching assets/scripts (for rigs.lua).
bool runModel(const std::string& code, ProcModel& out, std::string* error = nullptr) {
    ScriptVM vm;
    openProcgenLibrary(vm);
#ifdef RT_SOURCE_DIR
    openModuleLoader(vm, makeModuleSource(std::string(RT_SOURCE_DIR) + "/assets/scripts"));
#else
    openModuleLoader(vm, makeModuleSource("assets/scripts"));
#endif
    return runProcgenModelValue(vm, code, out, error);
}

Vec3 sumOf(const RenderMesh& mesh, bool reflectX) {
    Vec3 s(0, 0, 0);
    for (const Vertex& v : mesh.vertices) s = s + Vec3(reflectX ? -v.position.x : v.position.x, v.position.y, v.position.z);
    return s;
}

}  // namespace

TEST_CASE(a_move_stops_at_its_limits_and_a_wheel_turns_without_end) {
    const ProcMotion door = turn(Vec3(0, 1, 0), 0, 70);
    CHECK_APPROX(door.clamped(40 * DEG_TO_RAD), 40 * DEG_TO_RAD, 1e-12);
    CHECK_APPROX(door.clamped(200 * DEG_TO_RAD), 70 * DEG_TO_RAD, 1e-12);
    CHECK_APPROX(door.clamped(-5 * DEG_TO_RAD), 0.0, 1e-12);
    CHECK(near(door.frame(90 * DEG_TO_RAD).rotation.rotate(Vec3(0, 0, 1)), Vec3(std::sin(70 * DEG_TO_RAD), 0, std::cos(70 * DEG_TO_RAD))));
    const ProcMotion wheel = spin(Vec3(1, 0, 0));
    CHECK_APPROX(wheel.clamped(1000.0), 1000.0, 1e-12);
    const ProcMotion spring = slide(Vec3(0, 1, 0), -0.1, 0.08);
    CHECK(near(spring.frame(0.05).position, Vec3(0, 0.05, 0)));
    CHECK(near(spring.frame(-1).position, Vec3(0, -0.1, 0)));
    CHECK(near(ProcMotion{}.frame(3).position, Vec3(0, 0, 0)));           // a node that does not move
}

TEST_CASE(what_hangs_under_a_moved_node_goes_with_it) {
    // A crane: the cabin turns on the mast, the boom lifts on the cabin, a hook marker at the boom's tip.
    ProcModel crane;
    const int cabin = crane.addNode("cabin", 0, ProcFrame{Vec3(0, 4, 0), Quat()});
    crane.nodes[static_cast<std::size_t>(cabin)].motion = spin(Vec3(0, 1, 0));
    const int boom = crane.addNode("boom", cabin, ProcFrame{Vec3(0, 1, 0), Quat()});
    crane.nodes[static_cast<std::size_t>(boom)].motion = turn(Vec3(-1, 0, 0), 0, 75);
    ProcMarker hook;
    hook.name = "hook";
    hook.frame.position = Vec3(0, 0, 5);
    crane.addMarker(boom, hook);

    // No pose: exactly the model as built, bit for bit.
    const ProcPose rest;
    const ProcFrame a = crane.nodeFrame(boom), b = crane.nodeFrame(boom, rest);
    CHECK(a.position.x == b.position.x && a.position.y == b.position.y && a.position.z == b.position.z);
    CHECK(near(b.point(Vec3(0, 0, 5)), Vec3(0, 5, 5)));

    ProcPose pose;
    pose.values["cabin"] = 90 * DEG_TO_RAD;       // the boom now points along +x
    pose.values["boom"] = 30 * DEG_TO_RAD;        // and lifts
    const Vec3 tip = crane.nodeFrame(boom, pose).point(Vec3(0, 0, 5));
    CHECK(near(tip, Vec3(5 * std::cos(30 * DEG_TO_RAD), 5 + 5 * std::sin(30 * DEG_TO_RAD), 0)));

    // The model held in that pose: the marker is at the tip, and the still no longer moves.
    std::string why;
    const std::shared_ptr<ProcModel> held = posedModel(crane, pose, &why);
    CHECK(held != nullptr);
    if (held != nullptr) {
        ProcFrame at;
        CHECK(held->markerFrame("hook", at) && near(at.position, tip));
        CHECK(held->jointAt("boom") == nullptr);
        CHECK(crane.jointAt("boom") != nullptr);  // the model asked is not changed
    }
    // A value past the limit is held at the limit.
    pose.values["boom"] = 200 * DEG_TO_RAD;
    CHECK(near(crane.nodeFrame(boom, pose).point(Vec3(0, 0, 5)), Vec3(5 * std::cos(75 * DEG_TO_RAD), 5 + 5 * std::sin(75 * DEG_TO_RAD), 0)));

    // A pose that names what does not move is refused.
    ProcPose wrong;
    wrong.values["root"] = 1;
    CHECK(posedModel(crane, wrong, &why) == nullptr);
    CHECK(why.find("no moving node named 'root'") != std::string::npos);
    wrong.values.clear();
    wrong.values["jib"] = 1;
    CHECK(posedModel(crane, wrong, &why) == nullptr);
}

TEST_CASE(a_pose_reaches_into_a_used_piece_and_holds_only_that_copy_apart) {
    // A door with a window that slides down; the car uses the one door at two nodes, each a hinge that turns.
    std::shared_ptr<ProcModel> door = std::make_shared<ProcModel>();
    const int window = door->addNode("window", 0, ProcFrame{Vec3(0, 1, 0), Quat()});
    door->nodes[static_cast<std::size_t>(window)].motion = slide(Vec3(0, 1, 0), -0.4, 0);
    { ProcPart p; p.mesh = MeshBuilder::box(Vec3(0.02, 0.4, 0.8)); p.node = window; door->parts.push_back(p); }
    ProcMarker top;
    top.name = "glass_top";
    top.frame.position = Vec3(0, 0.2, 0);
    door->addMarker(window, top);

    ProcModel car;
    for (const char* name : {"door_front_l", "door_rear_l"}) {
        ProcUse use;
        use.piece = door;
        use.node = car.addNode(name, 0, ProcFrame{Vec3(0.8, 0, name[5] == 'f' ? 1.0 : -1.0), Quat()});
        car.nodes[static_cast<std::size_t>(use.node)].motion = turn(Vec3(0, -1, 0), 0, 70);
        use.placements.push_back(ProcPlacement{});
        car.uses.push_back(use);
    }
    CHECK(car.jointAt("door_front_l/window") != nullptr);
    CHECK(car.jointAt("door_front_l/glass") == nullptr);
    CHECK(car.jointAt("window") == nullptr);                // the car itself has no window

    ProcPose pose;
    pose.values["door_front_l/window"] = -0.3;                    // the front window down; the rear left alone
    const std::shared_ptr<ProcModel> held = posedModel(car, pose);
    CHECK(held != nullptr);
    if (held == nullptr) return;
    CHECK(held->uses[0].piece != door);                      // held apart, posed
    CHECK(held->uses[1].piece == door);                      // still the shared door
    ProcFrame front, rear;
    CHECK(held->markerFrame("door_front_l/glass_top", front) && held->markerFrame("door_rear_l/glass_top", rear));
    CHECK(near(front.position, Vec3(0.8, 0.9, 1.0)));
    CHECK(near(rear.position, Vec3(0.8, 1.2, -1.0)));
    // The copy goes with its own hinge, too: open the front door and its window swings out with it.
    pose.values["door_front_l"] = 90 * DEG_TO_RAD;                // asked past the limit: held at 70
    const std::shared_ptr<ProcModel> open = posedModel(car, pose);
    CHECK(open != nullptr && open->markerFrame("door_front_l/glass_top", front));
    CHECK(near(front.position, Vec3(0.8, 0.9, 1.0)));        // the marker is ON the hinge line here, so it stays
    const std::vector<ProcDrawnPiece> drawn = drawnPieces(*open);
    CHECK(drawn.size() == 2);                                // two doors now differ, so two draws of one copy each
    bool swung = false;
    for (const ProcDrawnPiece& d : drawn)
        swung = swung || near(d.transforms[0].transformPoint(Vec3(0, 0, -1)), Vec3(0.8, 0.7, 1.0) + Vec3(std::sin(70 * DEG_TO_RAD), 0, -std::cos(70 * DEG_TO_RAD)));
    CHECK(swung);                                            // the door's rear edge has swung outward (+x)
}

TEST_CASE(a_recipe_says_how_a_node_moves_and_holds_a_pose) {
    ProcModel m;
    std::string err;
    CHECK(runModel(R"LUA(
        local m = model.new()
        local cabin = m:node("cabin", { pos = { 0, 4, 0 }, motion = { turn = { 0, 1, 0 } } })
        local boom = cabin:node("boom", { pos = { 0, 1, 0 } })
        assert(boom:motion{ turn = { -2, 0, 0 }, min = 0, max = 75 } == boom)        -- chainable; the axis is made unit
        boom:marker("hook", { pos = { 0, 0, 5 } })
        local line = boom:node("line", { pos = { 0, 0, 5 }, motion = { slide = { 0, -1, 0 }, min = 0, max = 6 } })
        line:marker("load")

        local joints = m:joints()
        assert(#joints == 3 and joints[1].name == "cabin" and joints[1].kind == "turn" and joints[1].min == nil)
        assert(joints[2].name == "boom" and math.abs(joints[2].max - 75) < 1e-9 and math.abs(joints[2].axis[1] + 1) < 1e-12)
        assert(joints[3].kind == "slide" and joints[3].max == 6)

        local held = m:posed{ cabin = 90, boom = 30, line = 2 }                       -- degrees, degrees, metres
        local at = held:marker_at("hook")
        assert(math.abs(at[1] - 5 * math.cos(math.rad(30))) < 1e-9 and math.abs(at[2] - (5 + 2.5)) < 1e-9 and math.abs(at[3]) < 1e-9)
        assert(#held:joints() == 0)                                                   -- a still
        assert(#m:joints() == 3 and math.abs(m:marker_at("hook")[3] - 5) < 1e-9)      -- m is not changed
        local far = m:posed{ boom = 500, line = 99 }                                  -- past the limits: held at them
        local load = far:marker_at("load")
        assert(math.abs(load[2] - (5 + 5 * math.sin(math.rad(75)) - 6 * math.cos(math.rad(75)))) < 1e-9)
        return m
    )LUA", m, &err));
    if (!err.empty()) std::printf("    script error: %s\n", err.c_str());
    const ProcNode* boom = m.jointAt("boom");
    CHECK(boom != nullptr);
    if (boom != nullptr) {
        CHECK(boom->motion.kind == ProcMotion::Kind::Turn && boom->motion.limited);
        CHECK_APPROX(boom->motion.max, 75 * DEG_TO_RAD, 1e-12);                       // radians inside the engine
        CHECK(near(boom->motion.axis, Vec3(-1, 0, 0)));
    }

    // What stops a recipe.
    struct Wrong { const char* code; const char* says; };
    const Wrong wrongs[] = {
        {R"LUA(local m = model.new() m:node("a", { motion = { turn = {0,1,0}, min = 10, max = 70 } }) return m)LUA", "must have 0 between them"},
        {R"LUA(local m = model.new() m:node("a", { motion = { slide = {0,1,0} } }) return m)LUA", "a slide needs min and max"},
        {R"LUA(local m = model.new() m:node("a", { motion = { turn = {0,1,0}, min = 0 } }) return m)LUA", "both min and max"},
        {R"LUA(local m = model.new() m:node("a", { motion = { turn = {0,1,0}, slide = {1,0,0}, min = 0, max = 1 } }) return m)LUA", "one of them"},
        {R"LUA(local m = model.new() m:node("a", { motion = { min = 0, max = 1 } }) return m)LUA", "one of them"},
        {R"LUA(local m = model.new() m:node("a", { motion = { turn = {0,0,0} } }) return m)LUA", "no length"},
        {R"LUA(local m = model.new() m:find("root"):motion{ turn = {0,1,0} } return m)LUA", "the root does not move"},
        {R"LUA(local m = model.new() m:node("a") return m:posed{ a = 10 })LUA", "no moving node named 'a'"},
        {R"LUA(local m = model.new() return m:posed{ 10 })LUA", "name = number"},
    };
    for (const Wrong& w : wrongs) {
        err.clear();
        CHECK(!runModel(w.code, m, &err));
        if (err.find(w.says) == std::string::npos) std::printf("    wanted '%s', got: %s\n", w.says, err.c_str());
        CHECK(err.find(w.says) != std::string::npos);
    }
}

TEST_CASE(the_mirrored_twin_moves_the_mirrored_way) {
    // A left door on a tilted hinge with a sliding window: every joint of its twin, at the same values, is the
    // mirror image of the door at those values.
    ProcModel door;
    const int hinge = door.addNode("hinge", 0, ProcFrame{Vec3(0.8, 0.4, 0.5), Quat::fromEuler(0.05, 0.1, -0.08)});
    door.nodes[static_cast<std::size_t>(hinge)].motion = turn(normalize(Vec3(0.1, -1, 0.05)), 0, 70);
    { ProcPart p; p.mesh = MeshBuilder::box(Vec3(0.05, 0.9, 1.0)); MeshBuilder::transform(p.mesh, Mat4::translate(0.02, 0.3, -0.5)); p.node = hinge; door.parts.push_back(p); }
    const int window = door.addNode("window", hinge, ProcFrame{Vec3(0, 0.9, -0.5), Quat::fromEuler(0.1, 0, 0)});
    door.nodes[static_cast<std::size_t>(window)].motion = slide(normalize(Vec3(0.1, 1, 0.2)), -0.4, 0);
    { ProcPart p; p.mesh = MeshBuilder::box(Vec3(0.02, 0.4, 0.8)); p.node = window; door.parts.push_back(p); }

    const std::shared_ptr<ProcModel> twin = mirroredModel(door);
    CHECK(twin != nullptr);
    if (twin == nullptr) return;
    for (const Real open : {0.0, 25.0, 70.0}) {
        ProcPose pose;
        pose.values["hinge"] = open * DEG_TO_RAD;
        pose.values["window"] = -open / 200.0;
        const std::shared_ptr<ProcModel> a = posedModel(door, pose), b = posedModel(*twin, pose);
        CHECK(a != nullptr && b != nullptr);
        if (a == nullptr || b == nullptr) continue;
        CHECK(near(sumOf(flattenModel(*a), true), sumOf(flattenModel(*b), false), 1e-9));
    }
    // And opening really moves it (so the check above is not comparing two stills).
    ProcPose open;
    open.values["hinge"] = 60 * DEG_TO_RAD;
    CHECK(!near(sumOf(flattenModel(*posedModel(door, open)), false), sumOf(flattenModel(door), false), 1e-3));
}

TEST_CASE(the_car_rig_means_the_same_action_on_both_sides) {
    ProcModel car;
    std::string err;
    CHECK(runModel(R"LUA(
        local rigs = require "rigs"
        local tire = model.new()
        tire:add(mesh.box{ 0.2, 0.6, 0.6 })
        tire:marker("face", { pos = { 0.1, 0, 0 } })        -- the outer face of the tire as built (a left tire)
        tire:marker("tread_front", { pos = { 0, 0, 0.3 } }) -- the front of the tread
        tire:marker("tread_top", { pos = { 0, 0.3, 0 } })

        local car = model.new()
        car:add(mesh.box{ 1.5, 0.5, 4 })
        local joint = rigs.apply(car, "car", {
            door_front_l = { pos = { 0.78, 0.4, 0.5 } }, door_front_r = { pos = { -0.78, 0.4, 0.5 } },
            susp_front_l = { pos = { 0.8, 0.3, 1.3 } }, susp_front_r = { pos = { -0.8, 0.3, 1.3 } },
            susp_rear_l = { pos = { 0.8, 0.3, -1.3 } }, susp_rear_r = { pos = { -0.8, 0.3, -1.3 } },
            hood = { pos = { 0, 0.85, 0.9 }, max = 55 },    -- this car's hood opens less far than the table's
        })
        assert(joint.steer_front_l and joint.spin_front_l and joint.spin_rear_l)   -- they follow their suspension
        assert(joint.steer_rear_l == nil)                                -- the rear does not steer
        assert(joint.door_rear_l == nil and joint.trunk == nil)          -- given no place: left out
        joint.spin_front_l:use(tire)
        joint.spin_rear_l:use(tire)
        -- MOTION SITS ABOVE THE FLIP: the right-hand tires are the same tire turned half a turn, under the spin.
        joint.spin_front_r:node("tire_front_r", { turn = { 0, 180, 0 } }):use(tire)
        joint.spin_rear_r:node("tire_rear_r", { turn = { 0, 180, 0 } }):use(tire)
        joint.door_front_l:marker("handle_front_l", { pos = { 0.02, 0.3, -0.9 } })   -- toward each door's rear edge
        joint.door_front_r:marker("handle_front_r", { pos = { -0.02, 0.3, -0.9 } })

        return car
    )LUA", car, &err));
    if (!err.empty()) std::printf("    script error: %s\n", err.c_str());

    // The checks, through the model the recipe returned.
    auto at = [&](const ProcModel& m, const char* path) { ProcFrame f; CHECK(m.markerFrame(path, f)); return f.position; };
    auto posed = [&](std::initializer_list<std::pair<const char*, Real>> values) {
        ProcPose pose;
        for (const std::pair<const char*, Real>& v : values) {
            const ProcNode* joint = car.jointAt(v.first);
            CHECK(joint != nullptr);
            pose.values[v.first] = joint != nullptr && joint->motion.kind == ProcMotion::Kind::Turn ? v.second * DEG_TO_RAD : v.second;
        }
        return posedModel(car, pose);
    };
    CHECK(car.jointAt("steer_front_l") != nullptr && car.jointAt("steer_rear_l") == nullptr && car.jointAt("door_rear_l") == nullptr);

    // Outer faces face out on both sides.
    CHECK(at(car, "spin_front_l/face").x > 0.8 && at(car, "tire_front_r/face").x < -0.8);

    // Steering: the front of the LEFT wheel and the front of the RIGHT wheel both move toward +x. The right tire is
    // flipped, so the front of that wheel is where its "tread_front" marker is NOT: use the wheel centre + 0.3 z.
    const std::shared_ptr<ProcModel> steered = posed({{"steer_front_l", 20}, {"steer_front_r", 20}});
    CHECK(steered != nullptr);
    if (steered != nullptr) {
        const Real leftFront = at(*steered, "spin_front_l/tread_front").x - at(car, "spin_front_l/tread_front").x;
        const Real rightRear = at(*steered, "tire_front_r/tread_front").x - at(car, "tire_front_r/tread_front").x;   // its marker sits at the rear
        CHECK(leftFront > 0.05);
        CHECK(rightRear < -0.05);          // the rear of the right wheel goes to -x, so its front goes to +x too
        CHECK_APPROX(leftFront, -rightRear, 1e-9);
    }
    // Spin: the top of every tire moves forward by the same amount, flipped or not.
    const std::shared_ptr<ProcModel> rolled = posed({{"spin_front_l", 30}, {"spin_front_r", 30}, {"spin_rear_l", 30}, {"spin_rear_r", 30}});
    CHECK(rolled != nullptr);
    if (rolled != nullptr) {
        const Real fl = at(*rolled, "spin_front_l/tread_top").z - at(car, "spin_front_l/tread_top").z;
        const Real fr = at(*rolled, "tire_front_r/tread_top").z - at(car, "tire_front_r/tread_top").z;
        const Real rr = at(*rolled, "tire_rear_r/tread_top").z - at(car, "tire_rear_r/tread_top").z;
        CHECK(fl > 0.1);
        CHECK_APPROX(fr, fl, 1e-9);
        CHECK_APPROX(rr, fl, 1e-9);
    }
    // Doors: the same value swings both handles outward by the same amount.
    const std::shared_ptr<ProcModel> open = posed({{"door_front_l", 40}, {"door_front_r", 40}});
    CHECK(open != nullptr);
    if (open != nullptr) {
        const Real l = at(*open, "handle_front_l").x - at(car, "handle_front_l").x, r = at(*open, "handle_front_r").x - at(car, "handle_front_r").x;
        CHECK(l > 0.3);
        CHECK_APPROX(r, -l, 1e-9);
    }
    // The suspension carries the wheel, and stops at its travel.
    const std::shared_ptr<ProcModel> bumped = posed({{"susp_front_l", 0.5}});
    CHECK(bumped != nullptr);
    if (bumped != nullptr) CHECK_APPROX(at(*bumped, "spin_front_l/face").y - at(car, "spin_front_l/face").y, 0.08, 1e-9);
    // The recipe's own limit for the hood replaced the table's.
    const ProcNode* hood = car.jointAt("hood");
    CHECK(hood != nullptr);
    if (hood != nullptr) CHECK_APPROX(hood->motion.max, 55 * DEG_TO_RAD, 1e-12);

    // A place for a joint the rig lacks, or a kind that is not one, stops the recipe.
    CHECK(!runModel(R"LUA(local rigs = require "rigs" local m = model.new() rigs.apply(m, "car", { sunroof = { pos = {0,1,0} } }) return m)LUA", car, &err));
    CHECK(err.find("the car rig has no joint named 'sunroof'") != std::string::npos);
    CHECK(!runModel(R"LUA(local rigs = require "rigs" local m = model.new() rigs.apply(m, "boat", {}) return m)LUA", car, &err));
    CHECK(err.find("no kind named 'boat'") != std::string::npos);
}
