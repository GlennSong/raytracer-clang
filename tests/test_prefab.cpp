// Prefabs (#191, step 7): the list, a model's draws by distance, and stamping one entity.
#include "test_framework.h"

#include "../src/engine/asset_manager.h"
#include "../src/engine/level_loader.h"
#include "../src/engine/level_writer.h"
#include "../src/engine/mesh_uploader.h"
#include "../src/engine/model_inspect.h"
#include "../src/engine/model_motion.h"
#include "../src/engine/model_spawn.h"
#include "../src/engine/model_overlay.h"
#include "../src/engine/prefab_stamp.h"
#include "../src/engine/procgen/proc_model.h"
#include "../src/engine/scripting/model_recipes.h"
#include "../src/engine/scripting/prefab_list.h"
#include "../src/engine/mesh_builder.h"

#include <nlohmann/json.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

using namespace engine;

namespace {

struct Folder {
    std::string dir = "test_prefab_tmp";
    Folder() { std::filesystem::remove_all(dir); std::filesystem::create_directory(dir); }
    ~Folder() { std::filesystem::remove_all(dir); }
    void write(const std::string& file, const std::string& text) const { std::ofstream(dir + "/" + file) << text; }
};
const char* kPrefabs = "test_prefab_list_tmp.lua";
const char* kModels = "test_prefab_models_tmp.lua";

struct StubUploader : MeshUploader {
    int uploads = 0;
    uint32_t next = 1;
    MeshHandle uploadMesh(const RenderMesh&) override { uploads++; return MeshHandle{next++, 1}; }
    void removeMesh(MeshHandle) override {}
    BoundingSphere getMeshBounds(MeshHandle) const override { return {}; }
};

RenderMesh boxMesh(Real size) {
    return MeshBuilder::box(Vec3(size, size, size));
}

ProcPart partAt(int level, Real size) {
    ProcPart p;
    p.mesh = boxMesh(size);
    p.level = level;
    return p;
}

const ProcModelDraw* drawOf(const std::vector<ProcModelDraw>& draws, const ProcPart* part) {
    for (const ProcModelDraw& d : draws)
        if (d.part == part) return &d;
    return nullptr;
}

Transform turnedAt(const Vec3& position, Real degrees) {
    Transform t;
    t.position = position;
    t.orientation = Quat::fromAxisAngle(Vec3(0, 1, 0), degrees * 3.14159265358979 / 180.0);
    return t;
}

}  // namespace

TEST_CASE(prefab_list_every_shipped_prefab_reads_and_names_a_model) {
    std::string err;
    const std::vector<std::string> names = namedPrefabs(std::string(), &err);
    CHECK(err.empty());
    CHECK(!names.empty());
    for (const std::string& name : names) {
        Prefab p;
        err.clear();
        const bool ok = findPrefab(name, p, std::string(), &err);
        if (!ok) std::printf("    prefab '%s': %s\n", name.c_str(), err.c_str());
        CHECK(ok);
        CHECK(p.name == name);
        CHECK(!p.model.empty());
    }
    Prefab crane;
    CHECK(findPrefab("crane", crane));
    CHECK(crane.model == "crane");
    CHECK(crane.body == Prefab::Body::Static);
    CHECK(crane.drawClass == "structure");
}

TEST_CASE(prefab_list_says_what_is_wrong_with_a_line) {
    Folder f;
    f.write(kModels, R"LUA(return { box = { recipe = "test_prefab_box_tmp" } })LUA");
    f.write(kPrefabs, R"LUA(return {
        plain   = { model = "box" },
        solid   = { model = "box", physics = { friction = 0.3 }, draw = { class = "scenery", distance = 250 } },
        nomodel = { physics = { body = "static" } },
        ghost   = { model = "nothing" },
        typo    = { model = "box", phisics = { body = "static" } },
        later   = { model = "box", sounds = { { at = "exhaust", patch = "idle" } } },
        noscript = { model = "box", script = "test_prefab_no_such_script_tmp.lua" },
        thrown  = { model = "box", physics = { body = "dynamic" } },
        badkey  = { model = "box", physics = { mass = 3 } },
        badcls  = { model = "box", draw = { class = "cloud" } },
        baddist = { model = "box", draw = { distance = -4 } },
    })LUA");
    auto find = [&](const std::string& name, Prefab& p, std::string& err) { return findPrefab(name, p, f.dir, &err, kPrefabs, kModels); };
    Prefab p;
    std::string err;
    CHECK(find("plain", p, err));
    CHECK(p.body == Prefab::Body::None);      // no `physics`: not solid
    CHECK(p.friction < 0);
    CHECK(p.drawClass == "furniture");
    CHECK(find("solid", p, err));
    CHECK(p.body == Prefab::Body::Static);    // `physics` given: static unless it says otherwise
    CHECK_APPROX(p.friction, 0.3, 1e-9);
    CHECK(p.drawClass == "scenery");
    CHECK_APPROX(p.drawDistance, 250.0, 1e-9);

    struct Wrong { const char* name; const char* says; };
    const Wrong wrongs[] = {
        {"absent", "has no prefab named 'absent'"},
        {"nomodel", "needs `model`"},
        {"ghost", "has no model named 'nothing'"},
        {"typo", "has `phisics`, which a prefab does not have"},
        {"later", "`sounds` is not built yet"},
        {"noscript", "cannot be read"},
        {"thrown", "a body that moves is not built yet"},
        {"badkey", "physics has `mass`"},
        {"badcls", "draw.class is a draw class's name"},
        {"baddist", "draw.distance is a distance"},
    };
    for (const Wrong& w : wrongs) {
        Prefab q;
        q.model = "untouched";
        err.clear();
        CHECK(!find(w.name, q, err));
        if (err.find(w.says) == std::string::npos) std::printf("    '%s' said: %s\n", w.name, err.c_str());
        CHECK(err.find(w.says) != std::string::npos);
        CHECK(q.model == "untouched");        // a refused line leaves the record alone
    }
    CHECK(!findPrefab("plain", p, f.dir, &err, "test_prefab_no_such_list_tmp.lua", kModels));
    CHECK(err.find("cannot be read") != std::string::npos);
}

TEST_CASE(a_prefabs_entry_is_rewritten_in_the_list_and_nothing_else_is) {
    // The editor's inspector changes a prefab by rewriting its entry in prefabs.lua: only that entry.
    Folder f;
    f.write(kModels, R"LUA(return { box = { recipe = "test_prefab_box_tmp" }, ball = { recipe = "test_prefab_box_tmp" } })LUA");
    f.write("test_prefab_box_tmp.lua", "local m = model.new() return m");
    f.write("test_prefab_act_tmp.lua", "return {}");
    const std::string typed = R"LUA(-- the list { not a table } "nor this = {"
return {
  -- before: plain = { model = "wrong" }
  first = { model = "box" },   -- kept: a note after the first
  plain = {
    model = "box",   -- a comment inside the entry goes
    physics = { body = "static", friction = 0.5 },
    script = "has } and { in it.lua",
  },   -- kept: a note after plain
  plainer = { model = "ball" },
  --[[ a long comment with plain = { model = "no" } in it ]]
}
)LUA";

    // where the entry is: its own name only, not a longer one, a comment or a string
    std::size_t begin = 0, end = 0;
    int line = 0;
    CHECK(findPrefabLine(typed, "plain", begin, end, &line));
    CHECK(line == 5);
    CHECK(typed.compare(begin, 9, "plain = {") == 0);
    CHECK(typed.compare(end - 2, 2, "},") == 0);
    CHECK(typed.substr(begin, end - begin).find("has } and { in it.lua") != std::string::npos);
    CHECK(!findPrefabLine(typed, "plai", begin, end));
    CHECK(!findPrefabLine(typed, "model", begin, end));

    // the line a prefab is written as says only what differs from an unsaid key
    Prefab p;
    p.name = "plain";
    p.model = "ball";
    CHECK(prefabLine(p) == "plain = { model = \"ball\" },");
    p.body = Prefab::Body::Static;
    p.friction = 0.7;
    p.drawClass = "structure";
    p.drawDistance = 400;
    p.script = "test_prefab_act_tmp.lua";
    CHECK(prefabLine(p) == "plain = { model = \"ball\", physics = { body = \"static\", friction = 0.7 }, draw = { class = \"structure\", distance = 400 }, "
                           "script = \"test_prefab_act_tmp.lua\" },");

    // put into the text: the entry changes, every other character stays
    std::string text = typed;
    CHECK(putPrefabLine(text, p));
    CHECK(text.find("-- kept: a note after the first") != std::string::npos);
    CHECK(text.find("  " + prefabLine(p) + "   -- kept: a note after plain") != std::string::npos);
    CHECK(text.find("a comment inside the entry goes") == std::string::npos);
    CHECK(text.substr(0, begin - 0) == typed.substr(0, begin));
    CHECK(text.substr(text.find("  plainer")) == typed.substr(typed.find("  plainer")));
    // a new one goes in as the table's last line
    Prefab added;
    added.name = "added";
    added.model = "box";
    CHECK(putPrefabLine(text, added));
    CHECK(text.find("in it ]]\n  added = { model = \"box\" },\n}\n") != std::string::npos);
    std::string none = "-- nothing here";
    std::string err;
    CHECK(!putPrefabLine(none, added, &err));
    CHECK(err.find("no table") != std::string::npos);

    // through the file: checked first, and the list reads back as what was saved
    f.write(kPrefabs, typed);
    const std::string listPath = f.dir + "/" + kPrefabs;
    auto fileText = [&]() {
        std::ifstream in(listPath);
        return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    };
    err.clear();
    CHECK(!savePrefab("plain", {{"model", "no_such"}}, f.dir, &err, kPrefabs, kModels));
    CHECK(err.find("no model named 'no_such'") != std::string::npos);
    CHECK(!savePrefab("plain", {{"model", "box"}, {"script", "test_prefab_absent_tmp.lua"}}, f.dir, &err, kPrefabs, kModels));
    CHECK(!savePrefab("2bad", {{"model", "box"}}, f.dir, &err, kPrefabs, kModels));
    CHECK(fileText() == typed);                      // a wrong line writes nothing
    err.clear();
    CHECK(savePrefab("plain", {{"model", "ball"}, {"physics", {{"body", "static"}, {"friction", 0.7}}}, {"draw", {{"class", "structure"}, {"distance", 400}}},
                               {"script", "test_prefab_act_tmp.lua"}}, f.dir, &err, kPrefabs, kModels));
    CHECK(err.empty());
    CHECK(fileText() == text.substr(0, text.find("  added")) + "}\n");
    Prefab back;
    CHECK(findPrefab("plain", back, f.dir, &err, kPrefabs, kModels));
    CHECK(back.model == "ball" && back.body == Prefab::Body::Static && back.drawClass == "structure" && back.script == "test_prefab_act_tmp.lua");
    CHECK(std::abs(back.friction - 0.7) < 1e-9 && std::abs(back.drawDistance - 400) < 1e-9);
    Prefab other;
    CHECK(findPrefab("plainer", other, f.dir, &err, kPrefabs, kModels) && other.model == "ball");
    CHECK(savePrefab("newcomer", {{"model", "box"}}, f.dir, &err, kPrefabs, kModels));
    CHECK(findPrefab("newcomer", other, f.dir, &err, kPrefabs, kModels) && other.model == "box" && other.body == Prefab::Body::None);

    // the prefab as data, for the inspector and `prefab?`
    const nlohmann::json data = nlohmann::json::parse(prefabJson("plain", f.dir, kPrefabs, kModels));
    CHECK(data.value("model", std::string()) == "ball");
    CHECK(data.value("recipe", std::string()) == "test_prefab_box_tmp");
    CHECK(data["physics"].value("body", std::string()) == "static" && std::abs(data["physics"].value("friction", 0.0) - 0.7) < 1e-9);
    CHECK(data["draw"].value("class", std::string()) == "structure" && data.value("script", std::string()) == "test_prefab_act_tmp.lua");
    CHECK(data.value("line", 0) == 5);
    CHECK(nlohmann::json::parse(prefabJson("absent", f.dir, kPrefabs, kModels)).contains("error"));
    // ...and what a line can say: every field's part is a key findPrefab knows, the model choices are the list's
    const nlohmann::json fields = nlohmann::json::parse(prefabFieldsJson(f.dir, kModels));
    CHECK(fields.is_array() && fields.size() >= 6);
    bool sawModels = false;
    for (const nlohmann::json& field : fields) {
        const std::string part = field.value("part", std::string());
        CHECK(part == "model" || part == "physics" || part == "draw" || part == "script");
        if (part == "model") sawModels = field["choices"] == nlohmann::json({"ball", "box"});
    }
    CHECK(sawModels);
}

TEST_CASE(model_draws_carry_the_band_of_distances_each_is_seen_in) {
    // A model with a lower level from 50 m, using a piece with its own lower level from 20 m.
    std::shared_ptr<ProcModel> piece = std::make_shared<ProcModel>();
    CHECK(piece->addLevel(20) == 1);
    piece->parts.push_back(partAt(0, 1.0));
    piece->parts.push_back(partAt(1, 0.9));
    ProcModel m;
    CHECK(m.addLevel(50) == 1);
    m.parts.push_back(partAt(0, 2.0));
    m.parts.push_back(partAt(1, 1.9));
    ProcPlacement where;
    where.position = Vec3(3, 0, 0);
    for (int level = 0; level < 2; ++level) {
        ProcUse use;
        use.piece = piece;
        use.level = level;
        use.placements.push_back(where);
        m.uses.push_back(use);
    }
    const std::vector<ProcModelDraw> draws = modelDraws(m);
    CHECK(draws.size() == 4);
    const ProcModelDraw* full = drawOf(draws, &m.parts[0]);
    const ProcModelDraw* low = drawOf(draws, &m.parts[1]);
    const ProcModelDraw* pieceFull = drawOf(draws, &piece->parts[0]);
    const ProcModelDraw* pieceLow = drawOf(draws, &piece->parts[1]);
    CHECK(full != nullptr && low != nullptr && pieceFull != nullptr && pieceLow != nullptr);
    if (full == nullptr || low == nullptr || pieceFull == nullptr || pieceLow == nullptr) return;
    CHECK_APPROX(full->from, 0.0, 1e-12);
    CHECK_APPROX(full->until, 50.0, 1e-12);
    CHECK_APPROX(low->from, 50.0, 1e-12);
    CHECK_APPROX(low->until, 0.0, 1e-12);            // the last level has no end
    CHECK(full->owner == &m);
    // The piece changes level at ITS distance, inside the model's full level...
    CHECK_APPROX(pieceFull->from, 0.0, 1e-12);
    CHECK_APPROX(pieceFull->until, 20.0, 1e-12);
    // ...and its lower level, standing at the same place under the model's lower level, is one draw right through.
    CHECK_APPROX(pieceLow->from, 20.0, 1e-12);
    CHECK_APPROX(pieceLow->until, 0.0, 1e-12);
    CHECK(pieceLow->owner == piece.get());
    CHECK_APPROX(pieceLow->local.m[0][3], 3.0, 1e-12);
    // Every distance is covered by exactly the draws of the level seen from there (flattenModel is the reference).
    for (Real at : {0.0, 10.0, 20.0, 35.0, 50.0, 400.0}) {
        std::size_t triangles = 0;
        for (const ProcModelDraw& d : draws)
            if (at >= d.from && (d.until == 0 || at < d.until)) triangles += d.part->mesh.indices.size() / 3;
        CHECK(triangles == flattenModel(m, at).indices.size() / 3);
    }
    // A flat model with no levels is one draw per part, from 0 with no end.
    ProcModel flat;
    flat.parts.push_back(partAt(0, 1.0));
    const std::vector<ProcModelDraw> one = modelDraws(flat);
    CHECK(one.size() == 1);
    if (one.size() == 1) { CHECK_APPROX(one[0].from, 0.0, 1e-12); CHECK_APPROX(one[0].until, 0.0, 1e-12); }
}

TEST_CASE(stamping_a_prefab_makes_an_instance_its_pieces_and_its_bodies) {
    World world;
    ModelStore store;
    Prefab crane;
    CHECK(findPrefab("crane", crane));
    const Transform at = turnedAt(Vec3(10, 0, 5), 90);
    std::string err;
    const Entity e = stampPrefab(world, nullptr, crane, at, std::string(), &err, &store);
    CHECK(e.valid());
    if (!e.valid()) { std::printf("    %s\n", err.c_str()); return; }
    const ModelInstance* mi = world.get<ModelInstance>(e);
    CHECK(mi != nullptr);
    if (mi == nullptr) return;
    CHECK(mi->prefab == "crane");
    CHECK(mi->model != nullptr && mi->model == store.find("crane"));    // the store's one crane, not a copy
    CHECK(mi->model->frozen);
    CHECK(mi->pieces.size() == modelDraws(*mi->model).size());
    CHECK(mi->pieces.size() == 9);          // seven parts of the full crane, two of the far one
    CHECK(mi->bodies.size() == 4);          // base, mast, cabin, boom: each a box

    // A piece stands where the entity's place and its own place in the model put it. The boom's mesh is in the
    // boom's space; the boom is on the cabin, 0.6 m ahead of and 1 m above the mast's top.
    bool sawBoom = false;
    for (Entity pe : mi->pieces) {
        const ModelPiece* piece = world.get<ModelPiece>(pe);
        const Transform* t = world.get<Transform>(pe);
        CHECK(piece != nullptr && t != nullptr);
        if (piece == nullptr || t == nullptr) continue;
        CHECK(piece->owner == e);
        CHECK(world.get<Renderable>(pe) == nullptr);    // no dresser: nothing to draw
        const Vec3 inModel(piece->local.m[0][3], piece->local.m[1][3], piece->local.m[2][3]);
        const Vec3 want = at.matrix().transformPoint(inModel);
        CHECK((t->position - want).length() < 1e-9);
        if ((inModel - Vec3(0, 5.4, 0.6)).length() < 1e-9) sawBoom = true;
    }
    CHECK(sawBoom);

    // The boom's collision shape is on the boom's node: its body is at the boom's middle, turned with the crane.
    const Vec3 boomMiddle = at.matrix().transformPoint(Vec3(0, 5.4, 3.1));
    bool boomBody = false;
    for (Entity be : mi->bodies) {
        const Collider* c = world.get<Collider>(be);
        const RigidBody* rb = world.get<RigidBody>(be);
        const Transform* t = world.get<Transform>(be);
        CHECK(c != nullptr && rb != nullptr && t != nullptr);
        if (c == nullptr || rb == nullptr || t == nullptr) continue;
        if ((t->position - boomMiddle).length() > 1e-9) continue;
        boomBody = true;
        CHECK(rb->motion == BodyMotion::Kinematic);    // on a node that can move: its owner carries it
        CHECK(c->shape == ColliderShape::Box);
        CHECK_APPROX(c->halfExtent.z, 2.5, 1e-9);
        // its long side, the model's z, lies along the world's x after the quarter turn
        const Vec3 along = t->matrix().transformPoint(Vec3(0, 0, 1)) - t->position;
        CHECK(std::abs(std::abs(along.x) - 1.0) < 1e-9);
    }
    CHECK(boomBody);

    // A second copy shares the model and builds nothing; taking a copy away takes its pieces and bodies with it.
    const std::size_t builds = store.builds();
    const Entity second = stampPrefab(world, nullptr, crane, turnedAt(Vec3(-30, 0, 0), 0), std::string(), &err, &store);
    CHECK(second.valid());
    CHECK(store.builds() == builds);
    mi = world.get<ModelInstance>(e);      // (a new instance may have moved the others in their pool)
    CHECK(world.get<ModelInstance>(second)->model == mi->model);
    const std::vector<Entity> pieces = mi->pieces, bodies = mi->bodies;
    unstampPrefab(world, e);
    CHECK(!world.alive(e));
    for (Entity pe : pieces) CHECK(!world.alive(pe));
    for (Entity be : bodies) CHECK(!world.alive(be));
    CHECK(world.alive(second));
    CHECK(world.alive(world.get<ModelInstance>(second)->pieces.front()));

    // A prefab that is not solid has no bodies; one with a friction gives it to every shape.
    Prefab ghost = crane;
    ghost.body = Prefab::Body::None;
    const Entity g = stampPrefab(world, nullptr, ghost, Transform(), std::string(), &err, &store);
    CHECK(world.get<ModelInstance>(g)->bodies.empty());
    Prefab slick = crane;
    slick.friction = 0.05;
    const Entity s = stampPrefab(world, nullptr, slick, Transform(), std::string(), &err, &store);
    for (Entity be : world.get<ModelInstance>(s)->bodies) CHECK_APPROX(world.get<Collider>(be)->friction, 0.05, 1e-9);

    // A prefab whose model cannot be had stamps nothing and says why.
    Prefab lost = crane;
    lost.name = "lost";
    lost.model = "no_such_model";
    err.clear();
    CHECK(!stampPrefab(world, nullptr, lost, Transform(), std::string(), &err, &store).valid());
    CHECK(err.find("prefab 'lost'") != std::string::npos && err.find("no_such_model") != std::string::npos);
}

TEST_CASE(a_stamped_prefabs_pieces_draw_by_band_and_measure_from_the_models_own_point) {
    World world;
    ModelStore store;
    StubUploader up;
    AssetManager assets(up);
    std::unique_ptr<Renderer> renderer = Renderer::create();
    CHECK(renderer != nullptr);
    if (renderer == nullptr) return;
    ModelDresser dresser(*renderer, assets);
    Prefab crane;
    CHECK(findPrefab("crane", crane));
    const Transform at = turnedAt(Vec3(40, 2, -7), 30);
    const Entity e = stampPrefab(world, &dresser, crane, at, std::string(), nullptr, &store);
    CHECK(e.valid());
    if (!e.valid()) return;
    const std::vector<Entity> pieces = world.get<ModelInstance>(e)->pieces;   // (a copy: later stamps move the pool)
    int full = 0, far = 0;
    for (Entity pe : pieces) {
        const Renderable* r = world.get<Renderable>(pe);
        const ModelPiece* piece = world.get<ModelPiece>(pe);
        CHECK(r != nullptr);
        if (r == nullptr) continue;
        CHECK(r->mesh.valid());
        CHECK(r->drawClass == DrawClass::Structure);            // the prefab's draw class
        CHECK(r->lodAnchored);                                   // every piece measures to the crane's own point
        CHECK((r->lodAnchor - at.position).length() < 1e-12);
        CHECK_APPROX(r->lodAnchorReach, (world.get<Transform>(pe)->position - at.position).length(), 1e-9);
        CHECK_APPROX(r->minDistance, piece->from, 1e-12);
        CHECK_APPROX(r->drawDistance, piece->until, 1e-12);
        if (piece->from == 0) { ++full; CHECK_APPROX(r->drawDistance, 120.0, 1e-12); }
        else { ++far; CHECK_APPROX(r->minDistance, 120.0, 1e-12); CHECK_APPROX(r->drawDistance, 0.0, 1e-12); }
        // steel is steel on the renderer's material
        if (piece->local.m[1][3] > 5.0 && piece->from == 0) CHECK_APPROX(r->material.metallic, 0.6, 1e-6);
    }
    CHECK(full == 7);
    CHECK(far == 2);

    // A second crane uploads no mesh again: the copies share them by the model's build.
    const int uploads = up.uploads;
    CHECK(uploads == 9);
    CHECK(stampPrefab(world, &dresser, crane, Transform(), std::string(), nullptr, &store).valid());
    CHECK(up.uploads == uploads);

    // `distance` on the prefab ends the last level, and cuts a nearer one short.
    Prefab near = crane;
    near.drawDistance = 80;
    const Entity n = stampPrefab(world, &dresser, near, Transform(), std::string(), nullptr, &store);
    for (Entity pe : world.get<ModelInstance>(n)->pieces) CHECK_APPROX(world.get<Renderable>(pe)->drawDistance, 80.0, 1e-12);

    // Moving the entity and placing its pieces again moves them and their point.
    Transform* t = world.get<Transform>(e);
    t->position = Vec3(0, 0, 0);
    poseModelPieces(world, e);
    for (Entity pe : pieces) CHECK(world.get<Renderable>(pe)->lodAnchor.length() < 1e-12);
}

TEST_CASE(a_model_with_used_pieces_stamps_every_copy_at_every_level) {
    // The sedan: a body in three levels, four tires that are one piece with levels of its own.
    World world;
    ModelStore store;
    Prefab sedan;
    CHECK(findPrefab("parked_sedan", sedan));
    std::string err;
    const Entity e = stampPrefab(world, nullptr, sedan, Transform(), std::string(), &err, &store);
    CHECK(e.valid());
    if (!e.valid()) { std::printf("    %s\n", err.c_str()); return; }
    const ModelInstance* mi = world.get<ModelInstance>(e);
    const ProcModel& model = *mi->model;
    CHECK(model.levelFrom.size() == 3);
    for (Real at : {0.0, 39.0, 41.0, 149.0, 151.0, 900.0}) {
        std::size_t triangles = 0;
        for (Entity pe : mi->pieces) {
            const ModelPiece* piece = world.get<ModelPiece>(pe);
            if (at >= piece->from && (piece->until == 0 || at < piece->until)) triangles += 1;
        }
        std::size_t want = 0;
        for (const ProcModelDraw& d : modelDraws(model))
            if (at >= d.from && (d.until == 0 || at < d.until)) want += 1;
        CHECK(triangles == want);
        CHECK(triangles > 0);
    }
    CHECK(!mi->bodies.empty());
}

TEST_CASE(a_level_places_prefabs_and_saves_them_back) {
    // assets/levels/prefab_lab.json: three cranes and five parked kit cars, each { "shape": "prefab", "prefab": name }.
    std::unique_ptr<Renderer> renderer = Renderer::create();
    RendererMeshUploader uploader(*renderer);
    AssetManager assets(uploader);
    World world;
    RenderView view;
    CHECK(LevelLoader::load(std::string(RT_SOURCE_DIR) + "/assets/levels/prefab_lab.json", world, *renderer, view, assets, false));
    int cranes = 0, cars = 0;
    Entity turned;
    world.each<ModelInstance, SourceSpec>([&](Entity e, ModelInstance& mi, SourceSpec& spec) {
        CHECK(spec.shape == "prefab");
        CHECK(mi.model != nullptr);
        CHECK(!mi.pieces.empty());
        CHECK(!mi.bodies.empty());
        for (Entity pe : mi.pieces) {
            const Renderable* r = world.get<Renderable>(pe);
            CHECK(r != nullptr && r->mesh.valid() && r->lodAnchored);
        }
        if (mi.model->name == "crane" || mi.prefab.rfind("crane", 0) == 0) ++cranes; else ++cars;
        if (spec.name == "crane_working") turned = e;
    });
    CHECK(cranes == 4);
    CHECK(cars == 4 + 9);     // (everything that is not a crane: the kit cars and the props)
    // crane_working is turned a quarter: its boom, on the model's z, lies along the world's x.
    CHECK(turned.valid());
    if (turned.valid()) {
        const Transform* t = world.get<Transform>(turned);
        CHECK((t->position - Vec3(0, 0, 6)).length() < 1e-9);
        const Vec3 ahead = t->matrix().transformPoint(Vec3(0, 0, 1)) - t->position;
        CHECK(std::abs(std::abs(ahead.x) - 1.0) < 1e-6);
    }
    // Every copy of the crane is the one crane.
    const ProcModel* crane = nullptr;
    world.each<ModelInstance>([&](Entity, ModelInstance& mi) {
        if (mi.prefab.rfind("crane", 0) != 0) return;
        if (crane == nullptr) crane = mi.model.get();
        CHECK(mi.model.get() == crane);
    });

    // Saved, the level names each prefab again, where it stands and how it is turned; pieces and bodies are not saved.
    const char* tmp = "test_prefab_saved_tmp.json";
    CHECK(LevelWriter::save(tmp, world));
    nlohmann::json saved;
    { std::ifstream in(tmp); in >> saved; }
    std::remove(tmp);
    int prefabs = 0;
    bool sawTurned = false;
    for (const nlohmann::json& ent : saved["entities"]) {
        if (ent.value("shape", std::string()) != "prefab") continue;
        ++prefabs;
        CHECK(ent.contains("prefab") && ent["prefab"].is_string());
        if (ent.value("name", std::string()) == "crane_working") {
            sawTurned = true;
            CHECK(ent["prefab"] == "crane_working");
            CHECK(ent.contains("orientation"));
            CHECK_APPROX(ent["position"][2].get<double>(), 6.0, 1e-9);
        }
    }
    CHECK(prefabs == 17);
    CHECK(sawTurned);
}

TEST_CASE(a_model_instances_nodes_move_toward_their_targets_and_what_hangs_on_them_follows) {
    World world;
    ModelStore store;
    Prefab crane;
    CHECK(findPrefab("crane", crane));
    const Transform at = turnedAt(Vec3(12, 0, -3), 90);
    const Entity e = stampPrefab(world, nullptr, crane, at, std::string(), nullptr, &store);
    CHECK(e.valid());
    if (!e.valid()) return;
    const Real deg = 3.14159265358979 / 180.0;

    // At rest the hook marker is under the boom's tip: 5 m out on the boom, which is 0.6 m ahead of the mast and
    // 5.4 m up; the marker hangs 0.3 m below.
    Vec3 hook;
    Quat turn;
    CHECK(modelMarkerWorld(world, e, "hook", hook, turn));
    CHECK((hook - at.matrix().transformPoint(Vec3(0, 5.1, 5.6))).length() < 1e-9);
    CHECK(!modelMarkerWorld(world, e, "no_such_marker", hook, turn));

    // A value given with no speed is there at once, stopped at the model's limit (the boom lifts to 75 degrees).
    std::string why;
    CHECK(setModelTarget(world, e, "boom", 200 * deg, 0, &why));
    CHECK_APPROX(modelValue(world, e, "boom"), 75 * deg, 1e-12);
    CHECK(setModelTarget(world, e, "boom", 0, 0));
    CHECK(!setModelTarget(world, e, "mast", 1.0, 0, &why));      // not a node at all
    CHECK(why.find("no moving node named 'mast'") != std::string::npos);
    CHECK(!setModelTarget(world, world.create(), "boom", 1.0, 0, &why));
    CHECK(why.find("not a model instance") != std::string::npos);

    // With a speed it gets there step by step: 90 degrees at 30 a second is three seconds.
    CHECK(setModelTarget(world, e, "cabin", 90 * deg, 30 * deg));
    CHECK_APPROX(modelValue(world, e, "cabin"), 0.0, 1e-12);       // not yet
    int steps = 0;
    while (advanceModelMotion(world, e, 1.0 / 60.0)) ++steps;
    CHECK(steps == 180 || steps == 181);      // (the last sixtieth lands on the target, give or take rounding)
    CHECK_APPROX(modelValue(world, e, "cabin"), 90 * deg, 1e-9);
    CHECK(world.get<ModelInstance>(e)->target.values.empty());      // met: nothing more to do
    CHECK(!advanceModelMotion(world, e, 1.0 / 60.0));

    // The cab turned a quarter about the mast: the hook, child of the boom, child of the cab, swung with it. The
    // rig turns the cab about +y, so the model's +z goes to +x.
    CHECK(modelMarkerWorld(world, e, "hook", hook, turn));
    CHECK((hook - at.matrix().transformPoint(Vec3(5.6, 5.1, 0))).length() < 1e-6);
    // ...and running the hook line out 2 m lowers it 2 m.
    CHECK(setModelTarget(world, e, "hook", 2.0, 0));
    Vec3 lower;
    CHECK(modelMarkerWorld(world, e, "hook", lower, turn));
    CHECK((lower - (hook - Vec3(0, 2, 0))).length() < 1e-6);

    // The drawn pieces are where the pose puts them: the boom's piece (full level) is at the boom's node.
    Vec3 boomNode;
    CHECK(modelNodeWorld(world, e, "boom", boomNode, turn));
    int atBoom = 0;
    for (Entity pe : world.get<ModelInstance>(e)->pieces)
        if ((world.get<Transform>(pe)->position - boomNode).length() < 1e-6) ++atBoom;
    CHECK(atBoom == 2);      // the boom's mesh at both levels of detail

    // The bodies: the base and mast do not move (static); the cab's and the boom's shapes are on moving nodes
    // (kinematic), and each knows where it belongs in the pose.
    int fixed = 0, carried = 0;
    const Vec3 boomMiddle = at.matrix().transformPoint(Vec3(3.1, 5.4, 0));
    bool boomFound = false;
    for (Entity be : world.get<ModelInstance>(e)->bodies) {
        const RigidBody* rb = world.get<RigidBody>(be);
        CHECK(world.get<ModelShape>(be) != nullptr);
        if (rb->motion == BodyMotion::Static) { ++fixed; continue; }
        CHECK(rb->motion == BodyMotion::Kinematic);
        ++carried;
        Vec3 where;
        CHECK(modelBodyPlace(world, be, where, turn));
        if ((where - boomMiddle).length() < 1e-6) boomFound = true;
    }
    CHECK(fixed == 2);
    CHECK(carried == 2);
    CHECK(boomFound);
}

TEST_CASE(a_moving_node_inside_a_used_piece_is_reached_through_the_node_that_uses_it) {
    // The sedan's doors are nodes of the car; a tire is a used piece. Opening a door moves the door's pieces and
    // nothing else.
    World world;
    ModelStore store;
    Prefab sedan;
    CHECK(findPrefab("parked_sedan", sedan));
    const Entity e = stampPrefab(world, nullptr, sedan, Transform(), std::string(), nullptr, &store);
    CHECK(e.valid());
    if (!e.valid()) return;
    std::vector<Vec3> before;
    const std::vector<Entity> pieces = world.get<ModelInstance>(e)->pieces;
    for (Entity pe : pieces) before.push_back(world.get<Transform>(pe)->position);
    std::string why;
    const bool ok = setModelTarget(world, e, "door_front_l", 50 * 3.14159265358979 / 180.0, 0, &why);
    if (!ok) std::printf("    %s\n", why.c_str());
    CHECK(ok);
    CHECK(world.get<ModelInstance>(e)->pieces.size() == pieces.size());
    int moved = 0;
    for (std::size_t i = 0; i < pieces.size(); ++i) {
        const Transform* t = world.get<Transform>(pieces[i]);
        if ((t->position - before[i]).length() > 1e-6 || std::abs(t->orientation.w - 1.0) > 1e-9) ++moved;
    }
    CHECK(moved > 0);
    CHECK(moved < static_cast<int>(pieces.size()) / 2);
}

TEST_CASE(a_level_shows_a_model_straight_from_a_recipe_and_watches_what_it_is_built_from) {
    // The editor's model studio: a level whose one prefab is a script still being written, with no line in
    // models.lua or prefabs.lua. Its recipe and every library the recipe requires are watched.
    Folder f;
    f.write("test_prefab_studio_lib_tmp.lua", "return { size = 1.5 }");
    f.write("test_prefab_studio_tmp.lua", R"LUA(
        local lib = require "test_prefab_studio_lib_tmp"
        local a = args or {}
        local m = model.new()
        m:add(mesh.box{ lib.size, a.tall or 1, lib.size })
        m:shape("body", { box = { lib.size, a.tall or 1, lib.size } })
        return m
    )LUA");
    f.write("studio.json", R"JSON({ "version": 1, "watch_scripts": true, "name": "studio", "entities": [
        { "id": 1, "name": "it", "shape": "prefab", "prefab": { "recipe": "test_prefab_studio_tmp", "args": { "tall": 3 } }, "position": [2, 0, 0] },
        { "id": 2, "name": "ghost", "shape": "prefab", "prefab": { "recipe": "test_prefab_studio_tmp", "physics": "none" } }
    ] })JSON");
    std::unique_ptr<Renderer> renderer = Renderer::create();
    RendererMeshUploader uploader(*renderer);
    AssetManager assets(uploader);
    auto load = [&](World& world) {
        RenderView view;
        return LevelLoader::load(f.dir + "/studio.json", world, *renderer, view, assets, false);
    };
    {
        World world;
        CHECK(load(world));
        int seen = 0;
        world.each<ModelInstance, SourceSpec>([&](Entity, ModelInstance& mi, SourceSpec& spec) {
            ++seen;
            CHECK(mi.model != nullptr && mi.model->parts.size() == 1);
            if (spec.name == "it") {
                CHECK(mi.bodies.size() == 1);
                CHECK_APPROX(world.get<Collider>(mi.bodies[0])->halfExtent.y, 1.5, 1e-9);   // args.tall = 3
            } else {
                CHECK(mi.bodies.empty());                                                    // physics = "none"
                CHECK_APPROX(mi.model->colliders[0].halfExtent.y, 0.5, 1e-9);               // no args: its own model
            }
        });
        CHECK(seen == 2);
        const std::vector<std::string>& watched = LevelLoader::lastLoadedScriptFiles();
        auto watching = [&](const char* file) {
            for (const std::string& w : watched)
                if (w.find(file) != std::string::npos) return true;
            return false;
        };
        CHECK(watching("test_prefab_studio_tmp.lua"));
        CHECK(watching("test_prefab_studio_lib_tmp.lua"));
        // saved, the entity names its recipe and arguments again
        const std::string saved = f.dir + "/saved.json";
        CHECK(LevelWriter::save(saved, world));
        nlohmann::json j;
        { std::ifstream in(saved); in >> j; }
        bool back = false;
        for (const nlohmann::json& ent : j["entities"])
            if (ent.value("name", std::string()) == "it") {
                back = ent["prefab"].is_object() && ent["prefab"]["recipe"] == "test_prefab_studio_tmp" && ent["prefab"]["args"]["tall"] == 3;
            }
        CHECK(back);
    }
    // The library is edited: loaded again, the model is the new size (nothing else was told).
    f.write("test_prefab_studio_lib_tmp.lua", "return { size = 4.0 }");
    {
        World world;
        CHECK(load(world));
        bool wide = false;
        world.each<ModelInstance, SourceSpec>([&](Entity, ModelInstance& mi, SourceSpec& spec) {
            if (spec.name == "it") wide = std::abs(world.get<Collider>(mi.bodies[0])->halfExtent.x - 2.0) < 1e-9;
        });
        CHECK(wide);
    }
    // The recipe is broken: the load still succeeds, the model is shown as it last built (the 4 m one), and the
    // recipe is still watched so it can be mended.
    f.write("test_prefab_studio_tmp.lua", "local m = model.new() m:add(mesh.box{ 1, 1 ) return m");
    {
        World world;
        CHECK(load(world));
        int seen = 0;
        world.each<ModelInstance, SourceSpec>([&](Entity, ModelInstance& mi, SourceSpec& spec) {
            ++seen;
            if (spec.name == "it") CHECK_APPROX(world.get<Collider>(mi.bodies[0])->halfExtent.x, 2.0, 1e-9);
        });
        CHECK(seen == 2);
        // stamped by hand, the error is still said, with a valid entity: "this is the old one"
        Prefab direct;
        direct.name = direct.recipe = "test_prefab_studio_tmp";
        std::string why;
        const Entity old = stampPrefab(world, nullptr, direct, Transform(), f.dir, &why);
        CHECK(old.valid());
        CHECK(why.find("test_prefab_studio_tmp.lua") != std::string::npos);
        // a recipe that has never built has nothing to show
        ModelStore empty;
        why.clear();
        CHECK(!stampPrefab(world, nullptr, direct, Transform(), f.dir, &why, &empty).valid());
        CHECK(!why.empty());
        bool still = false;
        for (const std::string& w : LevelLoader::lastLoadedScriptFiles())
            if (w.find("test_prefab_studio_tmp.lua") != std::string::npos) still = true;
        CHECK(still);
    }
    modelStore().forget(recipeModelName("test_prefab_studio_tmp", "{\"tall\":3}"));
    modelStore().forget(recipeModelName("test_prefab_studio_tmp"));
}

TEST_CASE(a_stamped_piece_knows_the_recipe_line_that_made_it) {
    // The model studio's "what made this?": a part records the line of its m:add; a part added inside a library
    // the recipe calls records the recipe's line that called the library; a copy of a used piece records the
    // line of the use.
    Folder f;
    f.write("test_prefab_lines_lib_tmp.lua",
            "local M = {}\n"
            "function M.post(m)\n"
            "  m:add(mesh.translate(mesh.box{ 0.2, 2, 0.2 }, { 3, 1, 0 }))\n"
            "end\n"
            "return M\n");
    f.write("test_prefab_lines_tmp.lua",
            "local lib = require \"test_prefab_lines_lib_tmp\"\n"                       // 1
            "local m = model.new()\n"                                                     // 2
            "m:material(\"paint\", material.new{ roughness = 0.5 })\n"                  // 3
            "m:add(mesh.box{ 1, 1, 1 }, \"paint\")\n"                                   // 4
            "\n"                                                                          // 5
            "lib.post(m)\n"                                                               // 6
            "local arm = m:node(\"arm\", { pos = { 0, 2, 0 } })\n"                      // 7
            "arm:add(mesh.box{ 2, 0.2, 0.2 })\n"                                          // 8
            "local bolt = model.new()\n"                                                  // 9
            "bolt:add(mesh.box{ 0.1, 0.1, 0.1 })\n"                                       // 10
            "m:use(bolt, { { pos = { 1, 0, 0 } }, { pos = { -1, 0, 0 } } })\n"            // 11
            "local function cap()\n"                                                      // 12
            "  m:add(mesh.translate(mesh.box{ 0.3, 0.1, 0.3 }, { 0, 3, 0 }))\n"           // 13
            "end\n"                                                                       // 14
            "cap()\n"                                                                     // 15
            "return m\n");
    ModelStore store;
    std::string err;
    const std::shared_ptr<const ProcModel> model = obtainRecipeModel("test_prefab_lines_tmp", std::string(), f.dir, &err, &store);
    CHECK(model != nullptr);
    if (model == nullptr) { std::printf("    %s\n", err.c_str()); return; }
    CHECK(model->parts.size() == 4);
    CHECK(model->parts[3].line == 13);       // made inside a function of the recipe's own: the line in that function
    CHECK(model->parts[0].line == 4);
    CHECK(model->parts[1].line == 6);        // made inside the library: the recipe's line that called it
    CHECK(model->parts[2].line == 8);
    CHECK(model->uses.size() == 1 && model->uses[0].line == 11);
    // a node, and (below) a marker and a collision shape, know their lines too
    CHECK(model->findNode("arm") > 0 && model->nodes[static_cast<std::size_t>(model->findNode("arm"))].line == 7);
    CHECK(model->nodes[0].line == 0);        // the root was not made by a line

    World world;
    Prefab prefab;
    prefab.name = prefab.recipe = "test_prefab_lines_tmp";
    const Entity e = stampPrefab(world, nullptr, prefab, Transform(), f.dir, &err, &store);
    CHECK(e.valid());
    if (!e.valid()) return;
    int at4 = 0, at6 = 0, at8 = 0, at11 = 0;
    for (Entity pe : world.get<ModelInstance>(e)->pieces) {
        const ModelPiece* piece = world.get<ModelPiece>(pe);
        if (piece->line == 4) { ++at4; CHECK(piece->what == "paint on root, 12 triangles"); }
        if (piece->line == 6) { ++at6; CHECK(piece->what == "plain on root, 12 triangles"); }
        if (piece->line == 8) { ++at8; CHECK(piece->what == "plain on arm, 12 triangles"); }
        if (piece->line == 11) { ++at11; CHECK(piece->what.find("a used piece") != std::string::npos); }   // not the bolt's own line 10
    }
    CHECK(at4 == 1 && at6 == 1 && at8 == 1);
    CHECK(at11 == 2);                        // both copies of the bolt
}

TEST_CASE(a_lua_table_typed_as_text_becomes_the_json_a_level_carries) {
    std::string json, err;
    CHECK(luaTableToJson("{ kind = \"tree\", seed = 3, tall = 2.5, on = true }", json, &err));
    const nlohmann::json j = nlohmann::json::parse(json);
    CHECK(j["kind"] == "tree" && j["seed"] == 3 && j["tall"] == 2.5 && j["on"] == true);
    CHECK(luaTableToJson("{ size = { 1, 2, 3 }, look = { color = { 0.5, 0.5, 0.5 } } }", json, &err));
    CHECK(nlohmann::json::parse(json)["size"][2] == 3);
    // no text is no arguments
    CHECK(luaTableToJson("", json, &err) && json.empty());
    CHECK(luaTableToJson("   ", json, &err) && json.empty());
    // what is not a table of named values is refused, and says why in a line
    CHECK(!luaTableToJson("{ kind = ", json, &err));
    CHECK(!err.empty() && err.find("[string") == std::string::npos);
    CHECK(!luaTableToJson("42", json, &err));
    CHECK(err.find("table of named values") != std::string::npos);
    CHECK(!luaTableToJson("{ 1, 2, 3 }", json, &err));
}

TEST_CASE(a_recipe_says_what_arguments_it_takes_and_wrong_ones_are_refused) {
    Folder f;
    f.write("test_prefab_params_tmp.lua", R"LUA(
        local a = params{
          kind  = { "post", "slab", default = "slab" },
          seed  = 1,
          tall  = { 2.0, min = 1, max = 5 },
          label = "plain",
          solid = true,
        }
        assert(math.type(a.seed) == "integer", "a whole argument stays whole")
        local m = model.new()
        m:add(mesh.box{ a.kind == "post" and 0.2 or 2, a.tall, 0.2 })
        if a.solid then m:shape("body", { box = { 1, a.tall, 1 } }) end
        m:marker(a.label .. "_" .. a.seed, {})
        return m
    )LUA");
    ModelStore store;
    std::string err;
    auto build = [&](const std::string& args) { return obtainRecipeModel("test_prefab_params_tmp", args, f.dir, &err, &store); };
    // no arguments: every default
    std::shared_ptr<const ProcModel> m = build("");
    CHECK(m != nullptr);
    if (m == nullptr) { std::printf("    %s\n", err.c_str()); return; }
    CHECK(m->findMarker("plain_1") != nullptr);
    CHECK(m->colliders.size() == 1);
    CHECK_APPROX(m->colliders[0].halfExtent.y, 1.0, 1e-9);
    // some given (as a level carries them: every number a fraction), the rest defaults; a number is held to its limits
    m = build(R"({"kind":"post","seed":7.0,"tall":99,"solid":false,"label":"x"})");
    CHECK(m != nullptr);
    if (m != nullptr) {
        CHECK(m->findMarker("x_7") != nullptr);
        CHECK(m->colliders.empty());
        Real top = 0;
        for (const Vertex& v : m->parts[0].mesh.vertices) top = std::max(top, v.position.y);
        CHECK_APPROX(top, 2.5, 1e-6);          // tall 99 held at 5
    }
    // what it takes, for a host's form: sorted by name, each with its kind and default
    const nlohmann::json declared = nlohmann::json::parse(recipeParams("test_prefab_params_tmp"));
    CHECK(declared.size() == 5);
    CHECK(declared[0]["name"] == "kind" && declared[0]["type"] == "choice" && declared[0]["default"] == "slab" && declared[0]["choices"].size() == 2);
    CHECK(declared[1]["name"] == "label" && declared[1]["type"] == "text");
    CHECK(declared[2]["name"] == "seed" && declared[2]["type"] == "number" && declared[2]["whole"] == true);
    CHECK(declared[3]["name"] == "solid" && declared[3]["type"] == "bool" && declared[3]["default"] == true);
    CHECK(declared[4]["name"] == "tall" && declared[4]["min"] == 1 && declared[4]["max"] == 5 && declared[4]["whole"] == false);
    // wrong arguments say what is wrong and what the recipe does take
    struct Wrong { const char* args; const char* says; };
    const Wrong wrongs[] = {
        {R"({"knd":"post"})", "there is no argument 'knd' (this recipe takes: kind, label, seed, solid, tall)"},
        {R"({"kind":"tower"})", "the argument 'kind' is one of: post, slab"},
        {R"({"seed":"many"})", "the argument 'seed' is a number"},
        {R"({"label":4})", "the argument 'label' is a text"},
        {R"({"solid":"yes"})", "the argument 'solid' is true or false"},
    };
    for (const Wrong& w : wrongs) {
        err.clear();
        CHECK(build(w.args) == nullptr);
        if (err.find(w.says) == std::string::npos) std::printf("    %s said: %s\n", w.args, err.c_str());
        CHECK(err.find(w.says) != std::string::npos);
    }
    CHECK(!recipeParams("test_prefab_params_tmp").empty());      // still known after a refusal
    CHECK(recipeParams("test_prefab_no_such_recipe_tmp").empty());
    // the shipped prop models declare theirs
    CHECK(obtainRecipeModel("prop_models", R"({"kind":"rock"})", std::string(), &err, &store) != nullptr);
    CHECK(recipeParams("prop_models").find("\"kind\"") != std::string::npos);
}

TEST_CASE(a_stamped_models_tree_is_described_as_data) {
    // model_inspect.h: what the editor's model studio draws and the control channel's `model?` hands to a tool.
    World world;
    ModelStore store;
    Prefab crane;
    CHECK(findPrefab("crane", crane));
    CHECK(modelsListing(world) == "none");
    const Entity e = stampPrefab(world, nullptr, crane, turnedAt(Vec3(3, 0, 4), 0), std::string(), nullptr, &store);
    CHECK(e.valid());
    if (!e.valid()) return;
    CHECK(modelInstances(world).size() == 1);
    CHECK(modelsListing(world).find("0 crane at 3 0 4, 9 pieces, 4 bodies") == 0);
    const nlohmann::json rows = nlohmann::json::parse(modelTreeJson(world, e));
    CHECK(rows.is_array() && rows.size() > 15);
    // the nodes, each one deeper than its parent: root, cabin under it, boom under the cabin, hook under the boom
    std::vector<std::pair<std::string, int>> nodes;
    for (const nlohmann::json& r : rows)
        if (r["kind"] == "node") nodes.emplace_back(r["name"].get<std::string>(), r["depth"].get<int>());
    CHECK(nodes.size() == 4);
    if (nodes.size() == 4) {
        CHECK(nodes[0].first == "root" && nodes[0].second == 0);
        CHECK(nodes[1].first == "cabin" && nodes[1].second == 1);
        CHECK(nodes[2].first == "boom" && nodes[2].second == 2);
        CHECK(nodes[3].first == "hook" && nodes[3].second == 3);
    }
    int parts = 0, markers = 0, shapes = 0, joints = 0;
    for (const nlohmann::json& r : rows) {
        if (r["kind"] == "part") {
            ++parts;
            CHECK(r["line"].get<int>() > 0);                                  // every part knows its line
            CHECK(r["piece"].get<int>() >= 0 && r["piece"].get<int>() < 9);   // ...and which drawn piece it is
        }
        if (r["kind"] == "marker") { ++markers; CHECK(r["line"].get<int>() > 0); }
        if (r["kind"] == "shape") { ++shapes; CHECK(r["line"].get<int>() > 0); }
        if (r.contains("joint")) {
            ++joints;
            if (r["joint"] == "boom") {
                CHECK(r["turn"] == true);
                CHECK_APPROX(r["lo"].get<double>(), 0.0, 1e-9);
                CHECK_APPROX(r["hi"].get<double>(), 75.0, 1e-6);              // degrees, as a person reads them
                CHECK_APPROX(r["value"].get<double>(), 0.0, 1e-9);
                CHECK(r["detail"].get<std::string>().find("under cabin") != std::string::npos);
            }
            if (r["joint"] == "hook") CHECK(r["turn"] == false);              // a slide, in metres
        }
    }
    CHECK(parts == 9 && markers == 4 && shapes == 4 && joints == 3);
    // a moved node says where it is now
    CHECK(setModelTarget(world, e, "boom", 30 * 3.14159265358979 / 180.0, 0));
    for (const nlohmann::json& r : nlohmann::json::parse(modelTreeJson(world, e)))
        if (r.contains("joint") && r["joint"] == "boom") CHECK_APPROX(r["value"].get<double>(), 30.0, 1e-6);
    CHECK(modelTreeJson(world, world.create()) == "[]");
}

TEST_CASE(a_stamped_models_shapes_markers_and_joints_are_drawn_as_lines_where_they_now_are) {
    World world;
    std::string err;
    const Entity crane = stampPrefab(world, nullptr, "crane", turnedAt(Vec3(10, 0, 5), 0), std::string(), &err);
    CHECK(crane.valid());
    const ModelInstance* mi = world.get<ModelInstance>(crane);
    CHECK(mi != nullptr && mi->model != nullptr);
    if (mi == nullptr || mi->model == nullptr) return;
    auto linesOf = [&](bool shapes, bool markers, bool joints) {
        DebugDraw out;
        ModelOverlay what;
        what.shapes = shapes;
        what.markers = markers;
        what.joints = joints;
        drawModelOverlay(world, crane, what, out);
        return out.lines();
    };
    CHECK(linesOf(false, false, false).empty());
    // every box shape is its twelve edges; every marker its three axes
    std::size_t boxes = 0, moving = 0;
    for (const ProcCollider& c : mi->model->colliders) {
        boxes += c.kind == ProcCollider::Kind::Box ? 1 : 0;
        moving += mi->model->nodeMoves(c.node) ? 1 : 0;
    }
    CHECK(boxes == mi->model->colliders.size() && boxes > 0 && moving > 0);
    const std::vector<DebugDraw::Line> shapes = linesOf(true, false, false);
    CHECK(shapes.size() == boxes * 12);
    CHECK(linesOf(false, true, false).size() == mi->model->markerCount() * 3);
    CHECK(!linesOf(false, false, true).empty());
    CHECK(linesOf(true, true, true).size() == shapes.size() + linesOf(false, true, false).size() + linesOf(false, false, true).size());
    // they are where the instance is (it stands at x = 10), a still shape green and one on a moving node orange
    std::size_t green = 0, orange = 0;
    for (const DebugDraw::Line& l : shapes) {
        CHECK(std::abs(l.a.x - 10) < 12 && std::abs(l.a.z - 5) < 12);
        green += l.color.y > l.color.x ? 1 : 0;
        orange += l.color.x > l.color.y ? 1 : 0;
    }
    CHECK(orange == moving * 12 && green == (boxes - moving) * 12);
    // ...and they follow the pose: lift the boom and a moving shape's lines move, the still ones do not
    CHECK(setModelTarget(world, crane, "boom", 0.6));
    const std::vector<DebugDraw::Line> lifted = linesOf(true, false, false);
    CHECK(lifted.size() == shapes.size());
    std::size_t movedGreen = 0, movedOrange = 0;
    for (std::size_t i = 0; i < shapes.size() && i < lifted.size(); ++i) {
        const bool moved = (shapes[i].a - lifted[i].a).length() > 1e-6 || (shapes[i].b - lifted[i].b).length() > 1e-6;
        if (moved) (shapes[i].color.y > shapes[i].color.x ? movedGreen : movedOrange)++;
    }
    CHECK(movedGreen == 0 && movedOrange >= 12);
}
