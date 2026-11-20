// THE MODEL STORE AND NAMED MODELS (#191, model step 6, last piece).
//
// A model the world can ask for by name is a line in a list (assets/scripts/models.lua): a recipe to run with its
// arguments, or model files to read. It is built the first time it is asked for and kept in the model store, where
// everything that shows it shares it.
//
// What is held here:
//   - the store: a model kept under a name is the one everyone gets, named and frozen;
//   - asked for twice, a model is built once;
//   - it is built again when what it was built from has changed, and only then: its line in the list, its recipe, a
//     library the recipe requires, a file it reads, another model it uses; a model that none of this touches is left;
//   - a recipe uses a kept model as a piece (`model.get`), and cannot change it; two recipes that get the tire share
//     the one tire;
//   - a model that needs itself, a name the list lacks and a line that says neither recipe nor files are refused;
//   - the shipped list: the kit sedan, by name.
#include "test_framework.h"

#include "../src/engine/mesh_builder.h"
#include "../src/engine/procgen/model_file.h"
#include "../src/engine/procgen/model_store.h"
#include "../src/engine/scripting/model_recipes.h"
#include "../src/engine/scripting/procgen_bindings.h"
#include "../src/engine/scripting/script_vm.h"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

using namespace engine;

namespace {

// A folder of the test's own: its list of models, their recipes, a library and a model file. Removed at the end.
struct Folder {
    std::string dir = "test_model_store_tmp";
    Folder() { std::filesystem::remove_all(dir); std::filesystem::create_directory(dir); }
    ~Folder() { std::filesystem::remove_all(dir); }
    void write(const std::string& file, const std::string& text) const { std::ofstream(dir + "/" + file) << text; }
};
const char* kList = "test_model_store_list_tmp.lua";

std::shared_ptr<const ProcModel> get(const Folder& f, ModelStore& store, const std::string& name, std::string* error = nullptr) {
    return obtainModel(name, f.dir, error, &store, kList);
}

void writeEverything(const Folder& f) {
    f.write(kList, R"LUA(return {
        tire  = { recipe = "test_model_store_tire_tmp", args = { radius = 0.3 } },
        car   = { recipe = "test_model_store_car_tmp" },
        van   = { recipe = "test_model_store_car_tmp", args = { long = true } },
        rock  = { recipe = "test_model_store_rock_tmp" },
        crate = { files = { "test_model_store_crate_tmp.glb" } },
    })LUA");
    f.write("test_model_store_sizes_tmp.lua", "return { width = 0.2 }");
    f.write("test_model_store_tire_tmp.lua", R"LUA(
        local sizes = require "test_model_store_sizes_tmp"
        local m = model.new()
        m:add(mesh.box{ sizes.width, args.radius * 2, args.radius * 2 })
        m:marker("valve", { pos = { 0.1, 0.2, 0 } })
        return m
    )LUA");
    f.write("test_model_store_car_tmp.lua", R"LUA(
        local tire = model.get("tire")
        local car = model.new()
        car:add(mesh.box{ 1.5, 0.5, (args and args.long) and 6 or 4 })
        car:node("wheel_front_l", { pos = { 0.8, 0.3, 1.3 } }):use(tire)
        car:node("wheel_front_r", { pos = { -0.8, 0.3, 1.3 } }):use(tire)
        return car
    )LUA");
    f.write("test_model_store_rock_tmp.lua", "local m = model.new() m:add(mesh.box{ 1, 1, 1 }) return m");
    ProcModel crate;
    { ProcPart p; p.mesh = MeshBuilder::box(Vec3(1, 1, 1)); crate.parts.push_back(p); }
    writeModelGlb(crate, f.dir + "/test_model_store_crate_tmp.glb");
}

}  // namespace

TEST_CASE(a_model_kept_under_a_name_is_the_one_everyone_gets) {
    ModelStore store;
    CHECK(store.size() == 0 && store.find("crane") == nullptr);
    std::shared_ptr<ProcModel> crane = std::make_shared<ProcModel>();
    crane->parts.push_back(ProcPart{});
    const std::shared_ptr<const ProcModel> kept = store.put("crane", 42, {ModelInput{"crane.lua", 7}}, crane);
    CHECK(kept != nullptr && kept.get() == crane.get());
    CHECK(kept->name == "crane");                          // named by what it is kept under
    CHECK(kept->frozen);                                   // shared, so not to be changed
    CHECK(store.find("crane") == kept && store.size() == 1 && store.builds() == 1);
    ModelStore::Entry e;
    CHECK(store.entry("crane", e) && e.key == 42 && e.inputs.size() == 1 && e.inputs[0].what == "crane.lua" && e.inputs[0].hash == 7);
    CHECK(!store.entry("bench", e));
    store.put("bench", 1, {}, std::make_shared<ProcModel>());
    CHECK(store.names() == (std::vector<std::string>{"bench", "crane"}));
    // put again: the new one takes its place; whoever holds the old one still has it
    const std::shared_ptr<const ProcModel> newer = store.put("crane", 43, {}, std::make_shared<ProcModel>());
    CHECK(store.find("crane") == newer && newer != kept && kept->parts.size() == 1);
    store.forget("crane");
    CHECK(store.find("crane") == nullptr && store.size() == 1);
    store.clear();
    CHECK(store.size() == 0);
    CHECK(store.put("nothing", 1, {}, nullptr) == nullptr);
}

TEST_CASE(a_named_model_is_built_once_and_shared) {
    const Folder f;
    writeEverything(f);
    ModelStore store;
    std::string err;
    const std::shared_ptr<const ProcModel> car = get(f, store, "car", &err);
    CHECK(car != nullptr);
    if (car == nullptr) { std::printf("    %s\n", err.c_str()); return; }
    CHECK(car->name == "car" && car->frozen);
    CHECK(store.builds() == 2);                            // the car, and the tire it asked for
    CHECK(store.names() == (std::vector<std::string>{"car", "tire"}));

    // asked again: the same one, nothing built
    CHECK(get(f, store, "car") == car);
    CHECK(get(f, store, "tire") == store.find("tire"));
    CHECK(store.builds() == 2);

    // the car's wheels are the store's tire, and so are the van's: one tire in memory
    const std::shared_ptr<const ProcModel> tire = store.find("tire");
    CHECK(car->uses.size() == 2 && car->uses[0].piece == tire && car->uses[1].piece == tire);
    const std::shared_ptr<const ProcModel> van = get(f, store, "van", &err);
    CHECK(van != nullptr && van != car);
    if (van != nullptr) CHECK(van->uses.size() == 2 && van->uses[0].piece == tire);
    CHECK(store.builds() == 3);                            // only the van was built
    ProcFrame valve;
    CHECK(car->markerFrame("wheel_front_r/valve", valve));  // what is inside the kept tire, through the car

    // the car knows what it was built from: the tire, as a model
    ModelStore::Entry e;
    CHECK(store.entry("car", e));
    bool usesTire = false;
    for (const ModelInput& in : e.inputs) usesTire = usesTire || in.what == "model:tire";
    CHECK(usesTire);
    CHECK(store.entry("tire", e));
    bool readsSizes = false;
    for (const ModelInput& in : e.inputs) readsSizes = readsSizes || in.what.find("test_model_store_sizes_tmp.lua") != std::string::npos;
    CHECK(readsSizes);                                     // and the tire the library it required

    // a model read from files
    const std::shared_ptr<const ProcModel> crate = get(f, store, "crate", &err);
    CHECK(crate != nullptr && crate->parts.size() == 1 && crate->name == "crate");
    CHECK(get(f, store, "crate") == crate);
}

TEST_CASE(a_model_is_built_again_when_what_it_was_built_from_changes_and_only_then) {
    const Folder f;
    writeEverything(f);
    ModelStore store;
    const std::shared_ptr<const ProcModel> car = get(f, store, "car"), van = get(f, store, "van"), rock = get(f, store, "rock"),
                                           crate = get(f, store, "crate"), tire = store.find("tire");
    CHECK(car != nullptr && van != nullptr && rock != nullptr && crate != nullptr && tire != nullptr);
    if (car == nullptr || van == nullptr || rock == nullptr || crate == nullptr || tire == nullptr) return;
    const std::size_t built = store.builds();
    CHECK(built == 5);

    // nothing changed: nothing built
    CHECK(get(f, store, "car") == car && get(f, store, "rock") == rock);
    CHECK(store.builds() == built);

    // 1. the rock's recipe is edited: the rock is built again; the cars are not
    f.write("test_model_store_rock_tmp.lua", "local m = model.new() m:add(mesh.box{ 2, 2, 2 }) m:add(mesh.box{ 1, 1, 1 }) return m");
    const std::shared_ptr<const ProcModel> rock2 = get(f, store, "rock");
    CHECK(rock2 != nullptr && rock2 != rock && rock2->parts.size() == 2);
    CHECK(rock->parts.size() == 1);                        // whoever held the old rock still has it, whole
    CHECK(get(f, store, "car") == car && get(f, store, "van") == van);
    CHECK(store.builds() == built + 1);

    // 2. a library the TIRE requires is edited: the tire is built again, and with it everything that uses the tire
    f.write("test_model_store_sizes_tmp.lua", "return { width = 0.35 }");
    const std::shared_ptr<const ProcModel> car2 = get(f, store, "car");
    const std::shared_ptr<const ProcModel> tire2 = store.find("tire");
    CHECK(car2 != nullptr && car2 != car && tire2 != tire);
    if (car2 != nullptr) CHECK(car2->uses[0].piece == tire2);
    CHECK(get(f, store, "rock") == rock2);                 // the rock has nothing to do with it
    CHECK(store.builds() == built + 3);                    // the tire and the car
    const std::shared_ptr<const ProcModel> van2 = get(f, store, "van");
    CHECK(van2 != van && van2 != nullptr && van2->uses[0].piece == tire2);   // the van too, when it is next asked for
    CHECK(store.builds() == built + 4);

    // 3. the van's line in the list changes (its arguments): the van is built again; the car is not
    f.write(kList, R"LUA(return {
        tire  = { recipe = "test_model_store_tire_tmp", args = { radius = 0.3 } },
        car   = { recipe = "test_model_store_car_tmp" },
        van   = { recipe = "test_model_store_car_tmp", args = { long = false } },
        rock  = { recipe = "test_model_store_rock_tmp" },
        crate = { files = { "test_model_store_crate_tmp.glb" } },
    })LUA");
    const std::shared_ptr<const ProcModel> van3 = get(f, store, "van");
    CHECK(van3 != nullptr && van3 != van2);
    CHECK(get(f, store, "car") == car2 && store.find("tire") == tire2);
    CHECK(store.builds() == built + 5);

    // 4. the crate's file is written again with another model in it: the crate is read again
    ProcModel bigger;
    { ProcPart p; p.mesh = MeshBuilder::box(Vec3(2, 2, 2)); bigger.parts.push_back(p); }
    { ProcPart p; p.mesh = MeshBuilder::box(Vec3(1, 1, 1)); p.node = bigger.addNode("lid", 0, ProcFrame{Vec3(0, 1, 0), Quat()}); bigger.parts.push_back(p); }
    CHECK(writeModelGlb(bigger, f.dir + "/test_model_store_crate_tmp.glb"));
    const std::shared_ptr<const ProcModel> crate2 = get(f, store, "crate");
    CHECK(crate2 != nullptr && crate2 != crate && crate2->findNode("lid") > 0);
    CHECK(store.builds() == built + 6);
    CHECK(get(f, store, "crate") == crate2 && store.builds() == built + 6);
}

TEST_CASE(a_recipe_uses_a_kept_model_as_a_piece_and_cannot_change_it) {
    const Folder f;
    writeEverything(f);
    ModelStore store;
    auto run = [&](const std::string& code, ProcModel& out, std::string& err, bool withStore) {
        ScriptVM vm;
        openProcgenLibrary(vm);
        if (withStore) openModelStore(vm, [&](const std::string& name, std::string* why) { return get(f, store, name, why); });
        err.clear();
        return runProcgenModelValue(vm, code, out, &err);
    };
    ProcModel m;
    std::string err;
    CHECK(run(R"LUA(
        local tire = model.get("tire")
        assert(tire:name() == "tire" and tire:part_count() == 1)
        assert(tire:marker_at("valve") ~= nil)              -- asking a kept model is fine
        local flat = tire:flatten()                         -- and so is making something new from it
        local turned = tire:mirrored()
        turned:add(mesh.box{ 0.1, 0.1, 0.1 })                -- a model made FROM a kept one is the recipe's own
        local axle = model.new("axle")
        axle:use(tire, { { pos = { 0.8, 0, 0 } }, { pos = { -0.8, 0, 0 }, turn = { 0, 180, 0 } } })
        return axle
    )LUA", m, err, true));
    if (!err.empty()) std::printf("    script error: %s\n", err.c_str());
    CHECK(m.uses.size() == 1 && m.uses[0].piece == store.find("tire"));
    CHECK(store.find("tire") != nullptr && store.find("tire")->parts.size() == 1);   // untouched

    struct Wrong { const char* code; const char* says; bool withStore; };
    const Wrong wrongs[] = {
        {R"LUA(local t = model.get("tire") t:add(mesh.box{1,1,1}) return t)LUA", "kept in the model store", true},
        {R"LUA(local t = model.get("tire") t:node("hub") return t)LUA", "kept in the model store", true},
        {R"LUA(local t = model.get("tire") t:marker("cap") return t)LUA", "kept in the model store", true},
        {R"LUA(local t = model.get("tire") t:shape("s", { sphere = 1 }) return t)LUA", "kept in the model store", true},
        {R"LUA(local t = model.get("tire") t:material("x", material.new{}) return t)LUA", "kept in the model store", true},
        {R"LUA(local t = model.get("tire") t:level(40) return t)LUA", "kept in the model store", true},
        {R"LUA(local t = model.get("tire") t:use(mesh.box{1,1,1}) return t)LUA", "kept in the model store", true},
        {R"LUA(local t = model.get("tire") t:merge(model.new()) return t)LUA", "kept in the model store", true},
        {R"LUA(local t = model.get("tire") t:find("root"):add(mesh.box{1,1,1}) return t)LUA", "kept in the model store", true},
        {R"LUA(return model.get("anvil"))LUA", "no model named 'anvil'", true},
        {R"LUA(return model.get("tire"))LUA", "this host keeps no named models", false},
    };
    for (const Wrong& w : wrongs) {
        CHECK(!run(w.code, m, err, w.withStore));
        if (err.find(w.says) == std::string::npos) std::printf("    wanted '%s', got: %s\n", w.says, err.c_str());
        CHECK(err.find(w.says) != std::string::npos);
    }
    CHECK(store.find("tire") != nullptr && store.find("tire")->parts.size() == 1 && store.find("tire")->nodes.size() == 1);
}

TEST_CASE(a_model_that_needs_itself_or_is_not_in_the_list_is_refused) {
    const Folder f;
    f.write(kList, R"LUA(return {
        chicken = { recipe = "test_model_store_chicken_tmp" },
        egg     = { recipe = "test_model_store_egg_tmp" },
        broken  = { recipe = "test_model_store_broken_tmp" },
        lost    = { recipe = "test_model_store_nowhere_tmp" },
        empty   = { },
        both    = { recipe = "test_model_store_egg_tmp", files = { "x.glb" } },
        gone    = { files = { "test_model_store_no_such_tmp.glb" } },
    })LUA");
    f.write("test_model_store_chicken_tmp.lua", "local m = model.new() m:use(model.get('egg')) return m");
    f.write("test_model_store_egg_tmp.lua", "local m = model.new() m:use(model.get('chicken')) return m");
    f.write("test_model_store_broken_tmp.lua", "local m = model.new() m:add(nil) return m");
    ModelStore store;
    std::string err;
    CHECK(get(f, store, "chicken", &err) == nullptr);
    CHECK(err.find("needs itself") != std::string::npos);
    CHECK(get(f, store, "broken", &err) == nullptr);
    CHECK(err.find("the model 'broken'") != std::string::npos && err.find("test_model_store_broken_tmp.lua") != std::string::npos);
    CHECK(get(f, store, "lost", &err) == nullptr && err.find("no recipe named") != std::string::npos);
    CHECK(get(f, store, "empty", &err) == nullptr && err.find("needs a `recipe` or `files`") != std::string::npos);
    CHECK(get(f, store, "both", &err) == nullptr && err.find("one of them") != std::string::npos);
    CHECK(get(f, store, "gone", &err) == nullptr && err.find("no model file named") != std::string::npos);
    CHECK(get(f, store, "unicorn", &err) == nullptr && err.find("no model named 'unicorn'") != std::string::npos);
    CHECK(store.size() == 0);                              // nothing half built was kept
    CHECK(obtainModel("anything", f.dir, &err, &store, "test_model_store_no_list_tmp.lua") == nullptr);
    CHECK(err.find("cannot be read") != std::string::npos);
    CHECK(namedModels(f.dir, &err, kList).size() == 7);
}

TEST_CASE(the_shipped_list_names_the_kit_sedan) {
    // assets/scripts/models.lua, through the same store a game would use (one of its own here, so the test does not
    // depend on what ran before it).
    std::string err;
#ifdef RT_SOURCE_DIR
    const std::string scripts = std::string(RT_SOURCE_DIR) + "/assets/scripts";
#else
    const std::string scripts = "assets/scripts";
#endif
    const std::vector<std::string> names = namedModels(scripts, &err);
    bool hasSedan = false;
    for (const std::string& n : names) hasSedan = hasSedan || n == "sedan";
    CHECK(hasSedan);
    ModelStore store;
    const std::shared_ptr<const ProcModel> sedan = obtainModel("sedan", scripts, &err, &store);
    CHECK(sedan != nullptr);
    if (sedan == nullptr) { std::printf("    %s\n", err.c_str()); return; }
    CHECK(sedan->name == "sedan" && sedan->frozen);
    CHECK(sedan->jointAt("door_front_l") != nullptr && sedan->jointAt("spin_rear_r") != nullptr);
    CHECK(sedan->levelFrom.size() == 3 && sedan->findShape("body") != nullptr);
    ProcFrame lamp;
    CHECK(sedan->markerFrame("headlight_l", lamp));
    const std::size_t built = store.builds();
    CHECK(obtainModel("sedan", scripts, &err, &store) == sedan && store.builds() == built);
    // it was built from the kit and the rig: both are among its inputs, so an edit to either builds it again
    ModelStore::Entry e;
    CHECK(store.entry("sedan", e));
    bool kit = false, rig = false;
    for (const ModelInput& in : e.inputs) {
        kit = kit || in.what.find("vehicle_kit.lua") != std::string::npos;
        rig = rig || in.what.find("rigs.lua") != std::string::npos;
    }
    CHECK(kit && rig);
}
