// Named models (#191, model step 6). See model_recipes.h.
#include "model_recipes.h"

#include "../../log.h"
#include "../bundle/bundle.h"          // fnv1a, fnv1aFile
#include "../procgen/model_file.h"
#include "../script_assets.h"
#include "lua_json.h"
#include "lua_state.h"
#include "procgen_bindings.h"
#include "script_modules.h"
#include "script_vm.h"

#include <algorithm>
#include <cstdio>

namespace engine {

namespace {

uint64_t hashOfFile(const std::string& path) {
    uint64_t key = bundle::fnv1aStr("model-input");
    return bundle::fnv1aFile(path, key) ? key : 0;
}

// What is being built right now on this thread, outermost first: a model that asks for another adds it to its own
// inputs, and a model that (through others) asks for itself is caught.
struct Building {
    std::string name;
    std::vector<ModelInput>* inputs;
};
thread_local std::vector<Building> t_building;

std::shared_ptr<const ProcModel> obtainFromLine(const std::string& name, const nlohmann::json& line, const std::string& levelDir, std::string* error,
                                                ModelStore& store, const std::string& listFile);

}  // namespace

// A list script's table as JSON, read fresh each time it is asked (a list is small).
bool readScriptList(const std::string& file, const std::string& levelDir, nlohmann::json& out, std::string* error) {
    const std::string code = loadScriptCode(file, levelDir);
    if (code.empty()) {
        if (error != nullptr) *error = file + " cannot be read";
        return false;
    }
    ScriptVM vm;
    lua_State* L = luaState(vm);
    if (luaL_loadbuffer(L, code.data(), code.size(), file.c_str()) != LUA_OK || lua_pcall(L, 0, 1, 0) != LUA_OK) {
        if (error != nullptr) *error = file + ": " + (lua_tostring(L, -1) ? lua_tostring(L, -1) : "error");
        return false;
    }
    out = luaToJson(L, -1);
    if (!out.is_object()) {
        if (error != nullptr) *error = file + " must return a table of name = { ... }";
        return false;
    }
    return true;
}

std::vector<std::string> namedModels(const std::string& levelDir, std::string* error, const std::string& listFile) {
    std::vector<std::string> names;
    nlohmann::json list;
    if (!readScriptList(listFile, levelDir, list, error)) return names;
    for (nlohmann::json::const_iterator it = list.begin(); it != list.end(); ++it) names.push_back(it.key());
    std::sort(names.begin(), names.end());
    return names;
}

std::shared_ptr<const ProcModel> obtainModel(const std::string& name, const std::string& levelDir, std::string* error, ModelStore* storeOrNull,
                                             const std::string& listFile) {
    ModelStore& store = storeOrNull != nullptr ? *storeOrNull : modelStore();
    auto fail = [&](const std::string& why) -> std::shared_ptr<const ProcModel> {
        if (error != nullptr) *error = why;
        return nullptr;
    };
    for (const Building& b : t_building)
        if (b.name == name) return fail("the model '" + name + "' needs itself (through " + t_building.back().name + ")");

    nlohmann::json list;
    std::string why;
    if (!readScriptList(listFile, levelDir, list, &why)) return fail(why);
    if (!list.contains(name) || !list[name].is_object()) return fail("models.lua has no model named '" + name + "'");
    return obtainFromLine(name, list[name], levelDir, error, store, listFile);
}

namespace {

std::shared_ptr<const ProcModel> obtainFromLine(const std::string& name, const nlohmann::json& line, const std::string& levelDir, std::string* error,
                                                ModelStore& store, const std::string& listFile) {
    auto fail = [&](const std::string& why) -> std::shared_ptr<const ProcModel> {
        if (error != nullptr) *error = why;
        return nullptr;
    };
    std::string why;
    const std::string recipe = line.value("recipe", std::string());
    const bool fromFiles = line.contains("files") && line["files"].is_array();
    if (recipe.empty() == !fromFiles) return fail("models.lua: '" + name + "' needs a `recipe` or `files`, one of them");
    const std::string recipePath = fromFiles ? std::string() : resolveScriptPath(recipe + ".lua", levelDir);
    if (!fromFiles && recipePath.empty()) return fail("models.lua: '" + name + "': no recipe named '" + recipe + "'");

    // The key: the toolkit's tag, this model's line in the list, and its recipe's text.
    uint64_t key = bundle::fnv1aStr(std::string("model/") + kModelToolkitTag);
    key = bundle::fnv1aStr(line.dump(), key);
    if (!fromFiles) {
        const uint64_t text = hashOfFile(recipePath);
        key = bundle::fnv1a(&text, sizeof text, key);
    }

    // Kept, and built from what is still there? Then it is the one.
    ModelStore::Entry kept;
    bool fresh = store.entry(name, kept) && kept.key == key;
    for (std::size_t i = 0; fresh && i < kept.inputs.size(); ++i) {
        const ModelInput& in = kept.inputs[i];
        if (in.what.compare(0, 6, "model:") == 0) {
            const std::string other = in.what.substr(6);
            ModelStore::Entry theirs;
            fresh = obtainModel(other, levelDir, nullptr, &store, listFile) != nullptr && store.entry(other, theirs) && theirs.signature == in.hash;
        } else {
            fresh = hashOfFile(in.what) == in.hash;
        }
    }
    if (fresh) {
        if (!t_building.empty()) t_building.back().inputs->push_back(ModelInput{"model:" + name, kept.signature});
        return kept.model;
    }

    // Build it.
    std::vector<ModelInput> inputs;
    t_building.push_back(Building{name, &inputs});
    std::shared_ptr<ProcModel> built;
    if (fromFiles) {
        std::vector<std::string> paths;
        std::vector<Real> distances;
        const std::function<std::string(const std::string&)> find = makeModelFileSource(levelDir);
        for (const nlohmann::json& f : line["files"]) {
            const std::string path = f.is_string() ? find(f.get<std::string>()) : std::string();
            if (path.empty()) { why = "no model file named '" + (f.is_string() ? f.get<std::string>() : std::string("?")) + "'"; break; }
            paths.push_back(path);
            inputs.push_back(ModelInput{path, hashOfFile(path)});
        }
        if (line.contains("distances") && line["distances"].is_array())
            for (const nlohmann::json& d : line["distances"]) distances.push_back(d.get<double>());
        std::vector<std::string> notes;
        if (why.empty()) built = loadModelFiles(paths, distances, &why, &notes);
        for (const std::string& note : notes) LOG_WARN << "[model " << name << "] " << note;
    } else {
        const std::string code = loadScriptCode(recipe + ".lua", levelDir);
        std::vector<std::string> required;
        ScriptVM vm;
        openProcgenLibrary(vm);
        openModuleLoader(vm, makeModuleSource(levelDir, &required));
        const std::function<std::string(const std::string&)> find = makeModelFileSource(levelDir);
        openModelFiles(vm, [&](const std::string& file) {
            const std::string path = find(file);
            if (!path.empty()) inputs.push_back(ModelInput{path, hashOfFile(path)});
            return path;
        });
        openModelStore(vm, [&](const std::string& other, std::string* otherError) { return obtainModel(other, levelDir, otherError, &store, listFile); });
        if (line.contains("args") && line["args"].is_object()) setRecipeArgs(vm, line["args"].dump());
        ProcModel out;
        if (runProcgenModelValue(vm, code, out, &why)) {
            built = std::make_shared<ProcModel>(std::move(out));
            built->frozen = false;   // (a recipe that returned a kept model as it is: this is a copy of it)
        } else {
            why = recipe + ".lua: " + why;
        }
        for (const std::string& path : required) inputs.push_back(ModelInput{path, hashOfFile(path)});
    }
    t_building.pop_back();
    if (built == nullptr) return fail("the model '" + name + "': " + why);
    built->name = name;
    const std::shared_ptr<const ProcModel> model = store.put(name, key, inputs, built);
    ModelStore::Entry now;
    if (!t_building.empty() && store.entry(name, now)) t_building.back().inputs->push_back(ModelInput{"model:" + name, now.signature});
    return model;
}

}  // namespace

std::string recipeModelName(const std::string& recipe, const std::string& argsJson) {
    if (argsJson.empty() || argsJson == "null" || argsJson == "{}") return "recipe:" + recipe;
    char h[24];
    std::snprintf(h, sizeof h, "%016llx", static_cast<unsigned long long>(bundle::fnv1aStr(argsJson)));
    return "recipe:" + recipe + ":" + h;
}

std::shared_ptr<const ProcModel> obtainRecipeModel(const std::string& recipe, const std::string& argsJson, const std::string& levelDir,
                                                   std::string* error, ModelStore* storeOrNull, const std::string& listFile) {
    ModelStore& store = storeOrNull != nullptr ? *storeOrNull : modelStore();
    nlohmann::json line;
    line["recipe"] = recipe;
    if (!argsJson.empty()) {
        const nlohmann::json args = nlohmann::json::parse(argsJson, nullptr, false);
        if (args.is_object()) line["args"] = args;
    }
    const std::string name = recipeModelName(recipe, argsJson);
    for (const Building& b : t_building)
        if (b.name == name) {
            if (error != nullptr) *error = "the recipe '" + recipe + "' needs itself";
            return nullptr;
        }
    return obtainFromLine(name, line, levelDir, error, store, listFile);
}

std::vector<std::string> modelSourceFiles(const std::string& name, const std::string& levelDir, ModelStore* storeOrNull, const std::string& listFile) {
    ModelStore& store = storeOrNull != nullptr ? *storeOrNull : modelStore();
    std::vector<std::string> files;
    auto add = [&](const std::string& path) {
        if (!path.empty() && std::find(files.begin(), files.end(), path) == files.end()) files.push_back(path);
    };
    std::vector<std::string> todo{name}, seen;
    while (!todo.empty()) {
        const std::string at = todo.back();
        todo.pop_back();
        if (std::find(seen.begin(), seen.end(), at) != seen.end()) continue;
        seen.push_back(at);
        // its recipe: named by the model itself ("recipe:<file>[:args]") or by its line in the list
        if (at.compare(0, 7, "recipe:") == 0) {
            const std::string rest = at.substr(7);
            add(resolveScriptPath(rest.substr(0, rest.find(':')) + ".lua", levelDir));
        } else {
            add(resolveScriptPath(listFile, levelDir));
            nlohmann::json list;
            if (readScriptList(listFile, levelDir, list, nullptr) && list.contains(at) && list[at].is_object())
                add(resolveScriptPath(list[at].value("recipe", std::string()) + ".lua", levelDir));
        }
        ModelStore::Entry kept;
        if (!store.entry(at, kept)) continue;
        for (const ModelInput& in : kept.inputs) {
            if (in.what.compare(0, 6, "model:") == 0) todo.push_back(in.what.substr(6));
            else add(in.what);
        }
    }
    return files;
}

}  // namespace engine
