// The list of prefabs, read (#191, step 7). See prefab_list.h.
#include "prefab_list.h"

#include "../components.h"      // drawClassName
#include "model_recipes.h"      // readScriptList, namedModels

#include <algorithm>

namespace engine {

namespace {

// Is `name` a draw class a level can set a distance for?
bool knownDrawClass(const std::string& name) {
    for (int c = 0; c < static_cast<int>(DrawClass::Count); ++c) {
        const char* known = drawClassName(static_cast<DrawClass>(c));
        if (known != nullptr && name == known) return true;
    }
    return false;
}

// Every key of `table` is one of `allowed`; else false with the first that is not in `stray`.
bool onlyKeys(const nlohmann::json& table, const std::vector<std::string>& allowed, std::string& stray) {
    for (nlohmann::json::const_iterator it = table.begin(); it != table.end(); ++it) {
        if (std::find(allowed.begin(), allowed.end(), it.key()) != allowed.end()) continue;
        stray = it.key();
        return false;
    }
    return true;
}

}  // namespace

std::vector<std::string> namedPrefabs(const std::string& levelDir, std::string* error, const std::string& listFile) {
    std::vector<std::string> names;
    nlohmann::json list;
    if (!readScriptList(listFile, levelDir, list, error)) return names;
    for (nlohmann::json::const_iterator it = list.begin(); it != list.end(); ++it) names.push_back(it.key());
    std::sort(names.begin(), names.end());
    return names;
}

bool findPrefab(const std::string& name, Prefab& out, const std::string& levelDir, std::string* error, const std::string& listFile,
                const std::string& modelsFile) {
    auto fail = [&](const std::string& why) {
        if (error != nullptr) *error = why;
        return false;
    };
    nlohmann::json list;
    std::string why;
    if (!readScriptList(listFile, levelDir, list, &why)) return fail(why);
    if (!list.contains(name) || !list[name].is_object()) return fail(listFile + " has no prefab named '" + name + "'");
    const nlohmann::json& line = list[name];
    const std::string where = listFile + ": '" + name + "'";

    std::string stray;
    if (!onlyKeys(line, {"model", "physics", "draw"}, stray)) {
        // What the design note gives a prefab that is not built yet: say so, so a line is never half obeyed.
        const std::vector<std::string> later = {"sounds", "lights", "offers", "particles", "script"};
        if (std::find(later.begin(), later.end(), stray) != later.end())
            return fail(where + ": `" + stray + "` is not built yet (docs/model-record-design.md, steps 8 and 9)");
        return fail(where + " has `" + stray + "`, which a prefab does not have (model, physics, draw)");
    }

    Prefab p;
    p.name = name;
    if (!line.contains("model") || !line["model"].is_string()) return fail(where + " needs `model`, a name in " + modelsFile);
    p.model = line["model"].get<std::string>();
    const std::vector<std::string> models = namedModels(levelDir, &why, modelsFile);
    if (models.empty() && !why.empty()) return fail(why);
    if (std::find(models.begin(), models.end(), p.model) == models.end())
        return fail(where + ": " + modelsFile + " has no model named '" + p.model + "'");

    if (line.contains("physics")) {
        const nlohmann::json& phys = line["physics"];
        if (!phys.is_object()) return fail(where + ": `physics` is a table: { body = \"static\" }");
        if (!onlyKeys(phys, {"body", "friction"}, stray)) return fail(where + ": physics has `" + stray + "`, which it does not have (body, friction)");
        const std::string body = phys.value("body", std::string("static"));
        if (body == "static") p.body = Prefab::Body::Static;
        else if (body == "none") p.body = Prefab::Body::None;
        else return fail(where + ": physics.body is \"static\" or \"none\" (a body that moves is not built yet), not \"" + body + "\"");
        if (phys.contains("friction")) {
            if (!phys["friction"].is_number() || phys["friction"].get<double>() < 0) return fail(where + ": physics.friction is a number, 0 or more");
            p.friction = phys["friction"].get<double>();
        }
    }
    if (line.contains("draw")) {
        const nlohmann::json& draw = line["draw"];
        if (!draw.is_object()) return fail(where + ": `draw` is a table: { class = \"structure\" }");
        if (!onlyKeys(draw, {"class", "distance"}, stray)) return fail(where + ": draw has `" + stray + "`, which it does not have (class, distance)");
        if (draw.contains("class")) {
            if (!draw["class"].is_string() || !knownDrawClass(draw["class"].get<std::string>()))
                return fail(where + ": draw.class is a draw class's name (structure, scenery, furniture, ...)");
            p.drawClass = draw["class"].get<std::string>();
        }
        if (draw.contains("distance")) {
            if (!draw["distance"].is_number() || draw["distance"].get<double>() <= 0) return fail(where + ": draw.distance is a distance in metres, more than 0");
            p.drawDistance = draw["distance"].get<double>();
        }
    }
    out = p;
    return true;
}

}  // namespace engine
