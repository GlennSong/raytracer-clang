// The list of prefabs, read (#191, step 7). See prefab_list.h.
#include "prefab_list.h"

#include "../components.h"      // drawClassName
#include "../script_assets.h"   // loadScriptCode
#include "model_recipes.h"      // readScriptList, namedModels

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <iterator>

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
    nlohmann::json list;
    std::string why;
    if (!readScriptList(listFile, levelDir, list, &why) || !list.contains(name) || !list[name].is_object()) {
        if (error != nullptr) *error = why.empty() ? listFile + " has no prefab named '" + name + "'" : why;
        return false;
    }
    return readPrefabLine(name, list[name], out, levelDir, error, listFile, modelsFile);
}

bool readPrefabLine(const std::string& name, const nlohmann::json& line, Prefab& out, const std::string& levelDir, std::string* error,
                    const std::string& listFile, const std::string& modelsFile) {
    auto fail = [&](const std::string& why) {
        if (error != nullptr) *error = why;
        return false;
    };
    std::string why;
    const std::string where = listFile + ": '" + name + "'";
    if (!line.is_object()) return fail(where + " is a table: { model = \"...\" }");

    std::string stray;
    if (!onlyKeys(line, {"model", "physics", "draw", "script", "uses"}, stray)) {
        // What the design note gives a prefab that is not built yet: say so, so a line is never half obeyed.
        const std::vector<std::string> later = {"sounds", "lights", "offers", "particles"};
        if (std::find(later.begin(), later.end(), stray) != later.end())
            return fail(where + ": `" + stray + "` is not built yet (docs/model-record-design.md, step 9)");
        return fail(where + " has `" + stray + "`, which a prefab does not have (model, physics, draw, script, uses)");
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
    if (line.contains("script")) {
        if (!line["script"].is_string() || line["script"].get<std::string>().empty()) return fail(where + ": `script` is a script's file name");
        p.script = line["script"].get<std::string>();
        if (loadScriptCode(p.script, levelDir).empty()) return fail(where + ": the script '" + p.script + "' cannot be read");
    }
    if (line.contains("uses")) {
        const nlohmann::json& uses = line["uses"];
        // (an empty Lua table reads as an empty object)
        if (!uses.is_array() && !(uses.is_object() && uses.empty())) return fail(where + ": `uses` is a list: { { at = \"handle\", say = \"open\", ... }, ... }");
        std::size_t number = 0;
        for (const nlohmann::json& u : uses) {
            const std::string which = where + ": use " + std::to_string(++number);
            if (!u.is_object()) return fail(which + " is a table: { at = \"handle\", say = \"open\", ... }");
            if (!onlyKeys(u, {"at", "say", "back", "move", "speed", "run", "seconds", "stop", "coast", "sway", "period", "light", "light_at", "sit", "lie", "id", "needs", "unless"}, stray))
                return fail(which + " has `" + stray + "`, which a use does not have (at, say, back, move, speed, run, seconds, stop, coast, sway, period, light, light_at, sit, lie, id, needs, unless)");
            PrefabUse use;
            if (!u.contains("at") || !u["at"].is_string() || u["at"].get<std::string>().empty()) return fail(which + " needs `at`, a marker on the model");
            if (!u.contains("say") || !u["say"].is_string() || u["say"].get<std::string>().empty()) return fail(which + " needs `say`, the word offered (\"open\")");
            use.at = u["at"].get<std::string>();
            use.say = u["say"].get<std::string>();
            if (u.contains("back")) {
                if (!u["back"].is_string()) return fail(which + ": `back` is the word offered once it is done (\"close\")");
                use.back = u["back"].get<std::string>();
            }
            for (const char* key : {"move", "run", "sway"}) {
                if (!u.contains(key)) continue;
                if (!u[key].is_object()) return fail(which + ": `" + key + "` is a table of nodes and numbers: { door = 105 }");
                const std::string k = key;
                for (nlohmann::json::const_iterator it = u[key].begin(); it != u[key].end(); ++it) {
                    if (!it.value().is_number()) return fail(which + ": " + key + "." + it.key() + " is a number");
                    (k == "move" ? use.move : k == "run" ? use.run : use.sway).emplace_back(it.key(), it.value().get<double>());
                }
            }
            for (const char* key : {"speed", "seconds", "coast", "period"}) {
                if (!u.contains(key)) continue;
                if (!u[key].is_number() || u[key].get<double>() < 0) return fail(which + ": `" + key + "` is a number, 0 or more");
                const std::string k = key;
                (k == "speed" ? use.speed : k == "seconds" ? use.seconds : k == "coast" ? use.coast : use.period) = u[key].get<double>();
            }
            if (!use.sway.empty() && use.period <= 0) return fail(which + ": `period` is the seconds one sway takes, more than 0");
            if (u.contains("sit")) {
                if (!u["sit"].is_boolean()) return fail(which + ": `sit` is true or false");
                use.sit = u["sit"].get<bool>();
            }
            for (const char* key : {"id", "needs", "unless", "light", "light_at"}) {
                if (!u.contains(key)) continue;
                if (!u[key].is_string() || u[key].get<std::string>().empty()) return fail(which + ": `" + key + "` is a name");
                const std::string k = key;
                (k == "id" ? use.id : k == "needs" ? use.needs : k == "unless" ? use.unless : k == "light" ? use.light : use.lightAt) = u[key].get<std::string>();
            }
            if (!use.lightAt.empty() && use.light.empty()) return fail(which + ": `light_at` needs `light`, a kind of light");
            if (u.contains("stop")) {
                if (!u["stop"].is_boolean()) return fail(which + ": `stop` is true or false");
                use.stop = u["stop"].get<bool>();
            }
            if (u.contains("lie")) {
                if (!u["lie"].is_boolean()) return fail(which + ": `lie` is true or false");
                use.lie = u["lie"].get<bool>();
            }
            if (use.move.empty() && use.run.empty() && use.sway.empty() && use.light.empty() && !use.sit && !use.lie) return fail(which + " does nothing: give it `move`, `run`, `sway`, `light`, `sit` or `lie`");
            p.uses.push_back(use);
        }
        // a use that needs, or gives way to, another names one this line has
        // (a name with a `*` in it is matched when the copy is stamped, each `*` against the same text)
        auto starless = [](std::string s) { s.erase(std::remove(s.begin(), s.end(), '*'), s.end()); return s; };
        for (const PrefabUse& use : p.uses)
            for (const std::string* other : {&use.needs, &use.unless}) {
                if (other->empty()) continue;
                bool found = false;
                for (const PrefabUse& candidate : p.uses) found = found || starless(candidate.id) == starless(*other);
                if (!found) return fail(where + ": the use '" + use.say + "' names '" + *other + "', which no use of this prefab has as its `id`");
            }
    }
    out = p;
    return true;
}

namespace {

// A number as a list's line writes it: 0.7, 400 (no trailing zeros).
std::string plainNumber(double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.6g", v);
    return buf;
}

std::string luaQuoted(const std::string& text) {
    std::string out = "\"";
    for (char c : text) {
        if (c == '\\' || c == '"') out += '\\';
        out += c;
    }
    return out + "\"";
}

// A walk over Lua text that knows what is code: `visit(i)` is called at each character that is not inside a comment
// or a string, and may return a new place to go on from (or i, to step on by one).
template <typename Visit>
void walkLuaCode(const std::string& text, Visit visit) {
    // the end of a long bracket opened at `i` ("[[" or "[==["), or npos when `i` does not open one
    auto longEnd = [&](std::size_t i) -> std::size_t {
        if (i >= text.size() || text[i] != '[') return std::string::npos;
        std::size_t j = i + 1;
        while (j < text.size() && text[j] == '=') ++j;
        if (j >= text.size() || text[j] != '[') return std::string::npos;
        const std::string close = "]" + std::string(j - i - 1, '=') + "]";
        const std::size_t at = text.find(close, j + 1);
        return at == std::string::npos ? text.size() : at + close.size();
    };
    std::size_t i = 0;
    while (i < text.size()) {
        const char c = text[i];
        if (c == '-' && i + 1 < text.size() && text[i + 1] == '-') {
            const std::size_t block = longEnd(i + 2);
            if (block != std::string::npos) { i = block; continue; }
            const std::size_t eol = text.find('\n', i);
            i = eol == std::string::npos ? text.size() : eol;
            continue;
        }
        if (c == '"' || c == '\'') {
            std::size_t j = i + 1;
            while (j < text.size() && text[j] != c && text[j] != '\n') j += text[j] == '\\' ? 2 : 1;
            i = std::min(text.size(), j + 1);
            continue;
        }
        if (c == '[') {
            const std::size_t block = longEnd(i);
            if (block != std::string::npos) { i = block; continue; }
        }
        const std::size_t next = visit(i);
        i = next > i ? next : i + 1;
    }
}

bool nameChar(char c) { return std::isalnum(static_cast<unsigned char>(c)) != 0 || c == '_'; }

}  // namespace

std::string prefabLine(const Prefab& p) {
    std::string out = p.name + " = { model = " + luaQuoted(p.model);
    if (p.body == Prefab::Body::Static) {
        out += ", physics = { body = \"static\"";
        if (p.friction >= 0) out += ", friction = " + plainNumber(p.friction);
        out += " }";
    }
    const bool ownClass = p.drawClass != Prefab().drawClass;
    if (ownClass || p.drawDistance > 0) {
        out += ", draw = { ";
        if (ownClass) out += "class = " + luaQuoted(p.drawClass);
        if (p.drawDistance > 0) out += std::string(ownClass ? ", " : "") + "distance = " + plainNumber(p.drawDistance);
        out += " }";
    }
    if (!p.script.empty()) out += ", script = " + luaQuoted(p.script);
    if (!p.uses.empty()) {
        out += ", uses = { ";
        for (const PrefabUse& u : p.uses) {
            out += "{ at = " + luaQuoted(u.at) + ", say = " + luaQuoted(u.say);
            if (!u.id.empty()) out += ", id = " + luaQuoted(u.id);
            if (!u.needs.empty()) out += ", needs = " + luaQuoted(u.needs);
            if (!u.unless.empty()) out += ", unless = " + luaQuoted(u.unless);
            if (!u.back.empty()) out += ", back = " + luaQuoted(u.back);
            for (const std::vector<std::pair<std::string, Real>>* list : {&u.move, &u.run, &u.sway}) {
                if (list->empty()) continue;
                out += std::string(list == &u.move ? ", move = { " : list == &u.run ? ", run = { " : ", sway = { ");
                for (std::size_t i = 0; i < list->size(); ++i) {
                    const std::string& node = (*list)[i].first;
                    // a name with a `*` in it is not a plain Lua key
                    out += std::string(i > 0 ? ", " : "") + (node.find('*') != std::string::npos ? "[" + luaQuoted(node) + "]" : node) + " = " + plainNumber((*list)[i].second);
                }
                out += " }";
            }
            if (u.speed > 0) out += ", speed = " + plainNumber(u.speed);
            if (u.seconds > 0) out += ", seconds = " + plainNumber(u.seconds);
            if (u.stop) out += ", stop = true";
            if (u.coast > 0) out += ", coast = " + plainNumber(u.coast);
            if (!u.sway.empty()) out += ", period = " + plainNumber(u.period);
            if (!u.light.empty()) out += ", light = " + luaQuoted(u.light);
            if (!u.lightAt.empty()) out += ", light_at = " + luaQuoted(u.lightAt);
            if (u.sit) out += ", sit = true";
            if (u.lie) out += ", lie = true";
            out += " }, ";
        }
        out += "}";
    }
    return out + " },";
}

bool findPrefabLine(const std::string& text, const std::string& name, std::size_t& begin, std::size_t& end, int* line) {
    // an entry is  name = { ... }  directly inside the table the list returns
    int depth = 0;
    bool found = false;
    std::size_t entryDepthAt = 0;
    walkLuaCode(text, [&](std::size_t i) -> std::size_t {
        const char c = text[i];
        if (c == '{') { ++depth; return i; }
        if (c == '}') {
            --depth;
            if (found && end == 0 && depth == 1 && i > entryDepthAt) {
                end = i + 1;
                std::size_t j = end;
                while (j < text.size() && (text[j] == ' ' || text[j] == '\t')) ++j;
                if (j < text.size() && text[j] == ',') end = j + 1;
            }
            return i;
        }
        if (found || depth != 1 || !nameChar(c) || (i > 0 && nameChar(text[i - 1]))) return i;
        std::size_t j = i;
        while (j < text.size() && nameChar(text[j])) ++j;
        if (text.compare(i, j - i, name) != 0) return j;
        std::size_t k = j;
        while (k < text.size() && std::isspace(static_cast<unsigned char>(text[k])) != 0) ++k;
        if (k >= text.size() || text[k] != '=') return j;
        ++k;
        while (k < text.size() && std::isspace(static_cast<unsigned char>(text[k])) != 0) ++k;
        if (k >= text.size() || text[k] != '{') return j;
        found = true;
        begin = i;
        end = 0;
        entryDepthAt = k;
        return k;      // (the brace is counted on the next step)
    });
    if (!found || end == 0) return false;
    if (line != nullptr) *line = 1 + static_cast<int>(std::count(text.begin(), text.begin() + static_cast<std::ptrdiff_t>(begin), '\n'));
    return true;
}

bool putPrefabLine(std::string& text, const Prefab& prefab, std::string* error) {
    const std::string written = prefabLine(prefab);
    std::size_t begin = 0, end = 0;
    if (findPrefabLine(text, prefab.name, begin, end)) {
        text.replace(begin, end - begin, written);
        return true;
    }
    // new: before the brace that closes the table
    int depth = 0;
    std::size_t close = std::string::npos;
    walkLuaCode(text, [&](std::size_t i) -> std::size_t {
        if (text[i] == '{') ++depth;
        if (text[i] == '}' && --depth == 0) close = i;
        return i;
    });
    if (close == std::string::npos) {
        if (error != nullptr) *error = "the list has no table to add '" + prefab.name + "' to (it returns { name = { ... }, ... })";
        return false;
    }
    std::size_t at = close;
    while (at > 0 && (text[at - 1] == ' ' || text[at - 1] == '\t')) --at;
    const bool ownLine = at == 0 || text[at - 1] == '\n';
    text.insert(at, std::string(ownLine ? "" : "\n") + "  " + written + "\n");
    return true;
}

bool savePrefab(const std::string& name, const nlohmann::json& line, const std::string& levelDir, std::string* error, const std::string& listFile,
                const std::string& modelsFile) {
    auto fail = [&](const std::string& why) {
        if (error != nullptr) *error = why;
        return false;
    };
    if (name.empty() || std::isdigit(static_cast<unsigned char>(name[0])) != 0 || !std::all_of(name.begin(), name.end(), nameChar))
        return fail("a prefab's name is letters, digits and _, and does not start with a digit");
    Prefab prefab;
    if (!readPrefabLine(name, line, prefab, levelDir, error, listFile, modelsFile)) return false;
    const std::string path = resolveScriptPath(listFile, levelDir);
    if (path.empty()) return fail(listFile + " cannot be found");
    std::string text;
    {
        std::ifstream in(path, std::ios::binary);
        if (!in) return fail(path + " cannot be read");
        text.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    }
    const std::string before = text;
    if (!putPrefabLine(text, prefab, error)) return false;
    if (text == before) return true;
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) return fail(path + " cannot be written");
    out << text;
    return true;
}

std::string prefabJson(const std::string& name, const std::string& levelDir, const std::string& listFile, const std::string& modelsFile) {
    nlohmann::json o;
    o["name"] = name;
    o["list"] = resolveScriptPath(listFile, levelDir);
    o["line"] = 0;
    {
        std::ifstream in(o["list"].get<std::string>(), std::ios::binary);
        const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        std::size_t begin = 0, end = 0;
        int line = 0;
        if (findPrefabLine(text, name, begin, end, &line)) o["line"] = line;
    }
    Prefab p;
    std::string why;
    if (!findPrefab(name, p, levelDir, &why, listFile, modelsFile)) {
        o["error"] = why;
        return o.dump();
    }
    o["model"] = p.model;
    nlohmann::json models;
    if (readScriptList(modelsFile, levelDir, models) && models.contains(p.model) && models[p.model].is_object()) {
        const nlohmann::json& m = models[p.model];
        if (m.contains("recipe") && m["recipe"].is_string()) {
            o["recipe"] = m["recipe"];
            if (m.contains("args")) o["args"] = m["args"];
        } else if (m.contains("files")) {
            o["files"] = true;
        }
    }
    o["physics"] = {{"body", p.body == Prefab::Body::Static ? "static" : "none"}};
    if (p.friction >= 0) o["physics"]["friction"] = p.friction;
    o["draw"] = {{"class", p.drawClass}};
    if (p.drawDistance > 0) o["draw"]["distance"] = p.drawDistance;
    o["script"] = p.script;
    return o.dump();
}

std::string prefabFieldsJson(const std::string& levelDir, const std::string& modelsFile) {
    nlohmann::json classes = nlohmann::json::array();
    for (int c = 0; c < static_cast<int>(DrawClass::Count); ++c)
        if (const char* known = drawClassName(static_cast<DrawClass>(c))) classes.push_back(known);
    nlohmann::json fields = nlohmann::json::array();
    fields.push_back({{"part", "model"}, {"name", ""}, {"type", "choice"}, {"choices", namedModels(levelDir, nullptr, modelsFile)},
                      {"hint", "The model it shows: a name in " + modelsFile + ". Every copy shares the one model."}});
    fields.push_back({{"part", "physics"}, {"name", "body"}, {"type", "choice"}, {"choices", {"static", "none"}},
                      {"hint", "static: the model's collision shapes become bodies (those on moving nodes move with them). none: a copy is not solid."}});
    fields.push_back({{"part", "physics"}, {"name", "friction"}, {"type", "number"}, {"min", 0.0}, {"step", 0.05}, {"optional", true},
                      {"hint", "One friction for every shape. Left unsaid: each shape's own."}});
    fields.push_back({{"part", "draw"}, {"name", "class"}, {"type", "choice"}, {"choices", classes},
                      {"hint", "The draw class whose distance the level sets."}});
    fields.push_back({{"part", "draw"}, {"name", "distance"}, {"type", "number"}, {"min", 0.0}, {"step", 10.0}, {"optional", true},
                      {"hint", "How far it is drawn, in metres. Left unsaid: the class's distance."}});
    fields.push_back({{"part", "script"}, {"name", ""}, {"type", "text"}, {"optional", true},
                      {"hint", "A behaviour script every copy runs (a file under assets/scripts), which moves the model's nodes. Empty: none."}});
    return fields.dump();
}

}  // namespace engine
