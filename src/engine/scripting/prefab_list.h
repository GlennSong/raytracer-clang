#ifndef RAYTRACER_ENGINE_SCRIPTING_PREFAB_LIST_H
#define RAYTRACER_ENGINE_SCRIPTING_PREFAB_LIST_H

// The list of prefabs, read (#191, step 7): assets/scripts/prefabs.lua, or a level's own. See procgen/prefab.h.

#include "../procgen/prefab.h"

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

namespace engine {

// The prefab named `name`, checked: its line says only what a prefab can say today, and its model is in the list of
// models. False with `error` otherwise. `levelDir`: where a level keeps its own scripts (may be empty). `list` and
// `models`: the two lists' files, for a test with lists of its own.
bool findPrefab(const std::string& name, Prefab& out, const std::string& levelDir = std::string(), std::string* error = nullptr,
                const std::string& list = "prefabs.lua", const std::string& models = "models.lua");

// ONE LINE OF THE LIST, CHECKED: `line` is what the list's table gives for `name` (model, physics, draw, script), as
// JSON. The same check whether the line came from the file (findPrefab) or from a form (savePrefab).
bool readPrefabLine(const std::string& name, const nlohmann::json& line, Prefab& out, const std::string& levelDir = std::string(),
                    std::string* error = nullptr, const std::string& list = "prefabs.lua", const std::string& models = "models.lua");

// A PREFAB AS DATA, for whatever shows one (the editor's inspector, the control channel's `prefab?`): JSON with
//   "name", "error" (only when its line is wrong: then nothing else is sure),
//   "model", and how that model is built: "recipe" + "args" (a script), or "files": true,
//   "physics": { "body": "static" | "none", "friction": n (absent: each shape's own) },
//   "draw": { "class": name, "distance": metres (absent: the class's) }, "script": file ("" for none),
//   "list": the list's file path, "line": the line in it where the prefab's entry starts (0: not found).
std::string prefabJson(const std::string& name, const std::string& levelDir = std::string(), const std::string& list = "prefabs.lua",
                       const std::string& models = "models.lua");

// WHAT A PREFAB'S LINE CAN SAY, as data, for a form: a JSON array of
//   { "part": "model" | "physics" | "draw" | "script", "name": the key ("" when the part is the value itself),
//     "type": "choice" | "number" | "text", "choices": [...], "min": n, "step": n, "optional": true (may be left unsaid),
//     "hint": words }
// The choices are read now: the models in models.lua, the engine's draw classes.
std::string prefabFieldsJson(const std::string& levelDir = std::string(), const std::string& models = "models.lua");

// THE LINE A PREFAB IS WRITTEN AS:  name = { model = "crane", physics = { body = "static" }, ... },
// Only what differs from what an unsaid key means is written.
std::string prefabLine(const Prefab& prefab);

// `listText` (a prefab list's text) with `prefab`'s entry replaced by prefabLine, or added as the table's last line
// when it has none. ONLY the entry changes: every other line, and the comments around the entry, stay as typed;
// comments INSIDE the entry go. False with `error` when the text has no table to add to.
bool putPrefabLine(std::string& listText, const Prefab& prefab, std::string* error = nullptr);
// Where `name`'s entry is in the text: [begin, end) and the line it starts on (from 1). False when it has none.
bool findPrefabLine(const std::string& listText, const std::string& name, std::size_t& begin, std::size_t& end, int* line = nullptr);

// CHANGE A PREFAB IN THE LIST'S FILE: `line` checked as readPrefabLine checks it, then written into the file with
// putPrefabLine. Nothing is written when the line is wrong. The one place the editor rewrites a script.
bool savePrefab(const std::string& name, const nlohmann::json& line, const std::string& levelDir = std::string(), std::string* error = nullptr,
                const std::string& list = "prefabs.lua", const std::string& models = "models.lua");

// The names in the list, sorted. Empty with `error` when the list cannot be read.
std::vector<std::string> namedPrefabs(const std::string& levelDir = std::string(), std::string* error = nullptr, const std::string& list = "prefabs.lua");

}  // namespace engine

#endif
