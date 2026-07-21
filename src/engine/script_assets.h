#ifndef RAYTRACER_ENGINE_SCRIPT_ASSETS_H
#define RAYTRACER_ENGINE_SCRIPT_ASSETS_H

#include <string>
#include <vector>

#ifdef RT_ENABLE_SCRIPTING
#include "scripting/script_modules.h"
#endif

namespace engine {

// Finding Lua on disk — the ONE place that answers "which paths does the engine
// search for a script?".
//
// It used to have five answers. level_loader, level_scene and arena_state's
// hot-reload watcher each carried their own copy of a three-candidate loop; the
// flora loader used a bare ifstream on "assets/scripts/flora.lua" (so a
// level-relative flora.lua silently never worked); and vehicle_system searched a
// "../assets/scripts/" that nothing else did. Divergent copies meant the loader
// and the file watcher could disagree about which file a name referred to, which
// is a hot-reload bug waiting to happen.
//
// The candidate order below is the UNION of all five, so unifying can only widen
// what resolves, never break a path that used to work. `../assets/scripts/` in
// particular must stay: it is how a binary run from build/ finds its scripts.
//
// Deliberately in engine/, not engine/scripting/ — that module contains no
// <fstream> and must not gain one (ADR-0023: the VM never touches the
// filesystem, the host does).

// Absolute-or-relative path of the first candidate that exists. Empty if none.
std::string resolveScriptPath(const std::string& file, const std::string& levelDir);

// Contents of the first candidate that exists. Empty if none, or if empty.
std::string loadScriptCode(const std::string& file, const std::string& levelDir);

#ifdef RT_ENABLE_SCRIPTING
// A ScriptModuleSource resolving `require "name"` to <root>/name.lua through the
// same candidate list, so a module and a script entity's `file` agree by
// construction. When `touched` is non-null every resolved path is appended, which
// is what lets the hot-reload watcher observe files it could not have predicted
// from the level JSON alone.
ScriptModuleSource makeModuleSource(const std::string& levelDir,
                                    std::vector<std::string>* touched = nullptr);
#endif

}  // namespace engine

#endif
