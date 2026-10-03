#ifndef RAYTRACER_ENGINE_SCRIPTING_FURNITURE_LIBRARY_LUA_H
#define RAYTRACER_ENGINE_SCRIPTING_FURNITURE_LIBRARY_LUA_H

// Reads the furniture library's descriptions (procgen/furniture_library.h) from Lua: the global table
// `furniture_library` a chunk like assets/scripts/furniture_library.lua leaves behind, keyed by kit piece name.

#include "../procgen/furniture_library.h"
#include <string>

namespace engine {

class ScriptVM;

// Fill `out` from the VM's `furniture_library` global. False (and `err`) on a malformed entry; unknown piece names,
// verbs or spot ids are errors, so a typo cannot silently drop a seat.
bool readFurnitureLibrary(ScriptVM& vm, FurnitureLibrary& out, std::string* err = nullptr);

// The room programs (the VM's `furniture_rooms` global, furniture_rooms.lua) into `out`; absent is not an error.
bool readRoomPrograms(ScriptVM& vm, FurnitureLibrary& out, std::string* err = nullptr);

// Run Lua `source` (the library file's text, optionally followed by the room programs') in a fresh sandboxed VM and
// read both.
bool loadFurnitureLibrarySource(const std::string& source, FurnitureLibrary& out, std::string* err = nullptr);

// FurnitureLibrary::global() from assets/scripts/furniture_library.lua, once per process (later calls return the
// first result). Logs what it loaded.
bool ensureFurnitureLibraryLoaded();

}  // namespace engine

#endif
