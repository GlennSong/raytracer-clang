#ifndef RAYTRACER_ENGINE_SCRIPTING_FURNITURE_LIBRARY_LUA_H
#define RAYTRACER_ENGINE_SCRIPTING_FURNITURE_LIBRARY_LUA_H

// Reads the furniture library's descriptions (procgen/furniture_library.h) from Lua: the global table
// `furniture_library` a chunk like assets/scripts/furniture_library.lua leaves behind, keyed by kit piece name.

#include "../procgen/furniture_library.h"
#include "../procgen/city/floor_zones.h"
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

// THE FLOOR PROGRAMS (floor_zones.h): the VM's `floor_programs` global (assets/scripts/floor_programs.lua) into
// `out`. Unknown kinds, sites, walls or `near` values are errors.
bool readFloorPrograms(ScriptVM& vm, FloorProgramBook& out, std::string* err = nullptr);
bool loadFloorProgramsSource(const std::string& source, FloorProgramBook& out, std::string* err = nullptr);

// FurnitureLibrary::global() from assets/scripts/furniture_library.lua -- and FloorProgramBook::global() from
// floor_programs.lua -- once per process (later calls return the
// first result). Logs what it loaded.
bool ensureFurnitureLibraryLoaded();

// HOT RELOAD (the building-uses plan): floor_programs.lua read again when it has changed since it was last read (looked
// at no more than once a second), into FloorProgramBook::global(). Returns a generation, bumped on each reload, so the
// interior system can grow its resident interiors again. Call from the thread that grows interiors.
int floorProgramsGeneration();

}  // namespace engine

#endif
