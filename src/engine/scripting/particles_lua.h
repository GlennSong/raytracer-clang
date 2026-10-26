#ifndef RAYTRACER_ENGINE_SCRIPTING_PARTICLES_LUA_H
#define RAYTRACER_ENGINE_SCRIPTING_PARTICLES_LUA_H

// Reads the particle effects (engine/particles.h) from Lua: the global table `particles` that
// assets/scripts/particles.lua leaves behind, keyed by effect name.

#include "../particles.h"
#include <string>

namespace engine {

class ScriptVM;

// Each entry of the VM's `particles` global into `book` (added, or replacing the effect of that name). False (and
// `err`) on a malformed entry; an unknown field or shape is an error, so a typo cannot silently lose a setting.
bool readParticleBook(ScriptVM& vm, ParticleBook& book, std::string* err = nullptr);
// Run Lua `source` in a fresh sandboxed VM and read it.
bool loadParticleBookSource(const std::string& source, ParticleBook& book, std::string* err = nullptr);
// ParticleBook::global() from assets/scripts/particles.lua, once per process. Logs what it loaded.
bool ensureParticleBookLoaded();

}  // namespace engine

#endif
