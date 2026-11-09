#ifndef RAYTRACER_ENGINE_SCRIPTING_LIGHTS_LUA_H
#define RAYTRACER_ENGINE_SCRIPTING_LIGHTS_LUA_H

// THE LIGHTS AS DATA (ADR-0362): assets/scripts/lights.lua -> LightBook (lights/light_kinds.h).

#include "../lights/light_kinds.h"
#include <string>

namespace engine {

// Read `code` (it returns the table) into `out`. False, with `err`, when it does not read; `out` is then untouched.
bool loadLightsSource(const std::string& code, LightBook& out, std::string* err);
// Read lights.lua once and make it the book in use; absent or broken, the plain kind stands (and it says so).
void ensureLightsLoaded();

}  // namespace engine

#endif
