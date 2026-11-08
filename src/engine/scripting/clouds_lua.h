#ifndef RAYTRACER_ENGINE_SCRIPTING_CLOUDS_LUA_H
#define RAYTRACER_ENGINE_SCRIPTING_CLOUDS_LUA_H

// THE CLOUDS AS DATA (ADR-0386): assets/scripts/clouds.lua -> CloudBook (sky/cloud_book.h).

#include "../sky/cloud_book.h"
#include <string>

namespace engine {

// Read `code` (it returns the table) into `out`. False, with `err`, when it does not read; `out` is then untouched.
bool loadCloudsSource(const std::string& code, CloudBook& out, std::string* err);
// Read clouds.lua once and make it the book in use; absent or broken, the plain book stands (and it says so).
void ensureCloudsLoaded();

}  // namespace engine

#endif
