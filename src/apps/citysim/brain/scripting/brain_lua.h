#ifndef RAYTRACER_APPS_CITYSIM_BRAIN_SCRIPTING_BRAIN_LUA_H
#define RAYTRACER_APPS_CITYSIM_BRAIN_SCRIPTING_BRAIN_LUA_H

// The brain's states from Lua (assets/scripts/brain.lua): the global `brain` read into a BrainSpec, names and all,
// once at load. It checks only the shape (a table where a table is expected, a name where a name is expected, no key a
// row does not have); what the names mean is checked by compileBrainTable (../brain_table.h). Refused (false, `err`
// lists every fault as "brain.lua: key what was wrong") on a missing table, a row that is not a table, a key that is
// not one of a row's, or an `on` entry that is not a name.

#include "../brain_table.h"

#include <string>

namespace engine {
class ScriptVM;
}

namespace citysim {
namespace brain {

bool loadBrainSpec(engine::ScriptVM& vm, BrainSpec& out, std::string* err);

}  // namespace brain
}  // namespace citysim

#endif
