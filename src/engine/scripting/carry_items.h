#ifndef RAYTRACER_ENGINE_SCRIPTING_CARRY_ITEMS_H
#define RAYTRACER_ENGINE_SCRIPTING_CARRY_ITEMS_H

// THE THINGS TO CARRY (ADR-0358): assets/scripts/carry_items.lua -- how heavy a thing may be and still be lifted, and
// the kinds `carry_spawn <name>` puts down (a box, a crate, a cooler): each one's size, weight and colour.

#include "../../rt_math.h"
#include <string>
#include <vector>

namespace engine {

class ScriptVM;

struct CarryItem {
    std::string name;
    Vec3 size{0.4, 0.4, 0.4};   // m, its box
    Real mass = 8;               // kg
    Vec3 color{0.6, 0.5, 0.4};
};
struct CarryCatalogue {
    Real limit = 45;             // kg: anything heavier stays where it is
    std::vector<CarryItem> items;
};

// Run the file's code (it returns the table) and read it into `out`. False, with `err`, when it does not read.
bool loadCarryCatalogue(ScriptVM& vm, const std::string& code, CarryCatalogue& out, std::string* err);

}  // namespace engine

#endif
