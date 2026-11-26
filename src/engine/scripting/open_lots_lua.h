#pragma once
// open_lots.lua: what stands on an open lot (the prefabs a playground chooses from), read once (#315, ADR-0524).
#include "../procgen/city/city_lots.h"   // OpenLotBook

#include <string>

namespace engine {

bool loadOpenLotsSource(const std::string& code, OpenLotBook& out, std::string* err);
// Reads open_lots.lua once and hands it to setOpenLotBook; without the file the book stays empty.
void ensureOpenLotsLoaded();

}  // namespace engine
