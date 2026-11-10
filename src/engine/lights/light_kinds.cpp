#include "light_kinds.h"
#include "../scripting/book_slot.h"   // BookSlot

#ifdef RT_ENABLE_SCRIPTING
#include "../scripting/lights_lua.h"   // ensureLightsLoaded: lights.lua, once
#endif
#include "../../log.h"
#include <memory>
#include <mutex>
#include <set>

namespace engine {

int LightBook::kind(const std::string& name) const {
    for (std::size_t i = 0; i < kinds.size(); ++i)
        if (kinds[i].name == name) return static_cast<int>(i);
    return -1;
}

float LightBook::glowNits(const std::string& name) const {
    for (const Glow& g : glows)
        if (g.name == name) return g.nits;
    return 0.0f;
}

float lightGlowNits(const char* name) { return lightBook().glowNits(name); }

int LightBook::pool(const std::string& name) const {
    for (std::size_t i = 0; i < pools.size(); ++i)
        if (pools[i].name == name) return static_cast<int>(i);
    return -1;
}

namespace {
const LightBook& plainBook() {
    static const LightBook book = [] {
        LightBook b;
        LightKind k;
        k.name = "default";
        b.kinds.push_back(k);
        return b;
    }();
    return book;
}
BookSlot<LightBook>& bookSlot() {
    static BookSlot<LightBook> slot;
    return slot;
}
}  // namespace

void setLightBook(const LightBook& book) { bookSlot().set(book); }

const LightBook& lightBook() {
#ifdef RT_ENABLE_SCRIPTING
    ensureLightsLoaded();
#endif
    const LightBook* book = bookSlot().get();   // (kept for good: the reference stays valid across a reload)
    return book ? *book : plainBook();
}

int lightKindId(const char* name) {
    const int k = lightBook().kind(name);
    if (k < 0) {
        static std::mutex mu;
        static std::set<std::string> warned;
        std::lock_guard<std::mutex> lock(mu);
        if (warned.insert(name).second) LOG_WARN << "[lights] no light kind `" << name << "` in lights.lua: those lights are skipped";
    }
    return k;
}

}  // namespace engine
