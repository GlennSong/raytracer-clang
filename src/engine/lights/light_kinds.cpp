#include "light_kinds.h"

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
std::shared_ptr<const LightBook>& activeBook() {
    static std::shared_ptr<const LightBook> book;
    return book;
}
std::mutex& bookMutex() {
    static std::mutex mu;
    return mu;
}
}  // namespace

void setLightBook(const LightBook& book) {
    std::lock_guard<std::mutex> lock(bookMutex());
    activeBook() = std::make_shared<const LightBook>(book);
}

const LightBook& lightBook() {
#ifdef RT_ENABLE_SCRIPTING
    ensureLightsLoaded();
#endif
    std::lock_guard<std::mutex> lock(bookMutex());
    return activeBook() ? *activeBook() : plainBook();
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
