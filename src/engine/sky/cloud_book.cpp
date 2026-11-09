#include "cloud_book.h"

#ifdef RT_ENABLE_SCRIPTING
#include "../scripting/clouds_lua.h"   // ensureCloudsLoaded: clouds.lua, once
#endif
#include <algorithm>
#include <memory>
#include <mutex>

namespace engine {

namespace {
std::shared_ptr<const CloudBook>& activeBook() {
    static std::shared_ptr<const CloudBook> book;
    return book;
}
std::mutex& bookMutex() {
    static std::mutex mu;
    return mu;
}
// (no clouds.lua: one type, the struct's own cumulus, in every slot)
const CloudBook& plainBook() {
    static const CloudBook book = [] {
        CloudBook b;
        for (CloudTypeSpec& t : b.types) t.name = "cumulus";
        return b;
    }();
    return book;
}
}  // namespace

void setCloudBook(const CloudBook& book) {
    std::lock_guard<std::mutex> lock(bookMutex());
    activeBook() = std::make_shared<const CloudBook>(book);
}

const CloudBook& cloudBook() {
#ifdef RT_ENABLE_SCRIPTING
    ensureCloudsLoaded();
#endif
    std::lock_guard<std::mutex> lock(bookMutex());
    return activeBook() ? *activeBook() : plainBook();
}

WeatherTargets weatherTargets(WeatherKind k) {
    const int i = std::clamp(static_cast<int>(k), 0, 3);
    return cloudBook().weathers[static_cast<std::size_t>(i)];
}

}  // namespace engine
