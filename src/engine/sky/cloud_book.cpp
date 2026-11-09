#include "cloud_book.h"
#include "../scripting/book_slot.h"   // BookSlot

#ifdef RT_ENABLE_SCRIPTING
#include "../scripting/clouds_lua.h"   // ensureCloudsLoaded: clouds.lua, once
#endif
#include <algorithm>
#include <memory>

namespace engine {

namespace {
BookSlot<CloudBook>& bookSlot() {
    static BookSlot<CloudBook> slot;
    return slot;
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

void setCloudBook(const CloudBook& book) { bookSlot().set(book); }

const CloudBook& cloudBook() {
#ifdef RT_ENABLE_SCRIPTING
    ensureCloudsLoaded();
#endif
    const CloudBook* book = bookSlot().get();   // (kept for good: the reference stays valid across a reload)
    return book ? *book : plainBook();
}

WeatherTargets weatherTargets(WeatherKind k) {
    const int i = std::clamp(static_cast<int>(k), 0, 3);
    return cloudBook().weathers[static_cast<std::size_t>(i)];
}

}  // namespace engine
