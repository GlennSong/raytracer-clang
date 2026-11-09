#ifndef RAYTRACER_ENGINE_SCRIPTING_BOOK_SLOT_H
#define RAYTRACER_ENGINE_SCRIPTING_BOOK_SLOT_H

// THE SLOT A LUA BOOK IS PUBLISHED IN (clouds.lua's CloudBook, trades.lua's TradeBook, lights.lua's LightBook): set
// from the loader, read from anywhere -- any thread -- by reference. A book once published is never freed: a reload
// puts a new one in the slot and keeps the old, so a reference a reader still holds stays good. (Books are set once at
// start and again only on a hot reload: what is kept is a handful.)

#include <memory>
#include <mutex>
#include <vector>

namespace engine {

template <class Book>
class BookSlot {
public:
    void set(const Book& book) {
        std::lock_guard<std::mutex> lock(mu_);
        published_.push_back(std::make_shared<const Book>(book));
    }
    // the book now, or null before one is set
    const Book* get() const {
        std::lock_guard<std::mutex> lock(mu_);
        return published_.empty() ? nullptr : published_.back().get();
    }

private:
    mutable std::mutex mu_;
    std::vector<std::shared_ptr<const Book>> published_;   // every one set, the last the live one
};

}  // namespace engine

#endif
