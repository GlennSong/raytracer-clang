#include "furniture_library.h"

namespace engine {

const char* verbName(Verb v) {
    switch (v) {
        case Verb::Sit: return "sit";
        case Verb::Lie: return "lie";
        default: return "?";
    }
}

bool verbByName(const std::string& name, Verb& out) {
    for (int i = 0; i < static_cast<int>(Verb::Count); ++i)
        if (name == verbName(static_cast<Verb>(i))) { out = static_cast<Verb>(i); return true; }
    return false;
}

void FurnitureLibrary::clear() {
    assets_ = {};
    has_ = {};
}

void FurnitureLibrary::set(FurnitureAsset a) {
    const int i = static_cast<int>(a.piece);
    if (i < 0 || i >= kPieceCount) return;
    assets_[static_cast<std::size_t>(i)] = std::move(a);
    has_[static_cast<std::size_t>(i)] = true;
}

const FurnitureAsset* FurnitureLibrary::find(Piece p) const {
    const int i = static_cast<int>(p);
    if (i < 0 || i >= kPieceCount || !has_[static_cast<std::size_t>(i)]) return nullptr;
    return &assets_[static_cast<std::size_t>(i)];
}

std::vector<Piece> FurnitureLibrary::goodsFor(const std::vector<std::string>& tags) const {
    std::vector<Piece> out;
    for (int i = 0; i < kPieceCount; ++i) {
        if (!has_[static_cast<std::size_t>(i)]) continue;
        const FurnitureAsset& a = assets_[static_cast<std::size_t>(i)];
        if (a.family != "goods") continue;
        bool hit = false;
        for (const std::string& t : a.tags)
            for (const std::string& want : tags) hit = hit || t == want;
        if (hit) out.push_back(static_cast<Piece>(i));
    }
    return out;
}

std::size_t FurnitureLibrary::size() const {
    std::size_t n = 0;
    for (bool h : has_) n += h ? 1 : 0;
    return n;
}

FurnitureLibrary& FurnitureLibrary::global() {
    static FurnitureLibrary lib;
    return lib;
}

}  // namespace engine
