#include "furniture_library.h"
#include <cmath>

namespace engine {

const char* verbName(Verb v) {
    switch (v) {
        case Verb::Sit: return "sit";
        case Verb::Lie: return "lie";
        case Verb::Stand: return "stand";
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
    programs_.clear();
}

Piece FurnitureLibrary::pick(const FurnPick& p, Real w, Real d, uint32_t hash) const {
    if (p.piece != Piece::Count) return p.piece;
    Piece best = Piece::Count;
    Real bestArea = -1;
    uint32_t bestKey = 0;
    for (int i = 0; i < kPieceCount; ++i) {
        if (!has_[static_cast<std::size_t>(i)]) continue;
        const FurnitureAsset& a = assets_[static_cast<std::size_t>(i)];
        if (a.family != p.family) continue;
        bool all = true;
        for (const std::string& want : p.tags) {
            bool hit = false;
            for (const std::string& t : a.tags) hit = hit || t == want;
            all = all && hit;
        }
        if (!all) continue;
        const Vec3 sz = furniturePiece(static_cast<Piece>(i), 0).size;
        if (w > 0 && sz.x > w + 0.05) continue;   // it must fit the slot
        if (d > 0 && sz.z > d + 0.05) continue;
        const Real area = sz.x * sz.z;
        const uint32_t key = (hash ^ (static_cast<uint32_t>(i) * 2654435761u)) >> 7;
        if (area > bestArea + 1e-6 || (std::fabs(area - bestArea) <= 1e-6 && key > bestKey)) {
            best = static_cast<Piece>(i);
            bestArea = area;
            bestKey = key;
        }
    }
    return best;
}

void FurnitureLibrary::setProgram(RoomProgram prog) {
    for (RoomProgram& r : programs_)
        if (r.name == prog.name) { r = std::move(prog); return; }
    programs_.push_back(std::move(prog));
}

const RoomProgram* FurnitureLibrary::program(const std::string& name) const {
    for (const RoomProgram& r : programs_)
        if (r.name == name) return &r;
    return nullptr;
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

namespace engine {

Mat4 interactXform(const FurnitureAsset* a, const Mat4& xform, uint32_t variant) {
    if (!a || a->rise == 0) return xform;
    return xform * Mat4::translate(0, a->rise * static_cast<Real>((variant >> 5) & 7u), 0);
}

}  // namespace engine
