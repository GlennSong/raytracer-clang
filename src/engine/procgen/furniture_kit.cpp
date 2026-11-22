#include "furniture_kit.h"
#include "../../log.h"
#include "../scripting/furniture_models_lua.h"   // the recipes: assets/scripts/furniture_models.lua (#191)
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace engine {

namespace {

const char* const kPieceNames[kPieceCount] = {
    "desk", "office_chair", "monitor", "filing_cabinet", "bed", "nightstand", "wardrobe", "sofa", "coffee_table",
    "tv_unit", "kitchen_base", "kitchen_sink", "kitchen_hob", "kitchen_tall", "kitchen_wall", "dining_table",
    "dining_chair", "bathtub", "toilet", "vanity", "lounge_chair", "planter", "picture", "shelving", "rug",
    "desk_pod", "cubicle", "meeting_table", "whiteboard",
    "shop_counter", "gondola", "wall_shelf", "clothes_rack", "cafe_table", "display_case", "drinks_fridge", "bookcase",
    "pallet_rack", "checkout", "bench", "plaza_bench", "bistro_table", "bistro_chair", "ceiling_light",
    "desk_lamp", "potted_plant", "book_stack", "kettle", "fruit_bowl", "vase", "mug",
    "single_bed", "school_desk", "school_chair", "lecture_row", "lectern", "lab_bench", "reading_table", "aisle_step", "bleacher",
    "bar_counter", "back_bar", "bar_stool", "booth", "high_table", "dj_booth", "dance_floor", "kitchen_pass",
    "desk_screen", "reception_desk", "hospital_bed", "exam_couch", "waiting_seats",
    "pew", "altar", "cinema_row", "cinema_screen", "market_stall", "judge_bench", "jury_box",
    "gear_locker", "machine", "workbench", "pallet_stack", "forklift",
    "water_cooler", "copier", "phone_booth", "fitting_room", "sun_lounger", "beach_umbrella", "beach_towel"};

// the prefab pieces, in the order registered: piece number kPieceCount + 1 + index
struct PrefabPiece {
    std::string name;
    Vec3 size;
};
std::mutex& prefabMutex() { static std::mutex mu; return mu; }
std::deque<PrefabPiece>& prefabPieces() { static std::deque<PrefabPiece> list; return list; }   // (a deque: a name handed out never moves)
int prefabIndex(Piece p) { return static_cast<int>(p) - kPieceCount - 1; }

}  // namespace

Piece registerPrefabPiece(const std::string& name, const Vec3& size) {
    for (int i = 0; i < kPieceCount; ++i)
        if (name == kPieceNames[i]) return Piece::Count;
    std::lock_guard<std::mutex> lock(prefabMutex());
    std::deque<PrefabPiece>& list = prefabPieces();
    for (std::size_t i = 0; i < list.size(); ++i)
        if (list[i].name == name) { list[i].size = size; return static_cast<Piece>(kPieceCount + 1 + static_cast<int>(i)); }
    if (kPieceCount + 1 + static_cast<int>(list.size()) >= kPieceMax) return Piece::Count;
    list.push_back(PrefabPiece{name, size});
    return static_cast<Piece>(kPieceCount + static_cast<int>(list.size()));
}

bool pieceIsPrefab(Piece p) {
    std::lock_guard<std::mutex> lock(prefabMutex());
    const int i = prefabIndex(p);
    return i >= 0 && i < static_cast<int>(prefabPieces().size());
}

const FurniturePiece& furniturePiece(Piece p, uint32_t variant) {
    if (prefabIndex(p) >= 0) {   // a prefab piece: its footprint, and nothing to draw or collide with here
        static std::mutex mu;
        static std::map<int, std::unique_ptr<FurniturePiece>> made;
        Vec3 size(1, 1, 1);
        {
            std::lock_guard<std::mutex> lock(prefabMutex());
            const int i = prefabIndex(p);
            if (i < static_cast<int>(prefabPieces().size())) size = prefabPieces()[static_cast<std::size_t>(i)].size;
        }
        std::lock_guard<std::mutex> lock(mu);
        std::unique_ptr<FurniturePiece>& fp = made[static_cast<int>(p)];
        if (fp == nullptr) fp = std::make_unique<FurniturePiece>();
        fp->size = size;
        fp->solid = false;   // the prefab's own collision shapes are its bodies
        return *fp;
    }
    static std::mutex mu;
    static std::map<uint64_t, std::unique_ptr<FurniturePiece>> cache;
    const uint64_t key = (static_cast<uint64_t>(p) << 32) | variant;
    std::lock_guard<std::mutex> lock(mu);
    auto it = cache.find(key);
    if (it == cache.end()) {
        // modelled by its Lua recipe (furniture_models.lua); a recipe that fails leaves the piece empty, and says so
        auto fp = std::make_unique<FurniturePiece>();
        std::string err;
        if (!buildFurnitureModel(furniturePieceName(p), variant, *fp, &err))
            LOG_WARN << "[furniture] " << furniturePieceName(p) << " v" << variant << ": " << err;
        it = cache.emplace(key, std::move(fp)).first;
    }
    return *it->second;
}

const char* furniturePieceName(Piece p) {
    const int i = static_cast<int>(p);
    if (i >= 0 && i < kPieceCount) return kPieceNames[i];
    std::lock_guard<std::mutex> lock(prefabMutex());
    const int k = prefabIndex(p);
    // (the names are never taken out or moved while the process runs: the list only grows, by whole strings)
    return k >= 0 && k < static_cast<int>(prefabPieces().size()) ? prefabPieces()[static_cast<std::size_t>(k)].name.c_str() : "?";
}

bool furniturePieceByName(const std::string& name, Piece& out) {
    for (int i = 0; i < kPieceCount; ++i)
        if (name == kPieceNames[i]) { out = static_cast<Piece>(i); return true; }
    std::lock_guard<std::mutex> lock(prefabMutex());
    for (std::size_t i = 0; i < prefabPieces().size(); ++i)
        if (prefabPieces()[i].name == name) { out = static_cast<Piece>(kPieceCount + 1 + static_cast<int>(i)); return true; }
    return false;
}

}  // namespace engine
