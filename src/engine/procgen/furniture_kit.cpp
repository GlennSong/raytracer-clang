#include "furniture_kit.h"
#include "../../log.h"
#include "../scripting/furniture_models_lua.h"   // the recipes: assets/scripts/furniture_models.lua (#191)
#include <map>
#include <memory>
#include <mutex>

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

}  // namespace

const FurniturePiece& furniturePiece(Piece p, uint32_t variant) {
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
    return i >= 0 && i < kPieceCount ? kPieceNames[i] : "?";
}

bool furniturePieceByName(const std::string& name, Piece& out) {
    for (int i = 0; i < kPieceCount; ++i)
        if (name == kPieceNames[i]) { out = static_cast<Piece>(i); return true; }
    return false;
}

}  // namespace engine
