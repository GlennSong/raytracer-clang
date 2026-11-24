#ifndef RAYTRACER_ENGINE_LOT_CITYSIM_H
#define RAYTRACER_ENGINE_LOT_CITYSIM_H

// THE CITY SIM BLOCK AS THE LOT PASS SEES IT (ADR-0457). A level's `citysim` block holds two kinds of field: those the
// lot pass grows the city's lots and buildings from (its seed, its districts, its parcel grain, ...) and those only the
// running sim reads (how many people, which driving, a crowd file, the buses, ...). The lots bake is keyed on the first
// kind and nothing else, so changing how the sim runs never re-bakes a city.
//
// ONE LIST, kLotCitysimFields, says which fields the lot pass reads. LotCitysim keeps only those fields of a block, the
// lots producer's key hashes exactly what it keeps (keyText), and every reader of the lot pass reads the block through
// it. A read of a field the list does not name throws std::logic_error: a reader can never see what the key does not
// hash (the test `lot_citysim_the_lot_pass_reads_only_what_its_key_hashes` runs the readers over every shipped level).
// Adding a field the lot pass reads is one line in the list, and re-bakes every level that states it.

#include <nlohmann/json.hpp>

#include <cstddef>
#include <stdexcept>
#include <string>

namespace engine {

// Every citysim field the lot pass reads, and who reads it:
//   lots_producer.cpp lotsWanted, lot_grow_setup.cpp:  buildLots, planOnly, facadeDistance, detailDistance;
//   level_params.cpp readLotGrowParams:                seed, buildChance, sidewalk, downtownRadius, midtownRadius,
//                                                      plinth, hubRadius, wholeBlocks, recipe, recipeAt, coreness, stairs, lifts,
//                                                      districts, parcel, alleys, alleyWidth;
//   city_grow.cpp growCity:                            blocks, sidewalk;
//   city_producer.cpp cityRenderCell (the cell the lots are split by when written; also in the city key): renderCell.
constexpr const char* kLotCitysimFields[] = {
    "alleyWidth", "alleys",      "blocks",         "buildChance",   "buildLots",     "coreness",
    "detailDistance", "districts", "downtownRadius", "facadeDistance", "hubRadius",   "lifts",
    "midtownRadius", "parcel",   "planOnly",       "plinth",        "recipe",        "recipeAt",      "renderCell",
    "seed",         "sidewalk",  "stairs",         "wholeBlocks",
};
constexpr std::size_t kLotCitysimFieldCount = sizeof(kLotCitysimFields) / sizeof(kLotCitysimFields[0]);

class LotCitysim {
public:
    LotCitysim() = default;
    // The declared fields of a citysim block (a block that is not an object: none).
    static LotCitysim of(const nlohmann::json& citysim) {
        LotCitysim out;
        if (!citysim.is_object()) return out;
        for (const char* f : kLotCitysimFields)
            if (citysim.contains(f)) out.fields_[f] = citysim[f];
        return out;
    }
    // ...of a whole level file's root.
    static LotCitysim ofLevel(const nlohmann::json& level) {
        return of(level.is_object() ? level.value("citysim", nlohmann::json::object()) : nlohmann::json::object());
    }

    // THE READS. Each throws std::logic_error for a field kLotCitysimFields does not name.
    template <class T>
    T value(const char* field, const T& fallback) const {
        declared(field);
        return fields_.value(field, fallback);
    }
    std::string value(const char* field, const char* fallback) const { return value(field, std::string(fallback)); }
    bool contains(const char* field) const {
        declared(field);
        return fields_.contains(field);
    }
    // The field's JSON (null when the block does not state it).
    const nlohmann::json& at(const char* field) const {
        declared(field);
        static const nlohmann::json kNone;
        const nlohmann::json::const_iterator it = fields_.find(field);
        return it == fields_.end() ? kNone : *it;
    }

    // WHAT THE LOTS KEY HASHES: the declared fields the block states, as text (keys in order).
    std::string keyText() const { return fields_.dump(); }

    static bool isDeclared(const char* field) {
        for (const char* f : kLotCitysimFields)
            if (std::string(f) == field) return true;
        return false;
    }

private:
    static void declared(const char* field) {
        if (!isDeclared(field))
            throw std::logic_error(std::string("the lot pass read citysim.") + field +
                                   ", which kLotCitysimFields (lot_citysim.h) does not name: its lots key would not see it");
    }
    nlohmann::json fields_ = nlohmann::json::object();
};

}  // namespace engine

#endif
