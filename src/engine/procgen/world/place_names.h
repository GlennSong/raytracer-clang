#ifndef RAYTRACER_ENGINE_PROCGEN_WORLD_PLACE_NAMES_H
#define RAYTRACER_ENGINE_PROCGEN_WORLD_PLACE_NAMES_H

// NAMING THE PLACES (Glenn, 2026-09-25: "we can name the towns and show which direction they are ...
// I'll leave it to you to generate the town names. They don't have to be Hawaiian."). The towns and
// cities of a world get names the signs can carry. The words are data (assets/data/places.json);
// this is the algorithm: a ROOT and an ENDING run together (Ash + ford -> Ashford), and the site can
// shape the name -- a coastal place may be "Port ..." or "... Bay", one on a river "...ford" or
// "... Falls", a mountain town "... Ridge". Each word once per world while the banks last; the same
// seed gives the same names.

#include <nlohmann/json.hpp>

#include <cstdint>
#include <set>
#include <string>
#include <vector>

namespace engine {

struct PlaceNameBook {
    std::vector<std::string> roots, endings;
    struct Trait {
        double chance = 0.0;
        std::vector<std::string> prefix, suffix, endings;
    };
    Trait coastal, river, mountain;
    double cityChance = 0.25;   // how often a CITY takes its site's form (a town: its trait's chance)
    uint32_t seed = 20260925;
    nlohmann::json routes;      // how the roads are signed (route numbers, loop directions)
    bool fromAsset = false;
    bool usable() const { return !roots.empty() && !endings.empty(); }
};

PlaceNameBook parsePlaceNameBook(const nlohmann::json& file);
// assets/data/places.json, read once; a small built-in book when it is missing.
const PlaceNameBook& placeNameBook();

struct PlaceTraits {
    bool city = false, coastal = false, river = false, mountain = false;
};

// One name, unique within `used` (which it extends). `salt` makes each place's draw its own.
std::string placeName(const PlaceNameBook& book, const PlaceTraits& t, uint32_t worldSeed, uint32_t salt,
                      std::set<std::string>& used);

}  // namespace engine

#endif
