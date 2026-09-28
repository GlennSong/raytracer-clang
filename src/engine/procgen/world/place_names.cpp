#include "place_names.h"

#include "../../asset_root.h"
#include "../../../log.h"

#include <fstream>

namespace engine {

namespace {
uint32_t mix(uint32_t a, uint32_t b) {
    uint32_t x = a * 747796405u + b * 2891336453u + 0x9E3779B9u;
    x ^= x >> 16; x *= 0x7FEB352Du; x ^= x >> 15; x *= 0x846CA68Bu; x ^= x >> 16;
    return x;
}
std::vector<std::string> strings(const nlohmann::json& j, const char* key) {
    std::vector<std::string> out;
    if (j.contains(key)) for (const auto& s : j[key]) out.push_back(s.get<std::string>());
    return out;
}
PlaceNameBook::Trait trait(const nlohmann::json& t) {
    PlaceNameBook::Trait r;
    r.chance = t.value("chance", 0.0);
    r.prefix = strings(t, "prefix");
    r.suffix = strings(t, "suffix");
    r.endings = strings(t, "endings");
    return r;
}
// Ash + ford -> Ashford; Wick + ham -> Wickham; a doubled letter at the seam is kept once (Hol + ley -> Holley stays, Moss + sands -> Mossands)
std::string join(const std::string& root, const std::string& ending) {
    if (!root.empty() && !ending.empty() && root.back() == ending.front() && root.size() > 2 && root[root.size() - 2] == ending.front())
        return root + ending.substr(1);
    return root + ending;
}
}  // namespace

PlaceNameBook parsePlaceNameBook(const nlohmann::json& file) {
    PlaceNameBook b;
    const nlohmann::json p = file.value("places", nlohmann::json::object());
    b.roots = strings(p, "roots");
    b.endings = strings(p, "endings");
    const nlohmann::json tr = p.value("traits", nlohmann::json::object());
    if (tr.contains("coastal")) b.coastal = trait(tr["coastal"]);
    if (tr.contains("river")) b.river = trait(tr["river"]);
    if (tr.contains("mountain")) b.mountain = trait(tr["mountain"]);
    b.cityChance = p.value("cityChance", b.cityChance);
    b.seed = p.value("seed", b.seed);
    b.routes = file.value("routes", nlohmann::json::object());
    return b;
}

const PlaceNameBook& placeNameBook() {
    static const PlaceNameBook book = [] {
        const std::string path = assetPath("assets/data/places.json");
        std::ifstream in(path);
        if (in) {
            try {
                nlohmann::json j;
                in >> j;
                PlaceNameBook b = parsePlaceNameBook(j);
                if (b.usable()) { b.fromAsset = true; return b; }
            } catch (const std::exception& e) {
                LOG_WARN << "[places] " << path << ": " << e.what();
            }
        }
        LOG_WARN << "[places] no usable assets/data/places.json: using the built-in names";
        PlaceNameBook b;
        b.roots = {"Ash", "Oak", "Elm", "Stan", "Wey", "Fair", "Red", "Hart", "Glen", "Mill"};
        b.endings = {"ford", "field", "ton", "by", "wick", "dale", "wood", "bury"};
        b.coastal = {0.4, {"Port "}, {" Bay"}, {"mouth", "haven"}};
        b.river = {0.4, {}, {" Falls"}, {"ford", "bridge"}};
        b.mountain = {0.8, {}, {" Ridge"}, {"crest"}};
        b.routes = {{"freeway", "1"}, {"pass", "2"}, {"mountain", "3"}, {"freewayName", "Route"},
                    {"loop", {{"clockwise", "Inner Loop"}, {"counterclockwise", "Outer Loop"}}}};
        return b;
    }();
    return book;
}

std::string placeName(const PlaceNameBook& book, const PlaceTraits& t, uint32_t worldSeed, uint32_t salt,
                      std::set<std::string>& used) {
    if (!book.usable()) return "Place " + std::to_string(salt);
    // the site's strongest trait, if it takes one: a mountain town nearly always, a coastal or river
    // place sometimes, a city less often than a town
    const PlaceNameBook::Trait* tr = nullptr;
    if (t.mountain) tr = &book.mountain;
    else if (t.river && (!t.coastal || mix(worldSeed, salt * 7 + 1) % 2)) tr = &book.river;
    else if (t.coastal) tr = &book.coastal;
    for (uint32_t attempt = 0; attempt < 200; ++attempt) {
        uint32_t r = mix(mix(worldSeed, book.seed), salt * 1000 + attempt);
        auto pick = [&](const std::vector<std::string>& v) { r = mix(r, 0x51ED); return v.empty() ? std::string() : v[r % v.size()]; };
        const double u = (mix(r, 99) % 10000) / 10000.0;
        const bool shaped = tr && u < (t.city ? book.cityChance : tr->chance);   // a city takes its site's form less often
        const std::string root = pick(book.roots);
        std::string name;
        if (!shaped) {
            name = join(root, pick(book.endings));
        } else {
            // the trait takes one of its forms: an ending of its own, a prefix, or a suffix
            const int forms = (tr->endings.empty() ? 0 : 1) + (tr->prefix.empty() ? 0 : 1) + (tr->suffix.empty() ? 0 : 1);
            int f = forms ? static_cast<int>(mix(r, 7) % static_cast<uint32_t>(forms)) : -1;
            if (!tr->endings.empty() && f-- == 0) name = join(root, pick(tr->endings));
            else if (!tr->prefix.empty() && f-- == 0) name = pick(tr->prefix) + join(root, pick(book.endings));
            else name = join(root, pick(book.endings)) + pick(tr->suffix);
        }
        // a root once per world while the banks last
        bool rootUsed = false;
        for (const std::string& u2 : used) if (u2.find(root) != std::string::npos) rootUsed = true;
        if (used.count(name) || (rootUsed && attempt < 150)) continue;
        used.insert(name);
        return name;
    }
    return "Place " + std::to_string(salt);
}

}  // namespace engine
