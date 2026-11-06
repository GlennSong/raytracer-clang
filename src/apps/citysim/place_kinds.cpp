#include "place_kinds.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>

namespace citysim {

int PlaceKindCatalog::find(const std::string& name) const {
    for (std::size_t i = 0; i < kinds.size(); ++i)
        if (kinds[i].name == name) return static_cast<int>(i);
    return -1;
}

int PlaceKindCatalog::kindFor(const std::string& recipe, PlaceType type) const {
    if (!recipe.empty())
        for (std::size_t i = 0; i < kinds.size(); ++i)
            for (const std::string& r : kinds[i].recipes)
                if (r == recipe) return static_cast<int>(i);
    return find(placeTypeName(type));
}

uint32_t PlaceKindCatalog::tag(const std::string& name) const {
    for (std::size_t i = 0; i < tagNames.size() && i < 32; ++i)
        if (tagNames[i] == name) return 1u << i;
    return 0;
}

void PlaceKindCatalog::internTags() {
    tagNames.clear();
    for (const PlaceKind& k : kinds)
        for (const std::string& t : k.tagNames)
            if (std::find(tagNames.begin(), tagNames.end(), t) == tagNames.end()) tagNames.push_back(t);
    for (PlaceKind& k : kinds) {
        k.tags = 0;
        for (const std::string& t : k.tagNames) k.tags |= tag(t);
    }
}

std::string PlaceKindCatalog::check() const {
    for (int t = 0; t < static_cast<int>(PlaceType::Count); ++t) {
        const char* n = placeTypeName(static_cast<PlaceType>(t));
        const int k = find(n);
        if (k < 0) return std::string("no kind named `") + n + "` (every place type needs one)";
        if (kinds[static_cast<std::size_t>(k)].type != static_cast<PlaceType>(t))
            return std::string("kind `") + n + "` must be of type " + n;
    }
    std::set<std::string> seen, recipes;
    for (const PlaceKind& k : kinds) {
        if (!seen.insert(k.name).second) return "kind `" + k.name + "` twice";
        for (const std::string& r : k.recipes)
            if (!recipes.insert(r).second) return "recipe `" + r + "` claimed by two kinds";
    }
    if (tagNames.size() > 32) return "more than 32 tags";
    for (const auto& [use, rule] : dwellings) {
        if (rule.perUnit) continue;
        double s = 0;
        for (const DwellingSize& d : rule.mix) {
            if (d.m2 <= 0) return "dwellings." + use + ": every size needs m2";
            s += d.share;
        }
        if (s <= 0) return "dwellings." + use + ": the mix needs shares";
        if (rule.usable <= 0 || rule.usable > 1) return "dwellings." + use + ": usable is a share, (0, 1]";
    }
    if (houseSizes.empty()) return "house_sizes: none";
    for (std::size_t i = 1; i < houseSizes.size(); ++i)
        if (houseSizes[i].m2 < houseSizes[i - 1].m2) return "house_sizes: not ascending by m2";
    if (!dwellings.count("apartments")) return "dwellings: no `apartments` rule (the fallback)";
    return {};
}

std::string PlaceKindCatalog::describe() const {
    std::ostringstream o;
    for (const PlaceKind& k : kinds) {
        o << k.name << " type=" << placeTypeName(k.type) << " bed_m2=" << k.m2PerBed << " recipes=";
        for (const std::string& r : k.recipes) o << r << ",";
        o << " tags=";
        for (const std::string& t : k.tagNames) o << t << ",";
        o << "\n";
    }
    for (const auto& [use, rule] : dwellings) {
        o << "dwellings " << use << (rule.perUnit ? " per_unit" : " by_area") << " usable=" << rule.usable;
        for (const DwellingSize& d : rule.mix) o << " " << d.beds << "/" << d.m2 << "/" << d.share;
        o << "\n";
    }
    o << "house_sizes";
    for (const DwellingSize& d : houseSizes) o << " " << d.beds << "/" << d.m2;
    o << "\n";
    return o.str();
}

const DwellingRule* PlaceKindCatalog::dwellingRule(const std::string& use) const {
    auto it = dwellings.find(use);
    if (it == dwellings.end()) it = dwellings.find("apartments");
    return it == dwellings.end() ? nullptr : &it->second;
}

void dwellingsOf(const PlaceKindCatalog& c, const DwellingRule& rule, double floorArea, int units, std::vector<int>& beds) {
    if (rule.perUnit || rule.mix.empty()) {
        const int n = std::max(1, units);
        const double each = floorArea / n;
        int b = c.houseSizes.empty() ? 1 : c.houseSizes.front().beds;
        for (const DwellingSize& s : c.houseSizes)
            if (each >= s.m2) b = s.beds;
        for (int i = 0; i < n; ++i) beds.push_back(b);
        return;
    }
    // the mix's mean size sets how many; largest remainder splits them by share, ties to the earlier size
    double shares = 0, mean = 0;
    for (const DwellingSize& d : rule.mix) shares += d.share;
    for (const DwellingSize& d : rule.mix) mean += d.m2 * d.share / shares;
    const int n = std::max(1, static_cast<int>(std::lround(floorArea * rule.usable / mean)));
    std::vector<int> count(rule.mix.size(), 0);
    std::vector<std::pair<double, std::size_t>> rest;
    int given = 0;
    for (std::size_t i = 0; i < rule.mix.size(); ++i) {
        const double want = n * rule.mix[i].share / shares;
        count[i] = static_cast<int>(std::floor(want));
        given += count[i];
        rest.push_back({want - count[i], i});
    }
    std::stable_sort(rest.begin(), rest.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
    for (std::size_t j = 0; given < n; ++j, ++given) ++count[rest[j % rest.size()].second];
    for (std::size_t i = 0; i < rule.mix.size(); ++i)
        for (int k = 0; k < count[i]; ++k) beds.push_back(rule.mix[i].beds);
}

PlaceKindCatalog defaultPlaceKindCatalog() {
    PlaceKindCatalog c;
    auto kind = [&](const char* name, PlaceType type, std::vector<std::string> recipes, std::vector<std::string> tags,
                    double m2PerBed = 0) {
        PlaceKind k;
        k.name = name;
        k.type = type;
        k.recipes = std::move(recipes);
        k.tagNames = std::move(tags);
        k.m2PerBed = m2PerBed;
        c.kinds.push_back(std::move(k));
    };
    kind("home", PlaceType::Home, {}, {});
    kind("shop", PlaceType::Shop, {}, {});
    kind("office", PlaceType::Office, {}, {});
    kind("park", PlaceType::Park, {}, {});
    kind("civic", PlaceType::Civic, {}, {});
    kind("cafe", PlaceType::Cafe, {}, {});
    kind("restaurant", PlaceType::Restaurant, {}, {});
    kind("supermarket", PlaceType::Supermarket, {}, {});
    kind("bar", PlaceType::Bar, {}, {});
    kind("club", PlaceType::Club, {}, {});
    kind("teaching", PlaceType::Civic, {"teaching_hall"}, {"campus", "classes"});
    kind("library", PlaceType::Civic, {"campus_library"}, {"campus", "library"});
    kind("residence", PlaceType::Home, {"residence_hall"}, {"campus", "beds"}, 45.0);
    kind("quad", PlaceType::Park, {"campus_quad", "dorm_courtyard"}, {"campus", "grounds"});
    kind("field", PlaceType::Park, {"sports_field"}, {"campus", "grounds", "sports"});
    c.internTags();
    DwellingRule house;
    house.perUnit = true;
    c.dwellings["house"] = house;
    DwellingRule flats;
    flats.usable = 0.75;
    flats.mix = {{0, 40, 0.15}, {1, 55, 0.35}, {2, 75, 0.35}, {3, 100, 0.15}};
    c.dwellings["apartments"] = flats;
    c.houseSizes = {{1, 0, 0}, {2, 70, 0}, {3, 110, 0}, {4, 170, 0}, {5, 260, 0}};
    return c;
}

}  // namespace citysim
