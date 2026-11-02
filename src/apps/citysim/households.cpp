#include "households.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>

namespace citysim {

int PeopleCatalog::stage(const std::string& name) const {
    for (std::size_t i = 0; i < stages.size(); ++i)
        if (stages[i].name == name) return static_cast<int>(i);
    return -1;
}

int PeopleCatalog::band(const std::string& name) const {
    for (std::size_t i = 0; i < bands.size(); ++i)
        if (bands[i].name == name) return static_cast<int>(i);
    return -1;
}

int PeopleCatalog::household(const std::string& name) const {
    for (std::size_t i = 0; i < households.size(); ++i)
        if (households[i].name == name) return static_cast<int>(i);
    return -1;
}

std::string PeopleCatalog::check() const {
    if (stages.empty()) return "stages: none";
    if (bands.empty()) return "bands: none";
    if (households.empty()) return "households: none";
    std::set<std::string> seen;
    for (const LifeStage& s : stages)
        if (!seen.insert("stage " + s.name).second) return "stage `" + s.name + "` twice";
    for (const AgeBand& b : bands) {
        if (!seen.insert("band " + b.name).second) return "band `" + b.name + "` twice";
        if (b.lo < 0 || b.hi < b.lo || b.hi > 120) return "bands." + b.name + ": ages must be lo <= hi, 0..120";
        double s = 0;
        for (const auto& [st, share] : b.stages) {
            if (st < 0 || st >= static_cast<int>(stages.size())) return "bands." + b.name + ": an unknown stage";
            s += share;
        }
        if (s <= 0) return "bands." + b.name + ": the stages need shares";
    }
    int widest = 0;
    for (const HouseholdType& h : households) {
        if (!seen.insert("household " + h.name).second) return "household `" + h.name + "` twice";
        if (h.share <= 0) return "households." + h.name + ": needs a share";
        if (h.bedsLo < 0 || h.bedsHi < h.bedsLo) return "households." + h.name + ": beds must be lo <= hi";
        if (h.groups.empty()) return "households." + h.name + ": no members";
        for (const MemberGroup& g : h.groups) {
            if (g.lo < 1 || g.hi < g.lo) return "households." + h.name + ": a count must be 1 or more, lo <= hi";
            double s = 0;
            for (const auto& [b, share] : g.bands) {
                if (b < 0 || b >= static_cast<int>(bands.size())) return "households." + h.name + ": an unknown band";
                s += share;
            }
            if (s <= 0) return "households." + h.name + ": a group's bands need shares";
        }
        widest = std::max(widest, h.bedsHi);
    }
    for (int beds = 0; beds <= widest; ++beds) {   // every dwelling size, up to the biggest any type takes, fits one
        bool fits = false;
        for (const HouseholdType& h : households) fits = fits || (beds >= h.bedsLo && beds <= h.bedsHi);
        if (!fits) return "households: no type fits a dwelling of " + std::to_string(beds) + " bedrooms";
    }
    if (hallBand < 0 || hallBand >= static_cast<int>(bands.size())) return "halls: an unknown band";
    if (hallStage < 0 || hallStage >= static_cast<int>(stages.size()) || !stages[static_cast<std::size_t>(hallStage)].beds)
        return "halls: the stage must be one with beds";
    if (hallShare < 0 || hallShare > 1) return "halls: the share of walkers is 0..1";
    return {};
}

std::string PeopleCatalog::describe() const {
    std::ostringstream o;
    auto shares = [&](const std::vector<std::pair<int, double>>& v, bool ofBands) {
        for (const auto& [i, s] : v)
            o << " " << (ofBands ? bands[static_cast<std::size_t>(i)].name : stages[static_cast<std::size_t>(i)].name) << "=" << s;
    };
    for (const LifeStage& s : stages) o << "stage " << s.name << " job=" << s.job << " walks=" << s.walks << " beds=" << s.beds << "\n";
    for (const AgeBand& b : bands) {
        o << "band " << b.name << " " << b.lo << "-" << b.hi;
        shares(b.stages, false);
        o << "\n";
    }
    for (const HouseholdType& h : households) {
        o << "household " << h.name << " share=" << h.share << " beds=" << h.bedsLo << "-" << h.bedsHi;
        for (const MemberGroup& g : h.groups) {
            o << " [" << (g.perBed ? "per_bed " : "") << g.lo << "-" << g.hi;
            shares(g.bands, true);
            o << "]";
        }
        o << "\n";
    }
    o << "halls " << (hallBand >= 0 ? bands[static_cast<std::size_t>(hallBand)].name : "?") << " "
      << (hallStage >= 0 ? stages[static_cast<std::size_t>(hallStage)].name : "?") << " share=" << hallShare << "\n";
    return o.str();
}

PeopleCatalog defaultPeopleCatalog() {
    PeopleCatalog c;
    c.stages = {{"child", false, true, false},
                {"student", false, true, true},
                {"working", true, false, false},
                {"at_home", false, false, false},
                {"retired", false, false, false}};
    const int child = 0, working = 2, atHome = 3, retired = 4;
    c.bands = {{"child", 0, 17, {{child, 1.0}}},
               {"young", 18, 24, {{working, 0.6}, {atHome, 0.4}}},
               {"adult", 25, 64, {{working, 0.8}, {atHome, 0.2}}},
               {"senior", 65, 90, {{working, 0.15}, {retired, 0.85}}}};
    const int kid = 0, young = 1, adult = 2, senior = 3;
    auto group = [](int lo, int hi, std::vector<std::pair<int, double>> bands, bool perBed = false) {
        MemberGroup g;
        g.lo = lo; g.hi = hi; g.perBed = perBed; g.bands = std::move(bands);
        return g;
    };
    c.households = {
        {"single", 28, 0, 2, {group(1, 1, {{young, 0.25}, {adult, 0.75}})}},
        {"couple", 22, 0, 2, {group(2, 2, {{young, 0.1}, {adult, 0.9}})}},
        {"roommates", 8, 2, 4, {group(2, 4, {{young, 0.6}, {adult, 0.4}}, true)}},
        {"single_parent", 7, 1, 3, {group(1, 1, {{adult, 1.0}}), group(1, 2, {{kid, 1.0}})}},
        {"family", 20, 2, 5, {group(2, 2, {{adult, 1.0}}), group(1, 3, {{kid, 1.0}})}},
        {"three_generations", 3, 3, 5, {group(2, 2, {{adult, 1.0}}), group(1, 3, {{kid, 1.0}}), group(1, 2, {{senior, 1.0}})}},
        {"seniors", 12, 0, 3, {group(1, 2, {{senior, 1.0}})}},
    };
    c.hallBand = young;
    c.hallStage = 1;
    c.hallShare = 0.25;
    return c;
}

namespace {

uint32_t mix(uint32_t a, uint32_t b) {
    uint32_t h = a * 0x9E3779B9u ^ (b + 0x7F4A7C15u + (a << 6) + (a >> 2));
    h ^= h >> 16; h *= 0x7feb352dU;
    h ^= h >> 15; h *= 0x846ca68bU;
    h ^= h >> 16;
    return h;
}
double unit(uint32_t h) { return (static_cast<double>(h) + 0.5) / 4294967296.0; }
// The index a draw lands on in (index, share) pairs
int pick(const std::vector<std::pair<int, double>>& v, uint32_t h) {
    double total = 0;
    for (const auto& e : v) total += e.second;
    double u = unit(h) * total;
    for (const auto& e : v) {
        if (u < e.second) return e.first;
        u -= e.second;
    }
    return v.back().first;
}

// One household of type `t` for a dwelling of `beds` bedrooms, its members appended to `out`; `key` is its own bits.
void realize(const PeopleCatalog& cat, const HouseholdType& t, int beds, uint32_t key, std::vector<Resident>& out) {
    uint32_t salt = 1;
    for (const MemberGroup& g : t.groups) {
        const int bi = pick(g.bands, mix(key, salt++));
        const AgeBand& b = cat.bands[static_cast<std::size_t>(bi)];
        const int n = g.perBed ? std::clamp(beds, g.lo, g.hi)
                               : g.lo + static_cast<int>(mix(key, salt++) % static_cast<uint32_t>(g.hi - g.lo + 1));
        for (int k = 0; k < n; ++k) {
            Resident r;
            r.band = static_cast<uint8_t>(bi);
            r.age = static_cast<uint8_t>(b.lo + static_cast<int>(mix(key, salt++) % static_cast<uint32_t>(b.hi - b.lo + 1)));
            r.stage = static_cast<uint8_t>(pick(b.stages, mix(key, salt++)));
            r.walks = cat.stages[r.stage].walks;
            out.push_back(r);
        }
    }
}

}  // namespace

HouseholdPlan planHouseholds(const PeopleCatalog& cat, const PlaceMap& places, uint32_t bedsTag, int people, int walkers,
                             uint32_t seed) {
    HouseholdPlan plan;
    plan.asked = people;
    int peopleLeft = std::max(0, people), walkLeft = std::max(0, walkers);

    // THE HALLS' BEDS first, one student each, in proportion to each hall's beds
    std::vector<std::pair<PlaceId, int>> halls;
    if (bedsTag)
        for (const Place& p : places.places())
            if (p.tags & bedsTag) {
                halls.push_back({p.id, p.capacity > 0 ? p.capacity : 150});
                plan.hallBeds += halls.back().second;
            }
    plan.capacity += plan.hallBeds;
    const int students = std::min({plan.hallBeds, static_cast<int>(walkLeft * cat.hallShare), peopleLeft});
    if (students > 0) {
        std::vector<int> take(halls.size(), 0);
        int given = 0;
        for (std::size_t h = 0; h < halls.size(); ++h) given += take[h] = static_cast<int>(static_cast<long long>(halls[h].second) * students / plan.hallBeds);
        for (std::size_t h = 0; given < students; h = (h + 1) % halls.size())
            if (take[h] < halls[h].second) { ++take[h]; ++given; }
        const AgeBand& b = cat.bands[static_cast<std::size_t>(cat.hallBand)];
        for (std::size_t h = 0; h < halls.size(); ++h)
            for (int k = 0; k < take[h]; ++k) {
                Household hh;
                hh.type = Household::kHall;
                hh.place = halls[h].first;
                hh.first = static_cast<uint32_t>(plan.people.size());
                hh.count = 1;
                Resident r;
                r.band = static_cast<uint8_t>(cat.hallBand);
                r.stage = static_cast<uint8_t>(cat.hallStage);
                r.age = static_cast<uint8_t>(b.lo + static_cast<int>(mix(seed ^ 0x5EEDu, static_cast<uint32_t>(plan.people.size())) %
                                                                     static_cast<uint32_t>(b.hi - b.lo + 1)));
                r.walks = true;
                plan.people.push_back(r);
                plan.households.push_back(hh);
            }
        plan.hallFilled = students;
        peopleLeft -= students;
        walkLeft -= students;
        plan.walkersOnly += students;
    }

    // THE DWELLINGS, in an order of the seed's: each draws a household type that fits its bedrooms (by share); one
    // that would need more people, or more walkers, than are left tries the other fitting types, and failing all of
    // them the dwelling stays empty
    const std::vector<PlaceMap::Dwelling>& dw = places.dwellings();
    plan.dwellings = static_cast<int>(dw.size());
    int widest = 0;
    for (const HouseholdType& t : cat.households) widest = std::max(widest, t.bedsHi);
    std::vector<std::pair<uint32_t, uint32_t>> order(dw.size());
    for (std::size_t i = 0; i < dw.size(); ++i) order[i] = {mix(seed, static_cast<uint32_t>(i)), static_cast<uint32_t>(i)};
    std::sort(order.begin(), order.end());
    std::vector<std::pair<int, double>> fit;
    std::vector<Resident> drawn;
    for (const auto& [key, di] : order) {
        const int beds = std::min(static_cast<int>(dw[di].beds), widest);
        fit.clear();
        for (std::size_t t = 0; t < cat.households.size(); ++t) {
            const HouseholdType& ht = cat.households[t];
            if (beds >= ht.bedsLo && beds <= ht.bedsHi) fit.push_back({static_cast<int>(t), ht.share});
        }
        if (fit.empty()) continue;
        const int first = pick(fit, mix(key, 0xD1CEu));
        std::size_t at = 0;
        while (fit[at].first != first) ++at;
        bool housed = false;
        for (std::size_t k = 0; k < fit.size() && !housed; ++k) {
            const int t = fit[(at + k) % fit.size()].first;
            drawn.clear();
            realize(cat, cat.households[static_cast<std::size_t>(t)], beds, key, drawn);
            if (k == 0) plan.capacity += static_cast<int>(drawn.size());   // its household, had there been people enough
            int walk = 0;
            for (const Resident& r : drawn) walk += r.walks ? 1 : 0;
            if (static_cast<int>(drawn.size()) > peopleLeft || walk > walkLeft) continue;
            Household hh;
            hh.type = static_cast<uint16_t>(t);
            hh.place = dw[di].place;
            hh.dwelling = di;
            hh.first = static_cast<uint32_t>(plan.people.size());
            hh.count = static_cast<uint32_t>(drawn.size());
            plan.people.insert(plan.people.end(), drawn.begin(), drawn.end());
            plan.households.push_back(hh);
            peopleLeft -= static_cast<int>(drawn.size());
            walkLeft -= walk;
            plan.walkersOnly += walk;
            ++plan.dwellingsFilled;
            housed = true;
        }
    }
    return plan;
}

std::string describePlan(const PeopleCatalog& cat, const HouseholdPlan& plan) {
    std::vector<int> byType(cat.households.size(), 0), byBand(cat.bands.size(), 0), byStage(cat.stages.size(), 0);
    for (const Household& h : plan.households)
        if (h.type < byType.size()) ++byType[h.type];
    for (const Resident& r : plan.people) {
        if (r.band < byBand.size()) ++byBand[r.band];
        if (r.stage < byStage.size()) ++byStage[r.stage];
    }
    std::ostringstream o;
    o << plan.households.size() - static_cast<std::size_t>(plan.hallFilled) << " households (";
    for (std::size_t t = 0; t < byType.size(); ++t) o << (t ? ", " : "") << cat.households[t].name << " " << byType[t];
    o << "), " << plan.people.size() << " people (";
    for (std::size_t b = 0; b < byBand.size(); ++b) o << (b ? ", " : "") << cat.bands[b].name << " " << byBand[b];
    o << "; ";
    for (std::size_t s = 0; s < byStage.size(); ++s) o << (s ? ", " : "") << cat.stages[s].name << " " << byStage[s];
    o << "), dwellings " << plan.dwellingsFilled << " of " << plan.dwellings << " filled, hall beds " << plan.hallFilled
      << " of " << plan.hallBeds;
    const int housed = static_cast<int>(plan.people.size());
    if (plan.asked > housed) o << "; " << plan.asked - housed << " of the " << plan.asked << " asked for have no home";
    return o.str();
}

}  // namespace citysim
