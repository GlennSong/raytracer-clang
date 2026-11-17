#include "city_crowd.h"

#include "../../engine/procgen/city/roads/road_model.h"
#include "../../proc_rng.h"

#include <algorithm>
#include <cmath>

namespace citysim {

namespace {

// A seeded pick from a list of shares: the index of the one drawn (-1: the list is empty or has no share).
int pickShare(engine::ProcRng& rng, const std::vector<Real>& shares) {
    Real sum = 0;
    for (const Real s : shares) sum += std::max(Real(0), s);
    if (!(sum > 0)) return -1;
    const Real r = rng.next() * sum;
    Real at = 0;
    for (std::size_t i = 0; i < shares.size(); ++i) {
        at += std::max(Real(0), shares[i]);
        if (r < at) return static_cast<int>(i);
    }
    return static_cast<int>(shares.size()) - 1;
}

int pickValue(engine::ProcRng& rng, const std::vector<CrowdShare>& list, int otherwise) {
    std::vector<Real> shares;
    for (const CrowdShare& c : list) shares.push_back(c.share);
    const int i = pickShare(rng, shares);
    return i >= 0 ? list[static_cast<std::size_t>(i)].value : otherwise;
}

// Draws for a place people cannot walk to, before it is put at its street's middle.
constexpr int kStandTries = 8;

// A street a place can stand beside: it has a sidewalk, and room for a place `keep` from both its ends.
struct Run {
    uint32_t street;
    Real from, length;   // stations along the road's spine where places may go
};

}  // namespace

bool makeCrowd(const CrowdSpec& spec, const engine::RoadModel& model, const CrowdUsable& usable, uint32_t seed, CrowdMade& out,
               std::string* why) {
    const auto refuse = [&](const std::string& reason) {
        if (why) *why = "crowd: " + reason;
        return false;
    };
    out = CrowdMade();
    engine::ProcRng rng(seed, 0xC20D5EEDull);

    // THE HOUSEHOLDS' SIZES, until every resident has one
    std::vector<int> sizes;
    for (int left = spec.residents; left > 0;) {
        const int size = std::clamp(pickValue(rng, spec.householdSizes, 1), 1, left);
        sizes.push_back(size);
        left -= size;
    }

    // HOW MANY PLACES OF EACH KIND: the homes' kind one per household
    int homeKind = -1;
    std::vector<int> kindOfPlace;
    for (std::size_t k = 0; k < spec.places.size(); ++k) {
        const CrowdPlaceKind& pk = spec.places[k];
        if (pk.homes && homeKind >= 0) return refuse("two kinds of place are homes (`" + spec.places[static_cast<std::size_t>(homeKind)].kind +
                                                     "` and `" + pk.kind + "`)");
        if (pk.homes) homeKind = static_cast<int>(k);
        const int n = pk.homes ? static_cast<int>(sizes.size()) : pk.count;
        for (int i = 0; i < n; ++i) kindOfPlace.push_back(static_cast<int>(k));
        out.placesOfKind.push_back({pk.kind, n});
    }
    if (homeKind < 0) return refuse("no kind of place is marked `homes = true`");
    if (kindOfPlace.empty()) return refuse("no places");
    // ...dealt to the places in a seeded order (Fisher-Yates, one draw a place)
    for (std::size_t i = kindOfPlace.size(); i > 1; --i) std::swap(kindOfPlace[i - 1], kindOfPlace[static_cast<std::size_t>(rng.below(static_cast<int>(i)))]);

    // WHERE ON A STREET A PERSON STANDS, beside `station` on the kerb `dir`: the road model's sidewalk there, and the
    // caller's word that people can walk to it
    const auto standing = [&](uint32_t street, Real station, int dir, bool cars, engine::Vec2& stand) {
        const engine::RoadFrontage f = model.frontageOn(engine::ModelStreetId(street), station, dir);
        return f.valid() && model.sidewalkBeside(f, stand) && (!usable || usable(stand, cars));
    };
    // THE STREETS THE PLACES ARE SPREAD ALONG, end to end: those with a sidewalk, room for a place, and people able to
    // walk to their middle
    std::vector<Run> runs;
    Real total = 0;
    for (uint32_t s = 0; s < model.streets.size(); ++s) {
        const engine::ModelStreet& S = model.streets[s];
        if (!(S.section.sidewalk > 0)) continue;
        const Real length = (S.to - S.from) - 2 * spec.keepFromEnds;
        engine::Vec2 middle;
        if (!(length > 0) || !standing(s, S.from + spec.keepFromEnds + length * Real(0.5), 1, false, middle)) continue;
        runs.push_back(Run{s, S.from + spec.keepFromEnds, length});
        total += length;
    }
    if (runs.empty() || !(total > 0)) return refuse("no street has a sidewalk, room for a place and a way there on foot");

    // ONE PLACE A PIECE: a seeded way along its piece of the line and a seeded side, asked of the road model
    const std::size_t count = kindOfPlace.size();
    const Real piece = total / static_cast<Real>(count);
    std::vector<int> placesOf(spec.places.size(), 0);
    std::vector<std::vector<std::string>> namesOf(spec.places.size());
    std::vector<std::vector<engine::Vec2>> standsOf(spec.places.size());   // where a person stands at each
    // where along the line `at` is: a street and a station on it
    std::vector<Real> runBefore;
    {
        Real sum = 0;
        for (const Run& r : runs) {
            runBefore.push_back(sum);
            sum += r.length;
        }
    }
    const auto onLine = [&](Real at, uint32_t& street, Real& station) {
        at = std::fmod(std::max(Real(0), at), total);
        const std::size_t r = static_cast<std::size_t>(std::upper_bound(runBefore.begin(), runBefore.end(), at) - runBefore.begin()) - 1;
        street = runs[r].street;
        station = runs[r].from + std::clamp(at - runBefore[r], Real(0), runs[r].length);
    };
    for (std::size_t i = 0; i < count; ++i) {
        const int k = kindOfPlace[i];
        // (households' cars start beside a home; a kind people drive to says so)
        const bool cars = spec.places[static_cast<std::size_t>(k)].homes || spec.places[static_cast<std::size_t>(k)].parking;
        const Real way = rng.next();
        const Real sideDraw = rng.next();
        int dir = sideDraw < 0.5 ? 1 : -1;
        uint32_t street = 0;
        Real station = 0;
        onLine((static_cast<Real>(i) + way) * piece, street, station);
        engine::Vec2 stand;
        // (a point the caller refuses: drawn again along its piece, then looked for along the rest of the line, a
        // quarter piece at a time, both sides)
        bool found = standing(street, station, dir, cars, stand);
        for (int again = 0; again < kStandTries && !found; ++again) {
            const Real redraw = rng.next();
            onLine((static_cast<Real>(i) + redraw) * piece, street, station);
            found = standing(street, station, dir, cars, stand);
        }
        for (Real step = 0; !found && step < total; step += piece * Real(0.25))
            for (const int side : {1, -1}) {
                if (found) break;
                onLine((static_cast<Real>(i) + way) * piece + step, street, station);
                dir = side;
                found = standing(street, station, dir, cars, stand);
            }
        if (!found) return refuse("nowhere along the streets is a place for a " + spec.places[static_cast<std::size_t>(k)].kind);
        engine::CityMarker m;
        m.name = spec.places[static_cast<std::size_t>(k)].kind + "_" + std::to_string(placesOf[static_cast<std::size_t>(k)]++);
        // (a home is its own household's: another home is not the same kind of place to them, so it has no kind a
        // person could swap it for, CityMarker::kind)
        if (!spec.places[static_cast<std::size_t>(k)].homes) m.kind = spec.places[static_cast<std::size_t>(k)].kind;
        m.stated = true;
        m.street = model.streets[street].id;
        m.station = station;
        m.dir = dir;
        namesOf[static_cast<std::size_t>(k)].push_back(m.name);
        standsOf[static_cast<std::size_t>(k)].push_back(stand);
        out.markers.push_back(m);
    }
    const auto kindNamed = [&](const std::string& name) {
        for (std::size_t k = 0; k < spec.places.size(); ++k)
            if (spec.places[k].kind == name) return static_cast<int>(k);
        return -1;
    };
    for (const CrowdDay& d : spec.days)
        for (const CrowdStop& s : d.stops)
            if (!s.home && (kindNamed(s.kind) < 0 || namesOf[static_cast<std::size_t>(kindNamed(s.kind))].empty()))
                return refuse("a day stops at `" + s.kind + "`, and there are no places of that kind");

    // THE HOUSEHOLDS AND THEIR PEOPLE: each at a home place of its own, its cars near it; each person a day
    std::vector<Real> dayShares;
    for (const CrowdDay& d : spec.days) dayShares.push_back(d.share);
    int who = 0;
    for (std::size_t h = 0; h < sizes.size(); ++h) {
        engine::CityHousehold hh;
        hh.name = "h" + std::to_string(h);
        const std::string home = namesOf[static_cast<std::size_t>(homeKind)][h];
        const engine::Vec2 homeAt = standsOf[static_cast<std::size_t>(homeKind)][h];
        const int cars = std::clamp(pickValue(rng, spec.carsPerHousehold, 0), 0, sizes[h]);
        for (int c = 0; c < cars; ++c) {
            engine::CityHousehold::Car car;
            car.near = home;
            hh.cars.push_back(car);
        }
        out.cars += cars;
        for (int m = 0; m < sizes[h]; ++m) {
            engine::CityResident r;
            r.name = "r" + std::to_string(who++);
            r.home = home;
            r.intent = spec.intent;
            for (const CrowdTrait& t : spec.traits)
                if (rng.next() < t.share) r.traits.push_back(t.name);
            const int day = pickShare(rng, dayShares);
            if (day >= 0)
                for (const CrowdStop& s : spec.days[static_cast<std::size_t>(day)].stops) {
                    engine::CityResident::Stop stop;
                    if (s.home) {
                        stop.at = home;
                    } else {
                        // one of the kind, picked at random among those `beyond` from home (none: the farthest)
                        const std::size_t k = static_cast<std::size_t>(kindNamed(s.kind));
                        std::vector<std::size_t> far;
                        std::size_t farthest = 0;
                        for (std::size_t p = 0; p < namesOf[k].size(); ++p) {
                            const Real away = (standsOf[k][p] - homeAt).length();
                            if (engine::atOrPast(away, s.beyond, engine::kSamePoint)) far.push_back(p);
                            if (engine::strictlyPast(away, (standsOf[k][farthest] - homeAt).length(), engine::kSamePoint)) farthest = p;
                        }
                        const std::size_t pick = far.empty() ? farthest : far[static_cast<std::size_t>(rng.below(static_cast<int>(far.size())))];
                        stop.at = namesOf[k][pick];
                    }
                    stop.minutes = rng.in(s.minutesLo, s.minutesHi);
                    r.day.push_back(stop);
                }
            hh.members.push_back(r.name);
            out.residents.push_back(r);
        }
        out.households.push_back(hh);
    }
    return true;
}

}  // namespace citysim
