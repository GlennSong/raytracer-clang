#include "brain_people.h"

#include "city_brain.h"   // kMarkerSpots
#include "city_sim.h"
#include "households.h"
#include "places.h"

#include "../../engine/procgen/city/roads/road_model.h"
#include "../../proc_rng.h"

#include <algorithm>
#include <map>
#include <set>
#include <tuple>
#include <unordered_map>

namespace citysim {

namespace {

// A seeded pick from (value, share) rows: the value drawn (`otherwise`: no row has a share).
int pickValue(engine::ProcRng& rng, const std::vector<std::pair<int, double>>& rows, int otherwise) {
    double sum = 0;
    for (const std::pair<int, double>& r : rows) sum += std::max(0.0, r.second);
    if (!(sum > 0)) return otherwise;
    const double at = rng.next() * sum;
    double run = 0;
    for (const std::pair<int, double>& r : rows) {
        run += std::max(0.0, r.second);
        if (at < run) return r.first;
    }
    return rows.back().first;
}

// One place's marker, made once: where its door fronts the road, the point a person stands at, and how many people
// use it so far.
struct PlaceMarker {
    bool tried = false, resolved = false, walkable = false;
    engine::RoadFrontage frontage;
    engine::CityMarker marker;
    std::size_t users = 0;
};

}  // namespace

std::string BrainPeople::describe() const {
    return std::to_string(residents.size()) + " people in " + std::to_string(households.size()) + " households, " +
           std::to_string(cars) + " cars, " + std::to_string(workers) + " with a job to go to, " +
           std::to_string(markers.size()) + " homes (" + std::to_string(homesCutOff) +
           " households in a home cut off on foot); left out: " + std::to_string(homesUnusable) +
           " households whose home has no street beside it, " + std::to_string(carsNoSpace) + " cars with no kerb space";
}

engine::CityMarker placeMarkerSpec(const Place& p, PlaceId id, const engine::RoadModel& model) {
    engine::CityMarker m;
    m.name = placeMarkerName(id);
    // (a home or a job is the person's own: no other place of a kind is the same to them, CityMarker::kind)
    bool known = false;
    if (p.frontage.onStreet())
        for (const engine::ModelStreet& s : model.streets)
            if (s.id == p.frontage.street) { known = true; break; }
    if (known) {
        m.stated = true;
        m.street = p.frontage.street;
        m.station = p.frontage.station;
        m.dir = p.frontage.dir;
    } else {
        m.x = p.entrance.x;
        m.z = p.entrance.y;
    }
    return m;
}

std::string placeMarkerName(PlaceId id) { return "place_" + std::to_string(id); }

bool brainPeopleFromHouseholds(const CitySim& sim, const PlaceMap& places, const engine::RoadModel& model, int most,
                               const BrainPlaceUsable& usable, const BrainCarSpaces& spaces, uint32_t seed,
                               BrainPeople& out, std::string* why) {
    out = BrainPeople{};
    const PeopleCatalog& people = sim.people();
    const BrainPeopleRules& rules = people.brain;
    if (!rules.given) {
        if (why) *why = "people.lua has no `people.brain`: what the brain's people own and do";
        return false;
    }
    engine::ProcRng rng(seed, 0xB4A1F0E5ull);

    // EACH HOUSEHOLD'S MEMBERS, the sim's agents in their order
    const HouseholdPlan& plan = sim.households();
    std::vector<std::vector<int>> members(plan.households.size());
    const std::vector<Agent>& agents = sim.agents();
    for (std::size_t i = 0; i < agents.size(); ++i) {
        const int h = agents[i].household;
        if (h >= 0 && h < static_cast<int>(members.size())) members[static_cast<std::size_t>(h)].push_back(static_cast<int>(i));
    }

    // THE STREETS BY ID (a place's door states its street by the road file's id)
    std::unordered_map<std::string, uint32_t> streetOf;
    for (uint32_t s = 0; s < model.streets.size(); ++s) streetOf.emplace(model.streets[s].id, s);
    std::vector<PlaceMarker> placeMarker(places.places().size());
    const auto markerOf = [&](PlaceId id) -> PlaceMarker& {
        PlaceMarker& pm = placeMarker[static_cast<std::size_t>(id)];
        if (pm.tried) return pm;
        pm.tried = true;
        const Place& p = places[id];
        engine::CityMarker& m = pm.marker;
        m = placeMarkerSpec(p, id, model);
        if (m.stated) {
            const std::unordered_map<std::string, uint32_t>::const_iterator s = streetOf.find(m.street);
            pm.frontage = model.frontageOn(engine::ModelStreetId(s->second), m.station, m.dir);
        } else {
            pm.frontage = model.frontageAt(Vec2(m.x, m.z));
        }
        engine::Vec2 stand;
        if (!pm.frontage.valid() || !pm.frontage.street.valid() || !model.sidewalkBeside(pm.frontage, stand)) return pm;
        pm.resolved = true;
        pm.walkable = !usable || usable(stand, pm.frontage);
        return pm;
    };
    const auto use = [&](PlaceMarker& pm, std::size_t people) {
        if (pm.users == 0) out.markers.push_back(pm.marker);
        pm.users += people;
    };

    // THE HOUSEHOLDS, whole, in a seeded order, until `most` people. (Not the plan's own order: it fills the halls' beds
    // first, so its head is all students.)
    std::vector<std::size_t> order(plan.households.size());
    for (std::size_t h = 0; h < order.size(); ++h) order[h] = h;
    for (std::size_t i = order.size(); i > 1; --i) std::swap(order[i - 1], order[static_cast<std::size_t>(rng.below(static_cast<int>(i)))]);
    using SpaceKey = std::tuple<uint32_t, int, uint32_t>;
    std::set<SpaceKey> heldSpaces;
    std::vector<engine::KerbSpace> holdingSpaces;
    for (const std::size_t h : order) {
        const Household& hh = plan.households[h];
        const std::vector<int>& who = members[h];
        if (who.empty() || hh.place == kNoPlace) continue;
        if (static_cast<int>(out.residents.size() + who.size()) > most) continue;   // (a smaller one may still fit)
        // the draws for this household come first, whether it is taken or not, so one left out moves nobody else's
        int drivers = 0;
        for (const int a : who) {
            const int st = agents[static_cast<std::size_t>(a)].stage;
            drivers += st >= 0 && !people.stages[static_cast<std::size_t>(st)].walks ? 1 : 0;
        }
        const int cars = std::clamp(pickValue(rng, rules.cars, 0), 0, drivers);
        PlaceMarker& home = markerOf(hh.place);
        if (!home.resolved) {   // (no street beside its door: the brain has nowhere to put them)
            ++out.homesUnusable;
            continue;
        }
        if (!home.walkable) ++out.homesCutOff;   // (they live there all the same; their trips find no way)
        // ITS CARS, each holding a free kerb space near home (fewer when there are not enough)
        std::vector<SpaceKey> holding;
        holdingSpaces.clear();
        if (cars > 0 && spaces)
            for (const engine::KerbSpace& s : spaces(home.frontage)) {
                const SpaceKey key{s.street.index, s.dir, s.index};
                if (heldSpaces.count(key) > 0) continue;
                holding.push_back(key);
                holdingSpaces.push_back(s);
                if (static_cast<int>(holding.size()) >= cars) break;
            }
        out.carsNoSpace += cars - static_cast<int>(holding.size());
        heldSpaces.insert(holding.begin(), holding.end());
        use(home, who.size());
        engine::CityHousehold made;
        made.name = "h" + std::to_string(h);
        // (each car names the space it holds, so the brain stands it in exactly that one)
        for (const engine::KerbSpace& sp : holdingSpaces)
            made.cars.push_back(engine::CityHousehold::Car{home.marker.name, model.kerbSpaceName(sp)});
        out.cars += static_cast<int>(holding.size());
        // ITS PEOPLE, each the agent who is them
        for (const int a : who) {
            const Agent& agent = agents[static_cast<std::size_t>(a)];
            engine::CityResident r;
            r.name = "r" + std::to_string(a);
            r.home = home.marker.name;
            r.intent = rules.intent;
            r.drives = agent.stage < 0 || !people.stages[static_cast<std::size_t>(agent.stage)].walks;
            for (const std::pair<std::string, double>& t : rules.traits)
                if (rng.next() < t.second) r.traits.push_back(t.first);
            // THEIR DAY is their role's, run by the brain on the sim's clock (ADR-0491)
            r.roleDay = true;
            if (agent.workPlace != kNoPlace && agent.workPlace != hh.place) ++out.workers;
            made.members.push_back(r.name);
            out.residents.push_back(r);
            out.agents.push_back(a);
        }
        out.households.push_back(made);
    }
    return true;
}

}  // namespace citysim
