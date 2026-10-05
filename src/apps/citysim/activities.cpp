#include "activities.h"

namespace citysim {

const char* spotKindName(SpotKind k) {
    switch (k) {
        case SpotKind::Sit: return "sit";
        case SpotKind::Lie: return "lie";
        case SpotKind::Stand: return "stand";
        case SpotKind::Jog: return "jog";
        case SpotKind::Play: return "play";
        case SpotKind::Watch: return "watch";
        default: return "?";
    }
}

bool spotKindFromName(const std::string& name, SpotKind& out) {
    for (int i = 0; i < static_cast<int>(SpotKind::Count); ++i)
        if (name == spotKindName(static_cast<SpotKind>(i))) { out = static_cast<SpotKind>(i); return true; }
    return false;
}

uint32_t spotTagFromName(const std::string& name) {
    if (name == "campus") return spot_tag::kCampus;
    if (name == "park") return spot_tag::kPark;
    if (name == "sports") return spot_tag::kSports;
    return 0;
}

}  // namespace citysim

#include <cstdio>

namespace citysim {

int ActivityCatalog::find(const std::string& name) const {
    for (std::size_t i = 0; i < defs.size(); ++i)
        if (defs[i].name == name) return static_cast<int>(i);
    return -1;
}

const Menu* ActivityCatalog::menu(const std::string& name) const {
    for (const Menu& m : menus)
        if (m.name == name) return &m;
    return nullptr;
}

std::string ActivityCatalog::describe() const {
    std::string out;
    char b[256];
    for (const ActivityDef& d : defs) {
        out += "activity " + d.name + " sites";
        for (const std::string& s : d.sites) out += " " + s;
        std::snprintf(b, sizeof b, " tags=%u hours=%g-%g minutes=%g-%g dist=%g-%g nearest=%d perSite=%d walkers=%d shift=%d own=%d home=%d perform=%d\n",
                      d.tags, d.hourLo, d.hourHi, d.minutesLo, d.minutesHi, d.distLo, d.distHi, d.nearest, d.perSite ? 1 : 0,
                      d.walkersOnly ? 1 : 0, d.inShift ? 1 : 0, d.bringOwnEighths, d.fromHome ? 1 : 0, static_cast<int>(d.perform));
        out += b;
        for (const RoleDef& r : d.roles) {
            std::snprintf(b, sizeof b, "  role %s n=%d speed=%g-%g zone=%d\n", r.name.c_str(), r.n, r.speedLo, r.speedHi, r.zone);
            out += b;
        }
        if (!d.roles.empty() || !d.during.empty()) {
            std::snprintf(b, sizeof b, "  group min=%d gather=%g swap=%g during=%s\n", d.minPlayers, d.gatherMinutes, d.swapAt, d.during.c_str());
            out += b;
        }
    }
    for (const Menu& m : menus) {
        out += "menu " + m.name + "\n";
        for (const MenuBand& band : m.bands) {
            std::snprintf(b, sizeof b, "  band %g-%g:", band.hourLo, band.hourHi);
            out += b;
            for (const MenuEntry& e : band.first) { std::snprintf(b, sizeof b, " first %s %g", e.activity.c_str(), e.chance); out += b; }
            for (const MenuEntry& e : band.pick) { std::snprintf(b, sizeof b, " pick %s %g", e.activity.c_str(), e.weight); out += b; }
            out += "\n";
        }
    }
    return out;
}

void siteKindMinutes(const std::string& k, double& lo, double& hi) {
    // what each kind of place keeps a visitor (the old arrival switch, in minutes)
    if (k == "cafe") { lo = 15; hi = 30; }
    else if (k == "restaurant") { lo = 30; hi = 60; }
    else if (k == "supermarket") { lo = 12; hi = 24; }
    else if (k == "civic" || k == "teaching") { lo = 18; hi = 36; }
    else if (k == "park" || k == "field") { lo = 6; hi = 15; }
    else if (k == "library") { lo = 30; hi = 72; }
    else if (k == "quad") { lo = 6; hi = 18; }
    else if (k == "seat" || k == "bed" || k == "stand" || k == "watch") { lo = 4.8; hi = 15; }
    else if (k == "loop") { lo = 19.8; hi = 34.8; }
    else if (k == "street") { lo = 0.6; hi = 2.4; }
    else if (k == "pitch") { lo = 30; hi = 45; }
    else { lo = 6; hi = 18; }   // a shop, anything else
}

ActivityCatalog defaultActivityCatalog() {
    ActivityCatalog c;
    auto def = [&](const char* name, std::vector<std::string> sites) -> ActivityDef& {
        ActivityDef d;
        d.name = name;
        d.sites = std::move(sites);
        c.defs.push_back(d);
        return c.defs.back();
    };
    // A DAY OFF'S STOPS (the old pickOuting): a sit on a bench, a trip across town, the local places by kind (each
    // place counts: three cafes, three times the chance of a coffee), or a walk round the block.
    { ActivityDef& d = def("bench", {"seat"}); d.distLo = 30; d.distHi = 500; d.nearest = 4; d.perform = Perform::Spot; }
    { ActivityDef& d = def("across_town", {"park", "quad", "field", "civic", "library", "teaching", "restaurant", "cafe"});
      d.distLo = 1200; d.distHi = 3000; }
    { ActivityDef& d = def("park_visit", {"park", "quad", "field"}); d.distLo = 60; d.perSite = true; d.perform = Perform::Outside; }
    { ActivityDef& d = def("coffee", {"cafe"}); d.distLo = 60; d.perSite = true; }
    { ActivityDef& d = def("browse", {"shop"}); d.distLo = 60; d.perSite = true; }
    { ActivityDef& d = def("groceries", {"supermarket"}); d.distLo = 60; d.perSite = true; }
    { ActivityDef& d = def("meal", {"restaurant"}); d.distLo = 60; d.perSite = true; d.hourLo = 11.5; d.hourHi = 21.5; }
    { ActivityDef& d = def("civic_visit", {"civic", "library", "teaching"}); d.distLo = 60; d.perSite = true; }
    { ActivityDef& d = def("walk_round_block", {"street"}); d.distLo = 150; d.distHi = 450; d.perform = Perform::Wander; }
    // LUNCH OUT (the old pickLunch): walkers in their shift, three in eight brought theirs, one of the four nearest
    // open cafes or restaurants within 600 m
    { ActivityDef& d = def("lunch", {"cafe", "restaurant"}); d.distHi = 600; d.nearest = 4; d.walkersOnly = true;
      d.inShift = true; d.bringOwnEighths = 3; }
    // A STUDENT'S BREAK (the old pickCampusBreak): a campus bench (the quad's, the bleachers), a run round the track,
    // the library, the quad
    { ActivityDef& d = def("campus_bench", {"seat"}); d.tags = spot_tag::kCampus; d.distHi = 600; d.nearest = 6;
      d.perform = Perform::Spot; }
    { ActivityDef& d = def("jog", {"loop"}); d.tags = spot_tag::kCampus; d.distHi = 900; d.nearest = 6; d.perform = Perform::Spot; }
    { ActivityDef& d = def("study", {"library"}); d.distHi = 1e9; d.nearest = 1; }
    { ActivityDef& d = def("quad_time", {"quad"}); d.distHi = 1e9; d.nearest = 1; d.perform = Perform::Outside; }
    // A KICKABOUT on the campus pitch: two sides of five, each in its half, swapping ends at half time; it starts with
    // six there, and is given up if it has not after twenty minutes
    { ActivityDef& d = def("kickabout", {"pitch"}); d.tags = spot_tag::kCampus; d.distHi = 900; d.nearest = 3;
      d.perform = Perform::Roam; d.minutesLo = 30; d.minutesHi = 45; d.minPlayers = 6; d.gatherMinutes = 20; d.swapAt = 0.5;
      RoleDef home; home.name = "home"; home.n = 5; home.speedLo = 1.5; home.speedHi = 4.0; home.zone = 0;
      RoleDef away = home; away.name = "away"; away.zone = 1;
      d.roles = {home, away}; }
    // ...and watching it from the stand
    { ActivityDef& d = def("watch_game", {"seat"}); d.tags = spot_tag::kSports; d.distHi = 900; d.nearest = 6;
      d.perform = Perform::Spot; d.minutesLo = 15; d.minutesHi = 40; d.during = "kickabout"; }

    // THE ERRAND on the way home (the old fixed "shop near home"): a supermarket or a store near HOME, one of the three
    // nearest open
    { ActivityDef& d = def("errand", {"supermarket", "shop"}); d.fromHome = true; d.distHi = 700; d.nearest = 3; }

    // A DAY OFF, by the hour: coffee and the park in the morning; the shops and lunch at midday; errands and the park
    // in the afternoon; dinner and a coffee in the evening -- a bench or a trip across town now and then all day
    Menu outing;
    outing.name = "outing";
    auto band = [&](double lo, double hi, double bench, double across, std::vector<MenuEntry> pick) {
        MenuBand b;
        b.hourLo = lo; b.hourHi = hi;
        b.first = {{"bench", bench, 0}, {"across_town", across, 0}};
        b.pick = std::move(pick);
        outing.bands.push_back(b);
    };
    band(5.0, 10.5, 0.20, 0.10, {{"coffee", 0, 4.0}, {"park_visit", 0, 3.0}, {"groceries", 0, 0.5}, {"civic_visit", 0, 0.3},
                                {"walk_round_block", 0, 3.0}});
    band(10.5, 14.0, 0.25, 0.14, {{"browse", 0, 2.5}, {"meal", 0, 2.0}, {"coffee", 0, 2.0}, {"park_visit", 0, 3.0},
                                 {"civic_visit", 0, 0.7}, {"walk_round_block", 0, 2.0}});
    band(14.0, 17.5, 0.30, 0.14, {{"browse", 0, 2.5}, {"groceries", 0, 1.5}, {"park_visit", 0, 3.0}, {"coffee", 0, 1.5},
                                 {"civic_visit", 0, 0.7}, {"walk_round_block", 0, 3.0}});
    band(17.5, 23.0, 0.15, 0.10, {{"meal", 0, 3.0}, {"coffee", 0, 1.0}, {"park_visit", 0, 1.5}, {"groceries", 0, 1.0},
                                 {"walk_round_block", 0, 2.0}});
    band(23.0, 5.0, 0.0, 0.0, {{"walk_round_block", 0, 1.0}, {"park_visit", 0, 0.5}});
    c.menus.push_back(outing);
    Menu errand;
    errand.name = "errand";
    { MenuBand b; b.first = {{"errand", 1.0, 0}}; errand.bands.push_back(b); }
    c.menus.push_back(errand);
    Menu lunch;
    lunch.name = "lunch";
    { MenuBand b; b.first = {{"lunch", 1.0, 0}}; lunch.bands.push_back(b); }
    c.menus.push_back(lunch);
    // the break: lunch out at midday, a run in the late afternoon, else most often a bench, else the library (two in
    // three) or the quad -- the old single roll's thresholds as chances tried in order
    Menu brk;
    brk.name = "student_break";
    { MenuBand b; b.hourLo = 11.5; b.hourHi = 13.5; b.first = {{"lunch", 0.45, 0}, {"campus_bench", 0.55, 0}};
      b.pick = {{"study", 0, 2.0}, {"quad_time", 0, 1.0}}; brk.bands.push_back(b); }
    { MenuBand b; b.hourLo = 15.0; b.hourHi = 19.5;
      b.first = {{"kickabout", 0.2, 0}, {"watch_game", 0.15, 0}, {"jog", 0.25, 0}, {"campus_bench", 0.667, 0}};
      b.pick = {{"study", 0, 2.0}, {"quad_time", 0, 1.0}}; brk.bands.push_back(b); }
    { MenuBand b; b.first = {{"campus_bench", 0.75, 0}}; b.pick = {{"study", 0, 2.0}, {"quad_time", 0, 1.0}}; brk.bands.push_back(b); }
    c.menus.push_back(brk);
    return c;
}

}  // namespace citysim
