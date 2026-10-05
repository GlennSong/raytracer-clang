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
        std::snprintf(b, sizeof b, " tags=%u hours=%g-%g minutes=%g-%g dist=%g-%g nearest=%d perSite=%d walkers=%d shift=%d own=%d perform=%d\n",
                      d.tags, d.hourLo, d.hourHi, d.minutesLo, d.minutesHi, d.distLo, d.distHi, d.nearest, d.perSite ? 1 : 0,
                      d.walkersOnly ? 1 : 0, d.inShift ? 1 : 0, d.bringOwnEighths, static_cast<int>(d.perform));
        out += b;
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

    Menu outing;
    outing.name = "outing";
    {
        MenuBand b;
        b.first = {{"bench", 0.30, 0}, {"across_town", 0.14, 0}};
        b.pick = {{"park_visit", 0, 3.0}, {"coffee", 0, 2.5}, {"browse", 0, 2.0}, {"groceries", 0, 1.0},
                  {"meal", 0, 1.5}, {"civic_visit", 0, 0.7}, {"walk_round_block", 0, 3.0}};
        outing.bands.push_back(b);
    }
    c.menus.push_back(outing);
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
    { MenuBand b; b.hourLo = 15.0; b.hourHi = 19.5; b.first = {{"jog", 0.25, 0}, {"campus_bench", 0.667, 0}};
      b.pick = {{"study", 0, 2.0}, {"quad_time", 0, 1.0}}; brk.bands.push_back(b); }
    { MenuBand b; b.first = {{"campus_bench", 0.75, 0}}; b.pick = {{"study", 0, 2.0}, {"quad_time", 0, 1.0}}; brk.bands.push_back(b); }
    c.menus.push_back(brk);
    return c;
}

}  // namespace citysim
