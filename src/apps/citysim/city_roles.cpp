#include "city_roles.h"

#include <cstdio>

namespace citysim {

namespace {

const char* activityLabel(Activity a) {
    switch (a) {
        case Activity::Commuting: return "Commuting";
        case Activity::AtWork: return "AtWork";
        case Activity::Returning: return "Returning";
        case Activity::Shopping: return "Shopping";
        case Activity::Outing: return "Outing";
        case Activity::Lunch: return "Lunch";
        default: return "AtHome";
    }
}

const char* targetLabel(GoalTarget t) {
    switch (t) {
        case GoalTarget::Lunch: return "lunch";
        case GoalTarget::Campus: return "campus";
        case GoalTarget::Activity: return "activity";
        default: return "none";
    }
}

// A NIGHT OUT (Glenn, 2026-10-06: "we would want restaurants and clubs and such for a night life"): from home when
// its evening begins (GoalEvent::Evening), stop to stop off `menu` -- dinner, a bar, a club -- and home when the
// evening is over. Appended after the day's own states, so a clock jump still seats an agent on the first state
// wearing a label (ReturnHome before HomeTonight).
void addNightOut(GoalTable& t, const std::string& menu) {
    // the first stop leaves FROM HOME: nowhere to go is simply a night in (no trip home from the doorstep -- that
    // failed trip re-posed a stranded agent 21 m, agents_never_teleport); a later stop with nowhere to go goes home
    t.addState("GoOut", GoalAction::GoTo, GoalTarget::Activity, Activity::Outing);
    t.setMenu("GoOut", menu);
    t.addState("OutTonight", GoalAction::Rest, GoalTarget::None, Activity::Outing, 0.05);   // the stop sets its own
    t.addState("NextStop", GoalAction::GoTo, GoalTarget::Activity, Activity::Outing);
    t.setMenu("NextStop", menu);
    t.addState("HomeTonight", GoalAction::GoTo, GoalTarget::Home, Activity::Returning);
    t.addTransition("AtHome", GoalEvent::Evening, "GoOut");
    t.addTransition("GoOut", GoalEvent::Arrived, "OutTonight");
    t.addTransition("GoOut", GoalEvent::NoRoute, "AtHome");
    // the evening ends -> home (checked before the pause, as an outing's window is)
    t.addTransition("OutTonight", GoalEvent::EveningOver, "HomeTonight");
    t.addTransition("OutTonight", GoalEvent::DwellDone, "NextStop");
    t.addTransition("NextStop", GoalEvent::Arrived, "OutTonight");
    t.addTransition("NextStop", GoalEvent::NoRoute, "HomeTonight");
    t.addTransition("HomeTonight", GoalEvent::Arrived, "AtHome");
    t.addTransition("HomeTonight", GoalEvent::NoRoute, "AtHome");
}

}  // namespace

int RoleCatalog::find(const std::string& name) const {
    for (std::size_t i = 0; i < roles.size(); ++i)
        if (roles[i].name == name) return static_cast<int>(i);
    return -1;
}

const ResidentRole* RoleCatalog::role(const std::string& name) const {
    const int i = find(name);
    return i >= 0 ? &roles[static_cast<std::size_t>(i)] : nullptr;
}

std::string RoleCatalog::describe() const {
    std::string out;
    char b[512];
    for (const ResidentRole& r : roles) {
        const RoleDay& d = r.day;
        std::snprintf(b, sizeof b,
                      "role %s commute=%d pause=%g work=%g break=%s/%s/%s/%g repeat=%d errand=%d evening=%s share=%g\n",
                      r.name.c_str(), d.commute ? 1 : 0, d.outingPause, d.workBeforeBreak, targetLabel(d.breakTarget),
                      d.breakMenu.c_str(), activityLabel(d.breakActivity), d.breakHours, d.breaksRepeat ? 1 : 0,
                      d.errand ? 1 : 0, d.eveningMenu.c_str(), r.nightOutShare);
        out += b;
        if (!r.staff.places.empty()) {
            out += "  staff";
            for (const std::string& k : r.staff.places) out += " " + k;
            std::snprintf(b, sizeof b, " per=%d placeHours=%d start=%g-%g finish=%g-%g split=%g\n", r.staff.perPlace,
                          r.staff.placeHours ? 1 : 0, r.staff.startLo, r.staff.startHi, r.staff.finishLo,
                          r.staff.finishHi, r.staff.splitOver);
            out += b;
        }
    }
    return out;
}

GoalTable buildDayTable(const RoleDay& d) {
    GoalTable t;
    t.addState("AtHome", GoalAction::Rest, GoalTarget::None, Activity::AtHome);
    if (!d.commute) {
        // A DAY OUT (Glenn, 2026-09-19: "non workers and pedestrians who are out for a stroll or going to public
        // spaces"): from home, a chain of nearby stops -- park, cafe, store, a walk round the block -- each with a
        // pause, until the outing window closes and they head home.
        t.addState("Outing", GoalAction::GoTo, GoalTarget::Outing, Activity::Outing);
        t.addState("OutAndAbout", GoalAction::Rest, GoalTarget::None, Activity::Outing, d.outingPause);
        t.addState("ReturnHome", GoalAction::GoTo, GoalTarget::Home, Activity::Returning);
        t.addTransition("AtHome", GoalEvent::DepartWork, "Outing");
        t.addTransition("Outing", GoalEvent::Arrived, "OutAndAbout");
        t.addTransition("Outing", GoalEvent::NoRoute, "ReturnHome");
        // The window closes -> home (checked before the pause: see goalThink).
        t.addTransition("OutAndAbout", GoalEvent::DepartHome, "ReturnHome");
        t.addTransition("OutAndAbout", GoalEvent::DwellDone, "Outing");
        t.addTransition("ReturnHome", GoalEvent::Arrived, "AtHome");
        t.addTransition("ReturnHome", GoalEvent::NoRoute, "OutAndAbout");
    } else {
        const bool breaks = d.workBeforeBreak > 0 && d.breakTarget != GoalTarget::None;
        const bool once = breaks && !d.breaksRepeat;   // one break, then the afternoon
        t.addState("ToWork", GoalAction::GoTo, GoalTarget::Work, Activity::Commuting);
        t.addState("AtWork", GoalAction::Rest, GoalTarget::None, Activity::AtWork, breaks ? d.workBeforeBreak : 0.0);
        if (breaks) {
            t.addState("GoBreak", GoalAction::GoTo, d.breakTarget, d.breakActivity);
            if (d.breakTarget == GoalTarget::Activity && !d.breakMenu.empty()) t.setMenu("GoBreak", d.breakMenu);
            // the stop sets its own length when it has one (a library session, a coffee); this is the fallback
            t.addState("OnBreak", GoalAction::Rest, GoalTarget::None, d.breakActivity, d.breakHours);
            t.addState("BackToWork", GoalAction::GoTo, GoalTarget::Work, Activity::Commuting);
            if (once) t.addState("AtWorkPM", GoalAction::Rest, GoalTarget::None, Activity::AtWork);
        }
        if (d.errand) {
            // AN ERRAND ON THE WAY HOME (Glenn, 2026-09-17: "go to work / work till 5pm / go shopping / go home").
            t.addState("GoShopping", GoalAction::GoTo, GoalTarget::Shop, Activity::Shopping);
            t.addState("AtShop", GoalAction::Rest, GoalTarget::None, Activity::Shopping, 0.5);
        }
        t.addState("ReturnHome", GoalAction::GoTo, GoalTarget::Home, Activity::Returning);
        const char* leave = d.errand ? "GoShopping" : "ReturnHome";
        const char* atWork = once ? "AtWorkPM" : "AtWork";   // where a break comes back to
        t.addTransition("AtHome", GoalEvent::DepartWork, "ToWork");
        t.addTransition("ToWork", GoalEvent::Arrived, "AtWork");
        t.addTransition("ToWork", GoalEvent::NoRoute, "AtHome");
        t.addTransition("AtWork", GoalEvent::DepartHome, leave);
        if (breaks) {
            t.addTransition("AtWork", GoalEvent::DwellDone, "GoBreak");
            t.addTransition("GoBreak", GoalEvent::Arrived, "OnBreak");
            t.addTransition("GoBreak", GoalEvent::NoRoute, atWork);   // (brought lunch; nowhere open)
            if (!once) t.addTransition("OnBreak", GoalEvent::DepartHome, leave);
            t.addTransition("OnBreak", GoalEvent::DwellDone, "BackToWork");
            t.addTransition("BackToWork", GoalEvent::Arrived, atWork);
            t.addTransition("BackToWork", GoalEvent::NoRoute, atWork);
            if (once) t.addTransition("AtWorkPM", GoalEvent::DepartHome, leave);
        }
        if (d.errand) {
            // No shop, or none routable: straight on home.
            t.addTransition("GoShopping", GoalEvent::Arrived, "AtShop");
            t.addTransition("GoShopping", GoalEvent::NoRoute, "ReturnHome");
            t.addTransition("AtShop", GoalEvent::DwellDone, "ReturnHome");
        }
        t.addTransition("ReturnHome", GoalEvent::Arrived, "AtHome");
        t.addTransition("ReturnHome", GoalEvent::NoRoute, breaks ? (once ? "AtWorkPM" : "OnBreak") : "AtWork");
    }
    if (!d.eveningMenu.empty()) addNightOut(t, d.eveningMenu);
    t.setEntry("AtHome");
    return t;
}

RoleCatalog defaultRoleCatalog() {
    RoleCatalog c;
    auto add = [&](const char* name, RoleDay d, double share) {
        ResidentRole r;
        r.name = name;
        r.day = d;
        r.nightOutShare = share;
        c.roles.push_back(r);
    };
    // THE COMMUTER: to work, lunch about four hours in (a cafe or restaurant near work -- or brought, and stays in),
    // the afternoon, the errand on the way home, and out on about one night in five
    RoleDay work;
    work.workBeforeBreak = 4.0;
    work.breakTarget = GoalTarget::Lunch;
    work.breakActivity = Activity::Lunch;
    work.breakHours = 0.6;
    work.errand = true;
    work.eveningMenu = "evening";
    add("commuter", work, 0.22);
    // THE SHOPKEEPER: the same day, on the shop's hours (assignPlaces), out less often
    add("shopkeeper", work, 0.15);
    // THE STROLLER: no job -- a day out instead, and out more evenings
    RoleDay out;
    out.commute = false;
    out.outingPause = 0.05;
    out.eveningMenu = "evening";
    add("stroller", out, 0.30);
    // THE STUDENT: to class in the teaching hall, a break between classes (the quad, the library, a run), back to
    // class, until the day is done; a college club or a night out on two evenings in five
    RoleDay study;
    study.workBeforeBreak = 0.9;
    study.breakTarget = GoalTarget::Campus;
    study.breakActivity = Activity::Outing;
    study.breakHours = 0.25;
    study.breaksRepeat = true;
    study.eveningMenu = "student_evening";
    add("student", study, 0.40);

    // THE STAFF (Glenn: "you'd need teachers for the students, managers for shops, waiters for restaurants"). Each
    // place pulls its own from the commuters living nearest it.
    auto staff = [&](const char* name, RoleDay d, double share, std::vector<std::string> places, int per, bool placeHours,
                     double sLo = 8, double sHi = 9, double fLo = 17, double fHi = 18) {
        add(name, d, share);
        StaffSpec& s = c.roles.back().staff;
        s.places = std::move(places);
        s.perPlace = per;
        s.placeHours = placeHours;
        s.startLo = sLo; s.startHi = sHi; s.finishLo = fLo; s.finishHi = fHi;
    };
    // a teacher's and a librarian's day is a commuter's, on the campus's hours
    staff("teacher", work, 0.15, {"teaching"}, 4, false, 7.5, 8.5, 16.0, 17.5);
    staff("librarian", work, 0.15, {"library"}, 2, false, 8.5, 9.0, 17.0, 18.5);
    // a manager runs the place: its hours, lunch out, the errand home
    staff("manager", work, 0.12, {"shop", "supermarket", "cafe", "restaurant", "bar", "club"}, 1, true);
    // the floor: on the place's hours, no break out (they eat at work), straight home
    RoleDay floor;
    floor.eveningMenu = "evening";
    staff("waiter", floor, 0.10, {"restaurant", "cafe"}, 3, true);
    staff("cook", floor, 0.08, {"restaurant"}, 2, true);
    staff("clerk", floor, 0.15, {"supermarket", "shop"}, 2, true);
    staff("bartender", floor, 0.05, {"bar", "club"}, 2, true);
    return c;
}

GoalTable builtinDayTable(const std::string& name) {
    const RoleCatalog c = defaultRoleCatalog();
    const ResidentRole* r = c.role(name);
    return r ? buildDayTable(r->day) : GoalTable{};
}

}  // namespace citysim
