#ifndef RAYTRACER_CITYSIM_CITY_ROLES_H
#define RAYTRACER_CITYSIM_CITY_ROLES_H

// ONE ROLE SYSTEM FOR EVERY NPC (~/.claude/plans/citysim-roles.md; Glenn, 2026-10-06: "Students and normal people
// should all have the same interface in the end. Just that students should do student things and therefore we have a
// singular system that can describe all of the npc roles" / "a unified way to define these is better than a lot of
// one offs").
//
// A ROLE is data: a name and the parts of its DAY -- whether it goes to work, how long it works before a break, what
// the break is (lunch, a campus break, a menu of the activity catalog) and whether breaks repeat (class, break,
// class...), the errand on the way home, a day out instead of work, its evening menu and how often it goes out. ONE
// composer, buildDayTable, turns any role's day into its goal table: every resident runs a table built the same way
// from the same vocabulary of states, never a table written by hand for one kind of person.
//
// Authored in assets/scripts/roles.lua (scripting/roles_lua.h reads it); defaultRoleCatalog is the same catalog in
// C++ (a test keeps the two describing identically). Role ids are indexes into the catalog (RoleId). The sim names no
// role in code (#172, kinds are data): a life stage names its role (people.lua), a job found names its by the place
// (JobSpec), a place hires its staff (StaffSpec).

#include "city_goals.h"

#include <cstdint>

#include <string>
#include <vector>

namespace citysim {

// The parts of a day (buildDayTable's input).
struct RoleDay {
    // To work and back (AtHome -> ToWork -> AtWork ... -> ReturnHome). False: a day OUT instead -- from home, stop
    // to stop off the outing table (pickOuting), a pause at each (`outingPause` hours, the stop's own length when it
    // sets one), until the window closes.
    bool commute = true;
    double outingPause = 0.05;
    std::string outingMenu = "outing";   // the day out's stops: this activity menu (a child's: kid_outing)
    // At work: hours before the first break (0: no break -- at work until the window closes).
    double workBeforeBreak = 0;
    // The break: where it goes (Lunch, Campus, or Activity with `breakMenu`), the label it wears, how long it lasts,
    // and whether breaks REPEAT (a class, a break, a class...) or there is one, then the afternoon (AtWorkPM).
    GoalTarget breakTarget = GoalTarget::None;
    std::string breakMenu;
    Activity breakActivity = Activity::Lunch;
    double breakHours = 0;
    bool breaksRepeat = false;
    // The errand on the way home (a supermarket or a store near home; half an hour).
    bool errand = false;
    // The evening (addNightOut): the activity menu a night out picks from; empty = never goes out.
    std::string eveningMenu;
};

// WHO WORKS WHERE, for a STAFF role (Glenn: "you'd need teachers for the students, managers for shops, waiters for
// restaurants"): the kinds of place it staffs ("restaurant", "teaching", ... -- CitySim::placeKind), how many each
// place takes, and its hours -- fixed windows, or the PLACE's own (opening half an hour before it opens, leaving a
// quarter after it closes; a day longer than `splitOver` hours is two shifts, early and late). Each place pulls its
// staff from the catch-all job role's holders living nearest it (CitySim::assignStaff): the population is the same, its jobs are not.
struct StaffSpec {
    std::vector<std::string> places;   // empty: not a staff role
    int perPlace = 0;
    bool placeHours = false;
    double startLo = 8, startHi = 9, finishLo = 17, finishHi = 18;   // fixed hours (placeHours false)
    double splitOver = 9;
};

// A role the JOB SEARCH hands out (assignPlaces: the job a working resident finds near home): the first role in the
// catalog whose spec takes the place found. No places: any (the catch-all, whose holders the staff are hired from).
// `authoredHours`: only a place whose hours the level wrote, and the role keeps them (opens up, closes up).
struct JobSpec {
    bool search = false;
    std::vector<std::string> places;
    bool authoredHours = false;
};

// A SECOND WORK SLOT (plan step 6; a student's evening job): `share` of the role's holders work evenings at one place
// of these kinds near home (one of the four nearest within `reach` m, open through the shift), from start to finish,
// on `nights` of the nights. A shift night is their evening (CitySim::eveningPlan), spent at that place.
struct SideJobSpec {
    double share = 0;   // 0: none
    std::vector<std::string> places;
    double startLo = 17.5, startHi = 18.5, finishLo = 21.5, finishHi = 23.0;
    double nights = 0.6;
    double reach = 1500.0;
};

struct ResidentRole {
    std::string name;
    RoleDay day;
    double nightOutShare = 0;   // of this role, the share out on any one night (CitySim::eveningPlan)
    JobSpec job;
    StaffSpec staff;
    SideJobSpec side;
};

// AN AGENT'S ROLE (ADR-0282, #117 item 27): its index in the catalog.
using RoleId = uint8_t;

struct RoleCatalog {
    std::vector<ResidentRole> roles;   // the id is the index
    int find(const std::string& name) const;   // -1 when unknown
    const ResidentRole* role(const std::string& name) const;
    // Canonical text: two catalogs that describe identically make the same days.
    std::string describe() const;
    // The role's name, "?" past the catalog.
    const char* nameOf(RoleId id) const { return id < roles.size() ? roles[id].name.c_str() : "?"; }
    // The role of a job found at a place of `kind` (its hours authored or not): the first job role that takes it;
    // -1 when none does.
    int jobRoleFor(const std::string& kind, bool authored) const;
    // The first job role that takes any place: the job search's catch-all and the staff's pool; -1 when none.
    int catchAllJobRole() const;
    // What is wrong with it ("" when nothing): no catch-all job role, more roles than a RoleId holds.
    std::string check() const;
};

// The built-in catalog (roles.lua in C++): shopkeeper, worker, student, child, at_home, retired, then the staff --
// teacher, librarian, manager, waiter, cook, clerk, bartender.
RoleCatalog defaultRoleCatalog();

// A day's goal table, composed from its parts. State names are the one vocabulary every role shares: AtHome, ToWork,
// AtWork, GoBreak, OnBreak, BackToWork, AtWorkPM, GoShopping, AtShop, Outing, OutAndAbout, ReturnHome, and the night
// out's GoOut, OutTonight, NextStop, HomeTonight.
GoalTable buildDayTable(const RoleDay& day);
// The built-in role `name`'s table (empty table when unknown).
GoalTable builtinDayTable(const std::string& name);

}  // namespace citysim

#endif
