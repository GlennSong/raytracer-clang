-- THE ROLES (citysim; ~/.claude/plans/citysim-roles.md, src/apps/citysim/city_roles.h)
--
-- Glenn, 2026-10-06: "Students and normal people should all have the same interface in the end. Just that students
-- should do student things and therefore we have a singular system that can describe all of the npc roles."
--
-- Every resident's day is its ROLE's, built from these parts by one composer (buildDayTable) -- never a table written
-- by hand for one kind of person. With no file the citysim uses the same catalog built into C++ (defaultRoleCatalog --
-- a test keeps the two describing identically). The first four, in `order`, are the built-in roles.
--
-- roles.<name> = {
--   day = {
--     commute           = true | false   to work and back; false: a day OUT instead (stop to stop off the outing table)
--     outing_pause      = hours          (a day out) the pause at each stop, when the stop sets none
--     work_before_break = hours          at work before the first break (0: no break)
--     pause = { target = "lunch" | "campus" | "activity", menu = "<activity menu>", label = "Lunch" | "Outing" | ...,
--               hours = h, repeats = true }   -- the break; repeats: work, break, work, break... else one, then the PM
--     errand            = true           the errand on the way home
--     evening           = "<menu>"       a night out's activity menu (activities.lua); none: never goes out
--   },
--   night_out = share                    of this role, the share out on any one night
--   staff = {                            a STAFF role: each place of these kinds pulls per_place of the commuters living
--     places = { kinds }, per_place = n, nearest it; their hours are the place's ("place": from half an hour before it
--     hours = "place" | { start = { lo, hi }, finish = { lo, hi } },   opens to a quarter after it closes, two shifts
--     split_over = h                     when that is longer than split_over hours) or fixed
--   }
-- }

roles = {
  order = { "commuter", "shopkeeper", "stroller", "student",
            "teacher", "librarian", "manager", "waiter", "cook", "clerk", "bartender" },

  -- to work, lunch about four hours in (or brought, and stays in), the afternoon, the errand on the way home
  commuter = {
    day = { work_before_break = 4.0, pause = { target = "lunch", label = "Lunch", hours = 0.6 }, errand = true,
            evening = "evening" },
    night_out = 0.22,
  },
  -- the same day, on the shop's hours
  shopkeeper = {
    day = { work_before_break = 4.0, pause = { target = "lunch", label = "Lunch", hours = 0.6 }, errand = true,
            evening = "evening" },
    night_out = 0.15,
  },
  -- no job: a day out instead, and out more evenings
  stroller = {
    day = { commute = false, outing_pause = 0.05, evening = "evening" },
    night_out = 0.30,
  },
  -- to class in the teaching hall, a break between classes (the quad, the library, a run), back to class
  student = {
    day = { work_before_break = 0.9, pause = { target = "campus", label = "Outing", hours = 0.25, repeats = true },
            evening = "student_evening" },
    night_out = 0.40,
  },

  -- THE STAFF (Glenn: "you'd need teachers for the students, managers for shops, waiters for restaurants")
  -- a teacher's and a librarian's day is a commuter's, on the campus's hours
  teacher = {
    day = { work_before_break = 4.0, pause = { target = "lunch", label = "Lunch", hours = 0.6 }, errand = true,
            evening = "evening" },
    night_out = 0.15,
    staff = { places = { "teaching" }, per_place = 4, hours = { start = { 7.5, 8.5 }, finish = { 16.0, 17.5 } } },
  },
  librarian = {
    day = { work_before_break = 4.0, pause = { target = "lunch", label = "Lunch", hours = 0.6 }, errand = true,
            evening = "evening" },
    night_out = 0.15,
    staff = { places = { "library" }, per_place = 2, hours = { start = { 8.5, 9.0 }, finish = { 17.0, 18.5 } } },
  },
  -- a manager runs the place: its hours, lunch out, the errand home
  manager = {
    day = { work_before_break = 4.0, pause = { target = "lunch", label = "Lunch", hours = 0.6 }, errand = true,
            evening = "evening" },
    night_out = 0.12,
    staff = { places = { "shop", "supermarket", "cafe", "restaurant", "bar", "club" }, per_place = 1, hours = "place" },
  },
  -- the floor: on the place's hours, no break out (they eat at work), straight home
  waiter = {
    day = { evening = "evening" },
    night_out = 0.10,
    staff = { places = { "restaurant", "cafe" }, per_place = 3, hours = "place" },
  },
  cook = {
    day = { evening = "evening" },
    night_out = 0.08,
    staff = { places = { "restaurant" }, per_place = 2, hours = "place" },
  },
  clerk = {
    day = { evening = "evening" },
    night_out = 0.15,
    staff = { places = { "supermarket", "shop" }, per_place = 2, hours = "place" },
  },
  bartender = {
    day = { evening = "evening" },
    night_out = 0.05,
    staff = { places = { "bar", "club" }, per_place = 2, hours = "place" },
  },
}
