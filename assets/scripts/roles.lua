-- THE ROLES (citysim; ~/.claude/plans/citysim-roles.md, src/apps/citysim/city_roles.h)
--
-- Glenn, 2026-10-06: "Students and normal people should all have the same interface in the end. Just that students
-- should do student things and therefore we have a singular system that can describe all of the npc roles."
--
-- Every resident's day is its ROLE's, built from these parts by one composer (buildDayTable) -- never a table written
-- by hand for one kind of person. With no file the citysim uses the same catalog built into C++ (defaultRoleCatalog --
-- a test keeps the two describing identically).
--
-- WHO HOLDS A ROLE (#172; Glenn, 2026-11-01: roles are what a person is, commuting and strolling are parts of a day):
--   * a LIFE STAGE's people (people.lua: each stage without a job names its role -- child, student, at_home, retired);
--   * someone of a working stage, by the job they found: the FIRST role in `order` whose `job` takes that place;
--   * a STAFF role's, hired by the places it staffs, from the holders of a catch-all job role (`job = {}`).
--
-- roles.<name> = {
--   day = {
--     commute           = true | false   to work and back; false: a day OUT instead (stop to stop off the outing table)
--     outing_pause      = hours          (a day out) the pause at each stop, when the stop sets none
--     outings           = "<menu>"       (a day out) the activity menu its stops come from (default "outing")
--     work_before_break = hours          at work before the first break (0: no break)
--     pause = { target = "lunch" | "campus" | "activity", menu = "<activity menu>", label = "Lunch" | "Outing" | ...,
--               hours = h, repeats = true }   -- the break; repeats: work, break, work, break... else one, then the PM
--     errand            = true           the errand on the way home
--     evening           = "<menu>"       a night out's activity menu (activities.lua); none: never goes out
--   },
--   night_out = share                    of this role, the share out on any one night
--   job = { places = { kinds },          a job the job search finds: at these kinds of place (none: any -- the
--           hours = "authored" }         catch-all); "authored": only a place whose hours the level wrote, on them
--   staff = {                            a STAFF role: each place of these kinds pulls per_place of the catch-all job
--     places = { kinds }, per_place = n, holders living nearest it; their hours are the place's ("place": from half
--     hours = "place" | { start = { lo, hi }, finish = { lo, hi } },   an hour before it opens to a quarter after it
--     split_over = h                     closes, two shifts when that is longer than split_over hours) or fixed
--   },
--   side_job = {                         A SECOND WORK SLOT (an evening job): `share` of the role's holders work at
--     share = s, places = { kinds },     one place of these kinds near home (one of the four nearest within `reach`
--     start = { lo, hi }, finish = { lo, hi },   m, open through the shift), start to finish, on `nights` of the
--     nights = n, reach = m              nights -- a shift night is their evening, spent there
--   }
-- }

roles = {
  order = { "shopkeeper", "worker", "student", "child", "at_home", "retired",
            "teacher", "librarian", "manager", "waiter", "cook", "clerk", "bartender" },

  -- keeps a shop whose hours the level wrote: a worker's day, on the shop's hours
  shopkeeper = {
    day = { work_before_break = 4.0, pause = { target = "lunch", label = "Lunch", hours = 0.6 }, errand = true,
            evening = "evening" },
    night_out = 0.15,
    job = { places = { "shop", "cafe", "restaurant", "supermarket", "bar", "club" }, hours = "authored" },
  },
  -- any other job: to work, lunch about four hours in (or brought, and stays in), the afternoon, the errand home
  worker = {
    day = { work_before_break = 4.0, pause = { target = "lunch", label = "Lunch", hours = 0.6 }, errand = true,
            evening = "evening" },
    night_out = 0.22,
    job = {},
  },
  -- to class in the teaching hall, a break between classes (the quad, the library, a run), back to class
  student = {
    day = { work_before_break = 0.9, pause = { target = "campus", label = "Outing", hours = 0.25, repeats = true },
            evening = "student_evening" },
    night_out = 0.40,
    -- three in ten work evenings near the hall, on three nights in five
    side_job = { share = 0.3, places = { "cafe", "restaurant", "bar", "shop", "supermarket" },
                 start = { 17.5, 18.5 }, finish = { 21.5, 23.0 }, nights = 0.6, reach = 1500 },
  },
  -- no job: a day out instead (the park, a cafe, the shops, round the block). A child is never out at night alone.
  -- ...the playground, a game of tag, a skateboard round the block (activities.lua kid_outing)
  child = {
    day = { commute = false, outing_pause = 0.05, outings = "kid_outing" },
    night_out = 0,
  },
  at_home = {
    day = { commute = false, outing_pause = 0.05, evening = "evening" },
    night_out = 0.30,
  },
  retired = {
    day = { commute = false, outing_pause = 0.05, evening = "evening" },
    night_out = 0.15,
  },

  -- THE STAFF (Glenn: "you'd need teachers for the students, managers for shops, waiters for restaurants")
  -- a teacher's and a librarian's day is a worker's, on the campus's hours
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
