-- THE ACTIVITY CATALOG (citysim; ~/.claude/plans/activities-framework.md, src/apps/citysim/activities.h)
--
-- WHAT a body can go and do, and WHO picks what WHEN. The citysim reads this at load; with no file it uses the same
-- catalog built into C++ (defaultActivityCatalog -- a test keeps the two describing identically).
--
-- activities.<name> = {
--   sites      = { kinds } -- places: cafe restaurant shop supermarket civic park library teaching quad field office;
--                             spots: seat bed stand loop pitch watch; or street (a corner to walk to)
--   tags       = { campus | park | sports }   -- a spot site must carry them all
--   hours      = { lo, hi }    when it is offered (default all day; may wrap midnight)
--   minutes    = { lo, hi }    how long (default: what that kind of site keeps a visitor)
--   distance   = { lo, hi }    straight line from where the agent stands (default 0-650 m)
--   nearest    = k             one of the k nearest candidates (default: any in the band)
--   per_site   = true          its menu weight counts once per candidate site
--   walkers_only, in_shift, bring_own_eighths = n
--   from       = "home"      the distance band is measured from home (an errand near home), not from here
--   perform    = inside | outside | spot | wander | roam
--   a GROUP (perform = roam, sites = { "pitch" }): roles = { { name, n, speed = { lo, hi }, zone = -1 | 0 | 1 } },
--     min_players (it starts with them), gather_minutes (given up after), swap_at (0..1: the zones swap)
--   during     = "activity"   only offered while that activity is running within reach (watching it)
-- }
-- menus.<name> = { { hours = { lo, hi }, first = { { activity, chance }, ... }, pick = { { activity, weight }, ... } }, ... }
--   the first band holding the hour is used; `first` entries are tried in order, each with its chance, then the
--   weighted `pick`.

activities = {
  order = { "bench", "across_town", "park_visit", "coffee", "browse", "groceries", "meal", "civic_visit",
            "walk_round_block", "lunch", "campus_bench", "jog", "study", "quad_time", "kickabout", "watch_game", "errand" },

  -- A DAY OFF'S STOPS
  bench            = { sites = { "seat" }, distance = { 30, 500 }, nearest = 4, perform = "spot" },
  across_town      = { sites = { "park", "quad", "field", "civic", "library", "teaching", "restaurant", "cafe" },
                       distance = { 1200, 3000 } },
  park_visit       = { sites = { "park", "quad", "field" }, distance = { 60, 650 }, per_site = true, perform = "outside" },
  coffee           = { sites = { "cafe" }, distance = { 60, 650 }, per_site = true },
  browse           = { sites = { "shop" }, distance = { 60, 650 }, per_site = true },
  groceries        = { sites = { "supermarket" }, distance = { 60, 650 }, per_site = true },
  meal             = { sites = { "restaurant" }, hours = { 11.5, 21.5 }, distance = { 60, 650 }, per_site = true },
  civic_visit      = { sites = { "civic", "library", "teaching" }, distance = { 60, 650 }, per_site = true },
  walk_round_block = { sites = { "street" }, distance = { 150, 450 }, perform = "wander" },

  -- LUNCH OUT: walkers in their shift; three in eight brought theirs
  lunch            = { sites = { "cafe", "restaurant" }, distance = { 0, 600 }, nearest = 4, walkers_only = true,
                       in_shift = true, bring_own_eighths = 3 },

  -- THE ERRAND on the way home: a supermarket or a store near HOME, one of the three nearest open
  errand           = { sites = { "supermarket", "shop" }, from = "home", distance = { 0, 700 }, nearest = 3 },

  -- A STUDENT'S BREAK
  campus_bench     = { sites = { "seat" }, tags = { "campus" }, distance = { 0, 600 }, nearest = 6, perform = "spot" },
  jog              = { sites = { "loop" }, tags = { "campus" }, distance = { 0, 900 }, nearest = 6, perform = "spot" },
  study            = { sites = { "library" }, distance = { 0, 1e9 }, nearest = 1 },
  quad_time        = { sites = { "quad" }, distance = { 0, 1e9 }, nearest = 1, perform = "outside" },

  -- A KICKABOUT on the campus pitch: two sides of five, each in its half, swapping ends at half time; it starts with
  -- six there, and is given up if it has not after twenty minutes
  kickabout        = { sites = { "pitch" }, tags = { "campus" }, distance = { 0, 900 }, nearest = 3, perform = "roam",
                       minutes = { 30, 45 }, min_players = 6, gather_minutes = 20, swap_at = 0.5,
                       roles = { { name = "home", n = 5, speed = { 1.5, 4.0 }, zone = 0 },
                                 { name = "away", n = 5, speed = { 1.5, 4.0 }, zone = 1 } } },
  -- ...and watching it from the stand (only while one is on)
  watch_game       = { sites = { "seat" }, tags = { "sports" }, distance = { 0, 900 }, nearest = 6, perform = "spot",
                       minutes = { 15, 40 }, during = "kickabout" },
}

menus = {
  order = { "outing", "errand", "lunch", "student_break" },

  -- A DAY OFF, by the hour: coffee and the park in the morning; the shops and lunch at midday; errands and the park in
  -- the afternoon; dinner and a coffee in the evening -- a bench or a trip across town now and then all day
  outing = {
    { hours = { 5, 10.5 }, first = { { "bench", 0.20 }, { "across_town", 0.10 } },
      pick = { { "coffee", 4 }, { "park_visit", 3 }, { "groceries", 0.5 }, { "civic_visit", 0.3 }, { "walk_round_block", 3 } } },
    { hours = { 10.5, 14 }, first = { { "bench", 0.25 }, { "across_town", 0.14 } },
      pick = { { "browse", 2.5 }, { "meal", 2 }, { "coffee", 2 }, { "park_visit", 3 }, { "civic_visit", 0.7 }, { "walk_round_block", 2 } } },
    { hours = { 14, 17.5 }, first = { { "bench", 0.30 }, { "across_town", 0.14 } },
      pick = { { "browse", 2.5 }, { "groceries", 1.5 }, { "park_visit", 3 }, { "coffee", 1.5 }, { "civic_visit", 0.7 }, { "walk_round_block", 3 } } },
    { hours = { 17.5, 23 }, first = { { "bench", 0.15 }, { "across_town", 0.10 } },
      pick = { { "meal", 3 }, { "coffee", 1 }, { "park_visit", 1.5 }, { "groceries", 1 }, { "walk_round_block", 2 } } },
    { hours = { 23, 5 }, first = { { "bench", 0 }, { "across_town", 0 } },
      pick = { { "walk_round_block", 1 }, { "park_visit", 0.5 } } },
  },

  errand = { { hours = { 0, 24 }, first = { { "errand", 1.0 } } } },

  lunch = { { hours = { 0, 24 }, first = { { "lunch", 1.0 } } } },

  -- between classes: lunch out at midday, a run in the late afternoon, most often a bench, else the library or
  -- the quad
  student_break = {
    { hours = { 11.5, 13.5 }, first = { { "lunch", 0.45 }, { "campus_bench", 0.55 } },
      pick = { { "study", 2 }, { "quad_time", 1 } } },
    { hours = { 15, 19.5 }, first = { { "kickabout", 0.2 }, { "watch_game", 0.15 }, { "jog", 0.25 }, { "campus_bench", 0.667 } },
      pick = { { "study", 2 }, { "quad_time", 1 } } },
    { hours = { 0, 24 }, first = { { "campus_bench", 0.75 } }, pick = { { "study", 2 }, { "quad_time", 1 } } },
  },
}
