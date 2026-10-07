-- THE ACTIVITY CATALOG (citysim; ~/.claude/plans/activities-framework.md, src/apps/citysim/activities.h)
--
-- WHAT a body can go and do, and WHO picks what WHEN. The citysim reads this at load; with no file it uses the same
-- catalog built into C++ (defaultActivityCatalog -- a test keeps the two describing identically).
--
-- activities.<name> = {
--   sites      = { kinds } -- places: cafe restaurant bar club shop supermarket civic park library teaching quad field office;
--                             spots: seat bed stand loop pitch watch; areas: lawn pitch plaza (a paseo); or street
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
            "walk_round_block", "lunch", "campus_bench", "jog", "study", "quad_time", "kickabout", "watch_game",
            "chat", "picnic", "sunbathe", "catch", "errand", "dinner", "drinks", "clubbing", "college_club" },

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
  walk_round_block = { sites = { "street" }, distance = { 150, 450 }, perform = "wander", walkers_only = true },

  -- LUNCH OUT: walkers in their shift; three in eight brought theirs
  lunch            = { sites = { "cafe", "restaurant" }, distance = { 0, 600 }, nearest = 4, walkers_only = true,
                       in_shift = true, bring_own_eighths = 3 },

  -- THE ERRAND on the way home: a supermarket or a store near HOME, one of the three nearest open
  errand           = { sites = { "supermarket", "shop" }, from = "home", distance = { 0, 700 }, nearest = 3 },

  -- THE NIGHT (Glenn: "we would want restaurants and clubs and such for a night life ... College clubs"): dinner out,
  -- drinks at a bar, a club late, a college club's meeting on campus. Farther than a day's stops: a night out is worth
  -- the walk (or the bus).
  dinner           = { sites = { "restaurant" }, hours = { 17.5, 22.5 }, minutes = { 50, 100 }, distance = { 60, 1200 },
                       per_site = true },
  drinks           = { sites = { "bar" }, hours = { 16.5, 1.5 }, distance = { 60, 1200 }, per_site = true },
  clubbing         = { sites = { "club" }, hours = { 21.5, 2.5 }, distance = { 100, 2500 }, nearest = 3 },
  college_club     = { sites = { "teaching", "library" }, hours = { 18, 22 }, minutes = { 50, 90 }, distance = { 0, 1e9 },
                       nearest = 2 },

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

  -- ON THE LAWNS (a park's, the quad's): groups that settle rather than run about. A CHAT -- a ring standing, facing
  -- in, that anyone passing may join (up to six); a PICNIC -- a ring sitting on the grass; SUNBATHING -- lying side
  -- by side; CATCH -- two, ten metres apart, stepping about
  chat             = { sites = { "lawn", "plaza" }, hours = { 7, 22 }, distance = { 0, 650 }, nearest = 4, perform = "roam",
                       formation = "circle", radius = 0.8, minutes = { 10, 25 }, min_players = 2, gather_minutes = 12,
                       roles = { { name = "talker", n = 6, speed = { 1.1, 1.6 }, pose = "stand" } } },
  picnic           = { sites = { "lawn" }, hours = { 11, 15.5 }, distance = { 0, 650 }, nearest = 4, perform = "roam",
                       formation = "circle", radius = 0.85, minutes = { 30, 60 }, min_players = 1, gather_minutes = 15,
                       roles = { { name = "picnicker", n = 4, speed = { 1.1, 1.6 }, pose = "sit_ground" } } },
  sunbathe         = { sites = { "lawn" }, hours = { 10, 17 }, distance = { 0, 650 }, nearest = 4, perform = "roam",
                       formation = "spread", radius = 1.1, minutes = { 20, 45 }, min_players = 1, gather_minutes = 10,
                       roles = { { name = "sunbather", n = 2, speed = { 1.1, 1.6 }, pose = "lie" } } },
  catch            = { sites = { "lawn" }, hours = { 9, 20 }, distance = { 0, 650 }, nearest = 4, perform = "roam",
                       formation = "pair", radius = 9, minutes = { 10, 25 }, min_players = 2, gather_minutes = 10,
                       roles = { { name = "catcher", n = 2, speed = { 1.1, 1.6 }, pose = "stand" } } },
}

menus = {
  order = { "outing", "errand", "lunch", "student_break", "evening", "student_evening" },

  -- A DAY OFF, by the hour: coffee and the park in the morning; the shops and lunch at midday; errands and the park in
  -- the afternoon; dinner and a coffee in the evening -- a bench or a trip across town now and then all day
  outing = {
    { hours = { 5, 10.5 }, first = { { "bench", 0.20 }, { "across_town", 0.10 } },
      pick = { { "coffee", 4 }, { "park_visit", 3 }, { "groceries", 0.5 }, { "civic_visit", 0.3 }, { "walk_round_block", 3 } } },
    { hours = { 10.5, 14 }, first = { { "bench", 0.25 }, { "across_town", 0.14 } },
      pick = { { "browse", 2.5 }, { "meal", 2 }, { "coffee", 2 }, { "park_visit", 3 }, { "civic_visit", 0.7 }, { "walk_round_block", 2 },
               { "picnic", 1.5 }, { "sunbathe", 0.8 }, { "chat", 1 } } },
    { hours = { 14, 17.5 }, first = { { "bench", 0.30 }, { "across_town", 0.14 } },
      pick = { { "browse", 2.5 }, { "groceries", 1.5 }, { "park_visit", 3 }, { "coffee", 1.5 }, { "civic_visit", 0.7 }, { "walk_round_block", 3 },
               { "sunbathe", 1 }, { "catch", 0.8 }, { "chat", 1 }, { "picnic", 0.5 } } },
    { hours = { 17.5, 23 }, first = { { "bench", 0.15 }, { "across_town", 0.10 } },
      pick = { { "meal", 3 }, { "coffee", 1 }, { "park_visit", 1.5 }, { "groceries", 1 }, { "walk_round_block", 2 },
               { "chat", 1 }, { "catch", 0.5 } } },
    { hours = { 23, 5 }, first = { { "bench", 0 }, { "across_town", 0 } },
      pick = { { "walk_round_block", 1 }, { "park_visit", 0.5 } } },
  },

  errand = { { hours = { 0, 24 }, first = { { "errand", 1.0 } } } },

  lunch = { { hours = { 0, 24 }, first = { { "lunch", 1.0 } } } },

  -- between classes: lunch out at midday, a run in the late afternoon, most often a bench, else the library or
  -- the quad
  -- (and on the quad's lawns: a chat, a lie in the sun, a game of catch, a picnic lunch)
  student_break = {
    { hours = { 11.5, 13.5 }, first = { { "lunch", 0.45 }, { "campus_bench", 0.55 } },
      pick = { { "study", 2 }, { "quad_time", 1 }, { "chat", 1.5 }, { "picnic", 1 }, { "sunbathe", 0.6 } } },
    { hours = { 15, 19.5 }, first = { { "kickabout", 0.2 }, { "watch_game", 0.15 }, { "jog", 0.25 }, { "campus_bench", 0.667 } },
      pick = { { "study", 2 }, { "quad_time", 1 }, { "chat", 1.5 }, { "catch", 1 }, { "sunbathe", 0.6 } } },
    { hours = { 0, 24 }, first = { { "campus_bench", 0.75 } },
      pick = { { "study", 2 }, { "quad_time", 1 }, { "chat", 1.2 }, { "sunbathe", 0.5 } } },
  },

  -- A NIGHT OUT, by the hour: dinner and a first drink early; drinks and the clubs later; after midnight the clubs and
  -- the last bars -- a walk now and then between
  evening = {
    { hours = { 16, 21 }, first = {},
      pick = { { "dinner", 3 }, { "drinks", 2 }, { "walk_round_block", 0.8 }, { "park_visit", 0.4 }, { "chat", 0.5 } } },
    { hours = { 21, 23.5 }, first = {},
      pick = { { "drinks", 3 }, { "clubbing", 1.5 }, { "dinner", 0.8 }, { "walk_round_block", 0.5 } } },
    { hours = { 23.5, 16 }, first = {}, pick = { { "clubbing", 3 }, { "drinks", 2 } } },
  },

  -- A STUDENT'S EVENING: a college club's meeting early, then out like anyone
  student_evening = {
    { hours = { 16, 22 }, first = { { "college_club", 0.45 } },
      pick = { { "dinner", 2 }, { "drinks", 2 }, { "chat", 0.8 }, { "quad_time", 0.5 } } },
    { hours = { 22, 16 }, first = {}, pick = { { "clubbing", 3 }, { "drinks", 2.5 } } },
  },
}
