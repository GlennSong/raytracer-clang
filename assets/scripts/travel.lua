-- HOW PEOPLE GET ABOUT (the trip planner: src/apps/citysim/trip_plan.h, city_trip.h; ADR-0415)
--
-- One row per way of travelling. A person going somewhere prices every row as a whole trip -- a way that needs a
-- vehicle they own starts with getting to it and ends with getting from where they leave it to the door, by its
-- `access` way -- and takes the fewest weighted minutes (a minute of a way counts `weight` minutes). Ties go to the
-- earlier row. Adding a way is a row here, plus its execution in city_trip.cpp where it moves a person differently.
--
--   name           what it is called (the plan and the trip? line say it)
--   network        "footpaths" (where walkers go) or "roads" (where cars go)
--   speed_kmh      FOOTPATHS ONLY: average door-to-door speed, crossings included. A way on the roads has none:
--                  its minutes are the router's own time for the route it takes -- each road class at its speed
--                  (engine::classSpeed: freeway 100 km/h, arterial 58, collector 43, local 29, ramp 36) and a few
--                  seconds at each street junction -- the time the route was chosen by (ADR-0416)
--   start_minutes  fixed minutes at the start of a leg of it (unlocking, pulling out)
--   end_minutes    fixed minutes at its end (finding a space, parking, locking up)
--   own_vehicle    needs a vehicle the person owns, where it stands
--   access         the way used to reach that vehicle and to leave it (a row's name)
--   weight         how much the person minds a minute of it (1 = a plain minute)
--   colour         the plan's line for it, drawn in the world (r, g, b in 0..1)

travel = {
  modes = {
    { name = "walk", network = "footpaths", speed_kmh = 5.0, start_minutes = 0.0, end_minutes = 0.0,
      own_vehicle = false, weight = 1.0, colour = { 0.25, 0.95, 0.35 } },
    { name = "car", network = "roads", start_minutes = 2.0, end_minutes = 3.0,
      own_vehicle = true, access = "walk", weight = 1.0, colour = { 0.25, 0.55, 1.0 } },
  },
  -- THE KINDS OF ROAD a drive is told apart by: the plan says each leg's metres on each kind ("420 m streets,
  -- 1,350 m freeway, 2 ramps") and draws each kind's part of the line in its colour (none: the way's own).
  --   name     what the plan calls it          classes  the router's road classes it covers
  --   one      one of a counted kind           count    say how many separate stretches, not metres
  road_kinds = {
    { name = "streets", classes = { "arterial", "collector", "local", "alley" } },
    { name = "freeway", classes = { "freeway" }, colour = { 1.0, 0.45, 0.10 } },
    { name = "ramps", one = "ramp", classes = { "ramp" }, count = true, colour = { 1.0, 0.80, 0.20 } },
  },
  -- Where a person picks a destination when told "somewhere near" or "somewhere far": straight-line metres from
  -- where they stand.
  -- river_town is about 1.2 km across. The lab's person lives in the houses at the end of the south-west freeway
  -- stub (agent_lab citysim.homeNear, ADR-0416), whose nearest other buildings in town are about 500 m off, so
  -- "near" reaches 700 m. (A share of the reachable buildings, nearest third and farthest third, would fit any town
  -- and any home; these are plain metres for now.)
  near = { 100, 700 },
  far = { 700, 4000 },
}
