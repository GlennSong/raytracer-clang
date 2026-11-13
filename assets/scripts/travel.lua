-- HOW PEOPLE GET ABOUT (the trip planner: src/apps/citysim/trip_plan.h, city_trip.h; ADR-0415)
--
-- One row per way of travelling. A person going somewhere prices every row as a whole trip -- a way that needs a
-- vehicle they own starts with getting to it and ends with getting from where they leave it to the door, by its
-- `access` way -- and takes the fewest weighted minutes (a minute of a way counts `weight` minutes). Ties go to the
-- earlier row. Adding a way is a row here, plus its execution in city_trip.cpp where it moves a person differently.
--
--   name           what it is called (the plan and the trip? line say it)
--   network        "footpaths" (where walkers go) or "roads" (where cars go)
--   speed_kmh      average door-to-door speed on that network, junctions and lights included
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
    { name = "car", network = "roads", speed_kmh = 30.0, start_minutes = 2.0, end_minutes = 3.0,
      own_vehicle = true, access = "walk", weight = 1.0, colour = { 0.25, 0.55, 1.0 } },
  },
  -- Where a person picks a destination when told "somewhere near" or "somewhere far": straight-line metres from
  -- where they stand.
  -- river_town is about 1.2 km across and the person's home is near its middle, so "far" starts at 450 m: from
  -- home nothing reachable stands 900 m off. (A share of the reachable buildings, nearest third and farthest third,
  -- would fit any town; these are plain metres for now.)
  near = { 100, 450 },
  far = { 450, 4000 },
}
