-- HOW PEOPLE GET ABOUT (the trip planner: src/apps/citysim/trip_plan.h; ADR-0415, ADR-0440)
--
-- One row per way of travelling. A person going somewhere prices every row they can use as a whole trip, and takes the
-- fewest weighted minutes (a minute of a way counts `weight` minutes). Ties go to the earlier row. A way that needs a
-- vehicle they own starts with getting to it and ends with getting from where they leave it to the door, each by the
-- cheapest way on foot the person has (walking, or a board for someone who has one). Adding a way is a row here, plus
-- its execution where it moves a person differently.
--
--   name           what it is called (the plan and the trip? line say it)
--   network        "footpaths" (where walkers go), "roads" (where cars go) or "bus" (the road file's bus lines: on foot
--                  to a stop, the wait, the ride round the line, on foot from a stop; ADR-0484)
--   speed_kmh      FOOTPATHS ONLY: average door-to-door speed, crossings included. A way on the roads has none:
--                  its minutes are the router's own time for the route it takes -- each road class at its speed
--                  (engine::classSpeed: freeway 100 km/h, arterial 58, collector 43, local 29, ramp 36) and a few
--                  seconds at each street junction -- the time the route was chosen by (ADR-0416)
--   start_minutes  fixed minutes at the start of a leg of it (unlocking, pulling out)
--   end_minutes    fixed minutes at its end (finding a space, parking, locking up)
--   set_off_wait_minutes  OPTIONAL: the longest a leg of it waits to set off. A car ready to leave its kerb space
--                  waits for the lane beside it to be clear (and the lanes leading into that one, for 30 m back);
--                  after this long the person gives the car up as blocked and plans again without it (the agent
--                  brain's `blocked` event, ADR-0442). Left out or 0: it waits for as long as it takes
--   own_vehicle   needs a vehicle the person owns, where it stands; not priced while someone else has it
--   access         OPTIONAL: fixes the way used to reach that vehicle and to leave it (a row's name). Left out, the
--                  usual: the cheapest way on foot the person has
--   requires       OPTIONAL: a trait from `traits` below. The row is priced only for people who have it
--   longest_leg_minutes  OPTIONAL: the longest leg it suits. A leg that would take longer is not made this way
--   begins_in      what a person is doing when a leg of it begins: a state of brain.lua (the agent brain, ADR-0440)
--   weight         how much the person minds a minute of it (1 = a plain minute)
--   colour         the plan's line for it, drawn in the world (r, g, b in 0..1)

travel = {
  -- WHAT A PERSON CAN HAVE that a way requires (at most 8). Who has which is dealt with the population.
  traits = { "has_board", "has_scooter" },
  modes = {
    { name = "walk", network = "footpaths", speed_kmh = 5.0, start_minutes = 0.0, end_minutes = 0.0,
      own_vehicle = false, weight = 1.0, begins_in = "walking", colour = { 0.25, 0.95, 0.35 } },
    -- (set_off_wait_minutes is an ESTIMATE, not measured: about one cycle of a traffic light, after which a queue
    -- standing beside the space has had its chance to move on)
    -- (start and end minutes: two each, the owner's choice of 2026-11-16 -- they were 2 and 3. They decide when a
    -- car is worth taking: at a minute each the car beat a 4.6-minute walk to the shop in brain_lab)
    { name = "car", network = "roads", start_minutes = 2.0, end_minutes = 2.0, set_off_wait_minutes = 1.5,
      own_vehicle = true, weight = 1.0, begins_in = "pulling_out", colour = { 0.25, 0.55, 1.0 } },
    -- SMALL WHEELS: a faster way of doing a leg on foot, not a vehicle to be parked (agent-brain-design.md section
    -- 12). The numbers are ESTIMATES, not measured: a skateboard cruises at 10 to 14 km/h and a kick scooter a
    -- little faster, less the crossings; a minute of either is minded a little more than a minute of walking (the
    -- effort, the pavement); and neither is taken for a long leg.
    { name = "skateboard", network = "footpaths", speed_kmh = 10.0, start_minutes = 0.0, end_minutes = 0.0,
      own_vehicle = false, requires = "has_board", longest_leg_minutes = 15.0, weight = 1.2,
      begins_in = "on_small_wheels", colour = { 0.95, 0.45, 0.85 } },
    { name = "scooter", network = "footpaths", speed_kmh = 12.0, start_minutes = 0.0, end_minutes = 0.0,
      own_vehicle = false, requires = "has_scooter", longest_leg_minutes = 20.0, weight = 1.2,
      begins_in = "on_small_wheels", colour = { 0.60, 0.90, 0.95 } },
    -- BY BUS (ADR-0484): a bus row's minutes are its line's timetable (each leg's driving time and each stop's wait,
    -- assets/data/bus_service.json `dwell_seconds`), and its start minutes are the mean wait at the stop: HALF THE
    -- HEADWAY of bus_service.json (8 minutes -> 4; a level test holds the two together). A minute on the bus is minded
    -- as a plain minute. (ESTIMATES: a frequent city service, its wait the textbook half headway when people do not
    -- time their arrival; TCRP Report 165, chapter 4.)
    { name = "bus", network = "bus", start_minutes = 4.0, end_minutes = 0.0, own_vehicle = false, weight = 1.0,
      begins_in = "waiting", colour = { 0.95, 0.75, 0.15 } },
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
