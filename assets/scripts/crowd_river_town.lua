-- RIVER TOWN'S CROWD (ADR-0457): the same format as crowd.lua (read by the same reader, scripting/crowd_lua.cpp; every
-- field is described there), sized to river_town. river_town names it (citysim.crowd = "crowd_river_town.lua").
--
-- river_town's streets span about 1.4 by 1.7 km against the traffic lab's 1.14 km square (1.8 times the area), with 144
-- junctions to the lab's 156 and 325 streets to its 293: crowd.lua's 350 people and its places, scaled by the area and
-- rounded. The days are crowd.lua's (to work at least 900 m away and home again, the start of the day spread over
-- minutes 0-8), so the two levels' runs read the same way.
--
-- ESTIMATES, not measured, like crowd.lua's.

crowd = {
  residents = 630,
  intent = "be_outside_at",
  keep_from_ends = 15.0,
  places = {
    { kind = "home", homes = true },
    { kind = "work", count = 145, parking = true },
    { kind = "shop", count = 45, parking = true },
    { kind = "lunch", count = 27 },
  },
  household_sizes = {
    { size = 1, share = 0.90 },
    { size = 2, share = 0.10 },
  },
  cars = {
    { cars = 0, share = 0.33 },
    { cars = 1, share = 0.60 },
    { cars = 2, share = 0.07 },
  },
  traits = {
    { name = "has_board", share = 0.08 },
    { name = "has_scooter", share = 0.05 },
  },
  days = {
    { share = 0.94, stops = { { kind = "work", beyond = 900, minutes = { 1, 4 } }, { home = true, minutes = { 0, 8 } } } },
    { share = 0.03, stops = { { kind = "lunch", minutes = { 3, 6 } }, { home = true, minutes = { 2, 12 } } } },
    { share = 0.03, stops = { { kind = "shop", minutes = { 3, 8 } }, { home = true, minutes = { 4, 14 } } } },
  },
}
