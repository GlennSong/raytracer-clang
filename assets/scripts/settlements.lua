-- THE KINDS OF PLACE ON AN ISLAND (src/engine/procgen/world/settlements.h; plan ~/.claude/plans/beach-town.md)
--
-- What each kind of place IS: how big it grows from its flat ground, the roads that reach it, its buses, how its
-- streets are briefed, how the map draws it and what its limit sign says. Where a kind is FOUND on new ground is the
-- island planner's (island_world.cpp). With no file the planner uses the same catalog built into C++
-- (defaultSettlements -- a test keeps the two describing identically). A field left out is a town's.
--
-- settlements["<kind>"] = {
--   hub           = "financial" | "commercial"   its centre's district in the level (citysim.districts.hubs)
--   radius        = { scale, min, max }   its radius: the depth of its flat ground x scale, clamped (m)
--   coast_reach   = m                     coastal when the sea is this near
--   on_ring       = bool                  the island freeway's ring calls at it
--   pass_end      = bool                  the pass over the range joins two of these
--   mountain_road = bool                  a mountain road climbs to it from the freeway (its only road)
--   interchanges  = n                     freeway diamonds it is given first (more first)
--   links         = { candidates, spacing }   link roads offered to the freeway, their ends this far apart (m)
--   buses         = { routes, stops }     its local bus network; 0 routes: none
--   signed_far    = bool                  named on the island's distance and control-city signs
--   limit_sign    = "TOWN LIMIT"
--   naming        = "city" | "town" | "mountain"   how place_names names it
--   brief = { prefix, area_scale, size_min, spoke_spacing, core_block = { l, s }, midtown_block = { l, s }, warp,
--             ring_spacing, spokes }      its street plan (island_world.cpp islandSiteBrief)
--   strip = { length, rows, row, cross, sea }   a STRIP town, laid along the road it grew from (a beach town along
--                                       its coast road): `length` m of it is the seafront street, `rows` streets behind
--                                       it every `row` m inland, cross streets every `cross` m, its limits `sea` m out
--   map = { rgb = { r, g, b }, dot = px, label = m }
-- }

settlements = {
  order = { "city", "town", "mountain town", "beach town" },

  city = {
    hub = "financial",
    radius = { 1.6, 600, 2000 },
    coast_reach = 2500,
    pass_end = true,
    interchanges = 3,
    links = { 6, 700 },
    buses = { 4, 14 },
    signed_far = true,
    limit_sign = "CITY LIMIT",
    naming = "city",
    -- downtown blocks big enough for towers with room round them (Glenn: "Saltwood downtown blocks should be bigger")
    brief = { prefix = "city_", area_scale = 1.15, size_min = 1400, spoke_spacing = 480,
              core_block = { 128, 96 }, midtown_block = { 165, 118 }, warp = 18, ring_spacing = 140, spokes = 10 },
    map = { rgb = { 0.85, 0.12, 0.12 }, dot = 7, label = 260 },
  },

  -- a small walkable grid round its high street
  town = {
    hub = "commercial",
    radius = { 1.3, 250, 700 },
    coast_reach = 1600,
    interchanges = 1,
    links = { 4, 300 },
    buses = { 1, 8 },
    limit_sign = "TOWN LIMIT",
    naming = "town",
    brief = { prefix = "town_", area_scale = 1.0, size_min = 500, spoke_spacing = 360,
              core_block = { 94, 64 }, midtown_block = { 158, 108 }, warp = 10, ring_spacing = 120, spokes = 6 },
    map = { rgb = { 0.95, 0.45, 0.05 }, dot = 5, label = 150 },
  },

  -- up its own winding road, off the ring
  ["mountain town"] = {
    hub = "commercial",
    radius = { 1.4, 150, 450 },
    on_ring = false,
    mountain_road = true,
    links = { 0, 300 },
    buses = { 0, 8 },
    naming = "mountain",
    brief = { prefix = "mountain_town_" },
    map = { rgb = { 0.62, 0.25, 0.75 } },
  },

  -- shops on the seafront, homes behind: a strip along its stretch of the coast road (Glenn, 2026-11-01: the coast
  -- road "where the beach towns grow out of"); off the ring, reached by the coast road
  ["beach town"] = {
    hub = "commercial",
    radius = { 1.0, 300, 600 },
    on_ring = false,
    links = { 0, 300 },
    buses = { 1, 6 },
    naming = "town",
    brief = { prefix = "beach_town_" },
    strip = { length = 900, rows = 3, row = 85, cross = 100, sea = 30 },
    map = { rgb = { 0.10, 0.62, 0.68 } },
  },
}
