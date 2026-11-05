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
--   high_road     = bool                  a HIGH ROAD climbs from it over the range to the city across (a second pass)
--   interchanges  = n                     freeway diamonds it is given first (more first)
--   links         = { candidates, spacing }   link roads offered to the freeway, their ends this far apart (m)
--   buses         = { routes, stops }     its local bus network; 0 routes: none
--   signed_far    = bool                  named on the island's distance and control-city signs
--   limit_sign    = "TOWN LIMIT"
--   naming        = "city" | "town" | "mountain"   how place_names names it
--   layout        = "grown" | "terraces"   (not a strip) grown over its land by depth, the default; or a HILL TOWN: a
--                                       street along the back of each shelf its regrade cut, ramps between (needs
--                                       regrade.terrace)
--   brief = { prefix, area_scale, size_min, spoke_spacing, core_block = { l, s }, midtown_block = { l, s }, warp,
--             ring_spacing, spokes }      its street plan (island_world.cpp islandSiteBrief)
--   strip = { length, rows, row, cross, sea, promenade }   a STRIP town, laid along the road it grew from (a beach
--                                       town along its coast road): `length` m of it is the seafront street, `rows`
--                                       streets behind it every `row` m inland, cross streets every `cross` m, a
--                                       promenade `promenade` m out on the sand side (a beachfront row of blocks between;
--                                       0: none), its limits `sea` m beyond; `along` the road it grows from ("coast",
--                                       or "high": a high road), `both_sides` its rows either side of the spine, `main`
--                                       its spine's class ("arterial", the default; "collector"; "local")
--   regrade = { blur, max_slope, terrace, bank, feather, margin }   its ground levelled (procgen/regrade.h, ADR-0323):
--                                       over its disc (a strip: its outline) and `margin` m more, written into the
--                                       terrain block when the place is added
--   hub_reach = m                         (a strip town) its centre is its seafront LINE: commercial within this of it
--   mixes = { <district> = { { "recipe", weight }, ... } }   its own building mixes in place of the district ladders
--   civic = bool                          the city's full civic table (capitol, courthouse, hospital, university, ...)
--   landmarks = { { "landmark", count, min_lots = n, min_old_town = n, any = bool }, ... }   (not civic) its own:
--                                       `count` once it has `min_lots` lots (`min_old_town` old town); `any`: on any
--                                       district's lot when none of its own can carry one. Landmarks: school, church,
--                                       market_hall, police, fire_station, library, hospital, university, capitol,
--                                       courthouse, museum. Each place plans its own (the lot pass, per place).
--   map = { rgb = { r, g, b }, dot = px, label = m }
--   (a strip town with a promenade: the first and last blocks of its beachfront row are car parks for the beach's
--   visitors, written into its hub as `parking`)
-- }

settlements = {
  order = { "city", "town", "mountain town", "beach town", "foothill town", "hamlet" },

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
    civic = true,
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
    -- its school and church, and a market hall in an old town
    landmarks = { { "school", 1, min_lots = 10, any = true }, { "church", 1, min_lots = 8, any = true },
                  { "market_hall", 1, min_old_town = 6 } },
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
    strip = { length = 900, rows = 3, row = 85, cross = 100, sea = 30, promenade = 55 },
    hub_reach = 60,   -- the seafront's row of blocks
    -- a town's school and church, a market hall on the seafront and a police post by the beach
    landmarks = { { "school", 1, min_lots = 10, any = true }, { "church", 1, min_lots = 8, any = true },
                  { "market_hall", 1, any = true }, { "police", 1, any = true } },
    -- shops, cafes, flats over shops and a small hotel on the seafront; cottages, villas and garden flats behind
    mixes = {
      commercial = { { "mixed_use", 28 }, { "brick_shop", 18 }, { "oldtown_cafe", 12 }, { "corner_shop", 8 }, { "hotel", 10 },
                     { "terrace_condo", 8 }, { "walkup_homes", 6 }, { "supermarket", 4 }, { "plaza", 3 }, { "pocket_park", 3 } },
      residential = { { "cottage", 14 }, { "villa", 12 }, { "modern_house", 12 }, { "bungalow", 10 }, { "duplex", 8 },
                      { "yard_house", 8 }, { "garden_condo", 12 }, { "walkup_homes", 8 }, { "terrace_condo", 6 },
                      { "rowhouses", 6 }, { "pocket_park", 4 } },
    },
    map = { rgb = { 0.10, 0.62, 0.68 } },
  },

  -- a small town on the lower slopes inland of the ring, its ground regraded, the high road from it over the range
  -- (~/.claude/plans/mountain-towns.md; Glenn, 2026-11-04: "a town at the base of the mountain and then a road that
  -- goes into the mountains")
  ["foothill town"] = {
    hub = "commercial",
    radius = { 1.0, 200, 400 },
    on_ring = false,
    high_road = true,
    links = { 2, 300 },
    buses = { 1, 6 },
    layout = "terraces",   -- a hill town on its shelves (Glenn: "towns with terraces")
    brief = { prefix = "foothill_town_", size_min = 400, spokes = 5 },
    -- a hill town's own: two to four storeys, shops and cafes in its middle, houses and rows on its shelves (the town
    -- ladders built it of city walk-ups six to eight storeys high)
    mixes = {
      commercial = { { "brick_shop", 22 }, { "oldtown_cafe", 16 }, { "corner_shop", 14 }, { "walkup_homes", 16 }, { "rowhouses", 14 },
                     { "terrace_condo", 10 }, { "plaza", 4 }, { "pocket_park", 4 } },
      residential = { { "rowhouses", 22 }, { "cottage", 18 }, { "walkup_homes", 14 }, { "duplex", 12 }, { "yard_house", 12 },
                      { "bungalow", 10 }, { "garden_condo", 8 }, { "pocket_park", 4 } },
    },
    regrade = { max_slope = 0.10, terrace = 6, bank = 0.3, feather = 120, margin = 80 },
    map = { rgb = { 0.55, 0.40, 0.20 } },
  },

  -- a handful of buildings on a street up in the mountains (Glenn: "like a street with some buildings"): a short strip
  -- along the high road, a row of lots either side, its band of ground regraded
  hamlet = {
    hub = "commercial",
    radius = { 1.0, 120, 250 },
    on_ring = false,
    links = { 0, 300 },
    buses = { 0, 6 },
    naming = "mountain",
    brief = { prefix = "hamlet_" },
    strip = { length = 350, rows = 1, row = 55, cross = 120, sea = 0, along = "high", both_sides = true, main = "local" },
    hub_reach = 60,
    -- a shop or two and a cafe on the street (an inn is owed: "hotel" is a city's tower); cottages and houses behind
    mixes = {
      commercial = { { "oldtown_cafe", 20 }, { "corner_shop", 18 }, { "brick_shop", 12 }, { "cottage", 30 }, { "bungalow", 20 } },
      residential = { { "cottage", 40 }, { "bungalow", 25 }, { "yard_house", 20 }, { "villa", 15 } },
    },
    landmarks = { { "church", 1, any = true } },
    regrade = { blur = 60, max_slope = 0.10, feather = 60, margin = 40 },
    map = { rgb = { 0.45, 0.30, 0.15 }, dot = 3.5, label = 110 },
  },
}
