-- hill_roads.lua — a terrain-aware road network on rolling hills, no city, so the
-- road can be studied in isolation (ADR-0046 study view). Two mechanisms are at
-- work, both BEFORE block extraction (so blocks, if any, would follow for free):
--   * the tensor field bends its avenues to follow the hillside CONTOURS where the
--     ground is steep (slope_align), instead of marching straight up the fall line;
--   * any street still steeper than max_grade is PRUNED, so only walkable streets
--     survive and the steep flanks are left bare.
-- On this terrain that takes the network from ~36% walkable streets (the
-- terrain-blind tensor layout) to ~75%, and roughly halves the mean grade.
-- Compare with roads_terrain.lua, where the terrain-blind generators lay roads
-- regardless of slope and the *ground* is conformed to them.

local seed = 7

-- Rolling hills: a low-frequency ridged massif (peaks ~120 m, flanks up to ~0.8
-- grade) plus a little fine texture — relief steep enough that the 12% grade limit
-- meaningfully shapes the network, but not the cliffs that would leave nothing.
local hills = terrain.ridged{ seed = seed, freq = 0.004, amp = 30, octaves = 3 }
hills = hills:add(terrain.fbm{ seed = seed + 3, freq = 0.02, amp = 2.5, octaves = 2 })

-- Terrain-aware tensor network. Pass the heightfield in: avenues follow contours
-- on the steep flanks, and streets over ~12% grade are pruned away.
local lay = city.layout{
  pattern = "tensor", extent = 240, spacing = 60, step = 8,
  radial_decay = 160, radial_strength = 1.2, grid_strength = 1.0,
  terrain = hills, max_grade = 0.12, slope_align = 1.8, seed = seed,
}

-- Conform the ground to the surviving streets and render terrain + road only —
-- no buildings, no lots, so the network reads cleanly against the hillside.
local land = terrain.conform(hills, lay, { margin = 3, falloff = 14 })

local m = model.new()
m:add(terrain.mesh(land, { size = 560, resolution = 300, color = { 0.40, 0.46, 0.30 } }))
m:add_solid(city.road_mesh(lay, { height = land, lift = 0.3, color = { 0.07, 0.07, 0.08 },
                                  sidewalk = 2.4, curb = 0.16, markings = true }))
return m
