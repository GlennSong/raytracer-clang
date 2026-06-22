-- hill_roads.lua — a terrain-aware road network on rolling hills, no city, so the
-- road can be studied in isolation (ADR-0046 study view). Three things act on the
-- road graph BEFORE block extraction (so blocks, if any, would follow for free):
--   * the tensor field bends its streets to follow the hillside CONTOURS where the
--     ground is steep (slope_align), instead of marching straight up the fall line;
--   * any street still steeper than max_grade is PRUNED — but connectivity-first, so
--     a steep street survives where it's the only bridge and is dropped where a
--     gentler way around exists;
--   * any fragments the bend/prune leave are STITCHED back so the result is one
--     connected network — no gaps.
-- On this terrain that takes the network from ~40% walkable streets / ~0.19 mean
-- grade (the terrain-blind tensor layout) to ~68% / ~0.13, while staying a single
-- coherent network. Compare with roads_terrain.lua, where the terrain-blind
-- generators lay roads regardless of slope and the *ground* is conformed to them.

local seed = 7

-- Smooth, large-scale rolling hills (low-frequency fbm, few octaves): real relief
-- with steep flanks, but long clean contours for the streets to sweep along.
local hills = terrain.fbm{ seed = seed, freq = 0.004, amp = 80, octaves = 2 }

-- Terrain-aware tensor network. Pass the heightfield in: streets follow the
-- contours on the steep flanks, streets over ~12% grade are pruned where avoidable,
-- and the network is stitched into one connected whole.
local lay = city.layout{
  pattern = "tensor", extent = 240, spacing = 72, step = 8,
  radial_decay = 170, radial_strength = 1.2, grid_strength = 1.0,
  terrain = hills, max_grade = 0.12, slope_align = 1.5, seed = seed,
}

-- Conform the ground to the streets and render terrain + road only — no buildings,
-- no lots, so the network reads cleanly against the hillside.
local land = terrain.conform(hills, lay, { margin = 3, falloff = 14 })

local m = model.new()
m:add(terrain.mesh(land, { size = 560, resolution = 300, color = { 0.40, 0.46, 0.30 } }))
m:add_solid(city.road_mesh(lay, { height = land, lift = 0.3, color = { 0.07, 0.07, 0.08 },
                                  sidewalk = 2.4, curb = 0.16, markings = true }))
return m
