-- roads_terrain.lua — just the road network draped over terrain, no buildings
-- (ADR-0044 test view). Shows a generator's pattern hugging the hills: each road
-- is segmented and each segment seated at the terrain height under it.

local seed = 7
local hills = terrain.fbm{ seed = seed, freq = 0.012, amp = 7, octaves = 5 }
local land  = terrain.warp(hills, terrain.fbm{ seed = 2, freq = 0.02, amp = 1 }, 28)

local m = model.new()
m:add(terrain.mesh(land, { size = 520, resolution = 240, color = {0.36, 0.44, 0.28} }))

-- Swap pattern = "grid" | "radial" to compare generators.
local lay = city.layout{ pattern = "radial", extent = 220, ring_spacing = 55,
                         spokes = 14, jitter = 0.06, seed = seed }

-- A connected road surface: ribbons trimmed back to the junctions, the gaps
-- filled with intersection pads (city.road_mesh), draped on the terrain.
m:add(city.road_mesh(lay, { height = land, lift = 0.6, color = {0.07, 0.07, 0.08} }))

return m
