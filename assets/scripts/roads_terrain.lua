-- roads_terrain.lua — just the road network on terrain, no buildings (ADR-0044
-- test view). Shows urban roads CONFORMING the landscape: each street is a flat
-- (or single-incline) corridor and the terrain is cut/filled to meet it, instead
-- of the road rippling over every bump. Steep hills make the effect obvious.

local seed = 7
local base = terrain.fbm{ seed = seed, freq = 0.02, amp = 36, octaves = 5 }
base = terrain.warp(base, terrain.fbm{ seed = 2, freq = 0.02, amp = 1 }, 24)

-- Swap pattern = "grid" | "radial" to compare generators.
local lay = city.layout{ pattern = "radial", extent = 220, ring_spacing = 55,
                         spokes = 14, jitter = 0.06, seed = seed }

-- Conform the terrain to the network: flat road corridors, ground eased back to
-- the natural slope across `falloff`. Everything downstream samples `land`.
local land = terrain.conform(base, lay, { margin = 3, falloff = 12 })

local m = model.new()
m:add(terrain.mesh(land, { size = 520, resolution = 260, color = {0.36, 0.44, 0.28} }))

-- A connected road surface (ribbons trimmed to the junctions, gaps filled with
-- intersection pads), riding the now-flat corridors just clear of the ground.
m:add_solid(city.road_mesh(lay, { height = land, lift = 0.3, color = {0.07, 0.07, 0.08},
                                  sidewalk = 2.5, curb = 0.16, markings = true,
                                  mark_width = 0.18, crosswalks = true }))

return m
