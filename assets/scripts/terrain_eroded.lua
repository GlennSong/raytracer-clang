-- terrain_eroded.lua — the same composed terrain as terrain_demo, but run through
-- hydraulic + thermal erosion as a bake pass (ADR-0043). Droplets carve drainage
-- valleys and ridgelines; erode() returns a HeightField, so it still composes.

local hills = terrain.fbm{ seed = 7,  freq = 0.012, amp = 14, octaves = 5 }
local mtns  = terrain.ridged{ seed = 11, freq = 0.005, amp = 46, octaves = 6 }
local land  = terrain.warp(hills:add(mtns:scale(0.5)),
                           terrain.fbm{ seed = 3, freq = 0.02, amp = 1 }, 32)

-- Erode at a grid resolution; the result is a field again.
local carved = terrain.erode(land, { size = 400, resolution = 384,
                                      droplets = 90000, talus = 1.0, seed = 5 })

local m = model.new()
m:add(terrain.mesh(carved, { size = 400, resolution = 256,
                             color = {0.34, 0.42, 0.26} }))
return m
