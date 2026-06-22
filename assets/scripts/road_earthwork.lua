-- road_earthwork.lua — ONE road crossing a steep hill, showing the "hug <-> modify
-- the terrain" dial (ADR-0047, after Galin et al. 2010). The road carries its own
-- grade-limited elevation profile; `args.cut_fill` prices a cubic metre of earth:
--   * HIGH cut_fill -> moving earth is dear -> the road HUGS: it winds and
--     switchbacks around the hill, barely scratching the ground;
--   * LOW cut_fill  -> earth is cheap -> the road CUTS straight through: a short,
--     near-straight road that notches the hilltop and fills the hollows.
-- Either way the road never exceeds max_grade. The terrain is graded TO the road's
-- profile (terrain.conform reads each node's road height), so the cut notch and fill
-- embankment are real geometry, not a decal.

local cf   = (args and args.cut_fill) or 0.5
local seed = (args and args.seed) or 7
local amp  = (args and args.amp) or 130

local hills = terrain.fbm{ seed = seed, freq = 0.004, amp = amp, octaves = 2 }
local A = { x = 0, z = -220 }
local B = { x = 0, z =  220 }      -- the straight line crosses a ~0.9-grade peak

local smooth = (args and args.smooth) or false
local road = terrain.route(hills, { from = A, to = B, max_grade = 0.12, cell = 12,
                                    turn_penalty = 6, width = 14, cut_fill = cf,
                                    smooth = smooth })

-- Grade the ground to the road's own cut/fill profile (nodes carry their `y`).
local land = terrain.conform(hills, road, { margin = 4, falloff = 28 })

local m = model.new()
m:add(terrain.mesh(land, { size = 620, resolution = 340, color = { 0.42, 0.47, 0.32 } }))
m:add_solid(city.road_mesh(road, { height = land, lift = 0.4, color = { 0.08, 0.08, 0.09 },
                                   sidewalk = 2.2, curb = 0.16, markings = true }))
return m
