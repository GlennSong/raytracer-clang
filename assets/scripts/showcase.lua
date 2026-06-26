-- showcase.lua — the ONE unified road system (unified-road-plan). Every road here is a spline in
-- one graph: the crossing RESOLVER (city.resolve) ties the loose centerlines into a connected net
-- sharing nodes, and the weld engine (city.solid) turns them into ONE volumetric surface — real
-- thickness, welded junctions, rounded curb returns, open block interiors — draped on gently
-- smoothed terrain. Lane markings are a TEXTURE (the RoadMarkings surface reads the road-local UV),
-- not geometry. A wide HIGHWAY connects to the city grid through on/off ramps that share its nodes,
-- and a roundabout fuses its ring with four spokes. One mesh, one pipeline.

local DARK  = { 0.085, 0.085, 0.095 }
local PAINT = material.new{ surface = "roadmarkings", roughness = 0.9 }

local function line(x0, z0, x1, z1) return { { x = x0, z = z0 }, { x = x1, z = z1 } } end
local function sample(fn, n, t0, t1)
  local p = {}
  for i = 0, n do local t = t0 + (t1 - t0) * i / n; local x, z = fn(t); p[#p + 1] = { x = x, z = z } end
  return p
end

local spines = {}
local function road(points, width) spines[#spines + 1] = { points = points, width = width } end

-- City grid (3x3 blocks), local streets.
for i = 0, 3 do
  road(line(-150, -60 + i * 40, -30, -60 + i * 40), 13)   -- east-west streets
  road(line(-150 + i * 40, -60, -150 + i * 40, 60), 13)   -- north-south streets
end

-- A roundabout west of the grid: a closed ring + four approach spokes that fuse into it.
local cx, cz, R = -215, 0, 28
road(sample(function(t) return cx + R * math.cos(t), cz + R * math.sin(t) end, 48, 0, 2 * math.pi), 12)
for k = 0, 3 do
  local a = k * math.pi / 2 + math.pi / 4
  road(line(cx + math.cos(a) * (R - 5), cz + math.sin(a) * (R - 5),
            cx + math.cos(a) * (R + 55), cz + math.sin(a) * (R + 55)), 12)
end

-- A wide multilane HIGHWAY sweeping north-south to the east, an arterial reaching out of the grid
-- to meet it, and two curved on/off RAMPS splicing the highway into the arterial — so the highway
-- CONNECTS to the city instead of overlapping it (resolve shares the nodes, the weld fuses them).
road(sample(function(t) return 130 + 22 * math.sin(t / 80), t end, 60, -130, 130), 30)
road(line(-30, 0, 105, 0), 20)
road(sample(function(t) return 105 + 25 * t, -48 * (1 - t) * t * 4 end, 24, 0, 1), 12)   -- on-ramp
road(sample(function(t) return 105 + 25 * t,  48 * (1 - t) * t * 4 end, 24, 0, 1), 12)   -- off-ramp

-- Gentle hills: the road follows them but irons out bumps (city.solid max_grade), so the deck reads
-- clean rather than heaving. Resolve every crossing into a shared-node graph, then weld one solid.
local land = terrain.warp(
  terrain.fbm{ seed = 11, freq = 0.0025, amp = 4, octaves = 4 },
  terrain.fbm{ seed = 3, freq = 0.02, amp = 1 }, 16)

local resolved = city.resolve{ spines = spines }

local m = model.new()
m:add(terrain.mesh(land, { size = 640, resolution = 240, color = { 0.34, 0.41, 0.27 } }))
-- y lifts the deck above the terrain; a higher max_grade hugs the gentle hills closely so the
-- ground never poke through the ironed deck.
m:add_solid(city.solid{
  spines = resolved.spines, height = land, y = 1.2, thickness = 0.6, corner_radius = 5,
  max_grade = 0.15, color = DARK, side_color = { 0.16, 0.16, 0.17 },
}, PAINT)
return m
