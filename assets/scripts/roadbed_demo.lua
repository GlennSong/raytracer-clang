-- roadbed_demo.lua — the union taken to a full ROADBED (ADR-0048): a merged road
-- network with raised sidewalks + curbs, draped over hilly 3D terrain and textured
-- with a procedural asphalt/concrete grain. The carriageway is {sdf<0}, the sidewalk
-- the {0..width} band, the curb the step between — all from one distance field, so
-- junctions merge and sidewalks wrap the whole network with no special cases.

local hills = terrain.fbm{ seed = 4, freq = 0.011, amp = 14, octaves = 3 }

local function line(x0, z0, x1, z1) return { { x = x0, z = z0 }, { x = x1, z = z1 } } end
local function pts(fn, n, t0, t1)
  local p = {}
  for i = 0, n do local t = t0 + (t1 - t0) * i / n; local x, z = fn(t); p[#p+1] = { x = x, z = z } end
  return p
end
local function ring(cx, cz, r, n)
  local p = {}
  for i = 0, n - 1 do local t = 2*math.pi*i/n; p[#p+1] = { x = cx + r*math.cos(t), z = cz + r*math.sin(t) } end
  return p
end

-- A small network of spines: a curved arterial, a grid of cross streets, a roundabout.
local spines = {}
spines[#spines+1] = { width = 13, points = pts(function(t) return t, -45 + 14*math.sin(t/38) end, 70, -120, 120) }
for gx = -90, 90, 45 do spines[#spines+1] = { width = 9, points = line(gx, -90, gx, 20) } end
for gz = -75, -30, 45 do spines[#spines+1] = { width = 9, points = line(-110, gz, 110, gz) } end
spines[#spines+1] = { width = 9, closed = true, points = ring(60, 55, 17, 44) }   -- roundabout
for k = 0, 3 do                                                                   -- its spokes
  local a = k * math.pi/2
  spines[#spines+1] = { width = 9, points = line(60+12*math.cos(a), 55+12*math.sin(a),
                                                 60+34*math.cos(a), 55+34*math.sin(a)) }
end

-- Asphalt/concrete grain: a light grayscale texture that MODULATES the roadbed's
-- per-band vertex colours (dark road, light sidewalk), adding surface detail.
local grit  = texture.fbm{ seed = 7, scale = 40, octaves = 4 }:scale_bias(0.25, 0.82)
local grain = texture.bake_color(grit, { 0.70, 0.70, 0.72 }, { 1.0, 1.0, 1.0 }, 256)
local surf  = material.new{ albedo = grain, roughness = 0.93, tile = 7 }

local bed = city.roadbed{ spines = spines, cell = 0.6, sidewalk = 2.4, curb = 0.2,
                          lift = 0.06, height = hills }

local m = model.new()
m:add(terrain.mesh(hills, { size = 330, resolution = 240, color = { 0.37, 0.45, 0.28 } }))
m:add_solid(bed, surf)
return m
