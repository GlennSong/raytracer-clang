-- net_weld.lua — the unified polygonal JOIN engine (city.weld): exact ribbon outlines welded
-- by boolean union, no SDF grid and no analytic pad. Left: the 8-spoke hub (the original
-- "too many spokes" case) welded into one clean star. Right: a curved S-road + a cross,
-- welded. Sharp, light, real polygons.
local m = model.new()
local DARK = { 0.09, 0.09, 0.10 }
local function line(x0, z0, x1, z1) return { { x = x0, z = z0 }, { x = x1, z = z1 } } end
local function pts(fn, n, t0, t1)
  local p = {}
  for i = 0, n do local t = t0 + (t1 - t0) * i / n; local x, z = fn(t); p[#p+1] = { x = x, z = z } end
  return p
end

-- 8-spoke hub, welded into one star
local hub = {}
for i = 0, 7 do
  local a = i * math.pi / 4
  hub[#hub+1] = { points = line(-150, 0, -150 + 95*math.cos(a), 95*math.sin(a)), width = 12 }
end
m:add_solid(city.solid{ spines = hub, y = 0.4, thickness = 0.8, color = DARK, corner_radius = 5 })

-- a curved S-road crossed by a straight road (welded)
m:add_solid(city.solid{ y = 0.4, thickness = 0.8, color = DARK, corner_radius = 5, spines = {
  { points = pts(function(t) return 150 + t, 40*math.sin(t/55) end, 60, -120, 120), width = 14 },
  { points = line(150, -110, 150, 110), width = 12 },
} })

-- a small welded GRID: enclosed block interiors stay open (hole bridging)
local g = {}
for i = 0, 3 do
  g[#g+1] = { points = line(-60, -150 + i*36, 60, -150 + i*36), width = 13 }
  g[#g+1] = { points = line(-60 + i*40, -150, -60 + i*40, -42), width = 13 }
end
m:add_solid(city.solid{ spines = g, y = 0.4, thickness = 0.8, color = DARK, corner_radius = 4 })
return m
