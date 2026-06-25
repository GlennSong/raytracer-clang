-- net_city.lua — everything together, just roads. A GRID city and a RADIAL city (surface
-- streets, analytic road mesh) with an elevated FREEWAY viaduct (a gentle S-curve) flying
-- OVER both as an overpass, on piers, and two on/off RAMPS (Bezier curves) descending into
-- each city. Surface = city.road_mesh; freeway + ramps = city.deck (a vertical profile,
-- ADR-0054). Shows the analytic join (grid/radial junctions) and the deck/grade-separation
-- working in one map.
local m = model.new()
local DARK = { 0.09, 0.09, 0.10 }
local H = 7.5                       -- freeway deck height (clears the surface streets below)

local function bezier(P0, T0, P3, T3, k, n)
  local P1 = { P0[1] + T0[1]*k, P0[2] + T0[2]*k }
  local P2 = { P3[1] + T3[1]*k, P3[2] + T3[2]*k }
  local p = {}
  for i = 0, n do
    local t = i / n; local u = 1 - t
    p[#p+1] = { x = u*u*u*P0[1] + 3*u*u*t*P1[1] + 3*u*t*t*P2[1] + t*t*t*P3[1],
                z = u*u*u*P0[2] + 3*u*u*t*P1[2] + 3*u*t*t*P2[2] + t*t*t*P3[2] }
  end
  return p
end
local function fwz(x) return 34 * math.sin(x / 155) end       -- the freeway's S-curve centreline

-- Surface cities: build each generator at the origin, then place it in its half of the map.
local look = { lift = 0.25, color = DARK, sidewalk = 2.2, curb = 0.15, markings = true,
               mark_width = 0.18, crosswalks = true }
m:add_solid(mesh.translate(city.road_mesh(
  city.layout{ pattern = "grid", extent = 120, cell_size = 44, jitter = 0.10, seed = 3 }, look),
  { -250, 0, 70 }))
m:add_solid(mesh.translate(city.road_mesh(
  city.layout{ pattern = "radial", extent = 120, ring_spacing = 34, spokes = 8, jitter = 0.05, seed = 5 }, look),
  { 250, 0, 70 }))

-- Freeway: a gently S-curved viaduct across the top, over both cities, on piers with barriers
-- and divided-road lane markings.
do
  local p, h = {}, {}
  for x = -440, 440, 14 do p[#p+1] = { x = x, z = fwz(x) }; h[#h+1] = H end
  m:add_solid(city.deck{ points = p, width = 26, heights = h, thickness = 1.0, color = DARK,
                         piers = true, pier_spacing = 58, barriers = true, barrier_height = 0.9,
                         markings = true })
end

-- On/off ramp: a Bezier from the freeway (height H) curving down into a city (height 0), with
-- a gore taper at each end.
local function ramp(fx, cx, cz)
  local fz = fwz(fx)
  local arc = bezier({ fx, fz }, { (fx < 0) and -1 or 1, 0 },
                     { cx, cz }, { 0, (cz > fz) and 1 or -1 }, 80, 44)
  local n = #arc - 1
  local h, w = {}, {}
  for i = 0, n do
    h[#h+1] = H * (1 - i / n)
    w[#w+1] = (i < 3 or i > n - 3) and 3.0 or 8.0          -- taper to a nose at both ends
  end
  m:add_solid(city.deck{ points = arc, widths = w, heights = h, thickness = 0.5, color = DARK,
                         barriers = true, barrier_height = 0.8 })
end
ramp(-205, -250, 95)
ramp(205, 250, 95)
return m
