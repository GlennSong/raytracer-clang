-- net_city.lua — the road system together (just roads): a GRID city and a RADIAL city linked
-- by a surface ARTERIAL, an elevated FREEWAY crossing that arterial at a CLOVERLEAF interchange
-- (4 loop ramps climbing from the surface arterial up to the freeway deck), with piers,
-- barriers and lane markings. Surface streets = analytic road mesh; freeway/ramps = city.deck
-- (a vertical profile, ADR-0054).
local m = model.new()
local DARK = { 0.09, 0.09, 0.10 }
local H = 7.5                          -- freeway deck height (clears the arterial below)

local function pts(fn, n, t0, t1)
  local p = {}
  for i = 0, n do local t = t0 + (t1 - t0) * i / n; local x, z = fn(t); p[#p+1] = { x = x, z = z } end
  return p
end

-- Two surface cities, built at the origin then placed left / right.
local look = { lift = 0.25, color = DARK, sidewalk = 2.2, curb = 0.15, markings = true,
               mark_width = 0.18, crosswalks = true }
m:add_solid(mesh.translate(city.road_mesh(
  city.layout{ pattern = "grid", extent = 120, cell_size = 44, jitter = 0.10, seed = 3 }, look),
  { -330, 0, 0 }))
m:add_solid(mesh.translate(city.road_mesh(
  city.layout{ pattern = "radial", extent = 120, ring_spacing = 34, spokes = 8, jitter = 0.05, seed = 5 }, look),
  { 330, 0, 0 }))

-- Arterial: a surface multilane road (E-W) linking the two cities, passing under the freeway.
do
  local p, h = {}, {}
  for x = -235, 235, 10 do p[#p+1] = { x = x, z = 0 }; h[#h+1] = 0.05 end
  m:add_solid(city.deck{ points = p, width = 18, heights = h, thickness = 0.15, color = DARK, markings = true })
end

-- Freeway: an elevated viaduct (N-S) crossing the arterial, ramping to grade past the clover,
-- on piers, with barriers and divided-road markings.
do
  local N, p, h = 110, {}, {}
  for i = 0, N do
    local z = -300 + 600 * i / N
    p[#p+1] = { x = 0, z = z }
    local az = math.abs(z)
    h[#h+1] = (az <= 150) and H or (az <= 250 and H * (250 - az) / 100 or 0)
  end
  m:add_solid(city.deck{ points = p, width = 24, heights = h, thickness = 1.0, color = DARK,
                         piers = true, pier_spacing = 56, barriers = true, barrier_height = 0.9,
                         markings = true })
end

-- Cloverleaf: 4 loop ramps, each a 270° arc tangent to the arterial (surface, y=0) and the
-- freeway (deck, y=H), climbing between them, tapering to a gore nose at each merge.
local R, RW = 56, 9
for _, q in ipairs({ {1,1}, {-1,1}, {1,-1}, {-1,-1} }) do
  local sx, sz = q[1], q[2]
  local cx, cz = sx * R, sz * R
  local oa = math.atan(-sz, -sx)
  local arc = pts(function(t) return cx + R*math.cos(t), cz + R*math.sin(t) end, 60,
                  oa + math.rad(45), oa + math.rad(315))
  if math.abs(arc[1].z) >= math.abs(arc[1].x) then         -- arterial-end (z~0) first
    local r = {}; for i = #arc, 1, -1 do r[#r+1] = arc[i] end; arc = r
  end
  local aEnd, bEnd = arc[1], arc[#arc]
  local pp = { { x = aEnd.x * 0.5, z = 0 } }               -- stub onto the arterial
  for _, q2 in ipairs(arc) do pp[#pp+1] = q2 end
  pp[#pp+1] = { x = 0, z = bEnd.z * 0.5 }                  -- stub onto the freeway
  local hh, ww, n = { 0.05 }, { 2.5 }, #arc - 1
  for i = 0, n do hh[#hh+1] = 0.05 + (H - 0.05) * i / n; ww[#ww+1] = RW end
  hh[#hh+1] = H; ww[#ww+1] = 2.5
  m:add_solid(city.deck{ points = pp, widths = ww, heights = hh, thickness = 0.5, color = DARK,
                         barriers = true, barrier_height = 0.8 })
end
return m
