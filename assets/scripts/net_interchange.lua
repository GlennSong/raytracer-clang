-- net_interchange.lua — a CLOVERLEAF interchange: two highways crossing, with four loop
-- ramps (the on/off ramps) in the quadrants. Authored as spines and merged by city.union
-- (SDF roadbed, ADR-0048), so every loop welds tangentially into the highways it joins —
-- a clean entrance/exit, no overlap. Flat (one level); grade separation is a later pass.
local m = model.new()
local DARK = { 0.09, 0.09, 0.10 }

local function pts(fn, n, t0, t1)
  local p = {}
  for i = 0, n do local t = t0 + (t1 - t0) * i / n; local x, z = fn(t); p[#p+1] = { x = x, z = z } end
  return p
end
local function line(x0, z0, x1, z1) return { { x = x0, z = z0 }, { x = x1, z = z1 } } end

local sp = {}
sp[#sp+1] = { points = line(-280, 0, 280, 0), width = 22 }   -- highway A (E-W)
sp[#sp+1] = { points = line(0, -280, 0, 280), width = 22 }   -- highway B (N-S)

-- Four loop ramps: a circle in each quadrant, tangent to both highways, that carries a
-- right-turn movement around 270 degrees (the cloverleaf signature).
local R, OFF, RW = 46, 12, 9
for _, c in ipairs({ {1,1}, {-1,1}, {-1,-1}, {1,-1} }) do
  local cx, cz = c[1] * (R + OFF), c[2] * (R + OFF)
  sp[#sp+1] = { width = RW, closed = true,
                points = pts(function(t) return cx + R*math.cos(t), cz + R*math.sin(t) end, 72, 0, 2*math.pi) }
end

m:add_solid(city.union{ spines = sp, cell = 0.5, y = 0.06, color = DARK })
return m
