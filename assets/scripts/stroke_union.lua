-- stroke_union.lua — the same overlapping spines as stroke_crossings.lua, but UNIONED
-- (ADR-0048): each group's spines are merged into ONE non-overlapping surface with
-- city.union, so every crossing is a clean shared junction (single colour, no stacked
-- ribbons). Compare side-by-side with stroke_crossings.lua.

local m = model.new()
m:add(mesh.place(scope{ origin = { -130, -0.4, -130 }, size = { 260, 0.4, 130 } }
        :box({ 0.56, 0.58, 0.52 }), { 0, 0, -65 }, 0))
m:add(mesh.place(scope{ origin = { -130, -0.4, 0 }, size = { 260, 0.4, 130 } }
        :box({ 0.56, 0.58, 0.52 }), { 0, 0, 65 }, 0))

local DARK = { 0.10, 0.10, 0.13 }
local CELL = 0.7

local function pts(fn, n, t0, t1)
  local p = {}
  for i = 0, n do local t = t0 + (t1 - t0) * i / n; local x, z = fn(t); p[#p+1] = { x = x, z = z } end
  return p
end
local function line(x0, z0, x1, z1) return { { x = x0, z = z0 }, { x = x1, z = z1 } } end
-- union a list of spines into one merged surface
local function merge(spines) m:add_solid(city.union{ spines = spines, cell = CELL, y = 0.05, color = DARK }) end

-- 1) GRID
do
  local sp, gx, gz, n = {}, -82, 82, 3
  for i = 0, n do
    sp[#sp+1] = { points = line(gx-30, gz-30+i*20, gx+30, gz-30+i*20), width = 5 }
    sp[#sp+1] = { points = line(gx-30+i*20, gz-30, gx-30+i*20, gz+30), width = 5 }
  end
  merge(sp)
end

-- 2) CURVE CROSSING
merge({
  { points = pts(function(t) return t, 82 + 13*math.sin(t/9) end, 50, -28, 28), width = 6 },
  { points = pts(function(t) return 13*math.sin(t/9), 82 + t end, 50, -28, 28), width = 6 },
})

-- 3) ROUNDABOUT — ring + spokes
do
  local rx, rz = 82, 82
  local sp = { { points = pts(function(t) return rx+18*math.cos(t), rz+18*math.sin(t) end, 60, 0, 2*math.pi), width = 6, closed = true } }
  for k = 0, 3 do
    local a = k * math.pi/2
    sp[#sp+1] = { points = line(rx+12*math.cos(a), rz+12*math.sin(a), rx+34*math.cos(a), rz+34*math.sin(a)), width = 6 }
  end
  merge(sp)
end

-- 4) FIGURE-8 (self-crossing single spine — union merges the crossing)
merge({ { points = pts(function(t) return -82 + 28*math.sin(t), -82 + 18*math.sin(2*t) end, 90, 0, 2*math.pi), width = 6, closed = true } })

-- 5) DIAGONAL OVER A GRID
do
  local sp = {}
  for i = 0, 2 do
    sp[#sp+1] = { points = line(-28, -98+i*16, 28, -98+i*16), width = 5 }
    sp[#sp+1] = { points = line(-28+i*28, -110, -28+i*28, -54), width = 5 }
  end
  sp[#sp+1] = { points = line(-30, -110, 30, -54), width = 6 }
  merge(sp)
end

-- 6) MULTI-CURVE KNOT — three spines crossing at one point
do
  local sp, kx, kz = {}, 82, -82
  for j = 0, 2 do
    local ca, sa = math.cos(j*math.pi/3), math.sin(j*math.pi/3)
    sp[#sp+1] = { width = 6, points = pts(function(t)
      local u, v = t, 10*math.sin(t/8)
      return kx + u*ca - v*sa, kz + u*sa + v*ca
    end, 50, -30, 30) }
  end
  merge(sp)
end

return m
