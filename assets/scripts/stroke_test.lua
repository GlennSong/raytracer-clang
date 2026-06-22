-- stroke_test.lua — back-to-basics (ADR-0048): stroke flat curves into ribbons at
-- various widths and shapes, terrain-free, so the path-stroking primitive can be
-- judged on its own. Each ribbon is `city.stroke` of a centerline; the hard cases
-- (tight hairpin, circle, variable width) must read cleanly with no fold/overlap.

local m = model.new()
local function pts(fn, n, t0, t1)
  local p = {}
  for i = 0, n do
    local t = t0 + (t1 - t0) * i / n
    local x, z = fn(t)
    p[#p + 1] = { x = x, z = z }
  end
  return p
end

-- a flat ground slab to see the ribbons against
m:add(mesh.place(scope{ origin = { -130, -0.2, -130 }, size = { 260, 0.2, 260 } }
        :box({ 0.55, 0.58, 0.5 }), { 0, 0, 0 }, 0))

local DARK = { 0.08, 0.08, 0.10 }

-- 1) Circle (closed loop), constant width. A closed loop needs DISTINCT points (no
-- duplicated first/last), else the seam degenerates.
local circle = {}
for i = 0, 47 do
  local t = 2*math.pi * i / 48
  circle[#circle + 1] = { x = -80 + 28*math.cos(t), z = 80 + 28*math.sin(t) }
end
m:add_solid(city.stroke{ closed = true, width = 10, y = 0.05, color = DARK, points = circle })

-- 2) S-curve (sine), constant width.
m:add_solid(city.stroke{ width = 9, y = 0.05, color = DARK,
  points = pts(function(t) return t, 80 + 30*math.sin(t/26) end, 60, -10, 130) })

-- 3) Tight hairpin / switchback (the case that kept folding), constant width.
m:add_solid(city.stroke{ width = 9, y = 0.05, color = DARK, points = {
  { x = -110, z = -20 }, { x = -20, z = -20 }, { x = -16, z = -26 },
  { x = -22, z = -32 }, { x = -110, z = -32 } } })

-- 4) Variable-width taper along a gentle curve (10 -> 2 m).
local vp, vw = {}, {}
for i = 0, 40 do
  local t = i / 40
  vp[#vp + 1] = { x = 10 + t*110, z = -70 + 26*math.sin(t*2.4) }
  vw[#vw + 1] = 10 - 8*t
end
m:add_solid(city.stroke{ points = vp, widths = vw, y = 0.05, color = DARK })

-- 5) Spiral, constant width (curvature ramps up — stress the joins).
m:add_solid(city.stroke{ width = 6, y = 0.05, color = DARK,
  points = pts(function(t) return -70 + t*math.cos(t*3), -95 + t*math.sin(t*3) end, 80, 1, 11) })

return m
