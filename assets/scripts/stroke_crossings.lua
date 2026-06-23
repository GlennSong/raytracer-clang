-- stroke_crossings.lua — overlapping 2D spines (ADR-0048 study): grids, curve
-- crossings, a roundabout, a self-crossing figure-8, a diagonal over a grid, and a
-- multi-curve knot. Each spine is stroked independently with `city.stroke`, so where
-- spines cross the ribbons OVERLAP. To make that visible (rather than hidden as a
-- same-colour coplanar fill), crossing families get distinct colours and a tiny
-- height offset, so you see one ribbon passing over another — exactly the overlap a
-- non-overlapping network union (strokeNetwork) would merge into a shared junction.

local m = model.new()
m:add(mesh.place(scope{ origin = { -130, -0.4, -130 }, size = { 260, 0.4, 130 } }
        :box({ 0.56, 0.58, 0.52 }), { 0, 0, -65 }, 0))
m:add(mesh.place(scope{ origin = { -130, -0.4, 0 }, size = { 260, 0.4, 130 } }
        :box({ 0.56, 0.58, 0.52 }), { 0, 0, 65 }, 0))

local A = { 0.10, 0.10, 0.13 }   -- family A (asphalt)
local B = { 0.16, 0.10, 0.10 }   -- family B (rust) — drawn just above so crossings read
local C = { 0.10, 0.13, 0.18 }   -- family C (slate)

local function pts(fn, n, t0, t1)
  local p = {}
  for i = 0, n do local t = t0 + (t1 - t0) * i / n; local x, z = fn(t); p[#p+1] = { x = x, z = z } end
  return p
end
local function stroke(p, w, color, y, closed)
  m:add_solid(city.stroke{ points = p, width = w or 6, y = y or 0.05, color = color or A, closed = closed })
end
local function line(x0, z0, x1, z1) return { { x = x0, z = z0 }, { x = x1, z = z1 } } end

-- 1) GRID — perpendicular families crossing (centre -82, 82).
local gx, gz, n = -82, 82, 3
for i = 0, n do
  stroke(line(gx-30, gz-30+i*20, gx+30, gz-30+i*20), 5, A, 0.05)      -- E-W
  stroke(line(gx-30+i*20, gz-30, gx-30+i*20, gz+30), 5, B, 0.06)      -- N-S over
end

-- 2) CURVE CROSSING — two sine spines crossing (centre 0, 82).
stroke(pts(function(t) return t, 82 + 13*math.sin(t/9) end, 50, -28, 28), 6, A, 0.05)
stroke(pts(function(t) return 13*math.sin(t/9), 82 + t end, 50, -28, 28), 6, B, 0.06)

-- 3) ROUNDABOUT — a ring (closed) with spokes crossing it (centre 82, 82).
local rx, rz = 82, 82
stroke(pts(function(t) return rx+18*math.cos(t), rz+18*math.sin(t) end, 60, 0, 2*math.pi), 6, A, 0.05, true)
for k = 0, 3 do
  local a = k * math.pi/2
  stroke(line(rx+12*math.cos(a), rz+12*math.sin(a), rx+34*math.cos(a), rz+34*math.sin(a)), 6, B, 0.06)
end

-- 4) FIGURE-8 — a self-crossing closed spine (centre -82, -82).
stroke(pts(function(t) return -82 + 28*math.sin(t), -82 + 18*math.sin(2*t) end, 90, 0, 2*math.pi), 6, A, 0.05, true)

-- 5) DIAGONAL OVER A GRID — oblique crossings (centre 0, -82).
for i = 0, 2 do
  stroke(line(-28, -98+i*16, 28, -98+i*16), 5, A, 0.05)
  stroke(line(-28+i*28, -110, -28+i*28, -54), 5, A, 0.05)
end
stroke(line(-30, -110, 30, -54), 6, B, 0.06)                         -- the diagonal over

-- 6) MULTI-CURVE KNOT — three sine spines at 0/60/120 deg crossing (centre 82, -82).
local kx, kz = 82, -82
for j = 0, 2 do
  local ca, sa = math.cos(j*math.pi/3), math.sin(j*math.pi/3)
  local col = ({ A, B, C })[j+1]
  stroke(pts(function(t)
    local u, v = t, 10*math.sin(t/8)
    return kx + u*ca - v*sa, kz + u*sa + v*ca
  end, 50, -30, 30), 6, col, 0.05 + j*0.01)
end

return m
