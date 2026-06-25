-- net_interchange.lua — a CLOVERLEAF interchange, built correctly: each loop is a 270°
-- (three-quarter) ramp that CONNECTS the two highways — it leaves one highway tangentially
-- (the exit/diverge), curves three-quarters of a circle, and rejoins the other highway
-- tangentially (the entrance/merge). city.union (SDF roadbed, ADR-0048) welds each ramp
-- into the highways it touches, so cars can actually get on and off. Flat (one level for
-- now; a real cloverleaf is grade-separated — that's the next step).
local m = model.new()
local DARK = { 0.09, 0.09, 0.10 }

local function pts(fn, n, t0, t1)
  local p = {}
  for i = 0, n do local t = t0 + (t1 - t0) * i / n; local x, z = fn(t); p[#p+1] = { x = x, z = z } end
  return p
end
local function line(x0, z0, x1, z1) return { { x = x0, z = z0 }, { x = x1, z = z1 } } end

local sp = {}
sp[#sp+1] = { points = line(-300, 0, 300, 0), width = 22 }   -- highway A (E-W)
sp[#sp+1] = { points = line(0, -300, 0, 300), width = 22 }   -- highway B (N-S)

-- One loop ramp per quadrant. A circle of radius R centred at (±R, ±R) is tangent to both
-- highway centrelines; we sweep the 270° arc that AVOIDS the inner quarter (the 90° nearest
-- the crossing), so the ramp's two ends sit tangentially on the two highways and the loop
-- bulges outward — exactly the three-quarter loop. The SDF union welds the ends in.
local R, RW = 52, 9
for _, q in ipairs({ {1,1}, {-1,1}, {1,-1}, {-1,-1} }) do
  local sx, sz = q[1], q[2]
  local cx, cz = sx * R, sz * R
  local oa = math.atan(-sz, -sx)            -- angle from the loop centre toward the crossing
  local a0, a1 = oa + math.rad(45), oa + math.rad(315)   -- the 270° that skips the inner quarter
  sp[#sp+1] = { width = RW,
                points = pts(function(t) return cx + R*math.cos(t), cz + R*math.sin(t) end, 64, a0, a1) }
end

m:add_solid(city.union{ spines = sp, cell = 0.5, y = 0.06, color = DARK })
return m
