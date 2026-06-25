-- net_interchange.lua — a GRADE-SEPARATED cloverleaf. Highway A runs at grade; highway B
-- is a viaduct that flies over A at the crossing (city.deck rides a vertical profile, ADR-
-- 0054) on abutment piers; each 270° loop ramp CLIMBS from A's level up to B's deck, so the
-- loop carries you off one highway and up onto the other. Only the central crossing is
-- separated — the loops are gentle grades (270° of arc), as in a real cloverleaf.
local m = model.new()
local DARK = { 0.09, 0.09, 0.10 }
local H = 7.0                      -- viaduct clearance height (A passes under B)

local function pts(fn, n, t0, t1)
  local p = {}
  for i = 0, n do local t = t0 + (t1 - t0) * i / n; local x, z = fn(t); p[#p+1] = { x = x, z = z } end
  return p
end
local function line(x0, z0, x1, z1) return { { x = x0, z = z0 }, { x = x1, z = z1 } } end

-- Highway A: at grade, welded flat (SDF roadbed).
m:add_solid(city.union{ spines = { { points = line(-300, 0, 300, 0), width = 22 } },
                        cell = 0.6, y = 0.05, color = DARK })

-- Highway B: a viaduct at height H across the interchange, ramping down to grade beyond the
-- loops, carried on piers that straddle A (placed off the A carriageway, never on it).
do
  local N, pB, hB = 120, {}, {}
  for i = 0, N do
    local z = -300 + 600 * i / N
    pB[#pB + 1] = { x = 0, z = z }
    local az = math.abs(z)
    hB[#hB + 1] = (az <= 130) and H or (az <= 230 and H * (230 - az) / 100 or 0)
  end
  local pier_at = {}
  for _, pz in ipairs({ -120, -72, -22, 22, 72, 120 }) do
    pier_at[#pier_at + 1] = math.floor((pz + 300) / 5 + 0.5) + 1   -- z -> point index (1-based)
  end
  m:add_solid(city.deck{ points = pB, width = 22, heights = hB, thickness = 0.9,
                         color = DARK, piers = true, pier_at = pier_at, pier_depth = 3.5 })
end

-- Loops: each a 270° ramp tangent to both highways, climbing from A's level (0) to B's (H).
local R, RW = 52, 9
for _, q in ipairs({ {1,1}, {-1,1}, {1,-1}, {-1,-1} }) do
  local sx, sz = q[1], q[2]
  local cx, cz = sx * R, sz * R
  local oa = math.atan(-sz, -sx)
  local p = pts(function(t) return cx + R * math.cos(t), cz + R * math.sin(t) end,
                64, oa + math.rad(45), oa + math.rad(315))
  -- The end on highway A (z ~ 0) sits at grade; the end on highway B (x ~ 0) sits at H.
  local aAtStart = math.abs(p[1].z) < math.abs(p[1].x)
  local h, n = {}, #p - 1
  for i = 0, n do h[i + 1] = aAtStart and (H * i / n) or (H * (n - i) / n) end
  m:add_solid(city.deck{ points = p, width = RW, heights = h, thickness = 0.5, color = DARK })
end
return m
