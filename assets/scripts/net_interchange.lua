-- net_interchange.lua — a grade-separated CLOVERLEAF with markings, barriers and a multilane
-- freeway below. Highway A (below) is the multilane freeway, at grade; highway B is a
-- multilane road on a viaduct over A, with parapet barriers and piers; each 270° loop ramp
-- climbs from A up to B and is extended at both ends with a merge stub so it connects onto
-- the roads it joins. Decks ride a vertical profile (city.deck, ADR-0054); lane markings and
-- barriers are baked on.
local m = model.new()
local DARK = { 0.09, 0.09, 0.10 }
local H = 7.0

local function line(x0, z0, x1, z1, k) k = k or 24
  local p = {}; for i = 0, k do local t = i / k; p[#p+1] = { x = x0 + (x1-x0)*t, z = z0 + (z1-z0)*t } end; return p
end

-- Highway A: the multilane freeway, at grade, with markings.
m:add_solid(city.deck{ points = line(-300, 0, 300, 0, 80), width = 26, height = 0,
                       thickness = 0.18, color = DARK, markings = true })

-- Highway B: a multilane road on a viaduct over A — piers (straddling A), parapet barriers,
-- markings. Rides up to H across the interchange and ramps down to grade beyond the loops.
do
  local N, pB, hB = 120, {}, {}
  for i = 0, N do
    local z = -300 + 600 * i / N
    pB[#pB+1] = { x = 0, z = z }
    local az = math.abs(z)
    hB[#hB+1] = (az <= 130) and H or (az <= 230 and H * (230 - az) / 100 or 0)
  end
  local pier_at = {}
  for _, pz in ipairs({ -120, -72, -24, 24, 72, 120 }) do
    pier_at[#pier_at+1] = math.floor((pz + 300) / 5 + 0.5) + 1
  end
  m:add_solid(city.deck{ points = pB, width = 22, heights = hB, thickness = 0.9, color = DARK,
                         piers = true, pier_at = pier_at, pier_depth = 3.5,
                         barriers = true, barrier_height = 0.9, markings = true })
end

-- Loop ramps: 270° arc tangent to both highways, climbing A->B, with merge stubs at both ends.
local R, RW = 52, 9
for _, q in ipairs({ {1,1}, {-1,1}, {1,-1}, {-1,-1} }) do
  local sx, sz = q[1], q[2]
  local cx, cz = sx * R, sz * R
  local oa = math.atan(-sz, -sx)
  local a0, a1, N = oa + math.rad(45), oa + math.rad(315), 64
  local arc = {}
  for i = 0, N do local t = a0 + (a1-a0)*i/N; arc[#arc+1] = { x = cx + R*math.cos(t), z = cz + R*math.sin(t) } end
  if math.abs(arc[1].z) >= math.abs(arc[1].x) then            -- make the A-end (z~0) come first
    local r = {}; for i = #arc, 1, -1 do r[#r+1] = arc[i] end; arc = r
  end
  local aEnd, bEnd = arc[1], arc[#arc]
  local pts = { { x = aEnd.x * 0.5, z = 0 } }                 -- merge stub onto highway A
  for _, q2 in ipairs(arc) do pts[#pts+1] = q2 end
  pts[#pts+1] = { x = 0, z = bEnd.z * 0.5 }                   -- merge stub onto highway B
  local h, n = { 0.06 }, N
  for i = 0, n do h[#h+1] = 0.06 + (H - 0.06) * i / n end
  h[#h+1] = H
  m:add_solid(city.deck{ points = pts, width = RW, heights = h, thickness = 0.5, color = DARK,
                         barriers = true, barrier_height = 0.8,
                         markings = true, center = false, lane_width = 9 })
end
return m
