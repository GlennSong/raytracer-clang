-- PIECES FOR A HOME'S GARDEN AND PORCH, each with a part that moves (#191):
--
--   gate           a picket gate between two posts; it swings either way (`gate`)
--   porch_swing    a slatted bench hung on chains from a frame; it swings (`seat`)
--   bbq            a kettle barbecue on three legs; its lid lifts at the back (`lid`)
--   rotary_dryer   a washing line that turns in the wind: four arms on a pole (`head`), with washing on it
--   weathervane    an arrow that turns with the wind (`vane`) over the four points of the compass, on a short pole
--                  (put it on a roof's ridge)
--   wind_chime     five tubes hung from a garden hook, each swinging (`tube_1` ...), and the striker between them (`striker`)
--
-- Each stands on y = 0 at its own origin, its front toward +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind  = { "gate", "porch_swing", "bbq", "rotary_dryer", "weathervane", "wind_chime" },
  wood  = kit.wood_names,
  paint = { "green", "red", "blue", "yellow", "orange" },
}
local wood, paint = kit.wood(a.wood), kit.paint[a.paint]
local m = model.new()
local far = m:level(50)
local c = kit.maker()
local IRON = kit.BLACK * 1.6

if a.kind == "gate" then
  local W, H = 1.0, 1.0
  for i, x in ipairs{ -W / 2 - 0.05, W / 2 + 0.05 } do
    c.box("wood", V(x, (H + 0.15) / 2, 0), V(0.1, H + 0.15, 0.1), 0.008, wood, 1)
    c.mesh("wood", mesh.translate(mesh.rotate_y(mesh.lathe{ profile = { { 0, 0 }, { 0.075, 0 }, { 0, 0.06 } }, segments = 4 }, math.pi / 4), { x, H + 0.15, 0 }), wood)
    m:shape("post_" .. i, { box = { 0.1, H + 0.15, 0.1 }, pos = { x, (H + 0.15) / 2, 0 } })
  end
  kit.put(m, m, c)
  local gate = m:node("gate", { pos = { -W / 2 + 0.01, 0.08, 0 }, motion = { turn = { 0, -1, 0 }, min = -95, max = 95 } })
  local k = kit.maker()
  local w = W - 0.03
  for i = 0, 6 do
    local x = 0.04 + i * (w - 0.08) / 6
    k.box("wood", V(x, (H - 0.1) / 2, 0.012), V(0.07, H - 0.1, 0.02), 0.006, paint, 1)
  end
  for _, y in ipairs{ 0.15, H - 0.3 } do k.box("wood", V(w / 2, y, -0.012), V(w, 0.07, 0.025), 0.006, paint, 1) end
  k.post("wood", V(0.04, 0.18, -0.012), V(w - 0.04, H - 0.33, -0.012), 0.05, 0.05, paint)      -- the brace
  k.box("metal", V(w - 0.03, H - 0.3, 0.03), V(0.08, 0.02, 0.012), 0.004, IRON, 1)             -- the latch
  kit.put(m, gate, k)
  kit.far_box(m, gate, "wood", { w, H - 0.1, 0.03 }, { w / 2, (H - 0.1) / 2, 0 }, paint, far)
  gate:shape("gate", { box = { w, H - 0.1, 0.04 }, pos = { w / 2, (H - 0.1) / 2, 0 } })
  gate:marker("latch", { pos = { w - 0.03, H - 0.3, 0.05 }, role = "hold" })
elseif a.kind == "porch_swing" then
  local X, H, SPREAD, BW = 0.95, 2.0, 0.75, 1.4
  for i, x in ipairs{ -X, X } do
    for _, z in ipairs{ -SPREAD, SPREAD } do c.post("wood", V(x, 0, z), V(x, H, 0), 0.08, 0.08, wood) end
    c.box("wood", V(x, H * 0.42, 0), V(0.05, 0.07, SPREAD * 1.16), 0.006, wood, 1)
    m:shape("end_" .. i, { box = { 0.08, H, 0.2 }, pos = { x, H / 2, 0 } })
  end
  c.box("wood", V(0, H, 0), V(2 * X + 0.2, 0.09, 0.09), 0.01, wood, 1)
  kit.put(m, m, c)
  local seat = m:node("seat", { pos = { 0, H, 0 }, motion = { turn = { 1, 0, 0 }, min = -25, max = 25 } })
  local k = kit.maker()
  local SY = -H + 0.45
  for i = 0, 5 do k.box("wood", V(0, SY, -0.2 + i * 0.085), V(BW, 0.02, 0.07), 0.006, paint, 1) end           -- the seat's slats
  for i = 0, 4 do k.box("wood", V(0, SY + 0.12 + i * 0.09, -0.26 - i * 0.02), V(BW, 0.07, 0.02), 0.006, paint, 1) end   -- the back's
  for _, x in ipairs{ -BW / 2 + 0.03, BW / 2 - 0.03 } do
    k.box("wood", V(x, SY - 0.03, 0), V(0.04, 0.04, 0.5), 0.006, paint, 1)
    k.post("wood", V(x, SY - 0.03, -0.25), V(x, SY + 0.55, -0.35), 0.04, 0.04, paint)
    k.box("wood", V(x, SY + 0.22, 0), V(0.05, 0.025, 0.52), 0.006, paint, 1)                                  -- the arm
    k.post("wood", V(x, SY, 0.22), V(x, SY + 0.21, 0.22), 0.035, 0.035, paint)
    for _, z in ipairs{ -0.24, 0.22 } do k.rod("metal", V(x, -0.04, 0), V(x, SY + 0.23, z), 0.006, kit.STEEL, 6) end   -- the chains
  end
  kit.put(m, seat, k)
  kit.far_box(m, seat, "wood", { BW, 0.5, 0.5 }, { 0, SY + 0.2, -0.05 }, paint, far)
  seat:shape("seat", { box = { BW, 0.06, 0.5 }, pos = { 0, SY - 0.01, 0 }, friction = 0.8 })
  seat:shape("back", { box = { BW, 0.5, 0.04 }, pos = { 0, SY + 0.3, -0.3 } })
  for i, x in ipairs{ -0.35, 0.35 } do seat:marker("seat_" .. i, { pos = { x, SY + 0.01, 0 }, role = "seat", size = 0.5 }) end
elseif a.kind == "bbq" then
  local R, Y = 0.28, 0.72
  local function half(from, to)                                                    -- part of a ball's outline, bottom to top
    local p = {}
    for i = 0, 8 do local t = math.rad(from + (to - from) * i / 8); p[#p + 1] = { R * math.cos(t), R * math.sin(t) * 0.75 } end
    return p
  end
  -- a shell has an inside too: the same outline run back, a little smaller, faces in
  local function shell(p)
    for i = #p, 1, -1 do p[#p + 1] = { p[i][1] * 0.96, p[i][2] * 0.96 } end
    return p
  end
  local bowl = half(-90, 0); bowl[1][1] = 0
  bowl = shell(bowl)
  c.turned("paint", V(0, Y, 0), bowl, IRON, 1, 24)
  c.turned("hard", V(0, Y - 0.06, 0), { { 0, 0 }, { R - 0.03, 0 }, { R - 0.03, 0.004 }, { 0, 0.004 } }, V(0.25, 0.08, 0.05), 1, 20)   -- the coals
  for i = -5, 5 do c.rod("metal", V(i * 0.045, Y - 0.02, -math.sqrt(R * R - (i * 0.045) ^ 2) + 0.02), V(i * 0.045, Y - 0.02, math.sqrt(R * R - (i * 0.045) ^ 2) - 0.02), 0.003, kit.CHROME, 4) end
  for i = 0, 2 do
    local t = math.rad(90 + i * 120)
    c.rod("metal", V(0.16 * math.cos(t), Y - 0.17, 0.16 * math.sin(t)), V(0.33 * math.cos(t), 0, 0.33 * math.sin(t)), 0.012, kit.STEEL, 8)
  end
  for _, x in ipairs{ -R - 0.02, R + 0.02 } do c.box("wood", V(x * 1.08, Y - 0.02, 0), V(0.05, 0.02, 0.14), 0.008, wood, 1) end   -- the handles
  kit.put(m, m, c)
  kit.far_box(m, m, "paint", { 2 * R * 0.8, 0.4, 2 * R * 0.8 }, { 0, Y, 0 }, IRON, far)
  m:shape("bowl", { capsule = { radius = R, height = 0.02 }, pos = { 0, Y, 0 } })
  m:marker("smoke", { pos = { 0, Y + 0.25, 0 } })
  m:marker("front", { pos = { 0, 0, 0.9 }, turn = { 0, 180, 0 }, role = "stand" })
  local lid = m:node("lid", { pos = { 0, Y + 0.002, -R }, motion = { turn = { -1, 0, 0 }, min = 0, max = 80 } })
  local k = kit.maker()
  local dome = half(0, 90); dome[#dome][1] = 0
  dome = shell(dome)
  k.turned("paint", V(0, 0, R), dome, IRON, 1, 24)
  k.box("wood", V(0, R * 0.75 + 0.04, R), V(0.12, 0.025, 0.03), 0.01, wood, 1)
  for _, x in ipairs{ -0.045, 0.045 } do k.rod("metal", V(x, R * 0.74, R), V(x, R * 0.75 + 0.03, R), 0.005, kit.CHROME, 6) end
  kit.put(m, lid, k)
  lid:marker("lid", { pos = { 0, R * 0.75 + 0.05, R }, role = "hold" })
elseif a.kind == "rotary_dryer" then
  local TOP, ARM = 1.75, 1.05
  c.rod("metal", V(0, 0, 0), V(0, TOP, 0), 0.02, kit.STEEL, 10)
  c.turned("metal", V(0, 0, 0), { { 0, 0 }, { 0.06, 0 }, { 0.03, 0.04 }, { 0, 0.04 } }, kit.STEEL, 1, 12)
  kit.put(m, m, c)
  m:shape("pole", { capsule = { radius = 0.025, height = TOP }, pos = { 0, TOP / 2, 0 } })
  local head = m:node("head", { pos = { 0, TOP, 0 }, motion = { turn = { 0, 1, 0 } } })
  local k = kit.maker()
  local RISE = 0.3
  for i = 0, 3 do
    local cs, sn = math.cos(i * math.pi / 2), math.sin(i * math.pi / 2)
    k.rod("metal", V(0, -0.1, 0), V(ARM * cs, RISE, ARM * sn), 0.01, kit.STEEL, 6)
    k.rod("metal", V(0, -0.5, 0), V(ARM * 0.5 * cs, RISE * 0.5 - 0.05, ARM * 0.5 * sn), 0.006, kit.STEEL, 5)   -- the stay under it
  end
  for _, f in ipairs{ 0.4, 0.7, 1.0 } do                                                                      -- the lines, round the four arms
    local r, y = ARM * f, -0.1 + (RISE + 0.1) * f + 0.012
    k.pipe("hard", { { r, y, 0 }, { 0, y, r }, { -r, y, 0 }, { 0, y, -r } }, 0.0025, V(0.2, 0.5, 0.8), { closed = true, sides = 3 })
  end
  local wash = { { 0.25, 0.3, 0.45, kit.FABRIC[2] }, { 0.62, 0.5, 0.6, kit.WHITE }, { 1.3, 0.28, 0.35, kit.FABRIC[4] }, { 2.2, 0.55, 0.7, kit.FABRIC[7] }, { 2.75, 0.3, 0.3, kit.WHITE }, { 3.5, 0.4, 0.55, kit.FABRIC[8] } }
  local y = RISE + 0.012
  for _, w in ipairs(wash) do                                                                                 -- the washing, on the outer line
    local side, t = math.floor(w[1]), w[1] % 1
    local c0, s0 = math.cos(side * math.pi / 2), math.sin(side * math.pi / 2)
    local c1, s1 = math.cos((side + 1) * math.pi / 2), math.sin((side + 1) * math.pi / 2)
    local px, pz = ARM * (c0 + (c1 - c0) * t), ARM * (s0 + (s1 - s0) * t)
    local sheet = mesh.rotate_y(mesh.box{ w[2], w[3], 0.004 }, -math.atan(s1 - s0, c1 - c0))
    k.mesh("fabric", mesh.translate(sheet, { px, y - w[3] / 2, pz }), w[4])
  end
  kit.put(m, head, k)
  head:marker("line", { pos = { ARM * 0.7, RISE * 0.7, 0 }, role = "hold" })
elseif a.kind == "weathervane" then
  local TOP = 1.1
  c.rod("metal", V(0, 0, 0), V(0, TOP, 0), 0.012, IRON, 8)
  c.turned("metal", V(0, 0, 0), { { 0, 0 }, { 0.07, 0 }, { 0.02, 0.05 }, { 0, 0.05 } }, IRON, 1, 12)
  c.mesh("metal", mesh.translate(kit.ball(0.04), { 0, 0.55, 0 }), V(0.8, 0.62, 0.25))
  for i = 0, 3 do                                                                                             -- the four points, the north one longer
    local cs, sn = math.cos(i * math.pi / 2), math.sin(i * math.pi / 2)
    local len = i == 3 and 0.3 or 0.24
    c.rod("metal", V(0, 0.75, 0), V(len * cs, 0.75, len * sn), 0.006, IRON, 6)
    c.mesh("metal", mesh.translate(kit.ball(i == 3 and 0.028 or 0.018, 8), { len * cs, 0.75, len * sn }), IRON)
  end
  kit.put(m, m, c)
  m:shape("pole", { capsule = { radius = 0.02, height = TOP }, pos = { 0, TOP / 2, 0 } })
  local vane = m:node("vane", { pos = { 0, TOP, 0 }, motion = { turn = { 0, 1, 0 } } })
  local k = kit.maker()
  k.rod("metal", V(-0.32, 0, 0), V(0.3, 0, 0), 0.007, IRON, 6)
  k.mesh("metal", mesh.translate(mesh.rotate_z(mesh.lathe{ profile = { { 0, 0 }, { 0.045, 0 }, { 0, 0.11 } }, segments = 4 }, -math.pi / 2), { 0.3, 0, 0 }), IRON)   -- the point
  k.box("metal", V(-0.27, 0.0, 0), V(0.16, 0.15, 0.004), 0.0, IRON, 1)                                        -- the tail the wind pushes
  k.mesh("metal", mesh.translate(kit.ball(0.02, 8), { 0, 0.02, 0 }), V(0.8, 0.62, 0.25))
  kit.put(m, vane, k)
else
  local HANG = V(0.32, 1.78, 0)
  c.pipe("metal", { { 0, 0, 0 }, { 0, 1.95, 0 }, { 0.32, 1.95, 0 }, { 0.32, 1.86, 0 } }, 0.01, IRON, { bend = 0.12, sides = 8 })
  c.turned("metal", V(0, 0, 0), { { 0, 0 }, { 0.08, 0 }, { 0.02, 0.04 }, { 0, 0.04 } }, IRON, 1, 12)
  c.rod("hard", HANG + V(0, 0.08, 0), HANG + V(0, -0.1, 0), 0.0015, kit.WHITE, 3)
  c.turned("wood", HANG + V(0, -0.11, 0), { { 0, 0 }, { 0.065, 0 }, { 0.065, 0.012 }, { 0, 0.012 } }, wood, 1, 16)
  kit.put(m, m, c)
  m:shape("pole", { capsule = { radius = 0.02, height = 1.9 }, pos = { 0, 0.95, 0 } })
  m:marker("ring", { pos = { HANG[1], HANG[2] - 0.3, 0 } })
  for i = 1, 5 do
    local t = 2 * math.pi * i / 5
    local n = m:node("tube_" .. i, { pos = { HANG[1] + 0.05 * math.cos(t), HANG[2] - 0.11, 0.05 * math.sin(t) }, motion = { turn = { math.sin(t), 0, -math.cos(t) }, min = -12, max = 12 } })
    local k = kit.maker()
    local len = 0.2 + i * 0.035
    k.rod("hard", V(0, 0, 0), V(0, -0.05, 0), 0.001, kit.WHITE, 3)
    k.turned("metal", V(0, -0.05 - len, 0), { { 0.007, 0 }, { 0.009, 0 }, { 0.009, len }, { 0.007, len } }, kit.CHROME, 1, 8)
    kit.put(m, n, k)
  end
  local striker = m:node("striker", { pos = { HANG[1], HANG[2] - 0.11, 0 }, motion = { turn = { 0, 0, 1 }, min = -10, max = 10 } })
  local k = kit.maker()
  k.rod("hard", V(0, 0, 0), V(0, -0.5, 0), 0.001, kit.WHITE, 3)
  k.turned("wood", V(0, -0.2, 0), { { 0, 0 }, { 0.028, 0 }, { 0.028, 0.012 }, { 0, 0.012 } }, wood, 1, 12)
  k.box("wood", V(0, -0.55, 0), V(0.07, 0.1, 0.004), 0.0, wood, 1)                                            -- the sail the wind catches
  kit.put(m, striker, k)
end
return m
