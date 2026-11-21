-- BATHROOM PIECES FOR A HOME, each with parts that move (#191):
--
--   toilet      its lid and its seat lift (`lid`, `seat`) and the flush handle on the cistern turns (`flush`)
--   pedal_bin   pressing the pedal (`pedal`) lifts the lid (`lid`)
--   cabinet     a mirrored wall cabinet: two doors (`door_left`, `door_right`) over shelves of bottles
--   shower      a shower in a corner: a tray, one fixed glass side, a glass door (`door`), a riser and its head
--
-- Each has its front toward +z. The toilet, the bin and the shower stand on y = 0; the cabinet's origin is the
-- foot of its back, to be put on a wall.
local kit = require "model_kit"
local V = kit.V

local a = params{ kind = { "toilet", "pedal_bin", "cabinet", "shower" } }
local m = model.new()
local far = m:level(25)
local c = kit.maker()
local GLASS = V(0.75, 0.88, 0.9)

if a.kind == "toilet" then
  local P = kit.PORCELAIN
  -- the pan: an oval bowl on a foot, its hollow, and the water in it
  c.turned("ceramic", V(0, 0, 0.1), { { 0, 0 }, { 0.13, 0 }, { 0.11, 0.2 }, { 0.2, 0.36 }, { 0.2, 0.4 }, { 0.165, 0.4 }, { 0.12, 0.26 }, { 0, 0.24 } }, P, 1.3, 24)
  c.turned("glass", V(0, 0.27, 0.1), { { 0, 0 }, { 0.115, 0 }, { 0.115, 0.004 }, { 0, 0.004 } }, V(0.6, 0.8, 0.9), 1.3, 20)
  c.box("ceramic", V(0, 0.2, -0.2), V(0.2, 0.4, 0.14), 0.02, P, 2)
  c.box("ceramic", V(0, 0.6, -0.26), V(0.42, 0.38, 0.18), 0.025, P, 2)                 -- the cistern
  c.box("ceramic", V(0, 0.8, -0.26), V(0.44, 0.03, 0.2), 0.01, P, 1)
  kit.put(m, m, c)
  kit.far_box(m, m, "ceramic", { 0.42, 0.42, 0.6 }, { 0, 0.21, 0 }, P, far)
  kit.far_box(m, m, "ceramic", { 0.42, 0.4, 0.18 }, { 0, 0.6, -0.26 }, P, far)
  m:shape("pan", { box = { 0.4, 0.4, 0.52 }, pos = { 0, 0.2, 0.1 } })
  m:shape("cistern", { box = { 0.44, 0.82, 0.2 }, pos = { 0, 0.41, -0.26 } })
  local seat = m:node("seat", { pos = { 0, 0.405, -0.15 }, motion = { turn = { -1, 0, 0 }, min = 0, max = 95 } })
  local s = kit.maker()
  s.turned("hard", V(0, 0, 0.25), { { 0.13, 0 }, { 0.205, 0 }, { 0.205, 0.014 }, { 0.13, 0.014 } }, kit.WHITE, 1.3, 24)
  kit.put(m, seat, s)
  seat:marker("seat", { pos = { 0, 0.014, 0.25 }, role = "seat", size = 0.4 })
  local lid = m:node("lid", { pos = { 0, 0.422, -0.15 }, motion = { turn = { -1, 0, 0 }, min = 0, max = 100 } })
  local l = kit.maker()
  l.turned("hard", V(0, 0, 0.25), { { 0, 0 }, { 0.205, 0 }, { 0.2, 0.014 }, { 0, 0.018 } }, kit.WHITE, 1.3, 24)
  kit.put(m, lid, l)
  lid:marker("lid", { pos = { 0, 0.02, 0.5 }, role = "hold" })
  local flush = m:node("flush", { pos = { -0.15, 0.72, -0.17 }, motion = { turn = { 0, 0, 1 }, min = 0, max = 35 } })
  local f = kit.maker()
  f.rod("metal", V(0, 0, 0), V(0, 0, 0.02), 0.01, kit.CHROME, 8)
  f.box("metal", V(-0.035, 0, 0.022), V(0.09, 0.016, 0.008), 0.004, kit.CHROME, 1)
  kit.put(m, flush, f)
  flush:marker("flush", { pos = { -0.07, 0, 0.03 }, role = "hold" })
elseif a.kind == "pedal_bin" then
  local col = V(0.72, 0.74, 0.76)
  c.turned("metal", V(0, 0, 0), { { 0, 0.01 }, { 0.14, 0.01 }, { 0.15, 0.03 }, { 0.15, 0.42 }, { 0.14, 0.42 }, { 0.14, 0.05 }, { 0, 0.05 } }, col, 1, 24)
  c.turned("rubber", V(0, 0, 0), { { 0.12, 0 }, { 0.152, 0 }, { 0.152, 0.03 }, { 0.12, 0.03 } }, kit.BLACK, 1, 24)
  kit.put(m, m, c)
  kit.far_box(m, m, "metal", { 0.26, 0.42, 0.26 }, { 0, 0.21, 0 }, col, far)
  m:shape("body", { capsule = { radius = 0.15, height = 0.12 }, pos = { 0, 0.21, 0 } })
  local lid = m:node("lid", { pos = { 0, 0.42, -0.15 }, motion = { turn = { -1, 0, 0 }, min = 0, max = 75 } })
  local l = kit.maker()
  l.turned("metal", V(0, 0, 0.15), { { 0, 0 }, { 0.155, 0 }, { 0.155, 0.012 }, { 0.1, 0.035 }, { 0, 0.042 } }, col, 1, 24)
  kit.put(m, lid, l)
  local pedal = m:node("pedal", { pos = { 0, 0.035, 0.14 }, motion = { turn = { 1, 0, 0 }, min = 0, max = 18 } })
  local p = kit.maker()
  p.box("hard", V(0, 0.01, 0.045), V(0.09, 0.012, 0.09), 0.004, kit.BLACK, 1)
  kit.put(m, pedal, p)
  pedal:marker("pedal", { pos = { 0, 0.02, 0.06 }, role = "press" })
elseif a.kind == "cabinet" then
  local W, H, D, T = 0.7, 0.6, 0.15, 0.015
  kit.carcass(c, "paint", W, H, D, T, kit.WHITE)
  c.box("paint", V(0, H / 2, 0), V(T, H - 2 * T, D - 0.02), 0.0, kit.WHITE, 1)
  for i = 1, 2 do c.box("glass", V(0, H * i / 3, 0), V(W - 2 * T, 0.005, D - 0.03), 0.0, GLASS, 1) end
  local tints = { V(0.2, 0.5, 0.8), V(0.9, 0.9, 0.85), V(0.8, 0.3, 0.3), V(0.3, 0.7, 0.5), V(0.85, 0.7, 0.2) }
  for i = 0, 11 do                                                                     -- bottles and jars on the shelves
    local r, h = 0.016 + (i % 3) * 0.004, 0.07 + (i % 4) * 0.02
    c.turned("hard", V(-0.29 + i * 0.053, T + (i % 3) * H / 3 + (i % 3 > 0 and -T + 0.003 or 0), 0.0), { { 0, 0 }, { r, 0 }, { r, h * 0.75 }, { r * 0.5, h * 0.85 }, { r * 0.5, h }, { 0, h } }, tints[i % 5 + 1], 1, 10)
  end
  kit.put(m, m, c)
  kit.far_box(m, m, "paint", { W, H, D + 0.02 }, { 0, H / 2, 0.01 }, kit.WHITE, far)
  m:shape("body", { box = { W, H, D }, pos = { 0, H / 2, 0 } })
  for _, side in ipairs{ -1, 1 } do
    local name = side < 0 and "door_left" or "door_right"
    local n = m:node(name, { pos = { side * W / 2, 0, D / 2 }, motion = { turn = { 0, side, 0 }, min = 0, max = 110 } })
    local k = kit.maker()
    local w = W / 2 - 0.002
    k.box("paint", V(-side * w / 2, H / 2, 0.008), V(w, H, 0.016), 0.003, kit.WHITE, 1)
    k.box("metal", V(-side * w / 2, H / 2, 0.0165), V(w - 0.02, H - 0.02, 0.001), 0.0, V(0.86, 0.9, 0.92), 1)   -- the mirror
    kit.put(m, n, k)
    n:marker(name, { pos = { -side * (w - 0.02), 0.05, 0.02 }, role = "hold" })
  end
else
  local S, H = 0.9, 2.0
  c.box("ceramic", V(0, 0.05, 0), V(S, 0.1, S), 0.02, kit.PORCELAIN, 2)                -- the tray
  c.turned("metal", V(0, 0.1, 0), { { 0, 0 }, { 0.04, 0 }, { 0.04, 0.003 }, { 0, 0.003 } }, kit.CHROME, 1, 12)
  c.box("glass", V(S / 2 - 0.01, 0.1 + H / 2, 0), V(0.008, H, S - 0.04), 0.0, GLASS, 1) -- the fixed side
  c.rod("metal", V(S / 2 - 0.01, 0.1, S / 2 - 0.02), V(S / 2 - 0.01, 0.1 + H, S / 2 - 0.02), 0.012, kit.CHROME, 8)
  c.rod("metal", V(-S / 2 + 0.01, 0.1, S / 2 - 0.02), V(-S / 2 + 0.01, 0.1 + H, S / 2 - 0.02), 0.012, kit.CHROME, 8)
  c.rod("metal", V(-S / 2 + 0.01, 0.1 + H, S / 2 - 0.02), V(S / 2 - 0.01, 0.1 + H, S / 2 - 0.02), 0.01, kit.CHROME, 8)
  -- the riser up the back wall, over to the head
  c.pipe("metal", { { 0, 1.1, -S / 2 + 0.04 }, { 0, 2.05, -S / 2 + 0.04 }, { 0, 2.05, -S / 2 + 0.34 } }, 0.011, kit.CHROME, { bend = 0.08, sides = 8 })
  c.turned("metal", V(0, 2.0, -S / 2 + 0.34), { { 0, 0 }, { 0.1, 0 }, { 0.1, 0.012 }, { 0.02, 0.04 }, { 0, 0.04 } }, kit.CHROME, 1, 20)
  c.box("metal", V(0, 1.1, -S / 2 + 0.04), V(0.16, 0.06, 0.05), 0.015, kit.CHROME, 2)   -- the valve
  kit.put(m, m, c)
  kit.far_box(m, m, "glass", { S, H, S }, { 0, 0.1 + H / 2, 0 }, GLASS, far)
  m:shape("tray", { box = { S, 0.1, S }, pos = { 0, 0.05, 0 }, friction = 0.9 })
  m:shape("side", { box = { 0.02, H, S }, pos = { S / 2 - 0.01, 0.1 + H / 2, 0 } })
  m:marker("valve", { pos = { 0, 1.1, -S / 2 + 0.08 }, role = "hold" })
  m:marker("spray", { pos = { 0, 2.0, -S / 2 + 0.34 } })
  m:marker("stand", { pos = { 0, 0.1, 0 }, role = "stand" })
  local door = m:node("door", { pos = { -S / 2 + 0.01, 0.1, S / 2 - 0.02 }, motion = { turn = { 0, -1, 0 }, min = 0, max = 95 } })
  local k = kit.maker()
  local w = S - 0.05
  k.box("glass", V(w / 2 + 0.01, H / 2, 0), V(w, H - 0.02, 0.008), 0.0, GLASS, 1)
  k.rod("metal", V(w - 0.06, 0.9, 0.03), V(w - 0.06, 1.2, 0.03), 0.009, kit.CHROME, 8)
  for _, y in ipairs{ 0.9, 1.2 } do k.rod("metal", V(w - 0.06, y, 0), V(w - 0.06, y, 0.03), 0.006, kit.CHROME, 6) end
  kit.put(m, door, k)
  door:shape("door", { box = { w, H, 0.02 }, pos = { w / 2 + 0.01, H / 2, 0 } })
  door:marker("handle", { pos = { w - 0.06, 1.05, 0.04 }, role = "hold" })
end
m:marker("front", { pos = { 0, 0, 0.9 }, turn = { 0, 180, 0 }, role = "stand" })
return m
