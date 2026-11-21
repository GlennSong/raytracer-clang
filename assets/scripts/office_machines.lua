-- OFFICE MACHINES, each with parts that move (#191):
--
--   printer        a floor-standing copier: its scanner lid lifts (`lid`), a paper drawer slides out (`tray`), and a
--                  printed sheet comes out into the catch tray (`sheet`)
--   shredder       its bin slides out at the front (`bin`), and a sheet goes down into its slot (`sheet`)
--   server_rack    a glass door (`door`), and one server on rails that slides out (`server`); lights on the rest
--   flip_chart     an easel with a pad; the top sheet flips over the back (`sheet`)
--   paper_trimmer  a board with a blade arm that lifts (`arm`). It stands on a desk
--
-- Each stands on y = 0 at its own origin, its front toward +z.
local kit = require "model_kit"
local V = kit.V

local a = params{ kind = { "printer", "shredder", "server_rack", "flip_chart", "paper_trimmer" } }
local BODY, DARK, PAPER = kit.paint.white, kit.paint.black, V(0.97, 0.97, 0.95)
local m = model.new()
local far = m:level(30)
local c = kit.maker()

if a.kind == "printer" then
  local W, H, D = 0.6, 1.0, 0.6
  kit.carcass(c, "paint", W, H - 0.25, D, 0.02, BODY)
  c.box("paint", V(0, H - 0.18, 0), V(W, 0.14, D), 0.012, BODY, 1)                                  -- the scanner
  c.box("glass", V(0, H - 0.108, 0), V(W - 0.1, 0.004, D - 0.14), 0.0, V(0.3, 0.35, 0.4), 1)
  c.box("paint", V(0, H - 0.45, 0.1), V(W - 0.06, 0.02, D - 0.2), 0.006, DARK, 1)                   -- the catch tray
  c.box("hard", V(W / 2 - 0.12, H - 0.11, D / 2 - 0.05), V(0.2, 0.012, 0.08), 0.004, DARK, 1)
  c.box("screen", V(W / 2 - 0.12, H - 0.103, D / 2 - 0.05), V(0.12, 0.001, 0.06), 0.0, V(0.15, 0.4, 0.6), 1)
  for i = 0, 1 do c.box("paint", V(0, 0.16 + i * 0.2, D / 2 - 0.01), V(W - 0.05, 0.18, 0.02), 0.006, BODY * 0.95, 1) end   -- two more drawers, shut
  kit.put(m, m, c)
  kit.far_box(m, m, "paint", { W, H - 0.1, D }, { 0, (H - 0.1) / 2, 0 }, BODY, far)
  m:shape("body", { box = { W, H - 0.1, D }, pos = { 0, (H - 0.1) / 2, 0 } })
  m:marker("panel", { pos = { W / 2 - 0.12, H - 0.1, D / 2 - 0.05 }, role = "press" })
  local lid = m:node("lid", { pos = { 0, H - 0.105, -D / 2 + 0.02 }, motion = { turn = { -1, 0, 0 }, min = 0, max = 70 } })
  local l = kit.maker()
  l.box("paint", V(0, 0.02, D / 2 - 0.05), V(W - 0.04, 0.035, D - 0.08), 0.01, BODY, 1)
  kit.put(m, lid, l)
  lid:marker("lid", { pos = { 0, 0.02, D - 0.1 }, role = "hold" })
  local tray = m:node("tray", { pos = { 0, 0.47, D / 2 - 0.02 }, motion = { slide = { 0, 0, 1 }, min = 0, max = 0.4 } })
  local t = kit.maker()
  t.box("paint", V(0, 0.09, 0.01), V(W - 0.05, 0.18, 0.02), 0.006, BODY * 0.95, 1)
  t.box("paint", V(0, 0.01, -0.22), V(W - 0.08, 0.02, 0.44), 0.0, DARK, 1)
  t.box("hard", V(0, 0.05, -0.22), V(0.3, 0.06, 0.42), 0.0, PAPER, 1)                               -- the ream in it
  t.box("hard", V(0, 0.12, 0.022), V(0.12, 0.02, 0.006), 0.003, DARK, 1)
  kit.put(m, tray, t)
  tray:marker("tray", { pos = { 0, 0.12, 0.03 }, role = "hold" })
  local sheet = m:node("sheet", { pos = { 0, H - 0.435, -0.12 }, motion = { slide = { 0, 0, 1 }, min = 0, max = 0.22 } })
  local s = kit.maker()
  s.box("hard", V(0, 0, 0), V(0.21, 0.001, 0.297), 0.0, PAPER, 1)
  kit.put(m, sheet, s)
elseif a.kind == "shredder" then
  local W, H, D = 0.38, 0.62, 0.3
  kit.carcass(c, "paint", W, H - 0.14, D, 0.015, DARK)
  c.box("paint", V(0, H - 0.07, 0), V(W, 0.14, D), 0.02, DARK, 2)
  c.box("hard", V(0, H + 0.001, 0), V(0.24, 0.002, 0.012), 0.0, kit.BLACK * 0.5, 1)                  -- the slot
  c.box("light", V(W / 2 - 0.04, H + 0.001, 0.08), V(0.012, 0.002, 0.012), 0.0, V(0.3, 1, 0.4), 1)
  kit.put(m, m, c)
  kit.far_box(m, m, "paint", { W, H, D }, { 0, H / 2, 0 }, DARK, far)
  m:shape("body", { box = { W, H, D }, pos = { 0, H / 2, 0 } })
  local bin = m:node("bin", { pos = { 0, 0.02, D / 2 - 0.01 }, motion = { slide = { 0, 0, 1 }, min = 0, max = 0.26 } })
  local b = kit.maker()
  b.box("paint", V(0, 0.22, 0.008), V(W - 0.035, 0.44, 0.016), 0.004, DARK * 1.3, 1)
  b.box("glass", V(0, 0.2, 0.017), V(0.12, 0.24, 0.002), 0.0, V(0.5, 0.55, 0.6), 1)
  b.box("paint", V(0, 0.005, -0.13), V(W - 0.05, 0.01, 0.26), 0.0, DARK, 1)
  b.box("hard", V(0, 0.12, -0.13), V(W - 0.08, 0.22, 0.24), 0.03, PAPER * 0.95, 2)                   -- the shreds in it
  kit.put(m, bin, b)
  bin:marker("bin", { pos = { 0, 0.4, 0.03 }, role = "hold" })
  local sheet = m:node("sheet", { pos = { 0, H + 0.15, 0 }, motion = { slide = { 0, -1, 0 }, min = 0, max = 0.3 } })
  local s = kit.maker()
  s.box("hard", V(0, 0, 0), V(0.21, 0.297, 0.001), 0.0, PAPER, 1)
  kit.put(m, sheet, s)
elseif a.kind == "server_rack" then
  local W, H, D, T = 0.6, 2.0, 0.9, 0.03
  kit.carcass(c, "paint", W, H, D, T, DARK)
  local U = 0.089                               -- a server's height (two rack units)
  local slots = math.floor((H - 0.2) / U)
  for i = 0, slots - 1 do
    if i ~= 9 and i % 5 ~= 4 then
      local y = 0.1 + (i + 0.5) * U
      c.box("hard", V(0, y, D / 2 - 0.1), V(W - 2 * T - 0.02, U - 0.006, 0.02), 0.0, kit.BLACK * 1.2, 1)
      for j = 0, 3 do c.box("light", V(-0.2 + j * 0.03, y, D / 2 - 0.089), V(0.008, 0.008, 0.002), 0.0, (i + j) % 3 == 0 and V(1, 0.6, 0.1) or V(0.2, 1, 0.4), 1) end
    end
  end
  kit.put(m, m, c)
  kit.far_box(m, m, "paint", { W, H, D }, { 0, H / 2, 0 }, DARK, far)
  m:shape("back", { box = { W, H, T }, pos = { 0, H / 2, -D / 2 + T / 2 } })
  for i, x in ipairs{ -W / 2 + T / 2, W / 2 - T / 2 } do m:shape("side_" .. i, { box = { T, H, D }, pos = { x, H / 2, 0 } }) end
  m:marker("hum", { pos = { 0, H / 2, 0 } })
  local server = m:node("server", { pos = { 0, 0.1 + 9.5 * U, D / 2 - 0.09 }, motion = { slide = { 0, 0, 1 }, min = 0, max = 0.55 } })
  local s = kit.maker()
  s.box("metal", V(0, 0, -0.3), V(W - 2 * T - 0.03, U - 0.008, 0.62), 0.0, V(0.6, 0.61, 0.63), 1)
  s.box("hard", V(0, 0, 0.0), V(W - 2 * T - 0.02, U - 0.006, 0.02), 0.0, kit.BLACK * 1.2, 1)
  for _, x in ipairs{ -0.22, 0.22 } do s.rod("metal", V(x, -0.02, 0.01), V(x, 0.02, 0.035), 0.004, kit.CHROME, 4) end
  kit.put(m, server, s)
  server:marker("server", { pos = { 0, 0, 0.04 }, role = "hold" })
  local door = m:node("door", { pos = { -W / 2, 0.02, D / 2 }, motion = { turn = { 0, -1, 0 }, min = 0, max = 120 } })
  local k = kit.maker()
  for _, x in ipairs{ 0.02, W - 0.02 } do k.box("paint", V(x, (H - 0.04) / 2, 0.01), V(0.04, H - 0.04, 0.02), 0.004, DARK, 1) end
  for _, y in ipairs{ 0.02, H - 0.06 } do k.box("paint", V(W / 2, y, 0.01), V(W - 0.08, 0.04, 0.02), 0.004, DARK, 1) end
  k.box("glass", V(W / 2, (H - 0.04) / 2, 0.01), V(W - 0.08, H - 0.12, 0.005), 0.0, V(0.3, 0.33, 0.36), 1)
  k.box("metal", V(W - 0.02, H / 2, 0.025), V(0.02, 0.12, 0.012), 0.004, kit.CHROME, 1)
  kit.put(m, door, k)
  door:shape("door", { box = { W, H - 0.04, 0.02 }, pos = { W / 2, (H - 0.04) / 2, 0.01 } })
  door:marker("handle", { pos = { W - 0.02, H / 2, 0.04 }, role = "hold" })
elseif a.kind == "flip_chart" then
  local W, TOP = 0.7, 1.85
  for _, x in ipairs{ -0.3, 0.3 } do c.rod("metal", V(x * 1.3, 0, 0.25), V(x, TOP, 0), 0.012, V(0.7, 0.71, 0.73), 8) end
  c.rod("metal", V(0, 0, -0.5), V(0, TOP - 0.1, 0), 0.012, V(0.7, 0.71, 0.73), 8)
  c.tilted("paint", V(0, 1.33, 0.085), V(W, 1.0, 0.015), 0.004, -0.165, BODY)                         -- the board
  c.tilted("hard", V(0, 1.33, 0.096), V(W - 0.06, 0.95, 0.006), 0.0, -0.165, PAPER)                   -- the pad
  c.box("paint", V(0, 0.82, 0.2), V(W, 0.02, 0.06), 0.004, BODY, 1)                                  -- the pen tray
  for i, x in ipairs{ -0.1, 0, 0.1 } do c.rod("hard", V(x - 0.05, 0.84, 0.2), V(x + 0.05, 0.84, 0.2), 0.008, ({ kit.paint.red, kit.paint.blue, kit.BLACK })[i], 6) end
  kit.put(m, m, c)
  kit.far_box(m, m, "paint", { W, 1.0, 0.03 }, { 0, 1.33, 0.08 }, PAPER, far)
  m:shape("board", { box = { W, 1.9, 0.5 }, pos = { 0, 0.95, 0 } })
  local sheet = m:node("sheet", { pos = { 0, 1.815, 0.02 }, turn = { -9.5, 0, 0 }, motion = { turn = { 1, 0, 0 }, min = -170, max = 0 } })
  local k = kit.maker()
  k.box("hard", V(0, -0.475, 0.004), V(W - 0.06, 0.95, 0.001), 0.0, PAPER, 1)
  for i = 0, 3 do k.box("hard", V(-0.08 + (i % 2) * 0.04, -0.2 - i * 0.12, 0.0048), V(0.36 - (i % 2) * 0.1, 0.02, 0.0004), 0.0, i == 0 and kit.paint.red or kit.paint.blue, 1) end   -- what is written on it
  kit.put(m, sheet, k)
  sheet:marker("corner", { pos = { W / 2 - 0.04, -0.94, 0.01 }, role = "hold" })
else
  c.box("wood", V(0, 0.01, 0), V(0.46, 0.02, 0.36), 0.004, kit.wood("birch"), 1)
  c.box("hard", V(0, 0.021, -0.165), V(0.46, 0.004, 0.02), 0.0, DARK, 1)                             -- the rule along the top
  for i = 1, 8 do c.box("hard", V(-0.23 + i * 0.05, 0.0205, 0), V(0.001, 0.0006, 0.32), 0.0, V(0.5, 0.6, 0.5), 1) end
  c.box("hard", V(-0.05, 0.021, 0.02), V(0.21, 0.001, 0.297), 0.0, PAPER, 1)
  kit.put(m, m, c)
  m:shape("board", { box = { 0.46, 0.03, 0.36 }, pos = { 0, 0.015, 0 } })
  local arm = m:node("arm", { pos = { 0.232, 0.03, -0.17 }, motion = { turn = { -1, 0, 0 }, min = 0, max = 70 } })
  local k = kit.maker()
  k.box("metal", V(0, 0, 0.2), V(0.006, 0.03, 0.42), 0.0, V(0.75, 0.76, 0.78), 1)
  k.rod("hard", V(0, 0.01, 0.4), V(0, 0.02, 0.5), 0.012, DARK, 8)
  kit.put(m, arm, k)
  arm:marker("handle", { pos = { 0, 0.02, 0.48 }, role = "hold" })
end
return m
