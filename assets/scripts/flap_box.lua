-- A BOX ON THE STREET WITH A FLAP THAT OPENS: a litter bin, a mailbox, a newspaper box (#191). Each has one
-- moving part, on a node named `flap`, and a marker `handle` on it.
--
--   bin         a square bin with a hooded top; the flap hangs across the opening and pushes in, 60 degrees
--   mailbox     a round-topped box on four legs; the flap is the pull-down door near the top, 70 degrees
--   newspaper   a box on a stand with a window; the flap is the whole front, hinged at its foot, 80 degrees
--
-- It stands on y = 0 at its own origin, its flap facing +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind  = { "bin", "mailbox", "newspaper" },
  paint = { "green", "blue", "red", "yellow", "orange", "black" },
}
local col = a.paint == "black" and kit.BLACK * 1.8 or kit.paint[a.paint]
local m = model.new()
local far = m:level(50)
local c = kit.maker()
local T = 0.012

-- an open-fronted box: two sides, a back, a floor and a roof, from y0 to y1
local function carcass(W, D, y0, y1)
  local h = y1 - y0
  for _, x in ipairs{ -W / 2 + T / 2, W / 2 - T / 2 } do c.box("paint", V(x, y0 + h / 2, 0), V(T, h, D), 0.004, col, 1) end
  c.box("paint", V(0, y0 + h / 2, -D / 2 + T / 2), V(W - 2 * T, h, T), 0.0, col, 1)
  c.box("paint", V(0, y0 + T / 2, 0), V(W - 2 * T, T, D), 0.0, col, 1)
  c.box("paint", V(0, y1 - T / 2, 0), V(W - 2 * T, T, D), 0.0, col, 1)
end
local function front(W, D, y0, y1)             -- a fixed panel across the front from y0 to y1
  c.box("paint", V(0, (y0 + y1) / 2, D / 2 - T / 2), V(W - 2 * T, y1 - y0, T), 0.0, col, 1)
end

if a.kind == "bin" then
  local W, D, H = 0.5, 0.5, 1.0
  carcass(W, D, 0.04, H)
  front(W, D, 0.04, 0.68)
  front(W, D, 0.9, H)
  c.box("paint", V(0, H + 0.02, 0.02), V(W + 0.04, 0.04, D + 0.08), 0.015, col * 0.8, 2)     -- the hood
  c.box("hard", V(0, 0.02, 0), V(W - 0.06, 0.04, D - 0.06), 0.0, kit.BLACK, 1)
  kit.put(m, m, c)
  m:shape("body", { box = { W, H + 0.04, D }, pos = { 0, (H + 0.04) / 2, 0 } })
  kit.far_box(m, m, "paint", { W, H, D }, { 0, H / 2, 0 }, col, far)
  local flap = m:node("flap", { pos = { 0, 0.9, D / 2 - T / 2 }, motion = { turn = { 1, 0, 0 }, min = 0, max = 60 } })
  local k = kit.maker()
  k.box("metal", V(0, -0.11, 0), V(W - 2 * T - 0.01, 0.215, 0.006), 0.0, V(0.6, 0.62, 0.65), 1)
  kit.put(m, flap, k)
  flap:marker("handle", { pos = { 0, -0.12, 0.01 }, role = "hold" })
elseif a.kind == "mailbox" then
  local W, D, H, LEG = 0.5, 0.45, 1.25, 0.25
  carcass(W, D, LEG, H - W / 2)
  front(W, D, LEG, H - W / 2 - 0.2)
  -- the rounded top: half a drum lying across the box
  c.mesh("paint", mesh.translate(mesh.rotate_z(mesh.lathe{ profile = { { 0.0, -D / 2 }, { W / 2, -D / 2 }, { W / 2, D / 2 }, { 0.0, D / 2 } }, segments = 24, smooth = true }, 0),
                                 { 0, 0, 0 }), col)
  c.lists.paint[#c.lists.paint] = mesh.translate(mesh.rotate_x(c.lists.paint[#c.lists.paint], math.pi / 2), { 0, H - W / 2, 0 })
  for _, x in ipairs{ -W / 2 + 0.04, W / 2 - 0.04 } do
    for _, z in ipairs{ -D / 2 + 0.04, D / 2 - 0.04 } do c.box("paint", V(x, LEG / 2, z), V(0.04, LEG, 0.04), 0.006, col * 0.8, 1) end
  end
  c.box("hard", V(0, LEG + 0.35, D / 2 + 0.002), V(0.22, 0.14, 0.003), 0.0, kit.WHITE, 1)      -- the notice of collection times
  kit.put(m, m, c)
  m:shape("body", { box = { W, H, D }, pos = { 0, H / 2, 0 } })
  kit.far_box(m, m, "paint", { W, H, D }, { 0, H / 2, 0 }, col, far)
  local flap = m:node("flap", { pos = { 0, H - W / 2 - 0.2, D / 2 }, motion = { turn = { 1, 0, 0 }, min = 0, max = 70 } })
  local k = kit.maker()
  k.box("paint", V(0, 0.1, 0.004), V(W - 0.06, 0.2, 0.012), 0.004, col * 0.9, 1)
  k.rod("metal", V(-0.08, 0.17, 0.03), V(0.08, 0.17, 0.03), 0.008, kit.CHROME, 8)
  kit.put(m, flap, k)
  flap:shape("flap", { box = { W - 0.06, 0.2, 0.02 }, pos = { 0, 0.1, 0.004 } })
  flap:marker("handle", { pos = { 0, 0.17, 0.04 }, role = "hold" })
else
  local W, D, y0, H = 0.48, 0.42, 0.5, 1.15
  carcass(W, D, y0, H)
  c.box("paint", V(0, y0 / 2, 0), V(0.1, y0, 0.1), 0.01, col * 0.8, 1)                 -- the stand
  c.box("paint", V(0, 0.015, 0), V(W, 0.03, D), 0.008, col * 0.8, 1)
  c.box("hard", V(0, y0 + 0.2, -0.02), V(W - 0.08, 0.3, D * 0.5), 0.004, V(0.9, 0.9, 0.86), 1)    -- the papers inside
  c.box("metal", V(W / 2 - 0.08, H + 0.03, 0.05), V(0.1, 0.06, 0.12), 0.008, V(0.6, 0.62, 0.65), 1)   -- the coin box
  kit.put(m, m, c)
  m:shape("body", { box = { W, H, D }, pos = { 0, H / 2, 0 } })
  kit.far_box(m, m, "paint", { W, H, D }, { 0, H / 2, 0 }, col, far)
  m:marker("coins", { pos = { W / 2 - 0.08, H + 0.06, 0.1 }, role = "press" })
  local fh = H - y0
  local flap = m:node("flap", { pos = { 0, y0, D / 2 }, motion = { turn = { 1, 0, 0 }, min = 0, max = 80 } })
  local k = kit.maker()
  for _, y in ipairs{ 0.03, fh - 0.03 } do k.box("paint", V(0, y, 0.008), V(W, 0.06, 0.016), 0.004, col, 1) end
  for _, x in ipairs{ -W / 2 + 0.03, W / 2 - 0.03 } do k.box("paint", V(x, fh / 2, 0.008), V(0.06, fh, 0.016), 0.004, col, 1) end
  k.box("glass", V(0, fh / 2, 0.008), V(W - 0.12, fh - 0.12, 0.004), 0.0, V(0.8, 0.9, 0.9), 1)
  k.rod("metal", V(-0.1, fh - 0.03, 0.035), V(0.1, fh - 0.03, 0.035), 0.008, kit.CHROME, 8)
  kit.put(m, flap, k)
  flap:shape("flap", { box = { W, fh, 0.02 }, pos = { 0, fh / 2, 0.008 } })
  flap:marker("handle", { pos = { 0, fh - 0.03, 0.045 }, role = "hold" })
end
m:marker("front", { pos = { 0, 0, 0.75 }, turn = { 0, 180, 0 }, role = "stand" })
return m
