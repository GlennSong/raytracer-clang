-- A DRINKS VENDING MACHINE (#191): a lit cabinet with rows of sample bottles and cans behind glass, a button and
-- a price under each, a coin slot, and a flap at the foot where the drink drops, on a node, `flap`, that pushes
-- in 70 degrees. Markers: `coins`, `buttons`, `flap`, `front`, `lamp`.
-- It stands on y = 0 at its own origin, its front toward +z.
local kit = require "model_kit"
local scatter = require "scatter"
local V = kit.V

local a = params{
  width  = { 1.0, min = 0.7, max = 1.3 },         -- metres
  rows   = { 3, min = 2, max = 4, step = 1 },
  paint  = { "white", "red", "blue", "green", "orange", "yellow" },
}
local W, ROWS = a.width, math.floor(a.rows + 0.5)
local col = a.paint == "white" and kit.WHITE or kit.paint[a.paint]
local m = model.new()
local far = m:level(50)
local H, D = 1.83, 0.7
local c = kit.maker()
c.box("paint", V(0, H / 2, -0.02), V(W, H, D - 0.04), 0.012, col, 2)
-- the lit window and what is in it
local y0, y1 = 0.72, H - 0.08
c.box("light", V(-0.04, (y0 + y1) / 2, D / 2 - 0.045), V(W - 0.2, y1 - y0, 0.01), 0.0, V(0.97, 0.97, 0.93), 1)
c.box("glass", V(-0.04, (y0 + y1) / 2, D / 2 - 0.0), V(W - 0.2, y1 - y0, 0.006), 0.0, V(0.9, 0.95, 0.95), 1)
local per = math.max(4, math.floor((W - 0.24) / 0.085))
local rowh = (y1 - y0) / ROWS
local BOTTLE = { { 0, 0 }, { 0.028, 0 }, { 0.029, 0.12 }, { 0.012, 0.16 }, { 0.012, 0.185 }, { 0, 0.185 } }
local CAN = { { 0, 0 }, { 0.028, 0 }, { 0.031, 0.01 }, { 0.031, 0.105 }, { 0.026, 0.115 }, { 0, 0.115 } }
local colours = { V(0.1, 0.45, 0.2), V(0.75, 0.1, 0.1), V(0.9, 0.6, 0.1), V(0.15, 0.3, 0.7), V(0.35, 0.2, 0.1), V(0.1, 0.1, 0.12) }
for r = 0, ROWS - 1 do
  local y = y0 + r * rowh
  c.box("paint", V(-0.04, y + 0.03, D / 2 - 0.02), V(W - 0.2, 0.06, 0.03), 0.0, kit.WHITE * 0.85, 1)        -- the shelf, with a button under each drink
  for i = 0, per - 1 do
    local x = -W / 2 + 0.06 + (i + 0.5) * (W - 0.2) / per
    c.box("light", V(x, y + 0.03, D / 2 - 0.003), V(0.03, 0.016, 0.004), 0.0, i % 3 == 0 and V(1, 0.3, 0.2) or V(0.3, 0.5, 1), 1)
  end
  for half = 0, 1 do
    local p = model.new()
    local can = (r + half) % 2 == 0
    p:add(mesh.paint(mesh.lathe{ profile = can and CAN or BOTTLE, segments = 10, smooth = true }, colours[(r * 2 + half * 3) % #colours + 1]))
    local x0 = -W / 2 + 0.06 + half * (W - 0.2) / 2
    m:use(p, scatter.along{ path = { { x0 + 0.04, y + 0.06, D / 2 - 0.035 }, { x0 + (W - 0.2) / 2 - 0.04, y + 0.06, D / 2 - 0.035 } },
                            spacing = (W - 0.2) / per, normal = { 0, 1, 0 } })
  end
end
-- the money: a slot, a note reader, a small display, the coin return
local MX = W / 2 - 0.06
c.box("hard", V(MX, 1.15, D / 2 - 0.038), V(0.085, 0.34, 0.004), 0.0, kit.BLACK * 1.5, 1)
c.box("metal", V(MX, 1.24, D / 2 - 0.034), V(0.012, 0.04, 0.004), 0.0, kit.CHROME, 1)
c.box("light", V(MX, 1.14, D / 2 - 0.034), V(0.06, 0.025, 0.003), 0.0, V(1, 0.4, 0.2), 1)
c.box("metal", V(MX, 1.04, D / 2 - 0.034), V(0.06, 0.012, 0.004), 0.0, kit.CHROME, 1)
c.box("hard", V(MX, 0.62, D / 2 - 0.03), V(0.07, 0.07, 0.02), 0.01, kit.BLACK * 1.5, 1)
-- the opening the drink drops into
c.box("hard", V(-0.04, 0.36, D / 2 - 0.1), V(W - 0.3, 0.2, 0.12), 0.0, kit.BLACK, 1)
kit.put(m, m, c)
kit.far_box(m, m, "paint", { W, H, D }, { 0, H / 2, 0 }, col, far)
kit.far_box(m, m, "light", { W - 0.2, y1 - y0, 0.01 }, { -0.04, (y0 + y1) / 2, D / 2 + 0.002 }, V(0.97, 0.97, 0.93), far)
m:shape("body", { box = { W, H, D }, pos = { 0, H / 2, 0 } })
m:marker("coins", { pos = { MX, 1.24, D / 2 }, role = "press" })
m:marker("buttons", { pos = { -0.04, y0 + 0.03, D / 2 }, role = "press" })
m:marker("lamp", { pos = { 0, 1.3, D / 2 + 0.05 }, role = "lamp" })
m:marker("front", { pos = { 0, 0, D / 2 + 0.6 }, turn = { 0, 180, 0 }, role = "stand" })
local flap = m:node("flap", { pos = { -0.04, 0.46, D / 2 - 0.035 }, motion = { turn = { 1, 0, 0 }, min = 0, max = 70 } })
local k = kit.maker()
k.box("hard", V(0, -0.1, 0), V(W - 0.3, 0.2, 0.008), 0.0, V(0.25, 0.27, 0.3), 1)
kit.put(m, flap, k)
flap:marker("flap", { pos = { 0, -0.12, 0.01 }, role = "hold" })
return m
