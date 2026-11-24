-- A FREESTANDING MACHINE WITH A SCREEN (#191): a bank's cash machine, a mall's directory, a station's or a town
-- hall's ticket machine. Nothing on these moves; each is a lit screen and the markers for using it.
--
--   atm         a cash machine: screen, keypad, card and cash slots, a hood
--   directory   a tall slab with a big lit map and "you are here"
--   tickets     a queue-number or ticket machine: a small screen, one big button, a slot the ticket comes from
--
-- It stands on y = 0 at its own origin, its screen toward +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind  = { "atm", "directory", "tickets" },
  paint = { "blue", "green", "red", "orange", "grey", "black" },
}
local col = ({ grey = V(0.55, 0.57, 0.6), black = kit.BLACK * 1.8 })[a.paint] or kit.paint[a.paint]
local m = model.new()
local far = m:level(50)
local c = kit.maker()
local W, H, D
if a.kind == "atm" then
  W, H, D = 0.6, 1.6, 0.55
  c.box("paint", V(0, 0.5, 0), V(W, 1.0, D), 0.015, col, 2)
  c.box("paint", V(0, 1.3, -0.08), V(W, 0.6, D - 0.16), 0.015, col, 2)
  c.tilted("hard", V(0, 1.03, 0.12), V(W - 0.08, 0.02, 0.26), 0.006, 0.25, kit.BLACK * 2)        -- the shelf with the keypad
  c.tilted("light", V(0.0, 1.045, 0.13), V(0.12, 0.004, 0.12), 0.0, 0.25, V(0.8, 0.8, 0.8))
  c.box("screen", V(0, 1.32, D / 2 - 0.158), V(0.34, 0.26, 0.004), 0.0, V(0.1, 0.3, 0.6), 1)
  c.box("light", V(0.21, 1.2, D / 2 - 0.158), V(0.07, 0.012, 0.004), 0.0, V(0.3, 1, 0.4), 1)      -- the card slot
  c.box("hard", V(0, 0.9, D / 2 + 0.001), V(0.3, 0.03, 0.004), 0.0, kit.BLACK, 1)                 -- the cash slot
  c.box("paint", V(0, 1.62, 0.05), V(W + 0.04, 0.04, D + 0.1), 0.012, col * 0.8, 1)               -- the hood
  m:marker("screen", { pos = { 0, 1.32, D / 2 - 0.15 }, role = "screen", size = 0.3 })
  m:marker("keys", { pos = { 0, 1.05, 0.14 }, role = "press" })
  m:marker("cash", { pos = { 0, 0.9, D / 2 + 0.02 }, role = "hold" })
elseif a.kind == "directory" then
  W, H, D = 1.0, 2.1, 0.16
  c.box("paint", V(0, H / 2, 0), V(W, H, D), 0.02, col, 2)
  c.box("paint", V(0, 0.02, 0), V(W + 0.2, 0.04, 0.5), 0.01, col * 0.7, 1)
  for _, z in ipairs{ -1, 1 } do
    c.box("light", V(0, 1.25, z * (D / 2 + 0.001)), V(W - 0.12, 1.3, 0.003), 0.0, V(0.95, 0.95, 0.9), 1)       -- the lit map, on both faces
    for i = 0, 4 do c.box("hard", V(-0.25 + (i % 3) * 0.22, 1.5 - math.floor(i / 3) * 0.4, z * (D / 2 + 0.003)), V(0.18, 0.3, 0.002), 0.0, ({ kit.paint.blue, kit.paint.green, kit.paint.orange, kit.paint.red, kit.paint.yellow })[i + 1], 1) end
    c.box("light", V(0.05, 1.0, z * (D / 2 + 0.005)), V(0.03, 0.03, 0.002), 0.0, V(1, 0.1, 0.1), 1)             -- you are here
  end
  m:marker("screen", { pos = { 0, 1.25, D / 2 + 0.01 }, role = "screen", size = 1.0 })
else
  W, H, D = 0.4, 1.35, 0.35
  c.box("paint", V(0, 0.5, 0), V(0.14, 1.0, 0.14), 0.02, col * 0.8, 1)
  c.box("paint", V(0, 0.015, 0), V(W, 0.03, D), 0.01, col * 0.8, 1)
  c.tilted("paint", V(0, 1.15, 0), V(W, 0.4, 0.2), 0.02, -0.35, col)
  c.tilted("screen", V(0, 1.24, 0.085), V(0.26, 0.12, 0.004), 0.0, -0.35, V(0.1, 0.25, 0.15))
  c.tilted("light", V(0, 1.1, 0.135), V(0.09, 0.09, 0.012), 0.004, -0.35, V(0.3, 1, 0.4))                    -- the one big button
  c.tilted("hard", V(0, 1.0, 0.165), V(0.12, 0.015, 0.006), 0.0, -0.35, kit.BLACK)                           -- the slot
  m:marker("button", { pos = { 0, 1.1, 0.15 }, role = "press" })
  m:marker("ticket", { pos = { 0, 1.0, 0.18 }, role = "hold" })
end
kit.put(m, m, c)
kit.far_box(m, m, "paint", { W, H, D }, { 0, H / 2, 0 }, col, far)
m:shape("body", { box = { W, H, D }, pos = { 0, H / 2, 0 } })
m:marker("front", { pos = { 0, 0, D / 2 + 0.6 }, turn = { 0, 180, 0 }, role = "stand" })
m:marker("lamp", { pos = { 0, H * 0.7, D / 2 + 0.05 }, role = "lamp" })
return m
