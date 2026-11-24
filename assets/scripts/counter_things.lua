-- THINGS THAT STAND ON A SHOP'S OR A CAFE'S COUNTER, each with a part that moves (#191):
--
--   cash_register   a till: its cash drawer slides out (`drawer`), and the screen facing the staff tilts (`screen`)
--   microwave       its door swings open on the left (`door`); a lit window and a turning plate inside (`plate`)
--   espresso        an espresso machine: the lever on its group pulls down (`lever`), the steam wand swings (`wand`)
--
-- Each stands on y = 0 at its own origin (put it on a counter's top), its front toward +z (the staff's side).
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind   = { "cash_register", "microwave", "espresso" },
  finish = { "black", "white", "steel", "red" },
}
local col, part = kit.finish(a.finish)
local m = model.new()
local far = m:level(20)
local c = kit.maker()

if a.kind == "cash_register" then
  c.box(part, V(0, 0.06, 0), V(0.42, 0.12, 0.44), 0.01, col, 1)                        -- the base that holds the drawer
  c.box(part, V(0, 0.15, -0.08), V(0.36, 0.08, 0.26), 0.012, col, 1)
  c.tilted("hard", V(0, 0.2, 0.08), V(0.3, 0.012, 0.16), 0.004, 0.45, kit.BLACK * 2)   -- the keys
  c.box("light", V(0, 0.3, -0.19), V(0.2, 0.06, 0.004), 0.0, V(0.4, 1, 0.5), 1)        -- the customer's display, facing away
  c.rod(part, V(0, 0.18, -0.17), V(0, 0.3, -0.18), 0.012, col, 8)
  kit.put(m, m, c)
  m:shape("body", { box = { 0.42, 0.2, 0.44 }, pos = { 0, 0.1, 0 } })
  local screen = m:node("screen", { pos = { 0, 0.2, -0.05 }, turn = { -20, 0, 0 }, motion = { turn = { 1, 0, 0 }, min = -25, max = 25 } })
  local s = kit.maker()
  s.box("hard", V(0, 0.13, 0), V(0.3, 0.22, 0.02), 0.006, kit.BLACK, 1)
  s.box("screen", V(0, 0.13, 0.011), V(0.27, 0.19, 0.001), 0.0, V(0.05, 0.1, 0.2), 1)
  s.rod("hard", V(0, 0, 0), V(0, 0.05, -0.005), 0.014, kit.BLACK, 8)
  kit.put(m, screen, s)
  screen:marker("screen", { pos = { 0, 0.13, 0.015 }, role = "screen", size = 0.27 })
  local drawer = m:node("drawer", { pos = { 0, 0.02, 0.22 }, motion = { slide = { 0, 0, 1 }, min = 0, max = 0.3 } })
  local d = kit.maker()
  d.box(part, V(0, 0.04, 0.005), V(0.4, 0.08, 0.012), 0.004, col * 0.9, 1)
  d.box("hard", V(0, 0.01, -0.19), V(0.37, 0.008, 0.38), 0.0, kit.BLACK * 2, 1)
  for i = 0, 4 do d.box("hard", V(-0.148 + i * 0.074, 0.03, -0.12), V(0.066, 0.03, 0.2), 0.0, V(0.6, 0.68, 0.55), 1) end   -- the notes, in their slots
  kit.put(m, drawer, d)
  drawer:marker("drawer", { pos = { 0, 0.05, 0.02 }, role = "hold" })
elseif a.kind == "microwave" then
  local W, H, D, T = 0.5, 0.3, 0.38, 0.015
  for _, x in ipairs{ -W / 2 + T / 2, W / 2 - T / 2 } do c.box(part, V(x, H / 2, 0), V(T, H, D), 0.006, col, 1) end
  c.box(part, V(0, H / 2, -D / 2 + T / 2), V(W - 2 * T, H, T), 0.0, col, 1)
  c.box(part, V(0, T / 2, 0), V(W - 2 * T, T, D), 0.0, col, 1)
  c.box(part, V(0, H - T / 2, 0), V(W - 2 * T, T, D), 0.0, col, 1)
  c.box(part, V(W / 2 - 0.07, H / 2, D / 2 - T / 2), V(0.11, H - 2 * T, T), 0.0, col, 1)                    -- the control panel
  c.box("light", V(W / 2 - 0.07, H - 0.06, D / 2 + 0.001), V(0.08, 0.03, 0.002), 0.0, V(0.3, 1, 0.5), 1)
  c.mesh("metal", mesh.translate(mesh.rotate_x(mesh.lathe{ profile = { { 0, 0 }, { 0.025, 0 }, { 0.02, 0.015 }, { 0, 0.015 } }, segments = 14 }, math.pi / 2), { W / 2 - 0.07, 0.1, D / 2 }), kit.CHROME)
  c.box("light", V(-0.06, H - T - 0.004, 0), V(0.05, 0.004, 0.05), 0.0, V(1, 0.9, 0.6), 1)                  -- the lamp inside
  kit.put(m, m, c)
  m:shape("body", { box = { W, H, D }, pos = { 0, H / 2, 0 } })
  m:marker("buttons", { pos = { W / 2 - 0.07, 0.1, D / 2 + 0.02 }, role = "press" })
  m:marker("light", { pos = { -0.06, H - T - 0.03, 0 }, turn = { 90, 0, 0 }, role = "lamp" })   -- the lamp inside
  local plate = m:node("plate", { pos = { -0.06, T + 0.01, 0 }, motion = { turn = { 0, 1, 0 } } })
  local p = kit.maker()
  p.turned("glass", V(0, 0, 0), { { 0, 0 }, { 0.14, 0 }, { 0.15, 0.008 }, { 0, 0.006 } }, V(0.85, 0.9, 0.9), 1, 20)
  p.turned("ceramic", V(0.04, 0.008, 0.02), { { 0, 0 }, { 0.05, 0 }, { 0.06, 0.05 }, { 0.055, 0.05 }, { 0.045, 0.006 }, { 0, 0.006 } }, kit.PORCELAIN, 1, 14)   -- a bowl, off centre so the turning shows
  kit.put(m, plate, p)
  local door = m:node("door", { pos = { -W / 2, T, D / 2 }, motion = { turn = { 0, -1, 0 }, min = 0, max = 110 } })
  local k = kit.maker()
  local dw, dh = W - 0.125, H - 2 * T
  k.box("glass", V(dw / 2, dh / 2, 0.008), V(dw - 0.08, dh - 0.08, 0.008), 0.0, V(0.2, 0.25, 0.28), 1)
  for _, x in ipairs{ 0.02, dw - 0.02 } do k.box(part, V(x, dh / 2, 0.008), V(0.04, dh, 0.018), 0.004, col, 1) end
  for _, y in ipairs{ 0.02, dh - 0.02 } do k.box(part, V(dw / 2, y, 0.008), V(dw, 0.04, 0.018), 0.004, col, 1) end
  k.rod("metal", V(dw - 0.02, dh / 2 - 0.08, 0.04), V(dw - 0.02, dh / 2 + 0.08, 0.04), 0.007, kit.CHROME, 8)
  kit.put(m, door, k)
  door:shape("door", { box = { dw, dh, 0.02 }, pos = { dw / 2, dh / 2, 0.008 } })
  door:marker("handle", { pos = { dw - 0.02, dh / 2, 0.05 }, role = "hold" })
else
  local W = 0.6
  c.box(part, V(0, 0.03, 0), V(W, 0.06, 0.45), 0.01, col, 1)                           -- the drip tray's base
  c.box("metal", V(0, 0.065, 0.1), V(W - 0.06, 0.008, 0.2), 0.0, V(0.6, 0.61, 0.63), 1)
  c.box(part, V(0, 0.3, -0.12), V(W, 0.36, 0.2), 0.015, col, 2)                        -- the back
  c.box(part, V(0, 0.4, 0.0), V(W, 0.16, 0.44), 0.02, col, 2)                          -- the head, over the cups
  c.box("metal", V(0, 0.485, 0), V(W - 0.04, 0.01, 0.38), 0.0, V(0.75, 0.76, 0.78), 1) -- the cup warmer on top
  for i, x in ipairs{ -0.14, 0.14 } do
    c.turned("metal", V(x, 0.26, 0.1), { { 0, 0 }, { 0.035, 0 }, { 0.035, 0.06 }, { 0, 0.06 } }, kit.CHROME, 1, 14)     -- a group head
    c.rod("hard", V(x, 0.27, 0.12), V(x, 0.26, 0.26), 0.012, kit.BLACK, 8)                                              -- and its handle
    c.turned("ceramic", V(x, 0.07, 0.1), { { 0, 0 }, { 0.03, 0 }, { 0.038, 0.06 }, { 0.033, 0.06 }, { 0.026, 0.006 }, { 0, 0.006 } }, kit.PORCELAIN, 1, 12)
    m:marker("cup_" .. i, { pos = { x, 0.07, 0.1 } })
  end
  kit.put(m, m, c)
  m:shape("body", { box = { W, 0.49, 0.45 }, pos = { 0, 0.245, 0 } })
  local lever = m:node("lever", { pos = { 0, 0.45, 0.2 }, turn = { -30, 0, 0 }, motion = { turn = { 1, 0, 0 }, min = 0, max = 80 } })
  local l = kit.maker()
  l.rod("metal", V(0, 0, 0), V(0, 0.22, 0), 0.008, kit.CHROME, 8)
  l.mesh("hard", mesh.translate(mesh.sphere(0.022), { 0, 0.23, 0 }), kit.BLACK)
  kit.put(m, lever, l)
  lever:marker("lever", { pos = { 0, 0.23, 0 }, role = "hold" })
  local wand = m:node("wand", { pos = { W / 2 - 0.05, 0.32, 0.2 }, motion = { turn = { 0, 1, 0 }, min = -60, max = 60 } })
  local w = kit.maker()
  w.pipe("metal", { { 0, 0, 0 }, { 0, 0, 0.06 }, { 0, -0.2, 0.1 } }, 0.006, kit.CHROME, { bend = 0.03, sides = 8 })
  kit.put(m, wand, w)
  wand:marker("steam", { pos = { 0, -0.2, 0.1 } })
end
m:marker("front", { pos = { 0, -0.9, 0.6 }, turn = { 0, 180, 0 }, role = "stand" })
return m
