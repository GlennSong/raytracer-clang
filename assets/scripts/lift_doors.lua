-- A LIFT'S DOORS that slide open (#191): a steel frame in a wall, two panels that part in the middle, a call
-- button beside them and a lit floor number above. Each panel is on a node, `door_left` and `door_right`, sliding
-- half the opening's width into the wall. Markers: `call` (the button), `number`, `wait` on the floor outside.
-- It stands on y = 0 at its own origin, in the plane of the wall, the landing toward +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  width  = { 1.0, min = 0.8, max = 1.6 },         -- the opening's width (metres)
  height = { 2.1, min = 2.0, max = 2.4 },
  finish = { "steel", "brass", "black" },
}
local W, H = a.width, a.height
local col, part = kit.finish(a.finish)
local m = model.new()
local far = m:level(60)
local F = 0.09                                 -- the frame's width

local c = kit.maker()
for _, x in ipairs{ -W / 2 - F / 2, W / 2 + F / 2 } do c.box("metal", V(x, H / 2, 0.02), V(F, H, 0.1), 0.006, col, 1) end
c.box("metal", V(0, H + 0.11, 0.02), V(W + 2 * F, 0.22, 0.1), 0.006, col, 1)
c.box("hard", V(0, H + 0.11, 0.072), V(0.24, 0.1, 0.004), 0.0, kit.BLACK, 1)          -- the floor number's window
c.box("light", V(0, H + 0.11, 0.075), V(0.06, 0.06, 0.002), 0.0, V(1, 0.5, 0.1), 1)
c.box("metal", V(0, 0.005, 0.0), V(W, 0.01, 0.12), 0.0, col * 0.7, 1)                 -- the sill
c.box("metal", V(W / 2 + F + 0.12, 1.1, 0.008), V(0.09, 0.2, 0.016), 0.004, col, 1)   -- the call panel, on the wall
c.turned("light", V(W / 2 + F + 0.12, 1.1, 0.016), { { 0.0, 0.0 }, { 0.018, 0.0 }, { 0.018, 0.004 }, { 0.0, 0.004 } }, V(1, 0.9, 0.6), 1, 12)
kit.put(m, m, c)
kit.far_box(m, m, "metal", { W + 2 * F, H + 0.22, 0.1 }, { 0, (H + 0.22) / 2, 0.02 }, col, far)
for _, x in ipairs{ -W / 2 - F / 2, W / 2 + F / 2 } do m:shape("frame_" .. (x < 0 and "left" or "right"), { box = { F, H, 0.1 }, pos = { x, H / 2, 0.02 } }) end
m:shape("head", { box = { W + 2 * F, 0.22, 0.1 }, pos = { 0, H + 0.11, 0.02 } })
m:marker("call", { pos = { W / 2 + F + 0.12, 1.1, 0.03 }, role = "press" })
m:marker("number", { pos = { 0, H + 0.11, 0.08 } })
m:marker("wait", { pos = { 0, 0, 0.9 }, turn = { 0, 180, 0 }, role = "stand" })
m:marker("car", { pos = { 0, 0, -0.9 }, role = "stand" })

for _, side in ipairs{ -1, 1 } do
  local name = side < 0 and "door_left" or "door_right"
  local door = m:node(name, { pos = { side * W / 4, 0, -0.01 }, motion = { slide = { side, 0, 0 }, min = 0, max = W / 2 - 0.02 } })
  local k = kit.maker()
  k.box("metal", V(0, H / 2, 0), V(W / 2 - 0.004, H - 0.01, 0.03), 0.004, col * 0.95, 1)
  k.box("metal", V(0, H / 2, 0.016), V(W / 2 - 0.12, H - 0.3, 0.004), 0.0, col * 1.05, 1)       -- a raised panel on its face
  kit.put(m, door, k)
  kit.far_box(m, door, "metal", { W / 2, H, 0.03 }, { 0, H / 2, 0 }, col, far)
  door:shape(name, { box = { W / 2, H, 0.04 }, pos = { 0, H / 2, 0 } })
end
return m
