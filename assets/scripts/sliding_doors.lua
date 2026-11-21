-- AUTOMATIC SLIDING DOORS (#191; a convenience store's or a mall's way in): two glass panels that part in the
-- middle and slide behind a fixed glass panel each side, under a header that holds the sensor. Each moving panel
-- is on a node, `door_left`, `door_right`. Markers: `sensor`, and `outside` and `inside` on the floor.
-- It stands on y = 0 at its own origin, in the plane of the shop front, the street toward +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  width  = { 1.6, min = 1.2, max = 2.6 },         -- the way through when open (metres)
  height = { 2.2, min = 2.0, max = 2.6 },
  finish = { "steel", "black", "white" },
}
local W, H = a.width, a.height
local col = ({ steel = V(0.7, 0.71, 0.73), black = kit.BLACK * 1.8, white = kit.WHITE })[a.finish]
local GLASS = V(0.8, 0.9, 0.92)
local m = model.new()
local far = m:level(60)
local P = W / 2                                 -- one panel's width

local function panel(k, x, z, w)                -- a glass panel in a thin frame, its foot's middle at x
  k.box("glass", V(x, H / 2, z), V(w - 0.08, H - 0.1, 0.01), 0.0, GLASS, 1)
  for _, dx in ipairs{ -w / 2 + 0.02, w / 2 - 0.02 } do k.box("metal", V(x + dx, H / 2, z), V(0.04, H, 0.035), 0.004, col, 1) end
  for _, y in ipairs{ 0.04, H - 0.03 } do k.box("metal", V(x, y, z), V(w, y < 1 and 0.08 or 0.06, 0.035), 0.004, col, 1) end
end
local c = kit.maker()
for _, s in ipairs{ -1, 1 } do
  panel(c, s * (P + P / 2), -0.04, P)                                                -- the fixed panels
  m:shape("fixed_" .. (s < 0 and "left" or "right"), { box = { P, H, 0.04 }, pos = { s * 1.5 * P, H / 2, -0.04 } })
end
c.box("metal", V(0, H + 0.1, -0.01), V(4 * P + 0.04, 0.2, 0.16), 0.008, col, 1)      -- the header
c.box("hard", V(0, H + 0.02, 0.075), V(0.1, 0.04, 0.03), 0.008, kit.BLACK, 1)        -- the sensor
c.box("metal", V(0, 0.004, 0), V(4 * P, 0.008, 0.1), 0.0, col * 0.7, 1)              -- the track in the floor
kit.put(m, m, c)
kit.far_box(m, m, "glass", { 4 * P, H, 0.04 }, { 0, H / 2, 0 }, GLASS, far)
m:shape("header", { box = { 4 * P, 0.2, 0.16 }, pos = { 0, H + 0.1, -0.01 } })
m:marker("sensor", { pos = { 0, H + 0.02, 0.09 }, turn = { 60, 0, 0 } })
m:marker("outside", { pos = { 0, 0, 1.2 }, turn = { 0, 180, 0 }, role = "stand" })
m:marker("inside", { pos = { 0, 0, -1.2 }, role = "stand" })
for _, s in ipairs{ -1, 1 } do
  local name = s < 0 and "door_left" or "door_right"
  local door = m:node(name, { pos = { s * P / 2, 0, 0.01 }, motion = { slide = { s, 0, 0 }, min = 0, max = P - 0.04 } })
  local k = kit.maker()
  panel(k, 0, 0, P)
  kit.put(m, door, k)
  door:shape(name, { box = { P, H, 0.04 }, pos = { 0, H / 2, 0 } })
end
return m
