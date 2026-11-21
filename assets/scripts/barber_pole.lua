-- A BARBER'S POLE that turns (#191): red, white and blue stripes wound round a drum inside a glass tube, on a
-- bracket from the wall. The drum is on a node, `stripes`, that turns without end, which makes the stripes seem
-- to climb. A marker, `lamp`, is in its middle (they are lit from inside).
-- It HANGS on a wall: its origin is where the bracket meets the wall, and the pole stands out along +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  height = { 0.7, min = 0.4, max = 1.2 },         -- the glass tube's height (metres)
  radius = { 0.09, min = 0.05, max = 0.15 },
}
local H, R = a.height, a.radius
local OUT = 0.22                               -- how far from the wall the pole's middle is
local m = model.new()
local far = m:level(40)
local STEEL = V(0.78, 0.79, 0.8)
local c = kit.maker()
c.box("metal", V(0, 0, 0.006), V(0.1, 0.22, 0.012), 0.004, STEEL, 1)                  -- the plate on the wall
for _, y in ipairs{ -H / 2 - 0.03, H / 2 + 0.03 } do c.rod("metal", V(0, y * 0.25, 0.01), V(0, y, OUT), 0.012, STEEL, 8) end
for _, s in ipairs{ -1, 1 } do                  -- a cap at each end, and a ball on each cap
  c.turned("metal", V(0, s > 0 and H / 2 or -H / 2 - 0.06, OUT), { { 0.0, 0.0 }, { R + 0.012, 0.0 }, { R + 0.012, 0.06 }, { 0.0, 0.06 } }, STEEL, 1, 20)
  c.mesh("metal", mesh.translate(mesh.sphere(R * 0.55), { 0, s * (H / 2 + 0.06 + R * 0.4), OUT }), STEEL)
end
c.turned("glass", V(0, -H / 2, OUT), { { R + 0.006, 0.0 }, { R + 0.006, H } }, V(0.9, 0.95, 0.95), 1, 24)
kit.put(m, m, c)
kit.far_box(m, m, "light", { R * 1.6, H, R * 1.6 }, { 0, 0, OUT }, V(0.9, 0.5, 0.5), far)
m:shape("pole", { capsule = { radius = R + 0.01, height = H }, pos = { 0, 0, OUT } })
m:marker("lamp", { pos = { 0, 0, OUT }, role = "lamp" })

local stripes = m:node("stripes", { pos = { 0, -H / 2, OUT }, motion = { turn = { 0, 1, 0 } } })
local k = kit.maker()
k.turned("hard", V(0, 0, 0), { { R - 0.004, 0.0 }, { R - 0.004, H } }, V(0.97, 0.97, 0.95), 1, 24)
-- two bands wound up the drum, one red and one blue, half a turn apart
local TURNS, STEPS, BAND = 1.5, 36, H / 1.5 * 0.22
for i, col in ipairs{ { 0.85, 0.1, 0.1 }, { 0.1, 0.2, 0.7 } } do
  local start = (i - 1) * math.pi
  local function at(j, lift)                    -- a point on the drum, j steps along the band
    local t = start + 2 * math.pi * TURNS * j / STEPS
    local y = (H - BAND) * j / STEPS + lift
    return { (R - 0.002) * math.cos(t), y, (R - 0.002) * math.sin(t) }, { math.cos(t), 0, math.sin(t) }
  end
  for j = 0, STEPS - 1 do
    local p0, n = at(j, 0)
    local p1 = at(j + 1, 0)
    local p2, p3 = at(j + 1, BAND), at(j, BAND)
    k.put("hard", mesh.quad(p0, p1, p2, p3, n, col))
  end
end
kit.put(m, stripes, k)
return m
