-- PIECES FOR A CLINIC OR A HOSPITAL WARD, each with parts that move (#191):
--
--   care_bed     a bed on casters: the whole bed rises (`lift`), the head end tilts up (`back`, on the lift) and a
--                rail each side drops (`rail_left`, `rail_right`)
--   iv_stand     a drip stand on five casters; its upper pole slides up (`pole`), with a bag on its hook
--   wheelchair   its two big wheels turn (`wheel_left`, `wheel_right`) and its foot rests fold up (`footrest_left`, ...)
--   exam_lamp    a lamp on a floor stand: the arm swings round (`arm`) and the head tips (`head`, on the arm)
--
-- Each stands on y = 0 at its own origin. The bed's head is toward -z; the wheelchair faces +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind   = { "care_bed", "iv_stand", "wheelchair", "exam_lamp" },
  fabric = kit.fabric_names,
}
local cloth = kit.fabric(a.fabric)
local GREY, CREAM = kit.metal.steel, kit.paint.cream
local m = model.new()
local far = m:level(40)
local c = kit.maker()
local function disc(r, thick, segs)            -- a disc about +x
  return mesh.rotate_z(mesh.lathe{ profile = { { 0, -thick / 2 }, { r, -thick / 2 }, { r, thick / 2 }, { 0, thick / 2 } }, segments = segs or 20 }, math.pi / 2)
end

if a.kind == "care_bed" then
  local W, L = 0.9, 2.0
  for _, x in ipairs{ -W / 2 + 0.06, W / 2 - 0.06 } do
    c.box("paint", V(x, 0.13, 0), V(0.05, 0.05, L - 0.2), 0.01, GREY, 1)
    for _, z in ipairs{ -L / 2 + 0.12, L / 2 - 0.12 } do kit.caster(c, V(x, 0, z), 0.05) end
  end
  for _, z in ipairs{ -0.5, 0.5 } do
    c.box("paint", V(0, 0.13, z), V(W - 0.12, 0.05, 0.05), 0.01, GREY, 1)
    c.box("paint", V(0, 0.3, z), V(0.12, 0.34, 0.12), 0.01, GREY, 1)                       -- the column's outer half
  end
  kit.put(m, m, c)
  m:shape("base", { box = { W, 0.3, L }, pos = { 0, 0.15, 0 } })
  local lift = m:node("lift", { pos = { 0, 0.45, 0 }, motion = { slide = { 0, 1, 0 }, min = 0, max = 0.3 } })
  local k = kit.maker()
  for _, z in ipairs{ -0.5, 0.5 } do k.box("metal", V(0, -0.15, z), V(0.08, 0.34, 0.08), 0.006, kit.CHROME, 1) end   -- and its inner half
  k.box("paint", V(0, 0.02, 0), V(W, 0.04, L), 0.01, CREAM, 1)
  k.cushion("fabric", V(0, 0.1, 0.42), V(W - 0.04, 0.12, L - 0.9), kit.WHITE, { r = 0.05 })       -- the mattress under the legs
  k.cushion("fabric", V(0, 0.17, 0.5), V(W - 0.02, 0.03, L - 1.1), cloth, { r = 0.05 })           -- the blanket
  k.box("paint", V(0, 0.3, L / 2 + 0.02), V(W, 0.5, 0.03), 0.015, CREAM, 2)                        -- the foot board
  k.box("paint", V(0, 0.4, -L / 2 - 0.02), V(W, 0.7, 0.03), 0.015, CREAM, 2)                       -- the head board
  kit.put(m, lift, k)
  kit.far_box(m, lift, "paint", { W, 0.2, L }, { 0, 0.1, 0 }, CREAM, far)
  lift:shape("deck", { box = { W, 0.16, L }, pos = { 0, 0.08, 0 }, friction = 0.8 })
  lift:marker("lie", { pos = { 0, 0.17, 0 }, turn = { 0, 0, 0 }, role = "lie", size = W })
  local back = lift:node("back", { pos = { 0, 0.04, -0.06 }, motion = { turn = { 1, 0, 0 }, min = 0, max = 65 } })
  local b = kit.maker()
  b.box("paint", V(0, 0.0, -0.45), V(W - 0.02, 0.02, 0.9), 0.006, CREAM, 1)
  b.cushion("fabric", V(0, 0.07, -0.45), V(W - 0.04, 0.12, 0.88), kit.WHITE, { r = 0.05 })
  b.cushion("fabric", V(0, 0.16, -0.68), V(W * 0.6, 0.08, 0.34), kit.WHITE, { r = 0.08, pinch = 0.5 })
  kit.put(m, back, b)
  back:shape("back", { box = { W, 0.14, 0.9 }, pos = { 0, 0.06, -0.45 } })
  for _, side in ipairs{ -1, 1 } do
    local name = side < 0 and "rail_left" or "rail_right"
    local rail = lift:node(name, { pos = { side * (W / 2 + 0.02), 0.12, 0.1 }, motion = { slide = { 0, -1, 0 }, min = 0, max = 0.3 } })
    local r = kit.maker()
    r.pipe("metal", { { 0, 0, -0.5 }, { 0, 0.32, -0.5 }, { 0, 0.32, 0.5 }, { 0, 0, 0.5 } }, 0.012, kit.CHROME, { bend = 0.06, sides = 8 })
    r.rod("metal", V(0, 0.16, -0.5), V(0, 0.16, 0.5), 0.008, kit.CHROME, 6)
    kit.put(m, rail, r)
    rail:marker(name, { pos = { 0, 0.32, 0 }, role = "hold" })
  end
elseif a.kind == "iv_stand" then
  for i = 0, 4 do
    local t = i * 2 * math.pi / 5
    c.rod("metal", V(0, 0.09, 0), V(0.26 * math.cos(t), 0.07, 0.26 * math.sin(t)), 0.012, kit.CHROME, 6)
    kit.caster(c, V(0.26 * math.cos(t), 0, 0.26 * math.sin(t)), 0.025)
  end
  c.rod("metal", V(0, 0.08, 0), V(0, 1.2, 0), 0.014, kit.CHROME, 8)
  c.turned("hard", V(0, 1.18, 0), { { 0.014, 0 }, { 0.022, 0 }, { 0.022, 0.04 }, { 0.014, 0.04 } }, kit.BLACK, 1, 10)   -- the collar that grips the upper pole
  kit.put(m, m, c)
  m:shape("pole", { capsule = { radius = 0.03, height = 1.2 }, pos = { 0, 0.6, 0 } })
  local pole = m:node("pole", { pos = { 0, 1.2, 0 }, motion = { slide = { 0, 1, 0 }, min = 0, max = 0.55 } })
  local k = kit.maker()
  k.rod("metal", V(0, -0.6, 0), V(0, 0.5, 0), 0.009, kit.CHROME, 8)
  for _, side in ipairs{ -1, 1 } do k.pipe("metal", { { 0, 0.5, 0 }, { side * 0.1, 0.5, 0 }, { side * 0.1, 0.44, 0 } }, 0.004, kit.CHROME, { bend = 0.03, sides = 6 }) end
  k.box("glass", V(0.1, 0.32, 0), V(0.09, 0.2, 0.03), 0.012, V(0.85, 0.9, 0.95), 2)                 -- the bag
  k.rod("hard", V(0.1, 0.22, 0), V(0.1, -0.3, 0.02), 0.002, V(0.9, 0.9, 0.9), 4)                    -- and the line down from it
  kit.put(m, pole, k)
  pole:marker("adjust", { pos = { 0, 0, 0.03 }, role = "hold" })
elseif a.kind == "wheelchair" then
  local W, R = 0.46, 0.3
  c.cushion("fabric", V(0, 0.5, 0.02), V(W, 0.05, 0.42), cloth, { r = 0.03 })
  c.cushion("fabric", V(0, 0.75, -0.2), V(W, 0.4, 0.03), cloth, { r = 0.03, face = "z" })
  for _, x in ipairs{ -W / 2, W / 2 } do
    c.pipe("metal", { { x, 0.95, -0.3 }, { x, 0.92, -0.2 }, { x, 0.47, -0.2 }, { x, 0.47, 0.24 }, { x, 0.12, 0.34 } }, 0.012, kit.CHROME, { bend = 0.05, sides = 8 })
    c.pipe("metal", { { x, 0.47, -0.2 }, { x, R, -0.12 }, { x, 0.14, 0.2 } }, 0.011, kit.CHROME, { bend = 0.05, sides = 8 })
    c.box("hard", V(x, 0.68, 0.0), V(0.04, 0.02, 0.3), 0.008, kit.BLACK, 1)                         -- the arm rest
    c.rod("metal", V(x, 0.47, 0.1), V(x, 0.67, 0.1), 0.009, kit.CHROME, 6)
    c.rod("hard", V(x, 0.95, -0.3), V(x, 0.95, -0.4), 0.014, kit.BLACK, 8)                          -- the push handle
    kit.caster(c, V(x, 0, 0.22), 0.06)
  end
  kit.put(m, m, c)
  kit.far_box(m, m, "fabric", { W + 0.1, 0.9, 0.7 }, { 0, 0.45, 0 }, cloth, far)
  m:shape("seat", { box = { W, 0.5, 0.5 }, pos = { 0, 0.25, 0 } })
  m:shape("back", { box = { W, 0.45, 0.05 }, pos = { 0, 0.72, -0.2 } })
  m:marker("seat", { pos = { 0, 0.53, 0.02 }, role = "seat", size = W })
  m:marker("push", { pos = { 0, 0.95, -0.4 }, role = "hold" })
  for _, side in ipairs{ -1, 1 } do
    local s = side < 0 and "left" or "right"
    local wheel = m:node("wheel_" .. s, { pos = { side * (W / 2 + 0.05), R, -0.12 }, motion = { turn = { 1, 0, 0 } } })
    local k = kit.maker()
    k.mesh("rubber", mesh.rotate_z(mesh.torus(R - 0.015, 0.015), math.pi / 2), kit.BLACK)
    k.mesh("metal", mesh.translate(mesh.rotate_z(mesh.torus(R - 0.05, 0.007), math.pi / 2), { side * 0.03, 0, 0 }), kit.CHROME)   -- the rim to push on
    for i = 0, 7 do k.mesh("metal", mesh.rotate_x(mesh.translate(mesh.box{ 0.004, R - 0.02, 0.004 }, { 0, (R - 0.02) / 2, 0 }), i * math.pi / 4), kit.CHROME) end
    k.mesh("metal", disc(0.03, 0.05, 10), kit.CHROME)
    kit.put(m, wheel, k)
    wheel:marker("rim_" .. s, { pos = { side * 0.03, R - 0.05, 0 }, role = "hold" })
    local rest = m:node("footrest_" .. s, { pos = { side * W / 2, 0.12, 0.34 }, motion = { turn = { 0, 0, side }, min = 0, max = 90 } })
    local f = kit.maker()
    f.box("hard", V(-side * 0.09, 0, 0.02), V(0.16, 0.012, 0.12), 0.004, kit.BLACK, 1)
    kit.put(m, rest, f)
  end
else
  for i = 0, 4 do
    local t = i * 2 * math.pi / 5
    c.rod("paint", V(0, 0.08, 0), V(0.28 * math.cos(t), 0.06, 0.28 * math.sin(t)), 0.014, CREAM, 6)
    kit.caster(c, V(0.28 * math.cos(t), 0, 0.28 * math.sin(t)), 0.025)
  end
  c.rod("paint", V(0, 0.07, 0), V(0, 1.5, 0), 0.016, CREAM, 8)
  kit.put(m, m, c)
  m:shape("post", { capsule = { radius = 0.03, height = 1.5 }, pos = { 0, 0.75, 0 } })
  local arm = m:node("arm", { pos = { 0, 1.5, 0 }, motion = { turn = { 0, 1, 0 }, min = -150, max = 150 } })
  local k = kit.maker()
  k.pipe("paint", { { 0, 0, 0 }, { 0, 0.25, 0.1 }, { 0, 0.3, 0.6 } }, 0.012, CREAM, { bend = 0.1, sides = 8 })
  kit.put(m, arm, k)
  local head = arm:node("head", { pos = { 0, 0.3, 0.6 }, motion = { turn = { 1, 0, 0 }, min = -60, max = 60 } })
  local h = kit.maker()
  h.turned("paint", V(0, -0.1, 0), { { 0, 0.1 }, { 0.04, 0.1 }, { 0.16, 0.02 }, { 0.16, 0 }, { 0.15, 0 }, { 0.04, 0.085 }, { 0, 0.085 } }, CREAM, 1, 20)
  h.turned("light", V(0, -0.1, 0), { { 0, 0.05 }, { 0.09, 0.05 }, { 0.09, 0.045 }, { 0, 0.045 } }, V(1, 0.98, 0.9), 1, 16)
  h.rod("hard", V(0.16, -0.09, 0), V(0.22, -0.09, 0), 0.008, kit.BLACK, 6)                          -- the handle to aim it by
  kit.put(m, head, h)
  head:marker("light", { pos = { 0, -0.1, 0 }, turn = { 90, 0, 0 }, role = "lamp" })
  head:marker("aim", { pos = { 0.22, -0.09, 0 }, role = "hold" })
end
return m
