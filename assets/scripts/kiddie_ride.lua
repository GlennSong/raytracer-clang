-- A COIN-OPERATED RIDE for a small child (#191; outside a supermarket, in a mall): a little car or a rocket on a
-- base. The ride is on a node, `rock`, that tips back and forth 9 degrees either way, under which a node, `bob`,
-- lifts it up and down 6 cm; together they make its lazy gallop. Markers: `seat`, `coins`, `lamp`.
-- It stands on y = 0 at its own origin, facing +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind  = { "car", "rocket" },
  paint = { "red", "blue", "yellow", "green", "orange" },
}
local col = kit.paint[a.paint]
local m = model.new()
local far = m:level(40)
local c = kit.maker()
c.box("paint", V(0, 0.09, 0), V(0.7, 0.18, 1.1), 0.03, kit.paint.blue * 0.7, 2)
c.box("paint", V(0.28, 0.5, -0.4), V(0.1, 0.66, 0.1), 0.02, kit.paint.yellow, 1)         -- the coin post
c.box("light", V(0.28, 0.72, -0.348), V(0.07, 0.07, 0.004), 0.0, V(1, 0.9, 0.3), 1)
c.box("metal", V(0.28, 0.6, -0.348), V(0.012, 0.04, 0.004), 0.0, kit.CHROME, 1)
c.turned("rubber", V(0, 0.18, 0), { { 0.09, 0 }, { 0.11, 0.03 }, { 0.09, 0.06 }, { 0.11, 0.09 }, { 0.09, 0.12 } }, kit.BLACK, 1, 14)   -- the boot round the lift
kit.put(m, m, c)
kit.far_box(m, m, "paint", { 0.7, 0.18, 1.1 }, { 0, 0.09, 0 }, kit.paint.blue, far)
m:shape("base", { box = { 0.7, 0.18, 1.1 }, pos = { 0, 0.09, 0 }, friction = 0.9 })
m:shape("coin_post", { box = { 0.1, 0.66, 0.1 }, pos = { 0.28, 0.5, -0.4 } })
m:marker("coins", { pos = { 0.28, 0.6, -0.33 }, role = "press" })
m:marker("front", { pos = { 0.9, 0, 0 }, turn = { 0, 270, 0 }, role = "stand" })
local bob = m:node("bob", { pos = { 0, 0.3, 0 }, motion = { slide = { 0, 1, 0 }, min = 0, max = 0.06 } })
local rock = bob:node("rock", { pos = { 0, 0, 0 }, motion = { turn = { 1, 0, 0 }, min = -9, max = 9 } })
local k = kit.maker()
if a.kind == "car" then
  k.box("paint", V(0, 0.16, 0), V(0.5, 0.2, 0.9), 0.06, col, 3)
  k.box("paint", V(0, 0.3, -0.22), V(0.46, 0.14, 0.3), 0.05, col, 2)                     -- the seat's back
  k.cushion("fabric", V(0, 0.27, 0.0), V(0.36, 0.05, 0.26), kit.FABRIC[6], { r = 0.04 })
  k.tilted("glass", V(0, 0.36, 0.28), V(0.4, 0.16, 0.01), 0.0, -0.5, V(0.8, 0.9, 0.95))
  for _, x in ipairs{ -0.26, 0.26 } do for _, z in ipairs{ -0.28, 0.28 } do
    k.mesh("rubber", mesh.translate(mesh.rotate_z(mesh.lathe{ profile = { { 0, -0.03 }, { 0.09, -0.03 }, { 0.1, 0 }, { 0.09, 0.03 }, { 0, 0.03 } }, segments = 14, smooth = true }, math.pi / 2), { x, 0.1, z }), kit.BLACK)
  end end
  for _, x in ipairs{ -0.16, 0.16 } do k.mesh("light", mesh.translate(mesh.sphere(0.04), { x, 0.18, 0.44 }), V(1, 1, 0.8)) end
  k.pipe("hard", { { 0, 0.25, 0.2 }, { 0, 0.4, 0.14 } }, 0.012, kit.BLACK, { sides = 6 })   -- the wheel's column
  k.mesh("hard", mesh.translate(mesh.rotate_x(mesh.torus(0.07, 0.012), math.pi / 2 - 0.4), { 0, 0.41, 0.135 }), kit.BLACK)
else
  k.mesh("paint", mesh.translate(mesh.rotate_x(mesh.lathe{ profile = { { 0, -0.5 }, { 0.16, -0.46 }, { 0.22, -0.1 }, { 0.2, 0.25 }, { 0.08, 0.5 }, { 0, 0.56 } }, segments = 20, smooth = true }, math.pi / 2), { 0, 0.26, 0 }), col)
  for i = 0, 2 do
    local fin = mesh.translate(mesh.rounded_box({ 0.02, 0.22, 0.3 }, 0.008), { 0, 0.26, -0.36 })
    k.mesh("paint", mesh.translate(mesh.rotate_z(mesh.translate(fin, { 0, -0.01, 0 }), 2 * math.pi * i / 3 + math.pi), { 0, 0.26 * (1 - math.cos(2 * math.pi * i / 3 + math.pi)), 0 }), kit.WHITE)
  end
  k.cushion("fabric", V(0, 0.47, -0.05), V(0.26, 0.05, 0.34), kit.FABRIC[6], { r = 0.04 })   -- it is ridden astride
  k.pipe("metal", { { -0.14, 0.52, 0.2 }, { 0, 0.5, 0.2 }, { 0.14, 0.52, 0.2 } }, 0.012, kit.CHROME, { sides = 6 })
  k.mesh("light", mesh.translate(mesh.sphere(0.05), { 0, 0.26, 0.56 }), V(1, 0.4, 0.2))
end
kit.put(m, rock, k)
kit.far_box(m, rock, "paint", { 0.5, 0.35, 0.9 }, { 0, 0.2, 0 }, col, far)
rock:shape("ride", { box = { 0.5, 0.36, 0.9 }, pos = { 0, 0.2, 0 }, friction = 0.9 })
rock:marker("seat", { pos = { 0, a.kind == "car" and 0.3 or 0.5, -0.02 }, role = "seat", size = 0.3 })
rock:marker("lamp", { pos = { 0, 0.3, 0.5 }, role = "lamp" })
return m
