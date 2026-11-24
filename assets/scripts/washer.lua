-- A WASHING MACHINE OR A DRYER whose door opens and whose drum turns (#191; a laundromat's rows of them). The
-- round door is on a node, `door`, hinged on the left and opening 130 degrees; the drum inside is on a node,
-- `drum`, that turns without end and can be seen through the door's glass. Markers: `handle`, `controls`.
-- It stands on y = 0 at its own origin, its door facing +z. Machines stack: a dryer's origin can stand on a
-- washer's top.
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind   = { "washer", "dryer" },
  width  = { 0.6, min = 0.55, max = 0.8 },        -- metres; it is as deep as it is wide
  height = { 0.85, min = 0.8, max = 1.1 },
  finish = { "white", "steel" },
}
local W, H = a.width, a.height
local col = a.finish == "steel" and V(0.62, 0.64, 0.66) or kit.WHITE
local part = a.finish == "steel" and "metal" or "paint"
local m = model.new()
local far = m:level(30)
local T = 0.02
local CY = H * 0.47                           -- the height of the door's middle
local HOLE = 0.3                              -- the square hole in the front that the round door covers
local function facing(mesh_) return mesh.rotate_x(mesh_, math.pi / 2) end

local c = kit.maker()
for _, x in ipairs{ -W / 2 + T / 2, W / 2 - T / 2 } do c.box(part, V(x, H / 2, 0), V(T, H, W), 0.006, col, 1) end
c.box(part, V(0, H / 2, -W / 2 + T / 2), V(W - 2 * T, H, T), 0.0, col, 1)
c.box(part, V(0, H - T / 2, 0), V(W - 2 * T, T, W), 0.0, col, 1)
c.box(part, V(0, T / 2, 0), V(W - 2 * T, T, W), 0.0, col, 1)
-- the front: four strips round the hole
local z = W / 2 - T / 2
c.box(part, V(0, (CY - HOLE / 2) / 2, z), V(W - 2 * T, CY - HOLE / 2, T), 0.0, col, 1)
c.box(part, V(0, (H + CY + HOLE / 2) / 2, z), V(W - 2 * T, H - CY - HOLE / 2, T), 0.0, col, 1)
for _, s in ipairs{ -1, 1 } do c.box(part, V(s * (W / 2 - T + HOLE / 2) / 2, CY, z), V(W / 2 - T - HOLE / 2, HOLE, T), 0.0, col, 1) end
-- the ring round the opening, wide enough to cover the square hole's corners
c.mesh(part, mesh.translate(facing(mesh.lathe{ profile = { { 0.14, 0.0 }, { 0.225, 0.0 }, { 0.225, 0.012 }, { 0.15, 0.02 }, { 0.14, 0.0 } }, segments = 32, smooth = true }), { 0, CY, W / 2 }), col)
-- the control panel: a dark strip, a dial and a lit window
c.box("hard", V(0, H - 0.07, W / 2 + 0.002), V(W - 0.06, 0.09, 0.004), 0.0, kit.BLACK * 1.5, 1)
c.mesh("metal", mesh.translate(facing(mesh.lathe{ profile = { { 0, 0 }, { 0.03, 0 }, { 0.026, 0.02 }, { 0, 0.02 } }, segments = 16 }), { W * 0.22, H - 0.07, W / 2 + 0.004 }), kit.CHROME)
c.box("light", V(-W * 0.15, H - 0.07, W / 2 + 0.005), V(0.12, 0.035, 0.002), 0.0, a.kind == "dryer" and V(1, 0.6, 0.2) or V(0.4, 0.8, 1), 1)
kit.put(m, m, c)
kit.far_box(m, m, part, { W, H, W }, { 0, H / 2, 0 }, col, far)
m:shape("body", { box = { W, H, W }, pos = { 0, H / 2, 0 } })
m:marker("controls", { pos = { W * 0.22, H - 0.07, W / 2 + 0.03 }, role = "press" })
m:marker("front", { pos = { 0, 0, W / 2 + 0.6 }, turn = { 0, 180, 0 }, role = "stand" })

-- THE DRUM: a steel tub open toward the door, with three paddles so its turning shows
local drum = m:node("drum", { pos = { 0, CY, 0 }, motion = { turn = { 0, 0, 1 } } })
local d = kit.maker()
local DR, DL = 0.21, W - 0.16
d.mesh("metal", mesh.translate(facing(mesh.lathe{ profile = { { 0.0, 0.0 }, { DR, 0.0 }, { DR, DL }, { DR - 0.012, DL }, { DR - 0.012, 0.012 }, { 0.0, 0.012 } }, segments = 28 }), { 0, 0, -W / 2 + 0.06 }), V(0.7, 0.72, 0.75))
for i = 0, 2 do
  local t = 2 * math.pi * i / 3
  d.mesh("hard", mesh.rotate_z(mesh.translate(mesh.rounded_box({ 0.05, 0.035, DL * 0.8 }, 0.012), { 0, DR - 0.028, 0.02 }), t), V(0.85, 0.86, 0.9))
end
kit.put(m, drum, d)

-- THE DOOR: a ring with a glass bowl in it. Its space: the origin is the hinge, and it runs along +x.
local HX = 0.235
local door = m:node("door", { pos = { -HX, CY, W / 2 + 0.02 }, motion = { turn = { 0, -1, 0 }, min = 0, max = 130 } })
local k = kit.maker()
k.mesh(a.kind == "dryer" and "hard" or "metal", mesh.translate(facing(mesh.lathe{ profile = { { 0.13, 0.0 }, { 0.215, 0.0 }, { 0.215, 0.025 }, { 0.2, 0.04 }, { 0.145, 0.04 }, { 0.13, 0.02 }, { 0.13, 0.0 } },
                  segments = 32, smooth = true }), { HX, 0, 0 }), a.kind == "dryer" and kit.BLACK * 1.5 or kit.CHROME)
k.mesh("glass", mesh.translate(facing(mesh.lathe{ profile = { { 0.0, -0.05 }, { 0.07, -0.045 }, { 0.12, -0.02 }, { 0.14, 0.02 } }, segments = 24, smooth = true }), { HX, 0, 0 }), V(0.75, 0.85, 0.9))
k.box("hard", V(2 * HX - 0.03, 0, 0.045), V(0.03, 0.09, 0.02), 0.008, kit.BLACK * 1.5, 1)   -- the handle
k.box("metal", V(0.01, 0, 0.01), V(0.03, 0.1, 0.03), 0.006, kit.CHROME, 1)                  -- the hinge
kit.put(m, door, k)
door:shape("door", { box = { 0.43, 0.43, 0.05 }, pos = { HX, 0, 0.02 } })
door:marker("handle", { pos = { 2 * HX - 0.03, 0, 0.06 }, role = "hold" })
return m
