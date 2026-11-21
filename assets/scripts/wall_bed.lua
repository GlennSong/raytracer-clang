-- A BED THAT FOLDS UP INTO THE WALL (#191): a tall cabinet, and a bed that stands on end inside it and comes down
-- on a pivot near the floor (`bed`, 0 up to 90 down). A leg frame at its far end folds out to stand on (`legs`, on
-- the bed, 0 flat against it to 90 out). As placed it is folded up, and its underside is the cabinet's front.
-- Lower the bed before the legs touch the floor: send both together and they arrive together.
-- It stands on y = 0 at its own origin, its back to a wall, opening toward +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  width  = { 1.4, min = 0.9, max = 1.8 },         -- the mattress's width (metres)
  wood   = kit.wood_names,
  fabric = kit.fabric_names,
}
local W, wood, cloth = a.width, kit.wood(a.wood), kit.fabric(a.fabric)
local L, PIVOT, D, T = 1.98, 0.36, 0.45, 0.03
local H = PIVOT + L + 0.06
local m = model.new()
local far = m:level(40)
local c = kit.maker()
kit.carcass(c, "wood", W + 0.1, H, D, T, wood)
kit.put(m, m, c)
kit.far_box(m, m, "wood", { W + 0.1, H, D }, { 0, H / 2, 0 }, wood, far)
for i, x in ipairs{ -W / 2 - 0.035, W / 2 + 0.035 } do m:shape("side_" .. i, { box = { T, H, D }, pos = { x, H / 2, 0 } }) end
m:shape("back", { box = { W + 0.1, H, T }, pos = { 0, H / 2, -D / 2 + T / 2 } })
m:shape("top", { box = { W + 0.1, T, D }, pos = { 0, H - T / 2, 0 } })

-- the bed, in its own space: the pivot is its origin, its length runs up y and its underside faces +z
local bed = m:node("bed", { pos = { 0, PIVOT, D / 2 - 0.03 }, motion = { turn = { 1, 0, 0 }, min = 0, max = 90 } })
local k = kit.maker()
k.box("wood", V(0, L / 2 - 0.15, 0.0), V(W + 0.03, L + 0.3, 0.03), 0.006, wood, 1)             -- the base, and the cabinet's front
for _, x in ipairs{ -W / 2, W / 2 } do k.box("wood", V(x, L / 2, -0.08), V(0.025, L, 0.14), 0.006, wood, 1) end
k.box("wood", V(0, L - 0.012, -0.08), V(W, 0.025, 0.14), 0.006, wood, 1)
k.cushion("fabric", V(0, L / 2 - 0.02, -0.13), V(W - 0.04, L - 0.06, 0.2), kit.LINEN, { r = 0.06, face = "z" })   -- the mattress
k.cushion("fabric", V(0, L * 0.62, -0.25), V(W - 0.02, L * 0.66, 0.05), cloth, { r = 0.05, face = "z", crown = 0.015 })   -- the cover
k.cushion("fabric", V(0, 0.22, -0.27), V(W * 0.6, 0.36, 0.1), kit.WHITE, { r = 0.08, face = "z", pinch = 0.5 })   -- the pillow, at the wall end
kit.put(m, bed, k)
bed:shape("bed", { box = { W, L, 0.25 }, pos = { 0, L / 2, -0.11 }, friction = 0.8 })
bed:marker("lie", { pos = { 0, L / 2, -0.24 }, turn = { -90, 0, 0 }, role = "lie", size = W })
bed:marker("pull", { pos = { 0, L - 0.2, 0.03 }, role = "hold" })

local legs = bed:node("legs", { pos = { 0, L - 0.08, 0.02 }, motion = { turn = { -1, 0, 0 }, min = 0, max = 90 } })
local g = kit.maker()
local drop = PIVOT - 0.035
g.pipe("metal", { { -W / 2 + 0.1, 0, 0.012 }, { -W / 2 + 0.1, -drop, 0.012 }, { W / 2 - 0.1, -drop, 0.012 }, { W / 2 - 0.1, 0, 0.012 } }, 0.012, kit.STEEL, { bend = 0.05, sides = 8 })
kit.put(m, legs, g)
return m
