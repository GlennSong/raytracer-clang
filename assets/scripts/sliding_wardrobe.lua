-- A WARDROBE WITH SLIDING DOORS (#191): a wide cupboard with a hanging rail and shelves, and two doors that pass
-- each other on two tracks (`door_1` on the front track, at the left; `door_2` behind it, at the right; each
-- slides across to cover the other's half). One door is mirrored.
-- It stands on y = 0 at its own origin, its doors toward +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  width  = { 1.8, min = 1.2, max = 3.0 },         -- metres
  height = { 2.2, min = 1.8, max = 2.6 },
  wood   = kit.wood_names,
  mirror = { "one", "both", "none" },
}
local W, H, D, T = a.width, a.height, 0.62, 0.025
local wood = kit.wood(a.wood)
local m = model.new()
local far = m:level(40)
local c = kit.maker()
kit.carcass(c, "wood", W, H, D, T, wood)
c.box("wood", V(0, H / 2, -0.03), V(T, H - 2 * T, D - 0.1), 0.0, wood, 1)                 -- the division down the middle
c.rod("metal", V(-W / 2 + T, H - 0.3, -0.03), V(0, H - 0.3, -0.03), 0.012, kit.CHROME, 8) -- the rail, on the left
for i = 1, 4 do c.box("wood", V(W / 4, H * i / 5, -0.03), V(W / 2 - T, 0.02, D - 0.12), 0.0, wood, 1) end
-- what hangs on the rail, and what is folded on the shelves
local n = math.floor((W / 2 - 0.15) / 0.07)
for i = 1, n do c.box("fabric", V(-W / 2 + 0.08 + i * 0.07, H - 0.85, -0.03), V(0.035, 1.0, 0.42), 0.015, kit.FABRIC[i % 8 + 1], 1) end
for i = 1, 4 do for j = 0, 1 do c.box("fabric", V(W / 4 - 0.18 + j * 0.36, H * i / 5 + 0.07, -0.03), V(0.3, 0.1, 0.36), 0.03, kit.FABRIC[(i * 2 + j) % 8 + 1], 2) end end
kit.put(m, m, c)
kit.far_box(m, m, "wood", { W, H, D }, { 0, H / 2, 0 }, wood, far)
m:shape("back", { box = { W, H, T }, pos = { 0, H / 2, -D / 2 + T / 2 } })
for i, x in ipairs{ -W / 2 + T / 2, W / 2 - T / 2 } do m:shape("side_" .. i, { box = { T, H, D }, pos = { x, H / 2, 0 } }) end
m:shape("top", { box = { W, T, D }, pos = { 0, H - T / 2, 0 } })

local w, h = W / 2 - T + 0.03, H - 2 * T - 0.01
for i, side in ipairs{ -1, 1 } do
  local z = D / 2 - 0.02 - (i - 1) * 0.035
  local door = m:node("door_" .. i, { pos = { side * (W / 2 - T), T + 0.005, z }, motion = { slide = { -side, 0, 0 }, min = 0, max = w - 0.06 } })
  local k = kit.maker()
  local cx = -side * w / 2
  local mirrored = a.mirror == "both" or (a.mirror == "one" and i == 1)
  k.box("wood", V(cx, h / 2, 0), V(w, h, 0.022), 0.004, wood * 0.92, 1)
  if mirrored then k.box("metal", V(cx, h / 2, 0.0115), V(w - 0.08, h - 0.08, 0.001), 0.0, V(0.86, 0.9, 0.92), 1) end
  k.box("metal", V(-side * (w - 0.03), h / 2, 0.014), V(0.02, 0.3, 0.006), 0.003, kit.CHROME, 1)   -- the pull, at its meeting edge
  kit.put(m, door, k)
  door:shape("door_" .. i, { box = { w, h, 0.022 }, pos = { cx, h / 2, 0 } })
  door:marker("pull_" .. i, { pos = { -side * (w - 0.03), h / 2, 0.03 }, role = "hold" })
end
m:marker("front", { pos = { 0, 0, D / 2 + 0.8 }, turn = { 0, 180, 0 }, role = "stand" })
return m
