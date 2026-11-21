-- A ROUND DINING TABLE WITH A TURNING MIDDLE (#191; a restaurant's big table, the kind a party shares dishes
-- from). The disc in the middle is on a node, `turntable`, that turns without end, with dishes on it. A marker
-- `seat_N` on the floor round it for each chair, and `dish_N` on the turning part.
-- It stands on y = 0 at its own origin.
local kit = require "model_kit"
local V = kit.V

local a = params{
  size  = { 1.6, min = 1.0, max = 2.4 },          -- across the table (metres)
  seats = { 8, min = 4, max = 12, step = 1 },
  wood  = kit.wood_names,
  cloth = { "none", "white", "red" },
}
local R, N, wood = a.size / 2, math.floor(a.seats + 0.5), kit.wood(a.wood)
local m = model.new()
local far = m:level(40)
local TOP = 0.74
local c = kit.maker()
c.turned("wood", V(0, 0, 0), { { 0, 0 }, { 0.36, 0 }, { 0.36, 0.03 }, { 0.09, 0.07 }, { 0.07, 0.1 }, { 0.07, TOP - 0.08 }, { 0.2, TOP - 0.03 }, { 0, TOP - 0.03 } }, wood * 0.85, 1, 24)
if a.cloth == "none" then
  c.turned("wood", V(0, 0, 0), { { 0, TOP - 0.03 }, { R, TOP - 0.03 }, { R, TOP - 0.005 }, { R - 0.01, TOP }, { 0, TOP } }, wood, 1, 48)
else
  local cloth = a.cloth == "white" and kit.LINEN or kit.paint.red
  c.turned("fabric", V(0, 0, 0), { { R + 0.03, TOP - 0.3 }, { R + 0.012, TOP - 0.02 }, { R, TOP + 0.002 }, { 0, TOP + 0.002 } }, cloth, 1, 48)
end
kit.put(m, m, c)
kit.far_box(m, m, "wood", { 2 * R * 0.85, 0.03, 2 * R * 0.85 }, { 0, TOP - 0.015, 0 }, wood, far)
m:shape("top", { hull = mesh.lathe{ profile = { { 0, TOP - 0.03 }, { R, TOP - 0.03 }, { R, TOP }, { 0, TOP } }, segments = 12 }, friction = 0.8 })
m:shape("foot", { capsule = { radius = 0.08, height = TOP - 0.2 }, pos = { 0, TOP / 2, 0 } })
for i = 1, N do
  local t = 2 * math.pi * (i - 1) / N
  m:marker("seat_" .. i, { pos = { (R + 0.35) * math.sin(t), 0, (R + 0.35) * math.cos(t) }, turn = { 0, math.deg(t) + 180, 0 }, role = "stand" })
  m:marker("place_" .. i, { pos = { (R - 0.2) * math.sin(t), TOP + 0.003, (R - 0.2) * math.cos(t) } })
end
local TR = R * 0.55
local turn = m:node("turntable", { pos = { 0, TOP + 0.003, 0 }, motion = { turn = { 0, 1, 0 } } })
local k = kit.maker()
k.turned("glass", V(0, 0, 0), { { 0, 0.02 }, { TR, 0.02 }, { TR, 0.03 }, { 0, 0.03 } }, V(0.85, 0.92, 0.92), 1, 40)
k.turned("metal", V(0, 0, 0), { { 0, 0 }, { 0.12, 0 }, { 0.12, 0.02 }, { 0, 0.02 } }, kit.CHROME, 1, 20)
local DISH = { { 0, 0 }, { 0.07, 0 }, { 0.11, 0.03 }, { 0.105, 0.032 }, { 0.065, 0.008 }, { 0, 0.008 } }
local foods = { V(0.75, 0.3, 0.15), V(0.3, 0.55, 0.2), V(0.9, 0.8, 0.5), V(0.55, 0.3, 0.2), V(0.9, 0.85, 0.8) }
for i = 1, 5 do
  local t = 2 * math.pi * i / 5
  local at = V(TR * 0.62 * math.cos(t), 0.03, TR * 0.62 * math.sin(t))
  k.turned("ceramic", at, DISH, kit.PORCELAIN, 1, 16)
  k.turned("hard", at, { { 0, 0.008 }, { 0.06, 0.01 }, { 0.04, 0.035 }, { 0, 0.04 } }, foods[i], 1, 10)
  turn:marker("dish_" .. i, { pos = { at[1], 0.06, at[3] } })
end
k.turned("ceramic", V(0, 0.03, 0), { { 0, 0 }, { 0.04, 0 }, { 0.055, 0.05 }, { 0.03, 0.1 }, { 0.012, 0.12 }, { 0, 0.12 } }, kit.PORCELAIN, 1, 14)   -- the teapot
kit.put(m, turn, k)
return m
