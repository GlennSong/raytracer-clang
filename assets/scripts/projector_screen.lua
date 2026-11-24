-- A PROJECTOR SCREEN THAT ROLLS DOWN (#191): a case on the ceiling or high on a wall, and a white screen that
-- unrolls from it. The screen is ten strips, each on a node, `strip_1` (the last out, nearest the case) ...
-- `strip_10`, with a weighted bar on `bar`; each drops and stops at its own place, so sending them all down at
-- one speed unrolls the screen (projector_screen_roll.lua does). As placed it is rolled up.
-- It HANGS: its origin is the middle of the case's top, and the screen faces +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  width = { 2.4, min = 1.2, max = 5.0 },          -- metres
  drop  = { 1.6, min = 0.9, max = 3.0 },          -- how far the screen comes down
}
local W, L, STRIPS = a.width, a.drop, 10
local m = model.new()
local far = m:level(60)
local each = L / STRIPS
local CASE = each + 0.05
local c = kit.maker()
c.box("paint", V(0, -CASE / 2, 0), V(W + 0.12, CASE, 0.11), 0.012, kit.WHITE, 1)
kit.put(m, m, c)
m:shape("case", { box = { W + 0.12, CASE, 0.11 }, pos = { 0, -CASE / 2, 0 } })
m:marker("cord", { pos = { W / 2 + 0.03, -CASE - 0.3, 0.02 }, role = "hold" })
for i = 1, STRIPS do
  local strip = m:node("strip_" .. i, { pos = { 0, -CASE + 0.04, 0.02 + i * 0.0006 }, motion = { slide = { 0, -1, 0 }, min = 0, max = (i - 1) * each + 0.03 } })
  local k = kit.maker()
  k.box("hard", V(0, -each / 2, 0), V(W, each + 0.02, 0.002), 0.0, V(0.97, 0.97, 0.96), 1)
  kit.put(m, strip, k)
end
local bar = m:node("bar", { pos = { 0, -CASE + 0.04, 0.026 }, motion = { slide = { 0, -1, 0 }, min = 0, max = L + 0.03 } })
local b = kit.maker()
b.box("hard", V(0, -0.012, 0), V(W + 0.02, 0.024, 0.018), 0.006, kit.BLACK * 1.5, 1)
b.rod("hard", V(0, -0.02, 0), V(0, -0.12, 0), 0.002, kit.BLACK, 4)
kit.put(m, bar, b)
kit.far_box(m, bar, "hard", { W, 0.03, 0.02 }, { 0, -0.012, 0 }, kit.BLACK, far)
bar:marker("pull", { pos = { 0, -0.12, 0 }, role = "hold" })
m:marker("picture", { pos = { 0, -CASE - L / 2, 0.03 }, role = "screen", size = L })
return m
