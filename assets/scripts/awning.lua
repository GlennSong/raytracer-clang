-- A SHOP'S AWNING THAT ROLLS OUT AND IN (#191): a case on the wall, a front bar, and striped cloth between them.
-- The cloth is ten strips, each on a node, `strip_1` (nearest the wall) ... `strip_10`, and the front bar is on
-- `bar`. Each slides out and down the awning's slope and stops at its own place; slide them all out at one speed
-- and the awning unrolls, strip after strip, as cloth does off a roller (awning_roll.lua does). As placed it is in.
-- It HANGS on a wall: its origin is the middle of the case's back, and it rolls out along +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  width  = { 3.0, min = 1.5, max = 6.0 },         -- along the wall (metres)
  reach  = { 2.2, min = 1.0, max = 3.5 },         -- how far out it rolls
  slope  = { 14, min = 5, max = 30 },             -- how steeply it falls (degrees)
  cloth  = { "green", "red", "blue", "yellow", "orange" },
}
local W, L, STRIPS = a.width, a.reach, 10
local cloth = kit.paint[a.cloth]
local m = model.new()
local far = m:level(60)
local CASE = L / STRIPS + 0.06                 -- the case is deep enough to hold a strip
local c = kit.maker()
c.box("paint", V(0, 0.07, CASE / 2), V(W + 0.06, 0.02, CASE), 0.004, kit.WHITE, 1)    -- the case's top
c.box("paint", V(0, 0, 0.01), V(W + 0.06, 0.16, 0.02), 0.004, kit.WHITE, 1)           -- and its back, on the wall
for _, x in ipairs{ -W / 2 - 0.02, W / 2 + 0.02 } do c.box("paint", V(x, 0, CASE / 2), V(0.02, 0.16, CASE), 0.004, kit.WHITE, 1) end
kit.put(m, m, c)
kit.far_box(m, m, "paint", { W, 0.16, CASE }, { 0, 0, CASE / 2 }, kit.WHITE, far)
m:shape("case", { box = { W + 0.06, 0.16, CASE }, pos = { 0, 0, CASE / 2 } })
m:marker("crank", { pos = { W / 2, -0.1, 0.05 }, role = "hold" })

-- everything that rolls out is in a space that slopes: +z runs down the cloth
local slope = m:node("slope", { pos = { 0, 0, 0.03 }, turn = { a.slope, 0, 0 } })
local each = L / STRIPS
local stripes = math.max(4, math.floor(W / 0.3 + 0.5))
for i = 1, STRIPS do
  local strip = slope:node("strip_" .. i, { pos = { 0, 0.03 - i * 0.0015, 0 }, motion = { slide = { 0, 0, 1 }, min = 0, max = (i - 1) * each } })
  local k = kit.maker()
  for s = 1, stripes do
    local x = -W / 2 + (s - 0.5) * W / stripes
    k.box("fabric", V(x, 0, each / 2 + 0.01), V(W / stripes, 0.003, each + 0.02), 0.0, s % 2 == 1 and cloth or kit.LINEN, 1)
  end
  kit.put(m, strip, k)
end
local bar = slope:node("bar", { pos = { 0, 0.01, 0 }, motion = { slide = { 0, 0, 1 }, min = 0, max = L } })
local b = kit.maker()
b.box("paint", V(0, 0, 0.02), V(W + 0.04, 0.05, 0.04), 0.01, kit.WHITE, 1)
for s = 1, stripes do                           -- the scalloped strip that hangs from the front bar
  local x = -W / 2 + (s - 0.5) * W / stripes
  b.box("fabric", V(x, -0.1, 0.03), V(W / stripes, 0.16, 0.004), 0.0, s % 2 == 1 and cloth or kit.LINEN, 1)
end
kit.put(m, bar, b)
kit.far_box(m, bar, "fabric", { W, 0.2, 0.05 }, { 0, -0.08, 0.02 }, cloth, far)
bar:shape("bar", { box = { W, 0.06, 0.05 }, pos = { 0, 0, 0.02 } })
bar:marker("edge", { pos = { 0, 0, 0.04 } })
return m
