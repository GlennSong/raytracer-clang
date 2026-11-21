-- A ROLLER SHUTTER that goes up and down (#191): a shop front's or a garage's steel curtain, between two guides,
-- rolling into a box above the opening. The curtain is in sections, each on a node, `section_1` (the lowest) ...,
-- and each slides up until it is inside the box. Slide them all up at one speed and the curtain rises as one
-- piece, each section stopping as it reaches the box (roller_shutter_run.lua does). As placed it is down.
-- It stands on y = 0 at its own origin, in the plane of the opening, its outside toward +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  width    = { 2.6, min = 0.9, max = 6.0 },       -- the opening's width (metres)
  height   = { 2.4, min = 1.8, max = 4.5 },       -- and its height
  sections = { 8, min = 4, max = 12, step = 1 },
  finish   = { "steel", "white", "green", "blue" },
}
local W, H, N = a.width, a.height, math.floor(a.sections + 0.5)
local col, part = kit.finish(a.finish)
local m = model.new()
local far = m:level(80)
local each = H / N
local BOX = each + 0.08                        -- the box holds one section's height

local c = kit.maker()
for _, x in ipairs{ -W / 2 - 0.03, W / 2 + 0.03 } do c.box("metal", V(x, H / 2, 0), V(0.06, H, 0.07), 0.004, col * 0.8, 1) end   -- the guides
c.box("metal", V(0, H + BOX / 2, 0.02), V(W + 0.16, BOX, 0.22), 0.01, col * 0.8, 1)                                       -- the box
kit.put(m, m, c)
kit.far_box(m, m, "metal", { W + 0.16, BOX, 0.22 }, { 0, H + BOX / 2, 0.02 }, col, far)
m:shape("box", { box = { W + 0.16, BOX, 0.22 }, pos = { 0, H + BOX / 2, 0.02 } })
for _, x in ipairs{ -W / 2 - 0.03, W / 2 + 0.03 } do m:shape("guide_" .. (x < 0 and "left" or "right"), { box = { 0.06, H, 0.07 }, pos = { x, H / 2, 0 } }) end
m:marker("switch", { pos = { W / 2 + 0.25, 1.2, 0.03 }, role = "press" })
m:marker("lock", { pos = { 0, 0.1, 0.03 }, role = "hold" })

local SLATS = 4                                -- the ribs drawn on each section
for i = 1, N do
  local sec = m:node("section_" .. i, { pos = { 0, (i - 1) * each, 0 }, motion = { slide = { 0, 1, 0 }, min = 0, max = H - (i - 1) * each + 0.03 } })
  local k = kit.maker()
  k.box("metal", V(0, each / 2, 0), V(W, each, 0.012), 0.0, col, 1)
  for s = 1, SLATS do k.box("metal", V(0, (s - 0.5) * each / SLATS, 0.008), V(W, each / SLATS * 0.55, 0.012), 0.004, col * 1.08, 1) end
  if i == 1 then k.box("metal", V(0, 0.03, 0), V(W, 0.06, 0.04), 0.006, col * 0.7, 1) end        -- the bottom bar
  kit.put(m, sec, k)
  kit.far_box(m, sec, "metal", { W, each, 0.02 }, { 0, each / 2, 0 }, col, far)
  sec:shape("section_" .. i, { box = { W, each, 0.03 }, pos = { 0, each / 2, 0 } })
end
return m
