-- A DESK THAT GOES UP AND DOWN: a top on two telescoping legs, from sitting height (0.72 m) to standing (1.32 m)
-- (#191; Glenn, 2026-11-20: "an adjustable desk that could go up or down").
--
-- Each leg is three tubes, one inside the next. The middle tubes are on the node `stage` and the inner tubes and
-- the top on the node `top`, a child of it; each slides 0.3 m, so the two together lift the top 0.6 m. Move both
-- by the same amount and the legs open evenly (standing_desk_lift.lua does). The control is a marker, `control`,
-- under the front edge, for the press that will raise and lower it when prefabs carry interactions.
-- It stands on y = 0 at its own origin; the person sits or stands on the +z side.
local kit = require "model_kit"
local V = kit.V

local a = params{
  width = { 1.6, min = 1.0, max = 2.2 },          -- the top's width (metres)
  depth = { 0.75, min = 0.6, max = 0.9 },         -- the top's depth
  wood  = kit.wood_names,                         -- the top
  frame = { "white", "black", "steel" },          -- the legs
}
local W, D, wood = a.width, a.depth, kit.wood(a.wood)
local frame = ({ white = kit.WHITE, black = kit.BLACK * 1.6, steel = kit.STEEL })[a.frame]

local m = model.new()
local far = m:level(30)
local LEG_X = W / 2 - 0.18                   -- where each leg stands
local STEP = 0.3                             -- how far each of the two stages slides

-- THE FEET and the outer tubes
local foot = kit.maker()
for _, x in ipairs{ -LEG_X, LEG_X } do
  foot.box("paint", V(x, 0.02, 0), V(0.08, 0.035, D - 0.08), 0.01, frame, 1)
  foot.box("paint", V(x, 0.31, 0), V(0.09, 0.55, 0.06), 0.008, frame, 1)
  m:shape("leg_" .. (x < 0 and "left" or "right"), { box = { 0.09, 0.6, 0.06 }, pos = { x, 0.3, 0 } })
end
kit.put(m, m, foot)
kit.far_box(m, m, "paint", { 0.09, 0.6, 0.06 }, { -LEG_X, 0.3, 0 }, frame, far)
kit.far_box(m, m, "paint", { 0.09, 0.6, 0.06 }, { LEG_X, 0.3, 0 }, frame, far)

-- THE MIDDLE TUBES
local stage = m:node("stage", { pos = { 0, 0, 0 }, motion = { slide = { 0, 1, 0 }, min = 0, max = STEP } })
local mid = kit.maker()
for _, x in ipairs{ -LEG_X, LEG_X } do mid.box("paint", V(x, 0.36, 0), V(0.075, 0.52, 0.046), 0.006, frame, 1) end
kit.put(m, stage, mid)

-- THE INNER TUBES, the frame under the top, and the top
local top = stage:node("top", { pos = { 0, 0, 0 }, motion = { slide = { 0, 1, 0 }, min = 0, max = STEP } })
local t = kit.maker()
for _, x in ipairs{ -LEG_X, LEG_X } do
  t.box("paint", V(x, 0.40, 0), V(0.06, 0.54, 0.034), 0.005, frame, 1)
  t.box("paint", V(x, 0.68, 0), V(0.06, 0.02, D - 0.14), 0.005, frame, 1)             -- the bracket the top is screwed to
end
t.box("paint", V(0, 0.665, 0), V(2 * LEG_X, 0.03, 0.05), 0.005, frame, 1)             -- the rail between the legs
t.box("wood", V(0, 0.705, 0), V(W, 0.03, D), 0.008, wood, 1)
-- the control: a small panel under the front edge, right of the middle, with two buttons
local CX = W / 2 - 0.3
t.box("hard", V(CX, 0.682, D / 2 - 0.02), V(0.12, 0.016, 0.04), 0.004, kit.BLACK, 1)
for _, dx in ipairs{ -0.025, 0.025 } do t.box("light", V(CX + dx, 0.682, D / 2 + 0.001), V(0.018, 0.008, 0.003), 0.0, V(0.9, 0.9, 0.9), 1) end
kit.put(m, top, t)
kit.far_box(m, top, "wood", { W, 0.03, D }, { 0, 0.705, 0 }, wood, far)
top:shape("top", { box = { W, 0.03, D }, pos = { 0, 0.705, 0 }, friction = 0.8 })
top:marker("control", { pos = { CX, 0.682, D / 2 }, role = "press" })
top:marker("surface", { pos = { 0, 0.72, 0 } })
-- SLOTS: where a thing someone carries can be put down on it, one each side of where the screen stands. A slot is
-- a marker whose role is "slot"; a thing put on one rides it, so it rises with the top (systems/carry_system.cpp).
for _, side in ipairs{ { "left", -1 }, { "right", 1 } } do
  top:marker("slot_" .. side[1], { pos = { side[2] * (W / 2 - 0.3), 0.72, 0.05 }, role = "slot", size = 0.4 })
end
m:marker("work", { pos = { 0, 0, D / 2 + 0.45 }, turn = { 0, 180, 0 }, role = "stand" })
return m
