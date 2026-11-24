-- A STOOL THAT TURNS: at a bar, a diner's counter, a laboratory bench (#191). Its seat is on a node, `swivel`,
-- that turns without end, on a column that slides (`lift`) when the stool has a gas lift.
--
--   bar     a tall stool on a round foot with a ring to rest the feet on, and a low back
--   diner   a chrome post fixed to the floor, a round padded seat
--   lab     a low-backed stool on five casters, with a gas lift and a foot ring
--
-- It stands on y = 0 at its own origin, facing +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind   = { "bar", "diner", "lab" },
  fabric = kit.fabric_names,
}
local fabric, cloth = kit.upholstery(a.fabric)
local m = model.new()
local far = m:level(25)
local SEAT = ({ bar = 0.76, diner = 0.62, lab = 0.56 })[a.kind]
local c = kit.maker()
if a.kind == "lab" then
  for i = 0, 4 do
    local t = 2 * math.pi * i / 5
    c.bent("hard", { V(0, 0.09, 0), V(0.14 * math.cos(t), 0.085, 0.14 * math.sin(t)), V(0.27 * math.cos(t), 0.065, 0.27 * math.sin(t)) }, 0.028, 0.024, kit.BLACK, { count = 6 })
    kit.caster(c, V(0.27 * math.cos(t), 0, 0.27 * math.sin(t)), 0.026)
  end
  c.turned("hard", V(0, 0, 0), { { 0.04, 0.07 }, { 0.04, 0.13 }, { 0.03, 0.14 }, { 0.03, 0.34 }, { 0, 0.34 } }, kit.BLACK, 1, 14)
else
  local r = a.kind == "bar" and 0.21 or 0.13
  c.turned("metal", V(0, 0, 0), { { 0, 0 }, { r, 0 }, { r, 0.012 }, { 0.05, 0.035 }, { 0.03, 0.05 }, { 0.03, SEAT - 0.12 }, { 0, SEAT - 0.12 } }, kit.CHROME, 1, 20)
end
kit.put(m, m, c)
m:shape("foot", { box = { 0.44, 0.08, 0.44 }, pos = { 0, 0.04, 0 } })
m:shape("post", { capsule = { radius = 0.035, height = SEAT - 0.15 }, pos = { 0, SEAT / 2, 0 } })
local travel = a.kind == "lab" and 0.14 or 0.0
local lift = m:node("lift", { pos = { 0, 0, 0 }, motion = travel > 0 and { slide = { 0, 1, 0 }, min = 0, max = travel } or nil })
local l = kit.maker()
l.turned("metal", V(0, 0, 0), { { 0.022, 0.15 }, { 0.022, SEAT - 0.06 }, { 0, SEAT - 0.06 } }, kit.CHROME, 1, 12)
if a.kind ~= "diner" then                        -- the ring to rest the feet on
  local ring, rr, ry = {}, 0.19, a.kind == "bar" and 0.3 or 0.22
  for i = 0, 23 do local t = 2 * math.pi * i / 24; ring[#ring + 1] = { rr * math.cos(t), ry, rr * math.sin(t) } end
  l.pipe("metal", ring, 0.009, kit.CHROME, { closed = true, sides = 6 })
  for i = 0, 2 do local t = 2 * math.pi * i / 3; l.rod("metal", V(0, ry, 0), V(rr * math.cos(t), ry, rr * math.sin(t)), 0.006, kit.CHROME, 6) end
end
kit.put(m, lift, l)
local swivel = lift:node("swivel", { pos = { 0, SEAT - 0.06, 0 }, motion = { turn = { 0, 1, 0 } } })
local s = kit.maker()
s.turned("hard", V(0, 0, 0), { { 0, 0 }, { 0.15, 0 }, { 0.17, 0.02 }, { 0, 0.02 } }, kit.BLACK, 1, 20)
s.turned(cloth, V(0, 0.02, 0), { { 0, 0 }, { 0.175, 0 }, { 0.185, 0.02 }, { 0.17, 0.045 }, { 0.1, 0.055 }, { 0, 0.058 } }, fabric, 1, 24)
if a.kind ~= "diner" then
  s.bent("hard", { V(0, 0.0, -0.14), V(0, 0.08, -0.2), V(0, 0.2, -0.2) }, 0.04, 0.014, kit.BLACK, { count = 6 })
  s.cushion(cloth, V(0, 0.26, -0.19), V(0.3, 0.14, 0.04), fabric, { face = "z", r = 0.04, bow = 0.03 })
end
kit.put(m, swivel, s)
kit.far_box(m, swivel, cloth, { 0.34, 0.07, 0.34 }, { 0, 0.035, 0 }, fabric, far)
swivel:shape("seat", { box = { 0.34, 0.07, 0.34 }, pos = { 0, 0.035, 0 }, friction = 0.9 })
swivel:marker("seat", { pos = { 0, 0.078, 0 }, role = "seat", size = 0.34 })
return m
