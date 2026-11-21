-- A ROCKING CHAIR THAT ROCKS (#191): a wooden chair on two curved rockers. The whole chair is on a node, `rock`,
-- that turns 14 degrees either way about the middle of the circle its rockers are part of, which is how a chair
-- on rockers moves when it does not slide. It stands on y = 0 at its own origin, facing +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  wood    = kit.wood_names,
  cushion = { "none", "cushion" },
  fabric  = kit.fabric_names,
}
local wood = kit.wood(a.wood)
local m = model.new()
local far = m:level(25)
local RR = 1.1                                 -- the radius of the rockers' curve
local rock = m:node("rock", { pos = { 0, RR, 0 }, motion = { turn = { 1, 0, 0 }, min = -14, max = 14 } })
local k = kit.maker()
local function at(x, y, z) return V(x, y - RR, z) end                                -- a point by its height above the floor
for _, x in ipairs{ -0.27, 0.27 } do
  local pts = {}
  for i = -4, 4 do local t = math.rad(i * 8.5); pts[#pts + 1] = V(x, -RR * math.cos(t) + 0.02, RR * math.sin(t) - 0.03) end
  k.bent("wood", pts, 0.035, 0.04, wood, { count = 16 })                             -- the rocker
  k.post("wood", at(x, 0.04, 0.24), at(x, 0.62, 0.22), 0.036, 0.034, wood)           -- the front leg, up to the arm
  k.bent("wood", { at(x, 0.03, -0.25), at(x, 0.40, -0.24), at(x, 0.8, -0.32), at(x, 1.08, -0.38) }, 0.036, 0.036, wood, { count = 12 })   -- the back leg and post
  k.bent("wood", { at(x, 0.62, 0.26), at(x * 1.05, 0.63, 0.0), at(x, 0.60, -0.29) }, 0.05, 0.025, wood, { count = 8 })   -- the arm
end
k.box("wood", at(0, 0.40, 0.0), V(0.54, 0.03, 0.5), 0.012, wood, 2)                  -- the seat
for i = -2, 2 do k.bent("wood", { at(i * 0.09, 0.42, -0.24), at(i * 0.09, 0.8, -0.31), at(i * 0.09, 1.04, -0.37) }, 0.05, 0.012, wood, { count = 8 }) end   -- the slats
k.bent("wood", { at(-0.27, 1.06, -0.375), at(0, 1.1, -0.39), at(0.27, 1.06, -0.375) }, 0.03, 0.07, wood, { count = 8 })   -- the top rail
if a.cushion == "cushion" then k.cushion("fabric", at(0, 0.45, 0.0), V(0.5, 0.06, 0.46), kit.fabric(a.fabric), { r = 0.06 }) end
kit.put(m, rock, k)
kit.far_box(m, rock, "wood", { 0.58, 0.45, 0.6 }, { 0, 0.22 - RR, 0 }, wood, far)
kit.far_box(m, rock, "wood", { 0.56, 0.66, 0.04 }, { 0, 0.75 - RR, -0.32 }, wood, far)
rock:shape("seat", { box = { 0.58, 0.42, 0.6 }, pos = { 0, 0.21 - RR, 0 }, friction = 0.8 })
rock:shape("back", { box = { 0.56, 0.66, 0.05 }, pos = { 0, 0.75 - RR, -0.32 }, turn = { -12, 0, 0 } })
rock:marker("seat", { pos = { 0, 0.42 - RR, 0 }, role = "seat", size = 0.5 })
return m
