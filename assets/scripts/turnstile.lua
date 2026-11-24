-- A TURNSTILE whose arms turn (#191): a waist-high cabinet with three arms on a hub that leans over the way
-- through, so that one arm always bars it. The hub is on a node, `arms`, that turns without end; a third of a turn
-- lets one person through. Markers: `reader` on the cabinet's top, `enter` and `leave` on the floor.
-- It stands on y = 0 at its own origin, under the cabinet; the way through runs along z on the +x side.
local kit = require "model_kit"
local V = kit.V

local a = params{
  finish = { "steel", "black", "blue" },
  way    = { 0.55, min = 0.45, max = 0.8 },       -- an arm's reach across the way through (metres)
}
local L = a.way
local col, part = kit.finish(a.finish)
local m = model.new()
local far = m:level(40)
local c = kit.maker()
c.box(part, V(0, 0.5, 0), V(0.26, 1.0, 1.0), 0.03, col, 2)
c.box("hard", V(0, 1.003, 0.3), V(0.14, 0.006, 0.2), 0.0, kit.BLACK, 1)               -- the card reader
c.box("light", V(0, 1.007, 0.37), V(0.06, 0.002, 0.03), 0.0, V(0.3, 1, 0.4), 1)
-- the rail on the far side of the way through
c.pipe(part, { { L + 0.3, 0, -0.45 }, { L + 0.3, 0.95, -0.45 }, { L + 0.3, 0.95, 0.45 }, { L + 0.3, 0, 0.45 } }, 0.025, col, { bend = 0.1 })
kit.put(m, m, c)
kit.far_box(m, m, part, { 0.26, 1.0, 1.0 }, { 0, 0.5, 0 }, col, far)
m:shape("cabinet", { box = { 0.26, 1.0, 1.0 }, pos = { 0, 0.5, 0 } })
m:shape("rail", { box = { 0.05, 0.95, 0.95 }, pos = { L + 0.3, 0.475, 0 } })
m:marker("reader", { pos = { 0, 1.01, 0.3 }, role = "press" })
m:marker("enter", { pos = { L / 2 + 0.15, 0, 0.8 }, turn = { 0, 180, 0 }, role = "stand" })
m:marker("leave", { pos = { L / 2 + 0.15, 0, -0.8 }, role = "stand" })

-- the hub leans 45 degrees toward the way through; each arm leans 45 degrees from the hub's axis, so the arm that
-- is round at the front lies level across the way
local arms = m:node("arms", { pos = { 0.13, 0.86, 0 }, turn = { 0, 0, -45 }, motion = { turn = { 0, 1, 0 } } })
local k = kit.maker()
k.turned(part, V(0, -0.02, 0), { { 0.0, 0.0 }, { 0.07, 0.0 }, { 0.07, 0.06 }, { 0.04, 0.1 }, { 0.0, 0.1 } }, col, 1, 16)
local s = math.sqrt(0.5)
for i = 0, 2 do
  local t = 2 * math.pi * i / 3
  local dir = V(s * math.cos(t), s, s * math.sin(t))
  k.rod("metal", dir * 0.05, dir * (L + 0.1), 0.019, kit.CHROME, 10)
  arms:shape("arm_" .. (i + 1), { sphere = 0.04, pos = { dir[1] * L * 0.6, dir[2] * L * 0.6, dir[3] * L * 0.6 } })
end
kit.put(m, arms, k)
return m
