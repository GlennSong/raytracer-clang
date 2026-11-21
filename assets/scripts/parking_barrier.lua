-- A PARKING BARRIER whose arm lifts (#191): a cabinet beside the lane and a striped arm across it, on a node,
-- `arm`, that swings up 90 degrees. A post on the far side catches the arm's end. Markers: `ticket` on the
-- cabinet's face (where a driver reaches), `lamp` on its top.
-- It stands on y = 0 at its own origin, which is under the cabinet; the arm reaches along +x across the lane.
local kit = require "model_kit"
local V = kit.V

local a = params{
  reach = { 3.2, min = 2.0, max = 6.0 },          -- the arm's length (metres)
  paint = { "yellow", "orange", "white", "blue" },
}
local L = a.reach
local col = a.paint == "white" and kit.WHITE or kit.paint[a.paint]
local RED, WHITE = V(0.8, 0.1, 0.08), V(0.95, 0.95, 0.93)
local m = model.new()
local far = m:level(60)
local PIVOT = 0.95

local c = kit.maker()
c.box("paint", V(0, 0.55, 0), V(0.36, 1.1, 0.36), 0.02, col, 2)
c.box("hard", V(0, 0.01, 0), V(0.46, 0.02, 0.46), 0.0, V(0.5, 0.5, 0.5), 1)           -- the pad it is bolted to
c.box("hard", V(0, 1.0, 0.182), V(0.2, 0.12, 0.006), 0.0, kit.BLACK, 1)               -- the ticket slot's panel
c.box("light", V(0, 1.02, 0.186), V(0.1, 0.02, 0.002), 0.0, V(0.3, 1, 0.4), 1)
c.turned("light", V(0, 1.1, 0), { { 0.0, 0.0 }, { 0.05, 0.0 }, { 0.05, 0.05 }, { 0.03, 0.08 }, { 0.0, 0.085 } }, V(1, 0.6, 0.1), 1, 12)
-- the far post, with a fork for the arm's end
c.rod("paint", V(L + 0.05, 0, 0), V(L + 0.05, PIVOT - 0.06, 0), 0.03, col, 10)
c.box("paint", V(L + 0.05, PIVOT - 0.07, 0), V(0.06, 0.02, 0.14), 0.004, col, 1)
for _, z in ipairs{ -0.06, 0.06 } do c.box("paint", V(L + 0.05, PIVOT, z), V(0.06, 0.14, 0.012), 0.004, col, 1) end
kit.put(m, m, c)
kit.far_box(m, m, "paint", { 0.36, 1.1, 0.36 }, { 0, 0.55, 0 }, col, far)
m:shape("cabinet", { box = { 0.36, 1.1, 0.36 }, pos = { 0, 0.55, 0 } })
m:shape("post", { capsule = { radius = 0.04, height = PIVOT }, pos = { L + 0.05, PIVOT / 2, 0 } })
m:marker("ticket", { pos = { 0, 1.0, 0.2 }, role = "press" })
m:marker("lamp", { pos = { 0, 1.2, 0 }, role = "lamp" })
m:marker("wait", { pos = { L / 2, 0, 2.5 }, turn = { 0, 180, 0 }, role = "stand" })

local arm = m:node("arm", { pos = { 0.1, PIVOT, 0.2 }, motion = { turn = { 0, 0, 1 }, min = 0, max = 90 } })
local k = kit.maker()
k.box("paint", V(-0.12, 0, 0), V(0.4, 0.14, 0.04), 0.01, col, 1)                      -- the counterweight end
local bands = math.max(4, math.floor(L / 0.4))
local each = (L - 0.15) / bands
for i = 1, bands do
  k.box("paint", V(0.08 + (i - 0.5) * each, 0, 0), V(each, 0.09, 0.03), 0.0, i % 2 == 1 and RED or WHITE, 1)
end
kit.put(m, arm, k)
kit.far_box(m, arm, "paint", { L, 0.09, 0.03 }, { L / 2, 0, 0 }, RED, far)
arm:shape("arm", { box = { L, 0.09, 0.04 }, pos = { L / 2, 0, 0 } })
arm:marker("tip", { pos = { L - 0.1, 0, 0 } })
return m
