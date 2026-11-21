-- A REVOLVING DOOR that turns (#191): a round glass drum with an opening front and back, and four glass wings on
-- a post in the middle, on a node, `wings`, that turns without end. A quarter turn passes one person. Markers:
-- `outside` and `inside` on the floor either side.
-- It stands on y = 0 at its own origin, the way through along z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  radius = { 1.1, min = 0.9, max = 1.6 },         -- the drum's radius (metres)
  height = { 2.3, min = 2.1, max = 2.8 },
  finish = { "steel", "black", "bronze" },
}
local R, H = a.radius, a.height
local col, part = kit.finish(a.finish)
local GLASS = V(0.8, 0.9, 0.92)
local m = model.new()
local far = m:level(60)

local c = kit.maker()
c.turned("metal", V(0, H, 0), { { 0.0, 0.0 }, { R + 0.06, 0.0 }, { R + 0.06, 0.18 }, { 0.0, 0.18 } }, col, 1, 40)     -- the canopy
c.turned("metal", V(0, 0, 0), { { 0.0, 0.0 }, { R + 0.03, 0.0 }, { R + 0.03, 0.012 }, { 0.0, 0.012 } }, col * 0.7, 1, 40)   -- the floor plate
-- the drum's two curved walls, each of flat panes between posts; the openings are 80 degrees wide
local PANES, OPEN = 5, math.rad(80)
for _, side in ipairs{ 0, math.pi } do
  local from, to = side + OPEN / 2 - math.pi / 2, side + math.pi - OPEN / 2 - math.pi / 2
  for i = 0, PANES do
    local t = from + (to - from) * i / PANES
    local p = V(R * math.cos(t), 0, R * math.sin(t))
    c.rod("metal", p, p + V(0, H, 0), 0.02, col, 8)
    if i < PANES then
      local t2 = from + (to - from) * (i + 1) / PANES
      local q = V(R * math.cos(t2), 0, R * math.sin(t2))
      local mid, len = (p + q) * 0.5, math.sqrt((q[1] - p[1]) ^ 2 + (q[3] - p[3]) ^ 2)
      local yaw = math.atan(-(q[3] - p[3]), q[1] - p[1])
      c.mesh("glass", mesh.translate(mesh.rotate_y(mesh.box{ len, H, 0.012 }, yaw), { mid[1], H / 2, mid[3] }), GLASS)
      m:shape(string.format("wall_%d_%d", side == 0 and 1 or 2, i + 1), { box = { len, H, 0.04 }, pos = { mid[1], H / 2, mid[3] }, turn = { 0, math.deg(yaw), 0 } })
    end
  end
end
kit.put(m, m, c)
kit.far_box(m, m, "metal", { 2 * R, 0.18, 2 * R }, { 0, H + 0.09, 0 }, col, far)
kit.far_box(m, m, "glass", { 2 * R, H, 2 * R }, { 0, H / 2, 0 }, GLASS, far)
m:marker("outside", { pos = { 0, 0, R + 0.8 }, turn = { 0, 180, 0 }, role = "stand" })
m:marker("inside", { pos = { 0, 0, -R - 0.8 }, role = "stand" })

local wings = m:node("wings", { pos = { 0, 0, 0 }, turn = { 0, 45, 0 }, motion = { turn = { 0, 1, 0 } } })
local k = kit.maker()
k.rod("metal", V(0, 0.01, 0), V(0, H, 0), 0.04, col, 12)
local WL = R - 0.06
for i = 0, 3 do
  local one = kit.maker()
  one.box("glass", V(WL / 2 + 0.03, H / 2, 0), V(WL - 0.04, H - 0.12, 0.012), 0.0, GLASS, 1)
  for _, y in ipairs{ 0.05, H - 0.05 } do one.box("metal", V(WL / 2 + 0.03, y, 0), V(WL, 0.06, 0.04), 0.006, col, 1) end
  one.box("metal", V(WL + 0.02, H / 2, 0), V(0.04, H - 0.04, 0.04), 0.006, col, 1)
  one.rod("metal", V(WL * 0.35, 1.05, 0.03), V(WL * 0.9, 1.05, 0.03), 0.014, kit.CHROME, 8)      -- the push bar
  for part, list in pairs(one.lists) do k.put(part, mesh.rotate_y(mesh.merge(list), i * math.pi / 2)) end
  local c_, s_ = math.cos(i * math.pi / 2), -math.sin(i * math.pi / 2)
  wings:shape("wing_" .. (i + 1), { box = { WL, H - 0.1, 0.04 }, pos = { c_ * (WL / 2 + 0.03), H / 2, s_ * (WL / 2 + 0.03) }, turn = { 0, i * 90, 0 } })
end
kit.put(m, wings, k)
return m
