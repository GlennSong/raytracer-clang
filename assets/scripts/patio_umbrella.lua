-- A PATIO UMBRELLA THAT OPENS AND CLOSES, with a round table under it when asked (#191; a café's terrace). Each of
-- its eight ribs is on a node, `rib_1` ... `rib_8`, hinged at the top of the pole, and carries its own panel of
-- cloth; turn them all down by the same amount (up to 68 degrees) and the umbrella folds. As placed it is open.
-- The crank is a marker, `crank`. It stands on y = 0 at its own origin.
local kit = require "model_kit"
local V = kit.V

local a = params{
  span   = { 2.6, min = 1.6, max = 4.0 },         -- across the open umbrella (metres)
  cloth  = { "red", "blue", "yellow", "green", "orange", "white" },
  second = { "white", "red", "blue", "yellow", "green", "orange" },   -- every other panel
  table  = { "table", "none" },
}
local RIBS, R = 8, a.span / 2
local function cloth(name) return name == "white" and kit.LINEN or kit.paint[name] end
local m = model.new()
local far = m:level(50)
local TOP = 2.35                               -- the height of the hub the ribs hang from
local DROOP = math.rad(14)                     -- how far below level an open rib runs

local s = kit.maker()
s.turned("metal", V(0, 0, 0), { { 0.0, 0.0 }, { 0.26, 0.0 }, { 0.26, 0.03 }, { 0.05, 0.07 }, { 0.022, 0.09 }, { 0.022, TOP + 0.04 }, { 0.0, TOP + 0.06 } }, V(0.75, 0.76, 0.78), 1, 18)
s.box("hard", V(0, 1.15, 0.035), V(0.05, 0.08, 0.05), 0.01, kit.BLACK * 1.5, 1)       -- the crank's housing
s.rod("metal", V(0, 1.15, 0.06), V(0.07, 1.15, 0.1), 0.006, kit.CHROME, 6)
if a.table == "table" then
  s.turned("hard", V(0, 0, 0), { { 0.03, 0.70 }, { 0.5, 0.70 }, { 0.5, 0.73 }, { 0.03, 0.73 } }, kit.WHITE, 1, 32)
  s.turned("hard", V(0, 0, 0), { { 0.5, 0.73 }, { 0.03, 0.73 } }, kit.WHITE, 1, 32)
  m:shape("table", { box = { 0.72, 0.03, 0.72 }, pos = { 0, 0.715, 0 } })
  for i = 1, 4 do
    local t = math.pi / 4 + (i - 1) * math.pi / 2
    m:marker("chair_" .. i, { pos = { 0.85 * math.cos(t), 0, 0.85 * math.sin(t) }, turn = { 0, 270 - math.deg(t), 0 }, role = "stand" })
  end
end
kit.put(m, m, s)
m:shape("pole", { capsule = { radius = 0.03, height = TOP }, pos = { 0, TOP / 2, 0 } })
m:shape("foot", { box = { 0.5, 0.06, 0.5 }, pos = { 0, 0.03, 0 } })
m:marker("crank", { pos = { 0.07, 1.15, 0.1 }, role = "hold" })
m:marker("shade", { pos = { 0, 0, 0 } })

-- a rib's space: the hub is its origin and the rib runs along +x, drooping; its panel reaches half way to the
-- next rib on each side
local half = math.tan(math.pi / RIBS)
for i = 1, RIBS do
  local turn = (i - 1) * 360 / RIBS
  local rib = m:node("rib_" .. i, { pos = { 0, TOP, 0 }, turn = { 0, turn, 0 }, motion = { turn = { 0, 0, -1 }, min = 0, max = 68 } })
  local k = kit.maker()
  local c_, s_ = math.cos(DROOP), math.sin(DROOP)
  k.rod("metal", V(0.02, 0, 0), V(R * c_, -R * s_, 0), 0.007, V(0.75, 0.76, 0.78), 6)
  local panel = mesh.rounded_box({ R, 0.004, 2 * R * half }, 0.0)
  panel = mesh.move_by(panel, function(_, p) return { p[1], p[2], p[3] * (0.03 + 0.97 * (p[1] / R + 0.5)) } end)   -- narrow at the hub, full width at the rim
  panel = mesh.translate(mesh.rotate_z(mesh.translate(panel, { R / 2, 0.012, 0 }), -DROOP), { 0, 0, 0 })
  k.mesh("fabric", panel, cloth(i % 2 == 1 and a.cloth or a.second))
  kit.put(m, rib, k)
  rib:marker("tip_" .. i, { pos = { R * c_, -R * s_, 0 } })
end
kit.far_box(m, m, "fabric", { a.span * 0.8, 0.1, a.span * 0.8 }, { 0, TOP - 0.25, 0 }, cloth(a.cloth), far)
return m
