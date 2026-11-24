-- A BANK OF LOCKERS whose doors open (#191): a school's or a gym's tall lockers, a station's coin lockers, an
-- apartment lobby's mailboxes. Every door is on a node, `door_C_R` (column, then row from the foot), hinged on
-- its left and opening 110 degrees, with a marker `handle_C_R`. Each compartment is a real box with a shelf.
-- It stands on y = 0 at its own origin, its doors facing +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  columns = { 4, min = 1, max = 10, step = 1 },
  rows    = { 1, min = 1, max = 5, step = 1 },
  width   = { 0.32, min = 0.2, max = 0.6 },       -- one door's width (metres)
  height  = { 1.8, min = 0.4, max = 2.0 },        -- the whole bank's height above its foot
  depth   = { 0.5, min = 0.2, max = 0.7 },
  foot    = { 0.1, min = 0, max = 1.2 },          -- what it stands on (a mailbox bank hangs higher: a tall foot is its wall space)
  paint   = { "grey", "blue", "green", "red", "yellow", "orange", "steel" },
}
local C, R = math.floor(a.columns + 0.5), math.floor(a.rows + 0.5)
local DW, H, D, FOOT = a.width, a.height, a.depth, a.foot
local col = ({ grey = V(0.55, 0.57, 0.6), steel = V(0.68, 0.7, 0.72) })[a.paint] or kit.paint[a.paint]
local m = model.new()
local far = m:level(40)
local W, T, DH = C * DW, 0.012, H / R
local c = kit.maker()
if FOOT > 0 and FOOT <= 0.2 then c.box("paint", V(0, FOOT / 2, -0.02), V(W, FOOT, D - 0.06), 0.0, col * 0.6, 1) end
for i = 0, C do c.box("paint", V(-W / 2 + i * DW, FOOT + H / 2, 0), V(T, H, D), 0.0, col, 1) end
for j = 0, R do c.box("paint", V(0, FOOT + j * DH, 0), V(W, T, D), 0.0, col, 1) end
c.box("paint", V(0, FOOT + H / 2, -D / 2 + T / 2), V(W, H, T), 0.0, col * 0.9, 1)
if R == 1 then c.box("paint", V(0, FOOT + H - 0.3, -0.01), V(W, T, D - 0.03), 0.0, col * 0.9, 1) end     -- the hat shelf
kit.put(m, m, c)
kit.far_box(m, m, "paint", { W, H, D }, { 0, FOOT + H / 2, 0 }, col, far)
m:shape("back", { box = { W, H, T * 2 }, pos = { 0, FOOT + H / 2, -D / 2 + T } })
m:shape("top", { box = { W, T * 2, D }, pos = { 0, FOOT + H, 0 } })
m:shape("bottom", { box = { W, math.max(FOOT, T * 2), D }, pos = { 0, FOOT > 0.2 and FOOT or FOOT / 2, 0 } })
for _, x in ipairs{ -W / 2, W / 2 } do m:shape("side_" .. (x < 0 and "left" or "right"), { box = { T * 2, H, D }, pos = { x, FOOT + H / 2, 0 } }) end
m:marker("front", { pos = { 0, 0, D / 2 + 0.6 }, turn = { 0, 180, 0 }, role = "stand" })
for i = 1, C do
  for j = 1, R do
    local name = string.format("%d_%d", i, j)
    local door = m:node("door_" .. name, { pos = { -W / 2 + (i - 1) * DW, FOOT + (j - 1) * DH, D / 2 }, motion = { turn = { 0, -1, 0 }, min = 0, max = 110 } })
    local k = kit.maker()
    k.box("paint", V(DW / 2, DH / 2, 0.006), V(DW - 0.008, DH - 0.008, 0.012), 0.003, col, 1)
    if DH > 0.8 then for v = 0, 3 do k.box("paint", V(DW / 2, DH - 0.12 - v * 0.035, 0.013), V(DW * 0.6, 0.012, 0.004), 0.0, col * 0.6, 1) end end   -- the vents
    k.box("metal", V(DW - 0.04, DH / 2, 0.016), V(0.02, math.min(0.1, DH * 0.3), 0.012), 0.004, kit.CHROME, 1)
    k.box("hard", V(DW / 2, DH - math.min(0.07, DH * 0.2), 0.0125), V(0.05, 0.018, 0.001), 0.0, kit.WHITE, 1)      -- the number
    kit.put(m, door, k)
    door:shape("door_" .. name, { box = { DW, DH, 0.014 }, pos = { DW / 2, DH / 2, 0.006 } })
    door:marker("handle_" .. name, { pos = { DW - 0.04, DH / 2, 0.03 }, role = "hold" })
  end
end
return m
