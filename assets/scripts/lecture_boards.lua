-- A LECTURE HALL'S SLIDING BLACKBOARDS (#191): two boards, one in front of the other, in a tall frame on the wall.
-- Each is on a node, `board_front` and `board_back`, and slides up its own height; hung over a pulley as real
-- ones are, one goes up as the other comes down (lecture_boards_slide.lua moves them so). A chalk tray under each.
-- Markers: `write_front`, `write_back`, `teach` on the floor.
-- It stands on y = 0 at its own origin, against the wall, facing +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  width  = { 3.0, min = 1.8, max = 5.0 },         -- metres
  board  = { 1.1, min = 0.9, max = 1.4 },         -- one board's height
  colour = { "green", "black", "white" },
}
local W, BH = a.width, a.board
local slate = ({ green = V(0.12, 0.28, 0.2), black = V(0.07, 0.08, 0.08), white = V(0.95, 0.95, 0.94) })[a.colour]
local ALU = V(0.72, 0.73, 0.75)
local m = model.new()
local far = m:level(60)
local FOOT = 0.9                                 -- the lower board's foot
local TOP = FOOT + 2 * BH + 0.1
local c = kit.maker()
for _, x in ipairs{ -W / 2 - 0.04, W / 2 + 0.04 } do c.box("metal", V(x, TOP / 2, 0.05), V(0.08, TOP, 0.12), 0.006, ALU, 1) end
c.box("metal", V(0, TOP + 0.04, 0.05), V(W + 0.16, 0.08, 0.12), 0.006, ALU, 1)
c.box("paint", V(0, FOOT + BH, 0.005), V(W, 2 * BH, 0.01), 0.0, kit.WHITE * 0.85, 1)   -- the wall panel behind
kit.put(m, m, c)
kit.far_box(m, m, "paint", { W, 2 * BH, 0.02 }, { 0, FOOT + BH, 0.02 }, slate, far)
m:shape("frame", { box = { W + 0.16, TOP, 0.14 }, pos = { 0, TOP / 2, 0.05 } })
m:marker("teach", { pos = { 0, 0, 0.9 }, turn = { 0, 180, 0 }, role = "stand" })
local function board(name, y, z, lo, hi)
  local n = m:node("board_" .. name, { pos = { 0, y, z }, motion = { slide = { 0, 1, 0 }, min = lo, max = hi } })
  local k = kit.maker()
  k.box("paint", V(0, BH / 2, 0), V(W - 0.01, BH - 0.02, 0.016), 0.0, slate, 1)
  for _, yy in ipairs{ 0.012, BH - 0.012 } do k.box("metal", V(0, yy, 0), V(W, 0.024, 0.024), 0.004, ALU, 1) end
  k.box("metal", V(0, 0.01, 0.035), V(W - 0.1, 0.012, 0.06), 0.002, ALU, 1)           -- the chalk tray
  k.box("ceramic", V(-W * 0.3, 0.024, 0.035), V(0.07, 0.012, 0.012), 0.004, kit.WHITE, 1)
  k.box("fabric", V(W * 0.3, 0.035, 0.035), V(0.13, 0.04, 0.05), 0.008, kit.FABRIC[6], 1)
  kit.put(m, n, k)
  n:shape("board_" .. name, { box = { W, BH, 0.03 }, pos = { 0, BH / 2, 0 } })
  n:marker("write_" .. name, { pos = { 0, BH / 2, 0.012 }, role = "screen", size = BH })
end
board("front", FOOT, 0.09, 0, BH)                -- placed low, goes up
board("back", FOOT + BH, 0.04, -BH, 0)           -- placed high, comes down
return m
