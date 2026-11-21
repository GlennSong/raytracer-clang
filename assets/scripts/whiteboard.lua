-- A WHITEBOARD ON WHEELS (#191): a board in a frame between two uprights on castered feet. The board is on a node,
-- `board`, that turns over about its middle, so either side can face the room. A pen tray runs along its foot.
-- It stands on y = 0 at its own origin, the board facing +z (and -z).
local kit = require "model_kit"
local V = kit.V

local a = params{
  width  = { 1.5, min = 0.9, max = 2.4 },         -- the board's width (metres)
  height = { 1.0, min = 0.6, max = 1.2 },         -- and its height
}
local W, H = a.width, a.height
local ALU = V(0.75, 0.76, 0.78)

local m = model.new()
local far = m:level(40)
local MID = 0.55 + H / 2 + 0.25                -- the height of the board's middle
local X = W / 2 + 0.05                         -- where each upright stands

local s = kit.maker()
for _, x in ipairs{ -X, X } do
  s.box("metal", V(x, MID / 2 + 0.25, 0), V(0.04, MID + 0.4, 0.03), 0.006, ALU, 1)
  s.box("metal", V(x, 0.09, 0), V(0.05, 0.035, 0.6), 0.01, ALU, 1)
  for _, z in ipairs{ -0.27, 0.27 } do kit.caster(s, V(x, 0, z), 0.035) end
  m:shape("upright_" .. (x < 0 and "left" or "right"), { box = { 0.05, MID + 0.5, 0.05 }, pos = { x, (MID + 0.5) / 2, 0 } })
end
s.box("metal", V(0, 0.2, 0), V(2 * X, 0.03, 0.03), 0.006, ALU, 1)                     -- the rail between the feet
kit.put(m, m, s)

local board = m:node("board", { pos = { 0, MID, 0 }, motion = { turn = { 1, 0, 0 } } })
local b = kit.maker()
b.box("hard", V(0, 0, 0), V(W, H, 0.012), 0.0, V(0.96, 0.96, 0.95), 1)
for _, y in ipairs{ -H / 2, H / 2 } do b.box("metal", V(0, y, 0), V(W + 0.03, 0.022, 0.022), 0.004, ALU, 1) end
for _, x in ipairs{ -W / 2, W / 2 } do b.box("metal", V(x, 0, 0), V(0.022, H + 0.03, 0.022), 0.004, ALU, 1) end
b.box("metal", V(0, -H / 2 + 0.005, 0.045), V(W * 0.8, 0.008, 0.07), 0.002, ALU, 1)   -- the pen tray
b.box("hard", V(-W * 0.2, -H / 2 + 0.018, 0.045), V(0.12, 0.016, 0.016), 0.006, kit.paint.blue, 1)
b.box("hard", V(-W * 0.2 + 0.15, -H / 2 + 0.018, 0.045), V(0.12, 0.016, 0.016), 0.006, kit.paint.red, 1)
b.box("fabric", V(W * 0.25, -H / 2 + 0.028, 0.045), V(0.12, 0.035, 0.05), 0.008, kit.FABRIC[6], 1)    -- the eraser
for _, x in ipairs{ -W / 2 - 0.03, W / 2 + 0.03 } do b.rod("metal", V(x - 0.02, 0, 0), V(x + 0.02, 0, 0), 0.012, kit.BLACK * 2, 8) end   -- the pivots
kit.put(m, board, b)
kit.far_box(m, board, "hard", { W, H, 0.02 }, { 0, 0, 0 }, V(0.96, 0.96, 0.95), far)
board:shape("board", { box = { W, H, 0.03 }, pos = { 0, 0, 0 } })
board:marker("write", { pos = { 0, 0, 0.01 }, role = "screen", size = H })
m:marker("stand", { pos = { 0, 0, 0.7 }, turn = { 0, 180, 0 }, role = "stand" })
return m
