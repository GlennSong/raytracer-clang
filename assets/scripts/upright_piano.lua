-- AN UPRIGHT PIANO (#191): the cover over its keys folds back (`fallboard`) and its top lifts at the back (`lid`).
-- As placed the keys are covered. Markers: `keys` (where hands go), `bench` (where the player sits), `sound`.
-- It stands on y = 0 at its own origin, its back to a wall, the keys toward +z.
local kit = require "model_kit"
local V = kit.V

local a = params{ finish = { "black", "walnut", "oak", "white" } }
local col = a.finish == "black" and kit.BLACK * 1.3 or a.finish == "white" and kit.WHITE or kit.wood(a.finish)
local part = (a.finish == "walnut" or a.finish == "oak") and "wood" or "paint"
local W, H, KEY_Y = 1.46, 1.2, 0.72
local m = model.new()
local far = m:level(40)
local c = kit.maker()
c.box(part, V(0, H / 2, -0.12), V(W, H, 0.36), 0.01, col, 1)                              -- the case
c.box(part, V(0, KEY_Y - 0.05, 0.17), V(W, 0.1, 0.26), 0.01, col, 1)                      -- the shelf the keys lie on
for _, x in ipairs{ -W / 2 + 0.04, W / 2 - 0.04 } do
  c.box(part, V(x, KEY_Y + 0.03, 0.17), V(0.08, 0.07, 0.26), 0.01, col, 1)                -- a block at each end of the keys
  c.post(part, V(x, 0, 0.25), V(x, KEY_Y - 0.1, 0.25), 0.07, 0.055, col)                  -- and a leg under it
end
c.box(part, V(0, KEY_Y + 0.22, 0.07), V(0.6, 0.2, 0.012), 0.004, col, 1)                  -- the music desk
c.box(part, V(0, 0.06, 0.07), V(W - 0.1, 0.12, 0.03), 0.006, col, 1)
for _, x in ipairs{ -0.1, 0, 0.1 } do c.box("metal", V(x, 0.035, 0.12), V(0.035, 0.012, 0.09), 0.005, V(0.8, 0.62, 0.25), 1) end   -- the pedals
-- the keys: 52 white ones, and the black ones between them in twos and threes
local KW, K0 = 1.22 / 52, -0.61
c.box("hard", V(0, KEY_Y + 0.008, 0.215), V(1.22, 0.016, 0.15), 0.0, V(0.96, 0.95, 0.9), 1)
for i = 1, 51 do
  c.box("hard", V(K0 + i * KW, KEY_Y + 0.0165, 0.215), V(0.0012, 0.0006, 0.15), 0.0, kit.BLACK * 3, 1)
  local n = (i + 4) % 7                                                                   -- which white key of the octave this follows
  if n ~= 2 and n ~= 6 then c.box("hard", V(K0 + i * KW, KEY_Y + 0.022, 0.185), V(KW * 0.58, 0.012, 0.09), 0.0, kit.BLACK, 1) end
end
kit.put(m, m, c)
kit.far_box(m, m, part, { W, H, 0.36 }, { 0, H / 2, -0.12 }, col, far)
kit.far_box(m, m, part, { W, 0.14, 0.26 }, { 0, KEY_Y - 0.03, 0.17 }, col, far)
m:shape("case", { box = { W, H, 0.36 }, pos = { 0, H / 2, -0.12 } })
m:shape("keys", { box = { W, 0.14, 0.26 }, pos = { 0, KEY_Y - 0.03, 0.17 } })
m:marker("keys", { pos = { 0, KEY_Y + 0.03, 0.24 }, role = "press", size = 1.2 })
m:marker("bench", { pos = { 0, 0.48, 0.7 }, turn = { 0, 180, 0 }, role = "seat" })
m:marker("sound", { pos = { 0, 0.9, 0 } })

local board = m:node("fallboard", { pos = { 0, KEY_Y + 0.03, 0.065 }, motion = { turn = { -1, 0, 0 }, min = 0, max = 92 } })
local f = kit.maker()
f.box(part, V(0, 0.007, 0.115), V(1.3, 0.014, 0.23), 0.004, col, 1)
kit.put(m, board, f)
board:marker("fallboard", { pos = { 0, 0.014, 0.22 }, role = "hold" })
local lid = m:node("lid", { pos = { 0, H, -0.3 }, motion = { turn = { -1, 0, 0 }, min = 0, max = 30 } })
local l = kit.maker()
l.box(part, V(0, 0.012, 0.185), V(W + 0.03, 0.024, 0.39), 0.008, col, 1)
kit.put(m, lid, l)
lid:shape("lid", { box = { W, 0.024, 0.38 }, pos = { 0, 0.012, 0.185 } })
lid:marker("lid", { pos = { 0, 0.012, 0.38 }, role = "hold" })
return m
