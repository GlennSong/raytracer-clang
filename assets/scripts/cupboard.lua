-- A CUPBOARD WITH DOORS THAT OPEN: a wardrobe, a kitchen's floor or wall cupboard, an office's store cupboard
-- (#191). One or two doors, each on a node, `door_1`, `door_2`, hinged at the cupboard's outer edge and opening
-- 110 degrees; shelves inside, and a hanging rail in place of the upper shelves when asked. Each handle is a
-- marker, `handle_N`. It stands on y = 0 at its own origin (a wall cupboard: its origin is the middle of its
-- foot, so place it at the height it hangs), its doors facing +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  width   = { 1.0, min = 0.3, max = 1.6 },        -- metres
  height  = { 2.0, min = 0.4, max = 2.4 },
  depth   = { 0.58, min = 0.25, max = 0.7 },
  doors   = { 2, min = 1, max = 2, step = 1 },
  shelves = { 3, min = 0, max = 6, step = 1 },    -- inside, evenly spaced
  rail    = { "none", "rail" },                   -- a rail to hang clothes on, under the top shelf
  plinth  = { 0.08, min = 0, max = 0.15 },        -- the recessed foot it stands on (0: none, a wall cupboard)
  finish  = { "wood", "white", "black", "steel" },
  wood    = kit.wood_names,
}
local W, H, D, FOOT = a.width, a.height, a.depth, a.plinth
local doors, shelves = math.floor(a.doors + 0.5), math.floor(a.shelves + 0.5)
local part = a.finish == "wood" and "wood" or "paint"
local col = ({ wood = kit.wood(a.wood), white = kit.WHITE, black = kit.BLACK * 1.8, steel = V(0.55, 0.57, 0.6) })[a.finish]
local handle = a.finish == "black" and kit.CHROME or kit.BLACK * 2

local m = model.new()
local far = m:level(30)
local T = 0.018
local c = kit.maker()
if FOOT > 0 then c.box(part, V(0, FOOT / 2, -0.02), V(W - 0.04, FOOT, D - 0.08), 0.0, col * 0.7, 1) end
for _, x in ipairs{ -W / 2 + T / 2, W / 2 - T / 2 } do c.box(part, V(x, FOOT + (H - FOOT) / 2, 0), V(T, H - FOOT, D), 0.003, col, 1) end
c.box(part, V(0, FOOT + (H - FOOT) / 2, -D / 2 + T / 2), V(W - 2 * T, H - FOOT, T), 0.0, col * 0.9, 1)
c.box(part, V(0, FOOT + T / 2, 0), V(W - 2 * T, T, D), 0.0, col, 1)
c.box(part, V(0, H - T / 2, 0), V(W - 2 * T, T, D), 0.0, col, 1)
local inside = H - FOOT - 2 * T
for i = 1, shelves do
  local y = FOOT + T + inside * i / (shelves + 1)
  if not (a.rail == "rail" and i > 1) then c.box(part, V(0, y, -0.01), V(W - 2 * T, T, D - 0.04), 0.0, col * 0.9, 1) end
end
if a.rail == "rail" then c.rod("metal", V(-W / 2 + T, H - 0.12, 0), V(W / 2 - T, H - 0.12, 0), 0.012, kit.CHROME, 10) end
kit.put(m, m, c)
kit.far_box(m, m, part, { W, H, D }, { 0, H / 2, 0 }, col, far)
-- solid as its sides, back, top and bottom, so an open cupboard can be reached into
m:shape("back", { box = { W, H, T }, pos = { 0, H / 2, -D / 2 + T / 2 } })
m:shape("left", { box = { T, H, D }, pos = { -W / 2 + T / 2, H / 2, 0 } })
m:shape("right", { box = { T, H, D }, pos = { W / 2 - T / 2, H / 2, 0 } })
m:shape("top", { box = { W, T, D }, pos = { 0, H - T / 2, 0 } })
m:shape("bottom", { box = { W, FOOT + T, D }, pos = { 0, (FOOT + T) / 2, 0 } })

-- THE DOORS. A door's space: its origin is the foot of its hinge, and it runs along +x (left) or -x (right).
local DW, DH = W / doors, H - FOOT
for i = 1, doors do
  local left = i == 1
  local door = m:node("door_" .. i, { pos = { left and -W / 2 or W / 2, FOOT, D / 2 },
                                      motion = { turn = { 0, left and -1 or 1, 0 }, min = 0, max = 110 } })
  local k = kit.maker()
  local x0, x1 = left and 0 or -DW, left and DW or 0
  k.box(part, V((x0 + x1) / 2, DH / 2, 0.009), V(DW - 0.004, DH - 0.004, 0.018), 0.004, col, 1)
  local hx = left and DW - 0.05 or -DW + 0.05
  if doors == 1 then hx = left and DW - 0.05 or hx end
  local hy = DH > 1.2 and 1.0 - FOOT or DH / 2
  k.rod("metal", V(hx, hy - 0.08, 0.035), V(hx, hy + 0.08, 0.035), 0.007, handle, 8)
  for _, dy in ipairs{ -0.08, 0.08 } do k.rod("metal", V(hx, hy + dy, 0.018), V(hx, hy + dy, 0.035), 0.005, handle, 6) end
  kit.put(m, door, k)
  kit.far_box(m, door, part, { DW, DH, 0.018 }, { (x0 + x1) / 2, DH / 2, 0.009 }, col, far)
  door:shape("door_" .. i, { box = { DW, DH, 0.02 }, pos = { (x0 + x1) / 2, DH / 2, 0.01 } })
  door:marker("handle_" .. i, { pos = { hx, hy, 0.04 }, role = "hold" })
end
m:marker("front", { pos = { 0, 0, D / 2 + 0.6 }, turn = { 0, 180, 0 }, role = "stand" })
return m
