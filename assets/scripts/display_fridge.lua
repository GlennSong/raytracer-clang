-- A SHOP'S DRINKS FRIDGE (#191; a convenience store's back wall): a lit cabinet of shelves behind glass doors.
-- Each door is on a node, `door_1` ..., hinged on its left and opening 95 degrees. The shelves hold bottles and
-- cans, each kind stored once and drawn many times (scatter.grid). Markers: `handle_N`, `front`.
-- It stands on y = 0 at its own origin, its doors facing +z.
local kit = require "model_kit"
local scatter = require "scatter"
local V = kit.V

local a = params{
  doors   = { 3, min = 1, max = 5, step = 1 },
  door    = { 0.62, min = 0.5, max = 0.8 },       -- one door's width (metres)
  height  = { 2.0, min = 1.6, max = 2.2 },
  shelves = { 5, min = 3, max = 6, step = 1 },
  finish  = { "black", "white", "steel" },
}
local N, DW, H, S = math.floor(a.doors + 0.5), a.door, a.height, math.floor(a.shelves + 0.5)
local col, part = kit.finish(a.finish)
local m = model.new()
local far = m:level(40)
local W, D, T, FOOT, TOP = N * DW, 0.7, 0.04, 0.12, 0.16

local c = kit.maker()
for _, x in ipairs{ -W / 2 - T / 2, W / 2 + T / 2 } do c.box("paint", V(x, H / 2, 0), V(T, H, D), 0.006, col, 1) end
c.box("paint", V(0, H / 2, -D / 2 + T / 2), V(W, H, T), 0.0, col, 1)
c.box("paint", V(0, FOOT / 2, 0), V(W, FOOT, D), 0.0, col, 1)
c.box("paint", V(0, H - TOP / 2, 0), V(W, TOP, D), 0.0, col, 1)
c.box("light", V(0, H - TOP / 2, D / 2 + 0.002), V(W - 0.1, TOP - 0.06, 0.004), 0.0, V(0.9, 0.95, 1), 1)     -- the lit sign above the doors
c.box("hard", V(0, H / 2, -D / 2 + T + 0.002), V(W, H - FOOT - TOP, 0.004), 0.0, kit.WHITE, 1)               -- the white liner
for i = 1, N - 1 do c.box("paint", V(-W / 2 + i * DW, (FOOT + H - TOP) / 2, D / 2 - 0.03), V(0.03, H - FOOT - TOP, 0.06), 0.0, col, 1) end
local inside = H - FOOT - TOP
for s = 0, S - 1 do
  local y = FOOT + inside * s / S
  c.box("metal", V(0, y + 0.006, -0.03), V(W - 0.01, 0.012, D - T - 0.12), 0.0, V(0.8, 0.81, 0.83), 1)
  c.box("light", V(0, y + inside / S - 0.02, D / 2 - 0.1), V(W - 0.04, 0.012, 0.02), 0.0, V(1, 1, 0.95), 1)  -- a strip light over each shelf
end
kit.put(m, m, c)
kit.far_box(m, m, "paint", { W + 2 * T, H, D }, { 0, H / 2, 0 }, col, far)
m:shape("back", { box = { W, H, T }, pos = { 0, H / 2, -D / 2 + T / 2 } })
m:shape("left", { box = { T, H, D }, pos = { -W / 2 - T / 2, H / 2, 0 } })
m:shape("right", { box = { T, H, D }, pos = { W / 2 + T / 2, H / 2, 0 } })
m:shape("foot", { box = { W, FOOT, D }, pos = { 0, FOOT / 2, 0 } })
m:shape("top", { box = { W, TOP, D }, pos = { 0, H - TOP / 2, 0 } })
m:marker("front", { pos = { 0, 0, D / 2 + 0.7 }, turn = { 0, 180, 0 }, role = "stand" })
m:marker("lamp", { pos = { 0, H - TOP, 0 }, role = "lamp" })

-- THE STOCK: a bottle and a can, each in a few colours, in rows on every shelf
local function drink(profile, cap, colour)
  local p = model.new()
  p:add(mesh.paint(mesh.lathe{ profile = profile, segments = 10, smooth = true }, colour))
  if cap then p:add(mesh.paint(mesh.translate(mesh.cylinder(0.014, 0.02), { 0, cap, 0 }), kit.WHITE)) end
  return p
end
local BOTTLE = { { 0, 0 }, { 0.032, 0 }, { 0.033, 0.13 }, { 0.014, 0.18 }, { 0.014, 0.21 }, { 0, 0.21 } }
local CAN = { { 0, 0 }, { 0.03, 0 }, { 0.033, 0.01 }, { 0.033, 0.11 }, { 0.028, 0.122 }, { 0, 0.122 } }
local colours = { V(0.1, 0.45, 0.2), V(0.75, 0.1, 0.1), V(0.9, 0.6, 0.1), V(0.15, 0.3, 0.7), V(0.35, 0.2, 0.1), V(0.85, 0.85, 0.8), V(0.1, 0.1, 0.12) }
local across = math.max(2, math.floor((DW - 0.08) / 0.085))
for s = 0, S - 1 do
  for d = 0, N - 1 do
    local kind = (s + d) % 2 == 0
    local colour = colours[(s * 3 + d * 2) % #colours + 1]
    m:use(drink(kind and BOTTLE or CAN, kind and 0.21 or nil, colour),
          scatter.grid{ origin = { -W / 2 + d * DW + 0.04, FOOT + inside * s / S + 0.012, -D / 2 + T + 0.05 }, u = { DW - 0.08, 0, 0 }, v = { 0, 0, D - T - 0.22 },
                        count = { across, 4 }, margin = 0.035 })
  end
end

for i = 1, N do
  local door = m:node("door_" .. i, { pos = { -W / 2 + (i - 1) * DW, FOOT, D / 2 }, motion = { turn = { 0, -1, 0 }, min = 0, max = 95 } })
  local k = kit.maker()
  local h = inside
  k.box("glass", V(DW / 2, h / 2, 0.012), V(DW - 0.1, h - 0.1, 0.012), 0.0, V(0.85, 0.93, 0.95), 1)
  for _, x in ipairs{ 0.03, DW - 0.03 } do k.box("paint", V(x, h / 2, 0.012), V(0.055, h - 0.006, 0.03), 0.004, col, 1) end
  for _, y in ipairs{ 0.03, h - 0.03 } do k.box("paint", V(DW / 2, y, 0.012), V(DW - 0.006, 0.055, 0.03), 0.004, col, 1) end
  k.rod("metal", V(DW - 0.035, h / 2 - 0.2, 0.05), V(DW - 0.035, h / 2 + 0.2, 0.05), 0.01, kit.CHROME, 8)
  for _, dy in ipairs{ -0.2, 0.2 } do k.rod("metal", V(DW - 0.035, h / 2 + dy, 0.025), V(DW - 0.035, h / 2 + dy, 0.05), 0.006, kit.CHROME, 6) end
  kit.put(m, door, k)
  door:shape("door_" .. i, { box = { DW, h, 0.03 }, pos = { DW / 2, h / 2, 0.012 } })
  door:marker("handle_" .. i, { pos = { DW - 0.035, h / 2, 0.06 }, role = "hold" })
end
return m
