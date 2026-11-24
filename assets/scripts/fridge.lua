-- A FRIDGE WITH DOORS THAT OPEN (#191): a tall white box, the freezer below and the fridge above, each with its
-- own door on a node (`fridge_door`, `freezer_door`), hinged on the left and opening 120 degrees. Inside: glass
-- shelves, a lit back, bins in the door; in the freezer two drawers that slide out (`drawer_1`, `drawer_2`, each with
-- a marker `drawer_N_handle`). Markers: a handle on each door, and `light` inside.
-- It stands on y = 0 at its own origin, its doors facing +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  width   = { 0.6, min = 0.5, max = 0.9 },        -- metres
  height  = { 1.8, min = 1.2, max = 2.0 },
  depth   = { 0.62, min = 0.5, max = 0.75 },
  freezer = { 0.6, min = 0.3, max = 0.9 },        -- the freezer's height, at the foot
  finish  = { "white", "steel", "black" },
}
local W, H, D, FZ = a.width, a.height, a.depth, a.freezer
local col, part = kit.finish(a.finish)
local LINER = V(0.93, 0.94, 0.95)

local m = model.new()
local far = m:level(30)
local T, FOOT, DOOR = 0.04, 0.05, 0.05
local c = kit.maker()
c.box("hard", V(0, FOOT / 2, -0.02), V(W - 0.04, FOOT, D - 0.1), 0.0, kit.BLACK, 1)
for _, x in ipairs{ -W / 2 + T / 2, W / 2 - T / 2 } do c.box(part, V(x, FOOT + (H - FOOT) / 2, 0), V(T, H - FOOT, D), 0.008, col, 1) end
c.box(part, V(0, FOOT + (H - FOOT) / 2, -D / 2 + T / 2), V(W - 2 * T, H - FOOT, T), 0.0, col, 1)
c.box(part, V(0, H - T / 2, 0), V(W - 2 * T, T, D), 0.0, col, 1)
c.box(part, V(0, FOOT + T / 2, 0), V(W - 2 * T, T, D), 0.0, col, 1)
c.box(part, V(0, FOOT + FZ, 0), V(W - 2 * T, T, D), 0.0, col, 1)                      -- between the freezer and the fridge
-- the liner, the shelves and the light at the back of the fridge
c.box("hard", V(0, FOOT + (H - FOOT) / 2, -D / 2 + T + 0.002), V(W - 2 * T, H - FOOT - 2 * T, 0.004), 0.0, LINER, 1)
local low, high = FOOT + FZ + T / 2, H - T
for i = 1, 3 do c.box("glass", V(0, low + (high - low) * i / 4, -0.01), V(W - 2 * T, 0.006, D - T - 0.08), 0.0, V(0.8, 0.9, 0.9), 1) end
c.box("light", V(0, high - 0.05, -D / 2 + T + 0.02), V(0.2, 0.03, 0.02), 0.0, V(1, 1, 0.95), 1)
kit.put(m, m, c)
-- the freezer's two drawers, each on a node that slides out (`drawer_1` below, `drawer_2` above): a tray with a clear
-- front, frozen things in it
for i = 1, 2 do
  local dh, dw, dd = (FZ - T) / 2 - 0.03, W - 2 * T - 0.02, D - T - 0.08
  local y0 = FOOT + T + (FZ - T) * (i - 1) / 2 + 0.01
  local dr = m:node("drawer_" .. i, { pos = { 0, y0, 0.01 }, motion = { slide = { 0, 0, 1 }, min = 0, max = 0.4 } })
  local k = kit.maker()
  k.box("hard", V(0, 0.006, 0), V(dw, 0.012, dd), 0.0, LINER, 1)
  for _, x in ipairs{ -dw / 2 + 0.005, dw / 2 - 0.005 } do k.box("hard", V(x, dh / 2, 0), V(0.01, dh, dd), 0.0, LINER, 1) end
  k.box("hard", V(0, dh / 2, -dd / 2 + 0.005), V(dw, dh, 0.01), 0.0, LINER, 1)
  k.box("glass", V(0, dh / 2, dd / 2 - 0.005), V(dw, dh, 0.01), 0.0, V(0.8, 0.9, 0.95), 1)
  k.box("hard", V(0, dh - 0.02, dd / 2 + 0.008), V(dw * 0.5, 0.025, 0.016), 0.006, LINER * 0.9, 1)                 -- its grip
  local things = i == 1 and { { -0.12, V(0.85, 0.3, 0.25) }, { 0.06, V(0.3, 0.5, 0.8) }, { 0.17, V(0.9, 0.85, 0.5) } }
                         or { { -0.14, V(0.4, 0.7, 0.4) }, { 0.0, V(0.95, 0.95, 0.9) }, { 0.14, V(0.7, 0.4, 0.6) } }
  for _, t in ipairs(things) do k.box("hard", V(t[1], 0.012 + dh * 0.3, 0.02), V(0.1, dh * 0.6, dd * 0.55), 0.008, t[2], 1) end
  kit.put(m, dr, k)
  dr:marker("drawer_" .. i .. "_handle", { pos = { 0, dh - 0.02, dd / 2 + 0.03 }, role = "hold" })
end
kit.far_box(m, m, part, { W, H, D + DOOR }, { 0, H / 2, DOOR / 2 }, col, far)
m:shape("back", { box = { W, H, T }, pos = { 0, H / 2, -D / 2 + T / 2 } })
m:shape("left", { box = { T, H, D }, pos = { -W / 2 + T / 2, H / 2, 0 } })
m:shape("right", { box = { T, H, D }, pos = { W / 2 - T / 2, H / 2, 0 } })
m:shape("top", { box = { W, T, D }, pos = { 0, H - T / 2, 0 } })
m:shape("freezer", { box = { W, FOOT + FZ, D }, pos = { 0, (FOOT + FZ) / 2, 0 } })
m:marker("light", { pos = { 0, high - 0.06, -D / 2 + T + 0.04 }, turn = { 0, 0, 0 }, role = "lamp" })

local function door(name, y0, y1, bins)
  local n = m:node(name, { pos = { -W / 2, y0, D / 2 }, motion = { turn = { 0, -1, 0 }, min = 0, max = 120 } })
  local k = kit.maker()
  local h = y1 - y0
  k.box(part, V(W / 2, h / 2, DOOR / 2), V(W - 0.004, h - 0.006, DOOR), 0.012, col, 2)
  k.box("hard", V(W / 2, h / 2, -0.004), V(W - 0.1, h - 0.1, 0.008), 0.0, LINER, 1)   -- the liner on its inside
  for i = 1, bins do k.box("hard", V(W / 2, h * i / (bins + 1) - 0.05, -0.05), V(W - 0.14, 0.07, 0.09), 0.01, LINER, 1) end
  local hy = name == "fridge_door" and 0.25 or h - 0.2                                -- handles meet near the middle
  k.rod("metal", V(W - 0.06, hy - 0.14, DOOR + 0.035), V(W - 0.06, hy + 0.14, DOOR + 0.035), 0.009, kit.CHROME, 8)
  for _, dy in ipairs{ -0.14, 0.14 } do k.rod("metal", V(W - 0.06, hy + dy, DOOR), V(W - 0.06, hy + dy, DOOR + 0.035), 0.006, kit.CHROME, 6) end
  kit.put(m, n, k)
  n:shape(name, { box = { W, h, DOOR }, pos = { W / 2, h / 2, DOOR / 2 } })
  n:marker(name .. "_handle", { pos = { W - 0.06, hy, DOOR + 0.04 }, role = "hold" })
end
door("freezer_door", FOOT, FOOT + FZ, 0)
door("fridge_door", FOOT + FZ, H, 3)
m:marker("front", { pos = { 0, 0, D / 2 + 0.7 }, turn = { 0, 180, 0 }, role = "stand" })
return m
