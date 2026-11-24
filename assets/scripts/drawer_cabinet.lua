-- A CABINET OF DRAWERS that open: an office's filing cabinet, or the pedestal that stands under a desk (#191).
-- Each drawer is a node, `drawer_1` (the lowest) ..., that slides out four fifths of its depth, with a box
-- behind its front, so an open drawer is a drawer and not a panel. Each handle is a marker, `handle_N`.
-- It stands on y = 0 at its own origin, its fronts facing +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  drawers = { 4, min = 1, max = 5, step = 1 },    -- how many
  width   = { 0.46, min = 0.3, max = 1.0 },       -- the cabinet's width (metres)
  depth   = { 0.62, min = 0.4, max = 0.8 },
  drawer  = { 0.31, min = 0.12, max = 0.4 },      -- one drawer's height
  finish  = { "steel", "white", "black", "wood" },
  wood    = kit.wood_names,                       -- which wood, when the finish is wood
  casters = { "none", "casters" },                -- on the floor, or on four casters
}
local count = math.floor(a.drawers + 0.5)
local W, D, DH = a.width, a.depth, a.drawer
local col, part = kit.finish(a.finish == "wood" and a.wood or a.finish)
local handle = a.finish == "black" and kit.CHROME or kit.BLACK * 2

local m = model.new()
local far = m:level(30)
local T = 0.018                              -- the carcass's panels
local LIFT = a.casters == "casters" and 0.07 or 0.0
local H = count * DH + 2 * T                 -- the carcass's height

-- THE CARCASS: two sides, a back, a top and a bottom, open at the front
local c = kit.maker()
for _, x in ipairs{ -W / 2 + T / 2, W / 2 - T / 2 } do c.box(part, V(x, LIFT + H / 2, 0), V(T, H, D), 0.004, col, 1) end
c.box(part, V(0, LIFT + H / 2, -D / 2 + T / 2), V(W - 2 * T, H, T), 0.0, col, 1)
c.box(part, V(0, LIFT + T / 2, 0), V(W - 2 * T, T, D), 0.0, col, 1)
c.box(part, V(0, LIFT + H - T / 2, 0), V(W - 2 * T, T, D), 0.0, col, 1)
if LIFT > 0 then
  for _, x in ipairs{ -W / 2 + 0.05, W / 2 - 0.05 } do
    for _, z in ipairs{ -D / 2 + 0.05, D / 2 - 0.05 } do kit.caster(c, V(x, 0, z), 0.03) end
  end
end
kit.put(m, m, c)
kit.far_box(m, m, part, { W, H, D }, { 0, LIFT + H / 2, 0 }, col, far)
m:shape("carcass", { box = { W, H + LIFT, D }, pos = { 0, (H + LIFT) / 2, 0 } })

-- THE DRAWERS. A drawer's space: its origin is the middle of its front's foot, on the carcass's front face.
for i = 1, count do
  local y = LIFT + T + (i - 1) * DH
  local drawer = m:node("drawer_" .. i, { pos = { 0, y, D / 2 }, motion = { slide = { 0, 0, 1 }, min = 0, max = D * 0.8 } })
  local k = kit.maker()
  k.front(part, -W / 2, W / 2, 0, DH, 0, col, handle)
  local iw, ih, id = W - 2 * T - 0.02, DH * 0.72, D - T - 0.03          -- the box behind the front
  k.box(part, V(0, 0.012, -id / 2), V(iw, 0.008, id), 0.0, col * 0.8, 1)
  for _, x in ipairs{ -iw / 2, iw / 2 } do k.box(part, V(x, ih / 2 + 0.008, -id / 2), V(0.008, ih, id), 0.0, col * 0.8, 1) end
  k.box(part, V(0, ih / 2 + 0.008, -id), V(iw, ih, 0.008), 0.0, col * 0.8, 1)
  kit.put(m, drawer, k)
  drawer:shape("drawer_" .. i, { box = { W - 2 * T - 0.01, ih, id }, pos = { 0, ih / 2 + 0.008, -id / 2 } })
  drawer:marker("handle_" .. i, { pos = { 0, DH - math.min(0.06, DH * 0.3), 0.03 }, role = "hold" })
end
m:marker("front", { pos = { 0, 0, D / 2 + 0.5 }, turn = { 0, 180, 0 }, role = "stand" })
return m
