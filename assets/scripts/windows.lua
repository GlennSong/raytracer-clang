-- WINDOWS AND SCREENS THAT OPEN, for a home's walls (#191):
--
--   sash       a sash window: the upper half is fixed and the lower half slides up behind it (`lower_sash`)
--   casement   two glazed leaves hinged at the sides, opening outward (`leaf_left`, `leaf_right`)
--   shoji      two sliding paper screens in a wooden frame, each on its own track (`screen_1`, `screen_2`)
--
-- Each stands in the plane of its wall: the origin is the middle of its foot (a window's sill, a screen's
-- floor track), and the outside is toward +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind   = { "sash", "casement", "shoji" },
  width  = { 1.0, min = 0.6, max = 2.6 },         -- metres, over the frame
  height = { 1.3, min = 0.8, max = 2.2 },
  finish = { "white", "oak", "walnut", "black" },
}
local W, H = a.width, a.height
local col = a.finish == "white" and kit.WHITE or a.finish == "black" and kit.BLACK * 1.5 or kit.wood(a.finish)
local part = (a.finish == "oak" or a.finish == "walnut") and "wood" or "paint"
local GLASS = V(0.75, 0.86, 0.9)
local F, DEEP = 0.05, 0.1
local m = model.new()
local far = m:level(60)

local c = kit.maker()
for _, x in ipairs{ -W / 2 + F / 2, W / 2 - F / 2 } do c.box(part, V(x, H / 2, 0), V(F, H, DEEP), 0.004, col, 1) end
for _, y in ipairs{ F / 2, H - F / 2 } do c.box(part, V(0, y, 0), V(W - 2 * F, F, DEEP), 0.004, col, 1) end
for i, x in ipairs{ -W / 2 + F / 2, W / 2 - F / 2 } do m:shape("jamb_" .. i, { box = { F, H, DEEP }, pos = { x, H / 2, 0 } }) end
m:shape("head", { box = { W, F, DEEP }, pos = { 0, H - F / 2, 0 } })

-- a glazed leaf from (x0, y0) to (x1, y1) at depth z, with `bars` glazing bars across it
local function leaf(k, x0, x1, y0, y1, z, bars)
  local S = 0.04
  for _, x in ipairs{ x0 + S / 2, x1 - S / 2 } do k.box(part, V(x, (y0 + y1) / 2, z), V(S, y1 - y0, 0.03), 0.004, col, 1) end
  for _, y in ipairs{ y0 + S / 2, y1 - S / 2 } do k.box(part, V((x0 + x1) / 2, y, z), V(x1 - x0 - 2 * S, S, 0.03), 0.004, col, 1) end
  for i = 1, bars do k.box(part, V((x0 + x1) / 2, y0 + (y1 - y0) * i / (bars + 1), z), V(x1 - x0 - 2 * S, 0.015, 0.02), 0.0, col, 1) end
  k.box("glass", V((x0 + x1) / 2, (y0 + y1) / 2, z), V(x1 - x0 - 2 * S, y1 - y0 - 2 * S, 0.005), 0.0, GLASS, 1)
end

local IW = W - 2 * F                          -- the opening's width, inside the frame
if a.kind == "sash" then
  local mid = H / 2
  leaf(c, -IW / 2, IW / 2, mid - 0.02, H - F, 0.02, 0)
  m:shape("upper", { box = { IW, H / 2, 0.03 }, pos = { 0, H * 0.75, 0.02 } })
  local sash = m:node("lower_sash", { pos = { 0, F, -0.02 }, motion = { slide = { 0, 1, 0 }, min = 0, max = mid - F - 0.06 } })
  local k = kit.maker()
  leaf(k, -IW / 2, IW / 2, 0, mid - F + 0.02, 0, 0)
  for _, x in ipairs{ -0.12, 0.12 } do k.box("metal", V(x, 0.02, -0.022), V(0.06, 0.012, 0.014), 0.004, kit.CHROME, 1) end   -- the lifts
  kit.put(m, sash, k)
  sash:shape("lower_sash", { box = { IW, mid - F, 0.03 }, pos = { 0, (mid - F) / 2, 0 } })
  sash:marker("lift", { pos = { 0, 0.02, -0.03 }, turn = { 0, 180, 0 }, role = "hold" })
elseif a.kind == "casement" then
  c.box(part, V(0, F / 2, 0.04), V(W + 0.06, 0.03, DEEP + 0.08), 0.006, col, 1)          -- the sill
  for _, side in ipairs{ -1, 1 } do
    local name = side < 0 and "leaf_left" or "leaf_right"
    local n = m:node(name, { pos = { side * IW / 2, F, 0.02 }, motion = { turn = { 0, side, 0 }, min = 0, max = 90 } })
    local k = kit.maker()
    local w = IW / 2 - 0.002
    if side < 0 then leaf(k, 0, w, 0, H - 2 * F, 0, 1) else leaf(k, -w, 0, 0, H - 2 * F, 0, 1) end
    k.box("metal", V(-side * (w - 0.02), (H - 2 * F) / 2, -0.025), V(0.014, 0.1, 0.02), 0.004, kit.CHROME, 1)
    kit.put(m, n, k)
    n:shape(name, { box = { w, H - 2 * F, 0.03 }, pos = { -side * w / 2, (H - 2 * F) / 2, 0 } })
    n:marker(name, { pos = { -side * (w - 0.02), (H - 2 * F) / 2, -0.04 }, turn = { 0, 180, 0 }, role = "hold" })
  end
else
  local PAPER = V(0.96, 0.94, 0.88)
  local w, h = IW / 2 + 0.02, H - 2 * F
  for i, side in ipairs{ -1, 1 } do
    local n = m:node("screen_" .. i, { pos = { side * IW / 2, F, -side * 0.016 }, motion = { slide = { -side, 0, 0 }, min = 0, max = w - 0.05 } })
    local k = kit.maker()
    local cx, S = -side * w / 2, 0.03
    for _, x in ipairs{ cx - w / 2 + S / 2, cx + w / 2 - S / 2 } do k.box(part, V(x, h / 2, 0), V(S, h, 0.028), 0.003, col, 1) end
    for _, y in ipairs{ S / 2, h - S / 2 } do k.box(part, V(cx, y, 0), V(w - 2 * S, S, 0.028), 0.003, col, 1) end
    k.box(part, V(cx, 0.18, 0), V(w - 2 * S, 0.3, 0.012), 0.0, col * 0.95, 1)                                  -- the board at the foot
    for j = 1, 3 do k.box(part, V(cx - w / 2 + j * w / 4, (h + 0.33) / 2, 0), V(0.008, h - 0.33 - S, 0.012), 0.0, col, 1) end
    for j = 0, 5 do k.box(part, V(cx, 0.33 + (h - 0.33 - S) * j / 6, 0), V(w - 2 * S, 0.008, 0.012), 0.0, col, 1) end
    k.box("hard", V(cx, (h + 0.33) / 2, 0), V(w - 2 * S, h - 0.33 - S, 0.002), 0.0, PAPER, 1)                  -- the paper
    kit.put(m, n, k)
    kit.far_box(m, n, "hard", { w, h, 0.02 }, { cx, h / 2, 0 }, PAPER, far)
    n:shape("screen_" .. i, { box = { w, h, 0.028 }, pos = { cx, h / 2, 0 } })
    n:marker("pull_" .. i, { pos = { -side * (w - 0.06), h * 0.5, 0.02 }, role = "hold" })
  end
end
kit.put(m, m, c)
if a.kind ~= "shoji" then kit.far_box(m, m, "glass", { IW, H - 2 * F, 0.01 }, { 0, H / 2, 0 }, GLASS, far) end
return m
