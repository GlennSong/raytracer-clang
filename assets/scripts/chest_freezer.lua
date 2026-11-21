-- AN ICE-CREAM FREEZER with sliding glass lids (#191): a waist-high chest, its two curved lids each on a node,
-- `lid_left` and `lid_right`, sliding over the other. Baskets of tubs and lollies inside. Markers: `reach`
-- (over the open half), `front`. It stands on y = 0 at its own origin, its front toward +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  width  = { 1.5, min = 1.0, max = 2.2 },         -- metres
  finish = { "white", "blue", "red" },
}
local W = a.width
local col = a.finish == "white" and kit.WHITE or kit.paint[a.finish]
local m = model.new()
local far = m:level(40)
local D, H, T = 0.75, 0.85, 0.05
local c = kit.maker()
for _, x in ipairs{ -W / 2 + T / 2, W / 2 - T / 2 } do c.box("paint", V(x, H / 2, 0), V(T, H, D), 0.01, col, 1) end
for _, z in ipairs{ -D / 2 + T / 2, D / 2 - T / 2 } do c.box("paint", V(0, H / 2, z), V(W - 2 * T, H, T), 0.0, col, 1) end
c.box("paint", V(0, 0.2, 0), V(W - 2 * T, 0.4, D - 2 * T), 0.0, kit.WHITE, 1)                  -- the floor of the well
c.box("light", V(0, H * 0.55, D / 2 + 0.002), V(W - 0.2, 0.22, 0.004), 0.0, V(1, 0.9, 0.7), 1)  -- the lit panel on the front
for i = 0, 3 do                                 -- baskets of ices, in four colours
  local x = -W / 2 + T + (i + 0.5) * (W - 2 * T) / 4
  c.box("hard", V(x, 0.5, 0), V((W - 2 * T) / 4 - 0.03, 0.18, D - 2 * T - 0.04), 0.01, ({ V(0.9, 0.5, 0.6), V(0.5, 0.3, 0.15), V(0.95, 0.9, 0.7), V(0.4, 0.75, 0.6) })[i + 1], 1)
end
kit.put(m, m, c)
kit.far_box(m, m, "paint", { W, H, D }, { 0, H / 2, 0 }, col, far)
m:shape("body", { box = { W, H, D }, pos = { 0, H / 2, 0 } })
m:marker("front", { pos = { 0, 0, D / 2 + 0.6 }, turn = { 0, 180, 0 }, role = "stand" })
m:marker("reach", { pos = { -W / 4, H - 0.2, 0 }, role = "hold" })
local LW = W / 2 + 0.03
for i, s in ipairs{ -1, 1 } do
  local name = s < 0 and "lid_left" or "lid_right"
  local lid = m:node(name, { pos = { s * (W / 4 - 0.015), H + (i - 1) * 0.02, 0 }, motion = { slide = { -s, 0, 0 }, min = 0, max = W / 2 - 0.1 } })
  local k = kit.maker()
  k.box("glass", V(0, 0.008, 0), V(LW, 0.008, D - 0.06), 0.0, V(0.85, 0.93, 0.95), 1)
  for _, z in ipairs{ -D / 2 + 0.04, D / 2 - 0.04 } do k.box("metal", V(0, 0.008, z), V(LW, 0.016, 0.03), 0.004, V(0.75, 0.76, 0.78), 1) end
  k.box("hard", V(s * (LW / 2 - 0.03), 0.02, 0), V(0.03, 0.02, 0.2), 0.006, kit.BLACK, 1)       -- the grip
  kit.put(m, lid, k)
  lid:shape(name, { box = { LW, 0.02, D - 0.06 }, pos = { 0, 0.008, 0 } })
  lid:marker(name .. "_grip", { pos = { s * (LW / 2 - 0.03), 0.03, 0 }, role = "hold" })
end
return m
