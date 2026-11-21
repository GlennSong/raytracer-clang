-- A DOOR ON HINGES, in its frame (#191): one leaf or two, solid or glazed, with a lever handle, a push bar, or
-- plates to push (a kitchen's doors, which swing both ways). Each leaf is on a node, `door_1`, `door_2`, hinged at
-- the frame's outer edge. A leaf opens 100 degrees toward +z; a swing door opens 85 degrees either way.
-- Markers: `handle_N` on each leaf, `outside` and `inside` on the floor either side.
-- It stands on y = 0 at its own origin, in the plane of the wall.
local kit = require "model_kit"
local V = kit.V

local a = params{
  leaves = { 1, min = 1, max = 2, step = 1 },
  width  = { 0.9, min = 0.7, max = 1.2 },         -- one leaf's width (metres)
  height = { 2.05, min = 1.9, max = 2.4 },
  kind   = { "lever", "push_bar", "swing" },      -- how it is opened
  glazed = { "solid", "window", "glass" },        -- a solid leaf, a small window in it, or glass in a frame
  finish = { "wood", "white", "grey", "green", "blue", "red" },
  wood   = kit.wood_names,
}
local leaves, W, H = math.floor(a.leaves + 0.5), a.width, a.height
local col, part = kit.finish(a.finish == "wood" and a.wood or a.finish)
local GLASS, FRAME = V(0.8, 0.9, 0.92), kit.WHITE * 0.92
local m = model.new()
local far = m:level(50)
local total, F, T = leaves * W, 0.06, 0.045     -- the opening's width, the frame's width, a leaf's thickness

local c = kit.maker()
for _, x in ipairs{ -total / 2 - F / 2, total / 2 + F / 2 } do
  c.box("paint", V(x, H / 2, 0), V(F, H, 0.12), 0.006, FRAME, 1)
  m:shape("frame_" .. (x < 0 and "left" or "right"), { box = { F, H, 0.12 }, pos = { x, H / 2, 0 } })
end
c.box("paint", V(0, H + F / 2, 0), V(total + 2 * F, F, 0.12), 0.006, FRAME, 1)
m:shape("head", { box = { total + 2 * F, F, 0.12 }, pos = { 0, H + F / 2, 0 } })
kit.put(m, m, c)
m:marker("outside", { pos = { 0, 0, 0.9 }, turn = { 0, 180, 0 }, role = "stand" })
m:marker("inside", { pos = { 0, 0, -0.9 }, role = "stand" })

for i = 1, leaves do
  local left = i == 1
  local motion = a.kind == "swing" and { turn = { 0, left and -1 or 1, 0 }, min = -85, max = 85 }
                                    or { turn = { 0, left and -1 or 1, 0 }, min = 0, max = 100 }
  local door = m:node("door_" .. i, { pos = { left and -total / 2 or total / 2, 0, 0 }, motion = motion })
  local k = kit.maker()
  local s = left and 1 or -1                    -- which way the leaf runs from its hinge
  local function at(x, y, z) return V(s * x, y, z) end
  if a.glazed == "glass" then
    k.box("glass", at(W / 2, H / 2, 0), V(W - 0.14, H - 0.2, 0.01), 0.0, GLASS, 1)
    for _, x in ipairs{ 0.04, W - 0.04 } do k.box(part, at(x, H / 2, 0), V(0.07, H - 0.01, T), 0.004, col, 1) end
    for _, y in ipairs{ 0.07, H - 0.05 } do k.box(part, at(W / 2, y, 0), V(W - 0.01, y < 1 and 0.14 or 0.09, T), 0.004, col, 1) end
  else
    k.box(part, at(W / 2, H / 2, 0), V(W - 0.006, H - 0.01, T), 0.004, col, 1)
    if a.glazed == "window" then
      for _, z in ipairs{ -T / 2 - 0.001, T / 2 + 0.001 } do
        k.box("glass", at(W * 0.68, 1.5, z), V(0.16, 0.5, 0.002), 0.0, V(0.25, 0.32, 0.36), 1)
        k.box("paint", at(W * 0.68, 1.5, z * 0.9), V(0.2, 0.54, 0.004), 0.0, FRAME, 1)
      end
    end
  end
  local hx = W - 0.08
  for _, z in ipairs{ -1, 1 } do
    if a.kind == "lever" then
      k.rod("metal", at(hx, 1.0, z * T / 2), at(hx, 1.0, z * (T / 2 + 0.05)), 0.009, kit.CHROME, 8)
      k.rod("metal", at(hx, 1.0, z * (T / 2 + 0.05)), at(hx - 0.12, 1.0, z * (T / 2 + 0.05)), 0.009, kit.CHROME, 8)
    elseif a.kind == "push_bar" then
      k.rod("metal", at(0.1, 1.0, z * (T / 2 + 0.05)), at(W - 0.1, 1.0, z * (T / 2 + 0.05)), 0.014, kit.CHROME, 8)
      for _, x in ipairs{ 0.1, W - 0.1 } do k.rod("metal", at(x, 1.0, z * T / 2), at(x, 1.0, z * (T / 2 + 0.05)), 0.008, kit.CHROME, 6) end
    else
      k.box("metal", at(W - 0.14, 1.1, z * (T / 2 + 0.001)), V(0.16, 0.3, 0.002), 0.0, V(0.7, 0.71, 0.73), 1)     -- a plate to push
      k.box("metal", at(W / 2, 0.14, z * (T / 2 + 0.001)), V(W - 0.02, 0.26, 0.002), 0.0, V(0.7, 0.71, 0.73), 1)  -- and one to kick
    end
  end
  kit.put(m, door, k)
  kit.far_box(m, door, part, { W, H, T }, { s * W / 2, H / 2, 0 }, col, far)
  door:shape("door_" .. i, { box = { W, H, T }, pos = { s * W / 2, H / 2, 0 } })
  door:marker("handle_" .. i, { pos = { s * hx, 1.0, T / 2 + 0.06 }, role = "hold" })
end
return m
