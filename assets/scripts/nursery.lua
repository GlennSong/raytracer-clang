-- A CHILD'S ROOM, each piece with a part that moves (#191):
--
--   cradle          a slatted cradle hung between two posts; it rocks from side to side (`rock`)
--   rocking_horse   a wooden horse on two rockers; the whole horse rocks (`rock`)
--   toy_chest       a painted chest; its lid lifts at the back (`lid`), with blocks and a ball inside
--   mobile          arms with shapes on threads, turning slowly (`arms`). It HANGS: its origin is its top
--
-- The others stand on y = 0 at their own origin, facing +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind  = { "cradle", "rocking_horse", "toy_chest", "mobile" },
  wood  = kit.wood_names,
  paint = kit.paints,
}
local wood, paint = kit.wood(a.wood), kit.paint[a.paint]
local m = model.new()
local far = m:level(25)
local c = kit.maker()

if a.kind == "cradle" then
  local L, TOP = 0.95, 0.78
  for _, z in ipairs{ -L / 2 - 0.06, L / 2 + 0.06 } do
    c.post("wood", V(0, 0.03, z), V(0, TOP + 0.04, z), 0.045, 0.05, wood)
    c.box("wood", V(0, 0.02, z), V(0.6, 0.04, 0.06), 0.012, wood, 2)                    -- the foot
  end
  c.box("wood", V(0, 0.09, 0), V(0.04, 0.03, L + 0.12), 0.008, wood, 1)                 -- the stretcher between the feet
  kit.put(m, m, c)
  for i, z in ipairs{ -L / 2 - 0.06, L / 2 + 0.06 } do m:shape("post_" .. i, { box = { 0.6, TOP, 0.06 }, pos = { 0, TOP / 2, z } }) end
  local rock = m:node("rock", { pos = { 0, TOP, 0 }, motion = { turn = { 0, 0, 1 }, min = -18, max = 18 } })
  local k = kit.maker()
  local W, Y0, Y1 = 0.5, -0.5, -0.12
  k.box("wood", V(0, Y0, 0), V(W, 0.02, L), 0.006, wood, 1)
  for _, z in ipairs{ -L / 2 + 0.01, L / 2 - 0.01 } do
    k.box("wood", V(0, (Y0 + Y1) / 2 + 0.03, z), V(W, Y1 - Y0 + 0.06, 0.02), 0.008, wood, 1)
    k.rod("metal", V(0, 0, z), V(0, Y1, z), 0.006, kit.STEEL, 6)                        -- what it hangs by
  end
  for _, x in ipairs{ -W / 2 + 0.01, W / 2 - 0.01 } do
    k.box("wood", V(x, Y1, 0), V(0.025, 0.03, L), 0.008, wood, 1)
    for i = 1, 9 do k.rod("wood", V(x, Y0, -L / 2 + i * L / 10), V(x, Y1, -L / 2 + i * L / 10), 0.008, wood, 6) end
  end
  k.cushion("fabric", V(0, Y0 + 0.04, 0), V(W - 0.05, 0.06, L - 0.05), kit.LINEN, { r = 0.04 })
  k.cushion("fabric", V(0, Y0 + 0.09, 0.08), V(W - 0.07, 0.04, L * 0.6), kit.fabric("sage"), { r = 0.04, crown = 0.02 })
  kit.put(m, rock, k)
  kit.far_box(m, rock, "wood", { W, Y1 - Y0, L }, { 0, (Y0 + Y1) / 2, 0 }, wood, far)
  rock:shape("basket", { box = { W, Y1 - Y0, L }, pos = { 0, (Y0 + Y1) / 2, 0 } })
  rock:marker("lie", { pos = { 0, Y0 + 0.08, 0 }, role = "lie", size = 0.8 })
  rock:marker("push", { pos = { W / 2, Y1, 0 }, role = "hold" })
elseif a.kind == "rocking_horse" then
  local RR = 1.0
  local rock = m:node("rock", { pos = { 0, RR, 0 }, motion = { turn = { 1, 0, 0 }, min = -16, max = 16 } })
  local k = kit.maker()
  local function at(x, y, z) return V(x, y - RR, z) end
  for _, x in ipairs{ -0.16, 0.16 } do
    local pts = {}
    for i = -4, 4 do local t = math.rad(i * 8); pts[#pts + 1] = V(x, -RR * math.cos(t) + 0.02, RR * math.sin(t)) end
    k.bent("wood", pts, 0.03, 0.04, paint, { count = 16 })
    k.post("wood", at(x, 0.05, 0.32), at(x * 0.6, 0.5, 0.24), 0.05, 0.035, wood)        -- the legs, splayed
    k.post("wood", at(x, 0.05, -0.32), at(x * 0.6, 0.5, -0.24), 0.05, 0.035, wood)
  end
  for _, z in ipairs{ -0.3, 0.3 } do k.box("wood", at(0, 0.045, z), V(0.36, 0.025, 0.05), 0.006, paint, 1) end
  k.cushion("wood", at(0, 0.58, 0), V(0.2, 0.2, 0.62), wood, { r = 0.09, roll = 0.08 })                   -- the barrel
  k.bent("wood", { at(0, 0.62, 0.24), at(0, 0.78, 0.33), at(0, 0.92, 0.36) }, 0.1, 0.14, wood, { count = 8, up = { 0, 0, 1 } })   -- the neck
  k.cushion("wood", at(0, 0.93, 0.44), V(0.09, 0.11, 0.26), wood, { r = 0.04, tilt = 0.5 })               -- the head
  for _, x in ipairs{ -0.03, 0.03 } do k.box("wood", at(x, 1.02, 0.36), V(0.02, 0.06, 0.02), 0.006, wood, 1) end   -- the ears
  k.rod("wood", at(-0.14, 0.9, 0.36), at(0.14, 0.9, 0.36), 0.012, paint, 8)                               -- the handle
  k.cushion("fabric", at(0, 0.69, -0.03), V(0.2, 0.03, 0.26), paint, { r = 0.06 })                        -- the saddle
  k.bent("wood", { at(0, 0.66, -0.3), at(0, 0.6, -0.4), at(0, 0.42, -0.42) }, 0.04, 0.04, kit.BLACK * 2, { count = 8, scale = { 1, 0.4 } })   -- the tail
  kit.put(m, rock, k)
  kit.far_box(m, rock, "wood", { 0.36, 0.7, 0.8 }, { 0, 0.4 - RR, 0 }, wood, far)
  rock:shape("body", { box = { 0.36, 0.7, 0.8 }, pos = { 0, 0.35 - RR, 0 }, friction = 0.8 })
  rock:marker("seat", { pos = { 0, 0.71 - RR, -0.03 }, role = "seat", size = 0.25 })
  rock:marker("hold", { pos = { 0, 0.9 - RR, 0.36 }, role = "hold" })
elseif a.kind == "mobile" then
  local arms = m:node("arms", { pos = { 0, 0, 0 }, motion = { turn = { 0, 1, 0 } } })
  local k = kit.maker()
  k.rod("hard", V(0, 0, 0), V(0, -0.2, 0), 0.002, kit.WHITE, 4)
  local tints = { kit.paint.red, kit.paint.blue, kit.paint.yellow, kit.paint.green, kit.paint.orange }
  for i = 0, 1 do
    local cs, sn = math.cos(i * math.pi / 2), math.sin(i * math.pi / 2)
    k.rod("wood", V(-0.2 * cs, -0.2, -0.2 * sn), V(0.2 * cs, -0.2, 0.2 * sn), 0.004, wood, 6)
    for j, s in ipairs{ -1, 1 } do
      local p, drop = V(s * 0.2 * cs, -0.2, s * 0.2 * sn), 0.15 + ((i * 2 + j) % 3) * 0.05
      k.rod("hard", p, p + V(0, -drop, 0), 0.001, kit.WHITE, 3)
      local tint = tints[i * 2 + j]
      if (i + j) % 2 == 0 then k.mesh("paint", mesh.translate(kit.ball(0.035), { p[1], p[2] - drop - 0.03, p[3] }), tint)
      else k.box("paint", p + V(0, -drop - 0.03, 0), V(0.06, 0.06, 0.012), 0.004, tint, 1) end
    end
  end
  k.mesh("paint", mesh.translate(kit.ball(0.04), { 0, -0.42, 0 }), tints[5])
  k.rod("hard", V(0, -0.2, 0), V(0, -0.38, 0), 0.001, kit.WHITE, 3)
  kit.put(m, arms, k)
else
  -- a box open at the top: four sides and a floor
  local W, H, D, T = 0.8, 0.45, 0.42, 0.02
  local b = kit.maker()
  for _, x in ipairs{ -W / 2 + T / 2, W / 2 - T / 2 } do b.box("paint", V(x, H / 2, 0), V(T, H, D), 0.006, paint, 1) end
  for _, z in ipairs{ -D / 2 + T / 2, D / 2 - T / 2 } do b.box("paint", V(0, H / 2, z), V(W - 2 * T, H, T), 0.006, paint, 1) end
  b.box("paint", V(0, 0.03, 0), V(W - 2 * T, T, D - 2 * T), 0.0, paint, 1)
  local tints = { kit.paint.red, kit.paint.blue, kit.paint.yellow, kit.paint.green, kit.paint.orange }
  for i = 0, 7 do b.box("wood", V(-0.28 + (i % 4) * 0.17, 0.3 + (i // 4) * 0.012, -0.08 + (i // 4) * 0.15 + (i % 2) * 0.03), V(0.09, 0.09, 0.09), 0.008, tints[i % 5 + 1], 1) end
  b.mesh("rubber", mesh.translate(kit.ball(0.1, 16), { 0.2, 0.33, -0.06 }), tints[1])
  b.box("hard", V(0, 0.2, 0), V(W - 2 * T, 0.12, D - 2 * T), 0.0, V(0.3, 0.25, 0.2), 1)  -- what lies deeper, in shadow
  kit.put(m, m, b)
  kit.far_box(m, m, "paint", { W, H, D }, { 0, H / 2, 0 }, paint, far)
  for i, x in ipairs{ -W / 2 + T / 2, W / 2 - T / 2 } do m:shape("end_" .. i, { box = { T, H, D }, pos = { x, H / 2, 0 } }) end
  for i, z in ipairs{ -D / 2 + T / 2, D / 2 - T / 2 } do m:shape("side_" .. i, { box = { W, H, T }, pos = { 0, H / 2, z } }) end
  local lid = m:node("lid", { pos = { 0, H, -D / 2 }, motion = { turn = { -1, 0, 0 }, min = 0, max = 95 } })
  local k = kit.maker()
  k.box("paint", V(0, 0.012, D / 2 + 0.005), V(W + 0.02, 0.024, D + 0.02), 0.008, paint * 0.85, 1)
  kit.put(m, lid, k)
  lid:shape("lid", { box = { W, 0.024, D }, pos = { 0, 0.012, D / 2 }, friction = 0.8 })
  lid:marker("lid", { pos = { 0, 0.012, D + 0.02 }, role = "hold" })
  lid:marker("seat", { pos = { 0, 0.024, D / 2 }, role = "seat", size = 0.6 })
end
return m
