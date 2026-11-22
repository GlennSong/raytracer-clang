-- A BARBER'S (#191): what stands in the shop.
--
--   chair        the barber's chair: a padded seat on a hydraulic column with a footrest. The seat pumps up (`seat`,
--                0 to 0.2 m) and its back reclines for a shave (`back`); marker `seat` (role "seat")
--   station      a mirror on the wall over a shelf of bottles, combs and a jar, with a drawer unit under it
--                (its back to -z, to stand against a wall; a drawer slides, `drawer`)
--   backwash     a reclined chair at a basin, for washing hair (marker `seat`)
--   waiting      a row of three linked seats with a low magazine table at one end (markers `seat_1` ... `seat_3`)
--   hood_dryer   a dryer hood on a wheeled stand; the hood tips (`hood`)
--   trolley      a small wheeled trolley of trays with tools in them
--   towel_shelf  a wall shelf unit of rolled towels (its back to -z; hang it at 1.2 m)
--
-- Each stands on y = 0 at its own origin, its front toward +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind    = { "chair", "station", "backwash", "waiting", "hood_dryer", "trolley", "towel_shelf" },
  leather = { "leather_black", "leather_oxblood", "leather_tan", "leather_cream" },
  wood    = kit.wood_names,
}
local hide, hide_part = kit.upholstery(a.leather)
local wood = kit.wood(a.wood)
local CHROME, WHITE, DARK = kit.CHROME, kit.WHITE, kit.BLACK * 2
local m = model.new()
local far = m:level(40)
local c = kit.maker()

if a.kind == "chair" then
  c.turned("metal", V(0, 0, 0), { { 0, 0 }, { 0.32, 0 }, { 0.32, 0.03 }, { 0.1, 0.07 }, { 0.07, 0.3 }, { 0, 0.3 } }, CHROME, nil, 16)
  kit.put(m, m, c)
  local seat = m:node("seat", { pos = { 0, 0.3, 0 }, motion = { slide = { 0, 1, 0 }, min = 0, max = 0.2 } })
  local k = kit.maker()
  k.rod("metal", V(0, 0, 0), V(0, 0.2, 0), 0.05, CHROME, 12)
  k.box("metal", V(0, 0.22, 0), V(0.5, 0.05, 0.5), 0.02, CHROME, 2)
  k.cushion(hide_part, V(0, 0.31, 0.02), V(0.52, 0.12, 0.5), hide, { r = 0.06 })
  for _, s in ipairs{ -1, 1 } do                                           -- the arms
    k.rod("metal", V(s * 0.28, 0.24, -0.15), V(s * 0.28, 0.5, -0.15), 0.015, CHROME, 6)
    k.rod("metal", V(s * 0.28, 0.24, 0.15), V(s * 0.28, 0.5, 0.15), 0.015, CHROME, 6)
    k.cushion(hide_part, V(s * 0.28, 0.53, 0), V(0.08, 0.05, 0.42), hide, { r = 0.02 })
  end
  for _, s in ipairs{ -1, 1 } do k.rod("metal", V(s * 0.16, 0.2, 0.25), V(s * 0.16, -0.08, 0.6), 0.014, CHROME, 6) end
  k.box("metal", V(0, -0.09, 0.63), V(0.42, 0.02, 0.18), 0.006, CHROME, 1)   -- the footrest
  kit.put(m, seat, k)
  seat:marker("seat", { pos = { 0, 0.38, 0.05 }, role = "seat", size = 0.5 })
  seat:shape("seat", { box = { 0.6, 0.3, 0.55 }, pos = { 0, 0.3, 0 } })
  local back = seat:node("back", { pos = { 0, 0.34, -0.24 }, motion = { turn = { -1, 0, 0 }, min = 0, max = 50 } })
  local b = kit.maker()
  b.cushion(hide_part, V(0, 0.3, -0.03), V(0.48, 0.56, 0.1), hide, { r = 0.06, face = "z" })
  b.rod("metal", V(0, 0.58, -0.03), V(0, 0.72, -0.03), 0.012, CHROME, 6)
  b.cushion(hide_part, V(0, 0.78, -0.02), V(0.26, 0.14, 0.09), hide, { r = 0.04, face = "z" })
  kit.put(m, back, b)
  back:shape("back", { box = { 0.5, 0.9, 0.12 }, pos = { 0, 0.45, -0.03 } })
  kit.far_box(m, m, hide_part, { 0.6, 1.2, 0.6 }, { 0, 0.6, 0 }, hide, far)
elseif a.kind == "station" then
  c.box("wood", V(0, 0.4, 0.22), V(1.0, 0.8, 0.42), 0.006, wood, 1)
  c.box("stone", V(0, 0.82, 0.24), V(1.1, 0.04, 0.48), 0.006, V(0.2, 0.2, 0.22), 1)
  c.box("wood", V(0, 1.55, 0.02), V(0.9, 1.3, 0.04), 0.01, wood * 0.9, 1)
  c.box("screen", V(0, 1.55, 0.042), V(0.8, 1.2, 0.004), 0.0, V(0.75, 0.82, 0.85), 1)       -- the mirror's glass
  for i, p in ipairs{ { -0.4, 0.07, V(0.2, 0.5, 0.75) }, { -0.3, 0.09, V(0.85, 0.75, 0.2) }, { -0.2, 0.06, V(0.2, 0.6, 0.35) }, { 0.38, 0.08, V(0.8, 0.25, 0.25) } } do
    c.turned("ceramic", V(p[1], 0.84, 0.12), { { 0, 0 }, { 0.03, 0 }, { 0.03, p[2] * 1.5 }, { 0.012, p[2] * 1.8 }, { 0.012, p[2] * 2.2 }, { 0, p[2] * 2.2 } }, p[3], nil, 8)
  end
  c.turned("glass", V(0.22, 0.84, 0.16), { { 0, 0 }, { 0.05, 0 }, { 0.05, 0.16 }, { 0, 0.16 } }, V(0.4, 0.7, 0.9), nil, 10)   -- the jar of combs
  for i = 1, 3 do c.box("hard", V(0.2 + i * 0.012, 0.95, 0.16), V(0.004, 0.14, 0.03), 0.0, DARK, 1) end
  c.box("fabric", V(0.0, 0.855, 0.36), V(0.3, 0.02, 0.18), 0.004, WHITE, 1)                 -- a folded towel
  kit.put(m, m, c)
  local drawer = m:node("drawer", { pos = { 0, 0.62, 0.43 }, motion = { slide = { 0, 0, 1 }, min = 0, max = 0.3 } })
  local k = kit.maker()
  k.box("wood", V(0, 0, 0.01), V(0.9, 0.22, 0.02), 0.004, wood * 1.05, 1)
  k.rod("metal", V(-0.1, 0, 0.03), V(0.1, 0, 0.03), 0.01, CHROME, 6)
  kit.put(m, drawer, k)
  drawer:marker("handle", { pos = { 0, 0, 0.05 }, role = "hold" })
  m:shape("unit", { box = { 1.1, 0.86, 0.48 }, pos = { 0, 0.43, 0.24 } })
  kit.far_box(m, m, "wood", { 1.0, 2.2, 0.4 }, { 0, 1.1, 0.2 }, wood, far)
elseif a.kind == "backwash" then
  c.turned("ceramic", V(0, 0, -0.55), { { 0, 0 }, { 0.2, 0 }, { 0.14, 0.1 }, { 0.12, 0.72 }, { 0, 0.72 } }, WHITE, nil, 12)
  c.cage("ceramic", V(0, 0.72, -0.5), { { 0.5, 0.42, 0.1, 0 }, { 0.6, 0.5, 0.14, 0.2 }, { 0.54, 0.44, 0.12, 0.2 }, { 0.44, 0.36, 0.08, 0.04 } }, WHITE, { level = 0 })
  c.rod("metal", V(0.15, 0.92, -0.72), V(0.15, 1.05, -0.72), 0.012, CHROME, 6)
  c.rod("metal", V(0.15, 1.05, -0.72), V(0.05, 1.02, -0.55), 0.012, CHROME, 6)
  c.box("metal", V(0, 0.2, 0.15), V(0.5, 0.4, 0.55), 0.03, DARK, 2)
  c.cushion(hide_part, V(0, 0.46, 0.15), V(0.52, 0.12, 0.55), hide, { r = 0.06 })
  c.mesh(hide_part, mesh.translate(mesh.rotate_x(mesh.rounded_box({ 0.5, 0.6, 0.1 }, 0.04, { segs = 2 }), -0.5), { 0, 0.72, -0.2 }), hide)
  kit.put(m, m, c)
  m:marker("seat", { pos = { 0, 0.53, 0.18 }, role = "seat", size = 0.5 })
  m:shape("chair", { box = { 0.6, 0.9, 1.3 }, pos = { 0, 0.45, -0.2 } })
elseif a.kind == "waiting" then
  for _, x in ipairs{ -0.9, 0, 0.9 } do
    c.rod("metal", V(x, 0, -0.2), V(x, 0.4, -0.2), 0.02, CHROME, 6); c.rod("metal", V(x, 0, 0.2), V(x, 0.4, 0.2), 0.02, CHROME, 6)
    c.rod("metal", V(x, 0.02, -0.2), V(x, 0.02, 0.2), 0.02, CHROME, 6)
  end
  c.box("metal", V(0, 0.4, 0), V(1.9, 0.04, 0.06), 0.004, CHROME, 1)
  for i = 1, 3 do
    local x = (i - 2) * 0.6
    c.cushion(hide_part, V(x, 0.47, 0.02), V(0.52, 0.09, 0.48), hide, { r = 0.05 })
    c.cushion(hide_part, V(x, 0.78, -0.24), V(0.52, 0.42, 0.08), hide, { r = 0.05, face = "z" })
    m:marker("seat_" .. i, { pos = { x, 0.52, 0.05 }, role = "seat", size = 0.5 })
  end
  c.box("wood", V(1.3, 0.38, 0), V(0.5, 0.03, 0.5), 0.008, wood, 1)
  for _, x in ipairs{ 1.1, 1.5 } do for _, z in ipairs{ -0.2, 0.2 } do c.rod("wood", V(x, 0, z), V(x, 0.37, z), 0.015, wood * 0.9, 6) end end
  for i, col in ipairs{ V(0.8, 0.2, 0.2), V(0.2, 0.4, 0.7), V(0.9, 0.8, 0.3) } do c.tilted("hard", V(1.3 + (i - 2) * 0.05, 0.4 + i * 0.006, (i - 2) * 0.03), V(0.21, 0.005, 0.28), 0.0, 0, col) end
  kit.put(m, m, c)
  m:shape("bench", { box = { 1.9, 0.95, 0.5 }, pos = { 0, 0.47, 0 } })
  m:shape("table", { box = { 0.5, 0.4, 0.5 }, pos = { 1.3, 0.2, 0 } })
elseif a.kind == "hood_dryer" then
  for i = 0, 4 do
    local t = i * 2 * math.pi / 5
    c.rod("metal", V(0, 0.08, 0), V(0.28 * math.cos(t), 0.04, 0.28 * math.sin(t)), 0.014, CHROME, 6)
    kit.caster(c, V(0.28 * math.cos(t), 0, 0.28 * math.sin(t)), 0.025)
  end
  c.rod("metal", V(0, 0.08, 0), V(0, 1.45, 0), 0.02, CHROME, 8)
  c.rod("metal", V(0, 1.45, 0), V(0, 1.5, 0.18), 0.02, CHROME, 8)
  kit.put(m, m, c)
  local hood = m:node("hood", { pos = { 0, 1.5, 0.18 }, motion = { turn = { 1, 0, 0 }, min = -20, max = 45 } })
  local k = kit.maker()
  k.mesh("hard", mesh.translate(mesh.scale(kit.ball(0.24, 12), { 1, 0.9, 1.15 }), { 0, 0, 0.16 }), WHITE)
  k.turned("hard", V(0, -0.22, 0.16), { { 0.2, 0 }, { 0.22, 0.03 } }, V(0.3, 0.3, 0.32), nil, 12)
  kit.put(m, hood, k)
  m:shape("stand", { box = { 0.5, 1.7, 0.6 }, pos = { 0, 0.85, 0.1 } })
elseif a.kind == "trolley" then
  for _, x in ipairs{ -0.18, 0.18 } do for _, z in ipairs{ -0.15, 0.15 } do
    c.rod("metal", V(x, 0.05, z), V(x, 0.85, z), 0.01, CHROME, 6); kit.caster(c, V(x, 0, z), 0.025)
  end end
  for i, y in ipairs{ 0.2, 0.5, 0.82 } do
    c.box("hard", V(0, y, 0), V(0.4, 0.03, 0.34), 0.01, DARK, 1)
    for _, z in ipairs{ -0.17, 0.17 } do c.box("hard", V(0, y + 0.04, z), V(0.4, 0.06, 0.01), 0.0, DARK, 1) end
  end
  c.rod("hard", V(-0.1, 0.86, 0), V(0.08, 0.86, 0.05), 0.018, V(0.8, 0.2, 0.2), 6)               -- a dryer, a brush, clippers
  c.box("hard", V(0.12, 0.86, -0.08), V(0.05, 0.03, 0.16), 0.008, V(0.15, 0.15, 0.16), 1)
  c.turned("ceramic", V(-0.12, 0.535, 0.05), { { 0, 0 }, { 0.04, 0 }, { 0.04, 0.14 }, { 0, 0.14 } }, V(0.3, 0.6, 0.8), nil, 8)
  kit.put(m, m, c)
  m:shape("trolley", { box = { 0.42, 0.9, 0.36 }, pos = { 0, 0.45, 0 } })
else   -- towel_shelf
  c.box("wood", V(0, 0.4, 0.01), V(0.8, 0.8, 0.02), 0.004, wood * 0.9, 1)
  for _, y in ipairs{ 0.02, 0.4, 0.78 } do c.box("wood", V(0, y, 0.14), V(0.8, 0.03, 0.26), 0.004, wood, 1) end
  for _, x in ipairs{ -0.39, 0.39 } do c.box("wood", V(x, 0.4, 0.14), V(0.02, 0.8, 0.26), 0.004, wood, 1) end
  for row = 0, 1 do for i = 1, 5 do
    c.rod("fabric", V(-0.4 + i * 0.133, 0.1 + row * 0.38, 0.03), V(-0.4 + i * 0.133, 0.1 + row * 0.38, 0.25), 0.058, (i + row) % 2 == 0 and WHITE or V(0.75, 0.82, 0.9), 8)
  end end
  kit.put(m, m, c)
  m:shape("shelf", { box = { 0.8, 0.8, 0.28 }, pos = { 0, 0.4, 0.14 } })
end
return m
