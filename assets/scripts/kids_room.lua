-- A CHILD'S ROOM (#191): what a child's bedroom or a playroom holds.
--
--   bed          a low single bed with a bright duvet and a pillow (marker `lie`, role "lie")
--   bunk_bed     two beds one over the other with a ladder and a guard rail (markers `lie_1` below, `lie_2` above)
--   desk         a small desk with a pinboard behind it and a child's chair (marker `seat`)
--   toy_shelf    a low cubby shelf, its cubbies holding boxes, blocks, a ball and a teddy (its open side to +z)
--   play_mat     a road-map rug to lay on the floor, with a few toy cars on it
--   play_table   a low round table with two little stools and crayons on it (markers `seat_1`, `seat_2`)
--   dollhouse    a wooden house whose front swings open (`front`)
--   easel        a child's painting easel, a picture clipped to it
--   blocks       a tumbled pile of building blocks
--
-- Each stands on y = 0 at its own origin, its front toward +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind  = { "bed", "bunk_bed", "desk", "toy_shelf", "play_mat", "play_table", "dollhouse", "easel", "blocks" },
  paint = { "blue", "red", "yellow", "green", "orange" },
  wood  = kit.wood_names,
}
local wood, paint = kit.wood(a.wood), kit.paint[a.paint]
local P = kit.paint
local WHITE = kit.WHITE
local BRIGHT = { P.red, P.yellow, P.blue, P.green, P.orange }
local m = model.new()
local far = m:level(35)
local c = kit.maker()
local function ball(part, x, y, z, r, col, sy) c.mesh(part, mesh.translate(mesh.scale(kit.ball(r, 8), { 1, sy or 1, 1 }), { x, y, z }), col) end
-- a mattress with its duvet and pillow, the head at -z, lying on y
local function bedding(y, W, L)
  c.cushion("fabric", V(0, y + 0.07, 0), V(W, 0.14, L), WHITE * 0.95, { r = 0.04 })
  c.cushion("fabric", V(0, y + 0.16, 0.2), V(W + 0.02, 0.07, L - 0.5), paint, { r = 0.05 })
  c.cushion("fabric", V(0, y + 0.18, -L / 2 + 0.22), V(W * 0.6, 0.08, 0.3), WHITE, { r = 0.05 })
  for i = -1, 1 do ball("fabric", i * W * 0.25, y + 0.2, 0.3, 0.06, i == 0 and WHITE or P.yellow, 0.15) end     -- spots on the duvet
end

if a.kind == "bed" then
  local W, L = 0.9, 1.7
  c.box("wood", V(0, 0.2, 0), V(W + 0.06, 0.16, L + 0.06), 0.01, wood, 1)
  c.box("wood", V(0, 0.42, -L / 2 - 0.02), V(W + 0.06, 0.6, 0.04), 0.02, wood, 1)
  c.box("wood", V(0, 0.28, L / 2 + 0.02), V(W + 0.06, 0.32, 0.04), 0.02, wood, 1)
  for _, x in ipairs{ -W / 2, W / 2 } do for _, z in ipairs{ -L / 2, L / 2 } do c.box("wood", V(x, 0.06, z), V(0.06, 0.12, 0.06), 0.004, wood * 0.9, 1) end end
  bedding(0.28, W, L)
  m:marker("lie", { pos = { 0, 0.46, -L / 2 + 0.3 }, role = "lie", size = L })      -- (the head, its +z toward the feet)
  m:shape("bed", { box = { W + 0.06, 0.45, L + 0.06 }, pos = { 0, 0.225, 0 } })
elseif a.kind == "bunk_bed" then
  local W, L, H = 0.9, 1.9, 1.75
  for _, x in ipairs{ -W / 2 - 0.03, W / 2 + 0.03 } do for _, z in ipairs{ -L / 2, L / 2 } do c.box("wood", V(x, H / 2, z), V(0.07, H, 0.07), 0.006, wood, 1) end end
  for i, y in ipairs{ 0.3, 1.25 } do
    c.box("wood", V(0, y - 0.05, 0), V(W + 0.06, 0.1, L), 0.006, wood, 1)
    for _, z in ipairs{ -L / 2, L / 2 } do c.box("wood", V(0, y + 0.2, z), V(W + 0.06, 0.34, 0.03), 0.006, wood, 1) end
    bedding(y, W, L - 0.08)
    m:marker("lie_" .. i, { pos = { 0, y + 0.18, -L / 2 + 0.34 }, role = "lie", size = L })
    m:shape("bunk_" .. i, { box = { W + 0.06, 0.25, L }, pos = { 0, y, 0 } })
  end
  c.box("wood", V(0, 1.55, -L / 2 + 0.01), V(W, 0.08, 0.03), 0.006, wood, 1)
  for _, y in ipairs{ 1.45, 1.6 } do c.box("wood", V(W / 2 + 0.03, y, -0.2), V(0.03, 0.06, L - 0.5), 0.006, wood, 1) end      -- the guard rail
  for _, z in ipairs{ L / 2 - 0.45, L / 2 - 0.08 } do c.box("wood", V(W / 2 + 0.07, 0.8, z), V(0.04, 1.6, 0.05), 0.004, wood * 0.92, 1) end   -- the ladder
  for i = 1, 4 do c.rod("wood", V(W / 2 + 0.07, i * 0.33, L / 2 - 0.45), V(W / 2 + 0.07, i * 0.33, L / 2 - 0.08), 0.016, wood * 0.92, 6) end
  kit.far_box(m, m, "wood", { W, H, L }, { 0, H / 2, 0 }, wood, far)
elseif a.kind == "desk" then
  c.box("wood", V(0, 0.56, -0.05), V(0.9, 0.03, 0.5), 0.008, wood, 1)
  for _, x in ipairs{ -0.42, 0.42 } do c.box("paint", V(x, 0.28, -0.05), V(0.03, 0.55, 0.46), 0.006, paint, 1) end
  c.box("paint", V(0.25, 0.47, -0.05), V(0.3, 0.14, 0.42), 0.006, paint, 1)                       -- a drawer
  c.box("fabric", V(0, 0.95, -0.29), V(0.86, 0.6, 0.02), 0.004, V(0.7, 0.55, 0.35), 1)             -- the pinboard
  for i, n in ipairs{ { -0.25, 1.05, P.yellow }, { 0.05, 0.9, WHITE }, { 0.28, 1.08, P.red } } do c.box("hard", V(n[1], n[2], -0.278), V(0.16, 0.2, 0.002), 0.0, n[3], 1) end
  c.turned("ceramic", V(-0.3, 0.575, -0.15), { { 0, 0 }, { 0.035, 0 }, { 0.04, 0.09 }, { 0, 0.09 } }, P.blue, nil, 8)   -- a pot of pencils
  for i = 1, 4 do c.rod("wood", V(-0.3 + (i - 2.5) * 0.012, 0.6, -0.15), V(-0.3 + (i - 2.5) * 0.03, 0.74, -0.15 + (i - 2.5) * 0.01), 0.004, BRIGHT[i], 4) end
  c.box("hard", V(0.1, 0.578, 0), V(0.21, 0.004, 0.28), 0.0, WHITE, 1)
  -- the chair, turned to face the desk
  c.box("paint", V(0, 0.3, 0.42), V(0.3, 0.025, 0.3), 0.01, paint, 1)
  for _, x in ipairs{ -0.13, 0.13 } do for _, z in ipairs{ 0.29, 0.55 } do c.rod("metal", V(x, 0, z), V(x, 0.3, z), 0.012, kit.metal.steel, 6) end end
  for _, x in ipairs{ -0.13, 0.13 } do c.rod("metal", V(x, 0.3, 0.55), V(x, 0.6, 0.57), 0.012, kit.metal.steel, 6) end
  c.box("paint", V(0, 0.52, 0.565), V(0.3, 0.16, 0.02), 0.01, paint, 1)
  m:marker("seat", { pos = { 0, 0.33, 0.42 }, turn = { 0, 180, 0 }, role = "seat", size = 0.3 })
  m:shape("desk", { box = { 0.9, 0.6, 0.5 }, pos = { 0, 0.3, -0.05 } })
elseif a.kind == "toy_shelf" then
  local W, H, D = 1.2, 0.8, 0.32
  c.box("wood", V(0, H / 2, 0.006), V(W, H, 0.012), 0.0, wood * 0.9, 1)
  for i = 0, 2 do c.box("wood", V(0, i * (H - 0.02) / 2 + 0.01, D / 2), V(W, 0.02, D), 0.004, wood, 1) end
  for i = 0, 3 do c.box("wood", V(-W / 2 + 0.01 + i * (W - 0.02) / 3, H / 2, D / 2), V(0.02, H, D), 0.004, wood, 1) end
  local cw, ch = (W - 0.02) / 3, (H - 0.02) / 2
  for row = 0, 1 do for col = 0, 2 do
    local x, y = -W / 2 + 0.01 + (col + 0.5) * cw, 0.02 + row * ch
    local n = row * 3 + col
    if n == 0 or n == 4 then c.box("fabric", V(x, y + 0.13, D / 2), V(cw - 0.06, 0.26, D - 0.06), 0.02, BRIGHT[n % 5 + 1], 1)            -- a fabric box
    elseif n == 1 then for i = 1, 5 do c.box("hard", V(x - 0.12 + i * 0.045, y + 0.12, D / 2), V(0.035, 0.24, 0.2), 0.0, BRIGHT[i], 1) end   -- picture books
    elseif n == 2 then ball("rubber", x, y + 0.12, D / 2, 0.11, P.red)
    elseif n == 3 then                                                                                                                -- a teddy
      ball("fabric", x, y + 0.09, D / 2, 0.09, V(0.6, 0.42, 0.25), 1.1); ball("fabric", x, y + 0.22, D / 2, 0.065, V(0.6, 0.42, 0.25))
      for _, s in ipairs{ -1, 1 } do ball("fabric", x + s * 0.05, y + 0.28, D / 2, 0.025, V(0.6, 0.42, 0.25)); ball("fabric", x + s * 0.1, y + 0.1, D / 2 + 0.03, 0.035, V(0.6, 0.42, 0.25)) end
    else for i = 0, 5 do c.box("hard", V(x - 0.1 + (i % 3) * 0.1, y + 0.04 + (i // 3) * 0.08, D / 2), V(0.08, 0.08, 0.08), 0.004, BRIGHT[i % 5 + 1], 1) end end
  end end
  -- (built with its back at z = 0; stood with its middle on the origin, as a piece a room program places is)
  local body = m:node("body", { pos = { 0, 0, -D / 2 } })
  kit.put(m, body, c)
  body:shape("shelf", { box = { W, H, D }, pos = { 0, H / 2, D / 2 } })
  return m
elseif a.kind == "play_mat" then
  local W, D = 2.0, 1.4
  c.box("fabric", V(0, 0.006, 0), V(W, 0.012, D), 0.0, V(0.35, 0.6, 0.3), 1)
  local ROAD = V(0.25, 0.25, 0.27)
  c.box("fabric", V(0, 0.013, 0), V(W - 0.3, 0.002, 0.2), 0.0, ROAD, 1)
  for _, x in ipairs{ -0.55, 0.55 } do c.box("fabric", V(x, 0.013, 0), V(0.2, 0.002, D - 0.2), 0.0, ROAD, 1) end
  for i = -3, 3 do c.box("fabric", V(i * 0.22, 0.0145, 0), V(0.1, 0.001, 0.015), 0.0, WHITE, 1) end
  for i, p in ipairs{ { -0.85, 0.45, P.red }, { 0.85, -0.45, P.blue }, { 0.05, 0.5, P.yellow } } do c.box("fabric", V(p[1], 0.013, p[2]), V(0.22, 0.002, 0.22), 0.0, p[3], 1) end   -- little houses' plots
  for i, p in ipairs{ { -0.2, 0.0, P.red }, { 0.55, 0.3, P.blue }, { -0.55, -0.35, P.yellow } } do                  -- toy cars
    c.box("hard", V(p[1], 0.04, p[2]), V(0.12, 0.035, 0.06), 0.01, p[3], 1)
    c.box("hard", V(p[1] - 0.01, 0.065, p[2]), V(0.06, 0.03, 0.05), 0.01, p[3] * 0.8, 1)
  end
elseif a.kind == "play_table" then
  c.turned("wood", V(0, 0.45, 0), { { 0, 0.03 }, { 0.42, 0.03 }, { 0.42, 0 }, { 0, 0 } }, wood, nil, 20)
  c.turned("paint", V(0, 0, 0), { { 0, 0 }, { 0.2, 0 }, { 0.05, 0.05 }, { 0.04, 0.45 }, { 0, 0.45 } }, paint, nil, 12)
  for i, col in ipairs(BRIGHT) do c.rod("hard", V(-0.1 + i * 0.03, 0.488, 0.05), V(-0.06 + i * 0.03, 0.488, 0.13), 0.006, col, 5) end
  c.box("hard", V(0.12, 0.482, -0.08), V(0.21, 0.004, 0.28), 0.0, WHITE, 1)
  for i, s in ipairs{ -1, 1 } do
    c.turned("paint", V(s * 0.62, 0, 0), { { 0, 0 }, { 0.14, 0 }, { 0.12, 0.26 }, { 0.15, 0.28 }, { 0, 0.3 } }, BRIGHT[i + 1], nil, 12)
    m:marker("seat_" .. i, { pos = { s * 0.62, 0.3, 0 }, turn = { 0, s * -90, 0 }, role = "seat", size = 0.28 })
    m:shape("stool_" .. i, { box = { 0.28, 0.3, 0.28 }, pos = { s * 0.62, 0.15, 0 } })
  end
  m:shape("table", { box = { 0.6, 0.48, 0.6 }, pos = { 0, 0.24, 0 } })
elseif a.kind == "dollhouse" then
  local W, H, D, R = 0.8, 0.7, 0.36, 0.28
  c.box("wood", V(0, 0.02, 0), V(W + 0.1, 0.04, D + 0.1), 0.004, P.green * 0.8, 1)
  c.box("wood", V(0, 0.04 + H / 2, -D / 2), V(W, H, 0.012), 0.0, wood, 1)
  for _, x in ipairs{ -W / 2, 0, W / 2 } do c.box("wood", V(x, 0.04 + H / 2, 0), V(0.012, H, D), 0.0, wood, 1) end
  c.box("wood", V(0, 0.04 + H / 2, 0), V(W, 0.012, D), 0.0, wood, 1)
  for _, s in ipairs{ -1, 1 } do c.mesh("paint", mesh.translate(mesh.rotate_z(mesh.box{ W / 2 + 0.08, 0.015, D + 0.06 }, s * -math.atan(R, W / 2)), { s * W / 4, 0.04 + H + R / 2, 0 }), P.red) end
  for i, p in ipairs{ { -0.2, 0.08, P.blue }, { 0.2, 0.08, P.yellow }, { -0.2, 0.43, P.orange }, { 0.2, 0.43, P.green } } do c.box("hard", V(p[1], p[2] + 0.04, -0.05), V(0.16, 0.08, 0.1), 0.01, p[3], 1) end   -- its little furniture
  kit.put(m, m, c)
  local front = m:node("front", { pos = { -W / 2, 0.04, D / 2 }, motion = { turn = { 0, -1, 0 }, min = 0, max = 120 } })
  local k = kit.maker()
  k.box("paint", V(W / 2, H / 2, 0), V(W, H, 0.012), 0.0, P.yellow * 0.95, 1)
  for _, p in ipairs{ { 0.2, 0.52 }, { 0.6, 0.52 }, { 0.6, 0.18 } } do k.box("glass", V(p[1], p[2], 0.007), V(0.16, 0.16, 0.003), 0.0, V(0.6, 0.8, 0.95), 1) end
  k.box("paint", V(0.2, 0.15, 0.007), V(0.14, 0.28, 0.003), 0.0, P.red, 1)
  kit.put(m, front, k)
  front:marker("handle", { pos = { W - 0.05, H / 2, 0.02 }, role = "hold" })
  m:shape("house", { box = { W, H + R, D }, pos = { 0, (H + R) / 2, 0 } })
  return m
elseif a.kind == "easel" then
  for _, s in ipairs{ -1, 1 } do c.rod("wood", V(s * 0.3, 0, 0.18), V(s * 0.06, 1.2, 0), 0.016, wood, 6) end
  c.rod("wood", V(0, 0, -0.3), V(0, 1.2, 0), 0.016, wood, 6)
  c.mesh("hard", mesh.translate(mesh.rotate_x(mesh.box{ 0.6, 0.5, 0.012 }, -0.15), { 0, 0.78, 0.09 }), WHITE)
  c.mesh("wood", mesh.translate(mesh.rotate_x(mesh.box{ 0.66, 0.03, 0.08 }, -0.15), { 0, 0.5, 0.15 }), wood)
  for i, col in ipairs{ P.red, P.blue, P.yellow } do
    c.mesh("hard", mesh.translate(mesh.rotate_x(mesh.box{ 0.12 + i * 0.05, 0.1, 0.002 }, -0.15), { -0.12 + i * 0.08, 0.7 + i * 0.07, 0.1 }), col)
    c.turned("ceramic", V(-0.24 + i * 0.12, 0.52, 0.16), { { 0, 0 }, { 0.03, 0 }, { 0.03, 0.06 }, { 0, 0.06 } }, col, nil, 8)
  end
  m:shape("easel", { box = { 0.66, 1.2, 0.5 }, pos = { 0, 0.6, -0.05 } })
else   -- blocks
  local spots = { { 0, 0.04, 0, 0 }, { 0.1, 0.04, 0.03, 0.4 }, { 0.05, 0.12, 0.01, 0.2 }, { -0.14, 0.04, 0.1, 1.0 }, { -0.05, 0.04, -0.16, 0.7 },
                  { 0.2, 0.04, -0.12, 0.3 }, { 0.05, 0.2, 0.015, 0.9 }, { -0.22, 0.04, -0.08, 1.3 } }
  for i, s in ipairs(spots) do c.mesh("hard", mesh.translate(mesh.rotate_y(mesh.rounded_box({ 0.08, 0.08, 0.08 }, 0.008, { segs = 1 }), s[4]), { s[1], s[2], s[3] }), BRIGHT[i % 5 + 1]) end
  c.mesh("hard", mesh.translate(mesh.rotate_x(mesh.lathe{ profile = { { 0, -0.04 }, { 0.04, -0.04 }, { 0.04, 0.04 }, { 0, 0.04 } }, segments = 10 }, math.pi / 2), { 0.3, 0.04, 0.12 }), P.red)
  c.mesh("hard", mesh.translate(mesh.lathe{ profile = { { 0, 0 }, { 0.05, 0 }, { 0, 0.09 } }, segments = 4 }, { -0.3, 0, 0.12 }), P.green)
end
kit.put(m, m, c)
return m
