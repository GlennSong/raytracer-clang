-- THINGS FOR A QUAD, A SQUARE OR A SCHOOL'S GROUNDS (#191):
--
--   statue        a bronze figure on a stone plinth with a plaque
--   sundial       a stone pedestal with a brass dial and its gnomon
--   notice_board  a glazed board on two posts under a little roof, with notices pinned in it
--   stone_bench   a slab seat on two stone blocks (markers `seat_1`, `seat_2`, role "seat")
--   planter       a square stone planter with a clipped shrub
--   bike_shelter  a curved roof on steel hoops over a row of bike stands: `count` stands
--   clock_post    a four-faced clock on a cast-iron post (a meeting place)
--
-- Each stands on y = 0 at its own origin, its front toward +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind  = { "statue", "sundial", "notice_board", "stone_bench", "planter", "bike_shelter", "clock_post" },
  count = { 5, min = 2, max = 12, step = 1 },
  wood  = kit.wood_names,
}
local STONE, BRONZE, IRON, LEAF = V(0.72, 0.7, 0.65), V(0.3, 0.42, 0.36), kit.metal.iron, V(0.15, 0.35, 0.14)
local m = model.new()
local far = m:level(60)
local c = kit.maker()
local function ball(part, x, y, z, r, col, sy) c.mesh(part, mesh.translate(mesh.scale(kit.ball(r, 8), { 1, sy or 1, 1 }), { x, y, z }), col) end

if a.kind == "statue" then
  c.box("stone", V(0, 0.1, 0), V(1.6, 0.2, 1.6), 0.0, STONE * 0.9, 1)
  c.box("stone", V(0, 0.9, 0), V(1.0, 1.4, 1.0), 0.02, STONE, 1)
  c.box("stone", V(0, 1.65, 0), V(1.2, 0.1, 1.2), 0.02, STONE * 0.95, 1)
  c.box("metal", V(0, 1.0, 0.505), V(0.6, 0.35, 0.01), 0.0, kit.metal.brass, 1)
  local y = 1.7                                                       -- the figure: boots to hat, one arm raised with a book
  for _, x in ipairs{ -0.11, 0.11 } do c.turned("metal", V(x, y, 0), { { 0, 0 }, { 0.08, 0 }, { 0.07, 0.45 }, { 0.09, 0.85 }, { 0, 0.85 } }, BRONZE, nil, 8) end
  c.turned("metal", V(0, y + 0.8, 0), { { 0, 0 }, { 0.2, 0 }, { 0.24, 0.2 }, { 0.19, 0.6 }, { 0.08, 0.68 }, { 0, 0.68 } }, BRONZE, 0.7, 10)
  c.turned("metal", V(0, y + 0.55, 0), { { 0.22, 0.3 }, { 0.3, 0 } }, BRONZE * 0.9, 0.75, 10)   -- the coat's skirt
  ball("metal", 0, y + 1.62, 0, 0.12, BRONZE, 1.15)
  c.rod("metal", V(-0.22, y + 1.4, 0), V(-0.3, y + 0.95, 0.05), 0.05, BRONZE, 6)
  c.rod("metal", V(0.22, y + 1.4, 0), V(0.42, y + 1.55, 0.2), 0.05, BRONZE, 6)
  c.rod("metal", V(0.42, y + 1.55, 0.2), V(0.36, y + 1.8, 0.3), 0.045, BRONZE, 6)
  c.box("metal", V(0.36, y + 1.84, 0.32), V(0.16, 0.04, 0.22), 0.004, BRONZE * 1.1, 1)
  m:shape("plinth", { box = { 1.2, 1.7, 1.2 }, pos = { 0, 0.85, 0 } })
  m:shape("figure", { box = { 0.7, 1.9, 0.5 }, pos = { 0, 2.65, 0 } })
elseif a.kind == "sundial" then
  c.turned("stone", V(0, 0, 0), { { 0, 0 }, { 0.32, 0 }, { 0.32, 0.08 }, { 0.2, 0.14 }, { 0.13, 0.5 }, { 0.16, 0.86 }, { 0.3, 0.94 }, { 0.3, 1.0 }, { 0, 1.0 } }, STONE, nil, 12)
  c.turned("metal", V(0, 1.0, 0), { { 0, 0.012 }, { 0.26, 0.012 }, { 0.26, 0 } }, kit.metal.brass, nil, 20)
  c.mesh("metal", mesh.translate(mesh.rotate_z(mesh.rotate_x(mesh.box{ 0.006, 0.2, 0.28 }, 0), 0), { 0, 1.07, -0.02 }), kit.metal.brass * 0.9)
  c.mesh("metal", mesh.translate(mesh.rotate_x(mesh.box{ 0.008, 0.008, 0.36 }, -0.7), { 0, 1.13, -0.02 }), kit.metal.brass)
  m:shape("dial", { box = { 0.6, 1.2, 0.6 }, pos = { 0, 0.6, 0 } })
elseif a.kind == "notice_board" then
  local wood = kit.wood(a.wood)
  for _, x in ipairs{ -0.75, 0.75 } do c.box("wood", V(x, 1.05, 0), V(0.1, 2.1, 0.1), 0.008, wood, 1) end
  c.box("wood", V(0, 1.45, 0), V(1.4, 1.0, 0.06), 0.004, wood * 0.85, 1)
  c.box("fabric", V(0, 1.45, 0.032), V(1.28, 0.88, 0.004), 0.0, V(0.25, 0.4, 0.3), 1)
  local notes = { { -0.4, 1.65, 0.3, 0.4, kit.WHITE }, { 0.0, 1.6, 0.25, 0.3, V(0.95, 0.9, 0.5) }, { 0.38, 1.62, 0.3, 0.42, kit.WHITE * 0.95 },
                  { -0.35, 1.2, 0.36, 0.26, V(0.6, 0.8, 0.95) }, { 0.15, 1.2, 0.3, 0.3, V(0.95, 0.7, 0.7) } }
  for _, n in ipairs(notes) do c.box("hard", V(n[1], n[2], 0.036), V(n[3], n[4], 0.002), 0.0, n[5], 1) end
  c.box("glass", V(0, 1.45, 0.05), V(1.3, 0.9, 0.004), 0.0, V(0.9, 0.95, 0.95), 1)
  c.mesh("wood", mesh.translate(mesh.rotate_x(mesh.box{ 1.9, 0.03, 0.3 }, 0.45), { 0, 2.16, 0.11 }), wood * 0.7)
  c.mesh("wood", mesh.translate(mesh.rotate_x(mesh.box{ 1.9, 0.03, 0.3 }, -0.45), { 0, 2.16, -0.11 }), wood * 0.7)
  m:shape("board", { box = { 1.6, 2.2, 0.12 }, pos = { 0, 1.1, 0 } })
  m:marker("read", { pos = { 0, 0, 0.8 }, turn = { 0, 180, 0 }, role = "stand" })
elseif a.kind == "stone_bench" then
  for _, x in ipairs{ -0.7, 0.7 } do c.box("stone", V(x, 0.2, 0), V(0.28, 0.4, 0.46), 0.02, STONE * 0.92, 1) end
  c.box("stone", V(0, 0.45, 0), V(1.9, 0.1, 0.52), 0.02, STONE, 1)
  m:shape("bench", { box = { 1.9, 0.5, 0.52 }, pos = { 0, 0.25, 0 } })
  for i, x in ipairs{ -0.45, 0.45 } do m:marker("seat_" .. i, { pos = { x, 0.5, 0.05 }, role = "seat" }) end
elseif a.kind == "planter" then
  c.box("stone", V(0, 0.3, 0), V(1.0, 0.6, 1.0), 0.03, STONE, 1)
  c.box("stone", V(0, 0.61, 0), V(1.08, 0.06, 1.08), 0.02, STONE * 0.94, 1)
  c.box("soil", V(0, 0.62, 0), V(0.86, 0.04, 0.86), 0.0, V(0.2, 0.14, 0.1), 1)
  ball("leaf", 0, 1.05, 0, 0.46, LEAF, 1.0)
  for i = 0, 5 do ball("leaf", 0.3 * math.cos(i * 1.05), 0.74, 0.3 * math.sin(i * 1.05), 0.14, i % 2 == 0 and V(0.8, 0.25, 0.3) or V(0.9, 0.8, 0.3), 0.7) end
  m:shape("planter", { box = { 1.08, 1.5, 1.08 }, pos = { 0, 0.75, 0 } })
elseif a.kind == "bike_shelter" then
  local n = math.floor(a.count + 0.5)
  local L, D, H = n * 0.7 + 0.4, 2.1, 2.2
  local arc = {}
  for i = 0, 8 do
    local t = math.pi * 0.5 * i / 8
    arc[#arc + 1] = { -D / 2 + D * (1 - math.cos(t)) * 0.9, H * (0.72 + 0.28 * math.sin(t)) }
  end
  for _, x in ipairs{ -L / 2, 0, L / 2 } do
    c.rod("metal", V(x, 0, -D / 2), V(x, H * 0.72, -D / 2), 0.03, kit.metal.steel, 6)
    for i = 1, #arc - 1 do c.rod("metal", V(x, arc[i][2], arc[i][1]), V(x, arc[i + 1][2], arc[i + 1][1]), 0.03, kit.metal.steel, 6) end
  end
  for i = 1, #arc - 1 do
    local p, q = arc[i], arc[i + 1]
    local len, ang = math.sqrt((q[1] - p[1]) ^ 2 + (q[2] - p[2]) ^ 2), -math.atan(q[2] - p[2], q[1] - p[1])
    c.mesh("glass", mesh.translate(mesh.rotate_x(mesh.box{ L + 0.2, 0.008, len + 0.01 }, ang), { 0, (p[2] + q[2]) / 2 + 0.035, (p[1] + q[1]) / 2 }), V(0.7, 0.82, 0.85))
  end
  for i = 1, n do
    local x = -L / 2 + 0.55 + (i - 1) * 0.7
    c.pipe("metal", { V(x, 0, -0.35), V(x, 0.8, -0.35), V(x, 0.8, 0.35), V(x, 0, 0.35) }, 0.022, kit.metal.steel, { bend = 0.15, sides = 6 })
  end
  m:shape("back", { box = { L, H * 0.72, 0.08 }, pos = { 0, H * 0.36, -D / 2 } })
else   -- clock_post
  local H = 4.2
  c.turned("metal", V(0, 0, 0), { { 0, 0 }, { 0.22, 0 }, { 0.22, 0.1 }, { 0.12, 0.2 }, { 0.1, 0.9 }, { 0.06, 1.05 }, { 0.055, H - 0.75 }, { 0.12, H - 0.7 }, { 0, H - 0.7 } }, IRON, nil, 10)
  c.box("metal", V(0, H - 0.3, 0), V(0.7, 0.8, 0.7), 0.04, IRON, 2)
  for i = 0, 3 do
    local t = i * math.pi / 2
    local nx, nz = math.sin(t), math.cos(t)
    c.mesh("hard", mesh.translate(mesh.rotate_y(mesh.rotate_x(mesh.lathe{ profile = { { 0, 0 }, { 0.3, 0 }, { 0.3, 0.01 }, { 0, 0.012 } }, segments = 20 }, math.pi / 2), t), { nx * 0.351, H - 0.3, nz * 0.351 }), kit.WHITE)
    c.mesh("metal", mesh.translate(mesh.rotate_y(mesh.translate(mesh.box{ 0.02, 0.2, 0.006 }, { 0, 0.09, 0.366 }), t), { 0, H - 0.3, 0 }), V(0.05, 0.05, 0.05))
    c.mesh("metal", mesh.translate(mesh.rotate_y(mesh.translate(mesh.rotate_z(mesh.translate(mesh.box{ 0.014, 0.27, 0.006 }, { 0, 0.125, 0 }), -1.9), { 0, 0, 0.368 }), t), { 0, H - 0.3, 0 }), V(0.05, 0.05, 0.05))
  end
  c.turned("metal", V(0, H + 0.1, 0), { { 0.4, 0 }, { 0.1, 0.16 }, { 0.03, 0.4 }, { 0, 0.4 } }, IRON, nil, 4)
  m:shape("post", { box = { 0.4, H + 0.3, 0.4 }, pos = { 0, (H + 0.3) / 2, 0 } })
end
kit.put(m, m, c)
return m
