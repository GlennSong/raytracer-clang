-- SPORTS GROUNDS (#191): a school's or a park's pitches and courts, each one flat piece laid on the ground with its
-- lines, and the things that stand on it.
--
--   soccer_pitch      grass with its lines and two goals with nets                         (default 68 x 105 m; a school's is smaller)
--   baseball_diamond  a dirt infield and grass outfield, the bases, the mound, foul lines, a backstop fence behind home
--   tennis_court      a hard court with its lines and a net between two posts (a doubles court, 10.97 x 23.77 m, in its run-off)
--   basketball_court  a hard court with its lines and a hoop on a post at each end
--   pool              a 25 m pool sunk in a tiled deck: water, lane ropes, starting blocks, ladders (`lanes`)
--   running_track     a 200 m oval of red lanes round a grass infield
--   bleachers         a bank of bench seats on a steel frame (`rows`, `length`; markers seat_N)
--   scoreboard        a board on two posts
--
-- Each lies on y = 0 at its own origin, its long side along z. `length` and `width` are the playing area (0: the
-- kind's own).
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind   = { "soccer_pitch", "baseball_diamond", "tennis_court", "basketball_court", "pool", "running_track", "bleachers", "scoreboard" },
  length = { 0, min = 0, max = 130 },
  width  = { 0, min = 0, max = 100 },
  lanes  = { 6, min = 4, max = 10, step = 1 },
  rows   = { 5, min = 2, max = 10, step = 1 },
}
local GRASS, GRASS2, DIRT, WHITE = V(0.2, 0.42, 0.16), V(0.23, 0.47, 0.18), V(0.6, 0.42, 0.28), kit.WHITE
local STEEL = kit.metal.steel
local m = model.new()
local far = m:level(150)
local c = kit.maker()
local T = 0.02                                   -- how thick a ground piece is; lines lie 4 mm over it

local function flat(part, x0, x1, z0, z1, col, y) c.box(part, V((x0 + x1) / 2, (y or T) - T / 2, (z0 + z1) / 2), V(x1 - x0, T, z1 - z0), 0.0, col, 1) end
local function line(x0, z0, x1, z1, w, col)       -- a painted line along x or along z
  w = w or 0.1
  if math.abs(x1 - x0) > math.abs(z1 - z0) then c.box("paint", V((x0 + x1) / 2, T + 0.002, z0), V(math.abs(x1 - x0) + w, 0.004, w), 0.0, col or WHITE, 1)
  else c.box("paint", V(x0, T + 0.002, (z0 + z1) / 2), V(w, 0.004, math.abs(z1 - z0) + w), 0.0, col or WHITE, 1) end
end
local function box_lines(x0, x1, z0, z1, w, col) line(x0, z0, x1, z0, w, col); line(x0, z1, x1, z1, w, col); line(x0, z0, x0, z1, w, col); line(x1, z0, x1, z1, w, col) end
local function ring(x, z, r, w, col, from, to)    -- a painted circle, or an arc of one (radians)
  from, to = from or 0, to or 2 * math.pi
  local n = math.max(6, math.floor(24 * (to - from) / (2 * math.pi)))
  for i = 0, n - 1 do
    local t0, t1 = from + (to - from) * i / n, from + (to - from) * (i + 1) / n
    local tm, len = (t0 + t1) / 2, 2 * r * math.sin((t1 - t0) / 2) + w * 0.5
    c.mesh("paint", mesh.translate(mesh.rotate_y(mesh.box{ len, 0.004, w }, -(tm + math.pi / 2)), { x + r * math.cos(tm), T + 0.002, z + r * math.sin(tm) }), col or WHITE)
  end
end

if a.kind == "soccer_pitch" then
  local W, L = a.width > 0 and a.width or 60, a.length > 0 and a.length or 90
  local M = 4                                    -- the grass round the lines
  for i = 0, 9 do                                -- mown stripes
    flat("turf", -W / 2 - M, W / 2 + M, -L / 2 - M + i * (L + 2 * M) / 10, -L / 2 - M + (i + 1) * (L + 2 * M) / 10, i % 2 == 0 and GRASS or GRASS2)
  end
  box_lines(-W / 2, W / 2, -L / 2, L / 2, 0.12)
  line(-W / 2, 0, W / 2, 0, 0.12)
  ring(0, 0, 9.15, 0.12)
  for _, s in ipairs{ -1, 1 } do
    local z = s * L / 2
    box_lines(-20.16, 20.16, math.min(z, z - s * 16.5), math.max(z, z - s * 16.5), 0.12)
    box_lines(-9.16, 9.16, math.min(z, z - s * 5.5), math.max(z, z - s * 5.5), 0.12)
    c.box("paint", V(0, T + 0.002, z - s * 11), V(0.24, 0.004, 0.24), 0.0, WHITE, 1)
    -- the goal: 7.32 x 2.44 m, a frame with a net behind it
    for _, x in ipairs{ -3.66, 3.66 } do
      c.rod("metal", V(x, 0, z), V(x, 2.44, z), 0.06, WHITE, 8)
      c.rod("metal", V(x, 2.44, z), V(x, 0, z + s * 2), 0.03, WHITE, 6)
    end
    c.rod("metal", V(-3.72, 2.44, z), V(3.72, 2.44, z), 0.06, WHITE, 8)
    c.rod("metal", V(-3.66, 0.03, z + s * 2), V(3.66, 0.03, z + s * 2), 0.03, WHITE, 6)
    c.mesh("mesh", mesh.translate(mesh.rotate_x(mesh.box{ 7.3, 0.004, 3.15 }, s * math.atan(2.44, 2)), { 0, 1.22, z + s * 1 }), V(0.95, 0.95, 0.95))
    for _, x in ipairs{ -3.66, 3.66 } do c.box("mesh", V(x, 0.8, z + s * 0.7), V(0.004, 1.5, 1.3), 0.0, V(0.95, 0.95, 0.95), 1) end
    m:shape("goal_" .. (s < 0 and "south" or "north"), { box = { 7.4, 2.5, 0.3 }, pos = { 0, 1.25, z + s * 1.9 } })
  end
elseif a.kind == "baseball_diamond" then
  local R = a.length > 0 and a.length or 75        -- home plate to the outfield fence
  -- home plate is the origin; the field opens toward +z, the foul lines at 45 degrees either side
  -- the outfield: a square of grass standing on its corner at home plate, R along each foul line
  c.mesh("turf", mesh.translate(mesh.rotate_y(mesh.box{ R, T, R }, math.pi / 4), { 0, T / 2, R * 0.7071 }), GRASS)
  c.mesh("soil", mesh.translate(mesh.rotate_y(mesh.box{ 29, T, 29 }, math.pi / 4), { 0, T / 2 + 0.004, 19.4 }), DIRT)       -- the infield's dirt square
  c.mesh("turf", mesh.translate(mesh.rotate_y(mesh.box{ 20, T, 20 }, math.pi / 4), { 0, T / 2 + 0.008, 19.4 }), GRASS2)     -- and its grass
  c.turned("soil", V(0, T, 18.44), { { 0, 0.25 }, { 0.6, 0.25 }, { 2.7, 0 } }, DIRT * 1.05, nil, 16)                       -- the mound
  c.box("paint", V(0, T + 0.26, 18.44), V(0.6, 0.02, 0.15), 0.0, WHITE, 1)
  c.turned("soil", V(0, T + 0.008, 0), { { 0, 0.004 }, { 4, 0.004 }, { 4, 0 } }, DIRT, nil, 16)                            -- the circle at home
  for _, b in ipairs{ { 19.4, 19.4 }, { 0, 38.8 }, { -19.4, 19.4 } } do c.box("paint", V(b[1], T + 0.04, b[2]), V(0.38, 0.06, 0.38), 0.02, WHITE, 1) end
  c.box("paint", V(0, T + 0.014, 0), V(0.43, 0.01, 0.43), 0.0, WHITE, 1)
  for _, s in ipairs{ -1, 1 } do                    -- the foul lines
    c.mesh("paint", mesh.translate(mesh.rotate_y(mesh.translate(mesh.box{ R, 0.004, 0.1 }, { R / 2, 0, 0 }), -(math.pi / 2 - s * math.pi / 4)), { 0, T + 0.012, 0 }), WHITE)
    c.rod("metal", V(s * R * 0.707, 0, R * 0.707), V(s * R * 0.707, 6, R * 0.707), 0.05, V(0.95, 0.85, 0.1), 6)           -- a foul pole
    -- the outfield fence, from the foul pole round to dead centre
    local len = R
    c.mesh("mesh", mesh.translate(mesh.rotate_y(mesh.translate(mesh.box{ len, 2.4, 0.004 }, { len / 2, 0, 0 }), -(math.pi / 2 + s * math.pi / 4)), { s * R * 0.707, 1.2, R * 0.707 }), V(0.2, 0.35, 0.25))
    m:shape("outfield_" .. (s < 0 and "left" or "right"), { box = { len, 2.4, 0.1 }, pos = { s * R * 0.3536, 1.2, R * 1.0607 }, turn = { 0, math.deg(-(math.pi / 2 + s * math.pi / 4)), 0 } })
  end
  -- the backstop: a tall chain-link screen bent round behind home
  for i, p in ipairs{ { -7, -1.5, -4, -6 }, { -4, -6, 4, -6 }, { 4, -6, 7, -1.5 } } do
    local dx, dz = p[3] - p[1], p[4] - p[2]
    local len, ang = math.sqrt(dx * dx + dz * dz), math.atan(-dz, dx)
    c.mesh("mesh", mesh.translate(mesh.rotate_y(mesh.translate(mesh.box{ len, 5, 0.004 }, { len / 2, 0, 0 }), ang), { p[1], 2.5, p[2] }), V(0.62, 0.65, 0.66))
    c.rod("metal", V(p[1], 0, p[2]), V(p[1], 5, p[2]), 0.04, STEEL, 6); c.rod("metal", V(p[3], 0, p[4]), V(p[3], 5, p[4]), 0.04, STEEL, 6)
    c.rod("metal", V(p[1], 5, p[2]), V(p[3], 5, p[4]), 0.03, STEEL, 6)
    m:shape("backstop_" .. i, { box = { len, 5, 0.1 }, pos = { (p[1] + p[3]) / 2, 2.5, (p[2] + p[4]) / 2 }, turn = { 0, math.deg(ang), 0 } })
  end
  for _, s in ipairs{ -1, 1 } do                    -- a bench for each side
    c.box("wood", V(s * 12, 0.45, -3), V(6, 0.06, 0.4), 0.01, kit.wood("oak"), 1)
    for _, x in ipairs{ -2.5, 2.5 } do c.box("metal", V(s * 12 + x, 0.22, -3), V(0.06, 0.44, 0.36), 0.0, STEEL, 1) end
  end
elseif a.kind == "tennis_court" then
  local W, L, RX, RZ = 10.97, 23.77, 3.66, 6.4
  flat("court", -W / 2 - RX, W / 2 + RX, -L / 2 - RZ, L / 2 + RZ, V(0.2, 0.42, 0.3))
  flat("court", -W / 2, W / 2, -L / 2, L / 2, V(0.22, 0.36, 0.6), T + 0.001)
  box_lines(-W / 2, W / 2, -L / 2, L / 2, 0.06)
  for _, x in ipairs{ -4.115, 4.115 } do line(x, -L / 2, x, L / 2, 0.05) end
  for _, z in ipairs{ -6.4, 6.4 } do line(-4.115, z, 4.115, z, 0.05) end
  line(0, -6.4, 0, 6.4, 0.05)
  for _, x in ipairs{ -W / 2 - 0.91, W / 2 + 0.91 } do c.rod("metal", V(x, 0, 0), V(x, 1.07, 0), 0.04, V(0.1, 0.25, 0.15), 8) end
  c.box("mesh", V(0, 0.5, 0), V(W + 1.82, 0.9, 0.006), 0.0, V(0.1, 0.1, 0.1), 1)
  c.box("fabric", V(0, 0.96, 0), V(W + 1.82, 0.06, 0.012), 0.0, WHITE, 1)
  m:shape("net", { box = { W + 1.9, 1.0, 0.06 }, pos = { 0, 0.5, 0 } })
elseif a.kind == "basketball_court" then
  local W, L = 15, 28
  flat("court", -W / 2 - 2, W / 2 + 2, -L / 2 - 2, L / 2 + 2, V(0.3, 0.32, 0.35))
  box_lines(-W / 2, W / 2, -L / 2, L / 2, 0.06)
  line(-W / 2, 0, W / 2, 0, 0.06); ring(0, 0, 1.8, 0.06)
  for _, s in ipairs{ -1, 1 } do
    local z = s * L / 2
    box_lines(-2.45, 2.45, math.min(z, z - s * 5.8), math.max(z, z - s * 5.8), 0.06)
    ring(0, z - s * 5.8, 1.8, 0.06, WHITE, s > 0 and math.pi or 0, s > 0 and 2 * math.pi or math.pi)
    ring(0, z - s * 1.575, 6.75, 0.06, WHITE, s > 0 and math.pi * 1.07 or math.pi * 0.07, s > 0 and math.pi * 1.93 or math.pi * 0.93)
    c.rod("metal", V(0, 0, z + s * 1.2), V(0, 3.0, z + s * 1.2), 0.07, STEEL, 8)
    c.rod("metal", V(0, 3.0, z + s * 1.2), V(0, 3.3, z - s * 1.0), 0.06, STEEL, 8)
    c.box("hard", V(0, 3.4, z - s * 1.2), V(1.8, 1.05, 0.04), 0.0, WHITE, 1)
    c.mesh("metal", mesh.translate(mesh.lathe{ profile = { { 0.215, 0 }, { 0.235, 0.01 }, { 0.215, 0.02 } }, segments = 14 }, { 0, 3.05, z - s * 1.45 }), V(0.9, 0.35, 0.1))
    c.mesh("mesh", mesh.translate(mesh.lathe{ profile = { { 0.16, 0 }, { 0.22, 0.4 } }, segments = 10 }, { 0, 2.65, z - s * 1.45 }), WHITE)
    m:shape("hoop_" .. (s < 0 and "south" or "north"), { box = { 0.2, 3.0, 0.2 }, pos = { 0, 1.5, z + s * 1.2 } })
  end
elseif a.kind == "pool" then
  local lanes = math.floor(a.lanes + 0.5)
  local W, L, DK, DEPTH = lanes * 2.5, 25, 4, 1.4
  -- the deck is a frame round the basin; the basin's floor and walls are below y = 0 (the level's ground must have
  -- a hole there, or the pool stands raised: this one is RAISED, its deck `DEPTH` up, with steps to it)
  local Y = DEPTH + 0.1
  for _, s in ipairs{ -1, 1 } do
    c.box("court", V(s * (W / 2 + DK / 2), Y / 2, 0), V(DK, Y, L + 2 * DK), 0.0, V(0.78, 0.8, 0.8), 1)
    c.box("court", V(0, Y / 2, s * (L / 2 + DK / 2)), V(W, Y, DK), 0.0, V(0.78, 0.8, 0.8), 1)
  end
  c.box("ceramic", V(0, 0.05, 0), V(W, 0.1, L), 0.0, V(0.55, 0.8, 0.9), 1)                           -- the basin's floor
  c.box("water", V(0, Y - 0.2, 0), V(W, 0.02, L), 0.0, V(0.2, 0.55, 0.75), 1)
  for i = 1, lanes do
    local x = -W / 2 + (i - 0.5) * 2.5
    c.box("ceramic", V(x, 0.102, 0), V(0.25, 0.004, L - 4), 0.0, V(0.05, 0.1, 0.3), 1)               -- the lane's line on the floor
    c.box("hard", V(x, Y + 0.25, -L / 2 - 0.35), V(0.5, 0.5, 0.6), 0.03, V(0.9, 0.9, 0.92), 1)       -- a starting block
    c.tilted("rubber", V(x, Y + 0.52, -L / 2 - 0.33), V(0.5, 0.04, 0.6), 0.01, 0.15, V(0.1, 0.3, 0.6))
    if i < lanes then
      for j = 0, 24 do c.rod("hard", V(x + 1.25, Y - 0.18, -L / 2 + j * L / 25), V(x + 1.25, Y - 0.18, -L / 2 + (j + 0.9) * L / 25), 0.05, j % 2 == 0 and V(0.9, 0.2, 0.15) or WHITE, 6) end
    end
  end
  for _, p in ipairs{ { -W / 2, -L / 2 + 2 }, { W / 2, -L / 2 + 2 }, { -W / 2, L / 2 - 2 }, { W / 2, L / 2 - 2 } } do   -- the ladders
    local s = p[1] < 0 and 1 or -1
    for _, dz in ipairs{ -0.25, 0.25 } do
      c.pipe("metal", { V(p[1] - s * 0.5, Y, p[2] + dz), V(p[1] - s * 0.5, Y + 0.9, p[2] + dz), V(p[1] + s * 0.15, Y + 0.9, p[2] + dz), V(p[1] + s * 0.15, Y - 1.0, p[2] + dz) }, 0.02, kit.CHROME, { bend = 0.12, sides = 6 })
    end
    for k = 1, 3 do c.rod("metal", V(p[1] + s * 0.15, Y - k * 0.3, p[2] - 0.25), V(p[1] + s * 0.15, Y - k * 0.3, p[2] + 0.25), 0.015, kit.CHROME, 6) end
  end
  for k = 0, 6 do c.box("court", V(0, (k + 0.5) * Y / 7, L / 2 + DK + 0.3 * (7 - k) - 0.15), V(4, Y / 7, 0.3), 0.0, V(0.72, 0.74, 0.74), 1) end   -- the steps up
  for _, s in ipairs{ -1, 1 } do
    m:shape("deck_side_" .. (s < 0 and "west" or "east"), { box = { DK, Y, L + 2 * DK }, pos = { s * (W / 2 + DK / 2), Y / 2, 0 } })
    m:shape("deck_end_" .. (s < 0 and "south" or "north"), { box = { W, Y, DK }, pos = { 0, Y / 2, s * (L / 2 + DK / 2) } })
  end
  m:shape("floor", { box = { W, 0.1, L }, pos = { 0, 0.05, 0 } })
  for k = 0, 6 do m:shape("step_" .. k, { box = { 4, Y / 7, 0.3 }, pos = { 0, (k + 0.5) * Y / 7, L / 2 + DK + 0.3 * (7 - k) - 0.15 } }) end
  m:marker("water", { pos = { 0, Y - 0.2, 0 }, role = "stand", size = W })
elseif a.kind == "running_track" then
  local R, S, LN = 18, 43, 4                       -- the bends' inner radius, the straights' length, lanes
  local TW = LN * 1.22
  flat("turf", -R, R, -S / 2, S / 2, GRASS)
  local RED = V(0.62, 0.22, 0.16)
  for _, s in ipairs{ -1, 1 } do
    flat("court", s > 0 and R or -R - TW, s > 0 and R + TW or -R, -S / 2, S / 2, RED)
    for l = 0, LN do line(s * (R + l * 1.22), -S / 2, s * (R + l * 1.22), S / 2, 0.05) end
    local n = 16
    for i = 0, n - 1 do
      local t0, t1 = math.pi * i / n, math.pi * (i + 1) / n
      local tm = (t0 + t1) / 2
      local rm = R + TW / 2
      local len = 2 * (R + TW) * math.sin((t1 - t0) / 2) + 0.05
      c.mesh("court", mesh.translate(mesh.rotate_y(mesh.box{ len, T, TW }, -(s * tm + math.pi / 2)), { rm * math.cos(tm), T / 2, s * (S / 2 + rm * math.sin(tm)) }), RED)
      c.mesh("turf", mesh.translate(mesh.rotate_y(mesh.box{ 2 * R * math.sin((t1 - t0) / 2) + 0.05, T, R }, -(s * tm + math.pi / 2)), { R / 2 * math.cos(tm), T / 2 - 0.001, s * (S / 2 + R / 2 * math.sin(tm)) }), GRASS)
    end
    for l = 0, LN do ring(0, s * S / 2, R + l * 1.22, 0.05, WHITE, s > 0 and 0 or math.pi, s > 0 and math.pi or 2 * math.pi) end
  end
  line(R, -S / 2 + 5, R + TW, -S / 2 + 5, 0.1)     -- the finish
elseif a.kind == "bleachers" then
  local rows, L = math.floor(a.rows + 0.5), a.length > 0 and a.length or 8
  local wood = kit.wood("pine")
  local seat = 0
  for r = 1, rows do
    local y, z = 0.45 + (r - 1) * 0.4, -(r - 1) * 0.7
    c.box("wood", V(0, y, z), V(L, 0.05, 0.28), 0.008, wood, 1)
    c.box("metal", V(0, y - 0.38, z + 0.32), V(L, 0.03, 0.3), 0.0, STEEL * 0.9, 1)             -- the foot board
    m:shape("row_" .. r, { box = { L, 0.1, 0.7 }, pos = { 0, y - 0.05, z } })
    for i = 1, math.max(1, math.floor(L / 2)) do
      seat = seat + 1
      m:marker("seat_" .. seat, { pos = { -L / 2 + (i - 0.5) * L / math.max(1, math.floor(L / 2)), y + 0.03, z + 0.03 }, role = "seat" })
    end
  end
  local top, back = 0.45 + (rows - 1) * 0.4, -(rows - 1) * 0.7
  for i = 0, math.max(1, math.floor(L / 2.5)) do
    local x = -L / 2 + i * L / math.max(1, math.floor(L / 2.5))
    c.rod("metal", V(x, 0, 0.3), V(x, top, back - 0.2), 0.03, STEEL, 6)
    c.rod("metal", V(x, 0, back - 0.2), V(x, top + 1.0, back - 0.2), 0.03, STEEL, 6)
    c.rod("metal", V(x, 0, 0.3), V(x, 0, back - 0.2), 0.03, STEEL, 6)
  end
  for _, y in ipairs{ top + 0.5, top + 1.0 } do c.rod("metal", V(-L / 2, y, back - 0.2), V(L / 2, y, back - 0.2), 0.025, STEEL, 6) end
else   -- scoreboard
  for _, x in ipairs{ -1.6, 1.6 } do c.rod("metal", V(x, 0, 0), V(x, 5, 0), 0.07, STEEL, 8) end
  c.box("hard", V(0, 4.0, 0), V(4.6, 2.2, 0.2), 0.02, V(0.08, 0.1, 0.12), 1)
  c.box("paint", V(0, 4.85, 0.102), V(4.2, 0.3, 0.004), 0.0, WHITE, 1)
  for _, x in ipairs{ -1.3, 1.3 } do c.box("light", V(x, 3.9, 0.102), V(0.9, 0.9, 0.004), 0.0, V(1.0, 0.75, 0.15), 1) end
  c.box("light", V(0, 3.35, 0.102), V(0.5, 0.4, 0.004), 0.0, V(1.0, 0.2, 0.1), 1)
  m:shape("board", { box = { 4.6, 5, 0.3 }, pos = { 0, 2.5, 0 } })
end
kit.put(m, m, c)
return m
