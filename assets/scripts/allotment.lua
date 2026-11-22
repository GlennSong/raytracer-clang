-- THINGS FOR A VEGETABLE GARDEN (#191): what stands in an allotment or a community garden.
--
--   raised_bed    a timber-framed bed of soil with rows of a crop: `crop` = cabbage, lettuce, carrot, tomato, or bare
--   compost_bin   a slatted timber bin, its lid lifting at the back (`lid`)
--   tool_shed     a small boarded shed with a pitched felt roof; its door opens (`door`)
--   greenhouse    a small glass house on a low wall, with a ridge roof; its door opens (`door`)
--   cold_frame    a low glazed box for seedlings; its lid lifts (`lid`)
--   water_butt    a rain barrel on a stand, with a tap
--   wheelbarrow   a steel tray on one wheel and two legs
--   bean_poles    a row of crossed canes with beans climbing them
--   scarecrow     a figure of straw in an old coat and hat, on a post
--
-- Each stands on y = 0 at its own origin, its front toward +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind   = { "raised_bed", "compost_bin", "tool_shed", "greenhouse", "cold_frame", "water_butt", "wheelbarrow", "bean_poles", "scarecrow" },
  crop   = { "cabbage", "lettuce", "carrot", "tomato", "bare" },
  length = { 2.4, min = 1.0, max = 4.0 },      -- raised_bed, bean_poles (metres)
  wood   = kit.wood_names,
  paint  = { "green", "red", "blue", "yellow", "orange", "white" },
}
local wood, paint = kit.wood(a.wood), kit.paint[a.paint]
local SOIL, LEAF, PALE, STRAW = V(0.2, 0.14, 0.1), V(0.14, 0.36, 0.13), V(0.42, 0.6, 0.24), V(0.78, 0.66, 0.36)
local STEEL = kit.metal.steel
local m = model.new()
local far = m:level(45)
local c = kit.maker()
local function ball(part, x, y, z, r, col, sy) c.mesh(part, mesh.translate(mesh.scale(kit.ball(r, 6), { 1, sy or 1, 1 }), { x, y, z }), col) end
local function door_on(node, W, H, col)          -- a boarded leaf hung at x = 0, opening toward +z
  local k = kit.maker()
  k.box("wood", V(W / 2, H / 2, 0), V(W - 0.01, H - 0.01, 0.03), 0.004, col, 1)
  for _, y in ipairs{ H * 0.2, H * 0.8 } do k.box("wood", V(W / 2, y, 0.02), V(W - 0.06, 0.08, 0.02), 0.003, col * 0.85, 1) end
  k.rod("metal", V(W - 0.1, H * 0.5, 0.015), V(W - 0.1, H * 0.5, 0.06), 0.012, kit.metal.iron, 6)
  kit.put(m, node, k)
  node:shape("door", { box = { W, H, 0.04 }, pos = { W / 2, H / 2, 0 } })
  node:marker("handle", { pos = { W - 0.1, H * 0.5, 0.07 }, role = "hold" })
end

if a.kind == "raised_bed" then
  local L, W, H = a.length, 1.2, 0.3
  for _, z in ipairs{ -W / 2, W / 2 } do c.box("wood", V(0, H / 2, z), V(L, H, 0.04), 0.004, wood, 1) end
  for _, x in ipairs{ -L / 2, L / 2 } do c.box("wood", V(x, H / 2, 0), V(0.04, H, W), 0.004, wood, 1) end
  for _, x in ipairs{ -L / 2, L / 2 } do for _, z in ipairs{ -W / 2, W / 2 } do c.box("wood", V(x, (H + 0.04) / 2, z), V(0.07, H + 0.04, 0.07), 0.004, wood * 0.9, 1) end end
  c.box("soil", V(0, H - 0.06, 0), V(L - 0.08, 0.04, W - 0.08), 0.0, SOIL, 1)
  local top = H - 0.04
  local rows, step = (a.crop == "tomato") and 1 or 3, (a.crop == "cabbage" or a.crop == "tomato") and 0.45 or 0.22
  for r = 1, rows do
    local z = rows == 1 and 0 or (r - 2) * 0.36
    local n = math.max(1, math.floor((L - 0.3) / step))
    for i = 1, n do
      local x = -L / 2 + 0.15 + (i - 0.5) * (L - 0.3) / n
      if a.crop == "cabbage" then
        ball("leaf", x, top + 0.07, z, 0.15, PALE, 0.6); ball("leaf", x, top + 0.1, z, 0.09, PALE * 1.2, 0.9)
      elseif a.crop == "lettuce" then
        ball("leaf", x, top + 0.04, z, 0.09, (i + r) % 3 == 0 and V(0.45, 0.2, 0.25) or PALE * 1.15, 0.6)
      elseif a.crop == "carrot" then
        c.mesh("leaf", mesh.translate(mesh.lathe{ profile = { { 0, 0 }, { 0.05, 0.12 }, { 0, 0.2 } }, segments = 5 }, { x, top, z }), LEAF * 1.2)
      elseif a.crop == "tomato" then
        c.rod("wood", V(x, top, z), V(x, top + 1.2, z), 0.01, STRAW, 4)
        ball("leaf", x, top + 0.45, z, 0.2, LEAF, 1.6)
        for j = 1, 4 do ball("hard", x + 0.14 * math.cos(j * 1.7), top + 0.25 + j * 0.13, z + 0.14 * math.sin(j * 1.7), 0.035, V(0.8, 0.12, 0.08)) end
      end
    end
  end
  m:shape("bed", { box = { L, H, W }, pos = { 0, H / 2, 0 } })
elseif a.kind == "compost_bin" then
  local S, H = 1.0, 0.9
  for i = 0, 4 do
    local y = 0.09 + i * 0.18
    for _, z in ipairs{ -S / 2, S / 2 } do c.box("wood", V(0, y, z), V(S, 0.14, 0.025), 0.003, wood, 1) end
    for _, x in ipairs{ -S / 2, S / 2 } do c.box("wood", V(x, y, 0), V(0.025, 0.14, S), 0.003, wood, 1) end
  end
  for _, x in ipairs{ -S / 2, S / 2 } do for _, z in ipairs{ -S / 2, S / 2 } do c.box("wood", V(x, H / 2, z), V(0.07, H, 0.07), 0.004, wood * 0.85, 1) end end
  c.box("soil", V(0, 0.55, 0), V(S - 0.08, 0.06, S - 0.08), 0.0, SOIL * 1.2, 1)
  local lid = m:node("lid", { pos = { 0, H, -S / 2 }, motion = { turn = { -1, 0, 0 }, min = 0, max = 75 } })
  local k = kit.maker()
  k.box("wood", V(0, 0.015, S / 2), V(S + 0.06, 0.03, S + 0.06), 0.004, wood * 0.9, 1)
  k.rod("metal", V(-0.1, 0.03, S), V(0.1, 0.03, S), 0.012, kit.metal.iron, 6)
  kit.put(m, lid, k)
  lid:marker("handle", { pos = { 0, 0.05, S }, role = "hold" })
  m:shape("bin", { box = { S, H, S }, pos = { 0, H / 2, 0 } })
elseif a.kind == "tool_shed" then
  local W, D, H, R = 2.0, 1.5, 1.9, 0.5
  local n = 14
  for i = 1, n do
    local y = (i - 0.5) * H / n
    c.box("wood", V(0, y, -D / 2), V(W, H / n - 0.004, 0.025), 0.0, wood * (0.92 + 0.08 * (i % 2)), 1)
    for _, x in ipairs{ -W / 2, W / 2 } do c.box("wood", V(x, y, 0), V(0.025, H / n - 0.004, D), 0.0, wood * (0.92 + 0.08 * (i % 2)), 1) end
    for _, x in ipairs{ -W / 2 + 0.3, W / 2 - 0.55 } do c.box("wood", V(x, y, D / 2), V(x < 0 and 0.6 or 0.9, H / n - 0.004, 0.025), 0.0, wood * (0.92 + 0.08 * (i % 2)), 1) end
  end
  c.box("wood", V(0, 0.03, 0), V(W - 0.05, 0.06, D - 0.05), 0.0, wood * 0.7, 1)      -- the floor
  for _, s in ipairs{ -1, 1 } do                                                    -- the roof's two slopes and the gables under them
    c.mesh("rubber", mesh.translate(mesh.rotate_z(mesh.box{ W / 2 + 0.22, 0.03, D + 0.2 }, s * -math.atan(R, W / 2)), { s * W / 4, H + R / 2 + 0.02, 0 }), V(0.2, 0.22, 0.2))
  end
  for _, z in ipairs{ -D / 2, D / 2 } do                                             -- the gables: boards that shorten up to the ridge
    for j = 1, 5 do c.box("wood", V(0, H + (j - 0.5) * R / 5, z), V(W * (1 - (j - 0.5) / 5), R / 5 - 0.004, 0.025), 0.0, wood * (0.92 + 0.08 * (j % 2)), 1) end
  end
  c.box("glass", V(-W / 2 + 0.3, 1.3, D / 2 + 0.014), V(0.4, 0.4, 0.004), 0.0, V(0.5, 0.6, 0.62), 1)   -- a small window
  local door = m:node("door", { pos = { -W / 2 + 0.6, 0.06, D / 2 }, motion = { turn = { 0, -1, 0 }, min = 0, max = 110 } })
  door_on(door, 0.7, 1.75, paint)
  for i, w in ipairs{ { 0, -D / 2, W, 0.05 }, { -W / 2, 0, 0.05, D }, { W / 2, 0, 0.05, D } } do m:shape("wall_" .. i, { box = { w[3], H, w[4] }, pos = { w[1], H / 2, w[2] } }) end
  m:shape("roof", { box = { W + 0.3, 0.06, D + 0.2 }, pos = { 0, H + R * 0.6, 0 } })
elseif a.kind == "greenhouse" then
  local W, D, H, R, B = 2.4, 3.0, 1.7, 0.7, 0.5
  for _, z in ipairs{ -D / 2, D / 2 } do c.box("masonry", V(z > 0 and 0.45 or 0, B / 2, z), V(z > 0 and W - 0.9 or W, B, 0.11), 0.0, V(0.6, 0.33, 0.26), 1) end
  for _, x in ipairs{ -W / 2, W / 2 } do c.box("masonry", V(x, B / 2, 0), V(0.11, B, D), 0.0, V(0.6, 0.33, 0.26), 1) end
  local F = kit.WHITE * 0.92
  for _, x in ipairs{ -W / 2, W / 2 } do
    c.box("glass", V(x, (B + H) / 2, 0), V(0.006, H - B, D - 0.04), 0.0, V(0.85, 0.93, 0.9), 1)
    c.box("paint", V(x, H, 0), V(0.05, 0.05, D), 0.004, F, 1)
    for i = 0, 4 do c.box("paint", V(x, (B + H) / 2, -D / 2 + i * D / 4), V(0.04, H - B, 0.04), 0.004, F, 1) end
  end
  c.box("glass", V(0, (B + H) / 2, -D / 2), V(W - 0.04, H - B, 0.006), 0.0, V(0.85, 0.93, 0.9), 1)
  c.box("glass", V(0.45, (B + H) / 2, D / 2), V(W - 0.94, H - B, 0.006), 0.0, V(0.85, 0.93, 0.9), 1)
  for _, s in ipairs{ -1, 1 } do
    local ang = s * -math.atan(R, W / 2)
    c.mesh("glass", mesh.translate(mesh.rotate_z(mesh.box{ W / 2 + 0.1, 0.006, D }, ang), { s * W / 4, H + R / 2, 0 }), V(0.85, 0.93, 0.9))
    for i = 0, 4 do c.mesh("paint", mesh.translate(mesh.rotate_z(mesh.box{ W / 2 + 0.1, 0.035, 0.035 }, ang), { s * W / 4, H + R / 2, -D / 2 + i * D / 4 }), F) end
  end
  c.box("paint", V(0, H + R, 0), V(0.05, 0.05, D + 0.04), 0.004, F, 1)
  c.box("wood", V(W / 2 - 0.35, B + 0.3, 0), V(0.5, 0.03, D - 0.3), 0.004, wood, 1)       -- a potting bench with pots
  for i = 1, 5 do c.turned("ceramic", V(W / 2 - 0.35, B + 0.315, -D / 2 + i * 0.5), { { 0, 0 }, { 0.05, 0 }, { 0.07, 0.12 }, { 0, 0.12 } }, V(0.7, 0.36, 0.24), nil, 8); ball("leaf", W / 2 - 0.35, B + 0.5, -D / 2 + i * 0.5, 0.08, PALE) end
  local door = m:node("door", { pos = { -W / 2 + 0.06, 0, D / 2 }, motion = { turn = { 0, -1, 0 }, min = 0, max = 110 } })
  local k = kit.maker()
  k.box("glass", V(0.39, H / 2, 0), V(0.68, H - 0.16, 0.006), 0.0, V(0.85, 0.93, 0.9), 1)
  for _, x in ipairs{ 0.03, 0.75 } do k.box("paint", V(x, H / 2, 0), V(0.06, H - 0.02, 0.035), 0.004, F, 1) end
  for _, y in ipairs{ 0.05, H - 0.04 } do k.box("paint", V(0.39, y, 0), V(0.78, 0.08, 0.035), 0.004, F, 1) end
  kit.put(m, door, k)
  door:shape("door", { box = { 0.78, H, 0.04 }, pos = { 0.39, H / 2, 0 } })
  door:marker("handle", { pos = { 0.7, 1.0, 0.05 }, role = "hold" })
  for i, w in ipairs{ { 0, -D / 2, W, 0.06 }, { -W / 2, 0, 0.06, D }, { W / 2, 0, 0.06, D }, { 0.45, D / 2, W - 0.9, 0.06 } } do m:shape("wall_" .. i, { box = { w[3], H, w[4] }, pos = { w[1], H / 2, w[2] } }) end
elseif a.kind == "cold_frame" then
  local W, D, HB, HF = 1.2, 0.7, 0.5, 0.3
  c.box("wood", V(0, HB / 2, -D / 2), V(W, HB, 0.03), 0.004, wood, 1)
  c.box("wood", V(0, HF / 2, D / 2), V(W, HF, 0.03), 0.004, wood, 1)
  for _, x in ipairs{ -W / 2, W / 2 } do c.box("wood", V(x, HF / 2, 0), V(0.03, HF, D), 0.004, wood, 1) end
  c.box("soil", V(0, 0.1, 0), V(W - 0.06, 0.04, D - 0.06), 0.0, SOIL, 1)
  for i = 1, 5 do for j = 1, 3 do ball("leaf", -W / 2 + i * W / 6, 0.15, -D / 2 + j * D / 4, 0.04, PALE) end end
  local lid = m:node("lid", { pos = { 0, HB, -D / 2 }, turn = { math.deg(math.atan(HB - HF, D)), 0, 0 }, motion = { turn = { -1, 0, 0 }, min = 0, max = 70 } })
  local k = kit.maker()
  local LD = math.sqrt(D * D + (HB - HF) ^ 2)
  k.box("glass", V(0, 0.015, LD / 2), V(W - 0.1, 0.006, LD - 0.1), 0.0, V(0.85, 0.93, 0.9), 1)
  for _, x in ipairs{ -W / 2 + 0.03, 0, W / 2 - 0.03 } do k.box("wood", V(x, 0.015, LD / 2), V(0.05, 0.03, LD), 0.004, wood * 0.9, 1) end
  for _, z in ipairs{ 0.03, LD - 0.03 } do k.box("wood", V(0, 0.015, z), V(W, 0.03, 0.05), 0.004, wood * 0.9, 1) end
  kit.put(m, lid, k)
  lid:marker("handle", { pos = { 0, 0.04, LD }, role = "hold" })
  m:shape("frame", { box = { W, HF, D }, pos = { 0, HF / 2, 0 } })
elseif a.kind == "water_butt" then
  for _, x in ipairs{ -0.22, 0.22 } do c.box("masonry", V(x, 0.15, 0), V(0.2, 0.3, 0.5), 0.0, V(0.5, 0.5, 0.48), 1) end
  c.turned("hard", V(0, 0.3, 0), { { 0, 0 }, { 0.26, 0 }, { 0.31, 0.45 }, { 0.3, 0.9 }, { 0.31, 0.92 }, { 0, 0.94 } }, paint * 0.5, nil, 14)
  for _, y in ipairs{ 0.5, 0.95 } do c.turned("hard", V(0, y, 0), { { 0.285, 0 }, { 0.32, 0.015 }, { 0.285, 0.03 } }, paint * 0.4, nil, 14) end
  c.rod("metal", V(0, 0.42, 0.27), V(0, 0.42, 0.36), 0.015, kit.metal.brass, 6)
  c.rod("metal", V(0, 0.42, 0.34), V(0, 0.36, 0.34), 0.012, kit.metal.brass, 6)
  c.rod("hard", V(0.2, 1.24, -0.2), V(0.2, 1.9, -0.2), 0.04, V(0.25, 0.25, 0.26), 8)     -- the downpipe that fills it
  m:shape("butt", { box = { 0.62, 1.24, 0.62 }, pos = { 0, 0.62, 0 } })
elseif a.kind == "wheelbarrow" then
  c.cage("metal", V(0, 0.42, 0.05), { { 0.4, 0.5, 0.05, 0 }, { 0.62, 0.86, 0.08, 0.26 }, { 0.58, 0.82, 0.08, 0.26 }, { 0.37, 0.47, 0.05, 0.03 } }, paint * 0.8, { level = 0 })
  c.mesh("rubber", mesh.translate(mesh.rotate_z(mesh.lathe{ profile = { { 0.1, -0.04 }, { 0.19, -0.04 }, { 0.2, 0 }, { 0.19, 0.04 }, { 0.1, 0.04 } }, segments = 12 }, math.pi / 2), { 0, 0.2, 0.62 }), V(0.08, 0.08, 0.08))
  for _, x in ipairs{ -0.25, 0.25 } do
    c.rod("metal", V(x * 0.3, 0.2, 0.62), V(x, 0.4, 0.1), 0.015, STEEL, 6)
    c.rod("metal", V(x, 0.4, 0.1), V(x * 1.2, 0.6, -0.95), 0.015, STEEL, 6)
    c.rod("rubber", V(x * 1.2, 0.6, -0.8), V(x * 1.2, 0.6, -0.95), 0.02, V(0.1, 0.1, 0.1), 6)
    c.rod("metal", V(x, 0.42, -0.3), V(x, 0, -0.34), 0.014, STEEL, 6)
  end
  m:shape("barrow", { box = { 0.64, 0.7, 1.6 }, pos = { 0, 0.35, -0.15 } })
elseif a.kind == "bean_poles" then
  local L = a.length
  local n = math.max(2, math.floor(L / 0.4))
  for i = 1, n do
    local x = -L / 2 + (i - 0.5) * L / n
    for _, s in ipairs{ -1, 1 } do
      c.rod("wood", V(x, 0, s * 0.35), V(x, 2.0, s * -0.06), 0.012, STRAW, 4)
      for j = 1, 4 do ball("leaf", x, 0.25 + j * 0.36, s * (0.35 - j * 0.075), 0.13, j % 2 == 0 and LEAF or LEAF * 1.25, 1.3) end
    end
  end
  c.rod("wood", V(-L / 2, 1.82, 0), V(L / 2, 1.82, 0), 0.012, STRAW, 4)
  m:shape("row", { box = { L, 2.0, 0.7 }, pos = { 0, 1.0, 0 } })
else   -- scarecrow
  c.rod("wood", V(0, 0, 0), V(0, 1.85, 0), 0.03, wood, 6)
  c.rod("wood", V(-0.75, 1.4, 0), V(0.75, 1.4, 0), 0.025, wood, 6)
  c.box("fabric", V(0, 1.12, 0), V(0.46, 0.66, 0.2), 0.06, paint * 0.6, 2)                   -- the coat
  for _, s in ipairs{ -1, 1 } do
    c.mesh("fabric", mesh.translate(mesh.rotate_z(mesh.rounded_box({ 0.5, 0.15, 0.15 }, 0.05, { segs = 2 }), 0), { s * 0.48, 1.4, 0 }), paint * 0.6)
    for j = 1, 3 do c.rod("fabric", V(s * 0.72, 1.4, 0), V(s * 0.86, 1.4 + (j - 2) * 0.07, 0.02), 0.008, STRAW, 4) end
    c.box("fabric", V(s * 0.11, 0.55, 0), V(0.17, 0.5, 0.15), 0.04, V(0.25, 0.27, 0.36), 2)  -- a trouser leg
  end
  ball("fabric", 0, 1.68, 0, 0.14, V(0.82, 0.74, 0.55))
  c.turned("fabric", V(0, 1.76, 0), { { 0, 0.12 }, { 0.12, 0.1 }, { 0.14, 0 }, { 0.26, 0 } }, STRAW * 0.9, nil, 10)
  m:shape("post", { box = { 0.5, 1.9, 0.25 }, pos = { 0, 0.95, 0 } })
end
kit.put(m, m, c)
return m
