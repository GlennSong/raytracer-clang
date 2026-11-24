-- WHAT STANDS ON A ROOF (#191): the plant and fittings of a building's top.
--
--   hvac          a packaged air-handling unit on rails: louvred sides, two fans under grilles on top (`fan_1`, `fan_2` spin)
--   condenser     a small split-system condenser, one fan behind a grille (`fan` spins): flats and shops have rows of them
--   cooling_tower a round tower on a steel frame, a big fan in its throat (`fan` spins)
--   water_tower   a timber tank with a conical roof on a braced steel stand, a ladder up its side (the old-town skyline)
--   water_tank    a squat sectional steel tank on bearers (a modern block's)
--   helipad       a round pad with the H, its aiming circle, a rim of lights and a windsock on a mast (`sock` swings)
--   dish          a satellite dish on a short mast, tipped at the sky
--   antenna       a lattice mast with stub aerials and drum dishes, a red lamp on top (marker `lamp`)
--   vent          a spinning turbine ventilator on a curb (`turbine` spins)
--   stack         a pair of exhaust flues with rain caps
--   skylight      a ridged glass rooflight on a curb
--   solar         a row of solar panels tilted on frames: `count` panels
--   access        the stair's head: a small block with a door (`door`) and a lamp over it (marker `lamp`)
--
-- Each stands on y = 0 (the roof's deck) at its own origin, its front toward +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind  = { "hvac", "condenser", "cooling_tower", "water_tower", "water_tank", "helipad", "dish", "antenna", "vent", "stack",
            "skylight", "solar", "access" },
  size  = { 1.0, min = 0.5, max = 2.5 },       -- a scale on the kind's own size (hvac, water_tower, helipad, antenna)
  count = { 4, min = 1, max = 12, step = 1 },  -- solar
  wood  = kit.wood_names,
}
local Z = a.size
local GALV, DARK, WHITE = V(0.66, 0.68, 0.69), V(0.2, 0.21, 0.22), kit.WHITE * 0.9
local STEEL = kit.metal.steel
local m = model.new()
local far = m:level(120)
local c = kit.maker()

-- a fan on a node that spins about +y: a hub and `n` blades of radius r, at (x, y, z)
local function fan(name, x, y, z, r, n)
  local node = m:node(name, { pos = { x, y, z }, motion = { turn = { 0, 1, 0 } } })
  local k = kit.maker()
  k.rod("metal", V(0, -0.04, 0), V(0, 0.04, 0), r * 0.18, DARK, 8)
  for i = 1, n do
    k.mesh("metal", mesh.rotate_y(mesh.translate(mesh.rotate_x(mesh.box{ r * 0.34, 0.006, r * 0.8 }, 0.35), { 0, 0, r * 0.55 }), i * 2 * math.pi / n), DARK * 1.4)
  end
  kit.put(m, node, k)
end
-- a round grille over a fan: a ring and spokes
local function grille(x, y, z, r)
  c.turned("metal", V(x, y, z), { { r - 0.02, 0 }, { r, 0.02 }, { r - 0.02, 0.04 } }, DARK, nil, 16)
  for i = 0, 5 do
    local t = i * math.pi / 6
    c.rod("metal", V(x - r * math.cos(t), y + 0.02, z - r * math.sin(t)), V(x + r * math.cos(t), y + 0.02, z + r * math.sin(t)), 0.006, DARK, 4)
  end
end
local function louvres(x0, x1, y0, y1, z, col)        -- slats across a panel that faces +z or -z
  local n = math.max(2, math.floor((y1 - y0) / 0.07))
  for i = 1, n do c.tilted("metal", V((x0 + x1) / 2, y0 + (i - 0.5) * (y1 - y0) / n, z), V(x1 - x0, 0.012, 0.06), 0.0, 0.6, col) end
end

if a.kind == "hvac" then
  local W, H, D = 3.2 * Z, 1.4 * Z, 1.6 * Z
  for _, z in ipairs{ -D / 2 + 0.1, D / 2 - 0.1 } do c.box("metal", V(0, 0.08, z), V(W + 0.2, 0.16, 0.1), 0.0, DARK, 1) end
  c.box("metal", V(0, 0.16 + H / 2, 0), V(W, H, D), 0.02, GALV, 1)
  for _, z in ipairs{ -D / 2 - 0.02, D / 2 + 0.02 } do louvres(-W / 2 + 0.15, -0.1, 0.3, 0.16 + H - 0.15, z, GALV * 0.75) end
  c.box("metal", V(W * 0.25, 0.16 + H * 0.5, D / 2 + 0.006), V(W * 0.4, H * 0.8, 0.012), 0.004, GALV * 0.9, 1)      -- an access panel
  c.rod("metal", V(W * 0.4, 0.16 + H * 0.5, D / 2 + 0.02), V(W * 0.4, 0.16 + H * 0.62, D / 2 + 0.02), 0.012, DARK, 6)
  for i, x in ipairs{ -W * 0.22, W * 0.22 } do
    c.turned("metal", V(x, 0.16 + H, 0), { { D * 0.36, 0 }, { D * 0.36, 0.1 }, { D * 0.33, 0.1 } }, GALV * 0.85, nil, 16)
    grille(x, 0.16 + H + 0.1, 0, D * 0.36)
    fan("fan_" .. i, x, 0.16 + H + 0.05, 0, D * 0.33, 5)
  end
  c.box("metal", V(-W / 2 - 0.25, 0.5, 0), V(0.5, 0.5, 0.6), 0.02, GALV, 1)                                         -- the duct that drops into the roof
  m:shape("unit", { box = { W, H + 0.3, D }, pos = { 0, (H + 0.3) / 2, 0 } })
elseif a.kind == "condenser" then
  local W, H, D = 0.9, 0.7, 0.36
  for _, x in ipairs{ -0.3, 0.3 } do c.box("rubber", V(x, 0.05, 0), V(0.1, 0.1, D + 0.1), 0.0, DARK, 1) end
  c.box("metal", V(0, 0.1 + H / 2, 0), V(W, H, D), 0.03, WHITE, 2)
  c.mesh("metal", mesh.translate(mesh.rotate_x(mesh.lathe{ profile = { { 0.26, 0 }, { 0.28, 0.01 }, { 0.26, 0.02 } }, segments = 16 }, math.pi / 2), { -0.12, 0.1 + H / 2, D / 2 }), DARK)
  for i = -2, 2 do c.box("metal", V(-0.12, 0.1 + H / 2 + i * 0.09, D / 2 + 0.012), V(0.5, 0.008, 0.008), 0.0, DARK, 1) end
  local node = m:node("fan", { pos = { -0.12, 0.1 + H / 2, D / 2 - 0.03 }, motion = { turn = { 0, 0, 1 } } })
  local k = kit.maker()
  for i = 1, 3 do k.mesh("metal", mesh.rotate_z(mesh.translate(mesh.rotate_y(mesh.box{ 0.09, 0.2, 0.006 }, 0.4), { 0, 0.13, 0 }), i * 2 * math.pi / 3), DARK * 1.5) end
  kit.put(m, node, k)
  c.rod("rubber", V(W / 2, 0.3, -0.05), V(W / 2 + 0.05, 0.02, -0.3), 0.012, DARK, 5)                                  -- its pipes
  m:shape("unit", { box = { W, H + 0.1, D }, pos = { 0, (H + 0.1) / 2, 0 } })
elseif a.kind == "cooling_tower" then
  local R, H = 1.3 * Z, 2.6 * Z
  for i = 0, 3 do
    local t = i * math.pi / 2 + math.pi / 4
    c.box("metal", V(R * 0.8 * math.cos(t), 0.3, R * 0.8 * math.sin(t)), V(0.12, 0.6, 0.12), 0.0, DARK, 1)
  end
  c.turned("metal", V(0, 0.6, 0), { { 0, 0 }, { R, 0 }, { R, H * 0.55 }, { R * 0.72, H * 0.8 }, { R * 0.78, H }, { R * 0.74, H }, { R * 0.68, H * 0.8 } }, V(0.78, 0.78, 0.74), nil, 20)
  for i = 1, 5 do c.turned("metal", V(0, 0.6 + i * H * 0.09, 0), { { R, 0 }, { R + 0.02, 0.03 }, { R, 0.06 } }, GALV * 0.7, nil, 20) end
  fan("fan", 0, 0.6 + H * 0.88, 0, R * 0.66, 6)
  c.rod("metal", V(R + 0.1, 0.2, 0), V(R + 0.1, 1.2, 0), 0.08, V(0.2, 0.35, 0.55), 8)                                 -- its water pipe
  c.rod("metal", V(R + 0.1, 1.2, 0), V(R - 0.05, 1.2, 0), 0.08, V(0.2, 0.35, 0.55), 8)
  m:shape("tower", { box = { 2 * R, H + 0.6, 2 * R }, pos = { 0, (H + 0.6) / 2, 0 } })
elseif a.kind == "water_tower" then
  local R, TH, LEG = 1.5 * Z, 2.6 * Z, 3.0 * Z
  local wood = kit.wood(a.wood)
  for i = 0, 3 do
    local t = i * math.pi / 2 + math.pi / 4
    local x, z = R * 0.8 * math.cos(t), R * 0.8 * math.sin(t)
    c.box("metal", V(x, LEG / 2, z), V(0.12, LEG, 0.12), 0.0, DARK, 1)
    local t2 = t + math.pi / 2
    local x2, z2 = R * 0.8 * math.cos(t2), R * 0.8 * math.sin(t2)
    c.rod("metal", V(x, 0.2, z), V(x2, LEG - 0.2, z2), 0.02, DARK, 4)                                                 -- the braces
    c.rod("metal", V(x2, 0.2, z2), V(x, LEG - 0.2, z), 0.02, DARK, 4)
    c.rod("metal", V(x, LEG, z), V(x2, LEG, z2), 0.06, DARK, 4)
  end
  c.turned("wood", V(0, LEG + 0.06, 0), { { 0, 0 }, { R, 0 }, { R * 0.96, TH }, { 0, TH } }, wood * 0.8, nil, 20)
  for i = 0, 5 do c.turned("metal", V(0, LEG + 0.2 + i * (TH - 0.4) / 5, 0), { { R * (1 - 0.04 * i / 5) + 0.002, 0 }, { R * (1 - 0.04 * i / 5) + 0.02, 0.025 }, { R * (1 - 0.04 * i / 5) + 0.002, 0.05 } }, DARK, nil, 20) end
  c.turned("wood", V(0, LEG + 0.06 + TH, 0), { { R * 1.08, 0 }, { 0, R * 0.7 } }, wood * 0.55, nil, 20)
  c.mesh("metal", mesh.translate(kit.ball(0.08, 8), { 0, LEG + TH + R * 0.7 + 0.1, 0 }), DARK)
  for _, x in ipairs{ -0.2, 0.2 } do c.rod("metal", V(x, 0, R + 0.08), V(x, LEG + TH, R + 0.08), 0.015, DARK, 4) end   -- the ladder
  for i = 1, math.floor((LEG + TH) / 0.3) do c.rod("metal", V(-0.2, i * 0.3, R + 0.08), V(0.2, i * 0.3, R + 0.08), 0.01, DARK, 4) end
  c.rod("metal", V(0, 0, 0), V(0, LEG, 0), 0.07, DARK, 8)                                                              -- the riser pipe
  m:shape("tank", { box = { 2 * R, TH + R * 0.7, 2 * R }, pos = { 0, LEG + (TH + R * 0.7) / 2, 0 } })
  m:shape("stand", { box = { R * 1.4, LEG, R * 1.4 }, pos = { 0, LEG / 2, 0 } })
elseif a.kind == "water_tank" then
  local W, H, D = 3.0 * Z, 2.0 * Z, 2.0 * Z
  for _, x in ipairs{ -W / 3, 0, W / 3 } do c.box("metal", V(x, 0.1, 0), V(0.15, 0.2, D + 0.1), 0.0, DARK, 1) end
  c.box("metal", V(0, 0.2 + H / 2, 0), V(W, H, D), 0.0, GALV, 1)
  local nx, ny = math.max(1, math.floor(W)), math.max(1, math.floor(H))
  for i = 0, nx do c.box("metal", V(-W / 2 + i * W / nx, 0.2 + H / 2, 0), V(0.04, H + 0.02, D + 0.04), 0.0, GALV * 0.8, 1) end
  for j = 0, ny do c.box("metal", V(0, 0.2 + j * H / ny, 0), V(W + 0.04, 0.04, D + 0.04), 0.0, GALV * 0.8, 1) end
  c.turned("metal", V(W * 0.25, 0.2 + H, 0), { { 0, 0.05 }, { 0.3, 0.05 }, { 0.3, 0 } }, GALV * 0.7, nil, 12)          -- the hatch
  for _, x in ipairs{ -0.2, 0.2 } do c.rod("metal", V(W / 2 - 0.5 + x, 0, D / 2 + 0.08), V(W / 2 - 0.5 + x, 0.2 + H + 0.5, D / 2 + 0.08), 0.015, DARK, 4) end
  for i = 1, math.floor((H + 0.2) / 0.3) do c.rod("metal", V(W / 2 - 0.7, i * 0.3, D / 2 + 0.08), V(W / 2 - 0.3, i * 0.3, D / 2 + 0.08), 0.01, DARK, 4) end
  m:shape("tank", { box = { W, H + 0.2, D }, pos = { 0, (H + 0.2) / 2, 0 } })
elseif a.kind == "helipad" then
  local R = 7.5 * Z
  c.turned("stone", V(0, 0, 0), { { 0, 0.25 }, { R, 0.25 }, { R, 0 } }, V(0.3, 0.32, 0.33), nil, 32)
  c.turned("paint", V(0, 0.252, 0), { { R * 0.62, 0 }, { R * 0.68, 0 } }, V(0.95, 0.85, 0.1), nil, 32)                 -- the aiming circle
  c.turned("paint", V(0, 0.252, 0), { { R - 0.5, 0 }, { R - 0.3, 0 } }, WHITE, nil, 32)
  local s = R * 0.3
  for _, x in ipairs{ -s * 0.55, s * 0.55 } do c.box("paint", V(x, 0.254, 0), V(s * 0.28, 0.004, s * 1.9), 0.0, WHITE, 1) end   -- the H
  c.box("paint", V(0, 0.254, 0), V(s * 1.1, 0.004, s * 0.28), 0.0, WHITE, 1)
  for i = 0, 11 do
    local t = i * math.pi / 6
    c.rod("light", V((R - 0.12) * math.cos(t), 0.25, (R - 0.12) * math.sin(t)), V((R - 0.12) * math.cos(t), 0.33, (R - 0.12) * math.sin(t)), 0.05, V(0.3, 1.0, 0.4), 6)
  end
  for i = 0, 15 do                                                                                                       -- the safety net round its rim
    local t0, t1 = i * math.pi / 8, (i + 1) * math.pi / 8
    c.rod("metal", V(R * math.cos(t0), 0.2, R * math.sin(t0)), V((R + 1) * math.cos(t0), 0.3, (R + 1) * math.sin(t0)), 0.02, DARK, 4)
    c.rod("metal", V((R + 1) * math.cos(t0), 0.3, (R + 1) * math.sin(t0)), V((R + 1) * math.cos(t1), 0.3, (R + 1) * math.sin(t1)), 0.02, DARK, 4)
  end
  c.rod("metal", V(R + 0.6, 0, -R * 0.5), V(R + 0.6, 4, -R * 0.5), 0.04, WHITE, 8)                                       -- the windsock's mast
  local sock = m:node("sock", { pos = { R + 0.6, 3.9, -R * 0.5 }, motion = { turn = { 0, 1, 0 } } })
  local k = kit.maker()
  k.mesh("fabric", mesh.rotate_z(mesh.lathe{ profile = { { 0.18, 0 }, { 0.07, 1.1 } }, segments = 8 }, -math.pi / 2 + 0.25), V(0.95, 0.4, 0.1))
  kit.put(m, sock, k)
  m:shape("pad", { box = { R * 1.6, 0.25, R * 1.6 }, pos = { 0, 0.125, 0 } })      -- (a square inside the circle: walked on, landed on)
  m:marker("land", { pos = { 0, 0.25, 0 }, role = "stand" })
elseif a.kind == "dish" then
  c.box("metal", V(0, 0.03, 0), V(0.6, 0.06, 0.6), 0.0, DARK, 1)
  c.rod("metal", V(0, 0, 0), V(0, 0.9, 0), 0.04, GALV, 8)
  c.mesh("metal", mesh.translate(mesh.rotate_x(mesh.lathe{ profile = { { 0, 0 }, { 0.3, 0.03 }, { 0.55, 0.12 }, { 0.75, 0.26 } }, segments = 20, smooth = true }, math.rad(55)), { 0, 1.0, 0.05 }), WHITE)
  c.rod("metal", V(0, 1.0, 0.05), V(0, 1.42, 0.65), 0.012, GALV, 4)
  c.box("metal", V(0, 1.43, 0.66), V(0.07, 0.07, 0.12), 0.01, DARK, 1)
  m:shape("dish", { box = { 1.5, 1.8, 1.2 }, pos = { 0, 0.9, 0.2 } })
elseif a.kind == "antenna" then
  local H, S = 9 * Z, 0.5
  c.box("stone", V(0, 0.1, 0), V(1.0, 0.2, 1.0), 0.0, V(0.6, 0.6, 0.58), 1)
  local corners = { { -S / 2, -S / 2 }, { S / 2, -S / 2 }, { S / 2, S / 2 }, { -S / 2, S / 2 } }
  for i, p in ipairs(corners) do
    c.rod("metal", V(p[1], 0.2, p[2]), V(p[1] * 0.3, H, p[2] * 0.3), 0.02, V(0.8, 0.15, 0.1), 4)
    local q = corners[i % 4 + 1]
    local n = math.floor(H / 0.9)
    for j = 0, n - 1 do
      local f0, f1 = 1 - 0.7 * j / n, 1 - 0.7 * (j + 1) / n
      local y0, y1 = 0.2 + j * (H - 0.2) / n, 0.2 + (j + 1) * (H - 0.2) / n
      c.rod("metal", V(p[1] * f0, y0, p[2] * f0), V(q[1] * f1, y1, q[2] * f1), 0.01, j % 2 == 0 and V(0.8, 0.15, 0.1) or WHITE, 3)
    end
  end
  for i, y in ipairs{ H * 0.55, H * 0.7, H * 0.82 } do
    c.mesh("metal", mesh.translate(mesh.rotate_y(mesh.rotate_x(mesh.lathe{ profile = { { 0, 0 }, { 0.3, 0 }, { 0.3, 0.16 }, { 0, 0.2 } }, segments = 12 }, math.pi / 2), i * 2.1), { 0.3 * math.sin(i * 2.1), y, 0.3 * math.cos(i * 2.1) }), WHITE)
  end
  for i = 0, 2 do c.rod("metal", V(0.12 * math.cos(i * 2.1), H - 0.4, 0.12 * math.sin(i * 2.1)), V(0.12 * math.cos(i * 2.1), H + 1.2, 0.12 * math.sin(i * 2.1)), 0.012, WHITE, 4) end
  c.mesh("light", mesh.translate(kit.ball(0.09, 8), { 0, H + 0.1, 0 }), V(1.0, 0.1, 0.05))
  m:marker("lamp", { pos = { 0, H + 0.1, 0 }, turn = { 90, 0, 0 }, role = "lamp" })
  m:shape("mast", { box = { S, H, S }, pos = { 0, H / 2, 0 } })
elseif a.kind == "vent" then
  c.box("metal", V(0, 0.1, 0), V(0.5, 0.2, 0.5), 0.0, GALV * 0.8, 1)
  c.rod("metal", V(0, 0.2, 0), V(0, 0.45, 0), 0.15, GALV, 12)
  local node = m:node("turbine", { pos = { 0, 0.45, 0 }, motion = { turn = { 0, 1, 0 } } })
  local k = kit.maker()
  for i = 1, 12 do
    k.mesh("metal", mesh.rotate_y(mesh.translate(mesh.rotate_y(mesh.scale(kit.ball(0.2, 6), { 0.06, 1, 0.5 }), 0.6), { 0, 0.2, 0.2 }), i * math.pi / 6), GALV * 1.1)
  end
  k.turned("metal", V(0, 0.38, 0), { { 0, 0.04 }, { 0.12, 0 } }, GALV, nil, 12)
  kit.put(m, node, k)
  m:shape("vent", { box = { 0.5, 0.85, 0.5 }, pos = { 0, 0.42, 0 } })
elseif a.kind == "stack" then
  c.box("metal", V(0, 0.15, 0), V(1.2, 0.3, 0.6), 0.0, GALV * 0.8, 1)
  for _, x in ipairs{ -0.3, 0.3 } do
    c.rod("metal", V(x, 0.3, 0), V(x, 2.4, 0), 0.16, GALV, 12)
    c.turned("metal", V(x, 2.5, 0), { { 0.28, 0 }, { 0, 0.14 } }, GALV * 0.85, nil, 12)
    for i = 0, 2 do c.rod("metal", V(x + 0.14 * math.cos(i * 2.1), 2.4, 0.14 * math.sin(i * 2.1)), V(x + 0.2 * math.cos(i * 2.1), 2.52, 0.2 * math.sin(i * 2.1)), 0.008, DARK, 3) end
    c.turned("metal", V(x, 1.4, 0), { { 0.16, 0 }, { 0.18, 0.02 }, { 0.16, 0.04 } }, DARK, nil, 12)
  end
  m:shape("stack", { box = { 1.2, 2.6, 0.6 }, pos = { 0, 1.3, 0 } })
elseif a.kind == "skylight" then
  local W, D, R = 2.4, 1.4, 0.45
  for _, z in ipairs{ -D / 2, D / 2 } do c.box("metal", V(0, 0.15, z), V(W, 0.3, 0.08), 0.0, GALV * 0.8, 1) end
  for _, x in ipairs{ -W / 2, W / 2 } do c.box("metal", V(x, 0.15, 0), V(0.08, 0.3, D), 0.0, GALV * 0.8, 1) end
  for _, s in ipairs{ -1, 1 } do
    local ang = s * math.atan(R, D / 2)
    c.mesh("glass", mesh.translate(mesh.rotate_x(mesh.box{ W, 0.008, D / 2 + 0.12 }, ang), { 0, 0.3 + R / 2, s * D / 4 }), V(0.75, 0.88, 0.92))
    for i = 0, 4 do c.mesh("metal", mesh.translate(mesh.rotate_x(mesh.box{ 0.04, 0.03, D / 2 + 0.12 }, ang), { -W / 2 + i * W / 4, 0.3 + R / 2, s * D / 4 }), GALV) end
  end
  c.box("metal", V(0, 0.3 + R, 0), V(W + 0.04, 0.05, 0.06), 0.0, GALV, 1)
  m:shape("skylight", { box = { W, 0.3 + R, D }, pos = { 0, (0.3 + R) / 2, 0 } })
elseif a.kind == "solar" then
  local n = math.floor(a.count + 0.5)
  local PW, PD, tilt = 1.05, 1.75, math.rad(25)
  for i = 1, n do
    local x = (i - (n + 1) / 2) * (PW + 0.03)
    c.mesh("screen", mesh.translate(mesh.rotate_x(mesh.box{ PW, 0.03, PD }, tilt), { x, 0.62, 0 }), V(0.05, 0.08, 0.2))
    c.mesh("metal", mesh.translate(mesh.rotate_x(mesh.box{ PW + 0.02, 0.02, PD + 0.02 }, tilt), { x, 0.605, 0 }), GALV)
  end
  local L = n * (PW + 0.03)
  for _, x in ipairs{ -L / 2 + 0.3, L / 2 - 0.3 } do
    c.rod("metal", V(x, 0, -0.7), V(x, 0.95, -0.7), 0.025, GALV, 6)
    c.rod("metal", V(x, 0, 0.7), V(x, 0.3, 0.7), 0.025, GALV, 6)
    c.box("stone", V(x, 0.05, 0), V(0.3, 0.1, 1.7), 0.0, V(0.6, 0.6, 0.58), 1)
  end
  m:shape("array", { box = { L, 1.1, PD }, pos = { 0, 0.55, 0 } })
else   -- access
  local W, D, H = 3.0, 4.0, 2.6
  c.box("stone", V(0, H / 2, -0.03), V(W, H, D - 0.06), 0.0, V(0.74, 0.73, 0.7), 1)
  c.box("metal", V(0, H + 0.04, 0), V(W + 0.2, 0.08, D + 0.2), 0.0, DARK, 1)
  c.box("paint", V(0, 1.06, D / 2), V(1.1, 2.12, 0.05), 0.0, DARK, 1)                                                    -- the door's frame
  local door = m:node("door", { pos = { -0.48, 0, D / 2 + 0.03 }, motion = { turn = { 0, -1, 0 }, min = 0, max = 105 } })
  local k = kit.maker()
  k.box("metal", V(0.48, 1.03, 0), V(0.96, 2.06, 0.04), 0.004, V(0.45, 0.48, 0.5), 1)
  k.rod("metal", V(0.1, 1.0, 0.03), V(0.86, 1.0, 0.03), 0.014, kit.CHROME, 6)
  kit.put(m, door, k)
  door:shape("door", { box = { 0.96, 2.06, 0.04 }, pos = { 0.48, 1.03, 0 } })
  door:marker("handle", { pos = { 0.8, 1.0, 0.06 }, role = "hold" })
  c.box("metal", V(0, 2.3, D / 2 + 0.06), V(0.3, 0.1, 0.12), 0.02, DARK, 1)
  c.box("light", V(0, 2.24, D / 2 + 0.06), V(0.24, 0.02, 0.08), 0.0, V(1, 0.95, 0.85), 1)
  m:marker("lamp", { pos = { 0, 2.2, D / 2 + 0.1 }, turn = { 90, 0, 0 }, role = "lamp" })
  m:shape("block", { box = { W, H, D - 0.06 }, pos = { 0, H / 2, -0.03 } })
end
kit.put(m, m, c)
return m
