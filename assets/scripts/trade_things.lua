-- PIECES FOR A SCHOOL, A LIBRARY AND A BAKER'S OR GROCER'S SHOP, each with a part that moves (#191):
--
--   flip_desk        a school desk whose top lifts on hinges at the far edge (`lid`), with books inside. It has no
--                    seat: its chair is a prefab of its own (the kit's `school_chair`), stood at its `chair` marker
--   school_bell      a brass bell on a wall bracket; it swings (`bell`) and so does its clapper (`clapper`, on
--                    the bell). Its origin is the foot of the bracket's plate (put it on a wall)
--   library_ladder   a ladder that leans on a rail along the shelves and rolls along it (`ladder`); the rail is
--                    `length` long and fixes to the shelves at `height`
--   dough_mixer      a baker's floor mixer: the bowl turns (`bowl`), the hook in it turns the other way (`hook`),
--                    and a guard over the bowl lifts (`guard`)
--   deck_oven        a baker's oven of three decks, each with a door that drops open (`door_1` ...)
--   shop_scales      a grocer's scales: the pan sinks under a load (`pan`) and the needle on its dial turns (`needle`)
--
-- Each stands on y = 0 at its own origin, its front toward +z (the scales on a counter).
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind   = { "flip_desk", "school_bell", "library_ladder", "dough_mixer", "deck_oven", "shop_scales" },
  wood   = kit.wood_names,
  length = { 3.0, min = 1.5, max = 8.0 },         -- the library rail's length (metres)
  height = { 2.2, min = 1.8, max = 3.2 },         -- and its height
}
local wood = kit.wood(a.wood)
local STEEL, DARK, BRASS, CREAM = kit.metal.steel, kit.paint.black, kit.metal.brass, kit.paint.cream
local m = model.new()
local far = m:level(40)
local c = kit.maker()

if a.kind == "flip_desk" then
  local W, D, Y, BOX = 0.62, 0.45, 0.6, 0.12
  for _, x in ipairs{ -W / 2 + 0.03, W / 2 - 0.03 } do
    c.pipe("metal", { { x, 0, 0.2 }, { x, Y, 0.18 } }, 0.012, DARK, { sides = 8 })
    c.pipe("metal", { { x, 0, -0.2 }, { x, Y, -0.18 } }, 0.012, DARK, { sides = 8 })
    c.rod("metal", V(x, 0.02, -0.2), V(x, 0.02, 0.2), 0.012, DARK, 8)
    c.box("wood", V(x, Y + BOX / 2, 0), V(0.015, BOX, D), 0.003, wood, 1)
  end
  for _, z in ipairs{ -D / 2 + 0.008, D / 2 - 0.008 } do c.box("wood", V(0, Y + BOX / 2, z), V(W - 0.06, BOX, 0.015), 0.003, wood, 1) end
  c.box("wood", V(0, Y + 0.008, 0), V(W - 0.06, 0.015, D), 0.0, wood, 1)
  for i, col in ipairs{ kit.paint.red, kit.paint.blue, kit.paint.green } do c.box("hard", V(-0.12 + i * 0.01, Y + 0.02 + i * 0.022, 0), V(0.2, 0.02, 0.27), 0.003, col, 1) end
  c.box("hard", V(0.17, Y + 0.03, 0.05), V(0.06, 0.03, 0.2), 0.008, kit.paint.yellow, 1)             -- a pencil case
  kit.put(m, m, c)
  kit.far_box(m, m, "wood", { W, BOX, D }, { 0, Y + BOX / 2, 0 }, wood, far)
  m:shape("box", { box = { W, BOX, D }, pos = { 0, Y + BOX / 2, 0 } })
  m:marker("chair", { pos = { 0, 0, 0.55 }, turn = { 0, 180, 0 }, role = "stand" })   -- where its chair goes (a prefab of its own)
  local lid = m:node("lid", { pos = { 0, Y + BOX, -D / 2 }, motion = { turn = { -1, 0, 0 }, min = 0, max = 75 } })
  local k = kit.maker()
  k.box("wood", V(0, 0.01, D / 2 + 0.01), V(W, 0.02, D + 0.03), 0.006, wood, 1)
  k.box("hard", V(0, 0.021, 0.04), V(0.2, 0.002, 0.02), 0.0, DARK, 1)                                -- the groove for a pencil
  kit.put(m, lid, k)
  lid:shape("lid", { box = { W, 0.02, D }, pos = { 0, 0.01, D / 2 } })
  lid:marker("lid", { pos = { 0, 0.01, D + 0.02 }, role = "hold" })
elseif a.kind == "school_bell" then
  c.box("metal", V(0, 0.2, 0.006), V(0.12, 0.4, 0.012), 0.004, DARK, 1)
  c.pipe("metal", { { 0, 0.34, 0.01 }, { 0, 0.42, 0.22 }, { 0, 0.36, 0.22 } }, 0.012, DARK, { bend = 0.04, sides = 8 })
  c.rod("metal", V(0, 0.1, 0.01), V(0, 0.38, 0.2), 0.008, DARK, 6)
  kit.put(m, m, c)
  m:shape("bracket", { box = { 0.12, 0.42, 0.24 }, pos = { 0, 0.21, 0.12 } })
  m:marker("ring", { pos = { 0, 0.2, 0.22 } })
  local bell = m:node("bell", { pos = { 0, 0.36, 0.22 }, motion = { turn = { 1, 0, 0 }, min = -35, max = 35 } })
  local k = kit.maker()
  k.turned("metal", V(0, -0.2, 0), { { 0.105, 0 }, { 0.11, 0.01 }, { 0.075, 0.07 }, { 0.06, 0.15 }, { 0.03, 0.185 }, { 0, 0.19 }, { 0, 0.18 }, { 0.05, 0.145 }, { 0.065, 0.07 }, { 0.1, 0.0 } }, BRASS, 1, 20)
  k.rod("metal", V(0, -0.01, 0), V(0, 0, 0), 0.01, BRASS, 6)
  kit.put(m, bell, k)
  local clapper = bell:node("clapper", { pos = { 0, -0.03, 0 }, motion = { turn = { 1, 0, 0 }, min = -30, max = 30 } })
  local p = kit.maker()
  p.rod("metal", V(0, 0, 0), V(0, -0.14, 0), 0.004, DARK, 4)
  p.mesh("metal", mesh.translate(kit.ball(0.02, 8), { 0, -0.15, 0 }), DARK)
  p.rod("hard", V(0, -0.16, 0), V(0, -0.5, 0), 0.003, kit.WHITE, 4)                                   -- the cord to ring it by
  kit.put(m, clapper, p)
  clapper:marker("cord", { pos = { 0, -0.5, 0 }, role = "hold" })
elseif a.kind == "library_ladder" then
  local L, H = a.length, a.height
  c.rod("metal", V(-L / 2, H, 0.06), V(L / 2, H, 0.06), 0.014, BRASS, 8)
  for i = 0, math.floor(L) do
    local x = -L / 2 + i * L / math.floor(L)
    c.rod("metal", V(x, H, 0), V(x, H, 0.06), 0.008, BRASS, 6)
  end
  kit.put(m, m, c)
  kit.far_box(m, m, "metal", { L, 0.03, 0.03 }, { 0, H, 0.06 }, BRASS, far)
  local ladder = m:node("ladder", { pos = { -L / 2 + 0.3, 0, 0 }, motion = { slide = { 1, 0, 0 }, min = 0, max = L - 0.6 } })
  local k = kit.maker()
  local foot, top = V(0, 0.05, 0.75), V(0, H + 0.02, 0.09)
  for _, x in ipairs{ -0.2, 0.2 } do
    k.post("wood", foot + V(x, 0, 0), top + V(x, 0, 0), 0.04, 0.04, wood)
    k.pipe("metal", { { x, H + 0.02, 0.09 }, { x, H + 0.05, 0.06 }, { x, H, 0.03 } }, 0.006, BRASS, { bend = 0.02, sides = 6 })   -- the hook over the rail
    kit.caster(k, V(x, 0, 0.75), 0.025)
  end
  local rungs = math.floor(H / 0.28)
  for i = 1, rungs do
    local p = foot + (top - foot) * (i / (rungs + 1))
    k.box("wood", p, V(0.4, 0.025, 0.09), 0.006, wood, 1)
    ladder:shape("rung_" .. i, { box = { 0.4, 0.03, 0.09 }, pos = { p[1], p[2], p[3] }, friction = 0.9 })
  end
  kit.put(m, ladder, k)
  kit.far_box(m, ladder, "wood", { 0.44, H, 0.06 }, { 0, H / 2, 0.42 }, wood, far)
  ladder:marker("climb", { pos = { 0, 0, 1.1 }, turn = { 0, 180, 0 }, role = "stand" })
  ladder:marker("push", { pos = { 0.2, 1.1, 0.5 }, role = "hold" })
elseif a.kind == "dough_mixer" then
  c.box("paint", V(0, 0.06, 0.05), V(0.5, 0.12, 0.8), 0.02, CREAM, 2)
  c.box("paint", V(0, 0.6, -0.25), V(0.4, 1.0, 0.28), 0.03, CREAM, 2)                                -- the column
  c.box("paint", V(0, 1.0, 0.02), V(0.36, 0.22, 0.7), 0.05, CREAM, 3)                                -- the head, out over the bowl
  c.box("hard", V(0.201, 0.9, -0.25), V(0.004, 0.14, 0.1), 0.0, DARK, 1)
  c.box("light", V(0.204, 0.93, -0.25), V(0.002, 0.03, 0.03), 0.0, V(0.3, 1, 0.4), 1)
  kit.put(m, m, c)
  kit.far_box(m, m, "paint", { 0.5, 1.1, 0.8 }, { 0, 0.55, 0.05 }, CREAM, far)
  m:shape("body", { box = { 0.4, 1.1, 0.3 }, pos = { 0, 0.55, -0.25 } })
  m:shape("base", { box = { 0.5, 0.5, 0.5 }, pos = { 0, 0.25, 0.18 } })
  local bowl = m:node("bowl", { pos = { 0, 0.14, 0.18 }, motion = { turn = { 0, 1, 0 } } })
  local b = kit.maker()
  b.turned("metal", V(0, 0, 0), { { 0, 0 }, { 0.2, 0 }, { 0.26, 0.08 }, { 0.27, 0.4 }, { 0.26, 0.4 }, { 0.25, 0.09 }, { 0.19, 0.02 }, { 0, 0.02 } }, STEEL, 1, 28)
  b.turned("hard", V(0.03, 0.02, 0), { { 0, 0 }, { 0.2, 0 }, { 0.21, 0.12 }, { 0.1, 0.2 }, { 0, 0.21 } }, V(0.93, 0.88, 0.74), 0.85, 16)   -- the dough, off centre so the turning shows
  kit.put(m, bowl, b)
  local hook = m:node("hook", { pos = { 0.06, 0.89, 0.18 }, motion = { turn = { 0, -1, 0 } } })
  local h = kit.maker()
  h.pipe("metal", { { 0, 0, 0 }, { 0, -0.3, 0 }, { 0.08, -0.42, 0.03 }, { 0.03, -0.55, -0.06 }, { -0.06, -0.62, 0 } }, 0.014, STEEL, { bend = 0.06, sides = 8 })
  kit.put(m, hook, h)
  local guard = m:node("guard", { pos = { 0, 0.86, -0.1 }, motion = { turn = { -1, 0, 0 }, min = 0, max = 60 } })
  local g = kit.maker()
  for i = 0, 4 do g.pipe("metal", { { -0.28, 0, 0.02 + i * 0.001 }, { -0.28 + i * 0.02, -0.02 - i * 0.04, 0.3 }, { 0, -0.02 - i * 0.05, 0.56 }, { 0.28 - i * 0.02, -0.02 - i * 0.04, 0.3 }, { 0.28, 0, 0.02 + i * 0.001 } }, 0.004, STEEL, { bend = 0.12, sides = 5 }) end
  kit.put(m, guard, g)
  guard:marker("guard", { pos = { 0, -0.1, 0.56 }, role = "hold" })
elseif a.kind == "deck_oven" then
  local W, D, DECK, LEGS = 1.3, 1.0, 0.36, 0.5
  local H = LEGS + 3 * DECK + 0.12
  for _, x in ipairs{ -W / 2 + 0.05, W / 2 - 0.05 } do for _, z in ipairs{ -D / 2 + 0.05, D / 2 - 0.05 } do c.box("metal", V(x, LEGS / 2, z), V(0.06, LEGS, 0.06), 0.006, STEEL, 1) end end
  kit.carcass(c, "metal", W, H, D, 0.04, STEEL, LEGS)
  for i = 1, 2 do c.box("metal", V(0, LEGS + i * DECK, 0), V(W - 0.08, 0.04, D), 0.0, STEEL, 1) end
  for i = 0, 2 do
    c.box("hard", V(0, LEGS + i * DECK + 0.04, -0.02), V(W - 0.1, 0.02, D - 0.1), 0.0, V(0.55, 0.5, 0.45), 1)       -- the stone it bakes on
    for j = 0, 3 do c.cushion("hard", V(-0.42 + j * 0.28, LEGS + i * DECK + 0.1, 0.1), V(0.2, 0.1, 0.36), V(0.72, 0.48, 0.22), { r = 0.08, crown = 0.03 }) end   -- loaves
    c.box("light", V(0, LEGS + (i + 1) * DECK - 0.03, -D / 2 + 0.06), V(W - 0.2, 0.02, 0.02), 0.0, V(1, 0.6, 0.2), 1)
  end
  c.box("metal", V(W / 2 - 0.1, H - 0.06, D / 2 + 0.001), V(0.16, 0.08, 0.002), 0.0, DARK, 1)
  kit.put(m, m, c)
  kit.far_box(m, m, "metal", { W, H - LEGS, D }, { 0, (H + LEGS) / 2, 0 }, STEEL, far)
  m:shape("body", { box = { W, H - LEGS, D }, pos = { 0, (H + LEGS) / 2, 0 } })
  for i = 1, 3 do
    local n = m:node("door_" .. i, { pos = { 0, LEGS + (i - 1) * DECK + 0.04, D / 2 }, motion = { turn = { 1, 0, 0 }, min = 0, max = 90 } })
    local k = kit.maker()
    local h = DECK - 0.05
    for _, x in ipairs{ -W / 2 + 0.1, W / 2 - 0.1 } do k.box("metal", V(x, h / 2, 0.015), V(0.12, h, 0.03), 0.004, STEEL, 1) end
    for _, y in ipairs{ 0.03, h - 0.03 } do k.box("metal", V(0, y, 0.015), V(W - 0.32, 0.06, 0.03), 0.004, STEEL, 1) end
    k.box("glass", V(0, h / 2, 0.015), V(W - 0.32, h - 0.12, 0.008), 0.0, V(0.25, 0.2, 0.15), 1)
    k.rod("hard", V(-W / 2 + 0.2, h - 0.03, 0.06), V(W / 2 - 0.2, h - 0.03, 0.06), 0.012, DARK, 8)
    for _, x in ipairs{ -W / 2 + 0.22, W / 2 - 0.22 } do k.rod("metal", V(x, h - 0.03, 0.03), V(x, h - 0.03, 0.06), 0.006, STEEL, 6) end
    kit.put(m, n, k)
    n:shape("door_" .. i, { box = { W - 0.08, h, 0.03 }, pos = { 0, h / 2, 0.015 } })
    n:marker("door_" .. i, { pos = { 0, h - 0.03, 0.07 }, role = "hold" })
  end
  m:marker("front", { pos = { 0, 0, D / 2 + 0.9 }, turn = { 0, 180, 0 }, role = "stand" })
else
  c.box("paint", V(0, 0.04, 0), V(0.3, 0.08, 0.34), 0.02, kit.paint.red, 2)
  c.box("paint", V(0, 0.2, -0.12), V(0.06, 0.26, 0.05), 0.012, kit.paint.red, 2)                     -- the pillar the dial is on
  c.mesh("paint", mesh.translate(kit.facing(mesh.lathe{ profile = { { 0, -0.03 }, { 0.15, -0.03 }, { 0.15, 0.0 }, { 0, 0.0 } }, segments = 28 }), { 0, 0.42, -0.1 }), kit.paint.red)
  c.mesh("hard", mesh.translate(kit.facing(mesh.lathe{ profile = { { 0, 0 }, { 0.138, 0 }, { 0.138, 0.002 }, { 0, 0.002 } }, segments = 28 }), { 0, 0.42, -0.1 }), V(0.96, 0.95, 0.9))
  for i = 0, 9 do
    local t = math.rad(-135 + i * 30)
    c.box("hard", V(0.115 * math.sin(t), 0.42 + 0.115 * math.cos(t), -0.097), V(0.008, 0.008, 0.001), 0.0, kit.BLACK, 1)
  end
  kit.put(m, m, c)
  m:shape("body", { box = { 0.3, 0.5, 0.34 }, pos = { 0, 0.25, 0 } })
  local needle = m:node("needle", { pos = { 0, 0.42, -0.095 }, turn = { 0, 0, 135 }, motion = { turn = { 0, 0, -1 }, min = 0, max = 270 } })
  local n = kit.maker()
  n.box("hard", V(0, 0.05, 0), V(0.006, 0.13, 0.001), 0.0, kit.paint.red * 0.7, 1)
  kit.put(m, needle, n)
  local pan = m:node("pan", { pos = { 0, 0.11, 0.04 }, motion = { slide = { 0, -1, 0 }, min = 0, max = 0.02 } })
  local p = kit.maker()
  p.rod("metal", V(0, -0.03, 0), V(0, 0, 0), 0.012, STEEL, 6)
  p.turned("metal", V(0, 0, 0), { { 0, 0 }, { 0.1, 0 }, { 0.15, 0.03 }, { 0.145, 0.034 }, { 0.1, 0.008 }, { 0, 0.008 } }, STEEL, 1, 24)
  for i = 0, 4 do p.mesh("hard", mesh.translate(kit.ball(0.035, 10), { 0.05 * math.cos(i * 1.26), 0.04 + (i == 4 and 0.045 or 0), i == 4 and 0 or 0.05 * math.sin(i * 1.26) }), i % 2 == 0 and kit.paint.orange or kit.paint.red) end   -- fruit on it
  kit.put(m, pan, p)
  pan:marker("pan", { pos = { 0, 0.03, 0 } })
end
return m
