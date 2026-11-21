-- A TALL CASE CLOCK (#191): a pendulum swinging behind a glass door (`pendulum`), and three hands on its dial
-- (`hour`, `minute`, `second`), each turning without end from twelve, as the wall clock's do (wall_clock.lua);
-- grandfather_clock_run.lua keeps its time. Two brass weights hang beside the pendulum.
-- It stands on y = 0 at its own origin, its dial toward +z.
local kit = require "model_kit"
local V = kit.V

local a = params{ wood = { "walnut", "oak", "birch", "ebonised" } }
local wood = kit.wood(a.wood)
local BRASS = kit.metal.brass
local W, D, FOOT, HOOD = 0.46, 0.26, 0.5, 1.52          -- the hood begins at HOOD and holds the dial
local m = model.new()
local far = m:level(40)
local c = kit.maker()
c.box("wood", V(0, FOOT / 2, 0), V(W, FOOT, D), 0.012, wood, 2)
kit.carcass(c, "wood", W - 0.1, HOOD, D - 0.05, 0.02, wood, FOOT)                         -- the trunk the pendulum hangs in
c.box("glass", V(0, (FOOT + HOOD) / 2, D / 2 - 0.03), V(W - 0.14, HOOD - FOOT - 0.06, 0.005), 0.0, V(0.8, 0.88, 0.9), 1)
c.box("wood", V(0, HOOD + 0.25, 0), V(W, 0.5, D), 0.012, wood, 2)
c.box("wood", V(0, HOOD + 0.52, 0), V(W + 0.06, 0.05, D + 0.05), 0.012, wood, 2)
c.box("wood", V(0, HOOD - 0.01, 0), V(W + 0.03, 0.03, D + 0.03), 0.008, wood, 1)
local DIAL_Y, R, FACE = HOOD + 0.25, 0.17, D / 2 + 0.002
c.mesh("hard", mesh.translate(kit.facing(mesh.lathe{ profile = { { 0, 0 }, { R, 0 }, { R, 0.002 }, { 0, 0.002 } }, segments = 32 }), { 0, DIAL_Y, FACE - 0.001 }), V(0.95, 0.92, 0.82))
c.mesh("metal", mesh.translate(kit.facing(mesh.lathe{ profile = { { R, 0 }, { R + 0.012, 0 }, { R + 0.012, 0.006 }, { R, 0.006 } }, segments = 32 }), { 0, DIAL_Y, FACE - 0.001 }), BRASS)
for i = 0, 11 do
  local t = 2 * math.pi * i / 12
  local s = i % 3 == 0 and 0.022 or 0.012
  c.box("hard", V((R - 0.025) * math.sin(t), DIAL_Y + (R - 0.025) * math.cos(t), FACE + 0.0015), V(s, s, 0.001), 0.0, kit.BLACK, 1)
end
for _, x in ipairs{ -0.09, 0.09 } do                                                      -- the weights, on their chains
  c.turned("metal", V(x, 0.95, -0.03), { { 0, 0 }, { 0.025, 0 }, { 0.025, 0.2 }, { 0, 0.2 } }, BRASS, 1, 12)
  c.rod("metal", V(x, 1.15, -0.03), V(x, HOOD, -0.03), 0.002, BRASS, 4)
end
kit.put(m, m, c)
kit.far_box(m, m, "wood", { W, HOOD + 0.55, D }, { 0, (HOOD + 0.55) / 2, 0 }, wood, far)
m:shape("case", { box = { W, HOOD + 0.55, D }, pos = { 0, (HOOD + 0.55) / 2, 0 } })
m:marker("tick", { pos = { 0, DIAL_Y, FACE } })

local function hand(name, len, wide, z, col)
  local n = m:node(name, { pos = { 0, DIAL_Y, FACE + z }, motion = { turn = { 0, 0, -1 } } })
  local k = kit.maker()
  k.box("hard", V(0, len / 2 - 0.01, 0), V(wide, len + 0.02, 0.0015), 0.0, col, 1)
  kit.put(m, n, k)
end
hand("hour", R * 0.55, 0.014, 0.004, kit.BLACK)
hand("minute", R * 0.85, 0.009, 0.006, kit.BLACK)
hand("second", R * 0.9, 0.003, 0.008, V(0.7, 0.1, 0.08))

local pendulum = m:node("pendulum", { pos = { 0, HOOD - 0.05, 0.02 }, motion = { turn = { 0, 0, 1 }, min = -8, max = 8 } })
local p = kit.maker()
p.rod("metal", V(0, 0, 0), V(0, -0.72, 0), 0.004, BRASS, 6)
p.mesh("metal", mesh.translate(kit.facing(mesh.lathe{ profile = { { 0, -0.008 }, { 0.05, -0.008 }, { 0.065, 0 }, { 0.05, 0.008 }, { 0, 0.008 } }, segments = 20, smooth = true }), { 0, -0.74, 0 }), BRASS)
kit.put(m, pendulum, p)
return m
