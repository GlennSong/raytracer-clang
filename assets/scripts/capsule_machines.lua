-- CAPSULE TOY MACHINES (#191; the rows of them outside a shop or in a mall): a stack of machines, each a clear
-- bin of coloured capsules over a coin slot, a knob that turns, and a chute the capsule drops from. Each knob is
-- on a node, `knob_C_R` (column, then row from the foot), turning without end; a marker `chute_C_R` under it.
-- It stands on y = 0 at its own origin, facing +z.
local kit = require "model_kit"
local scatter = require "scatter"
local V = kit.V

local a = params{
  columns = { 3, min = 1, max = 8, step = 1 },
  rows    = { 2, min = 1, max = 3, step = 1 },
}
local C, R = math.floor(a.columns + 0.5), math.floor(a.rows + 0.5)
local m = model.new()
local far = m:level(40)
local UW, UH, D = 0.34, 0.62, 0.36               -- one machine
local W = C * UW
local shells = { kit.paint.red, kit.paint.blue, kit.paint.yellow, kit.paint.green, kit.paint.orange, kit.WHITE }
local c = kit.maker()
c.box("paint", V(0, 0.04, 0), V(W, 0.08, D), 0.01, kit.BLACK * 1.6, 1)
for i = 1, C do
  for j = 1, R do
    local x, y = -W / 2 + (i - 0.5) * UW, 0.08 + (j - 1) * UH
    local body = shells[((i - 1) + (j - 1) * 2) % #shells + 1]
    c.box("paint", V(x, y + 0.14, 0), V(UW - 0.01, 0.28, D), 0.012, body, 1)                    -- the base: coin slot, knob, chute
    c.box("glass", V(x, y + 0.44, 0), V(UW - 0.03, 0.32, D - 0.02), 0.0, V(0.9, 0.95, 0.95), 1)   -- the bin
    c.box("paint", V(x, y + 0.6, 0), V(UW - 0.01, 0.02, D), 0.006, body, 1)
    c.box("light", V(x, y + 0.5, D / 2 - 0.006), V(UW - 0.1, 0.12, 0.002), 0.0, V(1, 0.95, 0.8), 1)   -- the picture of what is inside
    c.box("hard", V(x, y + 0.05, D / 2 - 0.02), V(0.12, 0.07, 0.05), 0.0, kit.BLACK, 1)          -- the chute
    c.box("metal", V(x + 0.09, y + 0.2, D / 2 + 0.001), V(0.03, 0.05, 0.004), 0.0, kit.CHROME, 1) -- the coin slot
    -- the capsules: two colours, each stored once
    for half = 0, 1 do
      local ball = model.new()
      ball:add(mesh.paint(mesh.sphere(0.03), shells[(i + j + half * 3) % #shells + 1]))
      m:use(ball, scatter.jitter(scatter.grid{ origin = { x - UW / 2 + 0.03, y + 0.3 + half * 0.07, -D / 2 + 0.03 }, u = { UW - 0.06, 0, 0 }, v = { 0, 0, D - 0.06 },
                                               count = { 4, 4 }, margin = 0.035 }, { seed = i * 7 + j * 3 + half, move = 0.012 }))
    end
    local knob = m:node(string.format("knob_%d_%d", i, j), { pos = { x - 0.03, y + 0.17, D / 2 }, motion = { turn = { 0, 0, -1 } } })
    local k = kit.maker()
    k.mesh("hard", mesh.rotate_x(mesh.lathe{ profile = { { 0, 0 }, { 0.055, 0 }, { 0.055, 0.02 }, { 0, 0.02 } }, segments = 16 }, math.pi / 2), kit.WHITE)
    k.box("hard", V(0, 0, 0.03), V(0.1, 0.025, 0.025), 0.008, kit.WHITE, 1)                      -- the bar to grip
    kit.put(m, knob, k)
    knob:marker(string.format("knob_%d_%d", i, j), { pos = { 0, 0, 0.05 }, role = "hold" })
    m:marker(string.format("chute_%d_%d", i, j), { pos = { x, y + 0.05, D / 2 + 0.02 }, role = "hold" })
  end
end
kit.put(m, m, c)
local H = 0.08 + R * UH
kit.far_box(m, m, "paint", { W, H, D }, { 0, H / 2, 0 }, shells[1], far)
m:shape("body", { box = { W, H, D }, pos = { 0, H / 2, 0 } })
m:marker("front", { pos = { 0, 0, D / 2 + 0.6 }, turn = { 0, 180, 0 }, role = "stand" })
return m
