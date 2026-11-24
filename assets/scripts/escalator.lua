-- AN ESCALATOR whose steps move (#191; a mall's, a department store's, a station's). Every step is on a node,
-- `step_1` ..., and slides up the slope by exactly one step's place, so that when each jumps back the picture is
-- the same: a prefab's script runs them round like that (escalator_run.lua, model_moves.lua "loop") and the stairs
-- flow without end. Glass sides, a rubber hand rail, a landing plate at each end.
-- Markers: `foot` and `head` on the landings, `stop` (the red button).
-- It stands on y = 0 at its own origin, which is where the steps come out of the floor; it climbs toward +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  rise  = { 4.0, min = 2.0, max = 8.0 },          -- the height it climbs (metres)
  width = { 1.0, min = 0.6, max = 1.2 },          -- a step's width
  slope = { 30, min = 25, max = 35 },             -- degrees
}
local H, W = a.rise, a.width
local N = math.max(4, math.floor(H / 0.2 + 0.5))  -- how many steps show at once
local R = H / N                                   -- one step's rise
local Dp = R / math.tan(math.rad(a.slope))        -- and its depth
local RUN = N * Dp                                -- the length of floor under the slope
local STEEL, GLASS = V(0.68, 0.7, 0.72), V(0.8, 0.9, 0.92)
local m = model.new()
local far = m:level(80)

local SIDE = 0.12                                 -- each side's thickness
local c = kit.maker()
-- the landings: a comb plate at each end, level with the floor it meets
c.box("metal", V(0, -0.01, -0.6), V(W + 2 * SIDE, 0.02, 1.2), 0.0, STEEL * 0.8, 1)
c.box("metal", V(0, H - 0.01, RUN + Dp + 0.6), V(W + 2 * SIDE, 0.02, 1.2), 0.0, STEEL * 0.8, 1)
m:shape("landing_foot", { box = { W + 2 * SIDE, 0.02, 1.2 }, pos = { 0, -0.01, -0.6 } })
m:shape("landing_head", { box = { W + 2 * SIDE, 0.3, 1.2 }, pos = { 0, H - 0.15, RUN + Dp + 0.6 } })
local len = math.sqrt(RUN * RUN + H * H)
local pitch = math.deg(math.atan(H, RUN))
for _, s in ipairs{ -1, 1 } do
  local x = s * (W / 2 + SIDE / 2)
  -- the skirt beside the steps and the truss under them, as one sloping slab, and the glass above it
  local slab = mesh.translate(mesh.rotate_x(mesh.box{ SIDE, 0.7, len + 1.0 }, -math.rad(pitch)), { x, H / 2 - 0.1, RUN / 2 + Dp / 2 })
  c.mesh("metal", slab, STEEL)
  local glass = mesh.translate(mesh.rotate_x(mesh.box{ 0.012, 0.75, len + 0.4 }, -math.rad(pitch)), { x, H / 2 + 0.62, RUN / 2 + Dp / 2 })
  c.mesh("glass", glass, GLASS)
  -- the hand rail: along the top of the glass and round each end
  local top0, top1 = { x, 1.0, -0.5 }, { x, H + 1.0, RUN + Dp + 0.5 }
  c.pipe("rubber", { { x, 0.25, -0.75 }, { x, 1.0, -0.75 }, top0, { x, 1.0, 0.1 }, { x, H + 1.0, RUN + Dp - 0.1 }, top1, { x, H + 1.0, RUN + Dp + 0.75 }, { x, H + 0.25, RUN + Dp + 0.75 } },
         0.035, kit.BLACK, { bend = 0.2, sides = 8 })
  for _, e in ipairs{ { -0.4, 0 }, { RUN + Dp + 0.4, H } } do                       -- the newel at each end
    c.box("metal", V(x, e[2] + 0.45, e[1]), V(SIDE, 0.9, 0.7), 0.02, STEEL, 1)
    m:shape(string.format("newel_%s_%s", s < 0 and "left" or "right", e[2] == 0 and "foot" or "head"), { box = { SIDE, 1.0, 0.8 }, pos = { x, e[2] + 0.5, e[1] } })
  end
  m:shape("side_" .. (s < 0 and "left" or "right"), { box = { SIDE, 1.7, len + 0.4 }, pos = { x, H / 2 + 0.3, RUN / 2 + Dp / 2 }, turn = { -pitch, 0, 0 } })
end
kit.put(m, m, c)
kit.far_box(m, m, "metal", { W + 2 * SIDE, 0.8, len }, { 0, H / 2, RUN / 2 }, STEEL, far)
m:shape("ramp", { box = { W, 0.1, len }, pos = { 0, H / 2 - 0.12, RUN / 2 + Dp / 2 }, turn = { -pitch, 0, 0 }, friction = 0.9 })   -- under the steps, so nothing falls through
m:marker("foot", { pos = { 0, 0, -0.8 }, role = "stand" })
m:marker("head", { pos = { 0, H, RUN + Dp + 0.8 }, role = "stand" })
m:marker("stop", { pos = { W / 2 + SIDE / 2, 0.5, -0.75 }, role = "press" })
-- how far a step goes before it has taken the next one's place: from `run_from` to `run_to` (the script reads these)
m:marker("run_from", { pos = { 0, 0, 0 } })
m:marker("run_to", { pos = { 0, R, Dp } })

-- THE STEPS: one more than show, so the lowest is coming up out of the floor as the highest goes under the plate
local slide = { 0, R, Dp }
local span = math.sqrt(R * R + Dp * Dp)
for i = 0, N do
  local step = m:node("step_" .. (i + 1), { pos = { 0, (i - 1) * R, (i - 1) * Dp }, motion = { slide = slide, min = 0, max = span } })
  local k = kit.maker()
  k.box("metal", V(0, -R / 2 - 0.01, Dp / 2), V(W - 0.01, R + 0.02, Dp), 0.0, V(0.3, 0.31, 0.33), 1)
  for g = 0, 9 do k.box("metal", V(-W / 2 + (g + 0.5) * W / 10, 0.002, Dp / 2), V(W / 10 * 0.5, 0.004, Dp), 0.0, V(0.5, 0.51, 0.53), 1) end   -- the treads' grooves
  k.box("paint", V(0, 0.003, Dp - 0.015), V(W - 0.02, 0.004, 0.03), 0.0, kit.paint.yellow, 1)                                             -- the yellow edge
  kit.put(m, step, k)
  step:shape("step_" .. (i + 1), { box = { W, R, Dp }, pos = { 0, -R / 2, Dp / 2 }, friction = 0.9 })
end
return m
