-- A MONITOR ON AN ARM: a screen on a jointed arm clamped to a desk's back edge (#191). Four joints:
--
--   shoulder   the first arm swings about the post
--   elbow      the second arm swings about the end of the first
--   pan        the screen turns to face
--   tilt       the screen nods, 25 degrees either way
--
-- As placed the arm is folded so the screen sits over the post, facing +z. The post's foot is the origin: put it
-- on a desk's top (a standing desk's marker `surface`).
local kit = require "model_kit"
local V = kit.V

local a = params{
  screen = { 0.62, min = 0.4, max = 1.0 },        -- the screen's width (metres); it is 16 wide to 9 tall
  reach  = { 0.24, min = 0.15, max = 0.4 },       -- each arm's length
  post   = { 0.3, min = 0.15, max = 0.5 },        -- the post's height
  finish = { "black", "white", "steel" },
}
local SW, SH, L, P = a.screen, a.screen * 9 / 16, a.reach, a.post
local col = ({ black = kit.BLACK * 1.5, white = kit.WHITE, steel = kit.STEEL })[a.finish]

local m = model.new()
local far = m:level(25)
local FOLD = 50                               -- how far each arm is folded as placed (degrees)

local base = kit.maker()
base.box("paint", V(0, 0.006, 0), V(0.1, 0.012, 0.1), 0.004, col, 1)                  -- the clamp's plate
base.box("paint", V(0, -0.02, -0.06), V(0.06, 0.06, 0.016), 0.004, col, 1)            -- and its jaw, over the desk's edge
base.turned("paint", V(0, 0, 0), { { 0.018, 0.012 }, { 0.018, P }, { 0.0, P } }, col, 1, 14)
kit.put(m, m, base)
m:shape("post", { capsule = { radius = 0.02, height = P }, pos = { 0, P / 2, 0 } })

local function arm(k)                         -- an arm along +z from its own origin, with a knuckle at each end
  k.box("paint", V(0, 0, L / 2), V(0.04, 0.03, L), 0.01, col, 1)
  k.turned("paint", V(0, -0.02, 0), { { 0.026, 0 }, { 0.026, 0.04 }, { 0, 0.04 } }, col, 1, 12)
  k.turned("paint", V(0, -0.02, L), { { 0.026, 0 }, { 0.026, 0.04 }, { 0, 0.04 } }, col, 1, 12)
end
local shoulder = m:node("shoulder", { pos = { 0, P - 0.03, 0 }, turn = { 0, FOLD, 0 }, motion = { turn = { 0, 1, 0 }, min = -120, max = 120 } })
local k1 = kit.maker(); arm(k1); kit.put(m, shoulder, k1)
local elbow = shoulder:node("elbow", { pos = { 0, 0.04, L }, turn = { 0, -2 * FOLD, 0 }, motion = { turn = { 0, 1, 0 }, min = -150, max = 150 } })
local k2 = kit.maker(); arm(k2); kit.put(m, elbow, k2)
local pan = elbow:node("pan", { pos = { 0, 0, L }, turn = { 0, FOLD, 0 }, motion = { turn = { 0, 1, 0 }, min = -90, max = 90 } })
local tilt = pan:node("tilt", { pos = { 0, 0.03, 0.03 }, motion = { turn = { 1, 0, 0 }, min = -25, max = 25 } })
local s = kit.maker()
s.box("paint", V(0, 0, 0.012), V(0.1, 0.1, 0.02), 0.004, col, 1)                      -- the plate on the screen's back
s.box("hard", V(0, 0, 0.035), V(SW, SH, 0.022), 0.006, kit.BLACK, 1)                  -- the screen's case
s.box("screen", V(0, 0.004, 0.0465), V(SW - 0.02, SH - 0.03, 0.001), 0.0, V(0.02, 0.03, 0.05), 1)
kit.put(m, tilt, s)
kit.far_box(m, tilt, "hard", { SW, SH, 0.022 }, { 0, 0, 0.035 }, kit.BLACK, far)
tilt:shape("screen", { box = { SW, SH, 0.03 }, pos = { 0, 0, 0.035 } })
tilt:marker("screen", { pos = { 0, 0, 0.048 }, role = "screen", size = SW })
return m
