-- THINGS IN A BAR OR A CLUB THAT TURN (#191):
--
--   mirror_ball   a ball of little mirrors on a motor, HANGING from its origin (put the origin on the ceiling);
--                 the ball is on a node, `ball`, that turns without end. A marker `lamp` beside it for the spot.
--   dj_deck       a booth's table with two turntables and a mixer; each record is on a node, `record_1`,
--                 `record_2`, turning without end, and the crossfader slides (`fader`). It stands on y = 0.
--   beer_taps     a tower of taps for a bar's counter: each handle is on a node, `tap_1` ..., pulling 50 degrees.
--                 Put its origin on the counter's top.
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind = { "mirror_ball", "dj_deck", "beer_taps" },
  size = { 0.5, min = 0.2, max = 1.2 },           -- mirror ball: across (metres)
  taps = { 4, min = 1, max = 8, step = 1 },
}
local m = model.new()
local far = m:level(40)
local c = kit.maker()
if a.kind == "mirror_ball" then
  local R = a.size / 2
  c.turned("hard", V(0, -0.12, 0), { { 0, 0 }, { 0.05, 0 }, { 0.06, 0.12 }, { 0, 0.12 } }, kit.BLACK * 1.5, 1, 12)
  c.rod("metal", V(0, -0.12, 0), V(0, -0.3, 0), 0.006, kit.CHROME, 6)
  kit.put(m, m, c)
  m:marker("lamp", { pos = { 1.5, -0.1, 0 }, turn = { 0, 90, 10 }, role = "lamp" })
  local ball = m:node("ball", { pos = { 0, -0.3 - R, 0 }, motion = { turn = { 0, 1, 0 } } })
  local k = kit.maker()
  -- flat-sided so each little mirror is its own facet
  k.mesh("metal", mesh.flat_shade(mesh.lathe{ profile = (function() local p = {} for i = 0, 12 do local t = math.pi * i / 12 - math.pi / 2; p[#p + 1] = { math.max(0, R * math.cos(t)), R * math.sin(t) } end return p end)(),
                                             segments = 24 }), V(0.9, 0.9, 0.92))
  kit.put(m, ball, k)
  ball:shape("ball", { sphere = R })
elseif a.kind == "dj_deck" then
  local W, H, D = 1.4, 0.95, 0.6
  c.box("paint", V(0, H / 2 - 0.02, 0), V(W, H - 0.04, D), 0.01, kit.BLACK * 1.6, 1)
  c.box("light", V(0, H * 0.5, D / 2 + 0.002), V(W - 0.2, 0.3, 0.004), 0.0, V(0.6, 0.2, 1), 1)                 -- a lit panel on the booth's front
  c.box("hard", V(0, H + 0.02, -0.02), V(0.3, 0.05, 0.4), 0.008, kit.BLACK * 2, 1)                            -- the mixer
  for i = 0, 5 do c.turned("metal", V(-0.1 + (i % 3) * 0.1, H + 0.045, -0.12 + math.floor(i / 3) * 0.1), { { 0, 0 }, { 0.014, 0 }, { 0.012, 0.02 }, { 0, 0.02 } }, kit.CHROME, 1, 8) end
  kit.put(m, m, c)
  m:shape("booth", { box = { W, H, D }, pos = { 0, H / 2, 0 } })
  m:marker("dj", { pos = { 0, 0, -D / 2 - 0.5 }, role = "stand" })
  m:marker("sound", { pos = { 0, H, 0 } })
  for i, x in ipairs{ -0.45, 0.45 } do
    local base = kit.maker()
    base.box("hard", V(x, H + 0.02, -0.02), V(0.42, 0.05, 0.36), 0.008, V(0.75, 0.76, 0.78), 1)
    base.pipe("metal", { { x + 0.17, H + 0.06, -0.15 }, { x + 0.17, H + 0.075, -0.15 }, { x + 0.06, H + 0.075, 0.04 } }, 0.005, kit.CHROME, { bend = 0.02, sides = 6 })   -- the arm
    kit.put(m, m, base)
    local rec = m:node("record_" .. i, { pos = { x - 0.03, H + 0.045, -0.02 }, motion = { turn = { 0, -1, 0 } } })
    local k = kit.maker()
    k.turned("hard", V(0, 0, 0), { { 0, 0 }, { 0.15, 0 }, { 0.15, 0.006 }, { 0, 0.006 } }, kit.BLACK, 1, 32)
    k.turned("paint", V(0, 0.006, 0), { { 0, 0 }, { 0.05, 0 }, { 0.05, 0.001 }, { 0, 0.001 } }, i == 1 and kit.paint.red or kit.paint.yellow, 1, 16)
    k.box("ceramic", V(0.03, 0.0075, 0), V(0.03, 0.001, 0.008), 0.0, kit.WHITE, 1)       -- a mark on the label, so the turning shows
    kit.put(m, rec, k)
  end
  local fader = m:node("fader", { pos = { 0, H + 0.045, 0.13 }, motion = { slide = { 1, 0, 0 }, min = -0.06, max = 0.06 } })
  local f = kit.maker()
  f.box("metal", V(0, 0.01, 0), V(0.014, 0.02, 0.03), 0.003, kit.CHROME, 1)
  kit.put(m, fader, f)
else
  local N = math.floor(a.taps + 0.5)
  local W = N * 0.09 + 0.06
  local BRASS = V(0.8, 0.62, 0.25)
  c.turned("metal", V(0, 0, 0), { { 0, 0 }, { 0.07, 0 }, { 0.07, 0.015 }, { 0.035, 0.03 }, { 0.035, 0.3 }, { 0, 0.3 } }, BRASS, 1, 16)
  c.pipe("metal", { { -W / 2, 0.34, 0 }, { W / 2, 0.34, 0 } }, 0.04, BRASS, { sides = 12 })
  c.box("metal", V(0, 0.005, 0.12), V(W + 0.06, 0.01, 0.16), 0.0, V(0.7, 0.71, 0.73), 1)   -- the drip tray
  kit.put(m, m, c)
  m:shape("tower", { box = { W, 0.4, 0.1 }, pos = { 0, 0.2, 0 } })
  local badges = { kit.paint.red, kit.paint.green, kit.paint.yellow, kit.paint.blue, kit.WHITE, kit.paint.orange, kit.BLACK * 2, kit.paint.red }
  for i = 1, N do
    local x = -W / 2 + 0.075 + (i - 1) * 0.09
    local spout = kit.maker()
    spout.pipe("metal", { { x, 0.34, 0.03 }, { x, 0.34, 0.08 }, { x, 0.26, 0.1 } }, 0.011, kit.CHROME, { bend = 0.02, sides = 8 })
    kit.put(m, m, spout)
    local tap = m:node("tap_" .. i, { pos = { x, 0.37, 0.06 }, turn = { -12, 0, 0 }, motion = { turn = { 1, 0, 0 }, min = 0, max = 50 } })
    local k = kit.maker()
    k.rod("metal", V(0, 0, 0), V(0, 0.05, 0), 0.006, kit.CHROME, 6)
    k.box("paint", V(0, 0.13, 0), V(0.035, 0.16, 0.02), 0.008, badges[i], 1)
    kit.put(m, tap, k)
    tap:marker("tap_" .. i, { pos = { 0, 0.18, 0 }, role = "hold" })
    m:marker("glass_" .. i, { pos = { x, 0.01, 0.1 } })
  end
end
return m
