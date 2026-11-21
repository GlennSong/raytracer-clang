-- A DESK LAMP ON A JOINTED ARM (#191): a weighted base, two arms and a shade, with four joints:
--
--   swing    the whole arm turns on the base
--   lower    the first arm leans
--   upper    the second arm bends at the elbow
--   head     the shade nods
--
-- The marker `lamp` is where its light is and points the way it shines. It stands on y = 0 at its own origin.
local kit = require "model_kit"
local V = kit.V

local a = params{
  arm    = { 0.32, min = 0.2, max = 0.5 },        -- each arm's length (metres)
  finish = { "black", "white", "red", "green", "steel" },
}
local L = a.arm
local col, part = kit.finish(a.finish)

local m = model.new()
local far = m:level(20)
local base = kit.maker()
base.turned("paint", V(0, 0, 0), { { 0.0, 0.0 }, { 0.085, 0.0 }, { 0.085, 0.018 }, { 0.07, 0.028 }, { 0.0, 0.03 } }, col, 1, 20)
kit.put(m, m, base)
m:shape("base", { box = { 0.17, 0.03, 0.17 }, pos = { 0, 0.015, 0 } })

local function arm(k)                         -- two thin rods side by side, up +y from the origin
  for _, x in ipairs{ -0.012, 0.012 } do k.rod("metal", V(x, 0, 0), V(x, L, 0), 0.004, kit.CHROME, 6) end
  k.rod("paint", V(-0.02, 0, 0), V(0.02, 0, 0), 0.009, col, 8)
  k.rod("paint", V(-0.02, L, 0), V(0.02, L, 0), 0.009, col, 8)
end
local swing = m:node("swing", { pos = { 0, 0.03, 0 }, motion = { turn = { 0, 1, 0 } } })
local lower = swing:node("lower", { pos = { 0, 0.02, 0 }, turn = { -25, 0, 0 }, motion = { turn = { 1, 0, 0 }, min = -40, max = 60 } })
local k1 = kit.maker(); arm(k1); kit.put(m, lower, k1)
local upper = lower:node("upper", { pos = { 0, L, 0 }, turn = { 95, 0, 0 }, motion = { turn = { 1, 0, 0 }, min = -60, max = 60 } })
local k2 = kit.maker(); arm(k2); kit.put(m, upper, k2)
local head = upper:node("head", { pos = { 0, L, 0 }, turn = { 50, 0, 0 }, motion = { turn = { 1, 0, 0 }, min = -60, max = 60 } })
local h = kit.maker()
-- the shade: a cone open at its wide end, which points along the head's +y; its lining faces in
h.turned("paint", V(0, 0, 0), { { 0.0, 0.0 }, { 0.03, 0.0 }, { 0.035, 0.05 }, { 0.075, 0.13 } }, col, 1, 18)
h.turned("paint", V(0, 0, 0), { { 0.073, 0.13 }, { 0.033, 0.05 }, { 0.0, 0.01 } }, kit.WHITE, 1, 18)
h.turned("light", V(0, 0, 0), { { 0.0, 0.04 }, { 0.02, 0.05 }, { 0.028, 0.085 }, { 0.0, 0.11 } }, V(1, 0.95, 0.8), 1, 12)
kit.put(m, head, h)
head:marker("lamp", { pos = { 0, 0.1, 0 }, turn = { -90, 0, 0 }, role = "lamp" })
head:marker("switch", { pos = { 0, 0.01, 0 }, role = "press" })
kit.far_box(m, m, "paint", { 0.1, 0.4, 0.1 }, { 0, 0.2, 0 }, col, far)
return m
