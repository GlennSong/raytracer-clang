-- AN ELECTRIC FAN: on a desk, on a tall stand, or hung from a ceiling (#191). Its blades are on a node, `blades`,
-- that turns without end. A desk or stand fan's head is on a node, `head`, that sweeps 50 degrees either way; a
-- ceiling fan has no head. A desk or stand fan stands on y = 0 at its own origin, blowing toward +z; a ceiling fan
-- HANGS from its origin (put the origin on the ceiling).
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind   = { "desk", "stand", "ceiling" },
  blades = { 3, min = 2, max = 6, step = 1 },
  size   = { 0.2, min = 0.12, max = 0.8 },        -- a blade's length (metres): 0.2 a desk fan, 0.6 a ceiling fan
  finish = { "white", "black", "steel", "walnut" },
}
local count, R = math.floor(a.blades + 0.5), a.size
local col, part = kit.finish(a.finish)
local m = model.new()
local far = m:level(30)

-- one blade lying along +x from the hub, wider toward its tip, pitched a little
local function blade(k, part, len, wide, c)
  local P = mesh.rounded_box({ len, 0.004, wide }, 0.0)
  P = mesh.move_by(P, function(_, p) return { p[1], p[2], p[3] * (0.6 + 0.4 * (p[1] / len + 0.5)) } end)
  k.mesh(part, mesh.translate(mesh.rotate_x(P, math.rad(14)), { len / 2 + 0.03, 0, 0 }), c)
end

if a.kind == "ceiling" then
  local rod = kit.maker()
  rod.turned("paint", V(0, -0.05, 0), { { 0.0, 0.05 }, { 0.06, 0.05 }, { 0.05, 0.0 }, { 0.014, 0.0 } }, col, 1, 14)   -- the cup on the ceiling
  rod.rod("paint", V(0, -0.05, 0), V(0, -0.28, 0), 0.014, col, 10)
  rod.turned("paint", V(0, -0.4, 0), { { 0.0, 0.0 }, { 0.07, 0.01 }, { 0.09, 0.06 }, { 0.07, 0.12 }, { 0.0, 0.12 } }, col, 1, 18)   -- the motor
  rod.turned("light", V(0, -0.45, 0), { { 0.0, 0.0 }, { 0.05, 0.02 }, { 0.06, 0.05 }, { 0.0, 0.05 } }, V(1, 0.95, 0.85), 1, 14)
  kit.put(m, m, rod)
  local blades = m:node("blades", { pos = { 0, -0.33, 0 }, motion = { turn = { 0, 1, 0 } } })
  local k = kit.maker()
  for i = 1, count do
    local one = kit.maker()
    blade(one, "x", R, R * 0.24, col)
    one.box("x", V(0.06, 0, 0), V(0.1, 0.006, 0.03), 0.0, col, 1)
    k.put(part, mesh.rotate_y(mesh.merge(one.lists.x), 2 * math.pi * i / count))
  end
  kit.put(m, blades, k)
  kit.far_box(m, blades, "paint", { 2 * R, 0.01, 2 * R }, { 0, 0, 0 }, col, far)
  m:marker("lamp", { pos = { 0, -0.46, 0 }, turn = { 90, 0, 0 }, role = "lamp" })
  m:shape("motor", { box = { 0.18, 0.45, 0.18 }, pos = { 0, -0.225, 0 } })
  return m
end

local TOP = a.kind == "stand" and 1.25 or R + 0.14     -- the height of the head's pivot
local s = kit.maker()
local foot = a.kind == "stand" and 0.22 or R * 0.7
s.turned("paint", V(0, 0, 0), { { 0.0, 0.0 }, { foot, 0.0 }, { foot, 0.015 }, { foot * 0.3, 0.035 }, { 0.016, 0.05 }, { 0.016, TOP - 0.03 }, { 0.0, TOP - 0.03 } }, col, 1, 20)
kit.put(m, m, s)
m:shape("stand", { capsule = { radius = 0.03, height = TOP }, pos = { 0, TOP / 2, 0 } })
m:shape("foot", { box = { foot * 1.6, 0.03, foot * 1.6 }, pos = { 0, 0.015, 0 } })
m:marker("switch", { pos = { 0, 0.04, foot * 0.6 }, role = "press" })

local head = m:node("head", { pos = { 0, TOP, 0 }, motion = { turn = { 0, 1, 0 }, min = -50, max = 50 } })
local h = kit.maker()
h.mesh("paint", mesh.rotate_x(mesh.lathe{ profile = { { 0.0, -0.11 }, { 0.04, -0.1 }, { 0.055, -0.04 }, { 0.05, 0.0 }, { 0.0, 0.0 } }, segments = 14, smooth = true }, math.pi / 2), col)   -- the motor, behind
-- the cage: rings in front and behind, and wires from the middle out
local G = R + 0.035
for _, z in ipairs{ -0.035, 0.0, 0.05, 0.075 } do
  local r = (z == 0.075 or z == -0.035) and G * 0.75 or G
  local ring = {}
  for i = 0, 23 do local t = 2 * math.pi * i / 24; ring[#ring + 1] = { r * math.cos(t), r * math.sin(t), z } end
  h.pipe("metal", ring, 0.003, kit.CHROME, { closed = true, sides = 5 })
end
for i = 0, 15 do
  local t = 2 * math.pi * i / 16
  local c_, s_ = math.cos(t), math.sin(t)
  h.pipe("metal", { { 0.03 * c_, 0.03 * s_, 0.085 }, { G * 0.75 * c_, G * 0.75 * s_, 0.075 }, { G * c_, G * s_, 0.05 }, { G * c_, G * s_, 0.0 },
                    { G * 0.75 * c_, G * 0.75 * s_, -0.035 } }, 0.0018, kit.CHROME, { sides = 4 })
end
h.mesh("paint", mesh.translate(mesh.rotate_x(mesh.lathe{ profile = { { 0.0, 0.0 }, { 0.035, 0.0 }, { 0.03, 0.008 }, { 0.0, 0.008 } }, segments = 12 }, math.pi / 2), { 0, 0, 0.08 }), col)
kit.put(m, head, h)
head:shape("cage", { sphere = G, pos = { 0, 0, 0.02 } })
kit.far_box(m, head, "metal", { 2 * G, 2 * G, 0.1 }, { 0, 0, 0.02 }, kit.CHROME, far)

local blades = head:node("blades", { pos = { 0, 0, 0.02 }, motion = { turn = { 0, 0, 1 } } })
local k = kit.maker()
k.mesh("hard", mesh.rotate_x(mesh.lathe{ profile = { { 0.0, 0.0 }, { 0.03, 0.0 }, { 0.026, 0.03 }, { 0.0, 0.035 } }, segments = 12, smooth = true }, math.pi / 2), col)
for i = 1, count do
  local one = kit.maker()
  blade(one, "x", R - 0.03, R * 0.5, V(0.75, 0.8, 0.85))
  -- a blade made lying flat is stood up to face +z, then turned to its place round the hub
  k.put("hard", mesh.rotate_z(mesh.rotate_x(mesh.merge(one.lists.x), math.pi / 2), 2 * math.pi * i / count))
end
kit.put(m, blades, k)
return m
