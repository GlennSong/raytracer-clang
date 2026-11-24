-- A RACK OR STAND THAT TURNS, in a shop (#191): the whole display is on a node, `rack`, that turns without end.
--
--   postcards   a tall wire spinner with tiers of cards, as outside a bookshop
--   clothes     a round rail of garments on hangers, as in a boutique or a department store
--   turntable   a low round plinth that turns what stands on it: a marker `on_top` (a mannequin, a car, a prize)
--
-- It stands on y = 0 at its own origin.
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind = { "postcards", "clothes", "turntable" },
  size = { 0.0, min = 0, max = 4 },               -- across (metres); 0: the usual size for its kind
}
local m = model.new()
local far = m:level(40)
local c = kit.maker()
local STEEL = V(0.75, 0.76, 0.78)
local colours = { V(0.8, 0.2, 0.15), V(0.15, 0.35, 0.7), V(0.9, 0.75, 0.2), V(0.2, 0.55, 0.3), V(0.9, 0.9, 0.85), V(0.15, 0.15, 0.18), V(0.85, 0.5, 0.6), V(0.5, 0.35, 0.2) }

if a.kind == "postcards" then
  local R = (a.size > 0 and a.size or 0.44) / 2
  c.turned("metal", V(0, 0, 0), { { 0, 0 }, { 0.22, 0 }, { 0.22, 0.02 }, { 0.03, 0.04 }, { 0.02, 0.06 }, { 0.02, 0.5 }, { 0, 0.5 } }, STEEL, 1, 18)
  kit.put(m, m, c)
  m:shape("foot", { box = { 0.44, 0.04, 0.44 }, pos = { 0, 0.02, 0 } })
  local rack = m:node("rack", { pos = { 0, 0.5, 0 }, motion = { turn = { 0, 1, 0 } } })
  local k = kit.maker()
  k.rod("metal", V(0, 0, 0), V(0, 1.25, 0), 0.012, STEEL, 8)
  k.mesh("metal", mesh.translate(mesh.sphere(0.03), { 0, 1.27, 0 }), STEEL)
  for tier = 0, 5 do
    local y = 0.08 + tier * 0.19
    for side = 0, 3 do
      local t = side * math.pi / 2 + (tier % 2) * math.pi / 4
      local one = kit.maker()
      one.box("x", V(0, y + 0.075, R - 0.02), V(0.11, 0.15, 0.004), 0.0, colours[(tier * 3 + side) % #colours + 1], 1)   -- a card, face out
      one.rod("y", V(0, y, 0), V(0, y, R), 0.003, STEEL, 4)
      one.rod("y", V(-0.06, y, R), V(0.06, y, R), 0.003, STEEL, 4)
      k.put("hard", mesh.rotate_y(mesh.merge(one.lists.x), t))
      k.put("metal", mesh.rotate_y(mesh.merge(one.lists.y), t))
    end
  end
  kit.put(m, rack, k)
  kit.far_box(m, rack, "hard", { 2 * R, 1.2, 2 * R }, { 0, 0.65, 0 }, colours[5], far)
  rack:shape("rack", { capsule = { radius = R, height = 0.9 }, pos = { 0, 0.65, 0 } })
elseif a.kind == "clothes" then
  local R = (a.size > 0 and a.size or 1.0) / 2
  for i = 0, 3 do
    local t = i * math.pi / 2
    c.rod("metal", V(0, 0.05, 0), V(0.4 * math.cos(t), 0.05, 0.4 * math.sin(t)), 0.014, STEEL, 8)
    kit.caster(c, V(0.4 * math.cos(t), -0.01, 0.4 * math.sin(t)), 0.022)
  end
  c.rod("metal", V(0, 0.05, 0), V(0, 1.0, 0), 0.018, STEEL, 10)
  kit.put(m, m, c)
  m:shape("post", { capsule = { radius = 0.03, height = 1.0 }, pos = { 0, 0.5, 0 } })
  local rack = m:node("rack", { pos = { 0, 1.0, 0 }, motion = { turn = { 0, 1, 0 } } })
  local k = kit.maker()
  k.rod("metal", V(0, 0, 0), V(0, 0.5, 0), 0.014, STEEL, 8)
  local ring = {}
  for i = 0, 31 do local t = 2 * math.pi * i / 32; ring[#ring + 1] = { R * math.cos(t), 0.45, R * math.sin(t) } end
  k.pipe("metal", ring, 0.012, STEEL, { closed = true, sides = 8 })
  for i = 0, 3 do local t = i * math.pi / 2 + 0.4; k.rod("metal", V(0, 0.45, 0), V(R * math.cos(t), 0.45, R * math.sin(t)), 0.008, STEEL, 6) end
  local count = math.floor(2 * math.pi * R / 0.085)
  for i = 0, count - 1 do                        -- the garments, each hanging across the rail
    local t = 2 * math.pi * i / count
    local one = mesh.translate(mesh.rounded_box({ 0.42, 0.62 + 0.2 * ((i * 7) % 3) / 2, 0.035 }, 0.015), { 0, 0.45 - 0.07 - (0.62 + 0.2 * ((i * 7) % 3) / 2) / 2, 0 })
    k.mesh("fabric", mesh.translate(mesh.rotate_y(one, -t), { R * math.cos(t), 0, R * math.sin(t) }), colours[(i * 5) % #colours + 1])
  end
  kit.put(m, rack, k)
  kit.far_box(m, rack, "fabric", { 2 * R, 0.8, 2 * R }, { 0, 0.05, 0 }, colours[2], far)
  rack:shape("rack", { capsule = { radius = R * 0.9, height = 0.4 }, pos = { 0, 0.0, 0 } })
else
  local R = (a.size > 0 and a.size or 1.2) / 2
  c.turned("paint", V(0, 0, 0), { { 0, 0 }, { R + 0.04, 0 }, { R + 0.04, 0.1 }, { 0, 0.1 } }, kit.BLACK * 1.6, 1, 40)
  kit.put(m, m, c)
  m:shape("base", { hull = mesh.lathe{ profile = { { 0, 0 }, { R + 0.04, 0 }, { R + 0.04, 0.1 }, { 0, 0.1 } }, segments = 12 } })
  local rack = m:node("rack", { pos = { 0, 0.1, 0 }, motion = { turn = { 0, 1, 0 } } })
  local k = kit.maker()
  k.turned("paint", V(0, 0, 0), { { 0, 0 }, { R, 0 }, { R, 0.03 }, { R - 0.01, 0.04 }, { 0, 0.04 } }, kit.WHITE, 1, 40)
  k.box("light", V(R * 0.7, 0.041, 0), V(0.12, 0.002, 0.04), 0.0, V(1, 0.9, 0.6), 1)   -- a mark, so the turning shows
  kit.put(m, rack, k)
  rack:shape("top", { hull = mesh.lathe{ profile = { { 0, 0 }, { R, 0 }, { R, 0.04 }, { 0, 0.04 } }, segments = 12 }, friction = 1.0 })
  rack:marker("on_top", { pos = { 0, 0.04, 0 }, role = "stand" })
end
m:marker("front", { pos = { 0, 0, 1.2 }, turn = { 0, 180, 0 }, role = "stand" })
return m
