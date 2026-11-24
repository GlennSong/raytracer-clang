-- SMALL THINGS FOR A HOME'S SHELVES AND TABLES, each with a part that moves (#191):
--
--   record_player   its platter turns (`platter`), its arm swings in over the record (`arm`), its clear cover lifts (`cover`)
--   metronome       its arm ticks from side to side (`arm`)
--   globe           a globe on a tilted axis; it spins (`globe`)
--   laptop          its screen opens (`lid`); as placed it is shut
--
-- Each stands on y = 0 at its own origin (put it on a table), its front toward +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind = { "record_player", "metronome", "globe", "laptop" },
  wood = kit.wood_names,
}
local wood = kit.wood(a.wood)
local m = model.new()
local far = m:level(20)
local c = kit.maker()

if a.kind == "record_player" then
  local W, D, TOP = 0.44, 0.36, 0.09
  c.box("wood", V(0, TOP / 2 + 0.01, 0), V(W, TOP - 0.02, D), 0.012, wood, 2)
  for _, x in ipairs{ -W / 2 + 0.05, W / 2 - 0.05 } do for _, z in ipairs{ -D / 2 + 0.05, D / 2 - 0.05 } do c.turned("rubber", V(x, 0, z), { { 0, 0 }, { 0.02, 0 }, { 0.02, 0.02 }, { 0, 0.02 } }, kit.BLACK, 1, 10) end end
  c.box("metal", V(0, TOP + 0.002, 0), V(W - 0.02, 0.004, D - 0.02), 0.0, V(0.25, 0.25, 0.27), 1)
  c.turned("metal", V(0.17, TOP, -0.12), { { 0, 0 }, { 0.022, 0 }, { 0.022, 0.05 }, { 0, 0.05 } }, kit.CHROME, 1, 12)   -- the arm's pillar
  c.turned("hard", V(0.17, TOP, 0.13), { { 0, 0 }, { 0.016, 0 }, { 0.014, 0.012 }, { 0, 0.012 } }, kit.BLACK, 1, 10)   -- the speed knob
  kit.put(m, m, c)
  kit.far_box(m, m, "wood", { W, TOP, D }, { 0, TOP / 2, 0 }, wood, far)
  m:shape("body", { box = { W, TOP, D }, pos = { 0, TOP / 2, 0 } })
  m:marker("sound", { pos = { 0, TOP, 0 } })
  local platter = m:node("platter", { pos = { -0.05, TOP + 0.004, 0 }, motion = { turn = { 0, -1, 0 } } })
  local p = kit.maker()
  p.turned("metal", V(0, 0, 0), { { 0, 0 }, { 0.15, 0 }, { 0.15, 0.012 }, { 0, 0.012 } }, V(0.7, 0.71, 0.73), 1, 28)
  p.turned("hard", V(0, 0.012, 0), { { 0, 0 }, { 0.148, 0 }, { 0.148, 0.002 }, { 0, 0.002 } }, kit.BLACK, 1, 28)        -- the record
  p.turned("hard", V(0, 0.0142, 0), { { 0, 0 }, { 0.05, 0 }, { 0.05, 0.0005 }, { 0, 0.0005 } }, kit.paint.orange, 1, 16)   -- its label
  p.box("hard", V(0.025, 0.0152, 0), V(0.03, 0.0004, 0.012), 0.0, kit.WHITE, 1)                                          -- the title on it, so the turning shows
  kit.put(m, platter, p)
  local arm = m:node("arm", { pos = { 0.17, TOP + 0.05, -0.12 }, motion = { turn = { 0, -1, 0 }, min = 0, max = 30 } })
  local r = kit.maker()
  r.rod("metal", V(0, 0, -0.05), V(0, 0, 0.21), 0.004, kit.CHROME, 6)
  r.box("hard", V(0, -0.006, 0.22), V(0.018, 0.012, 0.035), 0.003, kit.BLACK, 1)                                        -- the head, with the needle under it
  r.turned("metal", V(0, -0.015, -0.06), { { 0, 0 }, { 0.014, 0 }, { 0.014, 0.03 }, { 0, 0.03 } }, kit.CHROME, 1, 10)  -- the weight behind
  kit.put(m, arm, r)
  arm:marker("lift", { pos = { 0.012, 0, 0.22 }, role = "hold" })
  local cover = m:node("cover", { pos = { 0, TOP + 0.004, -D / 2 + 0.01 }, motion = { turn = { -1, 0, 0 }, min = 0, max = 70 } })
  local k = kit.maker()
  local w, d, h, G = W - 0.02, D - 0.02, 0.085, V(0.85, 0.9, 0.92)
  k.box("glass", V(0, h, d / 2), V(w, 0.003, d), 0.0, G, 1)
  for _, x in ipairs{ -w / 2, w / 2 } do k.box("glass", V(x, h / 2, d / 2), V(0.003, h, d), 0.0, G, 1) end
  for _, z in ipairs{ 0, d } do k.box("glass", V(0, h / 2, z), V(w, h, 0.003), 0.0, G, 1) end
  kit.put(m, cover, k)
  cover:marker("cover", { pos = { 0, 0.02, d }, role = "hold" })
elseif a.kind == "metronome" then
  c.mesh("wood", mesh.rotate_y(mesh.lathe{ profile = { { 0, 0 }, { 0.085, 0 }, { 0.085, 0.02 }, { 0.03, 0.22 }, { 0, 0.225 } }, segments = 4 }, math.pi / 4), wood)
  c.box("hard", V(0, 0.115, 0.036), V(0.03, 0.17, 0.002), 0.0, V(0.9, 0.86, 0.7), 1)                                    -- the scale behind the arm
  kit.put(m, m, c)
  m:shape("body", { box = { 0.12, 0.22, 0.12 }, pos = { 0, 0.11, 0 } })
  m:marker("tick", { pos = { 0, 0.1, 0.05 } })
  local arm = m:node("arm", { pos = { 0, 0.03, 0.052 }, motion = { turn = { 0, 0, 1 }, min = -35, max = 35 } })
  local k = kit.maker()
  k.rod("metal", V(0, 0, 0), V(0, 0.2, 0), 0.0025, kit.CHROME, 6)
  k.box("metal", V(0, 0.14, 0), V(0.022, 0.026, 0.008), 0.003, V(0.8, 0.62, 0.25), 1)                                   -- the weight that sets its speed
  kit.put(m, arm, k)
elseif a.kind == "globe" then
  local R, Y = 0.15, 0.24
  c.turned("wood", V(0, 0, 0), { { 0, 0 }, { 0.09, 0 }, { 0.08, 0.02 }, { 0.02, 0.035 }, { 0.015, 0.07 }, { 0, 0.07 } }, wood, 1, 20)
  -- the half ring that holds it by its poles, leaning 23 degrees
  local arc = {}
  for i = 0, 12 do local t = math.rad(-90 + i * 15 - 23.5); arc[#arc + 1] = { (R + 0.02) * math.cos(t), Y + (R + 0.02) * math.sin(t), 0 } end
  c.pipe("metal", arc, 0.005, V(0.8, 0.62, 0.25), { sides = 6 })
  kit.put(m, m, c)
  m:shape("body", { capsule = { radius = R, height = 0.1 }, pos = { 0, 0.2, 0 } })
  local globe = m:node("globe", { pos = { 0, Y, 0 }, turn = { 0, 0, -23.5 }, motion = { turn = { 0, 1, 0 } } })
  local k = kit.maker()
  k.mesh("paint", kit.ball(R, 24), V(0.15, 0.4, 0.7))
  local land = { { 0.5, 0.6, 0.07, 1.0 }, { 0.1, 1.9, 0.06, 1.4 }, { -0.4, 2.3, 0.05, 1.0 }, { 0.6, 3.6, 0.08, 0.8 }, { -0.2, 4.4, 0.05, 1.6 }, { 0.3, 5.3, 0.06, 1.0 }, { -1.2, 1.0, 0.07, 0.6 } }
  for _, l in ipairs(land) do                                                                                            -- the lands: low domes on the sea
    local lat, lon = l[1], l[2]
    local d = mesh.scale(kit.ball(l[3], 16), { 0.7, 0.2, l[4] * 0.7 })
    d = mesh.rotate_y(mesh.rotate_z(mesh.translate(d, { 0, R - 0.008, 0 }), lat - math.pi / 2), lon)
    k.mesh("paint", d, V(0.3, 0.55, 0.25) * (0.8 + 0.4 * (l[3] * 10 % 1)))
  end
  kit.put(m, globe, k)
  globe:marker("spin", { pos = { R, 0, 0 }, role = "hold" })
else
  local W, D = 0.32, 0.22
  local col = V(0.7, 0.71, 0.73)
  c.box("metal", V(0, 0.008, 0), V(W, 0.016, D), 0.006, col, 1)
  c.box("hard", V(0, 0.0165, -0.025), V(W - 0.04, 0.001, 0.1), 0.0, kit.BLACK * 1.5, 1)                                 -- the keys
  c.box("hard", V(0, 0.0165, 0.065), V(0.1, 0.001, 0.06), 0.0, col * 0.85, 1)                                           -- the pad
  kit.put(m, m, c)
  m:shape("body", { box = { W, 0.016, D }, pos = { 0, 0.008, 0 } })
  m:marker("keys", { pos = { 0, 0.02, 0 }, role = "press" })
  local lid = m:node("lid", { pos = { 0, 0.017, -D / 2 + 0.005 }, motion = { turn = { -1, 0, 0 }, min = 0, max = 130 } })
  local k = kit.maker()
  k.box("metal", V(0, 0.004, D / 2 - 0.005), V(W, 0.007, D), 0.004, col, 1)
  k.box("screen", V(0, -0.0002, D / 2 - 0.005), V(W - 0.02, 0.001, D - 0.025), 0.0, V(0.1, 0.25, 0.5), 1)
  kit.put(m, lid, k)
  lid:marker("screen", { pos = { 0, -0.001, D / 2 }, turn = { 90, 0, 0 }, role = "screen", size = W - 0.02 })
  lid:marker("lid", { pos = { 0, 0.004, D - 0.01 }, role = "hold" })
end
return m
