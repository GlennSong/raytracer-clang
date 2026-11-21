-- GYM EQUIPMENT, each with parts that move (#191):
--
--   exercise_bike    its cranks and pedals go round (`crank`) and its flywheel spins (`flywheel`)
--   rowing_machine   the seat runs back along its rail (`seat`) as the handle is pulled (`handle`); a fan spins (`fan`)
--   weight_machine   pulling the bar down (`bar`) lifts the top of the weight stack (`stack`)
--   punch_bag        a heavy bag hung from a stand; it swings (`bag`)
--   treadmill        its deck tilts up at the front (`deck`), and marks on its belt run back along it
--                    (`mark_1` ..., each sliding one mark's spacing and jumping back: model_moves `loop`)
--
-- Each stands on y = 0 at its own origin; whoever uses it faces +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind  = { "exercise_bike", "rowing_machine", "weight_machine", "punch_bag", "treadmill" },
  paint = { "red", "blue", "yellow", "green", "orange" },
}
local paint = kit.paint[a.paint]
local DARK, GREY = kit.BLACK * 1.5, V(0.62, 0.64, 0.66)
local m = model.new()
local far = m:level(40)
local c = kit.maker()
local function wheel(r, thick, segs)           -- a disc about +x
  return mesh.rotate_z(mesh.lathe{ profile = { { 0, -thick / 2 }, { r, -thick / 2 }, { r, thick / 2 }, { 0, thick / 2 } }, segments = segs or 20 }, math.pi / 2)
end

if a.kind == "exercise_bike" then
  for _, z in ipairs{ -0.45, 0.45 } do c.box("paint", V(0, 0.03, z), V(0.5, 0.05, 0.07), 0.015, DARK, 2) end
  c.pipe("paint", { { 0, 0.05, -0.45 }, { 0, 0.3, -0.1 }, { 0, 0.35, 0.3 }, { 0, 0.05, 0.45 } }, 0.03, paint, { bend = 0.08 })
  c.pipe("paint", { { 0, 0.3, -0.1 }, { 0, 0.92, -0.26 } }, 0.022, paint)                                   -- the seat post
  c.pipe("paint", { { 0, 0.35, 0.3 }, { 0, 1.05, 0.42 } }, 0.022, paint)                                    -- the handlebar post
  c.pipe("metal", { { -0.22, 1.1, 0.3 }, { -0.2, 1.05, 0.42 }, { 0.2, 1.05, 0.42 }, { 0.22, 1.1, 0.3 } }, 0.013, DARK, { bend = 0.04, sides = 8 })
  c.cushion("fabric", V(0, 0.95, -0.27), V(0.2, 0.06, 0.26), kit.BLACK * 2, { r = 0.07 })
  c.box("hard", V(0, 1.12, 0.44), V(0.12, 0.09, 0.02), 0.006, kit.BLACK, 1)
  c.box("screen", V(0, 1.12, 0.429), V(0.1, 0.07, 0.001), 0.0, V(0.1, 0.4, 0.3), 1)
  c.mesh("paint", mesh.translate(wheel(0.25, 0.08), { 0, 0.35, 0.3 }), DARK)                                 -- the flywheel's guard
  kit.put(m, m, c)
  kit.far_box(m, m, "paint", { 0.3, 1.0, 1.0 }, { 0, 0.5, 0 }, paint, far)
  m:shape("body", { box = { 0.3, 0.9, 1.0 }, pos = { 0, 0.45, 0 } })
  m:marker("seat", { pos = { 0, 0.98, -0.27 }, role = "seat", size = 0.25 })
  m:marker("hold", { pos = { 0, 1.05, 0.42 }, role = "hold" })
  local fly = m:node("flywheel", { pos = { 0, 0.35, 0.3 }, motion = { turn = { 1, 0, 0 } } })
  local f = kit.maker()
  f.mesh("metal", wheel(0.22, 0.09), GREY)
  for i = 0, 2 do f.mesh("hard", mesh.rotate_x(mesh.translate(mesh.box{ 0.092, 0.2, 0.03 }, { 0, 0.11, 0 }), i * 2 * math.pi / 3), paint) end   -- marks, so the turning shows
  kit.put(m, fly, f)
  local crank = m:node("crank", { pos = { 0, 0.3, -0.1 }, motion = { turn = { 1, 0, 0 } } })
  local k = kit.maker()
  k.rod("metal", V(-0.09, 0, 0), V(0.09, 0, 0), 0.012, GREY, 8)
  for _, side in ipairs{ -1, 1 } do
    k.box("metal", V(side * 0.09, side * 0.085, 0), V(0.012, 0.19, 0.025), 0.004, GREY, 1)
    k.box("hard", V(side * 0.14, side * 0.17, 0), V(0.09, 0.02, 0.07), 0.006, kit.BLACK, 1)                  -- the pedal
  end
  kit.put(m, crank, k)
elseif a.kind == "rowing_machine" then
  c.box("metal", V(0, 0.28, -0.25), V(0.08, 0.05, 1.7), 0.01, GREY, 1)                                       -- the rail
  c.box("paint", V(0, 0.14, -1.05), V(0.4, 0.28, 0.05), 0.012, DARK, 1)
  c.box("paint", V(0, 0.03, 0.75), V(0.5, 0.05, 0.07), 0.015, DARK, 2)
  c.mesh("paint", mesh.translate(wheel(0.28, 0.14, 24), { 0, 0.42, 0.75 }), paint)                           -- the fan's housing
  c.box("paint", V(0, 0.2, 0.75), V(0.1, 0.3, 0.1), 0.01, DARK, 1)
  for _, x in ipairs{ -0.14, 0.14 } do c.tilted("hard", V(x, 0.3, 0.42), V(0.11, 0.02, 0.3), 0.006, -0.8, kit.BLACK) end   -- the foot plates
  c.box("hard", V(0, 0.82, 0.68), V(0.14, 0.1, 0.02), 0.006, kit.BLACK, 1)
  c.rod("paint", V(0, 0.6, 0.72), V(0, 0.8, 0.69), 0.012, DARK, 6)
  kit.put(m, m, c)
  kit.far_box(m, m, "paint", { 0.4, 0.5, 2.0 }, { 0, 0.25, -0.15 }, DARK, far)
  m:shape("rail", { box = { 0.3, 0.3, 1.8 }, pos = { 0, 0.15, -0.2 } })
  m:shape("fan", { box = { 0.2, 0.7, 0.56 }, pos = { 0, 0.35, 0.75 } })
  local fan = m:node("fan", { pos = { 0.071, 0.42, 0.75 }, motion = { turn = { 1, 0, 0 } } })
  local f = kit.maker()
  for i = 0, 5 do f.mesh("hard", mesh.rotate_x(mesh.translate(mesh.box{ 0.002, 0.22, 0.03 }, { 0, 0.12, 0 }), i * math.pi / 3), kit.BLACK) end
  kit.put(m, fan, f)
  local seat = m:node("seat", { pos = { 0, 0.305, 0.05 }, motion = { slide = { 0, 0, -1 }, min = 0, max = 0.8 } })
  local s = kit.maker()
  s.cushion("fabric", V(0, 0.045, 0), V(0.3, 0.05, 0.26), kit.BLACK * 2, { r = 0.06 })
  s.box("metal", V(0, 0.01, 0), V(0.14, 0.02, 0.2), 0.004, GREY, 1)
  kit.put(m, seat, s)
  seat:marker("seat", { pos = { 0, 0.07, 0 }, role = "seat", size = 0.3 })
  local handle = m:node("handle", { pos = { 0, 0.55, 0.42 }, motion = { slide = { 0, 0, -1 }, min = 0, max = 0.75 } })
  local h = kit.maker()
  h.rod("hard", V(-0.22, 0, 0), V(0.22, 0, 0), 0.015, kit.BLACK, 8)
  h.rod("hard", V(0, 0, 0), V(0, -0.1, 1.0), 0.004, kit.BLACK, 4)                                             -- the strap, back to the fan
  kit.put(m, handle, h)
  handle:marker("hold", { pos = { 0, 0, 0 }, role = "hold" })
elseif a.kind == "weight_machine" then
  local H = 2.1
  for _, x in ipairs{ -0.32, 0.32 } do c.box("paint", V(x, H / 2, -0.4), V(0.06, H, 0.06), 0.008, paint, 1) end
  c.box("paint", V(0, H - 0.03, -0.1), V(0.7, 0.06, 0.7), 0.01, paint, 1)
  c.box("paint", V(0, 0.03, 0), V(0.7, 0.06, 1.3), 0.01, paint, 1)
  for _, x in ipairs{ -0.1, 0.1 } do c.rod("metal", V(x, 0.06, -0.4), V(x, H - 0.06, -0.4), 0.012, kit.CHROME, 8) end   -- the stack's guide rods
  for i = 0, 7 do c.box("hard", V(0, 0.1 + i * 0.05, -0.4), V(0.36, 0.045, 0.12), 0.006, kit.BLACK * 1.3, 1) end
  c.box("paint", V(0, 0.25, 0.25), V(0.08, 0.4, 0.08), 0.008, paint, 1)                                      -- the seat's post
  c.cushion("fabric", V(0, 0.48, 0.25), V(0.34, 0.07, 0.34), kit.BLACK * 2, { r = 0.05 })
  c.box("paint", V(0, 0.75, 0.48), V(0.3, 0.06, 0.06), 0.02, DARK, 2)                                        -- the pad over the knees
  c.rod("paint", V(0, 0.45, 0.48), V(0, 0.75, 0.48), 0.02, paint, 8)
  kit.put(m, m, c)
  kit.far_box(m, m, "paint", { 0.7, H, 0.2 }, { 0, H / 2, -0.4 }, paint, far)
  m:shape("frame", { box = { 0.7, H, 0.2 }, pos = { 0, H / 2, -0.4 } })
  m:shape("seat", { box = { 0.34, 0.5, 0.34 }, pos = { 0, 0.25, 0.25 } })
  m:marker("seat", { pos = { 0, 0.52, 0.25 }, role = "seat", size = 0.34 })
  local stack = m:node("stack", { pos = { 0, 0.5, -0.4 }, motion = { slide = { 0, 1, 0 }, min = 0, max = 0.7 } })
  local s = kit.maker()
  for i = 0, 3 do s.box("hard", V(0, i * 0.05, 0), V(0.36, 0.045, 0.12), 0.006, kit.BLACK * 1.3, 1) end
  s.box("hard", V(0.1, 0.05, 0.062), V(0.04, 0.02, 0.004), 0.0, kit.paint.yellow, 1)                          -- the pin
  s.rod("hard", V(0, 0.2, 0), V(0, 1.5, 0), 0.004, kit.BLACK, 4)                                              -- the cable up to the pulleys
  kit.put(m, stack, s)
  local bar = m:node("bar", { pos = { 0, 1.75, 0.2 }, motion = { slide = { 0, -1, 0 }, min = 0, max = 0.7 } })
  local b = kit.maker()
  b.pipe("metal", { { -0.55, -0.08, 0 }, { -0.4, 0, 0 }, { 0.4, 0, 0 }, { 0.55, -0.08, 0 } }, 0.014, kit.CHROME, { bend = 0.05, sides = 8 })
  b.rod("hard", V(0, 0, 0), V(0, 1.0, 0), 0.004, kit.BLACK, 4)
  kit.put(m, bar, b)
  bar:marker("hold", { pos = { 0, 0, 0 }, role = "hold" })
elseif a.kind == "punch_bag" then
  c.turned("paint", V(0, 0, -0.5), { { 0, 0 }, { 0.38, 0 }, { 0.38, 0.05 }, { 0.1, 0.1 }, { 0, 0.1 } }, DARK, 1, 20)
  c.pipe("paint", { { 0, 0.05, -0.5 }, { 0, 2.2, -0.5 }, { 0, 2.2, 0.05 } }, 0.035, DARK, { bend = 0.2 })
  kit.put(m, m, c)
  m:shape("post", { capsule = { radius = 0.04, height = 2.1 }, pos = { 0, 1.1, -0.5 } })
  local bag = m:node("bag", { pos = { 0, 2.16, 0 }, motion = { turn = { 1, 0, 0 }, min = -25, max = 25 } })
  local k = kit.maker()
  for i = 0, 3 do
    local t = i * math.pi / 2 + math.pi / 4
    k.rod("metal", V(0, 0, 0), V(0.12 * math.cos(t), -0.3, 0.12 * math.sin(t)), 0.004, GREY, 4)
  end
  k.turned("fabric", V(0, -1.4, 0), { { 0, 0 }, { 0.15, 0 }, { 0.18, 0.04 }, { 0.18, 1.06 }, { 0.15, 1.1 }, { 0, 1.1 } }, paint, 1, 20)
  kit.put(m, bag, k)
  kit.far_box(m, bag, "fabric", { 0.3, 1.1, 0.3 }, { 0, -0.85, 0 }, paint, far)
  bag:shape("bag", { capsule = { radius = 0.18, height = 0.74 }, pos = { 0, -0.85, 0 } })
  bag:marker("hit", { pos = { 0, -0.8, 0.18 } })
else
  local LEN, WIDE, N = 1.5, 0.5, 6
  c.box("paint", V(0, 0.04, -0.72), V(WIDE + 0.24, 0.08, 0.1), 0.02, DARK, 2)
  for _, x in ipairs{ -WIDE / 2 - 0.08, WIDE / 2 + 0.08 } do
    c.pipe("paint", { { x, 0.1, 0.55 }, { x, 1.15, 0.75 }, { x, 1.1, 0.3 } }, 0.025, paint, { bend = 0.08 })  -- an upright and its hand rail
    c.box("paint", V(x, 0.04, 0.6), V(0.08, 0.08, 0.3), 0.02, DARK, 2)
  end
  c.box("hard", V(0, 1.22, 0.76), V(WIDE + 0.2, 0.2, 0.04), 0.012, kit.BLACK, 1)
  c.box("screen", V(0, 1.22, 0.739), V(WIDE, 0.14, 0.001), 0.0, V(0.1, 0.35, 0.5), 1)
  kit.put(m, m, c)
  kit.far_box(m, m, "paint", { WIDE + 0.2, 0.2, LEN + 0.2 }, { 0, 0.1, 0 }, DARK, far)
  m:marker("stand", { pos = { 0, 0.2, 0 }, role = "stand" })
  local deck = m:node("deck", { pos = { 0, 0.08, -0.75 }, motion = { turn = { -1, 0, 0 }, min = 0, max = 8 } })
  local d = kit.maker()
  d.box("paint", V(0, 0.05, LEN / 2), V(WIDE + 0.16, 0.1, LEN), 0.012, DARK, 1)
  d.box("rubber", V(0, 0.102, LEN / 2), V(WIDE, 0.004, LEN - 0.06), 0.0, kit.BLACK, 1)                        -- the belt
  kit.put(m, deck, d)
  deck:shape("deck", { box = { WIDE + 0.16, 0.1, LEN }, pos = { 0, 0.05, LEN / 2 }, friction = 0.9 })
  local each = (LEN - 0.06) / N
  for i = 1, N do
    local mark = deck:node("mark_" .. i, { pos = { 0, 0.105, LEN - 0.03 - (i - 1) * each }, motion = { slide = { 0, 0, -1 }, min = 0, max = each } })
    local k = kit.maker()
    k.box("rubber", V(0, 0, -0.01), V(WIDE - 0.04, 0.001, 0.012), 0.0, kit.BLACK * 3.5, 1)
    kit.put(m, mark, k)
  end
  deck:marker("belt", { pos = { 0, 0.105, each }, turn = { 0, 0, 0 }, size = each })
end
return m
