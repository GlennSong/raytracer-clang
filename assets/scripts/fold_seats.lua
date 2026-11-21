-- A ROW OF SEATS THAT FOLD UP: a lecture hall's or a cinema's (#191; the furniture kit's `lecture_row` and
-- `cinema_row` are the still ones). Each seat's pan is on a node, `seat_1` ..., hinged at its back; as placed it
-- is up, out of the way, and it turns down 80 degrees to be sat on. A lecture seat also has a writing tablet on
-- a node, `tablet_1` ..., that swings up from beside the seat. Each seat has a marker, `seat_N`.
-- It stands on y = 0 at its own origin, facing +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind   = { "lecture", "cinema" },
  seats  = { 5, min = 1, max = 12, step = 1 },
  fabric = kit.fabric_names,
  wood   = kit.wood_names,
}
local N, fabric, wood = math.floor(a.seats + 0.5), kit.fabric(a.fabric), kit.wood(a.wood)
local cinema = a.kind == "cinema"
local m = model.new()
local far = m:level(40)
local PITCH = cinema and 0.6 or 0.55            -- from one seat to the next
local W = N * PITCH
local c = kit.maker()
for i = 0, N do                                  -- the standards between the seats, each with an arm rest
  local x = -W / 2 + i * PITCH
  c.box("paint", V(x, 0.3, -0.02), V(0.04, 0.6, 0.32), 0.008, kit.BLACK * 1.6, 1)
  c.box("paint", V(x, 0.012, 0.0), V(0.07, 0.024, 0.5), 0.006, kit.BLACK * 1.6, 1)
  if cinema then c.cushion("hard", V(x, 0.62, 0.05), V(0.07, 0.035, 0.34), kit.BLACK * 1.4, { r = 0.02 })
  else c.box("wood", V(x, 0.61, 0.03), V(0.05, 0.02, 0.26), 0.006, wood, 1) end
  m:shape("standard_" .. i, { box = { 0.05, 0.62, 0.34 }, pos = { x, 0.31, 0 } })
end
for i = 1, N do                                  -- the backs
  local x = -W / 2 + (i - 0.5) * PITCH
  if cinema then c.cushion("fabric", V(x, 0.72, -0.2), V(PITCH - 0.07, 0.62, 0.1), fabric, { face = "z", r = 0.06, crown = 0.02, tilt = -0.18 })
  else c.tilted("wood", V(x, 0.66, -0.2), V(PITCH - 0.06, 0.44, 0.018), 0.01, -0.2, wood) end
end
kit.put(m, m, c)
kit.far_box(m, m, cinema and "fabric" or "wood", { W, 0.9, 0.12 }, { 0, 0.55, -0.2 }, cinema and fabric or wood, far)
m:shape("backs", { box = { W, 0.95, 0.1 }, pos = { 0, 0.5, -0.2 }, turn = { -11, 0, 0 } })

for i = 1, N do
  local x = -W / 2 + (i - 0.5) * PITCH
  -- the pan stands up against the back; turning it about its hinge brings it down level
  local seat = m:node("seat_" .. i, { pos = { x, 0.42, -0.14 }, motion = { turn = { 1, 0, 0 }, min = 0, max = 80 } })
  local k = kit.maker()
  if cinema then k.cushion("fabric", V(0, 0.22, 0.03), V(PITCH - 0.08, 0.44, 0.09), fabric, { face = "z", r = 0.05, crown = 0.02 })
  else k.box("wood", V(0, 0.2, 0.012), V(PITCH - 0.07, 0.4, 0.022), 0.01, wood, 1) end
  kit.put(m, seat, k)
  seat:shape("seat_" .. i, { box = { PITCH - 0.08, 0.42, 0.06 }, pos = { 0, 0.21, 0.02 } })
  seat:marker("seat_" .. i, { pos = { 0, 0.2, 0.06 }, turn = { 90, 0, 0 }, role = "seat", size = 0.42 })
  if not cinema then
    -- the tablet hangs beside the right arm and swings up and over the lap
    local tablet = m:node("tablet_" .. i, { pos = { x + PITCH / 2 - 0.03, 0.62, 0.1 }, motion = { turn = { 0, 0, 1 }, min = 0, max = 90 } })
    local t = kit.maker()
    t.box("wood", V(0.0, -0.13, 0.05), V(0.012, 0.26, 0.3), 0.004, wood * 1.05, 1)
    kit.put(m, tablet, t)
    tablet:marker("tablet_" .. i, { pos = { 0.01, -0.13, 0.05 } })
  end
end
return m
