-- A FOUNTAIN for a park, a plaza or a mall's court (#191): a round basin with a rim wide enough to sit on, and
-- one or two bowls on a stem in its middle. Nothing on it moves (water that runs is not a model's job); it has
-- the markers water and sound would come from: `jet` at the top, `spill_N` at each bowl's lip, `pool`, and a
-- `seat_N` round the rim.
-- It stands on y = 0 at its own origin.
local kit = require "model_kit"
local V = kit.V

local a = params{
  size  = { 3.2, min = 1.6, max = 8.0 },          -- across the basin (metres)
  bowls = { 2, min = 1, max = 3, step = 1 },
  stone = { "limestone", "granite", "bronze" },
}
local R, N = a.size / 2, math.floor(a.bowls + 0.5)
local col = ({ limestone = V(0.78, 0.75, 0.68), granite = V(0.5, 0.5, 0.52), bronze = V(0.35, 0.3, 0.2) })[a.stone]
local part = a.stone == "bronze" and "metal" or "ceramic"
local m = model.new()
local far = m:level(80)
local RIM, WALL = 0.45, 0.32
local c = kit.maker()
c.turned(part, V(0, 0, 0), { { R, 0 }, { R, RIM - 0.03 }, { R - 0.02, RIM }, { R - WALL + 0.02, RIM }, { R - WALL, RIM - 0.03 }, { R - WALL, 0.12 }, { 0, 0.12 } }, col, 1, 48)
c.turned("glass", V(0, 0, 0), { { R - WALL, RIM - 0.1 }, { 0, RIM - 0.1 } }, V(0.35, 0.55, 0.6), 1, 48)          -- the water
local y = 0.12
for i = 1, N do
  local br = R * (0.5 - (i - 1) * 0.16)           -- each bowl smaller than the one below
  local stem = 0.55 - (i - 1) * 0.08
  c.turned(part, V(0, y, 0), { { br * 0.3, 0 }, { br * 0.16, 0.08 }, { br * 0.12, stem * 0.5 }, { br * 0.2, stem }, { br * 0.6, stem + 0.1 }, { br, stem + 0.22 },
                               { br - 0.03, stem + 0.24 }, { br * 0.55, stem + 0.14 }, { 0, stem + 0.12 } }, col, 1, 36)
  c.turned("glass", V(0, y, 0), { { br * 0.9, stem + 0.2 }, { 0, stem + 0.2 } }, V(0.35, 0.55, 0.6), 1, 36)
  m:marker("spill_" .. i, { pos = { br, y + stem + 0.22, 0 } })
  y = y + stem + 0.12
end
c.turned(part, V(0, y, 0), { { 0.05, 0 }, { 0.035, 0.2 }, { 0.06, 0.26 }, { 0, 0.3 } }, col, 1, 14)
kit.put(m, m, c)
kit.far_box(m, m, part, { 2 * R * 0.9, RIM, 2 * R * 0.9 }, { 0, RIM / 2, 0 }, col, far)
m:shape("basin", { hull = mesh.lathe{ profile = { { 0, 0 }, { R, 0 }, { R, RIM }, { 0, RIM } }, segments = 16 }, friction = 0.8 })
m:shape("stem", { capsule = { radius = 0.25, height = y }, pos = { 0, y / 2 + 0.2, 0 } })
m:marker("jet", { pos = { 0, y + 0.3, 0 }, turn = { -90, 0, 0 } })
m:marker("pool", { pos = { R * 0.6, RIM - 0.1, 0 } })
local seats = math.max(4, math.floor(2 * math.pi * R / 1.2))
for i = 1, seats do
  local t = 2 * math.pi * (i - 1) / seats
  m:marker("seat_" .. i, { pos = { (R - WALL / 2) * math.sin(t), RIM, (R - WALL / 2) * math.cos(t) }, turn = { 0, math.deg(t), 0 }, role = "seat", size = 0.4 })
end
return m
