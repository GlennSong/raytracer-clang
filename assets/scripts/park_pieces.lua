-- PARK FURNITURE THAT STANDS STILL: a picnic table, a bench, a drinking fountain, a bike rack (#191). Nothing on
-- these moves; they are here so a park's whole set comes by name, with markers for where people sit, stand,
-- drink and lock a bike. Each stands on y = 0 at its own origin, its front toward +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind   = { "picnic_table", "bench", "drinking_fountain", "bike_rack" },
  length = { 1.8, min = 1.0, max = 3.0 },         -- picnic table, bench, bike rack (metres)
  wood   = kit.wood_names,                        -- picnic table, bench
  paint  = { "green", "black", "blue", "red" },   -- the metal parts
}
local L, wood = a.length, kit.wood(a.wood)
local metal = a.paint == "black" and kit.BLACK * 1.8 or kit.paint[a.paint]
local m = model.new()
local far = m:level(50)
local k = kit.maker()

if a.kind == "picnic_table" then
  for i = -2, 2 do k.box("wood", V(0, 0.74, i * 0.15), V(L, 0.04, 0.14), 0.008, wood, 1) end          -- the top
  for _, s in ipairs{ -1, 1 } do
    for _, dz in ipairs{ -0.08, 0.08 } do k.box("wood", V(0, 0.44, s * 0.68 + dz), V(L, 0.04, 0.14), 0.008, wood, 1) end   -- a seat
    for i, x in ipairs{ -L / 2 + 0.22, L / 2 - 0.22 } do
      k.post("wood", V(x, 0, s * 0.72), V(x, 0.72, s * 0.2), 0.045, 0.045, wood * 0.9)                  -- the A's leg
      m:marker(string.format("seat_%d", (s > 0 and 0 or 2) + i), { pos = { x * 0.6, 0.46, s * 0.68 }, turn = { 0, s > 0 and 180 or 0, 0 }, role = "seat", size = 0.4 })
    end
    m:shape("seat_" .. (s > 0 and "front" or "back"), { box = { L, 0.04, 0.3 }, pos = { 0, 0.44, s * 0.68 }, friction = 0.9 })
  end
  for _, x in ipairs{ -L / 2 + 0.22, L / 2 - 0.22 } do
    k.box("wood", V(x, 0.42, 0), V(0.045, 0.09, 1.56), 0.006, wood * 0.9, 1)                            -- the rail the seats sit on
    k.box("wood", V(x, 0.70, 0), V(0.045, 0.07, 0.72), 0.006, wood * 0.9, 1)
  end
  m:shape("top", { box = { L, 0.04, 0.74 }, pos = { 0, 0.74, 0 }, friction = 0.9 })
  kit.far_box(m, m, "wood", { L, 0.04, 0.74 }, { 0, 0.74, 0 }, wood, far)
  kit.far_box(m, m, "wood", { L, 0.04, 1.6 }, { 0, 0.44, 0 }, wood, far)
elseif a.kind == "bench" then
  for i = 0, 3 do k.box("wood", V(0, 0.44, -0.16 + i * 0.11), V(L, 0.035, 0.09), 0.008, wood, 1) end   -- the seat's slats
  for i = 0, 2 do k.tilted("wood", V(0, 0.58 + i * 0.12, -0.25 - i * 0.03), V(L, 0.09, 0.03), 0.008, -0.25, wood) end   -- the back's
  for _, x in ipairs{ -L / 2 + 0.12, L / 2 - 0.12 } do
    k.pipe("paint", { { x, 0, 0.2 }, { x, 0.42, 0.2 }, { x, 0.42, -0.22 }, { x, 0.9, -0.34 } }, 0.022, metal, { bend = 0.06 })
    k.pipe("paint", { { x, 0, -0.26 }, { x, 0.42, -0.22 } }, 0.022, metal)
    k.pipe("paint", { { x, 0.42, 0.2 }, { x, 0.64, 0.2 }, { x, 0.64, -0.27 } }, 0.018, metal, { bend = 0.06 })   -- the arm
  end
  local seats = math.max(2, math.floor(L / 0.6))
  for i = 1, seats do m:marker("seat_" .. i, { pos = { -L / 2 + (i - 0.5) * L / seats, 0.46, 0 }, role = "seat", size = 0.45 }) end
  m:shape("seat", { box = { L, 0.44, 0.45 }, pos = { 0, 0.22, 0 }, friction = 0.9 })
  m:shape("back", { box = { L, 0.45, 0.05 }, pos = { 0, 0.68, -0.28 }, turn = { -14, 0, 0 } })
  kit.far_box(m, m, "wood", { L, 0.04, 0.42 }, { 0, 0.44, 0 }, wood, far)
  kit.far_box(m, m, "wood", { L, 0.36, 0.04 }, { 0, 0.7, -0.28 }, wood, far)
elseif a.kind == "drinking_fountain" then
  local STONE = V(0.6, 0.6, 0.58)
  k.turned("ceramic", V(0, 0, 0), { { 0.0, 0.0 }, { 0.2, 0.0 }, { 0.2, 0.04 }, { 0.12, 0.08 }, { 0.1, 0.8 }, { 0.2, 0.9 }, { 0.22, 0.95 } }, STONE, 1, 20)
  k.turned("metal", V(0, 0, 0), { { 0.215, 0.95 }, { 0.19, 0.9 }, { 0.03, 0.87 }, { 0.0, 0.87 } }, V(0.75, 0.76, 0.78), 1, 20)       -- the bowl
  k.pipe("metal", { { 0.12, 0.9, 0 }, { 0.12, 1.02, 0 }, { 0.06, 1.02, 0 } }, 0.01, kit.CHROME, { bend = 0.03, sides = 8 })          -- the spout
  k.turned("metal", V(-0.21, 0.86, 0), { { 0.0, 0.0 }, { 0.022, 0.0 }, { 0.022, 0.012 }, { 0.0, 0.014 } }, kit.CHROME, 1, 12)         -- the button, on the side
  m:shape("pedestal", { capsule = { radius = 0.14, height = 0.7 }, pos = { 0, 0.48, 0 } })
  m:marker("button", { pos = { -0.22, 0.87, 0 }, role = "press" })
  m:marker("water", { pos = { 0.06, 1.02, 0 }, turn = { 0, 0, 100 } })
  m:marker("drink", { pos = { -0.1, 0, 0.5 }, turn = { 0, 180, 0 }, role = "stand" })
  kit.far_box(m, m, "ceramic", { 0.3, 0.95, 0.3 }, { 0, 0.475, 0 }, STONE, far)
else
  local hoops = math.max(2, math.floor(L / 0.6))
  for i = 1, hoops do
    local x = -L / 2 + (i - 0.5) * L / hoops
    k.pipe("paint", { { x, -0.02, -0.35 }, { x, 0.8, -0.35 }, { x, 0.8, 0.35 }, { x, -0.02, 0.35 } }, 0.024, metal, { bend = 0.22 })
    m:shape("hoop_" .. i, { box = { 0.05, 0.8, 0.72 }, pos = { x, 0.4, 0 } })
    for _, s in ipairs{ -1, 1 } do m:marker(string.format("bike_%d", 2 * i - (s < 0 and 1 or 0)), { pos = { x + s * 0.15, 0, 0 }, role = "park" }) end
  end
  for _, z in ipairs{ -0.35, 0.35 } do k.box("paint", V(0, 0.006, z), V(L, 0.012, 0.08), 0.0, metal * 0.8, 1) end   -- the flat bars they are welded to
  kit.far_box(m, m, "paint", { L, 0.8, 0.05 }, { 0, 0.4, 0 }, metal, far)
end
kit.put(m, m, k)
return m
