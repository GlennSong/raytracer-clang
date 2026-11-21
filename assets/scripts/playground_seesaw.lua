-- A PLAYGROUND SEESAW: a plank on a pivot, a seat and a handle at each end, and half a tyre in the ground under
-- each end for it to land on (#191; one of the playground pieces, playground_kit.lua). The plank is on a node,
-- `beam`, that tips about the pivot and stops where an end meets its tyre; nothing tips it here. A prefab with
-- the script playground_seesaw_rock.lua does (prefabs.lua `playground_seesaw_in_use`).
--
-- It stands on y = 0 at its own origin, the plank along x.
local kit = require "playground_kit"

local a = params{
  length = { 3.2, min = 2.4, max = 4.2 },         -- the plank's length (metres)
  paint  = { "blue", "red", "yellow", "green", "orange" },   -- the plank's colour
  seats  = { "yellow", "red", "blue", "green", "orange" },   -- the seats' colour
}
local half, color, seat_color = a.length / 2, kit.paint[a.paint], kit.paint[a.seats]

local m = model.new()
kit.materials(m)
local far = m:level(40)

local PIVOT = 0.62                         -- the axle's height
local TYRE, TYRE_R = 0.2, 0.07             -- a bumper tyre: its ring's radius and its rubber's
local END = half - 0.15                    -- where along the plank each tyre stands
-- how far it tips before the plank's underside meets a tyre's top
local tilt = math.deg(math.asin((PIVOT + 0.02 - (TYRE + TYRE_R)) / END))

-- THE STAND: an upturned V each side of the plank, and the axle through their tops
local stand = {}
for _, z in ipairs{ -0.17, 0.17 } do
  stand[#stand + 1] = kit.pipe({ { -0.34, -0.02, z }, { 0, PIVOT + 0.03, z }, { 0.34, -0.02, z } }, 0.03, kit.zinc, { bend = 0.07 })
  for _, x in ipairs{ -0.34, 0.34 } do stand[#stand + 1] = kit.foot({ x, 0, z }, 0.07, kit.zinc) end
end
stand[#stand + 1] = kit.pipe({ { 0, PIVOT, -0.21 }, { 0, PIVOT, 0.21 } }, 0.022, kit.zinc)
m:add(mesh.merge(stand), "steel")
m:add(kit.box({ 0.5, PIVOT, 0.38 }, { 0, PIVOT / 2, 0 }, kit.zinc), "steel", { level = far })
m:shape("stand", { box = { 0.5, PIVOT - 0.06, 0.4 }, pos = { 0, (PIVOT - 0.06) / 2, 0 } })

-- THE TYRES, half in the ground
local tyres = {}
for i, x in ipairs{ -END, END } do
  tyres[#tyres + 1] = mesh.translate(mesh.rotate_x(mesh.torus(TYRE, TYRE_R), math.pi / 2), { x, 0, 0 })
  m:shape("tyre_" .. i, { box = { 2 * (TYRE + TYRE_R) * 0.8, TYRE + TYRE_R, 2 * TYRE_R }, pos = { x, (TYRE + TYRE_R) / 2, 0 }, friction = 1.0 })
end
m:add(mesh.paint(mesh.merge(tyres), kit.black), "rubber")

-- THE PLANK, in its own space: the axle is its origin and it lies along x
local beam = m:node("beam", { pos = { 0, PIVOT, 0 }, motion = { turn = { 0, 0, 1 }, min = -tilt, max = tilt } })
local PLANK = { a.length, 0.06, 0.2 }
beam:add(kit.pad(PLANK, { 0, 0.05, 0 }, 0.015, color), "paint")
beam:add(kit.box(PLANK, { 0, 0.05, 0 }, color), "paint", { level = far })
beam:shape("plank", { box = PLANK, pos = { 0, 0.05, 0 }, friction = 0.9 })
local seats, handles = {}, {}
for i, side in ipairs{ -1, 1 } do
  local sx = side * (half - 0.2)             -- the seat
  seats[#seats + 1] = kit.pad({ 0.36, 0.045, 0.28 }, { sx, 0.1, 0 }, 0.018, seat_color)
  local hx = side * (half - 0.52)            -- the handle: a hoop to hold, across the plank
  handles[#handles + 1] = kit.pipe({ { hx, 0.06, -0.12 }, { hx, 0.4, -0.12 }, { hx, 0.4, 0.12 }, { hx, 0.06, 0.12 } }, 0.015, kit.zinc, { bend = 0.05 })
  beam:shape("handle_" .. i, { box = { 0.04, 0.34, 0.28 }, pos = { hx, 0.25, 0 } })
  -- each rider faces the middle
  beam:marker("seat_" .. i, { pos = { sx, 0.125, 0 }, turn = { 0, -side * 90, 0 }, role = "seat", size = 0.36 })
  beam:marker("hold_" .. i, { pos = { hx, 0.4, 0 }, role = "hold" })
end
beam:add(mesh.merge(seats), "plastic")
beam:add(mesh.merge(handles), "steel")

m:marker("creak", { pos = { 0, PIVOT, 0 } })
return m
