-- street_props.lua -- the KERB PROPS that are data (ADR-0354; Glenn, 2026-11-06: "if it was a fire hydrant, you might
-- knock a fire hydrant over and maybe it would spray water out").
--
-- Each entry is one kind the street planner places along the kerbs (planStreetFurniture's march, beside the bins,
-- racks and news boxes it already places) and the loader draws, collides and breaks:
--
--   name          what it is (the log, the census)
--   every         metres of kerb between two of them (per side of a link)
--   verge         metres from the kerb line out onto the pavement
--   junction      metres clear of a junction (sightlines)
--   main_only     only along main streets
--   mesh()        the prop at the origin: y = 0 the pavement, +X along the street, +Z from the kerb to the buildings
--   collider      { half = {x, y, z} }: a box standing on the pavement
--   breaks_at     m/s: a car hitting it this hard knocks it loose (nil: it stops the car, as a tree does)
--   mass          kg once it is loose
--   spray         { effect, rate, up }: what comes out of the stub once it is knocked off (a hydrant's water)
--
-- The planner's own three (litter bin, bike rack, news boxes) move here when the street kit does (#191).

local props = {}

local function at(m, x, y, z) return mesh.translate(m, { x, y, z }) end

-- A FIRE HYDRANT: a red barrel on a flange, a domed bonnet with the operating nut, two hose outlets either side and
-- the pumper outlet to the street, its caps chained on. 0.78 m to the top of the nut.
local function hydrant()
  local red, cap, dark = { 0.72, 0.08, 0.06 }, { 0.93, 0.78, 0.12 }, { 0.22, 0.22, 0.22 }
  local parts = {
    mesh.paint(at(mesh.cylinder(0.15, 0.05), 0, 0.025, 0), dark),        -- the flange, bolted to the pavement
    mesh.paint(at(mesh.cylinder(0.105, 0.52), 0, 0.31, 0), red),         -- the barrel
    mesh.paint(at(mesh.cylinder(0.125, 0.05), 0, 0.10, 0), red),         -- the lower collar
    mesh.paint(at(mesh.cylinder(0.12, 0.05), 0, 0.57, 0), red),          -- the upper collar
    mesh.paint(at(mesh.cone(0.115, 0.14), 0, 0.66, 0), cap),             -- the bonnet
    mesh.paint(at(mesh.cylinder(0.025, 0.06), 0, 0.75, 0), dark),        -- the operating nut
  }
  -- the hose outlets, along the street, and the pumper outlet facing it (-Z)
  for _, sx in ipairs({ -1, 1 }) do
    parts[#parts + 1] = mesh.paint(at(mesh.rotate_z(mesh.cylinder(0.035, 0.12), math.pi * 0.5), sx * 0.15, 0.42, 0), red)
    parts[#parts + 1] = mesh.paint(at(mesh.rotate_z(mesh.cylinder(0.042, 0.03), math.pi * 0.5), sx * 0.215, 0.42, 0), cap)
  end
  parts[#parts + 1] = mesh.paint(at(mesh.rotate_x(mesh.cylinder(0.055, 0.12), math.pi * 0.5), 0, 0.40, -0.15), red)
  parts[#parts + 1] = mesh.paint(at(mesh.rotate_x(mesh.cylinder(0.062, 0.035), math.pi * 0.5), 0, 0.40, -0.215), cap)
  return mesh.merge(parts)
end

props[#props + 1] = {
  name = "hydrant",
  every = 90,
  verge = 0.55,
  junction = 6,
  main_only = false,
  mesh = hydrant,
  collider = { half = { 0.16, 0.39, 0.16 } },
  breaks_at = 2.0,
  mass = 90,   -- kg: cast iron
  spray = { effect = "splash", rate = 260, up = 7.5 },
}

-- WHAT THE PLANNER ALREADY PLACES, as bodies: a car knocks the light pieces loose and stops at the rest
props.legacy = {
  bin = { collider = { half = { 0.24, 0.50, 0.24 } }, breaks_at = 2.0, mass = 25 },          -- a steel bin, 1.0 m
  bike_rack = { collider = { half = { 0.95, 0.42, 0.10 } }, breaks_at = 4.0, mass = 35 },    -- three hoops along the kerb
  news_boxes = { collider = { half = { 0.55, 0.55, 0.28 } }, breaks_at = 3.0, mass = 45 },   -- a pair on a plinth
}
-- how a piece knocked loose leaves (ADR-0355): this share of the knock's speed on along it, and up `up` m/s plus
-- `up_per_speed` for each m/s of the knock
props.knock = { push = 0.6, up = 1.0, up_per_speed = 0.05 }
-- a street tree's trunk (it stops the car) and a lamp post's (so does it)
props.tree_trunk = { radius = 0.16, height = 2.6 }
props.lamp_post = { radius = 0.09, height = 4.4 }

return props
