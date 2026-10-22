-- scale_block.lua -- THE SCALE BLOCK (Glenn, 2026-10-18: "How does this compare to the buildings in the city,
-- ultimately they also need to fit with them so everything needs to be in proportion to each other ... Let's get the
-- scale right"): a street of the city's own things at their own sizes, to stand the trees against (scale_lab.json
-- plants them across the street, a row a species, an age a column).
--   north of the street (-z): a house, a four-storey walk-up, a six-storey shop block, an eight-storey office --
--     grown by the architect as a lot would grow them (the seed rolled until the storeys are those)
--   on it: a sedan, a street lamp, the city's people (1.8 m), and a 30 m measuring pole banded every metre (red every five)
local m = model.new()
local classes = require "vehicle_classes"
local forms   = require "vehicle_forms"

local MATS = {
  wall      = material.new{ roughness = 0.85 },
  brick     = material.new{ surface = "brick",    roughness = 0.88 },
  concrete  = material.new{ surface = "concrete", roughness = 0.92 },
  stucco    = material.new{ surface = "stucco",   roughness = 0.85 },
  metal     = material.new{ surface = "metal",    roughness = 0.45, metallic = 0.55 },
  glass     = material.new{ roughness = 0.08, metallic = 0.9 },
  glass_lit = material.new{ roughness = 0.08, metallic = 0.9 },
  glass_clear = material.new{ roughness = 0.05, metallic = 0.2, opacity = 0.45, two_sided = true },
  trim      = material.new{ roughness = 0.7 },
  roof      = material.new{ roughness = 0.85 },
  door      = material.new{ roughness = 0.5 },
  ground    = material.new{ surface = "pavement", roughness = 0.95 },
  detail    = material.new{ roughness = 0.4, metallic = 0.6 },
  wood      = material.new{ roughness = 0.8 },
  lit_band  = material.new{ roughness = 0.5, emission = { 0.9, 0.8, 0.6 } },
  default   = material.new{ roughness = 0.8 },
  road      = material.new{ surface = "asphalt", roughness = 0.95 },
  paving    = material.new{ surface = "pavement", roughness = 0.95 },
  body      = material.new{ roughness = 0.3, metallic = 0.5 },
  car_glass = material.new{ roughness = 0.05, metallic = 0.9 },
  rubber    = material.new{ roughness = 0.9 },
  lamp      = material.new{ roughness = 0.25, emission = { 1.0, 0.88, 0.55 } },
  pole_w    = material.new{ roughness = 0.6 },
  pole_k    = material.new{ roughness = 0.6 },
  pole_r    = material.new{ roughness = 0.6 },
  skin      = material.new{ roughness = 0.8 },   -- (the people: their colours are their vertices')
}
local SKIP = { interior = true, interior_floor = true, interior_tile = true, interior_marble = true,
               interior_carpet = true, beacon_glow = true, beacon_haze = true }

-- THE STREET: a local street's 2 x 3.2 m lanes, 3 m walks either side (road_rules.cpp), along x
local x0, x1 = -20, 150
m:add(scope{ origin = { x0, -0.02, -3.2 }, size = { x1 - x0, 0.02, 6.4 } }:box{ 0.22, 0.22, 0.23 }, MATS.road)
m:add(scope{ origin = { x0, 0.0, -6.4 }, size = { x1 - x0, 0.12, 3.2 } }:box{ 0.55, 0.54, 0.52 }, MATS.paving)
m:add(scope{ origin = { x0, 0.0, 3.2 }, size = { x1 - x0, 0.12, 3.0 } }:box{ 0.55, 0.54, 0.52 }, MATS.paving)

-- THE BUILDINGS, their fronts on the north walk (z = -6.4)
local function grow(recipe, want, x, w, d)
  local plan = {{-w/2, -d/2}, {w/2, -d/2}, {w/2, d/2}, {-w/2, d/2}}
  local best, bestParts, bestSeed = nil, nil, 0
  for seed = 1, 40 do
    local spec = { recipe = recipe, seed = seed, coreness = 0.4, plan = plan }
    local parts = building.grow_plan_parts(spec)
    local f = spec.grown_floors or -1
    if best == nil or math.abs(f - want) < math.abs(best - want) then best, bestParts, bestSeed = f, parts, seed end
    if f == want then break end
  end
  for _, e in ipairs(bestParts) do
    if not SKIP[e.part] then m:add(mesh.translate(e.mesh, { x, 0, -6.4 - d/2 }), MATS[e.part] or MATS.default) end
  end
  print(string.format("[scale_block] %s at x %.0f: %d floors over its ground floor (seed %d)", recipe, x, best, bestSeed))
end
grow("yard_house", 1, 8, 10, 12)          -- two storeys
grow("walkup_homes", 3, 30, 12, 16)       -- four
grow("brick_shop", 5, 52, 12, 16)         -- six
grow("office_midrise", 7, 78, 20, 20)     -- eight

-- A SEDAN in the near lane, along it (its length on z, turned onto x), A LAMP on the south walk (streetfurniture.lua's)
do
  local c = classes.classes["sedan"]
  local d = classes.dims(c)
  local car = mesh.car(forms.car_params(c, d, { color = { 0.72, 0.10, 0.10 }, lod = "high" }))
  local lift = car.half_extent[2]
  local function place(part, mat, y)
    if part then m:add(mesh.translate(mesh.rotate_y(part, math.pi * 0.5), { 4, y, 1.6 }), mat) end
  end
  place(car.body, MATS.body, lift); place(car.glass, MATS.car_glass, lift); place(car.lamp, MATS.lamp, lift)
  place(car.wheel, MATS.rubber, lift)
  if car.wheel == nil then   -- (as car_gallery.lua: the wheels from the package's own numbers)
    local r = c.wheel_diameter * 0.5
    local wheel = mesh.rotate_z(mesh.cylinder(r, math.max(0.18, c.track * 0.13), 14), math.pi * 0.5)
    local frontZ, rearZ = d.length * 0.5 - d.front_overhang, -(d.length * 0.5 - d.rear_overhang)
    for _, sx in ipairs({ 1, -1 }) do
      for _, wz in ipairs({ frontZ, rearZ }) do place(mesh.translate(wheel, { sx * c.track * 0.5, 0, wz }), MATS.rubber, r) end
    end
  end
end
m:add(mesh.translate(streetfurniture.lamp{ height = 4.6, pole_radius = 0.07, head_size = { 0.5, 0.2, 0.32 },
                                           head_color = { 1.0, 0.88, 0.55 } }, { 12, 0.12, 3.6 }), MATS.metal)

-- PEOPLE, the city's walkers (mesh.person: 1.8 m, origin at the feet): one by the pole, one at the walk-up's door,
-- one under the street lamp
for _, p in ipairs({ { 0.0, 4.4, 0, 0 }, { 30.0, -5.4, 3, math.pi }, { 13.2, 4.0, 7, -0.6 } }) do
  m:add(mesh.translate(mesh.rotate_y(mesh.person{ outfit = p[3] }, p[4]), { p[1], 0.12, p[2] }), MATS.skin)
end

-- THE MEASURING POLE: 30 m, a band a metre, red every fifth
for i = 0, 29 do
  local col = (i + 1) % 5 == 0 and { 0.8, 0.1, 0.1 } or (i % 2 == 0 and { 0.92, 0.92, 0.9 } or { 0.08, 0.08, 0.08 })
  local mat = (i + 1) % 5 == 0 and MATS.pole_r or (i % 2 == 0 and MATS.pole_w or MATS.pole_k)
  m:add(scope{ origin = { -4.15, 0.12 + i, 4.25 }, size = { 0.3, 1.0, 0.3 } }:box(col), mat)
end
return m
