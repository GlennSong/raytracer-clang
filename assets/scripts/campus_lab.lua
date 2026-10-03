-- campus_lab.lua -- THE UNIVERSITY's buildings, a storey of each cut away at head height to be seen from above (the
-- campus, milestone 1: the interiors). building.grow_floor grows exactly what the streamed interior grows.
--   front row, the GROUND floors:  a teaching hall's lecture halls and classrooms | the library's reading room |
--                                  a residence hall's dorms and its lounge
--   back row, an UPPER floor:      classrooms and labs | the stacks | dorm rooms and shared baths
--   behind them, the buildings WHOLE, from outside (milestone 2): the recipes teaching_hall, campus_library,
--   residence_hall, doors toward the floors
-- opts.seed rolls the buildings again.
local opts = args or {}
local seed = opts.seed or 1
local m = model.new()

local MATS = {
  interior          = material.new{ roughness = 0.85 },
  interior_floor    = material.new{ surface = "wood", roughness = 0.55 },
  interior_tile     = material.new{ surface = "concrete", roughness = 0.35 },
  interior_marble   = material.new{ surface = "marble", roughness = 0.25 },
  interior_carpet   = material.new{ surface = "carpet", roughness = 0.95 },
  glass             = material.new{ roughness = 0.06, opacity = 0.25 },
  glass_clear       = material.new{ roughness = 0.06, opacity = 0.25 },
  glass_lit         = material.new{ roughness = 0.06, opacity = 0.25 },
  furniture_wood    = material.new{ surface = "woodgrain", roughness = 0.5 },
  furniture_fabric  = material.new{ surface = "fabric", roughness = 0.92 },
  furniture         = material.new{ roughness = 0.55 },
  furniture_metal   = material.new{ roughness = 0.28, metallic = 0.85 },
  furniture_ceramic = material.new{ roughness = 0.12 },
  default           = material.new{ roughness = 0.8 },
  wall      = material.new{ roughness = 0.85 },
  brick     = material.new{ surface = "brick",    roughness = 0.88 },
  concrete  = material.new{ surface = "concrete", roughness = 0.92 },
  stucco    = material.new{ surface = "stucco",   roughness = 0.85 },
  metal     = material.new{ surface = "metal",    roughness = 0.45, metallic = 0.55 },
  trim      = material.new{ roughness = 0.7 },
  roof      = material.new{ roughness = 0.85 },
  door      = material.new{ roughness = 0.5 },
  detail    = material.new{ roughness = 0.4, metallic = 0.6 },
  lit_band  = material.new{ roughness = 0.5, emission = { 0.9, 0.8, 0.6 } },
}
local SKIP = { interior = true, interior_floor = true, interior_tile = true, interior_marble = true,
               interior_carpet = true }
local function rect(w, d) return {{-w/2, -d/2}, {w/2, -d/2}, {w/2, d/2}, {-w/2, d/2}} end

local HALLS = {
  { name = "teaching hall", recipe = "teaching_hall", plan = rect(52, 22), x = 0 },
  { name = "library", recipe = "campus_library", plan = rect(40, 24), x = 62 },
  { name = "residence hall", recipe = "residence_hall", plan = rect(54, 15), x = 120 },
}
for _, b in ipairs(HALLS) do
  for row, storey in ipairs({ 0, 1 }) do
    local parts = building.grow_floor{ recipe = b.recipe, seed = seed * 13 + row, coreness = 0.5, plan = b.plan,
                                       storey = storey, cutaway = 2.6 }
    local z = (row - 1) * -36
    for _, e in ipairs(parts) do
      m:add(mesh.translate(e.mesh, { b.x, 0, z }), MATS[e.part] or MATS.default)
    end
    print(string.format("[campus_lab] %s storey %d: %d parts", b.name, storey, #parts))
  end
  local outside = building.grow_plan_parts{ recipe = b.recipe, seed = seed * 13 + 1, coreness = 0.5, plan = b.plan }
  for _, e in ipairs(outside) do
    if not SKIP[e.part] then m:add(mesh.translate(e.mesh, { b.x, 0, -76 }), MATS[e.part] or MATS.default) end
  end
  print(string.format("[campus_lab] %s outside: %d parts", b.name, #outside))
end
return m
