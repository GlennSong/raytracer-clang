-- floor_lab.lua -- ONE STOREY of a building's interior, cut away at head height and laid out to be seen from above
-- (buildings B; Glenn, 2026-09-30: "make entire apartments out of the floor"). building.grow_floor grows exactly
-- what the streamed interior grows -- rooms, walls, floors, the furniture -- so this is the plan the city has.
--   left:  a condo tower's apartment floor        right: a podium tower's office floor
-- opts.seed rolls the buildings again; opts.storey picks the floor.
local opts = args or {}
local seed = opts.seed or 1
local storey = opts.storey or 3
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
}
local function rect(w, d) return {{-w/2, -d/2}, {w/2, -d/2}, {w/2, d/2}, {-w/2, d/2}} end

local FLOORS = {
  { recipe = "condo_tower", plan = rect(44, 32), x = 0 },
  { recipe = "podium_tower", plan = rect(42, 32), x = 56 },
}
for _, f in ipairs(FLOORS) do
  local parts = building.grow_floor{ recipe = f.recipe, seed = seed * 13 + 7, coreness = 1.0, plan = f.plan,
                                     storey = storey, cutaway = 2.6 }
  for _, e in ipairs(parts) do
    m:add(mesh.translate(e.mesh, {f.x, 0, 0}), MATS[e.part] or MATS.default)
  end
  print(string.format("[floor_lab] %s storey %d: %d parts", f.recipe, storey, #parts))
end
return m
