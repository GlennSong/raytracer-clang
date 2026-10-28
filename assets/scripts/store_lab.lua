-- store_lab.lua -- A DEPARTMENT STORE'S FLOORS (the `department_store` floor program, floor_programs.lua; Glenn,
-- 2026-10-27: "a department store should be a whole floor"), cut away at head height and laid out to be seen from
-- above. building.grow_floor grows exactly what the streamed interior grows.
--   front row: a glass tower's store, storeys 1-5 -- fashion, fashion, electronics, books, the food hall
--   back row:  a podium tower's fashion floor
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
}
local function rect(w, d) return {{-w/2, -d/2}, {w/2, -d/2}, {w/2, d/2}, {-w/2, d/2}} end

local FLOORS = {}
for k = 1, 5 do
  FLOORS[#FLOORS + 1] = { recipe = "glass_tower", plan = rect(36, 32), x = (k - 3) * 44, z = 0, storey = k, floors = 24 }
end
FLOORS[#FLOORS + 1] = { recipe = "podium_tower", plan = rect(42, 32), x = -44, z = -44, storey = 1, floors = 18 }

for _, f in ipairs(FLOORS) do
  local parts = building.grow_floor{ recipe = f.recipe, seed = seed * 13 + 7, coreness = 1.0, plan = f.plan,
                                     floors = f.floors, store_floors = f.floors, storey = f.storey, cutaway = 2.6 }
  for _, e in ipairs(parts) do
    m:add(mesh.translate(e.mesh, {f.x, 0, f.z}), MATS[e.part] or MATS.default)
  end
  print(string.format("[store_lab] %s storey %d: %d parts", f.recipe, f.storey, #parts))
end
return m
