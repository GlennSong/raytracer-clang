-- big_box_lab.lua -- the BIG-BOX STORES (Glenn, 2026-10-01: "Big box stores like Costco or Bestbuy"), one per
-- chain, as the city grows them (building.grow_plan_parts{recipe = "big_box", big_box = chain}):
--   front row: the four stores from outside -- warehouse club | electronics | home improvement | discount store
--   back row:  the same four floors from above, cut away at head height -- checkouts, sales floor, stockroom
-- The doors face +z. The parking lot is the lot pass's (sculptParking), not drawn here: see the city.
local opts = args or {}
local seed = opts.seed or 1
local m = model.new()

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
  lit_band  = material.new{ roughness = 0.5, emission = { 0.9, 0.8, 0.6 } },
  interior          = material.new{ roughness = 0.85 },
  interior_floor    = material.new{ surface = "wood", roughness = 0.55 },
  interior_tile     = material.new{ surface = "concrete", roughness = 0.35 },
  interior_marble   = material.new{ surface = "marble", roughness = 0.25 },
  interior_carpet   = material.new{ surface = "carpet", roughness = 0.95 },
  furniture_wood    = material.new{ surface = "woodgrain", roughness = 0.5 },
  furniture_fabric  = material.new{ surface = "fabric", roughness = 0.92 },
  furniture         = material.new{ roughness = 0.55 },
  furniture_metal   = material.new{ roughness = 0.28, metallic = 0.85 },
  furniture_ceramic = material.new{ roughness = 0.12 },
  default   = material.new{ roughness = 0.8 },
}
local SKIP = { interior = true, interior_floor = true, interior_tile = true, interior_marble = true,
               interior_carpet = true }

local W, D = 80, 56
local plan = {{-W/2, -D/2}, {W/2, -D/2}, {W/2, D/2}, {-W/2, D/2}}
local NAMES = { "warehouse club", "electronics", "home improvement", "discount store" }
for chain = 1, 4 do
  local x = (chain - 1) * (W + 20)
  local parts = building.grow_plan_parts{ recipe = "big_box", seed = seed * 17 + chain, coreness = 0.2, plan = plan,
                                          big_box = chain }
  for _, e in ipairs(parts) do
    if not SKIP[e.part] then m:add(mesh.translate(e.mesh, {x, 0, 0}), MATS[e.part] or MATS.default) end
  end
  local floor = building.grow_floor{ recipe = "big_box", seed = seed * 17 + chain, coreness = 0.2, plan = plan,
                                     big_box = chain, storey = 0, cutaway = 3.0 }
  for _, e in ipairs(floor) do
    m:add(mesh.translate(e.mesh, {x, 0, -(D + 24)}), MATS[e.part] or MATS.default)
  end
  print(string.format("[big_box_lab] %s: %d exterior parts, %d floor parts", NAMES[chain], #parts, #floor))
end
return m
