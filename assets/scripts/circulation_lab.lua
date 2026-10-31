-- circulation_lab.lua -- STAIRS AND LIFTS PLACED APART (circulation.h, issue #105 step 3b; Glenn, 2026-10-15:
-- "disconnect them and place stairs and elevators separately and also create elevator setups like you see in real
-- buildings"), each floor cut away at head height and laid out to be seen from above.
--   the columns: stairs = "core" with lifts = "bank" (the classic core), stairs = "ends", lifts = "facing", both
--   the rows:    a hotel slab, an office tower, a block of flats
-- building.grow_floor grows exactly what the streamed interior grows. opts.seed rolls the buildings again,
-- opts.storey picks the floor.
local opts = args or {}
local seed = opts.seed or 1
local storey = opts.storey or 2
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

local LAYOUTS = {
  { stairs = "core", lifts = "bank" },
  { stairs = "ends", lifts = "bank" },
  { stairs = "core", lifts = "facing" },
  { stairs = "ends", lifts = "facing" },
}
local ROWS = {
  { recipe = "hotel",       plan = rect(52, 20) },
  { recipe = "glass_tower", plan = rect(60, 48) },
  { recipe = "apartments",  plan = rect(48, 22) },
}
for r, row in ipairs(ROWS) do
  for c, lay in ipairs(LAYOUTS) do
    local parts = building.grow_floor{ recipe = row.recipe, seed = seed * 7 + r, coreness = 1.0, plan = row.plan,
                                       storey = storey, cutaway = 2.6, stairs = lay.stairs, lifts = lay.lifts }
    local x, z = (c - 1) * 70, -(r - 1) * 56
    for _, e in ipairs(parts) do
      m:add(mesh.translate(e.mesh, {x, 0, z}), MATS[e.part] or MATS.default)
    end
    print(string.format("[circulation_lab] %s stairs=%s lifts=%s storey %d: %d parts", row.recipe, lay.stairs,
                        lay.lifts, storey, #parts))
  end
end
return m
