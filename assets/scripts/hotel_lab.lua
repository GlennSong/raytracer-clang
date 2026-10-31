-- hotel_lab.lua -- A HOTEL'S FLOORS (the `hotel` floor program, floor_programs.lua; Glenn, 2026-10-27: "lobby/front
-- desk, restaurants ... guest rooms, vip suites"), cut away at head height and laid out to be seen from above.
-- building.grow_floor grows exactly what the streamed interior grows.
--   two rows of three: the second floor of six hotels; the class (budget, business, luxury) is each one's own
--   right:     the lobby floor of the first
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
for k = 1, 6 do
  FLOORS[#FLOORS + 1] = { seed = seed * 7 + k, plan = rect(36, 24), x = ((k - 1) % 3) * 44, z = -math.floor((k - 1) / 3) * 32, storey = 2 }
end
FLOORS[#FLOORS + 1] = { seed = seed * 7 + 1, plan = rect(36, 24), x = 132, z = 0, storey = 0 }
for _, f in ipairs(FLOORS) do
  local parts = building.grow_floor{ recipe = "hotel", seed = f.seed, coreness = 1.0, plan = f.plan, storey = f.storey,
                                     cutaway = 2.6 }
  for _, e in ipairs(parts) do
    m:add(mesh.translate(e.mesh, {f.x, 0, f.z}), MATS[e.part] or MATS.default)
  end
  print(string.format("[hotel_lab] hotel seed %d storey %d: %d parts", f.seed, f.storey, #parts))
end
return m
