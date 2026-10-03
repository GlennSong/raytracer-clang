-- attached_row.lua -- a DENSE STREET of attached buildings (Glenn, 2026-10-01: "in the dense part of town ...
-- buildings right next to each other ... windows aren't made on the sides and we have back doors and different
-- ways up to the second floor"). Each grown by the architect as a downtown lot would grow it, wall to wall: the
-- inner sides are PARTY WALLS (blank), the two ends keep their windows; at the back (-z) every one has a service
-- door, and the walk-ups a fire escape from the yard to the top floor. The street is on the +z side.
-- opts.seed rolls the row again.
local opts = args or {}
local seed0 = opts.seed or 1
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
  wood      = material.new{ roughness = 0.8 },
  lit_band  = material.new{ roughness = 0.5, emission = { 0.9, 0.8, 0.6 } },
  default   = material.new{ roughness = 0.8 },
}
local SKIP = { interior = true, interior_floor = true, interior_tile = true, interior_marble = true,
               interior_carpet = true, beacon_glow = true, beacon_haze = true }

-- the row, left to right: recipe and frontage (m); every lot 18 m deep
local ROW = {
  { "oldtown_house", 9 }, { "brick_shop", 12 }, { "mixed_use", 14 }, { "apartments", 13 },
  { "oldtown_cafe", 10 }, { "loft_block", 16 }, { "corner_shop", 11 },
}
local depth = 18
local x = 0
for i, r in ipairs(ROW) do
  local w = r[2]
  local plan = {{-w/2, -depth/2}, {w/2, -depth/2}, {w/2, depth/2}, {-w/2, depth/2}}
  local party = {}
  if i > 1 then party[#party + 1] = {-1, 0, w/2} end      -- the left neighbour
  if i < #ROW then party[#party + 1] = {1, 0, w/2} end    -- the right one
  local spec = { recipe = r[1], seed = seed0 * 31 + i, coreness = 0.4, plan = plan, party = party,
                 back_door = true, fire_escape = true }
  local parts = building.grow_plan_parts(spec)
  for _, e in ipairs(parts) do
    if not SKIP[e.part] then m:add(mesh.translate(e.mesh, {x + w/2, 0, 0}), MATS[e.part] or MATS.default) end
  end
  print(string.format("[attached_row] %s %.0f m: %d floors", r[1], w, spec.grown_floors or -1))
  x = x + w
end
return m
