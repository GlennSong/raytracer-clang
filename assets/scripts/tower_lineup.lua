-- tower_lineup.lua -- the city's TOWER recipes side by side (Glenn, 2026-09-30: "I'd love to see the new
-- towers in a separate scene just to see them lined up"). Each is grown by the architect exactly as a
-- downtown lot would grow it (building.grow_plan_parts{recipe = ...}: the recipe's own dice, the slenderness
-- cap on its plan), on a lot shaped for it, in lines along +x (the street is on the +z side):
--   the New York forms: sky exposure | tower on a base | tapered glass | glass slab | pencil
--   then the towers that came before: glass tower | podium tower | art deco | stepped | office slab
-- opts.rows (default 3) lines, each rolled on its own dice; opts.seed (level JSON) rolls them all again --
-- glass tint, cladding, fins, height; opts.coreness (default 1)
-- is how deep downtown the lots sit (lower = shorter towers).

local opts = args or {}
local seed0 = opts.seed or 1
local coreness = opts.coreness or 1.0

local m = model.new()

local MATS = {
  wall      = material.new{ roughness = 0.85 },
  brick     = material.new{ surface = "brick",    roughness = 0.88 },
  concrete  = material.new{ surface = "concrete", roughness = 0.92 },
  stucco    = material.new{ surface = "stucco",   roughness = 0.85 },
  metal     = material.new{ surface = "metal",    roughness = 0.45, metallic = 0.55 },
  -- script materials take the vertex colour, which on glass IS the building's glass (shape_grammar glassGrey / tint)
  glass     = material.new{ roughness = 0.08, metallic = 0.9 },
  glass_lit = material.new{ roughness = 0.08, metallic = 0.9 },
  glass_clear = material.new{ roughness = 0.05, metallic = 0.2, opacity = 0.45, two_sided = true },
  trim      = material.new{ roughness = 0.7 },
  roof      = material.new{ roughness = 0.85 },
  door      = material.new{ roughness = 0.5 },
  ground    = material.new{ surface = "pavement", roughness = 0.95 },
  detail    = material.new{ roughness = 0.4, metallic = 0.6 },
  wood      = material.new{ roughness = 0.8 },
  path      = material.new{ surface = "pavement", roughness = 0.95 },
  foliage   = material.new{ roughness = 0.95 },
  vent      = material.new{ roughness = 0.6, metallic = 0.4 },
  utility   = material.new{ roughness = 0.6, metallic = 0.3 },
  fan       = material.new{ roughness = 0.6, metallic = 0.4 },
  lit_band  = material.new{ roughness = 0.5, emission = { 0.9, 0.8, 0.6 } },
  beacon    = material.new{ roughness = 0.3, emission = { 0.8, 0.05, 0.03 } },
}
-- the interior, the flooring and the beacons' translucent spheres stay out of a daylight lineup
local SKIP = { interior = true, interior_floor = true, interior_tile = true, interior_marble = true,
               interior_carpet = true, beacon_glow = true, beacon_haze = true }

local function rect(w, d) return {{-w/2, -d/2}, {w/2, -d/2}, {w/2, d/2}, {-w/2, d/2}} end

-- each recipe on a lot shaped for it (width along x, depth along z)
local LOTS = {
  sky_exposure_tower  = {52, 44},   -- a wide midtown lot: room for the steps
  tower_on_base       = {46, 38},
  tapered_glass_tower = {40, 40},
  glass_slab          = {66, 26},   -- long and shallow: the slab runs its length
  pencil_tower        = {24, 24},   -- a small lot on Billionaires' Row
  glass_tower         = {40, 34},
  podium_tower        = {46, 38},
  art_deco_tower      = {44, 38},
  stepped_tower       = {44, 38},
  office_slab         = {44, 30},
}
local LINE = {"sky_exposure_tower", "tower_on_base", "tapered_glass_tower", "glass_slab", "pencil_tower",
              "glass_tower", "podium_tower", "art_deco_tower", "stepped_tower", "office_slab"}

-- ROWS lines of the same ten, each on its own dice (the glass, cladding and height vary row to row), 120 m
-- apart: walk the avenues between them.
local ROWS = opts.rows or 3
local GAP = 28
for row = 1, ROWS do
  local z = -(row - 1) * 120
  local x = 0
  for i, name in ipairs(LINE) do
    local w, d = LOTS[name][1], LOTS[name][2]
    local cx = x + w / 2
    local req = { recipe = name, seed = seed0 * 101 + row * 1009 + i * 7, coreness = coreness, plan = rect(w, d) }
    local parts = building.grow_plan_parts(req)
    for _, e in ipairs(parts) do
      if not SKIP[e.part] then
        m:add(mesh.translate(e.mesh, {cx, 0, z}), MATS[e.part] or MATS.wall)
      end
    end
    -- the lot's paving, so each stands on its own plate
    m:add(scope{ origin = {x - 2, -0.1, z - d/2 - 2}, size = {w + 4, 0.1, d + 4} }:box{ 0.55, 0.55, 0.53 }, MATS.ground)
    print(string.format("[lineup] row %d %-20s %2d floors  lot %dx%d  at x %.0f z %.0f", row, name, req.grown_floors or -1, w, d, cx, z))
    x = x + w + GAP
  end
end
return m
