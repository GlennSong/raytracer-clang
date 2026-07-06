-- building_gallery.lua — every building ARCHETYPE side by side (the "see the
-- different building types" scene). Same hot-reload workflow as the lab: edit
-- this file (or the level opts) and the scene rebuilds; U reseeds everything.
--
-- Row 1 (back):   brick walk-up | stucco L-block | concrete slab | glass tiers
-- Row 2 (middle): flatiron      | metal warehouse | round tower  | pagoda
-- Row 3 (front):  painted row   | civic (pilaster)| brick tiers  | stucco U-court

local opts = args or {}
local seed = opts.seed or 5

local m = model.new()

local MATS = {
  brick    = material.new{ surface = "brick",    roughness = 0.88 },
  concrete = material.new{ surface = "concrete", roughness = 0.92 },
  stucco   = material.new{ surface = "stucco",   roughness = 0.85 },
  metal    = material.new{ surface = "metal",    roughness = 0.45, metallic = 0.55 },
  glass    = material.new{ roughness = 0.08, metallic = 0.9 },
  roof     = material.new{ roughness = 0.85 },
}

local function addParts(parts, off)
  for _, e in ipairs(parts) do
    m:add(mesh.translate(e.mesh, off), MATS[e.part])
  end
end

-- Plans centred on the origin (translated into place per cell).
local function rectPlan(w, d)
  return {{-w/2, -d/2}, {w/2, -d/2}, {w/2, d/2}, {-w/2, d/2}}
end
local function lPlan(w, d)
  return {{-w/2, -d/2}, {w/2, -d/2}, {w/2, 0}, {0, 0}, {0, d/2}, {-w/2, d/2}}
end
local function uPlan(w, d)   -- court opens toward +z
  local a = w * 0.30
  return {{-w/2, -d/2}, {w/2, -d/2}, {w/2, d/2}, {w/2 - a, d/2},
          {w/2 - a, -d/2 + a}, {-w/2 + a, -d/2 + a}, {-w/2 + a, d/2}, {-w/2, d/2}}
end
local function wedgePlan(w, d)
  return {{-w/2, -d/2}, {w/2, -d/2 + 2.5}, {w/2, d/2 - 3.5}, {-w/2 + 3, d/2}}
end

local SPECS = {
  { plan = rectPlan(18, 13), p = { floors = 4, style = "brick" } },
  { plan = lPlan(20, 18),    p = { floors = 3, style = "stucco" } },
  { plan = rectPlan(20, 14), p = { floors = 8, style = "concrete" } },
  { plan = rectPlan(24, 16), p = { floors = 14, style = "glass",
                                   setback_floors = 5, setback_every = 1.6 } },
  { plan = wedgePlan(20, 14), p = { floors = 6, style = "brick" } },
  { plan = rectPlan(22, 16), p = { floors = 1, style = "metal",
                                   solid_facade = true, ground_height = 7 } },
  { box = { shape = "cylinder", width = 16, depth = 16, floors = 12,
            style = "glass", sides = 36 } },
  { box = { shape = "pagoda", width = 14, depth = 14, tiers = 5,
            style = "painted" } },
  { plan = rectPlan(10, 11), p = { floors = 2, style = "painted",
                                   ground_retail = false } },
  { plan = rectPlan(20, 14), p = { floors = 3, style = "concrete",
                                   pilasters = true, ground_retail = false } },
  { plan = rectPlan(20, 16), p = { floors = 9, style = "brick",
                                   setback_floors = 3, setback_every = 1.4 } },
  { plan = uPlan(22, 18),    p = { floors = 3, style = "stucco" } },
}

local COLS, SPACING = 4, 38
for i, s in ipairs(SPECS) do
  local col = (i - 1) % COLS
  local row = math.floor((i - 1) / COLS)
  local ox = (col - (COLS - 1) / 2) * SPACING
  local oz = (row - 1) * SPACING
  m:add(scope{ origin = {ox - 15, -0.12, oz - 12},
               size = {30, 0.12, 24} }:box{0.50, 0.50, 0.48})
  local params = s.p or s.box
  params.seed = seed + i * 17
  params.walkable_ground = true
  if s.plan then
    params.plan = s.plan
    addParts(building.grow_plan_parts(params), {ox, 0, oz})
  else
    addParts(building.grow_parts(params), {ox, 0, oz})
  end
end

return m
