-- building_gallery.lua — every building ARCHETYPE side by side. Hot-reload
-- like the lab: edit + save and the scene rebuilds. U reseeds EVERYTHING
-- (massing jitter included); O sweeps one cladding style across the whole
-- gallery (press it six times to get back to each archetype's own style).
--
-- Row 1 (back):   brick walk-up | stucco L-block  | concrete slab | glass tiers
-- Row 2 (middle): flatiron      | metal warehouse | ROUND tower   | cruciform tower
-- Row 3 (front):  painted row   | civic (pilaster)| brick tiers   | stucco U-court
-- Row 4 (near):   chamfer tower | rounded flatiron| H-plan block  | octagon hall
--
-- Round/rounded masses are CURVED FLOORPLANS (chord-tessellated arcs) run
-- through the same plan grammar as everything else — doors, retail bases,
-- cornices and setback tiers included. No special-case cylinder.

local opts = args or {}
local seed = opts.seed or 5
local styleOverride = opts.style   -- O key: sweep one cladding over the gallery

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

-- ---- plan generators (centred on the origin) --------------------------------
local function rectPlan(w, d)
  return {{-w/2, -d/2}, {w/2, -d/2}, {w/2, d/2}, {-w/2, d/2}}
end
local function lPlan(w, d)
  return {{-w/2, -d/2}, {w/2, -d/2}, {w/2, 0}, {0, 0}, {0, d/2}, {-w/2, d/2}}
end
local function uPlan(w, d)
  local a = w * 0.30
  return {{-w/2, -d/2}, {w/2, -d/2}, {w/2, d/2}, {w/2 - a, d/2},
          {w/2 - a, -d/2 + a}, {-w/2 + a, -d/2 + a}, {-w/2 + a, d/2}, {-w/2, d/2}}
end
local function hPlan(w, d)
  local a = w * 0.30      -- leg width
  local c = d * 0.22      -- crossbar half-depth
  return {{-w/2, -d/2}, {-w/2 + a, -d/2}, {-w/2 + a, -c}, {w/2 - a, -c},
          {w/2 - a, -d/2}, {w/2, -d/2}, {w/2, d/2}, {w/2 - a, d/2},
          {w/2 - a, c}, {-w/2 + a, c}, {-w/2 + a, d/2}, {-w/2, d/2}}
end
local function cruciformPlan(w, d)
  local a = w * 0.28
  return {{-a, -d/2}, {a, -d/2}, {a, -a}, {w/2, -a}, {w/2, a}, {a, a},
          {a, d/2}, {-a, d/2}, {-a, a}, {-w/2, a}, {-w/2, -a}, {-a, -a}}
end
-- An arc of chords from angle a0 to a1 (radians) about (cx, cz).
local function arc(pts, cx, cz, r, a0, a1, n)
  for i = 0, n do
    local t = a0 + (a1 - a0) * i / n
    pts[#pts + 1] = {cx + r * math.cos(t), cz + r * math.sin(t)}
  end
end
local function circlePlan(r, n)
  local pts = {}
  for i = 0, n - 1 do
    local t = 2 * math.pi * i / n
    pts[#pts + 1] = {r * math.cos(t), r * math.sin(t)}
  end
  return pts
end
local function chamferPlan(w, d, c)
  return {{-w/2 + c, -d/2}, {w/2 - c, -d/2}, {w/2, -d/2 + c}, {w/2, d/2 - c},
          {w/2 - c, d/2}, {-w/2 + c, d/2}, {-w/2, d/2 - c}, {-w/2, -d/2 + c}}
end
local function roundedFlatiron(w, d)
  -- A wedge whose NOSE is a chord-tessellated arc: the classic flatiron prow.
  local pts = {{-w/2, -d/2}, {w/2 - 4, -d/2 + 1.0}}
  arc(pts, w/2 - 4, 0.6, 3.2, -0.55, 1.35, 6)   -- the curved prow
  pts[#pts + 1] = {-w/2 + 2.5, d/2}
  return pts
end
local function octagonPlan(r)
  return circlePlan(r, 8)
end

-- ---- the gallery -------------------------------------------------------------
local function jitter(i, span)   -- deterministic per-cell massing variety
  return ((seed * 37 + i * 101) % (span * 2 + 1)) - span
end

local SPECS = {
  { plan = rectPlan(18, 13), p = { floors = 4, style = "brick" } },
  { plan = lPlan(20, 18),    p = { floors = 3, style = "stucco" } },
  { plan = rectPlan(20, 14), p = { floors = 8, style = "concrete" } },
  { plan = rectPlan(24, 16), p = { floors = 14, style = "glass",
                                   setback_floors = 5, setback_every = 1.6 } },
  { plan = roundedFlatiron(20, 14), p = { floors = 6, style = "brick" } },
  { plan = rectPlan(22, 16), p = { floors = 1, style = "metal",
                                   solid_facade = true, ground_height = 7 } },
  { plan = circlePlan(9.5, 20), p = { floors = 12, style = "glass",
                                      setback_floors = 5, setback_every = 1.2 } },
  { plan = cruciformPlan(22, 22), p = { floors = 10, style = "concrete",
                                        setback_floors = 4, setback_every = 1.2 } },
  { plan = rectPlan(10, 13), p = { floors = 2, style = "painted",
                                   ground_retail = false, roof = "gable" } },
  { plan = rectPlan(20, 14), p = { floors = 3, style = "concrete",
                                   pilasters = true, ground_retail = false } },
  { plan = rectPlan(20, 16), p = { floors = 9, style = "brick",
                                   setback_floors = 3, setback_every = 1.4 } },
  { plan = uPlan(22, 18),    p = { floors = 3, style = "stucco" } },
  { plan = chamferPlan(18, 18, 4), p = { floors = 11, style = "glass",
                                         setback_floors = 4, setback_every = 1.3 } },
  { plan = rectPlan(16, 12), p = { floors = 2, style = "brick",
                                   ground_retail = false, roof = "hip",
                                   window_head = "flat" } },
  { plan = hPlan(24, 20),    p = { floors = 4, style = "brick" } },
  { plan = octagonPlan(9),   p = { floors = 3, style = "concrete",
                                   pilasters = true, ground_retail = false } },
}

local COLS, SPACING = 4, 38
for i, s in ipairs(SPECS) do
  local col = (i - 1) % COLS
  local row = math.floor((i - 1) / COLS)
  local ox = (col - (COLS - 1) / 2) * SPACING
  local oz = (row - 1.5) * SPACING
  m:add(scope{ origin = {ox - 15, -0.12, oz - 12},
               size = {30, 0.12, 24} }:box{0.50, 0.50, 0.48})
  local params = s.p
  params.plan = s.plan
  params.seed = seed + i * 17
  params.walkable_ground = true
  if styleOverride then params.style = styleOverride end
  -- Seed-driven massing jitter, so U shows real variety, not just tint.
  if params.floors > 2 then
    params.floors = math.max(2, params.floors + jitter(i, 2))
  end
  addParts(building.grow_plan_parts(params), {ox, 0, oz})
end

return m
