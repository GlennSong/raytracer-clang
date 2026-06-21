-- city.lua — author a whole procedural city in Lua, or any part of it (ADR-0042
-- Phase 4). Tunable params drive terrain, roads, districts and props; the C++
-- solvers/grammars do the heavy lifting and this composes them into one model.
-- The same file is the per-building authoring surface (city.midrise / tower /
-- house) and the whole-city composer (city.generate{...}). Edit the return at the
-- bottom to preview a single building or a whole city.

local M = {}

-- ----- per-building archetypes (each returns one mesh) ------------------------
function M.midrise(seed)
  return building.grow{ floors = 4 + (seed or 0) % 4, width = 18, depth = 14,
    ground_retail = true, walkable_ground = true, bay_width = 3.6, seed = seed or 1 }
end
function M.tower(seed)
  return building.grow{ floors = 20 + (seed or 0) % 20, width = 26, depth = 22,
    ground_retail = true, walkable_ground = true, bay_width = 4.2,
    setback_floors = 8, setback_every = 3.0, seed = seed or 1 }
end
function M.house(seed)
  return building.grow{ floors = 1 + (seed or 0) % 2, width = 12, depth = 10,
    ground_retail = false, walkable_ground = true, bay_width = 3.0, seed = seed or 1 }
end

-- ----- helpers ----------------------------------------------------------------
local function opt(o, k, v) if o[k] == nil then return v else return o[k] end end
local function centroid(poly)
  local cx, cz = 0, 0
  for _, p in ipairs(poly) do cx = cx + p.x; cz = cz + p.z end
  return cx / #poly, cz / #poly
end
local function extent(poly, cx, cz)
  local w, d = 0, 0
  for _, p in ipairs(poly) do
    w = math.max(w, math.abs(p.x - cx) * 2)
    d = math.max(d, math.abs(p.z - cz) * 2)
  end
  return w, d
end

-- ----- terrain: a composed heightfield (optionally eroded) --------------------
function M.terrain(o)
  o = o or {}
  local seed = opt(o, "seed", 7)
  local base = terrain.fbm{ seed = seed, freq = opt(o, "freq", 0.012),
                            amp = opt(o, "amp", 18), octaves = 5 }
  local land = terrain.warp(base, terrain.fbm{ seed = seed + 1, freq = 0.02, amp = 1 },
                            opt(o, "warp", 28))
  if o.eroded then
    land = terrain.erode(land, { size = opt(o, "size", 460), resolution = 384,
                                 droplets = opt(o, "droplets", 60000), seed = seed })
  end
  return land
end

-- ----- districts: storey count by distance from the centre --------------------
local function floors_at(o, r)
  if r < opt(o, "downtown_radius", 55) then
    return opt(o, "downtown_floors", 9) + (math.floor(r) % 12)
  elseif r < opt(o, "midtown_radius", 120) then
    return opt(o, "midtown_floors", 4) + (math.floor(r) % 5)
  end
  return opt(o, "residential_floors", 2) + (math.floor(r) % 3)
end

-- ----- blocks: classify each road-graph face, flatten the developable ones to a
-- level plot, and decide its contents (a building, or a park) ------------------
-- plan_blocks reads the NATURAL terrain (which faces are flat enough to develop,
-- and the plot heights); it returns `pads` (fed to terrain.conform so the ground
-- is cut/filled to a level plot) and a `plan` that drives placement once the
-- conformed land exists. Faces too hilly to level cheaply are left natural.
function M.plan_blocks(base, lay, o)
  local fill        = opt(o, "lot_fill", 0.55)
  local flat_relief = opt(o, "flat_relief", 8)    -- max terrain rise to develop (m)
  local pad_margin  = opt(o, "pad_margin", 3)
  local park_frac   = opt(o, "park_fraction", 0.12)
  local pads, plan  = {}, {}
  for _, block in ipairs(lay.blocks) do
    local cx, cz = centroid(block)
    local bw, bd = extent(block, cx, cz)
    local lo, hi = 1e9, -1e9
    for _, p in ipairs(block) do
      local y = base:at(p.x, p.z); lo = math.min(lo, y); hi = math.max(hi, y)
    end
    if (hi - lo) <= flat_relief and bw > 16 and bd > 16 then
      local fw, fd = math.max(8, bw * fill), math.max(8, bd * fill)
      local hw, hd = fw * 0.5 + pad_margin, fd * 0.5 + pad_margin
      local y = base:at(cx, cz)
      pads[#pads + 1] = { y = y, poly = {
        { x = cx - hw, z = cz - hd }, { x = cx + hw, z = cz - hd },
        { x = cx + hw, z = cz + hd }, { x = cx - hw, z = cz + hd } } }
      local park = (math.floor(cx * 13 + cz * 7) % 100) < (park_frac * 100)
      plan[#plan + 1] = { kind = park and "park" or "build",
        cx = cx, cz = cz, fw = fw, fd = fd, y = y,
        r = math.sqrt(cx * cx + cz * cz) }
    end
    -- else: undeveloped hillside, left as the natural green terrain.
  end
  return pads, plan
end

-- ----- place each planned block on the conformed (level) terrain --------------
function M.place_blocks(m, plan, land, o)
  for _, b in ipairs(plan) do
    local hw, hd = b.fw * 0.5, b.fd * 0.5
    if b.kind == "build" then
      -- The plot is flat at b.y, so just a short foundation plinth, then the tower.
      m:add_solid(scope{ origin = { b.cx - hw, b.y - 1.0, b.cz - hd },
                         size = { b.fw, 1.0, b.fd } }
                  :box(opt(o, "foundation_color", { 0.32, 0.32, 0.34 })))
      local floors = floors_at(o, b.r)
      local bld = building.grow{ floors = floors, width = b.fw, depth = b.fd,
        ground_retail = true, walkable_ground = true,
        setback_floors = (floors >= 14) and 6 or 0, setback_every = 2.5,
        seed = math.floor(b.cx * 7 + b.cz) % 100000 }
      m:add_solid(mesh.translate(bld, { b.cx, b.y, b.cz }))
    else
      -- A park: a flat manicured lawn on the same level plot (distinct green).
      m:add_solid(scope{ origin = { b.cx - hw, b.y, b.cz - hd },
                         size = { b.fw, 0.2, b.fd } }
                  :box(opt(o, "park_color", { 0.24, 0.46, 0.17 })))
    end
  end
end

-- ----- instanced street lamps along both verges -------------------------------
function M.lamps(m, lay, land, o)
  local lamp = streetfurniture.lamp{}
  local places = {}
  for _, e in ipairs(lay.edges) do
    local a, b = lay.nodes[e.a], lay.nodes[e.b]
    local dx, dz = b.x - a.x, b.z - a.z
    local len = math.sqrt(dx * dx + dz * dz)
    if len > 22 then
      local nx, nz = -dz / len, dx / len
      local off = e.width * 0.5 + 1.5
      local mx, mz = (a.x + b.x) * 0.5, (a.z + b.z) * 0.5
      for _, s in ipairs({ 1, -1 }) do
        local px, pz = mx + nx * off * s, mz + nz * off * s
        places[#places + 1] = { pos = { px, land:at(px, pz), pz } }
      end
    end
  end
  -- Each lamp is solid: a thin vertical capsule around the pole (the correct
  -- collider for a post — you walk into it, not a bounding box).
  m:add_instances(lamp, places,
    { collide = { shape = "capsule", radius = 0.3, height = 5 } })
end

-- ----- the whole city ---------------------------------------------------------
-- opts (all optional): seed, pattern ("grid"|"radial"), extent, cell_size,
--   ring_spacing, spokes, jitter; terrain_amp, terrain_freq, terrain_size,
--   terrain_resolution, eroded, droplets, ground_color; downtown_radius,
--   midtown_radius, *_floors, lot_fill, foundation_color.
function M.generate(o)
  o = o or {}
  local seed = opt(o, "seed", 7)
  local size = opt(o, "terrain_size", 460)

  -- The natural landscape, before the city imposes itself on it.
  local base = M.terrain{ seed = seed, freq = opt(o, "terrain_freq", 0.012),
    amp = opt(o, "terrain_amp", 18), warp = opt(o, "terrain_warp", 28),
    size = size, eroded = o.eroded, droplets = o.droplets }

  -- Lay the road network, plan the blocks on the natural terrain (which faces to
  -- develop, where the level plots are), then CONFORM the terrain to BOTH the
  -- roads and those plots: streets become flat/constant-grade corridors and each
  -- developed block a level plot, with the ground cut/filled to meet them instead
  -- of bending over every bump. `land` is what everything downstream samples.
  local lay = city.layout{ pattern = opt(o, "pattern", "grid"),
    extent = opt(o, "extent", 170), cell_size = opt(o, "cell_size", 64),
    ring_spacing = opt(o, "ring_spacing", 60), spokes = opt(o, "spokes", 12),
    jitter = opt(o, "jitter", 0.10), seed = seed }
  local pads, plan = M.plan_blocks(base, lay, o)
  local land = terrain.conform(base, lay, { margin = opt(o, "road_margin", 3),
    falloff = opt(o, "road_falloff", 10), pads = pads,
    pad_falloff = opt(o, "pad_falloff", 8) })

  local m = model.new()
  -- The terrain is the ground you walk on: render it and collide with the exact
  -- same surface (no approximation), so footing follows every contour.
  local ground = terrain.mesh(land, { size = size,
    resolution = opt(o, "terrain_resolution", 220),
    color = opt(o, "ground_color", {0.33, 0.40, 0.26}) })
  m:add(ground)
  m:collide(ground, { friction = 0.95 })

  -- Roads ride the conformed (flat) corridors, lifted just clear of the ground,
  -- with raised sidewalks skirting both verges (curb lip + concrete slab).
  m:add_solid(city.road_mesh(lay, { height = land, lift = 0.3,
    color = {0.08, 0.08, 0.09}, sidewalk = opt(o, "sidewalk", 2.5),
    curb = opt(o, "curb", 0.15), markings = opt(o, "markings", true) }))

  M.place_blocks(m, plan, land, o)
  M.lamps(m, lay, land, o)
  return m
end

-- Default entry point: a representative grid city on rolling terrain. Change the
-- opts (or return M.tower(3)) to author a different city or a single building.
return M.generate{ pattern = "grid", seed = 7 }
