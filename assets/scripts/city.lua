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

-- A rotated rectangle footprint (4 corners) centred at (cx,cz), half-extents
-- hw,hd, turned by `ang` radians — the pad/lawn outline for an oriented lot.
local function rect(cx, cz, hw, hd, ang)
  local c, s = math.cos(ang), math.sin(ang)
  local function pt(dx, dz)
    return { x = cx + dx * c - dz * s, z = cz + dx * s + dz * c }
  end
  return { pt(-hw, -hd), pt(hw, -hd), pt(hw, hd), pt(-hw, hd) }
end

-- ----- blocks: partition each road-graph face into lots, level the developable
-- ones, and seat a building sized to FIT each lot --------------------------------
-- plan_blocks reads the NATURAL terrain (which faces are flat enough to develop,
-- and the plot height), subdivides each into parcels (city.lots), and for every
-- lot big enough for a building emits a pad (so terrain.conform cuts a level plot)
-- + a placement. Lots too small are skipped (left as a gap); faces too hilly to
-- level cheaply are left as natural green hillside.
function M.plan_blocks(base, lay, o)
  local flat_relief = opt(o, "flat_relief", 8)     -- max terrain rise to develop (m)
  local setback     = opt(o, "lot_setback", 2)     -- building inset within its lot (m)
  local min_edge    = opt(o, "min_building", 9)    -- skip lots too small to build on
  local park_frac   = opt(o, "park_fraction", 0.10)
  local pads, plan  = {}, {}
  for _, block in ipairs(lay.blocks) do
    local cx, cz = centroid(block)
    local lo, hi = 1e9, -1e9
    for _, p in ipairs(block) do
      local y = base:at(p.x, p.z); lo = math.min(lo, y); hi = math.max(hi, y)
    end
    if (hi - lo) <= flat_relief then
      local y = base:at(cx, cz)        -- the whole block levels to its centre height
      for _, lot in ipairs(city.lots(block,
          { target_area = opt(o, "lot_area", 520),
            min_area = opt(o, "lot_min_area", 150), seed = opt(o, "seed", 7) })) do
        local fw, fd = lot.w - 2 * setback, lot.d - 2 * setback
        if fw >= min_edge and fd >= min_edge then     -- a building actually fits
          pads[#pads + 1] = { y = y,
            poly = rect(lot.cx, lot.cz, fw * 0.5 + 1, fd * 0.5 + 1, lot.angle) }
          local park = (math.floor(lot.cx * 13 + lot.cz * 7) % 100) < (park_frac * 100)
          plan[#plan + 1] = { kind = park and "park" or "build",
            cx = lot.cx, cz = lot.cz, fw = fw, fd = fd, angle = lot.angle, y = y,
            r = math.sqrt(lot.cx * lot.cx + lot.cz * lot.cz) }
        end
      end
    end
  end
  return pads, plan
end

-- ----- building variety: archetype by district + a per-plot hash -------------
-- Downtown grows glass/metal towers (some round, set back as they climb);
-- midtown concrete/stucco/brick mid-rises; the residential fringe brick & painted
-- low-rises. So the skyline reads as a varied city, not a field of equal boxes.
function M.building_args(o, b)
  local h = math.floor(b.cx * 31 + b.cz * 17) % 997
  local a = { width = b.fw, depth = b.fd, floors = floors_at(o, b.r),
              ground_retail = true, walkable_ground = true,
              seed = math.floor(b.cx * 7 + b.cz) % 100000 }
  if b.r < opt(o, "downtown_radius", 55) then
    a.style = ({ "glass", "glass", "metal", "concrete" })[(h % 4) + 1]
    a.bay_width = 3.6 + (h % 3) * 0.4
    if a.floors >= 12 then a.setback_floors = 5; a.setback_every = 2.5 end
    if h % 6 == 0 then a.shape = "cylinder"; a.sides = 40 end       -- round towers
    if h % 47 == 0 then a.shape = "pagoda"; a.tiers = 5 end          -- a rare landmark
  elseif b.r < opt(o, "midtown_radius", 120) then
    a.style = ({ "concrete", "stucco", "painted", "brick" })[(h % 4) + 1]
    a.bay_width = 3.2 + (h % 4) * 0.3
    if h % 7 == 0 and a.floors >= 8 then a.setback_floors = 4; a.setback_every = 2.0 end
  else
    a.style = ({ "brick", "painted", "stucco" })[(h % 3) + 1]
    a.ground_retail = false
    a.bay_width = 3.0
  end
  return a
end

-- ----- place each planned lot on the conformed (level) terrain ----------------
-- Each lot is oriented (turned to its parcel), so the plinth/building/lawn is
-- built centred at the origin and `mesh.place`d at the lot centre + angle.
function M.place_blocks(m, plan, land, o)
  for _, b in ipairs(plan) do
    local hw, hd = b.fw * 0.5, b.fd * 0.5
    if b.kind == "build" then
      local plinth = scope{ origin = { -hw, -1.0, -hd }, size = { b.fw, 1.0, b.fd } }
                       :box(opt(o, "foundation_color", { 0.32, 0.32, 0.34 }))
      m:add_solid(mesh.place(plinth, { b.cx, b.y, b.cz }, b.angle))
      local bld = building.grow(M.building_args(o, b))
      m:add_solid(mesh.place(bld, { b.cx, b.y, b.cz }, b.angle))
    else
      -- A park: a flat manicured lawn on the same level plot (distinct green).
      local lawn = scope{ origin = { -hw, 0, -hd }, size = { b.fw, 0.2, b.fd } }
                     :box(opt(o, "park_color", { 0.24, 0.46, 0.17 }))
      m:add_solid(mesh.place(lawn, { b.cx, b.y, b.cz }, b.angle))
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
    extent = opt(o, "extent", 180), cell_size = opt(o, "cell_size", 84),
    ring_spacing = opt(o, "ring_spacing", 64), spokes = opt(o, "spokes", 12),
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
    curb = opt(o, "curb", 0.15), markings = opt(o, "markings", true),
    plaza = opt(o, "plaza", 0) }))

  M.place_blocks(m, plan, land, o)
  M.lamps(m, lay, land, o)
  return m
end

-- Entry point: the level's `opts` block arrives as the global `args` (see
-- setRecipeArgs), so the SAME recipe drives a grid city or a radial one straight
-- from the level. With no opts it falls back to a representative grid city. Edit
-- to `return M.tower(3)` etc. to preview a single building instead.
return M.generate(args or { pattern = "grid", seed = 7 })
