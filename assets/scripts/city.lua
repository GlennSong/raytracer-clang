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
-- hw,hd, turned by `yaw` radians — the pad outline for an oriented lot. The sign
-- matches Mat4::rotateY (what mesh.place uses) so the pad aligns with the part.
local function rect(cx, cz, hw, hd, yaw)
  local c, s = math.cos(yaw), math.sin(yaw)
  local function pt(dx, dz)
    return { x = cx + dx * c + dz * s, z = cz - dx * s + dz * c }
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
  local flat_relief = opt(o, "flat_relief", 11)    -- max terrain rise to develop (m)
  local min_lot     = opt(o, "min_lot", 12)        -- lot must hold a building + yards
  local park_frac   = opt(o, "park_fraction", 0.10)
  local center_r    = opt(o, "center_plaza", false) and opt(o, "ring_spacing", 60) * 0.6 or -1
  local pads, plan  = {}, {}
  for _, block in ipairs(lay.blocks) do
    local cx, cz = centroid(block)
    -- The island inside the roundabout: a plaza + monument (the Étoile centre),
    -- not subdivided into lots.
    if math.sqrt(cx * cx + cz * cz) < center_r then
      local rad = 0
      for _, p in ipairs(block) do
        rad = math.max(rad, math.sqrt((p.x - cx) ^ 2 + (p.z - cz) ^ 2))
      end
      local y = base:at(cx, cz)
      pads[#pads + 1] = { y = y, poly = block }     -- level the whole island
      plan[#plan + 1] = { kind = "plaza", cx = cx, cz = cz, rad = rad, y = y }
      goto next_block
    end
    do
    local lo, hi = 1e9, -1e9
    for _, p in ipairs(block) do
      local y = base:at(p.x, p.z); lo = math.min(lo, y); hi = math.max(hi, y)
    end
    if (hi - lo) <= flat_relief then
      local y = base:at(cx, cz)        -- the whole block levels to its centre height
      -- `inset` pulls the parcels in off the road centrelines (carriageway +
      -- sidewalk + a front setback) so lots sit inside the block, not the street.
      for _, lot in ipairs(city.lots(block,
          { inset = opt(o, "block_inset", 12), target_area = opt(o, "lot_area", 520),
            min_area = opt(o, "lot_min_area", 150), seed = opt(o, "seed", 7) })) do
        if lot.w >= min_lot and lot.d >= min_lot then     -- room for a plot
          -- The pad levels the WHOLE lot (garden + building), so the parcel reads flat.
          pads[#pads + 1] = { y = y,
            poly = rect(lot.cx, lot.cz, lot.w * 0.5 + 0.5, lot.d * 0.5 + 0.5, lot.yaw) }
          local park = (math.floor(lot.cx * 13 + lot.cz * 7) % 100) < (park_frac * 100)
          plan[#plan + 1] = { kind = park and "park" or "build",
            cx = lot.cx, cz = lot.cz, w = lot.w, d = lot.d, yaw = lot.yaw, y = y,
            r = math.sqrt(lot.cx * lot.cx + lot.cz * lot.cz) }
        end
      end
    end
    end   -- the developable-block branch
    ::next_block::
  end
  return pads, plan
end

-- ----- building variety: archetype by district + a per-plot hash -------------
-- Downtown grows glass/metal towers (some round, set back as they climb);
-- midtown concrete/stucco/brick mid-rises; the residential fringe brick & painted
-- low-rises. So the skyline reads as a varied city, not a field of equal boxes.
-- Returns variety params (style/floors/setbacks/shape); the placer adds the
-- width/depth sized to the building's footprint within the lot.
function M.building_args(o, b)
  local h = math.floor(b.cx * 31 + b.cz * 17) % 997
  local a = { floors = floors_at(o, b.r),
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

-- ----- lot dressing: how a parcel is filled, by district ----------------------
-- coverage = building's share of the lot; front = forecourt depth toward the
-- street; ground = the lot slab colour (paved downtown -> lawn outward); hedge =
-- a low frame around garden lots. So a lot is a building PLUS its surrounds.
local function lot_style(o, r)
  if r < opt(o, "downtown_radius", 55) then
    -- A paved concrete plaza, no garden — a dense downtown lot.
    return { coverage = 0.85, front = 2.0,
             ground = opt(o, "plaza_color", { 0.56, 0.56, 0.54 }), hedge = false }
  elseif r < opt(o, "midtown_radius", 120) then
    return { coverage = 0.70, front = 3.0, ground = { 0.26, 0.42, 0.18 }, hedge = true }
  end
  return { coverage = 0.55, front = 4.5, ground = { 0.28, 0.46, 0.20 }, hedge = true }
end

-- ----- place each planned lot on the conformed (level) terrain -----------------
-- Everything is built centred at the origin (the building shifted to its local
-- footprint) and mesh.place'd at the lot centre + yaw, so the parcel turns as a
-- unit: a level ground slab, an optional open-front hedge, then the building set
-- back behind its forecourt and squared to the street.
function M.place_blocks(m, plan, land, o)
  for _, b in ipairs(plan) do
    -- The roundabout's central island: a round paved plaza + a monument (an
    -- Arc/obelisk stand-in) the avenues radiate around.
    if b.kind == "plaza" then
      local pr = b.rad * 0.82
      m:add_solid(mesh.translate(mesh.cylinder(pr, 0.2), { b.cx, b.y, b.cz }))
      m:add_solid(mesh.translate(
        scope{ origin = { -4, 0, -4 }, size = { 8, 2.5, 8 } }
          :box(opt(o, "monument_color", { 0.60, 0.58, 0.55 })), { b.cx, b.y, b.cz }))
      m:add_solid(mesh.translate(
        scope{ origin = { -1.6, 2.5, -1.6 }, size = { 3.2, 16, 3.2 } }
          :box(opt(o, "monument_color", { 0.62, 0.60, 0.57 })), { b.cx, b.y, b.cz }))
      goto next_lot
    end
    do
    local sty = lot_style(o, b.r)
    local hw, hd = b.w * 0.5, b.d * 0.5
    local function place(part) m:add_solid(mesh.place(part, { b.cx, b.y, b.cz }, b.yaw)) end

    -- Level lot ground (lawn / paved forecourt), filling the parcel.
    place(scope{ origin = { -hw, -0.05, -hd }, size = { b.w, 0.2, b.d } }:box(sty.ground))

    -- Open-front hedge: a thin low frame on the sides + rear (garden lots only),
    -- leaving the street side open onto the forecourt.
    if sty.hedge then
      local t, hh, col = 0.4, 0.7, opt(o, "hedge_color", { 0.17, 0.33, 0.13 })
      for _, bar in ipairs({ { -hw, -hd, b.w, t }, { -hw, -hd, t, b.d },
                             { hw - t, -hd, t, b.d } }) do
        place(scope{ origin = { bar[1], 0, bar[2] }, size = { bar[3], hh, bar[4] } }:box(col))
      end
    end

    if b.kind == "build" then
      -- Building footprint within the lot, set back to leave a forecourt toward
      -- the street (+Z). Sized from coverage; skip if it comes out too small.
      local bw = b.w * sty.coverage
      local bd = (b.d - sty.front) * sty.coverage
      if bw >= 8 and bd >= 8 then
        local yaw, zoff = b.yaw, b.d * 0.5 - sty.front - bd * 0.5
        -- A deliberate diagonal: a small share of lots turn (and shrink to fit) —
        -- intentional variety, not the accidental corner-lot skew of before.
        local hsh = math.floor(b.cx * 7 + b.cz * 3) % 100
        if hsh < opt(o, "diagonal_chance", 6) then
          yaw = b.yaw + math.pi * 0.22 * ((hsh % 2 == 0) and 1 or -1)
          bw, bd, zoff = bw * 0.72, bd * 0.72, 0
        end
        local args = M.building_args(o, b); args.width = bw; args.depth = bd
        local hbw, hbd = bw * 0.5, bd * 0.5
        m:add_solid(mesh.place(
          scope{ origin = { -hbw, -1.0, zoff - hbd }, size = { bw, 1.0, bd } }
            :box(opt(o, "foundation_color", { 0.32, 0.32, 0.34 })),
          { b.cx, b.y, b.cz }, yaw))
        m:add_solid(mesh.place(mesh.translate(building.grow(args), { 0, 0, zoff }),
          { b.cx, b.y, b.cz }, yaw))
      end
    end
    end   -- the normal lot branch
    ::next_lot::
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

  -- Roads ride the conformed (flat) corridors, lifted just clear of the ground.
  -- `roadbed = true` dresses the junction-aware road_mesh (trimmed ribbons + pads,
  -- ADR-0044/0048) with raised sidewalks, lane markings that stop at the junctions,
  -- crosswalks at the intersection mouths, and a baked asphalt grain material. The
  -- ribbon only subdivides where the terrain bends, so it stays light + stable.
  if opt(o, "roadbed", false) then
    local grit  = texture.fbm{ seed = 7, scale = 40, octaves = 4 }:scale_bias(0.25, 0.82)
    local grain = texture.bake_color(grit, { 0.70, 0.70, 0.72 }, { 1.0, 1.0, 1.0 }, 256)
    local surf  = material.new{ albedo = grain, roughness = 0.93, tile = 7 }
    m:add_solid(city.road_mesh(lay, { height = land, lift = 0.3,
      color = {0.08, 0.08, 0.09}, sidewalk = opt(o, "sidewalk", 2.6),
      curb = opt(o, "curb", 0.16), markings = true,
      mark_width = opt(o, "mark_width", 0.18),
      crosswalks = opt(o, "crosswalks", true),
      corner_radius = opt(o, "corner_radius", 3.0),
      plaza = opt(o, "plaza", 0) }), surf)
  else
    m:add_solid(city.road_mesh(lay, { height = land, lift = 0.3,
      color = {0.08, 0.08, 0.09}, sidewalk = opt(o, "sidewalk", 2.5),
      curb = opt(o, "curb", 0.15), markings = opt(o, "markings", true),
      plaza = opt(o, "plaza", 0) }))
  end

  M.place_blocks(m, plan, land, o)
  M.lamps(m, lay, land, o)
  return m
end

-- Entry point: the level's `opts` block arrives as the global `args` (see
-- setRecipeArgs), so the SAME recipe drives a grid city or a radial one straight
-- from the level. With no opts it falls back to a representative grid city. Edit
-- to `return M.tower(3)` etc. to preview a single building instead.
return M.generate(args or { pattern = "grid", seed = 7 })
