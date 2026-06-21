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

-- ----- buildings: one per block, seated on the terrain with a foundation ------
function M.buildings(m, lay, land, o)
  local fill = opt(o, "lot_fill", 0.55)
  for _, block in ipairs(lay.blocks) do
    local cx, cz = centroid(block)
    local bw, bd = extent(block, cx, cz)
    local fw, fd = math.max(8, bw * fill), math.max(8, bd * fill)
    local hw, hd = fw * 0.5, fd * 0.5
    local hi, lo = -1e9, 1e9
    for _, c in ipairs({ {cx-hw,cz-hd}, {cx+hw,cz-hd}, {cx+hw,cz+hd}, {cx-hw,cz+hd}, {cx,cz} }) do
      local y = land:at(c[1], c[2]); hi = math.max(hi, y); lo = math.min(lo, y)
    end
    local floors = floors_at(o, math.sqrt(cx * cx + cz * cz))
    m:add(scope{ origin = {cx - hw, lo - 1.0, cz - hd}, size = {fw, (hi - lo) + 1.0, fd} }
            :box(opt(o, "foundation_color", {0.32, 0.32, 0.34})))
    local bld = building.grow{ floors = floors, width = fw, depth = fd,
      ground_retail = true, walkable_ground = true,
      setback_floors = (floors >= 14) and 6 or 0, setback_every = 2.5,
      seed = math.floor(cx * 7 + cz) % 100000 }
    m:add(mesh.translate(bld, { cx, hi, cz }))
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
  m:add_instances(lamp, places)
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

  local land = M.terrain{ seed = seed, freq = opt(o, "terrain_freq", 0.012),
    amp = opt(o, "terrain_amp", 18), warp = opt(o, "terrain_warp", 28),
    size = size, eroded = o.eroded, droplets = o.droplets }

  local m = model.new()
  m:add(terrain.mesh(land, { size = size, resolution = opt(o, "terrain_resolution", 220),
                             color = opt(o, "ground_color", {0.33, 0.40, 0.26}) }))

  local lay = city.layout{ pattern = opt(o, "pattern", "grid"),
    extent = opt(o, "extent", 170), cell_size = opt(o, "cell_size", 64),
    ring_spacing = opt(o, "ring_spacing", 60), spokes = opt(o, "spokes", 12),
    jitter = opt(o, "jitter", 0.10), seed = seed }
  m:add(city.road_mesh(lay, { height = land, lift = 0.5, color = {0.08, 0.08, 0.09} }))

  M.buildings(m, lay, land, o)
  M.lamps(m, lay, land, o)
  return m
end

-- Default entry point: a representative grid city on rolling terrain. Change the
-- opts (or return M.tower(3)) to author a different city or a single building.
return M.generate{ pattern = "grid", seed = 7 }
