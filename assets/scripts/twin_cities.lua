-- twin_cities.lua — a radial city and a grid/tensor city on the engine's CDLOD
-- terrain, joined by a highway (ADR-0044). The recipe samples the injected
-- `ground` heightfield to seat everything, and hands its cut/fill footprints back
-- via m:conform so the loader grades the walkable terrain flat under the roads,
-- blocks and the connecting road. Drives city.layout in two patterns:
--   * a "radial" Étoile (rings + avenues) — the left city
--   * the new "tensor" field (radial core relaxing to a grid rim) — the right city
-- and scatters a simple L-system forest along the highway and city fringes.

-- The level injects `ground` (the natural terrain height). Fall back to flat so
-- the recipe still runs standalone for a quick preview.
local land = ground or terrain.flat(0)

local m = model.new()

-- ---- L-system tree (same grammar as lsystem_tree.lua, inlined: the sandbox has
--      no require) -------------------------------------------------------------
local function make_tree(seed)
  seed = math.floor(seed or 1)
  local sys = lsystem.create()
  sys:rule("F", "F[+&F][-^F][/&F][\\&F]")
  local sym = sys:expand("FF", 3, seed)
  local tp = { length = 1.0, radius = 0.26, angle_deg = 38, taper = 0.7, segment_slices = 5 }
  local branches = mesh.bake_height_color(lsystem.turtle_mesh(sym, tp),
                                          { 0.30, 0.21, 0.13 }, { 0.42, 0.31, 0.19 })
  local segs = lsystem.segments(sym, tp)
  local maxy = 0.001
  for _, s in ipairs(segs) do if s.b[2] > maxy then maxy = s.b[2] end end
  local tips = {}
  for _, s in ipairs(segs) do if s.b[2] > maxy * 0.55 then tips[#tips + 1] = s.b end end
  local blobs, stride = {}, math.max(1, math.floor(#tips / 14))
  for i = 1, #tips, stride do
    local r = 0.85 + 0.5 * ((seed + #blobs) % 4) / 4
    blobs[#blobs + 1] = mesh.translate(mesh.sphere(r), tips[i])
  end
  if #blobs == 0 then return branches end
  return mesh.merge({ branches,
    mesh.bake_height_color(mesh.merge(blobs), { 0.16, 0.33, 0.15 }, { 0.27, 0.47, 0.21 }) })
end
local tree_proto = make_tree(7)

-- ---- helpers ----------------------------------------------------------------
local function translate_layout(lay, dx, dz)
  local out = { nodes = {}, edges = lay.edges, blocks = {} }
  for i, n in ipairs(lay.nodes) do out.nodes[i] = { x = n.x + dx, z = n.z + dz } end
  for i, b in ipairs(lay.blocks) do
    local nb = {}
    for j, p in ipairs(b) do nb[j] = { x = p.x + dx, z = p.z + dz } end
    out.blocks[i] = nb
  end
  return out
end
local function centroid(poly)
  local x, z = 0, 0
  for _, p in ipairs(poly) do x = x + p.x; z = z + p.z end
  return x / #poly, z / #poly
end

-- Build one city centred at (cx,cz); `kind` tunes the district storey profile.
local function build_city(opts, cx, cz, kind)
  local lay = translate_layout(city.layout(opts), cx, cz)

  -- A flat pad at each block, seated at the natural ground under its centroid, so
  -- terrain.conform levels the plot the buildings stand on.
  local pads = {}
  for _, blk in ipairs(lay.blocks) do
    local px, pz = centroid(blk)
    pads[#pads + 1] = { y = land:at(px, pz), poly = blk }
  end
  local field, regions = terrain.conform(land, lay,
    { margin = 3, falloff = 16, pads = pads, pad_falloff = 12 })
  m:conform(regions)                                  -- grade the engine terrain

  -- Road surface (draped on the conformed field), collidable so you can walk it.
  m:add_solid(city.road_mesh(lay, { height = field, lift = 0.06, sidewalk = 2.4,
    curb = 0.16, markings = true, plaza = (kind == "radial") and 16 or 0 }))

  -- Buildings: subdivide each block into lots, stand a building per worthy lot,
  -- turned to face its street (lot.yaw) and seated on the graded pad.
  for _, blk in ipairs(lay.blocks) do
    for _, lot in ipairs(city.lots(blk, { inset = 4.5, target_area = 420 })) do
      if lot.area > 130 then
        local r = math.sqrt((lot.cx - cx) ^ 2 + (lot.cz - cz) ^ 2)
        local floors
        if kind == "radial" then
          floors = (r < 55 and 9 or (r < 110 and 5 or 2))
        else
          floors = (r < 65 and 8 or (r < 130 and 4 or 2))
        end
        local w = math.max(8, math.min(lot.w - 2.5, 26))
        local d = math.max(8, math.min(lot.d - 2.5, 20))
        local b = building.grow{ floors = floors, width = w, depth = d,
          ground_retail = floors > 2, walkable_ground = true,
          seed = math.floor(lot.cx * 7.0 + lot.cz * 13.0) }
        local y = field:at(lot.cx, lot.cz)
        m:add_solid(mesh.place(b, { lot.cx, y, lot.cz }, lot.yaw))
      end
    end
  end
  return field
end

-- ---- the two cities ---------------------------------------------------------
local A = { x = -230, z = -30 }     -- radial (Étoile)
local B = { x =  220, z =  50 }     -- tensor (radial core -> grid rim)
local fA = build_city({ pattern = "radial", extent = 150, ring_spacing = 50,
  spokes = 7, ring_subdiv = 6, jitter = 0.03, seed = 5 }, A.x, A.z, "radial")
local fB = build_city({ pattern = "tensor", extent = 150, spacing = 64, step = 8,
  radial_decay = 95, seed = 9 }, B.x, B.z, "tensor")

-- ---- connecting highway A -> B (a gentle dog-leg) ---------------------------
local mid = { x = (A.x + B.x) / 2, z = (A.z + B.z) / 2 - 40 }
local hw = {
  nodes = { { x = A.x + 60, z = A.z }, { x = mid.x, z = mid.z }, { x = B.x - 60, z = B.z } },
  edges = { { a = 1, b = 2, width = 14 }, { a = 2, b = 3, width = 14 } },
  blocks = {},
}
local hwfield, hwregions = terrain.conform(land, hw, { margin = 4, falloff = 22 })
m:conform(hwregions)
m:add_solid(city.road_mesh(hw, { height = hwfield, lift = 0.05, sidewalk = 1.6,
  curb = 0.14, markings = true }))

-- ---- a simple L-system forest along the highway verges ----------------------
local trees = {}
local function add_tree(x, z, fld, seed)
  trees[#trees + 1] = { pos = { x, fld:at(x, z), z },
                        yaw = (seed % 7) * 0.9, scale = 0.9 + (seed % 5) * 0.12 }
end
for i = 0, 28 do
  local t = i / 28
  -- sample the highway polyline (two segments) and offset to either verge.
  local p0, p1
  if t < 0.5 then p0, p1, t = hw.nodes[1], hw.nodes[2], t * 2
  else p0, p1, t = hw.nodes[2], hw.nodes[3], (t - 0.5) * 2 end
  local x = p0.x + (p1.x - p0.x) * t
  local z = p0.z + (p1.z - p0.z) * t
  local dx, dz = p1.x - p0.x, p1.z - p0.z
  local len = math.max(1e-3, math.sqrt(dx * dx + dz * dz))
  local nx, nz = -dz / len, dx / len        -- verge normal
  if i % 2 == 0 then add_tree(x + nx * 13, z + nz * 13, hwfield, i + 1)
  else add_tree(x - nx * 13, z - nz * 13, hwfield, i + 3) end
end
-- a loose ring of trees around each city fringe
for _, c in ipairs({ { A, fA }, { B, fB } }) do
  for k = 0, 17 do
    local a = k / 18 * math.pi * 2
    local rr = 168 + (k % 3) * 9
    add_tree(c[1].x + math.cos(a) * rr, c[1].z + math.sin(a) * rr, c[2], k + 2)
  end
end
m:add_instances(tree_proto, trees, { collide = { shape = "capsule", radius = 0.5, height = 4 } })

return m
