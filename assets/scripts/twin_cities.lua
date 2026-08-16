-- twin_cities.lua — two cities on the engine's CDLOD terrain, joined by a
-- highway (ADR-0044). The recipe FINDS where the cities go: it flood-fills the
-- injected `ground` for flat, low-slope valleys (terrain.flat_sites) and plants a
-- city on each, sized to the flat disc — never on the mountains. Each city grades
-- the terrain under its roads + lots (m:conform) and seats buildings on level
-- pads with foundations, so nothing floats; the highway is routed through low
-- ground between the two sites. One city is a radial Étoile, the other the new
-- tensor field (radial core -> grid rim).
--
-- Placement (lot fit, foundations, district variety) mirrors city.lua; the
-- sandbox has no require, so the essentials are ported here rather than shared.

local land = ground or terrain.flat(0)
local m = model.new()

-- ===== helpers ===============================================================
local function opt(o, k, v) if o[k] == nil then return v else return o[k] end end

local function centroid(poly)
  local x, z = 0, 0
  for _, p in ipairs(poly) do x = x + p.x; z = z + p.z end
  return x / #poly, z / #poly
end

-- A rotated rectangle footprint (4 corners), sign matching Mat4::rotateY so the
-- pad lines up with the mesh.place'd part.
local function rect(cx, cz, hw, hd, yaw)
  local c, s = math.cos(yaw), math.sin(yaw)
  local function pt(dx, dz)
    return { x = cx + dx * c + dz * s, z = cz - dx * s + dz * c }
  end
  return { pt(-hw, -hd), pt(hw, -hd), pt(hw, hd), pt(-hw, hd) }
end

-- Drop near-collinear vertices: the tensor generator samples curved roads into
-- many short segments, so its block faces carry 20-50 vertices. The parcel
-- inset/subdivide wants a simple convex-ish polygon, so collapse each gentle
-- curve to its corners (a few passes catches cascades) before subdividing.
local function simplify_poly(poly, tol)
  local cur = poly
  for _ = 1, 3 do
    local n = #cur
    if n <= 4 then break end
    local keep = {}
    for i = 1, n do
      local a, b, c = cur[(i - 2) % n + 1], cur[i], cur[i % n + 1]
      local ex, ez = c.x - a.x, c.z - a.z
      local len = math.sqrt(ex * ex + ez * ez)
      local d = (len < 1e-6) and 0
        or math.abs((b.x - a.x) * ez - (b.z - a.z) * ex) / len
      if d >= tol then keep[#keep + 1] = b end
    end
    if #keep < 4 then break end
    cur = keep
  end
  return cur
end

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

-- ----- district profiles (storeys + building variety by distance from centre) -
local function floors_at(r, kind)
  if kind == "radial" then
    if r < 55 then return 9 + math.floor(r) % 8
    elseif r < 110 then return 4 + math.floor(r) % 4 else return 2 + math.floor(r) % 2 end
  end
  if r < 65 then return 8 + math.floor(r) % 6
  elseif r < 130 then return 4 + math.floor(r) % 3 else return 2 + math.floor(r) % 2 end
end

local function building_args(r, cx, cz, kind)
  local h = math.floor(cx * 31 + cz * 17) % 997
  local a = { floors = floors_at(r, kind), ground_retail = true, walkable_ground = true,
              seed = math.floor(cx * 7 + cz) % 100000 }
  if r < 60 then
    a.style = ({ "glass", "glass", "metal", "concrete" })[(h % 4) + 1]
    a.bay_width = 3.6 + (h % 3) * 0.4
    if a.floors >= 12 then a.setback_floors = 5; a.setback_every = 2.5 end
    if h % 6 == 0 then a.shape = "cylinder"; a.sides = 40 end
    if h % 53 == 0 then a.shape = "pagoda"; a.tiers = 5 end
  elseif r < 125 then
    a.style = ({ "concrete", "stucco", "painted", "brick" })[(h % 4) + 1]
    a.bay_width = 3.2 + (h % 4) * 0.3
  else
    a.style = ({ "brick", "painted", "stucco" })[(h % 3) + 1]
    a.ground_retail = false
    a.bay_width = 3.0
  end
  return a
end

local function lot_style(r)
  if r < 60 then return { coverage = 0.85, front = 2.0, ground = { 0.56, 0.56, 0.54 } }
  elseif r < 125 then return { coverage = 0.70, front = 3.0, ground = { 0.26, 0.42, 0.18 } }
  else return { coverage = 0.55, front = 4.5, ground = { 0.28, 0.46, 0.20 } } end
end

-- ===== one city ==============================================================
-- Plan: classify each block by relief (skip the hilly ones), subdivide the flat
-- ones into lots, and emit a level pad per lot so terrain.conform cuts a plot.
local function plan_city(lay, cx, cz, kind)
  -- The site is already flat-ish (flat_sites picked it); the relief gate just
  -- skips the odd block that straddles a rise — the conform levels the rest.
  local flat_relief, min_lot = 16, 10
  local pads, plan = {}, {}
  for _, block0 in ipairs(lay.blocks) do
    local block = simplify_poly(block0, 2.5)
    local lo, hi = 1e9, -1e9
    for _, p in ipairs(block) do
      local y = land:at(p.x, p.z); lo = math.min(lo, y); hi = math.max(hi, y)
    end
    if (hi - lo) <= flat_relief then
      local bx, bz = centroid(block)
      local y = land:at(bx, bz)
      for _, lot in ipairs(city.lots(block, { inset = 7, target_area = 360,
                                              min_area = 120, seed = 7 })) do
        if lot.w >= min_lot and lot.d >= min_lot then
          pads[#pads + 1] = { y = y,
            poly = rect(lot.cx, lot.cz, lot.w * 0.5 + 0.5, lot.d * 0.5 + 0.5, lot.yaw) }
          plan[#plan + 1] = { cx = lot.cx, cz = lot.cz, w = lot.w, d = lot.d,
            yaw = lot.yaw, y = y, r = math.sqrt((lot.cx - cx) ^ 2 + (lot.cz - cz) ^ 2) }
        end
      end
    end
  end
  return pads, plan
end

-- Place: a level lot slab, then a building sized to fit (coverage + forecourt
-- setback toward the street), on a foundation so it meets the graded ground.
local function place_city(plan, kind)
  for _, b in ipairs(plan) do
    local sty = lot_style(b.r)
    local hw, hd = b.w * 0.5, b.d * 0.5
    m:add_solid(mesh.place(
      scope{ origin = { -hw, -0.05, -hd }, size = { b.w, 0.2, b.d } }:box(sty.ground),
      { b.cx, b.y, b.cz }, b.yaw))

    local bw, bd = b.w * sty.coverage, (b.d - sty.front) * sty.coverage
    if bw >= 8 and bd >= 8 then
      local zoff = b.d * 0.5 - sty.front - bd * 0.5
      local hbw, hbd = bw * 0.5, bd * 0.5
      m:add_solid(mesh.place(
        scope{ origin = { -hbw, -1.2, zoff - hbd }, size = { bw, 1.2, bd } }
          :box({ 0.32, 0.32, 0.34 }), { b.cx, b.y, b.cz }, b.yaw))
      local args = building_args(b.r, b.cx, b.cz, kind)
      args.width = bw; args.depth = bd
      m:add_solid(mesh.place(mesh.translate(building.grow(args), { 0, 0, zoff }),
        { b.cx, b.y, b.cz }, b.yaw))
    end
  end
end

-- Build a whole city centred at (cx,cz) with the given layout opts; returns the
-- conformed field (for draping the highway / trees nearby).
local function build_city(layout_opts, cx, cz, kind)
  local lay = translate_layout(city.layout(layout_opts), cx, cz)
  local pads, plan = plan_city(lay, cx, cz, kind)
  local field, regions = terrain.conform(land, lay,
    { margin = 3, falloff = 16, pads = pads, pad_falloff = 12 })
  m:conform(regions)
  -- City streets through the ONE road mesher (city.solid -> buildRoadNetLattice):
  -- the same swept lattice every level road uses, so this scene exercises shipping
  -- code. It builds carriageway bodies, junction pads and the curb/sidewalk band
  -- directly, and drapes on the conformed terrain via `height`.
  -- (Was city.roadbed, an SDF union the engine path had already retired.)
  m:add_solid(city.solid{ layout = lay, height = field, sidewalk = 2.4, curb = 0.16 })
  place_city(plan, kind)

  -- street lamps along the longer edges
  local lamp = streetfurniture.lamp{}
  local places = {}
  for _, e in ipairs(lay.edges) do
    local a, b2 = lay.nodes[e.a], lay.nodes[e.b]
    local dx, dz = b2.x - a.x, b2.z - a.z
    local len = math.sqrt(dx * dx + dz * dz)
    if len > 26 then
      local nx, nz = -dz / len, dx / len
      local mx, mz = (a.x + b2.x) * 0.5, (a.z + b2.z) * 0.5
      local off = e.width * 0.5 + 1.5
      places[#places + 1] = { pos = { mx + nx * off, field:at(mx + nx * off, mz + nz * off), mz + nz * off } }
      places[#places + 1] = { pos = { mx - nx * off, field:at(mx - nx * off, mz - nz * off), mz - nz * off } }
    end
  end
  m:add_instances(lamp, places, { collide = { shape = "capsule", radius = 0.3, height = 5 } })
  return field
end

-- ===== L-system tree (inlined; sandbox has no require) =======================
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
    blobs[#blobs + 1] = mesh.translate(mesh.sphere(0.85 + 0.5 * ((seed + #blobs) % 4) / 4), tips[i])
  end
  if #blobs == 0 then return branches end
  return mesh.merge({ branches,
    mesh.bake_height_color(mesh.merge(blobs), { 0.16, 0.33, 0.15 }, { 0.27, 0.47, 0.21 }) })
end

-- ===== site selection: the flattest valley NEAR each target =================
-- Search a box around each desired location and take its largest flat disc, so
-- the cities land predictably (the cameras/player know roughly where) but always
-- on buildable ground — shifted off a slope, never on the mountains.
local function find_site(tx, tz, fallback_r)
  local s = terrain.flat_sites(land, { region = 230, center = { x = tx, z = tz },
    cell = 10, max_slope = 0.10, max_height = 42, min_radius = 60, count = 1 })
  if s[1] then return s[1] end
  return { cx = tx, cz = tz, radius = fallback_r }
end
local A = find_site(-230, -10, 140)
local B = find_site(230, 30, 140)

-- Size each city to its flat disc (extent a touch inside the radius).
local exA = math.min(160, A.radius * 0.9)
local exB = math.min(160, B.radius * 0.9)
local fA = build_city({ pattern = "radial", extent = exA, ring_spacing = exA / 4.5,
  spokes = 8, ring_subdiv = 6, jitter = 0.03, seed = 5 }, A.cx, A.cz, "radial")
local fB = build_city({ pattern = "tensor", extent = exB, spacing = exB / 3.2, step = 8,
  radial_decay = exB * 0.6, seed = 9 }, B.cx, B.cz, "tensor")

-- ===== highway A -> B, routed to hold a walkable grade =======================
-- A road can't run straight up a mountain any more than the character can walk it.
-- Instead of a straight ramp that conforms into a cut/fill canyon over whatever
-- lies between, route a path over the NATURAL ground that never climbs steeper than
-- ~12% — so the highway bends around the peaks (switchbacking where it must) and
-- the conform only has to smooth a road that already follows gentle ground.
local function toward(from, to, dist)
  local ddx, ddz = to.cx - from.cx, to.cz - from.cz
  local l = math.max(1e-3, math.sqrt(ddx * ddx + ddz * ddz))
  return from.cx + ddx / l * dist, from.cz + ddz / l * dist
end
local ax, az = toward(A, B, A.radius * 0.92)   -- meet each city's ring road
local bx, bz = toward(B, A, B.radius * 0.92)
local hw = terrain.route(land, {
  from = { x = ax, z = az }, to = { x = bx, z = bz },
  max_grade = 0.12, cell = 14, width = 14, turn_penalty = 12, climb_cost = 0.6 })
local hwfield, hwregions = terrain.conform(land, hw, { margin = 4, falloff = 24 })
m:conform(hwregions)
m:add_solid(city.road_mesh(hw, { height = hwfield, lift = 0.05, sidewalk = 1.6,
  curb = 0.14, markings = true }))

-- ===== a simple L-system forest along the highway verges =====================
-- Walk the routed polyline (now any number of legs) and plant a tree on alternate
-- verges every ~16 m.
local tree_proto = make_tree(7)
local trees = {}
local ti = 0
for _, e in ipairs(hw.edges) do
  local p0, p1 = hw.nodes[e.a], hw.nodes[e.b]
  local ex, ez = p1.x - p0.x, p1.z - p0.z
  local l = math.max(1e-3, math.sqrt(ex * ex + ez * ez))
  local nx, nz = -ez / l, ex / l
  local n = math.max(1, math.floor(l / 16))
  for k = 0, n - 1 do
    local t = (k + 0.5) / n
    local x, z = p0.x + ex * t, p0.z + ez * t
    local s = (ti % 2 == 0) and 1 or -1
    local tx, tz = x + nx * 13 * s, z + nz * 13 * s
    trees[#trees + 1] = { pos = { tx, hwfield:at(tx, tz), tz },
                          yaw = (ti % 7) * 0.9, scale = 0.9 + (ti % 5) * 0.12 }
    ti = ti + 1
  end
end
m:add_instances(tree_proto, trees, { collide = { shape = "capsule", radius = 0.5, height = 4 } })

return m
