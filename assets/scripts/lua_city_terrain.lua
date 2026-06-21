-- lua_city_terrain.lua — a city draped on composed terrain (ADR-0042/0043). The
-- terrain is a heightfield (gentle warped fbm hills); the recipe samples it with
-- :at(x,z) to seat each building on the ground with a foundation that follows the
-- slope, and to lay roads/lamps at terrain height. One model, one pipeline.

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

local function build()
  local seed = 7

  -- Rolling hills the city climbs — a few hills across the footprint (higher
  -- frequency) and real amplitude, warped for organic contours.
  local hills = terrain.fbm{ seed = seed, freq = 0.014, amp = 30, octaves = 5 }
  local land  = terrain.warp(hills, terrain.fbm{ seed = 2, freq = 0.02, amp = 1 }, 30)

  local m = model.new()
  m:add(terrain.mesh(land, { size = 460, resolution = 220, color = {0.33, 0.40, 0.26} }))

  local lay = city.layout{ extent = 170, cell_size = 64, jitter = 0.10, seed = seed }

  -- Roads: an asphalt quad per edge, draped at each endpoint's terrain height.
  local roads = {}
  for _, e in ipairs(lay.edges) do
    local a, b = lay.nodes[e.a], lay.nodes[e.b]
    local dx, dz = b.x - a.x, b.z - a.z
    local len = math.sqrt(dx * dx + dz * dz)
    if len > 1 then
      local nx, nz = -dz / len * (e.width * 0.5), dx / len * (e.width * 0.5)
      local ya, yb = land:at(a.x, a.z) + 0.15, land:at(b.x, b.z) + 0.15
      roads[#roads + 1] = mesh.quad(
        { a.x - nx, ya, a.z - nz }, { a.x + nx, ya, a.z + nz },
        { b.x + nx, yb, b.z + nz }, { b.x - nx, yb, b.z - nz },
        { 0, 1, 0 }, { 0.08, 0.08, 0.09 })
    end
  end
  if #roads > 0 then m:add(mesh.merge(roads)) end

  -- A building per block, seated on the ground: sample the footprint corners,
  -- floor the building on the HIGH corner and run a foundation plinth down to the
  -- LOW corner so it sits on the slope instead of floating (ADR-0044 stairstep).
  for _, block in ipairs(lay.blocks) do
    local cx, cz = centroid(block)
    local bw, bd = extent(block, cx, cz)
    local fw = math.max(8, bw * 0.55)
    local fd = math.max(8, bd * 0.55)
    local hw, hd = fw * 0.5, fd * 0.5
    local hi, lo = -1e9, 1e9
    for _, c in ipairs({ {cx-hw,cz-hd}, {cx+hw,cz-hd}, {cx+hw,cz+hd}, {cx-hw,cz+hd}, {cx,cz} }) do
      local y = land:at(c[1], c[2])
      hi = math.max(hi, y); lo = math.min(lo, y)
    end
    local baseY = hi
    -- Foundation plinth from below the low corner up to the building floor.
    m:add(scope{ origin = { cx - hw, lo - 1.0, cz - hd },
                 size = { fw, (baseY - lo) + 1.0, fd } }:box{0.32, 0.32, 0.34})

    local r = math.sqrt(cx * cx + cz * cz)
    local floors
    if r < 55 then floors = 9 + (math.floor(r) % 12)
    elseif r < 120 then floors = 4 + (math.floor(r) % 5)
    else floors = 2 + (math.floor(r) % 3) end

    local bld = building.grow{
      floors = floors, width = fw, depth = fd,
      ground_retail = true, walkable_ground = true,
      seed = math.floor(cx * 7 + cz) % 100000,
    }
    m:add(mesh.translate(bld, { cx, baseY, cz }))
  end

  -- Lamps along the verges, each seated at its terrain height.
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

  return m
end

return build()
