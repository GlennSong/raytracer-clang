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

  -- Rolling hills the city climbs — gentle enough that draped roads read clean,
  -- steep enough that the buildings step up the slopes, warped for organic contours.
  local hills = terrain.fbm{ seed = seed, freq = 0.012, amp = 18, octaves = 5 }
  local land  = terrain.warp(hills, terrain.fbm{ seed = 2, freq = 0.02, amp = 1 }, 28)

  -- Ground, roads and buildings are all rendered AND collided from the same
  -- triangles (AGENTS.md § Playable Scenes, "collidable by default"): what you
  -- see is exactly what you stand on, so footing follows every contour.
  local m = model.new()
  m:add_solid(terrain.mesh(land, { size = 460, resolution = 220, color = {0.33, 0.40, 0.26} }))

  local lay = city.layout{ extent = 170, cell_size = 64, jitter = 0.10, seed = seed }

  -- Roads: a connected surface with junction geometry (ribbons trimmed back to
  -- the curb corners, intersection pads filling the gaps), draped on the terrain.
  m:add_solid(city.road_mesh(lay, { height = land, lift = 0.5, color = {0.08, 0.08, 0.09},
    sidewalk = 2.4, curb = 0.16, markings = true, mark_width = 0.18, crosswalks = true }))

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
    m:add_solid(scope{ origin = { cx - hw, lo - 1.0, cz - hd },
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
    m:add_solid(mesh.translate(bld, { cx, baseY, cz }))
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
  m:add_instances(lamp, places,
    { collide = { shape = "capsule", radius = 0.3, height = 5 } })

  return m
end

return build()
