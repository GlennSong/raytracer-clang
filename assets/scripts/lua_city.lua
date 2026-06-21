-- lua_city.lua — a whole (small) city composed in Lua (ADR-0042/0043): the road
-- layout comes from the C++ solver (city.layout), then the recipe lays a building
-- per block, asphalt road strips, a ground plane, and instanced street lamps —
-- and returns one composable model the engine bakes for either renderer.

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
  local lay = city.layout{ extent = 170, cell_size = 64, jitter = 0.12, seed = seed }
  local m = model.new()

  -- Ground plane.
  m:add(scope{ origin = {-260, -0.3, -260}, size = {520, 0.3, 520} }:box{0.27, 0.30, 0.25})

  -- Roads: a dark asphalt quad per edge, merged into one part.
  local roads = {}
  for _, e in ipairs(lay.edges) do
    local a, b = lay.nodes[e.a], lay.nodes[e.b]
    local dx, dz = b.x - a.x, b.z - a.z
    local len = math.sqrt(dx * dx + dz * dz)
    if len > 1 then
      local nx, nz = -dz / len * (e.width * 0.5), dx / len * (e.width * 0.5)
      local y = 0.03
      roads[#roads + 1] = mesh.quad(
        { a.x - nx, y, a.z - nz }, { a.x + nx, y, a.z + nz },
        { b.x + nx, y, b.z + nz }, { b.x - nx, y, b.z - nz },
        { 0, 1, 0 }, { 0.08, 0.08, 0.09 })
    end
  end
  if #roads > 0 then m:add(mesh.merge(roads)) end

  -- A building per block: footprint inset from the streets, height by radial
  -- falloff from the centre (downtown is taller). building.grow's own facade
  -- colours read on a single material, so no texture needed here.
  for _, block in ipairs(lay.blocks) do
    local cx, cz = centroid(block)
    local bw, bd = extent(block, cx, cz)
    local fw = math.max(8, bw * 0.55)
    local fd = math.max(8, bd * 0.55)
    local r = math.sqrt(cx * cx + cz * cz)
    local floors
    if r < 55 then floors = 9 + (math.floor(r) % 12)
    elseif r < 120 then floors = 4 + (math.floor(r) % 5)
    else floors = 2 + (math.floor(r) % 3) end

    local b = building.grow{
      floors = floors, width = fw, depth = fd,
      ground_retail = true, walkable_ground = true,
      setback_floors = (floors >= 14) and 6 or 0,
      setback_every = 2.5,
      seed = math.floor(cx * 7 + cz) % 100000,
    }
    m:add(mesh.translate(b, { cx, 0, cz }))
  end

  -- Instanced street lamps along both verges of each road.
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
      places[#places + 1] = { pos = { mx + nx * off, 0, mz + nz * off } }
      places[#places + 1] = { pos = { mx - nx * off, 0, mz - nz * off } }
    end
  end
  m:add_instances(lamp, places)

  return m
end

return build()
