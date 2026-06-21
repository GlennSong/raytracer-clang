-- roads_terrain.lua — just the road network draped over terrain, no buildings
-- (ADR-0044 test view). Shows a generator's pattern hugging the hills: each road
-- is segmented and each segment seated at the terrain height under it.

local seed = 7
local hills = terrain.fbm{ seed = seed, freq = 0.012, amp = 24, octaves = 5 }
local land  = terrain.warp(hills, terrain.fbm{ seed = 2, freq = 0.02, amp = 1 }, 28)

local m = model.new()
m:add(terrain.mesh(land, { size = 520, resolution = 240, color = {0.36, 0.44, 0.28} }))

-- Swap pattern = "grid" | "radial" to compare generators.
local lay = city.layout{ pattern = "radial", extent = 220, ring_spacing = 55,
                         spokes = 14, jitter = 0.06, seed = seed }

-- Drape a road as terrain-following segments (sample height along its length).
local roads = {}
local function strip(a, b, w)
  local dx, dz = b.x - a.x, b.z - a.z
  local len = math.sqrt(dx * dx + dz * dz)
  if len < 1e-3 then return end
  local segs = math.max(1, math.floor(len / 8))
  local nx, nz = -dz / len * (w * 0.5), dx / len * (w * 0.5)
  for s = 0, segs - 1 do
    local t0, t1 = s / segs, (s + 1) / segs
    local x0, z0 = a.x + dx * t0, a.z + dz * t0
    local x1, z1 = a.x + dx * t1, a.z + dz * t1
    local y0, y1 = land:at(x0, z0) + 0.25, land:at(x1, z1) + 0.25
    roads[#roads + 1] = mesh.quad(
      { x0 - nx, y0, z0 - nz }, { x0 + nx, y0, z0 + nz },
      { x1 + nx, y1, z1 + nz }, { x1 - nx, y1, z1 - nz },
      { 0, 1, 0 }, { 0.07, 0.07, 0.08 })
  end
end
for _, e in ipairs(lay.edges) do
  strip(lay.nodes[e.a], lay.nodes[e.b], e.width)
end
m:add(mesh.merge(roads))

return m
