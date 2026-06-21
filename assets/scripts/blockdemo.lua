-- blockdemo.lua — a small city block composed entirely in Lua (ADR-0042): a
-- ground slab, a row of buildings (building.grow), and instanced lamps. Returns a
-- composable model the engine bakes the same way for both renderers — parts to
-- geometry, the lamp group to a shared prototype placed by the path tracer's TLAS.

local m = model.new()

-- A ground slab, authored from a scope box.
m:add(scope{ origin = {-30, -0.2, -20}, size = {60, 0.2, 40} }:box{0.42, 0.44, 0.40})

-- A row of buildings of rising height along x (each a building.grow part).
for i = 0, 3 do
  local b = building.grow{
    floors = 3 + i, width = 10, depth = 10,
    ground_retail = true, walkable_ground = true, seed = i + 1,
  }
  m:add(mesh.translate(b, { -21 + i * 14, 0, -2 }))
end

-- Lamps along the front verge — one prototype, many placements (instanced).
local lamp = streetfurniture.lamp{}
local places = {}
for i = 0, 6 do places[#places + 1] = { pos = { -24 + i * 8, 0, 13 } } end
m:add_instances(lamp, places)

return m
