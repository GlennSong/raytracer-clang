-- citylamps.lua — place a street lamp at every road-segment midpoint, offset to
-- the verge, across the whole generated street network (ADR-0042 Phase 3). Pure
-- Lua over the bound road solver (city.layout) + the composable model + the
-- instanced lamp prototype — a worked example of authoring a city part in script.

local function citylamps(opts)
  opts = opts or {}
  local layout = city.layout{
    extent    = opts.extent or 200,
    cell_size = opts.cell_size or 95,
    seed      = opts.seed or 5,
  }
  local lamp = streetfurniture.lamp(opts.lamp)

  local places = {}
  for _, e in ipairs(layout.edges) do
    local a, b = layout.nodes[e.a], layout.nodes[e.b]
    local dx, dz = b.x - a.x, b.z - a.z
    local len = math.sqrt(dx * dx + dz * dz)
    if len > 2 then
      local nx, nz = -dz / len, dx / len          -- edge normal (to the verge)
      local off = e.width * 0.5 + 1.3
      local mx, mz = (a.x + b.x) * 0.5, (a.z + b.z) * 0.5
      places[#places + 1] = { pos = { mx + nx * off, 0, mz + nz * off } }
    end
  end

  local m = model.new()
  m:add_instances(lamp, places)        -- one prototype, many placements (instanced)
  return m
end

return citylamps()
