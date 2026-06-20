-- planter.lua — a street planter authored entirely from the scope op-vocabulary
-- (ADR-0042 Phase 2). Shows split / inset / box composing a prop in pure Lua over
-- the bound C++ grammar; the same mesh feeds the viewer and the path tracer.

local function planter(opts)
  opts = opts or {}
  local w = opts.width  or 1.6
  local h = opts.height or 0.6
  local d = opts.depth  or 1.6
  local stone = opts.stone or {0.55, 0.52, 0.48}
  local soil  = opts.soil  or {0.18, 0.13, 0.09}

  -- The stone body, then a soil slab inset from the rim and capping the top.
  local s    = scope{ origin = {-w / 2, 0, -d / 2}, size = {w, h, d} }
  local body = s:box(stone)
  local top  = s:inset(0.14):split('y', { h - 0.1, 0.1 })[2]   -- top 0.1 m
  return mesh.merge({ body, top:box(soil) })
end

-- Default recipe entry point.
return planter()
