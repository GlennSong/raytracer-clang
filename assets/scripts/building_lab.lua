-- building_lab.lua — the BUILDING LAB recipe (building-grammar-plan.md, P1/P2).
--
-- One lot, one building, regenerated live: the level watches this file
-- ("watch_scripts": true) and reloads the scene whenever it changes. In play
-- mode R reseeds and T cycles the style (both rewrite building_lab.json's
-- opts, which also triggers the reload) — so recipe edits (Claude) and
-- on-screen review (device) iterate in seconds.
--
-- Knobs ride the level entity's `opts` (read here via the `args` global):
--   lot    = "rect" | "wide" | "L" | "wedge" | "flatiron"   (outline shown)
--   style  = "brick" | "concrete" | "stucco" | "metal" | "glass" | "painted"
--   floors, seed
--   window_head = "flat" | "segmental" | "round"   (style default otherwise)
--   window_hood = "none" | "band" | "arch"
--   lights_x, lights_y, arch_rise, frame_width, quoins, sill
--
-- NOTE: massing is still a RECTANGLE fitted in the lot bbox — the outline
-- exists to SHOW the gap the floorplan phase (P3) closes.

local opts   = args or {}
local lotKey = opts.lot or "rect"
local style  = opts.style or "brick"
local floors = opts.floors or 4
local seed   = opts.seed or 7

local LOTS = {
  rect     = { {-11, -9}, {11, -9}, {11, 9}, {-11, 9} },
  wide     = { {-16, -8}, {16, -8}, {16, 8}, {-16, 8} },
  L        = { {-12, -10}, {12, -10}, {12, 2}, {2, 2}, {2, 10}, {-12, 10} },
  wedge    = { {-12, -9}, {12, -5}, {12, 7}, {-12, 9} },
  flatiron = { {-12, -9}, {12, -2}, {12, 4}, {-8, 9} },
}
local lot = LOTS[lotKey] or LOTS.rect

local m = model.new()

-- Pedestal apron + the lot: grass pad and outline strips.
m:add(scope{ origin = {-26, -0.3, -22}, size = {52, 0.3, 44} }:box{0.52, 0.52, 0.50})
local lo, hi = {1e9, 1e9}, {-1e9, -1e9}
for _, p in ipairs(lot) do
  lo[1] = math.min(lo[1], p[1]); lo[2] = math.min(lo[2], p[2])
  hi[1] = math.max(hi[1], p[1]); hi[2] = math.max(hi[2], p[2])
end
m:add(scope{ origin = {lo[1], 0.0, lo[2]},
             size = {hi[1] - lo[1], 0.02, hi[2] - lo[2]} }:box{0.36, 0.46, 0.32})
for i = 1, #lot do
  local a, b = lot[i], lot[(i % #lot) + 1]
  local dx, dz = b[1] - a[1], b[2] - a[2]
  local len = math.sqrt(dx * dx + dz * dz)
  local strip = mesh.box({len, 0.08, 0.25})
  strip = mesh.rotate_y(strip, math.atan(dz, dx))
  strip = mesh.translate(strip, {(a[1] + b[1]) * 0.5, 0.05, (a[2] + b[2]) * 0.5})
  m:add(strip)
end

-- The building, as PARTS with REAL materials (lab feedback: "don't see
-- materials on the surface") — the same analytic PBR surfaces the city binds:
-- brick/concrete/stucco/metal walls, reflective glass, matte roof.
local MATS = {
  brick    = material.new{ surface = "brick",    roughness = 0.88 },
  concrete = material.new{ surface = "concrete", roughness = 0.92 },
  stucco   = material.new{ surface = "stucco",   roughness = 0.85 },
  metal    = material.new{ surface = "metal",    roughness = 0.45, metallic = 0.55 },
  glass    = material.new{ roughness = 0.08, metallic = 0.9 },
  roof     = material.new{ roughness = 0.85 },
}

local width = (hi[1] - lo[1]) - 2.4
local depth = (hi[2] - lo[2]) - 2.4
local grown = building.grow_parts{
  floors = floors, width = width, depth = depth,
  style = style, seed = seed,
  ground_retail = true, walkable_ground = true,
  window_head = opts.window_head, window_hood = opts.window_hood,
  lights_x = opts.lights_x, lights_y = opts.lights_y,
  arch_rise = opts.arch_rise, frame_width = opts.frame_width,
  quoins = opts.quoins, sill = opts.sill,
}
local cxz = {(lo[1] + hi[1]) * 0.5, 0, (lo[2] + hi[2]) * 0.5}
for _, entry in ipairs(grown) do
  local placed = mesh.translate(entry.mesh, cxz)
  m:add(placed, MATS[entry.part])   -- nil material = plain vertex colour
end

return m
