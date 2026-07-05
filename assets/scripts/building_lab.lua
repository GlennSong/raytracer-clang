-- building_lab.lua — the BUILDING LAB recipe (building-grammar-plan.md, P1).
--
-- One lot, one building, regenerated live: in play mode the level watches this
-- file ("watch_scripts": true) and reloads the scene whenever it changes, so a
-- recipe edit shows up on screen in seconds — the collaborative loop the
-- grammar work (elements → floorplans → curtain walls) is developed in.
--
-- Knobs ride the level entity's `opts` (read here via the `args` global), so
-- variants are one-line edits in building_lab.json — or just edit the DEFAULTS
-- below and save this file:
--   lot    = "rect" | "wide" | "L" | "wedge" | "flatiron"   (lot outline shown)
--   style  = "brick" | "concrete" | "stucco" | "metal" | "glass" | "painted"
--   floors = number of storeys        seed = variation
--
-- NOTE (current honest state): building.grow masses a RECTANGLE fitted inside
-- the lot — the drawn lot outline exists precisely to SHOW the gap the
-- floorplan phase (P3) closes. building.grow also returns one merged
-- vertex-coloured mesh; per-part PBR binding in the lab arrives with the
-- element-system slice (P2).

local opts   = args or {}
local lotKey = opts.lot or "rect"
local style  = opts.style or "brick"
local floors = opts.floors or 4
local seed   = opts.seed or 7

-- Lot presets (world XZ, closed CCW polygons, metres).
local LOTS = {
  rect     = { {-11, -9}, {11, -9}, {11, 9}, {-11, 9} },
  wide     = { {-16, -8}, {16, -8}, {16, 8}, {-16, 8} },
  L        = { {-12, -10}, {12, -10}, {12, 2}, {2, 2}, {2, 10}, {-12, 10} },
  wedge    = { {-12, -9}, {12, -5}, {12, 7}, {-12, 9} },
  flatiron = { {-12, -9}, {12, -2}, {12, 4}, {-8, 9} },
}
local lot = LOTS[lotKey] or LOTS.rect

local m = model.new()

-- Pedestal: a concrete apron the lot sits on.
m:add(scope{ origin = {-26, -0.3, -22}, size = {52, 0.3, 44} }:box{0.52, 0.52, 0.50})

-- The lot: a grass pad over the polygon's bounding box (cheap stand-in) and a
-- dark outline strip along every lot line, so fit/overhang is visible at a
-- glance while the massing is still rectangular.
local lo = {1e9, 1e9}
local hi = {-1e9, -1e9}
for _, p in ipairs(lot) do
  lo[1] = math.min(lo[1], p[1]); lo[2] = math.min(lo[2], p[2])
  hi[1] = math.max(hi[1], p[1]); hi[2] = math.max(hi[2], p[2])
end
m:add(scope{ origin = {lo[1], 0.0, lo[2]},
             size = {hi[1] - lo[1], 0.02, hi[2] - lo[2]} }:box{0.36, 0.46, 0.32})
for i = 1, #lot do
  local a = lot[i]
  local b = lot[(i % #lot) + 1]
  local dx, dz = b[1] - a[1], b[2] - a[2]
  local len = math.sqrt(dx * dx + dz * dz)
  local strip = mesh.box({len, 0.08, 0.25})
  strip = mesh.rotate_y(strip, math.atan(dz, dx))
  strip = mesh.translate(strip, {(a[1] + b[1]) * 0.5, 0.05, (a[2] + b[2]) * 0.5})
  m:add(strip)
end

-- The building: fitted to the lot's bounding box with a small setback.
local width = (hi[1] - lo[1]) - 2.4
local depth = (hi[2] - lo[2]) - 2.4
local b = building.grow{
  floors = floors, width = width, depth = depth,
  style = style, seed = seed,
  ground_retail = true, walkable_ground = true,
}
m:add(mesh.translate(b, {(lo[1] + hi[1]) * 0.5, 0, (lo[2] + hi[2]) * 0.5}))

return m
