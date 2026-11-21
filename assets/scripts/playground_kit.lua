-- PLAYGROUND KIT: what the playground pieces share (#191; Glenn, 2026-11-20: the pieces are prefabs, and a
-- playground is the place they are set out in, not a prefab itself). A library: `local kit = require "playground_kit"`.
-- The pieces: playground_swing.lua, playground_seesaw.lua, playground_roundabout.lua, playground_slide.lua.
--
--   kit.materials(m)                 the materials every piece names: "paint" (painted steel), "steel" (bare, zinc
--                                    coated), "rubber", "wood", "plastic"
--   kit.paint[name]                  the paint colours a piece may be asked for (kit.paints: their names in order)
--   kit.bent(points, radius, steps)  the same line of points with each corner rounded to `radius` metres
--   kit.pipe(points, r, color, o)    a round pipe along a line of points: { bend = 0.12 } rounds its corners,
--                                    { sides = 10 }, { closed = true } for a hoop
--   kit.solid(P, color, angle)       a quad mesh as one painted mesh
--   kit.far(m, metres)               the model's far level, and kit.pipe's `sides` for it (4)
local M = {}

-- the paints are the palette's (furniture_models.lua, "THE PALETTE"): twelve, by name
require "furniture_models"
M.paints = furniture_kit.PAINT_NAMES
M.paint = furniture_kit.PAINT
M.zinc = { 0.66, 0.68, 0.7 }
M.black = { 0.06, 0.06, 0.065 }
M.timber = { 0.56, 0.38, 0.2 }

function M.materials(m)
  m:material("paint", material.new{ roughness = 0.42, metallic = 0.25 })
  m:material("steel", material.new{ roughness = 0.4, metallic = 0.9 })
  m:material("rubber", material.new{ roughness = 0.95, metallic = 0.0 })
  m:material("wood", material.new{ roughness = 0.75, metallic = 0.0 })
  m:material("plastic", material.new{ roughness = 0.3, metallic = 0.0 })
end

local function add(a, b) return { a[1] + b[1], a[2] + b[2], a[3] + b[3] } end
local function sub(a, b) return { a[1] - b[1], a[2] - b[2], a[3] - b[3] } end
local function mul(a, k) return { a[1] * k, a[2] * k, a[3] * k } end
local function dot(a, b) return a[1] * b[1] + a[2] * b[2] + a[3] * b[3] end
local function len(a) return math.sqrt(dot(a, a)) end

-- Each inside corner becomes an arc that touches both of its sides `radius` from the corner's own line (less where
-- a side is too short for it). The arc is drawn as a curve pulled toward the corner, which is close to round.
function M.bent(points, radius, steps)
  steps = steps or 6
  local out = { points[1] }
  for i = 2, #points - 1 do
    local p, a, b = points[i], points[i - 1], points[i + 1]
    local da, db = sub(a, p), sub(b, p)
    local la, lb = len(da), len(db)
    local cosine = dot(da, db) / (la * lb)
    if cosine < -0.9999 or la < 1e-6 or lb < 1e-6 then
      out[#out + 1] = p                                  -- no corner here: the line runs straight on
    else
      local half = math.acos(math.max(-1, math.min(1, cosine))) / 2
      local cut = math.min(radius / math.tan(half), la * 0.45, lb * 0.45)
      local from, to = add(p, mul(da, cut / la)), add(p, mul(db, cut / lb))
      for s = 0, steps do
        local t = s / steps
        local u = 1 - t
        out[#out + 1] = add(add(mul(from, u * u), mul(p, 2 * u * t)), mul(to, t * t))
      end
    end
  end
  out[#out + 1] = points[#points]
  return out
end

function M.pipe(points, r, color, o)
  o = o or {}
  local line = o.bend and M.bent(points, o.bend, o.steps) or points
  local path = {}
  for i, p in ipairs(line) do path[i] = p end
  path.closed = o.closed or false
  return mesh.sweep(mesh.profile.circle(r, o.sides or 10), path, { caps = not o.closed, color = color, crease = 50 })
end

function M.solid(P, color, angle)
  local parts = {}
  for _, part in pairs(P:to_parts(angle or 30)) do parts[#parts + 1] = part end
  return mesh.paint(mesh.merge(parts), color)
end

function M.box(size, at, color)
  return mesh.paint(mesh.translate(mesh.box(size), at), color)
end

-- a block with every edge rounded, where `at` is its middle
function M.pad(size, at, radius, color)
  return mesh.paint(mesh.translate(mesh.rounded_box(size, radius), at), color)
end

-- a foot plate where a pipe meets the ground: a low disc, a little into the ground so it never hovers
function M.foot(at, r, color)
  return mesh.paint(mesh.translate(mesh.lathe{ profile = { { 0, -0.02 }, { r, -0.02 }, { r, 0.012 }, { r * 0.8, 0.02 }, { 0, 0.02 } },
                                                segments = 12, smooth = true }, at), color)
end

return M
