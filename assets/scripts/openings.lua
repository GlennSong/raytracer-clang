-- OPENINGS: a hole through a wall with a frame round it and a pane in it (#191; Glenn, 2026-11-20: "actual
-- windows"). A library over the quad mesh words (poly.*), for any recipe: `local openings = require "openings"`.
--
--   local parts = openings.window(wall, {
--     at = { 0, 1.5, 0.1 },      -- the middle of the opening, on the wall's face
--     normal = { 0, 0, 1 },      -- the way that face looks (out of the wall)
--     width = 1.2, height = 1.0,
--     thickness = 0.2,           -- the wall's thickness there
--     frame = 0.06,              -- how wide the frame's bars are (0: no frame)
--     frame_depth = 0.08,        -- how deep the frame is, through the wall (default: frame)
--     set_back = 0.05,           -- how far in from the face the frame's front sits
--     up = { 0, 1, 0 },          -- the opening's "up" (default y)
--     pane = 0.01,               -- the pane's thickness (0: no pane: a doorway, a vent)
--     bars = { 1, 1 },           -- glazing bars: how many upright, how many level (default none)
--   })
--   -> parts.frame, parts.pane: quad meshes (poly) in the wall's space, or nil when there is none
--
-- `wall` is a quad mesh (poly.box, a loft ...) and is CUT in place: it is sliced along the opening's four sides,
-- the faces inside them on the front and the back are taken away and the two rims joined (P:bridge), so the wall
-- stays one closed solid with a hole through it. Several openings may be cut in one wall, one call each.
--
-- openings.hole(wall, opts) cuts the hole only (the same options; no frame, no pane) and returns nothing.
local M = {}

local function add(a, b) return { a[1] + b[1], a[2] + b[2], a[3] + b[3] } end
local function mul(a, k) return { a[1] * k, a[2] * k, a[3] * k } end
local function dot(a, b) return a[1] * b[1] + a[2] * b[2] + a[3] * b[3] end
local function cross(a, b) return { a[2] * b[3] - a[3] * b[2], a[3] * b[1] - a[1] * b[3], a[1] * b[2] - a[2] * b[1] } end
local function unit(a)
  local l = math.sqrt(dot(a, a))
  if l < 1e-12 then error("openings: a direction has no length", 3) end
  return mul(a, 1 / l)
end

-- the opening's own axes: n out of the wall, u up, r to the right as seen from outside
local function axes(o)
  if not (o.at and o.normal and o.width and o.height and o.thickness) then
    error("openings: give at, normal, width, height and thickness", 3)
  end
  local n = unit(o.normal)
  local up = o.up or { 0, 1, 0 }
  local r = unit(cross(up, n))
  local u = cross(n, r)
  return n, u, r
end

function M.hole(wall, o)
  local n, u, r = axes(o)
  local hw, hh = o.width / 2, o.height / 2
  -- the four sides, each a plane right through the wall
  wall:slice(add(o.at, mul(r, -hw)), r)
  wall:slice(add(o.at, mul(r, hw)), r)
  wall:slice(add(o.at, mul(u, -hh)), u)
  wall:slice(add(o.at, mul(u, hh)), u)
  -- the faces inside them: on the front (at the face) and on the back (a thickness behind it)
  local function inside(depth, facing)
    return wall:select{ where = function(c, fn)
      local d = { c[1] - o.at[1], c[2] - o.at[2], c[3] - o.at[3] }
      return dot(fn, n) * facing > 0.9 and math.abs(dot(d, n) + depth) < 1e-4
        and math.abs(dot(d, r)) < hw and math.abs(dot(d, u)) < hh
    end }
  end
  local front, back = inside(0, 1), inside(o.thickness, -1)
  if #front == 0 or #back == 0 then
    error("openings: no wall face at `at` looking along `normal`, or none `thickness` behind it", 2)
  end
  wall:bridge(front, back)
end

-- a box in the opening's axes: `size` = { along r, along u, along n }, its middle at `mid`
local function boxAt(size, mid, n, u, r)
  local b = poly.box(size)
  b:map(function(x, y, z)
    local p = add(mid, add(add(mul(r, x), mul(u, y)), mul(n, z)))
    return p[1], p[2], p[3]
  end)
  return b
end

function M.window(wall, o)
  M.hole(wall, o)
  local n, u, r = axes(o)
  local bar = o.frame or 0.06
  local depth = o.frame_depth or (bar > 0 and bar or 0.06)
  local back = (o.set_back or 0) + depth / 2
  local mid = add(o.at, mul(n, -back))
  local out = {}
  if bar > 0 then
    -- the frame: a slab filling the opening with its middle taken out (a hole through it, as the wall's)
    local frame = boxAt({ o.width, o.height, depth }, mid, n, u, r)
    M.hole(frame, { at = add(mid, mul(n, depth / 2)), normal = n, up = u, width = o.width - 2 * bar,
                    height = o.height - 2 * bar, thickness = depth })
    -- glazing bars across the light
    local bars = o.bars or { 0, 0 }
    local thin = bar * 0.6
    for i = 1, bars[1] do
      local x = (i / (bars[1] + 1) - 0.5) * (o.width - 2 * bar)
      frame:append(boxAt({ thin, o.height - 2 * bar, depth * 0.7 }, add(mid, mul(r, x)), n, u, r))
    end
    for i = 1, bars[2] do
      local y = (i / (bars[2] + 1) - 0.5) * (o.height - 2 * bar)
      frame:append(boxAt({ o.width - 2 * bar, thin, depth * 0.7 }, add(mid, mul(u, y)), n, u, r))
    end
    out.frame = frame
  end
  local pane = o.pane == nil and 0.01 or o.pane
  if pane > 0 then
    out.pane = boxAt({ o.width - bar, o.height - bar, pane }, mid, n, u, r)
  end
  return out
end

return M
