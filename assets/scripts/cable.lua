-- CABLE AND ROPE: a line that hangs between two points, and a rope of twisted strands along any line (#191; one of
-- the modelling tools Glenn asked for, 2026-11-20). A library: `local cable = require "cable"`.
--
--   cable.hang(a, b, { sag = 0.5, steps = 16 })  -> points
--       the curve a cable takes hung from a to b: `sag` is how far its lowest point is below the straight line
--       between them, in metres (0: taut). The true hanging curve (a catenary) when the ends are level; between
--       ends at different heights the same curve is leant along the line between them.
--   cable.length(points)                          -> metres along a line of points
--   cable.wire(points, { radius = 0.01, sides = 6, color = {r,g,b} })                 -> mesh: one round wire
--   cable.rope(points, { radius = 0.02, strands = 3, pitch = 0.12, sides = 5, color }) -> mesh: twisted strands,
--       `radius` the whole rope's, `pitch` the length of one full twist
--
-- These are shapes that stand still. A rope that swings and carries a load is physics (#311).
local M = {}

local function add(a, b) return { a[1] + b[1], a[2] + b[2], a[3] + b[3] } end
local function sub(a, b) return { a[1] - b[1], a[2] - b[2], a[3] - b[3] } end
local function mul(a, k) return { a[1] * k, a[2] * k, a[3] * k } end
local function dot(a, b) return a[1] * b[1] + a[2] * b[2] + a[3] * b[3] end
local function cross(a, b) return { a[2] * b[3] - a[3] * b[2], a[3] * b[1] - a[1] * b[3], a[1] * b[2] - a[2] * b[1] } end
local function len(a) return math.sqrt(dot(a, a)) end
local function unit(a) local l = len(a); return l > 1e-12 and mul(a, 1 / l) or { 0, 0, 0 } end
local function cosh(x) return (math.exp(x) + math.exp(-x)) / 2 end

function M.length(points)
  local total = 0
  for i = 1, #points - 1 do total = total + len(sub(points[i + 1], points[i])) end
  return total
end

function M.hang(a, b, o)
  o = o or {}
  local sag, steps = o.sag or 0, o.steps or 16
  if steps < 1 then error("cable.hang: steps is 1 or more", 2) end
  local span = len(sub(b, a))
  if span < 1e-9 then error("cable.hang: the two ends are the same point", 2) end
  local out = {}
  if sag <= 0 then
    for i = 0, steps do out[#out + 1] = add(a, mul(sub(b, a), i / steps)) end
    return out
  end
  -- the hanging curve y = c (cosh(x / c) - 1) dips `sag` at the middle of a span when c (cosh(span / 2c) - 1) = sag:
  -- found by halving (the dip falls as c grows)
  local lo, hi = span * 1e-3, span * 1e4
  for _ = 1, 80 do
    local c = (lo + hi) / 2
    if c * (cosh(span / (2 * c)) - 1) > sag then lo = c else hi = c end
  end
  local c = (lo + hi) / 2
  for i = 0, steps do
    local t = i / steps
    local x = (t - 0.5) * span
    local dip = sag - c * (cosh(x / c) - 1)          -- 0 at the ends, `sag` in the middle
    local p = add(a, mul(sub(b, a), t))
    out[#out + 1] = { p[1], p[2] - dip, p[3] }
  end
  return out
end

function M.wire(points, o)
  o = o or {}
  if #points < 2 then error("cable.wire: give two points or more", 2) end
  local r, color = o.radius or 0.01, o.color or { 0.1, 0.1, 0.1 }
  local radii, cols = {}, {}
  for i = 1, #points do radii[i], cols[i] = r, color end
  return mesh.tube(points, radii, o.sides or 6, cols)
end

function M.rope(points, o)
  o = o or {}
  if #points < 2 then error("cable.rope: give two points or more", 2) end
  local radius, strands, pitch = o.radius or 0.02, o.strands or 3, o.pitch or 0.12
  if strands < 2 then return M.wire(points, o) end
  -- finer points along the line, so each strand turns smoothly: eight to a twist
  local fine, total = {}, M.length(points)
  local n = math.max(#points - 1, math.ceil(total / pitch * 8))
  local segs, run = {}, 0
  for i = 1, #points - 1 do
    local l = len(sub(points[i + 1], points[i]))
    if l > 1e-9 then segs[#segs + 1] = { a = points[i], b = points[i + 1], len = l, from = run }; run = run + l end
  end
  local s = 1
  for i = 0, n do
    local d = total * i / n
    while s < #segs and d > segs[s].from + segs[s].len do s = s + 1 end
    local g = segs[s]
    fine[#fine + 1] = { at = add(g.a, mul(sub(g.b, g.a), math.min(1, (d - g.from) / g.len))), d = d, dir = unit(sub(g.b, g.a)) }
  end
  -- a frame carried along the line, so the twist is the rope's and not the path's
  local side
  local parts = {}
  local strand_r = radius / (1 + 1 / math.sin(math.pi / strands))    -- strands that just touch, inside `radius`
  local orbit = radius - strand_r
  local lines = {}
  for k = 1, strands do lines[k] = {} end
  for _, f in ipairs(fine) do
    local ref = side or (math.abs(f.dir[2]) < 0.9 and { 0, 1, 0 } or { 1, 0, 0 })
    local u = unit(sub(ref, mul(f.dir, dot(ref, f.dir))))
    side = u
    local v = cross(f.dir, u)
    for k = 1, strands do
      local ang = 2 * math.pi * (f.d / pitch + (k - 1) / strands)
      lines[k][#lines[k] + 1] = add(f.at, add(mul(u, orbit * math.cos(ang)), mul(v, orbit * math.sin(ang))))
    end
  end
  for k = 1, strands do
    parts[k] = M.wire(lines[k], { radius = strand_r, sides = o.sides or 5, color = o.color or { 0.55, 0.45, 0.3 } })
  end
  return mesh.merge(parts)
end

return M
