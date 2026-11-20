-- SCATTER: where to put many copies of one piece on a surface (#191; one of the modelling tools Glenn asked for,
-- 2026-11-20): bolts round a flange, rivets along a seam, studs over a plate. A library:
-- `local scatter = require "scatter"`.
--
-- Each function returns a list of PLACEMENTS, { pos = {x,y,z}, up = {x,y,z} [, spin = degrees] }, which is what
-- `m:use(piece, placements)` and `n:use(piece, placements)` take: the piece is stored once and drawn at every
-- placement, its own +y standing along `up`. Build the piece standing on y = 0 (a bolt head: a short cylinder).
--
--   scatter.ring{ at = {x,y,z}, normal = {0,1,0}, radius = 0.2, count = 8 [, phase = degrees] }
--       evenly round a circle that lies flat on the surface
--   scatter.along{ path = { {x,y,z}, ... }, spacing = 0.1, normal = {0,1,0} [, ends = false] [, closed = false] }
--       evenly along a line of points (a seam, a rim); `ends = false` leaves the two ends empty
--   scatter.grid{ origin = {x,y,z}, u = {x,y,z}, v = {x,y,z}, count = { nu, nv } [, margin = 0.05] }
--       rows and columns over the flat patch origin + [0,1] u + [0,1] v, `margin` metres in from its edges
--   scatter.faces(P, sel)
--       one at the middle of each selected face of a quad mesh, standing along the face
--   scatter.jitter(list, { seed, move, spin, scale, drop })   the same list nudged, by a seed (see below)
--   scatter.join(a, b, ...)           several lists as one
local M = {}

local function add(a, b) return { a[1] + b[1], a[2] + b[2], a[3] + b[3] } end
local function sub(a, b) return { a[1] - b[1], a[2] - b[2], a[3] - b[3] } end
local function mul(a, k) return { a[1] * k, a[2] * k, a[3] * k } end
local function dot(a, b) return a[1] * b[1] + a[2] * b[2] + a[3] * b[3] end
local function cross(a, b) return { a[2] * b[3] - a[3] * b[2], a[3] * b[1] - a[1] * b[3], a[1] * b[2] - a[2] * b[1] } end
local function len(a) return math.sqrt(dot(a, a)) end
local function unit(a, what)
  local l = len(a)
  if l < 1e-12 then error("scatter: " .. (what or "a direction") .. " has no length", 3) end
  return mul(a, 1 / l)
end

function M.ring(o)
  if not (o and o.at and o.radius and o.count) then error("scatter.ring: give at, radius and count", 2) end
  local n = unit(o.normal or { 0, 1, 0 }, "normal")
  -- two directions flat on the surface
  local side = math.abs(n[2]) < 0.9 and { 0, 1, 0 } or { 1, 0, 0 }
  local a = unit(cross(side, n))
  local b = cross(n, a)
  local out, phase = {}, math.rad(o.phase or 0)
  for i = 0, o.count - 1 do
    local t = phase + i * 2 * math.pi / o.count
    out[#out + 1] = { pos = add(o.at, add(mul(a, o.radius * math.cos(t)), mul(b, o.radius * math.sin(t)))), up = n }
  end
  return out
end

function M.along(o)
  if not (o and o.path and #o.path >= 2 and o.spacing and o.spacing > 0) then error("scatter.along: give path (two points or more) and spacing", 2) end
  local n = unit(o.normal or { 0, 1, 0 }, "normal")
  local pts = {}
  for i, p in ipairs(o.path) do pts[i] = p end
  if o.closed then pts[#pts + 1] = pts[1] end
  local total, segs = 0, {}
  for i = 1, #pts - 1 do
    local l = len(sub(pts[i + 1], pts[i]))
    if l > 1e-9 then segs[#segs + 1] = { a = pts[i], b = pts[i + 1], len = l, from = total }; total = total + l end
  end
  if #segs == 0 then error("scatter.along: the path has no length", 2) end
  local gaps = math.max(1, math.floor(total / o.spacing + 0.5))
  local first, last = 0, gaps
  if o.closed then last = gaps - 1 elseif o.ends == false then first, last = 1, gaps - 1 end
  local out, s = {}, 1
  for i = first, last do
    local d = total * i / gaps
    while s < #segs and d > segs[s].from + segs[s].len do s = s + 1 end
    local g = segs[s]
    out[#out + 1] = { pos = add(g.a, mul(sub(g.b, g.a), math.min(1, (d - g.from) / g.len))), up = n }
  end
  return out
end

function M.grid(o)
  if not (o and o.origin and o.u and o.v and o.count) then error("scatter.grid: give origin, u, v and count = { nu, nv }", 2) end
  local n = unit(cross(o.u, o.v), "u x v")
  local lu, lv = len(o.u), len(o.v)
  local margin = o.margin or 0
  if 2 * margin >= lu or 2 * margin >= lv then error("scatter.grid: the margin leaves no room", 2) end
  local nu, nv = o.count[1], o.count[2]
  local out = {}
  for j = 1, nv do
    for i = 1, nu do
      local fu = nu == 1 and 0.5 or (margin + (lu - 2 * margin) * (i - 1) / (nu - 1)) / lu
      local fv = nv == 1 and 0.5 or (margin + (lv - 2 * margin) * (j - 1) / (nv - 1)) / lv
      out[#out + 1] = { pos = add(o.origin, add(mul(o.u, fu), mul(o.v, fv))), up = n }
    end
  end
  return out
end

function M.faces(P, sel)
  local want = {}
  for _, f in ipairs(sel) do want[f] = true end
  local out, k = {}, 0
  -- P:select walks every face in order with its middle and its normal: the selection is picked out by count
  local all = P:select{ all = true }
  local mids = {}
  P:select{ where = function(c, n) k = k + 1; mids[k] = { pos = { c[1], c[2], c[3] }, up = { n[1], n[2], n[3] } }; return false end }
  for i, f in ipairs(all) do
    if want[f] then out[#out + 1] = mids[i] end
  end
  return out
end

-- scatter.jitter(list, { seed = 1, move = 0.01, spin = 180, scale = 0.1, drop = 0 }) -> a new list, each placement
-- nudged: up to `move` metres sideways (flat on its surface), turned up to `spin` degrees about its `up`, grown or
-- shrunk by up to `scale` (0.1: 90% to 110%), and left out with a chance of `drop` (0 to 1). The same seed gives the
-- same result: weeds between paving stones, rivets set by hand, studs with their heads at any angle.
function M.jitter(list, o)
  o = o or {}
  local r = rng.new(o.seed or 1)
  local move, spin, scale, drop = o.move or 0, o.spin or 0, o.scale or 0, o.drop or 0
  local out = {}
  for _, p in ipairs(list) do
    -- (every placement draws the same count of numbers, kept or not, so dropping one does not move the others)
    local gone = r:next() < drop
    local a, b, turn, size = r:range(-1, 1), r:range(-1, 1), r:range(-1, 1), r:range(-1, 1)
    if not gone then
      local n = unit(p.up or { 0, 1, 0 }, "up")
      local side = math.abs(n[2]) < 0.9 and { 0, 1, 0 } or { 1, 0, 0 }
      local u = unit(cross(side, n))
      local v = cross(n, u)
      out[#out + 1] = { pos = add(p.pos, add(mul(u, a * move), mul(v, b * move))), up = n,
                        spin = (p.spin or 0) + turn * spin, scale = (p.scale or 1) * (1 + size * scale) }
    end
  end
  return out
end

function M.join(...)
  local out = {}
  for _, list in ipairs{ ... } do
    for _, p in ipairs(list) do out[#out + 1] = p end
  end
  return out
end

return M
