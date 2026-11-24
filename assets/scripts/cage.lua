-- CAGE DEFORM: reshape a finished mesh by moving a few control points (#191; one of the modelling tools Glenn asked
-- for, 2026-11-20). A box of control points is put round the mesh; move some, and everything inside follows
-- smoothly, as if the mesh were set in a block of rubber. A library: `local cage = require "cage"`.
--
--   local c = cage.new{ lo = {x,y,z}, hi = {x,y,z}, counts = { 3, 2, 3 } }   -- control points along x, y, z (2 or more)
--   c:move(i, j, k, { dx, dy, dz })          -- move the control point i along x, j along y, k along z (from 1)
--   c:move_where(function(i, j, k, p) return { dx, dy, dz } end)   -- or all of them by a rule (nil: stay)
--   c:point(i, j, k)                         -- where a control point is now
--   local bent = c:mesh(m)                   -- a mesh reshaped (a new mesh; normals follow the shape)
--   c:poly(P)                                -- a quad mesh reshaped in place
--   c:at({x,y,z})                            -- where one point goes
--
-- With 2 control points along an axis the mesh follows straight between them (a shear, a taper); with 3 or more
-- it follows a smooth curve through the whole box, so one moved point bulges the surface without a kink. A point
-- outside the box is moved as the box's nearest face is. The control points pull the surface toward them: an
-- inner control point moved 10 cm moves the surface under it by less (the classic free-form deformation).
local M = {}
local Cage = {}
Cage.__index = Cage

function M.new(o)
  if not (o and o.lo and o.hi) then error("cage.new: give lo and hi", 2) end
  local counts = o.counts or { 2, 2, 2 }
  for a = 1, 3 do
    if counts[a] < 2 or counts[a] ~= math.floor(counts[a]) then error("cage.new: counts are whole numbers, 2 or more", 2) end
    if not (o.hi[a] > o.lo[a]) then error("cage.new: hi is past lo on every axis", 2) end
  end
  local c = setmetatable({ lo = o.lo, hi = o.hi, n = counts, d = {} }, Cage)
  -- binomial coefficients for each axis's blend
  c.binom = {}
  for a = 1, 3 do
    local deg, row = counts[a] - 1, { 1 }
    for k = 1, deg do row[k + 1] = row[k] * (deg - k + 1) / k end
    c.binom[a] = row
  end
  return c
end

local function key(c, i, j, k) return ((k - 1) * c.n[2] + (j - 1)) * c.n[1] + i end

local function check(c, i, j, k)
  if i < 1 or i > c.n[1] or j < 1 or j > c.n[2] or k < 1 or k > c.n[3] then
    error(string.format("cage: there is no control point %d, %d, %d (the cage is %d x %d x %d)", i, j, k, c.n[1], c.n[2], c.n[3]), 3)
  end
end

function Cage:move(i, j, k, delta)
  check(self, i, j, k)
  local d = self.d[key(self, i, j, k)] or { 0, 0, 0 }
  self.d[key(self, i, j, k)] = { d[1] + delta[1], d[2] + delta[2], d[3] + delta[3] }
  return self
end

function Cage:point(i, j, k)
  check(self, i, j, k)
  local d = self.d[key(self, i, j, k)] or { 0, 0, 0 }
  local f = { (i - 1) / (self.n[1] - 1), (j - 1) / (self.n[2] - 1), (k - 1) / (self.n[3] - 1) }
  return { self.lo[1] + (self.hi[1] - self.lo[1]) * f[1] + d[1], self.lo[2] + (self.hi[2] - self.lo[2]) * f[2] + d[2],
           self.lo[3] + (self.hi[3] - self.lo[3]) * f[3] + d[3] }
end

function Cage:move_where(rule)
  for k = 1, self.n[3] do
    for j = 1, self.n[2] do
      for i = 1, self.n[1] do
        local delta = rule(i, j, k, self:point(i, j, k))
        if delta then self:move(i, j, k, delta) end
      end
    end
  end
  return self
end

-- the blend weights along one axis at t in 0..1 (Bernstein: they add to 1)
local function weights(c, axis, t)
  local deg, w = c.n[axis] - 1, {}
  for k = 0, deg do w[k + 1] = c.binom[axis][k + 1] * t ^ k * (1 - t) ^ (deg - k) end
  return w
end

function Cage:at(p)
  local t = {}
  for a = 1, 3 do t[a] = math.min(1, math.max(0, (p[a] - self.lo[a]) / (self.hi[a] - self.lo[a]))) end
  local wx, wy, wz = weights(self, 1, t[1]), weights(self, 2, t[2]), weights(self, 3, t[3])
  local x, y, z = p[1], p[2], p[3]
  for idx, d in pairs(self.d) do
    local i = (idx - 1) % self.n[1] + 1
    local rest = (idx - i) // self.n[1]
    local j = rest % self.n[2] + 1
    local k = rest // self.n[2] + 1
    local w = wx[i] * wy[j] * wz[k]
    x, y, z = x + d[1] * w, y + d[2] * w, z + d[3] * w
  end
  return { x, y, z }
end

function Cage:mesh(m)
  return mesh.warp(m, function(p) return self:at(p) end)
end

function Cage:poly(P)
  P:map(function(x, y, z)
    local q = self:at({ x, y, z })
    return q[1], q[2], q[3]
  end)
  return P
end

return M
