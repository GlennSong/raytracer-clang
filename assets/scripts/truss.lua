-- TRUSS: a lattice of members along a path (#191; one of the modelling tools Glenn asked for, 2026-11-20): a crane's
-- boom, a pylon, a mast, a bridge's girder, a scaffold. A library over the mesh words:
-- `local truss = require "truss"`.
--
--   local lattice, info = truss.build{
--     path = { {0,0,0}, {0,0,12} },   -- the centreline: two points or more (it may bend)
--     bay = 1.0,                      -- metres between frames along it (the path is divided evenly, near this)
--     section = "square",             -- "square", "triangle", or corners of your own: { {x,y}, ... } in -1..1,
--                                     --   x across and y up in the section, going round
--     size = { 1.2, 1.2 },            -- the section's width and height at the start
--     size_end = { 0.4, 0.4 },        -- ... and at the end (default: the same; it tapers evenly between)
--     chord = 0.05,                   -- radius of the long members that run the whole length, one per corner
--     brace = 0.03,                   -- radius of the frames and diagonals (0: chords only)
--     pattern = "zigzag",             -- the diagonals on each side: "zigzag", "cross" or "none"
--     frames = true,                  -- a ring of members round the section at every bay
--     sides = 6,                      -- how round a member is
--     up = { 0, 1, 0 },               -- the section's "up" (for a path that runs straight up, give another)
--     color = { 0.8, 0.6, 0.1 },
--   }
--   -> lattice: one mesh. info: { bays, chords, frames, diagonals, length, corners = { per frame: { points } } }
--
-- `info.corners` gives every joint, frame by frame, so a recipe can hang things on the lattice (a hook block at the
-- tip, bolts at the joints, a cab at the foot).
local M = {}

local function add(a, b) return { a[1] + b[1], a[2] + b[2], a[3] + b[3] } end
local function sub(a, b) return { a[1] - b[1], a[2] - b[2], a[3] - b[3] } end
local function mul(a, k) return { a[1] * k, a[2] * k, a[3] * k } end
local function dot(a, b) return a[1] * b[1] + a[2] * b[2] + a[3] * b[3] end
local function cross(a, b) return { a[2] * b[3] - a[3] * b[2], a[3] * b[1] - a[1] * b[3], a[1] * b[2] - a[2] * b[1] } end
local function len(a) return math.sqrt(dot(a, a)) end
local function unit(a) local l = len(a); return l > 1e-12 and mul(a, 1 / l) or { 0, 0, 0 } end

local SECTIONS = {
  square = { { -1, -1 }, { 1, -1 }, { 1, 1 }, { -1, 1 } },
  triangle = { { -1, -1 }, { 1, -1 }, { 0, 1 } },
}

-- the path cut into `n` equal lengths: n + 1 points and the direction at each
local function stations(path, bay)
  local segs, total = {}, 0
  for i = 1, #path - 1 do
    local l = len(sub(path[i + 1], path[i]))
    if l > 1e-9 then segs[#segs + 1] = { a = path[i], b = path[i + 1], len = l, from = total }; total = total + l end
  end
  if #segs == 0 then error("truss: the path needs two different points", 3) end
  local n = math.max(1, math.floor(total / bay + 0.5))
  local out, s = {}, 1
  for i = 0, n do
    local d = total * i / n
    while s < #segs and d > segs[s].from + segs[s].len do s = s + 1 end
    local g = segs[s]
    local t = math.min(1, math.max(0, (d - g.from) / g.len))
    out[#out + 1] = { at = add(g.a, mul(sub(g.b, g.a), t)), t = i / n }
  end
  -- the direction at a station: toward the next and from the last, averaged (a bend's frame sits across the bend)
  for i, st in ipairs(out) do
    local ahead = out[i + 1] and unit(sub(out[i + 1].at, st.at)) or { 0, 0, 0 }
    local behind = out[i - 1] and unit(sub(st.at, out[i - 1].at)) or { 0, 0, 0 }
    st.dir = unit(add(ahead, behind))
  end
  return out, total
end

function M.build(o)
  if type(o) ~= "table" or type(o.path) ~= "table" or #o.path < 2 then error("truss: give path = { {x,y,z}, {x,y,z}, ... }", 2) end
  local bay = o.bay or 1.0
  if bay <= 0 then error("truss: bay is more than 0", 2) end
  local section = type(o.section) == "table" and o.section or SECTIONS[o.section or "square"]
  if not section or #section < 2 then error("truss: section is \"square\", \"triangle\" or a list of corners", 2) end
  local size = o.size or { 1, 1 }
  local size_end = o.size_end or size
  local chord, brace = o.chord or 0.05, o.brace == nil and 0.03 or o.brace
  local pattern = o.pattern or "zigzag"
  if pattern ~= "zigzag" and pattern ~= "cross" and pattern ~= "none" then error("truss: pattern is zigzag, cross or none", 2) end
  local sides, color = o.sides or 6, o.color or { 0.8, 0.8, 0.8 }
  local up = unit(o.up or { 0, 1, 0 })

  local sts, total = stations(o.path, bay)
  -- every frame's corners. The section's axes are carried from frame to frame, so a bent path does not twist it.
  local corners, right = {}, nil
  for i, st in ipairs(sts) do
    local r = cross(up, st.dir)
    if len(r) < 1e-6 then
      if not right then error("truss: the path runs along `up`; give another up = { x, y, z }", 2) end
      r = right
    end
    r = unit(sub(r, mul(st.dir, dot(r, st.dir))))
    right = r
    local u = cross(st.dir, r)
    local w = (size[1] + (size_end[1] - size[1]) * st.t) / 2
    local h = (size[2] + (size_end[2] - size[2]) * st.t) / 2
    local ring = {}
    for k, c in ipairs(section) do ring[k] = add(st.at, add(mul(r, c[1] * w), mul(u, c[2] * h))) end
    corners[i] = ring
  end

  local parts, info = {}, { bays = #sts - 1, chords = 0, frames = 0, diagonals = 0, length = total, corners = corners }
  local function member(a, b, radius)
    parts[#parts + 1] = mesh.tube({ a, b }, { radius, radius }, sides, { color, color })
  end
  local n = #section
  -- the chords: one long member per corner, through every frame
  for k = 1, n do
    local pts, radii, cols = {}, {}, {}
    for i = 1, #corners do pts[i], radii[i], cols[i] = corners[i][k], chord, color end
    parts[#parts + 1] = mesh.tube(pts, radii, sides, cols)
    info.chords = info.chords + 1
  end
  if brace > 0 then
    -- a two-cornered section (a flat ladder) has one side, not two
    local faces = n == 2 and 1 or n
    if o.frames ~= false then
      for i = 1, #corners do
        for k = 1, faces do member(corners[i][k], corners[i][k % n + 1], brace); info.frames = info.frames + 1 end
      end
    end
    if pattern ~= "none" then
      for i = 1, #corners - 1 do
        for k = 1, faces do
          local k2 = k % n + 1
          local flip = (i + k) % 2 == 0
          if pattern == "cross" or not flip then member(corners[i][k], corners[i + 1][k2], brace); info.diagonals = info.diagonals + 1 end
          if pattern == "cross" or flip then member(corners[i][k2], corners[i + 1][k], brace); info.diagonals = info.diagonals + 1 end
        end
      end
    end
  end
  return mesh.merge(parts), info
end

return M
