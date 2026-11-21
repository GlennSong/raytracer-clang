-- A WALL CLOCK whose hands turn (#191): a round case and face, a mark for each hour (scatter.ring), and three
-- hands, each on a node that turns without end: `hour`, `minute`, `second`. As placed all three point at twelve.
-- A prefab's script sets them (wall_clock_run.lua). It HANGS on a wall: its origin is the middle of its back, and
-- its face looks along +z.
local kit = require "model_kit"
local scatter = require "scatter"
local V = kit.V

local a = params{
  size = { 0.3, min = 0.2, max = 1.2 },           -- across the case (metres)
  rim  = { "black", "white", "steel", "red", "walnut" },
}
local R = a.size / 2
local rim = ({ black = kit.BLACK * 1.5, white = kit.WHITE, steel = kit.STEEL, red = kit.paint.red, walnut = kit.WOOD[2] })[a.rim]

local m = model.new()
local far = m:level(30)
local DEEP = 0.04
local function facing(mesh_) return mesh.rotate_x(mesh_, math.pi / 2) end          -- a shape turned about y, laid to face +z

local c = kit.maker()
c.mesh("paint", facing(mesh.lathe{ profile = { { 0.0, 0.0 }, { R, 0.0 }, { R, DEEP }, { R - 0.012, DEEP + 0.006 }, { R - 0.02, DEEP }, { R - 0.02, 0.012 }, { 0.0, 0.012 } },
                                   segments = 36, smooth = true }), rim)
c.mesh("hard", facing(mesh.lathe{ profile = { { 0.0, 0.012 }, { R - 0.02, 0.012 }, { R - 0.02, 0.014 }, { 0.0, 0.014 } }, segments = 36 }), V(0.95, 0.95, 0.92))
kit.put(m, m, c)
kit.far_box(m, m, "paint", { 2 * R * 0.9, 2 * R * 0.9, DEEP }, { 0, 0, DEEP / 2 }, rim, far)
m:shape("case", { box = { 2 * R * 0.9, 2 * R * 0.9, DEEP }, pos = { 0, 0, DEEP / 2 } })

-- the hour marks: every third one longer
local function mark(len, wide)
  local p = model.new()
  p:add(mesh.paint(mesh.translate(mesh.box{ wide, 0.001, len }, { 0, 0.0005, 0 }), kit.BLACK))
  return p
end
local small, big = {}, {}
for i = 0, 11 do
  local t = 2 * math.pi * i / 12
  local at = { pos = { (R - 0.045) * math.sin(t), (R - 0.045) * math.cos(t), 0.014 }, up = { 0, 0, 1 }, spin = -math.deg(t) }
  if i % 3 == 0 then big[#big + 1] = at else small[#small + 1] = at end
end
m:use(mark(R * 0.14, R * 0.035), big)
m:use(mark(R * 0.08, R * 0.02), small)

-- the hands turn the way a clock's do, seen from in front
local function hand(name, len, wide, z, col, tail)
  local n = m:node(name, { pos = { 0, 0, z }, motion = { turn = { 0, 0, -1 } } })
  local k = kit.maker()
  k.box("hard", V(0, (len - tail) / 2, 0), V(wide, len + tail, 0.002), 0.0, col, 1)
  k.mesh("hard", facing(mesh.lathe{ profile = { { 0, -0.002 }, { wide * 1.2, -0.002 }, { wide * 1.2, 0.002 }, { 0, 0.002 } }, segments = 10 }), col)
  kit.put(m, n, k)
end
hand("hour", R * 0.5, R * 0.05, 0.018, kit.BLACK, R * 0.1)
hand("minute", R * 0.78, R * 0.035, 0.022, kit.BLACK, R * 0.1)
hand("second", R * 0.82, R * 0.012, 0.026, kit.paint.red, R * 0.2)
return m
