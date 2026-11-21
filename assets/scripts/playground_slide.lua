-- A PLAYGROUND SLIDE: steps up to a railed platform, and a steel chute down from it that levels out near the
-- ground (#191; one of the playground pieces, playground_kit.lua). Nothing on it moves. Each step, the platform
-- and each length of the chute has a collision shape, so it can be climbed and stood on.
--
--   the chute    a U-shaped outline run along a curve (mesh.sweep along mesh.path.spline)
--   the frame    pipes (kit.pipe): four posts, a rail each side, a hoop over the chute's mouth, a hand rail up the steps
--
-- It stands on y = 0 at its own origin, which is under the platform; the steps come up from -z and the chute
-- runs down toward +z.
local kit = require "playground_kit"

local a = params{
  height = { 1.6, min = 1.0, max = 2.6 },         -- the platform's height (metres)
  paint  = { "green", "red", "blue", "yellow", "orange" },   -- the frame's colour
}
local H, color = a.height, kit.paint[a.paint]

local m = model.new()
kit.materials(m)
local far = m:level(50)

local HALF = 0.45                          -- half the platform's side
local RAIL = 0.85                          -- a rail's height above what it guards
local R = 0.03                             -- the frame pipe's radius

-- THE PLATFORM
m:add(kit.pad({ 2 * HALF, 0.05, 2 * HALF }, { 0, H - 0.025, 0 }, 0.012, kit.timber), "wood")
m:add(kit.box({ 2 * HALF, 0.05, 2 * HALF }, { 0, H - 0.025, 0 }, kit.timber), "wood", { level = far })
m:shape("platform", { box = { 2 * HALF, 0.05, 2 * HALF }, pos = { 0, H - 0.025, 0 }, friction = 0.9 })

-- THE STEPS: as many as keep each rise near 0.2 m; the platform is the last
local count = math.max(2, math.ceil(H / 0.2))
local rise, run = H / count, 0.25
local foot_z = -HALF - (count - 1) * run   -- where the lowest step begins
local treads = {}
for i = 1, count - 1 do
  local z = -HALF - (count - i) * run + run / 2
  treads[#treads + 1] = kit.pad({ 0.7, 0.04, run - 0.02 }, { 0, i * rise - 0.02, z }, 0.01, kit.timber)
  -- solid down to the step below, so a foot cannot go between them
  m:shape("step_" .. i, { box = { 0.7, rise, run }, pos = { 0, i * rise - rise / 2, z }, friction = 0.9 })
end
m:add(mesh.merge(treads), "wood")

-- THE FRAME
local function frame(sides, level)
  local parts = {}
  local function pipe(points, r, bend) parts[#parts + 1] = kit.pipe(points, r or R, color, { sides = sides, bend = bend }) end
  for _, x in ipairs{ -HALF, HALF } do
    pipe({ { x, -0.02, -HALF }, { x, H + RAIL, -HALF } })                              -- the back posts
    pipe({ { x, -0.02, HALF }, { x, H + RAIL, HALF } })                                -- the front posts
    pipe({ { x, H + RAIL, -HALF }, { x, H + RAIL, HALF } })                            -- the side rail
    pipe({ { x, 0.0, foot_z }, { x * 0.82, H - 0.05, -HALF } }, R * 0.9)               -- what the steps sit on
    -- the hand rail up the steps: a post at the foot, then up to the back post
    pipe({ { x, -0.02, foot_z }, { x, RAIL, foot_z }, { x, H + RAIL - 0.02, -HALF } }, R * 0.8, 0.1)
    if not level then
      for k = 1, 3 do                                                                  -- bars under the side rail
        local z = -HALF + k * 2 * HALF / 4
        pipe({ { x, H, z }, { x, H + RAIL, z } }, R * 0.5)
      end
    end
  end
  -- the hoop over the chute's mouth, to hold while sitting down
  pipe({ { -HALF, H + RAIL, HALF }, { -HALF, H + RAIL + 0.35, HALF }, { HALF, H + RAIL + 0.35, HALF }, { HALF, H + RAIL, HALF } }, R * 0.8, 0.18)
  m:add(mesh.merge(parts), "paint", level and { level = level } or nil)
end
frame(10)
frame(4, far)
local feet = {}
for _, x in ipairs{ -HALF, HALF } do
  for _, z in ipairs{ -HALF, HALF, foot_z } do feet[#feet + 1] = kit.foot({ x, 0, z }, 0.07, kit.zinc) end
  m:shape("rail_" .. (x < 0 and "left" or "right"), { box = { 0.05, RAIL, 2 * HALF }, pos = { x, H + RAIL / 2, 0 } })
end
m:add(mesh.merge(feet), "steel")

-- THE CHUTE: level for a hand's width at the top, then down at about 35 degrees, then level again at knee height
local END_Y = 0.32
local slope = (H - END_Y) / math.tan(math.rad(35))
local line = {
  { 0, H, HALF - 0.05 }, { 0, H - 0.02, HALF + 0.2 },
  { 0, H - 0.02 - (H - END_Y) * 0.25, HALF + 0.25 + slope * 0.25 },
  { 0, END_Y + (H - END_Y) * 0.22, HALF + 0.25 + slope * 0.78 },
  { 0, END_Y + 0.02, HALF + 0.3 + slope }, { 0, END_Y, HALF + 0.75 + slope }, { 0, END_Y, HALF + 1.15 + slope },
}
local WIDE, SIDE, SHEET = 0.56, 0.17, 0.012    -- the bed's width, the sides' height, the steel's thickness
local outline = {
  { -WIDE / 2, SIDE }, { -WIDE / 2, 0 }, { WIDE / 2, 0 }, { WIDE / 2, SIDE },
  { WIDE / 2 - SHEET, SIDE }, { WIDE / 2 - SHEET, SHEET }, { -WIDE / 2 + SHEET, SHEET }, { -WIDE / 2 + SHEET, SIDE },
}
local function chute(count_)
  return mesh.sweep(outline, mesh.path.spline(line, { count = count_ }), { color = { 0.8, 0.81, 0.83 }, crease = 40 })
end
m:add(chute(28), "steel")
m:add(chute(7), "steel", { level = far })

-- its collision shapes: a flat length and two sides for each stretch of the curve
local bed = mesh.path.spline(line, { count = 9 })
for i = 1, #bed - 1 do
  local p, q = bed[i], bed[i + 1]
  local dy, dz = q[2] - p[2], q[3] - p[3]
  local length, pitch = math.sqrt(dy * dy + dz * dz), -math.deg(math.atan(dy, dz))
  local mid = { 0, (p[2] + q[2]) / 2, (p[3] + q[3]) / 2 }
  m:shape("chute_" .. i, { box = { WIDE, SHEET * 2, length + 0.02 }, pos = mid, turn = { pitch, 0, 0 }, friction = 0.08 })
  for _, side in ipairs{ -1, 1 } do
    m:shape(string.format("chute_%d_%s", i, side < 0 and "left" or "right"),
            { box = { SHEET * 2, SIDE, length + 0.02 }, pos = { side * WIDE / 2, mid[2] + SIDE / 2, mid[3] }, turn = { pitch, 0, 0 } })
  end
end

-- the legs under the chute: a pair half way down and a pair where it levels out
local legs = {}
for _, t in ipairs{ 0.5, 0.86 } do
  local p = bed[math.max(1, math.floor(t * #bed + 0.5))]
  for _, x in ipairs{ -WIDE / 2 + 0.03, WIDE / 2 - 0.03 } do
    legs[#legs + 1] = kit.pipe({ { x, p[2] + 0.01, p[3] }, { x, -0.02, p[3] } }, R * 0.8, color, { sides = 8 })
    legs[#legs + 1] = kit.foot({ x, 0, p[3] }, 0.06, kit.zinc)
  end
end
m:add(mesh.merge(legs), "paint")

-- where a climber starts, stands at the top, sits down to slide, and gets off
local last = line[#line]
m:marker("climb", { pos = { 0, 0, foot_z - 0.4 }, role = "stand" })
m:marker("top", { pos = { 0, H, 0 }, role = "stand" })
m:marker("slide_start", { pos = { 0, H, HALF + 0.15 }, role = "seat" })
m:marker("slide_end", { pos = { 0, END_Y + SHEET, last[3] - 0.25 }, role = "seat" })
m:marker("off", { pos = { 0, 0, last[3] + 0.5 }, role = "stand" })
return m
