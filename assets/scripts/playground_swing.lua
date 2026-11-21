-- A PLAYGROUND SWING: a top bar on two A-shaped ends, with one to four seats hung from it on chains (#191; one of
-- the playground pieces, playground_kit.lua). Each seat is on a node of its own, `seat_1` ..., that swings about
-- the top bar and stops 65 degrees either way; nothing swings it here. A prefab with the script
-- playground_swing_sway.lua does (prefabs.lua `playground_swing_in_use`).
--
--   the frame    pipes (kit.pipe), a brace across each end, a foot plate under each leg
--   the chains   two twisted strands each (cable.rope), a plain wire from 40 m
--   the seats    a rubber belt with a seat marker, and a worn mat on the ground under each
--
-- It stands on y = 0 at its own origin, the top bar along x; the seats swing along z.
local kit = require "playground_kit"
local cable = require "cable"

local a = params{
  seats  = { 2, min = 1, max = 4, step = 1 },     -- how many seats hang from the bar
  height = { 2.4, min = 1.8, max = 3.2 },         -- the top bar's height (metres)
  paint  = kit.paints,                            -- the frame's colour
}
local seats = math.floor(a.seats + 0.5)
local H, color = a.height, kit.paint[a.paint]

local m = model.new()
kit.materials(m)
local far = m:level(40)

local BAY, R = 0.85, 0.042                -- the width given to a seat, and the frame pipe's radius
local half = seats * BAY / 2 + 0.2        -- from the middle to each A
local spread = H * 0.46                   -- how far each foot stands from under the bar

-- THE FRAME
local function frame(sides, level)
  local parts = { kit.pipe({ { -half - 0.12, H, 0 }, { half + 0.12, H, 0 } }, R * 1.1, color, { sides = sides }) }
  for _, x in ipairs{ -half, half } do
    for _, z in ipairs{ -spread, spread } do
      parts[#parts + 1] = kit.pipe({ { x, H, 0 }, { x, -0.02, z } }, R, color, { sides = sides })
    end
    if not level then
      local t = 0.58                       -- how far down the legs the brace is
      parts[#parts + 1] = kit.pipe({ { x, H * (1 - t), -spread * t }, { x, H * (1 - t), spread * t } }, R * 0.7, color)
    end
  end
  m:add(mesh.merge(parts), "paint", level and { level = level } or nil)
end
frame(12)
frame(4, far)
local feet = {}
for _, x in ipairs{ -half, half } do
  for _, z in ipairs{ -spread, spread } do
    feet[#feet + 1] = kit.foot({ x, 0, z }, 0.1, kit.zinc)
    local lean = math.deg(math.atan(spread, H))
    m:shape(string.format("leg_%s_%s", x < 0 and "l" or "r", z < 0 and "back" or "front"),
            { capsule = { radius = R, height = math.sqrt(H * H + spread * spread) }, pos = { x, H / 2, z / 2 },
              turn = { z > 0 and -lean or lean, 0, 0 } })
  end
end
m:add(mesh.merge(feet), "steel")
m:shape("top_bar", { capsule = { radius = R * 1.1, height = 2 * half }, pos = { 0, H, 0 }, turn = { 0, 0, 90 } })

-- THE SEATS: each hangs from the bar on two chains and swings about it
local SEAT_Y = 0.45                       -- a seat's height above the ground at rest
local drop = H - SEAT_Y                   -- from the bar down to the seat
local W, D, T = 0.46, 0.17, 0.035         -- the belt
local mats = {}
for i = 1, seats do
  local x = (i - (seats + 1) / 2) * BAY
  local seat = m:node("seat_" .. i, { pos = { x, H, 0 }, motion = { turn = { 1, 0, 0 }, min = -65, max = 65 } })
  local chains, plain, hangers = {}, {}, {}
  for _, side in ipairs{ -1, 1 } do
    local cx = side * W / 2
    local line = { { cx, -R, 0 }, { cx, -drop + 0.2, 0 } }
    chains[#chains + 1] = cable.rope(line, { radius = 0.008, strands = 2, pitch = 0.08, sides = 4, color = kit.zinc })
    plain[#plain + 1] = cable.wire(line, { radius = 0.008, sides = 3, color = kit.zinc })
    -- the eye it hangs from, and the two short links down to the belt's corners
    hangers[#hangers + 1] = mesh.paint(mesh.translate(mesh.rotate_z(mesh.torus(R * 1.1 + 0.012, 0.007), math.pi / 2), { cx, 0, 0 }), kit.zinc)
    for _, z in ipairs{ -D / 2 + 0.02, D / 2 - 0.02 } do
      hangers[#hangers + 1] = cable.wire({ { cx, -drop + 0.2, 0 }, { cx * 0.96, -drop + T / 2, z } }, { radius = 0.005, sides = 4, color = kit.zinc })
    end
  end
  seat:add(mesh.merge(chains), "steel")
  seat:add(mesh.merge(hangers), "steel")
  seat:add(mesh.merge(plain), "steel", { level = far })
  seat:add(kit.pad({ W, T, D }, { 0, -drop, 0 }, 0.012, kit.black), "rubber")
  seat:add(kit.box({ W, T, D }, { 0, -drop, 0 }, kit.black), "rubber", { level = far })
  seat:shape("seat_" .. i, { box = { W, T + 0.02, D }, pos = { 0, -drop, 0 }, friction = 0.9 })
  seat:marker("seat_" .. i, { pos = { 0, -drop + T / 2, 0 }, role = "seat", size = 0.45 })
  mats[#mats + 1] = kit.pad({ BAY - 0.15, 0.03, 1.7 }, { x, 0.0, 0 }, 0.012, { 0.2, 0.12, 0.1 })
end
m:add(mesh.merge(mats), "rubber")

-- where the sound of the chains would come from, and where someone pushing would stand
m:marker("creak", { pos = { 0, H, 0 } })
m:marker("push", { pos = { 0, 0, -1.3 }, role = "stand" })
return m
