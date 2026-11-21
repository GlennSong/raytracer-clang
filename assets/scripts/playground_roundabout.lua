-- A PLAYGROUND ROUNDABOUT: a round deck that spins on a post in the ground, with grab rails from its rim up and in
-- to the middle (#191; one of the playground pieces, playground_kit.lua). The deck, its post and its rails are on
-- a node, `deck`, that turns without end; nothing turns it here. A prefab with the script
-- playground_roundabout_spin.lua does (prefabs.lua `playground_roundabout_in_use`).
--
--   the deck     a disc turned from an outline (mesh.lathe), with rows of studs to grip (scatter.ring)
--   the rails    one bent pipe for each place to stand (kit.pipe), all meeting at the post
--
-- It stands on y = 0 at its own origin.
local kit = require "playground_kit"
local scatter = require "scatter"

local a = params{
  radius = { 1.25, min = 0.9, max = 1.8 },        -- the deck's radius (metres)
  rails  = { 4, min = 3, max = 8, step = 1 },     -- how many grab rails, and so how many places to stand
  paint  = { "red", "blue", "yellow", "green", "orange" },   -- the deck's colour
  rail   = { "yellow", "red", "blue", "green", "orange" },   -- the rails' colour
}
local R, rails = a.radius, math.floor(a.rails + 0.5)
local color, rail_color = kit.paint[a.paint], kit.paint[a.rail]

local m = model.new()
kit.materials(m)
local far = m:level(40)

local UNDER, TOP = 0.2, 0.29               -- the deck's underside and its top
local GRIP = 0.92                          -- the height of a rail's top

-- THE BEARING it turns on, fixed in the ground
m:add(mesh.paint(mesh.lathe{ profile = { { 0, -0.02 }, { 0.3, -0.02 }, { 0.3, 0.03 }, { 0.16, 0.08 }, { 0.16, UNDER }, { 0, UNDER } },
                             segments = 20, smooth = true }, kit.zinc), "steel")

-- THE DECK and everything on it, turning together
local deck = m:node("deck", { pos = { 0, 0, 0 }, motion = { turn = { 0, 1, 0 } } })
local function disc(segments)
  return mesh.lathe{ profile = { { 0, UNDER }, { R - 0.03, UNDER }, { R, UNDER + 0.02 }, { R, TOP - 0.015 }, { R - 0.015, TOP }, { 0, TOP } },
                     segments = segments, smooth = true }
end
deck:add(mesh.paint(disc(48), color), "paint")
deck:add(mesh.paint(disc(12), color), "paint", { level = far })
deck:shape("deck", { hull = disc(12), friction = 1.0 })

-- studs to grip, in two rows
local stud = model.new()
stud:add(mesh.paint(mesh.lathe{ profile = { { 0, 0 }, { 0.022, 0 }, { 0.016, 0.006 }, { 0, 0.006 } }, segments = 8, smooth = true }, kit.zinc))
deck:use(stud, scatter.join(
  scatter.ring{ at = { 0, TOP, 0 }, normal = { 0, 1, 0 }, radius = R * 0.82, count = math.floor(R * 0.82 * 2 * math.pi / 0.12) },
  scatter.ring{ at = { 0, TOP, 0 }, normal = { 0, 1, 0 }, radius = R * 0.55, count = math.floor(R * 0.55 * 2 * math.pi / 0.12) }))

-- the post in the middle, with a rounded cap
local POST = 0.055
deck:add(mesh.paint(mesh.lathe{ profile = { { 0, TOP }, { POST, TOP }, { POST, GRIP + 0.05 }, { POST * 0.7, GRIP + 0.09 }, { 0, GRIP + 0.1 } },
                                segments = 14, smooth = true }, rail_color), "paint")
deck:shape("post", { capsule = { radius = POST, height = GRIP - TOP }, pos = { 0, (GRIP + TOP) / 2, 0 } })

-- the rails: up from the rim and in to the post
local function rail(sides)
  return kit.pipe({ { R - 0.07, TOP - 0.02, 0 }, { R - 0.07, GRIP, 0 }, { POST * 0.5, GRIP, 0 } }, 0.021, rail_color, { bend = 0.16, sides = sides })
end
local near, low = {}, {}
for k = 1, rails do
  local turn = (k - 1) / rails * 2 * math.pi
  near[k], low[k] = mesh.rotate_y(rail(10), turn), mesh.rotate_y(rail(4), turn)
  -- (a turn about y takes +x toward -z)
  local c, s = math.cos(turn), -math.sin(turn)
  deck:shape("rail_" .. k, { capsule = { radius = 0.03, height = GRIP - TOP }, pos = { c * (R - 0.07), (GRIP + TOP) / 2, s * (R - 0.07) } })
  -- a place to stand between this rail and the next, facing out
  local mid = turn + math.pi / rails
  local mc, ms = math.cos(mid), -math.sin(mid)
  deck:marker("ride_" .. k, { pos = { mc * R * 0.62, TOP, ms * R * 0.62 }, turn = { 0, math.deg(mid) + 90, 0 }, role = "stand", size = 0.4 })
end
deck:add(mesh.merge(near), "paint")
deck:add(mesh.merge(low), "paint", { level = far })

-- where someone would stand to push it round, and where its rumble would come from
m:marker("push", { pos = { 0, 0, R + 0.5 }, turn = { 0, 180, 0 }, role = "stand" })
m:marker("rumble", { pos = { 0, UNDER, 0 } })
return m
