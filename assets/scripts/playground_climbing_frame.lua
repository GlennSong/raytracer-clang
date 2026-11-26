-- A PLAYGROUND CLIMBING FRAME: a cube of steel bars to climb through and sit on top of (#315; one of the
-- playground pieces, playground_kit.lua). Posts on a square grid, rails between them at each level. Every rail of
-- the outside and the top has a collision shape, so it can be climbed and stood on; the inside is left open.
--
-- It stands on y = 0, its origin at the middle of its square.
local kit = require "playground_kit"

local a = params{
  bays   = { 3, min = 2, max = 4 },                -- bays a side
  bay    = { 1.2, min = 0.9, max = 1.5 },          -- a bay's width (metres)
  levels = { 3, min = 2, max = 4 },                -- rails' levels
  rise   = { 0.75, min = 0.6, max = 0.9 },         -- from one level to the next
  paint  = { "red", "blue", "yellow", "green", "orange" },   -- the posts' colour
}
local n, lv = math.floor(a.bays + 0.5), math.floor(a.levels + 0.5)
local half, top, color = n * a.bay / 2, lv * a.rise, kit.paint[a.paint]

local m = model.new()
kit.materials(m)
local far = m:level(45)

-- THE POSTS, painted, a foot plate under each
local posts = {}
for i = 0, n do
  for j = 0, n do
    local x, z = -half + i * a.bay, -half + j * a.bay
    posts[#posts + 1] = kit.pipe({ { x, 0, z }, { x, top + 0.04, z } }, 0.035, color, { sides = 8 })
    posts[#posts + 1] = kit.foot({ x, 0, z }, 0.07, color)
  end
end
m:add(mesh.merge(posts), "paint")

-- THE RAILS, zinc: at each level a grid of bars both ways
local rails = {}
for l = 1, lv do
  local y = l * a.rise
  for i = 0, n do
    local c = -half + i * a.bay
    rails[#rails + 1] = kit.pipe({ { c, y, -half }, { c, y, half } }, 0.022, kit.zinc, { sides = 6 })
    rails[#rails + 1] = kit.pipe({ { -half, y, c }, { half, y, c } }, 0.022, kit.zinc, { sides = 6 })
    -- solid: the outside's rails at every level, and every rail of the top
    if i == 0 or i == n or l == lv then
      m:shape(string.format("rail_x_%d_%d", l, i), { box = { 0.07, 0.07, 2 * half }, pos = { c, y, 0 }, friction = 0.9 })
      m:shape(string.format("rail_z_%d_%d", l, i), { box = { 2 * half, 0.07, 0.07 }, pos = { 0, y, c }, friction = 0.9 })
    end
  end
end
m:add(mesh.merge(rails), "steel")
for i = 0, n, n do
  for j = 0, n, n do
    m:shape(string.format("post_%d_%d", i, j), { box = { 0.09, top, 0.09 }, pos = { -half + i * a.bay, top / 2, -half + j * a.bay } })
  end
end
-- far off: its corner posts and its top
local distant = {}
for i = 0, n, n do
  for j = 0, n, n do distant[#distant + 1] = kit.box({ 0.07, top, 0.07 }, { -half + i * a.bay, top / 2, -half + j * a.bay }, color) end
end
distant[#distant + 1] = kit.box({ 2 * half, 0.05, 2 * half }, { 0, top, 0 }, kit.zinc)
m:add(mesh.merge(distant), "paint", { level = far })

-- where a climber starts, and where two sit on top
m:marker("climb", { pos = { 0, 0, half + 0.4 }, turn = { 0, 180, 0 }, role = "stand" })
m:marker("seat_1", { pos = { -a.bay / 2, top + 0.03, half }, role = "seat", size = 0.36 })
m:marker("seat_2", { pos = { a.bay / 2, top + 0.03, half }, role = "seat", size = 0.36 })
return m
