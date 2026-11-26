-- A PLAYGROUND SANDPIT: a square of sand inside a low timber kerb wide enough to sit on (#315; one of the
-- playground pieces, playground_kit.lua).
--
-- It stands on y = 0, its origin at the middle of its square.
local kit = require "playground_kit"

local a = params{
  size = { 3.6, min = 2.0, max = 6.0 },            -- the square's side, outside the kerb (metres)
  kerb = { 0.28, min = 0.2, max = 0.45 },          -- the kerb's height
}
local half, K, T = a.size / 2, a.kerb, 0.22
local SAND = { 0.76, 0.66, 0.46 }

local m = model.new()
kit.materials(m)
m:material("sand", material.new{ roughness = 1.0, metallic = 0.0 })
local far = m:level(45)

-- THE KERB: four timbers, their top edges rounded
local kerb = {}
for _, s in ipairs{ -1, 1 } do
  kerb[#kerb + 1] = kit.pad({ a.size, K, T }, { 0, K / 2, s * (half - T / 2) }, 0.02, kit.timber)
  kerb[#kerb + 1] = kit.pad({ T, K, a.size - 2 * T }, { s * (half - T / 2), K / 2, 0 }, 0.02, kit.timber)
  m:shape("kerb_z_" .. (s < 0 and "back" or "front"), { box = { a.size, K, T }, pos = { 0, K / 2, s * (half - T / 2) }, friction = 0.9 })
  m:shape("kerb_x_" .. (s < 0 and "left" or "right"), { box = { T, K, a.size - 2 * T }, pos = { s * (half - T / 2), K / 2, 0 }, friction = 0.9 })
end
m:add(mesh.merge(kerb), "wood")
-- THE SAND, a little below the kerb's top, and solid to stand in
local inner = a.size - 2 * T
m:add(kit.box({ inner, 0.02, inner }, { 0, K - 0.09, 0 }, SAND), "sand")
m:shape("sand", { box = { inner, K - 0.08, inner }, pos = { 0, (K - 0.08) / 2, 0 }, friction = 1.0 })
m:add(kit.box({ a.size, K, a.size }, { 0, K / 2, 0 }, SAND), "sand", { level = far })

-- two sit on the front kerb, their feet outside
m:marker("seat_1", { pos = { -0.6, K, half - T / 2 }, role = "seat", size = 0.36 })
m:marker("seat_2", { pos = { 0.6, K, half - T / 2 }, role = "seat", size = 0.36 })
return m
