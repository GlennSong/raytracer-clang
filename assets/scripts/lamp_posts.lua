-- LAMP POSTS, each for a different kind of place (#191). Each has a marker `lamp` (role "lamp") where its light is,
-- pointing down; the prefab's line gives it a light of a kind in lights.lua that comes on at dusk.
--
--   lantern   a cast-iron post with a four-sided glass lantern: a park's path, an old street (3.6 m)
--   globe     a slim post with a white globe: a square, a campus walk (3.2 m)
--   bollard   a waist-high post that lights the path at its foot: a garden path, a doorstep (0.9 m)
--   twin      a tall mast with two arms: a car park, a wide road (8 m; markers `lamp` and `lamp_2`)
--   flood     a mast with a bank of four floodlights tipped down toward +z: a sports pitch (12 m)
--   wall      a lantern on a bracket, to hang on a wall at head height (its back to -z, its foot the bracket's)
--
-- Each stands on y = 0 at its own origin.
local kit = require "model_kit"
local V = kit.V

local a = params{
  kind   = { "lantern", "globe", "bollard", "twin", "flood", "wall" },
  height = { 0, min = 0, max = 20 },     -- 0: the kind's own
  metal  = { "iron", "steel", "brass", "bronze", "chrome", "aluminium" },
}
local OWN = { lantern = 3.6, globe = 3.2, bollard = 0.9, twin = 8.0, flood = 12.0, wall = 0.5 }
local H = a.height > 0 and a.height or OWN[a.kind]
local col = kit.metal[a.metal] or kit.metal.iron
local WARM, WHITE = V(1.0, 0.9, 0.7), V(0.97, 0.97, 0.95)
local m = model.new()
local far = m:level(60)
local c = kit.maker()
local function lamp(name, x, y, z) m:marker(name, { pos = { x, y, z }, turn = { 90, 0, 0 }, role = "lamp" }) end

-- a four-sided lantern whose foot is at (x, y, z): a cap, glass with a lit core, four glazing bars
local function lantern(x, y, z, s)
  c.box("metal", V(x, y + 0.02 * s, z), V(0.2 * s, 0.04 * s, 0.2 * s), 0.004, col, 1)
  c.box("light", V(x, y + 0.2 * s, z), V(0.1 * s, 0.26 * s, 0.1 * s), 0.02 * s, WARM, 1)
  c.box("glass", V(x, y + 0.2 * s, z), V(0.24 * s, 0.32 * s, 0.24 * s), 0.0, V(0.9, 0.92, 0.9), 1)
  for _, dx in ipairs{ -1, 1 } do
    for _, dz in ipairs{ -1, 1 } do
      c.rod("metal", V(x + dx * 0.12 * s, y + 0.04 * s, z + dz * 0.12 * s), V(x + dx * 0.12 * s, y + 0.36 * s, z + dz * 0.12 * s), 0.008 * s, col, 4)
    end
  end
  c.mesh("metal", mesh.translate(mesh.rotate_y(mesh.lathe{ profile = { { 0, 0 }, { 0.2 * s, 0 }, { 0.05 * s, 0.14 * s }, { 0, 0.2 * s } }, segments = 4 }, math.pi / 4), { x, y + 0.36 * s, z }), col)
end

if a.kind == "lantern" then
  c.turned("metal", V(0, 0, 0), { { 0, 0 }, { 0.16, 0 }, { 0.16, 0.08 }, { 0.1, 0.14 }, { 0.09, 0.5 }, { 0.055, 0.62 }, { 0.045, H - 0.5 }, { 0.07, H - 0.46 }, { 0.04, H - 0.4 }, { 0, H - 0.4 } }, col, nil, 10)
  lantern(0, H - 0.4, 0, 1)
  lamp("lamp", 0, H - 0.2, 0)
  m:shape("post", { box = { 0.2, H, 0.2 }, pos = { 0, H / 2, 0 } })
elseif a.kind == "globe" then
  c.turned("metal", V(0, 0, 0), { { 0, 0 }, { 0.12, 0 }, { 0.12, 0.05 }, { 0.045, 0.12 }, { 0.035, H - 0.36 }, { 0.07, H - 0.33 }, { 0, H - 0.33 } }, col, nil, 10)
  c.mesh("light", mesh.translate(kit.ball(0.19, 12), { 0, H - 0.17, 0 }), WHITE)
  lamp("lamp", 0, H - 0.17, 0)
  m:shape("post", { box = { 0.16, H, 0.16 }, pos = { 0, H / 2, 0 } })
elseif a.kind == "bollard" then
  c.rod("metal", V(0, 0, 0), V(0, H - 0.2, 0), 0.06, col, 10)
  c.rod("light", V(0, H - 0.2, 0), V(0, H - 0.06, 0), 0.05, WARM, 10)
  c.rod("metal", V(0, H - 0.06, 0), V(0, H, 0), 0.075, col, 10)
  lamp("lamp", 0, H - 0.13, 0)
  m:shape("post", { box = { 0.14, H, 0.14 }, pos = { 0, H / 2, 0 } })
elseif a.kind == "twin" then
  c.turned("metal", V(0, 0, 0), { { 0, 0 }, { 0.16, 0 }, { 0.16, 0.3 }, { 0.09, 0.4 }, { 0.06, H }, { 0, H } }, col, nil, 10)
  for i, s in ipairs{ -1, 1 } do
    c.rod("metal", V(0, H - 0.3, 0), V(s * 1.5, H - 0.05, 0), 0.035, col, 8)
    c.box("metal", V(s * 1.75, H - 0.04, 0), V(0.6, 0.09, 0.26), 0.03, col, 2)
    c.box("light", V(s * 1.75, H - 0.09, 0), V(0.46, 0.02, 0.18), 0.0, WHITE, 1)
    lamp(i == 1 and "lamp" or "lamp_2", s * 1.75, H - 0.1, 0)
  end
  m:shape("post", { box = { 0.3, H, 0.3 }, pos = { 0, H / 2, 0 } })
elseif a.kind == "flood" then
  c.turned("metal", V(0, 0, 0), { { 0, 0 }, { 0.22, 0 }, { 0.22, 0.4 }, { 0.13, 0.5 }, { 0.08, H }, { 0, H } }, col, nil, 10)
  c.box("metal", V(0, H - 0.25, 0), V(2.3, 0.08, 0.08), 0.01, col, 1)
  for i = 1, 4 do
    local x = -1.2 + i * 0.48
    c.tilted("metal", V(x, H - 0.05, 0.1), V(0.4, 0.3, 0.16), 0.02, math.rad(35), col)
    c.tilted("light", V(x, H - 0.1, 0.17), V(0.34, 0.24, 0.02), 0.0, math.rad(35), WHITE)
  end
  -- (one light for the bank, aimed out and down over the pitch)
  m:marker("lamp", { pos = { 0, H - 0.1, 0.2 }, turn = { 50, 0, 0 }, role = "lamp" })
  m:shape("post", { box = { 0.4, H, 0.4 }, pos = { 0, H / 2, 0 } })
else   -- wall
  c.box("metal", V(0, 0.2, 0.01), V(0.12, 0.3, 0.02), 0.004, col, 1)
  c.rod("metal", V(0, 0.3, 0.02), V(0, 0.42, 0.26), 0.012, col, 6)
  c.rod("metal", V(0, 0.42, 0.26), V(0, 0.36, 0.26), 0.01, col, 6)
  lantern(0, -0.02, 0.26, 0.9)
  lamp("lamp", 0, 0.16, 0.26)
end
kit.put(m, m, c)
if a.kind ~= "wall" then kit.far_box(m, m, "metal", { 0.12, H, 0.12 }, { 0, H / 2, 0 }, col, far) end
return m
