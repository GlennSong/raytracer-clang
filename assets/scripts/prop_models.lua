-- PROP MODELS: things the toolkit already makes, each as a model fit to be stamped as a prefab (#191; Glenn,
-- 2026-11-20: "test it with various procgen/meshbuilder models"). One recipe, a `kind` in its arguments; each line
-- in models.lua that names this recipe is one model.
--
--   { kind = "furniture", name = "office_chair", variant = 0 }   a piece of the furniture kit (furniture_models.lua)
--   { kind = "street_prop", name = "hydrant" }                    a kerb prop of street_props.lua
--   { kind = "lamp" }                                             a street lamp: a marker where its light is
--   { kind = "tree", shape = "oak", seed = 3, height = 8 }       a stylized tree, a second level from 80 m
--   { kind = "rock", seed = 5, size = 1.6 }                       a boulder: its collision shape is its hull
--
-- Every model stands on y = 0 at its own origin and carries the collision shape a copy needs.
local a = args or {}
local m = model.new()

local builders = {}

-- THE FURNITURE KIT: a mesh per finish, each finish a named material; the piece's footprint is its collision box.
-- Piece space has z = 0 against a wall; the model is centred on its footprint so a prefab turns about its middle.
local FINISH = {
  furniture_wood    = material.new{ surface = "woodgrain", roughness = 0.55 },
  furniture_fabric  = material.new{ surface = "fabric", roughness = 0.95 },
  furniture         = material.new{ roughness = 0.55 },
  furniture_metal   = material.new{ roughness = 0.3, metallic = 0.9 },
  furniture_ceramic = material.new{ roughness = 0.15 },
  lit_band          = material.new{ roughness = 0.5, emission = { 1, 0.95, 0.85 } },
}
function builders.furniture()
  local piece = furniture.piece{ name = a.name or "office_chair", variant = a.variant or 0 }
  local w, h, d = piece.size[1], piece.size[2], piece.size[3]
  for _, e in ipairs(piece) do
    local finish = e.part
    if FINISH[finish] then m:material(finish, FINISH[finish]) end
    m:add(mesh.translate(e.mesh, { 0, 0, -d / 2 }), FINISH[finish] and finish or nil)
  end
  m:shape("footprint", { box = { w, h, d }, pos = { 0, h / 2, 0 } })
  m:marker("front", { pos = { 0, 0, d / 2 }, role = "point" })
end

-- A KERB PROP: its mesh and its box, as street_props.lua says them.
function builders.street_prop()
  local props = require "street_props"
  local want = a.name or "hydrant"
  for _, p in ipairs(props) do
    if p.name == want then
      m:add(p.mesh())
      local half = p.collider.half
      m:shape("body", { box = { half[1] * 2, half[2] * 2, half[3] * 2 }, pos = { 0, half[2], 0 } })
      return
    end
  end
  error("street_props.lua has no prop named '" .. want .. "'")
end

-- A STREET LAMP: the kit's lamp, a capsule round its pole, a marker where the light is.
function builders.lamp()
  local height = a.height or 6.0
  m:add(streetfurniture.lamp{ height = height })
  local r = 0.09
  m:shape("pole", { capsule = { radius = r, height = height - 2 * r }, pos = { 0, height / 2, 0 } })
  m:marker("lamp", { pos = { 0, height, 0 }, turn = { 90, 0, 0 }, role = "lamp" })
end

-- A TREE: bark and canopy; from 80 m a lighter tree of the same seed (fewer clumps). The trunk is what collides.
function builders.tree()
  local seed, shape, height = a.seed or 1, a.shape or "oak", a.height or 8
  m:material("leaves", material.new{ roughness = 0.9, two_sided = true })
  local far = m:level(80)
  local bark, canopy = stylized.tree(seed, { shape = shape, height = height })
  m:add(bark)
  m:add(canopy, "leaves")
  local bark2, canopy2 = stylized.tree(seed, { shape = shape, height = height, clumps = 5 })
  m:add(bark2, nil, { level = far })
  m:add(canopy2, "leaves", { level = far })
  local trunk = height * 0.4
  m:shape("trunk", { capsule = { radius = 0.2, height = trunk }, pos = { 0, trunk / 2 + 0.2, 0 } })
end

-- A BOULDER: the rock, and its hull to collide with (triangles while it stands still).
function builders.rock()
  local rock = stylized.rock(a.seed or 1, { family = a.family or "boulder", size = a.size or 1.5 })
  m:add(rock)
  m:shape("rock", { hull = rock })
end

local build = builders[a.kind or ""]
if not build then error("prop_models: kind is one of furniture, street_prop, lamp, tree, rock") end
build()
return m
