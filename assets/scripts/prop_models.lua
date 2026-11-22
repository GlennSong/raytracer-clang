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
-- What it takes (the model studio shows these as a form). Each kind reads the ones that are its own.
local a = params{
  kind    = { "furniture", "street_prop", "lamp", "tree", "rock" },
  name    = "",                                  -- furniture: a piece's name (office_chair, sofa ...); street_prop: a prop's
  variant = 0,                                   -- furniture: bits 0-2 the fabric, 3-4 the wood, the rest the piece's own dice
  shape   = { "oak", "round", "spreading", "columnar", "flowering", "pine", "palm", "maple", "birch", "shrub" },  -- tree
  seed    = 1,                                   -- tree, rock
  height  = { 8.0, min = 2, max = 40 },          -- tree, lamp (metres)
  size    = { 1.5, min = 0.2, max = 10 },        -- rock (metres)
  family  = { "boulder", "slab", "pebbles", "outcrop" },  -- rock
}
local m = model.new()

local builders = {}

-- THE FURNITURE KIT: a mesh per finish, each finish a named material; the piece's footprint is its collision box.
-- (The finishes are model_kit.lua's, so a kit piece and a model with moving parts name the same materials.)
-- Piece space has z = 0 against a wall; the model is centred on its footprint so a prefab turns about its middle.
local kit_finishes = require("model_kit").finishes
local FINISH = {}
for _, f in pairs(kit_finishes) do FINISH[f[1]] = material.new(f[2]) end
function builders.furniture()
  local piece = furniture.piece{ name = a.name ~= "" and a.name or "office_chair", variant = a.variant }
  local w, h, d = piece.size[1], piece.size[2], piece.size[3]
  for _, e in ipairs(piece) do
    local finish = e.part
    if FINISH[finish] then m:material(finish, FINISH[finish]) end
    m:add(mesh.translate(e.mesh, { 0, 0, -d / 2 }), FINISH[finish] and finish or nil)
  end
  m:shape("footprint", { box = { w, h, d }, pos = { 0, h / 2, 0 } })
  m:marker("front", { pos = { 0, 0, d / 2 }, role = "point" })
  -- WHERE A BODY USES IT: the furniture library's own description of the piece (furniture_library.lua: its verbs),
  -- each as a marker whose role is "seat" or "lie". A stamped prefab offers sitting or lying at every such marker
  -- (engine/prefab_use.h), so the kit's chairs, sofas and beds are used as prefabs from the one description.
  -- The marker is put where the library's eye comes out: a sitter's eye is 0.72 m over the marker and 0.06 m back,
  -- a lier's 0.25 m over it (systems/prefab_use_system.cpp).
  -- WHERE THINGS GO ON IT: the library's surfaces for the piece (its anchors), each a SLOT marker, so a kit piece
  -- stamped as a prefab holds what is put on it where the city's own planner stands things.
  for _, s in ipairs(furniture.anchors{ name = a.name ~= "" and a.name or "office_chair", variant = a.variant }) do
    m:marker("slot_" .. s.id, { pos = { s.at[1], s.at[2], s.at[3] - d / 2 }, role = "slot", size = math.min(s.w, s.d) })
  end
  local count = { sit = 0, lie = 0 }
  for _, u in ipairs(furniture.uses{ name = a.name ~= "" and a.name or "office_chair", variant = a.variant }) do
    if count[u.verb] then
      count[u.verb] = count[u.verb] + 1
      local fx, fz = u.look[1], u.look[3]
      local len = math.sqrt(fx * fx + fz * fz)
      if len < 1e-6 then fx, fz, len = 0, 1, 1 end
      fx, fz = fx / len, fz / len
      local at = u.verb == "sit" and { u.eye[1] + fx * 0.06, u.eye[2] - 0.72, u.eye[3] + fz * 0.06 }
                 or { u.eye[1], u.eye[2] - 0.25, u.eye[3] }
      m:marker(u.verb .. "_" .. count[u.verb], { pos = { at[1], at[2], at[3] - d / 2 }, turn = { 0, math.deg(math.atan(fx, fz)), 0 },
                                                   role = u.verb == "sit" and "seat" or "lie", size = 0.4 })
    end
  end
end

-- A KERB PROP: its mesh and its box, as street_props.lua says them.
function builders.street_prop()
  local props = require "street_props"
  local want = a.name ~= "" and a.name or "hydrant"
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
  local height = a.height
  m:add(streetfurniture.lamp{ height = height })
  local r = 0.09
  m:shape("pole", { capsule = { radius = r, height = height - 2 * r }, pos = { 0, height / 2, 0 } })
  m:marker("lamp", { pos = { 0, height, 0 }, turn = { 90, 0, 0 }, role = "lamp" })
end

-- A TREE: bark and canopy; from 80 m a lighter tree of the same seed (fewer clumps). The trunk is what collides.
function builders.tree()
  local seed, shape, height = a.seed, a.shape, a.height
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
  local rock = stylized.rock(a.seed, { family = a.family, size = a.size })
  m:add(rock)
  m:shape("rock", { hull = rock })
end

builders[a.kind]()
return m
