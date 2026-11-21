-- MODEL KIT: the furniture kit's shape words, for models with moving parts (#191; Glenn, 2026-11-20: an office
-- chair that adjusts, a desk that goes up and down, and more furniture the city can use later).
-- A library: `local kit = require "model_kit"`.
--
-- The furniture kit (furniture_models.lua) builds a piece as one still shape: boxes with rounded edges, rods,
-- turned shapes, cushions, drawer fronts, each in a FINISH (wood, fabric, hard, metal, ceramic, light). This
-- library lets a recipe build with the same words and put what it built on a NODE of a model, so a drawer, a door
-- or a seat is a part that moves, in the same finishes and colours as the furniture the interiors already place.
--
--   kit.V(x, y, z)                a point that adds and scales (a + b, a * k), as the shape words expect
--   kit.WOOD, kit.FABRIC, kit.WHITE, kit.BLACK, kit.STEEL, kit.CHROME, kit.PORCELAIN     the kit's colours
--   kit.wood_names, kit.fabric_names     the names of those woods and fabrics, in order, for a recipe's `params`
--   kit.wood(name), kit.fabric(name)     a colour by its name
--   kit.paint[name], kit.paints          the painted-metal colours (the playground's; playground_kit.lua)
--
--   local k = kit.maker()         something to build on: k.box, k.rod, k.turned, k.cushion, k.front, k.bent ...
--                                 (furniture_models.lua, "THE MAKER"), and k.pipe(part, points, r, col, o) for a
--                                 round pipe with its corners rounded
--   kit.put(m, node, k [, o])     what was built on k goes on `node` (the model `m` itself, or one of its nodes),
--                                 one part for each finish used; o.level puts it in a level of detail
--   kit.far_box(m, node, finish, size, at, col, level)   a plain box for the far level
--   kit.carcass(k, finish, W, H, D, T, col [, y0])   a cabinet's shell, open toward +z: two sides, a back, a top
--                                 and a floor of boards T thick, standing from y0 (0) up to H
--   kit.facing(mesh)              a shape turned about +y (a disc, a knob) laid over to face +z
--   kit.ball(r [, segs])          a ball about the origin with few triangles (mesh.sphere's has thousands)
--
-- A finish names one of the model's materials, the same ones a furniture piece stamped as a model has
-- (prop_models.lua): furniture_wood, furniture_fabric, furniture, furniture_metal, furniture_ceramic, lit_band;
-- and here also `glass`, `rubber` and `screen`.
require "furniture_models"
local playground = require "playground_kit"
local F = furniture_kit

local M = { V = F.V, grey = F.grey, WOOD = F.WOOD, FABRIC = F.FABRIC, WHITE = F.WHITE, BLACK = F.BLACK, STEEL = F.STEEL,
            CHROME = F.CHROME, PORCELAIN = F.PORCELAIN, LINEN = F.LINEN, paint = {}, paints = playground.paints }
-- the playground's paints, as colours that scale (col * 0.8)
for name, c in pairs(playground.paint) do M.paint[name] = F.V(c[1], c[2], c[3]) end

M.wood_names = { "oak", "walnut", "birch", "ebonised" }
M.fabric_names = { "grey", "navy", "sage", "rust", "oat", "charcoal", "mustard", "teal" }
local function by_name(names, list, name)
  for i, n in ipairs(names) do if n == name then return list[i] end end
  return list[1]
end
function M.wood(name) return by_name(M.wood_names, F.WOOD, name) end
function M.fabric(name) return by_name(M.fabric_names, F.FABRIC, name) end

-- THE FINISHES: a maker's part name, the material's name on the model, and the material
M.finishes = {
  wood    = { "furniture_wood", { surface = "woodgrain", roughness = 0.55 } },
  fabric  = { "furniture_fabric", { surface = "fabric", roughness = 0.95 } },
  hard    = { "furniture", { roughness = 0.55 } },
  metal   = { "furniture_metal", { roughness = 0.3, metallic = 0.9 } },
  ceramic = { "furniture_ceramic", { roughness = 0.15 } },
  light   = { "lit_band", { roughness = 0.5, emission = { 1, 0.95, 0.85 } } },
  glass   = { "glass", { roughness = 0.05, metallic = 0.0, opacity = 0.3 } },
  rubber  = { "rubber", { roughness = 0.95, metallic = 0.0 } },
  screen  = { "screen", { roughness = 0.1, metallic = 0.0 } },
  paint   = { "paint", { roughness = 0.42, metallic = 0.25 } },
}
local made = {}
local function material_of(m, part)
  local f = M.finishes[part]
  if not f then error("model_kit: no finish named '" .. tostring(part) .. "'", 3) end
  if not m:has_material(f[1]) then
    made[part] = made[part] or material.new(f[2])
    m:material(f[1], made[part])
  end
  return f[1]
end
M.material_of = material_of

function M.maker()
  local k = F.maker()
  -- a round pipe along a line of points; o.bend rounds its corners to that radius, o.sides, o.closed
  function k.pipe(part, points, r, col, o)
    k.put(part, playground.pipe(points, r, col, o))
  end
  -- a plain painted mesh of any kind, already in place
  function k.mesh(part, mesh_, col)
    k.put(part, col and mesh.paint(mesh_, col) or mesh_)
  end
  return k
end

function M.put(m, node, k, o)
  local names = {}
  for part in pairs(k.lists) do names[#names + 1] = part end
  table.sort(names)                          -- the same order every time, so the model is the same model
  for _, part in ipairs(names) do
    node:add(mesh.merge(k.lists[part]), material_of(m, part), o)
  end
end

function M.far_box(m, node, part, size, at, col, level)
  node:add(mesh.paint(mesh.translate(mesh.box(size), at), col), material_of(m, part), { level = level })
end

function M.carcass(k, part, W, H, D, T, col, y0)
  y0 = y0 or 0
  local h = H - y0
  for _, x in ipairs{ -W / 2 + T / 2, W / 2 - T / 2 } do k.box(part, M.V(x, y0 + h / 2, 0), M.V(T, h, D), 0.006, col, 1) end
  k.box(part, M.V(0, y0 + h / 2, -D / 2 + T / 2), M.V(W - 2 * T, h, T), 0.0, col, 1)
  k.box(part, M.V(0, H - T / 2, 0), M.V(W - 2 * T, T, D), 0.0, col, 1)
  k.box(part, M.V(0, y0 + T / 2, 0), M.V(W - 2 * T, T, D), 0.0, col, 1)
end

function M.facing(mesh_) return mesh.rotate_x(mesh_, math.pi / 2) end

function M.ball(r, segs)
  segs = segs or 12
  local profile = {}
  for i = 0, segs // 2 do
    local t = -math.pi / 2 + math.pi * i / (segs // 2)
    profile[#profile + 1] = { r * math.cos(t), r * math.sin(t) }
  end
  profile[1][1], profile[#profile][1] = 0, 0
  return mesh.lathe{ profile = profile, segments = segs, smooth = true }
end

-- a caster standing on y = 0 under the origin: a fork and a wheel, `r` the wheel's radius
function M.caster(k, at, r)
  r = r or 0.03
  k.box("hard", at + M.V(0, r * 2 + 0.008, 0), M.V(r * 1.3, 0.012, r * 1.3), 0.004, M.BLACK, 1)
  k.mesh("hard", mesh.translate(mesh.rotate_z(mesh.lathe{ profile = { { 0, -r * 0.45 }, { r * 0.8, -r * 0.45 }, { r, -r * 0.2 }, { r, r * 0.2 },
                                                                   { r * 0.8, r * 0.45 }, { 0, r * 0.45 } }, segments = 12, smooth = true },
                                                math.pi / 2), { at[1], at[2] + r, at[3] + r * 0.4 }), M.BLACK)
end

return M
