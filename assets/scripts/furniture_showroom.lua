-- furniture_showroom.lua -- THE FURNITURE KIT laid out for a look (buildings M4b; Glenn, 2026-09-30: "legit build
-- furniture"). Every piece of procgen/furniture_kit.h in three variants (wood x fabric), a row per room, each
-- piece standing with its back to the row's line, facing +z. Hot-reloads like the building lab.
local opts = args or {}
local m = model.new()

local MATS = {
  furniture_wood    = material.new{ surface = "woodgrain", roughness = 0.5 },
  furniture_fabric  = material.new{ surface = "fabric", roughness = 0.92 },
  furniture         = material.new{ roughness = 0.55 },
  furniture_metal   = material.new{ roughness = 0.28, metallic = 0.85 },
  furniture_ceramic = material.new{ roughness = 0.12 },
  lit_band          = material.new{ roughness = 0.5, emission = { 1, 0.95, 0.85 } },
  floor             = material.new{ surface = "woodgrain", roughness = 0.6 },
}

-- every piece in the kit, in the kit's order, nine to a row (the port's A/B: #191, the recipes in Lua)
local ROWS, row = {}, nil
for i, name in ipairs(furniture.pieces()) do
  if (i - 1) % 9 == 0 then row = {}; ROWS[#ROWS + 1] = row end
  row[#row + 1] = name
end
-- variant: bits 0-2 fabric, 3-4 wood, 5+ style
local VARIANTS = { 0 + 0 * 8, 1 + 1 * 8 + 32, 3 + 2 * 8 + 64 }

local z, maxX = 0, 0
for _, row in ipairs(ROWS) do
  local x = 0
  local depth = 0
  for _, name in ipairs(row) do
    for vi, v in ipairs(VARIANTS) do
      local piece = furniture.piece{ name = name, variant = v }
      local w = piece.size[1]
      local cx = x + w / 2
      for _, e in ipairs(piece) do
        m:add(mesh.translate(e.mesh, {cx, 0, z}), MATS[e.part])
      end
      x = x + w + 0.6
      depth = math.max(depth, piece.size[3])
    end
    x = x + 0.8
  end
  maxX = math.max(maxX, x)
  z = z + depth + 2.2
end
-- the showroom floor
m:add(scope{ origin = {-2, -0.05, -2}, size = {maxX + 4, 0.05, z + 2} }:box{ 0.80, 0.74, 0.66 }, MATS.floor)
return m
