-- brickhouse.lua — a building wearing a brick material, all composed in Lua
-- (ADR-0043). The brick texture is built from primitives, bundled into a material
-- (albedo + a normal map from the lattice height for mortar relief), and applied
-- to a scope-box building by world-planar projection — no authored UVs.

-- The brick texture, piece by piece. One tile spans `tile` world metres below,
-- so 5 courses / 4 columns per tile keeps the bricks at a believable size.
local lattice = texture.brick{ cols = 4, rows = 5, mortar = 0.07,
                               variation = 0.5, seed = 7 }
local grit    = texture.fbm{ seed = 9, scale = 18, octaves = 4 }:scale_bias(0.2, 0.85)
local albedo  = texture.bake_color(lattice:mul(grit),
                                   {0.58, 0.55, 0.50},   -- mortar (light grey)
                                   {0.52, 0.16, 0.11},   -- brick (deep red)
                                   256)
-- Bricks proud, mortar recessed -> a normal map for relief (mild).
local normal  = texture.bake_normal(lattice:scale_bias(0.5, 0.5), 1.2, 256)

local brick = material.new{ albedo = albedo, normal = normal,
                            roughness = 0.9, tile = 1.5 }

local m = model.new()
m:add(scope{ origin = {-6, 0, -5},   size = {12, 9, 10} }:box{1, 1, 1}, brick)
-- A plain capping roof slab (untextured, vertex-coloured).
m:add(scope{ origin = {-6.5, 9, -5.5}, size = {13, 0.5, 11} }:box{0.30, 0.30, 0.32})
return m
