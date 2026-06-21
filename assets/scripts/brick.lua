-- brick.lua — a brick albedo texture composed from primitives (ADR-0043), not
-- requested as a preset. A running-bond brick lattice is modulated by fbm grit,
-- then two colours (mortar grey -> brick red) are mixed by the mask and baked.
-- Tune any number and re-bake (--bake-texture) to iterate.

local function brick(opts)
  opts = opts or {}
  local lattice = texture.brick{
    cols = opts.cols or 6, rows = opts.rows or 14,
    mortar = opts.mortar or 0.09, variation = opts.variation or 0.4,
    seed = opts.seed or 7,
  }
  -- Fine grit breaks up the flat brick faces (kept near 1 so bricks stay bright).
  local grit = texture.fbm{ seed = 9, scale = 40, octaves = 4 }:scale_bias(0.25, 0.78)
  local mask = lattice:mul(grit)

  return texture.bake_color(mask,
    opts.mortar_color or {0.64, 0.62, 0.58},   -- mortar (mask = 0)
    opts.brick_color  or {0.58, 0.20, 0.14},   -- brick  (mask = 1)
    opts.size or 256)
end

return brick()
