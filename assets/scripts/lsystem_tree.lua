-- lsystem_tree.lua — a deliberately SIMPLE L-system tree (ADR-0021/0044): a few
-- production rules, expanded into a turtle string, skinned to tapered branch
-- cylinders, with a leaf canopy of green blobs at the upper tips. Unlike the
-- heavyweight parametric flora.* trees, this is the whole grammar in a dozen
-- lines — small enough to read, rotate (mesh.rotate_y / place) and instance.
--
-- Returns one model (branches + canopy, baked vertex colour, collidable), so it
-- previews on its own (shape:"script") and twin_cities.lua reuses make_tree to
-- instance a forest. `args.seed` / `args.iterations` tune it.

-- Build a tree MESH (branches + canopy), deterministic for `seed`.
function make_tree(seed, iters)
  seed = math.floor(seed or 1)
  iters = iters or 2
  -- Grammar: a branch sprouts three side shoots pitched and rolled out of plane,
  -- so the crown fills in 3D (not a flat fan). Two iterations keep it stout and
  -- cheap enough to instance a forest (a deeper recursion explodes the tri count).
  -- A single leading shoot keeps the trunk short while every node throws four
  -- side shoots, so iterating bushes it out instead of growing a tall stick.
  local sys = lsystem.create()
  sys:rule("F", "F[+&F][-^F][/&F][\\&F]")
  local sym = sys:expand("FF", math.max(iters, 3), seed)

  -- Low slice count: a tree read at distance + instanced many times, so keep the
  -- branch cylinders coarse (the silhouette carries it, not the facets).
  local tp = { length = 1.0, radius = 0.26, angle_deg = 38, taper = 0.7,
               segment_slices = 5 }
  local branches = lsystem.turtle_mesh(sym, tp)
  -- Flat bark tint: bake the same colour at low and high so height doesn't matter.
  branches = mesh.bake_height_color(branches, { 0.30, 0.21, 0.13 }, { 0.42, 0.31, 0.19 })

  -- Canopy: a fat green blob at every branch tip in the upper crown — overlapping
  -- spheres read as one leafy mass.
  local segs = lsystem.segments(sym, tp)
  local maxy = 0.001                       -- lsystem.segments returns vec3 arrays
  for _, s in ipairs(segs) do if s.b[2] > maxy then maxy = s.b[2] end end
  -- Upper-crown tips only (so the trunk shows), capped so overlapping spheres
  -- stay cheap. Collect candidates, then keep an evenly-spaced subset.
  local tips = {}
  for _, s in ipairs(segs) do
    if s.b[2] > maxy * 0.55 then tips[#tips + 1] = s.b end
  end
  local blobs = {}
  local stride = math.max(1, math.floor(#tips / 14))
  for i = 1, #tips, stride do
    local r = 0.85 + 0.5 * ((seed + #blobs) % 4) / 4
    blobs[#blobs + 1] = mesh.translate(mesh.sphere(r), tips[i])
  end
  if #blobs == 0 then return branches end
  local crown = mesh.bake_height_color(mesh.merge(blobs),
                                       { 0.16, 0.33, 0.15 }, { 0.27, 0.47, 0.21 })
  return mesh.merge({ branches, crown })
end

-- Standalone preview: one collidable tree at the origin.
local seed = (args and args.seed) or seed or 1
local iters = (args and args.iterations) or 3
return model.new():add_solid(make_tree(seed, iters))
