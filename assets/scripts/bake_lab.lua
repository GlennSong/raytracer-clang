-- bake_lab.lua -- THE UNWRAPPER AND THE BAKER on the fleet (#191; Glenn, 2026-11-06: "let's build our own then and make
-- a battle tested"): each kit car three ways, a row each 9 m apart -- its mid-range body as drawn (the cage, normals
-- transferred), the same body unwrapped (mesh.unwrap) with a normal map baked from the smooth body (mesh.bake), and
-- the smooth body itself.
local vehicle = require "vehicles"
local opts = args or {}
local ORDER = opts.only or { "sedan", "suv", "pickup", "hatchback", "convertible", "step_van" }
local COLOR = { sedan = { 0.62, 0.06, 0.07 }, suv = { 0.09, 0.09, 0.11 }, pickup = { 0.14, 0.30, 0.20 },
                hatchback = { 0.85, 0.72, 0.10 }, convertible = { 0.80, 0.12, 0.10 }, step_van = { 0.36, 0.22, 0.10 } }
local m = model.new()
local plain = material.new{ roughness = 0.30, metallic = 0.55 }
m:add(scope{ origin = { -6, -0.3, -24 }, size = { 5 * #ORDER + 6, 0.3, 34 } }:box{ 0.30, 0.32, 0.34 })
local function floorOf(mm)
  local lo
  mesh.move_by(mm, function(_, p) if not lo or p[2] < lo then lo = p[2] end end)
  return lo or 0
end
for i, name in ipairs(ORDER) do
  local x = (i - 1) * 5
  local f = vehicle.kit_forms(name, COLOR[name] or { 0.5, 0.5, 0.5 })
  local lift = -floorOf(f.near)
  m:add(mesh.translate(f.mid, { x, lift, 0 }), plain)
  -- BY PART: the body baked from the smooth body, the glass and the wheels left as they are
  local parts = vehicle.kit_parts(name, COLOR[name] or { 0.5, 0.5, 0.5 })
  local low, maps, rep, pieces = mesh.bake_parts(parts, { texels_per_metre = opts.texels or 96 }, { cage = 0.06, samples = 2 })
  print(string.format("[bake_lab] %s: %d charts, %d px, %d texels, %d found", name, rep.charts, rep.size, maps.texels, maps.hits))
  local baked = material.new{ roughness = 0.30, metallic = 0.55, normal = maps.normal }
  for k, part in ipairs(parts) do   -- each part its own material: the body its baked normals, the glass and wheels plain
    m:add(mesh.translate(pieces[k], { x, lift, -9 }), part.bake == false and plain or baked)
  end
  m:add(mesh.translate(f.near, { x, lift, -18 }), plain)
end
return m
