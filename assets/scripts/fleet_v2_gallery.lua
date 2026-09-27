-- fleet_v2_gallery.lua -- every fleet v2 body (vehicle_kit.lua, on the poly.* kit) in a row, 5 m apart
-- along x, noses toward +z, for judging shape and materials. opts.level = subdivision level (default 2),
-- opts.only = a list of spec names.

local kit = require "vehicle_kit"
local opts = args or {}
local level = opts.level or 0
local ORDER = opts.only or { "sedan", "taxi", "hatchback", "suv", "jeep", "convertible", "pickup",
                             "step_van", "small_truck", "semi", "semi_tanker" }

local m = model.new()
local MATS = {
  body     = material.new{ roughness = 0.30, metallic = 0.55 },
  glass    = material.new{ roughness = 0.04, metallic = 0.0, opacity = 0.30, two_sided = true },
  liner    = material.new{ roughness = 0.90, metallic = 0.0 },
  carpet   = material.new{ roughness = 0.95, metallic = 0.0 },
  seat     = material.new{ roughness = 0.80, metallic = 0.0 },
  dash     = material.new{ roughness = 0.70, metallic = 0.0 },
  steer    = material.new{ roughness = 0.60, metallic = 0.0 },
  trim     = material.new{ roughness = 0.60, metallic = 0.0 },
  grille   = material.new{ roughness = 0.45, metallic = 0.6 },
  gasket   = material.new{ roughness = 0.75, metallic = 0.0 },
  interior = material.new{ roughness = 0.85, metallic = 0.0 },
  bed      = material.new{ roughness = 0.80, metallic = 0.1 },
  box      = material.new{ roughness = 0.55, metallic = 0.1 },
  trailer  = material.new{ roughness = 0.45, metallic = 0.3 },
  chassis  = material.new{ roughness = 0.70, metallic = 0.2 },
  tank     = material.new{ roughness = 0.25, metallic = 0.85 },
  door     = material.new{ roughness = 0.50, metallic = 0.2 },
  sign     = material.new{ roughness = 0.30, metallic = 0.0, emission = { 0.9, 0.7, 0.1 } },
  lamp     = material.new{ roughness = 0.15, metallic = 0.0, emission = { 0.9, 0.88, 0.8 } },
  lamp_red = material.new{ roughness = 0.20, metallic = 0.0, emission = { 0.6, 0.02, 0.02 } },
  rubber   = material.new{ roughness = 0.85, metallic = 0.0 },
  chrome   = material.new{ roughness = 0.25, metallic = 0.8 },
}
local COLOR = {
  glass = { 0.60, 0.66, 0.70 }, liner = { 0.55, 0.53, 0.50 }, carpet = { 0.10, 0.10, 0.11 },
  seat = { 0.22, 0.20, 0.19 }, dash = { 0.08, 0.08, 0.09 }, steer = { 0.05, 0.05, 0.05 }, trim = { 0.04, 0.04, 0.045 }, grille = { 0.06, 0.06, 0.065 }, chrome = { 0.75, 0.76, 0.78 }, gasket = { 0.025, 0.025, 0.028 }, interior = { 0.42, 0.30, 0.20 },
  bed = { 0.08, 0.08, 0.09 }, chassis = { 0.07, 0.07, 0.08 }, tank = { 0.78, 0.79, 0.80 }, door = { 0.80, 0.81, 0.82 }, box = { 0.92, 0.92, 0.90 }, trailer = { 0.86, 0.87, 0.88 }, sign = { 1.0, 0.85, 0.2 },
  lamp = { 0.95, 0.95, 0.92 }, lamp_red = { 0.8, 0.05, 0.05 },
}
local SPACING = 5.0
if #ORDER > 10 then SPACING = 5.0 end
local n = #ORDER
m:add(scope{ origin = { -6, -0.3, -24 }, size = { SPACING * n + 6, 0.3, 34 } }:box{ 0.30, 0.32, 0.34 })

for i, name in ipairs(ORDER) do
  local x = (i - 1) * SPACING
  local P = kit.SPECS[name]()
  local car = kit.build(P, level)
  local zoff = 0
  if name == "semi" or name == "semi_tanker" then zoff = car.length * 0.5 - 2.4 end   -- the tractor's nose in line with the others
  for part, mm in pairs(car.parts) do
    local col = part == "body" and (car.color or { 0.5, 0.5, 0.5 }) or (COLOR[part] or { 0.5, 0.5, 0.5 })
    m:add(mesh.translate(mesh.bake_height_color(mm, col, col), { x, car.lift, zoff }), MATS[part] or MATS.trim)
  end
  for part, mm in pairs(car.cabin or {}) do
    local col = COLOR[part] or { 0.4, 0.4, 0.4 }
    m:add(mesh.translate(mesh.bake_height_color(mm, col, col), { x, car.lift, zoff }), MATS[part] or MATS.trim)
  end
  if car.driver_seat and car.cabin then
    local ds = car.driver_seat
    m:add(mesh.translate(mesh.occupant{ back_angle = 22 }, { x + ds[1], ds[2] + car.lift, ds[3] + zoff }), MATS.seat)
  end
  local wheel = kit.wheel(P.wheel_r, P.wheel_w, level)
  local tyre = mesh.bake_height_color(wheel.tyre, { 0.05, 0.05, 0.055 }, { 0.05, 0.05, 0.055 })
  local rimR = mesh.bake_height_color(wheel.rim, { 0.75, 0.76, 0.78 }, { 0.75, 0.76, 0.78 })
  local rimL = mesh.rotate_y(rimR, math.pi)
  for _, w in ipairs(car.wheels) do
    local at = { x + w.pos[1], w.pos[2] + car.lift, w.pos[3] + zoff }
    m:add(mesh.translate(tyre, at), MATS.rubber)
    m:add(mesh.translate(w.pos[1] > 0 and rimR or rimL, at), MATS.chrome)
    if w.dual then   -- the inner tyre of a dual pair
      local inner = { at[1] - (w.pos[1] > 0 and 1 or -1) * (w.width + 0.03), at[2], at[3] }
      m:add(mesh.translate(tyre, inner), MATS.rubber)
    end
  end
  if car.spare then
    local sp = mesh.rotate_y(tyre, math.pi * 0.5)
    m:add(mesh.translate(sp, { x + car.spare[1], car.spare[2] + car.lift, car.spare[3] + zoff }), MATS.rubber)
  end
end
return m
