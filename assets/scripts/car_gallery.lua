-- car_gallery.lua — every vehicle class in a row, for judging them against each
-- other rather than one at a time. A class that looks fine alone can still be
-- indistinguishable from its neighbour, and that only shows up side by side.
--
--   opts.classes  list of class names (default: all five wheeled car classes)
--   opts.spacing  metres between centres (default 6.5)
--   opts.lod      "high" | "mid" | "low" | "proxy"
--   opts.no_glass drop the glass so interiors read
--   opts.driver   false to hide occupants

local classes = require "vehicle_classes"
local forms   = require "vehicle_forms"

local opts = args or {}
local names = opts.classes or { "hatchback", "sedan", "jeep", "pickup", "van" }
local spacing = opts.spacing or 6.5
local lod = opts.lod or "high"

local m = model.new()

local MATS = {
  body     = material.new{ roughness = 0.40, metallic = 0.50 },
  glass    = material.new{ roughness = 0.06, metallic = 0.0, opacity = 0.30,
                           two_sided = true },
  interior = material.new{ roughness = 0.90, metallic = 0.0 },
  lamp     = material.new{ roughness = 0.25, metallic = 0.0,
                           emission = { 1.1, 1.0, 0.85 } },
  rubber   = material.new{ roughness = 0.85, metallic = 0.0 },
  pad      = material.new{ roughness = 0.95, metallic = 0.0 },
}

-- Per-class paint, so neighbours are separable at a glance.
local PAINT = {
  hatchback = { 0.15, 0.45, 0.75 },
  sedan     = { 0.72, 0.10, 0.10 },
  jeep      = { 0.22, 0.34, 0.24 },
  pickup    = { 0.20, 0.28, 0.42 },
  van       = { 0.86, 0.86, 0.88 },
  box_truck = { 0.90, 0.72, 0.20 },
  bus       = { 0.85, 0.45, 0.10 },
}

local n = #names
local total = (n - 1) * spacing
m:add(scope{ origin = { -total * 0.5 - 4, -0.3, -4 },
             size = { total + 8, 0.3, 8 } }:box{ 0.30, 0.32, 0.34 })

for i, name in ipairs(names) do
  local ox = (i - 1) * spacing - total * 0.5
  local c = classes.classes[name]
  if c then
    local d = classes.dims(c)
    local car = mesh.car(forms.car_params(c, d,
                          { color = PAINT[name] or { 0.6, 0.6, 0.6 }, lod = lod }))
    -- Origin is the body centre, so lift by half the height to stand it on the pad.
    local lift = car.half_extent[2]
    local function place(part, mat)
      if part then m:add(mesh.translate(part, { ox, lift, 0 }), mat) end
    end
    place(car.body, MATS.body)
    if opts.no_glass ~= true then place(car.glass, MATS.glass) end
    place(car.interior, MATS.interior)
    place(car.lamp, MATS.lamp)
    place(car.wheel, MATS.rubber)

    -- Wheels from the SAME package numbers the arches were cut from, so a
    -- mismatch between tyre and arch is visible rather than inferred.
    if car.wheel == nil then
      local r = c.wheel_diameter * 0.5
      local wheel = mesh.rotate_z(mesh.cylinder(r, math.max(0.18, c.track * 0.13), 14),
                                  math.pi * 0.5)
      local frontZ = d.length * 0.5 - d.front_overhang
      local rearZ = -(d.length * 0.5 - d.rear_overhang)
      for _, sx in ipairs({ 1, -1 }) do
        for _, wz in ipairs({ frontZ, rearZ }) do
          m:add(mesh.bake_height_color(
                  mesh.translate(wheel, { ox + sx * c.track * 0.5, r, wz }),
                  { 0.05, 0.05, 0.06 }, { 0.05, 0.05, 0.06 }), MATS.rubber)
        end
      end
    end

    if opts.driver ~= false then
      for _, l in ipairs(car.lights or {}) do
        if l.name == "driver_seat" then
          m:add(mesh.translate(mesh.occupant{ back_angle = c.a40 or 25 },
                               { ox + l.pos[1], l.pos[2] + lift, l.pos[3] }),
                MATS.interior)
        end
      end
    end
  end
end

-- One 1.75 m reference at the end of the row: relative size between two cars is
-- easy to see, absolute size is not.
m:add(scope{ origin = { total * 0.5 + 2.0, 0.0, -0.15 },
             size = { 0.42, 1.75, 0.28 } }:box{ 0.20, 0.42, 0.30 })

return m
