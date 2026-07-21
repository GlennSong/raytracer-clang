-- car_lab.lua — the CAR LAB recipe.
--
-- One vehicle on a pedestal, regenerated live: the level watches this file
-- ("watch_scripts": true) and rebuilds the scene whenever it changes, so a
-- geometry edit and an on-screen look iterate in seconds. Mirrors
-- building_lab.lua exactly, including the U/O keys the watcher binds.
--
-- Knobs ride the level entity's `opts` (read here via the `args` global):
--   paint      = {r, g, b}
--   lod        = "high" | "mid" | "low" | "proxy"
--   tumblehome = degrees the sides lean in above the belt
--   chamfer    = corner chamfer, fraction of half-extent
--   scale_ref  = false to hide the human-height reference block
--
-- The body comes from mesh.car with the C++ class defaults (a sedan). Per-class
-- RECIPES over vehicle_classes.lua land with the other chassis families; until
-- then this lab varies the sedan.
--
-- LAB DRESSING vs GENERATOR OUTPUT — worth keeping straight while reviewing:
-- the wheels and the scale block below are drawn by THIS FILE, not by the
-- generator. Real wheels are a separate CarPart the caller places (that is what
-- stops them being baked twice); the placeholders here exist so the car's
-- stance is judgeable instead of floating.

local opts  = args or {}
local paint = opts.paint or { 0.72, 0.10, 0.10 }
local lod   = opts.lod or "high"

local m = model.new()

-- Pedestal: a pad to sit on and read shadows against.
m:add(scope{ origin = {-7, -0.3, -5}, size = {14, 0.3, 10} }:box{0.30, 0.32, 0.34})
m:add(scope{ origin = {-4, 0.0, -2.4}, size = {8, 0.02, 4.8} }:box{0.38, 0.39, 0.40})

local MATS = {
  body   = material.new{ roughness = 0.40, metallic = 0.50 },
  -- two_sided: ONE pane drawn from both faces. Without it the pane would have
  -- to be emitted twice at the same coordinates, and since the transparent pass
  -- does not write depth both copies blend — which is what made the glass read
  -- as a solid panel instead of a window.
  glass  = material.new{ roughness = 0.06, metallic = 0.00, opacity = 0.30,
                         two_sided = true },
  interior = material.new{ roughness = 0.90, metallic = 0.00 },
  -- Lamps are drawn EMISSIVE here so headlights and tail lights read in the
  -- lab. In the city the same lens part is bound unlit and citysim overlays lit
  -- instances at the markers per state (clock for headlights, deceleration for
  -- brakes, blink for indicators) — see car_lamps.h.
  lamp   = material.new{ roughness = 0.25, metallic = 0.0,
                         emission = opts.lamp_emission or { 1.1, 1.0, 0.85 } },
  rubber = material.new{ roughness = 0.85, metallic = 0.00 },
}

local car = mesh.car{
  paint      = paint,
  lod        = lod,
  tumblehome = opts.tumblehome,
  chamfer    = opts.chamfer,
}

-- The car's origin is its BODY CENTRE, so lift it by half its height to stand
-- the wheels on the pad rather than burying them in it.
local half = car.half_extent
local liftY = half[2]

local function place(mesh_, mat)
  if mesh_ then m:add(mesh.translate(mesh_, {0, liftY, 0}), mat) end
end
place(car.body, MATS.body)
if opts.no_glass ~= true then place(car.glass, MATS.glass) end
place(car.interior, MATS.interior)
place(car.lamp, MATS.lamp)
place(car.wheel, MATS.rubber)

-- Placeholder wheels (LAB DRESSING — see the header note). Sized and placed
-- from the same package numbers the generator uses for its arch openings, so if
-- these do not sit inside the arches, the arches are wrong.
if car.wheel == nil then
  local WD, FO, RO = 0.62, 0.90, 1.00     -- wheel diameter, front/rear overhang
  local L, W = half[3] * 2, half[1] * 2
  local frontZ, rearZ = L * 0.5 - FO, -(L * 0.5 - RO)
  local wheel = mesh.rotate_z(mesh.cylinder(WD * 0.5, 0.20, 14), math.pi * 0.5)
  for _, sx in ipairs({ 1, -1 }) do
    for _, wz in ipairs({ frontZ, rearZ }) do
      m:add(mesh.bake_height_color(
              mesh.translate(wheel, { sx * (W * 0.5 - 0.06), WD * 0.5, wz }),
              {0.05, 0.05, 0.06}, {0.05, 0.05, 0.06}), MATS.rubber)
    end
  end
end

-- The DRIVER, placed at the car's own "driver_seat" marker rather than at a
-- guessed offset — the same contract VehicleSystem uses for the player's car, so
-- if the marker is wrong the lab shows it.
if opts.driver ~= false then
  for _, l in ipairs(car.lights or {}) do
    if l.name == "driver_seat" then
      m:add(mesh.translate(mesh.occupant{ back_angle = opts.back_angle or 25 },
                           { l.pos[1], l.pos[2] + liftY, l.pos[3] }), MATS.interior)
    end
  end
end

-- A human-height reference. Proportion errors are invisible against an empty
-- pad and obvious against something 1.75 m tall standing next to the car.
if opts.scale_ref ~= false then
  m:add(scope{ origin = {-3.2, 0.0, -1.9}, size = {0.42, 1.75, 0.28} }
          :box{0.20, 0.42, 0.30})
end

return m
