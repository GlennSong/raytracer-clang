-- Procedural vehicle bodies (ADR-0059) — authored in Lua over the procgen
-- builders, exactly like flora.lua and the gun. Defines a global `vehicle` table:
--
--   vehicle.sedan / hatchback / jeep / van / pickup (seed, opts) -> spec
--
-- All five are one builder, from_class, over the SAE package data in
-- vehicle_classes.lua and the curve layer in vehicle_forms.lua (both reached by
-- `require`). Proportions and handling derive from the class; opts override any
-- field. `opts.class_overrides` tweaks the package before it is validated.
--
-- A "spec" is a plain table the C++ VehicleSpec reader (vehicle_spec.cpp)
-- consumes: a `body` Mesh built with mesh.* plus handling/physics parameters and
-- a wheel layout. Hosts load this once so `vehicle` is available, then run a
-- small chunk like `return vehicle.sedan(seed, {color={0,0.4,0.8}})`. Pure and
-- deterministic per seed.
--
-- Convention: the car faces +Z. HEADLIGHTS (pale) are at +Z, TAILLIGHTS (red) at
-- -Z, so the front reads at a glance and "W = drive toward the headlights".

vehicle = {}

-- The package data (SAE J1100/J826) and the form layer that turns it into
-- mesh.car curves. Both are shared modules — see script_modules.h for why
-- `require` exists in a sandbox that deliberately has no `package`.
local classes = require "vehicle_classes"
local forms   = require "vehicle_forms"


-- Build a drivable spec from a CLASS. One builder for every wheeled vehicle:
-- proportions come from vehicle_classes.lua, curves from vehicle_forms.lua, and
-- the shell from mesh.car. It replaces the old box-stack body that gave every
-- class the same silhouette.
--
-- The spec is MULTI-PART. mesh.car returns body / glass / interior / lamp
-- separately because each wants its own material, and a Renderable holds only
-- one. The opaque bits (painted body + dark lamp housings) merge into `body`;
-- the transparent glass and the matte interior ride in `parts`, which
-- spawnVehicle turns into child Renderables pinned to the chassis. That is what
-- lets the player's car show see-through glass and a cabin, matching the lab —
-- earlier this merged everything and the windows read as solid panels.
local function from_class(class_name, seed, opts)
  opts = opts or {}
  local c = classes.apply(class_name, opts.class_overrides)
  local d = classes.dims(c)

  -- Fail loudly on a cartoon. This is the check that caught four bad overhang
  -- values when the class file was written; a recipe override deserves it too.
  local bad = classes.validate(class_name, c)
  if bad then
    error(class_name .. ": " .. table.concat(bad, "; "))
  end

  local color = opts.color or c.color or { 0.70, 0.12, 0.12 }
  local car = mesh.car(forms.car_params(c, d, { color = color, lod = opts.lod }))

  -- Opaque shell: painted body + dark lamp housings, both white-albedo with the
  -- hue in vertex colours (the shader does albedo * vertexColour, so the body
  -- material must stay white or the paint squares and darkens).
  local shell = { car.body }
  if car.lamp then shell[#shell + 1] = car.lamp end
  local body = mesh.recompute_normals(mesh.merge(shell))

  -- Transparent glass + matte interior as their own materials.
  local parts = {}
  if car.glass then
    parts[#parts + 1] = { mesh = car.glass, albedo = { 1, 1, 1 },
                          metallic = 0.0, roughness = 0.06, opacity = 0.30 }
  end
  if car.interior then
    parts[#parts + 1] = { mesh = car.interior, albedo = { 1, 1, 1 },
                          metallic = 0.0, roughness = 0.90 }
  end

  -- Wheels DERIVED, not hardcoded. vehicle.sedan used to declare radius 0.34
  -- against a 0.62 m wheel diameter (r = 0.31), so the simulated wheels were
  -- 3 cm larger than the arches drawn for them.
  local r = c.wheel_diameter * 0.5
  local halfTrack = c.track * 0.5
  local axleY = -d.height * 0.5 + r + c.h156
  local frontZ = d.length * 0.5 - d.front_overhang
  local rearZ = -(d.length * 0.5 - d.rear_overhang)

  -- Mass scales with the box the vehicle occupies; a van is not a heavy sedan.
  local volume = d.length * d.width * d.height
  local mass = opts.mass or math.floor(volume * 120)

  return {
    body = body,
    -- WHITE: the shell mesh already carries the paint in its vertex colours and
    -- the shader multiplies albedo by them. A coloured albedo here squares the
    -- paint (the old box body did exactly that and drove darker than authored).
    albedo = { 1, 1, 1 },
    metallic = 0.6,
    roughness = 0.35,
    parts = parts,           -- transparent glass + matte interior
    lights = car.lights,     -- headlight_l/r, taillight_l/r, driver_seat markers
    chassis = { half = { d.width * 0.5, d.height * 0.5, d.length * 0.5 } },
    mass = mass,
    -- Centre of gravity below the body centre. Anchored so a sedan-height body
    -- (1.45 m) gets exactly -0.45 — the value the sedan was hand-tuned to before
    -- the class recipes ("low CoG so it doesn't tip in turns") — while TALLER
    -- bodies drop further, or a van/jeep rolls over under any real steering. So
    -- the sedan drives exactly as tuned and the height term only ever helps the
    -- new tall classes.
    com_offset = opts.com_offset or -(0.23 + 0.22 * (d.height / 1.45)),
    engine_torque = opts.engine_torque or math.floor(mass * 0.46),
    max_rpm = opts.max_rpm or 6000,
    max_steer_deg = opts.max_steer_deg or (c.form == "coupe" and 32 or 28),
    brake_torque = opts.brake_torque or math.floor(mass * 1.15),
    hand_brake_torque = opts.hand_brake_torque or math.floor(mass * 2.9),
    wheel = { radius = r, width = math.max(0.18, c.track * 0.13) },
    wheels = {
      { x =  halfTrack, y = axleY, z = frontZ, steered = true,  driven = true },
      { x = -halfTrack, y = axleY, z = frontZ, steered = true,  driven = true },
      { x =  halfTrack, y = axleY, z = rearZ,  steered = false, driven = true, hand_brake = true },
      { x = -halfTrack, y = axleY, z = rearZ,  steered = false, driven = true, hand_brake = true },
    },
  }
end

function vehicle.sedan(seed, opts)    return from_class("sedan", seed, opts) end
function vehicle.hatchback(seed, opts)
  opts = opts or {}
  opts.color = opts.color or { 0.15, 0.45, 0.75 }
  return from_class("hatchback", seed, opts)
end
function vehicle.jeep(seed, opts)
  opts = opts or {}
  opts.color = opts.color or { 0.22, 0.34, 0.24 }
  return from_class("jeep", seed, opts)
end
function vehicle.van(seed, opts)
  opts = opts or {}
  opts.color = opts.color or { 0.86, 0.86, 0.88 }
  return from_class("van", seed, opts)
end
function vehicle.pickup(seed, opts)
  opts = opts or {}
  opts.color = opts.color or { 0.20, 0.28, 0.42 }
  return from_class("pickup", seed, opts)
end
vehicle.from_class = from_class


-- ---------------------------------------------------------------------------
-- The AI car FLEET as DATA (ADR-0065). The citysim instanced renderer draws
-- ambient traffic as one mesh per fleet slot; this `fleet` array authors each
-- slot's body the same way buildCarMesh did in C++ — a box composition — so
-- moving it here is a DATA move, not a redesign (the fancier low-poly car is a
-- later task). The C++ reader (engine/scripting/vehicle_body.cpp) turns
-- `vehicle.fleet[slot+1]` into a vertex-coloured RenderMesh at LEVEL LOAD; a
-- Lua-free (Makefile) build keeps the identical C++ fleetCarMesh as a fallback,
-- so the streets are never data-dependent to be non-empty.
--
-- A recipe is:
--   parts  = { { pos={x,y,z}, size={w,h,l}, color={r,g,b} }, ... }   -- boxes
--   lights = { { name=, pos={x,y,z} }, ... }   -- named lamp ATTACHMENT markers
-- `parts` is the boxy body (hull + cabin + dark glass + corner lamps + wheels —
-- the same box primitives, but as flat data).
-- `lights` mark the front/rear lamp positions: the SEAM where future emissive
-- lens entities / light actuators will attach (parsed today, rendered later —
-- see the ADR-0065 register row in docs/decisions.md). Car faces +Z; x>0 = right.
--
-- Slot order + dimensions MIRROR the sim fleet (city_sim.cpp kFleet) and the
-- paints (city_meshes.cpp kCarColors) slot for slot: 3 sedans, 3 hatchbacks,
-- 3 SUVs, a pickup, a van, a box truck.

-- Build one fleet recipe. `style` picks the shape (0 sedan, 1 hatchback, 2 SUV,
-- 3 pickup, 4 van, 5 box truck); W/H/L are the slot's body dims; `color` its
-- paint. Mirrors buildCarMesh's box composition (and its wheels/lights).
local function fleet_car(style, W, H, L, color)
    local hw, hl = W * 0.5, L * 0.5
    local glass = { 0.05, 0.06, 0.09 }
    local tyre  = { 0.04, 0.04, 0.05 }
    local head  = { 1.0, 0.97, 0.82 }
    local tail  = { 0.85, 0.06, 0.05 }
    local parts = {}
    local function box(size, pos, col)
        parts[#parts + 1] = { size = size, pos = pos, color = col }
    end

    if style == 1 then          -- hatchback: cabin carried back, glass fore & aft
        box({ W, H * 0.46, L * 0.92 }, { 0, 0, 0 }, color)
        box({ W * 0.86, H * 0.44, L * 0.56 }, { 0, H * 0.40, -L * 0.06 }, color)
        box({ W * 0.80, H * 0.30, L * 0.05 }, { 0, H * 0.44, L * 0.12 }, glass)
        box({ W * 0.80, H * 0.30, L * 0.05 }, { 0, H * 0.44, -L * 0.34 }, glass)
    elseif style == 2 then      -- SUV: tall hull, big greenhouse
        box({ W, H * 0.58, L }, { 0, 0, 0 }, color)
        box({ W * 0.90, H * 0.46, L * 0.62 }, { 0, H * 0.44, -L * 0.02 }, color)
        box({ W * 0.84, H * 0.34, L * 0.05 }, { 0, H * 0.46, L * 0.20 }, glass)
        box({ W * 0.84, H * 0.34, L * 0.05 }, { 0, H * 0.46, -L * 0.26 }, glass)
    elseif style == 3 then      -- pickup: forward cab + open bed
        box({ W, H * 0.42, L }, { 0, -H * 0.04, 0 }, color)
        box({ W * 0.90, H * 0.40, L * 0.34 }, { 0, H * 0.34, L * 0.22 }, color)
        box({ W * 0.80, H * 0.26, L * 0.05 }, { 0, H * 0.40, L * 0.38 }, glass)
        box({ W * 0.92, H * 0.20, L * 0.42 }, { 0, H * 0.10, -L * 0.26 }, color)  -- bed walls
    elseif style == 4 then      -- van: one tall slab, raked windshield, low nose
        box({ W, H * 0.72, L * 0.86 }, { 0, H * 0.06, -L * 0.06 }, color)
        box({ W, H * 0.34, L * 0.20 }, { 0, -H * 0.10, L * 0.40 }, color)         -- nose
        box({ W * 0.86, H * 0.30, L * 0.05 }, { 0, H * 0.22, L * 0.30 }, glass)   -- windshield
        box({ W * 0.86, H * 0.24, L * 0.05 }, { 0, H * 0.24, -L * 0.48 }, glass)  -- rear glass
    elseif style == 5 then      -- box truck: small cab + tall cargo box
        box({ W, H * 0.44, L * 0.30 }, { 0, -H * 0.04, L * 0.33 }, color)         -- cab lower
        box({ W * 0.94, H * 0.40, L * 0.24 }, { 0, H * 0.30, L * 0.35 }, color)   -- cab roof
        box({ W * 0.84, H * 0.30, L * 0.05 }, { 0, H * 0.30, L * 0.47 }, glass)   -- windshield
        box({ W, H * 0.86, L * 0.62 }, { 0, H * 0.12, -L * 0.17 }, color)         -- cargo box
    else                        -- sedan: baseline proportions
        box({ W, H * 0.46, L }, { 0, 0, 0 }, color)
        box({ W * 0.84, H * 0.42, L * 0.46 }, { 0, H * 0.40, -L * 0.04 }, color)
        box({ W * 0.80, H * 0.30, L * 0.05 }, { 0, H * 0.42, L * 0.15 }, glass)
        box({ W * 0.80, H * 0.26, L * 0.05 }, { 0, H * 0.42, -L * 0.23 }, glass)
    end

    -- Head/taillights at the front (+Z pale) and rear (-Z red) corners: both the
    -- visible lamp BOXES (parts) and the named ATTACHMENT markers (lights), which
    -- coincide so a future emissive lens spawns right on the drawn lamp.
    local ly, lx = -H * 0.08, hw - 0.30
    local lights = {}
    for _, s in ipairs({ 1, -1 }) do
        local side = (s > 0) and "r" or "l"
        box({ 0.34, 0.18, 0.10 }, { s * lx, ly, hl - 0.05 }, head)
        box({ 0.34, 0.18, 0.10 }, { s * lx, ly, -hl + 0.05 }, tail)
        lights[#lights + 1] = { name = "headlight_" .. side, pos = { s * lx, ly, hl - 0.05 } }
        lights[#lights + 1] = { name = "taillight_" .. side, pos = { s * lx, ly, -hl + 0.05 } }
    end

    -- Four dark wheels at the hull's lower edge (baked into the instanced mesh,
    -- like buildCarMesh's withWheels path — ambient traffic has no physics wheels).
    local wr = H * 0.26
    local axleY = -H * 0.5 + wr
    local fz = L * 0.32
    local wxo = hw - 0.03
    for _, wx in ipairs({ wxo, -wxo }) do
        for _, wz in ipairs({ fz, -fz }) do
            box({ 0.12, wr * 2, wr * 2 }, { wx, axleY, wz }, tyre)
        end
    end

    return { parts = parts, lights = lights }
end

vehicle.fleet = {
    fleet_car(0, 1.80, 1.30, 4.2, { 0.72, 0.10, 0.10 }),   -- sedan (red)
    fleet_car(0, 1.80, 1.30, 4.2, { 0.10, 0.18, 0.52 }),   -- sedan (blue)
    fleet_car(0, 1.80, 1.30, 4.2, { 0.90, 0.90, 0.90 }),   -- sedan (white)
    fleet_car(1, 1.82, 1.45, 4.2, { 0.85, 0.78, 0.10 }),   -- hatchback (yellow)
    fleet_car(1, 1.82, 1.45, 4.2, { 0.10, 0.45, 0.30 }),   -- hatchback (green)
    fleet_car(1, 1.82, 1.45, 4.2, { 0.80, 0.40, 0.08 }),   -- hatchback (orange)
    fleet_car(2, 1.95, 1.70, 4.6, { 0.09, 0.09, 0.11 }),   -- SUV (black)
    fleet_car(2, 1.95, 1.70, 4.6, { 0.52, 0.53, 0.56 }),   -- SUV (silver)
    fleet_car(2, 1.95, 1.70, 4.6, { 0.30, 0.22, 0.14 }),   -- SUV (brown)
    fleet_car(3, 1.95, 1.60, 5.2, { 0.14, 0.30, 0.20 }),   -- pickup (green)
    fleet_car(4, 2.00, 2.10, 5.4, { 0.62, 0.60, 0.42 }),   -- van (tan)
    fleet_car(5, 2.40, 2.80, 6.6, { 0.20, 0.42, 0.55 }),   -- box truck (teal)
}

return vehicle
