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
  -- The wheel's DRAWN resting centre: one radius above the ground datum
  -- (y = -height/2), the same place the fleet bakes its wheels. The host
  -- (vehicle_spec.cpp) lifts the Jolt attach point by the spring's settle,
  -- so publishing the attach point here (the old "+ h156") double-counted
  -- the ride height and parked the player's car ~0.30 m in the air with its
  -- wheels dangling under the arches.
  local axleY = -d.height * 0.5 + r
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
-- slot's body. It used to be a BOX COMPOSITION (hull + cabin + glass slabs +
-- wheel boxes) — the "fancier low-poly car is a later task" note that made the
-- city read as old blocky traffic while the player's own car was the curved
-- generator car. THAT TASK IS DONE HERE: each slot is now built by `mesh.car`
-- — the SAME generator the car lab (car_lab.lua) and the drivable spec
-- (from_class above) use — over the class packages in vehicle_classes.lua.
--
-- The C++ reader (apps/citysim/scripting/vehicle_body.cpp) turns
-- `vehicle.fleet[slot+1]` into ONE vertex-coloured RenderMesh at LEVEL LOAD; a
-- Lua-free (Makefile) build keeps the C++ fleetCarMesh as a fallback, so the
-- streets are never data-dependent to be non-empty.
--
-- A recipe is:
--   body   = a Mesh (the mesh.car shell: painted body + lamps + tinted glass)
--   wheels = a Mesh (the baked wheelset, kept SEPARATE from the body — see below)
--   wheel_layout = { { pos=, radius=, width=, steered=, driven=, hand_brake= }, ... }
--   parts  = { { pos={x,y,z}, size={w,h,l}, color={r,g,b} }, ... }   -- boxes (legacy)
--   lights = { { name=, pos={x,y,z} }, ... }   -- named lamp ATTACHMENT markers
--
-- WHY body AND wheels, separately. mesh.car deliberately emits no wheel part
-- (real wheels are placed per-vehicle by the physics spec), so the fleet bakes
-- its own. Ambient instanced traffic has no physics wheels and wants them baked
-- IN; a car the player COMMANDEERS (ADR-0062) becomes a real Jolt Vehicle that
-- grows its own wheel entities, and wants the body WITHOUT them or it drives on
-- eight. Publishing both, cut from the same fitted geometry, is what lets the
-- promoted car be the same car — the alternative (one merged mesh) is why
-- promotion fell back to the retired C++ box body for so long.
--
-- `wheel_layout` is the same wheelset as DATA, post-fit: it is what the promoted
-- car's Jolt config is built from, so the simulated wheels land in the arches
-- drawn for them rather than at a second set of guessed numbers.
-- `lights` mark the front/rear lamp positions the emissive lamp pass draws
-- (city_render syncCarLamps). Car faces +Z; x>0 = right.
--
-- The fleet is 3 sedans, 3 hatchbacks, 3 SUVs, a pickup, a van, a box truck.
-- Its DIMENSIONS are no longer mirrored in C++: each recipe publishes its own
-- `size` from the class package and the sim adopts it (CitySim::setFleet). The
-- built-in C++ table survives only as the Lua-free build's fallback.

-- Ambient traffic is instanced in the hundreds inside the render bubble, so the
-- fleet runs a cheaper LOD than the hero car: "mid" keeps cut window apertures
-- and the shaped roofline; "low" would paint the glass on flat. Hero cars (the
-- player's, the lab's) stay "high".
local FLEET_LOD = "mid"

-- Build one fleet recipe from the real generator. `class_name` picks the
-- vehicle_classes package; `color` is the paint. THE CLASS IS THE SIZE: the
-- recipe publishes the package's own dimensions and the sim adopts them, so
-- there is one set of numbers per vehicle instead of two.
--
-- This replaces a hardcoded per-slot box (the C++ kFleet table) that every car
-- was scaled to fit. That box disagreed with the packages by up to 18% — and
-- because the disagreement differed per axis, the fit was NON-UNIFORM, which
-- turns a round wheel into an ellipse: the pickup's wheels were squashed 10%,
-- the hatchback's 8%. Nothing is scaled now; mesh.car builds at the class's
-- nominal size and that is what ships.
local function fleet_car(class_name, color)
    local c = classes.apply(class_name)
    local d = classes.dims(c)
    local car = mesh.car(forms.car_params(c, d, { color = color, lod = FLEET_LOD }))

    -- ONE opaque instanced mesh per slot: painted shell + lamp housings + the
    -- DARK-TINTED glass merged in. Ambient traffic cannot afford a transparent
    -- pass per car, and the tint (~0.16,0.20,0.24) reads as glass at traffic
    -- distance — exactly the role the box fleet's dark glass slabs played.
    local shell = { car.body }
    if car.lamp then shell[#shell + 1] = car.lamp end
    if car.glass then shell[#shell + 1] = car.glass end

    -- WHEELS. mesh.car deliberately emits no wheel part (real wheels are placed
    -- per-vehicle by the physics spec), so the fleet bakes its own — ROUND ones:
    -- a cylinder laid on its side, not the box the old fleet used, since the
    -- shell around them is now curved.
    --
    -- The GROUND DATUM is the thing to get right: the class package's box has
    -- its floor at y = -height/2, and the BODY floats above that by the ride
    -- height (h156). So a resting wheel's centre is one radius above the floor —
    -- NOT floor + radius + h156, which is the SUSPENSION ATTACHMENT point the
    -- drivable spec uses above (Jolt drops the wheel by its travel). Copying
    -- that formula parked every wheel a ride-height too high, tucked up inside
    -- the body with nothing touching the road.
    local r = c.wheel_diameter * 0.5
    local halfTrack = c.track * 0.5
    local axleY = -d.height * 0.5 + r
    local frontZ = d.length * 0.5 - d.front_overhang
    local rearZ = -(d.length * 0.5 - d.rear_overhang)
    local ww = math.max(0.18, c.track * 0.13)
    local tyre = { 0.04, 0.04, 0.05 }
    local wheelParts, placements = {}, {}
    for _, wx in ipairs({ halfTrack - ww * 0.5, -halfTrack + ww * 0.5 }) do
        for _, wz in ipairs({ frontZ, rearZ }) do
            -- cylinder(radius, height) stands on +Y; roll it onto the lateral
            -- axis so it spins the way the car drives. bake_height_color with
            -- one colour top and bottom is a flat tint (the mesh API has no
            -- separate vertex-paint call).
            local w = mesh.rotate_z(mesh.cylinder(r, ww), math.pi * 0.5)
            w = mesh.bake_height_color(w, tyre, tyre)
            wheelParts[#wheelParts + 1] = mesh.translate(w, { wx, axleY, wz })
            -- Front wheels steer; all four are driven (a loaded van still pulls
            -- away); the rears take the handbrake. Same roles the drivable spec
            -- above assigns, so a commandeered car handles like the player's own.
            placements[#placements + 1] =
                { x = wx, y = axleY, z = wz, front = (wz == frontZ) }
        end
    end

    -- No fit. Body and wheelset are built at the class's own size and stay there;
    -- `size` below tells the sim what that size is. They remain two meshes so a
    -- commandeered car can drop the baked wheels and grow physics ones.
    local body = mesh.recompute_normals(mesh.merge(shell))
    local wheels = mesh.recompute_normals(mesh.merge(wheelParts))

    -- The wheelset as DATA for the physics tier: exactly the wheels drawn above,
    -- round and at their true radius (nothing to correct for now that nothing is
    -- scaled). C++ converts these resting centres into Jolt suspension
    -- attachment points — see configFromBody.
    local layout = {}
    for _, p in ipairs(placements) do
        layout[#layout + 1] = {
            pos = { p.x, p.y, p.z },
            radius = r, width = ww,
            steered = p.front, driven = true, hand_brake = not p.front,
        }
    end

    local lights = {}
    for _, lt in ipairs(car.lights or {}) do
        lights[#lights + 1] = { name = lt.name, pos = lt.pos }
    end

    return {
        body = body, wheels = wheels, wheel_layout = layout, lights = lights,
        -- THE CATALOGUE ENTRY the sim adopts: how much road this car occupies.
        -- Car-following gaps, parking bays, kinematic collider proxies and the
        -- promoted car's chassis all read it, so they agree with the drawn body
        -- by construction rather than by a transcribed table.
        size = { d.width, d.height, d.length },
        class = class_name,
    }
end

-- A slot is a CLASS plus a PAINT. Adding a vehicle is a line here (and a package
-- in vehicle_classes.lua) — no C++ table to keep in step.
local FLEET_SLOTS = {
    { class = "sedan",     color = { 0.72, 0.10, 0.10 } },   -- sedan (red)
    { class = "sedan",     color = { 0.10, 0.18, 0.52 } },   -- sedan (blue)
    { class = "sedan",     color = { 0.90, 0.90, 0.90 } },   -- sedan (white)
    { class = "hatchback", color = { 0.85, 0.78, 0.10 } },   -- hatchback (yellow)
    { class = "hatchback", color = { 0.10, 0.45, 0.30 } },   -- hatchback (green)
    { class = "hatchback", color = { 0.80, 0.40, 0.08 } },   -- hatchback (orange)
    { class = "jeep",      color = { 0.09, 0.09, 0.11 } },   -- SUV (black)
    { class = "jeep",      color = { 0.52, 0.53, 0.56 } },   -- SUV (silver)
    { class = "jeep",      color = { 0.30, 0.22, 0.14 } },   -- SUV (brown)
    { class = "pickup",    color = { 0.14, 0.30, 0.20 } },   -- pickup (green)
    { class = "van",       color = { 0.62, 0.60, 0.42 } },   -- van (tan)
    { class = "box_truck", color = { 0.20, 0.42, 0.55 } },   -- box truck (teal)
}

-- The fleet is DESCRIPTION up front and GEOMETRY on demand.
--
-- Each slot carries its catalogue entry — class and size — as plain data, which
-- is all the SIM needs (follow gaps, parking bays, collider proxies) and costs
-- nothing to read. The body itself arrives only when `build()` is called, which
-- is where the real expense is: twelve mesh.car shells. A headless run that
-- wants correct car SIZES must not have to pay for twelve car MESHES to get
-- them — building them eagerly here made every headless city build do exactly
-- that.
vehicle.fleet = {}
for i, slot in ipairs(FLEET_SLOTS) do
    local c = classes.apply(slot.class)
    local d = classes.dims(c)
    vehicle.fleet[i] = {
        class = slot.class,
        size = { d.width, d.height, d.length },
        build = function() return fleet_car(slot.class, slot.color) end,
    }
end

return vehicle
