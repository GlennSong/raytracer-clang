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
    -- The forgiving handling defaults live in PhysicsWorld::VehicleConfig
    -- (physics_world.h: roll cone 65 deg, anti-roll bars off, yaw assist 3/s);
    -- a recipe may override any of them (nil = keep the default).
    max_roll_deg = opts.max_roll_deg,
    anti_roll = opts.anti_roll,
    yaw_assist = opts.yaw_assist,
    grip = opts.grip,               -- the tyres' peak lateral grip (default 1.7)
    -- real aero drag (a top speed near 205 km/h, not linear body damping) and quick shifts: Jolt's
    -- default gearbox cut the drive for ~1.3 s per upshift -- a 3.5 s stall at 105-115 km/h on the freeway
    drag_area = opts.drag_area or 2.0,
    shift_time = 0.2, clutch_time = 0.15, shift_latency = 0.25,
    drive = opts.drive,             -- nil: the wheels' driven flags (all four: AWD)
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

-- DRIVABLE KIT BODIES (ADR-0141): any vehicle_kit.lua spec as a player car. `h` is the handling: drive
-- ("fwd" | "rwd" | "awd" | "4wd" part-time), mass, drag, gears, suspension, grip, differentials.
local KIT_TINT = { trim = { 0.04, 0.04, 0.045 }, gasket = { 0.025, 0.025, 0.028 }, grille = { 0.06, 0.06, 0.065 },
                   chrome = { 0.72, 0.73, 0.75 }, lamp = { 0.95, 0.95, 0.90 }, lamp_red = { 0.75, 0.06, 0.05 },
                   chassis = { 0.07, 0.07, 0.08 }, box = { 0.90, 0.90, 0.88 }, bed = { 0.08, 0.08, 0.09 },
                   interior = { 0.35, 0.26, 0.18 }, sign = { 1.0, 0.85, 0.2 } }
local KIT_CABIN_TINT = { liner = { 0.45, 0.43, 0.40 }, carpet = { 0.10, 0.10, 0.11 }, seat = { 0.18, 0.17, 0.16 },
                         dash = { 0.08, 0.08, 0.09 }, steer = { 0.05, 0.05, 0.05 } }
local function kit_drivable(specName, opts, h)
  opts = opts or {}
  local kit = require "vehicle_kit"
  local P = kit.SPECS[specName]()
  if opts.color then P.color = opts.color end
  local car = kit.build(P, 0)
  local shell = {}
  for name, part in pairs(car.parts) do
    if name ~= "glass" then
      local col = name == "body" and P.color or (KIT_TINT[name] or { 0.4, 0.4, 0.4 })
      shell[#shell + 1] = mesh.bake_height_color(part, col, col)
    end
  end
  local parts = {}
  if car.parts.glass then
    parts[#parts + 1] = { mesh = car.parts.glass, albedo = { 1, 1, 1 }, metallic = 0.0, roughness = 0.06, opacity = 0.30 }
  end
  local cabin = {}
  for name, part in pairs(car.cabin or {}) do
    local col = KIT_CABIN_TINT[name] or { 0.3, 0.3, 0.3 }
    cabin[#cabin + 1] = mesh.bake_height_color(part, col, col)
  end
  if #cabin > 0 then
    parts[#parts + 1] = { mesh = mesh.merge(cabin), albedo = { 1, 1, 1 }, metallic = 0.0, roughness = 0.9 }
  end
  local lights = {}
  for _, l in ipairs(car.lights) do lights[#lights + 1] = l end
  lights[#lights + 1] = { name = "driver_seat", pos = car.driver_seat }
  local wheels = {}
  for _, w in ipairs(car.wheels) do
    wheels[#wheels + 1] = { x = w.pos[1], y = w.pos[2], z = w.pos[3], steered = w.front, driven = true,
                            hand_brake = not w.front }
  end
  local W, H, L = car.size[1], car.size[2], car.size[3]
  local mass = opts.mass or h.mass or math.floor(L * W * H * 120)
  return {
    body = mesh.recompute_normals(mesh.merge(shell)),
    albedo = { 1, 1, 1 }, metallic = h.metallic or 0.5, roughness = h.roughness or 0.4,
    parts = parts,
    lights = lights,
    chassis = { half = { W * 0.5, H * 0.5, L * 0.5 } },
    mass = mass,
    com_offset = h.com_offset or -(0.23 + 0.22 * (H / 1.45)),
    engine_torque = opts.engine_torque or math.floor(mass * (h.torque_per_kg or 0.46)),
    max_rpm = h.max_rpm or 6000,
    max_steer_deg = h.max_steer_deg or 30,
    brake_torque = math.floor(mass * 1.15),
    hand_brake_torque = math.floor(mass * 2.9),
    grip = h.grip,
    -- the collision's underside follows the drawn body: its floor at the body's ground clearance (never
    -- under the street rig's 0.22 kerb lift), and an off-roader's nose and tail cut to its approach and
    -- departure angles (physics_world.h) -- a square box caught rocks the tyres could climb
    floor_clearance = math.max(0.22, P.clear or 0),
    approach_deg = h.approach_deg, departure_deg = h.departure_deg,
    max_roll_deg = h.max_roll_deg,
    drive = h.drive,
    axle_lsd = h.axle_lsd, center_lsd = h.center_lsd, traction_split = h.traction_split,
    drag_area = h.drag_area or 2.0,
    shift_time = 0.2, clutch_time = 0.15, shift_latency = 0.25,
    gear_ratios = h.gear_ratios,
    suspension = h.suspension,
    wheel = { radius = P.wheel_r, width = h.wheel_width or P.wheel_w },
    wheels = wheels,
  }
end
vehicle.kit_drivable = kit_drivable

-- THE OFF-ROADER (#41): part-time four-wheel drive (2WD rear by default, Z / D-pad down engages 4WD), long
-- soft travel, near-locking axles, a fixed 50/50 split in 4WD that hands a hanging axle's torque to the one on
-- the ground (traction_split), sticky tyres, low gears.
local OFFROAD = {
  drive = "4wd", mass = 2300, torque_per_kg = 0.55, max_rpm = 5200, max_steer_deg = 32, com_offset = -0.62,
  grip = 2.3, axle_lsd = 1.15, center_lsd = 1e30, traction_split = true, drag_area = 2.8, gear_ratios = { 3.9, 2.4, 1.6, 1.15, 0.9 },
  suspension = { min = 0.0, max = 0.40, freq = 1.3, damp = 0.5, rest_drop = 0.30 }, wheel_width = 0.32,
  approach_deg = 42, departure_deg = 36,
  metallic = 0.35, roughness = 0.5,
}
-- a light off-road rig for the jeep and pickup (part-time 4WD, more travel than the street)
local function trail(approach, departure)
  return { drive = "4wd", axle_lsd = 1.2, center_lsd = 1e30, traction_split = true, grip = 2.1, drag_area = 2.5,
           suspension = { min = 0.0, max = 0.30, freq = 1.5, damp = 0.55, rest_drop = 0.23 },
           approach_deg = approach, departure_deg = departure }
end
local KIT_HANDLING = {
  sedan       = { drive = "fwd", drag_area = 2.0 },
  taxi        = { drive = "fwd", drag_area = 2.0 },
  hatchback   = { drive = "fwd", drag_area = 1.9 },
  convertible = { drive = "rwd", drag_area = 1.8, max_steer_deg = 32 },
  suv         = { drive = "awd", drag_area = 2.4 },
  jeep        = trail(38, 32),
  pickup      = trail(28, 24),
  step_van    = { drive = "rwd", drag_area = 3.2, torque_per_kg = 0.40 },
  small_truck = { drive = "rwd", drag_area = 3.4, torque_per_kg = 0.40 },
  offroad     = OFFROAD,
}
for name, h in pairs(KIT_HANDLING) do
  vehicle["kit_" .. name] = function(seed, opts) return kit_drivable(name, opts, h) end
end
function vehicle.offroad(seed, opts) return kit_drivable("offroad", opts, OFFROAD) end

-- THE DRIVABLE CATALOGUE: what the player can drop (VehicleSystem: - and = pick, N drops). `drive` is the
-- label the picker shows -- test_vehicle_body checks it matches the spec's drivetrain.
vehicle.drivable = {
  { recipe = "offroad",          label = "Off-roader",           drive = "4wd" },
  { recipe = "kit_jeep",         label = "Jeep",                 drive = "4wd" },
  { recipe = "kit_pickup",       label = "Pickup",               drive = "4wd" },
  { recipe = "kit_suv",          label = "SUV",                  drive = "awd" },
  { recipe = "kit_sedan",        label = "Sedan",                drive = "fwd" },
  { recipe = "kit_hatchback",    label = "Hatchback",            drive = "fwd" },
  { recipe = "kit_taxi",         label = "Taxi",                 drive = "fwd" },
  { recipe = "kit_convertible",  label = "Convertible",          drive = "rwd" },
  { recipe = "kit_step_van",     label = "Parcel van",           drive = "rwd" },
  { recipe = "kit_small_truck",  label = "Box truck",            drive = "rwd" },
  { recipe = "sedan",            label = "Sedan (classic)",      drive = "awd" },
  { recipe = "pickup",           label = "Pickup (classic)",     drive = "awd" },
  { recipe = "van",              label = "Van (classic)",        drive = "awd" },
}

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
-- The fleet is 3 sedans, 3 hatchbacks, 3 SUVs, a pickup, a van, a box truck
-- and a CITY BUS (the transit routes used to be driven by sedans).
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

    -- The opaque shell: painted body + lamp housings. The GLASS and the
    -- INTERIOR are their own parts for every slot (fleet v2, Glenn: "the opaque
    -- with reflection and the swap to hero car for ones nearby the player"): the
    -- city draws the glass OPAQUE with a reflective glass material for traffic
    -- at large -- no transparent pass per car -- and, for the few cars nearest
    -- the player, CLEAR with the interior and a driver behind it.
    local shell = { car.body }
    if car.lamp then shell[#shell + 1] = car.lamp end
    -- A BUS IS MEANT TO BE SEEN INTO (Glenn: "we should see the npcs sitting
    -- on the bus. There should be a driver"). Its saloon goes into the opaque
    -- shell and its glass stays a separate, CLEAR part; 24 buses can afford the
    -- transparent pass that hundreds of cars cannot.
    local seeInto = (c.form == "bus")
    if seeInto and car.interior then shell[#shell + 1] = car.interior end

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

    -- mesh.car publishes every mount point under `lights`: lamps, and the
    -- seats (the driver's hip point, and each passenger seat's on a bus).
    local lights, seats, doors, driver_seat = {}, {}, {}, nil
    for _, lt in ipairs(car.lights or {}) do
        if lt.name == "seat" then
            seats[#seats + 1] = lt.pos
        elseif lt.name == "door" then
            doors[#doors + 1] = lt.pos
        elseif lt.name == "driver_seat" then
            driver_seat = lt.pos
        else
            lights[#lights + 1] = { name = lt.name, pos = lt.pos }
        end
    end

    return {
        body = body, wheels = wheels, wheel_layout = layout, lights = lights,
        -- THE CATALOGUE ENTRY the sim adopts: how much road this car occupies.
        -- Car-following gaps, parking bays, kinematic collider proxies and the
        -- promoted car's chassis all read it, so they agree with the drawn body
        -- by construction rather than by a transcribed table.
        size = { d.width, d.height, d.length },
        class = class_name,
        glass = car.glass,
        -- the saloon is in the shell of a see-into vehicle; the others carry it
        -- apart, drawn only while they are near the player
        interior = (not seeInto) and car.interior or nil,
        see_into = seeInto,
        seats = seeInto and seats or nil,
        doors = seeInto and doors or nil,
        driver_seat = driver_seat,
    }
end

-- FLEET V2 (ADR-0139): a slot built by vehicle_kit.lua on the poly.* kit -- faceted, with a real gasket
-- round each window, grille and lamp lenses. The same recipe fields as fleet_car: one opaque vertex-
-- coloured body, the wheelset apart, the glass as its own part (drawn opaque and reflective), lamp
-- markers and the driver's seat. No cabin yet: until it exists the city keeps these cars' glass opaque.
local kit = require "vehicle_kit"
local KIT_TINT = {
  glass = { 0.16, 0.20, 0.24 }, trim = { 0.04, 0.04, 0.045 }, gasket = { 0.025, 0.025, 0.028 },
  grille = { 0.06, 0.06, 0.065 }, chrome = { 0.72, 0.73, 0.75 }, lamp = { 0.95, 0.95, 0.90 },
  lamp_red = { 0.75, 0.06, 0.05 }, interior = { 0.35, 0.26, 0.18 }, bed = { 0.08, 0.08, 0.09 },
  box = { 0.90, 0.90, 0.88 }, chassis = { 0.07, 0.07, 0.08 }, door = { 0.80, 0.81, 0.82 },
  tank = { 0.78, 0.79, 0.80 }, trailer = { 0.86, 0.87, 0.88 }, sign = { 1.0, 0.85, 0.2 },
}
local KIT_CABIN = {
  liner = { 0.55, 0.53, 0.50 }, carpet = { 0.10, 0.10, 0.11 }, seat = { 0.22, 0.20, 0.19 },
  dash = { 0.08, 0.08, 0.09 }, steer = { 0.05, 0.05, 0.05 },
}
local function kit_car(spec, color)
  local P = kit.SPECS[spec]()
  local car = kit.build(P, 0)
  local shell = {}
  for name, part in pairs(car.parts) do
    if name ~= "glass" then
      local col = name == "body" and color or (KIT_TINT[name] or { 0.5, 0.5, 0.5 })
      shell[#shell + 1] = mesh.bake_height_color(part, col, col)
    end
  end
  local glass = car.parts.glass and mesh.bake_height_color(car.parts.glass, KIT_TINT.glass, KIT_TINT.glass) or nil
  local w = kit.wheel(P.wheel_r, P.wheel_w, 0)
  local tyre = mesh.bake_height_color(w.tyre, { 0.05, 0.05, 0.055 }, { 0.05, 0.05, 0.055 })
  local rim = mesh.bake_height_color(w.rim, { 0.72, 0.73, 0.75 }, { 0.72, 0.73, 0.75 })
  local rimL = mesh.rotate_y(rim, math.pi)
  local wheelParts, layout = {}, {}
  for _, wh in ipairs(car.wheels) do
    wheelParts[#wheelParts + 1] = mesh.translate(tyre, wh.pos)
    wheelParts[#wheelParts + 1] = mesh.translate(wh.pos[1] > 0 and rim or rimL, wh.pos)
    if wh.dual then
      wheelParts[#wheelParts + 1] = mesh.translate(tyre, { wh.pos[1] - (wh.pos[1] > 0 and 1 or -1) * (wh.width + 0.03), wh.pos[2], wh.pos[3] })
    end
    layout[#layout + 1] = { pos = wh.pos, radius = wh.radius, width = wh.width,
                            steered = wh.front, driven = true, hand_brake = not wh.front }
  end
  -- the cabin: a near car shows it through clear glass; an OPEN car (the convertible) always does, so
  -- its cabin rides in the shell and it is a see-into vehicle like the bus
  local cabinParts = {}
  for name, part in pairs(car.cabin or {}) do
    local col = KIT_CABIN[name] or { 0.3, 0.3, 0.3 }
    cabinParts[#cabinParts + 1] = mesh.bake_height_color(part, col, col)
  end
  local open = P.open_top or false
  local interior = nil
  if #cabinParts > 0 then
    if open then for _, cp in ipairs(cabinParts) do shell[#shell + 1] = cp end
    else interior = mesh.merge(cabinParts) end
  end
  return {
    interior = interior,
    body = mesh.recompute_normals(mesh.merge(shell)),
    wheels = mesh.merge(wheelParts),
    wheel_layout = layout,
    lights = car.lights,
    size = car.size,
    class = spec,
    glass = glass,
    see_into = open,
    driver_seat = car.driver_seat,
  }
end
-- the catalogue size without building the meshes' materials: the faceted build is cheap, so it is simply
-- measured (a truck's box or a pickup's bed is part of its length)
local function kit_size(spec)
  local car = kit.build(kit.SPECS[spec](), 0)
  return car.size
end

-- THE CLASSIC SET: today's mesh.car bodies, kept (Glenn: "it would be nice and funny if we kept some of
-- the current designs as a 'classic' set"). vehicle.classic[i] has the same shape as vehicle.fleet[i].
local CLASSIC_SLOTS = {
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
    { class = "bus",       color = { 0.86, 0.62, 0.08 } },   -- CITY BUS (municipal yellow)
}

-- THE FLEET (v2): kit bodies for the cars and trucks; the city BUS stays the mesh.car bus Glenn likes,
-- in the 13th slot (the sim's built-in table types slot 13 as the bus). Semis wait for articulated
-- trailers (a 22 m rigid body would swing its trailer through every kerb).
local FLEET_SLOTS = {
    { kit = "sedan",       color = { 0.62, 0.06, 0.07 } },   -- sedan (red)
    { kit = "sedan",       color = { 0.10, 0.18, 0.52 } },   -- sedan (blue)
    { kit = "sedan",       color = { 0.88, 0.88, 0.88 } },   -- sedan (white)
    { kit = "hatchback",   color = { 0.85, 0.72, 0.10 } },   -- hatchback (yellow)
    { kit = "hatchback",   color = { 0.10, 0.45, 0.30 } },   -- hatchback (green)
    { kit = "suv",         color = { 0.09, 0.09, 0.11 } },   -- SUV (black)
    { kit = "suv",         color = { 0.52, 0.53, 0.56 } },   -- SUV (silver)
    { kit = "jeep",        color = { 0.24, 0.30, 0.18 } },   -- jeep (olive)
    { kit = "convertible", color = { 0.80, 0.12, 0.10 } },   -- convertible (red)
    { kit = "pickup",      color = { 0.14, 0.30, 0.20 } },   -- pickup (green)
    { kit = "step_van",    color = { 0.36, 0.22, 0.10 } },   -- parcel step van (brown)
    { kit = "small_truck", color = { 0.90, 0.90, 0.88 } },   -- small box truck (white cab)
    { class = "bus",       color = { 0.86, 0.62, 0.08 } },   -- CITY BUS (municipal yellow), mesh.car
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
local function catalogue(slots)
    local out = {}
    for i, slot in ipairs(slots) do
        if slot.kit then
            out[i] = {
                class = slot.kit,
                size = kit_size(slot.kit),
                build = function() return kit_car(slot.kit, slot.color) end,
            }
        else
            local c = classes.apply(slot.class)
            local d = classes.dims(c)
            out[i] = {
                class = slot.class,
                size = { d.width, d.height, d.length },
                build = function() return fleet_car(slot.class, slot.color) end,
            }
        end
    end
    return out
end
-- THE MIXED FLEET (Glenn: "get the new cars into island 8 along with the classic cars"): the kit cars and
-- trucks, then the classic ones, then the one city bus -- 25 slots, every body on the road at once.
local MIXED_SLOTS = {}
for _, s in ipairs(FLEET_SLOTS) do if s.class ~= "bus" then MIXED_SLOTS[#MIXED_SLOTS + 1] = s end end
for _, s in ipairs(CLASSIC_SLOTS) do if s.class ~= "bus" then MIXED_SLOTS[#MIXED_SLOTS + 1] = s end end
for _, s in ipairs(FLEET_SLOTS) do if s.class == "bus" then MIXED_SLOTS[#MIXED_SLOTS + 1] = s end end

-- The named fleets. A level picks one with `citysim.fleet` (level_loader points vehicle.fleet at it);
-- without one it gets the kit fleet.
vehicle.fleet_kit = catalogue(FLEET_SLOTS)
vehicle.classic = catalogue(CLASSIC_SLOTS)
vehicle.fleet_mixed = catalogue(MIXED_SLOTS)
vehicle.fleet = vehicle.fleet_kit

return vehicle
