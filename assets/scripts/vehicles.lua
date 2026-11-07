-- Procedural vehicle bodies (ADR-0059) -- authored in Lua over the procgen builders. Defines a global `vehicle` table:
--
--   vehicle.kit_<spec> (seed, opts) -> spec   the kit's drivable cars (vehicle_kit.lua bodies, ADR-0139)
--   vehicle.sedan / hatchback / jeep / pickup / van   the same, by the names levels spawn them with
--   vehicle.offroad, motorcycle, moped, bicycle; vehicle.drivable (the picker); vehicle.fleet* (traffic)
--
-- A "spec" is a plain table the C++ VehicleSpec reader (vehicle_spec.cpp) consumes: a `body` Mesh built with mesh.*
-- plus handling/physics parameters and a wheel layout. Hosts load this once so `vehicle` is available, then run a
-- small chunk like `return vehicle.sedan(seed, {color={0,0.4,0.8}})`. Pure and deterministic per seed. (The classic
-- mesh.car bodies over vehicle_classes/vehicle_forms retired 2026-11-06, ADR-0349.)
--
-- Convention: the car faces +Z. HEADLIGHTS (pale) are at +Z, TAILLIGHTS (red) at
-- -Z, so the front reads at a glance and "W = drive toward the headlights".

vehicle = {}


-- LAMPS IN THE BODY (ADR-0342, #186): a lens is a part of the body tagged with its ROLE in its texture slot
-- (engine/vehicle_lamps.h LensRole: u = -1000 - role), and the renderer lights the tagged lenses by the vehicle's
-- lamps this step -- the lamp is where the body put it, and goes where the body goes. The kit names its lens parts
-- by these materials; a hand-built body tags its own.
local LENS_ROLE = { lamp = 1, lamp_red = 2, lamp_amber = 3, lamp_marker = 4 }
local function lens_tag(m, mat)
  local r = LENS_ROLE[mat]
  return r and mesh.set_uv(m, -1000 - r, 0) or m
end
local LENS_TINT = { lamp = { 0.95, 0.95, 0.90 }, lamp_red = { 0.75, 0.06, 0.05 },
                    lamp_amber = { 0.85, 0.42, 0.05 }, lamp_marker = { 0.85, 0.42, 0.05 } }
local function lens(m, mat) local c = LENS_TINT[mat]; return lens_tag(mesh.paint(m, c), mat) end

-- THE KIT's FINISHES: a part's colour by its material name (the paint is the slot's), and the drivable car's cabin
local KIT_TINT = {
  glass = { 0.16, 0.20, 0.24 }, trim = { 0.04, 0.04, 0.045 }, gasket = { 0.025, 0.025, 0.028 },
  grille = { 0.06, 0.06, 0.065 }, chrome = { 0.72, 0.73, 0.75 }, lamp = { 0.95, 0.95, 0.90 },
  lamp_red = { 0.75, 0.06, 0.05 }, interior = { 0.35, 0.26, 0.18 }, bed = { 0.08, 0.08, 0.09 },
  box = { 0.90, 0.90, 0.88 }, chassis = { 0.07, 0.07, 0.08 }, door = { 0.80, 0.81, 0.82 },
  tank = { 0.78, 0.79, 0.80 }, trailer = { 0.86, 0.87, 0.88 }, sign = { 1.0, 0.85, 0.2 },
  beacon = { 0.80, 0.06, 0.05 },
}
local KIT_CABIN_TINT = { liner = { 0.45, 0.43, 0.40 }, carpet = { 0.10, 0.10, 0.11 }, seat = { 0.18, 0.17, 0.16 },
                         dash = { 0.08, 0.08, 0.09 }, steer = { 0.05, 0.05, 0.05 } }
-- A KIT BODY's HANDLING (item 23, #117: one handling builder): `P` the kit spec, `car` its kit.build(P, 0), `h`
-- the handling table (KIT_HANDLING). The drivable spec (kit_drivable) and the fleet slot of the same kit
-- (catalogue) both publish this, so a commandeered traffic car drives as the one the player drops.
local function kit_handling(P, car, h, opts)
  opts = opts or {}
  local wheels = {}
  for _, w in ipairs(car.wheels) do
    wheels[#wheels + 1] = { x = w.pos[1], y = w.pos[2], z = w.pos[3], steered = w.front, driven = true,
                            hand_brake = not w.front }
  end
  local W, H, L = car.size[1], car.size[2], car.size[3]
  local mass = opts.mass or h.mass or math.floor(L * W * H * 120)
  return {
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
    downforce = h.downforce or 0.4,   -- pressed onto the road at speed
    shift_time = 0.2, clutch_time = 0.15, shift_latency = 0.25,
    gear_ratios = h.gear_ratios,
    suspension = h.suspension,
    anti_roll = h.anti_roll,
    drift = h.drift,
    wheel = { radius = P.wheel_r, width = h.wheel_width or P.wheel_w },
    wheels = wheels,
    open_seat = P.open_top or false,   -- a convertible: nothing over the driver, so a crash throws them (ADR-0252)
  }
end
local function kit_drivable(specName, opts, h)
  opts = opts or {}
  local kit = require "vehicle_kit"
  local P = kit.SPECS[specName]()
  if opts.color then P.color = opts.color end
  local car = kit.build(P, 0)
  local shell = {}
  for name, part in pairs(car.parts) do
    if name ~= "glass" then
      local col = name == "body" and P.color or (KIT_TINT[name] or LENS_TINT[name] or { 0.4, 0.4, 0.4 })
      shell[#shell + 1] = lens_tag(mesh.paint(part, col), name)
    end
  end
  local parts = {}
  if car.parts.glass then
    parts[#parts + 1] = { mesh = car.parts.glass, albedo = { 1, 1, 1 }, metallic = 0.0, roughness = 0.06, opacity = 0.30 }
  end
  local cabin = {}
  for name, part in pairs(car.cabin or {}) do
    local col = KIT_CABIN_TINT[name] or { 0.3, 0.3, 0.3 }
    cabin[#cabin + 1] = mesh.paint(part, col)
  end
  if #cabin > 0 then
    parts[#parts + 1] = { mesh = mesh.merge(cabin), albedo = { 1, 1, 1 }, metallic = 0.0, roughness = 0.9 }
  end
  local lights = {}
  for _, l in ipairs(car.lights) do lights[#lights + 1] = l end
  lights[#lights + 1] = { name = "driver_seat", pos = car.driver_seat }
  local spec = kit_handling(P, car, h, opts)
  spec.body = mesh.recompute_normals(mesh.merge(shell))
  spec.albedo = { 1, 1, 1 }
  spec.metallic = h.metallic or 0.5
  spec.roughness = h.roughness or 0.4
  spec.parts = parts
  spec.lights = lights
  return spec
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
-- A STREET RIG of the car's own: springs at `freq` Hz with `max` m of travel and damping ratio `damp`, the wheel's rest
-- drop the travel less the spring's static sag (g / (2 pi f)^2) so the car sits at its drawn height on any spring
local function street(freq, max, damp)
  local sag = 9.81 / (2 * math.pi * freq) ^ 2
  return { min = 0.0, max = max, freq = freq, damp = damp or 0.55, rest_drop = max - sag }
end
-- THE HANDLING TABLE (Glenn, 2026-11-06: "different cars should have different forms of suspension ... so a tall
-- delivery truck feels different from driving an SUV different from driving a sedan ... codified in Lua or something
-- so we can tune it"). Per kit car: the drivetrain and drag, its springs (street(freq, travel, damping); none: the
-- street rig, 1.8 Hz / 0.22 m), its anti-roll bars (N/m, one per axle; stiffer leans less), and its drift --
-- rear_grip (the rear tyres' sideways grip in a slide: lower slides more), slip_deg (the angle full lock holds),
-- min_speed (m/s it starts from). The rest (mass, torque, brakes) follows the body in kit_handling.
local KIT_HANDLING = {
  sedan       = { drive = "fwd", drag_area = 2.0, anti_roll = 3000, drift = { rear_grip = 0.35, slip_deg = 32 } },
  taxi        = { drive = "fwd", drag_area = 2.0, anti_roll = 3000, drift = { rear_grip = 0.35, slip_deg = 32 } },
  hatchback   = { drive = "fwd", drag_area = 1.9, anti_roll = 3500, drift = { rear_grip = 0.42, slip_deg = 28 } },
  convertible = { drive = "rwd", drag_area = 1.8, max_steer_deg = 32, suspension = street(2.0, 0.20, 0.6),
                  anti_roll = 5000, drift = { rear_grip = 0.30, slip_deg = 36, min_speed = 10 } },
  suv         = { drive = "awd", drag_area = 2.4, suspension = street(1.5, 0.26, 0.55), anti_roll = 3000,
                  drift = { rear_grip = 0.48, slip_deg = 24 } },
  jeep        = trail(38, 32),
  pickup      = trail(28, 24),
  step_van    = { drive = "rwd", drag_area = 3.2, torque_per_kg = 0.40, suspension = street(1.4, 0.26, 0.5),
                  anti_roll = 2500, drift = { rear_grip = 0.65, slip_deg = 14, min_speed = 13 } },
  -- (the box truck's tall box: a stiffer spring and bars than the van's -- at 1.3 Hz it went over on a kerb at 29 km/h)
  small_truck = { drive = "rwd", drag_area = 3.4, torque_per_kg = 0.40, suspension = street(1.6, 0.24, 0.55),
                  anti_roll = 6000, drift = { rear_grip = 0.70, slip_deg = 12, min_speed = 13 } },
  offroad     = OFFROAD,
  bus         = { drive = "rwd", drag_area = 6.0, torque_per_kg = 0.32, max_steer_deg = 38, suspension = street(1.2, 0.30, 0.5),
                  anti_roll = 6000, drift = { rear_grip = 0.85, slip_deg = 8, min_speed = 14 } },
}
KIT_HANDLING.jeep.anti_roll, KIT_HANDLING.jeep.drift = 1500, { rear_grip = 0.45, slip_deg = 26 }
KIT_HANDLING.pickup.anti_roll, KIT_HANDLING.pickup.drift = 1800, { rear_grip = 0.42, slip_deg = 28 }
OFFROAD.anti_roll, OFFROAD.drift = 1200, { rear_grip = 0.5, slip_deg = 22 }
for name, h in pairs(KIT_HANDLING) do
  vehicle["kit_" .. name] = function(seed, opts) return kit_drivable(name, opts, h) end
end
function vehicle.offroad(seed, opts) return kit_drivable("offroad", opts, OFFROAD) end
-- THE OLD NAMES (levels spawn the player's car as "sedan", "hatchback", "jeep", "pickup" or "van"): the kit's cars
-- since the classic mesh.car builder retired (ADR-0349)
function vehicle.sedan(seed, opts) return vehicle.kit_sedan(seed, opts) end
function vehicle.hatchback(seed, opts) return vehicle.kit_hatchback(seed, opts) end
function vehicle.jeep(seed, opts) return vehicle.kit_jeep(seed, opts) end
function vehicle.pickup(seed, opts) return vehicle.kit_pickup(seed, opts) end
function vehicle.van(seed, opts) return vehicle.kit_step_van(seed, opts) end

-- TWO WHEELS (ADR-0235; Glenn: "skateboards, scooters, mopeds, motorcycles, bikes"): Jolt's MotorcycleController
-- leans these into a turn and holds them up (physics_world.h `two_wheeler`). Built from boxes on the centre line,
-- +z ahead, the wheels' drawn centres at -H/2 + r. Engines are tiny in torque because Jolt multiplies them by the
-- gear and a 3.42 final drive: the ratios set the top speed (rpm / 60 * 2 pi r / (gear * 3.42)).
local function part(m, col) return mesh.paint(m, col) end
local function at(m, x, y, z) return mesh.translate(m, { x, y, z }) end
local function two_wheeler(kind, opts)
  opts = opts or {}
  local K = ({
    motorcycle = { r = 0.32, w = 0.13, base = 1.45, H = 0.72, mass = 220, torque = 100, rpm = 9000, steer = 28,
                   lean = 50, gears = { 5.0, 3.6, 2.8, 2.3, 2.0 }, color = { 0.75, 0.08, 0.06 },
                   seat = { grip = { 0.30, 0.14, 0.67 }, foot = { 0.19, -0.44, 0.08 }, lean = 28, bars = true } },
    moped      = { r = 0.25, w = 0.10, base = 1.2, H = 0.62, mass = 130, torque = 80, rpm = 6000, steer = 32,
                   lean = 42, gears = { 6.0, 4.5, 3.3 }, color = { 0.2, 0.55, 0.62 },
                   seat = { grip = { 0.30, 0.14, 0.55 }, foot = { 0.12, -0.42, 0.30 }, lean = 8, bars = true } },
    -- (the legs as an engine: Jolt's automatic shifts up at 4000 rpm, so a 1500 rpm "engine" never left first;
    -- rpm and ratios both x4 keep the top speed and let it change gear)
    bicycle    = { r = 0.34, w = 0.04, base = 1.05, H = 0.75, mass = 90, torque = 60, rpm = 6000, steer = 35,
                   lean = 40, gears = { 16.0, 11.2, 8.8, 7.0 }, color = { 0.15, 0.35, 0.75 },
                   seat = { grip = { 0.24, 0.08, 0.60 }, foot = { 0.10, -0.55, 0.04 }, lean = 35, bars = true, pedals = true, crank = 0.17 } },
  })[kind]
  local col = opts.color or K.color
  local dark, metal, black = { 0.12, 0.12, 0.13 }, { 0.65, 0.66, 0.68 }, { 0.05, 0.05, 0.05 }
  local zf, zr, wy = K.base * 0.5, -K.base * 0.5, -K.H * 0.5 + K.r
  local shell = {}
  local function add(m, c) shell[#shell + 1] = part(m, c) end
  if kind == "bicycle" then
    -- a diamond frame of thin tubes: top tube, down tube, seat tube, stays, fork, bars, saddle
    add(at(mesh.box({ 0.035, 0.035, 0.62 }), 0, wy + 0.42, 0.05), col)
    add(at(mesh.rotate_x(mesh.box({ 0.035, 0.035, 0.7 }), 0.62), 0, wy + 0.2, 0.18), col)
    add(at(mesh.box({ 0.035, 0.5, 0.035 }), 0, wy + 0.22, -0.22), col)
    add(at(mesh.box({ 0.03, 0.03, 0.32 }), 0, wy + 0.02, zr + 0.16), col)
    add(at(mesh.rotate_x(mesh.box({ 0.03, 0.62, 0.03 }), -0.3), 0, wy + 0.28, zf - 0.09), metal)
    add(at(mesh.box({ 0.52, 0.03, 0.03 }), 0, wy + 0.6, zf - 0.18), metal)
    add(at(mesh.box({ 0.12, 0.05, 0.24 }), 0, wy + 0.5, -0.24), black)
  else
    -- a motorcycle / moped: frame, engine, tank or leg shield, seat, fork, bars, mudguards
    add(at(mesh.box({ 0.12, 0.12, K.base * 0.85 }), 0, wy + 0.18, 0), dark)
    add(at(mesh.box({ 0.28, 0.26, 0.38 }), 0, wy + 0.05, 0.02), metal)                        -- engine
    if kind == "moped" then
      add(at(mesh.box({ 0.36, 0.5, 0.08 }), 0, wy + 0.32, zf - 0.32), col)                     -- leg shield
      add(at(mesh.box({ 0.32, 0.22, 0.55 }), 0, wy + 0.3, -0.18), col)                         -- body under seat
    else
      add(at(mesh.box({ 0.3, 0.2, 0.46 }), 0, wy + 0.42, 0.18), col)                           -- tank
      add(at(mesh.box({ 0.24, 0.12, 0.3 }), 0, wy + 0.32, -0.42), col)                         -- tail
    end
    add(at(mesh.box({ 0.28, 0.08, 0.5 }), 0, wy + 0.5, -0.2), black)                           -- seat
    add(at(mesh.rotate_x(mesh.box({ 0.06, 0.7, 0.06 }), -0.42), 0, wy + 0.3, zf - 0.12), metal) -- fork
    add(at(mesh.box({ 0.66, 0.04, 0.04 }), 0, wy + 0.66, zf - 0.27), metal)                    -- bars
    add(at(mesh.box({ 0.16, 0.03, 0.42 }), 0, wy + K.r + 0.06, zf), col)                      -- front guard
    add(at(mesh.box({ 0.16, 0.03, 0.42 }), 0, wy + K.r + 0.06, zr), dark)                     -- rear guard
    add(at(mesh.rotate_x(mesh.cylinder(0.04, 0.6), math.pi * 0.5), 0.14, wy - 0.02, zr + 0.3), metal) -- exhaust
    -- THE LAMPS (#186): a round headlamp on the fork crown, a tail lamp on the tail, an indicator on a stalk at each
    -- corner (+x is the rider's left)
    local hz, hy = zf - 0.10, wy + 0.56
    add(at(mesh.rotate_x(mesh.cylinder(0.095, 0.10), math.pi * 0.5), 0, hy, hz), dark)            -- the headlamp shell
    shell[#shell + 1] = lens(at(mesh.rotate_x(mesh.cylinder(0.08, 0.02), math.pi * 0.5), 0, hy, hz + 0.055), "lamp")
    local tz, ty = kind == "moped" and -0.47 or -0.58, kind == "moped" and wy + 0.36 or wy + 0.34
    shell[#shell + 1] = lens(at(mesh.box({ 0.14, 0.06, 0.03 }), 0, ty, tz), "lamp_red")
    for _, sx in ipairs({ 1, -1 }) do
      add(at(mesh.box({ 0.10, 0.015, 0.015 }), sx * 0.12, hy - 0.03, hz - 0.02), dark)          -- front stalks
      shell[#shell + 1] = lens(at(mesh.box({ 0.05, 0.035, 0.05 }), sx * 0.19, hy - 0.03, hz - 0.02), "lamp_amber")
      add(at(mesh.box({ 0.10, 0.015, 0.015 }), sx * 0.12, ty, tz + 0.04), dark)                 -- rear stalks
      shell[#shell + 1] = lens(at(mesh.box({ 0.05, 0.035, 0.05 }), sx * 0.19, ty, tz + 0.04), "lamp_amber")
    end
  end
  return {
    body = mesh.recompute_normals(mesh.merge(shell)),
    albedo = { 1, 1, 1 }, metallic = 0.3, roughness = 0.45,
    lights = kind == "bicycle" and { { name = "driver_seat", pos = { 0, wy + 0.52, -0.26 } } } or {
      { name = "headlight_l", pos = { 0, wy + 0.56, zf - 0.04 } },
      { name = "taillight_l", pos = { 0, kind == "moped" and wy + 0.36 or wy + 0.34, kind == "moped" and -0.485 or -0.595 } },
      { name = "driver_seat", pos = { 0, wy + 0.52, -0.22 } },
    },
    chassis = { half = { 0.2, K.H * 0.5, K.base * 0.5 + K.r * 0.6 } },
    mass = K.mass, com_offset = -0.1,
    engine_torque = K.torque, max_rpm = K.rpm, gear_ratios = K.gears,
    max_steer_deg = K.steer,
    brake_torque = math.floor(K.mass * 1.4), hand_brake_torque = math.floor(K.mass * 2.0),
    two_wheeler = true, max_lean_deg = K.lean, pedals = kind == "bicycle",
    -- the rider about the seat's hip point (ADR-0252): the left grip on the bars, the left foot on its peg -- or on a
    -- bicycle the crank's centre, the feet going round it
    seat = K.seat,
    engine_damping = kind == "bicycle" and 0.0 or nil, engine_inertia = kind == "bicycle" and 0.05 or nil,
    yaw_assist = 0, anti_roll = 0, max_roll_deg = K.lean + 15,
    drive = "rwd", drag_area = kind == "bicycle" and 0.5 or 0.7,
    floor_clearance = 0.12,
    shift_time = 0.15, clutch_time = 0.1, shift_latency = 0.2,
    suspension = { min = 0.0, max = 0.18, freq = 1.8, damp = 0.6, rest_drop = 0.13 },
    wheel = { radius = K.r, width = K.w },
    wheels = {
      { x = 0, y = wy, z = zf, steered = true, driven = false },
      { x = 0, y = wy, z = zr, steered = false, driven = true, hand_brake = true },
    },
  }
end
function vehicle.motorcycle(seed, opts) return two_wheeler("motorcycle", opts) end
function vehicle.moped(seed, opts) return two_wheeler("moped", opts) end
function vehicle.bicycle(seed, opts) return two_wheeler("bicycle", opts) end

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
  { recipe = "motorcycle",       label = "Motorcycle",           drive = "rwd", quick = true },   -- M drops it
  { recipe = "moped",            label = "Moped",                drive = "rwd" },
  { recipe = "bicycle",          label = "Bicycle",              drive = "rwd" },
}

-- ---------------------------------------------------------------------------
-- The AI car FLEET as DATA (ADR-0065). The citysim instanced renderer draws ambient traffic as one mesh per fleet
-- slot; the C++ reader (apps/citysim/scripting/vehicle_body.cpp) turns `vehicle.fleet[slot+1]` into a vertex-coloured
-- RenderMesh at level load. A slot that fails to load logs an error and draws no car (city_render.cpp).
--
-- A recipe is:
--   body   = a Mesh (the painted shell with its lenses)
--   wheels = a Mesh (the baked wheelset, kept SEPARATE from the body: a car the player COMMANDEERS (ADR-0062)
--            becomes a Jolt Vehicle that grows its own wheels, and wants the body without them)
--   wheel_layout = { { pos=, radius=, width=, steered=, driven=, hand_brake= }, ... } -- the same wheels as data, for
--            the promoted car's Jolt config
--   glass, interior, mid, mid_from, far, seats, doors, driver_seat, rear_hip, see_into, open_seat
--   lights = { { name=, pos={x,y,z} }, ... }   -- named markers (lamps, the bus's dest_sign). +Z forward, x>0 left.
-- Each publishes its own `size` and the sim adopts it (CitySim::setFleet).
-- (The classic mesh.car fleet retired 2026-11-06, ADR-0349: Glenn, "we may also just need to drop those classic
-- looking vehicles".)

-- FLEET V2 (ADR-0139): a slot built by vehicle_kit.lua on the poly.* kit -- smooth since ADR-0348, a real gasket
-- round each window, the grille and lamp lenses shaped onto the body: one opaque vertex-coloured body, the wheelset
-- apart, the glass its own part (drawn opaque and reflective, clear near the player), the cabin, lamp markers and the
-- driver's seat.
local kit = require "vehicle_kit"
local KIT_CABIN = {
  liner = { 0.55, 0.53, 0.50 }, carpet = { 0.10, 0.10, 0.11 }, seat = { 0.22, 0.20, 0.19 },
  dash = { 0.08, 0.08, 0.09 }, steer = { 0.05, 0.05, 0.05 },
}
-- THE FLEET'S BODIES ARE SMOOTH (Glenn, 2026-11-06: "Smooth works"): one Catmull-Clark step over the cage, its
-- character lines creased (vehicle_kit.lua kit.body). A spec can keep its own `level` (0: faceted).
local FLEET_KIT_LEVEL = 1
-- PAST THIS (m) A FLEET CAR DRAWS ITS MID-RANGE FORM (Glenn, 2026-11-06: "faceted lower poly cars with baked
-- normals"): the faceted cage, a third of the smooth body's triangles, every vertex shaded with the smooth body's
-- normal (mesh.transfer_normals), its glass dark and merged in, its wheels baked.
local FLEET_MID_FROM = 50
-- The mid-range car: the kit's level-0 parts painted like the near ones, normals from the near (smooth) parts
local function kit_mid(P, color, near, wheels)
  local car = kit.build(P, 0, { coarse = true })
  local parts = {}
  for name, part in pairs(car.parts) do
    local col = name == "body" and color or (KIT_TINT[name] or LENS_TINT[name] or { 0.5, 0.5, 0.5 })
    parts[#parts + 1] = lens_tag(mesh.paint(part, col), name)
  end
  local m = mesh.transfer_normals(mesh.merge(parts), near, { radius = 0.08 })
  return mesh.merge({ m, wheels })
end
-- THE FAR STAND-IN (Glenn, 2026-11-06: "something in the distance that doesn't look like a cube on a cube"): the same
-- profile lofted through only its key stations (the ends, the arches, the glass keys) with every corner sharp -- the
-- car's own silhouette, a few hundred triangles -- its glass dark, eight-sided wheels in the arches. The city's far
-- band and the GPU crowd draw it past the near tier, in the slot's own colour.
local function kit_far(P, color)
  local Q = setmetatable({ radii = { bottom = 0, side = 0, shoulder = 0, glass = 0, roof = 0 }, lines = false },
                         { __index = P })
  local half, g = P.length * 0.5, P.glass
  local zs = {}
  local function add(z) if z >= -half - 1e-6 and z <= half + 1e-6 then zs[#zs + 1] = math.max(-half, math.min(half, z)) end end
  for _, z in ipairs({ -half, half, -half + 0.18, half - 0.18 }) do add(z) end
  for _, zc in ipairs(P.axles) do add(zc - P.wheel_r - 0.1); add(zc); add(zc + P.wheel_r + 0.1) end
  for _, z in ipairs({ g.cowl, g.a_top, g.c_top, g.deck, g.a_side, g.c_side }) do add(z - 0.03); add(z + 0.03) end
  table.sort(zs)
  local st = {}
  for _, z in ipairs(zs) do if #st == 0 or z - st[#st] > 0.05 then st[#st + 1] = z end end
  Q.stations = st
  local B = kit.body(Q)
  local lo, hi = B:bounds()
  B:translate{ 0, -0.5 * hi[2], -0.5 * (lo[3] + hi[3]) }
  local parts = {}
  for name, part in pairs(B:to_parts{ autosmooth = 8 }) do
    local col = name == "body" and color or (KIT_TINT[name] or { 0.1, 0.1, 0.11 })
    parts[#parts + 1] = mesh.paint(part, col)
  end
  local cy, cz = 0.5 * hi[2], 0.5 * (lo[3] + hi[3])
  local tyre = { 0.05, 0.05, 0.055 }
  for _, zc in ipairs(P.axles) do
    local wx = P.half_width(zc) - P.wheel_w * 0.5 - 0.035
    parts[#parts + 1] = mesh.sweep(mesh.profile.circle(P.wheel_r, 8),
                                   mesh.path.line({ -wx - P.wheel_w * 0.5, P.wheel_r - cy, zc - cz },
                                                  { wx + P.wheel_w * 0.5, P.wheel_r - cy, zc - cz }),
                                   { up = { 0, 1, 0 }, color = tyre })
  end
  return mesh.merge(parts)
end
local function kit_car(spec, color)
  local P = kit.SPECS[spec]()
  local car = kit.build(P, P.level or FLEET_KIT_LEVEL)
  local shell = {}
  for name, part in pairs(car.parts) do
    if name ~= "glass" then
      local col = name == "body" and color or (KIT_TINT[name] or LENS_TINT[name] or { 0.5, 0.5, 0.5 })
      shell[#shell + 1] = lens_tag(mesh.paint(part, col), name)
    end
  end
  local glass = car.parts.glass and mesh.paint(car.parts.glass, KIT_TINT.glass) or nil
  local w = kit.wheel(P.wheel_r, P.wheel_w, 0)
  local tyre = mesh.paint(w.tyre, { 0.05, 0.05, 0.055 })
  local rim = mesh.paint(w.rim, { 0.72, 0.73, 0.75 })
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
    cabinParts[#cabinParts + 1] = mesh.paint(part, col)
  end
  local open = P.open_top or false
  -- A BUS IS MEANT TO BE SEEN INTO (Glenn: "we should see the npcs sitting on the bus. There should be a driver"): its
  -- saloon rides in the shell, its glass stays clear, its seats and doors are where riders are posed
  if car.saloon then
    shell[#shell + 1] = car.saloon
    open = true
  end
  local interior = nil
  if #cabinParts > 0 then
    if open then for _, cp in ipairs(cabinParts) do shell[#shell + 1] = cp end
    else interior = mesh.merge(cabinParts) end
  end
  -- (the parts' own normals, from P:to_parts' autosmooth: re-deriving them over the merged shell gave the lenses'
  -- folded skirts vertices whose faces cancel -- zero normals, the mesh audit's "badnormal")
  local body = mesh.merge(shell)
  local wheels = mesh.merge(wheelParts)
  local level = P.level or FLEET_KIT_LEVEL
  local mid = level > 0 and not car.saloon and kit_mid(P, color, glass and mesh.merge({ body, glass }) or body, wheels) or nil
  return {
    interior = interior,
    body = body,
    wheels = wheels,
    mid = mid,
    mid_from = mid and FLEET_MID_FROM or nil,
    far = kit_far(P, color),
    wheel_layout = layout,
    lights = car.lights,
    size = car.size,
    class = spec,
    glass = glass,
    see_into = open,
    open_seat = open,   -- (ADR-0252: a crash throws a commandeered convertible's driver)
    driver_seat = car.driver_seat,
    rear_hip = car.rear_hip,
    seats = car.seats,
    doors = car.doors,
  }
end
-- A KIT CAR AS PARTS for baking (Glenn, 2026-11-06: "Should there be a way for us to separate the glass from the rest
-- of the body if we consider the car as a model"): the mid-range body (the level-0 cage, its normals transferred)
-- baked from the smooth body, the glass not baked (it is flat: its material does it), the wheels as they are. Each
-- { name, low, high, bake } for mesh.bake_parts.
function vehicle.kit_parts(spec, color)
  local P = kit.SPECS[spec]()
  local function painted(car)
    local shell, glass = {}, nil
    for name, part in pairs(car.parts) do
      if name == "glass" then glass = mesh.paint(part, KIT_TINT.glass)
      else
        local col = name == "body" and color or (KIT_TINT[name] or LENS_TINT[name] or { 0.5, 0.5, 0.5 })
        shell[#shell + 1] = lens_tag(mesh.paint(part, col), name)
      end
    end
    return mesh.merge(shell), glass
  end
  local nearShell, nearGlass = painted(kit.build(P, P.level or FLEET_KIT_LEVEL))
  local midShell, midGlass = painted(kit.build(P, 0, { coarse = true }))
  midShell = mesh.transfer_normals(midShell, nearShell, { radius = 0.08 })
  local c = kit_car(spec, color)
  local parts = { { name = "body", low = midShell, high = nearShell } }
  if midGlass then parts[#parts + 1] = { name = "glass", low = midGlass, bake = false } end
  parts[#parts + 1] = { name = "wheels", low = c.wheels, bake = false }
  return parts, { near = mesh.merge({ nearShell, nearGlass or nearShell, c.wheels }) }
end
-- A kit car's three drawn forms, for the fleet gallery: { near = smooth body + wheels, mid =, far = } (the slot's build)
function vehicle.kit_forms(spec, color)
  local c = kit_car(spec, color)
  return { near = mesh.merge({ c.body, c.wheels, c.glass }), mid = c.mid, far = c.far }
end
-- THE FLEET (v2): kit bodies for every slot, the city BUS last (the sim finds it by its class name, "bus"), the fire
-- engine before it. Semis wait for articulated
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
    { kit = "offroad",     color = { 0.55, 0.42, 0.22 } },   -- off-roader (sand)
    { kit = "taxi",        color = { 0.95, 0.74, 0.08 } },   -- TAXI (cab yellow): dealt to the cabs only, never ambient
    { kit = "step_van",    color = { 0.36, 0.22, 0.10 } },   -- parcel step van (brown)
    { kit = "small_truck", color = { 0.90, 0.90, 0.88 } },   -- small box truck (white cab)
    { bike = "motorcycle", color = { 0.75, 0.08, 0.06 } },   -- motorcycle (red): its rider drawn astride
    { bike = "moped",      color = { 0.20, 0.55, 0.62 } },   -- moped (teal): its rider astride too
    { kit = "fire_engine", color = { 0.70, 0.05, 0.05 } },   -- FIRE ENGINE (fire-engine red): stationed, never traffic
    { kit = "bus",         color = { 0.86, 0.62, 0.08 } },   -- CITY BUS (municipal yellow): the kit bus (ADR-0349)
}

-- A TWO-WHEELER IN THE FLEET (Glenn, 2026-11-04: "motorcycles -- can you make it available"): the drivable recipe
-- above, its wheels baked in for traffic like every fleet body. Its handling IS that drivable spec (two_wheeler,
-- lean, the seat's bars and pegs), so a commandeered bike rides like the one N drops, and the city draws its rider
-- astride from the same seat (a seat with bars, not a wheel).
local function bike_fleet(kind, color)
  local spec = two_wheeler(kind, { color = color })
  local tyre = { 0.04, 0.04, 0.05 }
  local wheelParts, layout = {}, {}
  for _, w in ipairs(spec.wheels) do
    local t = mesh.rotate_z(mesh.cylinder(spec.wheel.radius, spec.wheel.width), math.pi * 0.5)
    wheelParts[#wheelParts + 1] = mesh.translate(mesh.paint(t, tyre), { w.x, w.y, w.z })
    layout[#layout + 1] = { pos = { w.x, w.y, w.z }, radius = spec.wheel.radius, width = spec.wheel.width,
                            steered = w.steered, driven = w.driven, hand_brake = w.hand_brake }
  end
  local lights, driver_seat = {}, nil
  for _, lt in ipairs(spec.lights) do
    if lt.name == "driver_seat" then driver_seat = lt.pos else lights[#lights + 1] = lt end
  end
  local h = spec.chassis.half
  local wheelbase = math.abs(spec.wheels[1].z - spec.wheels[2].z)
  return {
    body = spec.body, wheels = mesh.recompute_normals(mesh.merge(wheelParts)), wheel_layout = layout,
    lights = lights, size = { 0.7, h[2] * 2, wheelbase + 2 * spec.wheel.radius }, class = kind,
    see_into = true,     -- nothing to see through: the rider is always in view
    open_seat = true,
    driver_seat = driver_seat,
  }
end

-- The fleet is DESCRIPTION up front and GEOMETRY on demand.
--
-- Each slot carries its catalogue entry — class and size — as plain data, which
-- is all the SIM needs (follow gaps, parking bays, collider proxies) and costs
-- nothing to read. The body itself arrives only when `build()` is called, which
-- is where the real expense is: the kit's subdivided bodies. A headless run that
-- wants correct car SIZES must not have to pay for the car MESHES to get
-- them — building them eagerly here made every headless city build do exactly
-- that.
local function catalogue(slots)
    local out = {}
    for i, slot in ipairs(slots) do
        if slot.bike then
            local spec = two_wheeler(slot.bike, { color = slot.color })
            out[i] = {
                class = slot.bike,
                size = { 0.7, spec.chassis.half[2] * 2, math.abs(spec.wheels[1].z - spec.wheels[2].z) + 2 * spec.wheel.radius },
                -- its handling IS the drivable spec: two wheels, the lean, the seat's bars and pegs
                handling = spec,
                drivable = slot.bike,
                build = function() return bike_fleet(slot.bike, slot.color) end,
            }
        elseif slot.kit then
            local P = kit.SPECS[slot.kit]()
            local car = kit.build(P, 0)
            out[i] = {
                class = slot.kit,
                size = car.size,
                -- what the physics tier and a commandeered car drive with: the kit's drivable handling
                handling = kit_handling(P, car, KIT_HANDLING[slot.kit] or {}),
                drivable = "kit_" .. slot.kit,   -- the menu's recipe for this car (nil: none)
                build = function() return kit_car(slot.kit, slot.color) end,
            }
        end
    end
    return out
end
-- THE MIXED FLEET (Glenn: "get the new cars into island 8 along with the classic cars"): since the classic cars
-- retired, the kit fleet ordered for the island -- the cars and trucks, the fire engine, the bus last.
local MIXED_SLOTS = {}
-- (Glenn, 2026-11-06: the classic cars retire; the mixed fleet is the kit fleet, the bus last)
for _, s in ipairs(FLEET_SLOTS) do if s.kit ~= "bus" and s.kit ~= "fire_engine" then MIXED_SLOTS[#MIXED_SLOTS + 1] = s end end
for _, s in ipairs(FLEET_SLOTS) do if s.kit == "fire_engine" then MIXED_SLOTS[#MIXED_SLOTS + 1] = s end end
for _, s in ipairs(FLEET_SLOTS) do if s.kit == "bus" then MIXED_SLOTS[#MIXED_SLOTS + 1] = s end end

-- The named fleets. A level picks one with `citysim.fleet` (level_loader points vehicle.fleet at it);
-- without one it gets the kit fleet.
vehicle.fleet_kit = catalogue(FLEET_SLOTS)
vehicle.fleet_mixed = catalogue(MIXED_SLOTS)
vehicle.fleet = vehicle.fleet_kit

return vehicle
