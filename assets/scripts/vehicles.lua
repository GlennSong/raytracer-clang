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

-- A CAR'S DAMAGE (ADR-0353/0355; Glenn, 2026-11-06: "move the tuning to lua"): how hard it takes a knock, how it
-- dents, when it smokes, how its panels leave, how a hurt engine sounds. `default` is every car's; a kit car's own
-- entry overrides only what it names (a sub-table -- dent, smoke, kick, rough -- field by field). Read into the
-- drivable spec as `damage` (vehicle_spec.cpp); a car dropped with N reads this file afresh, so a change shows on
-- the next drop.
--   knock_from / knock_full    m/s: a knock below the first leaves no mark; one at the second spends a whole zone
--   scrape_per_metre           a zone's health a scrape takes per metre slid (at ref_mass kg; heavier takes less)
--   spark_knock / spark_slide  m/s a knock / a scrape sparks from
--   crash_window               s: knocks on one zone this close together are one crash (the worst of them counts)
--   ground_share / break_share a knock from below (a hard landing) / knocking a street piece loose costs this share
--   zone_scale / panel_scale   a knock's cost to the zone it lands on / the panel there (at 0 it comes off)
--   dent = { from, radius_lo, radius_hi, depth_lo, depth_per, depth_max, scuff_lo, scuff_per }   (m)
--   smoke = { steam_below, black_below }   the nose's health the hood steams below, then smokes black
--   kick = { car, out, up }    a panel coming off: the share of the car's speed it keeps, and m/s out and up
--   rough = { below, wobble, misfire }   the nose's health the engine runs rough below; pitch lurch; misfire share
--   crack_from / smash_from    a knock's cost near glass that crazes it / smashes a hole in it (ADR-0356)
local KIT_DAMAGE = {
  default = {
    knock_from = 3.0, knock_full = 40.0, scrape_per_metre = 0.004, ref_mass = 1500,
    spark_knock = 4.0, spark_slide = 2.5, crash_window = 0.33, ground_share = 0.33, break_share = 0.10,
    zone_scale = 1.2, panel_scale = 1.6, crack_from = 0.04, smash_from = 0.30,
    dent = { from = 0.01, radius_lo = 0.25, radius_hi = 0.90, depth_lo = 0.04, depth_per = 0.35, depth_max = 0.26,
             scuff_lo = 0.35, scuff_per = 1.2 },
    smoke = { steam_below = 0.6, black_below = 0.25 },
    kick = { car = 0.7, out = 0.8, up = 0.8 },
    rough = { below = 0.6, wobble = 0.07, misfire = 0.5 },
  },
  -- the big ones shrug more off and dent less deep; the soft-top folds easier
  bus = { knock_full = 55.0, dent = { depth_per = 0.25 } },
  bus_standing = { knock_full = 55.0, dent = { depth_per = 0.25 } },
  fire_engine = { knock_full = 55.0, dent = { depth_per = 0.25 } },
  small_truck = { knock_full = 48.0 },
  step_van = { knock_full = 45.0 },
  offroad = { knock_full = 46.0, dent = { depth_per = 0.28 } },   -- (steel bumpers)
  convertible = { knock_full = 34.0 },
}
local function kit_damage(name)
  local out = {}
  for k, v in pairs(KIT_DAMAGE.default) do
    if type(v) == "table" then
      local t = {}
      for k2, v2 in pairs(v) do t[k2] = v2 end
      out[k] = t
    else
      out[k] = v
    end
  end
  for k, v in pairs(KIT_DAMAGE[name] or {}) do
    if type(v) == "table" then for k2, v2 in pairs(v) do out[k][k2] = v2 end else out[k] = v end
  end
  return out
end
vehicle.kit_damage = kit_damage

-- A CAR'S TANK (ADR-0357), read into its spec as `fuel` (engine/vehicle_fuel.h FuelRule); a name without a row takes
-- the default, a row's fields over it.
--   tank        litres
--   idle / full litres an hour ticking over / at full throttle (reverse burns like forward)
--   pace        game time: the burn runs this many times the real one (a tank is minutes of play)
--   start       the share of the tank a car starts with
--   fill        litres a second a pump pours
--   low         the share the readout goes red below
local KIT_FUEL = {
  default = { tank = 45, idle = 1.2, full = 30, pace = 12, start = 1.0, fill = 9, low = 0.15 },
  hatchback = { tank = 38, full = 24 },
  convertible = { tank = 50, full = 36 },
  suv = { tank = 60, full = 38 },
  pickup = { tank = 70, full = 42 },
  offroad = { tank = 75, full = 44 },
  jeep = { tank = 55, full = 34 },
  semi = { tank = 400, full = 90, fill = 25 },
  step_van = { tank = 110, full = 48, fill = 14 },
  small_truck = { tank = 120, full = 52, fill = 14 },
  fire_engine = { tank = 250, full = 80, fill = 20 },
  bus = { tank = 300, full = 70, fill = 20 },
  bus_standing = { tank = 300, full = 70, fill = 20 },
}
local function kit_fuel(name)
  local out = {}
  for k, v in pairs(KIT_FUEL.default) do out[k] = v end
  for k, v in pairs(KIT_FUEL[name] or {}) do out[k] = v end
  return out
end
vehicle.kit_fuel = kit_fuel

-- THE KIT's FINISHES: a part's colour by its material name (the paint is the slot's), and the drivable car's cabin
local KIT_TINT = {
  glass = { 0.16, 0.20, 0.24 }, trim = { 0.04, 0.04, 0.045 }, gasket = { 0.025, 0.025, 0.028 },
  grille = { 0.06, 0.06, 0.065 }, chrome = { 0.72, 0.73, 0.75 }, lamp = { 0.95, 0.95, 0.90 },
  lamp_red = { 0.75, 0.06, 0.05 }, interior = { 0.35, 0.26, 0.18 }, bed = { 0.08, 0.08, 0.09 },
  liner = { 0.62, 0.61, 0.58 },   -- a bus's inside walls and ceiling (ADR-0356)
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
  -- (subdivided once, as the traffic's near bodies are: a dent needs vertices to push -- the faceted body had a
  -- handful per panel and a knock barely showed; ADR-0355)
  local car = kit.build(P, P.level or 1, { panels = true })
  -- (its handling from the faceted build, as the traffic's catalogue takes it: the subdivided body is a touch smaller,
  -- and a commandeered car and the one N drops must drive as one)
  local handled = kit.build(P, 0)
  -- a material set (kit.build's parts) as its painted shell and its glass
  local function painted_set(set)
    local out = {}
    for name, part in pairs(set) do
      if name ~= "glass" then
        local col = name == "body" and P.color or (KIT_TINT[name] or LENS_TINT[name] or { 0.4, 0.4, 0.4 })
        out[#out + 1] = lens_tag(mesh.paint(part, col), name)
      end
    end
    return out, set.glass
  end
  local shell, glass = painted_set(car.parts)
  local parts = {}
  if glass then
    -- the glass's own colour is the kit's glass tint, not white: a pane's diffuse colour lies over everything seen
    -- through it, and white was a milky veil from inside (Glenn: the bus's view out "almost all white"; ADR-0378)
    parts[#parts + 1] = { mesh = glass, albedo = KIT_TINT.glass, metallic = 0.0, roughness = 0.06, opacity = 0.30 }
  end
  -- THE MOVING PANELS (ADR-0352): each door, the hood, the trunk -- its painted shell and its glass, on its hinge
  local panels = {}
  local names = {}
  for name in pairs(car.panels or {}) do names[#names + 1] = name end
  table.sort(names)
  for _, name in ipairs(names) do
    local pn = car.panels[name]
    local pshell, pglass = painted_set(pn.parts)
    local pcabin = {}
    for cname, part in pairs(pn.cabin or {}) do pcabin[#pcabin + 1] = mesh.paint(part, KIT_CABIN_TINT[cname] or { 0.3, 0.3, 0.3 }) end
    if #pshell > 0 or pglass then
      panels[#panels + 1] = { name = name, pivot = pn.hinge.pivot, axis = pn.hinge.axis, open = pn.hinge.open, lo = pn.lo, hi = pn.hi,
                              mass = pn.hinge.mass, slide = pn.hinge.slide,
                              centre = { 0.5 * (pn.lo[1] + pn.hi[1]), 0.5 * (pn.lo[2] + pn.hi[2]), 0.5 * (pn.lo[3] + pn.hi[3]) } }
      if #pshell > 0 then
        parts[#parts + 1] = { mesh = mesh.recompute_normals(mesh.merge(pshell)), albedo = { 1, 1, 1 },
                              metallic = h.metallic or 0.5, roughness = h.roughness or 0.4, panel = name }
      end
      if pglass then
        parts[#parts + 1] = { mesh = pglass, albedo = KIT_TINT.glass, metallic = 0.0, roughness = 0.06, opacity = 0.30, panel = name }
      end
      if #pcabin > 0 then
        parts[#parts + 1] = { mesh = mesh.merge(pcabin), albedo = { 1, 1, 1 }, metallic = 0.0, roughness = 0.9, panel = name }
      end
    end
  end
  local cabin = {}
  for name, part in pairs(car.cabin or {}) do
    local col = KIT_CABIN_TINT[name] or { 0.3, 0.3, 0.3 }
    cabin[#cabin + 1] = mesh.paint(part, col)
  end
  if #cabin > 0 then
    parts[#parts + 1] = { mesh = mesh.merge(cabin), albedo = { 1, 1, 1 }, metallic = 0.0, roughness = 0.9 }
  end
  -- A BUS's SALOON (its seats, poles, cab: kit.build's `saloon`, already coloured) -- a bus has no car cabin, and one
  -- taken from the traffic drove as an empty shell (Glenn: "the bus has no interior")
  if car.saloon then
    parts[#parts + 1] = { mesh = car.saloon, albedo = { 1, 1, 1 }, metallic = 0.0, roughness = 0.8 }
  end
  local lights = {}
  for _, l in ipairs(car.lights) do lights[#lights + 1] = l end
  lights[#lights + 1] = { name = "driver_seat", pos = car.driver_seat }
  local spec = kit_handling(P, handled, h, opts)
  spec.damage = kit_damage(P.name)
  spec.fuel = kit_fuel(P.name)
  spec.cargo = car.cargo
  spec.body = mesh.recompute_normals(mesh.merge(shell))
  spec.albedo = { 1, 1, 1 }
  spec.metallic = h.metallic or 0.5
  spec.roughness = h.roughness or 0.4
  spec.parts = parts
  spec.panels = panels
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
  -- (Glenn: "as a physics car the bus is too bouncy from side to side": 11 t on a car's bars rocked; air springs are
  -- firm and well damped, and a bus's anti-roll bars are heavy -- but past ~25000 N/m Jolt's rig goes unstable: at
  -- 30000 the bus would not pull away, at 60000 it was thrown 10 m when put down; the phys-tier tests hold this)
  bus         = { drive = "rwd", drag_area = 6.0, torque_per_kg = 0.32, max_steer_deg = 38, suspension = street(1.5, 0.24, 0.8),
                  anti_roll = 20000, drift = { rear_grip = 0.85, slip_deg = 8, min_speed = 14 } },
}
KIT_HANDLING.bus_standing = KIT_HANDLING.bus   -- (the same bus, another saloon)
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
  -- (a bus is built with its door panels apart: the city slides them open at its stops, ADR-0356)
  local car = kit.build(P, P.level or FLEET_KIT_LEVEL, { panels = P.saloon ~= nil })
  local shell = {}
  for name, part in pairs(car.parts) do
    if name ~= "glass" then
      local col = name == "body" and color or (KIT_TINT[name] or LENS_TINT[name] or { 0.5, 0.5, 0.5 })
      shell[#shell + 1] = lens_tag(mesh.paint(part, col), name)
    end
  end
  local glass = car.parts.glass and mesh.paint(car.parts.glass, KIT_TINT.glass) or nil
  -- A BUS's DOOR LEAVES (ADR-0356; Glenn: "When the bus driver stops does the bus door open for passengers to get
  -- on?"): each leaf's paint and its glass, and its slide at fully open, for the city to draw apart from the body
  local busDoors = {}
  local doorNames = {}
  for name in pairs(car.panels or {}) do if name:sub(1, 8) == "bus_door" then doorNames[#doorNames + 1] = name end end
  table.sort(doorNames)
  for _, name in ipairs(doorNames) do
    local pn = car.panels[name]
    local paint, leafGlass = {}, nil
    for mname, part in pairs(pn.parts) do
      if mname == "glass" then leafGlass = mesh.paint(part, KIT_TINT.glass)
      else paint[#paint + 1] = lens_tag(mesh.paint(part, mname == "body" and color or (KIT_TINT[mname] or { 0.5, 0.5, 0.5 })), mname) end
    end
    if #paint > 0 then busDoors[#busDoors + 1] = { body = mesh.merge(paint), glass = leafGlass, slide = pn.hinge.slide } end
  end
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
    bus_doors = #busDoors > 0 and busDoors or nil,
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
    -- ...and its STANDING-ROOM twin (benches along the walls): the routes are dealt round the bus bodies in turn
    { kit = "bus_standing", color = { 0.86, 0.62, 0.08 } },
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
                color = slot.color,
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
                color = slot.color,              -- ...and its paint: a car taken from the traffic is built as that
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
local function isBus(s) return s.kit == "bus" or s.kit == "bus_standing" end
for _, s in ipairs(FLEET_SLOTS) do if not isBus(s) and s.kit ~= "fire_engine" then MIXED_SLOTS[#MIXED_SLOTS + 1] = s end end
for _, s in ipairs(FLEET_SLOTS) do if s.kit == "fire_engine" then MIXED_SLOTS[#MIXED_SLOTS + 1] = s end end
for _, s in ipairs(FLEET_SLOTS) do if isBus(s) then MIXED_SLOTS[#MIXED_SLOTS + 1] = s end end

-- The named fleets. A level picks one with `citysim.fleet` (level_loader points vehicle.fleet at it);
-- without one it gets the kit fleet.
vehicle.fleet_kit = catalogue(FLEET_SLOTS)
vehicle.fleet_mixed = catalogue(MIXED_SLOTS)
vehicle.fleet = vehicle.fleet_kit

return vehicle
