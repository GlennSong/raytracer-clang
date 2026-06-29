-- Procedural vehicle bodies (ADR-0058) — authored in Lua over the procgen
-- builders, exactly like flora.lua and the gun. Defines a global `vehicle` table:
--
--   vehicle.sedan(seed, opts)      -> spec   four-door car
--   vehicle.hatchback(seed, opts)  -> spec   smaller, peppier variant
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

-- Solid-colour a part: bake_height_color with low==high paints it flat.
local function paint(m, c) return mesh.bake_height_color(m, c, c) end

-- A low car body that sits around the wheels, with a set-back cabin, dark glass,
-- and front/rear lights. `color` is the paint; faces +Z.
local function car_body(L, W, H, color)
    local hw, hl = W * 0.5, L * 0.5
    local glass = { 0.05, 0.06, 0.09 }
    local parts = {}
    -- Low main hull (its sides come down around the wheels).
    parts[#parts + 1] = paint(mesh.box({ W, H * 0.46, L }), color)
    -- Greenhouse / cabin: narrower, set back a touch, sitting on the hull.
    parts[#parts + 1] = paint(mesh.translate(
        mesh.box({ W * 0.84, H * 0.42, L * 0.46 }), { 0, H * 0.40, -L * 0.04 }), color)
    -- Windshield + rear window (dark glass bands).
    parts[#parts + 1] = paint(mesh.translate(
        mesh.box({ W * 0.80, H * 0.30, L * 0.05 }), { 0, H * 0.42, L * 0.15 }), glass)
    parts[#parts + 1] = paint(mesh.translate(
        mesh.box({ W * 0.80, H * 0.26, L * 0.05 }), { 0, H * 0.42, -L * 0.23 }), glass)
    -- Head/taillights at the front (+Z) and rear (-Z) corners.
    local ly = -H * 0.08
    local lx = hw - 0.30
    for _, sx in ipairs({ 1, -1 }) do
        parts[#parts + 1] = paint(mesh.translate(
            mesh.box({ 0.34, 0.18, 0.10 }), { sx * lx, ly, hl - 0.02 }), { 1.0, 0.97, 0.82 })
        parts[#parts + 1] = paint(mesh.translate(
            mesh.box({ 0.34, 0.18, 0.10 }), { sx * lx, ly, -hl + 0.02 }), { 0.85, 0.06, 0.05 })
    end
    return mesh.recompute_normals(mesh.merge(parts))
end

function vehicle.sedan(seed, opts)
    opts = opts or {}
    local L = opts.length or 4.2
    local W = opts.width or 1.8
    local H = opts.height or 1.3          -- lower, planted stance
    local color = opts.color or { 0.70, 0.12, 0.12 }
    local hw = W * 0.5
    local inset = 0.10
    local axleY = -H * 0.5 + 0.34         -- wheel centres near the body's lower edge
    local frontZ = L * 0.34
    local rearZ = -L * 0.34
    return {
        body = car_body(L, W, H, color),
        albedo = color,
        metallic = 0.6,
        roughness = 0.35,
        chassis = { half = { hw, H * 0.5, L * 0.5 } },
        mass = opts.mass or 1400,
        com_offset = opts.com_offset or -0.45,   -- low CoG so it doesn't tip in turns
        engine_torque = opts.engine_torque or 500,
        max_rpm = opts.max_rpm or 6000,
        max_steer_deg = opts.max_steer_deg or 32,
        brake_torque = opts.brake_torque or 1600,
        hand_brake_torque = opts.hand_brake_torque or 4000,
        wheel = { radius = 0.34, width = 0.32 },  -- chunkier tyres
        -- Front pair steers + drives (FWD, arcade-friendly); rear pair gets the
        -- handbrake. x>0 is the car's right, z>0 its front, y the axle height.
        wheels = {
            { x =  hw - inset, y = axleY, z = frontZ, steered = true,  driven = true },
            { x = -hw + inset, y = axleY, z = frontZ, steered = true,  driven = true },
            { x =  hw - inset, y = axleY, z = rearZ,  steered = false, driven = false, hand_brake = true },
            { x = -hw + inset, y = axleY, z = rearZ,  steered = false, driven = false, hand_brake = true },
        },
    }
end

function vehicle.hatchback(seed, opts)
    opts = opts or {}
    opts.length = opts.length or 3.6
    opts.mass = opts.mass or 1100
    opts.engine_torque = opts.engine_torque or 430
    opts.color = opts.color or { 0.15, 0.45, 0.75 }
    return vehicle.sedan(seed, opts)
end

return vehicle
