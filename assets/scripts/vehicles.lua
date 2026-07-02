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


-- The car body: the engine's lofted CURVED shell (mesh.car_shell) — smooth
-- superellipse sections along a real roofline with glass painted into the
-- greenhouse. The SAME generator the NPC fleet instances, so the player's car
-- matches the traffic 1:1 (ADR-0061). No baked wheels: the player's wheels are
-- separate physics-driven entities. `color` is the paint; faces +Z.
local function car_body(L, W, H, color)
    return mesh.car_shell("sedan", color, { W, H, L }, false)
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
        engine_torque = opts.engine_torque or 650,   -- enough grunt to climb a curb back onto the road
        max_rpm = opts.max_rpm or 6000,
        max_steer_deg = opts.max_steer_deg or 32,
        brake_torque = opts.brake_torque or 1600,
        hand_brake_torque = opts.hand_brake_torque or 4000,
        wheel = { radius = 0.34, width = 0.32 },  -- chunkier tyres
        -- All four wheels DRIVEN (4WD) so the car can crawl back up off-road and
        -- over kerbs; the front pair steers, the rear pair also takes the
        -- handbrake. x>0 is the car's right, z>0 its front, y the axle height.
        wheels = {
            { x =  hw - inset, y = axleY, z = frontZ, steered = true,  driven = true },
            { x = -hw + inset, y = axleY, z = frontZ, steered = true,  driven = true },
            { x =  hw - inset, y = axleY, z = rearZ,  steered = false, driven = true, hand_brake = true },
            { x = -hw + inset, y = axleY, z = rearZ,  steered = false, driven = true, hand_brake = true },
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
