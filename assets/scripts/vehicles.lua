-- Procedural vehicle bodies (ADR-0057) — authored in Lua over the procgen
-- builders, exactly like flora.lua and the gun. Defines a global `vehicle` table:
--
--   vehicle.sedan(seed, opts)      -> spec   four-door car
--   vehicle.hatchback(seed, opts)  -> spec   smaller, peppier variant
--
-- A "spec" is a plain table the C++ VehicleSpec reader (vehicle_spec.cpp)
-- consumes: a `body` Mesh built with mesh.* plus handling/physics parameters and
-- a wheel layout. Hosts load this once so `vehicle` is available, then run a
-- small chunk like `return vehicle.sedan(seed, {color={0,0.4,0.8}})`. Pure and
-- deterministic per seed. NOTE: geometry/params are unvalidated in this
-- environment (no Lua/Jolt build here); tune the look + handling on a real build.

vehicle = {}

-- A simple two-box car body: a hull with a smaller cabin set on top and back.
local function sedan_body(L, W, H, color)
    local hull = mesh.box({ W, H * 0.6, L })
    local cabin = mesh.translate(mesh.box({ W * 0.86, H * 0.5, L * 0.45 }),
                                 { 0, H * 0.5, -L * 0.05 })
    local body = mesh.merge({ hull, cabin })
    body = mesh.recompute_normals(body)
    -- Tint top-lighter so the form reads before real materials are tuned.
    return mesh.bake_height_color(body, color, { color[1] * 1.2, color[2] * 1.2,
                                                 color[3] * 1.2 })
end

function vehicle.sedan(seed, opts)
    opts = opts or {}
    local L = opts.length or 4.2
    local W = opts.width or 1.8
    local H = opts.height or 1.4
    local color = opts.color or { 0.70, 0.12, 0.12 }
    local hw = W * 0.5
    local inset = 0.15
    local axleY = -H * 0.5 + 0.10
    local frontZ = L * 0.32
    local rearZ = -L * 0.32
    return {
        body = sedan_body(L, W, H, color),
        albedo = color,
        metallic = 0.6,
        roughness = 0.35,
        chassis = { half = { hw, H * 0.5, L * 0.5 } },
        mass = opts.mass or 1400,
        engine_torque = opts.engine_torque or 500,
        max_rpm = opts.max_rpm or 6000,
        max_steer_deg = opts.max_steer_deg or 32,
        brake_torque = opts.brake_torque or 1600,
        hand_brake_torque = opts.hand_brake_torque or 4000,
        wheel = { radius = 0.34, width = 0.24 },
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
