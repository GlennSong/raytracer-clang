-- Procedural vehicle bodies (ADR-0059) — authored in Lua over the procgen
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
-- the same primitive shapes as the player's car_body above, but as flat data).
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
    else                        -- sedan: the player's car_body proportions
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
