-- Physics-block shooting gun (ADR-0024) — a MonoBehaviour-style ECS script on
-- the player. On "fire" it spawns a dynamic cube along the camera aim (the Lua
-- port of the C++ ShootingSystem). On start it ALSO generates a gun viewmodel
-- with the procgen builders and spawns it as its own camera-following entity.
--
-- `camera`/`input`/`spawn` are the gameplay surface; `mesh.*` are the procgen
-- builders — both open in the ScriptSystem VM (ADR-0023/0024).

local M = {}

local SPEED = 50.0   -- launch speed (world units/sec)
local SIZE  = 0.5    -- bullet cube edge
local MAX   = 500    -- bullet cap, like the C++ shooter

-- A little gun, kit-bashed from primitives. The barrel runs along local +Z (the
-- "forward" the follow script aims down the camera direction).
local function build_gun()
    local receiver = mesh.box({ 0.09, 0.12, 0.34 })
    local barrel = mesh.translate(
        mesh.rotate_x(mesh.cylinder(0.028, 0.55), math.pi * 0.5),  -- Y-up -> +Z
        { 0.0, 0.03, 0.34 })
    local grip = mesh.translate(
        mesh.rotate_x(mesh.box({ 0.07, 0.20, 0.09 }), 0.35),
        { 0.0, -0.16, -0.10 })
    local sight = mesh.translate(mesh.box({ 0.02, 0.04, 0.05 }), { 0.0, 0.10, 0.10 })
    return mesh.recompute_normals(mesh.merge({ receiver, barrel, grip, sight }))
end

-- The viewmodel's own behaviour: glue itself to the camera every frame, barrel
-- pointing where you look. Snap PrevTransform so it renders crisp (no smear).
local FOLLOW = [[
local G = {}
function G:update(e, dt)
    local eye, fwd, right = camera.eye(), camera.forward(), camera.right()
    entity.set_position(e, {
        eye[1] + fwd[1] * 0.35 + right[1] * 0.18 - 0.18 * 0.0,
        eye[2] + fwd[2] * 0.35 + right[2] * 0.18 - 0.18,
        eye[3] + fwd[3] * 0.35 + right[3] * 0.18,
    })
    entity.look_along(e, fwd)
    entity.snap_prev(e)
end
return G
]]

function M:start(e)
    self.fired = 0
    spawn.model{
        mesh      = build_gun(),
        position  = camera.eye(),
        color     = { 0.14, 0.14, 0.16 },
        metallic  = 0.9,
        roughness = 0.25,
        script    = FOLLOW,
    }
end

function M:update(e, dt)
    if self.fired >= MAX then return end
    if not input.pressed("fire") then return end

    local eye   = camera.eye()
    local fwd   = camera.forward()
    local right = camera.right()

    -- Muzzle: in front of the eye, nudged right and down (mirrors the C++ one).
    local pos = {
        eye[1] + fwd[1] * 0.5 + right[1] * 0.2,
        eye[2] + fwd[2] * 0.5 + right[2] * 0.2 - 0.15,
        eye[3] + fwd[3] * 0.5 + right[3] * 0.2,
    }
    local vel = { fwd[1] * SPEED, fwd[2] * SPEED, fwd[3] * SPEED }

    spawn.block{
        position    = pos,
        velocity    = vel,
        size        = SIZE,
        restitution = 0.7,
        friction    = 0.3,
        color       = { 1.0, 0.85, 0.1 },
        emission    = { 0.5, 0.3, 0.0 },
    }
    self.fired = self.fired + 1
end

return M
