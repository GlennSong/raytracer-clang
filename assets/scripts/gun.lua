-- Physics block shooting gun (ADR-0024) — a MonoBehaviour-style ECS script
-- attached to the player entity. The Lua port of the C++ ShootingSystem: on the
-- "fire" action it spawns a bouncing dynamic cube flying along the camera aim.
--
-- Attached via a ScriptBehaviour component; ScriptSystem runs start()/update().
-- `camera`, `input`, and `spawn` come from the gameplay binding surface.

local M = {}

local SPEED = 50.0   -- launch speed (world units/sec)
local SIZE  = 0.5    -- cube edge
local MAX   = 500    -- cap, like the C++ shooter

function M:start(e)
    self.fired = 0
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
