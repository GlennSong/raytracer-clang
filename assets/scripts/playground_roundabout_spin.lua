-- A roundabout in use: it is pushed up to speed, left to slow, and pushed again (#191; prefabs.lua
-- `playground_roundabout_in_use`). The model's `deck` node turns without end (playground_roundabout.lua), so its
-- target is kept a long way ahead and only its speed changes.
local M = {}

local FAST, SLOW = 110, 18   -- degrees a second: just after a push, and when the next push comes
local PUSH = 1.2             -- seconds a push takes
local COAST = 9              -- seconds it is left to slow
local AHEAD = 3600           -- how far ahead its target is kept, in degrees

function M:start(e)
  self.time, self.target = 0, AHEAD
end

function M:update(e, dt)
  self.time = (self.time + dt) % (PUSH + COAST)
  local speed
  if self.time < PUSH then
    speed = SLOW + (FAST - SLOW) * (self.time / PUSH)
  else
    -- it loses the same share of its speed each second, as a thing slowed by rubbing does
    local t = (self.time - PUSH) / COAST
    speed = FAST * (SLOW / FAST) ^ t
  end
  if self.target - entity.model_get(e, "deck") < AHEAD / 2 then self.target = self.target + AHEAD end
  entity.model_set(e, "deck", self.target, speed)
end

return M
