-- A park sculpture that turns: its form goes slowly round on the plinth, one turn in two minutes (#191;
-- prefabs.lua `park_sculpture_turning`). The model's `form` node turns without end, so it is sent a long way round.
local M = {}

local SPEED = 3          -- degrees a second
local AHEAD = 3600       -- how far ahead its target is kept, in degrees

function M:start(e)
  self.target = AHEAD
  entity.model_set(e, "form", self.target, SPEED)
end

function M:update(e, dt)
  if self.target - entity.model_get(e, "form") < AHEAD / 2 then
    self.target = self.target + AHEAD
    entity.model_set(e, "form", self.target, SPEED)
  end
end

return M
