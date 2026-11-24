-- A seesaw in use: the plank tips one way, rests on its tyre a moment, and tips back (#191; prefabs.lua
-- `playground_seesaw_in_use`). The model's plank is its moving node `beam` (playground_seesaw.lua), and the
-- model's own limits are where the tyres are, so the script asks for more than it can have and the limits stop it.
local M = {}

local PERIOD = 3.2       -- seconds for one full rock, there and back
local ASK = 40           -- degrees asked for either way; the model stops it at its tyres
local SPEED = 34         -- degrees a second while it tips

function M:start(e)
  self.time = 0
end

function M:update(e, dt)
  self.time = self.time + dt
  local up = math.sin(2 * math.pi * self.time / PERIOD) >= 0
  if up ~= self.up then
    self.up = up
    entity.model_set(e, "beam", up and ASK or -ASK, SPEED)
  end
end

return M
