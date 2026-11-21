-- A dishwasher being loaded (#191; a prefab's script, prefabs.lua): the door drops, the lower rack is slid out and
-- back, then the upper one, and the door is shut. It runs by the clock, twenty seconds a round, so a rack is never
-- out while the door is up.
local M = {}

function M:start(e) self.time = 0 end

function M:update(e, dt)
  self.time = (self.time + dt) % 20
  local t = self.time
  entity.model_set(e, "door", (t > 1 and t < 13) and 90 or 0, 70)
  entity.model_set(e, "rack_low", (t > 3 and t < 6.5) and 0.44 or 0, 0.5)
  entity.model_set(e, "rack_high", (t > 7 and t < 10.5) and 0.36 or 0, 0.5)
end

return M
