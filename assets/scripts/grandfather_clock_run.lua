-- A tall case clock keeping time (#191; a prefab's script, prefabs.lua `grandfather_clock`): its hands are moved
-- as the wall clock's are (wall_clock_run.lua), and its pendulum swings once every two seconds (model_moves.lua).
local clock = require("wall_clock_run")
local swing = require("model_moves").script{ { node = "pendulum", sway = 6, period = 2 } }
local M = {}

function M:start(e)
  clock.start(self, e)
  swing.start(self, e)
end

function M:update(e, dt)
  clock.update(self, e, dt)
  swing.update(self, e, dt)
end

return M
