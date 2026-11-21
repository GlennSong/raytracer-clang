-- An escalator running down (#191; prefabs.lua `escalator_down`): escalator_run.lua, the other way.
local M = require "escalator_run"
local down = {}
function down:start(e) M.speed = -0.5; M.start(self, e); M.speed = 0.5 end
function down:update(e, dt) M.update(self, e, dt) end
return down
