-- An escalator running (#191; a prefab's script, prefabs.lua `escalator_up`, `escalator_down`).
-- Every step slides up the slope by one step's place and jumps back, all together, so the stairs flow without
-- end (model_moves.lua "loop"). How far one place is depends on the escalator's rise and slope, so it is read
-- from the model: the distance between its markers `run_from` and `run_to` (escalator.lua).
-- Up at 0.5 m a second; a second prefab runs the same model down by naming this script's twin.
local moves = require "model_moves"
local M = {}
M.speed = 0.5

local function axis(p, i) return p[i] or p[({ "x", "y", "z" })[i]] end

function M:start(e)
  local a, b = entity.model_marker(e, "run_from"), entity.model_marker(e, "run_to")
  local span = 0
  for i = 1, 3 do span = span + (axis(b, i) - axis(a, i)) ^ 2 end
  span = math.sqrt(span)
  local parts = {}
  for i = 1, 60 do parts[i] = { node = "step_" .. i, loop = span, speed = M.speed } end
  self.run = moves.script(parts)
  self.run:start(e)
end

function M:update(e, dt) self.run:update(e, dt) end

return M
