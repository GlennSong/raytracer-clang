-- A wall clock that keeps time (#191; a prefab's script, prefabs.lua `wall_clock`). Its three hands are the model's
-- moving nodes `hour`, `minute` and `second` (wall_clock.lua), each turning without end from twelve.
-- It starts from the time on this computer's own clock and counts on from there. The game's time of day is not
-- something a script can ask for yet; when it is, this reads that instead.
local M = {}

function M:start(e)
  local now = (os and os.date) and os.date("*t") or { hour = 10, min = 10, sec = 30 }
  self.seconds = now.hour * 3600 + now.min * 60 + now.sec
  self.turns = { hour = 0, minute = 0, second = 0 }
end

-- a hand goes to its angle the short way on from where it is, so it never runs backward at twelve
local function point(self, e, node, degrees)
  local at = self.turns[node]
  local on = (degrees - at) % 360
  self.turns[node] = at + on
  entity.model_set(e, node, self.turns[node])
end

function M:update(e, dt)
  self.seconds = self.seconds + dt
  local s = self.seconds
  point(self, e, "second", math.floor(s % 60) * 6)          -- it ticks: one step each second
  point(self, e, "minute", (s / 60 % 60) * 6)
  point(self, e, "hour", (s / 3600 % 12) * 30)
end

return M
