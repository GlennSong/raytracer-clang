-- A clock that shows the game's time (#191; a prefab's script, prefabs.lua `wall_clock`, and the tall clock's).
-- Its three hands are the model's moving nodes `hour`, `minute` and `second` (wall_clock.lua), each turning without
-- end from twelve. The time is the game's own day (`game.hour()`, the day and night cycle's), so a clock agrees
-- with the sun outside and with every other clock. In a level with no cycle it starts at ten past ten and counts
-- real seconds.
local M = {}

function M:start(e)
  self.seconds = (10 * 60 + 10) * 60 + 30
  self.turns = { hour = 0, minute = 0, second = 0 }
end

-- a hand goes to its angle the short way from where it is, so it never sweeps a whole turn to get a hair on
local function point(self, e, node, degrees)
  local at = self.turns[node]
  local on = (degrees - at) % 360
  if on > 180 then on = on - 360 end
  self.turns[node] = at + on
  entity.model_set(e, node, self.turns[node])
end

function M:update(e, dt)
  local hour = game and game.hour and game.hour()
  if hour then self.seconds = hour * 3600 else self.seconds = self.seconds + dt end
  local s = self.seconds
  -- (the game's day may run faster than a real one: the second hand sweeps with it and does not tick)
  point(self, e, "second", hour and (s % 60) * 6 or math.floor(s % 60) * 6)
  point(self, e, "minute", (s / 60 % 60) * 6)
  point(self, e, "hour", (s / 3600 % 12) * 30)
end

return M
