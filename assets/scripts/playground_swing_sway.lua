-- A swing in use: each seat sways back and forth like a weight on a string, each at its own size of swing and out
-- of step with the next (#191; prefabs.lua `playground_swing_in_use`). The model's seats are its moving nodes
-- `seat_1`, `seat_2`, ... (playground_swing.lua); however many it has are found when the script starts.
local M = {}

local PERIOD = 2.8                          -- seconds for one swing out and back (a 2 m chain's)
local REACH = { 38, 24, 31, 17 }            -- how far each seat swings either way, in degrees
local LAG = { 0, 1.9, 0.7, 2.6 }            -- how far behind the first each one is, in seconds

function M:start(e)
  self.time, self.seats = 0, {}
  for i = 1, #REACH do
    local node = "seat_" .. i
    if pcall(entity.model_get, e, node) then self.seats[#self.seats + 1] = { node = node, reach = REACH[i], lag = LAG[i] } end
  end
end

function M:update(e, dt)
  self.time = self.time + dt
  local rate = 2 * math.pi / PERIOD
  for _, seat in ipairs(self.seats) do
    -- the angle it should be at now, and a speed fast enough to keep up with it
    entity.model_set(e, seat.node, seat.reach * math.sin(rate * (self.time - seat.lag)), seat.reach * rate * 1.5)
  end
end

return M
