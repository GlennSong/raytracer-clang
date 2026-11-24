-- A crane at work, as a prefab's script (#191, step 8; prefabs.lua `crane_working`): the cab swings from side to
-- side, the boom lifts and lowers, the hook line runs out and in. Each move is a target and a speed on one of the
-- model's moving nodes (rigs.lua "crane"); the model's limits stop it, and the crane's collision shapes go with it.
local M = {}

-- a node heads for one end, and when it is there, for the other
local moves = {
  { node = "cabin", lo = -70, hi = 70, speed = 12 },   -- degrees, degrees a second
  { node = "boom", lo = 5, hi = 55, speed = 6 },
  { node = "hook", lo = 0.3, hi = 4, speed = 0.8 },    -- metres, metres a second
}

function M:start(e)
  self.toward = {}
  for i, mv in ipairs(moves) do
    self.toward[i] = mv.hi
    entity.model_set(e, mv.node, mv.hi, mv.speed)
  end
end

function M:update(e, dt)
  for i, mv in ipairs(moves) do
    if math.abs(entity.model_get(e, mv.node) - self.toward[i]) < 1e-3 then
      self.toward[i] = (self.toward[i] == mv.hi) and mv.lo or mv.hi
      entity.model_set(e, mv.node, self.toward[i], mv.speed)
    end
  end
end

return M
