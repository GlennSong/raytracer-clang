-- MODEL MOVES: a behaviour script for a prefab whose model has moving nodes, written as a list of what each node
-- does (#191). A library for behaviour scripts: `return require("model_moves").script{ ... }`.
--
-- Each entry names a node of the model and one of two things:
--
--   { node = "drum", spin = 90 }                              turn without end, degrees a second (negative: the other way)
--   { node = "door", from = 0, to = 95, speed = 60,           go to `to`, wait there, come back to `from`, wait, again.
--     wait = 2 [, wait_back = 4] [, delay = 1] }              degrees (a slide: metres) and that many a second;
--                                                             `delay` seconds pass before the first move
--
--   { node = "rock", sway = 10, period = 3 [, phase = 0.5] }  swing either side of where it was placed, like a weight on
--                                                             a string: `sway` degrees each way, `period` seconds there and back
--   { node = "arms", step = 120, speed = 90, wait = 3 }       turn on by `step` degrees, wait, and again (a turnstile)
--   { node = "step_3", loop = 0.4, speed = 0.5 }               slide from where it was placed to `loop` metres on, then
--                                                             jump back and go again (an escalator's step, each taking
--                                                             the place of the next); a negative speed runs it backward
--
--   { node = "strip_3", from = 0, to = 10, speed = 0.3,        the same going and coming back, but BY THE CLOCK: `travel`
--     travel = 9, wait = 7 [, wait_back = 5] [, delay = 2] }  seconds are allowed for each move however soon the part
--                                                             arrives. Parts given the same numbers stay in step for
--                                                             good, which parts with different limits need (a
--                                                             shutter's sections, a curtain's folds)
--
-- A move stopped short by the model's own limits counts as arrived, so `to` may ask for more than a part can give
-- and the limit decides (a shutter's sections all sent up 5 m, each stopping in the box).
--
-- A node the model does not have is left out without complaint, so one script serves models with more or fewer
-- of a thing (a cabinet's drawers). These stand in for people using the thing; when prefabs carry interactions
-- (docs/model-record-design.md, step 9) a pull on a handle makes the same call this does.
local M = {}

local AHEAD = 3600

function M.script(parts)
  local S = {}

  function S:start(e)
    self.state = {}
    for i, p in ipairs(parts) do
      if pcall(entity.model_get, e, p.node) then
        local st = { hold = p.delay or 0, moving = false, toward = p.to, target = 0, time = 0, still = 0 }
        if p.spin then entity.model_set(e, p.node, 0) end
        self.state[i] = st
      end
    end
  end

  function S:update(e, dt)
    for i, p in ipairs(parts) do
      local st = self.state[i]
      if st and p.spin then
        local way = p.spin < 0 and -1 or 1
        if (st.target - entity.model_get(e, p.node)) * way < AHEAD / 2 then st.target = st.target + way * AHEAD end
        entity.model_set(e, p.node, st.target, math.abs(p.spin))
      elseif st and p.sway then
        st.time = st.time + dt
        local rate = 2 * math.pi / p.period
        entity.model_set(e, p.node, p.sway * math.sin(rate * (st.time - (p.phase or 0))), p.sway * rate * 1.5)
      elseif st and p.loop then
        st.time = (st.time + p.speed * dt) % p.loop
        entity.model_set(e, p.node, st.time)
      elseif st and p.step then
        st.hold = st.hold - dt
        if st.hold <= 0 then
          st.target = st.target + p.step
          st.hold = (p.wait or 1) + math.abs(p.step) / p.speed
          entity.model_set(e, p.node, st.target, p.speed)
        end
      elseif st and p.travel then
        st.time = st.time + dt
        local t = st.time - (p.delay or 0)
        local out_for = p.travel + (p.wait or 1)
        local out = t >= 0 and t % (out_for + p.travel + (p.wait_back or p.wait or 1)) < out_for
        if out ~= st.out then
          st.out = out
          entity.model_set(e, p.node, out and p.to or p.from, p.speed)
        end
      elseif st then
        if st.moving then
          local now = entity.model_get(e, p.node)
          st.still = (st.last and math.abs(now - st.last) < 1e-6) and st.still + dt or 0
          st.last = now
          if math.abs(now - st.toward) < 1e-3 or st.still > 0.3 then
            st.moving = false
            st.hold = (st.toward == p.to) and (p.wait or 1) or (p.wait_back or p.wait or 1)
            st.toward = (st.toward == p.to) and p.from or p.to
          end
        else
          st.hold = st.hold - dt
          if st.hold <= 0 then
            st.moving, st.still, st.last = true, 0, nil
            entity.model_set(e, p.node, st.toward, p.speed)
          end
        end
      end
    end
  end

  return S
end

return M
