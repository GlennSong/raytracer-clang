-- RIG TABLES: how each KIND of thing moves (#191, model step 4).
--
-- A rig table lists a kind's joints: each one's name, what it hangs under, how it moves (turn or slide), about which
-- axis, and between which limits. A recipe applies the table to its model and gives only what is its own: where each
-- joint is, and any limit that differs. So every car has a `door_front_l` that opens the same way, and one script drives
-- them all. (Glenn, 2026-11-19: one rig table per kind.)
--
--   local rigs = require "rigs"
--   local joint = rigs.apply(car, "car", {
--     door_front_l = { pos = { 0.78, 0.4, 0.55 } },             -- the hinge line
--     susp_front_l = { pos = { 0.8, 0.33, 1.3 } },              -- steer_front_l and spin_front_l follow, at the same place
--     hood    = { pos = { 0, 0.85, 0.9 }, max = 55 },      -- this car's hood opens less far
--   })
--   joint.spin_front_l:use(tire)                                -- a joint is a node: hang meshes and pieces on it
--
-- The rules a table follows:
--   * Axes are in the model's own space as the model is built: +x is the left side, +y up, +z the front.
--   * THE SAME VALUE MEANS THE SAME ACTION ON BOTH SIDES. "door open 40" opens a left or a right door, so the two
--     hinge axes point opposite ways. "steer 20" swings both front wheels the same way, so their axes agree.
--   * MOTION SITS ABOVE ANY FLIP. A right-hand tire is the left one turned half a turn; that turn goes on the piece
--     under `spin_front_r`, never on the joint, or the same spin would roll the two sides opposite ways.
--   * Turn limits are degrees, slide limits metres, and 0 (the joint as placed) lies between them. A turn with no
--     limits has no end.
--
-- Joint names are the ones the car kit and the game already use for a car's panels (vehicle_kit.lua: door_front_l,
-- door_rear_r, hood, trunk, hatch, tailgate), carried on to the wheels (susp_front_l, steer_front_l, spin_front_l).
--
-- A joint a recipe gives no place for is left out (a two-door car has no rear doors), and a joint under it goes with
-- it. A place for a name the table lacks is an error.

local rigs = {}

rigs.kinds = {
  car = {
    -- doors: hinged at the front edge, opening outward
    { name = "door_front_l", motion = { turn = { 0, -1, 0 }, min = 0, max = 70 } },
    { name = "door_front_r", motion = { turn = { 0, 1, 0 }, min = 0, max = 70 } },
    { name = "door_rear_l", motion = { turn = { 0, -1, 0 }, min = 0, max = 65 } },
    { name = "door_rear_r", motion = { turn = { 0, 1, 0 }, min = 0, max = 65 } },
    -- lids: the hood hinged at its rear edge, the trunk at its front edge, both lifting
    { name = "hood", motion = { turn = { -1, 0, 0 }, min = 0, max = 60 } },
    { name = "trunk", motion = { turn = { 1, 0, 0 }, min = 0, max = 80 } },
    { name = "hatch", motion = { turn = { 1, 0, 0 }, min = 0, max = 80 } },
    -- a pickup's tailgate: on its foot, the top dropping back to lie flat
    { name = "tailgate", motion = { turn = { -1, 0, 0 }, min = 0, max = 90 } },
    -- a wheel: the suspension slides, the steering turns under it (fronts only), the spin turns under that.
    -- Positive steer swings the front of the wheel toward +x (left). Positive spin rolls the car forward.
    { name = "susp_front_l", motion = { slide = { 0, 1, 0 }, min = -0.10, max = 0.08 } },
    { name = "steer_front_l", parent = "susp_front_l", motion = { turn = { 0, 1, 0 }, min = -35, max = 35 } },
    { name = "spin_front_l", parent = "steer_front_l", motion = { turn = { 1, 0, 0 } } },
    { name = "susp_front_r", motion = { slide = { 0, 1, 0 }, min = -0.10, max = 0.08 } },
    { name = "steer_front_r", parent = "susp_front_r", motion = { turn = { 0, 1, 0 }, min = -35, max = 35 } },
    { name = "spin_front_r", parent = "steer_front_r", motion = { turn = { 1, 0, 0 } } },
    { name = "susp_rear_l", motion = { slide = { 0, 1, 0 }, min = -0.10, max = 0.08 } },
    { name = "spin_rear_l", parent = "susp_rear_l", motion = { turn = { 1, 0, 0 } } },
    { name = "susp_rear_r", motion = { slide = { 0, 1, 0 }, min = -0.10, max = 0.08 } },
    { name = "spin_rear_r", parent = "susp_rear_r", motion = { turn = { 1, 0, 0 } } },
  },

  crane = {
    -- the cab turns on the mast without end; the boom lifts from level; the hook line runs out below the boom's tip.
    -- (The line runs along the boom's own "down", so it leans with the boom. A line that hangs plumb and swings
    -- needs a real joint: #311.)
    { name = "cabin", motion = { turn = { 0, 1, 0 } } },
    { name = "boom", parent = "cabin", motion = { turn = { -1, 0, 0 }, min = 0, max = 75 } },
    { name = "hook", parent = "boom", motion = { slide = { 0, -1, 0 }, min = 0, max = 6 } },
  },
}

-- Put a kind's joints on a model. `places` gives each joint the recipe has: { pos = {x,y,z} [, turn = {x,y,z}]
-- [, min =, max =] }, in its parent joint's space (a joint whose parent is another joint may be left out of
-- `places` and sits at its parent). Returns the joints by name, each a node of the model.
function rigs.apply(m, kind, places)
  local table_of = rigs.kinds[kind]
  if not table_of then error("rigs: no kind named '" .. tostring(kind) .. "'", 2) end
  local known = {}
  for _, joint in ipairs(table_of) do known[joint.name] = true end
  for name in pairs(places) do
    if not known[name] then error("rigs: the " .. kind .. " rig has no joint named '" .. tostring(name) .. "'", 2) end
  end
  local made = {}
  for _, joint in ipairs(table_of) do      -- the table's order: a parent always comes before what hangs under it
    local place = places[joint.name]
    local under = joint.parent and made[joint.parent]
    if place or under then                 -- given a place, or following a parent that was
      place = place or {}
      local motion = {}
      for k, v in pairs(joint.motion) do motion[k] = v end
      if place.min ~= nil then motion.min = place.min end
      if place.max ~= nil then motion.max = place.max end
      local opts = { pos = place.pos, turn = place.turn, motion = motion }
      made[joint.name] = (under or m):node(joint.name, opts)
    end
  end
  return made
end

return rigs
