-- THE CAR RIG LAB: an EXISTING kit car (vehicle_kit.lua, sedan by default) built as a model on the car rig (rigs.lua).
-- Nothing in the game reads this; it is the model sheet's view of what the car port will make.
-- Any kit car: --args '{"name":"pickup"}' (the names are vehicle_kit.lua SPECS).
--   * the body and the fixed cabin on the root;
--   * each of the kit's panels (doors, hood, trunk) on the rig's joint of the same name, placed at the kit's own
--     hinge point, its meshes moved into the hinge's space;
--   * ONE tire model (tyre + rim) used at four wheels, the right-hand ones flipped under their spin joints;
--   * the kit's lamps and the driver's hip point as markers; the kit's materials by name;
--   * three levels of detail: the full car, a simpler one from 40 m, one mesh from 150 m (--at 60, --at 200).
-- Try: rt_model_sheet assets/scripts/car_rig_lab.lua --set door_front_l=50 --set door_rear_r=40 --set hood=45 \
--        --set trunk=50 --set steer_front_l=25 --set steer_front_r=25
local kit = require "vehicle_kit"
local rigs = require "rigs"
local name = (args and args.name) or "sedan"
local level = (args and args.level) or 1
local P = kit.SPECS[name]()
local built = kit.build(P, level, { panels = true })
local lift = built.lift

local COLOR = { glass = { 0.60, 0.66, 0.70 }, trim = { 0.04, 0.04, 0.045 }, grille = { 0.06, 0.06, 0.065 },
                chrome = { 0.75, 0.76, 0.78 }, lamp = { 0.95, 0.95, 0.92 }, lamp_red = { 0.8, 0.05, 0.05 },
                seat = { 0.22, 0.20, 0.19 }, dash = { 0.08, 0.08, 0.09 }, carpet = { 0.10, 0.10, 0.11 },
                gasket = { 0.03, 0.03, 0.03 }, liner = { 0.30, 0.29, 0.28 }, steer = { 0.06, 0.06, 0.06 } }
local LOOK = { body = { roughness = 0.30, metallic = 0.55 }, glass = { roughness = 0.05 }, chrome = { roughness = 0.2, metallic = 0.9 } }

local car = model.new()
local function made_of(mat)                      -- the car's material of that name, made the first time it is asked for
  if not car:has_material(mat) then car:material(mat, material.new(LOOK[mat] or { roughness = 0.6 })) end
  return mat
end
local function painted(mat, m) return mesh.paint(m, mat == "body" and (built.color or { 0.5, 0.5, 0.5 }) or (COLOR[mat] or { 0.5, 0.5, 0.5 })) end
local function sorted(t) local k = {} for n in pairs(t or {}) do k[#k + 1] = n end table.sort(k) return k end   -- not pairs' order

-- the body and the fixed cabin
for _, mat in ipairs(sorted(built.parts)) do car:add(mesh.translate(painted(mat, built.parts[mat]), { 0, lift, 0 }), made_of(mat)) end
for _, mat in ipairs(sorted(built.cabin)) do car:add(mesh.translate(painted(mat, built.cabin[mat]), { 0, lift, 0 }), made_of(mat)) end

-- the joints: the kit's panels at the kit's hinges, the wheels at the kit's wheel places
local places = {}
for pname, pn in pairs(built.panels or {}) do
  local h = pn.hinge
  places[pname] = { pos = { h.pivot[1], h.pivot[2] + lift, h.pivot[3] }, max = h.open }   -- this car's own opening
end
local side = { [true] = "_l", [false] = "_r" }
for _, w in ipairs(built.wheels) do
  places["susp_" .. (w.front and "front" or "rear") .. side[w.pos[1] > 0]] = { pos = { w.pos[1], w.pos[2] + lift, w.pos[3] } }
end
local joint = rigs.apply(car, "car", places)

-- each panel's meshes, moved into its hinge's space
for _, pname in ipairs(sorted(built.panels)) do
  local pn = built.panels[pname]
  local to_hinge = { -pn.hinge.pivot[1], -pn.hinge.pivot[2], -pn.hinge.pivot[3] }
  for _, mat in ipairs(sorted(pn.parts)) do joint[pname]:add(mesh.translate(painted(mat, pn.parts[mat]), to_hinge), made_of(mat)) end
  for _, mat in ipairs(sorted(pn.cabin)) do joint[pname]:add(mesh.translate(painted(mat, pn.cabin[mat]), to_hinge), made_of(mat)) end
end

-- one tire, four wheels
local wheel = kit.wheel(P.wheel_r, P.wheel_w, level)
local tire = model.new()
tire:material("rubber", material.new{ roughness = 0.9 })
tire:material("alloy", material.new{ roughness = 0.25, metallic = 0.9 })
tire:add(mesh.paint(wheel.tyre, { 0.05, 0.05, 0.055 }), "rubber")
tire:add(mesh.paint(wheel.rim, { 0.75, 0.76, 0.78 }), "alloy")
for _, w in ipairs(built.wheels) do
  local left = w.pos[1] > 0
  local spin = joint["spin_" .. (w.front and "front" or "rear") .. side[left]]
  if left then spin:use(tire)
  else spin:node("tire_" .. (w.front and "front" or "rear") .. "_r", { turn = { 0, 180, 0 } }):use(tire) end
end

-- LEVELS OF DETAIL. The joints and markers serve every level.
--   * From 40 m: the same car, each mesh at 30% of its triangles. The tire has its own simpler level, so the four
--     wheels follow by themselves.
--   * From 150 m: simpler in STRUCTURE, not only in triangles, and made BY THE RECIPE: the kit's own low-poly build
--     of the same car (its level 0, no separate panels), the outside only, and a plain disc for each wheel. No
--     joints to pose.
--     (Two things were tried first and read badly. Crushing the whole car, interior and all, into one mesh: the
--     simplifier eats the small separate shells, the wheels first. Then the outside alone at 12%: the panels are
--     thin solids with back faces, and they crumple. `simplify` is good for a middle level, not for a far one.)
if not (args and args.levels == false) then
  tire:level(40, { simplify = 0.3 })
  local far_tire = tire:level(150)
  tire:add(mesh.paint(mesh.rotate_z(mesh.cylinder(P.wheel_r, P.wheel_w, 10), math.pi / 2), { 0.05, 0.05, 0.055 }), "rubber", { level = far_tire })

  car:level(40, { simplify = 0.3 })
  local far = car:level(150)
  local low = kit.build(kit.SPECS[name](), 0)
  for _, mat in ipairs{ "body", "glass", "lamp", "lamp_red", "grille" } do
    if low.parts[mat] then car:add(mesh.translate(painted(mat, low.parts[mat]), { 0, low.lift, 0 }), made_of(mat), { level = far }) end
  end
  local wheels = {}
  for _, w in ipairs(built.wheels) do wheels[#wheels + 1] = { pos = { w.pos[1], w.pos[2] + lift, w.pos[3] } } end
  car:use(tire, wheels, { level = far })
end

-- markers: the lamps and the driver's hip point
for _, l in ipairs(built.lights or {}) do
  local front = l.name:sub(1, 4) == "head"
  car:marker(l.name, { pos = { l.pos[1], l.pos[2] + lift, l.pos[3] }, turn = { 0, front and 0 or 180, 0 }, role = "lamp", size = 0.15 })
end
car:marker("driver", { pos = { built.driver_seat[1], built.driver_seat[2] + lift, built.driver_seat[3] }, role = "seat", size = 0.25 })
return car
