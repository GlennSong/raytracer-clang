-- AN ARMCHAIR THAT RECLINES (#191): its back leans (the node `back`, 40 degrees) and its footrest swings up from
-- under the seat's front (the node `footrest`, 90 degrees). The lever on its right side is a marker, `lever`.
-- It stands on y = 0 at its own origin, facing +z.
local kit = require "model_kit"
local V = kit.V

local a = params{ fabric = kit.fabric_names }
local fabric = kit.fabric(a.fabric)
local m = model.new()
local far = m:level(25)

local k = kit.maker()
k.box("fabric", V(0, 0.2, -0.02), V(0.62, 0.26, 0.7), 0.03, fabric * 0.85, 2)        -- the box under the seat
k.cushion("fabric", V(0, 0.39, 0.02), V(0.6, 0.14, 0.62), fabric, { r = 0.07, crown = 0.02 })
for _, x in ipairs{ -0.4, 0.4 } do
  k.cushion("fabric", V(x, 0.33, 0.0), V(0.2, 0.62, 0.84), fabric * 0.92, { r = 0.08, roll = 0.06 })   -- the arms
  for _, z in ipairs{ -0.34, 0.34 } do k.box("hard", V(x, 0.015, z), V(0.06, 0.03, 0.06), 0.008, kit.BLACK, 1) end
end
k.rod("hard", V(0.5, 0.36, 0.1), V(0.54, 0.46, 0.14), 0.008, kit.BLACK, 8)             -- the lever
kit.put(m, m, k)
kit.far_box(m, m, "fabric", { 1.0, 0.62, 0.84 }, { 0, 0.33, 0 }, fabric, far)
m:shape("body", { box = { 1.0, 0.62, 0.84 }, pos = { 0, 0.31, 0 }, friction = 0.9 })
m:marker("seat", { pos = { 0, 0.46, 0.05 }, role = "seat", size = 0.55 })
m:marker("lever", { pos = { 0.54, 0.46, 0.14 }, role = "hold" })

local back = m:node("back", { pos = { 0, 0.4, -0.3 }, turn = { -8, 0, 0 }, motion = { turn = { -1, 0, 0 }, min = 0, max = 40 } })
local b = kit.maker()
b.cushion("fabric", V(0, 0.36, -0.03), V(0.6, 0.76, 0.18), fabric, { face = "z", r = 0.09, roll = 0.05, crown = 0.03 })
b.cushion("fabric", V(0, 0.64, 0.07), V(0.44, 0.2, 0.08), fabric * 1.08, { face = "z", r = 0.06, pinch = 0.3 })   -- the head pillow
kit.put(m, back, b)
kit.far_box(m, back, "fabric", { 0.6, 0.76, 0.18 }, { 0, 0.36, -0.03 }, fabric, far)
back:shape("back", { box = { 0.6, 0.76, 0.18 }, pos = { 0, 0.36, -0.03 } })

-- the footrest hangs under the seat's front edge and swings up and out to lie level
local rest = m:node("footrest", { pos = { 0, 0.33, 0.34 }, motion = { turn = { -1, 0, 0 }, min = 0, max = 90 } })
local f = kit.maker()
f.cushion("fabric", V(0, -0.16, 0.03), V(0.58, 0.3, 0.07), fabric, { face = "z", r = 0.05, crown = 0.015 })
kit.put(m, rest, f)
rest:shape("footrest", { box = { 0.58, 0.3, 0.07 }, pos = { 0, -0.16, 0.03 } })
return m
