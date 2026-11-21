-- AN OFFICE CHAIR THAT ADJUSTS: the furniture kit's swivel chair (furniture_models.lua `office_chair`), built so
-- its parts move (#191; Glenn, 2026-11-20: "make it actually a rolling chair and have the lever adjust the height").
--
--   lift       the gas column: the seat and everything on it slides up, 12 cm
--   swivel     the seat turns on the column, without end
--   recline    the back leans, 20 degrees
--   caster_1 ... caster_5    each caster turns about its stem to trail behind the way the chair goes
--
-- Nothing here moves them: the studio's sliders do, or a prefab's script (swivel_chair_fidget.lua). The lever is a
-- marker, `lever`, for the pull that will raise and lower the seat when prefabs carry interactions. The chair does
-- not roll: a prefab is fixed or not solid for now, and rolling needs a body with weight (#311).
-- It stands on y = 0 at its own origin, facing +z.
local kit = require "model_kit"
local V = kit.V

local a = params{
  fabric = kit.fabric_names,                      -- the seat's and the back's cloth
  arms   = { "arms", "none" },                    -- arm rests, or none
}
local fabric, cloth = kit.upholstery(a.fabric)
fabric = fabric * 0.75
local BLACK, CHROME = kit.BLACK, kit.CHROME

local m = model.new()
local far = m:level(25)

-- THE BASE: a five-pointed star, a caster under each point, and the column's outer tube
local base = kit.maker()
for i = 0, 4 do
  local ang = 2 * math.pi * i / 5 + 0.3
  local c, s = math.cos(ang), math.sin(ang)
  base.bent("hard", { V(0, 0.10, 0), V(0.16 * c, 0.095, 0.16 * s), V(0.30 * c, 0.07, 0.30 * s) }, 0.03, 0.026, BLACK, { count = 6, scale = { 1.2, 0.8 } })
  local caster = m:node("caster_" .. (i + 1), { pos = { 0.30 * c, 0, 0.30 * s }, motion = { turn = { 0, 1, 0 } } })
  local k = kit.maker()
  kit.caster(k, V(0, 0, 0), 0.028)
  kit.put(m, caster, k)
end
base.turned("hard", V(0, 0, 0), { { 0.045, 0.075 }, { 0.045, 0.14 }, { 0.032, 0.155 }, { 0.032, 0.32 }, { 0.0, 0.32 } }, BLACK, 1, 16)
kit.put(m, m, base)
kit.far_box(m, m, "hard", { 0.6, 0.08, 0.6 }, { 0, 0.06, 0 }, BLACK, far)
m:shape("base", { box = { 0.62, 0.1, 0.62 }, pos = { 0, 0.05, 0 } })

-- THE LIFT: the chrome column, sliding in the outer tube
local SEAT = 0.44                           -- the height of the seat's mechanism with the column down
local lift = m:node("lift", { pos = { 0, 0, 0 }, motion = { slide = { 0, 1, 0 }, min = 0, max = 0.12 } })
local col = kit.maker()
col.turned("metal", V(0, 0, 0), { { 0.024, 0.16 }, { 0.024, SEAT - 0.02 }, { 0.03, SEAT }, { 0.0, SEAT } }, CHROME, 1, 16)
kit.put(m, lift, col)

-- THE SEAT, turning on the column. Its space: the origin is the top of the column.
local swivel = lift:node("swivel", { pos = { 0, SEAT, 0 }, motion = { turn = { 0, 1, 0 } } })
local seat = kit.maker()
seat.box("hard", V(0, 0.005, 0), V(0.30, 0.03, 0.30), 0.01, BLACK, 1)                 -- the mechanism
seat.cushion(cloth, V(0, 0.05, 0.02), V(0.50, 0.08, 0.48), fabric,
             { r = 0.08, roll = 0.03, crown = 0.012,
               warp = function(u, v, w)                                               -- dished, the front rolled down
                 return 0, -0.012 * math.max(0, 1 - u * u) * math.max(0, 1 - w * w) * (v > 0 and 1 or 0)
                           - 0.03 * math.max(0, w - 0.55) / 0.45 * (v + 1) * 0.5, 0
               end })
-- the lever: a rod out from under the right of the seat, a paddle on its end
seat.rod("hard", V(0.12, 0.0, 0.05), V(0.29, -0.01, 0.05), 0.006, BLACK, 8)
seat.box("hard", V(0.30, -0.01, 0.05), V(0.05, 0.012, 0.03), 0.005, BLACK, 1)
if a.arms == "arms" then
  for _, x in ipairs{ -0.27, 0.27 } do
    seat.bent("hard", { V(x * 0.7, 0.01, 0.0), V(x, 0.06, 0.0), V(x, 0.20, 0.0) }, 0.03, 0.02, BLACK, { count = 8 })
    seat.cushion("hard", V(x, 0.215, 0.02), V(0.075, 0.03, 0.25), BLACK, { r = 0.03, roll = 0.012, crown = 0.004 })
  end
end
kit.put(m, swivel, seat)
kit.far_box(m, swivel, cloth, { 0.5, 0.1, 0.48 }, { 0, 0.05, 0.02 }, fabric, far)
swivel:shape("seat", { box = { 0.5, 0.1, 0.48 }, pos = { 0, 0.05, 0.02 }, friction = 0.9 })
swivel:marker("seat", { pos = { 0, 0.09, 0.02 }, role = "seat", size = 0.48 })
swivel:marker("lever", { pos = { 0.30, -0.01, 0.05 }, role = "hold" })

-- THE BACK, leaning about a hinge at the rear of the seat. Its space: the origin is that hinge.
local HINGE = V(0, 0.03, -0.12)
local recline = swivel:node("recline", { pos = { HINGE[1], HINGE[2], HINGE[3] }, motion = { turn = { -1, 0, 0 }, min = 0, max = 20 } })
local back = kit.maker()
back.bent("hard", { V(0, -0.02, 0.02), V(0, 0.0, -0.10), V(0, 0.15, -0.13) }, 0.05, 0.016, BLACK, { count = 8 })    -- the spine
back.cushion(cloth, V(0, 0.33, -0.09), V(0.46, 0.54, 0.07), fabric,
             { face = "z", r = 0.09, roll = 0.025, crown = 0.01, bow = 0.045, tilt = -0.18,
               warp = function(u, v, w) return 0, 0.02 * math.exp(-((w - 0.45) / 0.3) ^ 2), 0 end })   -- the lumbar swell
kit.put(m, recline, back)
kit.far_box(m, recline, cloth, { 0.46, 0.54, 0.07 }, { 0, 0.33, -0.09 }, fabric, far)
recline:shape("back", { box = { 0.46, 0.54, 0.08 }, pos = { 0, 0.33, -0.09 }, turn = { -10, 0, 0 } })
return m
