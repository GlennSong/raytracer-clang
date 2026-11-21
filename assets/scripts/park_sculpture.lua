-- A PARK SCULPTURE: a pierced, twisted form strung with wires, standing on a stone plinth low enough to sit on
-- (#191; Glenn, 2026-11-20: "some kind of sculpture that could be placed in a park as a piece of furniture").
-- The first model written in the model studio from nothing, to try the modelling tools together:
--
--   the plinth     a block with rounded edges (P:bevel), a plaque on its front held by four bolts (scatter.lua)
--   the form       a slab with a hole cut through it (openings.lua), made smooth (P:subdivide), then twisted,
--                  narrowed toward its top and leant (P:map)
--   the strings    wires across the hole, each end moved as the form was (cable.lua)
--   far away       the same slab subdivided once, from 60 m
--
-- The form is on a node that can turn about the plinth's middle; nothing turns it here. A prefab with the script
-- park_sculpture_turn.lua turns it slowly (prefabs.lua `park_sculpture_turning`). The plinth has a seat marker on
-- each side and the model stands on y = 0 at its own origin.
local openings = require "openings"
local scatter = require "scatter"
local cable = require "cable"

local a = params{
  finish  = { "bronze", "corten", "steel", "stone" },   -- what the form is made of
  height  = { 2.6, min = 1.5, max = 5 },                -- the form's height above the plinth (metres)
  twist   = { 50, min = -180, max = 180 },              -- how far its top is turned from its foot (degrees)
  taper   = { 0.3, min = 0, max = 0.7 },                -- how much narrower its top is (0: not at all)
  lean    = { 0.15, min = -0.5, max = 0.5 },            -- how far its top leans along its width (metres)
  strings = { 11, min = 0, max = 32 },                  -- how many wires cross the hole
}

local FINISH = {
  bronze = { color = { 0.42, 0.27, 0.13 }, roughness = 0.38, metallic = 0.95 },
  corten = { color = { 0.45, 0.2, 0.1 }, roughness = 0.85, metallic = 0.5 },
  steel  = { color = { 0.72, 0.73, 0.75 }, roughness = 0.18, metallic = 1.0 },
  stone  = { color = { 0.78, 0.76, 0.7 }, roughness = 0.8, metallic = 0.0 },
}
local finish = FINISH[a.finish]

local m = model.new()
m:material("plinth", material.new{ roughness = 0.9, metallic = 0.0 })
m:material("form", material.new{ roughness = finish.roughness, metallic = finish.metallic })
m:material("wire", material.new{ roughness = 0.3, metallic = 1.0 })
m:material("plaque", material.new{ roughness = 0.35, metallic = 0.9 })
local far = m:level(60)

local function solid(P, color, angle)
  local parts = {}
  for _, part in pairs(P:to_parts(angle or 30)) do parts[#parts + 1] = part end
  return mesh.paint(mesh.merge(parts), color)
end

-- THE PLINTH: 2.4 m square and 0.45 m tall (a seat's height), its edges rounded
local SIDE, SEAT = 2.4, 0.45
local granite = { 0.5, 0.5, 0.52 }
local plinth = poly.box{ SIDE, SEAT, SIDE }
plinth:translate{ 0, SEAT / 2, 0 }
plinth:bevel(0.05, 30, nil, 3)
m:add(solid(plinth, granite), "plinth")
m:add(mesh.paint(mesh.translate(mesh.box{ SIDE, SEAT, SIDE }, { 0, SEAT / 2, 0 }), granite), "plinth", { level = far })
m:shape("plinth", { box = { SIDE, SEAT, SIDE }, pos = { 0, SEAT / 2, 0 }, friction = 0.9 })
-- somewhere to sit on each side, facing out
for i, side in ipairs{ { 0, 1, 0 }, { 1, 0, 90 }, { 0, -1, 180 }, { -1, 0, 270 } } do
  m:marker("seat_" .. i, { pos = { side[1] * (SIDE / 2 - 0.3), SEAT, side[2] * (SIDE / 2 - 0.3) }, turn = { 0, side[3], 0 }, role = "seat", size = 0.5 })
end

-- THE PLAQUE on the front face, and the four bolts that hold it
local plaque = poly.box{ 0.5, 0.26, 0.012 }
plaque:translate{ 0, SEAT / 2, SIDE / 2 + 0.006 }
plaque:bevel(0.003)
m:add(solid(plaque, { 0.55, 0.42, 0.2 }), "plaque")
local bolt = model.new()
bolt:add(mesh.paint(mesh.translate(mesh.cylinder(0.012, 0.006, 6), { 0, 0.003, 0 }), { 0.8, 0.78, 0.7 }))
m:use(bolt, scatter.grid{ origin = { -0.25, SEAT / 2 - 0.13, SIDE / 2 + 0.012 }, u = { 0.5, 0, 0 }, v = { 0, 0.26, 0 }, count = { 2, 2 }, margin = 0.03 })
m:marker("plaque", { pos = { 0, SEAT / 2, SIDE / 2 + 0.02 } })

-- THE FORM, on a node of its own that can turn about the plinth's middle
local form = m:node("form", { pos = { 0, SEAT, 0 }, motion = { turn = { 0, 1, 0 } } })
local H, W, D = a.height, 1.5, 0.55
local FOOT = -0.12                           -- it is let into the plinth a little, so its rounded foot does not hover
local HOLE = { y = H * 0.56, w = W * 0.46, h = H * 0.42 }

-- the shape every point of it is moved by: narrower, turned and leant more the higher it is
local function shaped(x, y, z)
  local t = math.max(0, math.min(1, y / H))
  local narrow = 1 - a.taper * t
  x, z = x * narrow, z * (1 - 0.4 * a.taper * t)
  local turn = math.rad(a.twist) * t
  local c, s = math.cos(turn), math.sin(turn)
  return x * c + z * s + a.lean * t * t, y, -x * s + z * c
end

local function slab(levels)
  local P = poly.box{ W, H, D }
  P:translate{ 0, H / 2 + FOOT, 0 }
  openings.hole(P, { at = { 0, HOLE.y, D / 2 }, normal = { 0, 0, 1 }, width = HOLE.w, height = HOLE.h, thickness = D })
  P = P:subdivide(levels)
  P:map(shaped)
  return P
end
form:add(solid(slab(3), finish.color, 60), "form")
form:add(solid(slab(1), finish.color, 60), "form", { level = far })
form:shape("form", { box = { W * (1 - a.taper / 2), H, D }, pos = { a.lean / 3, H / 2, 0 }, turn = { 0, a.twist / 2, 0 } })

-- THE STRINGS: from the hole's lower edge to its upper edge, each crossing to the other side, so together they
-- make a surface that twists. Their ends are set a little into the form, so no wire hangs in the air.
if a.strings > 0 then
  local wires = {}
  local half, low, high = HOLE.w / 2 - 0.02, HOLE.y - HOLE.h / 2 + 0.03, HOLE.y + HOLE.h / 2 - 0.03
  for i = 1, a.strings do
    local f = a.strings == 1 and 0.5 or (i - 1) / (a.strings - 1)
    local from = { shaped(-half + 2 * half * f, low, -D * 0.22) }
    local to = { shaped(half - 2 * half * f, high, D * 0.22) }
    wires[#wires + 1] = cable.wire({ from, to }, { radius = 0.004, sides = 5, color = { 0.85, 0.82, 0.7 } })
  end
  form:add(mesh.merge(wires), "wire")
end
form:marker("crown", { pos = { shaped(0, H + FOOT, 0) }, turn = { -90, 0, 0 } })

-- where a light in the ground would stand to light it from the front
m:marker("uplight", { pos = { 0, 0, SIDE / 2 + 1.2 }, turn = { -60, 180, 0 }, role = "lamp" })
return m
